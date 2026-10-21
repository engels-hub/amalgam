//! Parameters (S01 §5): the [`ParamTable`] in [`Design`], model parameters created from
//! feature expressions, cycle detection, rename rewriting and evaluation.
//!
//! - Every parameter has an expression ([`ExprSrc`]) and an optional unit. With a unit, the
//!   value must have that unit's dimension and a unitless expression (`10`) is read in it
//!   (Fusion behaviour). Without one, any dimension is accepted.
//! - References bind by name when an expression is set; the table keeps the resolved
//!   [`ParamId`]s, rejects cycles ([`ExprError::Cycle`]) and evaluates in topological order
//!   (Kahn), re-evaluating only the edited parameter and its dependents.
//! - Values are computed on the thread that edits (cheap: 1 000 chained parameters take well
//!   under 2 ms, `benches/params.rs`). A bad expression is rejected by the edit and the
//!   design is left unchanged.
//! - On disk only the parameters themselves are stored; names, parsed expressions and
//!   values are rebuilt on load, and a design whose parameters don't evaluate doesn't load.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::ops::Range;
use std::sync::Arc;

use amg_core::{Dimension, Quantity, Unit};
use serde::{Deserialize, Serialize};

use crate::expr::{self, Ast, Dim, EvalOptions, ExprError};
use crate::id::{FeatureId, ParamId};
use crate::model::{Design, ModelError, UnitPrefs};
use crate::pcoll::{PMap, entries};

/// Unit of a parameter (S01 Interfaces `UnitSpec`): one unit of the amg-core table.
pub type UnitSpec = Unit;

/// An expression field of a feature. After `AddFeature` features only hold
/// [`Expr::Param`]: every inline expression becomes a model parameter
/// ([`Design::add_model_params`]).
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Expr {
    /// Source text not yet turned into a parameter.
    Inline(ExprSrc),
    /// A parameter of the design.
    Param(ParamId),
}

/// Expression source text as the user typed it.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ExprSrc(pub String);

impl ExprSrc {
    /// The text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<&str> for ExprSrc {
    fn from(s: &str) -> ExprSrc {
        ExprSrc(s.to_owned())
    }
}

/// Whether a parameter was created by the user or for a feature dimension.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ParamKind {
    /// Created in the Parameters dialog.
    User,
    /// Created for an expression field of `owner` (Fusion `d<n>`).
    Model {
        /// The feature whose field this is.
        owner: FeatureId,
    },
}

/// One parameter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Param {
    /// Unique name; an identifier that is not a reserved unit/function/constant name.
    pub name: String,
    /// Expression source.
    pub expr: ExprSrc,
    /// User or model parameter.
    pub kind: ParamKind,
    /// Display unit and required dimension (`None`: any dimension, shown canonically).
    pub unit: Option<UnitSpec>,
    /// Free text.
    pub comment: String,
    /// Starred in the Parameters dialog.
    pub favorite: bool,
}

impl Param {
    fn options(&self, prefs: &UnitPrefs) -> EvalOptions {
        let o = EvalOptions::new(prefs);
        match self.unit {
            Some(u) => o.unit(u),
            None => o,
        }
    }
}

/// Why a parameter edit was rejected. The command layer (DOC-05) maps this into
/// `CommandError`.
#[derive(thiserror::Error, Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum ParamError {
    /// The expression or name is invalid.
    #[error(transparent)]
    Expr(#[from] ExprError),
    /// Another parameter has this name.
    #[error("name `{0}` already in use")]
    NameTaken(String),
    /// No such parameter.
    #[error("no such parameter {0}")]
    NotFound(ParamId),
    /// Other parameters still reference it.
    #[error("{id} is used by {by:?}")]
    InUse {
        /// The parameter.
        id: ParamId,
        /// Parameters whose expressions reference it.
        by: Vec<ParamId>,
    },
    /// A model-level error (ids exhausted, bad owner).
    #[error(transparent)]
    Model(#[from] ModelError),
}

impl amg_core::Diagnostic for ParamError {
    fn hint(&self) -> Option<String> {
        match self {
            ParamError::Expr(e) => amg_core::Diagnostic::hint(e),
            ParamError::NameTaken(_) => Some("Choose a different name.".to_owned()),
            ParamError::InUse { .. } => {
                Some("Change the expressions that use it first.".to_owned())
            }
            ParamError::NotFound(_) | ParamError::Model(_) => None,
        }
    }
}

/// A parsed expression with its references resolved to ids.
#[derive(Debug)]
struct Compiled {
    ast: Ast,
    /// Referenced parameters, sorted, without duplicates.
    refs: Vec<ParamId>,
}

/// All parameters of a design, with their evaluated values.
///
/// `clone()` is O(1) (persistent maps). Equality compares the stored parameters only.
#[derive(Clone, Debug, Default)]
pub struct ParamTable {
    params: PMap<ParamId, Arc<Param>>,
    // Runtime caches, rebuilt on load; never serialized.
    names: PMap<String, ParamId>,
    compiled: PMap<ParamId, Arc<Compiled>>,
    values: PMap<ParamId, Quantity>,
}

impl PartialEq for ParamTable {
    fn eq(&self, other: &ParamTable) -> bool {
        self.params == other.params
    }
}

impl Serialize for ParamTable {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        entries::serialize(&self.params, s)
    }
}

impl<'de> Deserialize<'de> for ParamTable {
    /// Loads the parameters only; [`Design`] rebuilds the caches and validates on load.
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<ParamTable, D::Error> {
        Ok(ParamTable {
            params: entries::deserialize(d)?,
            ..ParamTable::default()
        })
    }
}

impl ParamTable {
    /// The parameter `id`.
    pub fn get(&self, id: ParamId) -> Option<&Param> {
        self.params.get(&id).map(|p| &**p)
    }

    /// The parameter called `name`.
    pub fn by_name(&self, name: &str) -> Option<ParamId> {
        self.names.get(name).copied()
    }

    /// Every parameter in id order.
    pub fn iter(&self) -> impl Iterator<Item = (ParamId, &Param)> + '_ {
        self.params.iter().map(|(&id, p)| (id, &**p))
    }

    /// Number of parameters.
    pub fn len(&self) -> usize {
        self.params.len()
    }

    /// True if there are no parameters.
    pub fn is_empty(&self) -> bool {
        self.params.is_empty()
    }

    /// The evaluated value of `id` (canonical units). Always present for an existing
    /// parameter: edits that would leave a parameter unevaluable are rejected.
    pub fn value(&self, id: ParamId) -> Option<Quantity> {
        self.values.get(&id).copied()
    }

    /// Parameters referenced by `id`'s expression (sorted).
    pub fn references(&self, id: ParamId) -> &[ParamId] {
        self.compiled.get(&id).map_or(&[], |c| &c.refs)
    }

    /// Parameters whose expressions reference `id` directly (sorted).
    pub fn dependents(&self, id: ParamId) -> Vec<ParamId> {
        self.compiled
            .iter()
            .filter(|(_, c)| c.refs.binary_search(&id).is_ok())
            .map(|(&p, _)| p)
            .collect()
    }

    /// Parses and evaluates `src` against the current values without changing anything
    /// (dialog validation, previews). `expected` is the field dimension; a unitless whole
    /// expression takes its default unit ([`EvalOptions::default_unit`]).
    pub fn try_eval(
        &self,
        src: &str,
        expected: Option<Dim>,
        prefs: &UnitPrefs,
    ) -> Result<Quantity, ExprError> {
        let ast = expr::parse(src)?;
        let mut opts = EvalOptions::new(prefs);
        opts.expected = expected;
        expr::eval(&ast, &|n| self.lookup(n), &opts)
    }

    /// The value of a feature expression field: a parameter's value, or an inline
    /// expression evaluated like [`try_eval`](Self::try_eval). Checks `expected`.
    pub fn resolve(
        &self,
        e: &Expr,
        expected: Dim,
        prefs: &UnitPrefs,
    ) -> Result<Quantity, ExprError> {
        match e {
            Expr::Inline(src) => self.try_eval(src.as_str(), Some(expected), prefs),
            Expr::Param(id) => {
                let q = self.value(*id).ok_or_else(|| ExprError::UnknownParam {
                    name: id.to_string(),
                    span: 0..0,
                })?;
                if q.dim == expected {
                    Ok(q)
                } else {
                    Err(ExprError::WrongDimension {
                        expected,
                        got: q.dim,
                    })
                }
            }
        }
    }

    fn lookup(&self, name: &str) -> Option<Quantity> {
        self.names
            .get(name)
            .and_then(|id| self.values.get(id))
            .copied()
    }

    // ---- building blocks (operate on a scratch clone; the caller commits) -------------

    /// Parses `src` and binds its references to ids.
    fn compile(&self, src: &str) -> Result<Compiled, ExprError> {
        let ast = expr::parse(src)?;
        let mut refs = Vec::new();
        for (name, span) in expr::references(&ast) {
            match self.names.get(&name) {
                Some(&id) => refs.push(id),
                None => return Err(ExprError::UnknownParam { name, span }),
            }
        }
        refs.sort_unstable();
        refs.dedup();
        Ok(Compiled { ast, refs })
    }

    /// `Err(Cycle)` if `id` is reachable from `refs` (iterative DFS). The path starts and
    /// ends at `id`: `[a, b, a]`.
    fn check_cycle(&self, id: ParamId, refs: &[ParamId]) -> Result<(), ExprError> {
        let mut seen = BTreeSet::new();
        for &start in refs {
            // (node, index of the next child to visit); the stack is the current path.
            let mut stack: Vec<(ParamId, usize)> = vec![(start, 0)];
            while let Some(&(node, idx)) = stack.last() {
                if idx == 0 {
                    if node == id {
                        let mut path = vec![id];
                        path.extend(stack.iter().map(|&(p, _)| p));
                        return Err(ExprError::Cycle(path));
                    }
                    if !seen.insert(node) {
                        stack.pop();
                        continue;
                    }
                }
                match self.references(node).get(idx).copied() {
                    Some(child) => {
                        if let Some(top) = stack.last_mut() {
                            top.1 += 1;
                        }
                        stack.push((child, 0));
                    }
                    None => {
                        stack.pop();
                    }
                }
            }
        }
        Ok(())
    }

    /// Validates `name` for parameter `id` (unique among the others).
    fn check_name(&self, name: &str, id: ParamId) -> Result<(), ParamError> {
        expr::check_name(name)?;
        match self.names.get(name) {
            Some(&other) if other != id => Err(ParamError::NameTaken(name.to_owned())),
            _ => Ok(()),
        }
    }

    fn eval_one(&self, id: ParamId, prefs: &UnitPrefs) -> Result<Quantity, ExprError> {
        let p = &self.params[&id];
        let c = &self.compiled[&id];
        expr::eval(&c.ast, &|n| self.lookup(n), &p.options(prefs))
    }

    /// Re-evaluates `roots` and everything that depends on them, in topological order.
    fn reevaluate(&mut self, roots: &[ParamId], prefs: &UnitPrefs) -> Result<(), ParamError> {
        let mut rdeps: BTreeMap<ParamId, Vec<ParamId>> = BTreeMap::new();
        for (&p, c) in &self.compiled {
            for &r in &c.refs {
                rdeps.entry(r).or_default().push(p);
            }
        }
        let mut affected = BTreeSet::new();
        let mut stack: Vec<ParamId> = roots.to_vec();
        while let Some(p) = stack.pop() {
            if affected.insert(p) {
                stack.extend(rdeps.get(&p).into_iter().flatten().copied());
            }
        }
        // Kahn over the affected sub-graph.
        let mut indeg: BTreeMap<ParamId, usize> = affected
            .iter()
            .map(|&p| {
                let n = self
                    .references(p)
                    .iter()
                    .filter(|r| affected.contains(r))
                    .count();
                (p, n)
            })
            .collect();
        let mut ready: VecDeque<ParamId> = indeg
            .iter()
            .filter(|(_, n)| **n == 0)
            .map(|(&p, _)| p)
            .collect();
        let mut done = 0;
        while let Some(p) = ready.pop_front() {
            done += 1;
            let q = self.eval_one(p, prefs)?;
            self.values.insert(p, q);
            for &d in rdeps.get(&p).into_iter().flatten() {
                if let Some(n) = indeg.get_mut(&d) {
                    *n -= 1;
                    if *n == 0 {
                        ready.push_back(d);
                    }
                }
            }
        }
        if done != affected.len() {
            // Only a loaded file can get here: the edits reject cycles first. Report a
            // parameter that is on a cycle.
            for (&p, _) in indeg.iter().filter(|(_, n)| **n > 0) {
                let refs = self.references(p).to_vec();
                self.check_cycle(p, &refs)?;
            }
            return Err(ExprError::Cycle(Vec::new()).into());
        }
        Ok(())
    }

    /// Rebuilds names, compiled expressions and values from the stored parameters.
    fn rebuild(&mut self, prefs: &UnitPrefs) -> Result<(), ParamError> {
        self.names = PMap::new();
        self.compiled = PMap::new();
        self.values = PMap::new();
        for (&id, p) in &self.params {
            check_valid_name(&p.name)?;
            if self.names.insert(p.name.clone(), id).is_some() {
                return Err(ParamError::NameTaken(p.name.clone()));
            }
        }
        let ids: Vec<ParamId> = self.params.keys().copied().collect();
        for &id in &ids {
            let c = self.compile(self.params[&id].expr.as_str())?;
            self.compiled.insert(id, Arc::new(c));
        }
        self.reevaluate(&ids, prefs)
    }

    /// Inserts or replaces `id` with a compiled expression; checks cycles and re-evaluates.
    fn put(&mut self, id: ParamId, param: Param, prefs: &UnitPrefs) -> Result<(), ParamError> {
        if let Some(old) = self.params.get(&id)
            && old.name != param.name
        {
            self.names.remove(&old.name);
        }
        self.check_name(&param.name, id)?;
        self.names.insert(param.name.clone(), id);
        let c = self.compile(param.expr.as_str())?;
        self.check_cycle(id, &c.refs)?;
        self.params.insert(id, Arc::new(param));
        self.compiled.insert(id, Arc::new(c));
        self.reevaluate(&[id], prefs)
    }

    /// Smallest `d<n>` above every existing `d<digits>` name.
    fn next_model_name(&self) -> String {
        let max = self
            .names
            .keys()
            .filter_map(|n| n.strip_prefix('d'))
            .filter(|s| !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit()))
            .filter_map(|s| s.parse::<u64>().ok())
            .max()
            .unwrap_or(0);
        format!("d{}", max.saturating_add(1))
    }

    /// Checks a loaded or edited table: names, expressions, cycles and values all agree
    /// with a fresh rebuild. `owner_kind(id)` says which entity type owns a raw id.
    pub(crate) fn check(
        &self,
        prefs: &UnitPrefs,
        owner_kind: impl Fn(u64) -> Option<&'static str>,
    ) -> Result<(), String> {
        let mut fresh = self.clone();
        fresh
            .rebuild(prefs)
            .map_err(|e| format!("parameters: {e}"))?;
        for (id, p) in self.iter() {
            let (Some(a), Some(b)) = (self.value(id), fresh.value(id)) else {
                return Err(format!("{id} has no value"));
            };
            if a.dim != b.dim || a.value.to_bits() != b.value.to_bits() {
                return Err(format!("{id} has a stale value"));
            }
            if let ParamKind::Model { owner } = p.kind
                && let Some(o) = owner_kind(owner.0).filter(|&o| o != "feature")
            {
                return Err(format!("{id} is owned by {owner}, which is a {o}"));
            }
        }
        if self.names != fresh.names {
            return Err("parameter name index is stale".to_owned());
        }
        Ok(())
    }

    /// Ids of all parameters.
    pub(crate) fn ids(&self) -> impl Iterator<Item = ParamId> + '_ {
        self.params.keys().copied()
    }

    /// Rebuilds the runtime caches after deserialization.
    pub(crate) fn loaded(mut self, prefs: &UnitPrefs) -> Result<ParamTable, ModelError> {
        self.rebuild(prefs)
            .map_err(|e| ModelError::Inconsistent(format!("parameters: {e}")))?;
        Ok(self)
    }
}

fn check_valid_name(name: &str) -> Result<(), ExprError> {
    expr::check_name(name)
}

/// Replaces the spans of references to `old` in `src` with `new`.
fn rewrite(src: &str, ast: &Ast, old: &str, new: &str) -> String {
    let mut out = src.to_owned();
    let mut spans: Vec<Range<usize>> = expr::references(ast)
        .into_iter()
        .filter(|(n, _)| n == old)
        .map(|(_, s)| s)
        .collect();
    spans.sort_by_key(|s| std::cmp::Reverse(s.start));
    for s in spans {
        out.replace_range(s, new);
    }
    out
}

// ---- edits on Design --------------------------------------------------------------------
//
// Same contract as the other low-level edits: validate on a scratch copy, then commit and
// bump the revision; a failed edit changes nothing, a no-op edit doesn't bump.

impl Design {
    /// The parameter table.
    pub fn params(&self) -> &ParamTable {
        &self.params
    }

    /// Parses and evaluates `src` against the parameters with the document's default units
    /// (see [`ParamTable::try_eval`]).
    pub fn try_eval(&self, src: &str, expected: Option<Dim>) -> Result<Quantity, ExprError> {
        self.params
            .try_eval(src, expected, &self.meta().default_units)
    }

    /// Adds a user parameter.
    pub fn add_param(
        &mut self,
        name: &str,
        expr: &str,
        unit: Option<UnitSpec>,
        comment: &str,
    ) -> Result<ParamId, ParamError> {
        self.ensure_ids(1)?;
        let id = ParamId(self.meta().next_id);
        let mut t = self.params.clone();
        t.put(
            id,
            Param {
                name: name.to_owned(),
                expr: ExprSrc(expr.to_owned()),
                kind: ParamKind::User,
                unit,
                comment: comment.to_owned(),
                favorite: false,
            },
            &self.meta().default_units,
        )?;
        let allocated = self.alloc()?;
        debug_assert_eq!(allocated, id.0);
        self.params = t;
        self.bump();
        Ok(id)
    }

    /// Replaces the expression of `id`. Rejects syntax errors, unknown names, unit errors
    /// and cycles (`ExprError::Cycle` with the path).
    pub fn set_param_expr(&mut self, id: ParamId, expr: &str) -> Result<(), ParamError> {
        let p = self.param_or_err(id)?;
        if p.expr.as_str() == expr {
            return Ok(());
        }
        let mut t = self.params.clone();
        t.put(
            id,
            Param {
                expr: ExprSrc(expr.to_owned()),
                ..p
            },
            &self.meta().default_units,
        )?;
        self.params = t;
        self.bump();
        Ok(())
    }

    /// Renames `id` and rewrites every referencing expression through its token spans, so
    /// sources stay as the user wrote them apart from the name.
    pub fn rename_param(&mut self, id: ParamId, name: &str) -> Result<(), ParamError> {
        let p = self.param_or_err(id)?;
        if p.name == name {
            return Ok(());
        }
        let mut t = self.params.clone();
        t.check_name(name, id)?;
        let old = p.name.clone();
        t.names.remove(&old);
        t.names.insert(name.to_owned(), id);
        t.params.insert(
            id,
            Arc::new(Param {
                name: name.to_owned(),
                ..p
            }),
        );
        for d in self.params.dependents(id) {
            let c = &self.params.compiled[&d];
            let mut dp = self.params.params[&d].as_ref().clone();
            dp.expr = ExprSrc(rewrite(dp.expr.as_str(), &c.ast, &old, name));
            let nc = t.compile(dp.expr.as_str())?;
            if nc.refs != c.refs {
                return Err(ExprError::Parse {
                    span: 0..dp.expr.0.len(),
                    msg: format!("renaming `{old}` changed the meaning of `{}`", dp.expr.0),
                }
                .into());
            }
            t.params.insert(d, Arc::new(dp));
            t.compiled.insert(d, Arc::new(nc));
        }
        self.params = t;
        self.bump();
        Ok(())
    }

    /// Changes the comment and/or favourite flag (`None` keeps the current value).
    pub fn set_param_meta(
        &mut self,
        id: ParamId,
        comment: Option<String>,
        favorite: Option<bool>,
    ) -> Result<(), ParamError> {
        let mut p = self.param_or_err(id)?;
        let before = p.clone();
        if let Some(c) = comment {
            p.comment = c;
        }
        if let Some(f) = favorite {
            p.favorite = f;
        }
        if p == before {
            return Ok(());
        }
        self.params.params.insert(id, Arc::new(p));
        self.bump();
        Ok(())
    }

    /// Deletes `id`. Refused while other parameters reference it. The command layer
    /// (DOC-05) also checks feature references and converts or deletes model parameters
    /// with their feature.
    pub fn delete_param(&mut self, id: ParamId) -> Result<(), ParamError> {
        let p = self.param_or_err(id)?;
        let by = self.params.dependents(id);
        if !by.is_empty() {
            return Err(ParamError::InUse { id, by });
        }
        let t = &mut self.params;
        t.params.remove(&id);
        t.names.remove(&p.name);
        t.compiled.remove(&id);
        t.values.remove(&id);
        self.bump();
        Ok(())
    }

    /// Turns every [`Expr::Inline`] of a new feature into a model parameter `d<n>` owned by
    /// `owner` and rewrites the field to [`Expr::Param`] (Fusion behaviour, S01 §5). Each
    /// field comes with its dimension; the parameter unit is the document default for it
    /// (length unit, degrees, else canonical). [`Expr::Param`] fields must exist.
    ///
    /// All or nothing: on error no parameter is created and no field is changed.
    pub fn add_model_params(
        &mut self,
        owner: FeatureId,
        exprs: Vec<(&mut Expr, Dim)>,
    ) -> Result<Vec<ParamId>, ParamError> {
        let raw = owner.0;
        if raw == 0 || raw >= self.meta().next_id {
            return Err(ModelError::NotFound {
                what: "feature",
                id: raw,
            }
            .into());
        }
        if let Some(o) = self.owner_of(raw).filter(|&o| o != "feature") {
            return Err(ModelError::IdInUse {
                id: raw,
                owner: o,
                wanted: "feature",
            }
            .into());
        }
        let n_new = exprs
            .iter()
            .filter(|(e, _)| matches!(e, Expr::Inline(_)))
            .count();
        let n_new = u64::try_from(n_new).map_err(|_| ModelError::IdsExhausted)?;
        self.ensure_ids(n_new)?;
        let prefs = self.meta().default_units.clone();
        let mut t = self.params.clone();
        let mut next = self.meta().next_id;
        let mut created = Vec::new();
        for (e, dim) in &exprs {
            match e {
                Expr::Param(id) => {
                    let q = t.value(*id).ok_or(ParamError::NotFound(*id))?;
                    if q.dim != *dim {
                        return Err(ExprError::WrongDimension {
                            expected: *dim,
                            got: q.dim,
                        }
                        .into());
                    }
                }
                Expr::Inline(src) => {
                    let id = ParamId(next);
                    next += 1;
                    let unit = EvalOptions::new(&prefs).default_unit(*dim);
                    let p = Param {
                        name: t.next_model_name(),
                        expr: src.clone(),
                        kind: ParamKind::Model { owner },
                        unit,
                        comment: String::new(),
                        favorite: false,
                    };
                    if unit.is_none() && *dim != Dimension::NONE {
                        // No table unit for this dimension: check the dimension by hand.
                        let q = t.try_eval(src.as_str(), Some(*dim), &prefs)?;
                        debug_assert_eq!(q.dim, *dim);
                    }
                    t.put(id, p, &prefs)?;
                    created.push(id);
                }
            }
        }
        for _ in &created {
            self.alloc()?;
        }
        let mut ids = created.iter();
        for (e, _) in exprs {
            if matches!(e, Expr::Inline(_))
                && let Some(&id) = ids.next()
            {
                *e = Expr::Param(id);
            }
        }
        if !created.is_empty() {
            self.params = t;
            self.bump();
        }
        Ok(created)
    }

    fn param_or_err(&self, id: ParamId) -> Result<Param, ParamError> {
        self.params.get(id).cloned().ok_or(ParamError::NotFound(id))
    }
}

#[cfg(test)]
mod tests;
