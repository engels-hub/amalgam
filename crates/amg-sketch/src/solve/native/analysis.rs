//! DOF analysis and diagnosis (SK-04, note §3.3–3.4).
//!
//! **Rank and per-entity status** run on the substituted, decomposed system of the cached
//! [`Structure`]. Per component: if every over-determined piece has full column rank and every
//! square block is nonsingular (dense rank-revealing QR, [`dense::null_space`]), their unknowns
//! are pinned and the null space is the direct sum of the under-determined pieces' null
//! spaces; otherwise the whole component is factorised. Unknown classes in no component are
//! free. An entity's `free_dof` is the rank of the orthonormal null-basis rows of its
//! parameters (its own and its points'): the number of independent ways it can move.
//! Blocks above [`RANK_DENSE_MAX`] unknowns or rows use their structural rank (leftover:
//! sparse rank-revealing QR).
//!
//! **Diagnosis** runs only when something is deficient (rank below rows, substitution
//! redundancy, a double `Fix`, unsatisfied rows) and only on the raw components concerned:
//! circuits on the unsubstituted Jacobian ([`diagnose`]), then
//!
//! - **witness tie-break**: a *witness configuration* is the sketch with every driving
//!   dimension made driven, its free parameters perturbed deterministically and re-solved, so
//!   it satisfies every geometric constraint at generic dimension values (dimension rows have
//!   value-independent Jacobians). A satisfied circuit that is still dependent at the witness
//!   is `Redundant`; one that becomes independent is `DegenerateConfiguration` (it only
//!   depends at this configuration). DOF and entity states come from the witness when its
//!   rank is higher (the generic rank is the maximum over the configuration manifold);
//! - **leave-one-out refinement** of conflicts: in a raw component that stays unsatisfied,
//!   the `Conflicting` group is the set of constraints whose removal alone makes the component
//!   solvable (SolveSpace's "remove one of these"). If no single removal helps (several
//!   independent conflicts) or the component has more than [`LOO_MAX`] constraints, the
//!   unsatisfied circuits are reported instead.
//!
//! Entities referenced by a `Redundant` or `Conflicting` group's constraints are `Over`.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use smallvec::SmallVec;

use super::block::{self, to_dense};
use super::dense::{null_space, rank_abs};
use super::diagnose::{RawSystem, RawValues, RowSrc, circuits};
use super::structure::{Slot, Structure, Sub, SubKind};
use super::{NativeSolver, RANK_DENSE_MAX, Workspace, effective_tol};
use crate::error::SketchError;
use crate::ids::{ConstraintId, EntityId, ParamId};
use crate::model::{ConstraintKind, EntityKind, Sketch};
use crate::solve::api::{
    ConstraintCheck, ConstraintState, DependencyGroup, Diagnosis, DofReport, EntityStatus,
    GroupKind, SolveOptions, SolverBackend,
};
use crate::solve::residual::{SketchSystem, constraint_residual};
use crate::solve::system::{EquationOwner, EquationSystem, Residual};

/// Conflicting components with more removable constraints than this skip leave-one-out.
pub(crate) const LOO_MAX: usize = 48;
/// Absolute threshold on rows of an orthonormal null basis (per-entity rank).
const NULL_TAU: f64 = 1e-8;
/// Relative dependency threshold of a candidate row against the null basis (as the
/// diagnosis' `DEP_REL`).
const CAND_REL: f64 = 1e-8;
/// Witness perturbation: point coordinates by this times `L_c`, radii by this relative,
/// other parameters (curve parameters, weights, `ρ`) by a tenth of it absolute.
const WITNESS_STEP: f64 = 1e-2;

/// Null space of one piece of a component.
#[derive(Clone, Debug)]
enum Basis {
    /// Orthonormal basis, row-major `classes.len() × d`.
    Dense { d: usize, q: Vec<f64> },
    /// Too large for the dense path: `d` structurally free directions.
    Structural { d: usize },
}

#[derive(Clone, Debug)]
struct Piece {
    /// Unknown classes, ascending (row `i` of the basis is `classes[i]`).
    classes: Vec<u32>,
    basis: Basis,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ClassAt {
    /// In no component: moves freely.
    Free,
    /// Determined (null-space component zero).
    Pinned,
    /// Row `row` of piece `piece`.
    In { piece: u32, row: u32 },
}

/// Rank and null space of the substituted system at one configuration.
#[derive(Clone, Debug)]
pub(crate) struct SubAnalysis {
    pub rank: usize,
    /// Per component: numeric rank below its row count.
    pub deficient: Vec<bool>,
    pieces: Vec<Piece>,
    class_at: Vec<ClassAt>,
}

fn sub_jacobian(sys: &EquationSystem, sub: &Sub, x: &[f64], ws: &mut Workspace) -> Vec<f64> {
    let mut vals = vec![0.0; sub.nnz()];
    block::jacobian(sys, sub, x, &mut vals, ws);
    to_dense(sub, &vals)
}

fn too_big(sub: &Sub) -> bool {
    sub.n() > RANK_DENSE_MAX || sub.rows > RANK_DENSE_MAX
}

fn structural_rank(sub: &Sub) -> usize {
    match sub.kind {
        SubKind::Over | SubKind::Square => sub.n(),
        SubKind::Under => sub.rows,
        SubKind::Whole => sub.rows.min(sub.n()),
    }
}

fn piece_of(sys: &EquationSystem, sub: &Sub, x: &[f64], ws: &mut Workspace) -> (usize, Piece) {
    let (r, basis) = if too_big(sub) {
        let r = structural_rank(sub);
        (r, Basis::Structural { d: sub.n() - r })
    } else {
        let (r, q) = null_space(&sub_jacobian(sys, sub, x, ws), sub.rows, sub.n(), true);
        (r, Basis::Dense { d: sub.n() - r, q })
    };
    (
        r,
        Piece {
            classes: sub.unknowns.clone(),
            basis,
        },
    )
}

/// Rank and null space of the substituted system at `x`; `None` if cancelled.
pub(crate) fn substituted(
    sys: &EquationSystem,
    st: &Structure,
    x: &[f64],
    cancel: &dyn Fn() -> bool,
) -> Option<SubAnalysis> {
    let mut ws = Workspace::default();
    let mut class_at = vec![ClassAt::Free; st.unknowns()];
    let mut pieces = Vec::new();
    let mut deficient = Vec::with_capacity(st.components.len());
    let mut rank = 0;
    for comp in &st.components {
        if cancel() {
            return None;
        }
        for &k in &comp.unknowns {
            class_at[k as usize] = ClassAt::Pinned;
        }
        let mut split_ok = comp.blocks.len() > 1;
        let mut r_comp = 0;
        let mut unders = Vec::new();
        if split_ok {
            for b in &comp.blocks {
                match b.kind {
                    SubKind::Over | SubKind::Square => {
                        let r = if too_big(b) {
                            structural_rank(b)
                        } else {
                            null_space(&sub_jacobian(sys, b, x, &mut ws), b.rows, b.n(), false).0
                        };
                        split_ok &= r == b.n();
                        r_comp += r;
                    }
                    SubKind::Under => unders.push(b),
                    SubKind::Whole => split_ok = false,
                }
                if !split_ok {
                    break;
                }
            }
        }
        let mut new_pieces = Vec::new();
        if split_ok {
            for b in unders {
                if cancel() {
                    return None;
                }
                let (r, p) = piece_of(sys, b, x, &mut ws);
                r_comp += r;
                new_pieces.push(p);
            }
        } else {
            let (r, p) = piece_of(sys, comp.whole(), x, &mut ws);
            r_comp = r;
            new_pieces.push(p);
        }
        for p in new_pieces {
            let pi = pieces.len() as u32;
            for (row, &k) in p.classes.iter().enumerate() {
                class_at[k as usize] = ClassAt::In {
                    piece: pi,
                    row: row as u32,
                };
            }
            pieces.push(p);
        }
        deficient.push(r_comp < comp.rows);
        rank += r_comp;
    }
    Some(SubAnalysis {
        rank,
        deficient,
        pieces,
        class_at,
    })
}

/// Parameters that move an entity: its own and its points'.
fn entity_params(sketch: &Sketch, kind: &EntityKind) -> SmallVec<[ParamId; 8]> {
    let mut ps: SmallVec<[ParamId; 8]> = kind.own_params().into_iter().collect();
    for child in kind.children() {
        if let Some(c) = sketch.entity(child) {
            ps.extend(c.kind.own_params());
        }
    }
    ps
}

impl SubAnalysis {
    /// Independent directions in which the classes `ks` (distinct) can move.
    fn free_dof(&self, ks: &[u32]) -> usize {
        let mut free = 0;
        let mut by_piece: BTreeMap<u32, SmallVec<[u32; 8]>> = BTreeMap::new();
        for &k in ks {
            match self.class_at[k as usize] {
                ClassAt::Free => free += 1,
                ClassAt::Pinned => {}
                ClassAt::In { piece, row } => by_piece.entry(piece).or_default().push(row),
            }
        }
        for (piece, rows) in by_piece {
            free += match &self.pieces[piece as usize].basis {
                Basis::Structural { d } => rows.len().min(*d),
                Basis::Dense { d, q } => {
                    let d = *d;
                    if d == 0 {
                        0
                    } else {
                        let m: Vec<f64> = rows
                            .iter()
                            .flat_map(|&r| q[r as usize * d..(r as usize + 1) * d].iter().copied())
                            .collect();
                        rank_abs(&m, rows.len(), d, NULL_TAU)
                    }
                }
            };
        }
        free
    }

    /// Per-entity status; `over` entities are `Over`.
    fn report(
        &self,
        sketch: &Sketch,
        st: &Structure,
        over: &BTreeSet<EntityId>,
    ) -> BTreeMap<EntityId, EntityStatus> {
        let mut out = BTreeMap::new();
        for (id, e) in sketch.entities() {
            let mut ks: SmallVec<[u32; 8]> = entity_params(sketch, &e.kind)
                .iter()
                .filter_map(|p| match st.slot.get(p.index()) {
                    Some(Slot::Unknown(k)) => Some(*k),
                    _ => None,
                })
                .collect();
            ks.sort_unstable();
            ks.dedup();
            let free = self.free_dof(&ks);
            let state = if over.contains(&id) {
                ConstraintState::Over
            } else if free > 0 {
                ConstraintState::Under
            } else {
                ConstraintState::Full
            };
            out.insert(
                id,
                EntityStatus {
                    state,
                    free_dof: u8::try_from(free).unwrap_or(u8::MAX),
                },
            );
        }
        out
    }

    /// Null-space image of a gradient over parameters: one value per null direction it
    /// touches (`None` if it touches a structural piece).
    fn project(
        &self,
        st: &Structure,
        grad: &[(ParamId, f64)],
    ) -> Option<BTreeMap<(u32, u32), f64>> {
        let mut by_class: BTreeMap<u32, f64> = BTreeMap::new();
        for &(p, v) in grad {
            if let Some(Slot::Unknown(k)) = st.slot.get(p.index()) {
                *by_class.entry(*k).or_default() += v;
            }
        }
        let mut out: BTreeMap<(u32, u32), f64> = BTreeMap::new();
        for (k, v) in by_class {
            match self.class_at[k as usize] {
                ClassAt::Free => *out.entry((u32::MAX, k)).or_default() += v,
                ClassAt::Pinned => {}
                ClassAt::In { piece, row } => match &self.pieces[piece as usize].basis {
                    Basis::Structural { .. } => return None,
                    Basis::Dense { d, q } => {
                        for c in 0..*d {
                            *out.entry((piece, c as u32)).or_default() +=
                                v * q[row as usize * d + c];
                        }
                    }
                },
            }
        }
        Some(out)
    }
}

/// Deterministic value in `[-1, 1]` for parameter `p` (splitmix64).
fn jitter(p: ParamId) -> f64 {
    let mut z = u64::from(p.0).wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^= z >> 31;
    (z >> 11) as f64 / (1u64 << 52) as f64 - 1.0
}

/// Solver settings for internal scratch solves.
#[derive(Copy, Clone, Debug)]
pub(crate) struct Settings {
    pub dense_max: usize,
    pub decompose: bool,
}

impl Settings {
    fn solver(self) -> NativeSolver {
        NativeSolver::new()
            .with_dense_limit(self.dense_max)
            .with_decomposition(self.decompose)
    }
}

/// `sketch` with its parameters set to `x`.
fn at_config(sketch: &Sketch, x: &[f64]) -> Sketch {
    let mut s = sketch.clone();
    let params = s.params_mut();
    for (i, &v) in x.iter().enumerate().take(params.len()) {
        params.set(ParamId(i as u32), v);
    }
    s
}

/// Moves the free parameters of `g` off `x` by the deterministic witness jitter (points by
/// `WITNESS_STEP · L_c`, radii relatively, other parameters by `0.1 · WITNESS_STEP`).
fn jitter_free(g: &mut Sketch, x: &[f64], free: &[ParamId], length_scale: f64) {
    let is_free = |p: ParamId| free.binary_search(&p).is_ok();
    let mut touched = vec![false; x.len()];
    let entities: Vec<EntityKind> = g.entities().map(|(_, e)| e.kind.clone()).collect();
    let params = g.params_mut();
    let mut nudge = |p: ParamId, f: &dyn Fn(f64) -> f64| {
        if p.index() < touched.len() && !touched[p.index()] && is_free(p) {
            touched[p.index()] = true;
            params.set(p, f(x[p.index()]));
        }
    };
    for kind in &entities {
        match *kind {
            EntityKind::Point { x: px, y: py } => {
                for p in [px, py] {
                    nudge(p, &|v| v + WITNESS_STEP * length_scale * jitter(p));
                }
            }
            EntityKind::Circle { radius: r, .. }
            | EntityKind::Ellipse {
                minor_radius: r, ..
            } => nudge(r, &|v| v * (1.0 + WITNESS_STEP * jitter(r))),
            _ => {
                for p in kind.own_params() {
                    nudge(p, &|v| v + 0.1 * WITNESS_STEP * jitter(p));
                }
            }
        }
    }
    for p in free {
        nudge(*p, &|v| v + 0.1 * WITNESS_STEP * jitter(*p));
    }
}

/// The witness configuration of `sketch` at `x` (module docs): `Some(None)` if the geometric
/// constraints alone do not converge, `None` if cancelled.
fn witness(
    sketch: &Sketch,
    x: &[f64],
    free: &[ParamId],
    length_scale: f64,
    settings: Settings,
    cancel: &dyn Fn() -> bool,
) -> Option<Option<Vec<f64>>> {
    let mut g = at_config(sketch, x);
    let driving: Vec<ConstraintId> = sketch
        .constraints()
        .filter(|(_, c)| matches!(&c.kind, ConstraintKind::Dimension(d) if d.driving))
        .map(|(id, _)| id)
        .collect();
    for id in driving {
        if g.set_driving(id, false).is_err() {
            return Some(None);
        }
    }
    jitter_free(&mut g, x, free, length_scale);
    let r = settings
        .solver()
        .solve(&mut g, &SolveOptions::fast(), cancel);
    match r.status {
        crate::solve::api::SolveStatus::Cancelled => None,
        s if s.is_converged() => Some(Some(g.params().as_slice().to_vec())),
        _ => Some(None),
    }
}

/// Everything known about a sketch at one configuration (cached for `check_constraint`).
pub(crate) struct Analysis {
    pub dof: DofReport,
    pub diagnosis: Diagnosis,
    /// Nothing deficient and every row satisfied at the sketch's own values.
    pub clean: bool,
    pub sub: SubAnalysis,
}

fn sort_groups(groups: &mut Vec<DependencyGroup>) {
    let rank = |k: GroupKind| match k {
        GroupKind::Conflicting => 0u8,
        GroupKind::Redundant => 1,
        GroupKind::DegenerateConfiguration => 2,
    };
    groups.sort_by(|a, b| (rank(a.kind), &a.constraints).cmp(&(rank(b.kind), &b.constraints)));
    groups.dedup();
}

fn group_of(
    kind: GroupKind,
    owners: impl IntoIterator<Item = EquationOwner>,
) -> Option<DependencyGroup> {
    let set: BTreeSet<ConstraintId> = owners
        .into_iter()
        .filter_map(|o| match o {
            EquationOwner::Constraint(id) => Some(id),
            EquationOwner::Entity(_) => None,
        })
        .collect();
    (!set.is_empty()).then(|| DependencyGroup {
        kind,
        constraints: set.into_iter().collect(),
    })
}

fn grads_of<'a>(v: &'a RawValues, rows: &[usize]) -> Vec<&'a [(u32, f64)]> {
    rows.iter().map(|&r| v.grad[r].as_slice()).collect()
}

/// True if every row of `s` owned by one of `owners` (except `removed`) is satisfied at the
/// sketch's own values.
fn component_satisfied(
    s: &Sketch,
    owners: &BTreeSet<EquationOwner>,
    removed: ConstraintId,
    tol_of: &dyn Fn(f64) -> f64,
) -> bool {
    let Ok(sys) = crate::solve::residual::build_system(s) else {
        return false;
    };
    let tol = tol_of(sys.length_scale);
    let x = s.params().as_slice();
    let mut f = Vec::new();
    for (owner, res, _) in sys.system.iter() {
        if owner == EquationOwner::Constraint(removed) || !owners.contains(&owner) {
            continue;
        }
        f.clear();
        f.resize(res.rows(), 0.0);
        res.eval(x, &mut f);
        if f.iter().any(|v| v.abs() > tol || v.is_nan()) {
            return false;
        }
    }
    true
}

/// Full analysis of `sketch` at configuration `x` (the solved values, or a least-squares point
/// for an inconsistent sketch); `None` if cancelled.
pub(crate) fn analyze_at(
    sketch: &Sketch,
    x: &[f64],
    sys: &SketchSystem,
    st: &Structure,
    settings: Settings,
    cancel: &dyn Fn() -> bool,
) -> Option<Analysis> {
    let tol = effective_tol(SolveOptions::default().tolerance, sys.length_scale);
    let sa = substituted(&sys.system, st, x, cancel)?;
    let raw = RawSystem::new(sketch, &sys.system, &sys.free);
    let rv = raw.at(&sys.system, x);
    let unsat: Vec<bool> = rv.f.iter().map(|v| v.abs() > tol || v.is_nan()).collect();
    let suspicious = !st.redundant_subst.is_empty()
        || !st.const_blocks.is_empty()
        || !st.fixed_joins.is_empty()
        || raw.multi_fixed
        || sa.deficient.iter().any(|&d| d)
        || unsat.iter().any(|&u| u);
    let total =
        |sa: &SubAnalysis| u32::try_from(st.unknowns().saturating_sub(sa.rank)).unwrap_or(u32::MAX);
    if !suspicious {
        return Some(Analysis {
            dof: DofReport {
                dof: total(&sa),
                entities: sa.report(sketch, st, &BTreeSet::new()),
            },
            diagnosis: Diagnosis::default(),
            clean: true,
            sub: sa,
        });
    }

    // Suspect raw rows: unsatisfied, substitution-redundant or constant blocks, double fixes,
    // and rows touching a deficient substituted component or a fixed join.
    let mut comp_of_class = vec![u32::MAX; st.unknowns()];
    for (ci, c) in st.components.iter().enumerate() {
        for &k in &c.unknowns {
            comp_of_class[k as usize] = ci as u32;
        }
    }
    let mut param_of_col = vec![ParamId(u32::MAX); raw.ncols];
    for (i, &c) in raw.col.iter().enumerate() {
        if c != u32::MAX {
            param_of_col[c as usize] = ParamId(i as u32);
        }
    }
    let special_blocks: BTreeSet<u32> = st
        .redundant_subst
        .iter()
        .chain(&st.const_blocks)
        .copied()
        .collect();
    let joined: BTreeSet<u32> = st.fixed_joins.iter().flat_map(|j| [j.a, j.b]).collect();
    let mut fix_count: BTreeMap<ParamId, u32> = BTreeMap::new();
    for r in &raw.rows {
        if let RowSrc::Fix { param } = r.src {
            *fix_count.entry(param).or_default() += 1;
        }
    }
    let col_suspect = |c: u32| {
        let p = param_of_col[c as usize];
        if joined.contains(&p.0) {
            return true;
        }
        match st.slot.get(p.index()) {
            Some(Slot::Unknown(k)) => {
                let ci = comp_of_class[*k as usize];
                ci != u32::MAX && sa.deficient[ci as usize]
            }
            Some(Slot::Fixed { src }) => joined.contains(src),
            _ => false,
        }
    };
    let row_suspect = |i: usize| {
        let r = &raw.rows[i];
        unsat[i]
            || match r.src {
                RowSrc::Res { block, .. } => special_blocks.contains(&block),
                RowSrc::Fix { param } => fix_count.get(&param).copied().unwrap_or(0) > 1,
            }
            || r.cols.iter().any(|&c| col_suspect(c))
    };

    struct Comp {
        rows: Vec<usize>,
        conflicting: bool,
        circuits: Vec<Vec<usize>>,
    }
    let mut comps = Vec::new();
    for rows in raw.components() {
        if !rows.iter().any(|&i| row_suspect(i)) {
            continue;
        }
        let circ = circuits(&rows, &grads_of(&rv, &rows), cancel)?;
        let conflicting = rows.iter().any(|&i| unsat[i]);
        comps.push(Comp {
            rows,
            conflicting,
            circuits: circ,
        });
    }

    let owners_of = |rows: &[usize]| rows.iter().map(|&i| raw.rows[i].owner).collect::<Vec<_>>();
    let mut groups: Vec<DependencyGroup> = Vec::new();
    let any_circuit = comps.iter().any(|c| !c.circuits.is_empty());
    let mut sub_report = sa;
    if any_circuit {
        let w = witness(sketch, x, &sys.free, sys.length_scale, settings, cancel)?;
        let wv = w.as_ref().map(|xw| raw.at(&sys.system, xw));
        for c in &comps {
            for circ in &c.circuits {
                if circ.iter().any(|&i| unsat[i]) {
                    continue;
                }
                let genuine = match &wv {
                    Some(wv) => !circuits(circ, &grads_of(wv, circ), cancel)?.is_empty(),
                    None => true,
                };
                let kind = if genuine {
                    GroupKind::Redundant
                } else {
                    GroupKind::DegenerateConfiguration
                };
                groups.extend(group_of(kind, owners_of(circ)));
            }
        }
        if let Some(xw) = &w {
            let sw = substituted(&sys.system, st, xw, cancel)?;
            if sw.rank > sub_report.rank {
                sub_report = sw;
            }
        }
    }

    // Conflicts: leave-one-out per unsatisfied component.
    for c in comps.iter().filter(|c| c.conflicting) {
        let owners: BTreeSet<EquationOwner> = owners_of(&c.rows).into_iter().collect();
        let removable: Vec<ConstraintId> = owners
            .iter()
            .filter_map(|o| match o {
                EquationOwner::Constraint(id) => Some(*id),
                EquationOwner::Entity(_) => None,
            })
            .collect();
        let mut culprits = Vec::new();
        if removable.len() <= LOO_MAX {
            // Start from `x`, then from the sketch's own values: `x` can be a projection that is
            // singular for the remaining constraints (horizontal + vertical collapse a line).
            // Then from jittered values: a start can make the remaining rows' gradients
            // parallel (vertical + length on a horizontal line).
            let mut jittered = sketch.clone();
            jitter_free(&mut jittered, x, &sys.free, sys.length_scale);
            let starts = [at_config(sketch, x), sketch.clone(), jittered];
            let tol_of = |lc: f64| effective_tol(SolveOptions::default().tolerance, lc);
            for &id in &removable {
                for base in &starts {
                    if cancel() {
                        return None;
                    }
                    let mut s = base.clone();
                    if s.remove_constraint(id).is_err() {
                        break;
                    }
                    let r = settings
                        .solver()
                        .solve(&mut s, &SolveOptions::fast(), cancel);
                    if r.status == crate::solve::api::SolveStatus::Cancelled {
                        return None;
                    }
                    if component_satisfied(&s, &owners, id, &tol_of) {
                        culprits.push(EquationOwner::Constraint(id));
                        break;
                    }
                }
            }
        }
        if let Some(g) = group_of(GroupKind::Conflicting, culprits) {
            groups.push(g);
            continue;
        }
        let mut found = false;
        for circ in c
            .circuits
            .iter()
            .filter(|circ| circ.iter().any(|&i| unsat[i]))
        {
            if let Some(g) = group_of(GroupKind::Conflicting, owners_of(circ)) {
                groups.push(g);
                found = true;
            }
        }
        if !found {
            let bad: Vec<usize> = c.rows.iter().copied().filter(|&i| unsat[i]).collect();
            groups.extend(group_of(GroupKind::Conflicting, owners_of(&bad)));
        }
    }
    // A redundancy among constraints that already conflict adds nothing to the conflict.
    let conflicts: Vec<BTreeSet<ConstraintId>> = groups
        .iter()
        .filter(|g| g.kind == GroupKind::Conflicting)
        .map(|g| g.constraints.iter().copied().collect())
        .collect();
    groups.retain(|g| {
        g.kind != GroupKind::Redundant
            || !conflicts
                .iter()
                .any(|c| g.constraints.iter().all(|id| c.contains(id)))
    });
    sort_groups(&mut groups);

    let over: BTreeSet<EntityId> = groups
        .iter()
        .filter(|g| g.kind != GroupKind::DegenerateConfiguration)
        .flat_map(|g| g.constraints.iter())
        .filter_map(|&id| sketch.constraint(id))
        .flat_map(|c| c.kind.entities())
        .collect();
    Some(Analysis {
        dof: DofReport {
            dof: total(&sub_report),
            entities: sub_report.report(sketch, st, &over),
        },
        diagnosis: Diagnosis { groups },
        clean: false,
        sub: sub_report,
    })
}

/// Cached analysis of the last sketch `check_constraint` / `analyze` saw.
pub(crate) struct Base {
    rev: u64,
    x: Vec<f64>,
    /// Dimension values (baked into the residuals).
    dim_values: BTreeMap<ConstraintId, u64>,
    sys: SketchSystem,
    st: Arc<Structure>,
    pub analysis: Analysis,
}

impl Base {
    fn matches(&self, sketch: &Sketch) -> bool {
        self.rev == sketch.structure_rev()
            && self.x.len() == sketch.params().len()
            && self
                .x
                .iter()
                .zip(sketch.params().as_slice())
                .all(|(a, b)| a.to_bits() == b.to_bits())
            && sketch
                .constraints()
                .filter_map(|(id, c)| c.kind.as_dimension().map(|d| (id, d.value.to_bits())))
                .eq(self.dim_values.iter().map(|(&id, &v)| (id, v)))
    }
}

/// The group `check_constraint` reports for constraint `id` (conflict first).
fn check_result(groups: &[DependencyGroup], id: ConstraintId) -> ConstraintCheck {
    let with = |g: &DependencyGroup| -> Vec<ConstraintId> {
        g.constraints.iter().copied().filter(|&c| c != id).collect()
    };
    let mine = |k: GroupKind| {
        groups
            .iter()
            .find(|g| g.kind == k && g.constraints.contains(&id))
    };
    if let Some(g) = mine(GroupKind::Conflicting) {
        ConstraintCheck::Conflicting { with: with(g) }
    } else if let Some(g) = mine(GroupKind::Redundant) {
        ConstraintCheck::Redundant { with: with(g) }
    } else {
        ConstraintCheck::Independent
    }
}

impl NativeSolver {
    pub(crate) fn settings(&self) -> Settings {
        Settings {
            dense_max: self.dense_max,
            decompose: self.decompose,
        }
    }

    /// Analysis of `sketch` (solving a scratch copy first if its rows are not satisfied),
    /// cached for [`check_constraint`](SolverBackend::check_constraint); `None` if cancelled
    /// or the equations cannot be built.
    pub(crate) fn base_analysis(
        &mut self,
        sketch: &Sketch,
        cancel: &dyn Fn() -> bool,
    ) -> Option<Arc<Base>> {
        if let Some(b) = &self.analysis
            && b.matches(sketch)
        {
            return Some(b.clone());
        }
        let (sys, st) = self.prepare(sketch, cancel)?.ok()?;
        let x = sketch.params().as_slice().to_vec();
        let tol = effective_tol(SolveOptions::default().tolerance, sys.length_scale);
        let satisfied = sys.system.max_abs_residual(&x) <= tol;
        let settings = self.settings();
        let analysis = if satisfied {
            analyze_at(sketch, &x, &sys, &st, settings, cancel)?
        } else {
            let mut work = sketch.clone();
            let r = self.solve(&mut work, &SolveOptions::fast(), cancel);
            if r.status == crate::solve::api::SolveStatus::Cancelled {
                return None;
            }
            let xs = work.params().as_slice().to_vec();
            let mut a = analyze_at(sketch, &xs, &sys, &st, settings, cancel)?;
            a.clean = false;
            a
        };
        let dim_values = sketch
            .constraints()
            .filter_map(|(id, c)| c.kind.as_dimension().map(|d| (id, d.value.to_bits())))
            .collect();
        let base = Arc::new(Base {
            rev: sketch.structure_rev(),
            x,
            sys,
            st,
            dim_values,
            analysis,
        });
        self.analysis = Some(base.clone());
        Some(base)
    }

    /// Incremental [`check_constraint`](SolverBackend::check_constraint).
    pub(crate) fn check(
        &mut self,
        sketch: &Sketch,
        candidate: &ConstraintKind,
    ) -> Result<ConstraintCheck, SketchError> {
        sketch.validate_new_constraint(candidate)?;
        if matches!(candidate, ConstraintKind::Dimension(d) if !d.driving) {
            // A driven dimension adds no equations.
            return Ok(ConstraintCheck::Independent);
        }
        let never = || false;
        let fast = !matches!(candidate, ConstraintKind::Fix { .. })
            && sketch.new_constraint_aux_count(candidate) == 0;
        if fast
            && let Some(base) = self.base_analysis(sketch, &never)
            && base.analysis.clean
            && let Some(out) = self.check_fast(sketch, candidate, &base)?
        {
            return Ok(out);
        }
        // Full re-analysis of the sketch with the candidate.
        let mut s = sketch.clone();
        let id = s.add_constraint(candidate.clone())?;
        let mut scratch = self.settings().solver();
        let Some(base) = scratch.base_analysis(&s, &never) else {
            return Err(SketchError::NotApplicable("equations cannot be built"));
        };
        Ok(check_result(&base.analysis.diagnosis.groups, id))
    }

    /// Fast paths on a clean base: `None` when the full re-analysis is needed (the candidate
    /// is not satisfied at the current values, or touches a structural piece).
    fn check_fast(
        &mut self,
        sketch: &Sketch,
        candidate: &ConstraintKind,
        base: &Base,
    ) -> Result<Option<ConstraintCheck>, SketchError> {
        let Some(res) = constraint_residual(sketch, candidate, base.sys.length_scale)? else {
            return Ok(Some(ConstraintCheck::Independent));
        };
        let x = sketch.params().as_slice();
        let tol = effective_tol(SolveOptions::default().tolerance, base.sys.length_scale);
        let k = res.rows();
        let params = res.params().to_vec();
        let mut f = vec![0.0; k];
        res.eval(x, &mut f);
        if f.iter().any(|v| v.abs() > tol || v.is_nan()) {
            return Ok(None);
        }
        let mut jac = vec![0.0; k * params.len()];
        res.jacobian(x, &mut jac);
        // Independence: the rows' images in the null space have full rank.
        let mut images = Vec::with_capacity(k);
        let mut gmax = 0.0_f64;
        for r in 0..k {
            let g: Vec<(ParamId, f64)> = params
                .iter()
                .enumerate()
                .filter(|(_, p)| base.sys.free.binary_search(p).is_ok())
                .map(|(j, &p)| {
                    let v = jac[r * params.len() + j];
                    (p, if v.is_finite() { v } else { 0.0 })
                })
                .collect();
            gmax = gmax.max(g.iter().map(|e| e.1 * e.1).sum::<f64>().sqrt());
            let Some(img) = base.analysis.sub.project(&base.st, &g) else {
                return Ok(None);
            };
            images.push(img);
        }
        let keys: BTreeSet<(u32, u32)> = images.iter().flat_map(|m| m.keys().copied()).collect();
        let keys: Vec<(u32, u32)> = keys.into_iter().collect();
        let mut m = vec![0.0; k * keys.len()];
        for (r, img) in images.iter().enumerate() {
            for (c, key) in keys.iter().enumerate() {
                m[r * keys.len() + c] = img.get(key).copied().unwrap_or(0.0);
            }
        }
        if rank_abs(&m, k, keys.len(), CAND_REL * gmax.max(1e-300)) == k {
            return Ok(Some(ConstraintCheck::Independent));
        }
        // Dependent at the current values: circuits on the candidate's raw component plus
        // the candidate rows (last, as it has the largest id), classified at the witness of
        // the sketch with the candidate.
        let mut s = sketch.clone();
        let id = s.add_constraint(candidate.clone())?;
        let raw = RawSystem::new(sketch, &base.sys.system, &base.sys.free);
        let rv = raw.at(&base.sys.system, x);
        let cand_grad = |x: &[f64]| -> Vec<SmallVec<[(u32, f64); 8]>> {
            let mut jac = vec![0.0; k * params.len()];
            res.jacobian(x, &mut jac);
            (0..k)
                .map(|r| {
                    let mut g: SmallVec<[(u32, f64); 8]> = SmallVec::new();
                    for (j, p) in params.iter().enumerate() {
                        let c = raw.col.get(p.index()).copied().unwrap_or(u32::MAX);
                        let v = jac[r * params.len() + j];
                        if c == u32::MAX || v == 0.0 {
                            continue;
                        }
                        let v = if v.is_finite() { v } else { 0.0 };
                        match g.iter_mut().find(|(cc, _)| *cc == c) {
                            Some(e) => e.1 += v,
                            None => g.push((c, v)),
                        }
                    }
                    g
                })
                .collect()
        };
        let cg = cand_grad(x);
        let touched: BTreeSet<u32> = params
            .iter()
            .filter_map(|p| raw.col.get(p.index()).copied().filter(|&c| c != u32::MAX))
            .collect();
        let mut rows: Vec<usize> = raw
            .components()
            .into_iter()
            .filter(|c| {
                c.iter()
                    .any(|&i| raw.rows[i].cols.iter().any(|cc| touched.contains(cc)))
            })
            .flatten()
            .collect();
        rows.sort_unstable();
        let n0 = raw.rows.len();
        let ids: Vec<usize> = rows.iter().copied().chain(n0..n0 + k).collect();
        let grads: Vec<&[(u32, f64)]> = rows
            .iter()
            .map(|&i| rv.grad[i].as_slice())
            .chain(cg.iter().map(|g| g.as_slice()))
            .collect();
        let never = || false;
        let circ = circuits(&ids, &grads, &never).unwrap_or_default();
        let owner = |i: usize| {
            if i >= n0 {
                EquationOwner::Constraint(id)
            } else {
                raw.rows[i].owner
            }
        };
        let settings = self.settings();
        let xs = s.params().as_slice().to_vec();
        let w = witness(
            &s,
            &xs,
            &base.sys.free,
            base.sys.length_scale,
            settings,
            &never,
        )
        .flatten();
        let (wv, wc) = match &w {
            Some(xw) => (Some(raw.at(&base.sys.system, xw)), cand_grad(xw)),
            None => (None, Vec::new()),
        };
        let mut groups = Vec::new();
        for c in &circ {
            let genuine = match &wv {
                Some(wv) => {
                    let g: Vec<&[(u32, f64)]> = c
                        .iter()
                        .map(|&i| {
                            if i >= n0 {
                                wc[i - n0].as_slice()
                            } else {
                                wv.grad[i].as_slice()
                            }
                        })
                        .collect();
                    !circuits(c, &g, &never).unwrap_or_default().is_empty()
                }
                None => true,
            };
            let kind = if genuine {
                GroupKind::Redundant
            } else {
                GroupKind::DegenerateConfiguration
            };
            groups.extend(group_of(kind, c.iter().map(|&i| owner(i))));
        }
        sort_groups(&mut groups);
        Ok(Some(check_result(&groups, id)))
    }
}
