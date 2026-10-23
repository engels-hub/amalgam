//! The symbolic part of a solve, cached per `Sketch::structure_rev` (note §3.1):
//!
//! 1. **Substitution.** Equations that are exactly `a − b = 0` (coincident, horizontal,
//!    vertical, concentric, equal radius of two circles) alias their parameters with a
//!    union-find; classes that contain a fixed parameter are constants.
//! 2. **Connected components** of the remaining equations over the unknown classes.
//! 3. **Dulmage–Mendelsohn** per component: a maximum matching of rows to unknowns, the
//!    over-determined part (rows reachable from unmatched rows), the under-determined part
//!    (unknowns reachable from unmatched unknowns) and the square part split into strongly
//!    connected blocks (Tarjan) in dependency order. Over- and under-determined parts are split
//!    into their connected pieces. Solve order per component: over, square blocks, under.
//! 4. **Sparsity.** Every block gets its CSR Jacobian pattern, the scatter map from residual
//!    Jacobians into it and, above the dense limit, the sparse Gram structure.
//!
//! Nothing here depends on parameter values, so a cached structure gives bit-identical results
//! to a fresh one.

use smallvec::SmallVec;

use super::sparse::SparseGram;
use crate::ids::{EntityId, ParamId};
use crate::model::{ConstraintKind, EntityKind, HvTarget, Sketch};
use crate::solve::residual::SketchSystem;
use crate::solve::system::{EquationOwner, EquationSystem};

/// "No index" marker.
pub(crate) const NONE: u32 = u32::MAX;

/// How the value of one parameter slot is determined.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Slot {
    /// Not part of the system.
    Unused,
    /// Constant: a fixed param keeps its value; a free param aliased to a fixed class takes the
    /// value of param `src` (the class's first fixed member in union order).
    Fixed { src: u32 },
    /// Aliased to unknown class `k`.
    Unknown(u32),
}

/// A substitution that united two classes which both held a fixed param: a conflict if the
/// two values differ.
#[derive(Copy, Clone, Debug)]
pub(crate) struct FixedJoin {
    pub a: u32,
    pub b: u32,
}

/// The selected rows of one residual block inside a [`Sub`].
#[derive(Clone, Debug)]
pub(crate) struct Part {
    /// Residual block index in the equation system.
    pub res: u32,
    /// Selected rows of the residual, ascending.
    pub rows: SmallVec<[u16; 4]>,
    /// Local row of `rows[0]`.
    pub first_row: u32,
    /// `params().len()` of the residual.
    pub np: u32,
    /// Offset into [`Sub::scatter`] (`rows.len() · np` entries).
    pub scatter_off: u32,
}

/// Kind of a [`Sub`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum SubKind {
    /// More rows than unknowns (structurally redundant).
    Over,
    /// Square, structurally nonsingular.
    Square,
    /// Fewer rows than unknowns.
    Under,
    /// A whole component (the monolithic fallback).
    Whole,
}

/// A subsystem solved as one unit: rows over local unknowns.
#[derive(Debug)]
pub(crate) struct Sub {
    pub kind: SubKind,
    /// Unknown classes, ascending.
    pub unknowns: Vec<u32>,
    pub parts: Vec<Part>,
    pub rows: usize,
    /// CSR pattern of the local Jacobian.
    pub row_ptr: Vec<u32>,
    pub col_idx: Vec<u32>,
    /// Per part, per selected row, per residual param: CSR index or `NONE` (constant).
    pub scatter: Vec<u32>,
    /// Sparse Gram structure when `unknowns.len() > dense_max`.
    pub sparse: Option<Box<SparseGram>>,
}

impl Sub {
    pub(crate) fn n(&self) -> usize {
        self.unknowns.len()
    }

    pub(crate) fn nnz(&self) -> usize {
        self.col_idx.len()
    }
}

/// One connected component.
#[derive(Debug)]
pub(crate) struct Component {
    /// Dulmage–Mendelsohn blocks in solve order (one entry: the component is one block).
    pub blocks: Vec<Sub>,
    /// The whole component when it has more than one block (fallback and rank test).
    pub whole: Option<Sub>,
    /// Unknown classes of the component, ascending.
    pub unknowns: Vec<u32>,
    pub rows: usize,
}

impl Component {
    /// The whole component as one subsystem.
    pub(crate) fn whole(&self) -> &Sub {
        self.whole.as_ref().unwrap_or(&self.blocks[0])
    }
}

/// The cached structure of a sketch.
#[derive(Debug)]
pub(crate) struct Structure {
    pub rev: u64,
    pub n_params: usize,
    /// For validation against a rebuilt system.
    pub fixed: Vec<ParamId>,
    pub free: Vec<ParamId>,
    /// `(rows, params)` of every residual block.
    pub shape: Vec<(u32, u32)>,
    pub dense_max: usize,
    pub decompose: bool,

    pub slot: Vec<Slot>,
    /// Members of each unknown class, ascending.
    pub members: Vec<SmallVec<[ParamId; 4]>>,
    /// Substituted blocks (their rows hold by construction).
    pub substituted_rows: usize,
    /// Substituted blocks whose both sides were already aliased (dependent rows).
    pub redundant_subst: Vec<u32>,
    pub fixed_joins: Vec<FixedJoin>,
    /// Active blocks with no unknowns (pure checks; always dependent rows).
    pub const_blocks: Vec<u32>,
    pub components: Vec<Component>,
}

struct Uf(Vec<u32>);

impl Uf {
    fn new(n: usize) -> Self {
        Uf((0..n as u32).collect())
    }
    fn find(&mut self, mut i: usize) -> usize {
        while self.0[i] as usize != i {
            let p = self.0[i] as usize;
            self.0[i] = self.0[p];
            i = p;
        }
        i
    }
    /// Union by smaller index (deterministic roots).
    fn union(&mut self, a: usize, b: usize) -> usize {
        let (ra, rb) = (self.find(a), self.find(b));
        let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
        self.0[hi] = lo as u32;
        lo
    }
}

fn point_xy(sketch: &Sketch, id: EntityId) -> Option<[ParamId; 2]> {
    match sketch.entity(id)?.kind {
        EntityKind::Point { x, y } => Some([x, y]),
        _ => None,
    }
}

fn centre(sketch: &Sketch, id: EntityId) -> Option<EntityId> {
    match sketch.entity(id)?.kind {
        EntityKind::Circle { center, .. } | EntityKind::Arc { center, .. } => Some(center),
        _ => None,
    }
}

fn line_ends(sketch: &Sketch, id: EntityId) -> Option<(EntityId, EntityId)> {
    match sketch.entity(id)?.kind {
        EntityKind::Line { start, end } => Some((start, end)),
        _ => None,
    }
}

/// Parameter pairs `(p, q)` whose equations are exactly `p − q = 0`, if the block can be
/// replaced by aliasing.
fn subst_pairs(sketch: &Sketch, owner: EquationOwner) -> Option<SmallVec<[(ParamId, ParamId); 2]>> {
    let EquationOwner::Constraint(cid) = owner else {
        return None;
    };
    let kind = &sketch.constraint(cid)?.kind;
    let pts = |a: EntityId, b: EntityId| -> Option<([ParamId; 2], [ParamId; 2])> {
        Some((point_xy(sketch, a)?, point_xy(sketch, b)?))
    };
    let hv = |t: &HvTarget| -> Option<([ParamId; 2], [ParamId; 2])> {
        match *t {
            HvTarget::Line(l) => {
                let (s, e) = line_ends(sketch, l)?;
                pts(s, e)
            }
            HvTarget::Points(a, b) => pts(a, b),
        }
    };
    let mut out = SmallVec::new();
    match kind {
        ConstraintKind::Coincident { a, b } => {
            let (p, q) = pts(*a, *b)?;
            out.push((p[0], q[0]));
            out.push((p[1], q[1]));
        }
        ConstraintKind::Horizontal(t) => {
            let (p, q) = hv(t)?;
            out.push((p[1], q[1]));
        }
        ConstraintKind::Vertical(t) => {
            let (p, q) = hv(t)?;
            out.push((p[0], q[0]));
        }
        ConstraintKind::Concentric { a, b } => {
            let (p, q) = pts(centre(sketch, *a)?, centre(sketch, *b)?)?;
            out.push((p[0], q[0]));
            out.push((p[1], q[1]));
        }
        ConstraintKind::Equal { a, b } => {
            match (&sketch.entity(*a)?.kind, &sketch.entity(*b)?.kind) {
                (EntityKind::Circle { radius: ra, .. }, EntityKind::Circle { radius: rb, .. }) => {
                    out.push((*ra, *rb));
                }
                _ => return None,
            }
        }
        _ => return None,
    }
    Some(out)
}

impl Structure {
    /// True if this structure belongs to `sketch` / `sys` (same revision, fixed set and
    /// residual shapes).
    pub(crate) fn matches(
        &self,
        sketch: &Sketch,
        sys: &SketchSystem,
        dense_max: usize,
        decompose: bool,
    ) -> bool {
        self.rev == sketch.structure_rev()
            && self.dense_max == dense_max
            && self.decompose == decompose
            && self.n_params == sketch.params().len()
            && self.fixed.len() == sys.fixed.len()
            && self.fixed.iter().zip(&sys.fixed).all(|(a, b)| a == b)
            && self.free == sys.free
            && self.shape.len() == sys.system.len()
            && sys
                .system
                .iter()
                .zip(&self.shape)
                .all(|((_, r, rows), &(sr, sp))| {
                    rows.len() == sr as usize && r.params().len() == sp as usize
                })
    }

    /// Builds the structure of `sketch` from its system `sys`. With `decompose` off, every
    /// component is one block (no Dulmage–Mendelsohn split).
    pub(crate) fn new(
        sketch: &Sketch,
        sys: &SketchSystem,
        dense_max: usize,
        decompose: bool,
    ) -> Structure {
        let system = &sys.system;
        let n = sketch.params().len();
        let mut is_var = vec![false; n];
        let mut is_fixed = vec![false; n];
        for p in &sys.free {
            if p.index() < n {
                is_var[p.index()] = true;
            }
        }
        for p in &sys.fixed {
            if p.index() < n {
                is_var[p.index()] = true;
                is_fixed[p.index()] = true;
            }
        }

        // 1. Substitution.
        let mut uf = Uf::new(n);
        // First fixed member of each root (union order), `NONE` if none.
        let mut fixed_src: Vec<u32> = (0..n)
            .map(|i| if is_fixed[i] { i as u32 } else { NONE })
            .collect();
        let mut active = Vec::new();
        let mut redundant_subst = Vec::new();
        let mut fixed_joins = Vec::new();
        let mut substituted_rows = 0;
        for (bi, (owner, _, rows)) in system.iter().enumerate() {
            let pairs = subst_pairs(sketch, owner).filter(|ps| {
                ps.iter().all(|(p, q)| {
                    p.index() < n && q.index() < n && is_var[p.index()] && is_var[q.index()]
                })
            });
            let Some(pairs) = pairs else {
                active.push(bi);
                continue;
            };
            substituted_rows += rows.len();
            for (p, q) in pairs {
                let (rp, rq) = (uf.find(p.index()), uf.find(q.index()));
                if rp == rq {
                    if redundant_subst.last() != Some(&(bi as u32)) {
                        redundant_subst.push(bi as u32);
                    }
                    continue;
                }
                let (fa, fb) = (fixed_src[rp], fixed_src[rq]);
                if fa != NONE && fb != NONE {
                    fixed_joins.push(FixedJoin { a: fa, b: fb });
                }
                let r = uf.union(rp, rq);
                fixed_src[r] = if fa != NONE { fa } else { fb };
            }
        }

        // Classes, in ascending order of their smallest member.
        let mut slot = vec![Slot::Unused; n];
        let mut members: Vec<SmallVec<[ParamId; 4]>> = Vec::new();
        let mut class_of_root = vec![NONE; n];
        for i in 0..n {
            if !is_var[i] {
                continue;
            }
            let r = uf.find(i);
            if fixed_src[r] != NONE {
                slot[i] = Slot::Fixed { src: fixed_src[r] };
                continue;
            }
            if class_of_root[r] == NONE {
                class_of_root[r] = members.len() as u32;
                members.push(SmallVec::new());
            }
            let k = class_of_root[r];
            slot[i] = Slot::Unknown(k);
            members[k as usize].push(ParamId(i as u32));
        }

        // 2. Connected components over the active blocks.
        let nu = members.len();
        let mut cuf = Uf::new(nu);
        let mut const_blocks = Vec::new();
        let mut block_cols: Vec<(usize, SmallVec<[u32; 8]>)> = Vec::new();
        for &bi in &active {
            let mut ks: SmallVec<[u32; 8]> = SmallVec::new();
            for p in system.residual(bi).params() {
                if let Some(Slot::Unknown(k)) = slot.get(p.index())
                    && !ks.contains(k)
                {
                    ks.push(*k);
                }
            }
            if ks.is_empty() {
                const_blocks.push(bi as u32);
                continue;
            }
            ks.sort_unstable();
            for w in ks.windows(2) {
                cuf.union(w[0] as usize, w[1] as usize);
            }
            block_cols.push((bi, ks));
        }
        let mut comp_of_root = vec![usize::MAX; nu];
        let mut comp_blocks: Vec<Vec<usize>> = Vec::new();
        for (i, (_, ks)) in block_cols.iter().enumerate() {
            let r = cuf.find(ks[0] as usize);
            if comp_of_root[r] == usize::MAX {
                comp_of_root[r] = comp_blocks.len();
                comp_blocks.push(Vec::new());
            }
            comp_blocks[comp_of_root[r]].push(i);
        }
        let mut comp_unknowns: Vec<Vec<u32>> = vec![Vec::new(); comp_blocks.len()];
        for k in 0..nu {
            let r = cuf.find(k);
            if comp_of_root[r] != usize::MAX {
                comp_unknowns[comp_of_root[r]].push(k as u32);
            }
        }

        // 3. + 4. Decomposition and patterns per component.
        let mut local = vec![NONE; nu];
        let mut components = Vec::with_capacity(comp_blocks.len());
        for (blocks, unknowns) in comp_blocks.iter().zip(comp_unknowns) {
            // Scalar rows `(block, row)` and their columns (component-local).
            for (i, &k) in unknowns.iter().enumerate() {
                local[k as usize] = i as u32;
            }
            let mut rows: Vec<(u32, u16)> = Vec::new();
            let mut adj: Vec<SmallVec<[u32; 8]>> = Vec::new();
            for &i in blocks {
                let (bi, ks) = &block_cols[i];
                let cols: SmallVec<[u32; 8]> = ks.iter().map(|&k| local[k as usize]).collect();
                for r in 0..system.residual(*bi).rows() {
                    rows.push((*bi as u32, r as u16));
                    adj.push(cols.clone());
                }
            }
            let pieces = if decompose {
                dulmage_mendelsohn(rows.len(), unknowns.len(), &adj)
            } else {
                Vec::new()
            };
            let mut subs: Vec<Sub> = pieces
                .into_iter()
                .map(|(kind, rs, cs)| {
                    let rs: Vec<(u32, u16)> = rs.iter().map(|&r| rows[r as usize]).collect();
                    let cs: Vec<u32> = cs.iter().map(|&c| unknowns[c as usize]).collect();
                    make_sub(kind, &rs, cs, system, &slot, &mut local, dense_max)
                })
                .collect();
            let whole = if subs.len() == 1 {
                None
            } else {
                Some(make_sub(
                    SubKind::Whole,
                    &rows,
                    unknowns.clone(),
                    system,
                    &slot,
                    &mut local,
                    dense_max,
                ))
            };
            let total_rows = rows.len();
            let (blocks, whole) = match (subs.len(), whole) {
                (0, Some(w)) => (vec![w], None),
                (_, w) => (std::mem::take(&mut subs), w),
            };
            components.push(Component {
                blocks,
                whole,
                unknowns,
                rows: total_rows,
            });
        }

        Structure {
            rev: sketch.structure_rev(),
            n_params: n,
            fixed: sys.fixed.iter().copied().collect(),
            free: sys.free.clone(),
            shape: system
                .iter()
                .map(|(_, r, rows)| (rows.len() as u32, r.params().len() as u32))
                .collect(),
            dense_max,
            decompose,
            slot,
            members,
            substituted_rows,
            redundant_subst,
            fixed_joins,
            const_blocks,
            components,
        }
    }

    /// Number of unknown classes.
    pub(crate) fn unknowns(&self) -> usize {
        self.members.len()
    }

    /// Number of Dulmage–Mendelsohn blocks and the largest block's unknowns.
    pub(crate) fn block_stats(&self) -> (usize, usize) {
        let blocks = self.components.iter().map(|c| c.blocks.len()).sum();
        let largest = self
            .components
            .iter()
            .flat_map(|c| c.blocks.iter().map(Sub::n))
            .max()
            .unwrap_or(0);
        (blocks, largest)
    }
}

/// Builds a [`Sub`] over scalar rows `rows` (ascending) and unknown classes `cols` (ascending).
/// `local` is scratch of length `unknowns()`, all `NONE` on entry and exit.
fn make_sub(
    kind: SubKind,
    rows: &[(u32, u16)],
    cols: Vec<u32>,
    system: &EquationSystem,
    slot: &[Slot],
    local: &mut [u32],
    dense_max: usize,
) -> Sub {
    for (i, &k) in cols.iter().enumerate() {
        local[k as usize] = i as u32;
    }
    let mut parts: Vec<Part> = Vec::new();
    let mut row_ptr = vec![0u32];
    let mut col_idx = Vec::new();
    let mut scatter = Vec::new();
    let mut i = 0;
    while i < rows.len() {
        let res = rows[i].0;
        let mut j = i;
        while j < rows.len() && rows[j].0 == res {
            j += 1;
        }
        let r = system.residual(res as usize);
        let params = r.params();
        let pcols: SmallVec<[u32; 8]> = params
            .iter()
            .map(|p| match slot.get(p.index()) {
                Some(Slot::Unknown(k)) => local[*k as usize],
                _ => NONE,
            })
            .collect();
        let mut distinct: SmallVec<[u32; 8]> =
            pcols.iter().copied().filter(|&c| c != NONE).collect();
        distinct.sort_unstable();
        distinct.dedup();
        let part = Part {
            res,
            rows: rows[i..j].iter().map(|&(_, r)| r).collect(),
            first_row: i as u32,
            np: params.len() as u32,
            scatter_off: scatter.len() as u32,
        };
        for _ in i..j {
            let base = col_idx.len() as u32;
            col_idx.extend_from_slice(&distinct);
            row_ptr.push(col_idx.len() as u32);
            for &c in &pcols {
                scatter.push(if c == NONE {
                    NONE
                } else {
                    base + distinct.partition_point(|&d| d < c) as u32
                });
            }
        }
        parts.push(part);
        i = j;
    }
    for &k in &cols {
        local[k as usize] = NONE;
    }
    let m = rows.len();
    let sparse = (cols.len() > dense_max)
        .then(|| SparseGram::new(m, cols.len(), &row_ptr, &col_idx).map(Box::new))
        .flatten();
    Sub {
        kind,
        unknowns: cols,
        parts,
        rows: m,
        row_ptr,
        col_idx,
        scatter,
        sparse,
    }
}

/// Maximum matching of rows to columns (augmenting paths, rows in index order).
/// Returns `(match_row, match_col)` with `NONE` for unmatched.
fn max_matching(m: usize, n: usize, adj: &[SmallVec<[u32; 8]>]) -> (Vec<u32>, Vec<u32>) {
    let mut match_row = vec![NONE; m];
    let mut match_col = vec![NONE; n];
    // Cheap greedy pass first.
    for r in 0..m {
        if let Some(&c) = adj[r].iter().find(|&&c| match_col[c as usize] == NONE) {
            match_row[r] = c;
            match_col[c as usize] = r as u32;
        }
    }
    let mut seen = vec![0u32; n];
    let mut stamp = 0u32;
    let mut stack: Vec<(u32, usize)> = Vec::new();
    let mut via: Vec<u32> = Vec::new();
    for r0 in 0..m {
        if match_row[r0] != NONE {
            continue;
        }
        stamp += 1;
        stack.clear();
        via.clear();
        stack.push((r0 as u32, 0));
        via.push(NONE);
        while let Some(&mut (r, ref mut e)) = stack.last_mut() {
            let Some(&c) = adj[r as usize].get(*e) else {
                stack.pop();
                via.pop();
                continue;
            };
            *e += 1;
            if seen[c as usize] == stamp {
                continue;
            }
            seen[c as usize] = stamp;
            let owner = match_col[c as usize];
            if owner == NONE {
                // Augment: the top row takes `c`, each row below takes the column the row
                // above it held.
                let mut cur = c;
                for lvl in (0..stack.len()).rev() {
                    let row = stack[lvl].0;
                    match_row[row as usize] = cur;
                    match_col[cur as usize] = row;
                    cur = via[lvl];
                }
                break;
            }
            stack.push((owner, 0));
            via.push(c);
        }
    }
    (match_row, match_col)
}

/// Dulmage–Mendelsohn decomposition of an `m × n` bipartite graph (`adj`: columns of each
/// row). Returns the pieces in solve order as `(kind, rows, cols)`, rows and cols ascending.
#[allow(clippy::type_complexity, reason = "internal triple")]
fn dulmage_mendelsohn(
    m: usize,
    n: usize,
    adj: &[SmallVec<[u32; 8]>],
) -> Vec<(SubKind, Vec<u32>, Vec<u32>)> {
    let (match_row, match_col) = max_matching(m, n, adj);
    let mut adj_t: Vec<Vec<u32>> = vec![Vec::new(); n];
    for (r, cs) in adj.iter().enumerate() {
        for &c in cs {
            adj_t[c as usize].push(r as u32);
        }
    }
    // Over-determined part: alternating paths from unmatched rows.
    let (mut row_v, mut col_v) = (vec![false; m], vec![false; n]);
    let mut queue: Vec<u32> = (0..m as u32)
        .filter(|&r| match_row[r as usize] == NONE)
        .collect();
    for &r in &queue {
        row_v[r as usize] = true;
    }
    while let Some(r) = queue.pop() {
        for &c in &adj[r as usize] {
            if !col_v[c as usize] {
                col_v[c as usize] = true;
                let r2 = match_col[c as usize];
                if r2 != NONE && !row_v[r2 as usize] {
                    row_v[r2 as usize] = true;
                    queue.push(r2);
                }
            }
        }
    }
    // Under-determined part: alternating paths from unmatched columns.
    let (mut row_h, mut col_h) = (vec![false; m], vec![false; n]);
    let mut queue: Vec<u32> = (0..n as u32)
        .filter(|&c| match_col[c as usize] == NONE)
        .collect();
    for &c in &queue {
        col_h[c as usize] = true;
    }
    while let Some(c) = queue.pop() {
        for &r in &adj_t[c as usize] {
            if !row_h[r as usize] {
                row_h[r as usize] = true;
                let c2 = match_row[r as usize];
                if c2 != NONE && !col_h[c2 as usize] {
                    col_h[c2 as usize] = true;
                    queue.push(c2);
                }
            }
        }
    }

    let mut out = Vec::new();
    // Connected pieces of a part (rows linked through shared columns of the part).
    let pieces = |rows_in: &[bool], cols_in: &[bool], kind: SubKind| {
        let mut uf = Uf::new(n);
        for r in (0..m).filter(|&r| rows_in[r]) {
            let cs: SmallVec<[u32; 8]> = adj[r]
                .iter()
                .copied()
                .filter(|&c| cols_in[c as usize])
                .collect();
            for w in cs.windows(2) {
                uf.union(w[0] as usize, w[1] as usize);
            }
        }
        let mut idx = vec![usize::MAX; n];
        let mut groups: Vec<(Vec<u32>, Vec<u32>)> = Vec::new();
        for c in (0..n).filter(|&c| cols_in[c]) {
            let root = uf.find(c);
            if idx[root] == usize::MAX {
                idx[root] = groups.len();
                groups.push((Vec::new(), Vec::new()));
            }
            groups[idx[root]].1.push(c as u32);
        }
        for r in (0..m).filter(|&r| rows_in[r]) {
            if let Some(&c) = adj[r].iter().find(|&&c| cols_in[c as usize]) {
                let root = uf.find(c as usize);
                groups[idx[root]].0.push(r as u32);
            }
        }
        groups
            .into_iter()
            .filter(|(rs, _)| !rs.is_empty())
            .map(|(rs, cs)| (kind, rs, cs))
            .collect::<Vec<_>>()
    };
    out.extend(pieces(&row_v, &col_v, SubKind::Over));

    // Square part: strongly connected components of the matched digraph (Tarjan, iterative).
    // Node = matched column; edge c → c' if the row matched to c uses c'. Tarjan emits a
    // component after everything it reaches, i.e. dependencies first.
    let in_s = |c: usize| !col_v[c] && !col_h[c] && match_col[c] != NONE;
    let mut index = vec![NONE; n];
    let mut low = vec![0u32; n];
    let mut on_stack = vec![false; n];
    let mut stack: Vec<u32> = Vec::new();
    let mut next = 0u32;
    let mut call: Vec<(u32, usize)> = Vec::new();
    for start in 0..n {
        if !in_s(start) || index[start] != NONE {
            continue;
        }
        call.push((start as u32, 0));
        index[start] = next;
        low[start] = next;
        next += 1;
        stack.push(start as u32);
        on_stack[start] = true;
        while let Some(&mut (v, ref mut e)) = call.last_mut() {
            let row = match_col[v as usize] as usize;
            if let Some(&w) = adj[row].get(*e) {
                *e += 1;
                let w = w as usize;
                if w == v as usize || !in_s(w) {
                    continue;
                }
                if index[w] == NONE {
                    index[w] = next;
                    low[w] = next;
                    next += 1;
                    stack.push(w as u32);
                    on_stack[w] = true;
                    call.push((w as u32, 0));
                } else if on_stack[w] {
                    low[v as usize] = low[v as usize].min(index[w]);
                }
                continue;
            }
            call.pop();
            if let Some(&(parent, _)) = call.last() {
                low[parent as usize] = low[parent as usize].min(low[v as usize]);
            }
            if low[v as usize] == index[v as usize] {
                let mut cs = Vec::new();
                loop {
                    let w = stack.pop().unwrap_or(v);
                    on_stack[w as usize] = false;
                    cs.push(w);
                    if w == v {
                        break;
                    }
                }
                cs.sort_unstable();
                let mut rs: Vec<u32> = cs.iter().map(|&c| match_col[c as usize]).collect();
                rs.sort_unstable();
                out.push((SubKind::Square, rs, cs));
            }
        }
    }

    out.extend(pieces(&row_h, &col_h, SubKind::Under));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn adj(rows: &[&[u32]]) -> Vec<SmallVec<[u32; 8]>> {
        rows.iter().map(|r| r.iter().copied().collect()).collect()
    }

    #[test]
    fn matching_is_maximum() {
        // Greedy would match r0-c0 and leave r1 unmatched; augmenting fixes it.
        let a = adj(&[&[0, 1], &[0]]);
        let (mr, mc) = max_matching(2, 2, &a);
        assert_eq!(mr, [1, 0]);
        assert_eq!(mc, [1, 0]);
    }

    #[test]
    fn dm_orders_blocks_by_dependency() {
        // r0: c0 (c0 determined alone), r1: c0, c1 (needs c0), r2,r3: c2,c3 (2×2 cycle),
        // r4: c3 (over: c3 again with r2, r3 → no; give it its own over part below).
        let a = adj(&[&[0], &[0, 1], &[2, 3], &[2, 3]]);
        let p = dulmage_mendelsohn(4, 4, &a);
        let kinds: Vec<_> = p
            .iter()
            .map(|(k, rs, cs)| (*k, rs.clone(), cs.clone()))
            .collect();
        // c0's block must come before c1's.
        let pos = |c: u32| kinds.iter().position(|(_, _, cs)| cs.contains(&c)).unwrap();
        assert!(pos(0) < pos(1));
        assert!(
            kinds
                .iter()
                .any(|(k, rs, cs)| *k == SubKind::Square && rs == &[2, 3] && cs == &[2, 3])
        );
        assert!(kinds.iter().all(|(k, _, _)| *k == SubKind::Square));
    }

    #[test]
    fn dm_finds_over_and_under_parts() {
        // Over: r0, r1 both only on c0. Under: r2 on c1, c2.
        let a = adj(&[&[0], &[0], &[1, 2]]);
        let p = dulmage_mendelsohn(3, 3, &a);
        assert_eq!(p.len(), 2);
        assert_eq!(p[0], (SubKind::Over, vec![0, 1], vec![0]));
        assert_eq!(p[1], (SubKind::Under, vec![2], vec![1, 2]));
    }
}
