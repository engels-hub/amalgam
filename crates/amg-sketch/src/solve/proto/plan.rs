//! Structure of a prototype solve: substitution classes, connected components and the
//! per-component dense assembly.

use smallvec::SmallVec;

use super::ProtoOptions;
use super::linalg::max_abs;
use crate::error::SketchError;
use crate::ids::{EntityId, ParamId};
use crate::model::{ConstraintKind, EntityKind, HvTarget, Sketch};
use crate::solve::residual::{SketchSystem, build_system};
use crate::solve::system::EquationOwner;

/// How the value of one parameter slot is determined.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Slot {
    /// Not part of the system (unused or retired slot).
    Unused,
    /// Constant: a fixed/locked param, or aliased to one. The value is in `Plan::fixed_value`.
    Fixed,
    /// Aliased to unknown class `k`.
    Unknown(u32),
}

/// One connected component of the substituted system.
#[derive(Clone, Debug)]
pub(crate) struct Component {
    /// Unknown classes (global indices, ascending).
    pub unknowns: Vec<u32>,
    /// Equation blocks (indices into the system, ascending).
    pub blocks: Vec<usize>,
    /// First local row of each block.
    pub first_row: Vec<usize>,
    /// Local column of each block parameter (`None` = constant).
    pub cols: Vec<SmallVec<[Option<u32>; 8]>>,
    /// Total rows.
    pub rows: usize,
}

/// The substituted, decomposed system of a sketch.
pub(crate) struct Plan {
    pub sys: SketchSystem,
    pub slot: Vec<Slot>,
    /// Value of every `Slot::Fixed` param (indexed by param index; unused elsewhere).
    pub fixed_value: Vec<f64>,
    /// Members of each unknown class, ascending.
    pub members: Vec<SmallVec<[ParamId; 4]>>,
    pub components: Vec<Component>,
    /// Active blocks with no unknowns (pure checks; always dependent rows).
    pub const_blocks: Vec<usize>,
    /// Number of substituted equation rows.
    pub substituted_rows: usize,
    /// Blocks whose substitution was redundant (both sides already aliased).
    pub redundant_subst: Vec<usize>,
    /// Blocks whose substitution aliased two fixed params with different values.
    pub conflict_subst: Vec<usize>,
    /// Residual tolerance (mm).
    pub tol: f64,
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
/// replaced by aliasing (note §3.1: coincident, horizontal, vertical, concentric, equal radius
/// of two circles).
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

impl Plan {
    /// Builds the plan for `sketch` at its current values.
    pub(crate) fn new(sketch: &Sketch, opts: &ProtoOptions) -> Result<Plan, SketchError> {
        let sys = build_system(sketch)?;
        let tol = opts
            .tolerance
            .unwrap_or_else(|| (64.0 * f64::EPSILON * sys.length_scale).max(1e-10));
        let x0 = sketch.params().as_slice();
        let n = x0.len();
        let mut is_var = vec![false; n];
        let mut fixed: Vec<Option<f64>> = vec![None; n];
        for p in &sys.free {
            is_var[p.index()] = true;
        }
        for p in &sys.fixed {
            if p.index() < n {
                is_var[p.index()] = true;
                fixed[p.index()] = Some(x0[p.index()]);
            }
        }

        let mut uf = Uf::new(n);
        let mut active = Vec::new();
        let (mut redundant_subst, mut conflict_subst) = (Vec::new(), Vec::new());
        let mut substituted_rows = 0;
        for (bi, (owner, _, rows)) in sys.system.iter().enumerate() {
            let pairs = if opts.substitution {
                subst_pairs(sketch, owner).filter(|ps| {
                    ps.iter()
                        .all(|(p, q)| is_var[p.index()] && is_var[q.index()])
                })
            } else {
                None
            };
            let Some(pairs) = pairs else {
                active.push(bi);
                continue;
            };
            substituted_rows += rows.len();
            for (p, q) in pairs {
                let (rp, rq) = (uf.find(p.index()), uf.find(q.index()));
                if rp == rq {
                    if !redundant_subst.contains(&bi) {
                        redundant_subst.push(bi);
                    }
                    continue;
                }
                if let (Some(a), Some(b)) = (fixed[rp], fixed[rq])
                    && (a - b).abs() > tol
                    && !conflict_subst.contains(&bi)
                {
                    conflict_subst.push(bi);
                }
                let r = uf.union(rp, rq);
                fixed[r] = fixed[rp].or(fixed[rq]);
            }
        }

        // Classes, in ascending order of their smallest member.
        let mut slot = vec![Slot::Unused; n];
        let mut fixed_value = vec![0.0; n];
        let mut members: Vec<SmallVec<[ParamId; 4]>> = Vec::new();
        let mut class_of_root = vec![u32::MAX; n];
        for i in 0..n {
            if !is_var[i] {
                continue;
            }
            let r = uf.find(i);
            if let Some(v) = fixed[r] {
                slot[i] = Slot::Fixed;
                fixed_value[i] = v;
                continue;
            }
            if class_of_root[r] == u32::MAX {
                class_of_root[r] = members.len() as u32;
                members.push(SmallVec::new());
            }
            let k = class_of_root[r];
            slot[i] = Slot::Unknown(k);
            members[k as usize].push(ParamId(i as u32));
        }

        // Connected components over the active blocks.
        let nu = members.len();
        let mut cuf = Uf::new(nu);
        let mut const_blocks = Vec::new();
        let mut touched: Vec<(usize, SmallVec<[u32; 8]>)> = Vec::new();
        for &bi in &active {
            let mut ks: SmallVec<[u32; 8]> = SmallVec::new();
            for p in sys.system.residual(bi).params() {
                if let Some(Slot::Unknown(k)) = slot.get(p.index())
                    && !ks.contains(k)
                {
                    ks.push(*k);
                }
            }
            if ks.is_empty() {
                const_blocks.push(bi);
                continue;
            }
            for w in ks.windows(2) {
                cuf.union(w[0] as usize, w[1] as usize);
            }
            touched.push((bi, ks));
        }
        let mut comp_of_root = vec![usize::MAX; nu];
        let mut components: Vec<Component> = Vec::new();
        for (bi, ks) in &touched {
            let r = cuf.find(ks[0] as usize);
            if comp_of_root[r] == usize::MAX {
                comp_of_root[r] = components.len();
                components.push(Component {
                    unknowns: Vec::new(),
                    blocks: Vec::new(),
                    first_row: Vec::new(),
                    cols: Vec::new(),
                    rows: 0,
                });
            }
            components[comp_of_root[r]].blocks.push(*bi);
        }
        for k in 0..nu {
            let r = cuf.find(k);
            if comp_of_root[r] != usize::MAX {
                components[comp_of_root[r]].unknowns.push(k as u32);
            }
        }
        let mut local = vec![u32::MAX; nu];
        for c in &mut components {
            for (i, &k) in c.unknowns.iter().enumerate() {
                local[k as usize] = i as u32;
            }
            let mut row = 0;
            for &bi in &c.blocks {
                let res = sys.system.residual(bi);
                c.first_row.push(row);
                row += res.rows();
                c.cols.push(
                    res.params()
                        .iter()
                        .map(|p| match slot.get(p.index()) {
                            Some(Slot::Unknown(k)) => Some(local[*k as usize]),
                            _ => None,
                        })
                        .collect(),
                );
            }
            c.rows = row;
        }

        Ok(Plan {
            sys,
            slot,
            fixed_value,
            members,
            components,
            const_blocks,
            substituted_rows,
            redundant_subst,
            conflict_subst,
            tol,
        })
    }

    /// Number of unknown classes.
    pub(crate) fn unknowns(&self) -> usize {
        self.members.len()
    }

    /// The full parameter vector for unknown values `u` (all classes), starting from `x0`.
    pub(crate) fn expand(&self, x0: &[f64], u: &[f64]) -> Vec<f64> {
        let mut x = x0.to_vec();
        for (i, s) in self.slot.iter().enumerate() {
            match *s {
                Slot::Unused => {}
                Slot::Fixed => x[i] = self.fixed_value[i],
                Slot::Unknown(k) => x[i] = u[k as usize],
            }
        }
        x
    }

    /// Class values from `x`: the weighted mean of the members (`weight(p)` > 0). This is the
    /// minimum weighted movement that satisfies the substituted equations.
    pub(crate) fn gather(
        &self,
        x: &[f64],
        weight: &dyn Fn(ParamId) -> f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut u = Vec::with_capacity(self.members.len());
        let mut w = Vec::with_capacity(self.members.len());
        for ms in &self.members {
            let (mut sw, mut sx) = (0.0, 0.0);
            for &p in ms {
                let wp = weight(p);
                sw += wp;
                sx += wp * x[p.index()];
            }
            u.push(sx / sw);
            w.push(sw);
        }
        (u, w)
    }

    /// Writes component `c`'s local unknowns `uc` into `x` (all members).
    pub(crate) fn scatter(&self, c: &Component, uc: &[f64], x: &mut [f64]) {
        for (i, &k) in c.unknowns.iter().enumerate() {
            for p in &self.members[k as usize] {
                x[p.index()] = uc[i];
            }
        }
    }

    /// Residuals of component `c` at `x`.
    pub(crate) fn eval(&self, c: &Component, x: &[f64], f: &mut [f64]) {
        for (b, &bi) in c.blocks.iter().enumerate() {
            let res = self.sys.system.residual(bi);
            let r0 = c.first_row[b];
            res.eval(x, &mut f[r0..r0 + res.rows()]);
        }
    }

    /// Dense Jacobian of component `c` at `x` over its local unknowns (`rows × unknowns`).
    pub(crate) fn jacobian(&self, c: &Component, x: &[f64], j: &mut [f64]) {
        let n = c.unknowns.len();
        j[..c.rows * n].fill(0.0);
        let mut block = Vec::new();
        for (b, &bi) in c.blocks.iter().enumerate() {
            let res = self.sys.system.residual(bi);
            let (r, np) = (res.rows(), res.params().len());
            block.clear();
            block.resize(r * np, 0.0);
            res.jacobian(x, &mut block);
            let r0 = c.first_row[b];
            for i in 0..r {
                for (col, v) in c.cols[b].iter().zip(&block[i * np..(i + 1) * np]) {
                    if let Some(col) = col {
                        j[(r0 + i) * n + *col as usize] += v;
                    }
                }
            }
        }
    }

    /// Largest residual over the whole (unsubstituted) system at `x`.
    pub(crate) fn max_residual(&self, x: &[f64]) -> f64 {
        let mut f = vec![0.0; self.sys.system.rows()];
        self.sys.system.eval(x, &mut f);
        max_abs(&f)
    }
}
