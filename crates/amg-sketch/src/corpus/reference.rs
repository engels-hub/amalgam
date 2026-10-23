//! Reference residuals, numeric rank and a slow dense solver: an independent oracle for the
//! corpus (SK-14).
//!
//! This is **not** the production solver (SK-03) and not an ADR-007 candidate. It exists so
//! the corpus can be validated and the runner exercised before SK-02/SK-03 land, and as a
//! cross-check for them later:
//!
//! - Residuals follow the S03 constraint table, written once over forward-mode dual numbers,
//!   so Jacobians are exact without hand-written derivatives (and independent of SK-02's).
//! - Rank comes from Householder QR with column pivoting; `DOF = free params − rank`.
//! - [`ReferenceSolver`] is a damped minimum-norm Gauss–Newton (Levenberg–Marquardt on
//!   `J Jᵀ`), dense, with a fresh Jacobian every iteration.
//!
//! Conventions the corpus relies on (SK-02 should match them; see S03 Interface changes):
//! - Signed point–line distance `cross(d, p − a) / |d|`, positive on the left of the line
//!   oriented `start → end`. `Distance` with `Side::Positive` puts the point (or the circle
//!   centre, or line `b`'s start) on the left.
//! - Point–circle `Distance`: `Side::Positive` is outside (`|p − c| − r = v`), `Negative`
//!   inside (`r − |p − c| = v`). Line–circle: `side · s − r = v` with `s` the signed centre
//!   distance.
//! - Angular and dimensionless rows (parallel, perpendicular, endpoint tangency, angle) are
//!   multiplied by the characteristic length `L_c` (start bbox diagonal, ≥ 1 mm).
//! - `Symmetric` lines pair start↔start and end↔end; circles mirror centres and equal radii;
//!   arcs are not supported here.

use std::collections::BTreeSet;
use std::ops::{Add, Div, Mul, Neg, Sub};

use crate::ids::{ConstraintId, EntityId};
use crate::model::{
    ConstraintKind, DimensionKind, EntityKind, HvTarget, Side, Sketch, TangentKind,
};

use super::runner::{BackendOutcome, BackendSkip, CorpusBackend};
use super::{BuiltCase, Case, CaseStatus, Solution};

// ---- Dual numbers -------------------------------------------------------------------------

/// Forward-mode dual number: value and one directional derivative.
#[derive(Copy, Clone, Debug, PartialEq)]
struct D {
    v: f64,
    d: f64,
}

const fn c(v: f64) -> D {
    D { v, d: 0.0 }
}

impl Add for D {
    type Output = D;
    fn add(self, o: D) -> D {
        D {
            v: self.v + o.v,
            d: self.d + o.d,
        }
    }
}
impl Sub for D {
    type Output = D;
    fn sub(self, o: D) -> D {
        D {
            v: self.v - o.v,
            d: self.d - o.d,
        }
    }
}
impl Mul for D {
    type Output = D;
    fn mul(self, o: D) -> D {
        D {
            v: self.v * o.v,
            d: self.d * o.v + self.v * o.d,
        }
    }
}
impl Div for D {
    type Output = D;
    fn div(self, o: D) -> D {
        D {
            v: self.v / o.v,
            d: (self.d * o.v - self.v * o.d) / (o.v * o.v),
        }
    }
}
impl Neg for D {
    type Output = D;
    fn neg(self) -> D {
        D {
            v: -self.v,
            d: -self.d,
        }
    }
}
impl Mul<f64> for D {
    type Output = D;
    fn mul(self, k: f64) -> D {
        D {
            v: self.v * k,
            d: self.d * k,
        }
    }
}

impl D {
    fn sqrt(self) -> D {
        let s = self.v.sqrt();
        D {
            v: s,
            d: if s > 0.0 { self.d / (2.0 * s) } else { 0.0 },
        }
    }
    fn abs(self) -> D {
        if self.v < 0.0 { -self } else { self }
    }
    fn atan2(self, x: D) -> D {
        let r2 = self.v * self.v + x.v * x.v;
        D {
            v: self.v.atan2(x.v),
            d: if r2 > 0.0 {
                (x.v * self.d - self.v * x.d) / r2
            } else {
                0.0
            },
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct V {
    x: D,
    y: D,
}

impl Sub for V {
    type Output = V;
    fn sub(self, o: V) -> V {
        V {
            x: self.x - o.x,
            y: self.y - o.y,
        }
    }
}
impl Add for V {
    type Output = V;
    fn add(self, o: V) -> V {
        V {
            x: self.x + o.x,
            y: self.y + o.y,
        }
    }
}

impl V {
    fn dot(self, o: V) -> D {
        self.x * o.x + self.y * o.y
    }
    fn cross(self, o: V) -> D {
        self.x * o.y - self.y * o.x
    }
    fn len(self) -> D {
        self.dot(self).sqrt()
    }
    fn half(self) -> V {
        V {
            x: self.x * 0.5,
            y: self.y * 0.5,
        }
    }
    /// CCW perpendicular.
    fn perp(self) -> V {
        V {
            x: -self.y,
            y: self.x,
        }
    }
}

fn side_sign(s: Side) -> f64 {
    match s {
        Side::Positive => 1.0,
        Side::Negative => -1.0,
    }
}

// ---- Equation system ------------------------------------------------------------------------

/// Who owns a residual row.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RowOwner {
    /// A constraint or driving dimension.
    Constraint(ConstraintId),
    /// The internal `|s − c| = |e − c|` equation of an arc.
    Arc(EntityId),
}

/// The reference equation system of a sketch: unknown params, rows and their owners.
#[derive(Clone, Debug)]
pub struct RefSystem {
    /// Unknown parameter indices (not locked, not under `Fix`), in param order.
    pub unknowns: Vec<usize>,
    /// Owner of each row.
    pub owners: Vec<RowOwner>,
    /// Characteristic length (start bbox diagonal, ≥ 1 mm).
    pub lc: f64,
}

struct Ctx<'a> {
    s: &'a Sketch,
    x: &'a [D],
    lc: f64,
}

type R<T> = Result<T, String>;

impl Ctx<'_> {
    fn kind(&self, id: EntityId) -> R<&EntityKind> {
        self.s
            .entity(id)
            .map(|e| &e.kind)
            .ok_or_else(|| format!("unknown entity {}", id.0))
    }
    fn pt(&self, id: EntityId) -> R<V> {
        match *self.kind(id)? {
            EntityKind::Point { x, y } => Ok(V {
                x: self.x[x.index()],
                y: self.x[y.index()],
            }),
            ref k => Err(format!("entity {} is a {}, not a point", id.0, k.name())),
        }
    }
    fn line(&self, id: EntityId) -> R<(V, V)> {
        match *self.kind(id)? {
            EntityKind::Line { start, end } => Ok((self.pt(start)?, self.pt(end)?)),
            ref k => Err(format!("entity {} is a {}, not a line", id.0, k.name())),
        }
    }
    /// Centre and radius of a circle or arc.
    fn round(&self, id: EntityId) -> R<(V, D)> {
        match *self.kind(id)? {
            EntityKind::Circle { center, radius } => Ok((self.pt(center)?, self.x[radius.index()])),
            EntityKind::Arc { center, start, .. } => {
                let c = self.pt(center)?;
                Ok((c, (self.pt(start)? - c).len()))
            }
            ref k => Err(format!(
                "entity {} is a {}, not a circle or arc",
                id.0,
                k.name()
            )),
        }
    }
    fn arc(&self, id: EntityId) -> R<(V, V, V)> {
        match *self.kind(id)? {
            EntityKind::Arc { center, start, end } => {
                Ok((self.pt(center)?, self.pt(start)?, self.pt(end)?))
            }
            ref k => Err(format!("entity {} is a {}, not an arc", id.0, k.name())),
        }
    }
    fn is(&self, id: EntityId, f: fn(&EntityKind) -> bool) -> bool {
        self.kind(id).is_ok_and(f)
    }
}

fn is_point(k: &EntityKind) -> bool {
    matches!(k, EntityKind::Point { .. })
}
fn is_line(k: &EntityKind) -> bool {
    matches!(k, EntityKind::Line { .. })
}

/// Signed distance of `p` from the line `(a, b)`, positive on the left.
fn signed(p: V, (a, b): (V, V)) -> D {
    let d = b - a;
    d.cross(p - a) / d.len()
}

/// CCW sweep from `s` to `e` about `c`, in `(0, 2π]`.
fn sweep(c0: V, s: V, e: V) -> D {
    let (u, w) = (s - c0, e - c0);
    let t = u.cross(w).atan2(u.dot(w));
    if t.v <= 0.0 {
        t + c(std::f64::consts::TAU)
    } else {
        t
    }
}

/// Direction of travel of a line or arc at one of its ends.
fn travel(ctx: &Ctx<'_>, id: EntityId, end: crate::model::End) -> R<V> {
    match *ctx.kind(id)? {
        EntityKind::Line { .. } => {
            let (a, b) = ctx.line(id)?;
            Ok(b - a)
        }
        EntityKind::Arc { .. } => {
            let (c0, s, e) = ctx.arc(id)?;
            let p = match end {
                crate::model::End::Start => s,
                crate::model::End::End => e,
            };
            Ok((p - c0).perp())
        }
        ref k => Err(format!(
            "entity {} is a {}, not a line or arc",
            id.0,
            k.name()
        )),
    }
}

fn wrap_angle(a: D) -> D {
    let tau = std::f64::consts::TAU;
    let k = ((a.v + std::f64::consts::PI) / tau).floor();
    // (−π, π]: shift so that exactly −π maps to +π.
    let mut r = a - c(k * tau);
    if r.v <= -std::f64::consts::PI {
        r = r + c(tau);
    }
    r
}

fn constraint_rows(ctx: &Ctx<'_>, kind: &ConstraintKind, out: &mut Vec<D>) -> R<()> {
    use ConstraintKind as C;
    let lc = ctx.lc;
    match *kind {
        C::Coincident { a, b } => {
            let d = ctx.pt(a)? - ctx.pt(b)?;
            out.extend([d.x, d.y]);
        }
        C::PointOnCurve { point, curve } => {
            let p = ctx.pt(point)?;
            if ctx.is(curve, is_line) {
                out.push(signed(p, ctx.line(curve)?));
            } else {
                let (c0, r) = ctx.round(curve)?;
                out.push((p - c0).len() - r);
            }
        }
        C::Midpoint { point, line } => {
            let p = ctx.pt(point)?;
            let (a, b) = ctx.line(line)?;
            out.extend([p.x * 2.0 - a.x - b.x, p.y * 2.0 - a.y - b.y]);
        }
        C::Horizontal(t) | C::Vertical(t) => {
            let (a, b) = match t {
                HvTarget::Line(l) => ctx.line(l)?,
                HvTarget::Points(a, b) => (ctx.pt(a)?, ctx.pt(b)?),
            };
            out.push(if matches!(kind, C::Horizontal(_)) {
                a.y - b.y
            } else {
                a.x - b.x
            });
        }
        C::Parallel { a, b } | C::Perpendicular { a, b } => {
            let (a0, a1) = ctx.line(a)?;
            let (b0, b1) = ctx.line(b)?;
            let (da, db) = (a1 - a0, b1 - b0);
            let num = if matches!(kind, C::Parallel { .. }) {
                da.cross(db)
            } else {
                da.dot(db)
            };
            out.push(num / (da.len() * db.len()) * lc);
        }
        C::Collinear { a, b } => {
            let la = ctx.line(a)?;
            let (b0, b1) = ctx.line(b)?;
            out.extend([signed(b0, la), signed(b1, la)]);
        }
        C::Tangent { a, b, kind: tk } => match tk {
            TangentKind::External | TangentKind::Internal => {
                let ext = tk == TangentKind::External;
                if ctx.is(a, is_line) || ctx.is(b, is_line) {
                    let (l, r) = if ctx.is(a, is_line) { (a, b) } else { (b, a) };
                    let (c0, rad) = ctx.round(r)?;
                    let s = signed(c0, ctx.line(l)?);
                    out.push(if ext { s - rad } else { s + rad });
                } else {
                    let (c1, r1) = ctx.round(a)?;
                    let (c2, r2) = ctx.round(b)?;
                    let d = (c1 - c2).len();
                    out.push(if ext {
                        d - (r1 + r2)
                    } else {
                        d - (r1 - r2).abs()
                    });
                }
            }
            TangentKind::AtEndpoint { a_end, b_end, .. } => {
                let ta = travel(ctx, a, a_end)?;
                let tb = travel(ctx, b, b_end)?;
                out.push(ta.cross(tb) / (ta.len() * tb.len()) * lc);
            }
        },
        C::Concentric { a, b } => {
            let d = ctx.round(a)?.0 - ctx.round(b)?.0;
            out.extend([d.x, d.y]);
        }
        C::Equal { a, b } => {
            let len = |id: EntityId| -> R<D> {
                match *ctx.kind(id)? {
                    EntityKind::Line { .. } => {
                        let (p, q) = ctx.line(id)?;
                        Ok((q - p).len())
                    }
                    EntityKind::Arc { .. } => {
                        let (c0, s, e) = ctx.arc(id)?;
                        Ok((s - c0).len() * sweep(c0, s, e))
                    }
                    ref k => Err(format!("cannot take the length of a {}", k.name())),
                }
            };
            let lengths = ctx.is(a, is_line) || ctx.is(b, is_line);
            if lengths {
                out.push(len(a)? - len(b)?);
            } else {
                out.push(ctx.round(a)?.1 - ctx.round(b)?.1);
            }
        }
        C::Symmetric { a, b, axis } => {
            let ax = ctx.line(axis)?;
            let d = ax.1 - ax.0;
            let dl = d.len();
            let pair = |p: V, q: V, out: &mut Vec<D>| {
                out.push(signed((p + q).half(), ax));
                out.push((q - p).dot(d) / dl);
            };
            match *ctx.kind(a)? {
                EntityKind::Point { .. } => pair(ctx.pt(a)?, ctx.pt(b)?, out),
                EntityKind::Line { .. } => {
                    let (a0, a1) = ctx.line(a)?;
                    let (b0, b1) = ctx.line(b)?;
                    pair(a0, b0, out);
                    pair(a1, b1, out);
                }
                EntityKind::Circle { .. } => {
                    let (c1, r1) = ctx.round(a)?;
                    let (c2, r2) = ctx.round(b)?;
                    pair(c1, c2, out);
                    out.push(r1 - r2);
                }
                EntityKind::Arc { .. } => {
                    return Err("symmetric arcs are not supported by the reference".into());
                }
                EntityKind::Ellipse { .. } | EntityKind::EllipticArc { .. } => {
                    return Err("ellipses are not modelled by the reference".into());
                }
            }
        }
        C::Fix { .. } => {}
        C::Dimension(ref dim) => {
            if !dim.driving {
                return Ok(());
            }
            let v = c(dim.value);
            use DimensionKind as Dk;
            let row = match dim.kind {
                Dk::Distance { a, b, side } => {
                    let k = side_sign(side);
                    let (pa, pb) = (ctx.is(a, is_point), ctx.is(b, is_point));
                    if pa && pb {
                        (ctx.pt(a)? - ctx.pt(b)?).len() - v
                    } else if pa || pb {
                        let (p, o) = if pa { (a, b) } else { (b, a) };
                        let p = ctx.pt(p)?;
                        if ctx.is(o, is_line) {
                            signed(p, ctx.line(o)?) * k - v
                        } else {
                            let (c0, r) = ctx.round(o)?;
                            ((p - c0).len() - r) * k - v
                        }
                    } else if ctx.is(a, is_line) && ctx.is(b, is_line) {
                        let (b0, _) = ctx.line(b)?;
                        signed(b0, ctx.line(a)?) * k - v
                    } else {
                        let (l, r) = if ctx.is(a, is_line) { (a, b) } else { (b, a) };
                        let (c0, rad) = ctx.round(r)?;
                        signed(c0, ctx.line(l)?) * k - rad - v
                    }
                }
                Dk::HorizontalDistance { a, b, side } => {
                    (ctx.pt(b)?.x - ctx.pt(a)?.x) * side_sign(side) - v
                }
                Dk::VerticalDistance { a, b, side } => {
                    (ctx.pt(b)?.y - ctx.pt(a)?.y) * side_sign(side) - v
                }
                Dk::Length { line } => {
                    let (p, q) = ctx.line(line)?;
                    (q - p).len() - v
                }
                Dk::Angle { a, b, branch } => {
                    let (a0, a1) = ctx.line(a)?;
                    let (b0, b1) = ctx.line(b)?;
                    let da = if branch.reverse_a { a0 - a1 } else { a1 - a0 };
                    let db = if branch.reverse_b { b0 - b1 } else { b1 - b0 };
                    wrap_angle(da.cross(db).atan2(da.dot(db)) - v) * lc
                }
                Dk::Radius { curve } => ctx.round(curve)?.1 - v,
                Dk::Diameter { curve } => ctx.round(curve)?.1 * 2.0 - v,
                Dk::ArcLength { arc } => {
                    let (c0, s, e) = ctx.arc(arc)?;
                    (s - c0).len() * sweep(c0, s, e) - v
                }
                Dk::EllipseMajor { .. } | Dk::EllipseMinor { .. } => {
                    return Err("ellipse dimensions are not modelled by the reference".into());
                }
            };
            out.push(row);
        }
    }
    Ok(())
}

impl RefSystem {
    /// Builds the system for `sketch`. Errors name constructs the reference does not model.
    pub fn new(sketch: &Sketch) -> Result<RefSystem, String> {
        let mut fixed = BTreeSet::new();
        for (_, con) in sketch.constraints() {
            if let ConstraintKind::Fix { entity } = con.kind {
                let e = sketch
                    .entity(entity)
                    .ok_or_else(|| format!("fix of unknown entity {}", entity.0))?;
                fixed.extend(e.kind.own_params().iter().map(|p| p.index()));
                for ch in e.kind.children() {
                    if let Some(c) = sketch.entity(ch) {
                        fixed.extend(c.kind.own_params().iter().map(|p| p.index()));
                    }
                }
            }
        }
        let params = sketch.params();
        let mut unknowns = Vec::new();
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for (_, e) in sketch.entities() {
            for p in e.kind.own_params() {
                if !params.flags(p).locked && !fixed.contains(&p.index()) {
                    unknowns.push(p.index());
                }
            }
            let mut grow = |x: f64, y: f64| {
                lo = [lo[0].min(x), lo[1].min(y)];
                hi = [hi[0].max(x), hi[1].max(y)];
            };
            match e.kind {
                EntityKind::Point { x, y } => grow(params.get(x), params.get(y)),
                EntityKind::Circle { center, radius } => {
                    if let Some(c0) = sketch.point(center) {
                        let r = params.get(radius);
                        grow(c0.x - r, c0.y - r);
                        grow(c0.x + r, c0.y + r);
                    }
                }
                _ => {}
            }
        }
        unknowns.sort_unstable();
        unknowns.dedup();
        let lc = if lo[0].is_finite() {
            ((hi[0] - lo[0]).hypot(hi[1] - lo[1])).max(1.0)
        } else {
            1.0
        };
        let mut sys = RefSystem {
            unknowns,
            owners: Vec::new(),
            lc,
        };
        let x: Vec<D> = params.as_slice().iter().map(|&v| c(v)).collect();
        let mut out = Vec::new();
        sys.owners = sys.eval_owned(sketch, &x, &mut out)?;
        Ok(sys)
    }

    fn eval_owned(&self, sketch: &Sketch, x: &[D], out: &mut Vec<D>) -> R<Vec<RowOwner>> {
        out.clear();
        let mut owners = Vec::new();
        let ctx = Ctx {
            s: sketch,
            x,
            lc: self.lc,
        };
        for (id, e) in sketch.entities() {
            match e.kind {
                EntityKind::Arc { .. } => {
                    let (c0, s, en) = ctx.arc(id)?;
                    out.push((s - c0).len() - (en - c0).len());
                    owners.push(RowOwner::Arc(id));
                }
                EntityKind::Ellipse { .. } | EntityKind::EllipticArc { .. } => {
                    return Err("ellipses are not modelled by the reference".into());
                }
                EntityKind::Point { .. } | EntityKind::Line { .. } | EntityKind::Circle { .. } => {}
            }
        }
        for (id, con) in sketch.constraints() {
            let before = out.len();
            constraint_rows(&ctx, &con.kind, out)?;
            owners.extend(std::iter::repeat_n(
                RowOwner::Constraint(id),
                out.len() - before,
            ));
        }
        Ok(owners)
    }

    /// Number of rows.
    pub fn rows(&self) -> usize {
        self.owners.len()
    }

    fn full(&self, sketch: &Sketch, u: &[f64]) -> Vec<f64> {
        let mut x = sketch.params().as_slice().to_vec();
        for (k, &i) in self.unknowns.iter().enumerate() {
            x[i] = u[k];
        }
        x
    }

    /// The unknowns' current values.
    pub fn current(&self, sketch: &Sketch) -> Vec<f64> {
        let p = sketch.params().as_slice();
        self.unknowns.iter().map(|&i| p[i]).collect()
    }

    /// Writes unknown values back into the sketch.
    pub fn store(&self, sketch: &mut Sketch, u: &[f64]) {
        let ps = sketch.params_mut();
        for (k, &i) in self.unknowns.iter().enumerate() {
            ps.set(crate::ids::ParamId(i as u32), u[k]);
        }
    }

    /// Residuals at unknowns `u`.
    pub fn residuals(&self, sketch: &Sketch, u: &[f64]) -> R<Vec<f64>> {
        let x: Vec<D> = self.full(sketch, u).into_iter().map(c).collect();
        let mut out = Vec::new();
        self.eval_owned(sketch, &x, &mut out)?;
        Ok(out.into_iter().map(|d| d.v).collect())
    }

    /// Exact Jacobian at `u`, row-major `rows × unknowns`.
    pub fn jacobian(&self, sketch: &Sketch, u: &[f64]) -> R<Vec<f64>> {
        let (m, n) = (self.rows(), self.unknowns.len());
        let mut jac = vec![0.0; m * n];
        let mut x: Vec<D> = self.full(sketch, u).into_iter().map(c).collect();
        let mut out = Vec::new();
        for (k, &i) in self.unknowns.iter().enumerate() {
            x[i].d = 1.0;
            self.eval_owned(sketch, &x, &mut out)?;
            x[i].d = 0.0;
            for (r, d) in out.iter().enumerate() {
                jac[r * n + k] = d.d;
            }
        }
        Ok(jac)
    }

    /// Residual tolerance for a converged solve: `max(1e-10, 64 ε L_c)` mm.
    pub fn solve_tolerance(&self) -> f64 {
        (64.0 * f64::EPSILON * self.lc).max(1e-10)
    }
}

/// Numeric rank of a row-major `m × n` matrix (Householder QR with column pivoting).
/// Relative tolerance `1e-9 · max(m, n) · max|a|`.
pub fn rank(a: &[f64], m: usize, n: usize) -> usize {
    let mut a = a.to_vec();
    let amax = a.iter().fold(0.0_f64, |acc, v| acc.max(v.abs()));
    if amax == 0.0 || m == 0 || n == 0 {
        return 0;
    }
    let tol = 1e-9 * (m.max(n) as f64) * amax;
    let mut cols: Vec<usize> = (0..n).collect();
    let mut r = 0;
    for k in 0..m.min(n) {
        // Pivot: the remaining column with the largest norm below row k.
        let norm = |a: &[f64], j: usize| -> f64 {
            (k..m)
                .map(|i| a[i * n + j] * a[i * n + j])
                .sum::<f64>()
                .sqrt()
        };
        let (best, bn) = (k..n)
            .map(|j| (j, norm(&a, cols[j])))
            .fold((k, -1.0), |acc, x| if x.1 > acc.1 { x } else { acc });
        if bn <= tol {
            break;
        }
        cols.swap(k, best);
        let j = cols[k];
        // Householder vector for column j, rows k..m.
        let alpha = if a[k * n + j] > 0.0 { -bn } else { bn };
        let mut v: Vec<f64> = (k..m).map(|i| a[i * n + j]).collect();
        v[0] -= alpha;
        let vn2: f64 = v.iter().map(|x| x * x).sum();
        if vn2 > 0.0 {
            for &jj in &cols[k..] {
                let dot: f64 = (k..m).map(|i| v[i - k] * a[i * n + jj]).sum();
                let f = 2.0 * dot / vn2;
                for i in k..m {
                    a[i * n + jj] -= f * v[i - k];
                }
            }
        }
        r += 1;
    }
    r
}

/// Status and DOF from a residual vector and Jacobian (shared by both reference backends).
fn classify(sys: &RefSystem, jac: &[f64], satisfied: bool) -> (CaseStatus, u32) {
    let (m, n) = (sys.rows(), sys.unknowns.len());
    let r = rank(jac, m, n);
    let dof = u32::try_from(n - r).unwrap_or(u32::MAX);
    let status = match (satisfied, r < m) {
        (true, false) => CaseStatus::Converged,
        (true, true) => CaseStatus::ConvergedRedundant,
        (false, true) => CaseStatus::Inconsistent,
        (false, false) => CaseStatus::DidNotConverge,
    };
    (status, dof)
}

fn max_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0_f64, |a, x| a.max(x.abs()))
}

/// Checks the case's known solution without solving: writes it into the sketch, evaluates the
/// reference residuals and reports status and DOF there. Cases without a solution are skipped.
///
/// This validates the corpus itself (every stated solution satisfies its constraints, and the
/// stated DOF matches the rank there). It is the "solver" the runner uses before SK-03.
#[derive(Clone, Debug)]
pub struct ReferenceCheck {
    /// Residual tolerance at the stored solution, relative: `tol · max(1, L_c)` mm.
    /// Default 1e-9 (stored solutions are rounded and SolveSpace converges to ~1e-8 mm).
    pub tolerance: f64,
}

impl Default for ReferenceCheck {
    fn default() -> Self {
        ReferenceCheck { tolerance: 1e-9 }
    }
}

impl CorpusBackend for ReferenceCheck {
    fn name(&self) -> String {
        "reference-check".into()
    }

    fn solve(&mut self, case: &Case, built: &mut BuiltCase) -> Result<BackendOutcome, BackendSkip> {
        match &case.expected.solution {
            None => return Err(BackendSkip("no stored solution".into())),
            Some(Solution::Start) => {}
            Some(Solution::Target(v) | Solution::Witness(v)) => {
                built.apply(v).map_err(BackendSkip)?;
            }
        }
        let sys = RefSystem::new(&built.sketch).map_err(BackendSkip)?;
        let u = sys.current(&built.sketch);
        let res = sys.residuals(&built.sketch, &u).map_err(BackendSkip)?;
        let jac = sys.jacobian(&built.sketch, &u).map_err(BackendSkip)?;
        let worst = max_abs(&res);
        let satisfied = worst <= self.tolerance * sys.lc.max(1.0);
        let (status, dof) = classify(&sys, &jac, satisfied);
        Ok(BackendOutcome {
            status,
            dof: Some(dof),
            max_residual: Some(worst),
            iterations: Some(0),
            groups: None,
        })
    }
}

/// Dense damped minimum-norm Gauss–Newton (Levenberg–Marquardt on `J Jᵀ`) with exact
/// Jacobians. Slow (`O(n)` residual sweeps per Jacobian, dense `m × m` Cholesky); meant for
/// corpus validation and as a baseline backend, not for production.
#[derive(Clone, Debug)]
pub struct ReferenceSolver {
    /// Iteration cap (accepted and rejected steps).
    pub max_iterations: u32,
}

impl Default for ReferenceSolver {
    fn default() -> Self {
        ReferenceSolver {
            max_iterations: 200,
        }
    }
}

/// Solves `(A + λI) z = f` for symmetric positive definite `A` (`m × m`); `None` if the
/// Cholesky factorisation breaks down.
fn chol_solve(a: &[f64], m: usize, lambda: f64, f: &[f64]) -> Option<Vec<f64>> {
    let mut l = vec![0.0; m * m];
    let dmax = (0..m)
        .fold(0.0_f64, |acc, i| acc.max(a[i * m + i]))
        .max(1.0);
    for i in 0..m {
        for j in 0..=i {
            let mut s = a[i * m + j] + if i == j { lambda } else { 0.0 };
            for k in 0..j {
                s -= l[i * m + k] * l[j * m + k];
            }
            if i == j {
                if s <= 1e-13 * dmax {
                    return None;
                }
                l[i * m + i] = s.sqrt();
            } else {
                l[i * m + j] = s / l[j * m + j];
            }
        }
    }
    let mut y = f.to_vec();
    for i in 0..m {
        for k in 0..i {
            y[i] -= l[i * m + k] * y[k];
        }
        y[i] /= l[i * m + i];
    }
    for i in (0..m).rev() {
        for k in i + 1..m {
            y[i] -= l[k * m + i] * y[k];
        }
        y[i] /= l[i * m + i];
    }
    Some(y)
}

impl ReferenceSolver {
    /// Solves `sketch` in place. Returns `(status, dof, max |residual|, iterations)`.
    pub fn solve_sketch(&self, sketch: &mut Sketch) -> Result<(CaseStatus, u32, f64, u32), String> {
        let sys = RefSystem::new(sketch)?;
        let (m, n) = (sys.rows(), sys.unknowns.len());
        let tol = sys.solve_tolerance();
        let mut u = sys.current(sketch);
        let mut f = sys.residuals(sketch, &u)?;
        let norm2 = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>();
        let mut lambda = 0.0_f64;
        let mut iters = 0;
        while max_abs(&f) > tol && iters < self.max_iterations {
            iters += 1;
            let jac = sys.jacobian(sketch, &u)?;
            // A = J Jᵀ (m × m).
            let mut a = vec![0.0; m * m];
            for i in 0..m {
                for j in 0..=i {
                    let s: f64 = (0..n).map(|k| jac[i * n + k] * jac[j * n + k]).sum();
                    a[i * m + j] = s;
                    a[j * m + i] = s;
                }
            }
            let dmax = (0..m)
                .fold(0.0_f64, |acc, i| acc.max(a[i * m + i]))
                .max(1e-300);
            let floor = 1e-12 * dmax;
            let mut accepted = false;
            while iters <= self.max_iterations {
                if let Some(z) = chol_solve(&a, m, lambda, &f) {
                    let un: Vec<f64> = (0..n)
                        .map(|k| u[k] - (0..m).map(|i| jac[i * n + k] * z[i]).sum::<f64>())
                        .collect();
                    let fnew = sys.residuals(sketch, &un)?;
                    if fnew.iter().all(|x| x.is_finite()) && norm2(&fnew) < norm2(&f) {
                        u = un;
                        f = fnew;
                        lambda = if lambda <= floor { 0.0 } else { lambda / 10.0 };
                        accepted = true;
                        break;
                    }
                }
                lambda = (lambda * 10.0).max(floor);
                iters += 1;
                if lambda > 1e12 * dmax {
                    break;
                }
            }
            if !accepted {
                break;
            }
        }
        sys.store(sketch, &u);
        let jac = sys.jacobian(sketch, &u)?;
        let worst = max_abs(&f);
        let (status, dof) = classify(&sys, &jac, worst <= tol);
        Ok((status, dof, worst, iters))
    }
}

impl CorpusBackend for ReferenceSolver {
    fn name(&self) -> String {
        "reference-solver".into()
    }

    fn solve(
        &mut self,
        _case: &Case,
        built: &mut BuiltCase,
    ) -> Result<BackendOutcome, BackendSkip> {
        let (status, dof, worst, iterations) =
            self.solve_sketch(&mut built.sketch).map_err(BackendSkip)?;
        Ok(BackendOutcome {
            status,
            dof: Some(dof),
            max_residual: Some(worst),
            iterations: Some(iterations),
            groups: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rank_of_small_matrices() {
        assert_eq!(rank(&[1.0, 0.0, 0.0, 1.0], 2, 2), 2);
        assert_eq!(rank(&[1.0, 2.0, 2.0, 4.0], 2, 2), 1);
        assert_eq!(rank(&[0.0; 6], 2, 3), 0);
        assert_eq!(
            rank(&[1.0, 2.0, 3.0, 2.0, 4.0, 6.0, 1.0, 0.0, 0.0], 3, 3),
            2
        );
    }

    #[test]
    fn dual_derivatives() {
        let x = D { v: 3.0, d: 1.0 };
        let y = c(4.0);
        let r = (x * x + y * y).sqrt();
        assert!((r.v - 5.0).abs() < 1e-15 && (r.d - 0.6).abs() < 1e-15);
        let a = y.atan2(x);
        assert!((a.d + 4.0 / 25.0).abs() < 1e-15);
        let w = wrap_angle(c(3.0 * std::f64::consts::PI));
        assert!((w.v - std::f64::consts::PI).abs() < 1e-12);
    }
}
