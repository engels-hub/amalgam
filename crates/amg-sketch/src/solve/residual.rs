//! Residual equations of sketch entities, constraints and dimensions (SK-02).
//!
//! Conventions (S03 §Constraints and dimensions, `docs/notes/sketch-solver.md` §3.2):
//! - `d = end − start` of a line; "signed distance" is `cross(d, p − start)/|d|`, positive on
//!   the left of the oriented line.
//! - Length rows are in mm. **Angular rows** (parallel, perpendicular, endpoint tangency,
//!   angle dimension) are dimensionless or radians and are multiplied by the characteristic
//!   length `L_c` ([`characteristic_length`]), so all rows share the unit mm.
//! - Dimension rows are `measure − value` with the stored `Side` sign applied to the measure.
//! - Chirality: `Side`, `TangentKind` and `AngleBranch` are part of the equation, so mirrored
//!   configurations are not roots (see [`orient_constraint`](super::orient_constraint)).
//!
//! | Constraint | Rows |
//! |---|---|
//! | Arc (internal) | `|s − c| − |e − c|` |
//! | Elliptical arc (internal) | `s − P(t0)`, `e − P(t1)` (x, y each) |
//! | Coincident | `a − b` (x, y) |
//! | PointOnCurve | line: signed distance; circle: `|p − c| − r`; arc: `|p − c| − |s − c|`; ellipse: `p − P(t)` (x, y; `t` = the constraint's aux param) |
//! | Midpoint | `2p − a − b` (x, y) |
//! | Horizontal / Vertical | `a_y − b_y` / `a_x − b_x` (line: start, end) |
//! | Parallel / Perpendicular | `L_c · cross(d1, d2)/(|d1||d2|)` / `L_c · dot(..)/(..)` |
//! | Collinear | signed distance of line 2's start and end from line 1 |
//! | Tangent line–round | `s − r` (External) / `s + r` (Internal), `s` = signed centre distance |
//! | Tangent round–round | `|c1 − c2| − (r1 + r2)` / `|c1 − c2| − |r1 − r2|` |
//! | Tangent line–ellipse | `s − h` (External) / `s + h` (Internal), `s` = signed centre distance, `h` = half-width of the ellipse normal to the line |
//! | Ellipse major / minor (dimension) | `|M − c| − v` / `b − v` |
//! | Tangent AtEndpoint | `L_c · wrap(angle(t_a, σ t_b))`, `σ = −1` if `reversed` (line, arc, spline or conic ends; see [`spline`](super::spline)) |
//! | Curvature (G2) | the tangent row and `½ L_c² (κ_a − σ κ_b)`, `κ` the signed curvature at the shared end |
//! | PointOnCurve spline / conic | `p − C(t)` (x, y; `t` = the constraint's aux param) |
//! | Equal conic–conic / rational spline–spline | `ρ_a − ρ_b` / `w_a,i − w_b,i` |
//! | Concentric | `c1 − c2` (x, y) |
//! | Equal | `|d1| − |d2|`; line–arc `|d| − r·sweep`; round–round `r1 − r2`; ellipse–ellipse `|M1 − c1| − |M2 − c2|` and `b1 − b2` |
//! | Symmetric | see [`constraint_residual`] |
//! | Fix | no rows (params locked, [`fixed_params`]) |
//!
//! An arc's radius is `|start − center|`.

use std::collections::BTreeSet;
use std::f64::consts::PI;

use smallvec::SmallVec;

use super::func::{Dir, Func, MAX_ARITY, Rad, wrap_pi};
use super::spline::{Side as EndSide, Spl, SplineRes, WSrc};
use super::system::{EquationOwner, EquationSystem, Residual};
use crate::error::SketchError;
use crate::geom::{Curve2, EllipseArc2, FitMap};
use crate::ids::{EntityId, ParamId};
use crate::model::{
    ConstraintKind, DimensionKind, EntityKind, HvTarget, Side, Sketch, TangentKind,
};

type P2 = [ParamId; 2];

#[derive(Copy, Clone, Debug)]
struct Term {
    func: Func,
    coef: f64,
    /// First local slot; the term reads `func.arity()` consecutive slots.
    first: u16,
}

#[derive(Clone, Debug)]
struct Row {
    terms: SmallVec<[Term; 3]>,
    offset: f64,
    /// Wrap `Σ terms + offset` to (−π, π] before scaling (angle rows).
    wrap: bool,
    scale: f64,
}

/// The residual block of one sketch entity, constraint or dimension. Implements
/// [`Residual`]: `x` is the whole sketch parameter vector ([`crate::ParamStore::as_slice`]).
#[derive(Clone, Debug)]
pub struct SketchResidual {
    /// Distinct params, sorted by id: the Jacobian columns.
    params: SmallVec<[ParamId; 8]>,
    /// Local slots read by the terms (a param may appear in several slots).
    slots: SmallVec<[ParamId; 16]>,
    /// Column of each slot in `params`.
    cols: SmallVec<[u16; 16]>,
    rows: SmallVec<[Row; 2]>,
    /// Spline and conic blocks (variable arity) replace the term rows.
    custom: Option<SplineRes>,
}

impl SketchResidual {
    fn custom(r: SplineRes) -> SketchResidual {
        SketchResidual {
            params: SmallVec::new(),
            slots: SmallVec::new(),
            cols: SmallVec::new(),
            rows: SmallVec::new(),
            custom: Some(r),
        }
    }

    fn gather(&self, x: &[f64], t: &Term, buf: &mut [f64; MAX_ARITY]) {
        let first = usize::from(t.first);
        for (k, slot) in self.slots[first..first + t.func.arity()].iter().enumerate() {
            buf[k] = x[slot.index()];
        }
    }

    fn row_value(&self, x: &[f64], row: &Row) -> f64 {
        let mut buf = [0.0; MAX_ARITY];
        let mut g = [0.0; MAX_ARITY];
        let mut sum = row.offset;
        for t in &row.terms {
            self.gather(x, t, &mut buf);
            sum += t.coef * t.func.eval(&buf, &mut g);
        }
        if row.wrap {
            sum = wrap_pi(sum);
        }
        row.scale * sum
    }
}

impl Residual for SketchResidual {
    fn rows(&self) -> usize {
        self.custom.as_ref().map_or(self.rows.len(), Residual::rows)
    }

    fn params(&self) -> &[ParamId] {
        match &self.custom {
            Some(c) => c.params(),
            None => &self.params,
        }
    }

    fn eval(&self, x: &[f64], out: &mut [f64]) {
        if let Some(c) = &self.custom {
            c.eval(x, out);
            return;
        }
        for (o, row) in out.iter_mut().zip(&self.rows) {
            *o = self.row_value(x, row);
        }
    }

    fn jacobian(&self, x: &[f64], out: &mut [f64]) {
        if let Some(c) = &self.custom {
            c.jacobian(x, out);
            return;
        }
        let n = self.params.len();
        out[..self.rows.len() * n].fill(0.0);
        let mut buf = [0.0; MAX_ARITY];
        let mut g = [0.0; MAX_ARITY];
        for (i, row) in self.rows.iter().enumerate() {
            let out_row = &mut out[i * n..(i + 1) * n];
            for t in &row.terms {
                self.gather(x, t, &mut buf);
                t.func.eval(&buf, &mut g);
                let first = usize::from(t.first);
                let k = row.scale * t.coef;
                for (s, gk) in g[..t.func.arity()].iter().enumerate() {
                    out_row[usize::from(self.cols[first + s])] += k * gk;
                }
            }
        }
    }
}

// ---- Builder ---------------------------------------------------------------------------

#[derive(Default)]
struct Builder {
    slots: SmallVec<[ParamId; 16]>,
    rows: SmallVec<[Row; 2]>,
    terms: SmallVec<[Term; 3]>,
}

impl Builder {
    /// Adds a term of the row being built.
    fn term(&mut self, func: Func, coef: f64, slots: &[ParamId]) -> &mut Self {
        debug_assert_eq!(slots.len(), func.arity());
        let first = u16::try_from(self.slots.len()).unwrap_or(u16::MAX);
        self.slots.extend_from_slice(slots);
        self.terms.push(Term { func, coef, first });
        self
    }

    /// Finishes the current row.
    fn row(&mut self, offset: f64, wrap: bool, scale: f64) -> &mut Self {
        let terms = std::mem::take(&mut self.terms);
        self.rows.push(Row {
            terms,
            offset,
            wrap,
            scale,
        });
        self
    }

    /// Finishes a plain mm row.
    fn mm(&mut self, offset: f64) -> &mut Self {
        self.row(offset, false, 1.0)
    }

    fn finish(self) -> SketchResidual {
        let mut params: SmallVec<[ParamId; 8]> = self.slots.iter().copied().collect();
        params.sort_unstable();
        params.dedup();
        let cols = self
            .slots
            .iter()
            .map(|p| {
                let c = params.binary_search(p).unwrap_or(0);
                u16::try_from(c).unwrap_or(u16::MAX)
            })
            .collect();
        SketchResidual {
            params,
            slots: self.slots,
            cols,
            rows: self.rows,
            custom: None,
        }
    }
}

fn cat<const N: usize>(parts: &[&[ParamId]]) -> [ParamId; N] {
    let mut out = [ParamId(0); N];
    let mut i = 0;
    for p in parts {
        out[i..i + p.len()].copy_from_slice(p);
        i += p.len();
    }
    debug_assert_eq!(i, N);
    out
}

fn sign(side: Side) -> f64 {
    match side {
        Side::Positive => 1.0,
        Side::Negative => -1.0,
    }
}

#[derive(Copy, Clone)]
enum Round {
    Circle { c: P2, r: ParamId },
    Arc { c: P2, s: P2, e: P2 },
}

impl Round {
    fn center(self) -> P2 {
        match self {
            Round::Circle { c, .. } | Round::Arc { c, .. } => c,
        }
    }

    /// How the radius is read, and its slots.
    fn radius_slots(self) -> (Rad, SmallVec<[ParamId; 4]>) {
        match self {
            Round::Circle { r, .. } => (Rad::Param, [r].into_iter().collect()),
            Round::Arc { c, s, .. } => (Rad::Dist, cat::<4>(&[&s, &c]).into_iter().collect()),
        }
    }

    /// Adds `coef · radius` to the current row.
    fn radius(self, b: &mut Builder, coef: f64) {
        match self {
            Round::Circle { r, .. } => b.term(Func::Coord, coef, &[r]),
            Round::Arc { c, s, .. } => b.term(Func::Dist, coef, &cat::<4>(&[&s, &c])),
        };
    }
}

/// Endpoints of a line: `[start, end]`.
type Seg = [P2; 2];

/// An ellipse: centre, major-axis end point and minor radius param. An elliptical arc also
/// carries its end points and eccentric angle params.
#[derive(Copy, Clone)]
struct Ell {
    c: P2,
    m: P2,
    b: ParamId,
    arc: Option<EllArc>,
}

#[derive(Copy, Clone)]
struct EllArc {
    s: P2,
    e: P2,
    t0: ParamId,
    t1: ParamId,
}

/// Slots of `Func::EllipseOn`: `[p, c, M, b, t]`.
fn ellipse_slots(p: P2, ell: &Ell, t: ParamId) -> [ParamId; 8] {
    cat::<8>(&[&p, &ell.c, &ell.m, &[ell.b], &[t]])
}

enum Geo {
    Point(P2),
    Line(Seg),
    Round(Round),
    Ellipse(Ell),
    Spline(Spl),
}

struct Ctx<'a> {
    s: &'a Sketch,
}

impl Ctx<'_> {
    fn kind(&self, id: EntityId) -> Result<&EntityKind, SketchError> {
        self.s
            .entity(id)
            .map(|e| &e.kind)
            .ok_or(SketchError::UnknownEntity(id))
    }

    fn wrong(&self, id: EntityId, expected: &'static str) -> SketchError {
        match self.kind(id) {
            Ok(k) => SketchError::WrongEntityKind {
                entity: id,
                expected,
                found: k.name(),
            },
            Err(e) => e,
        }
    }

    fn point(&self, id: EntityId) -> Result<P2, SketchError> {
        match *self.kind(id)? {
            EntityKind::Point { x, y } => Ok([x, y]),
            _ => Err(self.wrong(id, "point")),
        }
    }

    fn geo(&self, id: EntityId) -> Result<Geo, SketchError> {
        Ok(match *self.kind(id)? {
            EntityKind::Point { x, y } => Geo::Point([x, y]),
            EntityKind::Line { start, end } => Geo::Line([self.point(start)?, self.point(end)?]),
            EntityKind::Circle { center, radius } => Geo::Round(Round::Circle {
                c: self.point(center)?,
                r: radius,
            }),
            EntityKind::Arc { center, start, end } => Geo::Round(Round::Arc {
                c: self.point(center)?,
                s: self.point(start)?,
                e: self.point(end)?,
            }),
            EntityKind::Ellipse {
                center,
                major,
                minor_radius,
            } => Geo::Ellipse(Ell {
                c: self.point(center)?,
                m: self.point(major)?,
                b: minor_radius,
                arc: None,
            }),
            EntityKind::EllipticArc {
                center,
                major,
                minor_radius,
                start,
                end,
                t0,
                t1,
            } => Geo::Ellipse(Ell {
                c: self.point(center)?,
                m: self.point(major)?,
                b: minor_radius,
                arc: Some(EllArc {
                    s: self.point(start)?,
                    e: self.point(end)?,
                    t0,
                    t1,
                }),
            }),
            EntityKind::Spline(ref d) => Geo::Spline(self.spline(d)?),
            EntityKind::Conic {
                start,
                end,
                apex,
                rho,
            } => Geo::Spline(Spl::new(
                2,
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                vec![self.point(start)?, self.point(apex)?, self.point(end)?],
                vec![
                    vec![1.0, 0.0, 0.0],
                    vec![0.0, 1.0, 0.0],
                    vec![0.0, 0.0, 1.0],
                ],
                WSrc::Rho(rho),
            )),
        })
    }

    /// A spline definition as the solver sees it: input points, pole map, weights.
    fn spline(&self, d: &crate::model::SplineDef) -> Result<Spl, SketchError> {
        use crate::model::SplineDef;
        match d {
            SplineDef::Control {
                degree,
                poles,
                weights,
                knots,
                ..
            } => {
                let pts = poles
                    .iter()
                    .map(|&p| self.point(p))
                    .collect::<Result<Vec<_>, _>>()?;
                let n = pts.len();
                let m = (0..n)
                    .map(|i| (0..n).map(|j| if i == j { 1.0 } else { 0.0 }).collect())
                    .collect();
                let w = match weights {
                    Some(ws) => WSrc::Params(ws.clone()),
                    None => WSrc::Ones,
                };
                Ok(Spl::new(usize::from(*degree), knots.clone(), pts, m, w))
            }
            SplineDef::Fit {
                start_tangent,
                end_tangent,
                params,
                ..
            } => {
                let pts = d
                    .points()
                    .iter()
                    .map(|&p| self.point(p))
                    .collect::<Result<Vec<_>, _>>()?;
                let map = FitMap::new(params, start_tangent.is_some(), end_tangent.is_some())?;
                Ok(Spl::new(3, map.knots, pts, map.m, WSrc::Ones))
            }
        }
    }

    fn is_spline(&self, id: EntityId) -> Result<bool, SketchError> {
        Ok(self.kind(id)?.is_bspline())
    }

    /// One side of an end-point tangent / curvature constraint.
    fn end_side(&self, id: EntityId, end: crate::End) -> Result<EndSide, SketchError> {
        match self.geo(id)? {
            Geo::Line([a, b]) => Ok(EndSide::Line { a, b }),
            Geo::Round(Round::Arc { c, s, e }) => Ok(EndSide::Arc {
                c,
                p: if end == crate::End::Start { s } else { e },
            }),
            Geo::Spline(spl) => Ok(EndSide::Spline { spl, end }),
            _ => Err(self.wrong(id, "line, arc, spline or conic")),
        }
    }

    /// Centre point params of a circle, arc, ellipse or elliptical arc.
    fn center(&self, id: EntityId) -> Result<P2, SketchError> {
        match self.geo(id)? {
            Geo::Round(r) => Ok(r.center()),
            Geo::Ellipse(e) => Ok(e.c),
            _ => Err(self.wrong(id, "circle, arc or ellipse")),
        }
    }

    fn ellipse(&self, id: EntityId) -> Result<Ell, SketchError> {
        match self.geo(id)? {
            Geo::Ellipse(e) => Ok(e),
            _ => Err(self.wrong(id, "ellipse or elliptical arc")),
        }
    }

    fn line(&self, id: EntityId) -> Result<Seg, SketchError> {
        match self.geo(id)? {
            Geo::Line(l) => Ok(l),
            _ => Err(self.wrong(id, "line")),
        }
    }

    fn round(&self, id: EntityId) -> Result<Round, SketchError> {
        match self.geo(id)? {
            Geo::Round(r) => Ok(r),
            _ => Err(self.wrong(id, "circle or arc")),
        }
    }

    /// Direction of travel at end `end` of a line or arc: `(Dir, 4 slots)`.
    fn travel(&self, id: EntityId, end: crate::End) -> Result<(Dir, [ParamId; 4]), SketchError> {
        match self.geo(id)? {
            Geo::Line([a, b]) => Ok((Dir::Line, cat::<4>(&[&a, &b]))),
            Geo::Round(Round::Arc { c, s, e }) => {
                let p = match end {
                    crate::End::Start => s,
                    crate::End::End => e,
                };
                Ok((Dir::ArcTangent, cat::<4>(&[&c, &p])))
            }
            _ => Err(self.wrong(id, "line or arc")),
        }
    }
}

/// Two rows `p − q`.
fn coincide(b: &mut Builder, p: P2, q: P2) {
    for k in 0..2 {
        b.term(Func::Coord, 1.0, &[p[k]])
            .term(Func::Coord, -1.0, &[q[k]])
            .mm(0.0);
    }
}

/// Signed distance of `p` from line `l` with coefficient `coef` (no row finish).
fn signed(b: &mut Builder, l: Seg, p: P2, coef: f64) {
    b.term(Func::SignedDist, coef, &cat::<6>(&[&l[0], &l[1], &p]));
}

/// Two rows: `p`, `q` mirror about axis `ax` (midpoint on the axis, `q − p` ⟂ axis).
fn mirror_points(b: &mut Builder, ax: Seg, p: P2, q: P2) {
    let s = cat::<8>(&[&ax[0], &ax[1], &p, &q]);
    b.term(Func::MidSignedDist, 1.0, &s).mm(0.0);
    b.term(Func::DirDot, 1.0, &s).mm(0.0);
}

/// Internal equations of an entity: `Some` for arcs (`|s − c| − |e − c|`, 1 row) and elliptical
/// arcs (`s − P(t0)` and `e − P(t1)`, 4 rows), `None` for the other kinds.
pub fn entity_residual(sketch: &Sketch, id: EntityId) -> Option<SketchResidual> {
    let ctx = Ctx { s: sketch };
    match ctx.geo(id).ok()? {
        Geo::Round(Round::Arc { c, s, e }) => {
            let mut b = Builder::default();
            b.term(Func::Dist, 1.0, &cat::<4>(&[&s, &c]))
                .term(Func::Dist, -1.0, &cat::<4>(&[&e, &c]))
                .mm(0.0);
            Some(b.finish())
        }
        Geo::Ellipse(ell) => {
            let arc = ell.arc?;
            let mut b = Builder::default();
            for (p, t) in [(arc.s, arc.t0), (arc.e, arc.t1)] {
                for k in 0..2 {
                    b.term(Func::EllipseOn(k), 1.0, &ellipse_slots(p, &ell, t))
                        .mm(0.0);
                }
            }
            Some(b.finish())
        }
        _ => None,
    }
}

/// Residual block of a constraint or **driving** dimension, with angular rows scaled by
/// `length_scale` (`L_c`, mm). `Ok(None)` for `Fix` (it locks params instead, see
/// [`fixed_params`]) and for driven dimensions (they add no equations).
///
/// The kind need not be in the sketch (candidate checks), but its entities must be, with the
/// kinds `Sketch::add_constraint` accepts (`UnknownEntity` / `WrongEntityKind` otherwise).
///
/// Rows not in the module table:
/// - **Symmetric** about axis line `l`: points: midpoint on `l` + `dot(q − p, l̂) = 0`
///   (2 rows); lines: that for start↔start and end↔end (4 rows); circles: centres (2) +
///   `r_a − r_b` (3 rows); arcs, whose mirror image reverses the CCW direction: centres (2),
///   `a.start ↔ b.end` (2) and `dot(b.start − a.end, l̂) = 0` (1), 5 rows (the remaining
///   condition follows from the arcs' internal equations).
/// - **Distance** (`σ = ±1` from `side`): point–point `|p − q| − v`; point–line
///   `σ·s(p) − v`; line–line `σ·s(midpoint of b, line a) − v`; point–round
///   `σ·(|p − c| − r) − v` (positive = outside); line–round `σ·s(c) − r − v` (the gap to the
///   near edge, `σ` = side of the centre). Either argument order.
/// - **Horizontal / Vertical distance**: `σ·(b_x − a_x) − v` / `σ·(b_y − a_y) − v`.
/// - **Length** `|d| − v`; **Angle** `L_c · wrap(angle(u, w) − v)` with `u`, `w` the line
///   directions, each reversed per `AngleBranch`; **Radius** `r − v`; **Diameter** `2r − v`;
///   **ArcLength** `r·sweep − v` (sweep ∈ (0, 2π], CCW).
///
/// **Ellipses (SK-22).** Point on ellipse needs the constraint's auxiliary curve parameter, so
/// this function returns `NotApplicable` for it; use [`constraint_residual_aux`] with the
/// stored [`Constraint::aux`](crate::Constraint::aux). Rows: point on ellipse or elliptical
/// arc `p − P(t)` (2); tangent line–ellipse `s ∓ h` with `s` the signed distance of the centre
/// from the line and `h` the half-width of the ellipse normal to it (1); `Concentric` centres
/// (2); `Equal` ellipse–ellipse `|M₁ − c₁| − |M₂ − c₂|`, `b₁ − b₂` (2); `Symmetric` ellipses
/// centres (2), major end points (2), `b₁ − b₂` (5); `EllipseMajor` `|M − c| − v`,
/// `EllipseMinor` `b − v` (1).
pub fn constraint_residual(
    sketch: &Sketch,
    kind: &ConstraintKind,
    length_scale: f64,
) -> Result<Option<SketchResidual>, SketchError> {
    build(sketch, kind, &[], length_scale, false)
}

/// [`constraint_residual`] for a constraint with auxiliary params (`aux` is
/// [`Constraint::aux`](crate::Constraint::aux): the curve parameter of a point on an ellipse).
pub fn constraint_residual_aux(
    sketch: &Sketch,
    kind: &ConstraintKind,
    aux: &[ParamId],
    length_scale: f64,
) -> Result<Option<SketchResidual>, SketchError> {
    build(sketch, kind, aux, length_scale, false)
}

/// Largest absolute residual of `kind` at the sketch's current values, for a constraint that
/// need not be in the sketch (candidate checks). A point on an ellipse is measured with the
/// curve parameter of the point's nearest ellipse point. `Ok(0)` for `Fix` and driven
/// dimensions.
pub(crate) fn candidate_max_residual(
    sketch: &Sketch,
    kind: &ConstraintKind,
    lc: f64,
) -> Result<f64, SketchError> {
    let mut x = std::borrow::Cow::Borrowed(sketch.params().as_slice());
    let mut aux: SmallVec<[ParamId; 1]> = SmallVec::new();
    if let ConstraintKind::PointOnCurve { point, curve } = *kind
        && let Some(p) = sketch.point(point)
    {
        let t = match sketch.curve(curve) {
            Some(Curve2::Ellipse(e)) => Some(
                EllipseArc2::full(e.center, e.major, e.minor_radius)
                    .closest(p)
                    .0,
            ),
            Some(Curve2::BSpline(b)) => Some(b.closest(p).0),
            _ => None,
        };
        if let Some(t) = t {
            let id = ParamId(u32::try_from(x.len()).unwrap_or(u32::MAX));
            x.to_mut().push(t);
            aux.push(id);
        }
    }
    Ok(match build(sketch, kind, &aux, lc, false)? {
        Some(r) => {
            let mut out = vec![0.0; r.rows()];
            r.eval(&x, &mut out);
            out.iter().fold(0.0_f64, |m, v| m.max(v.abs()))
        }
        None => 0.0,
    })
}

/// Like [`constraint_residual_aux`], but also builds driven dimensions (orientation capture and
/// driven-value measurement use it).
pub(crate) fn build(
    sketch: &Sketch,
    kind: &ConstraintKind,
    aux: &[ParamId],
    lc: f64,
    include_driven: bool,
) -> Result<Option<SketchResidual>, SketchError> {
    let ctx = Ctx { s: sketch };
    let mut b = Builder::default();
    match *kind {
        ConstraintKind::Coincident { a, b: q } => coincide(&mut b, ctx.point(a)?, ctx.point(q)?),
        ConstraintKind::PointOnCurve { point, curve } => {
            let p = ctx.point(point)?;
            match ctx.geo(curve)? {
                Geo::Line(l) => {
                    signed(&mut b, l, p, 1.0);
                    b.mm(0.0);
                }
                Geo::Round(r) => {
                    b.term(Func::Dist, 1.0, &cat::<4>(&[&p, &r.center()]));
                    r.radius(&mut b, -1.0);
                    b.mm(0.0);
                }
                Geo::Ellipse(ell) => {
                    let &[t] = aux else {
                        return Err(SketchError::NotApplicable(
                            "a point on an ellipse needs its curve parameter (use constraint_residual_aux)",
                        ));
                    };
                    for k in 0..2 {
                        b.term(Func::EllipseOn(k), 1.0, &ellipse_slots(p, &ell, t))
                            .mm(0.0);
                    }
                }
                Geo::Spline(spl) => {
                    let &[t] = aux else {
                        return Err(SketchError::NotApplicable(
                            "a point on a spline needs its curve parameter (use constraint_residual_aux)",
                        ));
                    };
                    return Ok(Some(SketchResidual::custom(SplineRes::point_on(spl, p, t))));
                }
                Geo::Point(_) => {
                    return Err(ctx.wrong(curve, "line, circle, arc, ellipse, spline or conic"));
                }
            }
        }
        ConstraintKind::Midpoint { point, line } => {
            let p = ctx.point(point)?;
            let [s, e] = ctx.line(line)?;
            for k in 0..2 {
                b.term(Func::Coord, 2.0, &[p[k]])
                    .term(Func::Coord, -1.0, &[s[k]])
                    .term(Func::Coord, -1.0, &[e[k]])
                    .mm(0.0);
            }
        }
        ConstraintKind::Horizontal(t) | ConstraintKind::Vertical(t) => {
            let k = usize::from(matches!(kind, ConstraintKind::Horizontal(_)));
            let (p, q) = match t {
                HvTarget::Line(l) => {
                    let [s, e] = ctx.line(l)?;
                    (s, e)
                }
                HvTarget::Points(p, q) => (ctx.point(p)?, ctx.point(q)?),
            };
            b.term(Func::Coord, 1.0, &[p[k]])
                .term(Func::Coord, -1.0, &[q[k]])
                .mm(0.0);
        }
        ConstraintKind::Parallel { a, b: l2 } | ConstraintKind::Perpendicular { a, b: l2 } => {
            let (la, lb) = (ctx.line(a)?, ctx.line(l2)?);
            let f = if matches!(kind, ConstraintKind::Parallel { .. }) {
                Func::Sin
            } else {
                Func::Cos
            };
            b.term(f, 1.0, &cat::<8>(&[&la[0], &la[1], &lb[0], &lb[1]]))
                .row(0.0, false, lc);
        }
        ConstraintKind::Collinear { a, b: l2 } => {
            let (la, lb) = (ctx.line(a)?, ctx.line(l2)?);
            for p in lb {
                signed(&mut b, la, p, 1.0);
                b.mm(0.0);
            }
        }
        ConstraintKind::Tangent {
            a,
            b: q,
            kind:
                TangentKind::AtEndpoint {
                    a_end,
                    b_end,
                    reversed,
                },
        } if ctx.is_spline(a)? || ctx.is_spline(q)? => {
            let (sa, sb) = (ctx.end_side(a, a_end)?, ctx.end_side(q, b_end)?);
            return Ok(Some(SketchResidual::custom(SplineRes::ends(
                sa, sb, reversed, false, lc,
            ))));
        }
        ConstraintKind::Curvature {
            a,
            b: q,
            a_end,
            b_end,
            reversed,
        } => {
            let (sa, sb) = (ctx.end_side(a, a_end)?, ctx.end_side(q, b_end)?);
            return Ok(Some(SketchResidual::custom(SplineRes::ends(
                sa, sb, reversed, true, lc,
            ))));
        }
        ConstraintKind::Tangent { a, b: q, kind: tk } => tangent(&ctx, &mut b, a, q, tk, lc)?,
        ConstraintKind::Concentric { a, b: q } => {
            coincide(&mut b, ctx.center(a)?, ctx.center(q)?);
        }
        ConstraintKind::Equal { a, b: q } => {
            let (ga, gb) = (ctx.geo(a)?, ctx.geo(q)?);
            match (ga, gb) {
                (Geo::Line(la), Geo::Line(lb)) => {
                    b.term(Func::Dist, 1.0, &cat::<4>(&[&la[1], &la[0]]))
                        .term(Func::Dist, -1.0, &cat::<4>(&[&lb[1], &lb[0]]))
                        .mm(0.0);
                }
                (Geo::Line(l), Geo::Round(Round::Arc { c, s, e })) => {
                    b.term(Func::Dist, 1.0, &cat::<4>(&[&l[1], &l[0]]))
                        .term(Func::ArcLen, -1.0, &cat::<6>(&[&c, &s, &e]))
                        .mm(0.0);
                }
                (Geo::Round(Round::Arc { c, s, e }), Geo::Line(l)) => {
                    b.term(Func::ArcLen, 1.0, &cat::<6>(&[&c, &s, &e]))
                        .term(Func::Dist, -1.0, &cat::<4>(&[&l[1], &l[0]]))
                        .mm(0.0);
                }
                (Geo::Round(ra), Geo::Round(rb)) => {
                    ra.radius(&mut b, 1.0);
                    rb.radius(&mut b, -1.0);
                    b.mm(0.0);
                }
                (Geo::Ellipse(ea), Geo::Ellipse(eb)) => {
                    b.term(Func::Dist, 1.0, &cat::<4>(&[&ea.m, &ea.c]))
                        .term(Func::Dist, -1.0, &cat::<4>(&[&eb.m, &eb.c]))
                        .mm(0.0);
                    b.term(Func::Coord, 1.0, &[ea.b])
                        .term(Func::Coord, -1.0, &[eb.b])
                        .mm(0.0);
                }
                (Geo::Spline(sa), Geo::Spline(sb)) => match (&sa.w, &sb.w) {
                    (WSrc::Params(wa), WSrc::Params(wb)) if wa.len() == wb.len() => {
                        for (&u, &v) in wa.iter().zip(wb) {
                            b.term(Func::Coord, 1.0, &[u])
                                .term(Func::Coord, -1.0, &[v])
                                .mm(0.0);
                        }
                    }
                    (WSrc::Rho(u), WSrc::Rho(v)) => {
                        b.term(Func::Coord, 1.0, &[*u])
                            .term(Func::Coord, -1.0, &[*v])
                            .mm(0.0);
                    }
                    _ => {
                        return Err(SketchError::NotApplicable(
                            "equal needs two conics or two rational control-point splines with the same pole count",
                        ));
                    }
                },
                (Geo::Spline(_), _) | (_, Geo::Spline(_)) => {
                    return Err(SketchError::NotApplicable(
                        "equal does not apply between a spline or conic and another kind",
                    ));
                }
                _ => return Err(ctx.wrong(q, "line or arc")),
            }
        }
        ConstraintKind::Symmetric { a, b: q, axis } => {
            let ax = ctx.line(axis)?;
            match (ctx.geo(a)?, ctx.geo(q)?) {
                (Geo::Point(p), Geo::Point(r)) => mirror_points(&mut b, ax, p, r),
                (Geo::Line(la), Geo::Line(lb)) => {
                    mirror_points(&mut b, ax, la[0], lb[0]);
                    mirror_points(&mut b, ax, la[1], lb[1]);
                }
                (Geo::Round(ra @ Round::Circle { .. }), Geo::Round(rb @ Round::Circle { .. })) => {
                    mirror_points(&mut b, ax, ra.center(), rb.center());
                    ra.radius(&mut b, 1.0);
                    rb.radius(&mut b, -1.0);
                    b.mm(0.0);
                }
                (
                    Geo::Round(Round::Arc {
                        c: ca,
                        s: sa,
                        e: ea,
                    }),
                    Geo::Round(Round::Arc {
                        c: cb,
                        s: sb,
                        e: eb,
                    }),
                ) => {
                    mirror_points(&mut b, ax, ca, cb);
                    mirror_points(&mut b, ax, sa, eb);
                    b.term(Func::DirDot, 1.0, &cat::<8>(&[&ax[0], &ax[1], &ea, &sb]))
                        .mm(0.0);
                }
                (Geo::Ellipse(ea), Geo::Ellipse(eb)) if ea.arc.is_none() && eb.arc.is_none() => {
                    mirror_points(&mut b, ax, ea.c, eb.c);
                    mirror_points(&mut b, ax, ea.m, eb.m);
                    b.term(Func::Coord, 1.0, &[ea.b])
                        .term(Func::Coord, -1.0, &[eb.b])
                        .mm(0.0);
                }
                (Geo::Ellipse(_), Geo::Ellipse(_)) => {
                    return Err(SketchError::NotApplicable(
                        "symmetry of elliptical arcs is not supported",
                    ));
                }
                (Geo::Spline(_), _) | (_, Geo::Spline(_)) => {
                    return Err(SketchError::NotApplicable(
                        "symmetry of splines and conics is not supported; make their points symmetric",
                    ));
                }
                _ => return Err(ctx.wrong(q, "entity of the same kind")),
            }
        }
        ConstraintKind::Fix { entity } => {
            ctx.kind(entity)?;
            return Ok(None);
        }
        ConstraintKind::Dimension(ref d) => {
            if !d.driving && !include_driven {
                return Ok(None);
            }
            dimension(&ctx, &mut b, &d.kind, d.value, lc)?;
        }
    }
    Ok(Some(b.finish()))
}

fn tangent(
    ctx: &Ctx<'_>,
    b: &mut Builder,
    a: EntityId,
    q: EntityId,
    tk: TangentKind,
    lc: f64,
) -> Result<(), SketchError> {
    match tk {
        TangentKind::AtEndpoint {
            a_end,
            b_end,
            reversed,
        } => {
            let (da, sa) = ctx.travel(a, a_end)?;
            let (db, sb) = ctx.travel(q, b_end)?;
            // angle(t_a, −t_b) = angle(t_a, t_b) − π (mod 2π)
            let offset = if reversed { -PI } else { 0.0 };
            b.term(Func::Angle(da, db), 1.0, &cat::<8>(&[&sa, &sb]))
                .row(offset, true, lc);
        }
        TangentKind::External | TangentKind::Internal => {
            let ext = tk == TangentKind::External;
            match (ctx.geo(a)?, ctx.geo(q)?) {
                (Geo::Line(l), Geo::Round(r)) | (Geo::Round(r), Geo::Line(l)) => {
                    signed(b, l, r.center(), 1.0);
                    r.radius(b, if ext { -1.0 } else { 1.0 });
                    b.mm(0.0);
                }
                (Geo::Round(r1), Geo::Round(r2)) => {
                    b.term(Func::Dist, 1.0, &cat::<4>(&[&r1.center(), &r2.center()]));
                    if ext {
                        r1.radius(b, -1.0);
                        r2.radius(b, -1.0);
                    } else {
                        abs_radius_diff(b, r1, r2);
                    }
                    b.mm(0.0);
                }
                (Geo::Line(l), Geo::Ellipse(e)) | (Geo::Ellipse(e), Geo::Line(l)) => {
                    // Signed centre distance minus (External) / plus (Internal) the ellipse's
                    // half-width normal to the line.
                    signed(b, l, e.c, 1.0);
                    b.term(
                        Func::EllipseSupport,
                        if ext { -1.0 } else { 1.0 },
                        &cat::<9>(&[&l[0], &l[1], &e.c, &e.m, &[e.b]]),
                    );
                    b.mm(0.0);
                }
                (Geo::Point(_), _) => return Err(ctx.wrong(a, "line, circle or arc")),
                (Geo::Spline(_), _) | (_, Geo::Spline(_)) => {
                    return Err(SketchError::NotApplicable(
                        "splines and conics are tangent only at a shared end point",
                    ));
                }
                (Geo::Ellipse(_), _) | (_, Geo::Ellipse(_)) => {
                    return Err(SketchError::NotApplicable(
                        "an ellipse can only be tangent to a line",
                    ));
                }
                _ => return Err(ctx.wrong(q, "circle or arc")),
            }
        }
    }
    Ok(())
}

/// Adds `−|r1 − r2|` to the current row.
fn abs_radius_diff(b: &mut Builder, r1: Round, r2: Round) {
    let (k1, s1) = r1.radius_slots();
    let (k2, s2) = r2.radius_slots();
    let slots: SmallVec<[ParamId; 8]> = s1.iter().chain(&s2).copied().collect();
    b.term(Func::AbsRadDiff(k1, k2), -1.0, &slots);
}

fn dimension(
    ctx: &Ctx<'_>,
    b: &mut Builder,
    kind: &DimensionKind,
    v: f64,
    lc: f64,
) -> Result<(), SketchError> {
    match *kind {
        DimensionKind::Distance { a, b: q, side } => {
            let sg = sign(side);
            match (ctx.geo(a)?, ctx.geo(q)?) {
                (Geo::Point(p), Geo::Point(r)) => {
                    b.term(Func::Dist, 1.0, &cat::<4>(&[&p, &r]));
                }
                (Geo::Point(p), Geo::Line(l)) | (Geo::Line(l), Geo::Point(p)) => {
                    signed(b, l, p, sg);
                }
                (Geo::Line(la), Geo::Line(lb)) => {
                    b.term(
                        Func::MidSignedDist,
                        sg,
                        &cat::<8>(&[&la[0], &la[1], &lb[0], &lb[1]]),
                    );
                }
                (Geo::Point(p), Geo::Round(r)) | (Geo::Round(r), Geo::Point(p)) => {
                    b.term(Func::Dist, sg, &cat::<4>(&[&p, &r.center()]));
                    r.radius(b, -sg);
                }
                (Geo::Line(l), Geo::Round(r)) | (Geo::Round(r), Geo::Line(l)) => {
                    signed(b, l, r.center(), sg);
                    r.radius(b, -1.0);
                }
                (Geo::Round(_), Geo::Round(_)) => return Err(ctx.wrong(q, "point or line")),
                (Geo::Ellipse(_), _) | (_, Geo::Ellipse(_)) => {
                    return Err(SketchError::NotApplicable(
                        "distances to an ellipse are not supported",
                    ));
                }
                (Geo::Spline(_), _) | (_, Geo::Spline(_)) => {
                    return Err(SketchError::NotApplicable(
                        "distances to a spline or conic are not supported",
                    ));
                }
            }
            b.mm(-v);
        }
        DimensionKind::HorizontalDistance { a, b: q, side }
        | DimensionKind::VerticalDistance { a, b: q, side } => {
            let k = usize::from(matches!(kind, DimensionKind::VerticalDistance { .. }));
            let (p, r) = (ctx.point(a)?, ctx.point(q)?);
            let sg = sign(side);
            b.term(Func::Coord, sg, &[r[k]])
                .term(Func::Coord, -sg, &[p[k]])
                .mm(-v);
        }
        DimensionKind::Length { line } => {
            let [s, e] = ctx.line(line)?;
            b.term(Func::Dist, 1.0, &cat::<4>(&[&e, &s])).mm(-v);
        }
        DimensionKind::Angle { a, b: q, branch } => {
            let (la, lb) = (ctx.line(a)?, ctx.line(q)?);
            let u = if branch.reverse_a { [la[1], la[0]] } else { la };
            let w = if branch.reverse_b { [lb[1], lb[0]] } else { lb };
            b.term(
                Func::Angle(Dir::Line, Dir::Line),
                1.0,
                &cat::<8>(&[&u[0], &u[1], &w[0], &w[1]]),
            )
            .row(-v, true, lc);
        }
        DimensionKind::Radius { curve } | DimensionKind::Diameter { curve } => {
            let r = ctx.round(curve)?;
            let k = if matches!(kind, DimensionKind::Diameter { .. }) {
                2.0
            } else {
                1.0
            };
            r.radius(b, k);
            b.mm(-v);
        }
        DimensionKind::ArcLength { arc } => match ctx.round(arc)? {
            Round::Arc { c, s, e } => {
                b.term(Func::ArcLen, 1.0, &cat::<6>(&[&c, &s, &e])).mm(-v);
            }
            Round::Circle { .. } => return Err(ctx.wrong(arc, "arc")),
        },
        DimensionKind::EllipseMajor { ellipse } => {
            let e = ctx.ellipse(ellipse)?;
            b.term(Func::Dist, 1.0, &cat::<4>(&[&e.m, &e.c])).mm(-v);
        }
        DimensionKind::EllipseMinor { ellipse } => {
            let e = ctx.ellipse(ellipse)?;
            b.term(Func::Coord, 1.0, &[e.b]).mm(-v);
        }
    }
    Ok(())
}

// ---- Whole-sketch helpers ----------------------------------------------------------------

/// Characteristic length `L_c` (mm) that scales angular rows: the diagonal of the bounding box
/// of all points and circle/arc extents, clamped to ≥ 1 mm (1 mm for an empty sketch).
pub fn characteristic_length(sketch: &Sketch) -> f64 {
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    let mut grow = |x: f64, y: f64, r: f64| {
        lo = [lo[0].min(x - r), lo[1].min(y - r)];
        hi = [hi[0].max(x + r), hi[1].max(y + r)];
    };
    for (id, e) in sketch.entities() {
        match e.kind {
            EntityKind::Point { .. } => {
                if let Some(p) = sketch.point(id) {
                    grow(p.x, p.y, 0.0);
                }
            }
            EntityKind::Circle { center, radius } => {
                if let Some(c) = sketch.point(center) {
                    grow(c.x, c.y, sketch.params().get(radius).abs());
                }
            }
            EntityKind::Arc { center, start, .. } => {
                if let (Some(c), Some(s)) = (sketch.point(center), sketch.point(start)) {
                    grow(c.x, c.y, c.distance(s));
                }
            }
            EntityKind::Ellipse {
                center,
                major,
                minor_radius,
            }
            | EntityKind::EllipticArc {
                center,
                major,
                minor_radius,
                ..
            } => {
                if let (Some(c), Some(m)) = (sketch.point(center), sketch.point(major)) {
                    let r = c.distance(m).max(sketch.params().get(minor_radius).abs());
                    grow(c.x, c.y, r);
                }
            }
            EntityKind::Line { .. } | EntityKind::Spline(_) | EntityKind::Conic { .. } => {}
        }
    }
    let diag = (hi[0] - lo[0]).hypot(hi[1] - lo[1]);
    if diag.is_finite() { diag.max(1.0) } else { 1.0 }
}

/// Params the solver must not change: those flagged `locked` (projected geometry) and those of
/// `Fix`ed entities (their own params and their points' params).
pub fn fixed_params(sketch: &Sketch) -> BTreeSet<ParamId> {
    let store = sketch.params();
    let mut out: BTreeSet<ParamId> = sketch
        .entities()
        .flat_map(|(_, e)| e.kind.own_params())
        .chain(sketch.constraints().flat_map(|(_, c)| c.aux.clone()))
        .filter(|&p| store.flags(p).locked)
        .collect();
    for (_, c) in sketch.constraints() {
        if let ConstraintKind::Fix { entity } = c.kind
            && let Some(e) = sketch.entity(entity)
        {
            out.extend(e.kind.own_params());
            for child in e.kind.children() {
                if let Some(ce) = sketch.entity(child) {
                    out.extend(ce.kind.own_params());
                }
            }
        }
    }
    out
}

/// The sketch's equation system plus what the solver needs alongside it.
#[derive(Debug)]
pub struct SketchSystem {
    /// Arc internal equations (by entity id), then constraints and driving dimensions (by
    /// constraint id). Rows within a block in the order documented on
    /// [`constraint_residual`].
    pub system: EquationSystem,
    /// Params the solver keeps constant ([`fixed_params`]).
    pub fixed: BTreeSet<ParamId>,
    /// Free unknowns: params owned by entities or constraints, not in `fixed`, sorted.
    pub free: Vec<ParamId>,
    /// `L_c` used for the angular rows (mm).
    pub length_scale: f64,
}

/// Builds the equation system of `sketch` with `L_c = `[`characteristic_length`].
pub fn build_system(sketch: &Sketch) -> Result<SketchSystem, SketchError> {
    build_system_scaled(sketch, characteristic_length(sketch))
}

/// Builds the equation system of `sketch` with a given `L_c` (mm, > 0).
pub fn build_system_scaled(
    sketch: &Sketch,
    length_scale: f64,
) -> Result<SketchSystem, SketchError> {
    let mut system = EquationSystem::new();
    for (id, _) in sketch.entities() {
        if let Some(r) = entity_residual(sketch, id) {
            system.push(EquationOwner::Entity(id), Box::new(r));
        }
    }
    for (id, c) in sketch.constraints() {
        if let Some(r) = constraint_residual_aux(sketch, &c.kind, &c.aux, length_scale)? {
            system.push(EquationOwner::Constraint(id), Box::new(r));
        }
    }
    let fixed = fixed_params(sketch);
    let mut free: Vec<ParamId> = sketch
        .entities()
        .flat_map(|(_, e)| e.kind.own_params())
        .chain(sketch.constraints().flat_map(|(_, c)| c.aux.clone()))
        .filter(|p| !fixed.contains(p))
        .collect();
    free.sort_unstable();
    free.dedup();
    Ok(SketchSystem {
        system,
        fixed,
        free,
        length_scale,
    })
}
