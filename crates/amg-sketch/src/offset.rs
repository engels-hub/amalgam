//! Offset of curve chains (SK-17): lines, arcs and circles, with gap closing and an optional
//! offset constraint.
//!
//! [`offset`] copies a connected chain of lines and arcs (or one circle) at a distance to one
//! side, giving a new chain in the same sketch.
//!
//! # Algorithm
//!
//! 1. The chain is ordered by shared or coincident end points (a chain may be open or closed;
//!    branching is rejected). The `side` point decides the side: the copy lies on the side of
//!    the chain's direction of travel the point is on, at the point of the chain nearest to it.
//! 2. Every element is offset on its own (lines shift, arcs keep their centre and change their
//!    radius). Where two neighbours' offsets leave a gap (an outer corner) the gap is closed by
//!    [`CornerStyle::Extend`] (both offsets are extended along their carrier to their
//!    intersection, the miter) or by [`CornerStyle::Round`] (an arc about the original corner of
//!    radius `distance`). `Extend` falls back to `Round` when the carriers do not meet or the
//!    miter is longer than [`OffsetOptions::miter_limit`] times the distance.
//! 3. The raw result may self-intersect (inner corners, short elements, narrow shapes). All raw
//!    curves are split at their mutual intersections and a piece is kept only if every sample
//!    point of it is at least `distance` away from the original chain. That discards the loops
//!    and keeps the true offset ("trimming"). Duplicate pieces are merged and the end points of
//!    the kept pieces are welded, so the copy is topologically connected.
//!
//! # Constraints (`constrain = true`)
//!
//! The copy follows its source. Per kept piece:
//! - a line: [`Parallel`](ConstraintKind::Parallel) to its source and a driving
//!   [`Distance`](DimensionKind::Distance) dimension line-line;
//! - an arc (or the copy of a circle): [`Concentric`](ConstraintKind::Concentric) with its
//!   source and a driving distance dimension from the piece's start point to the source arc.
//!   A circle copy gets a construction point (on the copy, horizontal from its centre) to carry
//!   that dimension;
//! - a round corner arc: centred on the source corner point itself, with
//!   [`AtEndpoint`](TangentKind::AtEndpoint) tangency to the adjacent copies;
//! - open chain ends and smooth (tangent or collinear) junctions, where no corner fixes the
//!   copy's end along its carrier: a construction line from the source point to the copy's
//!   end point, perpendicular to a source line, or a point-on-line condition on the radius
//!   through a source arc's end.
//!
//! The model has no "equal distance" constraint. All distance dimensions are driving and carry
//! the same value, returned in [`OffsetResult::dimensions`]. Change them together with
//! [`set_offset_distance`] (SK-19 binds them to one model parameter).
//!
//! # Splines and conics
//!
//! A selection of splines and conics (and nothing else) is offset curve by curve into new
//! fit-point splines that approximate the exact offset within
//! [`OffsetOptions::spline_tol`] (see the `spline` submodule). There is no corner closing,
//! trimming or constraint for them: `constrain = true`, a selection that mixes splines with
//! lines and arcs, and a distance at or beyond the local radius of curvature are errors.
//!
//! The geometry helpers in this module are private to it.

use std::collections::HashMap;
use std::f64::consts::TAU;

mod spline;

use amg_core::{Point2, Segment2, Tolerance, Vec2};

use crate::error::SketchError;
use crate::geom::{Arc2, Circle2, Curve2, intersect};
use crate::ids::{ConstraintId, EntityId};
use crate::model::{
    ConstraintKind, Dimension, DimensionKind, End, EntityKind, EntityOrigin, HvTarget, Side,
    Sketch, TangentKind,
};

const TOL: Tolerance = Tolerance::DEFAULT;
/// Two points closer than this are one point.
const WELD_TOL: f64 = 5e-7;
/// Pieces and parameter gaps shorter than this are dropped (mm).
const MIN_LEN: f64 = 1e-6;

/// How an outer corner (a gap between two offset elements) is closed.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum CornerStyle {
    /// Extend both offsets to their intersection (sharp corner); rounds when impossible.
    #[default]
    Extend,
    /// Always close with an arc about the original corner.
    Round,
}

/// Options of [`offset_with`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct OffsetOptions {
    /// Gap closing at outer corners.
    pub corner: CornerStyle,
    /// An extended corner reaching further than `miter_limit * distance` from the original
    /// corner is rounded instead.
    pub miter_limit: f64,
    /// Largest deviation (mm) of the approximating spline from the exact offset of a spline or
    /// conic (the distance of the copy from its source differs from the offset distance by at
    /// most this). Default `1e-4`.
    pub spline_tol: f64,
}

impl Default for OffsetOptions {
    fn default() -> Self {
        OffsetOptions {
            corner: CornerStyle::Extend,
            miter_limit: 10.0,
            spline_tol: 1e-4,
        }
    }
}

/// What an offset created. Mirrors the S03 `EditResult` (which the shared `edit` module will
/// introduce) plus the offset dimensions.
#[derive(Clone, Debug, Default)]
pub struct OffsetResult {
    /// Everything created: curves, points, construction helpers.
    pub created: Vec<EntityId>,
    /// Entities removed (always empty: the source is kept).
    pub removed: Vec<EntityId>,
    /// Constraints created (`constrain = true` only).
    pub constraints_added: Vec<ConstraintId>,
    /// Constraints removed (always empty).
    pub constraints_removed: Vec<ConstraintId>,
    /// The curve entities of the copy (a subset of `created`).
    pub copies: Vec<EntityId>,
    /// The driving offset dimensions, all with the same value; see [`set_offset_distance`].
    pub dimensions: Vec<ConstraintId>,
}

/// Offsets `chain` by `distance` (mm, `> 0`) to the side of `side`, closing outer corners with
/// a sharp extension ([`CornerStyle::Extend`]). With `constrain` the copy is tied to its source
/// (see the module docs).
///
/// `chain` holds the line, arc and circle entities of one or more connected chains in any
/// order. Each connected chain (or circle) is offset on its own.
///
/// # Errors
/// - `InvalidValue`: `distance` is not finite and positive, or `side` is not finite.
/// - `UnknownEntity`, `WrongEntityKind`: a chain entry is not a line, arc or circle.
/// - `NotApplicable`: duplicates, a branching chain, a full-turn arc, or `side` on the chain.
/// - `Degenerate`: the offset is empty (a circle or arc collapses, or the whole copy lies
///   closer than `distance` to the source).
pub fn offset(
    s: &mut Sketch,
    chain: &[EntityId],
    distance: f64,
    side: Point2,
    constrain: bool,
) -> Result<OffsetResult, SketchError> {
    offset_with(
        s,
        chain,
        distance,
        side,
        constrain,
        &OffsetOptions::default(),
    )
}

/// [`offset`] with explicit [`OffsetOptions`]. On error the sketch's geometry is unchanged.
pub fn offset_with(
    s: &mut Sketch,
    chain: &[EntityId],
    distance: f64,
    side: Point2,
    constrain: bool,
    opts: &OffsetOptions,
) -> Result<OffsetResult, SketchError> {
    if !(distance.is_finite() && distance > 0.0) {
        return Err(SketchError::InvalidValue {
            what: "offset distance",
            value: distance,
        });
    }
    for v in [side.x, side.y] {
        if !v.is_finite() {
            return Err(SketchError::InvalidValue {
                what: "offset side point",
                value: v,
            });
        }
    }
    if chain.is_empty() {
        return Err(SketchError::NotApplicable("nothing to offset"));
    }
    if chain.iter().any(|&id| {
        s.entity(id)
            .is_some_and(|e| matches!(e.kind, EntityKind::Spline(_) | EntityKind::Conic { .. }))
    }) {
        return spline::offset_splines(s, chain, distance, side, constrain, opts);
    }
    let elems = read_elems(s, chain)?;
    let chains = order_chains(&elems)?;
    // Plan everything (pure) before touching the sketch.
    let mut plans = Vec::new();
    for ch in &chains {
        plans.push(plan_chain(&elems, ch, distance, side, opts)?);
    }
    if plans.iter().all(Plan::is_empty) {
        return Err(SketchError::Degenerate(
            "the offset is empty: the distance is larger than the shape allows",
        ));
    }
    let mut out = OffsetResult::default();
    let res = plans
        .iter()
        .try_for_each(|p| build(s, &elems, p, distance, constrain, &mut out));
    if let Err(e) = res {
        let _ = s.remove_entities(&out.created);
        return Err(e);
    }
    Ok(out)
}

/// Sets every offset dimension of an offset to `value` (mm, `>= 0`). The copy follows after the
/// next solve.
///
/// # Errors
/// `NotADimension` / `UnknownConstraint` for a bad id, `InvalidValue` for a negative value.
pub fn set_offset_distance(
    s: &mut Sketch,
    dimensions: &[ConstraintId],
    value: f64,
) -> Result<(), SketchError> {
    for &d in dimensions {
        s.set_dimension_value(d, value)?;
    }
    Ok(())
}

// ---- Input --------------------------------------------------------------------------------

/// One selected entity, in entity form.
struct Elem {
    id: EntityId,
    curve: Curve2,
    construction: bool,
    /// End point entities (line/arc) or `None`; arcs also keep their centre.
    start: Option<EntityId>,
    end: Option<EntityId>,
    center: Option<EntityId>,
}

fn read_elems(s: &Sketch, ids: &[EntityId]) -> Result<Vec<Elem>, SketchError> {
    let mut seen = std::collections::BTreeSet::new();
    let mut out = Vec::new();
    for &id in ids {
        let e = s.entity(id).ok_or(SketchError::UnknownEntity(id))?;
        if !seen.insert(id) {
            return Err(SketchError::NotApplicable(
                "an entity is listed twice in the chain",
            ));
        }
        let curve = s.curve(id).ok_or(SketchError::WrongEntityKind {
            entity: id,
            expected: "line, circle or arc",
            found: e.kind.name(),
        })?;
        let (start, end, center) = match e.kind {
            EntityKind::Line { start, end } => (Some(start), Some(end), None),
            EntityKind::Arc { center, start, end } => (Some(start), Some(end), Some(center)),
            EntityKind::Circle { center, .. } => (None, None, Some(center)),
            EntityKind::Point { .. } => {
                return Err(SketchError::WrongEntityKind {
                    entity: id,
                    expected: "line, circle or arc",
                    found: "point",
                });
            }
            EntityKind::Ellipse { .. } | EntityKind::EllipticArc { .. } => {
                // The offset of an ellipse is not an ellipse (it is an eighth-degree curve).
                return Err(SketchError::NotApplicable(
                    "an ellipse cannot be offset (its offset curve is not an ellipse)",
                ));
            }
            // `offset_with` sends selections with splines elsewhere.
            EntityKind::Spline(_) | EntityKind::Conic { .. } => {
                return Err(SketchError::NotApplicable(
                    "a spline or conic cannot be offset together with other curves",
                ));
            }
        };
        match &curve {
            Curve2::Line(l) if l.a.distance(l.b) <= MIN_LEN => {
                return Err(SketchError::Degenerate("zero-length line in the chain"));
            }
            Curve2::Arc(a) if a.sweep.abs() >= TAU - 1e-9 => {
                return Err(SketchError::NotApplicable(
                    "a full-turn arc cannot be offset; use a circle",
                ));
            }
            _ => {}
        }
        out.push(Elem {
            id,
            curve,
            construction: e.construction,
            start,
            end,
            center,
        });
    }
    Ok(out)
}

/// A hashed point cloud that merges points closer than `WELD_TOL`.
struct Weld {
    grid: HashMap<(i64, i64), Vec<usize>>,
    pts: Vec<Point2>,
}

impl Weld {
    const CELL: f64 = 1e-5;

    fn new() -> Self {
        Weld {
            grid: HashMap::new(),
            pts: Vec::new(),
        }
    }

    fn cell(p: Point2) -> (i64, i64) {
        // Truncation to the grid index is the intent; coordinates are far from i64 range.
        #[allow(clippy::cast_possible_truncation)]
        (
            (p.x / Self::CELL).floor() as i64,
            (p.y / Self::CELL).floor() as i64,
        )
    }

    fn find_in(&self, p: Point2, tol: f64) -> Option<usize> {
        let (cx, cy) = Self::cell(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(v) = self.grid.get(&(cx + dx, cy + dy))
                    && let Some(&i) = v.iter().find(|&&i| self.pts[i].distance(p) <= tol)
                {
                    return Some(i);
                }
            }
        }
        None
    }

    fn find(&self, p: Point2) -> Option<usize> {
        self.find_in(p, WELD_TOL)
    }

    fn insert(&mut self, p: Point2) -> usize {
        if let Some(i) = self.find(p) {
            return i;
        }
        let i = self.pts.len();
        self.pts.push(p);
        self.grid.entry(Self::cell(p)).or_default().push(i);
        i
    }
}

/// One element of a chain in travel order.
#[derive(Copy, Clone)]
struct Item {
    /// Index into the `Elem` slice.
    e: usize,
    /// Travelled from its end to its start.
    rev: bool,
}

struct Chain {
    items: Vec<Item>,
    closed: bool,
}

fn order_chains(elems: &[Elem]) -> Result<Vec<Chain>, SketchError> {
    let mut chains = Vec::new();
    // End-point nodes of lines and arcs.
    let mut weld = Weld::new();
    let mut nodes: Vec<Vec<(usize, bool)>> = Vec::new();
    let mut node_of: Vec<[usize; 2]> = vec![[usize::MAX; 2]; elems.len()];
    for (i, el) in elems.iter().enumerate() {
        let Some((a, b)) = curve_ends(&el.curve) else {
            continue;
        };
        for (k, p) in [a, b].into_iter().enumerate() {
            // Chain joints are exact or solver-coincident; allow a little slack.
            let n = match weld.find_in(p, 1e-6) {
                Some(n) => n,
                None => {
                    nodes.push(Vec::new());
                    weld.insert(p)
                }
            };
            nodes[n].push((i, k == 1));
            node_of[i][k] = n;
        }
    }
    if let Some(n) = nodes.iter().find(|n| n.len() > 2) {
        let _ = n;
        return Err(SketchError::NotApplicable(
            "the chain branches at a junction; select simple chains",
        ));
    }
    let mut visited = vec![false; elems.len()];
    for (i, el) in elems.iter().enumerate() {
        if matches!(el.curve, Curve2::Circle(_)) {
            visited[i] = true;
            chains.push(Chain {
                items: vec![Item { e: i, rev: false }],
                closed: true,
            });
        }
    }
    let walk = |start: usize, rev0: bool, visited: &mut [bool]| -> Chain {
        let mut items = Vec::new();
        let (mut e, mut rev) = (start, rev0);
        loop {
            visited[e] = true;
            items.push(Item { e, rev });
            let exit = node_of[e][usize::from(!rev)];
            let next = nodes[exit].iter().find(|&&(o, _)| o != e).copied();
            match next {
                None => {
                    return Chain {
                        items,
                        closed: false,
                    };
                }
                Some((o, at_end)) => {
                    if visited[o] {
                        return Chain {
                            items,
                            closed: o == start,
                        };
                    }
                    e = o;
                    rev = at_end;
                }
            }
        }
    };
    // Open chains first: start at a free end.
    for i in 0..elems.len() {
        if visited[i] {
            continue;
        }
        for k in 0..2 {
            if !visited[i] && nodes[node_of[i][k]].len() == 1 {
                // Free end k: start there (k == 1 means the end is free, so travel reversed).
                chains.push(walk(i, k == 1, &mut visited));
            }
        }
    }
    for i in 0..elems.len() {
        if !visited[i] {
            chains.push(walk(i, false, &mut visited));
        }
    }
    // A loop of one element (both ends in one node) is not a chain we can offset.
    if chains.iter().any(|c| {
        c.closed && c.items.len() == 1 && !matches!(elems[c.items[0].e].curve, Curve2::Circle(_))
    }) {
        return Err(SketchError::NotApplicable(
            "an element whose two ends coincide cannot be offset; use a circle",
        ));
    }
    Ok(chains)
}

fn curve_ends(c: &Curve2) -> Option<(Point2, Point2)> {
    match c {
        Curve2::Line(l) => Some((l.a, l.b)),
        Curve2::Arc(_) => Some((c.eval(0.0), c.eval(c.param_range().1))),
        Curve2::Ellipse(e) if !e.is_full() => Some((c.eval(0.0), c.eval(c.param_range().1))),
        Curve2::Circle(_) | Curve2::Ellipse(_) | Curve2::BSpline(_) => None,
    }
}

// ---- Travel-ordered geometry --------------------------------------------------------------

fn dir(a: f64) -> Vec2 {
    Vec2::new(a.cos(), a.sin())
}

fn cross(a: Vec2, b: Vec2) -> f64 {
    a.x * b.y - a.y * b.x
}

/// A line or arc in the direction of travel.
#[derive(Copy, Clone, Debug)]
enum Tg {
    Line {
        a: Point2,
        b: Point2,
    },
    Arc {
        c: Point2,
        r: f64,
        /// Angle of the start point.
        th: f64,
        /// Direction of travel: `+1` CCW, `-1` CW.
        sg: f64,
        /// Angular length, `> 0`.
        span: f64,
    },
}

impl Tg {
    fn of(curve: &Curve2, rev: bool) -> Tg {
        match *curve {
            Curve2::Line(Segment2 { a, b }) => {
                if rev {
                    Tg::Line { a: b, b: a }
                } else {
                    Tg::Line { a, b }
                }
            }
            Curve2::Arc(Arc2 {
                center,
                radius,
                start_angle,
                sweep,
            }) => {
                if rev {
                    Tg::Arc {
                        c: center,
                        r: radius,
                        th: start_angle + sweep,
                        sg: -1.0,
                        span: sweep.abs(),
                    }
                } else {
                    Tg::Arc {
                        c: center,
                        r: radius,
                        th: start_angle,
                        sg: 1.0,
                        span: sweep.abs(),
                    }
                }
            }
            // Circles are offset separately, and `read_elems` rejects ellipses and splines, so
            // none reaches a chain; the placeholder is never used.
            Curve2::Circle(_) | Curve2::Ellipse(_) | Curve2::BSpline(_) => Tg::Line {
                a: Point2::ZERO,
                b: Point2::X,
            },
        }
    }

    fn start(&self) -> Point2 {
        match *self {
            Tg::Line { a, .. } => a,
            Tg::Arc { c, r, th, .. } => c + dir(th) * r,
        }
    }

    fn end(&self) -> Point2 {
        match *self {
            Tg::Line { b, .. } => b,
            Tg::Arc { c, r, th, sg, span } => c + dir(th + sg * span) * r,
        }
    }

    fn tan_start(&self) -> Vec2 {
        match *self {
            Tg::Line { a, b } => (b - a).normalize(),
            Tg::Arc { th, sg, .. } => dir(th).perp() * sg,
        }
    }

    fn tan_end(&self) -> Vec2 {
        match *self {
            Tg::Line { a, b } => (b - a).normalize(),
            Tg::Arc { th, sg, span, .. } => dir(th + sg * span).perp() * sg,
        }
    }

    /// Shifts to the left of the direction of travel by `delta` (negative = right).
    fn offset_left(&self, delta: f64) -> Option<Tg> {
        match *self {
            Tg::Line { a, b } => {
                let n = (b - a).normalize().perp() * delta;
                Some(Tg::Line { a: a + n, b: b + n })
            }
            Tg::Arc { c, r, th, sg, span } => {
                // Left of a CCW arc is towards the centre.
                let r2 = r - sg * delta;
                (r2 > MIN_LEN).then_some(Tg::Arc {
                    c,
                    r: r2,
                    th,
                    sg,
                    span,
                })
            }
        }
    }

    /// Extends the start by `e0` and the end by `e1` (mm along the curve).
    fn extended(&self, e0: f64, e1: f64) -> Tg {
        match *self {
            Tg::Line { a, b } => {
                let u = (b - a).normalize();
                Tg::Line {
                    a: a - u * e0,
                    b: b + u * e1,
                }
            }
            Tg::Arc { c, r, th, sg, span } => Tg::Arc {
                c,
                r,
                th: th - sg * e0 / r,
                sg,
                span: span + (e0 + e1) / r,
            },
        }
    }

    /// The curve in sketch form (arcs CCW).
    fn curve(&self) -> Curve2 {
        match *self {
            Tg::Line { a, b } => Curve2::Line(Segment2 { a, b }),
            Tg::Arc { c, r, th, sg, span } => Curve2::Arc(Arc2 {
                center: c,
                radius: r,
                start_angle: if sg > 0.0 { th } else { th - span },
                sweep: span,
            }),
        }
    }
}

/// Intersections of the carriers (infinite line / full circle) of two travel curves.
fn carrier_hits(a: &Tg, b: &Tg) -> Vec<Point2> {
    match (*a, *b) {
        (Tg::Line { a: a0, b: a1 }, Tg::Line { a: b0, b: b1 }) => {
            let (u, v) = ((a1 - a0).normalize(), (b1 - b0).normalize());
            let den = cross(u, v);
            if den.abs() < 1e-12 {
                return Vec::new();
            }
            let t = cross(b0 - a0, v) / den;
            vec![a0 + u * t]
        }
        (Tg::Line { a: p, b: q }, Tg::Arc { c, r, .. })
        | (Tg::Arc { c, r, .. }, Tg::Line { a: p, b: q }) => {
            let u = (q - p).normalize();
            let foot = p + u * (c - p).dot(u);
            let h2 = r * r - foot.distance_squared(c);
            if h2 < -1e-12 * r * r {
                return Vec::new();
            }
            let h = h2.max(0.0).sqrt();
            if h < 1e-9 {
                vec![foot]
            } else {
                vec![foot - u * h, foot + u * h]
            }
        }
        (Tg::Arc { c: c1, r: r1, .. }, Tg::Arc { c: c2, r: r2, .. }) => {
            let d = c1.distance(c2);
            if d < 1e-12 {
                return Vec::new();
            }
            let x = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
            let h2 = r1 * r1 - x * x;
            if h2 < -1e-12 * r1 * r1 {
                return Vec::new();
            }
            let h = h2.max(0.0).sqrt();
            let e = (c2 - c1) / d;
            let m = c1 + e * x;
            if h < 1e-9 {
                vec![m]
            } else {
                vec![m + e.perp() * h, m - e.perp() * h]
            }
        }
    }
}

/// Length to extend `t` beyond its end to reach `m` (`None` if impossible).
fn forward_len(t: &Tg, m: Point2) -> Option<f64> {
    match *t {
        Tg::Line { a, b } => {
            let l = (m - b).dot((b - a).normalize());
            (l > -1e-9).then_some(l.max(0.0))
        }
        Tg::Arc { c, r, sg, .. } => {
            let (e, mm) = (t.end() - c, m - c);
            Some(wrap_ang(sg * (mm.y.atan2(mm.x) - e.y.atan2(e.x))) * r)
        }
    }
}

/// Length to extend `t` before its start to reach `m`.
fn backward_len(t: &Tg, m: Point2) -> Option<f64> {
    match *t {
        Tg::Line { a, b } => {
            let l = (a - m).dot((b - a).normalize());
            (l > -1e-9).then_some(l.max(0.0))
        }
        Tg::Arc { c, r, sg, .. } => {
            let (s, mm) = (t.start() - c, m - c);
            Some(wrap_ang(sg * (s.y.atan2(s.x) - mm.y.atan2(mm.x))) * r)
        }
    }
}

/// Wraps to `[0, 2π)`, mapping values within rounding of `2π` to 0.
fn wrap_ang(a: f64) -> f64 {
    let w = a.rem_euclid(TAU);
    if w > TAU - 1e-9 { 0.0 } else { w }
}

// ---- Planning (pure) ----------------------------------------------------------------------

/// Where a raw curve comes from.
#[derive(Copy, Clone, Debug)]
enum Src {
    /// The offset of chain item `i`.
    Item(usize),
    /// A round corner between items `a` and `b`, centred on the source point `vertex`
    /// (at `at`).
    Corner {
        a: usize,
        b: usize,
        vertex: Option<EntityId>,
    },
}

/// An end point of a raw curve that must be tied to the source point `vertex`.
#[derive(Copy, Clone, Debug)]
struct Foot {
    vertex: Option<EntityId>,
    at: Point2,
}

struct Raw {
    curve: Curve2,
    src: Src,
    feet: Vec<Foot>,
}

/// One kept piece of the trimmed offset.
struct Piece {
    curve: Curve2,
    raw: usize,
}

enum Plan {
    /// Offset of a circle: elem index and new radius.
    Circle { e: usize, r: f64 },
    Chain {
        items: Vec<Item>,
        raws: Vec<Raw>,
        pieces: Vec<Piece>,
    },
}

impl Plan {
    fn is_empty(&self) -> bool {
        matches!(self, Plan::Chain { pieces, .. } if pieces.is_empty())
    }
}

fn plan_chain(
    elems: &[Elem],
    ch: &Chain,
    d: f64,
    side: Point2,
    opts: &OffsetOptions,
) -> Result<Plan, SketchError> {
    let sgn = chain_side(elems, ch, side)?;
    if let [Item { e, .. }] = ch.items[..]
        && let Curve2::Circle(Circle2 { radius, .. }) = elems[e].curve
    {
        let r = radius - sgn * d;
        if r <= MIN_LEN {
            return Err(SketchError::Degenerate(
                "the offset circle would have no radius",
            ));
        }
        return Ok(Plan::Circle { e, r });
    }
    let n = ch.items.len();
    let orig: Vec<Tg> = ch
        .items
        .iter()
        .map(|it| Tg::of(&elems[it.e].curve, it.rev))
        .collect();
    let off: Vec<Option<Tg>> = orig.iter().map(|t| t.offset_left(sgn * d)).collect();
    let item_end = |i: usize, at_end: bool| -> Option<EntityId> {
        let it = ch.items[i];
        let el = &elems[it.e];
        if at_end ^ it.rev { el.end } else { el.start }
    };
    let mut ext = vec![(0.0_f64, 0.0_f64); n];
    let mut feet: Vec<Vec<Foot>> = vec![Vec::new(); n];
    let mut corners: Vec<Raw> = Vec::new();
    let joints = if ch.closed { n } else { n - 1 };
    for i in 0..n {
        let Some(o) = off[i] else { continue };
        if !ch.closed && i == 0 {
            feet[i].push(Foot {
                vertex: item_end(i, false),
                at: o.start(),
            });
        }
        if !ch.closed && i == n - 1 {
            feet[i].push(Foot {
                vertex: item_end(i, true),
                at: o.end(),
            });
        }
    }
    for i in 0..joints {
        let j = (i + 1) % n;
        let (Some(a), Some(b)) = (off[i], off[j]) else {
            continue;
        };
        let v = orig[i].end();
        let (p, q) = (a.end(), b.start());
        if p.distance(q) <= MIN_LEN {
            // Smooth junction: tangent or collinear. The copy's end needs a foot.
            feet[i].push(Foot {
                vertex: item_end(i, true),
                at: p,
            });
            continue;
        }
        let (ta, tb) = (a.tan_end(), b.tan_start());
        let turn = cross(ta, tb);
        let outer = if turn.abs() > 1e-9 {
            sgn * turn < 0.0
        } else {
            ta.dot(tb) < 0.0
        };
        if !outer {
            continue;
        }
        let mut closed = false;
        if opts.corner == CornerStyle::Extend {
            let best = carrier_hits(&a, &b)
                .into_iter()
                .filter(|m| m.distance(v) <= opts.miter_limit * d)
                .filter_map(|m| Some((m, forward_len(&a, m)?, backward_len(&b, m)?)))
                .filter(|&(_, ea, eb)| {
                    let span_ok = |t: &Tg, e: f64| match *t {
                        Tg::Arc { r, span, .. } => span + e / r < TAU - 1e-3,
                        Tg::Line { .. } => true,
                    };
                    span_ok(&a, ea) && span_ok(&b, eb)
                })
                .min_by(|x, y| (x.1 + x.2).total_cmp(&(y.1 + y.2)));
            if let Some((_, ea, eb)) = best {
                ext[i].1 = ea;
                ext[j].0 = eb;
                closed = true;
            }
        }
        if !closed {
            let (tp, tq) = ((p - v).y.atan2((p - v).x), (q - v).y.atan2((q - v).x));
            // Left offsets turn clockwise around the corner, right offsets counter-clockwise.
            let (start, sweep) = if sgn > 0.0 {
                (tq, wrap_ang(tp - tq))
            } else {
                (tp, wrap_ang(tq - tp))
            };
            if sweep > 1e-9 {
                corners.push(Raw {
                    curve: Curve2::Arc(Arc2 {
                        center: v,
                        radius: d,
                        start_angle: start,
                        sweep,
                    }),
                    src: Src::Corner {
                        a: i,
                        b: j,
                        vertex: item_end(i, true),
                    },
                    feet: Vec::new(),
                });
            }
        }
    }
    let mut raws: Vec<Raw> = Vec::new();
    for i in 0..n {
        let Some(o) = off[i] else { continue };
        let t = o.extended(ext[i].0, ext[i].1);
        raws.push(Raw {
            curve: t.curve(),
            src: Src::Item(i),
            feet: std::mem::take(&mut feet[i]),
        });
    }
    raws.extend(corners);
    let origs: Vec<Curve2> = orig.iter().map(Tg::curve).collect();
    let pieces = trim(&raws, &origs, d);
    Ok(Plan::Chain {
        items: ch.items.clone(),
        raws,
        pieces,
    })
}

/// `+1` if the copy lies left of the chain's direction of travel, `-1` if right.
fn chain_side(elems: &[Elem], ch: &Chain, side: Point2) -> Result<f64, SketchError> {
    let mut best: Option<(f64, Vec2, Point2)> = None;
    for it in &ch.items {
        let c = &elems[it.e].curve;
        let (t, pt) = c.closest(side);
        let mut tan = c.derivative(t);
        if it.rev {
            tan = -tan;
        }
        let dist = pt.distance(side);
        if best.is_none_or(|b| dist < b.0) {
            best = Some((dist, tan, pt));
        }
    }
    let (dist, tan, pt) = best.ok_or(SketchError::NotApplicable("nothing to offset"))?;
    if dist <= 10.0 * MIN_LEN {
        return Err(SketchError::NotApplicable(
            "the side point lies on the chain",
        ));
    }
    let c = cross(tan.normalize(), (side - pt) / dist);
    if c.abs() < 1e-9 {
        return Err(SketchError::NotApplicable(
            "the side point is on the tangent of the chain end; pick a point clearly to one side",
        ));
    }
    Ok(c.signum())
}

/// Splits the raw curves at their mutual intersections and keeps the pieces that are at least
/// `d` from every original curve; duplicates are merged.
fn trim(raws: &[Raw], origs: &[Curve2], d: f64) -> Vec<Piece> {
    let boxes: Vec<_> = raws.iter().map(|r| r.curve.bbox().expanded(1e-5)).collect();
    let scale = |c: &Curve2| match c {
        Curve2::Line(l) => l.a.distance(l.b),
        Curve2::Arc(a) => a.radius,
        Curve2::Circle(c) => c.radius,
        Curve2::Ellipse(e) => e.semi_major().min(e.minor_radius),
        Curve2::BSpline(b) => b.length(),
    };
    let mut splits: Vec<Vec<f64>> = vec![Vec::new(); raws.len()];
    for i in 0..raws.len() {
        for j in (i + 1)..raws.len() {
            if !boxes[i].intersects(&boxes[j], &TOL) {
                continue;
            }
            for h in intersect(&raws[i].curve, &raws[j].curve, &TOL) {
                splits[i].push(h.ta);
                splits[j].push(h.tb);
            }
        }
    }
    let obox: Vec<_> = origs.iter().map(|c| c.bbox().expanded(d)).collect();
    let eps = 1e-6 + 1e-7 * d;
    let valid = |c: &Curve2| -> bool {
        let (t0, t1) = c.param_range();
        [0.25, 0.5, 0.75].iter().all(|f| {
            let p = c.eval(t0 + f * (t1 - t0));
            origs.iter().zip(&obox).all(|(o, b)| {
                !(p.x >= b.min.x && p.x <= b.max.x && p.y >= b.min.y && p.y <= b.max.y)
                    || o.closest(p).1.distance(p) >= d - eps
            })
        })
    };
    let mut pieces: Vec<Piece> = Vec::new();
    for (k, raw) in raws.iter().enumerate() {
        let (t0, t1) = raw.curve.param_range();
        let sc = scale(&raw.curve);
        // Parameter gaps are compared as lengths; line parameters are fractions.
        let to_len = |dt: f64| dt * sc;
        let mut ts: Vec<f64> = splits[k]
            .iter()
            .copied()
            .filter(|&t| to_len(t - t0) > 1e-7 && to_len(t1 - t) > 1e-7)
            .collect();
        ts.sort_by(f64::total_cmp);
        let mut all = vec![t0];
        for t in ts {
            if to_len(t - all[all.len() - 1]) > 1e-7 {
                all.push(t);
            }
        }
        if to_len(t1 - all[all.len() - 1]) <= 1e-7 && all.len() > 1 {
            all.pop();
        }
        all.push(t1);
        for w in all.windows(2) {
            let c = raw.curve.trimmed(w[0], w[1]);
            if c.length() <= MIN_LEN || !valid(&c) {
                continue;
            }
            let dup = pieces.iter().any(|p| same_piece(&p.curve, &c));
            if !dup {
                pieces.push(Piece { curve: c, raw: k });
            }
        }
    }
    pieces
}

fn same_piece(a: &Curve2, b: &Curve2) -> bool {
    let mid = |c: &Curve2| {
        let (t0, t1) = c.param_range();
        c.eval(0.5 * (t0 + t1))
    };
    match (a, b) {
        (Curve2::Line(_), Curve2::Line(_)) | (Curve2::Arc(_), Curve2::Arc(_)) => {
            (a.length() - b.length()).abs() <= 1e-6 && mid(a).distance(mid(b)) <= 1e-6
        }
        _ => false,
    }
}

// ---- Building -----------------------------------------------------------------------------

/// A created piece.
struct Placed {
    id: EntityId,
    raw: usize,
    start: EntityId,
    end: EntityId,
}

fn build(
    s: &mut Sketch,
    elems: &[Elem],
    plan: &Plan,
    d: f64,
    constrain: bool,
    out: &mut OffsetResult,
) -> Result<(), SketchError> {
    match plan {
        Plan::Circle { e, r } => build_circle(s, &elems[*e], *r, d, constrain, out),
        Plan::Chain {
            items,
            raws,
            pieces,
        } => build_chain(s, elems, items, raws, pieces, d, constrain, out),
    }
}

fn mid_of(c: &Curve2) -> Point2 {
    let (t0, t1) = c.param_range();
    c.eval(0.5 * (t0 + t1))
}

fn add_dim(
    s: &mut Sketch,
    out: &mut OffsetResult,
    kind: DimensionKind,
    d: f64,
    at: Point2,
) -> Result<(), SketchError> {
    let id = s.add_constraint_oriented(ConstraintKind::Dimension(Dimension {
        kind,
        value: d,
        driving: true,
        label_pos: at,
    }))?;
    out.constraints_added.push(id);
    out.dimensions.push(id);
    Ok(())
}

fn build_circle(
    s: &mut Sketch,
    el: &Elem,
    r: f64,
    d: f64,
    constrain: bool,
    out: &mut OffsetResult,
) -> Result<(), SketchError> {
    let Curve2::Circle(Circle2 { center, .. }) = el.curve else {
        return Err(SketchError::NotApplicable("not a circle"));
    };
    let id = s.add_circle(center, r)?;
    s.set_construction(id, el.construction)?;
    let EntityKind::Circle { center: cc, .. } = s
        .entity(id)
        .map(|e| e.kind.clone())
        .ok_or(SketchError::UnknownEntity(id))?
    else {
        return Err(SketchError::NotApplicable("not a circle"));
    };
    out.created.extend([cc, id]);
    out.copies.push(id);
    if !constrain {
        return Ok(());
    }
    let conc = s.add_constraint(ConstraintKind::Concentric { a: el.id, b: id })?;
    out.constraints_added.push(conc);
    // A helper point on the copy carries the dimension (a circle has no point of its own).
    let p = s.add_point(center + Vec2::new(r, 0.0))?;
    s.set_construction(p, true)?;
    out.created.push(p);
    for k in [
        ConstraintKind::PointOnCurve {
            point: p,
            curve: id,
        },
        ConstraintKind::Horizontal(HvTarget::Points(cc, p)),
    ] {
        out.constraints_added.push(s.add_constraint(k)?);
    }
    add_dim(
        s,
        out,
        DimensionKind::Distance {
            a: p,
            b: el.id,
            side: Side::Positive,
        },
        d,
        center + Vec2::new(r, 0.0),
    )
}

#[allow(clippy::too_many_arguments)]
fn build_chain(
    s: &mut Sketch,
    elems: &[Elem],
    items: &[Item],
    raws: &[Raw],
    pieces: &[Piece],
    d: f64,
    constrain: bool,
    out: &mut OffsetResult,
) -> Result<(), SketchError> {
    let mut weld = Weld::new();
    let mut point_ids: Vec<EntityId> = Vec::new();
    let mut centers: HashMap<usize, EntityId> = HashMap::new();
    let mut placed: Vec<Placed> = Vec::new();
    let mut conc_done: std::collections::BTreeSet<usize> = std::collections::BTreeSet::new();
    let mut vertex = |s: &mut Sketch,
                      out: &mut OffsetResult,
                      weld: &mut Weld,
                      p: Point2|
     -> Result<EntityId, SketchError> {
        if let Some(i) = weld.find(p) {
            return Ok(point_ids[i]);
        }
        let id = s.add_point(p)?;
        out.created.push(id);
        weld.insert(p);
        point_ids.push(id);
        Ok(id)
    };
    for pc in pieces {
        let raw = &raws[pc.raw];
        let el = &elems[match raw.src {
            Src::Item(i) => items[i].e,
            Src::Corner { a, .. } => items[a].e,
        }];
        let (id, a, b) = match pc.curve {
            Curve2::Line(l) => {
                let a = vertex(s, out, &mut weld, l.a)?;
                let b = vertex(s, out, &mut weld, l.b)?;
                if a == b {
                    continue;
                }
                (s.add_line_between(a, b)?, a, b)
            }
            Curve2::Arc(arc) => {
                let t1 = pc.curve.param_range().1;
                let a = vertex(s, out, &mut weld, pc.curve.eval(0.0))?;
                let b = vertex(s, out, &mut weld, pc.curve.eval(t1))?;
                if a == b {
                    return Err(SketchError::Degenerate("an offset arc closes on itself"));
                }
                let center = match (raw.src, constrain) {
                    (
                        Src::Corner {
                            vertex: Some(v), ..
                        },
                        true,
                    ) => v,
                    _ => match centers.get(&pc.raw) {
                        Some(&c) => c,
                        None => {
                            let c = s.add_point(arc.center)?;
                            out.created.push(c);
                            centers.insert(pc.raw, c);
                            c
                        }
                    },
                };
                (
                    s.add_entity(
                        EntityKind::Arc {
                            center,
                            start: a,
                            end: b,
                        },
                        false,
                        EntityOrigin::User,
                    )?,
                    a,
                    b,
                )
            }
            Curve2::Circle(_) => continue,
            Curve2::Ellipse(_) | Curve2::BSpline(_) => {
                return Err(SketchError::NotApplicable(
                    "an ellipse, spline or conic cannot be offset together with lines and arcs",
                ));
            }
        };
        s.set_construction(id, el.construction)?;
        out.created.push(id);
        out.copies.push(id);
        placed.push(Placed {
            id,
            raw: pc.raw,
            start: a,
            end: b,
        });
    }
    if placed.is_empty() {
        return Err(SketchError::Degenerate("the offset is empty"));
    }
    if !constrain {
        return Ok(());
    }
    // Carrier constraints.
    for pl in &placed {
        match raws[pl.raw].src {
            Src::Item(i) => {
                let src = elems[items[i].e].id;
                let is_line = matches!(elems[items[i].e].curve, Curve2::Line(_));
                let at = mid_of(&s_curve(s, pl.id)?);
                if is_line {
                    out.constraints_added
                        .push(s.add_constraint(ConstraintKind::Parallel { a: src, b: pl.id })?);
                    add_dim(
                        s,
                        out,
                        DimensionKind::Distance {
                            a: src,
                            b: pl.id,
                            side: Side::Positive,
                        },
                        d,
                        at,
                    )?;
                } else {
                    if conc_done.insert(pl.raw) {
                        out.constraints_added.push(
                            s.add_constraint(ConstraintKind::Concentric { a: src, b: pl.id })?,
                        );
                    }
                    add_dim(
                        s,
                        out,
                        DimensionKind::Distance {
                            a: pl.start,
                            b: src,
                            side: Side::Positive,
                        },
                        d,
                        at,
                    )?;
                }
            }
            Src::Corner { a, b, .. } => {
                // Tangent to the neighbouring copies at both ends.
                for (pt, c_end) in [(pl.start, End::Start), (pl.end, End::End)] {
                    let nb = placed.iter().find(|o| {
                        o.id != pl.id
                            && matches!(raws[o.raw].src, Src::Item(k) if k == a || k == b)
                            && (o.start == pt || o.end == pt)
                    });
                    if let Some(o) = nb {
                        let o_end = if o.start == pt { End::Start } else { End::End };
                        out.constraints_added.push(s.add_constraint_oriented(
                            ConstraintKind::Tangent {
                                a: o.id,
                                b: pl.id,
                                kind: TangentKind::AtEndpoint {
                                    a_end: o_end,
                                    b_end: c_end,
                                    reversed: false,
                                },
                            },
                        )?);
                    }
                }
            }
        }
    }
    // Feet: open chain ends and smooth junctions.
    let mut done_feet: Vec<(EntityId, EntityId)> = Vec::new();
    let mut radial: HashMap<EntityId, EntityId> = HashMap::new();
    for pl in &placed {
        let raw = &raws[pl.raw];
        let Src::Item(i) = raw.src else { continue };
        let el = &elems[items[i].e];
        for f in &raw.feet {
            let (Some(v), Some(wi)) = (f.vertex, weld.find(f.at)) else {
                continue;
            };
            let foot = point_ids[wi];
            if foot != pl.start && foot != pl.end {
                continue;
            }
            if v == foot || done_feet.contains(&(v, foot)) {
                continue;
            }
            done_feet.push((v, foot));
            match el.curve {
                Curve2::Line(_) => {
                    let aux = s.add_entity(
                        EntityKind::Line {
                            start: v,
                            end: foot,
                        },
                        true,
                        EntityOrigin::User,
                    )?;
                    out.created.push(aux);
                    out.constraints_added.push(
                        s.add_constraint(ConstraintKind::Perpendicular { a: el.id, b: aux })?,
                    );
                }
                _ => {
                    let Some(c) = el.center else { continue };
                    let aux = match radial.get(&v) {
                        Some(&a) => a,
                        None => {
                            let a = s.add_entity(
                                EntityKind::Line { start: c, end: v },
                                true,
                                EntityOrigin::User,
                            )?;
                            out.created.push(a);
                            radial.insert(v, a);
                            a
                        }
                    };
                    out.constraints_added
                        .push(s.add_constraint(ConstraintKind::PointOnCurve {
                            point: foot,
                            curve: aux,
                        })?);
                }
            }
        }
    }
    Ok(())
}

fn s_curve(s: &Sketch, id: EntityId) -> Result<Curve2, SketchError> {
    s.curve(id).ok_or(SketchError::UnknownEntity(id))
}
