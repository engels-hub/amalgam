//! The shape model of the UI-13 creation tools: which picks each tool takes, where typed
//! heads-up values put a pick, the SK-21 / SK-22 builder call the picks stand for, and the
//! inferred constraints and driving dimensions that go with it. Pure (no UI), so the tool in
//! [`create`](super::create) only collects [`Pick`]s.
//!
//! **Pick order** (a pick is a click or an Enter in the heads-up input):
//!
//! | Tool | Picks |
//! |---|---|
//! | 2-point / centre rectangle | corner (centre), opposite corner |
//! | 3-point rectangle | base start, base end, a point on the opposite side |
//! | centre / 2-point circle | centre (diameter start), point on the circle (diameter end) |
//! | 3-point circle | three points |
//! | 2-tangent circle | two curves, a point near the centre (its distance is the radius) |
//! | 3-point arc | start, end, a point on the arc |
//! | centre point arc | centre, start, end (the cursor's turning direction sets the sweep) |
//! | tangent arc | the end of a line or arc, the new end |
//! | polygon | centre, first vertex (inscribed) or first side midpoint (circumscribed) |
//! | straight slots | the builder's two points, a point giving the half width |
//! | centre point arc slot | centre, start, end, a point giving the half width |
//! | point | the point |
//! | ellipse | centre, major axis end, a point on the ellipse |

use amg_core::{Dimension as Dim, Point2, Vec2};
use amg_doc::Expr;
use amg_features::sketch::SketchEdit;
use amg_sketch::query::SnapKind;
use amg_sketch::solve::orient_constraint;
use amg_sketch::{
    Built, ConstraintKind, Dimension, DimensionKind, End, EntityId, EntityKind, HvTarget, Side,
    Sketch, SketchError, build,
};

use super::infer::Inference;
use super::session::Inferred;
use crate::command::CmdId;
use crate::module::IconId;

/// Lengths below this are degenerate, mm.
const TINY: f64 = 1e-9;

/// A sketch creation tool of UI-13.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum CreateKind {
    /// Axis-aligned rectangle from two corners.
    Rect2Point,
    /// Rectangle at any angle from three points.
    Rect3Point,
    /// Axis-aligned rectangle from its centre.
    RectCenter,
    /// Circle from centre and radius.
    CircleCenter,
    /// Circle from the two ends of a diameter.
    Circle2Point,
    /// Circle through three points.
    Circle3Point,
    /// Circle tangent to two curves.
    Circle2Tangent,
    /// Arc through three points.
    Arc3Point,
    /// Arc from centre, start and end.
    ArcCenter,
    /// Arc tangent to the end of a line or arc.
    ArcTangent,
    /// Regular polygon.
    Polygon,
    /// Slot from the two arc centres.
    SlotCenterToCenter,
    /// Slot from its overall length.
    SlotOverall,
    /// Slot from its centre point.
    SlotCenterPoint,
    /// Slot along an arc.
    SlotArc,
    /// Sketch point.
    Point,
    /// Ellipse.
    Ellipse,
}

/// The command data of a tool.
#[derive(Debug)]
pub struct ToolDef {
    /// Command id.
    pub id: CmdId,
    /// Command label (also the undo label).
    pub label: &'static str,
    /// Tooltip.
    pub tooltip: &'static str,
    /// Icon.
    pub icon: IconId,
    /// Extra S-key search terms.
    pub keywords: &'static [&'static str],
    /// Default shortcut (Fusion's, where it has one).
    pub shortcut: Option<&'static str>,
    /// Pinned in the SKETCH tab's Create group.
    pub pinned: bool,
}

macro_rules! def {
    ($id:literal, $label:literal, $tip:literal, $icon:ident, $kw:expr) => {
        def!($id, $label, $tip, $icon, $kw, None, false)
    };
    ($id:literal, $label:literal, $tip:literal, $icon:ident, $kw:expr, $sc:expr, $pin:expr) => {
        &ToolDef {
            id: CmdId($id),
            label: $label,
            tooltip: $tip,
            icon: crate::icons::ids::$icon,
            keywords: $kw,
            shortcut: $sc,
            pinned: $pin,
        }
    };
}

impl CreateKind {
    /// Every tool, in toolbar order.
    pub const ALL: [CreateKind; 17] = [
        CreateKind::Rect2Point,
        CreateKind::Rect3Point,
        CreateKind::RectCenter,
        CreateKind::CircleCenter,
        CreateKind::Circle2Point,
        CreateKind::Circle3Point,
        CreateKind::Circle2Tangent,
        CreateKind::Arc3Point,
        CreateKind::ArcCenter,
        CreateKind::ArcTangent,
        CreateKind::Polygon,
        CreateKind::SlotCenterToCenter,
        CreateKind::SlotOverall,
        CreateKind::SlotCenterPoint,
        CreateKind::SlotArc,
        CreateKind::Ellipse,
        CreateKind::Point,
    ];

    /// The command data.
    pub fn def(self) -> &'static ToolDef {
        match self {
            CreateKind::Rect2Point => def!(
                "sketch.rect_2pt",
                "2-Point Rectangle",
                "Rectangle from two opposite corners",
                RECT_2PT,
                &["sketch", "rectangle", "box", "corner"],
                Some("sketch:R"),
                true
            ),
            CreateKind::Rect3Point => def!(
                "sketch.rect_3pt",
                "3-Point Rectangle",
                "Rectangle at any angle: base side, then height",
                RECT_3PT,
                &["sketch", "rectangle", "rotated"]
            ),
            CreateKind::RectCenter => def!(
                "sketch.rect_center",
                "Center Rectangle",
                "Rectangle from its centre and a corner",
                RECT_CENTER,
                &["sketch", "rectangle", "centre"]
            ),
            CreateKind::CircleCenter => def!(
                "sketch.circle_center",
                "Center Diameter Circle",
                "Circle from its centre and diameter",
                CIRCLE_CENTER,
                &["sketch", "circle", "centre", "radius"],
                Some("sketch:C"),
                true
            ),
            CreateKind::Circle2Point => def!(
                "sketch.circle_2pt",
                "2-Point Circle",
                "Circle from the two ends of a diameter",
                CIRCLE_2PT,
                &["sketch", "circle", "diameter"]
            ),
            CreateKind::Circle3Point => def!(
                "sketch.circle_3pt",
                "3-Point Circle",
                "Circle through three points",
                CIRCLE_3PT,
                &["sketch", "circle"]
            ),
            CreateKind::Circle2Tangent => def!(
                "sketch.circle_2tangent",
                "2-Tangent Circle",
                "Circle tangent to two lines, circles or arcs",
                CIRCLE_2TANGENT,
                &["sketch", "circle", "tangent"]
            ),
            CreateKind::Arc3Point => def!(
                "sketch.arc_3pt",
                "3-Point Arc",
                "Arc from start, end and a point on it",
                ARC_3PT,
                &["sketch", "arc"],
                None,
                true
            ),
            CreateKind::ArcCenter => def!(
                "sketch.arc_center",
                "Center Point Arc",
                "Arc from its centre, start and end",
                ARC_CENTER,
                &["sketch", "arc", "centre"]
            ),
            CreateKind::ArcTangent => def!(
                "sketch.arc_tangent",
                "Tangent Arc",
                "Arc tangent to the end of a line or arc",
                ARC_TANGENT,
                &["sketch", "arc", "tangent"]
            ),
            CreateKind::Polygon => def!(
                "sketch.polygon",
                "Polygon",
                "Regular polygon, inscribed or circumscribed",
                POLYGON,
                &["sketch", "hexagon", "triangle", "ngon"]
            ),
            CreateKind::SlotCenterToCenter => def!(
                "sketch.slot_center",
                "Center to Center Slot",
                "Slot from the centres of its two ends and the width",
                SLOT_CENTER,
                &["sketch", "slot", "obround", "stadium"]
            ),
            CreateKind::SlotOverall => def!(
                "sketch.slot_overall",
                "Overall Slot",
                "Slot from its overall length and the width",
                SLOT_OVERALL,
                &["sketch", "slot", "length"]
            ),
            CreateKind::SlotCenterPoint => def!(
                "sketch.slot_center_point",
                "Center Point Slot",
                "Slot from its centre, one end centre and the width",
                SLOT_CENTER_POINT,
                &["sketch", "slot", "centre"]
            ),
            CreateKind::SlotArc => def!(
                "sketch.slot_arc",
                "Center Point Arc Slot",
                "Slot along an arc: centre, start, end and width",
                SLOT_ARC,
                &["sketch", "slot", "arc", "curved"]
            ),
            CreateKind::Point => def!(
                "sketch.point",
                "Sketch Point",
                "Add a sketch point",
                POINT,
                &["sketch", "point", "dot"]
            ),
            CreateKind::Ellipse => def!(
                "sketch.ellipse",
                "Ellipse",
                "Ellipse from centre, major axis and a point on it",
                ELLIPSE,
                &["sketch", "ellipse", "oval"]
            ),
        }
    }

    /// How many picks complete the shape.
    pub fn picks(self) -> usize {
        match self {
            CreateKind::Point => 1,
            CreateKind::Rect2Point
            | CreateKind::RectCenter
            | CreateKind::CircleCenter
            | CreateKind::Circle2Point
            | CreateKind::ArcTangent
            | CreateKind::Polygon => 2,
            CreateKind::Rect3Point
            | CreateKind::Circle3Point
            | CreateKind::Circle2Tangent
            | CreateKind::Arc3Point
            | CreateKind::ArcCenter
            | CreateKind::SlotCenterToCenter
            | CreateKind::SlotOverall
            | CreateKind::SlotCenterPoint
            | CreateKind::Ellipse => 3,
            CreateKind::SlotArc => 4,
        }
    }

    /// What the next pick is, for the status bar.
    pub fn step_hint(self, stage: usize) -> &'static str {
        use CreateKind as K;
        let steps: &[&str] = match self {
            K::Rect2Point => &["click the first corner", "click the opposite corner"],
            K::RectCenter => &["click the centre", "click a corner"],
            K::Rect3Point => &[
                "click the start of the base",
                "click the end of the base",
                "click the height",
            ],
            K::CircleCenter => &["click the centre", "click a point on the circle"],
            K::Circle2Point => &[
                "click the start of a diameter",
                "click the end of the diameter",
            ],
            K::Circle3Point => &[
                "click the first point",
                "click the second point",
                "click the third point",
            ],
            K::Circle2Tangent => &[
                "select the first curve",
                "select the second curve",
                "move for the radius and click",
            ],
            K::Arc3Point => &[
                "click the start",
                "click the end",
                "click a point on the arc",
            ],
            K::ArcCenter => &["click the centre", "click the start", "click the end"],
            K::ArcTangent => &["click the end of a line or arc", "click the end of the arc"],
            K::Polygon => &["click the centre", "click the radius"],
            K::SlotCenterToCenter => &[
                "click the first centre",
                "click the second centre",
                "click the width",
            ],
            K::SlotOverall => &["click one end", "click the other end", "click the width"],
            K::SlotCenterPoint => &[
                "click the slot centre",
                "click an end centre",
                "click the width",
            ],
            K::SlotArc => &[
                "click the arc centre",
                "click the start",
                "click the end",
                "click the width",
            ],
            K::Point => &["click the position"],
            K::Ellipse => &[
                "click the centre",
                "click the end of the major axis",
                "click a point on the ellipse",
            ],
        };
        steps.get(stage).copied().unwrap_or("")
    }

    /// The heads-up fields of the pick number `stage` (empty: no heads-up input there).
    pub fn fields(self, stage: usize) -> &'static [Field] {
        use CreateKind as K;
        use Field as F;
        match (self, stage) {
            (K::Rect2Point | K::RectCenter, 1) => &[F::Width, F::Height],
            (K::Rect3Point, 1) => &[F::Length, F::Angle],
            (K::Rect3Point, 2) => &[F::Height],
            (K::CircleCenter | K::Circle2Point, 1) | (K::Circle3Point, 2) => &[F::Diameter],
            (K::Circle2Tangent | K::Arc3Point, 2) | (K::ArcTangent, 1) => &[F::Radius],
            (K::Arc3Point, 1) => &[F::Length, F::Angle],
            (K::ArcCenter | K::SlotArc | K::Polygon, 1) => &[F::Radius, F::Angle],
            (K::ArcCenter | K::SlotArc, 2) => &[F::Length],
            (K::SlotArc, 3) => &[F::Width],
            (K::SlotCenterToCenter | K::SlotOverall | K::SlotCenterPoint, 1) => {
                &[F::Length, F::Angle]
            }
            (K::SlotCenterToCenter | K::SlotOverall | K::SlotCenterPoint, 2) => &[F::Width],
            (K::Point, 0) => &[F::X, F::Y],
            (K::Ellipse, 1) => &[F::Major, F::Angle],
            (K::Ellipse, 2) => &[F::Minor],
            _ => &[],
        }
    }

    /// The pick number `stage` is inferred against the first pick (the segment between them
    /// may become horizontal or vertical).
    pub fn directional(self, stage: usize) -> bool {
        use CreateKind as K;
        stage == 1
            && matches!(
                self,
                K::Rect3Point
                    | K::SlotCenterToCenter
                    | K::SlotOverall
                    | K::SlotCenterPoint
                    | K::Ellipse
            )
    }

    /// The pick number `stage` selects a curve of the sketch instead of a position.
    pub fn picks_curve(self, stage: usize) -> bool {
        self == CreateKind::Circle2Tangent && stage < 2
    }

    /// The builder reads existing sketch entities.
    pub fn needs_sketch(self) -> bool {
        matches!(self, CreateKind::Circle2Tangent | CreateKind::ArcTangent)
    }
}

/// A heads-up field.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Field {
    /// Rectangle or slot width.
    Width,
    /// Rectangle height.
    Height,
    /// A length (side, slot length, chord, arc length).
    Length,
    /// Direction from the sketch x axis (sets the geometry, no dimension).
    Angle,
    /// Radius.
    Radius,
    /// Diameter.
    Diameter,
    /// Semi-major radius of an ellipse.
    Major,
    /// Minor radius of an ellipse.
    Minor,
    /// Sketch x coordinate (no dimension).
    X,
    /// Sketch y coordinate (no dimension).
    Y,
}

impl Field {
    /// The short name in the heads-up label.
    pub fn name(self) -> &'static str {
        match self {
            Field::Width => "W",
            Field::Height => "H",
            Field::Length => "L",
            Field::Angle => "A",
            Field::Radius => "R",
            Field::Diameter => "D",
            Field::Major => "a",
            Field::Minor => "b",
            Field::X => "X",
            Field::Y => "Y",
        }
    }

    /// The physical dimension of the value.
    pub fn quantity(self) -> Dim {
        match self {
            Field::Angle => Dim::ANGLE,
            _ => Dim::LENGTH,
        }
    }

    /// The value must be greater than zero.
    pub fn positive(self) -> bool {
        !matches!(self, Field::Angle | Field::X | Field::Y)
    }
}

/// A typed heads-up value (mm or rad) and its expression.
#[derive(Clone, Debug)]
pub struct Typed {
    /// Evaluated value.
    pub value: f64,
    /// What was typed, with its unit.
    pub expr: Expr,
}

/// One pick of a creation tool.
#[derive(Clone, Debug, Default)]
pub struct Pick {
    /// Position, sketch coordinates.
    pub at: Point2,
    /// Why it is there (plain for typed picks).
    pub inference: Inference,
    /// Typed values, in the order of [`CreateKind::fields`].
    pub typed: [Option<Typed>; 2],
    /// The curve picked (2-tangent circle) or extended (tangent arc).
    pub curve: Option<EntityId>,
    /// The end of `curve` the tangent arc starts at.
    pub end: Option<End>,
    /// The arc's sweep turns clockwise from its start to this pick.
    pub clockwise: bool,
}

impl Pick {
    /// A pick at `at` without inference.
    pub fn at(at: Point2) -> Pick {
        Pick {
            at,
            inference: Inference::plain(at),
            ..Pick::default()
        }
    }
}

/// The polygon options (tool dialog).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PolygonOptions {
    /// Number of sides (at least 3).
    pub sides: u32,
    /// The picked point is a vertex (else the midpoint of a side).
    pub inscribed: bool,
}

impl Default for PolygonOptions {
    fn default() -> Self {
        PolygonOptions {
            sides: 6,
            inscribed: true,
        }
    }
}

// ===== where a pick goes =====

fn unit_or(v: Vec2, or: Vec2) -> Vec2 {
    if v.length() > TINY { v.normalize() } else { or }
}

fn sign(v: f64) -> f64 {
    if v < 0.0 { -1.0 } else { 1.0 }
}

/// The point at distance `r` from `base` in the typed direction, else toward `cursor`.
fn polar(base: Point2, cursor: Point2, r: Option<f64>, angle: Option<f64>) -> Point2 {
    let here = cursor - base;
    let dir = match angle {
        Some(a) => Vec2::from_angle(a),
        None => unit_or(here, Vec2::X),
    };
    base + dir * r.unwrap_or_else(|| here.length())
}

/// The position of the next pick: the cursor, moved to satisfy the typed `values` (in the
/// order of [`CreateKind::fields`]). `None` when no position satisfies them.
pub fn resolve(
    kind: CreateKind,
    picks: &[Pick],
    cursor: &Pick,
    values: [Option<f64>; 2],
    sketch: &Sketch,
) -> Option<Point2> {
    use CreateKind as K;
    let stage = picks.len();
    let c = cursor.at;
    let [first, second] = values;
    if first.is_none() && second.is_none() {
        return Some(c);
    }
    let base = picks.first().map_or(Point2::ZERO, |p| p.at);
    let at = match (kind, stage) {
        (K::Point, 0) => Point2::new(first.unwrap_or(c.x), second.unwrap_or(c.y)),
        (K::Rect2Point | K::RectCenter, 1) => {
            let k = if kind == K::RectCenter { 0.5 } else { 1.0 };
            let d = c - base;
            let w = first.map_or(d.x.abs(), |w| w * k);
            let h = second.map_or(d.y.abs(), |h| h * k);
            base + Vec2::new(sign(d.x) * w, sign(d.y) * h)
        }
        (K::Rect3Point | K::Ellipse, 2) => {
            // The distance from the first side (major axis), on the cursor's side.
            let n = unit_or(picks.get(1)?.at - base, Vec2::X).perp();
            let h = first?;
            let s = sign((c - base).dot(n));
            let from = if kind == K::Ellipse {
                base
            } else {
                picks.get(1)?.at
            };
            from + n * (s * h)
        }
        (K::CircleCenter, 1) => polar(base, c, first.map(|d| d * 0.5), None),
        (K::Circle2Point, 1) => polar(base, c, first, None),
        (K::Circle3Point | K::Arc3Point, 2) => {
            let (a, b) = (base, picks.get(1)?.at);
            let r = if kind == K::Circle3Point {
                first? * 0.5
            } else {
                first?
            };
            let half = a.distance(b) * 0.5;
            if r < half * (1.0 - 1e-12) {
                return None;
            }
            let mid = (a + b) * 0.5;
            let n = unit_or(b - a, Vec2::X).perp();
            let s = sign((c - mid).dot(n));
            let h = (r * r - half * half).max(0.0).sqrt();
            if kind == K::Circle3Point {
                // The centre on the cursor's side, the third point opposite the chord.
                mid + n * (s * (h + r))
            } else {
                // The major arc when the arc through the cursor is one, that is when the
                // cursor sees the chord under an acute angle.
                let major = (a - c).dot(b - c) > 0.0;
                let centre = mid + n * (if major { s * h } else { -s * h });
                centre + n * (s * r)
            }
        }
        (K::ArcCenter | K::SlotArc, 2) => {
            let start = picks.get(1)?.at - base;
            let r = start.length();
            if r <= TINY {
                return None;
            }
            let turn = first? / r * if cursor.clockwise { -1.0 } else { 1.0 };
            base + Vec2::from_angle(turn).rotate(start)
        }
        (K::ArcTangent, 1) => {
            let from = picks.first()?;
            let out = super::tools::travel_dir(sketch, from.curve?, from.end?)?;
            let n = out.perp();
            let r = first?;
            let centre = base + n * (sign((c - base).dot(n)) * r);
            polar(centre, c, Some(r), None)
        }
        (K::SlotCenterToCenter | K::SlotOverall | K::SlotCenterPoint, 2) => {
            let end = picks.get(1)?.at;
            let n = unit_or(end - base, Vec2::X).perp();
            end + n * (sign((c - end).dot(n)) * first? * 0.5)
        }
        (K::SlotArc, 3) => {
            let r = picks.get(1)?.at.distance(base);
            polar(base, c, Some(r + first? * 0.5), None)
        }
        // Polar picks: length / radius and direction.
        _ => polar(base, c, first, second),
    };
    (at.x.is_finite() && at.y.is_finite()).then_some(at)
}

// ===== the builder call =====

/// The slot width the last pick stands for.
fn slot_width(kind: CreateKind, picks: &[Pick]) -> f64 {
    let last = picks.last().map_or(Point2::ZERO, |p| p.at);
    if kind == CreateKind::SlotArc {
        let c = picks[0].at;
        2.0 * (last.distance(c) - picks[1].at.distance(c)).abs()
    } else {
        let (a, b) = (picks[0].at, picks[1].at);
        let n = unit_or(b - a, Vec2::X).perp();
        2.0 * (last - a).dot(n).abs()
    }
}

/// Distance from `p` to the full line / circle of curve `e`.
fn curve_distance(s: &Sketch, e: EntityId, p: Point2) -> Option<f64> {
    match s.entity(e)?.kind {
        EntityKind::Line { start, end } => {
            let (a, b) = (s.point(start)?, s.point(end)?);
            let d = (b - a).try_normalize()?;
            Some((p - a).perp_dot(d).abs())
        }
        EntityKind::Circle { center, radius } => {
            Some((p.distance(s.point(center)?) - s.params().get(radius)).abs())
        }
        EntityKind::Arc { center, start, .. } => {
            let c = s.point(center)?;
            Some((p.distance(c) - s.point(start)?.distance(c)).abs())
        }
        _ => None,
    }
}

/// The minor radius of the ellipse (centre, major end) through `p`; the distance from the
/// major axis when `p` lies beyond its ends.
fn minor_radius(center: Point2, major: Point2, p: Point2) -> f64 {
    let a = center.distance(major);
    let u = unit_or(major - center, Vec2::X);
    let q = p - center;
    let (x, y) = (q.dot(u), q.dot(u.perp()).abs());
    let rest = 1.0 - (x / a.max(TINY)).powi(2);
    if rest > 1e-9 { y / rest.sqrt() } else { y }
}

fn sketch_entity(
    s: &Sketch,
    id: EntityId,
    mut entities: Vec<EntityId>,
) -> Result<Built, SketchError> {
    let kind = &s.entity(id).ok_or(SketchError::UnknownEntity(id))?.kind;
    entities.extend(kind.children());
    entities.push(id);
    Ok(Built {
        entities,
        constraints: Vec::new(),
    })
}

/// Adds the shape of the complete list of `picks` to `s`.
///
/// `Built::entities` is in the SK-21 builder's order; for the shapes without a builder
/// (centre / 2-point circle, centre point arc, point, ellipse) it is the child points
/// followed by the curve. An overall slot with a typed length also gets a construction line
/// between its two extreme points (the last three entities), which carries the dimension.
///
/// # Errors
/// The picks are degenerate; the sketch's geometry is unchanged.
pub fn build_shape(
    kind: CreateKind,
    s: &mut Sketch,
    picks: &[Pick],
    polygon: PolygonOptions,
) -> Result<Built, SketchError> {
    use CreateKind as K;
    if picks.len() < kind.picks() {
        return Err(SketchError::NotApplicable("the shape needs more points"));
    }
    let p = |i: usize| picks[i].at;
    match kind {
        K::Rect2Point => build::rectangle_2pt(s, p(0), p(1)),
        K::Rect3Point => build::rectangle_3pt(s, p(0), p(1), p(2)),
        K::RectCenter => build::rectangle_center(s, p(0), p(1)),
        K::CircleCenter | K::Circle2Point => {
            let (c, r) = if kind == K::CircleCenter {
                (p(0), p(0).distance(p(1)))
            } else {
                ((p(0) + p(1)) * 0.5, p(0).distance(p(1)) * 0.5)
            };
            if r <= TINY {
                return Err(SketchError::Degenerate("circle has zero radius"));
            }
            let id = s.add_circle(c, r)?;
            sketch_entity(s, id, Vec::new())
        }
        K::Circle3Point => build::circle_3pt(s, p(0), p(1), p(2)),
        K::Circle2Tangent => {
            let (Some(a), Some(b)) = (picks[0].curve, picks[1].curve) else {
                return Err(SketchError::NotApplicable("two curves are needed"));
            };
            let r = match &picks[2].typed[0] {
                Some(t) => t.value,
                None => {
                    let da = curve_distance(s, a, p(2)).ok_or(SketchError::UnknownEntity(a))?;
                    let db = curve_distance(s, b, p(2)).ok_or(SketchError::UnknownEntity(b))?;
                    (da + db) * 0.5
                }
            };
            build::circle_2_tangent(s, a, b, r, p(2))
        }
        K::Arc3Point => build::arc_3pt(s, p(0), p(2), p(1)),
        K::ArcCenter => {
            let (c, mut a, mut b) = (p(0), p(1), p(2));
            if picks[2].clockwise {
                // The start pick fixes the radius, so it is projected before swapping.
                let r = a.distance(c);
                let end = c + unit_or(b - c, Vec2::X) * r;
                (a, b) = (end, a);
            }
            let id = s.add_arc(c, a, b)?;
            sketch_entity(s, id, Vec::new())
        }
        K::ArcTangent => {
            let (Some(from), Some(at)) = (picks[0].curve, picks[0].end) else {
                return Err(SketchError::NotApplicable("pick the end of a line or arc"));
            };
            build::arc_tangent(s, from, at, p(1))
        }
        K::Polygon => build::polygon(s, p(0), p(1), polygon.sides, polygon.inscribed),
        K::SlotCenterToCenter => {
            build::slot_center_to_center(s, p(0), p(1), slot_width(kind, picks))
        }
        K::SlotCenterPoint => build::slot_center_point(s, p(0), p(1), slot_width(kind, picks)),
        K::SlotOverall => {
            let mut built = build::slot_overall(s, p(0), p(1), slot_width(kind, picks))?;
            if picks[1].typed[0].is_some()
                && let Err(e) = overall_axis(s, &mut built, p(0), p(1))
            {
                let _ = s.remove_entities(&built.entities);
                return Err(e);
            }
            Ok(built)
        }
        K::SlotArc => {
            let c = p(0);
            let (mut a, mut b) = (p(1), p(2));
            if picks[2].clockwise {
                let r = a.distance(c);
                let end = c + unit_or(b - c, Vec2::X) * r;
                (a, b) = (end, a);
            }
            build::slot_arc(s, c, a, b, slot_width(kind, picks))
        }
        K::Point => {
            let id = s.add_point(p(0))?;
            Ok(Built {
                entities: vec![id],
                constraints: Vec::new(),
            })
        }
        K::Ellipse => {
            let b = minor_radius(p(0), p(1), p(2));
            if b <= TINY {
                return Err(SketchError::Degenerate("ellipse has zero minor radius"));
            }
            let id = s.add_ellipse(p(0), p(1), b)?;
            sketch_entity(s, id, Vec::new())
        }
    }
}

/// The construction line between the extreme points `a`, `b` of an overall slot.
fn overall_axis(
    s: &mut Sketch,
    built: &mut Built,
    a: Point2,
    b: Point2,
) -> Result<(), SketchError> {
    // slot_core order: centres, 4 end points, 2 lines, right arc (index 8), left arc (9).
    let (c1, c2, right, left) = (
        built.entities[0],
        built.entities[1],
        built.entities[8],
        built.entities[9],
    );
    let pa = s.add_point(a)?;
    built.entities.push(pa);
    let pb = s.add_point(b)?;
    built.entities.push(pb);
    let axis = s.add_line_between(pa, pb)?;
    built.entities.push(axis);
    s.set_construction(axis, true)?;
    for (point, curve) in [(pa, left), (pb, right), (c1, axis), (c2, axis)] {
        built
            .constraints
            .push(s.add_constraint(ConstraintKind::PointOnCurve { point, curve })?);
    }
    Ok(())
}

// ===== constraints and dimensions =====

/// Where the inference of a pick applies in the new shape.
#[derive(Copy, Clone)]
enum Target {
    /// The pick became this point entity.
    Point(EntityId),
    /// The picked position lies on this curve: a snapped point goes onto it.
    OnCurve(EntityId),
    /// The picked position is the midpoint of this line.
    Midpoint(EntityId),
    /// This point entity, only if the pick lies at its position (arc ends are projected).
    IfThere(EntityId),
    None,
}

/// The existing point a pick snapped to.
fn snapped_point(pick: &Pick) -> Option<EntityId> {
    let s = pick.inference.snap.as_ref()?;
    matches!(s.kind, SnapKind::Endpoint | SnapKind::Center)
        .then(|| s.entities.first().copied())
        .flatten()
}

fn arc_points(s: &Sketch, arc: EntityId) -> Option<(EntityId, EntityId, EntityId)> {
    match s.entity(arc)?.kind {
        EntityKind::Arc { center, start, end } => Some((center, start, end)),
        _ => None,
    }
}

fn targets(
    kind: CreateKind,
    picks: &[Pick],
    e: &[EntityId],
    s: &Sketch,
    polygon: PolygonOptions,
) -> [Target; 4] {
    use CreateKind as K;
    use Target as T;
    let n = T::None;
    let at = |i: usize| e.get(i).copied();
    let pt = |i: usize| at(i).map_or(n, T::Point);
    let on = |i: usize| at(i).map_or(n, T::OnCurve);
    let last = e.last().copied();
    match kind {
        K::Rect2Point => [pt(0), pt(2), n, n],
        K::RectCenter => [pt(9), pt(2), n, n],
        K::Rect3Point => [pt(0), pt(1), on(6), n],
        K::CircleCenter => [pt(0), on(1), n, n],
        K::Circle2Point => [on(1), on(1), n, n],
        K::Circle3Point => [on(1), on(1), on(1), n],
        K::Circle2Tangent => [n; 4],
        K::Arc3Point => {
            // arc_3pt: centre, start, end, arc; it starts at pick 0 when it turns left
            // from pick 0 through pick 2 to pick 1.
            let (a, b, t) = (picks[0].at, picks[1].at, picks[2].at);
            let ccw = (t - a).perp_dot(b - t) > 0.0;
            let (p0, p1) = if ccw { (1, 2) } else { (2, 1) };
            [pt(p0), pt(p1), on(3), n]
        }
        K::ArcCenter | K::SlotArc => {
            let cw = picks[2].clockwise;
            let (start, end) = if kind == K::ArcCenter {
                match last.and_then(|a| arc_points(s, a)) {
                    Some((_, a, b)) => (Some(a), Some(b)),
                    None => (None, None),
                }
            } else {
                (at(1), at(2))
            };
            let (from, to) = if cw { (end, start) } else { (start, end) };
            [pt(0), from.map_or(n, T::Point), to.map_or(n, T::IfThere), n]
        }
        K::ArcTangent => {
            let from = picks[0].curve.zip(picks[0].end);
            let shared = from.and_then(|(c, end)| s.entity(c)?.kind.endpoint(end));
            let far = last
                .and_then(|a| arc_points(s, a))
                .map(|(_, a, b)| if Some(a) == shared { b } else { a });
            [n, far.map_or(n, T::Point), n, n]
        }
        K::Polygon => {
            let second = if polygon.inscribed {
                pt(2)
            } else {
                at(2 + polygon.sides as usize).map_or(n, T::Midpoint)
            };
            [pt(0), second, n, n]
        }
        K::SlotCenterToCenter => [pt(0), pt(1), n, n],
        K::SlotOverall => [on(9), on(8), n, n],
        K::SlotCenterPoint => [pt(11), pt(1), n, n],
        K::Point => [pt(0), n, n, n],
        K::Ellipse => [pt(0), pt(1), on(2), n],
    }
}

/// The constraint that makes the segment of picks 0 and 1 horizontal (`true`) or vertical.
fn direction(kind: CreateKind, e: &[EntityId], horizontal: bool) -> Option<ConstraintKind> {
    use CreateKind as K;
    let target = match kind {
        K::Rect3Point => HvTarget::Line(*e.get(4)?),
        K::SlotCenterToCenter => HvTarget::Points(*e.first()?, *e.get(1)?),
        K::SlotOverall => HvTarget::Line(*e.get(6)?),
        K::SlotCenterPoint => HvTarget::Points(*e.get(11)?, *e.get(1)?),
        K::Ellipse => HvTarget::Points(*e.first()?, *e.get(1)?),
        _ => return None,
    };
    Some(if horizontal {
        ConstraintKind::Horizontal(target)
    } else {
        ConstraintKind::Vertical(target)
    })
}

/// The dimension of typed field `field` of pick `stage`, and a label position.
fn dimension_of(
    kind: CreateKind,
    stage: usize,
    field: Field,
    e: &[EntityId],
    s: &Sketch,
    targets: &[Target; 4],
) -> Option<DimensionKind> {
    use CreateKind as K;
    use DimensionKind as D;
    use Field as F;
    let at = |i: usize| e.get(i).copied();
    let last = e.last().copied();
    let between = |a: Option<EntityId>, b: Option<EntityId>| {
        Some(D::Distance {
            a: a?,
            b: b?,
            side: Side::Positive,
        })
    };
    Some(match (kind, stage, field) {
        (K::Rect2Point | K::RectCenter, 1, F::Width) => D::Length { line: at(4)? },
        (K::Rect2Point | K::RectCenter, 1, F::Height) => D::Length { line: at(5)? },
        (K::Rect3Point, 1, F::Length) => D::Length { line: at(4)? },
        (K::Rect3Point, 2, F::Height) => D::Length { line: at(5)? },
        (K::CircleCenter | K::Circle2Point | K::Circle3Point, _, F::Diameter) => {
            D::Diameter { curve: at(1)? }
        }
        (K::Arc3Point, 1, F::Length) => {
            let point = |t: &Target| match t {
                Target::Point(p) => Some(*p),
                _ => None,
            };
            between(point(&targets[0]), point(&targets[1]))?
        }
        (K::Arc3Point | K::ArcCenter | K::ArcTangent, _, F::Radius) => D::Radius { curve: last? },
        (K::ArcCenter, 2, F::Length) => D::ArcLength { arc: last? },
        (K::Polygon, 1, F::Radius) => D::Radius { curve: at(1)? },
        (K::SlotCenterToCenter, 1, F::Length) => between(at(0), at(1))?,
        (K::SlotCenterPoint, 1, F::Length) => between(at(11), at(1))?,
        (K::SlotOverall, 1, F::Length) => D::Length { line: at(12)? },
        (K::SlotCenterToCenter | K::SlotOverall | K::SlotCenterPoint, 2, F::Width) => {
            D::Diameter { curve: at(8)? }
        }
        // slot_arc order: 7 points, centreline arc (7), outer, inner, start cap (10).
        (K::SlotArc, 1, F::Radius) => D::Radius { curve: at(7)? },
        (K::SlotArc, 2, F::Length) => D::ArcLength { arc: at(7)? },
        (K::SlotArc, 3, F::Width) => D::Diameter { curve: at(10)? },
        (K::Ellipse, 1, F::Major) => D::EllipseMajor {
            ellipse: last.filter(|e| s.entity(*e).is_some())?,
        },
        (K::Ellipse, 2, F::Minor) => D::EllipseMinor { ellipse: last? },
        _ => return None,
    })
}

/// Where the label of a new dimension goes: beside the middle of what it measures.
fn label_pos(s: &Sketch, kind: &DimensionKind) -> Point2 {
    let anchor = |e: EntityId| {
        s.point(e).or_else(|| {
            let c = s.curve(e)?;
            let (t0, t1) = c.param_range();
            Some(c.eval(t0 + (t1 - t0) * 0.5))
        })
    };
    let pts: Vec<Point2> = kind.entities().iter().filter_map(|e| anchor(*e)).collect();
    let n = pts.len().max(1) as f64;
    pts.into_iter().fold(Point2::ZERO, |a, p| a + p) / n
}

/// The follow-up edits of a built shape: the driving dimensions of the typed values, and the
/// inferred constraints (checked by the solver before they are added).
pub fn follow_ups(
    kind: CreateKind,
    picks: &[Pick],
    entities: &[EntityId],
    builder_constraints: &[amg_sketch::ConstraintId],
    s: &Sketch,
    polygon: PolygonOptions,
) -> Inferred {
    let mut out = Inferred::default();
    let targets = targets(kind, picks, entities, s, polygon);
    for (stage, pick) in picks.iter().enumerate() {
        for (field, typed) in kind.fields(stage).iter().zip(&pick.typed) {
            let Some(t) = typed else { continue };
            if kind == CreateKind::Circle2Tangent {
                // The builder's own radius dimension takes the expression.
                if let Some(&id) = builder_constraints.last() {
                    out.edits.push(SketchEdit::SetDimension {
                        id,
                        expr: t.expr.clone(),
                    });
                }
                continue;
            }
            let Some(dim) = dimension_of(kind, stage, *field, entities, s, &targets) else {
                continue;
            };
            let label_pos = label_pos(s, &dim);
            let constraint = ConstraintKind::Dimension(Dimension {
                kind: dim,
                value: t.value,
                driving: true,
                label_pos,
            });
            out.edits.push(SketchEdit::AddConstraint {
                kind: orient_constraint(s, constraint),
                expr: Some(t.expr.clone()),
            });
        }
        // A typed pick was placed by its values, not by inference.
        if pick.typed.iter().any(Option::is_some) {
            continue;
        }
        match targets.get(stage).copied().unwrap_or(Target::None) {
            Target::Point(p) => out.checked.extend(pick.inference.constraints(p, None)),
            Target::IfThere(p) => {
                if s.point(p).is_some_and(|q| {
                    q.distance(pick.at) <= 1e-7 * q.distance(Point2::ZERO).max(1.0)
                }) {
                    out.checked.extend(pick.inference.constraints(p, None));
                }
            }
            Target::OnCurve(curve) => {
                if let Some(point) = snapped_point(pick) {
                    out.checked
                        .push(ConstraintKind::PointOnCurve { point, curve });
                }
            }
            Target::Midpoint(line) => {
                if let Some(point) = snapped_point(pick) {
                    out.checked.push(ConstraintKind::Midpoint { point, line });
                }
            }
            Target::None => {}
        }
        if kind.directional(stage)
            && let Some(h) = pick.inference.line_axis
            && let Some(k) = direction(kind, entities, h)
        {
            out.checked.push(k);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::Plane;

    fn p(x: f64, y: f64) -> Point2 {
        Point2::new(x, y)
    }

    fn placed(kind: CreateKind, picks: &[Point2], cursor: Point2, v: [Option<f64>; 2]) -> Point2 {
        let picks: Vec<Pick> = picks.iter().map(|q| Pick::at(*q)).collect();
        resolve(kind, &picks, &Pick::at(cursor), v, &Sketch::new(Plane::XY)).unwrap()
    }

    #[test]
    fn typed_values_place_the_pick() {
        use CreateKind as K;
        let q = placed(
            K::Rect2Point,
            &[p(0.0, 0.0)],
            p(-3.0, 4.0),
            [Some(10.0), Some(5.0)],
        );
        assert!((q - p(-10.0, 5.0)).length() < 1e-12);
        let q = placed(
            K::RectCenter,
            &[p(1.0, 1.0)],
            p(3.0, -4.0),
            [Some(10.0), None],
        );
        assert!((q - p(6.0, -4.0)).length() < 1e-12);
        // 3-point circle through (0,0), (6,0) with diameter 10: centre (3,4), third point
        // opposite the chord.
        let q = placed(
            K::Circle3Point,
            &[p(0.0, 0.0), p(6.0, 0.0)],
            p(3.0, 2.0),
            [Some(10.0), None],
        );
        assert!((q - p(3.0, 9.0)).length() < 1e-12, "{q:?}");
        // 3-point arc: a shallow cursor gives the minor arc on its side, a deep one the
        // major arc.
        for (cursor, want) in [(p(3.0, 0.5), p(3.0, 1.0)), (p(3.0, 20.0), p(3.0, 9.0))] {
            let q = placed(
                K::Arc3Point,
                &[p(0.0, 0.0), p(6.0, 0.0)],
                cursor,
                [Some(5.0), None],
            );
            assert!((q - want).length() < 1e-12, "{cursor:?}: {q:?}");
        }
        // Too small a radius for the chord.
        let picks = [Pick::at(p(0.0, 0.0)), Pick::at(p(6.0, 0.0))];
        assert!(
            resolve(
                K::Arc3Point,
                &picks,
                &Pick::at(p(3.0, 1.0)),
                [Some(2.0), None],
                &Sketch::new(Plane::XY)
            )
            .is_none()
        );
    }

    #[test]
    fn ellipse_minor_radius_passes_through_the_pick() {
        let b = minor_radius(p(0.0, 0.0), p(10.0, 0.0), p(6.0, 4.0));
        assert!((b - 5.0).abs() < 1e-12);
        assert!((minor_radius(p(0.0, 0.0), p(10.0, 0.0), p(0.0, 3.0)) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn every_tool_has_fields_and_hints_for_its_picks() {
        for k in CreateKind::ALL {
            for stage in 0..k.picks() {
                assert!(!k.step_hint(stage).is_empty(), "{k:?} {stage}");
                assert!(k.fields(stage).len() <= 2);
            }
            assert!(
                (0..k.picks()).any(|s| !k.fields(s).is_empty()),
                "{k:?} has heads-up input"
            );
        }
    }
}
