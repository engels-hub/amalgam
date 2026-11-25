//! The sketch modify tools (spec §13, UI-16) on the S03 edit operations: trim, extend and break
//! (SK-07), fillet and chamfer (SK-16), offset (SK-17), mirror, rectangular and circular
//! pattern, move / copy, rotate and scale (SK-18). Each is a command in the SKETCH tab's Modify
//! group and in the S-key search; Trim (`T`), Offset (`O`) and Move/Copy (`M`) have Fusion's
//! shortcuts.
//!
//! Every tool turns its picks into one [`ModifyOp`], which runs the `amg_sketch` operation on
//! the session's working copy and gives the dimensions it creates the typed expressions (a
//! fillet radius `r` stays bound to the parameter `r`). One op is one undo entry labelled with
//! the tool; the tool stays active for the next one, and the newest op is kept in
//! [`SketchSession::last_modify`](super::SketchSession::last_modify) for tests and scripts.
//!
//! | Tool | Picks |
//! |---|---|
//! | Trim, Extend | click the part of a curve to remove / the end to lengthen |
//! | Break | click a curve where it should split (snaps to points and intersections) |
//! | Fillet, Chamfer | click a corner point, or two curves near their corner |
//! | Offset | click a curve (its connected chain is offset), then click the side |
//! | Mirror | select objects, Enter, click the mirror line |
//! | Rectangular pattern | select objects, Enter (counts and spacings in the dialog) |
//! | Circular pattern | select objects, Enter, click the centre (point, circle or arc) |
//! | Move/Copy | select objects, Enter, click a base point and a target (or Enter: dialog X / Y) |
//! | Rotate, Scale | select objects, Enter, click the centre / base point (angle, factor in the dialog) |
//!
//! A selection made before the tool starts is taken as its objects. Esc steps back (picks,
//! then objects), then leaves the tool.

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};

use amg_core::{Dimension as Dim, Point2, Transform2, Vec2};
use amg_doc::{Expr, ExprSrc, ParamTable, UnitPrefs};
use amg_features::sketch::{SketchEdit, SketchFeature};
use amg_sketch::query::{self, PickFilter, SnapKind};
use amg_sketch::{
    ChamferSpec, ConstraintId, ConstraintKind, DimensionKind, EditResult, End, EntityId,
    EntityKind, Quantity, Sketch, SketchError, edit, offset, transform,
};

use super::infer::SNAP_RADIUS_PT;
use super::session::{Inferred, with_session};
use super::shapes::ToolDef;
use super::tools::{Cached, PICK_RADIUS_PT, cursor_on};
use crate::app::CommitBatch;
use crate::command::CmdId;
use crate::input::ViewportEvent;
use crate::module::IconId;
use crate::select::EntityFilter;
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};
use crate::widgets::ExprInput;

/// A sketch modify tool of UI-16.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ModifyKind {
    /// Removes the part of a curve between its nearest intersections (SK-07 `trim`).
    Trim,
    /// Lengthens a line or arc end to the next curve (SK-07 `extend`).
    Extend,
    /// Splits a line or arc at a point (SK-07 `break_at`).
    Break,
    /// Rounds a corner with a tangent arc and a radius dimension (SK-16 `fillet`).
    Fillet,
    /// Cuts a corner with a line (SK-16 `chamfer`).
    Chamfer,
    /// Offsets a chain of curves (SK-17 `offset`).
    Offset,
    /// Mirrors objects about a line (SK-18 `mirror`).
    Mirror,
    /// Rectangular pattern (SK-18 `pattern_rect`).
    RectPattern,
    /// Circular pattern (SK-18 `pattern_circ`).
    CircPattern,
    /// Moves objects (or a copy of them; SK-18 `transform`).
    Move,
    /// Copies objects (Move/Copy with "Create copy" on).
    Copy,
    /// Rotates objects about a point.
    Rotate,
    /// Scales objects about a point.
    Scale,
}

macro_rules! mdef {
    ($id:literal, $label:literal, $tip:literal, $icon:expr, $kw:expr, $short:expr, $pin:expr) => {
        &ToolDef {
            id: CmdId($id),
            label: $label,
            tooltip: $tip,
            icon: $icon,
            keywords: $kw,
            shortcut: $short,
            pinned: $pin,
        }
    };
}

/// The Move/Copy icon (vendored by the UI-23 set; registered by the assembly module too).
const MOVE_ICON: IconId = IconId("arrows-move");
/// The Copy icon.
const COPY_ICON: IconId = IconId("copy");

impl ModifyKind {
    /// Every tool, in toolbar order (Fusion's Modify panel).
    pub const ALL: [ModifyKind; 13] = [
        ModifyKind::Fillet,
        ModifyKind::Chamfer,
        ModifyKind::Trim,
        ModifyKind::Extend,
        ModifyKind::Break,
        ModifyKind::Offset,
        ModifyKind::Mirror,
        ModifyKind::RectPattern,
        ModifyKind::CircPattern,
        ModifyKind::Move,
        ModifyKind::Copy,
        ModifyKind::Rotate,
        ModifyKind::Scale,
    ];

    /// The command data. Trim `T`, Offset `O` and Move/Copy `M` are Fusion's shortcuts.
    pub fn def(self) -> &'static ToolDef {
        use crate::icons::ids;
        match self {
            ModifyKind::Trim => mdef!(
                "sketch.trim",
                "Trim",
                "Remove the part of a curve between its nearest intersections",
                ids::TRIM,
                &["sketch", "trim", "cut", "remove", "delete segment"],
                Some("sketch:T"),
                true
            ),
            ModifyKind::Extend => mdef!(
                "sketch.extend",
                "Extend",
                "Lengthen a line or arc to the next curve",
                ids::EXTEND,
                &["sketch", "extend", "lengthen", "stretch"],
                None,
                false
            ),
            ModifyKind::Break => mdef!(
                "sketch.break",
                "Break",
                "Split a line or arc into two at a point",
                ids::BREAK,
                &["sketch", "break", "split", "divide"],
                None,
                false
            ),
            ModifyKind::Fillet => mdef!(
                "sketch.fillet",
                "Fillet",
                "Round a corner with a tangent arc",
                ids::FILLET,
                &["sketch", "fillet", "round", "radius", "corner"],
                None,
                true
            ),
            ModifyKind::Chamfer => mdef!(
                "sketch.chamfer",
                "Chamfer",
                "Cut a corner with a line (equal distances, two distances, or distance and angle)",
                ids::CHAMFER,
                &["sketch", "chamfer", "bevel", "corner"],
                None,
                false
            ),
            ModifyKind::Offset => mdef!(
                "sketch.offset",
                "Offset",
                "Copy a chain of curves at a distance",
                ids::OFFSET,
                &["sketch", "offset", "parallel", "inset", "outset"],
                Some("sketch:O"),
                true
            ),
            ModifyKind::Mirror => mdef!(
                "sketch.mirror",
                "Mirror",
                "Mirror objects about a line (symmetric constraints)",
                ids::MIRROR,
                &["sketch", "mirror", "symmetric", "reflect", "flip"],
                None,
                false
            ),
            ModifyKind::RectPattern => mdef!(
                "sketch.pattern_rect",
                "Rectangular Pattern",
                "Copy objects in rows and columns",
                ids::PATTERN_RECT,
                &["sketch", "pattern", "rectangular", "array", "grid"],
                None,
                false
            ),
            ModifyKind::CircPattern => mdef!(
                "sketch.pattern_circ",
                "Circular Pattern",
                "Copy objects around a centre",
                ids::PATTERN_CIRC,
                &["sketch", "pattern", "circular", "polar", "array"],
                None,
                false
            ),
            ModifyKind::Move => mdef!(
                "sketch.move",
                "Move/Copy",
                "Move objects, or a copy of them, from a base point to a target",
                MOVE_ICON,
                &["sketch", "move", "copy", "translate"],
                Some("sketch:M"),
                false
            ),
            ModifyKind::Copy => mdef!(
                "sketch.copy",
                "Copy",
                "Copy objects from a base point to a target",
                COPY_ICON,
                &["sketch", "copy", "duplicate"],
                None,
                false
            ),
            ModifyKind::Rotate => mdef!(
                "sketch.rotate",
                "Rotate",
                "Rotate objects about a point",
                ids::ROTATE,
                &["sketch", "rotate", "turn", "angle"],
                None,
                false
            ),
            ModifyKind::Scale => mdef!(
                "sketch.scale",
                "Sketch Scale",
                "Scale objects about a point (dimensions scale with them)",
                ids::SCALE,
                &["sketch", "scale", "resize", "enlarge", "shrink"],
                None,
                false
            ),
        }
    }

    /// The tool of command `id`.
    pub fn from_id(id: CmdId) -> Option<ModifyKind> {
        ModifyKind::ALL.into_iter().find(|k| k.def().id == id)
    }

    /// Tools that work on a set of selected objects.
    pub fn takes_objects(self) -> bool {
        matches!(
            self,
            ModifyKind::Mirror
                | ModifyKind::RectPattern
                | ModifyKind::CircPattern
                | ModifyKind::Move
                | ModifyKind::Copy
                | ModifyKind::Rotate
                | ModifyKind::Scale
        )
    }
}

// ===== operations =====

/// One modify operation with everything it needs: what the tools hand to the session, and what
/// a test replays on a copy of the sketch.
#[derive(Clone, Debug)]
pub enum ModifyOp {
    /// SK-07 `trim`.
    Trim {
        /// The curve.
        entity: EntityId,
        /// Where it was clicked.
        pick: Point2,
    },
    /// SK-07 `extend`.
    Extend {
        /// The line or arc.
        entity: EntityId,
        /// Near the end to lengthen.
        pick: Point2,
    },
    /// SK-07 `break_at`.
    Break {
        /// The line or arc.
        entity: EntityId,
        /// The split point.
        at: Point2,
    },
    /// SK-16 `fillet`; the radius dimension gets `expr`.
    Fillet {
        /// First curve.
        a: EntityId,
        /// Second curve.
        b: EntityId,
        /// Picks the fillet among the candidates.
        near: Point2,
        /// Radius, mm.
        radius: f64,
        /// The radius expression.
        expr: Option<Expr>,
    },
    /// SK-16 `chamfer`; its dimensions get `exprs` (lengths in leg order, then the angle).
    Chamfer {
        /// First curve.
        a: EntityId,
        /// Second curve.
        b: EntityId,
        /// Picks the corner.
        near: Point2,
        /// Distances / angle.
        spec: ChamferSpec,
        /// First distance, second distance or angle.
        exprs: [Option<Expr>; 2],
    },
    /// SK-17 `offset`; with `constrain` its distance dimensions get `expr`.
    Offset {
        /// The curves (one or more connected chains).
        chain: Vec<EntityId>,
        /// Distance, mm.
        distance: f64,
        /// A point on the side to offset to.
        side: Point2,
        /// Tie the copy to the source with an offset dimension.
        constrain: bool,
        /// The distance expression.
        expr: Option<Expr>,
    },
    /// SK-18 `mirror`.
    Mirror {
        /// Objects.
        items: Vec<EntityId>,
        /// The mirror line.
        axis: EntityId,
        /// Symmetric constraints between source and image.
        constrain: bool,
    },
    /// SK-18 `pattern_rect`.
    RectPattern {
        /// Objects.
        items: Vec<EntityId>,
        /// First direction.
        dir_a: Vec2,
        /// Instances along it (the source included).
        n_a: u32,
        /// Spacing, mm.
        d_a: f64,
        /// Second direction, instances and spacing.
        dir_b: Option<(Vec2, u32, f64)>,
        /// Tie the copies to the source.
        constrain: bool,
    },
    /// SK-18 `pattern_circ`.
    CircPattern {
        /// Objects.
        items: Vec<EntityId>,
        /// The centre: a point, circle or arc.
        center: EntityId,
        /// Instances (the source included).
        n: u32,
        /// Total angle, rad (a full turn spaces them evenly).
        total_angle: f64,
        /// Tie the copies to the source.
        constrain: bool,
    },
    /// SK-18 `transform` (move, rotate, scale; in place or as a copy).
    Transform {
        /// Objects.
        items: Vec<EntityId>,
        /// The in-plane transform.
        transform: Transform2,
        /// Leave the source and create a copy.
        copy: bool,
    },
}

impl ModifyOp {
    /// Runs the operation on `f` (through `SketchFeature::edit_sketch`, so dimension
    /// expressions stay in step) and gives its new dimensions their expressions.
    ///
    /// # Errors
    /// The S03 operation's error; `f` is unchanged.
    pub fn apply(&self, f: &mut SketchFeature) -> Result<EditResult, SketchError> {
        let mut work = f.clone();
        let (res, exprs) = work.edit_sketch(|s| self.run(s))?;
        for (id, expr) in exprs {
            work.apply(SketchEdit::SetDimension { id, expr })?;
        }
        *f = work;
        Ok(res)
    }

    /// The S03 call, and the expressions of the dimensions it created.
    fn run(&self, s: &mut Sketch) -> Result<(EditResult, Vec<(ConstraintId, Expr)>), SketchError> {
        let none = Vec::new();
        Ok(match self {
            ModifyOp::Trim { entity, pick } => (edit::trim(s, *entity, *pick)?, none),
            ModifyOp::Extend { entity, pick } => (edit::extend(s, *entity, *pick)?, none),
            ModifyOp::Break { entity, at } => (edit::break_at(s, *entity, *at)?, none),
            ModifyOp::Fillet {
                a,
                b,
                near,
                radius,
                expr,
            } => {
                let res = edit::fillet(s, *a, *b, *near, *radius)?;
                let dims = new_dimensions(s, &res.constraints_added);
                let exprs = dims
                    .into_iter()
                    .filter(|(_, k)| matches!(k, DimensionKind::Radius { .. }))
                    .zip(expr.iter())
                    .map(|((id, _), e)| (id, e.clone()))
                    .collect();
                (res, exprs)
            }
            ModifyOp::Chamfer {
                a,
                b,
                near,
                spec,
                exprs,
            } => {
                let res = edit::chamfer(s, *a, *b, *near, *spec)?;
                let dims = new_dimensions(s, &res.constraints_added);
                let lengths = dims
                    .iter()
                    .filter(|(_, k)| k.quantity() == Quantity::Length)
                    .map(|(id, _)| *id);
                let angles = dims
                    .iter()
                    .filter(|(_, k)| k.quantity() == Quantity::Angle)
                    .map(|(id, _)| *id);
                let ids: Vec<ConstraintId> = match spec {
                    ChamferSpec::Equal(_) => lengths.take(1).collect(),
                    ChamferSpec::TwoDistances(..) => lengths.take(2).collect(),
                    ChamferSpec::DistanceAngle(..) => lengths.take(1).chain(angles).collect(),
                };
                let out = ids
                    .into_iter()
                    .zip(exprs.iter())
                    .filter_map(|(id, e)| Some((id, e.clone()?)))
                    .collect();
                (res, out)
            }
            ModifyOp::Offset {
                chain,
                distance,
                side,
                constrain,
                expr,
            } => {
                let r = offset::offset(s, chain, *distance, *side, *constrain)?;
                let exprs = match expr {
                    Some(e) => r.dimensions.iter().map(|d| (*d, e.clone())).collect(),
                    None => Vec::new(),
                };
                let res = EditResult {
                    created: r.created,
                    removed: r.removed,
                    constraints_added: r.constraints_added,
                    constraints_removed: r.constraints_removed,
                };
                (res, exprs)
            }
            ModifyOp::Mirror {
                items,
                axis,
                constrain,
            } => (transform::mirror(s, items, *axis, *constrain)?, none),
            ModifyOp::RectPattern {
                items,
                dir_a,
                n_a,
                d_a,
                dir_b,
                constrain,
            } => (
                transform::pattern_rect(s, items, *dir_a, *n_a, *d_a, *dir_b, *constrain)?,
                none,
            ),
            ModifyOp::CircPattern {
                items,
                center,
                n,
                total_angle,
                constrain,
            } => (
                transform::pattern_circ(s, items, *center, *n, *total_angle, *constrain)?,
                none,
            ),
            ModifyOp::Transform {
                items,
                transform: t,
                copy,
            } => (transform::transform(s, items, *t, *copy)?, none),
        })
    }
}

/// The dimensions among `ids`, in order.
fn new_dimensions(s: &Sketch, ids: &[ConstraintId]) -> Vec<(ConstraintId, DimensionKind)> {
    ids.iter()
        .filter_map(|id| {
            let d = s.constraint(*id)?.kind.as_dimension()?;
            Some((*id, d.kind.clone()))
        })
        .collect()
}

// ===== geometry helpers (pure) =====

/// Point entities joined by `Coincident` constraints: each point's group representative.
fn point_groups(s: &Sketch) -> BTreeMap<EntityId, EntityId> {
    let mut parent: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    fn root(p: &BTreeMap<EntityId, EntityId>, mut e: EntityId) -> EntityId {
        while let Some(&n) = p.get(&e) {
            if n == e {
                break;
            }
            e = n;
        }
        e
    }
    for (_, c) in s.constraints() {
        if let ConstraintKind::Coincident { a, b } = c.kind {
            let (ra, rb) = (root(&parent, a), root(&parent, b));
            if ra != rb {
                let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
                parent.insert(hi, lo);
            }
        }
    }
    let keys: Vec<EntityId> = parent.keys().copied().collect();
    keys.into_iter().map(|k| (k, root(&parent, k))).collect()
}

fn group_of(groups: &BTreeMap<EntityId, EntityId>, p: EntityId) -> EntityId {
    groups.get(&p).copied().unwrap_or(p)
}

/// The line / arc ends of a curve.
fn ends(kind: &EntityKind) -> Option<(EntityId, EntityId)> {
    match kind {
        EntityKind::Line { .. } | EntityKind::Arc { .. } => {
            Some((kind.endpoint(End::Start)?, kind.endpoint(End::End)?))
        }
        _ => None,
    }
}

/// The lines and arcs that end at point `p` (or at a point coincident with it).
pub fn curves_at_point(s: &Sketch, p: EntityId) -> Vec<EntityId> {
    let groups = point_groups(s);
    let g = group_of(&groups, p);
    s.entities()
        .filter(|(_, e)| {
            ends(&e.kind)
                .is_some_and(|(a, b)| group_of(&groups, a) == g || group_of(&groups, b) == g)
        })
        .map(|(id, _)| id)
        .collect()
}

/// The connected chain of `start`: the lines and arcs joined end to end with it (shared or
/// coincident end points) and of the same construction state; a circle alone.
pub fn chain_of(s: &Sketch, start: EntityId) -> Vec<EntityId> {
    let Some(first) = s.entity(start) else {
        return Vec::new();
    };
    if matches!(first.kind, EntityKind::Circle { .. }) {
        return vec![start];
    }
    let construction = first.construction;
    let groups = point_groups(s);
    let mut by_point: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
    for (id, e) in s.entities() {
        if e.construction != construction {
            continue;
        }
        if let Some((a, b)) = ends(&e.kind) {
            for p in [a, b] {
                by_point.entry(group_of(&groups, p)).or_default().push(id);
            }
        }
    }
    let mut seen = BTreeSet::from([start]);
    let mut todo = vec![start];
    while let Some(c) = todo.pop() {
        let Some((a, b)) = s.entity(c).and_then(|e| ends(&e.kind)) else {
            continue;
        };
        for p in [a, b] {
            for n in by_point.get(&group_of(&groups, p)).into_iter().flatten() {
                if seen.insert(*n) {
                    todo.push(*n);
                }
            }
        }
    }
    seen.into_iter().collect()
}

/// Rectangular pattern directions from an angle (rad): the first along it, the second 90°
/// counter-clockwise from it.
pub fn pattern_dirs(angle: f64) -> (Vec2, Vec2) {
    let a = Vec2::from_angle(angle);
    (a, a.perp())
}

// ===== the tool =====

/// The chamfer variants of the dialog.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ChamferType {
    /// One distance on both legs.
    Equal,
    /// A distance per leg.
    TwoDistances,
    /// A distance and an angle.
    DistanceAngle,
}

/// The values of a modify tool's dialog (texts are expressions, see `ExprInput`).
#[derive(Clone, Debug, PartialEq)]
pub struct ModifyOptions {
    /// Fillet radius.
    pub radius: String,
    /// Chamfer variant.
    pub chamfer: ChamferType,
    /// Chamfer (first) distance.
    pub distance: String,
    /// Chamfer second distance.
    pub distance2: String,
    /// Chamfer angle.
    pub chamfer_angle: String,
    /// Offset distance.
    pub offset: String,
    /// Constrain the result to its source (offset, mirror, patterns).
    pub constrain: bool,
    /// Rectangular pattern: instances along the first direction.
    pub columns: u32,
    /// Rectangular pattern: spacing along the first direction.
    pub column_spacing: String,
    /// Rectangular pattern: instances along the second direction.
    pub rows: u32,
    /// Rectangular pattern: spacing along the second direction.
    pub row_spacing: String,
    /// Rectangular pattern: angle of the first direction from the sketch x axis.
    pub direction: String,
    /// Circular pattern: instances.
    pub count: u32,
    /// Circular pattern: total angle.
    pub total_angle: String,
    /// Move by typed values: x.
    pub dx: String,
    /// Move by typed values: y.
    pub dy: String,
    /// Rotation angle.
    pub angle: String,
    /// Scale factor.
    pub factor: String,
    /// Move / rotate / scale a copy.
    pub copy: bool,
}

impl Default for ModifyOptions {
    fn default() -> Self {
        ModifyOptions {
            radius: "2 mm".into(),
            chamfer: ChamferType::Equal,
            distance: "2 mm".into(),
            distance2: "2 mm".into(),
            chamfer_angle: "45 deg".into(),
            offset: "2 mm".into(),
            constrain: true,
            columns: 2,
            column_spacing: "10 mm".into(),
            rows: 1,
            row_spacing: "10 mm".into(),
            direction: "0 deg".into(),
            count: 3,
            total_angle: "360 deg".into(),
            dx: "0 mm".into(),
            dy: "0 mm".into(),
            angle: "90 deg".into(),
            factor: "2".into(),
            copy: false,
        }
    }
}

/// Accessible names of the dialog fields (tests and scripts type into them).
pub mod field {
    /// Fillet radius.
    pub const RADIUS: &str = "Fillet radius";
    /// Chamfer distance.
    pub const DISTANCE: &str = "Chamfer distance";
    /// Chamfer second distance.
    pub const DISTANCE2: &str = "Second distance";
    /// Chamfer angle.
    pub const CHAMFER_ANGLE: &str = "Chamfer angle";
    /// Offset distance.
    pub const OFFSET: &str = "Offset distance";
    /// Column spacing.
    pub const COLUMN_SPACING: &str = "Column spacing";
    /// Row spacing.
    pub const ROW_SPACING: &str = "Row spacing";
    /// Pattern direction.
    pub const DIRECTION: &str = "Pattern direction";
    /// Circular pattern angle.
    pub const TOTAL_ANGLE: &str = "Total angle";
    /// Move x.
    pub const DX: &str = "X distance";
    /// Move y.
    pub const DY: &str = "Y distance";
    /// Rotation angle.
    pub const ANGLE: &str = "Rotation angle";
    /// Scale factor.
    pub const FACTOR: &str = "Scale factor";
}

/// Where a tool is in its pick sequence.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Stage {
    /// Collecting objects (object tools) or the first pick.
    First,
    /// The second pick (mirror line, centre, base point, offset side, second curve).
    Second,
    /// The third pick (move target).
    Third,
}

/// A modify tool (see [`ModifyKind`]).
#[derive(Debug)]
pub struct ModifyTool {
    kind: ModifyKind,
    cache: Cached,
    stage: Stage,
    /// Selected objects (object tools).
    items: Vec<EntityId>,
    /// The first picked curve and where (fillet / chamfer by two curves, offset).
    first: Option<(EntityId, Point2)>,
    /// The base point of a move.
    base: Option<Point2>,
    /// Dialog values.
    pub options: ModifyOptions,
    problem: Option<String>,
    applied: u32,
    mmpp: f64,
}

/// A number from an expression text, with the expression.
fn eval(
    params: &ParamTable,
    prefs: &UnitPrefs,
    text: &str,
    dim: Dim,
) -> Result<(f64, Expr), String> {
    let expr = Expr::Inline(ExprSrc(text.trim().to_owned()));
    params
        .resolve(&expr, dim, prefs)
        .map(|q| (q.value, expr))
        .map_err(|e| e.to_string())
}

impl ModifyTool {
    /// A new tool.
    pub fn new(kind: ModifyKind, _init: &ToolInit<'_>) -> ModifyTool {
        ModifyTool {
            kind,
            cache: Cached::default(),
            stage: Stage::First,
            items: Vec::new(),
            first: None,
            base: None,
            options: ModifyOptions {
                copy: kind == ModifyKind::Copy,
                ..ModifyOptions::default()
            },
            problem: None,
            applied: 0,
            mmpp: 1.0,
        }
    }

    /// The tool kind.
    pub fn kind(&self) -> ModifyKind {
        self.kind
    }

    /// Operations applied so far.
    pub fn applied(&self) -> u32 {
        self.applied
    }

    /// The selected objects.
    pub fn items(&self) -> &[EntityId] {
        &self.items
    }

    /// Why the last pick or operation was refused.
    pub fn problem(&self) -> Option<&str> {
        self.problem.as_deref()
    }

    fn reset(&mut self) {
        self.stage = Stage::First;
        self.items.clear();
        self.first = None;
        self.base = None;
    }

    /// The cursor on the sketch plane (and the mm per point there).
    fn cursor(&mut self, cx: &ToolCx<'_>, ray: &amg_core::Ray3) -> Option<Point2> {
        let plane = self.cache.plane?;
        let (p, mmpp) = cursor_on(&plane, cx.camera(), ray)?;
        self.mmpp = mmpp;
        Some(p)
    }

    /// The entity under `p` (points first unless `curves_only`).
    fn hit(&self, p: Point2, filter: PickFilter) -> Option<EntityId> {
        let sketch = self.cache.sketch.as_ref()?;
        query::pick(sketch, p, self.mmpp * PICK_RADIUS_PT, filter)
            .first()
            .map(|h| h.entity)
    }

    /// `p`, moved onto an existing point, a curve's centre / midpoint or an intersection
    /// within the snap radius (exact positions for base points, centres and break points).
    fn snap(&self, p: Point2) -> Point2 {
        let Some(sketch) = self.cache.sketch.as_ref() else {
            return p;
        };
        query::snap_candidates(sketch, p, self.mmpp * SNAP_RADIUS_PT, None)
            .into_iter()
            .find(|c| {
                matches!(
                    c.kind,
                    SnapKind::Endpoint
                        | SnapKind::Center
                        | SnapKind::Midpoint
                        | SnapKind::Intersection
                )
            })
            .map_or(p, |c| c.point)
    }

    /// The dialog values as numbers.
    fn values(&self, cx: &ToolCx<'_>) -> (ParamTable, UnitPrefs) {
        let env = cx.env();
        (
            env.design.params().clone(),
            env.design.meta().default_units.clone(),
        )
    }

    /// Shows the objects (or the first pick) as the session selection.
    fn show_selection(&self, cx: &mut ToolCx<'_>) {
        let mut sel: BTreeSet<EntityId> = self.items.iter().copied().collect();
        if let Some((e, _)) = self.first {
            sel.insert(e);
        }
        with_session(cx.app(), |s, _| s.selected = sel);
    }

    /// Applies `op` (one undo entry); the tool goes back to its first pick.
    fn apply(&mut self, cx: &mut ToolCx<'_>, op: ModifyOp) {
        let label = self.kind.def().label;
        let res = with_session(cx.app(), |s, app| {
            let r = s.commit_op(app, label, |f| op.apply(f), |_, _| Inferred::default());
            if r.is_ok() {
                s.last_modify = Some(op);
                s.notice = None;
            }
            r
        });
        match res {
            Some(Ok(_)) => {
                self.applied += 1;
                self.problem = None;
                self.reset();
            }
            Some(Err(e)) => {
                self.problem = Some(format!("Not applied: {e}"));
                // Keep the objects; drop the pick that failed.
                self.first = None;
                self.base = None;
                if !self.kind.takes_objects() {
                    self.stage = Stage::First;
                }
            }
            None => {}
        }
    }

    fn corner_op(
        &self,
        cx: &ToolCx<'_>,
        a: EntityId,
        b: EntityId,
        near: Point2,
    ) -> Option<ModifyOp> {
        let (params, prefs) = self.values(cx);
        let o = &self.options;
        if self.kind == ModifyKind::Fillet {
            let (r, e) = eval(&params, &prefs, &o.radius, Dim::LENGTH).ok()?;
            return Some(ModifyOp::Fillet {
                a,
                b,
                near,
                radius: r,
                expr: Some(e),
            });
        }
        let (d, e1) = eval(&params, &prefs, &o.distance, Dim::LENGTH).ok()?;
        let (spec, e2) = match o.chamfer {
            ChamferType::Equal => (ChamferSpec::Equal(d), None),
            ChamferType::TwoDistances => {
                let (d2, e2) = eval(&params, &prefs, &o.distance2, Dim::LENGTH).ok()?;
                (ChamferSpec::TwoDistances(d, d2), Some(e2))
            }
            ChamferType::DistanceAngle => {
                let (a, e2) = eval(&params, &prefs, &o.chamfer_angle, Dim::ANGLE).ok()?;
                (ChamferSpec::DistanceAngle(d, a), Some(e2))
            }
        };
        Some(ModifyOp::Chamfer {
            a,
            b,
            near,
            spec,
            exprs: [Some(e1), e2],
        })
    }

    /// A click at `p` in the first stage.
    fn first_pick(&mut self, cx: &mut ToolCx<'_>, p: Point2) {
        let Some(sketch) = self.cache.sketch.clone() else {
            return;
        };
        let curves = PickFilter {
            points: false,
            ..PickFilter::ALL
        };
        match self.kind {
            ModifyKind::Trim | ModifyKind::Extend => {
                let Some(e) = self.hit(p, curves) else {
                    self.problem = Some("click a curve".into());
                    return;
                };
                let op = if self.kind == ModifyKind::Trim {
                    ModifyOp::Trim { entity: e, pick: p }
                } else {
                    ModifyOp::Extend { entity: e, pick: p }
                };
                self.apply(cx, op);
            }
            ModifyKind::Break => {
                let Some(e) = self.hit(p, curves) else {
                    self.problem = Some("click a line or arc".into());
                    return;
                };
                let at = self.snap(p);
                let at = sketch.curve(e).map_or(at, |c| c.closest(at).1);
                self.apply(cx, ModifyOp::Break { entity: e, at });
            }
            ModifyKind::Fillet | ModifyKind::Chamfer => {
                let Some(e) = self.hit(p, PickFilter::ALL) else {
                    self.problem = Some("click a corner or a curve".into());
                    return;
                };
                if let Some(at) = sketch.point(e) {
                    let cs = curves_at_point(&sketch, e);
                    if cs.len() != 2 {
                        self.problem = Some(format!(
                            "a corner joins two curves; this point has {}",
                            cs.len()
                        ));
                        return;
                    }
                    match self.corner_op(cx, cs[0], cs[1], at) {
                        Some(op) => self.apply(cx, op),
                        None => self.problem = Some("check the values in the dialog".into()),
                    }
                    return;
                }
                if let Some((a, pa)) = self.first.take() {
                    if a == e {
                        return;
                    }
                    let near = pa + (p - pa) * 0.5;
                    match self.corner_op(cx, a, e, near) {
                        Some(op) => self.apply(cx, op),
                        None => self.problem = Some("check the values in the dialog".into()),
                    }
                } else {
                    self.first = Some((e, p));
                    self.problem = None;
                }
            }
            ModifyKind::Offset => {
                let Some(e) = self.hit(p, curves) else {
                    self.problem = Some("click a line, arc or circle".into());
                    return;
                };
                self.first = Some((e, p));
                self.items = chain_of(&sketch, e);
                self.stage = Stage::Second;
                self.problem = None;
            }
            _ => {
                // Object tools: toggle the object under the cursor.
                let Some(e) = self.hit(p, PickFilter::ALL) else {
                    return;
                };
                if let Some(i) = self.items.iter().position(|x| *x == e) {
                    self.items.remove(i);
                } else {
                    self.items.push(e);
                }
                self.problem = None;
            }
        }
    }

    /// Enter: objects done (object tools), or apply typed values.
    fn next(&mut self, cx: &mut ToolCx<'_>) {
        if !self.kind.takes_objects() {
            return;
        }
        if self.items.is_empty() {
            self.problem = Some("select objects first".into());
            return;
        }
        let (params, prefs) = self.values(cx);
        let o = self.options.clone();
        match (self.kind, self.stage) {
            (ModifyKind::RectPattern, _) => {
                let get = |t: &str, d| eval(&params, &prefs, t, d).map(|v| v.0);
                let vals = (
                    get(&o.column_spacing, Dim::LENGTH),
                    get(&o.row_spacing, Dim::LENGTH),
                    get(&o.direction, Dim::ANGLE),
                );
                let (Ok(da), Ok(db), Ok(dir)) = vals else {
                    self.problem = Some("check the values in the dialog".into());
                    return;
                };
                let (ua, ub) = pattern_dirs(dir);
                let op = ModifyOp::RectPattern {
                    items: self.items.clone(),
                    dir_a: ua,
                    n_a: o.columns.max(1),
                    d_a: da,
                    dir_b: (o.rows > 1).then_some((ub, o.rows, db)),
                    constrain: o.constrain,
                };
                self.apply(cx, op);
            }
            (ModifyKind::Move | ModifyKind::Copy, Stage::Second) => {
                // Enter at the base point stage: move by the typed X / Y.
                let dx = eval(&params, &prefs, &o.dx, Dim::LENGTH);
                let dy = eval(&params, &prefs, &o.dy, Dim::LENGTH);
                let (Ok((dx, _)), Ok((dy, _))) = (dx, dy) else {
                    self.problem = Some("check the values in the dialog".into());
                    return;
                };
                let op = ModifyOp::Transform {
                    items: self.items.clone(),
                    transform: transform::translation(Vec2::new(dx, dy)),
                    copy: o.copy,
                };
                self.apply(cx, op);
            }
            (_, Stage::First) => {
                self.stage = Stage::Second;
                self.problem = None;
            }
            _ => {}
        }
    }

    /// A click at `p` after the first stage.
    fn later_pick(&mut self, cx: &mut ToolCx<'_>, p: Point2) {
        let Some(sketch) = self.cache.sketch.clone() else {
            return;
        };
        let (params, prefs) = self.values(cx);
        let o = self.options.clone();
        let bad = || Some("check the values in the dialog".to_owned());
        match (self.kind, self.stage) {
            (ModifyKind::Offset, _) => {
                let Ok((d, e)) = eval(&params, &prefs, &o.offset, Dim::LENGTH) else {
                    self.problem = bad();
                    return;
                };
                let op = ModifyOp::Offset {
                    chain: self.items.clone(),
                    distance: d,
                    side: p,
                    constrain: o.constrain,
                    expr: o.constrain.then_some(e),
                };
                self.apply(cx, op);
            }
            (ModifyKind::Mirror, _) => {
                let line = self.hit(
                    p,
                    PickFilter {
                        points: false,
                        ..PickFilter::ALL
                    },
                );
                match line.filter(|l| {
                    matches!(
                        sketch.entity(*l).map(|e| &e.kind),
                        Some(EntityKind::Line { .. })
                    )
                }) {
                    Some(axis) => {
                        let op = ModifyOp::Mirror {
                            items: self.items.clone(),
                            axis,
                            constrain: o.constrain,
                        };
                        self.apply(cx, op);
                    }
                    None => self.problem = Some("click the mirror line".into()),
                }
            }
            (ModifyKind::CircPattern, _) => {
                let center = self.hit(p, PickFilter::ALL).filter(|c| {
                    matches!(
                        sketch.entity(*c).map(|e| &e.kind),
                        Some(
                            EntityKind::Point { .. }
                                | EntityKind::Circle { .. }
                                | EntityKind::Arc { .. }
                        )
                    )
                });
                let Some(center) = center else {
                    self.problem = Some("click a point, circle or arc as the centre".into());
                    return;
                };
                let Ok((total, _)) = eval(&params, &prefs, &o.total_angle, Dim::ANGLE) else {
                    self.problem = bad();
                    return;
                };
                let op = ModifyOp::CircPattern {
                    items: self.items.clone(),
                    center,
                    n: o.count,
                    total_angle: total,
                    constrain: o.constrain,
                };
                self.apply(cx, op);
            }
            (ModifyKind::Move | ModifyKind::Copy, Stage::Second) => {
                self.base = Some(self.snap(p));
                self.stage = Stage::Third;
            }
            (ModifyKind::Move | ModifyKind::Copy, Stage::Third) => {
                let Some(base) = self.base else { return };
                let target = self.snap(p);
                let op = ModifyOp::Transform {
                    items: self.items.clone(),
                    transform: transform::translation(target - base),
                    copy: o.copy,
                };
                self.apply(cx, op);
            }
            (ModifyKind::Rotate, _) => {
                let Ok((a, _)) = eval(&params, &prefs, &o.angle, Dim::ANGLE) else {
                    self.problem = bad();
                    return;
                };
                let op = ModifyOp::Transform {
                    items: self.items.clone(),
                    transform: transform::rotation_about(self.snap(p), a),
                    copy: o.copy,
                };
                self.apply(cx, op);
            }
            (ModifyKind::Scale, _) => {
                let Ok((f, _)) = eval(&params, &prefs, &o.factor, Dim::NONE) else {
                    self.problem = bad();
                    return;
                };
                let op = ModifyOp::Transform {
                    items: self.items.clone(),
                    transform: transform::scale_about(self.snap(p), f),
                    copy: o.copy,
                };
                self.apply(cx, op);
            }
            _ => {}
        }
    }

    fn step_hint(&self) -> &'static str {
        use ModifyKind as K;
        match (self.kind, self.stage) {
            (K::Trim, _) => "click the part of a curve to remove",
            (K::Extend, _) => "click near the end of a line or arc to extend",
            (K::Break, _) => "click a line or arc where it should split",
            (K::Fillet | K::Chamfer, _) if self.first.is_some() => "click the second curve",
            (K::Fillet | K::Chamfer, _) => "click a corner, or two curves",
            (K::Offset, Stage::First) => "click a curve (its chain is offset)",
            (K::Offset, _) => "click the side to offset to",
            (K::RectPattern, _) => "select objects, then Enter",
            (_, Stage::First) => "select objects, then Enter",
            (K::Mirror, _) => "click the mirror line",
            (K::CircPattern, _) => "click the centre point, circle or arc",
            (K::Move | K::Copy, Stage::Second) => "click the base point (or Enter: move by X / Y)",
            (K::Move | K::Copy, _) => "click the target point",
            (K::Rotate, _) => "click the centre of rotation",
            (K::Scale, _) => "click the base point",
        }
    }
}

impl Tool for ModifyTool {
    fn id(&self) -> CmdId {
        self.kind.def().id
    }

    fn label(&self) -> Cow<'static, str> {
        self.kind.def().label.into()
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::NONE
    }

    fn start(&mut self, cx: &mut ToolCx<'_>) {
        self.cache.refresh(cx);
        let sel: Vec<EntityId> = with_session(cx.app(), |s, _| {
            s.notice = None;
            s.selected.iter().copied().collect()
        })
        .unwrap_or_default();
        if self.kind.takes_objects() && !sel.is_empty() {
            // A selection made before the tool started is its objects (Fusion).
            self.items = sel;
            self.stage = Stage::Second;
            if self.kind == ModifyKind::RectPattern {
                self.stage = Stage::First;
            }
        }
        self.show_selection(cx);
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        let problem = self
            .problem
            .as_ref()
            .or(self.cache.notice.as_ref())
            .map(|p| format!(" · {p}"))
            .unwrap_or_default();
        Some(
            format!(
                "{} · {}: {}{problem}",
                self.cache.dof_text(),
                self.kind.def().label,
                self.step_hint()
            )
            .into(),
        )
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        if !self.cache.refresh(cx) {
            cx.request_finish(FinishReason::Cancel);
            return;
        }
        if let Some(s) = self.cache.sketch.clone() {
            self.items.retain(|e| s.entity(*e).is_some());
            if self.first.is_some_and(|(e, _)| s.entity(e).is_none()) {
                self.first = None;
            }
        }
        self.show_selection(cx);
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        match ev {
            ViewportEvent::Hover { ray, .. } => {
                let hit = self
                    .cursor(cx, ray)
                    .and_then(|p| self.hit(p, PickFilter::ALL));
                with_session(cx.app(), |s, _| s.hovered = hit);
                InputResult::Consumed
            }
            ViewportEvent::Click {
                button: egui::PointerButton::Primary,
                ray,
                ..
            } => {
                if self.cache.busy {
                    return InputResult::Consumed;
                }
                let Some(p) = self.cursor(cx, ray) else {
                    return InputResult::Consumed;
                };
                if self.stage == Stage::First {
                    self.first_pick(cx, p);
                } else {
                    self.later_pick(cx, p);
                }
                self.show_selection(cx);
                InputResult::Consumed
            }
            ViewportEvent::Key {
                key: egui::Key::Enter,
                pressed: true,
                ..
            } => {
                if self.kind.takes_objects() && !self.cache.busy {
                    self.next(cx);
                    self.show_selection(cx);
                }
                InputResult::Consumed
            }
            ViewportEvent::Key {
                key: egui::Key::Escape,
                pressed: true,
                ..
            } => {
                let idle =
                    self.stage == Stage::First && self.items.is_empty() && self.first.is_none();
                if idle {
                    cx.request_finish(FinishReason::Cancel);
                } else if self.stage == Stage::Third {
                    self.stage = Stage::Second;
                    self.base = None;
                } else if self.stage == Stage::Second && self.kind.takes_objects() {
                    self.stage = Stage::First;
                } else {
                    self.reset();
                }
                self.problem = None;
                self.show_selection(cx);
                InputResult::Consumed
            }
            _ => InputResult::Ignored,
        }
    }

    fn dialog_ui(&mut self, ui: &mut egui::Ui, cx: &mut ToolCx<'_>) -> bool {
        use ModifyKind as K;
        let (params, prefs) = self.values(cx);
        let kind = self.kind;
        let o = &mut self.options;
        let expr = |ui: &mut egui::Ui, text: &mut String, dim: Dim, name: &str| {
            ui.horizontal(|ui| {
                ui.label(format!("{name}:"));
                ExprInput::new(text, dim, &params, &prefs)
                    .id_salt(("amg_sketch_modify", name))
                    .accessible_name(name)
                    .desired_width(110.0)
                    .show(ui);
            });
        };
        match kind {
            K::Trim | K::Extend | K::Break => return false,
            K::Fillet => expr(ui, &mut o.radius, Dim::LENGTH, field::RADIUS),
            K::Chamfer => {
                ui.horizontal(|ui| {
                    ui.radio_value(&mut o.chamfer, ChamferType::Equal, "Equal distance");
                    ui.radio_value(&mut o.chamfer, ChamferType::TwoDistances, "Two distances");
                    ui.radio_value(
                        &mut o.chamfer,
                        ChamferType::DistanceAngle,
                        "Distance and angle",
                    );
                });
                expr(ui, &mut o.distance, Dim::LENGTH, field::DISTANCE);
                match o.chamfer {
                    ChamferType::Equal => {}
                    ChamferType::TwoDistances => {
                        expr(ui, &mut o.distance2, Dim::LENGTH, field::DISTANCE2);
                    }
                    ChamferType::DistanceAngle => {
                        expr(ui, &mut o.chamfer_angle, Dim::ANGLE, field::CHAMFER_ANGLE);
                    }
                }
            }
            K::Offset => {
                expr(ui, &mut o.offset, Dim::LENGTH, field::OFFSET);
                ui.checkbox(&mut o.constrain, "Constrain to source");
            }
            K::Mirror => {
                ui.checkbox(&mut o.constrain, "Constrain to source");
            }
            K::RectPattern => {
                ui.horizontal(|ui| {
                    ui.label("Columns");
                    ui.add(egui::DragValue::new(&mut o.columns).range(1..=1000));
                });
                expr(
                    ui,
                    &mut o.column_spacing,
                    Dim::LENGTH,
                    field::COLUMN_SPACING,
                );
                ui.horizontal(|ui| {
                    ui.label("Rows");
                    ui.add(egui::DragValue::new(&mut o.rows).range(1..=1000));
                });
                expr(ui, &mut o.row_spacing, Dim::LENGTH, field::ROW_SPACING);
                expr(ui, &mut o.direction, Dim::ANGLE, field::DIRECTION);
                ui.checkbox(&mut o.constrain, "Constrain to source");
            }
            K::CircPattern => {
                ui.horizontal(|ui| {
                    ui.label("Quantity");
                    ui.add(egui::DragValue::new(&mut o.count).range(2..=1000));
                });
                expr(ui, &mut o.total_angle, Dim::ANGLE, field::TOTAL_ANGLE);
                ui.checkbox(&mut o.constrain, "Constrain to source");
            }
            K::Move | K::Copy => {
                expr(ui, &mut o.dx, Dim::LENGTH, field::DX);
                expr(ui, &mut o.dy, Dim::LENGTH, field::DY);
                ui.checkbox(&mut o.copy, "Create copy");
            }
            K::Rotate => {
                expr(ui, &mut o.angle, Dim::ANGLE, field::ANGLE);
                ui.checkbox(&mut o.copy, "Create copy");
            }
            K::Scale => {
                expr(ui, &mut o.factor, Dim::NONE, field::FACTOR);
                ui.checkbox(&mut o.copy, "Create copy");
            }
        }
        true
    }

    fn finish(&mut self, _reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        self.reset();
        with_session(cx.app(), |s, _| {
            s.selected.clear();
            s.hovered = None;
        });
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::Plane;

    fn pt(x: f64, y: f64) -> Point2 {
        Point2::new(x, y)
    }

    #[test]
    fn commands_are_unique_and_found_by_id() {
        let mut ids = BTreeSet::new();
        for k in ModifyKind::ALL {
            assert!(ids.insert(k.def().id.0), "{k:?}");
            assert_eq!(ModifyKind::from_id(k.def().id), Some(k));
        }
    }

    #[test]
    fn corners_and_chains_follow_shared_and_coincident_points() {
        let mut s = Sketch::new(Plane::XY);
        let r = amg_sketch::build::rectangle_2pt(&mut s, pt(0.0, 0.0), pt(10.0, 5.0)).unwrap();
        let lines: Vec<EntityId> = r
            .entities
            .iter()
            .copied()
            .filter(|e| matches!(s.entity(*e).unwrap().kind, EntityKind::Line { .. }))
            .collect();
        assert_eq!(lines.len(), 4);
        assert_eq!(chain_of(&s, lines[0]).len(), 4);
        // Every corner joins two lines.
        let EntityKind::Line { start, .. } = s.entity(lines[0]).unwrap().kind else {
            unreachable!()
        };
        assert_eq!(curves_at_point(&s, start).len(), 2);
        // A separate line through coincident points joins the chain.
        let l = s.add_line(pt(10.0, 5.0), pt(20.0, 5.0)).unwrap();
        let EntityKind::Line { start: ls, .. } = s.entity(l).unwrap().kind else {
            unreachable!()
        };
        let corner = s
            .entities()
            .find(|(id, e)| {
                matches!(e.kind, EntityKind::Point { .. })
                    && *id != ls
                    && s.point(*id).unwrap().distance(pt(10.0, 5.0)) < 1e-9
            })
            .unwrap()
            .0;
        s.add_constraint(ConstraintKind::Coincident { a: corner, b: ls })
            .unwrap();
        assert_eq!(chain_of(&s, lines[0]).len(), 5);
        assert_eq!(curves_at_point(&s, ls).len(), 3);
        let c = s.add_circle(pt(30.0, 0.0), 2.0).unwrap();
        assert_eq!(chain_of(&s, c), vec![c]);
    }

    #[test]
    fn fillet_op_binds_the_radius_expression() {
        let mut f = SketchFeature::new(amg_features::sketch::PlaneInput::OriginXY);
        let r = f
            .edit_sketch(|s| amg_sketch::build::rectangle_2pt(s, pt(0.0, 0.0), pt(10.0, 5.0)))
            .unwrap();
        let s = &f.sketch;
        let corner = r
            .entities
            .iter()
            .copied()
            .find(|e| s.point(*e).is_some_and(|p| p.distance(pt(0.0, 0.0)) < 1e-9))
            .unwrap();
        let cs = curves_at_point(s, corner);
        let expr = Expr::Inline(ExprSrc("1.5 mm".into()));
        let op = ModifyOp::Fillet {
            a: cs[0],
            b: cs[1],
            near: pt(0.0, 0.0),
            radius: 1.5,
            expr: Some(expr.clone()),
        };
        let res = op.apply(&mut f).unwrap();
        let dims = new_dimensions(&f.sketch, &res.constraints_added);
        assert_eq!(dims.len(), 1);
        assert_eq!(f.dims.get(&dims[0].0), Some(&expr));
        assert_eq!(pattern_dirs(0.0).1, Vec2::Y);
    }
}
