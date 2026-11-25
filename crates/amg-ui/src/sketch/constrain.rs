//! The geometric constraint tools (spec §13, UI-14): Fusion's twelve, each a command in the
//! SKETCH tab's Constraints group and in the S-key search.
//!
//! A tool applies to the sketch selection when it starts (several lines for Horizontal /
//! Vertical, several entities for Fix / Unfix), otherwise it collects picks: every click is
//! checked by [`resolve`] and an entity of the wrong kind is refused with a status hint
//! ("Parallel needs a line, not a circle"). A complete pick set is added through the session's
//! SK-04 check ([`SketchSession::add_constraint`](super::SketchSession::add_constraint)): a
//! redundant or conflicting constraint is not added and the status bar says so. The tool stays
//! active for the next pick set; Esc drops the picks, then leaves the tool.

use std::borrow::Cow;

use amg_features::sketch::SketchEdit;
use amg_sketch::query::{self, PickFilter};
use amg_sketch::solve::orient_constraint;
use amg_sketch::{
    ConstraintId, ConstraintKind, End, EntityId, EntityKind, HvTarget, Sketch, TangentKind,
};

use super::session::with_session;
use super::shapes::ToolDef;
use super::tools::{Cached, PICK_RADIUS_PT, cursor_on};
use crate::app::CommitBatch;
use crate::command::CmdId;
use crate::input::ViewportEvent;
use crate::select::EntityFilter;
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};

/// The twelve constraint tools (Fusion's Constraints panel, in its order).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ConstraintTool {
    /// Horizontal / Vertical: a line or two points, whichever direction is nearer.
    HorizontalVertical,
    /// Coincident: two points, or a point and a curve (point on curve).
    Coincident,
    /// Tangent: two curves (not two lines); at a shared end point, tangent there.
    Tangent,
    /// Equal: two lines, a line and an arc, two circles / arcs or two ellipses.
    Equal,
    /// Parallel: two lines.
    Parallel,
    /// Perpendicular: two lines.
    Perpendicular,
    /// Fix / Unfix: toggles `Fix` on each picked entity.
    Fix,
    /// Midpoint: a point and a line.
    Midpoint,
    /// Concentric: two circles, arcs or ellipses.
    Concentric,
    /// Collinear: two lines.
    Collinear,
    /// Symmetric: two points or two curves of the same kind, then the symmetry line.
    Symmetric,
    /// Curvature (G2): a spline or conic and a curve sharing an end point.
    Curvature,
}

/// What a constraint tool does with a pick set.
#[derive(Clone, Debug, PartialEq)]
pub enum Action {
    /// Add this constraint (after the SK-04 check).
    Add(ConstraintKind),
    /// Remove this constraint (Unfix).
    Remove(ConstraintId),
}

/// The state of a pick set.
#[derive(Clone, Debug, PartialEq)]
pub enum Step {
    /// More picks are needed; what to pick next.
    Need(&'static str),
    /// Complete.
    Apply(Action),
    /// The last pick is of the wrong kind (the status hint).
    Refuse(String),
}

macro_rules! cdef {
    ($id:literal, $label:literal, $tip:literal, $icon:ident, $kw:expr, $pin:expr) => {
        &ToolDef {
            id: CmdId($id),
            label: $label,
            tooltip: $tip,
            icon: crate::icons::ids::$icon,
            keywords: $kw,
            shortcut: None,
            pinned: $pin,
        }
    };
}

impl ConstraintTool {
    /// Every tool, in toolbar order.
    pub const ALL: [ConstraintTool; 12] = [
        ConstraintTool::HorizontalVertical,
        ConstraintTool::Coincident,
        ConstraintTool::Tangent,
        ConstraintTool::Equal,
        ConstraintTool::Parallel,
        ConstraintTool::Perpendicular,
        ConstraintTool::Fix,
        ConstraintTool::Midpoint,
        ConstraintTool::Concentric,
        ConstraintTool::Collinear,
        ConstraintTool::Symmetric,
        ConstraintTool::Curvature,
    ];

    /// The command data (Fusion has no default shortcuts for constraints).
    pub fn def(self) -> &'static ToolDef {
        match self {
            ConstraintTool::HorizontalVertical => cdef!(
                "sketch.constraint.horizontal_vertical",
                "Horizontal/Vertical",
                "Make a line, or two points, horizontal or vertical (whichever is nearer)",
                HORIZONTAL,
                &["sketch", "constraint", "horizontal", "vertical", "level"],
                true
            ),
            ConstraintTool::Coincident => cdef!(
                "sketch.constraint.coincident",
                "Coincident",
                "Join two points, or put a point on a curve",
                COINCIDENT,
                &["sketch", "constraint", "coincident", "join", "on"],
                true
            ),
            ConstraintTool::Tangent => cdef!(
                "sketch.constraint.tangent",
                "Tangent",
                "Make two curves tangent",
                TANGENT,
                &["sketch", "constraint", "tangent", "touch"],
                true
            ),
            ConstraintTool::Equal => cdef!(
                "sketch.constraint.equal",
                "Equal",
                "Equal length or radius",
                EQUAL,
                &["sketch", "constraint", "equal", "same"],
                true
            ),
            ConstraintTool::Parallel => cdef!(
                "sketch.constraint.parallel",
                "Parallel",
                "Make two lines parallel",
                PARALLEL,
                &["sketch", "constraint", "parallel"],
                true
            ),
            ConstraintTool::Perpendicular => cdef!(
                "sketch.constraint.perpendicular",
                "Perpendicular",
                "Make two lines perpendicular",
                PERPENDICULAR,
                &[
                    "sketch",
                    "constraint",
                    "perpendicular",
                    "square",
                    "right angle"
                ],
                true
            ),
            ConstraintTool::Fix => cdef!(
                "sketch.constraint.fix",
                "Fix/Unfix",
                "Lock or unlock the position of sketch geometry",
                FIX,
                &["sketch", "constraint", "fix", "unfix", "lock", "ground"],
                true
            ),
            ConstraintTool::Midpoint => cdef!(
                "sketch.constraint.midpoint",
                "Midpoint",
                "Put a point at the middle of a line",
                MIDPOINT,
                &["sketch", "constraint", "midpoint", "middle", "centre"],
                false
            ),
            ConstraintTool::Concentric => cdef!(
                "sketch.constraint.concentric",
                "Concentric",
                "Give two circles, arcs or ellipses the same centre",
                CONCENTRIC,
                &["sketch", "constraint", "concentric", "centre"],
                false
            ),
            ConstraintTool::Collinear => cdef!(
                "sketch.constraint.collinear",
                "Collinear",
                "Put two lines on one line",
                COLLINEAR,
                &["sketch", "constraint", "collinear", "align"],
                false
            ),
            ConstraintTool::Symmetric => cdef!(
                "sketch.constraint.symmetric",
                "Symmetric",
                "Mirror two points or curves about a line",
                SYMMETRIC,
                &["sketch", "constraint", "symmetric", "mirror"],
                false
            ),
            ConstraintTool::Curvature => cdef!(
                "sketch.constraint.curvature",
                "Curvature",
                "Curvature-continuous (G2) join of a spline and another curve",
                SMOOTH,
                &["sketch", "constraint", "curvature", "smooth", "g2"],
                false
            ),
        }
    }

    /// The tool with command id `id`.
    pub fn from_id(id: CmdId) -> Option<ConstraintTool> {
        ConstraintTool::ALL.into_iter().find(|t| t.def().id == id)
    }

    /// Picks a complete set takes.
    pub fn picks(self) -> usize {
        match self {
            ConstraintTool::Fix => 1,
            ConstraintTool::Symmetric => 3,
            _ => 2,
        }
    }
}

// ---- entity classes ----------------------------------------------------------------------

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Class {
    Point,
    Line,
    Circle,
    Arc,
    Ellipse,
    EllipticArc,
    Spline,
}

fn class(s: &Sketch, e: EntityId) -> Option<Class> {
    Some(match s.entity(e)?.kind {
        EntityKind::Point { .. } => Class::Point,
        EntityKind::Line { .. } => Class::Line,
        EntityKind::Circle { .. } => Class::Circle,
        EntityKind::Arc { .. } => Class::Arc,
        EntityKind::Ellipse { .. } => Class::Ellipse,
        EntityKind::EllipticArc { .. } => Class::EllipticArc,
        EntityKind::Spline(_) | EntityKind::Conic { .. } => Class::Spline,
    })
}

impl Class {
    fn name(self) -> &'static str {
        match self {
            Class::Point => "a point",
            Class::Line => "a line",
            Class::Circle => "a circle",
            Class::Arc => "an arc",
            Class::Ellipse => "an ellipse",
            Class::EllipticArc => "an elliptical arc",
            Class::Spline => "a spline",
        }
    }
    fn round(self) -> bool {
        matches!(self, Class::Circle | Class::Arc)
    }
    fn centred(self) -> bool {
        matches!(
            self,
            Class::Circle | Class::Arc | Class::Ellipse | Class::EllipticArc
        )
    }
    fn curve(self) -> bool {
        self != Class::Point
    }
}

/// The ends of `a` and `b` that are the same point, or points joined by a `Coincident`.
fn shared_end(s: &Sketch, a: EntityId, b: EntityId) -> Option<(End, End)> {
    let (ka, kb) = (&s.entity(a)?.kind, &s.entity(b)?.kind);
    let joined = |p: EntityId, q: EntityId| {
        p == q
            || s.constraints().any(|(_, c)| {
                matches!(c.kind, ConstraintKind::Coincident { a: x, b: y }
                    if (x == p && y == q) || (x == q && y == p))
            })
    };
    for ea in [End::Start, End::End] {
        for eb in [End::Start, End::End] {
            if let (Some(p), Some(q)) = (ka.endpoint(ea), kb.endpoint(eb))
                && joined(p, q)
            {
                return Some((ea, eb));
            }
        }
    }
    None
}

fn refuse(tool: ConstraintTool, needs: &str, got: Class) -> Step {
    Step::Refuse(format!(
        "{} needs {needs}, not {}",
        tool.def().label,
        got.name()
    ))
}

/// The existing `Fix` constraint of entity `e`.
fn fix_of(s: &Sketch, e: EntityId) -> Option<ConstraintId> {
    s.constraints()
        .find(|(_, c)| matches!(c.kind, ConstraintKind::Fix { entity } if entity == e))
        .map(|(id, _)| id)
}

/// Classifies the pick set `picks` (in pick order) of `tool` on `s`.
pub fn resolve(tool: ConstraintTool, s: &Sketch, picks: &[EntityId]) -> Step {
    use ConstraintTool as T;
    let mut cls = Vec::with_capacity(picks.len());
    for &e in picks {
        match class(s, e) {
            Some(c) => cls.push(c),
            None => return Step::Refuse("that entity no longer exists".to_owned()),
        }
    }
    let add = |k: ConstraintKind| Step::Apply(Action::Add(orient_constraint(s, k)));
    match (tool, picks, cls.as_slice()) {
        (_, [], _) => Step::Need(match tool {
            T::HorizontalVertical => "select a line or a point",
            T::Coincident => "select a point or a curve",
            T::Tangent | T::Curvature => "select a curve",
            T::Equal => "select a line, circle, arc or ellipse",
            T::Parallel | T::Perpendicular | T::Collinear => "select a line",
            T::Fix => "select geometry to fix or unfix",
            T::Midpoint => "select a point or a line",
            T::Concentric => "select a circle, arc or ellipse",
            T::Symmetric => "select a point or a curve",
        }),

        // ---- Horizontal / Vertical
        (T::HorizontalVertical, &[l], [Class::Line]) => {
            let Some(c) = s.curve(l) else {
                return Step::Refuse("the line has no geometry".to_owned());
            };
            let (t0, t1) = c.param_range();
            let d = c.eval(t1) - c.eval(t0);
            add(if d.x.abs() >= d.y.abs() {
                ConstraintKind::Horizontal(HvTarget::Line(l))
            } else {
                ConstraintKind::Vertical(HvTarget::Line(l))
            })
        }
        (T::HorizontalVertical, [_], [Class::Point]) => Step::Need("select a second point"),
        (T::HorizontalVertical, [_], [c]) => refuse(tool, "a line or two points", *c),
        (T::HorizontalVertical, &[a, b], [Class::Point, Class::Point]) => {
            let (Some(p), Some(q)) = (s.point(a), s.point(b)) else {
                return Step::Refuse("missing point".to_owned());
            };
            let d = q - p;
            add(if d.x.abs() >= d.y.abs() {
                ConstraintKind::Horizontal(HvTarget::Points(a, b))
            } else {
                ConstraintKind::Vertical(HvTarget::Points(a, b))
            })
        }
        (T::HorizontalVertical, [_, _], [_, c]) => refuse(tool, "a second point", *c),

        // ---- Coincident
        (T::Coincident, [_], [Class::Point]) => Step::Need("select a point or a curve"),
        (T::Coincident, [_], [_]) => Step::Need("select a point"),
        (T::Coincident, &[a, b], [Class::Point, Class::Point]) => {
            add(ConstraintKind::Coincident { a, b })
        }
        (T::Coincident, &[point, curve], [Class::Point, _])
        | (T::Coincident, &[curve, point], [_, Class::Point]) => {
            add(ConstraintKind::PointOnCurve { point, curve })
        }
        (T::Coincident, [_, _], [_, c]) => refuse(tool, "a point", *c),

        // ---- Tangent / Curvature
        (T::Tangent | T::Curvature, [_], [c]) if !c.curve() => refuse(tool, "a curve", *c),
        (T::Tangent, [_], [_]) => Step::Need("select a second curve"),
        (T::Tangent, [_, _], [Class::Line, Class::Line]) => Step::Refuse(
            "Tangent needs a curve, not two lines (use Collinear for lines)".to_owned(),
        ),
        (T::Tangent, [_, _], [_, Class::Point]) => refuse(tool, "a curve", Class::Point),
        (T::Tangent, &[a, b], [ca, cb]) => {
            let at_end = |ca: Class| matches!(ca, Class::Line | Class::Arc | Class::Spline);
            if at_end(*ca)
                && at_end(*cb)
                && let Some((a_end, b_end)) = shared_end(s, a, b)
            {
                return add(ConstraintKind::Tangent {
                    a,
                    b,
                    kind: TangentKind::AtEndpoint {
                        a_end,
                        b_end,
                        reversed: false,
                    },
                });
            }
            let ok = match (ca, cb) {
                (Class::Line, x) | (x, Class::Line) => x.round() || *x == Class::Ellipse,
                (x, y) => x.round() && y.round(),
            };
            if ok {
                add(ConstraintKind::Tangent {
                    a,
                    b,
                    kind: TangentKind::External,
                })
            } else {
                Step::Refuse(format!(
                    "Tangent of {} and {} needs a shared end point",
                    ca.name(),
                    cb.name()
                ))
            }
        }
        (T::Curvature, [_], [_]) => Step::Need("select a second curve"),
        (T::Curvature, &[a, b], [ca, cb]) => {
            if !cb.curve() {
                return refuse(tool, "a curve", *cb);
            }
            if *ca != Class::Spline && *cb != Class::Spline {
                return Step::Refuse("Curvature needs a spline or conic".to_owned());
            }
            match shared_end(s, a, b) {
                Some((a_end, b_end)) => add(ConstraintKind::Curvature {
                    a,
                    b,
                    a_end,
                    b_end,
                    reversed: false,
                }),
                None => Step::Refuse("Curvature needs curves that share an end point".to_owned()),
            }
        }

        // ---- Equal
        (T::Equal, [_], [c]) if *c == Class::Point || *c == Class::Spline => {
            refuse(tool, "a line, circle, arc or ellipse", *c)
        }
        (T::Equal, [_], [_]) => Step::Need("select a second entity of the same kind"),
        (T::Equal, &[a, b], [ca, cb]) => {
            let ok = match (ca, cb) {
                (Class::Line, Class::Line)
                | (Class::Line, Class::Arc)
                | (Class::Arc, Class::Line) => true,
                (x, y) if x.round() && y.round() => true,
                (Class::Ellipse | Class::EllipticArc, Class::Ellipse | Class::EllipticArc) => true,
                _ => false,
            };
            if ok {
                add(ConstraintKind::Equal { a, b })
            } else {
                Step::Refuse(format!(
                    "Equal needs entities of the same kind, not {} and {}",
                    ca.name(),
                    cb.name()
                ))
            }
        }

        // ---- Parallel / Perpendicular / Collinear
        (T::Parallel | T::Perpendicular | T::Collinear, [_], [Class::Line]) => {
            Step::Need("select a second line")
        }
        (T::Parallel | T::Perpendicular | T::Collinear, [_], [c])
        | (T::Parallel | T::Perpendicular | T::Collinear, [_, _], [_, c])
            if *c != Class::Line =>
        {
            refuse(tool, "a line", *c)
        }
        (T::Parallel, &[a, b], _) => add(ConstraintKind::Parallel { a, b }),
        (T::Perpendicular, &[a, b], _) => add(ConstraintKind::Perpendicular { a, b }),
        (T::Collinear, &[a, b], _) => add(ConstraintKind::Collinear { a, b }),

        // ---- Fix / Unfix
        (T::Fix, &[e], _) => match fix_of(s, e) {
            Some(id) => Step::Apply(Action::Remove(id)),
            None => add(ConstraintKind::Fix { entity: e }),
        },

        // ---- Midpoint
        (T::Midpoint, [_], [Class::Point]) => Step::Need("select a line"),
        (T::Midpoint, [_], [Class::Line]) => Step::Need("select a point"),
        (T::Midpoint, [_], [c]) => refuse(tool, "a point or a line", *c),
        (T::Midpoint, &[point, line], [Class::Point, Class::Line])
        | (T::Midpoint, &[line, point], [Class::Line, Class::Point]) => {
            add(ConstraintKind::Midpoint { point, line })
        }
        (T::Midpoint, [_, _], [Class::Point, c]) => refuse(tool, "a line", *c),
        (T::Midpoint, [_, _], [_, c]) => refuse(tool, "a point", *c),

        // ---- Concentric
        (T::Concentric, [_], [c]) | (T::Concentric, [_, _], [_, c]) if !c.centred() => {
            refuse(tool, "a circle, arc or ellipse", *c)
        }
        (T::Concentric, [_], _) => Step::Need("select a second circle, arc or ellipse"),
        (T::Concentric, &[a, b], _) => add(ConstraintKind::Concentric { a, b }),

        // ---- Symmetric
        (T::Symmetric, [_], [c]) if *c == Class::EllipticArc || *c == Class::Spline => {
            refuse(tool, "a point, line, circle, arc or ellipse", *c)
        }
        (T::Symmetric, [_], [_]) => Step::Need("select the matching entity"),
        (T::Symmetric, [_, _], [ca, cb]) if ca != cb => Step::Refuse(format!(
            "Symmetric needs {} to mirror {}, not {}",
            ca.name(),
            ca.name(),
            cb.name()
        )),
        (T::Symmetric, [_, _], _) => Step::Need("select the symmetry line"),
        (T::Symmetric, &[a, b, axis], [.., c]) => {
            if *c != Class::Line {
                return refuse(tool, "a symmetry line", *c);
            }
            if axis == a || axis == b {
                return Step::Refuse(
                    "the symmetry line can't be one of the mirrored lines".to_owned(),
                );
            }
            add(ConstraintKind::Symmetric { a, b, axis })
        }

        _ => Step::Refuse(format!("{} takes {} picks", tool.def().label, tool.picks())),
    }
}

/// The actions of `tool` on a selection made before it started: each entity for Fix and for
/// Horizontal / Vertical on lines, else the selection as one pick set. `None` when the
/// selection is not a complete, valid set (the tool then starts with no picks).
pub fn resolve_selection(
    tool: ConstraintTool,
    s: &Sketch,
    sel: &[EntityId],
) -> Option<Vec<Action>> {
    if sel.is_empty() {
        return None;
    }
    let each = match tool {
        ConstraintTool::Fix => true,
        ConstraintTool::HorizontalVertical => sel.iter().all(|e| class(s, *e) == Some(Class::Line)),
        _ => false,
    };
    if each {
        return sel
            .iter()
            .map(|e| match resolve(tool, s, &[*e]) {
                Step::Apply(a) => Some(a),
                _ => None,
            })
            .collect();
    }
    match resolve(tool, s, sel) {
        Step::Apply(a) => Some(vec![a]),
        _ => None,
    }
}

/// A constraint tool (see [`ConstraintTool`]).
#[derive(Debug)]
pub struct ConstrainTool {
    tool: ConstraintTool,
    cache: Cached,
    picks: Vec<EntityId>,
    /// Why the last pick was refused, or why the last constraint was not added.
    problem: Option<String>,
    applied: u32,
}

impl ConstrainTool {
    /// A new tool.
    pub fn new(tool: ConstraintTool, _init: &ToolInit<'_>) -> ConstrainTool {
        ConstrainTool {
            tool,
            cache: Cached::default(),
            picks: Vec::new(),
            problem: None,
            applied: 0,
        }
    }

    /// The picks so far.
    pub fn picks(&self) -> &[EntityId] {
        &self.picks
    }

    /// The refusal shown in the status bar.
    pub fn problem(&self) -> Option<&str> {
        self.problem.as_deref()
    }

    /// Pick sets applied so far.
    pub fn applied(&self) -> u32 {
        self.applied
    }

    fn hit(&self, cx: &ToolCx<'_>, ray: &amg_core::Ray3) -> Option<EntityId> {
        let (plane, sketch) = (self.cache.plane?, self.cache.sketch.as_ref()?);
        let (p, mmpp) = cursor_on(&plane, cx.camera(), ray)?;
        query::pick(sketch, p, mmpp * PICK_RADIUS_PT, PickFilter::ALL)
            .first()
            .map(|h| h.entity)
    }

    /// Shows the picks as the session selection.
    fn mirror(&self, cx: &mut ToolCx<'_>) {
        let picks = self.picks.clone();
        with_session(cx.app(), |s, _| {
            s.selected = picks.into_iter().collect();
        });
    }

    /// Applies `actions` (one undo entry): one addition goes through the SK-04 check with a
    /// refusal message; several are checked in order and the dependent ones dropped.
    fn apply(&mut self, cx: &mut ToolCx<'_>, actions: Vec<Action>) {
        let label = self.tool.def().label;
        let mut adds = Vec::new();
        let mut removes = Vec::new();
        for a in actions {
            match a {
                Action::Add(k) => adds.push(k),
                Action::Remove(id) => removes.push(SketchEdit::RemoveConstraint(id)),
            }
        }
        let res = with_session(cx.app(), |s, app| {
            if removes.is_empty() && adds.len() == 1 {
                s.add_constraint(app, label, adds.remove(0))
            } else {
                s.commit(app, label, SketchEdit::Batch(removes), adds)
                    .map(|_| ())
            }
        });
        match res {
            Some(Ok(())) => {
                self.applied += 1;
                self.problem = None;
            }
            Some(Err(e)) => self.problem = Some(format!("Not added: {e}")),
            None => {}
        }
        self.picks.clear();
    }

    fn pick(&mut self, cx: &mut ToolCx<'_>, e: EntityId) {
        let Some(sketch) = self.cache.sketch.clone() else {
            return;
        };
        if let Some(i) = self.picks.iter().position(|p| *p == e) {
            self.picks.remove(i);
            return;
        }
        self.picks.push(e);
        match resolve(self.tool, &sketch, &self.picks) {
            Step::Need(_) => self.problem = None,
            Step::Apply(a) => self.apply(cx, vec![a]),
            Step::Refuse(why) => {
                self.picks.pop();
                self.problem = Some(why);
            }
        }
    }
}

impl Tool for ConstrainTool {
    fn id(&self) -> CmdId {
        self.tool.def().id
    }

    fn label(&self) -> Cow<'static, str> {
        self.tool.def().label.into()
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::NONE
    }

    fn start(&mut self, cx: &mut ToolCx<'_>) {
        self.cache.refresh(cx);
        let sel: Vec<EntityId> = with_session(cx.app(), |s, _| {
            let v = s.selected.iter().copied().collect();
            s.selected.clear();
            s.notice = None;
            v
        })
        .unwrap_or_default();
        let Some(sketch) = self.cache.sketch.clone() else {
            return;
        };
        if let Some(actions) = resolve_selection(self.tool, &sketch, &sel) {
            self.apply(cx, actions);
        } else if !sel.is_empty() && matches!(resolve(self.tool, &sketch, &sel), Step::Need(_)) {
            // A valid start of a pick set: keep it.
            self.picks = sel;
        }
        self.mirror(cx);
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        let next = match self.cache.sketch.as_ref() {
            Some(s) => match resolve(self.tool, s, &self.picks) {
                Step::Need(n) => n,
                _ => "select geometry",
            },
            None => "select geometry",
        };
        let problem = self
            .problem
            .as_ref()
            .or(self.cache.notice.as_ref())
            .map(|p| format!(" · {p}"))
            .unwrap_or_default();
        Some(
            format!(
                "{} · {}: {next}{problem}",
                self.cache.dof_text(),
                self.tool.def().label
            )
            .into(),
        )
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        if !self.cache.refresh(cx) {
            cx.request_finish(FinishReason::Cancel);
            return;
        }
        if let Some(s) = &self.cache.sketch {
            self.picks.retain(|e| s.entity(*e).is_some());
        }
        self.mirror(cx);
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        match ev {
            ViewportEvent::Hover { ray, .. } => {
                let hit = self.hit(cx, ray);
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
                if let Some(e) = self.hit(cx, ray) {
                    self.pick(cx, e);
                    self.mirror(cx);
                }
                InputResult::Consumed
            }
            ViewportEvent::Key {
                key: egui::Key::Escape,
                pressed: true,
                ..
            } => {
                if self.picks.is_empty() {
                    cx.request_finish(FinishReason::Cancel);
                } else {
                    self.picks.clear();
                    self.problem = None;
                    self.mirror(cx);
                }
                InputResult::Consumed
            }
            _ => InputResult::Ignored,
        }
    }

    fn finish(&mut self, _reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        self.picks.clear();
        self.mirror(cx);
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::{Plane, Point2};

    fn pt(x: f64, y: f64) -> Point2 {
        Point2::new(x, y)
    }

    #[test]
    fn every_tool_names_its_next_pick_and_refuses_points_where_curves_are_needed() {
        let mut s = Sketch::new(Plane::XY);
        let p = s.add_point(pt(1.0, 1.0)).unwrap();
        for t in ConstraintTool::ALL {
            assert!(matches!(resolve(t, &s, &[]), Step::Need(_)), "{t:?}");
            assert_eq!(ConstraintTool::from_id(t.def().id), Some(t));
        }
        for t in [
            ConstraintTool::Tangent,
            ConstraintTool::Equal,
            ConstraintTool::Concentric,
            ConstraintTool::Curvature,
        ] {
            match resolve(t, &s, &[p]) {
                Step::Refuse(why) => assert!(why.contains("not a point"), "{t:?}: {why}"),
                other => panic!("{t:?}: {other:?}"),
            }
        }
    }

    #[test]
    fn horizontal_vertical_follows_the_nearer_direction() {
        let mut s = Sketch::new(Plane::XY);
        let a = s.add_line(pt(0.0, 0.0), pt(10.0, 1.0)).unwrap();
        let b = s.add_line(pt(0.0, 0.0), pt(1.0, 10.0)).unwrap();
        assert_eq!(
            resolve(ConstraintTool::HorizontalVertical, &s, &[a]),
            Step::Apply(Action::Add(ConstraintKind::Horizontal(HvTarget::Line(a))))
        );
        assert_eq!(
            resolve(ConstraintTool::HorizontalVertical, &s, &[b]),
            Step::Apply(Action::Add(ConstraintKind::Vertical(HvTarget::Line(b))))
        );
        let both = resolve_selection(ConstraintTool::HorizontalVertical, &s, &[a, b]).unwrap();
        assert_eq!(both.len(), 2);
    }

    #[test]
    fn tangent_at_a_shared_end_and_fix_toggles() {
        let mut s = Sketch::new(Plane::XY);
        let l = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
        let EntityKind::Line { end, .. } = s.entity(l).unwrap().kind else {
            unreachable!()
        };
        let a = s
            .add_arc(pt(10.0, 5.0), pt(10.0, 0.0), pt(15.0, 5.0))
            .unwrap();
        let EntityKind::Arc { start, .. } = s.entity(a).unwrap().kind else {
            unreachable!()
        };
        s.add_constraint(ConstraintKind::Coincident { a: end, b: start })
            .unwrap();
        match resolve(ConstraintTool::Tangent, &s, &[l, a]) {
            Step::Apply(Action::Add(ConstraintKind::Tangent {
                kind: TangentKind::AtEndpoint { a_end, b_end, .. },
                ..
            })) => assert_eq!((a_end, b_end), (End::End, End::Start)),
            other => panic!("{other:?}"),
        }
        let Step::Apply(Action::Add(fix)) = resolve(ConstraintTool::Fix, &s, &[l]) else {
            panic!("fix")
        };
        let id = s.add_constraint(fix).unwrap();
        assert_eq!(
            resolve(ConstraintTool::Fix, &s, &[l]),
            Step::Apply(Action::Remove(id))
        );
    }
}
