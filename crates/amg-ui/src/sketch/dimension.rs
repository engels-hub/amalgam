//! Sketch Dimension (spec §13, UI-14): one "smart" tool that picks the dimension from what is
//! selected, the inline value editor and the "Create as driven dimension?" prompt.
//!
//! - **By pick** ([`smart`]): a line → its length, or the horizontal / vertical distance of its
//!   ends when the label is placed above / beside it; two points → the same by placement;
//!   a point and a line, two parallel lines → their distance; two lines that meet → the angle
//!   in the quadrant of the label; a circle → diameter; an arc → radius; an ellipse → major
//!   radius; a point or line and a circle / arc → the distance to its centre. A click on empty
//!   space places the label there.
//! - **Value**: measured from the current geometry through the public SK-02 residuals
//!   ([`measure`]), with the side / angle branch that makes it positive.
//! - **Over-constraint**: the new dimension goes through the session's SK-04 check. When it
//!   would over-constrain the sketch the [`DrivenPromptDialog`] asks "Create as driven
//!   dimension?" (QUESTIONS #16): Yes adds it as a driven dimension, No drops it.
//! - **Editing** ([`DimensionEditDialog`]): opened on a new driving dimension and by a double
//!   click on a label; an [`ExprInput`] at the label sets the value expression (a number, or
//!   an expression such as `w/2` of the user parameters). Enter commits, Esc cancels.

use std::borrow::Cow;

use amg_core::{Dimension as Dim, Point2};
use amg_doc::{Expr, ExprSrc, ParamTable, UnitPrefs};
use amg_render::{DepthMode, LabelStyle, LineStyle, OverlayList};
use amg_sketch::query::{self, PickFilter};
use amg_sketch::solve::{Residual as _, characteristic_length, constraint_residual};
use amg_sketch::{
    AngleBranch, ConstraintId, ConstraintKind, Dimension, DimensionKind, EntityId, EntityKind,
    Quantity, Side, Sketch,
};

use super::session::{quantity_dim, with_session};
use super::shapes::ToolDef;
use super::tools::{Cached, PICK_RADIUS_PT, cursor_on};
use super::view::{self, colors};
use crate::app::{AppCx, CommitBatch};
use crate::command::CmdId;
use crate::dialog::AppDialog;
use crate::input::ViewportEvent;
use crate::select::EntityFilter;
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};
use crate::widgets::ExprInput;

/// The Sketch Dimension command (`D`, Fusion's shortcut).
pub const DIMENSION_DEF: &ToolDef = &ToolDef {
    id: CmdId("sketch.dimension"),
    label: "Sketch Dimension",
    tooltip: "Dimension the selected geometry: length, distance, angle, radius or diameter, \
              chosen from the picks and where the label is placed",
    icon: crate::icons::ids::DIMENSION,
    keywords: &[
        "sketch",
        "dimension",
        "length",
        "distance",
        "angle",
        "radius",
        "diameter",
        "measure",
    ],
    shortcut: Some("sketch:D"),
    pinned: true,
};

/// Title of the over-constraint prompt.
pub const DRIVEN_TITLE: &str = "Create Driven Dimension";

/// What [`smart`] makes of a pick set.
#[derive(Clone, Debug, PartialEq)]
pub enum Smart {
    /// More picks are needed (what to pick).
    Need(&'static str),
    /// The dimension for a label at the given cursor.
    Ready(DimensionKind),
    /// The picks cannot be dimensioned (the status hint).
    Refuse(String),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Pick {
    Point,
    Line,
    /// Circle or arc, with its centre point.
    Round {
        arc: bool,
        center: EntityId,
    },
    Ellipse,
    Other(&'static str),
}

fn pick_class(s: &Sketch, e: EntityId) -> Option<Pick> {
    Some(match s.entity(e)?.kind {
        EntityKind::Point { .. } => Pick::Point,
        EntityKind::Line { .. } => Pick::Line,
        EntityKind::Circle { center, .. } => Pick::Round { arc: false, center },
        EntityKind::Arc { center, .. } => Pick::Round { arc: true, center },
        EntityKind::Ellipse { .. } => Pick::Ellipse,
        EntityKind::EllipticArc { .. } => Pick::Other("an elliptical arc"),
        EntityKind::Spline(_) | EntityKind::Conic { .. } => Pick::Other("a spline"),
    })
}

/// Horizontal, vertical or aligned, from where the label is placed relative to `a`–`b`:
/// above or below the pair (within its x range) → horizontal distance, beside it → vertical,
/// elsewhere → aligned.
fn by_placement(a: EntityId, b: EntityId, pa: Point2, pb: Point2, c: Point2) -> DimensionKind {
    let (lo, hi) = (
        Point2::new(pa.x.min(pb.x), pa.y.min(pb.y)),
        Point2::new(pa.x.max(pb.x), pa.y.max(pb.y)),
    );
    let in_x = c.x > lo.x && c.x < hi.x;
    let in_y = c.y > lo.y && c.y < hi.y;
    let side = Side::Positive;
    if in_x && !in_y {
        DimensionKind::HorizontalDistance { a, b, side }
    } else if in_y && !in_x {
        DimensionKind::VerticalDistance { a, b, side }
    } else {
        DimensionKind::Distance { a, b, side }
    }
}

fn line_points(s: &Sketch, l: EntityId) -> Option<(EntityId, EntityId, Point2, Point2)> {
    let EntityKind::Line { start, end } = s.entity(l)?.kind else {
        return None;
    };
    Some((start, end, s.point(start)?, s.point(end)?))
}

/// The dimension of pick set `picks` with its label at `cursor` (sketch coordinates).
pub fn smart(s: &Sketch, picks: &[EntityId], cursor: Point2) -> Smart {
    let mut cls = Vec::with_capacity(picks.len());
    for &e in picks {
        match pick_class(s, e) {
            Some(Pick::Other(what)) => {
                return Smart::Refuse(format!("{what} can't be dimensioned"));
            }
            Some(c) => cls.push(c),
            None => return Smart::Refuse("that entity no longer exists".to_owned()),
        }
    }
    let side = Side::Positive;
    match (picks, cls.as_slice()) {
        ([], _) => Smart::Need("select a line, circle, arc or point"),
        ([_], [Pick::Point]) => Smart::Need("select a second point, a line or a circle"),
        (&[l], [Pick::Line]) => {
            let Some((a, b, pa, pb)) = line_points(s, l) else {
                return Smart::Refuse("the line has no end points".to_owned());
            };
            let d = pb - pa;
            let len = d.length();
            if d.x.abs() <= 1e-9 * len || d.y.abs() <= 1e-9 * len {
                return Smart::Ready(DimensionKind::Length { line: l });
            }
            match by_placement(a, b, pa, pb, cursor) {
                DimensionKind::Distance { .. } => Smart::Ready(DimensionKind::Length { line: l }),
                k => Smart::Ready(k),
            }
        }
        (&[e], [Pick::Round { arc, .. }]) => Smart::Ready(if *arc {
            DimensionKind::Radius { curve: e }
        } else {
            DimensionKind::Diameter { curve: e }
        }),
        (&[e], [Pick::Ellipse]) => Smart::Ready(DimensionKind::EllipseMajor { ellipse: e }),
        (&[a, b], [Pick::Point, Pick::Point]) => match (s.point(a), s.point(b)) {
            (Some(pa), Some(pb)) => Smart::Ready(by_placement(a, b, pa, pb, cursor)),
            _ => Smart::Refuse("missing point".to_owned()),
        },
        (&[p, l], [Pick::Point, Pick::Line]) | (&[l, p], [Pick::Line, Pick::Point]) => {
            Smart::Ready(DimensionKind::Distance { a: p, b: l, side })
        }
        (&[a, b], [Pick::Line, Pick::Line]) => line_pair(s, a, b, cursor),
        (&[p, _], [Pick::Point | Pick::Line, Pick::Round { center, .. }])
        | (&[_, p], [Pick::Round { center, .. }, Pick::Point | Pick::Line]) => {
            Smart::Ready(DimensionKind::Distance {
                a: p,
                b: *center,
                side,
            })
        }
        (_, [Pick::Round { center: a, .. }, Pick::Round { center: b, .. }]) => {
            Smart::Ready(DimensionKind::Distance { a: *a, b: *b, side })
        }
        (_, [_, _]) => Smart::Refuse("these two entities can't be dimensioned together".to_owned()),
        _ => Smart::Refuse("select one or two entities".to_owned()),
    }
}

/// Distance of two parallel lines, else the angle between them in the cursor's quadrant.
fn line_pair(s: &Sketch, a: EntityId, b: EntityId, c: Point2) -> Smart {
    let (Some((_, _, a0, a1)), Some((_, _, b0, b1))) = (line_points(s, a), line_points(s, b))
    else {
        return Smart::Refuse("missing line".to_owned());
    };
    let (u, w) = (a1 - a0, b1 - b0);
    let den = u.perp_dot(w);
    if den.abs() <= 1e-9 * u.length() * w.length() {
        return Smart::Ready(DimensionKind::Distance {
            a,
            b,
            side: Side::Positive,
        });
    }
    // Where the (infinite) lines meet; measure the wedge of ±u, ±w that holds the label.
    let t = (b0 - a0).perp_dot(w) / den;
    let d = c - (a0 + u * t);
    let mut branch = AngleBranch {
        reverse_a: false,
        reverse_b: false,
    };
    for (ra, rb) in [(false, false), (true, false), (false, true), (true, true)] {
        let r1 = if ra { -u } else { u };
        let r2 = if rb { -w } else { w };
        let turn = r1.perp_dot(r2);
        if r1.perp_dot(d) * turn >= 0.0 && d.perp_dot(r2) * turn >= 0.0 {
            branch = AngleBranch {
                reverse_a: ra,
                reverse_b: rb,
            };
            break;
        }
    }
    Smart::Ready(DimensionKind::Angle { a, b, branch })
}

/// The current value of `kind` on `s` (mm or rad, signed as the SK-02 residual measures it).
pub fn measure(s: &Sketch, kind: &DimensionKind) -> Option<f64> {
    let k = ConstraintKind::Dimension(Dimension {
        kind: kind.clone(),
        value: 0.0,
        driving: true,
        label_pos: Point2::new(0.0, 0.0),
    });
    let lc = characteristic_length(s);
    let r = constraint_residual(s, &k, lc).ok()??;
    let mut v = vec![0.0; r.rows()];
    r.eval(s.params().as_slice(), &mut v);
    let m = *v.first()?;
    Some(match kind.quantity() {
        Quantity::Angle => m / lc,
        Quantity::Length => m,
    })
}

fn flip(side: Side) -> Side {
    match side {
        Side::Positive => Side::Negative,
        Side::Negative => Side::Positive,
    }
}

/// `kind` with the side (distances) or line order (angles) that gives a positive value, and
/// that value.
pub fn oriented(s: &Sketch, kind: DimensionKind) -> Option<(DimensionKind, f64)> {
    let v = measure(s, &kind)?;
    if v >= 0.0 {
        return Some((kind, v));
    }
    let other = match kind {
        DimensionKind::Distance { a, b, side } => DimensionKind::Distance {
            a,
            b,
            side: flip(side),
        },
        DimensionKind::HorizontalDistance { a, b, side } => DimensionKind::HorizontalDistance {
            a,
            b,
            side: flip(side),
        },
        DimensionKind::VerticalDistance { a, b, side } => DimensionKind::VerticalDistance {
            a,
            b,
            side: flip(side),
        },
        DimensionKind::Angle { a, b, branch } => DimensionKind::Angle {
            a: b,
            b: a,
            branch: AngleBranch {
                reverse_a: branch.reverse_b,
                reverse_b: branch.reverse_a,
            },
        },
        k => return Some((k, v)),
    };
    let w = measure(s, &other)?;
    Some((other, w))
}

/// The literal a measured value is written as (mm, or degrees for angles).
pub fn literal(value: f64, q: Quantity) -> String {
    let (v, unit) = match q {
        Quantity::Length => (value, "mm"),
        Quantity::Angle => (value.to_degrees(), "deg"),
    };
    let mut t = format!("{v:.6}");
    while t.ends_with('0') {
        t.pop();
    }
    if t.ends_with('.') {
        t.pop();
    }
    format!("{t} {unit}")
}

/// Checks a typed dimension value `src` (a bare number takes mm or degrees) and returns the
/// expression text to store.
///
/// # Errors
/// The expression's error message.
pub fn dimension_text_expr(
    params: &ParamTable,
    prefs: &UnitPrefs,
    src: &str,
    dim: Dim,
) -> Result<String, String> {
    let src = src.trim();
    if src.is_empty() {
        return Err("enter a value".to_owned());
    }
    let text = if src.parse::<f64>().is_ok() {
        let unit = if dim == Dim::ANGLE { "deg" } else { "mm" };
        format!("{src} {unit}")
    } else {
        src.to_owned()
    };
    params
        .try_eval(&text, Some(dim), prefs)
        .map(|_| text)
        .map_err(|e| e.to_string())
}

/// The Sketch Dimension tool.
#[derive(Debug, Default)]
pub struct DimensionTool {
    cache: Cached,
    picks: Vec<EntityId>,
    cursor: Option<Point2>,
    problem: Option<String>,
    /// A dimension was handed to the session; open its editor once it is added.
    waiting: bool,
    placed: u32,
}

impl DimensionTool {
    /// A new tool.
    pub fn new(_init: &ToolInit<'_>) -> DimensionTool {
        DimensionTool::default()
    }

    /// The picks so far.
    pub fn picks(&self) -> &[EntityId] {
        &self.picks
    }

    /// Dimensions placed so far (including refused ones).
    pub fn placed(&self) -> u32 {
        self.placed
    }

    fn hit(&self, cx: &ToolCx<'_>, ray: &amg_core::Ray3) -> (Option<EntityId>, Option<Point2>) {
        let (Some(plane), Some(sketch)) = (self.cache.plane, self.cache.sketch.as_ref()) else {
            return (None, None);
        };
        let Some((p, mmpp)) = cursor_on(&plane, cx.camera(), ray) else {
            return (None, None);
        };
        let e = query::pick(sketch, p, mmpp * PICK_RADIUS_PT, PickFilter::ALL)
            .first()
            .map(|h| h.entity);
        (e, Some(p))
    }

    fn mirror(&self, cx: &mut ToolCx<'_>) {
        let picks = self.picks.clone();
        with_session(cx.app(), |s, _| s.selected = picks.into_iter().collect());
    }

    /// The dimension a label at `at` would get, with its value.
    fn candidate(&self, at: Point2) -> Option<(DimensionKind, f64)> {
        let s = self.cache.sketch.as_ref()?;
        match smart(s, &self.picks, at) {
            Smart::Ready(k) => oriented(s, k),
            _ => None,
        }
    }

    fn place(&mut self, cx: &mut ToolCx<'_>, at: Point2) {
        let Some(sketch) = self.cache.sketch.clone() else {
            return;
        };
        let kind = match smart(&sketch, &self.picks, at) {
            Smart::Ready(k) => k,
            Smart::Need(n) => {
                self.problem = Some(n.to_owned());
                return;
            }
            Smart::Refuse(why) => {
                self.problem = Some(why);
                return;
            }
        };
        let Some((kind, value)) = oriented(&sketch, kind) else {
            self.problem = Some("the picks can't be measured".to_owned());
            return;
        };
        let q = kind.quantity();
        let dim = ConstraintKind::Dimension(Dimension {
            kind,
            value,
            driving: true,
            label_pos: at,
        });
        let expr = Expr::Inline(ExprSrc(literal(value, q)));
        let res = with_session(cx.app(), |s, app| {
            s.last_dimension = None;
            s.add_dimension(app, DIMENSION_DEF.label, dim, expr)
        });
        match res {
            Some(Ok(())) => {
                self.waiting = true;
                self.placed += 1;
                self.problem = None;
                self.picks.clear();
            }
            Some(Err(e)) => self.problem = Some(format!("Not added: {e}")),
            None => {}
        }
    }
}

impl Tool for DimensionTool {
    fn id(&self) -> CmdId {
        DIMENSION_DEF.id
    }

    fn label(&self) -> Cow<'static, str> {
        DIMENSION_DEF.label.into()
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::NONE
    }

    fn start(&mut self, cx: &mut ToolCx<'_>) {
        self.cache.refresh(cx);
        let sel: Vec<EntityId> = with_session(cx.app(), |s, _| {
            s.notice = None;
            s.selected.iter().copied().take(2).collect()
        })
        .unwrap_or_default();
        if let Some(s) = &self.cache.sketch
            && !matches!(smart(s, &sel, Point2::new(0.0, 0.0)), Smart::Refuse(_))
        {
            self.picks = sel;
        }
        self.mirror(cx);
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        let step = match self.cache.sketch.as_ref() {
            Some(s) => match smart(s, &self.picks, self.cursor.unwrap_or(Point2::new(0.0, 0.0))) {
                Smart::Need(n) => n,
                Smart::Ready(_) => "click to place the dimension, or select a second entity",
                Smart::Refuse(_) => "select one or two entities",
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
                "{} · {}: {step}{problem}",
                self.cache.dof_text(),
                DIMENSION_DEF.label
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
        if self.waiting && !self.cache.busy {
            self.waiting = false;
            let open = with_session(cx.app(), |s, _| {
                let id = s.last_dimension?;
                let driving = s
                    .working()
                    .sketch
                    .constraint(id)
                    .and_then(|c| c.kind.as_dimension())
                    .is_some_and(|d| d.driving);
                driving.then_some((s.serial(), id))
            })
            .flatten();
            if let Some((serial, id)) = open {
                cx.app()
                    .open_dialog(Box::new(DimensionEditDialog::new(serial, id)));
            }
        }
        self.mirror(cx);
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        match ev {
            ViewportEvent::Hover { ray, .. } => {
                let (hit, at) = self.hit(cx, ray);
                self.cursor = at;
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
                let (hit, at) = self.hit(cx, ray);
                self.cursor = at;
                match (hit, at) {
                    (Some(e), _) if self.picks.contains(&e) => {
                        self.picks.retain(|p| *p != e);
                    }
                    (Some(e), _) if self.picks.len() < 2 => {
                        self.picks.push(e);
                        if let Some(s) = &self.cache.sketch
                            && let Smart::Refuse(why) =
                                smart(s, &self.picks, at.unwrap_or(Point2::new(0.0, 0.0)))
                        {
                            self.picks.pop();
                            self.problem = Some(why);
                        } else {
                            self.problem = None;
                        }
                    }
                    (_, Some(p)) if !self.picks.is_empty() => self.place(cx, p),
                    _ => {}
                }
                self.mirror(cx);
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

    fn overlay(&mut self, out: &mut OverlayList, _cx: &ToolCx<'_>) {
        let (Some(plane), Some(at)) = (self.cache.plane, self.cursor) else {
            return;
        };
        let Some((kind, value)) = self.candidate(at) else {
            return;
        };
        if let Some(sketch) = &self.cache.sketch
            && let Some(from) = kind
                .entities()
                .first()
                .and_then(|e| super::glyphs::anchor_of(sketch, *e))
        {
            view::polyline(
                out,
                &plane,
                &[from, at],
                LineStyle {
                    width_px: 1.0,
                    color: colors::PREVIEW,
                    dash: Some((4.0, 3.0)),
                    depth: DepthMode::AlwaysOnTop,
                },
            );
        }
        out.label(
            plane.to_world(at),
            view::dimension_text(&kind, value),
            LabelStyle {
                size_pt: 12.0,
                color: colors::PREVIEW,
                offset_px: [0.0, 0.0],
                background: Some(colors::LABEL_BG),
            },
        );
    }

    fn finish(&mut self, _reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        self.picks.clear();
        self.mirror(cx);
        None
    }
}

/// Accessible name of the inline dimension value field.
pub const EDIT_FIELD_NAME: &str = "Dimension value";

/// The inline value editor at a dimension label (`ExprInput`; Enter commits, Esc cancels).
#[derive(Debug)]
pub struct DimensionEditDialog {
    /// The session it belongs to (`SketchSession::serial`).
    pub session: u32,
    /// The dimension.
    pub id: ConstraintId,
    /// The field text.
    pub text: String,
    original: Option<String>,
    /// Why the last commit was refused.
    pub error: Option<String>,
    focused_once: bool,
}

impl DimensionEditDialog {
    /// An editor for dimension `id` of session `session`.
    pub fn new(session: u32, id: ConstraintId) -> DimensionEditDialog {
        DimensionEditDialog {
            session,
            id,
            text: String::new(),
            original: None,
            error: None,
            focused_once: false,
        }
    }
}

impl AppDialog for DimensionEditDialog {
    fn title(&self) -> Cow<'static, str> {
        "Edit Dimension".into()
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        let id = self.id;
        let info = {
            let design = cx.env().design.clone();
            with_session(cx, |s, _| {
                if s.serial() != self.session {
                    return None;
                }
                let c = s.working().sketch.constraint(id)?;
                let d = c.kind.as_dimension().filter(|d| d.driving)?;
                let src = s.dimension_source(&design, id)?;
                Some((
                    *s.plane(),
                    d.label_pos,
                    quantity_dim(d.kind.quantity()),
                    src,
                    s.dimension_param(id),
                ))
            })
            .flatten()
        };
        let Some((plane, at, dim, src, param)) = info else {
            return false;
        };
        let original = self.original.get_or_insert_with(|| src.clone()).clone();
        if self.text.is_empty() && !self.focused_once {
            self.text = original.clone();
        }
        let pos = cx
            .doc_tab()
            .view
            .as_ref()
            .and_then(|v| v.project(plane.to_world(at)))
            .unwrap_or_else(|| ctx.content_rect().center());
        let (params, prefs) = {
            let env = cx.env();
            (
                env.design.params().clone(),
                env.design.meta().default_units.clone(),
            )
        };
        let field = egui::Id::new(("amg_sketch_dim_edit", self.session, id.0));
        if !self.focused_once {
            ctx.memory_mut(|m| m.request_focus(field));
        }
        let mut committed = false;
        let mut focused = false;
        egui::Area::new(egui::Id::new("amg_sketch_dim_edit_area"))
            .order(egui::Order::Foreground)
            .fixed_pos(pos - egui::vec2(70.0, 14.0))
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    let r = ExprInput::new(&mut self.text, dim, &params, &prefs)
                        .id(field)
                        .bound_param(param)
                        .accessible_name(EDIT_FIELD_NAME)
                        .desired_width(120.0)
                        .show(ui);
                    committed = r.committed;
                    focused = r.response.has_focus();
                    if let Some(e) = &self.error {
                        ui.colored_label(ui.visuals().error_fg_color, e);
                    }
                });
            });
        if focused && !self.focused_once {
            // Typing replaces the value (Fusion): select it all when the editor opens.
            self.focused_once = true;
            let n = self.text.chars().count();
            let mut st = egui::text_edit::TextEditState::load(ctx, field).unwrap_or_default();
            st.cursor.set_char_range(Some(egui::text::CCursorRange::two(
                egui::text::CCursor::new(0),
                egui::text::CCursor::new(n),
            )));
            st.store(ctx, field);
        }
        if committed {
            if self.text.trim() == original.trim() {
                return false;
            }
            let text = self.text.clone();
            match with_session(cx, |s, app| s.set_dimension_expr(app, id, &text)) {
                Some(Ok(())) => return false,
                Some(Err(e)) => {
                    self.error = Some(e);
                    ctx.memory_mut(|m| m.request_focus(field));
                    return true;
                }
                None => return false,
            }
        }
        // Esc (the field gave up the focus without a commit) closes the editor.
        !(self.focused_once && !focused)
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

/// "Create as driven dimension?": shown while the session has a
/// [`DrivenOffer`](super::session::DrivenOffer).
#[derive(Debug, Default)]
pub struct DrivenPromptDialog {
    /// The session it belongs to (`SketchSession::serial`).
    pub session: u32,
}

impl AppDialog for DrivenPromptDialog {
    fn title(&self) -> Cow<'static, str> {
        DRIVEN_TITLE.into()
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        let offer = with_session(cx, |s, _| {
            (s.serial() == self.session)
                .then(|| s.driven_offer.clone())
                .flatten()
        })
        .flatten();
        let Some(offer) = offer else {
            return false;
        };
        let mut answer = None;
        egui::Window::new(DRIVEN_TITLE)
            .id(egui::Id::new("amg_sketch_driven_prompt"))
            .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                let why = if offer.conflicting {
                    "contradicts the existing constraints"
                } else {
                    "is already determined by the existing constraints"
                };
                ui.label(format!(
                    "This dimension {why} and would over-constrain the sketch."
                ));
                ui.label("Create it as a driven dimension?");
                ui.horizontal(|ui| {
                    if ui.button("Yes").clicked() {
                        answer = Some(true);
                    }
                    if ui.button("No").clicked() {
                        answer = Some(false);
                    }
                });
            });
        if answer.is_none() && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            answer = Some(false);
        }
        match answer {
            Some(yes) => {
                with_session(cx, |s, app| s.accept_driven(app, yes));
                cx.request_repaint();
                false
            }
            None => true,
        }
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
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
    fn picks_choose_the_dimension_and_its_value() {
        let mut s = Sketch::new(Plane::XY);
        let l = s.add_line(pt(0.0, 0.0), pt(30.0, 40.0)).unwrap();
        let c = s.add_circle(pt(100.0, 0.0), 5.0).unwrap();
        let a = s
            .add_arc(pt(0.0, 100.0), pt(7.0, 100.0), pt(0.0, 107.0))
            .unwrap();
        let v = s.add_line(pt(0.0, 0.0), pt(0.0, 10.0)).unwrap();
        let ready = |picks: &[EntityId], at: Point2| match smart(&s, picks, at) {
            Smart::Ready(k) => oriented(&s, k).unwrap(),
            other => panic!("{other:?}"),
        };
        let (k, val) = ready(&[l], pt(-10.0, 50.0));
        assert!(matches!(k, DimensionKind::Length { .. }) && (val - 50.0).abs() < 1e-9);
        let (k, val) = ready(&[l], pt(15.0, 60.0));
        assert!(
            matches!(k, DimensionKind::HorizontalDistance { .. }),
            "{k:?}"
        );
        assert!((val - 30.0).abs() < 1e-9);
        let (k, val) = ready(&[l], pt(50.0, 20.0));
        assert!(matches!(k, DimensionKind::VerticalDistance { .. }), "{k:?}");
        assert!((val - 40.0).abs() < 1e-9);
        let (k, val) = ready(&[c], pt(0.0, 0.0));
        assert!(matches!(k, DimensionKind::Diameter { .. }) && (val - 10.0).abs() < 1e-9);
        let (k, val) = ready(&[a], pt(0.0, 0.0));
        assert!(matches!(k, DimensionKind::Radius { .. }) && (val - 7.0).abs() < 1e-9);
        // The angle between the slanted line and the vertical one, in the quadrant of the
        // label: atan(30/40).
        let (k, val) = ready(&[l, v], pt(3.0, 20.0));
        assert!(matches!(k, DimensionKind::Angle { .. }), "{k:?}");
        assert!((val - (30.0f64).atan2(40.0)).abs() < 1e-9, "{val}");
        // The label in the opposite quadrant measures the supplement.
        let (_, val) = ready(&[l, v], pt(-3.0, 20.0));
        assert!(
            (val - (std::f64::consts::PI - 30.0f64.atan2(40.0))).abs() < 1e-9,
            "{val}"
        );
        assert!(matches!(smart(&s, &[], pt(0.0, 0.0)), Smart::Need(_)));
    }

    #[test]
    fn distances_are_positive_on_either_side() {
        let mut s = Sketch::new(Plane::XY);
        let l = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
        let p = s.add_point(pt(4.0, -6.0)).unwrap();
        let Smart::Ready(k) = smart(&s, &[p, l], pt(0.0, 0.0)) else {
            panic!()
        };
        let (k, v) = oriented(&s, k).unwrap();
        assert!((v - 6.0).abs() < 1e-9);
        assert!(matches!(
            k,
            DimensionKind::Distance {
                side: Side::Negative,
                ..
            }
        ));
    }

    #[test]
    fn literals_and_typed_values() {
        assert_eq!(literal(25.0, Quantity::Length), "25 mm");
        assert_eq!(literal(12.345_678_9, Quantity::Length), "12.345679 mm");
        assert_eq!(
            literal(std::f64::consts::FRAC_PI_2, Quantity::Angle),
            "90 deg"
        );
        let params = ParamTable::default();
        let prefs = UnitPrefs::default();
        assert_eq!(
            dimension_text_expr(&params, &prefs, "12", Dim::LENGTH).unwrap(),
            "12 mm"
        );
        assert!(dimension_text_expr(&params, &prefs, "w / 2", Dim::LENGTH).is_err());
        assert!(dimension_text_expr(&params, &prefs, "", Dim::LENGTH).is_err());
    }
}
