//! The sketch-mode tools of UI-09: Create Sketch (plane pick), the session's select & drag
//! tool, and the Line tool (chained, tangent-arc drag-back, inference, heads-up input).

use std::borrow::Cow;
use std::cell::Cell;
use std::rc::Rc;
use std::sync::Arc;

use amg_core::{Dimension as Dim, Plane, Point2, Vec2};
use amg_doc::{
    Command, ConstructionGeom, EntityRef, Expr, ExprSrc, FeatureBox, ParamTable, UnitPrefs,
};
use amg_features::sketch::{PlaneInput, SketchEdit, SketchFeature};
use amg_render::{DepthMode, LabelStyle, LineStyle, OverlayList, PointStyle};
use amg_sketch::query::{self, PickFilter, SnapKind};
use amg_sketch::{
    AngleBranch, ConstraintKind, Dimension, DimensionKind, EditResult, End, EntityId, EntityKind,
    Sketch, SketchError, TangentKind,
};

use super::infer::{Anchor, InferInput, Inference, SNAP_RADIUS_PT, infer};
use super::session::{Inferred, SketchLayer, with_session};
use super::view::{self, colors, mm_per_point, ray_to_plane};
use crate::app::CommitBatch;
use crate::command::CmdId;
use crate::input::{CameraView, ViewportEvent};
use crate::pick::PickTarget;
use crate::select::EntityFilter;
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};

/// Pick radius for sketch entities, points.
pub const PICK_RADIUS_PT: f64 = 6.0;

/// What the sketch tools cache from the session each frame (for `overlay` and hints).
#[derive(Clone, Debug, Default)]
pub(super) struct Cached {
    pub(super) plane: Option<Plane>,
    pub(super) sketch: Option<Arc<Sketch>>,
    dof: Option<u32>,
    pub(super) grid: bool,
    pub(super) busy: bool,
    /// New geometry is construction geometry (`X`).
    pub(super) construction: bool,
}

impl Cached {
    pub(super) fn refresh(&mut self, cx: &mut ToolCx<'_>) -> bool {
        let got = with_session(cx.app(), |s, _| {
            (
                *s.plane(),
                Arc::clone(s.display()),
                s.dof().map(|d| d.dof),
                s.palette.snap,
                s.is_busy(),
                s.construction,
            )
        });
        match got {
            Some((plane, sketch, dof, grid, busy, construction)) => {
                self.construction = construction;
                self.plane = Some(plane);
                self.sketch = Some(sketch);
                self.dof = dof;
                self.grid = grid;
                self.busy = busy;
                true
            }
            None => {
                *self = Cached::default();
                false
            }
        }
    }

    pub(super) fn dof_text(&self) -> String {
        match self.dof {
            Some(n) => format!("DOF: {n}"),
            None => "DOF: …".to_owned(),
        }
    }
}

/// A "nice" grid step (1, 2 or 5 × 10ⁿ mm) of at least `min` mm.
pub fn nice_step(min: f64) -> f64 {
    if !(min.is_finite() && min > 0.0) {
        return 1.0;
    }
    let e = 10f64.powf(min.log10().floor());
    [1.0, 2.0, 5.0, 10.0]
        .into_iter()
        .map(|m| m * e)
        .find(|s| *s >= min)
        .unwrap_or(10.0 * e)
}

/// The cursor on the sketch plane and the mm-per-point there.
pub(super) fn cursor_on(
    plane: &Plane,
    view: Option<&CameraView>,
    ray: &amg_core::Ray3,
) -> Option<(Point2, f64)> {
    let p = ray_to_plane(plane, ray)?;
    Some((p, mm_per_point(view, plane, p)))
}

pub(super) fn mods_ctrl(m: egui::Modifiers) -> bool {
    m.ctrl || m.command
}

// ===== Create Sketch =====

/// Create Sketch: pick an origin plane (drawn as quads) or a construction plane; the new
/// sketch opens in sketch mode.
#[derive(Debug)]
pub struct CreateSketchTool {
    hovered: Option<(&'static str, Plane)>,
    hovered_construction: Option<(amg_doc::ConstructionId, Plane)>,
    half: f64,
}

impl CreateSketchTool {
    /// A new tool.
    pub fn new(_init: &ToolInit<'_>) -> CreateSketchTool {
        CreateSketchTool {
            hovered: None,
            hovered_construction: None,
            half: 40.0,
        }
    }

    const ORIGIN: [(&'static str, Plane); 3] =
        [("XY", Plane::XY), ("XZ", Plane::XZ), ("YZ", Plane::YZ)];

    fn origin_hit(&self, ray: &amg_core::Ray3) -> Option<(&'static str, Plane)> {
        let mut best: Option<(f64, &'static str, Plane)> = None;
        for (name, plane) in Self::ORIGIN {
            let Some(uv) = ray_to_plane(&plane, ray) else {
                continue;
            };
            if uv.x.abs() > self.half || uv.y.abs() > self.half {
                continue;
            }
            let t = (plane.to_world(uv) - ray.origin).length();
            if best.is_none_or(|(bt, _, _)| t < bt) {
                best = Some((t, name, plane));
            }
        }
        best.map(|(_, n, p)| (n, p))
    }

    /// Creates the sketch on `input` and finishes the tool.
    fn create(&mut self, cx: &mut ToolCx<'_>, input: PlaneInput) {
        let cell: Rc<Cell<Option<amg_doc::FeatureId>>> = Rc::new(Cell::new(None));
        if let Some(l) = cx.app().doc_layer::<SketchLayer>() {
            l.created = Some(Rc::clone(&cell));
        }
        cx.app().apply_document("Create Sketch", move |doc| {
            let context = doc.active_component();
            let out = doc.apply(Command::AddFeature {
                at: None,
                feature: FeatureBox::new(SketchFeature::new(input)),
                context,
                name: None,
            })?;
            for c in out.created {
                if let amg_doc::CreatedId::Feature(id) = c {
                    cell.set(Some(id));
                    break;
                }
            }
            Ok(())
        });
        cx.request_finish(FinishReason::Cancel);
    }

    /// Creates the sketch on origin plane `name` (`"XY"`, `"XZ"`, `"YZ"`).
    pub fn create_on_origin(&mut self, cx: &mut ToolCx<'_>, name: &str) {
        let input = match name {
            "XZ" => PlaneInput::OriginXZ,
            "YZ" => PlaneInput::OriginYZ,
            _ => PlaneInput::OriginXY,
        };
        self.create(cx, input);
    }
}

impl Tool for CreateSketchTool {
    fn id(&self) -> CmdId {
        super::CREATE_SKETCH_CMD
    }

    fn label(&self) -> Cow<'static, str> {
        "Create Sketch".into()
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::CONSTRUCTION
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        Some("Create Sketch: select a plane or a construction plane".into())
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        if let Some(v) = cx.camera() {
            self.half = (v.world_per_point(amg_core::Point3::ZERO) * 90.0).max(1e-3);
        }
        self.hovered_construction = cx.hover().and_then(|h| match h.item.target {
            PickTarget::Construction(c) => cx
                .env()
                .evaluated
                .constructions()
                .find(|(id, _)| *id == c)
                .and_then(|(_, g)| match g {
                    ConstructionGeom::Plane(p) => Some((c, *p)),
                    _ => None,
                }),
            _ => None,
        });
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        match ev {
            ViewportEvent::Hover { ray, .. } => {
                self.hovered = self.origin_hit(ray);
                InputResult::Ignored
            }
            ViewportEvent::Click {
                button: egui::PointerButton::Primary,
                ray,
                ..
            } => {
                if let Some((c, _)) = self.hovered_construction {
                    self.create(cx, PlaneInput::Ref(EntityRef::Construction(c)));
                    return InputResult::Consumed;
                }
                match self.origin_hit(ray) {
                    Some((name, _)) => {
                        self.create_on_origin(cx, name);
                        InputResult::Consumed
                    }
                    None => InputResult::Ignored,
                }
            }
            _ => InputResult::Ignored,
        }
    }

    fn dialog_ui(&mut self, ui: &mut egui::Ui, cx: &mut ToolCx<'_>) -> bool {
        ui.label("Select a plane in the viewport, or:");
        ui.horizontal(|ui| {
            for (name, _) in Self::ORIGIN {
                if ui.button(name).clicked() {
                    self.create_on_origin(cx, name);
                }
            }
        });
        true
    }

    fn overlay(&mut self, out: &mut OverlayList, _cx: &ToolCx<'_>) {
        for (name, plane) in Self::ORIGIN {
            let hot = self.hovered.is_some_and(|(n, _)| n == name);
            let a = if hot { 0.40 } else { 0.15 };
            let fill = match name {
                "XY" => [0.1, 0.3, 0.9, a],
                "XZ" => [0.1, 0.7, 0.2, a],
                _ => [0.9, 0.2, 0.1, a],
            };
            out.plane_quad(
                &plane,
                [self.half, self.half],
                fill,
                LineStyle {
                    width_px: if hot { 2.5 } else { 1.0 },
                    color: [fill[0], fill[1], fill[2], 1.0],
                    dash: None,
                    depth: DepthMode::AlwaysOnTop,
                },
                None,
            );
        }
    }

    fn finish(&mut self, _reason: FinishReason, _cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        None
    }
}

// ===== Select & drag =====

/// The session's default tool: hover, click-select and drag sketch geometry (SK-05
/// `DragSession`; release = one `CommitDrag`). Esc clears the selection, then leaves the
/// sketch; Delete removes the selection.
#[derive(Debug, Default)]
pub struct SketchSelectTool {
    cache: Cached,
    dragging: bool,
}

impl SketchSelectTool {
    /// A new tool.
    pub fn new(_init: &ToolInit<'_>) -> SketchSelectTool {
        SketchSelectTool::default()
    }

    fn hit(&self, cx: &ToolCx<'_>, ray: &amg_core::Ray3) -> Option<(EntityId, Point2)> {
        let (plane, sketch) = (self.cache.plane?, self.cache.sketch.as_ref()?);
        let (p, mmpp) = cursor_on(&plane, cx.camera(), ray)?;
        query::pick(sketch, p, mmpp * PICK_RADIUS_PT, PickFilter::ALL)
            .first()
            .map(|h| (h.entity, p))
    }
}

impl Tool for SketchSelectTool {
    fn id(&self) -> CmdId {
        super::SKETCH_SELECT_CMD
    }

    fn label(&self) -> Cow<'static, str> {
        "Sketch".into()
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::NONE
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        let construction = if self.cache.construction {
            " · construction"
        } else {
            ""
        };
        Some(
            format!(
                "{} · click to select, drag to move{construction}",
                self.cache.dof_text()
            )
            .into(),
        )
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        if !self.cache.refresh(cx) {
            cx.request_finish(FinishReason::Cancel);
        }
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        match ev {
            ViewportEvent::Hover { ray, .. } => {
                let hit = self.hit(cx, ray).map(|(e, _)| e);
                with_session(cx.app(), |s, _| s.hovered = hit);
                InputResult::Consumed
            }
            ViewportEvent::Click {
                button: egui::PointerButton::Primary,
                ray,
                mods,
                ..
            } => {
                let hit = self.hit(cx, ray).map(|(e, _)| e);
                let add = mods.shift || mods_ctrl(*mods);
                with_session(cx.app(), |s, _| match hit {
                    Some(e) if add => {
                        if !s.selected.remove(&e) {
                            s.selected.insert(e);
                        }
                    }
                    Some(e) => {
                        s.selected.clear();
                        s.selected.insert(e);
                    }
                    None if !add => s.selected.clear(),
                    None => {}
                });
                InputResult::Consumed
            }
            ViewportEvent::DragStart {
                button: egui::PointerButton::Primary,
                from,
                ..
            } => {
                let Some(view) = cx.camera() else {
                    return InputResult::Ignored;
                };
                let ray = view.ray(*from);
                let Some((e, grab)) = self.hit(cx, &ray) else {
                    return InputResult::Ignored;
                };
                let ok = with_session(cx.app(), |s, _| s.begin_drag(e, grab)).unwrap_or(false);
                self.dragging = ok;
                if ok {
                    cx.set_cursor(egui::CursorIcon::Grabbing);
                    InputResult::Consumed
                } else {
                    InputResult::Ignored
                }
            }
            ViewportEvent::Drag { ray, .. } if self.dragging => {
                if let Some(plane) = self.cache.plane
                    && let Some(p) = ray_to_plane(&plane, ray)
                {
                    with_session(cx.app(), |s, _| s.drag_to(p));
                }
                InputResult::Consumed
            }
            ViewportEvent::DragEnd { ray, .. } if self.dragging => {
                self.dragging = false;
                let p = self.cache.plane.and_then(|pl| ray_to_plane(&pl, ray));
                with_session(cx.app(), |s, _| {
                    if let Some(p) = p {
                        s.drag_to(p);
                    }
                    s.end_drag();
                });
                InputResult::Consumed
            }
            ViewportEvent::Key {
                key: egui::Key::Escape,
                pressed: true,
                ..
            } => {
                let had = with_session(cx.app(), |s, _| {
                    let had = !s.selected.is_empty();
                    s.selected.clear();
                    had
                })
                .unwrap_or(false);
                if !had && let Some(l) = cx.app().doc_layer::<SketchLayer>() {
                    l.request_finish();
                }
                InputResult::Consumed
            }
            ViewportEvent::Key {
                key: egui::Key::Delete | egui::Key::Backspace,
                pressed: true,
                ..
            } => {
                with_session(cx.app(), |s, app| {
                    let ids: Vec<EntityId> = s.selected.iter().copied().collect();
                    if !ids.is_empty() {
                        let _ = s.commit(
                            app,
                            "Delete sketch geometry",
                            SketchEdit::RemoveEntities(ids),
                            Vec::new(),
                        );
                    }
                });
                InputResult::Consumed
            }
            ViewportEvent::Key {
                key: egui::Key::Enter,
                ..
            } => InputResult::Consumed,
            _ => InputResult::Ignored,
        }
    }

    fn finish(&mut self, _reason: FinishReason, _cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        None
    }
}

// ===== Line =====

/// The heads-up numeric input of the Line tool (length, angle).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct HeadsUp {
    /// Field texts: length, angle.
    pub fields: [String; 2],
    /// The field typing goes to (0 length, 1 angle; Tab switches).
    pub active: usize,
}

#[derive(Clone, Debug)]
struct ChainPoint {
    at: Point2,
    /// The point entity there (the previous segment's end, or a snapped existing point).
    entity: Option<EntityId>,
    /// The previous curve of the chain and which of its ends is here.
    prev: Option<(EntityId, End)>,
    /// The inference of the chain's first click (its constraints go on the first line's
    /// start point).
    first: Option<Inference>,
}

/// The Line tool: click-click chains of lines with inference (snap glyphs, alignment guides,
/// auto-constraints; Ctrl suppresses), heads-up length / angle input (type a number; Tab
/// switches; Enter places; the values become driving dimensions) and tangent-arc drag-back
/// (press on the chain end and drag).
#[derive(Debug, Default)]
pub struct LineTool {
    cache: Cached,
    start: Option<ChainPoint>,
    /// The chain's first point entity (clicking it closes the chain).
    first_point: Option<EntityId>,
    cursor: Option<Point2>,
    mmpp: f64,
    ctrl: bool,
    inference: Option<Inference>,
    heads_up: Option<HeadsUp>,
    /// Tangent-arc drag-back in progress: the arc end under the cursor.
    arc_to: Option<Point2>,
    segments: u32,
}

impl LineTool {
    /// A new tool.
    pub fn new(_init: &ToolInit<'_>) -> LineTool {
        LineTool {
            mmpp: 1.0,
            ..LineTool::default()
        }
    }

    /// The inference at the cursor (tests).
    pub fn inference(&self) -> Option<&Inference> {
        self.inference.as_ref()
    }

    /// The open heads-up input (tests).
    pub fn heads_up(&self) -> Option<&HeadsUp> {
        self.heads_up.as_ref()
    }

    /// A chain is being drawn.
    pub fn chaining(&self) -> bool {
        self.start.is_some()
    }

    /// Segments created so far.
    pub fn segments(&self) -> u32 {
        self.segments
    }

    fn anchor(&self) -> Option<Anchor> {
        self.start.as_ref().map(|s| Anchor {
            at: s.at,
            entity: s.entity,
        })
    }

    fn infer_at(&self, p: Point2) -> Inference {
        let Some(sketch) = &self.cache.sketch else {
            return Inference::plain(p);
        };
        let grid = self.cache.grid.then(|| nice_step(self.mmpp * 20.0));
        infer(
            sketch,
            p,
            &InferInput {
                anchor: self.anchor(),
                radius: self.mmpp * SNAP_RADIUS_PT,
                suppress: self.ctrl,
                grid,
                exclude: &[],
            },
        )
    }

    fn update_cursor(&mut self, cx: &ToolCx<'_>, ray: &amg_core::Ray3) {
        let Some(plane) = self.cache.plane else {
            return;
        };
        if let Some((p, mmpp)) = cursor_on(&plane, cx.camera(), ray) {
            self.cursor = Some(p);
            self.mmpp = mmpp;
            self.inference = Some(self.infer_at(p));
            if self.arc_to.is_some() {
                self.arc_to = Some(p);
            }
        }
    }

    fn end_chain(&mut self) {
        self.start = None;
        self.first_point = None;
        self.heads_up = None;
        self.arc_to = None;
    }

    /// Places the next point at `inf` (typed `dims`: length and angle expressions).
    fn place(&mut self, cx: &mut ToolCx<'_>, inf: Inference, dims: Option<TypedDims>) {
        let Some(start) = self.start.clone() else {
            let entity = inf
                .snap
                .as_ref()
                .filter(|s| s.kind == SnapKind::Endpoint)
                .and_then(|s| s.entities.first().copied());
            self.start = Some(ChainPoint {
                at: inf.point,
                entity,
                prev: None,
                first: Some(inf),
            });
            return;
        };
        if (inf.point - start.at).length() <= 1e-9 * self.mmpp.max(1.0) {
            return;
        }
        let (a, b) = (start.at, inf.point);
        let closes = self.first_point.is_some()
            && inf.snap.as_ref().is_some_and(|s| {
                s.kind == SnapKind::Endpoint && s.entities.first().copied() == self.first_point
            });
        let inf2 = inf.clone();
        let st = start.clone();
        let res = with_session(cx.app(), |s, app| {
            let construction = s.construction;
            s.commit_op(
                app,
                "Line",
                |f| {
                    let res = f.apply(SketchEdit::AddLine { a, b })?;
                    if construction {
                        f.edit_sketch(|sk| mark_construction(sk, &res.created))?;
                    }
                    Ok(res)
                },
                |res, sk| line_follow_ups(res, sk, &st, &inf2, dims.as_ref()),
            )
        });
        let Some(Ok(res)) = res else {
            return;
        };
        let Some(&line) = res.created.first() else {
            return;
        };
        self.segments += 1;
        let ends = with_session(cx.app(), |s, _| line_ends(&s.working().sketch, line)).flatten();
        let Some((sp, ep)) = ends else {
            self.end_chain();
            return;
        };
        if self.first_point.is_none() {
            self.first_point = Some(sp);
        }
        if closes {
            self.end_chain();
            return;
        }
        self.start = Some(ChainPoint {
            at: b,
            entity: Some(ep),
            prev: Some((line, End::End)),
            first: None,
        });
        self.heads_up = None;
    }

    /// The tangent arc from the chain end to `to` (drag-back).
    fn place_arc(&mut self, cx: &mut ToolCx<'_>, to: Point2) {
        let Some(start) = self.start.clone() else {
            return;
        };
        let Some((prev, prev_end)) = start.prev else {
            return;
        };
        let Some(sketch) = self.cache.sketch.clone() else {
            return;
        };
        let Some(arc) = tangent_arc(&sketch, prev, prev_end, start.at, to) else {
            return;
        };
        let st = start.clone();
        let res = with_session(cx.app(), |s, app| {
            let construction = s.construction;
            s.commit_op(
                app,
                "Tangent arc",
                |f| {
                    f.edit_sketch(|sk| {
                        let id = sk.add_arc(arc.center, arc.start, arc.end)?;
                        if construction {
                            mark_construction(sk, &[id])?;
                        }
                        Ok(id)
                    })
                    .map(|id| EditResult {
                        created: vec![id],
                        ..EditResult::default()
                    })
                },
                |res, sk| {
                    let mut edits = Vec::new();
                    if let (Some(&id), Some(joint)) = (res.created.first(), st.entity)
                        && let Some(EntityKind::Arc {
                            start: a0, end: a1, ..
                        }) = sk.entity(id).map(|e| &e.kind)
                    {
                        let (at_joint, b_end) = if arc.ccw {
                            (*a0, End::Start)
                        } else {
                            (*a1, End::End)
                        };
                        edits.push(add(ConstraintKind::Coincident {
                            a: joint,
                            b: at_joint,
                        }));
                        let tangent = ConstraintKind::Tangent {
                            a: prev,
                            b: id,
                            kind: TangentKind::AtEndpoint {
                                a_end: prev_end,
                                b_end,
                                reversed: !arc.ccw,
                            },
                        };
                        edits.push(add(amg_sketch::solve::orient_constraint(sk, tangent)));
                    }
                    Inferred {
                        edits,
                        checked: Vec::new(),
                    }
                },
            )
        });
        let Some(Ok(res)) = res else {
            return;
        };
        let Some(&id) = res.created.first() else {
            return;
        };
        self.segments += 1;
        let far = with_session(cx.app(), |s, _| {
            match s.working().sketch.entity(id).map(|e| &e.kind) {
                Some(EntityKind::Arc {
                    start: a0, end: a1, ..
                }) => Some(if arc.ccw {
                    (*a1, End::End)
                } else {
                    (*a0, End::Start)
                }),
                _ => None,
            }
        })
        .flatten();
        match far {
            Some((pt, end)) => {
                self.start = Some(ChainPoint {
                    at: to,
                    entity: Some(pt),
                    prev: Some((id, end)),
                    first: None,
                });
            }
            None => self.end_chain(),
        }
    }

    /// Places the point typed in the heads-up fields.
    fn place_typed(&mut self, cx: &mut ToolCx<'_>) {
        let (Some(hu), Some(start)) = (self.heads_up.clone(), self.start.clone()) else {
            return;
        };
        let params = cx.env().design.params().clone();
        let prefs = UnitPrefs::default();
        let cursor = self.cursor.unwrap_or(start.at + Vec2::X);
        let here = cursor - start.at;
        let length = eval_typed(&params, &prefs, &hu.fields[0], Dim::LENGTH);
        let angle = eval_typed(&params, &prefs, &hu.fields[1], Dim::ANGLE);
        let len = length.as_ref().map_or(here.length(), |(v, _)| *v);
        if !(len.is_finite() && len > 1e-9) {
            return;
        }
        // The angle is measured from the previous line (straight on = 0), else from the
        // sketch x axis.
        let base = start
            .prev
            .and_then(|(e, end)| {
                self.cache
                    .sketch
                    .as_ref()
                    .and_then(|s| travel_dir(s, e, end))
            })
            .unwrap_or(Vec2::X);
        let dir = match &angle {
            Some((a, _)) => Vec2::from_angle(*a).rotate(base),
            None if here.length() > 1e-12 => here.normalize(),
            None => base,
        };
        let point = start.at + dir * len;
        let dims = TypedDims {
            length: length.map(|(_, e)| e),
            angle: angle.and_then(|(v, e)| {
                let prev_line = start.prev.filter(|(e, _)| {
                    self.cache.sketch.as_ref().is_some_and(|s| {
                        matches!(s.entity(*e).map(|x| &x.kind), Some(EntityKind::Line { .. }))
                    })
                })?;
                Some((prev_line.0, v, e))
            }),
        };
        self.heads_up = None;
        self.place(cx, Inference::plain(point), Some(dims));
    }

    /// Mirrors the inference and heads-up input into the session (tests, scripts).
    fn mirror(&self, cx: &mut ToolCx<'_>) {
        let (inf, hu) = (self.inference.clone(), self.heads_up.clone());
        with_session(cx.app(), |s, _| {
            s.inference = inf;
            s.heads_up = hu;
        });
    }

    fn handle(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        // Ctrl as held now; a button or key event carries its own modifiers.
        self.ctrl = match ev {
            ViewportEvent::Press { mods, .. }
            | ViewportEvent::Release { mods, .. }
            | ViewportEvent::Click { mods, .. }
            | ViewportEvent::Drag { mods, .. }
            | ViewportEvent::DragStart { mods, .. }
            | ViewportEvent::Key { mods, .. } => mods_ctrl(*mods),
            _ => mods_ctrl(cx.modifiers()),
        };
        match ev {
            ViewportEvent::Hover { ray, .. } => {
                self.update_cursor(cx, ray);
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
                self.update_cursor(cx, ray);
                if let Some(inf) = self.inference.clone() {
                    self.place(cx, inf, None);
                }
                InputResult::Consumed
            }
            ViewportEvent::DragStart {
                button: egui::PointerButton::Primary,
                from,
                ..
            } => {
                // Tangent-arc drag-back: press on the chain end.
                let (Some(start), Some(view)) = (&self.start, cx.camera()) else {
                    return InputResult::Ignored;
                };
                if start.prev.is_none() {
                    return InputResult::Ignored;
                }
                let Some(plane) = self.cache.plane else {
                    return InputResult::Ignored;
                };
                let Some((p, mmpp)) = cursor_on(&plane, Some(view), &view.ray(*from)) else {
                    return InputResult::Ignored;
                };
                if p.distance(start.at) > mmpp * PICK_RADIUS_PT * 1.5 {
                    return InputResult::Ignored;
                }
                self.arc_to = Some(p);
                InputResult::Consumed
            }
            ViewportEvent::Drag { ray, .. } if self.arc_to.is_some() => {
                self.update_cursor(cx, ray);
                InputResult::Consumed
            }
            ViewportEvent::DragEnd { ray, .. } if self.arc_to.is_some() => {
                self.update_cursor(cx, ray);
                let to = self.arc_to.take();
                if let Some(to) = to
                    && !self.cache.busy
                {
                    self.place_arc(cx, to);
                }
                InputResult::Consumed
            }
            ViewportEvent::Text(t) if self.start.is_some() => {
                let starts_number = t
                    .chars()
                    .all(|c| c.is_ascii_digit() || ".-+*/() ".contains(c))
                    || self.heads_up.is_some();
                if !starts_number {
                    return InputResult::Ignored;
                }
                let hu = self.heads_up.get_or_insert_with(HeadsUp::default);
                hu.fields[hu.active].push_str(t);
                InputResult::Consumed
            }
            ViewportEvent::Key {
                key, pressed: true, ..
            } => match key {
                egui::Key::Tab if self.heads_up.is_some() => {
                    if let Some(hu) = &mut self.heads_up {
                        hu.active = 1 - hu.active;
                    }
                    InputResult::Consumed
                }
                egui::Key::Backspace if self.heads_up.is_some() => {
                    if let Some(hu) = &mut self.heads_up {
                        hu.fields[hu.active].pop();
                    }
                    InputResult::Consumed
                }
                egui::Key::Enter if self.heads_up.is_some() => {
                    if !self.cache.busy {
                        self.place_typed(cx);
                    }
                    InputResult::Consumed
                }
                egui::Key::Escape if self.heads_up.is_some() => {
                    self.heads_up = None;
                    InputResult::Consumed
                }
                egui::Key::Escape if self.start.is_some() => {
                    self.end_chain();
                    InputResult::Consumed
                }
                _ => InputResult::Ignored,
            },
            ViewportEvent::Key { .. } if self.heads_up.is_some() => InputResult::Consumed,
            _ => InputResult::Ignored,
        }
    }
}

/// Evaluates a heads-up field: a bare number takes the default unit (mm, deg), anything else
/// is an S01 expression of dimension `dim`. Returns the value (mm, rad) and the expression.
pub(super) fn eval_typed(
    params: &ParamTable,
    prefs: &UnitPrefs,
    src: &str,
    dim: Dim,
) -> Option<(f64, Expr)> {
    let src = src.trim();
    if src.is_empty() {
        return None;
    }
    let unit = if dim == Dim::ANGLE { "deg" } else { "mm" };
    let text = if src.parse::<f64>().is_ok() {
        format!("{src} {unit}")
    } else {
        src.to_owned()
    };
    let q = params.try_eval(&text, Some(dim), prefs).ok()?;
    Some((q.value, Expr::Inline(ExprSrc(text))))
}

/// Marks the curves among `ids` as construction geometry.
pub(super) fn mark_construction(s: &mut Sketch, ids: &[EntityId]) -> Result<(), SketchError> {
    for &id in ids {
        if s.entity(id)
            .is_some_and(|e| !matches!(e.kind, EntityKind::Point { .. }))
        {
            s.set_construction(id, true)?;
        }
    }
    Ok(())
}

/// Heads-up values of one segment.
#[derive(Clone, Debug)]
struct TypedDims {
    length: Option<Expr>,
    /// Previous line, angle (rad) and its expression.
    angle: Option<(EntityId, f64, Expr)>,
}

fn add(kind: ConstraintKind) -> SketchEdit {
    SketchEdit::AddConstraint { kind, expr: None }
}

fn line_ends(s: &Sketch, line: EntityId) -> Option<(EntityId, EntityId)> {
    match s.entity(line)?.kind {
        EntityKind::Line { start, end } => Some((start, end)),
        _ => None,
    }
}

/// Unit direction of travel of curve `e` at its end `end` (out of the chain point).
pub(super) fn travel_dir(s: &Sketch, e: EntityId, end: End) -> Option<Vec2> {
    let c = s.curve(e)?;
    let (t0, t1) = c.param_range();
    let d = match end {
        End::End => c.derivative(t1),
        End::Start => -c.derivative(t0),
    };
    (d.length() > 1e-12).then(|| d.normalize())
}

/// The constraints and dimensions that go with a new line (see [`LineTool`]).
fn line_follow_ups(
    res: &EditResult,
    sk: &Sketch,
    start: &ChainPoint,
    inf: &Inference,
    dims: Option<&TypedDims>,
) -> Inferred {
    let mut out = Inferred::default();
    let Some(&line) = res.created.first() else {
        return out;
    };
    let Some((sp, ep)) = line_ends(sk, line) else {
        return out;
    };
    // Start: joined to the chain, or the first click's inference.
    match (start.prev, start.entity, &start.first) {
        (Some(_), Some(joint), _) => out
            .edits
            .push(add(ConstraintKind::Coincident { a: joint, b: sp })),
        (_, _, Some(first)) => {
            for k in first.constraints(sp, None) {
                out.edits.push(add(k));
            }
        }
        _ => {}
    }
    // End: the snap, the line direction and alignments. Direction constraints of a line whose
    // end also snapped to existing geometry may be implied already: check them first.
    let typed = dims.is_some_and(|d| d.length.is_some() || d.angle.is_some());
    if !typed {
        for k in inf.constraints(ep, Some(line)) {
            let direction = matches!(
                k,
                ConstraintKind::Horizontal(amg_sketch::HvTarget::Line(_))
                    | ConstraintKind::Vertical(amg_sketch::HvTarget::Line(_))
                    | ConstraintKind::Tangent { .. }
                    | ConstraintKind::Perpendicular { .. }
                    | ConstraintKind::Parallel { .. }
            );
            if direction && inf.snap.is_some() {
                out.checked.push(k);
            } else {
                out.edits.push(add(k));
            }
        }
    }
    if let Some(d) = dims {
        let (a, b) = (sk.point(sp), sk.point(ep));
        let mid = match (a, b) {
            (Some(a), Some(b)) => (a + b) * 0.5,
            _ => Point2::ZERO,
        };
        if let Some(expr) = &d.length {
            let value = match (a, b) {
                (Some(a), Some(b)) => a.distance(b),
                _ => 0.0,
            };
            out.edits.push(SketchEdit::AddConstraint {
                kind: ConstraintKind::Dimension(Dimension {
                    kind: DimensionKind::Length { line },
                    value,
                    driving: true,
                    label_pos: mid,
                }),
                expr: Some(expr.clone()),
            });
        }
        if let Some((prev, value, expr)) = &d.angle {
            let kind = ConstraintKind::Dimension(Dimension {
                kind: DimensionKind::Angle {
                    a: *prev,
                    b: line,
                    branch: AngleBranch {
                        reverse_a: false,
                        reverse_b: false,
                    },
                },
                value: *value,
                driving: true,
                label_pos: a.unwrap_or(mid),
            });
            out.edits.push(SketchEdit::AddConstraint {
                kind: amg_sketch::solve::orient_constraint(sk, kind),
                expr: Some(expr.clone()),
            });
        }
    }
    out
}

/// A tangent arc from the chain end.
#[derive(Copy, Clone, Debug, PartialEq)]
struct TangentArc {
    center: Point2,
    /// `add_arc` start / end (CCW).
    start: Point2,
    end: Point2,
    /// The arc runs CCW from the chain end (its start is the joint).
    ccw: bool,
}

/// The arc tangent to `prev` (at its end `prev_end`, the point `from`) through `to`.
fn tangent_arc(
    s: &Sketch,
    prev: EntityId,
    prev_end: End,
    from: Point2,
    to: Point2,
) -> Option<TangentArc> {
    let t = travel_dir(s, prev, prev_end)?;
    let n = t.perp();
    let d = to - from;
    let k = d.dot(n);
    if k.abs() <= 1e-6 * d.length().max(1e-9) {
        return None;
    }
    let r = d.length_squared() / (2.0 * k);
    let center = from + n * r;
    Some(if r > 0.0 {
        TangentArc {
            center,
            start: from,
            end: to,
            ccw: true,
        }
    } else {
        TangentArc {
            center,
            start: to,
            end: from,
            ccw: false,
        }
    })
}

impl Tool for LineTool {
    fn id(&self) -> CmdId {
        super::LINE_CMD
    }

    fn label(&self) -> Cow<'static, str> {
        "Line".into()
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::NONE
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        let what = if self.start.is_some() {
            "click the next point, type a length, drag back from the end for an arc"
        } else {
            "click the start point"
        };
        Some(
            format!(
                "{} · Line: {what} (Ctrl: no inference)",
                self.cache.dof_text()
            )
            .into(),
        )
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        if !self.cache.refresh(cx) {
            cx.request_finish(FinishReason::Cancel);
            return;
        }
        self.mirror(cx);
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        let r = self.handle(ev, cx);
        self.mirror(cx);
        r
    }

    fn overlay(&mut self, out: &mut OverlayList, _cx: &ToolCx<'_>) {
        let (Some(plane), Some(sketch)) = (self.cache.plane, self.cache.sketch.clone()) else {
            return;
        };
        let preview = LineStyle {
            width_px: view::CURVE_PX,
            color: colors::PREVIEW,
            dash: None,
            depth: DepthMode::AlwaysOnTop,
        };
        let guide = LineStyle {
            width_px: 1.0,
            color: colors::INFER,
            dash: Some((4.0, 4.0)),
            depth: DepthMode::AlwaysOnTop,
        };
        if let Some(to) = self.arc_to
            && let Some(st) = &self.start
            && let Some((prev, end)) = st.prev
        {
            if let Some(a) = tangent_arc(&sketch, prev, end, st.at, to) {
                let arc = amg_sketch::Arc2::from_center_start_end(a.center, a.start, a.end);
                view::polyline(
                    out,
                    &plane,
                    &view::sample(&amg_sketch::Curve2::Arc(arc)),
                    preview,
                );
            }
            return;
        }
        let Some(inf) = &self.inference else {
            return;
        };
        if let Some(st) = &self.start {
            view::polyline(out, &plane, &[st.at, inf.point], preview);
        }
        for (a, b) in inf.guides(&sketch, self.anchor()) {
            view::polyline(out, &plane, &[a, b], guide);
        }
        out.points(
            &[plane.to_world(inf.point)],
            PointStyle {
                size_px: view::POINT_PX,
                color: colors::PREVIEW,
                depth: DepthMode::AlwaysOnTop,
            },
            None,
        );
        if let Some(g) = inf.glyph() {
            out.label(
                plane.to_world(inf.point),
                g,
                LabelStyle {
                    size_pt: 11.0,
                    color: colors::INFER,
                    offset_px: [18.0, -16.0],
                    background: Some(colors::LABEL_BG),
                },
            );
        }
        if let Some(hu) = &self.heads_up {
            for (i, name) in ["L", "A"].into_iter().enumerate() {
                let caret = if hu.active == i { "▏" } else { "" };
                out.label(
                    plane.to_world(inf.point),
                    format!("{name} {}{caret}", hu.fields[i]),
                    LabelStyle {
                        size_pt: 12.0,
                        color: if hu.active == i {
                            colors::FULL
                        } else {
                            colors::UNDER
                        },
                        offset_px: [70.0, 18.0 + 20.0 * i as f32],
                        background: Some(colors::LABEL_BG),
                    },
                );
            }
        }
    }

    fn finish(&mut self, _reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        with_session(cx.app(), |s, _| {
            s.inference = None;
            s.heads_up = None;
        });
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nice_steps() {
        for (min, step) in [(0.7, 1.0), (1.2, 2.0), (3.0, 5.0), (6.0, 10.0), (-1.0, 1.0)] {
            assert!((nice_step(min) - step).abs() < 1e-12, "{min} -> {step}");
        }
    }

    #[test]
    fn tangent_arc_leaves_the_line_along_its_direction() {
        let mut s = Sketch::new(Plane::XY);
        let l = s
            .add_line(Point2::new(0.0, 0.0), Point2::new(10.0, 0.0))
            .unwrap();
        // To the upper right: a CCW arc centred above the joint.
        let a = tangent_arc(
            &s,
            l,
            End::End,
            Point2::new(10.0, 0.0),
            Point2::new(20.0, 10.0),
        )
        .unwrap();
        assert!(a.ccw);
        assert!((a.center - Point2::new(10.0, 10.0)).length() < 1e-12);
        // To the lower right: CW, so the CCW arc starts at the far end.
        let b = tangent_arc(
            &s,
            l,
            End::End,
            Point2::new(10.0, 0.0),
            Point2::new(20.0, -10.0),
        )
        .unwrap();
        assert!(!b.ccw && b.end == Point2::new(10.0, 0.0));
        // Straight on: no arc.
        assert!(
            tangent_arc(
                &s,
                l,
                End::End,
                Point2::new(10.0, 0.0),
                Point2::new(20.0, 0.0)
            )
            .is_none()
        );
    }
}
