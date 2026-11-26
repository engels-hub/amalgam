//! [`SectionTool`]: the hand-built Phase 1 Section Analysis dialog (S05 §3, INS-09) on the
//! lane-D framework (UI-03 tool, UI-04 picking, UI-15 widgets).
//!
//! - **Base:** an origin plane (XY / YZ / XZ), a picked planar region or a construction plane:
//!   *Pick face* and a click on a mesh face grows its coplanar region (INS-01
//!   `grow_planar_region`, as a job) and fits a plane, stored by value (`SectionBase::Free`);
//!   a click on a construction plane stores its reference (`SectionBase::Ref`, CON-04), so the
//!   section follows the plane. A face or construction plane preselected when the tool starts
//!   is used the same way.
//! - **Offset:** an expression field (any length expression, `10 mm`, `0.5 in`, parameters),
//!   a slider over the scene extent along the base normal, and − / + step buttons; tilt about the
//!   base X / Y axes (degrees), flip, hatch pattern, cap colour (body colour or a uniform
//!   colour) and scope (all visible bodies or the listed ones: clicks on bodies toggle them).
//! - The draft is shown live: [`SectionLayer::set_preview`] puts it in the view's planes every
//!   frame, so the GPU clip follows the slider while the caps job catches up.
//! - **OK** adds the record (`Command::AddExt`) or replaces the edited one (`Command::SetExt`):
//!   one undo step. Cancel / Esc changes nothing.
//!
//! Tests read the dialog's state through [`section_tool_snapshot`].

use std::borrow::Cow;
use std::sync::Arc;

use amg_core::{Aabb3, Dimension, Plane, Transform};
use amg_doc::{BodyGeom, BodyId, EntityRef, ExtKey, TopoKind};
use amg_features::inspect::InspectError;
use amg_features::inspect::doc::{
    CapColor, HatchPattern, OriginPlane, SectionAnalysis, SectionBase, SectionScope, next_name,
};
use amg_features::inspect::measure::{RegionOpts, fit_plane_faces, grow_planar_region};
use amg_jobs::{JobCtx, JobError, JobHandle, SpawnOptions};
use amg_mesh::{FaceId, IndexedMesh};

use super::section::{AnalysisStats, Preview, SECTION_CMD, SectionLayer, stats_ui};
use crate::app::CommitBatch;
use crate::command::CmdId;
use crate::input::ViewportEvent;
use crate::pick::{PickHitUi, PickMode, PickTarget, PickTicket};
use crate::select::{EntityFilter, SelItem};
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};
use crate::widgets::{ChoiceField, DialogAction, DialogFrame, ExprInput, flip, toggle};

/// Step of the − / + offset buttons, mm.
pub const OFFSET_STEP: f64 = 1.0;
/// Label of the face-region job.
pub const BASE_JOB: &str = "Section base";

/// Which base the dialog uses.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BaseChoice {
    /// An origin plane.
    Origin(OriginPlane),
    /// The picked planar region (`SectionBase::Free`) or construction plane
    /// (`SectionBase::Ref`).
    Picked,
}

const BASES: [(BaseChoice, &str); 4] = [
    (BaseChoice::Origin(OriginPlane::XY), "XY plane"),
    (BaseChoice::Origin(OriginPlane::YZ), "YZ plane"),
    (BaseChoice::Origin(OriginPlane::XZ), "XZ plane"),
    (BaseChoice::Picked, "Picked face"),
];

const HATCHES: [(HatchPattern, &str); 3] = [
    (HatchPattern::None, "None"),
    (HatchPattern::Lines45, "Lines"),
    (HatchPattern::Cross, "Cross"),
];

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ColorChoice {
    Body,
    Uniform,
}

const COLORS: [(ColorChoice, &str); 2] = [
    (ColorChoice::Body, "Body colour"),
    (ColorChoice::Uniform, "Uniform"),
];

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ScopeChoice {
    All,
    Listed,
}

const SCOPES: [(ScopeChoice, &str); 2] = [
    (ScopeChoice::All, "All visible bodies"),
    (ScopeChoice::Listed, "Selected bodies"),
];

/// Default uniform cap colour (sRGB).
const DEFAULT_CAP: amg_mesh::Rgba8 = amg_mesh::Rgba8([214, 72, 48, 255]);

/// The dialog's state as tests and scripts see it.
#[derive(Clone, Debug)]
pub struct SectionToolSnapshot {
    /// The record being edited (`None`: a new analysis).
    pub editing: Option<ExtKey>,
    /// The draft.
    pub draft: SectionAnalysis,
    /// The offset expression text.
    pub offset_text: String,
    /// The offset slider range, mm.
    pub slider: (f64, f64),
    /// Waiting for a face click.
    pub picking: bool,
    /// A face-region job is running.
    pub resolving: bool,
    /// The statistics of the draft's plane, when the caps are current.
    pub stats: Option<AnalysisStats>,
    /// The last error.
    pub error: Option<String>,
}

fn snapshot_id() -> egui::Id {
    egui::Id::new("amg_ui.inspect.section_tool_snapshot")
}

/// The Section Analysis dialog's state as of its last draw (`None` before the first one).
pub fn section_tool_snapshot(ctx: &egui::Context) -> Option<SectionToolSnapshot> {
    ctx.data(|d| d.get_temp(snapshot_id()))
}

/// A face pick to turn into a base plane.
#[derive(Clone, Debug)]
struct FacePick {
    mesh: Arc<IndexedMesh>,
    to_world: Transform,
    face: FaceId,
}

/// The world plane of the coplanar region around `p.face` (normal along the face normals).
fn region_plane(p: &FacePick, ctx: &JobCtx) -> Result<Plane, InspectError> {
    let faces = grow_planar_region(&p.mesh, p.face, &RegionOpts::default(), ctx)?;
    let fit = fit_plane_faces(&p.mesh, &faces).ok_or(InspectError::Degenerate(
        "the picked region has no plane (normals cancel)",
    ))?;
    let local = fit.plane;
    let origin = p.to_world.transform_point3(local.origin);
    let n = p.to_world.matrix3 * local.normal.get();
    let x = p.to_world.matrix3 * local.x.get();
    let n = amg_core::Dir3::new(n, &amg_core::Tolerance::default())
        .ok_or(InspectError::Degenerate("picked plane normal"))?;
    Plane::from_normal_x(origin, n, x, &amg_core::Tolerance::default())
        .or_else(|_| Ok(Plane::from_normal(origin, n)))
}

/// The reference and world plane of construction geometry `c`, if it is a plane.
fn construction_base(
    env: &crate::app::CmdEnv<'_>,
    c: amg_doc::ConstructionId,
) -> Option<(EntityRef, Plane)> {
    let entity = crate::construct::world_ref(env.design, c);
    let plane = crate::construct::base_plane(env.design, env.evaluated, &entity)?;
    Some((entity, plane))
}

/// The Section Analysis tool (see the module docs).
pub struct SectionTool {
    editing: Option<ExtKey>,
    draft: SectionAnalysis,
    base: BaseChoice,
    picked: Option<Plane>,
    /// The construction plane `picked` stands for (CON-04); `None` for a mesh region.
    picked_ref: Option<EntityRef>,
    offset_text: String,
    color: ColorChoice,
    uniform: amg_mesh::Rgba8,
    scope: ScopeChoice,
    listed: Vec<SelItem>,
    listed_active: bool,
    picking: bool,
    click: Option<PickTicket>,
    job: Option<JobHandle<Plane, InspectError>>,
    preselected_face: Option<PickTarget>,
    bounds: Aabb3,
    error: Option<String>,
    started: bool,
}

impl std::fmt::Debug for SectionTool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SectionTool")
            .field("editing", &self.editing)
            .field("draft", &self.draft)
            .finish_non_exhaustive()
    }
}

fn mesh_of(env: &crate::app::CmdEnv<'_>, body: BodyId) -> Option<Arc<IndexedMesh>> {
    match &env.evaluated.body(body)?.geom {
        BodyGeom::Mesh(m) => Some(Arc::clone(m)),
        _ => None,
    }
}

fn mm_text(v: f64) -> String {
    let s = format!("{v:.4}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    format!("{} mm", if s == "-0" { "0" } else { s })
}

impl SectionTool {
    /// A new analysis at XY + 0 of every visible body, named "Section N". Preselected mesh
    /// bodies become the scope; a preselected mesh face becomes the base.
    pub fn new(init: &ToolInit<'_>) -> SectionTool {
        let env = &init.env;
        let draft =
            SectionAnalysis::new(next_name(env.design), SectionBase::Origin(OriginPlane::XY));
        let mut listed = Vec::new();
        let mut preselected_face = None;
        let mut construction = None;
        for i in init.preselection {
            if construction.is_none()
                && let PickTarget::Construction(c) = i.target
            {
                construction = construction_base(env, c);
            }
            match i.target {
                PickTarget::Body {
                    body, sub: None, ..
                } if mesh_of(env, body).is_some() => {
                    listed.push(i.clone());
                }
                PickTarget::Body {
                    body,
                    sub: Some((TopoKind::Face, _)),
                    ..
                } if mesh_of(env, body).is_some() && preselected_face.is_none() => {
                    preselected_face = Some(i.target);
                }
                _ => {}
            }
        }
        let scope = if listed.is_empty() {
            ScopeChoice::All
        } else {
            ScopeChoice::Listed
        };
        let base = if construction.is_some() {
            BaseChoice::Picked
        } else {
            BaseChoice::Origin(OriginPlane::XY)
        };
        let (picked_ref, picked) = construction.unzip();
        SectionTool {
            editing: None,
            offset_text: mm_text(draft.offset),
            draft,
            base,
            picked,
            picked_ref,
            color: ColorChoice::Body,
            uniform: DEFAULT_CAP,
            scope,
            listed,
            listed_active: false,
            picking: false,
            click: None,
            job: None,
            preselected_face,
            bounds: Aabb3::EMPTY,
            error: None,
            started: false,
        }
    }

    /// Loads the record under `key` for editing.
    fn load(&mut self, key: ExtKey, a: SectionAnalysis) {
        self.picked_ref = match &a.base {
            SectionBase::Ref { entity, .. } => Some(entity.clone()),
            _ => None,
        };
        self.base = match &a.base {
            SectionBase::Origin(o) => BaseChoice::Origin(*o),
            SectionBase::Free(p) | SectionBase::Ref { last: p, .. } => {
                self.picked = Some(*p);
                BaseChoice::Picked
            }
        };
        self.offset_text = mm_text(a.offset);
        (self.color, self.uniform) = match a.cap_color {
            CapColor::BodyColor => (ColorChoice::Body, DEFAULT_CAP),
            CapColor::Uniform(c) => (ColorChoice::Uniform, c),
        };
        (self.scope, self.listed) = match &a.scope {
            SectionScope::AllVisible => (ScopeChoice::All, Vec::new()),
            SectionScope::Bodies(list) => (
                ScopeChoice::Listed,
                list.iter()
                    .filter_map(|b| {
                        SelItem::from_target(PickTarget::Body {
                            body: *b,
                            instance: 0,
                            sub: None,
                        })
                    })
                    .collect(),
            ),
        };
        self.draft = a;
        self.editing = Some(key);
    }

    /// The base plane of the current choice.
    fn base_plane(&self) -> Option<Plane> {
        match self.base {
            BaseChoice::Origin(o) => Some(o.plane()),
            BaseChoice::Picked => self.picked,
        }
    }

    /// The slider range: the scene bounds along the base normal, padded by 5 %.
    fn slider_range(&self) -> (f64, f64) {
        let (Some(base), false) = (self.base_plane(), self.bounds.is_empty()) else {
            return (-100.0, 100.0);
        };
        let n = base.normal.get();
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for i in 0..8 {
            let p = amg_core::Point3::new(
                if i & 1 == 0 {
                    self.bounds.min.x
                } else {
                    self.bounds.max.x
                },
                if i & 2 == 0 {
                    self.bounds.min.y
                } else {
                    self.bounds.max.y
                },
                if i & 4 == 0 {
                    self.bounds.min.z
                } else {
                    self.bounds.max.z
                },
            );
            let d = n.dot(p - base.origin);
            lo = lo.min(d);
            hi = hi.max(d);
        }
        let pad = ((hi - lo) * 0.05).max(1.0);
        (lo - pad, hi + pad)
    }

    /// Writes the dialog choices into the draft.
    fn sync_draft(&mut self) {
        self.draft.base = match (self.base, self.picked) {
            (BaseChoice::Origin(o), _) => SectionBase::Origin(o),
            (BaseChoice::Picked, Some(p)) => match &self.picked_ref {
                Some(entity) => SectionBase::Ref {
                    entity: entity.clone(),
                    last: p,
                },
                None => SectionBase::Free(p),
            },
            (BaseChoice::Picked, None) => self.draft.base.clone(),
        };
        self.draft.cap_color = match self.color {
            ColorChoice::Body => CapColor::BodyColor,
            ColorChoice::Uniform => CapColor::Uniform(self.uniform),
        };
        self.draft.scope = match self.scope {
            ScopeChoice::All => SectionScope::AllVisible,
            ScopeChoice::Listed => {
                let mut b: Vec<BodyId> =
                    self.listed.iter().filter_map(|i| i.target.body()).collect();
                b.sort();
                b.dedup();
                SectionScope::Bodies(b)
            }
        };
    }

    fn push_preview(&self, cx: &mut ToolCx<'_>) {
        let p = Preview {
            key: self.editing.clone(),
            analysis: self.draft.clone(),
        };
        if let Some(l) = cx.app().doc_layer::<SectionLayer>() {
            l.set_preview(Some(p));
        }
    }

    /// Starts the region job for a face pick.
    fn resolve_face(&mut self, target: PickTarget, cx: &mut ToolCx<'_>) {
        let PickTarget::Body {
            body,
            instance,
            sub: Some((TopoKind::Face, f)),
        } = target
        else {
            return;
        };
        let Some(mesh) = mesh_of(&cx.env(), body) else {
            return;
        };
        if f as usize >= mesh.face_count() {
            return;
        }
        let Some(to_world) = cx.scene().instance_world(body, instance) else {
            return;
        };
        let pick = FacePick {
            mesh,
            to_world,
            face: FaceId(f),
        };
        if let Some(j) = self.job.take() {
            j.cancel();
        }
        self.job = Some(cx.app().jobs().spawn_fn(
            BASE_JOB,
            SpawnOptions::default(),
            move |ctx: &JobCtx| region_plane(&pick, ctx),
        ));
        self.picking = false;
    }

    fn on_hits(&mut self, hits: Vec<PickHitUi>, cx: &mut ToolCx<'_>) {
        if self.picking {
            // The front-most face or construction plane.
            for t in hits.iter().map(|h| h.target) {
                match t {
                    PickTarget::Body {
                        sub: Some((TopoKind::Face, _)),
                        ..
                    } => {
                        self.resolve_face(t, cx);
                        break;
                    }
                    PickTarget::Construction(c) => {
                        if let Some((entity, plane)) = construction_base(&cx.env(), c) {
                            if let Some(j) = self.job.take() {
                                j.cancel();
                            }
                            self.picked_ref = Some(entity);
                            self.picked = Some(plane);
                            self.base = BaseChoice::Picked;
                            self.picking = false;
                            self.error = None;
                            break;
                        }
                    }
                    _ => {}
                }
            }
            return;
        }
        if self.scope == ScopeChoice::Listed
            && let Some(body) = hits.iter().find_map(|h| h.target.body())
        {
            if let Some(i) = self
                .listed
                .iter()
                .position(|s| s.target.body() == Some(body))
            {
                self.listed.remove(i);
            } else if let Some(item) = SelItem::from_target(PickTarget::Body {
                body,
                instance: 0,
                sub: None,
            }) {
                self.listed.push(item);
            }
        }
    }

    fn poll_job(&mut self) {
        let Some(j) = &mut self.job else { return };
        let Some(r) = j.try_take() else { return };
        self.job = None;
        match r {
            Ok(p) => {
                self.picked_ref = None;
                self.picked = Some(p);
                self.base = BaseChoice::Picked;
                self.error = None;
            }
            Err(JobError::Cancelled) => {}
            Err(e) => self.error = Some(e.to_string()),
        }
    }

    fn current_stats(&self, cx: &mut ToolCx<'_>) -> Option<AnalysisStats> {
        let s = cx.app().doc_layer::<SectionLayer>()?.snapshot();
        if !s.current {
            return None;
        }
        s.stats.into_iter().find(|st| st.key == self.editing)
    }

    fn snapshot(&self, stats: Option<AnalysisStats>) -> SectionToolSnapshot {
        SectionToolSnapshot {
            editing: self.editing.clone(),
            draft: self.draft.clone(),
            offset_text: self.offset_text.clone(),
            slider: self.slider_range(),
            picking: self.picking,
            resolving: self.job.is_some(),
            stats,
            error: self.error.clone(),
        }
    }

    /// The dialog body.
    fn body_ui(&mut self, ui: &mut egui::Ui, cx: &mut ToolCx<'_>) {
        let (params, prefs) = {
            let env = cx.env();
            (
                env.design.params().clone(),
                env.design.meta().default_units.clone(),
            )
        };
        ui.horizontal(|ui| {
            let l = ui.label("Name");
            ui.text_edit_singleline(&mut self.draft.name)
                .labelled_by(l.id);
        });
        ui.horizontal(|ui| {
            let can_pick = self.picked.is_some();
            let before = self.base;
            ChoiceField::new("Base", &mut self.base, &BASES)
                .buttons()
                .show(ui);
            if self.base == BaseChoice::Picked && !can_pick {
                // Nothing picked yet: keep the old base until a face is picked.
                self.base = before;
                self.picking = true;
            }
        });
        ui.horizontal(|ui| {
            if ui
                .add(egui::Button::new("Pick face").selected(self.picking))
                .on_hover_text(
                    "Click a construction plane, or a planar face of a mesh body (its coplanar \
                     region is the base)",
                )
                .clicked()
            {
                self.picking = !self.picking;
            }
            if self.picking {
                ui.weak("Click a face or construction plane in the viewport…");
            } else if self.job.is_some() {
                ui.spinner();
                ui.weak("Fitting the plane…");
            }
        });
        // Offset: expression, slider, − / +.
        let (lo, hi) = self.slider_range();
        let r = ExprInput::new(&mut self.offset_text, Dimension::LENGTH, &params, &prefs)
            .label("Offset")
            .id_salt("section_offset")
            .nudge_step(OFFSET_STEP)
            .show(ui);
        if (r.changed || r.committed)
            && let Ok(q) = r.value
        {
            self.draft.offset = q.value;
        }
        let mut text_from_value = false;
        ui.horizontal(|ui| {
            let mut v = self.draft.offset;
            let range = lo.min(v)..=hi.max(v);
            let s = ui.add(
                egui::Slider::new(&mut v, range)
                    .show_value(false)
                    .text("Offset slider"),
            );
            if s.changed() {
                self.draft.offset = v;
                text_from_value = true;
            }
            let minus = ui.small_button("−").on_hover_text("Offset − 1 mm");
            minus.widget_info(|| {
                egui::WidgetInfo::labeled(egui::WidgetType::Button, true, "Offset down")
            });
            if minus.clicked() {
                self.draft.offset -= OFFSET_STEP;
                text_from_value = true;
            }
            let plus = ui.small_button("+").on_hover_text("Offset + 1 mm");
            plus.widget_info(|| {
                egui::WidgetInfo::labeled(egui::WidgetType::Button, true, "Offset up")
            });
            if plus.clicked() {
                self.draft.offset += OFFSET_STEP;
                text_from_value = true;
            }
        });
        if text_from_value {
            self.offset_text = mm_text(self.draft.offset);
        }
        // Tilt (degrees in the dialog, radians in the record).
        for (i, label) in ["Tilt X", "Tilt Y"].into_iter().enumerate() {
            ui.horizontal(|ui| {
                let l = ui.label(label);
                let mut deg = self.draft.tilt[i].to_degrees();
                if ui
                    .add(
                        egui::DragValue::new(&mut deg)
                            .range(-90.0..=90.0)
                            .speed(0.5)
                            .suffix("°"),
                    )
                    .labelled_by(l.id)
                    .changed()
                {
                    self.draft.tilt[i] = deg.to_radians();
                }
            });
        }
        flip(ui, "Flip", &mut self.draft.flip);
        ChoiceField::new("Hatch", &mut self.draft.hatch.pattern, &HATCHES)
            .buttons()
            .show(ui);
        ui.horizontal(|ui| {
            ChoiceField::new("Cap colour", &mut self.color, &COLORS)
                .buttons()
                .show(ui);
            if self.color == ColorChoice::Uniform {
                let [r, g, b, a] = self.uniform.0;
                let mut c = egui::Color32::from_rgba_unmultiplied(r, g, b, a);
                if egui::color_picker::color_edit_button_srgba(
                    ui,
                    &mut c,
                    egui::color_picker::Alpha::Opaque,
                )
                .changed()
                {
                    self.uniform = amg_mesh::Rgba8(c.to_srgba_unmultiplied());
                }
            }
        });
        ChoiceField::new("Bodies", &mut self.scope, &SCOPES)
            .buttons()
            .show(ui);
        if self.scope == ScopeChoice::Listed {
            crate::widgets::SelectionField::new(
                "Section bodies",
                &mut self.listed,
                EntityFilter::BODY,
                &mut self.listed_active,
            )
            .limits(1, None)
            .show(ui);
            ui.weak("Click bodies in the viewport to add or remove them.");
        }
        let mut visible = self.draft.visible;
        if toggle(ui, "Visible", &mut visible).changed() {
            self.draft.visible = visible;
        }
        if let Some(e) = &self.error {
            ui.colored_label(ui.visuals().error_fg_color, e);
        }
    }
}

impl Tool for SectionTool {
    fn id(&self) -> CmdId {
        SECTION_CMD
    }

    fn label(&self) -> Cow<'static, str> {
        if self.editing.is_some() {
            "Edit Section Analysis".into()
        } else {
            "Section Analysis".into()
        }
    }

    fn start(&mut self, cx: &mut ToolCx<'_>) {
        self.started = true;
        self.bounds = cx.scene().bounds();
        let edit = cx
            .app()
            .doc_layer::<SectionLayer>()
            .and_then(|l| l.take_edit());
        if let Some(key) = edit {
            let rec = cx
                .env()
                .design
                .ext(&key)
                .and_then(|r| SectionAnalysis::from_record(r).ok());
            if let Some(a) = rec {
                self.load(key, a);
            }
        } else if let Some(t) = self.preselected_face.take() {
            self.resolve_face(t, cx);
        }
        self.sync_draft();
        self.push_preview(cx);
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        match ev {
            ViewportEvent::Click {
                button: egui::PointerButton::Primary,
                pos,
                ..
            } if self.picking || self.scope == ScopeChoice::Listed => {
                self.click = Some(cx.pick_point(*pos, PickMode::Nearest));
                InputResult::Consumed
            }
            _ => InputResult::Ignored,
        }
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        if let Some(t) = self.click
            && let Some(hits) = cx.poll_pick(t)
        {
            self.click = None;
            self.on_hits(hits, cx);
        }
        self.poll_job();
        // A referenced base moves with its construction plane (slider range, fallback plane).
        if let Some(e) = &self.picked_ref {
            let env = cx.env();
            if let Some(p) = crate::construct::base_plane(env.design, env.evaluated, e) {
                self.picked = Some(p);
            }
        }
        self.sync_draft();
        self.push_preview(cx);
        if self.job.is_some() || self.click.is_some() {
            cx.app()
                .request_repaint_after(std::time::Duration::from_millis(30));
        }
    }

    fn overlay(&mut self, out: &mut amg_render::OverlayList, cx: &ToolCx<'_>) {
        // The draft's plane as a translucent square over the scene.
        let env = cx.env();
        let (plane, _) = self
            .draft
            .plane(&|e| crate::construct::base_plane(env.design, env.evaluated, e));
        let half = if self.bounds.is_empty() {
            50.0
        } else {
            self.bounds.diagonal() * 0.6
        };
        out.plane_quad(
            &plane,
            [half, half],
            [0.25, 0.5, 0.95, 0.08],
            amg_render::LineStyle {
                width_px: 1.5,
                color: [0.25, 0.5, 0.95, 0.9],
                dash: Some((6.0, 4.0)),
                depth: amg_render::DepthMode::Test,
            },
            None,
        );
    }

    fn dialog_ui(&mut self, ui: &mut egui::Ui, cx: &mut ToolCx<'_>) -> bool {
        let title = if self.editing.is_some() {
            "Edit Section Analysis"
        } else {
            "Section Analysis"
        };
        let valid = self.draft.validate().is_ok()
            && !(self.scope == ScopeChoice::Listed && self.listed.is_empty());
        let stats = self.current_stats(cx);
        let action = DialogFrame::new(title)
            .ok_enabled(if valid {
                crate::command::Enablement::Enabled
            } else {
                crate::command::Enablement::disabled("Check the values")
            })
            .show_inside(ui, |ui| {
                self.body_ui(ui, cx);
                ui.separator();
                match &stats {
                    Some(s) => stats_ui(ui, s),
                    None => {
                        ui.horizontal(|ui| {
                            ui.spinner();
                            ui.weak("Updating caps…");
                        });
                    }
                }
            });
        self.sync_draft();
        self.push_preview(cx);
        let snap = self.snapshot(stats);
        ui.ctx().data_mut(|d| d.insert_temp(snapshot_id(), snap));
        match action {
            DialogAction::Ok => cx.request_finish(FinishReason::Commit),
            DialogAction::Cancel => cx.request_finish(FinishReason::Cancel),
            DialogAction::None => {}
        }
        true
    }

    fn pick_filter(&self) -> EntityFilter {
        if self.picking {
            EntityFilter::FACE.union(EntityFilter::CONSTRUCTION)
        } else {
            EntityFilter::BODY
        }
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        Some(if self.picking {
            "Section Analysis: click a planar face or construction plane for the base; Esc cancels"
                .into()
        } else {
            "Section Analysis: set the offset; Enter applies, Esc cancels".into()
        })
    }

    fn finish(&mut self, reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        if let Some(j) = self.job.take() {
            j.cancel();
        }
        if let Some(l) = cx.app().doc_layer::<SectionLayer>() {
            l.set_preview(None);
        }
        if reason != FinishReason::Commit {
            return None;
        }
        self.sync_draft();
        if self.draft.validate().is_err() {
            return None;
        }
        let name = self.draft.name.trim();
        if name.is_empty() {
            self.draft.name = next_name(cx.env().design);
        } else if name.len() != self.draft.name.len() {
            self.draft.name = name.to_owned();
        }
        Some(match self.editing.clone() {
            Some(key) => CommitBatch {
                label: "Edit Section Analysis".into(),
                commands: vec![self.draft.command(key)],
            },
            None => CommitBatch {
                label: "Section Analysis".into(),
                commands: vec![self.draft.add_command()],
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mm_text_trims() {
        assert_eq!(mm_text(10.0), "10 mm");
        assert_eq!(mm_text(-2.5), "-2.5 mm");
        assert_eq!(mm_text(0.12345), "0.1235 mm");
        assert_eq!(mm_text(-0.00001), "0 mm");
    }
}
