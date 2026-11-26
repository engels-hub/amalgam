//! Section analysis in the viewport (S05 §3, INS-09): the per-document [`SectionLayer`], the
//! browser "Analysis" folder and the Section Statistics panel. The dialog is
//! [`super::section_tool::SectionTool`].
//!
//! - **Persistence.** Analyses are `inspect.section` extension records
//!   ([`amg_features::inspect::doc::SectionAnalysis`]): created, edited, hidden, renamed and
//!   deleted through `Command::AddExt` / `SetExt`, one undo step each, saved with the document.
//! - **Combining.** Every visible analysis gives one plane; the kept region is the
//!   intersection of their kept half-spaces (at most 6 planes, `inspect::doc::combine`). While
//!   the dialog is open its draft replaces (or, for a new analysis, joins) the records.
//! - **Rendering split.** The planes go to RND-06 every frame through
//!   [`LayerOut::sections`](crate::layer::LayerOut) (the GPU clips and caps closed bodies, so
//!   dragging the offset slider costs no geometry work on the UI thread). The bodies in the
//!   analyses' scope are clipped (`Instance::sectioned`), with the analysis cap colour.
//! - **Caps job.** The geometric caps (INS-02 `section_mesh`) run as one `amg-jobs` job
//!   ([`SECTION_JOB`]) over the bodies in scope (every visible placement of each); a newer plane
//!   set or body change cancels it and starts a new one (supersede). Its outlines and open
//!   curves are drawn as RND-07 polylines once they match the current planes, and its
//!   [`SectionStats`] feed the panel and the dialog.
//! - **Solids** (INS-11). A solid body is cut on its B-rep tessellation: one
//!   [`SECTION_TESS_JOB`] per shape, which plane changes do not cancel, kept while the shape is
//!   unchanged. The caps job starts once every solid in scope is tessellated.
//! - **Reference bases** (INS-11). `SectionBase::Ref` construction planes are resolved every
//!   frame; faces need a kernel query, so they are resolved in a [`SECTION_REF_JOB`] whenever
//!   the evaluated state changes. While a face resolves, its last resolved plane (else the
//!   stored one) is used.
//! - **Fallback.** A base that no longer resolves keeps cutting and raises a [`BaseWarning`]
//!   (toast, panel, dialog). A face keeps the last plane it resolved to while the document was
//!   open (the stored plane after reopening); a construction plane falls back to the stored
//!   plane (CON-04). The record is only changed by the dialog's OK.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_brep::Shape;
use amg_core::{Aabb2, Plane, Transform};
use amg_doc::{BodyGeom, BodyId, DocEvent, EntityRef, Evaluated, ExtKey, Revision};
use amg_features::inspect::InspectError;
use amg_features::inspect::doc::base::{BaseError, ResolvedBase, is_construction, resolve_base};
use amg_features::inspect::doc::{
    CapColor, HatchPattern, HatchStyle, SECTION_KIND, SectionAnalysis, SectionBase, analyses,
    combine, delete_command,
};
use amg_features::inspect::section::{
    BodySection, CapOpts, SectionPlanes, section_mesh, tessellate_solid,
};
use amg_jobs::{JobCtx, JobError, JobHandle, JobId, SpawnOptions};
use amg_mesh::MeshBody;
use amg_render::{DepthMode, LineStyle, OverlayList};

use super::props::sig;
use crate::app::{AppCx, CmdEnv, CommitBatch};
use crate::browser::{BrowserAnchor, BrowserNode, BrowserProvider, NodeKey};
use crate::command::{CmdId, Enablement};
use crate::design::DESIGN;
use crate::layer::{DocLayer, DocLayerDef, LayerOut};
use crate::module::PanelId;
use crate::notify::Notification;
use crate::panel::{DockSlot, Panel, PanelDef};
use crate::scene::SectionStyle;

/// Starts the Section Analysis tool (new analysis).
pub const SECTION_CMD: CmdId = CmdId("inspect.section");
/// Opens the Section Statistics panel.
pub const SECTION_STATS_CMD: CmdId = CmdId("inspect.section_stats");
/// The Section Statistics panel.
pub const SECTION_PANEL: PanelId = PanelId("inspect.section_stats");
/// Label of the caps job (jobs panel).
pub const SECTION_JOB: &str = "Section caps";
/// Label of the job that tessellates a solid body in scope (INS-11).
pub const SECTION_TESS_JOB: &str = "Section tessellation";
/// Label of the job that resolves face bases (INS-11).
pub const SECTION_REF_JOB: &str = "Section reference";
/// Id of the document layer.
pub const SECTION_LAYER: &str = "inspect.section";
/// The browser folder of the analyses.
pub const ANALYSIS_FOLDER: NodeKey = NodeKey::new("inspect.analysis");
/// Prefix of an analysis row's key (`inspect.section/<ext id>`).
const ROW_PREFIX: &str = "inspect.section/";

/// Colour of the closed section outlines (linear RGBA).
const OUTLINE: [f32; 4] = [0.02, 0.02, 0.025, 1.0];
/// Colour of the open section curves of non-closed bodies.
const OPEN_CURVE: [f32; 4] = [0.9, 0.12, 0.08, 1.0];
/// Colour of hatch lines.
const HATCH_LINES: [f32; 4] = [0.02, 0.02, 0.025, 0.85];

/// The browser row key of the analysis under `key`.
pub fn row_key(key: &ExtKey) -> NodeKey {
    NodeKey::owned(format!("{ROW_PREFIX}{}", key.id))
}

/// The record key of an analysis row.
pub fn key_of_row(node: &NodeKey) -> Option<ExtKey> {
    let id = node.as_str().strip_prefix(ROW_PREFIX)?.parse().ok()?;
    Some(ExtKey {
        kind: SECTION_KIND.to_owned(),
        id,
    })
}

/// RND-06 hatch of a record hatch style.
pub fn render_hatch(h: &HatchStyle) -> amg_render::Hatch {
    match h.pattern {
        HatchPattern::None => amg_render::Hatch::NONE,
        HatchPattern::Lines45 | HatchPattern::Cross => amg_render::Hatch {
            angle: h.angle,
            spacing_px: h.spacing_px,
            color: HATCH_LINES,
            cross: h.pattern == HatchPattern::Cross,
        },
    }
}

/// Linear RGB of an sRGB colour.
pub fn linear_rgb(c: amg_mesh::Rgba8) -> [f32; 3] {
    let [r, g, b, a] = c.0;
    let l = egui::Rgba::from(egui::Color32::from_rgba_unmultiplied(r, g, b, a));
    let a = l.a().max(f32::EPSILON);
    [l.r() / a, l.g() / a, l.b() / a]
}

/// The dialog's draft, shown instead of (or in addition to) the records.
#[derive(Clone, Debug, PartialEq)]
pub struct Preview {
    /// The record it replaces; `None` for a new analysis (appended).
    pub key: Option<ExtKey>,
    /// The draft.
    pub analysis: SectionAnalysis,
}

/// One applied plane.
#[derive(Clone, Debug, PartialEq)]
pub struct PlaneEntry {
    /// Its record (`None`: the dialog's new analysis).
    pub key: Option<ExtKey>,
    /// Display name.
    pub name: String,
    /// The world plane (removes the side its normal points into).
    pub plane: Plane,
}

/// Section statistics of one analysis, summed over the bodies in scope.
#[derive(Clone, Debug, PartialEq)]
pub struct AnalysisStats {
    /// Its record (`None`: the dialog's new analysis).
    pub key: Option<ExtKey>,
    /// Display name.
    pub name: String,
    /// Σ cap area, mm².
    pub area: f64,
    /// Σ outline length, mm.
    pub perimeter: f64,
    /// Closed section loops.
    pub loops: u32,
    /// Open section curves (bodies that are not closed).
    pub open_loops: u32,
    /// Extent in the plane frame (union over the bodies).
    pub extent: Aabb2,
    /// Bodies with a section on this plane.
    pub bodies: usize,
}

/// The layer's state as tests, the panel and the dialog see it.
#[derive(Clone, Debug, Default)]
pub struct SectionSnapshot {
    /// The applied planes, in order.
    pub planes: Vec<PlaneEntry>,
    /// Statistics per applied plane (same order), from the newest finished caps job.
    pub stats: Vec<AnalysisStats>,
    /// The statistics and outlines match the current planes and bodies.
    pub current: bool,
    /// The running caps job.
    pub running: Option<JobId>,
    /// Caps jobs started so far.
    pub jobs_started: u64,
    /// Caps jobs cancelled by a newer plane set, with the time of the cancel.
    pub superseded: Vec<(JobId, Instant)>,
    /// Time from the last input change to the end of the caps job that matched it (the
    /// "caps catch up" latency), newest last.
    pub catch_up: Vec<Duration>,
    /// Names of visible analyses beyond the 6-plane limit (not applied).
    pub dropped: Vec<String>,
    /// Records that cannot be read (name or key, reason).
    pub bad: Vec<(ExtKey, String)>,
    /// The last caps-job error.
    pub error: Option<String>,
    /// Outline and open-curve polylines drawn this frame.
    pub outlines_drawn: usize,
    /// Reference bases that fell back or may have changed (INS-11).
    pub warnings: Vec<BaseWarning>,
    /// Solid bodies in scope still being tessellated (INS-11).
    pub tessellating: usize,
}

/// A problem with the reference base of one analysis.
#[derive(Clone, Debug, PartialEq)]
pub struct BaseWarning {
    /// Its record (`None`: the dialog's new analysis).
    pub key: Option<ExtKey>,
    /// Display name.
    pub name: String,
    /// What happened, in words.
    pub message: String,
    /// The reference no longer resolves, so a kept plane is used.
    pub fallback: bool,
}

/// Whether a face base resolved.
#[derive(Clone, Debug, PartialEq)]
enum FaceStatus {
    Found,
    /// Only the fingerprint matched.
    Uncertain,
    /// It did not resolve, with the reason.
    Lost(String),
}

/// The state of one face base.
#[derive(Clone, Debug)]
struct FaceBase {
    entity: EntityRef,
    /// The newest plane it resolved to.
    plane: Option<Plane>,
    status: FaceStatus,
}

type FaceResults = Vec<(EntityRef, Result<ResolvedBase, BaseError>)>;

struct Resolving {
    handle: JobHandle<FaceResults, InspectError>,
    entities: Vec<EntityRef>,
}

/// The section tessellation of one solid body.
struct SolidMesh {
    shape: Shape,
    state: SolidState,
}

enum SolidState {
    Running(JobHandle<MeshBody, InspectError>),
    Ready(MeshBody),
    Failed(String),
}

/// The geometry of a body in scope, before it is capped.
enum Source {
    Mesh(MeshBody),
    Solid(Shape),
}

/// The face reference of `a`'s base, if it needs a kernel query.
fn face_ref(a: &SectionAnalysis) -> Option<&EntityRef> {
    match &a.base {
        SectionBase::Ref { entity, .. } if !is_construction(entity) => Some(entity),
        _ => None,
    }
}

/// One placed body to cap.
#[derive(Clone, Debug)]
struct CapInput {
    body: BodyId,
    mesh: MeshBody,
    world: Transform,
}

/// What a caps result depends on.
type CapKey = Vec<u64>;

fn plane_bits(p: &Plane, out: &mut CapKey) {
    for v in [p.origin, p.x.get(), p.y.get(), p.normal.get()] {
        out.extend(v.to_array().map(f64::to_bits));
    }
}

/// A finished caps job.
#[derive(Debug)]
struct CapResult {
    key: CapKey,
    planes: usize,
    bodies: Vec<(BodyId, BodySection)>,
    finished: Instant,
}

struct Running {
    handle: JobHandle<CapResult, InspectError>,
    key: CapKey,
}

/// Per-document section state (a [`DocLayer`]); see the module docs.
pub struct SectionLayer {
    rev: Option<Revision>,
    records: Vec<(ExtKey, SectionAnalysis)>,
    bad: Vec<(ExtKey, String)>,
    preview: Option<Preview>,
    pending_edit: Option<ExtKey>,
    planes: Vec<PlaneEntry>,
    dropped: Vec<String>,
    key: Option<CapKey>,
    changed_at: Instant,
    job: Option<Running>,
    result: Option<Arc<CapResult>>,
    jobs_started: u64,
    superseded: Vec<(JobId, Instant)>,
    catch_up: Vec<Duration>,
    error: Option<String>,
    outlines_drawn: usize,
    faces: Vec<FaceBase>,
    /// The evaluated state the face bases were (or are being) resolved against.
    faces_eval: Option<Arc<Evaluated>>,
    resolving: Option<Resolving>,
    solids: BTreeMap<BodyId, SolidMesh>,
    tessellating: usize,
    tess_errors: Vec<String>,
    warnings: Vec<BaseWarning>,
}

impl std::fmt::Debug for SectionLayer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SectionLayer")
            .field("records", &self.records.len())
            .field("planes", &self.planes.len())
            .field("running", &self.job.is_some())
            .finish_non_exhaustive()
    }
}

impl Default for SectionLayer {
    fn default() -> SectionLayer {
        SectionLayer {
            rev: None,
            records: Vec::new(),
            bad: Vec::new(),
            preview: None,
            pending_edit: None,
            planes: Vec::new(),
            dropped: Vec::new(),
            key: None,
            changed_at: Instant::now(),
            job: None,
            result: None,
            jobs_started: 0,
            superseded: Vec::new(),
            catch_up: Vec::new(),
            error: None,
            outlines_drawn: 0,
            faces: Vec::new(),
            faces_eval: None,
            resolving: None,
            solids: BTreeMap::new(),
            tessellating: 0,
            tess_errors: Vec::new(),
            warnings: Vec::new(),
        }
    }
}

fn create_layer() -> Box<dyn DocLayer> {
    Box::new(SectionLayer::default())
}

impl SectionLayer {
    /// Shows the dialog's draft (`None`: only the records).
    pub fn set_preview(&mut self, p: Option<Preview>) {
        if self.preview != p {
            self.preview = p;
        }
    }

    /// The dialog's draft.
    pub fn preview(&self) -> Option<&Preview> {
        self.preview.as_ref()
    }

    /// Asks the next Section Analysis tool to edit `key` (browser double-click / Edit).
    pub fn request_edit(&mut self, key: ExtKey) {
        self.pending_edit = Some(key);
    }

    /// The analysis to edit, once.
    pub fn take_edit(&mut self) -> Option<ExtKey> {
        self.pending_edit.take()
    }

    /// The decoded records, in key order.
    pub fn records(&self) -> &[(ExtKey, SectionAnalysis)] {
        &self.records
    }

    /// The newest plane the face base `entity` resolved to (`None`: not resolved yet, or not a
    /// face base of any analysis).
    pub fn face_plane(&self, entity: &EntityRef) -> Option<Plane> {
        self.faces.iter().find(|f| &f.entity == entity)?.plane
    }

    /// The state as of the last frame.
    pub fn snapshot(&self) -> SectionSnapshot {
        let current = self.is_current() && self.resolving.is_none() && self.tessellating == 0;
        SectionSnapshot {
            planes: self.planes.clone(),
            stats: self.stats(),
            current,
            running: self.job.as_ref().map(|j| j.handle.id()),
            jobs_started: self.jobs_started,
            superseded: self.superseded.clone(),
            catch_up: self.catch_up.clone(),
            dropped: self.dropped.clone(),
            bad: self.bad.clone(),
            error: self
                .error
                .clone()
                .or_else(|| self.tess_errors.first().cloned()),
            outlines_drawn: self.outlines_drawn,
            warnings: self.warnings.clone(),
            tessellating: self.tessellating,
        }
    }

    fn is_current(&self) -> bool {
        match (&self.result, &self.key) {
            (Some(r), Some(k)) => &r.key == k,
            (None, None) => true,
            _ => false,
        }
    }

    /// Statistics per applied plane from the newest result (empty when the plane count
    /// differs).
    fn stats(&self) -> Vec<AnalysisStats> {
        let Some(r) = &self.result else {
            return Vec::new();
        };
        if r.planes != self.planes.len() {
            return Vec::new();
        }
        self.planes
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let mut s = AnalysisStats {
                    key: p.key.clone(),
                    name: p.name.clone(),
                    area: 0.0,
                    perimeter: 0.0,
                    loops: 0,
                    open_loops: 0,
                    extent: Aabb2::EMPTY,
                    bodies: 0,
                };
                let mut bodies = BTreeSet::new();
                for (id, b) in &r.bodies {
                    let Some(st) = b.stats.get(i) else { continue };
                    if st.area.get() > 0.0 || st.loops > 0 || st.open_loops > 0 {
                        bodies.insert(*id);
                        s.bodies = bodies.len();
                    }
                    s.area += st.area.get();
                    s.perimeter += st.perimeter.get();
                    s.loops += st.loops;
                    s.open_loops += st.open_loops;
                    s.extent = s.extent.union(&st.extent);
                }
                s
            })
            .collect()
    }

    fn reload(&mut self, env: &CmdEnv<'_>) {
        let rev = env.design.revision();
        if self.rev == Some(rev) {
            return;
        }
        self.rev = Some(rev);
        self.records.clear();
        self.bad.clear();
        for (k, r) in analyses(env.design) {
            match r {
                Ok(a) => self.records.push((k, a)),
                Err(e) => self.bad.push((k, e.to_string())),
            }
        }
    }

    /// The analyses in effect: the records with the draft applied.
    fn effective(&self) -> Vec<(Option<ExtKey>, &SectionAnalysis)> {
        let mut list: Vec<(Option<ExtKey>, &SectionAnalysis)> = self
            .records
            .iter()
            .map(|(k, a)| match &self.preview {
                Some(p) if p.key.as_ref() == Some(k) => (Some(k.clone()), &p.analysis),
                _ => (Some(k.clone()), a),
            })
            .collect();
        if let Some(p) = &self.preview
            && p.key.is_none()
        {
            list.push((None, &p.analysis));
        }
        list
    }

    /// Brings the face bases up to date: resolves the references in `wanted` in a job when the
    /// evaluated state or the set of references changed.
    fn update_faces(&mut self, cx: &mut AppCx<'_>, wanted: &[EntityRef]) {
        self.faces.retain(|f| wanted.contains(&f.entity));
        if wanted.is_empty() {
            if let Some(r) = self.resolving.take() {
                r.handle.cancel();
            }
            self.faces_eval = None;
            return;
        }
        let eval = Arc::clone(cx.env().evaluated);
        let stale = self
            .faces_eval
            .as_ref()
            .is_none_or(|e| !Arc::ptr_eq(e, &eval));
        let missing = wanted.iter().any(|e| {
            !self.faces.iter().any(|f| &f.entity == e)
                && !self
                    .resolving
                    .as_ref()
                    .is_some_and(|r| r.entities.contains(e))
        });
        if stale || missing {
            if let Some(r) = self.resolving.take() {
                r.handle.cancel();
            }
            let design = cx.env().design.clone();
            let entities = wanted.to_vec();
            let job_entities = entities.clone();
            let job_eval = Arc::clone(&eval);
            let handle = cx.jobs().spawn_fn(
                SECTION_REF_JOB,
                SpawnOptions::default(),
                move |ctx: &JobCtx| -> Result<FaceResults, InspectError> {
                    let mut out = Vec::with_capacity(job_entities.len());
                    for e in job_entities {
                        ctx.checkpoint()?;
                        let r = resolve_base(&job_eval, &design, &e);
                        out.push((e, r));
                    }
                    Ok(out)
                },
            );
            self.resolving = Some(Resolving { handle, entities });
            self.faces_eval = Some(eval);
        }
        let Some(r) = &mut self.resolving else { return };
        let Some(result) = r.handle.try_take() else {
            return;
        };
        let entities = std::mem::take(&mut r.entities);
        self.resolving = None;
        let results: Vec<(EntityRef, Result<ResolvedBase, String>)> = match result {
            Ok(list) => list
                .into_iter()
                .map(|(e, r)| (e, r.map_err(|e| e.to_string())))
                .collect(),
            Err(JobError::Cancelled) => {
                self.faces_eval = None;
                return;
            }
            Err(e) => entities
                .into_iter()
                .map(|r| (r, Err(e.to_string())))
                .collect(),
        };
        for (entity, result) in results {
            if !wanted.contains(&entity) {
                continue;
            }
            let i = match self.faces.iter().position(|f| f.entity == entity) {
                Some(i) => i,
                None => {
                    self.faces.push(FaceBase {
                        entity,
                        plane: None,
                        status: FaceStatus::Found,
                    });
                    self.faces.len() - 1
                }
            };
            let face = &mut self.faces[i];
            match result {
                Ok(b) => {
                    face.plane = Some(b.plane);
                    face.status = if b.uncertain {
                        FaceStatus::Uncertain
                    } else {
                        FaceStatus::Found
                    };
                }
                Err(why) => face.status = FaceStatus::Lost(why),
            }
        }
    }

    /// The warning of `a`'s reference base, if any, and whether a kept plane is used.
    fn base_warning(&self, env: &CmdEnv<'_>, a: &SectionAnalysis) -> Option<(String, bool)> {
        let SectionBase::Ref { entity, .. } = &a.base else {
            return None;
        };
        if is_construction(entity) {
            return crate::construct::base_plane(env.design, env.evaluated, entity)
                .is_none()
                .then(|| {
                    (
                        "the base construction plane is gone; using the last plane".to_owned(),
                        true,
                    )
                });
        }
        match &self.faces.iter().find(|f| &f.entity == entity)?.status {
            FaceStatus::Found => None,
            FaceStatus::Uncertain => Some((
                "the original base face is gone; matched a face with the same shape and position"
                    .to_owned(),
                false,
            )),
            FaceStatus::Lost(why) => Some((
                format!("the base face is gone ({why}); using the last plane"),
                true,
            )),
        }
    }

    /// Replaces the warnings and shows a toast for each new one.
    fn set_warnings(&mut self, cx: &mut AppCx<'_>, warnings: Vec<BaseWarning>) {
        for w in &warnings {
            let known = self
                .warnings
                .iter()
                .any(|o| o.name == w.name && o.message == w.message);
            if !known {
                cx.notify(Notification::warning(
                    "Section Analysis",
                    format!("{}: {}", w.name, w.message),
                ));
            }
        }
        self.warnings = warnings;
    }

    /// The section tessellation of solid `body`, or `None` while it is being made or after it
    /// failed. A new shape starts a new job.
    fn solid_mesh(&mut self, cx: &mut AppCx<'_>, body: BodyId, shape: Shape) -> Option<MeshBody> {
        if !self
            .solids
            .get(&body)
            .is_some_and(|m| m.shape.ptr_eq(&shape))
        {
            let job_shape = shape.clone();
            let handle = cx.jobs().spawn_fn(
                SECTION_TESS_JOB,
                SpawnOptions::default(),
                move |ctx: &JobCtx| tessellate_solid(&job_shape, ctx),
            );
            let new = SolidMesh {
                shape,
                state: SolidState::Running(handle),
            };
            if let Some(SolidMesh {
                state: SolidState::Running(old),
                ..
            }) = self.solids.insert(body, new)
            {
                old.cancel();
            }
        }
        let entry = self.solids.get_mut(&body)?;
        if let SolidState::Running(handle) = &mut entry.state
            && let Some(r) = handle.try_take()
        {
            entry.state = match r {
                Ok(mesh) => SolidState::Ready(mesh),
                Err(e) => SolidState::Failed(e.to_string()),
            };
        }
        match &entry.state {
            SolidState::Running(_) => {
                self.tessellating += 1;
                None
            }
            SolidState::Ready(mesh) => Some(mesh.clone()),
            SolidState::Failed(e) => {
                self.tess_errors
                    .push(format!("a solid body cannot be sectioned: {e}"));
                None
            }
        }
    }

    /// Drops the tessellations of bodies that are gone or changed (all of them when `unused`).
    fn prune_solids(&mut self, eval: &Evaluated, unused: bool) {
        self.solids.retain(|id, m| {
            let keep = !unused
                && matches!(eval.body(*id).map(|b| &b.geom),
                    Some(BodyGeom::Solid(s)) if s.brep().is_some_and(|b| b.ptr_eq(&m.shape)));
            if !keep && let SolidState::Running(h) = &m.state {
                h.cancel();
            }
            keep
        });
    }

    /// The face references of the records and of the dialog's draft.
    fn face_refs(&self) -> Vec<EntityRef> {
        let mut out: Vec<EntityRef> = Vec::new();
        let records = self.records.iter().map(|(_, a)| a);
        for a in records.chain(self.preview.iter().map(|p| &p.analysis)) {
            if let Some(e) = face_ref(a)
                && !out.contains(e)
            {
                out.push(e.clone());
            }
        }
        out
    }

    fn update_inner(&mut self, cx: &mut AppCx<'_>, out: &mut LayerOut<'_>) {
        self.reload(&cx.env());
        let wanted = self.face_refs();
        self.update_faces(cx, &wanted);
        let env = cx.env();
        // Planes.
        let (planes, used, dropped, warnings) = {
            // A face base that resolved once keeps that plane as its fallback.
            let list: Vec<(Option<ExtKey>, SectionAnalysis)> = self
                .effective()
                .into_iter()
                .map(|(k, a)| {
                    let mut a = a.clone();
                    if let SectionBase::Ref { entity, last } = &mut a.base
                        && !is_construction(entity)
                        && let Some(p) = self.face_plane(entity)
                    {
                        *last = p;
                    }
                    (k, a)
                })
                .collect();
            let warnings: Vec<BaseWarning> = list
                .iter()
                .filter_map(|(key, a)| {
                    let (message, fallback) = self.base_warning(&env, a)?;
                    Some(BaseWarning {
                        key: key.clone(),
                        name: a.name.clone(),
                        message,
                        fallback,
                    })
                })
                .collect();
            let refs: Vec<&SectionAnalysis> = list.iter().map(|(_, a)| a).collect();
            // `Ref` bases follow their entity through every recompute (CON-04, INS-11).
            let c = combine(&refs, &|e| {
                if is_construction(e) {
                    return crate::construct::base_plane(env.design, env.evaluated, e);
                }
                let face = self.faces.iter().find(|f| &f.entity == e)?;
                match face.status {
                    FaceStatus::Lost(_) => None,
                    FaceStatus::Found | FaceStatus::Uncertain => face.plane,
                }
            });
            let planes: Vec<PlaneEntry> = c
                .owners
                .iter()
                .zip(&c.planes.0)
                .map(|(&i, p)| PlaneEntry {
                    key: list[i].0.clone(),
                    name: list[i].1.name.clone(),
                    plane: *p,
                })
                .collect();
            let used: Vec<SectionAnalysis> = c.owners.iter().map(|&i| list[i].1.clone()).collect();
            let dropped: Vec<String> = c.dropped.iter().map(|&i| list[i].1.name.clone()).collect();
            (planes, used, dropped, warnings)
        };
        self.planes = planes;
        self.dropped = dropped;
        out.sections.planes = self
            .planes
            .iter()
            .zip(&used)
            .map(|(p, a)| amg_render::SectionPlane {
                plane: p.plane,
                caps: true,
                hatch: render_hatch(&a.hatch),
                enabled: true,
            })
            .collect();
        // Bodies in scope: clipped, with the first scoped analysis's uniform cap colour.
        let mut sources: Vec<(BodyId, Source, Vec<Transform>)> = Vec::new();
        let mut style = SectionStyle::default();
        if !used.is_empty() {
            let mut clipped = BTreeSet::new();
            for (id, b) in env.evaluated.bodies() {
                let visible = env.design.body_props(id).visible;
                let covering: Vec<&SectionAnalysis> = used
                    .iter()
                    .filter(|a| a.scope.covers(id, visible))
                    .collect();
                if covering.is_empty() {
                    continue;
                }
                clipped.insert(id);
                if let Some(c) = covering.iter().find_map(|a| match a.cap_color {
                    CapColor::Uniform(c) => Some(c),
                    CapColor::BodyColor => None,
                }) {
                    style.cap_colors.insert(id, linear_rgb(c));
                }
                let source = match &b.geom {
                    BodyGeom::Mesh(m) => Source::Mesh(MeshBody::from_arc(Arc::clone(m))),
                    BodyGeom::Solid(s) => match s.brep() {
                        Some(shape) => Source::Solid(shape.clone()),
                        None => continue,
                    },
                    _ => continue,
                };
                let instances = env.evaluated.instances(id);
                let worlds = if instances.is_empty() {
                    vec![Transform::IDENTITY]
                } else {
                    instances
                        .into_iter()
                        .filter(|i| {
                            i.path
                                .iter()
                                .all(|o| env.design.occurrence(*o).is_some_and(|o| o.visible))
                        })
                        .map(|i| i.world)
                        .collect()
                };
                if !worlds.is_empty() {
                    sources.push((id, source, worlds));
                }
            }
            style.clipped = Some(clipped);
        }
        out.sections.style = style;
        let eval = Arc::clone(env.evaluated);
        self.prune_solids(&eval, self.records.is_empty() && self.preview.is_none());
        self.set_warnings(cx, warnings);
        self.tessellating = 0;
        self.tess_errors.clear();
        let mut inputs = Vec::new();
        for (body, source, worlds) in sources {
            let mesh = match source {
                Source::Mesh(m) => Some(m),
                Source::Solid(shape) => self.solid_mesh(cx, body, shape),
            };
            if let Some(mesh) = mesh {
                inputs.extend(worlds.into_iter().map(|world| CapInput {
                    body,
                    mesh: mesh.clone(),
                    world,
                }));
            }
        }
        // The caps job.
        // While a solid is tessellated, the previous result stays (not current) and no caps
        // job runs.
        let waiting = !self.planes.is_empty() && self.tessellating > 0;
        let key = if self.planes.is_empty() || inputs.is_empty() || waiting {
            None
        } else {
            let mut k = CapKey::new();
            for p in &self.planes {
                plane_bits(&p.plane, &mut k);
            }
            for i in &inputs {
                k.push(i.body.origin.0);
                k.push(u64::from(i.body.slot));
                k.push(Arc::as_ptr(i.mesh.mesh()) as usize as u64);
                for c in i.world.matrix3.to_cols_array() {
                    k.push(c.to_bits());
                }
                k.extend(i.world.translation.to_array().map(f64::to_bits));
            }
            Some(k)
        };
        if key != self.key {
            self.key = key.clone();
            self.changed_at = Instant::now();
            if let Some(j) = self.job.take() {
                j.handle.cancel();
                self.superseded.push((j.handle.id(), Instant::now()));
            }
            self.error = None;
            match key {
                None if waiting => {}
                None => self.result = None,
                Some(key) => {
                    let planes = SectionPlanes(self.planes.iter().map(|p| p.plane).collect());
                    let jkey = key.clone();
                    let handle = cx.jobs().spawn_fn(
                        SECTION_JOB,
                        SpawnOptions::default(),
                        move |ctx: &JobCtx| -> Result<CapResult, InspectError> {
                            let n = inputs.len().max(1) as f32;
                            let mut bodies = Vec::with_capacity(inputs.len());
                            for (i, inp) in inputs.iter().enumerate() {
                                ctx.checkpoint()?;
                                let sub = ctx.sub(i as f32 / n, (i + 1) as f32 / n);
                                let s = section_mesh(
                                    &inp.mesh,
                                    &inp.world,
                                    &planes,
                                    &CapOpts::default(),
                                    &sub,
                                )?;
                                bodies.push((inp.body, s));
                            }
                            Ok(CapResult {
                                key: jkey,
                                planes: planes.0.len(),
                                bodies,
                                finished: Instant::now(),
                            })
                        },
                    );
                    self.jobs_started += 1;
                    self.job = Some(Running { handle, key });
                }
            }
        }
        // Collect a finished job.
        if let Some(j) = &mut self.job
            && let Some(r) = j.handle.try_take()
        {
            let key = j.key.clone();
            self.job = None;
            match r {
                Ok(res) => {
                    if self.key.as_ref() == Some(&key) {
                        self.catch_up
                            .push(res.finished.saturating_duration_since(self.changed_at));
                    }
                    self.result = Some(Arc::new(res));
                }
                Err(JobError::Cancelled) => {}
                Err(e) => self.error = Some(e.to_string()),
            }
        }
        if self.job.is_some() || self.resolving.is_some() || self.tessellating > 0 {
            cx.request_repaint_after(Duration::from_millis(16));
        }
        // Outlines and open curves of the current caps.
        self.outlines_drawn = 0;
        if self.is_current()
            && self.tessellating == 0
            && let Some(r) = &self.result
        {
            self.outlines_drawn = draw_outlines(out.overlays, r);
        }
    }
}

fn draw_outlines(out: &mut OverlayList, r: &CapResult) -> usize {
    let mut n = 0;
    let outline = LineStyle {
        width_px: 2.0,
        color: OUTLINE,
        dash: None,
        depth: DepthMode::XRay { hidden_alpha: 0.2 },
    };
    let open = LineStyle {
        width_px: 2.5,
        color: OPEN_CURVE,
        ..outline
    };
    for (_, b) in &r.bodies {
        for cap in &b.caps {
            for (lines, style) in [(&cap.outlines, outline), (&cap.open, open)] {
                for l in lines {
                    if l.closed && l.points.len() > 2 {
                        let mut pts = l.points.clone();
                        pts.push(l.points[0]);
                        out.polyline(&pts, style, None);
                    } else {
                        out.polyline(&l.points, style, None);
                    }
                    n += 1;
                }
            }
        }
    }
    n
}

impl DocLayer for SectionLayer {
    fn update(&mut self, cx: &mut AppCx<'_>, out: &mut LayerOut<'_>) {
        self.update_inner(cx, out);
    }
}

// ---- text --------------------------------------------------------------------------------------

/// The statistics rows of one analysis (label, value).
pub fn stats_rows(s: &AnalysisStats) -> Vec<(&'static str, String)> {
    let mut rows = vec![
        ("Section area", format!("{} mm²", sig(s.area, 7))),
        ("Perimeter", format!("{} mm", sig(s.perimeter, 7))),
        ("Loops", s.loops.to_string()),
    ];
    if !s.extent.is_empty() {
        let d = s.extent.size();
        rows.push(("Extent", format!("{} × {} mm", sig(d.x, 6), sig(d.y, 6))));
    }
    rows
}

/// The warning for open section curves, if any.
pub fn open_warning(s: &AnalysisStats) -> Option<String> {
    (s.open_loops > 0).then(|| {
        format!(
            "body is not closed: {} open section curve{}",
            s.open_loops,
            if s.open_loops == 1 { "" } else { "s" }
        )
    })
}

/// The text of a reference-base warning (panel and dialog).
pub fn warning_text(w: &BaseWarning) -> String {
    format!("Warning, {}: {}", w.name, w.message)
}

/// Draws the statistics of `s` (shared by the panel and the dialog).
pub fn stats_ui(ui: &mut egui::Ui, s: &AnalysisStats) {
    egui::Grid::new(("section_stats", s.key.as_ref().map_or(0, |k| k.id)))
        .num_columns(1)
        .striped(true)
        .show(ui, |ui| {
            for (l, v) in stats_rows(s) {
                ui.label(format!("{l}: {v}"));
                ui.end_row();
            }
        });
    if let Some(w) = open_warning(s) {
        ui.colored_label(ui.visuals().warn_fg_color, w);
    }
}

// ---- commands on records ----------------------------------------------------------------------

/// Section Analysis is enabled while the design has a body it can cut: a mesh, or a solid with
/// B-rep geometry.
pub(crate) fn enabled(env: &CmdEnv<'_>) -> Enablement {
    let cut = env.evaluated.bodies().any(|(_, b)| match &b.geom {
        BodyGeom::Mesh(_) => true,
        BodyGeom::Solid(s) => s.brep().is_some(),
        _ => false,
    });
    if cut {
        Enablement::Enabled
    } else {
        Enablement::disabled("needs a mesh or solid body")
    }
}

/// Submits `analysis` under `key` as one undo step labelled `label`.
fn submit_set(cx: &mut AppCx<'_>, label: &str, key: ExtKey, analysis: &SectionAnalysis) {
    cx.submit(CommitBatch {
        label: label.to_owned(),
        commands: vec![analysis.command(key)],
    });
}

fn record(env: &CmdEnv<'_>, key: &ExtKey) -> Option<SectionAnalysis> {
    SectionAnalysis::from_record(env.design.ext(key)?).ok()
}

/// Opens the Section Analysis tool on the record under `key`.
pub fn edit(cx: &mut AppCx<'_>, key: ExtKey) {
    if let Some(l) = cx.doc_layer::<SectionLayer>() {
        l.request_edit(key);
        cx.start_tool(SECTION_CMD);
    }
}

/// Deletes the record under `key` (one undo step).
pub fn delete(cx: &mut AppCx<'_>, key: ExtKey) {
    cx.submit(CommitBatch {
        label: "Delete Section Analysis".to_owned(),
        commands: vec![delete_command(key)],
    });
}

/// Shows or hides the record under `key` (one undo step).
pub fn set_visible(cx: &mut AppCx<'_>, key: ExtKey, visible: bool) {
    let Some(mut a) = record(&cx.env(), &key) else {
        return;
    };
    if a.visible == visible {
        return;
    }
    a.visible = visible;
    let label = if visible {
        "Show Section Analysis"
    } else {
        "Hide Section Analysis"
    };
    submit_set(cx, label, key, &a);
}

/// Renames the record under `key` (one undo step).
pub fn rename(cx: &mut AppCx<'_>, key: ExtKey, name: &str) {
    let Some(mut a) = record(&cx.env(), &key) else {
        return;
    };
    if a.name == name {
        return;
    }
    name.clone_into(&mut a.name);
    submit_set(cx, "Rename Section Analysis", key, &a);
}

// ---- browser ------------------------------------------------------------------------------------

/// The "Analysis" folder at the browser root: one row per section analysis with its eye.
#[derive(Debug)]
struct AnalysisNodes;

impl BrowserProvider for AnalysisNodes {
    fn id(&self) -> &'static str {
        "inspect.section"
    }

    fn affected_by(&self, ev: &DocEvent) -> bool {
        matches!(ev, DocEvent::ExtChanged(k) if k.kind == SECTION_KIND)
    }

    fn build(&self, env: &CmdEnv<'_>, anchor: &BrowserAnchor, out: &mut Vec<BrowserNode>) {
        if *anchor != BrowserAnchor::Root {
            return;
        }
        let rows: Vec<BrowserNode> = analyses(env.design)
            .into_iter()
            .map(|(k, r)| match r {
                Ok(a) => BrowserNode::new(row_key(&k), a.name)
                    .icon(crate::icons::ids::SECTION)
                    .visible(a.visible)
                    .renamable(),
                Err(_) => BrowserNode::new(row_key(&k), format!("Section {} (unreadable)", k.id))
                    .badge(Some(amg_core::Severity::Error)),
            })
            .collect();
        if rows.is_empty() {
            return;
        }
        out.push(
            BrowserNode::new(ANALYSIS_FOLDER, "Analysis")
                .icon(crate::icons::ids::ANALYSIS_FOLDER)
                .children(rows),
        );
    }

    fn set_visible(&self, node: &NodeKey, visible: bool, cx: &mut AppCx<'_>) {
        if let Some(k) = key_of_row(node) {
            set_visible(cx, k, visible);
        }
    }

    fn rename(&self, node: &NodeKey, name: &str, cx: &mut AppCx<'_>) {
        if let Some(k) = key_of_row(node) {
            rename(cx, k, name);
        }
    }

    fn double_click(&self, node: &NodeKey, cx: &mut AppCx<'_>) {
        if let Some(k) = key_of_row(node) {
            edit(cx, k);
        }
    }

    fn context_menu(&self, node: &NodeKey, ui: &mut egui::Ui, cx: &mut AppCx<'_>) {
        let Some(k) = key_of_row(node) else {
            return;
        };
        if ui.button("Edit Section Analysis").clicked() {
            edit(cx, k.clone());
            ui.close();
        }
        if ui.button("Delete").clicked() {
            delete(cx, k);
            ui.close();
        }
    }
}

// ---- panel --------------------------------------------------------------------------------------

/// The Section Statistics panel: area, perimeter, loops and extent of every applied analysis.
#[derive(Debug, Default)]
struct StatsPanel;

impl Panel for StatsPanel {
    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut AppCx<'_>) {
        let Some(s) = cx.doc_layer::<SectionLayer>().map(|l| l.snapshot()) else {
            ui.label("No section analysis.");
            return;
        };
        if s.planes.is_empty() {
            ui.label("No visible section analysis.");
        }
        for w in &s.warnings {
            ui.colored_label(ui.visuals().warn_fg_color, warning_text(w));
        }
        if !s.current || s.running.is_some() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("Updating caps…");
            });
        }
        if let Some(e) = &s.error {
            ui.colored_label(ui.visuals().error_fg_color, e);
        }
        for st in &s.stats {
            ui.heading(&st.name);
            stats_ui(ui, st);
            ui.separator();
        }
        if !s.dropped.is_empty() {
            ui.colored_label(
                ui.visuals().warn_fg_color,
                format!(
                    "At most 6 sections combine; not applied: {}",
                    s.dropped.join(", ")
                ),
            );
        }
        for (k, e) in &s.bad {
            ui.colored_label(
                ui.visuals().error_fg_color,
                format!("Section record {} cannot be read: {e}", k.id),
            );
        }
    }
}

fn create_panel() -> Box<dyn Panel> {
    Box::new(StatsPanel)
}

/// Registers the layer, the browser folder and the panel (commands: `inspect::register`).
pub(crate) fn register(r: &mut crate::module::Registrar<'_>) {
    r.doc_layer(DocLayerDef {
        id: SECTION_LAYER,
        create: create_layer,
    });
    r.browser_provider(Box::new(AnalysisNodes));
    r.panel(PanelDef {
        id: SECTION_PANEL,
        title: "Section Statistics",
        icon: crate::module::IconId::NONE,
        slot: DockSlot::Right,
        workspaces: Some(vec![DESIGN]),
        create: create_panel,
    });
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::float_cmp, reason = "tests")]

    use super::*;

    #[test]
    fn row_keys_round_trip() {
        let k = ExtKey {
            kind: SECTION_KIND.to_owned(),
            id: 42,
        };
        assert_eq!(row_key(&k).as_str(), "inspect.section/42");
        assert_eq!(key_of_row(&row_key(&k)), Some(k));
        assert_eq!(key_of_row(&ANALYSIS_FOLDER), None);
    }

    #[test]
    fn hatch_and_colours_map_to_rnd06() {
        let mut h = HatchStyle::default();
        assert_eq!(render_hatch(&h), amg_render::Hatch::NONE);
        h.pattern = HatchPattern::Cross;
        let r = render_hatch(&h);
        assert!(r.cross && r.color[3] > 0.0);
        assert_eq!(r.spacing_px, 8.0);
        let c = linear_rgb(amg_mesh::Rgba8([255, 0, 0, 255]));
        assert!((c[0] - 1.0).abs() < 1e-6 && c[1] == 0.0 && c[2] == 0.0);
    }

    #[test]
    fn stats_text() {
        let s = AnalysisStats {
            key: None,
            name: "S".into(),
            area: 100.0,
            perimeter: 40.0,
            loops: 1,
            open_loops: 2,
            extent: Aabb2::new(
                amg_core::Point2::new(0.0, 0.0),
                amg_core::Point2::new(10.0, 10.0),
            ),
            bodies: 1,
        };
        let rows = stats_rows(&s);
        assert_eq!(rows[0], ("Section area", "100 mm²".to_owned()));
        assert_eq!(rows[3], ("Extent", "10 × 10 mm".to_owned()));
        assert_eq!(
            open_warning(&s).unwrap(),
            "body is not closed: 2 open section curves"
        );
    }
}
