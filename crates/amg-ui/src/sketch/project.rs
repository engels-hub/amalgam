//! Project, Include and Intersect (spec §13, UI-17) on the S03 SK-09 projected geometry.
//!
//! Each tool takes model geometry into the open sketch as locked entities with a
//! `ProjectionLink`, which every recompute refreshes from its source:
//!
//! | Tool | Picks | `ProjectionMode` |
//! |---|---|---|
//! | Project (`P`) | B-rep edges, faces (all their edges) and vertices, construction points | `Project` |
//! | Include | the same, lying in the sketch plane | `Include` |
//! | Intersect | edges, vertices and construction axes → points; faces and bodies → their section curves | `Intersect` / `IntersectionCurve` |
//!
//! A pick (or a model selection made before the tool started) is resolved to its persistent
//! reference (`SelItem`), then `amg_features::sketch::project_reference` computes its geometry in
//! a background job (OCCT projection and sections are not cheap; never on the UI thread). The
//! result becomes one `SketchEdit::AddProjection`: one undo entry per pick. The tool stays
//! active for the next pick; Esc leaves it.
//!
//! The projected entities are drawn in [`colors::LOCKED`](super::view::colors::LOCKED) and
//! cannot be dragged ([`SketchSession::begin_drag`](super::SketchSession::begin_drag)). After an
//! upstream edit the session copies the refreshed projections of the document's recompute into
//! its working copy and solves again, so the geometry constrained to them follows
//! ([`SketchSession`](super::SketchSession), *projections*).

use std::borrow::Cow;
use std::collections::VecDeque;
use std::sync::Arc;

use amg_core::Plane;
use amg_doc::{Design, EntityRef, Evaluated, FeatureId, TopoKind};
use amg_features::sketch::{
    ProjectionError, ProjectionLink, ProjectionMode, SketchEdit, project_reference,
};
use amg_jobs::{Cancelled, JobCtx, JobError, JobHandle, Priority, SpawnOptions};
use amg_sketch::{EntityId, ProjectedGeom};

use super::session::with_session;
use super::shapes::ToolDef;
use super::tools::Cached;
use crate::app::CommitBatch;
use crate::command::CmdId;
use crate::input::ViewportEvent;
use crate::module::IconId;
use crate::pick::{PickHitUi, PickMode, PickTarget, PickTicket};
use crate::select::{EntityFilter, SelItem};
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};

/// A UI-17 tool.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ProjectKind {
    /// Project edges, faces, vertices and construction points onto the sketch plane.
    Project,
    /// Include geometry that lies in the sketch plane.
    Include,
    /// Intersect edges, faces and bodies with the sketch plane.
    Intersect,
}

/// The Project icon (vendored by the UI-23 set).
const PROJECT_ICON: IconId = IconId("arrow-bar-to-down");
/// The Include icon.
const INCLUDE_ICON: IconId = IconId("layers-union");
/// The Intersect icon.
const INTERSECT_ICON: IconId = IconId("section");

impl ProjectKind {
    /// Every tool, in toolbar order (Fusion's Create ▸ Project / Include menu).
    pub const ALL: [ProjectKind; 3] = [
        ProjectKind::Project,
        ProjectKind::Include,
        ProjectKind::Intersect,
    ];

    /// The command data. Project has Fusion's `P`.
    pub fn def(self) -> &'static ToolDef {
        match self {
            ProjectKind::Project => &ToolDef {
                id: CmdId("sketch.project"),
                label: "Project",
                tooltip: "Project model edges, faces or vertices onto the sketch plane as \
                          locked geometry that follows its source",
                icon: PROJECT_ICON,
                keywords: &[
                    "sketch",
                    "project",
                    "projection",
                    "reference",
                    "edge",
                    "face",
                ],
                shortcut: Some("sketch:P"),
                pinned: false,
            },
            ProjectKind::Include => &ToolDef {
                id: CmdId("sketch.include"),
                label: "Include",
                tooltip: "Include model geometry that lies in the sketch plane as locked \
                          geometry that follows its source",
                icon: INCLUDE_ICON,
                keywords: &["sketch", "include", "reference", "edge", "in plane"],
                shortcut: None,
                pinned: false,
            },
            ProjectKind::Intersect => &ToolDef {
                id: CmdId("sketch.intersect"),
                label: "Intersect",
                tooltip: "Intersect edges, faces or bodies with the sketch plane: points for \
                          edges, section curves for faces and bodies",
                icon: INTERSECT_ICON,
                keywords: &["sketch", "intersect", "section", "cut", "slice", "body"],
                shortcut: None,
                pinned: false,
            },
        }
    }

    /// The tool of a command id.
    pub fn from_id(id: CmdId) -> Option<ProjectKind> {
        Self::ALL.into_iter().find(|k| k.def().id == id)
    }

    /// What the tool picks.
    pub fn filter(self) -> EntityFilter {
        let base = EntityFilter::FACE
            .union(EntityFilter::EDGE)
            .union(EntityFilter::VERTEX)
            .union(EntityFilter::CONSTRUCTION);
        match self {
            ProjectKind::Project | ProjectKind::Include => base,
            ProjectKind::Intersect => base.union(EntityFilter::BODY),
        }
    }

    /// The SK-09 mode for a picked target, or why the target does not apply.
    pub fn mode_for(self, target: &PickTarget) -> Result<ProjectionMode, &'static str> {
        let sub = match target {
            PickTarget::Body { sub, .. } => *sub,
            PickTarget::Construction(_) => None,
            PickTarget::Sketch(_) | PickTarget::Occurrence(_) => {
                return Err("pick an edge, face or vertex of a body");
            }
        };
        let construction = matches!(target, PickTarget::Construction(_));
        match self {
            ProjectKind::Project | ProjectKind::Include => {
                if sub.is_none() && !construction {
                    return Err("a whole body can't be projected; pick its faces or edges");
                }
                Ok(if self == ProjectKind::Project {
                    ProjectionMode::Project
                } else {
                    ProjectionMode::Include
                })
            }
            ProjectKind::Intersect => Ok(match sub {
                Some((TopoKind::Face | TopoKind::FaceGroup, _)) => {
                    ProjectionMode::IntersectionCurve
                }
                None if !construction => ProjectionMode::IntersectionCurve,
                _ => ProjectionMode::Intersect,
            }),
        }
    }
}

/// What the newest pick of a projection tool gave (tests and scripts).
#[derive(Clone, Debug, PartialEq)]
pub enum ProjectOutcome {
    /// The projection was added: its locked entities.
    Added(Vec<EntityId>),
    /// It was refused; the message says why (also the status hint and the session notice).
    Refused(String),
}

/// Why the projection job ended without geometry.
#[derive(Debug, thiserror::Error)]
enum ProjectJobError {
    #[error(transparent)]
    Projection(#[from] ProjectionError),
    #[error("cancelled")]
    Cancelled(#[from] Cancelled),
}

/// One running projection.
struct Running {
    handle: JobHandle<Vec<ProjectedGeom>, ProjectJobError>,
    source: EntityRef,
    mode: ProjectionMode,
}

/// The source of `item` with its selection key cleared (`AddProjection` gives it a new one).
fn plain_source(item: &SelItem) -> EntityRef {
    let mut source = item.entity.clone();
    if let EntityRef::Topo(t) = &mut source {
        t.key = 0;
    }
    source
}

/// Whether `link` already projects `source` in `mode` (the selection key does not count).
fn same_link(link: &ProjectionLink, source: &EntityRef, mode: ProjectionMode) -> bool {
    if link.mode != mode {
        return false;
    }
    match (&link.source, source) {
        (EntityRef::Topo(a), EntityRef::Topo(b)) => {
            a.body == b.body && a.kind == b.kind && a.name == b.name
        }
        (a, b) => a == b,
    }
}

/// The feature that made the model entity `source` (`None` for references without one).
fn source_feature(source: &EntityRef) -> Option<FeatureId> {
    match source {
        EntityRef::Body(b) => Some(b.origin),
        EntityRef::Topo(t) => Some(t.body.origin),
        EntityRef::Construction(c) => Some(c.0),
        EntityRef::InContext { target, .. } => source_feature(target),
        EntityRef::SketchCurve { sketch, .. }
        | EntityRef::SketchPoint { sketch, .. }
        | EntityRef::Profile { sketch, .. } => Some(sketch.0),
    }
}

/// The projection tool (Project, Include or Intersect).
pub struct ProjectTool {
    kind: ProjectKind,
    cache: Cached,
    /// Picks and preselected items waiting for their turn (one job at a time).
    queue: VecDeque<SelItem>,
    /// The click pick in flight.
    click: Option<PickTicket>,
    running: Option<Running>,
    /// A finished projection waiting until the session takes edits again.
    ready: Option<(EntityRef, ProjectionMode, Vec<ProjectedGeom>)>,
    /// Why the newest pick was refused (status hint).
    problem: Option<String>,
    /// The newest outcome (tests and scripts).
    last: Option<ProjectOutcome>,
    /// Projections added by this tool run.
    added: u32,
}

impl std::fmt::Debug for ProjectTool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ProjectTool")
            .field("kind", &self.kind)
            .field("queue", &self.queue.len())
            .field("running", &self.running.is_some())
            .field("last", &self.last)
            .finish_non_exhaustive()
    }
}

impl ProjectTool {
    /// A new tool; a model selection made before it started is projected first.
    pub fn new(kind: ProjectKind, init: &ToolInit<'_>) -> ProjectTool {
        ProjectTool {
            kind,
            cache: Cached::default(),
            queue: init.preselection.iter().cloned().collect(),
            click: None,
            running: None,
            ready: None,
            problem: None,
            last: None,
            added: 0,
        }
    }

    /// The tool kind.
    pub fn kind(&self) -> ProjectKind {
        self.kind
    }

    /// The newest outcome.
    pub fn last(&self) -> Option<&ProjectOutcome> {
        self.last.as_ref()
    }

    /// Projections added since the tool started.
    pub fn added(&self) -> u32 {
        self.added
    }

    /// A pick or projection is still in progress.
    pub fn is_working(&self) -> bool {
        self.click.is_some()
            || self.running.is_some()
            || self.ready.is_some()
            || !self.queue.is_empty()
    }

    fn refuse(&mut self, cx: &mut ToolCx<'_>, why: String) {
        tracing::debug!("projection refused: {why}");
        with_session(cx.app(), |s, _| s.notice = Some(why.clone()));
        self.problem = Some(why.clone());
        self.last = Some(ProjectOutcome::Refused(why));
        cx.app().request_repaint();
    }

    /// The first hit of a click pick, as a selectable item.
    fn on_hits(&mut self, cx: &mut ToolCx<'_>, hits: &[PickHitUi]) {
        let Some(item) = hits.iter().find_map(|h| cx.item_for(h.target)) else {
            return;
        };
        self.queue.push_back(item);
    }

    /// Starts the job of the next queued item (when none runs).
    fn start_next(&mut self, cx: &mut ToolCx<'_>) {
        while self.running.is_none() && self.ready.is_none() {
            let Some(item) = self.queue.pop_front() else {
                return;
            };
            match self.prepare(cx, &item) {
                Ok((design, eval, sketch, plane, source, mode)) => {
                    let job_source = source.clone();
                    let handle = cx.app().jobs().spawn_fn(
                        format!("{}: computing", self.kind.def().label),
                        SpawnOptions {
                            priority: Priority::Interactive,
                            hidden: true,
                        },
                        move |ctx: &JobCtx| -> Result<Vec<ProjectedGeom>, ProjectJobError> {
                            ctx.checkpoint()?;
                            Ok(project_reference(
                                &design,
                                &eval,
                                sketch,
                                &plane,
                                &job_source,
                                mode,
                            )?)
                        },
                    );
                    self.running = Some(Running {
                        handle,
                        source,
                        mode,
                    });
                }
                Err(why) => self.refuse(cx, why),
            }
        }
    }

    /// Checks a queued item and gathers what its projection job needs.
    #[allow(clippy::type_complexity, reason = "one private call site")]
    fn prepare(
        &self,
        cx: &mut ToolCx<'_>,
        item: &SelItem,
    ) -> Result<
        (
            Design,
            Arc<Evaluated>,
            FeatureId,
            Plane,
            EntityRef,
            ProjectionMode,
        ),
        String,
    > {
        let mode = self.kind.mode_for(&item.target).map_err(str::to_owned)?;
        let source = plain_source(item);
        let (sketch, plane, duplicate) = with_session(cx.app(), |s, _| {
            let dup = s
                .working()
                .links
                .values()
                .any(|l| same_link(l, &source, mode));
            (s.feature(), *s.plane(), dup)
        })
        .ok_or_else(|| "no sketch is open".to_owned())?;
        if duplicate {
            return Err("that geometry is already projected into this sketch".to_owned());
        }
        let env = cx.env();
        let timeline = env.design.timeline();
        let at = |f: FeatureId| timeline.node(f).map(|(i, _)| i);
        if let (Some(from), Some(here)) = (source_feature(&source).and_then(at), at(sketch))
            && from >= here
        {
            return Err("that geometry comes after the sketch in the timeline".to_owned());
        }
        Ok((
            env.design.clone(),
            Arc::clone(env.evaluated),
            sketch,
            plane,
            source,
            mode,
        ))
    }

    /// Collects the finished job and commits its projection (one undo entry).
    fn finish_job(&mut self, cx: &mut ToolCx<'_>) {
        if let Some(r) = &mut self.running
            && let Some(res) = r.handle.try_take()
            && let Some(r) = self.running.take()
        {
            match res {
                Ok(geometry) => self.ready = Some((r.source, r.mode, geometry)),
                Err(JobError::Cancelled | JobError::Failed(ProjectJobError::Cancelled(_))) => {}
                Err(JobError::Failed(ProjectJobError::Projection(e))) => {
                    self.refuse(cx, e.to_string());
                }
                Err(e @ JobError::Panicked(_)) => self.refuse(cx, e.to_string()),
            }
        }
        if self.ready.is_none() || self.cache.busy {
            return;
        }
        let Some((source, mode, geometry)) = self.ready.take() else {
            return;
        };
        let label = self.kind.def().label;
        let res = with_session(cx.app(), |s, app| {
            let edit = SketchEdit::AddProjection {
                source,
                mode,
                geometry,
            };
            let r = s.commit(app, label, edit, Vec::new());
            if r.is_ok() {
                s.notice = None;
            }
            r
        });
        match res {
            Some(Ok(r)) => {
                self.added += 1;
                self.problem = None;
                self.last = Some(ProjectOutcome::Added(r.created));
            }
            Some(Err(e)) => self.refuse(cx, format!("Not added: {e}")),
            None => {}
        }
    }

    fn step_hint(&self) -> &'static str {
        if self.is_working() {
            return "computing…";
        }
        match self.kind {
            ProjectKind::Project => "click edges, faces or vertices to project",
            ProjectKind::Include => "click edges, faces or vertices in the sketch plane",
            ProjectKind::Intersect => "click edges, faces or bodies to intersect",
        }
    }
}

impl Tool for ProjectTool {
    fn id(&self) -> CmdId {
        self.kind.def().id
    }

    fn label(&self) -> Cow<'static, str> {
        self.kind.def().label.into()
    }

    fn pick_filter(&self) -> EntityFilter {
        self.kind.filter()
    }

    fn start(&mut self, cx: &mut ToolCx<'_>) {
        self.cache.refresh(cx);
        with_session(cx.app(), |s, _| s.notice = None);
        self.start_next(cx);
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        let problem = self
            .problem
            .as_ref()
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
        if let Some(t) = self.click
            && let Some(hits) = cx.poll_pick(t)
        {
            self.click = None;
            self.on_hits(cx, &hits);
        }
        self.finish_job(cx);
        self.start_next(cx);
        if self.is_working() {
            cx.app().request_repaint();
        }
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        match ev {
            ViewportEvent::Hover { .. } => {
                with_session(cx.app(), |s, _| s.hovered = None);
                InputResult::Ignored
            }
            ViewportEvent::Click {
                button: egui::PointerButton::Primary,
                pos,
                ..
            } => {
                if self.click.is_none() {
                    self.problem = None;
                    self.click = Some(cx.pick_point(*pos, PickMode::Nearest));
                }
                InputResult::Consumed
            }
            ViewportEvent::Key {
                key: egui::Key::Escape,
                pressed: true,
                ..
            } => {
                cx.request_finish(FinishReason::Cancel);
                InputResult::Consumed
            }
            _ => InputResult::Ignored,
        }
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn finish(&mut self, _reason: FinishReason, _cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        if let Some(r) = self.running.take() {
            r.handle.cancel();
        }
        self.queue.clear();
        self.click = None;
        self.ready = None;
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_doc::{BodyId, ConstructionId};

    fn body(sub: Option<(TopoKind, u32)>) -> PickTarget {
        PickTarget::Body {
            body: BodyId {
                origin: FeatureId(3),
                slot: 0,
            },
            instance: 0,
            sub,
        }
    }

    #[test]
    fn commands_are_unique_and_found_by_id() {
        let mut ids = std::collections::BTreeSet::new();
        for k in ProjectKind::ALL {
            assert!(ids.insert(k.def().id.0), "{k:?}");
            assert_eq!(ProjectKind::from_id(k.def().id), Some(k));
        }
    }

    #[test]
    fn picks_map_to_projection_modes() {
        use ProjectionMode as M;
        let face = body(Some((TopoKind::Face, 1)));
        let edge = body(Some((TopoKind::Edge, 2)));
        let vertex = body(Some((TopoKind::Vertex, 0)));
        let whole = body(None);
        let cons = PickTarget::Construction(ConstructionId(FeatureId(5)));
        let p = ProjectKind::Project;
        assert_eq!(p.mode_for(&face), Ok(M::Project));
        assert_eq!(p.mode_for(&edge), Ok(M::Project));
        assert_eq!(p.mode_for(&vertex), Ok(M::Project));
        assert_eq!(p.mode_for(&cons), Ok(M::Project));
        assert!(p.mode_for(&whole).is_err());
        assert_eq!(ProjectKind::Include.mode_for(&edge), Ok(M::Include));
        assert!(ProjectKind::Include.mode_for(&whole).is_err());
        let i = ProjectKind::Intersect;
        assert_eq!(i.mode_for(&edge), Ok(M::Intersect));
        assert_eq!(i.mode_for(&vertex), Ok(M::Intersect));
        assert_eq!(i.mode_for(&cons), Ok(M::Intersect));
        assert_eq!(i.mode_for(&face), Ok(M::IntersectionCurve));
        assert_eq!(i.mode_for(&whole), Ok(M::IntersectionCurve));
        assert!(
            p.mode_for(&PickTarget::Sketch(amg_doc::SketchId(FeatureId(1))))
                .is_err()
        );
    }

    #[test]
    fn filters_follow_the_tool() {
        assert!(ProjectKind::Project.filter().contains(EntityFilter::EDGE));
        assert!(ProjectKind::Project.filter().contains(EntityFilter::FACE));
        assert!(!ProjectKind::Project.filter().contains(EntityFilter::BODY));
        assert!(ProjectKind::Intersect.filter().contains(EntityFilter::BODY));
        assert!(!ProjectKind::Include.filter().contains(EntityFilter::SKETCH));
    }
}
