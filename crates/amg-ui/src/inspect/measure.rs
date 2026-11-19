//! [`MeasureTool`]: the hand-built Phase 1 Measure dialog (S05 §2, INS-08) on the lane-D
//! framework: UI-03 tool and router, UI-04 picking, UI-10 jobs, UI-15 widgets.
//!
//! - **Selection.** A primary click picks the mesh face under the cursor (RND-04 / the CPU pick
//!   service). The pick is snapped and classified by [`super::snap`] under the dialog's filter
//!   (Body / Face / Edge / Vertex / Point) and fills *Selection 1*, then *Selection 2*; a third
//!   click starts over. Clicking a selection field makes the next click replace it. With
//!   *Smart selection* a face grows to its coplanar region and an edge to its feature chain (a
//!   closed chain gets a circle fit: hole diameters in one click); Alt-click keeps the single
//!   triangle or edge. Picks preselected before the tool started are used as Selection 1 and 2.
//! - **3-point circle** mode takes three snapped points and shows their circumcircle.
//! - **Results** are computed by INS-01 (`amg_features::inspect::measure`). Only work proven
//!   cheap runs inline (`estimated_cost == Inline`, no smart growth); everything else is an
//!   `Interactive` job, and a newer selection cancels the running job and starts a new one
//!   (supersede). The table shows [`report_rows`] in the chosen unit (default: the document
//!   display unit) and precision (default: `FormatSpec::for_unit`), each with a copy button.
//! - **Overlay** (RND-07, [`super::overlay`]): witness line with the distance, Δ legs, angle
//!   arc, fitted circles, pick markers and the hover snap. *Pin* keeps the current overlay until
//!   the tool closes.
//! - Measure is transient: no document command, nothing saved. Esc or OK closes it.
//!
//! Tests and scripts read the tool's state through [`measure_snapshot`].

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Instant;

use amg_core::{FormatSpec, LengthUnit, Point3, Tolerance, Unit};
use amg_doc::{BodyGeom, BodyId, EntityRef, Evaluated, TopoKind};
use amg_features::inspect::InspectError;
use amg_features::inspect::measure::{
    CircleFit, Cost, DisplayPrefs, EntityMeasure, Exactness, MeasureOpts, MeasureReport,
    MeasureRow, Operand, OperandKind, circle_through, estimated_cost, measure, report_rows,
};
use amg_jobs::{JobCtx, JobError, JobHandle, JobId, SpawnOptions};
use amg_mesh::{FaceId, IndexedMesh};

use super::MEASURE_CMD;
use super::overlay::MeasureOverlay;
use super::snap::{self, Choice, MeasureFilter, MeshPick, Resolved};
use crate::app::{CmdEnv, CommitBatch};
use crate::command::CmdId;
use crate::input::ViewportEvent;
use crate::pick::{PickHitUi, PickMode, PickTarget, PickTicket};
use crate::scene::HighlightLayer;
use crate::select::{EntityFilter, SelItem};
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};
use crate::widgets::{ChoiceField, DialogAction, DialogFrame, SelectionField, toggle};

/// Label of the measure job (jobs panel).
pub const MEASURE_JOB: &str = "Measure";
/// Largest precision (decimals).
pub const MAX_PRECISION: u8 = 6;

/// Length units offered by the dialog.
const LENGTH_UNITS: [(LengthUnit, &str); 6] = [
    (LengthUnit::Millimetre, "mm"),
    (LengthUnit::Centimetre, "cm"),
    (LengthUnit::Metre, "m"),
    (LengthUnit::Inch, "in"),
    (LengthUnit::Foot, "ft"),
    (LengthUnit::Micrometre, "µm"),
];
/// Angle units offered by the dialog.
const ANGLE_UNITS: [(Unit, &str); 2] = [(Unit::Degree, "deg"), (Unit::Radian, "rad")];

/// The mesh of `body`, if it is a mesh body.
fn mesh_of(ev: &Evaluated, body: BodyId) -> Option<Arc<IndexedMesh>> {
    match &ev.body(body)?.geom {
        BodyGeom::Mesh(m) => Some(Arc::clone(m)),
        _ => None,
    }
}

/// One selection: picked (resolution pending) or resolved.
#[derive(Clone, Debug)]
enum Slot {
    Pending(MeshPick),
    Ready(Resolved),
}

impl Slot {
    fn at(&self) -> Point3 {
        match self {
            Slot::Pending(p) => p.at,
            Slot::Ready(r) => r.at,
        }
    }

    /// The display target (for the selection field and the highlight).
    fn target(&self) -> Option<PickTarget> {
        let (body, instance, face, whole) = match self {
            Slot::Pending(p) => (p.body, p.instance, p.face, matches!(p.choice, Choice::Body)),
            Slot::Ready(r) => {
                let (body, instance) = r.body?;
                let whole = r.kind() == OperandKind::Body;
                let face = r.faces.as_ref().and_then(|f| f.first().copied());
                (body, instance, face.unwrap_or(FaceId(0)), whole)
            }
        };
        Some(PickTarget::Body {
            body,
            instance,
            sub: (!whole).then_some((TopoKind::Face, face.0)),
        })
    }

    fn item(&self) -> Option<SelItem> {
        let target = self.target()?;
        Some(SelItem {
            target,
            entity: EntityRef::Body(target.body()?),
        })
    }

    /// A short description for the dialog.
    fn describe(&self) -> String {
        match self {
            Slot::Pending(p) => format!("{} (resolving…)", choice_name(&p.choice)),
            Slot::Ready(r) => {
                let kind = r.kind().name();
                match (&r.operand, &r.faces) {
                    (_, Some(f)) if f.len() > 1 => format!("{kind} ({} triangles)", f.len()),
                    (Operand::Mesh(m), _) => match &m.element {
                        amg_features::inspect::measure::MeshElement::Polyline { verts, closed } => {
                            format!(
                                "{kind} ({} edges{})",
                                verts.len() - usize::from(!*closed),
                                if *closed { ", closed" } else { "" }
                            )
                        }
                        _ => kind.to_owned(),
                    },
                    _ => kind.to_owned(),
                }
            }
        }
    }
}

fn choice_name(c: &Choice) -> &'static str {
    match c {
        Choice::Vertex(_) => "Vertex",
        Choice::Edge { .. } => "Edge",
        Choice::Face => "Face",
        Choice::Point(_) => "Point",
        Choice::Body => "Body",
    }
}

/// What the job returns.
#[derive(Debug)]
struct JobOut {
    resolved: Vec<Resolved>,
    report: MeasureReport,
}

/// The running measurement.
struct Running {
    handle: JobHandle<JobOut, InspectError>,
    id: JobId,
    generation: u64,
    /// Selection indices the job resolves, in operand order.
    slots: Vec<usize>,
}

/// A click waiting for its pick.
#[derive(Clone, Copy, Debug)]
struct PendingClick {
    ticket: PickTicket,
    ray: amg_core::Ray3,
    single: bool,
}

/// A finished measurement.
#[derive(Clone, Debug)]
struct Outcome {
    report: MeasureReport,
    inline: bool,
}

/// The tool's state as tests and scripts see it ([`measure_snapshot`]).
#[derive(Clone, Debug, Default)]
pub struct MeasureSnapshot {
    /// Bumped by every selection change (each one supersedes the previous measurement).
    pub generation: u64,
    /// Kind of each selection (`None` while it resolves), in operand order.
    pub kinds: Vec<Option<OperandKind>>,
    /// The resolved operands, in operand order (only once all are resolved).
    pub operands: Vec<Operand>,
    /// The current result.
    pub report: Option<MeasureReport>,
    /// Its displayed rows (chosen unit and precision).
    pub rows: Vec<MeasureRow>,
    /// The display settings.
    pub prefs: DisplayPrefs,
    /// The running measure job.
    pub running: Option<JobId>,
    /// Jobs cancelled because a newer selection superseded them, with the time of the
    /// cancel, oldest first.
    pub superseded: Vec<(JobId, Instant)>,
    /// The result was computed inline (on the UI thread, proven cheap).
    pub inline: bool,
    /// The last error.
    pub error: Option<String>,
    /// Pinned overlays.
    pub pinned: usize,
    /// 3-point circle mode.
    pub three_point: bool,
    /// The selection filter.
    pub filter: MeasureFilter,
    /// Smart selection.
    pub smart: bool,
}

fn snapshot_id() -> egui::Id {
    egui::Id::new("amg_ui.inspect.measure_snapshot")
}

/// The Measure tool's state as of its last dialog draw (`None` before the first one). Read by
/// tests and scripts; stale once the tool closed (check `AmalgamApp::active_tool`).
pub fn measure_snapshot(ctx: &egui::Context) -> Option<MeasureSnapshot> {
    ctx.data(|d| d.get_temp(snapshot_id()))
}

/// The Measure tool (see the module docs).
pub struct MeasureTool {
    filter: MeasureFilter,
    smart: bool,
    three_point: bool,
    prefs: DisplayPrefs,
    /// Selection 1 and 2.
    picks: [Option<Slot>; 2],
    /// 3-point circle mode points (resolved, ≤ 3).
    points: Vec<Resolved>,
    /// The selection the next click fills.
    active: usize,
    /// The next click into Selection 1 clears Selection 2 (after both were filled).
    restart_next: bool,
    field_active: [bool; 2],
    points_active: bool,
    /// Targets preselected when the tool started (applied in `start`).
    preselected: Vec<PickTarget>,
    click: Option<PendingClick>,
    generation: u64,
    job: Option<Running>,
    outcome: Option<Outcome>,
    rows: Vec<MeasureRow>,
    error: Option<String>,
    superseded: Vec<(JobId, Instant)>,
    pinned: Vec<MeasureOverlay>,
    hover: Option<(Choice, Point3)>,
    /// What the Preview highlight layer shows: `(generation, resolved count)` it was built for.
    highlight_key: Option<(u64, usize)>,
    highlighted: bool,
}

impl std::fmt::Debug for MeasureTool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MeasureTool")
            .field("filter", &self.filter)
            .field("generation", &self.generation)
            .field("running", &self.job.as_ref().map(|j| j.id))
            .finish_non_exhaustive()
    }
}

impl MeasureTool {
    /// The tool, with the document's display unit and up to two preselected mesh bodies or
    /// mesh faces as Selection 1 and 2.
    pub fn new(init: &ToolInit<'_>) -> MeasureTool {
        let length = init.env.design.meta().default_units.length;
        let ev = init.env.evaluated;
        let preselected = init
            .preselection
            .iter()
            .map(|i| i.target)
            .filter(|t| match t {
                PickTarget::Body { body, sub, .. } => {
                    matches!(sub, None | Some((TopoKind::Face, _))) && mesh_of(ev, *body).is_some()
                }
                _ => false,
            })
            .take(2)
            .collect();
        MeasureTool {
            filter: MeasureFilter::default(),
            smart: true,
            three_point: false,
            prefs: DisplayPrefs {
                length,
                angle: Unit::Degree,
                precision: FormatSpec::for_unit(Unit::from(length)).precision,
            },
            picks: [None, None],
            points: Vec::new(),
            active: 0,
            restart_next: false,
            field_active: [true, false],
            points_active: true,
            preselected,
            click: None,
            generation: 0,
            job: None,
            outcome: None,
            rows: Vec::new(),
            error: None,
            superseded: Vec::new(),
            pinned: Vec::new(),
            hover: None,
            highlight_key: None,
            highlighted: false,
        }
    }

    /// The effective filter (points only in 3-point mode).
    fn effective_filter(&self) -> MeasureFilter {
        if self.three_point {
            MeasureFilter::POINTS
        } else {
            self.filter
        }
    }

    /// The pick of a preselected target (faces grow smartly; no snap without a ray).
    fn preselected_pick(&self, t: PickTarget, cx: &ToolCx<'_>) -> Option<MeshPick> {
        let PickTarget::Body {
            body,
            instance,
            sub,
        } = t
        else {
            return None;
        };
        let mesh = mesh_of(cx.env().evaluated, body)?;
        let to_world = cx.scene().instance_world(body, instance)?;
        let (face, choice, local) = match sub {
            Some((TopoKind::Face, f)) if (f as usize) < mesh.face_count() => {
                let c = mesh.triangle(FaceId(f));
                (FaceId(f), Choice::Face, (c[0] + c[1] + c[2]) / 3.0)
            }
            None => (FaceId(0), Choice::Body, mesh.aabb().center()),
            _ => return None,
        };
        Some(MeshPick {
            body,
            instance,
            at: to_world.transform_point3(local),
            mesh,
            to_world,
            face,
            choice,
            smart: self.smart,
        })
    }

    /// A finished pick on the viewport: snap, classify, fill a selection, remeasure.
    fn on_hits(&mut self, hits: Vec<PickHitUi>, click: PendingClick, cx: &mut ToolCx<'_>) {
        let filter = self.effective_filter();
        let found = hits.into_iter().find_map(|h| {
            let PickTarget::Body {
                body,
                instance,
                sub: Some((TopoKind::Face, f)),
            } = h.target
            else {
                return None;
            };
            let mesh = mesh_of(cx.env().evaluated, body)?;
            let to_world = cx.scene().instance_world(body, instance)?;
            let mm = cx.camera().map_or(0.0, |v| snap::mm_per_px(v, h.world));
            let (choice, at) =
                snap::snap_pick(&mesh, &to_world, FaceId(f), &click.ray, mm, filter)?;
            Some(MeshPick {
                body,
                instance,
                mesh,
                to_world,
                face: FaceId(f),
                at,
                choice,
                smart: self.smart && !click.single,
            })
        });
        let Some(pick) = found else {
            return;
        };
        if self.three_point {
            // Points resolve in O(1).
            let Ok(r) = snap::resolve(&pick, &JobCtx::detached()) else {
                return;
            };
            if self.points.len() >= 3 {
                self.points.clear();
            }
            self.points.push(r);
        } else {
            self.place(Slot::Pending(pick));
        }
        self.remeasure(cx);
    }

    /// Puts a pick into the active selection and moves the focus on.
    fn place(&mut self, slot: Slot) {
        let i = self.active.min(1);
        if i == 0 && self.restart_next {
            self.picks[1] = None;
        }
        let other_empty = self.picks[1 - i].is_none();
        self.picks[i] = Some(slot);
        self.restart_next = false;
        if i == 0 && other_empty {
            self.focus(1);
        } else if i == 1 {
            self.focus(0);
            self.restart_next = true;
        }
    }

    fn focus(&mut self, i: usize) {
        self.active = i;
        self.field_active = [i == 0, i == 1];
    }

    /// Clears every selection and result (Restart).
    fn restart(&mut self, cx: &mut ToolCx<'_>) {
        self.picks = [None, None];
        self.points.clear();
        self.restart_next = false;
        self.focus(0);
        self.points_active = true;
        self.remeasure(cx);
    }

    /// The selection changed: supersede the running job and measure again (inline when
    /// proven cheap, else in a job).
    fn remeasure(&mut self, cx: &mut ToolCx<'_>) {
        self.generation += 1;
        if let Some(j) = self.job.take() {
            j.handle.cancel();
            tracing::debug!(job = j.id.0, "measure superseded");
            self.superseded.push((j.id, Instant::now()));
        }
        self.outcome = None;
        self.error = None;
        cx.app().request_repaint();
        if self.three_point {
            self.measure_circle();
            return;
        }
        let idx: Vec<usize> = (0..2).filter(|&i| self.picks[i].is_some()).collect();
        if idx.is_empty() {
            return;
        }
        let cheap = idx.iter().all(|&i| match &self.picks[i] {
            Some(Slot::Pending(p)) => !snap::needs_job(p),
            _ => true,
        });
        if cheap {
            let ctx = JobCtx::detached();
            for &i in &idx {
                if let Some(Slot::Pending(p)) = &self.picks[i] {
                    match snap::resolve(p, &ctx) {
                        Ok(r) => self.picks[i] = Some(Slot::Ready(r)),
                        Err(e) => {
                            self.error = Some(e.to_string());
                            return;
                        }
                    }
                }
            }
            let ops: Vec<&Operand> = idx
                .iter()
                .filter_map(|&i| match &self.picks[i] {
                    Some(Slot::Ready(r)) => Some(&r.operand),
                    _ => None,
                })
                .collect();
            if estimated_cost(ops[0], ops.get(1).copied()) == Cost::Inline {
                match measure(ops[0], ops.get(1).copied(), &MeasureOpts::default(), &ctx) {
                    Ok(report) => {
                        self.outcome = Some(Outcome {
                            report,
                            inline: true,
                        });
                    }
                    Err(e) => self.error = Some(e.to_string()),
                }
                return;
            }
        }
        let input: Vec<Slot> = idx.iter().filter_map(|&i| self.picks[i].clone()).collect();
        let handle = cx.app().jobs().spawn_fn(
            MEASURE_JOB,
            SpawnOptions::default(),
            move |ctx: &JobCtx| -> Result<JobOut, InspectError> {
                let mut resolved = Vec::with_capacity(input.len());
                for s in &input {
                    resolved.push(match s {
                        Slot::Ready(r) => r.clone(),
                        Slot::Pending(p) => snap::resolve(p, ctx)?,
                    });
                }
                ctx.checkpoint()?;
                let report = measure(
                    &resolved[0].operand,
                    resolved.get(1).map(|r| &r.operand),
                    &MeasureOpts::default(),
                    ctx,
                )?;
                Ok(JobOut { resolved, report })
            },
        );
        tracing::debug!(job = handle.id().0, "measure job started");
        self.job = Some(Running {
            id: handle.id(),
            handle,
            generation: self.generation,
            slots: idx,
        });
    }

    /// 3-point circle: the circumcircle of three points (inline, O(1)).
    fn measure_circle(&mut self) {
        let [a, b, c] = match self.points.as_slice() {
            [a, b, c] => [a.at, b.at, c.at],
            _ => return,
        };
        match circle_through(a, b, c, &Tolerance::mesh()) {
            Ok(circle) => {
                let e = EntityMeasure {
                    kind: OperandKind::Polyline,
                    position: Some(circle.center),
                    length: None,
                    area: None,
                    volume: None,
                    centroid: Some(circle.center),
                    plane: None,
                    direction: None,
                    circle: Some(CircleFit {
                        circle,
                        rms: 0.0,
                        max_residual: 0.0,
                    }),
                    radius: None,
                    bbox: None,
                    face_count: None,
                    exactness: Exactness::Exact,
                };
                self.outcome = Some(Outcome {
                    report: MeasureReport {
                        first: e,
                        second: None,
                        pair: None,
                    },
                    inline: true,
                });
            }
            Err(e) => self.error = Some(e.to_string()),
        }
    }

    /// Collects a finished job (only the newest generation counts).
    fn poll_job(&mut self) {
        let Some(j) = &mut self.job else {
            return;
        };
        let Some(result) = j.handle.try_take() else {
            return;
        };
        let Some(j) = self.job.take() else {
            return;
        };
        if j.generation != self.generation {
            return;
        }
        match result {
            Ok(out) => {
                for (&i, r) in j.slots.iter().zip(out.resolved) {
                    self.picks[i] = Some(Slot::Ready(r));
                }
                self.outcome = Some(Outcome {
                    report: out.report,
                    inline: false,
                });
            }
            Err(JobError::Cancelled) => {}
            Err(e) => self.error = Some(e.to_string()),
        }
    }

    /// The selections in operand order.
    fn slots(&self) -> impl Iterator<Item = &Slot> {
        self.picks.iter().flatten()
    }

    /// What the current measurement draws (without pins and hover).
    fn current_overlay(&self) -> MeasureOverlay {
        let mut o = self
            .outcome
            .as_ref()
            .map(|r| MeasureOverlay::of_report(&r.report, &self.rows))
            .unwrap_or_default();
        if self.three_point {
            o.markers.extend(self.points.iter().map(|p| p.at));
        } else {
            for s in self.slots() {
                o.markers.push(s.at());
                if let Slot::Ready(r) = s
                    && r.chain.len() >= 2
                {
                    o.chains.push(r.chain.clone());
                }
            }
        }
        o
    }

    /// The Preview highlight layer: picked faces and regions, whole picked bodies.
    fn sync_highlight(&mut self, cx: &mut ToolCx<'_>) {
        let ready = self.slots().filter(|s| matches!(s, Slot::Ready(_))).count();
        let key = (self.generation, ready);
        if self.highlight_key == Some(key) {
            return;
        }
        self.highlight_key = Some(key);
        let mut targets = Vec::new();
        if !self.three_point {
            for s in self.picks.iter().flatten() {
                match s {
                    Slot::Ready(r) => match (&r.faces, r.body) {
                        (Some(faces), Some((body, instance))) => {
                            targets.extend(faces.iter().map(|f| PickTarget::Body {
                                body,
                                instance,
                                sub: Some((TopoKind::Face, f.0)),
                            }));
                        }
                        (None, Some((body, instance))) if r.kind() == OperandKind::Body => {
                            targets.push(PickTarget::Body {
                                body,
                                instance,
                                sub: None,
                            });
                        }
                        _ => {}
                    },
                    Slot::Pending(_) => targets.extend(s.target()),
                }
            }
        }
        if targets.is_empty() && !self.highlighted {
            return;
        }
        self.highlighted = !targets.is_empty();
        cx.app().set_highlight(HighlightLayer::Preview, targets);
    }

    fn snapshot(&self) -> MeasureSnapshot {
        let kinds = if self.three_point {
            self.points.iter().map(|p| Some(p.kind())).collect()
        } else {
            self.slots()
                .map(|s| match s {
                    Slot::Ready(r) => Some(r.kind()),
                    Slot::Pending(_) => None,
                })
                .collect()
        };
        let operands: Vec<Operand> = if self.three_point {
            self.points.iter().map(|p| p.operand.clone()).collect()
        } else {
            self.slots()
                .map_while(|s| match s {
                    Slot::Ready(r) => Some(r.operand.clone()),
                    Slot::Pending(_) => None,
                })
                .collect()
        };
        MeasureSnapshot {
            generation: self.generation,
            kinds,
            operands,
            report: self.outcome.as_ref().map(|o| o.report.clone()),
            rows: self.rows.clone(),
            prefs: self.prefs,
            running: self.job.as_ref().map(|j| j.id),
            superseded: self.superseded.clone(),
            inline: self.outcome.as_ref().is_some_and(|o| o.inline),
            error: self.error.clone(),
            pinned: self.pinned.len(),
            three_point: self.three_point,
            filter: self.filter,
            smart: self.smart,
        }
    }

    /// The options part of the dialog: filter, smart selection, units and precision (a change
    /// applies to the next pick; units and precision re-format the current result).
    fn options_ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label("Filter:");
            let f = &mut self.filter;
            ui.add_enabled_ui(!self.three_point, |ui| {
                toggle(ui, "Body", &mut f.body);
                toggle(ui, "Face", &mut f.face);
                toggle(ui, "Edge", &mut f.edge);
                toggle(ui, "Vertex", &mut f.vertex);
                toggle(ui, "Point", &mut f.point);
            });
        });
        toggle(ui, "Smart selection", &mut self.smart)
            .on_hover_text("Faces grow to their planar region, edges to their feature chain (Alt-click: single triangle or edge)");
        ChoiceField::new("Units", &mut self.prefs.length, &LENGTH_UNITS).show(ui);
        ChoiceField::new("Angle unit", &mut self.prefs.angle, &ANGLE_UNITS).show(ui);
        ui.horizontal(|ui| {
            ui.label(format!("Precision: {}", self.prefs.precision));
            let fewer = ui
                .add_enabled(self.prefs.precision > 0, egui::Button::new("−").small())
                .on_hover_text("Fewer decimals");
            fewer.widget_info(|| {
                egui::WidgetInfo::labeled(
                    egui::WidgetType::Button,
                    self.prefs.precision > 0,
                    "Fewer decimals",
                )
            });
            if fewer.clicked() {
                self.prefs.precision = self.prefs.precision.saturating_sub(1);
            }
            let more_ok = self.prefs.precision < MAX_PRECISION;
            let more = ui
                .add_enabled(more_ok, egui::Button::new("+").small())
                .on_hover_text("More decimals");
            more.widget_info(|| {
                egui::WidgetInfo::labeled(egui::WidgetType::Button, more_ok, "More decimals")
            });
            if more.clicked() {
                self.prefs.precision = (self.prefs.precision + 1).min(MAX_PRECISION);
            }
        });
    }

    /// The selection fields; true if a selection was cleared.
    fn selection_ui(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = false;
        if self.three_point {
            let mut items: Vec<SelItem> = self
                .points
                .iter()
                .filter_map(|p| {
                    let (body, instance) = p.body?;
                    Some(SelItem {
                        target: PickTarget::Body {
                            body,
                            instance,
                            sub: None,
                        },
                        entity: EntityRef::Body(body),
                    })
                })
                .collect();
            let before = items.len();
            SelectionField::new(
                "Points",
                &mut items,
                EntityFilter::FACE,
                &mut self.points_active,
            )
            .limits(3, Some(3))
            .show(ui);
            if items.len() < before {
                self.points.clear();
                changed = true;
            }
            return changed;
        }
        for (i, label) in ["Selection 1", "Selection 2"].into_iter().enumerate() {
            let mut items: Vec<SelItem> = self.picks[i].iter().filter_map(Slot::item).collect();
            let had = !items.is_empty();
            let was = self.field_active[i];
            SelectionField::new(
                label,
                &mut items,
                EntityFilter::FACE,
                &mut self.field_active[i],
            )
            .limits(u32::from(i == 0), Some(1))
            .show(ui);
            if self.field_active[i] && !was {
                // The user activated this field: the next click replaces it.
                self.active = i;
                self.restart_next = false;
            }
            if had && items.is_empty() {
                self.picks[i] = None;
                changed = true;
            }
            if let Some(s) = &self.picks[i] {
                ui.weak(format!("  {}", s.describe()));
            }
        }
        changed
    }

    fn results_ui(&mut self, ui: &mut egui::Ui) {
        if self.job.is_some() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("Measuring…");
            });
        }
        if let Some(e) = &self.error {
            ui.colored_label(ui.visuals().error_fg_color, e);
        }
        if self.outcome.is_none() && self.job.is_none() && self.error.is_none() {
            ui.weak(if self.three_point {
                "Pick three points on the model."
            } else {
                "Pick one or two entities in the viewport."
            });
            return;
        }
        let mut copy = None;
        egui::Grid::new("amg_measure_results")
            .num_columns(3)
            .striped(true)
            .show(ui, |ui| {
                for r in &self.rows {
                    ui.label(&r.label);
                    ui.label(&r.text);
                    let name = format!("Copy {}", r.label);
                    let b = ui.small_button("⧉").on_hover_text(name.as_str());
                    b.widget_info(|| {
                        egui::WidgetInfo::labeled(egui::WidgetType::Button, true, name.as_str())
                    });
                    if b.clicked() {
                        copy = Some(r.text.clone());
                    }
                    ui.end_row();
                }
            });
        if !self.rows.is_empty() && ui.button("Copy all").clicked() {
            copy = Some(
                self.rows
                    .iter()
                    .map(|r| format!("{}\t{}", r.label, r.text))
                    .collect::<Vec<_>>()
                    .join("\n"),
            );
        }
        if let Some(text) = copy {
            ui.ctx().copy_text(text);
        }
    }
}

impl Tool for MeasureTool {
    fn id(&self) -> CmdId {
        MEASURE_CMD
    }

    fn label(&self) -> Cow<'static, str> {
        "Measure".into()
    }

    fn start(&mut self, cx: &mut ToolCx<'_>) {
        let targets = std::mem::take(&mut self.preselected);
        let picks: Vec<MeshPick> = targets
            .into_iter()
            .filter_map(|t| self.preselected_pick(t, cx))
            .collect();
        if picks.is_empty() {
            return;
        }
        for p in picks {
            self.place(Slot::Pending(p));
        }
        self.remeasure(cx);
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        match ev {
            ViewportEvent::Click {
                button: egui::PointerButton::Primary,
                pos,
                ray,
                mods,
                ..
            } => {
                let ticket = cx.pick_point(*pos, PickMode::Nearest);
                self.click = Some(PendingClick {
                    ticket,
                    ray: *ray,
                    single: mods.alt,
                });
                InputResult::Consumed
            }
            _ => InputResult::Ignored,
        }
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        if let Some(click) = self.click
            && let Some(hits) = cx.poll_pick(click.ticket)
        {
            self.click = None;
            self.on_hits(hits, click, cx);
        }
        self.poll_job();
        self.rows = self
            .outcome
            .as_ref()
            .map(|o| report_rows(&o.report, &self.prefs))
            .unwrap_or_default();
        // The snap under the cursor (O(1)).
        let filter = self.effective_filter();
        self.hover = cx.hover().and_then(|h| {
            let PickTarget::Body {
                body,
                instance,
                sub: Some((TopoKind::Face, f)),
            } = h.item.target
            else {
                return None;
            };
            let view = cx.camera()?;
            let mesh = mesh_of(cx.env().evaluated, body)?;
            let to_world = cx.scene().instance_world(body, instance)?;
            let ray = view.ray(h.at);
            snap::snap_pick(
                &mesh,
                &to_world,
                FaceId(f),
                &ray,
                snap::mm_per_px(view, h.world),
                filter,
            )
        });
        self.sync_highlight(cx);
        if self.job.is_some() || self.click.is_some() {
            // Progress has no notifier; the job's finish has (repaint_notifier).
            cx.app()
                .request_repaint_after(std::time::Duration::from_millis(50));
        }
    }

    fn overlay(&mut self, out: &mut amg_render::OverlayList, cx: &ToolCx<'_>) {
        let Some(view) = cx.camera() else {
            return;
        };
        for p in &self.pinned {
            p.draw(out, view);
        }
        self.current_overlay().draw(out, view);
        if let Some((choice, at)) = &self.hover {
            let size_px = match choice {
                Choice::Vertex(_) => 11.0,
                Choice::Edge { .. } => 9.0,
                _ => 6.0,
            };
            out.points(
                &[*at],
                amg_render::PointStyle {
                    size_px,
                    color: [1.0, 1.0, 1.0, 0.9],
                    depth: amg_render::DepthMode::AlwaysOnTop,
                },
                None,
            );
        }
    }

    fn dialog_ui(&mut self, ui: &mut egui::Ui, cx: &mut ToolCx<'_>) -> bool {
        let was_three_point = self.three_point;
        let mut restart = false;
        let mut cleared = false;
        let action = DialogFrame::new("Measure").show_inside(ui, |ui| {
            self.options_ui(ui);
            toggle(ui, "3-point circle", &mut self.three_point);
            ui.separator();
            cleared = self.selection_ui(ui);
            ui.horizontal(|ui| {
                if ui.button("Restart").clicked() {
                    restart = true;
                }
                let can_pin = self.outcome.is_some();
                if ui
                    .add_enabled(can_pin, egui::Button::new("Pin"))
                    .on_hover_text("Keep this measurement in the viewport until Measure closes")
                    .clicked()
                {
                    let o = self.current_overlay();
                    if !o.is_empty() {
                        self.pinned.push(o);
                    }
                }
                if !self.pinned.is_empty() && ui.button("Clear pins").clicked() {
                    self.pinned.clear();
                }
            });
            ui.separator();
            self.results_ui(ui);
        });
        if restart || self.three_point != was_three_point {
            self.restart(cx);
        } else if cleared {
            if self.picks[0].is_none() && self.picks[1].is_some() {
                self.picks.swap(0, 1);
            }
            self.focus(usize::from(self.picks[0].is_some()));
            self.restart_next = false;
            self.remeasure(cx);
        }
        // Recompute the rows now so the snapshot reflects unit / precision changes.
        self.rows = self
            .outcome
            .as_ref()
            .map(|o| report_rows(&o.report, &self.prefs))
            .unwrap_or_default();
        let snap = self.snapshot();
        ui.ctx().data_mut(|d| d.insert_temp(snapshot_id(), snap));
        match action {
            DialogAction::Ok => cx.request_finish(FinishReason::Commit),
            DialogAction::Cancel => cx.request_finish(FinishReason::Cancel),
            DialogAction::None => {}
        }
        true
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::FACE
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        Some(if self.three_point {
            "Measure: pick three points for a circle; Esc closes".into()
        } else {
            "Measure: pick one or two entities (Alt: single triangle or edge); Esc closes".into()
        })
    }

    fn finish(&mut self, _reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        if let Some(j) = self.job.take() {
            j.handle.cancel();
        }
        if self.highlighted {
            cx.app().set_highlight(HighlightLayer::Preview, Vec::new());
        }
        None
    }
}

/// Enabled while the design has a mesh body (Phase 1 measures meshes; INS-07 adds B-rep).
pub(crate) fn enabled(env: &CmdEnv<'_>) -> crate::command::Enablement {
    if env
        .evaluated
        .bodies()
        .any(|(_, b)| matches!(b.geom, BodyGeom::Mesh(_)))
    {
        crate::command::Enablement::Enabled
    } else {
        crate::command::Enablement::disabled("The design has no mesh body")
    }
}
