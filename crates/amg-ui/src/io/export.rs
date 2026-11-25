//! The Export dialog (S12 §6, IO-15): format, selection, unit, 3MF flavour, the refinement
//! preset of solid bodies (IO-09), the validity report with "Repair first" (MESH-04) and
//! "Export anyway", progress and Cancel.
//!
//! Everything that touches geometry runs on a job: `scene_from_document`, the validity check
//! and the writer ([`amg_io::export`]). A 3MF with a non-watertight body comes back as
//! [`Outcome::Blocked`] with the per-object counts (the `Require` policy). "Export anyway"
//! writes the kept scene with `AllowInvalid`. "Repair first" adds one `mesh.repair` feature per
//! defective body (one undo entry), waits for the recompute and runs the export again.

use std::borrow::Cow;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::time::Duration;

use amg_core::units::LengthUnit;
use amg_doc::RecomputeStatus;
use amg_doc::{
    BodyGeom, BodyId, Command as DocCommand, Design, Evaluated, FeatureBox, FeatureId, Revision,
};
use amg_features::io::{ExportPrep, ExportRefinement, ExportSelection, scene_from_document};
use amg_features::mesh::MeshRepairFeature;
use amg_io::{
    ExportCheck, ExportOptions, ExportReport, Format, GeometryKind, IoError, Scene, Sink,
    SlicerFlavor, ValidityPolicy,
};
use amg_jobs::{JobCtx, JobHandle, SpawnOptions};

use super::import::{IoJobError, unit_name};
use super::prefs::{self, ExportPref};
use crate::app::{AppCx, CommitBatch};
use crate::command::Enablement;
use crate::dialog::AppDialog;
use crate::file::{FileDialogKind, FileDialogRequest, FileFilter};
use crate::notify::Notification;
use crate::pick::PickTarget;
use crate::widgets::{DialogAction, DialogFrame};

/// Repaint period while a job runs.
const PROGRESS_REFRESH: Duration = Duration::from_millis(100);

/// What an export job came back with.
#[derive(Debug)]
pub enum Outcome {
    /// The file was written.
    Written(ExportReport),
    /// The format requires valid solids and some objects are not: nothing was written.
    Blocked {
        /// The scene that was prepared (kept for "Export anyway").
        scene: Box<Scene>,
        /// Per-object validity.
        check: ExportCheck,
    },
}

enum Stage {
    Form,
    Picking(Receiver<Option<PathBuf>>),
    Working(JobHandle<Outcome, IoJobError>),
    Blocked {
        scene: Box<Scene>,
        check: ExportCheck,
        /// A repair already ran and the report is still not clean.
        repaired: bool,
    },
    Repairing {
        before: Revision,
    },
}

/// The Export dialog.
pub struct ExportDialog {
    stage: Stage,
    format: Format,
    selection_only: bool,
    unit: LengthUnit,
    flavor: SlicerFlavor,
    ascii: bool,
    binary: bool,
    mtl: bool,
    refinement: ExportRefinement,
    path: Option<PathBuf>,
    /// Repaired once already (a second block shows the report without "Repair first").
    repaired: bool,
    /// How often the document was prepared for export (tests).
    prepares: u32,
}

impl std::fmt::Debug for ExportDialog {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ExportDialog")
            .field("format", &self.format)
            .field("selection_only", &self.selection_only)
            .field("unit", &self.unit)
            .field("flavor", &self.flavor)
            .field("refinement", &self.refinement)
            .field("path", &self.path)
            .finish_non_exhaustive()
    }
}

/// The mesh formats this build can write.
pub fn export_formats() -> Vec<Format> {
    amg_io::registry()
        .iter()
        .filter(|f| f.can_export && f.kind == GeometryKind::Mesh)
        .map(|f| f.format)
        .collect()
}

/// The bodies of the selection.
fn selected_bodies(cx: &AppCx<'_>) -> Vec<BodyId> {
    let mut v: Vec<BodyId> = cx
        .selection()
        .items()
        .iter()
        .filter_map(|i| match i.target {
            PickTarget::Body { body, .. } => Some(body),
            _ => None,
        })
        .collect();
    v.sort();
    v.dedup();
    v
}

impl ExportDialog {
    /// A dialog with the remembered choices.
    pub fn new(cx: &AppCx<'_>) -> ExportDialog {
        let prefs = prefs::load();
        let formats = export_formats();
        let format = prefs
            .last_export_format
            .filter(|f| formats.contains(f))
            .or_else(|| formats.iter().copied().find(|f| *f == Format::ThreeMf))
            .or_else(|| formats.first().copied())
            .unwrap_or(Format::Stl);
        let mut d = ExportDialog {
            stage: Stage::Form,
            format,
            selection_only: false,
            unit: LengthUnit::Millimetre,
            flavor: SlicerFlavor::Generic,
            ascii: false,
            binary: true,
            mtl: true,
            refinement: ExportRefinement::Medium,
            path: None,
            repaired: false,
            prepares: 0,
        };
        d.load_pref(format);
        d.selection_only = d.selection_only && !selected_bodies(cx).is_empty();
        d
    }

    fn load_pref(&mut self, format: Format) {
        let p = prefs::load().export_for(format);
        self.format = format;
        self.unit = p.unit;
        self.flavor = p.flavor;
        self.selection_only = p.selection_only;
        self.ascii = p.ascii;
        self.binary = p.binary;
        self.mtl = p.mtl;
        self.refinement = p.refinement;
    }

    fn remember(&self) {
        let pref = ExportPref {
            unit: self.unit,
            flavor: self.flavor,
            selection_only: self.selection_only,
            ascii: self.ascii,
            binary: self.binary,
            mtl: self.mtl,
            refinement: self.refinement,
        };
        let format = self.format;
        let dir = self
            .path
            .as_deref()
            .and_then(Path::parent)
            .map(Path::to_owned);
        prefs::update(|p| {
            p.export.insert(format, pref);
            p.last_export_format = Some(format);
            if dir.is_some() {
                p.last_dir = dir;
            }
        });
    }

    /// The chosen format.
    pub fn format(&self) -> Format {
        self.format
    }

    /// Chooses the format and loads its remembered choices.
    pub fn set_format(&mut self, format: Format) {
        self.load_pref(format);
    }

    /// The chosen unit.
    pub fn unit(&self) -> LengthUnit {
        self.unit
    }

    /// Chooses the unit of the written numbers.
    pub fn set_unit(&mut self, unit: LengthUnit) {
        self.unit = unit;
    }

    /// The chosen 3MF flavour.
    pub fn flavor(&self) -> SlicerFlavor {
        self.flavor
    }

    /// Chooses the 3MF flavour.
    pub fn set_flavor(&mut self, flavor: SlicerFlavor) {
        self.flavor = flavor;
    }

    /// The tessellation preset of solid bodies.
    pub fn refinement(&self) -> ExportRefinement {
        self.refinement
    }

    /// Chooses the tessellation preset of solid bodies.
    pub fn set_refinement(&mut self, refinement: ExportRefinement) {
        self.refinement = refinement;
    }

    /// Export the selected bodies only.
    pub fn set_selection_only(&mut self, on: bool) {
        self.selection_only = on;
    }

    /// STL: ASCII instead of binary.
    pub fn set_ascii(&mut self, on: bool) {
        self.ascii = on;
    }

    /// The target file once chosen.
    pub fn path(&self) -> Option<&Path> {
        self.path.as_deref()
    }

    /// True while the validity report is shown.
    pub fn is_blocked(&self) -> bool {
        matches!(self.stage, Stage::Blocked { .. })
    }

    /// The validity report, while it is shown.
    pub fn report(&self) -> Option<&ExportCheck> {
        match &self.stage {
            Stage::Blocked { check, .. } => Some(check),
            _ => None,
        }
    }

    /// How often the design was prepared for export.
    pub fn prepares(&self) -> u32 {
        self.prepares
    }

    /// Starts the export to `path` (the Save dialog's answer). Tests call it directly.
    pub fn start(&mut self, cx: &mut AppCx<'_>, path: PathBuf) {
        self.path = Some(fix_extension(path, self.format));
        self.remember();
        self.start_write(cx, None, None);
    }

    fn options(&self, validity: Option<ValidityPolicy>) -> ExportOptions {
        let mut o = ExportOptions {
            unit: self.unit,
            validity,
            ..ExportOptions::default()
        };
        o.stl.ascii = self.ascii;
        o.ply.binary = self.binary;
        o.obj.mtl = self.mtl;
        o.threemf.flavor = self.flavor;
        o
    }

    fn selection(&self, cx: &AppCx<'_>) -> ExportSelection {
        let bodies = selected_bodies(cx);
        if self.selection_only && !bodies.is_empty() {
            ExportSelection::Bodies(bodies)
        } else {
            ExportSelection::AllVisible
        }
    }

    /// Spawns the export job. `scene`: a scene prepared before ("Export anyway"); `validity`:
    /// the policy override.
    fn start_write(
        &mut self,
        cx: &mut AppCx<'_>,
        scene: Option<Box<Scene>>,
        validity: Option<ValidityPolicy>,
    ) {
        let Some(path) = self.path.clone() else {
            return;
        };
        let doc = &cx.doc_tab().document;
        let design: Arc<Design> = doc.snapshot();
        let eval: Arc<Evaluated> = Arc::clone(doc.evaluated());
        let job = Input {
            design,
            eval,
            selection: self.selection(cx),
            scene,
            opts: self.options(validity),
            refinement: self.refinement,
            format: self.format,
            path: path.clone(),
        };
        if job.scene.is_none() {
            self.prepares += 1;
        }
        let name = path.file_name().map_or_else(
            || path.display().to_string(),
            |n| n.to_string_lossy().into_owned(),
        );
        let handle = cx.jobs().spawn_fn(
            format!("Export {name}"),
            SpawnOptions::default(),
            move |ctx| run(job, ctx),
        );
        self.stage = Stage::Working(handle);
    }

    fn start_picking(&mut self, cx: &mut AppCx<'_>) {
        let info = self.format.info();
        let stem = cx.doc_tab().name.clone();
        let ext = info.extensions.first().copied().unwrap_or("stl");
        let prefs = prefs::load();
        self.stage = Stage::Picking(cx.pick_file(FileDialogRequest {
            kind: FileDialogKind::Save,
            title: format!("Export {}", info.name),
            filters: vec![FileFilter {
                name: info.name.to_owned(),
                extensions: info.extensions.iter().map(|e| (*e).to_owned()).collect(),
            }],
            directory: prefs.last_dir,
            file_name: Some(format!("{stem}.{ext}")),
        }));
    }

    /// The body ids of the objects the report flags.
    fn defective_bodies(scene: &Scene, check: &ExportCheck) -> Vec<BodyId> {
        let mut v: Vec<BodyId> = check
            .objects
            .iter()
            .filter(|o| {
                o.boundary_edges > 0
                    || o.non_manifold_edges > 0
                    || o.inconsistent_orientation
                    || o.negative_volume
            })
            .filter_map(|o| scene.objects.get(o.object as usize)?.source_id.as_deref())
            .filter_map(parse_source_id)
            .collect();
        v.sort();
        v.dedup();
        v
    }

    /// The defective bodies "Repair first" can work on: mesh bodies (a solid is exported
    /// from its B-rep, which mesh repair cannot change).
    fn repairable_bodies(cx: &AppCx<'_>, scene: &Scene, check: &ExportCheck) -> Vec<BodyId> {
        let evaluated = cx.doc_tab().document.evaluated();
        let mut v = Self::defective_bodies(scene, check);
        v.retain(|id| {
            evaluated
                .body(*id)
                .is_some_and(|b| matches!(b.geom, BodyGeom::Mesh(_)))
        });
        v
    }

    /// "Repair first": one `mesh.repair` feature per defective body, as one undo entry.
    pub fn repair_first(&mut self, cx: &mut AppCx<'_>) {
        let Stage::Blocked { scene, check, .. } = &self.stage else {
            return;
        };
        let bodies = Self::repairable_bodies(cx, scene, check);
        if bodies.is_empty() {
            return;
        }
        let root = cx.doc_tab().document.design().root();
        let commands = bodies
            .into_iter()
            .map(|b| DocCommand::AddFeature {
                at: None,
                feature: FeatureBox::new(MeshRepairFeature::new(b)),
                context: root,
                name: None,
            })
            .collect();
        let before = cx.doc_tab().document.history().revision();
        cx.submit(CommitBatch {
            label: "Repair before export".to_owned(),
            commands,
        });
        self.repaired = true;
        self.stage = Stage::Repairing { before };
    }

    /// "Export anyway": writes the prepared scene with `ValidityPolicy::AllowInvalid`.
    pub fn export_anyway(&mut self, cx: &mut AppCx<'_>) {
        let stage = std::mem::replace(&mut self.stage, Stage::Form);
        if let Stage::Blocked { scene, .. } = stage {
            self.start_write(cx, Some(scene), Some(ValidityPolicy::AllowInvalid));
        } else {
            self.stage = stage;
        }
    }

    /// Cancels the running job.
    pub fn cancel(&self) {
        if let Stage::Working(h) = &self.stage {
            h.cancel();
        }
    }

    fn finish_ok(&self, cx: &mut AppCx<'_>, report: &ExportReport) {
        let path = self
            .path
            .as_deref()
            .map_or_else(String::new, |p| p.display().to_string());
        cx.notify(Notification::info(
            "Exported",
            format!(
                "{path}: {} triangles, {} bytes in {:.1} s",
                report.triangles,
                report.bytes,
                report.duration.as_secs_f32()
            ),
        ));
        if !report.warnings.is_empty() {
            cx.notify(Notification::warning(
                "Exported with warnings",
                report
                    .warnings
                    .iter()
                    .map(amg_io::IoWarning::message)
                    .collect::<Vec<_>>()
                    .join("\n"),
            ));
        }
    }

    /// Polls the stage. False: the dialog is finished.
    fn poll(&mut self, cx: &mut AppCx<'_>) -> bool {
        match &mut self.stage {
            Stage::Form | Stage::Blocked { .. } => {}
            Stage::Picking(rx) => match rx.try_recv() {
                Ok(Some(path)) => self.start(cx, path),
                Ok(None) | Err(TryRecvError::Disconnected) => self.stage = Stage::Form,
                Err(TryRecvError::Empty) => {}
            },
            Stage::Working(h) => {
                if let Some(result) = h.try_take() {
                    match result {
                        Ok(Outcome::Written(report)) => {
                            self.finish_ok(cx, &report);
                            return false;
                        }
                        Ok(Outcome::Blocked { scene, check }) => {
                            self.stage = Stage::Blocked {
                                scene,
                                check,
                                repaired: self.repaired,
                            };
                        }
                        Err(e) => {
                            cx.notify_error(&e);
                            // Back to the form: the choices can be changed and tried again.
                            self.stage = Stage::Form;
                            return !e.is_cancelled_job();
                        }
                    }
                }
            }
            Stage::Repairing { before } => {
                let doc = &cx.doc_tab().document;
                let now = doc.history().revision();
                if now != *before {
                    match doc.recompute_status() {
                        RecomputeStatus::Idle => {
                            let ok = doc.is_evaluated();
                            if ok {
                                self.start_write(cx, None, None);
                            }
                        }
                        RecomputeStatus::Failed(m) => {
                            cx.notify(Notification::error("Repair failed", m));
                            return false;
                        }
                        _ => {}
                    }
                }
            }
        }
        true
    }

    fn form_ui(&mut self, ui: &mut egui::Ui, cx: &AppCx<'_>) {
        let selected = selected_bodies(cx).len();
        let formats = export_formats();
        let mut format = self.format;
        ui.horizontal(|ui| {
            ui.label("Format");
            egui::ComboBox::from_id_salt("amg_io_export_format")
                .selected_text(self.format.info().name)
                .show_ui(ui, |ui| {
                    for f in &formats {
                        ui.selectable_value(&mut format, *f, f.info().name);
                    }
                });
        });
        if format != self.format {
            self.load_pref(format);
        }
        ui.horizontal(|ui| {
            ui.label("Selection");
            ui.selectable_value(&mut self.selection_only, false, "All visible bodies");
            ui.add_enabled_ui(selected > 0, |ui| {
                ui.selectable_value(
                    &mut self.selection_only,
                    true,
                    format!("Selected bodies ({selected})"),
                );
            });
        });
        ui.horizontal(|ui| {
            ui.label("Unit");
            egui::ComboBox::from_id_salt("amg_io_export_unit")
                .selected_text(unit_name(self.unit))
                .show_ui(ui, |ui| {
                    for u in LengthUnit::ALL {
                        ui.selectable_value(&mut self.unit, u, unit_name(u));
                    }
                });
        });
        match self.format {
            Format::ThreeMf => {
                ui.horizontal(|ui| {
                    ui.label("Slicer");
                    ui.selectable_value(&mut self.flavor, SlicerFlavor::Generic, "Generic 3MF");
                    ui.selectable_value(
                        &mut self.flavor,
                        SlicerFlavor::Orca,
                        "OrcaSlicer / Bambu Studio",
                    );
                    ui.add_enabled(false, egui::Button::new("PrusaSlicer (not available yet)"));
                });
            }
            Format::Stl => {
                ui.checkbox(&mut self.ascii, "ASCII (default: binary)");
            }
            Format::Ply => {
                ui.checkbox(&mut self.binary, "Binary (default)");
            }
            Format::Obj => {
                ui.checkbox(&mut self.mtl, "Write materials (.mtl)");
            }
            _ => {}
        }
        let has_solid = cx
            .doc_tab()
            .document
            .evaluated()
            .bodies()
            .any(|(_, b)| matches!(b.geom, BodyGeom::Solid(_)));
        if has_solid {
            refinement_ui(ui, &mut self.refinement);
        }
        if matches!(self.format, Format::ThreeMf) {
            ui.weak("A 3MF needs closed solids: a body with holes is reported before writing.");
        }
    }

    fn blocked_ui(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        let Stage::Blocked {
            scene,
            check,
            repaired,
        } = &self.stage
        else {
            return true;
        };
        let can_repair = !*repaired && !Self::repairable_bodies(cx, scene, check).is_empty();
        let mut action = None;
        egui::Window::new("Export — validity report")
            .id(egui::Id::new("amg_io_export_report"))
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                ui.colored_label(
                    ui.visuals().error_fg_color,
                    format!(
                        "{} is not a closed solid for {}:",
                        if check.objects.iter().filter(|o| is_bad(o)).count() == 1 {
                            "1 object"
                        } else {
                            "Some objects"
                        },
                        self.format.info().name
                    ),
                );
                for o in check.objects.iter().filter(|o| is_bad(o)) {
                    ui.label(format!(
                        "• {}: {}",
                        o.name.as_deref().unwrap_or("(unnamed)"),
                        describe(o)
                    ));
                }
                if *repaired {
                    ui.weak("The repair ran but could not fix everything.");
                }
                ui.separator();
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(can_repair, egui::Button::new("Repair first"))
                        .on_hover_text("Run Mesh ▸ Repair on the defective bodies, then export")
                        .clicked()
                    {
                        action = Some(BlockedAction::Repair);
                    }
                    if ui.button("Export anyway").clicked() {
                        action = Some(BlockedAction::Anyway);
                    }
                    if ui.button("Cancel").clicked() {
                        action = Some(BlockedAction::Cancel);
                    }
                });
            });
        match action {
            Some(BlockedAction::Repair) => self.repair_first(cx),
            Some(BlockedAction::Anyway) => self.export_anyway(cx),
            Some(BlockedAction::Cancel) => return false,
            None => {}
        }
        true
    }

    fn progress_ui(&self, ctx: &egui::Context, text: &str) {
        let (fraction, message) = match &self.stage {
            Stage::Working(h) => {
                let p = h.progress();
                (p.fraction, p.message)
            }
            _ => (0.0, None),
        };
        let mut cancel = false;
        egui::Window::new("Export")
            .id(egui::Id::new("amg_io_export_progress"))
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                ui.label(text);
                ui.add(
                    egui::ProgressBar::new(fraction.clamp(0.0, 1.0))
                        .desired_width(280.0)
                        .text(format!("{:.0} %", fraction * 100.0)),
                );
                if let Some(m) = message {
                    ui.weak(m.as_ref());
                }
                if ui.button("Cancel").clicked() {
                    cancel = true;
                }
            });
        ctx.request_repaint_after(PROGRESS_REFRESH);
        if cancel {
            self.cancel();
        }
    }
}

/// Short name of a preset.
fn refinement_name(r: ExportRefinement) -> &'static str {
    match r {
        ExportRefinement::Low => "Low",
        ExportRefinement::Medium => "Medium",
        ExportRefinement::High => "High",
        ExportRefinement::Custom { .. } => "Custom",
    }
}

/// The refinement preset of solid bodies; Custom shows its two deviations.
fn refinement_ui(ui: &mut egui::Ui, refinement: &mut ExportRefinement) {
    // Custom starts from the values of the preset it replaces.
    let custom = ExportRefinement::Custom {
        surface_deviation_mm: refinement.surface_deviation_mm(),
        normal_deviation_rad: refinement.normal_deviation_rad(),
    };
    ui.horizontal(|ui| {
        ui.label("Refinement");
        egui::ComboBox::from_id_salt("amg_io_export_refinement")
            .selected_text(refinement_name(*refinement))
            .show_ui(ui, |ui| {
                for r in [
                    ExportRefinement::Low,
                    ExportRefinement::Medium,
                    ExportRefinement::High,
                    custom,
                ] {
                    ui.selectable_value(refinement, r, refinement_name(r));
                }
            });
    });
    if let ExportRefinement::Custom {
        surface_deviation_mm,
        normal_deviation_rad,
    } = refinement
    {
        ui.horizontal(|ui| {
            ui.label("Surface deviation");
            ui.add(
                egui::DragValue::new(surface_deviation_mm)
                    .range(0.0005..=10.0)
                    .speed(0.001)
                    .max_decimals(4)
                    .suffix(" mm"),
            );
            ui.label("Normal deviation");
            let mut degrees = normal_deviation_rad.to_degrees();
            let angle = egui::DragValue::new(&mut degrees)
                .range(1.0..=90.0)
                .speed(0.5)
                .max_decimals(1)
                .suffix("\u{b0}");
            // Written back only on an edit, so the radians keep their exact value.
            if ui.add(angle).changed() {
                *normal_deviation_rad = degrees.to_radians();
            }
        });
    } else {
        ui.weak(format!(
            "Surface deviation {} mm, normal deviation {:.0}\u{b0}",
            refinement.surface_deviation_mm(),
            refinement.normal_deviation_rad().to_degrees()
        ));
    }
}

enum BlockedAction {
    Repair,
    Anyway,
    Cancel,
}

fn is_bad(o: &amg_io::ObjectCheck) -> bool {
    o.boundary_edges > 0
        || o.non_manifold_edges > 0
        || o.inconsistent_orientation
        || o.negative_volume
}

fn describe(o: &amg_io::ObjectCheck) -> String {
    let mut parts = Vec::new();
    if o.boundary_edges > 0 {
        parts.push(format!("{} boundary (hole) edges", o.boundary_edges));
    }
    if o.non_manifold_edges > 0 {
        parts.push(format!("{} non-manifold edges", o.non_manifold_edges));
    }
    if o.inconsistent_orientation {
        parts.push("inconsistent orientation".to_owned());
    }
    if o.negative_volume {
        parts.push("inside-out (negative volume)".to_owned());
    }
    parts.join(", ")
}

/// `"<feature id>:<slot>"` as `scene_from_document` writes it.
fn parse_source_id(s: &str) -> Option<BodyId> {
    let (o, slot) = s.split_once(':')?;
    Some(BodyId {
        origin: FeatureId(o.parse().ok()?),
        slot: slot.parse().ok()?,
    })
}

/// `path` with the format's extension (kept if it already has one of them).
fn fix_extension(path: PathBuf, format: Format) -> PathBuf {
    let exts = format.info().extensions;
    let has = path
        .extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| exts.iter().any(|x| x.eq_ignore_ascii_case(e)));
    if has {
        return path;
    }
    let mut s = path.into_os_string();
    s.push(".");
    s.push(exts.first().copied().unwrap_or("stl"));
    PathBuf::from(s)
}

/// What the export job needs (all owned: the job outlives the frame).
struct Input {
    design: Arc<Design>,
    eval: Arc<Evaluated>,
    selection: ExportSelection,
    scene: Option<Box<Scene>>,
    opts: ExportOptions,
    refinement: ExportRefinement,
    format: Format,
    path: PathBuf,
}

trait JobErrorExt {
    fn is_cancelled_job(&self) -> bool;
}

impl JobErrorExt for amg_jobs::JobError<IoJobError> {
    fn is_cancelled_job(&self) -> bool {
        matches!(self, amg_jobs::JobError::Cancelled)
    }
}

/// The export job: prepare the scene (unless one is given), check and write.
fn run(job: Input, ctx: &JobCtx) -> Result<Outcome, IoJobError> {
    let Input {
        design,
        eval,
        selection,
        scene,
        opts,
        refinement,
        format,
        path,
    } = job;
    let (scene, write_ctx) = match scene {
        Some(s) => (s, ctx.sub(0.0, 1.0)),
        None => {
            ctx.progress_msg(0.0, "Preparing the design");
            let prep = ExportPrep {
                refinement: Some(refinement),
                ..ExportPrep::default()
            };
            // Solid bodies are tessellated here, which can take most of the export.
            let share = if eval
                .bodies()
                .any(|(_, b)| matches!(b.geom, BodyGeom::Solid(_)))
            {
                0.7
            } else {
                0.15
            };
            let s = scene_from_document(&design, &eval, &selection, &prep, &ctx.sub(0.0, share))?;
            (Box::new(s), ctx.sub(share, 1.0))
        }
    };
    match amg_io::export(&scene, Sink::Path(&path), format, &opts, &write_ctx) {
        Ok(report) => Ok(Outcome::Written(report)),
        Err(IoError::InvalidForFormat { check, .. }) => Ok(Outcome::Blocked {
            scene,
            check: *check,
        }),
        Err(e) => Err(e.into()),
    }
}

impl AppDialog for ExportDialog {
    fn title(&self) -> Cow<'static, str> {
        "Export".into()
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        if !self.poll(cx) {
            return false;
        }
        match &self.stage {
            Stage::Form => {
                let busy = !cx.doc_tab().document.is_evaluated();
                let has_body = cx.doc_tab().document.evaluated().bodies().next().is_some();
                let ok = if busy {
                    Enablement::disabled("The design is recomputing")
                } else if !has_body {
                    Enablement::disabled("The design has no body to export")
                } else {
                    Enablement::Enabled
                };
                let title = self.title();
                let cx_ref = &*cx;
                let action = DialogFrame::new(&title)
                    .ok_enabled(ok)
                    .show(ctx, |ui| self.form_ui(ui, cx_ref));
                match action {
                    DialogAction::Ok => {
                        self.start_picking(cx);
                        true
                    }
                    DialogAction::Cancel => false,
                    DialogAction::None => true,
                }
            }
            Stage::Picking(_) => {
                ctx.request_repaint_after(PROGRESS_REFRESH);
                true
            }
            Stage::Working(_) => {
                self.progress_ui(ctx, "Exporting");
                true
            }
            Stage::Blocked { .. } => self.blocked_ui(ctx, cx),
            Stage::Repairing { .. } => {
                ctx.request_repaint_after(PROGRESS_REFRESH);
                egui::Window::new("Export")
                    .id(egui::Id::new("amg_io_export_progress"))
                    .collapsible(false)
                    .resizable(false)
                    .show(ctx, |ui| {
                        ui.label("Repairing the meshes before the export…");
                    });
                true
            }
        }
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}
