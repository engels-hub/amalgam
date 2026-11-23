//! The Insert Mesh / Import dialog (S12 §4, IO-15).
//!
//! The file is read **once**, off the UI thread, with [`raw_options`] (file numbers, no scale):
//! the dialog then shows the extents under the chosen unit and up axis from the raw bounding
//! box, so a unit change re-reads nothing (Done-when 3). OK builds the "Insert <file>"
//! transaction on a second job ([`import_commands`]) and applies it as one undo entry.

use std::borrow::Cow;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::time::Duration;

use amg_core::units::LengthUnit;
use amg_core::{Cancelled, Diagnostic, Severity, Transform};
use amg_features::io::{
    ImportChoices, ImportPlan, ImportPrepError, ImportSource, import_commands, raw_options,
};
use amg_io::{
    Confidence, Format, ImportOptions, Imported, IoError, Scene, SceneNode, SceneObject, Source,
    UnitChoice, UpAxis, guess_unit, scale_to_mm,
};
use amg_jobs::{JobCtx, JobHandle, Priority, SpawnOptions};

use super::prefs::{self, ImportPref};
use crate::app::AppCx;
use crate::command::Enablement;
use crate::dialog::AppDialog;
use crate::file::{FileDialogKind, FileDialogRequest, FileFilter};
use crate::notify::Notification;
use crate::widgets::{DialogAction, DialogFrame};

/// Files below this size are read on the interactive pool, larger ones in the background
/// (S12 §1).
pub const BACKGROUND_THRESHOLD: u64 = 50 * 1024 * 1024;

/// Repaint period while a job runs (progress has no notifier).
const PROGRESS_REFRESH: Duration = Duration::from_millis(100);

/// The error of the dialog's jobs.
#[derive(Debug, thiserror::Error)]
pub enum IoJobError {
    /// A reader or writer error.
    #[error(transparent)]
    Io(#[from] IoError),
    /// Building the document transaction failed.
    #[error(transparent)]
    Prep(ImportPrepError),
    /// The job was cancelled.
    #[error("operation cancelled")]
    Cancelled(#[from] Cancelled),
}

impl From<ImportPrepError> for IoJobError {
    fn from(e: ImportPrepError) -> Self {
        match e {
            ImportPrepError::Io(e) => IoJobError::Io(e),
            e => IoJobError::Prep(e),
        }
    }
}

impl Diagnostic for IoJobError {
    fn user_message(&self) -> String {
        match self {
            IoJobError::Io(e) => e.user_message(),
            IoJobError::Prep(e) => e.user_message(),
            IoJobError::Cancelled(_) => "Cancelled.".to_owned(),
        }
    }
    fn hint(&self) -> Option<String> {
        match self {
            IoJobError::Io(e) => e.hint(),
            IoJobError::Prep(e) => e.hint(),
            IoJobError::Cancelled(_) => None,
        }
    }
    fn severity(&self) -> Severity {
        match self {
            IoJobError::Io(e) => e.severity(),
            IoJobError::Prep(e) => e.severity(),
            IoJobError::Cancelled(_) => Severity::Info,
        }
    }
    fn is_cancelled(&self) -> bool {
        match self {
            IoJobError::Io(e) => e.is_cancelled(),
            IoJobError::Prep(e) => e.is_cancelled(),
            IoJobError::Cancelled(_) => true,
        }
    }
}

/// The unit choice of the dialog.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UnitSel {
    /// The unit the file declares (mm if it declares none).
    FromFile,
    /// Read the numbers in this unit, whatever the file declares.
    Unit(LengthUnit),
}

/// Short label of a unit ("mm", "in").
pub fn unit_label(u: LengthUnit) -> &'static str {
    match u {
        LengthUnit::Micrometre => "µm",
        LengthUnit::Millimetre => "mm",
        LengthUnit::Centimetre => "cm",
        LengthUnit::Metre => "m",
        LengthUnit::Inch => "in",
        LengthUnit::Foot => "ft",
    }
}

/// Name of a unit for the dropdown.
pub fn unit_name(u: LengthUnit) -> &'static str {
    match u {
        LengthUnit::Micrometre => "Micrometres (µm)",
        LengthUnit::Millimetre => "Millimetres (mm)",
        LengthUnit::Centimetre => "Centimetres (cm)",
        LengthUnit::Metre => "Metres (m)",
        LengthUnit::Inch => "Inches (in)",
        LengthUnit::Foot => "Feet (ft)",
    }
}

/// `x × y × z` with three decimals.
pub fn fmt_size(e: [f64; 3]) -> String {
    format!("{:.3} × {:.3} × {:.3}", e[0], e[1], e[2])
}

/// Whether a part of size `size_mm` fits a build volume of `bed_mm`; the part may be turned
/// about Z (x and y swapped).
pub fn bed_fit(size_mm: [f64; 3], bed_mm: [f64; 3]) -> bool {
    let eps = 1e-9;
    let direct = size_mm[0] <= bed_mm[0] + eps && size_mm[1] <= bed_mm[1] + eps;
    let turned = size_mm[1] <= bed_mm[0] + eps && size_mm[0] <= bed_mm[1] + eps;
    (direct || turned) && size_mm[2] <= bed_mm[2] + eps
}

/// Extents of a raw bounding box once `up` is applied (Y up turns +90° about X: the
/// extents are x, z, y).
pub fn up_axis_extents(raw: [f64; 3], up: UpAxis) -> [f64; 3] {
    match up {
        UpAxis::Z => raw,
        UpAxis::Y => [raw[0], raw[2], raw[1]],
    }
}

/// What the read job produced: the raw scene (file numbers) plus its source description.
#[derive(Clone, Debug)]
pub struct Loaded {
    /// The scene as read with [`raw_options`].
    pub imported: Imported,
    /// Hash and name of the file.
    pub source: ImportSource,
    /// File size in bytes.
    pub bytes: u64,
    /// Bounding box size of the raw scene, in file numbers.
    pub raw_extents: [f64; 3],
}

enum Stage {
    Picking(Receiver<Option<PathBuf>>),
    Reading(JobHandle<Loaded, IoJobError>),
    Ready(Box<Loaded>),
    Inserting(JobHandle<ImportPlan, IoJobError>, Vec<String>),
}

/// The Insert Mesh dialog.
pub struct ImportDialog {
    path: Option<PathBuf>,
    stage: Stage,
    unit: UnitSel,
    up_axis: UpAxis,
    merge: bool,
    bed_mm: [f64; 3],
    /// The bed comes from a print setup of the design.
    bed_from_setup: bool,
    reads: u32,
}

impl std::fmt::Debug for ImportDialog {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ImportDialog")
            .field("path", &self.path)
            .field("unit", &self.unit)
            .field("up_axis", &self.up_axis)
            .field("merge", &self.merge)
            .field("reads", &self.reads)
            .finish_non_exhaustive()
    }
}

/// The build volume of the design's first valid print setup (AM-01), mm.
fn setup_volume(cx: &AppCx<'_>) -> Option<[f64; 3]> {
    let design = cx.doc_tab().document.design();
    amg_features::am::setups(design)
        .into_iter()
        .find_map(|(_, s)| s.ok())
        .map(|s| {
            let d = s.machine.volume.aabb().size();
            [d.x, d.y, d.z]
        })
}

impl ImportDialog {
    fn base(cx: &AppCx<'_>) -> ImportDialog {
        let prefs = prefs::load();
        let setup = setup_volume(cx);
        ImportDialog {
            path: None,
            stage: Stage::Picking(std::sync::mpsc::channel().1),
            unit: UnitSel::FromFile,
            up_axis: UpAxis::Z,
            merge: false,
            bed_mm: setup.unwrap_or(prefs.bed_mm),
            bed_from_setup: setup.is_some(),
            reads: 0,
        }
    }

    /// A dialog that starts with the native Open dialog (File ▸ Insert Mesh…).
    pub fn pick(cx: &mut AppCx<'_>) -> ImportDialog {
        let mut d = ImportDialog::base(cx);
        let prefs = prefs::load();
        let formats = amg_io::registry()
            .iter()
            .filter(|f| f.can_import && f.kind == amg_io::GeometryKind::Mesh)
            .collect::<Vec<_>>();
        let all: Vec<String> = formats
            .iter()
            .flat_map(|f| f.extensions.iter().map(|e| (*e).to_owned()))
            .collect();
        let mut filters = vec![FileFilter {
            name: "All mesh files".to_owned(),
            extensions: all,
        }];
        filters.extend(formats.iter().map(|f| FileFilter {
            name: f.name.to_owned(),
            extensions: f.extensions.iter().map(|e| (*e).to_owned()).collect(),
        }));
        d.stage = Stage::Picking(cx.pick_file(FileDialogRequest {
            kind: FileDialogKind::Open,
            title: "Insert Mesh".to_owned(),
            filters,
            directory: prefs.last_dir,
            file_name: None,
        }));
        d
    }

    /// A dialog for `path` (drag and drop, tests): starts reading at once.
    pub fn for_path(cx: &mut AppCx<'_>, path: &Path) -> ImportDialog {
        let mut d = ImportDialog::base(cx);
        d.start_read(cx, path.to_owned());
        d
    }

    fn start_read(&mut self, cx: &mut AppCx<'_>, path: PathBuf) {
        let size = std::fs::metadata(&path).map_or(0, |m| m.len());
        let priority = if size >= BACKGROUND_THRESHOLD {
            Priority::Background
        } else {
            Priority::Interactive
        };
        let label = format!(
            "Read {}",
            path.file_name().map_or_else(
                || path.display().to_string(),
                |n| n.to_string_lossy().into_owned()
            )
        );
        let job_path = path.clone();
        self.reads += 1;
        let handle = cx.jobs().spawn_fn(
            label,
            SpawnOptions {
                priority,
                hidden: false,
            },
            move |ctx| read_file(&job_path, ctx),
        );
        self.path = Some(path);
        self.stage = Stage::Reading(handle);
    }

    /// The file this dialog imports.
    pub fn path(&self) -> Option<&Path> {
        self.path.as_deref()
    }

    /// How many times the file was read (a unit change must not add one).
    pub fn reads(&self) -> u32 {
        self.reads
    }

    /// True once the file is read and the options are shown.
    pub fn is_ready(&self) -> bool {
        matches!(self.stage, Stage::Ready(_))
    }

    /// True while the file is being read.
    pub fn is_reading(&self) -> bool {
        matches!(self.stage, Stage::Reading(_))
    }

    /// Cancels the running read or insert job (the Cancel button does the same).
    pub fn cancel(&self) {
        match &self.stage {
            Stage::Reading(h) => h.cancel(),
            Stage::Inserting(h, _) => h.cancel(),
            _ => {}
        }
    }

    /// The read result, once ready.
    pub fn loaded(&self) -> Option<&Loaded> {
        match &self.stage {
            Stage::Ready(l) => Some(l),
            _ => None,
        }
    }

    /// The chosen unit.
    pub fn unit(&self) -> UnitSel {
        self.unit
    }

    /// Chooses the unit; the shown extents follow at once, the file is not read again.
    pub fn set_unit(&mut self, unit: UnitSel) {
        self.unit = unit;
    }

    /// The chosen up axis.
    pub fn up_axis(&self) -> UpAxis {
        self.up_axis
    }

    /// Chooses the up axis.
    pub fn set_up_axis(&mut self, up: UpAxis) {
        self.up_axis = up;
    }

    /// One body for the whole file.
    pub fn merge(&self) -> bool {
        self.merge
    }

    /// Chooses merge or separate.
    pub fn set_merge(&mut self, merge: bool) {
        self.merge = merge;
    }

    /// The unit the numbers are read in under the current choice.
    pub fn effective_unit(&self) -> LengthUnit {
        match self.unit {
            UnitSel::Unit(u) => u,
            UnitSel::FromFile => self
                .loaded()
                .and_then(|l| l.imported.report.declared_unit)
                .unwrap_or(LengthUnit::Millimetre),
        }
    }

    /// The size of the result in mm under the current unit and up axis.
    pub fn extents_mm(&self) -> Option<[f64; 3]> {
        let l = self.loaded()?;
        let s = scale_to_mm(1.0, self.effective_unit());
        let e = up_axis_extents(l.raw_extents, self.up_axis);
        Some(e.map(|v| v * s))
    }

    /// The import options the choices stand for.
    pub fn options(&self) -> ImportOptions {
        ImportOptions {
            unit: match self.unit {
                UnitSel::FromFile => UnitChoice::default(),
                UnitSel::Unit(u) => UnitChoice::Assume(u),
            },
            up_axis: self.up_axis,
            merge_objects: self.merge,
            ..ImportOptions::default()
        }
    }

    fn ready(&mut self, l: Loaded) {
        let format = l.imported.report.format.unwrap_or(Format::Stl);
        let pref = prefs::load().import_for(format);
        self.unit = pref.unit.map_or(UnitSel::FromFile, UnitSel::Unit);
        self.up_axis = pref.up_axis;
        self.merge = pref.merge;
        self.stage = Stage::Ready(Box::new(l));
    }

    fn remember(&self) {
        let Some(l) = self.loaded() else { return };
        let Some(format) = l.imported.report.format else {
            return;
        };
        let pref = ImportPref {
            unit: match self.unit {
                UnitSel::FromFile => None,
                UnitSel::Unit(u) => Some(u),
            },
            up_axis: self.up_axis,
            merge: self.merge,
        };
        let dir = self
            .path
            .as_deref()
            .and_then(Path::parent)
            .map(Path::to_owned);
        prefs::update(|p| {
            p.import.insert(format, pref);
            if dir.is_some() {
                p.last_dir = dir;
            }
        });
    }

    /// OK: builds the transaction on a job.
    pub fn insert(&mut self, cx: &mut AppCx<'_>) {
        let Stage::Ready(loaded) = &self.stage else {
            return;
        };
        self.remember();
        let loaded = (**loaded).clone();
        let warnings: Vec<String> = loaded
            .imported
            .report
            .warnings
            .iter()
            .map(amg_io::IoWarning::message)
            .collect();
        let opts = self.options();
        let target = cx.doc_tab().document.active_component();
        let label = format!("Insert {}", loaded.source.file_name);
        let handle = cx
            .jobs()
            .spawn_fn(label, SpawnOptions::default(), move |ctx| {
                insert_plan(loaded, &opts, target, ctx)
            });
        self.stage = Stage::Inserting(handle, warnings);
    }

    fn file_title(&self) -> String {
        self.path
            .as_deref()
            .and_then(Path::file_name)
            .map_or_else(String::new, |n| n.to_string_lossy().into_owned())
    }

    /// The window of a running job: progress and Cancel.
    fn progress_ui(&self, ctx: &egui::Context, what: &str) {
        let (fraction, message) = match &self.stage {
            Stage::Reading(h) => {
                let p = h.progress();
                (p.fraction, p.message)
            }
            Stage::Inserting(h, _) => {
                let p = h.progress();
                (p.fraction, p.message)
            }
            _ => (0.0, None),
        };
        let mut cancel = false;
        egui::Window::new("Insert Mesh")
            .id(egui::Id::new("amg_io_import_window"))
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                ui.label(format!("{what} {}", self.file_title()));
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

    fn options_ui(&mut self, ui: &mut egui::Ui) {
        let Stage::Ready(loaded) = &self.stage else {
            return;
        };
        let loaded = loaded.clone();
        let report = &loaded.imported.report;
        let format_name = report.format.map_or("Unknown", |f| f.info().name);
        ui.label(format!("File: {}", self.file_title()));
        ui.label(format!(
            "Format: {format_name} · {} triangles · {} objects",
            report.triangles_read,
            loaded.imported.scene.objects.len()
        ));
        ui.separator();
        let declared = report.declared_unit;
        let guess = guess_unit(loaded.raw_extents, declared);
        let from_file = match declared {
            Some(u) => format!("From file ({})", unit_label(u)),
            None => "From file (mm, none declared)".to_owned(),
        };
        ui.horizontal(|ui| {
            ui.label("Unit");
            let current = match self.unit {
                UnitSel::FromFile => from_file.clone(),
                UnitSel::Unit(u) => unit_name(u).to_owned(),
            };
            egui::ComboBox::from_id_salt("amg_io_import_unit")
                .selected_text(current)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.unit, UnitSel::FromFile, from_file);
                    for u in LengthUnit::ALL {
                        ui.selectable_value(&mut self.unit, UnitSel::Unit(u), unit_name(u));
                    }
                });
        });
        match declared {
            Some(u) => {
                ui.weak(format!("The file declares {}", unit_name(u)));
            }
            None => {
                ui.weak("The file does not declare a unit");
                if guess.suggested != LengthUnit::Millimetre {
                    let how = match guess.confidence {
                        Confidence::High => "",
                        Confidence::Medium => " (probably)",
                        Confidence::Low => " (unsure)",
                    };
                    ui.colored_label(
                        ui.visuals().warn_fg_color,
                        format!("Suggested unit: {}{how}", unit_name(guess.suggested)),
                    );
                }
            }
        }
        ui.horizontal(|ui| {
            ui.label("Up axis");
            ui.selectable_value(&mut self.up_axis, UpAxis::Z, "Z up");
            ui.selectable_value(&mut self.up_axis, UpAxis::Y, "Y up");
        });
        if loaded.imported.scene.objects.len() > 1 {
            ui.checkbox(&mut self.merge, "Merge into one body");
        }
        ui.separator();
        if let Some(e) = self.extents_mm() {
            ui.strong(format!("Size: {} mm", fmt_size(e)));
            let inch = scale_to_mm(1.0, LengthUnit::Inch);
            ui.label(format!("({} in)", fmt_size(e.map(|v| v / inch))));
            let raw = up_axis_extents(loaded.raw_extents, self.up_axis);
            for (u, _) in &guess.candidates {
                if matches!(u, LengthUnit::Millimetre | LengthUnit::Inch)
                    && *u != self.effective_unit()
                {
                    let alt = raw.map(|v| v * scale_to_mm(1.0, *u));
                    ui.weak(format!("As {}: {} mm", unit_label(*u), fmt_size(alt)));
                }
            }
            let [bx, by, bz] = self.bed_mm;
            let source = if self.bed_from_setup {
                "print setup"
            } else {
                "default bed"
            };
            if bed_fit(e, self.bed_mm) {
                ui.colored_label(
                    egui::Color32::from_rgb(0x4c, 0xaf, 0x50),
                    format!("Fits the build volume ({bx:.0} × {by:.0} × {bz:.0} mm, {source})"),
                );
            } else {
                ui.colored_label(
                    ui.visuals().warn_fg_color,
                    format!(
                        "Does not fit the build volume ({bx:.0} × {by:.0} × {bz:.0} mm, {source})"
                    ),
                );
            }
        }
        if !report.warnings.is_empty() {
            ui.separator();
            for w in &report.warnings {
                ui.colored_label(ui.visuals().warn_fg_color, format!("⚠ {}", w.message()));
            }
        }
    }

    /// Polls the running stage. Returns false if the dialog is finished.
    fn poll(&mut self, cx: &mut AppCx<'_>) -> bool {
        match &mut self.stage {
            Stage::Picking(rx) => match rx.try_recv() {
                Ok(Some(path)) => self.start_read(cx, path),
                Ok(None) | Err(TryRecvError::Disconnected) => return false,
                Err(TryRecvError::Empty) => {}
            },
            Stage::Reading(h) => {
                if let Some(result) = h.try_take() {
                    match result {
                        Ok(loaded) => self.ready(loaded),
                        Err(e) => {
                            cx.notify_error(&e);
                            return false;
                        }
                    }
                }
            }
            Stage::Inserting(h, warnings) => {
                if let Some(result) = h.try_take() {
                    let warnings = std::mem::take(warnings);
                    match result {
                        Ok(plan) => {
                            let label = plan.label.clone();
                            cx.apply_document(label, move |doc| {
                                plan.apply(doc).map(|_| ()).map_err(|e| match e {
                                    ImportPrepError::Command(c) => c,
                                    other => amg_doc::CommandError::Invalid(other.to_string()),
                                })
                            });
                            if !warnings.is_empty() {
                                cx.notify(Notification::warning(
                                    "Imported with warnings",
                                    warnings.join("\n"),
                                ));
                            }
                        }
                        Err(e) => cx.notify_error(&e),
                    }
                    return false;
                }
            }
            Stage::Ready(_) => {}
        }
        true
    }
}

impl AppDialog for ImportDialog {
    fn title(&self) -> Cow<'static, str> {
        "Insert Mesh".into()
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        if !self.poll(cx) {
            return false;
        }
        match &self.stage {
            Stage::Reading(_) => {
                self.progress_ui(ctx, "Reading");
                true
            }
            Stage::Inserting(..) => {
                self.progress_ui(ctx, "Inserting");
                true
            }
            Stage::Ready(_) => {
                let title = self.title();
                let action = DialogFrame::new(&title)
                    .ok_enabled(Enablement::Enabled)
                    .show(ctx, |ui| self.options_ui(ui));
                match action {
                    DialogAction::Ok => {
                        self.insert(cx);
                        true
                    }
                    DialogAction::Cancel => false,
                    DialogAction::None => true,
                }
            }
            Stage::Picking(_) => true,
        }
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

/// The read job: raw scene, source hash, raw extents. Progress: the read takes 90 %, the
/// hash 10 %.
fn read_file(path: &Path, ctx: &JobCtx) -> Result<Loaded, IoJobError> {
    let bytes = std::fs::metadata(path)
        .map_err(|source| IoError::Read {
            path: path.to_owned(),
            source,
        })?
        .len();
    let opts = raw_options(&ImportOptions::default());
    let imported = amg_io::import(Source::Path(path), None, &opts, &ctx.sub(0.0, 0.9))?;
    let format = imported.report.format.unwrap_or(Format::Stl);
    let source = ImportSource::from_path(
        path,
        format,
        imported.report.declared_unit,
        &ctx.sub(0.9, 1.0),
    )?;
    let size = imported.scene.world_aabb().size();
    Ok(Loaded {
        imported,
        source,
        bytes,
        raw_extents: [size.x, size.y, size.z],
    })
}

/// One object holding every instance of `scene` in world space (file units).
fn merged_scene(scene: &Scene, name: &str, ctx: &JobCtx) -> Result<Scene, IoError> {
    let (mesh, slots) = scene.flatten(ctx)?;
    let mut object = SceneObject::new(Arc::new(mesh));
    object.name = Some(name.to_owned());
    object.face_materials = slots.map(Arc::from);
    Ok(Scene {
        objects: vec![object],
        roots: vec![SceneNode::mesh(0, Transform::IDENTITY)],
        materials: scene.materials.clone(),
        metadata: scene.metadata.clone(),
        plates: Vec::new(),
        thumbnail: scene.thumbnail.clone(),
    })
}

/// The insert job: merge if asked, then the "Insert <file>" plan.
fn insert_plan(
    loaded: Loaded,
    opts: &ImportOptions,
    target: amg_doc::ComponentId,
    ctx: &JobCtx,
) -> Result<ImportPlan, IoJobError> {
    let Loaded {
        mut imported,
        source,
        ..
    } = loaded;
    if opts.merge_objects && imported.scene.objects.len() > 1 {
        let stem = Path::new(&source.file_name)
            .file_stem()
            .map_or_else(|| "Import".to_owned(), |s| s.to_string_lossy().into_owned());
        imported.scene = merged_scene(&imported.scene, &stem, ctx)?;
    }
    ctx.checkpoint()?;
    let choices = ImportChoices::from_options(target, opts, &imported.report);
    Ok(import_commands(imported, source, &choices)?)
}

/// The drop handler: every dropped file is read; the reader sniffs the format, so an
/// unsupported file ends in the reader's `IoError` toast with its hint.
pub(crate) fn on_drop(cx: &mut AppCx<'_>, path: &Path) -> bool {
    let dialog = ImportDialog::for_path(cx, path);
    cx.open_dialog(Box::new(dialog));
    true
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact literals")]
mod tests {
    use super::*;

    #[test]
    fn bed_fit_allows_turning_about_z() {
        let bed = [256.0, 200.0, 256.0];
        assert!(bed_fit([250.0, 190.0, 10.0], bed));
        assert!(bed_fit([190.0, 250.0, 10.0], bed), "turned");
        assert!(!bed_fit([250.0, 250.0, 10.0], bed));
        assert!(!bed_fit([10.0, 10.0, 300.0], bed));
    }

    #[test]
    fn y_up_swaps_y_and_z() {
        assert_eq!(up_axis_extents([1.0, 2.0, 3.0], UpAxis::Z), [1.0, 2.0, 3.0]);
        assert_eq!(up_axis_extents([1.0, 2.0, 3.0], UpAxis::Y), [1.0, 3.0, 2.0]);
    }

    #[test]
    fn size_text_has_three_decimals() {
        assert_eq!(fmt_size([1.0, 25.4, 0.5]), "1.000 × 25.400 × 0.500");
    }
}
