//! Export a design's bodies to a mesh file, and convert between mesh files.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use amg_core::units::LengthUnit;
use amg_features::io::{ExportPrep, ExportRefinement, ExportSelection, scene_from_document};
use amg_io::{
    CoordPrecision, ExportOptions, ExportReport, Format, ImportOptions, IoWarning, Sink, Source,
    UnitChoice,
};
use amg_jobs::{CancelToken, JobCtx, JobSystem, Priority};
use serde::Serialize;

use super::{BodyFilter, REPORT_SCHEMA, explicit_bodies, output_format};
use crate::ScriptError;
use crate::host::ScriptDoc;

/// What [`export`] writes.
#[derive(Clone, Debug)]
pub struct ExportRequest {
    /// The file to write.
    pub path: PathBuf,
    /// `None`: from the extension.
    pub format: Option<Format>,
    /// Which bodies.
    pub bodies: BodyFilter,
    /// Tessellation of solid bodies; `None` is `Medium`.
    pub refinement: Option<ExportRefinement>,
    /// Writer options.
    pub options: ExportOptions,
}

impl ExportRequest {
    /// Every visible body to `path`, format from the extension, coordinates written without
    /// loss ([`lossless_options`]).
    pub fn new(path: impl Into<PathBuf>) -> ExportRequest {
        ExportRequest {
            path: path.into(),
            format: None,
            bodies: BodyFilter::AllVisible,
            refinement: None,
            options: lossless_options(),
        }
    }
}

/// The writer defaults of scripts: `ExportOptions::default()` with
/// [`CoordPrecision::Roundtrip`], so 3MF and OBJ keep every digit (STL is `f32` by format).
fn lossless_options() -> ExportOptions {
    ExportOptions {
        precision: CoordPrecision::Roundtrip,
        ..ExportOptions::default()
    }
}

/// Result of [`export`] and [`convert`].
#[derive(Clone, Debug, Serialize)]
pub struct ExportSummary {
    /// [`REPORT_SCHEMA`].
    pub schema: u32,
    /// The file written.
    pub path: PathBuf,
    /// Format as its main extension: `"stl"`, `"3mf"`, `"obj"` or `"ply"`.
    pub format: String,
    /// Bodies (scene objects) written.
    pub bodies: usize,
    /// Triangles in the file, as the writer counts them.
    pub triangles: u64,
    /// Bytes written.
    pub bytes: u64,
    /// Reader (convert) and writer warnings.
    pub warnings: Vec<String>,
    /// Wall time of this call, including any recompute or read.
    pub millis: f64,
}

fn summary(
    path: PathBuf,
    format: Format,
    bodies: usize,
    report: &ExportReport,
    read_warnings: &[IoWarning],
    started: Instant,
) -> ExportSummary {
    ExportSummary {
        schema: REPORT_SCHEMA,
        path,
        format: format
            .info()
            .extensions
            .first()
            .map_or_else(String::new, |e| (*e).to_owned()),
        bodies,
        triangles: report.triangles,
        bytes: report.bytes,
        warnings: read_warnings
            .iter()
            .chain(&report.warnings)
            .map(IoWarning::message)
            .collect(),
        millis: started.elapsed().as_secs_f64() * 1000.0,
    }
}

/// Writes bodies of the design to a mesh file (STL, 3MF, OBJ, PLY), recomputing first if
/// needed. Placements are applied; the file is replaced atomically.
///
/// # Errors
/// [`ScriptError::NotFound`] for an unknown body, [`ScriptError::Io`] for an unknown or
/// unwritable format, [`ScriptError::Export`] (nothing to export, tessellation) or
/// [`ScriptError::Cancelled`].
pub fn export(
    doc: &mut ScriptDoc,
    req: &ExportRequest,
    cancel: &CancelToken,
) -> Result<ExportSummary, ScriptError> {
    let started = Instant::now();
    let format = output_format(req.format, &req.path)?;
    let eval = doc.evaluated(cancel)?;
    let design = doc.document().snapshot();
    let selection = explicit_bodies(&design, &eval, &req.bodies)?
        .map_or(ExportSelection::AllVisible, ExportSelection::Bodies);
    let prep = ExportPrep {
        refinement: Some(req.refinement.unwrap_or_default()),
        ..ExportPrep::default()
    };
    let jobs = Arc::clone(doc.jobs());
    let (bodies, report) = jobs.install(Priority::Interactive, || {
        let ctx = JobCtx::with_token(cancel.clone());
        let scene = scene_from_document(&design, &eval, &selection, &prep, &ctx)?;
        let report = amg_io::export(&scene, Sink::Path(&req.path), format, &req.options, &ctx)?;
        Ok::<_, ScriptError>((scene.objects.len(), report))
    })?;
    Ok(summary(
        req.path.clone(),
        format,
        bodies,
        &report,
        &[],
        started,
    ))
}

/// What [`convert`] reads and writes.
#[derive(Clone, Debug)]
pub struct ConvertRequest {
    /// The mesh file to read.
    pub src: PathBuf,
    /// The mesh file to write.
    pub dst: PathBuf,
    /// `None`: sniffed from the content.
    pub src_format: Option<Format>,
    /// `None`: from the extension.
    pub dst_format: Option<Format>,
    /// Unit of the source numbers, overriding what the file declares.
    pub unit: Option<LengthUnit>,
    /// Reader options.
    pub import: ImportOptions,
    /// Writer options.
    pub export: ExportOptions,
}

impl ConvertRequest {
    /// `src` to `dst` with default reader options and lossless writer options
    /// ([`ExportRequest::new`]).
    pub fn new(src: impl Into<PathBuf>, dst: impl Into<PathBuf>) -> ConvertRequest {
        ConvertRequest {
            src: src.into(),
            dst: dst.into(),
            src_format: None,
            dst_format: None,
            unit: None,
            import: ImportOptions::default(),
            export: lossless_options(),
        }
    }
}

/// Reads a mesh file and writes it in another format, without a document: objects,
/// placements, names and materials go through the `amg-io` scene.
///
/// # Errors
/// [`ScriptError::Io`] or [`ScriptError::Cancelled`].
pub fn convert(
    req: &ConvertRequest,
    jobs: &JobSystem,
    cancel: &CancelToken,
) -> Result<ExportSummary, ScriptError> {
    let started = Instant::now();
    let format = output_format(req.dst_format, &req.dst)?;
    let mut import = req.import.clone();
    if let Some(unit) = req.unit {
        import.unit = UnitChoice::Assume(unit);
    }
    let (imported, bodies, report) = jobs.install(Priority::Interactive, || {
        let ctx = JobCtx::with_token(cancel.clone());
        let imported = amg_io::import(Source::Path(&req.src), req.src_format, &import, &ctx)?;
        let report = amg_io::export(
            &imported.scene,
            Sink::Path(&req.dst),
            format,
            &req.export,
            &ctx,
        )?;
        let bodies = imported.scene.objects.len();
        Ok::<_, ScriptError>((imported.report, bodies, report))
    })?;
    Ok(summary(
        req.dst.clone(),
        format,
        bodies,
        &report,
        &imported.warnings,
        started,
    ))
}
