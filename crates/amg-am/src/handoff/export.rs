//! Writing the plate: plate frame → slicer frame, 3MF (Orca flavour for Orca/Bambu) or STL,
//! atomically; the hand-off directory and its pruning.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use amg_core::Transform;
use amg_core::glam::DVec3;
use amg_io::{
    ExportOptions, IoError, MaterialSlot, Plate as IoPlate, Scene, SceneNode, SceneObject,
    SlicerFlavor, Thumbnail, ValidityPolicy,
};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, Rgba8};
use serde::{Deserialize, Serialize};

use crate::AmError;
use crate::machine::{Machine, SlicerKind};

/// File format of a hand-off.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HandoffFormat {
    /// 3MF (Orca flavour for Orca/Bambu, core 3MF otherwise).
    ThreeMf,
    /// One binary STL with all items merged.
    Stl,
}

impl HandoffFormat {
    /// `3mf` or `stl`.
    pub fn extension(self) -> &'static str {
        match self {
            HandoffFormat::ThreeMf => "3mf",
            HandoffFormat::Stl => "stl",
        }
    }
}

/// One body on the plate.
#[derive(Clone, Debug)]
pub struct HandoffItem {
    /// Object name in the slicer.
    pub name: String,
    /// The body's mesh, body-local.
    pub mesh: Arc<IndexedMesh>,
    /// Body-local → plate frame (`place::effective_transform`).
    pub transform: Transform,
    /// 1-based filament slot.
    pub filament: Option<u8>,
    /// Display colour. Written with the filament for Orca/Bambu, as a material otherwise.
    pub color: Option<Rgba8>,
}

/// A plate ready to hand off.
#[derive(Clone, Debug)]
pub struct HandoffPlate<'a> {
    /// Design name (3MF `Title`, file name).
    pub design_name: String,
    /// Plate name (file name, plate name in Orca).
    pub plate_name: String,
    /// The machine; its `volume.slicer_origin` places the plate in slicer coordinates.
    pub machine: &'a Machine,
    /// The items.
    pub items: Vec<HandoffItem>,
    /// Optional PNG thumbnail of the plate.
    pub thumbnail_png: Option<Vec<u8>>,
}

/// What [`export_plate`] wrote.
#[derive(Clone, Debug)]
pub struct HandoffReport {
    /// The file.
    pub path: PathBuf,
    /// Objects written.
    pub objects: usize,
    /// Triangles written.
    pub triangles: usize,
    /// File size.
    pub bytes: u64,
    /// Wall time of the export.
    pub millis: u32,
    /// Writer warnings (for example an open mesh; the slicer repairs those), as text.
    pub warnings: Vec<String>,
}

/// `%LOCALAPPDATA%\Amalgam\handoff` (the temp directory when `LOCALAPPDATA` is unset).
pub fn default_handoff_dir() -> PathBuf {
    std::env::var_os("LOCALAPPDATA")
        .map_or_else(std::env::temp_dir, PathBuf::from)
        .join("Amalgam")
        .join("handoff")
}

/// A file-name-safe version of `s`: letters, digits, `-`, `_` and `.` stay, everything else
/// becomes `_`; leading/trailing dots and spaces go; empty gives `plate`; at most 48 chars.
fn sanitise(s: &str) -> String {
    let t: String = s
        .chars()
        .map(|c| {
            if c.is_alphanumeric() || matches!(c, '-' | '_' | '.') {
                c
            } else {
                '_'
            }
        })
        .collect();
    let t = t.trim_matches(|c| c == '.' || c == '_').to_owned();
    let t: String = t.chars().take(48).collect();
    if t.is_empty() { "plate".to_owned() } else { t }
}

/// `YYYYMMDD-HHMMSS-mmm` (UTC) of `t`.
fn timestamp(t: SystemTime) -> String {
    let d = t.duration_since(UNIX_EPOCH).unwrap_or_default();
    let secs = d.as_secs();
    let days = i64::try_from(secs / 86_400).unwrap_or(0);
    let rem = secs % 86_400;
    // Civil-from-days (H. Hinnant).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}{month:02}{day:02}-{:02}{:02}{:02}-{:03}",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60,
        d.subsec_millis()
    )
}

/// `<dir>/<design>_<plate>_<YYYYMMDD-HHMMSS-mmm>.<ext>`, names sanitised for the file system.
///
/// Deviation from the S16 sketch: takes the `format`, because the extension depends on it.
pub fn handoff_path(
    dir: &Path,
    design: &str,
    plate: &str,
    now: SystemTime,
    format: HandoffFormat,
) -> PathBuf {
    dir.join(format!(
        "{}_{}_{}.{}",
        sanitise(design),
        sanitise(plate),
        timestamp(now),
        format.extension()
    ))
}

/// Rigid = right-handed orthonormal linear part and finite numbers.
fn is_rigid(t: &Transform) -> bool {
    const TOL: f64 = 1e-6;
    let m = t.matrix3;
    let (x, y, z) = (m.x_axis, m.y_axis, m.z_axis);
    t.translation.is_finite()
        && m.to_cols_array().iter().all(|v| v.is_finite())
        && (x.length() - 1.0).abs() < TOL
        && (y.length() - 1.0).abs() < TOL
        && (z.length() - 1.0).abs() < TOL
        && x.dot(y).abs() < TOL
        && x.dot(z).abs() < TOL
        && y.dot(z).abs() < TOL
        && x.cross(y).dot(z) > 0.0
}

fn io_err(e: IoError) -> AmError {
    match e {
        IoError::Cancelled => AmError::Cancelled,
        other => AmError::Io(other.to_string()),
    }
}

/// The 3MF flavour a slicer reads best. PrusaSlicer's flavour (IO-12) is not written yet, so
/// it gets the core 3MF (geometry, names and positions; no per-object extruders).
fn flavour_of(kind: SlicerKind) -> SlicerFlavor {
    match kind {
        SlicerKind::OrcaSlicer | SlicerKind::BambuStudio => SlicerFlavor::Orca,
        SlicerKind::PrusaSlicer | SlicerKind::Chitubox | SlicerKind::Custom => {
            SlicerFlavor::Generic
        }
    }
}

/// The plate as an `amg-io` scene in slicer coordinates: every item translated by
/// `−machine.volume.slicer_origin` (z unchanged), one object per item.
fn scene_of(p: &HandoffPlate<'_>, flavor: SlicerFlavor, ctx: &JobCtx) -> Result<Scene, AmError> {
    let origin = p.machine.volume.slicer_origin;
    let shift = Transform::from_translation(DVec3::new(-origin.x, -origin.y, 0.0));
    let orca = flavor == SlicerFlavor::Orca;
    let mut scene = Scene::default();
    // Slot per filament (Orca) or per colour (otherwise).
    let mut slot_keys: Vec<(Option<u8>, Option<Rgba8>)> = Vec::new();
    for (k, item) in p.items.iter().enumerate() {
        ctx.checkpoint()?;
        if !is_rigid(&item.transform) {
            return Err(AmError::NotRigid);
        }
        let mut o = SceneObject::new(Arc::clone(&item.mesh));
        o.name = Some(item.name.clone());
        o.filament = item.filament;
        if let Some(color) = item.color
            && (!orca || item.filament.is_some())
        {
            let key = if orca {
                (item.filament, None)
            } else {
                (None, Some(color))
            };
            let slot = match slot_keys.iter().position(|s| *s == key) {
                Some(i) => i,
                None => {
                    slot_keys.push(key);
                    scene.materials.push(MaterialSlot {
                        name: format!("Item colour {}", slot_keys.len()),
                        color,
                        filament: if orca { item.filament } else { None },
                    });
                    slot_keys.len() - 1
                }
            };
            o.material = u16::try_from(slot).ok();
        }
        scene.objects.push(o);
        let mut n = SceneNode::mesh(k as u32, shift * item.transform);
        n.name = Some(item.name.clone());
        scene.roots.push(n);
    }
    scene.plates.push(IoPlate {
        name: Some(p.plate_name.clone()),
        roots: (0..scene.roots.len() as u32).collect(),
    });
    scene
        .metadata
        .insert("Title".to_owned(), p.design_name.clone());
    if let Some(png) = &p.thumbnail_png {
        scene.thumbnail = Some(Thumbnail::Png(Arc::from(png.as_slice())));
    }
    Ok(scene)
}

/// Writes the plate to `path` (temp file + rename, so a reader never sees half a file) in
/// slicer coordinates (`plate − machine.volume.slicer_origin`). `flavor_for` picks the 3MF
/// flavour (Orca/Bambu get the Orca one). Open meshes are written with a warning: the slicers
/// repair them, and blocking a hand-off on that is the export dialog's job.
///
/// # Errors
/// [`AmError::EmptyPlate`], [`AmError::NotRigid`], [`AmError::Io`] (the `amg-io` message) or
/// [`AmError::Cancelled`].
pub fn export_plate(
    p: &HandoffPlate<'_>,
    format: HandoffFormat,
    flavor_for: SlicerKind,
    path: &Path,
    ctx: &JobCtx,
) -> Result<HandoffReport, AmError> {
    let t0 = Instant::now();
    ctx.checkpoint()?;
    if p.items.is_empty() {
        return Err(AmError::EmptyPlate);
    }
    if let Some(dir) = path.parent()
        && !dir.as_os_str().is_empty()
    {
        std::fs::create_dir_all(dir).map_err(|e| AmError::Io(format!("{}: {e}", dir.display())))?;
    }
    let flavor = match format {
        HandoffFormat::ThreeMf => flavour_of(flavor_for),
        HandoffFormat::Stl => SlicerFlavor::Generic,
    };
    let scene = scene_of(p, flavor, ctx)?;
    ctx.progress(0.05);
    let mut opts = ExportOptions {
        validity: Some(ValidityPolicy::Warn),
        ..ExportOptions::default()
    };
    opts.threemf.flavor = flavor;
    let sub = ctx.sub(0.05, 1.0);
    let report = match format {
        HandoffFormat::ThreeMf => {
            amg_io::threemf::write(&scene, amg_io::Sink::Path(path), &opts, &sub)
        }
        HandoffFormat::Stl => amg_io::stl::write(&scene, amg_io::Sink::Path(path), &opts, &sub),
    }
    .map_err(io_err)?;
    Ok(HandoffReport {
        path: path.to_owned(),
        objects: p.items.len(),
        triangles: p.items.iter().map(|i| i.mesh.face_count()).sum(),
        bytes: report.bytes,
        millis: u32::try_from(t0.elapsed().as_millis()).unwrap_or(u32::MAX),
        warnings: report.warnings.iter().map(|w| format!("{w:?}")).collect(),
    })
}

/// Deletes all but the newest `keep` `*.3mf` / `*.stl` files of `dir` (by modification time,
/// ties by name). A missing directory is fine. Returns the number deleted.
///
/// # Errors
/// Reading the directory or deleting a file.
pub fn prune_handoff_dir(dir: &Path, keep: usize) -> std::io::Result<usize> {
    let rd = match std::fs::read_dir(dir) {
        Ok(rd) => rd,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(0),
        Err(e) => return Err(e),
    };
    let mut files: Vec<(SystemTime, PathBuf)> = Vec::new();
    for entry in rd {
        let entry = entry?;
        let path = entry.path();
        let is_target = path
            .extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| e.eq_ignore_ascii_case("3mf") || e.eq_ignore_ascii_case("stl"));
        if is_target && entry.file_type()?.is_file() {
            files.push((entry.metadata()?.modified()?, path));
        }
    }
    files.sort_by(|a, b| b.cmp(a));
    let mut removed = 0;
    for (_, path) in files.into_iter().skip(keep) {
        std::fs::remove_file(&path)?;
        removed += 1;
    }
    Ok(removed)
}
