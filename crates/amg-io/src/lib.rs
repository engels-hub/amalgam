//! File formats: STEP/IGES (via brep), STL, 3MF, OBJ, PLY, DXF, SVG.
//!
//! Spec: S12. Lane: G.
//!
//! This is the IO-01 foundation: the format registry and content sniffing, sources and
//! atomic sinks, the format-independent [`Scene`], units and up-axis, options, reports and
//! errors, and the `probe`/`import`/`export` entry points. The readers and writers of the
//! individual formats land with IO-02 (STL, done), IO-05 (OBJ) and later; until then `import` and
//! `export` answer [`IoError::Unsupported`] for the others.

#![warn(missing_docs)]

mod endpoint;
mod error;
mod format;
pub mod obj;
mod options;
pub mod paint;
mod scene;
pub mod stl;
pub mod threemf;
mod units;
mod zipdir;

use amg_core::units::LengthUnit;
use amg_jobs::JobCtx;
use amg_mesh::{AnalyzeOpts, analyze};

pub use endpoint::{Sink, Source};
pub use error::{
    ExportCheck, ExportReport, ImportReport, IoError, IoWarning, Location, ObjectCheck,
};
pub use format::{Format, FormatInfo, GeometryKind, UnitDeclaration, check_extension, registry};
pub use options::{
    Compression, CoordPrecision, ExportOptions, ImportOptions, ObjGroups, ObjReadOptions,
    ObjWriteOptions, PlyWriteOptions, ReadLimits, SlicerFlavor, StlReadOptions, StlWeld,
    StlWriteOptions, ThreeMfReadOptions, ThreeMfWriteOptions, ValidityPolicy,
};
pub use scene::{
    MAX_GROUP_DEPTH, MaterialSlot, NO_SLOT, NodeContent, ObjectKind, PartSubtype, Plate, Scene,
    SceneNode, SceneObject, SlotIndex, Thumbnail,
};
pub use units::{
    Confidence, UnitChoice, UnitGuess, UpAxis, convert_length, guess_unit, scale_from_mm,
    scale_to_mm,
};

/// Result of [`import`].
#[derive(Clone, Debug)]
pub struct Imported {
    /// The scene, in mm.
    pub scene: Scene,
    /// What was found on the way.
    pub report: ImportReport,
}

/// Header-only inspection result (no geometry was read).
#[derive(Clone, Debug)]
pub struct ProbeInfo {
    /// Detected format.
    pub format: Format,
    /// Unit the file declares (filled by the format's reader WP; `None` until then).
    pub declared_unit: Option<LengthUnit>,
    /// Object count, when cheaply known.
    pub objects: Option<u32>,
    /// Triangle count estimate, when cheaply known (binary STL: from the file size).
    pub triangles_estimate: Option<u64>,
    /// A 3MF that carries slicer project data (Orca/Bambu/Prusa parts).
    pub has_slicer_data: bool,
}

/// Header-only inspection: format, and what is cheap to know about the content.
///
/// Zip containers are told apart by their central directory: a 3MF has `[Content_Types].xml`
/// and a `3D/` part; an Amalgam `.amg` project is not an import and gives
/// [`IoError::UnknownFormat`] (whose hint points to File > Open).
///
/// # Errors
/// [`IoError::Read`], [`IoError::UnknownFormat`], or [`IoError::LimitExceeded`] for zips with
/// too many entries.
pub fn probe(src: Source<'_>, limits: &ReadLimits) -> Result<ProbeInfo, IoError> {
    let (head, len) = src.head(format::SNIFF_HEAD)?;
    let mut info = ProbeInfo {
        format: Format::Stl,
        declared_unit: None,
        objects: None,
        triangles_estimate: None,
        has_slicer_data: false,
    };
    if head.starts_with(b"PK\x03\x04") {
        let Some(names) = zipdir::entry_names(&src, limits)? else {
            return Err(IoError::UnknownFormat);
        };
        if zipdir::classify(&names) != zipdir::ZipKind::ThreeMf {
            return Err(IoError::UnknownFormat);
        }
        info.format = Format::ThreeMf;
        info.has_slicer_data = zipdir::has_slicer_parts(&names);
        return Ok(info);
    }
    info.format = Format::sniff(&head, len).ok_or(IoError::UnknownFormat)?;
    if info.format == Format::Stl
        && let Ok(stl::StlKind::Binary { .. }) = stl::detect(&head, len)
    {
        info.triangles_estimate = Some((len - 84) / 50);
    }
    Ok(info)
}

/// Reads any mesh format. `format = None` sniffs the content. Runs inside a job.
///
/// The format readers are IO-02 (STL), IO-03 (3MF), IO-05 (OBJ) and IO-13 (PLY); until they
/// land this resolves the format and answers [`IoError::Unsupported`].
///
/// # Errors
/// See [`probe`]; [`IoError::Unsupported`] for every format for now, and for B-rep and 2D
/// formats, which have their own entry points.
pub fn import(
    src: Source<'_>,
    format: Option<Format>,
    opts: &ImportOptions,
    ctx: &JobCtx,
) -> Result<Imported, IoError> {
    ctx.checkpoint()?;
    let sniffed = format.is_none();
    let format = match format {
        Some(f) => f,
        None => probe(src, &opts.limits)?.format,
    };
    match format {
        Format::Stl => return import_stl(src, sniffed, opts, ctx),
        Format::Obj => return import_obj(src, sniffed, opts, ctx),
        Format::ThreeMf => return import_3mf(src, sniffed, opts, ctx),
        _ => {}
    }
    Err(IoError::Unsupported {
        format,
        direction: "import",
        reason: match format.info().kind {
            GeometryKind::Mesh => "this reader has not been implemented yet",
            GeometryKind::Brep => "B-rep files are read with the brep module",
            GeometryKind::Curves2d => "2D files are read by the sketch importer",
        },
    })
}

/// STL: reads, then applies the unit, `merge_objects` and the up axis (S12 §4).
fn import_stl(
    src: Source<'_>,
    sniffed: bool,
    opts: &ImportOptions,
    ctx: &JobCtx,
) -> Result<Imported, IoError> {
    let out = stl::read(src, opts, ctx)?;
    finish_import(out, src, sniffed, Format::Stl, opts, ctx)
}

/// OBJ: reads, then applies the unit, `merge_objects` and the up axis (S12 §4).
fn import_obj(
    src: Source<'_>,
    sniffed: bool,
    opts: &ImportOptions,
    ctx: &JobCtx,
) -> Result<Imported, IoError> {
    let out = obj::read(src, opts, ctx)?;
    finish_import(out, src, sniffed, Format::Obj, opts, ctx)
}

/// 3MF: reads, then applies the declared (or chosen) unit, `merge_objects` and the up axis
/// on top of the build transforms.
fn import_3mf(
    src: Source<'_>,
    sniffed: bool,
    opts: &ImportOptions,
    ctx: &JobCtx,
) -> Result<Imported, IoError> {
    let mut out = threemf::read(src, opts, ctx)?;
    let declared = out.report.declared_unit;
    let unit = opts.unit.resolve(declared);
    out.report.applied_unit = Some(unit);
    if sniffed
        && let Source::Path(p) = src
        && let Some(ext) = p.extension().and_then(|e| e.to_str())
        && let Some(w) = check_extension(ext, Format::ThreeMf)
    {
        out.report.warnings.push(w);
    }
    let scene = &mut out.scene;
    scene.rescale(unit, LengthUnit::Millimetre);
    if opts.merge_objects && scene.instances().len() > 1 {
        let name = scene.metadata.get("Title").cloned();
        let (mesh, slots) = scene.flatten(ctx)?;
        let mut o = SceneObject::new(std::sync::Arc::new(mesh));
        o.name = name;
        o.face_materials = slots.map(std::sync::Arc::from);
        scene.objects = vec![o];
        scene.roots = vec![SceneNode::mesh(0, amg_core::Transform::IDENTITY)];
        scene.plates.clear();
    }
    let up = opts.up_axis.to_z_up();
    for r in &mut scene.roots {
        r.transform = up * r.transform;
    }
    Ok(out)
}

/// What every unit-less mesh format shares after reading.
fn finish_import(
    mut out: Imported,
    src: Source<'_>,
    sniffed: bool,
    format: Format,
    opts: &ImportOptions,
    ctx: &JobCtx,
) -> Result<Imported, IoError> {
    // The format declares no unit.
    let unit = opts.unit.resolve(None);
    out.report.applied_unit = Some(unit);
    if let UnitChoice::FromFile { fallback } = opts.unit
        && fallback != LengthUnit::Millimetre
    {
        out.report.warnings.push(IoWarning::UnitAssumed(fallback));
    }
    if sniffed
        && let Source::Path(p) = src
        && let Some(ext) = p.extension().and_then(|e| e.to_str())
        && let Some(w) = check_extension(ext, format)
    {
        out.report.warnings.push(w);
    }
    let scene = &mut out.scene;
    scene.rescale(unit, LengthUnit::Millimetre);
    if opts.merge_objects && scene.objects.len() > 1 {
        let name = scene.objects[0].name.clone();
        let (mesh, slots) = scene.flatten(ctx)?;
        let mut o = SceneObject::new(std::sync::Arc::new(mesh));
        o.name = name;
        o.face_materials = slots.map(std::sync::Arc::from);
        scene.objects = vec![o];
        scene.roots = vec![SceneNode::mesh(0, amg_core::Transform::IDENTITY)];
    }
    let up = opts.up_axis.to_z_up();
    for r in &mut scene.roots {
        r.transform = up;
    }
    Ok(out)
}

/// Validity of every placed object for `format` (MESH-02 analysis).
///
/// # Errors
/// Any [`Scene::validate`] error, [`IoError::Unsupported`] for formats without a mesh writer,
/// or [`IoError::Cancelled`].
pub fn check_export(scene: &Scene, format: Format, ctx: &JobCtx) -> Result<ExportCheck, IoError> {
    if format.info().kind != GeometryKind::Mesh {
        return Err(IoError::Unsupported {
            format,
            direction: "export",
            reason: "only mesh formats are written from a scene",
        });
    }
    scene.validate()?;
    let mut seen = std::collections::BTreeSet::new();
    let mut check = ExportCheck::default();
    for (i, _) in scene.instances() {
        if !seen.insert(i) {
            continue;
        }
        ctx.checkpoint()?;
        let o = &scene.objects[i as usize];
        let a = analyze(&o.mesh, &AnalyzeOpts::default(), ctx).map_err(|e| match e {
            amg_mesh::MeshError::Cancelled => IoError::Cancelled,
            e => IoError::Mesh(e),
        })?;
        let oc = ObjectCheck {
            object: i,
            name: o.name.clone(),
            boundary_edges: a.boundary_edges as u64,
            non_manifold_edges: a.non_manifold_edges.len() as u64,
            inconsistent_orientation: !a.is_oriented,
            negative_volume: a.is_inverted,
        };
        if !oc.is_valid() {
            check.warnings.push(IoWarning::NotManifold {
                object: o.name.clone().unwrap_or_else(|| format!("#{i}")),
                boundary_edges: oc.boundary_edges,
                non_manifold_edges: oc.non_manifold_edges,
            });
        }
        check.objects.push(oc);
    }
    Ok(check)
}

/// Writes `scene`. Runs [`check_export`] per `opts.validity` first. Atomic for
/// [`Sink::Path`].
///
/// The writers are IO-02 (STL), IO-11 (3MF), IO-12/IO-04 (flavours) and IO-05/IO-13 (OBJ,
/// PLY); until they land this validates the scene and answers [`IoError::Unsupported`].
///
/// # Errors
/// Any [`Scene::validate`] error, [`IoError::Unsupported`], or [`IoError::Cancelled`].
pub fn export(
    scene: &Scene,
    dst: Sink<'_>,
    format: Format,
    opts: &ExportOptions,
    ctx: &JobCtx,
) -> Result<ExportReport, IoError> {
    ctx.checkpoint()?;
    scene.validate()?;
    match format {
        Format::Stl => return stl::write(scene, dst, opts, ctx),
        Format::Obj => return obj::write(scene, dst, opts, ctx),
        _ => {}
    }
    Err(IoError::Unsupported {
        format,
        direction: "export",
        reason: "this writer has not been implemented yet",
    })
}
