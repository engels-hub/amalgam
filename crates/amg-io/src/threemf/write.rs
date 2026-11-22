//! 3MF writer, Generic flavour (IO-11; spec S12 §6, note `import-export-formats.md` §2-§3).
//!
//! - **Package:** `[Content_Types].xml`, `_rels/.rels`, `/3D/3dmodel.model` and, with a
//!   thumbnail, `/Metadata/thumbnail.png` (package thumbnail relationship), in that order. Every
//!   entry has the timestamp 1980-01-01 00:00, Unix permissions 0644 and the chosen
//!   [`Compression`]; ZIP64 is used only when the size bound of the model part can exceed
//!   4 GiB. The bytes depend only on the scene and the options (not on the thread count).
//! - **Model:** core 1.4, `unit` from [`ExportOptions::unit`], `xml:lang="en-US"`,
//!   `Application = Amalgam-<version>` plus the core metadata of the scene. Mesh objects are
//!   shared between their placements. A scene with groups becomes components (one components
//!   object per group node); otherwise every placement is a build item of its mesh object.
//! - **Materials:** one `<basematerials>` with the scene's slots in order; object `pid/pindex`
//!   for the default slot, triangle `pid/p1` only where a face's slot differs.
//! - **Numbers:** vertices per [`CoordPrecision`]; transforms always in shortest round-trip
//!   form (so placements read back within 1e-12); `-0` is written as `0` and non-finite values
//!   are refused before anything is written.
//! - **Production UUIDs:** build, items, objects and components carry `p:UUID`s derived from a
//!   hash of the content (deterministic, unique within the file). The Production extension is
//!   not required (`requiredextensions` is not written); the Consortium consolidated schema
//!   expects the UUIDs, and Orca/Bambu use them.
//! - **Slicer filament:** [`SceneObject::filament`] has no core 3MF form, so it is written as
//!   object metadata `amg:filament` in Amalgam's own namespace, which other consumers ignore and
//!   the reader reads back.
//!
//! [`SceneObject::filament`]: crate::SceneObject::filament

use std::collections::{BTreeSet, HashMap};
use std::io::{Cursor, Write};
use std::ops::Range;
use std::path::PathBuf;
use std::time::Instant;

use amg_core::Transform;
use amg_core::units::LengthUnit;
use amg_jobs::JobCtx;
use rayon::prelude::*;
use zip::write::SimpleFileOptions;

use super::thumb;
use super::xml::{NS_AMG, NS_CORE, NS_MAT, NS_PROD};
use crate::endpoint::Sink;
use crate::error::{ExportCheck, ExportReport, IoError, IoWarning};
use crate::format::Format;
use crate::obj::{num, push_u64, roundtrip};
use crate::options::{Compression, CoordPrecision, ExportOptions, SlicerFlavor, ValidityPolicy};
use crate::scene::{NO_SLOT, NodeContent, ObjectKind, Scene, SceneNode, SlotIndex};
use crate::units::scale_from_mm;

mod orca;

/// Vertices or triangles per serialisation block (the unit of parallel work; fixed, so the
/// output does not depend on the thread count).
const BLOCK: usize = 1 << 14;
/// Largest rounding error (mm) of `Decimals(n)` that is not worth a warning.
const PRECISION_WARN_MM: f64 = 1e-5;
/// Core metadata names (3MF core 1.4 §4.1), in their canonical spelling.
const CORE_METADATA: [&str; 9] = [
    "Title",
    "Designer",
    "Description",
    "Copyright",
    "LicenseTerms",
    "Rating",
    "CreationDate",
    "ModificationDate",
    "Application",
];
/// Above this bound on the model part's size the entry is written as ZIP64.
const ZIP64_BOUND: u64 = 0xFFFF_FFFF - (1 << 24);

const CT_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
 <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
 <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
"#;
const RELS_HEAD: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
"#;
const REL_THUMB: &str = r#" <Relationship Target="/Metadata/thumbnail.png" Id="rel1" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail"/>
"#;

/// The `Application` metadata value.
pub fn application() -> String {
    format!("Amalgam-{}", env!("CARGO_PKG_VERSION"))
}

/// One written mesh object.
struct MeshOut {
    /// Scene object index.
    object: u32,
    /// 3MF resource id.
    id: u32,
    /// Default base index (object `pindex`), if any.
    default: Option<u32>,
    /// Resource id of the property group of `default` and of the triangle `pid`s (1 in the
    /// Generic flavour; the object's filament number in the Orca flavour).
    pid: u32,
    /// Orca flavour: the object's filament (1-based), 0 in the Generic flavour.
    fil: u8,
}

/// One written components object (a scene group node).
struct GroupOut {
    id: u32,
    name: Option<String>,
    /// (resource id, transform) per child.
    children: Vec<(u32, Transform)>,
}

/// Everything decided before the XML is written.
struct Plan {
    meshes: Vec<MeshOut>,
    groups: Vec<GroupOut>,
    items: Vec<(u32, Transform)>,
    /// Write `<basematerials id="1">`.
    materials: bool,
    /// Index of the synthetic "Default" base (objects with face slots but no default slot).
    synthetic_default: Option<u32>,
    filament: bool,
    triangles: u64,
    /// Seed of the Production `p:UUID`s (a hash of the written content).
    uuid_seed: u64,
    /// Scene root index of every build item (parallel to `items`).
    item_root: Vec<u32>,
    /// Orca/Bambu flavour data (`None` = Generic).
    orca: Option<orca::OrcaPlan>,
}

/// Writes `scene` as a 3MF package (Generic, or Orca/Bambu flavour).
///
/// # Errors
/// [`IoError::Unsupported`] for the Prusa flavour (IO-12),
/// [`IoError::InvalidScene`] (inconsistent scene, non-finite numbers, bad thumbnail),
/// [`IoError::Empty`], [`IoError::InvalidForFormat`] (policy `Require`),
/// [`IoError::LimitExceeded`], [`IoError::Write`] or [`IoError::Cancelled`].
pub fn write(
    scene: &Scene,
    dst: Sink<'_>,
    opts: &ExportOptions,
    ctx: &JobCtx,
) -> Result<ExportReport, IoError> {
    let t0 = Instant::now();
    ctx.checkpoint()?;
    if opts.threemf.flavor == SlicerFlavor::Prusa {
        return Err(IoError::Unsupported {
            format: Format::ThreeMf,
            direction: "export",
            reason: "the Prusa 3MF flavour is not implemented yet (IO-12)",
        });
    }
    scene.validate()?;
    let mut report = ExportReport::default();
    let instances = scene.instances();
    if instances
        .iter()
        .all(|(i, _)| scene.objects[*i as usize].mesh.tris.is_empty())
    {
        return Err(IoError::Empty);
    }

    let unit = opts.unit;
    check_finite(scene, unit)?;

    // Validity gate (S12 §6): every policy runs the check; `Require` refuses, the others warn.
    let check = crate::check_export(scene, Format::ThreeMf, ctx)?;
    if opts.validity_for(Format::ThreeMf) == ValidityPolicy::Require && !check.is_valid() {
        return Err(invalid(check));
    }
    report.warnings.extend(check.warnings);

    if let CoordPrecision::Decimals(d) = opts.precision {
        let step_mm =
            0.5 * 10f64.powi(-i32::from(d.min(30))) * crate::units::scale_to_mm(1.0, unit);
        if step_mm > PRECISION_WARN_MM {
            report.warnings.push(IoWarning::PrecisionLoss {
                max_error_mm: step_mm,
            });
        }
    }

    let mut plan = plan(scene, opts, &mut report.warnings)?;
    plan.uuid_seed = content_hash(scene, &plan, opts);
    let thumbnail = match &scene.thumbnail {
        Some(t) => thumb::png_bytes(t, opts.threemf.thumbnail_max_px, &mut report.warnings)?,
        None => None,
    };

    // ---- Package, in memory (the zip writer needs `Seek`; the sink only `Write`s).
    let zerr = |e: zip::result::ZipError| IoError::Write {
        path: PathBuf::new(),
        source: std::io::Error::other(e),
    };
    let ioerr = |source| IoError::Write {
        path: PathBuf::new(),
        source,
    };
    let base = SimpleFileOptions::default()
        .last_modified_time(zip::DateTime::default())
        .system(zip::System::Unix)
        .unix_permissions(0o644);
    let base = match opts.threemf.compression {
        Compression::Stored => base.compression_method(zip::CompressionMethod::Stored),
        Compression::Deflate { level } => base
            .compression_method(zip::CompressionMethod::Deflated)
            .compression_level(Some(i64::from(level.clamp(1, 9)))),
    };
    let mut z = zip::ZipWriter::new(Cursor::new(Vec::new()));

    z.start_file("[Content_Types].xml", base).map_err(zerr)?;
    let mut ct = String::from(CT_XML);
    if thumbnail.is_some() {
        ct.push_str(" <Default Extension=\"png\" ContentType=\"image/png\"/>\n");
    }
    if plan.orca.is_some() {
        ct.push_str(" <Default Extension=\"config\" ContentType=\"text/xml\"/>\n");
    }
    ct.push_str("</Types>\n");
    z.write_all(ct.as_bytes()).map_err(ioerr)?;

    z.start_file("_rels/.rels", base).map_err(zerr)?;
    let mut rels = String::from(RELS_HEAD);
    if thumbnail.is_some() {
        rels.push_str(REL_THUMB);
    }
    rels.push_str("</Relationships>\n");
    z.write_all(rels.as_bytes()).map_err(ioerr)?;

    let large = model_size_bound(scene, &plan, opts.precision) > ZIP64_BOUND;
    z.start_file("3D/3dmodel.model", base.large_file(large))
        .map_err(zerr)?;
    write_model(&mut z, scene, &plan, opts, ctx, &mut report.warnings)?;

    if plan.orca.is_some() {
        z.start_file("Metadata/model_settings.config", base)
            .map_err(zerr)?;
        z.write_all(&orca::model_settings(scene, &plan))
            .map_err(ioerr)?;
    }
    if let Some(png) = &thumbnail {
        z.start_file("Metadata/thumbnail.png", base).map_err(zerr)?;
        z.write_all(png).map_err(ioerr)?;
    }
    let bytes = z.finish().map_err(zerr)?.into_inner();
    ctx.checkpoint()?;

    let path = match &dst {
        Sink::Path(p) => (*p).to_owned(),
        Sink::Memory(_) => PathBuf::new(),
    };
    let written = dst.write(|w| {
        w.write_all(&bytes).map_err(|source| IoError::Write {
            path: path.clone(),
            source,
        })
    })?;
    report.bytes = written;
    report.triangles = plan.triangles;
    report.files.push(path);
    ctx.progress(1.0);
    report.duration = t0.elapsed();
    Ok(report)
}

fn invalid(check: ExportCheck) -> IoError {
    IoError::InvalidForFormat {
        format: Format::ThreeMf,
        count: check.objects.iter().filter(|o| !o.is_valid()).count(),
        check: Box::new(check),
    }
}

/// Refuses non-finite coordinates and transforms (after the unit change).
fn check_finite(scene: &Scene, unit: LengthUnit) -> Result<(), IoError> {
    let used: BTreeSet<u32> = scene.instances().into_iter().map(|(i, _)| i).collect();
    for i in used {
        let m = &scene.objects[i as usize].mesh;
        let bad = m.positions.par_iter().any(|q| {
            q.to_array()
                .iter()
                .any(|&x| !scale_from_mm(x, unit).is_finite())
        });
        if bad {
            return Err(IoError::InvalidScene {
                message: format!("object #{i} has a vertex with a non-finite coordinate"),
            });
        }
    }
    let mut stack: Vec<&SceneNode> = scene.roots.iter().collect();
    while let Some(n) = stack.pop() {
        if !transform_numbers(&n.transform, unit)
            .iter()
            .all(|x| x.is_finite())
        {
            return Err(IoError::InvalidScene {
                message: "a placement transform has a non-finite number".into(),
            });
        }
        if let NodeContent::Group(c) = &n.content {
            stack.extend(c.iter());
        }
    }
    Ok(())
}

/// Resource ids, build items and material use.
fn plan(
    scene: &Scene,
    opts: &ExportOptions,
    warnings: &mut Vec<IoWarning>,
) -> Result<Plan, IoError> {
    let placeable = |i: u32| {
        let o = &scene.objects[i as usize];
        o.kind != ObjectKind::Other && !o.mesh.tris.is_empty()
    };
    // Mesh objects in first-use order.
    let mut order: Vec<u32> = Vec::new();
    let mut seen: BTreeSet<u32> = BTreeSet::new();
    let mut skipped_other = 0u64;
    let mut skipped_empty = 0u64;
    let mut triangles = 0u64;
    for (i, _) in scene.instances() {
        if !placeable(i) {
            if scene.objects[i as usize].kind == ObjectKind::Other {
                skipped_other += 1;
            }
            continue;
        }
        triangles += scene.objects[i as usize].mesh.face_count() as u64;
        if seen.insert(i) {
            order.push(i);
        }
    }
    for o in &order {
        let n = scene.objects[*o as usize].mesh.face_count();
        if n > u32::MAX as usize {
            return Err(IoError::LimitExceeded {
                what: "triangles per object",
                limit: u64::from(u32::MAX),
                actual: n as u64,
            });
        }
    }

    // Materials.
    let orca_plan = (opts.threemf.flavor == SlicerFlavor::Orca && opts.threemf.write_materials)
        .then(|| orca::plan_filaments(scene, &order, warnings));
    let slot_count = scene.materials.len() as u32;
    let uses_slots = order.iter().any(|&i| {
        let o = &scene.objects[i as usize];
        o.material.is_some_and(|s| s != NO_SLOT)
            || o.face_materials
                .as_ref()
                .is_some_and(|f| f.iter().any(|&s| s != NO_SLOT))
    });
    let materials = match &orca_plan {
        Some(o) => o.groups() > 0,
        None => opts.threemf.write_materials && uses_slots && slot_count > 0,
    };
    let mut synthetic_default = None;
    let mut meshes = Vec::with_capacity(order.len());
    // Generic: one `basematerials` (id 1); Orca: one colour group per filament (ids 1..=N).
    let first_mesh_id = match &orca_plan {
        Some(o) => 1 + o.groups(),
        None => 1 + u32::from(materials),
    };
    let mut ids: HashMap<u32, u32> = HashMap::new();
    for (k, &i) in order.iter().enumerate() {
        let o = &scene.objects[i as usize];
        let id = first_mesh_id + k as u32;
        ids.insert(i, id);
        let (default, pid, fil) = if let Some(op) = orca_plan.as_ref().filter(|o| o.groups() > 0) {
            let f = op.object_filament(o);
            let assigned = o.material.is_some_and(|s| s != NO_SLOT)
                || o.filament.is_some()
                || o.face_materials
                    .as_ref()
                    .is_some_and(|fm| fm.iter().any(|&s| s != NO_SLOT));
            (assigned.then_some(0), u32::from(f), f)
        } else if orca_plan.is_some() {
            (None, 1, 1)
        } else {
            let d = if !materials {
                None
            } else {
                match o.material.filter(|&s| s != NO_SLOT) {
                    Some(s) => Some(u32::from(s)),
                    None => {
                        let has_faces = o
                            .face_materials
                            .as_ref()
                            .is_some_and(|f| f.iter().any(|&s| s != NO_SLOT));
                        if has_faces {
                            // Triangles with properties need an object default; faces without
                            // a slot get a neutral "Default" base.
                            Some(*synthetic_default.get_or_insert(slot_count))
                        } else {
                            None
                        }
                    }
                }
            };
            (d, 1, 0)
        };
        meshes.push(MeshOut {
            object: i,
            id,
            default,
            pid,
            fil,
        });
    }
    // Orca writes the extruder into `model_settings.config` instead.
    let filament = orca_plan.is_none()
        && order
            .iter()
            .any(|&i| scene.objects[i as usize].filament.is_some());

    // Groups (post-order, children first) and build items.
    let mut next_id = first_mesh_id + order.len() as u32;
    let mut groups = Vec::new();
    let mut items = Vec::new();
    let mut item_root = Vec::new();
    for (ri, r) in scene.roots.iter().enumerate() {
        if let Some(id) = node_id(
            r,
            &ids,
            &placeable,
            &mut next_id,
            &mut groups,
            &mut skipped_empty,
        ) {
            items.push((id, r.transform));
            item_root.push(ri as u32);
        }
    }
    if skipped_other > 0 {
        warnings.push(IoWarning::UnsupportedContentIgnored {
            what: "placements of objects of type other (3MF cannot place them)".into(),
            count: skipped_other,
        });
    }
    if skipped_empty > 0 {
        warnings.push(IoWarning::UnsupportedContentIgnored {
            what: "empty groups".into(),
            count: skipped_empty,
        });
    }
    if items.is_empty() {
        return Err(IoError::Empty);
    }
    Ok(Plan {
        meshes,
        groups,
        items,
        materials,
        synthetic_default,
        filament,
        triangles,
        uuid_seed: 0,
        item_root,
        orca: orca_plan,
    })
}

/// The resource id that places `n`: its mesh object, or a new components object for a group
/// (`None` if nothing in it can be placed).
fn node_id(
    n: &SceneNode,
    ids: &HashMap<u32, u32>,
    placeable: &dyn Fn(u32) -> bool,
    next_id: &mut u32,
    groups: &mut Vec<GroupOut>,
    skipped_empty: &mut u64,
) -> Option<u32> {
    match &n.content {
        NodeContent::Mesh(i) => placeable(*i).then(|| ids[i]),
        NodeContent::Group(children) => {
            let mut out = Vec::with_capacity(children.len());
            for c in children {
                if let Some(id) = node_id(c, ids, placeable, next_id, groups, skipped_empty) {
                    out.push((id, c.transform));
                }
            }
            if out.is_empty() {
                *skipped_empty += 1;
                return None;
            }
            let id = *next_id;
            *next_id += 1;
            groups.push(GroupOut {
                id,
                name: n.name.clone(),
                children: out,
            });
            Some(id)
        }
    }
}

/// Upper bound of the model part's size (decides ZIP64 before anything is written).
fn model_size_bound(scene: &Scene, plan: &Plan, prec: CoordPrecision) -> u64 {
    let num_len: u64 = match prec {
        CoordPrecision::Decimals(d) => 20 + u64::from(d),
        CoordPrecision::Roundtrip => 26,
    };
    let mut b: u64 = 1 << 20;
    for m in &plan.meshes {
        let mesh = &scene.objects[m.object as usize].mesh;
        b += mesh.vertex_count() as u64 * (32 + 3 * num_len);
        b += mesh.face_count() as u64 * if plan.orca.is_some() { 150 } else { 100 };
    }
    b += (plan.groups.iter().map(|g| g.children.len()).sum::<usize>() + plan.items.len()) as u64
        * (64 + 12 * 26);
    b
}

/// The 12 numbers of a 3MF `ST_Matrix3D` (row-vector convention), translation in `unit`.
fn transform_numbers(t: &Transform, unit: LengthUnit) -> [f64; 12] {
    let (x, y, z) = (t.matrix3.x_axis, t.matrix3.y_axis, t.matrix3.z_axis);
    [
        x.x,
        x.y,
        x.z,
        y.x,
        y.y,
        y.z,
        z.x,
        z.y,
        z.z,
        scale_from_mm(t.translation.x, unit),
        scale_from_mm(t.translation.y, unit),
        scale_from_mm(t.translation.z, unit),
    ]
}

/// ` transform="…"` (nothing for the identity).
fn transform_attr(out: &mut Vec<u8>, t: &Transform, unit: LengthUnit) {
    if *t == Transform::IDENTITY {
        return;
    }
    out.extend_from_slice(b" transform=\"");
    for (k, v) in transform_numbers(t, unit).iter().enumerate() {
        if k > 0 {
            out.push(b' ');
        }
        roundtrip(out, *v);
    }
    out.push(b'"');
}

/// XML attribute / text escaping; characters XML 1.0 cannot carry are dropped.
pub(crate) fn escape(out: &mut Vec<u8>, s: &str) {
    for c in s.chars() {
        match c {
            '&' => out.extend_from_slice(b"&amp;"),
            '<' => out.extend_from_slice(b"&lt;"),
            '>' => out.extend_from_slice(b"&gt;"),
            '"' => out.extend_from_slice(b"&quot;"),
            '\'' => out.extend_from_slice(b"&apos;"),
            '\t' => out.extend_from_slice(b"&#9;"),
            '\n' => out.extend_from_slice(b"&#10;"),
            '\r' => out.extend_from_slice(b"&#13;"),
            c if (c as u32) < 0x20 || matches!(c, '\u{FFFE}' | '\u{FFFF}') => {}
            c => {
                let mut b = [0u8; 4];
                out.extend_from_slice(c.encode_utf8(&mut b).as_bytes());
            }
        }
    }
}

fn unit_name(u: LengthUnit) -> &'static str {
    match u {
        LengthUnit::Micrometre => "micron",
        LengthUnit::Millimetre => "millimeter",
        LengthUnit::Centimetre => "centimeter",
        LengthUnit::Metre => "meter",
        LengthUnit::Inch => "inch",
        LengthUnit::Foot => "foot",
    }
}

fn color_hex(out: &mut Vec<u8>, c: amg_mesh::Rgba8) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    out.push(b'#');
    for b in c.0 {
        out.push(HEX[usize::from(b >> 4)]);
        out.push(HEX[usize::from(b & 15)]);
    }
}

/// Writes the model part.
fn write_model<W: Write + Send>(
    w: &mut W,
    scene: &Scene,
    plan: &Plan,
    opts: &ExportOptions,
    ctx: &JobCtx,
    warnings: &mut Vec<IoWarning>,
) -> Result<(), IoError> {
    let werr = |source| IoError::Write {
        path: PathBuf::new(),
        source,
    };
    let unit = opts.unit;
    let mut h: Vec<u8> = Vec::with_capacity(4096);
    h.extend_from_slice(b"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<model unit=\"");
    h.extend_from_slice(unit_name(unit).as_bytes());
    h.extend_from_slice(b"\" xml:lang=\"en-US\" xmlns=\"");
    h.extend_from_slice(NS_CORE.as_bytes());
    h.extend_from_slice(b"\" xmlns:p=\"");
    h.extend_from_slice(NS_PROD.as_bytes());
    h.push(b'"');
    if plan.orca.as_ref().is_some_and(|o| o.groups() > 0) {
        h.extend_from_slice(b" xmlns:m=\"");
        h.extend_from_slice(NS_MAT.as_bytes());
        h.push(b'"');
    }
    if plan.filament {
        h.extend_from_slice(b" xmlns:amg=\"");
        h.extend_from_slice(NS_AMG.as_bytes());
        h.push(b'"');
    }
    h.extend_from_slice(b">\n");

    // Metadata: Application is always ours; other core names from the scene.
    let mut meta: Vec<(&'static str, String)> = vec![("Application", application())];
    let mut dropped = 0u64;
    for (k, v) in &scene.metadata {
        match CORE_METADATA.iter().find(|c| c.eq_ignore_ascii_case(k)) {
            Some(&"Application") => {}
            Some(&c) => {
                if !v.is_empty() && !meta.iter().any(|(n, _)| *n == c) {
                    meta.push((c, v.clone()));
                }
            }
            None => dropped += 1,
        }
    }
    meta.sort_by_key(|(n, _)| CORE_METADATA.iter().position(|c| c == n));
    let mut value = Vec::new();
    for (n, v) in &meta {
        // Empty values are not written, also when only characters that XML cannot carry made
        // them non-empty (IO-16 fuzz finding `roundtrip_scene/control-char-metadata`).
        value.clear();
        escape(&mut value, v);
        if value.is_empty() {
            continue;
        }
        h.extend_from_slice(b" <metadata name=\"");
        h.extend_from_slice(n.as_bytes());
        h.extend_from_slice(b"\">");
        h.extend_from_slice(&value);
        h.extend_from_slice(b"</metadata>\n");
    }
    if dropped > 0 {
        warnings.push(IoWarning::UnsupportedContentIgnored {
            what: "non-core metadata (the Generic flavour writes core names only)".into(),
            count: dropped,
        });
    }
    h.extend_from_slice(b" <resources>\n");
    if let Some(o) = &plan.orca {
        // One single-colour group per filament, in filament order: Orca numbers the distinct
        // colours in ascending group-id order and takes an object's extruder from its `pid`.
        for (k, c) in o.colors.iter().enumerate() {
            h.extend_from_slice(b"  <m:colorgroup id=\"");
            push_u64(&mut h, k as u64 + 1);
            h.extend_from_slice(b"\">\n   <m:color color=\"");
            color_hex(&mut h, *c);
            h.extend_from_slice(b"\"/>\n  </m:colorgroup>\n");
        }
    } else if plan.materials {
        h.extend_from_slice(b"  <basematerials id=\"1\">\n");
        for m in &scene.materials {
            h.extend_from_slice(b"   <base name=\"");
            escape(&mut h, &m.name);
            h.extend_from_slice(b"\" displaycolor=\"");
            color_hex(&mut h, m.color);
            h.extend_from_slice(b"\"/>\n");
        }
        if plan.synthetic_default.is_some() {
            h.extend_from_slice(b"   <base name=\"Default\" displaycolor=\"#808080FF\"/>\n");
        }
        h.extend_from_slice(b"  </basematerials>\n");
    }
    w.write_all(&h).map_err(werr)?;

    let total: usize = plan
        .meshes
        .iter()
        .map(|m| {
            let mesh = &scene.objects[m.object as usize].mesh;
            mesh.vertex_count() + mesh.face_count()
        })
        .sum::<usize>()
        .max(1);
    let mut done = 0usize;
    for m in &plan.meshes {
        ctx.checkpoint()?;
        let o = &scene.objects[m.object as usize];
        let mesh = &*o.mesh;
        let mut s: Vec<u8> = Vec::with_capacity(256);
        s.extend_from_slice(b"  <object id=\"");
        push_u64(&mut s, u64::from(m.id));
        s.extend_from_slice(b"\" type=\"");
        s.extend_from_slice(match o.kind {
            ObjectKind::Model | ObjectKind::Other => b"model".as_slice(),
            ObjectKind::SolidSupport => b"solidsupport",
            ObjectKind::Support => b"support",
            ObjectKind::Surface => b"surface",
        });
        s.push(b'"');
        if let Some(n) = &o.name {
            s.extend_from_slice(b" name=\"");
            escape(&mut s, n);
            s.push(b'"');
        }
        if let Some(d) = m.default {
            s.extend_from_slice(b" pid=\"");
            push_u64(&mut s, u64::from(m.pid));
            s.extend_from_slice(b"\" pindex=\"");
            push_u64(&mut s, u64::from(d));
            s.push(b'"');
        }
        uuid_attr(&mut s, plan.uuid_seed, TAG_OBJECT, u64::from(m.id));
        s.extend_from_slice(b">\n");
        if let Some(f) = o.filament.filter(|_| plan.orca.is_none()) {
            s.extend_from_slice(b"   <metadatagroup>\n    <metadata name=\"amg:filament\">");
            push_u64(&mut s, u64::from(f));
            s.extend_from_slice(b"</metadata>\n   </metadatagroup>\n");
        }
        s.extend_from_slice(b"   <mesh>\n    <vertices>\n");
        w.write_all(&s).map_err(werr)?;

        let prec = opts.precision;
        stream_blocks(
            w,
            mesh.vertex_count(),
            ctx,
            &mut |r| {
                done += r.len();
                ctx.progress(0.95 * done as f32 / total as f32);
            },
            &|r| {
                let mut out = Vec::with_capacity(r.len() * 64);
                for p in &mesh.positions[r] {
                    out.extend_from_slice(b"     <vertex x=\"");
                    num(&mut out, scale_from_mm(p.x, unit), prec);
                    out.extend_from_slice(b"\" y=\"");
                    num(&mut out, scale_from_mm(p.y, unit), prec);
                    out.extend_from_slice(b"\" z=\"");
                    num(&mut out, scale_from_mm(p.z, unit), prec);
                    out.extend_from_slice(b"\"/>\n");
                }
                out
            },
        )?;
        w.write_all(b"    </vertices>\n    <triangles>\n")
            .map_err(werr)?;
        let faces: Option<&[SlotIndex]> = if m.default.is_some() {
            o.face_materials.as_deref()
        } else {
            None
        };
        let default = m.default;
        let slot_fil: Option<(&[u8], u8)> = plan.orca.as_ref().map(|o| (&o.slot_fil[..], m.fil));
        stream_blocks(
            w,
            mesh.face_count(),
            ctx,
            &mut |r| {
                done += r.len();
                ctx.progress(0.95 * done as f32 / total as f32);
            },
            &|r| {
                let mut out = Vec::with_capacity(r.len() * 48);
                for f in r {
                    let t = mesh.tris[f];
                    out.extend_from_slice(b"     <triangle v1=\"");
                    push_u64(&mut out, u64::from(t[0]));
                    out.extend_from_slice(b"\" v2=\"");
                    push_u64(&mut out, u64::from(t[1]));
                    out.extend_from_slice(b"\" v3=\"");
                    push_u64(&mut out, u64::from(t[2]));
                    out.push(b'"');
                    // A face keeps the object default (real or synthetic) unless its own slot
                    // differs.
                    if let Some((slot_fil, obj_fil)) = slot_fil {
                        // Orca: the face's filament as `paint_color` (what Orca reads) and as
                        // a triangle property into its colour group (what other readers see).
                        if let Some(f) = faces
                            .map(|fs| fs[f])
                            .filter(|&s| s != NO_SLOT)
                            .map(|s| slot_fil[usize::from(s)])
                            .filter(|&f| f != obj_fil)
                        {
                            out.extend_from_slice(b" pid=\"");
                            push_u64(&mut out, u64::from(f));
                            out.extend_from_slice(b"\" p1=\"0\" paint_color=\"");
                            out.extend_from_slice(crate::paint::encode_state(f).as_bytes());
                            out.push(b'"');
                        }
                    } else if let Some(s) = faces.map(|fs| fs[f]).filter(|&s| s != NO_SLOT)
                        && Some(u32::from(s)) != default
                    {
                        out.extend_from_slice(b" pid=\"1\" p1=\"");
                        push_u64(&mut out, u64::from(s));
                        out.push(b'"');
                    }
                    out.extend_from_slice(b"/>\n");
                }
                out
            },
        )?;
        w.write_all(b"    </triangles>\n   </mesh>\n  </object>\n")
            .map_err(werr)?;
    }

    let mut s: Vec<u8> = Vec::new();
    for g in &plan.groups {
        s.extend_from_slice(b"  <object id=\"");
        push_u64(&mut s, u64::from(g.id));
        s.extend_from_slice(b"\" type=\"model\"");
        if let Some(n) = &g.name {
            s.extend_from_slice(b" name=\"");
            escape(&mut s, n);
            s.push(b'"');
        }
        uuid_attr(&mut s, plan.uuid_seed, TAG_OBJECT, u64::from(g.id));
        s.extend_from_slice(b">\n   <components>\n");
        for (k, (id, t)) in g.children.iter().enumerate() {
            s.extend_from_slice(b"    <component objectid=\"");
            push_u64(&mut s, u64::from(*id));
            s.push(b'"');
            let n = (u64::from(g.id) << 32) | k as u64;
            uuid_attr(&mut s, plan.uuid_seed, TAG_COMPONENT, n);
            transform_attr(&mut s, t, unit);
            s.extend_from_slice(b"/>\n");
        }
        s.extend_from_slice(b"   </components>\n  </object>\n");
    }
    s.extend_from_slice(b" </resources>\n <build");
    uuid_attr(&mut s, plan.uuid_seed, TAG_BUILD, 0);
    s.extend_from_slice(b">\n");
    for (k, (id, t)) in plan.items.iter().enumerate() {
        s.extend_from_slice(b"  <item objectid=\"");
        push_u64(&mut s, u64::from(*id));
        s.push(b'"');
        uuid_attr(&mut s, plan.uuid_seed, TAG_ITEM, k as u64);
        transform_attr(&mut s, t, unit);
        s.extend_from_slice(b"/>\n");
    }
    s.extend_from_slice(b" </build>\n</model>\n");
    w.write_all(&s).map_err(werr)?;
    Ok(())
}

const TAG_BUILD: u64 = 1;
const TAG_ITEM: u64 = 2;
const TAG_OBJECT: u64 = 3;
const TAG_COMPONENT: u64 = 4;

fn mix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn fold(h: u64, v: u64) -> u64 {
    mix(h ^ v)
}

fn fold_text(h: u64, s: &str) -> u64 {
    s.bytes()
        .fold(fold(h, s.len() as u64), |h, b| fold(h, u64::from(b)))
}

fn fold_transform(h: u64, id: u32, t: &Transform, unit: LengthUnit) -> u64 {
    transform_numbers(t, unit)
        .iter()
        .fold(fold(h, u64::from(id)), |h, v| fold(h, v.to_bits()))
}

/// Order-dependent 64-bit hash of what is written (positions, triangles, slots, names,
/// transforms, unit), over fixed blocks, so it does not depend on the thread count.
fn content_hash(scene: &Scene, plan: &Plan, opts: &ExportOptions) -> u64 {
    let mut h = fold(0x3A4D_465F_494F_3131, plan.meshes.len() as u64);
    for m in &plan.meshes {
        let o = &scene.objects[m.object as usize];
        let mesh = &o.mesh;
        let pos: Vec<u64> = mesh
            .positions
            .par_chunks(BLOCK)
            .map(|c| {
                c.iter().fold(0, |h, p| {
                    fold(fold(fold(h, p.x.to_bits()), p.y.to_bits()), p.z.to_bits())
                })
            })
            .collect();
        let tri: Vec<u64> = mesh
            .tris
            .par_chunks(BLOCK)
            .map(|c| {
                c.iter().fold(0, |h, t| {
                    fold(
                        h,
                        ((u64::from(t[0]) << 32) | u64::from(t[1])) ^ (u64::from(t[2]) << 17),
                    )
                })
            })
            .collect();
        h = pos
            .iter()
            .chain(&tri)
            .fold(fold(h, u64::from(m.id)), |h, &v| fold(h, v));
        if let Some(n) = &o.name {
            h = fold_text(h, n);
        }
        h = fold(h, m.default.map_or(u64::MAX, u64::from));
        if let Some(f) = &o.face_materials {
            h = f.iter().fold(h, |h, &s| fold(h, u64::from(s)));
        }
    }
    for g in &plan.groups {
        h = fold(h, u64::from(g.id));
        for (id, t) in &g.children {
            h = fold_transform(h, *id, t, opts.unit);
        }
    }
    for (id, t) in &plan.items {
        h = fold_transform(h, *id, t, opts.unit);
    }
    for m in &scene.materials {
        h = fold(
            fold_text(h, &m.name),
            u64::from(u32::from_le_bytes(m.color.0)),
        );
    }
    if let Some(o) = &plan.orca {
        // Orca: object filaments, filament colours (distinct file → distinct UUIDs).
        h = plan
            .meshes
            .iter()
            .fold(fold(h, 0x4F52_4341), |h, m| fold(h, u64::from(m.fil)));
        h = o.slot_fil.iter().fold(h, |h, &f| fold(h, u64::from(f)));
        h = o
            .colors
            .iter()
            .fold(h, |h, c| fold(h, u64::from(u32::from_le_bytes(c.0))));
    }
    fold_text(h, unit_name(opts.unit))
}

/// ` p:UUID="…"`: a version-4-shaped UUID derived from the content hash, the element kind and
/// its index (deterministic, and unique within the file).
fn uuid_attr(out: &mut Vec<u8>, seed: u64, tag: u64, n: u64) {
    let a = mix(seed ^ mix(tag.wrapping_mul(0x0100_0000_01B3) ^ n));
    let b = mix(a ^ tag ^ n.rotate_left(29));
    let a = (a & !0xF000) | 0x4000;
    let b = (b & !(0xC << 60)) | (0x8 << 60);
    let s = format!(
        " p:UUID=\"{:08x}-{:04x}-{:04x}-{:04x}-{:012x}\"",
        a >> 32,
        (a >> 16) & 0xFFFF,
        a & 0xFFFF,
        b >> 48,
        b & 0xFFFF_FFFF_FFFF
    );
    out.extend_from_slice(s.as_bytes());
}

/// Serialises `0..n` in fixed blocks of [`BLOCK`] items, formatting the next group of blocks
/// in parallel while the current one is written (compressed).
fn stream_blocks<W: Write + Send>(
    w: &mut W,
    n: usize,
    ctx: &JobCtx,
    progress: &mut dyn FnMut(Range<usize>),
    f: &(dyn Fn(Range<usize>) -> Vec<u8> + Sync),
) -> Result<(), IoError> {
    let group = BLOCK * 4 * rayon::current_num_threads().max(1);
    let format = |g: Range<usize>| -> Vec<Vec<u8>> {
        g.clone()
            .step_by(BLOCK)
            .collect::<Vec<_>>()
            .into_par_iter()
            .map(|a| f(a..(a + BLOCK).min(g.end)))
            .collect()
    };
    let mut start = 0usize;
    let mut pending: Option<(Range<usize>, Vec<Vec<u8>>)> = None;
    loop {
        ctx.checkpoint()?;
        let next = (start < n).then(|| {
            let end = (start + group).min(n);
            start..end
        });
        if pending.is_none() && next.is_none() {
            return Ok(());
        }
        let to_write = pending.take();
        let (written, formatted) = rayon::join(
            || -> std::io::Result<Option<Range<usize>>> {
                match to_write {
                    Some((r, blocks)) => {
                        for b in &blocks {
                            w.write_all(b)?;
                        }
                        Ok(Some(r))
                    }
                    None => Ok(None),
                }
            },
            || next.map(|g| (g.clone(), format(g))),
        );
        let written = written.map_err(|source| IoError::Write {
            path: PathBuf::new(),
            source,
        })?;
        if let Some(r) = written {
            progress(r);
        }
        if let Some((g, _)) = &formatted {
            start = g.end;
        }
        pending = formatted;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escaping_covers_markup_and_controls() {
        let mut v = Vec::new();
        escape(&mut v, "a<b>&\"c'\t\n\u{1}é");
        assert_eq!(
            String::from_utf8(v).unwrap(),
            "a&lt;b&gt;&amp;&quot;c&apos;&#9;&#10;é"
        );
    }

    #[test]
    fn transforms_use_the_row_vector_layout() {
        let t = Transform::from_cols(
            amg_core::Vec3::new(1.0, 2.0, 3.0),
            amg_core::Vec3::new(4.0, 5.0, 6.0),
            amg_core::Vec3::new(7.0, 8.0, 9.0),
            amg_core::Vec3::new(10.0, -0.0, 0.1),
        );
        let mut v = Vec::new();
        transform_attr(&mut v, &t, LengthUnit::Millimetre);
        assert_eq!(
            String::from_utf8(v).unwrap(),
            " transform=\"1 2 3 4 5 6 7 8 9 10 0 0.1\""
        );
        let mut v = Vec::new();
        transform_attr(&mut v, &Transform::IDENTITY, LengthUnit::Millimetre);
        assert!(v.is_empty());
        let parsed =
            super::super::xml::parse_transform("1 2 3 4 5 6 7 8 9 10 0 0.1", true).unwrap();
        assert_eq!(
            parsed,
            Transform {
                translation: amg_core::Vec3::new(10.0, 0.0, 0.1),
                ..t
            }
        );
    }

    #[test]
    fn colors_are_rrggbbaa() {
        let mut v = Vec::new();
        color_hex(&mut v, amg_mesh::Rgba8([255, 128, 0, 10]));
        assert_eq!(v, b"#FF80000A");
    }
}
