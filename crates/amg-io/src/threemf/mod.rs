//! 3MF reader (IO-03; spec S12 §6, note `import-export-formats.md` §2-§4, §9, §10).
//!
//! - **Package** (`package`): zip with ZIP64 and data descriptors (the `zip` crate),
//!   case-insensitive part names, content types, relationships resolved against their part,
//!   and the size, total, ratio and entry-count limits of [`ReadLimits`](crate::ReadLimits).
//! - **Model** (`model`): core 1.4 resources, components, build items and metadata, the
//!   Production extension's `p:path` multi-part models, base materials and the Materials
//!   extension's colour groups, textures (UVs only), composites and multiproperties
//!   (flattened). `requiredextensions` that we do not read give
//!   [`IoError::UnsupportedRequiredExtension`]; Secure Content gives [`IoError::Encrypted`].
//! - **Hot loop** (`scan`): vertices and triangles are parsed in parallel chunks by a byte
//!   scanner with a quick-xml fallback.
//! - **Slicer extras** (`config`, [`crate::paint`]): `paint_color` / `mmu_segmentation`
//!   become per-face filament slots, model configs give names, extruders, part roles and plates.
//! - **Strict mode** checks the MUSTs the conformance suites test, including closed,
//!   oriented, outward `model` meshes; normal mode loads non-manifold meshes with a warning.
//!
//! The scene keeps the file's numbers (in the root model's unit); [`crate::import`] applies the
//! unit, `merge_objects` and the up axis.

mod build;
mod config;
mod model;
mod package;
mod scan;
mod xml;

use std::collections::{BTreeMap, BTreeSet};
use std::time::Instant;

use amg_core::units::LengthUnit;
use amg_jobs::JobCtx;

use self::build::{Builder, Extras, edge_counts, strict_scene_checks};
use self::model::ModelPart;
use self::package::{CT_MODEL, Package, REL_MODEL, REL_TEXTURE, REL_THUMBNAIL};
use self::xml::{NS_PROD, malformed_part, part_key};
use crate::Imported;
use crate::endpoint::Source;
use crate::error::{ImportReport, IoError, IoWarning};
use crate::format::Format;
use crate::options::ImportOptions;
use crate::scene::{ObjectKind, Thumbnail};

/// Largest thumbnail that is read.
const MAX_THUMBNAIL: u64 = 32 << 20;
/// Largest slicer config part that is read.
const MAX_CONFIG: u64 = 64 << 20;

/// Knobs for tests, benches and fuzzing; [`Tuning::default`] is what [`read`] uses.
#[doc(hidden)]
#[derive(Clone, Debug)]
pub struct Tuning {
    /// Bytes per parallel scanner chunk.
    pub chunk_bytes: usize,
    /// Parse vertices and triangles with the quick-xml reference path only (differential
    /// tests).
    pub reference_only: bool,
}

impl Default for Tuning {
    fn default() -> Self {
        Tuning {
            chunk_bytes: 4 << 20,
            reference_only: false,
        }
    }
}

/// Reads a 3MF package into a [`crate::Scene`] whose numbers are the root model's unit.
///
/// # Errors
/// [`IoError::Malformed`] (with the part and XML line), [`IoError::LimitExceeded`],
/// [`IoError::UnsupportedRequiredExtension`], [`IoError::Encrypted`], [`IoError::Empty`],
/// [`IoError::Read`] or [`IoError::Cancelled`].
pub fn read(src: Source<'_>, opts: &ImportOptions, ctx: &JobCtx) -> Result<Imported, IoError> {
    read_with(src, opts, ctx, &Tuning::default())
}

/// [`read`] with explicit [`Tuning`].
///
/// # Errors
/// As [`read`].
#[doc(hidden)]
pub fn read_with(
    src: Source<'_>,
    opts: &ImportOptions,
    ctx: &JobCtx,
    tuning: &Tuning,
) -> Result<Imported, IoError> {
    let t0 = Instant::now();
    ctx.checkpoint()?;
    let len = src.len()?;
    if len == 0 {
        return Err(IoError::Empty);
    }
    let o = &opts.threemf;
    let strict = o.strict;
    let limits = &opts.limits;
    let mut pkg = Package::open(&src, limits, strict)?;
    let total = pkg.compressed_total.max(1);
    let mut done = 0u64;
    let mut warnings: Vec<IoWarning> = Vec::new();

    // ---- StartPart.
    let starts: Vec<_> = pkg
        .root_rels
        .iter()
        .filter(|r| r.kind == REL_MODEL)
        .cloned()
        .collect();
    if strict {
        if starts.len() > 1 {
            return Err(malformed_part(
                "_rels/.rels",
                "more than one StartPart relationship",
            ));
        }
        if let Some(r) = pkg.root_rels.iter().find(|r| r.external) {
            return Err(malformed_part(
                "_rels/.rels",
                format!("relationship {} points outside the package", r.id),
            ));
        }
        for r in pkg.root_rels.iter().filter(|r| !r.external) {
            if let Some(name) = pkg.entry_name(&r.target)
                && xml::normalize_exact(name) != r.exact
            {
                return Err(malformed_part(
                    "_rels/.rels",
                    format!(
                        "relationship target {} differs in case from the part {name}",
                        r.exact
                    ),
                ));
            }
        }
    }
    let start = starts.iter().find(|r| !r.external).ok_or_else(|| {
        malformed_part(
            "_rels/.rels",
            "the package has no StartPart (3D model) relationship",
        )
    })?;
    let root_key = start.target.clone();
    if !pkg.has(&root_key) {
        return Err(malformed_part(
            "_rels/.rels",
            format!("the StartPart {} does not exist", start.exact),
        ));
    }
    if strict && pkg.content_types.of(&root_key) != Some(CT_MODEL) {
        return Err(malformed_part(
            &pkg.display_name(&root_key),
            "the StartPart is not a 3D model part",
        ));
    }

    // ---- Model parts: the root, then every part reached through p:path.
    let mut parts: BTreeMap<String, ModelPart> = BTreeMap::new();
    let mut totals = (0u64, 0u64);
    let mut queue = vec![root_key.clone()];
    let mut root_rels = None;
    while let Some(key) = queue.pop() {
        if parts.contains_key(&key) {
            continue;
        }
        let is_root = key == root_key;
        let name = pkg.display_name(&key);
        if !pkg.has(&key) {
            return Err(malformed_part(
                &name,
                "the referenced model part does not exist",
            ));
        }
        if strict && !is_root {
            if pkg.content_types.of(&key) != Some(CT_MODEL) {
                return Err(malformed_part(
                    &name,
                    "the referenced part is not a 3D model part",
                ));
            }
            let rels = root_rels.get_or_insert_with(Vec::new);
            if !rels
                .iter()
                .any(|r: &package::Rel| r.kind == REL_MODEL && r.target == key)
            {
                return Err(malformed_part(
                    &name,
                    "the non-root model part has no 3D model relationship from the root model",
                ));
            }
        }
        let base = done;
        let data = pkg.read_part(&key, ctx, &mut |b| {
            ctx.progress(0.5 * ((base + b) as f32 / total as f32).min(1.0));
        })?;
        done += pkg.compressed_size(&key);
        let dlen = data.len().max(1);
        let part = model::parse(
            data,
            &key,
            &name,
            is_root,
            strict,
            limits,
            tuning,
            ctx,
            &mut totals,
            &mut |at| ctx.progress(0.5 + 0.4 * (at as f32 / dlen as f32).min(1.0)),
        )?;
        if is_root && strict {
            root_rels = Some(pkg.rels_of(&key)?.unwrap_or_default());
        }
        for p in
            part.build
                .iter()
                .filter_map(|i| i.path.clone())
                .chain(part.objects.values().flat_map(|o| match &o.content {
                    model::Content::Components(c) => {
                        c.iter().filter_map(|c| c.path.clone()).collect()
                    }
                    _ => Vec::new(),
                }))
        {
            if !parts.contains_key(&p) && p != key {
                queue.push(p);
            }
        }
        parts.insert(key, part);
    }
    ctx.checkpoint()?;
    let root = &parts[&root_key];
    let root_unit = root.unit.unwrap_or(LengthUnit::Millimetre);

    if strict {
        strict_part_checks(&mut pkg, &parts, &root_key)?;
    }

    // ---- Slicer extras.
    let mut extras = Extras {
        read_paint: o.read_slicer_extras,
        ..Extras::default()
    };
    if o.read_slicer_extras {
        read_extras(&mut pkg, &mut extras, limits, ctx, &mut warnings)?;
    }

    // ---- Scene.
    let mut b = Builder::new(
        &parts,
        &root_key,
        root_unit,
        strict,
        o.keep_components,
        limits.max_component_depth,
        &extras,
        ctx,
    );
    b.run()?;
    warnings.extend(b.warnings());
    let mut scene = std::mem::take(&mut b.scene);
    drop(b);
    let root = &parts[&root_key];
    for (k, v) in &root.metadata {
        scene.metadata.entry(k.clone()).or_insert_with(|| v.clone());
    }

    // ---- Thumbnail.
    scene.thumbnail = read_thumbnail(&mut pkg, strict, ctx, &mut warnings)?;

    // ---- Ignored content.
    let mut ns: BTreeSet<&String> = BTreeSet::new();
    let mut ignored: BTreeMap<&String, u64> = BTreeMap::new();
    for p in parts.values() {
        ns.extend(p.ignored_ns.iter());
        for (k, v) in &p.ignored {
            *ignored.entry(k).or_default() += v;
        }
    }
    for n in ns {
        warnings.push(IoWarning::UnknownExtensionIgnored {
            namespace: n.clone(),
        });
    }
    for (what, count) in ignored {
        warnings.push(IoWarning::UnsupportedContentIgnored {
            what: what.clone(),
            count,
        });
    }

    if scene.roots.is_empty() || scene.objects.iter().all(|o| o.mesh.face_count() == 0) {
        return Err(IoError::Empty);
    }
    ctx.progress(0.92);
    if strict {
        strict_scene_checks(&scene, ctx)?;
    } else {
        let used: BTreeSet<u32> = scene.instances().into_iter().map(|(i, _)| i).collect();
        for i in used {
            ctx.checkpoint()?;
            let ob = &scene.objects[i as usize];
            if !matches!(ob.kind, ObjectKind::Model | ObjectKind::SolidSupport) {
                continue;
            }
            let (boundary, nonman) = edge_counts(&ob.mesh, ctx)?;
            if boundary > 0 || nonman > 0 {
                warnings.push(IoWarning::NotManifold {
                    object: ob
                        .name
                        .clone()
                        .or_else(|| ob.source_id.clone())
                        .unwrap_or_else(|| format!("#{i}")),
                    boundary_edges: boundary,
                    non_manifold_edges: nonman,
                });
            }
        }
    }

    let mut report = ImportReport {
        format: Some(Format::ThreeMf),
        bytes: len,
        declared_unit: Some(root_unit),
        warnings,
        ..ImportReport::default()
    };
    for ob in &scene.objects {
        report.triangles_read += ob.mesh.face_count() as u64;
        report.vertices += ob.mesh.vertex_count() as u64;
        report.degenerate_triangles += crate::stl::count_degenerate(&ob.mesh);
    }
    report.duration = t0.elapsed();
    ctx.progress(1.0);
    Ok(Imported { scene, report })
}

/// Strict MUSTs that need the package and every model part.
fn strict_part_checks(
    pkg: &mut Package<'_>,
    parts: &BTreeMap<String, ModelPart>,
    root_key: &str,
) -> Result<(), IoError> {
    let root = &parts[root_key];
    // Production: UUIDs are required when the extension is.
    if root.requires(NS_PROD) {
        if root.has_build && root.build_uuid.is_none() {
            return Err(root.err(0, "the build has no p:UUID"));
        }
        let mut seen: BTreeSet<&str> = BTreeSet::new();
        for i in &root.build {
            match &i.uuid {
                None => return Err(root.err(i.offset, "a build item has no p:UUID")),
                Some(u) => {
                    if !seen.insert(u) {
                        return Err(root.err(i.offset, format!("duplicate p:UUID {u}")));
                    }
                }
            }
        }
        for p in parts.values() {
            for o in p.objects.values() {
                match &o.uuid {
                    None => return Err(p.err(o.offset, format!("object {} has no p:UUID", o.id))),
                    Some(u) => {
                        if !seen.insert(u) {
                            return Err(p.err(o.offset, format!("duplicate p:UUID {u}")));
                        }
                    }
                }
                if let model::Content::Components(cs) = &o.content {
                    for c in cs {
                        match &c.uuid {
                            None => return Err(p.err(c.offset, "a component has no p:UUID")),
                            Some(u) => {
                                if !seen.insert(u) {
                                    return Err(p.err(c.offset, format!("duplicate p:UUID {u}")));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    // Every p:path target object must exist.
    for p in parts.values() {
        for o in p.objects.values() {
            if let model::Content::Components(cs) = &o.content {
                for c in cs {
                    let tk = c.path.as_deref().unwrap_or(&p.key);
                    if !parts
                        .get(tk)
                        .is_some_and(|t| t.objects.contains_key(&c.objectid))
                    {
                        return Err(p.err(
                            c.offset,
                            format!("component refers to a missing object {}", c.objectid),
                        ));
                    }
                }
            }
        }
        for i in &p.build {
            let tk = i.path.as_deref().unwrap_or(&p.key);
            if !parts
                .get(tk)
                .is_some_and(|t| t.objects.contains_key(&i.objectid))
            {
                return Err(p.err(
                    i.offset,
                    format!("build item refers to a missing object {}", i.objectid),
                ));
            }
        }
    }
    // Textures: the part exists, is PNG or JPEG, and the model part has a texture
    // relationship to it.
    for p in parts.values() {
        let mut rels = None;
        for g in p.groups.values() {
            let model::PropGroup::Texture2d { path, .. } = g else {
                continue;
            };
            if !pkg.has(path) {
                return Err(p.err(0, format!("texture {path} does not exist")));
            }
            match pkg.content_types.of(path) {
                Some("image/png" | "image/jpeg") => {}
                other => {
                    return Err(malformed_part(
                        &pkg.display_name(path),
                        format!("texture content type {other:?} is not image/png or image/jpeg"),
                    ));
                }
            }
            if rels.is_none() {
                rels = Some(pkg.rels_of(&p.key)?.unwrap_or_default());
            }
            let ok = rels
                .as_ref()
                .is_some_and(|r| r.iter().any(|r| r.target == *path && r.kind == REL_TEXTURE));
            if !ok {
                return Err(p.err(
                    0,
                    format!("texture {path} has no texture relationship from {}", p.name),
                ));
            }
        }
    }
    Ok(())
}

/// Reads a small part, or `None` if it is absent.
fn small_part(pkg: &mut Package<'_>, key: &str, ctx: &JobCtx) -> Result<Option<Vec<u8>>, IoError> {
    if !pkg.has(key) {
        return Ok(None);
    }
    if pkg.uncompressed_size(key) > MAX_CONFIG {
        return Err(IoError::LimitExceeded {
            what: "slicer config size",
            limit: MAX_CONFIG,
            actual: pkg.uncompressed_size(key),
        });
    }
    pkg.read_part(key, ctx, &mut |_| {}).map(Some)
}

fn read_extras(
    pkg: &mut Package<'_>,
    extras: &mut Extras,
    limits: &crate::ReadLimits,
    ctx: &JobCtx,
    warnings: &mut Vec<IoWarning>,
) -> Result<(), IoError> {
    let mut unreadable = 0u64;
    let orca = part_key("Metadata/model_settings.config");
    if let Some(d) = small_part(pkg, &orca, ctx)? {
        match config::parse_orca(&d, "Metadata/model_settings.config", limits) {
            Ok(c) => extras.orca = Some(c),
            Err(IoError::LimitExceeded {
                what,
                limit,
                actual,
            }) => {
                return Err(IoError::LimitExceeded {
                    what,
                    limit,
                    actual,
                });
            }
            Err(_) => unreadable += 1,
        }
        if let Some(p) = small_part(pkg, &part_key("Metadata/project_settings.config"), ctx)? {
            extras.filament_colors = config::orca_filament_colors(&p);
        }
    }
    let prusa = part_key("Metadata/Slic3r_PE_model.config");
    if let Some(d) = small_part(pkg, &prusa, ctx)? {
        match config::parse_prusa(&d, "Metadata/Slic3r_PE_model.config", limits) {
            Ok(c) => extras.prusa = Some(c),
            Err(IoError::LimitExceeded {
                what,
                limit,
                actual,
            }) => {
                return Err(IoError::LimitExceeded {
                    what,
                    limit,
                    actual,
                });
            }
            Err(_) => unreadable += 1,
        }
        if extras.filament_colors.is_empty()
            && let Some(p) = small_part(pkg, &part_key("Metadata/Slic3r_PE.config"), ctx)?
        {
            extras.filament_colors = config::prusa_filament_colors(&p);
        }
    }
    if unreadable > 0 {
        warnings.push(IoWarning::UnsupportedContentIgnored {
            what: "unreadable slicer model config".into(),
            count: unreadable,
        });
    }
    Ok(())
}

/// JPEG with four colour components (CMYK), which 3MF forbids: scans the SOF marker.
fn jpeg_is_cmyk(b: &[u8]) -> bool {
    let mut i = 2;
    while i + 4 <= b.len() {
        if b[i] != 0xFF {
            return false;
        }
        let m = b[i + 1];
        if m == 0xFF {
            i += 1;
            continue;
        }
        if matches!(m, 0xD8 | 0x01 | 0xD0..=0xD7) {
            i += 2;
            continue;
        }
        let len = usize::from(u16::from_be_bytes([b[i + 2], b[i + 3]]));
        let sof = matches!(m, 0xC0..=0xCF) && !matches!(m, 0xC4 | 0xC8 | 0xCC);
        if sof {
            return b.get(i + 9).is_some_and(|&n| n == 4);
        }
        i += 2 + len;
    }
    false
}

fn read_thumbnail(
    pkg: &mut Package<'_>,
    strict: bool,
    ctx: &JobCtx,
    warnings: &mut Vec<IoWarning>,
) -> Result<Option<Thumbnail>, IoError> {
    let rels: Vec<_> = pkg
        .root_rels
        .iter()
        .filter(|r| r.kind == REL_THUMBNAIL && !r.external)
        .cloned()
        .collect();
    let mut out = None;
    for r in rels {
        if !pkg.has(&r.target) {
            if strict {
                return Err(malformed_part(
                    "_rels/.rels",
                    format!("thumbnail {} does not exist", r.exact),
                ));
            }
            warnings.push(IoWarning::MissingReference {
                what: format!("thumbnail {}", r.exact),
            });
            continue;
        }
        let name = pkg.display_name(&r.target);
        let ct = pkg.content_types.of(&r.target).map(str::to_owned);
        if strict && !matches!(ct.as_deref(), Some("image/png" | "image/jpeg")) {
            return Err(malformed_part(
                &name,
                format!("thumbnail content type {ct:?} is not image/png or image/jpeg"),
            ));
        }
        if pkg.uncompressed_size(&r.target) > MAX_THUMBNAIL {
            warnings.push(IoWarning::UnsupportedContentIgnored {
                what: "oversized thumbnail".into(),
                count: 1,
            });
            continue;
        }
        let data = pkg.read_part(&r.target, ctx, &mut |_| {})?;
        let png = data.starts_with(b"\x89PNG\r\n\x1a\n");
        let jpeg = data.starts_with(&[0xFF, 0xD8]);
        if strict && jpeg && jpeg_is_cmyk(&data) {
            return Err(malformed_part(
                &name,
                "CMYK JPEG thumbnails are not allowed",
            ));
        }
        if strict && !png && !jpeg {
            return Err(malformed_part(
                &name,
                "the thumbnail is neither PNG nor JPEG",
            ));
        }
        if out.is_none() {
            if png {
                out = Some(Thumbnail::Png(data.into()));
            } else {
                warnings.push(IoWarning::UnsupportedContentIgnored {
                    what: "JPEG thumbnail".into(),
                    count: 1,
                });
            }
        }
    }
    Ok(out)
}
