//! STEP exchange (S12 §8, IO-06; cargo feature `brep`).
//!
//! S02 BREP-13 reads and writes STEP through XCAF into a plain-data [`ForeignModel`]. This
//! module maps it to and from a [`BrepScene`]:
//!
//! - one [`BrepObject`] per shared part (`ForeignModel::prototypes`), so instances of one part
//!   stay one object (`instance_of`);
//! - a tree of [`BrepNode`]s: assemblies are groups, part instances point at their object;
//! - XCAF's linear RGBA becomes sRGB [`Rgba8`] (and back);
//! - units: the geometry is already mm; the file's declared unit goes into
//!   [`ImportReport::declared_unit`]. Exports are always written in mm.
//!
//! [`import_step_with`] heals every part itself (instead of BREP-13's built-in pass) so that it
//! can tell, per part, how many fixes healing applied and whether the result passes
//! `BRepCheck`: that is the [`HealthReport`]. Face colours follow the healing history.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

use amg_brep::{
    BrepKernel, CheckOpts, FaceId, ForeignModel, ForeignNode, ForeignPrototype, HealOpts,
    ImportOpts, InputSub, KernelConfig, KernelError, KernelWarning, OcctKernel, Relation, Shape,
    SubKind, SubShape, Validate,
};
use amg_core::Transform;
use amg_core::units::LengthUnit;
use amg_jobs::JobCtx;
use amg_mesh::Rgba8;

pub use amg_brep::StepSchema;

use crate::{ExportReport, Format, ImportReport, IoError, IoWarning};

/// One shared B-rep part of a [`BrepScene`].
#[derive(Clone, Debug)]
pub struct BrepObject {
    /// The part's name.
    pub name: Option<String>,
    /// The part shape (solid, shell or compound), mm, in part space.
    pub shape: Shape,
    /// The part colour (sRGB).
    pub color: Option<Rgba8>,
    /// Colours of single faces (`FaceId` index of `shape`, sRGB), ascending by face.
    pub face_colors: Vec<(u32, Rgba8)>,
}

/// One node of a [`BrepScene`] tree.
#[derive(Clone, Debug)]
pub struct BrepNode {
    /// Instance or assembly name.
    pub name: Option<String>,
    /// Placement relative to the parent (rigid), mm.
    pub transform: Transform,
    /// A colour of this instance or assembly that differs from its part's (sRGB). Additive to
    /// the S12 Interfaces block (BREP-13 reads instance colours).
    pub color: Option<Rgba8>,
    /// A part instance or an assembly.
    pub content: BrepContent,
}

/// What a [`BrepNode`] holds.
#[derive(Clone, Debug)]
pub enum BrepContent {
    /// An instance of `BrepScene::objects[i]`.
    Shape(u32),
    /// An assembly.
    Group(Vec<BrepNode>),
}

/// A B-rep file's content: shared parts and the assembly tree over them.
#[derive(Clone, Debug, Default)]
pub struct BrepScene {
    /// The parts.
    pub objects: Vec<BrepObject>,
    /// Top-level nodes, in file order.
    pub roots: Vec<BrepNode>,
    /// Free-form metadata (not read from or written to STEP yet).
    pub metadata: BTreeMap<String, String>,
}

impl BrepScene {
    /// Whether any node is an assembly (a group). Files without one import as plain bodies.
    pub fn has_assemblies(&self) -> bool {
        self.roots
            .iter()
            .any(|n| matches!(n.content, BrepContent::Group(_)))
    }

    /// Every part instance: `(object, transform from the scene root, nearest name)`, depth
    /// first in file order.
    pub fn instances(&self) -> Vec<(u32, Transform, Option<&str>)> {
        fn walk<'a>(
            node: &'a BrepNode,
            parent: Transform,
            out: &mut Vec<(u32, Transform, Option<&'a str>)>,
        ) {
            let world = parent * node.transform;
            match &node.content {
                BrepContent::Shape(o) => out.push((*o, world, node.name.as_deref())),
                BrepContent::Group(children) => {
                    for c in children {
                        walk(c, world, out);
                    }
                }
            }
        }
        let mut out = Vec::new();
        for r in &self.roots {
            walk(r, Transform::IDENTITY, &mut out);
        }
        out
    }

    /// Checks object indices, face indices and transforms.
    ///
    /// # Errors
    /// [`IoError::InvalidScene`] naming the first problem.
    pub fn validate(&self) -> Result<(), IoError> {
        let bad = |message: String| Err(IoError::InvalidScene { message });
        for (i, o) in self.objects.iter().enumerate() {
            if o.shape.is_null() {
                return bad(format!("object {i} has a null shape"));
            }
            let faces = o.shape.count(SubKind::Face);
            if let Some((f, _)) = o.face_colors.iter().find(|(f, _)| *f >= faces) {
                return bad(format!(
                    "object {i} colours face {f} of a shape with {faces} faces"
                ));
            }
        }
        fn node(s: &BrepScene, n: &BrepNode, depth: usize) -> Result<(), IoError> {
            let bad = |message: String| Err(IoError::InvalidScene { message });
            if depth > crate::MAX_GROUP_DEPTH {
                return bad("the assembly tree is too deep".to_owned());
            }
            if !(n.transform.matrix3.is_finite() && n.transform.translation.is_finite()) {
                return bad(format!("node {:?} has a non-finite transform", n.name));
            }
            match &n.content {
                BrepContent::Shape(o) if *o as usize >= s.objects.len() => bad(format!(
                    "node {:?} places object {o}, which does not exist",
                    n.name
                )),
                BrepContent::Shape(_) => Ok(()),
                BrepContent::Group(children) => {
                    children.iter().try_for_each(|c| node(s, c, depth + 1))
                }
            }
        }
        self.roots.iter().try_for_each(|r| node(self, r, 0))
    }
}

/// What [`import_step_with`] does.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BrepImportOptions {
    /// Heal every part (`ShapeFix`, BREP-13 `BrepKernel::heal` with default options). A part
    /// whose healing fails is kept as read and listed in the health report.
    pub heal: bool,
    /// Run `BRepCheck` on every part after healing (the health report's `valid`).
    pub check: bool,
    /// Read part, instance and face colours.
    pub read_colors: bool,
    /// Read part, assembly and instance names.
    pub read_names: bool,
}

impl Default for BrepImportOptions {
    fn default() -> Self {
        BrepImportOptions {
            heal: true,
            check: true,
            read_colors: true,
            read_names: true,
        }
    }
}

/// Health of one imported part.
#[derive(Clone, Debug, PartialEq)]
pub struct ShapeHealth {
    /// Index into `BrepScene::objects`.
    pub object: u32,
    /// Part name.
    pub name: Option<String>,
    /// Fixes healing applied (0: the part needed none, or healing was off or failed).
    pub fixes: u32,
    /// Why healing failed (the part is kept as read).
    pub heal_error: Option<String>,
    /// `BRepCheck` verdict after healing (`None` when the check was off).
    pub valid: Option<bool>,
    /// `BRepCheck` problems after healing (status names).
    pub problems: Vec<String>,
    /// Solids in the part.
    pub solids: u32,
    /// Faces in the part.
    pub faces: u32,
}

impl ShapeHealth {
    /// Healing changed this part.
    pub fn was_fixed(&self) -> bool {
        self.fixes > 0
    }

    /// Healing failed, or the part is invalid after healing.
    pub fn failed(&self) -> bool {
        self.heal_error.is_some() || self.valid == Some(false)
    }
}

/// The health report of a B-rep import: one entry per part, plus the kernel's notes.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HealthReport {
    /// One entry per object, in object order.
    pub shapes: Vec<ShapeHealth>,
    /// BREP-13's read notes (missing 3D curves built, unknown units, ...).
    pub notes: Vec<String>,
}

impl HealthReport {
    /// Parts that healing changed.
    pub fn fixed(&self) -> impl Iterator<Item = &ShapeHealth> {
        self.shapes.iter().filter(|s| s.was_fixed())
    }

    /// Parts whose healing failed or that are invalid after it.
    pub fn failed(&self) -> impl Iterator<Item = &ShapeHealth> {
        self.shapes.iter().filter(|s| s.failed())
    }

    /// Every part passed the check.
    pub fn all_valid(&self) -> bool {
        self.shapes.iter().all(|s| s.valid != Some(false))
    }
}

/// Result of [`import_step_with`].
#[derive(Clone, Debug)]
pub struct BrepImport {
    /// The parts and the tree, mm.
    pub scene: BrepScene,
    /// Format, size, declared unit, warnings, time.
    pub report: ImportReport,
    /// Healing and check result per part.
    pub health: HealthReport,
}

// ---------------------------------------------------------------- colours

/// sRGB encoding of one linear channel in `[0, 1]`, rounded to a byte.
fn encode_srgb(c: f32) -> u8 {
    let c = f64::from(c).clamp(0.0, 1.0);
    let s = if c <= 0.003_130_8 {
        12.92 * c
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    };
    // In [0, 255] after the clamp.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let b = (s * 255.0).round().clamp(0.0, 255.0) as u8;
    b
}

/// Linear value of one sRGB byte.
fn decode_srgb(b: u8) -> f32 {
    let s = f64::from(b) / 255.0;
    let c = if s <= 0.040_45 {
        s / 12.92
    } else {
        ((s + 0.055) / 1.055).powf(2.4)
    };
    #[allow(clippy::cast_possible_truncation, reason = "colour channel")]
    let c = c as f32;
    c.clamp(0.0, 1.0)
}

/// XCAF linear RGBA → sRGB colour with linear alpha.
pub fn srgb_from_linear(c: [f32; 4]) -> Rgba8 {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let a = (f64::from(c[3]).clamp(0.0, 1.0) * 255.0).round() as u8;
    Rgba8([encode_srgb(c[0]), encode_srgb(c[1]), encode_srgb(c[2]), a])
}

/// sRGB colour with linear alpha → XCAF linear RGBA.
pub fn linear_from_srgb(c: Rgba8) -> [f32; 4] {
    let [r, g, b, a] = c.0;
    [
        decode_srgb(r),
        decode_srgb(g),
        decode_srgb(b),
        f32::from(a) / 255.0,
    ]
}

// ---------------------------------------------------------------- mapping

/// Maps a BREP-13 model to a [`BrepScene`]: one object per prototype (shared parts stay
/// shared), plus one per part node without a prototype index; assemblies become groups. A
/// node colour equal to its part's colour is not repeated on the node.
pub fn scene_from_foreign(m: &ForeignModel) -> BrepScene {
    let mut scene = BrepScene {
        objects: m.prototypes.iter().map(object_from_prototype).collect(),
        ..BrepScene::default()
    };
    scene.roots = m
        .roots
        .iter()
        .map(|n| node_from_foreign(n, &mut scene.objects))
        .collect();
    scene
}

/// A name as the scene keeps it: blank names (some exporters write `''`) are no name.
fn clean_name(n: Option<&str>) -> Option<String> {
    n.filter(|n| !n.trim().is_empty()).map(str::to_owned)
}

fn object_from_prototype(p: &ForeignPrototype) -> BrepObject {
    object_from_parts(
        clean_name(p.name.as_deref()),
        &p.shape,
        p.color,
        &p.face_colors,
    )
}

fn object_from_parts(
    name: Option<String>,
    shape: &Shape,
    color: Option<[f32; 4]>,
    face_colors: &[(FaceId, [f32; 4])],
) -> BrepObject {
    let mut fc: Vec<(u32, Rgba8)> = face_colors
        .iter()
        .map(|&(f, c)| (f.0, srgb_from_linear(c)))
        .collect();
    fc.sort_by_key(|&(f, _)| f);
    fc.dedup_by_key(|&mut (f, _)| f);
    BrepObject {
        name,
        shape: shape.clone(),
        color: color.map(srgb_from_linear),
        face_colors: fc,
    }
}

fn node_from_foreign(n: &ForeignNode, objects: &mut Vec<BrepObject>) -> BrepNode {
    let own = n.color.map(srgb_from_linear);
    let content = match (n.instance_of, &n.shape) {
        (Some(p), _) if (p as usize) < objects.len() => BrepContent::Shape(p),
        (_, Some(shape)) => {
            let i = u32::try_from(objects.len()).unwrap_or(u32::MAX);
            objects.push(object_from_parts(
                clean_name(n.name.as_deref()),
                shape,
                n.color,
                &n.face_colors,
            ));
            BrepContent::Shape(i)
        }
        _ => BrepContent::Group(
            n.children
                .iter()
                .map(|c| node_from_foreign(c, objects))
                .collect(),
        ),
    };
    let color = match &content {
        BrepContent::Shape(o) if objects[*o as usize].color == own => None,
        _ => own,
    };
    BrepNode {
        name: clean_name(n.name.as_deref()),
        transform: n.transform,
        color,
        content,
    }
}

/// Maps a [`BrepScene`] back to a BREP-13 model in mm: objects become prototypes, part nodes
/// instances of them (with their part's colour unless the node has its own).
pub fn foreign_from_scene(s: &BrepScene) -> ForeignModel {
    let prototypes: Vec<ForeignPrototype> = s
        .objects
        .iter()
        .map(|o| ForeignPrototype {
            name: o.name.clone(),
            shape: o.shape.clone(),
            color: o.color.map(linear_from_srgb),
            face_colors: o
                .face_colors
                .iter()
                .map(|&(f, c)| (FaceId(f), linear_from_srgb(c)))
                .collect(),
        })
        .collect();
    fn node(n: &BrepNode, protos: &[ForeignPrototype]) -> ForeignNode {
        match &n.content {
            BrepContent::Shape(o) => {
                let p = protos.get(*o as usize);
                ForeignNode {
                    name: n.name.clone(),
                    transform: n.transform,
                    color: n.color.map(linear_from_srgb).or(p.and_then(|p| p.color)),
                    shape: p.map(|p| p.shape.clone()),
                    face_colors: p.map(|p| p.face_colors.clone()).unwrap_or_default(),
                    children: Vec::new(),
                    instance_of: Some(*o),
                }
            }
            BrepContent::Group(children) => ForeignNode {
                name: n.name.clone(),
                transform: n.transform,
                color: n.color.map(linear_from_srgb),
                shape: None,
                face_colors: Vec::new(),
                children: children.iter().map(|c| node(c, protos)).collect(),
                instance_of: None,
            },
        }
    }
    let roots = s.roots.iter().map(|r| node(r, &prototypes)).collect();
    ForeignModel {
        roots,
        prototypes,
        source_unit_mm: 1.0,
        warnings: Vec::new(),
    }
}

// ---------------------------------------------------------------- errors, units

/// A kernel error as an `amg-io` error.
fn io_error(e: KernelError, format: Format, direction: &'static str, path: &Path) -> IoError {
    match e {
        KernelError::Cancelled => IoError::Cancelled,
        KernelError::BackendUnavailable => IoError::Unsupported {
            format,
            direction,
            reason: "this build has no geometry kernel (cargo feature `occt`)",
        },
        KernelError::Io(m) => {
            let source = std::io::Error::other(m);
            if direction == "import" {
                IoError::Read {
                    path: path.to_owned(),
                    source,
                }
            } else {
                IoError::Write {
                    path: path.to_owned(),
                    source,
                }
            }
        }
        other => IoError::Brep(other.to_string()),
    }
}

/// The [`LengthUnit`] of a declared unit in mm, if it is one.
fn length_unit(mm: f64) -> Option<LengthUnit> {
    LengthUnit::ALL
        .into_iter()
        .find(|&u| (crate::scale_to_mm(1.0, u) - mm).abs() <= 1e-12 * mm)
}

// ---------------------------------------------------------------- import

/// Reads a STEP file with the default [`BrepImportOptions`] (heal and check every part).
/// The health report's failures are also in the report's warnings.
///
/// # Errors
/// As [`import_step_with`].
pub fn import_step(src: &Path, ctx: &JobCtx) -> Result<(BrepScene, ImportReport), IoError> {
    let r = import_step_with(src, &BrepImportOptions::default(), ctx)?;
    Ok((r.scene, r.report))
}

/// Reads a STEP file (AP203, AP214, AP242) into a [`BrepScene`] with its [`HealthReport`].
/// Run it in a job: the read reports progress (parse and transfer to 80 %, healing and the
/// check the rest) and stops on cancel.
///
/// # Errors
/// [`IoError::Read`] when the file cannot be read, [`IoError::Brep`] when OCCT cannot parse or
/// transfer it, [`IoError::Unsupported`] without the OCCT backend, [`IoError::Cancelled`].
pub fn import_step_with(
    src: &Path,
    opts: &BrepImportOptions,
    ctx: &JobCtx,
) -> Result<BrepImport, IoError> {
    let started = Instant::now();
    ctx.checkpoint()?;
    let bytes = std::fs::metadata(src)
        .map_err(|source| IoError::Read {
            path: src.to_owned(),
            source,
        })?
        .len();
    let read_opts = ImportOpts {
        heal: false,
        read_colors: opts.read_colors,
        read_names: opts.read_names,
    };
    let mut model = amg_brep::io::read_step(src, &read_opts, &ctx.sub(0.0, 0.8))
        .map_err(|e| io_error(e, Format::Step, "import", src))?;
    let health = heal_and_check(&mut model, opts, &ctx.sub(0.8, 1.0))?;
    let scene = scene_from_foreign(&model);
    let mut report = ImportReport {
        format: Some(Format::Step),
        bytes,
        declared_unit: length_unit(model.source_unit_mm),
        applied_unit: Some(LengthUnit::Millimetre),
        ..ImportReport::default()
    };
    report.warnings.extend(
        health
            .notes
            .iter()
            .map(|n| IoWarning::KernelNote(n.clone())),
    );
    report.warnings.extend(health.failed().map(|s| {
        IoWarning::ShapeInvalid {
            object: s
                .name
                .clone()
                .unwrap_or_else(|| format!("part {}", s.object + 1)),
            problems: s
                .heal_error
                .iter()
                .cloned()
                .chain(s.problems.iter().cloned())
                .collect(),
        }
    }));
    ctx.progress(1.0);
    report.duration = started.elapsed();
    Ok(BrepImport {
        scene,
        report,
        health,
    })
}

/// Heals and checks every prototype of `model` in place (face colours follow the healing
/// history; part nodes get the healed shapes too) and returns the health report.
fn heal_and_check(
    model: &mut ForeignModel,
    opts: &BrepImportOptions,
    ctx: &JobCtx,
) -> Result<HealthReport, IoError> {
    let kernel = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let heal_opts = HealOpts::default();
    let mut report = HealthReport {
        shapes: Vec::with_capacity(model.prototypes.len()),
        notes: std::mem::take(&mut model.warnings),
    };
    #[allow(clippy::cast_precision_loss, reason = "progress fractions")]
    let n = model.prototypes.len().max(1) as f32;
    for (p, proto) in model.prototypes.iter_mut().enumerate() {
        ctx.checkpoint()?;
        #[allow(clippy::cast_precision_loss, reason = "progress fractions")]
        let step = ctx.sub(p as f32 / n, (p + 1) as f32 / n);
        let mut h = ShapeHealth {
            object: u32::try_from(p).unwrap_or(u32::MAX),
            name: proto.name.clone(),
            fixes: 0,
            heal_error: None,
            valid: None,
            problems: Vec::new(),
            solids: 0,
            faces: 0,
        };
        let has_faces = !proto.shape.is_null() && proto.shape.count(SubKind::Face) > 0;
        if opts.heal && has_faces {
            match kernel.heal(&proto.shape, &heal_opts, &step.sub(0.0, 0.5)) {
                Ok(healed) => {
                    h.fixes = healed
                        .warnings
                        .iter()
                        .map(|w| match w {
                            KernelWarning::Healed { fixes } => *fixes,
                            _ => 0,
                        })
                        .sum();
                    // A part that needed no fix keeps its shape (and its sharing) as read.
                    if h.fixes > 0 {
                        proto.face_colors = carry_face_colors(&proto.face_colors, &healed.history);
                        proto.shape = healed.shape;
                    }
                }
                Err(KernelError::Cancelled) => return Err(IoError::Cancelled),
                Err(e) => h.heal_error = Some(format!("healing failed: {e}")),
            }
        }
        if opts.check && !proto.shape.is_null() {
            step.sub(0.5, 1.0).checkpoint()?;
            let check = kernel.check(&proto.shape, &CheckOpts::default());
            h.valid = Some(check.valid);
            h.problems = check
                .problems
                .into_iter()
                .map(|(sub, status)| match sub {
                    Some(s) => format!("{status} on {:?} {}", s.kind, s.index),
                    None => status,
                })
                .collect();
        }
        if !proto.shape.is_null() {
            h.solids = u32::try_from(proto.shape.solids().len()).unwrap_or(u32::MAX);
            h.faces = proto.shape.count(SubKind::Face);
        }
        step.progress(1.0);
        report.shapes.push(h);
    }
    // Part nodes carry clones of their prototype's shape: give them the healed ones.
    fn refresh(node: &mut ForeignNode, protos: &[ForeignPrototype]) {
        if let Some(p) = node.instance_of.and_then(|p| protos.get(p as usize)) {
            node.shape = Some(p.shape.clone());
            node.face_colors.clone_from(&p.face_colors);
        }
        for c in &mut node.children {
            refresh(c, protos);
        }
    }
    for r in &mut model.roots {
        refresh(r, &model.prototypes);
    }
    Ok(report)
}

/// Face colours mapped through a healing history (split faces keep the colour, deleted ones
/// drop it).
fn carry_face_colors(
    colors: &[(FaceId, [f32; 4])],
    history: &amg_brep::ShapeHistory,
) -> Vec<(FaceId, [f32; 4])> {
    let mut out = Vec::with_capacity(colors.len());
    for &(face, color) in colors {
        let input = InputSub {
            operand: 0,
            sub: SubShape {
                kind: SubKind::Face,
                index: face.0,
            },
        };
        for relation in history.of_input(input) {
            let outputs: &[SubShape] = match relation {
                Relation::Modified(v) | Relation::Generated(v) => v,
                Relation::Kept(s) => std::slice::from_ref(s),
                Relation::Deleted => &[],
            };
            out.extend(
                outputs
                    .iter()
                    .filter(|s| s.kind == SubKind::Face)
                    .map(|s| (FaceId(s.index), color)),
            );
        }
    }
    out.sort_by_key(|c| c.0.0);
    out.dedup_by_key(|c| c.0.0);
    out
}

// ---------------------------------------------------------------- export

/// A temporary path next to `dst` (same directory, so the rename is atomic).
fn temp_path(dst: &Path) -> PathBuf {
    let name = dst
        .file_name()
        .map_or_else(|| "export".into(), |n| n.to_string_lossy().into_owned());
    dst.with_file_name(format!(".{name}.{}.tmp", std::process::id()))
}

/// Writes `scene` as STEP in `schema`, in mm, with names, part, instance and face colours and
/// the assembly tree (shared objects stay shared parts). The file appears atomically: it is
/// written to a temporary file next to `dst` and renamed; on error `dst` is untouched.
///
/// # Errors
/// [`IoError::InvalidScene`] for a scene that fails [`BrepScene::validate`] or that the kernel
/// refuses (a non-rigid transform); [`IoError::Write`]; [`IoError::Brep`] when OCCT cannot
/// write it; [`IoError::Unsupported`] without the OCCT backend; [`IoError::Cancelled`].
pub fn export_step(
    scene: &BrepScene,
    schema: StepSchema,
    dst: &Path,
    ctx: &JobCtx,
) -> Result<ExportReport, IoError> {
    let started = Instant::now();
    ctx.checkpoint()?;
    scene.validate()?;
    let model = foreign_from_scene(scene);
    let tmp = temp_path(dst);
    let written = amg_brep::io::write_step(&tmp, &model, schema, ctx);
    if let Err(e) = written {
        let _ = std::fs::remove_file(&tmp);
        return Err(match e {
            KernelError::InvalidInput {
                what: "model",
                reason,
                ..
            } => IoError::InvalidScene { message: reason },
            e => io_error(e, Format::Step, "export", dst),
        });
    }
    if let Err(source) = std::fs::rename(&tmp, dst) {
        let _ = std::fs::remove_file(&tmp);
        return Err(IoError::Write {
            path: dst.to_owned(),
            source,
        });
    }
    let bytes = std::fs::metadata(dst).map_or(0, |m| m.len());
    Ok(ExportReport {
        bytes,
        triangles: 0,
        files: vec![dst.to_owned()],
        warnings: Vec::new(),
        duration: started.elapsed(),
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
mod tests {
    use super::*;

    #[test]
    fn srgb_round_trips_every_byte() {
        for b in 0..=255u8 {
            let c = Rgba8([b, b, b, b]);
            assert_eq!(srgb_from_linear(linear_from_srgb(c)), c, "{b}");
        }
        assert_eq!(
            srgb_from_linear([0.5, 0.0, 1.0, 1.0]),
            Rgba8([188, 0, 255, 255])
        );
        assert_eq!(
            srgb_from_linear([2.0, -1.0, f32::NAN, 1.0]).0[..2],
            [255, 0]
        );
    }

    #[test]
    fn units_map_to_length_units() {
        assert_eq!(length_unit(1.0), Some(LengthUnit::Millimetre));
        assert_eq!(length_unit(25.4), Some(LengthUnit::Inch));
        assert_eq!(length_unit(1000.0), Some(LengthUnit::Metre));
        assert_eq!(length_unit(0.0254), None);
    }

    #[test]
    fn empty_scene_maps_both_ways() {
        let s = scene_from_foreign(&ForeignModel::default());
        assert!(s.objects.is_empty() && s.roots.is_empty());
        assert!(!s.has_assemblies());
        s.validate().unwrap();
        let m = foreign_from_scene(&s);
        assert!(m.roots.is_empty() && m.prototypes.is_empty());
        assert!((m.source_unit_mm - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn assemblies_without_parts_map_to_groups() {
        let m = ForeignModel::new(vec![ForeignNode::assembly(
            Some("top"),
            vec![ForeignNode::assembly(
                Some("sub"),
                Vec::new(),
                Transform::IDENTITY,
            )],
            Transform::IDENTITY,
        )]);
        let s = scene_from_foreign(&m);
        assert!(s.has_assemblies());
        let BrepContent::Group(children) = &s.roots[0].content else {
            panic!("group expected");
        };
        assert_eq!(children[0].name.as_deref(), Some("sub"));
        let back = foreign_from_scene(&s);
        assert_eq!(back.nodes().len(), 2);
        assert!(s.instances().is_empty());
    }

    #[test]
    fn bad_scenes_are_rejected() {
        let s = BrepScene {
            roots: vec![BrepNode {
                name: None,
                transform: Transform::IDENTITY,
                color: None,
                content: BrepContent::Shape(3),
            }],
            ..BrepScene::default()
        };
        assert!(matches!(s.validate(), Err(IoError::InvalidScene { .. })));
    }

    #[test]
    fn missing_file_is_a_read_error() {
        let r = import_step(Path::new("does-not-exist.step"), &JobCtx::detached());
        assert!(matches!(r, Err(IoError::Read { .. })), "{r:?}");
    }
}
