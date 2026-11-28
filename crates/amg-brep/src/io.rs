//! STEP (AP203/214/242) and IGES read and write through XCAF (S02 §9, BREP-13).
//!
//! Reads give a [`ForeignModel`] in mm: the assembly tree (XCAF free shapes and components),
//! names, linear RGBA colours of parts, instances and single faces, and one shared prototype per
//! part. Edges without a 3D curve get one before anything indexes the shapes (BREP-10 leftover),
//! and with [`ImportOpts::heal`] every prototype runs through [`BrepKernel::heal`].
//!
//! Progress and cancellation go through the `JobCtx`: a STEP read reports parsing (by bytes, the
//! first 16 %), the transfer (to 80 %) and healing, and a cancel stops the parser or the
//! transfer. IGES reads and writes hold the global I/O lock (OCCT's IGES translator is not
//! thread-safe); STEP reads and writes run in parallel, unlocked (OCCT 8.0.1): each call has its
//! own reader or writer, work session and STEP controller, so no translator actor is shared
//! (OCCT's registered STEP controller owns one write actor whose assembly map concurrent writers
//! used to clear for each other; BREP-13 fix).

use std::path::Path;

use crate::backend::xde::{self, Format};
use crate::error::{KResult, KernelError};
use crate::foreign::{
    FlatModel, ForeignModel, ImportOpts, StepSchema, flatten, iges_unit_mm, step_source_unit,
    unflatten, write_unit_mm,
};
use crate::opts::HealOpts;
use crate::shape::{SubKind, SubShape};
use crate::{BrepKernel, InputSub, JobCtx, KernelConfig, OcctKernel, Relation, Validate};

/// The share of the progress the file read takes; healing reports the rest.
const READ_SHARE: f32 = 0.8;

/// Reads a STEP file.
///
/// # Errors
/// `Io` when `path` is not a file; `InvalidInput { what: "path" }` for a path that is not UTF-8;
/// `Format` when OCCT cannot parse or transfer it; `Cancelled`; `BackendUnavailable` without
/// OCCT.
pub fn read_step(path: &Path, opts: &ImportOpts, ctx: &JobCtx) -> KResult<ForeignModel> {
    read(Format::Step, path, opts, ctx)
}

/// Writes `model` as STEP in `schema`, in the unit `model.source_unit_mm` names (mm when it is
/// not one of mm, cm, m, inch, foot, micron). Names and colours are written in every schema
/// (AP203 through `SHAPE_APPEARANCE_LAYER_MIM`). The caller's shapes are not changed: the writer works on private copies.
///
/// # Errors
/// `InvalidInput { what: "model" }` for a transform that is not rigid, a prototype index or face
/// colour that does not exist, a colour outside `[0, 1]`, or a node with both a shape and
/// children; `InvalidInput { what: "path" }`; `Format` when OCCT cannot transfer or write it;
/// `Cancelled`; `BackendUnavailable` without OCCT.
pub fn write_step(
    path: &Path,
    model: &ForeignModel,
    schema: StepSchema,
    ctx: &JobCtx,
) -> KResult<()> {
    let flat = flatten(model)?;
    xde::write(
        Format::Step,
        path,
        &flat,
        schema.code(),
        write_unit_mm(model.source_unit_mm),
        ctx,
    )
}

/// Reads an IGES file (under the global lock). IGES has no assemblies: every root is a part.
///
/// # Errors
/// As [`read_step`].
pub fn read_iges(path: &Path, opts: &ImportOpts, ctx: &JobCtx) -> KResult<ForeignModel> {
    read(Format::Iges, path, opts, ctx)
}

/// Writes `model` as IGES 5.3 B-rep solids (MSBO entities; under the global lock). Assemblies
/// are flattened into their placed parts.
///
/// # Errors
/// As [`write_step`].
pub fn write_iges(path: &Path, model: &ForeignModel, ctx: &JobCtx) -> KResult<()> {
    let flat = flatten(model)?;
    xde::write(
        Format::Iges,
        path,
        &flat,
        0,
        write_unit_mm(model.source_unit_mm),
        ctx,
    )
}

fn read(format: Format, path: &Path, opts: &ImportOpts, ctx: &JobCtx) -> KResult<ForeignModel> {
    let raw = xde::read(
        format,
        path,
        opts.read_names,
        opts.read_colors,
        &ctx.sub(0.0, READ_SHARE),
    )?;
    let mut warnings = raw.warnings;
    let unit = match format {
        Format::Step => step_source_unit(&raw.unit_names, &mut warnings),
        Format::Iges => iges_unit_mm(raw.iges_unit_flag).unwrap_or_else(|| {
            warnings.push(format!(
                "IGES unit flag {} is not a known unit; mm assumed",
                raw.iges_unit_flag
            ));
            1.0
        }),
    };
    let mut flat = raw.flat;
    if opts.heal {
        heal_prototypes(&mut flat, &mut warnings, &ctx.sub(READ_SHARE, 1.0))?;
    }
    ctx.progress(1.0);
    Ok(unflatten(flat, unit, warnings))
}

/// Heals every prototype; face colours follow each face's history. A prototype that fails to
/// heal stays as read, with a warning.
fn heal_prototypes(flat: &mut FlatModel, warnings: &mut Vec<String>, ctx: &JobCtx) -> KResult<()> {
    let kernel = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let opts = HealOpts::default();
    #[allow(clippy::cast_precision_loss, reason = "progress fractions")]
    let n = flat.shapes.len().max(1) as f32;
    let mut colors = Vec::with_capacity(flat.face_colors.len());
    for (p, shape) in flat.shapes.iter_mut().enumerate() {
        let p32 = u32::try_from(p).unwrap_or(u32::MAX);
        let own: Vec<(u32, [f32; 4])> = flat
            .face_colors
            .iter()
            .filter(|c| c.0 == p32)
            .map(|&(_, face, color)| (face, color))
            .collect();
        if shape.is_null() || shape.count(SubKind::Face) == 0 {
            colors.extend(own.into_iter().map(|(face, color)| (p32, face, color)));
            continue;
        }
        #[allow(clippy::cast_precision_loss, reason = "progress fractions")]
        let step = ctx.sub(p as f32 / n, (p + 1) as f32 / n);
        match kernel.heal(shape, &opts, &step) {
            Ok(healed) => {
                for (face, color) in own {
                    let input = InputSub {
                        operand: 0,
                        sub: SubShape {
                            kind: SubKind::Face,
                            index: face,
                        },
                    };
                    for relation in healed.history.of_input(input) {
                        let outputs: &[SubShape] = match relation {
                            Relation::Modified(v) | Relation::Generated(v) => v,
                            Relation::Kept(s) => std::slice::from_ref(s),
                            Relation::Deleted => &[],
                        };
                        for out in outputs.iter().filter(|s| s.kind == SubKind::Face) {
                            colors.push((p32, out.index, color));
                        }
                    }
                }
                *shape = healed.shape;
            }
            Err(KernelError::Cancelled) => return Err(KernelError::Cancelled),
            Err(error) => {
                let name = flat.protos[p]
                    .name
                    .as_deref()
                    .map(|n| format!(" `{n}`"))
                    .unwrap_or_default();
                warnings.push(format!(
                    "prototype {p}{name}: healing failed ({error}); kept as read"
                ));
                colors.extend(own.into_iter().map(|(face, color)| (p32, face, color)));
            }
        }
    }
    colors.sort_by_key(|c| (c.0, c.1));
    colors.dedup_by_key(|c| (c.0, c.1));
    flat.face_colors = colors;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(not(feature = "occt"))]
    #[test]
    fn no_backend_is_unavailable() {
        let ctx = JobCtx::detached();
        let path = Path::new("missing.step");
        assert!(matches!(
            read_step(path, &ImportOpts::default(), &ctx),
            Err(KernelError::BackendUnavailable)
        ));
        assert!(matches!(
            read_iges(path, &ImportOpts::default(), &ctx),
            Err(KernelError::BackendUnavailable)
        ));
        let empty = ForeignModel::default();
        assert!(matches!(
            write_step(path, &empty, StepSchema::Ap242, &ctx),
            Err(KernelError::BackendUnavailable)
        ));
        assert!(matches!(
            write_iges(path, &empty, &ctx),
            Err(KernelError::BackendUnavailable)
        ));
    }

    /// BREP-10 leftover: an edge without a 3D curve (IGES can give one) crashed the index and
    /// BRepCheck. The readers build the curve first; this runs that repair on a box whose edge
    /// lost its curve.
    #[cfg(feature = "occt")]
    #[test]
    fn readers_build_missing_3d_curves_before_indexing() {
        use crate::Shape;
        let part = Shape::test_box(10.0, 20.0, 30.0);
        for edge in 0..12 {
            let repaired = Shape::from_occ(part.occ().guard_probe(edge).expect("probe"));
            let report = repaired.check_report();
            assert!(report.valid, "edge {edge}: {report:?}");
            assert_eq!(repaired.count(SubKind::Edge), 12);
            let curve = repaired.curve(crate::EdgeId(edge));
            assert!(curve.is_some(), "edge {edge} has a curve again");
            let volume = repaired.volume().expect("volume");
            assert!((volume - 6000.0).abs() < 1e-9, "edge {edge}: {volume}");
        }
    }
}
