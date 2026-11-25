//! The OCCT side of STEP and IGES (BREP-13): the shim's XCAF readers and writers over the flat
//! node list of `crate::foreign`.

use std::path::Path;

use amg_core::{Mat3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_occt_sys::ffi;

use super::boolean::list;
use super::{Occ, mat34, progress_sink, to_kernel_error};
use crate::KResult;
use crate::error::KernelError;
use crate::foreign::{FlatModel, FlatNode, FlatProto};
use crate::lock::global_io_lock;
use crate::shape::Shape;

/// The file format of one read or write.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Format {
    Step,
    Iges,
}

/// What a read gave before healing.
pub(crate) struct RawRead {
    pub flat: FlatModel,
    pub warnings: Vec<String>,
    pub unit_names: Vec<String>,
    pub iges_unit_flag: i32,
}

const NONE: u32 = u32::MAX;

fn path_str(path: &Path) -> KResult<&str> {
    path.to_str().ok_or_else(|| KernelError::InvalidInput {
        what: "path",
        reason: format!("{} is not valid UTF-8", path.display()),
        offending: Vec::new(),
    })
}

fn transform(m: &ffi::OcMat34) -> Transform {
    let m = m.m;
    Transform::from_mat3_translation(
        Mat3::from_cols(
            Vec3::new(m[0], m[4], m[8]),
            Vec3::new(m[1], m[5], m[9]),
            Vec3::new(m[2], m[6], m[10]),
        ),
        Vec3::new(m[3], m[7], m[11]),
    )
}

fn opt_name(has: bool, name: String) -> Option<String> {
    has.then_some(name)
}

fn opt_color(has: bool, color: [f32; 4]) -> Option<[f32; 4]> {
    has.then_some(color)
}

/// Fails on a status that reports a cancellation or errors.
fn check(status: &ffi::OcXdeStatus, what: &str) -> KResult<()> {
    if status.cancelled {
        return Err(KernelError::Cancelled);
    }
    if !status.errors.is_empty() {
        return Err(KernelError::Format(format!(
            "{what}: {}",
            status.errors.join("; ")
        )));
    }
    Ok(())
}

/// Reads a STEP or IGES file (IGES under the global lock) into plain nodes and prototype shapes.
pub(crate) fn read(
    format: Format,
    path: &Path,
    names: bool,
    colors: bool,
    ctx: &JobCtx,
) -> KResult<RawRead> {
    let file = path_str(path)?;
    if !path.is_file() {
        return Err(KernelError::Io(format!("{} is not a file", path.display())));
    }
    ctx.checkpoint()?;
    let sink = progress_sink(ctx);
    let xde = match format {
        Format::Step => ffi::xde_read_step(file, names, colors, &sink),
        Format::Iges => {
            let _lock = global_io_lock();
            ffi::xde_read_iges(file, names, colors, &sink)
        }
    }
    .map_err(|e| to_kernel_error(&e))?;
    let status = ffi::xde_status(&xde).map_err(|e| to_kernel_error(&e))?;
    if status.cancelled && format == Format::Step {
        // The reader and its parsed model came back with the result: freeing a large model takes
        // about half a second, which a cancel should not wait for.
        std::thread::spawn(move || drop(xde));
        return Err(KernelError::Cancelled);
    }
    let label = match format {
        Format::Step => "STEP read",
        Format::Iges => "IGES read",
    };
    check(&status, label)?;
    let nodes = ffi::xde_nodes(&xde).map_err(|e| to_kernel_error(&e))?;
    let protos = ffi::xde_protos(&xde).map_err(|e| to_kernel_error(&e))?;
    let face_colors = ffi::xde_face_colors(&xde).map_err(|e| to_kernel_error(&e))?;
    let mut flat = FlatModel::default();
    for (i, proto) in protos.into_iter().enumerate() {
        let i = u32::try_from(i).unwrap_or(u32::MAX);
        let shape = ffi::xde_proto_shape(&xde, i).map_err(|e| to_kernel_error(&e))?;
        flat.shapes.push(Shape::from_occ(Occ::new(shape)));
        flat.protos.push(FlatProto {
            name: opt_name(proto.has_name, proto.name),
            color: opt_color(proto.has_color, proto.color),
        });
    }
    flat.nodes = nodes
        .into_iter()
        .map(|n| FlatNode {
            parent: (n.parent != NONE).then_some(n.parent),
            name: opt_name(n.has_name, n.name),
            transform: transform(&n.transform),
            color: opt_color(n.has_color, n.color),
            proto: (n.proto != NONE).then_some(n.proto),
        })
        .collect();
    flat.face_colors = face_colors
        .into_iter()
        .map(|c| (c.proto, c.face, c.color))
        .collect();
    Ok(RawRead {
        flat,
        warnings: status.warnings,
        unit_names: status.unit_names,
        iges_unit_flag: status.iges_unit_flag,
    })
}

/// Writes a flat model as STEP (with `schema`) or IGES (under the global lock), in `unit_mm`.
pub(crate) fn write(
    format: Format,
    path: &Path,
    flat: &FlatModel,
    schema: u8,
    unit_mm: f64,
    ctx: &JobCtx,
) -> KResult<()> {
    let file = path_str(path)?;
    ctx.checkpoint()?;
    let shapes: Vec<&Occ> = flat.shapes.iter().map(Shape::occ).collect();
    let protos = list(&shapes)?;
    let meta: Vec<ffi::OcXdeProto> = flat
        .protos
        .iter()
        .map(|p| ffi::OcXdeProto {
            has_name: p.name.is_some(),
            name: p.name.clone().unwrap_or_default(),
            has_color: p.color.is_some(),
            color: p.color.unwrap_or_default(),
        })
        .collect();
    let face_colors: Vec<ffi::OcXdeFaceColor> = flat
        .face_colors
        .iter()
        .map(|&(proto, face, color)| ffi::OcXdeFaceColor { proto, face, color })
        .collect();
    let nodes: Vec<ffi::OcXdeNode> = flat
        .nodes
        .iter()
        .map(|n| ffi::OcXdeNode {
            parent: n.parent.unwrap_or(NONE),
            has_name: n.name.is_some(),
            name: n.name.clone().unwrap_or_default(),
            transform: mat34(&n.transform),
            has_color: n.color.is_some(),
            color: n.color.unwrap_or_default(),
            proto: n.proto.unwrap_or(NONE),
        })
        .collect();
    let sink = progress_sink(ctx);
    let params = ffi::OcXdeWriteParams { schema, unit_mm };
    let status = match format {
        Format::Step => {
            ffi::xde_write_step(file, &params, &protos, &meta, &face_colors, &nodes, &sink)
        }
        Format::Iges => {
            let _lock = global_io_lock();
            ffi::xde_write_iges(file, &params, &protos, &meta, &face_colors, &nodes, &sink)
        }
    }
    .map_err(|e| to_kernel_error(&e))?;
    check(
        &status,
        match format {
            Format::Step => "STEP write",
            Format::Iges => "IGES write",
        },
    )
}

impl Occ {
    /// A copy whose edge `edge` lost its 3D curve and got it back from the readers' repair
    /// (BREP-10 leftover). Test support.
    #[cfg(test)]
    pub(crate) fn guard_probe(&self, edge: u32) -> KResult<Occ> {
        ffi::xde_guard_probe(&self.shape, edge)
            .map(Occ::new)
            .map_err(|e| to_kernel_error(&e))
    }
}
