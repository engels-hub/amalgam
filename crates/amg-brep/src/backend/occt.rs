//! The OCCT backend of `Shape`. The only module that sees the bridge types.

use std::num::NonZeroUsize;
use std::sync::OnceLock;

use amg_core::{Aabb3, Dir3, Plane, Point3, Transform};
use amg_jobs::JobCtx;
use amg_occt_sys::cxx::UniquePtr;
use amg_occt_sys::{ProgressSink, ProgressTarget, ffi};

use crate::KResult;
use crate::check::CheckReport;
use crate::error::KernelError;
use crate::geom::{BSplineCurveData, BSplineSurfaceData, Curve3, GeomClass, Surface};
use crate::index::RawTopology;
use crate::shape::{ShapeKind, SubKind, SubShape};

/// `occt-<OCC_VERSION_COMPLETE>`: the OCCT part of the shim's version string.
pub(crate) fn occt_version() -> String {
    let full = must(amg_occt_sys::kernel_version(), "kernel_version");
    full.split(';').next().unwrap_or(full).to_owned()
}

/// The process-global OCCT setup of `amg_brep::init` (category G, run once under the global
/// lock): the `tracing` message printer, then the thread pool size.
pub(crate) fn init_process(threads: NonZeroUsize) {
    if let Err(error) = ffi::install_message_printer() {
        tracing::error!(
            "could not install the OCCT message printer: {}",
            error.what()
        );
    }
    let wanted = i32::try_from(threads.get()).unwrap_or(i32::MAX);
    match ffi::set_default_thread_pool(wanted) {
        Ok(got) => tracing::debug!("OCCT thread pool: {got} threads (asked for {wanted})"),
        Err(error) => tracing::error!("could not size the OCCT thread pool: {}", error.what()),
    }
}

/// Reports OCCT progress to a `JobCtx` and asks it for cancellation (S02 §1).
struct JobProgress(JobCtx);

impl ProgressTarget for JobProgress {
    fn show(&self, fraction: f64) {
        #[allow(
            clippy::cast_possible_truncation,
            reason = "progress needs no f64 precision"
        )]
        self.0.progress(fraction as f32);
    }

    fn user_break(&self) -> bool {
        self.0.is_cancelled()
    }
}

/// The progress/cancel bridge for one shim call that takes a `ProgressSink`.
pub(crate) fn progress_sink(ctx: &JobCtx) -> ProgressSink {
    ProgressSink::new(Box::new(JobProgress(ctx.clone())))
}

pub(crate) fn progress_probe(steps: u32, step_micros: u32, ctx: &JobCtx) -> KResult<()> {
    ctx.checkpoint()?;
    let sink = progress_sink(ctx);
    let completed =
        ffi::progress_test_op(&sink, steps, step_micros).map_err(|e| to_kernel_error(&e))?;
    if !completed {
        return Err(KernelError::Cancelled);
    }
    ctx.checkpoint()?;
    Ok(())
}

pub(crate) struct Occ {
    shape: UniquePtr<ffi::OcShape>,
    index: OnceLock<UniquePtr<ffi::OcIndex>>,
    /// Whether the geometry already reads back bit-exactly from `BinTools`. The reader rebuilds
    /// each `gp_Ax3` Y direction as `±(N × X)`, so a reflected frame loses `-0.0` signs on its
    /// first round trip; shapes that came from `read_bin` (or share its TShapes) have none left.
    bin_canonical: bool,
}

/// Unwraps a read-only query on a valid sub-shape. These don't raise on topology that OCCT
/// built or read, so an exception here is a broken invariant, like an out-of-bounds index.
fn must<T>(result: Result<T, amg_occt_sys::cxx::Exception>, what: &str) -> T {
    match result {
        Ok(value) => value,
        Err(error) => panic!("OCCT raised during {what}: {error}"),
    }
}

/// Splits the shim's `"<ExceptionType>: <message>"`.
fn to_kernel_error(error: &amg_occt_sys::cxx::Exception) -> KernelError {
    let text = error.what();
    match text.split_once(": ") {
        Some((kind, message)) => KernelError::Occt {
            kind: kind.to_owned(),
            message: message.to_owned(),
        },
        None => KernelError::Occt {
            kind: "unknown".to_owned(),
            message: text.to_owned(),
        },
    }
}

fn sub_kind(kind: SubKind) -> u8 {
    match kind {
        SubKind::Face => 0,
        SubKind::Edge => 1,
        SubKind::Vertex => 2,
    }
}

fn point(v: ffi::V3) -> Point3 {
    Point3::new(v.x, v.y, v.z)
}

fn dir(v: ffi::V3) -> Dir3 {
    Dir3::new_unchecked(point(v))
}

fn plane(frame: &ffi::OcFrame) -> Plane {
    Plane {
        origin: point(frame.origin),
        x: dir(frame.x),
        y: dir(frame.y),
        normal: dir(frame.normal),
    }
}

fn weights(values: Vec<f64>) -> Option<Vec<f64>> {
    (!values.is_empty()).then_some(values)
}

fn surface_from(raw: ffi::OcSurface) -> Surface {
    match raw.kind {
        ffi::OcSurfaceKind::Plane => Surface::Plane(plane(&raw.frame)),
        ffi::OcSurfaceKind::Cylinder => Surface::Cylinder {
            frame: plane(&raw.frame),
            radius: raw.a,
        },
        ffi::OcSurfaceKind::Cone => Surface::Cone {
            frame: plane(&raw.frame),
            ref_radius: raw.a,
            half_angle: raw.b,
        },
        ffi::OcSurfaceKind::Sphere => Surface::Sphere {
            frame: plane(&raw.frame),
            radius: raw.a,
        },
        ffi::OcSurfaceKind::Torus => Surface::Torus {
            frame: plane(&raw.frame),
            major: raw.a,
            minor: raw.b,
        },
        ffi::OcSurfaceKind::BSpline => Surface::BSpline(BSplineSurfaceData {
            u_degree: raw.u_degree as u8,
            v_degree: raw.v_degree as u8,
            u_knots: raw.u_knots,
            u_mults: raw.u_mults,
            v_knots: raw.v_knots,
            v_mults: raw.v_mults,
            poles: raw.poles.into_iter().map(point).collect(),
            nu: raw.nu,
            nv: raw.nv,
            weights: weights(raw.weights),
            u_periodic: raw.u_periodic,
            v_periodic: raw.v_periodic,
        }),
        _ => Surface::Other {
            class: GeomClass::Other,
        },
    }
}

fn curve_from(raw: ffi::OcCurve) -> Option<(Curve3, f64, f64)> {
    let (first, last) = (raw.first, raw.last);
    let curve = match raw.kind {
        ffi::OcCurveKind::Line => Curve3::Line {
            origin: point(raw.frame.origin),
            dir: dir(raw.frame.normal),
        },
        ffi::OcCurveKind::Circle => Curve3::Circle {
            frame: plane(&raw.frame),
            radius: raw.a,
        },
        ffi::OcCurveKind::Ellipse => Curve3::Ellipse {
            frame: plane(&raw.frame),
            major_radius: raw.a,
            minor_radius: raw.b,
        },
        ffi::OcCurveKind::BSpline => Curve3::BSpline(BSplineCurveData {
            degree: raw.degree as u8,
            knots: raw.knots,
            mults: raw.mults,
            poles: raw.poles.into_iter().map(point).collect(),
            weights: weights(raw.weights),
            periodic: raw.periodic,
        }),
        ffi::OcCurveKind::Other => Curve3::Other,
        _ => return None,
    };
    Some((curve, first, last))
}

fn mat34(t: &Transform) -> ffi::OcMat34 {
    let (x, y, z, d) = (
        t.matrix3.x_axis,
        t.matrix3.y_axis,
        t.matrix3.z_axis,
        t.translation,
    );
    ffi::OcMat34 {
        m: [
            x.x, y.x, z.x, d.x, //
            x.y, y.y, z.y, d.y, //
            x.z, y.z, z.z, d.z,
        ],
    }
}

impl Occ {
    fn new(shape: UniquePtr<ffi::OcShape>) -> Occ {
        Occ {
            shape,
            index: OnceLock::new(),
            bin_canonical: false,
        }
    }

    /// A shape sharing `self`'s TShapes (sub-shape, solid, location change).
    fn derived(&self, shape: UniquePtr<ffi::OcShape>) -> Occ {
        Occ {
            bin_canonical: self.bin_canonical,
            ..Occ::new(shape)
        }
    }

    fn index(&self) -> &ffi::OcIndex {
        self.index
            .get_or_init(|| must(ffi::build_index(&self.shape), "build_index"))
    }

    #[cfg(test)]
    pub(crate) fn test_box(dx: f64, dy: f64, dz: f64) -> Occ {
        Occ::new(must(ffi::make_box(dx, dy, dz), "make_box"))
    }

    #[cfg(test)]
    pub(crate) fn test_open_wire_face() -> Occ {
        Occ::new(must(ffi::make_open_wire_face(), "make_open_wire_face"))
    }

    #[cfg(feature = "fixture-gen")]
    pub(crate) fn fixture(name: &str) -> KResult<Occ> {
        amg_occt_sys::fixture_ffi::fixture_shape(name)
            .map(Occ::new)
            .map_err(|e| to_kernel_error(&e))
    }

    pub(crate) fn read_bin(data: &[u8]) -> KResult<Occ> {
        ffi::shape_read_bin(data)
            .map(|shape| Occ {
                bin_canonical: true,
                ..Occ::new(shape)
            })
            .map_err(|e| KernelError::Format(e.what().to_owned()))
    }

    pub(crate) fn read_text(data: &[u8]) -> KResult<Occ> {
        ffi::shape_read_text(data)
            .map(Occ::new)
            .map_err(|e| KernelError::Format(e.what().to_owned()))
    }

    pub(crate) fn kind(&self) -> ShapeKind {
        match must(ffi::shape_kind(&self.shape), "kind") {
            0 => ShapeKind::Compound,
            1 => ShapeKind::CompSolid,
            2 => ShapeKind::Solid,
            3 => ShapeKind::Shell,
            4 => ShapeKind::Face,
            5 => ShapeKind::Wire,
            6 => ShapeKind::Edge,
            7 => ShapeKind::Vertex,
            other => panic!("unexpected TopAbs_ShapeEnum value {other}"),
        }
    }

    pub(crate) fn is_null(&self) -> bool {
        must(ffi::shape_is_null(&self.shape), "is_null")
    }

    pub(crate) fn is_same(&self, other: &Occ) -> bool {
        must(ffi::shape_is_same(&self.shape, &other.shape), "is_same")
    }

    pub(crate) fn topology(&self) -> RawTopology {
        let raw = must(ffi::index_topology(self.index()), "index_topology");
        RawTopology {
            n_faces: raw.n_faces,
            n_edges: raw.n_edges,
            n_vertices: raw.n_vertices,
            face_wire_starts: raw.face_wire_starts,
            wire_edge_starts: raw.wire_edge_starts,
            wire_edges: raw.wire_edges,
            edge_vertices: raw.edge_vertices,
            vertex_edge_starts: raw.vertex_edge_starts,
            vertex_edges: raw.vertex_edges,
        }
    }

    pub(crate) fn solids(&self) -> Vec<Occ> {
        let index = self.index();
        let count = must(ffi::index_solid_count(index), "solid count");
        (0..count)
            .map(|i| self.derived(must(ffi::index_solid(index, i), "solid")))
            .collect()
    }

    pub(crate) fn sub_shape(&self, kind: SubKind, index: u32) -> KResult<Occ> {
        ffi::index_sub_shape(self.index(), sub_kind(kind), index)
            .map(|shape| self.derived(shape))
            .map_err(|e| to_kernel_error(&e))
    }

    pub(crate) fn bbox(&self) -> Aabb3 {
        let raw = must(ffi::shape_bbox(&self.shape), "bbox");
        if raw.is_void {
            Aabb3::EMPTY
        } else {
            Aabb3::new(point(raw.min), point(raw.max))
        }
    }

    pub(crate) fn surface(&self, face: u32) -> Surface {
        surface_from(must(ffi::face_surface(self.index(), face), "face_surface"))
    }

    pub(crate) fn face_reversed(&self, face: u32) -> bool {
        must(
            ffi::face_is_reversed(self.index(), face),
            "face_is_reversed",
        )
    }

    pub(crate) fn face_uv_bounds(&self, face: u32) -> [f64; 4] {
        let raw = must(ffi::face_uv_bounds(self.index(), face), "face_uv_bounds");
        [raw.umin, raw.umax, raw.vmin, raw.vmax]
    }

    pub(crate) fn face_area(&self, face: u32) -> f64 {
        must(ffi::face_area(self.index(), face), "face_area")
    }

    pub(crate) fn curve(&self, edge: u32) -> Option<(Curve3, f64, f64)> {
        curve_from(must(ffi::edge_curve(self.index(), edge), "edge_curve"))
    }

    pub(crate) fn is_seam(&self, edge: u32, face: u32) -> bool {
        must(ffi::edge_is_seam(self.index(), edge, face), "edge_is_seam")
    }

    pub(crate) fn vertex_point(&self, vertex: u32) -> Point3 {
        point(must(
            ffi::vertex_point(self.index(), vertex),
            "vertex_point",
        ))
    }

    pub(crate) fn tolerance(&self, kind: SubKind, index: u32) -> f64 {
        must(
            ffi::index_tolerance(self.index(), sub_kind(kind), index),
            "tolerance",
        )
    }

    pub(crate) fn max_tolerance(&self) -> f64 {
        must(ffi::shape_max_tolerance(&self.shape), "shape_max_tolerance")
    }

    pub(crate) fn check(&self) -> CheckReport {
        let raw = match ffi::check_shape(&self.shape, self.index()) {
            Ok(raw) => raw,
            Err(error) => {
                return CheckReport {
                    valid: false,
                    problems: vec![(None, format!("CheckFail: {}", error.what()))],
                };
            }
        };
        let problems = raw
            .problems
            .into_iter()
            .map(|problem| {
                let kind = match problem.kind {
                    0 => Some(SubKind::Face),
                    1 => Some(SubKind::Edge),
                    2 => Some(SubKind::Vertex),
                    _ => None,
                };
                let sub = kind.map(|kind| SubShape {
                    kind,
                    index: problem.index,
                });
                (sub, problem.status)
            })
            .collect();
        CheckReport {
            valid: raw.valid,
            problems,
        }
    }

    pub(crate) fn estimated_bytes(&self) -> u64 {
        must(
            ffi::index_estimated_bytes(self.index()),
            "index_estimated_bytes",
        )
    }

    pub(crate) fn volume(&self) -> KResult<f64> {
        ffi::volume(&self.shape).map_err(|e| to_kernel_error(&e))
    }

    pub(crate) fn moved(&self, transform: &Transform) -> KResult<Occ> {
        ffi::shape_moved(&self.shape, &mat34(transform))
            .map(|shape| self.derived(shape))
            .map_err(|e| to_kernel_error(&e))
    }

    /// `BinTools` bytes that read back to a shape writing the same bytes (S02 Done-when 3).
    pub(crate) fn write_bin(&self) -> KResult<Vec<u8>> {
        let first = ffi::shape_write_bin(&self.shape).map_err(|e| to_kernel_error(&e))?;
        if self.bin_canonical {
            return Ok(first.as_bytes().to_vec());
        }
        // One round trip reaches the reader's fixed point (see `bin_canonical`).
        let reread = ffi::shape_read_bin(first.as_bytes()).map_err(|e| to_kernel_error(&e))?;
        ffi::shape_write_bin(&reread)
            .map(|bytes| bytes.as_bytes().to_vec())
            .map_err(|e| to_kernel_error(&e))
    }

    pub(crate) fn write_text(&self) -> KResult<Vec<u8>> {
        ffi::shape_write_text(&self.shape)
            .map(|bytes| bytes.as_bytes().to_vec())
            .map_err(|e| to_kernel_error(&e))
    }
}
