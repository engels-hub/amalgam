//! The OCCT side of the face and wire builders (BREP-03): lowering to the bridge's plain data.

use amg_core::{Plane, Point3};
use amg_occt_sys::ffi;

use super::{Occ, to_kernel_error};
use crate::KResult;
use crate::builders::{BoundsIn, LoopsIn, PlanarIn};
use crate::error::KernelError;
use crate::geom::{Curve3, Surface};

fn v3(p: Point3) -> ffi::V3 {
    ffi::V3 {
        x: p.x,
        y: p.y,
        z: p.z,
    }
}

pub(super) fn oc_frame(p: &Plane) -> ffi::OcFrame {
    ffi::OcFrame {
        origin: v3(p.origin),
        x: v3(p.x.get()),
        y: v3(p.y.get()),
        normal: v3(p.normal.get()),
    }
}

fn oc_curve(c: &Curve3, first: f64, last: f64) -> ffi::OcCurve {
    let mut out = ffi::OcCurve {
        kind: ffi::OcCurveKind::Absent,
        first,
        last,
        frame: ffi::OcFrame::default(),
        a: 0.0,
        b: 0.0,
        degree: 0,
        knots: Vec::new(),
        mults: Vec::new(),
        poles: Vec::new(),
        weights: Vec::new(),
        periodic: false,
    };
    match c {
        Curve3::Line { origin, dir } => {
            out.kind = ffi::OcCurveKind::Line;
            out.frame.origin = v3(*origin);
            out.frame.normal = v3(dir.get());
        }
        Curve3::Circle { frame, radius } => {
            out.kind = ffi::OcCurveKind::Circle;
            out.frame = oc_frame(frame);
            out.a = *radius;
        }
        Curve3::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => {
            out.kind = ffi::OcCurveKind::Ellipse;
            out.frame = oc_frame(frame);
            out.a = *major_radius;
            out.b = *minor_radius;
        }
        Curve3::BSpline(b) => {
            out.kind = ffi::OcCurveKind::BSpline;
            out.degree = u32::from(b.degree);
            out.knots.clone_from(&b.knots);
            out.mults.clone_from(&b.mults);
            out.poles = b.poles.iter().map(|p| v3(*p)).collect();
            out.weights = b.weights.clone().unwrap_or_default();
            out.periodic = b.periodic;
        }
        // Rejected by the builders' validation; the shim raises on `Absent`.
        Curve3::Other => {}
    }
    out
}

fn oc_loops(loops: &LoopsIn) -> ffi::OcLoopsIn {
    ffi::OcLoopsIn {
        edges: loops
            .edges
            .iter()
            .map(|e| ffi::OcEdgeIn {
                curve: oc_curve(&e.curve, e.first, e.last),
                reversed: e.reversed,
            })
            .collect(),
        loop_starts: loops.starts.clone(),
    }
}

fn oc_surface(s: &Surface) -> ffi::OcSurface {
    let mut out = ffi::OcSurface {
        kind: ffi::OcSurfaceKind::Other,
        frame: ffi::OcFrame::default(),
        a: 0.0,
        b: 0.0,
        u_degree: 0,
        v_degree: 0,
        u_knots: Vec::new(),
        u_mults: Vec::new(),
        v_knots: Vec::new(),
        v_mults: Vec::new(),
        poles: Vec::new(),
        nu: 0,
        nv: 0,
        weights: Vec::new(),
        u_periodic: false,
        v_periodic: false,
    };
    match s {
        Surface::Plane(p) => {
            out.kind = ffi::OcSurfaceKind::Plane;
            out.frame = oc_frame(p);
        }
        Surface::Cylinder { frame, radius } => {
            out.kind = ffi::OcSurfaceKind::Cylinder;
            out.frame = oc_frame(frame);
            out.a = *radius;
        }
        Surface::Cone {
            frame,
            ref_radius,
            half_angle,
        } => {
            out.kind = ffi::OcSurfaceKind::Cone;
            out.frame = oc_frame(frame);
            out.a = *ref_radius;
            out.b = *half_angle;
        }
        Surface::Sphere { frame, radius } => {
            out.kind = ffi::OcSurfaceKind::Sphere;
            out.frame = oc_frame(frame);
            out.a = *radius;
        }
        Surface::Torus {
            frame,
            major,
            minor,
        } => {
            out.kind = ffi::OcSurfaceKind::Torus;
            out.frame = oc_frame(frame);
            out.a = *major;
            out.b = *minor;
        }
        Surface::BSpline(b) => {
            out.kind = ffi::OcSurfaceKind::BSpline;
            out.u_degree = u32::from(b.u_degree);
            out.v_degree = u32::from(b.v_degree);
            out.u_knots.clone_from(&b.u_knots);
            out.u_mults.clone_from(&b.u_mults);
            out.v_knots.clone_from(&b.v_knots);
            out.v_mults.clone_from(&b.v_mults);
            out.poles = b.poles.iter().map(|p| v3(*p)).collect();
            out.nu = b.nu;
            out.nv = b.nv;
            out.weights = b.weights.clone().unwrap_or_default();
            out.u_periodic = b.u_periodic;
            out.v_periodic = b.v_periodic;
        }
        // Rejected by the builders' validation; the shim raises on `Other`.
        Surface::Other { .. } => {}
    }
    out
}

/// The shapes and edge maps of one builder call.
fn take_build(build: &ffi::OcBuild) -> KResult<Vec<(Occ, Vec<u32>)>> {
    let count = ffi::build_count(build).map_err(|e| to_kernel_error(&e))?;
    (0..count)
        .map(|i| {
            let shape = ffi::build_shape(build, i).map_err(|e| to_kernel_error(&e))?;
            let map = ffi::build_edge_map(build, i).map_err(|e| to_kernel_error(&e))?;
            Ok((Occ::new(shape), map))
        })
        .collect()
}

fn single(mut built: Vec<(Occ, Vec<u32>)>, op: &'static str) -> KResult<(Occ, Vec<u32>)> {
    match (built.pop(), built.is_empty()) {
        (Some(one), true) => Ok(one),
        _ => Err(KernelError::OpFailed {
            op,
            message: "expected exactly one shape".into(),
            offending: Vec::new(),
        }),
    }
}

impl Occ {
    /// One planar face per input, with the edge index of every input edge.
    pub(crate) fn planar_faces(inputs: &[PlanarIn], tol: f64) -> KResult<Vec<(Occ, Vec<u32>)>> {
        let raw: Vec<ffi::OcPlanarIn> = inputs
            .iter()
            .map(|p| ffi::OcPlanarIn {
                frame: oc_frame(&p.plane),
                loops: oc_loops(&p.loops),
            })
            .collect();
        let build = ffi::make_planar_faces(&raw, tol).map_err(|e| to_kernel_error(&e))?;
        take_build(&build)
    }

    /// A face on `surface` within `bounds`, with the edge index of every loop edge.
    pub(crate) fn face_on_surface(
        surface: &Surface,
        bounds: &BoundsIn,
        tol: f64,
    ) -> KResult<(Occ, Vec<u32>)> {
        let empty = LoopsIn::default();
        let (kind, uv, loops, points, starts): (u8, [f64; 4], &LoopsIn, &[Point3], &[u32]) =
            match bounds {
                BoundsIn::Natural => (0, [0.0; 4], &empty, &[], &[]),
                BoundsIn::UvRect(uv) => (1, *uv, &empty, &[], &[]),
                BoundsIn::Loops(loops) => (2, [0.0; 4], loops, &[], &[]),
                BoundsIn::Polylines { points, starts } => (3, [0.0; 4], &empty, points, starts),
            };
        let uv = ffi::OcUvBounds {
            umin: uv[0],
            umax: uv[1],
            vmin: uv[2],
            vmax: uv[3],
        };
        let points: Vec<ffi::V3> = points.iter().map(|p| v3(*p)).collect();
        let build = ffi::make_face_on_surface(
            &oc_surface(surface),
            kind,
            &uv,
            &oc_loops(loops),
            &points,
            starts,
            tol,
        )
        .map_err(|e| to_kernel_error(&e))?;
        single(take_build(&build)?, "make_face")
    }

    /// A wire through `edges` (one loop), closed when `closed`.
    pub(crate) fn wire(edges: &LoopsIn, closed: bool, tol: f64) -> KResult<(Occ, Vec<u32>)> {
        let build =
            ffi::make_wire(&oc_loops(edges), closed, tol).map_err(|e| to_kernel_error(&e))?;
        single(take_build(&build)?, "make_wire")
    }
}
