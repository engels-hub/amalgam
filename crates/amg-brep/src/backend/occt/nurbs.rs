//! The OCCT side of `nurbs_conv` (S20 KH-16): `Geom_BSpline*` round trips and evaluation (the
//! differential-test oracle), wire edge order and `GeomFill_Gordon` (the KH-15 oracle).

use amg_core::{Point3, Vec3};
use amg_occt_sys::ffi;

use super::build::{oc_curve, oc_surface};
use super::{Occ, curve_from, point, surface_from, to_kernel_error};
use crate::KResult;
use crate::error::KernelError;
use crate::geom::{BSplineCurveData, BSplineSurfaceData, Curve3, Surface};

/// What one `GeomFill_Gordon` run gave.
pub(crate) struct RawGordon {
    pub(crate) status: u8,
    pub(crate) approximate: bool,
    pub(crate) max_contact_gap: f64,
    pub(crate) max_profile_deviation: f64,
    pub(crate) max_guide_deviation: f64,
    pub(crate) surface: Option<BSplineSurfaceData>,
}

fn curve_in(c: &BSplineCurveData) -> ffi::OcCurve {
    oc_curve(&Curve3::BSpline(c.clone()), 0.0, 0.0)
}

fn surface_in(s: &BSplineSurfaceData) -> ffi::OcSurface {
    oc_surface(&Surface::BSpline(s.clone()))
}

fn curve_out(raw: ffi::OcCurve) -> KResult<BSplineCurveData> {
    match curve_from(raw) {
        Some((Curve3::BSpline(b), _, _)) => Ok(b),
        _ => Err(KernelError::OpFailed {
            op: "nurbs_conv",
            message: "OCCT returned no B-spline curve".to_owned(),
            offending: Vec::new(),
        }),
    }
}

fn surface_out(raw: ffi::OcSurface) -> KResult<BSplineSurfaceData> {
    match surface_from(raw) {
        Surface::BSpline(b) => Ok(b),
        _ => Err(KernelError::OpFailed {
            op: "nurbs_conv",
            message: "OCCT returned no B-spline surface".to_owned(),
            offending: Vec::new(),
        }),
    }
}

fn vec3(v: ffi::V3) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

pub(crate) fn curve_round_trip(c: &BSplineCurveData) -> KResult<BSplineCurveData> {
    curve_out(ffi::bspline_curve_round_trip(&curve_in(c)).map_err(|e| to_kernel_error(&e))?)
}

pub(crate) fn surface_round_trip(s: &BSplineSurfaceData) -> KResult<BSplineSurfaceData> {
    surface_out(ffi::bspline_surface_round_trip(&surface_in(s)).map_err(|e| to_kernel_error(&e))?)
}

pub(crate) fn curve_not_periodic(c: &BSplineCurveData) -> KResult<BSplineCurveData> {
    curve_out(ffi::bspline_curve_not_periodic(&curve_in(c)).map_err(|e| to_kernel_error(&e))?)
}

pub(crate) fn surface_not_periodic(s: &BSplineSurfaceData) -> KResult<BSplineSurfaceData> {
    surface_out(ffi::bspline_surface_not_periodic(&surface_in(s)).map_err(|e| to_kernel_error(&e))?)
}

pub(crate) fn curve_d2(c: &BSplineCurveData, ts: &[f64]) -> KResult<Vec<(Point3, Vec3, Vec3)>> {
    let raw = ffi::bspline_curve_d2(&curve_in(c), ts).map_err(|e| to_kernel_error(&e))?;
    Ok(raw
        .chunks_exact(3)
        .map(|d| (point(d[0]), vec3(d[1]), vec3(d[2])))
        .collect())
}

pub(crate) fn surface_d2(
    s: &BSplineSurfaceData,
    uvs: &[(f64, f64)],
) -> KResult<Vec<(Point3, [Vec3; 5])>> {
    let flat: Vec<f64> = uvs.iter().flat_map(|&(u, v)| [u, v]).collect();
    let raw = ffi::bspline_surface_d2(&surface_in(s), &flat).map_err(|e| to_kernel_error(&e))?;
    Ok(raw
        .chunks_exact(6)
        .map(|d| {
            (
                point(d[0]),
                [vec3(d[1]), vec3(d[2]), vec3(d[3]), vec3(d[4]), vec3(d[5])],
            )
        })
        .collect())
}

pub(crate) fn gordon(
    profiles: &[BSplineCurveData],
    guides: &[BSplineCurveData],
    tol: f64,
) -> KResult<RawGordon> {
    let p: Vec<ffi::OcCurve> = profiles.iter().map(curve_in).collect();
    let g: Vec<ffi::OcCurve> = guides.iter().map(curve_in).collect();
    let raw = ffi::gordon_surface(&p, &g, tol).map_err(|e| to_kernel_error(&e))?;
    let surface = match raw.surface.kind {
        ffi::OcSurfaceKind::BSpline => Some(surface_out(raw.surface)?),
        _ => None,
    };
    Ok(RawGordon {
        status: raw.status,
        approximate: raw.approximate,
        max_contact_gap: raw.max_contact_gap,
        max_profile_deviation: raw.max_profile_deviation,
        max_guide_deviation: raw.max_guide_deviation,
        surface,
    })
}

impl Occ {
    /// The edges of this wire (or edge) in `BRepTools_WireExplorer` order, each with whether it
    /// runs reversed in the wire (S20 request R16).
    pub(crate) fn wire_order(&self) -> KResult<Vec<(u32, bool)>> {
        let raw =
            ffi::wire_edge_order(&self.shape, self.index()).map_err(|e| to_kernel_error(&e))?;
        Ok(raw.chunks_exact(2).map(|p| (p[0], p[1] != 0)).collect())
    }
}
