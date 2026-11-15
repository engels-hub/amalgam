//! The OCCT side of the measurement queries (BREP-09). All category R.

use amg_core::{Point2, Point3, Vec3};
use amg_occt_sys::ffi;

use super::{Occ, point, to_kernel_error};
use crate::KResult;
use crate::query::{
    MassBasis, RawCurvature, RawDistance, RawHit, RawMass, RawSurfacePoint, tangent,
};
use crate::shape::{SubKind, SubShape};
use crate::{Classification, EdgeEnds};

fn v3(p: Point3) -> ffi::V3 {
    ffi::V3 {
        x: p.x,
        y: p.y,
        z: p.z,
    }
}

fn support(kind: u8, index: u32) -> Option<SubShape> {
    let kind = match kind {
        0 => SubKind::Face,
        1 => SubKind::Edge,
        2 => SubKind::Vertex,
        _ => return None,
    };
    Some(SubShape { kind, index })
}

fn distance_from(raw: &ffi::OcDistance) -> RawDistance {
    RawDistance {
        distance: raw.distance,
        on_a: point(raw.on_a),
        on_b: point(raw.on_b),
        sub_a: support(raw.kind_a, raw.index_a),
        sub_b: support(raw.kind_b, raw.index_b),
        uv_a: Point2::new(raw.u_a, raw.v_a),
    }
}

impl Occ {
    pub(crate) fn mass_props(&self, eps: f64) -> KResult<RawMass> {
        let raw =
            ffi::mass_props(&self.shape, self.index(), eps).map_err(|e| to_kernel_error(&e))?;
        let basis = match raw.basis {
            0 => MassBasis::Volume,
            1 => MassBasis::Surface,
            2 => MassBasis::Linear,
            _ => MassBasis::Nothing,
        };
        Ok(RawMass {
            basis,
            volume: raw.volume,
            area: raw.area,
            centroid: point(raw.centroid),
            inertia: raw.inertia,
        })
    }

    pub(crate) fn has_solids(&self) -> bool {
        super::must(ffi::index_solid_count(self.index()), "solid count") > 0
    }

    pub(crate) fn classify(&self, p: Point3, tol: f64) -> KResult<Classification> {
        let code =
            ffi::classify_point(self.index(), &v3(p), tol).map_err(|e| to_kernel_error(&e))?;
        Ok(match code {
            0 => Classification::Inside,
            2 => Classification::OnBoundary,
            _ => Classification::Outside,
        })
    }

    pub(crate) fn distance(&self, other: &Occ) -> KResult<RawDistance> {
        ffi::distance_shapes(&self.shape, self.index(), &other.shape, other.index())
            .map(|raw| distance_from(&raw))
            .map_err(|e| to_kernel_error(&e))
    }

    pub(crate) fn closest_point(&self, p: Point3) -> KResult<RawDistance> {
        ffi::closest_point_on(&self.shape, self.index(), &v3(p))
            .map(|raw| distance_from(&raw))
            .map_err(|e| to_kernel_error(&e))
    }

    pub(crate) fn ray_cast(
        &self,
        origin: Point3,
        dir: Vec3,
        max_t: f64,
        tol: f64,
    ) -> KResult<Option<RawHit>> {
        let raw = ffi::ray_cast_faces(self.index(), &v3(origin), &v3(dir), max_t, tol)
            .map_err(|e| to_kernel_error(&e))?;
        Ok(raw.hit.then(|| RawHit {
            t: raw.t,
            point: point(raw.point),
            face: raw.face,
            uv: Point2::new(raw.u, raw.v),
        }))
    }

    pub(crate) fn eval_face(&self, face: u32, uv: Point2, tol: f64) -> KResult<RawSurfacePoint> {
        let raw =
            ffi::eval_face(self.index(), face, uv.x, uv.y, tol).map_err(|e| to_kernel_error(&e))?;
        let curvature = (raw.normal_defined && raw.curvature_defined).then(|| RawCurvature {
            k_min: raw.k_min,
            k_max: raw.k_max,
            dirs: (!raw.umbilic).then(|| (point(raw.dir_max), point(raw.dir_min))),
        });
        Ok(RawSurfacePoint {
            point: point(raw.point),
            d1u: point(raw.d1u),
            d1v: point(raw.d1v),
            normal: raw.normal_defined.then(|| point(raw.normal)),
            curvature,
        })
    }

    pub(crate) fn project_on_face(&self, face: u32, p: Point3, tol: f64) -> KResult<RawDistance> {
        ffi::project_on_face(self.index(), face, &v3(p), tol)
            .map(|raw| distance_from(&raw))
            .map_err(|e| to_kernel_error(&e))
    }

    /// `None` for a degenerated edge or a vanishing end derivative.
    pub(crate) fn edge_ends(&self, edge: u32) -> KResult<Option<EdgeEnds>> {
        let raw = ffi::edge_ends(self.index(), edge).map_err(|e| to_kernel_error(&e))?;
        if raw.degenerated {
            return Ok(None);
        }
        let ends = (|| -> Option<EdgeEnds> {
            Some(EdgeEnds {
                p0: point(raw.p0),
                t0: tangent(point(raw.t0))?,
                p1: point(raw.p1),
                t1: tangent(point(raw.t1))?,
            })
        })();
        Ok(ends)
    }
}
