//! B-rep kernel: `BrepKernel` trait, OCCT backend, shape handles, tessellation, topology and history.
//!
//! Spec: S02, S20. Lane: A.

mod backend;
mod error;
pub mod geom;
mod index;
mod shape;

pub use amg_core::{Aabb3, Dir3, Line3, Plane, Point2, Point3, Tolerance, Transform, Vec2, Vec3};
pub use amg_jobs::JobCtx;

pub use error::{KResult, KernelError};
pub use shape::{EdgeId, FORMAT_VERSION, FaceId, Shape, ShapeKind, SubKind, SubShape, VertexId};

/// Whether this build links the OCCT backend (cargo feature `occt`).
pub const OCCT_BACKEND: bool = cfg!(feature = "occt");

#[cfg(test)]
mod tests {
    #[test]
    fn backend_flag_matches_occt_sys() {
        assert_eq!(super::OCCT_BACKEND, amg_occt_sys::OCCT_ENABLED);
    }
}
