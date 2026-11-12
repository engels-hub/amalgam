//! B-rep kernel: `BrepKernel` trait, OCCT backend, shape handles, tessellation, topology and history.
//!
//! Spec: S02, S20. Lane: A.

mod backend;
mod builders;
mod check;
mod config;
mod error;
pub mod geom;
mod history;
mod index;
mod kernel;
mod lock;
pub mod opts;
mod shape;
mod tess;

pub use amg_core::{Aabb3, Dir3, Line3, Plane, Point2, Point3, Tolerance, Transform, Vec2, Vec3};
pub use amg_jobs::JobCtx;

pub use check::{CheckOpts, CheckReport};
pub use config::{ENV_OCCT_THREADS, ENV_VALIDATE, KernelConfig, Validate};
pub use error::{KResult, KernelError};
pub use history::{FaceBuild, InputSub, KernelWarning, OpResult, OpRole, Relation, ShapeHistory};
pub use kernel::{BrepKernel, OcctKernel, POLICY_VERSION, init, kernel_version};
pub use opts::{
    BoolOp, BoolOpts, ChamferSpec, DraftOpts, FilletOpts, FilletShape, FilletSpec, Glue, HealOpts,
    HelixSpec, LoftSection, OffsetJoin, PipeMode, PipeOpts, PrismExtent, PrismOpts, SewOpts,
    ShellOpts, TessQuality, ThruSectionsOpts, Transition, UnifyOpts,
};
pub use shape::{EdgeId, FORMAT_VERSION, FaceId, Shape, ShapeKind, SubKind, SubShape, VertexId};
pub use tess::{EdgePolyline, TessMesh, TessProfile};

/// Whether this build links the OCCT backend (cargo feature `occt`).
pub const OCCT_BACKEND: bool = cfg!(feature = "occt");

#[cfg(test)]
mod tests {
    #[test]
    fn backend_flag_matches_occt_sys() {
        assert_eq!(super::OCCT_BACKEND, amg_occt_sys::OCCT_ENABLED);
    }
}
