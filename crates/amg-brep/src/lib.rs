//! B-rep kernel: `BrepKernel` trait, OCCT backend, shape handles, tessellation, topology and history.
//!
//! Spec: S02, S20. Lane: A.

mod backend;
mod blend;
mod boolean;
mod builders;
pub mod capture;
mod check;
mod config;
mod error;
mod extract;
mod foreign;
pub mod geom;
mod heal;
mod history;
mod index;
pub mod io;
mod kernel;
mod local;
mod lock;
pub mod nurbs_conv;
mod offset;
mod ops;
pub mod opts;
mod query;
pub mod robust;
mod shape;
mod sweep;
mod tess;

pub use amg_core::{
    Aabb3, Dir3, Line3, Mat3, Plane, Point2, Point3, Ray3, Tolerance, Transform, Vec2, Vec3,
};
pub use amg_jobs::JobCtx;

pub use check::{CheckOpts, CheckReport};
pub use config::{ENV_FAST_PATHS, ENV_OCCT_THREADS, ENV_VALIDATE, KernelConfig, Validate};
pub use error::{KResult, KernelError};
pub use foreign::{ForeignModel, ForeignNode, ForeignPrototype, ImportOpts, StepSchema};
pub use history::{
    FaceBuild, InputSub, KernelWarning, OpResult, OpRole, Relation, SPLIT_QUANTUM, ShapeHistory,
    SplitKey, ThinFace, split_order_key,
};
pub use kernel::{BrepKernel, OcctKernel, init, kernel_version};
pub use opts::{
    BoolOp, BoolOpts, ChamferSpec, DraftOpts, FilletOpts, FilletShape, FilletSpec, Glue, HealOpts,
    HelixSpec, LoftSection, OffsetJoin, PipeMode, PipeOpts, PrismExtent, PrismOpts, SewOpts,
    ShellOpts, TessQuality, ThruSectionsOpts, Transition, UnifyOpts,
};
pub use query::{
    Classification, DistanceResult, EdgeEnds, MASS_PROPS_EPS, MassProps, RayHit, SubShapeGeom,
    SurfacePoint,
};
pub use robust::POLICY_VERSION;
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
