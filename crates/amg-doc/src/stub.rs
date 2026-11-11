//! Placeholders for types owned by other work packages (S01 §1 Geometry stubs).
//!
//! Each stub has the name of the real type and is replaced by a re-export when the owning WP
//! lands. Stubs carry no data: nothing may depend on their contents. All of them are
//! `Send + Sync`, which the real types must be as well (S01 §3).

use std::sync::Arc;

use amg_core::{Plane, Transform};
use serde::{Deserialize, Serialize};

/// B-rep shape handle. Cheap to clone; clones share the shape.
///
/// Either a data-less placeholder ([`Shape::new`], what the test features and the stub codecs
/// produce) or a real [`amg_brep::Shape`] ([`Shape::from_brep`], S06 ASM-09). The placeholder
/// stays until the whole stub is replaced by `pub use amg_brep::Shape;`.
///
/// TODO(BREP-02): replace with `pub use amg_brep::Shape;` (must be `Send + Sync`, S01 §3).
/// Code that only calls [`Shape::transformed`] and [`Shape::ptr_eq`] keeps compiling then.
#[derive(Clone, Debug, Default)]
pub struct Shape {
    handle: Arc<Option<amg_brep::Shape>>,
}

impl Shape {
    /// A new, distinct placeholder shape.
    pub fn new() -> Shape {
        Shape::default()
    }

    /// Wraps a real B-rep shape.
    pub fn from_brep(shape: amg_brep::Shape) -> Shape {
        Shape {
            handle: Arc::new(Some(shape)),
        }
    }

    /// The real B-rep shape, or `None` for a placeholder.
    pub fn brep(&self) -> Option<&amg_brep::Shape> {
        self.handle.as_ref().as_ref()
    }

    /// True if both handles refer to the same shape (used for change events, DOC-09).
    pub fn ptr_eq(&self, other: &Shape) -> bool {
        Arc::ptr_eq(&self.handle, &other.handle)
    }

    /// The shape moved by a rigid transform ([`amg_brep::Shape::transformed`]); a placeholder
    /// gives a new placeholder.
    ///
    /// # Errors
    /// `NonRigid` for scale, shear, reflection or non-finite values (also for a placeholder,
    /// so behaviour doesn't depend on the stub), or a kernel error.
    pub fn transformed(&self, t: &Transform) -> Result<Shape, amg_brep::KernelError> {
        match self.brep() {
            Some(s) => s.transformed(t).map(Shape::from_brep),
            None if is_rigid(t) => Ok(Shape::new()),
            None => Err(amg_brep::KernelError::NonRigid),
        }
    }
}

/// Same rule as `amg_brep::Shape::transformed`: finite, orthonormal linear part, det = +1.
fn is_rigid(t: &Transform) -> bool {
    const EPS: f64 = 1e-9;
    let m = t.matrix3;
    if !t.to_cols_array().iter().all(|x| x.is_finite()) {
        return false;
    }
    let rtr = m.transpose() * m;
    let id = amg_core::Mat3::IDENTITY;
    let ortho = (0..3).all(|c| (rtr.col(c) - id.col(c)).abs().max_element() <= EPS);
    ortho && (m.determinant() - 1.0).abs() <= EPS
}

/// Indexed triangle mesh (MESH-16: the real `amg-mesh` type; a mesh body is
/// `BodyGeom::Mesh(Arc<IndexedMesh>)`).
pub use amg_mesh::IndexedMesh;

macro_rules! data_stub {
    ($($(#[$m:meta])* $name:ident;)*) => {$(
        $(#[$m])*
        #[derive(Clone, Debug, Default, PartialEq, Eq)]
        #[non_exhaustive]
        pub struct $name {}
    )*};
}

data_stub! {
    /// Implicit (SDF) node graph. TODO(IMP-01): replace with `pub use amg_implicit::SdfGraph;`.
    SdfGraph;
    /// Subdivision control mesh. TODO(FRM-01): replace with `pub use amg_subd::SubdMesh;`.
    SubdMesh;
}

/// Solved sketch carried in `FeatureOutput::sketch` and read via `RecomputeCtx::sketch` (SK-19;
/// S03 Interfaces). Unlike the other stubs it carries data: `amg-doc` already depends on
/// `amg-sketch`, so the real type lives here until it moves into `amg-sketch`.
#[derive(Clone, Debug)]
pub struct SketchGeom {
    /// World placement of the sketch plane at this recompute.
    pub plane: Plane,
    /// Solved geometry (the last good geometry when `report` is not converged).
    pub sketch: Arc<amg_sketch::Sketch>,
    /// Outcome of the solve that produced (or failed to replace) `sketch`.
    pub report: amg_sketch::solve::SolveReport,
    /// Closed regions of `sketch`.
    pub profiles: Arc<amg_sketch::ProfileSet>,
}

/// Stable key of a sketch curve or point. TODO(SK-01): replace with the `amg-sketch` type
/// (must stay `Ord + Hash + Serialize`, S01 Interfaces).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct SketchEntityKey(pub u64);

/// Stable key of a sketch profile. TODO(SK-06): replace with the `amg-sketch` type.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ProfileKey(pub u64);

/// Material library key. TODO(S17): replace with the `amg-materials` type.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct MaterialKey(pub String);

/// Appearance library key. TODO(S17): replace with the `amg-materials` type.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct AppearanceKey(pub String);

/// Document length unit: re-exported from FND-10 (`amg_core::units::LengthUnit`; same variants
/// and serde names as the former stub, so the file format is unchanged).
pub use amg_core::units::LengthUnit;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shape_handles_compare_by_identity() {
        let a = Shape::new();
        let b = a.clone();
        assert!(a.ptr_eq(&b));
        assert!(!a.ptr_eq(&Shape::new()));
        assert!(a.brep().is_none());
    }

    #[test]
    fn placeholder_transform_follows_the_rigid_rule() {
        let a = Shape::new();
        let rigid =
            Transform::from_rotation_z(0.3) * Transform::from_translation([1.0, 2.0, 3.0].into());
        let b = a.transformed(&rigid).unwrap();
        assert!(!a.ptr_eq(&b), "a moved shape is new geometry");
        let mirror = Transform::from_scale([-1.0, 1.0, 1.0].into());
        assert!(matches!(
            a.transformed(&mirror),
            Err(amg_brep::KernelError::NonRigid)
        ));
        let scale = Transform::from_scale([2.0, 2.0, 2.0].into());
        assert!(a.transformed(&scale).is_err());
    }

    #[test]
    fn length_unit_serde_matches_s00_names() {
        assert_eq!(
            serde_json::to_string(&LengthUnit::Millimetre).unwrap(),
            r#""Millimetre""#
        );
        assert_eq!(LengthUnit::default(), LengthUnit::Millimetre);
    }
}
