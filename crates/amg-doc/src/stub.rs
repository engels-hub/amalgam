//! Placeholders for types owned by other work packages (S01 §1 Geometry stubs).
//!
//! Each stub has the name of the real type and is replaced by a re-export when the owning WP
//! lands. Stubs carry no data: nothing may depend on their contents. All of them are
//! `Send + Sync`, which the real types must be as well (S01 §3).

use std::sync::Arc;

use serde::{Deserialize, Serialize};

/// B-rep shape handle. Cheap to clone; clones share the shape.
///
/// TODO(BREP-02): replace with `pub use amg_brep::Shape;` (must be `Send + Sync`, S01 §3).
#[derive(Clone, Debug, Default)]
pub struct Shape {
    handle: Arc<()>,
}

impl Shape {
    /// A new, distinct placeholder shape.
    pub fn new() -> Shape {
        Shape::default()
    }

    /// True if both handles refer to the same shape (used for change events, DOC-09).
    pub fn ptr_eq(&self, other: &Shape) -> bool {
        Arc::ptr_eq(&self.handle, &other.handle)
    }
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
    /// Solved sketch geometry. TODO(SK-01): replace with the `amg-sketch` type.
    SketchGeom;
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
