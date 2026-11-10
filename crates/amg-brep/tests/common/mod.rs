//! Shared by the fixture tests and the `gen_brep_fixtures` generator.
#![allow(dead_code)]

use amg_brep::Shape;
use amg_brep::geom::GeomClass;

/// Fixture names; each is `testdata/brep/<name>.brep`.
pub const FIXTURES: [&str; 11] = [
    "box",
    "cylinder",
    "cone",
    "sphere",
    "torus",
    "filleted_box",
    "plate_with_holes",
    "located_compound",
    "bspline_patch",
    "revolved_profile",
    "cylinder_mirrored",
];

/// Fixtures with open boundaries, where an edge may have a single face.
pub const OPEN_FIXTURES: [&str; 2] = ["bspline_patch", "revolved_profile"];

/// One letter per face class, used in the expectation file.
pub fn class_letter(class: GeomClass) -> char {
    match class {
        GeomClass::Plane => 'P',
        GeomClass::Cylinder => 'Y',
        GeomClass::Cone => 'C',
        GeomClass::Sphere => 'S',
        GeomClass::Torus => 'T',
        GeomClass::BSpline => 'B',
        GeomClass::Other => 'O',
        other => panic!("{other:?} is not a face class"),
    }
}

/// The class letter of every face, in index order.
pub fn face_classes(shape: &Shape) -> String {
    shape
        .faces()
        .map(|f| class_letter(shape.surface(f).class()))
        .collect()
}
