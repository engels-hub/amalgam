#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "tests")]

use std::sync::Arc;

use amg_core::units::LengthUnit;
use amg_core::{Point3, Transform, Vec3};
use amg_doc::Feature;
use amg_io::{Format, MaterialSlot, NO_SLOT, UpAxis};
use amg_mesh::{IndexedMesh, Rgba8};

use super::feature::{face_slots, set_face_slots};
use super::*;

fn tri() -> IndexedMesh {
    IndexedMesh::new(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ],
        vec![[0, 2, 1], [0, 1, 3]],
    )
    .unwrap()
}

fn feature(unit: LengthUnit, up: UpAxis, t: Transform) -> ImportMeshFeature {
    ImportMeshFeature {
        source: ImportSource::from_bytes("a.stl", Format::Stl, None, b"bytes"),
        unit,
        up_axis: up,
        placement: Transform::IDENTITY,
        bodies: vec![ImportedBodySpec {
            slot: 0,
            name: "a".to_owned(),
            transform: t,
            material: None,
            filament: None,
        }],
        materials: Vec::new(),
    }
}

#[test]
fn unit_scale_is_exact_per_coordinate() {
    let f = feature(LengthUnit::Inch, UpAxis::Z, Transform::IDENTITY);
    let raw = tri();
    let m = f.body_mesh(&raw, &f.bodies[0]);
    for (a, b) in raw.positions.iter().zip(&m.positions) {
        assert_eq!(b.x, a.x * 25.4);
        assert_eq!(b.y, a.y * 25.4);
        assert_eq!(b.z, a.z * 25.4);
    }
}

#[test]
fn transform_order_matches_scale_first() {
    let t = Transform::from_translation(Vec3::new(1.0, 2.0, 3.0));
    let f = feature(LengthUnit::Centimetre, UpAxis::Y, t);
    let raw = tri();
    let m = f.body_mesh(&raw, &f.bodies[0]);
    // placement · up · scale · T applied to raw points.
    for (a, b) in raw.positions.iter().zip(&m.positions) {
        let moved = t.transform_point3(*a) * 10.0;
        let want = UpAxis::Y.to_z_up().transform_point3(moved);
        assert!((want - *b).length() < 1e-12, "{want:?} vs {b:?}");
    }
}

#[test]
fn validate_rejects_bad_slots() {
    let mut f = feature(LengthUnit::Millimetre, UpAxis::Z, Transform::IDENTITY);
    assert!(f.validate().is_ok());
    f.bodies.push(f.bodies[0].clone());
    assert!(f.validate().is_err());
    f.bodies.pop();
    f.bodies[0].material = Some(0);
    assert!(f.validate().is_err());
    f.materials.push(MaterialSlot {
        name: "red".to_owned(),
        color: Rgba8([255, 0, 0, 255]),
        filament: None,
    });
    assert!(f.validate().is_ok());
}

#[test]
fn face_slots_round_trip_through_tri_properties() {
    let mut m = tri();
    assert_eq!(face_slots(&m), None);
    set_face_slots(&mut m, &[3, NO_SLOT]);
    assert_eq!(face_slots(&m), Some(vec![3, NO_SLOT]));
}

#[test]
fn feature_serializes_round_trip() {
    let f = feature(LengthUnit::Metre, UpAxis::Y, Transform::IDENTITY);
    let json = serde_json::to_string(&f).unwrap();
    let back: ImportMeshFeature = serde_json::from_str(&json).unwrap();
    assert_eq!(back, f);
    let _ = Arc::new(back);
}
