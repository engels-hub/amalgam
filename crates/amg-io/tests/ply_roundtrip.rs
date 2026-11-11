//! IO-13 Done-when 3: PLY write→read round trips (`double` bit-exact, `float` within one f32
//! ulp with `PrecisionLoss` reported), plus the writer's own behaviour.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test code; a failed expectation is a test failure"
)]

mod ply_common;

use std::sync::Arc;

use amg_core::{Point3, Transform, Vec3, units::LengthUnit};
use amg_io::{
    ExportOptions, Format, ImportOptions, IoError, IoWarning, MaterialSlot, PlyWriteOptions, Scene,
    SceneNode, SceneObject, Sink, UnitChoice, ValidityPolicy, export,
};
use amg_mesh::corpus::bench::{Displacement, geodesic_sphere};
use amg_mesh::{IndexedMesh, Rgba8};
use ply_common::*;
use proptest::prelude::*;

fn scene_of(m: IndexedMesh) -> Scene {
    let mut s = Scene::default();
    s.objects.push(SceneObject::new(Arc::new(m)));
    s.roots.push(SceneNode::mesh(0, Transform::IDENTITY));
    s
}

fn opts(binary: bool, double: bool) -> ExportOptions {
    ExportOptions {
        validity: Some(ValidityPolicy::AllowInvalid),
        ply: PlyWriteOptions { binary, double },
        ..ExportOptions::default()
    }
}

fn write(s: &Scene, o: &ExportOptions) -> (Vec<u8>, amg_io::ExportReport) {
    let mut out = Vec::new();
    let r = export(s, Sink::Memory(&mut out), Format::Ply, o, &ctx()).expect("export");
    assert_eq!(r.bytes, out.len() as u64);
    (out, r)
}

/// A displaced sphere of radius 100 mm: f32 cannot hold its coordinates exactly.
fn sphere() -> IndexedMesh {
    geodesic_sphere(
        6,
        100.0,
        Some(Displacement {
            amplitude: 0.37,
            seed: 7,
        }),
    )
}

fn ulp32(x: f64) -> f64 {
    let f = (x as f32).abs();
    f64::from(f32::from_bits(f.to_bits() + 1) - f)
}

#[test]
fn double_round_trips_bit_exact() {
    let m = sphere();
    let s = scene_of(m.clone());
    for binary in [true, false] {
        let (bytes, r) = write(&s, &opts(binary, true));
        assert!(
            !r.warnings
                .iter()
                .any(|w| matches!(w, IoWarning::PrecisionLoss { .. })),
            "{:?}",
            r.warnings
        );
        let i = run(&bytes).expect("read back");
        let b = &i.scene.objects[0].mesh;
        assert_eq!(b.tris, m.tris, "binary={binary}");
        assert_eq!(b.positions.len(), m.positions.len());
        for (p, q) in b.positions.iter().zip(&m.positions) {
            assert_eq!(
                p.to_array().map(f64::to_bits),
                q.to_array().map(f64::to_bits)
            );
        }
        assert!(i.report.warnings.is_empty());
    }
}

#[test]
fn float_round_trips_within_one_ulp_and_reports_precision_loss() {
    let m = sphere();
    let s = scene_of(m.clone());
    for binary in [true, false] {
        let (bytes, r) = write(&s, &opts(binary, false));
        let loss: Vec<f64> = r
            .warnings
            .iter()
            .filter_map(|w| match w {
                IoWarning::PrecisionLoss { max_error_mm } => Some(*max_error_mm),
                _ => None,
            })
            .collect();
        assert_eq!(loss.len(), 1, "{:?}", r.warnings);
        assert!(loss[0] > 1e-6 && loss[0] < 1e-5, "{}", loss[0]);
        let i = run(&bytes).expect("read back");
        let b = &i.scene.objects[0].mesh;
        assert_eq!(b.tris, m.tris);
        let mut worst = 0.0f64;
        for (p, q) in b.positions.iter().zip(&m.positions) {
            for (x, y) in p.to_array().into_iter().zip(q.to_array()) {
                let err = (x - y).abs();
                assert!(err <= ulp32(y), "{x} vs {y}");
                worst = worst.max(err);
            }
        }
        // The warning reports the worst error that happened.
        assert!((worst - loss[0]).abs() < 1e-12, "{worst} vs {}", loss[0]);
    }
    // Small coordinates are exact enough that nothing is reported.
    let small = IndexedMesh::new(
        vec![
            Point3::new(0.5, 0.25, 0.125),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 2.0),
        ],
        vec![[0, 1, 2]],
    )
    .expect("mesh");
    let (_, r) = write(&scene_of(small), &opts(true, false));
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
}

#[test]
fn vertex_colors_round_trip() {
    let mut m = sphere();
    let n = m.vertex_count();
    m.attrs.vertex_colors = Some(
        (0..n)
            .map(|i| {
                Rgba8([
                    (i % 256) as u8,
                    (i * 7 % 256) as u8,
                    (i * 13 % 256) as u8,
                    (255 - i % 200) as u8,
                ])
            })
            .collect(),
    );
    let s = scene_of(m.clone());
    for binary in [true, false] {
        let (bytes, _) = write(&s, &opts(binary, true));
        let o = oracle(&bytes);
        assert_eq!(o.colors.as_ref().expect("oracle colours").len(), n);
        let i = run(&bytes).expect("read back");
        assert_eq!(
            i.scene.objects[0].mesh.attrs.vertex_colors,
            m.attrs.vertex_colors
        );
        assert_eq!(
            o.colors.unwrap(),
            m.attrs
                .vertex_colors
                .as_ref()
                .unwrap()
                .iter()
                .map(|c| c.0)
                .collect::<Vec<_>>()
        );
    }
}

#[test]
fn objects_are_merged_in_world_space_and_units_scale() {
    let a = IndexedMesh::new(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
    )
    .unwrap();
    let mut s = Scene::default();
    s.objects.push(SceneObject::new(Arc::new(a.clone())));
    s.objects.push(SceneObject::new(Arc::new(a)));
    s.roots.push(SceneNode::mesh(0, Transform::IDENTITY));
    s.roots.push(SceneNode::mesh(
        1,
        Transform::from_translation(Vec3::new(10.0, 0.0, 0.0)),
    ));
    let (bytes, r) = write(&s, &opts(true, true));
    assert_eq!(r.triangles, 2);
    let i = run(&bytes).expect("read back");
    let m = &i.scene.objects[0].mesh;
    assert_eq!((m.vertex_count(), m.face_count()), (6, 2));
    assert_eq!(m.positions[4].x, 11.0);
    assert_eq!(m.tris[1], [3, 4, 5]);
    // Inches: the numbers are scaled, and reading as inches gives the mm back.
    let o = ExportOptions {
        unit: LengthUnit::Inch,
        ..opts(true, true)
    };
    let (bytes, _) = write(&s, &o);
    assert!(String::from_utf8_lossy(&bytes[..120]).contains("unit: in"));
    let back = amg_io::import(
        amg_io::Source::Bytes(&bytes),
        Some(Format::Ply),
        &ImportOptions {
            unit: UnitChoice::Assume(LengthUnit::Inch),
            ..ImportOptions::default()
        },
        &ctx(),
    )
    .expect("import");
    let p = back.scene.objects[0].mesh.positions[4].x;
    assert!((p - 11.0).abs() < 1e-12, "{p}");
}

#[test]
fn output_is_deterministic_and_the_same_for_every_sink() {
    let s = scene_of(sphere());
    let a = write(&s, &opts(true, false)).0;
    let b = write(&s, &opts(true, false)).0;
    assert_eq!(a, b);
    let dir = tempfile::tempdir().expect("tempdir");
    let p = dir.path().join("x.ply");
    let r = export(&s, Sink::Path(&p), Format::Ply, &opts(true, false), &ctx()).expect("path");
    assert_eq!(std::fs::read(&p).unwrap(), a);
    assert_eq!(r.files, vec![p.clone()]);
    assert!(!dir.path().join("x.ply.tmp").exists());
    // ASCII has no trailing garbage and ends with a newline.
    let t = write(&s, &opts(false, true)).0;
    assert!(t.ends_with(b"\n"));
    // Default options: binary LE, float.
    let d = write(
        &s,
        &ExportOptions {
            validity: Some(ValidityPolicy::AllowInvalid),
            ..ExportOptions::default()
        },
    )
    .0;
    assert!(String::from_utf8_lossy(&d[..80]).contains("binary_little_endian"));
    assert!(String::from_utf8_lossy(&d[..300]).contains("property float x"));
}

#[test]
fn writer_errors_and_warnings() {
    // Empty scene.
    let e = export(
        &Scene::default(),
        Sink::Memory(&mut Vec::new()),
        Format::Ply,
        &opts(true, false),
        &ctx(),
    );
    assert!(matches!(e, Err(IoError::Empty)));
    // A non-finite coordinate.
    let bad = IndexedMesh::new(
        vec![Point3::new(f64::NAN, 0.0, 0.0), Point3::X, Point3::Y],
        vec![[0, 1, 2]],
    )
    .unwrap();
    let e = export(
        &scene_of(bad),
        Sink::Memory(&mut Vec::new()),
        Format::Ply,
        &opts(true, true),
        &ctx(),
    );
    assert!(matches!(e, Err(IoError::InvalidScene { .. })), "{e:?}");
    // A coordinate a float cannot hold, written as double.
    let huge = IndexedMesh::new(
        vec![Point3::new(1e300, 0.0, 0.0), Point3::X, Point3::Y],
        vec![[0, 1, 2]],
    )
    .unwrap();
    let s = scene_of(huge);
    assert!(matches!(
        export(
            &s,
            Sink::Memory(&mut Vec::new()),
            Format::Ply,
            &opts(true, false),
            &ctx()
        ),
        Err(IoError::InvalidScene { .. })
    ));
    assert!(
        export(
            &s,
            Sink::Memory(&mut Vec::new()),
            Format::Ply,
            &opts(true, true),
            &ctx()
        )
        .is_ok()
    );
    // An open mesh warns by default (STL/OBJ/PLY use `Warn`) and fails under `Require`.
    let open = IndexedMesh::new(vec![Point3::ZERO, Point3::X, Point3::Y], vec![[0, 1, 2]]).unwrap();
    let s = scene_of(open);
    let mut out = Vec::new();
    let r = export(
        &s,
        Sink::Memory(&mut out),
        Format::Ply,
        &ExportOptions::default(),
        &ctx(),
    )
    .expect("warn");
    assert!(
        r.warnings
            .iter()
            .any(|w| matches!(w, IoWarning::NotManifold { .. }))
    );
    let strict = ExportOptions {
        validity: Some(ValidityPolicy::Require),
        ..ExportOptions::default()
    };
    assert!(matches!(
        export(
            &s,
            Sink::Memory(&mut Vec::new()),
            Format::Ply,
            &strict,
            &ctx()
        ),
        Err(IoError::InvalidForFormat {
            format: Format::Ply,
            ..
        })
    ));
    // Face materials are not written, and that is said.
    let mut s = scene_of(
        IndexedMesh::new(vec![Point3::ZERO, Point3::X, Point3::Y], vec![[0, 1, 2]]).unwrap(),
    );
    s.materials.push(MaterialSlot {
        name: "Red".into(),
        color: Rgba8([255, 0, 0, 255]),
        filament: None,
    });
    s.objects[0].material = Some(0);
    let (_, r) = write(&s, &opts(true, false));
    assert!(
        r.warnings
            .iter()
            .any(|w| matches!(w, IoWarning::UnsupportedContentIgnored { .. }))
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// `double` binary and ASCII round-trip arbitrary finite coordinates bit-exactly (`-0`
    /// reads back as `0`), and the indices exactly.
    #[test]
    fn double_round_trip_is_exact(
        coords in prop::collection::vec(prop::collection::vec(-1e12f64..1e12f64, 3), 3..40),
        tiny in prop::collection::vec(-1e-300f64..1e-300f64, 3),
        faces in prop::collection::vec((0u32..40, 0u32..40, 0u32..40), 1..60),
    ) {
        let mut positions: Vec<Point3> = coords.iter().map(|c| Point3::new(c[0], c[1], c[2])).collect();
        positions.push(Point3::new(tiny[0], tiny[1], tiny[2]));
        let n = positions.len() as u32;
        let tris: Vec<[u32; 3]> = faces.iter().map(|&(a, b, c)| [a % n, b % n, c % n]).collect();
        let m = IndexedMesh::new(positions, tris).unwrap();
        let s = scene_of(m.clone());
        for binary in [true, false] {
            let (bytes, _) = write(&s, &opts(binary, true));
            let i = run(&bytes).unwrap();
            let b = &i.scene.objects[0].mesh;
            prop_assert_eq!(&b.tris, &m.tris);
            for (p, q) in b.positions.iter().zip(&m.positions) {
                let norm = |v: f64| if v == 0.0 { 0.0 } else { v };
                prop_assert_eq!(p.to_array().map(|v| norm(v).to_bits()), q.to_array().map(|v| norm(v).to_bits()));
            }
        }
    }
}
