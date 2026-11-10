//! IO-05 Done-when 4: OBJ write→read round-trips positions (`Roundtrip` bit-exact), indices,
//! objects and slots; plus the writer's own behaviour (determinism, options, units).

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test code; a failed expectation is a test failure"
)]

use std::sync::Arc;

use amg_core::{Point2, Point3, Transform, Vec3, units::LengthUnit};
use amg_io::{
    CoordPrecision, ExportOptions, Format, ImportOptions, IoError, IoWarning, MaterialSlot,
    NO_SLOT, Scene, SceneNode, SceneObject, Sink, Source, UnitChoice, ValidityPolicy, export,
    import,
};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, Rgba8};
use proptest::prelude::*;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn scratch_dir() -> tempfile::TempDir {
    let base = std::env::var_os("CARGO_TARGET_DIR")
        .map_or_else(std::env::temp_dir, std::path::PathBuf::from);
    std::fs::create_dir_all(&base).expect("base");
    tempfile::Builder::new()
        .prefix("obj-roundtrip")
        .tempdir_in(base)
        .expect("tempdir")
}

fn raw() -> ExportOptions {
    ExportOptions {
        precision: CoordPrecision::Roundtrip,
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    }
}

/// A tetrahedron-like mesh with awkward f64 coordinates, offset by `k`.
fn tetra(k: f64) -> IndexedMesh {
    let third = 1.0 / 3.0;
    IndexedMesh::new(
        vec![
            Point3::new(k + 0.1 + 0.2, third, -1e-7),
            Point3::new(k + 1e5 + third, 2.0f64.sqrt(), 3.5e-12),
            Point3::new(k - 7.0, -third, 1_234.567_891_234_567_8),
            Point3::new(k, 1e20, -0.0),
        ],
        vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]],
    )
    .expect("mesh")
}

fn scene_of(meshes: Vec<(Option<&str>, IndexedMesh)>) -> Scene {
    let mut s = Scene::default();
    for (i, (name, m)) in meshes.into_iter().enumerate() {
        let mut o = SceneObject::new(Arc::new(m));
        o.name = name.map(str::to_owned);
        s.objects.push(o);
        s.roots.push(SceneNode::mesh(i as u32, Transform::IDENTITY));
    }
    s
}

fn bits(p: Point3) -> [u64; 3] {
    // -0 reads back as +0 by design.
    [p.x, p.y, p.z].map(|v| if v == 0.0 { 0 } else { v.to_bits() })
}

fn write_mem(s: &Scene, opts: &ExportOptions) -> Vec<u8> {
    let mut out = Vec::new();
    export(s, Sink::Memory(&mut out), Format::Obj, opts, &ctx()).expect("export");
    out
}

fn read_mem(b: &[u8]) -> amg_io::Imported {
    import(
        Source::Bytes(b),
        Some(Format::Obj),
        &ImportOptions::default(),
        &ctx(),
    )
    .expect("import")
}

#[test]
fn objects_names_indices_and_positions_are_bit_exact() {
    let s = scene_of(vec![
        (Some("alpha"), tetra(0.0)),
        (Some("beta gamma"), tetra(10.0)),
    ]);
    let back = read_mem(&write_mem(&s, &raw())).scene;
    assert_eq!(back.objects.len(), 2);
    for (a, b) in s.objects.iter().zip(&back.objects) {
        assert_eq!(a.name, b.name);
        assert_eq!(a.mesh.tris, b.mesh.tris);
        let pa: Vec<_> = a.mesh.positions.iter().map(|&p| bits(p)).collect();
        let pb: Vec<_> = b.mesh.positions.iter().map(|&p| bits(p)).collect();
        assert_eq!(pa, pb);
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// Any finite f64 survives `Roundtrip`.
    #[test]
    fn arbitrary_doubles_roundtrip(
        coords in prop::collection::vec(prop::num::f64::NORMAL | prop::num::f64::SUBNORMAL | prop::num::f64::ZERO, 9..60)
    ) {
        let n = coords.len() / 3;
        let positions: Vec<Point3> = (0..n)
            .map(|i| Point3::new(coords[3 * i], coords[3 * i + 1], coords[3 * i + 2]))
            .collect();
        let tris: Vec<[u32; 3]> = (0..n.saturating_sub(2))
            .map(|i| [i as u32, i as u32 + 1, i as u32 + 2])
            .collect();
        prop_assume!(!tris.is_empty());
        // Exact export unit mm; scale_from_mm(mm) is exact.
        let m = IndexedMesh::new(positions, tris).expect("mesh");
        let s = scene_of(vec![(None, m)]);
        let back = read_mem(&write_mem(&s, &raw())).scene;
        let a: Vec<_> = s.objects[0].mesh.positions.iter().map(|&p| bits(p)).collect();
        let b: Vec<_> = back.objects[0].mesh.positions.iter().map(|&p| bits(p)).collect();
        prop_assert_eq!(a, b);
        prop_assert_eq!(&s.objects[0].mesh.tris, &back.objects[0].mesh.tris);
    }
}

#[test]
fn slots_colours_and_uvs_roundtrip_through_a_file_pair() {
    let mut m0 = tetra(0.0);
    m0.attrs.vertex_colors = Some(vec![
        Rgba8([255, 0, 0, 255]),
        Rgba8([0, 255, 0, 255]),
        Rgba8([1, 2, 3, 255]),
        Rgba8([200, 100, 50, 255]),
    ]);
    let uv = |a: f64, b: f64| Point2::new(a, b);
    m0.attrs.corner_uvs = Some(vec![
        [uv(0., 0.), uv(1., 0.), uv(0., 1.)],
        [uv(0.1, 0.2), uv(0.3, 0.4), uv(0.5, 0.6)],
        [uv(0., 0.), uv(1., 0.), uv(0., 1.)],
        [uv(1. / 3., 2. / 3.), uv(0., 0.), uv(1., 1.)],
    ]);
    let mut s = scene_of(vec![(Some("a"), m0), (Some("b"), tetra(5.0))]);
    s.materials = vec![
        MaterialSlot {
            name: "PLA Red".into(),
            color: Rgba8([200, 30, 30, 255]),
            filament: None,
        },
        MaterialSlot {
            name: "Clear".into(),
            color: Rgba8([10, 20, 30, 128]),
            filament: None,
        },
    ];
    s.objects[0].face_materials = Some(Arc::from(vec![0u16, 0, 1, NO_SLOT]));
    s.objects[1].material = Some(1);

    let dir = scratch_dir();
    let path = dir.path().join("scene.obj");
    let r = export(&s, Sink::Path(&path), Format::Obj, &raw(), &ctx()).expect("export");
    assert_eq!(r.files, vec![path.clone(), path.with_extension("mtl")]);
    assert!(path.with_extension("mtl").exists());
    let text = std::fs::read_to_string(&path).expect("read");
    assert!(text.contains("mtllib scene.mtl\n"), "{text}");

    let back =
        import(Source::Path(&path), None, &ImportOptions::default(), &ctx()).expect("import");
    let b = &back.scene;
    assert!(
        back.report.warnings.is_empty(),
        "{:?}",
        back.report.warnings
    );
    assert_eq!(b.materials, s.materials);
    assert_eq!(b.objects.len(), 2);
    let (a0, b0) = (&s.objects[0], &b.objects[0]);
    assert_eq!(b0.face_materials.as_deref(), a0.face_materials.as_deref());
    assert_eq!(b0.mesh.attrs.vertex_colors, a0.mesh.attrs.vertex_colors);
    assert_eq!(b0.mesh.attrs.corner_uvs, a0.mesh.attrs.corner_uvs);
    // The default slot of object b becomes explicit per-face slots.
    assert_eq!(
        b.objects[1].face_materials.as_deref(),
        Some(&[1u16, 1, 1, 1][..])
    );
}

#[test]
fn output_is_byte_identical_and_independent_of_thread_count() {
    let s = scene_of(vec![(Some("a"), tetra(0.0)), (Some("b"), tetra(3.0))]);
    let o = ExportOptions {
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    };
    let one = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .expect("pool")
        .install(|| write_mem(&s, &o));
    let many = rayon::ThreadPoolBuilder::new()
        .num_threads(16)
        .build()
        .expect("pool")
        .install(|| write_mem(&s, &o));
    assert_eq!(one, many);
    assert_eq!(one, write_mem(&s, &o));
}

#[test]
fn default_output_has_six_trimmed_decimals_and_one_based_indices() {
    let m = IndexedMesh::new(
        vec![
            Point3::new(0.0, 0.0, -0.0),
            Point3::new(1.5, 0.000_000_4, 100.0),
            Point3::new(-2.123_456_789, 3.0, 0.1 + 0.2),
        ],
        vec![[0, 1, 2]],
    )
    .expect("mesh");
    let s = scene_of(vec![(None, m)]);
    let o = ExportOptions {
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    };
    let text = String::from_utf8(write_mem(&s, &o)).expect("utf8");
    let body: Vec<&str> = text.lines().filter(|l| !l.starts_with('#')).collect();
    assert_eq!(
        body,
        vec!["v 0 0 0", "v 1.5 0 100", "v -2.123457 3 0.3", "f 1 2 3"]
    );
}

#[test]
fn normals_and_uvs_use_the_right_face_syntax() {
    let mut m = tetra(0.0);
    let o = ExportOptions {
        validity: Some(ValidityPolicy::AllowInvalid),
        precision: CoordPrecision::Roundtrip,
        obj: amg_io::ObjWriteOptions {
            normals: true,
            mtl: true,
        },
        ..ExportOptions::default()
    };
    let s = scene_of(vec![(None, m.clone())]);
    let text = String::from_utf8(write_mem(&s, &o)).expect("utf8");
    assert_eq!(text.lines().filter(|l| l.starts_with("vn ")).count(), 4);
    assert!(text.contains("f 1//1 3//3 2//2"), "{text}");
    // Normals are unit length.
    for l in text.lines().filter(|l| l.starts_with("vn ")) {
        let v: Vec<f64> = l[3..].split(' ').map(|x| x.parse().expect("num")).collect();
        let len = Vec3::new(v[0], v[1], v[2]).length();
        assert!((len - 1.0).abs() < 1e-5, "{l}");
    }
    m.attrs.corner_uvs = Some(vec![[Point2::ZERO; 3]; 4]);
    let s = scene_of(vec![(None, m)]);
    let text = String::from_utf8(write_mem(&s, &o)).expect("utf8");
    // One de-duplicated uv.
    assert_eq!(text.lines().filter(|l| l.starts_with("vt ")).count(), 1);
    assert!(text.contains("f 1/1/1 3/1/3 2/1/2"), "{text}");
    // Reading back drops the normals but keeps everything else.
    let back = read_mem(text.as_bytes());
    assert_eq!(back.scene.objects[0].mesh.tris, s.objects[0].mesh.tris);
}

#[test]
fn placed_instances_are_written_in_world_space() {
    let m = Arc::new(tetra(0.0));
    let mut s = Scene::default();
    let mut o = SceneObject::new(m.clone());
    o.name = Some("part".into());
    s.objects.push(o);
    s.roots.push(SceneNode::mesh(0, Transform::IDENTITY));
    s.roots.push(SceneNode::mesh(
        0,
        Transform::from_translation(Vec3::new(100.0, 0.0, 0.0)),
    ));
    let back = read_mem(&write_mem(&s, &raw())).scene;
    assert_eq!(back.objects.len(), 2);
    let d = back.objects[1].mesh.positions[0].x - back.objects[0].mesh.positions[0].x;
    assert!((d - 100.0).abs() < 1e-9, "{d}");
}

#[test]
fn export_unit_scales_and_import_assume_undoes_it() {
    let s = scene_of(vec![(Some("a"), tetra(1.0))]);
    let o = ExportOptions {
        unit: LengthUnit::Inch,
        ..raw()
    };
    let bytes = write_mem(&s, &o);
    let text = String::from_utf8(bytes.clone()).expect("utf8");
    assert!(text.starts_with("# Amalgam"), "{text}");
    let i = ImportOptions {
        unit: UnitChoice::Assume(LengthUnit::Inch),
        ..ImportOptions::default()
    };
    let back = import(Source::Bytes(&bytes), Some(Format::Obj), &i, &ctx())
        .expect("import")
        .scene;
    for (a, b) in s.objects[0]
        .mesh
        .positions
        .iter()
        .zip(&back.objects[0].mesh.positions)
    {
        assert!((a.x - b.x).abs() <= 1e-12 * a.x.abs().max(1.0));
        assert!((a.z - b.z).abs() <= 1e-9 * a.z.abs().max(1.0));
    }
}

#[test]
fn lossy_decimals_are_reported_and_the_validity_gate_applies() {
    let s = scene_of(vec![(Some("a"), tetra(1.0))]);
    // The default policy for OBJ is Warn: the open tetrahedron-like mesh is reported.
    let mut out = Vec::new();
    let r = export(
        &s,
        Sink::Memory(&mut out),
        Format::Obj,
        &ExportOptions {
            precision: CoordPrecision::Decimals(3),
            ..ExportOptions::default()
        },
        &ctx(),
    )
    .expect("export");
    assert!(
        r.warnings
            .iter()
            .any(|w| matches!(w, IoWarning::PrecisionLoss { .. }))
    );
    // Require refuses an invalid mesh (this one has inconsistent faces / is not a closed solid).
    let open =
        IndexedMesh::new(vec![Point3::ZERO, Point3::X, Point3::Y], vec![[0, 1, 2]]).expect("mesh");
    let s = scene_of(vec![(None, open)]);
    let e = export(
        &s,
        Sink::Memory(&mut Vec::new()),
        Format::Obj,
        &ExportOptions {
            validity: Some(ValidityPolicy::Require),
            ..ExportOptions::default()
        },
        &ctx(),
    );
    assert!(matches!(e, Err(IoError::InvalidForFormat { .. })), "{e:?}");
}

#[test]
fn a_memory_sink_writes_usemtl_but_no_mtl_and_says_so() {
    let mut s = scene_of(vec![(Some("a"), tetra(1.0))]);
    s.materials = vec![MaterialSlot {
        name: "Red".into(),
        color: Rgba8([255, 0, 0, 255]),
        filament: None,
    }];
    s.objects[0].material = Some(0);
    let mut out = Vec::new();
    let r = export(
        &s,
        Sink::Memory(&mut out),
        Format::Obj,
        &ExportOptions {
            validity: Some(ValidityPolicy::AllowInvalid),
            ..ExportOptions::default()
        },
        &ctx(),
    )
    .expect("export");
    let text = String::from_utf8(out).expect("utf8");
    assert!(text.contains("usemtl Red\n") && !text.contains("mtllib"));
    assert!(r.warnings.iter().any(|w| matches!(
        w,
        IoWarning::UnsupportedContentIgnored { what, .. } if what.contains("MTL")
    )));
}

#[test]
fn material_names_are_made_unique_and_safe() {
    let mut s = scene_of(vec![(Some("a"), tetra(1.0))]);
    s.materials = ["Same", "Same", "off", "Line\nbreak"]
        .into_iter()
        .map(|n| MaterialSlot {
            name: n.into(),
            color: Rgba8([1, 2, 3, 255]),
            filament: None,
        })
        .collect();
    s.objects[0].face_materials = Some(Arc::from(vec![0u16, 1, 2, 3]));
    let dir = scratch_dir();
    let path = dir.path().join("n.obj");
    export(&s, Sink::Path(&path), Format::Obj, &raw(), &ctx()).expect("export");
    let back = import(Source::Path(&path), None, &ImportOptions::default(), &ctx())
        .expect("import")
        .scene;
    let names: Vec<_> = back.materials.iter().map(|m| m.name.as_str()).collect();
    assert_eq!(names, ["Same", "Same_2", "Material.3", "Line break"]);
    assert_eq!(
        back.objects[0].face_materials.as_deref(),
        Some(&[0u16, 1, 2, 3][..])
    );
}

#[test]
fn an_empty_scene_is_an_error() {
    let s = scene_of(vec![(None, IndexedMesh::default())]);
    let e = export(
        &s,
        Sink::Memory(&mut Vec::new()),
        Format::Obj,
        &raw(),
        &ctx(),
    );
    assert!(matches!(e, Err(IoError::Empty)), "{e:?}");
}
