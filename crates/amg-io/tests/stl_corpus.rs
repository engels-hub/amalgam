//! IO-02 Done-when 1, 4 and 7 on real data: the MESH-28 corpus parses with the manifest's
//! triangle counts and agrees with `stl_io`; binary write→read is bit-exact.
//!
//! Un-fetched git-LFS pointers are skipped unless `AMG_REQUIRE_TESTDATA=1`.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test code; a failed expectation is a test failure"
)]

use std::sync::Arc;

use amg_core::Transform;
use amg_io::{
    ExportOptions, Format, ImportOptions, IoError, IoWarning, Scene, SceneNode, SceneObject, Sink,
    Source, StlWeld, ValidityPolicy, export, import,
};
use amg_jobs::JobCtx;
use amg_mesh::IndexedMesh;
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root};

fn require() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn read_file(path: &std::path::Path, opts: &ImportOptions) -> Result<amg_io::Imported, IoError> {
    import(Source::Path(path), Some(Format::Stl), opts, &ctx())
}

fn f32_triangle(m: &IndexedMesh, f: usize) -> [[f32; 3]; 3] {
    m.tris[f].map(|i| {
        let p = m.positions[i as usize];
        [p.x as f32, p.y as f32, p.z as f32]
    })
}

/// Done-when 1 and 7.
#[test]
fn corpus_stl_entries_parse_with_manifest_counts_and_agree_with_stl_io() {
    let manifest = Manifest::load().expect("mesh manifest");
    let root = corpus_root();
    let (mut checked, mut skipped, mut oracle_failed, mut dropped_files) = (0, 0, 0, 0);
    for e in manifest.meshes.iter().filter(|e| e.format == "stl") {
        let path = match e.locate(&root) {
            FileStatus::Ready(p) => p,
            FileStatus::LfsPointer(p) => {
                assert!(
                    !require(),
                    "{}: un-fetched LFS pointer {}",
                    e.id,
                    p.display()
                );
                skipped += 1;
                continue;
            }
            FileStatus::NotFetched(_) => {
                skipped += 1;
                continue;
            }
            FileStatus::Missing(p) => panic!("{}: {} is missing", e.id, p.display()),
        };
        let got = read_file(&path, &ImportOptions::default())
            .unwrap_or_else(|err| panic!("{}: {err}", e.id));
        let mesh = &got.scene.objects[0].mesh;
        let dropped: u64 = got
            .report
            .warnings
            .iter()
            .map(|w| match w {
                IoWarning::NonFiniteDropped(n) => *n,
                _ => 0,
            })
            .sum();
        if dropped == 0 {
            assert_eq!(mesh.face_count(), e.faces, "{}: triangle count", e.id);
        } else {
            dropped_files += 1;
            assert_eq!(
                mesh.face_count() as u64 + dropped,
                e.faces as u64,
                "{}: triangle count with {dropped} dropped",
                e.id
            );
        }
        mesh.validate().expect("valid mesh");

        // The oracle.
        let bytes = std::fs::read(&path).expect("read");
        let theirs = match stl_io::read_stl(&mut std::io::Cursor::new(&bytes)) {
            Ok(m) => m,
            Err(_) => {
                oracle_failed += 1;
                continue;
            }
        };
        if dropped > 0 {
            continue; // stl_io keeps the NaN triangles; the counts differ by design.
        }
        assert_eq!(
            theirs.faces.len(),
            mesh.face_count(),
            "{}: stl_io count",
            e.id
        );
        for f in 0..mesh.face_count() {
            let ours = f32_triangle(mesh, f);
            let t = theirs.faces[f].vertices.map(|i| theirs.vertices[i]);
            let theirs_pos = t.map(|v| [v[0], v[1], v[2]]);
            // `==` also treats -0 and +0 alike, which is what the weld does.
            assert!(
                ours == theirs_pos,
                "{}: face {f}: {ours:?} vs {theirs_pos:?}",
                e.id
            );
        }
        checked += 1;
    }
    eprintln!(
        "stl corpus: {checked} compared with stl_io, {skipped} skipped, {oracle_failed} stl_io \
         failures, {dropped_files} files with dropped non-finite triangles"
    );
    assert!(
        checked + skipped + oracle_failed + dropped_files >= 100,
        "the manifest lists the whole corpus"
    );
    // The oracle should read nearly everything we read.
    assert!(
        oracle_failed * 10 <= checked + oracle_failed + 10,
        "stl_io failed too often"
    );
}

fn sphere_scene(k: u32) -> Scene {
    // f32-representable input: round the generated positions to f32 first.
    let mut m = amg_mesh::corpus::bench::geodesic_sphere(k, 10.0, None);
    for p in &mut m.positions {
        p.x = f64::from(p.x as f32);
        p.y = f64::from(p.y as f32);
        p.z = f64::from(p.z as f32);
    }
    scene_of(m)
}

fn scene_of(m: IndexedMesh) -> Scene {
    let mut s = Scene::default();
    s.objects.push(SceneObject::new(Arc::new(m)));
    s.roots.push(SceneNode::mesh(0, Transform::IDENTITY));
    s
}

fn quiet() -> ExportOptions {
    ExportOptions {
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    }
}

/// Done-when 4.
#[test]
fn binary_write_then_read_is_bit_exact() {
    let scene = sphere_scene(40);
    let mesh = &scene.objects[0].mesh;
    let mut bytes = Vec::new();
    let rep = export(
        &scene,
        Sink::Memory(&mut bytes),
        Format::Stl,
        &quiet(),
        &ctx(),
    )
    .expect("write");
    assert_eq!(rep.triangles, mesh.face_count() as u64);
    assert_eq!(bytes.len(), 84 + 50 * mesh.face_count());
    assert!(!bytes.starts_with(b"solid"));
    assert!(rep.warnings.is_empty(), "{:?}", rep.warnings);

    let back = import(
        Source::Bytes(&bytes),
        None,
        &ImportOptions::default(),
        &ctx(),
    )
    .expect("read");
    let m2 = &back.scene.objects[0].mesh;
    assert_eq!(m2.face_count(), mesh.face_count());
    for f in 0..mesh.face_count() {
        // Same triangle, same winding, same bits (the vertex numbering may differ).
        let (a, b) = (
            mesh.triangle(amg_mesh::FaceId(f as u32)),
            m2.triangle(amg_mesh::FaceId(f as u32)),
        );
        for (p, q) in a.iter().zip(&b) {
            assert_eq!(
                [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()],
                [q.x.to_bits(), q.y.to_bits(), q.z.to_bits()],
                "face {f}"
            );
        }
    }
    // The sphere is welded already, so the exact weld finds the same vertex count.
    assert_eq!(m2.vertex_count(), mesh.vertex_count());
    // Written twice, identical bytes.
    let mut again = Vec::new();
    export(
        &back.scene,
        Sink::Memory(&mut again),
        Format::Stl,
        &quiet(),
        &ctx(),
    )
    .expect("write");
    assert_eq!(again.len(), bytes.len());
    assert!(again == bytes, "the second generation differs");
}

#[test]
fn ascii_write_then_read_is_exact_for_f32_input() {
    let scene = sphere_scene(12);
    let mut o = quiet();
    o.stl.ascii = true;
    let mut bytes = Vec::new();
    export(&scene, Sink::Memory(&mut bytes), Format::Stl, &o, &ctx()).expect("write");
    assert!(bytes.starts_with(b"solid "));
    assert!(bytes.ends_with(b"endsolid Amalgam\n"));
    let back = import(
        Source::Bytes(&bytes),
        None,
        &ImportOptions::default(),
        &ctx(),
    )
    .expect("read");
    let (m1, m2) = (&scene.objects[0].mesh, &back.scene.objects[0].mesh);
    assert_eq!(m1.face_count(), m2.face_count());
    for f in 0..m1.face_count() {
        let id = amg_mesh::FaceId(f as u32);
        assert!(m1.triangle(id) == m2.triangle(id), "face {f}");
    }
}

#[test]
fn path_sink_is_atomic_and_matches_memory() {
    let scene = sphere_scene(8);
    let dir = tempfile::tempdir().expect("tempdir");
    let p = dir.path().join("s.stl");
    let rep = export(&scene, Sink::Path(&p), Format::Stl, &quiet(), &ctx()).expect("write");
    let mut mem = Vec::new();
    export(
        &scene,
        Sink::Memory(&mut mem),
        Format::Stl,
        &quiet(),
        &ctx(),
    )
    .expect("write");
    assert_eq!(std::fs::read(&p).expect("read"), mem);
    assert_eq!(rep.files, vec![p.clone()]);
    assert_eq!(rep.bytes, mem.len() as u64);
    assert!(!dir.path().join("s.stl.tmp").exists());
    // Reading from the path agrees with reading the bytes.
    let a = read_file(&p, &ImportOptions::default()).expect("read");
    assert_eq!(a.report.bytes, mem.len() as u64);
    assert_eq!(a.report.triangles_read, scene.triangle_count());
}

#[test]
fn merge_flattens_in_world_space_and_split_writes_one_file_per_object() {
    let cube = amg_mesh::corpus::bench::geodesic_sphere(2, 1.0, None);
    let mut s = scene_of(cube);
    s.roots.push(SceneNode::mesh(
        0,
        Transform::from_translation(amg_core::Vec3::new(10.0, 0.0, 0.0)),
    ));
    let per_object = 80u64;
    // Merged: one solid of both.
    let mut mem = Vec::new();
    let r = export(&s, Sink::Memory(&mut mem), Format::Stl, &quiet(), &ctx()).expect("write");
    assert_eq!(r.triangles, 2 * per_object);
    let back = import(Source::Bytes(&mem), None, &ImportOptions::default(), &ctx()).expect("read");
    let bb = back.scene.world_aabb();
    assert!(
        (bb.max.x - 11.0).abs() < 1e-5 && (bb.min.x + 1.0).abs() < 1e-5,
        "{bb:?}"
    );

    // merge = false: two numbered files.
    let dir = tempfile::tempdir().expect("tempdir");
    let p = dir.path().join("pair.stl");
    let mut o = quiet();
    o.stl.merge = false;
    let r = export(&s, Sink::Path(&p), Format::Stl, &o, &ctx()).expect("write");
    assert_eq!(
        r.files,
        vec![dir.path().join("pair_1.stl"), dir.path().join("pair_2.stl")]
    );
    assert!(!p.exists());
    let second = read_file(&r.files[1], &ImportOptions::default()).expect("read");
    assert!((second.scene.world_aabb().min.x - 9.0).abs() < 1e-5);
    // Several files to a buffer is not possible.
    let e = export(&s, Sink::Memory(&mut Vec::new()), Format::Stl, &o, &ctx());
    assert!(matches!(e, Err(IoError::Unsupported { .. })));
}

#[test]
fn face_colours_round_trip_through_the_attribute_word() {
    let mut s = sphere_scene(3);
    let n = s.objects[0].mesh.face_count();
    s.materials = vec![
        amg_io::MaterialSlot {
            name: "red".into(),
            color: amg_mesh::Rgba8([248, 0, 0, 255]),
            filament: None,
        },
        amg_io::MaterialSlot {
            name: "teal".into(),
            color: amg_mesh::Rgba8([0, 128, 128, 255]),
            filament: None,
        },
    ];
    let slots: Vec<u16> = (0..n)
        .map(|f| match f % 3 {
            0 => 0,
            1 => 1,
            _ => amg_io::NO_SLOT,
        })
        .collect();
    s.objects[0].face_materials = Some(slots.clone().into());
    let mut o = quiet();
    o.stl.face_colors = true;
    let mut bytes = Vec::new();
    export(&s, Sink::Memory(&mut bytes), Format::Stl, &o, &ctx()).expect("write");
    let mut io = ImportOptions::default();
    io.stl.face_colors = true;
    let back = import(Source::Bytes(&bytes), None, &io, &ctx()).expect("read");
    let fm = back.scene.objects[0]
        .face_materials
        .as_ref()
        .expect("slots");
    assert_eq!(back.scene.materials.len(), 2);
    for (f, (&a, &b)) in slots.iter().zip(fm.iter()).enumerate() {
        assert_eq!(a, b, "face {f}");
    }
    assert_eq!(back.scene.materials[0].color.0, [255, 0, 0, 255]); // 248 -> 5 bits -> 8 bits
    assert_eq!(back.scene.materials[1].color.0, [0, 132, 132, 255]); // 128 -> 5 bits -> 8 bits
    // Without the option the attribute words are zero.
    let mut plain = Vec::new();
    export(&s, Sink::Memory(&mut plain), Format::Stl, &quiet(), &ctx()).expect("write");
    assert!(plain[84 + 48..84 + 50] == [0, 0]);
}

#[test]
fn export_unit_scales_and_f32_loss_is_reported() {
    let mut m = amg_mesh::corpus::bench::geodesic_sphere(2, 1.0, None);
    m.transform(&Transform::from_translation(amg_core::Vec3::new(
        1000.0, 0.1, 0.0,
    )));
    let s = scene_of(m);
    let mut o = quiet();
    o.unit = amg_core::units::LengthUnit::Metre;
    let mut bytes = Vec::new();
    let rep = export(&s, Sink::Memory(&mut bytes), Format::Stl, &o, &ctx()).expect("write");
    // Numbers are in metres: the sphere sits near x = 1.0.
    let io = ImportOptions {
        unit: amg_io::UnitChoice::Assume(amg_core::units::LengthUnit::Metre),
        ..ImportOptions::default()
    };
    let back = import(Source::Bytes(&bytes), None, &io, &ctx()).expect("read");
    let bb = back.scene.world_aabb();
    assert!(
        (bb.min.x - 999.0).abs() < 1e-3 && (bb.max.x - 1001.0).abs() < 1e-3,
        "{bb:?}"
    );
    // 0.1 mm at x = 1000 mm cannot be kept in f32 to 1e-5 mm: the writer says so.
    assert!(
        rep.warnings
            .iter()
            .any(|w| matches!(w, IoWarning::PrecisionLoss { .. })),
        "{:?}",
        rep.warnings
    );
}

#[test]
fn validity_policy_gates_the_export() {
    // An open surface: one triangle.
    let m = IndexedMesh::new(
        vec![
            amg_core::Point3::new(0.0, 0.0, 0.0),
            amg_core::Point3::new(1.0, 0.0, 0.0),
            amg_core::Point3::new(0.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
    )
    .expect("mesh");
    let s = scene_of(m);
    // The STL default is Warn: written, with a warning.
    let rep = export(
        &s,
        Sink::Memory(&mut Vec::new()),
        Format::Stl,
        &ExportOptions::default(),
        &ctx(),
    )
    .expect("write");
    assert!(
        rep.warnings
            .iter()
            .any(|w| matches!(w, IoWarning::NotManifold { .. }))
    );
    // Require refuses.
    let strict = ExportOptions {
        validity: Some(ValidityPolicy::Require),
        ..ExportOptions::default()
    };
    let mut out = Vec::new();
    let e = export(&s, Sink::Memory(&mut out), Format::Stl, &strict, &ctx());
    assert!(matches!(e, Err(IoError::InvalidForFormat { count: 1, .. })));
    assert!(out.is_empty(), "nothing is written on refusal");
}

#[test]
fn empty_scene_is_an_error() {
    let e = export(
        &Scene::default(),
        Sink::Memory(&mut Vec::new()),
        Format::Stl,
        &quiet(),
        &ctx(),
    );
    assert!(matches!(e, Err(IoError::Empty)));
}

#[test]
fn tolerance_weld_merges_a_soup_written_by_the_exporter() {
    let scene = sphere_scene(6);
    let mut bytes = Vec::new();
    export(
        &scene,
        Sink::Memory(&mut bytes),
        Format::Stl,
        &quiet(),
        &ctx(),
    )
    .expect("write");
    let mut o = ImportOptions::default();
    o.stl.weld = StlWeld::None;
    let soup = import(Source::Bytes(&bytes), None, &o, &ctx()).expect("read");
    let n = soup.scene.objects[0].mesh.face_count();
    assert_eq!(soup.scene.objects[0].mesh.vertex_count(), 3 * n);
    o.stl.weld = StlWeld::Tolerance(1e-6);
    let merged = import(Source::Bytes(&bytes), None, &o, &ctx()).expect("read");
    assert_eq!(
        merged.scene.objects[0].mesh.vertex_count(),
        scene.objects[0].mesh.vertex_count()
    );
}
