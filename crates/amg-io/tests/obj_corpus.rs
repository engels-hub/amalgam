//! IO-05 Done-when 1 and 5 on real data.
//!
//! The MESH-28 manifest holds no OBJ entries (STL and PLY only), so besides iterating any
//! `format = "obj"` entry it may gain, this test derives an OBJ file from every STL entry
//! (our writer, in memory), reads it back, and checks the face count against the manifest and
//! the triangles against `tobj`. Convex polygons are checked against `tobj` too.
//!
//! Un-fetched git-LFS pointers are skipped unless `AMG_REQUIRE_TESTDATA=1`.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test code; a failed expectation is a test failure"
)]

use std::io::Cursor;

use amg_io::{
    CoordPrecision, ExportOptions, Format, ImportOptions, Imported, IoWarning, Scene, Sink, Source,
    ValidityPolicy, export, import,
};
use amg_jobs::JobCtx;
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root};

fn require() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

/// Triangles of `tobj`'s result as f32 corner triples, in file order.
fn tobj_tris(bytes: &[u8]) -> (Vec<[[f32; 3]; 3]>, Vec<[f32; 3]>) {
    let opts = tobj::LoadOptions {
        triangulate: true,
        single_index: false,
        ..tobj::LoadOptions::default()
    };
    let (models, _) = tobj::load_obj_buf(&mut Cursor::new(bytes), &opts, |_| {
        Ok((Vec::new(), Default::default()))
    })
    .expect("tobj");
    let mut tris = Vec::new();
    let mut verts = Vec::new();
    for m in &models {
        let p = &m.mesh.positions;
        let v = |i: u32| {
            let i = i as usize;
            [p[3 * i], p[3 * i + 1], p[3 * i + 2]]
        };
        if verts.is_empty() {
            verts = p.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
        }
        for t in m.mesh.indices.chunks_exact(3) {
            tris.push([v(t[0]), v(t[1]), v(t[2])]);
        }
    }
    (tris, verts)
}

fn our_tris(s: &Scene) -> Vec<[[f32; 3]; 3]> {
    let mut out = Vec::new();
    for o in &s.objects {
        let m = &o.mesh;
        let c = |i: u32| {
            let p = m.positions[i as usize];
            [p.x as f32, p.y as f32, p.z as f32]
        };
        for t in &m.tris {
            out.push([c(t[0]), c(t[1]), c(t[2])]);
        }
    }
    out
}

fn read_obj(bytes: &[u8]) -> Imported {
    import(
        Source::Bytes(bytes),
        Some(Format::Obj),
        &ImportOptions::default(),
        &ctx(),
    )
    .expect("obj import")
}

#[test]
fn corpus_entries_parse_with_manifest_counts_and_agree_with_tobj() {
    let manifest = Manifest::load().expect("mesh manifest");
    let root = corpus_root();
    let (mut checked, mut skipped, mut obj_entries) = (0, 0, 0);
    let write_opts = ExportOptions {
        precision: CoordPrecision::Roundtrip,
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    };
    for e in manifest
        .meshes
        .iter()
        .filter(|e| e.format == "stl" || e.format == "obj")
    {
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
        // A real OBJ entry is read as is; an STL entry goes through our writer first.
        let (bytes, expect_faces) = if e.format == "obj" {
            obj_entries += 1;
            (std::fs::read(&path).expect("read"), e.faces)
        } else {
            let stl = import(
                Source::Path(&path),
                Some(Format::Stl),
                &ImportOptions::default(),
                &ctx(),
            )
            .unwrap_or_else(|err| panic!("{}: {err}", e.id));
            let faces = stl.scene.triangle_count() as usize;
            // NaN triangles are dropped by the STL reader; otherwise the manifest count holds.
            let dropped = stl
                .report
                .warnings
                .iter()
                .any(|w| matches!(w, IoWarning::NonFiniteDropped(_)));
            assert!(dropped || faces == e.faces, "{}: STL face count", e.id);
            let mut out = Vec::new();
            export(
                &stl.scene,
                Sink::Memory(&mut out),
                Format::Obj,
                &write_opts,
                &ctx(),
            )
            .unwrap_or_else(|err| panic!("{}: {err}", e.id));
            (out, faces)
        };
        let got = read_obj(&bytes);
        assert_eq!(
            got.scene.triangle_count() as usize,
            expect_faces,
            "{}: face count",
            e.id
        );
        for o in &got.scene.objects {
            o.mesh.validate().expect("valid mesh");
        }
        // The oracle: tobj on the same bytes (convex = triangles here).
        let (theirs, their_verts) = tobj_tris(&bytes);
        let ours = our_tris(&got.scene);
        assert_eq!(theirs.len(), ours.len(), "{}: tobj count", e.id);
        assert!(
            theirs == ours,
            "{}: triangle positions differ from tobj",
            e.id
        );
        if got.scene.objects.len() == 1 {
            let m = &got.scene.objects[0].mesh;
            assert_eq!(
                their_verts.len(),
                m.vertex_count(),
                "{}: tobj vertices",
                e.id
            );
        }
        checked += 1;
    }
    eprintln!("obj corpus: {checked} checked ({obj_entries} OBJ entries), {skipped} skipped");
    assert!(
        checked > 0 || skipped > 0,
        "the manifest has no STL or OBJ entries"
    );
}

/// Convex polygons: `tobj` fans them and we ear-clip, so triangles differ, but the vertex
/// array, the triangle count and the total area agree.
#[test]
fn convex_polygons_agree_with_tobj_on_count_positions_and_area() {
    // A unit cube with quads, a pentagonal prism's caps, and a hexagon.
    let mut t = String::new();
    for (x, y, z) in [
        (0, 0, 0),
        (1, 0, 0),
        (1, 1, 0),
        (0, 1, 0),
        (0, 0, 1),
        (1, 0, 1),
        (1, 1, 1),
        (0, 1, 1),
    ] {
        t.push_str(&format!("v {x} {y} {z}\n"));
    }
    t.push_str("f 4 3 2 1\nf 5 6 7 8\nf 1 2 6 5\nf 2 3 7 6\nf 3 4 8 7\nf 4 1 5 8\n");
    for k in 0..6 {
        let a = f64::from(k) * std::f64::consts::TAU / 6.0;
        t.push_str(&format!("v {} {} 3\n", a.cos() * 2.0, a.sin() * 2.0));
    }
    t.push_str("f 9 10 11 12 13 14\n");
    let (theirs, their_verts) = tobj_tris(t.as_bytes());
    let got = read_obj(t.as_bytes());
    let m = &got.scene.objects[0].mesh;
    assert_eq!(m.face_count(), theirs.len());
    assert_eq!(m.face_count(), 6 * 2 + 4);
    assert_eq!(m.vertex_count(), their_verts.len());
    // `tobj` numbers vertices by first use, so compare the sets of positions.
    let key = |v: [f32; 3]| v.map(f32::to_bits);
    let mut ours: Vec<[u32; 3]> = m
        .positions
        .iter()
        .map(|p| key([p.x as f32, p.y as f32, p.z as f32]))
        .collect();
    let mut theirs_v: Vec<[u32; 3]> = their_verts.iter().map(|&v| key(v)).collect();
    ours.sort_unstable();
    theirs_v.sort_unstable();
    assert_eq!(ours, theirs_v);
    let area_of = |tris: &[[[f32; 3]; 3]]| -> f64 {
        tris.iter()
            .map(|t| {
                let a = |i: usize, j: usize| f64::from(t[i][j]);
                let (u, v) = (
                    [a(1, 0) - a(0, 0), a(1, 1) - a(0, 1), a(1, 2) - a(0, 2)],
                    [a(2, 0) - a(0, 0), a(2, 1) - a(0, 1), a(2, 2) - a(0, 2)],
                );
                let c = [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ];
                0.5 * (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt()
            })
            .sum()
    };
    let (a_theirs, a_ours) = (area_of(&theirs), area_of(&our_tris(&got.scene)));
    assert!(
        (a_theirs - a_ours).abs() < 1e-4 * a_theirs,
        "{a_theirs} vs {a_ours}"
    );
    assert_eq!(
        got.report.warnings,
        vec![IoWarning::PolygonTriangulated {
            polygons: 7,
            fallback_fan: 0
        }]
    );
}
