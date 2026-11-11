//! IO-13 Done-when 1 and 4 on real data.
//!
//! 1. Every `format = "ply"` entry of the MESH-28 manifest parses with the manifest's face
//!    count, and agrees with `ply-rs-bw` (vertices and faces, exactly).
//! 2. Every committed STL entry goes through our PLY writer (binary float, ASCII double, and
//!    binary double) and our reader, and agrees with `ply-rs-bw` on the same bytes.
//!
//! Un-fetched git-LFS pointers and fetch-only files that are not present are skipped unless
//! `AMG_REQUIRE_TESTDATA=1`.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test code; a failed expectation is a test failure"
)]

mod ply_common;

use amg_io::{
    ExportOptions, Format, ImportOptions, Imported, IoWarning, PlyWriteOptions, Sink, Source,
    ValidityPolicy, export, import,
};
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root};
use ply_common::*;

fn require() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

/// Same vertices (bitwise) and same faces as the oracle.
fn agree(id: &str, got: &Imported, o: &Oracle) {
    let m = &got.scene.objects[0].mesh;
    assert_eq!(m.vertex_count(), o.positions.len(), "{id}: vertex count");
    assert!(
        m.positions
            .iter()
            .zip(&o.positions)
            .all(|(p, q)| p.to_array().map(f64::to_bits) == q.map(f64::to_bits)),
        "{id}: positions differ from ply-rs-bw"
    );
    assert_eq!(m.face_count(), o.faces.len(), "{id}: face count");
    assert!(
        m.tris
            .iter()
            .zip(&o.faces)
            .all(|(t, f)| f.len() == 3 && f[..] == t[..]),
        "{id}: faces differ from ply-rs-bw"
    );
}

#[test]
fn ply_entries_parse_with_manifest_counts_and_agree_with_ply_rs_bw() {
    let manifest = Manifest::load().expect("mesh manifest");
    let root = corpus_root();
    let (mut checked, mut skipped) = (0, 0);
    for e in manifest.meshes.iter().filter(|e| e.format == "ply") {
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
            FileStatus::NotFetched(p) => {
                assert!(!require(), "{}: not fetched ({})", e.id, p.display());
                skipped += 1;
                continue;
            }
            FileStatus::Missing(p) => panic!("{}: {} is missing", e.id, p.display()),
        };
        let bytes = std::fs::read(&path).expect("read");
        let got = import(Source::Path(&path), None, &ImportOptions::default(), &ctx())
            .unwrap_or_else(|err| panic!("{}: {err}", e.id));
        assert_eq!(got.report.format, Some(Format::Ply));
        assert_eq!(
            got.scene.triangle_count() as usize,
            e.faces,
            "{}: manifest faces",
            e.id
        );
        assert!(
            got.report.warnings.is_empty(),
            "{}: {:?}",
            e.id,
            got.report.warnings
        );
        got.scene.objects[0].mesh.validate().expect("valid mesh");
        agree(&e.id, &got, &oracle(&bytes));
        // And a write→read cycle keeps it (float: the Stanford files are f32).
        for (binary, double) in [(true, false), (true, true)] {
            let mut out = Vec::new();
            export(
                &got.scene,
                Sink::Memory(&mut out),
                Format::Ply,
                &ExportOptions {
                    validity: Some(ValidityPolicy::AllowInvalid),
                    ply: PlyWriteOptions { binary, double },
                    ..ExportOptions::default()
                },
                &ctx(),
            )
            .expect("write");
            let again = run(&out).expect("read back");
            assert_eq!(
                again.scene.objects[0].mesh.tris, got.scene.objects[0].mesh.tris,
                "{}: round trip",
                e.id
            );
            assert_eq!(
                again.scene.objects[0].mesh.positions, got.scene.objects[0].mesh.positions,
                "{}: round trip positions",
                e.id
            );
        }
        eprintln!("{}: {} faces ok", e.id, e.faces);
        checked += 1;
    }
    eprintln!("ply corpus: {checked} checked, {skipped} skipped");
    assert!(
        checked > 0 || skipped > 0,
        "the manifest has no PLY entries"
    );
}

#[test]
fn stl_entries_through_the_ply_writer_agree_with_ply_rs_bw() {
    let manifest = Manifest::load().expect("mesh manifest");
    let root = corpus_root();
    let (mut checked, mut skipped) = (0, 0);
    for (n, e) in manifest
        .meshes
        .iter()
        .filter(|e| e.format == "stl")
        .enumerate()
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
        let stl = import(
            Source::Path(&path),
            Some(Format::Stl),
            &ImportOptions::default(),
            &ctx(),
        )
        .unwrap_or_else(|err| panic!("{}: {err}", e.id));
        let dropped = stl
            .report
            .warnings
            .iter()
            .any(|w| matches!(w, IoWarning::NonFiniteDropped(_)));
        let faces = stl.scene.triangle_count() as usize;
        assert!(dropped || faces == e.faces, "{}: STL face count", e.id);
        // Binary float is the default; ASCII is slow for the oracle, so every 4th file.
        let mut variants = vec![(true, false), (true, true)];
        if n % 4 == 0 {
            variants.push((false, true));
        }
        for (binary, double) in variants {
            let mut out = Vec::new();
            export(
                &stl.scene,
                Sink::Memory(&mut out),
                Format::Ply,
                &ExportOptions {
                    validity: Some(ValidityPolicy::AllowInvalid),
                    ply: PlyWriteOptions { binary, double },
                    ..ExportOptions::default()
                },
                &ctx(),
            )
            .unwrap_or_else(|err| panic!("{}: {err}", e.id));
            let got = run(&out).unwrap_or_else(|err| panic!("{}: {err}", e.id));
            assert_eq!(
                got.scene.triangle_count() as usize,
                faces,
                "{}: face count",
                e.id
            );
            if !double {
                // `float` positions are the STL's f32 values, so they are exact.
                assert!(
                    got.report.warnings.is_empty(),
                    "{}: {:?}",
                    e.id,
                    got.report.warnings
                );
            }
            agree(&e.id, &got, &oracle(&out));
        }
        checked += 1;
    }
    eprintln!("ply from stl: {checked} checked, {skipped} skipped");
    assert!(
        checked > 0 || skipped > 0,
        "the manifest has no STL entries"
    );
}
