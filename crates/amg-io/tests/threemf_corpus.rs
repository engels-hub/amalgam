//! IO-03 Done-when 3: every 3MF entry of the MESH-28 corpus loads with the manifest's face
//! count (summed over placed instances' objects).
//!
//! The manifest has no 3MF entry today (173 STL, 3 PLY); the test iterates whatever
//! `format = "3mf"` entries exist, so new entries are covered without a code change. Un-fetched
//! LFS pointers are skipped unless `AMG_REQUIRE_TESTDATA=1`.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::print_stderr,
    reason = "test code; a failed expectation is a test failure"
)]

use amg_io::{Format, ImportOptions, Source, import};
use amg_jobs::JobCtx;
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root};

#[test]
fn corpus_3mf_entries_load_with_manifest_counts() {
    let manifest = Manifest::load().expect("mesh manifest");
    let root = corpus_root();
    let (mut checked, mut skipped) = (0, 0);
    for e in manifest.meshes.iter().filter(|e| e.format == "3mf") {
        let path = match e.locate(&root) {
            FileStatus::Ready(p) => p,
            FileStatus::LfsPointer(p) => {
                assert!(
                    std::env::var("AMG_REQUIRE_TESTDATA").map_or(true, |v| v != "1"),
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
        let r = import(
            Source::Path(&path),
            Some(Format::ThreeMf),
            &ImportOptions::default(),
            &JobCtx::detached(),
        )
        .unwrap_or_else(|err| panic!("{}: {err}", e.id));
        let faces: usize = r.scene.objects.iter().map(|o| o.mesh.face_count()).sum();
        assert_eq!(faces, e.faces, "{}: face count", e.id);
        checked += 1;
    }
    eprintln!("MESH-28 3MF entries: {checked} checked, {skipped} skipped");
}
