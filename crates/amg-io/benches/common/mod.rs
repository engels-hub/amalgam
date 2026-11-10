//! Shared by the STL benches: the 1 GB inputs, generated on first use from the MESH-28
//! benchmark generator (displaced geodesic sphere) into `CARGO_TARGET_DIR` (or
//! `AMG_IO_BENCH_DIR`), never into the repository.
#![allow(
    dead_code,
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "bench support"
)]

use std::path::PathBuf;
use std::sync::Arc;

use amg_core::Transform;
use amg_io::{ExportOptions, Format, Scene, SceneNode, SceneObject, Sink, ValidityPolicy, export};
use amg_jobs::JobCtx;
use amg_mesh::IndexedMesh;
use amg_mesh::corpus::bench::{Displacement, geodesic_sphere};

/// Frequency 1000: 20 M triangles, 84 + 50 * 20 M = 1 000 000 084 bytes.
pub const BIN_FREQ: u32 = 1000;
/// Frequency 450: 4.05 M triangles, about 1 GB of ASCII.
pub const ASCII_FREQ: u32 = 450;

pub fn data_dir() -> PathBuf {
    let base = std::env::var_os("AMG_IO_BENCH_DIR")
        .or_else(|| std::env::var_os("CARGO_TARGET_DIR"))
        .map_or_else(
            || PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target"),
            PathBuf::from,
        );
    let d = base.join("amg-io-bench-data");
    std::fs::create_dir_all(&d).expect("create bench data dir");
    d
}

pub fn mesh(freq: u32) -> IndexedMesh {
    geodesic_sphere(
        freq,
        10.0,
        Some(Displacement {
            amplitude: 0.06,
            seed: 20_260_101,
        }),
    )
}

pub fn scene(mesh: IndexedMesh) -> Scene {
    let mut s = Scene::default();
    s.objects.push(SceneObject::new(Arc::new(mesh)));
    s.roots.push(SceneNode::mesh(0, Transform::IDENTITY));
    s
}

pub fn raw_options(ascii: bool) -> ExportOptions {
    let mut o = ExportOptions {
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    };
    o.stl.ascii = ascii;
    o
}

fn ensure(name: &str, freq: u32, ascii: bool) -> PathBuf {
    let path = data_dir().join(name);
    let marker = data_dir().join(format!("{name}.ok"));
    if !marker.exists() || !path.exists() {
        eprintln!("generating {} (frequency {freq})", path.display());
        let s = scene(mesh(freq));
        export(
            &s,
            Sink::Path(&path),
            Format::Stl,
            &raw_options(ascii),
            &JobCtx::detached(),
        )
        .expect("generate bench input");
        std::fs::write(&marker, b"ok").expect("marker");
    }
    path
}

/// 1 GB binary STL, 20 M triangles.
pub fn bin_1gb() -> PathBuf {
    ensure("stl_bin_1gb.stl", BIN_FREQ, false)
}

/// About 1 GB of ASCII STL.
pub fn ascii_1gb() -> PathBuf {
    ensure("stl_ascii_1gb.stl", ASCII_FREQ, true)
}

/// `AMG_STRICT_TIMING` set: timing bounds are asserted.
pub fn strict() -> bool {
    std::env::var_os("AMG_STRICT_TIMING").is_some()
}
