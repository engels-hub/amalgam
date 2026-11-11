//! IO-13 Done-when 5: `io/ply_read_1gb`.
//!
//! The 1 GB binary PLY is generated on first use from the MESH-28 benchmark generator
//! (displaced geodesic sphere, 20 M triangles, 10 M vertices) into `CARGO_TARGET_DIR` (or
//! `AMG_IO_BENCH_DIR`), never into the repository. It is written by this bench rather than by
//! our writer, with the properties a scanner file has: `double` positions and normals, `float`
//! s/t, `uchar` RGBA per vertex, and per face the index list, `uchar` RGBA and a `float` quality
//! (1.02 GB). The < 3 s bound is asserted only under `AMG_STRICT_TIMING=1`.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

mod common;

use std::io::Write;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use amg_io::{Format, ImportOptions, Source, import};
use amg_jobs::JobCtx;
use criterion::{Criterion, criterion_group, criterion_main};

/// Frequency 1000 (the generator's maximum): 20 M triangles, 10 M vertices.
const PLY_FREQ: u32 = 1000;

fn header(vertices: usize, faces: usize) -> String {
    let lines = [
        "ply".to_owned(),
        "format binary_little_endian 1.0".to_owned(),
        "comment amg-io bench input".to_owned(),
        format!("element vertex {vertices}"),
        "property double x".to_owned(),
        "property double y".to_owned(),
        "property double z".to_owned(),
        "property double nx".to_owned(),
        "property double ny".to_owned(),
        "property double nz".to_owned(),
        "property float s".to_owned(),
        "property float t".to_owned(),
        "property uchar red".to_owned(),
        "property uchar green".to_owned(),
        "property uchar blue".to_owned(),
        "property uchar alpha".to_owned(),
        format!("element face {faces}"),
        "property list uchar int vertex_indices".to_owned(),
        "property uchar red".to_owned(),
        "property uchar green".to_owned(),
        "property uchar blue".to_owned(),
        "property uchar alpha".to_owned(),
        "property float quality".to_owned(),
        "end_header".to_owned(),
    ];
    lines.join("\n") + "\n"
}

fn ply_1gb() -> PathBuf {
    let name = "ply_1gb.ply";
    let path = common::data_dir().join(name);
    let marker = common::data_dir().join(format!("{name}.ok"));
    if !marker.exists() || !path.exists() {
        eprintln!("generating {} (frequency {PLY_FREQ})", path.display());
        let m = common::mesh(PLY_FREQ);
        let tmp = path.with_extension("ply.tmp");
        let file = std::fs::File::create(&tmp).expect("create");
        let mut w = std::io::BufWriter::with_capacity(8 << 20, file);
        w.write_all(header(m.positions.len(), m.tris.len()).as_bytes())
            .expect("header");
        for (i, p) in m.positions.iter().enumerate() {
            let n = p.normalize_or_zero();
            for v in [p.x, p.y, p.z, n.x, n.y, n.z] {
                w.write_all(&v.to_le_bytes()).expect("vertex");
            }
            w.write_all(&(i as f32 * 1e-7).to_le_bytes()).expect("s");
            w.write_all(&(1.0 - i as f32 * 1e-7).to_le_bytes())
                .expect("t");
            let rgba = [
                (i & 255) as u8,
                (i >> 8 & 255) as u8,
                (i >> 16 & 255) as u8,
                255,
            ];
            w.write_all(&rgba).expect("rgba");
        }
        for (i, t) in m.tris.iter().enumerate() {
            w.write_all(&[3]).expect("count");
            for c in t {
                w.write_all(&(*c as i32).to_le_bytes()).expect("index");
            }
            w.write_all(&[(i & 255) as u8, 128, 64, 255]).expect("rgba");
            w.write_all(&(i as f32).to_le_bytes()).expect("quality");
        }
        w.flush().expect("flush");
        drop(w);
        std::fs::rename(&tmp, &path).expect("rename");
        std::fs::write(&marker, b"ok").expect("marker");
    }
    path
}

fn read_once(path: &std::path::Path) -> (Duration, u64) {
    let t = Instant::now();
    let i = import(
        Source::Path(path),
        Some(Format::Ply),
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .expect("import");
    (t.elapsed(), i.report.triangles_read)
}

fn ply_read_1gb(c: &mut Criterion) {
    let path = ply_1gb();
    let bytes = std::fs::metadata(&path).expect("meta").len();
    eprintln!("ply input: {bytes} bytes");
    if common::strict() {
        assert!(
            bytes >= 1_000_000_000,
            "the input is smaller than 1 GB: {bytes}"
        );
    }
    let (t, tris) = read_once(&path);
    eprintln!("ply_read_1gb: {tris} triangles in {t:.2?} (bound 3 s)");
    if common::strict() {
        assert!(t < Duration::from_secs(3), "ply_read_1gb: {t:?}");
    }
    let mut g = c.benchmark_group("io");
    g.sample_size(10)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(30));
    g.bench_function("ply_read_1gb", |b| b.iter(|| read_once(&path)));
    g.finish();
}

criterion_group!(benches, ply_read_1gb);
criterion_main!(benches);
