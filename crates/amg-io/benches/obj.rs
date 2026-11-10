//! IO-05 Done-when 6: `io/obj_read_1gb`.
//!
//! The 1 GB input is generated on first use from the MESH-28 benchmark generator (displaced
//! geodesic sphere, 20 M triangles, `Roundtrip` coordinates) with our own OBJ writer into `CARGO_TARGET_DIR` (or
//! `AMG_IO_BENCH_DIR`), never into the repository. The < 8 s bound is asserted only under
//! `AMG_STRICT_TIMING=1`.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

mod common;

use std::path::PathBuf;
use std::time::{Duration, Instant};

use amg_io::{
    CoordPrecision, ExportOptions, Format, ImportOptions, Sink, Source, ValidityPolicy, export,
    import,
};
use amg_jobs::JobCtx;
use criterion::{Criterion, criterion_group, criterion_main};

/// Frequency 1000 (the generator's maximum): 20 M triangles, 10 M vertices. With shortest
/// round-trip coordinates (about 17 digits) that is about 1.1 GB of OBJ text.
const OBJ_FREQ: u32 = 1000;

fn obj_1gb() -> PathBuf {
    let name = "obj_1gb.obj";
    let path = common::data_dir().join(name);
    let marker = common::data_dir().join(format!("{name}.ok"));
    if !marker.exists() || !path.exists() {
        eprintln!("generating {} (frequency {OBJ_FREQ})", path.display());
        let s = common::scene(common::mesh(OBJ_FREQ));
        let opts = ExportOptions {
            precision: CoordPrecision::Roundtrip,
            validity: Some(ValidityPolicy::AllowInvalid),
            ..ExportOptions::default()
        };
        export(
            &s,
            Sink::Path(&path),
            Format::Obj,
            &opts,
            &JobCtx::detached(),
        )
        .expect("generate bench input");
        std::fs::write(&marker, b"ok").expect("marker");
    }
    path
}

fn read_once(path: &std::path::Path) -> (Duration, u64) {
    let t = Instant::now();
    let i = import(
        Source::Path(path),
        Some(Format::Obj),
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .expect("import");
    (t.elapsed(), i.report.triangles_read)
}

fn obj_read_1gb(c: &mut Criterion) {
    let path = obj_1gb();
    let bytes = std::fs::metadata(&path).expect("meta").len();
    eprintln!("obj input: {bytes} bytes");
    if common::strict() {
        assert!(
            bytes >= 1_000_000_000,
            "the input is smaller than 1 GB: {bytes}"
        );
    }
    let (t, tris) = read_once(&path);
    eprintln!("obj_read_1gb: {tris} triangles in {t:.2?} (bound 8 s)");
    if common::strict() {
        assert!(t < Duration::from_secs(8), "obj_read_1gb: {t:?}");
    }
    let mut g = c.benchmark_group("io");
    g.sample_size(10)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(30));
    g.bench_function("obj_read_1gb", |b| b.iter(|| read_once(&path)));
    g.finish();
}

criterion_group!(benches, obj_read_1gb);
criterion_main!(benches);
