//! IO-03 Done-when 5 and 7: `io/3mf_read_10m` (< 6 s) and cancel latency (< 100 ms).
//!
//! The input is generated on first use from the MESH-28 benchmark generator (displaced
//! geodesic sphere, frequency 708: 10 025 280 triangles) as a plain 3MF (one model part,
//! Deflate level 1, 6-decimal coordinates), written into `CARGO_TARGET_DIR` (or
//! `AMG_IO_BENCH_DIR`), never into the repository. The 3MF writer is IO-11, so the bench
//! writes the XML itself. Bounds are asserted only under `AMG_STRICT_TIMING=1`.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

mod common;

use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use amg_io::{Format, ImportOptions, IoError, Source, import};
use amg_jobs::{CancelToken, JobCtx};
use criterion::{Criterion, criterion_group, criterion_main};

/// 20 · 708² = 10 025 280 triangles.
const FREQ: u32 = 708;

fn write_3mf(path: &Path) {
    let mesh = common::mesh(FREQ);
    let f = std::fs::File::create(path).expect("create");
    let mut z = zip::ZipWriter::new(BufWriter::with_capacity(1 << 20, f));
    let o = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated)
        .compression_level(Some(1))
        .large_file(true);
    z.start_file("[Content_Types].xml", o).unwrap();
    z.write_all(br#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>"#).unwrap();
    z.start_file("_rels/.rels", o).unwrap();
    z.write_all(br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>"#).unwrap();
    z.start_file("3D/3dmodel.model", o).unwrap();
    let mut w = BufWriter::with_capacity(8 << 20, &mut z);
    w.write_all(br#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources>
  <object id="1" type="model" name="sphere">
   <mesh>
    <vertices>
"#).unwrap();
    for p in &mesh.positions {
        writeln!(
            w,
            r#"     <vertex x="{:.6}" y="{:.6}" z="{:.6}"/>"#,
            p.x, p.y, p.z
        )
        .unwrap();
    }
    w.write_all(b"    </vertices>\n    <triangles>\n").unwrap();
    for t in &mesh.tris {
        writeln!(
            w,
            r#"     <triangle v1="{}" v2="{}" v3="{}"/>"#,
            t[0], t[1], t[2]
        )
        .unwrap();
    }
    w.write_all(b"    </triangles>\n   </mesh>\n  </object>\n </resources>\n <build>\n  <item objectid=\"1\"/>\n </build>\n</model>\n").unwrap();
    w.flush().unwrap();
    drop(w);
    z.finish().unwrap().flush().unwrap();
}

fn input() -> PathBuf {
    let name = "3mf_10m.3mf";
    let path = common::data_dir().join(name);
    let marker = common::data_dir().join(format!("{name}.ok"));
    if !marker.exists() || !path.exists() {
        eprintln!("generating {} (frequency {FREQ})", path.display());
        write_3mf(&path);
        std::fs::write(&marker, b"ok").expect("marker");
    }
    path
}

fn read_once(path: &Path) -> (Duration, u64) {
    let t = Instant::now();
    let i = import(
        Source::Path(path),
        Some(Format::ThreeMf),
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .expect("import");
    (t.elapsed(), i.report.triangles_read)
}

/// Done-when 5.
fn threemf_read_10m(c: &mut Criterion) {
    let path = input();
    let bytes = std::fs::metadata(&path).expect("meta").len();
    let (t, tris) = read_once(&path);
    eprintln!("3mf_read_10m: {tris} triangles, {bytes} bytes zipped, in {t:.2?} (bound 6 s)");
    assert_eq!(tris, 20 * u64::from(FREQ) * u64::from(FREQ));
    if common::strict() {
        assert!(t < Duration::from_secs(6), "3mf_read_10m: {t:?}");
    }
    let mut g = c.benchmark_group("io");
    g.sample_size(10)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(40));
    g.bench_function("3mf_read_10m", |b| b.iter(|| read_once(&path)));
    g.finish();
}

/// Done-when 7: cancelling the import returns `Cancelled` within 100 ms, early, mid-way and late.
fn threemf_cancel_latency(_c: &mut Criterion) {
    let path = input();
    let (full, _) = read_once(&path);
    let full_ms = full.as_millis() as u64;
    let mut worst = Duration::ZERO;
    for delay_ms in [
        full_ms / 20,
        full_ms * 2 / 5,
        full_ms * 3 / 4,
        full_ms * 9 / 10,
    ] {
        let token = CancelToken::new();
        let ctx = JobCtx::with_token(token.clone());
        let worker = {
            let path = path.clone();
            std::thread::spawn(move || {
                import(
                    Source::Path(&path),
                    Some(Format::ThreeMf),
                    &ImportOptions::default(),
                    &ctx,
                )
                .map(|_| ())
            })
        };
        std::thread::sleep(Duration::from_millis(delay_ms));
        let t = Instant::now();
        token.cancel();
        let r = worker.join().expect("join");
        let latency = t.elapsed();
        eprintln!("cancel after {delay_ms} ms of {full_ms}: {r:?}, returned {latency:.2?} later");
        assert!(
            matches!(r, Err(IoError::Cancelled)),
            "finished before the cancel ({delay_ms} ms): {r:?}"
        );
        worst = worst.max(latency);
    }
    eprintln!("3mf_cancel_latency: worst {worst:.2?} (bound 100 ms)");
    if common::strict() {
        assert!(worst < Duration::from_millis(100), "{worst:?}");
    }
}

criterion_group!(benches, threemf_read_10m, threemf_cancel_latency);
criterion_main!(benches);
