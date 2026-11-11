//! IO-03 Done-when 6: a zip bomb, 100-deep XML, a component cycle and an encrypted package
//! each give the expected error in < 1 s and < 100 MB.
//!
//! Memory is the peak heap measured with `dhat` (this test binary's global allocator); nextest
//! runs every test in its own process, so each test has its own profiler. The time bound is
//! asserted only under `AMG_STRICT_TIMING=1` (it is printed always).

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::print_stderr,
    reason = "test code; a failed expectation is a test failure"
)]

mod threemf_common;

use std::io::Write;
use std::time::{Duration, Instant};

use amg_io::{Format, ImportOptions, IoError, Source, import};
use amg_jobs::JobCtx;
use threemf_common::*;

#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

const MAX_HEAP: usize = 100 << 20;

fn strict_timing() -> bool {
    std::env::var_os("AMG_STRICT_TIMING").is_some()
}

/// Imports `bytes` under a fresh heap profile; returns the error, time and peak heap.
fn measure(name: &str, bytes: &[u8]) -> IoError {
    let _p = dhat::Profiler::builder().testing().build();
    let before = dhat::HeapStats::get();
    let t = Instant::now();
    let r = import(
        Source::Bytes(bytes),
        Some(Format::ThreeMf),
        &ImportOptions::default(),
        &JobCtx::detached(),
    );
    let dt = t.elapsed();
    let stats = dhat::HeapStats::get();
    let peak = stats.max_bytes.saturating_sub(before.curr_bytes);
    eprintln!(
        "{name}: {dt:.2?}, peak heap {:.1} MB ({} input bytes)",
        peak as f64 / 1e6,
        bytes.len()
    );
    assert!(peak < MAX_HEAP, "{name}: peak heap {peak} bytes");
    if strict_timing() {
        assert!(dt < Duration::from_secs(1), "{name}: {dt:?}");
    }
    match r {
        Err(e) => e,
        Ok(_) => panic!("{name}: loaded"),
    }
}

/// A model part of 1 GiB of zeros (Deflate ≈ 1000:1), streamed into the zip.
fn zip_bomb() -> Vec<u8> {
    let mut w = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
    let o = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated)
        .compression_level(Some(9))
        .large_file(true);
    w.start_file("[Content_Types].xml", o).unwrap();
    w.write_all(CONTENT_TYPES.as_bytes()).unwrap();
    w.start_file("_rels/.rels", o).unwrap();
    w.write_all(ROOT_RELS.as_bytes()).unwrap();
    w.start_file("3D/3dmodel.model", o).unwrap();
    let zeros = vec![b' '; 1 << 20];
    for _ in 0..1024 {
        w.write_all(&zeros).unwrap();
    }
    w.finish().unwrap().into_inner()
}

#[test]
fn zip_bomb_hits_the_ratio_limit() {
    let b = zip_bomb();
    assert!(b.len() < 4 << 20, "bomb is {} bytes", b.len());
    let e = measure("zip bomb", &b);
    assert!(
        matches!(
            e,
            IoError::LimitExceeded {
                what: "compression ratio",
                ..
            }
        ),
        "{e:?}"
    );
}

#[test]
fn deep_xml_hits_the_depth_limit() {
    let deep = format!(
        r#"<v:a xmlns:v="urn:v">{}{}</v:a>"#,
        "<v:a>".repeat(100),
        "</v:a>".repeat(100)
    );
    let m = model("", &deep, "");
    let e = measure("100-deep XML", &package(&m, &[]));
    assert!(
        matches!(
            e,
            IoError::LimitExceeded {
                what: "XML depth",
                ..
            }
        ),
        "{e:?}"
    );
}

#[test]
fn component_cycle_is_malformed() {
    let m = model(
        "",
        r#"<object id="1"><components><component objectid="2"/></components></object>
           <object id="2"><components><component objectid="1"/></components></object>"#,
        r#"<item objectid="1"/>"#,
    );
    let e = measure("component cycle", &package(&m, &[]));
    assert!(
        matches!(&e, IoError::Malformed { message, .. } if message.contains("cycle")),
        "{e:?}"
    );
}

#[test]
fn encrypted_package_is_refused() {
    // Secure Content: a keystore part with its content type and relationship.
    let ct = CONTENT_TYPES.replace(
        "</Types>",
        r#"<Override PartName="/Secure/keystore.xml" ContentType="application/vnd.ms-package.3dmanufacturing-keystore+xml"/></Types>"#,
    );
    let rels = ROOT_RELS.replace(
        "</Relationships>",
        r#"<Relationship Target="/Secure/keystore.xml" Id="rel1" Type="http://schemas.microsoft.com/3dmanufacturing/2019/04/keystore"/></Relationships>"#,
    );
    let m = model("", "", "");
    let b = zip(
        &[
            ("[Content_Types].xml", ct.as_bytes()),
            ("_rels/.rels", rels.as_bytes()),
            ("3D/3dmodel.model", m.as_bytes()),
            ("Secure/keystore.xml", b"<keystore/>"),
        ],
        false,
    );
    let e = measure("encrypted", &b);
    assert!(matches!(e, IoError::Encrypted), "{e:?}");
}

#[test]
fn oversized_entry_hits_the_entry_limit_before_inflating() {
    // Declared sizes are checked before any byte is inflated.
    let b = zip_bomb();
    let mut o = ImportOptions::default();
    o.limits.max_entry_uncompressed = 1 << 20;
    let r = import(
        Source::Bytes(&b),
        Some(Format::ThreeMf),
        &o,
        &JobCtx::detached(),
    );
    assert!(
        matches!(
            r,
            Err(IoError::LimitExceeded {
                what: "zip entry size",
                ..
            })
        ),
        "{r:?}"
    );
    let mut o = ImportOptions::default();
    o.limits.max_total_uncompressed = 1 << 20;
    let r = import(
        Source::Bytes(&b),
        Some(Format::ThreeMf),
        &o,
        &JobCtx::detached(),
    );
    assert!(
        matches!(
            r,
            Err(IoError::LimitExceeded {
                what: "total uncompressed size",
                ..
            })
        ),
        "{r:?}"
    );
}
