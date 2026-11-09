//! Corpus harness (IO-01): iterates `testdata/io/corpus.toml` and the MESH-28 manifest.
//!
//! - `head` entries run `Format::sniff` (the sniff table, ≥ 30 heads).
//! - `file` entries run `probe` and `Format::sniff`.
//! - Fetch-only entries that are absent are skipped; un-fetched git-LFS pointer files are
//!   skipped unless `AMG_REQUIRE_TESTDATA=1`, which turns them into failures. The same rule as
//!   the `amg-mesh` corpus tests.
//! - Reader WPs (IO-02, IO-03, ...) extend [`check_entry`] with `triangles`/`objects`/`unit`.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test harness; a failed read is a test failure"
)]

use std::path::{Path, PathBuf};

use amg_core::Diagnostic;
use amg_io::{Format, IoError, ReadLimits, Source, probe};
use sha2::{Digest, Sha256};

const LFS_PREFIX: &[u8] = b"version https://git-lfs.github.com/spec/v1";

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

#[derive(Debug, serde::Deserialize)]
struct Manifest {
    corpus: CorpusMeta,
    #[serde(default)]
    entry: Vec<Entry>,
}

#[derive(Debug, serde::Deserialize)]
struct CorpusMeta {
    format: u32,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct Entry {
    id: String,
    file: String,
    #[serde(default = "default_kind")]
    kind: String,
    file_len: Option<u64>,
    format: Option<String>,
    expect: String,
    license: String,
    source_url: String,
    sha256: Option<String>,
    #[serde(default)]
    fetch: bool,
    // Asserted by the reader WPs.
    #[allow(dead_code)]
    triangles: Option<u64>,
    #[allow(dead_code)]
    objects: Option<u32>,
    #[allow(dead_code)]
    unit: Option<String>,
}

impl Entry {
    fn format(&self) -> Option<Format> {
        self.format.as_deref().map(|f| {
            Format::from_extension(f).unwrap_or_else(|| panic!("{}: unknown format {f}", self.id))
        })
    }
}

fn default_kind() -> String {
    "file".into()
}

fn io_root() -> PathBuf {
    amg_testkit::testdata_dir().join("io")
}

fn load(path: &Path) -> Manifest {
    let text = std::fs::read_to_string(path).expect("read manifest");
    toml::from_str(&text).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

/// What the harness did with one entry.
#[derive(Debug, PartialEq, Eq)]
enum Outcome {
    Checked,
    Skipped(String),
}

fn is_lfs_pointer(bytes: &[u8]) -> bool {
    bytes.len() <= 1024 && bytes.starts_with(LFS_PREFIX)
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

/// Resolves and checks one entry. `require` = `AMG_REQUIRE_TESTDATA=1`.
fn check_entry(root: &Path, e: &Entry, require: bool) -> Outcome {
    let path = root.join(&e.file);
    if !path.is_file() {
        assert!(
            e.fetch,
            "{}: file {} is missing and the entry is not fetch-only",
            e.id,
            path.display()
        );
        return Outcome::Skipped(format!("{}: fetch-only file not fetched", e.id));
    }
    let bytes = std::fs::read(&path).expect("read file");
    if is_lfs_pointer(&bytes) {
        assert!(
            !require,
            "{}: {} is an un-fetched git-LFS pointer and AMG_REQUIRE_TESTDATA=1; run `git lfs pull`",
            e.id,
            path.display()
        );
        return Outcome::Skipped(format!("{}: un-fetched git-LFS pointer", e.id));
    }
    if let Some(sha) = &e.sha256 {
        assert_eq!(&sha256_hex(&bytes), sha, "{}: sha256 mismatch", e.id);
    }
    let head = &bytes[..bytes.len().min(4096)];
    let len = match e.kind.as_str() {
        "head" => e.file_len.expect("head entries carry file_len"),
        "file" => bytes.len() as u64,
        other => panic!("{}: unknown kind {other}", e.id),
    };
    let want = e.format();
    assert_eq!(
        Format::sniff(head, len),
        want,
        "{}: sniff result (note: {:?})",
        e.id,
        e.file
    );
    if e.kind == "file" {
        let r = probe(Source::Bytes(&bytes), &ReadLimits::default());
        match (e.expect.as_str(), want, r) {
            ("ok", Some(f), Ok(info)) => assert_eq!(info.format, f, "{}: probe format", e.id),
            ("UnknownFormat", None, Err(IoError::UnknownFormat)) => {}
            (expect, fmt, r) => panic!("{}: expected {expect}/{fmt:?}, probe gave {r:?}", e.id),
        }
        // The path source must agree with the bytes source.
        let r2 = probe(Source::Path(&path), &ReadLimits::default());
        assert_eq!(
            r2.as_ref().map(|i| i.format).ok(),
            want,
            "{}: probe(Path)",
            e.id
        );
    }
    Outcome::Checked
}

#[test]
fn manifest_is_well_formed() {
    let m = load(&io_root().join("corpus.toml"));
    assert_eq!(m.corpus.format, 1);
    let mut ids = std::collections::BTreeSet::new();
    for e in &m.entry {
        assert!(ids.insert(e.id.clone()), "duplicate id {}", e.id);
        assert!(!e.license.is_empty(), "{}: licence missing", e.id);
        assert!(!e.source_url.is_empty(), "{}: source_url missing", e.id);
        assert!(e.fetch || e.sha256.is_some(), "{}: sha256 missing", e.id);
        assert_eq!(e.format().is_some(), e.expect == "ok", "{}", e.id);
        assert!(matches!(e.kind.as_str(), "file" | "head"), "{}", e.id);
        assert_eq!(e.kind == "head", e.file_len.is_some(), "{}", e.id);
    }
}

#[test]
fn every_manifest_entry_checks_out() {
    let root = io_root();
    let m = load(&root.join("corpus.toml"));
    let mut checked = 0;
    for e in &m.entry {
        match check_entry(&root, e, require_testdata()) {
            Outcome::Checked => checked += 1,
            Outcome::Skipped(why) => eprintln!("SKIP: {why}"),
        }
    }
    assert!(
        checked >= 30,
        "the sniff table needs at least 30 fixture heads, checked {checked}"
    );
}

#[test]
fn sniff_table_covers_the_required_cases() {
    let m = load(&io_root().join("corpus.toml"));
    let by_id = |id: &str| {
        m.entry
            .iter()
            .find(|e| e.id == id)
            .unwrap_or_else(|| panic!("fixture {id} missing"))
    };
    // Spec IO-01 Done-when 1: named cases.
    assert_eq!(by_id("stl-binary-solid-header").format(), Some(Format::Stl));
    assert_eq!(by_id("stl-ascii-bom").format(), Some(Format::Stl));
    assert_eq!(by_id("none-amg-project-zip").format(), None);
    assert_eq!(by_id("step-lf").format(), Some(Format::Step));
    assert_eq!(by_id("iges-lf").format(), Some(Format::Iges));
    assert_eq!(by_id("dxf-lf").format(), Some(Format::Dxf));
    assert_eq!(by_id("svg-bare").format(), Some(Format::Svg));
    // Every format is represented, and several fixtures are negative.
    for f in [
        Format::Stl,
        Format::ThreeMf,
        Format::Obj,
        Format::Ply,
        Format::Step,
        Format::Iges,
        Format::Dxf,
        Format::Svg,
    ] {
        assert!(m.entry.iter().any(|e| e.format() == Some(f)), "{f:?}");
    }
    assert!(m.entry.iter().filter(|e| e.format.is_none()).count() >= 8);
}

#[test]
fn amg_container_is_unknown_format_with_a_file_open_hint() {
    let root = io_root();
    let bytes = std::fs::read(root.join("heads/none-amg-project-zip.fixture")).expect("fixture");
    let e = probe(Source::Bytes(&bytes), &ReadLimits::default()).expect_err("must not import");
    assert!(matches!(e, IoError::UnknownFormat));
    let hint = e.hint().expect("hint");
    assert!(
        hint.contains(".amg") && hint.contains("File > Open"),
        "{hint}"
    );
}

#[test]
fn probe_reports_slicer_data_and_binary_stl_estimate() {
    let root = io_root();
    let read = |n: &str| std::fs::read(root.join(format!("heads/{n}.fixture"))).expect("fixture");
    let lim = ReadLimits::default();
    let plain = probe(Source::Bytes(&read("3mf-content-types-first")), &lim).expect("3mf");
    assert!(!plain.has_slicer_data);
    let orca = probe(Source::Bytes(&read("3mf-slicer-project")), &lim).expect("3mf");
    assert!(orca.has_slicer_data);
    let stl = probe(Source::Bytes(&read("stl-binary-plain")), &lim).expect("stl");
    assert_eq!((stl.format, stl.triangles_estimate), (Format::Stl, Some(2)));
    let ascii = probe(Source::Bytes(&read("stl-ascii-lf")), &lim).expect("stl");
    assert_eq!(ascii.triangles_estimate, None);
}

/// Fetch-only entries and LFS pointers must be skipped, not failed. Synthetic manifest in a
/// temp dir, so this runs the same code as the real harness.
#[test]
fn harness_skips_absent_fetch_only_entries_and_lfs_pointers() {
    let dir = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        dir.path().join("pointer.stl"),
        b"version https://git-lfs.github.com/spec/v1\noid sha256:00\nsize 123\n",
    )
    .expect("write");
    std::fs::write(dir.path().join("real.obj"), b"v 0 0 0\n").expect("write");
    let manifest = r#"
[corpus]
format = 1

[[entry]]
id = "absent-fetch-only"
file = "fetch/big.3mf"
format = "3mf"
expect = "ok"
license = "BSD-2-Clause"
source_url = "https://example.invalid/"
fetch = true

[[entry]]
id = "lfs-pointer"
file = "pointer.stl"
format = "stl"
expect = "ok"
license = "CC0-1.0"
source_url = "x"
sha256 = "00"

[[entry]]
id = "present"
file = "real.obj"
format = "obj"
expect = "ok"
license = "CC0-1.0"
source_url = "x"
"#;
    let m: Manifest = toml::from_str(manifest).expect("parse");
    let outcomes: Vec<_> = m
        .entry
        .iter()
        .map(|e| check_entry(dir.path(), e, false))
        .collect();
    assert!(matches!(&outcomes[0], Outcome::Skipped(w) if w.contains("fetch-only")));
    assert!(matches!(&outcomes[1], Outcome::Skipped(w) if w.contains("LFS")));
    assert_eq!(outcomes[2], Outcome::Checked);

    // AMG_REQUIRE_TESTDATA=1: the pointer fails, the absent fetch-only entry still skips.
    let m2: Manifest = toml::from_str(manifest).expect("parse");
    let strict = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        check_entry(dir.path(), &m2.entry[1], true)
    }));
    assert!(strict.is_err(), "pointer + require must fail");
    assert!(matches!(
        check_entry(dir.path(), &m2.entry[0], true),
        Outcome::Skipped(_)
    ));
}

/// The MESH-28 corpus (git-LFS) is real data: every STL/PLY in it must sniff as such.
#[test]
fn mesh_corpus_files_sniff_as_their_declared_format() {
    #[derive(serde::Deserialize)]
    struct MeshManifest {
        mesh: Vec<MeshEntry>,
    }
    #[derive(serde::Deserialize)]
    struct MeshEntry {
        id: String,
        file: String,
        format: String,
        #[serde(default)]
        fetch: bool,
    }
    let root = amg_testkit::testdata_dir().join("mesh");
    let text = std::fs::read_to_string(root.join("corpus.toml")).expect("mesh manifest");
    let m: MeshManifest = toml::from_str(&text).expect("parse mesh manifest");
    let (mut checked, mut skipped) = (0, 0);
    for e in &m.mesh {
        let Some(want) = Format::from_extension(&e.format) else {
            continue;
        };
        let path = root.join(&e.file);
        if !path.is_file() {
            assert!(e.fetch, "{}: {} missing", e.id, path.display());
            skipped += 1;
            continue;
        }
        let (head, len) = Source::Path(&path).head(4096).expect("head");
        if is_lfs_pointer(&head) {
            assert!(!require_testdata(), "{}: un-fetched LFS pointer", e.id);
            skipped += 1;
            continue;
        }
        assert_eq!(Format::sniff(&head, len), Some(want), "{}", e.id);
        let info = probe(Source::Path(&path), &ReadLimits::default()).expect("probe");
        assert_eq!(info.format, want, "{}", e.id);
        checked += 1;
    }
    eprintln!("mesh corpus: {checked} sniffed, {skipped} skipped");
    assert!(
        checked + skipped >= 100,
        "the mesh manifest should list the whole corpus"
    );
}
