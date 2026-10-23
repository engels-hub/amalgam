//! DOC-07: the `.amg` container, migrations, diagnostics, safe save, jobs.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use amg_core::{Point3, Transform};
use amg_doc::io::{
    DOC_MIGRATIONS, FORMAT_VERSION, SavePhase, StopAction, StubBodyCodec, load_in_job,
    migrate_document, save_in_job,
};
use amg_doc::testing::{TestBox, TestCounter};
use amg_doc::{
    BakedBody, BlobRef, BodyGeom, BodyId, BodyProps, Design, DesignSnapshot, ExtRecord, FeatureBox,
    FormatError, SaveOptions, load, save,
};
use amg_jobs::{CancelToken, JobCtx, JobSystem, JobSystemConfig};
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, ZipArchive, ZipWriter};

// ---- helpers ----------------------------------------------------------------------------

struct TempDir(PathBuf);

impl TempDir {
    fn new() -> TempDir {
        static N: AtomicU32 = AtomicU32::new(0);
        let p = std::env::temp_dir().join(format!(
            "amg-doc-format-{}-{}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&p).unwrap();
        TempDir(p)
    }
    fn path(&self, name: &str) -> PathBuf {
        self.0.join(name)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn mesh_bytes() -> Vec<u8> {
    let mesh = amg_mesh::IndexedMesh::new(
        vec![
            Point3::new(0.0, 0.1, 0.2),
            Point3::new(1.0 / 3.0, 0.0, f64::MIN_POSITIVE),
            Point3::new(0.0, 1.0, -0.0),
        ],
        vec![[0, 1, 2]],
    )
    .unwrap();
    let mut out = Vec::new();
    amg_mesh::MeshBody::new(mesh)
        .write_binary(&mut out)
        .unwrap();
    out
}

fn blob(p: &str) -> BlobRef {
    BlobRef(p.to_owned())
}

/// A design exercising features, props, a baked body, an extension record with a blob.
fn sample() -> Design {
    amg_doc::testing::link();
    let mut d = Design::new("Bracket");
    let root = d.root();
    let f1 = d
        .add_feature(
            None,
            FeatureBox::new(TestBox {
                size: [10.0, 20.0, 0.1 + 0.2],
            }),
            root,
            Some("Base".into()),
        )
        .unwrap();
    d.add_feature(
        None,
        FeatureBox::new(TestCounter {
            deps: vec![],
            value: -7,
            fail: false,
        }),
        root,
        None,
    )
    .unwrap();
    d.add_component(root, "Arm", Transform::IDENTITY).unwrap();
    d.set_body_props(
        BodyId {
            origin: f1,
            slot: 0,
        },
        BodyProps {
            name: Some("Plate".into()),
            opacity: 0.75,
            ..BodyProps::default()
        },
    )
    .unwrap();
    d.set_baked(
        BodyId {
            origin: f1,
            slot: 3,
        },
        Some(BakedBody {
            component: root,
            blob: blob("bodies/1-3.mesh"),
            geom: Some(BodyGeom::Mesh(Arc::default())),
        }),
    )
    .unwrap();
    let key = d.new_ext_key("fem.study").unwrap();
    d.set_ext(
        key,
        Some(ExtRecord {
            kind: "fem.study".into(),
            version: 1,
            data: serde_json::json!({ "loads": [1.5, 2.0], "n": null }),
            blobs: vec![blob("ext/fem.study-9/result.bin")],
        }),
    )
    .unwrap();
    d
}

fn sample_opts() -> SaveOptions {
    let mut o = SaveOptions::default();
    o.blobs
        .insert(blob("ext/fem.study-9/result.bin"), mesh_bytes().into());
    o.thumbnail = Some(vec![0x89, b'P', b'N', b'G', 1, 2, 3]);
    o
}

fn snap(d: Design) -> DesignSnapshot {
    Arc::new(d)
}

fn entries_of(path: &Path) -> BTreeMap<String, Vec<u8>> {
    let mut zip = ZipArchive::new(File::open(path).unwrap()).unwrap();
    let mut out = BTreeMap::new();
    for i in 0..zip.len() {
        let mut e = zip.by_index(i).unwrap();
        let mut data = Vec::new();
        e.read_to_end(&mut data).unwrap();
        out.insert(e.name().unwrap().into_owned(), data);
    }
    out
}

/// Writes a container from raw entries. `manifest_version` goes into the manifest; the hashes
/// are computed here unless `bad_hash` names an entry to corrupt.
fn write_container(
    path: &Path,
    manifest_version: u32,
    entries: &BTreeMap<String, Vec<u8>>,
    bad_hash: Option<&str>,
) {
    let hashes: BTreeMap<_, _> = entries
        .iter()
        .filter(|(k, _)| *k != "manifest.json")
        .map(|(k, v)| {
            let h = if bad_hash == Some(k.as_str()) {
                "0".repeat(64)
            } else {
                blake3::hash(v).to_hex().to_string()
            };
            (k.clone(), h)
        })
        .collect();
    let manifest = serde_json::json!({
        "format": "amalgam",
        "format_version": manifest_version,
        "app_version": "test",
        "created_by": "test",
        "entries": hashes,
    });
    let mut zip = ZipWriter::new(File::create(path).unwrap());
    let opts = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);
    zip.start_file("manifest.json", opts).unwrap();
    zip.write_all(&serde_json::to_vec_pretty(&manifest).unwrap())
        .unwrap();
    for (k, v) in entries.iter().filter(|(k, _)| *k != "manifest.json") {
        zip.start_file(k.as_str(), opts).unwrap();
        zip.write_all(v).unwrap();
    }
    zip.finish().unwrap();
}

/// Saves `sample()`, lets `edit` change design.json, writes it back with a valid manifest.
fn tampered(dir: &TempDir, name: &str, edit: impl FnOnce(&mut serde_json::Value)) -> PathBuf {
    let src = dir.path("src.amg");
    save(&src, &snap(sample()), &sample_opts()).unwrap();
    let mut entries = entries_of(&src);
    let mut v: serde_json::Value = serde_json::from_slice(&entries["design.json"]).unwrap();
    edit(&mut v);
    entries.insert("design.json".into(), serde_json::to_vec_pretty(&v).unwrap());
    let out = dir.path(name);
    write_container(&out, FORMAT_VERSION, &entries, None);
    out
}

// ---- 1. round trip ----------------------------------------------------------------------

#[test]
fn save_load_save_is_byte_identical() {
    let dir = TempDir::new();
    let (a, b) = (dir.path("a.amg"), dir.path("b.amg"));
    let d = sample();
    save(&a, &snap(d.clone()), &sample_opts()).unwrap();
    let loaded = load(&a).unwrap();
    assert_eq!(loaded.file_version, FORMAT_VERSION);
    assert!(loaded.warnings.is_empty());
    assert_eq!(
        serde_json::to_string(&loaded.design).unwrap(),
        serde_json::to_string(&d).unwrap()
    );
    let mut opts = sample_opts();
    opts.blobs = loaded.blobs.clone();
    opts.thumbnail = loaded.thumbnail.clone();
    save(&b, &snap(loaded.design), &opts).unwrap();
    let (ea, eb) = (entries_of(&a), entries_of(&b));
    assert_eq!(ea["design.json"], eb["design.json"], "design.json bytes");
    assert_eq!(ea, eb, "every entry, manifest included");
    assert_eq!(
        fs::read(&a).unwrap(),
        fs::read(&b).unwrap(),
        "the whole archive is deterministic"
    );
}

#[test]
fn baked_bodies_blobs_and_thumbnail_round_trip() {
    let dir = TempDir::new();
    let p = dir.path("a.amg");
    save(&p, &snap(sample()), &sample_opts()).unwrap();
    let names: Vec<_> = entries_of(&p).into_keys().collect();
    assert_eq!(
        names,
        [
            "bodies/1-3.mesh",
            "design.json",
            "ext/fem.study-9/result.bin",
            "manifest.json",
            "thumbnail.png"
        ]
    );
    let l = load(&p).unwrap();
    let (_, baked) = l.design.baked_bodies().next().unwrap();
    assert!(
        matches!(baked.geom, Some(BodyGeom::Mesh(_))),
        "geometry filled in"
    );
    // A MeshBody's binary form (S07) survives the container bit-exact (the future body codec
    // stores exactly these bytes in `bodies/`).
    let blob_bytes = &l.blobs[&blob("ext/fem.study-9/result.bin")];
    assert_eq!(&blob_bytes[..], &mesh_bytes()[..]);
    let back = amg_mesh::MeshBody::read_binary(&blob_bytes[..]).unwrap();
    let mut again = Vec::new();
    back.write_binary(&mut again).unwrap();
    assert_eq!(again, mesh_bytes());
    assert_eq!(l.thumbnail.as_deref(), sample_opts().thumbnail.as_deref());
}

#[test]
fn unknown_feature_loads_with_a_warning_and_saves_back_unchanged() {
    let dir = TempDir::new();
    let p = tampered(&dir, "u.amg", |v| {
        let node = &mut v["timeline"]["nodes"][0];
        node["kind"] = "future.thing".into();
        node["v"] = 9.into();
        node["data"] = serde_json::json!({ "x": [1, 2.5] });
    });
    let l = load(&p).unwrap();
    assert_eq!(l.warnings.len(), 1);
    assert!(l.warnings[0].contains("future.thing"));
    let q = dir.path("q.amg");
    let mut opts = sample_opts();
    opts.blobs = l.blobs.clone();
    save(&q, &snap(l.design), &opts).unwrap();
    let value = |f: &Path| -> serde_json::Value {
        serde_json::from_slice(&entries_of(f)["design.json"]).unwrap()
    };
    assert_eq!(value(&p), value(&q));
}

#[test]
fn feature_migration_runs_on_load() {
    let dir = TempDir::new();
    // test.box v1 stored a single `edge`; the registry migrates it.
    let p = tampered(&dir, "m.amg", |v| {
        let node = &mut v["timeline"]["nodes"][0];
        node["v"] = 1.into();
        node["data"] = serde_json::json!({ "edge": 4.0 });
    });
    let l = load(&p).unwrap();
    let (_, node) = l
        .design
        .timeline()
        .node(l.design.timeline().nodes().next().unwrap().id)
        .unwrap();
    assert_eq!(
        node.feature.downcast_ref::<TestBox>().unwrap().size,
        [4.0; 3]
    );
}

// ---- 2. specific errors -----------------------------------------------------------------

#[test]
fn too_new_is_refused_and_the_file_is_untouched() {
    let dir = TempDir::new();
    let p = dir.path("new.amg");
    let mut entries = BTreeMap::new();
    entries.insert("design.json".to_owned(), b"{}".to_vec());
    write_container(&p, FORMAT_VERSION + 1, &entries, None);
    let before = fs::read(&p).unwrap();
    match load(&p) {
        Err(FormatError::TooNew { file, supported }) => {
            assert_eq!((file, supported), (FORMAT_VERSION + 1, FORMAT_VERSION));
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(fs::read(&p).unwrap(), before);
}

#[test]
fn not_amalgam() {
    let dir = TempDir::new();
    let txt = dir.path("x.amg");
    fs::write(&txt, "hello").unwrap();
    assert!(matches!(load(&txt), Err(FormatError::NotAmalgam)));
    let empty = dir.path("e.amg");
    fs::write(&empty, "").unwrap();
    assert!(matches!(load(&empty), Err(FormatError::NotAmalgam)));
    // A zip without a manifest.
    let z = dir.path("z.amg");
    let mut w = ZipWriter::new(File::create(&z).unwrap());
    w.start_file("a.txt", SimpleFileOptions::default()).unwrap();
    w.write_all(b"a").unwrap();
    w.finish().unwrap();
    assert!(matches!(load(&z), Err(FormatError::NotAmalgam)));
    // A manifest of some other format.
    let o = dir.path("o.amg");
    let mut w = ZipWriter::new(File::create(&o).unwrap());
    w.start_file("manifest.json", SimpleFileOptions::default())
        .unwrap();
    w.write_all(br#"{"format":"other","format_version":1}"#)
        .unwrap();
    w.finish().unwrap();
    assert!(matches!(load(&o), Err(FormatError::NotAmalgam)));
}

#[test]
fn truncated_zip_is_corrupt() {
    let dir = TempDir::new();
    let p = dir.path("a.amg");
    save(&p, &snap(sample()), &sample_opts()).unwrap();
    let bytes = fs::read(&p).unwrap();
    for keep in [bytes.len() / 2, bytes.len() - 5, 30] {
        fs::write(&p, &bytes[..keep]).unwrap();
        match load(&p) {
            Err(FormatError::Corrupt { entry }) => assert_eq!(entry, "zip directory", "{keep}"),
            other => panic!("{keep}: {other:?}"),
        }
    }
}

#[test]
fn flipped_byte_is_corrupt_and_names_the_entry() {
    let dir = TempDir::new();
    let p = dir.path("a.amg");
    save(&p, &snap(sample()), &sample_opts()).unwrap();
    let offset = {
        let mut zip = ZipArchive::new(File::open(&p).unwrap()).unwrap();
        let e = zip.by_name("design.json").unwrap();
        e.data_start().unwrap() as usize + 20
    };
    let mut bytes = fs::read(&p).unwrap();
    bytes[offset] ^= 0x40;
    fs::write(&p, &bytes).unwrap();
    match load(&p) {
        Err(FormatError::Corrupt { entry }) => assert_eq!(entry, "design.json"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn manifest_hash_mismatch_is_corrupt() {
    let dir = TempDir::new();
    let src = dir.path("src.amg");
    save(&src, &snap(sample()), &sample_opts()).unwrap();
    let entries = entries_of(&src);
    for bad in [
        "design.json",
        "ext/fem.study-9/result.bin",
        "bodies/1-3.mesh",
    ] {
        let p = dir.path("h.amg");
        write_container(&p, FORMAT_VERSION, &entries, Some(bad));
        match load(&p) {
            Err(FormatError::Corrupt { entry }) => assert_eq!(entry, bad),
            other => panic!("{bad}: {other:?}"),
        }
    }
    // A blob the design references but the container lacks.
    let mut missing = entries.clone();
    missing.remove("ext/fem.study-9/result.bin");
    let p = dir.path("miss.amg");
    write_container(&p, FORMAT_VERSION, &missing, None);
    assert!(matches!(
        load(&p),
        Err(FormatError::Corrupt { entry }) if entry == "ext/fem.study-9/result.bin"
    ));
}

#[test]
fn bad_json_reports_the_path() {
    let dir = TempDir::new();
    // Syntax error.
    let src = dir.path("src.amg");
    save(&src, &snap(sample()), &sample_opts()).unwrap();
    let mut entries = entries_of(&src);
    entries.insert("design.json".into(), b"{ \"meta\": ".to_vec());
    let p = dir.path("syntax.amg");
    write_container(&p, FORMAT_VERSION, &entries, None);
    match load(&p) {
        Err(FormatError::Json { path, msg }) => {
            assert_eq!(path, ".");
            assert!(msg.contains("line 1"), "{msg}");
        }
        other => panic!("{other:?}"),
    }
    // Wrong type deep inside: the path names it.
    let p = tampered(&dir, "type.amg", |v| {
        v["meta"]["default_units"]["length"] = 12.into()
    });
    match load(&p) {
        Err(FormatError::Json { path, msg }) => {
            assert_eq!(path, "meta.default_units.length");
            assert!(!msg.is_empty());
        }
        other => panic!("{other:?}"),
    }
    // A bad feature node is reported under `timeline`.
    let p = tampered(&dir, "node.amg", |v| {
        v["timeline"]["nodes"][1]["data"] = serde_json::json!({ "value": "x" });
    });
    match load(&p) {
        Err(FormatError::Json { path, msg }) => {
            assert_eq!(path, "timeline");
            assert!(msg.contains("timeline node 1"), "{msg}");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn invalid_and_missing_blobs_are_rejected_on_save() {
    let dir = TempDir::new();
    let p = dir.path("a.amg");
    // Missing payload for a referenced ext blob.
    let err = save(&p, &snap(sample()), &SaveOptions::default()).unwrap_err();
    assert!(matches!(err, FormatError::MissingBlob(b) if b == "ext/fem.study-9/result.bin"));
    assert!(
        !p.exists() && !dir.path("a.amg.tmp").exists(),
        "failed save leaves nothing"
    );
    // Path escapes.
    let mut d = sample();
    let f = d.timeline().nodes().next().unwrap().id;
    let root = d.root();
    d.set_baked(
        BodyId { origin: f, slot: 9 },
        Some(BakedBody {
            component: root,
            blob: blob("bodies/../evil"),
            geom: Some(BodyGeom::Mesh(Arc::default())),
        }),
    )
    .unwrap();
    let err = save(&p, &snap(d), &sample_opts()).unwrap_err();
    assert!(matches!(err, FormatError::InvalidBlobPath(_)), "{err:?}");
}

// ---- 3. migrations ----------------------------------------------------------------------

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/format")
        .join(name)
}

#[test]
fn migration_chain_covers_every_version() {
    assert_eq!(DOC_MIGRATIONS.len() as u32, FORMAT_VERSION);
}

#[test]
fn v0_fixture_migrates_to_v1() {
    let l = load(&fixture("v0.amg")).unwrap();
    assert_eq!(l.file_version, 0);
    assert_eq!(l.design.meta().name, "Fixture");
    assert_eq!(
        l.design.meta().default_units,
        amg_doc::UnitPrefs::default(),
        "meta.units became meta.default_units"
    );
    // Saving writes the current version.
    let dir = TempDir::new();
    let p = dir.path("v1.amg");
    save(&p, &snap(l.design), &SaveOptions::default()).unwrap();
    assert_eq!(load(&p).unwrap().file_version, FORMAT_VERSION);
}

#[test]
fn v1_fixture_loads_and_resaves_identically() {
    let l = load(&fixture("v1.amg")).unwrap();
    assert_eq!(l.file_version, 1);
    assert_eq!(l.design.meta().name, "Fixture");
    assert_eq!(l.design.timeline().len(), 1);
    let dir = TempDir::new();
    let p = dir.path("again.amg");
    save(&p, &snap(l.design), &SaveOptions::default()).unwrap();
    assert_eq!(
        entries_of(&p)["design.json"],
        entries_of(&fixture("v1.amg"))["design.json"]
    );
}

#[test]
fn migration_steps_run_in_order_and_failures_are_located() {
    use serde_json::{Value, json};
    fn a(mut v: Value) -> Result<Value, amg_doc::MigrationError> {
        v["a"] = 1.into();
        Ok(v)
    }
    fn b(v: Value) -> Result<Value, amg_doc::MigrationError> {
        assert_eq!(v["a"], 1);
        Err(amg_doc::MigrationError("boom".into()))
    }
    let chain: [amg_doc::io::DocMigration; 2] = [a, b];
    assert_eq!(
        migrate_document(&chain, 0, 1, json!({})).unwrap(),
        json!({ "a": 1 })
    );
    assert_eq!(migrate_document(&chain, 1, 1, json!(5)).unwrap(), json!(5));
    match migrate_document(&chain, 0, 2, json!({})) {
        Err(FormatError::Migration { from, to, msg }) => {
            assert_eq!((from, to, msg.as_str()), (1, 2, "boom"));
        }
        other => panic!("{other:?}"),
    }
    assert!(matches!(
        migrate_document(&chain, 2, 3, json!({})),
        Err(FormatError::Migration { from: 2, to: 3, .. })
    ));
}

/// Regenerates the fixtures. Fixtures are never edited once committed, so this refuses to
/// overwrite and exists only to document how they were made:
/// `AMG_WRITE_FIXTURES=1 cargo test -p amg-doc --test format -- --ignored regenerate`
#[test]
#[ignore = "documents how the fixtures were generated"]
fn regenerate_fixtures() {
    if std::env::var_os("AMG_WRITE_FIXTURES").is_none() {
        return;
    }
    amg_doc::testing::link();
    let mut d = Design::new("Fixture");
    let root = d.root();
    d.add_feature(
        None,
        FeatureBox::new(TestBox {
            size: [1.0, 2.0, 3.0],
        }),
        root,
        None,
    )
    .unwrap();
    let dir = TempDir::new();
    let tmp = dir.path("v1.amg");
    save(&tmp, &snap(d), &SaveOptions::default()).unwrap();
    let v1 = fixture("v1.amg");
    assert!(!v1.exists(), "fixtures are never overwritten");
    fs::create_dir_all(v1.parent().unwrap()).unwrap();
    fs::copy(&tmp, &v1).unwrap();
    // v0: meta.units instead of meta.default_units.
    let mut entries = entries_of(&tmp);
    let mut j: serde_json::Value = serde_json::from_slice(&entries["design.json"]).unwrap();
    let units = j["meta"]
        .as_object_mut()
        .unwrap()
        .remove("default_units")
        .unwrap();
    j["meta"]["units"] = units;
    entries.insert("design.json".into(), serde_json::to_vec_pretty(&j).unwrap());
    let v0 = fixture("v0.amg");
    assert!(!v0.exists(), "fixtures are never overwritten");
    write_container(&v0, 0, &entries, None);
}

// ---- 4. safe save -----------------------------------------------------------------------

#[test]
fn backup_is_the_previous_file() {
    let dir = TempDir::new();
    let p = dir.path("a.amg");
    save(&p, &snap(sample()), &sample_opts()).unwrap();
    assert!(
        !dir.path("a.amg.bak").exists(),
        "nothing to back up the first time"
    );
    assert!(!dir.path("a.amg.tmp").exists());
    let first = fs::read(&p).unwrap();
    let mut d = sample();
    d.set_name("Second");
    save(&p, &snap(d), &sample_opts()).unwrap();
    assert_eq!(fs::read(dir.path("a.amg.bak")).unwrap(), first);
    assert_eq!(load(&p).unwrap().design.meta().name, "Second");
    assert!(!dir.path("a.amg.tmp").exists());

    let q = dir.path("b.amg");
    let opts = SaveOptions {
        keep_backup: false,
        ..sample_opts()
    };
    save(&q, &snap(sample()), &opts).unwrap();
    save(&q, &snap(sample()), &opts).unwrap();
    assert!(!dir.path("b.amg.bak").exists());
}

#[test]
fn stopped_save_leaves_the_original_intact() {
    let dir = TempDir::new();
    let p = dir.path("a.amg");
    save(&p, &snap(sample()), &sample_opts()).unwrap();
    let original = fs::read(&p).unwrap();
    let mut changed = sample();
    changed.set_name("Changed");
    for phase in [
        SavePhase::TmpWritten,
        SavePhase::TmpSynced,
        SavePhase::BackupMade,
    ] {
        let opts = SaveOptions {
            stop_after: Some((phase, StopAction::Return)),
            ..sample_opts()
        };
        let err = save(&p, &snap(changed.clone()), &opts).unwrap_err();
        assert!(matches!(err, FormatError::Stopped(ph) if ph == phase));
        assert_eq!(fs::read(&p).unwrap(), original, "{phase:?}");
        assert_eq!(load(&p).unwrap().design.meta().name, "Bracket");
    }
    // A later normal save recovers from the leftovers.
    save(&p, &snap(changed), &sample_opts()).unwrap();
    assert_eq!(load(&p).unwrap().design.meta().name, "Changed");
    assert!(!dir.path("a.amg.tmp").exists());
}

/// Child half of [`killed_process_leaves_the_original_intact`]: aborts mid-save.
#[test]
fn kill_child() {
    let (Some(path), Some(phase)) = (
        std::env::var_os("AMG_KILL_PATH"),
        std::env::var_os("AMG_KILL_PHASE"),
    ) else {
        return; // not the child
    };
    let phase = match phase.to_str().unwrap() {
        "written" => SavePhase::TmpWritten,
        "synced" => SavePhase::TmpSynced,
        _ => SavePhase::BackupMade,
    };
    let mut d = sample();
    d.set_name("Dying");
    let opts = SaveOptions {
        stop_after: Some((phase, StopAction::Abort)),
        ..sample_opts()
    };
    save(Path::new(&path), &snap(d), &opts).unwrap();
    unreachable!("the process aborts inside save");
}

#[test]
fn killed_process_leaves_the_original_intact() {
    let dir = TempDir::new();
    let p = dir.path("a.amg");
    save(&p, &snap(sample()), &sample_opts()).unwrap();
    let original = fs::read(&p).unwrap();
    for phase in ["written", "synced", "backup"] {
        let status = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "kill_child", "--nocapture", "--test-threads=1"])
            .env("AMG_KILL_PATH", &p)
            .env("AMG_KILL_PHASE", phase)
            .status()
            .unwrap();
        assert!(!status.success(), "{phase}: the child must have died");
        assert_eq!(fs::read(&p).unwrap(), original, "{phase}");
        let l = load(&p).unwrap();
        assert_eq!(l.design.meta().name, "Bracket");
    }
}

// ---- 5. jobs ----------------------------------------------------------------------------

#[test]
fn save_and_load_run_as_jobs() {
    let dir = TempDir::new();
    let p = dir.path("a.amg");
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let h = amg_doc::save_job(&jobs, p.clone(), snap(sample()), sample_opts());
    h.wait_blocking().unwrap();
    let h = amg_doc::io::load_job(&jobs, p.clone(), Arc::new(StubBodyCodec));
    let l = h.wait_blocking().unwrap();
    assert_eq!(l.design.meta().name, "Bracket");
    // load_job for a bad file reports the specific error through the job.
    let bad = dir.path("bad.amg");
    fs::write(&bad, "nope").unwrap();
    let h = amg_doc::io::load_job(&jobs, bad, Arc::new(StubBodyCodec));
    match h.wait_blocking() {
        Err(amg_jobs::JobError::Failed(FormatError::NotAmalgam)) => {}
        other => panic!("{other:?}"),
    }
}

#[test]
fn cancelled_save_leaves_the_original_and_cancelled_load_stops() {
    let dir = TempDir::new();
    let p = dir.path("a.amg");
    save(&p, &snap(sample()), &sample_opts()).unwrap();
    let original = fs::read(&p).unwrap();
    let token = CancelToken::new();
    token.cancel();
    let ctx = JobCtx::with_token(token);
    let mut changed = sample();
    changed.set_name("Changed");
    let err = save_in_job(&p, &snap(changed), &sample_opts(), &ctx).unwrap_err();
    assert!(matches!(err, FormatError::Cancelled), "{err:?}");
    assert_eq!(fs::read(&p).unwrap(), original);
    assert!(!dir.path("a.amg.tmp").exists());
    let err = load_in_job(&p, &StubBodyCodec, &ctx).unwrap_err();
    assert!(matches!(err, FormatError::Cancelled), "{err:?}");
}

/// DOC-08: v1 baked bodies have no component; the v1 to v2 step puts them in the root.
#[test]
fn v1_baked_bodies_migrate_into_the_root_component() {
    let dir = TempDir::new();
    let src = dir.path("src.amg");
    save(&src, &snap(sample()), &sample_opts()).unwrap();
    let mut entries = entries_of(&src);
    let mut v: serde_json::Value = serde_json::from_slice(&entries["design.json"]).unwrap();
    let baked = v["baked"].as_array_mut().unwrap();
    assert_eq!(baked.len(), 1);
    baked[0][1]
        .as_object_mut()
        .unwrap()
        .remove("component")
        .unwrap();
    entries.insert("design.json".into(), serde_json::to_vec_pretty(&v).unwrap());
    let old = dir.path("v1-baked.amg");
    write_container(&old, 1, &entries, None);
    let l = load(&old).unwrap();
    assert_eq!(l.file_version, 1);
    let (_, b) = l.design.baked_bodies().next().unwrap();
    assert_eq!(b.component, l.design.root());
}
