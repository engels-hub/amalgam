//! DOC-14: geometry-cache persistence in the `.amg` container, lazy seeding, salt check.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use amg_doc::io::CACHE_INDEX;
use amg_doc::testing::{TestCounter, compute_count};
use amg_doc::{
    BodyId, CacheKey, Dependency, Design, Document, FeatureBox, FeatureId, GeomCache, SaveOptions,
    engine_salt, load, save,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, ZipArchive, ZipWriter};

struct TempDir(PathBuf);

impl TempDir {
    fn new() -> TempDir {
        static N: AtomicU32 = AtomicU32::new(0);
        let p = std::env::temp_dir().join(format!(
            "amg-doc-cache-{}-{}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&p).unwrap();
        TempDir(p)
    }
    fn path(&self, n: &str) -> PathBuf {
        self.0.join(n)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

/// `n` chained `test.counter` features (each reads the previous one's body).
fn chain(n: usize) -> (Design, Vec<FeatureId>) {
    let mut d = Design::new("Chain");
    let root = d.root();
    let mut ids: Vec<FeatureId> = Vec::new();
    for i in 0..n {
        let deps = ids
            .last()
            .map(|f| {
                vec![Dependency::Body(BodyId {
                    origin: *f,
                    slot: 0,
                })]
            })
            .unwrap_or_default();
        let f = FeatureBox::new(TestCounter {
            deps,
            value: i as i64,
            fail: false,
        });
        ids.push(d.add_feature(None, f, root, None).unwrap());
    }
    (d, ids)
}

fn counts(d: &Design, ids: &[FeatureId]) -> Vec<u64> {
    ids.iter()
        .map(|f| compute_count(d.meta().uuid, *f))
        .collect()
}

/// A saved 50-feature design with its cache; returns the design (for counters) and the path.
fn saved_chain(tmp: &TempDir) -> (Design, Vec<FeatureId>, PathBuf) {
    let (d, ids) = chain(50);
    let mut doc = Document::from_design(d.clone());
    doc.recompute_blocking(&jobs()).unwrap();
    assert_eq!(doc.last_run().unwrap().computed.len(), 50);
    let p = tmp.path("a.amg");
    let opts = SaveOptions {
        cache: Some(doc.cache_export()),
        ..SaveOptions::default()
    };
    save(&p, &doc.snapshot(), &opts).unwrap();
    (d, ids, p)
}

fn entry_names(p: &Path) -> Vec<String> {
    let mut z = ZipArchive::new(File::open(p).unwrap()).unwrap();
    (0..z.len())
        .map(|i| z.by_index(i).unwrap().name().unwrap().into_owned())
        .collect()
}

/// Rewrites the zip with `f(name, bytes)` applied to every entry (the manifest keeps the old
/// hashes, so a changed entry is damaged as far as the loader can tell).
fn rewrite(p: &Path, f: impl Fn(&str, Vec<u8>) -> Vec<u8>) {
    let mut z = ZipArchive::new(File::open(p).unwrap()).unwrap();
    let mut entries = Vec::new();
    for i in 0..z.len() {
        let mut e = z.by_index(i).unwrap();
        let mut data = Vec::new();
        e.read_to_end(&mut data).unwrap();
        entries.push((e.name().unwrap().into_owned(), data));
    }
    drop(z);
    let mut w = ZipWriter::new(File::create(p).unwrap());
    for (name, data) in entries {
        let data = f(&name, data);
        w.start_file(
            &name,
            SimpleFileOptions::default().compression_method(CompressionMethod::Deflated),
        )
        .unwrap();
        w.write_all(&data).unwrap();
    }
    w.finish().unwrap();
}

// ---- Done-when 1: open with cache computes nothing; another salt computes everything ---------

#[test]
fn opening_a_saved_design_with_cache_computes_nothing() {
    let tmp = TempDir::new();
    let (d, ids, p) = saved_chain(&tmp);
    let before = counts(&d, &ids);
    assert!(before.iter().all(|c| *c == 1));

    let loaded = load(&p).unwrap();
    assert_eq!(loaded.cache_seed.len(), 50);
    assert!(loaded.warnings.is_empty());
    let mut doc = loaded.into_document();
    // Lazy: nothing is read at open.
    assert_eq!(doc.cache().stats().entries, 0);
    doc.recompute_blocking(&jobs()).unwrap();

    let run = doc.last_run().unwrap();
    assert!(run.computed.is_empty(), "computed {:?}", run.computed);
    assert_eq!(run.cached.len(), 50);
    assert_eq!(counts(&d, &ids), before, "feature compute counters");
    assert_eq!(
        doc.cache().stats().entries,
        50,
        "entries loaded on first hit"
    );
    assert_eq!(doc.cache().stats().misses, 0);

    // The evaluated state equals a fresh computation.
    let mut fresh = Document::from_design(d.clone());
    fresh.recompute_blocking(&jobs()).unwrap();
    for id in &ids {
        assert_eq!(
            doc.evaluated().node(*id).unwrap().key,
            fresh.evaluated().node(*id).unwrap().key
        );
        let b = BodyId {
            origin: *id,
            slot: 0,
        };
        assert_eq!(
            std::mem::discriminant(&doc.evaluated().body(b).unwrap().geom),
            std::mem::discriminant(&fresh.evaluated().body(b).unwrap().geom)
        );
    }
}

#[test]
fn a_changed_salt_recomputes_everything() {
    let tmp = TempDir::new();
    let (d, ids, p) = saved_chain(&tmp);
    let before = counts(&d, &ids);

    let loaded = load(&p).unwrap();
    let salt = engine_salt(&loaded.design.meta().tolerance);
    assert_eq!(loaded.cache_seed.stored_salt(), Some(salt));
    let cache = GeomCache::new(1 << 30);
    let mut other = salt;
    other[0] ^= 1;
    assert!(!loaded.cache_seed.attach_to(&cache, &other), "refused");
    let mut doc = Document::with_cache(loaded.design, cache);
    doc.recompute_blocking(&jobs()).unwrap();
    assert_eq!(doc.last_run().unwrap().computed.len(), 50);
    let after = counts(&d, &ids);
    assert!(after.iter().zip(&before).all(|(a, b)| *a == b + 1));
}

// ---- save details -----------------------------------------------------------------------

#[test]
fn save_compact_omits_the_cache() {
    let tmp = TempDir::new();
    let (d, ids) = chain(5);
    let mut doc = Document::from_design(d.clone());
    doc.recompute_blocking(&jobs()).unwrap();
    let full = tmp.path("full.amg");
    let compact = tmp.path("compact.amg");
    let with = SaveOptions {
        cache: Some(doc.cache_export()),
        ..SaveOptions::default()
    };
    save(&full, &doc.snapshot(), &with).unwrap();
    save(&compact, &doc.snapshot(), &SaveOptions::default()).unwrap();
    assert!(entry_names(&full).iter().any(|n| n == CACHE_INDEX));
    assert_eq!(
        entry_names(&full)
            .iter()
            .filter(|n| n.starts_with("cache/") && n.ends_with(".out"))
            .count(),
        5
    );
    assert!(
        entry_names(&compact)
            .iter()
            .all(|n| !n.starts_with("cache/"))
    );
    assert!(fs::metadata(&compact).unwrap().len() < fs::metadata(&full).unwrap().len());

    // A compact file opens fine and recomputes.
    let l = load(&compact).unwrap();
    assert!(l.cache_seed.is_empty());
    let before = counts(&d, &ids);
    let mut doc2 = l.into_document();
    doc2.recompute_blocking(&jobs()).unwrap();
    assert_eq!(doc2.last_run().unwrap().computed.len(), 5);
    assert!(
        counts(&d, &ids)
            .iter()
            .zip(&before)
            .all(|(a, b)| *a == b + 1)
    );
}

#[test]
fn only_outputs_of_the_current_timeline_are_written_and_saves_are_deterministic() {
    let tmp = TempDir::new();
    let (d, _) = chain(3);
    let mut doc = Document::from_design(d);
    doc.recompute_blocking(&jobs()).unwrap();
    // Leftovers of earlier edits stay in the LRU but are not part of the timeline.
    let junk = doc
        .cache()
        .get(&doc.evaluated().nodes().next().unwrap().1.key.unwrap());
    doc.cache().insert(CacheKey([7; 32]), junk.unwrap());
    let opts = SaveOptions {
        cache: Some(doc.cache_export()),
        ..SaveOptions::default()
    };
    let (a, b) = (tmp.path("a.amg"), tmp.path("b.amg"));
    save(&a, &doc.snapshot(), &opts).unwrap();
    save(&b, &doc.snapshot(), &opts).unwrap();
    assert_eq!(
        entry_names(&a)
            .iter()
            .filter(|n| n.ends_with(".out"))
            .count(),
        3
    );
    assert_eq!(fs::read(&a).unwrap(), fs::read(&b).unwrap());
}

#[test]
fn a_lazily_opened_file_can_be_saved_again_with_its_cache() {
    let tmp = TempDir::new();
    let (_, _, p) = saved_chain(&tmp);
    let mut doc = load(&p).unwrap().into_document();
    doc.recompute_blocking(&jobs()).unwrap();
    let q = tmp.path("copy.amg");
    let opts = SaveOptions {
        cache: Some(doc.cache_export()),
        ..SaveOptions::default()
    };
    save(&q, &doc.snapshot(), &opts).unwrap();
    assert_eq!(load(&q).unwrap().cache_seed.len(), 50);

    // Saving without first recomputing (nothing resident) still carries entries over: they
    // are read from the backing file on demand.
    let doc3 = load(&q).unwrap().into_document();
    let keys = doc.cache_export().keys;
    let export = amg_doc::io::CacheExport::new(Arc::clone(doc3.cache()), keys);
    let r = tmp.path("again.amg");
    let opts = SaveOptions {
        cache: Some(export),
        ..SaveOptions::default()
    };
    save(&r, &doc3.snapshot(), &opts).unwrap();
    assert_eq!(load(&r).unwrap().cache_seed.len(), 50);
}

// ---- damaged caches never fail a load ---------------------------------------------------------

#[test]
fn a_damaged_cache_entry_recomputes_just_that_feature() {
    let tmp = TempDir::new();
    let (d, ids, p) = saved_chain(&tmp);
    let before = counts(&d, &ids);
    let victim = entry_names(&p)
        .into_iter()
        .find(|n| n.ends_with(".out"))
        .unwrap();
    rewrite(&p, |n, mut data| {
        if n == victim {
            data[0] ^= 0xff;
        }
        data
    });
    let loaded = load(&p).unwrap(); // lazily verified: the load itself is fine
    let mut doc = loaded.into_document();
    doc.recompute_blocking(&jobs()).unwrap();
    assert_eq!(doc.last_run().unwrap().computed.len(), 1);
    let after = counts(&d, &ids);
    assert_eq!(after.iter().zip(&before).filter(|(a, b)| a != b).count(), 1);
}

#[test]
fn a_damaged_index_gives_an_empty_seed_and_a_warning() {
    let tmp = TempDir::new();
    let (_, _, p) = saved_chain(&tmp);
    rewrite(&p, |n, data| {
        if n == CACHE_INDEX {
            b"{not json".to_vec()
        } else {
            data
        }
    });
    let l = load(&p).unwrap();
    assert!(l.cache_seed.is_empty());
    assert!(l.warnings.iter().any(|w| w.contains("geometry cache")));
    let mut doc = l.into_document();
    doc.recompute_blocking(&jobs()).unwrap();
    assert_eq!(doc.last_run().unwrap().computed.len(), 50);
}
