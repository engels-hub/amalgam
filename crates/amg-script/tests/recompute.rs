//! SCR-07 Done-when 1 (recompute reports on three fixture designs) and 6 (cancellation), and
//! the `ScriptDoc` behaviour around them.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    reason = "test helpers; a failure is a test failure"
)]

mod common;

use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use amg_doc::testing::TestSlow;
use amg_doc::{
    BlobRef, Command, DesignMode, Expr, ExtRecord, FeatureBox, FormatError, SaveOptions,
};
use amg_jobs::CancelToken;
use amg_script::ScriptError;
use amg_script::host::ScriptDoc;
use amg_script::ops::{BodyFilter, ExportRequest, RecomputeReport, export, properties, recompute};

fn healths(r: &RecomputeReport) -> Vec<&'static str> {
    r.features.iter().map(|f| f.health).collect()
}

/// Opens the compact file, recomputes (cold), saves with the cache and reopens (warm).
/// Returns the cold report, the warm report and the reopened document.
fn cold_and_warm(
    doc: &amg_doc::Document,
    name: &str,
    dir: &std::path::Path,
) -> (RecomputeReport, RecomputeReport, ScriptDoc) {
    let jobs = common::jobs();
    let cancel = CancelToken::new();
    let path = common::save(doc, dir, name);
    let mut cold = ScriptDoc::open(&path, Arc::clone(&jobs)).unwrap();
    assert!(cold.warnings().is_empty(), "{:?}", cold.warnings());
    let first = recompute(&mut cold, &cancel).unwrap();
    let opts = SaveOptions {
        cache: Some(cold.document().cache_export()),
        ..SaveOptions::default()
    };
    let cached_path = dir.join(format!("{name}-cached.amg"));
    assert_eq!(cold.save(Some(&cached_path), &opts).unwrap(), cached_path);
    assert_eq!(cold.path(), Some(cached_path.as_path()));
    assert!(!cold.document().is_dirty());
    let mut warm = ScriptDoc::open(&cached_path, jobs).unwrap();
    let second = recompute(&mut warm, &cancel).unwrap();
    for (label, r) in [("cold", &first), ("warm", &second)] {
        eprintln!(
            "[recompute] {name} {label}: computed {} cached {} health {:?} {:.2} ms",
            r.computed,
            r.cached,
            healths(r),
            r.millis
        );
    }
    (first, second, warm)
}

#[test]
fn mesh_only_design_computes_then_comes_from_the_cache() {
    let dir = tempfile::tempdir().unwrap();
    let (cold, warm, _) = cold_and_warm(&common::mesh_only(), "mesh-only", dir.path());
    assert_eq!((cold.computed, cold.cached), (2, 0));
    assert_eq!((warm.computed, warm.cached), (0, 2));
    for r in [&cold, &warm] {
        assert_eq!(r.schema, 1);
        assert_eq!(healths(r), ["ok", "ok"]);
        assert!(!r.has_errors());
        assert_eq!(r.features[0].kind, "io.import_mesh");
        assert_eq!(r.features[0].name, "Insert cube.stl");
    }
    assert!(warm.features.iter().all(|f| f.millis == 0.0));
}

#[test]
fn mixed_design_recomputes_only_what_an_edit_touches() {
    let dir = tempfile::tempdir().unwrap();
    let (doc, test, cube) = common::mixed();
    let (cold, warm, mut doc) = cold_and_warm(&doc, "mixed", dir.path());
    assert_eq!((cold.computed, cold.cached), (3, 0));
    assert_eq!((warm.computed, warm.cached), (0, 3));
    assert_eq!(healths(&warm), ["ok", "ok", "ok"]);
    assert!(!warm.has_errors());
    assert_eq!(warm.features[2].kind, "test.counter");
    assert_eq!(warm.features[2].name, "probe");

    let cancel = CancelToken::new();
    // Asking again does nothing: the report still describes the pass behind the state.
    let again = recompute(&mut doc, &cancel).unwrap();
    assert_eq!((again.computed, again.cached), (0, 3));
    assert_eq!(again.revision, warm.revision);

    doc.apply(
        "Edit probe",
        vec![Command::EditFeature {
            id: test,
            feature: common::counter(cube, 2, false),
        }],
    )
    .unwrap();
    let edited = recompute(&mut doc, &cancel).unwrap();
    eprintln!(
        "[recompute] mixed after editing the test feature: computed {} cached {}",
        edited.computed, edited.cached
    );
    assert_eq!((edited.computed, edited.cached), (1, 2));
    assert!(edited.revision > warm.revision);

    doc.apply(
        "Suppress",
        vec![Command::SetSuppressed {
            ids: vec![test],
            suppressed: true,
        }],
    )
    .unwrap();
    let off = recompute(&mut doc, &cancel).unwrap();
    assert_eq!(healths(&off), ["ok", "ok", "suppressed"]);
    assert_eq!((off.computed, off.cached), (0, 2));
    doc.apply("Roll back", vec![Command::SetMarker { index: 1 }])
        .unwrap();
    let back = recompute(&mut doc, &cancel).unwrap();
    assert_eq!(healths(&back), ["ok", "rolled_back", "rolled_back"]);
}

#[test]
fn failing_feature_is_reported_with_its_message() {
    let dir = tempfile::tempdir().unwrap();
    let (doc, bad) = common::failing();
    let (cold, warm, _) = cold_and_warm(&doc, "failing", dir.path());
    // A failed feature is in neither count and is never cached.
    assert_eq!((cold.computed, cold.cached), (2, 0));
    assert_eq!((warm.computed, warm.cached), (0, 2));
    for r in [&cold, &warm] {
        assert!(r.has_errors());
        assert_eq!(healths(r), ["ok", "ok", "error"]);
        let row = &r.features[2];
        assert_eq!(row.id, bad);
        assert_eq!(row.name, "broken");
        assert_eq!(row.messages.len(), 1);
        assert!(
            row.messages[0].contains("asked to fail"),
            "{:?}",
            row.messages
        );
    }
}

#[test]
fn only_the_failing_fixture_has_errors() {
    let cancel = CancelToken::new();
    let dir = tempfile::tempdir().unwrap();
    let fixtures = [
        (common::mesh_only(), "a"),
        (common::mixed().0, "b"),
        (common::failing().0, "c"),
    ];
    let errors: Vec<bool> = fixtures
        .iter()
        .map(|(d, name)| {
            let path = common::save(d, dir.path(), name);
            let mut doc = ScriptDoc::open(&path, common::jobs()).unwrap();
            recompute(&mut doc, &cancel).unwrap().has_errors()
        })
        .collect();
    assert_eq!(errors, [false, false, true]);
}

fn slow_feature(ms: &str) -> FeatureBox {
    FeatureBox::new(TestSlow {
        ms: Expr::Inline(ms.into()),
        tag: 0,
        panic: false,
    })
}

/// Cancels `op` 100 ms after it starts and returns how long it took to return afterwards.
fn cancel_latency<T: std::fmt::Debug>(
    doc: &mut ScriptDoc,
    op: impl FnOnce(&mut ScriptDoc, &CancelToken) -> Result<T, ScriptError>,
) -> Duration {
    let cancel = CancelToken::new();
    let remote = cancel.clone();
    let canceller = thread::spawn(move || {
        thread::sleep(Duration::from_millis(100));
        let at = Instant::now();
        remote.cancel();
        at
    });
    let result = op(doc, &cancel);
    let returned = Instant::now();
    let cancelled_at = canceller.join().unwrap();
    assert!(matches!(result, Err(ScriptError::Cancelled)), "{result:?}");
    returned.duration_since(cancelled_at)
}

#[test]
fn cancelling_a_slow_feature_returns_within_100_ms() {
    amg_script::link().unwrap();
    let dir = tempfile::tempdir().unwrap();
    let mut doc = ScriptDoc::new("slow", common::jobs());
    let root = doc.document().design().root();
    // A minute: the test can only pass through cancellation.
    doc.apply(
        "Add slow",
        vec![common::add_command(root, slow_feature("60000"), "slow")],
    )
    .unwrap();

    let out = dir.path().join("slow.stl");
    let times = [
        ("recompute", cancel_latency(&mut doc, recompute)),
        ("evaluated", cancel_latency(&mut doc, |d, c| d.evaluated(c))),
        (
            "export",
            cancel_latency(&mut doc, |d, c| export(d, &ExportRequest::new(&out), c)),
        ),
        (
            "properties",
            cancel_latency(&mut doc, |d, c| properties(d, &BodyFilter::AllVisible, c)),
        ),
    ];
    for (name, t) in times {
        eprintln!("[cancel] {name}: returned {t:?} after cancel");
        assert!(t < Duration::from_millis(100), "{name} took {t:?}");
    }
    assert!(!out.exists(), "a cancelled export leaves no file");

    // An already cancelled token does not start the work.
    let cancelled = CancelToken::new();
    cancelled.cancel();
    let started = Instant::now();
    assert!(matches!(
        doc.evaluated(&cancelled),
        Err(ScriptError::Cancelled)
    ));
    assert!(started.elapsed() < Duration::from_millis(100));

    // The document stays usable.
    assert!(!doc.document().is_evaluated());
    assert!(doc.undo());
    let r = recompute(&mut doc, &CancelToken::new()).unwrap();
    assert!(r.features.is_empty() && !r.has_errors());
    assert!(doc.redo());
    assert_eq!(doc.document().undo_label(), Some("Add slow"));
}

#[test]
fn apply_is_one_atomic_undo_entry() {
    let mut doc = ScriptDoc::new("params", common::jobs());
    let add = |name: &str, expr: &str| Command::AddParam {
        name: name.to_owned(),
        expr: expr.to_owned(),
        unit: None,
        comment: String::new(),
    };
    let out = doc
        .apply("Two params", vec![add("a", "1 mm"), add("b", "a * 2")])
        .unwrap();
    assert_eq!(out.len(), 2);
    assert!(out.iter().all(|o| o.created.len() == 1));
    assert!(
        out.iter()
            .all(|o| o.revision == doc.document().history().revision())
    );
    assert_eq!(doc.document().undo_label(), Some("Two params"));

    let before = doc.document().history().revision();
    let err = doc
        .apply("Bad", vec![add("c", "1 mm"), add("a", "1 mm")])
        .unwrap_err();
    assert!(matches!(err, ScriptError::Command(_)), "{err:?}");
    assert_eq!(doc.document().history().revision(), before);
    assert_eq!(doc.document().design().params().len(), 2);

    assert!(doc.undo());
    assert_eq!(doc.document().design().params().len(), 0);
    assert!(!doc.undo());
    assert!(doc.redo());
    assert!(!doc.redo());
}

#[test]
fn switching_to_direct_mode_recomputes_first() {
    let dir = tempfile::tempdir().unwrap();
    let path = common::save(&common::mesh_only(), dir.path(), "direct");
    let mut doc = ScriptDoc::open(&path, common::jobs()).unwrap();
    assert!(!doc.document().is_evaluated());
    doc.apply(
        "Direct",
        vec![Command::SetDesignMode {
            mode: DesignMode::Direct,
        }],
    )
    .unwrap();
    assert_eq!(doc.document().design().meta().mode, DesignMode::Direct);
    let r = recompute(&mut doc, &CancelToken::new()).unwrap();
    assert!(r.features.is_empty());
    let p = properties(&mut doc, &BodyFilter::AllVisible, &CancelToken::new()).unwrap();
    assert_eq!(p.bodies.len(), 2);
}

#[test]
fn save_needs_a_path_and_keeps_extension_blobs() {
    let dir = tempfile::tempdir().unwrap();
    let mut fresh = ScriptDoc::new("new", common::jobs());
    let err = fresh.save(None, &SaveOptions::default()).unwrap_err();
    assert!(matches!(err, ScriptError::Field { .. }), "{err:?}");

    let blob = BlobRef("ext/test/payload.bin".to_owned());
    fresh
        .apply(
            "Record",
            vec![Command::AddExt {
                record: ExtRecord {
                    kind: "test.blob".to_owned(),
                    version: 1,
                    data: serde_json::json!({}),
                    blobs: vec![blob.clone()],
                },
            }],
        )
        .unwrap();
    let first = dir.path().join("first.amg");
    let mut opts = SaveOptions::default();
    opts.blobs.insert(blob.clone(), Arc::from(&b"payload"[..]));
    opts.thumbnail = Some(b"png".to_vec());
    fresh.save(Some(&first), &opts).unwrap();

    // Reopen and save without providing anything: the blob and thumbnail survive.
    let mut opened = ScriptDoc::open(&first, common::jobs()).unwrap();
    assert_eq!(opened.path(), Some(first.as_path()));
    assert_eq!(opened.save(None, &SaveOptions::default()).unwrap(), first);
    let second = dir.path().join("second.amg");
    opened.save(Some(&second), &SaveOptions::default()).unwrap();
    let loaded = amg_doc::load(&second).unwrap();
    assert_eq!(
        loaded.blobs.get(&blob).map(|b| &b[..]),
        Some(&b"payload"[..])
    );
    assert_eq!(loaded.thumbnail.as_deref(), Some(&b"png"[..]));
}

#[test]
fn missing_and_corrupt_files_give_format_errors() {
    let dir = tempfile::tempdir().unwrap();
    let missing = ScriptDoc::open(&dir.path().join("no.amg"), common::jobs()).unwrap_err();
    assert!(
        matches!(missing, ScriptError::Format(FormatError::Io(_))),
        "{missing:?}"
    );
    let bad = dir.path().join("bad.amg");
    std::fs::write(&bad, b"not a zip").unwrap();
    let corrupt = ScriptDoc::open(&bad, common::jobs()).unwrap_err();
    assert!(
        matches!(corrupt, ScriptError::Format(FormatError::NotAmalgam)),
        "{corrupt:?}"
    );
}

#[test]
fn link_registers_the_feature_kinds() {
    amg_script::link().unwrap();
    let registry = amg_doc::registry();
    for kind in [
        "io.import_mesh",
        "mesh.repair",
        "solid.extrude",
        "sketch.sketch",
        "construct.plane",
    ] {
        assert!(
            registry.get(kind).is_some(),
            "{kind}: {:?}",
            registry.kinds().collect::<Vec<_>>()
        );
    }
}
