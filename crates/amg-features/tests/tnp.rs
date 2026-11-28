//! DOC-13: the full golden TNP corpus (`tests/golden/tnp/`, persistent-naming note §11, 30
//! core cases) on the S04 solid features, CI-gated.
//!
//! Every case runs: the DOC-13 cases with their sketch/extrude/fillet macros and the S04
//! features they name, the DOC-06 cases with the lane-E test features mapped to S04 features
//! (see `tnp_common`). The run checks each `expect`, classifies it (expect met / feature error
//! / warning / silent wrong), measures the naive index baseline and writes
//! `<target>/tmp/tnp-doc13-report.json`. Done-when (S01 DOC-13): ≥ 90 % of the cases meet their
//! `expect`, the others fail with a feature error or a warning, 0 silent wrong, ≥ 5 cases
//! measured to break the naive baseline, < 60 s.
//!
//! Needs OCCT: `cargo nextest run -p amg-features --features occt --test tnp`. Without it only
//! the corpus files are parsed.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    reason = "test harness"
)]

mod tnp_common;

use serde_json::json;
use tnp_common::{cases, corpus_dir, merge_patch};

/// Feature kinds a corpus case may name (S04 final schemas, lane-E test features).
const KINDS: [&str; 10] = [
    "solid.pattern",
    "solid.mirror",
    "solid.combine",
    "solid.split_body",
    "solid.revolve",
    "test.brep_box",
    "test.brep_prism",
    "test.brep_fillet",
    "test.probe",
    "sketch.sketch",
];

fn kinds_in(v: &serde_json::Value, out: &mut Vec<String>) {
    match v {
        serde_json::Value::Object(m) => {
            if let (Some(k), Some(_)) = (m.get("kind").and_then(|k| k.as_str()), m.get("data")) {
                out.push(k.to_owned());
            }
            m.values().for_each(|x| kinds_in(x, out));
        }
        serde_json::Value::Array(a) => a.iter().for_each(|x| kinds_in(x, out)),
        _ => {}
    }
}

#[test]
fn corpus_files_parse_and_name_known_features() {
    let all = cases();
    assert_eq!(all.len(), 30, "30 core cases (note §11.4)");
    for c in &all {
        for key in ["id", "phase", "edit", "expect"] {
            assert!(c.get(key).is_some(), "{}: missing {key}", c["id"]);
        }
        let mut kinds = Vec::new();
        kinds_in(c, &mut kinds);
        for s in tnp_common::base_steps(c) {
            kinds_in(&s, &mut kinds);
        }
        for k in kinds {
            assert!(KINDS.contains(&k.as_str()), "{}: unknown kind {k}", c["id"]);
        }
    }
    assert!(corpus_dir().join("README.md").exists());
}

#[test]
fn merge_patch_follows_rfc_7396() {
    let mut a = json!({"a": 1, "b": {"c": 2, "d": 3}});
    merge_patch(&mut a, &json!({"b": {"c": null, "e": 4}, "f": [1]}));
    assert_eq!(a, json!({"a": 1, "b": {"d": 3, "e": 4}, "f": [1]}));
}

#[cfg(feature = "occt")]
#[test]
fn doc13_corpus_on_s04_features_meets_its_expectations_with_no_silent_wrong() {
    use std::panic::{AssertUnwindSafe, catch_unwind};
    use std::path::PathBuf;

    let tmp = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("tnp-doc13");
    std::fs::create_dir_all(&tmp).unwrap();
    let started_all = std::time::Instant::now();
    let mut report = Vec::new();
    let mut met = 0;
    let mut not_met = Vec::new();
    let mut silent = Vec::new();
    let mut harness = Vec::new();
    let mut breaks = Vec::new();
    let mut wrong_predictions = Vec::new();
    let all = cases();
    for case in &all {
        let id = case["id"].as_str().unwrap().to_owned();
        let started = std::time::Instant::now();
        let r = catch_unwind(AssertUnwindSafe(|| tnp_common::run_case(case, &tmp)));
        let ms = started.elapsed().as_millis();
        let Ok(r) = r else {
            println!("{id}: HARNESS ERROR [{ms} ms] (panic above)");
            harness.push(id.clone());
            report.push(json!({"id": id, "outcome": "harness error", "ms": ms}));
            continue;
        };
        println!("{id}: {} [{ms} ms] {}", r.outcome(), r.text);
        if r.ok() {
            met += 1;
        } else {
            not_met.push(id.clone());
        }
        if r.silent_wrong() {
            silent.push(id.clone());
        }
        if r.naive == Some(true) {
            breaks.push(id.clone());
        }
        let predicted = &case["naive_index_breaks"];
        if let Some(m) = r.naive
            && predicted.as_bool() != Some(m)
        {
            println!("{id}: naive baseline measured {m}, predicted {predicted} (correct the case)");
            wrong_predictions.push(id.clone());
        }
        report.push(json!({
            "id": id, "phase": case["phase"], "outcome": r.outcome(), "result": r.text,
            "naive_index_breaks_measured": r.naive, "naive_index_breaks_predicted": predicted,
            "ms": ms,
        }));
    }
    let secs = started_all.elapsed().as_secs_f64();
    let path = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("tnp-doc13-report.json");
    std::fs::write(&path, serde_json::to_string_pretty(&report).unwrap()).unwrap();
    println!(
        "DOC-13: {} cases, {met} meet their expect ({:.0} %), not met {not_met:?}, silent wrong \
         {silent:?}, harness errors {harness:?}, {} measured to break the naive baseline \
         {breaks:?}, wrong predictions {wrong_predictions:?}; {secs:.1} s; report {}",
        all.len(),
        100.0 * f64::from(met) / all.len() as f64,
        breaks.len(),
        path.display()
    );
    assert!(silent.is_empty(), "silent wrong resolutions: {silent:?}");
    assert!(harness.is_empty(), "harness errors: {harness:?}");
    assert!(
        f64::from(met) >= 0.9 * all.len() as f64,
        "fewer than 90 % of the cases meet their expect: {not_met:?}"
    );
    assert!(
        breaks.len() >= 5,
        "only {} cases break the naive baseline",
        breaks.len()
    );
    assert!(
        wrong_predictions.is_empty(),
        "naive predictions to correct: {wrong_predictions:?}"
    );
    assert!(secs < 60.0, "corpus took {secs:.1} s (budget 60 s)");
}
