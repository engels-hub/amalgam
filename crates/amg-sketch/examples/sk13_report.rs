//! SK-13 report generator: every note §5.3 metric for every option on D1–D10, plus a
//! whole-corpus run per option (for the "> 2 % corpus gap" rule of ADR-007).
//!
//! ```text
//! cargo run --release -p amg-sketch --example sk13_report [--features slvs,ezpz-bench] -- [runs] [out-dir]
//! ```
//! Prints Markdown; with `out-dir`, also writes `sk13_decision.csv` and `sk13_corpus.csv` there.

#![allow(clippy::print_stdout, clippy::unwrap_used, reason = "report tool")]

use std::collections::BTreeMap;
use std::path::PathBuf;

use amg_sketch::corpus::compare::{self, ContenderBackend, metrics};
use amg_sketch::corpus::{self, Case, Verdict, run_corpus};

fn decision_number(c: &Case) -> Option<u32> {
    c.tags
        .iter()
        .find_map(|t| t.strip_prefix("decision:D"))
        .and_then(|n| n.parse().ok())
}

fn main() {
    let mut args = std::env::args().skip(1);
    let runs: usize = args.next().and_then(|a| a.parse().ok()).unwrap_or(200);
    let out_dir = args.next().map(PathBuf::from);
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/sketch/corpus");
    let loaded = corpus::load_corpus(&root).unwrap();
    let mut decision: Vec<&Case> = loaded
        .cases
        .iter()
        .filter(|c| decision_number(c).is_some())
        .collect();
    decision.sort_by_key(|c| decision_number(c));

    let mut rows = Vec::new();
    for case in &decision {
        for c in compare::contenders() {
            rows.push(metrics::measure(c.as_ref(), case, runs, 3));
        }
    }
    println!("## Decision cases D1–D10 ({runs} runs each)\n");
    println!("{}", metrics::markdown(&rows));

    let mut summary: BTreeMap<String, usize> = BTreeMap::new();
    for r in &rows {
        if r.passed() {
            *summary.entry(r.contender.clone()).or_default() += 1;
        }
    }
    println!("Decision cases solved: {summary:?}\n");

    println!("## Whole corpus ({} cases)\n", loaded.cases.len());
    println!("| Option | Pass | Fail | Skipped (unsupported) | Pass rate of run cases |");
    println!("|---|---|---|---|---|");
    let mut corpus_csv = String::from("case,option,verdict,detail\n");
    let mut verdicts: BTreeMap<String, BTreeMap<String, Verdict>> = BTreeMap::new();
    for c in compare::contenders() {
        let mut backend = ContenderBackend(c);
        let report = run_corpus(&mut backend, &loaded.cases);
        println!(
            "| {} | {} | {} | {} | {:.1} % |",
            report.backend,
            report.passed(),
            report.failures().len(),
            report.skipped(),
            100.0 * report.pass_rate()
        );
        for r in &report.results {
            let (v, d) = match &r.verdict {
                Verdict::Pass => ("pass", String::new()),
                Verdict::Fail(w) => ("fail", w.join("; ")),
                Verdict::Skipped(w) => ("skip", w.clone()),
            };
            corpus_csv.push_str(&format!(
                "{},{},{v},\"{}\"\n",
                r.id,
                report.backend,
                d.replace('"', "'")
            ));
            verdicts
                .entry(report.backend.clone())
                .or_default()
                .insert(r.id.clone(), r.verdict.clone());
        }
    }
    println!();
    if let (Some(a), Some(b)) = (verdicts.get("native"), verdicts.get("libslvs")) {
        let gap: Vec<&String> = b
            .iter()
            .filter(|(id, v)| **v == Verdict::Pass && a.get(*id) != Some(&Verdict::Pass))
            .map(|(id, _)| id)
            .collect();
        let rev: Vec<&String> = a
            .iter()
            .filter(|(id, v)| **v == Verdict::Pass && b.get(*id) != Some(&Verdict::Pass))
            .map(|(id, _)| id)
            .collect();
        println!(
            "libslvs passes, native does not: {} of {} cases {gap:?}\n",
            gap.len(),
            a.len()
        );
        println!("native passes, libslvs does not: {} cases\n", rev.len());
    }
    for (name, v) in &verdicts {
        let fails: Vec<String> = v
            .iter()
            .filter_map(|(id, v)| match v {
                Verdict::Fail(w) => Some(format!("{id}: {}", w.join("; "))),
                _ => None,
            })
            .collect();
        if !fails.is_empty() {
            println!("### {name} failures\n");
            for f in fails {
                println!("- {f}");
            }
            println!();
        }
    }
    if let Some(dir) = out_dir {
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("sk13_decision.csv"), metrics::csv(&rows)).unwrap();
        std::fs::write(dir.join("sk13_corpus.csv"), corpus_csv).unwrap();
    }
}
