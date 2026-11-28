//! DOC-13: the held-out TNP fuzz set (persistent-naming note §11.6) on the S04 features.
//!
//! 200 seeded runs. Each builds base B1 (box 100 × 60 × 40) or B2 (the same rectangle as a
//! tagged polygon prism), on the S04 features through the corpus harness (`tnp_common`), with a
//! fillet on the top edge through a random point P, then applies one random upstream edit: a
//! dimension edit (±50 % per axis), a notch inserted at a random position on the picked edge,
//! a groove inserted across the top face, a polygon vertex added or removed (B2), a groove
//! suppressed, or two cuts reordered. Nobody tunes on these runs (note §11.2 held-out set).
//!
//! **Oracle** (no hand-made expectation): P is mapped through the edit (scaled by a dimension
//! edit, unchanged otherwise). At the fillet's input state after the edit, if exactly one Line
//! edge of the body contains P, a quiet `Ok` must return that edge; if none or several do, a
//! quiet `Ok` is wrong. Any quiet `Ok` on another entity is a silent wrong and fails the run.
//! Warnings and errors are always allowed. The run prints the count of each outcome.
//!
//! Needs OCCT: `cargo nextest run -p amg-features --features occt --test tnp_fuzz`.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    reason = "test harness"
)]

mod tnp_common;

use serde_json::{Value, json};

const RUNS: u64 = 200;
const SEED: u64 = 0x0D0C_0013_7A9F_5EED;
const W: f64 = 100.0;
const D: f64 = 60.0;
const H: f64 = 40.0;

/// SplitMix64: a fixed, dependency-free stream.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    /// A value rounded to 0.1 mm (keeps the generated documents readable).
    fn mm(&mut self, lo: f64, hi: f64) -> f64 {
        (self.range(lo, hi) * 10.0).round() / 10.0
    }
}

/// Top edge `e` (0 front y=0, 1 right x=W, 2 back y=D, 3 left x=0) at parameter `t`.
fn on_edge(e: u64, t: f64) -> [f64; 3] {
    match e {
        0 => [t * W, 0.0, H],
        1 => [W, t * D, H],
        2 => [t * W, D, H],
        _ => [0.0, t * D, H],
    }
}

fn along_x(e: u64) -> bool {
    e == 0 || e == 2
}

/// A Cut prism (lane-E test feature, mapped to sketch + extrude by the harness) of the
/// rectangle x0..x1 × y0..y1, z H−10..H+10, on the base body.
fn cut(base: &str, tags: [i64; 4], x0: f64, x1: f64, y0: f64, y1: f64, at: Value) -> Value {
    json!({"AddFeature": {"at": at, "feature": {"kind": "test.brep_prism", "v": 1, "data": {
        "points": [[tags[0], x0, y0], [tags[1], x1, y0], [tags[2], x1, y1], [tags[3], x0, y1]],
        "z0": H - 10.0, "height": 20.0, "op": "Cut",
        "target": {"origin": format!("${base}"), "slot": 0}}},
        "context": "$root", "name": null}})
}

/// A groove across the top: along y at x a..a+10 (`across_x`), or along x at y a..a+10.
fn groove(base: &str, tags: [i64; 4], across_x: bool, a: f64, at: Value) -> Value {
    if across_x {
        cut(base, tags, a, a + 10.0, -10.0, D + 10.0, at)
    } else {
        cut(base, tags, -10.0, W + 10.0, a, a + 10.0, at)
    }
}

fn prism_points(pts: &[(i64, f64, f64)]) -> Value {
    json!(
        pts.iter()
            .map(|&(t, x, y)| json!([t, x, y]))
            .collect::<Vec<_>>()
    )
}

fn rect() -> Vec<(i64, f64, f64)> {
    vec![(1, 0.0, 0.0), (2, W, 0.0), (3, W, D), (4, 0.0, D)]
}

/// A random point on top edge `e`, 5 % away from the corners and 2 mm away from the
/// intervals in `avoid` (along the edge).
fn pick_point(rng: &mut Rng, e: u64, avoid: &[(f64, f64)]) -> [f64; 3] {
    let len = if along_x(e) { W } else { D };
    loop {
        let t = rng.range(0.05, 0.95);
        let s = t * len;
        if avoid.iter().all(|&(a, b)| s < a - 2.0 || s > b + 2.0) {
            return on_edge(e, (s * 10.0).round() / 10.0 / len);
        }
    }
}

struct Generated {
    case: Value,
    kind: &'static str,
    /// The pick point mapped through the edit.
    mapped: [f64; 3],
}

fn generate(rng: &mut Rng) -> Generated {
    let b2 = rng.below(2) == 1;
    let base = if b2 { "prism" } else { "box" };
    let mut script: Vec<Value> = Vec::new();
    script.push(if b2 {
        json!({"bind": "prism", "cmd": {"AddFeature": {"at": null, "feature": {"kind": "test.brep_prism", "v": 1,
            "data": {"points": prism_points(&rect()), "z0": 0, "height": H, "op": "New", "target": null}},
            "context": "$root", "name": null}}})
    } else {
        json!({"bind": "box", "cmd": {"AddFeature": {"at": null, "feature": {"kind": "test.brep_box", "v": 1,
            "data": {"size": [W, D, H]}}, "context": "$root", "name": null}}})
    });
    let e = rng.below(4);
    let mut kind = match rng.below(6) {
        0 => "dimension",
        1 => "notch",
        2 => "groove",
        3 => "vertex",
        4 => "suppress",
        _ => "reorder",
    };
    if kind == "vertex" && !b2 {
        kind = "dimension";
    }
    // Pre-existing cuts (suppress / reorder) and the intervals of the picked edge they remove.
    let mut avoid = Vec::new();
    let across_x = rng.below(2) == 0;
    if kind == "suppress" || kind == "reorder" {
        let a = rng.mm(5.0, if across_x { W - 15.0 } else { D - 15.0 });
        script.push(json!({"bind": "cutA", "cmd": groove(base, [51, 52, 53, 54], across_x, a, Value::Null)}));
        if along_x(e) == across_x {
            avoid.push((a, a + 10.0));
        }
        if kind == "reorder" {
            // A pocket inside the top face, clear of the groove.
            let (x0, y0) = if across_x {
                let x0 = if a > 50.0 {
                    rng.mm(5.0, a - 20.0)
                } else {
                    rng.mm(a + 15.0, W - 20.0)
                };
                (x0, rng.mm(10.0, D - 25.0))
            } else {
                let y0 = if a > 30.0 { 5.0 } else { D - 20.0 };
                (rng.mm(10.0, W - 25.0), y0.min(D - 20.0))
            };
            script.push(json!({"bind": "cutB", "cmd": cut(base, [21, 22, 23, 24], x0, x0 + 15.0, y0, y0 + 15.0, Value::Null)}));
        }
    }
    let p = pick_point(rng, e, &avoid);
    script.push(json!({"bind": "fil", "cmd": {"AddFeature": {"at": null, "feature": {"kind": "test.brep_fillet", "v": 1,
        "data": {"body": {"origin": format!("${base}"), "slot": 0},
                 "edges": [{"$pick_on": {"kind": "Edge", "at": p}}], "radius": 2}},
        "context": "$root", "name": null}}}));
    let mut mapped = p;
    let before_fil = json!("$index_of:fil");
    let edit: Vec<Value> = match kind {
        "dimension" => {
            let s = [
                rng.range(0.5, 1.5),
                rng.range(0.5, 1.5),
                rng.range(0.5, 1.5),
            ]
            .map(|v| (v * 100.0).round() / 100.0);
            mapped = [p[0] * s[0], p[1] * s[1], p[2] * s[2]];
            if b2 {
                let pts: Vec<(i64, f64, f64)> = rect()
                    .into_iter()
                    .map(|(t, x, y)| (t, x * s[0], y * s[1]))
                    .collect();
                vec![
                    json!({"patch": {"id": "$prism", "merge": {"points": prism_points(&pts), "height": H * s[2]}}}),
                ]
            } else {
                vec![
                    json!({"cmd": {"EditFeature": {"id": "$box", "feature": {"kind": "test.brep_box", "v": 1,
                    "data": {"size": [W * s[0], D * s[1], H * s[2]]}}}}}),
                ]
            }
        }
        "notch" => {
            let q = on_edge(e, rng.range(0.1, 0.9));
            let (x0, x1, y0, y1) = if along_x(e) {
                (q[0] - 5.0, q[0] + 5.0, q[1] - 10.0, q[1] + 10.0)
            } else {
                (q[0] - 10.0, q[0] + 10.0, q[1] - 5.0, q[1] + 5.0)
            };
            vec![
                json!({"bind": "notch", "cmd": cut(base, [41, 42, 43, 44], x0, x1, y0, y1, before_fil)}),
            ]
        }
        "groove" => {
            let a = rng.mm(5.0, if across_x { W - 15.0 } else { D - 15.0 });
            vec![
                json!({"bind": "groove", "cmd": groove(base, [61, 62, 63, 64], across_x, a, before_fil)}),
            ]
        }
        "vertex" => {
            let mut pts = rect();
            if rng.below(2) == 0 {
                // A vertex pushed 15 mm out of the middle of polygon edge i.
                let i = rng.below(4) as usize;
                let (a, b) = (pts[i], pts[(i + 1) % 4]);
                let (mx, my) = ((a.1 + b.1) / 2.0, (a.2 + b.2) / 2.0);
                let out = match i {
                    0 => (mx, my - 15.0),
                    1 => (mx + 15.0, my),
                    2 => (mx, my + 15.0),
                    _ => (mx - 15.0, my),
                };
                pts.insert(i + 1, (5, out.0, out.1));
            } else {
                pts.remove(rng.below(4) as usize);
            }
            vec![json!({"patch": {"id": "$prism", "merge": {"points": prism_points(&pts)}}})]
        }
        "suppress" => {
            vec![json!({"cmd": {"SetSuppressed": {"ids": ["$cutA"], "suppressed": true}}})]
        }
        _ => vec![json!({"cmd": {"MoveFeature": {"id": "$cutB", "to": 1}}})],
    };
    Generated {
        case: json!({"id": "fuzz", "script": script, "edit": edit}),
        kind,
        mapped,
    }
}

#[test]
fn fuzz_cases_are_deterministic() {
    let mut a = Rng(SEED);
    let mut b = Rng(SEED);
    for _ in 0..RUNS {
        let (x, y) = (generate(&mut a), generate(&mut b));
        assert_eq!(x.case, y.case);
        assert_eq!(x.kind, y.kind);
        assert_eq!(x.mapped.map(f64::to_bits), y.mapped.map(f64::to_bits));
    }
}

#[cfg(feature = "occt")]
#[test]
fn doc13_fuzz_set_has_no_silent_wrong() {
    use std::collections::BTreeMap;
    use std::panic::{AssertUnwindSafe, catch_unwind};
    use std::path::PathBuf;

    use amg_doc::EntityRef;

    let tmp = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("tnp-fuzz");
    std::fs::create_dir_all(&tmp).unwrap();
    let started = std::time::Instant::now();
    let mut rng = Rng(SEED);
    let mut counts: BTreeMap<String, u32> = BTreeMap::new();
    let mut by_edit: BTreeMap<String, u32> = BTreeMap::new();
    let mut silent = Vec::new();
    let mut harness = Vec::new();
    for n in 0..RUNS {
        let g = generate(&mut rng);
        *by_edit.entry(g.kind.to_owned()).or_default() += 1;
        let out = catch_unwind(AssertUnwindSafe(|| {
            let mut run = tnp_common::Run::new(tmp.clone());
            for s in g.case["script"].as_array().unwrap() {
                run.step(s);
            }
            if let Some(e) = tnp_common::first_error(&run.eval_at(None)) {
                return ("base feature error".to_owned(), None, e);
            }
            for s in g.case["edit"].as_array().unwrap() {
                run.step(s);
            }
            run.assert_s04_only();
            let fil = run.feature("fil");
            let eval = run.eval_at(None);
            let node = eval.node(fil).unwrap();
            let rec = node
                .resolutions
                .first()
                .expect("the fillet's edge record")
                .clone();
            let input = run.eval_at(Some(run.index_of(fil)));
            let &(body, _, _) = run.picks.get(&(fil, 0)).unwrap();
            let intended = tnp_common::lines_containing(&input, body, g.mapped, 1e-6);
            let label = match &rec.result {
                Ok(v) => match v[0].warning {
                    Some(w) => format!("Warning {}", tnp_common::variant(&format!("{w:?}"))),
                    None => format!("Ok {:?}", v[0].quality),
                },
                Err(e) => format!("Error {}", tnp_common::variant(&format!("{e:?}"))),
            };
            let quiet_ok = matches!(&rec.result, Ok(v) if v[0].warning.is_none());
            let wrong = if !quiet_ok {
                None
            } else if intended.len() != 1 {
                Some(format!("quiet Ok, but {} edges contain P", intended.len()))
            } else {
                let got = match rec.current.first() {
                    Some(EntityRef::Topo(t)) => t.fingerprint.clone(),
                    _ => None,
                };
                let v = rec.result.as_ref().unwrap();
                match got {
                    Some(fp)
                        if v[0].body == body
                            && (fp.centroid - intended[0].1.centroid).length() <= 1e-6
                            && (fp.measure - intended[0].1.measure).abs() <= 1e-6 =>
                    {
                        None
                    }
                    other => Some(format!(
                        "quiet Ok on {:?} {:?}, intended {:?}",
                        v[0].body,
                        other.map(|f| (f.centroid, f.measure)),
                        (intended[0].1.centroid, intended[0].1.measure)
                    )),
                }
            };
            (label, wrong, String::new())
        }));
        match out {
            Ok((label, wrong, note)) => {
                if let Some(w) = wrong {
                    println!("run {n} ({}): SILENT WRONG {w}\n  {}", g.kind, g.case);
                    silent.push(n);
                }
                if !note.is_empty() {
                    println!("run {n} ({}): {label}: {note}", g.kind);
                }
                *counts.entry(format!("{} / {label}", g.kind)).or_default() += 1;
            }
            Err(_) => {
                println!("run {n} ({}): HARNESS ERROR\n  {}", g.kind, g.case);
                harness.push(n);
            }
        }
    }
    let secs = started.elapsed().as_secs_f64();
    println!("fuzz: {RUNS} runs, seed {SEED:#x}, {secs:.1} s; edits {by_edit:?}");
    let mut totals: BTreeMap<String, u32> = BTreeMap::new();
    for (k, v) in &counts {
        println!("  {k}: {v}");
        let outcome = k.split(" / ").nth(1).unwrap_or(k);
        *totals.entry(outcome.to_owned()).or_default() += v;
    }
    println!("  totals {totals:?}; silent wrong {silent:?}; harness errors {harness:?}");
    assert!(silent.is_empty(), "silent wrong runs: {silent:?}");
    assert!(harness.is_empty(), "harness errors: {harness:?}");
}
