//! Note §5.3 metrics for one option on one case: correctness (runner verdict + uniform residual
//! judge), cold and warm solve time (median / p95), drag frame time and jitter, chirality flips,
//! DOF and the reported failing sets.

use std::fmt::Write as _;
use std::time::Instant;

use super::{Contender, judge};
use crate::corpus::runner::{BackendOutcome, BackendSkip, CorpusBackend, Verdict, run_case};
use crate::corpus::{BuiltCase, Case, CaseGeom, CaseStatus, ExpectedGroup, Values};
use crate::model::ConstraintKind;

/// Timing summary in microseconds.
#[derive(Clone, Debug, PartialEq)]
pub struct Stats {
    /// Samples.
    pub runs: usize,
    /// Median, µs.
    pub median_us: f64,
    /// 95th percentile, µs.
    pub p95_us: f64,
}

impl Stats {
    /// From samples in seconds (`None` if empty).
    pub fn from_secs(mut v: Vec<f64>) -> Option<Stats> {
        if v.is_empty() {
            return None;
        }
        v.sort_by(f64::total_cmp);
        let at = |q: f64| v[((v.len() - 1) as f64 * q).round() as usize] * 1e6;
        Some(Stats {
            runs: v.len(),
            median_us: at(0.5),
            p95_us: at(0.95),
        })
    }
}

/// Drag metrics.
#[derive(Clone, Debug, PartialEq)]
pub struct DragMetrics {
    /// Frames per pass.
    pub frames: usize,
    /// Per-frame time over all passes.
    pub frame: Stats,
    /// `max_k max_{p not dragged} ‖p_k − p_{k−1}‖ / ‖c_k − c_{k−1}‖` (first pass).
    pub jitter: f64,
    /// Frames whose result satisfies all equations (first pass).
    pub converged_frames: usize,
    /// Assembly-branch flips (cases with points `o1`, `a`, `b`: sign of
    /// `cross(a − o1, b − o1)`, ignoring frames at the toggle where `|sin| < 1e-6`), first pass.
    pub flips: Option<u32>,
}

/// All metrics of one option on one case.
#[derive(Clone, Debug)]
pub struct CaseMetrics {
    /// Case id.
    pub case: String,
    /// `D1`..`D10` from the `decision:` tag.
    pub decision: Option<String>,
    /// Option name.
    pub contender: String,
    /// Why the option cannot run the case.
    pub skipped: Option<String>,
    /// Corpus runner verdict (status, DOF when reported, expected root).
    pub verdict: Option<Verdict>,
    /// Reported status of the correctness solve.
    pub status: Option<CaseStatus>,
    /// Largest SK-02 residual after the correctness solve, mm.
    pub max_residual: f64,
    /// Judge tolerance, mm.
    pub tolerance: f64,
    /// Reported DOF.
    pub dof: Option<u32>,
    /// Reported failing / dependent sets.
    pub groups: Option<Vec<ExpectedGroup>>,
    /// Cold solve (map + solve from the start configuration).
    pub cold: Option<Stats>,
    /// Warm solve (one driving dimension ±1 %, from the previous solution).
    pub warm: Option<Stats>,
    /// Warm solves that satisfied all equations.
    pub warm_converged: Option<(usize, usize)>,
    /// Drag metrics.
    pub drag: Option<DragMetrics>,
}

impl CaseMetrics {
    /// True when the runner passed the case.
    pub fn passed(&self) -> bool {
        self.verdict == Some(Verdict::Pass)
    }
}

struct Dyn<'a>(&'a dyn Contender);

impl CorpusBackend for Dyn<'_> {
    fn name(&self) -> String {
        self.0.name()
    }
    fn solve(
        &mut self,
        _case: &Case,
        built: &mut BuiltCase,
    ) -> Result<BackendOutcome, BackendSkip> {
        let mut l = self.0.load(built)?;
        let out = l.solve();
        built.apply(&l.values()).map_err(BackendSkip)?;
        Ok(BackendOutcome {
            status: out.status,
            dof: out.dof,
            max_residual: Some(judge(&built.sketch).0),
            iterations: out.iterations,
            groups: out.groups,
        })
    }
}

fn first_driving_dimension(case: &Case) -> Option<(u32, f64)> {
    case.constraints
        .iter()
        .enumerate()
        .find_map(|(i, c)| match c {
            ConstraintKind::Dimension(d) if d.driving => Some((u32::try_from(i).ok()?, d.value)),
            _ => None,
        })
}

fn branch_points(case: &Case) -> Option<[u32; 3]> {
    let find = |name: &str| {
        case.entities
            .iter()
            .position(|e| e.name == name && matches!(e.geom, CaseGeom::Point { .. }))
            .and_then(|i| u32::try_from(i).ok())
    };
    Some([find("o1")?, find("a")?, find("b")?])
}

fn residual_of(built: &BuiltCase, values: &Values) -> f64 {
    let mut b = built.clone();
    if b.apply(values).is_err() {
        return f64::INFINITY;
    }
    judge(&b.sketch).0
}

/// Measures `contender` on `case`: `runs` cold and warm solves, `drag_passes` passes over the
/// drag script.
pub fn measure(
    contender: &dyn Contender,
    case: &Case,
    runs: usize,
    drag_passes: usize,
) -> CaseMetrics {
    let mut m = CaseMetrics {
        case: case.id.clone(),
        decision: case
            .tags
            .iter()
            .find_map(|t| t.strip_prefix("decision:").map(str::to_string)),
        contender: contender.name(),
        skipped: None,
        verdict: None,
        status: None,
        max_residual: f64::INFINITY,
        tolerance: 0.0,
        dof: None,
        groups: None,
        cold: None,
        warm: None,
        warm_converged: None,
        drag: None,
    };
    let built = match case.build() {
        Ok(b) => b,
        Err(e) => {
            m.skipped = Some(format!("case does not build: {e}"));
            return m;
        }
    };
    m.tolerance = judge(&built.sketch).1;
    let r = run_case(&mut Dyn(contender), case);
    if let Verdict::Skipped(why) = &r.verdict {
        m.skipped = Some(why.clone());
        return m;
    }
    m.verdict = Some(r.verdict.clone());
    if let Some(o) = &r.outcome {
        m.status = Some(o.status);
        m.dof = o.dof;
        m.groups = o.groups.clone();
        m.max_residual = o.max_residual.unwrap_or(f64::INFINITY);
    }

    // Cold.
    let mut t = Vec::with_capacity(runs);
    for _ in 0..runs {
        let b = built.clone();
        let t0 = Instant::now();
        if let Ok(mut l) = contender.load(&b) {
            let _ = l.solve();
        }
        t.push(t0.elapsed().as_secs_f64());
    }
    m.cold = Stats::from_secs(t);

    // Warm: one driving dimension alternates between +1 % and its value.
    if let Some((ci, v0)) = first_driving_dimension(case)
        && let Ok(mut l) = contender.load(&built)
    {
        let _ = l.solve();
        let mut t = Vec::with_capacity(runs);
        let mut ok = 0;
        for i in 0..runs {
            let v = if i % 2 == 0 { v0 * 1.01 } else { v0 };
            let t0 = Instant::now();
            let set = l.set_dimension(ci, v);
            let out = l.solve();
            t.push(t0.elapsed().as_secs_f64());
            if set.is_ok() && out.status.is_converged() {
                ok += 1;
            }
        }
        m.warm = Stats::from_secs(t);
        m.warm_converged = Some((ok, runs));
    }

    // Drag.
    if let Some(script) = &case.drag
        && !script.path.is_empty()
    {
        let mut times = Vec::new();
        let (mut jitter, mut converged, mut flips) = (0.0_f64, 0, 0_u32);
        let branch = branch_points(case);
        let side = |v: &Values| -> Option<f64> {
            let [o, a, b] = branch?;
            let (o, a, b) = (v.points.get(&o)?, v.points.get(&a)?, v.points.get(&b)?);
            let (u, w) = ([a[0] - o[0], a[1] - o[1]], [b[0] - o[0], b[1] - o[1]]);
            // Normalised sine; at the toggle (|sin| < 1e-6) the branch is undefined.
            let sin = (u[0] * w[1] - u[1] * w[0]) / (u[0].hypot(u[1]) * w[0].hypot(w[1]));
            (sin.abs() >= 1e-6).then_some(sin)
        };
        for pass in 0..drag_passes.max(1) {
            let Ok(mut l) = contender.load(&built) else {
                break;
            };
            let _ = l.solve();
            let mut prev = l.values();
            let mut prev_cursor = prev.points.get(&script.entity).copied();
            let mut prev_side = side(&prev);
            for &target in &script.path {
                let t0 = Instant::now();
                let _ = l.drag(script.entity, target);
                times.push(t0.elapsed().as_secs_f64());
                if pass > 0 {
                    continue;
                }
                let now = l.values();
                if residual_of(&built, &now) <= m.tolerance {
                    converged += 1;
                }
                if let Some(c0) = prev_cursor {
                    let step = (target[0] - c0[0]).hypot(target[1] - c0[1]);
                    if step > 0.0 {
                        for (i, p) in &now.points {
                            if *i == script.entity {
                                continue;
                            }
                            if let Some(q) = prev.points.get(i) {
                                jitter = jitter.max((p[0] - q[0]).hypot(p[1] - q[1]) / step);
                            }
                        }
                    }
                }
                let s = side(&now);
                if let (Some(a), Some(b)) = (prev_side, s)
                    && a * b < 0.0
                {
                    flips += 1;
                }
                prev_side = s.or(prev_side);
                prev_cursor = Some(target);
                prev = now;
            }
        }
        m.drag = Stats::from_secs(times).map(|frame| DragMetrics {
            frames: script.path.len(),
            frame,
            jitter,
            converged_frames: converged,
            flips: branch.map(|_| flips),
        });
    }
    m
}

fn fmt_groups(g: &Option<Vec<ExpectedGroup>>) -> String {
    match g {
        None => "-".into(),
        Some(v) if v.is_empty() => "none".into(),
        Some(v) => v
            .iter()
            .map(|g| {
                let ids: Vec<String> = g.constraints.iter().map(u32::to_string).collect();
                format!("{:?}{{{}}}", g.kind, ids.join(","))
            })
            .collect::<Vec<_>>()
            .join(" "),
    }
}

fn fmt_stats(s: &Option<Stats>) -> String {
    s.as_ref().map_or("-".into(), |s| {
        format!("{:.1} / {:.1}", s.median_us, s.p95_us)
    })
}

/// Markdown table, one row per (case, option).
pub fn markdown(rows: &[CaseMetrics]) -> String {
    let mut s = String::new();
    s.push_str(
        "| Case | Option | Solved (runner) | Status | max‖F‖ (mm) | DOF | Cold µs med / p95 | Warm µs med / p95 | Drag µs/frame med / p95 | Jitter | Flips | Drag frames ok | Failing sets |\n",
    );
    s.push_str("|---|---|---|---|---|---|---|---|---|---|---|---|---|\n");
    for r in rows {
        let name = r.decision.clone().unwrap_or_else(|| r.case.clone());
        if let Some(why) = &r.skipped {
            let _ = writeln!(
                s,
                "| {name} | {} | unsupported: {why} | - | - | - | - | - | - | - | - | - | - |",
                r.contender
            );
            continue;
        }
        let solved = match &r.verdict {
            Some(Verdict::Pass) => "yes".to_string(),
            Some(Verdict::Fail(w)) => format!("**no**: {}", w.join("; ")),
            _ => "-".into(),
        };
        let drag = r.drag.as_ref();
        let _ = writeln!(
            s,
            "| {name} | {} | {solved} | {} | {:.1e} | {} | {} | {}{} | {} | {} | {} | {} | {} |",
            r.contender,
            r.status.map_or("-".into(), |x| format!("{x:?}")),
            r.max_residual,
            r.dof.map_or("-".into(), |d| d.to_string()),
            fmt_stats(&r.cold),
            fmt_stats(&r.warm),
            r.warm_converged
                .map_or(String::new(), |(ok, n)| if ok == n {
                    String::new()
                } else {
                    format!(" ({ok}/{n} conv.)")
                }),
            drag.map_or("-".into(), |d| fmt_stats(&Some(d.frame.clone()))),
            drag.map_or("-".into(), |d| format!("{:.3}", d.jitter)),
            drag.and_then(|d| d.flips)
                .map_or("-".into(), |f| f.to_string()),
            drag.map_or("-".into(), |d| format!(
                "{}/{}",
                d.converged_frames, d.frames
            )),
            fmt_groups(&r.groups),
        );
    }
    s
}

/// CSV with the same columns (raw numbers).
pub fn csv(rows: &[CaseMetrics]) -> String {
    let mut s = String::from(
        "case,option,skipped,pass,status,max_residual,dof,cold_med_us,cold_p95_us,warm_med_us,warm_p95_us,drag_med_us,drag_p95_us,jitter,flips,drag_ok,drag_frames\n",
    );
    let num = |v: Option<f64>| v.map_or(String::new(), |x| format!("{x:.3}"));
    for r in rows {
        let d = r.drag.as_ref();
        let _ = writeln!(
            s,
            "{},{},{},{},{},{:e},{},{},{},{},{},{},{},{},{},{},{}",
            r.decision.clone().unwrap_or_else(|| r.case.clone()),
            r.contender,
            r.skipped.is_some(),
            r.passed(),
            r.status.map_or(String::new(), |x| format!("{x:?}")),
            r.max_residual,
            r.dof.map_or(String::new(), |x| x.to_string()),
            num(r.cold.as_ref().map(|x| x.median_us)),
            num(r.cold.as_ref().map(|x| x.p95_us)),
            num(r.warm.as_ref().map(|x| x.median_us)),
            num(r.warm.as_ref().map(|x| x.p95_us)),
            num(d.map(|x| x.frame.median_us)),
            num(d.map(|x| x.frame.p95_us)),
            num(d.map(|x| x.jitter)),
            d.and_then(|x| x.flips)
                .map_or(String::new(), |x| x.to_string()),
            d.map_or(String::new(), |x| x.converged_frames.to_string()),
            d.map_or(String::new(), |x| x.frames.to_string()),
        );
    }
    s
}
