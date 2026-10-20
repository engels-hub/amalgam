//! SK-14 corpus generator: the `own/` and `planegcs/` JSON cases and the `expected.json`
//! sidecars of the imported SolveSpace and ezpz sets.
//!
//! The checked-in files are the source of truth for the runner; this test regenerates them
//! and compares (JSON trees, 1e-9 relative tolerance, because libm differs across platforms).
//! `AMG_BLESS=1 cargo nextest run -p amg-sketch -E 'test(corpus_gen)'` rewrites them.
//!
//! Every generated case states its status and DOF *by hand*; the generator solves the case
//! with the independent reference solver and fails if the oracle disagrees, so a wrong
//! hand-derived expectation or a wrong oracle is caught either way. Hand-known coordinates
//! are asserted and used as the stored solution.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    reason = "test helpers; a failure is a test failure"
)]

use std::path::{Path, PathBuf};

use amg_core::Point2;
use amg_sketch::corpus::{
    self, Case, CaseEntity, CaseGeom, CaseSource, CaseStatus, CorpusBackend, DragScript, Expected,
    ReferenceCheck, ReferenceSolver, Sidecar, SidecarEntry, Solution, SourceSet,
};
use amg_sketch::model::HvTarget;
use amg_sketch::model::{AngleBranch, End, Side, TangentKind};
use amg_sketch::{ConstraintKind as K, Dimension, DimensionKind as Dk, EntityId};
use serde_json::Value;

use std::f64::consts::PI;

// ---- Case builder -------------------------------------------------------------------------

#[derive(Default)]
struct B {
    e: Vec<CaseEntity>,
    c: Vec<K>,
    drag: Option<DragScript>,
}

fn id(i: u32) -> EntityId {
    EntityId(i)
}

impl B {
    fn new() -> B {
        B::default()
    }
    fn push(&mut self, name: &str, geom: CaseGeom, construction: bool) -> u32 {
        self.e.push(CaseEntity {
            name: name.to_string(),
            geom,
            construction,
        });
        u32::try_from(self.e.len() - 1).unwrap()
    }
    fn pt(&mut self, name: &str, x: f64, y: f64) -> u32 {
        self.push(name, CaseGeom::Point { at: [x, y] }, false)
    }
    fn line(&mut self, name: &str, start: u32, end: u32) -> u32 {
        self.push(name, CaseGeom::Line { start, end }, false)
    }
    fn cline(&mut self, name: &str, start: u32, end: u32) -> u32 {
        self.push(name, CaseGeom::Line { start, end }, true)
    }
    fn circle(&mut self, name: &str, center: u32, radius: f64) -> u32 {
        self.push(name, CaseGeom::Circle { center, radius }, false)
    }
    fn arc(&mut self, name: &str, center: u32, start: u32, end: u32) -> u32 {
        self.push(name, CaseGeom::Arc { center, start, end }, false)
    }
    fn con(&mut self, k: K) -> u32 {
        self.c.push(k);
        u32::try_from(self.c.len() - 1).unwrap()
    }
    fn dim(&mut self, kind: Dk, value: f64, driving: bool) -> u32 {
        self.con(K::Dimension(Dimension {
            kind,
            value,
            driving,
            label_pos: Point2::ZERO,
        }))
    }
    fn fix(&mut self, e: u32) {
        self.con(K::Fix { entity: id(e) });
    }
    fn coin(&mut self, a: u32, b: u32) {
        self.con(K::Coincident { a: id(a), b: id(b) });
    }
    fn hz(&mut self, l: u32) {
        self.con(K::Horizontal(HvTarget::Line(id(l))));
    }
    fn vt(&mut self, l: u32) {
        self.con(K::Vertical(HvTarget::Line(id(l))));
    }
    fn hzp(&mut self, a: u32, b: u32) {
        self.con(K::Horizontal(HvTarget::Points(id(a), id(b))));
    }
    fn vtp(&mut self, a: u32, b: u32) {
        self.con(K::Vertical(HvTarget::Points(id(a), id(b))));
    }
    fn par(&mut self, a: u32, b: u32) {
        self.con(K::Parallel { a: id(a), b: id(b) });
    }
    fn perp(&mut self, a: u32, b: u32) {
        self.con(K::Perpendicular { a: id(a), b: id(b) });
    }
    fn col(&mut self, a: u32, b: u32) {
        self.con(K::Collinear { a: id(a), b: id(b) });
    }
    fn conc(&mut self, a: u32, b: u32) {
        self.con(K::Concentric { a: id(a), b: id(b) });
    }
    fn eq(&mut self, a: u32, b: u32) {
        self.con(K::Equal { a: id(a), b: id(b) });
    }
    fn mid(&mut self, point: u32, line: u32) {
        self.con(K::Midpoint {
            point: id(point),
            line: id(line),
        });
    }
    fn sym(&mut self, a: u32, b: u32, axis: u32) {
        self.con(K::Symmetric {
            a: id(a),
            b: id(b),
            axis: id(axis),
        });
    }
    fn on(&mut self, point: u32, curve: u32) {
        self.con(K::PointOnCurve {
            point: id(point),
            curve: id(curve),
        });
    }
    fn tan(&mut self, a: u32, b: u32, kind: TangentKind) {
        self.con(K::Tangent {
            a: id(a),
            b: id(b),
            kind,
        });
    }
    fn tan_end(&mut self, a: u32, a_end: End, b: u32, b_end: End, reversed: bool) {
        self.tan(
            a,
            b,
            TangentKind::AtEndpoint {
                a_end,
                b_end,
                reversed,
            },
        );
    }
    fn len(&mut self, line: u32, v: f64) {
        self.dim(Dk::Length { line: id(line) }, v, true);
    }
    fn dist(&mut self, a: u32, b: u32, side: Side, v: f64) {
        self.dim(
            Dk::Distance {
                a: id(a),
                b: id(b),
                side,
            },
            v,
            true,
        );
    }
    fn dist_driven(&mut self, a: u32, b: u32, side: Side, v: f64) {
        self.dim(
            Dk::Distance {
                a: id(a),
                b: id(b),
                side,
            },
            v,
            false,
        );
    }
    /// Horizontal distance `b.x − a.x = v` (the sign of `v` picks the side).
    fn hd(&mut self, a: u32, b: u32, v: f64) {
        let side = if v < 0.0 {
            Side::Negative
        } else {
            Side::Positive
        };
        self.dim(
            Dk::HorizontalDistance {
                a: id(a),
                b: id(b),
                side,
            },
            v.abs(),
            true,
        );
    }
    /// Vertical distance `b.y − a.y = v`.
    fn vd(&mut self, a: u32, b: u32, v: f64) {
        let side = if v < 0.0 {
            Side::Negative
        } else {
            Side::Positive
        };
        self.dim(
            Dk::VerticalDistance {
                a: id(a),
                b: id(b),
                side,
            },
            v.abs(),
            true,
        );
    }
    fn rad(&mut self, curve: u32, v: f64) {
        self.dim(Dk::Radius { curve: id(curve) }, v, true);
    }
    fn dia(&mut self, curve: u32, v: f64) {
        self.dim(Dk::Diameter { curve: id(curve) }, v, true);
    }
    fn arclen(&mut self, arc: u32, v: f64) {
        self.dim(Dk::ArcLength { arc: id(arc) }, v, true);
    }
    fn ang(&mut self, a: u32, b: u32, v: f64) {
        self.dim(
            Dk::Angle {
                a: id(a),
                b: id(b),
                branch: AngleBranch {
                    reverse_a: false,
                    reverse_b: false,
                },
            },
            v,
            true,
        );
    }
}

// ---- Expectations -------------------------------------------------------------------------

/// What the author knows about a case before the oracle looks at it.
enum Want {
    /// Solvable. `start` = the start configuration already satisfies the constraints (the
    /// expectation is `Solution::Start`).
    Solved {
        status: CaseStatus,
        dof: u32,
        start: bool,
    },
    /// No solution; any of these statuses is acceptable.
    Unsolvable {
        status: CaseStatus,
        also: &'static [CaseStatus],
    },
}

use CaseStatus::{Converged, ConvergedRedundant, DidNotConverge, Inconsistent};

fn solved(dof: u32) -> Want {
    Want::Solved {
        status: Converged,
        dof,
        start: false,
    }
}
fn solved_start(dof: u32) -> Want {
    Want::Solved {
        status: Converged,
        dof,
        start: true,
    }
}
fn redundant(dof: u32) -> Want {
    Want::Solved {
        status: ConvergedRedundant,
        dof,
        start: false,
    }
}

fn round12(v: f64) -> f64 {
    (v * 1e12).round() / 1e12
}

/// Turns a builder into a case, checking `want` (and the hand-known coordinates `hand`)
/// against the reference solver.
#[allow(clippy::too_many_arguments)]
fn finish(
    b: B,
    set: SourceSet,
    name: &str,
    origin: &str,
    licence: &str,
    description: &str,
    tags: &[&str],
    want: &Want,
    hand: &[(u32, [f64; 2])],
) -> Case {
    let id = format!("{}/{name}", set.dir());
    let mut case = Case {
        format: corpus::FORMAT_VERSION,
        id: id.clone(),
        source: CaseSource {
            set,
            origin: origin.to_string(),
            licence: licence.to_string(),
        },
        description: description.to_string(),
        tags: tags.iter().map(|t| (*t).to_string()).collect(),
        entities: b.e,
        constraints: b.c,
        expected: Expected {
            status: Converged,
            also_accept: Vec::new(),
            dof: Some(0),
            solution: None,
            tolerance: corpus::DEFAULT_SOLUTION_TOL,
            groups: Vec::new(),
        },
        drag: b.drag,
    };
    let mut built = case.build().unwrap_or_else(|e| panic!("{id}: {e}"));
    let (status, dof, worst, _) = ReferenceSolver::default()
        .solve_sketch(&mut built.sketch)
        .unwrap_or_else(|e| panic!("{id}: {e}"));
    match want {
        Want::Solved {
            status: ws,
            dof: wd,
            start,
        } => {
            assert_eq!(status, *ws, "{id}: oracle status");
            assert_eq!(dof, *wd, "{id}: oracle dof");
            assert!(worst < 1e-8, "{id}: residual {worst}");
            let mut values = built.values();
            for v in values.points.values_mut() {
                v[0] = round12(v[0]);
                v[1] = round12(v[1]);
            }
            for r in values.radii.values_mut() {
                *r = round12(*r);
            }
            for (i, h) in hand {
                let got = values.points[i];
                assert!(
                    (got[0] - h[0]).abs() < 1e-6 && (got[1] - h[1]).abs() < 1e-6,
                    "{id}: entity {i} solved to {got:?}, hand value {h:?}"
                );
                values.points.insert(*i, *h);
            }
            case.expected.status = *ws;
            case.expected.dof = Some(*wd);
            case.expected.solution = Some(if *start {
                Solution::Start
            } else if *wd == 0 {
                Solution::Target(values)
            } else {
                Solution::Witness(values)
            });
        }
        Want::Unsolvable { status: ws, also } => {
            assert!(
                status == *ws || also.contains(&status),
                "{id}: oracle status {status:?}, expected {ws:?} or {also:?}"
            );
            case.expected.status = *ws;
            case.expected.also_accept = also.to_vec();
            case.expected.dof = None;
        }
    }
    // The stored solution must satisfy the constraints (checked by a second backend path).
    if case.expected.solution.is_some() {
        let mut b2 = case.build().unwrap();
        let out = ReferenceCheck::default().solve(&case, &mut b2).unwrap();
        assert!(
            out.status.is_converged() && out.dof == case.expected.dof,
            "{id}: reference check {out:?}"
        );
    }
    case.validate().unwrap();
    case
}

const OWN_LICENCE: &str = "LicenseRef-Amalgam-Personal";
const FC_LICENCE: &str = "LGPL-2.1-or-later";

fn own(b: B, name: &str, desc: &str, tags: &[&str], want: &Want, hand: &[(u32, [f64; 2])]) -> Case {
    finish(
        b,
        SourceSet::Own,
        name,
        "docs/notes/sketch-solver.md; written for Amalgam",
        OWN_LICENCE,
        desc,
        tags,
        want,
        hand,
    )
}

fn fc(
    b: B,
    name: &str,
    scenario: &str,
    desc: &str,
    tags: &[&str],
    want: &Want,
    hand: &[(u32, [f64; 2])],
) -> Case {
    finish(
        b,
        SourceSet::PlaneGcs,
        name,
        &format!(
            "scenario after FreeCAD Sketcher tests ({scenario}); numbers authored for Amalgam"
        ),
        FC_LICENCE,
        desc,
        tags,
        want,
        hand,
    )
}

// ---- Small helpers ------------------------------------------------------------------------

/// Deterministic jitter in `[-1, 1)` (LCG), so generated start configurations are stable.
struct Jitter(u64);
impl Jitter {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    }
}

/// Rectangle `P0 P1 P2 P3` (counter-clockwise) with sides `L01 L12 L23 L30`, start
/// configuration offset by `d`; returns `([points], [lines])`.
fn rect_off(b: &mut B, x0: f64, y0: f64, w: f64, h: f64, d: [[f64; 2]; 4]) -> ([u32; 4], [u32; 4]) {
    let p0 = b.pt("p0", x0 + d[0][0], y0 + d[0][1]);
    let p1 = b.pt("p1", x0 + w + d[1][0], y0 + d[1][1]);
    let p2 = b.pt("p2", x0 + w + d[2][0], y0 + h + d[2][1]);
    let p3 = b.pt("p3", x0 + d[3][0], y0 + h + d[3][1]);
    let l = [
        b.line("l01", p0, p1),
        b.line("l12", p1, p2),
        b.line("l23", p2, p3),
        b.line("l30", p3, p0),
    ];
    ([p0, p1, p2, p3], l)
}

const NUDGE: [[f64; 2]; 4] = [[0.0, 0.0], [0.4, 0.3], [-0.3, 0.5], [0.3, -0.4]];

/// Full rectangle with H/V, fixed `p0`, and the two size dimensions.
fn dimensioned_rect(b: &mut B, w: f64, h: f64) -> ([u32; 4], [u32; 4]) {
    let (p, l) = rect_off(b, 0.0, 0.0, w, h, NUDGE);
    b.fix(p[0]);
    b.hz(l[0]);
    b.vt(l[1]);
    b.hz(l[2]);
    b.vt(l[3]);
    b.len(l[0], w);
    b.len(l[1], h);
    (p, l)
}

/// Slot (stadium) with two tangent arcs: centres `c1 = (0,0)`, `c2 = (len, 0)`, radius `r`.
/// Returns (builder entity ids) `[c1, c2, t1, t2, b2, b1, top, bottom, right_arc, left_arc]`.
/// Tangencies and the arc-orientation flags are set; no dimensions.
fn slot(b: &mut B, len: f64, r: f64) -> [u32; 10] {
    // Start configuration: slightly rotated/shifted, but each arc's ends stay at radius `r`.
    let at = |cx: f64, cy: f64, deg: f64| {
        (
            round12(cx + r * deg.to_radians().cos()),
            round12(cy + r * deg.to_radians().sin()),
        )
    };
    let c1 = b.pt("c1", 0.0, 0.0);
    let c2 = b.pt("c2", len + 0.3, 0.2);
    let (x, y) = at(0.0, 0.0, 97.0);
    let t1 = b.pt("t1", x, y);
    let (x, y) = at(len + 0.3, 0.2, 84.0);
    let t2 = b.pt("t2", x, y);
    let (x, y) = at(len + 0.3, 0.2, -86.0);
    let b2 = b.pt("b2", x, y);
    let (x, y) = at(0.0, 0.0, -95.0);
    let b1 = b.pt("b1", x, y);
    let top = b.line("top", t1, t2);
    let bottom = b.line("bottom", b2, b1);
    let right = b.arc("right", c2, b2, t2);
    let left = b.arc("left", c1, t1, b1);
    // Directions of travel at the shared ends are opposite on all four joints.
    b.tan_end(top, End::End, right, End::End, true);
    b.tan_end(right, End::Start, bottom, End::Start, true);
    b.tan_end(left, End::End, bottom, End::End, true);
    b.tan_end(top, End::Start, left, End::Start, true);
    [c1, c2, t1, t2, b2, b1, top, bottom, right, left]
}

// ---- Own cases ----------------------------------------------------------------------------

fn own_cases() -> Vec<Case> {
    let mut out = Vec::new();

    // D1: triangle, 6-8-10, one vertex fixed, base horizontal.
    {
        let mut b = B::new();
        let a = b.pt("a", 0.0, 0.0);
        let bb = b.pt("b", 10.3, 0.4);
        let c = b.pt("c", 3.4, 5.0);
        let ab = b.line("ab", a, bb);
        let bc = b.line("bc", bb, c);
        let ca = b.line("ca", c, a);
        b.fix(a);
        b.hz(ab);
        b.len(ab, 10.0);
        b.len(bc, 8.0);
        b.len(ca, 6.0);
        out.push(own(
            b,
            "d01_triangle",
            "D1: triangle with three length dimensions, one fixed vertex and a horizontal base (rigid cycle).",
            &["decision:D1"],
            &solved(0),
            &[(1, [10.0, 0.0]), (2, [3.6, 4.8])],
        ));
    }
    // D2: fully constrained rectangle.
    {
        let mut b = B::new();
        let (p, _) = dimensioned_rect(&mut b, 20.0, 10.0);
        out.push(own(
            b,
            "d02_rectangle",
            "D2: rectangle with H/V sides, two lengths and a fixed corner (substitution, DOF 0).",
            &["decision:D2"],
            &solved(0),
            &[
                (p[1], [20.0, 0.0]),
                (p[2], [20.0, 10.0]),
                (p[3], [0.0, 10.0]),
            ],
        ));
    }
    // D3: slot.
    {
        let mut b = B::new();
        let s = slot(&mut b, 20.0, 5.0);
        let [c1, c2, _, _, _, _, _, _, right, left] = s;
        b.fix(c1);
        b.hzp(c1, c2);
        b.dist(c1, c2, Side::Positive, 20.0);
        b.rad(right, 5.0);
        b.eq(left, right);
        out.push(own(
            b,
            "d03_slot",
            "D3: slot of two lines and two arcs joined by four endpoint tangencies; length and radius dimensions (tangency chirality).",
            &["decision:D3", "chirality"],
            &solved(0),
            &[(s[1], [20.0, 0.0]), (s[2], [0.0, 5.0]), (s[3], [20.0, 5.0])],
        ));
    }
    // D4: 50-segment polyline with a drag script on the middle vertex.
    {
        let mut b = B::new();
        let mut pts = Vec::new();
        for i in 0..=50 {
            let x = f64::from(i) * 2.0;
            let y = 3.0 * (f64::from(i) * 0.4).sin();
            pts.push(b.pt("", round12(x), round12(y)));
        }
        for w in pts.windows(2) {
            b.line("", w[0], w[1]);
        }
        let mid = pts[25];
        let (mx, my) = (50.0, 3.0 * (10.0_f64).sin());
        b.drag = Some(DragScript {
            entity: mid,
            path: (0..100)
                .map(|k| {
                    let t = f64::from(k) / 99.0;
                    [round12(mx + 12.0 * t), round12(my + 8.0 * (t * PI).sin())]
                })
                .collect(),
        });
        out.push(own(
            b,
            "d04_polyline50",
            "D4: 50-segment polyline whose lines share endpoints and nothing else; drag the middle vertex over 100 frames (under-constrained min-norm).",
            &["decision:D4", "drag"],
            &solved_start(102),
            &[],
        ));
    }
    // D5: rectangle + redundant parallel + redundant equal.
    {
        let mut b = B::new();
        let (p, l) = dimensioned_rect(&mut b, 20.0, 10.0);
        b.par(l[0], l[2]);
        b.eq(l[1], l[3]);
        let _ = p;
        out.push(own(
            b,
            "d05_rectangle_redundant",
            "D5: the D2 rectangle plus a parallel and an equal constraint that follow from the others (consistent redundancy).",
            &["decision:D5", "redundant"],
            &redundant(0),
            &[],
        ));
    }
    // D6: triangle 10-10-30.
    {
        let mut b = B::new();
        let a = b.pt("a", 0.0, 0.0);
        let bb = b.pt("b", 10.0, 0.0);
        let c = b.pt("c", 5.0, 5.0);
        let ab = b.line("ab", a, bb);
        let bc = b.line("bc", bb, c);
        let ca = b.line("ca", c, a);
        b.fix(a);
        b.hz(ab);
        b.len(ab, 10.0);
        b.len(bc, 10.0);
        b.len(ca, 30.0);
        out.push(own(
            b,
            "d06_triangle_impossible",
            "D6: triangle with sides 10, 10, 30, which violates the triangle inequality (no solution).",
            &["decision:D6", "conflict"],
            &Want::Unsolvable {
                status: DidNotConverge,
                also: &[Inconsistent],
            },
            &[],
        ));
    }
    // D7: four-bar linkage.
    {
        let mut b = B::new();
        let o1 = b.pt("o1", 0.0, 0.0);
        let o2 = b.pt("o2", 10.0, 0.0);
        let a = b.pt("a", 2.0, 3.4);
        let bb = b.pt("b", 9.0, 6.0);
        let crank = b.line("crank", o1, a);
        let coupler = b.line("coupler", a, bb);
        let rocker = b.line("rocker", o2, bb);
        b.fix(o1);
        b.fix(o2);
        b.len(crank, 4.0);
        b.len(coupler, 8.0);
        b.len(rocker, 6.0);
        // Cursor path: swing the coupler point around, through near-toggle positions.
        b.drag = Some(DragScript {
            entity: bb,
            path: (0..200)
                .map(|k| {
                    let t = f64::from(k) / 199.0;
                    let ang = 1.2 - 2.2 * t;
                    [round12(10.0 + 6.0 * ang.cos()), round12(6.0 * ang.sin())]
                })
                .collect(),
        });
        out.push(own(
            b,
            "d07_four_bar",
            "D7: four-bar linkage (ground fixed, crank 4, coupler 8, rocker 6); drag the coupler point through near-singular toggle positions (no chirality flip).",
            &["decision:D7", "drag", "chirality"],
            &solved(1),
            &[],
        ));
    }
    // D8: 24 holes on a bolt circle.
    {
        let mut b = B::new();
        let o = b.pt("o", 0.0, 0.0);
        let n = 24_usize;
        let mut cent = Vec::new();
        let mut circ = Vec::new();
        let mut spoke = Vec::new();
        let mut j = Jitter(8);
        for k in 0..n {
            let a = 2.0 * PI * k as f64 / n as f64;
            let (x, y) = (
                50.0 * a.cos() + 0.4 * j.next(),
                50.0 * a.sin() + 0.4 * j.next(),
            );
            cent.push(b.pt("", round12(x), round12(y)));
        }
        for &c in &cent {
            circ.push(b.circle("", c, 3.4));
        }
        for &c in &cent {
            spoke.push(b.cline("", o, c));
        }
        b.fix(o);
        b.hz(spoke[0]);
        for &c in &cent {
            b.dist(o, c, Side::Positive, 50.0);
        }
        for k in 0..n - 1 {
            b.ang(spoke[k], spoke[k + 1], 2.0 * PI / n as f64);
        }
        b.rad(circ[0], 3.0);
        for &c in &circ[1..] {
            b.eq(c, circ[0]);
        }
        out.push(own(
            b,
            "d08_bolt_circle",
            "D8 (reduced): 24 equal holes on a bolt circle of radius 50, spaced by 15 degree angle dimensions from a fixed centre (many identical blocks).",
            &["decision:D8", "large"],
            &solved(0),
            &[],
        ));
    }
    // D9: 10 x 10 grid of rectangles sharing edges.
    {
        let mut b = B::new();
        let mut j = Jitter(9);
        let n = 11_usize;
        let mut p = vec![vec![0_u32; n]; n];
        for (r, row) in p.iter_mut().enumerate() {
            for (c, cell) in row.iter_mut().enumerate() {
                // The fixed corner stays exactly at the origin.
                let k = if r + c == 0 { 0.0 } else { 0.5 };
                let x = 10.0 * c as f64 + k * j.next();
                let y = 10.0 * r as f64 + k * j.next();
                *cell = b.pt("", round12(x), round12(y));
            }
        }
        let mut hl = vec![vec![0_u32; n - 1]; n];
        let mut vl = vec![vec![0_u32; n]; n - 1];
        for r in 0..n {
            for c in 0..n - 1 {
                hl[r][c] = b.line("", p[r][c], p[r][c + 1]);
            }
        }
        for r in 0..n - 1 {
            for c in 0..n {
                vl[r][c] = b.line("", p[r][c], p[r + 1][c]);
            }
        }
        b.fix(p[0][0]);
        for row in &hl {
            for &l in row {
                b.hz(l);
            }
        }
        for row in &vl {
            for &l in row {
                b.vt(l);
            }
        }
        for &l in &hl[0] {
            b.len(l, 10.0);
        }
        for row in &vl {
            b.len(row[0], 10.0);
        }
        out.push(own(
            b,
            "d09_grid10x10",
            "D9: 10 x 10 grid of rectangles sharing edges: H/V on all 220 lines, the first row and column dimensioned, one corner fixed.",
            &["decision:D9", "large"],
            &solved(0),
            &[(p[10][10], [100.0, 100.0])],
        ));
    }
    // D10: 1e5 mm part with a 1e-3 mm feature.
    {
        let mut b = B::new();
        let o = b.pt("o", 0.0, 0.0);
        let a = b.pt("a", 100_000.4, 0.0004);
        let f = b.pt("f", 100_000.2, 0.0013);
        let oa = b.line("oa", o, a);
        b.fix(o);
        b.hz(oa);
        b.len(oa, 100_000.0);
        b.vtp(a, f);
        b.dist(a, f, Side::Positive, 1e-3);
        out.push(own(
            b,
            "d10_large_and_tiny",
            "D10: a 1e5 mm horizontal edge with a 1e-3 mm vertical feature at its end (conditioning, scaling, tolerance).",
            &["decision:D10", "scale"],
            &solved(0),
            &[(1, [100_000.0, 0.0]), (2, [100_000.0, 1e-3])],
        ));
    }
    // Further hand-checkable cases.
    {
        let mut b = B::new();
        b.pt("p", 3.0, 4.0);
        out.push(own(
            b,
            "free_point",
            "A single free point (DOF 2).",
            &["dof"],
            &solved_start(2),
            &[],
        ));
    }
    {
        let mut b = B::new();
        let c = b.pt("c", 1.0, 2.0);
        let k = b.circle("k", c, 4.5);
        b.fix(c);
        b.rad(k, 5.0);
        out.push(own(
            b,
            "circle_fixed_centre_radius",
            "Circle with fixed centre and a radius dimension (DOF 0).",
            &["dof"],
            &solved(0),
            &[],
        ));
    }
    {
        let mut b = B::new();
        let c = b.pt("c", 0.0, 0.0);
        let s = b.pt("s", 10.0, 0.0);
        let e = b.pt("e", 1.0, 10.0 * 0.99_f64.sqrt() + 0.0);
        let a = b.arc("a", c, s, e);
        b.fix(c);
        b.fix(s);
        b.arclen(a, 10.0 * PI / 2.0);
        out.push(own(
            b,
            "arc_length",
            "Quarter arc with fixed centre and start; the arc length dimension places the end point at (0, 10).",
            &["arc"],
            &solved(0),
            &[(e, [0.0, 10.0])],
        ));
    }
    {
        let mut b = B::new();
        let p = b.pt("p", -8.0, -5.0);
        let q = b.pt("q", 7.5, -4.6);
        let c = b.pt("c", 0.0, 0.0);
        let l = b.line("l", p, q);
        let k = b.circle("k", c, 4.8);
        b.fix(p);
        b.fix(c);
        b.rad(k, 5.0);
        b.tan(l, k, TangentKind::External);
        b.len(l, 16.0);
        out.push(own(
            b,
            "line_circle_tangent",
            "Line from a fixed point, tangent to a circle on its left, of length 16: the horizontal tangent y = -5.",
            &["tangent", "chirality"],
            &solved(0),
            &[(q, [8.0, -5.0])],
        ));
    }
    {
        let mut b = B::new();
        let c1 = b.pt("c1", 0.0, 0.0);
        let c2 = b.pt("c2", 0.4, -0.3);
        let k1 = b.circle("k1", c1, 4.0);
        let k2 = b.circle("k2", c2, 7.0);
        b.fix(c1);
        b.conc(k1, k2);
        b.rad(k1, 5.0);
        b.rad(k2, 8.0);
        out.push(own(
            b,
            "concentric_circles",
            "Two concentric circles with their own radii.",
            &["circle"],
            &solved(0),
            &[(c2, [0.0, 0.0])],
        ));
    }
    {
        let mut b = B::new();
        let a = b.pt("a", 0.0, 0.0);
        let bb = b.pt("b", 10.0, 0.0);
        let c = b.pt("c", 0.1, 0.1);
        let d = b.pt("d", 0.3, 9.7);
        let ab = b.line("ab", a, bb);
        let cd = b.line("cd", c, d);
        b.fix(a);
        b.fix(bb);
        b.coin(c, a);
        b.perp(ab, cd);
        b.eq(ab, cd);
        out.push(own(
            b,
            "perpendicular_equal_lines",
            "A second line from the same start, perpendicular to and as long as the first.",
            &["line"],
            &solved(0),
            &[(c, [0.0, 0.0]), (d, [0.0, 10.0])],
        ));
    }
    {
        let mut b = B::new();
        let a = b.pt("a", -4.0, 0.0);
        let bb = b.pt("b", 4.0, 0.0);
        let ab = b.line("ab", a, bb);
        let m = b.pt("m", 0.4, 0.3);
        let p = b.pt("p", 3.0, 5.0);
        let q = b.pt("q", 2.5, -4.5);
        b.fix(a);
        b.fix(bb);
        b.fix(p);
        b.mid(m, ab);
        b.sym(p, q, ab);
        out.push(own(
            b,
            "midpoint_symmetric",
            "A midpoint of a fixed line and the mirror image of a fixed point about it.",
            &["symmetric"],
            &solved(0),
            &[(m, [0.0, 0.0]), (q, [3.0, -5.0])],
        ));
    }
    {
        let mut b = B::new();
        let a = b.pt("a", 0.0, 0.0);
        let bb = b.pt("b", 10.0, 0.0);
        let c = b.pt("c", 12.0, 0.3);
        let d = b.pt("d", 20.0, -0.2);
        let l1 = b.line("l1", a, bb);
        let l2 = b.line("l2", c, d);
        b.fix(a);
        b.fix(bb);
        b.col(l1, l2);
        out.push(own(
            b,
            "collinear_lines",
            "A free line made collinear with a fixed one (2 rows, DOF 2).",
            &["line", "dof"],
            &solved(2),
            &[],
        ));
    }
    {
        let mut b = B::new();
        let a = b.pt("a", 0.0, 0.0);
        let bb = b.pt("b", 10.0, 0.0);
        let c = b.pt("c", 5.0, 3.3);
        let ab = b.line("ab", a, bb);
        let ac = b.line("ac", a, c);
        b.fix(a);
        b.fix(bb);
        b.ang(ab, ac, PI / 6.0);
        b.len(ac, 6.0);
        out.push(own(
            b,
            "angle_and_length",
            "A line at 30 degrees (counter-clockwise) to a fixed line, length 6.",
            &["angle"],
            &solved(0),
            &[(c, [6.0 * (PI / 6.0).cos(), 3.0])],
        ));
    }
    {
        let mut b = B::new();
        let p = b.pt("p", 0.0, 0.0);
        let q = b.pt("q", 6.0, -2.0);
        b.fix(p);
        b.hd(p, q, 7.0);
        b.vd(p, q, -3.0);
        out.push(own(
            b,
            "horizontal_vertical_distance",
            "Horizontal and vertical distance dimensions with a negative vertical side.",
            &["dimension"],
            &solved(0),
            &[(q, [7.0, -3.0])],
        ));
    }
    {
        let mut b = B::new();
        let (_, l) = dimensioned_rect(&mut b, 20.0, 10.0);
        b.len(l[2], 15.0);
        out.push(own(
            b,
            "conflicting_rectangle",
            "The D2 rectangle with a second, contradicting length on the opposite side.",
            &["conflict"],
            &Want::Unsolvable {
                status: Inconsistent,
                also: &[DidNotConverge],
            },
            &[],
        ));
    }
    {
        let mut b = B::new();
        let a = b.pt("a", 0.0, 0.0);
        let c = b.pt("b", 0.1, 0.1);
        b.coin(a, c);
        b.coin(c, a);
        out.push(own(
            b,
            "duplicate_coincident",
            "The same coincidence stated twice (redundant, DOF 2).",
            &["redundant"],
            &redundant(2),
            &[],
        ));
    }
    {
        let mut b = B::new();
        let c1 = b.pt("c1", 0.0, 0.0);
        let c2 = b.pt("c2", 7.6, 0.3);
        let k1 = b.circle("k1", c1, 5.0);
        let k2 = b.circle("k2", c2, 3.0);
        b.fix(c1);
        b.hzp(c1, c2);
        b.rad(k1, 5.0);
        b.rad(k2, 3.0);
        b.tan(k1, k2, TangentKind::External);
        out.push(own(
            b,
            "tangent_circles",
            "Two externally tangent circles with the second centre on the horizontal through the first.",
            &["tangent", "circle"],
            &solved(0),
            &[(c2, [8.0, 0.0])],
        ));
    }
    {
        let mut b = B::new();
        let mut pts = Vec::new();
        for (x, y) in [
            (0.0, 0.0),
            (5.0, 2.0),
            (9.0, 7.0),
            (14.0, 6.0),
            (19.0, 1.0),
            (24.0, 3.0),
        ] {
            pts.push(b.pt("", x, y));
        }
        for w in pts.windows(2) {
            b.line("", w[0], w[1]);
        }
        out.push(own(
            b,
            "polyline5",
            "Polyline of five segments sharing endpoints (DOF 12).",
            &["dof"],
            &solved_start(12),
            &[],
        ));
    }
    out
}

// ---- PlaneGCS-style cases -----------------------------------------------------------------

fn planegcs_cases() -> Vec<Case> {
    let mut out = Vec::new();
    let tags = |t: &'static str| -> [&'static str; 2] { ["freecad", t] };

    // Box helpers.
    {
        let mut b = B::new();
        let (_, l) = rect_off(&mut b, 0.0, 0.0, 30.0, 15.0, NUDGE);
        b.hz(l[0]);
        b.vt(l[1]);
        b.hz(l[2]);
        b.vt(l[3]);
        out.push(fc(
            b,
            "box_set",
            "TestSketcherSolver: box with four H/V constraints",
            "Four lines sharing corners with horizontal/vertical sides only: position and both sizes are free (DOF 4).",
            &tags("dof"),
            &solved(4),
            &[],
        ));
    }
    {
        let mut b = B::new();
        let (p, l) = rect_off(&mut b, 0.0, 0.0, 30.0, 15.0, NUDGE);
        b.fix(p[0]);
        b.hz(l[0]);
        b.vt(l[1]);
        b.hz(l[2]);
        b.vt(l[3]);
        b.hd(p[0], p[1], 30.0);
        b.vd(p[1], p[2], 15.0);
        out.push(fc(
            b,
            "box_fully_constrained",
            "TestSketcherSolver: fully constrained box",
            "Box with a fixed corner and horizontal/vertical distance dimensions (DOF 0).",
            &tags("dof"),
            &solved(0),
            &[(p[2], [30.0, 15.0]), (p[3], [0.0, 15.0])],
        ));
    }
    // Rectangle helper: three perpendiculars.
    {
        let mut b = B::new();
        let (p, l) = rect_off(&mut b, 0.0, 0.0, 12.0, 7.0, NUDGE);
        b.fix(p[0]);
        b.hz(l[0]);
        b.perp(l[0], l[1]);
        b.perp(l[1], l[2]);
        b.perp(l[2], l[3]);
        b.len(l[0], 12.0);
        b.len(l[1], 7.0);
        out.push(fc(
            b,
            "rectangle_helper",
            "TestSketcherSolver: rectangle helper (three right angles)",
            "Rectangle from three perpendicular constraints: the fourth right angle follows from closure.",
            &tags("rectangle"),
            &solved(0),
            &[(p[2], [12.0, 7.0])],
        ));
    }
    {
        let mut b = B::new();
        let (p, l) = rect_off(&mut b, 0.0, 0.0, 12.0, 7.0, NUDGE);
        b.fix(p[0]);
        b.hz(l[0]);
        b.perp(l[0], l[1]);
        b.perp(l[1], l[2]);
        b.perp(l[2], l[3]);
        b.perp(l[3], l[0]);
        b.len(l[0], 12.0);
        b.len(l[1], 7.0);
        out.push(fc(
            b,
            "rectangle_four_right_angles",
            "TestSketcherSolver: rectangle helper with a redundant fourth right angle",
            "As rectangle_helper but with all four perpendiculars: the last is redundant but consistent.",
            &tags("redundant"),
            &redundant(0),
            &[(p[2], [12.0, 7.0])],
        ));
    }
    // Square helper.
    {
        let mut b = B::new();
        let (p, l) = rect_off(&mut b, 0.0, 0.0, 10.0, 10.0, NUDGE);
        b.fix(p[0]);
        b.hz(l[0]);
        b.perp(l[0], l[1]);
        b.perp(l[1], l[2]);
        b.perp(l[2], l[3]);
        b.len(l[0], 10.0);
        b.eq(l[1], l[0]);
        out.push(fc(
            b,
            "square_helper",
            "TestSketcherSolver: square helper",
            "Square from perpendicular sides, one equal-length constraint and one length.",
            &tags("rectangle"),
            &solved(0),
            &[(p[2], [10.0, 10.0]), (p[3], [0.0, 10.0])],
        ));
    }
    // Circle helper.
    {
        let mut b = B::new();
        let c = b.pt("c", 2.0, -1.0);
        let q = b.pt("q", 0.3, 5.5);
        let k = b.circle("k", c, 5.0);
        b.fix(c);
        b.rad(k, 6.0);
        b.on(q, k);
        b.vtp(c, q);
        out.push(fc(
            b,
            "circle_helper",
            "TestSketcherSolver: circle with a point on it",
            "Circle with a fixed centre and radius and a point on it, vertically above the centre.",
            &tags("circle"),
            &solved(0),
            &[(q, [2.0, 5.0])],
        ));
    }
    // Slot plate.
    {
        let mut b = B::new();
        let s = slot(&mut b, 30.0, 6.0);
        let [c1, c2, _, _, _, _, _, _, right, left] = s;
        b.fix(c1);
        b.hzp(c1, c2);
        b.hd(c1, c2, 30.0);
        b.dia(right, 12.0);
        b.eq(left, right);
        out.push(fc(
            b,
            "slot_plate",
            "TestSketcherSolver: slot plate",
            "Slot made of two lines and two tangent arcs, dimensioned by centre distance and diameter.",
            &tags("slot"),
            &solved(0),
            &[(s[1], [30.0, 0.0]), (s[2], [0.0, 6.0])],
        ));
    }
    {
        let mut b = B::new();
        let s = slot(&mut b, 30.0, 6.0);
        let [c1, c2, _, _, _, _, _, _, right, left] = s;
        let ci = b.pt("ci", 0.5, 0.4);
        let hole = b.circle("hole", ci, 2.5);
        b.fix(c1);
        b.hzp(c1, c2);
        b.hd(c1, c2, 30.0);
        b.dia(right, 12.0);
        b.eq(left, right);
        b.conc(hole, left);
        b.dia(hole, 6.0);
        out.push(fc(
            b,
            "slot_plate_inner_hole",
            "TestSketcherSolver: slot plate with a mounting hole",
            "The slot plate plus a hole concentric with the left arc.",
            &tags("slot"),
            &solved(0),
            &[(ci, [0.0, 0.0])],
        ));
    }
    // issue3245: line - tangent arc - line fillets.
    for (name, down) in [
        ("issue3245_left_turn", false),
        ("issue3245_right_turn", true),
    ] {
        let mut b = B::new();
        let s = if down { -1.0 } else { 1.0 };
        let a = b.pt("a", 0.0, 0.0);
        let bb = b.pt("b", 10.2, 0.2);
        let c = b.pt("c", 10.0, 5.0 * s + 0.2);
        let e = b.pt("e", 15.0, 5.0 * s);
        let f = b.pt("f", 15.3, 15.0 * s);
        let l1 = b.line("l1", a, bb);
        let l2 = b.line("l2", e, f);
        // Left turn is a CCW arc b -> e; a right turn is the same arc traversed CCW e -> b.
        let arc = if down {
            b.arc("arc", c, e, bb)
        } else {
            b.arc("arc", c, bb, e)
        };
        b.fix(a);
        b.hz(l1);
        b.len(l1, 10.0);
        b.vt(l2);
        b.len(l2, 10.0);
        b.rad(arc, 5.0);
        if down {
            b.tan_end(l1, End::End, arc, End::End, true);
            b.tan_end(arc, End::Start, l2, End::Start, true);
        } else {
            b.tan_end(l1, End::End, arc, End::Start, false);
            b.tan_end(arc, End::End, l2, End::Start, false);
        }
        out.push(fc(
            b,
            name,
            "TestSketcherSolver: issue 3245 (tangent arc between two lines)",
            "Line, tangent fillet arc and line, one fixed end; both turn directions (arc orientation vs line direction).",
            &tags("tangent"),
            &solved(0),
            &[(e, [15.0, 5.0 * s]), (f, [15.0, 15.0 * s])],
        ));
    }
    // Missing H/V.
    {
        let mut b = B::new();
        let (p, l) = rect_off(&mut b, 0.0, 0.0, 20.0, 10.0, NUDGE);
        b.fix(p[0]);
        b.hz(l[0]);
        b.vt(l[1]);
        b.len(l[0], 20.0);
        b.len(l[1], 10.0);
        out.push(fc(
            b,
            "box_missing_hv",
            "TestSketcherSolver: box with missing H/V constraints",
            "Box where the last two sides lack their H/V constraints: the fourth corner stays free (DOF 2).",
            &tags("dof"),
            &solved(2),
            &[],
        ));
    }
    // Three lines.
    {
        let mut b = B::new();
        let a = b.pt("a", 0.0, 0.0);
        let bb = b.pt("b", 10.3, 0.3);
        let c = b.pt("c", 9.7, 5.2);
        let d = b.pt("d", 19.0, 5.3);
        let l1 = b.line("l1", a, bb);
        let l2 = b.line("l2", bb, c);
        let l3 = b.line("l3", c, d);
        b.fix(a);
        b.hz(l1);
        b.len(l1, 10.0);
        b.perp(l1, l2);
        b.len(l2, 5.0);
        b.par(l1, l3);
        b.len(l3, 10.0);
        out.push(fc(
            b,
            "three_lines_chain",
            "TestSketcherSolver: three lines (perpendicular, parallel, lengths)",
            "A Z-shaped chain of three lines with a horizontal first segment.",
            &tags("lines"),
            &solved(0),
            &[(bb, [10.0, 0.0]), (c, [10.0, 5.0]), (d, [20.0, 5.0])],
        ));
    }
    {
        let mut b = B::new();
        let a = b.pt("a", 0.0, 0.0);
        let bb = b.pt("b", 10.0, 0.0);
        let c = b.pt("c", 12.3, 0.2);
        let d = b.pt("d", 21.0, -0.3);
        let l1 = b.line("l1", a, bb);
        let l2 = b.line("l2", c, d);
        b.fix(a);
        b.fix(bb);
        b.col(l1, l2);
        b.eq(l1, l2);
        b.hd(bb, c, 2.0);
        out.push(fc(
            b,
            "three_lines_collinear",
            "TestSketcherSolver: collinear lines of equal length",
            "A second line collinear with and as long as a fixed one, starting 2 mm further along.",
            &tags("lines"),
            &solved(0),
            &[(c, [12.0, 0.0]), (d, [22.0, 0.0])],
        ));
    }
    // Circle - line distance family: the line y = 3 (fixed, oriented +x), circle centre above.
    for (name, scenario, side, dist, radius, target_y, start_y, driving) in [
        (
            "circle_line_distance",
            "circle-line distance",
            Side::Positive,
            2.0,
            5.0,
            10.0,
            9.4,
            true,
        ),
        (
            "circle_line_distance_driven",
            "circle-line distance, reference",
            Side::Positive,
            2.0,
            5.0,
            9.4,
            9.4,
            false,
        ),
        (
            "circle_line_distance_below",
            "circle-line distance, oriented",
            Side::Negative,
            2.0,
            5.0,
            -4.0,
            -3.5,
            true,
        ),
        (
            "circle_line_distance_r200",
            "circle-line distance, large radius",
            Side::Positive,
            3.0,
            200.0,
            206.0,
            205.0,
            true,
        ),
    ] {
        let mut b = B::new();
        let p = b.pt("p", -10.0, 3.0);
        let q = b.pt("q", 10.0, 3.0);
        let o = b.pt("o", 0.0, 0.0);
        let c = b.pt("c", 0.2, start_y);
        let l = b.line("l", p, q);
        let k = b.circle("k", c, radius - 0.3);
        b.fix(p);
        b.fix(q);
        b.fix(o);
        b.vtp(o, c);
        b.rad(k, radius);
        if driving {
            b.dist(l, k, side, dist);
        } else {
            b.dist_driven(l, k, side, dist);
        }
        let want = if driving { solved(0) } else { solved(1) };
        let hand: &[(u32, [f64; 2])] = if driving {
            &[(3, [0.0, target_y])]
        } else {
            &[]
        };
        out.push(fc(
            b,
            name,
            scenario,
            "Signed distance from a fixed oriented line to a circle edge (positive side: circle left of the line; the driven variant only measures).",
            &tags("distance"),
            &want,
            hand,
        ));
    }
    // Point - line signed distance.
    {
        let mut b = B::new();
        let p = b.pt("p", 0.0, 0.0);
        let q = b.pt("q", 10.0, 0.0);
        let x = b.pt("x", 0.2, 3.5);
        let l = b.line("l", p, q);
        b.fix(p);
        b.fix(q);
        b.vtp(p, x);
        b.dist(x, l, Side::Positive, 4.0);
        out.push(fc(
            b,
            "point_line_signed",
            "TestSketcherSolver: point-to-line distance, signed",
            "Point 4 mm left of an oriented line, on the vertical through its start.",
            &tags("distance"),
            &solved(0),
            &[(x, [0.0, 4.0])],
        ));
    }
    {
        let mut b = B::new();
        let p = b.pt("p", 0.0, 0.0);
        let q = b.pt("q", 10.0, 10.0);
        let x = b.pt("x", -139.0, 0.0);
        let l = b.line("l", p, q);
        b.fix(p);
        b.fix(q);
        b.hzp(p, x);
        b.dist(x, l, Side::Positive, 100.0);
        let t = -100.0 * 2.0_f64.sqrt();
        out.push(fc(
            b,
            "point_line_signed_100",
            "TestSketcherSolver: point-to-line distance, signed, large value",
            "Distance 100 on the left of a diagonal line, on the horizontal through its start.",
            &tags("distance"),
            &solved(0),
            &[(x, [round12(t), 0.0])],
        ));
    }
    // Circle - line tangent, oriented.
    {
        let mut b = B::new();
        let p = b.pt("p", -10.0, 5.0);
        let q = b.pt("q", 10.0, 5.0);
        let o = b.pt("o", 0.0, 0.0);
        let c = b.pt("c", 0.3, 1.5);
        let l = b.line("l", p, q);
        let k = b.circle("k", c, 3.5);
        b.fix(p);
        b.fix(q);
        b.fix(o);
        b.vtp(o, c);
        b.rad(k, 4.0);
        b.tan(l, k, TangentKind::Internal);
        out.push(fc(
            b,
            "circle_line_tangent_right",
            "TestSketcherSolver: circle-line tangent, oriented",
            "Circle on the right of an oriented line, tangent to it: the centre sits 4 below the line.",
            &tags("tangent"),
            &solved(0),
            &[(c, [0.0, 1.0])],
        ));
    }
    // Symmetric, redundant.
    {
        let mut b = B::new();
        let p = b.pt("p", 5.0, 0.0);
        let q = b.pt("q", 5.0, 10.0);
        let a = b.pt("a", 2.0, 3.0);
        let c = b.pt("c", 8.3, 3.2);
        let axis = b.line("axis", p, q);
        b.fix(p);
        b.fix(q);
        b.fix(a);
        b.sym(a, c, axis);
        b.hzp(a, c);
        out.push(fc(
            b,
            "symmetric_mirrored_redundant",
            "TestSketcherSolver: symmetric points about a vertical axis",
            "Mirror image across a vertical axis plus a horizontal constraint that the mirror already implies.",
            &tags("redundant"),
            &redundant(0),
            &[(c, [8.0, 3.0])],
        ));
    }
    // gtest-style point on arc and equal radius.
    {
        let mut b = B::new();
        let c = b.pt("c", 0.0, 0.0);
        let s = b.pt("s", 5.0, 0.0);
        let (ex, ey) = (
            round12(5.0 * 87.0_f64.to_radians().cos()),
            round12(5.0 * 87.0_f64.to_radians().sin()),
        );
        let e = b.pt("e", ex, ey);
        let x = b.pt("x", 0.2, 4.9);
        let arc = b.arc("arc", c, s, e);
        b.fix(c);
        b.fix(s);
        b.vtp(c, e);
        b.on(x, arc);
        b.vtp(c, x);
        out.push(fc(
            b,
            "point_on_arc",
            "planegcs gtest: point on arc",
            "A quarter arc (fixed centre and start, end vertically above the centre) and a point on it, also above the centre.",
            &tags("arc"),
            &solved(0),
            &[(e, [0.0, 5.0]), (x, [0.0, 5.0])],
        ));
    }
    {
        let mut b = B::new();
        let c = b.pt("c", 10.0, 0.0);
        let s = b.pt("s", 14.0, 3.0);
        let e = b.pt("e", 10.348782368720626, 4.987820251299121);
        let arc = b.arc("arc", c, s, e);
        let oc = b.pt("oc", -10.0, 0.0);
        let k = b.circle("k", oc, 6.0);
        b.fix(c);
        b.fix(s);
        b.vtp(c, e);
        b.fix(oc);
        b.eq(k, arc);
        out.push(fc(
            b,
            "equal_radius_arc_circle",
            "planegcs gtest: equal radius, arc and circle",
            "A circle forced to the radius (5) of an arc with fixed centre and start.",
            &tags("arc"),
            &solved(0),
            &[(e, [10.0, 5.0])],
        ));
    }
    out
}

// ---- Sidecars for the imported sets -------------------------------------------------------

fn corpus_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/sketch/corpus")
}

fn files_below(dir: &Path, suffix: &str) -> Vec<String> {
    fn rec(root: &Path, dir: &Path, suffix: &str, out: &mut Vec<String>) {
        for e in std::fs::read_dir(dir).unwrap() {
            let p = e.unwrap().path();
            if p.is_dir() {
                rec(root, &p, suffix, out);
            } else if p.to_string_lossy().ends_with(suffix) {
                out.push(
                    p.strip_prefix(root)
                        .unwrap()
                        .to_string_lossy()
                        .replace('\\', "/"),
                );
            }
        }
    }
    let mut out = Vec::new();
    rec(dir, dir, suffix, &mut out);
    out.sort();
    out
}

fn solvespace_sidecar(root: &Path) -> Sidecar {
    let dir = root.join("solvespace");
    let mut cases = Vec::new();
    for rel in files_below(&dir, ".slvs") {
        let stem = rel.trim_end_matches(".slvs");
        let c = corpus::slvs::import(
            &std::fs::read(dir.join(&rel)).unwrap(),
            &format!("solvespace/{stem}"),
            &rel,
        )
        .unwrap_or_else(|e| panic!("{rel}: {e}"));
        let mut tags = vec![stem.split('/').next().unwrap().to_string()];
        let v22 = stem.ends_with("_v22");
        if v22 {
            tags.push("format-v22".to_string());
        }
        let description = format!(
            "SolveSpace test/constraint/{rel}: {}{}",
            if stem.contains("reference") {
                "reference (driven) dimension"
            } else {
                "driving constraint"
            },
            if v22 {
                "; re-saved in file format v22"
            } else {
                ""
            }
        );
        // The files are saved solved: the check backend validates the stored geometry.
        let mut case_start = c.clone();
        case_start.expected.solution = Some(Solution::Start);
        let mut built = case_start.build().unwrap();
        let chk = ReferenceCheck::default()
            .solve(&case_start, &mut built)
            .unwrap();
        let expected = if chk.status.is_converged() {
            Expected {
                status: chk.status,
                also_accept: Vec::new(),
                dof: chk.dof,
                solution: Some(Solution::Start),
                tolerance: corpus::DEFAULT_SOLUTION_TOL,
                groups: Vec::new(),
            }
        } else {
            // Saved unsolved (large_dimensions): solve from the file's geometry instead.
            let mut built = c.build().unwrap();
            let (status, dof, worst, _) = ReferenceSolver::default()
                .solve_sketch(&mut built.sketch)
                .unwrap();
            assert!(
                status.is_converged() && worst < 1e-8,
                "{rel}: {status:?} {worst}"
            );
            Expected {
                status,
                also_accept: Vec::new(),
                dof: Some(dof),
                solution: None,
                tolerance: corpus::DEFAULT_SOLUTION_TOL,
                groups: Vec::new(),
            }
        };
        cases.push(SidecarEntry {
            files: vec![rel.clone()],
            description,
            tags,
            expected,
        });
    }
    Sidecar {
        licence: "GPL-3.0-or-later".into(),
        upstream: "https://github.com/solvespace/solvespace, commit 5bb74ad2, test/constraint/"
            .into(),
        cases,
    }
}

fn ezpz_sidecar(root: &Path) -> Sidecar {
    let dir = root.join("ezpz");
    let mut cases = Vec::new();
    for rel in files_below(&dir, "problem.md") {
        let name = rel.trim_end_matches("/problem.md");
        let c = corpus::ezpz::import(
            &std::fs::read_to_string(dir.join(&rel)).unwrap(),
            &format!("ezpz/{name}"),
            &rel,
        )
        .unwrap_or_else(|e| panic!("{rel}: {e}"));
        let mut built = c.build().unwrap();
        let (status, dof, worst, _) = ReferenceSolver::default()
            .solve_sketch(&mut built.sketch)
            .unwrap();
        let expected = if status.is_converged() {
            assert!(worst < 1e-8, "{rel}: residual {worst}");
            let mut values = built.values();
            for v in values.points.values_mut() {
                v[0] = round12(v[0]);
                v[1] = round12(v[1]);
            }
            for r in values.radii.values_mut() {
                *r = round12(*r);
            }
            Expected {
                status,
                also_accept: Vec::new(),
                dof: Some(dof),
                // A fully constrained case has isolated roots: the one reached from the
                // guesses is the target. Under-constrained results are solver dependent.
                solution: (dof == 0).then_some(Solution::Target(values)),
                tolerance: corpus::DEFAULT_SOLUTION_TOL,
                groups: Vec::new(),
            }
        } else {
            Expected {
                status,
                also_accept: if status == Inconsistent {
                    vec![DidNotConverge]
                } else {
                    vec![Inconsistent]
                },
                dof: None,
                solution: None,
                tolerance: corpus::DEFAULT_SOLUTION_TOL,
                groups: Vec::new(),
            }
        };
        cases.push(SidecarEntry {
            files: vec![rel.clone()],
            description: format!("ezpz test_cases/{name}"),
            tags: vec!["ezpz".into()],
            expected,
        });
    }
    Sidecar {
        licence: "MIT".into(),
        upstream: "https://github.com/KittyCAD/ezpz, commit 915882cc, test_cases/".into(),
        cases,
    }
}

// ---- Comparison / writing -----------------------------------------------------------------

fn json_close(a: &Value, b: &Value) -> bool {
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => match (x.as_f64(), y.as_f64()) {
            (Some(x), Some(y)) => (x - y).abs() <= 1e-9 * x.abs().max(y.abs()).max(1e-3),
            _ => x == y,
        },
        (Value::Array(x), Value::Array(y)) => {
            x.len() == y.len() && x.iter().zip(y).all(|(a, b)| json_close(a, b))
        }
        (Value::Object(x), Value::Object(y)) => {
            x.len() == y.len()
                && x.iter()
                    .all(|(k, v)| y.get(k).is_some_and(|w| json_close(v, w)))
        }
        _ => a == b,
    }
}

fn bless() -> bool {
    std::env::var("AMG_BLESS").is_ok_and(|v| !v.is_empty() && v != "0")
}

/// Writes `text` to `path` when blessing, otherwise requires the file to match it.
fn sync(path: &Path, text: &str) {
    if bless() {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, text).unwrap();
        return;
    }
    let on_disk = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("{}: {e} (run with AMG_BLESS=1)", path.display()));
    let (a, b): (Value, Value) = (
        serde_json::from_str(&on_disk).unwrap(),
        serde_json::from_str(text).unwrap(),
    );
    assert!(
        json_close(&a, &b),
        "{} is stale; regenerate with AMG_BLESS=1",
        path.display()
    );
}

fn sync_cases(set: SourceSet, cases: &[Case]) {
    let root = corpus_root();
    for case in cases {
        assert_eq!(case.source.set, set);
        let path = corpus::case_path(&root, case);
        sync(&path, &corpus::case_to_json(case).unwrap());
    }
    // No stale case files.
    if !bless() {
        let dir = root.join(set.dir());
        let want: Vec<String> = cases
            .iter()
            .map(|c| {
                format!(
                    "{}.json",
                    c.id.trim_start_matches(&format!("{}/", set.dir()))
                )
            })
            .collect();
        for f in files_below(&dir, ".json") {
            assert!(want.contains(&f), "{}/{f} is not generated", set.dir());
        }
    }
}

#[test]
fn corpus_gen_own() {
    sync_cases(SourceSet::Own, &own_cases());
}

#[test]
fn corpus_gen_planegcs() {
    sync_cases(SourceSet::PlaneGcs, &planegcs_cases());
}

fn sync_sidecar(set: &str, sc: &Sidecar) {
    let mut text = serde_json::to_string_pretty(sc).unwrap();
    text.push('\n');
    sync(&corpus_root().join(set).join("expected.json"), &text);
}

#[test]
fn corpus_gen_solvespace_sidecar() {
    sync_sidecar("solvespace", &solvespace_sidecar(&corpus_root()));
}

#[test]
fn corpus_gen_ezpz_sidecar() {
    sync_sidecar("ezpz", &ezpz_sidecar(&corpus_root()));
}
