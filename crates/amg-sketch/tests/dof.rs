//! SK-04: the 50 hand-made DOF cases (`testdata/sketch/dof/`, S03 §Testing) and the
//! `check_constraint` agreement test.
//!
//! Every case states its total DOF, the `ConstraintState` and `free_dof` of every entity and
//! the dependency groups (constraint indices, as sets) **by hand**; the derivations are in the
//! comments next to each case. The generator below is the source of truth for the checked-in
//! JSON files: the tests compare the files with it (JSON trees) and
//! `AMG_BLESS=1 cargo nextest run -p amg-sketch -E 'test(dof_cases_match_files)'` rewrites them.
//!
//! Conventions of the analysis (see `solve::native::analysis`): an entity's `free_dof` is the
//! number of independent ways its own and its points' parameters can move; entities referenced
//! by a `Redundant` or `Conflicting` group are `Over`; `Conflicting` groups are the constraints
//! whose removal alone makes the sketch solvable; DOF and states are generic (witness
//! configuration) when the given configuration is degenerate.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    clippy::too_many_lines,
    reason = "test helpers; a failure is a test failure"
)]

use std::collections::BTreeSet;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_6, PI};
use std::path::{Path, PathBuf};

use amg_sketch::corpus::{
    self, Case, CaseEntity, CaseGeom, CaseSource, CaseStatus, Expected, ExpectedGroup, GroupKind,
    SourceSet,
};
use amg_sketch::model::{AngleBranch, End, HvTarget, Side, TangentKind};
use amg_sketch::solve::{
    ConstraintCheck, ConstraintState, NativeSolver, SolveOptions, SolverBackend,
};
use amg_sketch::{ConstraintId, ConstraintKind as K, Dimension, DimensionKind as Dk, EntityId};
use serde::{Deserialize, Serialize};

// ---- File format ----------------------------------------------------------------------------

/// Expected status of one entity.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum St {
    Under,
    Full,
    Over,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Ent {
    state: St,
    free_dof: u8,
}

/// One DOF case: the corpus case (its `expected.groups` are the expected dependency groups),
/// the expected total DOF (also for non-converged cases, where the corpus format has none) and
/// the expected status of every entity, by index.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct DofCase {
    case: Case,
    dof: u32,
    entities: Vec<Ent>,
}

// ---- Builder --------------------------------------------------------------------------------

#[derive(Default)]
struct B {
    e: Vec<CaseEntity>,
    c: Vec<K>,
    st: Vec<Ent>,
}

fn id(i: u32) -> EntityId {
    EntityId(i)
}

use St::{Full, Over, Under};

impl B {
    fn push(&mut self, name: &str, geom: CaseGeom, construction: bool, st: (St, u8)) -> u32 {
        self.e.push(CaseEntity {
            name: name.to_string(),
            geom,
            construction,
        });
        self.st.push(Ent {
            state: st.0,
            free_dof: st.1,
        });
        u32::try_from(self.e.len() - 1).unwrap()
    }
    fn pt(&mut self, name: &str, x: f64, y: f64, st: (St, u8)) -> u32 {
        self.push(name, CaseGeom::Point { at: [x, y] }, false, st)
    }
    fn line(&mut self, name: &str, start: u32, end: u32, st: (St, u8)) -> u32 {
        self.push(name, CaseGeom::Line { start, end }, false, st)
    }
    fn cline(&mut self, name: &str, start: u32, end: u32, st: (St, u8)) -> u32 {
        self.push(name, CaseGeom::Line { start, end }, true, st)
    }
    fn circle(&mut self, name: &str, center: u32, radius: f64, st: (St, u8)) -> u32 {
        self.push(name, CaseGeom::Circle { center, radius }, false, st)
    }
    fn arc(&mut self, name: &str, center: u32, start: u32, end: u32, st: (St, u8)) -> u32 {
        self.push(name, CaseGeom::Arc { center, start, end }, false, st)
    }
    fn con(&mut self, k: K) -> u32 {
        self.c.push(k);
        u32::try_from(self.c.len() - 1).unwrap()
    }
    fn dim(&mut self, kind: Dk, value: f64) -> u32 {
        self.con(K::Dimension(Dimension {
            kind,
            value,
            driving: true,
            label_pos: amg_core::Point2::ZERO,
        }))
    }
    fn fix(&mut self, e: u32) -> u32 {
        self.con(K::Fix { entity: id(e) })
    }
    fn coin(&mut self, a: u32, b: u32) -> u32 {
        self.con(K::Coincident { a: id(a), b: id(b) })
    }
    fn hz(&mut self, l: u32) -> u32 {
        self.con(K::Horizontal(HvTarget::Line(id(l))))
    }
    fn vt(&mut self, l: u32) -> u32 {
        self.con(K::Vertical(HvTarget::Line(id(l))))
    }
    fn par(&mut self, a: u32, b: u32) -> u32 {
        self.con(K::Parallel { a: id(a), b: id(b) })
    }
    fn perp(&mut self, a: u32, b: u32) -> u32 {
        self.con(K::Perpendicular { a: id(a), b: id(b) })
    }
    fn col(&mut self, a: u32, b: u32) -> u32 {
        self.con(K::Collinear { a: id(a), b: id(b) })
    }
    fn conc(&mut self, a: u32, b: u32) -> u32 {
        self.con(K::Concentric { a: id(a), b: id(b) })
    }
    fn eq(&mut self, a: u32, b: u32) -> u32 {
        self.con(K::Equal { a: id(a), b: id(b) })
    }
    fn mid(&mut self, point: u32, line: u32) -> u32 {
        self.con(K::Midpoint {
            point: id(point),
            line: id(line),
        })
    }
    fn sym(&mut self, a: u32, b: u32, axis: u32) -> u32 {
        self.con(K::Symmetric {
            a: id(a),
            b: id(b),
            axis: id(axis),
        })
    }
    fn on(&mut self, point: u32, curve: u32) -> u32 {
        self.con(K::PointOnCurve {
            point: id(point),
            curve: id(curve),
        })
    }
    fn tan(&mut self, a: u32, b: u32, kind: TangentKind) -> u32 {
        self.con(K::Tangent {
            a: id(a),
            b: id(b),
            kind,
        })
    }
    fn tan_end(&mut self, a: u32, a_end: End, b: u32, b_end: End, reversed: bool) -> u32 {
        self.tan(
            a,
            b,
            TangentKind::AtEndpoint {
                a_end,
                b_end,
                reversed,
            },
        )
    }
    fn len(&mut self, line: u32, v: f64) -> u32 {
        self.dim(Dk::Length { line: id(line) }, v)
    }
    fn dist(&mut self, a: u32, b: u32, v: f64) -> u32 {
        self.dim(
            Dk::Distance {
                a: id(a),
                b: id(b),
                side: Side::Positive,
            },
            v,
        )
    }
    fn vd(&mut self, a: u32, b: u32, v: f64) -> u32 {
        self.dim(
            Dk::VerticalDistance {
                a: id(a),
                b: id(b),
                side: Side::Positive,
            },
            v,
        )
    }
    fn rad(&mut self, curve: u32, v: f64) -> u32 {
        self.dim(Dk::Radius { curve: id(curve) }, v)
    }
    fn arclen(&mut self, arc: u32, v: f64) -> u32 {
        self.dim(Dk::ArcLength { arc: id(arc) }, v)
    }
    fn ang(&mut self, a: u32, b: u32, v: f64) -> u32 {
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
        )
    }

    fn finish(
        self,
        name: &str,
        description: &str,
        status: CaseStatus,
        dof: u32,
        groups: &[(GroupKind, &[u32])],
    ) -> DofCase {
        let mut groups: Vec<ExpectedGroup> = groups
            .iter()
            .map(|(kind, cs)| {
                let mut constraints = cs.to_vec();
                constraints.sort_unstable();
                ExpectedGroup {
                    kind: *kind,
                    constraints,
                }
            })
            .collect();
        groups.sort();
        DofCase {
            case: Case {
                format: corpus::FORMAT_VERSION,
                id: format!("dof/{name}"),
                source: CaseSource {
                    set: SourceSet::Own,
                    origin: "docs/specs/S03-sketch.md §Testing (SK-04); written for Amalgam"
                        .to_string(),
                    licence: "LicenseRef-Amalgam-Personal".to_string(),
                },
                description: description.to_string(),
                tags: vec!["dof".to_string()],
                entities: self.e,
                constraints: self.c,
                expected: Expected {
                    status,
                    also_accept: Vec::new(),
                    dof: status.is_converged().then_some(dof),
                    solution: None,
                    tolerance: corpus::DEFAULT_SOLUTION_TOL,
                    groups,
                },
                drag: None,
            },
            dof,
            entities: self.st,
        }
    }
}

const U1: (St, u8) = (Under, 1);
const U2: (St, u8) = (Under, 2);
const U3: (St, u8) = (Under, 3);
const U4: (St, u8) = (Under, 4);
const F0: (St, u8) = (Full, 0);
const O0: (St, u8) = (Over, 0);

use CaseStatus::{Converged, ConvergedRedundant, Inconsistent};
use GroupKind::{Conflicting, DegenerateConfiguration as Degenerate, Redundant};

/// Rectangle `p0..p3` (20 × 10 at the origin), lines `l01, l12, l23, l30`.
fn rect(b: &mut B, pst: [(St, u8); 4], lst: [(St, u8); 4]) -> ([u32; 4], [u32; 4]) {
    let p0 = b.pt("p0", 0.0, 0.0, pst[0]);
    let p1 = b.pt("p1", 20.0, 0.0, pst[1]);
    let p2 = b.pt("p2", 20.0, 10.0, pst[2]);
    let p3 = b.pt("p3", 0.0, 10.0, pst[3]);
    let l = [
        b.line("l01", p0, p1, lst[0]),
        b.line("l12", p1, p2, lst[1]),
        b.line("l23", p2, p3, lst[2]),
        b.line("l30", p3, p0, lst[3]),
    ];
    ([p0, p1, p2, p3], l)
}

/// Rectangle with `Fix p0`, H/V on the four sides and lengths 20 / 10: constraints 0–6.
fn dim_rect(b: &mut B, lst: [(St, u8); 4]) -> ([u32; 4], [u32; 4]) {
    let (p, l) = rect(b, [F0; 4], lst);
    b.fix(p[0]);
    b.hz(l[0]);
    b.vt(l[1]);
    b.hz(l[2]);
    b.vt(l[3]);
    b.len(l[0], 20.0);
    b.len(l[1], 10.0);
    (p, l)
}

fn cases() -> Vec<DofCase> {
    let mut out = Vec::new();

    // ---- 10 under-constrained ----
    {
        let mut b = B::default();
        b.pt("p", 1.0, 2.0, U2);
        out.push(b.finish("u01_free_point", "A free point.", Converged, 2, &[]));
    }
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, U2);
        let p1 = b.pt("p1", 10.0, 3.0, U2);
        b.line("l", p0, p1, U4);
        out.push(b.finish("u02_free_line", "A free line.", Converged, 4, &[]));
    }
    {
        // H aliases the two y: the line keeps x0, x1, y (3); each point still moves 2 ways.
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 1.0, U2);
        let p1 = b.pt("p1", 10.0, 1.0, U2);
        let l = b.line("l", p0, p1, U3);
        b.hz(l);
        out.push(b.finish(
            "u03_line_horizontal",
            "A line with a horizontal constraint.",
            Converged,
            3,
            &[],
        ));
    }
    {
        let mut b = B::default();
        let c = b.pt("c", 1.0, 1.0, U2);
        b.circle("k", c, 4.0, U3);
        out.push(b.finish("u04_circle", "A free circle.", Converged, 3, &[]));
    }
    {
        // 6 point params − the arc's internal equation.
        let mut b = B::default();
        let c = b.pt("c", 0.0, 0.0, U2);
        let s = b.pt("s", 5.0, 0.0, U2);
        let e = b.pt("e", 0.0, 5.0, U2);
        b.arc("a", c, s, e, (Under, 5));
        out.push(b.finish("u05_arc", "A free arc.", Converged, 5, &[]));
    }
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, U2);
        let p1 = b.pt("p1", 10.0, 0.0, U2);
        let p2 = b.pt("p2", 12.0, 8.0, U2);
        b.line("l1", p0, p1, U4);
        b.line("l2", p1, p2, U4);
        out.push(b.finish(
            "u06_two_lines_shared_point",
            "Two lines sharing an end point.",
            Converged,
            6,
            &[],
        ));
    }
    {
        let mut b = B::default();
        let pts: Vec<u32> = (0..6)
            .map(|i| {
                b.pt(
                    "",
                    f64::from(i) * 5.0,
                    if i % 2 == 0 { 0.0 } else { 3.0 },
                    U2,
                )
            })
            .collect();
        for w in pts.windows(2) {
            b.line("", w[0], w[1], U4);
        }
        out.push(b.finish(
            "u07_polyline5",
            "A polyline of five segments.",
            Converged,
            12,
            &[],
        ));
    }
    {
        // 8 params − 4 H/V = 4 (x0, x1, y0, y1 of the corners); a side keeps 3 classes.
        let mut b = B::default();
        let (_, l) = rect(&mut b, [U2; 4], [U3; 4]);
        b.hz(l[0]);
        b.vt(l[1]);
        b.hz(l[2]);
        b.vt(l[3]);
        out.push(b.finish(
            "u08_rectangle_hv_only",
            "A rectangle with only H/V constraints.",
            Converged,
            4,
            &[],
        ));
    }
    {
        let mut b = B::default();
        let c = b.pt("c", 1.0, 1.0, F0);
        b.circle("k", c, 4.0, U1);
        b.fix(c);
        out.push(b.finish(
            "u09_circle_centre_fixed",
            "A circle with a fixed centre.",
            Converged,
            1,
            &[],
        ));
    }
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, U2);
        let p1 = b.pt("p1", 6.0, 8.0, U2);
        let l = b.line("l", p0, p1, U3);
        b.len(l, 10.0);
        out.push(b.finish(
            "u10_line_length_only",
            "A line with only a length.",
            Converged,
            3,
            &[],
        ));
    }

    // ---- 10 fully constrained ----
    {
        let mut b = B::default();
        let p = b.pt("p", 1.0, 2.0, F0);
        b.fix(p);
        out.push(b.finish("f01_fixed_point", "A fixed point.", Converged, 0, &[]));
    }
    {
        // 6 − 2 (fix) − 1 (H) − 3 lengths; the 6-8-10 triangle.
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 10.0, 0.0, F0);
        let p2 = b.pt("p2", 3.6, 4.8, F0);
        let l01 = b.line("l01", p0, p1, F0);
        let l12 = b.line("l12", p1, p2, F0);
        let l20 = b.line("l20", p2, p0, F0);
        b.fix(p0);
        b.hz(l01);
        b.len(l01, 10.0);
        b.len(l12, 8.0);
        b.len(l20, 6.0);
        out.push(b.finish(
            "f02_triangle",
            "Triangle with three lengths, a fixed vertex and a horizontal side.",
            Converged,
            0,
            &[],
        ));
    }
    {
        let mut b = B::default();
        dim_rect(&mut b, [F0; 4]);
        out.push(b.finish(
            "f03_rectangle",
            "Rectangle with H/V, two dimensions and a fixed corner.",
            Converged,
            0,
            &[],
        ));
    }
    {
        // Slot: 10 params − 2 internal − 4 tangencies − 1 equal = 5; fix (2), H (1),
        // centre distance (1), radius (1).
        let mut b = B::default();
        let c1 = b.pt("c1", 0.0, 0.0, F0);
        let c2 = b.pt("c2", 20.0, 0.0, F0);
        let t1 = b.pt("t1", 0.0, 5.0, F0);
        let t2 = b.pt("t2", 20.0, 5.0, F0);
        let b2 = b.pt("b2", 20.0, -5.0, F0);
        let b1 = b.pt("b1", 0.0, -5.0, F0);
        let top = b.line("top", t1, t2, F0);
        let bottom = b.line("bottom", b2, b1, F0);
        let right = b.arc("right", c2, b2, t2, F0);
        let left = b.arc("left", c1, t1, b1, F0);
        b.tan_end(top, End::End, right, End::End, true);
        b.tan_end(right, End::Start, bottom, End::Start, true);
        b.tan_end(left, End::End, bottom, End::End, true);
        b.tan_end(top, End::Start, left, End::Start, true);
        b.eq(left, right);
        b.fix(c1);
        b.con(K::Horizontal(HvTarget::Points(id(c1), id(c2))));
        b.dist(c1, c2, 20.0);
        b.rad(right, 5.0);
        out.push(b.finish(
            "f04_slot",
            "Slot with length and width dimensions, a fixed centre and H.",
            Converged,
            0,
            &[],
        ));
    }
    {
        let mut b = B::default();
        let c = b.pt("c", 1.0, 2.0, F0);
        let k = b.circle("k", c, 4.5, F0);
        b.fix(c);
        b.rad(k, 5.0);
        out.push(b.finish(
            "f05_circle",
            "Circle with a fixed centre and a radius.",
            Converged,
            0,
            &[],
        ));
    }
    {
        // s, e (4 params) − internal − radius − two angles against a fixed reference line.
        let mut b = B::default();
        let (s30, c30) = FRAC_PI_6.sin_cos();
        let (s120, c120) = (2.0 * PI / 3.0).sin_cos();
        let c = b.pt("c", 0.0, 0.0, F0);
        let s = b.pt("s", 5.0 * c30, 5.0 * s30, F0);
        let e = b.pt("e", 5.0 * c120, 5.0 * s120, F0);
        let a = b.arc("a", c, s, e, F0);
        let r0 = b.pt("r0", -10.0, -10.0, F0);
        let r1 = b.pt("r1", 10.0, -10.0, F0);
        let h = b.cline("ref", r0, r1, F0);
        let g1 = b.cline("g1", c, s, F0);
        let g2 = b.cline("g2", c, e, F0);
        b.fix(c);
        b.fix(h);
        b.rad(a, 5.0);
        b.ang(h, g1, FRAC_PI_6);
        b.ang(h, g2, 2.0 * PI / 3.0);
        out.push(b.finish(
            "f06_arc_angles",
            "Arc with fixed centre, radius and two angles to a fixed reference line.",
            Converged,
            0,
            &[],
        ));
    }
    {
        // 12 − 2 − 6 H/V − 4 lengths.
        let mut b = B::default();
        let xy = [
            (0.0, 0.0),
            (30.0, 0.0),
            (30.0, 10.0),
            (10.0, 10.0),
            (10.0, 30.0),
            (0.0, 30.0),
        ];
        let p: Vec<u32> = xy.iter().map(|&(x, y)| b.pt("", x, y, F0)).collect();
        let l: Vec<u32> = (0..6)
            .map(|i| b.line("", p[i], p[(i + 1) % 6], F0))
            .collect();
        b.fix(p[0]);
        for (i, &li) in l.iter().enumerate() {
            if i % 2 == 0 {
                b.hz(li);
            } else {
                b.vt(li);
            }
        }
        b.len(l[0], 30.0);
        b.len(l[1], 10.0);
        b.len(l[3], 20.0);
        b.len(l[4], 10.0);
        out.push(b.finish(
            "f07_l_bracket",
            "L-bracket of six lines with H/V, four lengths and a fixed corner.",
            Converged,
            0,
            &[],
        ));
    }
    {
        let mut b = B::default();
        let c1 = b.pt("c1", 0.0, 0.0, F0);
        let c2 = b.pt("c2", 0.0, 0.0, F0);
        let k1 = b.circle("k1", c1, 5.0, F0);
        let k2 = b.circle("k2", c2, 8.0, F0);
        b.fix(c1);
        b.conc(k1, k2);
        b.rad(k1, 5.0);
        b.rad(k2, 8.0);
        out.push(b.finish(
            "f08_concentric_radii",
            "Concentric circles with radii and a fixed centre.",
            Converged,
            0,
            &[],
        ));
    }
    {
        // a, b, c, d (8) − 2 symmetries (4) − 4 dimensions; the axis line is fixed.
        let mut b = B::default();
        let q0 = b.pt("q0", 0.0, 0.0, F0);
        let q1 = b.pt("q1", 0.0, 10.0, F0);
        let axis = b.cline("axis", q0, q1, F0);
        let pa = b.pt("a", -6.0, 0.0, F0);
        let pb = b.pt("b", 6.0, 0.0, F0);
        let pc = b.pt("c", 3.0, 5.0, F0);
        let pd = b.pt("d", -3.0, 5.0, F0);
        let ab = b.line("ab", pa, pb, F0);
        b.line("bc", pb, pc, F0);
        let cd = b.line("cd", pc, pd, F0);
        b.line("da", pd, pa, F0);
        b.fix(axis);
        b.sym(pa, pb, axis);
        b.sym(pd, pc, axis);
        b.len(ab, 12.0);
        b.len(cd, 6.0);
        b.vd(q0, pa, 0.0);
        b.vd(pa, pd, 5.0);
        out.push(b.finish(
            "f09_symmetric_trapezoid",
            "Trapezoid symmetric about a fixed axis, with two widths and two heights.",
            Converged,
            0,
            &[],
        ));
    }
    {
        // 7 points (14) − 3 internal − 2 tangencies − fix c1, s1 (4) − 5 dimensions.
        let mut b = B::default();
        let c1 = b.pt("c1", 0.0, 0.0, F0);
        let s1 = b.pt("s1", 5.0, 0.0, F0);
        let j1 = b.pt("j1", 0.0, 5.0, F0);
        let c2 = b.pt("c2", 0.0, 2.0, F0);
        let j2 = b.pt("j2", -3.0, 2.0, F0);
        let c3 = b.pt("c3", -1.0, 2.0, F0);
        let e3 = b.pt("e3", -1.0, 0.0, F0);
        let a1 = b.arc("a1", c1, s1, j1, F0);
        let a2 = b.arc("a2", c2, j1, j2, F0);
        let a3 = b.arc("a3", c3, j2, e3, F0);
        b.fix(c1);
        b.fix(s1);
        b.tan_end(a1, End::End, a2, End::Start, false);
        b.tan_end(a2, End::End, a3, End::Start, false);
        b.arclen(a1, 5.0 * FRAC_PI_2);
        b.rad(a2, 3.0);
        b.arclen(a2, 3.0 * FRAC_PI_2);
        b.rad(a3, 2.0);
        b.arclen(a3, 2.0 * FRAC_PI_2);
        out.push(b.finish(
            "f10_tangent_arc_chain",
            "Chain of three arcs tangent at shared end points, fully dimensioned.",
            Converged,
            0,
            &[],
        ));
    }

    // ---- 10 redundant (consistent) ----
    {
        // Parallel(l01, l23) = −H01 − H23 at the rectangle.
        let mut b = B::default();
        let (_, l) = dim_rect(&mut b, [O0, F0, O0, F0]);
        let p = b.par(l[0], l[2]);
        out.push(b.finish(
            "r01_rectangle_extra_parallel",
            "Rectangle plus a parallel that follows from H/H.",
            ConvergedRedundant,
            0,
            &[(Redundant, &[1, 3, p])],
        ));
    }
    {
        // Perpendicular(l30, l01) is a combination of V30 and H01.
        let mut b = B::default();
        let (_, l) = dim_rect(&mut b, [O0, F0, F0, O0]);
        let p = b.perp(l[3], l[0]);
        out.push(b.finish(
            "r02_rectangle_fourth_perpendicular",
            "Rectangle plus a perpendicular that follows from H/V.",
            ConvergedRedundant,
            0,
            &[(Redundant, &[1, 4, p])],
        ));
    }
    {
        // Midpoint(m, ab) + m on the axis imply the "midpoint on axis" row of Symmetric(a, b).
        // Unknowns a, b, m (6) − rank 4: a moves freely (b mirrors, m follows).
        let mut b = B::default();
        let q0 = b.pt("q0", 0.0, 0.0, F0);
        let q1 = b.pt("q1", 0.0, 10.0, F0);
        let axis = b.cline("axis", q0, q1, (Over, 0));
        let pa = b.pt("a", -3.0, 4.0, (Over, 2));
        let pb = b.pt("b", 3.0, 4.0, (Over, 2));
        // m = (a + b)/2 stays on the axis: it only moves along it.
        let m = b.pt("m", 0.0, 4.0, (Over, 1));
        let ab = b.line("ab", pa, pb, (Over, 2));
        b.fix(axis);
        let s = b.sym(pa, pb, axis);
        let mi = b.mid(m, ab);
        let o = b.on(m, axis);
        out.push(b.finish(
            "r03_midpoint_and_symmetric",
            "Midpoint of a mirrored pair on the mirror axis, plus the symmetry.",
            ConvergedRedundant,
            2,
            &[(Redundant, &[s, mi, o])],
        ));
    }
    {
        let mut b = B::default();
        let h = 5.0 * 3.0_f64.sqrt();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 10.0, 0.0, F0);
        let p2 = b.pt("p2", 5.0, h, F0);
        let l01 = b.line("l01", p0, p1, O0);
        let l12 = b.line("l12", p1, p2, O0);
        let l20 = b.line("l20", p2, p0, O0);
        b.fix(p0);
        b.hz(l01);
        b.len(l01, 10.0);
        b.len(l12, 10.0);
        b.len(l20, 10.0);
        b.eq(l01, l12);
        b.eq(l12, l20);
        b.eq(l20, l01);
        out.push(b.finish(
            "r04_equal_triangle_with_lengths",
            "Equilateral triangle with three lengths and three equal constraints.",
            ConvergedRedundant,
            0,
            &[
                (Redundant, &[2, 3, 5]),
                (Redundant, &[3, 4, 6]),
                (Redundant, &[2, 4, 7]),
            ],
        ));
    }
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, U2);
        let p1 = b.pt("p1", 5.0, 0.0, (Over, 2));
        let p2 = b.pt("p2", 5.0, 0.0, (Over, 2));
        let p3 = b.pt("p3", 5.0, 5.0, U2);
        b.line("l1", p0, p1, U4);
        b.line("l2", p2, p3, U4);
        b.coin(p1, p2);
        b.coin(p1, p2);
        out.push(b.finish(
            "r05_double_coincident",
            "Two lines joined by the same coincidence twice.",
            ConvergedRedundant,
            6,
            &[(Redundant, &[0, 1])],
        ));
    }
    {
        // H(l2) = Col_1 − Col_2 − ((x2 − x3)/L) H(l1); unknowns 8 − rank 3.
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, U2);
        let p1 = b.pt("p1", 10.0, 0.0, U2);
        let p2 = b.pt("p2", 15.0, 0.0, U2);
        let p3 = b.pt("p3", 25.0, 0.0, U2);
        let l1 = b.line("l1", p0, p1, (Over, 3));
        let l2 = b.line("l2", p2, p3, (Over, 3));
        b.col(l1, l2);
        b.hz(l1);
        b.hz(l2);
        out.push(b.finish(
            "r06_collinear_pair_both_horizontal",
            "Collinear lines with a horizontal constraint on each.",
            ConvergedRedundant,
            5,
            &[(Redundant, &[0, 1, 2])],
        ));
    }
    {
        let mut b = B::default();
        let c1 = b.pt("c1", 1.0, 1.0, (Over, 2));
        let c2 = b.pt("c2", 1.0, 1.0, (Over, 2));
        b.circle("k1", c1, 3.0, (Over, 3));
        b.circle("k2", c2, 5.0, (Over, 3));
        b.conc(2, 3);
        b.coin(c1, c2);
        out.push(b.finish(
            "r07_concentric_and_coincident_centres",
            "Concentric circles whose centres are also coincident.",
            ConvergedRedundant,
            4,
            &[(Redundant, &[0, 1])],
        ));
    }
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 6.0, 8.0, F0);
        let l = b.line("l", p0, p1, O0);
        b.fix(l);
        b.len(l, 10.0);
        out.push(b.finish(
            "r08_fix_and_dimension",
            "A fixed line with a (consistent) length.",
            ConvergedRedundant,
            0,
            &[(Redundant, &[0, 1])],
        ));
    }
    {
        // 10 params − internal − coincidence (2) − tangency; both duplicated.
        let mut b = B::default();
        let p0 = b.pt("p0", -10.0, 5.0, U2);
        let p1 = b.pt("p1", 0.0, 5.0, (Over, 2));
        let c = b.pt("c", 0.0, 0.0, U2);
        let s = b.pt("s", 0.0, 5.0, (Over, 2));
        let e = b.pt("e", -5.0, 0.0, U2);
        let l = b.line("l", p0, p1, (Over, 4));
        let a = b.arc("a", c, s, e, (Over, 5));
        b.coin(p1, s);
        b.tan_end(l, End::End, a, End::Start, true);
        b.coin(p1, s);
        b.tan_end(l, End::End, a, End::Start, true);
        out.push(b.finish(
            "r09_tangent_and_coincident_duplicated",
            "Line and arc joined by a coincidence and an endpoint tangency, both added twice.",
            ConvergedRedundant,
            6,
            &[(Redundant, &[0, 2]), (Redundant, &[1, 3])],
        ));
    }
    {
        // The second length was driven and has been toggled to driving.
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 10.0, 0.0, F0);
        let l = b.line("l", p0, p1, O0);
        b.fix(p0);
        b.hz(l);
        b.len(l, 10.0);
        b.len(l, 10.0);
        out.push(b.finish(
            "r10_driven_toggled_duplicate",
            "A length duplicated by a driven dimension toggled to driving.",
            ConvergedRedundant,
            0,
            &[(Redundant, &[2, 3])],
        ));
    }

    // ---- 10 conflicting ----
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, O0);
        let p1 = b.pt("p1", 11.0, 0.0, (Over, 1));
        b.fix(p0);
        b.dist(p0, p1, 10.0);
        b.dist(p0, p1, 12.0);
        out.push(b.finish(
            "c01_two_distances",
            "Two different distances between the same points.",
            Inconsistent,
            1,
            &[(Conflicting, &[1, 2])],
        ));
    }
    {
        // D6; generic (witness) rank 3, so DOF 0 and the vertices are determined.
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 10.0, 0.0, F0);
        let p2 = b.pt("p2", 5.0, 4.0, F0);
        let l01 = b.line("l01", p0, p1, O0);
        let l12 = b.line("l12", p1, p2, O0);
        let l20 = b.line("l20", p2, p0, O0);
        b.fix(p0);
        b.hz(l01);
        b.len(l01, 10.0);
        b.len(l12, 10.0);
        b.len(l20, 30.0);
        out.push(b.finish(
            "c02_triangle_inequality",
            "Triangle with sides 10, 10, 30.",
            Inconsistent,
            0,
            &[(Conflicting, &[2, 3, 4])],
        ));
    }
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 10.0, 0.0, F0);
        let l = b.line("l", p0, p1, O0);
        b.fix(p0);
        b.hz(l);
        b.vt(l);
        b.len(l, 10.0);
        out.push(b.finish(
            "c03_horizontal_and_vertical",
            "Horizontal and vertical on a line of length 10.",
            Inconsistent,
            0,
            &[(Conflicting, &[1, 2, 3])],
        ));
    }
    {
        // sin θ and cos θ cannot both vanish: every direction is a least-squares point, both
        // rows only see the direction of l2, so l2 keeps its two translations.
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 10.0, 0.0, F0);
        let (s, c) = FRAC_PI_6.sin_cos();
        let p2 = b.pt("p2", 0.0, 5.0, U2);
        let p3 = b.pt("p3", 10.0 * c, 5.0 + 10.0 * s, U2);
        let l1 = b.line("l1", p0, p1, O0);
        let l2 = b.line("l2", p2, p3, (Over, 2));
        b.fix(l1);
        b.len(l2, 10.0);
        b.par(l1, l2);
        b.perp(l1, l2);
        out.push(b.finish(
            "c04_perpendicular_and_parallel",
            "Perpendicular and parallel on the same pair of lines.",
            Inconsistent,
            2,
            &[(Conflicting, &[2, 3])],
        ));
    }
    {
        let mut b = B::default();
        let c = b.pt("c", 0.0, 0.0, O0);
        let k = b.circle("k", c, 6.0, O0);
        let q = b.pt("q", 7.0, 0.0, O0);
        b.fix(c);
        b.fix(q);
        b.rad(k, 5.0);
        b.on(q, k);
        out.push(b.finish(
            "c05_radius_vs_fixed_rim_point",
            "Radius 5 but a fixed rim point at distance 7.",
            Inconsistent,
            0,
            &[(Conflicting, &[0, 1, 2, 3])],
        ));
    }
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, O0);
        let p1 = b.pt("p1", 10.0, 0.0, O0);
        let l = b.line("l", p0, p1, O0);
        b.fix(p0);
        b.fix(p1);
        b.len(l, 12.0);
        out.push(b.finish(
            "c06_length_vs_fixed_ends",
            "Length 12 between fixed end points 10 apart.",
            Inconsistent,
            0,
            &[(Conflicting, &[0, 1, 2])],
        ));
    }
    {
        // Angle and parallel both only see the direction of l2: rank 1 of 4.
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 10.0, 0.0, F0);
        let p2 = b.pt("p2", 0.0, 5.0, U2);
        let p3 = b.pt("p3", 10.0, 7.0, U2);
        let l1 = b.line("l1", p0, p1, O0);
        let l2 = b.line("l2", p2, p3, (Over, 3));
        b.fix(l1);
        b.ang(l1, l2, FRAC_PI_6);
        b.par(l1, l2);
        out.push(b.finish(
            "c07_angle_and_parallel",
            "A 30° angle and a parallel on the same pair.",
            Inconsistent,
            3,
            &[(Conflicting, &[1, 2])],
        ));
    }
    {
        // Equal radii are substituted: one radius class with two radius rows.
        let mut b = B::default();
        let c1 = b.pt("c1", 0.0, 0.0, U2);
        let c2 = b.pt("c2", 20.0, 0.0, U2);
        let k1 = b.circle("k1", c1, 6.0, (Over, 2));
        let k2 = b.circle("k2", c2, 6.0, (Over, 2));
        b.eq(k1, k2);
        b.rad(k1, 5.0);
        b.rad(k2, 7.0);
        out.push(b.finish(
            "c08_equal_radius_different_dims",
            "Equal radii with radius dimensions 5 and 7.",
            Inconsistent,
            4,
            &[(Conflicting, &[0, 1, 2])],
        ));
    }
    {
        // Removing any of the five makes it solvable; generic rank 3 (witness).
        let mut b = B::default();
        let c1 = b.pt("c1", 0.0, 0.0, F0);
        let c2 = b.pt("c2", 30.0, 0.0, F0);
        let c3 = b.pt("c3", 15.0, 3.0, F0);
        let k1 = b.circle("k1", c1, 2.0, O0);
        let k2 = b.circle("k2", c2, 2.0, O0);
        let k3 = b.circle("k3", c3, 3.0, O0);
        b.fix(k1);
        b.fix(k2);
        b.rad(k3, 3.0);
        b.tan(k3, k1, TangentKind::External);
        b.tan(k3, k2, TangentKind::External);
        out.push(b.finish(
            "c09_tangent_unreachable_circles",
            "A radius-3 circle tangent to two fixed circles 30 apart.",
            Inconsistent,
            0,
            &[(Conflicting, &[0, 1, 2, 3, 4])],
        ));
    }
    {
        let mut b = B::default();
        let q0 = b.pt("q0", 0.0, 0.0, F0);
        let q1 = b.pt("q1", 0.0, 10.0, F0);
        let axis = b.cline("axis", q0, q1, O0);
        let pa = b.pt("a", -3.0, 4.0, O0);
        let pb = b.pt("b", 5.0, 4.0, O0);
        b.fix(axis);
        b.fix(pa);
        b.fix(pb);
        b.sym(pa, pb, axis);
        out.push(b.finish(
            "c10_symmetric_off_axis_fixed",
            "Symmetry of two fixed points that are not mirror images.",
            Inconsistent,
            0,
            &[(Conflicting, &[0, 1, 2, 3])],
        ));
    }

    // ---- 5 mixed ----
    {
        let mut b = B::default();
        let (p, _) = dim_rect(&mut b, [F0; 4]);
        let q0 = b.pt("q0", 20.0, 10.0, F0);
        let q1 = b.pt("q1", 30.0, 15.0, U2);
        b.line("tail", q0, q1, U2);
        b.coin(q0, p[2]);
        out.push(b.finish(
            "m01_rectangle_with_free_tail",
            "Fully constrained rectangle with a free line hanging off a corner.",
            Converged,
            2,
            &[],
        ));
    }
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 10.0, 0.0, F0);
        let p2 = b.pt("p2", 3.6, 4.8, F0);
        let l01 = b.line("l01", p0, p1, F0);
        let l12 = b.line("l12", p1, p2, F0);
        let l20 = b.line("l20", p2, p0, F0);
        let c = b.pt("c", 3.6, 4.8, F0);
        b.circle("k", c, 2.0, U1);
        b.fix(p0);
        b.hz(l01);
        b.len(l01, 10.0);
        b.len(l12, 8.0);
        b.len(l20, 6.0);
        b.coin(c, p2);
        out.push(b.finish(
            "m02_triangle_with_circle",
            "Fully constrained triangle with a circle centred on a vertex (free radius).",
            Converged,
            1,
            &[],
        ));
    }
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 6.0, 8.0, U1);
        let l = b.line("l", p0, p1, U1);
        let c = b.pt("c", 30.0, 0.0, F0);
        let k = b.circle("k", c, 3.0, F0);
        b.fix(p0);
        b.len(l, 10.0);
        b.fix(k);
        out.push(b.finish(
            "m03_rotating_line_and_fixed_circle",
            "A line rotating about a fixed point next to a fixed circle.",
            Converged,
            1,
            &[],
        ));
    }
    {
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 10.0, 0.0, U1);
        let l = b.line("l", p0, p1, U1);
        let m = b.pt("m", 5.0, 0.0, U1);
        b.fix(p0);
        b.hz(l);
        b.mid(m, l);
        out.push(b.finish(
            "m04_horizontal_line_with_midpoint",
            "A horizontal line from a fixed point with its midpoint.",
            Converged,
            1,
            &[],
        ));
    }
    {
        let mut b = B::default();
        let (p, l) = rect(&mut b, [F0, F0, U1, U1], [F0, U1, U1, U1]);
        b.fix(p[0]);
        b.hz(l[0]);
        b.vt(l[1]);
        b.hz(l[2]);
        b.vt(l[3]);
        b.len(l[0], 20.0);
        out.push(b.finish(
            "m05_rectangle_free_height",
            "Rectangle with a fixed corner and a width but no height.",
            Converged,
            1,
            &[],
        ));
    }

    // ---- 5 degenerate / witness ----
    {
        // 10 = 4 + 6: the satisfied configuration is collinear (rank 2 there); a generic
        // triangle with these constraint types has rank 3.
        let mut b = B::default();
        let p0 = b.pt("p0", 0.0, 0.0, F0);
        let p1 = b.pt("p1", 10.0, 0.0, F0);
        let p2 = b.pt("p2", 6.0, 0.0, F0);
        let l01 = b.line("l01", p0, p1, F0);
        let l12 = b.line("l12", p1, p2, F0);
        let l20 = b.line("l20", p2, p0, F0);
        b.fix(p0);
        b.hz(l01);
        b.len(l01, 10.0);
        b.len(l12, 4.0);
        b.len(l20, 6.0);
        out.push(b.finish(
            "w01_collinear_triangle",
            "Triangle 10 / 4 / 6: satisfied only collinear; generically well-constrained.",
            Converged,
            0,
            &[(Degenerate, &[2, 3, 4])],
        ));
    }
    {
        // A zero-length line parallel to a fixed one: the parallel row vanishes here.
        let mut b = B::default();
        let r0 = b.pt("r0", 0.0, 0.0, F0);
        let r1 = b.pt("r1", 10.0, 0.0, F0);
        let m = b.line("m", r0, r1, F0);
        let p0 = b.pt("p0", 2.0, 3.0, U2);
        let p1 = b.pt("p1", 2.0, 3.0, U2);
        let l = b.line("l", p0, p1, U3);
        b.fix(m);
        b.par(l, m);
        out.push(b.finish(
            "w02_zero_length_line",
            "A zero-length line parallel to a fixed line.",
            Converged,
            3,
            &[(Degenerate, &[1])],
        ));
    }
    {
        // Distance 0 between coincident centres: the row vanishes here; generically it keeps
        // the centre on a circle about c1 (1 of 2) and leaves the radius free.
        let mut b = B::default();
        let c1 = b.pt("c1", 0.0, 0.0, F0);
        let c2 = b.pt("c2", 0.0, 0.0, U1);
        let k1 = b.circle("k1", c1, 5.0, F0);
        b.circle("k2", c2, 3.0, U2);
        b.fix(k1);
        b.dist(c1, c2, 0.0);
        out.push(b.finish(
            "w03_coincident_centres",
            "Two circles with coincident centres held by a zero distance (before concentric).",
            Converged,
            2,
            &[(Degenerate, &[1])],
        ));
    }
    {
        let mut b = B::default();
        let c = b.pt("c", 0.0, 0.0, F0);
        let s = b.pt("s", 5.0, 0.0, U1);
        let e = b.pt("e", 5.0, 0.0, U1);
        let a = b.arc("a", c, s, e, U2);
        b.fix(c);
        b.rad(a, 5.0);
        out.push(b.finish(
            "w04_arc_start_equals_end",
            "An arc whose start and end coincide, with fixed centre and radius.",
            Converged,
            2,
            &[],
        ));
    }
    {
        // Crank and coupler collinear (toggle); the ground is not on that line, so the rank
        // stays 3 and the mechanism keeps its single DOF. To first order `b` cannot move: the
        // crank and coupler rows give δb ⊥ a and the rocker row δb ⊥ (b − o2), so `b` and the
        // rocker are `Full` at this dead point (only `a` moves).
        let mut b = B::default();
        let by = 35.84_f64.sqrt();
        let o1 = b.pt("o1", 0.0, 0.0, F0);
        let o2 = b.pt("o2", 10.0, 0.0, F0);
        let pa = b.pt("a", 10.4 / 3.0, by / 3.0, U1);
        let pb = b.pt("b", 10.4, by, F0);
        let crank = b.line("crank", o1, pa, U1);
        let coupler = b.line("coupler", pa, pb, U1);
        let rocker = b.line("rocker", o2, pb, F0);
        b.fix(o1);
        b.fix(o2);
        b.len(crank, 4.0);
        b.len(coupler, 8.0);
        b.len(rocker, 6.0);
        out.push(b.finish(
            "w05_four_bar_toggle",
            "Four-bar linkage at the crank–coupler toggle.",
            Converged,
            1,
            &[],
        ));
    }
    out
}

// ---- Files ----------------------------------------------------------------------------------

fn dof_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/sketch/dof")
}

fn file_name(c: &DofCase) -> PathBuf {
    let name = c.case.id.trim_start_matches("dof/");
    dof_dir().join(format!("{name}.json"))
}

fn to_json(c: &DofCase) -> String {
    let mut s = serde_json::to_string_pretty(c).unwrap();
    s.push('\n');
    s
}

/// JSON trees equal up to a 1e-9 relative tolerance on numbers (libm differs across
/// platforms).
fn same_json(a: &serde_json::Value, b: &serde_json::Value) -> bool {
    use serde_json::Value as V;
    match (a, b) {
        (V::Number(x), V::Number(y)) => {
            let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
            (x - y).abs() <= 1e-9 * x.abs().max(y.abs()).max(1.0)
        }
        (V::Array(x), V::Array(y)) => {
            x.len() == y.len() && x.iter().zip(y).all(|(a, b)| same_json(a, b))
        }
        (V::Object(x), V::Object(y)) => {
            x.len() == y.len()
                && x.iter()
                    .all(|(k, v)| y.get(k).is_some_and(|w| same_json(v, w)))
        }
        _ => a == b,
    }
}

fn load_files() -> Vec<DofCase> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dof_dir())
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|e| e == "json"))
        .collect();
    paths.sort();
    paths
        .iter()
        .map(|p| serde_json::from_str(&std::fs::read_to_string(p).unwrap()).unwrap())
        .collect()
}

#[test]
fn dof_cases_match_files() {
    let cases = cases();
    assert_eq!(cases.len(), 50);
    if std::env::var("AMG_BLESS").is_ok_and(|v| v == "1") {
        std::fs::create_dir_all(dof_dir()).unwrap();
        for c in &cases {
            std::fs::write(file_name(c), to_json(c)).unwrap();
        }
    }
    let files = load_files();
    assert_eq!(
        files.len(),
        cases.len(),
        "testdata/sketch/dof has stray or missing files"
    );
    for c in &cases {
        let text = std::fs::read_to_string(file_name(c))
            .unwrap_or_else(|_| panic!("{} missing (AMG_BLESS=1)", c.case.id));
        let a: serde_json::Value = serde_json::from_str(&text).unwrap();
        let b: serde_json::Value = serde_json::from_str(&to_json(c)).unwrap();
        assert!(
            same_json(&a, &b),
            "{} differs from the generator (AMG_BLESS=1)",
            c.case.id
        );
        assert_eq!(c.entities.len(), c.case.entities.len(), "{}", c.case.id);
        c.case.validate().unwrap();
    }
}

// ---- Analysis ---------------------------------------------------------------------------------

fn state(s: ConstraintState) -> St {
    match s {
        ConstraintState::Under => Under,
        ConstraintState::Full => Full,
        ConstraintState::Over => Over,
    }
}

fn kind_of(k: amg_sketch::solve::GroupKind) -> GroupKind {
    match k {
        amg_sketch::solve::GroupKind::Redundant => Redundant,
        amg_sketch::solve::GroupKind::Conflicting => Conflicting,
        amg_sketch::solve::GroupKind::DegenerateConfiguration => Degenerate,
    }
}

/// Runs the analysis on a case and returns the mismatches.
fn check_case(c: &DofCase, solver: &mut NativeSolver) -> Vec<String> {
    let built = c.case.build().unwrap();
    let (dof, diag) = solver.analyze(&built.sketch);
    let mut why = Vec::new();
    if dof.dof != c.dof {
        why.push(format!("dof {} (expected {})", dof.dof, c.dof));
    }
    for (i, want) in c.entities.iter().enumerate() {
        let got = dof.entities.get(&built.entities[i]).map(|s| Ent {
            state: state(s.state),
            free_dof: s.free_dof,
        });
        if got != Some(*want) {
            why.push(format!(
                "entity {i} ({}): {got:?}, expected {want:?}",
                c.case.entities[i].name
            ));
        }
    }
    if dof.entities.len() != c.entities.len() {
        why.push(format!("{} entity statuses", dof.entities.len()));
    }
    let got: BTreeSet<ExpectedGroup> = diag
        .groups
        .iter()
        .map(|g| ExpectedGroup {
            kind: kind_of(g.kind),
            constraints: g
                .constraints
                .iter()
                .map(|&id| built.constraint_index(id).unwrap())
                .collect(),
        })
        .collect();
    let want: BTreeSet<ExpectedGroup> = c.case.expected.groups.iter().cloned().collect();
    if got != want {
        why.push(format!("groups {got:?}, expected {want:?}"));
    }
    why
}

#[test]
fn dof_cases_match_analysis() {
    let cases = load_files();
    let mut failures = Vec::new();
    for c in &cases {
        for (label, mut solver) in [
            ("default", NativeSolver::new()),
            ("sparse", NativeSolver::new().with_dense_limit(0)),
            ("monolithic", NativeSolver::new().with_decomposition(false)),
        ] {
            let why = check_case(c, &mut solver);
            if !why.is_empty() {
                failures.push(format!("{} [{label}]: {}", c.case.id, why.join("; ")));
            }
        }
    }
    println!("{} cases", cases.len());
    assert!(
        failures.is_empty(),
        "{} failures:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

#[test]
fn per_entity_solve_matches_analysis() {
    // `SolveOptions { dof: PerEntity }` reports the same as `analyze` on the solved sketch.
    for c in load_files() {
        let built = c.case.build().unwrap();
        let mut s = built.sketch.clone();
        let mut solver = NativeSolver::new();
        let r = solver.solve(
            &mut s,
            &SolveOptions {
                dof: amg_sketch::solve::DofAnalysis::PerEntity,
                ..SolveOptions::default()
            },
            &|| false,
        );
        let d = r.dof.unwrap();
        assert_eq!(d.entities.len(), c.entities.len(), "{}", c.case.id);
        let (a, _) = NativeSolver::new().analyze(&s);
        assert_eq!(d, a, "{}", c.case.id);
    }
}

/// `check_constraint` against a full re-analysis of the sketch with the candidate.
fn reference_check(base: &amg_sketch::Sketch, cand: &K) -> ConstraintCheck {
    let mut s = base.clone();
    let id = s.add_constraint(cand.clone()).unwrap();
    let (_, diag) = NativeSolver::new().analyze(&s);
    let with = |g: &amg_sketch::solve::DependencyGroup| -> Vec<ConstraintId> {
        g.constraints.iter().copied().filter(|&c| c != id).collect()
    };
    let mine = |k: amg_sketch::solve::GroupKind| {
        diag.groups
            .iter()
            .find(|g| g.kind == k && g.constraints.contains(&id))
    };
    if let Some(g) = mine(amg_sketch::solve::GroupKind::Conflicting) {
        ConstraintCheck::Conflicting { with: with(g) }
    } else if let Some(g) = mine(amg_sketch::solve::GroupKind::Redundant) {
        ConstraintCheck::Redundant { with: with(g) }
    } else {
        ConstraintCheck::Independent
    }
}

#[test]
fn check_constraint_agrees_with_full_analysis() {
    let mut failures = Vec::new();
    let mut counts = [0usize; 3];
    for c in load_files() {
        let built = c.case.build().unwrap();
        // On the start configuration and on the solved one.
        let mut solved = built.sketch.clone();
        NativeSolver::new().solve(&mut solved, &SolveOptions::default(), &|| false);
        for (label, sketch) in [("start", &built.sketch), ("solved", &solved)] {
            let mut solver = NativeSolver::new();
            for &cid in &built.constraints {
                let mut base = sketch.clone();
                let kind = base.constraint(cid).unwrap().kind.clone();
                base.remove_constraint(cid).unwrap();
                let got = solver.check_constraint(&base, &kind).unwrap();
                let want = reference_check(&base, &kind);
                counts[match want {
                    ConstraintCheck::Independent => 0,
                    ConstraintCheck::Redundant { .. } => 1,
                    ConstraintCheck::Conflicting { .. } => 2,
                }] += 1;
                if got != want {
                    failures.push(format!(
                        "{} [{label}] constraint {}: {got:?}, full analysis {want:?}",
                        c.case.id,
                        built.constraint_index(cid).unwrap()
                    ));
                }
            }
        }
    }
    println!("independent / redundant / conflicting: {counts:?}");
    assert!(counts.iter().all(|&n| n > 0), "{counts:?}");
    assert!(
        failures.is_empty(),
        "{} failures:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

// ---- Ellipse (SK-22) and spline (SK-08) rows ---------------------------------------------------

fn pt(x: f64, y: f64) -> amg_core::Point2 {
    amg_core::Point2::new(x, y)
}

#[test]
fn equal_on_mirrored_ellipse_is_redundant_not_conflicting() {
    // SK-22: `Symmetric` of two ellipses already implies `Equal` (both axes): a consistent
    // redundancy, classified as such (not conflicting, not degenerate).
    let mut s = amg_sketch::Sketch::new(amg_core::Plane::XY);
    let axis = s.add_line(pt(0.0, -10.0), pt(0.0, 10.0)).unwrap();
    let e1 = s.add_ellipse(pt(-5.0, 1.0), pt(-2.0, 1.0), 1.5).unwrap();
    let e2 = s.add_ellipse(pt(5.0, 1.0), pt(2.0, 1.0), 1.5).unwrap();
    s.add_constraint(K::Fix { entity: axis }).unwrap();
    s.add_constraint(K::Fix { entity: e1 }).unwrap();
    let sym = s
        .add_constraint(K::Symmetric { a: e1, b: e2, axis })
        .unwrap();
    let eq = s.add_constraint(K::Equal { a: e1, b: e2 }).unwrap();
    let mut solver = NativeSolver::new();
    let (dof, diag) = solver.analyze(&s);
    assert_eq!(dof.dof, 0, "{diag:?}");
    assert!(!diag.groups.is_empty());
    assert!(
        diag.groups.iter().all(
            |g| g.kind == amg_sketch::solve::GroupKind::Redundant && g.constraints == [sym, eq]
        ),
        "{diag:?}"
    );
    assert_eq!(dof.entities[&e2].state, ConstraintState::Over);
    assert_eq!(dof.entities[&e2].free_dof, 0);
    // The candidate check says the same.
    s.remove_constraint(eq).unwrap();
    let check = solver
        .check_constraint(&s, &K::Equal { a: e1, b: e2 })
        .unwrap();
    assert_eq!(check, ConstraintCheck::Redundant { with: vec![sym] });
}

#[test]
fn point_on_spline_counts_its_curve_parameter() {
    // SK-08: a point on a free cubic spline owns the aux parameter `t`: 8 + 2 + 1 − 2 = 9.
    let mut s = amg_sketch::Sketch::new(amg_core::Plane::XY);
    let poles = [pt(0.0, 0.0), pt(3.0, 4.0), pt(7.0, 4.0), pt(10.0, 0.0)];
    let sp = s.add_spline(3, &poles, None, None).unwrap();
    let on = s.curve(sp).unwrap().eval(0.4);
    let q = s.add_point(on).unwrap();
    s.add_constraint(K::PointOnCurve {
        point: q,
        curve: sp,
    })
    .unwrap();
    let (dof, diag) = NativeSolver::new().analyze(&s);
    assert_eq!(dof.dof, 9, "{diag:?}");
    assert!(diag.groups.is_empty(), "{diag:?}");
    assert_eq!(dof.entities[&sp].state, ConstraintState::Under);
    assert_eq!(dof.entities[&sp].free_dof, 8);
    assert_eq!(dof.entities[&q].free_dof, 2);

    // With the spline fixed the point slides along it (1).
    s.add_constraint(K::Fix { entity: sp }).unwrap();
    let (dof, diag) = NativeSolver::new().analyze(&s);
    assert_eq!(dof.dof, 1, "{diag:?}");
    assert_eq!(dof.entities[&sp].state, ConstraintState::Full);
    assert_eq!(dof.entities[&q].free_dof, 1);
}

// ---- Timing (Done-when 3; the criterion bench `dof` is the measurement) -------------------

/// 63 fully dimensioned rectangles on a grid (504 entities).
fn rect_grid() -> amg_sketch::Sketch {
    let mut s = amg_sketch::Sketch::new(amg_core::Plane::XY);
    for i in 0..63 {
        let (x, y) = (f64::from(i % 16) * 30.0, f64::from(i / 16) * 20.0);
        let p: Vec<EntityId> = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]
            .iter()
            .map(|&(dx, dy)| s.add_point(pt(x + dx, y + dy)).unwrap())
            .collect();
        let l: Vec<EntityId> = (0..4)
            .map(|k| s.add_line_between(p[k], p[(k + 1) % 4]).unwrap())
            .collect();
        for (k, &li) in l.iter().enumerate() {
            let t = HvTarget::Line(li);
            s.add_constraint(if k % 2 == 0 {
                K::Horizontal(t)
            } else {
                K::Vertical(t)
            })
            .unwrap();
        }
        s.add_constraint(K::Fix { entity: p[0] }).unwrap();
        for (li, v) in [(l[0], 20.0), (l[1], 10.0)] {
            s.add_constraint(K::Dimension(Dimension {
                kind: Dk::Length { line: li },
                value: v,
                driving: true,
                label_pos: amg_core::Point2::ZERO,
            }))
            .unwrap();
        }
    }
    s
}

#[test]
fn analysis_of_500_entities_is_fast() {
    // Done-when bound (10 ms) only with AMG_STRICT_TIMING=1; shared runners get 10×.
    let strict = std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1");
    let bound = if strict { 10.0 } else { 100.0 };
    let s = rect_grid();
    assert!(s.entities().count() >= 500);
    let mut warm = NativeSolver::new();
    let mut solved = s.clone();
    warm.solve(&mut solved, &SolveOptions::fast(), &|| false);
    let mut best = f64::INFINITY;
    for _ in 0..5 {
        let mut solver = warm.clone();
        let t = std::time::Instant::now();
        let (dof, diag) = solver.analyze(&solved);
        best = best.min(t.elapsed().as_secs_f64() * 1e3);
        assert_eq!(dof.dof, 0);
        assert!(diag.groups.is_empty());
        assert!(
            dof.entities
                .values()
                .all(|e| e.state == ConstraintState::Full)
        );
    }
    println!("500-entity analysis: best {best:.3} ms");
    assert!(best < bound, "{best} ms");
}
