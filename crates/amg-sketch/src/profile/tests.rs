//! Profile detection fuzz tests: 10 000 random line/arc/circle soups.
//!
//! Every case checks that detection does not panic, that the planar-graph Euler relation
//! `V - E + F = 1 + C` holds for the arrangement (`F` counts the unbounded face once, so
//! `cycles = F - 1 + C`), and that the loops are well formed. Lattice-snapped soups also compare
//! the summed profile area with a raster oracle: a flood fill over a grid of cell centres, where
//! a link between neighbouring centres is blocked iff a curve crosses it. The cells the fill
//! cannot reach from outside make up the union of all bounded faces, which is the sum of the net
//! profile areas (holes are covered by their own island profiles).
//!
//! Lattice soups use integer end points, so every vertex is either a lattice point or a
//! rational crossing far from its neighbours relative to the weld tolerance; no near-gap can
//! seal a pocket narrower than a cell and make the oracle disagree. Generic soups (random
//! floats) only get the structural checks.

#![allow(
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    reason = "test code"
)]

use std::f64::consts::{FRAC_PI_2, TAU};

use amg_core::{Plane, Point2};

use super::{ProfileOptions, arrange};
use crate::model::Sketch;

/// SplitMix64.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform integer in `lo..=hi`.
    fn int(&mut self, lo: i64, hi: i64) -> i64 {
        lo + (self.next() % (hi - lo + 1) as u64) as i64
    }

    /// Uniform float in `[lo, hi)`.
    fn float(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (self.next() >> 11) as f64 / (1u64 << 53) as f64 * (hi - lo)
    }
}

/// Oracle-side description of one curve.
#[derive(Copy, Clone, Debug)]
enum Shape {
    Seg([f64; 2], [f64; 2]),
    /// Centre, radius, start angle, sweep (`TAU` for a circle).
    Round([f64; 2], f64, f64, f64),
}

/// Domain of the raster oracle: `[LO, LO + SPAN]` squared, which holds every lattice shape with
/// a margin. The origin is shifted, differently per axis, so grid rows and columns never coincide
/// with lattice lines and diagonal lattice lines never pass through cell centres.
const LO: [f64; 2] = [-1.0 + 0.003_71, -1.0 + 0.006_13];
const SPAN: f64 = 8.0;
const N: usize = 320;

fn pt(p: [f64; 2]) -> Point2 {
    Point2::new(p[0], p[1])
}

/// Quarter-turn unit vectors.
const DIRS: [[f64; 2]; 4] = [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]];

fn lattice_soup(rng: &mut Rng) -> (Sketch, Vec<Shape>) {
    let mut s = Sketch::new(Plane::XY);
    let mut shapes = Vec::new();
    let count = rng.int(2, 9);
    for _ in 0..count {
        match rng.int(0, 5) {
            0..=2 => {
                let a = [rng.int(0, 6), rng.int(0, 6)];
                let b = [rng.int(0, 6), rng.int(0, 6)];
                if a != b {
                    let (a, b) = (a.map(|v| v as f64), b.map(|v| v as f64));
                    s.add_line(pt(a), pt(b)).unwrap();
                    shapes.push(Shape::Seg(a, b));
                }
            }
            3 => {
                let c = [rng.int(2, 4) as f64, rng.int(2, 4) as f64];
                let r = rng.int(1, 2) as f64;
                s.add_circle(pt(c), r).unwrap();
                shapes.push(Shape::Round(c, r, 0.0, TAU));
            }
            _ => {
                let c = [rng.int(2, 4) as f64, rng.int(2, 4) as f64];
                let r = rng.int(1, 2) as f64;
                let q0 = rng.int(0, 3) as usize;
                let k = rng.int(1, 3) as usize;
                let (d0, d1) = (DIRS[q0], DIRS[(q0 + k) % 4]);
                let at = |d: [f64; 2]| pt([c[0] + r * d[0], c[1] + r * d[1]]);
                s.add_arc(pt(c), at(d0), at(d1)).unwrap();
                shapes.push(Shape::Round(
                    c,
                    r,
                    q0 as f64 * FRAC_PI_2,
                    k as f64 * FRAC_PI_2,
                ));
            }
        }
    }
    (s, shapes)
}

fn generic_soup(rng: &mut Rng) -> Sketch {
    let mut s = Sketch::new(Plane::XY);
    for _ in 0..rng.int(2, 12) {
        let kind = rng.int(0, 3);
        let mut p = || pt([rng.float(-5.0, 5.0), rng.float(-5.0, 5.0)]);
        let (a, b) = (p(), p());
        let r = rng.float(0.2, 3.0);
        let (a0, a1) = (rng.float(0.0, TAU), rng.float(0.0, TAU));
        // Degenerate entities are rejected by the model; the fuzz only needs the valid ones.
        let _ = match kind {
            0 | 1 => s.add_line(a, b),
            2 => s.add_circle(a, r),
            _ => {
                let at = |a: f64| Point2::new(b.x + r * a.cos(), b.y + r * a.sin());
                s.add_arc(b, at(a0), at(a1))
            }
        };
    }
    s
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn point_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (d[0] * d[0] + d[1] * d[1]))
        .clamp(0.0, 1.0);
    dist(p, [a[0] + t * d[0], a[1] + t * d[1]])
}

/// End points of a shape (none for a full circle).
fn ends(s: Shape) -> Vec<[f64; 2]> {
    match s {
        Shape::Seg(a, b) => vec![a, b],
        Shape::Round(c, r, a0, sweep) if sweep < TAU - 1e-12 => [a0, a0 + sweep]
            .iter()
            .map(|a| [c[0] + r * a.cos(), c[1] + r * a.sin()])
            .collect(),
        Shape::Round(..) => Vec::new(),
    }
}

/// Distance from `p` to the shape's carrier (a full circle for arcs: conservative).
fn carrier_distance(p: [f64; 2], s: Shape) -> f64 {
    match s {
        Shape::Seg(a, b) => point_segment(p, a, b),
        Shape::Round(c, r, ..) => (dist(p, c) - r).abs(),
    }
}

/// True if two distinct shapes pass within `gap` of touching without touching (distance in
/// `(1e-9, gap]`). Such a corridor can be narrower than a raster cell, which seals it for the
/// oracle but not for the exact arrangement, so these soups are skipped.
fn has_near_miss(shapes: &[Shape], gap: f64) -> bool {
    let near = |d: f64| d > 1e-9 && d <= gap;
    for (i, &s) in shapes.iter().enumerate() {
        for (j, &t) in shapes.iter().enumerate() {
            if i == j {
                continue;
            }
            if ends(s).into_iter().any(|p| near(carrier_distance(p, t))) {
                return true;
            }
            if let (Shape::Round(c1, r1, ..), Shape::Round(c2, r2, ..)) = (s, t) {
                let d = dist(c1, c2);
                if near((d - (r1 + r2)).abs()) || near((d - (r1 - r2).abs()).abs()) {
                    return true;
                }
            }
            if let (Shape::Round(c, r, ..), Shape::Seg(a, b)) = (s, t)
                && near((point_segment(c, a, b) - r).abs())
            {
                return true;
            }
        }
    }
    false
}

/// Coordinates along axis `1 - f` where `shape` crosses the line `coord[f] == v`.
fn crossings(shape: Shape, f: usize, v: f64, out: &mut Vec<f64>) {
    let g = 1 - f;
    match shape {
        Shape::Seg(a, b) => {
            if (a[f] > v) != (b[f] > v) {
                out.push(a[g] + (v - a[f]) * (b[g] - a[g]) / (b[f] - a[f]));
            }
        }
        Shape::Round(c, r, a0, sweep) => {
            let d = v - c[f];
            if d.abs() < r {
                let w = (r * r - d * d).sqrt();
                for sign in [-1.0, 1.0] {
                    let mut q = [0.0; 2];
                    q[f] = v;
                    q[g] = c[g] + sign * w;
                    let ang = (q[1] - c[1]).atan2(q[0] - c[0]);
                    if sweep >= TAU - 1e-12 || (ang - a0).rem_euclid(TAU) < sweep {
                        out.push(q[g]);
                    }
                }
            }
        }
    }
}

/// Area (mm^2) of all cells that cannot reach the outside without crossing a curve.
fn raster_area(shapes: &[Shape]) -> f64 {
    let h = SPAN / N as f64;
    let centre = |axis: usize, i: usize| LO[axis] + (i as f64 + 0.5) * h;
    // blocked[f][line * N + i]: the link between cells i and i + 1 along the line
    // `coord[f] == centre(line)` is crossed by a curve.
    let mut blocked = [vec![false; N * N], vec![false; N * N]];
    let mut xs = Vec::new();
    for &shape in shapes {
        for (f, flags) in blocked.iter_mut().enumerate() {
            for line in 0..N {
                xs.clear();
                crossings(shape, f, centre(f, line), &mut xs);
                for &x in &xs {
                    let i = ((x - LO[1 - f]) / h - 0.5).floor();
                    if i >= 0.0 && (i as usize) + 1 < N {
                        flags[line * N + i as usize] = true;
                    }
                }
            }
        }
    }
    // Cell (ix, iy). The horizontal link (ix, iy)-(ix+1, iy) lies on y == centre(iy), i.e.
    // f = 1, line iy, index ix. Vertical links use f = 0, line ix, index iy.
    let mut seen = vec![false; N * N];
    let mut stack = vec![0usize];
    seen[0] = true;
    let mut reached = 1usize;
    while let Some(c) = stack.pop() {
        let (ix, iy) = (c % N, c / N);
        let mut nbrs = [None; 4];
        if ix + 1 < N && !blocked[1][iy * N + ix] {
            nbrs[0] = Some(c + 1);
        }
        if ix > 0 && !blocked[1][iy * N + ix - 1] {
            nbrs[1] = Some(c - 1);
        }
        if iy + 1 < N && !blocked[0][ix * N + iy] {
            nbrs[2] = Some(c + N);
        }
        if iy > 0 && !blocked[0][ix * N + iy - 1] {
            nbrs[3] = Some(c - N);
        }
        for n in nbrs.into_iter().flatten() {
            if !seen[n] {
                seen[n] = true;
                reached += 1;
                stack.push(n);
            }
        }
    }
    (N * N - reached) as f64 * h * h
}

/// Structural checks shared by all soups; returns the summed net profile area.
fn check_structure(sketch: &Sketch, label: &str) -> f64 {
    let (set, st) = arrange::detect(sketch, &ProfileOptions::default());
    // V - E + F = 1 + C with F = cycles - C + 1  <=>  V - E + cycles = 2C.
    assert_eq!(
        st.vertices as i64 - st.edges as i64 + st.cycles as i64,
        2 * st.components as i64,
        "Euler relation ({label}): {}V {}E {}cycles {}C",
        st.vertices,
        st.edges,
        st.cycles,
        st.components
    );
    let mut total = 0.0;
    for (i, p) in set.profiles.iter().enumerate() {
        if i > 0 {
            assert!(set.profiles[i - 1].id < p.id, "sorted unique ids ({label})");
        }
        assert!(p.fingerprint.area > 0.0, "positive profile area ({label})");
        assert!(p.outer.signed_area() > 0.0, "outer is CCW ({label})");
        assert!(
            p.holes.iter().all(|h| h.signed_area() < 0.0),
            "holes are CW ({label})"
        );
        total += p.fingerprint.area;
    }
    total
}

fn fuzz_lattice(seed: u64) {
    let mut rng = Rng(seed);
    let h = SPAN / N as f64;
    let mut worst: f64 = 0.0;
    for case in 0..1000 {
        let (sketch, shapes) = loop {
            let soup = lattice_soup(&mut rng);
            if !has_near_miss(&soup.1, 3.0 * h) {
                break soup;
            }
        };
        let label = format!("seed {seed:#x} case {case}");
        let total = check_structure(&sketch, &label);
        let oracle = raster_area(&shapes);
        // 0.5 % of the union plus a raster discretisation allowance: boundary cells are
        // counted whole, so the error scales with the cell size times the region diameter;
        // and a few cells can be pinched off in an acute wedge between two curves, where the
        // exact area is below the sliver threshold (a 30-cell absolute floor covers those).
        let tol = 0.005 * oracle + 2.0 * h * oracle.sqrt() + 30.0 * h * h;
        if oracle > 1.0 {
            worst = worst.max((total - oracle).abs() / oracle);
        }
        assert!(
            (total - oracle).abs() <= tol,
            "area ({label}): profiles {total}, raster {oracle}; {shapes:?}"
        );
    }
    eprintln!("seed {seed:#x}: worst relative area error (oracle > 1) {worst:.5}");
}

fn fuzz_generic(seed: u64) {
    let mut rng = Rng(seed);
    for case in 0..1000 {
        let sketch = generic_soup(&mut rng);
        check_structure(&sketch, &format!("seed {seed:#x} case {case}"));
    }
}

macro_rules! fuzz_tests {
    ($($name:ident: $f:ident($seed:expr);)*) => {
        $(#[test] fn $name() { $f($seed); })*
    };
}

fuzz_tests! {
    fuzz_lattice_0: fuzz_lattice(0x5106_0001);
    fuzz_lattice_1: fuzz_lattice(0x5106_0002);
    fuzz_lattice_2: fuzz_lattice(0x5106_0003);
    fuzz_lattice_3: fuzz_lattice(0x5106_0004);
    fuzz_lattice_4: fuzz_lattice(0x5106_0005);
    fuzz_lattice_5: fuzz_lattice(0x5106_0006);
    fuzz_lattice_6: fuzz_lattice(0x5106_0007);
    fuzz_generic_0: fuzz_generic(0x5106_0101);
    fuzz_generic_1: fuzz_generic(0x5106_0102);
    fuzz_generic_2: fuzz_generic(0x5106_0103);
}
