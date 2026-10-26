//! Synthetic tessellated CAD-like parts with known face ids (MESH-12 tests and bench).
//!
//! Twenty parts in three families, all closed, outward oriented and welded, with every pair of
//! adjacent true faces meeting at a crease of at least 45 degrees:
//! - ten solids of revolution (stepped shaft, flange with bore, cone, dome, pill, pulley,
//!   sphere zone, cup, frustum, spindle): planes, cylinders, cones, spheres and a torus;
//! - four regular K-gon prisms with a coaxial (counter-bored) hole;
//! - six rectilinear extrusions on a lattice (L, T, U, stairs, frame, plate).
//!
//! `build(noise)` returns the mesh (optionally with every vertex displaced by up to 0.02 mm per
//! axis, seeded by the vertex index) rotated and moved off the axes, and the true face id of
//! every triangle.
#![allow(dead_code, clippy::expect_used, clippy::cast_precision_loss)]

use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

use amg_core::{Point3, Quat, Vec3};
use amg_mesh::IndexedMesh;

/// A part with the true face id of every triangle.
pub struct Part {
    pub name: &'static str,
    pub mesh: IndexedMesh,
    pub truth: Vec<u32>,
}

/// Welding builder: vertices equal to 1e-6 mm are one vertex; degenerate triangles are dropped.
#[derive(Default)]
struct Builder {
    pos: Vec<Point3>,
    lookup: HashMap<[i64; 3], u32>,
    tris: Vec<[u32; 3]>,
    truth: Vec<u32>,
}

impl Builder {
    fn v(&mut self, p: Point3) -> u32 {
        let key = [p.x, p.y, p.z].map(|c| (c * 1e6).round() as i64);
        *self.lookup.entry(key).or_insert_with(|| {
            self.pos.push(p);
            (self.pos.len() - 1) as u32
        })
    }

    fn tri(&mut self, a: Point3, b: Point3, c: Point3, face: u32) {
        let (i, j, k) = (self.v(a), self.v(b), self.v(c));
        if i != j && j != k && i != k {
            self.tris.push([i, j, k]);
            self.truth.push(face);
        }
    }

    /// Quad `a b c d` (counter-clockwise seen from outside); `alt` picks the other diagonal.
    fn quad(&mut self, q: [Point3; 4], face: u32, alt: bool) {
        let [a, b, c, d] = q;
        if alt {
            self.tri(a, b, d, face);
            self.tri(b, c, d, face);
        } else {
            self.tri(a, b, c, face);
            self.tri(a, c, d, face);
        }
    }

    fn finish(self, name: &'static str, noise: bool, seed: u64) -> Part {
        let mut mesh = IndexedMesh::new(self.pos, self.tris).expect("valid part");
        let rot = Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0).normalize(), 0.7 + seed as f64);
        let shift = Vec3::new(13.0, -7.0, 4.0 + seed as f64);
        for (i, p) in mesh.positions.iter_mut().enumerate() {
            if noise {
                let h = |axis: u64| {
                    let x = splitmix(seed.wrapping_mul(0x9E37_79B9) ^ ((i as u64) << 2) ^ axis);
                    ((x >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
                };
                *p += Vec3::new(h(0), h(1), h(2)) * 0.02;
            }
            *p = rot * *p + shift;
        }
        Part {
            name,
            mesh,
            truth: self.truth,
        }
    }
}

fn splitmix(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    x ^ (x >> 31)
}

// ------------------------------------------------------------------ solids of revolution

type Rz = (f64, f64);

fn line(a: Rz, b: Rz, h: f64) -> Vec<Rz> {
    let len = ((b.0 - a.0).hypot(b.1 - a.1)).max(1e-9);
    let k = (len / h).ceil().max(1.0) as usize;
    let mut v: Vec<Rz> = (0..k)
        .map(|i| {
            let t = i as f64 / k as f64;
            (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
        })
        .collect();
    v.push(b);
    v
}

/// Circle arc around `c` from `a0` to `a1` degrees.
fn arc(c: Rz, rad: f64, a0: f64, a1: f64, h: f64) -> Vec<Rz> {
    let k = ((a1 - a0).abs().to_radians() * rad / h).ceil().max(1.0) as usize;
    (0..=k)
        .map(|i| {
            let a = (a0 + (a1 - a0) * i as f64 / k as f64).to_radians();
            (c.0 + rad * a.cos(), c.1 + rad * a.sin())
        })
        .collect()
}

/// Revolves profile polylines (one per true face, outward when walked counter-clockwise in the
/// (r, z) half plane) around the z axis in `m` steps.
fn revolve(faces: &[Vec<Rz>], m: u32) -> Builder {
    let mut b = Builder::default();
    for (fi, pts) in faces.iter().enumerate() {
        let p = |i: usize, j: u32| {
            let th = TAU * f64::from(j % m) / f64::from(m);
            Point3::new(pts[i].0 * th.cos(), pts[i].0 * th.sin(), pts[i].1)
        };
        for i in 0..pts.len() - 1 {
            for j in 0..m {
                b.quad(
                    [p(i, j), p(i, j + 1), p(i + 1, j + 1), p(i + 1, j)],
                    fi as u32,
                    false,
                );
            }
        }
    }
    b
}

fn chain(points: &[Rz], h: f64) -> Vec<Vec<Rz>> {
    points.windows(2).map(|w| line(w[0], w[1], h)).collect()
}

/// Profile of revolved part `i` (0..10).
fn revolved(i: usize, h: f64) -> (&'static str, Vec<Vec<Rz>>) {
    match i {
        0 => (
            "stepped_shaft",
            chain(
                &[
                    (0.0, 0.0),
                    (10.0, 0.0),
                    (10.0, 20.0),
                    (7.0, 20.0),
                    (7.0, 45.0),
                    (5.0, 47.0),
                    (0.0, 47.0),
                ],
                h,
            ),
        ),
        1 => (
            "flange_bore",
            chain(
                &[
                    (4.0, 0.0),
                    (20.0, 0.0),
                    (20.0, 4.0),
                    (8.0, 4.0),
                    (8.0, 18.0),
                    (4.0, 18.0),
                    (4.0, 0.0),
                ],
                h,
            ),
        ),
        2 => (
            "cone_on_cylinder",
            chain(&[(0.0, 0.0), (10.0, 0.0), (10.0, 15.0), (0.0, 25.0)], h),
        ),
        3 => {
            let r = 17.0_f64;
            let zc = 12.0 - (r * r - 144.0).sqrt();
            let a0 = (12.0 - zc).atan2(12.0).to_degrees();
            let mut f = chain(&[(0.0, 0.0), (12.0, 0.0), (12.0, 12.0)], h);
            f.push(arc((0.0, zc), r, a0, 90.0, h));
            ("domed_cylinder", f)
        }
        4 => {
            let (c, r) = ((14.0, 6.0), 6.0);
            let s = arc(c, r, -45.0, 45.0, h);
            let (p0, p1) = (s[0], s[s.len() - 1]);
            (
                "pill_disc",
                vec![line((0.0, p0.1), p0, h), s, line(p1, (0.0, p1.1), h)],
            )
        }
        5 => (
            "v_pulley",
            chain(
                &[
                    (0.0, 0.0),
                    (20.0, 0.0),
                    (20.0, 6.0),
                    (14.0, 9.0),
                    (20.0, 12.0),
                    (20.0, 18.0),
                    (0.0, 18.0),
                ],
                h,
            ),
        ),
        6 => {
            let r = 15.0_f64;
            let a = (10.0_f64 / r).asin().to_degrees();
            let rr = (r * r - 100.0).sqrt();
            (
                "sphere_zone",
                vec![
                    line((0.0, -10.0), (rr, -10.0), h),
                    arc((0.0, 0.0), r, -a, a, h),
                    line((rr, 10.0), (0.0, 10.0), h),
                ],
            )
        }
        7 => (
            "cup",
            chain(
                &[
                    (0.0, 0.0),
                    (12.0, 0.0),
                    (12.0, 20.0),
                    (9.0, 20.0),
                    (9.0, 5.0),
                    (0.0, 5.0),
                ],
                h,
            ),
        ),
        8 => (
            "frustum",
            chain(&[(0.0, 0.0), (15.0, 0.0), (8.0, 25.0), (0.0, 25.0)], h),
        ),
        _ => (
            "spindle",
            chain(
                &[
                    (0.0, 0.0),
                    (6.0, 0.0),
                    (14.0, 8.0),
                    (14.0, 28.0),
                    (8.0, 34.0),
                    (0.0, 34.0),
                ],
                h,
            ),
        ),
    }
}

// ------------------------------------------------------------------ regular K-gon prisms

struct PrismSpec {
    name: &'static str,
    k: u32,
    m: u32,
    circumradius: f64,
    height: f64,
    /// Hole radius at the top, and (counter-bore) radius below `step_z`.
    top_hole: f64,
    low_hole: f64,
    step_z: f64,
}

fn prism(s: &PrismSpec, h: f64) -> Builder {
    let apothem = s.circumradius * (PI / f64::from(s.k)).cos();
    let theta = |j: u32| TAU * f64::from(j % s.m) / f64::from(s.m);
    // Point of the polygon boundary in direction `th` (corners at multiples of 2 pi / k).
    let poly = |j: u32, z: f64| {
        let th = theta(j);
        let sector = TAU / f64::from(s.k);
        let mid = ((th / sector).floor() + 0.5) * sector;
        let r = apothem / (th - mid).cos();
        Point3::new(r * th.cos(), r * th.sin(), z)
    };
    let circle = |j: u32, r: f64, z: f64| {
        let th = theta(j);
        Point3::new(r * th.cos(), r * th.sin(), z)
    };
    let per = s.m / s.k;
    let nz = |z0: f64, z1: f64| ((z1 - z0) / h).ceil().max(1.0) as u32;
    let zlerp = |z0: f64, z1: f64, i: u32, n: u32| z0 + (z1 - z0) * f64::from(i) / f64::from(n);
    let mut b = Builder::default();
    let counterbore = (s.low_hole - s.top_hole).abs() > 1e-9;
    let mut face = 0u32;
    // Rings: (inner radius, plane z, facing up) -> radial lerp to the polygon or another circle.
    let ring = |b: &mut Builder, r_in: f64, outer: Option<f64>, z: f64, up: bool, f: u32| {
        let nr = ((outer.map_or(apothem, |r| r) - r_in) / h).ceil().max(1.0) as u32;
        for i in 0..nr {
            for j in 0..s.m {
                let q = |ii: u32, jj: u32| {
                    let t = f64::from(ii) / f64::from(nr);
                    let a = circle(jj, r_in, z);
                    let o = outer.map_or_else(|| poly(jj, z), |r| circle(jj, r, z));
                    a + (o - a) * t
                };
                let (a, bb, c, d) = (q(i, j), q(i, j + 1), q(i + 1, j + 1), q(i + 1, j));
                if up {
                    b.quad([a, d, c, bb], f, false);
                } else {
                    b.quad([a, bb, c, d], f, false);
                }
            }
        }
    };
    // Top, bottom.
    ring(&mut b, s.top_hole, None, s.height, true, face);
    face += 1;
    ring(&mut b, s.low_hole, None, 0.0, false, face);
    face += 1;
    // Sides.
    for side in 0..s.k {
        let n = nz(0.0, s.height);
        for j in side * per..(side + 1) * per {
            for i in 0..n {
                let (z0, z1) = (zlerp(0.0, s.height, i, n), zlerp(0.0, s.height, i + 1, n));
                b.quad(
                    [poly(j, z0), poly(j + 1, z0), poly(j + 1, z1), poly(j, z1)],
                    face,
                    false,
                );
            }
        }
        face += 1;
    }
    // Hole walls (normal towards the axis).
    let wall = |b: &mut Builder, r: f64, z0: f64, z1: f64, f: u32| {
        let n = nz(z0, z1);
        for j in 0..s.m {
            for i in 0..n {
                let (a, c) = (zlerp(z0, z1, i, n), zlerp(z0, z1, i + 1, n));
                b.quad(
                    [
                        circle(j, r, a),
                        circle(j, r, c),
                        circle(j + 1, r, c),
                        circle(j + 1, r, a),
                    ],
                    f,
                    false,
                );
            }
        }
    };
    if counterbore {
        wall(&mut b, s.top_hole, s.step_z, s.height, face);
        face += 1;
        // Shoulder facing up: from the narrow low hole out to the wide top hole.
        ring(
            &mut b,
            s.low_hole.min(s.top_hole),
            Some(s.low_hole.max(s.top_hole)),
            s.step_z,
            true,
            face,
        );
        face += 1;
        wall(&mut b, s.low_hole, 0.0, s.step_z, face);
    } else {
        wall(&mut b, s.top_hole, 0.0, s.height, face);
    }
    b
}

fn prism_specs() -> [PrismSpec; 4] {
    [
        PrismSpec {
            name: "square_counterbore",
            k: 4,
            m: 72,
            circumradius: 22.0,
            height: 12.0,
            top_hole: 9.0,
            low_hole: 5.0,
            step_z: 6.0,
        },
        PrismSpec {
            name: "hex_through_hole",
            k: 6,
            m: 72,
            circumradius: 20.0,
            height: 14.0,
            top_hole: 7.0,
            low_hole: 7.0,
            step_z: 0.0,
        },
        PrismSpec {
            name: "triangle_counterbore",
            k: 3,
            m: 72,
            circumradius: 26.0,
            height: 10.0,
            top_hole: 8.0,
            low_hole: 5.0,
            step_z: 5.0,
        },
        PrismSpec {
            name: "octagon_through_hole",
            k: 8,
            m: 80,
            circumradius: 24.0,
            height: 9.0,
            top_hole: 10.0,
            low_hole: 10.0,
            step_z: 0.0,
        },
    ]
}

// ------------------------------------------------------------------ rectilinear extrusions

/// Extrudes lattice loops (integer coordinates in cells of size `cell`; outer loops counter-
/// clockwise, holes clockwise) by `height` cells. Faces: top, bottom, then one per loop edge.
fn extrude(loops: &[Vec<(i32, i32)>], height: i32, cell: f64) -> Builder {
    let pt = |x: i32, y: i32, z: i32| {
        Point3::new(
            f64::from(x) * cell,
            f64::from(y) * cell,
            f64::from(z) * cell,
        )
    };
    let all: Vec<(i32, i32)> = loops.iter().flatten().copied().collect();
    let (x0, x1) = (
        all.iter().map(|p| p.0).min().expect("points"),
        all.iter().map(|p| p.0).max().expect("points"),
    );
    let (y0, y1) = (
        all.iter().map(|p| p.1).min().expect("points"),
        all.iter().map(|p| p.1).max().expect("points"),
    );
    // Even-odd test of the cell centre (doubled coordinates avoid fractions).
    let inside = |cx: i32, cy: i32| {
        let (px, py) = (2 * cx + 1, 2 * cy + 1);
        let mut odd = false;
        for l in loops {
            for w in 0..l.len() {
                let (a, b) = (l[w], l[(w + 1) % l.len()]);
                if a.0 == b.0 {
                    let (ya, yb) = (2 * a.1.min(b.1), 2 * a.1.max(b.1));
                    if 2 * a.0 > px && py > ya && py < yb {
                        odd = !odd;
                    }
                }
            }
        }
        odd
    };
    let mut b = Builder::default();
    for cx in x0..x1 {
        for cy in y0..y1 {
            if inside(cx, cy) {
                let alt = (cx + cy) % 2 == 0;
                b.quad(
                    [
                        pt(cx, cy, height),
                        pt(cx + 1, cy, height),
                        pt(cx + 1, cy + 1, height),
                        pt(cx, cy + 1, height),
                    ],
                    0,
                    alt,
                );
                b.quad(
                    [
                        pt(cx, cy, 0),
                        pt(cx, cy + 1, 0),
                        pt(cx + 1, cy + 1, 0),
                        pt(cx + 1, cy, 0),
                    ],
                    1,
                    alt,
                );
            }
        }
    }
    let mut face = 2;
    for l in loops {
        for w in 0..l.len() {
            let (a, c) = (l[w], l[(w + 1) % l.len()]);
            let (dx, dy) = ((c.0 - a.0).signum(), (c.1 - a.1).signum());
            let len = (c.0 - a.0).abs() + (c.1 - a.1).abs();
            for i in 0..len {
                for k in 0..height {
                    let p = |s: i32, z: i32| pt(a.0 + dx * s, a.1 + dy * s, z);
                    b.quad(
                        [p(i, k), p(i + 1, k), p(i + 1, k + 1), p(i, k + 1)],
                        face,
                        false,
                    );
                }
            }
            face += 1;
        }
    }
    b
}

type Loop = Vec<(i32, i32)>;

/// Loops of rectilinear part `i` (0..6) with its height, scaled by `s` lattice cells per mm.
fn rectilinear(i: usize, s: i32) -> (&'static str, Vec<Loop>, i32) {
    let sc = |l: &[(i32, i32)]| -> Loop { l.iter().map(|&(x, y)| (x * s, y * s)).collect() };
    match i {
        0 => (
            "l_bracket",
            vec![sc(&[
                (0, 0),
                (30, 0),
                (30, 10),
                (10, 10),
                (10, 30),
                (0, 30),
            ])],
            12 * s,
        ),
        1 => (
            "t_bar",
            vec![sc(&[
                (0, 20),
                (0, 0),
                (36, 0),
                (36, 20),
                (24, 20),
                (24, 36),
                (12, 36),
                (12, 20),
            ])],
            10 * s,
        ),
        2 => (
            "u_channel",
            vec![sc(&[
                (0, 0),
                (30, 0),
                (30, 25),
                (22, 25),
                (22, 8),
                (8, 8),
                (8, 25),
                (0, 25),
            ])],
            40 * s,
        ),
        3 => (
            "stairs",
            vec![sc(&[
                (0, 0),
                (36, 0),
                (36, 8),
                (28, 8),
                (28, 16),
                (20, 16),
                (20, 24),
                (12, 24),
                (12, 32),
                (0, 32),
            ])],
            10 * s,
        ),
        4 => (
            "square_frame",
            vec![
                sc(&[(0, 0), (40, 0), (40, 40), (0, 40)]),
                sc(&[(8, 8), (8, 32), (32, 32), (32, 8)]),
            ],
            8 * s,
        ),
        _ => (
            "plate",
            vec![sc(&[(0, 0), (40, 0), (40, 30), (0, 30)])],
            6 * s,
        ),
    }
}

// ------------------------------------------------------------------ API

/// Number of parts.
pub const PART_COUNT: usize = 20;

/// Part `i` (0..20) at the test resolution (about 1 mm triangles).
pub fn build(i: usize, noise: bool) -> Part {
    let seed = i as u64 + 1;
    match i {
        0..=9 => {
            let (name, faces) = revolved(i, 1.0);
            revolve(&faces, 72).finish(name, noise, seed)
        }
        10..=13 => {
            let spec = &prism_specs()[i - 10];
            prism(spec, 1.0).finish(spec.name, noise, seed)
        }
        _ => {
            let (name, loops, h) = rectilinear(i - 14, 1);
            extrude(&loops, h, 1.0).finish(name, noise, seed)
        }
    }
}

/// All twenty parts.
pub fn all_parts(noise: bool) -> Vec<Part> {
    (0..PART_COUNT).map(|i| build(i, noise)).collect()
}

/// L bracket with `s` cells per mm (about 4 900 s² triangles).
pub fn l_bracket_fine(s: i32, noise: bool) -> Part {
    let (name, loops, h) = rectilinear(0, s);
    extrude(&loops, h, 1.0 / f64::from(s)).finish(name, noise, 1)
}

/// Stepped shaft with `m` angular steps and axial/radial step `h` mm.
pub fn shaft_fine(m: u32, h: f64, noise: bool) -> Part {
    let (name, faces) = revolved(0, h);
    revolve(&faces, m).finish(name, noise, 1)
}

/// Best one-to-one matching of predicted to true labels, as the fraction of faces whose
/// predicted group is matched to their true face (Hungarian algorithm on the confusion matrix).
pub fn matched_fraction(truth: &[u32], pred: &[u32]) -> f64 {
    let nt = truth.iter().max().map_or(0, |&m| m as usize + 1);
    let np = pred.iter().max().map_or(0, |&m| m as usize + 1);
    let n = nt.max(np);
    let mut gain = vec![vec![0i64; n]; n];
    for (&t, &p) in truth.iter().zip(pred) {
        gain[t as usize][p as usize] += 1;
    }
    // Minimise the negated gain.
    let cost: Vec<Vec<i64>> = gain
        .iter()
        .map(|r| r.iter().map(|&g| -g).collect())
        .collect();
    let assign = hungarian(&cost);
    let hit: i64 = assign.iter().enumerate().map(|(t, &p)| gain[t][p]).sum();
    hit as f64 / truth.len().max(1) as f64
}

/// Min-cost assignment of a square matrix (rows to columns), O(n^3).
fn hungarian(a: &[Vec<i64>]) -> Vec<usize> {
    let n = a.len();
    let inf = i64::MAX / 4;
    let (mut u, mut v) = (vec![0i64; n + 1], vec![0i64; n + 1]);
    let (mut p, mut way) = (vec![0usize; n + 1], vec![0usize; n + 1]);
    for i in 1..=n {
        p[0] = i;
        let mut j0 = 0;
        let mut minv = vec![inf; n + 1];
        let mut used = vec![false; n + 1];
        loop {
            used[j0] = true;
            let (i0, mut delta, mut j1) = (p[j0], inf, 0);
            for j in 1..=n {
                if !used[j] {
                    let cur = a[i0 - 1][j - 1] - u[i0] - v[j];
                    if cur < minv[j] {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if minv[j] < delta {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for j in 0..=n {
                if used[j] {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
            if p[j0] == 0 {
                break;
            }
        }
        loop {
            let j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
            if j0 == 0 {
                break;
            }
        }
    }
    let mut res = vec![0usize; n];
    for j in 1..=n {
        if p[j] != 0 {
            res[p[j] - 1] = j - 1;
        }
    }
    res
}
