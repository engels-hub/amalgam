//! Seeded synthetic defect generator with ground truth.
//!
//! `generate(&SynthSpec { shape, defects, seed })` builds a clean closed base shape and injects
//! the requested defects in a fixed order (independent of the order in the spec), using a
//! `ChaCha8Rng` seeded from `seed`. The result and the [`GroundTruth`] are a pure function of the
//! spec: bit-identical on one platform and toolchain (positions use the platform libm `sin` /
//! `cos`; topology never depends on it).
//!
//! Defects that touch the surface choose faces that are pairwise vertex-disjoint and not within
//! one face of an earlier defect, so each defect's contribution to the ground truth is exact and
//! additive (checked against [`crate::corpus::quick_stats`] in `tests/corpus.rs`).
//!
//! Application order: holes, unwelded seam, flipped faces, duplicate faces, zero-area slivers,
//! needle slivers, non-manifold fins, bow-ties, overlapping shell, inverted shell, floating
//! shells, f32 rounding. Shell-adding defects copy the pristine base shape, not the defects.

use std::collections::HashMap;

use amg_core::{Point3, Vec3};
use rand_chacha::ChaCha8Rng;
use rand_chacha::rand_core::{RngCore, SeedableRng};

use crate::corpus::bench::{Displacement, geodesic_sphere};
use crate::{FaceGroups, IndexedMesh, MeshAttrs};

const NONE: u32 = u32::MAX;

/// Clean base shapes (closed, manifold, outward, one shell).
#[derive(Clone, Debug, PartialEq)]
pub enum Shape {
    /// Geodesic sphere, `20 frequency^2` triangles.
    Icosphere {
        /// Subdivision frequency (>= 1).
        frequency: u32,
        /// Radius, mm.
        radius: f64,
    },
    /// Geodesic sphere with a seeded radial relief (peak `amplitude` relative).
    DisplacedSphere {
        /// Subdivision frequency (>= 1).
        frequency: u32,
        /// Radius, mm.
        radius: f64,
        /// Peak relative relief, e.g. 0.06.
        amplitude: f64,
    },
    /// Torus of `2 * major_segments * minor_segments` triangles.
    Torus {
        /// Segments around the axis (>= 3).
        major_segments: u32,
        /// Segments around the tube (>= 3).
        minor_segments: u32,
        /// Distance from the axis to the tube centre, mm.
        major: f64,
        /// Tube radius, mm.
        minor: f64,
    },
    /// Square plate `size` x `size` x `size / 4` with `holes_per_side^2` square through-holes.
    BoxWithHoles {
        /// Cells per side of the plate (>= `3 * holes_per_side + 1`).
        cells: u32,
        /// Holes per side (>= 0).
        holes_per_side: u32,
        /// Plate size, mm.
        size: f64,
    },
    /// Tube around a (2, 3) torus knot (R = 10, r = 5 mm), tube radius 1 mm.
    TorusKnot {
        /// Samples along the knot (>= 64).
        segments: u32,
        /// Samples around the tube (>= 3).
        sides: u32,
    },
}

/// One defect class with its size parameters.
#[derive(Clone, Debug, PartialEq)]
pub enum DefectSpec {
    /// Remove `count` disk-shaped patches; boundary loop lengths are drawn log-uniformly from
    /// `3 ..= max_boundary` (capped by the patch limit of 10 % of the faces).
    Holes {
        /// Number of holes.
        count: u32,
        /// Largest boundary loop to aim for.
        max_boundary: u32,
    },
    /// Duplicate every vertex used on both sides of a horizontal plane through the shape, with a
    /// per-axis jitter up to `jitter` mm on the upper copies (an unwelded STL seam).
    UnweldedSeam {
        /// Maximum jitter per axis, mm (<= 1e-4 for the corpus).
        jitter: f64,
    },
    /// Reverse the winding of `count` isolated faces.
    FlippedFaces {
        /// Number of faces.
        count: u32,
    },
    /// Append a same-winding copy of `count` isolated faces.
    DuplicateFaces {
        /// Number of faces.
        count: u32,
    },
    /// Split `count` isolated faces so one sub-triangle has exactly collinear corners.
    ZeroAreaSlivers {
        /// Number of slivers.
        count: u32,
    },
    /// Like [`DefectSpec::ZeroAreaSlivers`] but the sliver has height `1e-6` of its base.
    NeedleSlivers {
        /// Number of slivers.
        count: u32,
    },
    /// Attach a zero-thickness double-sided flap to an edge of `count` isolated faces (a
    /// non-manifold edge with four faces).
    NonManifoldFins {
        /// Number of fins.
        count: u32,
    },
    /// Pinch `count` vertex pairs three edges apart into one vertex with two fans.
    BowTies {
        /// Number of pinched vertices.
        count: u32,
    },
    /// Append a copy of the base shape shifted by a fraction of its size, so the shells cross.
    OverlappingShell,
    /// Append a quarter-size copy of the base shape with reversed winding, outside the bbox.
    InvertedShell,
    /// Append `count` tiny closed tetrahedra outside the bbox.
    FloatingShells {
        /// Number of tetrahedra.
        count: u32,
    },
    /// Shift by (1000, -1000, 1000) mm and round every coordinate through `f32`.
    F32Rounded,
}

impl DefectSpec {
    fn rank(&self) -> u8 {
        match self {
            DefectSpec::Holes { .. } => 0,
            DefectSpec::UnweldedSeam { .. } => 1,
            DefectSpec::FlippedFaces { .. } => 2,
            DefectSpec::DuplicateFaces { .. } => 3,
            DefectSpec::ZeroAreaSlivers { .. } => 4,
            DefectSpec::NeedleSlivers { .. } => 5,
            DefectSpec::NonManifoldFins { .. } => 6,
            DefectSpec::BowTies { .. } => 7,
            DefectSpec::OverlappingShell => 8,
            DefectSpec::InvertedShell => 9,
            DefectSpec::FloatingShells { .. } => 10,
            DefectSpec::F32Rounded => 11,
        }
    }
}

/// `(shape, defects, seed)`: everything the generator depends on.
#[derive(Clone, Debug, PartialEq)]
pub struct SynthSpec {
    /// Base shape.
    pub shape: Shape,
    /// Defects to inject.
    pub defects: Vec<DefectSpec>,
    /// RNG seed.
    pub seed: u64,
}

/// What was injected, by construction. Face and vertex ids refer to the returned mesh.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GroundTruth {
    /// Vertices of the clean base shape.
    pub base_vertices: usize,
    /// Faces of the clean base shape.
    pub base_faces: usize,
    /// Boundary loop lengths of the holes, sorted (measured on the hole step's output).
    pub hole_loops: Vec<usize>,
    /// `(original, upper copy)` vertex pairs of the seam.
    pub seam_pairs: Vec<(u32, u32)>,
    /// Jitter applied to the upper copies.
    pub seam_jitter: f64,
    /// Flipped faces.
    pub flipped_faces: Vec<u32>,
    /// `(original, duplicate)` face pairs.
    pub duplicate_faces: Vec<(u32, u32)>,
    /// Faces with exactly collinear corners.
    pub zero_area_faces: Vec<u32>,
    /// Needle slivers (positive area, aspect ratio about 1e6).
    pub needle_faces: Vec<u32>,
    /// Number of fins.
    pub fins: usize,
    /// Pinched vertices (their second fan came from another vertex of the base shape).
    pub bow_tie_vertices: Vec<u32>,
    /// Edges with more than two faces: 3 per duplicate face, 1 per fin.
    pub non_manifold_edges: usize,
    /// Vertices with more than one fan.
    pub non_manifold_vertices: usize,
    /// Appended shells that cross the base shell.
    pub overlapping_shells: usize,
    /// Appended shells with reversed winding.
    pub inverted_shells: usize,
    /// Appended tiny tetrahedra.
    pub floating_shells: usize,
    /// Connected components once the seam is welded.
    pub shells: usize,
    /// Offset applied before rounding to `f32`.
    pub f32_offset: Option<Vec3>,
}

impl GroundTruth {
    /// `true` if some triangles cross each other by construction.
    pub fn has_self_intersections(&self) -> bool {
        self.overlapping_shells > 0
    }
}

/// Generator output.
#[derive(Clone, Debug)]
pub struct Synth {
    /// The broken mesh.
    pub mesh: IndexedMesh,
    /// What was injected.
    pub truth: GroundTruth,
}

// ---------------------------------------------------------------------------------------------
// Base shapes

fn finish(positions: Vec<Point3>, tris: Vec<[u32; 3]>) -> IndexedMesh {
    IndexedMesh {
        groups: FaceGroups::new(tris.len()),
        positions,
        tris,
        attrs: MeshAttrs::default(),
    }
}

fn orient_outward(mut m: IndexedMesh) -> IndexedMesh {
    if m.signed_volume() < 0.0 {
        for t in &mut m.tris {
            t.swap(1, 2);
        }
    }
    m
}

fn torus(nu: u32, nv: u32, major: f64, minor: f64) -> IndexedMesh {
    let mut p = Vec::new();
    for i in 0..nu {
        let u = f64::from(i) / f64::from(nu) * std::f64::consts::TAU;
        for j in 0..nv {
            let v = f64::from(j) / f64::from(nv) * std::f64::consts::TAU;
            let r = major + minor * v.cos();
            p.push(Point3::new(r * u.cos(), r * u.sin(), minor * v.sin()));
        }
    }
    let id = |i: u32, j: u32| (i % nu) * nv + (j % nv);
    let mut t = Vec::new();
    for i in 0..nu {
        for j in 0..nv {
            let (a, b, c, d) = (id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
            t.push([a, b, c]);
            t.push([a, c, d]);
        }
    }
    orient_outward(finish(p, t))
}

fn plate(n: u32, m: u32, size: f64) -> IndexedMesh {
    let h = size / 4.0;
    let s = size / f64::from(n);
    let stride = n + 1;
    let layer = stride * stride;
    let vid = |l: u32, x: u32, y: u32| l * layer + y * stride + x;
    let mut pos = Vec::with_capacity(2 * layer as usize);
    for l in 0..2 {
        for y in 0..=n {
            for x in 0..=n {
                pos.push(Point3::new(
                    f64::from(x) * s,
                    f64::from(y) * s,
                    f64::from(l) * h,
                ));
            }
        }
    }
    let is_hole = |x: i64, y: i64| -> bool {
        if m == 0 || x < 2 || y < 2 {
            return false;
        }
        let (a, b) = ((x - 2), (y - 2));
        a % 3 == 0 && b % 3 == 0 && a / 3 < i64::from(m) && b / 3 < i64::from(m)
    };
    let outside = |x: i64, y: i64| x < 0 || y < 0 || x >= i64::from(n) || y >= i64::from(n);
    let mut t = Vec::new();
    for y in 0..n {
        for x in 0..n {
            if is_hole(i64::from(x), i64::from(y)) {
                continue;
            }
            let c = |dx: u32, dy: u32, l: u32| vid(l, x + dx, y + dy);
            // Top (l = 1) counter-clockwise from +z; bottom reversed.
            t.push([c(0, 0, 1), c(1, 0, 1), c(1, 1, 1)]);
            t.push([c(0, 0, 1), c(1, 1, 1), c(0, 1, 1)]);
            t.push([c(0, 0, 0), c(1, 1, 0), c(1, 0, 0)]);
            t.push([c(0, 0, 0), c(0, 1, 0), c(1, 1, 0)]);
            let (xi, yi) = (i64::from(x), i64::from(y));
            // Cell edges in counter-clockwise order with the neighbour across each.
            let edges = [
                ((0, 0), (1, 0), (xi, yi - 1)),
                ((1, 0), (1, 1), (xi + 1, yi)),
                ((1, 1), (0, 1), (xi, yi + 1)),
                ((0, 1), (0, 0), (xi - 1, yi)),
            ];
            for ((px, py), (qx, qy), (nx, ny)) in edges {
                if outside(nx, ny) || is_hole(nx, ny) {
                    let (p0, q0) = (c(px, py, 0), c(qx, qy, 0));
                    let (p1, q1) = (c(px, py, 1), c(qx, qy, 1));
                    t.push([p0, q0, q1]);
                    t.push([p0, q1, p1]);
                }
            }
        }
    }
    orient_outward(finish(pos, t))
}

fn torus_knot(segments: u32, sides: u32) -> IndexedMesh {
    let (p, q, big, small, tube) = (2.0, 3.0, 10.0, 5.0, 1.0);
    let mut pos = Vec::new();
    for i in 0..segments {
        let t = f64::from(i) / f64::from(segments) * std::f64::consts::TAU;
        let radial = Vec3::new((q * t).cos(), (q * t).sin(), 0.0);
        let tang = Vec3::new(-(q * t).sin(), (q * t).cos(), 0.0);
        let (cp, sp) = ((p * t).cos(), (p * t).sin());
        let k = radial * (big + small * cp) + Vec3::Z * (small * sp);
        let d = radial * (-small * p * sp)
            + tang * ((big + small * cp) * q)
            + Vec3::Z * (small * p * cp);
        let nrm = radial * cp + Vec3::Z * sp;
        let tn = d.normalize();
        let bn = tn.cross(nrm).normalize();
        for j in 0..sides {
            let a = f64::from(j) / f64::from(sides) * std::f64::consts::TAU;
            pos.push(k + (nrm * a.cos() + bn * a.sin()) * tube);
        }
    }
    let id = |i: u32, j: u32| (i % segments) * sides + (j % sides);
    let mut tris = Vec::new();
    for i in 0..segments {
        for j in 0..sides {
            let (a, b, c, d) = (id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }
    orient_outward(finish(pos, tris))
}

/// The clean base shape of a spec.
///
/// # Panics
/// On parameters outside the documented ranges.
pub fn base_shape(shape: &Shape, seed: u64) -> IndexedMesh {
    match *shape {
        Shape::Icosphere { frequency, radius } => geodesic_sphere(frequency, radius, None),
        Shape::DisplacedSphere {
            frequency,
            radius,
            amplitude,
        } => geodesic_sphere(frequency, radius, Some(Displacement { amplitude, seed })),
        Shape::Torus {
            major_segments,
            minor_segments,
            major,
            minor,
        } => {
            assert!(major_segments >= 3 && minor_segments >= 3);
            torus(major_segments, minor_segments, major, minor)
        }
        Shape::BoxWithHoles {
            cells,
            holes_per_side,
            size,
        } => {
            assert!(cells > 3 * holes_per_side && cells >= 2);
            plate(cells, holes_per_side, size)
        }
        Shape::TorusKnot { segments, sides } => {
            assert!(segments >= 64 && sides >= 3);
            torus_knot(segments, sides)
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Injection

struct Work {
    pos: Vec<Point3>,
    tris: Vec<[u32; 3]>,
    locked: Vec<bool>,
    rng: ChaCha8Rng,
    truth: GroundTruth,
}

/// Vertex -> faces in CSR form.
fn vertex_faces(nv: usize, tris: &[[u32; 3]]) -> (Vec<u32>, Vec<u32>) {
    let mut off = vec![0u32; nv + 1];
    for t in tris {
        for &v in t {
            off[v as usize + 1] += 1;
        }
    }
    for i in 0..nv {
        off[i + 1] += off[i];
    }
    let mut fill = off.clone();
    let mut faces = vec![0u32; tris.len() * 3];
    for (f, t) in tris.iter().enumerate() {
        for &v in t {
            faces[fill[v as usize] as usize] = f as u32;
            fill[v as usize] += 1;
        }
    }
    (off, faces)
}

fn star<'a>(off: &[u32], faces: &'a [u32], v: u32) -> &'a [u32] {
    &faces[off[v as usize] as usize..off[v as usize + 1] as usize]
}

/// Face across each directed edge `t[k] -> t[k+1]` (`NONE` if open or non-manifold).
fn face_neighbors(tris: &[[u32; 3]]) -> Vec<[u32; 3]> {
    let mut map: HashMap<(u32, u32), u32> = HashMap::with_capacity(tris.len() * 3);
    for (f, t) in tris.iter().enumerate() {
        for k in 0..3 {
            map.insert((t[k], t[(k + 1) % 3]), f as u32);
        }
    }
    tris.iter()
        .map(|t| [0, 1, 2].map(|k| map.get(&(t[(k + 1) % 3], t[k])).copied().unwrap_or(NONE)))
        .collect()
}

impl Work {
    fn unit(&mut self) -> f64 {
        (self.rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn below(&mut self, n: usize) -> usize {
        (self.rng.next_u64() % n.max(1) as u64) as usize
    }

    fn shuffled(&mut self, n: usize) -> Vec<u32> {
        let mut v: Vec<u32> = (0..n as u32).collect();
        for i in (1..n).rev() {
            let j = self.below(i + 1);
            v.swap(i, j);
        }
        v
    }

    fn bbox(&self) -> (Point3, Point3) {
        let mut lo = Point3::splat(f64::INFINITY);
        let mut hi = Point3::splat(f64::NEG_INFINITY);
        for p in &self.pos {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
        (lo, hi)
    }

    fn lock_neighbourhood(&mut self, faces: &[u32]) {
        let (off, vf) = vertex_faces(self.pos.len(), &self.tris);
        for &f in faces {
            for v in self.tris[f as usize] {
                for &g in star(&off, &vf, v) {
                    self.locked[g as usize] = true;
                }
            }
        }
    }

    /// `count` unlocked faces, pairwise vertex-disjoint and disjoint from earlier defects; they
    /// and their neighbourhoods are locked.
    fn pick_free(&mut self, count: u32) -> Vec<u32> {
        let (off, vf) = vertex_faces(self.pos.len(), &self.tris);
        let order = self.shuffled(self.tris.len());
        let mut out = Vec::new();
        for f in order {
            if out.len() == count as usize {
                break;
            }
            if self.locked[f as usize] {
                continue;
            }
            out.push(f);
            for v in self.tris[f as usize] {
                for &g in star(&off, &vf, v) {
                    self.locked[g as usize] = true;
                }
            }
        }
        out
    }

    fn push_vertex(&mut self, p: Point3) -> u32 {
        self.pos.push(p);
        (self.pos.len() - 1) as u32
    }

    fn push_face(&mut self, t: [u32; 3]) -> u32 {
        self.tris.push(t);
        self.locked.push(true);
        (self.tris.len() - 1) as u32
    }

    fn holes(&mut self, count: u32, max_boundary: u32) {
        let nbr = face_neighbors(&self.tris);
        let limit = (self.tris.len() / 10).max(1);
        let mut removed = vec![false; self.tris.len()];
        let mut loops_all: Vec<usize> = Vec::new();
        let max_b = f64::from(max_boundary.max(3));
        for _ in 0..count {
            let target = (3.0f64.ln() + self.unit() * (max_b.ln() - 3.0f64.ln())).exp() as usize;
            for _attempt in 0..30 {
                let seed = self.below(self.tris.len()) as u32;
                if self.locked[seed as usize] || removed[seed as usize] {
                    continue;
                }
                let mut set: Vec<u32> = vec![seed];
                let mut inset: HashMap<u32, ()> = HashMap::from([(seed, ())]);
                let mut b = 3usize;
                let mut head = 0;
                while b < target && head < set.len() && set.len() < limit {
                    let f = set[head];
                    head += 1;
                    for k in 0..3 {
                        let g = nbr[f as usize][k];
                        if g == NONE
                            || inset.contains_key(&g)
                            || self.locked[g as usize]
                            || removed[g as usize]
                        {
                            continue;
                        }
                        let inside = nbr[g as usize]
                            .iter()
                            .filter(|&&h| h != NONE && inset.contains_key(&h))
                            .count();
                        b = b + 3 - 2 * inside;
                        set.push(g);
                        inset.insert(g, ());
                        if b >= target {
                            break;
                        }
                    }
                }
                // Boundary half-edges of the remaining faces next to the patch.
                let mut next: HashMap<u32, u32> = HashMap::new();
                let mut ok = true;
                for &f in &set {
                    for k in 0..3 {
                        let g = nbr[f as usize][k];
                        if g == NONE || inset.contains_key(&g) {
                            continue;
                        }
                        let t = self.tris[f as usize];
                        // g owns the directed edge t[k+1] -> t[k]
                        if next.insert(t[(k + 1) % 3], t[k]).is_some() {
                            ok = false;
                        }
                    }
                }
                if !ok || next.is_empty() {
                    continue;
                }
                let start = *next.keys().min().unwrap_or(&0);
                let (mut n, mut cur) = (0usize, start);
                while let Some(&nx) = next.get(&cur) {
                    n += 1;
                    cur = nx;
                    if cur == start || n > next.len() {
                        break;
                    }
                }
                if cur != start || n != next.len() {
                    continue; // several loops or a non-simple loop
                }
                // Neighbourhood must be free.
                let (off, vf) = vertex_faces(self.pos.len(), &self.tris);
                let mut free = true;
                let mut around: Vec<u32> = Vec::new();
                for &f in &set {
                    for v in self.tris[f as usize] {
                        for &g in star(&off, &vf, v) {
                            if (self.locked[g as usize] || removed[g as usize])
                                && !inset.contains_key(&g)
                            {
                                free = false;
                            }
                            around.push(g);
                        }
                    }
                }
                if !free {
                    continue;
                }
                for &f in &set {
                    removed[f as usize] = true;
                }
                for g in around {
                    self.locked[g as usize] = true;
                }
                loops_all.push(n);
                break;
            }
        }
        // Compact faces and vertices.
        let mut new_tris = Vec::with_capacity(self.tris.len());
        let mut new_locked = Vec::with_capacity(self.tris.len());
        for (f, t) in self.tris.iter().enumerate() {
            if !removed[f] {
                new_tris.push(*t);
                new_locked.push(self.locked[f]);
            }
        }
        let mut map = vec![NONE; self.pos.len()];
        let mut new_pos = Vec::new();
        for t in &mut new_tris {
            for v in t.iter_mut() {
                if map[*v as usize] == NONE {
                    map[*v as usize] = new_pos.len() as u32;
                    new_pos.push(self.pos[*v as usize]);
                }
                *v = map[*v as usize];
            }
        }
        self.tris = new_tris;
        self.locked = new_locked;
        self.pos = new_pos;
        loops_all.sort_unstable();
        self.truth.hole_loops = loops_all;
    }

    fn seam(&mut self, jitter: f64) {
        let (lo, hi) = self.bbox();
        let z0 = lo.z + (0.4 + 0.2 * self.unit()) * (hi.z - lo.z);
        let upper: Vec<bool> = self
            .tris
            .iter()
            .map(|t| {
                (self.pos[t[0] as usize].z + self.pos[t[1] as usize].z + self.pos[t[2] as usize].z)
                    / 3.0
                    > z0
            })
            .collect();
        let nv = self.pos.len();
        let (mut used_up, mut used_low) = (vec![false; nv], vec![false; nv]);
        for (f, t) in self.tris.iter().enumerate() {
            for &v in t {
                if upper[f] {
                    used_up[v as usize] = true;
                } else {
                    used_low[v as usize] = true;
                }
            }
        }
        let mut copy = vec![NONE; nv];
        let mut pairs = Vec::new();
        for v in 0..nv {
            if used_up[v] && used_low[v] {
                let d = Vec3::new(
                    2.0 * self.unit() - 1.0,
                    2.0 * self.unit() - 1.0,
                    2.0 * self.unit() - 1.0,
                ) * jitter;
                let p = self.pos[v] + d;
                let id = self.push_vertex(p);
                copy[v] = id;
                pairs.push((v as u32, id));
            }
        }
        let mut touched: Vec<u32> = Vec::new();
        for (t, _) in self.tris.iter_mut().zip(&upper).filter(|(_, u)| **u) {
            for v in t.iter_mut() {
                if copy[*v as usize] != NONE {
                    *v = copy[*v as usize];
                }
            }
        }
        for (f, t) in self.tris.iter().enumerate() {
            if t.iter()
                .any(|&v| v as usize >= nv || copy[v as usize] != NONE)
            {
                touched.push(f as u32);
            }
        }
        self.lock_neighbourhood(&touched);
        self.truth.seam_pairs = pairs;
        self.truth.seam_jitter = jitter;
    }

    fn flip(&mut self, count: u32) {
        for f in self.pick_free(count) {
            self.tris[f as usize].swap(1, 2);
            self.truth.flipped_faces.push(f);
        }
    }

    fn duplicate(&mut self, count: u32) {
        for f in self.pick_free(count) {
            let d = self.push_face(self.tris[f as usize]);
            self.truth.duplicate_faces.push((f, d));
            self.truth.non_manifold_edges += 3;
        }
    }

    fn sliver(&mut self, count: u32, needle: bool) {
        for f in self.pick_free(count) {
            let [a, b, c] = self.tris[f as usize];
            let (pa, pb, pc) = (
                self.pos[a as usize],
                self.pos[b as usize],
                self.pos[c as usize],
            );
            let mid = (pa + pb) * 0.5;
            let p = if needle {
                let towards = (pc - mid).normalize_or_zero();
                mid + towards * (1e-6 * (pb - pa).length())
            } else {
                mid
            };
            let m = self.push_vertex(p);
            self.tris[f as usize] = [a, m, c];
            self.locked[f as usize] = true;
            self.push_face([m, b, c]);
            let s = self.push_face([a, b, m]);
            if needle {
                self.truth.needle_faces.push(s);
            } else {
                self.truth.zero_area_faces.push(s);
            }
        }
    }

    fn fins(&mut self, count: u32) {
        for f in self.pick_free(count) {
            let [u, v, w] = self.tris[f as usize];
            let (pu, pv, pw) = (
                self.pos[u as usize],
                self.pos[v as usize],
                self.pos[w as usize],
            );
            let n = (pv - pu).cross(pw - pu).normalize_or_zero();
            let apex = self.push_vertex((pu + pv) * 0.5 + n * (0.5 * (pv - pu).length()));
            self.push_face([u, v, apex]);
            self.push_face([v, u, apex]);
            self.truth.fins += 1;
            self.truth.non_manifold_edges += 1;
        }
    }

    fn bow_ties(&mut self, count: u32) {
        let (off, vf) = vertex_faces(self.pos.len(), &self.tris);
        let nv = self.pos.len();
        let star_free =
            |w: &Work, v: u32| star(&off, &vf, v).iter().all(|&g| !w.locked[g as usize]);
        let order = self.shuffled(nv);
        let mut done = 0;
        for v in order {
            if done == count {
                break;
            }
            if off[v as usize] == off[v as usize + 1] || !star_free(self, v) {
                continue;
            }
            // Breadth-first distances over vertex adjacency, up to 3.
            let mut dist: HashMap<u32, u32> = HashMap::from([(v, 0)]);
            let mut frontier = vec![v];
            for d in 1..=3u32 {
                let mut nxt = Vec::new();
                for &x in &frontier {
                    for &g in star(&off, &vf, x) {
                        for y in self.tris[g as usize] {
                            if let std::collections::hash_map::Entry::Vacant(e) = dist.entry(y) {
                                e.insert(d);
                                nxt.push(y);
                            }
                        }
                    }
                }
                nxt.sort_unstable();
                frontier = nxt;
            }
            let cands: Vec<u32> = frontier
                .into_iter()
                .filter(|&w| star_free(self, w))
                .collect();
            if cands.is_empty() {
                continue;
            }
            let w = cands[self.below(cands.len())];
            let faces_w: Vec<u32> = star(&off, &vf, w).to_vec();
            let faces_v: Vec<u32> = star(&off, &vf, v).to_vec();
            for &g in &faces_w {
                for x in &mut self.tris[g as usize] {
                    if *x == w {
                        *x = v;
                    }
                }
            }
            for &g in faces_v.iter().chain(&faces_w) {
                for y in self.tris[g as usize] {
                    for &h in star(&off, &vf, y) {
                        self.locked[h as usize] = true;
                    }
                }
            }
            self.truth.bow_tie_vertices.push(v);
            self.truth.non_manifold_vertices += 1;
            done += 1;
        }
    }

    fn append_shell(&mut self, base: &IndexedMesh, f: impl Fn(Point3) -> Point3, reverse: bool) {
        let at = self.pos.len() as u32;
        for &p in &base.positions {
            self.pos.push(f(p));
        }
        for t in &base.tris {
            let mut t = t.map(|i| i + at);
            if reverse {
                t.swap(1, 2);
            }
            self.push_face(t);
        }
    }
}

/// Generates the broken mesh and its ground truth.
///
/// # Panics
/// On shape parameters outside their documented ranges.
pub fn generate(spec: &SynthSpec) -> Synth {
    let base = base_shape(&spec.shape, spec.seed);
    let mut w = Work {
        pos: base.positions.clone(),
        tris: base.tris.clone(),
        locked: vec![false; base.tris.len()],
        rng: ChaCha8Rng::seed_from_u64(spec.seed ^ 0x004D_4553_482D_3238),
        truth: GroundTruth {
            base_vertices: base.positions.len(),
            base_faces: base.tris.len(),
            shells: 1,
            ..GroundTruth::default()
        },
    };
    let (blo, bhi) = {
        let bb = base.aabb();
        (bb.min, bb.max)
    };
    let size = bhi - blo;
    let centre = (blo + bhi) * 0.5;
    let mut defects = spec.defects.clone();
    defects.sort_by_key(DefectSpec::rank);
    for d in &defects {
        match *d {
            DefectSpec::Holes {
                count,
                max_boundary,
            } => w.holes(count, max_boundary),
            DefectSpec::UnweldedSeam { jitter } => w.seam(jitter),
            DefectSpec::FlippedFaces { count } => w.flip(count),
            DefectSpec::DuplicateFaces { count } => w.duplicate(count),
            DefectSpec::ZeroAreaSlivers { count } => w.sliver(count, false),
            DefectSpec::NeedleSlivers { count } => w.sliver(count, true),
            DefectSpec::NonManifoldFins { count } => w.fins(count),
            DefectSpec::BowTies { count } => w.bow_ties(count),
            DefectSpec::OverlappingShell => {
                let shift = Vec3::new(0.2 * size.x, 0.1 * size.y, 0.05 * size.z);
                w.append_shell(&base, |p| p + shift, false);
                w.truth.overlapping_shells += 1;
                w.truth.shells += 1;
            }
            DefectSpec::InvertedShell => {
                let shift = Vec3::new(1.5 * size.x, 0.0, 0.0);
                w.append_shell(&base, |p| centre + (p - centre) * 0.25 + shift, true);
                w.truth.inverted_shells += 1;
                w.truth.shells += 1;
            }
            DefectSpec::FloatingShells { count } => {
                let e = 1e-3 * size.length();
                for _ in 0..count {
                    let o = Point3::new(
                        bhi.x + (0.1 + w.unit()) * size.x,
                        bhi.y + (0.1 + w.unit()) * size.y,
                        bhi.z + (0.1 + w.unit()) * size.z,
                    );
                    let at = w.pos.len() as u32;
                    for dv in [Vec3::ZERO, Vec3::X * e, Vec3::Y * e, Vec3::Z * e] {
                        w.pos.push(o + dv);
                    }
                    for t in [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]] {
                        w.push_face(t.map(|i| i + at));
                    }
                    w.truth.floating_shells += 1;
                    w.truth.shells += 1;
                }
            }
            DefectSpec::F32Rounded => {
                let off = Vec3::new(1000.0, -1000.0, 1000.0);
                for p in &mut w.pos {
                    let q = *p + off;
                    *p = Point3::new(
                        f64::from(q.x as f32),
                        f64::from(q.y as f32),
                        f64::from(q.z as f32),
                    );
                }
                w.truth.f32_offset = Some(off);
            }
        }
    }
    Synth {
        mesh: finish(w.pos, w.tris),
        truth: w.truth,
    }
}
