//! Liepa's refinement: split triangles whose centroid is farther from its corners than the
//! local edge length, then relax interior edges by Delaunay flips, until nothing splits.

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

use amg_core::{Point3, Vec3};
use amg_jobs::JobCtx;

use crate::MeshResult;

/// The triangulated hole polygon under construction. Vertices `0..n_loop` are the loop (fixed),
/// the rest are new.
pub(super) struct Patch {
    pub pos: Vec<Point3>,
    pub tris: Vec<[u32; 3]>,
    /// Target edge length per vertex (Liepa's sigma).
    pub sigma: Vec<f64>,
    pub n_loop: usize,
}

#[derive(Default)]
struct KeyHasher(u64);

impl Hasher for KeyHasher {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
        }
    }
    fn write_u64(&mut self, v: u64) {
        self.0 = (v ^ (v >> 29)).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
}

type EdgeMap = HashMap<u64, u32, BuildHasherDefault<KeyHasher>>;

fn key(a: u32, b: u32) -> u64 {
    (u64::from(a) << 32) | u64::from(b)
}

const MAX_SWEEPS: usize = 40;

struct Work<'a> {
    patch: &'a mut Patch,
    edges: EdgeMap,
    forbidden: &'a dyn Fn(usize, usize) -> bool,
    /// Edges whose flip test is stale (rim of every split since the last relaxation).
    dirty: Vec<(u32, u32)>,
}

impl Work<'_> {
    fn add_edges(&mut self, t: usize) {
        let v = self.patch.tris[t];
        for k in 0..3 {
            self.edges.insert(key(v[k], v[(k + 1) % 3]), t as u32);
        }
    }

    fn remove_edges(&mut self, t: usize) {
        let v = self.patch.tris[t];
        for k in 0..3 {
            self.edges.remove(&key(v[k], v[(k + 1) % 3]));
        }
    }

    fn normal(&self, a: u32, b: u32, c: u32) -> Vec3 {
        let p = &self.patch.pos;
        (p[b as usize] - p[a as usize]).cross(p[c as usize] - p[a as usize])
    }

    /// Splits triangle `t` at its centroid.
    fn split(&mut self, t: usize, c: Point3, sigma: f64) {
        self.remove_edges(t);
        let [a, b, d] = self.patch.tris[t];
        self.dirty.extend([(a, b), (b, d), (d, a)]);
        let cv = self.patch.pos.len() as u32;
        self.patch.pos.push(c);
        self.patch.sigma.push(sigma);
        self.patch.tris[t] = [a, b, cv];
        self.patch.tris.push([b, d, cv]);
        self.patch.tris.push([d, a, cv]);
        let n = self.patch.tris.len();
        self.add_edges(t);
        self.add_edges(n - 2);
        self.add_edges(n - 1);
    }

    /// Flips the interior edge `a -> b` if the opposite vertex of the other triangle lies in the
    /// sphere whose great circle is the circumcircle of this one. Returns the four rim edges
    /// when it flipped.
    fn try_flip(&mut self, a: u32, b: u32) -> Option<[(u32, u32); 4]> {
        let &tid = self.edges.get(&key(a, b))?;
        let &sid = self.edges.get(&key(b, a))?;
        let (tid, sid) = (tid as usize, sid as usize);
        let third = |t: [u32; 3], p: u32, q: u32| t.into_iter().find(|&v| v != p && v != q);
        let x = third(self.patch.tris[tid], a, b)?;
        let y = third(self.patch.tris[sid], a, b)?;
        if self.edges.contains_key(&key(x, y)) || self.edges.contains_key(&key(y, x)) {
            return None;
        }
        let n = self.patch.n_loop;
        if (x as usize) < n && (y as usize) < n && (self.forbidden)(x as usize, y as usize) {
            return None;
        }
        let p = &self.patch.pos;
        let (pa, pb, px, py) = (p[a as usize], p[b as usize], p[x as usize], p[y as usize]);
        let (u, v) = (pb - pa, px - pa);
        let w = u.cross(v);
        let w2 = w.length_squared();
        if w2 <= 0.0 {
            return None;
        }
        let cc =
            pa + (v.length_squared() * w.cross(u) + u.length_squared() * v.cross(w)) / (2.0 * w2);
        let r2 = (pa - cc).length_squared();
        if (py - cc).length_squared() >= r2 * (1.0 - 1e-9) {
            return None;
        }
        // The flipped pair must stay a proper fold-free pair: both new normals agree with the
        // old ones.
        let old = w + self.normal(b, a, y);
        let (n1, n2) = (self.normal(a, y, x), self.normal(y, b, x));
        let min_area = 1e-9 * old.length();
        if n1.dot(old) <= 0.0
            || n2.dot(old) <= 0.0
            || n1.length() < min_area
            || n2.length() < min_area
        {
            return None;
        }
        self.remove_edges(tid);
        self.remove_edges(sid);
        self.patch.tris[tid] = [a, y, x];
        self.patch.tris[sid] = [y, b, x];
        self.add_edges(tid);
        self.add_edges(sid);
        Some([(a, y), (y, b), (b, x), (x, a)])
    }

    /// Flips edges until none improves; `all` tests every interior edge, otherwise only the
    /// dirty ones.
    fn relax(&mut self, all: bool) {
        let mut stack: Vec<(u32, u32)> = if all {
            self.edges
                .keys()
                .map(|&k| ((k >> 32) as u32, k as u32))
                .filter(|&(a, b)| a < b)
                .collect()
        } else {
            std::mem::take(&mut self.dirty)
        };
        self.dirty.clear();
        stack.sort_unstable();
        stack.dedup();
        let mut budget = 30 * self.patch.tris.len() + 1000;
        while let Some((a, b)) = stack.pop() {
            if budget == 0 {
                break;
            }
            budget -= 1;
            if let Some(rim) = self.try_flip(a, b) {
                stack.extend(rim);
            }
        }
    }
}

/// Refines `patch` in place. `alpha` is the density factor (sqrt 2 in Liepa's paper).
/// `max_vertices` bounds the vertex count of the patch.
pub(super) fn refine(
    patch: &mut Patch,
    alpha: f64,
    max_vertices: usize,
    forbidden: &dyn Fn(usize, usize) -> bool,
    ctx: &JobCtx,
) -> MeshResult<()> {
    if patch.sigma.iter().any(|s| !(*s > 0.0 && s.is_finite())) {
        return Ok(());
    }
    let mut edges = EdgeMap::default();
    edges.reserve(patch.tris.len() * 3);
    let mut w = Work {
        patch,
        edges,
        forbidden,
        dirty: Vec::new(),
    };
    for t in 0..w.patch.tris.len() {
        w.add_edges(t);
    }
    // Flip the initial triangulation first: it is often a zipper of slivers.
    w.relax(true);
    for _ in 0..MAX_SWEEPS {
        ctx.checkpoint()?;
        let nt = w.patch.tris.len();
        let mut split = 0usize;
        for t in 0..nt {
            if w.patch.pos.len() >= max_vertices {
                break;
            }
            let v = w.patch.tris[t];
            let p = &w.patch;
            let c = (p.pos[v[0] as usize] + p.pos[v[1] as usize] + p.pos[v[2] as usize]) / 3.0;
            let sc =
                (p.sigma[v[0] as usize] + p.sigma[v[1] as usize] + p.sigma[v[2] as usize]) / 3.0;
            let ok = v.iter().all(|&i| {
                let d = alpha * (c - p.pos[i as usize]).length();
                d > sc && d > p.sigma[i as usize]
            });
            if ok {
                w.split(t, c, sc);
                split += 1;
            }
        }
        if split == 0 {
            break;
        }
        w.relax(false);
    }
    Ok(())
}
