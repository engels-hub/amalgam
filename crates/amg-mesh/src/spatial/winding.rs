//! Fast generalized winding number (Barill, Dickson, Schmidt, Levin & Jacobson 2018).
//!
//! The winding number of a surface seen from `q` is
//! `w(q) = 1/4π ∮ n(x)·(x−q)/|x−q|³ dS`. For a cluster of triangles far from `q`, the kernel
//! `f(x) = (x−q)/|x−q|³` is replaced by its Taylor expansion about the cluster's area-weighted
//! centroid `p`, truncated at order 2:
//!
//! ```text
//! w ≈ 1/4π [ N_k f_k + M_kj ∂_j f_k + ½ T_kjl ∂_j ∂_l f_k ]
//! N_k = ∮ n_k dS, M_kj = ∮ n_k (x−p)_j dS, T_kjl = ∮ n_k (x−p)_j (x−p)_l dS
//! ```
//!
//! The moments of a triangle are exact (a degree-2 quadrature), and moments of an inner node
//! are obtained from its children by exact translation, so building costs O(n). The far-field
//! test is `|q − p| > β r` with `r` a conservative bound of the cluster radius.

use amg_core::Point3;
use amg_jobs::JobCtx;
use rayon::prelude::*;

use super::bvh::{Bvh, STACK};
use super::tri::solid_angle_over_4pi;
use crate::error::MeshResult;
use crate::indexed::IndexedMesh;

/// Nodes covering at most this many triangles are not given an expansion (cheaper to descend).
const STORE_MIN: u32 = 8;
/// Subtrees with more triangles than this are expanded in parallel.
const PAR_MIN: u32 = 4096;
const NONE: u32 = u32::MAX;

/// Index of the symmetric pair `(j, l)` in a 6-vector.
const SYM: [[usize; 3]; 3] = [[0, 3, 4], [3, 1, 5], [4, 5, 2]];

/// Order-2 dipole expansion of a triangle cluster.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Expansion {
    c: [f64; 3],
    r: f64,
    area: f64,
    n: [f64; 3],
    m: [[f64; 3]; 3],
    t: [[f64; 6]; 3],
}

impl Expansion {
    fn zero() -> Self {
        Self {
            c: [0.0; 3],
            r: 0.0,
            area: 0.0,
            n: [0.0; 3],
            m: [[0.0; 3]; 3],
            t: [[0.0; 6]; 3],
        }
    }

    /// Expansion of the triangles `tris` of `mesh`.
    fn from_tris(mesh: &IndexedMesh, tris: impl Iterator<Item = u32> + Clone) -> Self {
        let corners = |f: u32| mesh.tris[f as usize].map(|i| mesh.positions[i as usize]);
        let mut area = 0.0;
        let mut csum = Point3::ZERO;
        let mut vsum = Point3::ZERO;
        let mut nv = 0.0;
        for f in tris.clone() {
            let [a, b, c] = corners(f);
            let ar = 0.5 * (b - a).cross(c - a).length();
            let cen = (a + b + c) / 3.0;
            area += ar;
            csum += cen * ar;
            vsum += cen;
            nv += 1.0;
        }
        let p = if area > 0.0 {
            csum / area
        } else if nv > 0.0 {
            vsum / nv
        } else {
            Point3::ZERO
        };
        let mut e = Self::zero();
        e.c = p.to_array();
        e.area = area;
        for f in tris {
            let [a, b, c] = corners(f);
            let av = 0.5 * (b - a).cross(c - a);
            let d = [a - p, b - p, c - p];
            e.r = e.r.max(d[0].length()).max(d[1].length()).max(d[2].length());
            let s = d[0] + d[1] + d[2];
            let g = s / 3.0;
            // E[(x-p)(x-p)^T] over the triangle, symmetric 6-vector.
            let mut q = [0.0; 6];
            for j in 0..3 {
                for l in j..3 {
                    q[SYM[j][l]] =
                        (d[0][j] * d[0][l] + d[1][j] * d[1][l] + d[2][j] * d[2][l] + s[j] * s[l])
                            / 12.0;
                }
            }
            for k in 0..3 {
                e.n[k] += av[k];
                for j in 0..3 {
                    e.m[k][j] += av[k] * g[j];
                }
                for (tt, qq) in e.t[k].iter_mut().zip(q) {
                    *tt += av[k] * qq;
                }
            }
        }
        e
    }

    /// Union of two expansions, re-centred on the combined area-weighted centroid.
    fn combine(a: &Self, b: &Self) -> Self {
        let area = a.area + b.area;
        let mut c = [0.0; 3];
        for (i, ci) in c.iter_mut().enumerate() {
            *ci = if area > 0.0 {
                (a.area * a.c[i] + b.area * b.c[i]) / area
            } else {
                0.5 * (a.c[i] + b.c[i])
            };
        }
        let mut out = Self::zero();
        out.c = c;
        out.area = area;
        for part in [a, b] {
            let s = [part.c[0] - c[0], part.c[1] - c[1], part.c[2] - c[2]];
            let dist = (s[0] * s[0] + s[1] * s[1] + s[2] * s[2]).sqrt();
            out.r = out.r.max(dist + part.r);
            for k in 0..3 {
                out.n[k] += part.n[k];
                for (j, sj) in s.iter().enumerate() {
                    out.m[k][j] += part.m[k][j] + part.n[k] * sj;
                }
                for j in 0..3 {
                    for l in j..3 {
                        out.t[k][SYM[j][l]] += part.t[k][SYM[j][l]]
                            + part.m[k][j] * s[l]
                            + part.m[k][l] * s[j]
                            + part.n[k] * s[j] * s[l];
                    }
                }
            }
        }
        out
    }

    /// Far-field winding number contribution at `q`.
    #[inline]
    fn far(&self, q: Point3) -> f64 {
        let d = [self.c[0] - q.x, self.c[1] - q.y, self.c[2] - q.z];
        let rho2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        let rho = rho2.sqrt();
        let i3 = 1.0 / (rho2 * rho);
        let i5 = i3 / rho2;
        let i7 = i5 / rho2;
        let mut w = 0.0;
        for k in 0..3 {
            // order 0
            w += self.n[k] * d[k] * i3;
            // order 1: ∂_j f_k = δ_kj/ρ³ − 3 D_k D_j/ρ⁵
            for j in 0..3 {
                let dkj = if k == j { i3 } else { 0.0 };
                w += self.m[k][j] * (dkj - 3.0 * d[k] * d[j] * i5);
            }
            // order 2: ∂_j∂_l f_k = −3(δ_kj D_l + δ_kl D_j + δ_jl D_k)/ρ⁵ + 15 D_k D_j D_l/ρ⁷
            for j in 0..3 {
                for l in j..3 {
                    let mut dd = 0.0;
                    if k == j {
                        dd += d[l];
                    }
                    if k == l {
                        dd += d[j];
                    }
                    if j == l {
                        dd += d[k];
                    }
                    let deriv = -3.0 * dd * i5 + 15.0 * d[k] * d[j] * d[l] * i7;
                    let mult = if j == l { 0.5 } else { 1.0 };
                    w += mult * self.t[k][SYM[j][l]] * deriv;
                }
            }
        }
        w / (4.0 * std::f64::consts::PI)
    }
}

/// Expansions for the BVH nodes that cover more than [`STORE_MIN`] triangles.
#[derive(Debug)]
pub(crate) struct WindingTree {
    /// Per BVH node: ordinal into `exp`, or `NONE`.
    ord: Vec<u32>,
    exp: Vec<Expansion>,
}

struct Builder<'a> {
    bvh: &'a Bvh,
    mesh: &'a IndexedMesh,
    ord: &'a [u32],
    cnt: &'a [u32],
}

impl Builder<'_> {
    fn go(&self, i: u32, slice: &mut [Expansion]) -> Expansion {
        let n = &self.bvh.nodes[i as usize];
        let own = if n.is_leaf() {
            let f = n.left_or_first as usize;
            Expansion::from_tris(
                self.mesh,
                self.bvh.prim_indices[f..f + n.count as usize]
                    .iter()
                    .copied(),
            )
        } else {
            let (l, r) = (i + 1, n.left_or_first);
            let (_, rest) = if self.ord[i as usize] != NONE {
                slice.split_at_mut(1)
            } else {
                slice.split_at_mut(0)
            };
            let left_len = if self.ord[r as usize] != NONE {
                (self.ord[r as usize] - self.ord[i as usize] - 1) as usize
            } else {
                rest.len()
            };
            let (ls, rs) = rest.split_at_mut(left_len);
            let (el, er) = if self.cnt[i as usize] > PAR_MIN {
                rayon::join(|| self.go(l, ls), || self.go(r, rs))
            } else {
                (self.go(l, ls), self.go(r, rs))
            };
            Expansion::combine(&el, &er)
        };
        if self.ord[i as usize] != NONE {
            slice[0] = own;
        }
        own
    }
}

impl WindingTree {
    pub fn build(bvh: &Bvh, mesh: &IndexedMesh, ctx: &JobCtx) -> MeshResult<Self> {
        let nn = bvh.nodes.len();
        if nn == 0 {
            return Ok(Self {
                ord: Vec::new(),
                exp: Vec::new(),
            });
        }
        let mut cnt = vec![0u32; nn];
        for i in (0..nn).rev() {
            let n = &bvh.nodes[i];
            cnt[i] = if n.is_leaf() {
                n.count
            } else {
                cnt[i + 1] + cnt[n.left_or_first as usize]
            };
        }
        let mut ord = vec![NONE; nn];
        let mut stored = 0u32;
        for i in 0..nn {
            if cnt[i] > STORE_MIN {
                ord[i] = stored;
                stored += 1;
            }
        }
        ctx.checkpoint()?;
        let mut exp = vec![Expansion::zero(); stored as usize];
        let b = Builder {
            bvh,
            mesh,
            ord: &ord,
            cnt: &cnt,
        };
        b.go(0, &mut exp);
        ctx.checkpoint()?;
        Ok(Self { ord, exp })
    }

    /// Winding number at `q` with accuracy parameter `beta` (larger is more accurate).
    pub fn eval(&self, bvh: &Bvh, mesh: &IndexedMesh, q: Point3, beta: f64) -> f64 {
        if bvh.nodes.is_empty() {
            return 0.0;
        }
        let mut w = 0.0;
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let i = stack[sp];
            let o = self.ord[i as usize];
            if o != NONE {
                let e = &self.exp[o as usize];
                let d = Point3::from_array(e.c) - q;
                let lim = beta * e.r;
                if d.length_squared() > lim * lim {
                    w += e.far(q);
                    continue;
                }
            }
            let n = &bvh.nodes[i as usize];
            if n.is_leaf() {
                let f = n.left_or_first as usize;
                for &face in &bvh.prim_indices[f..f + n.count as usize] {
                    let [a, b, c] = mesh.tris[face as usize].map(|v| mesh.positions[v as usize]);
                    w += solid_angle_over_4pi(q, a, b, c);
                }
            } else {
                stack[sp] = n.left_or_first;
                stack[sp + 1] = i + 1;
                sp += 2;
            }
        }
        w
    }

    /// Parallel batch evaluation (each result is independent of the thread count).
    pub fn eval_batch(&self, bvh: &Bvh, mesh: &IndexedMesh, pts: &[Point3], beta: f64) -> Vec<f64> {
        pts.par_iter()
            .map(|&q| self.eval(bvh, mesh, q, beta))
            .collect()
    }
}
