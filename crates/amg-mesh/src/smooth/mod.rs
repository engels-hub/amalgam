//! Mesh smoothing (MESH-08): uniform / cotangent Laplacian, Taubin λ|μ and bilateral normal
//! filtering (Zheng et al. 2011).
//!
//! All methods are Jacobi iterations (every vertex reads the previous positions), so results do
//! not depend on thread count or vertex order. Topology, attributes and face groups are
//! unchanged; only `positions` move. Vertices that are not selected, that lie on a locked
//! boundary or that are unreferenced stay where they are.

mod adjacency;
mod bilateral;

use amg_core::{Point3, Vec3};
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::error::{MeshError, MeshResult};
use crate::{FaceId, IndexedMesh};
use adjacency::Adjacency;

/// Edge weights of the Laplacian operator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LaplacianWeights {
    /// Every neighbour counts the same (fast; drifts tangentially on irregular meshes).
    Uniform,
    /// Cotangent weights (Desbrun et al. 1999); negative weights are clamped to zero.
    Cotangent,
}

/// Smoothing algorithm.
#[derive(Clone, Copy, Debug)]
pub enum SmoothMethod {
    /// Explicit Laplacian step `p += λ·L(p)` with `0 < λ ≤ 1` (shrinks the mesh).
    Laplacian {
        /// Step size.
        lambda: f64,
        /// Edge weights.
        weights: LaplacianWeights,
    },
    /// Taubin λ|μ (uniform weights): a shrinking step λ followed by an inflating step μ,
    /// with `μ < -λ < 0`. Standard values 0.5 / -0.53.
    Taubin {
        /// Shrinking step, `0 < λ ≤ 1`.
        lambda: f64,
        /// Inflating step, `μ < -λ`.
        mu: f64,
    },
    /// Feature preserving: bilateral filtering of face normals, then vertex update.
    BilateralNormal {
        /// Range sigma on the normal difference `|n_i - n_j|` (unit normals), e.g. 0.35.
        sigma_normal: f64,
        /// Normal filtering passes per outer iteration.
        normal_iterations: u32,
        /// Vertex update passes per outer iteration.
        vertex_iterations: u32,
    },
}

impl SmoothMethod {
    /// Taubin with the standard parameters (0.5, -0.53).
    pub const TAUBIN_DEFAULT: SmoothMethod = SmoothMethod::Taubin {
        lambda: 0.5,
        mu: -0.53,
    };
}

/// Options of [`smooth`].
#[derive(Clone, Debug)]
pub struct SmoothOpts {
    /// Algorithm and its parameters.
    pub method: SmoothMethod,
    /// Outer iterations. Laplacian: steps; Taubin: λ|μ pairs; bilateral: (normal filter +
    /// vertex update) rounds.
    pub iterations: u32,
    /// Keep boundary vertices (and vertices on non-manifold edges) fixed.
    pub fix_boundary: bool,
    /// Keep vertices fixed that touch faces of more than one face group.
    pub preserve_group_borders: bool,
    /// Only vertices of these faces move (`None`: all).
    pub selection: Option<Vec<FaceId>>,
}

impl SmoothOpts {
    /// All vertices may move except boundary vertices; group borders are not preserved.
    pub fn new(method: SmoothMethod, iterations: u32) -> Self {
        SmoothOpts {
            method,
            iterations,
            fix_boundary: true,
            preserve_group_borders: false,
            selection: None,
        }
    }
}

fn invalid(name: &'static str, reason: &str) -> MeshError {
    MeshError::InvalidOption {
        name,
        reason: reason.to_owned(),
    }
}

fn validate(opts: &SmoothOpts) -> MeshResult<()> {
    let step_ok = |l: f64| l > 0.0 && l <= 1.0;
    match opts.method {
        SmoothMethod::Laplacian { lambda, .. } if !step_ok(lambda) => {
            Err(invalid("lambda", "must be in (0, 1]"))
        }
        SmoothMethod::Taubin { lambda, .. } if !step_ok(lambda) => {
            Err(invalid("lambda", "must be in (0, 1]"))
        }
        SmoothMethod::Taubin { lambda, mu } if mu.is_nan() || mu >= -lambda => {
            Err(invalid("mu", "must be below -lambda"))
        }
        SmoothMethod::BilateralNormal { sigma_normal, .. }
            if !(sigma_normal.is_finite() && sigma_normal > 0.0) =>
        {
            Err(invalid("sigma_normal", "must be positive"))
        }
        _ => Ok(()),
    }
}

/// Smooths `m`, returning a mesh with the same topology, attributes and groups.
///
/// # Errors
/// [`MeshError::InvalidOption`] for bad parameters, [`MeshError::IndexOutOfRange`] for bad
/// selection faces, [`MeshError::Cancelled`] if `ctx` is cancelled.
pub fn smooth(m: &IndexedMesh, opts: &SmoothOpts, ctx: &JobCtx) -> MeshResult<IndexedMesh> {
    validate(opts)?;
    let mut out = m.clone();
    if m.tris.is_empty() || opts.iterations == 0 {
        return Ok(out);
    }
    let adj = Adjacency::build(m);
    let movable = movable_mask(m, &adj, opts)?;
    let n_iter = opts.iterations;
    let tick = |i: u32| -> MeshResult<()> {
        ctx.checkpoint().map_err(|_| MeshError::Cancelled)?;
        ctx.progress(i as f32 / n_iter as f32);
        Ok(())
    };
    match opts.method {
        SmoothMethod::Laplacian { lambda, weights } => {
            let mut scratch = out.positions.clone();
            for i in 0..n_iter {
                tick(i)?;
                laplacian_step(
                    &out.positions,
                    &mut scratch,
                    &m.tris,
                    &adj,
                    &movable,
                    lambda,
                    weights,
                );
                std::mem::swap(&mut out.positions, &mut scratch);
            }
        }
        SmoothMethod::Taubin { lambda, mu } => {
            let mut scratch = out.positions.clone();
            for i in 0..n_iter {
                tick(i)?;
                for step in [lambda, mu] {
                    laplacian_step(
                        &out.positions,
                        &mut scratch,
                        &m.tris,
                        &adj,
                        &movable,
                        step,
                        LaplacianWeights::Uniform,
                    );
                    std::mem::swap(&mut out.positions, &mut scratch);
                }
            }
        }
        SmoothMethod::BilateralNormal {
            sigma_normal,
            normal_iterations,
            vertex_iterations,
        } => {
            for i in 0..n_iter {
                tick(i)?;
                bilateral::round(
                    &mut out.positions,
                    &m.tris,
                    &adj,
                    &movable,
                    sigma_normal,
                    normal_iterations,
                    vertex_iterations,
                );
            }
        }
    }
    Ok(out)
}

/// Which vertices may move.
fn movable_mask(m: &IndexedMesh, adj: &Adjacency, opts: &SmoothOpts) -> MeshResult<Vec<bool>> {
    let nv = m.positions.len();
    let mut movable = vec![true; nv];
    if let Some(sel) = &opts.selection {
        movable.fill(false);
        for f in sel {
            let t = m.tris.get(f.0 as usize).ok_or(MeshError::IndexOutOfRange {
                index: f.0,
                len: m.tris.len(),
            })?;
            for &v in t {
                movable[v as usize] = true;
            }
        }
    }
    if opts.fix_boundary {
        for (b, &fixed) in movable.iter_mut().zip(&adj.boundary) {
            if fixed {
                *b = false;
            }
        }
    }
    let groups = m.groups.per_face();
    for (v, b) in movable.iter_mut().enumerate() {
        let faces = adj.faces_of(v);
        if faces.is_empty() {
            *b = false;
        } else if opts.preserve_group_borders {
            let g = groups[faces[0] as usize];
            if faces.iter().any(|&f| groups[f as usize] != g) {
                *b = false;
            }
        }
    }
    Ok(movable)
}

/// `dst = src + step · L(src)` for movable vertices, `dst = src` otherwise.
fn laplacian_step(
    src: &[Point3],
    dst: &mut [Point3],
    tris: &[[u32; 3]],
    adj: &Adjacency,
    movable: &[bool],
    step: f64,
    weights: LaplacianWeights,
) {
    dst.par_iter_mut().enumerate().for_each(|(v, out)| {
        let p = src[v];
        *out = p;
        if !movable[v] {
            return;
        }
        let d = match weights {
            LaplacianWeights::Uniform => uniform_delta(src, adj, v, p),
            LaplacianWeights::Cotangent => {
                cotan_delta(src, tris, adj, v, p).unwrap_or_else(|| uniform_delta(src, adj, v, p))
            }
        };
        *out = p + step * d;
    });
}

fn uniform_delta(src: &[Point3], adj: &Adjacency, v: usize, p: Point3) -> Vec3 {
    let ns = adj.neighbors_of(v);
    if ns.is_empty() {
        return Vec3::ZERO;
    }
    let sum: Vec3 = ns.iter().map(|&n| src[n as usize]).sum();
    sum / ns.len() as f64 - p
}

/// Normalised cotangent Laplacian `Σ w_ij (p_j - p_i) / Σ w_ij`; `None` if degenerate.
fn cotan_delta(
    src: &[Point3],
    tris: &[[u32; 3]],
    adj: &Adjacency,
    v: usize,
    p: Point3,
) -> Option<Vec3> {
    // For each incident face (v, a, b) in ccw order, the angle at b is opposite edge v-a and
    // the angle at a is opposite edge v-b. The (up to) two faces on an edge add up to
    // cot α + cot β. Valences are small, so a linear-probe accumulator is fastest.
    let mut acc: Vec<(u32, f64)> = Vec::with_capacity(12);
    let mut add = |k: u32, w: f64| {
        if let Some(e) = acc.iter_mut().find(|e| e.0 == k) {
            e.1 += w;
        } else {
            acc.push((k, w));
        }
    };
    for &f in adj.faces_of(v) {
        let t = tris[f as usize];
        let k = t.iter().position(|&x| x as usize == v)?;
        let (a, b) = (t[(k + 1) % 3], t[(k + 2) % 3]);
        let (pa, pb) = (src[a as usize], src[b as usize]);
        add(a, cot(p - pb, pa - pb));
        add(b, cot(p - pa, pb - pa));
    }
    let mut wsum = 0.0;
    let mut d = Vec3::ZERO;
    for &(n, w) in &acc {
        let w = w.max(0.0);
        wsum += w;
        d += w * (src[n as usize] - p);
    }
    (wsum > 1e-12).then(|| d / wsum)
}

fn cot(u: Vec3, w: Vec3) -> f64 {
    let c = u.cross(w).length();
    if c < 1e-300 { 0.0 } else { u.dot(w) / c }
}

#[cfg(test)]
mod tests;
