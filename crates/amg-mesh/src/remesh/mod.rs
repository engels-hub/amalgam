//! Isotropic remeshing (MESH-06): Botsch & Kobbelt 2004.
//!
//! One iteration is: split edges longer than 4/3 L, collapse edges shorter than 4/5 L, flip
//! edges toward valence 6 (4 on a boundary), then tangential relaxation followed by projection
//! back onto the input surface.
//!
//! Constraints:
//! - *Constrained edges* (dihedral angle above [`RemeshOpts::feature_angle`], borders between
//!   face groups, and mesh boundaries) are never flipped. They may be split and collapsed, but
//!   their vertices only slide along the curve they form and are snapped back onto the input
//!   curve; a vertex where the curve ends, branches or turns sharply is a corner and stays put.
//! - With a [`RemeshOpts::selection`], faces outside the selection are untouched: every vertex
//!   of such a face is fixed and no edge next to one is split, collapsed or flipped.
//!
//! The edit passes are sequential with total-order tie-breaks; the parallel stages (candidate
//! scans and sorts, relaxation, projection) are pure per-vertex computations that are applied
//! afterwards. The result is therefore bit-identical across rayon thread counts.

mod adaptive;
mod features;
mod ops;
mod relax;

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};
use std::sync::Arc;

use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::{
    BvhOpts, FaceId, HalfEdgeMesh, IndexedMesh, MeshError, MeshResult, MeshSpatial,
    NonManifoldPolicy, VertexId,
};
use features::Snapper;

/// Target edge length.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EdgeLength {
    /// Millimetres.
    Absolute(f64),
    /// Factor times the mean edge length of the input.
    RelativeToMean(f64),
}

/// Sizing field of [`remesh`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sizing {
    /// One target edge length everywhere.
    Uniform(EdgeLength),
    /// Curvature-adaptive sizing (MESH-22): edge length `L = sqrt(6 eps / kappa - 3 eps^2)`
    /// (Dunyach et al. 2013) from the input's discrete curvature `kappa`, clamped to
    /// `[min_length, max_length]` and graded smoothly.
    Adaptive {
        /// Maximum deviation `eps` from the input, mm.
        tolerance: f64,
        /// Shortest edge, mm.
        min_length: f64,
        /// Longest edge, mm.
        max_length: f64,
    },
}

/// Options for [`remesh`].
#[derive(Clone, Debug)]
pub struct RemeshOpts {
    /// Target edge length.
    pub sizing: Sizing,
    /// Split / collapse / flip / relax rounds, default 5.
    pub iterations: u32,
    /// Edges whose dihedral angle exceeds this (radians) are constrained; `None` = no feature
    /// edges. Default 45 degrees.
    pub feature_angle: Option<f64>,
    /// Keep open boundaries on their input curve. When false they relax freely (and shrink).
    pub preserve_boundary: bool,
    /// Keep borders between face groups on their input curves.
    pub preserve_group_borders: bool,
    /// Project vertices back onto the input surface after relaxing.
    pub project_to_input: bool,
    /// Faces to remesh (`None` = the whole mesh). Others are left as they are.
    pub selection: Option<Vec<FaceId>>,
}

impl Default for RemeshOpts {
    fn default() -> Self {
        RemeshOpts {
            sizing: Sizing::Uniform(EdgeLength::RelativeToMean(1.0)),
            iterations: 5,
            feature_angle: Some(45f64.to_radians()),
            preserve_boundary: true,
            preserve_group_borders: true,
            project_to_input: true,
            selection: None,
        }
    }
}

#[derive(Default)]
pub(crate) struct IdHasher(u64);

impl Hasher for IdHasher {
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

/// Constrained edges: key `(min vertex, max vertex)` to a kind flag.
pub(crate) type EdgeMap = HashMap<u64, u8, BuildHasherDefault<IdHasher>>;

pub(crate) fn key(a: u32, b: u32) -> u64 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    (u64::from(lo) << 32) | u64::from(hi)
}

/// Chunk of the ordered parallel sum (fixed, so the sum does not depend on the thread count).
const SUM_CHUNK: usize = 8192;

/// Mean edge length of `m` over its unique edges (each shared edge counted once).
fn mean_edge_length(he: &HalfEdgeMesh) -> f64 {
    let n = he.face_slots();
    let parts: Vec<(f64, usize)> = (0..n.div_ceil(SUM_CHUNK))
        .into_par_iter()
        .map(|c| {
            let (mut s, mut cnt) = (0.0, 0usize);
            for f in c * SUM_CHUNK..((c + 1) * SUM_CHUNK).min(n) {
                let f = FaceId(f as u32);
                if !he.is_face_alive(f) {
                    continue;
                }
                for k in 0..3 {
                    let h = he.halfedge(f, k);
                    if he.twin(h).is_some_and(|t| t.0 < h.0) {
                        continue;
                    }
                    s += (he.position(he.target(h)) - he.position(he.origin(h))).length();
                    cnt += 1;
                }
            }
            (s, cnt)
        })
        .collect();
    let (sum, cnt) = parts
        .iter()
        .fold((0.0, 0usize), |a, b| (a.0 + b.0, a.1 + b.1));
    if cnt == 0 { 0.0 } else { sum / cnt as f64 }
}

/// Working state shared by the passes.
pub(crate) struct State {
    pub(crate) he: HalfEdgeMesh,
    /// Constrained edges.
    pub(crate) cons: EdgeMap,
    /// Constrained edges at each vertex slot.
    pub(crate) ncons: Vec<u8>,
    /// Vertices that never move and are never removed.
    pub(crate) fixed: Vec<bool>,
    /// Selected faces per face slot; `None` = all.
    pub(crate) sel: Option<Vec<bool>>,
    /// Target edge length at each vertex slot (an edge uses the smaller of its two ends).
    pub(crate) vs: Vec<f64>,
    /// Curvature size field of the input (adaptive sizing only).
    pub(crate) field: Option<adaptive::SizeField>,
    /// Project relaxed vertices back onto the input.
    pub(crate) project: bool,
}

impl State {
    pub(crate) fn face_selected(&self, f: FaceId) -> bool {
        self.sel.as_ref().is_none_or(|s| s[f.0 as usize])
    }
}

/// Isotropic remesh of `m`. See the module docs and [`RemeshOpts`].
///
/// The result keeps the topology (genus, shells, boundary loops). Face groups are inherited by
/// split faces. Vertex normals are dropped (stale); vertex colours and UVs are interpolated by
/// the topology edits but not re-evaluated when vertices slide.
///
/// # Errors
/// [`MeshError::Empty`] without faces, [`MeshError::NonManifold`] for non-manifold input,
/// [`MeshError::InvalidOption`] for a bad option,
/// [`MeshError::Cancelled`] when `ctx` is cancelled.
#[allow(clippy::too_many_lines)]
pub fn remesh(m: &IndexedMesh, opts: &RemeshOpts, ctx: &JobCtx) -> MeshResult<IndexedMesh> {
    let bad = |name: &'static str, reason: &str| MeshError::InvalidOption {
        name,
        reason: reason.to_string(),
    };
    if m.tris.is_empty() {
        return Err(MeshError::Empty);
    }
    if let Some(a) = opts.feature_angle
        && !(a.is_finite() && a > 0.0)
    {
        return Err(bad("feature_angle", "must be finite and > 0"));
    }
    let (rel, abs) = match opts.sizing {
        Sizing::Uniform(EdgeLength::Absolute(l)) => (None, Some(l)),
        Sizing::Uniform(EdgeLength::RelativeToMean(f)) => (Some(f), None),
        Sizing::Adaptive {
            tolerance,
            min_length,
            max_length,
        } => {
            if !(tolerance.is_finite() && tolerance > 0.0) {
                return Err(bad("tolerance", "must be finite and > 0"));
            }
            if !(min_length.is_finite() && min_length > 0.0) {
                return Err(bad("min_length", "must be finite and > 0"));
            }
            if !(max_length.is_finite() && max_length >= min_length) {
                return Err(bad("max_length", "must be finite and >= min_length"));
            }
            (None, None)
        }
    };
    if let Some(v) = rel.or(abs)
        && !(v.is_finite() && v > 0.0)
    {
        return Err(bad("sizing", "edge length must be finite and > 0"));
    }
    if let Some(sel) = &opts.selection
        && let Some(f) = sel.iter().find(|f| f.0 as usize >= m.tris.len())
    {
        return Err(bad("selection", &format!("face {} out of range", f.0)));
    }

    let (he, _) = HalfEdgeMesh::from_indexed(m, NonManifoldPolicy::Fail)?;
    let target = match (rel, abs) {
        (Some(f), _) => f * mean_edge_length(&he),
        (_, Some(l)) => l,
        // Adaptive: per-vertex sizes below; the longest bound stands in as the scalar.
        _ => match opts.sizing {
            Sizing::Adaptive { max_length, .. } => max_length,
            Sizing::Uniform(_) => unreachable!("uniform sizing sets rel or abs"),
        },
    };
    if !(target.is_finite() && target > 0.0) {
        return Err(bad(
            "sizing",
            "target edge length is zero (degenerate mesh)",
        ));
    }
    if opts.iterations == 0 {
        let mut out = he.to_indexed();
        out.attrs.vertex_normals = None;
        return Ok(out);
    }
    ctx.checkpoint()?;

    let nv = he.vertex_slots();
    let nfs = he.face_slots();
    // Selection per half-edge-mesh face slot (degenerate input faces were dropped, in order).
    let (sel, fixed) = match &opts.selection {
        None => (None, vec![false; nv]),
        Some(list) => {
            let mut in_sel = vec![false; m.tris.len()];
            for f in list {
                in_sel[f.0 as usize] = true;
            }
            let sel: Vec<bool> = m
                .tris
                .iter()
                .zip(&in_sel)
                .filter(|(t, _)| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
                .map(|(_, &s)| s)
                .collect();
            debug_assert_eq!(sel.len(), nfs);
            let mut fixed = vec![false; nv];
            for f in he.faces() {
                if !sel[f.0 as usize] {
                    for v in he.face_vertices(f) {
                        fixed[v.0 as usize] = true;
                    }
                }
            }
            (Some(sel), fixed)
        }
    };

    let adaptive_sizing = matches!(opts.sizing, Sizing::Adaptive { .. });
    let input = Arc::new(m.clone());
    let spatial = if opts.project_to_input || adaptive_sizing {
        Some(MeshSpatial::build(
            Arc::clone(&input),
            &BvhOpts::default(),
            &ctx.sub(0.0, 0.05),
        )?)
    } else {
        None
    };
    let (vs, field) = match opts.sizing {
        Sizing::Adaptive {
            tolerance,
            min_length,
            max_length,
        } => {
            let sizes = adaptive::sizes(m, tolerance, min_length, max_length, opts.feature_angle);
            let field = adaptive::SizeField {
                mesh: Arc::clone(&input),
                sizes,
            };
            let sp = spatial
                .as_ref()
                .ok_or_else(|| bad("sizing", "no spatial index for the size field"))?;
            let mut vs = vec![target; nv];
            let ids: Vec<u32> = he.vertices().map(|v| v.0).collect();
            let found: Vec<f64> = ids
                .par_iter()
                .map(|&v| {
                    field
                        .at(sp, he.position(VertexId(v)), 1e-6 * target.max(1.0))
                        .unwrap_or(max_length)
                })
                .collect();
            for (&v, s) in ids.iter().zip(found) {
                vs[v as usize] = s;
            }
            (vs, Some(field))
        }
        Sizing::Uniform(_) => (vec![target; nv], None),
    };
    let (cons, segs) = features::classify(&he, opts);
    let snapper = Snapper::build(&segs, ctx)?;
    let mut ncons = vec![0u8; nv];
    for &k in cons.keys() {
        ncons[(k >> 32) as usize] = ncons[(k >> 32) as usize].saturating_add(1);
        ncons[(k & 0xffff_ffff) as usize] = ncons[(k & 0xffff_ffff) as usize].saturating_add(1);
    }
    let mut st = State {
        he,
        cons,
        ncons,
        fixed,
        sel,
        vs,
        field,
        project: opts.project_to_input,
    };

    for it in 0..opts.iterations {
        let base = 0.05 + 0.95 * it as f32 / opts.iterations as f32;
        let span = 0.95 / opts.iterations as f32;
        let sub = |a: f32, b: f32| ctx.sub(base + span * a, base + span * b);
        st.split_pass(&sub(0.0, 0.2))?;
        st.collapse_pass(&sub(0.2, 0.4))?;
        st.flip_pass(&sub(0.4, 0.6))?;
        st.relax_pass(spatial.as_ref(), &snapper, &sub(0.6, 1.0))?;
    }
    ctx.checkpoint()?;
    ctx.progress(1.0);

    let mut out = st.he.to_indexed();
    out.attrs.vertex_normals = None;
    Ok(out)
}
