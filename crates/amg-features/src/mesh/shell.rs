//! Robust shell (MESH-10, part 2): an SDF band extracted with marching tetrahedra.
//!
//! [`offset_shell_sdf`] builds the field `f(p) = max(s(p) - hi, lo - s(p))`, where `s` is the
//! exact signed distance to the input ([`amg_implicit::MeshSdf`], IMP-03, negative inside) and
//! `[lo, hi]` is the wall's distance range:
//!
//! | side      | `lo`   | `hi`  |
//! |-----------|--------|-------|
//! | `Inside`  | `-t`   | `0`   |
//! | `Outside` | `0`    | `t`   |
//! | `Both`    | `-t/2` | `t/2` |
//!
//! (`Inside` gives the spec's `max(s, -(s + t))`.) The material is `f < 0`. It is extracted with
//! [`amg_mesh::iso::extract_isosurface`] (MESH-17, sparse marching tetrahedra), which always
//! gives a closed, oriented 2-manifold, then decimated (MESH-07) to remove the lattice slivers.
//! A decimated candidate must keep every vertex within [`MAX_VERTEX_ERROR`] voxels of its
//! level. The result is checked with the exact MESH-19 test; if decimation (or a degenerate lattice
//! alignment) left an intersecting pair, the function retries with a finer decimation, without
//! decimation, and with a shifted lattice (see [`LATTICE_SHIFTS`]), so the returned mesh is
//! always closed, manifold and free of self-intersections.
//!
//! The SDF's band is `max(|lo|, |hi|) + 2 voxels`, so every lattice edge that the surface
//! crosses has exact (unclamped) values at both ends, and `may_cross` prunes every block
//! farther than that from the input. The lattice block size is [`BLOCK_CELLS`].
//!
//! The input should be closed and outward oriented. An inside-out input (negative signed
//! volume) is flipped first. On an open input the winding-number sign of `MeshSdf` still gives
//! a result, but the field is not pruned there (slower) and a wall can appear where the
//! winding number passes 0.5 away from the surface.

use std::sync::Arc;

use amg_core::{Aabb3, Point3, Vec3};
use amg_implicit::MeshSdf;
use amg_jobs::JobCtx;
use amg_mesh::iso::{IsoOpts, ScalarField, extract_isosurface};
use amg_mesh::{
    BvhOpts, DEFAULT_GROUP, DecimateOpts, DecimateTarget, FaceGroupInfo, FaceGroups, GroupOrigin,
    IndexedMesh, MeshError, MeshSpatial, Rgba8, SelfIntersectOpts, ShellSide, decimate,
    reverse_normals, self_intersections,
};

use super::MeshFeatureError;

/// Default decimation tolerance, as a fraction of the voxel size (used when
/// [`SdfShellOpts::simplify_tolerance`] is `None`).
pub const DEFAULT_SIMPLIFY_FRACTION: f64 = 0.25;

/// A decimated candidate is accepted only if every vertex lies within this many voxels of its
/// offset level (`|f| <= MAX_VERTEX_ERROR * voxel_size`); otherwise a finer decimation, then
/// the undecimated mesh, is used.
pub const MAX_VERTEX_ERROR: f64 = 0.5;

/// Raw meshes with at least this many faces are decimated in parallel chunks
/// ([`CHUNK_CELLS`] voxels on a side, two passes on grids shifted by half a chunk) instead of
/// in one piece: MESH-07 is sequential, and the lattice mesh of a 100 mm part at 0.25 mm has
/// about 9 M faces (35 s in one piece).
pub const CHUNKED_MIN_FACES: usize = 500_000;

/// Chunk edge for the parallel decimation, in voxels.
pub const CHUNK_CELLS: f64 = 32.0;

/// Chunks with fewer faces are kept as they are.
const CHUNK_MIN_DECIMATE: usize = 64;

/// Lattice block edge in cells. Smaller blocks prune more of the empty space around a thin
/// wall (the pruning test is a ball of the block's half diagonal) at a little more sharing
/// overhead; 8 is about 2× faster than MESH-17's default 16 at 0.25 mm voxels.
pub const BLOCK_CELLS: u32 = 8;

/// Lattice origin shifts (fractions of a voxel, applied to `bounds.min`) tried in turn.
/// Non-round values keep lattice planes off the planar faces of CAD-like inputs, where exact
/// zeros would put vertices of different lattice edges on the same point.
pub const LATTICE_SHIFTS: [f64; 3] = [0.137_174_211, 0.381_966_011, 0.723_606_798];

/// Options for [`offset_shell_sdf`].
#[derive(Clone, Debug)]
pub struct SdfShellOpts {
    /// Wall thickness, mm (finite, > 0). Walls under about two voxels can break up.
    pub thickness: f64,
    /// Which side of the input the wall goes to.
    pub side: ShellSide,
    /// Lattice spacing, mm (finite, > 0). The wall thickness is accurate to about one voxel.
    pub voxel_size: f64,
    /// Decimation tolerance in mm (area-weighted RMS plane distance, MESH-07 `MaxError`).
    /// `None` = [`DEFAULT_SIMPLIFY_FRACTION`] × `voxel_size`; `Some(0.0)` (or less) keeps the
    /// raw lattice mesh.
    pub simplify_tolerance: Option<f64>,
}

impl SdfShellOpts {
    /// `thickness` on `side` at `voxel_size`, default decimation.
    pub fn new(thickness: f64, side: ShellSide, voxel_size: f64) -> Self {
        Self {
            thickness,
            side,
            voxel_size,
            simplify_tolerance: None,
        }
    }

    /// The wall's signed distance range `[lo, hi]` (see the module docs).
    pub fn levels(&self) -> (f64, f64) {
        let t = self.thickness;
        match self.side {
            ShellSide::Inside => (-t, 0.0),
            ShellSide::Outside => (0.0, t),
            ShellSide::Both => (-0.5 * t, 0.5 * t),
        }
    }
}

/// The shell field `max(s - hi, lo - s)` over a [`MeshSdf`] `s` (negative inside the wall).
#[derive(Debug)]
pub struct ShellField {
    sdf: MeshSdf,
    lo: f64,
    hi: f64,
}

impl ShellField {
    /// Wall between the signed distances `lo <= hi` of `sdf`. The band of `sdf` must exceed
    /// `max(|lo|, |hi|)`, or the wall is clipped where the band clamps.
    pub fn new(sdf: MeshSdf, lo: f64, hi: f64) -> Self {
        Self { sdf, lo, hi }
    }

    /// The underlying distance field.
    pub fn sdf(&self) -> &MeshSdf {
        &self.sdf
    }

    #[inline]
    fn combine(&self, s: f64) -> f64 {
        if s.is_nan() {
            return f64::NAN;
        }
        (s - self.hi).max(self.lo - s)
    }
}

impl ScalarField for ShellField {
    fn value(&self, p: Point3) -> f64 {
        self.combine(self.sdf.signed(p))
    }

    fn values(&self, ps: &[Point3], out: &mut [f64]) {
        self.sdf.values(ps, out);
        for o in out.iter_mut() {
            *o = self.combine(*o);
        }
    }

    /// `f = iso` needs `s = hi + iso` or `s = lo - iso`, so the box can only cross where `s`
    /// can reach one of the two levels.
    fn may_cross(&self, b: &Aabb3, iso: f64) -> bool {
        self.sdf.may_cross(b, self.hi + iso) || self.sdf.may_cross(b, self.lo - iso)
    }
}

/// Robust SDF shell of `m`. See the module docs.
///
/// The output is one face group with origin `GroupOrigin::Shell { side }`; no vertex
/// attributes.
///
/// # Errors
/// [`MeshError::Empty`] for an empty input, [`MeshError::InvalidOption`] for a bad thickness,
/// voxel size or tolerance, [`MeshError::TooLarge`] when the lattice is too large,
/// [`MeshError::Cancelled`] (all wrapped in [`MeshFeatureError::Mesh`]);
/// [`MeshFeatureError::Sdf`] when the wall is empty (nothing of the input is thicker than a
/// voxel) or no attempt gave a self-intersection-free mesh.
pub fn offset_shell_sdf(
    m: &Arc<IndexedMesh>,
    opts: &SdfShellOpts,
    ctx: &JobCtx,
) -> Result<IndexedMesh, MeshFeatureError> {
    let bad = |name: &'static str, reason: String| {
        MeshFeatureError::Mesh(MeshError::InvalidOption { name, reason })
    };
    let (t, h) = (opts.thickness, opts.voxel_size);
    if !(t.is_finite() && t > 0.0) {
        return Err(bad("thickness", format!("must be finite and > 0, got {t}")));
    }
    if !(h.is_finite() && h > 0.0) {
        return Err(bad(
            "voxel_size",
            format!("must be finite and > 0, got {h}"),
        ));
    }
    let tol = opts
        .simplify_tolerance
        .unwrap_or(DEFAULT_SIMPLIFY_FRACTION * h);
    if tol.is_nan() || tol.is_infinite() {
        return Err(bad(
            "simplify_tolerance",
            format!("must be finite, got {tol}"),
        ));
    }
    if m.tris.is_empty() {
        return Err(MeshError::Empty.into());
    }
    ctx.checkpoint().map_err(MeshError::from)?;

    // MeshSdf is negative where the winding number exceeds 0.5: flip an inside-out input.
    let input = if m.signed_volume() < 0.0 {
        let mut f = IndexedMesh::clone(m);
        reverse_normals(&mut f, None);
        Arc::new(f)
    } else {
        Arc::clone(m)
    };
    let spatial = Arc::new(MeshSpatial::build(
        Arc::clone(&input),
        &BvhOpts::default(),
        &ctx.sub(0.0, 0.03),
    )?);
    spatial.prepare_winding(&ctx.sub(0.03, 0.06))?;
    let (lo, hi) = opts.levels();
    let band = lo.abs().max(hi.abs()) + 2.0 * h;
    let field = ShellField::new(MeshSdf::new(spatial, band), lo, hi);
    let margin = hi.max(0.0) + 2.0 * h;
    let aabb = input.aabb();

    let n_attempts = LATTICE_SHIFTS.len() as f32;
    for (k, &shift) in LATTICE_SHIFTS.iter().enumerate() {
        let span = |i: f32| 0.06 + 0.94 * (k as f32 + i) / n_attempts;
        let actx = ctx.sub(span(0.0), span(1.0));
        let bounds = Aabb3::new(
            aabb.min - Vec3::splat(margin + shift * h),
            aabb.max + Vec3::splat(margin),
        );
        let mut iso = IsoOpts::new(bounds, h);
        iso.block_cells = BLOCK_CELLS;
        iso.project_steps = 1;
        let raw = extract_isosurface(&field, &iso, &actx.sub(0.0, 0.7))?;
        if raw.tris.is_empty() {
            return Err(MeshFeatureError::Sdf(format!(
                "the {t} mm shell at {h} mm voxels is empty"
            )));
        }
        // Candidates in order: decimated at `tol`, at `tol / 4`, raw.
        let tols: &[f64] = if tol > 0.0 { &[1.0, 0.25] } else { &[] };
        for (j, &f) in tols.iter().enumerate() {
            let cctx = actx.sub(0.7 + 0.1 * j as f32, 0.8 + 0.1 * j as f32);
            let c = simplify(&raw, f * tol, h, &cctx)?;
            // MESH-07's `MaxError` bounds an RMS plane distance, not each vertex: reject a
            // decimation that moved a vertex more than `MAX_VERTEX_ERROR` voxels off its level.
            if max_abs_field(&field, &c) > MAX_VERTEX_ERROR * h {
                continue;
            }
            let (c, clean) = is_si_free(c, &cctx)?;
            if clean {
                return Ok(finish(c, opts.side));
            }
        }
        let (raw, clean) = is_si_free(raw, &actx.sub(0.9, 1.0))?;
        if clean {
            return Ok(finish(raw, opts.side));
        }
    }
    Err(MeshFeatureError::Sdf(
        "no lattice alignment gave a self-intersection-free shell".into(),
    ))
}

/// Largest `|f|` over the vertices of `m`: how far (mm, up to the band) a vertex is from the
/// nearer of the two offset levels.
fn max_abs_field(field: &ShellField, m: &IndexedMesh) -> f64 {
    let mut out = vec![0.0; m.positions.len()];
    field.values(&m.positions, &mut out);
    out.iter().fold(0.0, |a, v| {
        if v.is_nan() {
            f64::INFINITY
        } else {
            a.max(v.abs())
        }
    })
}

fn decimate_opts(tol: f64) -> DecimateOpts {
    DecimateOpts {
        target: DecimateTarget::MaxError(tol),
        max_error: Some(tol),
        preserve_boundary: true,
        preserve_group_borders: false,
        feature_angle: None,
        preserve_attributes: false,
        ..DecimateOpts::default()
    }
}

/// MESH-07 at `tol`: in one piece below [`CHUNKED_MIN_FACES`], else in parallel chunks
/// ([`decimate_chunked`], two passes on grids shifted by half a chunk).
fn simplify(
    m: &IndexedMesh,
    tol: f64,
    h: f64,
    ctx: &JobCtx,
) -> Result<IndexedMesh, MeshFeatureError> {
    if m.tris.len() < CHUNKED_MIN_FACES {
        return Ok(decimate(m, &decimate_opts(tol), ctx)?.0);
    }
    let edge = CHUNK_CELLS * h;
    let Some(a) = decimate_chunked(m, tol, edge, Vec3::ZERO, &ctx.sub(0.0, 0.7))? else {
        return Ok(decimate(m, &decimate_opts(tol), ctx)?.0);
    };
    match decimate_chunked(&a, tol, edge, Vec3::splat(0.5 * edge), &ctx.sub(0.7, 1.0))? {
        Some(b) => Ok(b),
        None => Ok(a),
    }
}

/// One chunk after decimation: positions, the global seam vertex of each position (or
/// `NO_SEAM`), and the faces.
type Piece = (Vec<Point3>, Vec<u32>, Vec<[u32; 3]>);

const NO_SEAM: u32 = u32::MAX;

/// Decimates `m` in independent spatial chunks of edge `edge` (grid anchored at
/// `aabb.min - shift`), in parallel.
///
/// Faces go to the chunk of their centroid. A vertex used by faces of two or more chunks is a
/// seam vertex and is locked in every chunk: MESH-07 never collapses an edge at a vertex of a
/// degenerate face, so each chunk gets one `[v, v, v]` face per seam vertex, removed
/// afterwards. Seam vertices therefore keep their exact positions, and the chunks are stitched
/// back by their global indices (carried through MESH-07 as vertex-colour tags). The topology is what MESH-07 keeps in one piece, but a band of undecimated
/// faces stays along the seams (the second, shifted pass reduces it). Returns `None` for a
/// single chunk, so the caller falls back to one piece. The output is identical for any thread
/// count.
fn decimate_chunked(
    m: &IndexedMesh,
    tol: f64,
    edge: f64,
    shift: Vec3,
    ctx: &JobCtx,
) -> Result<Option<IndexedMesh>, MeshFeatureError> {
    use rayon::prelude::*;
    let nv = m.positions.len();
    let origin = m.aabb().min - shift;
    let cell = |p: Point3| {
        let c = ((p - origin) / edge).floor();
        let k = |v: f64| (v.max(0.0) as u64).min((1 << 21) - 1);
        (k(c.x) << 42) | (k(c.y) << 21) | k(c.z)
    };
    let keys: Vec<u64> = m
        .tris
        .par_iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| m.positions[i as usize]);
            cell((a + b + c) / 3.0)
        })
        .collect();
    let mut uniq = keys.clone();
    uniq.par_sort_unstable();
    uniq.dedup();
    if uniq.len() < 2 {
        return Ok(None);
    }
    let chunk_of: Vec<u32> = keys
        .par_iter()
        .map(|k| uniq.binary_search(k).unwrap_or(0) as u32)
        .collect();
    let mut owner = vec![NO_SEAM; nv];
    let mut seam = vec![false; nv];
    for (t, &c) in m.tris.iter().zip(&chunk_of) {
        for &v in t {
            let o = &mut owner[v as usize];
            if *o == NO_SEAM {
                *o = c;
            } else if *o != c {
                seam[v as usize] = true;
            }
        }
    }
    let mut faces_of: Vec<Vec<u32>> = vec![Vec::new(); uniq.len()];
    for (f, &c) in chunk_of.iter().enumerate() {
        faces_of[c as usize].push(f as u32);
    }
    ctx.checkpoint().map_err(MeshError::from)?;
    let opts = decimate_opts(tol);

    let pieces: Vec<Result<Option<Piece>, MeshFeatureError>> = faces_of
        .par_iter()
        .map(|faces| {
            if ctx.is_cancelled() {
                return Err(MeshError::Cancelled.into());
            }
            decimate_piece(m, faces, &seam, &opts)
        })
        .collect();
    ctx.checkpoint().map_err(MeshError::from)?;

    // Stitch: seam vertices get one slot each, in index order; the rest follow by chunk.
    let mut slot = vec![NO_SEAM; nv];
    let mut positions = Vec::new();
    for (v, &is_seam) in seam.iter().enumerate() {
        if is_seam {
            slot[v] = positions.len() as u32;
            positions.push(m.positions[v]);
        }
    }
    let mut tris = Vec::new();
    for piece in pieces {
        let Some((pos, tag, ptris)) = piece? else {
            return Ok(None);
        };
        let map: Vec<u32> = pos
            .iter()
            .zip(&tag)
            .map(|(&p, &g)| {
                if g == NO_SEAM {
                    positions.push(p);
                    (positions.len() - 1) as u32
                } else {
                    slot[g as usize]
                }
            })
            .collect();
        tris.extend(ptris.iter().map(|t| t.map(|i| map[i as usize])));
    }
    let mut out = IndexedMesh::new(positions, tris)?;
    out.compact();
    Ok(Some(out))
}

/// Decimates the faces `faces` of `m` with every `seam` vertex locked. Each local vertex
/// carries its global seam index (or `NO_SEAM`) in the vertex-colour attribute, which MESH-07
/// keeps per surviving vertex (`preserve_attributes`); locked vertices are never merged, so the
/// tags come back exact even when two vertices share a position.
fn decimate_piece(
    m: &IndexedMesh,
    faces: &[u32],
    seam: &[bool],
    opts: &DecimateOpts,
) -> Result<Option<Piece>, MeshFeatureError> {
    let mut local = std::collections::HashMap::new();
    let mut pos = Vec::new();
    let mut glob = Vec::new();
    let mut tris = Vec::with_capacity(faces.len() + 16);
    for &f in faces {
        let t = m.tris[f as usize].map(|v| {
            *local.entry(v).or_insert_with(|| {
                pos.push(m.positions[v as usize]);
                glob.push(v);
                (pos.len() - 1) as u32
            })
        });
        tris.push(t);
    }
    let tags: Vec<Rgba8> = glob
        .iter()
        .map(|&g| Rgba8(if seam[g as usize] { g } else { NO_SEAM }.to_le_bytes()))
        .collect();
    for (l, &g) in glob.iter().enumerate() {
        if seam[g as usize] {
            let l = l as u32;
            tris.push([l, l, l]);
        }
    }
    let mut piece = IndexedMesh::new(pos, tris)?;
    piece.attrs.vertex_colors = Some(tags);
    let out = if faces.len() >= CHUNK_MIN_DECIMATE {
        let opts = DecimateOpts {
            preserve_attributes: true,
            ..opts.clone()
        };
        decimate(&piece, &opts, &JobCtx::detached())?.0
    } else {
        piece
    };
    let Some(colors) = out.attrs.vertex_colors.as_ref() else {
        return Ok(None);
    };
    let tag: Vec<u32> = colors.iter().map(|c| u32::from_le_bytes(c.0)).collect();
    let tris: Vec<[u32; 3]> = out
        .tris
        .iter()
        .copied()
        .filter(|t| t[0] != t[1] && t[1] != t[2] && t[2] != t[0])
        .collect();
    Ok(Some((out.positions, tag, tris)))
}

/// `(m, true)` when MESH-19 finds no intersecting pair.
fn is_si_free(m: IndexedMesh, ctx: &JobCtx) -> Result<(IndexedMesh, bool), MeshFeatureError> {
    let s = MeshSpatial::build(Arc::new(m), &BvhOpts::default(), ctx)?;
    let pairs = self_intersections(&s, &SelfIntersectOpts { max_pairs: Some(1) }, ctx)?;
    let arc = Arc::clone(s.mesh());
    drop(s);
    Ok((Arc::unwrap_or_clone(arc), pairs.is_empty()))
}

fn finish(mut m: IndexedMesh, side: ShellSide) -> IndexedMesh {
    let mut g = FaceGroups::new(m.tris.len());
    if let Some(info) = g.info_mut(DEFAULT_GROUP) {
        *info = FaceGroupInfo::new(GroupOrigin::Shell { side });
    }
    m.groups = g;
    m
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, reason = "tests")]

    use super::*;
    use amg_mesh::{AnalyzeOpts, analyze};

    /// The chunked decimation (used above `CHUNKED_MIN_FACES`) keeps the lattice mesh closed,
    /// manifold and two-shelled, keeps the volume, and is identical across thread counts.
    #[test]
    fn chunked_decimation_stitches_cleanly() {
        let s = amg_testkit::shapes::icosphere(Point3::ZERO, 10.0, 4);
        let m = Arc::new(IndexedMesh::new(s.positions, s.triangles).unwrap());
        let sp = Arc::new(MeshSpatial::build(m, &BvhOpts::default(), &JobCtx::detached()).unwrap());
        let h = 0.25;
        let field = ShellField::new(MeshSdf::new(sp, 2.0 + 2.0 * h), -2.0, 0.0);
        let b = Aabb3::new(
            Point3::splat(-10.0 - 3.0 * h),
            Point3::splat(10.0 + 2.0 * h),
        );
        let mut iso = IsoOpts::new(b, h);
        iso.block_cells = BLOCK_CELLS;
        let raw = extract_isosurface(&field, &iso, &JobCtx::detached()).unwrap();
        let run = || {
            let a = decimate_chunked(&raw, 0.25 * h, 8.0 * h, Vec3::ZERO, &JobCtx::detached())
                .unwrap()
                .expect("stitchable");
            decimate_chunked(
                &a,
                0.25 * h,
                8.0 * h,
                Vec3::splat(4.0 * h),
                &JobCtx::detached(),
            )
            .unwrap()
            .expect("stitchable")
        };
        let out = run();
        let rep = analyze(&out, &AnalyzeOpts::default(), &JobCtx::detached()).unwrap();
        assert!(rep.is_watertight(), "{rep:?}");
        assert_eq!(rep.shells.len(), 2);
        let (v0, v1) = (raw.signed_volume(), out.signed_volume());
        assert!((v0 - v1).abs() < 0.01 * v0, "{v0} vs {v1}");
        assert!(
            out.face_count() * 3 < raw.face_count(),
            "{} of {}",
            out.face_count(),
            raw.face_count()
        );
        assert!(max_abs_field(&field, &out) <= MAX_VERTEX_ERROR * h);
        let h1 = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap()
            .install(|| run().content_hash());
        assert_eq!(h1, out.content_hash());
    }
}
