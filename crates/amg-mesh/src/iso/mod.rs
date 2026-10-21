//! Isosurface extraction: sparse marching tetrahedra over a [`ScalarField`] (MESH-17).
//!
//! The lattice has spacing `cell_size`, anchored at `bounds.min`, with one extra layer of
//! forced-outside points on every side. Every cube is split into six Kuhn tetrahedra
//! (see `tables`), and every output vertex sits on a unique tet edge, keyed by its lattice
//! point and direction. A sample is *inside* when `value < iso`; `value >= iso`, NaN and
//! everything outside `bounds` are *outside*. With that rule the result is a closed,
//! oriented 2-manifold for **any** field (possibly empty or multi-shell), normals point
//! towards increasing values, and the signed volume of the inside is positive.
//!
//! Work is done in blocks of `block_cells³` cells. A block that lies wholly inside `bounds`
//! and for which [`ScalarField::may_cross`] is `false` is skipped without evaluating the
//! field. Blocks are processed in parallel and merged in block order, so the output is
//! identical for any thread count.

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};
use std::sync::atomic::{AtomicUsize, Ordering};

use amg_core::{Aabb3, Point3};
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::{IndexedMesh, MeshError, MeshResult};

mod tables;
use tables::{CASES, TETS};

/// A scalar field sampled by the extractor. Negative-inside (signed distance) convention:
/// the solid is `value < iso`.
pub trait ScalarField: Sync {
    /// Field value at `p` (mm).
    fn value(&self, p: Point3) -> f64;

    /// Batched evaluation; the default loops over [`value`](Self::value).
    fn values(&self, ps: &[Point3], out: &mut [f64]) {
        for (p, o) in ps.iter().zip(out) {
            *o = self.value(*p);
        }
    }

    /// Conservative: return `false` only if the field cannot reach `iso` inside `b`.
    fn may_cross(&self, b: &Aabb3, iso: f64) -> bool {
        let _ = (b, iso);
        true
    }
}

/// Extraction parameters.
#[derive(Clone, Debug)]
pub struct IsoOpts {
    /// Region to sample. Everything outside is forced outside, which closes the surface.
    pub bounds: Aabb3,
    /// Lattice spacing, mm.
    pub cell_size: f64,
    /// Iso level.
    pub iso: f64,
    /// Block edge length in cells (default 16).
    pub block_cells: u32,
    /// Secant refinement steps per vertex along its edge (0..=3; more is clamped to 3).
    pub project_steps: u32,
}

impl IsoOpts {
    /// Defaults: `iso = 0`, 16-cell blocks, no projection.
    pub fn new(bounds: Aabb3, cell_size: f64) -> Self {
        Self {
            bounds,
            cell_size,
            iso: 0.0,
            block_cells: 16,
            project_steps: 0,
        }
    }
}

/// Sparse marching tetrahedra. The output is always a closed, oriented 2-manifold (possibly
/// empty or multi-shell). Values `>= iso` count as outside; outside `bounds` is forced
/// outside. Vertices lie on lattice tet edges, so triangles can be slivers: follow with
/// decimation or remeshing.
///
/// # Errors
/// [`MeshError::InvalidOption`] for a non-positive or non-finite `cell_size`, an empty or
/// non-finite `bounds`, a non-finite `iso` or a zero `block_cells`; [`MeshError::TooLarge`]
/// when the lattice or the output exceeds the index range; [`MeshError::Cancelled`].
pub fn extract_isosurface(
    f: &dyn ScalarField,
    opts: &IsoOpts,
    ctx: &JobCtx,
) -> MeshResult<IndexedMesh> {
    let g = Grid::new(opts)?;
    let nb = g.blocks;
    let total = nb[0] as usize * nb[1] as usize * nb[2] as usize;
    let done = AtomicUsize::new(0);

    let outs: Vec<MeshResult<BlockOut>> = (0..total)
        .into_par_iter()
        .map_init(Scratch::default, |scratch, i| {
            if ctx.is_cancelled() {
                return Err(MeshError::Cancelled);
            }
            let (nx, ny) = (nb[0] as usize, nb[1] as usize);
            let b = [i % nx, (i / nx) % ny, i / (nx * ny)].map(|v| v as u32);
            let out = g.block(f, b, scratch);
            let d = done.fetch_add(1, Ordering::Relaxed) + 1;
            if d.is_multiple_of(32) {
                ctx.progress(0.9 * d as f32 / total as f32);
            }
            Ok(out)
        })
        .collect();

    let mut blocks = Vec::with_capacity(outs.len());
    for o in outs {
        blocks.push(o?);
    }
    ctx.checkpoint()?;

    // Merge in block order: vertex ids depend only on the data, never on scheduling.
    let n_local: usize = blocks.iter().map(|b| b.verts.len()).sum();
    let n_tris: usize = blocks.iter().map(|b| b.tris.len()).sum();
    let mut index: HashMap<u64, u32, KeyBuild> =
        HashMap::with_capacity_and_hasher(n_local / 2 + 16, KeyBuild::default());
    let mut positions: Vec<Point3> = Vec::with_capacity(n_local / 2 + 16);
    let mut tris: Vec<[u32; 3]> = Vec::with_capacity(n_tris);
    let mut remap: Vec<u32> = Vec::new();
    for b in &blocks {
        remap.clear();
        for &(key, p) in &b.verts {
            let next = positions.len();
            let id = *index.entry(key).or_insert_with(|| {
                positions.push(p);
                next as u32
            });
            if positions.len() > u32::MAX as usize - 1 {
                return Err(MeshError::TooLarge {
                    count: positions.len(),
                });
            }
            remap.push(id);
        }
        tris.extend(b.tris.iter().map(|t| t.map(|v| remap[v as usize])));
    }
    ctx.progress(1.0);
    IndexedMesh::new(positions, tris)
}

// ── lattice ────────────────────────────────────────────────────────────────────────────

struct Grid {
    bounds: Aabb3,
    h: f64,
    iso: f64,
    steps: u32,
    /// Block edge in cells.
    bc: u32,
    /// Cells per axis (including the forced-outside layer on both sides).
    m: [u32; 3],
    /// Blocks per axis.
    blocks: [u32; 3],
    /// Largest in-bounds lattice coordinate per axis (lattice coordinate 1 is `bounds.min`).
    umax: [u32; 3],
    /// Stride for the packed lattice-point id.
    stride: u64,
}

impl Grid {
    fn new(o: &IsoOpts) -> MeshResult<Grid> {
        let bad = |name: &'static str, reason: &str| MeshError::InvalidOption {
            name,
            reason: reason.to_owned(),
        };
        if !(o.cell_size.is_finite() && o.cell_size > 0.0) {
            return Err(bad("cell_size", "must be finite and positive"));
        }
        if !o.iso.is_finite() {
            return Err(bad("iso", "must be finite"));
        }
        if o.block_cells == 0 {
            return Err(bad("block_cells", "must be at least 1"));
        }
        let (lo, hi) = (o.bounds.min, o.bounds.max);
        if !(lo.is_finite() && hi.is_finite() && lo.cmple(hi).all()) {
            return Err(bad("bounds", "must be finite and non-empty"));
        }
        let size = hi - lo;
        let h = o.cell_size;
        let mut m = [0u32; 3];
        let mut umax = [0u32; 3];
        for k in 0..3 {
            let n = (size[k] / h).ceil().max(1.0);
            if n > f64::from(1u32 << 20) {
                return Err(MeshError::TooLarge { count: n as usize });
            }
            m[k] = n as u32 + 2;
            umax[k] = ((size[k] + 1e-9 * h) / h).floor() as u32 + 1;
        }
        let bc = o.block_cells.min(1 << 10);
        let blocks = m.map(|c| c.div_ceil(bc));
        Ok(Grid {
            bounds: o.bounds,
            h,
            iso: o.iso,
            steps: o.project_steps.min(3),
            bc,
            m,
            blocks,
            umax,
            stride: u64::from(m.into_iter().max().unwrap_or(0)) + 1,
        })
    }

    #[inline]
    fn world(&self, u: [u32; 3]) -> Point3 {
        let d = Point3::new(
            (f64::from(u[0]) - 1.0) * self.h,
            (f64::from(u[1]) - 1.0) * self.h,
            (f64::from(u[2]) - 1.0) * self.h,
        );
        self.bounds.min + d
    }

    #[inline]
    fn in_bounds(&self, u: [u32; 3]) -> bool {
        (0..3).all(|k| u[k] >= 1 && u[k] <= self.umax[k])
    }

    fn block(&self, f: &dyn ScalarField, b: [u32; 3], s: &mut Scratch) -> BlockOut {
        let c0 = b.map(|v| v * self.bc);
        let nc: [u32; 3] = std::array::from_fn(|k| self.bc.min(self.m[k] - c0[k]));
        let np = nc.map(|v| v as usize + 1);
        let c1: [u32; 3] = std::array::from_fn(|k| c0[k] + nc[k]);
        let mut out = BlockOut::default();

        // Block with no in-bounds point: everything is outside.
        if (0..3).any(|k| c1[k] < 1 || c0[k] > self.umax[k]) {
            return out;
        }
        let whole = (0..3).all(|k| c0[k] >= 1 && c1[k] <= self.umax[k]);
        // The in-bounds part of the block. If the field cannot cross `iso` there, it is on one
        // side everywhere; one probe says which. A block that is wholly outside is empty. A
        // block on the inside only has the cap against the forced-outside layer, and the
        // probe value stands in for every sample (only the sign is used there).
        let lo: [u32; 3] = std::array::from_fn(|k| c0[k].max(1));
        let hi: [u32; 3] = std::array::from_fn(|k| c1[k].min(self.umax[k]));
        let bb = Aabb3::new(self.world(lo), self.world(hi));
        let mut uniform = None;
        if !f.may_cross(&bb, self.iso) {
            if whole {
                return out;
            }
            let v = f.value(bb.center());
            if v.is_nan() || v >= self.iso {
                return out;
            }
            uniform = Some(v);
        }

        // Sample the lattice points of the block.
        let n = np[0] * np[1] * np[2];
        s.vals.clear();
        s.vals.resize(n, f64::INFINITY);
        s.pts.clear();
        s.slots.clear();
        for z in 0..np[2] {
            for y in 0..np[1] {
                for x in 0..np[0] {
                    let u = [c0[0] + x as u32, c0[1] + y as u32, c0[2] + z as u32];
                    if self.in_bounds(u) {
                        s.pts.push(self.world(u));
                        s.slots.push((x + np[0] * (y + np[1] * z)) as u32);
                    }
                }
            }
        }
        s.tmp.clear();
        s.tmp.resize(s.pts.len(), 0.0);
        match uniform {
            Some(v) => s.tmp.fill(v),
            None => f.values(&s.pts, &mut s.tmp),
        }
        let mut any_in = false;
        let mut any_out = s.pts.len() != n;
        for (&slot, &v) in s.slots.iter().zip(&s.tmp) {
            s.vals[slot as usize] = v;
            if v < self.iso {
                any_in = true;
            } else {
                any_out = true;
            }
        }
        if !(any_in && any_out) {
            return out;
        }

        let cases = &*CASES;
        let mut local: HashMap<u64, u32, KeyBuild> = HashMap::default();
        let at = |x: usize, y: usize, z: usize| x + np[0] * (y + np[1] * z);
        for cz in 0..nc[2] as usize {
            for cy in 0..nc[1] as usize {
                for cx in 0..nc[0] as usize {
                    // Corner values, bit k of the corner index = offset along axis k.
                    let mut cv = [0.0f64; 8];
                    let mut cmask = 0u32;
                    for (c, v) in cv.iter_mut().enumerate() {
                        *v = s.vals[at(cx + (c & 1), cy + ((c >> 1) & 1), cz + ((c >> 2) & 1))];
                        cmask |= u32::from(*v < self.iso) << c;
                    }
                    if cmask == 0 || cmask == 0xff {
                        continue;
                    }
                    let cu = [c0[0] + cx as u32, c0[1] + cy as u32, c0[2] + cz as u32];
                    for (t, corners) in TETS.iter().enumerate() {
                        let mut mask = 0usize;
                        for (i, &c) in corners.iter().enumerate() {
                            mask |= usize::from((cmask >> c) & 1 == 1) << i;
                        }
                        let case = &cases[t][mask];
                        for tri in &case.tris[..usize::from(case.n)] {
                            let mut ids = [0u32; 3];
                            for (slot, &(lo, code)) in ids.iter_mut().zip(tri) {
                                let ulo = offset(cu, lo);
                                let key = (u64::from(ulo[0])
                                    + self.stride
                                        * (u64::from(ulo[1]) + self.stride * u64::from(ulo[2])))
                                    * 8
                                    + u64::from(code);
                                *slot = *local.entry(key).or_insert_with(|| {
                                    let uhi = offset(ulo, code);
                                    let p = self.vertex(
                                        f,
                                        (self.world(ulo), cv[(lo) as usize]),
                                        (self.world(uhi), cv[(lo | code) as usize]),
                                    );
                                    out.verts.push((key, p));
                                    (out.verts.len() - 1) as u32
                                });
                            }
                            out.tris.push(ids);
                        }
                    }
                }
            }
        }
        out
    }

    /// Position of the vertex on the edge `a`-`b` (point, value); exactly one is inside.
    fn vertex(&self, f: &dyn ScalarField, a: (Point3, f64), b: (Point3, f64)) -> Point3 {
        let ((mut pi, mut fi), (mut po, mut fo)) = if a.1 < self.iso { (a, b) } else { (b, a) };
        let finite = fi.is_finite() && fo.is_finite();
        let tt = |fi: f64, fo: f64| {
            if fi.is_finite() && fo.is_finite() {
                let t = (self.iso - fi) / (fo - fi);
                if t.is_finite() {
                    t.clamp(0.0, 1.0)
                } else {
                    0.5
                }
            } else {
                0.5
            }
        };
        let mut t = tt(fi, fo);
        if finite {
            for _ in 0..self.steps {
                let p = pi + (po - pi) * t;
                let fp = f.value(p);
                if !fp.is_finite() {
                    break;
                }
                if fp < self.iso {
                    (pi, fi) = (p, fp);
                } else {
                    (po, fo) = (p, fp);
                }
                t = tt(fi, fo);
            }
        }
        (pi + (po - pi) * t).clamp(self.bounds.min, self.bounds.max)
    }
}

#[inline]
fn offset(u: [u32; 3], bits: u8) -> [u32; 3] {
    [
        u[0] + u32::from(bits & 1),
        u[1] + u32::from((bits >> 1) & 1),
        u[2] + u32::from((bits >> 2) & 1),
    ]
}

#[derive(Default)]
struct Scratch {
    vals: Vec<f64>,
    tmp: Vec<f64>,
    pts: Vec<Point3>,
    slots: Vec<u32>,
}

/// Block-local vertices (global edge key + position) and triangles over local ids.
#[derive(Default)]
struct BlockOut {
    verts: Vec<(u64, Point3)>,
    tris: Vec<[u32; 3]>,
}

type KeyBuild = BuildHasherDefault<KeyHasher>;

/// Multiplicative hasher for packed edge keys.
#[derive(Default)]
struct KeyHasher(u64);

impl Hasher for KeyHasher {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_u64(u64::from(b));
        }
    }
    fn write_u64(&mut self, n: u64) {
        let h = (n ^ self.0).wrapping_mul(0x9E37_79B9_7F4A_7C15);
        self.0 = h ^ (h >> 32);
    }
}
