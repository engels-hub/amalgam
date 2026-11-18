//! Wrap repair (MESH-18): a morphological closing through distance fields.
//!
//! For inputs whose topology is hopeless (soups, overlapping shells, open sheets, holes,
//! inconsistent orientation) [`wrap`] builds a watertight envelope without looking at the
//! input's connectivity or orientation:
//!
//! 1. **Dilate.** `g(p) = u(p) - r`, where `u` is the unsigned distance to the input
//!    ([`MeshSdf::unsigned`], IMP-03) and `r` the offset. `g < 0` is the input thickened by
//!    `r`; it is extracted with [`amg_mesh::iso::extract_isosurface`] (MESH-17), which always
//!    gives a closed, oriented 2-manifold.
//! 2. **Keep the outer component.** Shells of negative volume (cavities: the inner side of a
//!    thick part, hollow interiors) and positive shells nested inside another positive shell
//!    (islands in a cavity) are dropped. The rest bounds `F`, the dilated input with every
//!    region not reachable from infinity filled.
//! 3. **Erode.** `e(p) = s_F(p) + (r - skin)`, where `s_F` is the signed distance to `F`
//!    (a second [`MeshSdf`], on the MESH-07-decimated boundary of `F`, which is closed and
//!    outward, so the sign and the block pruning are exact). `e < 0` is extracted again and
//!    decimated (MESH-07); a candidate must keep every vertex within
//!    [`shell::MAX_VERTEX_ERROR`](super::shell::MAX_VERTEX_ERROR) voxels of `e = 0` and pass
//!    the exact MESH-19 test, otherwise a finer decimation, the raw lattice mesh and then the
//!    other lattice shifts ([`shell::LATTICE_SHIFTS`](super::shell::LATTICE_SHIFTS)) are tried.
//!
//! The erosion stops `skin` = min([`SKIN_VOXELS`] voxels, `r`) short of the full closing, so
//! the result is the closing of the input by `r`, grown by `skin`. Without it an open sheet
//! (zero thickness) would erode to nothing and walls thinner than a voxel would break up;
//! with it every part of the input lies at least about a voxel inside the result. Gaps and
//! holes narrower than about `2 r` are closed. Every output point is within about `r` of the
//! input (the result lies inside the dilation), so the Hausdorff distance from the result to
//! the input stays below `r + voxel`.
//!
//! The output is closed, manifold, outward and free of self-intersections, one default face
//! group, no vertex attributes. It is identical for any rayon thread count.

use std::sync::Arc;

use amg_core::{Aabb3, Point3, Vec3};
use amg_implicit::MeshSdf;
use amg_jobs::JobCtx;
use amg_mesh::iso::{IsoOpts, ScalarField, extract_isosurface};
use amg_mesh::{BvhOpts, FaceGroups, IndexedMesh, MeshError, MeshSpatial, separate_shells};
use rayon::prelude::*;

use super::MeshFeatureError;
use super::shell::{
    BLOCK_CELLS, DEFAULT_SIMPLIFY_FRACTION, LATTICE_SHIFTS, MAX_VERTEX_ERROR, is_si_free,
    max_abs_field, simplify,
};

/// Default offset (dilation radius) in voxels, used when [`WrapOpts::offset`] is `None`.
pub const DEFAULT_OFFSET_VOXELS: f64 = 1.5;

/// The erosion stops this many voxels short of the dilation (capped at the offset): the
/// result is the closing grown by this skin, so sheets and sub-voxel walls survive.
pub const SKIN_VOXELS: f64 = 1.0;

/// Options for [`wrap`].
#[derive(Clone, Debug)]
pub struct WrapOpts {
    /// Lattice spacing, mm (finite, > 0). The envelope is accurate to about one voxel.
    pub voxel_size: f64,
    /// Dilation radius, mm (finite, > 0): gaps and holes narrower than about twice this are
    /// closed. `None` = [`DEFAULT_OFFSET_VOXELS`] × `voxel_size`.
    pub offset: Option<f64>,
    /// Decimation tolerance in mm (MESH-07 `MaxError`). `None` =
    /// [`DEFAULT_SIMPLIFY_FRACTION`] × `voxel_size`; `Some(0.0)` (or less) keeps the raw
    /// lattice mesh.
    pub simplify_tolerance: Option<f64>,
}

impl WrapOpts {
    /// Wrap at `voxel_size` with the default offset and decimation.
    pub fn new(voxel_size: f64) -> Self {
        Self {
            voxel_size,
            offset: None,
            simplify_tolerance: None,
        }
    }

    /// The dilation radius `r`, mm.
    pub fn radius(&self) -> f64 {
        self.offset
            .unwrap_or(DEFAULT_OFFSET_VOXELS * self.voxel_size)
    }

    /// The erosion depth `r - skin`, mm (`skin` = min([`SKIN_VOXELS`] × voxel, `r`)).
    pub fn erosion(&self) -> f64 {
        let r = self.radius();
        r - (SKIN_VOXELS * self.voxel_size).min(r)
    }
}

/// The dilation field `u - r` over the unsigned distance `u` of a [`MeshSdf`] (negative within
/// `r` of the surface). Valid for any input, open or closed: `u` is 1-Lipschitz everywhere, so
/// [`ScalarField::may_cross`] prunes on open meshes too.
#[derive(Debug)]
pub struct DilateField {
    sdf: MeshSdf,
    radius: f64,
}

impl DilateField {
    /// Dilation of the surface of `sdf` by `radius`. The band of `sdf` must exceed `radius`.
    pub fn new(sdf: MeshSdf, radius: f64) -> Self {
        Self { sdf, radius }
    }

    /// The underlying distance field.
    pub fn sdf(&self) -> &MeshSdf {
        &self.sdf
    }
}

impl ScalarField for DilateField {
    fn value(&self, p: Point3) -> f64 {
        self.sdf.unsigned(p) - self.radius
    }

    fn values(&self, ps: &[Point3], out: &mut [f64]) {
        let n = ps.len().min(out.len());
        ps[..n]
            .par_iter()
            .zip(out[..n].par_iter_mut())
            .with_min_len(64)
            .for_each(|(p, o)| *o = self.value(*p));
    }

    /// `g = iso` needs `u = r + iso`. `u` is clamped to the band (a lower bound of the true
    /// distance there), so `|u(centre) - level| > half_diagonal` proves no crossing.
    fn may_cross(&self, b: &Aabb3, iso: f64) -> bool {
        let level = self.radius + iso;
        if level.is_nan() {
            return true;
        }
        if level < 0.0 {
            return false; // u >= 0
        }
        if level >= self.sdf.band() || !(b.min.is_finite() && b.max.is_finite()) {
            return true;
        }
        if b.min.cmpgt(b.max).any() {
            return true;
        }
        let c = (b.min + b.max) * 0.5;
        let r = (b.max - b.min).length() * 0.5;
        let slack = r * (1.0 + 1e-9) + 1e-9;
        (self.sdf.unsigned(c) - level).abs() <= slack
    }
}

/// The erosion field `s + depth` over a signed [`MeshSdf`] `s` of a closed, outward surface
/// (negative deeper than `depth` inside it).
#[derive(Debug)]
pub struct ErodeField {
    sdf: MeshSdf,
    depth: f64,
}

impl ErodeField {
    /// Erosion of the solid of `sdf` by `depth >= 0`. The band of `sdf` must exceed `depth`.
    pub fn new(sdf: MeshSdf, depth: f64) -> Self {
        Self { sdf, depth }
    }

    /// The underlying distance field.
    pub fn sdf(&self) -> &MeshSdf {
        &self.sdf
    }
}

impl ScalarField for ErodeField {
    fn value(&self, p: Point3) -> f64 {
        self.sdf.signed(p) + self.depth
    }

    fn values(&self, ps: &[Point3], out: &mut [f64]) {
        self.sdf.values(ps, out);
        for o in out.iter_mut() {
            *o += self.depth;
        }
    }

    fn may_cross(&self, b: &Aabb3, iso: f64) -> bool {
        self.sdf.may_cross(b, iso - self.depth)
    }
}

/// Wrap of `m`: a watertight envelope (see the module docs).
///
/// # Errors
/// [`MeshError::Empty`] for an input without faces, [`MeshError::InvalidOption`] for a bad
/// voxel size, offset or tolerance, [`MeshError::TooLarge`] when the lattice is too large,
/// [`MeshError::Cancelled`] (all wrapped in [`MeshFeatureError::Mesh`]);
/// [`MeshFeatureError::Sdf`] when the envelope is empty (no finite input vertex) or no lattice
/// alignment gave a self-intersection-free mesh.
pub fn wrap(
    m: &Arc<IndexedMesh>,
    opts: &WrapOpts,
    ctx: &JobCtx,
) -> Result<IndexedMesh, MeshFeatureError> {
    let bad = |name: &'static str, reason: String| {
        MeshFeatureError::Mesh(MeshError::InvalidOption { name, reason })
    };
    let h = opts.voxel_size;
    if !(h.is_finite() && h > 0.0) {
        return Err(bad(
            "voxel_size",
            format!("must be finite and > 0, got {h}"),
        ));
    }
    let r = opts.radius();
    if !(r.is_finite() && r > 0.0) {
        return Err(bad("offset", format!("must be finite and > 0, got {r}")));
    }
    let tol = opts
        .simplify_tolerance
        .unwrap_or(DEFAULT_SIMPLIFY_FRACTION * h);
    if !tol.is_finite() {
        return Err(bad(
            "simplify_tolerance",
            format!("must be finite, got {tol}"),
        ));
    }
    if m.tris.is_empty() {
        return Err(MeshError::Empty.into());
    }
    ctx.checkpoint().map_err(MeshError::from)?;
    let depth = opts.erosion();

    // 1. Dilation by `r` of the unsigned distance (orientation and connectivity ignored).
    let spatial = Arc::new(MeshSpatial::build(
        Arc::clone(m),
        &BvhOpts::default(),
        &ctx.sub(0.0, 0.03),
    )?);
    let dilate = DilateField::new(MeshSdf::new(spatial, r + 2.0 * h), r);
    let aabb = finite_aabb(m)
        .ok_or_else(|| MeshFeatureError::Sdf("the input has no finite vertex to wrap".into()))?;
    let dilated_bounds = |shift: f64| {
        let margin = r + 2.0 * h;
        Aabb3::new(
            aabb.min - Vec3::splat(margin + shift * h),
            aabb.max + Vec3::splat(margin),
        )
    };
    let dilated = |shift: f64, c: &JobCtx| -> Result<IndexedMesh, MeshFeatureError> {
        let raw = extract(&dilate, dilated_bounds(shift), h, &c.sub(0.0, 0.85))?;
        // 2. Keep the outer component.
        keep_outer(raw, h, &c.sub(0.85, 1.0))
    };

    let n_attempts = LATTICE_SHIFTS.len() as f32;
    if depth <= 0.0 {
        // Nothing to erode (offset within one skin): the outer dilation is the result.
        for (k, &shift) in LATTICE_SHIFTS.iter().enumerate() {
            let span = |i: f32| 0.03 + 0.97 * (k as f32 + i) / n_attempts;
            let actx = ctx.sub(span(0.0), span(1.0));
            let raw = dilated(shift, &actx.sub(0.0, 0.7))?;
            if let Some(out) = accept(raw, &dilate, tol, h, &actx.sub(0.7, 1.0))? {
                return Ok(out);
            }
        }
        return Err(no_clean_alignment());
    }

    let outer = dilated(LATTICE_SHIFTS[0], &ctx.sub(0.03, 0.4))?;
    // The boundary of `F` only drives the erosion field: decimate it for speed when the
    // decimation keeps its vertices on the dilation level, else use it as extracted.
    let fctx = ctx.sub(0.4, 0.5);
    let boundary = if tol > 0.0 {
        let c = simplify(&outer, tol, h, &fctx)?;
        if max_abs_field(&dilate, &c) <= MAX_VERTEX_ERROR * h {
            c
        } else {
            outer
        }
    } else {
        outer
    };
    drop(dilate);
    let f_aabb = boundary.aabb();
    let fspatial = Arc::new(MeshSpatial::build(
        Arc::new(boundary),
        &BvhOpts::default(),
        &ctx.sub(0.5, 0.52),
    )?);
    fspatial.prepare_winding(&ctx.sub(0.52, 0.55))?;
    // 3. Erosion by `depth` of the filled dilation.
    let erode = ErodeField::new(MeshSdf::new(fspatial, depth + 2.0 * h), depth);
    for (k, shift) in LATTICE_SHIFTS.iter().cycle().skip(1).take(3).enumerate() {
        let span = |i: f32| 0.55 + 0.45 * (k as f32 + i) / n_attempts;
        let actx = ctx.sub(span(0.0), span(1.0));
        let bounds = Aabb3::new(
            f_aabb.min - Vec3::splat(2.0 * h + shift * h),
            f_aabb.max + Vec3::splat(2.0 * h),
        );
        let raw = extract(&erode, bounds, h, &actx.sub(0.0, 0.6))?;
        if raw.tris.is_empty() {
            return Err(MeshFeatureError::Sdf(format!(
                "the wrap at {h} mm voxels with a {r} mm offset is empty"
            )));
        }
        if let Some(out) = accept(raw, &erode, tol, h, &actx.sub(0.6, 1.0))? {
            return Ok(out);
        }
    }
    Err(no_clean_alignment())
}

fn no_clean_alignment() -> MeshFeatureError {
    MeshFeatureError::Sdf("no lattice alignment gave a self-intersection-free wrap".into())
}

/// Bounding box of the finite vertices used by faces, `None` if there is none.
fn finite_aabb(m: &IndexedMesh) -> Option<Aabb3> {
    let mut lo = Point3::splat(f64::INFINITY);
    let mut hi = Point3::splat(f64::NEG_INFINITY);
    for t in &m.tris {
        for &v in t {
            if let Some(p) = m.positions.get(v as usize).filter(|p| p.is_finite()) {
                lo = lo.min(*p);
                hi = hi.max(*p);
            }
        }
    }
    lo.cmple(hi).all().then(|| Aabb3::new(lo, hi))
}

/// MESH-17 extraction of `field < 0` in `bounds` (8-cell blocks, one secant step).
fn extract(
    field: &dyn ScalarField,
    bounds: Aabb3,
    h: f64,
    ctx: &JobCtx,
) -> Result<IndexedMesh, MeshFeatureError> {
    let mut iso = IsoOpts::new(bounds, h);
    iso.block_cells = BLOCK_CELLS;
    iso.project_steps = 1;
    Ok(extract_isosurface(field, &iso, ctx)?)
}

/// Candidates in order: decimated at `tol`, at `tol / 4`, raw. A decimated candidate must keep
/// every vertex within `MAX_VERTEX_ERROR` voxels of `field = 0`; the first that MESH-19 finds
/// free of intersecting pairs is returned.
fn accept(
    raw: IndexedMesh,
    field: &dyn ScalarField,
    tol: f64,
    h: f64,
    ctx: &JobCtx,
) -> Result<Option<IndexedMesh>, MeshFeatureError> {
    let tols: &[f64] = if tol > 0.0 { &[1.0, 0.25] } else { &[] };
    for (j, &f) in tols.iter().enumerate() {
        let cctx = ctx.sub(0.4 * j as f32, 0.4 * (j + 1) as f32);
        let c = simplify(&raw, f * tol, h, &cctx)?;
        if max_abs_field(field, &c) > MAX_VERTEX_ERROR * h {
            continue;
        }
        let (c, clean) = is_si_free(c, &cctx)?;
        if clean {
            return Ok(Some(finish(c)));
        }
    }
    let (raw, clean) = is_si_free(raw, &ctx.sub(0.8, 1.0))?;
    Ok(clean.then(|| finish(raw)))
}

fn finish(mut m: IndexedMesh) -> IndexedMesh {
    m.groups = FaceGroups::new(m.tris.len());
    m
}

/// The outer component of a closed, oriented lattice mesh (MESH-17 output): drops shells of
/// negative volume (cavities) and positive shells nested inside another positive shell.
///
/// Nesting is decided by the winding number of all positive shells together at points just
/// outside each shell (off its three largest faces, by majority): MESH-17 shells never cross,
/// so the point is outside every other shell unless one encloses it.
fn keep_outer(raw: IndexedMesh, h: f64, ctx: &JobCtx) -> Result<IndexedMesh, MeshFeatureError> {
    ctx.checkpoint().map_err(MeshError::from)?;
    let shells: Vec<IndexedMesh> = separate_shells(&raw)
        .into_iter()
        .filter(|s| s.signed_volume() > 0.0)
        .collect();
    if shells.is_empty() {
        return Err(MeshFeatureError::Sdf(
            "the dilated input has no outer surface".into(),
        ));
    }
    if shells.len() == 1 {
        return Ok(merge(&shells));
    }
    let all = Arc::new(merge(&shells));
    let spatial = MeshSpatial::build(all, &BvhOpts::default(), &ctx.sub(0.0, 0.3))?;
    spatial.prepare_winding(&ctx.sub(0.3, 0.6))?;
    ctx.checkpoint().map_err(MeshError::from)?;
    let eps = 0.01 * h;
    let keep: Vec<bool> = shells
        .par_iter()
        .map(|s| {
            let probes = probe_points(s, eps);
            let inside = probes.iter().filter(|&&q| spatial.is_inside(q)).count();
            inside * 2 <= probes.len()
        })
        .collect();
    let kept: Vec<IndexedMesh> = shells
        .into_iter()
        .zip(keep)
        .filter_map(|(s, k)| k.then_some(s))
        .collect();
    Ok(merge(&kept))
}

/// Up to three points `eps` outside the largest faces of `s` (centroid + `eps` × unit normal).
fn probe_points(s: &IndexedMesh, eps: f64) -> Vec<Point3> {
    let mut faces: Vec<(f64, Point3, Vec3)> = s
        .tris
        .iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| s.positions[i as usize]);
            let n = (b - a).cross(c - a);
            (n.length(), (a + b + c) / 3.0, n)
        })
        .filter(|(l, ..)| *l > 0.0 && l.is_finite())
        .collect();
    // Largest first; ties by centroid coordinates, so the choice is deterministic.
    faces.sort_by(|x, y| {
        y.0.total_cmp(&x.0)
            .then(x.1.x.total_cmp(&y.1.x))
            .then(x.1.y.total_cmp(&y.1.y))
            .then(x.1.z.total_cmp(&y.1.z))
    });
    faces
        .iter()
        .take(3)
        .map(|(l, c, n)| *c + *n * (eps / l))
        .collect()
}

/// The shells as one mesh (positions and faces concatenated, one default group).
fn merge(shells: &[IndexedMesh]) -> IndexedMesh {
    let mut positions = Vec::with_capacity(shells.iter().map(|s| s.positions.len()).sum());
    let mut tris = Vec::with_capacity(shells.iter().map(|s| s.tris.len()).sum());
    for s in shells {
        let base = positions.len() as u32;
        positions.extend_from_slice(&s.positions);
        tris.extend(s.tris.iter().map(|t| t.map(|i| i + base)));
    }
    let n = tris.len();
    IndexedMesh {
        positions,
        tris,
        groups: FaceGroups::new(n),
        ..IndexedMesh::default()
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, reason = "tests")]

    use super::*;

    fn sphere(c: Point3, r: f64, flip: bool) -> IndexedMesh {
        let s = amg_testkit::shapes::icosphere(c, r, 3);
        let mut t = s.triangles;
        if flip {
            for f in &mut t {
                f.swap(1, 2);
            }
        }
        IndexedMesh::new(s.positions, t).unwrap()
    }

    /// Outer sphere, a cavity (inward sphere) and an island inside the cavity, plus a separate
    /// sphere outside: only the two outermost positive shells stay.
    #[test]
    fn keep_outer_drops_cavities_and_islands() {
        let shells = [
            sphere(Point3::ZERO, 10.0, false),
            sphere(Point3::ZERO, 8.0, true),
            sphere(Point3::ZERO, 3.0, false),
            sphere(Point3::new(30.0, 0.0, 0.0), 2.0, false),
        ];
        let out = keep_outer(merge(&shells), 0.5, &JobCtx::detached()).unwrap();
        let parts = separate_shells(&out);
        assert_eq!(parts.len(), 2);
        let expect = shells[0].signed_volume() + shells[3].signed_volume();
        assert!((out.signed_volume() - expect).abs() < 1e-9 * expect);
    }

    #[test]
    fn dilate_field_pruning_is_conservative() {
        let m = Arc::new(sphere(Point3::ZERO, 5.0, false));
        let sp = Arc::new(MeshSpatial::build(m, &BvhOpts::default(), &JobCtx::detached()).unwrap());
        let f = DilateField::new(MeshSdf::new(sp, 3.0), 1.0);
        // A box around the dilated surface crosses; one far away or deep inside does not.
        let near = Aabb3::new(Point3::new(5.5, -0.5, -0.5), Point3::new(6.5, 0.5, 0.5));
        assert!(f.may_cross(&near, 0.0));
        let far = Aabb3::new(Point3::splat(20.0), Point3::splat(21.0));
        assert!(!f.may_cross(&far, 0.0));
        let deep = Aabb3::new(Point3::splat(-0.5), Point3::splat(0.5));
        assert!(!f.may_cross(&deep, 0.0));
        assert!(!f.may_cross(&near, -2.0)); // level below zero is never reached
    }
}
