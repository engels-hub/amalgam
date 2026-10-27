//! Primitive fitting (MESH-24): Efficient RANSAC (Schnabel et al. 2007) with Levenberg–Marquardt
//! (damped Gauss–Newton) refinement on faithful distance functions (Lukács et al. 1998).
//!
//! 1. Points are moved to their centroid and scaled to unit rms radius, so the tolerances and
//!    finite-difference steps below are scale free.
//! 2. RANSAC: candidates from minimal samples of oriented points (plane: 3 points; sphere and
//!    cylinder: 2 points with normals; cone: 3 points with normals, apex = intersection of the
//!    tangent planes; torus: 8 normal lines, axis = the line meeting all of them in the least
//!    squares Plücker sense, then a circle fit in the meridian half plane). A candidate is
//!    scored on a fixed random subset (at most [`SCORE_SUBSET`] points) by the points within
//!    `epsilon` whose normal is within `normal_threshold` of the surface normal (orientation
//!    ignored). Sampling stops when the best candidate is found with 99 % probability (at least
//!    [`MIN_ITERATIONS`], at most `max_iterations` draws).
//! 3. Refinement: Levenberg–Marquardt on the signed distances of the inliers (at most
//!    [`REFINE_POINTS`] of them, evenly strided), with the parameters in a local chart around
//!    the current estimate (axis tilts in the plane normal to the axis), then the inliers are
//!    recomputed on all points; up to four rounds.
//!
//! Every random draw comes from a SplitMix64 stream seeded by `FitOpts::seed` and the kind, so
//! the result depends only on the input and the options.

// Small dense matrices (3x3 to 7x7) read best with explicit row/column indices.
#![allow(clippy::needless_range_loop)]

use amg_core::{Point3, Vec3};
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::error::{MeshError, MeshResult};
use crate::groups::{FaceGroupId, Primitive, PrimitiveKind};
use crate::{FaceId, IndexedMesh};

/// Candidates are scored on at most this many points.
pub const SCORE_SUBSET: usize = 4096;
/// The refinement uses at most this many inliers.
pub const REFINE_POINTS: usize = 50_000;
/// RANSAC draws at least this many samples (unless `max_iterations` is lower).
pub const MIN_ITERATIONS: u32 = 16;
/// [`fit_group_primitives`]: a more complex kind replaces a simpler one only when its support
/// is larger by more than this fraction of the group's points.
const TIE_FRACTION: f64 = 0.02;
/// Groups per cancellation checkpoint in [`fit_group_primitives`].
const GROUP_CHUNK: usize = 64;

/// Options of [`fit_primitive`] and [`fit_group_primitives`].
#[derive(Clone, Debug, PartialEq)]
pub struct FitOpts {
    /// Kinds to try ([`fit_group_primitives`] tries them simplest first, in the order of
    /// [`PrimitiveKind`]).
    pub kinds: Vec<PrimitiveKind>,
    /// Largest distance of an inlier from the surface, mm (> 0).
    pub epsilon: f64,
    /// Largest angle between an inlier's normal and the surface normal, radians, in
    /// `(0, pi/2]`; normal orientation is ignored.
    pub normal_threshold: f64,
    /// Fewest inliers of a result.
    pub min_support: usize,
    /// [`fit_group_primitives`]: fewest inliers of a stored fit as a fraction of the group's
    /// points, in `[0, 1]`.
    pub min_coverage: f64,
    /// Random seed.
    pub seed: u64,
    /// Largest number of RANSAC draws.
    pub max_iterations: u32,
}

impl Default for FitOpts {
    fn default() -> Self {
        FitOpts {
            kinds: vec![
                PrimitiveKind::Plane,
                PrimitiveKind::Sphere,
                PrimitiveKind::Cylinder,
                PrimitiveKind::Cone,
                PrimitiveKind::Torus,
            ],
            epsilon: 0.1,
            normal_threshold: 20f64.to_radians(),
            min_support: 10,
            min_coverage: 0.9,
            seed: 0,
            max_iterations: 1000,
        }
    }
}

/// A fitted primitive with its fit statistics over the inliers (mm).
#[derive(Clone, Debug, PartialEq)]
pub struct FitResult {
    /// The primitive (canonical form, see [`fit_primitive`]).
    pub primitive: Primitive,
    /// Root mean square distance of the inliers.
    pub rms: f64,
    /// Largest distance of an inlier.
    pub max_dev: f64,
    /// Number of inliers.
    pub support: usize,
}

impl Primitive {
    /// Signed distance of `p` from the surface, mm: positive along the plane normal, outside a
    /// sphere / cylinder / torus and on the convex side of a cone. For a cone this is the
    /// distance to the nappe the axis points into (`rho cos a - h sin a`, Lukács 1998); it is
    /// the exact distance wherever the foot point is not past the apex.
    pub fn signed_distance(&self, p: Point3) -> f64 {
        match *self {
            Primitive::Plane { origin, normal } => (p - origin).dot(normal),
            Primitive::Sphere { center, radius } => (p - center).length() - radius,
            Primitive::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let v = p - origin;
                (v - axis * v.dot(axis)).length() - radius
            }
            Primitive::Cone {
                apex,
                axis,
                half_angle,
            } => {
                let v = p - apex;
                let h = v.dot(axis);
                let rho = (v - axis * h).length();
                rho * half_angle.cos() - h * half_angle.sin()
            }
            Primitive::Torus {
                center,
                axis,
                major,
                minor,
            } => {
                let v = p - center;
                let h = v.dot(axis);
                let rho = (v - axis * h).length();
                (rho - major).hypot(h) - minor
            }
        }
    }

    /// Unit surface normal at the foot point of `p` (zero where it is undefined, e.g. on the
    /// axis).
    fn normal_at(&self, p: Point3) -> Vec3 {
        match *self {
            Primitive::Plane { normal, .. } => normal,
            Primitive::Sphere { center, .. } => (p - center).normalize_or_zero(),
            Primitive::Cylinder { origin, axis, .. } => {
                let v = p - origin;
                (v - axis * v.dot(axis)).normalize_or_zero()
            }
            Primitive::Cone {
                apex,
                axis,
                half_angle,
            } => {
                let v = p - apex;
                let radial = (v - axis * v.dot(axis)).normalize_or_zero();
                if radial == Vec3::ZERO {
                    return Vec3::ZERO;
                }
                radial * half_angle.cos() - axis * half_angle.sin()
            }
            Primitive::Torus {
                center,
                axis,
                major,
                ..
            } => {
                let v = p - center;
                let h = v.dot(axis);
                let radial = (v - axis * h).normalize_or_zero();
                let rho = (v - axis * h).length();
                (radial * (rho - major) + axis * h).normalize_or_zero()
            }
        }
    }

    fn is_finite(&self) -> bool {
        match *self {
            Primitive::Plane { origin, normal } => origin.is_finite() && normal.is_finite(),
            Primitive::Sphere { center, radius } => center.is_finite() && radius.is_finite(),
            Primitive::Cylinder {
                origin,
                axis,
                radius,
            } => origin.is_finite() && axis.is_finite() && radius.is_finite(),
            Primitive::Cone {
                apex,
                axis,
                half_angle,
            } => apex.is_finite() && axis.is_finite() && half_angle.is_finite(),
            Primitive::Torus {
                center,
                axis,
                major,
                minor,
            } => center.is_finite() && axis.is_finite() && major.is_finite() && minor.is_finite(),
        }
    }

    /// `self` with every point mapped by `p * s + c` and every length scaled by `s`.
    fn scaled(&self, c: Vec3, s: f64) -> Primitive {
        match *self {
            Primitive::Plane { origin, normal } => Primitive::Plane {
                origin: origin * s + c,
                normal,
            },
            Primitive::Sphere { center, radius } => Primitive::Sphere {
                center: center * s + c,
                radius: radius * s,
            },
            Primitive::Cylinder {
                origin,
                axis,
                radius,
            } => Primitive::Cylinder {
                origin: origin * s + c,
                axis,
                radius: radius * s,
            },
            Primitive::Cone {
                apex,
                axis,
                half_angle,
            } => Primitive::Cone {
                apex: apex * s + c,
                axis,
                half_angle,
            },
            Primitive::Torus {
                center,
                axis,
                major,
                minor,
            } => Primitive::Torus {
                center: center * s + c,
                axis,
                major: major * s,
                minor: minor * s,
            },
        }
    }
}

/// Fits one primitive of `kind` to oriented points (`normals[i]` belongs to `points[i]`; unit
/// length is not required).
///
/// Canonical form of the result: a plane's normal points along the mean input normal and its
/// origin is the foot of the centroid; cylinder and torus axes have their largest component
/// positive; a cylinder's origin is the foot of the inliers' centroid on the axis; a cone's
/// axis points from the apex into the cone and its half angle is in `(0, pi/2)`.
///
/// `None` when `points` and `normals` differ in length, there are too few points, an option is
/// out of range (see [`FitOpts`]) or no candidate reaches `min_support` inliers.
pub fn fit_primitive(
    points: &[Point3],
    normals: &[Vec3],
    kind: PrimitiveKind,
    opts: &FitOpts,
) -> Option<FitResult> {
    if points.len() != normals.len() || check_opts(opts).is_err() {
        return None;
    }
    let n = points.len();
    let k = sample_size(kind);
    if n < k.max(opts.min_support).max(1) {
        return None;
    }
    // Local frame: centroid at the origin, unit rms radius.
    let c = points.iter().copied().sum::<Vec3>() / n as f64;
    let s = (points
        .iter()
        .map(|p| (*p - c).length_squared())
        .sum::<f64>()
        / n as f64)
        .sqrt();
    if !(s.is_finite() && s > 0.0) {
        return None;
    }
    let pts: Vec<Point3> = points.iter().map(|p| (*p - c) / s).collect();
    let nrm: Vec<Vec3> = normals.iter().map(|v| v.normalize_or_zero()).collect();
    let eps = opts.epsilon / s;
    let cos_t = opts.normal_threshold.cos();
    let mean_normal = nrm.iter().copied().sum::<Vec3>();

    let mut rng = SplitMix64(opts.seed ^ kind_salt(kind));
    let subset = random_subset(&mut rng, n, SCORE_SUBSET);
    let sub_pts: Vec<Point3> = subset.iter().map(|&i| pts[i]).collect();
    let sub_nrm: Vec<Vec3> = subset.iter().map(|&i| nrm[i]).collect();

    let mut best: Option<(usize, Primitive)> = None;
    let mut required = u64::MAX;
    let mut sample = vec![0usize; k];
    let mut it = 0u32;
    while it < opts.max_iterations && (u64::from(it) < required || it < MIN_ITERATIONS) {
        it += 1;
        draw_distinct(&mut rng, sub_pts.len(), &mut sample);
        let sp: Vec<Point3> = sample.iter().map(|&i| sub_pts[i]).collect();
        let sn: Vec<Vec3> = sample.iter().map(|&i| sub_nrm[i]).collect();
        let Some(cand) = candidate(kind, &sp, &sn).map(|p| canonical(p, mean_normal)) else {
            continue;
        };
        if !cand.is_finite()
            || !sp
                .iter()
                .zip(&sn)
                .all(|(p, v)| is_inlier(&cand, *p, *v, eps, cos_t))
        {
            continue;
        }
        let score = sub_pts
            .iter()
            .zip(&sub_nrm)
            .filter(|(p, v)| is_inlier(&cand, **p, **v, eps, cos_t))
            .count();
        if best.as_ref().is_none_or(|(b, _)| score > *b) {
            best = Some((score, cand));
            let w = score as f64 / sub_pts.len() as f64;
            let miss = 1.0 - w.powi(k as i32);
            required = if miss <= 0.0 {
                0
            } else if miss >= 1.0 {
                u64::MAX
            } else {
                (0.01f64.ln() / miss.ln()).ceil().min(u64::MAX as f64) as u64
            };
        }
    }
    let (_, mut prim) = best?;

    let inliers = |prim: &Primitive| -> Vec<usize> {
        (0..n)
            .filter(|&i| is_inlier(prim, pts[i], nrm[i], eps, cos_t))
            .collect()
    };
    let mut inl = inliers(&prim);
    for _ in 0..4 {
        if inl.len() < k {
            return None;
        }
        let stride = inl.len().div_ceil(REFINE_POINTS);
        let fit_pts: Vec<Point3> = inl.iter().step_by(stride).map(|&i| pts[i]).collect();
        let refined = canonical(refine(prim.clone(), &fit_pts), mean_normal);
        if !refined.is_finite() {
            break;
        }
        // The least-squares fit replaces the sample-based candidate even when a few inliers at
        // the edge of the epsilon band drop out; a refit that loses most of them is rejected.
        let next = inliers(&refined);
        if next.len() < k || 2 * next.len() < inl.len() {
            break;
        }
        prim = refined;
        let done = next == inl;
        inl = next;
        if done {
            break;
        }
    }
    if inl.len() < opts.min_support.max(1) {
        return None;
    }
    // Final canonical form with the inliers' centroid as the reference point.
    let ref_pt = inl.iter().map(|&i| pts[i]).sum::<Vec3>() / inl.len() as f64;
    prim = with_reference(prim, ref_pt);
    let (sum2, max) = inl.iter().fold((0.0, 0.0f64), |(s2, m), &i| {
        let d = prim.signed_distance(pts[i]).abs();
        (s2 + d * d, m.max(d))
    });
    Some(FitResult {
        primitive: prim.scaled(c, s),
        rms: (sum2 / inl.len() as f64).sqrt() * s,
        max_dev: max * s,
        support: inl.len(),
    })
}

/// Fits a primitive to every face group of `m` that has faces and stores it in the group table
/// (`FaceGroupInfo::primitive`; `None` for a group without an acceptable fit).
///
/// A group's points are the vertices of its faces, with normals averaged (area weighted) over
/// the group's own faces. Each kind of `opts.kinds` is fitted with [`fit_primitive`] (seed
/// mixed with the group id); a fit counts when its support is at least `min_coverage` of the
/// points; among those the simplest kind wins unless a more complex one has more than 2 % more
/// support, or about the same support (within 2 %) and less than half the rms of a fit whose
/// rms is above `epsilon / 10`. Returns the fitted groups in ascending id order.
///
/// # Errors
/// `InvalidOption` for an option out of range, `Cancelled` when `ctx` is cancelled (the mesh is
/// then unchanged).
pub fn fit_group_primitives(
    m: &mut IndexedMesh,
    opts: &FitOpts,
    ctx: &JobCtx,
) -> MeshResult<Vec<(FaceGroupId, FitResult)>> {
    check_opts(opts)?;
    let mut kinds = opts.kinds.clone();
    kinds.sort_by_key(|k| kind_rank(*k));
    kinds.dedup();

    // Faces of every group, by one pass over the faces (ascending face id inside a group).
    let mut faces: std::collections::BTreeMap<FaceGroupId, Vec<u32>> = Default::default();
    for (f, g) in m.groups.per_face().iter().enumerate() {
        faces.entry(*g).or_default().push(f as u32);
    }
    let groups: Vec<(FaceGroupId, Vec<u32>)> = faces.into_iter().collect();
    ctx.checkpoint()?;
    ctx.progress(0.05);

    let mesh: &IndexedMesh = m;
    let mut fits: Vec<(FaceGroupId, Option<FitResult>)> = Vec::with_capacity(groups.len());
    for (ci, chunk) in groups.chunks(GROUP_CHUNK).enumerate() {
        ctx.checkpoint()?;
        let part: Vec<(FaceGroupId, Option<FitResult>)> = chunk
            .par_iter()
            .map(|(g, fs)| (*g, fit_group(mesh, fs, &kinds, opts, *g)))
            .collect();
        fits.extend(part);
        let done = ((ci + 1) * GROUP_CHUNK).min(groups.len());
        ctx.progress(0.05 + 0.95 * done as f32 / groups.len().max(1) as f32);
    }
    ctx.checkpoint()?;

    let mut out = Vec::new();
    for (g, fit) in fits {
        if let Some(info) = m.groups.info_mut(g) {
            info.primitive = fit.as_ref().map(|r| r.primitive.clone());
        }
        if let Some(r) = fit {
            out.push((g, r));
        }
    }
    ctx.progress(1.0);
    Ok(out)
}

fn fit_group(
    m: &IndexedMesh,
    faces: &[u32],
    kinds: &[PrimitiveKind],
    opts: &FitOpts,
    g: FaceGroupId,
) -> Option<FitResult> {
    let mut verts: Vec<u32> = faces.iter().flat_map(|&f| m.tris[f as usize]).collect();
    verts.sort_unstable();
    verts.dedup();
    let mut normals = vec![Vec3::ZERO; verts.len()];
    for &f in faces {
        let [a, b, c] = m.triangle(FaceId(f));
        let n = (b - a).cross(c - a);
        for v in m.tris[f as usize] {
            if let Ok(i) = verts.binary_search(&v) {
                normals[i] += n;
            }
        }
    }
    let points: Vec<Point3> = verts.iter().map(|&v| m.positions[v as usize]).collect();
    let n = points.len();
    let need = (opts.min_coverage * n as f64).ceil() as usize;
    let tie = (TIE_FRACTION * n as f64).ceil() as usize;
    let group_opts = FitOpts {
        seed: SplitMix64(opts.seed ^ u64::from(g.0).wrapping_mul(0xA24B_AED4_963E_E407)).next(),
        ..opts.clone()
    };
    let mut best: Option<FitResult> = None;
    for &kind in kinds {
        let Some(r) = fit_primitive(&points, &normals, kind, &group_opts) else {
            continue;
        };
        if r.support < need {
            continue;
        }
        // A more complex kind wins with clearly more support, or with about the same support
        // and a clearly better fit (a sphere covers a narrow cone band within epsilon, too).
        let replaces = |b: &FitResult| {
            r.support > b.support + tie
                || (r.support + tie >= b.support
                    && b.rms > 0.1 * opts.epsilon
                    && r.rms < 0.5 * b.rms)
        };
        if best.as_ref().is_none_or(replaces) {
            best = Some(r);
        }
    }
    best
}

fn check_opts(opts: &FitOpts) -> MeshResult<()> {
    if !(opts.epsilon.is_finite() && opts.epsilon > 0.0) {
        return Err(MeshError::InvalidOption {
            name: "epsilon",
            reason: format!("must be finite and > 0, got {}", opts.epsilon),
        });
    }
    if !(opts.normal_threshold > 0.0 && opts.normal_threshold <= std::f64::consts::FRAC_PI_2) {
        return Err(MeshError::InvalidOption {
            name: "normal_threshold",
            reason: format!("must be in (0, pi/2], got {}", opts.normal_threshold),
        });
    }
    if !(0.0..=1.0).contains(&opts.min_coverage) {
        return Err(MeshError::InvalidOption {
            name: "min_coverage",
            reason: format!("must be in [0, 1], got {}", opts.min_coverage),
        });
    }
    Ok(())
}

fn kind_rank(k: PrimitiveKind) -> u8 {
    match k {
        PrimitiveKind::Plane => 0,
        PrimitiveKind::Sphere => 1,
        PrimitiveKind::Cylinder => 2,
        PrimitiveKind::Cone => 3,
        PrimitiveKind::Torus => 4,
    }
}

fn kind_salt(k: PrimitiveKind) -> u64 {
    0x5851_F42D_4C95_7F2D_u64.wrapping_mul(u64::from(kind_rank(k)) + 1)
}

fn sample_size(k: PrimitiveKind) -> usize {
    match k {
        PrimitiveKind::Plane | PrimitiveKind::Cone => 3,
        PrimitiveKind::Sphere | PrimitiveKind::Cylinder => 2,
        PrimitiveKind::Torus => 8,
    }
}

fn is_inlier(prim: &Primitive, p: Point3, n: Vec3, eps: f64, cos_t: f64) -> bool {
    prim.signed_distance(p).abs() <= eps && prim.normal_at(p).dot(n).abs() >= cos_t
}

// ------------------------------------------------------------------ random numbers

/// SplitMix64 (Steele, Lea & Flood 2014): small, fast, and good enough for sampling.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `0..n` (`n > 0`), by multiply-shift (bias below 2^-32 for `n < 2^32`).
    fn below(&mut self, n: usize) -> usize {
        ((u128::from(self.next()) * n as u128) >> 64) as usize
    }
}

/// `min(n, max)` distinct indices of `0..n` (all of them, in order, when `n <= max`).
fn random_subset(rng: &mut SplitMix64, n: usize, max: usize) -> Vec<usize> {
    let mut idx: Vec<usize> = (0..n).collect();
    if n <= max {
        return idx;
    }
    for i in 0..max {
        let j = i + rng.below(n - i);
        idx.swap(i, j);
    }
    idx.truncate(max);
    idx
}

/// Fills `out` with distinct indices of `0..n` (`out.len() <= n`).
fn draw_distinct(rng: &mut SplitMix64, n: usize, out: &mut [usize]) {
    for i in 0..out.len() {
        loop {
            let x = rng.below(n);
            if !out[..i].contains(&x) {
                out[i] = x;
                break;
            }
        }
    }
}

// ------------------------------------------------------------------ candidates

/// Closest points of the lines `p + t u` and `q + s v`; `None` when nearly parallel.
fn closest_on_lines(p: Point3, u: Vec3, q: Point3, v: Vec3) -> Option<(Point3, Point3)> {
    let w = p - q;
    let (a, b, c) = (u.dot(u), u.dot(v), v.dot(v));
    let (d, e) = (u.dot(w), v.dot(w));
    let den = a * c - b * b;
    if den <= 1e-12 * a * c {
        return None;
    }
    let t = (b * e - c * d) / den;
    let s = (a * e - b * d) / den;
    Some((p + u * t, q + v * s))
}

fn candidate(kind: PrimitiveKind, p: &[Point3], n: &[Vec3]) -> Option<Primitive> {
    match kind {
        PrimitiveKind::Plane => {
            let normal = (p[1] - p[0]).cross(p[2] - p[0]);
            let len = normal.length();
            if len <= 1e-9 * (p[1] - p[0]).length() * (p[2] - p[0]).length() || len == 0.0 {
                return None;
            }
            Some(Primitive::Plane {
                origin: p[0],
                normal: normal / len,
            })
        }
        PrimitiveKind::Sphere => {
            let (a, b) = closest_on_lines(p[0], n[0], p[1], n[1])?;
            let center = (a + b) * 0.5;
            let radius = 0.5 * ((p[0] - center).length() + (p[1] - center).length());
            Some(Primitive::Sphere { center, radius })
        }
        PrimitiveKind::Cylinder => {
            let axis = n[0].cross(n[1]).normalize_or_zero();
            if axis == Vec3::ZERO || n[0].cross(n[1]).length() < 1e-6 {
                return None;
            }
            // Project onto the plane normal to the axis and intersect the normal lines there.
            let flat = |v: Vec3| v - axis * v.dot(axis);
            let (a, _) = closest_on_lines(flat(p[0]), flat(n[0]), flat(p[1]), flat(n[1]))?;
            let origin = a;
            let radius = 0.5 * ((flat(p[0]) - origin).length() + (flat(p[1]) - origin).length());
            Some(Primitive::Cylinder {
                origin,
                axis,
                radius,
            })
        }
        PrimitiveKind::Cone => {
            // Apex: intersection of the three tangent planes n_i . x = n_i . p_i.
            let rows = [n[0], n[1], n[2]];
            let rhs = [n[0].dot(p[0]), n[1].dot(p[1]), n[2].dot(p[2])];
            let apex = solve3(rows, rhs)?;
            let u: Vec<Vec3> = p.iter().map(|q| (*q - apex).normalize_or_zero()).collect();
            if u.contains(&Vec3::ZERO) {
                return None;
            }
            let mut axis = (u[1] - u[0]).cross(u[2] - u[0]).normalize_or_zero();
            if axis == Vec3::ZERO {
                return None;
            }
            if axis.dot(u[0] + u[1] + u[2]) < 0.0 {
                axis = -axis;
            }
            let half_angle = u
                .iter()
                .map(|v| v.dot(axis).clamp(-1.0, 1.0).acos())
                .sum::<f64>()
                / 3.0;
            if !(1e-3..std::f64::consts::FRAC_PI_2 - 1e-3).contains(&half_angle) {
                return None;
            }
            Some(Primitive::Cone {
                apex,
                axis,
                half_angle,
            })
        }
        PrimitiveKind::Torus => torus_candidate(p, n),
    }
}

/// Torus from oriented points: the axis is the line meeting every normal line (least squares
/// over Plücker coordinates: line `(d, m)` meets `(d_i, m_i)` iff `d . m_i + m . d_i = 0`), then
/// a circle fit (Kåsa) to the points in the meridian half plane gives the tube.
fn torus_candidate(p: &[Point3], n: &[Vec3]) -> Option<Primitive> {
    let mut a = [[0.0; 6]; 6];
    for (q, d) in p.iter().zip(n) {
        let m = q.cross(*d);
        let row = [m.x, m.y, m.z, d.x, d.y, d.z];
        for i in 0..6 {
            for j in 0..6 {
                a[i][j] += row[i] * row[j];
            }
        }
    }
    let u = smallest_eigenvector(a);
    let d = Vec3::new(u[0], u[1], u[2]);
    let m = Vec3::new(u[3], u[4], u[5]);
    let dl = d.length();
    if dl < 1e-6 {
        return None;
    }
    let axis = d / dl;
    let on_axis = d.cross(m) / (dl * dl);
    // Meridian coordinates (rho, h) relative to `on_axis`; Kåsa circle fit.
    let mut ata = [[0.0; 3]; 3];
    let mut atb = [0.0; 3];
    for q in p {
        let v = *q - on_axis;
        let h = v.dot(axis);
        let rho = (v - axis * h).length();
        let row = [rho, h, 1.0];
        let rhs = rho * rho + h * h;
        for i in 0..3 {
            for j in 0..3 {
                ata[i][j] += row[i] * row[j];
            }
            atb[i] += row[i] * rhs;
        }
    }
    let rows = ata.map(|r| Vec3::new(r[0], r[1], r[2]));
    let x = solve3(rows, atb)?;
    let (rc, hc) = (0.5 * x.x, 0.5 * x.y);
    let r2 = x.z + rc * rc + hc * hc;
    if !(r2 > 0.0 && rc > 0.0) {
        return None;
    }
    Some(Primitive::Torus {
        center: on_axis + axis * hc,
        axis,
        major: rc,
        minor: r2.sqrt(),
    })
}

/// Solves the 3x3 system with rows `r`; `None` when (nearly) singular.
fn solve3(r: [Vec3; 3], b: [f64; 3]) -> Option<Vec3> {
    let det = r[0].dot(r[1].cross(r[2]));
    let scale = r[0].length() * r[1].length() * r[2].length();
    if det.is_nan() || det.abs() <= 1e-9 * scale {
        return None;
    }
    // Cramer's rule via the cofactor rows.
    let c0 = r[1].cross(r[2]);
    let c1 = r[2].cross(r[0]);
    let c2 = r[0].cross(r[1]);
    Some((c0 * b[0] + c1 * b[1] + c2 * b[2]) / det)
}

/// Unit eigenvector of the smallest eigenvalue of a symmetric 6x6 matrix (cyclic Jacobi).
fn smallest_eigenvector(mut a: [[f64; 6]; 6]) -> [f64; 6] {
    let mut v = [[0.0; 6]; 6];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for _ in 0..50 {
        let off: f64 = (0..6)
            .flat_map(|i| (0..6).filter(move |&j| j != i).map(move |j| (i, j)))
            .map(|(i, j)| a[i][j] * a[i][j])
            .sum();
        let diag: f64 = (0..6).map(|i| a[i][i] * a[i][i]).sum();
        if off <= 1e-30 * diag || off == 0.0 {
            break;
        }
        for p in 0..5 {
            for q in p + 1..6 {
                if a[p][q] == 0.0 {
                    continue;
                }
                let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..6 {
                    let (akp, akq) = (a[k][p], a[k][q]);
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for k in 0..6 {
                    let (apk, aqk) = (a[p][k], a[q][k]);
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for row in &mut v {
                    let (vkp, vkq) = (row[p], row[q]);
                    row[p] = c * vkp - s * vkq;
                    row[q] = s * vkp + c * vkq;
                }
            }
        }
    }
    let mut best = 0;
    for i in 1..6 {
        if a[i][i] < a[best][best] {
            best = i;
        }
    }
    std::array::from_fn(|k| v[k][best])
}

// ------------------------------------------------------------------ canonical form

/// Axis sign with the largest component positive (ties: first axis).
fn canonical_axis(a: Vec3) -> Vec3 {
    let c = [a.x, a.y, a.z];
    let mut i = 0;
    for j in 1..3 {
        if c[j].abs() > c[i].abs() {
            i = j;
        }
    }
    if c[i] < 0.0 { -a } else { a }
}

/// Sign and range conventions (see [`fit_primitive`]), with the local origin as reference point.
fn canonical(p: Primitive, mean_normal: Vec3) -> Primitive {
    let p = match p {
        Primitive::Plane { origin, normal } => {
            let normal = normal.normalize_or_zero();
            let normal = if normal.dot(mean_normal) < 0.0 {
                -normal
            } else {
                normal
            };
            Primitive::Plane { origin, normal }
        }
        Primitive::Sphere { center, radius } => Primitive::Sphere {
            center,
            radius: radius.abs(),
        },
        Primitive::Cylinder {
            origin,
            axis,
            radius,
        } => Primitive::Cylinder {
            origin,
            axis: canonical_axis(axis.normalize_or_zero()),
            radius: radius.abs(),
        },
        Primitive::Cone {
            apex,
            axis,
            half_angle,
        } => {
            let mut axis = axis.normalize_or_zero();
            let mut a = half_angle.rem_euclid(std::f64::consts::TAU);
            if a > std::f64::consts::PI {
                a -= std::f64::consts::TAU;
            }
            if a < 0.0 {
                // rho cos(-a) - h sin(-a) = rho cos a - (-h) sin a.
                a = -a;
                axis = -axis;
            }
            if a > std::f64::consts::FRAC_PI_2 {
                // rho cos a - h sin a = -(rho cos(pi - a) - (-h) sin(pi - a)).
                a = std::f64::consts::PI - a;
                axis = -axis;
            }
            Primitive::Cone {
                apex,
                axis,
                half_angle: a,
            }
        }
        Primitive::Torus {
            center,
            axis,
            major,
            minor,
        } => Primitive::Torus {
            center,
            axis: canonical_axis(axis.normalize_or_zero()),
            major,
            minor: minor.abs(),
        },
    };
    with_reference(p, Vec3::ZERO)
}

/// Moves a plane's origin / a cylinder's origin to the foot of `r`.
fn with_reference(p: Primitive, r: Point3) -> Primitive {
    match p {
        Primitive::Plane { origin, normal } => Primitive::Plane {
            origin: r - normal * (r - origin).dot(normal),
            normal,
        },
        Primitive::Cylinder {
            origin,
            axis,
            radius,
        } => Primitive::Cylinder {
            origin: origin + axis * (r - origin).dot(axis),
            axis,
            radius,
        },
        other => other,
    }
}

// ------------------------------------------------------------------ refinement

fn n_params(p: &Primitive) -> usize {
    match p {
        Primitive::Plane { .. } => 3,
        Primitive::Sphere { .. } => 4,
        Primitive::Cylinder { .. } => 5,
        Primitive::Cone { .. } => 6,
        Primitive::Torus { .. } => 7,
    }
}

fn tilt(axis: Vec3, x0: f64, x1: f64) -> Vec3 {
    let (e1, e2) = axis.any_orthonormal_pair();
    (axis + e1 * x0 + e2 * x1).normalize()
}

/// The primitive at local chart coordinates `x` around `p` (`x = 0` is `p`).
fn perturb(p: &Primitive, x: &[f64]) -> Primitive {
    match *p {
        Primitive::Plane { origin, normal } => Primitive::Plane {
            origin: origin + normal * x[2],
            normal: tilt(normal, x[0], x[1]),
        },
        Primitive::Sphere { center, radius } => Primitive::Sphere {
            center: center + Vec3::new(x[0], x[1], x[2]),
            radius: radius + x[3],
        },
        Primitive::Cylinder {
            origin,
            axis,
            radius,
        } => {
            let (e1, e2) = axis.any_orthonormal_pair();
            Primitive::Cylinder {
                origin: origin + e1 * x[2] + e2 * x[3],
                axis: tilt(axis, x[0], x[1]),
                radius: radius + x[4],
            }
        }
        Primitive::Cone {
            apex,
            axis,
            half_angle,
        } => Primitive::Cone {
            apex: apex + Vec3::new(x[0], x[1], x[2]),
            axis: tilt(axis, x[3], x[4]),
            half_angle: half_angle + x[5],
        },
        Primitive::Torus {
            center,
            axis,
            major,
            minor,
        } => Primitive::Torus {
            center: center + Vec3::new(x[0], x[1], x[2]),
            axis: tilt(axis, x[3], x[4]),
            major: major + x[5],
            minor: minor + x[6],
        },
    }
}

fn cost(p: &Primitive, pts: &[Point3]) -> f64 {
    pts.iter().map(|q| p.signed_distance(*q).powi(2)).sum()
}

/// Levenberg–Marquardt on the signed distances (central-difference Jacobian in the local chart).
fn refine(start: Primitive, pts: &[Point3]) -> Primitive {
    const H: f64 = 1e-6;
    let k = n_params(&start);
    let mut cur = start;
    let mut c = cost(&cur, pts);
    let mut lambda = 1e-3;
    for _ in 0..100 {
        if c <= 1e-30 * pts.len() as f64 {
            break;
        }
        let shifted: Vec<(Primitive, Primitive)> = (0..k)
            .map(|j| {
                let mut x = [0.0; 7];
                x[j] = H;
                let plus = perturb(&cur, &x[..k]);
                x[j] = -H;
                (plus, perturb(&cur, &x[..k]))
            })
            .collect();
        let mut jtj = [[0.0; 7]; 7];
        let mut jtr = [0.0; 7];
        let mut row = [0.0; 7];
        for q in pts {
            let r = cur.signed_distance(*q);
            for (j, (plus, minus)) in shifted.iter().enumerate() {
                row[j] = (plus.signed_distance(*q) - minus.signed_distance(*q)) / (2.0 * H);
            }
            for i in 0..k {
                jtr[i] += row[i] * r;
                for j in 0..=i {
                    jtj[i][j] += row[i] * row[j];
                }
            }
        }
        for i in 0..k {
            for j in 0..i {
                jtj[j][i] = jtj[i][j];
            }
        }
        let mut improved = false;
        while lambda <= 1e12 {
            let mut a = jtj;
            for (i, r) in a.iter_mut().enumerate().take(k) {
                r[i] += lambda * jtj[i][i].max(1e-12);
            }
            let rhs: [f64; 7] = std::array::from_fn(|i| -jtr[i]);
            let Some(dx) = solve_n(a, rhs, k) else {
                lambda *= 10.0;
                continue;
            };
            let next = canonical(perturb(&cur, &dx[..k]), Vec3::ZERO);
            let nc = cost(&next, pts);
            if next.is_finite() && nc < c {
                let step = dx[..k].iter().map(|v| v * v).sum::<f64>().sqrt();
                let gain = c - nc;
                cur = next;
                c = nc;
                lambda = (lambda * 0.1).max(1e-15);
                improved = true;
                if step < 1e-13 || gain <= 1e-15 * c {
                    return cur;
                }
                break;
            }
            lambda *= 10.0;
        }
        if !improved {
            break;
        }
    }
    cur
}

/// Gaussian elimination with partial pivoting on the leading `k x k` block.
fn solve_n(mut a: [[f64; 7]; 7], mut b: [f64; 7], k: usize) -> Option<[f64; 7]> {
    for col in 0..k {
        let piv = (col..k).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[piv][col].is_nan() || a[piv][col].abs() <= 1e-300 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for r in col + 1..k {
            let f = a[r][col] / a[col][col];
            for c in col..k {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = [0.0; 7];
    for r in (0..k).rev() {
        let s: f64 = (r + 1..k).map(|c| a[r][c] * x[c]).sum();
        x[r] = (b[r] - s) / a[r][r];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eigenvector_of_diagonal_matrix() {
        let mut a = [[0.0; 6]; 6];
        for (i, r) in a.iter_mut().enumerate() {
            r[i] = 6.0 - i as f64 + 1.0;
        }
        a[0][1] = 0.5;
        a[1][0] = 0.5;
        let v = smallest_eigenvector(a);
        assert!((v[5].abs() - 1.0).abs() < 1e-12, "{v:?}");
    }

    #[test]
    fn subset_and_draws_are_distinct() {
        let mut rng = SplitMix64(7);
        let mut s = random_subset(&mut rng, 10_000, 100);
        s.sort_unstable();
        s.dedup();
        assert_eq!(s.len(), 100);
        let mut out = [0usize; 8];
        draw_distinct(&mut rng, 8, &mut out);
        let mut o = out.to_vec();
        o.sort_unstable();
        assert_eq!(o, (0..8).collect::<Vec<_>>());
    }

    #[test]
    fn cone_canonical_form_keeps_the_surface() {
        let c = Primitive::Cone {
            apex: Vec3::new(1.0, 2.0, 3.0),
            axis: Vec3::Z,
            half_angle: 0.4,
        };
        let flipped = Primitive::Cone {
            apex: Vec3::new(1.0, 2.0, 3.0),
            axis: -Vec3::Z,
            half_angle: std::f64::consts::PI - 0.4,
        };
        let q = Vec3::new(4.0, -1.0, 9.0);
        let canon = canonical(flipped, Vec3::ZERO);
        assert!(
            (canon.signed_distance(q).abs() - c.signed_distance(q).abs()).abs() < 1e-12,
            "{canon:?}"
        );
        assert!(
            matches!(canon, Primitive::Cone { half_angle, .. } if (half_angle - 0.4).abs() < 1e-12)
        );
    }
}
