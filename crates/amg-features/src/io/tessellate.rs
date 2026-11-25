//! IO-09: solid bodies → export meshes. The tessellation cache and the deviation estimate.
//!
//! A cache entry is keyed by (body key, quality). The body key is the identity of the B-rep
//! shape ([`amg_brep::Shape::ptr_eq`]): shapes are immutable, the recompute engine hands an
//! unchanged body the same shape again, and a changed body always gets a new one. The cache
//! holds the shape, so its identity cannot be reused while the entry lives.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};

use amg_brep::{BrepKernel, KernelConfig, KernelError, OcctKernel, Shape, TessMesh, TessQuality};
use amg_core::Vec3;
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, MeshError};
use rayon::prelude::*;

/// Byte limit of [`TessCache::shared`] (meshes plus the shapes they keep alive).
const SHARED_CACHE_BYTES: u64 = 512 * 1024 * 1024;

/// Most times a body is meshed again because its estimate is above the requested deviation.
const MAX_REFINEMENTS: u32 = 3;

/// Barycentric grid steps per triangle edge for the deviation estimate.
const GRID: u32 = 6;

/// How far an export mesh is estimated to stray from the exact surface.
///
/// The estimate is second order: each triangle edge gets the sagitta of the circular arc that
/// has the two exact surface normals at its ends (exact on planes, circles, cylinders and
/// spheres), and the triangle's interior is bounded by the quadratic patch through those three
/// sagittae. It costs a few percent of the tessellation and needs no reference mesh. It is not
/// a bound: a surface that oscillates inside one triangle is not seen.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DeviationEstimate {
    /// Largest estimated distance between surface and facets, mm.
    pub surface_mm: f64,
    /// Largest angle between the surface normals at the two ends of a triangle edge, radians.
    pub normal_rad: f64,
}

/// One tessellated body as the export uses it.
#[derive(Debug)]
pub(super) struct Tessellated {
    /// Welded, indexed mesh in component space.
    pub(super) mesh: Arc<IndexedMesh>,
    pub(super) deviation: DeviationEstimate,
}

/// Counters of a [`TessCache`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TessCacheStats {
    /// Solid bodies served from the cache.
    pub hits: u64,
    /// Kernel tessellations run (successful or not), refinement passes included.
    pub tessellations: u64,
    /// Entries held now.
    pub entries: usize,
    /// Approximate bytes held now.
    pub bytes: u64,
}

struct Entry {
    shape: Shape,
    quality: TessQuality,
    result: Arc<Tessellated>,
    bytes: u64,
    last_used: u64,
}

#[derive(Default)]
struct Inner {
    entries: Vec<Entry>,
    bytes: u64,
    clock: u64,
}

/// Tessellations by (body key, quality), so a repeated export does not mesh again. Least
/// recently used entries are dropped above the byte limit. Safe to share between jobs.
pub struct TessCache {
    inner: Mutex<Inner>,
    max_bytes: u64,
    hits: AtomicU64,
    tessellations: AtomicU64,
}

impl std::fmt::Debug for TessCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TessCache")
            .field("max_bytes", &self.max_bytes)
            .field("stats", &self.stats())
            .finish()
    }
}

impl TessCache {
    /// An empty cache holding at most about `max_bytes`. A single result above the limit is
    /// returned but not kept.
    pub fn new(max_bytes: u64) -> TessCache {
        TessCache {
            inner: Mutex::default(),
            max_bytes,
            hits: AtomicU64::new(0),
            tessellations: AtomicU64::new(0),
        }
    }

    /// The process-wide cache (512 MiB) used when [`super::ExportPrep::cache`] is `None`.
    pub fn shared() -> &'static Arc<TessCache> {
        static SHARED: OnceLock<Arc<TessCache>> = OnceLock::new();
        SHARED.get_or_init(|| Arc::new(TessCache::new(SHARED_CACHE_BYTES)))
    }

    /// Hits, kernel calls and current size.
    pub fn stats(&self) -> TessCacheStats {
        let inner = self.lock();
        TessCacheStats {
            hits: self.hits.load(Ordering::Relaxed),
            tessellations: self.tessellations.load(Ordering::Relaxed),
            entries: inner.entries.len(),
            bytes: inner.bytes,
        }
    }

    /// Drops every entry (the counters stay).
    pub fn clear(&self) {
        let mut inner = self.lock();
        inner.entries.clear();
        inner.bytes = 0;
    }

    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn get(&self, shape: &Shape, quality: &TessQuality) -> Option<Arc<Tessellated>> {
        let mut inner = self.lock();
        inner.clock += 1;
        let now = inner.clock;
        let entry = inner
            .entries
            .iter_mut()
            .find(|e| e.quality == *quality && e.shape.ptr_eq(shape))?;
        entry.last_used = now;
        Some(Arc::clone(&entry.result))
    }

    fn insert(&self, shape: Shape, quality: TessQuality, result: Arc<Tessellated>) {
        let bytes = mesh_bytes(&result.mesh).saturating_add(shape.estimated_bytes());
        if bytes > self.max_bytes {
            return;
        }
        let mut inner = self.lock();
        // Another job may have filled the same key meanwhile.
        if inner
            .entries
            .iter()
            .any(|e| e.quality == quality && e.shape.ptr_eq(&shape))
        {
            return;
        }
        while inner.bytes.saturating_add(bytes) > self.max_bytes {
            let Some(oldest) = inner
                .entries
                .iter()
                .enumerate()
                .min_by_key(|(_, e)| e.last_used)
                .map(|(i, _)| i)
            else {
                break;
            };
            let dropped = inner.entries.swap_remove(oldest);
            inner.bytes -= dropped.bytes;
        }
        inner.clock += 1;
        let last_used = inner.clock;
        inner.bytes += bytes;
        inner.entries.push(Entry {
            shape,
            quality,
            result,
            bytes,
            last_used,
        });
    }
}

fn mesh_bytes(m: &IndexedMesh) -> u64 {
    // Positions, triangles and the per-face group ids.
    (m.positions.len() * size_of::<[f64; 3]>()
        + m.tris.len() * (size_of::<[u32; 3]>() + size_of::<u32>())) as u64
}

/// Why one body could not be tessellated.
#[derive(Debug)]
pub(super) enum TessFailure {
    Kernel(KernelError),
    Mesh(MeshError),
    /// The kernel meshed the shape but produced no triangle (no faces).
    NoTriangles,
}

/// One body's outcome: the result and whether it came from the cache.
pub(super) type TessOutcome = Result<(Arc<Tessellated>, bool), TessFailure>;

/// Meshes `shape` again, tighter, while the estimate of `best` is above the deviation that
/// `requested` asks for: OCCT's mesher does not hold its linear deflection everywhere (a
/// sphere at 0.005 mm strays 0.010 mm). Stops after [`MAX_REFINEMENTS`] or when a step stops
/// helping; the finest estimate wins, and the report shows one that still exceeds the request.
fn refine(
    cache: &TessCache,
    kernel: &OcctKernel,
    shape: &Shape,
    requested: &TessQuality,
    mut best: Arc<Tessellated>,
    ctx: &JobCtx,
) -> Result<Arc<Tessellated>, TessFailure> {
    let mut quality = *requested;
    for _ in 0..MAX_REFINEMENTS {
        let estimate = best.deviation.surface_mm;
        if estimate <= requested.linear {
            break;
        }
        let ratio = (0.9 * requested.linear / estimate).max(0.25);
        quality.linear *= ratio;
        // On a curve the sagitta grows with the square of the angle.
        quality.angular *= ratio.sqrt();
        cache.tessellations.fetch_add(1, Ordering::Relaxed);
        let mesh = kernel
            .tessellate(shape, &quality, ctx)
            .map_err(TessFailure::Kernel)?;
        let next = convert(&mesh)?;
        if next.deviation.surface_mm >= estimate {
            break;
        }
        best = next;
    }
    Ok(best)
}

/// Tessellates every `(shape, quality)` request, in order, through `cache`. Requests for the
/// same shape and quality are meshed once. A single miss meshes its faces in parallel;
/// several misses of one quality mesh the bodies in parallel.
pub(super) fn tessellate_all(
    cache: &TessCache,
    requests: &[(Shape, TessQuality)],
    ctx: &JobCtx,
) -> Vec<TessOutcome> {
    let mut out: Vec<Option<TessOutcome>> = requests.iter().map(|_| None).collect();
    // Distinct misses, each with the requests it answers.
    let mut misses: Vec<(Shape, TessQuality, Vec<usize>)> = Vec::new();
    for (i, (shape, quality)) in requests.iter().enumerate() {
        if let Some(found) = cache.get(shape, quality) {
            cache.hits.fetch_add(1, Ordering::Relaxed);
            out[i] = Some(Ok((found, true)));
        } else if let Some(m) = misses
            .iter_mut()
            .find(|(s, q, _)| q == quality && s.ptr_eq(shape))
        {
            m.2.push(i);
        } else {
            misses.push((shape.clone(), *quality, vec![i]));
        }
    }
    let kernel = OcctKernel::new(KernelConfig::default());
    let total = misses.len();
    let mut done = 0usize;
    while !misses.is_empty() {
        // The next batch: every miss with the quality of the first one.
        let quality = misses[0].1;
        let (batch, rest): (Vec<_>, Vec<_>) =
            misses.into_iter().partition(|(_, q, _)| *q == quality);
        misses = rest;
        #[allow(clippy::cast_precision_loss, reason = "progress needs no precision")]
        let sub = ctx.sub(
            done as f32 / total as f32,
            (done + batch.len()) as f32 / total as f32,
        );
        done += batch.len();
        cache
            .tessellations
            .fetch_add(batch.len() as u64, Ordering::Relaxed);
        let meshes = if let [(shape, _, _)] = batch.as_slice() {
            vec![kernel.tessellate(shape, &quality, &sub)]
        } else {
            let shapes: Vec<Shape> = batch.iter().map(|(s, _, _)| s.clone()).collect();
            kernel.tessellate_many(&shapes, &quality, &sub)
        };
        let converted: Vec<Result<Arc<Tessellated>, TessFailure>> = meshes
            .into_par_iter()
            .map(|m| m.map_err(TessFailure::Kernel).and_then(|m| convert(&m)))
            .collect();
        for ((shape, quality, asked), result) in batch.into_iter().zip(converted) {
            let result = result.and_then(|t| refine(cache, &kernel, &shape, &quality, t, &sub));
            match result {
                Ok(t) => {
                    cache.insert(shape, quality, Arc::clone(&t));
                    for i in asked {
                        out[i] = Some(Ok((Arc::clone(&t), false)));
                    }
                }
                Err(e) => {
                    // `TessFailure` is not `Clone`: the first request gets it, and the export
                    // stops at the first failure anyway.
                    if let Some(&i) = asked.first() {
                        out[i] = Some(Err(e));
                    }
                }
            }
        }
    }
    out.into_iter()
        .map(|o| o.unwrap_or(Err(TessFailure::NoTriangles)))
        .collect()
}

fn convert(tess: &TessMesh) -> Result<Arc<Tessellated>, TessFailure> {
    let (positions, tris, _) = tess.to_indexed_parts();
    if tris.is_empty() {
        return Err(TessFailure::NoTriangles);
    }
    let mesh = IndexedMesh::new(positions, tris).map_err(TessFailure::Mesh)?;
    Ok(Arc::new(Tessellated {
        mesh: Arc::new(mesh),
        deviation: estimate_deviation(tess),
    }))
}

/// Signed sagitta of the arc from `a` to `b` with unit surface normals `na`, `nb` (positive:
/// the surface bulges towards the normals), and the angle between the normals.
fn sagitta(chord: Vec3, na: Vec3, nb: Vec3) -> (f64, f64) {
    let cos = na.dot(nb).clamp(-1.0, 1.0);
    // (nb − na)·chord = 4R sin²(θ/2) on a circle of radius R, whatever part of the chord runs
    // along the axis of a cylinder; the sagitta is 2R sin²(θ/4).
    let cos_half = f64::midpoint(1.0, cos).sqrt();
    let cos_quarter_sq = f64::midpoint(1.0, cos_half);
    ((nb - na).dot(chord) / (8.0 * cos_quarter_sq), cos.acos())
}

/// Largest `|h|` of the quadratic patch with zero height at the corners and heights `s` at the
/// middles of the edges 01, 12 and 20.
fn patch_max(s: [f64; 3]) -> f64 {
    let step = 1.0 / f64::from(GRID);
    let mut max = 0.0f64;
    for i in 0..=GRID {
        for j in 0..=GRID - i {
            let a = f64::from(i) * step;
            let b = f64::from(j) * step;
            let c = 1.0 - a - b;
            let h = 4.0 * (a * b * s[0] + b * c * s[1] + c * a * s[2]);
            max = max.max(h.abs());
        }
    }
    max
}

/// [`DeviationEstimate`] of a tessellation, from its per-face vertex normals.
///
/// At a singular point (a sphere pole, a cone apex) one face has several nodes at one place
/// with different normals; edges ending there are left out.
fn estimate_deviation(tess: &TessMesh) -> DeviationEstimate {
    let ids = tess
        .weld
        .iter()
        .copied()
        .max()
        .map_or(0, |m| m as usize + 1);
    // Per weld id: (face + 1, first vertex seen in that face, singular in that face).
    let mut seen: Vec<(usize, u32, bool)> = vec![(0, 0, false); ids];
    let mut out = DeviationEstimate::default();
    for (face, range) in tess.face_ranges.iter().enumerate() {
        let tris = &tess.tris[range.start as usize..range.end as usize];
        for &v in tris.iter().flatten() {
            let slot = &mut seen[tess.weld[v as usize] as usize];
            if slot.0 != face + 1 {
                *slot = (face + 1, v, false);
            } else if tess.normals[v as usize].dot(tess.normals[slot.1 as usize]) < 1.0 - 1e-9 {
                slot.2 = true;
            }
        }
        for tri in tris {
            let mut s = [0.0f64; 3];
            for (k, s) in s.iter_mut().enumerate() {
                let (a, b) = (tri[k] as usize, tri[(k + 1) % 3] as usize);
                if seen[tess.weld[a] as usize].2 || seen[tess.weld[b] as usize].2 {
                    continue;
                }
                let (sag, angle) = sagitta(
                    tess.positions[b] - tess.positions[a],
                    tess.normals[a],
                    tess.normals[b],
                );
                *s = sag;
                out.normal_rad = out.normal_rad.max(angle);
            }
            out.surface_mm = out.surface_mm.max(patch_max(s));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use amg_core::Point3;

    use super::*;

    #[test]
    fn sagitta_of_a_circle_and_of_a_slanted_cylinder_chord() {
        let (r, theta) = (10.0f64, 0.4f64);
        let at = |t: f64, z: f64| Point3::new(r * t.cos(), r * t.sin(), z);
        let n = |t: f64| Vec3::new(t.cos(), t.sin(), 0.0);
        let want = r * (1.0 - (theta / 2.0).cos());
        for z in [0.0, 50.0] {
            let (s, angle) = sagitta(at(theta, z) - at(0.0, 0.0), n(0.0), n(theta));
            assert!((s - want).abs() < 1e-12, "{s} != {want}");
            assert!((angle - theta).abs() < 1e-12);
        }
        // The same arc seen from inside the material (a hole): the sign flips.
        let (s, _) = sagitta(at(theta, 0.0) - at(0.0, 0.0), -n(0.0), -n(theta));
        assert!((s + want).abs() < 1e-12);
    }

    #[test]
    fn patch_peaks_at_the_centroid_for_equal_sagittae() {
        assert!((patch_max([0.3, 0.3, 0.3]) - 0.4).abs() < 1e-12);
        assert!((patch_max([0.0, 0.2, 0.0]) - 0.2).abs() < 1e-12);
        assert!(patch_max([0.0; 3]) < 1e-300);
    }

    #[test]
    fn a_flat_mesh_has_no_deviation_and_a_singular_node_is_skipped() {
        let mut tess = TessMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            normals: vec![Vec3::Z; 3],
            tris: vec![[0, 1, 2]],
            face_ranges: std::iter::once(0..1).collect(),
            weld: vec![0, 1, 2],
            ..TessMesh::default()
        };
        assert_eq!(estimate_deviation(&tess), DeviationEstimate::default());
        // A second node at vertex 0 with another normal makes it singular.
        tess.positions.push(Point3::new(0.0, 0.0, 0.0));
        tess.positions.push(Point3::new(0.0, -1.0, 0.0));
        tess.normals.extend([Vec3::X, Vec3::Z]);
        tess.weld.extend([0, 3]);
        tess.tris.push([3, 4, 1]);
        tess.face_ranges = std::iter::once(0..2).collect();
        assert_eq!(estimate_deviation(&tess), DeviationEstimate::default());
    }
}
