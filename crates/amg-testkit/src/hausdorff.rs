//! Sampled Hausdorff distance between two triangle soups.
//!
//! Surface points are drawn area-weighted with a fixed seed: sample `i` comes from a
//! SplitMix64 stream seeded with `SEED + i`, so results do not depend on thread count or
//! scheduling. Each sample's nearest-triangle distance is found through [`Bvh`].

use std::sync::atomic::{AtomicU64, Ordering};

use amg_core::Point3;
use rayon::prelude::*;

use crate::TriSoup;
use crate::bvh::Bvh;

const SEED: u64 = 0x416D_616C_6761_6D21; // "Amalgam!"

/// Symmetric Hausdorff distance in mm, estimated from every referenced vertex plus `samples`
/// area-weighted surface points on each soup (fixed seed, so the result is reproducible bit
/// for bit).
///
/// This is a lower bound of the true distance that converges as `samples` grows; 100 k
/// samples per mesh resolve features down to about `sqrt(area / 100 000)`, and the vertex
/// queries catch corners and spikes exactly. `samples == 0` measures vertices only.
///
/// Empty and zero-area soups: a direction *from* a soup without triangles contributes 0, and
/// a direction *to* a soup without triangles is `+inf`. So two empty soups give 0 and exactly
/// one empty soup gives `+inf`. A zero-area soup (all triangles degenerate) still has
/// vertices: they are measured from, and they are valid targets (distance to a degenerate
/// triangle is distance to its nearest edge).
///
/// # Panics
/// If a triangle index is out of range.
pub fn hausdorff(a: TriSoup<'_>, b: TriSoup<'_>, samples: usize) -> f64 {
    let (ba, bb) = rayon::join(|| Bvh::new(a), || Bvh::new(b));
    let (ab, ba) = rayon::join(|| directed(a, &bb, samples), || directed(b, &ba, samples));
    ab.max(ba)
}

/// Directed Hausdorff distance `max over p on from (min over q on to |p - q|)`, mm, from
/// `samples` area-weighted points on `from`. See [`hausdorff`] for the conventions.
///
/// # Panics
/// If a triangle index is out of range.
pub fn one_sided_hausdorff(from: TriSoup<'_>, to: TriSoup<'_>, samples: usize) -> f64 {
    directed(from, &Bvh::new(to), samples)
}

fn directed(from: TriSoup<'_>, to: &Bvh<'_>, samples: usize) -> f64 {
    if from.triangles.is_empty() {
        return 0.0;
    }
    // Running max of squared distances, as f64 bits (monotone for non-negative values). It is
    // only an early-out bound for the BVH: every returned value is either exact or `<=` the
    // max at that moment, so the final max is exact and independent of scheduling.
    let max = AtomicU64::new(0.0f64.to_bits());
    let query = |p: Point3| {
        let floor = f64::from_bits(max.load(Ordering::Relaxed));
        let d2 = to.closest_dist2_until(p, floor);
        max.fetch_max(d2.to_bits(), Ordering::Relaxed);
    };
    // Every referenced vertex: the samples would miss isolated corners and spikes.
    let mut used = vec![false; from.positions.len()];
    for &i in from.triangles.iter().flatten() {
        used[i as usize] = true;
    }
    (0..from.positions.len())
        .into_par_iter()
        .filter(|&i| used[i])
        .for_each(|i| query(from.positions[i]));
    let sampler = Sampler::new(from);
    // A zero-area soup has no surface to sample: its vertices are all there is to measure.
    if sampler.total > 0.0 {
        (0..samples)
            .into_par_iter()
            .for_each(|i| query(sampler.sample(i as u64)));
    }
    f64::from_bits(max.into_inner()).sqrt()
}

/// Area-weighted surface sampler.
struct Sampler<'a> {
    soup: TriSoup<'a>,
    /// Inclusive prefix sums of triangle areas.
    cdf: Vec<f64>,
    total: f64,
}

impl<'a> Sampler<'a> {
    fn new(soup: TriSoup<'a>) -> Sampler<'a> {
        let mut acc = 0.0;
        let cdf: Vec<f64> = (0..soup.triangles.len())
            .map(|t| {
                let [a, b, c] = soup.corners(t);
                acc += 0.5 * (b - a).cross(c - a).length();
                acc
            })
            .collect();
        Sampler {
            soup,
            cdf,
            total: acc,
        }
    }

    fn sample(&self, i: u64) -> Point3 {
        let mut rng = SplitMix64::new(SEED.wrapping_add(i));
        let x = rng.unit() * self.total;
        let t = self
            .cdf
            .partition_point(|&c| c <= x)
            .min(self.cdf.len() - 1);
        let [a, b, c] = self.soup.corners(t);
        // Uniform point in a triangle (Osada et al. 2002).
        let r1 = rng.unit().sqrt();
        let r2 = rng.unit();
        a * (1.0 - r1) + b * (r1 * (1.0 - r2)) + c * (r1 * r2)
    }
}

/// SplitMix64 (Steele, Lea & Flood 2014): tiny, fast and good enough for sampling.
#[derive(Clone, Debug)]
pub(crate) struct SplitMix64(u64);

impl SplitMix64 {
    pub(crate) fn new(seed: u64) -> SplitMix64 {
        SplitMix64(seed)
    }

    pub(crate) fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)` with 53 random bits.
    pub(crate) fn unit(&mut self) -> f64 {
        #[allow(clippy::cast_precision_loss, reason = "53-bit value is exact in f64")]
        let v = (self.next_u64() >> 11) as f64;
        v * (1.0 / (1u64 << 53) as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes;

    #[test]
    fn concentric_spheres() {
        let inner = shapes::icosphere(Point3::ZERO, 10.0, 5);
        let outer = shapes::icosphere(Point3::ZERO, 10.5, 5);
        let h = hausdorff(inner.soup(), outer.soup(), 20_000);
        assert!((h - 0.5).abs() <= 0.01, "hausdorff {h}");
        let sym = hausdorff(outer.soup(), inner.soup(), 20_000);
        assert_eq!(h.to_bits(), sym.to_bits(), "symmetric and deterministic");
    }

    #[test]
    fn identical_meshes_are_zero() {
        let s = shapes::icosphere(Point3::ZERO, 5.0, 3);
        assert!(hausdorff(s.soup(), s.soup(), 5_000) < 1e-12);
    }

    #[test]
    fn deterministic_across_runs() {
        let a = shapes::icosphere(Point3::ZERO, 3.0, 3);
        let b = shapes::cube(Point3::splat(-2.0), 4.0);
        let h1 = hausdorff(a.soup(), b.soup(), 10_000);
        let h2 = hausdorff(a.soup(), b.soup(), 10_000);
        assert_eq!(h1.to_bits(), h2.to_bits());
    }

    #[test]
    fn one_sided_sees_only_one_direction() {
        // A small cube inside a big one: every point of the small cube is >= 2 mm from the
        // big cube's surface, but the big cube's corners are much further from the small one.
        let small = shapes::cube(Point3::splat(-1.0), 2.0);
        let big = shapes::cube(Point3::splat(-3.0), 6.0);
        let s2b = one_sided_hausdorff(small.soup(), big.soup(), 20_000);
        let b2s = one_sided_hausdorff(big.soup(), small.soup(), 20_000);
        assert!((s2b - 2.0).abs() < 1e-9, "{s2b}");
        assert!(b2s > 3.0, "{b2s}");
        let h = hausdorff(small.soup(), big.soup(), 20_000);
        assert_eq!(h.to_bits(), s2b.max(b2s).to_bits());
    }

    #[test]
    fn empty_inputs() {
        let s = shapes::cube(Point3::ZERO, 1.0);
        let e = TriSoup {
            positions: &[],
            triangles: &[],
        };
        assert_eq!(hausdorff(e, e, 100).to_bits(), 0.0f64.to_bits());
        assert!(hausdorff(s.soup(), e, 100).is_infinite());
        assert_eq!(hausdorff(s.soup(), s.soup(), 0).to_bits(), 0.0f64.to_bits());
        assert!(hausdorff(e, s.soup(), 100).is_infinite(), "symmetric");
    }

    #[test]
    fn early_out_gives_the_exact_max() {
        let a = shapes::icosphere(Point3::new(0.3, 0.0, 0.0), 4.0, 3);
        let b = shapes::cube(Point3::splat(-3.0), 6.0);
        let bvh = Bvh::new(b.soup());
        let sampler = Sampler::new(a.soup());
        let exact = a
            .positions
            .iter()
            .copied()
            .chain((0..2000).map(|i| sampler.sample(i)))
            .map(|p| bvh.closest_dist2(p))
            .fold(0.0, f64::max)
            .sqrt();
        let fast = one_sided_hausdorff(a.soup(), b.soup(), 2000);
        assert_eq!(fast.to_bits(), exact.to_bits(), "{fast} vs {exact}");
    }

    #[test]
    fn vertices_catch_spikes_that_samples_miss() {
        // A cube with one corner pulled out 5 mm along the diagonal: the spike has little
        // area, but its tip vertex is exactly 5 mm from the original cube's corner region.
        let base = shapes::cube(Point3::ZERO, 10.0);
        let mut spiky = base.clone();
        let dir = amg_core::Vec3::splat(1.0).normalize();
        spiky.positions[7] += 5.0 * dir; // vertex 7 is (10, 10, 10)
        let h = one_sided_hausdorff(spiky.soup(), base.soup(), 0);
        assert!((h - 5.0).abs() < 1e-12, "vertex-only query: {h}");
        let h = one_sided_hausdorff(spiky.soup(), base.soup(), 3);
        assert!(
            (h - 5.0).abs() < 1e-12,
            "few samples, still exact at the tip: {h}"
        );
    }

    #[test]
    fn zero_area_soup_is_measured_by_its_vertices() {
        // A degenerate triangle on a line 2 mm above a unit square.
        let pos = [
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(1.0, 0.0, 2.0),
            Point3::new(0.5, 0.0, 2.0),
        ];
        let flat = TriSoup {
            positions: &pos,
            triangles: &[[0, 1, 2]],
        };
        let square = shapes::cube(Point3::new(0.0, 0.0, -1.0), 1.0);
        let h = one_sided_hausdorff(flat, square.soup(), 1000);
        assert!((h - 2.0).abs() < 1e-12, "{h}");
        // As a target, the degenerate triangle is its segment.
        let back = one_sided_hausdorff(flat, flat, 10);
        assert!(back < 1e-12, "{back}");
    }

    #[test]
    fn samples_are_on_the_surface_and_area_weighted() {
        // Two triangles, one 3x the area of the other: about 3/4 of the samples land in it.
        let pos = [
            Point3::ZERO,
            Point3::X,
            Point3::Y,
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(13.0, 0.0, 0.0),
            Point3::new(10.0, 1.0, 0.0),
        ];
        let soup = TriSoup {
            positions: &pos,
            triangles: &[[0, 1, 2], [3, 4, 5]],
        };
        let s = Sampler::new(soup);
        let n = 20_000u64;
        let big = (0..n).filter(|&i| s.sample(i).x >= 10.0).count();
        #[allow(clippy::cast_precision_loss, reason = "small counts")]
        let frac = big as f64 / n as f64;
        assert!((frac - 0.75).abs() < 0.02, "{frac}");
        for i in 0..1000 {
            let p = s.sample(i);
            assert!(p.z.abs() < 1e-15 && p.x >= 0.0 && p.y >= 0.0 && p.x + p.y <= 13.0);
        }
    }

    #[test]
    fn splitmix_unit_range() {
        let mut r = SplitMix64::new(1);
        let mut sum = 0.0;
        for _ in 0..10_000 {
            let u = r.unit();
            assert!((0.0..1.0).contains(&u));
            sum += u;
        }
        assert!((sum / 10_000.0 - 0.5).abs() < 0.02);
    }
}
