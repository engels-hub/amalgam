//! Polygon triangulation for OBJ faces (note §6): ear clipping in the plane of the Newell
//! normal, with a fan fallback for degenerate polygons.

use amg_core::Point3;

/// Longest polygon that is ear-clipped (O(n²)); longer ones are fanned.
pub(crate) const MAX_EAR_CLIP: usize = 4096;

/// How a polygon was triangulated.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Method {
    /// Ear clipping produced every triangle.
    EarClip,
    /// A fan from the first vertex (degenerate, self-intersecting or too long).
    Fan,
}

/// Newell normal of a polygon (not normalised; its length is twice the area).
pub(crate) fn newell(pts: &[Point3]) -> [f64; 3] {
    let mut n = [0.0f64; 3];
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        n[0] += (a.y - b.y) * (a.z + b.z);
        n[1] += (a.z - b.z) * (a.x + b.x);
        n[2] += (a.x - b.x) * (a.y + b.y);
    }
    n
}

/// Triangulates the polygon `pts` (at least 3 corners, in order). Writes exactly
/// `pts.len() - 2` triples of corner positions (indices into `pts`) to `out`, wound like the
/// polygon.
pub(crate) fn triangulate(pts: &[Point3], out: &mut Vec<[u32; 3]>) -> Method {
    let n = pts.len();
    debug_assert!(n >= 3);
    let start = out.len();
    if n == 3 {
        out.push([0, 1, 2]);
        return Method::EarClip;
    }
    if n <= MAX_EAR_CLIP && ear_clip(pts, out) {
        return Method::EarClip;
    }
    out.truncate(start);
    for i in 1..n as u32 - 1 {
        out.push([0, i, i + 1]);
    }
    Method::Fan
}

/// Project onto the plane orthogonal to the dominant axis of the Newell normal, flipped so
/// that the polygon is counter-clockwise in the projection. `None` if the polygon has no
/// usable normal.
fn project(pts: &[Point3]) -> Option<Vec<[f64; 2]>> {
    let nrm = newell(pts);
    let (ax, ay, az) = (nrm[0].abs(), nrm[1].abs(), nrm[2].abs());
    let big = ax.max(ay).max(az);
    if big.is_nan() || big <= 0.0 || !big.is_finite() {
        return None;
    }
    // (u, v, w) is a right-handed frame whose w is the dominant axis; the polygon is CCW in
    // (u, v) when the normal's w-component is positive.
    let (u, v, w) = if az >= ax && az >= ay {
        (0, 1, 2)
    } else if ax >= ay {
        (1, 2, 0)
    } else {
        (2, 0, 1)
    };
    let flip = nrm[w] < 0.0;
    Some(
        pts.iter()
            .map(|p| {
                let a = p.to_array();
                if flip { [a[v], a[u]] } else { [a[u], a[v]] }
            })
            .collect(),
    )
}

/// Bit-identical points (duplicated corners of a polygon).
#[inline]
fn same(a: [f64; 2], b: [f64; 2]) -> bool {
    a.map(f64::to_bits) == b.map(f64::to_bits)
}

#[inline]
fn cross(o: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

/// `p` inside or on the boundary of the CCW triangle `a b c`.
#[inline]
fn in_triangle(a: [f64; 2], b: [f64; 2], c: [f64; 2], p: [f64; 2]) -> bool {
    cross(a, b, p) >= 0.0 && cross(b, c, p) >= 0.0 && cross(c, a, p) >= 0.0
}

/// Ear clipping. Returns `false` (and the caller falls back) when no ear can be found.
fn ear_clip(pts: &[Point3], out: &mut Vec<[u32; 3]>) -> bool {
    let Some(p2) = project(pts) else {
        return false;
    };
    let n = pts.len();
    // Doubly linked ring of the remaining corners.
    let mut next: Vec<u32> = (1..=n as u32).map(|i| i % n as u32).collect();
    let mut prev: Vec<u32> = (0..n as u32)
        .map(|i| (i + n as u32 - 1) % n as u32)
        .collect();
    let mut remaining = n;
    let mut cur = 0usize;
    // Number of consecutive vertices tested without clipping.
    let mut stuck = 0usize;
    // First pass accepts only strictly convex ears; once a whole lap fails, collinear
    // (zero-area) corners may be clipped too.
    let mut allow_flat = false;
    while remaining > 3 {
        let (pv, nx) = (prev[cur] as usize, next[cur] as usize);
        let (a, b, c) = (p2[pv], p2[cur], p2[nx]);
        let turn = cross(a, b, c);
        let convex = if allow_flat { turn >= 0.0 } else { turn > 0.0 };
        let mut ear = convex;
        if ear && turn > 0.0 {
            // No other remaining corner may lie inside the candidate ear.
            let mut k = next[nx] as usize;
            while k != pv {
                let q = p2[k];
                // Corners coincident with an ear corner are harmless (duplicated vertices).
                if ![a, b, c].iter().any(|v| same(*v, q)) && in_triangle(a, b, c, q) {
                    ear = false;
                    break;
                }
                k = next[k] as usize;
            }
        }
        if ear {
            out.push([pv as u32, cur as u32, nx as u32]);
            next[pv] = nx as u32;
            prev[nx] = pv as u32;
            remaining -= 1;
            cur = nx;
            stuck = 0;
        } else {
            cur = nx;
            stuck += 1;
            if stuck >= remaining {
                if allow_flat {
                    return false;
                }
                allow_flat = true;
                stuck = 0;
            }
        }
    }
    let (pv, nx) = (prev[cur] as usize, next[cur] as usize);
    out.push([pv as u32, cur as u32, nx as u32]);
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn area2d(p: &[Point3], t: &[u32; 3], nrm: [f64; 3]) -> f64 {
        // Signed area along the polygon normal.
        let (a, b, c) = (p[t[0] as usize], p[t[1] as usize], p[t[2] as usize]);
        let n = (b - a).cross(c - a);
        0.5 * (n.x * nrm[0] + n.y * nrm[1] + n.z * nrm[2])
            / (nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]).sqrt()
    }

    /// Star-shaped (hence simple, usually concave) polygon about the origin: one corner per
    /// angular sector (so no gap reaches half a turn) with a random radius, placed in an
    /// arbitrary plane and orientation.
    fn star() -> impl Strategy<Value = Vec<Point3>> {
        (
            prop::collection::vec((0.05f64..0.95, 0.2f64..2.0), 4..40),
            -1.0f64..1.0,
            -1.0f64..1.0,
            0.2f64..1.0,
            -50.0f64..50.0,
            any::<bool>(),
        )
            .prop_map(|(v, nx, ny, nz, off, reverse)| {
                let n = v.len() as f64;
                let nrm = amg_core::Vec3::new(nx, ny, nz).normalize_or(amg_core::Vec3::Z);
                let u = nrm.any_orthonormal_vector();
                let w = nrm.cross(u);
                let mut pts: Vec<Point3> = v
                    .iter()
                    .enumerate()
                    .map(|(i, (j, r))| {
                        let a = (i as f64 + j) / n * std::f64::consts::TAU;
                        Point3::splat(off) + u * (r * a.cos()) + w * (r * a.sin())
                    })
                    .collect();
                if reverse {
                    pts.reverse();
                }
                pts
            })
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(1000))]

        /// Done-when 3: area preserved within 1e-12 relative, no flipped triangle, n-2
        /// triangles, every corner used.
        #[test]
        fn ear_clipping_preserves_area_and_orientation(pts in star()) {
            let nrm = newell(&pts);
            let total = 0.5 * (nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]).sqrt();
            let mut tris = Vec::new();
            let m = triangulate(&pts, &mut tris);
            prop_assert_eq!(m, Method::EarClip);
            prop_assert_eq!(tris.len(), pts.len() - 2);
            let mut sum = 0.0;
            for t in &tris {
                let a = area2d(&pts, t, nrm);
                prop_assert!(a >= -1e-15 * total, "flipped triangle {t:?}: {a}");
                sum += a;
            }
            prop_assert!(((sum - total) / total).abs() < 1e-12, "{sum} vs {total}");
            let mut used = vec![false; pts.len()];
            for t in &tris { for &i in t { used[i as usize] = true; } }
            prop_assert!(used.iter().all(|&u| u));
        }
    }

    fn p(x: f64, y: f64) -> Point3 {
        Point3::new(x, y, 0.0)
    }

    #[test]
    fn concave_arrow_is_not_fanned_across_the_notch() {
        // A concave "chevron": the fan from vertex 0 would leave the polygon.
        let pts = [p(0., 0.), p(4., 0.), p(4., 4.), p(2., 1.), p(0., 4.)];
        let mut tris = Vec::new();
        assert_eq!(triangulate(&pts, &mut tris), Method::EarClip);
        let nrm = newell(&pts);
        assert!(tris.iter().all(|t| area2d(&pts, t, nrm) >= 0.0));
        let sum: f64 = tris.iter().map(|t| area2d(&pts, t, nrm)).sum();
        assert!((sum - 10.0).abs() < 1e-12);
    }

    #[test]
    fn degenerate_polygon_falls_back_to_a_fan() {
        let pts = [p(0., 0.), p(1., 0.), p(2., 0.), p(3., 0.)];
        let mut tris = Vec::new();
        assert_eq!(triangulate(&pts, &mut tris), Method::Fan);
        assert_eq!(tris, vec![[0, 1, 2], [0, 2, 3]]);
    }

    #[test]
    fn convex_quad_and_reversed_winding() {
        let pts = [p(0., 0.), p(1., 0.), p(1., 1.), p(0., 1.)];
        let mut tris = Vec::new();
        assert_eq!(triangulate(&pts, &mut tris), Method::EarClip);
        assert_eq!(tris.len(), 2);
        let rev: Vec<Point3> = pts.iter().rev().copied().collect();
        let mut t2 = Vec::new();
        assert_eq!(triangulate(&rev, &mut t2), Method::EarClip);
        let nrm = newell(&rev);
        assert!(t2.iter().all(|t| area2d(&rev, t, nrm) > 0.0));
    }

    #[test]
    fn collinear_corner_gives_a_zero_area_triangle_not_a_failure() {
        let pts = [p(0., 0.), p(1., 0.), p(2., 0.), p(2., 2.), p(0., 2.)];
        let mut tris = Vec::new();
        assert_eq!(triangulate(&pts, &mut tris), Method::EarClip);
        assert_eq!(tris.len(), 3);
        let nrm = newell(&pts);
        let sum: f64 = tris.iter().map(|t| area2d(&pts, t, nrm)).sum();
        assert!((sum - 4.0).abs() < 1e-12);
    }
}
