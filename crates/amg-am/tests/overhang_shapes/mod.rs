//! Synthetic test shapes for the AM-02 tests and benches: unions of axis-aligned boxes on a
//! common rectilinear grid (watertight, conforming, exact coordinates) and UV spheres with a
//! latitude ring at a chosen polar angle.

#![allow(dead_code, clippy::cast_precision_loss, reason = "test helpers")]

use std::collections::BTreeMap;

use amg_core::glam::DVec3;
use amg_mesh::IndexedMesh;

/// Watertight outward-oriented mesh of the union of boxes `(min, max)`. All boxes share one
/// rectilinear grid built from their coordinates, so the result has no T-junctions.
pub fn box_union(boxes: &[(DVec3, DVec3)]) -> IndexedMesh {
    let axis = |k: usize| {
        let mut v: Vec<f64> = boxes.iter().flat_map(|(a, b)| [a[k], b[k]]).collect();
        v.sort_by(f64::total_cmp);
        v.dedup();
        v
    };
    let (xs, ys, zs) = (axis(0), axis(1), axis(2));
    let n = [xs.len() - 1, ys.len() - 1, zs.len() - 1];
    let filled = |i: isize, j: isize, k: isize| -> bool {
        if i < 0 || j < 0 || k < 0 || i >= n[0] as isize || j >= n[1] as isize || k >= n[2] as isize
        {
            return false;
        }
        let c = DVec3::new(
            0.5 * (xs[i as usize] + xs[i as usize + 1]),
            0.5 * (ys[j as usize] + ys[j as usize + 1]),
            0.5 * (zs[k as usize] + zs[k as usize + 1]),
        );
        boxes
            .iter()
            .any(|(a, b)| c.cmpgt(*a).all() && c.cmplt(*b).all())
    };
    let mut ids: BTreeMap<[usize; 3], u32> = BTreeMap::new();
    let mut pos = Vec::new();
    let mut vid = |g: [usize; 3], pos: &mut Vec<DVec3>| -> u32 {
        *ids.entry(g).or_insert_with(|| {
            pos.push(DVec3::new(xs[g[0]], ys[g[1]], zs[g[2]]));
            (pos.len() - 1) as u32
        })
    };
    let mut tris = Vec::new();
    for i in 0..n[0] {
        for j in 0..n[1] {
            for k in 0..n[2] {
                let (ii, jj, kk) = (i as isize, j as isize, k as isize);
                if !filled(ii, jj, kk) {
                    continue;
                }
                // Each face: the neighbour offset and the 4 corners in CCW order seen from outside.
                let faces: [([isize; 3], [[usize; 3]; 4]); 6] = [
                    (
                        [-1, 0, 0],
                        [[i, j, k], [i, j, k + 1], [i, j + 1, k + 1], [i, j + 1, k]],
                    ),
                    (
                        [1, 0, 0],
                        [
                            [i + 1, j, k],
                            [i + 1, j + 1, k],
                            [i + 1, j + 1, k + 1],
                            [i + 1, j, k + 1],
                        ],
                    ),
                    (
                        [0, -1, 0],
                        [[i, j, k], [i + 1, j, k], [i + 1, j, k + 1], [i, j, k + 1]],
                    ),
                    (
                        [0, 1, 0],
                        [
                            [i, j + 1, k],
                            [i, j + 1, k + 1],
                            [i + 1, j + 1, k + 1],
                            [i + 1, j + 1, k],
                        ],
                    ),
                    (
                        [0, 0, -1],
                        [[i, j, k], [i, j + 1, k], [i + 1, j + 1, k], [i + 1, j, k]],
                    ),
                    (
                        [0, 0, 1],
                        [
                            [i, j, k + 1],
                            [i + 1, j, k + 1],
                            [i + 1, j + 1, k + 1],
                            [i, j + 1, k + 1],
                        ],
                    ),
                ];
                for (o, q) in faces {
                    if filled(ii + o[0], jj + o[1], kk + o[2]) {
                        continue;
                    }
                    let v = q.map(|g| vid(g, &mut pos));
                    tris.push([v[0], v[1], v[2]]);
                    tris.push([v[0], v[2], v[3]]);
                }
            }
        }
    }
    IndexedMesh::new(pos, tris).expect("valid box union")
}

/// Axis-aligned box `[min, max]`.
pub fn cuboid(min: DVec3, max: DVec3) -> IndexedMesh {
    box_union(&[(min, max)])
}

/// T-shape of note §3.3: stem 10×10×20 on the plate, bar 30×10×5 centred on top.
pub fn t_shape() -> IndexedMesh {
    box_union(&[
        (DVec3::new(10.0, 0.0, 0.0), DVec3::new(20.0, 10.0, 20.0)),
        (DVec3::new(0.0, 0.0, 20.0), DVec3::new(30.0, 10.0, 25.0)),
    ])
}

/// Shelf of note §3.3: base 30×30×5, post 5×5×15 at one corner, top plate 30×30×3.
pub fn shelf() -> IndexedMesh {
    box_union(&[
        (DVec3::new(0.0, 0.0, 0.0), DVec3::new(30.0, 30.0, 5.0)),
        (DVec3::new(0.0, 0.0, 5.0), DVec3::new(5.0, 5.0, 20.0)),
        (DVec3::new(0.0, 0.0, 20.0), DVec3::new(30.0, 30.0, 23.0)),
    ])
}

/// UV sphere of radius `r` centred at the origin with poles on ±Z, `n_lat` latitude bands
/// and `2·n_lat` longitudes. With `n_lat` divisible by 4 a ring lies exactly at 45° from the
/// south pole, so the 45° cap boundary is a ring.
pub fn uv_sphere(r: f64, n_lat: usize) -> IndexedMesh {
    let n_lon = 2 * n_lat;
    let mut pos = vec![DVec3::new(0.0, 0.0, -r)];
    for k in 1..n_lat {
        let th = std::f64::consts::PI * k as f64 / n_lat as f64; // from the south pole
        let (s, c) = th.sin_cos();
        for l in 0..n_lon {
            let ph = std::f64::consts::TAU * l as f64 / n_lon as f64;
            pos.push(DVec3::new(r * s * ph.cos(), r * s * ph.sin(), -r * c));
        }
    }
    pos.push(DVec3::new(0.0, 0.0, r));
    let north = (pos.len() - 1) as u32;
    let ring = |k: usize, l: usize| (1 + (k - 1) * n_lon + l % n_lon) as u32;
    let mut tris = Vec::new();
    for l in 0..n_lon {
        // South fan: seen from below (outside), CCW is clockwise seen from above.
        tris.push([0, ring(1, l + 1), ring(1, l)]);
        tris.push([north, ring(n_lat - 1, l), ring(n_lat - 1, l + 1)]);
    }
    for k in 1..n_lat - 1 {
        for l in 0..n_lon {
            let (a, b) = (ring(k, l), ring(k, l + 1));
            let (c, d) = (ring(k + 1, l + 1), ring(k + 1, l));
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }
    IndexedMesh::new(pos, tris).expect("valid sphere")
}
