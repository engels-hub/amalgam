//! Circle and plane fits (note §2.3–2.4).

// `!(x > 0.0)` rejects NaN as well; the small dense solvers index rows and columns directly.
#![allow(clippy::neg_cmp_op_on_partial_ord, clippy::needless_range_loop)]

use amg_core::{Dir3, Mat3, Plane, Point3, Tolerance, Vec3};
use amg_mesh::{FaceId, IndexedMesh};

use super::{Circle3, CircleFit, MeasureOpts, PlaneFit};
use crate::inspect::InspectError;

/// Best-fit plane of a triangle set (note §2.3): area-weighted normal, area-weighted centroid,
/// and the largest vertex deviation. `None` if the set is empty or the normals cancel (norm of
/// the area-weighted normal < 1e-12, for example a closed body). Faces out of range are
/// skipped.
pub fn fit_plane_faces(m: &IndexedMesh, faces: &[FaceId]) -> Option<PlaneFit> {
    let mut n = Vec3::ZERO;
    let mut c = Vec3::ZERO;
    let mut area2 = 0.0;
    for f in faces.iter().filter(|f| (f.0 as usize) < m.face_count()) {
        let [a, b, p] = m.triangle(*f);
        let raw = (b - a).cross(p - a);
        let w = raw.length();
        n += raw;
        c += (a + b + p) * (w / 3.0);
        area2 += w;
    }
    let len = n.length();
    if !(len >= 1e-12) || !(area2 > 0.0) {
        return None;
    }
    let normal = Dir3::new_unchecked(n / len);
    let origin = c / area2;
    let mut max_dev = 0.0f64;
    for f in faces.iter().filter(|f| (f.0 as usize) < m.face_count()) {
        for p in m.triangle(*f) {
            max_dev = max_dev.max(normal.get().dot(p - origin).abs());
        }
    }
    Some(PlaneFit {
        plane: Plane::from_normal(origin, normal),
        max_deviation: max_dev,
    })
}

/// Circumcircle of three points (note §2.4). The normal is `(b − a) × (c − a)` normalised.
///
/// # Errors
/// [`InspectError::Degenerate`] if the points are collinear:
/// `|(b − a) × (c − a)| ≤ tol.linear · max(|b − a|, |c − a|)`.
pub fn circle_through(
    a: Point3,
    b: Point3,
    c: Point3,
    tol: &Tolerance,
) -> Result<Circle3, InspectError> {
    let u = b - a;
    let v = c - a;
    let w = u.cross(v);
    let wl = w.length();
    if !wl.is_finite() || wl <= tol.linear * u.length().max(v.length()) || wl == 0.0 {
        return Err(InspectError::Degenerate("the three points are collinear"));
    }
    let off = (v.cross(w) * u.length_squared() + w.cross(u) * v.length_squared()) / (2.0 * wl * wl);
    let center = a + off;
    Ok(Circle3 {
        center,
        normal: Dir3::new_unchecked(w / wl),
        radius: off.length(),
    })
}

/// Best-fit circle of an ordered point loop (note §2.4): PCA plane (max deviation ≤
/// `planar_tolerance`), Kåsa algebraic fit, then up to 10 Gauss–Newton steps on the geometric
/// distance. Accepted iff the RMS geometric residual ≤ `circle_tolerance` and the largest
/// residual ≤ 5 × that. The normal follows the loop by the right-hand rule. `None` for fewer
/// than 5 points or a rejected fit.
pub fn fit_circle(points: &[Point3], opts: &MeasureOpts) -> Option<CircleFit> {
    if points.len() < 5 || !points.iter().all(|p| p.is_finite()) {
        return None;
    }
    let n = points.len() as f64;
    let mean = points.iter().fold(Vec3::ZERO, |s, p| s + *p) / n;
    let mut cov = Mat3::ZERO;
    for p in points {
        let d = *p - mean;
        cov += Mat3::from_cols(d * d.x, d * d.y, d * d.z);
    }
    let (vals, vecs) = sym_eigen(cov);
    // Smallest eigenvalue → plane normal; the other two span the plane.
    let mut order = [0usize, 1, 2];
    order.sort_by(|&i, &j| vals[i].total_cmp(&vals[j]));
    let mut normal = vecs[order[0]].normalize_or_zero();
    if normal == Vec3::ZERO {
        return None;
    }
    // Orient by the loop's area vector (right-hand rule).
    let area: Vec3 = (0..points.len())
        .map(|i| (points[i] - mean).cross(points[(i + 1) % points.len()] - mean))
        .fold(Vec3::ZERO, |s, v| s + v);
    if area.dot(normal) < 0.0 {
        normal = -normal;
    }
    let max_dev = points
        .iter()
        .map(|p| normal.dot(*p - mean).abs())
        .fold(0.0, f64::max);
    if max_dev > opts.planar_tolerance {
        return None;
    }
    let nd = Dir3::new_unchecked(normal);
    let ex = nd.any_orthogonal().get();
    let ey = normal.cross(ex);
    let uv: Vec<(f64, f64)> = points
        .iter()
        .map(|p| {
            let d = *p - mean;
            (d.dot(ex), d.dot(ey))
        })
        .collect();
    // Kåsa: x² + y² + D x + E y + F = 0 in least squares.
    let mut ata = [[0.0f64; 3]; 3];
    let mut atb = [0.0f64; 3];
    for &(x, y) in &uv {
        let row = [x, y, 1.0];
        let rhs = -(x * x + y * y);
        for i in 0..3 {
            for j in 0..3 {
                ata[i][j] += row[i] * row[j];
            }
            atb[i] += row[i] * rhs;
        }
    }
    let [d, e, f] = solve3(ata, atb)?;
    let (mut cx, mut cy) = (-d / 2.0, -e / 2.0);
    let r2 = cx * cx + cy * cy - f;
    if !(r2 > 0.0) {
        return None;
    }
    let mut r = r2.sqrt();
    // Gauss–Newton on rᵢ = |pᵢ − c| − r.
    for _ in 0..10 {
        let mut jtj = [[0.0f64; 3]; 3];
        let mut jtr = [0.0f64; 3];
        for &(x, y) in &uv {
            let (dx, dy) = (x - cx, y - cy);
            let di = (dx * dx + dy * dy).sqrt();
            if di == 0.0 {
                return None;
            }
            let res = di - r;
            let jrow = [-dx / di, -dy / di, -1.0];
            for i in 0..3 {
                for j in 0..3 {
                    jtj[i][j] += jrow[i] * jrow[j];
                }
                jtr[i] -= jrow[i] * res;
            }
        }
        let Some(step) = solve3(jtj, jtr) else { break };
        cx += step[0];
        cy += step[1];
        r += step[2];
        if step.iter().map(|s| s.abs()).fold(0.0, f64::max) <= 1e-15 * (1.0 + r) {
            break;
        }
    }
    let mut sum2 = 0.0;
    let mut max_res = 0.0f64;
    let mut max_full = 0.0f64;
    let center = mean + ex * cx + ey * cy;
    for (p, &(x, y)) in points.iter().zip(&uv) {
        let res = ((x - cx).hypot(y - cy) - r).abs();
        sum2 += res * res;
        max_res = max_res.max(res);
        max_full = max_full.max(((*p - center).length() - r).abs());
    }
    let rms = (sum2 / n).sqrt();
    if !(rms <= opts.circle_tolerance) || !(max_res <= 5.0 * opts.circle_tolerance) || !(r > 0.0) {
        return None;
    }
    Some(CircleFit {
        circle: Circle3 {
            center,
            normal: nd,
            radius: r,
        },
        rms,
        max_residual: max_full.max(max_res),
    })
}

/// Solves a 3×3 system by Gaussian elimination with partial pivoting; `None` if singular.
fn solve3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    let scale = a
        .iter()
        .flatten()
        .map(|v| v.abs())
        .fold(0.0, f64::max)
        .max(f64::MIN_POSITIVE);
    for col in 0..3 {
        let piv = (col..3).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if !(a[piv][col].abs() > 1e-14 * scale) {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for row in col + 1..3 {
            let f = a[row][col] / a[col][col];
            for k in col..3 {
                a[row][k] -= f * a[col][k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0f64; 3];
    for row in (0..3).rev() {
        let s: f64 = (row + 1..3).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - s) / a[row][row];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// Eigen-decomposition of a symmetric 3×3 matrix by cyclic Jacobi rotations: eigenvalues and
/// the matching eigenvectors (columns of the accumulated rotation).
fn sym_eigen(m: Mat3) -> ([f64; 3], [Vec3; 3]) {
    let mut a = [[0.0f64; 3]; 3];
    for (j, row) in a.iter_mut().enumerate() {
        for (i, v) in row.iter_mut().enumerate() {
            *v = m.col(i)[j];
        }
    }
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0f64]];
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        let diag = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if off <= 1e-300 || off <= f64::EPSILON * 1e-3 * diag {
            break;
        }
        for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let akp = a[k][p];
                let akq = a[k][q];
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let apk = a[p][k];
                let aqk = a[q][k];
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for row in &mut v {
                let vp = row[p];
                let vq = row[q];
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    let vec = |k: usize| Vec3::new(v[0][k], v[1][k], v[2][k]);
    ([a[0][0], a[1][1], a[2][2]], [vec(0), vec(1), vec(2)])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jacobi_diagonalises() {
        let m = Mat3::from_cols(
            Vec3::new(4.0, 1.0, 0.5),
            Vec3::new(1.0, 3.0, 0.25),
            Vec3::new(0.5, 0.25, 1.0),
        );
        let (vals, vecs) = sym_eigen(m);
        for k in 0..3 {
            let r = m * vecs[k] - vecs[k] * vals[k];
            assert!(r.length() < 1e-12, "residual {r:?}");
        }
    }

    #[test]
    fn solve3_matches() {
        let x = solve3(
            [[2.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 4.0]],
            [3.0, 5.0, 5.0],
        )
        .unwrap();
        for v in x {
            assert!((v - 1.0).abs() < 1e-14);
        }
        assert!(
            solve3(
                [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [0.0, 0.0, 1.0]],
                [1.0, 2.0, 3.0]
            )
            .is_none()
        );
    }
}
