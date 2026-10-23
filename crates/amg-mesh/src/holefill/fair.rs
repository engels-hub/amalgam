//! Harmonic and bi-harmonic fairing of the new patch vertices (Kobbelt's discrete fairing with
//! scale-dependent umbrella weights).
//!
//! With the umbrella operator `U` (rows normalised, weights `1 / edge length` from the patch as
//! refined), the free vertices `F` minimise
//! - harmonic: `sum_{u in F} |U x_u|^2` (so `U x = 0` on `F`: a discrete minimal-like surface),
//! - bi-harmonic: `sum_{u in F and the loop} |U x_u|^2` where the loop rows use the full ring of
//!   the surrounding mesh (so the patch continues the surface tangentially).
//!
//! Both are linear least-squares problems solved through the normal equations with a sparse
//! Cholesky factorisation (`faer`), falling back to conjugate gradients if that fails.

use amg_core::Point3;
use faer::linalg::solvers::Solve;
use faer::sparse::{SparseColMat, SymbolicSparseColMat};
use faer::{Mat, Side};
use rayon::prelude::*;

use super::Fairing;
use super::refine::Patch;

/// Ring of a loop vertex in the surrounding mesh, beyond the patch.
#[derive(Default, Clone, Debug)]
pub(super) struct Ring {
    /// Neighbours that are themselves loop vertices (local index).
    pub loop_nb: Vec<u32>,
    /// Positions of the other neighbours.
    pub fixed: Vec<Point3>,
}

/// More free vertices than this: bi-harmonic is downgraded to harmonic.
const BIHARMONIC_MAX_FREE: usize = 60_000;
/// More free vertices than this: no fairing.
const FAIR_MAX_FREE: usize = 600_000;

struct Row {
    cols: Vec<(u32, f64)>,
    /// Constant part of the residual `A x + c`.
    c: Point3,
}

/// Moves the free (non-loop) vertices of `patch` to fair positions.
pub(super) fn fair(patch: &mut Patch, rings: &[Ring], mode: Fairing) {
    let n = patch.n_loop;
    let m = patch.pos.len() - n;
    if mode == Fairing::None || m == 0 || m > FAIR_MAX_FREE {
        return;
    }
    let biharmonic = mode == Fairing::Biharmonic && m <= BIHARMONIC_MAX_FREE;
    let total = patch.pos.len();
    let mut adj: Vec<Vec<u32>> = vec![Vec::new(); total];
    for t in &patch.tris {
        for k in 0..3 {
            adj[t[k] as usize].push(t[(k + 1) % 3]);
            adj[t[(k + 1) % 3] as usize].push(t[k]);
        }
    }
    for a in &mut adj {
        a.sort_unstable();
        a.dedup();
    }
    let rows_of: Vec<usize> = if biharmonic {
        (0..total).collect()
    } else {
        (n..total).collect()
    };
    let pos = &patch.pos;
    let rows: Vec<Row> = rows_of
        .par_iter()
        .filter_map(|&u| {
            let mut locals: Vec<u32> = adj[u].clone();
            let mut fixed: &[Point3] = &[];
            if u < n {
                locals.extend(rings[u].loop_nb.iter().copied());
                locals.sort_unstable();
                locals.dedup();
                fixed = &rings[u].fixed;
            }
            let pu = pos[u];
            let wt = |q: Point3| 1.0 / (q - pu).length().max(1e-300);
            let total_w: f64 = locals.iter().map(|&j| wt(pos[j as usize])).sum::<f64>()
                + fixed.iter().map(|&q| wt(q)).sum::<f64>();
            if !(total_w > 0.0 && total_w.is_finite()) {
                return None;
            }
            let mut cols = Vec::with_capacity(locals.len() + 1);
            let mut c = Point3::ZERO;
            for &j in &locals {
                let wj = wt(pos[j as usize]) / total_w;
                if (j as usize) >= n {
                    cols.push((j - n as u32, wj));
                } else {
                    c += wj * pos[j as usize];
                }
            }
            for &q in fixed {
                c += wt(q) / total_w * q;
            }
            if u >= n {
                cols.push((u as u32 - n as u32, -1.0));
            } else {
                c -= pu;
            }
            if cols.is_empty() {
                return None;
            }
            Some(Row { cols, c })
        })
        .collect();

    let x0: Vec<Point3> = pos[n..].to_vec();
    let solved = solve_cholesky(m, &rows).or_else(|| solve_cg(m, &rows, &x0));
    if let Some(x) = solved
        && x.iter().all(|p| p.is_finite())
    {
        patch.pos[n..].copy_from_slice(&x);
    }
}

/// Normal equations `A^T A x = -A^T c` (lower triangle assembled column by column) and a sparse
/// Cholesky solve.
fn solve_cholesky(m: usize, rows: &[Row]) -> Option<Vec<Point3>> {
    // A in column-compressed form.
    let mut start = vec![0usize; m + 1];
    for r in rows {
        for &(c, _) in &r.cols {
            start[c as usize + 1] += 1;
        }
    }
    for i in 0..m {
        start[i + 1] += start[i];
    }
    let mut fill = start.clone();
    let mut a_row = vec![0u32; start[m]];
    let mut a_val = vec![0.0f64; start[m]];
    for (ri, r) in rows.iter().enumerate() {
        for &(c, v) in &r.cols {
            let p = fill[c as usize];
            fill[c as usize] += 1;
            a_row[p] = ri as u32;
            a_val[p] = v;
        }
    }
    let mut rhs = vec![[0.0f64; 3]; m];
    for (i, out) in rhs.iter_mut().enumerate() {
        for p in start[i]..start[i + 1] {
            let (r, v) = (&rows[a_row[p] as usize], a_val[p]);
            out[0] -= v * r.c.x;
            out[1] -= v * r.c.y;
            out[2] -= v * r.c.z;
        }
    }
    // Lower triangle of A^T A with a sparse accumulator per column.
    let mut col_ptr = Vec::with_capacity(m + 1);
    col_ptr.push(0usize);
    let mut row_idx: Vec<usize> = Vec::with_capacity(m * 16);
    let mut values: Vec<f64> = Vec::with_capacity(m * 16);
    let mut acc = vec![0.0f64; m];
    let mut mark = vec![usize::MAX; m];
    let mut touched: Vec<usize> = Vec::new();
    for i in 0..m {
        touched.clear();
        for p in start[i]..start[i + 1] {
            let (r, vi) = (&rows[a_row[p] as usize], a_val[p]);
            for &(j, vj) in &r.cols {
                let j = j as usize;
                if j < i {
                    continue;
                }
                if mark[j] != i {
                    mark[j] = i;
                    acc[j] = 0.0;
                    touched.push(j);
                }
                acc[j] += vi * vj;
            }
        }
        touched.sort_unstable();
        for &j in &touched {
            row_idx.push(j);
            values.push(acc[j]);
        }
        col_ptr.push(row_idx.len());
    }
    let symbolic = SymbolicSparseColMat::new_checked(m, m, col_ptr, None, row_idx);
    let a = SparseColMat::<usize, f64>::new(symbolic, values);
    let llt = a.sp_cholesky(Side::Lower).ok()?;
    let mut b = Mat::<f64>::zeros(m, 3);
    for (i, r) in rhs.iter().enumerate() {
        for (k, &v) in r.iter().enumerate() {
            b[(i, k)] = v;
        }
    }
    llt.solve_in_place(b.as_mut());
    Some(
        (0..m)
            .map(|i| Point3::new(b[(i, 0)], b[(i, 1)], b[(i, 2)]))
            .collect(),
    )
}

/// Conjugate gradients on the normal equations (CGLS), matrix-free over the rows.
fn solve_cg(m: usize, rows: &[Row], x0: &[Point3]) -> Option<Vec<Point3>> {
    let mut out = vec![Point3::ZERO; m];
    for axis in 0..3 {
        let comp = |p: Point3| p[axis];
        let mut x: Vec<f64> = x0.iter().map(|p| comp(*p)).collect();
        let apply = |x: &[f64], r: &mut [f64]| {
            for (ri, row) in rows.iter().enumerate() {
                r[ri] = row
                    .cols
                    .iter()
                    .map(|&(c, v)| v * x[c as usize])
                    .sum::<f64>()
                    + comp(row.c);
            }
        };
        let apply_t = |r: &[f64], s: &mut [f64]| {
            s.fill(0.0);
            for (ri, row) in rows.iter().enumerate() {
                for &(c, v) in &row.cols {
                    s[c as usize] += v * r[ri];
                }
            }
        };
        let mut r = vec![0.0; rows.len()];
        apply(&x, &mut r);
        r.iter_mut().for_each(|v| *v = -*v);
        let mut s = vec![0.0; m];
        apply_t(&r, &mut s);
        let mut p = s.clone();
        let mut gamma: f64 = s.iter().map(|v| v * v).sum();
        let gamma0 = gamma;
        let mut q = vec![0.0; rows.len()];
        for _ in 0..(20 * m + 200).min(20_000) {
            if gamma <= 1e-24 * gamma0 || gamma == 0.0 {
                break;
            }
            // q = A p without the constant part.
            for (ri, row) in rows.iter().enumerate() {
                q[ri] = row
                    .cols
                    .iter()
                    .map(|&(c, v)| v * p[c as usize])
                    .sum::<f64>();
            }
            let qq: f64 = q.iter().map(|v| v * v).sum();
            if qq == 0.0 {
                break;
            }
            let alpha = gamma / qq;
            for i in 0..m {
                x[i] += alpha * p[i];
            }
            for (ri, v) in q.iter().enumerate() {
                r[ri] -= alpha * v;
            }
            apply_t(&r, &mut s);
            let g2: f64 = s.iter().map(|v| v * v).sum();
            let beta = g2 / gamma;
            gamma = g2;
            for i in 0..m {
                p[i] = s[i] + beta * p[i];
            }
        }
        for i in 0..m {
            out[i][axis] = x[i];
        }
    }
    Some(out)
}
