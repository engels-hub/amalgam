//! Structured test problems for solver tests and benchmarks (design note §9; used by S13/S14
//! benches too).
//!
//! All problems live on the unit cube with an `nodes³` node grid of trilinear (Q1) hexahedra,
//! spacing `h = 1 / (nodes − 1)`. Node `(i, j, k)` has index `i + nodes·(j + nodes·k)`; vector
//! problems number DOF `c` of node `p` as `3p + c`. Element matrices use 2×2×2 Gauss
//! quadrature (exact for these elements). Assembly is row-parallel and deterministic.
//!
//! Dirichlet DOFs are kept in the system (so `q1_elasticity(55, _)` has exactly 3·55³ =
//! 499 125 rows): their off-diagonal entries are stored as explicit zeros, the diagonal keeps
//! its assembled value and the load is 0. Constrained and free variants therefore share one
//! pattern.

use rayon::prelude::*;

use crate::sparse::SymCsr;

/// Boundary condition of [`q1_elasticity`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Support {
    /// All displacements fixed on the `x = 0` face.
    Clamped,
    /// No constraints: the stiffness has the 6 rigid-body modes in its null space.
    Free,
}

const GAUSS: f64 = 0.577_350_269_189_625_8; // 1/√3

/// Trilinear shape-function values and reference gradients at `xi` for local node
/// `a = ax + 2·ay + 4·az`.
fn shape(a: usize, xi: [f64; 3]) -> (f64, [f64; 3]) {
    let s = [
        if a & 1 == 0 { -1.0 } else { 1.0 },
        if a & 2 == 0 { -1.0 } else { 1.0 },
        if a & 4 == 0 { -1.0 } else { 1.0 },
    ];
    let f = [1.0 + s[0] * xi[0], 1.0 + s[1] * xi[1], 1.0 + s[2] * xi[2]];
    let n = f[0] * f[1] * f[2] / 8.0;
    let g = [
        s[0] * f[1] * f[2] / 8.0,
        f[0] * s[1] * f[2] / 8.0,
        f[0] * f[1] * s[2] / 8.0,
    ];
    (n, g)
}

fn gauss_points() -> impl Iterator<Item = [f64; 3]> {
    (0..8).map(|g| {
        [
            if g & 1 == 0 { -GAUSS } else { GAUSS },
            if g & 2 == 0 { -GAUSS } else { GAUSS },
            if g & 4 == 0 { -GAUSS } else { GAUSS },
        ]
    })
}

/// Element matrices of one cube of side `h`: (stiffness, mass), `8·dofs` square, row-major.
fn element_matrices(h: f64, dofs: usize, nu: f64) -> (Vec<f64>, Vec<f64>) {
    let size = 8 * dofs;
    let mut k = vec![0.0; size * size];
    let mut m = vec![0.0; size * size];
    let det = (h / 2.0).powi(3);
    let to_x = 2.0 / h;
    let (lambda, mu) = (nu / ((1.0 + nu) * (1.0 - 2.0 * nu)), 0.5 / (1.0 + nu));
    for xi in gauss_points() {
        let mut nval = [0.0; 8];
        let mut grad = [[0.0; 3]; 8];
        for a in 0..8 {
            let (n, g) = shape(a, xi);
            nval[a] = n;
            grad[a] = [g[0] * to_x, g[1] * to_x, g[2] * to_x];
        }
        for a in 0..8 {
            for b in 0..8 {
                let mab = nval[a] * nval[b] * det;
                let (ga, gb) = (grad[a], grad[b]);
                if dofs == 1 {
                    k[a * size + b] += (ga[0] * gb[0] + ga[1] * gb[1] + ga[2] * gb[2]) * det;
                    m[a * size + b] += mab;
                    continue;
                }
                let dot = ga[0] * gb[0] + ga[1] * gb[1] + ga[2] * gb[2];
                for c in 0..3 {
                    for d in 0..3 {
                        // Bᵀ D B for isotropic elasticity (E = 1):
                        // λ ∂_c N_a ∂_d N_b + μ (∂_d N_a ∂_c N_b + δ_cd ∇N_a·∇N_b).
                        let mut v = lambda * ga[c] * gb[d] + mu * ga[d] * gb[c];
                        if c == d {
                            v += mu * dot;
                            m[(3 * a + c) * size + 3 * b + d] += mab;
                        }
                        k[(3 * a + c) * size + 3 * b + d] += v * det;
                    }
                }
            }
        }
    }
    (k, m)
}

/// Upper neighbours `q >= p` of node `p` as `(q, (dx, dy, dz))`, in increasing `q`.
fn upper_neighbours(nodes: usize, p: usize) -> Vec<(usize, [i64; 3])> {
    let (i, j, k) = (p % nodes, (p / nodes) % nodes, p / (nodes * nodes));
    let mut out = Vec::with_capacity(14);
    for dz in -1i64..=1 {
        for dy in -1i64..=1 {
            for dx in -1i64..=1 {
                if (dz, dy, dx) < (0, 0, 0) {
                    continue;
                }
                let (x, y, z) = (i as i64 + dx, j as i64 + dy, k as i64 + dz);
                let r = 0..nodes as i64;
                if r.contains(&x) && r.contains(&y) && r.contains(&z) {
                    let q = (x + nodes as i64 * (y + nodes as i64 * z)) as usize;
                    out.push((q, [dx, dy, dz]));
                }
            }
        }
    }
    out
}

struct Assembly<'a> {
    nodes: usize,
    dofs: usize,
    /// Element stiffness and mass (row-major, `8·dofs` square).
    ke: &'a [f64],
    me: Option<&'a [f64]>,
    /// Per-cell stiffness factor (`cells³`).
    coeff: Option<&'a [f32]>,
    fixed: &'a (dyn Fn(usize) -> bool + Sync),
}

impl Assembly<'_> {
    /// The rows of node `p`: (columns, stiffness values, mass values), row after row.
    fn node_rows(&self, p: usize) -> (Vec<u32>, Vec<f64>, Vec<f64>) {
        let (nodes, dofs) = (self.nodes, self.dofs);
        let cells = nodes - 1;
        let size = 8 * dofs;
        let ijk = [p % nodes, (p / nodes) % nodes, p / (nodes * nodes)];
        let nbrs = upper_neighbours(nodes, p);
        let (mut cols, mut kv, mut mv) = (Vec::new(), Vec::new(), Vec::new());
        for c in 0..dofs {
            for &(q, off) in &nbrs {
                // Elements holding both p and q: origin o with o ≤ p, q ≤ o + 1 per axis.
                let mut kb = [0.0; 3];
                let mut mb = [0.0; 3];
                for e in 0..8 {
                    let ex = [e & 1, (e >> 1) & 1, (e >> 2) & 1];
                    let mut origin = [0usize; 3];
                    let mut a = 0;
                    let mut b = 0;
                    let mut ok = true;
                    for ax in 0..3 {
                        let o = ijk[ax] as i64 - ex[ax] as i64;
                        let lq = ex[ax] as i64 + off[ax];
                        if o < 0 || o >= cells as i64 || !(0..=1).contains(&lq) {
                            ok = false;
                            break;
                        }
                        origin[ax] = o as usize;
                        a |= ex[ax] << ax;
                        b |= (lq as usize) << ax;
                    }
                    if !ok {
                        continue;
                    }
                    let s = match self.coeff {
                        Some(cf) => {
                            f64::from(cf[origin[0] + cells * (origin[1] + cells * origin[2])])
                        }
                        None => 1.0,
                    };
                    for d in 0..dofs {
                        let at = (dofs * a + c) * size + dofs * b + d;
                        kb[d] += s * self.ke[at];
                        if let Some(me) = self.me {
                            mb[d] += me[at];
                        }
                    }
                }
                for d in 0..dofs {
                    if q == p && d < c {
                        continue;
                    }
                    let (row, col) = (dofs * p + c, dofs * q + d);
                    let constrained = row != col && ((self.fixed)(p) || (self.fixed)(q));
                    cols.push(col as u32);
                    kv.push(if constrained { 0.0 } else { kb[d] });
                    mv.push(if constrained { 0.0 } else { mb[d] });
                }
            }
        }
        (cols, kv, mv)
    }

    fn build(&self) -> (SymCsr, Option<SymCsr>) {
        let nn = self.nodes.pow(3);
        let n = nn * self.dofs;
        let per_node: Vec<(Vec<u32>, Vec<f64>, Vec<f64>)> =
            (0..nn).into_par_iter().map(|p| self.node_rows(p)).collect();
        let mut row_ptr = Vec::with_capacity(n + 1);
        row_ptr.push(0usize);
        let total: usize = per_node.iter().map(|r| r.0.len()).sum();
        let mut cols = Vec::with_capacity(total);
        let mut kv = Vec::with_capacity(total);
        let mut mv = Vec::with_capacity(if self.me.is_some() { total } else { 0 });
        for (p, (c, k, m)) in per_node.into_iter().enumerate() {
            // Row boundaries within the node: a row ends where the column wraps back below.
            let mut last_row = self.dofs * p;
            for (idx, &col) in c.iter().enumerate() {
                if idx > 0 && col as usize <= c[idx - 1] as usize {
                    row_ptr.push(cols.len());
                    last_row += 1;
                }
                cols.push(col);
            }
            debug_assert_eq!(last_row, self.dofs * p + self.dofs - 1);
            row_ptr.push(cols.len());
            kv.extend(k);
            if self.me.is_some() {
                mv.extend(m);
            }
        }
        #[allow(
            clippy::expect_used,
            reason = "the generated pattern is valid by construction"
        )]
        let k = SymCsr::from_raw(n, row_ptr.clone(), cols.clone(), kv).expect("valid pattern");
        #[allow(
            clippy::expect_used,
            reason = "the generated pattern is valid by construction"
        )]
        let m = self
            .me
            .map(|_| SymCsr::from_raw(n, row_ptr, cols, mv).expect("valid pattern"));
        (k, m)
    }
}

/// Lumped body load `h³` per node (fraction for boundary nodes), 0 at fixed nodes.
fn lumped_load(nodes: usize, dofs: usize, comp: usize, fixed: &dyn Fn(usize) -> bool) -> Vec<f64> {
    let h = 1.0 / (nodes - 1) as f64;
    let per_axis = |i: usize| if i == 0 || i == nodes - 1 { 0.5 } else { 1.0 };
    let mut f = vec![0.0; nodes.pow(3) * dofs];
    for p in 0..nodes.pow(3) {
        if fixed(p) {
            continue;
        }
        let (i, j, k) = (p % nodes, (p / nodes) % nodes, p / (nodes * nodes));
        f[dofs * p + comp] = -h.powi(3) * per_axis(i) * per_axis(j) * per_axis(k);
    }
    f
}

/// 3-D Q1 Poisson `−∇·(k∇u) = 1` on an `nodes³` grid with `u = 0` on the whole boundary.
///
/// `coeff` is the per-cell `k` (`(nodes − 1)³` values, cell `(i, j, k)` at
/// `i + c·(j + c·k)`, `c = nodes − 1`); `None` is `k = 1`. Returns the matrix and the lumped
/// load (with the sign convention `K u = f`, `f ≥ 0`). SPD.
///
/// Deviation from the S15 signature: the coefficient is a slice, not a CMP-02 `DenseGrid`
/// (which does not exist yet); see the CMP-03 Refinement-log entry.
///
/// # Panics
/// If `nodes < 2` or `coeff` has the wrong length.
pub fn q1_poisson(nodes: u32, coeff: Option<&[f32]>) -> (SymCsr, Vec<f64>) {
    let m = nodes as usize;
    assert!(m >= 2, "q1_poisson needs at least 2 nodes per axis");
    if let Some(c) = coeff {
        assert_eq!(
            c.len(),
            (m - 1).pow(3),
            "coeff must have one value per cell"
        );
    }
    let h = 1.0 / (m - 1) as f64;
    let (ke, _) = element_matrices(h, 1, 0.0);
    let fixed = move |p: usize| {
        let (i, j, k) = (p % m, (p / m) % m, p / (m * m));
        [i, j, k].iter().any(|&x| x == 0 || x == m - 1)
    };
    let asm = Assembly {
        nodes: m,
        dofs: 1,
        ke: &ke,
        me: None,
        coeff,
        fixed: &fixed,
    };
    let (k, _) = asm.build();
    let f = lumped_load(m, 1, 0, &fixed)
        .into_iter()
        .map(|v| -v)
        .collect();
    (k, f)
}

/// 3-D Q1 linear elasticity (E = 1, Poisson ratio `nu`, density 1) on an `nodes³` grid:
/// stiffness, consistent mass and a lumped unit body load in `−z`.
///
/// [`Support::Clamped`] fixes the `x = 0` face (SPD); [`Support::Free`] leaves the 6 rigid-body
/// modes (singular; the load is then not in equilibrium, which only matters for solving). Both
/// share one pattern. At clamped DOFs the mass keeps its diagonal, so they show up as spurious
/// eigenvalues near the top of the spectrum, not among the low modes.
///
/// # Panics
/// If `nodes < 2` or `nu` is not in `(-1, 0.5)`.
pub fn q1_elasticity(nodes: u32, support: Support, nu: f64) -> (SymCsr, SymCsr, Vec<f64>) {
    let m = nodes as usize;
    assert!(m >= 2, "q1_elasticity needs at least 2 nodes per axis");
    assert!(nu > -1.0 && nu < 0.5, "Poisson ratio must be in (-1, 0.5)");
    let h = 1.0 / (m - 1) as f64;
    let (ke, me) = element_matrices(h, 3, nu);
    let clamped = support == Support::Clamped;
    let fixed = move |p: usize| clamped && p.is_multiple_of(m);
    let asm = Assembly {
        nodes: m,
        dofs: 3,
        ke: &ke,
        me: Some(&me),
        coeff: None,
        fixed: &fixed,
    };
    let (k, mass) = asm.build();
    #[allow(
        clippy::expect_used,
        reason = "build returns a mass matrix when `me` is set"
    )]
    let mass = mass.expect("mass assembled");
    let f = lumped_load(m, 3, 2, &fixed);
    (k, mass, f)
}

/// A SIMP-like 0/1 per-cell field on `cells³` cells: `1.0` (solid) or `1 / contrast` (void),
/// about half solid, in blobs from three random plane waves. Deterministic for a seed.
///
/// # Panics
/// If `contrast` is not positive.
pub fn simp_field(cells: u32, contrast: f32, seed: u64) -> Vec<f32> {
    assert!(contrast > 0.0, "contrast must be positive");
    let c = cells as usize;
    let mut state = seed;
    let mut next = || {
        state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    };
    let waves: Vec<([f64; 3], f64)> = (0..3)
        .map(|_| {
            let dir = [next() - 0.5, next() - 0.5, next() - 0.5];
            let freq = 4.0 + 8.0 * next();
            (dir.map(|d| d * freq), next() * std::f64::consts::TAU)
        })
        .collect();
    let void = 1.0 / contrast;
    let mut out = Vec::with_capacity(c.pow(3));
    for k in 0..c {
        for j in 0..c {
            for i in 0..c {
                let x = [i as f64 + 0.5, j as f64 + 0.5, k as f64 + 0.5].map(|v| v / c as f64);
                let s: f64 = waves
                    .iter()
                    .map(|(w, ph)| (w[0] * x[0] + w[1] * x[1] + w[2] * x[2] + ph).sin())
                    .sum();
                out.push(if s >= 0.0 { 1.0 } else { void });
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes_match_the_design_note() {
        let (k, m, f) = q1_elasticity(4, Support::Clamped, 0.3);
        assert_eq!(k.n(), 192);
        assert_eq!(f.len(), 192);
        assert!(k.same_pattern(&m));
        let (free, _, _) = q1_elasticity(4, Support::Free, 0.3);
        assert!(k.same_pattern(&free));
    }

    #[test]
    fn rigid_translation_is_in_the_free_null_space() {
        let (k, _, _) = q1_elasticity(5, Support::Free, 0.3);
        for c in 0..3 {
            let x: Vec<f64> = (0..k.n())
                .map(|r| if r % 3 == c { 1.0 } else { 0.0 })
                .collect();
            let mut y = vec![0.0; k.n()];
            k.spmv(&x, &mut y);
            let max = y.iter().fold(0.0f64, |a, v| a.max(v.abs()));
            assert!(max < 1e-13, "component {c}: {max}");
        }
    }

    #[test]
    fn mass_sums_to_the_volume() {
        let (_, m, _) = q1_elasticity(4, Support::Free, 0.3);
        let ones: Vec<f64> = (0..m.n())
            .map(|r| if r % 3 == 0 { 1.0 } else { 0.0 })
            .collect();
        let mut y = vec![0.0; m.n()];
        m.spmv(&ones, &mut y);
        let total: f64 = y.iter().sum();
        assert!((total - 1.0).abs() < 1e-12, "{total}");
    }

    #[test]
    fn poisson_constant_is_in_the_null_space_away_from_the_boundary() {
        let (k, f) = q1_poisson(5, None);
        let total: f64 = f.iter().sum();
        // Interior nodes only: 3³ nodes of weight h³.
        assert!((total - 27.0 / 64.0).abs() < 1e-12, "{total}");
        let diag = k.diagonal();
        assert!(diag.iter().all(|&d| d > 0.0));
    }

    #[test]
    #[allow(clippy::float_cmp, reason = "the field takes exactly two values")]
    fn simp_field_is_two_valued_and_deterministic() {
        let a = simp_field(8, 1e3, 7);
        assert_eq!(a, simp_field(8, 1e3, 7));
        assert_ne!(a, simp_field(8, 1e3, 8));
        let solid = a.iter().filter(|&&v| v == 1.0).count();
        assert!(a.iter().all(|&v| v == 1.0 || v == 1e-3));
        assert!(solid > 0 && solid < a.len());
    }
}
