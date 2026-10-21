//! Marching-tetrahedra case tables for the Kuhn (Freudenthal) 6-tet split of a cube.
//!
//! Every tet is the chain `0 -> e_a -> e_a + e_b -> 7` for one permutation `(a, b, c)` of the
//! axes. All six share the cube's main diagonal, and the split is translation invariant, so
//! the diagonals of two neighbouring cubes agree on their shared face. Every tet edge goes
//! from a corner `lo` to a corner `hi` with `hi ⊃ lo` bitwise, so an edge is identified by
//! its lower lattice point plus `code = hi & !lo` (1..=7).

use std::sync::LazyLock;

/// A tet edge: lower corner (3-bit offset inside the cube) and direction code (1..=7).
pub(super) type Edge = (u8, u8);

/// Triangles for one inside/outside mask of one tet (at most two).
#[derive(Clone, Copy, Default)]
pub(super) struct Case {
    pub n: u8,
    pub tris: [[Edge; 3]; 2],
}

/// Cube corner offsets of the four tet vertices, for each of the six tets.
pub(super) const TETS: [[u8; 4]; 6] = {
    let perms: [[u8; 3]; 6] = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    let mut out = [[0u8; 4]; 6];
    let mut i = 0;
    while i < 6 {
        let [a, b, _] = perms[i];
        out[i] = [0, 1 << a, (1 << a) | (1 << b), 7];
        i += 1;
    }
    out
};

/// `CASES[tet][mask]`; bit `i` of `mask` is set when tet vertex `i` is inside.
/// Triangles are counter-clockwise seen from the outside (normal points inside -> outside).
pub(super) static CASES: LazyLock<[[Case; 16]; 6]> = LazyLock::new(build);

fn edge(c: &[u8; 4], i: usize, j: usize) -> Edge {
    let (lo, hi) = (c[i].min(c[j]), c[i].max(c[j]));
    (lo, hi & !lo)
}

/// Edge midpoint in doubled integer lattice coordinates.
fn mid2(e: Edge) -> [i32; 3] {
    std::array::from_fn(|k| 2 * i32::from((e.0 >> k) & 1) + i32::from((e.1 >> k) & 1))
}

fn corner(c: u8) -> [i32; 3] {
    std::array::from_fn(|k| i32::from((c >> k) & 1))
}

#[allow(clippy::needless_range_loop)] // `mask` is a bit pattern, not just an index
fn build() -> [[Case; 16]; 6] {
    let mut out = [[Case::default(); 16]; 6];
    for (t, corners) in TETS.iter().enumerate() {
        for mask in 1..15usize {
            let inside: Vec<usize> = (0..4).filter(|i| (mask >> i) & 1 == 1).collect();
            let outside: Vec<usize> = (0..4).filter(|i| (mask >> i) & 1 == 0).collect();
            let mut raw: Vec<[Edge; 3]> = Vec::new();
            match inside.len() {
                1 => raw.push([0, 1, 2].map(|k| edge(corners, inside[0], outside[k]))),
                3 => raw.push([0, 1, 2].map(|k| edge(corners, inside[k], outside[0]))),
                _ => {
                    let (a, b, c, d) = (inside[0], inside[1], outside[0], outside[1]);
                    let (ac, ad, bd, bc) = (
                        edge(corners, a, c),
                        edge(corners, a, d),
                        edge(corners, b, d),
                        edge(corners, b, c),
                    );
                    raw.push([ac, ad, bd]);
                    raw.push([ac, bd, bc]);
                }
            }
            // Direction inside -> outside, scaled by n_in * n_out (stays integral).
            let sum = |v: &[usize]| {
                v.iter().fold([0i32; 3], |s, &i| {
                    let c = corner(corners[i]);
                    [s[0] + c[0], s[1] + c[1], s[2] + c[2]]
                })
            };
            let (si, so) = (sum(&inside), sum(&outside));
            let (ni, no) = (inside.len() as i32, outside.len() as i32);
            let dir: [i32; 3] = std::array::from_fn(|k| so[k] * ni - si[k] * no);
            let case = &mut out[t][mask];
            for tri in raw {
                let [p, q, r] = tri.map(mid2);
                let u: [i32; 3] = std::array::from_fn(|k| q[k] - p[k]);
                let v: [i32; 3] = std::array::from_fn(|k| r[k] - p[k]);
                let n = [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ];
                let d = n[0] * dir[0] + n[1] * dir[1] + n[2] * dir[2];
                assert!(d != 0, "degenerate marching-tet triangle");
                let tri = if d > 0 { tri } else { [tri[0], tri[2], tri[1]] };
                case.tris[usize::from(case.n)] = tri;
                case.n += 1;
            }
        }
    }
    out
}
