//! Meshes for the MESH-09 tests and bench.
#![allow(dead_code, clippy::expect_used)] // also compiled into the bench

use std::collections::HashMap;

use amg_core::Point3;
use amg_mesh::IndexedMesh;

/// Closed box `[0, n]^3` whose six sides are `n × n` lattices of unit cells (integer
/// coordinates, `12 n²` outward triangles, alternating diagonals). Planes at integer offsets,
/// or `x + y = k`, run exactly through vertices and edges.
pub fn lattice_box(n: u32) -> IndexedMesh {
    let mut ids: HashMap<[u32; 3], u32> = HashMap::new();
    let mut positions = Vec::new();
    let mut vertex = |p: [u32; 3]| -> u32 {
        *ids.entry(p).or_insert_with(|| {
            positions.push(Point3::new(
                f64::from(p[0]),
                f64::from(p[1]),
                f64::from(p[2]),
            ));
            (positions.len() - 1) as u32
        })
    };
    let mut tris = Vec::new();
    for axis in 0..3usize {
        let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
        for side in [0, n] {
            for i in 0..n {
                for j in 0..n {
                    let corner = |di: u32, dj: u32| {
                        let mut p = [0u32; 3];
                        p[axis] = side;
                        p[u] = i + di;
                        p[v] = j + dj;
                        p
                    };
                    let mut quad = [
                        vertex(corner(0, 0)),
                        vertex(corner(1, 0)),
                        vertex(corner(1, 1)),
                        vertex(corner(0, 1)),
                    ];
                    if side == 0 {
                        quad.reverse();
                    }
                    let [q0, q1, q2, q3] = quad;
                    if (i + j) % 2 == 0 {
                        tris.push([q0, q1, q2]);
                        tris.push([q0, q2, q3]);
                    } else {
                        tris.push([q0, q1, q3]);
                        tris.push([q1, q2, q3]);
                    }
                }
            }
        }
    }
    IndexedMesh::new(positions, tris).expect("valid lattice box")
}
