//! Shared by the MESH-18 tests and bench: the scan-like budget input.

use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use amg_mesh::{FaceGroups, IndexedMesh};

/// A scan-like 100 mm part: the S07 benchmark sphere scaled x5 (radius 50 mm, 6 % relief)
/// with every 37th face dropped (about 2.7 % of the faces: single-triangle holes all over, so
/// the mesh is open like a raw scan).
pub fn scan_part(size: BenchSize) -> IndexedMesh {
    let mut m = benchmark_sphere(size, 1);
    for p in &mut m.positions {
        *p *= 5.0;
    }
    let tris: Vec<[u32; 3]> = m
        .tris
        .iter()
        .enumerate()
        .filter(|(i, _)| i % 37 != 36)
        .map(|(_, t)| *t)
        .collect();
    m.groups = FaceGroups::new(tris.len());
    m.tris = tris;
    m.attrs = Default::default();
    m
}
