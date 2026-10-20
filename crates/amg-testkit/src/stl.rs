//! Minimal binary STL, used only for golden reference meshes (`<case>.stl` next to the TOML).
//! Real import/export lives in `amg-io`.

use std::io;
use std::path::Path;

use amg_core::{Point3, Vec3};

use crate::TriSoup;
use crate::shapes::TriMesh;

const HEADER: &[u8] = b"amg-testkit golden reference (binary STL, f32, mm)";

/// Writes `m` as binary STL (f32 coordinates, so about 1e-7 relative precision).
pub(crate) fn write(path: &Path, m: TriSoup<'_>) -> io::Result<()> {
    let n = u32::try_from(m.triangles.len())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "too many triangles"))?;
    let mut buf = Vec::with_capacity(84 + 50 * m.triangles.len());
    buf.extend_from_slice(HEADER);
    buf.resize(80, b' ');
    buf.extend_from_slice(&n.to_le_bytes());
    for t in 0..m.triangles.len() {
        let [a, b, c] = m.corners(t);
        let normal = (b - a).cross(c - a).normalize_or_zero();
        for v in [normal, a, b, c] {
            for x in v.to_array() {
                #[allow(clippy::cast_possible_truncation, reason = "STL is f32 by definition")]
                buf.extend_from_slice(&(x as f32).to_le_bytes());
            }
        }
        buf.extend_from_slice(&[0, 0]);
    }
    std::fs::write(path, buf)
}

/// Reads a binary STL as an unwelded soup (3 vertices per triangle).
pub(crate) fn read(path: &Path) -> io::Result<TriMesh> {
    let bytes = std::fs::read(path)?;
    let bad = |what: &str| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("{what}: {}", path.display()),
        )
    };
    let count: [u8; 4] = bytes
        .get(80..84)
        .ok_or_else(|| bad("truncated STL header"))?
        .try_into()
        .map_err(|_| bad("STL header"))?;
    let n = u32::from_le_bytes(count) as usize;
    if bytes.len() != 84 + 50 * n {
        return Err(bad("STL size does not match its triangle count"));
    }
    let f = |o: usize| {
        f64::from(f32::from_le_bytes([
            bytes[o],
            bytes[o + 1],
            bytes[o + 2],
            bytes[o + 3],
        ]))
    };
    let mut mesh = TriMesh {
        positions: Vec::with_capacity(3 * n),
        triangles: Vec::with_capacity(n),
    };
    for t in 0..n {
        let base = 84 + 50 * t + 12; // skip the normal
        for k in 0..3 {
            let o = base + 12 * k;
            mesh.positions
                .push(Point3::from(Vec3::new(f(o), f(o + 4), f(o + 8))));
        }
        let i = u32::try_from(3 * t).map_err(|_| bad("too many triangles"))?;
        mesh.triangles.push([i, i + 1, i + 2]);
    }
    Ok(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{metrics, shapes};

    #[test]
    fn round_trip() {
        let dir = crate::testdata::tests::scratch("stl-round-trip");
        let path = dir.join("cube.stl");
        let cube = shapes::cube(Point3::new(1.0, 2.0, 3.0), 10.0);
        write(&path, cube.soup()).unwrap();
        assert_eq!(std::fs::metadata(&path).unwrap().len(), 84 + 50 * 12);
        let back = read(&path).unwrap();
        let (m0, m1) = (metrics(cube.soup()), metrics(back.soup()));
        assert_eq!(m0.bbox, m1.bbox);
        assert!((m0.volume - m1.volume).abs() < 1e-9);
        assert_eq!(back.triangles.len(), 12);

        std::fs::write(&path, b"short").unwrap();
        assert!(read(&path).is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
