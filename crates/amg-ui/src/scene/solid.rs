//! Solid bodies (UI-22): BREP-08 tessellation → display mesh (per-face triangle ranges, so the
//! face sub-id is the B-rep face index) and edge polylines (edge sub-id = B-rep edge index).
//!
//! Only `amg_brep`'s public tessellation API is used; no OCCT type is visible here. Without the
//! `occt` feature no real [`amg_brep::Shape`] can exist, and a placeholder solid has no
//! geometry: [`SolidTessellator`] handles neither, so such bodies show nothing (the adapter
//! logs that once).

use std::sync::OnceLock;

use amg_brep::{BrepKernel, KernelConfig, KernelError, OcctKernel, TessMesh, TessQuality};
use amg_core::{Aabb3, Cancelled, Point3};
use amg_doc::BodyGeom;
use amg_jobs::JobCtx;
use amg_render::{EdgeData, MeshData, Polyline};

use super::tess::{BodyTessellator, DisplayGeometry, DisplayMesh, DisplayQuality, TessellateError};

/// The BREP-08 preset of a display quality: `Preview` → [`TessQuality::preview`], `Normal` →
/// [`TessQuality::display`], `Fine` → [`TessQuality::export`].
pub fn quality_preset(quality: DisplayQuality) -> TessQuality {
    match quality {
        DisplayQuality::Preview => TessQuality::preview(),
        DisplayQuality::Normal => TessQuality::display(),
        DisplayQuality::Fine => TessQuality::export(),
    }
}

fn kernel() -> &'static OcctKernel {
    static KERNEL: OnceLock<OcctKernel> = OnceLock::new();
    KERNEL.get_or_init(|| OcctKernel::new(KernelConfig::default()))
}

/// Solid (and surface) bodies: [`BrepKernel::tessellate`] at the quality's preset, rebased to
/// the centre of the bounds (RTE, S10 note §3).
#[derive(Debug, Default)]
pub struct SolidTessellator;

impl BodyTessellator for SolidTessellator {
    fn handles(&self, geom: &BodyGeom) -> bool {
        matches!(geom, BodyGeom::Solid(s) if s.brep().is_some())
    }

    fn depends_on_quality(&self) -> bool {
        true
    }

    fn tessellate(
        &self,
        geom: &BodyGeom,
        quality: DisplayQuality,
        ctx: &JobCtx,
    ) -> Result<DisplayMesh, TessellateError> {
        self.tessellate_display(geom, quality, ctx).map(|g| g.mesh)
    }

    fn tessellate_display(
        &self,
        geom: &BodyGeom,
        quality: DisplayQuality,
        ctx: &JobCtx,
    ) -> Result<DisplayGeometry, TessellateError> {
        let shape = match geom {
            BodyGeom::Solid(s) => s.brep(),
            _ => None,
        };
        let Some(shape) = shape else {
            return Err(TessellateError::Failed(
                "not a solid body with a B-rep shape".into(),
            ));
        };
        let tess = kernel()
            .tessellate(shape, &quality_preset(quality), ctx)
            .map_err(|e| match e {
                KernelError::Cancelled => TessellateError::Cancelled(Cancelled),
                other => TessellateError::Failed(other.to_string()),
            })?;
        ctx.checkpoint()?;
        display_geometry(tess, ctx)
    }
}

/// Converts a [`TessMesh`] to upload data.
fn display_geometry(tess: TessMesh, ctx: &JobCtx) -> Result<DisplayGeometry, TessellateError> {
    let bounds = Aabb3::from_points(tess.positions.iter().chain(&tess.edge_points).copied());
    let origin = if bounds.is_empty() {
        Point3::ZERO
    } else {
        bounds.center()
    };
    let local = |p: &Point3| (*p - origin).as_vec3().to_array();
    let closed = is_closed_oriented(&tess);
    ctx.checkpoint()?;
    let positions = tess.positions.iter().map(local).collect();
    let normals = tess
        .normals
        .iter()
        .map(|n| n.as_vec3().to_array())
        .collect();
    let indices = tess.tris.iter().flatten().copied().collect();
    ctx.checkpoint()?;
    // Degenerated edges (a sphere's poles) have no points: nothing to draw or pick.
    let polylines = tess
        .edges
        .iter()
        .filter(|e| e.points.len() >= 2)
        .map(|e| Polyline {
            first: e.points.start,
            len: e.points.end - e.points.start,
            id: e.edge.0,
            closed: false,
        })
        .collect();
    let edges = EdgeData {
        origin,
        points: tess.edge_points.iter().map(local).collect(),
        polylines,
    };
    ctx.checkpoint()?;
    Ok(DisplayGeometry {
        mesh: DisplayMesh {
            data: MeshData {
                origin,
                positions,
                normals,
                indices,
                closed,
            },
            face_ranges: tess.face_ranges,
        },
        edges: Some(edges),
    })
}

/// Whether the welded triangles close up with consistent orientation: every undirected edge is
/// used by exactly two triangles, once in each direction (RND-06 caps only such meshes).
/// Triangles that collapse under the weld (sphere poles) are ignored.
fn is_closed_oriented(tess: &TessMesh) -> bool {
    let mut uses: Vec<(u64, i8)> = Vec::with_capacity(tess.tris.len() * 3);
    for tri in &tess.tris {
        let [a, b, c] = tri.map(|v| tess.weld[v as usize]);
        if a == b || b == c || a == c {
            continue;
        }
        for (u, v) in [(a, b), (b, c), (c, a)] {
            let key = (u64::from(u.min(v)) << 32) | u64::from(u.max(v));
            uses.push((key, if u < v { 1 } else { -1 }));
        }
    }
    uses.sort_unstable();
    !uses.is_empty()
        && uses
            .chunk_by(|x, y| x.0 == y.0)
            .all(|c| c.len() == 2 && c[0].1 + c[1].1 == 0)
}

#[cfg(test)]
mod tests {
    use amg_brep::{EdgeId, EdgePolyline};
    use amg_doc::stub::Shape;

    use super::*;

    /// A tetrahedron as four faces with per-face vertices and one real and one degenerated edge.
    fn tetrahedron() -> TessMesh {
        let p = [
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(12.0, 0.0, 0.0),
            Point3::new(10.0, 2.0, 0.0),
            Point3::new(10.0, 0.0, 2.0),
        ];
        let faces: [[u32; 3]; 4] = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
        let weld: Vec<u32> = faces.iter().flatten().copied().collect();
        TessMesh {
            positions: weld.iter().map(|&i| p[i as usize]).collect(),
            normals: vec![amg_core::Vec3::Z; 12],
            tris: (0..4).map(|f| [3 * f, 3 * f + 1, 3 * f + 2]).collect(),
            face_ranges: (0..4).map(|f| f..f + 1).collect(),
            weld,
            edges: vec![
                EdgePolyline {
                    edge: EdgeId(0),
                    points: 0..0,
                },
                EdgePolyline {
                    edge: EdgeId(1),
                    points: 0..2,
                },
            ],
            edge_points: vec![p[0], p[1]],
            uvs: None,
        }
    }

    #[test]
    fn a_tess_mesh_becomes_rebased_faces_and_edge_polylines() {
        let g = display_geometry(tetrahedron(), &JobCtx::detached()).unwrap();
        assert_eq!(g.mesh.face_count(), 4);
        assert_eq!(g.mesh.triangle_count(), 4);
        assert!(g.mesh.data.closed);
        assert_eq!(g.mesh.data.origin, Point3::new(11.0, 1.0, 1.0));
        assert_eq!(
            g.mesh.data.positions[0].map(f32::to_bits),
            [-1.0f32; 3].map(f32::to_bits)
        );
        let edges = g.edges.unwrap();
        assert_eq!(edges.origin, g.mesh.data.origin);
        // The degenerated edge is left out; ids stay the B-rep edge indices.
        assert_eq!(
            edges.polylines,
            vec![Polyline {
                first: 0,
                len: 2,
                id: 1,
                closed: false
            }]
        );
        assert_eq!(
            edges.points[1].map(f32::to_bits),
            [1.0f32, -1.0, -1.0].map(f32::to_bits)
        );
    }

    #[test]
    fn open_or_inconsistently_oriented_meshes_are_not_closed() {
        assert!(is_closed_oriented(&tetrahedron()));
        let mut open = tetrahedron();
        open.tris.pop();
        assert!(!is_closed_oriented(&open));
        let mut flipped = tetrahedron();
        flipped.tris[0].swap(1, 2);
        assert!(!is_closed_oriented(&flipped));
        assert!(!is_closed_oriented(&TessMesh::default()));
    }

    #[test]
    fn presets_and_placeholders() {
        assert_eq!(
            quality_preset(DisplayQuality::Preview),
            TessQuality::preview()
        );
        assert_eq!(
            quality_preset(DisplayQuality::Normal),
            TessQuality::display()
        );
        assert_eq!(quality_preset(DisplayQuality::Fine), TessQuality::export());
        assert!(!SolidTessellator.handles(&BodyGeom::Solid(Shape::new())));
    }
}
