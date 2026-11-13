//! Tessellation output (S02 §8, BREP-08): [`TessMesh`] and its welded, indexed view.

use std::ops::Range;

use amg_core::{Point2, Point3, Vec3};
use amg_jobs::JobCtx;

use crate::error::{KResult, KernelError};
use crate::opts::TessQuality;
use crate::shape::{EdgeId, FaceId, Shape};

/// One edge's polyline: `points` is a range into [`TessMesh::edge_points`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EdgePolyline {
    /// The edge.
    pub edge: EdgeId,
    /// Its points, in the edge's parameter order. Empty for degenerated edges.
    pub points: Range<u32>,
}

/// A B-rep tessellation in f64 and mm.
///
/// Guarantees (S02 §8, S10 request):
/// - vertices are per face: no vertex is shared between two faces, so normals split at face
///   boundaries;
/// - `face_ranges` has one entry per face in index order (`face_ranges[i]` belongs to
///   `FaceId(i)`), and the ranges tile `0..tris.len()` in order;
/// - `edges` has one entry per edge in index order (`edges[i].edge == EdgeId(i)`);
/// - `weld` is exact: nodes on a shared edge come from that edge's single discretisation, so
///   vertices that are the same B-rep point get the same id without a distance search.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TessMesh {
    /// Per-face vertices, mm.
    pub positions: Vec<Point3>,
    /// Unit normals facing out of the material, one per vertex.
    pub normals: Vec<Vec3>,
    /// Triangles, counter-clockwise seen from outside.
    pub tris: Vec<[u32; 3]>,
    /// Triangle range per face (`len == face count`).
    pub face_ranges: Vec<Range<u32>>,
    /// Canonical shared-vertex id per vertex.
    pub weld: Vec<u32>,
    /// One polyline per edge.
    pub edges: Vec<EdgePolyline>,
    /// Points of all edge polylines, mm.
    pub edge_points: Vec<Point3>,
    /// Texture coordinates per vertex (P2; always `None` for now).
    pub uvs: Option<Vec<Point2>>,
}

impl TessMesh {
    /// The face that triangle `tri` belongs to (binary search on `face_ranges`).
    ///
    /// # Panics
    /// If `tri` is not a triangle of this mesh.
    pub fn face_of_triangle(&self, tri: u32) -> FaceId {
        assert!(
            (tri as usize) < self.tris.len(),
            "triangle {tri} is out of range: the mesh has {} triangles",
            self.tris.len()
        );
        // The first face whose range ends after `tri`; empty ranges before it are skipped.
        let face = self.face_ranges.partition_point(|r| r.end <= tri);
        FaceId(u32::try_from(face).unwrap_or(u32::MAX))
    }

    /// The welded mesh: `(positions, tris, face id per tri)`.
    ///
    /// Vertices with the same [`TessMesh::weld`] id become one (numbered in order of first
    /// use), and triangles that collapse under the weld (two corners on one degenerated edge,
    /// such as a sphere pole) are dropped. For a closed solid the result is watertight: every
    /// edge has exactly two triangles. These are the core fields of `amg_mesh::IndexedMesh`
    /// plus the per-triangle face id for its `FaceGroups` (S07).
    pub fn to_indexed_parts(&self) -> (Vec<Point3>, Vec<[u32; 3]>, Vec<u32>) {
        let ids = self
            .weld
            .iter()
            .copied()
            .max()
            .map_or(0, |m| m as usize + 1);
        let mut remap = vec![u32::MAX; ids];
        let mut positions = Vec::with_capacity(self.positions.len());
        let mut tris = Vec::with_capacity(self.tris.len());
        let mut faces = Vec::with_capacity(self.tris.len());
        for (f, range) in self.face_ranges.iter().enumerate() {
            let face = u32::try_from(f).unwrap_or(u32::MAX);
            for tri in &self.tris[range.start as usize..range.end as usize] {
                let welded = tri.map(|v| self.weld[v as usize]);
                if welded[0] == welded[1] || welded[1] == welded[2] || welded[0] == welded[2] {
                    continue;
                }
                let mut out = [0u32; 3];
                for (k, &v) in tri.iter().enumerate() {
                    let slot = &mut remap[welded[k] as usize];
                    if *slot == u32::MAX {
                        *slot = u32::try_from(positions.len()).unwrap_or(u32::MAX);
                        positions.push(self.positions[v as usize]);
                    }
                    out[k] = *slot;
                }
                tris.push(out);
                faces.push(face);
            }
        }
        (positions, tris, faces)
    }

    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.tris.len()
    }

    /// Approximate heap footprint in bytes (for caches).
    pub fn estimated_bytes(&self) -> u64 {
        use std::mem::size_of;
        let bytes = self.positions.capacity() * size_of::<Point3>()
            + self.normals.capacity() * size_of::<Vec3>()
            + self.tris.capacity() * size_of::<[u32; 3]>()
            + self.face_ranges.capacity() * size_of::<Range<u32>>()
            + self.weld.capacity() * size_of::<u32>()
            + self.edges.capacity() * size_of::<EdgePolyline>()
            + self.edge_points.capacity() * size_of::<Point3>()
            + self
                .uvs
                .as_ref()
                .map_or(0, |uvs| uvs.capacity() * size_of::<Point2>());
        bytes as u64
    }
}

/// How one backend `tessellate` call meshes: absolute values, resolved from a [`TessQuality`].
#[derive(Clone, Copy, Debug)]
#[cfg_attr(
    not(feature = "occt"),
    allow(dead_code, reason = "only the OCCT backend reads it")
)]
pub(crate) struct TessParams {
    /// Linear deflection, mm.
    pub(crate) linear: f64,
    /// Angular deflection, rad.
    pub(crate) angular: f64,
    /// Minimum element size, mm; 0 = OCCT default.
    pub(crate) min_size: f64,
    /// OCCT meshes the faces of this one shape in parallel.
    pub(crate) in_parallel: bool,
    /// Report OCCT's progress to the context (else only cancellation is asked).
    pub(crate) report_progress: bool,
}

/// Where the time of one `tessellate` call went. From `OcctKernel::tessellate_profiled`.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TessProfile {
    /// The absolute linear deflection used, mm.
    pub deflection: f64,
    /// `BRepBuilderAPI_Copy` of the topology plus its index maps, s.
    pub copy_seconds: f64,
    /// `BRepMesh_IncrementalMesh`, s.
    pub mesh_seconds: f64,
    /// Copying the result out of OCCT, s.
    pub extract_seconds: f64,
    /// Edge discretisations that differed between two faces (0 for a watertight weld).
    pub weld_mismatches: u32,
}

/// The absolute linear deflection (mm) of `q` for a shape whose bbox diagonal is `diagonal`.
///
/// # Errors
/// `InvalidInput` when a deflection is not positive and finite, or a relative quality meets an
/// empty or infinite bbox.
pub(crate) fn absolute_deflection(q: &TessQuality, diagonal: f64) -> KResult<f64> {
    let invalid = |reason: String| KernelError::InvalidInput {
        what: "TessQuality",
        reason,
        offending: Vec::new(),
    };
    if !(q.linear > 0.0 && q.linear.is_finite()) {
        return Err(invalid(format!(
            "linear deflection {} must be positive",
            q.linear
        )));
    }
    if !(q.angular > 0.0 && q.angular.is_finite()) {
        return Err(invalid(format!(
            "angular deflection {} must be positive",
            q.angular
        )));
    }
    if !(q.min_size >= 0.0 && q.min_size.is_finite()) {
        return Err(invalid(format!("min_size {} must be >= 0", q.min_size)));
    }
    if !q.relative {
        return Ok(q.linear);
    }
    if !(diagonal > 0.0 && diagonal.is_finite()) {
        return Err(invalid(format!(
            "relative deflection needs a finite, non-empty bbox (diagonal {diagonal})"
        )));
    }
    let mut linear = q.linear * diagonal;
    if let Some(max) = q.max_linear_abs {
        if !(max > 0.0 && max.is_finite()) {
            return Err(invalid(format!("max_linear_abs {max} must be positive")));
        }
        linear = linear.min(max);
    }
    Ok(linear)
}

/// `tessellate` of one shape (S02 §8): resolves `q` against the shape's bbox, meshes a private
/// topology copy and copies the result out.
pub(crate) fn tessellate(
    s: &Shape,
    q: &TessQuality,
    ctx: &JobCtx,
    in_parallel: bool,
    report_progress: bool,
) -> KResult<(TessMesh, TessProfile)> {
    ctx.checkpoint()?;
    let linear = absolute_deflection(q, s.bbox().diagonal())?;
    let params = TessParams {
        linear,
        angular: q.angular,
        min_size: q.min_size,
        in_parallel,
        report_progress,
    };
    let result = s.occ().tessellate(params, ctx)?;
    ctx.checkpoint()?;
    if report_progress {
        ctx.progress(1.0);
    }
    Ok(result)
}

/// `tessellate_edge`: points along edge `e` within `q` (relative to the shape's bbox).
pub(crate) fn tessellate_edge(s: &Shape, e: EdgeId, q: &TessQuality) -> KResult<Vec<Point3>> {
    let n = s.count(crate::shape::SubKind::Edge);
    if e.0 >= n {
        return Err(KernelError::InvalidInput {
            what: "edge",
            reason: format!("{e:?} is out of range: the shape has {n} edges"),
            offending: Vec::new(),
        });
    }
    let linear = absolute_deflection(q, s.bbox().diagonal())?;
    s.occ()
        .tessellate_edge(e.0, linear, q.angular)
        .map_err(|error| match error {
            KernelError::Occt { message, .. } if message.contains("degenerated") => {
                KernelError::InvalidInput {
                    what: "edge",
                    reason: "degenerated edges have no curve to tessellate".to_owned(),
                    offending: vec![crate::shape::SubShape {
                        kind: crate::shape::SubKind::Edge,
                        index: e.0,
                    }],
                }
            }
            other => other,
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two faces sharing an edge: a unit square split along its diagonal, one triangle per face.
    fn two_faces() -> TessMesh {
        TessMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            normals: vec![Vec3::Z; 6],
            tris: vec![[0, 1, 2], [3, 4, 5]],
            face_ranges: vec![0..1, 1..1, 1..2],
            weld: vec![0, 1, 2, 0, 2, 3],
            edges: Vec::new(),
            edge_points: Vec::new(),
            uvs: None,
        }
    }

    #[test]
    fn face_of_triangle_skips_empty_faces() {
        let mesh = two_faces();
        assert_eq!(mesh.face_of_triangle(0), FaceId(0));
        assert_eq!(mesh.face_of_triangle(1), FaceId(2));
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn face_of_triangle_rejects_unknown_triangles() {
        let _ = two_faces().face_of_triangle(2);
    }

    #[test]
    fn indexed_parts_share_welded_vertices_and_drop_collapsed_triangles() {
        let mut mesh = two_faces();
        // A third face whose triangle collapses under the weld (two corners on one pole).
        mesh.positions.extend([Point3::new(0.0, 0.0, 1.0); 3]);
        mesh.normals.extend([Vec3::Z; 3]);
        mesh.weld.extend([4, 4, 5]);
        mesh.tris.push([6, 7, 8]);
        mesh.face_ranges.push(2..3);
        let (positions, tris, faces) = mesh.to_indexed_parts();
        assert_eq!(positions.len(), 4);
        assert_eq!(tris, vec![[0, 1, 2], [0, 2, 3]]);
        assert_eq!(faces, vec![0, 2]);
        assert_eq!(mesh.triangle_count(), 3);
        assert!(mesh.estimated_bytes() > 0);
    }

    #[test]
    fn relative_deflection_scales_and_clamps() {
        let close = |q: &TessQuality, diagonal: f64, want: f64| {
            let got = absolute_deflection(q, diagonal).unwrap();
            assert!((got - want).abs() < 1e-15, "{got} != {want}");
        };
        let display = TessQuality::display();
        close(&display, 200.0, 0.2);
        let export = TessQuality::export();
        close(&export, 50.0, 0.005);
        close(&export, 1000.0, 0.01);
        let absolute = TessQuality {
            relative: false,
            linear: 0.3,
            ..display
        };
        close(&absolute, f64::NAN, 0.3);
        for bad in [
            TessQuality {
                linear: 0.0,
                ..display
            },
            TessQuality {
                angular: f64::INFINITY,
                ..display
            },
            TessQuality {
                min_size: -1.0,
                ..display
            },
            TessQuality {
                max_linear_abs: Some(0.0),
                ..export
            },
        ] {
            assert!(matches!(
                absolute_deflection(&bad, 10.0),
                Err(KernelError::InvalidInput { .. })
            ));
        }
        assert!(absolute_deflection(&display, 0.0).is_err());
    }
}

#[cfg(all(test, feature = "occt"))]
mod occt_tests {
    use super::*;
    use crate::kernel::{BrepKernel, OcctKernel};

    /// Area enclosed by `mesh`'s welded triangles.
    fn mesh_area(mesh: &TessMesh) -> f64 {
        let (positions, tris, _) = mesh.to_indexed_parts();
        tris.iter()
            .map(|t| {
                let [a, b, c] = t.map(|v| positions[v as usize]);
                0.5 * (b - a).cross(c - a).length()
            })
            .sum()
    }

    /// BREP-08 Done-when 3: sphere r = 10 tessellations converge to the exact area 4πr² as the
    /// deflection shrinks.
    #[test]
    fn sphere_area_converges_as_deflection_shrinks() {
        let radius = 10.0;
        let kernel = OcctKernel::new(crate::KernelConfig::default());
        let sphere = kernel
            .make_sphere(&amg_core::Plane::XY, radius)
            .unwrap()
            .shape;
        let exact = 4.0 * std::f64::consts::PI * radius * radius;
        let mut last = f64::INFINITY;
        for linear in [1.0, 0.3, 0.1, 0.03, 0.01, 0.003, 0.001] {
            let q = TessQuality {
                linear,
                // Wide enough that the linear deflection alone sets the density.
                angular: 1.5,
                relative: false,
                min_size: 0.0,
                max_linear_abs: None,
            };
            let mesh = kernel.tessellate(&sphere, &q, &JobCtx::detached()).unwrap();
            let area = mesh_area(&mesh);
            let error = (exact - area) / exact;
            eprintln!(
                "deflection {linear}: {} tris, area {area:.6} (exact {exact:.6}), rel. error {error:.3e}",
                mesh.triangle_count()
            );
            // Inscribed: never above the exact area; strictly closer with each step.
            assert!(error >= 0.0, "{error}");
            assert!(error < last, "{error} after {last}");
            // A chord sagitta d on radius r loses about 2d/r of the area.
            assert!(error <= 2.5 * linear / radius, "{error} at {linear}");
            last = error;
        }
        assert!(last < 1e-4, "{last}");
    }
}
