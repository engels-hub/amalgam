//! `Shape`: the immutable, `Send + Sync` handle to a B-rep (S02 §2–3, BREP-02).

use std::fmt;
use std::sync::{Arc, OnceLock};

use amg_core::{Aabb3, Mat3, Point3, Transform};

use crate::KResult;
use crate::backend::Occ;
use crate::check::CheckReport;
use crate::error::KernelError;
use crate::geom;
use crate::index::SubShapeIndex;

/// Version byte after the `AMGB` magic in [`Shape::write_bin`] output.
pub const FORMAT_VERSION: u8 = 1;

const BIN_MAGIC: &[u8; 4] = b"AMGB";

/// How far `RᵀR` may stray from the identity for a transform to count as rigid.
const RIGID_EPS: f64 = 1e-9;

/// The topological type of a shape.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum ShapeKind {
    /// A group of arbitrary shapes.
    Compound,
    /// Solids sharing faces.
    CompSolid,
    /// A solid.
    Solid,
    /// Connected faces.
    Shell,
    /// A trimmed surface.
    Face,
    /// Connected edges.
    Wire,
    /// A trimmed curve.
    Edge,
    /// A point.
    Vertex,
}

/// The kinds of indexed sub-shape.
#[derive(
    Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub enum SubKind {
    /// A face.
    Face,
    /// An edge.
    Edge,
    /// A vertex.
    Vertex,
}

/// 0-based face index, valid only for the `Shape` it came from.
#[derive(
    Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(transparent)]
pub struct FaceId(pub u32);

/// 0-based edge index, valid only for the `Shape` it came from.
#[derive(
    Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(transparent)]
pub struct EdgeId(pub u32);

/// 0-based vertex index, valid only for the `Shape` it came from.
#[derive(
    Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(transparent)]
pub struct VertexId(pub u32);

/// Kind plus 0-based index (`TopExp::MapShapes` order). Valid only for its `Shape`.
#[derive(
    Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct SubShape {
    /// Which index space.
    pub kind: SubKind,
    /// Position in it.
    pub index: u32,
}

struct ShapeInner {
    occ: Occ,
    index: OnceLock<SubShapeIndex>,
    bbox: OnceLock<Aabb3>,
}

/// An immutable B-rep shape. `Clone` is an `Arc` clone.
///
/// # Panics
/// Query methods that take a [`FaceId`], [`EdgeId`] or [`VertexId`] panic when the id is out of
/// range for this shape, as slice indexing does.
#[derive(Clone)]
pub struct Shape {
    inner: Arc<ShapeInner>,
}

static_assertions::assert_impl_all!(Shape: Send, Sync);

impl Shape {
    pub(crate) fn from_occ(occ: Occ) -> Shape {
        Shape {
            inner: Arc::new(ShapeInner {
                occ,
                index: OnceLock::new(),
                bbox: OnceLock::new(),
            }),
        }
    }

    /// Builds a named fixture part for the `gen_brep_fixtures` generator.
    #[cfg(feature = "fixture-gen")]
    #[doc(hidden)]
    pub fn fixture(name: &str) -> KResult<Shape> {
        Occ::fixture(name).map(Shape::from_occ)
    }

    #[cfg(all(test, feature = "occt"))]
    pub(crate) fn test_box(dx: f64, dy: f64, dz: f64) -> Shape {
        Shape::from_occ(Occ::test_box(dx, dy, dz))
    }

    #[cfg(all(test, feature = "occt"))]
    pub(crate) fn test_open_wire_face() -> Shape {
        Shape::from_occ(Occ::test_open_wire_face())
    }

    fn occ(&self) -> &Occ {
        &self.inner.occ
    }

    fn index(&self) -> &SubShapeIndex {
        self.inner
            .index
            .get_or_init(|| SubShapeIndex::from_raw(self.occ().topology()))
    }

    fn check_face(&self, f: FaceId) -> u32 {
        let n = self.index().n_faces();
        assert!(f.0 < n, "{f:?} is out of range: the shape has {n} faces");
        f.0
    }

    fn check_edge(&self, e: EdgeId) -> u32 {
        let n = self.index().n_edges();
        assert!(e.0 < n, "{e:?} is out of range: the shape has {n} edges");
        e.0
    }

    fn check_vertex(&self, v: VertexId) -> u32 {
        let n = self.index().n_vertices();
        assert!(v.0 < n, "{v:?} is out of range: the shape has {n} vertices");
        v.0
    }

    /// The shape's topological type.
    pub fn kind(&self) -> ShapeKind {
        self.occ().kind()
    }

    /// Whether the shape is empty. Constructors reject null shapes, so this is `false` in practice.
    pub fn is_null(&self) -> bool {
        self.occ().is_null()
    }

    /// Whether both handles share one allocation.
    pub fn ptr_eq(&self, other: &Shape) -> bool {
        Arc::ptr_eq(&self.inner, &other.inner)
    }

    /// OCCT `IsSame`: same TShape and location, any orientation.
    pub fn is_same(&self, other: &Shape) -> bool {
        self.occ().is_same(other.occ())
    }

    /// How many sub-shapes of kind `k` the shape has.
    pub fn count(&self, k: SubKind) -> u32 {
        let index = self.index();
        match k {
            SubKind::Face => index.n_faces(),
            SubKind::Edge => index.n_edges(),
            SubKind::Vertex => index.n_vertices(),
        }
    }

    /// All faces in index order.
    pub fn faces(&self) -> impl ExactSizeIterator<Item = FaceId> + '_ {
        (0..self.count(SubKind::Face)).map(FaceId)
    }

    /// All edges in index order.
    pub fn edges(&self) -> impl ExactSizeIterator<Item = EdgeId> + '_ {
        (0..self.count(SubKind::Edge)).map(EdgeId)
    }

    /// All vertices in index order.
    pub fn vertices(&self) -> impl ExactSizeIterator<Item = VertexId> + '_ {
        (0..self.count(SubKind::Vertex)).map(VertexId)
    }

    /// The solids in the shape, in index order.
    pub fn solids(&self) -> Vec<Shape> {
        self.occ()
            .solids()
            .into_iter()
            .map(Shape::from_occ)
            .collect()
    }

    /// Extracts a sub-shape as a standalone `Shape` sharing its TShape.
    ///
    /// # Errors
    /// `InvalidInput` when `s` is out of range.
    pub fn sub_shape(&self, s: SubShape) -> KResult<Shape> {
        let n = self.count(s.kind);
        if s.index >= n {
            return Err(KernelError::InvalidInput {
                what: "sub_shape",
                reason: format!("index {} is out of range ({n} sub-shapes)", s.index),
                offending: vec![s],
            });
        }
        self.occ().sub_shape(s.kind, s.index).map(Shape::from_occ)
    }

    /// The face's distinct edges: outer wire first, then holes, each in wire order. A seam
    /// edge, which a wire traverses twice, is listed once; [`Shape::face_wires`] keeps both.
    pub fn face_edges(&self, f: FaceId) -> &[EdgeId] {
        self.check_face(f);
        self.index().face_edges(f)
    }

    /// The face's wires, outer first, each in traversal order. A seam edge appears twice.
    pub fn face_wires(&self, f: FaceId) -> Vec<Vec<EdgeId>> {
        self.check_face(f);
        self.index().face_wires(f)
    }

    /// The faces using this edge, ascending: none (free), one (boundary or seam), two
    /// (manifold) or more.
    pub fn edge_faces(&self, e: EdgeId) -> &[FaceId] {
        self.check_edge(e);
        self.index().edge_faces(e)
    }

    /// The edge's end vertices along its curve (forward end, then reversed end). A closed edge
    /// has the same vertex twice; an end with no vertex (infinite edges) is `VertexId(u32::MAX)`.
    pub fn edge_vertices(&self, e: EdgeId) -> [VertexId; 2] {
        self.check_edge(e);
        self.index().edge_vertices(e)
    }

    /// The edges meeting at this vertex, ascending.
    pub fn vertex_edges(&self, v: VertexId) -> &[EdgeId] {
        self.check_vertex(v);
        self.index().vertex_edges(v)
    }

    /// Whether `e` closes `f`'s periodic surface (it has two pcurves on the face).
    pub fn is_seam(&self, e: EdgeId, f: FaceId) -> bool {
        let edge = self.check_edge(e);
        let face = self.check_face(f);
        self.occ().is_seam(edge, face)
    }

    /// Optimal bounding box (`BRepBndLib::AddOptimal`), cached. Empty for a shape with no
    /// geometry.
    pub fn bbox(&self) -> Aabb3 {
        *self.inner.bbox.get_or_init(|| self.occ().bbox())
    }

    /// The face's underlying surface, untrimmed, with the face's location applied.
    pub fn surface(&self, f: FaceId) -> geom::Surface {
        let face = self.check_face(f);
        self.occ().surface(face)
    }

    /// Whether the face's normal opposes its surface's parametric normal.
    pub fn face_reversed(&self, f: FaceId) -> bool {
        let face = self.check_face(f);
        self.occ().face_reversed(face)
    }

    /// `[umin, umax, vmin, vmax]` of the face in its surface's parameters.
    pub fn face_uv_bounds(&self, f: FaceId) -> [f64; 4] {
        let face = self.check_face(f);
        self.occ().face_uv_bounds(face)
    }

    /// The face's exact area in mm², from its surface and trim rather than a mesh.
    pub fn face_area(&self, f: FaceId) -> f64 {
        let face = self.check_face(f);
        self.occ().face_area(face)
    }

    /// The edge's curve and parameter range, with the edge's location applied. `None` for
    /// degenerated edges and edges with no 3D curve. The range follows the curve's direction,
    /// whatever the edge's orientation.
    pub fn curve(&self, e: EdgeId) -> Option<(geom::Curve3, f64, f64)> {
        let edge = self.check_edge(e);
        self.occ().curve(edge)
    }

    /// The vertex's position.
    pub fn vertex_point(&self, v: VertexId) -> Point3 {
        let vertex = self.check_vertex(v);
        self.occ().vertex_point(vertex)
    }

    /// OCCT's tolerance for the sub-shape, mm.
    ///
    /// # Panics
    /// When `s` is out of range.
    pub fn tolerance(&self, s: SubShape) -> f64 {
        let n = self.count(s.kind);
        assert!(
            s.index < n,
            "{s:?} is out of range: the shape has {n} of that kind"
        );
        self.occ().tolerance(s.kind, s.index)
    }

    /// The largest tolerance of any sub-shape (faces, edges, vertices) in mm
    /// (`ShapeAnalysis_ShapeTolerance`), for S20's validation gate. 0 for a shape with no
    /// sub-shapes.
    pub fn max_tolerance(&self) -> f64 {
        self.occ().max_tolerance()
    }

    /// `BRepCheck` validity (S02 §9); [`crate::BrepKernel::check`] is the public entry.
    pub(crate) fn check_report(&self) -> CheckReport {
        self.occ().check()
    }

    /// Volume enclosed by the shape's solids in mm³ (`BRepGProp::VolumeProperties`; 0 for a
    /// shape without solids). Added for S06 ASM-09, whose tests check that moving a body
    /// between components keeps its volume.
    ///
    /// # Errors
    /// `Occt` if OCCT raises.
    pub fn volume(&self) -> KResult<f64> {
        self.occ().volume()
    }

    /// The same shape moved by a rigid transform. Shares TShapes and keeps every index.
    ///
    /// # Errors
    /// `NonRigid` for scale, shear, reflection or non-finite values.
    pub fn transformed(&self, t: &Transform) -> KResult<Shape> {
        if !is_rigid(t) {
            return Err(KernelError::NonRigid);
        }
        self.occ().moved(t).map(Shape::from_occ)
    }

    /// Writes the binary form: `AMGB`, [`FORMAT_VERSION`], then `BinTools` data. Bit-exact, and
    /// indices are stable across it.
    ///
    /// # Errors
    /// `Occt` if serialization raises, `Io` if `w` fails.
    pub fn write_bin<W: std::io::Write>(&self, mut w: W) -> KResult<()> {
        let body = self.occ().write_bin()?;
        w.write_all(BIN_MAGIC)?;
        w.write_all(&[FORMAT_VERSION])?;
        w.write_all(&body)?;
        Ok(())
    }

    /// Reads what [`Shape::write_bin`] wrote.
    ///
    /// # Errors
    /// `Format` for a wrong header or unreadable data, `Io` if `r` fails, `BackendUnavailable`
    /// without the `occt` feature.
    pub fn read_bin<R: std::io::Read>(mut r: R) -> KResult<Shape> {
        let mut data = Vec::new();
        r.read_to_end(&mut data)?;
        let (header, body) = data
            .split_at_checked(BIN_MAGIC.len() + 1)
            .ok_or_else(|| KernelError::Format("too short for an AMGB header".to_owned()))?;
        if &header[..BIN_MAGIC.len()] != BIN_MAGIC {
            return Err(KernelError::Format("missing AMGB magic".to_owned()));
        }
        let version = header[BIN_MAGIC.len()];
        if version != FORMAT_VERSION {
            return Err(KernelError::Format(format!(
                "AMGB format {version} is not supported (expected {FORMAT_VERSION})"
            )));
        }
        Occ::read_bin(body).map(Shape::from_occ)
    }

    /// Writes OCCT's ASCII `.brep`, without triangulation.
    ///
    /// # Errors
    /// `Occt` if serialization raises, `Io` if `w` fails.
    pub fn write_text<W: std::io::Write>(&self, mut w: W) -> KResult<()> {
        w.write_all(&self.occ().write_text()?)?;
        Ok(())
    }

    /// Reads an ASCII `.brep`.
    ///
    /// # Errors
    /// `Format` for unreadable data, `Io` if `r` fails, `BackendUnavailable` without the `occt`
    /// feature.
    pub fn read_text<R: std::io::Read>(mut r: R) -> KResult<Shape> {
        let mut data = Vec::new();
        r.read_to_end(&mut data)?;
        Occ::read_text(&data).map(Shape::from_occ)
    }

    /// Approximate heap footprint for S01's LRU cache accounting.
    pub fn estimated_bytes(&self) -> u64 {
        self.occ().estimated_bytes() + self.index().heap_bytes()
    }
}

impl fmt::Debug for Shape {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shape")
            .field("kind", &self.kind())
            .field("faces", &self.count(SubKind::Face))
            .field("edges", &self.count(SubKind::Edge))
            .field("vertices", &self.count(SubKind::Vertex))
            .field("bbox", &self.bbox())
            .finish()
    }
}

fn is_rigid(t: &Transform) -> bool {
    let m = t.matrix3;
    let error = m.transpose() * m - Mat3::IDENTITY;
    let orthonormal = error
        .to_cols_array()
        .iter()
        .all(|value| value.abs() <= RIGID_EPS);
    orthonormal && m.determinant() > 0.0 && t.translation.is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::Vec3;

    #[test]
    fn rigid_transforms_are_accepted() {
        assert!(is_rigid(&Transform::IDENTITY));
        assert!(is_rigid(&Transform::from_translation(Vec3::new(
            1.0, 2.0, 3.0
        ))));
        assert!(is_rigid(&Transform::from_rotation_z(0.7)));
    }

    #[test]
    fn scale_reflection_and_non_finite_are_not_rigid() {
        assert!(!is_rigid(&Transform::from_scale(Vec3::splat(2.0))));
        assert!(!is_rigid(&Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0))));
        assert!(!is_rigid(&Transform::from_translation(Vec3::new(
            f64::NAN,
            0.0,
            0.0
        ))));
    }

    #[test]
    fn surface_class_covers_each_kind() {
        let other = geom::Surface::Other {
            class: geom::GeomClass::Other,
        };
        assert_eq!(other.class(), geom::GeomClass::Other);
        assert_eq!(
            geom::Surface::Plane(amg_core::Plane::XY).class(),
            geom::GeomClass::Plane
        );
    }

    #[cfg(not(feature = "occt"))]
    #[test]
    fn no_backend_rejects_valid_header_as_unavailable() {
        let mut data = BIN_MAGIC.to_vec();
        data.push(FORMAT_VERSION);
        assert!(matches!(
            Shape::read_bin(data.as_slice()),
            Err(KernelError::BackendUnavailable)
        ));
        assert!(matches!(
            Shape::read_text(&b"x"[..]),
            Err(KernelError::BackendUnavailable)
        ));
    }

    #[test]
    fn bad_headers_are_format_errors() {
        for data in [&b"AMG"[..], &b"NOPE\x01rest"[..], &b"AMGB\xFFrest"[..]] {
            assert!(matches!(Shape::read_bin(data), Err(KernelError::Format(_))));
        }
    }

    #[cfg(feature = "occt")]
    #[test]
    fn volume_is_kept_by_a_rigid_move() {
        let b = Shape::test_box(10.0, 20.0, 30.0);
        assert!((b.volume().unwrap() - 6000.0).abs() < 1e-9);
        let t = Transform::from_translation(Vec3::new(5.0, -2.0, 7.0))
            * Transform::from_axis_angle(Vec3::new(1.0, 2.0, 3.0).normalize(), 0.7);
        let moved = b.transformed(&t).unwrap();
        assert!((moved.volume().unwrap() - 6000.0).abs() < 1e-9);
    }
}
