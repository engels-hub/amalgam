//! The cxx bridge. Every function returns `Result`, so a C++ exception can't reach
//! `std::terminate`: the shim's `rust::behavior::trycatch` turns each one into an `Err`.
// The bridge macro expands to unsafe FFI glue, which cxx checks against the C++ declarations.
// Opaque C++ types have nothing Rust could print, so they don't implement `Debug`.
// Its generated code also names shared structs by full path, which `unused_qualifications` flags.
#![allow(unsafe_code, missing_debug_implementations, unused_qualifications)]

#[cxx::bridge(namespace = "amg::occt")]
pub mod ffi {
    /// A point or vector in mm.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct V3 {
        x: f64,
        y: f64,
        z: f64,
    }

    /// A right-handed frame: `normal = x × y`, all unit length.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcFrame {
        origin: V3,
        x: V3,
        y: V3,
        normal: V3,
    }

    /// The upper 3×4 of an affine transform in row order: `[a11 a12 a13 tx a21 … tz]`.
    #[derive(Clone, Copy, Debug, PartialEq)]
    struct OcMat34 {
        m: [f64; 12],
    }

    /// An axis-aligned box. `min`/`max` are meaningless when `is_void`.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcBox {
        is_void: bool,
        min: V3,
        max: V3,
    }

    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcUvBounds {
        umin: f64,
        umax: f64,
        vmin: f64,
        vmax: f64,
    }

    /// Topology tables, all 0-based, flat, in `TopExp::MapShapes` index order.
    ///
    /// Wires are listed per face, outer wire first; a wire's edges are in traversal order, so a
    /// seam edge appears twice.
    #[derive(Clone, Debug, Default, PartialEq)]
    struct OcTopology {
        n_faces: u32,
        n_edges: u32,
        n_vertices: u32,
        /// `n_faces + 1` offsets into the wire list.
        face_wire_starts: Vec<u32>,
        /// `n_wires + 1` offsets into `wire_edges`.
        wire_edge_starts: Vec<u32>,
        wire_edges: Vec<u32>,
        /// Two per edge: forward, then reversed end; `u32::MAX` where an end has no vertex.
        edge_vertices: Vec<u32>,
        /// `n_vertices + 1` offsets into `vertex_edges`.
        vertex_edge_starts: Vec<u32>,
        vertex_edges: Vec<u32>,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum OcSurfaceKind {
        Plane,
        Cylinder,
        Cone,
        Sphere,
        Torus,
        BSpline,
        Other,
    }

    /// An untrimmed surface with the face's location applied.
    ///
    /// `a`/`b`: cylinder and sphere radius in `a`; cone reference radius and semi-angle;
    /// torus major and minor radius. B-spline fields are empty for every other kind.
    #[derive(Clone, Debug, PartialEq)]
    struct OcSurface {
        kind: OcSurfaceKind,
        frame: OcFrame,
        a: f64,
        b: f64,
        u_degree: u32,
        v_degree: u32,
        u_knots: Vec<f64>,
        u_mults: Vec<u32>,
        v_knots: Vec<f64>,
        v_mults: Vec<u32>,
        /// Row-major, `u` index outer.
        poles: Vec<V3>,
        nu: u32,
        nv: u32,
        /// Empty when non-rational.
        weights: Vec<f64>,
        u_periodic: bool,
        v_periodic: bool,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum OcCurveKind {
        /// Degenerated edge, or no 3D curve.
        Absent,
        Line,
        Circle,
        Ellipse,
        BSpline,
        Other,
    }

    /// An edge's 3D curve with the edge's location applied.
    ///
    /// A line stores its origin in `frame.origin` and direction in `frame.normal`. `a`/`b`:
    /// circle radius in `a`; ellipse major and minor radius.
    #[derive(Clone, Debug, PartialEq)]
    struct OcCurve {
        kind: OcCurveKind,
        first: f64,
        last: f64,
        frame: OcFrame,
        a: f64,
        b: f64,
        degree: u32,
        knots: Vec<f64>,
        mults: Vec<u32>,
        poles: Vec<V3>,
        /// Empty when non-rational.
        weights: Vec<f64>,
        periodic: bool,
    }

    unsafe extern "C++" {
        include!("amg-occt-sys/cpp/occt_shim.hxx");

        /// An owned `TopoDS_Shape`. Opaque: OCCT types stay on the C++ side.
        type OcShape;

        /// The sub-shape maps of one shape. Immutable once built.
        type OcIndex;

        /// `occt-<OCC_VERSION_COMPLETE>;shim-<N>`.
        fn kernel_version() -> Result<String>;

        /// An axis-aligned box with one corner at the origin; sizes in mm.
        fn make_box(dx: f64, dy: f64, dz: f64) -> Result<UniquePtr<OcShape>>;

        /// The volume of `shape` in mm³.
        fn volume(shape: &OcShape) -> Result<f64>;

        /// Builds a `gp_Dir` from a zero vector, which raises `Standard_ConstructionError`.
        /// Exists to test the exception path (BREP-01 Done-when 2).
        fn gp_dir_from_zero_vector() -> Result<()>;

        // -- handle --

        fn shape_is_null(shape: &OcShape) -> Result<bool>;
        /// `TopAbs_ShapeEnum` value. Raises on a null shape.
        fn shape_kind(shape: &OcShape) -> Result<u8>;
        /// OCCT `IsSame`: same TShape and location, any orientation.
        fn shape_is_same(a: &OcShape, b: &OcShape) -> Result<bool>;
        /// A copy sharing the TShape, moved by the rigid transform `m`.
        fn shape_moved(shape: &OcShape, m: &OcMat34) -> Result<UniquePtr<OcShape>>;
        fn shape_bbox(shape: &OcShape) -> Result<OcBox>;

        // -- index --

        fn build_index(shape: &OcShape) -> Result<UniquePtr<OcIndex>>;
        fn index_topology(index: &OcIndex) -> Result<OcTopology>;
        fn index_solid_count(index: &OcIndex) -> Result<u32>;
        fn index_solid(index: &OcIndex, i: u32) -> Result<UniquePtr<OcShape>>;
        /// `kind`: 0 face, 1 edge, 2 vertex.
        fn index_sub_shape(index: &OcIndex, kind: u8, i: u32) -> Result<UniquePtr<OcShape>>;
        /// `kind`: 0 face, 1 edge, 2 vertex.
        fn index_tolerance(index: &OcIndex, kind: u8, i: u32) -> Result<f64>;
        /// Rough heap footprint of the shape's topology and geometry in bytes.
        fn index_estimated_bytes(index: &OcIndex) -> Result<u64>;

        // -- geometry --

        fn face_surface(index: &OcIndex, f: u32) -> Result<OcSurface>;
        fn face_is_reversed(index: &OcIndex, f: u32) -> Result<bool>;
        fn face_uv_bounds(index: &OcIndex, f: u32) -> Result<OcUvBounds>;
        /// Exact face area in mm² (`BRepGProp`, no triangulation).
        fn face_area(index: &OcIndex, f: u32) -> Result<f64>;
        fn edge_curve(index: &OcIndex, e: u32) -> Result<OcCurve>;
        fn edge_is_seam(index: &OcIndex, e: u32, f: u32) -> Result<bool>;
        fn vertex_point(index: &OcIndex, v: u32) -> Result<V3>;

        // -- serialization --

        /// `BinTools` bytes, format version 4, no triangulation.
        fn shape_write_bin(shape: &OcShape) -> Result<UniquePtr<CxxString>>;
        fn shape_read_bin(data: &[u8]) -> Result<UniquePtr<OcShape>>;
        /// `BRepTools` ASCII, no triangulation.
        fn shape_write_text(shape: &OcShape) -> Result<UniquePtr<CxxString>>;
        fn shape_read_text(data: &[u8]) -> Result<UniquePtr<OcShape>>;
    }
}

// SAFETY: `OcShape` and `OcIndex` are immutable once constructed: the bridge exposes them only
// through `&` to shim functions that read them (categories R, N, C of S02 §2, each tagged in
// `occt_shim.hxx`). Reference counts inside `TopoDS_Shape` are atomic, and built topology and
// geometry have no internal caches (occt-integration.md §4.1), so concurrent reads and drops on
// any thread are race-free.
unsafe impl Send for ffi::OcShape {}
// SAFETY: as for `Send` above.
unsafe impl Sync for ffi::OcShape {}
// SAFETY: as for `OcShape` above. `OcIndex` holds `TopoDS_Shape` maps that are only read after
// `build_index` returns.
unsafe impl Send for ffi::OcIndex {}
// SAFETY: as for `Send` above.
unsafe impl Sync for ffi::OcIndex {}
