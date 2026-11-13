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

    /// A plane section (BREP-17): one curve per section edge, and the 0-based index of the input
    /// face each edge lies on (`u32::MAX` when OCCT names none), in the section's edge order.
    #[derive(Clone, Debug, Default, PartialEq)]
    struct OcSection {
        curves: Vec<OcCurve>,
        faces: Vec<u32>,
    }

    /// One `BRepCheck` finding. `kind`: 0 face, 1 edge, 2 vertex, 3 no sub-shape (a shell, a
    /// solid or the root shape). Wire findings are reported on the face holding the wire.
    #[derive(Clone, Debug, PartialEq, Eq)]
    struct OcCheckProblem {
        kind: u8,
        index: u32,
        /// The `BRepCheck_Status` name without its prefix, for example `NotClosed`.
        status: String,
    }

    /// `BRepCheck_Analyzer` result. `problems` is empty when `valid`.
    #[derive(Clone, Debug, Default, PartialEq, Eq)]
    struct OcCheckReport {
        valid: bool,
        problems: Vec<OcCheckProblem>,
    }

    /// `tessellate_shape` input. `linear` is the absolute deflection in mm (amg-brep resolves
    /// relative qualities); `min_size` 0 keeps OCCT's default.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcTessParams {
        linear: f64,
        angular: f64,
        min_size: f64,
        /// `IMeshTools_Parameters::InParallel` (faces of one shape on OCCT's pool).
        in_parallel: bool,
    }

    /// A tessellation copied out of the private, meshed topology copy. Flat arrays: three
    /// `f64` per point, three `u32` per triangle. Faces and edges in index order.
    #[derive(Clone, Debug, Default, PartialEq)]
    struct OcTessMesh {
        /// `true` when the progress sink asked to stop; every other field is then empty.
        cancelled: bool,
        /// Per-face vertices, mm.
        positions: Vec<f64>,
        /// Unit normals, one per vertex, facing out of the material.
        normals: Vec<f64>,
        /// Counter-clockwise seen from outside, 0-based into `positions`.
        tris: Vec<u32>,
        /// `n_faces + 1` offsets into the triangle list.
        face_tri_starts: Vec<u32>,
        /// Canonical shared-vertex id per vertex: vertex index for B-rep vertices, then edge
        /// interior nodes, then face interior nodes.
        weld: Vec<u32>,
        /// `n_edges + 1` offsets (in points) into `edge_points`.
        edge_point_starts: Vec<u32>,
        edge_points: Vec<f64>,
        /// Edge polygons whose node count differed between two faces (welded as distinct).
        weld_mismatches: u32,
        /// Wall time of the topology copy, the meshing and the copy-out, in seconds.
        copy_seconds: f64,
        mesh_seconds: f64,
        extract_seconds: f64,
    }

    /// One builder input edge (BREP-03): `curve` between `curve.first < curve.last`, run
    /// backwards when `reversed`. Uses `OcCurve`'s output layout; `Absent`/`Other` raise.
    #[derive(Clone, Debug, PartialEq)]
    struct OcEdgeIn {
        curve: OcCurve,
        reversed: bool,
    }

    /// Edges of several loops: loop `i` is `edges[loop_starts[i]..loop_starts[i + 1]]`.
    /// Consecutive edges of a loop meet (the builders' callers checked the gaps).
    #[derive(Clone, Debug, Default, PartialEq)]
    struct OcLoopsIn {
        edges: Vec<OcEdgeIn>,
        loop_starts: Vec<u32>,
    }

    /// One planar face to build: outer loop first (CCW about `frame.normal`), then holes (CW).
    #[derive(Clone, Debug, PartialEq)]
    struct OcPlanarIn {
        frame: OcFrame,
        loops: OcLoopsIn,
    }

    /// The history of one shape op (BREP-14) from operand 0, in index terms (kinds: 0 face,
    /// 1 edge, 2 vertex). Relation `i` maps input `(in_kinds[i], in_indices[i])` with
    /// `relations[i]` (0 modified, 1 generated, 2 deleted: no outputs) to the outputs
    /// `out_kinds/out_indices[out_starts[i]..out_starts[i + 1]]`; `out_starts` has one more
    /// entry than there are relations. `roles[j]` (0 start cap, 1 end cap, 2 lateral) labels output
    /// face `role_faces[j]`.
    #[derive(Clone, Debug, Default, PartialEq, Eq)]
    struct OcOpHistory {
        in_kinds: Vec<u8>,
        in_indices: Vec<u32>,
        relations: Vec<u8>,
        out_starts: Vec<u32>,
        out_kinds: Vec<u8>,
        out_indices: Vec<u32>,
        role_faces: Vec<u32>,
        roles: Vec<u8>,
    }

    extern "Rust" {
        /// The progress/cancel target of one shim call (S02 §1, occt-integration.md §3.4).
        type ProgressSink;

        /// Called from OCCT progress `Show`: `fraction` of the whole call in `[0, 1]`.
        fn progress_show(sink: &ProgressSink, fraction: f64);

        /// Called from OCCT progress `UserBreak`: whether the call should stop.
        fn progress_user_break(sink: &ProgressSink) -> bool;

        /// Called by the message printer `install_message_printer` puts in place. `gravity` is
        /// `Message_Gravity`: 0 trace, 1 info, 2 warning, 3 alarm, 4 fail.
        fn occt_log(gravity: u8, message: &[u8]);
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

        // -- kernel infrastructure (BREP-12) --

        /// Replaces the default messenger's printers (stdout) with one that calls `occt_log`.
        /// Process-global: call once, before other kernel work.
        fn install_message_printer() -> Result<()>;
        /// Sends `text` through the default messenger with `gravity` (0..=4).
        fn emit_message(gravity: u8, text: &str) -> Result<()>;
        /// Creates or resizes `OSD_ThreadPool::DefaultPool`; returns its thread count.
        /// Process-global: call once, before other kernel work.
        fn set_default_thread_pool(threads: i32) -> Result<i32>;
        fn default_thread_pool_size() -> Result<i32>;
        /// A progress scope of `steps` busy-waits of `step_micros` µs each, reporting to `sink`.
        /// Returns `false` when `sink` asked to stop. Exists to test the progress bridge.
        fn progress_test_op(sink: &ProgressSink, steps: u32, step_micros: u32) -> Result<bool>;
        /// The largest tolerance of any sub-shape in mm (`ShapeAnalysis_ShapeTolerance`); 0 for
        /// a null shape.
        fn shape_max_tolerance(shape: &OcShape) -> Result<f64>;
        /// `BRepCheck_Analyzer` over `shape`; `index` must be `shape`'s index.
        fn check_shape(shape: &OcShape, index: &OcIndex) -> Result<OcCheckReport>;
        /// A planar face bounded by an open two-edge wire, which `BRepCheck` rejects. For tests of
        /// the validation policy.
        fn make_open_wire_face() -> Result<UniquePtr<OcShape>>;

        // -- tessellation (BREP-08) --

        /// Meshes a private copy of `shape` (category C) and copies the result out. Reports
        /// progress to `sink`; returns `cancelled` when it asks to stop.
        fn tessellate_shape(
            shape: &OcShape,
            params: &OcTessParams,
            sink: &ProgressSink,
        ) -> Result<OcTessMesh>;
        /// Points along edge `e` (parameter order) within `linear` mm and `angular` rad.
        /// Raises on a degenerated edge.
        fn tessellate_edge(index: &OcIndex, e: u32, linear: f64, angular: f64) -> Result<Vec<V3>>;
        /// `object` minus `tool`, non-destructive. A test probe until BREP-04.
        fn boolean_cut_probe(object: &OcShape, tool: &OcShape) -> Result<UniquePtr<OcShape>>;

        // -- face and wire builders (BREP-03), implemented in occt_build.cpp --

        /// The shapes one builder call made, each with the 0-based index (in the shape's own
        /// `TopExp::MapShapes` edge order) of the edge built from every input edge, or
        /// `u32::MAX` where it did not survive. Immutable once built.
        type OcBuild;

        /// One planar face per input. Raises naming the face, loop and edge OCCT rejects.
        fn make_planar_faces(faces: &[OcPlanarIn], tol: f64) -> Result<UniquePtr<OcBuild>>;
        /// One face on `surface`. `bounds`: 0 natural, 1 `uv` rectangle, 2 `loops` of 3D curves,
        /// 3 polylines (`points`, polyline `i` = `points[point_starts[i]..point_starts[i + 1]]`).
        fn make_face_on_surface(
            surface: &OcSurface,
            bounds: u8,
            uv: &OcUvBounds,
            loops: &OcLoopsIn,
            points: &[V3],
            point_starts: &[u32],
            tol: f64,
        ) -> Result<UniquePtr<OcBuild>>;
        /// One wire through the single loop of `edges`, closed when `closed`.
        fn make_wire(edges: &OcLoopsIn, closed: bool, tol: f64) -> Result<UniquePtr<OcBuild>>;
        fn build_count(build: &OcBuild) -> Result<u32>;
        fn build_shape(build: &OcBuild, i: u32) -> Result<UniquePtr<OcShape>>;
        fn build_edge_map(build: &OcBuild, i: u32) -> Result<Vec<u32>>;

        /// `shape` cut by `plane` (`BRepAlgoAPI_Section`, non-destructive, approximated). Lines,
        /// circles and ellipses come back as such (B-splines are tried for them within `tol`);
        /// everything else as a B-spline. `index` must be `shape`'s index.
        fn plane_section(
            shape: &OcShape,
            index: &OcIndex,
            plane: &OcFrame,
            tol: f64,
        ) -> Result<OcSection>;
        /// Edge `e`'s curve as a line, circle, ellipse or B-spline (exact conversion where OCCT
        /// has one, else a C2 approximation within `tol`). `Absent` for a degenerated edge.
        fn edge_curve_recognised(index: &OcIndex, e: u32, tol: f64) -> Result<OcCurve>;

        // -- primitives, sweeps and transforms (BREP-14), implemented in occt_ops.cpp --

        /// The result of one shape op and its history. Immutable once built.
        type OcOp;

        /// A primitive solid on `frame` (`BRepPrimAPI_Make*` on `gp_Ax2(origin, normal, x)`).
        /// `kind` and `params`: 0 box `[dx, dy, dz]`, 1 cylinder `[r, h, angle]`, 2 cone
        /// `[r1, r2, h, angle]`, 3 sphere `[r]`, 4 torus `[major, minor, angle]`.
        fn make_primitive(kind: u8, frame: &OcFrame, params: &[f64]) -> Result<UniquePtr<OcShape>>;
        /// `profile` swept by `vec` (`mode` 0), or infinitely along `vec` (1 one way, 2 both
        /// ways). A finite prism of a planar face may be tapered by `taper` rad (positive
        /// grows the profile along the sweep) and have coplanar faces merged.
        fn prism(
            profile: &OcShape,
            vec: &V3,
            mode: u8,
            taper: f64,
            merge_coplanar: bool,
        ) -> Result<UniquePtr<OcOp>>;
        /// `profile` revolved by `angle` rad about the axis through `origin` along `axis`.
        fn revol(profile: &OcShape, origin: &V3, axis: &V3, angle: f64) -> Result<UniquePtr<OcOp>>;
        /// `shape` mapped by the affine `m` into new geometry: a similarity (rotation, uniform
        /// scale, reflection) keeps analytic geometry, any other map goes through
        /// `BRepBuilderAPI_GTransform` (B-spline geometry).
        fn shape_gtransformed(shape: &OcShape, m: &OcMat34) -> Result<UniquePtr<OcOp>>;
        fn op_shape(op: &OcOp) -> Result<UniquePtr<OcShape>>;
        fn op_history(op: &OcOp) -> Result<OcOpHistory>;
    }
}

/// What an OCCT progress indicator reports to (S02 §1). `amg-brep` implements it over `JobCtx`.
///
/// Both methods run on whatever thread OCCT calls from, possibly several at once.
pub trait ProgressTarget {
    /// `fraction` of the whole call, in `[0, 1]`.
    fn show(&self, fraction: f64);
    /// Whether the call should stop.
    fn user_break(&self) -> bool;
}

/// The progress/cancel target handed to one shim call.
pub struct ProgressSink {
    target: Box<dyn ProgressTarget>,
}

impl ProgressSink {
    /// Wraps `target`.
    pub fn new(target: Box<dyn ProgressTarget>) -> ProgressSink {
        ProgressSink { target }
    }
}

impl std::fmt::Debug for ProgressSink {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("ProgressSink")
    }
}

// A panic must not unwind into C++ (cxx would abort), so the callbacks swallow them: a lost
// progress update is harmless, and a panicking cancel check counts as "stop".
fn progress_show(sink: &ProgressSink, fraction: f64) {
    let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        sink.target.show(fraction);
    }));
}

fn progress_user_break(sink: &ProgressSink) -> bool {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| sink.target.user_break()))
        .unwrap_or(true)
}

/// `tracing` target of every OCCT message.
pub const LOG_TARGET: &str = "occt";

fn occt_log(gravity: u8, message: &[u8]) {
    let text = String::from_utf8_lossy(message);
    let text = text.trim_end();
    match gravity {
        0 => tracing::trace!(target: "occt", "{text}"),
        1 => tracing::info!(target: "occt", "{text}"),
        2 => tracing::warn!(target: "occt", "{text}"),
        _ => tracing::error!(target: "occt", gravity, "{text}"),
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
