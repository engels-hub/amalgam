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
    ///
    /// Multi-operand ops (BREP-04) fill `in_operands` (one per relation; empty = operand 0 for
    /// all) and use relation 3 (kept: the same TShape is in the result, one output).
    #[derive(Clone, Debug, Default, PartialEq, Eq)]
    struct OcOpHistory {
        in_operands: Vec<u16>,
        in_kinds: Vec<u8>,
        in_indices: Vec<u32>,
        relations: Vec<u8>,
        out_starts: Vec<u32>,
        out_kinds: Vec<u8>,
        out_indices: Vec<u32>,
        role_faces: Vec<u32>,
        roles: Vec<u8>,
        /// One per role (or empty): the contour of a fillet blend face, 0 otherwise.
        role_args: Vec<u32>,
    }

    /// One edge of a fillet or chamfer (BREP-06), by index in the input shape. `kind`: 0 fillet
    /// constant (radius `a`), 1 fillet variable (law `laws[law_start..law_end]`: `(t, r)` pairs,
    /// `t` relative along the contour), 2 chamfer equal (distance `a`), 3 chamfer two distances
    /// (`a` on face `face`, `b` on the other), 4 chamfer distance-angle (`a` on face `face`,
    /// angle `b` rad from it).
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcBlendEdge {
        edge: u32,
        kind: u8,
        a: f64,
        b: f64,
        face: u32,
        law_start: usize,
        law_end: usize,
    }

    /// Fillet options (BREP-06). `shape`: 0 rational, 1 quasi-angular, 2 polynomial
    /// (`ChFi3d_FilletShape`); `tol_app_3d` and `fleche` ≤ 0 keep OCCT's defaults.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcFilletParams {
        shape: u8,
        tol_app_3d: f64,
        fleche: f64,
    }

    /// Shell, offset or thicken (BREP-07). `mode`: 0 shell (thick solid), 1 offset, 2 thicken,
    /// 3 offset faces (R-S04-5, BREP-16: `offset` 0, only the per-face list moves);
    /// `offset` along the outward normal (mm; negative is inward), `tol` the offset algorithm's
    /// confusion tolerance, `join` 0 arc, 1 intersection.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcOffsetParams {
        mode: u8,
        offset: f64,
        tol: f64,
        join: u8,
    }

    /// Heal options (BREP-10): `tol` the ShapeFix precision and `max_tol` the largest tolerance
    /// a fix may set (mm); `fix_small_edges` runs `ShapeFix_Wireframe::FixSmallEdges`;
    /// `fix_orientation` false turns off the face, shell and solid orientation fixes.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcHealParams {
        tol: f64,
        max_tol: f64,
        fix_small_edges: bool,
        fix_orientation: bool,
    }

    /// `ShapeUpgrade_UnifySameDomain` options (BREP-10).
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcUnifyParams {
        faces: bool,
        edges: bool,
        concat_bsplines: bool,
        lin_tol: f64,
        ang_tol: f64,
    }

    /// `BRepBuilderAPI_Sewing` options (BREP-10); `make_solid` turns every closed shell into a
    /// solid and fails when one is open.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcSewParams {
        tol: f64,
        non_manifold: bool,
        make_solid: bool,
    }

    /// Pipe-shell options (BREP-15). Operands are `[profiles..., spine, aux?]`; `profiles` is
    /// the profile count, `aux` the operand of the guide (`mode` 4) or support shell (`mode` 5).
    /// `mode`: 0 corrected Frenet, 1 Frenet, 2 fixed, 3 constant binormal `binormal`, 4 guide
    /// wire (`contact`: keep the section in contact with it), 5 spine support. `transition`:
    /// 0 transformed, 1 right corner, 2 round corner. `solid` closes the result with caps.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcPipeParams {
        profiles: u32,
        mode: u8,
        binormal: V3,
        aux: u32,
        contact: bool,
        transition: u8,
        solid: bool,
    }

    /// Raw `BRepOffsetAPI_ThruSections` options (BREP-15); `max_degree` 0 keeps OCCT's.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcLoftParams {
        solid: bool,
        ruled: bool,
        check_compatibility: bool,
        smoothing: bool,
        max_degree: u8,
    }

    /// Draft options (BREP-16): faces tilted by `angle` rad about their intersection with the
    /// `neutral` plane; a positive angle removes material on the `pull` side of the plane
    /// (`BRepOffsetAPI_DraftAngle`).
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcDraftParams {
        pull: V3,
        angle: f64,
        neutral: OcFrame,
    }

    /// One loft section: wire operand `wire`, or the point `point` when `wire` is `u32::MAX`.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcLoftSection {
        wire: u32,
        point: V3,
    }

    /// One sub-shape's fingerprint (BREP-05). `geom_class` is amg-brep's `GeomClass` in
    /// declaration order (0 plane … 11 point). `measure`: face area, edge length, 0 for a vertex;
    /// `centroid` area/length weighted (fixed-order `BRepGProp`); `axis` (when `has_axis`): the
    /// outward normal of a plane face, the axis of a cylinder, cone or torus, the direction of a
    /// line, the normal of a circle or ellipse; `bmin`/`bmax`: `BRepBndLib::AddOptimal` box.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcSubGeom {
        geom_class: u8,
        measure: f64,
        centroid: V3,
        has_axis: bool,
        axis: V3,
        bmin: V3,
        bmax: V3,
    }

    /// For each face, edge and vertex of a part (in its own index order), its index in the whole
    /// shape, `u32::MAX` when it is not there (BREP-05 `OpResult::per_solid`).
    #[derive(Clone, Debug, Default, PartialEq, Eq)]
    struct OcSubMap {
        faces: Vec<u32>,
        edges: Vec<u32>,
        vertices: Vec<u32>,
    }

    /// Boolean options (BREP-04). `glue`: 0 off, 1 shift, 2 full. Non-destructive is always on.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcBoolOpts {
        fuzzy: f64,
        glue: u8,
        check_inverted: bool,
        use_obb: bool,
        parallel: bool,
    }

    /// How a boolean ended (BREP-04): cancelled, or the BOPAlgo alert names (`errors` non-empty
    /// means it failed) and the operand sub-shapes (kinds 0 face, 1 edge, 2 vertex) they name.
    #[derive(Clone, Debug, Default, PartialEq, Eq)]
    struct OcBoolStatus {
        cancelled: bool,
        errors: Vec<String>,
        warnings: Vec<String>,
        offending_operands: Vec<u16>,
        offending_kinds: Vec<u8>,
        offending_indices: Vec<u32>,
    }

    /// Mass properties (BREP-09). `basis`: 0 volume (the shape has solids), 1 surface (faces
    /// only), 2 linear (edges only), 3 nothing to measure. `measure` is the mass of that basis
    /// at unit density; `centroid` and `inertia` (row-major, about the centroid) belong to it.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcMassProps {
        basis: u8,
        volume: f64,
        area: f64,
        measure: f64,
        centroid: V3,
        inertia: [f64; 9],
    }

    /// A minimum distance (BREP-09). Supports are `kind` 0 face, 1 edge, 2 vertex, 255 none
    /// (a point given by the caller, or `inner`: one shape lies inside a solid of the other);
    /// `u`/`v` are the face parameters of a face support.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcDistance {
        distance: f64,
        inner: bool,
        on_a: V3,
        on_b: V3,
        kind_a: u8,
        index_a: u32,
        kind_b: u8,
        index_b: u32,
        u_a: f64,
        v_a: f64,
        u_b: f64,
        v_b: f64,
    }

    /// The nearest ray hit (BREP-09); `face` is `u32::MAX` without a hit.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcRayHit {
        hit: bool,
        t: f64,
        point: V3,
        face: u32,
        u: f64,
        v: f64,
    }

    /// OCCT's natural differential properties of a face's surface at `(u, v)`: the normal is
    /// `D1U × D1V` (not flipped for a reversed face), curvatures are signed against it.
    /// Directions are zero when `umbilic`.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcSurfacePoint {
        point: V3,
        d1u: V3,
        d1v: V3,
        normal_defined: bool,
        normal: V3,
        curvature_defined: bool,
        k_min: f64,
        k_max: f64,
        umbilic: bool,
        dir_min: V3,
        dir_max: V3,
    }

    /// Edge end points and first derivatives in the 3D curve's parameter direction.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcEdgeEnds {
        degenerated: bool,
        p0: V3,
        t0: V3,
        p1: V3,
        t1: V3,
    }

    /// One node of an XCAF tree (BREP-13), in depth-first pre-order: free shapes are roots
    /// (`parent` `u32::MAX`), an assembly's components follow it. `proto` is the prototype a
    /// part node shows (`u32::MAX` for an assembly); `transform` is the location relative to the
    /// parent. Colours are linear RGBA.
    #[derive(Clone, Debug, PartialEq)]
    struct OcXdeNode {
        parent: u32,
        has_name: bool,
        name: String,
        transform: OcMat34,
        has_color: bool,
        color: [f32; 4],
        proto: u32,
    }

    /// The name and colour of one prototype (an XCAF simple-shape label).
    #[derive(Clone, Debug, Default, PartialEq)]
    struct OcXdeProto {
        has_name: bool,
        name: String,
        has_color: bool,
        color: [f32; 4],
    }

    /// The colour of face `face` (0-based, `TopExp::MapShapes` order) of prototype `proto`.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct OcXdeFaceColor {
        proto: u32,
        face: u32,
        color: [f32; 4],
    }

    /// How a STEP or IGES write is made: `schema` 0 AP203, 1 AP214, 2 AP242 (STEP only); `unit_mm`
    /// the file's length unit in mm (1, 10, 1000, 25.4, 304.8 or 0.001).
    #[derive(Clone, Copy, Debug, PartialEq)]
    struct OcXdeWriteParams {
        schema: u8,
        unit_mm: f64,
    }

    /// How a read or write went. `unit_names`: the STEP file's length units as written
    /// (`STEPControl_Reader::FileUnits`); `iges_unit_flag`: the IGES global-section unit flag
    /// (0 when not IGES).
    #[derive(Clone, Debug, Default, PartialEq)]
    struct OcXdeStatus {
        cancelled: bool,
        errors: Vec<String>,
        warnings: Vec<String>,
        unit_names: Vec<String>,
        iges_unit_flag: i32,
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
        /// The planar wall `left` mm to the left and `right` mm to the right of one open or
        /// closed chain (the single loop of `chain`). Its edge map holds `(edge, code)` pairs.
        fn make_thin_face(
            chain: &OcPlanarIn,
            closed: bool,
            left: f64,
            right: f64,
            tol: f64,
        ) -> Result<UniquePtr<OcBuild>>;
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
        /// `ShapeUpgrade_UnifySameDomain` (faces and edges, no B-spline concatenation: the
        /// options of `prism`'s `merge_coplanar`) on a private copy of `shape`, with the history
        /// from `shape`.
        fn unify_op(shape: &OcShape) -> Result<UniquePtr<OcOp>>;

        // -- history and naming inputs (BREP-05), implemented in occt_naming.cpp --

        /// Fingerprints of the sub-shapes `indices` of `kind` (0 face, 1 edge, 2 vertex).
        fn sub_geoms(index: &OcIndex, kind: u8, indices: &[u32]) -> Result<Vec<OcSubGeom>>;
        /// Where each sub-shape of `part` (a sub-shape of `whole`, e.g. one of its solids) is in
        /// `whole`'s index.
        fn sub_index_map(whole: &OcIndex, part: &OcIndex) -> Result<OcSubMap>;

        // -- booleans, split and section (BREP-04), implemented in occt_bool.cpp --

        /// Argument or tool shapes of one boolean.
        type OcShapeList;

        fn new_shape_list() -> Result<UniquePtr<OcShapeList>>;
        /// Appends a handle to `shape` (raises on a null shape).
        fn shape_list_push(list: Pin<&mut OcShapeList>, shape: &OcShape) -> Result<()>;
        /// `op`: 0 fuse, 1 cut, 2 common (`BRepAlgoAPI_BooleanOperation`), 3 split
        /// (`BRepAlgoAPI_Splitter`). Non-destructive, history filled for every operand (objects
        /// first, then tools). Progress and cancel through `sink`. A failure or cancellation is
        /// reported by `op_bool_status`, and the op then has no shape.
        fn boolean_op(
            op: u8,
            objects: &OcShapeList,
            tools: &OcShapeList,
            opts: &OcBoolOpts,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;
        /// The intersection edges and vertices of `a` and `b` (`BRepAlgoAPI_Section`,
        /// approximated, no pcurves). History: each face of either operand `Generated` into the
        /// section edges on it.
        fn section_op(
            a: &OcShape,
            b: &OcShape,
            parallel: bool,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;
        fn op_bool_status(op: &OcOp) -> Result<OcBoolStatus>;
        /// Volume by `BRepGProp::VolumeProperties` with the adaptive relative error bound `eps`
        /// (the plain `volume` uses fixed-order quadrature, which under-integrates faces whose
        /// boundaries are not iso-lines).
        fn volume_adaptive(shape: &OcShape, eps: f64) -> Result<f64>;

        // -- fillets and chamfers (BREP-06), implemented in occt_blend.cpp --

        /// Fillets (`chamfer` false) or chamfers `edges` of `shape` (`index` is its index) on a
        /// private copy. A failed build is reported by `op_bool_status` (errors, offending
        /// edges); its op then holds OCCT's partial result, or a null shape. An error starting
        /// with `invalid: ` is a bad input (an edge OCCT ignores, conflicting specs on one tangent
        /// chain). `with_history` false skips the history (the max-radius probe).
        fn blend_op(
            shape: &OcShape,
            index: &OcIndex,
            chamfer: bool,
            edges: &[OcBlendEdge],
            laws: &[f64],
            params: &OcFilletParams,
            with_history: bool,
        ) -> Result<UniquePtr<OcOp>>;
        /// The angle (rad, 0..π) between the outward normals of faces `f1` and `f2` at the middle
        /// of edge `e`.
        fn edge_face_angle(index: &OcIndex, e: u32, f1: u32, f2: u32) -> Result<f64>;

        // -- shell, offset and thicken (BREP-07), implemented in occt_offset.cpp --

        /// Shells (removing faces `remove`), offsets or thickens `shape` (`index` is its index)
        /// on a private copy; `faces[i]` gets offset `offsets[i]` instead of `params.offset`. A
        /// failed build is reported by `op_bool_status` (errors, offending faces: those whose
        /// offset reaches their centre of curvature, or that OCCT's input check blames); its op
        /// then holds the result without solid, or a null shape. An error starting with
        /// `invalid: ` is a bad input. Progress and cancellation go through `sink`.
        fn offset_op(
            shape: &OcShape,
            index: &OcIndex,
            params: &OcOffsetParams,
            remove: &[u32],
            faces: &[u32],
            offsets: &[f64],
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;

        // -- pipe-shell sweeps and the raw loft (BREP-15), implemented in occt_sweep.cpp --

        /// `BRepOffsetAPI_MakePipeShell` of `operands` (see `OcPipeParams`) on private copies.
        /// `twist` and `scale` are flat `(t, value)` pairs, `t` in [0, 1] along the spine (empty
        /// for none): the scale law is OCCT's homothetic law of the one profile; the twist (rad,
        /// about the tangent, relative to the base mode) is realised by an auxiliary spine built
        /// here. History per operand: copies `Modified`, `Generated` images; roles start cap, end
        /// cap and lateral. A failure (`invalid: ` for a bad input) or cancellation is reported by
        /// `op_bool_status`, and the op then has no shape. Progress and cancel through `sink`.
        fn pipe_op(
            operands: &OcShapeList,
            params: &OcPipeParams,
            twist: &[f64],
            scale: &[f64],
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;
        /// `BRepOffsetAPI_ThruSections` through `sections` (wires of `wires` by index, or points)
        /// on private copies, with the history of every wire (edges `Generated` into their
        /// lateral faces) and roles: caps loft section (role 8, `role_args` the section index),
        /// every other face lateral. Failures as for `pipe_op`.
        fn thru_sections_op(
            wires: &OcShapeList,
            sections: &[OcLoftSection],
            params: &OcLoftParams,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;

        // -- draft, defeature, replace face, split faces (BREP-16), in occt_local.cpp --

        /// `BRepOffsetAPI_DraftAngle` of `faces` (see `OcDraftParams`) on a private copy, with
        /// the history from `shape` (faces `Modified` 1:1). A face OCCT cannot draft, or a failed
        /// build, is reported by `op_bool_status` (errors, offending faces; `invalid: ` for a face
        /// that is not planar, cylindrical or conical), and the op then has no shape.
        fn draft_op(
            shape: &OcShape,
            index: &OcIndex,
            faces: &[u32],
            params: &OcDraftParams,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;
        /// `BRepAlgoAPI_Defeaturing` removing `faces` on a private copy, with the history from
        /// `shape`. A feature OCCT skips (`BOPAlgo_AlertUnableToRemoveTheFeature`) is a failure
        /// naming its faces (`op_bool_status`).
        fn defeature_op(
            shape: &OcShape,
            index: &OcIndex,
            faces: &[u32],
            parallel: bool,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;
        /// The largest signed distance (mm, along the outward normals) from sample points of
        /// face `face` to the surface of the face `tool`; NaN when no normal line meets it.
        fn face_reach(index: &OcIndex, face: u32, tool: &OcShape) -> Result<f64>;
        /// `shape` split by the face `tool` (on a new surface for face `face`), keeping the
        /// pieces on the material side of the surface. Face `face` is `Modified` into the faces on
        /// the surface. Failures as for `draft_op`.
        fn trim_face_op(
            shape: &OcShape,
            index: &OcIndex,
            face: u32,
            tool: &OcShape,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;
        /// `faces` of `shape` split by the curves of `tool` on them (its edges lying on them, or
        /// its section with them when it has faces), on private copies. Solids are not split.
        /// Failures as for `draft_op` (`invalid: ` when the tool does not meet the faces).
        fn split_faces_op(
            shape: &OcShape,
            index: &OcIndex,
            faces: &[u32],
            tool: &OcShape,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;

        // -- healing and assembly (BREP-10), implemented in occt_heal.cpp --

        /// `ShapeFix_Shape` then `ShapeFix_Wireframe` (wire gaps, small edges) on a private copy
        /// of `shape`, with the history from `shape`; `op_fix_count` gives the number of fixes.
        /// Progress and cancellation (inside `ShapeFix_Shape`) go through `sink`; a failure is
        /// reported by `op_bool_status`.
        fn heal_op(
            shape: &OcShape,
            params: &OcHealParams,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;
        /// `ShapeUpgrade_UnifySameDomain` on a private copy, with the history from `shape`.
        fn unify_same_domain_op(shape: &OcShape, params: &OcUnifyParams)
        -> Result<UniquePtr<OcOp>>;
        /// `BRepBuilderAPI_Sewing` of all `shapes` (one operand each) on a private copy, then
        /// with `make_solid` a solid per closed shell. A failure (an open result with
        /// `make_solid`, an OCCT exception) is reported by `op_bool_status`; the op then holds
        /// the sewn shape, or a null shape.
        fn sew_op(
            shapes: &OcShapeList,
            params: &OcSewParams,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcOp>>;
        /// A solid per closed shell of `shape`, on a private copy. Open shells are an error
        /// starting with `invalid: ` (`op_bool_status`).
        fn make_solid_op(shape: &OcShape) -> Result<UniquePtr<OcOp>>;
        /// A compound of `shapes` (sharing their TShapes).
        fn make_compound_of(shapes: &OcShapeList) -> Result<UniquePtr<OcShape>>;
        /// `BOPAlgo_ArgumentAnalyzer` problems (self-interference, too small edges, curves off
        /// their surfaces), per sub-shape of `index` (kind 3 for anything else).
        fn analyze_shape(shape: &OcShape, index: &OcIndex) -> Result<Vec<OcCheckProblem>>;
        /// The number of fixes a `heal_op` applied.
        fn op_fix_count(op: &OcOp) -> Result<u32>;

        // -- measurement queries (BREP-09), implemented in occt_query.cpp --

        /// Volume (over the closed shells), area, centroid and inertia about the centroid by
        /// adaptive `BRepGProp` integration to the relative error bound `eps`.
        fn mass_props(shape: &OcShape, index: &OcIndex, eps: f64) -> Result<OcMassProps>;
        /// 0 inside some solid of the index, 1 outside all, 2 on a boundary within `tol`.
        /// Raises for a shape without solids.
        fn classify_point(index: &OcIndex, p: &V3, tol: f64) -> Result<u8>;
        /// Minimum distance between two shapes (`BRepExtrema_DistShapeShape`).
        fn distance_shapes(
            a: &OcShape,
            ia: &OcIndex,
            b: &OcShape,
            ib: &OcIndex,
        ) -> Result<OcDistance>;
        /// Nearest point of the shape's boundary to `p` (support on side `a`).
        fn closest_point_on(shape: &OcShape, index: &OcIndex, p: &V3) -> Result<OcDistance>;
        /// Nearest intersection with `t` in `[0, max_t]` of the ray with any face.
        fn ray_cast_faces(
            index: &OcIndex,
            origin: &V3,
            dir: &V3,
            max_t: f64,
            tol: f64,
        ) -> Result<OcRayHit>;
        /// Face `f`'s surface properties at `(u, v)`.
        fn eval_face(index: &OcIndex, f: u32, u: f64, v: f64, tol: f64) -> Result<OcSurfacePoint>;
        /// Nearest point of face `f` (within its boundary) to `p`, with its `(u, v)` in
        /// `u_a`/`v_a`.
        fn project_on_face(index: &OcIndex, f: u32, p: &V3, tol: f64) -> Result<OcDistance>;
        fn edge_ends(index: &OcIndex, e: u32) -> Result<OcEdgeEnds>;

        // -- STEP and IGES through XCAF (BREP-13), implemented in occt_xde.cpp --

        /// What one STEP or IGES read gave. Immutable once built.
        type OcXde;

        /// Reads a STEP file (`path` UTF-8) into plain nodes and prototype shapes, in mm, with
        /// names and colours when asked. Parse progress is the first 20 %, the transfer the rest;
        /// a cancel stops the parser (its stream ends) or the transfer. A failure or cancellation
        /// is reported by `xde_status`. Edges of the read shapes that lack a 3D curve get one.
        fn xde_read_step(
            path: &str,
            names: bool,
            colors: bool,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcXde>>;
        /// As `xde_read_step` for IGES (no assemblies). Not thread-safe: hold the global lock.
        fn xde_read_iges(
            path: &str,
            names: bool,
            colors: bool,
            sink: &ProgressSink,
        ) -> Result<UniquePtr<OcXde>>;
        fn xde_status(xde: &OcXde) -> Result<OcXdeStatus>;
        fn xde_nodes(xde: &OcXde) -> Result<Vec<OcXdeNode>>;
        fn xde_protos(xde: &OcXde) -> Result<Vec<OcXdeProto>>;
        fn xde_face_colors(xde: &OcXde) -> Result<Vec<OcXdeFaceColor>>;
        fn xde_proto_shape(xde: &OcXde, i: u32) -> Result<UniquePtr<OcShape>>;
        /// Writes `nodes` (as read: pre-order, parents first) over the prototypes `protos` (one
        /// `meta` record each) as STEP. A failure or cancellation is reported in the status.
        fn xde_write_step(
            path: &str,
            params: &OcXdeWriteParams,
            protos: &OcShapeList,
            meta: &[OcXdeProto],
            face_colors: &[OcXdeFaceColor],
            nodes: &[OcXdeNode],
            sink: &ProgressSink,
        ) -> Result<OcXdeStatus>;
        /// As `xde_write_step` for IGES (B-rep solids, assemblies flattened). Not thread-safe:
        /// hold the global lock.
        fn xde_write_iges(
            path: &str,
            params: &OcXdeWriteParams,
            protos: &OcShapeList,
            meta: &[OcXdeProto],
            face_colors: &[OcXdeFaceColor],
            nodes: &[OcXdeNode],
            sink: &ProgressSink,
        ) -> Result<OcXdeStatus>;
        /// A copy of `shape` whose edge `edge` lost its 3D curve and got it back from the
        /// readers' repair. Raises if the repair did not rebuild exactly that curve.
        fn xde_guard_probe(shape: &OcShape, edge: u32) -> Result<UniquePtr<OcShape>>;
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
// SAFETY: an `OcXde` is built by one reader call and only read afterwards (`xde_*` accessors take
// `&OcXde`; category R). What it owns (plain vectors, shapes, and after a cancelled STEP read the
// reader with its parsed model) is referenced by nothing else, so it may be dropped on another
// thread; amg-brep does that to keep a cancel fast.
unsafe impl Send for ffi::OcXde {}
