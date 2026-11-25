//! The `BrepKernel` trait, the OCCT backend `OcctKernel`, process init and `kernel_version`
//! (S02 §1, §9, §10; BREP-12).

use std::num::NonZeroUsize;
use std::sync::{Arc, OnceLock};
use std::time::Duration;

use amg_core::{Line3, Plane, Point2, Point3, Ray3, Tolerance, Vec3};
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::backend::{self, Occ};
use crate::blend::{self, BlendEdge};
use crate::boolean::{self, BoolCode, BoolOutcome};
use crate::builders;
use crate::check::{CheckOpts, CheckReport};
use crate::config::KernelConfig;
use crate::error::{KResult, KernelError};
use crate::extract;
use crate::geom::{self, Tag};
use crate::heal;
use crate::history::{FaceBuild, OpResult, ShapeHistory};
use crate::local;
use crate::offset::{self, OffsetIn};
use crate::ops::{self, PrimitiveKind};
use crate::opts::{
    BoolOp, BoolOpts, ChamferSpec, DraftOpts, FilletOpts, FilletSpec, HealOpts, LoftSection,
    OffsetJoin, PipeOpts, PrismExtent, PrismOpts, SewOpts, ShellOpts, TessQuality,
    ThruSectionsOpts, UnifyOpts,
};
use crate::query::{
    self, Classification, DistanceResult, EdgeEnds, MassProps, RayHit, SubShapeGeom, SurfacePoint,
};
use crate::shape::{EdgeId, FORMAT_VERSION, FaceId, Shape, SubKind, SubShape};
use crate::sweep;
use crate::tess::{self, TessMesh, TessProfile};

/// Version of the robustness policy, the third part of [`kernel_version`]. S20's
/// `amg_brep::robust` module takes it over and bumps it when a strategy changes results.
pub const POLICY_VERSION: u32 = 0;

/// `occt-<version>;amg-brep-<FORMAT_VERSION>;policy-<POLICY_VERSION>`, for example
/// `occt-8.0.1;amg-brep-1;policy-0`. Part of S01's `ENGINE_SALT`: any change invalidates
/// cached kernel results. `occt-none` in a build without the `occt` feature.
pub fn kernel_version() -> &'static str {
    static VERSION: OnceLock<String> = OnceLock::new();
    VERSION.get_or_init(|| {
        format!(
            "{};amg-brep-{FORMAT_VERSION};policy-{POLICY_VERSION}",
            backend::occt_version()
        )
    })
}

/// What the first [`init`] call set up.
#[derive(Debug)]
struct InitState {
    /// OCCT pool threads as requested.
    requested: NonZeroUsize,
}

static INIT: OnceLock<InitState> = OnceLock::new();

/// Idempotent process init: routes OCCT messages to `tracing` (target `occt`, nothing to
/// stdout) and sizes OCCT's thread pool to [`KernelConfig::effective_occt_threads`].
///
/// Only the first call has an effect; a later call asking for a different pool size is logged
/// and ignored. [`OcctKernel::new`] calls it. Without the `occt` feature it does nothing.
pub fn init(cfg: &KernelConfig) {
    let wanted = cfg.effective_occt_threads();
    let state = INIT.get_or_init(|| {
        let _guard = crate::lock::global_io_lock();
        backend::init_process(wanted);
        InitState { requested: wanted }
    });
    if state.requested != wanted {
        tracing::debug!(
            "amg_brep::init: OCCT pool already sized to {} threads; ignoring {wanted}",
            state.requested
        );
    }
}

/// The B-rep kernel. Object-safe, so features take `&dyn BrepKernel` and S20's decorators
/// (`RecordingKernel<K>`, `HardenedKernel<K>`) wrap any implementation (S02 §10).
///
/// Rules for every method (S02 §2, §10, S20 request R3):
/// - inputs are only `&Shape` / `&[Shape]` and are never modified;
/// - every other parameter is an op parameter type (`Clone + Debug + PartialEq + Serialize +
///   DeserializeOwned`, see [`crate::opts`]);
/// - ops that run long take `&JobCtx` for progress and cancellation.
///
/// Each op WP adds its methods here together with S20's `KernelOp` variant and the forwarding
/// arm in `tests/api_contract.rs`, which keeps the trait forwardable.
pub trait BrepKernel: Send + Sync {
    /// `BRepCheck_Analyzer` validity of `s`, with problems per sub-shape (S02 §9): the
    /// `BRepCheck_Status` names without their prefix (`NotClosed`, `InvalidPointOnCurve`, ...).
    ///
    /// With `opts.self_interference` (slower; S20 request R5) `BOPAlgo_ArgumentAnalyzer` runs
    /// too, on a private copy, with its self-interference, small-edge and curve-on-surface tests:
    /// its findings carry the full `BOPAlgo_CheckStatus` names (`BOPAlgo_SelfIntersect` on each of
    /// the interfering sub-shapes, `BOPAlgo_TooSmallEdge`, `BOPAlgo_InvalidCurveOnSurface`), and
    /// any of them makes the report invalid. An analyzer that raises adds `BOPAlgo_CheckFail`.
    /// The validation policy ([`crate::Validate`]) runs the `BRepCheck` part only.
    fn check(&self, s: &Shape, opts: &CheckOpts) -> CheckReport;

    // -- builders (BREP-03) --

    /// One planar face per profile, built in one backend call (S02 §4). Each [`FaceBuild`]
    /// lists `(edge, tag)` for every profile edge, outer loop first, then the holes in order.
    ///
    /// Loops are validated first: every curve well-formed, each loop closed within
    /// `tol.linear`, the outer loop CCW and holes CW about `plane.normal`, holes inside the
    /// outer loop. Holes overlapping each other are not detected.
    ///
    /// # Errors
    /// `OpenLoop { loop_index, edge_index, gap }` for the first open loop (of the first
    /// failing profile); `InvalidInput { what: "profiles", .. }` naming profile, loop and edge
    /// for malformed curves, a wrong orientation or a hole outside its outer loop; `Occt` if
    /// OCCT rejects a curve; `InvalidResult` under the validation policy.
    fn make_planar_faces(
        &self,
        profiles: &[geom::PlanarProfile],
        tol: &Tolerance,
    ) -> KResult<Vec<FaceBuild>>;

    /// A planar face bounded by straight edges through `outer` and `holes` (S07 MESH-13).
    /// Consecutive points within `tol.linear` merge (a repeated closing point is fine), the
    /// plane is fitted through the outer loop (Newell normal, so the outer loop runs CCW about
    /// the face normal) and every vertex is projected onto it. Holes may run either way.
    ///
    /// # Errors
    /// `NotPlanar { max_dev }` when a point is farther than `tol.linear` from the fitted plane;
    /// `InvalidInput` for fewer than 3 distinct points, no area, or a hole outside the outer
    /// loop.
    fn make_planar_polygon(
        &self,
        outer: &[Point3],
        holes: &[Vec<Point3>],
        tol: &Tolerance,
    ) -> KResult<Shape>;

    /// A face on an analytic or B-spline surface (S07 MESH-25, S09, S20). `tol` (mm) is the
    /// distance within which loop curves lie on the surface and polylines are approximated.
    /// `edge_tags` is filled for `Loops` bounds (from each [`geom::CurveSeg::tag`]) and empty
    /// otherwise.
    ///
    /// # Errors
    /// `InvalidInput` for `Surface::Other`, malformed surface data, `Natural` bounds on an
    /// infinite surface (plane, cylinder, cone) or an empty `UvRect`; `OpenLoop` for a `Loops`
    /// loop that does not close within `tol`; `Occt` when OCCT cannot build it.
    fn make_face(
        &self,
        surface: &geom::Surface,
        bounds: &geom::FaceBounds,
        tol: f64,
    ) -> KResult<FaceBuild>;

    /// A wire through `segs` in order (sweep paths, rails). Consecutive segments must meet
    /// within `tol.linear`; the wire is closed when the last ends at the first's start. The
    /// history carries each edge's tag.
    ///
    /// # Errors
    /// `InvalidInput` for an empty list, a malformed segment (`Curve3::Other`, a periodic
    /// B-spline, an empty or out-of-range parameter range) or a gap.
    fn make_wire(&self, segs: &[geom::CurveSeg], tol: &Tolerance) -> KResult<OpResult>;

    // -- primitives, sweeps and transforms (BREP-14) --

    /// A box with one corner at `frame.origin`, `size` along `frame.x`, `frame.y` and
    /// `frame.normal` (S02 §5). The history has no relations (a primitive has no input) but a
    /// role on every face (BREP-05): the bottom `StartCap`, the top `EndCap`, the sides
    /// `Lateral`; the other primitives likewise (`ops::primitive_roles`).
    ///
    /// # Errors
    /// `InvalidInput` for a frame that is not orthonormal and right-handed or a size not larger
    /// than the linear tolerance; `InvalidResult` under the validation policy.
    fn make_box(&self, frame: &Plane, size: Vec3) -> KResult<OpResult>;

    /// A cylinder of `radius` on `frame` (base centre at `frame.origin`, axis `frame.normal`),
    /// `height` high, swept by `angle` rad in `(0, 2π]` from `frame.x` (CCW about the normal).
    ///
    /// # Errors
    /// As [`BrepKernel::make_box`], also for an angle outside `(0, 2π]`.
    fn make_cylinder(
        &self,
        frame: &Plane,
        radius: f64,
        height: f64,
        angle: f64,
    ) -> KResult<OpResult>;

    /// A cone frustum on `frame`: radius `r1` at `frame.origin`, `r2` at `height` along the
    /// normal (one may be 0 for a full cone), swept by `angle` rad from `frame.x`.
    ///
    /// # Errors
    /// As [`BrepKernel::make_cylinder`], also for equal radii (a cylinder).
    fn make_cone(
        &self,
        frame: &Plane,
        r1: f64,
        r2: f64,
        height: f64,
        angle: f64,
    ) -> KResult<OpResult>;

    /// A sphere of `radius` about `frame.origin`; its poles lie on `frame.normal` and its seam
    /// in the `frame.x` half-plane.
    ///
    /// # Errors
    /// As [`BrepKernel::make_box`].
    fn make_sphere(&self, frame: &Plane, radius: f64) -> KResult<OpResult>;

    /// A torus about `frame.normal` through `frame.origin`, swept by `angle` rad from
    /// `frame.x`. Needs `major > minor` (no self-intersecting spindle tori).
    ///
    /// # Errors
    /// As [`BrepKernel::make_cylinder`], also for `major ≤ minor`.
    fn make_torus(&self, frame: &Plane, major: f64, minor: f64, angle: f64) -> KResult<OpResult>;

    /// `profile` (a face, shell, wire, edge or a compound of them) swept along a straight line
    /// (`BRepPrimAPI_MakePrism`): by a vector, or infinitely one or both ways. A face gives a
    /// solid, a wire a shell. Prisms infinite both ways skip the validation policy: BRepCheck
    /// rejects every such solid (no cap closes it).
    ///
    /// - `opts.taper` (rad, finite prisms of one planar face only) tilts every side face about
    ///   the profile plane (`BRepOffsetAPI_DraftAngle` on the straight prism): positive grows
    ///   the profile along the sweep, so holes shrink; negative shrinks it. Side faces must be
    ///   planar, cylindrical or conical (profiles of lines and circle arcs).
    /// - `opts.merge_coplanar` merges coplanar faces and collinear edges of the result
    ///   (`ShapeUpgrade_UnifySameDomain`), with the history composed through it.
    ///
    /// History (operand 0 = `profile`): each profile face is `Modified` into the start cap and
    /// `Generated` into the end cap; each edge `Modified` into its start-cap edge and
    /// `Generated` into its side face(s) and end-cap edge; each vertex likewise into its
    /// start vertex, side edge and end vertex. Roles `StartCap`, `EndCap` and `Lateral`.
    /// Cancellation is checked before and after the OCCT call.
    ///
    /// # Errors
    /// `InvalidInput { what: "profile" | "extent" | "taper" }`; `Occt` when OCCT fails (for
    /// example a taper on a B-spline side face); `InvalidResult` under the validation policy
    /// (for example an inward taper that collapses the profile); `Cancelled`.
    fn prism(
        &self,
        profile: &Shape,
        extent: &PrismExtent,
        opts: &PrismOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// `profile` revolved about `axis` by `angle` rad in `(0, 2π]`, counter-clockwise about
    /// `axis.dir` (`BRepPrimAPI_MakeRevol`). History and roles as for [`BrepKernel::prism`]; a
    /// full turn has no caps. Edges on the axis sweep nothing. A profile crossing the axis gives
    /// `InvalidResult` under the validation policy.
    ///
    /// # Errors
    /// `InvalidInput { what: "profile" | "axis" | "angle" }`; `Occt`; `InvalidResult`;
    /// `Cancelled`.
    fn revol(&self, profile: &Shape, axis: &Line3, angle: f64, ctx: &JobCtx) -> KResult<OpResult>;

    /// Profiles swept along a spine (`BRepOffsetAPI_MakePipeShell`), on private copies of the
    /// operands. `operands` are `[profiles..., spine, aux?]`: `profiles` leading profiles (wires
    /// or edges; the first and last may be vertices, punctual sections), the spine (a wire or
    /// an edge), and for [`crate::PipeMode::Guide`] / [`crate::PipeMode::Support`] the guide
    /// wire or support shell, whose operand index is the one after the spine.
    ///
    /// - `opts.mode`: corrected Frenet, Frenet, fixed (sections stay parallel: they are only
    ///   translated), constant binormal, a guide wire (plane mode; `contact` keeps the section
    ///   on the guide), or a support shell whose own (curved) edges the spine is (S04 R-S04-7).
    /// - `opts.twist` turns the section about the tangent by `twist(t) − twist(0)` rad relative
    ///   to the mode's trihedron (`t` in 0..1 along the spine). OCCT has no twist law: the shim
    ///   builds an auxiliary spine around the spine at the twist angles (64+ samples, cubic
    ///   interpolation) and sweeps in guide mode. Corrected Frenet, Frenet and binormal only.
    /// - `opts.scale` scales the one profile about the spine point (OCCT's homothetic law).
    ///   OCCT cannot combine it with a guide, a support or a twist.
    /// - `opts.solid` closes a sweep of closed profiles with planar caps (`MakeSolid`);
    ///   otherwise the result is the swept shell.
    /// - `opts.transition`: how sections turn at spine corners.
    ///
    /// History, one entry of `operand_counts` per operand: profile edges `Generated` into the
    /// lateral faces they sweep, profile vertices into the lateral edges; a profile edge that is
    /// a boundary of the result (the first section) is `Modified` into it. Roles: the cap faces
    /// `StartCap` / `EndCap`, every other face `Lateral`. Cancellation reaches inside the build.
    ///
    /// # Errors
    /// `InvalidInput { what: "profiles" | "operands" | "spine" | "guide" | "support" | "mode" |
    /// "twist" | "scale" }`; `OpFailed { op: "pipe" }` when OCCT fails (with the
    /// `BRepBuilderAPI_PipeError` name or the exception), also when `MakeSolid` fails on open
    /// profiles; `InvalidResult` under the validation policy; `Cancelled`.
    fn pipe(
        &self,
        operands: &[Shape],
        profiles: usize,
        opts: &PipeOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// The raw loft through `sections` (`BRepOffsetAPI_ThruSections`) on private copies of
    /// `wires`: each [`LoftSection::Wire`] names a wire operand (a wire or an edge), each
    /// [`LoftSection::Point`] a punctual first or last section (a cone tip). A wire is used once,
    /// except that the first one may be repeated as the last section to close the loft.
    /// `opts.ruled` builds ruled faces between neighbours, else one smooth B-spline face per
    /// edge column; `check_compatibility` lets OCCT orient the wires alike and split edges to
    /// match their counts; `max_degree` 0 keeps OCCT's 8. This is the baseline that S20's KH-05
    /// measures and the fallback of KH-08's loft chain.
    ///
    /// History, one entry of `operand_counts` per wire: each section edge `Generated` into the
    /// lateral face(s) it bounds, section vertices into the lateral edges; first and last
    /// section edges that stay boundaries are `Modified` into them. Roles: the cap faces
    /// `LoftSection { index: 0 }` and `LoftSection { index: n − 1 }`, every other face `Lateral`.
    /// Cancellation reaches inside the build.
    ///
    /// # Errors
    /// `InvalidInput { what: "sections" | "wires" | "opts" }`; `OpFailed { op: "thru_sections" }`
    /// when OCCT fails or raises (incompatible sections: open and closed wires mixed, different
    /// edge counts without the compatibility check, ...); `InvalidResult` under the validation
    /// policy; `Cancelled`.
    fn thru_sections(
        &self,
        wires: &[Shape],
        sections: &[LoftSection],
        opts: &ThruSectionsOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// `s` reflected about `plane` into new geometry (S04 R-S04-6), faces still facing out of
    /// the material; history `Modified` 1:1 ([`Shape::gtransformed`]). [`Shape::transformed`]
    /// keeps rejecting reflections with `NonRigid`.
    ///
    /// # Errors
    /// `InvalidInput { what: "frame" }` for a bad plane; `Occt`; `InvalidResult`.
    fn mirrored(&self, s: &Shape, plane: &Plane) -> KResult<OpResult>;

    // -- booleans (BREP-04) --

    /// `objects` fused with, cut by or intersected with `tools` (`BRepAlgoAPI_BooleanOperation`;
    /// N objects × M tools). A fuse may leave `tools` empty and fuse the objects with each other
    /// (it needs two shapes in all); cut and common need a tool.
    ///
    /// Always non-destructive (category N): the inputs are never modified. `opts.fuzzy`,
    /// `glue`, `check_inverted` and `use_obb` go to BOPAlgo as is; `opts.parallel` (default
    /// `KernelConfig::bool_parallel`, off with a one-thread OCCT pool) turns on OCCT's internal
    /// parallelism, which uses the OCCT thread pool (`KernelConfig::occt_threads`). Progress and
    /// cancellation are mid-op, through OCCT's progress indicator.
    ///
    /// History: operands are numbered objects first, then tools. Every face, edge and vertex of
    /// every operand is `Kept` (it is in the result unchanged), `Modified` (split or trimmed),
    /// `Generated` (a face into the section edges on it, an edge into new vertices) or
    /// `Deleted`. `operand_counts` has one entry per operand. Warnings: `FuzzyUsed` when
    /// `fuzzy > 0`, `ToleranceGrown` when the result's largest tolerance exceeds every input's
    /// and the kernel's linear tolerance. The result is a compound when it has several solids
    /// (or none: an empty result is a valid empty compound).
    ///
    /// # Errors
    /// `InvalidInput { what: "objects" | "tools" | "fuzzy" }`; `OpFailed { op: "boolean" }`
    /// with the BOPAlgo alert names in `message` (and the operands they name) and the alert
    /// sub-shapes in `offending`, for example `BOPAlgo_AlertBOPNotAllowed` for a fuse of a solid
    /// and a face; `InvalidResult` under the validation policy; `Cancelled`; `Occt`.
    fn boolean(
        &self,
        op: BoolOp,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// `objects` split by `tools` (`BRepAlgoAPI_Splitter`; with no tools, the objects split each
    /// other): every piece is kept, so a solid cut by a face gives a compound of two solids.
    /// Options, history, warnings and cancellation as for [`BrepKernel::boolean`].
    ///
    /// # Errors
    /// As [`BrepKernel::boolean`], with `op: "split"`.
    fn split(
        &self,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// The intersection curves of `a` and `b` (`BRepAlgoAPI_Section`, approximated, no
    /// pcurves, non-destructive): a compound of edges and vertices, empty when they do not
    /// meet. History: each face of `a` (operand 0) and `b` (operand 1) `Generated` into the
    /// section edges on it, edges and vertices that are in the result `Kept`; every edge has the
    /// `SectionEdge` role.
    ///
    /// # Errors
    /// `OpFailed { op: "section" }`; `Cancelled`; `Occt`.
    fn section(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<OpResult>;

    // -- local ops (BREP-06) --

    /// `s` with `edges` rounded (`BRepFilletAPI_MakeFillet` on a private copy, category C).
    ///
    /// Each edge must lie between two distinct faces of `s` and be listed once. OCCT extends an
    /// edge along its tangent chain into a *contour*; contours are numbered in the order of the
    /// first edge of `edges` on them. An edge already on an earlier edge's chain is accepted
    /// when its spec is the same, and refused otherwise. Specs:
    /// - `Constant(r)`: radius `r`;
    /// - `Variable(law)`: `(t, r)` samples with `t` the relative arc length along the contour,
    ///   from exactly 0 to exactly 1 and strictly increasing (OCCT interpolates between them);
    /// - `Chord(c)`: the constant radius whose circular section has chord `c` between the two
    ///   tangent lines, `r = c / (2 sin(φ/2))` with `φ` the angle between the faces' outward
    ///   normals at the middle of the edge (exact for edges of constant dihedral angle).
    ///
    /// `opts.shape` is the blend surface's `ChFi3d_FilletShape`; `tol_app_3d` and `fleche`
    /// override OCCT's approximation parameters (`SetParams`), `None` keeps its defaults.
    /// Cancellation is checked before and after the call only.
    ///
    /// History (operand 0 = `s`): faces trimmed by the blend are `Modified`, untouched faces and
    /// edges `Modified` 1:1 (they pass through the private copy, so they are new TShapes),
    /// blended edges `Generated` into their blend faces and vertices into the corner faces OCCT
    /// attributes to them. Every blend face has `OpRole::FilletBlend { contour }`; every other
    /// new face (a corner patch between contours) `OpRole::CornerPatch`.
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" | "edges" | "opts" }` (with the edge in `offending` where
    /// one is to blame); `LocalOpFailed { op: "fillet" }` when OCCT cannot build the blend:
    /// `offending` lists the input edges of the faulty contours (`NbFaultyContours`) and of the
    /// failing corners (`NbFaultyVertices`), or, when OCCT names none, the edges of every
    /// contour that fails on its own (all edges when they only fail together); `message` has
    /// the stripe status per contour (`ChFiDS_StartsolFailure`, ...) and OCCT's exception;
    /// `partial` holds OCCT's partial result (`BadShape`) for display only. `InvalidResult`
    /// under the validation policy; `Cancelled`; `Occt`.
    fn fillet(
        &self,
        s: &Shape,
        edges: &[(EdgeId, FilletSpec)],
        opts: &FilletOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// `s` with `edges` bevelled (`BRepFilletAPI_MakeChamfer` on a private copy, category C).
    /// Edges, contours and cancellation as for [`BrepKernel::fillet`]. Specs: `Equal(d)`,
    /// `TwoDistances { d1, d2, face }` (`d1` measured on `face`, which must be one of the edge's
    /// two faces, `d2` on the other), `DistanceAngle { d, angle, face }` (`d` on `face`, the
    /// chamfer plane at `angle` ∈ (0, π/2) rad from it, so `d·tan(angle)` on the other face).
    /// History as for `fillet`, with `OpRole::ChamferFace` on the chamfer faces.
    ///
    /// # Errors
    /// As [`BrepKernel::fillet`], with `op: "chamfer"`; OCCT gives no fault list for chamfers,
    /// so `offending` is always found by building each contour on its own.
    fn chamfer(
        &self,
        s: &Shape,
        edges: &[(EdgeId, ChamferSpec)],
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// The largest constant radius `r ≤ hi` that fillets all of `edges` of `s` into a valid
    /// (BRepCheck) shape, by bisection: `hi` itself when it works, else `iterations` halvings of
    /// `(0, hi]`, so the answer is within `hi / 2^iterations` below the boundary of the workable
    /// range (S20 KH-03). Assumes the workable radii form one interval from 0; cancellation is
    /// checked between steps.
    ///
    /// # Errors
    /// `InvalidInput` as for `fillet`, or for `hi` not finite and positive or `iterations` 0 or
    /// above 64; `LocalOpFailed { op: "fillet_max_radius" }` when no tried radius works
    /// (`offending` from the smallest try); `Cancelled`; `Occt`.
    fn fillet_max_radius(
        &self,
        s: &Shape,
        edges: &[EdgeId],
        hi: f64,
        iterations: u32,
        ctx: &JobCtx,
    ) -> KResult<f64>;

    // -- shell, offset, thicken (BREP-07) --

    /// The thick solid of `s` (one solid) with walls of `opts.thickness` along the outward
    /// normal (negative: the walls grow inward and `s`'s faces stay the outside; positive: they
    /// grow outward), open at the faces in `remove` (`BRepOffset_MakeOffset::MakeThickSolid` on
    /// a private copy, category C). An empty `remove` gives a hollow solid with a closed inner
    /// void (S04 R-S04-8): the input (one shell) and its offset assembled into one solid.
    /// `opts.per_face` gives single faces their own thickness, of the same sign. `opts.join`
    /// closes the gaps that open between offset faces at edges convex in the
    /// offset direction: `Arc` with round pipes and spheres, `Intersection` by extending the
    /// faces to sharp edges. `opts.tol` is the offset algorithm's confusion tolerance (0: the
    /// kernel tolerance). Progress and cancellation reach inside the build.
    ///
    /// Every face that is offset is checked first: an offset of `d` towards a centre of
    /// curvature closer than `|d|` (a convex radius smaller than an inward wall, a concave one
    /// smaller than an outward wall) would collapse the offset face, and is refused.
    ///
    /// History (operand 0 = `s`): `s`'s faces, edges and vertices are `Modified` 1:1 into the
    /// result (they pass through the private copy, so they are new TShapes); each offset face is
    /// `Generated` from its source face, arc-join faces from their edge or vertex. A removed face
    /// has no `Modified` relation; the rim at the opening is `Generated` from it. Offset and arc
    /// faces have `OpRole::ShellInner` (the inner wall of an inward shell; the outer one of an
    /// outward shell); the rims and any other new face have `OpRole::Lateral`. Face order is
    /// canonical (S02 §7: shells rebuilt with faces by area, then centroid), so indices and
    /// bytes repeat from run to run.
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" | "remove" | "opts" | "tol" }` (a face named twice or out
    /// of range, every face removed, a zero or non-finite thickness, a per-face thickness of the
    /// other sign or on a removed face); `LocalOpFailed { op: "shell" }` naming the faces whose
    /// offset collapses (`message` gives the radius), or OCCT's `BRepOffset_Error` and the
    /// faces its input check blames (`offending` may then be empty), with the result as
    /// `partial` when OCCT made one without a solid; `InvalidResult` under the validation
    /// policy; `Cancelled`; `Occt`.
    fn shell(
        &self,
        s: &Shape,
        remove: &[FaceId],
        opts: &ShellOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// `s` (faces, shells, solids or a compound of them) with every face moved `distance` along
    /// its outward normal (negative: inward), gaps closed by `join` as for
    /// [`BrepKernel::shell`] (`BRepOffset_MakeOffset::MakeOffsetShape` on a private copy). A
    /// solid gives a solid; the input must be connected (two loose solids fail with
    /// `BRepOffset_NotConnectedShell`). `tol` as `ShellOpts::tol`. The collapse check, progress,
    /// cancellation and face order as for `shell`.
    ///
    /// History: each face is `Modified` into its offset face, arc faces are `Generated` from
    /// their edge or vertex; every output face has `OpRole::OffsetFace`.
    ///
    /// # Errors
    /// As [`BrepKernel::shell`], with `what: "shape" | "distance" | "tol"` and
    /// `op: "offset"`.
    fn offset(
        &self,
        s: &Shape,
        distance: f64,
        join: OffsetJoin,
        tol: f64,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// The solid swept by the face or shell `s` moving `thickness` along its normal (negative:
    /// against it): `s`, its offset and the side walls between their boundaries
    /// (`BRepOffset_MakeOffset` with thickening, on a private copy). `join` as for
    /// [`BrepKernel::shell`]; the collapse check, progress and cancellation as for `shell`.
    ///
    /// History: `s`'s faces are `Modified` 1:1, their offsets `Generated` from them with
    /// `OpRole::OffsetFace`; the side walls have `OpRole::Lateral` (and are `Generated` from
    /// the boundary edges where OCCT says so).
    ///
    /// # Errors
    /// As [`BrepKernel::shell`], with `what: "shape" | "thickness"` (a solid is refused) and
    /// `op: "thicken"`.
    fn thicken(
        &self,
        s: &Shape,
        thickness: f64,
        join: OffsetJoin,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    // -- draft, defeature, replace face, split faces, offset faces (BREP-16) --

    /// `s` with `faces` (planar, cylindrical or conical) tilted by `opts.angle` about their
    /// intersection with `opts.neutral` (`BRepOffsetAPI_DraftAngle` on a private copy, category
    /// C). A positive angle removes material on the `opts.pull` side of the neutral plane (the
    /// part narrows along `pull`, as a moulded part pulled out along `pull`); a negative one adds
    /// material there. A drafted planar face makes the angle `opts.angle` with `pull` (its
    /// outward normal · pull = sin angle for a face parallel to `pull`); a cylinder becomes a
    /// cone. OCCT propagates the draft along faces tangent to a given one. The neighbours are
    /// re-intersected with the tilted faces. Cancellation is checked before and after the build.
    ///
    /// History (operand 0 = `s`): every face, edge and vertex is `Modified` 1:1 (through the
    /// copy; a drafted face into its tilted face). No roles.
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" | "faces" | "opts" }` (no face, a face twice or out of
    /// range, an angle outside (−π/2, π/2), a bad neutral frame, `pull` in the neutral plane, a
    /// face that is not planar, cylindrical or conical); `LocalOpFailed { op: "draft" }` naming
    /// the faces OCCT cannot tilt (`Draft_FaceRecomputation`, ...); `InvalidResult` under the
    /// validation policy; `Cancelled`; `Occt`.
    fn draft(
        &self,
        s: &Shape,
        faces: &[FaceId],
        opts: &DraftOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// `s` (solids) with `faces` removed and the gap closed by extending the faces around them
    /// (`BRepAlgoAPI_Defeaturing` on a private copy, category C): holes, bosses, fillets,
    /// chamfers. Give every face of a feature (a boss's wall and top). Progress and cancellation
    /// reach inside the build.
    ///
    /// History (operand 0 = `s`): the removed faces, and edges and vertices that vanish with
    /// them, are `Deleted`; the extended neighbours are `Modified` (1:1 when untouched, through
    /// the copy).
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" | "faces" }` (no solid, no face, a face twice or out of
    /// range, every face); `LocalOpFailed { op: "defeature" }` naming the faces of a feature OCCT
    /// cannot remove (`BOPAlgo_AlertUnableToRemoveTheFeature`), whose neighbours cannot be
    /// extended to close the gap; `InvalidResult`; `Cancelled`; `Occt`.
    fn defeature(&self, s: &Shape, faces: &[FaceId], ctx: &JobCtx) -> KResult<OpResult>;

    /// `s` (solids) with `face` replaced by a face on the surface `with`: its neighbours are
    /// extended or trimmed to meet the new surface. Two steps: when `with` lies beyond the face
    /// (along its outward normals), the face is first pushed past it with
    /// [`BrepKernel::offset_faces`] so the neighbours extend; then the body is split by a large
    /// face on `with` (`BRepAlgoAPI_Splitter`) and the pieces beyond it are dropped. The material
    /// stays on the side of `with` opposite to the face's outward normal. `with` must cross the
    /// lines along the face's normals (a plane through the body, a cylinder around it, ...).
    ///
    /// History (operand 0 = `s`, the two steps composed): `face` is `Modified` into the faces
    /// on the new surface, its edges into the edges where those meet the face's neighbours'
    /// images; the neighbours are `Modified` (trimmed or extended); whatever the trim drops is
    /// `Deleted`.
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" | "face" | "with" }` (no solid, a face out of range,
    /// `Surface::Other`, a surface the face's normals do not meet); `LocalOpFailed { op:
    /// "offset_faces" | "replace_face" }` when the extension or the trim fails (nothing left on
    /// the material side); `InvalidResult`; `Cancelled`; `Occt`.
    fn replace_face(
        &self,
        s: &Shape,
        face: FaceId,
        with: &geom::Surface,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// `s` with each of `faces` split by the curves of `tool` on it (on private copies,
    /// category C): the edges of an edge or wire `tool` that lie on the faces (a sketch curve on
    /// a face; edges reaching past a face's boundary are cut there), or, for a `tool` with faces,
    /// its section with the faces (`BRepAlgoAPI_Section`). The curves are imprinted with
    /// `BRepAlgoAPI_Splitter`; a solid stays one solid. A curve must cross a face from boundary
    /// to boundary or close on itself to split it. A neighbour on which the same curves lie (a
    /// curve along a shared edge) is split as well.
    ///
    /// History (operand 0 = `s`): a split face is `Modified` into its pieces (split order:
    /// area descending), boundary edges cut by a curve into their parts; everything else
    /// `Modified` 1:1 through the copy.
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" | "faces" | "tool" }` (no face, a face twice or out of
    /// range, a null tool or one without edges) and `InvalidInput { what: "faces" }` when the
    /// tool does not meet or does not split the faces; `LocalOpFailed { op: "split_faces" }`
    /// with the BOPAlgo alert names; `InvalidResult`; `Cancelled`; `Occt`.
    fn split_faces(
        &self,
        s: &Shape,
        faces: &[FaceId],
        tool: &Shape,
        ctx: &JobCtx,
    ) -> KResult<OpResult>;

    /// `s` with each listed face moved by its own distance along its outward normal (negative:
    /// inward), the other faces extended or trimmed to meet the moved ones (S04 R-S04-5: offset
    /// face and press-pull). `BRepOffset_MakeOffset` with offset 0 and per-face offsets,
    /// intersection join, on a private copy (category C); the collapse check, progress,
    /// cancellation and canonical face order as for [`BrepKernel::shell`].
    ///
    /// History (operand 0 = `s`): every face is `Modified` into its image (moved, extended or
    /// trimmed); the moved faces have `OpRole::OffsetFace`.
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" | "faces" }` (no face, a face twice or out of range, a zero
    /// or non-finite distance); `LocalOpFailed { op: "offset_faces" }` as for `shell`;
    /// `InvalidResult`; `Cancelled`; `Occt`.
    fn offset_faces(&self, s: &Shape, faces: &[(FaceId, f64)], ctx: &JobCtx) -> KResult<OpResult>;

    // -- healing and assembly (BREP-10) --

    /// `s` healed (S02 §9): `ShapeFix_Shape` (missing or bad pcurves and 3D curves,
    /// `SameParameter`, vertex and edge tolerances, wire order, gaps and self-intersections, face,
    /// shell and solid orientation), then `ShapeFix_Wireframe` (wire gaps, and with
    /// `opts.fix_small_edges` edges shorter than `opts.tol` merged into their neighbours or
    /// dropped), on a private copy with its geometry (category C). `opts.tol` is the fixes'
    /// precision; no tolerance grows beyond `opts.max_tol`. `opts.fix_orientation` false keeps
    /// every orientation as it is. Progress and cancellation reach inside `ShapeFix_Shape`; the
    /// wireframe pass is checked before and after.
    ///
    /// History (operand 0 = `s`): every face, edge and vertex is `Modified` into its fixed version
    /// (1:1 when the fix left it alone: it passes through the copy), into several when a fix split
    /// it, and `Deleted` when a fix removed it (a dropped small edge, a degenerated face). Merged
    /// small edges are all `Modified` into the merged edge. Warnings: `Healed { fixes }` with the
    /// number of sub-shapes a fix replaced, removed or re-toleranced (none when `s` needed no
    /// fix), `ToleranceGrown` when the result's largest tolerance exceeds `s`'s and the kernel's.
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" | "opts" }` (a null shape, a non-positive `tol`, `max_tol`
    /// below `tol`); `LocalOpFailed { op: "heal" }` when a fix raises; `InvalidResult` under the
    /// validation policy when the result is still invalid; `Cancelled`; `Occt`.
    fn heal(&self, s: &Shape, opts: &HealOpts, ctx: &JobCtx) -> KResult<OpResult>;

    /// `s` with same-domain faces (on one surface) and edges (on one curve) merged
    /// (`ShapeUpgrade_UnifySameDomain` on a private copy, category C): `opts.faces`,
    /// `opts.edges`, `opts.concat_bsplines` (merge the B-spline edges of one smooth chain into
    /// one B-spline), `opts.lin_tol` (mm) and `opts.ang_tol` (rad) are the algorithm's options.
    /// The defaults are those of [`PrismOpts::merge_coplanar`]. No progress or cancellation (one
    /// fast pass).
    ///
    /// History (operand 0 = `s`): each merged face or edge is `Modified` into the merged one, the
    /// others `Modified` 1:1 (through the copy); edges and vertices that vanish inside a merged
    /// face or edge are `Deleted`.
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" | "opts" }` (a null shape, a negative or non-finite
    /// `lin_tol`, `ang_tol` outside `[0, π)`); `InvalidResult` under the validation policy;
    /// `Occt`.
    fn unify_same_domain(&self, s: &Shape, opts: &UnifyOpts) -> KResult<OpResult>;

    /// `faces` (faces, shells, or any shapes with faces: one operand each) sewn into shells
    /// (`BRepBuilderAPI_Sewing` with cutting and degenerated-face analysis, on one private copy
    /// of all of them, category C): boundary edges within `opts.tol` of each other are merged, and
    /// sub-shapes the inputs already share stay shared. `opts.non_manifold` allows edges with more
    /// than two faces. With `opts.make_solid` every closed shell becomes a solid, oriented to
    /// bound a finite volume (as [`BrepKernel::make_solid`]); the result is the solid, or a
    /// compound of solids. Without it the result is OCCT's sewn shape: a shell, a face, or a
    /// compound of shells and free faces. Progress and cancellation reach inside the sewing.
    ///
    /// History (operands in `faces` order): every face, edge and vertex is `Modified` into its
    /// sewn version (1:1 for a face; two edges merged into one both map to it; an edge the sewing
    /// cut into sections maps to each of them); degenerated faces the sewing removed are
    /// `Deleted`.
    ///
    /// # Errors
    /// `InvalidInput { what: "faces" | "opts" }` (no shapes, a shape without faces, more than
    /// 65 536 shapes, a non-positive `tol`); `LocalOpFailed { op: "sew" }` when `make_solid` is
    /// set and the sewn shape has an open shell or a free face (`message` gives their count and
    /// the number of free edges; `partial` is the sewn shape); `InvalidResult` under the
    /// validation policy; `Cancelled`; `Occt`.
    fn sew(&self, faces: &[Shape], opts: &SewOpts, ctx: &JobCtx) -> KResult<OpResult>;

    /// A solid per closed shell of `shell` (a shell, or a compound of shells, without solids; a
    /// face that closes on itself, a sphere or a torus, counts as a shell), oriented to bound a
    /// finite volume: `BRepBuilderAPI_MakeSolid`, reversed when the point at
    /// infinity classifies inside (`ShapeFix_Solid::SolidFromShell`), on a private copy
    /// (category C). One solid comes back as such, several as a compound. History (operand 0):
    /// everything `Modified` 1:1.
    ///
    /// # Errors
    /// `InvalidInput { what: "shell" }` for a null shape, one without faces, one with solids, or
    /// one with an open shell or an open face outside any shell; `InvalidResult` under the
    /// validation policy; `Occt`.
    fn make_solid(&self, shell: &Shape) -> KResult<OpResult>;

    /// A compound of `parts` in order, sharing their sub-shapes (category N: no copy). An empty
    /// list gives an empty compound. Its indices follow `TopExp::MapShapes` over the parts in
    /// order, so a sub-shape that two parts share is indexed once.
    ///
    /// # Errors
    /// `InvalidInput { what: "parts" }` for a null part; `BackendUnavailable`.
    fn make_compound(&self, parts: &[Shape]) -> KResult<Shape>;

    // -- geometry extraction (BREP-17) --

    /// `s` cut by `plane`: the 3D section curves (S03 SK-08/SK-09, S07 tests, S12 DXF).
    ///
    /// Section edges come from a non-destructive `BRepAlgoAPI_Section` (category N). Lines,
    /// circles and ellipses come back as such, including B-spline results that are one within
    /// the kernel tolerance; everything else is a non-periodic B-spline. The segments are
    /// chained: each chain runs head to tail (closed chains end where they start), chains follow
    /// one another, and pieces of one carrier curve on one face are merged (a cylinder cut
    /// across its axis gives one full circle). Each segment's `tag` is the index of the face of
    /// `s` it lies on (`Tag(u64::MAX)` when OCCT names none). A plane that misses `s` gives an
    /// empty list.
    ///
    /// # Errors
    /// `InvalidInput` for a malformed plane, `Occt` when the section fails,
    /// `BackendUnavailable` without OCCT.
    fn plane_section(&self, s: &Shape, plane: &Plane) -> KResult<Vec<geom::CurveSeg>>;

    /// `edges` of `s` orthogonally projected onto `plane` (S03 SK-09 "project", S12 drawings).
    ///
    /// Projection is exact: lines stay lines, circles and ellipses become circles, ellipses or
    /// (seen edge-on) line segments, B-splines keep knots and weights with projected poles.
    /// B-spline edges that are lines, circles or ellipses are recognised first; other curves
    /// (offset, parabola, curves on surfaces only) are approximated by B-splines within the kernel
    /// tolerance. Circles and ellipses get `frame.normal = plane.normal`. Parts that collapse to
    /// a point (a line along the normal) and degenerated edges give nothing; an edge-on arc that
    /// turns back gives two segments.
    ///
    /// Segments keep the order of `edges`, and each `tag` is its edge's index. A segment is
    /// reversed where needed so that consecutive segments sharing an end point run head to tail:
    /// the edges of one wire in wire order ([`Shape::face_wires`], outer wire first) give an
    /// ordered loop, closed when the wire is.
    ///
    /// # Errors
    /// `InvalidInput` for a malformed plane or an edge out of range, `OpFailed` naming the edge
    /// when OCCT cannot convert its curve, `BackendUnavailable` without OCCT.
    fn project_edges(
        &self,
        s: &Shape,
        edges: &[EdgeId],
        plane: &Plane,
    ) -> KResult<Vec<geom::CurveSeg>>;

    // -- tessellation (BREP-08) --

    /// Triangulates `s` (S02 §8). Meshes a private copy of the topology (category C), with
    /// OCCT's face-parallel meshing on. Linear deflection is `q.linear` mm, or `q.linear` times
    /// the bbox diagonal (clamped to `q.max_linear_abs`) when `q.relative`.
    ///
    /// Cancellation is mid-op: `ctx` is polled through OCCT's progress indicator.
    ///
    /// # Errors
    /// `InvalidInput` for a quality with non-positive deflections, `Cancelled`, `Occt` when
    /// meshing raises, `BackendUnavailable` without OCCT.
    fn tessellate(&self, s: &Shape, q: &TessQuality, ctx: &JobCtx) -> KResult<TessMesh>;

    /// [`BrepKernel::tessellate`] of many bodies: rayon over the bodies, OCCT's face-parallel
    /// meshing off. One result per shape, in order. `ctx` progress counts finished bodies.
    fn tessellate_many(
        &self,
        shapes: &[Shape],
        q: &TessQuality,
        ctx: &JobCtx,
    ) -> Vec<KResult<TessMesh>>;

    /// Points along edge `e` in parameter order, within `q` (relative to the bbox of `s`).
    /// Independent of [`BrepKernel::tessellate`]'s edge polylines.
    ///
    /// # Errors
    /// `InvalidInput` for an out-of-range or degenerated edge or a bad quality.
    fn tessellate_edge(&self, s: &Shape, e: EdgeId, q: &TessQuality) -> KResult<Vec<Point3>>;

    // -- measurement queries (BREP-09) --

    /// Volume, area, centroid, inertia about the centroid and principal axes at unit density
    /// ([`MassProps`]), by adaptive `BRepGProp` integration to [`crate::MASS_PROPS_EPS`]
    /// relative. A shape without solids is measured by its faces (or edges); see
    /// [`MassProps`].
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" }` for a shape with nothing to measure (vertices only,
    /// empty); `Occt`; `BackendUnavailable`.
    fn mass_props(&self, s: &Shape) -> KResult<MassProps>;

    /// Whether `p` is inside, outside or on the boundary (within `tol` mm) of the solids of `s`
    /// (`BRepClass3d_SolidClassifier` per solid; inside any solid wins over on a boundary).
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" }` when `s` has no solid, `{ what: "tol" }` for a negative
    /// or non-finite tolerance; `Occt`; `BackendUnavailable`.
    fn classify(&self, s: &Shape, p: Point3, tol: f64) -> KResult<Classification>;

    /// The minimum distance between `a` and `b` (`BRepExtrema_DistShapeShape`) with the
    /// nearest points and the sub-shapes they lie on. Overlapping solids (or a shape inside a
    /// solid) give 0 with no sub-shapes.
    ///
    /// # Errors
    /// `InvalidInput { what: "shapes" }` for a null shape; `Occt`; `BackendUnavailable`.
    fn distance(&self, a: &Shape, b: &Shape) -> KResult<DistanceResult>;

    /// The point of `s`'s boundary nearest to `p` (its faces; edges or vertices for a shape
    /// without faces): `on_a`/`sub_a` are on `s`, `on_b` is `p`, `sub_b` is `None`. A point
    /// inside a solid still gets its nearest boundary point (distance > 0).
    ///
    /// # Errors
    /// `InvalidInput { what: "shape" }` for a null or empty shape; `Occt`; `BackendUnavailable`.
    fn closest_point(&self, s: &Shape, p: Point3) -> KResult<DistanceResult>;

    /// The first face hit by `ray` with `0 ≤ t ≤ max_t` (`IntCurvesFace_Intersector` per face,
    /// within the kernel's linear tolerance of the face boundaries), with the outward normal
    /// there. On a tie (an edge shared by two faces) the lower face index wins. `None` without
    /// a hit.
    ///
    /// # Errors
    /// `InvalidInput { what: "max_t" }` for a negative or NaN `max_t`; `Occt`;
    /// `BackendUnavailable`.
    fn ray_cast(&self, s: &Shape, ray: &Ray3, max_t: f64) -> KResult<Option<RayHit>>;

    /// The common volume of the solids of `a` and `b` (`boolean(Common)` with default options;
    /// history and warnings as for [`BrepKernel::boolean`]), or `None` when they do not overlap
    /// (touching faces, edges or points included).
    ///
    /// # Errors
    /// `InvalidInput { what: "objects" | "tools" }` when `a` or `b` has no solid; as
    /// [`BrepKernel::boolean`].
    fn interference(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<Option<OpResult>>;

    /// Face `f`'s surface at face parameters `uv`: point, outward normal, first derivatives
    /// and principal curvatures with their directions ([`SurfacePoint`]: positive = convex).
    ///
    /// # Errors
    /// `InvalidInput { what: "face" }` for an out-of-range face; `OpFailed { op:
    /// "eval_surface" }` where the normal or the curvature is undefined (a pole); `Occt`;
    /// `BackendUnavailable`.
    fn eval_surface(&self, s: &Shape, f: FaceId, uv: Point2) -> KResult<SurfacePoint>;

    /// The point of face `f` (within its boundary) nearest to `p`, and its face parameters.
    /// Where that point is on the boundary, `uv` comes from projecting it onto the surface,
    /// wrapped into the face's parameter range on periodic surfaces.
    ///
    /// # Errors
    /// `InvalidInput { what: "face" }` for an out-of-range face; `Occt`; `BackendUnavailable`.
    fn project_point_on_face(&self, s: &Shape, f: FaceId, p: Point3) -> KResult<(Point2, Point3)>;

    /// End points and unit tangents of edge `e` in its curve's parameter direction (S04
    /// R-S04-1, tangent-chain propagation).
    ///
    /// # Errors
    /// `InvalidInput { what: "edge" }` for an out-of-range or degenerated edge, or one whose
    /// tangent vanishes at an end; `Occt`; `BackendUnavailable`.
    fn edge_ends(&self, s: &Shape, e: EdgeId) -> KResult<EdgeEnds>;

    /// The fingerprint of one face, edge or vertex (S02 §3, BREP-05): class, measure, centroid,
    /// axis and bounding box, the input of S01's `GeomFingerprint`.
    ///
    /// # Errors
    /// `InvalidInput { what: "sub" }` for an out-of-range sub-shape; `Occt`;
    /// `BackendUnavailable`.
    fn sub_geom(&self, s: &Shape, sub: SubShape) -> KResult<SubShapeGeom>;

    /// The fingerprints of every face, edge and vertex of `s`, each list in index order
    /// (`[faces, edges, vertices]`). Spread over the current rayon pool in chunks.
    ///
    /// # Errors
    /// `InvalidInput` for a null shape; `Occt`; `BackendUnavailable`.
    fn sub_geom_all(&self, s: &Shape) -> KResult<[Vec<SubShapeGeom>; 3]>;
}

/// The OCCT backend. Cheap to clone; `Send + Sync`. Without the `occt` feature every op returns
/// `KernelError::BackendUnavailable`.
#[derive(Clone, Debug)]
pub struct OcctKernel {
    cfg: Arc<KernelConfig>,
}

static_assertions::assert_impl_all!(OcctKernel: Send, Sync, Clone);
static_assertions::assert_obj_safe!(BrepKernel);

impl OcctKernel {
    /// A kernel with `cfg`; runs [`init`] first.
    pub fn new(cfg: KernelConfig) -> OcctKernel {
        init(&cfg);
        OcctKernel { cfg: Arc::new(cfg) }
    }

    /// The configuration.
    pub fn config(&self) -> &KernelConfig {
        &self.cfg
    }

    /// Applies the validation policy to an op's result: with [`crate::Validate`] enabled for
    /// this build, an invalid `result` becomes `KernelError::InvalidResult`. Every op calls this
    /// on its result before returning it.
    ///
    /// # Errors
    /// `InvalidResult { op, report }` when checking is on and `result` is invalid.
    pub fn validate_result(&self, op: &'static str, result: &Shape) -> KResult<()> {
        if !self.cfg.validate.is_enabled() {
            return Ok(());
        }
        let report = result.check_report();
        if report.valid {
            Ok(())
        } else {
            Err(KernelError::InvalidResult { op, report })
        }
    }

    /// A synthetic op with an OCCT progress range of `steps` steps of `step` busy work each,
    /// reporting through the progress/cancel bridge. Exists to test that bridge.
    ///
    /// # Errors
    /// `Cancelled` when `ctx` is cancelled before or during the run, `BackendUnavailable`
    /// without the `occt` feature.
    #[doc(hidden)]
    pub fn progress_probe(&self, steps: u32, step: Duration, ctx: &JobCtx) -> KResult<()> {
        let micros = u32::try_from(step.as_micros()).unwrap_or(u32::MAX);
        backend::progress_probe(steps, micros, ctx)
    }

    /// [`BrepKernel::tessellate`] plus where its time went (the topology copy's share, for the
    /// S02 budget bench).
    ///
    /// # Errors
    /// As [`BrepKernel::tessellate`].
    #[doc(hidden)]
    pub fn tessellate_profiled(
        &self,
        s: &Shape,
        q: &TessQuality,
        ctx: &JobCtx,
    ) -> KResult<(TessMesh, TessProfile)> {
        tess::tessellate(s, q, ctx, true, true)
    }
}

/// `(edge, tag)` pairs from a backend edge map (one entry per input edge, `u32::MAX` where the
/// edge did not survive).
impl OcctKernel {
    fn primitive(
        &self,
        op: &'static str,
        kind: PrimitiveKind,
        frame: &Plane,
        params: &[f64],
    ) -> KResult<OpResult> {
        let shape = Shape::from_occ(Occ::primitive(kind, frame, params)?);
        self.validate_result(op, &shape)?;
        let history = ShapeHistory {
            roles: ops::primitive_roles(&shape, frame),
            ..ShapeHistory::default()
        };
        Ok(finished(shape, history, Vec::new()))
    }

    /// Merges coplanar faces and collinear edges of `s` (`ShapeUpgrade_UnifySameDomain` on a
    /// copy, with the options of [`PrismOpts::merge_coplanar`]), with history from `s`. The
    /// internal step of `prism`'s `merge_coplanar` as an op of its own, so the composition of
    /// the two can be checked against the end-to-end history (BREP-05 Done-when 4). The public
    /// `unify_same_domain` with options is BREP-10's.
    ///
    /// # Errors
    /// `InvalidInput` for a null shape; `InvalidResult` under the validation policy; `Occt`;
    /// `BackendUnavailable`.
    #[doc(hidden)]
    pub fn unify_coplanar(&self, s: &Shape) -> KResult<OpResult> {
        backend::require()?;
        if s.is_null() {
            return Err(KernelError::InvalidInput {
                what: "shape",
                reason: "the shape is null".into(),
                offending: Vec::new(),
            });
        }
        let (occ, raw) = s.occ().unify()?;
        let shape = Shape::from_occ(occ);
        self.validate_result("unify_coplanar", &shape)?;
        let history = ops::history(&raw, ops::counts(s));
        Ok(finished(shape, history, Vec::new()))
    }

    /// The `OpResult` of a boolean, split or section run over `operands`.
    fn bool_result(
        &self,
        op: &'static str,
        operands: &[&Shape],
        fuzzy: f64,
        outcome: BoolOutcome,
    ) -> KResult<OpResult> {
        let (occ, raw, status) = match outcome {
            Ok(done) => done,
            Err(status) => {
                return Err(
                    boolean::status_error(op, &status).unwrap_or(KernelError::OpFailed {
                        op,
                        message: "no result".into(),
                        offending: Vec::new(),
                    }),
                );
            }
        };
        if !status.warnings.is_empty() {
            tracing::debug!("{op}: BOPAlgo warnings {}", status.warnings.join(", "));
        }
        let shape = Shape::from_occ(occ);
        self.validate_result(op, &shape)?;
        let counts = operands.iter().map(|s| ops::counts(s)).collect();
        let mut history = ops::history_n(&raw, counts);
        if op == "section" {
            history.roles = boolean::section_roles(&shape);
        } else if op == "split" {
            history.roles = boolean::split_roles(&shape)?;
        }
        let warnings = boolean::warnings(fuzzy, operands, &shape, self.cfg.tolerance.linear);
        Ok(finished(shape, history, warnings))
    }

    /// `KernelConfig::bool_parallel`, unless the OCCT pool has one thread (BREP-04 bench: parallel
    /// mode on one thread is 24 % slower than serial).
    fn default_parallel(&self) -> bool {
        self.cfg.bool_parallel && self.cfg.effective_occt_threads().get() > 1
    }

    fn run_boolean(
        &self,
        op: &'static str,
        code: BoolCode,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let input = boolean::lower(code, objects, tools, opts, self.default_parallel())?;
        ctx.checkpoint()?;
        let (args, tools_in) = boolean::operand_split(code, objects, tools);
        let args: Vec<&Occ> = args.into_iter().map(Shape::occ).collect();
        let tools_in: Vec<&Occ> = tools_in.into_iter().map(Shape::occ).collect();
        let outcome = Occ::boolean(code, &args, &tools_in, &input, ctx)?;
        ctx.checkpoint()?;
        let operands: Vec<&Shape> = objects.iter().chain(tools).collect();
        self.bool_result(op, &operands, input.fuzzy, outcome)
    }

    /// Runs a lowered fillet or chamfer and builds its result.
    fn blended(
        &self,
        op: &'static str,
        s: &Shape,
        chamfer: bool,
        specs: &[BlendEdge],
        params: &blend::FilletParams,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        ctx.checkpoint()?;
        let outcome = s.occ().blend(chamfer, specs, params, true)?;
        ctx.checkpoint()?;
        let (occ, raw) =
            outcome.map_err(|(status, partial)| blend::failure(op, status, partial))?;
        let shape = Shape::from_occ(occ);
        self.validate_result(op, &shape)?;
        let history = ops::history(&raw, ops::counts(s));
        Ok(finished(shape, history, Vec::new()))
    }

    /// Runs a lowered shell, offset or thicken and builds its result.
    fn offset_result(
        &self,
        op: &'static str,
        s: &Shape,
        input: &OffsetIn,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        ctx.checkpoint()?;
        let outcome = s.occ().offset(input, ctx)?;
        ctx.checkpoint()?;
        let (occ, raw) =
            outcome.map_err(|(status, partial)| offset::failure(op, status, partial))?;
        let shape = Shape::from_occ(occ);
        self.validate_result(op, &shape)?;
        let history = ops::history(&raw, ops::counts(s));
        Ok(finished(shape, history, Vec::new()))
    }

    /// The result of a one-operand BREP-16 op (draft, defeature, split faces).
    fn local_result(
        &self,
        op: &'static str,
        s: &Shape,
        outcome: offset::OffsetOutcome<(Occ, ops::RawHistory)>,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        ctx.checkpoint()?;
        let (occ, raw) =
            outcome.map_err(|(status, partial)| offset::failure(op, status, partial))?;
        let shape = Shape::from_occ(occ);
        self.validate_result(op, &shape)?;
        let history = ops::history(&raw, ops::counts(s));
        Ok(finished(shape, history, Vec::new()))
    }

    /// Whether a constant-radius fillet of `edges` (lowered) at `r` gives a valid shape: `None`
    /// when it does, else why not.
    fn fillet_fails(&self, s: &Shape, edges: &[BlendEdge], r: f64) -> KResult<Option<KernelError>> {
        let specs: Vec<BlendEdge> = edges
            .iter()
            .map(|e| BlendEdge { a: r, ..e.clone() })
            .collect();
        let params = blend::lower_opts(&FilletOpts::default())?;
        match s.occ().blend(false, &specs, &params, false)? {
            Ok((occ, _)) => {
                let report = Shape::from_occ(occ).check_report();
                Ok((!report.valid).then_some(KernelError::InvalidResult {
                    op: "fillet",
                    report,
                }))
            }
            Err((status, partial)) => match blend::failure("fillet", status, partial) {
                e @ KernelError::LocalOpFailed { .. } => Ok(Some(e)),
                e => Err(e),
            },
        }
    }

    /// The result of a sweep over several operands (pipe, thru_sections).
    fn swept_n(
        &self,
        op: &'static str,
        operands: &[Shape],
        occ: Occ,
        raw: &ops::RawHistory,
    ) -> KResult<OpResult> {
        let shape = Shape::from_occ(occ);
        self.validate_result(op, &shape)?;
        let history = ops::history_n(raw, operands.iter().map(ops::counts).collect());
        Ok(finished(shape, history, Vec::new()))
    }

    fn swept(
        &self,
        op: &'static str,
        profile: &Shape,
        (occ, raw): (Occ, ops::RawHistory),
    ) -> KResult<OpResult> {
        let shape = Shape::from_occ(occ);
        self.validate_result(op, &shape)?;
        let history = ops::history(&raw, ops::counts(profile));
        Ok(finished(shape, history, Vec::new()))
    }
}

/// An op result whose history went through the BREP-05 finish (split order, completeness).
pub(crate) fn finished(
    shape: Shape,
    mut history: ShapeHistory,
    mut warnings: Vec<crate::KernelWarning>,
) -> OpResult {
    crate::history::finish(&mut history, &shape, &mut warnings);
    OpResult {
        shape,
        history,
        warnings,
    }
}

fn edge_tags(map: &[u32], tags: &[Tag]) -> Vec<(EdgeId, Tag)> {
    map.iter()
        .zip(tags)
        .filter(|(e, _)| **e != u32::MAX)
        .map(|(e, t)| (EdgeId(*e), *t))
        .collect()
}

impl BrepKernel for OcctKernel {
    fn check(&self, s: &Shape, opts: &CheckOpts) -> CheckReport {
        let mut report = s.check_report();
        if opts.self_interference && !s.is_null() {
            match s.occ().analyze() {
                Ok(problems) => {
                    report.valid &= problems.is_empty();
                    report.problems.extend(problems);
                }
                Err(e) => {
                    report.valid = false;
                    report
                        .problems
                        .push((None, format!("BOPAlgo_CheckFail: {e}")));
                }
            }
        }
        report
    }

    fn make_planar_faces(
        &self,
        profiles: &[geom::PlanarProfile],
        tol: &Tolerance,
    ) -> KResult<Vec<FaceBuild>> {
        backend::require()?;
        let (inputs, tags) = builders::lower_profiles(profiles, tol)?;
        let built = Occ::planar_faces(&inputs, tol.linear)?;
        built
            .into_iter()
            .zip(&tags)
            .map(|((occ, map), tags)| {
                let face = Shape::from_occ(occ);
                self.validate_result("make_planar_faces", &face)?;
                Ok(FaceBuild {
                    edge_tags: edge_tags(&map, tags),
                    face,
                })
            })
            .collect()
    }

    fn make_planar_polygon(
        &self,
        outer: &[Point3],
        holes: &[Vec<Point3>],
        tol: &Tolerance,
    ) -> KResult<Shape> {
        backend::require()?;
        let input = builders::lower_polygon(outer, holes, tol)?;
        let mut built = Occ::planar_faces(std::slice::from_ref(&input), tol.linear)?;
        let (occ, _) = built.pop().ok_or_else(|| KernelError::OpFailed {
            op: "make_planar_polygon",
            message: "no face was built".into(),
            offending: Vec::new(),
        })?;
        let face = Shape::from_occ(occ);
        self.validate_result("make_planar_polygon", &face)?;
        Ok(face)
    }

    fn make_face(
        &self,
        surface: &geom::Surface,
        bounds: &geom::FaceBounds,
        tol: f64,
    ) -> KResult<FaceBuild> {
        backend::require()?;
        let (bounds, tags) = builders::lower_face(surface, bounds, tol)?;
        let (occ, map) = Occ::face_on_surface(surface, &bounds, tol)?;
        let face = Shape::from_occ(occ);
        self.validate_result("make_face", &face)?;
        Ok(FaceBuild {
            edge_tags: edge_tags(&map, &tags),
            face,
        })
    }

    fn make_wire(&self, segs: &[geom::CurveSeg], tol: &Tolerance) -> KResult<OpResult> {
        backend::require()?;
        let (edges, closed) = builders::lower_wire(segs, tol)?;
        let (occ, map) = Occ::wire(&edges, closed, tol.linear)?;
        let shape = Shape::from_occ(occ);
        self.validate_result("make_wire", &shape)?;
        let seg_tags: Vec<Tag> = segs.iter().map(|s| s.tag).collect();
        let mut tags: Vec<(SubShape, Tag)> = edge_tags(&map, &seg_tags)
            .into_iter()
            .map(|(e, t)| {
                let sub = SubShape {
                    kind: SubKind::Edge,
                    index: e.0,
                };
                (sub, t)
            })
            .collect();
        tags.sort();
        Ok(OpResult {
            shape,
            history: ShapeHistory {
                tags,
                ..ShapeHistory::default()
            },
            warnings: Vec::new(),
        })
    }

    fn make_box(&self, frame: &Plane, size: Vec3) -> KResult<OpResult> {
        backend::require()?;
        let params = ops::box_params(frame, size, &self.cfg.tolerance)?;
        self.primitive("make_box", PrimitiveKind::Box, frame, &params)
    }

    fn make_cylinder(
        &self,
        frame: &Plane,
        radius: f64,
        height: f64,
        angle: f64,
    ) -> KResult<OpResult> {
        backend::require()?;
        let params = ops::cylinder_params(frame, radius, height, angle, &self.cfg.tolerance)?;
        self.primitive("make_cylinder", PrimitiveKind::Cylinder, frame, &params)
    }

    fn make_cone(
        &self,
        frame: &Plane,
        r1: f64,
        r2: f64,
        height: f64,
        angle: f64,
    ) -> KResult<OpResult> {
        backend::require()?;
        let params = ops::cone_params(frame, r1, r2, height, angle, &self.cfg.tolerance)?;
        self.primitive("make_cone", PrimitiveKind::Cone, frame, &params)
    }

    fn make_sphere(&self, frame: &Plane, radius: f64) -> KResult<OpResult> {
        backend::require()?;
        let params = ops::sphere_params(frame, radius, &self.cfg.tolerance)?;
        self.primitive("make_sphere", PrimitiveKind::Sphere, frame, &params)
    }

    fn make_torus(&self, frame: &Plane, major: f64, minor: f64, angle: f64) -> KResult<OpResult> {
        backend::require()?;
        let params = ops::torus_params(frame, major, minor, angle, &self.cfg.tolerance)?;
        self.primitive("make_torus", PrimitiveKind::Torus, frame, &params)
    }

    fn prism(
        &self,
        profile: &Shape,
        extent: &PrismExtent,
        opts: &PrismOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let input = ops::lower_prism(profile, extent, opts, &self.cfg.tolerance)?;
        ctx.checkpoint()?;
        let built = profile.occ().prism(&input)?;
        ctx.checkpoint()?;
        if input.mode == 2 {
            // A prism unbounded both ways has no cap to close its shell, so BRepCheck always
            // rejects it (`UnorientableShape`, `NotConnected`); S04 only splits with it.
            let (occ, raw) = built;
            let history = ops::history(&raw, ops::counts(profile));
            return Ok(finished(Shape::from_occ(occ), history, Vec::new()));
        }
        self.swept("prism", profile, built)
    }

    fn revol(&self, profile: &Shape, axis: &Line3, angle: f64, ctx: &JobCtx) -> KResult<OpResult> {
        backend::require()?;
        let (origin, dir, angle) = ops::lower_revol(profile, axis, angle, &self.cfg.tolerance)?;
        ctx.checkpoint()?;
        let built = profile.occ().revol(origin, dir, angle)?;
        ctx.checkpoint()?;
        self.swept("revol", profile, built)
    }

    fn pipe(
        &self,
        operands: &[Shape],
        profiles: usize,
        opts: &PipeOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let input = sweep::lower_pipe(operands, profiles, opts)?;
        ctx.checkpoint()?;
        let occs: Vec<&Occ> = operands.iter().map(Shape::occ).collect();
        let outcome = Occ::pipe(&occs, &input, ctx)?;
        ctx.checkpoint()?;
        let (occ, raw) = outcome.map_err(|status| sweep::failure("pipe", "operands", &status))?;
        self.swept_n("pipe", operands, occ, &raw)
    }

    fn thru_sections(
        &self,
        wires: &[Shape],
        sections: &[LoftSection],
        opts: &ThruSectionsOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let input = sweep::lower_thru_sections(wires, sections, opts)?;
        ctx.checkpoint()?;
        let occs: Vec<&Occ> = wires.iter().map(Shape::occ).collect();
        let outcome = Occ::thru_sections(&occs, &input, ctx)?;
        ctx.checkpoint()?;
        let (occ, raw) =
            outcome.map_err(|status| sweep::failure("thru_sections", "sections", &status))?;
        self.swept_n("thru_sections", wires, occ, &raw)
    }

    fn mirrored(&self, s: &Shape, plane: &Plane) -> KResult<OpResult> {
        backend::require()?;
        ops::check_frame(plane)?;
        let result = s.gtransformed(&ops::reflection(plane))?;
        self.validate_result("mirrored", &result.shape)?;
        Ok(result)
    }

    fn boolean(
        &self,
        op: BoolOp,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.run_boolean("boolean", op.into(), objects, tools, opts, ctx)
    }

    fn split(
        &self,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.run_boolean("split", BoolCode::Split, objects, tools, opts, ctx)
    }

    fn section(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<OpResult> {
        backend::require()?;
        if a.is_null() || b.is_null() {
            return Err(KernelError::InvalidInput {
                what: "objects",
                reason: "a section needs two non-null shapes".into(),
                offending: Vec::new(),
            });
        }
        ctx.checkpoint()?;
        let outcome = a.occ().section(b.occ(), self.default_parallel(), ctx)?;
        ctx.checkpoint()?;
        self.bool_result("section", &[a, b], 0.0, outcome)
    }

    fn fillet(
        &self,
        s: &Shape,
        edges: &[(EdgeId, FilletSpec)],
        opts: &FilletOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let params = blend::lower_opts(opts)?;
        let specs = blend::lower_fillet(s, edges, &self.cfg.tolerance, |e, f1, f2| {
            s.occ().edge_face_angle(e.0, f1.0, f2.0)
        })?;
        self.blended("fillet", s, false, &specs, &params, ctx)
    }

    fn chamfer(
        &self,
        s: &Shape,
        edges: &[(EdgeId, ChamferSpec)],
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let params = blend::lower_opts(&FilletOpts::default())?;
        let specs = blend::lower_chamfer(s, edges, &self.cfg.tolerance)?;
        self.blended("chamfer", s, true, &specs, &params, ctx)
    }

    fn fillet_max_radius(
        &self,
        s: &Shape,
        edges: &[EdgeId],
        hi: f64,
        iterations: u32,
        ctx: &JobCtx,
    ) -> KResult<f64> {
        backend::require()?;
        let tol = &self.cfg.tolerance;
        if !hi.is_finite() || hi <= tol.linear {
            return Err(KernelError::InvalidInput {
                what: "hi",
                reason: format!(
                    "the upper radius must be finite and larger than {:e} mm, got {hi}",
                    tol.linear
                ),
                offending: Vec::new(),
            });
        }
        if iterations == 0 || iterations > 64 {
            return Err(KernelError::InvalidInput {
                what: "iterations",
                reason: format!("iterations must be in 1..=64, got {iterations}"),
                offending: Vec::new(),
            });
        }
        let specs: Vec<(EdgeId, FilletSpec)> = edges
            .iter()
            .map(|&e| (e, FilletSpec::Constant(hi)))
            .collect();
        let lowered = blend::lower_fillet(s, &specs, tol, |_, _, _| Ok(0.0))?;
        ctx.checkpoint()?;
        let Some(mut last_failure) = self.fillet_fails(s, &lowered, hi)? else {
            return Ok(hi);
        };
        let (mut lo, mut bad) = (0.0, hi);
        for _ in 0..iterations {
            ctx.checkpoint()?;
            let mid = 0.5 * (lo + bad);
            match self.fillet_fails(s, &lowered, mid)? {
                None => lo = mid,
                Some(e) => {
                    bad = mid;
                    last_failure = e;
                }
            }
        }
        ctx.checkpoint()?;
        if lo > 0.0 {
            return Ok(lo);
        }
        let (message, offending) = match last_failure {
            KernelError::LocalOpFailed {
                message, offending, ..
            } => (message, offending),
            other => (other.to_string(), Vec::new()),
        };
        Err(KernelError::LocalOpFailed {
            op: "fillet_max_radius",
            message: format!("no radius in (0, {bad:e}] mm works: {message}"),
            offending,
            partial: None,
        })
    }

    fn shell(
        &self,
        s: &Shape,
        remove: &[FaceId],
        opts: &ShellOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let input = offset::lower_shell(s, remove, opts, &self.cfg.tolerance)?;
        self.offset_result("shell", s, &input, ctx)
    }

    fn offset(
        &self,
        s: &Shape,
        distance: f64,
        join: OffsetJoin,
        tol: f64,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let input = offset::lower_offset(s, distance, join, tol, &self.cfg.tolerance)?;
        self.offset_result("offset", s, &input, ctx)
    }

    fn thicken(
        &self,
        s: &Shape,
        thickness: f64,
        join: OffsetJoin,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let input = offset::lower_thicken(s, thickness, join, &self.cfg.tolerance)?;
        self.offset_result("thicken", s, &input, ctx)
    }

    fn draft(
        &self,
        s: &Shape,
        faces: &[FaceId],
        opts: &DraftOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let input = local::lower_draft(s, faces, opts)?;
        ctx.checkpoint()?;
        let outcome = s.occ().draft(&input, ctx)?;
        self.local_result("draft", s, outcome, ctx)
    }

    fn defeature(&self, s: &Shape, faces: &[FaceId], ctx: &JobCtx) -> KResult<OpResult> {
        backend::require()?;
        let faces = local::lower_defeature(s, faces)?;
        ctx.checkpoint()?;
        let outcome = s.occ().defeature(&faces, self.default_parallel(), ctx)?;
        self.local_result("defeature", s, outcome, ctx)
    }

    fn replace_face(
        &self,
        s: &Shape,
        face: FaceId,
        with: &geom::Surface,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        local::check_replace(s, face, with)?;
        ctx.checkpoint()?;
        let bbox = s.bbox();
        let bounds = local::tool_bounds(with, &bbox)?;
        let tool = self
            .make_face(with, &bounds, self.cfg.tolerance.linear)
            .map_err(|e| match e {
                KernelError::InvalidInput { reason, .. } => KernelError::InvalidInput {
                    what: "with",
                    reason,
                    offending: Vec::new(),
                },
                other => other,
            })?
            .face;
        let reach = s.occ().face_reach(face.0, tool.occ())?;
        if reach.is_nan() {
            return Err(KernelError::InvalidInput {
                what: "with",
                reason: "the new surface does not meet the lines along the face's normals".into(),
                offending: vec![offset::face_sub(face)],
            });
        }
        // Push the face past the new surface first, so its neighbours extend to reach it.
        let (base, first, base_face) = if reach > self.cfg.tolerance.linear {
            let d = local::overshoot(reach, &bbox, &self.cfg.tolerance);
            let r = self.offset_faces(s, &[(face, d)], ctx)?;
            let first_image = r
                .history
                .of_input(crate::history::InputSub {
                    operand: 0,
                    sub: offset::face_sub(face),
                })
                .next()
                .cloned();
            let moved = match first_image {
                Some(crate::history::Relation::Modified(out)) if out.len() == 1 => out[0].index,
                _ => {
                    return Err(KernelError::OpFailed {
                        op: "replace_face",
                        message: "the extended face has no single image".into(),
                        offending: vec![offset::face_sub(face)],
                    });
                }
            };
            let mut h = r.history;
            // The offset is internal: its roles do not describe the final faces.
            h.roles.clear();
            (r.shape, Some(h), moved)
        } else {
            (s.clone(), None, face.0)
        };
        ctx.checkpoint()?;
        let outcome = base.occ().trim_face(base_face, tool.occ(), ctx)?;
        ctx.checkpoint()?;
        let (occ, raw) = outcome
            .map_err(|(status, partial)| offset::failure("replace_face", status, partial))?;
        let shape = Shape::from_occ(occ);
        self.validate_result("replace_face", &shape)?;
        let second = ops::history(&raw, ops::counts(&base));
        let history = match first {
            Some(h) => h.compose(&second),
            None => second,
        };
        Ok(finished(shape, history, Vec::new()))
    }

    fn split_faces(
        &self,
        s: &Shape,
        faces: &[FaceId],
        tool: &Shape,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        backend::require()?;
        let list = local::face_list("faces", s, faces)?;
        if tool.is_null() || tool.count(SubKind::Edge) == 0 {
            return Err(KernelError::InvalidInput {
                what: "tool",
                reason: "the tool has no edges".into(),
                offending: Vec::new(),
            });
        }
        ctx.checkpoint()?;
        let outcome = s.occ().split_faces(&list, tool.occ(), ctx)?;
        self.local_result("split_faces", s, outcome, ctx)
    }

    fn offset_faces(&self, s: &Shape, faces: &[(FaceId, f64)], ctx: &JobCtx) -> KResult<OpResult> {
        backend::require()?;
        let input = local::lower_offset_faces(s, faces, &self.cfg.tolerance)?;
        self.offset_result("offset_faces", s, &input, ctx)
    }

    fn heal(&self, s: &Shape, opts: &HealOpts, ctx: &JobCtx) -> KResult<OpResult> {
        backend::require()?;
        let input = heal::lower_heal(s, opts)?;
        ctx.checkpoint()?;
        let outcome = s.occ().heal(&input, ctx)?;
        ctx.checkpoint()?;
        let (occ, raw, fixes) =
            outcome.map_err(|(status, partial)| heal::failure("heal", "shape", status, partial))?;
        let shape = Shape::from_occ(occ);
        self.validate_result("heal", &shape)?;
        let history = ops::history(&raw, ops::counts(s));
        let mut warnings = Vec::new();
        if fixes > 0 {
            warnings.push(crate::KernelWarning::Healed { fixes });
        }
        warnings.extend(boolean::warnings(
            0.0,
            &[s],
            &shape,
            self.cfg.tolerance.linear,
        ));
        Ok(finished(shape, history, warnings))
    }

    fn unify_same_domain(&self, s: &Shape, opts: &UnifyOpts) -> KResult<OpResult> {
        backend::require()?;
        let input = heal::lower_unify(s, opts)?;
        let (occ, raw) = s.occ().unify_same_domain(&input)?;
        let shape = Shape::from_occ(occ);
        self.validate_result("unify_same_domain", &shape)?;
        let history = ops::history(&raw, ops::counts(s));
        Ok(finished(shape, history, Vec::new()))
    }

    fn sew(&self, faces: &[Shape], opts: &SewOpts, ctx: &JobCtx) -> KResult<OpResult> {
        backend::require()?;
        let input = heal::lower_sew(faces, opts)?;
        ctx.checkpoint()?;
        let occs: Vec<&Occ> = faces.iter().map(Shape::occ).collect();
        let outcome = Occ::sew(&occs, &input, ctx)?;
        ctx.checkpoint()?;
        let (occ, raw) =
            outcome.map_err(|(status, partial)| heal::failure("sew", "faces", status, partial))?;
        let shape = Shape::from_occ(occ);
        self.validate_result("sew", &shape)?;
        let counts = faces.iter().map(ops::counts).collect();
        let history = ops::history_n(&raw, counts);
        let operands: Vec<&Shape> = faces.iter().collect();
        let warnings = boolean::warnings(0.0, &operands, &shape, self.cfg.tolerance.linear);
        Ok(finished(shape, history, warnings))
    }

    fn make_solid(&self, shell: &Shape) -> KResult<OpResult> {
        backend::require()?;
        heal::check_make_solid(shell)?;
        let (occ, raw) = shell
            .occ()
            .make_solid()?
            .map_err(|(status, partial)| heal::failure("make_solid", "shell", status, partial))?;
        let shape = Shape::from_occ(occ);
        self.validate_result("make_solid", &shape)?;
        let history = ops::history(&raw, ops::counts(shell));
        Ok(finished(shape, history, Vec::new()))
    }

    fn make_compound(&self, parts: &[Shape]) -> KResult<Shape> {
        backend::require()?;
        heal::check_parts(parts)?;
        let occs: Vec<&Occ> = parts.iter().map(Shape::occ).collect();
        Ok(Shape::from_occ(Occ::compound(&occs)?))
    }

    fn plane_section(&self, s: &Shape, plane: &Plane) -> KResult<Vec<geom::CurveSeg>> {
        backend::require()?;
        extract::plane_section(s, plane, &self.cfg.tolerance)
    }

    fn project_edges(
        &self,
        s: &Shape,
        edges: &[EdgeId],
        plane: &Plane,
    ) -> KResult<Vec<geom::CurveSeg>> {
        backend::require()?;
        extract::project_edges(s, edges, plane, &self.cfg.tolerance)
    }

    fn tessellate(&self, s: &Shape, q: &TessQuality, ctx: &JobCtx) -> KResult<TessMesh> {
        tess::tessellate(s, q, ctx, true, true).map(|(mesh, _)| mesh)
    }

    fn tessellate_many(
        &self,
        shapes: &[Shape],
        q: &TessQuality,
        ctx: &JobCtx,
    ) -> Vec<KResult<TessMesh>> {
        let done = std::sync::atomic::AtomicUsize::new(0);
        let total = shapes.len().max(1);
        shapes
            .par_iter()
            .map(|s| {
                let result = tess::tessellate(s, q, ctx, false, false).map(|(mesh, _)| mesh);
                let finished = done.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
                #[allow(clippy::cast_precision_loss, reason = "progress needs no precision")]
                ctx.progress(finished as f32 / total as f32);
                result
            })
            .collect()
    }

    fn tessellate_edge(&self, s: &Shape, e: EdgeId, q: &TessQuality) -> KResult<Vec<Point3>> {
        tess::tessellate_edge(s, e, q)
    }

    fn mass_props(&self, s: &Shape) -> KResult<MassProps> {
        backend::require()?;
        query::mass_props(s)
    }

    fn classify(&self, s: &Shape, p: Point3, tol: f64) -> KResult<Classification> {
        backend::require()?;
        query::classify(s, p, tol)
    }

    fn distance(&self, a: &Shape, b: &Shape) -> KResult<DistanceResult> {
        backend::require()?;
        query::distance(a, b)
    }

    fn closest_point(&self, s: &Shape, p: Point3) -> KResult<DistanceResult> {
        backend::require()?;
        query::closest_point(s, p)
    }

    fn ray_cast(&self, s: &Shape, ray: &Ray3, max_t: f64) -> KResult<Option<RayHit>> {
        backend::require()?;
        query::ray_cast(s, ray, max_t, &self.cfg.tolerance)
    }

    fn interference(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<Option<OpResult>> {
        backend::require()?;
        for (what, s) in [("objects", a), ("tools", b)] {
            if s.is_null() || s.solids().is_empty() {
                return Err(KernelError::InvalidInput {
                    what,
                    reason: "interference needs shapes with solids".into(),
                    offending: Vec::new(),
                });
            }
        }
        let common = self.boolean(
            BoolOp::Common,
            std::slice::from_ref(a),
            std::slice::from_ref(b),
            &BoolOpts::default(),
            ctx,
        )?;
        if common.shape.solids().is_empty() {
            return Ok(None);
        }
        let tol = self.cfg.tolerance.linear;
        let volume = query::mass_props(&common.shape)?.volume;
        Ok((volume > tol * tol * tol).then_some(common))
    }

    fn eval_surface(&self, s: &Shape, f: FaceId, uv: Point2) -> KResult<SurfacePoint> {
        backend::require()?;
        query::eval_surface(s, f, uv, &self.cfg.tolerance)
    }

    fn project_point_on_face(&self, s: &Shape, f: FaceId, p: Point3) -> KResult<(Point2, Point3)> {
        backend::require()?;
        query::project_point_on_face(s, f, p, &self.cfg.tolerance)
    }

    fn edge_ends(&self, s: &Shape, e: EdgeId) -> KResult<EdgeEnds> {
        backend::require()?;
        query::edge_ends(s, e)
    }

    fn sub_geom(&self, s: &Shape, sub: SubShape) -> KResult<SubShapeGeom> {
        backend::require()?;
        query::sub_geom(s, sub)
    }

    fn sub_geom_all(&self, s: &Shape) -> KResult<[Vec<SubShapeGeom>; 3]> {
        backend::require()?;
        query::sub_geom_all(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_has_three_parts() {
        let version = kernel_version();
        let parts: Vec<&str> = version.split(';').collect();
        assert_eq!(parts.len(), 3, "{version}");
        assert!(parts[0].starts_with("occt-"), "{version}");
        assert_eq!(parts[1], format!("amg-brep-{FORMAT_VERSION}"));
        assert_eq!(parts[2], format!("policy-{POLICY_VERSION}"));
        if crate::OCCT_BACKEND {
            assert!(version.starts_with("occt-8.0.1;"), "{version}");
        } else {
            assert!(version.starts_with("occt-none;"), "{version}");
        }
    }

    #[test]
    fn init_is_idempotent() {
        let cfg = KernelConfig::default();
        init(&cfg);
        init(&cfg);
        let other = KernelConfig {
            occt_threads: NonZeroUsize::new(cfg.effective_occt_threads().get() + 1),
            ..KernelConfig::default()
        };
        init(&other);
        let kernel = OcctKernel::new(other.clone());
        assert_eq!(kernel.config(), &other);
    }

    #[cfg(not(feature = "occt"))]
    #[test]
    fn no_backend_probe_is_unavailable() {
        let kernel = OcctKernel::new(KernelConfig::default());
        let result = kernel.progress_probe(1, Duration::ZERO, &JobCtx::detached());
        assert!(matches!(result, Err(KernelError::BackendUnavailable)));
    }

    #[cfg(feature = "occt")]
    mod occt {
        use super::*;
        use crate::Validate;
        use crate::shape::{SubKind, SubShape};

        fn kernel(validate: Validate) -> OcctKernel {
            OcctKernel::new(KernelConfig {
                validate,
                ..KernelConfig::default()
            })
        }

        #[test]
        fn valid_results_pass_every_policy() {
            let block = Shape::test_box(10.0, 20.0, 30.0);
            for policy in [Validate::Off, Validate::DebugOnly, Validate::Always] {
                kernel(policy).validate_result("box", &block).unwrap();
            }
            let report = kernel(Validate::Off).check(&block, &CheckOpts::default());
            assert_eq!(report, CheckReport::valid());
        }

        #[test]
        fn invalid_results_follow_the_policy() {
            let face = Shape::test_open_wire_face();
            let report = kernel(Validate::Off).check(&face, &CheckOpts::default());
            assert!(!report.valid);
            assert!(report.problem_kinds().contains(&"NotClosed"), "{report:?}");
            let on_face = SubShape {
                kind: SubKind::Face,
                index: 0,
            };
            assert!(report.problems.iter().any(|(s, _)| *s == Some(on_face)));

            kernel(Validate::Off)
                .validate_result("face", &face)
                .unwrap();
            match kernel(Validate::Always).validate_result("face", &face) {
                Err(KernelError::InvalidResult { op, report: got }) => {
                    assert_eq!(op, "face");
                    assert_eq!(got, report);
                }
                other => panic!("expected InvalidResult, got {other:?}"),
            }
            let debug = kernel(Validate::DebugOnly).validate_result("face", &face);
            assert_eq!(debug.is_err(), cfg!(debug_assertions));
        }

        #[test]
        fn max_tolerance_is_the_largest_sub_shape_tolerance() {
            let block = Shape::test_box(1.0, 2.0, 3.0);
            let mut largest: f64 = 0.0;
            for kind in [SubKind::Face, SubKind::Edge, SubKind::Vertex] {
                for index in 0..block.count(kind) {
                    largest = largest.max(block.tolerance(SubShape { kind, index }));
                }
            }
            assert!(largest > 0.0);
            assert_eq!(block.max_tolerance().to_bits(), largest.to_bits());
        }
    }
}
