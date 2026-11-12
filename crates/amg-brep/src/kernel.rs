//! The `BrepKernel` trait, the OCCT backend `OcctKernel`, process init and `kernel_version`
//! (S02 §1, §9, §10; BREP-12).

use std::num::NonZeroUsize;
use std::sync::{Arc, OnceLock};
use std::time::Duration;

use amg_core::{Plane, Point3, Tolerance};
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::backend::{self, Occ};
use crate::builders;
use crate::check::{CheckOpts, CheckReport};
use crate::config::KernelConfig;
use crate::error::{KResult, KernelError};
use crate::extract;
use crate::geom::{self, Tag};
use crate::history::{FaceBuild, OpResult, ShapeHistory};
use crate::opts::TessQuality;
use crate::shape::{EdgeId, FORMAT_VERSION, Shape, SubKind, SubShape};
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
    /// `BRepCheck_Analyzer` validity of `s`, with problems per sub-shape (S02 §9).
    /// `opts.self_interference` is BREP-10's and ignored until then.
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

    /// `object` minus `tool` through a non-destructive `BRepAlgoAPI_Cut` (category N). A probe
    /// for BREP-08's concurrency test until BREP-04 lands `boolean`; no history, no validation.
    ///
    /// # Errors
    /// `Occt` when the cut fails, `BackendUnavailable` without OCCT.
    #[doc(hidden)]
    pub fn boolean_cut_probe(&self, object: &Shape, tool: &Shape) -> KResult<Shape> {
        object
            .occ()
            .boolean_cut_probe(tool.occ())
            .map(Shape::from_occ)
    }
}

/// `(edge, tag)` pairs from a backend edge map (one entry per input edge, `u32::MAX` where the
/// edge did not survive).
fn edge_tags(map: &[u32], tags: &[Tag]) -> Vec<(EdgeId, Tag)> {
    map.iter()
        .zip(tags)
        .filter(|(e, _)| **e != u32::MAX)
        .map(|(e, t)| (EdgeId(*e), *t))
        .collect()
}

impl BrepKernel for OcctKernel {
    fn check(&self, s: &Shape, _opts: &CheckOpts) -> CheckReport {
        s.check_report()
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
