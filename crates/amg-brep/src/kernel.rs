//! The `BrepKernel` trait, the OCCT backend `OcctKernel`, process init and `kernel_version`
//! (S02 §1, §9, §10; BREP-12).

use std::num::NonZeroUsize;
use std::sync::{Arc, OnceLock};
use std::time::Duration;

use amg_jobs::JobCtx;

use crate::backend;
use crate::check::{CheckOpts, CheckReport};
use crate::config::KernelConfig;
use crate::error::{KResult, KernelError};
use crate::shape::{FORMAT_VERSION, Shape};

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
}

impl BrepKernel for OcctKernel {
    fn check(&self, s: &Shape, _opts: &CheckOpts) -> CheckReport {
        s.check_report()
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
