//! Harness for GPU tests of `amg-compute` (S15 Testing).
//!
//! Numeric GPU tests are written once, generic over `R: GpuReal`, and
//! [`gpu_test_per_precision!`] runs them once per precision:
//! - No adapter: skipped, or failed under `AMG_REQUIRE_GPU=1` (`amg_testkit::gpu_available`) or
//!   `AMG_REQUIRE_F64=1`.
//! - f64 on a device without `SHADER_F64`: reported as skipped with the reason on stderr
//!   (never silently passed), and failed instead under `AMG_REQUIRE_F64=1`.
//! - `AMG_COMPUTE_PRECISION=f32` runs the f32 cases only, so one f64-capable device can run the
//!   F32 path alone.

#![allow(dead_code, reason = "each test binary uses a part of the harness")]

use std::sync::Arc;

use amg_compute::precision::{GpuReal, Precision, PrecisionPolicy};
use amg_jobs::gpu::{Gpu, GpuConfig, wgpu};

/// `AMG_REQUIRE_F64=1`: an f64 case that cannot run fails instead of skipping (the f64 gate).
pub const REQUIRE_F64_ENV: &str = "AMG_REQUIRE_F64";

/// The skip reason of an f64 case on a device without `SHADER_F64` (S15 Done-when wording).
pub const NO_SHADER_F64: &str = "f64: adapter has no SHADER_F64";

/// The skip reason of an f64 case with `AMG_COMPUTE_PRECISION=f32`.
pub const F32_ONLY: &str = "f64: AMG_COMPUTE_PRECISION=f32";

/// What to do with the f64 case of a test.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum F64Case {
    Run,
    Skip(&'static str),
    Fail(&'static str),
}

/// The skip/require rule of S15 Testing. `RequireF64` (`AMG_COMPUTE_PRECISION=f64`) requires the
/// device like `AMG_REQUIRE_F64=1` does.
pub fn f64_case(has_f64: bool, policy: PrecisionPolicy, require: bool) -> F64Case {
    let reason = match (policy, has_f64) {
        (PrecisionPolicy::ForceF32, _) => F32_ONLY,
        (_, true) => return F64Case::Run,
        (_, false) => NO_SHADER_F64,
    };
    if require || policy == PrecisionPolicy::RequireF64 {
        F64Case::Fail(reason)
    } else {
        F64Case::Skip(reason)
    }
}

/// True when `AMG_REQUIRE_F64=1`.
pub fn require_f64() -> bool {
    std::env::var(REQUIRE_F64_ENV).is_ok_and(|v| v == "1")
}

/// The device selected by `AMG_WGPU_BACKEND`, or `None` (with a notice) without an adapter.
pub fn gpu() -> Option<Arc<Gpu>> {
    gpu_with(GpuConfig::from_env())
}

/// Like [`gpu`], with a caller-adjusted config.
pub fn gpu_with(cfg: GpuConfig) -> Option<Arc<Gpu>> {
    if !amg_testkit::gpu_available() {
        return None;
    }
    let gpu = Gpu::new_blocking(&cfg).expect("adapter probed, device opens");
    let info = gpu.info();
    eprintln!(
        "adapter: {} ({:?}, {:?}, driver {} {}), SHADER_F64: {}, TIMESTAMP_QUERY: {}",
        info.name,
        info.backend,
        info.device_type,
        info.driver,
        info.driver_info,
        gpu.has(wgpu::Features::SHADER_F64),
        gpu.has(wgpu::Features::TIMESTAMP_QUERY),
    );
    Some(gpu)
}

/// The device for a case in precision `R`, applying the skip/require rule. `None` means the
/// case was reported as skipped.
pub fn gpu_for<R: GpuReal>() -> Option<Arc<Gpu>> {
    let Some(gpu) = gpu() else {
        assert!(
            R::PRECISION == Precision::F32 || !require_f64(),
            "f64: no GPU adapter and {REQUIRE_F64_ENV}=1 is set"
        );
        return None;
    };
    if R::PRECISION == Precision::F32 {
        return Some(gpu);
    }
    let policy = PrecisionPolicy::from_env().expect("valid AMG_COMPUTE_PRECISION");
    match f64_case(gpu.has(wgpu::Features::SHADER_F64), policy, require_f64()) {
        F64Case::Run => Some(gpu),
        F64Case::Skip(reason) => {
            eprintln!("SKIP: {reason}");
            None
        }
        F64Case::Fail(reason) => panic!("{reason} (the f64 case is required here)"),
    }
}

/// Runs one case of a per-precision test.
pub fn run_case<R: GpuReal>(test: impl FnOnce(Arc<Gpu>)) {
    if let Some(gpu) = gpu_for::<R>() {
        test(gpu);
    }
}

/// Declares `mod $name` with tests `gpu_f32` and `gpu_f64` that run the generic function
/// `fn $name<R: GpuReal>(gpu: Arc<Gpu>)` per precision. The names contain `gpu`, which puts them
/// in nextest's serial `gpu` test group.
#[macro_export]
macro_rules! gpu_test_per_precision {
    ($($name:ident),+ $(,)?) => {
        $(
            mod $name {
                #[test]
                fn gpu_f32() {
                    $crate::common::run_case::<f32>(super::$name::<f32>);
                }
                #[test]
                fn gpu_f64() {
                    $crate::common::run_case::<f64>(super::$name::<f64>);
                }
            }
        )+
    };
}
