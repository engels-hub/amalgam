//! GPU test gating. The adapter probe needs the default-on `gpu` cargo feature (wgpu); a
//! crate that opts out with `default-features = false` gets a `gpu_available()` that always
//! skips.

use std::sync::OnceLock;

use crate::env;

/// True when a GPU test should run: a wgpu adapter exists for the backend selected by
/// `AMG_WGPU_BACKEND` (`dx12`, `vulkan` or `warp`; default: DX12 on Windows, the primary
/// backends elsewhere — the same choice as `amg_jobs::gpu::GpuConfig::from_env`).
///
/// Without an adapter it prints a skip notice and returns `false`, unless `AMG_REQUIRE_GPU=1`
/// (CI runs GPU tests on WARP), in which case it panics. The probe runs once per process.
/// Built without the `gpu` feature there is never an adapter, so this returns `false` (and
/// still panics under `AMG_REQUIRE_GPU=1`, since the GPU test cannot run).
///
/// ```no_run
/// #[test]
/// fn gpu_add_buffers() {
///     if !amg_testkit::gpu_available() {
///         return;
///     }
///     // ...
/// }
/// ```
///
/// # Panics
/// As described above, or if `AMG_WGPU_BACKEND` has an unknown value.
pub fn gpu_available() -> bool {
    static PROBE: OnceLock<Result<String, String>> = OnceLock::new();
    let probe = PROBE.get_or_init(probe);
    decide(probe, env::flag(env::REQUIRE_GPU))
}

#[cfg(feature = "gpu")]
fn probe() -> Result<String, String> {
    let value = std::env::var(env::WGPU_BACKEND).ok();
    let sel = wgpu_probe::backend::BackendSelection::parse(value.as_deref())
        .unwrap_or_else(|e| panic!("{e}"));
    wgpu_probe::probe_adapter(sel)
}

#[cfg(not(feature = "gpu"))]
fn probe() -> Result<String, String> {
    Err("amg-testkit was built without its `gpu` feature".to_owned())
}

fn decide(probe: &Result<String, String>, require: bool) -> bool {
    match probe {
        Ok(_) => true,
        Err(why) => {
            assert!(
                !require,
                "no GPU adapter ({why}) and {}=1 is set",
                env::REQUIRE_GPU
            );
            eprintln!("SKIP: GPU test, no adapter ({why})");
            false
        }
    }
}

#[cfg(feature = "gpu")]
mod wgpu_probe {
    /// The `AMG_WGPU_BACKEND` parser of `amg_jobs::gpu`, compiled here from the same source
    /// file. The testkit may not depend on `amg-jobs` (layering; `amg-jobs` dev-depends on the
    /// testkit), but both must pick the same adapter. Its unit tests run in both crates.
    #[path = "../../../../amg-jobs/src/gpu/backend.rs"]
    #[allow(dead_code, reason = "shared with amg-jobs, which uses the whole file")]
    pub(super) mod backend;

    use backend::BackendSelection;

    /// `Ok(adapter name)` or `Err(reason)`. Same adapter request as `amg_jobs::gpu::Gpu::new`
    /// with the default config.
    pub(super) fn probe_adapter(sel: BackendSelection) -> Result<String, String> {
        let mut desc = wgpu::InstanceDescriptor::new_without_display_handle();
        desc.backends = sel.backends;
        let instance = wgpu::Instance::new(desc);
        let options = wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: sel.force_fallback_adapter,
            compatible_surface: None,
            apply_limit_buckets: false,
        };
        #[allow(
            clippy::disallowed_methods,
            reason = "test helper, never on the UI thread; probes once per test process"
        )]
        let adapter = pollster::block_on(instance.request_adapter(&options));
        adapter
            .map(|a| {
                let info = a.get_info();
                format!("{} ({:?})", info.name, info.backend)
            })
            .map_err(|e| e.to_string())
    }

    #[cfg(test)]
    mod tests {
        #[test]
        fn env_var_name_is_shared() {
            assert_eq!(super::backend::WGPU_BACKEND_ENV, crate::env::WGPU_BACKEND);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decide_skips_or_fails() {
        assert!(decide(&Ok("x".into()), true));
        assert!(!decide(&Err("none".into()), false));
        let r = std::panic::catch_unwind(|| decide(&Err("none".into()), true));
        assert!(r.is_err());
    }

    #[cfg(not(feature = "gpu"))]
    #[test]
    fn without_feature_there_is_no_adapter() {
        assert!(probe().is_err());
    }

    #[test]
    fn gpu_available_probe() {
        // The real probe: skips without an adapter, fails with AMG_REQUIRE_GPU=1 (CI/WARP).
        let first = gpu_available();
        assert_eq!(first, gpu_available(), "probe result is cached");
        eprintln!("gpu_available() = {first}");
    }
}
