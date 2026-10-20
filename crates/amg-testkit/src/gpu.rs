//! GPU test gating.

use std::sync::OnceLock;

use crate::env;

/// True when a GPU test should run: a wgpu adapter exists for the backend selected by
/// `AMG_WGPU_BACKEND` (`dx12`, `vulkan` or `warp`; default: the primary backends).
///
/// Without an adapter it prints a skip notice and returns `false`, unless `AMG_REQUIRE_GPU=1`
/// (CI runs GPU tests on WARP), in which case it panics. The probe runs once per process.
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
    let probe = PROBE.get_or_init(|| {
        probe_adapter(Selection::from_env(
            std::env::var(env::WGPU_BACKEND).ok().as_deref(),
        ))
    });
    decide(probe, env::flag(env::REQUIRE_GPU))
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

#[derive(Clone, Copy, Debug, PartialEq)]
struct Selection {
    backends: wgpu::Backends,
    /// WARP is the DX12 software fallback adapter.
    fallback: bool,
}

impl Selection {
    fn from_env(v: Option<&str>) -> Selection {
        let s = |backends, fallback| Selection { backends, fallback };
        match v.map(|v| v.trim().to_ascii_lowercase()).as_deref() {
            None | Some("") => s(wgpu::Backends::PRIMARY, false),
            Some("dx12") => s(wgpu::Backends::DX12, false),
            Some("vulkan") => s(wgpu::Backends::VULKAN, false),
            Some("warp") => s(wgpu::Backends::DX12, true),
            Some(other) => panic!(
                "{}={other:?} is not one of dx12, vulkan, warp",
                env::WGPU_BACKEND
            ),
        }
    }
}

/// `Ok(adapter name)` or `Err(reason)`.
fn probe_adapter(sel: Selection) -> Result<String, String> {
    let mut desc = wgpu::InstanceDescriptor::new_without_display_handle();
    desc.backends = sel.backends;
    let instance = wgpu::Instance::new(desc);
    let options = wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::HighPerformance,
        force_fallback_adapter: sel.fallback,
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
    use super::*;

    #[test]
    fn backend_selection() {
        assert_eq!(
            Selection::from_env(None),
            Selection {
                backends: wgpu::Backends::PRIMARY,
                fallback: false
            }
        );
        assert_eq!(
            Selection::from_env(Some(" WARP ")),
            Selection {
                backends: wgpu::Backends::DX12,
                fallback: true
            }
        );
        assert_eq!(
            Selection::from_env(Some("vulkan")).backends,
            wgpu::Backends::VULKAN
        );
    }

    #[test]
    #[should_panic(expected = "not one of")]
    fn unknown_backend_panics() {
        let _ = Selection::from_env(Some("metal"));
    }

    #[test]
    fn decide_skips_or_fails() {
        assert!(decide(&Ok("x".into()), true));
        assert!(!decide(&Err("none".into()), false));
        let r = std::panic::catch_unwind(|| decide(&Err("none".into()), true));
        assert!(r.is_err());
    }

    #[test]
    fn gpu_available_probe() {
        // The real probe: skips without an adapter, fails with AMG_REQUIRE_GPU=1 (CI/WARP).
        let first = gpu_available();
        assert_eq!(first, gpu_available(), "probe result is cached");
        eprintln!("gpu_available() = {first}");
    }
}
