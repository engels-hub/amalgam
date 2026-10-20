//! The one parser for `AMG_WGPU_BACKEND`.
//!
//! This file is self-contained (it uses only `wgpu` and `std`) because it is compiled twice:
//! as `amg_jobs::gpu::backend` and, through `#[path]`, inside `amg-testkit`'s GPU probe. The
//! testkit cannot depend on `amg-jobs` (layering: `amg-testkit -> {amg-core}`, and `amg-jobs`
//! dev-depends on the testkit), so sharing the source is how both pick the same adapter.

use std::fmt;

/// Name of the backend override variable: `dx12`, `vulkan` or `warp` (case-insensitive).
pub const WGPU_BACKEND_ENV: &str = "AMG_WGPU_BACKEND";

/// What `AMG_WGPU_BACKEND` selects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BackendSelection {
    /// Backends the wgpu instance enables.
    pub backends: wgpu::Backends,
    /// Only a fallback (software) adapter may be returned. `warp` sets this: WARP is the DX12
    /// fallback adapter on Windows.
    pub force_fallback_adapter: bool,
}

impl BackendSelection {
    /// The selection without an override: DX12 on Windows, the primary backends elsewhere,
    /// hardware adapters only.
    pub fn platform_default() -> BackendSelection {
        let backends = if cfg!(windows) {
            wgpu::Backends::DX12
        } else {
            wgpu::Backends::PRIMARY
        };
        BackendSelection {
            backends,
            force_fallback_adapter: false,
        }
    }

    /// Parses a value of `AMG_WGPU_BACKEND`. `None` and the empty string mean
    /// [`BackendSelection::platform_default`]. Surrounding whitespace and case are ignored.
    ///
    /// # Errors
    /// [`UnknownBackend`] for anything other than `dx12`, `vulkan` or `warp`.
    pub fn parse(value: Option<&str>) -> Result<BackendSelection, UnknownBackend> {
        let pick = |backends, force_fallback_adapter| BackendSelection {
            backends,
            force_fallback_adapter,
        };
        match value.map(|v| v.trim().to_ascii_lowercase()).as_deref() {
            None | Some("") => Ok(BackendSelection::platform_default()),
            Some("dx12") => Ok(pick(wgpu::Backends::DX12, false)),
            Some("vulkan") => Ok(pick(wgpu::Backends::VULKAN, false)),
            Some("warp") => Ok(pick(wgpu::Backends::DX12, true)),
            Some(_) => Err(UnknownBackend(value.unwrap_or_default().to_owned())),
        }
    }

    /// Reads and parses `AMG_WGPU_BACKEND` from the process environment.
    ///
    /// # Errors
    /// As [`BackendSelection::parse`].
    pub fn from_env() -> Result<BackendSelection, UnknownBackend> {
        BackendSelection::parse(std::env::var(WGPU_BACKEND_ENV).ok().as_deref())
    }
}

/// `AMG_WGPU_BACKEND` has a value other than `dx12`, `vulkan` or `warp`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnknownBackend(pub String);

impl fmt::Display for UnknownBackend {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{WGPU_BACKEND_ENV}={:?} is not one of dx12, vulkan, warp",
            self.0
        )
    }
}

impl std::error::Error for UnknownBackend {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_every_value() {
        assert_eq!(
            BackendSelection::parse(None),
            Ok(BackendSelection::platform_default())
        );
        assert_eq!(
            BackendSelection::parse(Some("  ")),
            Ok(BackendSelection::platform_default())
        );
        assert_eq!(
            BackendSelection::parse(Some(" WARP ")),
            Ok(BackendSelection {
                backends: wgpu::Backends::DX12,
                force_fallback_adapter: true
            })
        );
        assert_eq!(
            BackendSelection::parse(Some("Dx12")),
            Ok(BackendSelection {
                backends: wgpu::Backends::DX12,
                force_fallback_adapter: false
            })
        );
        assert_eq!(
            BackendSelection::parse(Some("vulkan")).map(|s| s.backends),
            Ok(wgpu::Backends::VULKAN)
        );
    }

    #[test]
    fn unknown_value_is_an_error() {
        let e = BackendSelection::parse(Some("metal")).unwrap_err();
        assert_eq!(e, UnknownBackend("metal".into()));
        assert!(e.to_string().contains("not one of dx12, vulkan, warp"));
    }

    #[test]
    fn default_is_dx12_on_windows() {
        let d = BackendSelection::platform_default();
        assert!(!d.force_fallback_adapter);
        if cfg!(windows) {
            assert_eq!(d.backends, wgpu::Backends::DX12);
        }
    }
}
