//! The error type of this crate.

use amg_core::{Cancelled, Diagnostic, Severity};
use amg_jobs::gpu::{GpuError, WGPU_BACKEND_ENV};

/// Errors raised by `amg-render`.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum RenderError {
    /// A GPU failure, including device loss.
    #[error(transparent)]
    Gpu(#[from] GpuError),
    /// `egui_glue::install` was given a `Gpu` other than the one egui draws with.
    #[error("the egui device is not the shared Gpu device")]
    DeviceMismatch,
    /// Input data is malformed.
    #[error("invalid mesh data: {0}")]
    InvalidData(String),
    /// The key does not name a live item.
    #[error("unknown {0} key")]
    UnknownKey(&'static str),
    /// The operation was cancelled.
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
}

impl Diagnostic for RenderError {
    fn user_message(&self) -> String {
        match self {
            RenderError::Gpu(e) => e.user_message(),
            other => other.to_string(),
        }
    }

    fn hint(&self) -> Option<String> {
        match self {
            RenderError::Gpu(e) => e.hint(),
            RenderError::DeviceMismatch => Some(format!(
                "Update the graphics driver, or set {WGPU_BACKEND_ENV} to try another backend."
            )),
            _ => None,
        }
    }

    fn severity(&self) -> Severity {
        match self {
            RenderError::Gpu(e) => e.severity(),
            RenderError::Cancelled(_) => Severity::Info,
            _ => Severity::Error,
        }
    }

    fn is_cancelled(&self) -> bool {
        match self {
            RenderError::Gpu(e) => e.is_cancelled(),
            RenderError::Cancelled(_) => true,
            _ => false,
        }
    }
}
