//! The crate's error type.

use amg_core::{Cancelled, Diagnostic, Severity};

/// Errors from `amg-compute`.
#[derive(Debug, thiserror::Error)]
pub enum ComputeError {
    /// A device-level failure (device lost, shader compilation, mapping).
    #[cfg(feature = "gpu")]
    #[error(transparent)]
    Gpu(#[from] amg_jobs::gpu::GpuError),
    /// The job was cancelled.
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
    /// A buffer does not fit in one storage binding of the device.
    #[error("buffer `{label}` needs {bytes} bytes; the device binding limit is {limit}")]
    TooLarge {
        /// The buffer's label.
        label: String,
        /// Bytes it needs (`u64::MAX` when the size overflows).
        bytes: u64,
        /// The device's limit in bytes.
        limit: u64,
    },
    /// A matrix failed validation.
    #[error("invalid matrix: {0}")]
    InvalidMatrix(String),
    /// A pivot of a Cholesky factorisation was not positive.
    #[error("matrix is not positive definite: pivot {pivot:e} at row {row}")]
    NotPositiveDefinite {
        /// The row, in the original numbering.
        row: usize,
        /// The offending pivot.
        pivot: f64,
    },
    /// A zero pivot.
    #[error("singular matrix at row {row}")]
    Singular {
        /// The row, in the original numbering.
        row: usize,
    },
    /// An iterative solver stopped before reaching its tolerance.
    #[error("not converged after {iterations} iterations (relative residual {residual:e})")]
    NotConverged {
        /// Iterations done.
        iterations: u32,
        /// The relative residual reached.
        residual: f64,
    },
    /// Arguments have inconsistent dimensions.
    #[error("invalid shape: {0}")]
    Shape(String),
    /// E.g. an f64 kernel or `PrecisionPolicy::RequireF64` on a device without `SHADER_F64`.
    #[error("not supported on this device: {0}")]
    Unsupported(String),
    /// E.g. an unknown `AMG_COMPUTE_PRECISION` value.
    #[error("invalid setting: {0}")]
    Config(String),
}

/// `Result` with [`ComputeError`].
pub type ComputeResult<T> = Result<T, ComputeError>;

impl Diagnostic for ComputeError {
    fn user_message(&self) -> String {
        match self {
            #[cfg(feature = "gpu")]
            ComputeError::Gpu(e) => e.user_message(),
            ComputeError::TooLarge { .. } => {
                "The computation needs a larger GPU buffer than this graphics adapter allows."
                    .to_owned()
            }
            ComputeError::NotPositiveDefinite { .. } | ComputeError::Singular { .. } => {
                "The system of equations could not be solved.".to_owned()
            }
            ComputeError::NotConverged { .. } => "The solver did not converge.".to_owned(),
            ComputeError::Unsupported(_) => {
                "This computation is not supported on this graphics adapter.".to_owned()
            }
            other => other.to_string(),
        }
    }

    fn hint(&self) -> Option<String> {
        match self {
            #[cfg(feature = "gpu")]
            ComputeError::Gpu(e) => e.hint(),
            ComputeError::TooLarge { .. } => {
                Some("Reduce the resolution or the size of the model.".to_owned())
            }
            ComputeError::NotPositiveDefinite { .. } | ComputeError::Singular { .. } => {
                Some("Check that the model is fully constrained.".to_owned())
            }
            ComputeError::NotConverged { .. } => {
                Some("Loosen the tolerance or allow more iterations.".to_owned())
            }
            ComputeError::Unsupported(_) => Some(
                "64-bit GPU kernels need the Vulkan backend: set AMG_WGPU_BACKEND=vulkan."
                    .to_owned(),
            ),
            ComputeError::Config(_) => {
                Some("Valid values of AMG_COMPUTE_PRECISION are auto, f32 and f64.".to_owned())
            }
            _ => None,
        }
    }

    fn severity(&self) -> Severity {
        match self {
            #[cfg(feature = "gpu")]
            ComputeError::Gpu(e) => e.severity(),
            ComputeError::Cancelled(_) => Severity::Info,
            _ => Severity::Error,
        }
    }

    fn is_cancelled(&self) -> bool {
        match self {
            #[cfg(feature = "gpu")]
            ComputeError::Gpu(e) => e.is_cancelled(),
            ComputeError::Cancelled(_) => true,
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cancelled_is_info_and_cancelled() {
        let e = ComputeError::from(Cancelled);
        assert!(e.is_cancelled());
        assert_eq!(e.severity(), Severity::Info);
    }

    #[test]
    fn too_large_names_the_buffer_and_the_limit() {
        let e = ComputeError::TooLarge {
            label: "field".into(),
            bytes: 10,
            limit: 4,
        };
        assert_eq!(
            e.to_string(),
            "buffer `field` needs 10 bytes; the device binding limit is 4"
        );
        assert!(e.hint().is_some());
        assert!(!e.is_cancelled());
        assert_eq!(e.severity(), Severity::Error);
    }

    #[cfg(feature = "gpu")]
    #[test]
    fn gpu_errors_delegate() {
        use amg_jobs::gpu::GpuError;
        let inner = GpuError::DeviceLost("gone".into());
        let (message, hint) = (inner.user_message(), inner.hint());
        let e = ComputeError::from(inner);
        assert_eq!(e.user_message(), message);
        assert_eq!(e.hint(), hint);
        assert!(ComputeError::from(GpuError::Cancelled(Cancelled)).is_cancelled());
    }

    #[test]
    fn unsupported_points_at_vulkan() {
        let e = ComputeError::Unsupported("f64".into());
        assert!(e.hint().unwrap().contains("AMG_WGPU_BACKEND"));
    }
}
