//! Runtime selection of the floating-point precision of GPU numeric kernels (CMP-07).
//!
//! Policy (ADR-004 note of 2026-11-09, design note §2.1): f64 kernels where the device reports
//! `SHADER_F64` (in wgpu 30, Vulkan only), f32 kernels plus f64 iterative refinement on the CPU
//! everywhere else. The choice is a runtime value per device, never a Cargo feature.

use crate::prim::ReduceScalar;
use crate::{ComputeError, ComputeResult, sealed};

#[cfg(feature = "gpu")]
use amg_jobs::gpu::{Gpu, wgpu};

/// Floating-point precision of GPU numeric kernels.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Precision {
    /// 32-bit kernels; accuracy comes from f64 refinement on the CPU.
    F32,
    /// 64-bit kernels (needs `SHADER_F64`).
    F64,
}

/// How the precision is chosen for a device.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PrecisionPolicy {
    /// F64 iff the device has `SHADER_F64`, else F32 (the owner's policy; the default).
    #[default]
    Auto,
    /// The f32 + refinement path on any device (tests both paths on one f64-capable device).
    ForceF32,
    /// Errors with `Unsupported` on a device without `SHADER_F64` (the f64 test gate).
    RequireF64,
}

/// `auto` | `f32` | `f64`, case-insensitive; unset or empty = `Auto`.
pub const PRECISION_ENV: &str = "AMG_COMPUTE_PRECISION";

impl PrecisionPolicy {
    /// Parses a value of [`PRECISION_ENV`]. `None` and the empty string mean `Auto`; `f32` means
    /// `ForceF32` and `f64` means `RequireF64`. Surrounding whitespace and case are ignored.
    ///
    /// # Errors
    /// [`ComputeError::Config`] on any other value.
    pub fn parse(value: Option<&str>) -> ComputeResult<PrecisionPolicy> {
        match value.map(|v| v.trim().to_ascii_lowercase()).as_deref() {
            None | Some("" | "auto") => Ok(PrecisionPolicy::Auto),
            Some("f32") => Ok(PrecisionPolicy::ForceF32),
            Some("f64") => Ok(PrecisionPolicy::RequireF64),
            Some(_) => Err(ComputeError::Config(format!(
                "{PRECISION_ENV}={:?} is not one of auto, f32, f64",
                value.unwrap_or_default()
            ))),
        }
    }

    /// Reads and parses [`PRECISION_ENV`] from the process environment.
    ///
    /// # Errors
    /// As [`PrecisionPolicy::parse`]; a value that is not Unicode is a `Config` error too.
    pub fn from_env() -> ComputeResult<PrecisionPolicy> {
        match std::env::var(PRECISION_ENV) {
            Ok(v) => PrecisionPolicy::parse(Some(&v)),
            Err(std::env::VarError::NotPresent) => Ok(PrecisionPolicy::Auto),
            Err(e) => Err(ComputeError::Config(format!("{PRECISION_ENV}: {e}"))),
        }
    }

    /// The precision this policy gives on `gpu`.
    ///
    /// # Errors
    /// [`ComputeError::Unsupported`] for `RequireF64` on a device without `SHADER_F64`.
    #[cfg(feature = "gpu")]
    pub fn resolve(self, gpu: &Gpu) -> ComputeResult<Precision> {
        match self {
            PrecisionPolicy::Auto => Ok(device_precision(gpu)),
            PrecisionPolicy::ForceF32 => Ok(Precision::F32),
            PrecisionPolicy::RequireF64 => match device_precision(gpu) {
                Precision::F64 => Ok(Precision::F64),
                Precision::F32 => Err(ComputeError::Unsupported(format!(
                    "f64 kernels were required, but adapter `{}` ({:?}) has no SHADER_F64",
                    gpu.info().name,
                    gpu.info().backend
                ))),
            },
        }
    }
}

/// `Precision::F64` iff `gpu.has(wgpu::Features::SHADER_F64)`.
#[cfg(feature = "gpu")]
pub fn device_precision(gpu: &Gpu) -> Precision {
    if gpu.has(wgpu::Features::SHADER_F64) {
        Precision::F64
    } else {
        Precision::F32
    }
}

/// Scalars GPU numeric kernels are instantiated for: f32 and f64.
pub trait GpuReal:
    ReduceScalar + bytemuck::Pod + std::fmt::Debug + Send + Sync + sealed::Sealed + 'static
{
    /// The precision of this scalar.
    const PRECISION: Precision;
    /// `"f32"` or `"f64"`: the WGSL type the prelude binds to `real`.
    const WGSL: &'static str;
    /// Rounds an f64 to this type.
    fn from_f64(v: f64) -> Self;
    /// Widens this value to f64 (exact).
    fn to_f64(self) -> f64;
}

impl GpuReal for f32 {
    const PRECISION: Precision = Precision::F32;
    const WGSL: &'static str = "f32";
    fn from_f64(v: f64) -> f32 {
        v as f32
    }
    fn to_f64(self) -> f64 {
        f64::from(self)
    }
}

impl GpuReal for f64 {
    const PRECISION: Precision = Precision::F64;
    const WGSL: &'static str = "f64";
    fn from_f64(v: f64) -> f64 {
        v
    }
    fn to_f64(self) -> f64 {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_accepts_every_value_in_any_case() {
        for (value, expected) in [
            (None, PrecisionPolicy::Auto),
            (Some(""), PrecisionPolicy::Auto),
            (Some("  "), PrecisionPolicy::Auto),
            (Some("auto"), PrecisionPolicy::Auto),
            (Some("AUTO"), PrecisionPolicy::Auto),
            (Some("f32"), PrecisionPolicy::ForceF32),
            (Some("F32"), PrecisionPolicy::ForceF32),
            (Some(" f64 "), PrecisionPolicy::RequireF64),
            (Some("F64"), PrecisionPolicy::RequireF64),
        ] {
            assert_eq!(
                PrecisionPolicy::parse(value).unwrap(),
                expected,
                "{value:?}"
            );
        }
    }

    #[test]
    fn parse_rejects_anything_else_with_config() {
        for value in ["f16", "double", "f64x", "32"] {
            let e = PrecisionPolicy::parse(Some(value)).unwrap_err();
            assert!(matches!(e, ComputeError::Config(_)), "{value}: {e:?}");
            assert!(e.to_string().contains("auto, f32, f64"), "{e}");
        }
    }

    #[test]
    fn default_policy_is_auto() {
        assert_eq!(PrecisionPolicy::default(), PrecisionPolicy::Auto);
    }

    #[test]
    fn gpu_real_constants_and_conversions() {
        assert_eq!(<f32 as GpuReal>::PRECISION, Precision::F32);
        assert_eq!(<f64 as GpuReal>::PRECISION, Precision::F64);
        assert_eq!(<f32 as GpuReal>::WGSL, "f32");
        assert_eq!(<f64 as GpuReal>::WGSL, "f64");
        let third = 1.0f64 / 3.0;
        assert_eq!(f64::from_f64(third).to_bits(), third.to_bits());
        assert_eq!(f32::from_f64(third).to_bits(), (third as f32).to_bits());
        assert_eq!(
            f32::from_f64(third).to_f64().to_bits(),
            f64::from(third as f32).to_bits()
        );
    }
}
