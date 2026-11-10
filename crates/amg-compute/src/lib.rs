//! GPU kernel helpers, voxel/brick grids, sparse matrices and iterative/direct/eigen solvers.
//!
//! Spec: S15. Lane: F. Design note: `docs/notes/compute-numerics.md`.
//!
//! Today this crate has the GPU layer (CMP-07) and the GPU primitives (CMP-01):
//! - [`precision`]: which floating-point precision GPU numeric kernels run in on a device.
//! - [`gpu`] (feature `gpu`): typed buffers, kernels, the dispatch context and the WGSL prelude.
//! - [`prim`]: reduce, dot, segmented reduce, scan, compaction, radix sort and `AtomicCounter`
//!   (feature `gpu`), plus their CPU references in `prim::cpu`.
//!
//! Numeric kernels are written once over the WGSL alias `real`, which the prelude binds to f64
//! on a device with `SHADER_F64` and to f32 elsewhere (ADR-004 note of 2026-11-09).

mod error;
pub mod precision;
pub mod prim;

#[cfg(feature = "gpu")]
pub mod gpu;

pub use error::{ComputeError, ComputeResult};
pub use half::f16;

use precision::{Precision, PrecisionPolicy};

/// Seals the scalar traits of this crate: the kernels only exist for these types.
mod sealed {
    pub trait Sealed {}
    impl Sealed for f32 {}
    impl Sealed for f64 {}
    impl Sealed for u32 {}
    impl Sealed for i32 {}
}

/// Where a solver runs.
#[derive(Clone, Debug)]
pub enum SolverDevice {
    /// The CPU, always in f64.
    Cpu,
    /// A wgpu device.
    #[cfg(feature = "gpu")]
    Wgpu(std::sync::Arc<amg_jobs::gpu::Gpu>),
}

impl SolverDevice {
    /// The precision a solver runs in on this device: `Cpu` is always F64, `Wgpu` is
    /// `policy.resolve(gpu)`.
    ///
    /// # Errors
    /// As [`PrecisionPolicy::resolve`].
    #[cfg_attr(
        not(feature = "gpu"),
        allow(unused_variables, reason = "only the GPU arm reads the policy")
    )]
    pub fn precision(&self, policy: PrecisionPolicy) -> ComputeResult<Precision> {
        match self {
            SolverDevice::Cpu => Ok(Precision::F64),
            #[cfg(feature = "gpu")]
            SolverDevice::Wgpu(gpu) => policy.resolve(gpu),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_is_always_f64() {
        for policy in [
            PrecisionPolicy::Auto,
            PrecisionPolicy::ForceF32,
            PrecisionPolicy::RequireF64,
        ] {
            assert_eq!(
                SolverDevice::Cpu.precision(policy).unwrap(),
                Precision::F64,
                "{policy:?}"
            );
        }
    }

    #[test]
    fn f16_is_reexported_with_bytemuck_support() {
        let v = [f16::from_f32(1.5)];
        assert_eq!(bytemuck::cast_slice::<f16, u8>(&v).len(), 2);
    }
}
