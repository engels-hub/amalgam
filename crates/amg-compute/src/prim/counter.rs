//! A one-element `u32` counter for atomic slot allocation in user kernels.

use amg_jobs::gpu::{Gpu, Readback, wgpu};

use super::wgsl;
use crate::ComputeResult;
use crate::gpu::{ComputeCtx, GpuVec, Kernel};

/// One-element `u32` counter for atomic slot allocation in user kernels (S16 A-buffer).
///
/// Bind it as `var<storage, read_write> counter: array<atomic<u32>>` and take slots with
/// `atomicAdd(&counter[0], 1u)`. Slot order depends on thread scheduling, so a kernel that needs
/// a deterministic layout allocates with `scan` instead, or sorts afterwards.
#[derive(Debug)]
pub struct AtomicCounter {
    value: GpuVec<u32>,
    reset: Kernel,
}

impl AtomicCounter {
    /// A counter holding 0.
    ///
    /// # Errors
    /// [`ComputeError::Gpu`](crate::ComputeError::Gpu) when the reset kernel does not compile.
    pub fn new(gpu: &Gpu, label: &str) -> ComputeResult<Self> {
        Ok(AtomicCounter {
            value: GpuVec::zeroed(gpu, label, 1)?,
            reset: Kernel::new(
                gpu,
                "atomic counter reset",
                &wgsl::counter_reset(),
                "main",
                &[],
            )?,
        })
    }

    /// Records `counter = 0` into `cx`, ordered with the other work recorded there.
    pub fn reset(&self, cx: &mut ComputeCtx) {
        let bind = self.reset.bind_group(cx.gpu(), 0, &[self.value.binding()]);
        cx.dispatch(&self.reset, &[&bind], [1, 1, 1]);
    }

    /// The counter as a bind-group resource.
    pub fn binding(&self) -> wgpu::BindingResource<'_> {
        self.value.binding()
    }

    /// Reads the counter. `submit` the context first so pending work is included.
    pub fn read(&self, gpu: &Gpu) -> Readback<u32> {
        self.value.read(gpu)
    }
}
