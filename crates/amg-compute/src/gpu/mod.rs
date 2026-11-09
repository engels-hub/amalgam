//! Thin layer over `amg_jobs::gpu` (CMP-07, design note §3.1): typed storage buffers, compiled
//! kernels, a batching dispatch context and the shared WGSL prelude. It does not hide wgpu;
//! kernels still bind and dispatch through wgpu types.
//!
//! ```no_run
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! use amg_compute::gpu::{ComputeCtx, GpuVec, Kernel};
//! use amg_jobs::JobCtx;
//! use amg_jobs::gpu::{Gpu, GpuConfig};
//!
//! let gpu = Gpu::new_blocking(&GpuConfig::from_env())?;
//! let v = GpuVec::from_slice(&gpu, "v", &[1.0f32, 2.0, 3.0])?;
//! let wgsl = "@group(0) @binding(0) var<storage, read_write> v: array<real>;
//!             @compute @workgroup_size(WG)
//!             fn main(@builtin(global_invocation_id) gid: vec3<u32>,
//!                     @builtin(num_workgroups) nwg: vec3<u32>) {
//!                 let i = amg_linear_id(gid, nwg);
//!                 if (i < arrayLength(&v)) { v[i] = v[i] * real(2); }
//!             }";
//! let double = Kernel::new(&gpu, "double", wgsl, "main", &[])?;
//! let bind = double.bind_group(&gpu, 0, &[v.binding()]);
//! let mut cx = ComputeCtx::new(gpu.clone(), "example");
//! cx.dispatch_1d(&double, &[&bind], v.len());
//! cx.finish(&JobCtx::detached())?; // job threads only
//! assert_eq!(v.read(&gpu).wait()?, [2.0, 4.0, 6.0]);
//! # Ok(()) }
//! ```

mod ctx;
mod kernel;
mod vec;

pub use ctx::{ComputeCtx, UniformSlot};
pub use kernel::{Kernel, prelude_wgsl};
pub use vec::GpuVec;
