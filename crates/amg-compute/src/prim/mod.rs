//! GPU primitives (CMP-01): reduce, dot, segmented reduce, scan, compaction, stable radix sort
//! and an atomic counter, with CPU references in [`cpu`].
//!
//! Spec: S15 (`amg_compute::prim`). Design: every result is a pure function of the input, never
//! of the device or the scheduling.
//! - **Reductions** use a fixed tree: a pass cuts the input into tiles of [`TILE`] = 2 048
//!   elements; thread `t` of a 256-thread workgroup folds `tile[t + 256 k]`, `k = 0..8`, in that
//!   order, then the 256 partial results are combined by halving strides (`s = 128, 64, … 1`:
//!   `a[t] = a[t] op a[t + s]`). Passes repeat on the per-tile results until one value is left.
//!   [`cpu`] replays exactly this order, so f32 and f64 sums are bit-equal on any IEEE device
//!   that does not flush denormals. There are no float atomics anywhere.
//! - **`dot`** rounds every product to the working precision first (a separate elementwise
//!   kernel), so a compiler cannot contract it into a fused multiply-add and change the result.
//! - **Integer primitives** (scan, compaction, sort) are exact by construction.

pub mod cpu;
mod scalar;

#[cfg(feature = "gpu")]
mod counter;
#[cfg(feature = "gpu")]
mod primitives;
#[cfg(feature = "gpu")]
mod wgsl;

pub use scalar::{ReduceOp, ReduceScalar, ScanKind};

#[cfg(feature = "gpu")]
pub use counter::AtomicCounter;
#[cfg(feature = "gpu")]
pub use primitives::Primitives;

/// Threads per workgroup of every primitive kernel.
pub(crate) const THREADS: usize = 256;
/// Elements each thread folds in the first step of a reduction or scan tile.
pub(crate) const PER_THREAD: usize = 8;
/// Elements per workgroup of the reduction and scan kernels.
pub const TILE: usize = THREADS * PER_THREAD;
/// Bits of the sort digit per pass (the radix is `1 << DIGIT_BITS`).
#[cfg(feature = "gpu")]
pub(crate) const DIGIT_BITS: u32 = 5;
/// Elements per workgroup of the sort kernels.
#[cfg(feature = "gpu")]
pub(crate) const SORT_TILE: usize = THREADS * wgsl::SORT_PT as usize;
