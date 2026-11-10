//! The GPU primitives: recording of the passes of reduce, dot, segmented reduce, scan,
//! compaction and radix sort into a [`ComputeCtx`].
//!
//! Buffers handed to one call must be distinct (a buffer cannot be read and written by one
//! dispatch). Scratch buffers are allocated per call; wgpu keeps them alive until the recorded
//! work has run, so nothing is shared between calls or contexts.

use std::collections::HashMap;
use std::sync::Mutex;

use amg_jobs::gpu::{Gpu, wgpu};

use super::scalar::{ReduceOp, ReduceScalar, ScanKind};
use super::{DIGIT_BITS, SORT_TILE, TILE, wgsl};
use crate::gpu::{ComputeCtx, GpuVec, Kernel};
use crate::precision::{GpuReal, Precision};
use crate::{ComputeError, ComputeResult};

/// `max_compute_workgroups_per_dimension` of WebGPU and DX12.
const MAX_GROUP_DIM: u32 = 65_535;

/// Workgroup counts for `groups` workgroups, folded into two dimensions when needed.
fn groups_dim(groups: u32) -> [u32; 3] {
    if groups <= MAX_GROUP_DIM {
        [groups, 1, 1]
    } else {
        [MAX_GROUP_DIM, groups.div_ceil(MAX_GROUP_DIM), 1]
    }
}

/// Checks that `len` fits the `u32` indices of the kernels.
fn index_len(what: &str, len: u64) -> ComputeResult<u32> {
    u32::try_from(len).map_err(|_| {
        ComputeError::Shape(format!("{what}: {len} elements exceed the u32 index range"))
    })
}

/// Cached pipelines for all primitives.
///
/// Pipelines are compiled on first use (and cached by the device, see `Gpu::compute_pipeline`);
/// the f64 ones only on a device with `SHADER_F64`. An f64 call on any other device returns
/// `Unsupported`.
pub struct Primitives {
    kernels: Mutex<HashMap<String, Kernel>>,
}

impl std::fmt::Debug for Primitives {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Primitives").finish_non_exhaustive()
    }
}

impl Primitives {
    /// Builds the integer kernels (scan, compaction, sort) up front, so a shader that does not
    /// compile on `gpu` is reported here.
    ///
    /// # Errors
    /// [`ComputeError::Gpu`] when a kernel does not compile or the device is lost.
    pub fn new(gpu: &Gpu) -> ComputeResult<Self> {
        let p = Primitives {
            kernels: Mutex::new(HashMap::new()),
        };
        p.kernel(gpu, "scan_tiles", Precision::F32, wgsl::scan_tiles)?;
        p.kernel(
            gpu,
            "compact_scatter",
            Precision::F32,
            wgsl::compact_scatter,
        )?;
        p.kernel(gpu, "sort_histogram", Precision::F32, wgsl::sort_histogram)?;
        p.kernel(gpu, "sort_scatter", Precision::F32, wgsl::sort_scatter)?;
        Ok(p)
    }

    /// The kernel `key`, compiled with `build()` on first use.
    fn kernel(
        &self,
        gpu: &Gpu,
        key: &str,
        precision: Precision,
        build: impl FnOnce() -> String,
    ) -> ComputeResult<Kernel> {
        let mut kernels = self
            .kernels
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Some(k) = kernels.get(key) {
            return Ok(k.clone());
        }
        let k = Kernel::without_workgroup_zeroing(gpu, key, &build(), precision)?;
        kernels.insert(key.to_owned(), k.clone());
        Ok(k)
    }

    fn reduce_kernel<T: ReduceScalar>(
        &self,
        gpu: &Gpu,
        op: ReduceOp,
        flags: bool,
    ) -> ComputeResult<Kernel> {
        let key = format!(
            "reduce_{}_{op:?}_{:?}{}",
            T::WGSL_TYPE,
            T::KERNEL_PRECISION,
            if flags { "_flags" } else { "" }
        );
        self.kernel(gpu, &key, T::KERNEL_PRECISION, || {
            wgsl::reduce(T::WGSL_TYPE, op, T::wgsl_identity(op), flags)
        })
    }

    /// Reduces `input` to `out[out_index]` (no readback). An empty input writes the identity
    /// of `op`.
    ///
    /// # Errors
    /// [`ComputeError::Shape`] when `out_index` is outside `out` or a length exceeds `u32`;
    /// [`ComputeError::Unsupported`] for f64 on a device without `SHADER_F64`.
    pub fn reduce<T: ReduceScalar>(
        &self,
        cx: &mut ComputeCtx,
        op: ReduceOp,
        input: &GpuVec<T>,
        out: &GpuVec<T>,
        out_index: u64,
    ) -> ComputeResult<()> {
        let out_index = check_out("reduce", out.len(), out_index)?;
        let k = self.reduce_kernel::<T>(cx.gpu(), op, false)?;
        reduce_passes(cx, &k, input, out, out_index, 0)
    }

    /// `out[out_index] = sum(a[i] * b[i])`, products rounded to `R` first.
    ///
    /// # Errors
    /// As [`Primitives::reduce`]; [`ComputeError::Shape`] when `a` and `b` differ in length.
    pub fn dot<R: GpuReal>(
        &self,
        cx: &mut ComputeCtx,
        a: &GpuVec<R>,
        b: &GpuVec<R>,
        out: &GpuVec<R>,
        out_index: u64,
    ) -> ComputeResult<()> {
        if a.len() != b.len() {
            return Err(ComputeError::Shape(format!(
                "dot: vectors of {} and {} elements",
                a.len(),
                b.len()
            )));
        }
        let n = index_len("dot", a.len())?;
        check_out("dot", out.len(), out_index)?;
        if n == 0 {
            // An empty `products` buffer would be smaller than one `real`; the sum of nothing
            // is the identity.
            return self.reduce(cx, ReduceOp::Sum, a, out, out_index);
        }
        let mul = self.kernel(
            cx.gpu(),
            if R::PRECISION == Precision::F64 {
                "multiply_f64"
            } else {
                "multiply_f32"
            },
            R::PRECISION,
            wgsl::multiply,
        )?;
        let products = GpuVec::<R>::zeroed(cx.gpu(), "dot products", a.len())?;
        launch_1d(
            cx,
            &mul,
            &[n, 0, 0, 0],
            &[a.binding(), b.binding(), products.binding()],
            u64::from(n),
        );
        self.reduce(cx, ReduceOp::Sum, &products, out, out_index)
    }

    /// Reduces every segment `values[offsets[s]..offsets[s + 1]]` into `out[s]`; an empty
    /// segment gives the identity of `op`. `offsets` must be non-decreasing; offsets past the
    /// end of `values` are clamped. `offsets.len() - 1` segments are reduced (none for an empty
    /// `offsets`).
    ///
    /// # Errors
    /// [`ComputeError::Shape`] when `out` is shorter than the segment count, or there are more
    /// than 2²⁴ segments; [`ComputeError::Unsupported`] for f64 without `SHADER_F64`.
    pub fn segmented_reduce<R: GpuReal>(
        &self,
        cx: &mut ComputeCtx,
        op: ReduceOp,
        values: &GpuVec<R>,
        offsets: &GpuVec<u32>,
        out: &GpuVec<R>,
    ) -> ComputeResult<()> {
        let segments = offsets.len().saturating_sub(1);
        if out.len() < segments {
            return Err(ComputeError::Shape(format!(
                "segmented_reduce: {segments} segments but `out` has {} elements",
                out.len()
            )));
        }
        if segments >= 1 << 24 {
            return Err(ComputeError::Shape(format!(
                "segmented_reduce: {segments} segments (the limit is 2^24 - 1)"
            )));
        }
        let values_len = index_len("segmented_reduce", values.len())?;
        let key = format!(
            "segmented_{}_{op:?}",
            if R::PRECISION == Precision::F64 {
                "f64"
            } else {
                "f32"
            }
        );
        let k = self.kernel(cx.gpu(), &key, R::PRECISION, || {
            wgsl::segmented(R::WGSL_TYPE, op, R::wgsl_identity(op))
        })?;
        let segments = segments as u32;
        launch(
            cx,
            &k,
            &[segments, values_len, 0, 0],
            &[values.binding(), offsets.binding(), out.binding()],
            segments,
        );
        Ok(())
    }

    /// Exclusive or inclusive prefix sum of `input` into `output` (wrapping `u32` addition).
    /// `total` (optional) receives the sum of all inputs, the allocation size.
    ///
    /// # Errors
    /// [`ComputeError::Shape`] when `output` is shorter than `input` or `total` is outside its
    /// vector.
    pub fn scan(
        &self,
        cx: &mut ComputeCtx,
        kind: ScanKind,
        input: &GpuVec<u32>,
        output: &GpuVec<u32>,
        total: Option<(&GpuVec<u32>, u64)>,
    ) -> ComputeResult<()> {
        self.scan_impl(cx, kind, input, output, total, false)
    }

    fn scan_impl(
        &self,
        cx: &mut ComputeCtx,
        kind: ScanKind,
        input: &GpuVec<u32>,
        output: &GpuVec<u32>,
        total: Option<(&GpuVec<u32>, u64)>,
        flags: bool,
    ) -> ComputeResult<()> {
        let n = index_len("scan", input.len())?;
        if output.len() < input.len() {
            return Err(ComputeError::Shape(format!(
                "scan: output of {} elements for an input of {}",
                output.len(),
                input.len()
            )));
        }
        let total_index = match total {
            Some((t, i)) => check_out("scan total", t.len(), i)?,
            None => 0,
        };
        let gpu = cx.gpu_arc();
        let scan = self.kernel(&gpu, "scan_tiles", Precision::F32, wgsl::scan_tiles)?;
        // Bindings the pass does not use still need distinct buffers (one is read-only, the
        // other writable, and a buffer cannot be both in one dispatch).
        let dummy = GpuVec::<u32>::zeroed(&gpu, "scan dummy", 1)?;
        let dummy_rw = GpuVec::<u32>::zeroed(&gpu, "scan dummy (written)", 1)?;
        let total_vec = total.map_or(&dummy_rw, |(t, _)| t);
        let write_total = u32::from(total.is_some());
        let tiles = (n as usize).div_ceil(TILE).max(1) as u32;
        let inclusive = u32::from(kind == ScanKind::Inclusive);
        let xform = u32::from(flags);
        if tiles == 1 {
            launch(
                cx,
                &scan,
                &[n, 1, inclusive, 0, xform, write_total, total_index, 0],
                &[
                    input.binding(),
                    output.binding(),
                    dummy.binding(),
                    total_vec.binding(),
                ],
                1,
            );
            return Ok(());
        }
        let sums = GpuVec::<u32>::zeroed(&gpu, "scan tile sums", u64::from(tiles))?;
        let sum_kernel = self.reduce_kernel::<u32>(&gpu, ReduceOp::Sum, flags)?;
        launch(
            cx,
            &sum_kernel,
            &[n, tiles, 0, 0],
            &[input.binding(), sums.binding()],
            tiles,
        );
        let scanned = GpuVec::<u32>::zeroed(&gpu, "scan tile offsets", u64::from(tiles))?;
        self.scan_impl(cx, ScanKind::Exclusive, &sums, &scanned, total, false)?;
        launch(
            cx,
            &scan,
            &[n, tiles, inclusive, 1, xform, 0, 0, 0],
            &[
                input.binding(),
                output.binding(),
                scanned.binding(),
                dummy_rw.binding(),
            ],
            tiles,
        );
        Ok(())
    }

    /// Keeps `values[i]` where `flags[i] != 0`, in order, in `out`; `count[0]` = kept count.
    ///
    /// # Errors
    /// [`ComputeError::Shape`] when `flags` and `values` differ in length, `out` is shorter than
    /// `flags` (the kept count is unknown on the CPU) or `count` is empty.
    pub fn compact(
        &self,
        cx: &mut ComputeCtx,
        flags: &GpuVec<u32>,
        values: &GpuVec<u32>,
        out: &GpuVec<u32>,
        count: &GpuVec<u32>,
    ) -> ComputeResult<()> {
        if flags.len() != values.len() || out.len() < flags.len() {
            return Err(ComputeError::Shape(format!(
                "compact: flags {}, values {}, out {} elements",
                flags.len(),
                values.len(),
                out.len()
            )));
        }
        let n = index_len("compact", flags.len())?;
        let gpu = cx.gpu_arc();
        let positions = GpuVec::<u32>::zeroed(&gpu, "compact positions", flags.len())?;
        self.scan_impl(
            cx,
            ScanKind::Exclusive,
            flags,
            &positions,
            Some((count, 0)),
            true,
        )?;
        let k = self.kernel(
            &gpu,
            "compact_scatter",
            Precision::F32,
            wgsl::compact_scatter,
        )?;
        launch_1d(
            cx,
            &k,
            &[n, 0, 0, 0],
            &[
                flags.binding(),
                values.binding(),
                positions.binding(),
                out.binding(),
            ],
            u64::from(n),
        );
        Ok(())
    }

    /// Stable LSD radix sort of the low `key_bits` bits (1..=32) of `keys`; `values` move with
    /// the keys. Bits above `key_bits` do not take part in the order. The sorted data may end
    /// up in new buffers: the vectors are replaced, so bind groups made from them before the
    /// call are stale.
    ///
    /// # Errors
    /// [`ComputeError::Config`] for `key_bits` outside `1..=32`; [`ComputeError::Shape`] when
    /// `values` has a different length than `keys`.
    pub fn sort(
        &self,
        cx: &mut ComputeCtx,
        keys: &mut GpuVec<u32>,
        values: Option<&mut GpuVec<u32>>,
        key_bits: u32,
    ) -> ComputeResult<()> {
        if !(1..=32).contains(&key_bits) {
            return Err(ComputeError::Config(format!(
                "sort: key_bits {key_bits} is not in 1..=32"
            )));
        }
        if let Some(v) = &values
            && v.len() != keys.len()
        {
            return Err(ComputeError::Shape(format!(
                "sort: {} keys but {} values",
                keys.len(),
                v.len()
            )));
        }
        let n = index_len("sort", keys.len())?;
        if n < 2 {
            return Ok(());
        }
        let gpu = cx.gpu_arc();
        let hist_kernel =
            self.kernel(&gpu, "sort_histogram", Precision::F32, wgsl::sort_histogram)?;
        let scatter = self.kernel(&gpu, "sort_scatter", Precision::F32, wgsl::sort_scatter)?;
        let tiles = (n as usize).div_ceil(SORT_TILE) as u32;
        let has_vals = u32::from(values.is_some());
        let mut keys_tmp = GpuVec::<u32>::zeroed(&gpu, "sort keys", keys.len())?;
        let mut vals_tmp = match &values {
            Some(v) => GpuVec::<u32>::zeroed(&gpu, "sort values", v.len())?,
            None => GpuVec::<u32>::zeroed(&gpu, "sort values (unused)", 1)?,
        };
        let vals_dummy = GpuVec::<u32>::zeroed(&gpu, "sort values (unused)", 1)?;

        let passes = key_bits.div_ceil(DIGIT_BITS);
        for pass in 0..passes {
            let shift = pass * DIGIT_BITS;
            let bits = (key_bits - shift).min(DIGIT_BITS);
            let mask = (1u32 << bits) - 1;
            // Even passes read the caller's vectors, odd passes the temporaries.
            let (k_in, k_out, v_in, v_out): (
                &GpuVec<u32>,
                &GpuVec<u32>,
                &GpuVec<u32>,
                &GpuVec<u32>,
            ) = match (&values, pass % 2 == 0) {
                (Some(v), true) => (keys, &keys_tmp, v, &vals_tmp),
                (Some(v), false) => (&keys_tmp, keys, &vals_tmp, v),
                (None, true) => (keys, &keys_tmp, &vals_dummy, &vals_tmp),
                (None, false) => (&keys_tmp, keys, &vals_dummy, &vals_tmp),
            };
            let hist = GpuVec::<u32>::zeroed(
                &gpu,
                "sort histogram",
                u64::from(tiles) * u64::from(mask + 1),
            )?;
            launch(
                cx,
                &hist_kernel,
                &[n, tiles, shift, mask],
                &[k_in.binding(), hist.binding()],
                tiles,
            );
            let offsets = GpuVec::<u32>::zeroed(&gpu, "sort offsets", hist.len())?;
            self.scan_impl(cx, ScanKind::Exclusive, &hist, &offsets, None, false)?;
            launch(
                cx,
                &scatter,
                &[n, tiles, shift, mask, bits, has_vals, 0, 0],
                &[
                    k_in.binding(),
                    v_in.binding(),
                    offsets.binding(),
                    k_out.binding(),
                    v_out.binding(),
                ],
                tiles,
            );
        }
        if passes % 2 == 1 {
            // The result is in the temporaries; hand them to the caller.
            std::mem::swap(keys, &mut keys_tmp);
            if let Some(v) = values {
                std::mem::swap(v, &mut vals_tmp);
            }
        }
        Ok(())
    }
}

/// `out_index` as `u32`, checked against the length of `out`.
fn check_out(what: &str, out_len: u64, out_index: u64) -> ComputeResult<u32> {
    if out_index >= out_len {
        return Err(ComputeError::Shape(format!(
            "{what}: output index {out_index} is outside a vector of {out_len} elements"
        )));
    }
    index_len(what, out_index)
}

/// Records the tree passes of a reduction of `input` with kernel `k`; the last one writes
/// `out[out_index]`. `flags_xform` is the `xform` parameter word.
fn reduce_passes<T: ReduceScalar>(
    cx: &mut ComputeCtx,
    k: &Kernel,
    input: &GpuVec<T>,
    out: &GpuVec<T>,
    out_index: u32,
    xform: u32,
) -> ComputeResult<()> {
    let gpu = cx.gpu_arc();
    let mut len = index_len("reduce", input.len())?;
    let mut carried: Option<GpuVec<T>> = None;
    loop {
        let groups = (len as usize).div_ceil(TILE).max(1) as u32;
        let src = carried.as_ref().unwrap_or(input);
        if groups == 1 {
            launch(
                cx,
                k,
                &[len, 1, out_index, xform],
                &[src.binding(), out.binding()],
                1,
            );
            return Ok(());
        }
        let next = GpuVec::<T>::zeroed(&gpu, "reduce partials", u64::from(groups))?;
        launch(
            cx,
            k,
            &[len, groups, 0, xform],
            &[src.binding(), next.binding()],
            groups,
        );
        carried = Some(next);
        len = groups;
    }
}

/// Records one dispatch of `groups` workgroups with the uniform `params` at binding 0 and
/// `resources` from binding 1 on.
fn launch(
    cx: &mut ComputeCtx,
    k: &Kernel,
    params: &[u32],
    resources: &[wgpu::BindingResource<'_>],
    groups: u32,
) {
    let bind = bind(cx, k, params, resources);
    cx.dispatch(k, &[&bind], groups_dim(groups));
}

/// Like [`launch`] with one thread per element of `n`.
fn launch_1d(
    cx: &mut ComputeCtx,
    k: &Kernel,
    params: &[u32],
    resources: &[wgpu::BindingResource<'_>],
    n: u64,
) {
    let bind = bind(cx, k, params, resources);
    cx.dispatch_1d(k, &[&bind], n);
}

fn bind(
    cx: &mut ComputeCtx,
    k: &Kernel,
    params: &[u32],
    resources: &[wgpu::BindingResource<'_>],
) -> wgpu::BindGroup {
    let mut padded = [0u32; 8];
    padded[..params.len()].copy_from_slice(params);
    let slot = cx.uniform(&padded);
    let mut entries = vec![cx.uniform_binding(slot)];
    entries.extend(resources.iter().cloned());
    k.bind_group(cx.gpu(), 0, &entries)
}
