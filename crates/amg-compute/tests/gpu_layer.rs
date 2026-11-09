//! CMP-07 GPU layer tests. GPU tests are named `*gpu*` (nextest test-group `gpu`, serial); they
//! skip without an adapter and fail under `AMG_REQUIRE_GPU=1` (CI runs them on WARP).
//! Per-precision tests go through `common` (the f64 skip/require rule).

#![cfg(feature = "gpu")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]
#![allow(
    clippy::float_cmp,
    reason = "bit-exact GPU vs CPU comparisons are the point"
)]

#[macro_use]
mod common;

use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_compute::gpu::{ComputeCtx, GpuVec, Kernel, prelude_wgsl};
use amg_compute::precision::{GpuReal, Precision, PrecisionPolicy, device_precision};
use amg_compute::{ComputeError, SolverDevice};
use amg_core::Cancelled;
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig, GpuError, wgpu};
use common::{F32_ONLY, F64Case, NO_SHADER_F64, f64_case};

/// Counts launches per index and writes a value that is exact in both precisions.
const COUNTER_WGSL: &str = r"
struct Params { n: u32, p0: u32, p1: u32, p2: u32 }
@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> count: array<atomic<u32>>;
@group(0) @binding(2) var<storage, read_write> value: array<real>;

@compute @workgroup_size(WG)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    let i = amg_linear_id(gid, nwg);
    if (i < params.n) {
        atomicAdd(&count[i], 1u);
        value[i] = real(i % 65536u) + real(0.5);
    }
}
";

/// `dst = src` and `doubled = src + src`.
const COPY_WGSL: &str = r"
@group(0) @binding(0) var<storage, read> src: array<real>;
@group(0) @binding(1) var<storage, read_write> dst: array<real>;
@group(0) @binding(2) var<storage, read_write> doubled: array<real>;

@compute @workgroup_size(WG)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    let i = amg_linear_id(gid, nwg);
    if (i < arrayLength(&dst)) {
        dst[i] = src[i];
        doubled[i] = src[i] + src[i];
    }
}
";

/// Copies its uniform into `out[index]`.
const UNIFORM_WGSL: &str = r"
struct Params { index: u32, value: u32, p0: u32, p1: u32 }
@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> out: array<u32>;

@compute @workgroup_size(1)
fn main() {
    out[params.index] = params.value;
}
";

const FILL_WGSL: &str = r"
@group(0) @binding(0) var<storage, read_write> out: array<u32>;

@compute @workgroup_size(WG)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    let i = amg_linear_id(gid, nwg);
    if (i < arrayLength(&out)) {
        out[i] = i + 1u;
    }
}
";

/// Finite normal values with full mantissas, exponents small enough to stay normal in f32.
fn normals(seed: u64, n: usize) -> Vec<f64> {
    let mut s = seed;
    (0..n)
        .map(|_| {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            let sign = s & (1 << 63);
            let exp = 1023 - 60 + (s >> 52) % 121; // 2^-60 ..= 2^60
            f64::from_bits(sign | (exp << 52) | (s & ((1 << 52) - 1)))
        })
        .collect()
}

fn check_exactly_once<R: GpuReal>(n: u64, counts: &[u32], values: &[R]) {
    let n = n as usize;
    let mut wrong_counts = 0usize;
    let mut wrong_values = 0usize;
    for (i, (&c, &v)) in counts.iter().zip(values).enumerate() {
        if i < n {
            wrong_counts += usize::from(c != 1);
            wrong_values += usize::from(v.to_f64() != (i % 65_536) as f64 + 0.5);
        } else {
            wrong_counts += usize::from(c != 0);
            wrong_values += usize::from(v.to_f64() != 0.0);
        }
    }
    assert_eq!(wrong_counts, 0, "n = {n}: indices not written exactly once");
    assert_eq!(wrong_values, 0, "n = {n}: wrong values");
}

gpu_test_per_precision!(
    dispatch_1d_writes_every_index_exactly_once,
    kernel_copy_is_bit_exact
);

/// Done-when 1, and the f64 kernel of Done-when 9.
fn dispatch_1d_writes_every_index_exactly_once<R: GpuReal>(gpu: Arc<Gpu>) {
    let ctx = JobCtx::detached();
    let kernel =
        Kernel::with_precision(&gpu, "counter", COUNTER_WGSL, "main", &[], R::PRECISION).unwrap();
    assert_eq!(kernel.precision(), R::PRECISION);
    assert_eq!(kernel.workgroup(), [256, 1, 1]);
    const GUARD: u64 = 16;
    for n in [0, 1, 255, 256, 257, 65_535 * 256 + 1, (1u64 << 26) + 3] {
        let t = Instant::now();
        let count = GpuVec::<u32>::zeroed(&gpu, "count", n + GUARD).unwrap();
        let value = GpuVec::<R>::zeroed(&gpu, "value", n + GUARD).unwrap();
        let mut cx = ComputeCtx::new(gpu.clone(), "counter");
        let slot = cx.uniform(&[n as u32, 0, 0, 0]);
        let bind = kernel.bind_group(
            &gpu,
            0,
            &[cx.uniform_binding(slot), count.binding(), value.binding()],
        );
        cx.dispatch_1d(&kernel, &[&bind], n);
        cx.finish(&ctx).unwrap();
        let counts = count.read(&gpu).wait().unwrap();
        let values = value.read(&gpu).wait().unwrap();
        check_exactly_once::<R>(n, &counts, &values);
        eprintln!(
            "n = {n}: every index written exactly once ({:?}, {:?} on {})",
            R::PRECISION,
            t.elapsed(),
            gpu.info().name
        );
    }
}

/// The bit-exact upload -> kernel copy -> readback of Done-when 9.
fn kernel_copy_is_bit_exact<R: GpuReal>(gpu: Arc<Gpu>) {
    const N: usize = 1 << 20;
    let source: Vec<R> = normals(7, N).into_iter().map(R::from_f64).collect();
    let src = GpuVec::from_slice(&gpu, "src", &source).unwrap();
    let dst = GpuVec::<R>::zeroed(&gpu, "dst", N as u64).unwrap();
    let doubled = GpuVec::<R>::zeroed(&gpu, "doubled", N as u64).unwrap();
    let kernel =
        Kernel::with_precision(&gpu, "copy", COPY_WGSL, "main", &[], R::PRECISION).unwrap();
    let bind = kernel.bind_group(&gpu, 0, &[src.binding(), dst.binding(), doubled.binding()]);
    let mut cx = ComputeCtx::new(gpu.clone(), "copy");
    cx.dispatch_1d(&kernel, &[&bind], N as u64);
    cx.finish(&JobCtx::detached()).unwrap();

    let copied = dst.read(&gpu).wait().unwrap();
    let copy_mismatches = copied
        .iter()
        .zip(&source)
        .filter(|(a, b)| bytemuck::bytes_of(*a) != bytemuck::bytes_of(*b))
        .count();
    assert_eq!(copy_mismatches, 0, "{copy_mismatches} of {N} copies differ");

    // `x + x` is exact and uses the whole mantissa, so an f32 kernel would not pass as f64.
    let sums = doubled.read(&gpu).wait().unwrap();
    let sum_mismatches = sums
        .iter()
        .zip(&source)
        .filter(|(a, b)| a.to_f64().to_bits() != (b.to_f64() * 2.0).to_bits())
        .count();
    assert_eq!(sum_mismatches, 0, "{sum_mismatches} of {N} sums differ");
    eprintln!("bit-exact: {N} of {N} {:?} values", R::PRECISION);
}

#[test]
fn f64_skip_and_require_rule() {
    use PrecisionPolicy::{Auto, ForceF32, RequireF64};
    assert_eq!(f64_case(true, Auto, false), F64Case::Run);
    assert_eq!(f64_case(true, Auto, true), F64Case::Run);
    assert_eq!(f64_case(true, RequireF64, false), F64Case::Run);
    assert_eq!(f64_case(false, Auto, false), F64Case::Skip(NO_SHADER_F64));
    assert_eq!(f64_case(false, Auto, true), F64Case::Fail(NO_SHADER_F64));
    assert_eq!(
        f64_case(false, RequireF64, false),
        F64Case::Fail(NO_SHADER_F64)
    );
    // `AMG_COMPUTE_PRECISION=f32` selects the f32 cases on any device.
    assert_eq!(f64_case(true, ForceF32, false), F64Case::Skip(F32_ONLY));
    assert_eq!(f64_case(false, ForceF32, false), F64Case::Skip(F32_ONLY));
    assert_eq!(f64_case(true, ForceF32, true), F64Case::Fail(F32_ONLY));
    assert_eq!(NO_SHADER_F64, "f64: adapter has no SHADER_F64");
}

/// Done-when 7, on whichever device this run selected.
#[test]
fn gpu_precision_policy_resolves_per_device() {
    let Some(gpu) = common::gpu() else { return };
    let has = gpu.has(wgpu::Features::SHADER_F64);
    let expected = if has { Precision::F64 } else { Precision::F32 };
    assert_eq!(device_precision(&gpu), expected);
    assert_eq!(PrecisionPolicy::Auto.resolve(&gpu).unwrap(), expected);
    assert_eq!(
        PrecisionPolicy::ForceF32.resolve(&gpu).unwrap(),
        Precision::F32
    );
    match PrecisionPolicy::RequireF64.resolve(&gpu) {
        Ok(p) => {
            assert!(has);
            assert_eq!(p, Precision::F64);
        }
        Err(e) => {
            assert!(!has);
            assert!(matches!(e, ComputeError::Unsupported(_)), "{e:?}");
            eprintln!("RequireF64: {e}");
        }
    }
    // The device matrix of Done-when 7 under the pinned wgpu 30.
    match gpu.info().backend {
        wgpu::Backend::Dx12 => assert_eq!(expected, Precision::F32, "DX12 has no SHADER_F64"),
        wgpu::Backend::Vulkan if common::require_f64() => {
            assert_eq!(expected, Precision::F64, "the f64 gate needs SHADER_F64");
        }
        _ => {}
    }

    let device = SolverDevice::Wgpu(gpu);
    assert_eq!(device.precision(PrecisionPolicy::Auto).unwrap(), expected);
    assert_eq!(
        device.precision(PrecisionPolicy::ForceF32).unwrap(),
        Precision::F32
    );
    eprintln!("Auto resolved to {expected:?}");
}

#[test]
fn gpu_f64_kernel_needs_shader_f64() {
    let Some(gpu) = common::gpu() else { return };
    let source = "@compute @workgroup_size(WG) fn main() {}";
    let kernel = Kernel::with_precision(&gpu, "empty", source, "main", &[], Precision::F64);
    if gpu.has(wgpu::Features::SHADER_F64) {
        assert_eq!(kernel.unwrap().precision(), Precision::F64);
    } else {
        let e = kernel.unwrap_err();
        assert!(matches!(e, ComputeError::Unsupported(_)), "{e:?}");
        assert!(e.to_string().contains("SHADER_F64"), "{e}");
    }
}

#[test]
fn gpu_both_precisions_of_a_kernel_are_separate_pipelines() {
    let Some(gpu) = common::gpu() else { return };
    let source = "@compute @workgroup_size(WG) fn main() { var x: real = real(1); x = x + x; }";
    let default = Kernel::new(&gpu, "k", source, "main", &[]).unwrap();
    assert_eq!(default.precision(), Precision::F32);
    let f32_again = Kernel::with_precision(&gpu, "k", source, "main", &[], Precision::F32).unwrap();
    assert_eq!(f32_again.precision(), Precision::F32);
    if gpu.has(wgpu::Features::SHADER_F64) {
        let f64_kernel =
            Kernel::with_precision(&gpu, "k", source, "main", &[], Precision::F64).unwrap();
        assert_eq!(f64_kernel.precision(), Precision::F64);
    }
    assert_ne!(
        prelude_wgsl(Precision::F32),
        prelude_wgsl(Precision::F64),
        "the cache keys on the full source"
    );
}

/// Done-when 2.
#[test]
fn gpu_zeroed_past_the_binding_limit_is_too_large() {
    let Some(gpu) = common::gpu() else { return };
    let limits = gpu.device().limits();
    let limit = limits
        .max_storage_buffer_binding_size
        .min(limits.max_buffer_size);
    eprintln!("binding limit: {limit} bytes");
    for len in [limit / 4 + 1, u64::MAX / 2, u64::MAX] {
        let e = GpuVec::<u32>::zeroed(&gpu, "huge", len).unwrap_err();
        let ComputeError::TooLarge {
            label,
            bytes,
            limit: l,
        } = &e
        else {
            panic!("expected TooLarge, got {e:?}");
        };
        assert_eq!(label, "huge");
        assert!(*bytes > *l, "{e}");
        assert_eq!(*l, limit);
    }
    assert!(gpu.poll().is_ok(), "the device keeps working");
}

/// Done-when 3; the 1 s bound applies under `AMG_STRICT_TIMING=1`.
#[test]
fn timing_gpu_finish_after_device_destroyed_is_device_lost() {
    let Some(gpu) = common::gpu() else { return };
    let kernel = Kernel::new(&gpu, "fill", FILL_WGSL, "main", &[]).unwrap();
    let out = GpuVec::<u32>::zeroed(&gpu, "out", 1024).unwrap();
    let bind = kernel.bind_group(&gpu, 0, &[out.binding()]);
    let mut cx = ComputeCtx::new(gpu.clone(), "lost");
    cx.dispatch_1d(&kernel, &[&bind], 1024);

    gpu.device().destroy();
    let start = Instant::now();
    let err = cx.finish(&JobCtx::detached()).unwrap_err();
    let elapsed = start.elapsed();
    assert!(
        matches!(err, ComputeError::Gpu(GpuError::DeviceLost(_))),
        "{err:?}"
    );
    eprintln!("finish returned `{err}` after {elapsed:?}");
    let bound = if std::env::var_os("AMG_STRICT_TIMING").is_some() {
        Duration::from_secs(1)
    } else {
        Duration::from_secs(5)
    };
    assert!(elapsed < bound, "finish took {elapsed:?}");

    // Recording on the lost device is harmless and keeps reporting the loss.
    cx.dispatch_1d(&kernel, &[&bind], 1024);
    assert!(matches!(
        cx.finish(&JobCtx::detached()),
        Err(ComputeError::Gpu(GpuError::DeviceLost(_)))
    ));
    assert!(cx.take_timings().is_empty());
}

#[test]
fn gpu_finish_reports_cancellation() {
    let Some(gpu) = common::gpu() else { return };
    let ctx = JobCtx::with_token({
        let token = JobCtx::detached().cancel_token();
        token.cancel();
        token
    });
    let mut cx = ComputeCtx::new(gpu, "cancelled");
    let err = cx.finish(&ctx).unwrap_err();
    assert!(matches!(err, ComputeError::Cancelled(Cancelled)), "{err:?}");
}

/// Done-when 5, with timestamps.
#[test]
fn gpu_take_timings_reports_labelled_dispatches() {
    let Some(gpu) = common::gpu() else { return };
    let kernel = Kernel::new(&gpu, "fill", FILL_WGSL, "main", &[]).unwrap();
    let out = GpuVec::<u32>::zeroed(&gpu, "out", 1 << 16).unwrap();
    let bind = kernel.bind_group(&gpu, 0, &[out.binding()]);
    let mut cx = ComputeCtx::new(gpu.clone(), "timed");
    assert!(cx.take_timings().is_empty(), "nothing dispatched yet");
    for _ in 0..3 {
        cx.dispatch_1d(&kernel, &[&bind], 1 << 16);
    }
    cx.finish(&JobCtx::detached()).unwrap();
    let timings = cx.take_timings();
    if gpu.has(wgpu::Features::TIMESTAMP_QUERY) {
        assert_eq!(timings.len(), 3, "{timings:?}");
        assert!(timings.iter().all(|(label, _)| label == "fill"));
        eprintln!("timings on {}: {timings:?}", gpu.info().name);
        assert!(cx.take_timings().is_empty(), "taken once");
    } else {
        assert!(timings.is_empty());
        eprintln!("no TIMESTAMP_QUERY on {}", gpu.info().name);
    }
    if std::env::var_os("AMG_BENCH_REFERENCE").is_some() {
        assert!(
            gpu.has(wgpu::Features::TIMESTAMP_QUERY),
            "the reference GPU has timestamps"
        );
    }
}

/// Done-when 5, without timestamps.
#[test]
fn gpu_take_timings_is_empty_without_timestamp_query() {
    let cfg = GpuConfig {
        optional_features: wgpu::Features::SHADER_F64,
        ..GpuConfig::from_env()
    };
    let Some(gpu) = common::gpu_with(cfg) else {
        return;
    };
    assert!(!gpu.has(wgpu::Features::TIMESTAMP_QUERY));
    let kernel = Kernel::new(&gpu, "fill", FILL_WGSL, "main", &[]).unwrap();
    let out = GpuVec::<u32>::zeroed(&gpu, "out", 256).unwrap();
    let bind = kernel.bind_group(&gpu, 0, &[out.binding()]);
    let mut cx = ComputeCtx::new(gpu, "untimed");
    cx.dispatch_1d(&kernel, &[&bind], 256);
    cx.finish(&JobCtx::detached()).unwrap();
    assert!(cx.take_timings().is_empty());
}

/// More uniforms than one arena buffer holds and more dispatches than one batch: slots taken
/// before an automatic submission stay valid.
#[test]
fn gpu_uniform_arena_and_auto_batching() {
    let Some(gpu) = common::gpu() else { return };
    const N: u32 = 2600;
    let kernel = Kernel::new(&gpu, "uniform", UNIFORM_WGSL, "main", &[]).unwrap();
    let out = GpuVec::<u32>::zeroed(&gpu, "out", u64::from(N)).unwrap();
    let mut cx = ComputeCtx::new(gpu.clone(), "arena");
    let slots: Vec<_> = (0..N)
        .map(|i| {
            let slot = cx.uniform(&[i, i * 3 + 1, 0, 0]);
            assert_eq!(slot.offset % 256, 0, "256-byte slots");
            assert_eq!(slot.size, 16);
            slot
        })
        .collect();
    let binds: Vec<_> = slots
        .iter()
        .map(|&s| kernel.bind_group(&gpu, 0, &[cx.uniform_binding(s), out.binding()]))
        .collect();
    for bind in &binds {
        cx.dispatch(&kernel, &[bind], [1, 1, 1]);
    }
    cx.finish(&JobCtx::detached()).unwrap();
    let values = out.read(&gpu).wait().unwrap();
    for (i, v) in values.iter().enumerate() {
        assert_eq!(*v, i as u32 * 3 + 1, "element {i}");
    }
    if gpu.has(wgpu::Features::TIMESTAMP_QUERY) {
        assert_eq!(cx.take_timings().len(), N as usize, "every batch is timed");
    }
}

#[test]
fn gpu_uniforms_are_reusable_across_submissions() {
    let Some(gpu) = common::gpu() else { return };
    let kernel = Kernel::new(&gpu, "uniform", UNIFORM_WGSL, "main", &[]).unwrap();
    let out = GpuVec::<u32>::zeroed(&gpu, "out", 2).unwrap();
    let mut cx = ComputeCtx::new(gpu.clone(), "reuse");
    for (index, value) in [(0u32, 11u32), (1, 22)] {
        let slot = cx.uniform(&[index, value, 0, 0]);
        assert_eq!(slot.offset, 0, "submit releases the slots");
        let bind = kernel.bind_group(&gpu, 0, &[cx.uniform_binding(slot), out.binding()]);
        cx.dispatch(&kernel, &[&bind], [1, 1, 1]);
        cx.submit();
    }
    cx.finish(&JobCtx::detached()).unwrap();
    assert_eq!(out.read(&gpu).wait().unwrap(), [11, 22]);
}

#[test]
fn gpu_wg_override_sets_the_workgroup_size() {
    let Some(gpu) = common::gpu() else { return };
    let kernel = Kernel::new(&gpu, "fill64", FILL_WGSL, "main", &[("WG", 64.0)]).unwrap();
    assert_eq!(kernel.workgroup(), [64, 1, 1]);
    const N: u64 = 1000;
    let out = GpuVec::<u32>::zeroed(&gpu, "out", N).unwrap();
    let bind = kernel.bind_group(&gpu, 0, &[out.binding()]);
    let mut cx = ComputeCtx::new(gpu.clone(), "wg");
    cx.dispatch_1d(&kernel, &[&bind], N);
    cx.finish(&JobCtx::detached()).unwrap();
    let values = out.read(&gpu).wait().unwrap();
    assert!(values.iter().enumerate().all(|(i, v)| *v == i as u32 + 1));

    let e = Kernel::new(&gpu, "bad", FILL_WGSL, "main", &[("WG", 1.5)]).unwrap_err();
    assert!(matches!(e, ComputeError::Config(_)), "{e:?}");
}

#[test]
fn gpu_invalid_kernel_is_a_shader_error() {
    let Some(gpu) = common::gpu() else { return };
    let e = Kernel::new(&gpu, "broken", "fn main( {", "main", &[]).unwrap_err();
    assert!(
        matches!(e, ComputeError::Gpu(GpuError::Shader { ref label, .. }) if label == "broken"),
        "{e:?}"
    );
}

#[test]
fn gpu_vec_round_trips_and_partial_access() {
    let Some(gpu) = common::gpu() else { return };
    let data: Vec<u32> = (0..1024).map(|i| i * 7).collect();
    let v = GpuVec::from_slice(&gpu, "v", &data).unwrap();
    assert_eq!(v.len(), 1024);
    assert!(!v.is_empty());
    assert_eq!(v.read(&gpu).wait().unwrap(), data);
    assert_eq!(v.read_range(&gpu, 10..12).wait().unwrap(), [70, 77]);

    v.write(&gpu, 2, &[5, 6]);
    v.write(&gpu, 0, &[]);
    assert_eq!(v.read_range(&gpu, 1..4).wait().unwrap(), [7, 5, 6]);

    // `copy` goes through the context's encoder.
    let dst = GpuVec::<u32>::zeroed(&gpu, "dst", 2048).unwrap();
    let mut cx = ComputeCtx::new(gpu.clone(), "copy");
    cx.copy(&v, &dst);
    cx.finish(&JobCtx::detached()).unwrap();
    let copied = dst.read(&gpu).wait().unwrap();
    assert_eq!(copied[3], 6);
    assert_eq!(copied[1023], 1023 * 7);
    assert_eq!(copied[1024], 0);
}

#[test]
fn gpu_empty_and_small_element_vectors() {
    let Some(gpu) = common::gpu() else { return };
    let empty = GpuVec::<u32>::zeroed(&gpu, "empty", 0).unwrap();
    assert!(empty.is_empty());
    assert!(
        GpuVec::<u32>::from_slice(&gpu, "empty", &[])
            .unwrap()
            .is_empty()
    );
    // An empty vector can still be bound and dispatched over.
    let kernel = Kernel::new(&gpu, "fill", FILL_WGSL, "main", &[]).unwrap();
    let bind = kernel.bind_group(&gpu, 0, &[empty.binding()]);
    let mut cx = ComputeCtx::new(gpu.clone(), "empty");
    cx.dispatch_1d(&kernel, &[&bind], 0);
    cx.copy(&empty, &empty);
    cx.finish(&JobCtx::detached()).unwrap();

    // 5 bytes round up to one binding of 8 bytes.
    let bytes = GpuVec::<u8>::from_slice(&gpu, "bytes", &[1, 2, 3, 4, 5]).unwrap();
    assert_eq!(bytes.len(), 5);
    assert_eq!(bytes.read_range(&gpu, 0..4).wait().unwrap(), [1, 2, 3, 4]);
    let halves = GpuVec::<amg_compute::f16>::zeroed(&gpu, "halves", 3).unwrap();
    assert_eq!(halves.len(), 3);
}

#[test]
#[should_panic(expected = "outside a vector")]
fn gpu_read_past_the_end_panics() {
    let Some(gpu) = common::gpu() else {
        panic!("outside a vector (no adapter)");
    };
    let v = GpuVec::<u32>::zeroed(&gpu, "v", 4).unwrap();
    let _ = v.read_range(&gpu, 0..5);
}
