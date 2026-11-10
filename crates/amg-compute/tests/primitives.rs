//! CMP-01 GPU primitive tests. GPU tests are named `*gpu*` (nextest test-group `gpu`, serial);
//! they skip without an adapter and fail under `AMG_REQUIRE_GPU=1`. Float cases go through
//! `common` (the f64 skip/require rule).
//!
//! Float results are compared bit-for-bit with the tree-order CPU reference on a hardware
//! adapter, and within `n * eps * sum(|x|)` on a software adapter (WARP).

#![cfg(feature = "gpu")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]
#![allow(
    clippy::float_cmp,
    reason = "bit-exact GPU vs CPU comparisons are the point"
)]

#[macro_use]
mod common;

use std::sync::Arc;

use amg_compute::gpu::{ComputeCtx, GpuVec, Kernel};
use amg_compute::precision::GpuReal;
use amg_compute::prim::{AtomicCounter, Primitives, ReduceOp, ReduceScalar, ScanKind, cpu};
use amg_compute::{ComputeError, ComputeResult};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, wgpu};

/// The sizes of the Done-when.
const SIZES: [usize; 8] = [0, 1, 255, 256, 257, 65_537, (1 << 20) + 7, 1 << 24];
const OPS: [ReduceOp; 3] = [ReduceOp::Sum, ReduceOp::Min, ReduceOp::Max];

/// Deterministic generator (splitmix64).
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn u32(&mut self) -> u32 {
        (self.next() >> 32) as u32
    }
    fn below(&mut self, n: u32) -> u32 {
        (self.next() % u64::from(n)) as u32
    }
    /// Uniform in [-1, 1).
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    }
}

fn is_software(gpu: &Gpu) -> bool {
    gpu.info().device_type == wgpu::DeviceType::Cpu
}

fn real_data<R: GpuReal>(rng: &mut Rng, n: usize) -> Vec<R> {
    (0..n).map(|_| R::from_f64(rng.unit())).collect()
}

fn job() -> JobCtx {
    JobCtx::detached()
}

fn new_cx(gpu: &Arc<Gpu>) -> ComputeCtx {
    ComputeCtx::new(gpu.clone(), "primitives test")
}

fn read<T: bytemuck::Pod>(gpu: &Gpu, v: &GpuVec<T>) -> Vec<T> {
    if v.is_empty() {
        return Vec::new();
    }
    v.read(gpu).wait().unwrap()
}

/// Checks `got` against `want`: bit-equal on hardware, within `n eps sum|x|` on software.
fn check_real<R: GpuReal>(gpu: &Gpu, what: &str, got: R, want: R, n: usize, abs_sum: f64) {
    if got.to_f64().to_bits() == want.to_f64().to_bits() {
        return;
    }
    let eps = if R::PRECISION == amg_compute::precision::Precision::F32 {
        f64::from(f32::EPSILON)
    } else {
        f64::EPSILON
    };
    let bound = n as f64 * eps * abs_sum;
    let err = (got.to_f64() - want.to_f64()).abs();
    assert!(
        is_software(gpu) && err <= bound,
        "{what}: got {got:?}, want {want:?} (|err| {err:e}, bound {bound:e}, software: {})",
        is_software(gpu)
    );
}

fn bits<R: GpuReal>(v: &[R]) -> Vec<u64> {
    v.iter().map(|x| x.to_f64().to_bits()).collect()
}

fn reduce_on<T: ReduceScalar>(
    gpu: &Arc<Gpu>,
    p: &Primitives,
    op: ReduceOp,
    data: &[T],
) -> ComputeResult<T> {
    let input = GpuVec::from_slice(gpu, "in", data)?;
    let out = GpuVec::<T>::zeroed(gpu, "out", 3)?;
    let mut cx = new_cx(gpu);
    p.reduce(&mut cx, op, &input, &out, 2)?;
    cx.finish(&job())?;
    let r = read(gpu, &out);
    assert!(
        r[0].as_bytes_zero() && r[1].as_bytes_zero(),
        "reduce wrote outside out[out_index]"
    );
    Ok(r[2])
}

trait Zero {
    fn as_bytes_zero(&self) -> bool;
}
impl<T: bytemuck::Pod> Zero for T {
    fn as_bytes_zero(&self) -> bool {
        bytemuck::bytes_of(self).iter().all(|&b| b == 0)
    }
}

// ───────────────────────────── reduce ─────────────────────────────

#[test]
fn reduce_integers_gpu() {
    let Some(gpu) = common::gpu() else { return };
    let p = Primitives::new(&gpu).unwrap();
    let mut rng = Rng(1);
    for n in SIZES {
        let u: Vec<u32> = (0..n).map(|_| rng.u32()).collect();
        let i: Vec<i32> = u.iter().map(|&x| x as i32).collect();
        for op in OPS {
            assert_eq!(
                reduce_on(&gpu, &p, op, &u).unwrap(),
                cpu::reduce(op, &u),
                "u32 {op:?} n={n}"
            );
            assert_eq!(
                reduce_on(&gpu, &p, op, &i).unwrap(),
                cpu::reduce(op, &i),
                "i32 {op:?} n={n}"
            );
        }
        if n > 0 {
            assert_eq!(
                reduce_on(&gpu, &p, ReduceOp::Min, &u).unwrap(),
                *u.iter().min().unwrap()
            );
            assert_eq!(
                reduce_on(&gpu, &p, ReduceOp::Max, &i).unwrap(),
                *i.iter().max().unwrap()
            );
        }
    }
}

fn reduce_real<R: GpuReal>(gpu: Arc<Gpu>) {
    let p = Primitives::new(&gpu).unwrap();
    let mut rng = Rng(2);
    for n in SIZES {
        let v = real_data::<R>(&mut rng, n);
        let abs_sum: f64 = v.iter().map(|x| x.to_f64().abs()).sum();
        for op in OPS {
            let want = cpu::reduce(op, &v);
            let runs: Vec<R> = (0..3)
                .map(|_| reduce_on(&gpu, &p, op, &v).unwrap())
                .collect();
            assert_eq!(bits(&runs[..1]), bits(&runs[1..2]), "{op:?} n={n} run 1");
            assert_eq!(bits(&runs[..1]), bits(&runs[2..]), "{op:?} n={n} run 2");
            if op == ReduceOp::Sum {
                check_real(&gpu, &format!("sum n={n}"), runs[0], want, n, abs_sum);
            } else {
                assert_eq!(
                    runs[0].to_f64().to_bits(),
                    want.to_f64().to_bits(),
                    "{op:?} n={n}"
                );
            }
        }
    }
}

fn dot_real<R: GpuReal>(gpu: Arc<Gpu>) {
    let p = Primitives::new(&gpu).unwrap();
    let mut rng = Rng(3);
    for n in SIZES {
        let a = real_data::<R>(&mut rng, n);
        let b = real_data::<R>(&mut rng, n);
        let abs_sum: f64 = a
            .iter()
            .zip(&b)
            .map(|(x, y)| (x.to_f64() * y.to_f64()).abs())
            .sum();
        let want = cpu::dot(&a, &b);
        let (ga, gb) = (
            GpuVec::from_slice(&gpu, "a", &a).unwrap(),
            GpuVec::from_slice(&gpu, "b", &b).unwrap(),
        );
        let runs: Vec<R> = (0..3)
            .map(|_| {
                let out = GpuVec::<R>::zeroed(&gpu, "out", 2).unwrap();
                let mut cx = new_cx(&gpu);
                p.dot(&mut cx, &ga, &gb, &out, 1).unwrap();
                cx.finish(&job()).unwrap();
                read(&gpu, &out)[1]
            })
            .collect();
        assert_eq!(bits(&runs[..1]), bits(&runs[1..2]), "dot n={n} run 1");
        assert_eq!(bits(&runs[..1]), bits(&runs[2..]), "dot n={n} run 2");
        check_real(&gpu, &format!("dot n={n}"), runs[0], want, n, abs_sum);
    }
}

/// Segment boundaries covering `0..n`: mostly short segments, some empty, a few long.
fn random_offsets(rng: &mut Rng, n: usize) -> Vec<u32> {
    let mut offsets = vec![0u32];
    let mut at = 0usize;
    while at < n {
        let len = match rng.below(10) {
            0..=5 => rng.below(8),
            6..=8 => rng.below(600),
            _ => rng.below(6000),
        } as usize;
        at = (at + len).min(n);
        offsets.push(at as u32);
    }
    offsets
}

fn segmented_real<R: GpuReal>(gpu: Arc<Gpu>) {
    let p = Primitives::new(&gpu).unwrap();
    let mut rng = Rng(4);
    for n in SIZES {
        let v = real_data::<R>(&mut rng, n);
        let offsets = random_offsets(&mut rng, n);
        let segments = offsets.len() - 1;
        let (gv, go) = (
            GpuVec::from_slice(&gpu, "v", &v).unwrap(),
            GpuVec::from_slice(&gpu, "o", &offsets).unwrap(),
        );
        for op in OPS {
            let want = cpu::segmented_reduce(op, &v, &offsets);
            assert_eq!(want.len(), segments);
            let runs: Vec<Vec<R>> = (0..3)
                .map(|_| {
                    let out = GpuVec::<R>::zeroed(&gpu, "out", segments.max(1) as u64).unwrap();
                    let mut cx = new_cx(&gpu);
                    p.segmented_reduce(&mut cx, op, &gv, &go, &out).unwrap();
                    cx.finish(&job()).unwrap();
                    read(&gpu, &out)[..segments].to_vec()
                })
                .collect();
            assert_eq!(bits(&runs[0]), bits(&runs[1]), "{op:?} n={n} run 1");
            assert_eq!(bits(&runs[0]), bits(&runs[2]), "{op:?} n={n} run 2");
            for (s, (&g, &w)) in runs[0].iter().zip(&want).enumerate() {
                let (a, b) = (offsets[s] as usize, offsets[s + 1] as usize);
                let abs_sum: f64 = v[a..b].iter().map(|x| x.to_f64().abs()).sum();
                if op == ReduceOp::Sum {
                    check_real(&gpu, &format!("segment {s}"), g, w, b - a, abs_sum);
                } else {
                    assert_eq!(
                        g.to_f64().to_bits(),
                        w.to_f64().to_bits(),
                        "{op:?} segment {s}"
                    );
                }
            }
        }
    }
}

gpu_test_per_precision!(reduce_real, dot_real, segmented_real);

#[test]
fn segmented_reduce_with_no_segments_gpu() {
    // An empty `offsets` means no segments.
    let Some(gpu) = common::gpu() else { return };
    let p = Primitives::new(&gpu).unwrap();
    let v = GpuVec::<f32>::zeroed(&gpu, "v", 0).unwrap();
    let o = GpuVec::<u32>::zeroed(&gpu, "o", 0).unwrap();
    let out = GpuVec::<f32>::zeroed(&gpu, "out", 0).unwrap();
    let mut cx = new_cx(&gpu);
    p.segmented_reduce(&mut cx, ReduceOp::Sum, &v, &o, &out)
        .unwrap();
    cx.finish(&job()).unwrap();
}

#[test]
fn f64_without_shader_f64_is_unsupported_gpu() {
    let Some(gpu) = common::gpu() else { return };
    if gpu.has(wgpu::Features::SHADER_F64) {
        eprintln!("SKIP: this adapter has SHADER_F64");
        return;
    }
    let p = Primitives::new(&gpu).unwrap();
    let v = GpuVec::from_slice(&gpu, "v", &[1.0f64, 2.0]).unwrap();
    let out = GpuVec::<f64>::zeroed(&gpu, "out", 1).unwrap();
    let off = GpuVec::from_slice(&gpu, "o", &[0u32, 2]).unwrap();
    let mut cx = new_cx(&gpu);
    let e = p.reduce(&mut cx, ReduceOp::Sum, &v, &out, 0).unwrap_err();
    assert!(matches!(e, ComputeError::Unsupported(_)), "{e:?}");
    let e = p.dot(&mut cx, &v, &v, &out, 0).unwrap_err();
    assert!(matches!(e, ComputeError::Unsupported(_)), "{e:?}");
    let e = p
        .segmented_reduce(&mut cx, ReduceOp::Max, &v, &off, &out)
        .unwrap_err();
    assert!(matches!(e, ComputeError::Unsupported(_)), "{e:?}");
}

#[test]
fn argument_errors_are_reported_gpu() {
    let Some(gpu) = common::gpu() else { return };
    let p = Primitives::new(&gpu).unwrap();
    let a = GpuVec::from_slice(&gpu, "a", &[1.0f32, 2.0, 3.0]).unwrap();
    let b = GpuVec::from_slice(&gpu, "b", &[1.0f32, 2.0]).unwrap();
    let out = GpuVec::<f32>::zeroed(&gpu, "out", 1).unwrap();
    let u = GpuVec::from_slice(&gpu, "u", &[1u32, 2, 3]).unwrap();
    let u_short = GpuVec::<u32>::zeroed(&gpu, "u", 2).unwrap();
    let mut cx = new_cx(&gpu);
    let shape = |r: ComputeResult<()>| matches!(r, Err(ComputeError::Shape(_)));
    assert!(shape(p.reduce(&mut cx, ReduceOp::Sum, &a, &out, 1)));
    assert!(shape(p.dot(&mut cx, &a, &b, &out, 0)));
    assert!(shape(p.scan(
        &mut cx,
        ScanKind::Exclusive,
        &u,
        &u_short,
        None
    )));
    assert!(shape(p.compact(&mut cx, &u, &u_short, &u, &u)));
    let mut k = GpuVec::from_slice(&gpu, "k", &[3u32, 1, 2]).unwrap();
    let mut v = GpuVec::<u32>::zeroed(&gpu, "v", 2).unwrap();
    assert!(shape(p.sort(&mut cx, &mut k, Some(&mut v), 32)));
    for bad in [0, 33] {
        let e = p.sort(&mut cx, &mut k, None, bad).unwrap_err();
        assert!(matches!(e, ComputeError::Config(_)), "{e:?}");
    }
}

// ───────────────────────── scan and compaction ─────────────────────────

fn scan_on(gpu: &Arc<Gpu>, p: &Primitives, kind: ScanKind, data: &[u32]) -> (Vec<u32>, u32) {
    let input = GpuVec::from_slice(gpu, "in", data).unwrap();
    let output = GpuVec::<u32>::zeroed(gpu, "out", data.len() as u64).unwrap();
    let total = GpuVec::<u32>::zeroed(gpu, "total", 3).unwrap();
    let mut cx = new_cx(gpu);
    p.scan(&mut cx, kind, &input, &output, Some((&total, 1)))
        .unwrap();
    cx.finish(&job()).unwrap();
    let t = read(gpu, &total);
    assert_eq!((t[0], t[2]), (0, 0), "scan wrote outside total[1]");
    (read(gpu, &output), t[1])
}

#[test]
fn scan_gpu() {
    let Some(gpu) = common::gpu() else { return };
    let p = Primitives::new(&gpu).unwrap();
    let mut rng = Rng(5);
    let mut sizes = SIZES.to_vec();
    sizes.extend([2047, 2048, 2049, 2048 * 256, 2048 * 256 + 1]);
    for n in sizes {
        // Small values keep totals meaningful; the wrapping case is covered by `big`.
        let small: Vec<u32> = (0..n).map(|_| rng.below(5)).collect();
        let big: Vec<u32> = (0..n).map(|_| rng.u32()).collect();
        for kind in [ScanKind::Exclusive, ScanKind::Inclusive] {
            for data in [&small, &big] {
                let want = cpu::scan(kind, data);
                let first = scan_on(&gpu, &p, kind, data);
                assert!(first == want, "{kind:?} n={n}");
                for run in 1..3 {
                    assert!(
                        scan_on(&gpu, &p, kind, data) == first,
                        "{kind:?} n={n} run {run}"
                    );
                }
            }
        }
    }
}

#[test]
fn scan_without_total_and_for_a_zero_total_gpu() {
    let Some(gpu) = common::gpu() else { return };
    let p = Primitives::new(&gpu).unwrap();
    let data = vec![1u32; 5000];
    let input = GpuVec::from_slice(&gpu, "in", &data).unwrap();
    let output = GpuVec::<u32>::zeroed(&gpu, "out", 5000).unwrap();
    let mut cx = new_cx(&gpu);
    p.scan(&mut cx, ScanKind::Inclusive, &input, &output, None)
        .unwrap();
    cx.finish(&job()).unwrap();
    assert_eq!(read(&gpu, &output), cpu::scan(ScanKind::Inclusive, &data).0);
    // An empty input writes a total of 0.
    let empty = GpuVec::<u32>::zeroed(&gpu, "empty", 0).unwrap();
    let total = GpuVec::from_slice(&gpu, "total", &[77u32]).unwrap();
    let mut cx = new_cx(&gpu);
    p.scan(
        &mut cx,
        ScanKind::Exclusive,
        &empty,
        &empty_out(&gpu),
        Some((&total, 0)),
    )
    .unwrap();
    cx.finish(&job()).unwrap();
    assert_eq!(read(&gpu, &total), [0]);
}

fn empty_out(gpu: &Gpu) -> GpuVec<u32> {
    GpuVec::<u32>::zeroed(gpu, "empty out", 0).unwrap()
}

fn compact_on(gpu: &Arc<Gpu>, p: &Primitives, flags: &[u32], values: &[u32]) -> Vec<u32> {
    let f = GpuVec::from_slice(gpu, "flags", flags).unwrap();
    let v = GpuVec::from_slice(gpu, "values", values).unwrap();
    let out = GpuVec::<u32>::zeroed(gpu, "out", flags.len() as u64).unwrap();
    let count = GpuVec::from_slice(gpu, "count", &[99u32]).unwrap();
    let mut cx = new_cx(gpu);
    p.compact(&mut cx, &f, &v, &out, &count).unwrap();
    cx.finish(&job()).unwrap();
    let kept = read(gpu, &count)[0] as usize;
    if flags.is_empty() {
        return Vec::new();
    }
    read(gpu, &out)[..kept].to_vec()
}

#[test]
fn compact_gpu() {
    let Some(gpu) = common::gpu() else { return };
    let p = Primitives::new(&gpu).unwrap();
    let mut rng = Rng(6);
    for n in SIZES {
        // Flags are not 0/1: any non-zero keeps the value.
        let flags: Vec<u32> = (0..n)
            .map(|_| match rng.below(4) {
                0 => 0,
                1 => 1,
                2 => 0,
                _ => rng.u32() | 1,
            })
            .collect();
        let values: Vec<u32> = (0..n).map(|_| rng.u32()).collect();
        let want = cpu::compact(&flags, &values);
        let first = compact_on(&gpu, &p, &flags, &values);
        assert!(first == want, "n={n}");
        for run in 1..3 {
            assert!(
                compact_on(&gpu, &p, &flags, &values) == first,
                "n={n} run {run}"
            );
        }
    }
    // All dropped, all kept.
    let n = 5000;
    let values: Vec<u32> = (0..n as u32).collect();
    assert!(compact_on(&gpu, &p, &vec![0; n], &values).is_empty());
    assert_eq!(compact_on(&gpu, &p, &vec![1; n], &values), values);
}

// ───────────────────────────── sort ─────────────────────────────

fn sort_on(
    gpu: &Arc<Gpu>,
    p: &Primitives,
    keys: &[u32],
    values: Option<&[u32]>,
    key_bits: u32,
) -> (Vec<u32>, Option<Vec<u32>>) {
    let mut k = GpuVec::from_slice(gpu, "keys", keys).unwrap();
    let mut v = values.map(|v| GpuVec::from_slice(gpu, "values", v).unwrap());
    let mut cx = new_cx(gpu);
    p.sort(&mut cx, &mut k, v.as_mut(), key_bits).unwrap();
    cx.finish(&job()).unwrap();
    (read(gpu, &k), v.as_ref().map(|v| read(gpu, v)))
}

/// The oracle: `slice::sort_by_key` on (key, value) pairs.
fn oracle(keys: &[u32], values: &[u32], key_bits: u32) -> (Vec<u32>, Vec<u32>) {
    let mask = if key_bits == 32 {
        u32::MAX
    } else {
        (1 << key_bits) - 1
    };
    let mut pairs: Vec<(u32, u32)> = keys.iter().copied().zip(values.iter().copied()).collect();
    pairs.sort_by_key(|p| p.0 & mask);
    pairs.into_iter().unzip()
}

#[test]
fn sort_matches_the_cpu_reference_gpu() {
    let Some(gpu) = common::gpu() else { return };
    let p = Primitives::new(&gpu).unwrap();
    let mut rng = Rng(7);
    for n in SIZES {
        let keys: Vec<u32> = (0..n).map(|_| rng.u32()).collect();
        let values: Vec<u32> = (0..n as u32).collect();
        for key_bits in [32, 17, 8, 1] {
            let (want_k, want_v) = oracle(&keys, &values, key_bits);
            let first = sort_on(&gpu, &p, &keys, Some(&values), key_bits);
            assert!(first.0 == want_k, "keys n={n} bits={key_bits}");
            assert!(
                first.1.as_deref() == Some(&want_v[..]),
                "values n={n} bits={key_bits}"
            );
            if n <= 1 << 20 {
                // Keys only; and a second and third run must be bit-identical.
                let keys_only = sort_on(&gpu, &p, &keys, None, key_bits);
                assert!(keys_only.0 == want_k, "keys-only n={n} bits={key_bits}");
                for run in 1..3 {
                    let again = sort_on(&gpu, &p, &keys, Some(&values), key_bits);
                    assert!(again == first, "n={n} bits={key_bits} run {run}");
                }
            }
        }
    }
}

#[test]
fn sort_is_stable_property_gpu() {
    let Some(gpu) = common::gpu() else { return };
    let p = Primitives::new(&gpu).unwrap();
    let mut rng = Rng(8);
    for case in 0..40 {
        let n = rng.below(120_000) as usize;
        let key_bits = 1 + rng.below(32);
        // Few distinct keys, so most keys repeat and stability decides the order of values.
        let distinct = 1 + rng.below(if case % 3 == 0 { 3 } else { 300 });
        // Random bits above `key_bits` must not influence the order.
        let keys: Vec<u32> = (0..n)
            .map(|_| {
                rng.below(distinct)
                    | if key_bits < 32 {
                        rng.u32() << key_bits
                    } else {
                        0
                    }
            })
            .collect();
        let values: Vec<u32> = (0..n).map(|_| rng.u32()).collect();
        let (want_k, want_v) = oracle(&keys, &values, key_bits);
        let (got_k, got_v) = sort_on(&gpu, &p, &keys, Some(&values), key_bits);
        assert!(got_k == want_k, "case {case}: n={n} bits={key_bits} keys");
        assert!(
            got_v.as_deref() == Some(&want_v[..]),
            "case {case}: n={n} bits={key_bits} values"
        );
        // Equal keys keep the order of their values (indices make this checkable directly).
        let idx: Vec<u32> = (0..n as u32).collect();
        let (_, got_idx) = sort_on(&gpu, &p, &keys, Some(&idx), key_bits);
        let mask = if key_bits == 32 {
            u32::MAX
        } else {
            (1 << key_bits) - 1
        };
        let got_idx = got_idx.unwrap();
        for w in got_idx.windows(2) {
            let (a, b) = (keys[w[0] as usize] & mask, keys[w[1] as usize] & mask);
            assert!(
                a < b || (a == b && w[0] < w[1]),
                "case {case}: unstable at {w:?}"
            );
        }
    }
}

// ───────────────────────── atomic counter ─────────────────────────

#[test]
fn atomic_counter_allocates_unique_slots_gpu() {
    let Some(gpu) = common::gpu() else { return };
    const N: u32 = 100_000;
    let counter = AtomicCounter::new(&gpu, "counter").unwrap();
    let slots = GpuVec::<u32>::zeroed(&gpu, "slots", u64::from(N)).unwrap();
    let wgsl = r"
struct Params { n: u32, p0: u32, p1: u32, p2: u32 }
@group(0) @binding(0) var<uniform> prm: Params;
@group(0) @binding(1) var<storage, read_write> counter: array<atomic<u32>>;
@group(0) @binding(2) var<storage, read_write> slots: array<u32>;
@compute @workgroup_size(WG)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    let i = amg_linear_id(gid, nwg);
    if (i < prm.n) {
        let slot = atomicAdd(&counter[0], 1u);
        slots[slot] = i + 1u;
    }
}
";
    let k = Kernel::new(&gpu, "alloc", wgsl, "main", &[]).unwrap();
    for round in 0..2 {
        let mut cx = new_cx(&gpu);
        counter.reset(&mut cx);
        let slot = cx.uniform(&[N, 0, 0, 0]);
        let bind = k.bind_group(
            &gpu,
            0,
            &[cx.uniform_binding(slot), counter.binding(), slots.binding()],
        );
        cx.dispatch_1d(&k, &[&bind], u64::from(N));
        cx.finish(&job()).unwrap();
        assert_eq!(counter.read(&gpu).wait().unwrap(), [N], "round {round}");
        let mut seen = read(&gpu, &slots);
        seen.sort_unstable();
        assert!(
            seen.iter().copied().eq(1..=N),
            "round {round}: slots are not a permutation"
        );
    }
    // Reset alone gives 0.
    let mut cx = new_cx(&gpu);
    counter.reset(&mut cx);
    cx.finish(&job()).unwrap();
    assert_eq!(counter.read(&gpu).wait().unwrap(), [0]);
}
