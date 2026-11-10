//! CPU references with the semantics of the GPU primitives.
//!
//! Reductions replay the GPU's tree order (see the [module docs](super)), so they are the
//! bit-exact expectation for f32 and f64 sums, and a plain deterministic fallback otherwise.

use super::scalar::{ReduceOp, ReduceScalar, ScanKind};
use super::{PER_THREAD, THREADS, TILE};
use crate::precision::GpuReal;

/// One tree pass: one result per tile of [`TILE`] elements (one tile even for an empty input).
fn tree_pass<T: ReduceScalar>(op: ReduceOp, n: usize, load: impl Fn(usize) -> T) -> Vec<T> {
    let tiles = n.div_ceil(TILE).max(1);
    let mut out = Vec::with_capacity(tiles);
    let mut sh = [T::identity(op); THREADS];
    for g in 0..tiles {
        let base = g * TILE;
        for (t, slot) in sh.iter_mut().enumerate() {
            let mut acc = T::identity(op);
            for k in 0..PER_THREAD {
                let i = base + k * THREADS + t;
                if i < n {
                    acc = T::combine(op, acc, load(i));
                }
            }
            *slot = acc;
        }
        out.push(tree(op, &mut sh));
    }
    out
}

/// Combines `sh` by halving strides.
fn tree<T: ReduceScalar>(op: ReduceOp, sh: &mut [T; THREADS]) -> T {
    let mut s = THREADS / 2;
    while s > 0 {
        for t in 0..s {
            sh[t] = T::combine(op, sh[t], sh[t + s]);
        }
        s /= 2;
    }
    sh[0]
}

/// Reduces `v` in the GPU's tree order. An empty slice gives the identity of `op`.
pub fn reduce<T: ReduceScalar>(op: ReduceOp, v: &[T]) -> T {
    let mut cur = tree_pass(op, v.len(), |i| v[i]);
    while cur.len() > 1 {
        let prev = cur;
        cur = tree_pass(op, prev.len(), |i| prev[i]);
    }
    cur[0]
}

/// `sum(a[i] * b[i])`: the products are rounded first, then summed in tree order.
///
/// # Panics
/// If the slices differ in length.
pub fn dot<R: GpuReal>(a: &[R], b: &[R]) -> R {
    assert_eq!(a.len(), b.len(), "dot of slices of different length");
    // The f64 product of two f32 values is exact, so rounding it to f32 is the correctly
    // rounded f32 product, which is what the GPU's multiply returns. For f64 it is the product.
    let products: Vec<R> = a
        .iter()
        .zip(b)
        .map(|(&x, &y)| R::from_f64(x.to_f64() * y.to_f64()))
        .collect();
    reduce(ReduceOp::Sum, &products)
}

/// Exclusive or inclusive prefix sums (wrapping) and the total of all elements.
pub fn scan(kind: ScanKind, v: &[u32]) -> (Vec<u32>, u32) {
    let mut out = Vec::with_capacity(v.len());
    let mut sum = 0u32;
    for &x in v {
        match kind {
            ScanKind::Exclusive => {
                out.push(sum);
                sum = sum.wrapping_add(x);
            }
            ScanKind::Inclusive => {
                sum = sum.wrapping_add(x);
                out.push(sum);
            }
        }
    }
    (out, sum)
}

/// `values[i]` where `flags[i] != 0`, in order.
///
/// # Panics
/// If the slices differ in length.
pub fn compact(flags: &[u32], values: &[u32]) -> Vec<u32> {
    assert_eq!(flags.len(), values.len(), "compact: flags/values length");
    flags
        .iter()
        .zip(values)
        .filter(|(f, _)| **f != 0)
        .map(|(_, v)| *v)
        .collect()
}

/// Reduces every segment `values[offsets[s]..offsets[s + 1]]` in the GPU's order (thread `t`
/// folds `t, t + 256, …` of the segment, then the halving tree). Empty segments give the
/// identity of `op`. Offsets must be non-decreasing; ranges past `values` are clamped.
pub fn segmented_reduce<R: GpuReal>(op: ReduceOp, values: &[R], offsets: &[u32]) -> Vec<R> {
    let segments = offsets.len().saturating_sub(1);
    let mut out = Vec::with_capacity(segments);
    let mut sh = [R::identity(op); THREADS];
    for s in 0..segments {
        let end = (offsets[s + 1] as usize).min(values.len());
        let start = (offsets[s] as usize).min(end);
        for (t, slot) in sh.iter_mut().enumerate() {
            let mut acc = R::identity(op);
            let mut i = start + t;
            while i < end {
                acc = R::combine(op, acc, values[i]);
                i += THREADS;
            }
            *slot = acc;
        }
        out.push(tree(op, &mut sh));
    }
    out
}

/// Stable sort of `keys` by their low `key_bits` bits (1..=32), `values` moving with them.
/// Bits above `key_bits` do not take part in the order.
///
/// # Panics
/// If `key_bits` is not in `1..=32` or `values` has a different length than `keys`.
pub fn sort(keys: &mut [u32], values: Option<&mut [u32]>, key_bits: u32) {
    assert!((1..=32).contains(&key_bits), "key_bits must be in 1..=32");
    let mask = if key_bits == 32 {
        u32::MAX
    } else {
        (1u32 << key_bits) - 1
    };
    let mut order: Vec<u32> = (0..keys.len() as u32).collect();
    order.sort_by_key(|&i| keys[i as usize] & mask);
    let sorted_keys: Vec<u32> = order.iter().map(|&i| keys[i as usize]).collect();
    keys.copy_from_slice(&sorted_keys);
    if let Some(values) = values {
        assert_eq!(values.len(), order.len(), "sort: keys/values length");
        let sorted: Vec<u32> = order.iter().map(|&i| values[i as usize]).collect();
        values.copy_from_slice(&sorted);
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "the tree-order sums are exact, compared bit for bit"
    )]
    use super::*;

    #[test]
    fn empty_reductions_give_the_identity() {
        assert_eq!(reduce::<u32>(ReduceOp::Sum, &[]), 0);
        assert_eq!(reduce::<u32>(ReduceOp::Min, &[]), u32::MAX);
        assert_eq!(reduce::<i32>(ReduceOp::Max, &[]), i32::MIN);
        assert_eq!(reduce::<f32>(ReduceOp::Min, &[]), f32::MAX);
        assert_eq!(reduce::<f64>(ReduceOp::Max, &[]), f64::MIN);
    }

    #[test]
    fn integer_reductions_match_the_obvious_fold() {
        let v: Vec<u32> = (0..10_000u32)
            .map(|i| i.wrapping_mul(2_654_435_761))
            .collect();
        let sum = v.iter().fold(0u32, |a, &b| a.wrapping_add(b));
        assert_eq!(reduce(ReduceOp::Sum, &v), sum);
        assert_eq!(reduce(ReduceOp::Min, &v), *v.iter().min().unwrap());
        assert_eq!(reduce(ReduceOp::Max, &v), *v.iter().max().unwrap());
        let w: Vec<i32> = v.iter().map(|&x| x as i32).collect();
        assert_eq!(reduce(ReduceOp::Min, &w), *w.iter().min().unwrap());
    }

    #[test]
    fn float_sum_of_exact_values_is_exact_in_any_order() {
        let v: Vec<f64> = (0..5000).map(f64::from).collect();
        assert_eq!(reduce(ReduceOp::Sum, &v), 5000.0 * 4999.0 / 2.0);
        let a: Vec<f32> = vec![1.5; 3000];
        let b: Vec<f32> = vec![2.0; 3000];
        assert_eq!(dot(&a, &b), 9000.0);
    }

    #[test]
    fn scan_kinds() {
        let (ex, total) = scan(ScanKind::Exclusive, &[3, 1, 4, 1]);
        assert_eq!((ex, total), (vec![0, 3, 4, 8], 9));
        let (inc, total) = scan(ScanKind::Inclusive, &[3, 1, 4, 1]);
        assert_eq!((inc, total), (vec![3, 4, 8, 9], 9));
        assert_eq!(scan(ScanKind::Exclusive, &[]), (vec![], 0));
    }

    #[test]
    fn compaction_keeps_flagged_values_in_order() {
        assert_eq!(
            compact(&[1, 0, 7, 0, 1], &[10, 11, 12, 13, 14]),
            [10, 12, 14]
        );
        assert!(compact(&[], &[]).is_empty());
    }

    #[test]
    fn segmented_reduce_handles_empty_and_clamped_segments() {
        let none = segmented_reduce::<f32>(ReduceOp::Sum, &[], &[0]);
        assert!(none.is_empty());
        let v: Vec<f64> = (1..=10).map(f64::from).collect();
        let r = segmented_reduce(ReduceOp::Sum, &v, &[0, 0, 3, 3, 10, 99]);
        assert_eq!(r, [0.0, 6.0, 0.0, 49.0, 0.0]);
    }

    #[test]
    fn sort_is_stable_and_respects_key_bits() {
        let mut k = vec![5, 1, 5, 3, 1, 0x103];
        let mut v = vec![0, 1, 2, 3, 4, 5];
        sort(&mut k, Some(&mut v), 4);
        assert_eq!(k, [1, 1, 3, 0x103, 5, 5]);
        assert_eq!(v, [1, 4, 3, 5, 0, 2]);
    }
}
