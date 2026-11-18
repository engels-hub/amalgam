//! KH-07 Done-when 1: P&T Ex. 2.3 basis values to 1e-15, partition of unity to 1e-14.
#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "test code")]

mod common;

use amg_nurbs::basis::{basis_funs, ders_basis_funs};
use amg_nurbs::{KnotVector, MAX_DEGREE};
use common::{Rng, knots, samples};

#[test]
fn ex_2_3_basis_values() {
    // U = {0,0,0,1,2,3,4,4,5,5,5}, p = 2, u = 5/2 → span 4.
    let k = KnotVector::new(vec![0., 0., 0., 1., 2., 3., 4., 4., 5., 5., 5.]).unwrap();
    let span = k.find_span(2, 8, 2.5);
    assert_eq!(span, 4);
    let mut n = [0.0; 3];
    basis_funs(k.as_slice(), span, 2.5, 2, &mut n);
    let want = [1.0 / 8.0, 6.0 / 8.0, 1.0 / 8.0];
    for (got, want) in n.iter().zip(want) {
        assert!((got - want).abs() <= 1e-15, "{n:?}");
    }
}

#[test]
fn partition_of_unity() {
    let mut r = Rng(0x5eed_0001);
    let mut worst = 0.0f64;
    let mut worst_d = 0.0f64;
    for p in 1..=MAX_DEGREE.min(9) {
        for _ in 0..4 {
            let n = r.usize(p + 1, p + 10);
            let k = knots(&mut r, p, n);
            let (a, b) = k.domain(p);
            for u in samples(a, b, 1000, r.next_u64()) {
                let span = k.find_span(p, n, u);
                let mut out = [0.0; MAX_DEGREE + 1];
                basis_funs(k.as_slice(), span, u, p, &mut out);
                let s: f64 = out[..=p].iter().sum();
                assert!(
                    out[..=p].iter().all(|&x| x >= -1e-15),
                    "negative basis {out:?}"
                );
                worst = worst.max((s - 1.0).abs());
                // Derivatives of a partition of unity sum to zero (scaled by the span length).
                let mut d = [[0.0; MAX_DEGREE + 1]; 2];
                ders_basis_funs(k.as_slice(), span, u, p, 1, &mut d);
                let s0: f64 = d[0][..=p].iter().sum();
                worst = worst.max((s0 - 1.0).abs());
                let scale: f64 = d[1][..=p].iter().map(|x| x.abs()).sum::<f64>().max(1.0);
                let s1: f64 = d[1][..=p].iter().sum();
                worst_d = worst_d.max(s1.abs() / scale);
            }
        }
    }
    eprintln!("partition of unity: max |sum - 1| = {worst:e}, derivative sum {worst_d:e}");
    assert!(worst <= 1e-14, "partition of unity error {worst:e}");
    assert!(worst_d <= 1e-13, "derivative sum {worst_d:e}");
}
