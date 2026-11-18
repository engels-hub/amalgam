//! S20 Testing property tests for the KH-07 operations: affine invariance, reverse ∘ reverse =
//! id, reparametrisation keeps the shape, determinism (bit-identical repeated evaluation).
#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "test code")]

mod common;

use amg_core::{Quat, Transform, Vec3};
use amg_nurbs::UV;
use common::{Rng, curve, diag, samples, surface};
use proptest::prelude::*;

proptest! {
    #![proptest_config(ProptestConfig { cases: 512, ..ProptestConfig::default() })]

    #[test]
    fn curve_invariants(seed in any::<u64>(), p in 1usize..8, rational in any::<bool>(),
                        axis in (-1.0f64..1.0, -1.0f64..1.0, 0.1f64..1.0), angle in -3.0f64..3.0,
                        shift in (-50.0f64..50.0, -50.0f64..50.0, -50.0f64..50.0)) {
        let mut r = Rng(seed);
        let c = curve(&mut r, p, rational);
        let tf = Transform::from_rotation_translation(
            Quat::from_axis_angle(Vec3::new(axis.0, axis.1, axis.2).normalize(), angle),
            Vec3::new(shift.0, shift.1, shift.2),
        );
        let ct = c.transformed(&tf);
        let rr = c.reversed().reversed();
        let (a, b) = c.domain();
        let rp = c.reparametrized((0.0, 1.0));
        let eps = 1e-12 * (diag(c.poles()) + 100.0).max(1.0);
        for t in samples(a, b, 50, seed) {
            let x = c.eval(t);
            prop_assert!((ct.eval(t) - tf.transform_point3(x)).length() <= 10.0 * eps);
            prop_assert!((rr.eval(t) - x).length() <= eps);
            prop_assert!((c.reversed().eval(a + b - t) - x).length() <= eps);
            prop_assert!((rp.eval((t - a) / (b - a)) - x).length() <= eps);
            prop_assert_eq!(c.eval(t).to_array().map(f64::to_bits), x.to_array().map(f64::to_bits));
        }
        prop_assert!(c.bbox().expanded(eps).contains(c.eval(0.5 * (a + b)), &amg_core::Tolerance::DEFAULT));
    }

    #[test]
    fn surface_invariants(seed in any::<u64>(), p in 1usize..6, q in 1usize..6, rational in any::<bool>()) {
        let mut r = Rng(seed);
        let s = surface(&mut r, p, q, rational);
        let [(u0, u1), (v0, v1)] = s.domain();
        let eps = 1e-12 * diag(s.poles().as_slice()).max(1.0);
        let tf = Transform::from_rotation_z(0.7) * Transform::from_translation(Vec3::new(3.0, -1.0, 2.0));
        let st = s.transformed(&tf);
        let su = s.reversed(UV::U);
        let tr = s.transposed();
        for &u in &samples(u0, u1, 7, seed) {
            for &v in &samples(v0, v1, 7, !seed) {
                let x = s.eval(u, v);
                prop_assert!((st.eval(u, v) - tf.transform_point3(x)).length() <= 10.0 * eps);
                prop_assert!((su.eval(u0 + u1 - u, v) - x).length() <= eps);
                prop_assert!((tr.eval(v, u) - x).length() <= eps);
                prop_assert!((s.iso_curve(UV::U, u).eval(v) - x).length() <= eps);
                prop_assert!((s.iso_curve(UV::V, v).eval(u) - x).length() <= eps);
            }
        }
        prop_assert!((s.boundary(amg_nurbs::Side::VMax).eval(0.5 * (u0 + u1)) - s.eval(0.5 * (u0 + u1), v1)).length() <= eps);
    }
}
