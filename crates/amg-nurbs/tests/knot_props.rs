//! KH-12 Done-when 1–3: property tests (512 cases each, 1000 samples per curve) for the knot
//! operations: insertion, refinement, elevation, Bézier decomposition, split + join and
//! segment keep the shape within ε = 1e-12·max(1, D); removal keeps the deviation ≤ `tol` and
//! undoes insertion; elevating a Bézier-form curve and removing knots gives the pole count of
//! the elevated original back. Surface counterparts are checked on a 32 × 32 sample grid.
#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "test code")]

mod common;

use std::sync::atomic::{AtomicU64, Ordering};

use amg_core::Tolerance;
use amg_nurbs::{NurbsCurve3, NurbsSurface3, UV};
use common::{Rng, curve, diag, samples, surface};
use proptest::prelude::*;

/// Worst observed values (`error / max(1, D)` or `deviation / tol`), for the measured report.
static WORST_SHAPE: AtomicU64 = AtomicU64::new(0);
static WORST_SURF: AtomicU64 = AtomicU64::new(0);
static WORST_REMOVE_RATIO: AtomicU64 = AtomicU64::new(0);
static WORST_BEZ_ELEV: AtomicU64 = AtomicU64::new(0);
static WORST_RT: AtomicU64 = AtomicU64::new(0);
static REMOVALS: AtomicU64 = AtomicU64::new(0);

fn record(slot: &AtomicU64, v: f64) {
    // Non-negative floats order like their bit patterns.
    slot.fetch_max(v.to_bits(), Ordering::Relaxed);
}

fn read(slot: &AtomicU64) -> f64 {
    f64::from_bits(slot.load(Ordering::Relaxed))
}

fn eps_of(c: &NurbsCurve3) -> f64 {
    1e-12 * diag(c.poles()).max(1.0)
}

/// Max deviation between `a` and `b` over 1000 samples of `a`'s domain.
fn dev(a: &NurbsCurve3, b: &NurbsCurve3, seed: u64) -> f64 {
    let (t0, t1) = a.domain();
    samples(t0, t1, 1000, seed)
        .into_iter()
        .map(|t| (a.eval(t) - b.eval(t)).length())
        .fold(0.0, f64::max)
}

/// Random curve whose neighbouring knot spans differ by at most 4× (random multiplicities
/// 1..=min(p, 2) like `common::knots`).
fn graded_curve(r: &mut Rng, p: usize, rational: bool) -> NurbsCurve3 {
    let n = r.usize(p + 1, p + 8);
    let a = r.range(-5.0, 5.0);
    let len = r.range(0.5, 20.0);
    let mut inner: Vec<usize> = Vec::new(); // multiplicities of distinct interior knots
    let mut left = n - p - 1;
    while left > 0 {
        let m = r.usize(1, p.min(left).clamp(1, 2));
        inner.push(m);
        left -= m;
    }
    let w: Vec<f64> = (0..=inner.len()).map(|_| r.range(1.0, 4.0)).collect();
    let total: f64 = w.iter().sum();
    let mut v = vec![a; p + 1];
    let mut x = a;
    for (i, &m) in inner.iter().enumerate() {
        x += len * w[i] / total;
        v.extend(std::iter::repeat_n(x, m));
    }
    v.extend(std::iter::repeat_n(a + len, p + 1));
    let poles = (0..n).map(|_| r.point(100.0)).collect();
    let wts = rational.then(|| (0..n).map(|_| r.range(0.2, 5.0)).collect());
    NurbsCurve3::new(p, amg_nurbs::KnotVector::new(v).unwrap(), poles, wts).unwrap()
}

fn param(c: &NurbsCurve3, f: f64) -> f64 {
    let (a, b) = c.domain();
    a + (b - a) * f
}

#[allow(clippy::too_many_arguments, reason = "proptest inputs")]
fn curve_ops(
    seed: u64,
    p: usize,
    rational: bool,
    f: f64,
    times: usize,
    by: usize,
    nref: usize,
    f2: f64,
) {
    let mut r = Rng(seed);
    let c = curve(&mut r, p, rational);
    let eps = eps_of(&c);
    let t = param(&c, f);
    let (a, b) = c.domain();

    // Insertion (A5.1).
    let ins = c.insert_knot(t, times).unwrap();
    let e_ins = dev(&c, &ins, seed);
    assert!(e_ins <= eps, "insert {e_ins:e} > {eps:e}");
    assert_eq!(ins.domain(), c.domain());
    // Determinism.
    assert_eq!(&ins, &c.insert_knot(t, times).unwrap());

    // Refinement (A5.4) with random new knots, some doubled.
    let mut xs: Vec<f64> = (0..nref)
        .map(|_| a + (b - a) * r.range(0.001, 0.999))
        .collect();
    if p >= 2 {
        xs.push(xs[0]);
    }
    let refd = c.refine_knots(&xs).unwrap();
    assert_eq!(refd.poles().len(), c.poles().len() + xs.len());
    let e_ref = dev(&c, &refd, seed ^ 1);
    assert!(e_ref <= eps, "refine {e_ref:e} > {eps:e}");

    // Degree elevation (A5.9).
    let el = c.elevate_degree(by).unwrap();
    assert_eq!(el.degree(), p + by);
    let e_el = dev(&c, &el, seed ^ 2);
    assert!(e_el <= eps, "elevate {e_el:e} > {eps:e}");
    // Elevated knot vector: every distinct knot's multiplicity grows by `by`.
    let (k0, m0) = c.knots().to_knots_mults(0.0);
    let (k1, m1) = el.knots().to_knots_mults(0.0);
    assert_eq!(&k0, &k1);
    assert!(m0.iter().zip(&m1).all(|(x, y)| x + by == *y));

    // Bézier decomposition (A5.6): each segment reproduces its span.
    let segs = c.bezier_segments();
    assert_eq!(segs.len(), k0.len() - 1);
    let mut e_bez = 0.0f64;
    for (i, s) in segs.iter().enumerate() {
        assert_eq!(s.poles().len(), p + 1);
        let (sa, sb) = s.domain();
        assert_eq!((sa, sb), (k0[i], k0[i + 1]));
        for u in samples(sa, sb, 1000 / segs.len() + 2, seed ^ i as u64) {
            e_bez = e_bez.max((s.eval(u) - c.eval(u)).length());
        }
    }
    assert!(e_bez <= eps, "bezier {e_bez:e} > {eps:e}");

    // Split + join = id; segment.
    let (lft, rgt) = c.split(t).unwrap();
    assert_eq!(lft.end(), rgt.start());
    assert_eq!(lft.domain().1, rgt.domain().0);
    let j = lft.join(&rgt, &Tolerance::DEFAULT).unwrap();
    assert_eq!(j.domain(), c.domain());
    let e_join = dev(&c, &j, seed ^ 3);
    assert!(e_join <= eps, "split + join {e_join:e} > {eps:e}");
    let t2 = param(&c, f2);
    let (s0, s1) = if t < t2 { (t, t2) } else { (t2, t) };
    if s1 - s0 > 1e-6 * (b - a) {
        let seg = c.segment(s0, s1).unwrap();
        assert_eq!(seg.domain(), (s0, s1));
        let e_seg = dev(&seg, &c, seed ^ 4);
        assert!(e_seg <= eps, "segment {e_seg:e} > {eps:e}");
        record(&WORST_SHAPE, e_seg / eps * 1e-12);
    }
    for e in [e_ins, e_ref, e_el, e_bez, e_join] {
        record(&WORST_SHAPE, e / eps * 1e-12);
    }
}

fn surface_ops(seed: u64, p: usize, q: usize, rational: bool, f: f64, dir: bool, by: usize) {
    let mut r = Rng(seed);
    let s = surface(&mut r, p, q, rational);
    let d = if dir { UV::U } else { UV::V };
    let eps = 1e-12 * diag(s.poles().as_slice()).max(1.0);
    let [(u0, u1), (v0, v1)] = s.domain();
    let (a, b) = if dir { (u0, u1) } else { (v0, v1) };
    let t = a + (b - a) * f;
    let us = samples(u0, u1, 32, seed);
    let vs = samples(v0, v1, 32, !seed);
    let devs = |o: &NurbsSurface3| {
        let mut e = 0.0f64;
        for &u in &us {
            for &v in &vs {
                e = e.max((o.eval(u, v) - s.eval(u, v)).length());
            }
        }
        e
    };
    let ins = s.insert_knot(d, t, 2).unwrap();
    let xs: Vec<f64> = (0..6)
        .map(|_| a + (b - a) * r.range(0.001, 0.999))
        .collect();
    let refd = s.refine_knots(d, &xs).unwrap();
    let el = s.elevate_degree(d, by).unwrap();
    assert_eq!(el.degree(d), s.degree(d) + by);
    for (name, o) in [("insert", &ins), ("refine", &refd), ("elevate", &el)] {
        let e = devs(o);
        assert!(e <= eps, "{name} {e:e} > {eps:e}");
        record(&WORST_SURF, e / eps * 1e-12);
    }
    // Split: halves reproduce their part and share the boundary bit-identically.
    let (lo, hi) = s.split(d, t).unwrap();
    let (lside, hside) = match d {
        UV::U => (amg_nurbs::Side::UMax, amg_nurbs::Side::UMin),
        UV::V => (amg_nurbs::Side::VMax, amg_nurbs::Side::VMin),
    };
    assert_eq!(lo.boundary(lside).poles(), hi.boundary(hside).poles());
    let mut e = 0.0f64;
    for &u in &us {
        for &v in &vs {
            let w = if dir { u } else { v };
            let half = if w <= t { &lo } else { &hi };
            e = e.max((half.eval(u, v) - s.eval(u, v)).length());
        }
    }
    assert!(e <= eps, "split {e:e} > {eps:e}");
    // Bézier strips (A5.7).
    let strips = s.bezier_strips(d);
    let mut e = 0.0f64;
    for st in &strips {
        assert_eq!(st.knots(d).len(), 2 * (s.degree(d) + 1));
        let [(su0, su1), (sv0, sv1)] = st.domain();
        for &u in &samples(su0, su1, 8, seed) {
            for &v in &samples(sv0, sv1, 8, !seed) {
                e = e.max((st.eval(u, v) - s.eval(u, v)).length());
            }
        }
    }
    assert!(e <= eps, "strips {e:e} > {eps:e}");
    record(&WORST_SURF, e / eps * 1e-12);
}

/// Done-when 2: removal after insertion is a round trip; removal with `tol` never moves the
/// curve by more than `tol`.
fn removal(seed: u64, p: usize, rational: bool, f: f64, times: usize, ltol: f64) {
    let mut r = Rng(seed);
    let c = curve(&mut r, p, rational);
    let eps = eps_of(&c);
    let t = param(&c, f);
    let k = times.min(p);
    let ins = c.insert_knot(t, k).unwrap();
    let ti = ins.knots().as_slice()[ins.knots().as_slice().partition_point(|&x| x < t)];
    // Round-trip tolerance 1e-9·max(1, D) (the measured error is at rounding level; the
    // rational bound of eq. 5.30 is conservative by (1 + |P|max) / w_min).
    let rt_tol = 1e3 * eps;
    let (back, removed) = ins.remove_knot(ti, k, rt_tol);
    assert_eq!(removed, k);
    assert_eq!(back.poles().len(), c.poles().len());
    assert_eq!(back.knots().len(), c.knots().len());
    let e_rt = dev(&c, &back, seed);
    assert!(e_rt <= rt_tol, "round trip {e_rt:e} > {rt_tol:e}");
    record(&WORST_RT, e_rt / eps * 1e-12);

    // Approximate removal of an existing interior knot.
    let (ks, _) = c.knots().to_knots_mults(0.0);
    if ks.len() > 2 {
        let kt = ks[1 + r.usize(0, ks.len() - 3)];
        let tol = 10f64.powf(ltol) * diag(c.poles()).max(1.0) * 1e-2;
        let (cr, n) = c.remove_knot(kt, p, tol);
        let e = dev(&c, &cr, seed ^ 7);
        assert!(e <= tol, "removed {n}: deviation {e:e} > tol {tol:e}");
        if n > 0 {
            record(&WORST_REMOVE_RATIO, e / tol);
            REMOVALS.fetch_add(1, Ordering::Relaxed);
        }
    }

    // Approximate removal near the bound: perturb the inserted curve by δ·D, remove with a
    // tolerance around δ·D; whatever is removed must stay within tol.
    let dd = diag(c.poles()).max(1.0);
    let delta = 10f64.powf(-8.0 + 6.0 * f) * dd;
    let poles: Vec<_> = ins.poles().iter().map(|&q| q + r.point(delta)).collect();
    let pert = NurbsCurve3::new(
        p,
        ins.knots().clone(),
        poles,
        ins.weights().map(<[f64]>::to_vec),
    )
    .unwrap();
    let tol = delta * 10f64.powf(ltol + 1.0);
    let (cr, n) = pert.remove_knot(ti, k, tol);
    let e = dev(&pert, &cr, seed ^ 9);
    assert!(
        e <= tol,
        "perturbed: removed {n}: deviation {e:e} > tol {tol:e}"
    );
    if n > 0 {
        record(&WORST_REMOVE_RATIO, e / tol);
        REMOVALS.fetch_add(1, Ordering::Relaxed);
    }
}

/// Done-when 3: a curve brought into Bézier form (every interior knot at multiplicity p),
/// elevated by k, then cleaned by knot removal has exactly the pole count of the elevated
/// original (the redundant knots are all removable), and the same shape.
/// Removal tolerance 1e-9·max(1, D). `graded` curves have neighbouring spans within 4× of
/// each other; the others use `common::knots` (spans down to ~1e-5 of their neighbours).
fn bezier_form_elevate_remove(seed: u64, p: usize, by: usize, rational: bool, graded: bool) {
    let mut r = Rng(seed);
    let c = if graded {
        graded_curve(&mut r, p, rational)
    } else {
        curve(&mut r, p, rational)
    };
    let eps = eps_of(&c);
    let rm_tol = 1e3 * eps;
    let (ks, ms) = c.knots().to_knots_mults(0.0);
    let mut xs = Vec::new();
    for (k, m) in ks[1..ks.len() - 1].iter().zip(&ms[1..ms.len() - 1]) {
        xs.extend(std::iter::repeat_n(*k, p - m));
    }
    let bez = c.refine_knots(&xs).unwrap();
    let target = c.elevate_degree(by).unwrap();
    let mut e = bez.elevate_degree(by).unwrap();
    for (k, m) in ks[1..ks.len() - 1].iter().zip(&ms[1..ms.len() - 1]) {
        let (next, n) = e.remove_knot(*k, p - m, rm_tol);
        assert_eq!(n, p - m, "knot {} kept copies", k);
        e = next;
    }
    assert_eq!(e.poles().len(), target.poles().len());
    assert_eq!(e.knots().as_slice(), target.knots().as_slice());
    let d = dev(&c, &e, seed);
    assert!(d <= rm_tol, "{d:e} > removal tol {rm_tol:e}");
    record(&WORST_BEZ_ELEV, d / eps * 1e-12);
    // A single Bézier input stays Bézier: elevation adds exactly `by` poles and leaves
    // nothing to remove.
    let b0 = &c.bezier_segments()[0];
    let be = b0.elevate_degree(by).unwrap();
    assert_eq!(be.poles().len(), p + 1 + by);
    assert!(dev(b0, &be, seed) <= eps);
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 512, ..ProptestConfig::default() })]

    #[test]
    fn prop_curve_ops(seed in any::<u64>(), p in 1usize..8, rational in any::<bool>(),
                      f in 0.01f64..0.99, times in 1usize..8, by in 1usize..4,
                      nref in 1usize..20, f2 in 0.01f64..0.99) {
        curve_ops(seed, p, rational, f, times, by, nref, f2);
    }

    #[test]
    fn prop_surface_ops(seed in any::<u64>(), p in 1usize..6, q in 1usize..6,
                        rational in any::<bool>(), f in 0.01f64..0.99,
                        dir in any::<bool>(), by in 1usize..3) {
        surface_ops(seed, p, q, rational, f, dir, by);
    }

    #[test]
    fn prop_removal(seed in any::<u64>(), p in 1usize..8, rational in any::<bool>(),
                    f in 0.01f64..0.99, times in 1usize..8, ltol in -3.0f64..1.0) {
        removal(seed, p, rational, f, times, ltol);
    }

    #[test]
    fn prop_bezier_form_elevate_remove(seed in any::<u64>(), p in 1usize..7, by in 1usize..4,
                                       rational in any::<bool>(), graded in any::<bool>()) {
        bezier_form_elevate_remove(seed, p, by, rational, graded);
    }
}

/// Deterministic sweep over the same checks (512 seeded cases each) that prints the worst
/// measured values for the KH-12 report.
#[test]
fn measured_worst() {
    let mut r = Rng(0x0c12_0001);
    for i in 0..512usize {
        let seed = r.next_u64();
        let p = 1 + i % 7;
        let rational = (i / 7) % 2 == 1;
        curve_ops(
            seed,
            p,
            rational,
            r.range(0.01, 0.99),
            r.usize(1, 7),
            r.usize(1, 3),
            r.usize(1, 19),
            r.range(0.01, 0.99),
        );
        surface_ops(
            seed,
            1 + i % 5,
            1 + (i / 5) % 5,
            rational,
            r.range(0.01, 0.99),
            i % 2 == 0,
            r.usize(1, 2),
        );
        removal(
            seed,
            p,
            rational,
            r.range(0.01, 0.99),
            r.usize(1, 7),
            r.range(-3.0, 1.0),
        );
        bezier_form_elevate_remove(seed, 1 + i % 6, r.usize(1, 3), rational, true);
        bezier_form_elevate_remove(seed, 1 + i % 6, r.usize(1, 3), rational, false);
    }
    eprintln!(
        "KH-12 measured over 512 cases: curve ops worst {:e}·max(1,D); surface ops worst {:e}·max(1,D); insert+remove round trip worst {:e}·max(1,D); approximate removals {} with worst deviation/tol {:.3e}; Bézier-form elevate+remove worst {:e}·max(1,D)",
        read(&WORST_SHAPE),
        read(&WORST_SURF),
        read(&WORST_RT),
        REMOVALS.load(Ordering::Relaxed),
        read(&WORST_REMOVE_RATIO),
        read(&WORST_BEZ_ELEV),
    );
}
