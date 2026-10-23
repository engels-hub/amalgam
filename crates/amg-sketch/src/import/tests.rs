//! Unit tests for the import pipeline.

#![allow(clippy::float_cmp, reason = "exact table constants and snapped points")]

use amg_core::Point2;

use super::{Item, Prim, ccw_sweep, insunits_mm, merge_ends};
use crate::model::End;

fn line(a: (f64, f64), b: (f64, f64)) -> Item {
    Item {
        layer: String::new(),
        construction: false,
        prim: Prim::Line {
            a: Point2::new(a.0, a.1),
            b: Point2::new(b.0, b.1),
        },
    }
}

#[test]
fn insunits_table() {
    assert_eq!(insunits_mm(0), None);
    assert_eq!(insunits_mm(99), None);
    assert_eq!(insunits_mm(4), Some(1.0));
    assert_eq!(insunits_mm(1), Some(25.4));
    assert_eq!(insunits_mm(2), Some(304.8));
    assert_eq!(insunits_mm(5), Some(10.0));
    assert_eq!(insunits_mm(6), Some(1000.0));
    assert_eq!(insunits_mm(13), Some(0.001));
}

#[test]
fn sweep_is_ccw_and_full_for_equal_angles() {
    assert!((ccw_sweep(0.0, 1.0) - 1.0).abs() < 1e-15);
    assert!((ccw_sweep(1.0, 0.0) - (std::f64::consts::TAU - 1.0)).abs() < 1e-15);
    assert_eq!(ccw_sweep(2.0, 2.0), std::f64::consts::TAU);
}

#[test]
fn chain_of_near_ends_is_one_cluster() {
    // Single linkage: 0 - 0.6 tol - 1.2 tol are one cluster although the outer two are
    // farther apart than the tolerance.
    let tol = 1e-3;
    let mut items = vec![
        line((0.0, 0.0), (10.0, 0.0)),
        line((10.0 + 0.6e-3, 0.0), (20.0, 0.0)),
        line((10.0 + 1.2e-3, 0.0), (10.0, 5.0)),
    ];
    let m = merge_ends(&mut items, tol);
    assert_eq!(m.groups.len(), 1);
    assert_eq!(m.groups[0].len(), 3);
    assert_eq!(m.moved, 2);
    let ends: Vec<Point2> = [(0, End::End), (1, End::Start), (2, End::Start)]
        .iter()
        .map(|&(i, e)| items[i].prim.end_pos(e).unwrap())
        .collect();
    assert!(ends.windows(2).all(|w| w[0] == w[1]));
}

#[test]
fn elliptical_arc_ends_stay_put() {
    let mut items = vec![
        line((10.0, 0.0), (10.0, 5.0)),
        Item {
            layer: String::new(),
            construction: false,
            prim: Prim::EllipticArc {
                c: Point2::ZERO,
                major_end: Point2::new(10.0, 0.0),
                minor: 5.0,
                t0: 0.0,
                t1: 1.0,
            },
        },
    ];
    let before = items[1].prim.end_pos(End::Start).unwrap();
    let m = merge_ends(&mut items, 1e-3);
    assert_eq!(m.groups.len(), 1);
    // The elliptical arc is the representative; the line end moved onto it.
    assert_eq!(m.groups[0][0].0, 1);
    assert_eq!(items[1].prim.end_pos(End::Start).unwrap(), before);
    assert_eq!(items[0].prim.end_pos(End::Start).unwrap(), before);
}
