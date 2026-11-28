//! SOL-29 Done-when 2 (correctness half): the 50-feature part builds, stays healthy through
//! edits of its first sketch, and the edit reaches the features downstream. The timing half is
//! `benches/sol_recompute.rs`. Needs the `occt` feature (AMG_OCCT_DIR).

#![cfg(feature = "occt")]
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout,
    missing_docs,
    reason = "tests"
)]

#[path = "solid_recompute/part.rs"]
mod part;
mod solid_common;

use std::time::Duration;

use amg_brep::{BrepKernel, CheckOpts};
use part::{FEATURES, WIDTH, build_part};
use solid_common::{brep, kernel, volume};

#[test]
fn the_50_feature_part_recomputes_after_an_edit_to_the_first_sketch() {
    let mut p = build_part();
    assert_eq!(p.features.len(), FEATURES);
    assert_eq!(p.doc.design().timeline().nodes().count(), FEATURES);
    let kinds: std::collections::BTreeSet<&str> = p
        .doc
        .design()
        .timeline()
        .nodes()
        .map(|n| n.feature.kind())
        .collect();
    for kind in [
        "sketch.sketch",
        "solid.extrude",
        "solid.fillet",
        "solid.chamfer",
        "solid.shell",
        "solid.pattern",
    ] {
        assert!(kinds.contains(kind), "{kind} missing from {kinds:?}");
    }

    let check = |p: &part::Part, width: f64| {
        assert!(p.unhealthy().is_empty(), "{:?}", p.unhealthy());
        assert_eq!(p.doc.evaluated().bodies().count(), 1);
        let s = brep(&p.doc, p.body);
        assert_eq!(s.solids().len(), 1);
        assert!(kernel().check(&s, &CheckOpts::default()).valid, "valid");
        let b = s.bbox();
        assert!(
            (b.min.x).abs() < 1e-6 && (b.max.x - width).abs() < 1e-6,
            "width {width}: {b:?}"
        );
        volume(&s)
    };
    let v0 = check(&p, WIDTH);
    println!(
        "final body: {} faces",
        solid_common::faces(&brep(&p.doc, p.body))
    );

    p.set_width(104.0);
    let v1 = check(&p, 104.0);
    // 4 mm more of floor (4 × 60 × 2 = 480 mm³) and of the two long walls above it
    // (2 × 4 × 2 × 18 = 288 mm³), less the slivers the bottom fillet and rim chamfer take.
    println!("volume {v0:.3} -> {v1:.3} mm³ ({:+.3})", v1 - v0);
    assert!(v1 - v0 > 740.0 && v1 - v0 < 768.0, "{}", v1 - v0);

    // Everything built on the body was computed again, not read from the cache: all extrudes,
    // blends, the shell and the patterns, plus the edited sketch and the face sketch.
    let recomputed: Vec<&str> = p
        .features
        .iter()
        .filter(|(id, _)| p.doc.evaluated().node(*id).unwrap().duration > Duration::ZERO)
        .map(|(_, label)| label.as_str())
        .collect();
    println!("recomputed {} of {FEATURES}", recomputed.len());
    for (_, label) in &p.features {
        // A sketch on an origin plane does not read the body, so it stays cached.
        let off_the_body = label.ends_with(" sketch")
            && label != "outline sketch"
            && label != "back cut-out sketch";
        assert_eq!(
            recomputed.contains(&label.as_str()),
            !off_the_body,
            "{label}"
        );
    }

    // Back again, and once more to a new width.
    p.set_width(WIDTH);
    let back = check(&p, WIDTH);
    assert!((back - v0).abs() <= 1e-9 * v0, "{back} vs {v0}");
    p.set_width(101.5);
    check(&p, 101.5);
}
