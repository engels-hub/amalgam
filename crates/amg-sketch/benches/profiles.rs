//! SK-06 budget: profile detection for a 500-curve sketch < 5 ms (5000 curves < 100 ms).
//!
//! The sketch is a grid of rectangles (4 lines each, sharing point entities) with a circle
//! inside every rectangle, so every profile is a rectangle with a circular hole plus an island.
//! `profiles/500` is 100 rectangles + 100 circles = 500 curves; `profiles/5000` is 1000 + 1000
//! = 5000.

#![allow(
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    reason = "bench setup"
)]

use std::hint::black_box;

use amg_core::{Plane, Point2};
use amg_sketch::{ProfileOptions, Sketch, detect_profiles};
use criterion::{Criterion, criterion_group, criterion_main};

/// `cols * rows` rectangles of 8 x 8 mm on a 10 mm pitch, each holding a circle.
fn sketch(cols: usize, rows: usize) -> Sketch {
    let mut s = Sketch::new(Plane::XY);
    for i in 0..cols {
        for j in 0..rows {
            let (x, y) = (i as f64 * 10.0, j as f64 * 10.0);
            let pts: Vec<_> = [(0.0, 0.0), (8.0, 0.0), (8.0, 8.0), (0.0, 8.0)]
                .iter()
                .map(|&(dx, dy)| s.add_point(Point2::new(x + dx, y + dy)).unwrap())
                .collect();
            for k in 0..4 {
                s.add_line_between(pts[k], pts[(k + 1) % 4]).unwrap();
            }
            s.add_circle(Point2::new(x + 4.0, y + 4.0), 2.5).unwrap();
        }
    }
    s
}

fn bench(c: &mut Criterion) {
    let opts = ProfileOptions::default();
    let mut g = c.benchmark_group("profiles");
    for (name, cols, rows) in [("500", 10, 10), ("5000", 40, 25)] {
        let s = sketch(cols, rows);
        let n = detect_profiles(&s, &opts).profiles.len();
        assert_eq!(n, 2 * cols * rows, "frame with hole + island per cell");
        g.bench_function(name, |b| b.iter(|| detect_profiles(black_box(&s), &opts)));
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
