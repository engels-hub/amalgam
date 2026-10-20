//! FND-10 budget: parsing a typical quantity literal takes < 1 µs median.

use std::hint::black_box;

use amg_core::{FormatSpec, Quantity, Unit, format_quantity, parse_quantity};
use criterion::{Criterion, criterion_group, criterion_main};

fn parse(c: &mut Criterion) {
    let mut g = c.benchmark_group("units/parse_quantity");
    for lit in ["12.5 mm", "1 1/4\"", "5' 3 1/2\"", "7.85 g/cm^3", "30"] {
        g.bench_function(lit, |b| b.iter(|| parse_quantity(black_box(lit))));
    }
    g.finish();
}

fn format(c: &mut Criterion) {
    let q = Quantity::from_unit(12.5, Unit::Millimetre);
    let spec = FormatSpec::for_unit(Unit::Millimetre);
    c.bench_function("units/format_quantity/decimal", |b| {
        b.iter(|| format_quantity(black_box(q), Unit::Millimetre, &spec))
    });
    c.bench_function("units/format_quantity/exact_inch", |b| {
        b.iter(|| format_quantity(black_box(q), Unit::Inch, &FormatSpec::EXACT))
    });
}

criterion_group!(benches, parse, format);
criterion_main!(benches);
