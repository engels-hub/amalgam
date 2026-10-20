//! Expression tests: grammar, units, functions, bare numbers, errors with spans, printing
//! round-trip, and property tests (round-trip of random expressions, no-panic fuzzing).

use std::f64::consts::{E, PI};

use amg_core::{Dimension as D, LengthUnit, Quantity, Unit};
use proptest::prelude::*;

use super::*;
use crate::model::UnitPrefs;

const L: D = D::LENGTH;
const A: D = D::ANGLE;
const N: D = D::NONE;

fn env(name: &str) -> Option<Quantity> {
    Some(match name {
        "d1" => Quantity::new(10.0, L),
        "d2" => Quantity::new(20.0, L),
        "ang" => Quantity::new(30f64.to_radians(), A),
        "n" => Quantity::dimensionless(4.0),
        "F" => Quantity::new(100.0, D::FORCE),
        _ => return None,
    })
}

fn opts() -> EvalOptions {
    EvalOptions::new(&UnitPrefs::default())
}

fn ev_with(src: &str, o: &EvalOptions) -> Result<Quantity, ExprError> {
    eval(&parse(src)?, &env, o)
}

fn ev(src: &str) -> Result<Quantity, ExprError> {
    ev_with(src, &opts())
}

#[track_caller]
fn check(src: &str, r: Result<Quantity, ExprError>, value: f64, dim: D) {
    let q = r.unwrap_or_else(|e| panic!("`{src}` failed: {e:?}"));
    assert_eq!(q.dim, dim, "`{src}` dimension");
    let tol = 1e-12 * value.abs().max(1.0);
    assert!(
        (q.value - value).abs() <= tol,
        "`{src}` = {} (want {value})",
        q.value
    );
}

/// (source, canonical value, dimension) with no expected dimension.
const OK: &[(&str, f64, D)] = &[
    // numbers
    ("1", 1.0, N),
    ("1.5", 1.5, N),
    (".5", 0.5, N),
    ("5.", 5.0, N),
    ("2.5e3", 2500.0, N),
    ("1e-3", 1e-3, N),
    ("1E+2", 100.0, N),
    ("007", 7.0, N),
    // operators, precedence, associativity
    ("1 + 2", 3.0, N),
    ("5 - 3", 2.0, N),
    ("2 * 3", 6.0, N),
    ("6 / 4", 1.5, N),
    ("2 ^ 3", 8.0, N),
    ("2^3^2", 512.0, N),
    ("(2^3)^2", 64.0, N),
    ("-2^2", -4.0, N),
    ("(-2)^2", 4.0, N),
    ("2^-1", 0.5, N),
    ("2^-1^2", 0.5, N),
    ("1 - 2 - 3", -4.0, N),
    ("8 / 4 / 2", 1.0, N),
    ("2 + 3 * 4", 14.0, N),
    ("(2 + 3) * 4", 20.0, N),
    ("2 * 3 + 4", 10.0, N),
    ("--3", 3.0, N),
    ("+3", 3.0, N),
    ("-+3", -3.0, N),
    ("2 * -3", -6.0, N),
    ("2 - -3", 5.0, N),
    ("-3 * 4", -12.0, N),
    ("((1))", 1.0, N),
    ("1/2", 0.5, N),
    ("2/4^2", 0.125, N),
    ("2 \u{d7} 3", 6.0, N),
    ("2 \u{b7} 3", 6.0, N),
    ("6 \u{f7} 3", 2.0, N),
    ("5 \u{2212} 3", 2.0, N),
    // whitespace
    (" 1 +\t2\n", 3.0, N),
    ("1\u{a0}mm", 1.0, L),
    ("10\u{202f}mm", 10.0, L),
    ("\r\n7", 7.0, N),
    // length units
    ("10 mm", 10.0, L),
    ("10mm", 10.0, L),
    ("1 cm", 10.0, L),
    ("1 m", 1000.0, L),
    ("1 in", 25.4, L),
    ("1 inch", 25.4, L),
    ("1 ft", 304.8, L),
    ("2 feet", 609.6, L),
    ("1 foot", 304.8, L),
    ("1 um", 1e-3, L),
    ("1 \u{b5}m", 1e-3, L),
    ("1 \u{3bc}m", 1e-3, L),
    ("1 mil", 0.0254, L),
    ("1 thou", 0.0254, L),
    ("1\"", 25.4, L),
    ("1'", 304.8, L),
    ("2e3mm", 2000.0, L),
    // fractions, mixed numbers, feet-inch
    ("1/4 in", 6.35, L),
    ("1/4\"", 6.35, L),
    ("1 1/4 in", 31.75, L),
    ("1 1/4\"", 31.75, L),
    ("1 1/2", 1.5, N),
    ("6/2 mm", 3.0, L),
    ("5' 3\"", 1600.2, L),
    ("5'3\"", 1600.2, L),
    ("5' 3 1/2\"", 63.5 * 25.4, L),
    ("5 ft 3 in", 1600.2, L),
    ("-1/4 in", -6.35, L),
    // angles
    ("90 deg", PI / 2.0, A),
    ("1 rad", 1.0, A),
    ("100 grad", PI / 2.0, A),
    ("1 rev", 2.0 * PI, A),
    ("1 turn", 2.0 * PI, A),
    ("45\u{b0}", PI / 4.0, A),
    ("30 \u{b0}", PI / 6.0, A),
    // compound units
    ("1 N/mm^2", 1.0, D::PRESSURE),
    ("1 N/mm\u{b2}", 1.0, D::PRESSURE),
    ("1 MPa", 1.0, D::PRESSURE),
    ("1 kg*m", 1.0, D::new(1, 1, 0, 0, 0)),
    ("9.81 m/s^2", 9810.0, D::new(1, 0, -2, 0, 0)),
    ("1 kg/m^3", 1e-12, D::DENSITY),
    ("1 g/cc", 1e-9, D::DENSITY),
    ("1 mm^2", 1.0, D::AREA),
    ("2 mm\u{b2}", 2.0, D::AREA),
    ("2 mm\u{b3}", 2.0, D::VOLUME),
    ("1 cm^3", 1000.0, D::VOLUME),
    ("1 cc", 1000.0, D::VOLUME),
    ("1 L", 1e6, D::VOLUME),
    ("1 mL", 1000.0, D::VOLUME),
    ("1 s^-1", 1.0, D::new(0, 0, -1, 0, 0)),
    ("1 m^+2", 1e6, D::AREA),
    ("1 N*mm", 1.0, D::ENERGY),
    ("1 N\u{b7}mm", 1.0, D::ENERGY),
    ("1 J", 1000.0, D::ENERGY),
    ("1 kN", 1000.0, D::FORCE),
    ("1 lbf", 4.4482216152605, D::FORCE),
    ("1 psi", 6.894757293168e-3, D::PRESSURE),
    ("1 ksi", 6.894757293168, D::PRESSURE),
    ("1 GPa", 1000.0, D::PRESSURE),
    ("1 bar", 0.1, D::PRESSURE),
    ("1 Pa", 1e-6, D::PRESSURE),
    ("1 g", 1e-6, D::MASS),
    ("1 t", 1.0, D::MASS),
    ("1 lb", 4.5359237e-4, D::MASS),
    ("1 oz", 2.8349523125e-5, D::MASS),
    ("1 min", 60.0, D::TIME),
    ("1 h", 3600.0, D::TIME),
    ("1 ms", 1e-3, D::TIME),
    ("1 K", 1.0, D::TEMPERATURE),
    ("10 delta_degC", 10.0, D::TEMPERATURE),
    ("10 mm * s", 10.0, D::new(1, 0, 1, 0, 0)),
    // affine temperatures (whole expression only)
    ("20 degC", 293.15, D::TEMPERATURE),
    ("-40 degF", 233.15, D::TEMPERATURE),
    ("20 \u{b0}C", 293.15, D::TEMPERATURE),
    ("32 \u{b0}F", 273.15, D::TEMPERATURE),
    ("-(-20 degC)", 293.15, D::TEMPERATURE),
    // bare units are one of that unit
    ("mm", 1.0, L),
    ("in", 25.4, L),
    ("2 * mm", 2.0, L),
    ("(1 + 2) * 1 mm", 3.0, L),
    ("d1 / 5 mm", 2.0, N),
    ("1 m + 20 cm", 1200.0, L),
    ("10 mm * 2", 20.0, L),
    ("10 mm / 2", 5.0, L),
    ("(10 mm)^2", 100.0, D::AREA),
    ("10 mm^2", 10.0, D::AREA),
    ("mm^2", 1.0, D::AREA),
    // parameters
    ("d1", 10.0, L),
    ("d1 + d2", 30.0, L),
    ("d1 * 2", 20.0, L),
    ("d1 * d2", 200.0, D::AREA),
    ("d1 / d2", 0.5, N),
    ("d1 ^ 2", 100.0, D::AREA),
    ("n * d1", 40.0, L),
    ("F / (d1 * d2)", 0.5, D::PRESSURE),
    ("F * d1", 1000.0, D::ENERGY),
    ("d1 + 1 in", 35.4, L),
    ("floor(d1 / 3 mm) * 3 mm", 9.0, L),
    // Fusion-lenient bare numbers next to lengths/angles
    ("d1 + 5", 15.0, L),
    ("5 + d1", 15.0, L),
    ("d1 - 2.5", 7.5, L),
    ("d1 + 2 * 3", 16.0, L),
    ("d1 + (1 + 1)", 12.0, L),
    ("d1 + 5 + 3", 18.0, L),
    ("d1 - -5", 15.0, L),
    ("ang + 15", PI / 4.0, A),
    ("n + 1", 5.0, N),
    // constants
    ("PI", PI, N),
    ("pi", PI, N),
    ("E", E, N),
    ("2 * PI", 2.0 * PI, N),
    // functions
    ("sin(90 deg)", 1.0, N),
    ("sin(0)", 0.0, N),
    ("sin(ang)", 0.5, N),
    ("cos(0 deg)", 1.0, N),
    ("cos(PI)", -1.0, N),
    ("tan(45 deg)", 1.0, N),
    ("asin(1)", PI / 2.0, A),
    ("acos(1)", 0.0, A),
    ("atan(1)", PI / 4.0, A),
    ("atan2(1, 1)", PI / 4.0, A),
    ("atan2(d1, d1)", PI / 4.0, A),
    ("sqrt(16)", 4.0, N),
    ("sqrt(4 mm^2)", 2.0, L),
    ("sqrt(d1 * d1)", 10.0, L),
    ("abs(-3 mm)", 3.0, L),
    ("abs(2)", 2.0, N),
    ("hypot(3 mm, 4 mm)", 5.0, L),
    ("min(3, 1, 2)", 1.0, N),
    ("max(d1, d2)", 20.0, L),
    ("min(d1)", 10.0, L),
    ("floor(2.7)", 2.0, N),
    ("ceil(2.1)", 3.0, N),
    ("round(2.5)", 3.0, N),
    ("round(-2.5)", -3.0, N),
    ("sign(-4)", -1.0, N),
    ("sign(0)", 0.0, N),
    ("sign(3)", 1.0, N),
    ("exp(0)", 1.0, N),
    ("ln(E)", 1.0, N),
    ("log10(1000)", 3.0, N),
    ("pow(2, 10)", 1024.0, N),
    ("10 min", 600.0, D::TIME),
    ("min(1, 2) * 1 min", 60.0, D::TIME),
    ("sin(ang)^2 + cos(ang)^2", 1.0, N),
];

#[test]
fn evaluates_every_ok_case() {
    for &(src, v, d) in OK {
        check(src, ev(src), v, d);
    }
    assert!(OK.len() >= 150, "{} cases", OK.len());
}

#[test]
fn bare_numbers_take_the_field_unit() {
    let inch = EvalOptions::new(&UnitPrefs {
        length: LengthUnit::Inch,
    });
    let cases: &[(&str, EvalOptions, f64, D)] = &[
        ("10", opts().expect(L), 10.0, L),
        ("10", inch.expect(L), 254.0, L),
        ("10", opts().unit(Unit::Inch), 254.0, L),
        ("10", opts().unit(Unit::Centimetre), 100.0, L),
        ("2 * 3", opts().expect(L), 6.0, L),
        ("-5", opts().expect(L), -5.0, L),
        ("1/2", opts().expect(L), 0.5, L),
        ("45", opts().expect(A), PI / 4.0, A),
        ("1", opts().unit(Unit::Radian), 1.0, A),
        (
            "20",
            opts().unit(Unit::DegreeCelsius),
            293.15,
            D::TEMPERATURE,
        ),
        ("10", opts().expect(D::PRESSURE), 10.0, D::PRESSURE),
        ("10", opts().expect(N), 10.0, N),
        ("d1 + 5", inch.expect(L), 10.0 + 127.0, L),
        // the field unit wins over the document unit for its own dimension
        ("d1 + 5", opts().unit(Unit::Inch), 10.0 + 127.0, L),
        ("10 mm", opts().unit(Unit::Inch), 10.0, L),
        ("d1 / d2", opts().expect(N), 0.5, N),
    ];
    for (src, o, v, d) in cases {
        check(src, ev_with(src, o), *v, *d);
    }
}

#[test]
fn strict_mode_rejects_bare_operands() {
    let s = opts().strict(true);
    assert!(matches!(
        ev_with("d1 + 5", &s),
        Err(ExprError::UnitMismatch { left: L, right: N, span }) if span == (0..6)
    ));
    // The whole-expression rule is not leniency: it stays in strict mode.
    check("10", ev_with("10", &s.expect(L)), 10.0, L);
    check("d1 + 5 mm", ev_with("d1 + 5 mm", &s), 15.0, L);
}

/// Lenient addition only applies to lengths and angles, and only to unit-free literals.
#[test]
fn leniency_is_limited() {
    assert!(matches!(ev("F + 5"), Err(ExprError::UnitMismatch { .. })));
    assert!(matches!(ev("d1 + n"), Err(ExprError::UnitMismatch { .. })));
    assert!(matches!(
        ev("d1 + sqrt(4)"),
        Err(ExprError::UnitMismatch { .. })
    ));
}

#[track_caller]
fn parse_err(src: &str, span: Range<usize>, needle: &str) {
    match parse(src) {
        Err(ExprError::Parse { span: s, msg }) => {
            assert_eq!(s, span, "`{src}`: {msg}");
            assert!(msg.contains(needle), "`{src}`: `{msg}` lacks `{needle}`");
        }
        other => panic!("`{src}` should be a syntax error, got {other:?}"),
    }
}

#[test]
fn syntax_errors_have_spans() {
    parse_err("", 0..0, "empty");
    parse_err("   ", 0..3, "empty");
    parse_err("1 +", 3..3, "end");
    parse_err("(1 + 2", 0..1, "unclosed");
    parse_err("1 + 2)", 5..6, "`)`");
    parse_err("2 3", 2..3, "missing operator");
    parse_err("1,5", 1..2, "decimal separator");
    parse_err("1 , 5", 2..3, "unexpected `,`");
    parse_err("max(1, 2", 0..3, "unclosed");
    parse_err("sin", 0..3, "needs arguments");
    parse_err("sin + 1", 0..3, "needs arguments");
    parse_err("foo(1)", 0..3, "unknown function");
    parse_err("sin(1, 2)", 0..9, "takes 1");
    parse_err("atan2(1)", 0..8, "takes 2");
    parse_err("min()", 0..5, "at least 1");
    parse_err("1/0 in", 0..3, "division by zero");
    parse_err("1 1/0", 2..5, "division by zero");
    parse_err("10 mm^x", 5..7, "integer");
    parse_err("10 mm^0", 5..7, "integer");
    parse_err("10 mm^1.5", 5..9, "integer");
    parse_err("1e400", 0..5, "too large");
    parse_err("2 d1", 2..4, "not a unit");
    parse_err("#", 0..1, "unexpected character");
    parse_err("1 $", 2..3, "unexpected character");
    parse_err("10 degC + 1 K", 0..7, "degC");
    parse_err("degC", 0..4, "degC");
    parse_err("10 degC/s", 0..9, "degC");
    parse_err("2 * 10 degF", 4..11, "degF");
    parse_err("10 degC^2", 0..9, "degC");
    parse_err("()", 1..2, "`)`");
    parse_err("1 * * 2", 4..5, "`*`");
    parse_err("d1 d2", 3..5, "missing operator");
    parse_err("5 mm 3", 5..6, "missing operator");
    let deep = format!("{}1{}", "(".repeat(300), ")".repeat(300));
    assert!(matches!(parse(&deep), Err(ExprError::Parse { msg, .. }) if msg.contains("deeply")));
    let tower = vec!["2"; 300].join("^");
    assert!(matches!(parse(&tower), Err(ExprError::Parse { msg, .. }) if msg.contains("deeply")));
    let chain = vec!["1"; 300].join(" + ");
    assert!(matches!(parse(&chain), Err(ExprError::Parse { msg, .. }) if msg.contains("deeply")));
    let ok_chain = vec!["1"; 200].join(" + ");
    check(&ok_chain, ev(&ok_chain), 200.0, N);
}

#[test]
fn unknown_parameters_have_spans() {
    assert_eq!(
        ev("x + 1"),
        Err(ExprError::UnknownParam {
            name: "x".into(),
            span: 0..1
        })
    );
    assert_eq!(
        ev("d1 + foo * 2"),
        Err(ExprError::UnknownParam {
            name: "foo".into(),
            span: 5..8
        })
    );
}

#[test]
fn unit_mismatches_have_correct_spans() {
    let cases: &[(&str, D, D, Range<usize>)] = &[
        ("10 mm + 5 deg", L, A, 0..13),
        ("d1 + 1 N", L, D::FORCE, 0..8),
        ("2 * (d1 - ang)", L, A, 4..14),
        ("1 + (d1 + 1 s) * 2", L, D::TIME, 4..14),
        ("max(1 mm, 1 s)", L, D::TIME, 0..14),
        ("min(1, 2, 3 mm)", N, L, 0..15),
        ("atan2(1 mm, 1)", L, N, 0..14),
        ("hypot(1 mm, 1 s)", L, D::TIME, 0..16),
        ("d1 + 5 kg", L, D::MASS, 0..9),
        ("F - d1", D::FORCE, L, 0..6),
    ];
    for (src, l, r, span) in cases {
        assert_eq!(
            ev(src),
            Err(ExprError::UnitMismatch {
                left: *l,
                right: *r,
                span: span.clone()
            }),
            "`{src}`"
        );
    }
}

#[test]
fn domain_errors() {
    let cases: &[(&str, &str, Range<usize>)] = &[
        ("1 / 0", "/", 0..5),
        ("d1 / (d2 - 20 mm)", "/", 0..17),
        ("sqrt(-1)", "sqrt", 0..8),
        ("sqrt(1 mm)", "sqrt", 0..10),
        ("ln(0)", "ln", 0..5),
        ("log10(-1)", "log10", 0..9),
        ("asin(2)", "asin", 0..7),
        ("acos(-2)", "acos", 0..8),
        ("atan(1 mm)", "atan", 0..10),
        ("sin(1 mm)", "sin", 0..9),
        ("floor(1 mm)", "floor", 0..11),
        ("ceil(d1)", "ceil", 0..8),
        ("round(ang)", "round", 0..10),
        ("sign(F)", "sign", 0..7),
        ("exp(1000)", "exp", 0..9),
        ("pow(2 mm, 2)", "pow", 0..12),
        ("pow(-8, 1/3)", "pow", 0..12),
        ("2 ^ 1 mm", "^", 0..8),
        ("(1 mm) ^ 0.5", "^", 0..12),
        ("d1 ^ (n / 3)", "^", 0..12),
        ("(-8)^(1/3)", "^", 0..10),
        ("10^400", "^", 0..6),
        ("1e300 * 1e300", "*", 0..13),
        ("1e300 + 1e308 * 2", "*", 8..17),
        ("1 mm^100 * 1 mm^100", "*", 0..19),
        ("1e308 m", "unit", 0..7),
    ];
    for (src, func, span) in cases {
        match ev(src) {
            Err(ExprError::Domain {
                func: f, span: s, ..
            }) => {
                assert_eq!((f, &s), (*func, span), "`{src}`");
            }
            other => panic!("`{src}`: expected a domain error, got {other:?}"),
        }
    }
}

#[test]
fn wrong_dimension_for_the_field() {
    assert_eq!(
        ev_with("10 mm", &opts().expect(A)),
        Err(ExprError::WrongDimension {
            expected: A,
            got: L
        })
    );
    assert_eq!(
        ev_with("d1", &opts().expect(N)),
        Err(ExprError::WrongDimension {
            expected: N,
            got: L
        })
    );
    assert_eq!(
        ev_with("5 deg", &opts().unit(Unit::Millimetre)),
        Err(ExprError::WrongDimension {
            expected: L,
            got: A
        })
    );
    // A unitless non-literal stays dimensionless: `d1 / d2` in a length field is an error.
    assert!(matches!(
        ev_with("d1 / d2", &opts().expect(L)),
        Err(ExprError::WrongDimension { .. })
    ));
}

#[test]
fn references_list_names_and_spans() {
    let ast = parse("d1 * 2 + max(width, d1) - 1 mm").unwrap();
    assert_eq!(
        references(&ast),
        vec![
            ("d1".to_owned(), 0..2),
            ("width".to_owned(), 13..18),
            ("d1".to_owned(), 20..22)
        ]
    );
    assert!(references(&parse("10 mm + PI * sin(1 deg) * min(1, 2) * 1 min").unwrap()).is_empty());
}

#[test]
fn reserved_names() {
    for n in [
        "mm",
        "in",
        "inch",
        "m",
        "s",
        "t",
        "g",
        "h",
        "l",
        "L",
        "K",
        "N",
        "cc",
        "deg",
        "rad",
        "degC",
        "delta_degC",
        "min",
        "sin",
        "atan2",
        "PI",
        "pi",
        "E",
        "um",
        "\u{b5}m",
    ] {
        assert!(is_reserved(n), "{n}");
        assert_eq!(check_name(n), Err(ExprError::ReservedName(n.to_owned())));
    }
    for n in ["", "1a", "a b", "a-b", "a.b", "d1(", "\u{b0}"] {
        assert_eq!(
            check_name(n),
            Err(ExprError::InvalidName(n.to_owned())),
            "{n:?}"
        );
    }
    for n in [
        "d1",
        "width",
        "_x",
        "Length2",
        "\u{e9}paisseur",
        "mm2",
        "x_mm",
    ] {
        assert_eq!(check_name(n), Ok(()), "{n}");
    }
    let all = super::reserved_names();
    assert!(all.contains(&"mm") && all.contains(&"sqrt") && all.contains(&"PI"));
    assert!(!all.contains(&"mm^2") && !all.contains(&"\""));
}

#[test]
fn affine_unit_detection() {
    assert!(is_affine(Unit::DegreeCelsius) && is_affine(Unit::DegreeFahrenheit));
    assert!(!is_affine(Unit::DeltaCelsius) && !is_affine(Unit::Kelvin));
    assert_eq!(canonical_unit(L), Some(Unit::Millimetre));
    assert_eq!(canonical_unit(D::PRESSURE), Some(Unit::Megapascal));
    assert_eq!(canonical_unit(D::new(1, 0, 0, 0, 1)), None);
}

/// (source, canonical printed form).
const PRINT: &[(&str, &str)] = &[
    ("1+2", "1 + 2"),
    ("(2+3)*4", "(2 + 3) * 4"),
    ("2^3^2", "2^3^2"),
    ("(2^3)^2", "(2^3)^2"),
    ("-(2^2)", "-2^2"),
    ("(-2)^2", "(-2)^2"),
    ("a - (b - c)", "a - (b - c)"),
    ("a - b - c", "a - b - c"),
    ("a / (b * c)", "a / (b * c)"),
    ("a / b * c", "a / b * c"),
    ("-(a + b)", "-(a + b)"),
    ("1/4 in", "0.25 in"),
    ("1 1/4\"", "1.25 in"),
    ("1 N/mm^2", "1 N/mm^2"),
    ("1 N/mm\u{b2}", "1 N/mm^2"),
    ("1 / (4 in)", "1 / (4 in)"),
    ("(10 mm)^2", "(10 mm)^2"),
    ("10 mm^2", "10 mm^2"),
    ("(10 mm) * s", "10 mm * (s)"),
    ("10 mm * s", "10 mm*s"),
    ("5' 3\"", "5 ft + 3 in"),
    ("1 cc", "1 cc"),
    ("20 \u{b0}C", "20 degC"),
    ("1 \u{b5}m", "1 um"),
    ("sin(x)^2", "sin(x)^2"),
    ("x^-2", "x^-2"),
    ("10 s^-1", "10 s^-1"),
    ("10 min", "10 min"),
    ("min( 1,2 )", "min(1, 2)"),
    ("1e20 mm", "1e20 mm"),
    ("1e-7", "1e-7"),
    ("2 * mm", "2 * (mm)"),
    ("PI*E", "PI * E"),
    ("--x", "--x"),
    ("+x", "+x"),
    ("2^(-1)", "2^-1"),
    ("(2*3)^2", "(2 * 3)^2"),
    ("45\u{b0}", "45 deg"),
];

#[test]
fn printing_is_canonical_and_round_trips() {
    for &(src, want) in PRINT {
        let ast = parse(src).unwrap();
        let printed = ast.to_string();
        assert_eq!(printed, want, "`{src}`");
        let back = parse(&printed).unwrap_or_else(|e| panic!("`{printed}` won't parse: {e}"));
        assert_eq!(
            back.without_spans(),
            ast.without_spans(),
            "`{src}` -> `{printed}`"
        );
    }
    for &(src, ..) in OK {
        let ast = parse(src).unwrap();
        let back = parse(&ast.to_string()).unwrap();
        assert_eq!(
            back.without_spans(),
            ast.without_spans(),
            "`{src}` -> `{ast}`"
        );
    }
}

#[test]
fn format_value_round_trips_through_eval() {
    let q = Quantity::new(31.75, L);
    assert_eq!(format_value(q, Unit::Inch, 3).unwrap(), "1.25 in");
    assert_eq!(format_value(q, Unit::Millimetre, 2).unwrap(), "31.75 mm");
    let back = ev(&format_value(q, Unit::Inch, 3).unwrap()).unwrap();
    assert_eq!(back.dim, L);
    assert!((back.value - 31.75).abs() < 1e-12);
    assert!(format_value(q, Unit::Degree, 2).is_err());
}

#[test]
fn errors_are_diagnostics() {
    use amg_core::Diagnostic;
    let e = ev("10 mm + 5 deg").unwrap_err();
    assert_eq!(e.span(), Some(0..13));
    assert!(e.hint().is_some());
    assert_eq!(e.user_message(), "unit mismatch: mm vs rad");
    assert_eq!(ExprError::Cycle(vec![]).span(), None);
}

// ---- property tests -----------------------------------------------------------------------

/// Random well-formed expression source from the grammar.
fn expr_src() -> impl Strategy<Value = String> {
    let num = prop_oneof![
        (0u32..1000).prop_map(|n| n.to_string()),
        (0.0f64..1e6).prop_map(|x| format!("{x}")),
        (1e-30f64..1e30).prop_map(|x| format!("{x:e}")),
        (1u32..20, 1u32..20).prop_map(|(a, b)| format!("{a}/{b} in")),
        (1u32..20, 1u32..20, 1u32..20).prop_map(|(w, a, b)| format!("{w} {a}/{b}\"")),
        (0u32..20, 0u32..12).prop_map(|(f, i)| format!("{f}' {i}\"")),
    ];
    let unit = prop::sample::select(vec![
        "mm", "cm", "m", "in", "ft", "deg", "rad", "N", "MPa", "kg", "s", "mm^2", "N/mm^2",
        "m/s^2", "s^-1", "cc", "\u{b0}", "mm\u{b2}", "kg*m", "min",
    ]);
    let ident = prop::sample::select(vec![
        "d1", "d2", "ang", "n", "F", "width", "PI", "E", "mm", "in",
    ]);
    let leaf = prop_oneof![
        num.clone(),
        (num, unit).prop_map(|(n, u)| format!("{n} {u}")),
        ident.prop_map(str::to_owned),
    ];
    leaf.prop_recursive(5, 48, 4, |inner| {
        let func = prop::sample::select(vec![
            "sin", "sqrt", "abs", "floor", "min", "max", "hypot", "atan2", "pow",
        ]);
        prop_oneof![
            (
                inner.clone(),
                prop::sample::select(vec!["+", "-", "*", "/", "^"]),
                inner.clone()
            )
                .prop_map(|(a, op, b)| format!("{a} {op} {b}")),
            inner.clone().prop_map(|a| format!("({a})")),
            inner.clone().prop_map(|a| format!("-{a}")),
            (func, prop::collection::vec(inner, 1..3))
                .prop_map(|(f, args)| format!("{f}({})", args.join(", "))),
        ]
    })
}

fn same_result(a: &Result<Quantity, ExprError>, b: &Result<Quantity, ExprError>) -> bool {
    match (a, b) {
        (Ok(x), Ok(y)) => x.dim == y.dim && x.value.to_bits() == y.value.to_bits(),
        (Err(_), Err(_)) => true,
        _ => false,
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]

    /// parse → print → parse gives the same tree, and evaluation agrees bit-for-bit.
    #[test]
    fn parse_print_parse_round_trips(src in expr_src()) {
        // Some generated sources are invalid (arity, nesting); those must fail cleanly.
        if let Ok(ast) = parse(&src) {
            let printed = ast.to_string();
            let back = parse(&printed).map_err(|e| TestCaseError::fail(format!("`{src}` -> `{printed}`: {e}")))?;
            prop_assert_eq!(back.without_spans(), ast.without_spans(), "`{}` -> `{}`", src, printed);
            prop_assert_eq!(back.to_string(), printed.clone());
            let o = opts();
            prop_assert!(same_result(&eval(&ast, &env, &o), &eval(&back, &env, &o)), "`{}`", printed);
        }
    }

    /// The parser and evaluator never panic on arbitrary input, and every error span lies
    /// inside the input on char boundaries.
    #[test]
    fn fuzz_never_panics(src in "[ 0-9a-zA-Z_.,+\\-*/^()'\"\u{b0}\u{b2}\u{b5}\u{a0}#eE]{0,40}|\\PC{0,24}") {
        match parse(&src) {
            Ok(ast) => {
                let _ = eval(&ast, &env, &opts());
                let _ = eval(&ast, &env, &opts().expect(L).strict(true));
                let printed = ast.to_string();
                let back = parse(&printed).map_err(|e| TestCaseError::fail(format!("`{src}` -> `{printed}`: {e}")))?;
                prop_assert_eq!(back.without_spans(), ast.without_spans());
            }
            Err(e) => {
                if let Some(s) = e.span() {
                    prop_assert!(s.start <= s.end && s.end <= src.len(), "{:?} in {:?}", s, src);
                    prop_assert!(src.is_char_boundary(s.start) && src.is_char_boundary(s.end));
                }
            }
        }
    }

    /// Formatting a value and reading it back is exact within the printed precision.
    #[test]
    fn value_format_parse_round_trips(
        v in -1e6f64..1e6,
        unit in prop::sample::select(vec![Unit::Millimetre, Unit::Inch, Unit::Metre, Unit::Degree, Unit::Megapascal, Unit::DegreeCelsius]),
        precision in 0u8..9,
    ) {
        let q = Quantity::new(unit.to_canonical(v), unit.dimension());
        let text = format_value(q, unit, precision).unwrap();
        let back = ev_with(&text, &opts().unit(unit)).map_err(|e| TestCaseError::fail(format!("`{text}`: {e}")))?;
        prop_assert_eq!(back.dim, q.dim);
        let tol = (0.5 * 10f64.powi(-i32::from(precision)) * unit.scale()) * (1.0 + 1e-9) + 1e-9 * q.value.abs();
        prop_assert!((back.value - q.value).abs() <= tol, "{} -> `{}` -> {}", q.value, text, back.value);
    }
}
