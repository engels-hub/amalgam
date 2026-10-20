use super::*;
use crate::error::{Diagnostic, Error};
use approx::assert_relative_eq;
use proptest::prelude::*;
use std::f64::consts::{PI, TAU};

fn parse(s: &str) -> ParsedQuantity {
    parse_quantity(s).unwrap_or_else(|e| panic!("{s:?}: {e}"))
}
/// Parses and resolves to canonical with `mm` as the default unit.
fn canon(s: &str) -> Quantity {
    parse(s).resolve(Unit::Millimetre)
}
fn err(s: &str) -> ParseQuantityError {
    parse_quantity(s).expect_err(s)
}
fn spec(style: FormatStyle, precision: u8) -> FormatSpec {
    FormatSpec {
        style,
        precision,
        trim_zeros: false,
        show_unit: true,
    }
}
fn fmt(v: f64, u: Unit, s: &FormatSpec) -> String {
    format_quantity(Quantity::from_unit(v, u), u, s).unwrap()
}

// ---- Dimension ------------------------------------------------------------------------

#[test]
fn dimension_algebra() {
    let d = Dimension::LENGTH;
    assert_eq!(d * d, Dimension::AREA);
    assert_eq!(d.pow(3), Dimension::VOLUME);
    assert_eq!(Dimension::VOLUME / d, Dimension::AREA);
    assert_eq!(Dimension::FORCE / Dimension::AREA, Dimension::PRESSURE);
    assert_eq!(Dimension::FORCE * d, Dimension::ENERGY);
    assert_eq!(Dimension::MASS / Dimension::VOLUME, Dimension::DENSITY);
    assert_eq!(
        Dimension::MASS * d / Dimension::TIME.pow(2),
        Dimension::FORCE
    );
    assert_eq!(d.pow(0), Dimension::NONE);
    assert_eq!(Dimension::default(), Dimension::NONE);
    assert_eq!(Dimension::new(1, 2, 3, 4, 5).angle, 5);
    // Saturates instead of overflowing.
    assert_eq!(Dimension::new(100, 0, 0, 0, 0).pow(2).length, i8::MAX);
    assert_eq!(
        (Dimension::new(-100, 0, 0, 0, 0) / d.pow(100)).length,
        i8::MIN
    );
}

#[test]
fn dimension_display() {
    let cases = [
        (Dimension::NONE, "1"),
        (Dimension::LENGTH, "mm"),
        (Dimension::AREA, "mm^2"),
        (Dimension::VOLUME, "mm^3"),
        (Dimension::ANGLE, "rad"),
        (Dimension::MASS, "t"),
        (Dimension::TIME, "s"),
        (Dimension::FORCE, "N"),
        (Dimension::PRESSURE, "MPa"),
        (Dimension::DENSITY, "t/mm^3"),
        (Dimension::ENERGY, "mJ"),
        (Dimension::TEMPERATURE, "K"),
        (Dimension::new(1, 1, -3, 0, 0), "mm*t/s^3"),
        (Dimension::new(0, 0, -1, 0, 0), "1/s"),
        (Dimension::new(1, 0, 0, -1, 1), "mm*rad/K"),
        (Dimension::new(4, 0, 0, 0, 0), "mm^4"),
        (Dimension::new(0, 0, -2, -1, 0), "1/s^2/K"),
    ];
    for (d, s) in cases {
        assert_eq!(d.to_string(), s);
    }
}

#[test]
fn dimension_serde() {
    let s = serde_json::to_string(&Dimension::FORCE).unwrap();
    assert_eq!(
        s,
        r#"{"length":1,"mass":1,"time":-2,"temperature":0,"angle":0}"#
    );
    assert_eq!(
        serde_json::from_str::<Dimension>(&s).unwrap(),
        Dimension::FORCE
    );
}

// ---- Quantity -------------------------------------------------------------------------

#[test]
fn quantity_constructors_and_conversion() {
    let q = Quantity::from_unit(2.0, Unit::Inch);
    assert_eq!(q, Quantity::new(50.8, Dimension::LENGTH));
    assert_relative_eq!(q.in_unit(Unit::Inch).unwrap(), 2.0);
    assert_relative_eq!(q.in_unit(Unit::Centimetre).unwrap(), 5.08);
    assert!(matches!(
        q.in_unit(Unit::Degree),
        Err(Error::DimensionMismatch { expected, found })
            if expected == Dimension::ANGLE && found == Dimension::LENGTH
    ));
    assert_eq!(Quantity::dimensionless(3.0).dim, Dimension::NONE);
    assert_relative_eq!(q.expect_dim(Dimension::LENGTH).unwrap(), 50.8);
    assert!(q.expect_dim(Dimension::AREA).is_err());
}

#[test]
fn quantity_arithmetic() {
    let a = Quantity::new(10.0, Dimension::LENGTH);
    let b = Quantity::new(4.0, Dimension::LENGTH);
    let ang = Quantity::from_unit(5.0, Unit::Degree);
    assert_eq!(
        a.checked_add(b).unwrap(),
        Quantity::new(14.0, Dimension::LENGTH)
    );
    assert_eq!(
        a.checked_sub(b).unwrap(),
        Quantity::new(6.0, Dimension::LENGTH)
    );
    // Fusion semantics: `10 mm + 5 deg` is an error.
    let e = a.checked_add(ang).unwrap_err();
    assert_eq!(e.to_string(), "unit mismatch: expected mm, got rad");
    assert!(a.checked_sub(ang).is_err());
    assert_eq!(a * b, Quantity::new(40.0, Dimension::AREA));
    assert_eq!(a / b, Quantity::new(2.5, Dimension::NONE));
    assert_eq!(-a, Quantity::new(-10.0, Dimension::LENGTH));
    assert_eq!(a.powi(3), Quantity::new(1000.0, Dimension::VOLUME));
    assert_eq!(
        a.powi(-1),
        Quantity::new(0.1, Dimension::new(-1, 0, 0, 0, 0))
    );
    // Force times distance is energy in mJ.
    let f = Quantity::from_unit(2.0, Unit::Kilonewton);
    assert_eq!((f * a).dim, Dimension::ENERGY);
    assert_relative_eq!((f * a).in_unit(Unit::Joule).unwrap(), 20.0);
}

#[test]
fn quantity_display_and_serde() {
    assert_eq!(
        Quantity::new(12.5, Dimension::LENGTH).to_string(),
        "12.5 mm"
    );
    assert_eq!(Quantity::dimensionless(2.0).to_string(), "2");
    let q = Quantity::from_unit(1.5, Unit::Megapascal);
    let back: Quantity = serde_json::from_str(&serde_json::to_string(&q).unwrap()).unwrap();
    assert_eq!(back, q);
}

// ---- Unit table -----------------------------------------------------------------------

#[test]
fn table_order_and_all() {
    assert!(table::table_is_ordered());
    let all = Unit::all();
    assert_eq!(all.len(), 49);
    assert_eq!(all[0], Unit::Micrometre);
    assert_eq!(all[all.len() - 1], Unit::DeltaCelsius);
}

#[test]
fn every_symbol_is_unique_and_found() {
    let mut seen = std::collections::HashMap::new();
    for &u in Unit::all() {
        assert_eq!(u.symbols()[0], u.symbol());
        assert_eq!(&u.symbols()[1..], u.aliases());
        assert_eq!(u.to_string(), u.symbol());
        for &s in u.symbols() {
            assert_eq!(seen.insert(s, u), None, "duplicate symbol {s}");
            assert_eq!(Unit::from_symbol(s), Some(u), "{s}");
        }
    }
    assert_eq!(Unit::from_symbol("MM"), None);
    assert_eq!(Unit::from_symbol(""), None);
}

/// Table-driven: every symbol and alias of every unit parses, with and without a space.
#[test]
fn every_symbol_and_alias_parses() {
    for &u in Unit::all() {
        for sym in u.symbols() {
            for text in [
                format!("2.5 {sym}"),
                format!("2.5{sym}"),
                format!(" -1 1/4\t{sym} "),
            ] {
                let p = parse(&text);
                assert_eq!(p.unit, Some(u), "{text:?}");
                let expected = if text.contains('-') { -1.25 } else { 2.5 };
                assert_relative_eq!(p.value, expected);
            }
        }
    }
}

#[test]
fn every_unit_converts_both_ways() {
    for &u in Unit::all() {
        for v in [-3.5, 0.0, 1.0, 1234.5678] {
            assert_relative_eq!(
                u.from_canonical(u.to_canonical(v)),
                v,
                epsilon = 1e-9,
                max_relative = 1e-12
            );
        }
        let canon =
            Unit::canonical_for(u.dimension()).expect("every dimension has a canonical unit");
        assert_eq!(canon.dimension(), u.dimension());
        assert_relative_eq!(canon.scale(), 1.0);
    }
}

#[test]
fn table_values() {
    let cases: [(Unit, f64, f64); 18] = [
        (Unit::Micrometre, 1.0, 1e-3),
        (Unit::Foot, 1.0, 304.8),
        (Unit::Thou, 1.0, 0.0254),
        (Unit::Gradian, 200.0, PI),
        (Unit::Turn, 1.0, TAU),
        (Unit::SquareInch, 1.0, 645.16),
        (Unit::CubicInch, 1.0, 16387.064),
        (Unit::Litre, 1.0, 1e6),
        (Unit::Pound, 1.0, 4.5359237e-4),
        (Unit::Ounce, 16.0, 4.5359237e-4),
        (Unit::Hour, 1.0, 3600.0),
        (Unit::PoundForce, 1.0, 4.4482216152605),
        (Unit::Psi, 1000.0, 6.894757293168),
        (Unit::Bar, 1.0, 0.1),
        (Unit::Gigapascal, 210.0, 210_000.0),
        (Unit::Joule, 1.0, 1000.0),
        (Unit::DegreeFahrenheit, 32.0, 273.15),
        (Unit::DeltaCelsius, 10.0, 10.0),
    ];
    for (u, v, c) in cases {
        assert_relative_eq!(u.to_canonical(v), c, max_relative = 1e-12);
    }
    assert_relative_eq!(
        Unit::DegreeFahrenheit.from_canonical(373.15),
        212.0,
        max_relative = 1e-12
    );
    // Without an offset, -0.0 keeps its sign.
    assert!(Unit::Inch.to_canonical(-0.0).is_sign_negative());
}

#[test]
fn length_unit_subset() {
    assert_eq!(LengthUnit::default(), LengthUnit::Millimetre);
    for l in LengthUnit::ALL {
        let u = Unit::from(l);
        assert_eq!(u.dimension(), Dimension::LENGTH);
        assert_eq!(LengthUnit::try_from(u), Ok(l));
    }
    assert_eq!(LengthUnit::try_from(Unit::Thou), Err(Unit::Thou));
    assert_eq!(LengthUnit::try_from(Unit::Degree), Err(Unit::Degree));
}

// ---- Parsing ----------------------------------------------------------------------------

/// The examples of the FND-10 "Done when".
#[test]
fn done_when_examples() {
    let close = |s: &str, v: f64, d: Dimension| {
        let q = canon(s);
        assert_eq!(q.dim, d, "{s}");
        assert_relative_eq!(q.value, v, max_relative = 1e-12);
    };
    close("1/4 in", 6.35, Dimension::LENGTH);
    close("1 1/4\"", 31.75, Dimension::LENGTH);
    close("5' 3 1/2\"", 1612.9, Dimension::LENGTH);
    close("30 deg", PI / 6.0, Dimension::ANGLE);
    close("20 degC", 293.15, Dimension::TEMPERATURE);
    close("7.85 g/cm^3", 7.85e-9, Dimension::DENSITY);
    close("-.5e1 mm", -5.0, Dimension::LENGTH);
}

#[test]
fn grammar_forms() {
    let cases: &[(&str, f64, Option<Unit>)] = &[
        ("12.5 mm", 12.5, Some(Unit::Millimetre)),
        ("12.5", 12.5, None),
        ("  +3\t", 3.0, None),
        ("5.", 5.0, None),
        (".5", 0.5, None),
        ("1e3mm", 1000.0, Some(Unit::Millimetre)),
        ("1E-3 m", 1e-3, Some(Unit::Metre)),
        ("2.5e+2", 250.0, None),
        ("3/8", 0.375, None),
        ("2 3/8 in", 2.375, Some(Unit::Inch)),
        ("1  1/2", 1.5, None),
        ("5'", 5.0, Some(Unit::Foot)),
        ("5'3\"", 63.0, Some(Unit::Inch)),
        ("5 ft 3 in", 63.0, Some(Unit::Inch)),
        ("5 feet 1/2 inch", 60.5, Some(Unit::Inch)),
        ("-5' 3\"", -63.0, Some(Unit::Inch)),
        ("1 1/2' 6\"", 24.0, Some(Unit::Inch)),
        ("10 mm^2", 10.0, Some(Unit::SquareMillimetre)),
        ("10 mm²", 10.0, Some(Unit::SquareMillimetre)),
        ("3 µm", 3.0, Some(Unit::Micrometre)),
        ("3 μm", 3.0, Some(Unit::Micrometre)),
        ("90°", 90.0, Some(Unit::Degree)),
        ("20 °C", 20.0, Some(Unit::DegreeCelsius)),
        ("2 min", 2.0, Some(Unit::Minute)),
        ("2 mil", 2.0, Some(Unit::Thou)),
        ("2 m", 2.0, Some(Unit::Metre)),
        ("2 t", 2.0, Some(Unit::Tonne)),
        ("2 turn", 2.0, Some(Unit::Turn)),
        ("235 N/mm^2", 235.0, Some(Unit::Megapascal)),
        ("1 N*mm", 1.0, Some(Unit::Millijoule)),
        ("10 delta_degC", 10.0, Some(Unit::DeltaCelsius)),
        ("-0", -0.0, None),
    ];
    for &(s, v, u) in cases {
        let p = parse(s);
        assert_eq!(p.unit, u, "{s:?}");
        assert_relative_eq!(p.value, v, max_relative = 1e-15);
    }
    assert!(parse("-0").value.is_sign_negative());
}

#[test]
fn resolve_applies_the_default_unit() {
    let p = parse("10");
    assert_relative_eq!(p.resolve(Unit::Inch).value, 254.0);
    assert_relative_eq!(parse("90").resolve(Unit::Degree).value, PI / 2.0);
    // A typed unit wins over the default.
    assert_relative_eq!(parse("10 mm").resolve(Unit::Inch).value, 10.0);
    // Temperature differences do not get the absolute offset.
    assert_relative_eq!(canon("10 delta_degC").value, 10.0);
}

/// The error spans of the FND-10 "Done when".
#[test]
fn done_when_error_spans() {
    assert_eq!(
        err("12,5 mm"),
        ParseQuantityError::DecimalComma { span: 2..3 }
    );
    assert_eq!(
        err("3 foo"),
        ParseQuantityError::UnknownUnit {
            span: 2..5,
            text: "foo".into()
        }
    );
    assert_eq!(
        err("1/0 in"),
        ParseQuantityError::ZeroDenominator { span: 0..3 }
    );
    assert_eq!(
        err("1 m 20 cm"),
        ParseQuantityError::Trailing {
            span: 4..9,
            text: "20 cm".into()
        }
    );
}

#[test]
fn other_errors() {
    assert_eq!(err(""), ParseQuantityError::Empty);
    assert_eq!(err(" \t "), ParseQuantityError::Empty);
    let invalid = |s: &str, span: std::ops::Range<usize>, text: &str| {
        assert_eq!(
            err(s),
            ParseQuantityError::InvalidNumber {
                span,
                text: text.into()
            },
            "{s:?}"
        );
    };
    invalid("mm", 0..2, "mm");
    invalid(" abc", 1..4, "abc");
    invalid("-", 0..1, "-");
    invalid("- 5", 0..1, "-");
    invalid(".", 0..1, ".");
    invalid("-x2", 0..3, "-x2");
    invalid("1/ in", 0..2, "1/");
    assert_eq!(
        err("1 1/0"),
        ParseQuantityError::ZeroDenominator { span: 2..5 }
    );
    assert_eq!(
        err("1,000"),
        ParseQuantityError::DecimalComma { span: 1..2 }
    );
    assert_eq!(
        err("1.2,5"),
        ParseQuantityError::DecimalComma { span: 3..4 }
    );
    let trailing = |s: &str, span: std::ops::Range<usize>| {
        let e = err(s);
        assert!(
            matches!(&e, ParseQuantityError::Trailing { span: sp, text } if *sp == span && *text == s[span.clone()]),
            "{s:?}: {e:?}"
        );
    };
    trailing("1 2", 2..3);
    trailing("5 ft 3", 5..6);
    trailing("5' 3 mm", 3..7);
    trailing("5' 3\" x", 6..7);
    trailing("2 in 3\"", 5..7);
    assert_eq!(
        err("2 mmx"),
        ParseQuantityError::UnknownUnit {
            span: 2..5,
            text: "mmx".into()
        }
    );
    assert_eq!(
        err("2 mm)"),
        ParseQuantityError::UnknownUnit {
            span: 2..5,
            text: "mm)".into()
        }
    );
    assert_eq!(
        err("2 MM"),
        ParseQuantityError::UnknownUnit {
            span: 2..4,
            text: "MM".into()
        }
    );
    assert_eq!(
        err("5 µx"),
        ParseQuantityError::UnknownUnit {
            span: 2..5,
            text: "µx".into()
        }
    );
}

/// Regression: overflow used to parse as `Ok(inf)`.
#[test]
fn overflow_is_an_invalid_number() {
    let invalid = |s: &str, span: std::ops::Range<usize>| {
        let e = err(s);
        assert!(
            matches!(&e, ParseQuantityError::InvalidNumber { span: sp, text } if *sp == span && *text == s[span.clone()]),
            "{s:?}: {e:?}"
        );
    };
    invalid("1e400 mm", 0..5);
    invalid(" -1e400", 1..7);
    let nines = "9".repeat(400);
    invalid(&nines, 0..400);
    invalid(&format!("{nines}/1 in"), 0..402);
    invalid(&format!("{nines} 1/2"), 0..404);
    invalid("1e308' 1e308\"", 0..13);
    // Underflow is a legitimate (tiny) number, and huge denominators give 0.
    assert_eq!(parse("1e-400 mm").value.to_bits(), 0);
    assert_eq!(parse(&format!("1/{nines}")).value.to_bits(), 0);
    assert!(parse("1.7976931348623157e308").value.is_finite());
}

/// Surrounding CR/LF and non-breaking spaces (U+00A0, U+202F) are whitespace.
#[test]
fn whitespace_variants() {
    let cases: &[(&str, f64, Option<Unit>)] = &[
        ("12.5 mm\r\n", 12.5, Some(Unit::Millimetre)),
        ("\r\n 12.5\n", 12.5, None),
        ("12.5\u{A0}mm", 12.5, Some(Unit::Millimetre)),
        ("\u{A0}1\u{A0}1/4\u{A0}in\u{A0}", 1.25, Some(Unit::Inch)),
        ("5'\u{202F}3\"", 63.0, Some(Unit::Inch)),
        ("30\u{202F}deg", 30.0, Some(Unit::Degree)),
    ];
    for &(s, v, u) in cases {
        let p = parse(s);
        assert_eq!(p.unit, u, "{s:?}");
        assert_relative_eq!(p.value, v);
    }
    assert_eq!(err("\r\n\u{A0}"), ParseQuantityError::Empty);
    assert_eq!(
        err("3\u{A0}foo\u{A0}"),
        ParseQuantityError::UnknownUnit {
            span: 3..6,
            text: "foo".into()
        }
    );
    assert_eq!(
        err("1 m\u{A0}20 cm"),
        ParseQuantityError::Trailing {
            span: 5..10,
            text: "20 cm".into()
        }
    );
}

#[test]
fn error_spans_and_hints() {
    assert_eq!(ParseQuantityError::Empty.span(), None);
    for s in ["12,5 mm", "3 foo", "1/0 in", "1 m 20 cm", "x"] {
        let e = err(s);
        let span = e.span().unwrap();
        assert!(span.end <= s.len() && span.start < span.end, "{s:?}");
        assert!(e.hint().is_some());
    }
    assert!(ParseQuantityError::Empty.hint().is_some());
    assert_eq!(err("1/0").to_string(), "division by zero in fraction");
    assert_eq!(err("3 foo").to_string(), "unknown unit `foo`");
}

#[test]
fn core_error_variants() {
    let e: Error = err("3 foo").into();
    assert_eq!(e.to_string(), "unknown unit `foo`");
    assert!(e.hint().unwrap().contains("mm"));
    assert!(matches!(
        e,
        Error::Parse(ParseQuantityError::UnknownUnit { .. })
    ));
    let m = Error::DimensionMismatch {
        expected: Dimension::LENGTH,
        found: Dimension::FORCE,
    };
    assert_eq!(m.to_string(), "unit mismatch: expected mm, got N");
    assert!(m.hint().unwrap().contains("mm"));
    assert_eq!(m.severity(), crate::Severity::Error);
    assert!(!m.is_cancelled());
    // `transparent`: the parse error is the message, not a cause.
    assert!(std::error::Error::source(&e).is_none());
}

// ---- Typed quantities -------------------------------------------------------------------

#[test]
fn typed_constructors_and_accessors() {
    assert_relative_eq!(Length::mm(2.0).get(), 2.0);
    assert_eq!(Length::inches(0.25), Length::mm(6.35));
    assert_relative_eq!(Length::feet(1.0).get(), 304.8);
    assert_relative_eq!(Length::m(1.5).to_cm(), 150.0);
    assert_relative_eq!(Length::um(5.0).get(), 0.005);
    assert_relative_eq!(Length::mm(25.4).to_inches(), 1.0);
    assert_relative_eq!(Length::mm(304.8).to_unit(LengthUnit::Foot), 1.0);
    assert_relative_eq!(Length::mm(3.0).to_mm(), 3.0);
    assert_relative_eq!(Angle::deg(180.0).get(), PI);
    assert_relative_eq!(Angle::rad(PI).to_deg(), 180.0);
    assert_relative_eq!(Angle::rad(1.0).to_rad(), 1.0);
    assert_relative_eq!(Area::cm2(1.0).to_mm2(), 100.0);
    assert_relative_eq!(Area::in2(1.0).get(), 645.16);
    assert_relative_eq!(Area::m2(1.0).get(), 1e6);
    assert_relative_eq!(Volume::litres(1.0).to_cm3(), 1000.0);
    assert_relative_eq!(Volume::ml(1.0).get(), 1000.0);
    assert_relative_eq!(Volume::m3(1.0).to_litres(), 1000.0);
    assert_relative_eq!(Volume::in3(1.0).to_mm3(), 16387.064);
    assert_relative_eq!(Mass::kg(1.0).get(), 1e-3);
    assert_relative_eq!(Mass::g(500.0).to_kg(), 0.5);
    assert_relative_eq!(Mass::tonnes(2.0).to_g(), 2e6);
    assert_relative_eq!(Mass::lb(1.0).to_kg(), 0.45359237);
    assert_relative_eq!(Time::min(2.0).to_s(), 120.0);
    assert_relative_eq!(Time::h(1.0).to_min(), 60.0);
    assert_relative_eq!(Time::ms(1500.0).get(), 1.5);
    assert_relative_eq!(Force::kn(1.0).to_newtons(), 1000.0);
    assert_relative_eq!(Force::lbf(1.0).get(), 4.4482216152605);
    assert_relative_eq!(Pressure::gpa(210.0).to_mpa(), 210_000.0);
    assert_relative_eq!(Pressure::pa(1e6).get(), 1.0);
    assert_relative_eq!(Pressure::kpa(100.0).to_bar(), 1.0);
    assert_relative_eq!(Pressure::psi(1000.0).get(), 6.894757293168);
    assert_relative_eq!(
        Density::kg_per_m3(7850.0).get(),
        7.85e-9,
        max_relative = 1e-12
    );
    assert_relative_eq!(
        Density::g_per_cm3(7.85).to_kg_per_m3(),
        7850.0,
        max_relative = 1e-12
    );
    assert_relative_eq!(Energy::j(1.0).to_mj(), 1000.0);
    assert_relative_eq!(Temperature::celsius(20.0).get(), 293.15);
    assert_relative_eq!(
        Temperature::fahrenheit(212.0).to_celsius(),
        100.0,
        max_relative = 1e-12
    );
    assert_relative_eq!(
        Temperature::kelvin(0.0).to_fahrenheit(),
        -459.67,
        max_relative = 1e-12
    );
    assert_relative_eq!(Pressure::mpa(6.894757293168).to_psi(), 1000.0);
    assert_relative_eq!(Length::from_canonical(4.0).get(), 4.0);
    assert_relative_eq!(Length::mm(25.4).in_unit(Unit::Inch).unwrap(), 1.0);
    assert!(Length::mm(1.0).in_unit(Unit::Second).is_err());
    assert_eq!(Length::DIMENSION, Dimension::LENGTH);
    assert_eq!(Pressure::DIMENSION, Dimension::PRESSURE);
}

#[test]
fn typed_arithmetic() {
    let a = Length::mm(3.0);
    let b = Length::mm(1.0);
    assert_eq!(a + b, Length::mm(4.0));
    assert_eq!(a - b, Length::mm(2.0));
    assert_eq!(-a, Length::mm(-3.0));
    assert_eq!(a * 2.0, Length::mm(6.0));
    assert_eq!(2.0 * a, Length::mm(6.0));
    assert_eq!(a / 2.0, Length::mm(1.5));
    assert_relative_eq!(a / b, 3.0);
    let mut c = a;
    c += b;
    c -= Length::mm(0.5);
    assert_eq!(c, Length::mm(3.5));
    assert!(b < a);
    assert_eq!(Length::ZERO, Length::default());
    assert_eq!(a * b, Area::mm2(3.0));
    assert_eq!(a * b * a, Volume::mm3(9.0));
    assert_eq!(a * Area::mm2(2.0), Volume::mm3(6.0));
    assert_eq!(Area::mm2(6.0) / a, Length::mm(2.0));
    assert_eq!(Volume::mm3(6.0) / a, Area::mm2(2.0));
    assert_eq!(Volume::mm3(6.0) / Area::mm2(2.0), a);
    assert_eq!(Force::newtons(10.0) / Area::mm2(2.0), Pressure::mpa(5.0));
    assert_eq!(Pressure::mpa(5.0) * Area::mm2(2.0), Force::newtons(10.0));
    let rho = Mass::tonnes(1.0) / Volume::mm3(2.0);
    assert_eq!(rho, Density::from_canonical(0.5));
    assert_eq!(rho * Volume::mm3(2.0), Mass::tonnes(1.0));
    assert_eq!(Force::newtons(2.0) * a, Energy::mj(6.0));
    assert_eq!(Energy::mj(6.0) / a, Force::newtons(2.0));
}

#[test]
fn typed_quantity_conversions() {
    let q = Quantity::from(Force::kn(2.0));
    assert_eq!(q, Quantity::new(2000.0, Dimension::FORCE));
    assert_eq!(Force::try_from(q).unwrap(), Force::kn(2.0));
    assert!(matches!(
        Length::try_from(q),
        Err(Error::DimensionMismatch { expected, found })
            if expected == Dimension::LENGTH && found == Dimension::FORCE
    ));
    assert_eq!(Quantity::from(Angle::deg(90.0)).dim, Dimension::ANGLE);
    assert_eq!(
        Quantity::from(Temperature::kelvin(1.0)).dim,
        Dimension::TEMPERATURE
    );
    assert_eq!(Quantity::from(Time::s(1.0)).dim, Dimension::TIME);
    assert_eq!(Quantity::from(Mass::kg(1.0)).dim, Dimension::MASS);
    assert_eq!(
        Quantity::from(Density::g_per_cm3(1.0)).dim,
        Dimension::DENSITY
    );
    assert_eq!(Quantity::from(Energy::j(1.0)).dim, Dimension::ENERGY);
    assert_eq!(Quantity::from(Area::cm2(1.0)).dim, Dimension::AREA);
    assert_eq!(Quantity::from(Volume::cm3(1.0)).dim, Dimension::VOLUME);
    assert_eq!(Quantity::from(Pressure::bar(1.0)).dim, Dimension::PRESSURE);
}

#[test]
fn typed_serde_is_the_canonical_f64() {
    assert_eq!(serde_json::to_string(&Length::inches(1.0)).unwrap(), "25.4");
    assert_eq!(
        serde_json::from_str::<Mass>("0.001").unwrap(),
        Mass::kg(1.0)
    );
}

#[test]
fn angle_normalisation() {
    let n02 = |v: f64| Angle::rad(v).normalized_0_2pi().get();
    let npm = |v: f64| Angle::rad(v).normalized_pm_pi().get();
    assert_relative_eq!(n02(-PI / 2.0), 1.5 * PI);
    assert_relative_eq!(n02(5.0 * PI), PI);
    assert_relative_eq!(n02(TAU), 0.0);
    assert_relative_eq!(n02(-1e-20), 0.0);
    assert!((0.0..TAU).contains(&n02(-1e-20)));
    assert_relative_eq!(npm(1.5 * PI), -0.5 * PI);
    assert_relative_eq!(npm(PI), PI);
    assert_relative_eq!(npm(-PI), PI);
    assert_relative_eq!(npm(0.25), 0.25);
}

// ---- Formatting -------------------------------------------------------------------------

#[test]
fn decimal_style() {
    let d3 = spec(FormatStyle::Decimal, 3);
    assert_eq!(fmt(12.5, Unit::Millimetre, &d3), "12.500 mm");
    assert_eq!(fmt(-0.0001, Unit::Millimetre, &d3), "0.000 mm");
    let trim = FormatSpec {
        trim_zeros: true,
        ..d3
    };
    assert_eq!(fmt(12.5, Unit::Millimetre, &trim), "12.5 mm");
    assert_eq!(fmt(12.0, Unit::Millimetre, &trim), "12 mm");
    assert_eq!(fmt(-0.0001, Unit::Millimetre, &trim), "0 mm");
    assert_eq!(fmt(1200.0, Unit::Millimetre, &trim), "1200 mm");
    let bare = FormatSpec {
        show_unit: false,
        ..trim
    };
    assert_eq!(fmt(30.0, Unit::Degree, &bare), "30");
    assert_eq!(fmt(20.0, Unit::DegreeCelsius, &trim), "20 degC");
    assert_eq!(fmt(1.0, Unit::Inch, &spec(FormatStyle::Decimal, 0)), "1 in");
}

#[test]
fn for_unit_defaults() {
    let l = FormatSpec::for_unit(Unit::Millimetre);
    assert_eq!(l.style, FormatStyle::Decimal);
    assert_eq!(l.precision, 3);
    assert!(l.trim_zeros && l.show_unit);
    assert_eq!(FormatSpec::for_unit(Unit::Degree).precision, 2);
    assert_eq!(FormatSpec::for_unit(Unit::DegreeCelsius).precision, 2);
    assert_eq!(FormatSpec::for_unit(Unit::Newton).precision, 3);
    assert_eq!(
        FormatSpec::for_unit(Unit::TonnePerCubicMillimetre),
        FormatSpec::EXACT
    );
    let q = Quantity::from(Angle::rad(PI / 6.0));
    assert_eq!(
        format_quantity(q, Unit::Degree, &FormatSpec::for_unit(Unit::Degree)).unwrap(),
        "30 deg"
    );
}

#[test]
fn fraction_style() {
    let f = spec(
        FormatStyle::Fraction {
            max_denominator: 64,
        },
        3,
    );
    // FND-10 Done-when: 31.75 mm prints as `1 1/4 in`.
    let q = Quantity::from(Length::mm(31.75));
    assert_eq!(format_quantity(q, Unit::Inch, &f).unwrap(), "1 1/4 in");
    assert_eq!(fmt(0.375, Unit::Inch, &f), "3/8 in");
    assert_eq!(fmt(-2.5, Unit::Inch, &f), "-2 1/2 in");
    assert_eq!(fmt(3.0, Unit::Inch, &f), "3 in");
    assert_eq!(fmt(0.0, Unit::Inch, &f), "0 in");
    assert_eq!(fmt(-0.0001, Unit::Inch, &f), "0 in");
    assert_eq!(fmt(1.9999, Unit::Inch, &f), "2 in");
    assert_eq!(fmt(5.0 / 64.0, Unit::Inch, &f), "5/64 in");
    // Only binary fractions; others fall back to decimals.
    assert_eq!(fmt(1.0 / 3.0, Unit::Inch, &f), "0.333 in");
    assert_eq!(fmt(0.123, Unit::Inch, &f), "0.123 in");
    // Needs a denominator above 64: falls back to decimals.
    let f6 = spec(
        FormatStyle::Fraction {
            max_denominator: 64,
        },
        6,
    );
    assert_eq!(fmt(0.1234567, Unit::Inch, &f6), "0.123457 in");
    let small = spec(FormatStyle::Fraction { max_denominator: 4 }, 3);
    assert_eq!(fmt(0.375, Unit::Inch, &small), "0.375 in");
    let bare = FormatSpec {
        show_unit: false,
        ..f
    };
    assert_eq!(fmt(1.25, Unit::Inch, &bare), "1 1/4");
}

#[test]
fn feet_inch_style() {
    let fi = spec(
        FormatStyle::FeetInch {
            max_denominator: 64,
        },
        3,
    );
    let len = |mm: f64| format_quantity(Quantity::from(Length::mm(mm)), Unit::Foot, &fi).unwrap();
    assert_eq!(len(1612.9), "5' 3 1/2\"");
    assert_eq!(len(304.8 * 2.0), "2'");
    assert_eq!(len(25.4 * 3.25), "3 1/4\"");
    assert_eq!(len(0.0), "0\"");
    assert_eq!(len(-1612.9), "-5' 3 1/2\"");
    // 11.9999 in carries into the feet.
    assert_eq!(len(25.4 * 23.9999), "2'");
    // Decimal fallback, also carrying.
    let coarse = spec(FormatStyle::FeetInch { max_denominator: 2 }, 3);
    let len2 =
        |mm: f64| format_quantity(Quantity::from(Length::mm(mm)), Unit::Inch, &coarse).unwrap();
    assert_eq!(len2(25.4 * 15.123), "1' 3.123\"");
    assert_eq!(len2(25.4 * 11.9999), "1'");
    // FeetInch accepts any length target unit but no other dimension.
    let e = format_quantity(Quantity::from(Angle::deg(1.0)), Unit::Degree, &fi).unwrap_err();
    assert!(
        matches!(e, Error::DimensionMismatch { expected, .. } if expected == Dimension::LENGTH)
    );
}

#[test]
fn exact_style() {
    let e = FormatSpec::EXACT;
    assert_eq!(fmt(12.5, Unit::Millimetre, &e), "12.5 mm");
    assert_eq!(fmt(0.1, Unit::Millimetre, &e), "0.1 mm");
    assert_eq!(fmt(-0.0, Unit::Millimetre, &e), "-0 mm");
    assert_eq!(fmt(1e20, Unit::Millimetre, &e), "1e20 mm");
    assert_eq!(fmt(1.5e-7, Unit::Millimetre, &e), "1.5e-7 mm");
    assert_eq!(fmt(2.0, Unit::Inch, &e), "2 in");
    let q = Quantity::from(Density::from_canonical(7.85e-9));
    let s = format_quantity(q, Unit::TonnePerCubicMillimetre, &e).unwrap();
    assert_eq!(s, "7.85e-9 t/mm^3");
    // Hidden unit stays hidden when the target unit is used.
    let bare = FormatSpec {
        show_unit: false,
        ..e
    };
    assert_eq!(fmt(2.0, Unit::Inch, &bare), "2");
}

/// When no `f64` in the target unit maps exactly onto the value, Exact falls back to the
/// canonical unit and always prints its symbol.
#[test]
fn exact_falls_back_to_canonical_unit() {
    let bare = FormatSpec {
        show_unit: false,
        ..FormatSpec::EXACT
    };
    let mut fallbacks = 0;
    for i in 0..2000 {
        let v = 1.9 + f64::from(i) * 1e-13;
        let s = format_quantity(Quantity::new(v, Dimension::LENGTH), Unit::Inch, &bare).unwrap();
        if s.ends_with(" mm") {
            fallbacks += 1;
        } else {
            assert!(!s.contains(' '), "{s}");
        }
        let back = parse(&s).resolve(Unit::Inch).value;
        assert_eq!(back.to_bits(), v.to_bits(), "{s}");
    }
    // 25.4 is not a power of two, so where the mantissa of mm exceeds that of inches some
    // canonical values are unreachable from inches.
    assert!(fallbacks > 0);
    // Overflow in the target unit also falls back.
    let big = Quantity::new(f64::MAX, Dimension::LENGTH);
    let s = format_quantity(big, Unit::Micrometre, &bare).unwrap();
    assert_eq!(s, "1.7976931348623157e308 mm");
}

/// Regression: finite values that overflow in the target unit printed `inf um`.
#[test]
fn rounded_styles_fall_back_on_overflow() {
    let big = Quantity::new(f64::MAX, Dimension::LENGTH);
    for style in [
        FormatStyle::Decimal,
        FormatStyle::Fraction {
            max_denominator: 64,
        },
    ] {
        let s = FormatSpec {
            show_unit: false,
            ..spec(style, 3)
        };
        let text = format_quantity(big, Unit::Micrometre, &s).unwrap();
        assert_eq!(text, "1.7976931348623157e308 mm");
        assert_eq!(
            parse(&text).resolve(Unit::Micrometre).value.to_bits(),
            f64::MAX.to_bits()
        );
    }
}

#[test]
fn format_errors() {
    let q = Quantity::new(1.0, Dimension::LENGTH);
    let e = format_quantity(q, Unit::Degree, &FormatSpec::EXACT).unwrap_err();
    assert!(matches!(e, Error::DimensionMismatch { expected, found }
        if expected == Dimension::ANGLE && found == Dimension::LENGTH));
    for v in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let e = format_quantity(
            Quantity::new(v, Dimension::LENGTH),
            Unit::Millimetre,
            &FormatSpec::EXACT,
        );
        assert!(matches!(e, Err(Error::InvalidArgument { name: "q", .. })));
    }
}

// ---- Properties -------------------------------------------------------------------------

fn arb_unit() -> impl Strategy<Value = Unit> {
    prop::sample::select(Unit::all())
}
fn arb_finite() -> impl Strategy<Value = f64> {
    prop_oneof![
        any::<f64>().prop_filter("finite", |v| v.is_finite()),
        -1e6..1e6f64,
        (-10_000i32..10_000).prop_map(|i| f64::from(i) / 64.0),
    ]
}
/// Allowed canonical error for a rounded format: half a printed digit, plus float rounding
/// of the unit conversion (which involves the offset for °C/°F).
fn rounded_tol(v: f64, u: Unit, precision: u8) -> f64 {
    let half = 0.5 * 10f64.powi(-i32::from(precision)) * u.scale();
    half * (1.0 + 1e-9) + 8.0 * f64::EPSILON * (v.abs() + 500.0 * u.scale())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2048))]

    /// `Exact` round-trips bit-for-bit for every unit and any finite value.
    #[test]
    fn prop_exact_round_trips_bit_for_bit(v in arb_finite(), u in arb_unit()) {
        let q = Quantity::new(v, u.dimension());
        let s = format_quantity(q, u, &FormatSpec::EXACT).unwrap();
        let back = parse_quantity(&s).unwrap().resolve(u);
        prop_assert_eq!(back.dim, q.dim);
        prop_assert_eq!(back.value.to_bits(), v.to_bits(), "{} -> {}", v, s);
    }

    /// `Decimal` stays within half a unit of the last printed digit, for any finite value.
    #[test]
    fn prop_decimal_within_printed_precision(
        v in arb_finite(), u in arb_unit(), precision in 0u8..9, trim in any::<bool>(),
    ) {
        let s = FormatSpec { style: FormatStyle::Decimal, precision, trim_zeros: trim, show_unit: true };
        let text = format_quantity(Quantity::new(v, u.dimension()), u, &s).unwrap();
        let back = parse_quantity(&text).unwrap().resolve(u).value;
        prop_assert!((back - v).abs() <= rounded_tol(v, u, precision), "{} -> {} -> {}", v, text, back);
    }

    /// `Fraction` and `FeetInch` obey the same bound (fractions match within it).
    #[test]
    fn prop_fraction_and_feet_inch_within_precision(
        v in arb_finite(), max_denominator in 1u32..128, precision in 0u8..7,
    ) {
        let q = Quantity::new(v, Dimension::LENGTH);
        for (style, unit) in [
            (FormatStyle::Fraction { max_denominator }, Unit::Inch),
            (FormatStyle::FeetInch { max_denominator }, Unit::Inch),
            (FormatStyle::Fraction { max_denominator }, Unit::Millimetre),
        ] {
            let s = FormatSpec { style, precision, trim_zeros: true, show_unit: true };
            let text = format_quantity(q, unit, &s).unwrap();
            let back = parse_quantity(&text).unwrap().resolve(unit).value;
            prop_assert!((back - v).abs() <= rounded_tol(v, unit, precision), "{} -> {} -> {}", v, text, back);
        }
    }

    /// Dimension algebra: `(a * b) / b` has the dimension of `a`.
    #[test]
    fn prop_quantity_dimension_algebra(
        a in arb_finite(), b in 1e-3..1e3f64, ua in arb_unit(), ub in arb_unit(), n in -3i8..4,
    ) {
        let qa = Quantity::new(a, ua.dimension());
        let qb = Quantity::new(b, ub.dimension());
        prop_assert_eq!(((qa * qb) / qb).dim, qa.dim);
        prop_assert_eq!(qa.powi(n).dim, ua.dimension().pow(n));
        prop_assert_eq!(qa.checked_add(qb).is_ok(), ua.dimension() == ub.dimension());
    }

    /// The parser never panics, and error spans lie on char boundaries of the input.
    #[test]
    fn prop_parse_total(s in "[ 0-9./,'\"eE+\\-a-zµ°²^*]{0,12}") {
        if let Err(e) = parse_quantity(&s)
            && let Some(span) = e.span()
        {
            prop_assert!(span.start <= span.end && span.end <= s.len());
            prop_assert!(s.is_char_boundary(span.start) && s.is_char_boundary(span.end));
        }
    }
}

/// Absolute temperatures are affine: only differences (in K) and explicit offsets.
#[test]
fn temperature_is_affine() {
    let a = Temperature::celsius(30.0);
    let b = Temperature::celsius(20.0);
    let dk: f64 = a - b;
    assert_relative_eq!(dk, 10.0, epsilon = 1e-12);
    assert_relative_eq!(b.offset_k(dk).get(), a.get(), epsilon = 1e-12);
    assert!(b < a);
}
