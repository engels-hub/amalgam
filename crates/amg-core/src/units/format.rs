//! Formatter (units note §8). Everything it prints parses back with
//! [`parse_quantity`](super::parse_quantity).

use super::{Dimension, Quantity, Unit};
use crate::error::{Error, Result};

/// How [`format_quantity`] writes the number.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FormatStyle {
    /// Fixed `precision` decimals (`12.500 mm`, or `12.5 mm` with `trim_zeros`).
    Decimal,
    /// Inch-style mixed fractions (`1 1/4 in`): binary denominators 1, 2, 4, ... up to
    /// `max_denominator`, smallest first. A value that no such fraction matches within the
    /// printed precision (`0.5 · 10^-precision`) falls back to `Decimal`.
    Fraction {
        /// Largest denominator tried (64 is usual for inches).
        max_denominator: u32,
    },
    /// Feet and inches with fractional inches (`5' 3 1/2"`). Needs a length; the target
    /// unit and `show_unit` are ignored. Inches fall back to decimals like `Fraction`.
    FeetInch {
        /// Largest denominator tried for the inches.
        max_denominator: u32,
    },
    /// Shortest decimal that parses back to the identical `f64` (`.amg` files, Python API).
    /// If no `f64` in the target unit maps exactly onto the canonical value, the value is
    /// printed in the canonical unit of its dimension instead, always with its symbol.
    /// `precision` and `trim_zeros` are ignored.
    Exact,
}

/// Formatting options for [`format_quantity`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FormatSpec {
    /// Number style.
    pub style: FormatStyle,
    /// Decimals printed (`Decimal`, and the fallback and matching tolerance of
    /// `Fraction`/`FeetInch`).
    pub precision: u8,
    /// Drop trailing zeros (and a trailing `.`) after the decimal point.
    pub trim_zeros: bool,
    /// Append ` <symbol>`.
    pub show_unit: bool,
}

impl FormatSpec {
    /// The lossless spec: [`FormatStyle::Exact`] with the unit shown.
    pub const EXACT: FormatSpec = FormatSpec {
        style: FormatStyle::Exact,
        precision: 0,
        trim_zeros: false,
        show_unit: true,
    };

    /// Display default for `u`: `Decimal` with 3 decimals (2 for angles and temperatures),
    /// trailing zeros trimmed, unit shown. `t/mm^3` (values around 1e-9) uses `Exact`.
    pub fn for_unit(u: Unit) -> FormatSpec {
        if u == Unit::TonnePerCubicMillimetre {
            return FormatSpec::EXACT;
        }
        let dim = u.dimension();
        let precision = if dim == Dimension::ANGLE || dim == Dimension::TEMPERATURE {
            2
        } else {
            3
        };
        FormatSpec {
            style: FormatStyle::Decimal,
            precision,
            trim_zeros: true,
            show_unit: true,
        }
    }
}

/// Formats `q` in `unit`.
///
/// `Err(DimensionMismatch)` if `q.dim != unit.dimension()` (for `FeetInch`: unless `q` is a
/// length); `Err(InvalidArgument)` if the value is not finite.
///
/// Round-trip contract (property-tested): for finite `v`, parsing the output and converting
/// to canonical gives `v` within `0.5 · 10^-precision · unit.scale()` (plus float rounding of
/// the unit conversion); with [`FormatStyle::Exact`] it gives `v` bit-for-bit.
///
/// ```
/// use amg_core::{format_quantity, FormatSpec, FormatStyle, Length, Quantity, Unit};
/// let q = Quantity::from(Length::mm(31.75));
/// let frac = FormatSpec { style: FormatStyle::Fraction { max_denominator: 64 },
///                         ..FormatSpec::for_unit(Unit::Inch) };
/// assert_eq!(format_quantity(q, Unit::Inch, &frac)?, "1 1/4 in");
/// assert_eq!(format_quantity(q, Unit::Millimetre, &FormatSpec::EXACT)?, "31.75 mm");
/// # Ok::<(), amg_core::Error>(())
/// ```
pub fn format_quantity(q: Quantity, unit: Unit, spec: &FormatSpec) -> Result<String> {
    let expected = match spec.style {
        FormatStyle::FeetInch { .. } => Dimension::LENGTH,
        _ => unit.dimension(),
    };
    if q.dim != expected {
        return Err(Error::DimensionMismatch {
            expected,
            found: q.dim,
        });
    }
    if !q.value.is_finite() {
        return Err(Error::InvalidArgument {
            name: "q",
            reason: format!("cannot format the non-finite value {}", q.value),
        });
    }
    let tol = 0.5 * 10f64.powi(-i32::from(spec.precision));
    let with_unit = |s: String, u: Unit, show: bool| {
        if show {
            format!("{s} {}", u.symbol())
        } else {
            s
        }
    };
    Ok(match spec.style {
        FormatStyle::Decimal => with_unit(
            decimal(unit.from_canonical(q.value), spec),
            unit,
            spec.show_unit,
        ),
        FormatStyle::Fraction { max_denominator } => {
            let x = unit.from_canonical(q.value);
            let s = match fraction_parts(x.abs(), max_denominator, tol) {
                Some((w, n, d)) => signed(x < 0.0, mixed(w, n, d)),
                None => decimal(x, spec),
            };
            with_unit(s, unit, spec.show_unit)
        }
        FormatStyle::FeetInch { max_denominator } => feet_inch(
            Unit::Inch.from_canonical(q.value),
            max_denominator,
            tol,
            spec,
        ),
        FormatStyle::Exact => {
            let (s, u, forced) = exact(q.value, unit);
            with_unit(s, u, spec.show_unit || forced)
        }
    })
}

/// Fixed decimals, optional trimming, `-0` normalised to `0`.
fn decimal(x: f64, spec: &FormatSpec) -> String {
    let mut s = format!("{:.*}", usize::from(spec.precision), x);
    if spec.trim_zeros && s.contains('.') {
        let keep = s.trim_end_matches('0').trim_end_matches('.').len();
        s.truncate(keep);
    }
    if s.starts_with('-') && s[1..].bytes().all(|b| b == b'0' || b == b'.') {
        s.remove(0);
    }
    s
}

fn signed(negative: bool, s: String) -> String {
    if negative && s != "0" {
        format!("-{s}")
    } else {
        s
    }
}

/// `a >= 0` as `whole + n/d` with the smallest power-of-two `d <= max_den` within `tol`;
/// `n < d`.
fn fraction_parts(a: f64, max_den: u32, tol: f64) -> Option<(f64, u32, u32)> {
    let whole = a.floor();
    let frac = a - whole;
    let dens = std::iter::successors(Some(1u32), |d| d.checked_mul(2));
    dens.take_while(|&d| d <= max_den.max(1)).find_map(|d| {
        let df = f64::from(d);
        let n = (frac * df).round();
        if (frac - n / df).abs() > tol {
            return None;
        }
        // 0 <= n <= d, so the cast is exact.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n = n as u32;
        Some(if n == d {
            (whole + 1.0, 0, 1)
        } else {
            (whole, n, d)
        })
    })
}

/// `1 1/4`, `1/4`, `1` or `0`.
fn mixed(whole: f64, n: u32, d: u32) -> String {
    match (whole == 0.0, n == 0) {
        (_, true) => format!("{whole}"),
        (true, false) => format!("{n}/{d}"),
        (false, false) => format!("{whole} {n}/{d}"),
    }
}

fn feet_inch(inches: f64, max_den: u32, tol: f64, spec: &FormatSpec) -> String {
    let a = inches.abs();
    let mut feet = (a / 12.0).floor();
    let rem = (a - feet * 12.0).max(0.0);
    let inch = match fraction_parts(rem, max_den, tol) {
        Some((mut w, n, d)) => {
            if w >= 12.0 {
                feet += 1.0;
                w -= 12.0;
            }
            mixed(w, n, d)
        }
        None => {
            let mut r = rem;
            let step = 10f64.powi(i32::from(spec.precision));
            if (r * step).round() / step >= 12.0 {
                feet += 1.0;
                r = (r - 12.0).max(0.0);
            }
            decimal(r, spec)
        }
    };
    let s = match (feet == 0.0, inch == "0") {
        (true, _) => format!("{inch}\""),
        (false, true) => format!("{feet}'"),
        (false, false) => format!("{feet}' {inch}\""),
    };
    if inches < 0.0 && s != "0\"" {
        format!("-{s}")
    } else {
        s
    }
}

/// Shortest round-trip text of `v` in `unit`, or in the canonical unit if no `f64` in
/// `unit` converts back exactly. Returns (text, unit used, unit must be shown).
fn exact(v: f64, unit: Unit) -> (String, Unit, bool) {
    let hits = |x: f64| x.is_finite() && unit.to_canonical(x).to_bits() == v.to_bits();
    let x0 = unit.from_canonical(v);
    if hits(x0) {
        return (shortest(x0), unit, false);
    }
    if x0.is_finite() {
        let (mut up, mut down) = (x0, x0);
        for _ in 0..8 {
            up = up.next_up();
            down = down.next_down();
            for x in [up, down] {
                if hits(x) {
                    return (shortest(x), unit, false);
                }
            }
        }
    }
    let canon = Unit::canonical_for(unit.dimension()).unwrap_or(unit);
    (shortest(canon.from_canonical(v)), canon, canon != unit)
}

/// Shortest decimal that parses back to `x` exactly; exponent form outside `[1e-5, 1e16)`.
fn shortest(x: f64) -> String {
    let a = x.abs();
    if a == 0.0 || (1e-5..1e16).contains(&a) {
        format!("{x}")
    } else {
        format!("{x:e}")
    }
}
