//! Units and quantities. Design: `docs/notes/units-and-quantities.md`.
//!
//! - Canonical (stored) values use the coherent **mm, t, s, K, rad** system: forces are N,
//!   stresses MPa, densities t/mm³, energies mJ. Raw geometry `f64` stays plain mm/rad.
//! - [`Dimension`] tracks five exponents (length, mass, time, temperature, angle); angle is a
//!   real dimension, so `10 mm + 5 deg` is a [`DimensionMismatch`](crate::Error).
//! - [`Quantity`] is the dynamically dimensioned value used by expressions (S01).
//! - Typed newtypes ([`Length`], [`Angle`], [`Pressure`], ...) are for APIs at unit
//!   boundaries, not for inner loops.
//! - [`Unit`] is the closed table of parse/display units; [`parse_quantity`] reads one literal
//!   (`12.5 mm`, `1/4 in`, `5' 3 1/2"`) and [`format_quantity`] writes one back.
//!
//! ```
//! use amg_core::{parse_quantity, format_quantity, FormatSpec, Unit, Length};
//! let q = parse_quantity("1 1/4\"")?.resolve(Unit::Millimetre);
//! assert_eq!(Length::try_from(q)?, Length::mm(31.75));
//! let spec = FormatSpec::for_unit(Unit::Inch);
//! assert_eq!(format_quantity(q, Unit::Inch, &spec)?, "1.25 in");
//! # Ok::<(), amg_core::Error>(())
//! ```

mod format;
mod parse;
mod table;
mod typed;

#[cfg(test)]
mod tests;

pub use format::{FormatSpec, FormatStyle, format_quantity};
pub use parse::{ParseQuantityError, ParsedQuantity, parse_quantity};
pub use table::{LengthUnit, Unit};
pub use typed::{
    Angle, Area, Density, Energy, Force, Length, Mass, Pressure, Temperature, Time, Volume,
};

use crate::error::{Error, Result};

/// Physical dimension as exponents of the five base quantities.
///
/// Multiplication adds exponents, division subtracts them, [`pow`](Dimension::pow) scales
/// them. Exponent arithmetic saturates at the `i8` range instead of overflowing.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, Hash, Default, serde::Serialize, serde::Deserialize,
)]
pub struct Dimension {
    /// Exponent of length (mm).
    pub length: i8,
    /// Exponent of mass (t).
    pub mass: i8,
    /// Exponent of time (s).
    pub time: i8,
    /// Exponent of temperature (K).
    pub temperature: i8,
    /// Exponent of angle (rad).
    pub angle: i8,
}

impl Dimension {
    /// Builds a dimension from its exponents (length, mass, time, temperature, angle).
    pub const fn new(length: i8, mass: i8, time: i8, temperature: i8, angle: i8) -> Dimension {
        Dimension {
            length,
            mass,
            time,
            temperature,
            angle,
        }
    }

    /// Dimensionless (all exponents zero).
    pub const NONE: Dimension = Dimension::new(0, 0, 0, 0, 0);
    /// Length, canonical mm.
    pub const LENGTH: Dimension = Dimension::new(1, 0, 0, 0, 0);
    /// Area, canonical mm².
    pub const AREA: Dimension = Dimension::new(2, 0, 0, 0, 0);
    /// Volume, canonical mm³.
    pub const VOLUME: Dimension = Dimension::new(3, 0, 0, 0, 0);
    /// Plane angle, canonical rad.
    pub const ANGLE: Dimension = Dimension::new(0, 0, 0, 0, 1);
    /// Mass, canonical t (tonne).
    pub const MASS: Dimension = Dimension::new(0, 1, 0, 0, 0);
    /// Time, canonical s.
    pub const TIME: Dimension = Dimension::new(0, 0, 1, 0, 0);
    /// Force, canonical N (t·mm/s²).
    pub const FORCE: Dimension = Dimension::new(1, 1, -2, 0, 0);
    /// Pressure or stress, canonical MPa (N/mm²).
    pub const PRESSURE: Dimension = Dimension::new(-1, 1, -2, 0, 0);
    /// Density, canonical t/mm³.
    pub const DENSITY: Dimension = Dimension::new(-3, 1, 0, 0, 0);
    /// Energy, canonical mJ (N·mm).
    pub const ENERGY: Dimension = Dimension::new(2, 1, -2, 0, 0);
    /// Absolute temperature, canonical K.
    pub const TEMPERATURE: Dimension = Dimension::new(0, 0, 0, 1, 0);

    /// Every exponent multiplied by `n` (`LENGTH.pow(2) == AREA`).
    pub fn pow(self, n: i8) -> Dimension {
        self.zip(Dimension::NONE, |a, _| a.saturating_mul(n))
    }

    fn zip(self, o: Dimension, f: impl Fn(i8, i8) -> i8) -> Dimension {
        Dimension {
            length: f(self.length, o.length),
            mass: f(self.mass, o.mass),
            time: f(self.time, o.time),
            temperature: f(self.temperature, o.temperature),
            angle: f(self.angle, o.angle),
        }
    }

    /// Canonical name of a well-known dimension.
    fn named(self) -> Option<&'static str> {
        Some(match self {
            Dimension::NONE => "1",
            Dimension::LENGTH => "mm",
            Dimension::AREA => "mm^2",
            Dimension::VOLUME => "mm^3",
            Dimension::ANGLE => "rad",
            Dimension::MASS => "t",
            Dimension::TIME => "s",
            Dimension::FORCE => "N",
            Dimension::PRESSURE => "MPa",
            Dimension::DENSITY => "t/mm^3",
            Dimension::ENERGY => "mJ",
            Dimension::TEMPERATURE => "K",
            _ => return None,
        })
    }
}

impl std::ops::Mul for Dimension {
    type Output = Dimension;
    fn mul(self, o: Dimension) -> Dimension {
        self.zip(o, i8::saturating_add)
    }
}

impl std::ops::Div for Dimension {
    type Output = Dimension;
    fn div(self, o: Dimension) -> Dimension {
        self.zip(o, i8::saturating_sub)
    }
}

impl std::fmt::Display for Dimension {
    /// The canonical unit: a name for well-known dimensions (`mm^2`, `N`, `MPa`, `1` for
    /// dimensionless), otherwise base units with exponents (`mm*t/s^3`, `1/s`).
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(n) = self.named() {
            return f.write_str(n);
        }
        let parts = [
            ("mm", self.length),
            ("t", self.mass),
            ("s", self.time),
            ("K", self.temperature),
            ("rad", self.angle),
        ];
        let write = |f: &mut std::fmt::Formatter<'_>, sym: &str, e: i8| {
            if e == 1 {
                f.write_str(sym)
            } else {
                write!(f, "{sym}^{e}")
            }
        };
        let mut first = true;
        for &(sym, e) in parts.iter().filter(|p| p.1 > 0) {
            if !first {
                f.write_str("*")?;
            }
            first = false;
            write(f, sym, e)?;
        }
        if first {
            f.write_str("1")?;
        }
        for &(sym, e) in parts.iter().filter(|p| p.1 < 0) {
            f.write_str("/")?;
            write(f, sym, e.saturating_neg())?;
        }
        Ok(())
    }
}

/// Value in canonical coherent units (mm, t, s, K, rad) with its dimension.
///
/// Temperatures are always absolute kelvin here; see the units note §5.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Quantity {
    /// Value in the canonical unit of `dim`.
    pub value: f64,
    /// Dimension of the value.
    pub dim: Dimension,
}

impl Quantity {
    /// A quantity from a canonical value.
    pub const fn new(value: f64, dim: Dimension) -> Quantity {
        Quantity { value, dim }
    }

    /// A dimensionless quantity.
    pub fn dimensionless(v: f64) -> Quantity {
        Quantity::new(v, Dimension::NONE)
    }

    /// The value expressed in `u` (`Err(DimensionMismatch)` if the dimensions differ).
    pub fn in_unit(&self, u: Unit) -> Result<f64> {
        check_dim(u.dimension(), self.dim)?;
        Ok(u.from_canonical(self.value))
    }

    /// `v` in unit `u`, converted to canonical.
    pub fn from_unit(v: f64, u: Unit) -> Quantity {
        Quantity::new(u.to_canonical(v), u.dimension())
    }

    /// `self + o`; `Err(DimensionMismatch)` unless both have the same dimension.
    pub fn checked_add(self, o: Quantity) -> Result<Quantity> {
        check_dim(self.dim, o.dim)?;
        Ok(Quantity::new(self.value + o.value, self.dim))
    }

    /// `self - o`; `Err(DimensionMismatch)` unless both have the same dimension.
    pub fn checked_sub(self, o: Quantity) -> Result<Quantity> {
        check_dim(self.dim, o.dim)?;
        Ok(Quantity::new(self.value - o.value, self.dim))
    }

    /// `self` to the integer power `n` (value and dimension).
    pub fn powi(self, n: i8) -> Quantity {
        Quantity::new(self.value.powi(i32::from(n)), self.dim.pow(n))
    }

    /// The canonical value if the dimension is `d`, else `Err(DimensionMismatch)`.
    pub fn expect_dim(self, d: Dimension) -> Result<f64> {
        check_dim(d, self.dim)?;
        Ok(self.value)
    }
}

impl std::ops::Mul for Quantity {
    type Output = Quantity;
    fn mul(self, o: Quantity) -> Quantity {
        Quantity::new(self.value * o.value, self.dim * o.dim)
    }
}

impl std::ops::Div for Quantity {
    type Output = Quantity;
    fn div(self, o: Quantity) -> Quantity {
        Quantity::new(self.value / o.value, self.dim / o.dim)
    }
}

impl std::ops::Neg for Quantity {
    type Output = Quantity;
    fn neg(self) -> Quantity {
        Quantity::new(-self.value, self.dim)
    }
}

impl std::fmt::Display for Quantity {
    /// Canonical value (Rust `f64` display) and unit, e.g. `12.5 mm`, `2` (dimensionless).
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Display::fmt(&self.value, f)?;
        if self.dim != Dimension::NONE {
            write!(f, " {}", self.dim)?;
        }
        Ok(())
    }
}

fn check_dim(expected: Dimension, found: Dimension) -> Result<()> {
    if expected == found {
        Ok(())
    } else {
        Err(Error::DimensionMismatch { expected, found })
    }
}
