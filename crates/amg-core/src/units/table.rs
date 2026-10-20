//! The unit table (units note §6). Adding a unit means adding a variant, a row in [`TABLE`]
//! and a test.

use std::f64::consts::{PI, TAU};

use super::Dimension;

/// Every parse/display unit (table: `docs/notes/units-and-quantities.md` §6).
///
/// Symbols are case-sensitive. Composite symbols such as `kg/m^3` are atoms of the table, not
/// expressions; unit algebra belongs to S01 expressions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum Unit {
    /// µm (1e-3 mm).
    Micrometre,
    /// mm, the canonical length.
    Millimetre,
    /// cm.
    Centimetre,
    /// m.
    Metre,
    /// in (25.4 mm exactly).
    Inch,
    /// ft (304.8 mm exactly).
    Foot,
    /// mil / thou (0.0254 mm).
    Thou,
    /// rad, the canonical angle.
    Radian,
    /// deg (π/180 rad).
    Degree,
    /// grad (π/200 rad).
    Gradian,
    /// rev / turn (2π rad).
    Turn,
    /// mm², the canonical area.
    SquareMillimetre,
    /// cm².
    SquareCentimetre,
    /// m².
    SquareMetre,
    /// in².
    SquareInch,
    /// mm³, the canonical volume.
    CubicMillimetre,
    /// cm³ / cc.
    CubicCentimetre,
    /// m³.
    CubicMetre,
    /// L (litre, 1e6 mm³).
    Litre,
    /// mL (1e3 mm³).
    Millilitre,
    /// in³.
    CubicInch,
    /// g.
    Gram,
    /// kg.
    Kilogram,
    /// t (tonne), the canonical mass.
    Tonne,
    /// lb (avoirdupois pound, 0.45359237 kg exactly).
    Pound,
    /// oz (avoirdupois ounce).
    Ounce,
    /// ms.
    Millisecond,
    /// s, the canonical time.
    Second,
    /// min.
    Minute,
    /// h.
    Hour,
    /// N, the canonical force.
    Newton,
    /// kN.
    Kilonewton,
    /// lbf.
    PoundForce,
    /// Pa.
    Pascal,
    /// kPa.
    Kilopascal,
    /// MPa (N/mm²), the canonical pressure.
    Megapascal,
    /// GPa.
    Gigapascal,
    /// bar (0.1 MPa).
    Bar,
    /// psi.
    Psi,
    /// ksi.
    Ksi,
    /// kg/m³.
    KilogramPerCubicMetre,
    /// g/cm³.
    GramPerCubicCentimetre,
    /// t/mm³, the canonical density (FEA input decks). Not in the note's table; added so
    /// every dimension has a canonical unit for `FormatStyle::Exact`.
    TonnePerCubicMillimetre,
    /// mJ (N·mm), the canonical energy.
    Millijoule,
    /// J.
    Joule,
    /// K, the canonical (absolute) temperature.
    Kelvin,
    /// °C, absolute (affine: `20 degC` is 293.15 K).
    DegreeCelsius,
    /// °F, absolute (affine).
    DegreeFahrenheit,
    /// A temperature difference in °C (scale 1, no offset): `10 delta_degC` is 10 K.
    DeltaCelsius,
}

struct Info {
    unit: Unit,
    dim: Dimension,
    /// Canonical = (value + offset) · scale.
    scale: f64,
    /// Offset in the unit itself (non-zero only for absolute °C/°F).
    offset: f64,
    /// Printed symbol first, then the accepted aliases.
    symbols: &'static [&'static str],
}

const fn row(unit: Unit, dim: Dimension, scale: f64, symbols: &'static [&'static str]) -> Info {
    Info {
        unit,
        dim,
        scale,
        offset: 0.0,
        symbols,
    }
}

use Dimension as D;
use Unit as U;

/// Indexed by `Unit as usize`; a test checks the order.
static TABLE: [Info; 49] = [
    row(U::Micrometre, D::LENGTH, 1e-3, &["um", "µm", "μm"]),
    row(U::Millimetre, D::LENGTH, 1.0, &["mm"]),
    row(U::Centimetre, D::LENGTH, 10.0, &["cm"]),
    row(U::Metre, D::LENGTH, 1000.0, &["m"]),
    row(U::Inch, D::LENGTH, 25.4, &["in", "\"", "inch"]),
    row(U::Foot, D::LENGTH, 304.8, &["ft", "'", "foot", "feet"]),
    row(U::Thou, D::LENGTH, 0.0254, &["mil", "thou"]),
    row(U::Radian, D::ANGLE, 1.0, &["rad"]),
    row(U::Degree, D::ANGLE, PI / 180.0, &["deg", "°"]),
    row(U::Gradian, D::ANGLE, PI / 200.0, &["grad"]),
    row(U::Turn, D::ANGLE, TAU, &["rev", "turn"]),
    row(U::SquareMillimetre, D::AREA, 1.0, &["mm^2", "mm²"]),
    row(U::SquareCentimetre, D::AREA, 100.0, &["cm^2"]),
    row(U::SquareMetre, D::AREA, 1e6, &["m^2"]),
    row(U::SquareInch, D::AREA, 645.16, &["in^2"]),
    row(U::CubicMillimetre, D::VOLUME, 1.0, &["mm^3", "mm³"]),
    row(U::CubicCentimetre, D::VOLUME, 1e3, &["cm^3", "cc"]),
    row(U::CubicMetre, D::VOLUME, 1e9, &["m^3"]),
    row(U::Litre, D::VOLUME, 1e6, &["L", "l"]),
    row(U::Millilitre, D::VOLUME, 1e3, &["mL", "ml"]),
    row(U::CubicInch, D::VOLUME, 16387.064, &["in^3"]),
    row(U::Gram, D::MASS, 1e-6, &["g"]),
    row(U::Kilogram, D::MASS, 1e-3, &["kg"]),
    row(U::Tonne, D::MASS, 1.0, &["t"]),
    row(U::Pound, D::MASS, 4.5359237e-4, &["lb", "lbm"]),
    row(U::Ounce, D::MASS, 2.8349523125e-5, &["oz"]),
    row(U::Millisecond, D::TIME, 1e-3, &["ms"]),
    row(U::Second, D::TIME, 1.0, &["s"]),
    row(U::Minute, D::TIME, 60.0, &["min"]),
    row(U::Hour, D::TIME, 3600.0, &["h"]),
    row(U::Newton, D::FORCE, 1.0, &["N"]),
    row(U::Kilonewton, D::FORCE, 1e3, &["kN"]),
    row(U::PoundForce, D::FORCE, 4.4482216152605, &["lbf"]),
    row(U::Pascal, D::PRESSURE, 1e-6, &["Pa"]),
    row(U::Kilopascal, D::PRESSURE, 1e-3, &["kPa"]),
    row(U::Megapascal, D::PRESSURE, 1.0, &["MPa", "N/mm^2"]),
    row(U::Gigapascal, D::PRESSURE, 1e3, &["GPa"]),
    row(U::Bar, D::PRESSURE, 0.1, &["bar"]),
    row(U::Psi, D::PRESSURE, 6.894757293168e-3, &["psi"]),
    row(U::Ksi, D::PRESSURE, 6.894757293168, &["ksi"]),
    row(U::KilogramPerCubicMetre, D::DENSITY, 1e-12, &["kg/m^3"]),
    row(
        U::GramPerCubicCentimetre,
        D::DENSITY,
        1e-9,
        &["g/cm^3", "g/cc"],
    ),
    row(U::TonnePerCubicMillimetre, D::DENSITY, 1.0, &["t/mm^3"]),
    row(U::Millijoule, D::ENERGY, 1.0, &["mJ", "N*mm"]),
    row(U::Joule, D::ENERGY, 1e3, &["J"]),
    row(U::Kelvin, D::TEMPERATURE, 1.0, &["K"]),
    Info {
        offset: 273.15,
        ..row(U::DegreeCelsius, D::TEMPERATURE, 1.0, &["degC", "°C"])
    },
    Info {
        offset: 459.67,
        ..row(
            U::DegreeFahrenheit,
            D::TEMPERATURE,
            5.0 / 9.0,
            &["degF", "°F"],
        )
    },
    row(U::DeltaCelsius, D::TEMPERATURE, 1.0, &["delta_degC"]),
];

/// `Unit::all()` in declaration order.
static ALL: [Unit; 49] = {
    let mut all = [Unit::Millimetre; 49];
    let mut i = 0;
    while i < TABLE.len() {
        all[i] = TABLE[i].unit;
        i += 1;
    }
    all
};

impl Unit {
    fn info(self) -> &'static Info {
        &TABLE[self as usize]
    }

    /// Dimension this unit measures.
    pub fn dimension(self) -> Dimension {
        self.info().dim
    }

    /// The printed symbol (`mm`, `in`, `deg`, `MPa`, `degC`).
    pub fn symbol(self) -> &'static str {
        self.info().symbols[0]
    }

    /// Other accepted spellings, excluding [`symbol`](Unit::symbol) (`"`, `inch` for
    /// [`Unit::Inch`]). Empty for most units.
    pub fn aliases(self) -> &'static [&'static str] {
        &self.info().symbols[1..]
    }

    /// All spellings: the symbol followed by the aliases.
    #[cfg(test)]
    pub(crate) fn symbols(self) -> &'static [&'static str] {
        self.info().symbols
    }

    /// Multiplier to canonical (`25.4` for inch, `5/9` for °F).
    pub fn scale(self) -> f64 {
        self.info().scale
    }

    /// `v` in this unit, converted to canonical: `(v + offset) · scale`. The offset is
    /// non-zero only for absolute °C/°F; without it `-0.0` keeps its sign.
    pub fn to_canonical(self, v: f64) -> f64 {
        let i = self.info();
        if i.offset == 0.0 {
            v * i.scale
        } else {
            (v + i.offset) * i.scale
        }
    }

    /// Canonical `v` converted to this unit: `v / scale - offset`.
    pub fn from_canonical(self, v: f64) -> f64 {
        let i = self.info();
        if i.offset == 0.0 {
            v / i.scale
        } else {
            v / i.scale - i.offset
        }
    }

    /// The unit whose symbol or alias is exactly `s` (case-sensitive).
    pub fn from_symbol(s: &str) -> Option<Unit> {
        TABLE
            .iter()
            .find(|i| i.symbols.contains(&s))
            .map(|i| i.unit)
    }

    /// Every unit, in declaration order.
    pub fn all() -> &'static [Unit] {
        &ALL
    }

    /// Every (symbol, unit) pair, longest symbol first (the parser's longest-match order).
    pub(crate) fn symbols_longest_first() -> &'static [(&'static str, Unit)] {
        static SYMBOLS: std::sync::OnceLock<Vec<(&'static str, Unit)>> = std::sync::OnceLock::new();
        SYMBOLS.get_or_init(|| {
            let mut v: Vec<_> = TABLE
                .iter()
                .flat_map(|i| i.symbols.iter().map(move |&s| (s, i.unit)))
                .collect();
            v.sort_by_key(|&(s, _)| std::cmp::Reverse(s.len()));
            v
        })
    }

    /// The unit with scale 1 and no offset for `dim`, if the table has one.
    #[allow(clippy::float_cmp, reason = "exact table constants")]
    pub(crate) fn canonical_for(dim: Dimension) -> Option<Unit> {
        TABLE
            .iter()
            .find(|i| i.dim == dim && i.scale == 1.0 && i.offset == 0.0)
            .map(|i| i.unit)
    }
}

impl std::fmt::Display for Unit {
    /// The printed [`symbol`](Unit::symbol).
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.symbol())
    }
}

/// Subset used for the document length unit and display preferences.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, Hash, Default, serde::Serialize, serde::Deserialize,
)]
pub enum LengthUnit {
    /// µm.
    Micrometre,
    /// mm (default).
    #[default]
    Millimetre,
    /// cm.
    Centimetre,
    /// m.
    Metre,
    /// in.
    Inch,
    /// ft.
    Foot,
}

impl LengthUnit {
    /// Every length unit, in declaration order.
    pub const ALL: [LengthUnit; 6] = [
        LengthUnit::Micrometre,
        LengthUnit::Millimetre,
        LengthUnit::Centimetre,
        LengthUnit::Metre,
        LengthUnit::Inch,
        LengthUnit::Foot,
    ];
}

impl From<LengthUnit> for Unit {
    fn from(u: LengthUnit) -> Unit {
        match u {
            LengthUnit::Micrometre => Unit::Micrometre,
            LengthUnit::Millimetre => Unit::Millimetre,
            LengthUnit::Centimetre => Unit::Centimetre,
            LengthUnit::Metre => Unit::Metre,
            LengthUnit::Inch => Unit::Inch,
            LengthUnit::Foot => Unit::Foot,
        }
    }
}

impl TryFrom<Unit> for LengthUnit {
    type Error = Unit;
    /// The matching [`LengthUnit`]; `Err(u)` for units outside the subset.
    fn try_from(u: Unit) -> Result<LengthUnit, Unit> {
        LengthUnit::ALL
            .into_iter()
            .find(|&l| Unit::from(l) == u)
            .ok_or(u)
    }
}

#[cfg(test)]
pub(super) fn table_is_ordered() -> bool {
    TABLE
        .iter()
        .enumerate()
        .all(|(i, info)| info.unit as usize == i && !info.symbols.is_empty())
}
