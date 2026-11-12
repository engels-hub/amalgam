//! Result rows for display and copy (S05 §2 "Formatting").

use amg_core::{Dimension, FormatSpec, LengthUnit, Quantity, Unit, format_quantity};

use super::{EntityMeasure, MeasureReport};

/// Display settings of the measure tool.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DisplayPrefs {
    /// Length unit (document display unit).
    pub length: LengthUnit,
    /// Angle unit (`Unit::Degree` by default).
    pub angle: Unit,
    /// Decimals, 0–6.
    pub precision: u8,
}

impl Default for DisplayPrefs {
    /// mm, degrees, 3 decimals.
    fn default() -> Self {
        DisplayPrefs {
            length: LengthUnit::Millimetre,
            angle: Unit::Degree,
            precision: 3,
        }
    }
}

/// Area unit that goes with a length unit: mm → mm², cm → cm², m → m², in → in², ft → in²
/// (no ft² unit), µm → mm² (no µm² unit).
pub fn area_unit(l: LengthUnit) -> Unit {
    match l {
        LengthUnit::Micrometre | LengthUnit::Millimetre => Unit::SquareMillimetre,
        LengthUnit::Centimetre => Unit::SquareCentimetre,
        LengthUnit::Metre => Unit::SquareMetre,
        LengthUnit::Inch | LengthUnit::Foot => Unit::SquareInch,
    }
}

/// Volume unit that goes with a length unit: mm → mm³, cm → cm³, m → m³, in → in³, ft → in³,
/// µm → mm³.
pub fn volume_unit(l: LengthUnit) -> Unit {
    match l {
        LengthUnit::Micrometre | LengthUnit::Millimetre => Unit::CubicMillimetre,
        LengthUnit::Centimetre => Unit::CubicCentimetre,
        LengthUnit::Metre => Unit::CubicMetre,
        LengthUnit::Inch | LengthUnit::Foot => Unit::CubicInch,
    }
}

/// One displayed result.
#[derive(Clone, Debug, PartialEq)]
pub struct MeasureRow {
    /// Stable key: `distance`, `dx`, `dy`, `dz`, `angle`, `center_distance`, then per entity
    /// `first.*` / `second.*` (`x`, `y`, `z`, `length`, `area`, `volume`, `diameter`,
    /// `radius`).
    pub key: &'static str,
    /// Label shown in the results table.
    pub label: String,
    /// The value.
    pub value: Quantity,
    /// Formatted value (what Copy writes).
    pub text: String,
}

const FIRST: [&str; 8] = [
    "first.x",
    "first.y",
    "first.z",
    "first.length",
    "first.area",
    "first.volume",
    "first.diameter",
    "first.radius",
];
const SECOND: [&str; 8] = [
    "second.x",
    "second.y",
    "second.z",
    "second.length",
    "second.area",
    "second.volume",
    "second.diameter",
    "second.radius",
];
const LABELS: [&str; 8] = [
    "X", "Y", "Z", "Length", "Area", "Volume", "Diameter", "Radius",
];

/// Rows in a stable order (tested): distance, dx, dy, dz, angle, center distance, then the
/// first entity's rows, then the second's.
pub fn report_rows(r: &MeasureReport, prefs: &DisplayPrefs) -> Vec<MeasureRow> {
    let len_unit = Unit::from(prefs.length);
    let row = |key: &'static str, label: String, value: Quantity| {
        let unit = if value.dim == Dimension::ANGLE {
            prefs.angle
        } else if value.dim == Dimension::AREA {
            area_unit(prefs.length)
        } else if value.dim == Dimension::VOLUME {
            volume_unit(prefs.length)
        } else {
            len_unit
        };
        let spec = FormatSpec {
            precision: prefs.precision.min(6),
            ..FormatSpec::for_unit(unit)
        };
        let text = format_quantity(value, unit, &spec).unwrap_or_else(|_| "—".to_owned());
        MeasureRow {
            key,
            label,
            value,
            text,
        }
    };
    let len = |v: f64| Quantity::new(v, Dimension::LENGTH);
    let mut rows = Vec::new();
    if let Some(p) = &r.pair {
        let d = p.min_distance.delta();
        rows.push(row(
            "distance",
            "Minimum distance".into(),
            len(p.min_distance.distance.to_mm()),
        ));
        rows.push(row("dx", "ΔX".into(), len(d.x)));
        rows.push(row("dy", "ΔY".into(), len(d.y)));
        rows.push(row("dz", "ΔZ".into(), len(d.z)));
        if let Some(a) = &p.angle {
            rows.push(row("angle", "Angle".into(), a.angle.into()));
        }
        rows.push(row(
            "center_distance",
            "Center distance".into(),
            len(p.center_distance.to_mm()),
        ));
    }
    let entity =
        |e: &EntityMeasure, keys: &[&'static str; 8], n: u8, rows: &mut Vec<MeasureRow>| {
            let label = |i: usize| {
                if r.second.is_some() {
                    format!("{} {n}", LABELS[i])
                } else {
                    LABELS[i].to_owned()
                }
            };
            if let Some(p) = e.position {
                for (i, v) in [p.x, p.y, p.z].into_iter().enumerate() {
                    rows.push(row(keys[i], label(i), len(v)));
                }
            }
            if let Some(l) = e.length {
                rows.push(row(keys[3], label(3), l.into()));
            }
            if let Some(a) = e.area {
                rows.push(row(keys[4], label(4), a.into()));
            }
            if let Some(v) = e.volume {
                rows.push(row(keys[5], label(5), v.into()));
            }
            let radius = e
                .circle
                .map(|c| c.circle.radius)
                .or(e.radius.map(|r| r.to_mm()));
            if let Some(rad) = radius {
                rows.push(row(keys[6], label(6), len(2.0 * rad)));
                rows.push(row(keys[7], label(7), len(rad)));
            }
        };
    entity(&r.first, &FIRST, 1, &mut rows);
    if let Some(s) = &r.second {
        entity(s, &SECOND, 2, &mut rows);
    }
    rows
}
