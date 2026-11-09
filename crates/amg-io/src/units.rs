//! Units, up-axis and the unit guess (S12 §4, note §7).

use amg_core::{Mat3, Transform, Vec3, units::LengthUnit};

/// Exact factor file unit → mm applied to `v` (note §7).
///
/// Multiplications by `10`, `1000`, `25.4` and `304.8` use the literal decimal, so the result is
/// bit-equal to the literal product. µm divides by `1000.0` instead of multiplying by the
/// inexact `1e-3`, which gives the correctly rounded quotient.
pub fn scale_to_mm(v: f64, unit: LengthUnit) -> f64 {
    match unit {
        LengthUnit::Micrometre => v / 1000.0,
        LengthUnit::Millimetre => v,
        LengthUnit::Centimetre => v * 10.0,
        LengthUnit::Metre => v * 1000.0,
        LengthUnit::Inch => v * 25.4,
        LengthUnit::Foot => v * 304.8,
    }
}

/// Inverse of [`scale_to_mm`]: millimetres → `unit`.
pub fn scale_from_mm(v: f64, unit: LengthUnit) -> f64 {
    match unit {
        LengthUnit::Micrometre => v * 1000.0,
        LengthUnit::Millimetre => v,
        LengthUnit::Centimetre => v / 10.0,
        LengthUnit::Metre => v / 1000.0,
        LengthUnit::Inch => v / 25.4,
        LengthUnit::Foot => v / 304.8,
    }
}

/// Converts a length between two units through mm.
pub fn convert_length(v: f64, from: LengthUnit, to: LengthUnit) -> f64 {
    if from == to {
        v
    } else {
        scale_from_mm(scale_to_mm(v, from), to)
    }
}

/// Which model axis points up in the file.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum UpAxis {
    /// Z-up: the Amalgam and slicer convention.
    #[default]
    Z,
    /// Y-up: the usual OBJ/PLY export of DCC tools.
    Y,
}

impl UpAxis {
    /// Rotation that brings this file convention to Z-up: for `Y`, +90° about X, as an exact
    /// matrix (`(x, y, z) → (x, −z, y)`, no trigonometry); for `Z` the identity.
    pub fn to_z_up(self) -> Transform {
        match self {
            UpAxis::Z => Transform::IDENTITY,
            UpAxis::Y => Transform {
                matrix3: Mat3::from_cols(Vec3::X, Vec3::Z, Vec3::NEG_Y),
                translation: Vec3::ZERO,
            },
        }
    }
}

/// How the unit of a file is chosen.
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum UnitChoice {
    /// Use the file's declared unit; unitless files use `fallback` (default mm).
    FromFile {
        /// Unit for files that declare none.
        fallback: LengthUnit,
    },
    /// Ignore the declaration and read the numbers in this unit.
    Assume(LengthUnit),
}

impl Default for UnitChoice {
    fn default() -> Self {
        UnitChoice::FromFile {
            fallback: LengthUnit::Millimetre,
        }
    }
}

impl UnitChoice {
    /// The unit to apply for a file that declares `declared`.
    pub fn resolve(self, declared: Option<LengthUnit>) -> LengthUnit {
        match self {
            UnitChoice::Assume(u) => u,
            UnitChoice::FromFile { fallback } => declared.unwrap_or(fallback),
        }
    }
}

/// How sure [`guess_unit`] is.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Confidence {
    /// Little evidence.
    Low,
    /// Some evidence.
    Medium,
    /// Declared, or an ordinary size.
    High,
}

/// Suggestion for the import dialog. Nothing is ever applied without the user's say.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitGuess {
    /// The unit the file declares.
    pub declared: Option<LengthUnit>,
    /// Suggested unit.
    pub suggested: LengthUnit,
    /// Confidence in the suggestion.
    pub confidence: Confidence,
    /// Extents (x, y, z) in mm under each plausible interpretation, for the dialog. Always
    /// holds mm and inch, plus the declared and the suggested unit.
    pub candidates: Vec<(LengthUnit, [f64; 3])>,
}

/// Largest extent (file numbers) below which a unitless file is suggested as metres.
const METRE_BELOW: f64 = 1.0;
/// Largest extent (file numbers) above which a unitless file is suggested as micrometres.
const MICROMETRE_ABOVE: f64 = 5000.0;

/// Suggests a unit (note §7). `raw_extents` is the bounding-box size in file numbers, before
/// any scale.
///
/// - A declared unit is used, with high confidence.
/// - Unitless: largest extent `E < 1` → metres (medium), `E > 5000` → micrometres (low),
///   otherwise mm (high). An empty or non-finite extent gives mm with low confidence.
///
/// Inches cannot be told from mm by geometry; the candidates carry both readings.
pub fn guess_unit(raw_extents: [f64; 3], declared: Option<LengthUnit>) -> UnitGuess {
    let e = raw_extents.iter().copied().fold(0.0_f64, f64::max);
    let sane = raw_extents.iter().all(|v| v.is_finite() && *v >= 0.0) && e > 0.0;
    let (suggested, confidence) = match declared {
        Some(u) => (u, Confidence::High),
        None if !sane => (LengthUnit::Millimetre, Confidence::Low),
        None if e < METRE_BELOW => (LengthUnit::Metre, Confidence::Medium),
        None if e > MICROMETRE_ABOVE => (LengthUnit::Micrometre, Confidence::Low),
        None => (LengthUnit::Millimetre, Confidence::High),
    };
    let mut units = vec![LengthUnit::Millimetre, LengthUnit::Inch];
    for u in declared.into_iter().chain([suggested]) {
        if !units.contains(&u) {
            units.push(u);
        }
    }
    let candidates = units
        .into_iter()
        .map(|u| (u, raw_extents.map(|v| scale_to_mm(v, u))))
        .collect();
    UnitGuess {
        declared,
        suggested,
        confidence,
        candidates,
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "exact unit conversion is the property under test"
)]
mod tests {
    use super::*;
    use LengthUnit::*;

    #[test]
    fn scale_is_bit_equal_to_the_literal_products() {
        for v in [1.0, 0.1, 3.0, 123.456, 1e-7, 7.0e5, -2.5] {
            assert_eq!(scale_to_mm(v, Millimetre).to_bits(), v.to_bits());
            assert_eq!(scale_to_mm(v, Centimetre).to_bits(), (v * 10.0).to_bits());
            assert_eq!(scale_to_mm(v, Metre).to_bits(), (v * 1000.0).to_bits());
            assert_eq!(scale_to_mm(v, Inch).to_bits(), (v * 25.4).to_bits());
            assert_eq!(scale_to_mm(v, Foot).to_bits(), (v * 304.8).to_bits());
        }
        assert_eq!(scale_to_mm(1.0, Inch), 25.4);
        assert_eq!(scale_to_mm(1.0, Foot), 304.8);
        assert_eq!(scale_to_mm(1.0, Centimetre), 10.0);
        assert_eq!(scale_to_mm(1.0, Metre), 1000.0);
    }

    #[test]
    fn micrometre_is_correctly_rounded() {
        // The division is the correctly rounded quotient: it differs from the multiplication
        // by 1e-3 (which is not exact) for some inputs, and it is exact for multiples of 1000.
        assert_eq!(scale_to_mm(1000.0, Micrometre), 1.0);
        assert_eq!(scale_to_mm(2500.0, Micrometre), 2.5);
        let mut differs = 0;
        for i in 1..=2000 {
            let v = f64::from(i) * 3.0 + 0.7;
            let got = scale_to_mm(v, Micrometre);
            // Oracle in exact rational arithmetic: round(v / 1000) via the neighbours.
            let up = got.next_up();
            let down = got.next_down();
            let err = |c: f64| c.mul_add(1000.0, -v).abs();
            assert!(err(got) <= err(up) && err(got) <= err(down), "{v}");
            if got.to_bits() != (v * 1e-3).to_bits() {
                differs += 1;
            }
        }
        assert!(
            differs > 0,
            "the test must discriminate division from 1e-3 multiplication"
        );
    }

    #[test]
    fn convert_round_trips_through_mm() {
        for u in LengthUnit::ALL {
            assert_eq!(convert_length(5.0, u, u), 5.0);
        }
        assert_eq!(convert_length(1.0, Inch, Millimetre), 25.4);
        assert_eq!(convert_length(1.0, Millimetre, Micrometre), 1000.0);
        assert!((convert_length(convert_length(3.0, Foot, Inch), Inch, Foot) - 3.0).abs() < 1e-15);
    }

    #[test]
    fn up_axis_matrix_is_exact() {
        assert_eq!(UpAxis::Z.to_z_up(), Transform::IDENTITY);
        let t = UpAxis::Y.to_z_up();
        let p = t.transform_point3(amg_core::Point3::new(1.0, 2.0, 3.0));
        assert_eq!((p.x, p.y, p.z), (1.0, -3.0, 2.0));
        // Y (up in the file) maps to +Z; it is a proper rotation.
        assert_eq!(t.transform_vector3(Vec3::Y), Vec3::Z);
        assert_eq!(t.matrix3.determinant(), 1.0);
    }

    #[test]
    fn unit_choice_resolves() {
        let d = UnitChoice::default();
        assert_eq!(d.resolve(None), Millimetre);
        assert_eq!(d.resolve(Some(Inch)), Inch);
        assert_eq!(UnitChoice::Assume(Metre).resolve(Some(Inch)), Metre);
        assert_eq!(UnitChoice::FromFile { fallback: Foot }.resolve(None), Foot);
    }

    struct Case {
        name: &'static str,
        extents: [f64; 3],
        declared: Option<LengthUnit>,
        suggested: LengthUnit,
        confidence: Confidence,
    }

    #[test]
    fn guess_unit_table() {
        use Confidence::*;
        let c = |name, extents, declared, suggested, confidence| Case {
            name,
            extents,
            declared,
            suggested,
            confidence,
        };
        let cases = [
            c("ordinary part", [40.0, 25.0, 10.0], None, Millimetre, High),
            c(
                "tiny, metres from Blender",
                [0.2, 0.1, 0.05],
                None,
                Metre,
                Medium,
            ),
            c("just under 1", [0.999, 0.5, 0.1], None, Metre, Medium),
            c("exactly 1", [1.0, 0.5, 0.1], None, Millimetre, High),
            c("exactly 5000", [5000.0, 10.0, 10.0], None, Millimetre, High),
            c(
                "just over 5000",
                [5000.5, 10.0, 10.0],
                None,
                Micrometre,
                Low,
            ),
            c(
                "scanner in microns",
                [25_000.0, 18_000.0, 9_000.0],
                None,
                Micrometre,
                Low,
            ),
            c(
                "largest axis decides",
                [0.1, 0.1, 300.0],
                None,
                Millimetre,
                High,
            ),
            c(
                "a 4 inch cube cannot be told from 4 mm",
                [4.0, 4.0, 4.0],
                None,
                Millimetre,
                High,
            ),
            c(
                "declared inch wins over a small size",
                [0.5, 0.5, 0.5],
                Some(Inch),
                Inch,
                High,
            ),
            c(
                "declared mm wins over a huge size",
                [9e4, 1.0, 1.0],
                Some(Millimetre),
                Millimetre,
                High,
            ),
            c("declared metre", [2.0, 1.0, 1.0], Some(Metre), Metre, High),
            c("empty extents", [0.0, 0.0, 0.0], None, Millimetre, Low),
            c("NaN extents", [f64::NAN, 1.0, 1.0], None, Millimetre, Low),
            c(
                "infinite extents",
                [f64::INFINITY, 1.0, 1.0],
                None,
                Millimetre,
                Low,
            ),
            c(
                "flat sheet, one axis zero",
                [200.0, 100.0, 0.0],
                None,
                Millimetre,
                High,
            ),
        ];
        assert!(cases.len() >= 12);
        for case in &cases {
            let g = guess_unit(case.extents, case.declared);
            assert_eq!(g.suggested, case.suggested, "{}", case.name);
            assert_eq!(g.confidence, case.confidence, "{}", case.name);
            assert_eq!(g.declared, case.declared, "{}", case.name);
            // The dialog always has the mm and inch readings, and the suggestion's.
            let units: Vec<_> = g.candidates.iter().map(|c| c.0).collect();
            for u in [Millimetre, Inch, case.suggested] {
                assert!(
                    units.contains(&u),
                    "{}: {u:?} missing in {units:?}",
                    case.name
                );
            }
            let mut dedup = units.clone();
            dedup.dedup();
            assert_eq!(
                dedup.len(),
                units.len(),
                "{}: duplicate candidates",
                case.name
            );
        }
    }

    #[test]
    fn candidates_hold_the_scaled_extents() {
        let g = guess_unit([2.0, 4.0, 8.0], None);
        let inch = g.candidates.iter().find(|c| c.0 == Inch).map(|c| c.1);
        assert_eq!(inch, Some([2.0 * 25.4, 4.0 * 25.4, 8.0 * 25.4]));
        let mm = g.candidates.iter().find(|c| c.0 == Millimetre).map(|c| c.1);
        assert_eq!(mm, Some([2.0, 4.0, 8.0]));
    }
}
