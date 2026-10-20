//! Typed quantities: `#[repr(transparent)]` newtypes over the canonical `f64`.
//!
//! Each type has `ZERO`, `from_canonical`/`get` (canonical raw value), a named constructor per
//! unit (`Length::inches(0.25)`) with a matching `to_*` accessor (`l.to_inches()`), and
//! `Add`/`Sub`/`Neg`/`Mul<f64>`/`Div<f64>`/`Div<Self> -> f64`. They convert to [`Quantity`]
//! with `From` and back with `TryFrom` (`Err(DimensionMismatch)`), and serialise as the bare
//! canonical `f64`.

use super::{Dimension, LengthUnit, Quantity, Unit};
use crate::error::Error;

macro_rules! typed_quantity {
    (
        $(#[$m:meta])*
        $name:ident, $dim:expr, $canon:literal;
        $( $(#[$cm:meta])* $ctor:ident / $acc:ident => $unit:ident ),+ $(,)?
    ) => {
        $(#[$m])*
        #[derive(
            Clone, Copy, Debug, PartialEq, PartialOrd, Default, serde::Serialize,
            serde::Deserialize,
        )]
        #[repr(transparent)]
        #[serde(transparent)]
        pub struct $name(f64);

        impl $name {
            /// Zero.
            pub const ZERO: $name = $name(0.0);
            #[doc = concat!("Dimension of every `", stringify!($name), "`.")]
            pub const DIMENSION: Dimension = $dim;

            #[doc = concat!("From the canonical value (", $canon, ").")]
            pub const fn from_canonical(v: f64) -> $name {
                $name(v)
            }

            #[doc = concat!("The canonical value (", $canon, ").")]
            pub const fn get(self) -> f64 {
                self.0
            }

            /// The value in `u` (`Err(DimensionMismatch)` for a unit of another dimension).
            pub fn in_unit(self, u: Unit) -> Result<f64, Error> {
                Quantity::from(self).in_unit(u)
            }

            $(
                $(#[$cm])*
                #[doc = concat!("From a value in `", stringify!($unit), "`.")]
                pub fn $ctor(v: f64) -> $name {
                    $name(Unit::$unit.to_canonical(v))
                }

                #[doc = concat!("The value in `", stringify!($unit), "`.")]
                pub fn $acc(self) -> f64 {
                    Unit::$unit.from_canonical(self.0)
                }
            )+
        }

        impl std::ops::Add for $name {
            type Output = $name;
            fn add(self, o: $name) -> $name {
                $name(self.0 + o.0)
            }
        }
        impl std::ops::Sub for $name {
            type Output = $name;
            fn sub(self, o: $name) -> $name {
                $name(self.0 - o.0)
            }
        }
        impl std::ops::AddAssign for $name {
            fn add_assign(&mut self, o: $name) {
                self.0 += o.0;
            }
        }
        impl std::ops::SubAssign for $name {
            fn sub_assign(&mut self, o: $name) {
                self.0 -= o.0;
            }
        }
        impl std::ops::Neg for $name {
            type Output = $name;
            fn neg(self) -> $name {
                $name(-self.0)
            }
        }
        impl std::ops::Mul<f64> for $name {
            type Output = $name;
            fn mul(self, k: f64) -> $name {
                $name(self.0 * k)
            }
        }
        impl std::ops::Mul<$name> for f64 {
            type Output = $name;
            fn mul(self, q: $name) -> $name {
                $name(self * q.0)
            }
        }
        impl std::ops::Div<f64> for $name {
            type Output = $name;
            fn div(self, k: f64) -> $name {
                $name(self.0 / k)
            }
        }
        impl std::ops::Div for $name {
            type Output = f64;
            fn div(self, o: $name) -> f64 {
                self.0 / o.0
            }
        }
        impl From<$name> for Quantity {
            fn from(q: $name) -> Quantity {
                Quantity::new(q.0, $dim)
            }
        }
        impl TryFrom<Quantity> for $name {
            type Error = Error;
            fn try_from(q: Quantity) -> Result<$name, Error> {
                q.expect_dim($dim).map($name)
            }
        }
    };
}

typed_quantity! {
    /// A length (canonical mm). Geometry stays raw `f64`; this type is for API boundaries.
    Length, Dimension::LENGTH, "mm";
    um / to_um => Micrometre,
    cm / to_cm => Centimetre,
    m / to_m => Metre,
    inches / to_inches => Inch,
    feet / to_feet => Foot,
}

impl Length {
    /// From millimetres (canonical).
    pub const fn mm(v: f64) -> Length {
        Length(v)
    }
    /// The value in millimetres (same as [`get`](Length::get)).
    pub const fn to_mm(self) -> f64 {
        self.0
    }
    /// The value in a document length unit.
    pub fn to_unit(self, u: LengthUnit) -> f64 {
        Unit::from(u).from_canonical(self.0)
    }
}

typed_quantity! {
    /// A plane angle (canonical rad).
    Angle, Dimension::ANGLE, "rad";
    deg / to_deg => Degree,
}

impl Angle {
    /// From radians (canonical).
    pub const fn rad(v: f64) -> Angle {
        Angle(v)
    }
    /// The value in radians (same as [`get`](Angle::get)).
    pub const fn to_rad(self) -> f64 {
        self.0
    }
    /// Wrapped into `[0, 2π)`.
    pub fn normalized_0_2pi(self) -> Angle {
        let r = self.0.rem_euclid(std::f64::consts::TAU);
        // rem_euclid may round up to exactly 2π for tiny negative inputs.
        Angle(if r >= std::f64::consts::TAU { 0.0 } else { r })
    }
    /// Wrapped into `(-π, π]` (the `atan2` range).
    pub fn normalized_pm_pi(self) -> Angle {
        let r = self.normalized_0_2pi().0;
        Angle(if r > std::f64::consts::PI {
            r - std::f64::consts::TAU
        } else {
            r
        })
    }
}

typed_quantity! {
    /// An area (canonical mm²).
    Area, Dimension::AREA, "mm²";
    mm2 / to_mm2 => SquareMillimetre,
    cm2 / to_cm2 => SquareCentimetre,
    m2 / to_m2 => SquareMetre,
    in2 / to_in2 => SquareInch,
}

typed_quantity! {
    /// A volume (canonical mm³).
    Volume, Dimension::VOLUME, "mm³";
    mm3 / to_mm3 => CubicMillimetre,
    cm3 / to_cm3 => CubicCentimetre,
    litres / to_litres => Litre,
    ml / to_ml => Millilitre,
    m3 / to_m3 => CubicMetre,
    in3 / to_in3 => CubicInch,
}

typed_quantity! {
    /// A mass (canonical t, tonne). Use [`Mass::kg`]/[`Mass::to_kg`] for SI kilograms.
    Mass, Dimension::MASS, "t";
    g / to_g => Gram,
    kg / to_kg => Kilogram,
    tonnes / to_tonnes => Tonne,
    lb / to_lb => Pound,
}

typed_quantity! {
    /// A duration (canonical s).
    Time, Dimension::TIME, "s";
    ms / to_ms => Millisecond,
    s / to_s => Second,
    min / to_min => Minute,
    h / to_h => Hour,
}

typed_quantity! {
    /// A force (canonical N).
    Force, Dimension::FORCE, "N";
    newtons / to_newtons => Newton,
    kn / to_kn => Kilonewton,
    lbf / to_lbf => PoundForce,
}

typed_quantity! {
    /// A pressure or stress (canonical MPa = N/mm²).
    Pressure, Dimension::PRESSURE, "MPa";
    pa / to_pa => Pascal,
    kpa / to_kpa => Kilopascal,
    mpa / to_mpa => Megapascal,
    gpa / to_gpa => Gigapascal,
    bar / to_bar => Bar,
    psi / to_psi => Psi,
}

typed_quantity! {
    /// A density (canonical t/mm³; steel 7850 kg/m³ is 7.85e-9).
    Density, Dimension::DENSITY, "t/mm³";
    kg_per_m3 / to_kg_per_m3 => KilogramPerCubicMetre,
    g_per_cm3 / to_g_per_cm3 => GramPerCubicCentimetre,
}

typed_quantity! {
    /// An energy (canonical mJ = N·mm).
    Energy, Dimension::ENERGY, "mJ";
    mj / to_mj => Millijoule,
    j / to_j => Joule,
}

typed_quantity! {
    /// An absolute temperature (canonical K). Arithmetic treats values as absolute kelvin;
    /// enter differences in K.
    Temperature, Dimension::TEMPERATURE, "K";
    kelvin / to_kelvin => Kelvin,
    celsius / to_celsius => DegreeCelsius,
    fahrenheit / to_fahrenheit => DegreeFahrenheit,
}

macro_rules! cross_op {
    ($a:ident $op:ident $f:ident $b:ident = $out:ident, $sym:tt) => {
        impl std::ops::$op<$b> for $a {
            type Output = $out;
            fn $f(self, o: $b) -> $out {
                $out(self.0 $sym o.0)
            }
        }
    };
}

cross_op!(Length Mul mul Length = Area, *);
cross_op!(Area Mul mul Length = Volume, *);
cross_op!(Length Mul mul Area = Volume, *);
cross_op!(Area Div div Length = Length, /);
cross_op!(Volume Div div Length = Area, /);
cross_op!(Volume Div div Area = Length, /);
cross_op!(Force Div div Area = Pressure, /);
cross_op!(Pressure Mul mul Area = Force, *);
cross_op!(Mass Div div Volume = Density, /);
cross_op!(Density Mul mul Volume = Mass, *);
cross_op!(Force Mul mul Length = Energy, *);
cross_op!(Energy Div div Length = Force, /);
