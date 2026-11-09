//! The physical material model (S17 Interfaces, note §2).

use amg_core::units::{
    Density, Mass, Pressure, SpecificHeat, Temperature, ThermalConductivity, ThermalExpansion,
    Volume,
};

use crate::elastic::Elastic;
use crate::error::MaterialError;
use crate::id::{AppearanceId, MaterialId};
use crate::provenance::{Provenance, ValueProv};

/// Broad material class.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Category {
    /// Unfilled thermoplastic.
    Polymer,
    /// Fibre- or particle-filled polymer.
    PolymerComposite,
    /// Rubber-like (TPU).
    Elastomer,
    /// Cured resin.
    Photopolymer,
    /// Metal or alloy.
    Metal,
    /// Ceramic.
    Ceramic,
    /// Anything else.
    Other,
}

impl Category {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Category::Polymer => "polymer",
            Category::PolymerComposite => "polymer_composite",
            Category::Elastomer => "elastomer",
            Category::Photopolymer => "photopolymer",
            Category::Metal => "metal",
            Category::Ceramic => "ceramic",
            Category::Other => "other",
        }
    }

    pub(crate) fn from_str(s: &str) -> Option<Category> {
        Some(match s {
            "polymer" => Category::Polymer,
            "polymer_composite" => Category::PolymerComposite,
            "elastomer" => Category::Elastomer,
            "photopolymer" => Category::Photopolymer,
            "metal" => Category::Metal,
            "ceramic" => Category::Ceramic,
            "other" => Category::Other,
            _ => return None,
        })
    }
}

macro_rules! families {
    ($($variant:ident => $name:literal),+ $(,)?) => {
        /// Material family, used for search and import name matching. Serialized as a string
        /// (`"pla"`, `"pa_cf"`, `"ti6al4v"`, `"other:my-alloy"`).
        #[derive(Clone, Debug, PartialEq, Eq, Hash)]
        #[non_exhaustive]
        pub enum MaterialFamily {
            $(#[doc = $name] $variant,)+
            /// Any other family.
            Other(String),
        }

        impl MaterialFamily {
            /// The serialized name.
            pub fn as_str(&self) -> std::borrow::Cow<'_, str> {
                match self {
                    $(MaterialFamily::$variant => $name.into(),)+
                    MaterialFamily::Other(s) => format!("other:{s}").into(),
                }
            }

            /// Inverse of [`as_str`](MaterialFamily::as_str); `None` for an unknown name.
            pub fn parse(s: &str) -> Option<MaterialFamily> {
                Some(match s {
                    $($name => MaterialFamily::$variant,)+
                    _ => MaterialFamily::Other(s.strip_prefix("other:")?.to_owned()),
                })
            }
        }
    };
}

families!(
    Pla => "pla", Petg => "petg", Pctg => "pctg", Abs => "abs", Asa => "asa", Pc => "pc",
    Pa6 => "pa6", Pa11 => "pa11", Pa12 => "pa12", PaCf => "pa_cf", PaGf => "pa_gf",
    PlaCf => "pla_cf", PetgCf => "petg_cf", Tpu => "tpu",
    ResinStandard => "resin_standard", ResinTough => "resin_tough",
    ResinFlexible => "resin_flexible", ResinHighTemp => "resin_high_temp",
    Ti6Al4V => "ti6al4v", AlSi10Mg => "alsi10mg", Ss316L => "ss316l", In718 => "in718",
    MaragingSteel => "maraging_steel", CarbonSteel => "carbon_steel",
    StainlessSteel => "stainless_steel", AluminiumAlloy => "aluminium_alloy",
    TitaniumAlloy => "titanium_alloy",
);

impl serde::Serialize for MaterialFamily {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&self.as_str())
    }
}

impl<'de> serde::Deserialize<'de> for MaterialFamily {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = String::deserialize(d)?;
        MaterialFamily::parse(&s)
            .ok_or_else(|| serde::de::Error::custom(format!("unknown family `{s}`")))
    }
}

/// `xy` = in-layer value (axes 1-2); `z` = along axis 3. `z == None` means isotropic.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Directional<T> {
    /// In-layer value.
    pub xy: T,
    /// Inter-layer value, if known.
    #[serde(default)]
    pub z: Option<T>,
}

impl<T: Copy + PartialOrd> Directional<T> {
    /// An isotropic value (no `z`).
    pub fn iso(v: T) -> Self {
        Directional { xy: v, z: None }
    }

    /// `z` if present, else `xy`.
    pub fn z_or_xy(&self) -> T {
        self.z.unwrap_or(self.xy)
    }

    /// The smaller of `xy` and `z`.
    pub fn min(&self) -> T {
        match self.z {
            Some(z) if z < self.xy => z,
            _ => self.xy,
        }
    }
}

impl<T> Directional<T> {
    pub(crate) fn map<U>(self, f: impl Fn(T) -> U) -> Directional<U> {
        Directional {
            xy: f(self.xy),
            z: self.z.map(f),
        }
    }
}

/// Mechanical properties.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Mechanical {
    /// Elastic model.
    pub elastic: Elastic,
    /// Strength values.
    #[serde(default)]
    pub strength: Strength,
    /// Elongation at break as a fraction (0.12 = 12 %).
    #[serde(default)]
    pub elongation_at_break: Option<Directional<f64>>,
    /// Hardness.
    #[serde(default)]
    pub hardness: Option<Hardness>,
    /// Plasticity model.
    #[serde(default)]
    pub plasticity: Option<Plasticity>,
}

/// Strength values (typical values, not design allowables).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Strength {
    /// Yield strength (`Rp0.2` for metals).
    #[serde(default)]
    pub yield_tensile: Option<Directional<Pressure>>,
    /// Ultimate tensile strength.
    #[serde(default)]
    pub ultimate_tensile: Option<Directional<Pressure>>,
    /// Ultimate compressive strength.
    #[serde(default)]
    pub ultimate_compressive: Option<Directional<Pressure>>,
    /// Shear strength.
    #[serde(default)]
    pub shear: Option<Pressure>,
    /// Flexural strength.
    #[serde(default)]
    pub flexural: Option<Pressure>,
}

/// Hardness with its scale.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "scale", content = "value", rename_all = "snake_case")]
pub enum Hardness {
    /// Shore A, 0..=100.
    ShoreA(f64),
    /// Shore D, 0..=100.
    ShoreD(f64),
    /// Vickers (HV).
    Vickers(f64),
    /// Brinell (HB).
    Brinell(f64),
}

/// Plasticity model for nonlinear analysis.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "model", rename_all = "snake_case")]
pub enum Plasticity {
    /// Elastic-plastic with linear hardening.
    Bilinear {
        /// Initial yield stress.
        yield_stress: Pressure,
        /// Tangent modulus after yield.
        tangent_modulus: Pressure,
    },
    /// `(true plastic strain, true stress)`; strain strictly increasing and starting at 0.
    Table {
        /// The curve points.
        points: Vec<(f64, Pressure)>,
    },
}

/// Thermal properties (all optional).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Thermal {
    /// Thermal conductivity.
    #[serde(default)]
    pub conductivity: Option<Directional<ThermalConductivity>>,
    /// Specific heat capacity.
    #[serde(default)]
    pub specific_heat: Option<SpecificHeat>,
    /// Linear thermal expansion coefficient.
    #[serde(default)]
    pub expansion: Option<Directional<ThermalExpansion>>,
    /// Reference temperature of `expansion`.
    #[serde(default)]
    pub expansion_reference: Option<Temperature>,
    /// Glass transition temperature.
    #[serde(default)]
    pub glass_transition: Option<Temperature>,
    /// Melting point (solidus for alloys).
    #[serde(default)]
    pub melting: Option<Temperature>,
    /// Heat deflection temperature, ISO 75 at 0.45 MPa.
    #[serde(default)]
    pub heat_deflection_045: Option<Temperature>,
    /// Maximum service temperature.
    #[serde(default)]
    pub max_service: Option<Temperature>,
}

/// ISO/ASTM 52900 process categories.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AmProcess {
    /// FFF / FDM.
    MaterialExtrusion,
    /// SLA / MSLA / DLP.
    VatPhotopolymerization,
    /// SLS / MJF.
    PowderBedFusionPolymer,
    /// LPBF / SLM / DMLS.
    PowderBedFusionMetal,
    /// PolyJet and similar.
    MaterialJetting,
    /// Binder jetting.
    BinderJetting,
    /// DED / WAAM.
    DirectedEnergyDeposition,
}

impl AmProcess {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            AmProcess::MaterialExtrusion => "material_extrusion",
            AmProcess::VatPhotopolymerization => "vat_photopolymerization",
            AmProcess::PowderBedFusionPolymer => "powder_bed_fusion_polymer",
            AmProcess::PowderBedFusionMetal => "powder_bed_fusion_metal",
            AmProcess::MaterialJetting => "material_jetting",
            AmProcess::BinderJetting => "binder_jetting",
            AmProcess::DirectedEnergyDeposition => "directed_energy_deposition",
        }
    }

    pub(crate) fn from_str(s: &str) -> Option<AmProcess> {
        Some(match s {
            "material_extrusion" => AmProcess::MaterialExtrusion,
            "vat_photopolymerization" => AmProcess::VatPhotopolymerization,
            "powder_bed_fusion_polymer" => AmProcess::PowderBedFusionPolymer,
            "powder_bed_fusion_metal" => AmProcess::PowderBedFusionMetal,
            "material_jetting" => AmProcess::MaterialJetting,
            "binder_jetting" => AmProcess::BinderJetting,
            "directed_energy_deposition" => AmProcess::DirectedEnergyDeposition,
            _ => return None,
        })
    }
}

/// An absolute temperature range.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TempRange {
    /// Lower bound.
    pub min: Temperature,
    /// Upper bound.
    pub max: Temperature,
}

/// Cost per kilogram.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Cost {
    /// Amount per kg.
    pub per_kg: f64,
    /// ISO 4217 code, e.g. `EUR`.
    pub currency: String,
}

/// Additive-manufacturing data.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AmInfo {
    /// ISO/ASTM 52900 process.
    pub process: AmProcess,
    /// OrcaSlicer `filament_type` (`PLA`, `PETG`, `PA-CF`, ...); `None` for resin and metal.
    #[serde(default)]
    pub slicer_type: Option<String>,
    /// Nozzle temperature range.
    #[serde(default)]
    pub nozzle_temp: Option<TempRange>,
    /// Bed temperature range.
    #[serde(default)]
    pub bed_temp: Option<TempRange>,
    /// Chamber temperature range.
    #[serde(default)]
    pub chamber_temp: Option<TempRange>,
    /// Linear shrinkage fraction.
    #[serde(default)]
    pub shrinkage: Option<Directional<f64>>,
    /// Cost per kg.
    #[serde(default)]
    pub cost: Option<Cost>,
}

/// A physical material (a fully dense printed or wrought material, canonical mm–t–s–K).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Material {
    /// Id (`<library>/<local>`).
    pub id: MaterialId,
    /// Bumped on any value change; drives drift detection.
    pub revision: u32,
    /// Display name.
    pub name: String,
    /// Broad class.
    pub category: Category,
    /// Family for search and import matching.
    #[serde(default)]
    pub family: Option<MaterialFamily>,
    /// "printed, 100 % infill, dry, 23 degC", "post-cured 60 min 60 degC", ...
    #[serde(default)]
    pub condition: Option<String>,
    /// Free text.
    #[serde(default)]
    pub description: Option<String>,
    /// Density (required).
    pub density: Density,
    /// Mechanical properties.
    #[serde(default)]
    pub mechanical: Option<Mechanical>,
    /// Thermal properties.
    #[serde(default)]
    pub thermal: Option<Thermal>,
    /// AM data.
    #[serde(default)]
    pub am: Option<AmInfo>,
    /// Default appearance applied on assignment when the body has none.
    #[serde(default)]
    pub appearance: Option<AppearanceId>,
    /// Free tags for search.
    #[serde(default)]
    pub tags: Vec<String>,
    /// Sources of the values.
    #[serde(default)]
    pub provenance: Provenance,
}

/// blake3 over the compact JSON of `v`.
pub(crate) fn hash_json<T: serde::Serialize>(v: &T) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    // Serialising plain data into a hasher cannot fail (no maps with non-string keys, and the
    // hasher's `Write` never errors); a failure would only make the hash less specific.
    if serde_json::to_writer(&mut h, v).is_err() {
        h.update(b"serialize-error");
    }
    *h.finalize().as_bytes()
}

impl Material {
    /// Every validation rule of note §2.1–2.4. `property` of the error is the property path
    /// (`mechanical.elastic.poisson`); the library loader prefixes it with `material[i].`.
    pub fn validate(&self) -> Result<(), MaterialError> {
        let id = self.id.as_str();
        let bad = |property: std::fmt::Arguments<'_>, reason: String| MaterialError::Invalid {
            id: id.to_owned(),
            property: property.to_string(),
            reason,
        };
        if self.name.trim().is_empty() {
            return Err(bad(format_args!("name"), "must not be empty".into()));
        }
        if self.revision == 0 {
            return Err(bad(format_args!("revision"), "must be at least 1".into()));
        }
        pos(format_args!("density"), self.density.get(), &bad)?;
        if let Some(m) = &self.mechanical {
            if let Err((name, reason)) = m.elastic.check() {
                return Err(bad(format_args!("mechanical.elastic.{name}"), reason));
            }
            let s = &m.strength;
            for (name, d) in [
                ("yield_tensile", &s.yield_tensile),
                ("ultimate_tensile", &s.ultimate_tensile),
                ("ultimate_compressive", &s.ultimate_compressive),
            ] {
                if let Some(d) = d {
                    dir(
                        format_args!("mechanical.strength.{name}"),
                        d.map(Pressure::get),
                        &bad,
                    )?;
                }
            }
            for (name, v) in [("shear", s.shear), ("flexural", s.flexural)] {
                if let Some(v) = v {
                    pos(format_args!("mechanical.strength.{name}"), v.get(), &bad)?;
                }
            }
            if let Some(e) = &m.elongation_at_break {
                dir(format_args!("mechanical.elongation_at_break"), *e, &bad)?;
            }
            if let Some(h) = &m.hardness {
                let (v, max) = match *h {
                    Hardness::ShoreA(v) | Hardness::ShoreD(v) => (v, 100.0),
                    Hardness::Vickers(v) | Hardness::Brinell(v) => (v, 10_000.0),
                };
                if !(v > 0.0 && v <= max) {
                    return Err(bad(
                        format_args!("mechanical.hardness"),
                        format!("must be in (0, {max}]"),
                    ));
                }
            }
            if let Some(p) = &m.plasticity {
                check_plasticity(p, &bad)?;
            }
        }
        if let Some(t) = &self.thermal {
            if let Some(c) = &t.conductivity {
                dir(
                    format_args!("thermal.conductivity"),
                    c.map(ThermalConductivity::get),
                    &bad,
                )?;
            }
            if let Some(c) = t.specific_heat {
                pos(format_args!("thermal.specific_heat"), c.get(), &bad)?;
            }
            if let Some(a) = &t.expansion {
                let a = a.map(ThermalExpansion::get);
                for v in std::iter::once(a.xy).chain(a.z) {
                    if !v.is_finite() {
                        return Err(bad(
                            format_args!("thermal.expansion"),
                            "must be finite".into(),
                        ));
                    }
                }
            }
            for (name, v) in [
                ("expansion_reference", t.expansion_reference),
                ("glass_transition", t.glass_transition),
                ("melting", t.melting),
                ("heat_deflection_045", t.heat_deflection_045),
                ("max_service", t.max_service),
            ] {
                if let Some(v) = v {
                    pos(format_args!("thermal.{name}"), v.get(), &bad)?;
                }
            }
        }
        if let Some(am) = &self.am {
            for (name, r) in [
                ("nozzle_temp", am.nozzle_temp),
                ("bed_temp", am.bed_temp),
                ("chamber_temp", am.chamber_temp),
            ] {
                if let Some(r) = r {
                    pos(format_args!("am.{name}.min"), r.min.get(), &bad)?;
                    pos(format_args!("am.{name}.max"), r.max.get(), &bad)?;
                    if r.min > r.max {
                        return Err(bad(format_args!("am.{name}"), "min exceeds max".into()));
                    }
                }
            }
            if let Some(s) = &am.shrinkage {
                for v in std::iter::once(s.xy).chain(s.z) {
                    if !(v.is_finite() && v > -1.0 && v < 1.0) {
                        return Err(bad(
                            format_args!("am.shrinkage"),
                            "must be a fraction in (-1, 1)".into(),
                        ));
                    }
                }
            }
            if let Some(c) = &am.cost {
                if !(c.per_kg.is_finite() && c.per_kg >= 0.0) {
                    return Err(bad(
                        format_args!("am.cost.per_kg"),
                        "must be non-negative".into(),
                    ));
                }
                let iso4217 =
                    c.currency.len() == 3 && c.currency.bytes().all(|b| b.is_ascii_uppercase());
                if !iso4217 {
                    return Err(bad(
                        format_args!("am.cost.currency"),
                        "expected an ISO 4217 code such as EUR".into(),
                    ));
                }
            }
        }
        // Specimen rule (note §2.2): only Z** specimens may fill a `z` slot.
        for (path, p) in &self.provenance.values {
            if path.0.ends_with(".z")
                && let Some(spec) = p.specimen
                && !spec.is_z()
            {
                return Err(bad(
                    format_args!("{}", path.0),
                    format!(
                        "a `z` value needs a Z** specimen, but the specimen is {} (the \
                         \"vertical (XZ)\" datasheet pitfall)",
                        spec.as_str()
                    ),
                ));
            }
        }
        Ok(())
    }

    /// blake3 over the canonical JSON of the values: everything except `provenance`, `id` and
    /// `revision` (so re-keying into a library or bumping the revision does not change it).
    pub fn content_hash(&self) -> [u8; 32] {
        #[derive(serde::Serialize)]
        struct View<'a> {
            name: &'a str,
            category: Category,
            family: &'a Option<MaterialFamily>,
            condition: &'a Option<String>,
            description: &'a Option<String>,
            density: Density,
            mechanical: &'a Option<Mechanical>,
            thermal: &'a Option<Thermal>,
            am: &'a Option<AmInfo>,
            appearance: &'a Option<AppearanceId>,
            tags: &'a [String],
        }
        hash_json(&View {
            name: &self.name,
            category: self.category,
            family: &self.family,
            condition: &self.condition,
            description: &self.description,
            density: self.density,
            mechanical: &self.mechanical,
            thermal: &self.thermal,
            am: &self.am,
            appearance: &self.appearance,
            tags: &self.tags,
        })
    }

    /// Mass of `v` of this material.
    pub fn mass_of(&self, v: Volume) -> Mass {
        self.density * v
    }

    /// The elastic model; `MissingProperty` if `mechanical` is absent.
    pub fn elastic(&self) -> Result<&Elastic, MaterialError> {
        self.mechanical
            .as_ref()
            .map(|m| &m.elastic)
            .ok_or_else(|| MaterialError::MissingProperty {
                id: self.id.to_string(),
                property: "mechanical",
            })
    }

    /// True if the elastic model is not isotropic, or any directional value has a `z` that
    /// differs from its `xy`.
    pub fn is_anisotropic(&self) -> bool {
        fn differs<T: PartialEq>(d: &Option<Directional<T>>) -> bool {
            d.as_ref()
                .is_some_and(|d| d.z.as_ref().is_some_and(|z| *z != d.xy))
        }
        let mech = self.mechanical.as_ref().is_some_and(|m| {
            !m.elastic.is_isotropic()
                || differs(&m.strength.yield_tensile)
                || differs(&m.strength.ultimate_tensile)
                || differs(&m.strength.ultimate_compressive)
                || differs(&m.elongation_at_break)
        });
        let th = self
            .thermal
            .as_ref()
            .is_some_and(|t| differs(&t.conductivity) || differs(&t.expansion));
        mech || th
    }

    /// Provenance entry of a property path (`None` means the material's default source).
    pub fn provenance_of(&self, p: &str) -> Option<&ValueProv> {
        self.provenance.values.get(p)
    }
}

type Bad<'a> = &'a dyn Fn(std::fmt::Arguments<'_>, String) -> MaterialError;

fn pos(path: std::fmt::Arguments<'_>, v: f64, bad: Bad<'_>) -> Result<(), MaterialError> {
    if v.is_finite() && v > 0.0 {
        Ok(())
    } else {
        Err(bad(path, format!("must be positive and finite, got {v}")))
    }
}

fn dir(
    path: std::fmt::Arguments<'_>,
    d: Directional<f64>,
    bad: Bad<'_>,
) -> Result<(), MaterialError> {
    pos(format_args!("{path}.xy"), d.xy, bad)?;
    if let Some(z) = d.z {
        pos(format_args!("{path}.z"), z, bad)?;
    }
    Ok(())
}

fn check_plasticity(p: &Plasticity, bad: Bad<'_>) -> Result<(), MaterialError> {
    match p {
        Plasticity::Bilinear {
            yield_stress,
            tangent_modulus,
        } => {
            pos(
                format_args!("mechanical.plasticity.yield_stress"),
                yield_stress.get(),
                bad,
            )?;
            let t = tangent_modulus.get();
            if !(t.is_finite() && t >= 0.0) {
                return Err(bad(
                    format_args!("mechanical.plasticity.tangent_modulus"),
                    format!("must be non-negative, got {t}"),
                ));
            }
        }
        Plasticity::Table { points } => {
            let path = "mechanical.plasticity.points";
            let Some(first) = points.first() else {
                return Err(bad(
                    format_args!("{path}"),
                    "needs at least one point".into(),
                ));
            };
            if first.0.abs() > 0.0 {
                return Err(bad(
                    format_args!("{path}"),
                    format!("first plastic strain must be 0, got {}", first.0),
                ));
            }
            for (i, (strain, stress)) in points.iter().enumerate() {
                pos(format_args!("{path}[{i}].stress"), stress.get(), bad)?;
                if !strain.is_finite() || (i > 0 && *strain <= points[i - 1].0) {
                    return Err(bad(
                        format_args!("{path}[{i}].strain"),
                        "plastic strain must be strictly increasing".into(),
                    ));
                }
            }
        }
    }
    Ok(())
}
