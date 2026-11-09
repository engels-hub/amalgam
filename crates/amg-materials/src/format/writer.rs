//! TOML library writer (note §4 writer rules): materials sorted by id, keys in schema order,
//! quantities in the property's library unit, provenance tables only where non-default.
//! Output is a fixed point: load → save → load → save is byte-identical.

use std::fmt::Write as _;

use amg_core::units::{Dimension, Quantity, Unit};

use crate::appearance::Appearance;
use crate::elastic::Elastic;
use crate::library::{FORMAT_VERSION, Library};
use crate::material::{
    AmInfo, Directional, Hardness, Material, Mechanical, Plasticity, Strength, TempRange, Thermal,
};
use crate::provenance::Source;

/// A TOML basic or literal string.
fn quote(s: &str) -> String {
    toml::Value::String(s.to_owned()).to_string()
}

/// `v` (canonical) written in `unit`, shortest text that parses back to the identical `f64`.
fn qty(v: f64, dim: Dimension, unit: Unit) -> String {
    amg_core::units::format_quantity(
        Quantity::new(v, dim),
        unit,
        &amg_core::units::FormatSpec::EXACT,
    )
    .unwrap_or_else(|_| format!("{v:?}"))
}

/// A dimensionless value as shortest round-trip text.
fn plain(v: f64) -> String {
    format!("{v:?}")
}

fn f32_text(v: f32) -> String {
    format!("{v:?}")
}

pub(crate) fn write_library(lib: &Library) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "format = \"amalgam-materials\"");
    let _ = writeln!(out, "format_version = {FORMAT_VERSION}\n");
    let _ = writeln!(out, "[library]");
    let m = &lib.meta;
    let _ = writeln!(out, "id = {}", quote(&m.id));
    let _ = writeln!(out, "name = {}", quote(&m.name));
    let _ = writeln!(out, "version = {}", quote(&m.version));
    opt(&mut out, "data_licence", m.data_licence.as_deref());
    opt(&mut out, "description", m.description.as_deref());
    for s in lib.sources.values() {
        source(&mut out, s);
    }
    for a in lib.appearances.values() {
        appearance(&mut out, a);
    }
    for mat in lib.materials.values() {
        material(&mut out, mat);
    }
    out
}

fn opt(out: &mut String, key: &str, v: Option<&str>) {
    if let Some(v) = v {
        let _ = writeln!(out, "{key} = {}", quote(v));
    }
}

fn source(out: &mut String, s: &Source) {
    let _ = writeln!(out, "\n[[source]]");
    let _ = writeln!(out, "key = {}", quote(&s.key.0));
    let _ = writeln!(out, "kind = {}", quote(s.kind.as_str()));
    let _ = writeln!(out, "title = {}", quote(&s.title));
    opt(out, "publisher", s.publisher.as_deref());
    opt(out, "version", s.version.as_deref());
    opt(out, "date", s.date.as_deref());
    opt(out, "url", s.url.as_deref());
    opt(out, "accessed", s.accessed.as_deref());
    opt(out, "licence", s.licence.as_deref());
}

fn appearance(out: &mut String, a: &Appearance) {
    let _ = writeln!(out, "\n[[appearance]]");
    let _ = writeln!(out, "id = {}", quote(a.id.local()));
    let _ = writeln!(out, "revision = {}", a.revision);
    let _ = writeln!(out, "name = {}", quote(&a.name));
    let _ = writeln!(out, "base_color = {}", quote(&a.base_color.to_hex()));
    let _ = writeln!(out, "metallic = {}", f32_text(a.metallic));
    let _ = writeln!(out, "roughness = {}", f32_text(a.roughness));
}

struct W<'a> {
    m: &'a Material,
    out: &'a mut String,
}

fn material(out: &mut String, m: &Material) {
    let _ = writeln!(out, "\n[[material]]");
    let mut w = W { m, out };
    w.top();
    if let Some(mech) = &m.mechanical {
        w.mechanical(mech);
    }
    if let Some(t) = &m.thermal {
        w.thermal(t);
    }
    if let Some(am) = &m.am {
        w.am(am);
    }
}

impl W<'_> {
    fn line(&mut self, key: &str, rhs: &str) {
        let _ = writeln!(self.out, "{key} = {rhs}");
    }

    /// The `, src = .., q = .., ...` tail for a property with non-default provenance.
    fn prov_tail(&self, prop: &str) -> String {
        let mut s = String::new();
        let Some(p) = self.m.provenance.values.get(prop) else {
            return s;
        };
        if let Some(k) = &p.source {
            let _ = write!(s, ", src = {}", quote(&k.0));
        }
        if p.quality != crate::provenance::Quality::Measured {
            let _ = write!(s, ", q = {}", quote(p.quality.as_str()));
        }
        if let Some(sp) = p.specimen {
            let _ = write!(s, ", spec = {}", quote(sp.as_str()));
        }
        if let Some(st) = &p.standard {
            let _ = write!(s, ", std = {}", quote(st));
        }
        if let Some(n) = &p.note {
            let _ = write!(s, ", note = {}", quote(n));
        }
        s
    }

    /// `text` as a plain string, or an inline table when the property has its own provenance.
    fn val(&self, prop: &str, text: &str) -> String {
        let tail = self.prov_tail(prop);
        if tail.is_empty() {
            quote(text)
        } else {
            format!("{{ v = {}{tail} }}", quote(text))
        }
    }

    fn q(&self, prop: &str, v: f64, dim: Dimension, unit: Unit) -> String {
        self.val(prop, &qty(v, dim, unit))
    }

    fn press(&self, prop: &str, v: f64) -> String {
        self.q(prop, v, Dimension::PRESSURE, Unit::Megapascal)
    }

    fn temp(&self, prop: &str, v: f64) -> String {
        self.q(prop, v, Dimension::TEMPERATURE, Unit::DegreeCelsius)
    }

    fn frac(&self, prop: &str, v: f64) -> String {
        self.val(prop, &plain(v))
    }

    /// `key.xy = ..` and `key.z = ..` lines.
    fn directional(
        &mut self,
        key: &str,
        prop: &str,
        d: Directional<f64>,
        text: impl Fn(&Self, &str, f64) -> String,
    ) {
        let xy = text(self, &format!("{prop}.xy"), d.xy);
        self.line(&format!("{key}.xy"), &xy);
        if let Some(z) = d.z {
            let z = text(self, &format!("{prop}.z"), z);
            self.line(&format!("{key}.z"), &z);
        }
    }

    fn top(&mut self) {
        let m = self.m;
        self.line("id", &quote(m.id.local()));
        self.line("revision", &m.revision.to_string());
        self.line("name", &quote(&m.name));
        self.line("category", &quote(m.category.as_str()));
        if let Some(f) = &m.family {
            self.line("family", &quote(&f.as_str()));
        }
        if let Some(c) = &m.condition {
            self.line("condition", &quote(c));
        }
        if let Some(d) = &m.description {
            self.line("description", &quote(d));
        }
        if let Some(s) = &m.provenance.default_source {
            self.line("default_source", &quote(&s.0));
        }
        if let Some(a) = &m.appearance {
            self.line("appearance", &quote(a.as_str()));
        }
        if !m.tags.is_empty() {
            let tags: Vec<String> = m.tags.iter().map(|t| quote(t)).collect();
            self.line("tags", &format!("[{}]", tags.join(", ")));
        }
        let d = self.q(
            "density",
            m.density.get(),
            Dimension::DENSITY,
            Unit::GramPerCubicCentimetre,
        );
        self.line("density", &d);
    }

    fn mechanical(&mut self, m: &Mechanical) {
        let scalar = m.elongation_at_break.is_some() || m.hardness.is_some();
        if scalar {
            let _ = writeln!(self.out, "\n[material.mechanical]");
            if let Some(e) = m.elongation_at_break {
                self.directional(
                    "elongation_at_break",
                    "mechanical.elongation_at_break",
                    e,
                    Self::frac,
                );
            }
            if let Some(h) = m.hardness {
                let (scale, v) = match h {
                    Hardness::ShoreA(v) => ("shore_a", v),
                    Hardness::ShoreD(v) => ("shore_d", v),
                    Hardness::Vickers(v) => ("vickers", v),
                    Hardness::Brinell(v) => ("brinell", v),
                };
                let rhs = self.hardness(scale, v);
                self.line("hardness", &rhs);
            }
        }
        let _ = writeln!(self.out, "\n[material.mechanical.elastic]");
        self.elastic(&m.elastic);
        self.strength(&m.strength);
        if let Some(p) = &m.plasticity {
            self.plasticity(p);
        }
    }

    fn hardness(&self, scale: &str, v: f64) -> String {
        let tail = self.prov_tail("mechanical.hardness");
        format!("{{ scale = {}, v = {}{tail} }}", quote(scale), plain(v))
    }

    fn elastic(&mut self, e: &Elastic) {
        const P: &str = "mechanical.elastic";
        match e {
            Elastic::Isotropic { youngs, poisson } => {
                self.line("model", &quote("isotropic"));
                let y = self.press(&format!("{P}.youngs"), youngs.get());
                self.line("youngs", &y);
                let n = self.frac(&format!("{P}.poisson"), *poisson);
                self.line("poisson", &n);
            }
            Elastic::TransverselyIsotropic {
                e_p,
                e_t,
                nu_p,
                nu_pt,
                g_pt,
            } => {
                self.line("model", &quote("transversely_isotropic"));
                for (k, v) in [("e_p", e_p.get()), ("e_t", e_t.get())] {
                    let t = self.press(&format!("{P}.{k}"), v);
                    self.line(k, &t);
                }
                for (k, v) in [("nu_p", *nu_p), ("nu_pt", *nu_pt)] {
                    let t = self.frac(&format!("{P}.{k}"), v);
                    self.line(k, &t);
                }
                let g = self.press(&format!("{P}.g_pt"), g_pt.get());
                self.line("g_pt", &g);
            }
            Elastic::Orthotropic { e, nu, g } => {
                self.line("model", &quote("orthotropic"));
                let items = |w: &Self, key: &str, suf: [&str; 3], v: [f64; 3], pressure: bool| {
                    let parts: Vec<String> = (0..3)
                        .map(|i| {
                            let prop = format!("{P}.{key}.{}", suf[i]);
                            if pressure {
                                w.press(&prop, v[i])
                            } else {
                                w.frac(&prop, v[i])
                            }
                        })
                        .collect();
                    format!("[{}]", parts.join(", "))
                };
                let es = items(self, "e", ["1", "2", "3"], e.map(|x| x.get()), true);
                self.line("e", &es);
                let ns = items(self, "nu", ["12", "13", "23"], *nu, false);
                self.line("nu", &ns);
                let gs = items(self, "g", ["12", "13", "23"], g.map(|x| x.get()), true);
                self.line("g", &gs);
            }
        }
    }

    fn strength(&mut self, s: &Strength) {
        if *s == Strength::default() {
            return;
        }
        let _ = writeln!(self.out, "\n[material.mechanical.strength]");
        for (key, d) in [
            ("yield_tensile", s.yield_tensile),
            ("ultimate_tensile", s.ultimate_tensile),
            ("ultimate_compressive", s.ultimate_compressive),
        ] {
            if let Some(d) = d {
                let prop = format!("mechanical.strength.{key}");
                self.directional(key, &prop, d.map(|p| p.get()), Self::press);
            }
        }
        for (key, v) in [("shear", s.shear), ("flexural", s.flexural)] {
            if let Some(v) = v {
                let t = self.press(&format!("mechanical.strength.{key}"), v.get());
                self.line(key, &t);
            }
        }
    }

    fn plasticity(&mut self, p: &Plasticity) {
        let _ = writeln!(self.out, "\n[material.mechanical.plasticity]");
        const P: &str = "mechanical.plasticity";
        match p {
            Plasticity::Bilinear {
                yield_stress,
                tangent_modulus,
            } => {
                self.line("model", &quote("bilinear"));
                let y = self.press(&format!("{P}.yield_stress"), yield_stress.get());
                self.line("yield_stress", &y);
                let t = self.press(&format!("{P}.tangent_modulus"), tangent_modulus.get());
                self.line("tangent_modulus", &t);
            }
            Plasticity::Table { points } => {
                self.line("model", &quote("table"));
                let pts: Vec<String> = points
                    .iter()
                    .map(|(e, s)| {
                        format!(
                            "{{ strain = {}, stress = {} }}",
                            plain(*e),
                            quote(&qty(s.get(), Dimension::PRESSURE, Unit::Megapascal))
                        )
                    })
                    .collect();
                self.line("points", &format!("[{}]", pts.join(", ")));
            }
        }
    }

    fn thermal(&mut self, t: &Thermal) {
        let _ = writeln!(self.out, "\n[material.thermal]");
        if let Some(c) = t.conductivity {
            self.directional(
                "conductivity",
                "thermal.conductivity",
                c.map(|v| v.get()),
                |w, p, v| {
                    w.q(
                        p,
                        v,
                        Dimension::THERMAL_CONDUCTIVITY,
                        Unit::WattPerMetreKelvin,
                    )
                },
            );
        }
        if let Some(c) = t.specific_heat {
            let s = self.q(
                "thermal.specific_heat",
                c.get(),
                Dimension::SPECIFIC_HEAT,
                Unit::JoulePerKilogramKelvin,
            );
            self.line("specific_heat", &s);
        }
        if let Some(a) = t.expansion {
            self.directional(
                "expansion",
                "thermal.expansion",
                a.map(|v| v.get()),
                |w, p, v| w.q(p, v, Dimension::THERMAL_EXPANSION, Unit::PpmPerKelvin),
            );
        }
        for (key, v) in [
            ("expansion_reference", t.expansion_reference),
            ("glass_transition", t.glass_transition),
            ("melting", t.melting),
            ("heat_deflection_045", t.heat_deflection_045),
            ("max_service", t.max_service),
        ] {
            if let Some(v) = v {
                let s = self.temp(&format!("thermal.{key}"), v.get());
                self.line(key, &s);
            }
        }
    }

    fn range(&mut self, key: &str, r: TempRange) {
        let min = self.temp(&format!("am.{key}.min"), r.min.get());
        let max = self.temp(&format!("am.{key}.max"), r.max.get());
        self.line(key, &format!("{{ min = {min}, max = {max} }}"));
    }

    fn am(&mut self, am: &AmInfo) {
        let _ = writeln!(self.out, "\n[material.am]");
        self.line("process", &quote(am.process.as_str()));
        if let Some(s) = &am.slicer_type {
            self.line("slicer_type", &quote(s));
        }
        for (key, r) in [
            ("nozzle_temp", am.nozzle_temp),
            ("bed_temp", am.bed_temp),
            ("chamber_temp", am.chamber_temp),
        ] {
            if let Some(r) = r {
                self.range(key, r);
            }
        }
        if let Some(s) = am.shrinkage {
            self.directional("shrinkage", "am.shrinkage", s, Self::frac);
        }
        if let Some(c) = &am.cost {
            self.line(
                "cost",
                &format!(
                    "{{ per_kg = {}, currency = {} }}",
                    plain(c.per_kg),
                    quote(&c.currency)
                ),
            );
        }
    }
}
