//! TOML library reader (note §4).
//!
//! Error paths (`material[0].density`) are rendered lazily, only when an error is built: the
//! happy path allocates nothing for them (the 1000-material parse budget is 20 ms).

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use amg_core::units::{
    Density, Dimension, Pressure, SpecificHeat, Temperature, ThermalConductivity, ThermalExpansion,
    parse_quantity,
};

use super::doc::{self, Tab, Val};
use crate::appearance::{Appearance, Srgba8};
use crate::elastic::Elastic;
use crate::error::MaterialError;
use crate::id::{AppearanceId, MaterialId, valid_token};
use crate::library::{FORMAT_VERSION, Library, LibraryMeta};
use crate::material::{
    AmInfo, AmProcess, Category, Cost, Directional, Hardness, Material, MaterialFamily, Mechanical,
    Plasticity, Strength, TempRange, Thermal,
};
use crate::provenance::{
    PropPath, Provenance, Quality, Source, SourceKey, SourceKind, SpecimenOrientation, ValueProv,
};

/// Rewrites file contents from `from` to [`FORMAT_VERSION`]. v1 is current, so this is the
/// hook where `v1 → v2 → …` steps are added (each mutating the document tree).
fn migrate(_doc: &mut Tab<'_>, from: u32) -> Result<(), MaterialError> {
    let mut v = from;
    while v < FORMAT_VERSION {
        #[allow(
            clippy::match_single_binding,
            reason = "the migration chain grows here"
        )]
        match v {
            // 1 => { ...rewrite to version 2...; }
            _ => {}
        }
        v += 1;
    }
    Ok(())
}

/// Parses a library file. `origin` appears in errors.
pub(crate) fn read_library(src: &str, origin: &str) -> Result<Library, MaterialError> {
    let mut doc = doc::parse(src, origin)?;
    let rd = Rd { origin };
    let version = match doc.get("format_version") {
        Some(Val::Int(i)) => u32::try_from(*i)
            .ok()
            .filter(|v| *v >= 1)
            .ok_or_else(|| rd.bad("format_version", "must be a positive integer"))?,
        _ => return Err(rd.bad("format_version", "required integer")),
    };
    if version > FORMAT_VERSION {
        return Err(MaterialError::TooNew {
            origin: origin.to_owned(),
            found: version,
            supported: FORMAT_VERSION,
        });
    }
    migrate(&mut doc, version)?;
    rd.keys(
        &doc,
        &[
            "format",
            "format_version",
            "library",
            "source",
            "material",
            "appearance",
        ],
        "",
    )?;
    if let Some(f) = doc.get("format")
        && f.as_str() != Some("amalgam-materials")
    {
        return Err(rd.bad("format", "expected \"amalgam-materials\""));
    }
    let meta = rd.meta(doc.get("library"))?;
    let mut lib = Library::new(meta);
    for (i, v) in rd.array(doc.get("source"), "source")?.iter().enumerate() {
        let s = rd.source(v, &format!("source[{i}]"))?;
        if lib.sources.insert(s.key.clone(), s.clone()).is_some() {
            return Err(MaterialError::Duplicate {
                origin: origin.to_owned(),
                id: s.key.0,
            });
        }
    }
    for (i, v) in rd
        .array(doc.get("appearance"), "appearance")?
        .iter()
        .enumerate()
    {
        let a = rd.appearance(v, &format!("appearance[{i}]"), &lib.meta.id)?;
        let id = a.id.clone();
        if lib.appearances.insert(id.clone(), Arc::new(a)).is_some() {
            return Err(MaterialError::Duplicate {
                origin: origin.to_owned(),
                id: id.to_string(),
            });
        }
    }
    for (i, v) in rd
        .array(doc.get("material"), "material")?
        .iter()
        .enumerate()
    {
        let m = rd.material(v, i, &lib)?;
        let id = m.id.clone();
        if lib.materials.insert(id.clone(), Arc::new(m)).is_some() {
            return Err(MaterialError::Duplicate {
                origin: origin.to_owned(),
                id: id.to_string(),
            });
        }
    }
    Ok(lib)
}

/// A dotted property path built on the stack (`mechanical.strength.ultimate_tensile.z`).
#[derive(Clone, Copy)]
struct Path<'p> {
    parts: [&'p str; 6],
    n: usize,
}

impl<'p> Path<'p> {
    fn new(root: &'p str) -> Path<'p> {
        let mut parts = [""; 6];
        parts[0] = root;
        Path { parts, n: 1 }
    }

    fn at(self, s: &'p str) -> Path<'p> {
        let mut p = self;
        p.parts[p.n] = s;
        p.n += 1;
        p
    }

    fn join(&self) -> String {
        self.parts[..self.n].join(".")
    }
}

/// A path that is rendered only when an error needs it.
trait Render {
    fn render(&self) -> String;
}

impl Render for String {
    fn render(&self) -> String {
        self.clone()
    }
}

/// `material[i]` + a property path.
struct MatPath<'a> {
    mat: &'a str,
    prop: Path<'a>,
}

impl Render for MatPath<'_> {
    fn render(&self) -> String {
        format!("{}.{}", self.mat, self.prop.join())
    }
}

struct Rd<'a> {
    origin: &'a str,
}

impl Rd<'_> {
    fn bad(&self, path: &str, reason: &str) -> MaterialError {
        MaterialError::Invalid {
            id: self.origin.to_owned(),
            property: path.to_owned(),
            reason: reason.to_owned(),
        }
    }

    fn bad_at(&self, p: &dyn Render, key: &str, reason: &str) -> MaterialError {
        let base = p.render();
        let path = match (base.is_empty(), key.is_empty()) {
            (_, true) => base,
            (true, false) => key.to_owned(),
            (false, false) => format!("{base}.{key}"),
        };
        self.bad(&path, reason)
    }

    fn keys(&self, t: &Tab<'_>, allowed: &[&str], path: &str) -> Result<(), MaterialError> {
        self.keys_at(t, allowed, &path.to_owned())
    }

    fn keys_at(
        &self,
        t: &Tab<'_>,
        allowed: &[&str],
        path: &dyn Render,
    ) -> Result<(), MaterialError> {
        for k in t.keys() {
            if !allowed.contains(&k) {
                return Err(self.bad_at(path, k, "unknown key"));
            }
        }
        Ok(())
    }

    fn array<'v, 'a>(
        &self,
        v: Option<&'v Val<'a>>,
        path: &str,
    ) -> Result<&'v [Val<'a>], MaterialError> {
        match v {
            None => Ok(&[]),
            Some(v) => v
                .as_array()
                .ok_or_else(|| self.bad(path, "expected an array of tables")),
        }
    }

    fn table<'v, 'a>(
        &self,
        v: &'v Val<'a>,
        path: &dyn Render,
    ) -> Result<&'v Tab<'a>, MaterialError> {
        v.as_table()
            .ok_or_else(|| self.bad_at(path, "", "expected a table"))
    }

    fn opt_string(
        &self,
        t: &Tab<'_>,
        key: &str,
        path: &dyn Render,
    ) -> Result<Option<String>, MaterialError> {
        match t.get(key) {
            None => Ok(None),
            Some(Val::Str(s)) => Ok(Some(s.to_string())),
            Some(_) => Err(self.bad_at(path, key, "expected a string")),
        }
    }

    fn req_string(
        &self,
        t: &Tab<'_>,
        key: &str,
        path: &dyn Render,
    ) -> Result<String, MaterialError> {
        self.opt_string(t, key, path)?
            .ok_or_else(|| self.bad_at(path, key, "missing key"))
    }

    fn revision(&self, t: &Tab<'_>, path: &dyn Render) -> Result<u32, MaterialError> {
        match t.get("revision") {
            None => Ok(1),
            Some(Val::Int(i)) => u32::try_from(*i)
                .ok()
                .filter(|v| *v >= 1)
                .ok_or_else(|| self.bad_at(path, "revision", "must be at least 1")),
            Some(_) => Err(self.bad_at(path, "revision", "expected an integer")),
        }
    }

    fn meta(&self, v: Option<&Val<'_>>) -> Result<LibraryMeta, MaterialError> {
        let p = "library".to_owned();
        let v = v.ok_or_else(|| self.bad("library", "missing [library] table"))?;
        let t = self.table(v, &p)?;
        self.keys_at(
            t,
            &["id", "name", "version", "data_licence", "description"],
            &p,
        )?;
        let id = self.req_string(t, "id", &p)?;
        if !valid_token(&id) {
            return Err(self.bad("library.id", "expected [a-z0-9._-]+"));
        }
        Ok(LibraryMeta {
            id,
            name: self.req_string(t, "name", &p)?,
            version: self.req_string(t, "version", &p)?,
            data_licence: self.opt_string(t, "data_licence", &p)?,
            description: self.opt_string(t, "description", &p)?,
        })
    }

    fn source(&self, v: &Val<'_>, path: &str) -> Result<Source, MaterialError> {
        let p = path.to_owned();
        let t = self.table(v, &p)?;
        self.keys_at(
            t,
            &[
                "key",
                "kind",
                "title",
                "publisher",
                "version",
                "date",
                "url",
                "accessed",
                "licence",
            ],
            &p,
        )?;
        let kind = self.req_string(t, "kind", &p)?;
        Ok(Source {
            key: SourceKey(self.req_string(t, "key", &p)?),
            kind: SourceKind::from_str(&kind)
                .ok_or_else(|| self.bad(&format!("{path}.kind"), "unknown source kind"))?,
            title: self.req_string(t, "title", &p)?,
            publisher: self.opt_string(t, "publisher", &p)?,
            version: self.opt_string(t, "version", &p)?,
            date: self.opt_string(t, "date", &p)?,
            url: self.opt_string(t, "url", &p)?,
            accessed: self.opt_string(t, "accessed", &p)?,
            licence: self.opt_string(t, "licence", &p)?,
        })
    }

    /// A number given as a TOML number or a numeric string.
    fn float(&self, v: &Val<'_>, path: &dyn Render, key: &str) -> Result<f64, MaterialError> {
        let x = match v {
            Val::Float(f) => *f,
            Val::Int(i) => *i as f64,
            Val::Str(s) => s
                .trim()
                .parse::<f64>()
                .map_err(|_| self.bad_at(path, key, "expected a number"))?,
            _ => return Err(self.bad_at(path, key, "expected a number")),
        };
        if x.is_finite() {
            Ok(x)
        } else {
            Err(self.bad_at(path, key, "must be finite"))
        }
    }

    fn appearance(&self, v: &Val<'_>, path: &str, lib: &str) -> Result<Appearance, MaterialError> {
        let p = path.to_owned();
        let t = self.table(v, &p)?;
        self.keys_at(
            t,
            &[
                "id",
                "revision",
                "name",
                "base_color",
                "metallic",
                "roughness",
            ],
            &p,
        )?;
        let local = self.req_string(t, "id", &p)?;
        let id = AppearanceId::new(lib, &local)
            .map_err(|_| self.bad(&format!("{path}.id"), "expected [a-z0-9._-]+"))?;
        let color = self.req_string(t, "base_color", &p)?;
        let base_color = Srgba8::from_hex(&color).map_err(|_| {
            self.bad(
                &format!("{path}.base_color"),
                "expected #RRGGBB or #RRGGBBAA",
            )
        })?;
        let mut a = Appearance::new(id, self.req_string(t, "name", &p)?, base_color);
        a.revision = self.revision(t, &p)?;
        if let Some(m) = t.get("metallic") {
            a.metallic = self.float(m, &p, "metallic")? as f32;
        }
        if let Some(r) = t.get("roughness") {
            a.roughness = self.float(r, &p, "roughness")? as f32;
        }
        a.validate().map_err(|e| match e {
            MaterialError::Invalid {
                property, reason, ..
            } => self.bad(&format!("{path}.{property}"), &reason),
            e => e,
        })?;
        Ok(a)
    }

    fn material(&self, v: &Val<'_>, idx: usize, lib: &Library) -> Result<Material, MaterialError> {
        let path = format!("material[{idx}]");
        let t = self.table(v, &path)?;
        self.keys_at(
            t,
            &[
                "id",
                "revision",
                "name",
                "category",
                "family",
                "condition",
                "description",
                "default_source",
                "appearance",
                "tags",
                "density",
                "mechanical",
                "thermal",
                "am",
            ],
            &path,
        )?;
        let local = self.req_string(t, "id", &path)?;
        let id = MaterialId::new(&lib.meta.id, &local)
            .map_err(|_| self.bad(&format!("{path}.id"), "expected [a-z0-9._-]+"))?;
        let default_source = match self.opt_string(t, "default_source", &path)? {
            None => None,
            Some(k) => Some(self.resolve_source(lib, &k)?),
        };
        let mut mr = MatRd {
            rd: self,
            path: &path,
            lib,
            default_source: default_source.clone(),
            prov: BTreeMap::new(),
            used: BTreeSet::new(),
        };
        let category = self.req_string(t, "category", &path)?;
        let category = Category::from_str(&category)
            .ok_or_else(|| self.bad(&format!("{path}.category"), "unknown category"))?;
        let family = match self.opt_string(t, "family", &path)? {
            None => None,
            Some(f) => Some(
                MaterialFamily::parse(&f)
                    .ok_or_else(|| self.bad(&format!("{path}.family"), "unknown family"))?,
            ),
        };
        let appearance =
            match self.opt_string(t, "appearance", &path)? {
                None => None,
                Some(a) => Some(AppearanceId::parse(&a).map_err(|_| {
                    self.bad(&format!("{path}.appearance"), "expected <library>/<id>")
                })?),
            };
        let tags = match t.get("tags") {
            None => Vec::new(),
            Some(v) => {
                let items = v.as_array().ok_or_else(|| {
                    self.bad(&format!("{path}.tags"), "expected an array of strings")
                })?;
                items
                    .iter()
                    .map(|x| {
                        x.as_str().map(str::to_owned).ok_or_else(|| {
                            self.bad(&format!("{path}.tags"), "expected an array of strings")
                        })
                    })
                    .collect::<Result<_, _>>()?
            }
        };
        let density_v = t
            .get("density")
            .ok_or_else(|| self.bad(&format!("{path}.density"), "missing key"))?;
        let density = Density::from_canonical(mr.value(
            density_v,
            Path::new("density"),
            Dimension::DENSITY,
        )?);
        let mechanical = t.get("mechanical").map(|m| mr.mechanical(m)).transpose()?;
        let thermal = t.get("thermal").map(|m| mr.thermal(m)).transpose()?;
        let am = t.get("am").map(|m| mr.am(m)).transpose()?;
        let mut sources = BTreeMap::new();
        for k in mr.used.iter().chain(default_source.iter()) {
            if let Some(s) = lib.sources.get(k) {
                sources.insert(k.clone(), s.clone());
            }
        }
        let material = Material {
            id,
            revision: self.revision(t, &path)?,
            name: self.req_string(t, "name", &path)?,
            category,
            family,
            condition: self.opt_string(t, "condition", &path)?,
            description: self.opt_string(t, "description", &path)?,
            density,
            mechanical,
            thermal,
            am,
            appearance,
            tags,
            provenance: Provenance {
                default_source,
                values: mr.prov,
                sources,
            },
        };
        material.validate().map_err(|e| match e {
            MaterialError::Invalid {
                id,
                property,
                reason,
            } => MaterialError::Invalid {
                id,
                property: format!("{path}.{property}"),
                reason,
            },
            e => e,
        })?;
        Ok(material)
    }

    fn resolve_source(&self, lib: &Library, key: &str) -> Result<SourceKey, MaterialError> {
        let k = SourceKey(key.to_owned());
        if lib.sources.contains_key(&k) {
            Ok(k)
        } else {
            Err(MaterialError::UnknownSource {
                origin: self.origin.to_owned(),
                key: key.to_owned(),
            })
        }
    }
}

/// Reader state for one `[[material]]`.
struct MatRd<'a> {
    rd: &'a Rd<'a>,
    /// `material[i]`.
    path: &'a str,
    lib: &'a Library,
    default_source: Option<SourceKey>,
    prov: BTreeMap<PropPath, ValueProv>,
    used: BTreeSet<SourceKey>,
}

const PROV_KEYS: [&str; 6] = ["v", "src", "q", "spec", "std", "note"];

impl MatRd<'_> {
    fn mp<'p>(&'p self, prop: Path<'p>) -> MatPath<'p> {
        MatPath {
            mat: self.path,
            prop,
        }
    }

    fn missing_table(&self, prop: Path<'_>) -> MaterialError {
        self.rd.bad_at(&self.mp(prop), "", "missing table")
    }

    /// Reads one quantity (plain string or `{ v, src, q, spec, std, note }`) into canonical.
    fn value(&mut self, v: &Val<'_>, prop: Path<'_>, dim: Dimension) -> Result<f64, MaterialError> {
        match v {
            Val::Tab(t) => {
                self.rd.keys_at(t, &PROV_KEYS, &self.mp(prop))?;
                let inner = t
                    .get("v")
                    .ok_or_else(|| self.rd.bad_at(&self.mp(prop), "v", "missing key"))?;
                let x = self.scalar(inner, prop.at("v"), dim, true)?;
                self.prov_of(t, prop)?;
                Ok(x)
            }
            other => self.scalar(other, prop, dim, false),
        }
    }

    /// `inner`: the value sits under a `v` key (error paths then end in `.v`, except that
    /// quantity errors name the property itself).
    fn scalar(
        &self,
        v: &Val<'_>,
        prop: Path<'_>,
        dim: Dimension,
        inner: bool,
    ) -> Result<f64, MaterialError> {
        let origin = self.rd.origin;
        // Quantity errors point at the property, as the spec's `material[0].density`.
        let qpath = || {
            let mut p = prop;
            if inner {
                p.n -= 1;
            }
            self.mp(p).render()
        };
        match v {
            Val::Str(s) => {
                let p = parse_quantity(s).map_err(|source| MaterialError::Quantity {
                    origin: origin.to_owned(),
                    path: qpath(),
                    source,
                })?;
                match p.unit {
                    None if dim == Dimension::NONE => Ok(p.value),
                    None => Err(MaterialError::MissingUnit {
                        origin: origin.to_owned(),
                        path: qpath(),
                    }),
                    Some(u) if u.dimension() == dim => Ok(u.to_canonical(p.value)),
                    Some(u) => Err(MaterialError::Dimension {
                        origin: origin.to_owned(),
                        path: qpath(),
                        expected: dim,
                        found: u.dimension(),
                    }),
                }
            }
            Val::Float(_) | Val::Int(_) if dim == Dimension::NONE => {
                self.rd.float(v, &self.mp(prop), "")
            }
            Val::Float(_) | Val::Int(_) => Err(MaterialError::MissingUnit {
                origin: origin.to_owned(),
                path: qpath(),
            }),
            _ => Err(self
                .rd
                .bad_at(&self.mp(prop), "", "expected a string with a unit")),
        }
    }

    /// Records the provenance fields of an inline table (nothing if it equals the default).
    fn prov_of(&mut self, t: &Tab<'_>, prop: Path<'_>) -> Result<(), MaterialError> {
        let rd = self.rd;
        let mat: &str = self.path;
        let here = MatPath { mat, prop };
        let mut source = None;
        if let Some(s) = rd.opt_string(t, "src", &here)? {
            let k = rd.resolve_source(self.lib, &s)?;
            if self.default_source.as_ref() != Some(&k) {
                self.used.insert(k.clone());
                source = Some(k);
            }
        }
        let quality = match rd.opt_string(t, "q", &here)? {
            None => Quality::Measured,
            Some(q) => Quality::from_str(&q).ok_or_else(|| {
                rd.bad_at(&here, "q", "expected measured|derived|typical|estimate")
            })?,
        };
        let specimen = match rd.opt_string(t, "spec", &here)? {
            None => None,
            Some(s) => Some(SpecimenOrientation::parse(&s).ok_or_else(|| {
                rd.bad_at(
                    &here,
                    "spec",
                    "expected an ISO/ASTM 52921 orientation such as XYZ or ZXY",
                )
            })?),
        };
        let p = ValueProv {
            source,
            quality,
            specimen,
            standard: rd.opt_string(t, "std", &here)?,
            note: rd.opt_string(t, "note", &here)?,
        };
        if !p.is_default() {
            self.prov.insert(PropPath(prop.join()), p);
        }
        Ok(())
    }

    fn opt_value(
        &mut self,
        t: &Tab<'_>,
        key: &str,
        base: Path<'_>,
        dim: Dimension,
    ) -> Result<Option<f64>, MaterialError> {
        match t.get(key) {
            None => Ok(None),
            Some(v) => self.value(v, base.at(key), dim).map(Some),
        }
    }

    fn req_value(
        &mut self,
        t: &Tab<'_>,
        key: &str,
        base: Path<'_>,
        dim: Dimension,
    ) -> Result<f64, MaterialError> {
        match self.opt_value(t, key, base, dim)? {
            Some(x) => Ok(x),
            None => Err(self.rd.bad_at(&self.mp(base), key, "missing key")),
        }
    }

    /// A directional value: `{ xy = .., z = .. }`, or a plain value (isotropic).
    fn directional(
        &mut self,
        v: &Val<'_>,
        prop: Path<'_>,
        dim: Dimension,
    ) -> Result<Directional<f64>, MaterialError> {
        if let Val::Tab(t) = v
            && !t.contains_key("v")
        {
            self.rd.keys_at(t, &["xy", "z"], &self.mp(prop))?;
            let xy = t
                .get("xy")
                .ok_or_else(|| self.rd.bad_at(&self.mp(prop), "xy", "missing key"))?;
            let xy = self.value(xy, prop.at("xy"), dim)?;
            let z = t
                .get("z")
                .map(|z| self.value(z, prop.at("z"), dim))
                .transpose()?;
            return Ok(Directional { xy, z });
        }
        Ok(Directional::iso(self.value(v, prop.at("xy"), dim)?))
    }

    fn opt_directional(
        &mut self,
        t: &Tab<'_>,
        key: &str,
        base: Path<'_>,
        dim: Dimension,
    ) -> Result<Option<Directional<f64>>, MaterialError> {
        match t.get(key) {
            None => Ok(None),
            Some(v) => self.directional(v, base.at(key), dim).map(Some),
        }
    }

    fn mechanical(&mut self, v: &Val<'_>) -> Result<Mechanical, MaterialError> {
        let base = Path::new("mechanical");
        let t = self.rd.table(v, &self.mp(base))?;
        self.rd.keys_at(
            t,
            &[
                "elastic",
                "strength",
                "elongation_at_break",
                "hardness",
                "plasticity",
            ],
            &self.mp(base),
        )?;
        let elastic = t
            .get("elastic")
            .ok_or_else(|| self.missing_table(base.at("elastic")))?;
        let elastic = self.elastic(elastic)?;
        let strength = match t.get("strength") {
            None => Strength::default(),
            Some(s) => self.strength(s)?,
        };
        let elongation_at_break =
            self.opt_directional(t, "elongation_at_break", base, Dimension::NONE)?;
        let hardness = t.get("hardness").map(|h| self.hardness(h)).transpose()?;
        let plasticity = t
            .get("plasticity")
            .map(|p| self.plasticity(p))
            .transpose()?;
        Ok(Mechanical {
            elastic,
            strength,
            elongation_at_break,
            hardness,
            plasticity,
        })
    }

    fn elastic(&mut self, v: &Val<'_>) -> Result<Elastic, MaterialError> {
        let base = Path::new("mechanical").at("elastic");
        let t = self.rd.table(v, &self.mp(base))?;
        let model = self.rd.req_string(t, "model", &self.mp(base))?;
        let press = Dimension::PRESSURE;
        let none = Dimension::NONE;
        match model.as_str() {
            "isotropic" => {
                self.rd
                    .keys_at(t, &["model", "youngs", "poisson"], &self.mp(base))?;
                Ok(Elastic::Isotropic {
                    youngs: Pressure::from_canonical(self.req_value(t, "youngs", base, press)?),
                    poisson: self.req_value(t, "poisson", base, none)?,
                })
            }
            "transversely_isotropic" => {
                self.rd.keys_at(
                    t,
                    &["model", "e_p", "e_t", "nu_p", "nu_pt", "g_pt"],
                    &self.mp(base),
                )?;
                Ok(Elastic::TransverselyIsotropic {
                    e_p: Pressure::from_canonical(self.req_value(t, "e_p", base, press)?),
                    e_t: Pressure::from_canonical(self.req_value(t, "e_t", base, press)?),
                    nu_p: self.req_value(t, "nu_p", base, none)?,
                    nu_pt: self.req_value(t, "nu_pt", base, none)?,
                    g_pt: Pressure::from_canonical(self.req_value(t, "g_pt", base, press)?),
                })
            }
            "orthotropic" => {
                self.rd
                    .keys_at(t, &["model", "e", "nu", "g"], &self.mp(base))?;
                let e = self.triple(t, "e", ["1", "2", "3"], press)?;
                let nu = self.triple(t, "nu", ["12", "13", "23"], none)?;
                let g = self.triple(t, "g", ["12", "13", "23"], press)?;
                Ok(Elastic::Orthotropic {
                    e: e.map(Pressure::from_canonical),
                    nu,
                    g: g.map(Pressure::from_canonical),
                })
            }
            _ => Err(self.rd.bad_at(
                &self.mp(base),
                "model",
                "expected isotropic | transversely_isotropic | orthotropic",
            )),
        }
    }

    /// `key = [v1, v2, v3]` with provenance paths `mechanical.elastic.<key>.<suffix>`.
    fn triple(
        &mut self,
        t: &Tab<'_>,
        key: &str,
        suffixes: [&str; 3],
        dim: Dimension,
    ) -> Result<[f64; 3], MaterialError> {
        let base = Path::new("mechanical").at("elastic").at(key);
        let items = t
            .get(key)
            .and_then(Val::as_array)
            .filter(|a| a.len() == 3)
            .ok_or_else(|| {
                self.rd
                    .bad_at(&self.mp(base), "", "expected an array of 3 values")
            })?;
        let mut out = [0.0; 3];
        for (i, item) in items.iter().enumerate() {
            out[i] = self.value(item, base.at(suffixes[i]), dim)?;
        }
        Ok(out)
    }

    fn strength(&mut self, v: &Val<'_>) -> Result<Strength, MaterialError> {
        let base = Path::new("mechanical").at("strength");
        let t = self.rd.table(v, &self.mp(base))?;
        self.rd.keys_at(
            t,
            &[
                "yield_tensile",
                "ultimate_tensile",
                "ultimate_compressive",
                "shear",
                "flexural",
            ],
            &self.mp(base),
        )?;
        let press = Dimension::PRESSURE;
        let dir =
            |s: &mut Self, key: &str| -> Result<Option<Directional<Pressure>>, MaterialError> {
                Ok(s.opt_directional(t, key, base, press)?
                    .map(|d| d.map(Pressure::from_canonical)))
            };
        Ok(Strength {
            yield_tensile: dir(self, "yield_tensile")?,
            ultimate_tensile: dir(self, "ultimate_tensile")?,
            ultimate_compressive: dir(self, "ultimate_compressive")?,
            shear: self
                .opt_value(t, "shear", base, press)?
                .map(Pressure::from_canonical),
            flexural: self
                .opt_value(t, "flexural", base, press)?
                .map(Pressure::from_canonical),
        })
    }

    fn hardness(&mut self, v: &Val<'_>) -> Result<Hardness, MaterialError> {
        let base = Path::new("mechanical").at("hardness");
        let t = self.rd.table(v, &self.mp(base))?;
        let mut allowed = PROV_KEYS.to_vec();
        allowed.push("scale");
        self.rd.keys_at(t, &allowed, &self.mp(base))?;
        let scale = self.rd.req_string(t, "scale", &self.mp(base))?;
        let inner = t
            .get("v")
            .ok_or_else(|| self.rd.bad_at(&self.mp(base), "v", "missing key"))?;
        let x = self.rd.float(inner, &self.mp(base), "v")?;
        self.prov_of(t, base)?;
        match scale.as_str() {
            "shore_a" => Ok(Hardness::ShoreA(x)),
            "shore_d" => Ok(Hardness::ShoreD(x)),
            "vickers" => Ok(Hardness::Vickers(x)),
            "brinell" => Ok(Hardness::Brinell(x)),
            _ => Err(self.rd.bad_at(
                &self.mp(base),
                "scale",
                "expected shore_a | shore_d | vickers | brinell",
            )),
        }
    }

    fn plasticity(&mut self, v: &Val<'_>) -> Result<Plasticity, MaterialError> {
        let base = Path::new("mechanical").at("plasticity");
        let t = self.rd.table(v, &self.mp(base))?;
        let press = Dimension::PRESSURE;
        match self.rd.req_string(t, "model", &self.mp(base))?.as_str() {
            "bilinear" => {
                self.rd.keys_at(
                    t,
                    &["model", "yield_stress", "tangent_modulus"],
                    &self.mp(base),
                )?;
                Ok(Plasticity::Bilinear {
                    yield_stress: Pressure::from_canonical(self.req_value(
                        t,
                        "yield_stress",
                        base,
                        press,
                    )?),
                    tangent_modulus: Pressure::from_canonical(self.req_value(
                        t,
                        "tangent_modulus",
                        base,
                        press,
                    )?),
                })
            }
            "table" => {
                self.rd.keys_at(t, &["model", "points"], &self.mp(base))?;
                let pts = base.at("points");
                let items = t.get("points").and_then(Val::as_array).ok_or_else(|| {
                    self.rd
                        .bad_at(&self.mp(pts), "", "expected an array of { strain, stress }")
                })?;
                let mut points = Vec::with_capacity(items.len());
                for (i, p) in items.iter().enumerate() {
                    let here = format!("{}[{i}]", self.mp(pts).render());
                    let pt = self.rd.table(p, &here)?;
                    self.rd.keys_at(pt, &["strain", "stress"], &here)?;
                    let get = |k: &str| {
                        pt.get(k)
                            .ok_or_else(|| self.rd.bad_at(&here, k, "missing key"))
                    };
                    let strain =
                        self.points_scalar(get("strain")?, &here, "strain", Dimension::NONE)?;
                    let stress = self.points_scalar(get("stress")?, &here, "stress", press)?;
                    points.push((strain, Pressure::from_canonical(stress)));
                }
                Ok(Plasticity::Table { points })
            }
            _ => Err(self
                .rd
                .bad_at(&self.mp(base), "model", "expected bilinear | table")),
        }
    }

    /// A `points[i].strain|stress` scalar (error paths carry the index).
    fn points_scalar(
        &self,
        v: &Val<'_>,
        here: &str,
        key: &str,
        dim: Dimension,
    ) -> Result<f64, MaterialError> {
        let origin = self.rd.origin;
        let path = || format!("{here}.{key}");
        match v {
            Val::Str(s) => {
                let p = parse_quantity(s).map_err(|source| MaterialError::Quantity {
                    origin: origin.to_owned(),
                    path: path(),
                    source,
                })?;
                match p.unit {
                    None if dim == Dimension::NONE => Ok(p.value),
                    None => Err(MaterialError::MissingUnit {
                        origin: origin.to_owned(),
                        path: path(),
                    }),
                    Some(u) if u.dimension() == dim => Ok(u.to_canonical(p.value)),
                    Some(u) => Err(MaterialError::Dimension {
                        origin: origin.to_owned(),
                        path: path(),
                        expected: dim,
                        found: u.dimension(),
                    }),
                }
            }
            Val::Float(_) | Val::Int(_) if dim == Dimension::NONE => {
                self.rd.float(v, &here.to_owned(), key)
            }
            Val::Float(_) | Val::Int(_) => Err(MaterialError::MissingUnit {
                origin: origin.to_owned(),
                path: path(),
            }),
            _ => Err(self.rd.bad(&path(), "expected a string with a unit")),
        }
    }

    fn thermal(&mut self, v: &Val<'_>) -> Result<Thermal, MaterialError> {
        let base = Path::new("thermal");
        let t = self.rd.table(v, &self.mp(base))?;
        self.rd.keys_at(
            t,
            &[
                "conductivity",
                "specific_heat",
                "expansion",
                "expansion_reference",
                "glass_transition",
                "melting",
                "heat_deflection_045",
                "max_service",
            ],
            &self.mp(base),
        )?;
        let temp = Dimension::TEMPERATURE;
        let t_of = |s: &mut Self, key: &str| -> Result<Option<Temperature>, MaterialError> {
            Ok(s.opt_value(t, key, base, temp)?
                .map(Temperature::from_canonical))
        };
        let expansion_reference = t_of(self, "expansion_reference")?;
        let glass_transition = t_of(self, "glass_transition")?;
        let melting = t_of(self, "melting")?;
        let heat_deflection_045 = t_of(self, "heat_deflection_045")?;
        let max_service = t_of(self, "max_service")?;
        Ok(Thermal {
            conductivity: self
                .opt_directional(t, "conductivity", base, Dimension::THERMAL_CONDUCTIVITY)?
                .map(|d| d.map(ThermalConductivity::from_canonical)),
            specific_heat: self
                .opt_value(t, "specific_heat", base, Dimension::SPECIFIC_HEAT)?
                .map(SpecificHeat::from_canonical),
            expansion: self
                .opt_directional(t, "expansion", base, Dimension::THERMAL_EXPANSION)?
                .map(|d| d.map(ThermalExpansion::from_canonical)),
            expansion_reference,
            glass_transition,
            melting,
            heat_deflection_045,
            max_service,
        })
    }

    fn range(&mut self, t: &Tab<'_>, key: &str) -> Result<Option<TempRange>, MaterialError> {
        let Some(v) = t.get(key) else { return Ok(None) };
        let base = Path::new("am").at(key);
        let r = self.rd.table(v, &self.mp(base))?;
        self.rd.keys_at(r, &["min", "max"], &self.mp(base))?;
        let temp = Dimension::TEMPERATURE;
        let min = self.req_value(r, "min", base, temp)?;
        let max = self.req_value(r, "max", base, temp)?;
        Ok(Some(TempRange {
            min: Temperature::from_canonical(min),
            max: Temperature::from_canonical(max),
        }))
    }

    fn am(&mut self, v: &Val<'_>) -> Result<AmInfo, MaterialError> {
        let base = Path::new("am");
        let t = self.rd.table(v, &self.mp(base))?;
        self.rd.keys_at(
            t,
            &[
                "process",
                "slicer_type",
                "nozzle_temp",
                "bed_temp",
                "chamber_temp",
                "shrinkage",
                "cost",
            ],
            &self.mp(base),
        )?;
        let process = self.rd.req_string(t, "process", &self.mp(base))?;
        let process = AmProcess::from_str(&process).ok_or_else(|| {
            self.rd
                .bad_at(&self.mp(base), "process", "unknown ISO/ASTM 52900 process")
        })?;
        let cost = match t.get("cost") {
            None => None,
            Some(c) => {
                let cbase = base.at("cost");
                let ct = self.rd.table(c, &self.mp(cbase))?;
                self.rd
                    .keys_at(ct, &["per_kg", "currency"], &self.mp(cbase))?;
                let per_kg = ct
                    .get("per_kg")
                    .ok_or_else(|| self.rd.bad_at(&self.mp(cbase), "per_kg", "missing key"))?;
                Some(Cost {
                    per_kg: self.rd.float(per_kg, &self.mp(cbase), "per_kg")?,
                    currency: self.rd.req_string(ct, "currency", &self.mp(cbase))?,
                })
            }
        };
        Ok(AmInfo {
            process,
            slicer_type: self.rd.opt_string(t, "slicer_type", &self.mp(base))?,
            nozzle_temp: self.range(t, "nozzle_temp")?,
            bed_temp: self.range(t, "bed_temp")?,
            chamber_temp: self.range(t, "chamber_temp")?,
            shrinkage: self.opt_directional(t, "shrinkage", base, Dimension::NONE)?,
            cost,
        })
    }
}
