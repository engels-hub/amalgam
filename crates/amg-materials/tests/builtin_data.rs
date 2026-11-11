//! MAT-05 data tests on the embedded built-in library (note §3.2 provenance rules, §3.4
//! plausibility bounds), the content-hash golden `tests/golden/materials/builtin.toml` and the
//! seeded review sample.
//!
//! `AMG_BLESS=1 cargo nextest run -p amg-materials -E 'test(golden)'` rewrites the golden.
//! `cargo nextest run -p amg-materials -E 'test(seeded_review_sample)' --no-capture` prints the
//! ten values a reviewer checks against the cited documents.
#![allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;
use std::time::Instant;

use amg_materials::{
    AmProcess, Elastic, Library, Material, MaterialFamily, PropPath, Quality, SourceKind, builtin,
    default_material_id,
};

const MATERIALS_TOML: &str = include_str!("../data/builtin/materials.toml");

/// The 17 materials of S17 §Design "Built-in library v1".
const V1_IDS: [&str; 17] = [
    "pla.fff",
    "petg.fff",
    "abs.fff",
    "asa.fff",
    "pa12.fff",
    "pa-cf.fff",
    "tpu-95a.fff",
    "pa12.pbf",
    "resin.standard",
    "resin.tough",
    "ti6al4v.lpbf",
    "alsi10mg.lpbf",
    "316l.lpbf",
    "steel.s235",
    "aluminium.6061-t6",
    "stainless.1.4404",
    "ti6al4v.wrought",
];

/// FFF materials whose cited TDS reports Z specimens (all seven do).
const FFF_WITH_Z: [&str; 7] = [
    "pla.fff",
    "petg.fff",
    "abs.fff",
    "asa.fff",
    "pa12.fff",
    "pa-cf.fff",
    "tpu-95a.fff",
];

fn quality(m: &Material, path: &str) -> Quality {
    m.provenance
        .values
        .get(&PropPath(path.to_owned()))
        .map_or(Quality::Measured, |p| p.quality)
}

fn youngs_mpa(m: &Material) -> f64 {
    match m.elastic().unwrap() {
        Elastic::Isotropic { youngs, .. } => youngs.get(),
        Elastic::TransverselyIsotropic { e_p, .. } => e_p.get(),
        Elastic::Orthotropic { e, .. } => e[0].get(),
    }
}

fn poisson(m: &Material) -> f64 {
    match m.elastic().unwrap() {
        Elastic::Isotropic { poisson, .. } => *poisson,
        Elastic::TransverselyIsotropic { nu_p, .. } => *nu_p,
        Elastic::Orthotropic { nu, .. } => nu[0],
    }
}

/// The in-plane tensile strength used by the provenance and bounds rules: ultimate if given,
/// else yield, with its provenance path.
fn tensile_xy(m: &Material) -> (f64, &'static str) {
    let s = &m.mechanical.as_ref().unwrap().strength;
    if let Some(u) = &s.ultimate_tensile {
        (u.xy.get(), "mechanical.strength.ultimate_tensile.xy")
    } else {
        let y = s.yield_tensile.as_ref().expect("a tensile strength");
        (y.xy.get(), "mechanical.strength.yield_tensile.xy")
    }
}

#[test]
fn builtin_loads_validates_and_has_the_v1_set() {
    let lib = builtin();
    let ids: BTreeSet<&str> = lib.materials().map(|m| m.id.local()).collect();
    assert_eq!(ids, V1_IDS.into_iter().collect(), "v1 material ids");
    for m in lib.materials() {
        m.validate().unwrap_or_else(|e| panic!("{}: {e}", m.id));
        assert_eq!(m.id.library(), "amg.builtin");
    }
    assert!(
        lib.material(&default_material_id()).is_some(),
        "default PLA"
    );
    // The embedded file round-trips through the writer.
    let again = Library::from_toml_str(&lib.to_toml_string(), "builtin-resaved").unwrap();
    assert_eq!(again.materials().count(), 17);
    for (a, b) in lib.materials().zip(again.materials()) {
        assert_eq!(a, b);
    }
}

#[test]
fn provenance_rules_note_3_2() {
    let lib = builtin();
    for m in lib.materials() {
        let id = &m.id;
        // density, in-plane E and one tensile strength are `measured`; density never `estimate`.
        assert_eq!(quality(m, "density"), Quality::Measured, "{id} density");
        let e_path = match m.elastic().unwrap() {
            Elastic::Isotropic { .. } => "mechanical.elastic.youngs",
            Elastic::TransverselyIsotropic { .. } => "mechanical.elastic.e_p",
            Elastic::Orthotropic { .. } => "mechanical.elastic.e.1",
        };
        assert_eq!(quality(m, e_path), Quality::Measured, "{id} {e_path}");
        let (_, s_path) = tensile_xy(m);
        assert_eq!(quality(m, s_path), Quality::Measured, "{id} {s_path}");

        // Every material states its condition.
        assert!(
            m.condition.as_deref().is_some_and(|c| !c.trim().is_empty()),
            "{id} condition"
        );

        // Every source key resolves, in the material copy and in the library.
        let mut keys: Vec<_> = m
            .provenance
            .values
            .values()
            .filter_map(|p| p.source.as_ref())
            .collect();
        keys.extend(m.provenance.default_source.as_ref());
        assert!(m.provenance.default_source.is_some(), "{id} default_source");
        for k in keys {
            assert!(m.provenance.sources.contains_key(k), "{id}: {} copied", k.0);
            assert!(lib.source(k).is_some(), "{id}: {} in library", k.0);
        }

        // `estimate` is only used for Poisson's ratio, and only with a note.
        for (path, p) in &m.provenance.values {
            if p.quality == Quality::Estimate {
                assert_eq!(path.0, "mechanical.elastic.poisson", "{id} estimate");
                assert!(p.note.is_some(), "{id} {} estimate note", path.0);
            }
        }
    }
}

#[test]
fn every_source_is_a_primary_document() {
    // Note §3.1: no aggregator or commercial database, no Fusion library.
    const BANNED: [&str; 9] = [
        "matweb",
        "campus",
        "granta",
        "ansys",
        "total materia",
        "totalmateria",
        "autodesk",
        "fusion",
        "prospector",
    ];
    let lib = builtin();
    let mut n = 0;
    for s in lib.sources() {
        n += 1;
        let text = format!(
            "{} {} {}",
            s.title,
            s.publisher.as_deref().unwrap_or(""),
            s.url.as_deref().unwrap_or("")
        )
        .to_ascii_lowercase();
        for b in BANNED {
            assert!(!text.contains(b), "source {} mentions {b}", s.key.0);
        }
        assert_ne!(s.kind, SourceKind::CommunityDb, "{} community db", s.key.0);
        if s.kind != SourceKind::Estimate {
            assert!(
                s.url.is_some() && s.accessed.is_some(),
                "{} url/accessed",
                s.key.0
            );
        }
    }
    assert!(n >= 17, "{n} sources");
}

/// Plausibility bounds of note §3.4: (ρ g/cm³, E MPa, ν, tensile strength MPa).
struct Bounds {
    name: &'static str,
    rho: (f64, f64),
    e_mpa: (f64, f64),
    nu: (f64, f64),
    uts: (f64, f64),
}

fn bounds_for(m: &Material) -> Bounds {
    let b = |name, rho, e_mpa, nu, uts| Bounds {
        name,
        rho,
        e_mpa,
        nu,
        uts,
    };
    let pbf_polymer =
        m.am.as_ref()
            .is_some_and(|a| a.process == AmProcess::PowderBedFusionPolymer);
    use MaterialFamily as F;
    match m.family.as_ref().expect("family") {
        // Sintered PA12 is porous (0.93 g/cm³), below the FFF row; own row (S17 log, MAT-05).
        F::Pa12 | F::Pa11 if pbf_polymer => b(
            "PBF polymer",
            (0.85, 1.10),
            (1000.0, 2500.0),
            (0.30, 0.42),
            (30.0, 60.0),
        ),
        F::Pla | F::Petg | F::Pctg | F::Abs | F::Asa | F::Pc | F::Pa6 | F::Pa11 | F::Pa12 => b(
            "unfilled FFF",
            (0.95, 1.45),
            (800.0, 4500.0),
            (0.30, 0.42),
            (15.0, 90.0),
        ),
        F::PaCf | F::PaGf | F::PlaCf | F::PetgCf => b(
            "fibre-filled FFF",
            (1.0, 1.5),
            (2000.0, 15000.0),
            (0.25, 0.42),
            (30.0, 150.0),
        ),
        F::Tpu => b(
            "TPU",
            (1.05, 1.30),
            (5.0, 150.0),
            (0.40, 0.49),
            (10.0, 60.0),
        ),
        F::ResinStandard | F::ResinTough | F::ResinFlexible | F::ResinHighTemp => b(
            "photopolymer",
            (1.05, 1.30),
            (300.0, 4000.0),
            (0.30, 0.45),
            (15.0, 90.0),
        ),
        F::Ti6Al4V => b(
            "Ti-6Al-4V",
            (4.35, 4.50),
            (100e3, 125e3),
            (0.30, 0.36),
            (850.0, 1300.0),
        ),
        F::AlSi10Mg => b(
            "AlSi10Mg",
            (2.60, 2.72),
            (60e3, 80e3),
            (0.30, 0.35),
            (250.0, 500.0),
        ),
        F::Ss316L => b(
            "316L",
            (7.85, 8.05),
            (160e3, 210e3),
            (0.26, 0.31),
            (450.0, 750.0),
        ),
        F::CarbonSteel | F::StainlessSteel | F::AluminiumAlloy => b(
            "wrought steel / Al",
            (2.6, 8.1),
            (65e3, 215e3),
            (0.26, 0.35),
            (200.0, 800.0),
        ),
        other => panic!("{}: no plausibility bounds for {other:?}", m.id),
    }
}

#[test]
fn plausibility_bounds_note_3_4() {
    let within = |v: f64, (lo, hi): (f64, f64)| v >= lo && v <= hi;
    for m in builtin().materials() {
        let b = bounds_for(m);
        let rho = m.density.to_g_per_cm3();
        let e = youngs_mpa(m);
        let nu = poisson(m);
        let (uts, _) = tensile_xy(m);
        assert!(within(rho, b.rho), "{} ({}): rho {rho} g/cm3", m.id, b.name);
        assert!(within(e, b.e_mpa), "{} ({}): E {e} MPa", m.id, b.name);
        assert!(within(nu, b.nu), "{} ({}): nu {nu}", m.id, b.name);
        assert!(within(uts, b.uts), "{} ({}): UTS {uts} MPa", m.id, b.name);
    }
}

#[test]
fn fff_materials_with_z_specimens_have_z_strengths() {
    let lib = builtin();
    for m in lib.materials() {
        let fff =
            m.am.as_ref()
                .is_some_and(|a| a.process == AmProcess::MaterialExtrusion);
        assert_eq!(fff, FFF_WITH_Z.contains(&m.id.local()), "{} FFF list", m.id);
        if !fff {
            continue;
        }
        let s = &m.mechanical.as_ref().unwrap().strength;
        let z = [
            ("ultimate_tensile", &s.ultimate_tensile),
            ("yield_tensile", &s.yield_tensile),
        ]
        .into_iter()
        .filter_map(|(n, d)| d.as_ref().and_then(|d| d.z.map(|z| (n, z))))
        .collect::<Vec<_>>();
        assert!(!z.is_empty(), "{} has a z strength", m.id);
        for (name, v) in z {
            let p = &m.provenance.values[&PropPath(format!("mechanical.strength.{name}.z"))];
            assert!(
                p.specimen.is_some_and(|s| s.is_z()),
                "{} {name}.z specimen",
                m.id
            );
            let xy = match name {
                "ultimate_tensile" => s.ultimate_tensile.as_ref().unwrap().xy,
                _ => s.yield_tensile.as_ref().unwrap().xy,
            };
            assert!(v <= xy, "{} {name}: z {v:?} <= xy {xy:?}", m.id);
        }
        assert!(
            m.am.as_ref().unwrap().slicer_type.is_some(),
            "{} slicer",
            m.id
        );
    }
}

#[test]
fn appearances_resolve_and_cover_the_generic_set() {
    let lib = builtin();
    for m in lib.materials() {
        let a = m.appearance.as_ref().expect("default appearance");
        assert!(lib.appearance(a).is_some(), "{}: {a} resolves", m.id);
    }
    let ids: BTreeSet<&str> = lib.appearances().map(|a| a.id.local()).collect();
    assert!(ids.len() >= 12, "{} appearances", ids.len());
    for want in [
        "plastic.white-matte",
        "plastic.black",
        "plastic.red",
        "plastic.blue",
        "metal.aluminium-brushed",
        "metal.steel",
        "metal.titanium",
        "resin.grey",
        "resin.clear",
    ] {
        assert!(ids.contains(want), "{want}");
    }
}

#[test]
fn builtin_parse_budget() {
    // S17 performance target: `builtin()` first call < 5 ms (asserted only when strict).
    let src = builtin().to_toml_string();
    let mut runs: Vec<f64> = (0..15)
        .map(|_| {
            let t = Instant::now();
            std::hint::black_box(Library::from_toml_str(&src, "builtin").unwrap());
            t.elapsed().as_secs_f64() * 1e3
        })
        .collect();
    runs.sort_by(f64::total_cmp);
    let ms = runs[runs.len() / 2];
    println!("built-in library parse: {ms:.3} ms median");
    if std::env::var_os("AMG_STRICT_TIMING").is_some_and(|v| v == "1") {
        assert!(ms < 5.0, "built-in parse took {ms:.3} ms, budget 5 ms");
    }
}

// ---- content-hash golden -------------------------------------------------------------------

fn golden_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/materials/builtin.toml")
}

fn hex(h: [u8; 32]) -> String {
    h.iter().map(|b| format!("{b:02x}")).collect()
}

fn render_golden(lib: &Library) -> String {
    let mut out = String::from(
        "# Content hash of every built-in material (MAT-05). A changed hash needs a revision bump.\n\
         # Regenerate: AMG_BLESS=1 cargo nextest run -p amg-materials -E 'test(golden)'\n",
    );
    for m in lib.materials() {
        out.push_str(&format!(
            "\n[[material]]\nid = \"{}\"\nrevision = {}\nhash = \"{}\"\n",
            m.id,
            m.revision,
            hex(m.content_hash())
        ));
    }
    out
}

#[test]
fn golden_content_hashes() {
    let lib = builtin();
    let path = golden_path();
    let old: BTreeMap<String, (u32, String)> = std::fs::read_to_string(&path)
        .ok()
        .map(|s| {
            let v: toml::Value = toml::from_str(&s).expect("golden parses");
            v.get("material")
                .and_then(|m| m.as_array())
                .map(|a| {
                    a.iter()
                        .map(|t| {
                            let id = t["id"].as_str().unwrap().to_owned();
                            let rev = u32::try_from(t["revision"].as_integer().unwrap()).unwrap();
                            (id, (rev, t["hash"].as_str().unwrap().to_owned()))
                        })
                        .collect()
                })
                .unwrap_or_default()
        })
        .unwrap_or_default();

    // A value change must come with a revision bump, blessing or not.
    for m in lib.materials() {
        if let Some((rev, hash)) = old.get(m.id.as_str())
            && *hash != hex(m.content_hash())
        {
            assert!(
                m.revision > *rev,
                "{}: content changed but revision is still {} (bump it)",
                m.id,
                m.revision
            );
        }
    }

    let new = render_golden(lib);
    if std::env::var("AMG_BLESS").is_ok_and(|v| v == "1") {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, &new).unwrap();
        return;
    }
    let on_disk = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e} (run with AMG_BLESS=1)", path.display()));
    assert_eq!(
        on_disk.replace("\r\n", "\n"),
        new,
        "{} is stale; regenerate with AMG_BLESS=1",
        path.display()
    );
}

// ---- seeded review sample ------------------------------------------------------------------

/// One value as written in `materials.toml`, with its citation.
#[derive(Debug, Clone)]
struct Cited {
    material: String,
    path: String,
    value: String,
    source: String,
    note: String,
}

/// Keys of a material table that are metadata, not values.
const META: [&str; 13] = [
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
    "model",
    "process",
    "slicer_type",
];

fn collect(v: &toml::Value, path: &str, default_src: &str, mat: &str, out: &mut Vec<Cited>) {
    let push = |out: &mut Vec<Cited>, value: String, src: &str, note: &str| {
        out.push(Cited {
            material: mat.to_owned(),
            path: path.to_owned(),
            value,
            source: src.to_owned(),
            note: note.to_owned(),
        });
    };
    match v {
        toml::Value::String(s) => push(out, s.clone(), default_src, ""),
        toml::Value::Table(t) if t.contains_key("v") => {
            let value = match &t["v"] {
                toml::Value::String(s) => s.clone(),
                other => other.to_string(),
            };
            let src = t.get("src").and_then(|s| s.as_str()).unwrap_or(default_src);
            let note = t.get("note").and_then(|s| s.as_str()).unwrap_or("");
            let value = match t.get("scale").and_then(|s| s.as_str()) {
                Some(scale) => format!("{value} ({scale})"),
                None => value,
            };
            push(out, value, src, note);
        }
        toml::Value::Table(t) => {
            for (k, v) in t {
                if META.contains(&k.as_str()) {
                    continue;
                }
                let p = if path.is_empty() {
                    k.clone()
                } else {
                    format!("{path}.{k}")
                };
                collect(v, &p, default_src, mat, out);
            }
        }
        _ => {}
    }
}

fn cited_values() -> Vec<Cited> {
    let doc: toml::Value = toml::from_str(MATERIALS_TOML).unwrap();
    let mut out = Vec::new();
    for m in doc["material"].as_array().unwrap() {
        let id = m["id"].as_str().unwrap();
        let src = m["default_source"].as_str().unwrap();
        collect(m, "", src, id, &mut out);
    }
    out
}

/// splitmix64, so the sample is reproducible without a `rand` dependency.
fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Seed of the MAT-05 review sample (listed in the PR description / handoff report).
const REVIEW_SEED: u64 = 0x4D41_5430_3500_2026;

fn review_sample() -> Vec<Cited> {
    let all = cited_values();
    let mut state = REVIEW_SEED;
    let mut picked = BTreeSet::new();
    while picked.len() < 10 {
        let i = usize::try_from(splitmix64(&mut state) % all.len() as u64).unwrap();
        picked.insert(i);
    }
    picked.into_iter().map(|i| all[i].clone()).collect()
}

#[test]
fn seeded_review_sample() {
    let lib = builtin();
    let all = cited_values();
    assert!(all.len() > 150, "{} cited values", all.len());
    let sample = review_sample();
    assert_eq!(sample.len(), 10);
    assert_eq!(review_sample().len(), 10);
    println!(
        "MAT-05 review sample (seed {REVIEW_SEED:#x}, {} values):",
        all.len()
    );
    for c in &sample {
        let s = lib
            .sources()
            .find(|s| s.key.0 == c.source)
            .unwrap_or_else(|| panic!("source {}", c.source));
        println!(
            "- {} {} = {} | {} ({}) | {}",
            c.material,
            c.path,
            c.value,
            s.title,
            s.version.as_deref().or(s.date.as_deref()).unwrap_or("-"),
            if c.note.is_empty() { "-" } else { &c.note }
        );
    }
}

#[test]
fn match_name_finds_real_builtins() {
    // MAT-07 leftover: the import name checks now run on the real built-in data.
    let set = amg_materials::LibrarySet::builtin_only();
    let local = |q: &str| set.match_name(q).map(|m| m.id.local().to_owned());
    assert_eq!(local("PLA").as_deref(), Some("pla.fff"));
    assert_eq!(local("pla").as_deref(), Some("pla.fff"));
    assert_eq!(local("PETG Basic").as_deref(), Some("petg.fff"));
    assert_eq!(local("Bambu PAHT-CF").as_deref(), Some("pa-cf.fff"));
}
