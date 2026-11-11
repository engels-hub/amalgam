//! MAT-01 Done-when (a)–(g) and the supporting unit tests.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    reason = "test and bench code"
)]

use amg_core::units::{Density, Dimension, Pressure, Volume};
use amg_materials::{
    Appearance, AppearanceId, Category, Directional, Elastic, IsoPolicy, Library, LibraryMeta,
    Material, MaterialError, MaterialFamily, MaterialId, Mechanical, Srgba8, Strength, builtin,
    default_material_id,
};
use proptest::prelude::*;

const FULL: &str = include_str!("fixtures/full.toml");

fn full() -> Library {
    Library::from_toml_str(FULL, "full.toml").expect("fixture loads")
}

fn load_err(src: &str) -> MaterialError {
    Library::from_toml_str(src, "t.toml").expect_err("must fail")
}

/// A one-material library around `body` (the `[[material]]` contents after `id`/`name`).
fn one(body: &str) -> String {
    format!(
        "format = \"amalgam-materials\"\nformat_version = 1\n\
         [library]\nid = \"user.t\"\nname = \"t\"\nversion = \"1\"\n\
         [[source]]\nkey = \"s\"\nkind = \"datasheet\"\ntitle = \"S\"\n\
         [[material]]\nid = \"m\"\nname = \"M\"\ncategory = \"polymer\"\n{body}"
    )
}

fn assert_invalid(src: &str, property: &str) {
    match load_err(src) {
        MaterialError::Invalid { property: p, .. } => assert_eq!(p, property),
        e => panic!("expected Invalid at `{property}`, got {e:?}"),
    }
}

// ---- (a) fixture with every property kind; round trips ----------------------------------

#[test]
fn full_fixture_uses_every_property_kind() {
    let lib = full();
    assert_eq!(lib.meta().id, "user.fixture");
    assert_eq!(lib.materials().count(), 5);
    assert_eq!(lib.appearances().count(), 2);
    let id = |l: &str| MaterialId::new("user.fixture", l).unwrap();
    let iso = lib.material(&id("iso.full")).unwrap();
    assert_eq!(iso.revision, 3);
    assert_eq!(iso.family, Some(MaterialFamily::Petg));
    assert!(iso.description.as_deref().unwrap().contains("\"quoted\""));
    let m = iso.mechanical.as_ref().unwrap();
    assert!(m.elastic.is_isotropic());
    assert!(m.hardness.is_some() && m.plasticity.is_some());
    let s = &m.strength;
    assert!(s.yield_tensile.unwrap().z.is_some());
    assert!(s.ultimate_tensile.is_some() && s.ultimate_compressive.is_some());
    assert!(s.shear.is_some() && s.flexural.is_some());
    let e = m.elongation_at_break.unwrap();
    assert!((e.xy - 0.12).abs() < 1e-15 && (e.z.unwrap() - 0.04).abs() < 1e-15);
    let t = iso.thermal.as_ref().unwrap();
    assert!(t.conductivity.unwrap().z.is_some() && t.specific_heat.is_some());
    assert!(t.expansion.unwrap().z.is_some() && t.expansion_reference.is_some());
    assert!(t.glass_transition.is_some() && t.melting.is_some());
    assert!(t.heat_deflection_045.is_some() && t.max_service.is_some());
    let am = iso.am.as_ref().unwrap();
    assert!(am.nozzle_temp.is_some() && am.bed_temp.is_some() && am.chamber_temp.is_some());
    assert!(am.shrinkage.is_some() && am.cost.is_some() && am.slicer_type.is_some());
    // Provenance: default source, an override, specimen and standard.
    assert_eq!(iso.provenance.default_source.as_ref().unwrap().0, "tds-a");
    let p = iso
        .provenance_of("mechanical.elongation_at_break.xy")
        .unwrap();
    assert_eq!(p.source.as_ref().unwrap().0, "handbook-b");
    assert_eq!(iso.provenance.sources.len(), 2);
    assert!(
        iso.provenance_of("mechanical.strength.yield_tensile.z")
            .unwrap()
            .specimen
            .unwrap()
            .is_z()
    );
    assert!(iso.is_anisotropic());

    let ti = lib.material(&id("ti.fff")).unwrap();
    assert!(matches!(
        ti.elastic().unwrap(),
        Elastic::TransverselyIsotropic { .. }
    ));
    assert!(ti.is_anisotropic());
    let ortho = lib.material(&id("ortho.metal")).unwrap();
    assert!(matches!(
        ortho.elastic().unwrap(),
        Elastic::Orthotropic { .. }
    ));
    assert_eq!(ortho.family, Some(MaterialFamily::Other("my-alloy".into())));
    let bare = lib.material(&id("bare.minimum")).unwrap();
    assert!(bare.mechanical.is_none() && bare.elastic().is_err());
    assert!(!bare.is_anisotropic());
}

/// Absolute spacing in ulps between two finite f64 of the same sign.
fn ulps(a: f64, b: f64) -> u64 {
    if a.to_bits() == b.to_bits() {
        return 0;
    }
    (a.to_bits() as i64 - b.to_bits() as i64).unsigned_abs()
}

fn flat_values(m: &Material) -> Vec<f64> {
    let j = serde_json::to_value(m).unwrap();
    let mut out = Vec::new();
    fn walk(v: &serde_json::Value, out: &mut Vec<f64>) {
        match v {
            serde_json::Value::Number(n) => out.push(n.as_f64().unwrap()),
            serde_json::Value::Array(a) => a.iter().for_each(|x| walk(x, out)),
            serde_json::Value::Object(o) => o.values().for_each(|x| walk(x, out)),
            _ => {}
        }
    }
    walk(&j, &mut out);
    out
}

#[test]
fn load_save_load_keeps_values_and_second_save_is_byte_identical() {
    let first = full();
    let text1 = first.to_toml_string();
    let second = Library::from_toml_str(&text1, "saved.toml").expect("saved text loads");
    let text2 = second.to_toml_string();
    assert_eq!(text1, text2, "second save must be byte-identical");
    for (a, b) in first.materials().zip(second.materials()) {
        assert_eq!(a, b, "material {} changed", a.id);
        let (va, vb) = (flat_values(a), flat_values(b));
        assert_eq!(va.len(), vb.len());
        for (x, y) in va.iter().zip(&vb) {
            assert!(ulps(*x, *y) <= 2, "{x} vs {y}");
        }
    }
    for (a, b) in first.appearances().zip(second.appearances()) {
        assert_eq!(a, b);
    }
    assert_eq!(first.meta(), second.meta());
    assert_eq!(first.sources().count(), second.sources().count());
}

#[test]
fn writer_uses_library_units_and_plain_strings_for_default_provenance() {
    let text = full().to_toml_string();
    assert!(
        text.contains("density = { v = \"1.27 g/cm^3\", std = \"ISO 1183\" }"),
        "{text}"
    );
    assert!(text.contains("e_p = \"6000 MPa\""), "{text}");
    assert!(text.contains("specific_heat = \"1800 J/(kg*K)\""), "{text}");
    assert!(text.contains("\"68 ppm/K\""), "{text}");
    assert!(text.contains("glass_transition = \"80 degC\""), "{text}");
    let first_material = text.find("[[material]]").unwrap();
    let ids: Vec<&str> = text[first_material..]
        .lines()
        .filter(|l| l.starts_with("id = "))
        .collect();
    let mut sorted = ids.clone();
    sorted.sort_unstable();
    assert_eq!(ids, sorted, "materials sorted by id");
}

// ---- (b) JSON bit-exactness -------------------------------------------------------------

#[test]
fn json_round_trip_is_bit_exact() {
    let lib = full();
    for m in lib.materials().chain(builtin().materials()) {
        let json = serde_json::to_string(&**m).unwrap();
        let back: Material = serde_json::from_str(&json).unwrap();
        assert_eq!(&back, &**m, "{}", m.id);
        assert_eq!(serde_json::to_string(&back).unwrap(), json);
        let (a, b) = (flat_values(m), flat_values(&back));
        assert!(a.iter().zip(&b).all(|(x, y)| x.to_bits() == y.to_bits()));
    }
}

fn arb_material() -> impl Strategy<Value = Material> {
    (
        1e-10f64..2e-8,
        1.0f64..400_000.0,
        -0.9f64..0.49,
        proptest::option::of((1.0f64..2000.0, 1.0f64..2000.0)),
        proptest::option::of(1e-3f64..1.0),
        0u32..3,
    )
        .prop_map(|(rho, e, nu, strength, elong, model)| {
            let elastic = match model {
                0 => Elastic::Isotropic {
                    youngs: Pressure::from_canonical(e),
                    poisson: nu,
                },
                1 => Elastic::TransverselyIsotropic {
                    e_p: Pressure::from_canonical(e),
                    e_t: Pressure::from_canonical(e * 0.7),
                    nu_p: nu.min(0.4),
                    nu_pt: 0.1,
                    g_pt: Pressure::from_canonical(e * 0.3),
                },
                _ => Elastic::Orthotropic {
                    e: [
                        Pressure::from_canonical(e),
                        Pressure::from_canonical(e * 0.9),
                        Pressure::from_canonical(e * 0.8),
                    ],
                    nu: [0.1, 0.1, 0.1],
                    g: [Pressure::from_canonical(e * 0.3); 3],
                },
            };
            let mut m = Material {
                id: MaterialId::new("user.p", "m").unwrap(),
                revision: 1,
                name: "Generated".into(),
                category: Category::Other,
                family: None,
                condition: None,
                description: None,
                density: Density::from_canonical(rho),
                mechanical: Some(Mechanical {
                    elastic,
                    strength: Strength::default(),
                    elongation_at_break: elong.map(Directional::iso),
                    hardness: None,
                    plasticity: None,
                }),
                thermal: None,
                am: None,
                appearance: None,
                tags: vec![],
                provenance: Default::default(),
            };
            if let Some((xy, z)) = strength {
                m.mechanical.as_mut().unwrap().strength.ultimate_tensile = Some(Directional {
                    xy: Pressure::from_canonical(xy),
                    z: Some(Pressure::from_canonical(z)),
                });
            }
            m
        })
}

proptest! {
    #[test]
    fn prop_toml_and_json_round_trips(m in arb_material()) {
        prop_assert!(m.validate().is_ok());
        let json = serde_json::to_string(&m).unwrap();
        prop_assert_eq!(serde_json::from_str::<Material>(&json).unwrap(), m.clone());
        let mut lib = Library::new(LibraryMeta {
            id: "user.p".into(), name: "p".into(), version: "1".into(),
            data_licence: None, description: None,
        });
        lib.upsert_material(m.clone()).unwrap();
        let t1 = lib.to_toml_string();
        let back = Library::from_toml_str(&t1, "p.toml").unwrap();
        prop_assert_eq!(back.materials().next().unwrap().as_ref(), &m);
        prop_assert_eq!(back.to_toml_string(), t1);
    }
}

// ---- (c) validation rules, with the TOML key path ---------------------------------------

#[test]
fn iso_poisson_one_half_is_rejected() {
    let src = one(
        "density = \"1 g/cm^3\"\n[material.mechanical.elastic]\nmodel = \"isotropic\"\n\
         youngs = \"1 GPa\"\npoisson = \"0.5\"\n",
    );
    assert_invalid(&src, "material[0].mechanical.elastic.poisson");
}

#[test]
fn transversely_isotropic_inadmissible_is_rejected() {
    let src = one(
        "density = \"1 g/cm^3\"\n[material.mechanical.elastic]\nmodel = \"transversely_isotropic\"\n\
         e_p = \"1 GPa\"\ne_t = \"1 GPa\"\nnu_p = \"0.9\"\nnu_pt = \"0.5\"\ng_pt = \"0.4 GPa\"\n",
    );
    assert_invalid(&src, "material[0].mechanical.elastic.nu_pt");
    // The |nu_pt| < sqrt(E_p/E_t) bound too.
    let src = one(
        "density = \"1 g/cm^3\"\n[material.mechanical.elastic]\nmodel = \"transversely_isotropic\"\n\
         e_p = \"1 GPa\"\ne_t = \"4 GPa\"\nnu_p = \"0.3\"\nnu_pt = \"0.6\"\ng_pt = \"0.4 GPa\"\n",
    );
    assert_invalid(&src, "material[0].mechanical.elastic.nu_pt");
}

#[test]
fn orthotropic_nonpositive_determinant_is_rejected() {
    let src = one(
        "density = \"1 g/cm^3\"\n[material.mechanical.elastic]\nmodel = \"orthotropic\"\n\
         e = [\"1 GPa\", \"1 GPa\", \"1 GPa\"]\nnu = [\"0.9\", \"0.9\", \"0.9\"]\n\
         g = [\"0.4 GPa\", \"0.4 GPa\", \"0.4 GPa\"]\n",
    );
    assert_invalid(&src, "material[0].mechanical.elastic.nu");
}

#[test]
fn negative_modulus_is_rejected() {
    let src = one(
        "density = \"1 g/cm^3\"\n[material.mechanical.elastic]\nmodel = \"isotropic\"\n\
         youngs = \"-5 GPa\"\npoisson = \"0.3\"\n",
    );
    assert_invalid(&src, "material[0].mechanical.elastic.youngs");
}

#[test]
fn non_monotone_plasticity_table_is_rejected() {
    let src = one(
        "density = \"1 g/cm^3\"\n[material.mechanical.elastic]\nmodel = \"isotropic\"\n\
         youngs = \"1 GPa\"\npoisson = \"0.3\"\n[material.mechanical.plasticity]\nmodel = \"table\"\n\
         points = [{ strain = 0.0, stress = \"10 MPa\" }, { strain = 0.1, stress = \"12 MPa\" }, \
         { strain = 0.05, stress = \"13 MPa\" }]\n",
    );
    assert_invalid(&src, "material[0].mechanical.plasticity.points[2].strain");
    // Must start at 0.
    let src = one(
        "density = \"1 g/cm^3\"\n[material.mechanical.elastic]\nmodel = \"isotropic\"\n\
         youngs = \"1 GPa\"\npoisson = \"0.3\"\n[material.mechanical.plasticity]\nmodel = \"table\"\n\
         points = [{ strain = 0.01, stress = \"10 MPa\" }]\n",
    );
    assert_invalid(&src, "material[0].mechanical.plasticity.points");
}

#[test]
fn z_value_with_xzy_specimen_is_rejected() {
    let src = one(
        "default_source = \"s\"\ndensity = \"1 g/cm^3\"\n[material.mechanical.elastic]\n\
         model = \"isotropic\"\nyoungs = \"1 GPa\"\npoisson = \"0.3\"\n\
         [material.mechanical.strength]\nultimate_tensile.xy = \"40 MPa\"\n\
         ultimate_tensile.z = { v = \"35 MPa\", spec = \"XZY\" }\n",
    );
    match load_err(&src) {
        e @ MaterialError::Invalid { .. } => {
            let MaterialError::Invalid {
                property, reason, ..
            } = &e
            else {
                unreachable!()
            };
            assert_eq!(
                property,
                "material[0].mechanical.strength.ultimate_tensile.z"
            );
            assert!(reason.contains("Z**") && reason.contains("XZY"));
            use amg_core::Diagnostic;
            assert!(e.hint().unwrap().contains("Z**"));
        }
        e => panic!("{e:?}"),
    }
    // A Z specimen is fine.
    let ok = src.replace("XZY", "ZXY");
    Library::from_toml_str(&ok, "t.toml").expect("ZXY fills a z slot");
}

#[test]
fn other_range_checks() {
    let base = |extra: &str| one(&format!("density = \"1 g/cm^3\"\n{extra}"));
    assert_invalid(&one("density = \"-1 g/cm^3\"\n"), "material[0].density");
    assert_invalid(
        &base(
            "[material.am]\nprocess = \"material_extrusion\"\nnozzle_temp = { min = \"250 degC\", max = \"200 degC\" }\n",
        ),
        "material[0].am.nozzle_temp",
    );
    assert_invalid(
        &base(
            "[material.am]\nprocess = \"material_extrusion\"\ncost = { per_kg = 5.0, currency = \"eur\" }\n",
        ),
        "material[0].am.cost.currency",
    );
    assert_invalid(
        &base(
            "[material.mechanical]\nhardness = { scale = \"shore_a\", v = 120 }\n[material.mechanical.elastic]\nmodel = \"isotropic\"\nyoungs = \"1 GPa\"\npoisson = \"0.3\"\n",
        ),
        "material[0].mechanical.hardness",
    );
}

#[test]
fn unknown_keys_and_structure_errors_name_the_path() {
    assert_invalid(
        &one("density = \"1 g/cm^3\"\ncolour = \"red\"\n"),
        "material[0].colour",
    );
    assert_invalid(
        &one("category = \"nonsense\"\ndensity = \"1 g/cm^3\"\n")
            .replace("category = \"polymer\"\n", ""),
        "material[0].category",
    );
}

// ---- (d) quantity errors ----------------------------------------------------------------

#[test]
fn wrong_dimension_names_the_key_path() {
    match load_err(&one("density = \"1.24 MPa\"\n")) {
        MaterialError::Dimension {
            path,
            expected,
            found,
            origin,
        } => {
            assert_eq!(path, "material[0].density");
            assert_eq!(expected, Dimension::DENSITY);
            assert_eq!(found, Dimension::PRESSURE);
            assert_eq!(origin, "t.toml");
        }
        e => panic!("{e:?}"),
    }
}

#[test]
fn bare_numbers_need_a_unit() {
    for text in [
        "density = \"1.24\"\n",
        "density = 1.24\n",
        "density = { v = \"1.24\" }\n",
    ] {
        match load_err(&one(text)) {
            MaterialError::MissingUnit { path, .. } => assert_eq!(path, "material[0].density"),
            e => panic!("{text}: {e:?}"),
        }
    }
    // Dimensionless values take a bare number.
    let ok = one(
        "density = \"1 g/cm^3\"\n[material.mechanical.elastic]\nmodel = \"isotropic\"\n\
         youngs = \"1 GPa\"\npoisson = 0.3\n",
    );
    Library::from_toml_str(&ok, "t.toml").expect("bare nu");
}

#[test]
fn unparsable_quantity_is_a_quantity_error() {
    match load_err(&one("density = \"1.24 furlongs\"\n")) {
        MaterialError::Quantity { path, .. } => assert_eq!(path, "material[0].density"),
        e => panic!("{e:?}"),
    }
}

// ---- (e) sources and versions -----------------------------------------------------------

#[test]
fn unknown_source_is_reported() {
    match load_err(&one("density = { v = \"1 g/cm^3\", src = \"nope\" }\n")) {
        MaterialError::UnknownSource { key, .. } => assert_eq!(key, "nope"),
        e => panic!("{e:?}"),
    }
    match load_err(&one("default_source = \"nope2\"\ndensity = \"1 g/cm^3\"\n")) {
        MaterialError::UnknownSource { key, .. } => assert_eq!(key, "nope2"),
        e => panic!("{e:?}"),
    }
}

#[test]
fn newer_format_version_is_too_new() {
    let src = one("density = \"1 g/cm^3\"\n").replace("format_version = 1", "format_version = 2");
    match load_err(&src) {
        MaterialError::TooNew {
            found, supported, ..
        } => assert_eq!((found, supported), (2, 1)),
        e => panic!("{e:?}"),
    }
}

#[test]
fn syntax_error_has_a_span_and_duplicates_are_rejected() {
    match load_err("format_version = [") {
        MaterialError::Toml { span, .. } => assert!(span.is_some()),
        e => panic!("{e:?}"),
    }
    let dup = format!(
        "{}[[material]]\nid = \"m\"\nname = \"M2\"\ncategory = \"polymer\"\ndensity = \"1 g/cm^3\"\n",
        one("density = \"1 g/cm^3\"\n")
    );
    assert!(matches!(load_err(&dup), MaterialError::Duplicate { id, .. } if id == "user.t/m"));
}

// ---- (g) Srgba8 -------------------------------------------------------------------------

#[test]
fn srgba8_hex_round_trips_for_all_colours() {
    for r in 0..=255u8 {
        for g in 0..=255u8 {
            for b in 0..=255u8 {
                let c = Srgba8([r, g, b, 0xFF]);
                let hex = c.to_hex();
                assert_eq!(Srgba8::from_hex(&hex).unwrap(), c);
                assert_eq!(
                    Srgba8::from_hex(&hex[..7]).unwrap(),
                    c,
                    "#RRGGBB means alpha FF"
                );
            }
        }
    }
    assert!(Srgba8::from_hex("#12345").is_err());
    assert!(Srgba8::from_hex("123456").is_err());
    assert!(Srgba8::from_hex("#12345G").is_err());
    assert_eq!(Srgba8::from_hex("#aBcDeF12").unwrap().to_hex(), "#ABCDEF12");
}

#[test]
fn srgb_linear_srgb_is_identity_for_all_levels() {
    for v in 0..=255u8 {
        let c = Srgba8([v, v, v, v]);
        assert_eq!(Srgba8::from_linear_f32(c.to_linear_f32()), c, "level {v}");
    }
    let l = Srgba8([255, 0, 128, 255]).to_linear_f32();
    assert!((l[0] - 1.0).abs() < 1e-6 && l[1] == 0.0 && (l[2] - 0.2158605).abs() < 1e-5);
    assert!((Srgba8([0, 0, 0, 128]).to_linear_f32()[3] - 128.0 / 255.0).abs() < 1e-6);
}

#[test]
fn srgba8_serde_is_the_hex_string() {
    let c = Srgba8([1, 2, 3, 4]);
    assert_eq!(serde_json::to_string(&c).unwrap(), "\"#01020304\"");
    assert_eq!(
        serde_json::from_str::<Srgba8>("\"#010203\"").unwrap(),
        Srgba8([1, 2, 3, 255])
    );
}

// ---- ids, material helpers, library editing ---------------------------------------------

#[test]
fn ids_parse_and_validate() {
    let id = MaterialId::parse("amg.builtin/pla.fff").unwrap();
    assert_eq!(
        (id.library(), id.local(), id.as_str()),
        ("amg.builtin", "pla.fff", "amg.builtin/pla.fff")
    );
    assert_eq!(id, MaterialId::new("amg.builtin", "pla.fff").unwrap());
    for bad in ["", "pla", "a/b/c", "A/b", "a/", "/b", "a b/c"] {
        assert!(MaterialId::parse(bad).is_err(), "{bad:?}");
    }
    assert_eq!(
        serde_json::to_string(&id).unwrap(),
        "\"amg.builtin/pla.fff\""
    );
    assert!(serde_json::from_str::<MaterialId>("\"nope\"").is_err());
    assert_eq!(AppearanceId::parse("doc/red").unwrap().library(), "doc");
    assert_eq!(default_material_id().as_str(), "amg.builtin/pla.fff");
}

#[test]
fn material_helpers() {
    let lib = full();
    let iso = lib
        .material(&MaterialId::parse("user.fixture/iso.full").unwrap())
        .unwrap();
    let mass = iso.mass_of(Volume::cm3(10.0));
    assert!((mass.to_g() - 12.7).abs() < 1e-9);
    // Hash covers values, not provenance, id or revision.
    let mut c = (**iso).clone();
    let h = c.content_hash();
    c.revision = 9;
    c.provenance.default_source = None;
    c.id = MaterialId::parse("doc/x").unwrap();
    assert_eq!(c.content_hash(), h);
    c.name.push('!');
    assert_ne!(c.content_hash(), h);
}

#[test]
fn elastic_helpers() {
    let lib = full();
    let get = |l: &str| {
        lib.material(&MaterialId::new("user.fixture", l).unwrap())
            .unwrap()
            .clone()
    };
    let e_iso = Elastic::Isotropic {
        youngs: Pressure::mpa(2000.0),
        poisson: 0.25,
    };
    assert_eq!(e_iso.as_isotropic(), Some((Pressure::mpa(2000.0), 0.25)));
    let ec = e_iso.engineering_constants();
    assert!((ec.g[0].get() - 800.0).abs() < 1e-9 && ec.e[2].get() == 2000.0);
    let ti = get("ti.fff");
    let e = ti.elastic().unwrap();
    assert!(e.as_isotropic().is_none() && !e.is_isotropic());
    let (e_cons, nu) = e.isotropic_equivalent(IsoPolicy::Conservative);
    assert_eq!((e_cons.get(), nu), (4200.0, 0.3));
    let (e_in, _) = e.isotropic_equivalent(IsoPolicy::InPlane);
    assert_eq!(e_in.get(), 6000.0);
    let ec = e.engineering_constants();
    assert_eq!(
        (ec.e[0].get(), ec.e[2].get(), ec.nu[1], ec.g[1].get()),
        (6000.0, 4200.0, 0.35, 1400.0)
    );
    assert!((ec.g[0].get() - 6000.0 / 2.6).abs() < 1e-9);
    let ortho = get("ortho.metal");
    let (e_c, _) = ortho
        .elastic()
        .unwrap()
        .isotropic_equivalent(IsoPolicy::Conservative);
    assert_eq!(e_c.get(), 190_000.0);
    assert!(ortho.elastic().unwrap().validate().is_ok());
    let bad = Elastic::Isotropic {
        youngs: Pressure::mpa(1.0),
        poisson: 0.5,
    };
    assert!(bad.validate().unwrap_err().contains("poisson"));
}

#[test]
fn directional_helpers() {
    let d = Directional {
        xy: 5.0,
        z: Some(3.0),
    };
    assert_eq!((d.z_or_xy(), d.min()), (3.0, 3.0));
    let i = Directional::iso(7.0);
    assert_eq!((i.z_or_xy(), i.min(), i.z), (7.0, 7.0, None));
}

fn meta(id: &str) -> LibraryMeta {
    LibraryMeta {
        id: id.into(),
        name: id.into(),
        version: "1".into(),
        data_licence: None,
        description: None,
    }
}

#[test]
fn upsert_rekeys_and_bumps_revision_on_change_only() {
    let mut lib = Library::new(meta("user.edit"));
    let mut m = (**builtin().material(&default_material_id()).unwrap()).clone();
    let stored = lib.upsert_material(m.clone()).unwrap().clone();
    assert_eq!(stored.id.as_str(), "user.edit/pla.fff");
    assert_eq!(stored.revision, 1);
    assert!(
        lib.source(&"bambu-pla-basic-tds-v2".into_key()).is_some(),
        "sources copied in"
    );
    // Unchanged upsert: same revision.
    assert_eq!(lib.upsert_material(m.clone()).unwrap().revision, 1);
    // Changed value: +1.
    m.density = Density::g_per_cm3(1.25);
    assert_eq!(lib.upsert_material(m.clone()).unwrap().revision, 2);
    assert_eq!(lib.upsert_material(m.clone()).unwrap().revision, 2);
    // Invalid is refused and leaves the library alone.
    m.density = Density::g_per_cm3(-1.0);
    assert!(lib.upsert_material(m).is_err());
    assert_eq!(lib.materials().next().unwrap().revision, 2);
    assert!(lib.remove_material(&stored.id).is_some() && lib.materials().count() == 0);
}

trait IntoKey {
    fn into_key(self) -> amg_materials::SourceKey;
}
impl IntoKey for &str {
    fn into_key(self) -> amg_materials::SourceKey {
        amg_materials::SourceKey(self.to_owned())
    }
}

#[test]
fn appearance_upsert_and_validation() {
    let mut lib = Library::new(meta("user.app"));
    let id = AppearanceId::new("x", "red").unwrap();
    let a = Appearance::new(id, "Red", Srgba8([255, 0, 0, 255]));
    assert_eq!((a.metallic, a.roughness), (0.0, 1.0));
    assert_eq!(
        lib.upsert_appearance(a.clone()).unwrap().id.as_str(),
        "user.app/red"
    );
    let mut b = a.clone();
    b.roughness = 0.5;
    assert_eq!(lib.upsert_appearance(b.clone()).unwrap().revision, 2);
    assert_eq!(lib.upsert_appearance(b.clone()).unwrap().revision, 2);
    b.metallic = 1.5;
    assert!(lib.upsert_appearance(b).is_err());
}

// ---- built-in library and files ---------------------------------------------------------

#[test]
fn builtin_fixture_loads_and_is_read_only() {
    let lib = builtin();
    assert!(lib.is_read_only());
    assert_eq!(lib.meta().id, "amg.builtin");
    // MAT-05 replaced the two-material MAT-01 fixture with the 17-material v1 data.
    assert_eq!(lib.materials().count(), 17);
    let pla = lib.material(&default_material_id()).unwrap();
    assert!((pla.density.to_g_per_cm3() - 1.24).abs() < 1e-12);
    for m in lib.materials() {
        m.validate().unwrap();
        let a = m.appearance.as_ref().expect("default appearance");
        assert!(lib.appearance(a).is_some(), "{a} resolves");
        for k in m
            .provenance
            .values
            .values()
            .filter_map(|p| p.source.as_ref())
            .chain(m.provenance.default_source.iter())
        {
            assert!(lib.source(k).is_some());
        }
    }
    assert!(matches!(lib.save(), Err(MaterialError::ReadOnly(_))));
    // The embedded text is itself a fixed point.
    let again = Library::from_toml_str(&lib.to_toml_string(), "re").unwrap();
    assert_eq!(again.to_toml_string(), lib.to_toml_string());
}

fn tmp_dir(name: &str) -> std::path::PathBuf {
    let d = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("materials-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

#[test]
fn save_is_atomic_and_load_requires_a_user_library() {
    let dir = tmp_dir("save");
    let path = dir.join("fixture.toml");
    std::fs::write(&path, FULL).unwrap();
    let lib = Library::load(&path).unwrap();
    assert_eq!(lib.path(), Some(path.as_path()));
    assert!(!lib.is_read_only());
    lib.save().unwrap();
    let saved = std::fs::read_to_string(&path).unwrap();
    assert_eq!(saved, lib.to_toml_string());
    assert!(
        !dir.join("fixture.toml.tmp").exists(),
        "temp file renamed away"
    );
    assert_eq!(Library::load(&path).unwrap().to_toml_string(), saved);
    // A library without a path cannot save.
    assert!(matches!(
        Library::new(meta("user.nopath")).save(),
        Err(MaterialError::NoPath(_))
    ));
    // `amg.builtin` ids are not user libraries.
    let bad = dir.join("bad.toml");
    std::fs::write(&bad, FULL.replace("user.fixture", "amg.builtin")).unwrap();
    assert!(matches!(
        Library::load(&bad),
        Err(MaterialError::Invalid { .. })
    ));
    assert!(matches!(
        Library::load(&dir.join("missing.toml")),
        Err(MaterialError::Io(_))
    ));
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn diagnostics_have_hints() {
    use amg_core::Diagnostic;
    let e = load_err(&one("density = \"1.24\"\n"));
    assert!(e.hint().unwrap().contains("1.24 g/cm^3"));
    assert!(e.user_message().contains("needs a unit"));
    assert!(MaterialError::InvalidId("x".into()).hint().is_none());
}

#[test]
fn family_names_round_trip() {
    for f in [
        MaterialFamily::Pla,
        MaterialFamily::PaCf,
        MaterialFamily::Ti6Al4V,
        MaterialFamily::Other("x-y".into()),
    ] {
        assert_eq!(MaterialFamily::parse(&f.as_str()), Some(f.clone()));
        let j = serde_json::to_string(&f).unwrap();
        assert_eq!(serde_json::from_str::<MaterialFamily>(&j).unwrap(), f);
    }
    assert_eq!(MaterialFamily::parse("nonsense"), None);
}
