//! Generated libraries for the perf test and the bench (`#[path]`-included).
#![allow(dead_code)]

use amg_core::units::{
    Density, Pressure, SpecificHeat, Temperature, ThermalConductivity, ThermalExpansion,
};
use amg_materials::{
    AmInfo, AmProcess, Category, Directional, Elastic, Library, LibraryMeta, Material, MaterialId,
    Mechanical, Strength, TempRange, Thermal,
};

/// A realistic FFF-like material (elastic, strength, thermal and AM blocks), index `i`.
pub fn material(i: usize) -> Material {
    let f = 1.0 + (i % 97) as f64 / 100.0;
    let mut m = Material {
        id: MaterialId::new("user.gen", &format!("m{i}")).unwrap(),
        revision: 1,
        name: format!("Generated material {i}"),
        category: Category::Polymer,
        family: None,
        condition: Some("printed, dry".into()),
        description: None,
        density: Density::g_per_cm3(1.1 * f),
        mechanical: Some(Mechanical {
            elastic: Elastic::Isotropic {
                youngs: Pressure::mpa(2000.0 * f),
                poisson: 0.35,
            },
            strength: Strength {
                yield_tensile: Some(Directional {
                    xy: Pressure::mpa(40.0 * f),
                    z: Some(Pressure::mpa(30.0 * f)),
                }),
                ultimate_tensile: Some(Directional::iso(Pressure::mpa(50.0 * f))),
                ..Strength::default()
            },
            elongation_at_break: Some(Directional::iso(0.05 * f)),
            hardness: None,
            plasticity: None,
        }),
        thermal: Some(Thermal {
            conductivity: Some(Directional::iso(ThermalConductivity::w_per_m_k(0.2 * f))),
            specific_heat: Some(SpecificHeat::j_per_kg_k(1800.0 * f)),
            expansion: Some(Directional::iso(ThermalExpansion::ppm_per_k(68.0 * f))),
            glass_transition: Some(Temperature::celsius(60.0 * f)),
            ..Thermal::default()
        }),
        am: Some(AmInfo {
            process: AmProcess::MaterialExtrusion,
            slicer_type: Some("PLA".into()),
            nozzle_temp: Some(TempRange {
                min: Temperature::celsius(190.0),
                max: Temperature::celsius(230.0),
            }),
            bed_temp: None,
            chamber_temp: None,
            shrinkage: None,
            cost: None,
        }),
        appearance: None,
        tags: vec!["generated".into()],
        provenance: Default::default(),
    };
    m.provenance.default_source = None;
    m
}

pub fn library(n: usize) -> Library {
    let mut lib = Library::new(LibraryMeta {
        id: "user.gen".into(),
        name: "Generated".into(),
        version: "1".into(),
        data_licence: None,
        description: None,
    });
    for i in 0..n {
        lib.upsert_material(material(i)).unwrap();
    }
    lib
}
