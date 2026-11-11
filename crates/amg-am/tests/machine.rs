//! AM-01: machine profiles, library and TOML.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::path::PathBuf;

use amg_am::machine::{
    BedShape, BuildVolume, FffParams, Machine, MachineKey, MachineLibrary, MachineSource,
    ProcessParams, ResinParams, SlicerKind, Technology, machine_from_toml, machine_to_toml,
};
use amg_core::glam::DVec2;
use proptest::prelude::*;

fn tmp_dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("amg-am-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

#[test]
fn five_builtins_validate() {
    let lib = MachineLibrary::builtin();
    assert_eq!(lib.len(), 5);
    for m in lib.iter() {
        m.validate().unwrap();
        assert_eq!(m.source, MachineSource::Builtin);
        assert!(m.key.0.starts_with("builtin:"));
        let again = machine_from_toml(&machine_to_toml(m)).unwrap();
        assert_eq!(&again, m);
    }
}

#[test]
fn builtin_values() {
    let lib = MachineLibrary::builtin();
    let get = |k: &str| lib.get(&MachineKey(k.into())).unwrap();
    let ke = get("builtin:creality/ender-3-v3-ke");
    assert_eq!(
        ke.volume.bed,
        BedShape::Rect {
            size: DVec2::new(220.0, 220.0)
        }
    );
    assert_eq!(ke.volume.height, 240.0);
    assert_eq!(ke.slicer, Some(SlicerKind::OrcaSlicer));
    let kcm = get("builtin:creality/ender-3-v3-ke-kcm");
    let ProcessParams::Fff(f) = &kcm.process else {
        panic!()
    };
    assert_eq!((f.filament_slots, f.nozzle_diameters.len()), (4, 1));
    let mars = get("builtin:elegoo/mars-3-pro");
    assert_eq!(
        mars.volume.bed,
        BedShape::Rect {
            size: DVec2::new(143.43, 89.6)
        }
    );
    assert_eq!(mars.volume.height, 175.0);
    let ProcessParams::Resin(r) = &mars.process else {
        panic!()
    };
    assert_eq!(r.resolution, [4098, 2560]);
    assert_eq!(mars.slicer, Some(SlicerKind::Chitubox));
    let g = get("builtin:generic/fff-220");
    assert_eq!(g.volume.aabb().max.z, 220.0);
    assert_eq!(get("builtin:generic/msla").technology, Technology::Msla);
}

#[test]
fn invalid_user_toml_warns_and_library_still_loads() {
    let dir = tmp_dir("warn");
    let good = machine_to_toml(MachineLibrary::builtin().iter().next().unwrap());
    std::fs::write(dir.join("good.toml"), &good).unwrap();
    std::fs::write(
        dir.join("bad_height.toml"),
        good.replace("height = 240.0", "height = -5.0")
            .replace("height = 220.0", "height = -5.0")
            .replace("height = 175.0", "height = -5.0"),
    )
    .unwrap();
    std::fs::write(dir.join("garbage.toml"), "this is = = not toml").unwrap();
    std::fs::write(
        dir.join("typo.toml"),
        good.replace("min_feature", "min_featur"),
    )
    .unwrap();
    std::fs::write(dir.join("ignored.txt"), "x").unwrap();

    let mut lib = MachineLibrary::builtin();
    let warnings = lib.load_user_dir(&dir);
    assert_eq!(warnings.len(), 3, "{warnings:?}");
    let by = |stem: &str| {
        warnings
            .iter()
            .find(|w| w.file.file_stem().unwrap() == stem)
            .unwrap()
    };
    assert!(
        by("bad_height").message.contains("volume.height"),
        "{}",
        by("bad_height").message
    );
    assert!(
        by("typo").message.contains("min_featur"),
        "{}",
        by("typo").message
    );
    assert!(!by("garbage").message.is_empty());
    assert!(lib.get(&MachineKey("user:good".into())).is_some());
    assert_eq!(lib.len(), 6);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn missing_user_dir_is_not_an_error() {
    let mut lib = MachineLibrary::builtin();
    assert!(
        lib.load_user_dir(&std::env::temp_dir().join("amg-am-no-such-dir-xyz"))
            .is_empty()
    );
}

#[test]
fn save_user_round_trips() {
    let dir = tmp_dir("save");
    let lib = MachineLibrary::builtin();
    let key = MachineKey("builtin:elegoo/mars-3-pro".into());
    let path = lib.save_user(&key, &dir).unwrap();
    let mut lib2 = MachineLibrary::default();
    assert!(lib2.load_user_dir(&dir).is_empty());
    let m = lib2.iter().next().unwrap();
    assert_eq!(m.key.0, "user:builtin-elegoo-mars-3-pro");
    assert_eq!(m.source, MachineSource::User { path });
    assert_eq!(m.volume, lib.get(&key).unwrap().volume);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn validation_rejects() {
    let base = MachineLibrary::builtin()
        .get(&MachineKey("builtin:creality/ender-3-v3-ke-kcm".into()))
        .unwrap()
        .clone();
    let mut m = base.clone();
    if let ProcessParams::Fff(f) = &mut m.process {
        f.filament_slots = 0;
    }
    assert!(
        m.validate()
            .unwrap_err()
            .to_string()
            .contains("filament_slots")
    );
    let mut m = base.clone();
    m.volume.bed = BedShape::Polygon {
        points: vec![DVec2::ZERO, DVec2::X],
    };
    assert!(
        m.validate()
            .unwrap_err()
            .to_string()
            .contains("volume.bed.points")
    );
    let mut m = base.clone();
    m.technology = Technology::Msla;
    assert!(m.validate().unwrap_err().to_string().contains("technology"));
    let mut m = base;
    m.volume.height = f64::NAN;
    assert!(m.validate().is_err());
}

#[test]
fn volume_geometry() {
    let mut v = BuildVolume {
        bed: BedShape::Rect {
            size: DVec2::new(200.0, 100.0),
        },
        height: 50.0,
        exclude: vec![vec![
            DVec2::new(0.0, 0.0),
            DVec2::new(20.0, 0.0),
            DVec2::new(20.0, 20.0),
            DVec2::new(0.0, 20.0),
        ]],
        slicer_origin: DVec2::ZERO,
    };
    assert_eq!(v.centre(), DVec2::new(100.0, 50.0));
    assert_eq!(v.aabb().max.z, 50.0);
    assert!(v.contains_xy(DVec2::new(100.0, 50.0), 0.0));
    assert!(
        !v.contains_xy(DVec2::new(10.0, 10.0), 0.0),
        "inside exclude"
    );
    assert!(
        v.contains_xy(DVec2::new(20.0, 10.0), 1e-9),
        "on exclude edge is allowed"
    );
    assert!(!v.contains_xy(DVec2::new(201.0, 50.0), 0.5));
    assert_eq!(v.bed_polygon().len(), 4);
    v.exclude.clear();
    v.bed = BedShape::Circle { diameter: 100.0 };
    assert_eq!(v.bed_polygon().len(), 128);
    assert_eq!(v.centre(), DVec2::splat(50.0));
    assert!(v.contains_xy(DVec2::new(50.0, 50.0), 0.0));
    assert!(
        !v.contains_xy(DVec2::new(2.0, 2.0), 0.0),
        "corner is off a round bed"
    );
    v.bed = BedShape::Polygon {
        points: vec![
            DVec2::new(0.0, 0.0),
            DVec2::new(100.0, 0.0),
            DVec2::new(100.0, 100.0),
            DVec2::new(50.0, 50.0),
            DVec2::new(0.0, 100.0),
        ],
    };
    assert!(v.contains_xy(DVec2::new(25.0, 25.0), 0.0));
    assert!(
        !v.contains_xy(DVec2::new(50.0, 90.0), 0.0),
        "inside the notch"
    );
}

fn f(lo: f64, hi: f64) -> impl Strategy<Value = f64> {
    lo..hi
}

fn pt() -> impl Strategy<Value = DVec2> {
    (f(-500.0, 500.0), f(-500.0, 500.0)).prop_map(|(x, y)| DVec2::new(x, y))
}

fn arb_bed() -> impl Strategy<Value = BedShape> {
    prop_oneof![
        (f(1.0, 600.0), f(1.0, 600.0)).prop_map(|(x, y)| BedShape::Rect {
            size: DVec2::new(x, y)
        }),
        f(1.0, 600.0).prop_map(|diameter| BedShape::Circle { diameter }),
        // A triangle with area > 0 by construction.
        (f(1.0, 300.0), f(1.0, 300.0), f(1.0, 300.0)).prop_map(|(a, b, c)| BedShape::Polygon {
            points: vec![DVec2::ZERO, DVec2::new(a, 0.0), DVec2::new(b.min(a), c)],
        }),
    ]
}

fn arb_machine() -> impl Strategy<Value = Machine> {
    let volume = (
        arb_bed(),
        f(1.0, 600.0),
        prop::collection::vec(prop::collection::vec(pt(), 3..6), 0..3),
        pt(),
    )
        .prop_map(|(bed, height, exclude, slicer_origin)| BuildVolume {
            bed,
            height,
            exclude,
            slicer_origin,
        });
    let fff = (
        prop::collection::vec(f(0.1, 1.2), 1..4),
        0u8..4,
        f(0.05, 0.4),
        f(0.05, 0.4),
        (f(0.01, 0.2), f(0.2, 0.6)),
    )
        .prop_map(|(nozzles, extra, lh, fl, range)| {
            ProcessParams::Fff(FffParams {
                filament_slots: nozzles.len() as u8 + extra,
                nozzle_diameters: nozzles,
                layer_height: lh,
                first_layer_height: fl,
                layer_height_range: range,
            })
        });
    let resin = (
        1u32..9000,
        1u32..6000,
        f(0.005, 0.1),
        f(0.005, 0.1),
        f(0.005, 0.2),
        f(0.0, 10.0),
    )
        .prop_map(|(rx, ry, px, py, lh, el)| {
            ProcessParams::Resin(ResinParams {
                resolution: [rx, ry],
                pixel_size: DVec2::new(px, py),
                layer_height: lh,
                default_elevation: el,
            })
        });
    let powder = f(0.01, 0.2).prop_map(|layer_height| ProcessParams::Powder { layer_height });
    let process = prop_oneof![
        fff.prop_map(|p| (Technology::Fff, p)),
        (
            prop_oneof![Just(Technology::Msla), Just(Technology::Sla)],
            resin
        )
            .prop_map(|(t, p)| (t, p)),
        (
            prop_oneof![Just(Technology::Sls), Just(Technology::Mpbf)],
            powder
        )
            .prop_map(|(t, p)| (t, p)),
    ];
    let slicer = prop::option::of(prop_oneof![
        Just(SlicerKind::OrcaSlicer),
        Just(SlicerKind::PrusaSlicer),
        Just(SlicerKind::BambuStudio),
        Just(SlicerKind::Chitubox),
        Just(SlicerKind::Custom),
    ]);
    let source = prop_oneof![
        Just(MachineSource::Builtin),
        "[a-z]{1,8}".prop_map(|s| MachineSource::User {
            path: PathBuf::from(s)
        }),
        (
            "[A-Za-z ]{1,8}",
            "[a-z]{1,8}",
            prop::collection::vec("[A-Za-z]{1,6}", 0..3)
        )
            .prop_map(|(preset, file, chain)| MachineSource::Orca {
                preset,
                file: PathBuf::from(file),
                chain
            }),
    ];
    (
        "[a-z]{1,6}:[a-z ]{1,10}",
        "[A-Za-z0-9][A-Za-z0-9 ]{0,11}",
        prop::option::of("[A-Za-z ]{1,8}"),
        prop::option::of("[A-Za-z0-9 ]{1,8}"),
        volume,
        process,
        f(0.001, 5.0),
        f(0.01, 1.5),
        slicer,
        source,
    )
        .prop_map(
            |(
                key,
                name,
                vendor,
                model,
                volume,
                (technology, process),
                min_feature,
                angle,
                slicer,
                source,
            )| Machine {
                key: MachineKey(key),
                name,
                vendor,
                model,
                technology,
                volume,
                process,
                min_feature,
                default_critical_angle: angle,
                slicer,
                source,
            },
        )
}

proptest! {
    #[test]
    fn toml_round_trip_is_lossless(m in arb_machine()) {
        m.validate().unwrap();
        let s = machine_to_toml(&m);
        let back = machine_from_toml(&s).unwrap();
        prop_assert_eq!(&back, &m);
        prop_assert_eq!(machine_to_toml(&back), s);
    }
}
