//! AM-10: importing slicer profiles. Fixtures in `testdata/am/profiles/` (values only); other
//! layouts are written into temporary folders. No test writes into a slicer folder.

// Imported values are parsed decimals, so they compare exactly.
#![allow(clippy::float_cmp, clippy::unwrap_used)]

use std::path::{Path, PathBuf};

use amg_am::handoff::detect_slicers;
use amg_am::import::{
    ImportResult, SlicerDataDirs, import_chitubox, import_orca, import_prusa,
    orca_process_critical_angle,
};
use amg_am::machine::{
    BedShape, Machine, MachineKey, MachineLibrary, MachineSource, ProcessParams, SlicerKind,
    Technology,
};
use amg_core::glam::DVec2;

const KCM: &str = "orca:Creality Ender-3 V3 KE 0.4 nozzle - Connected KCM";
const KE: &str = "orca:Creality Ender-3 V3 KE 0.4 nozzle";

fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/am/profiles")
}

fn get<'a>(r: &'a ImportResult, key: &str) -> &'a Machine {
    r.machines
        .iter()
        .find(|m| m.key.0 == key)
        .unwrap_or_else(|| panic!("no {key}: {:?} / {:?}", keys(r), r.warnings))
}

fn keys(r: &ImportResult) -> Vec<&str> {
    r.machines.iter().map(|m| m.key.0.as_str()).collect()
}

fn size(m: &Machine) -> DVec2 {
    let BedShape::Rect { size } = m.volume.bed else {
        panic!("{:?}", m.volume.bed);
    };
    size
}

fn fff(m: &Machine) -> &amg_am::machine::FffParams {
    let ProcessParams::Fff(f) = &m.process else {
        panic!("{:?}", m.process);
    };
    f
}

fn write(dir: &Path, rel: &str, text: &str) {
    let p = dir.join(rel);
    std::fs::create_dir_all(p.parent().unwrap()).unwrap();
    std::fs::write(p, text).unwrap();
}

// ---- Done-when 1 ----------------------------------------------------------------------------

#[test]
fn the_user_kcm_preset_resolves_through_user_system_and_install_roots() {
    let data = fixtures().join("orca/data");
    let install = fixtures().join("orca/install");
    let r = import_orca(&[data], &[install]);
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    let m = get(&r, KCM);
    assert_eq!(size(m), DVec2::new(220.0, 220.0));
    assert_eq!(m.volume.height, 240.0, "the user value, not 245 or 250");
    assert_eq!(fff(m).filament_slots, 4);
    assert_eq!(fff(m).layer_height_range, (0.08, 0.32));
    assert_eq!(m.min_feature, 0.4);
    assert!(m.volume.exclude.is_empty(), "[\"0x0\"] is no area");
    assert_eq!(m.technology, Technology::Fff);
    assert_eq!(m.vendor.as_deref(), Some("Creality"));
    assert_eq!(m.model.as_deref(), Some("Creality Ender-3 V3 KE"));
    assert_eq!(m.slicer, Some(SlicerKind::OrcaSlicer));
    let MachineSource::Orca {
        preset,
        chain,
        file,
    } = &m.source
    else {
        panic!("{:?}", m.source);
    };
    assert_eq!(preset, &m.name);
    assert!(file.ends_with("Creality Ender-3 V3 KE 0.4 nozzle - Connected KCM.json"));
    assert_eq!(
        chain,
        &[
            "Creality Ender-3 V3 KE 0.4 nozzle - Connected KCM",
            "Creality Ender-3 V3 KE 0.4 nozzle",
            "fdm_creality_common",
            "fdm_machine_common",
        ]
    );
    let system = get(&r, KE);
    assert_eq!(system.volume.height, 245.0);
    assert_eq!(fff(system).filament_slots, 1);
}

#[test]
fn a_user_bed_override_replaces_the_system_bed() {
    let r = import_orca(
        &[fixtures().join("orca-narrow"), fixtures().join("orca/data")],
        &[fixtures().join("orca/install")],
    );
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    // The first data folder's preset wins.
    let m = get(&r, KCM);
    assert_eq!(size(m), DVec2::new(200.0, 220.0));
    assert_eq!(m.volume.height, 240.0);
    assert_eq!(fff(m).filament_slots, 4);
}

// ---- Done-when 2 ----------------------------------------------------------------------------

#[test]
fn abstract_presets_and_machine_models_are_not_listed() {
    let r = import_orca(
        &[fixtures().join("orca/data")],
        &[fixtures().join("orca/install")],
    );
    assert_eq!(keys(&r), [KE, KCM]);
}

#[test]
fn cycles_missing_parents_and_deep_chains_give_inherits_warnings() {
    let tmp = tempfile::tempdir().unwrap();
    let user = "user/default/machine";
    let preset = |name: &str, parent: &str| {
        format!(
            r#"{{"name":"{name}","inherits":"{parent}","printable_area":["0x0","10x0","10x10","0x10"],
                "printable_height":"10","nozzle_diameter":["0.4"]}}"#
        )
    };
    for (name, parent) in [("A", "B"), ("B", "A"), ("Self", "Self"), ("Orphan", "Gone")] {
        write(
            tmp.path(),
            &format!("{user}/{name}.json"),
            &preset(name, parent),
        );
    }
    for i in 0..20 {
        write(
            tmp.path(),
            &format!("{user}/L{i:02}.json"),
            &preset(&format!("L{i:02}"), &format!("L{:02}", i + 1)),
        );
    }
    write(tmp.path(), &format!("{user}/L20.json"), &preset("L20", ""));
    write(
        tmp.path(),
        &format!("{user}/Good.json"),
        &preset("Good", ""),
    );
    write(tmp.path(), &format!("{user}/Broken.json"), "{ not json");
    let r = import_orca(&[tmp.path().to_owned()], &[]);

    // L00..L03 need more than 16 steps to reach L20; L04 needs exactly 16.
    let listed: Vec<String> = (4..=20).map(|i| format!("orca:L{i:02}")).collect();
    let mut expected = vec!["orca:Good".to_owned()];
    expected.extend(listed);
    assert_eq!(keys(&r), expected);

    let message = |file: &str| {
        let w: Vec<&str> = r
            .warnings
            .iter()
            .filter(|w| w.file.file_name().is_some_and(|n| n == file))
            .map(|w| w.message.as_str())
            .collect();
        assert_eq!(w.len(), 1, "{file}: {:?}", r.warnings);
        w[0].to_owned()
    };
    for file in ["A.json", "B.json", "Self.json"] {
        let m = message(file);
        assert!(m.starts_with("profile inheritance of `"), "{m}");
        assert!(m.contains("cycle"), "{m}");
    }
    let m = message("Orphan.json");
    assert!(
        m.starts_with("profile inheritance of `Orphan` is broken") && m.contains("`Gone`"),
        "{m}"
    );
    assert!(message("L00.json").contains("more than 16 levels"));
    assert!(message("Broken.json").starts_with("cannot read profile"));
    assert_eq!(r.warnings.len(), 9, "{:?}", r.warnings);
}

#[test]
fn a_user_preset_may_inherit_a_system_preset_of_its_own_name() {
    let tmp = tempfile::tempdir().unwrap();
    write(
        tmp.path(),
        "user/default/machine/X.json",
        r#"{"name":"X","inherits":"X","printable_height":"20"}"#,
    );
    write(
        tmp.path(),
        "system/V/machine/X.json",
        r#"{"name":"X","instantiation":"true","printable_area":["-5x-5","5x-5","5x5","-5x5"],
            "printable_height":"10","nozzle_diameter":["0.4"]}"#,
    );
    let r = import_orca(&[tmp.path().to_owned()], &[]);
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    assert_eq!(keys(&r), ["orca:X"]);
    let m = get(&r, "orca:X");
    assert_eq!(m.volume.height, 20.0);
    assert_eq!(m.volume.slicer_origin, DVec2::new(5.0, 5.0));
    assert_eq!(size(m), DVec2::new(10.0, 10.0));
}

#[test]
fn presets_without_a_usable_volume_give_a_warning() {
    let tmp = tempfile::tempdir().unwrap();
    write(
        tmp.path(),
        "user/default/machine/NoBed.json",
        r#"{"name":"NoBed","printable_height":"10","nozzle_diameter":["0.4"]}"#,
    );
    let r = import_orca(&[tmp.path().to_owned()], &[]);
    assert!(r.machines.is_empty());
    assert!(
        r.warnings[0].message.contains("printable_area"),
        "{:?}",
        r.warnings
    );
}

// ---- Done-when 3 ----------------------------------------------------------------------------

#[test]
fn the_chitubox_fixture_gives_the_mars_3_pro() {
    let r = import_chitubox(&[fixtures().join("chitubox")]);
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    assert_eq!(keys(&r), ["chitubox:ELEGOO Mars 3 Pro"]);
    let m = &r.machines[0];
    assert_eq!(size(m), DVec2::new(143.43, 89.6));
    assert_eq!(m.volume.height, 175.0);
    let ProcessParams::Resin(p) = &m.process else {
        panic!("{:?}", m.process);
    };
    assert_eq!(p.resolution, [4098, 2560]);
    assert!(p.pixel_size.abs_diff_eq(DVec2::splat(0.035), 1e-6), "{p:?}");
    assert_eq!(p.layer_height, 0.05);
    assert_eq!(m.technology, Technology::Msla);
    assert_eq!(m.vendor.as_deref(), Some("Elegoo"));
    assert_eq!(m.slicer, Some(SlicerKind::Chitubox));
    assert!(matches!(m.source, MachineSource::Chitubox { .. }));
}

#[test]
fn chitubox_duplicates_are_numbered_and_bad_files_warn() {
    let tmp = tempfile::tempdir().unwrap();
    let cfg = |height: u32| {
        format!(
            r#"{{"printerinfo":{{"machinetype":{{"currentvalue":"P"}},
            "machinewidth":{{"currentvalue":100}},"machinedepth":{{"currentvalue":50}},
            "machineheight":{{"currentvalue":{height}}},"resolutionx":{{"currentvalue":1000}},
            "resolutiony":{{"currentvalue":500}}}}}}"#
        )
    };
    write(tmp.path(), "V/a.cfgx", &cfg(100));
    write(tmp.path(), "V/b.cfgx", &cfg(100));
    write(tmp.path(), "V/c.cfgx", &cfg(120));
    write(tmp.path(), "V/d.cfgx", r#"{"printerinfo":{}}"#);
    let r = import_chitubox(&[tmp.path().to_owned()]);
    assert_eq!(keys(&r), ["chitubox:P", "chitubox:P (2)"]);
    assert_eq!(r.machines[1].volume.height, 120.0);
    assert_eq!(r.warnings.len(), 1, "{:?}", r.warnings);
    assert!(r.warnings[0].file.ends_with("d.cfgx"));
}

// ---- Done-when 4 ----------------------------------------------------------------------------

#[test]
fn the_prusa_fixture_parses_bed_shape_and_height() {
    let r = import_prusa(&[fixtures().join("prusa")]);
    assert_eq!(
        keys(&r),
        [
            "prusa:Example MK 0.4 nozzle",
            "prusa:Example MK MMU",
            "prusa:Twin",
        ]
    );
    let m = get(&r, "prusa:Example MK 0.4 nozzle");
    assert_eq!(size(m), DVec2::new(250.0, 210.0));
    assert_eq!(m.volume.height, 210.0);
    assert_eq!(fff(m).filament_slots, 1);
    assert_eq!(fff(m).layer_height_range.0, 0.07);
    assert!(
        (fff(m).layer_height_range.1 - 0.3).abs() < 1e-12,
        "0 = automatic"
    );
    assert_eq!(m.vendor.as_deref(), Some("Example"));
    assert_eq!(m.model.as_deref(), Some("EXMK"));
    assert_eq!(m.slicer, Some(SlicerKind::PrusaSlicer));

    // Two parents: the later one wins; one nozzle fed by five filaments.
    let mmu = get(&r, "prusa:Example MK MMU");
    assert_eq!(fff(mmu).filament_slots, 5);
    assert_eq!(fff(mmu).nozzle_diameters, [0.4]);
    assert_eq!(mmu.volume.height, 210.0);

    let twin = get(&r, "prusa:Twin");
    assert_eq!(twin.volume.height, 180.0);
    assert_eq!(fff(twin).nozzle_diameters, [0.4, 0.4]);

    // The resin preset is reported, not imported.
    assert_eq!(r.warnings.len(), 1, "{:?}", r.warnings);
    assert!(r.warnings[0].message.contains("SLA"));
}

#[test]
fn broken_prusa_chains_warn_and_complete_user_presets_still_load() {
    let tmp = tempfile::tempdir().unwrap();
    let complete = "inherits = Gone\nbed_shape = 0x0,10x0,10x10,0x10\nmax_print_height = 9\n\
                    nozzle_diameter = 0.4\n";
    write(tmp.path(), "printer/Complete.ini", complete);
    write(tmp.path(), "printer/Partial.ini", "inherits = Gone\n");
    write(
        tmp.path(),
        "vendor/V.ini",
        "[printer:A]\ninherits = B\n[printer:B]\ninherits = A\n",
    );
    let r = import_prusa(&[tmp.path().to_owned()]);
    assert_eq!(keys(&r), ["prusa:Complete"]);
    assert_eq!(r.warnings.len(), 4, "{:?}", r.warnings);
    assert!(
        r.warnings
            .iter()
            .all(|w| w.message.starts_with("profile inheritance of")),
        "{:?}",
        r.warnings
    );
}

// ---- critical angle, discovery, saving ------------------------------------------------------

#[test]
fn the_orca_process_preset_gives_its_critical_angle() {
    let s = std::fs::read_to_string(
        fixtures().join("orca/data/user/default/process/0.20mm Standard.json"),
    )
    .unwrap();
    let v = serde_json::from_str(&s).unwrap();
    assert_eq!(orca_process_critical_angle(&v), Some(30f64.to_radians()));
}

#[test]
fn discover_lists_only_existing_folders() {
    let tmp = tempfile::tempdir().unwrap();
    let roaming = tmp.path().join("Roaming");
    let local = tmp.path().join("Local");
    std::fs::create_dir_all(roaming.join("OrcaSlicer")).unwrap();
    std::fs::create_dir_all(local.join("chitubox2_0/Conf/MachineConfig")).unwrap();
    // A file of that name is not a data folder.
    std::fs::write(roaming.join("PrusaSlicer"), "").unwrap();
    let env = |name: &str| match name {
        "APPDATA" => Some(roaming.clone().into_os_string()),
        "LOCALAPPDATA" => Some(local.clone().into_os_string()),
        _ => None,
    };
    let d = SlicerDataDirs::discover_with(env);
    assert_eq!(d.orca, [roaming.join("OrcaSlicer")]);
    assert!(d.prusa.is_empty());
    assert_eq!(
        d.chitubox,
        [local.join("chitubox2_0").join("Conf").join("MachineConfig")]
    );

    // XDG folders, with `~/.config` as the default.
    let home = tmp.path().join("home");
    std::fs::create_dir_all(home.join(".config/PrusaSlicer")).unwrap();
    let d = SlicerDataDirs::discover_with(|n| (n == "HOME").then(|| home.clone().into()));
    assert_eq!(d.prusa, [home.join(".config").join("PrusaSlicer")]);
    assert!(d.orca.is_empty() && d.chitubox.is_empty());
    assert_eq!(
        SlicerDataDirs::discover_with(|_| None),
        SlicerDataDirs::default()
    );
}

#[test]
fn every_imported_machine_saves_as_a_user_profile_without_replacing_files() {
    let mut machines = import_orca(
        &[fixtures().join("orca/data")],
        &[fixtures().join("orca/install")],
    )
    .machines;
    machines.extend(import_chitubox(&[fixtures().join("chitubox")]).machines);
    machines.extend(import_prusa(&[fixtures().join("prusa")]).machines);
    assert_eq!(machines.len(), 6);
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path().join("machines");
    let mut lib = MachineLibrary::default();
    for m in &machines {
        lib.insert(m.clone()).unwrap();
    }
    for m in &machines {
        let (key, path) = lib.save_user_new(&m.key, &dir).unwrap();
        let (second, other) = lib.save_user_new(&m.key, &dir).unwrap();
        assert_ne!(path, other);
        assert_eq!(second.0, format!("{}-2", key.0));
    }
    let mut back = MachineLibrary::default();
    assert!(back.load_user_dir(&dir).is_empty());
    assert_eq!(back.len(), 2 * machines.len());
    for m in &machines {
        let stem: String = m
            .key
            .0
            .chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() || c == '_' {
                    c
                } else {
                    '-'
                }
            })
            .collect();
        let copy = back.get(&MachineKey(format!("user:{stem}"))).unwrap();
        assert_eq!(copy.volume, m.volume);
        assert_eq!(copy.process, m.process);
        assert_eq!(copy.name, m.name);
        assert!(matches!(copy.source, MachineSource::User { .. }));
    }
}

#[test]
fn saving_a_long_preset_name_gives_a_short_file_name() {
    let mut m = MachineLibrary::builtin().iter().next().unwrap().clone();
    m.key = MachineKey(format!("orca:{}", "Ж".repeat(400)));
    let mut lib = MachineLibrary::default();
    lib.insert(m.clone()).unwrap();
    let tmp = tempfile::tempdir().unwrap();
    let (_, path) = lib.save_user_new(&m.key, tmp.path()).unwrap();
    assert_eq!(path.file_stem().unwrap().len(), 100);
}

// ---- Done-when 5 (the owner's machine) ------------------------------------------------------

/// Read-only run on the machine's own slicer folders. Run with
/// `AMG_AM10_OWNER=1 cargo nextest run -p amg-am --test import owner --no-capture`; it prints
/// folder paths, preset counts and build volumes only.
#[test]
fn owner_machine_discovery() {
    if std::env::var_os("AMG_AM10_OWNER").is_none() {
        eprintln!("SKIP: set AMG_AM10_OWNER=1 to read this machine's slicer folders");
        return;
    }
    let d = SlicerDataDirs::discover();
    println!("orca: {:?}", d.orca);
    println!("prusa: {:?}", d.prusa);
    println!("chitubox: {:?}", d.chitubox);
    assert!(!d.orca.is_empty() && !d.prusa.is_empty() && !d.chitubox.is_empty());
    let installs: Vec<PathBuf> = detect_slicers()
        .into_iter()
        .filter(|s| s.kind == SlicerKind::OrcaSlicer)
        .filter_map(|s| s.exe.parent().map(Path::to_owned))
        .collect();
    println!("orca installs: {installs:?}");
    for (what, r) in [
        ("orca", {
            let t = std::time::Instant::now();
            let r = import_orca(&d.orca, &installs);
            println!("orca import took {:?}", t.elapsed());
            r
        }),
        ("prusa", import_prusa(&d.prusa)),
        ("chitubox", import_chitubox(&d.chitubox)),
    ] {
        println!(
            "{what}: {} machines, {} warnings",
            r.machines.len(),
            r.warnings.len()
        );
        for m in &r.machines {
            let user = match &m.source {
                MachineSource::Orca { file, .. } => {
                    file.components().any(|c| c.as_os_str() == "user")
                }
                MachineSource::Chitubox { .. } => true,
                _ => false,
            };
            if user {
                let (lo, hi) = m.volume.bed_rect();
                let slots = match &m.process {
                    ProcessParams::Fff(f) => format!("{} slots", f.filament_slots),
                    ProcessParams::Resin(r) => format!("{:?} px", r.resolution),
                    ProcessParams::Powder { .. } => String::new(),
                };
                println!(
                    "  {}: {} x {} x {} mm, {slots}",
                    m.key.0,
                    hi.x - lo.x,
                    hi.y - lo.y,
                    m.volume.height
                );
            }
        }
        for w in r.warnings.iter().take(10) {
            println!("  warning {}: {}", w.file.display(), w.message);
        }
    }
}
