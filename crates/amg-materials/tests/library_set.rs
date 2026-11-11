//! MAT-07 Done-when (a)–(g): `LibrarySet`, atomic save, search, name matching, drift.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    reason = "test and bench code"
)]

mod common;

use std::path::PathBuf;
use std::time::Instant;

use amg_core::units::Density;
use amg_materials::{
    AmProcess, Category, Drift, Library, LibraryMeta, LibrarySet, Material, MaterialError,
    MaterialFamily, MaterialId, MaterialQuery, SavePhase, builtin, default_material_id,
};

/// A fresh empty directory under the target dir.
fn temp_dir(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("mat07-{name}"));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
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

fn mat(
    lib: &str,
    local: &str,
    name: &str,
    family: Option<MaterialFamily>,
    slicer: Option<&str>,
) -> Material {
    let mut m = common::material(1);
    m.id = MaterialId::new(lib, local).unwrap();
    m.name = name.into();
    m.family = family;
    m.am.as_mut().unwrap().slicer_type = slicer.map(str::to_owned);
    m
}

fn write_lib(dir: &std::path::Path, file: &str, id: &str, n: usize) {
    let mut lib = Library::new(meta(id));
    for i in 0..n {
        lib.upsert_material(mat(id, &format!("m{i}"), &format!("Mat {i}"), None, None))
            .unwrap();
    }
    std::fs::write(dir.join(file), lib.to_toml_string()).unwrap();
}

/// A leaked stand-in for the built-in library that also has PETG.
fn base_with_petg() -> &'static Library {
    let mut lib = Library::new(meta("amg.builtin"));
    lib.upsert_material(mat(
        "amg.builtin",
        "pla.fff",
        "PLA (FFF printed)",
        Some(MaterialFamily::Pla),
        Some("PLA"),
    ))
    .unwrap();
    lib.upsert_material(mat(
        "amg.builtin",
        "petg.fff",
        "PETG (FFF printed)",
        Some(MaterialFamily::Petg),
        Some("PETG"),
    ))
    .unwrap();
    lib.upsert_material(mat(
        "amg.builtin",
        "pacf.fff",
        "PA-CF (FFF printed)",
        Some(MaterialFamily::PaCf),
        Some("PA-CF"),
    ))
    .unwrap();
    Box::leak(Box::new(lib))
}

#[test]
fn a_load_skips_and_reports_the_broken_file() {
    let dir = temp_dir("load");
    write_lib(&dir, "one.toml", "user.one", 2);
    write_lib(&dir, "two.toml", "user.two", 1);
    write_lib(&dir, "three.toml", "user.three", 3);
    std::fs::write(dir.join("broken.toml"), "this is = = not toml").unwrap();
    std::fs::write(dir.join("notes.txt"), "ignored").unwrap();
    std::fs::write(dir.join("one.toml.tmp"), "stale temp from a crash").unwrap();

    let mut set = LibrarySet::builtin_only();
    let errors = set.load_user_dir(&dir);
    assert_eq!(errors.len(), 1, "{errors:?}");
    assert!(
        errors[0].to_string().contains("broken.toml"),
        "{}",
        errors[0]
    );
    let ids: Vec<_> = set.libraries().map(|l| l.meta().id.clone()).collect();
    assert_eq!(ids, ["amg.builtin", "user.one", "user.three", "user.two"]);

    // Reloading replaces by path; no duplicates.
    let errors = set.load_user_dir(&dir);
    assert_eq!(errors.len(), 1);
    assert_eq!(set.libraries().count(), 4);
}

#[test]
fn load_creates_a_missing_dir_and_reports_duplicate_ids() {
    let dir = temp_dir("missing").join("sub").join("libs");
    let mut set = LibrarySet::builtin_only();
    assert!(set.load_user_dir(&dir).is_empty());
    assert!(dir.is_dir());
    write_lib(&dir, "a.toml", "user.same", 1);
    write_lib(&dir, "b.toml", "user.same", 1);
    let errors = set.load_user_dir(&dir);
    assert!(
        matches!(errors.as_slice(), [MaterialError::Duplicate { id, .. }] if id == "user.same")
    );
    assert_eq!(set.libraries().count(), 2);
}

#[test]
fn create_user_library_saves_and_reloads() {
    let dir = temp_dir("create");
    let mut set = LibrarySet::builtin_only();
    let lib = set.create_user_library(&dir, "user.mine", "Mine").unwrap();
    lib.upsert_material(mat("user.mine", "x", "My PLA", None, None))
        .unwrap();
    lib.save().unwrap();
    assert!(dir.join("user.mine.toml").is_file());
    assert!(set.create_user_library(&dir, "user.mine", "Again").is_err());
    assert!(set.create_user_library(&dir, "mine", "No prefix").is_err());
    assert!(set.create_user_library(&dir, "user.Bad Id", "x").is_err());

    let mut fresh = LibrarySet::builtin_only();
    assert!(fresh.load_user_dir(&dir).is_empty());
    let id = MaterialId::parse("user.mine/x").unwrap();
    assert_eq!(fresh.material(&id).unwrap().name, "My PLA");
}

#[test]
fn b_upsert_bumps_revision_only_on_change() {
    let dir = temp_dir("rev");
    let mut set = LibrarySet::builtin_only();
    set.create_user_library(&dir, "user.rev", "Rev").unwrap();
    let lib = set.library_mut("user.rev").unwrap();
    let m = mat("user.rev", "a", "A", None, None);
    assert_eq!(lib.upsert_material(m.clone()).unwrap().revision, 1);
    assert_eq!(
        lib.upsert_material(m.clone()).unwrap().revision,
        1,
        "unchanged"
    );
    let mut edited = m.clone();
    edited.density = Density::g_per_cm3(1.5);
    assert_eq!(lib.upsert_material(edited.clone()).unwrap().revision, 2);
    assert_eq!(lib.upsert_material(edited).unwrap().revision, 2);
    let id = MaterialId::parse("user.rev/a").unwrap();
    assert!(lib.remove_material(&id).is_some());
    assert!(set.material(&id).is_none());
}

#[test]
fn c_save_crash_between_write_and_rename_keeps_the_old_file() {
    let dir = temp_dir("crash");
    let mut set = LibrarySet::builtin_only();
    let lib = set
        .create_user_library(&dir, "user.crash", "Crash")
        .unwrap();
    lib.upsert_material(mat("user.crash", "a", "A", None, None))
        .unwrap();
    lib.save().unwrap();
    let path = dir.join("user.crash.toml");
    let before = std::fs::read(&path).unwrap();

    lib.upsert_material(mat("user.crash", "b", "B", None, None))
        .unwrap();
    let mut phases = Vec::new();
    let r = lib.save_with_hook(&mut |p| {
        phases.push(p);
        Err(std::io::Error::other("simulated crash"))
    });
    assert!(matches!(r, Err(MaterialError::Io(_))));
    assert_eq!(phases, [SavePhase::TempWritten]);
    assert_eq!(std::fs::read(&path).unwrap(), before, "old file intact");
    Library::load(&path).expect("old file still loads");

    // A real crash would leave the temp file; it must not be picked up as a library.
    std::fs::write(dir.join("user.crash.toml.tmp"), "half written").unwrap();
    let mut fresh = LibrarySet::builtin_only();
    assert!(fresh.load_user_dir(&dir).is_empty());
    assert_eq!(fresh.libraries().count(), 2);

    // And a normal save afterwards works.
    lib.save().unwrap();
    assert_ne!(std::fs::read(&path).unwrap(), before);
}

#[test]
fn d_match_name_builtin_user_wins() {
    let mut set = LibrarySet::with_builtin(base_with_petg());
    let id = |s: &str| MaterialId::parse(s).unwrap();
    let pla = id("amg.builtin/pla.fff");
    assert_eq!(set.match_name("PLA").unwrap().id, pla);
    assert_eq!(set.match_name("pla").unwrap().id, pla);
    assert_eq!(
        set.match_name("PETG Basic").unwrap().id,
        id("amg.builtin/petg.fff")
    );
    assert_eq!(
        set.match_name("Generic PA-CF").unwrap().id,
        id("amg.builtin/pacf.fff")
    );
    assert_eq!(
        set.match_name("pla (fff printed)").unwrap().id,
        pla,
        "exact name"
    );
    assert!(set.match_name("Unobtainium").is_none());
    assert!(set.match_name("  ").is_none());

    let dir = temp_dir("match");
    let lib = set.create_user_library(&dir, "user.mine", "Mine").unwrap();
    lib.upsert_material(mat(
        "user.mine",
        "mypla",
        "My PLA",
        Some(MaterialFamily::Pla),
        Some("PLA"),
    ))
    .unwrap();
    lib.upsert_material(mat("user.mine", "pla2", "pla", None, None))
        .unwrap();
    // Exact name beats slicer type; user beats built-in.
    assert_eq!(set.match_name("PLA").unwrap().id, id("user.mine/pla2"));
    assert_eq!(
        set.match_name("Brand PLA+").unwrap().id,
        id("user.mine/mypla")
    );

    // The real built-in answers PLA too.
    let real = LibrarySet::builtin_only();
    assert_eq!(real.match_name("PLA").unwrap().id, default_material_id());
}

#[test]
fn e_drift_returns_all_four_variants() {
    let dir = temp_dir("drift");
    let mut set = LibrarySet::builtin_only();
    let lib = set.create_user_library(&dir, "user.d", "D").unwrap();
    let v1 = lib
        .upsert_material(mat("user.d", "a", "A", None, None))
        .unwrap()
        .as_ref()
        .clone();
    assert_eq!(set.drift(&v1), Drift::UpToDate);

    let mut local = v1.clone();
    local.density = Density::g_per_cm3(2.0);
    assert_eq!(set.drift(&local), Drift::LocallyModified);

    let mut gone = v1.clone();
    gone.id = MaterialId::parse("user.d/gone").unwrap();
    assert_eq!(set.drift(&gone), Drift::NotInLibrary);
    let mut unknown_lib = v1.clone();
    unknown_lib.id = MaterialId::parse("user.nope/a").unwrap();
    assert_eq!(set.drift(&unknown_lib), Drift::NotInLibrary);

    let mut v2 = v1.clone();
    v2.density = Density::g_per_cm3(1.9);
    set.library_mut("user.d")
        .unwrap()
        .upsert_material(v2)
        .unwrap();
    assert_eq!(
        set.drift(&v1),
        Drift::LibraryNewer {
            library_revision: 2
        }
    );
    // A copy edited AND behind the library still reports the newer library.
    assert_eq!(
        set.drift(&local),
        Drift::LibraryNewer {
            library_revision: 2
        }
    );

    // Built-in entries: unchanged copy is up to date.
    let pla = builtin().material(&default_material_id()).unwrap();
    assert_eq!(set.drift(pla), Drift::UpToDate);
}

#[test]
fn search_filters_and_orders() {
    let mut set = LibrarySet::with_builtin(base_with_petg());
    let dir = temp_dir("search");
    let lib = set.create_user_library(&dir, "user.s", "S").unwrap();
    let mut z = mat("user.s", "z", "Zeta resin", None, None);
    z.category = Category::Photopolymer;
    z.am.as_mut().unwrap().process = AmProcess::VatPhotopolymerization;
    z.tags = vec!["Tough".into()];
    lib.upsert_material(z).unwrap();
    lib.upsert_material(mat("user.s", "a", "Alpha", None, None))
        .unwrap();

    let names = |q: &MaterialQuery| -> Vec<String> {
        set.search(q).iter().map(|m| m.name.clone()).collect()
    };
    assert_eq!(
        names(&MaterialQuery::default()),
        [
            "PA-CF (FFF printed)",
            "PETG (FFF printed)",
            "PLA (FFF printed)",
            "Alpha",
            "Zeta resin"
        ]
    );
    let text = |t: &str| MaterialQuery {
        text: Some(t.into()),
        ..Default::default()
    };
    assert_eq!(names(&text("petg")), ["PETG (FFF printed)"]);
    assert_eq!(
        names(&text("TOUGH")),
        ["Zeta resin"],
        "tag, case-insensitive"
    );
    assert_eq!(names(&text("user.s/a")), ["Alpha"], "id");
    assert!(names(&text("nothing like this")).is_empty());
    let q = MaterialQuery {
        process: Some(AmProcess::VatPhotopolymerization),
        ..Default::default()
    };
    assert_eq!(names(&q), ["Zeta resin"]);
    let q = MaterialQuery {
        category: Some(Category::Photopolymer),
        ..Default::default()
    };
    assert_eq!(names(&q), ["Zeta resin"]);
    let q = MaterialQuery {
        family: Some(MaterialFamily::Petg),
        ..Default::default()
    };
    assert_eq!(names(&q), ["PETG (FFF printed)"]);
}

#[test]
fn f_search_5000_materials_budget() {
    let mut set = LibrarySet::builtin_only();
    let dir = temp_dir("perf");
    let lib = set.create_user_library(&dir, "user.gen", "Gen").unwrap();
    for i in 0..5000 {
        let mut m = common::material(i);
        m.id = MaterialId::new("user.gen", &format!("m{i}")).unwrap();
        lib.upsert_material(m).unwrap();
    }
    let queries = [
        MaterialQuery {
            text: Some("material 4242".into()),
            ..Default::default()
        },
        MaterialQuery {
            text: Some("generated".into()),
            ..Default::default()
        },
        MaterialQuery::default(),
    ];
    for q in &queries {
        let mut runs: Vec<f64> = (0..15)
            .map(|_| {
                let t = Instant::now();
                std::hint::black_box(set.search(std::hint::black_box(q)));
                t.elapsed().as_secs_f64() * 1e3
            })
            .collect();
        runs.sort_by(f64::total_cmp);
        let ms = runs[runs.len() / 2];
        println!("search {:?} over 5000: {ms:.3} ms median", q.text);
        if std::env::var_os("AMG_STRICT_TIMING").is_some_and(|v| v == "1") {
            assert!(ms < 1.0, "search took {ms:.3} ms, budget 1 ms");
        }
    }
}

#[test]
fn g_builtin_rejects_library_mut() {
    let mut set = LibrarySet::builtin_only();
    assert!(set.library_mut("amg.builtin").is_none());
    assert!(set.library_mut("user.unknown").is_none());
    assert!(builtin().is_read_only());
    assert!(matches!(builtin().save(), Err(MaterialError::ReadOnly(_))));
}
