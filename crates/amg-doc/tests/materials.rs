//! MAT-06 (S17): material and appearance tables in the design, commands, events, builders,
//! `.amg` round trip, drift update and the default-material preference.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::Read;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, Instant};

use amg_core::units::Density;
use amg_doc::DocEvent::{
    AppearanceChanged, BodyGeometryChanged, BodyPropsChanged, MaterialChanged, RecomputeFinished,
    RecomputeStarted,
};
use amg_doc::stub::{AppearanceKey, MaterialKey};
use amg_doc::testing::TestBox;
use amg_doc::{
    BodyId, BodyProps, Command, CommandError, Design, DocEvent, Document, FeatureBox, FeatureId,
    SaveOptions, assign_appearance, assign_material, assign_material_by_id, assign_material_in,
    load, remove_unused_materials, save, update_from_library,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_materials::{
    Appearance, AppearanceId, Drift, Library, LibraryMeta, LibrarySet, Material, MaterialId,
    MaterialPrefs, Srgba8, builtin, default_material_id,
};

// ---- helpers ----------------------------------------------------------------------------

struct TempDir(PathBuf);

impl TempDir {
    fn new(tag: &str) -> TempDir {
        static N: AtomicU32 = AtomicU32::new(0);
        let p = std::env::temp_dir().join(format!(
            "amg-doc-mat06-{tag}-{}-{}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        ));
        let _ = fs::remove_dir_all(&p);
        fs::create_dir_all(&p).unwrap();
        TempDir(p)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn mid(s: &str) -> MaterialId {
    MaterialId::parse(s).unwrap()
}

fn builtin_mat(local: &str) -> Arc<Material> {
    Arc::clone(
        builtin()
            .material(&mid(&format!("amg.builtin/{local}")))
            .unwrap(),
    )
}

fn bx(x: f64) -> FeatureBox {
    FeatureBox::new(TestBox {
        size: [x, 1.0, 1.0],
    })
}

fn body(f: FeatureId) -> BodyId {
    BodyId { origin: f, slot: 0 }
}

/// A design with `n` box features; returns it and one body per feature.
fn design(n: usize) -> (Design, Vec<BodyId>) {
    amg_doc::testing::link();
    let mut d = Design::new("mat06");
    let root = d.root();
    let bodies = (0..n)
        .map(|i| body(d.add_feature(None, bx(1.0 + i as f64), root, None).unwrap()))
        .collect();
    (d, bodies)
}

/// Builds commands from the current design and applies them.
fn apply_with(d: &mut Design, build: impl FnOnce(&Design) -> Vec<Command>) {
    let cmds = build(d);
    apply_all(d, cmds);
}

fn apply_all(d: &mut Design, cmds: Vec<Command>) {
    for c in cmds {
        amg_doc::apply_command(d, c).unwrap();
    }
    d.validate().unwrap();
}

/// A `LibrarySet` without any library entries (no built-in either).
fn empty_set() -> LibrarySet {
    let lib = Library::new(LibraryMeta {
        id: "amg.empty".into(),
        name: "Empty".into(),
        version: "0".into(),
        data_licence: None,
        description: None,
    });
    LibrarySet::with_builtin(Box::leak(Box::new(lib)))
}

fn entries_of(path: &std::path::Path) -> BTreeMap<String, Vec<u8>> {
    let mut zip = zip::ZipArchive::new(File::open(path).unwrap()).unwrap();
    let mut out = BTreeMap::new();
    for i in 0..zip.len() {
        let mut e = zip.by_index(i).unwrap();
        let mut data = Vec::new();
        e.read_to_end(&mut data).unwrap();
        out.insert(e.name().unwrap().into_owned(), data);
    }
    out
}

// ---- (a) one transaction, one undo ------------------------------------------------------

#[test]
fn a_assign_builtin_to_three_bodies_is_one_undo_step() {
    let (d, b) = design(3);
    let mut doc = Document::from_design(d);
    let before = doc.snapshot();
    let pla = builtin_mat("pla.fff");
    let cmds = assign_material(doc.design(), &b, &pla);
    // SetMaterial + SetAppearance (default appearance) + 3 x SetBodyProps.
    assert_eq!(cmds.len(), 5, "{cmds:#?}");
    let mut tx = doc.transaction("Assign material");
    for c in cmds {
        tx.apply(c).unwrap();
    }
    tx.commit();
    assert_eq!(doc.history().stack().applied(), 1, "one undo entry");
    assert_eq!(doc.undo_label(), Some("Assign material"));

    let dsg = doc.design();
    dsg.validate().unwrap();
    assert_eq!(dsg.materials().count(), 1);
    let app_id = pla.appearance.clone().unwrap();
    assert_eq!(
        dsg.appearances().map(|a| a.id.clone()).collect::<Vec<_>>(),
        vec![app_id.clone()]
    );
    for &bb in &b {
        assert_eq!(dsg.body_material(bb).unwrap().id, pla.id);
        assert_eq!(dsg.body_appearance(bb).unwrap().id, app_id);
        assert_eq!(
            dsg.body_props(bb).material,
            Some(MaterialKey("amg.builtin/pla.fff".into()))
        );
    }
    assert_eq!(dsg.material_users(&pla.id), b);

    assert!(doc.undo());
    let after = doc.design();
    assert_eq!(after.materials().count(), 0, "table restored");
    assert_eq!(after.appearances().count(), 0);
    for &bb in &b {
        assert_eq!(after.body_props(bb), BodyProps::default(), "props restored");
    }
    assert_eq!(
        serde_json::to_string(after).unwrap(),
        serde_json::to_string(&*before).unwrap()
    );
    assert!(doc.redo());
    assert_eq!(doc.design().material_users(&pla.id), b);
}

#[test]
fn assign_is_idempotent_and_respects_explicit_appearances() {
    let (mut d, b) = design(3);
    let pla = builtin_mat("pla.fff");
    let petg = builtin_mat("petg.fff");
    // Body 2 gets an explicit appearance first.
    let red = Arc::new(Appearance::new(
        AppearanceId::parse("doc/red").unwrap(),
        "Red",
        Srgba8::from_hex("#FF0000").unwrap(),
    ));
    apply_with(&mut d, |d| assign_appearance(d, &b[2..], &red));
    apply_with(&mut d, |d| assign_material(d, &b, &pla));
    assert!(
        assign_material(&d, &b, &pla).is_empty(),
        "nothing left to do"
    );
    assert_eq!(d.body_appearance(b[2]).unwrap().id, red.id, "kept");

    // Re-assigning another material re-colours the bodies that had PLA's default appearance.
    apply_with(&mut d, |d| assign_material(d, &b, &petg));
    let petg_app = petg.appearance.clone().unwrap();
    assert_eq!(d.body_appearance(b[0]).unwrap().id, petg_app);
    assert_eq!(d.body_appearance(b[1]).unwrap().id, petg_app);
    assert_eq!(d.body_appearance(b[2]).unwrap().id, red.id);
    // Duplicate bodies give one command each.
    let (d2, b2) = design(1);
    let cmds = assign_material(&d2, &[b2[0], b2[0]], &pla);
    let props = cmds
        .iter()
        .filter(|c| matches!(c, Command::SetBodyProps { .. }))
        .count();
    assert_eq!(props, 1);
}

#[test]
fn cli_python_entry_resolves_ids_and_names() {
    let (mut d, b) = design(2);
    let libs = LibrarySet::builtin_only();
    apply_with(&mut d, |d| {
        assign_material_by_id(d, &libs, &b[..1], "amg.builtin/petg.fff").unwrap()
    });
    assert_eq!(d.body_material(b[0]).unwrap().id.local(), "petg.fff");
    apply_with(&mut d, |d| {
        assign_material_by_id(d, &libs, &b[1..], "PLA").unwrap()
    });
    assert_eq!(d.body_material(b[1]).unwrap().id, default_material_id());
    let e = assign_material_by_id(&d, &libs, &b, "unobtainium").unwrap_err();
    assert!(matches!(e, CommandError::Invalid(_)), "{e:?}");
    // Commands serialize (crash journal, macros, Python share the JSON form).
    let cmd = Command::SetMaterial {
        material: builtin_mat("pla.fff"),
    };
    let json = serde_json::to_string(&cmd).unwrap();
    let back: Command = serde_json::from_str(&json).unwrap();
    assert_eq!(serde_json::to_string(&back).unwrap(), json);
    assert_eq!(cmd.label(), "Set material");
}

#[test]
fn invalid_material_is_refused_and_changes_nothing() {
    let (mut d, _) = design(1);
    let mut bad = (*builtin_mat("pla.fff")).clone();
    bad.density = Density::g_per_cm3(-1.0);
    let before = serde_json::to_string(&d).unwrap();
    let e = amg_doc::apply_command(
        &mut d,
        Command::SetMaterial {
            material: Arc::new(bad),
        },
    )
    .unwrap_err();
    assert!(matches!(e, CommandError::Model(_)), "{e:?}");
    assert_eq!(serde_json::to_string(&d).unwrap(), before);
}

// ---- (b) .amg round trip ----------------------------------------------------------------

#[test]
fn b_save_load_save_keeps_design_json_byte_identical() {
    let (mut d, b) = design(3);
    apply_with(&mut d, |d| {
        assign_material(d, &b[..2], &builtin_mat("pla.fff"))
    });
    apply_with(&mut d, |d| {
        assign_material(d, &b[2..], &builtin_mat("ti6al4v.lpbf"))
    });
    // A document-only material with awkward floats.
    let mut own = (*builtin_mat("petg.fff")).clone();
    own.id = mid("doc/petg-dried");
    own.density = Density::g_per_cm3(0.1 + 0.2);
    apply_all(
        &mut d,
        vec![Command::SetMaterial {
            material: Arc::new(own),
        }],
    );

    let dir = TempDir::new("rt");
    let (pa, pb) = (dir.0.join("a.amg"), dir.0.join("b.amg"));
    save(&pa, &Arc::new(d.clone()), &SaveOptions::default()).unwrap();
    let loaded = load(&pa).unwrap();
    assert!(loaded.warnings.is_empty());
    save(
        &pb,
        &Arc::new(loaded.design.clone()),
        &SaveOptions::default(),
    )
    .unwrap();
    let (ea, eb) = (entries_of(&pa), entries_of(&pb));
    assert_eq!(ea["design.json"], eb["design.json"], "design.json bytes");
    let text = String::from_utf8(ea["design.json"].clone()).unwrap();
    assert!(text.contains("\"materials\""), "{text}");
    assert!(text.contains("\"appearances\""));

    // Materials are bit-exact copies of what was assigned.
    for m in d.materials() {
        assert_eq!(**loaded.design.material(&m.id).unwrap(), **m);
    }
    // Opening without any library still resolves every body material.
    let empty = empty_set();
    for &bb in &b {
        let m = loaded
            .design
            .body_material(bb)
            .expect("resolves from the document");
        assert_eq!(empty.drift(m), Drift::NotInLibrary);
        assert!(loaded.design.body_appearance(bb).is_some());
    }
}

#[test]
fn designs_without_materials_serialize_as_before() {
    let (d, _) = design(1);
    let v: serde_json::Value = serde_json::to_value(&d).unwrap();
    let obj = v.as_object().unwrap();
    assert!(!obj.contains_key("materials"));
    assert!(!obj.contains_key("appearances"));
    // A free-form key from before MAT-06 still loads and counts as unassigned.
    let (mut d, b) = design(1);
    d.set_body_props(
        b[0],
        BodyProps {
            material: Some(MaterialKey("PLA".into())),
            ..BodyProps::default()
        },
    )
    .unwrap();
    let back: Design = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
    assert!(back.body_material(b[0]).is_none());
    assert!(back.material_users(&default_material_id()).is_empty());
}

#[test]
fn load_rejects_corrupt_tables() {
    let (mut d, b) = design(1);
    apply_with(&mut d, |d| assign_material(d, &b, &builtin_mat("pla.fff")));
    let mut v = serde_json::to_value(&d).unwrap();
    // Duplicate entry.
    let mut dup = v.clone();
    let first = dup["materials"][0].clone();
    dup["materials"].as_array_mut().unwrap().push(first);
    let e = serde_json::from_value::<Design>(dup).unwrap_err();
    assert!(e.to_string().contains("duplicate id"), "{e}");
    // A value that fails validation.
    v["materials"][0]["revision"] = serde_json::json!(0);
    assert!(serde_json::from_value::<Design>(v).is_err());
}

// ---- (c) removal --------------------------------------------------------------------------

#[test]
fn c_remove_referenced_is_in_use_and_remove_unused_is_exact() {
    let (mut d, b) = design(3);
    let pla = builtin_mat("pla.fff");
    let petg = builtin_mat("petg.fff");
    let ti = builtin_mat("ti6al4v.lpbf");
    apply_with(&mut d, |d| assign_material(d, &b[..2], &pla));
    apply_with(&mut d, |d| assign_material(d, &b[2..], &petg));
    // Unreferenced copies: Ti64 and a stray appearance.
    let stray = Arc::new(Appearance::new(
        AppearanceId::parse("doc/stray").unwrap(),
        "Stray",
        Srgba8::from_hex("#00FF00").unwrap(),
    ));
    apply_all(
        &mut d,
        vec![
            Command::SetMaterial {
                material: Arc::clone(&ti),
            },
            Command::SetAppearance {
                appearance: Arc::clone(&stray),
            },
        ],
    );

    let e =
        amg_doc::apply_command(&mut d, Command::RemoveMaterial { id: pla.id.clone() }).unwrap_err();
    match e {
        CommandError::InUse { what, by, .. } => {
            assert_eq!(what, "material");
            assert_eq!(by, vec![b[0].origin.0, b[1].origin.0]);
        }
        other => panic!("expected InUse, got {other:?}"),
    }
    let used_app = d.body_props(b[0]).appearance.unwrap().id().unwrap();
    let e = amg_doc::apply_command(&mut d, Command::RemoveAppearance { id: used_app }).unwrap_err();
    assert!(
        matches!(
            e,
            CommandError::InUse {
                what: "appearance",
                ..
            }
        ),
        "{e:?}"
    );

    let cmds = remove_unused_materials(&d);
    let removed: Vec<String> = cmds
        .iter()
        .map(|c| match c {
            Command::RemoveMaterial { id } => format!("m:{id}"),
            Command::RemoveAppearance { id } => format!("a:{id}"),
            other => panic!("unexpected {other:?}"),
        })
        .collect();
    // Ti64's default appearance was never copied (SetMaterial alone doesn't copy it).
    assert_eq!(
        removed,
        vec![format!("m:{}", ti.id), "a:doc/stray".to_owned()]
    );
    apply_all(&mut d, cmds);
    assert_eq!(d.materials().count(), 2);
    assert_eq!(d.appearances().count(), 2);
    assert!(remove_unused_materials(&d).is_empty());

    // Unassign everything: both materials and their appearances go.
    for &bb in &b {
        d.set_body_props(bb, BodyProps::default()).unwrap();
    }
    apply_with(&mut d, remove_unused_materials);
    assert_eq!(d.materials().count() + d.appearances().count(), 0);
    let e =
        amg_doc::apply_command(&mut d, Command::RemoveMaterial { id: pla.id.clone() }).unwrap_err();
    assert!(matches!(e, CommandError::Model(_)), "{e:?}");
}

// ---- (d) drift and update from library --------------------------------------------------

#[test]
fn d_library_revision_bump_gives_library_newer_and_update_is_undoable() {
    let dir = TempDir::new("drift");
    let mut libs = LibrarySet::builtin_only();
    libs.create_user_library(&dir.0, "user.shop", "Shop")
        .unwrap();
    let lib = libs.library_mut("user.shop").unwrap();
    let mut m = (*builtin_mat("pla.fff")).clone();
    m.id = mid("user.shop/pla-house");
    m.name = "House PLA".into();
    let v1 = Arc::clone(lib.upsert_material(m.clone()).unwrap());
    assert_eq!(v1.revision, 1);

    let (d, b) = design(2);
    let mut doc = Document::from_design(d);
    let mut tx = doc.transaction("Assign");
    for c in assign_material_in(tx.design(), &libs, &b, &v1) {
        tx.apply(c).unwrap();
    }
    tx.commit();
    let copy = Arc::clone(doc.design().material(&v1.id).unwrap());
    assert_eq!(libs.drift(&copy), Drift::UpToDate);
    assert!(
        update_from_library(doc.design(), &libs, &v1.id)
            .unwrap()
            .is_empty()
    );

    // Edit the library entry: revision 2.
    m.density = Density::g_per_cm3(1.30);
    let lib = libs.library_mut("user.shop").unwrap();
    let v2 = Arc::clone(lib.upsert_material(m).unwrap());
    assert_eq!(v2.revision, 2);
    assert_eq!(
        libs.drift(&copy),
        Drift::LibraryNewer {
            library_revision: 2
        }
    );

    let cmds = update_from_library(doc.design(), &libs, &v1.id).unwrap();
    assert_eq!(cmds.len(), 1, "{cmds:#?}");
    let mut tx = doc.transaction("Update from library");
    for c in cmds {
        tx.apply(c).unwrap();
    }
    tx.commit();
    let now = doc.design().material(&v1.id).unwrap();
    assert_eq!(**now, *v2);
    assert_eq!(libs.drift(now), Drift::UpToDate);
    assert_eq!(doc.design().material_users(&v1.id), b, "bodies untouched");

    assert!(doc.undo());
    assert_eq!(
        **doc.design().material(&v1.id).unwrap(),
        *v1,
        "undo restores v1"
    );
    assert!(doc.redo());
    assert_eq!(**doc.design().material(&v1.id).unwrap(), *v2);

    // A material that is in no library can't be updated.
    let (mut d, b) = design(1);
    let mut own = (*builtin_mat("pla.fff")).clone();
    own.id = mid("doc/own");
    apply_with(&mut d, |d| assign_material(d, &b, &Arc::new(own)));
    let e = update_from_library(&d, &libs, &mid("doc/own")).unwrap_err();
    assert!(matches!(e, CommandError::Invalid(_)));
}

// ---- (e) events -------------------------------------------------------------------------

#[test]
fn e_events_are_minimal() {
    amg_doc::testing::link();
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let mut d = Design::new("events");
    let root = d.root();
    for x in [1.0, 2.0, 3.0] {
        d.add_feature(None, bx(x), root, None).unwrap();
    }
    let mut doc = Document::from_design(d);
    doc.recompute_blocking(&jobs).unwrap();
    let b: Vec<BodyId> = doc.evaluated().bodies().map(|(id, _)| id).collect();
    doc.drain_events();

    let pla = builtin_mat("pla.fff");
    let app = pla.appearance.clone().unwrap();
    let mut tx = doc.transaction("Assign");
    for c in assign_material(tx.design(), &b[..2], &pla) {
        tx.apply(c).unwrap();
    }
    tx.commit();
    assert_eq!(
        doc.drain_events(),
        vec![
            BodyPropsChanged(b[0]),
            BodyPropsChanged(b[1]),
            MaterialChanged(pla.id.clone()),
            AppearanceChanged(app.clone()),
        ]
    );
    let no_geometry = |ev: &[DocEvent]| {
        !ev.iter()
            .any(|e| matches!(e, BodyGeometryChanged(_) | DocEvent::BodyAdded(_)))
    };
    doc.recompute_blocking(&jobs).unwrap();
    let rev = doc.history().revision();
    let ev = doc.drain_events();
    assert!(no_geometry(&ev), "{ev:?}");
    assert_eq!(
        ev,
        vec![
            RecomputeStarted(rev),
            RecomputeFinished {
                revision: rev,
                failed: 0
            }
        ]
    );

    // Editing the copy: only MaterialChanged (the bodies' props are unchanged).
    let mut edited = (*pla).clone();
    edited.density = Density::g_per_cm3(1.25);
    doc.apply(Command::SetMaterial {
        material: Arc::new(edited),
    })
    .unwrap();
    assert_eq!(doc.drain_events(), vec![MaterialChanged(pla.id.clone())]);
    // A no-op SetMaterial: no event, no undo entry.
    let entries = doc.history().stack().applied();
    let same = Arc::clone(doc.design().material(&pla.id).unwrap());
    doc.apply(Command::SetMaterial { material: same }).unwrap();
    assert_eq!(doc.drain_events(), vec![]);
    assert_eq!(doc.history().stack().applied(), entries);

    // Appearance edits.
    let mut a = (**doc.design().appearance(&app).unwrap()).clone();
    a.roughness = 0.25;
    doc.apply(Command::SetAppearance {
        appearance: Arc::new(a),
    })
    .unwrap();
    assert_eq!(doc.drain_events(), vec![AppearanceChanged(app)]);

    // Undo of the edit gives the same minimal event.
    doc.undo();
    doc.undo();
    let ev = doc.drain_events();
    assert!(ev.contains(&MaterialChanged(pla.id.clone())), "{ev:?}");
    assert!(
        ev.iter()
            .all(|e| matches!(e, MaterialChanged(_) | AppearanceChanged(_))),
        "{ev:?}"
    );
    doc.recompute_blocking(&jobs).unwrap();
    assert!(no_geometry(&doc.drain_events()));
}

// ---- (f) clone cost -----------------------------------------------------------------------

fn hundred_materials() -> Design {
    let (mut d, b) = design(1);
    let base = builtin_mat("pla.fff");
    let mut cmds = Vec::new();
    for i in 0..100 {
        let mut m = (*base).clone();
        m.id = MaterialId::new("doc", &format!("m{i:03}")).unwrap();
        cmds.push(Command::SetMaterial {
            material: Arc::new(m),
        });
    }
    apply_all(&mut d, cmds);
    apply_with(&mut d, |d| assign_material(d, &b, &base));
    assert_eq!(d.materials().count(), 101);
    d
}

#[test]
fn f_design_with_100_materials_clones_in_under_a_microsecond() {
    let d = hundred_materials();
    let n = 100_000u32;
    let mut best = Duration::MAX;
    for _ in 0..5 {
        let t = Instant::now();
        for _ in 0..n {
            std::hint::black_box(std::hint::black_box(&d).clone());
        }
        best = best.min(t.elapsed() / n);
    }
    println!("Design clone with 101 materials: {best:?}");
    if std::env::var_os("AMG_STRICT_TIMING").is_some_and(|v| v == "1") {
        assert!(best < Duration::from_micros(1), "{best:?}");
    }
    // The clone shares the table.
    let c = d.clone();
    let id = MaterialId::new("doc", "m000").unwrap();
    assert!(Arc::ptr_eq(
        c.material(&id).unwrap(),
        d.material(&id).unwrap()
    ));
}

// ---- default material preference ------------------------------------------------------

#[test]
fn mass_material_uses_assignment_then_default_preference() {
    let (mut d, b) = design(2);
    let libs = LibrarySet::builtin_only();
    let prefs = MaterialPrefs::default();
    let ti = builtin_mat("ti6al4v.lpbf");
    apply_with(&mut d, |d| assign_material(d, &b[..1], &ti));

    let m0 = d.mass_material(b[0], &prefs, &libs).unwrap();
    assert_eq!(m0.material.id, ti.id);
    assert!(!m0.defaulted);
    let m1 = d.mass_material(b[1], &prefs, &libs).unwrap();
    assert_eq!(m1.material.id, default_material_id());
    assert!(m1.defaulted);

    // Another default from the preference file; the design copy wins over the library.
    let petg_prefs: MaterialPrefs =
        serde_json::from_str(r#"{"default_material":"amg.builtin/petg.fff"}"#).unwrap();
    assert_eq!(
        d.mass_material(b[1], &petg_prefs, &libs)
            .unwrap()
            .material
            .id
            .local(),
        "petg.fff"
    );
    let mut own = (*builtin_mat("petg.fff")).clone();
    own.density = Density::g_per_cm3(1.0);
    apply_all(
        &mut d,
        vec![Command::SetMaterial {
            material: Arc::new(own.clone()),
        }],
    );
    assert_eq!(
        *d.mass_material(b[1], &petg_prefs, &libs).unwrap().material,
        own
    );
    // Default neither in the design nor in the libraries.
    assert!(d.mass_material(b[1], &prefs, &empty_set()).is_none());
    // Keys round-trip.
    assert_eq!(MaterialKey::of(&ti.id).id(), Some(ti.id.clone()));
    assert_eq!(AppearanceKey("nope".into()).id(), None);
}
