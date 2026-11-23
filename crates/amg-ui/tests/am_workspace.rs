//! AM-11: the Phase 1 Manufacture workspace in the real app (`AppHarness`, kittest).
//!
//! - every command is on the ADDITIVE toolbar and in S-key search; every edit is one undo entry;
//! - Place on Plate: picking a face of an imported STL puts that face on the plate;
//! - an item moved out of the build volume turns red with a size message;
//! - Send to Slicer (a stand-in executable, never the slicer GUI) and Export Plate 3MF;
//! - the machine picker (built-ins + user folder) and the settings dialog;
//! - the UI-thread frame time while placing a 1 M-triangle body (asserted only under
//!   `AMG_STRICT_TIMING=1`; numbers are printed for the report);
//! - the Phase 1 script (open STL → new setup on Ender-3 V3 KE KCM → place face down → Send to
//!   Slicer) on three corpus models (skipped for un-fetched git-LFS pointers unless
//!   `AMG_REQUIRE_TESTDATA=1`), with an OrcaSlicer CLI `--info` check when `AMG_ORCA_SLICER`
//!   names the executable (the CLI only, never the GUI).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "test code"
)]

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_am::handoff::{HandoffFormat, SlicerConfig};
use amg_am::machine::{MachineKey, MachineLibrary, SlicerKind, machine_to_toml};
use amg_am::place::{ZMode, effective_transform, pick_plane_normal};
use amg_core::Transform;
use amg_core::glam::{DQuat, DVec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, Dependency, Document, Evaluated, ExtKey,
    FeatureBox, FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_features::am::{PrintSetup, setups};
use amg_mesh::{FaceId, IndexedMesh};
use amg_ui::CommitBatch;
use amg_ui::am::settings::{self, AmSettings};
use amg_ui::am::{
    ADD_BODIES, AM, AM_TAB, AmLayer, CENTRE, DROP, EXPORT_3MF, ItemStatus, MOVE_ROTATE,
    MoveRotateDialog, NEW_SETUP, NewSetupDialog, OUT_OF_VOLUME_TINT, PLACE_FACE_DOWN, REMOVE_ITEMS,
    SEND_TO_SLICER, SETTINGS, SettingsDialog, SetupBrowser, active_setup, command_ids,
};
use amg_ui::browser::{BrowserAnchor, BrowserProvider};
use amg_ui::command::Placement;
use amg_ui::pick::{PickMode, PickTarget};
use amg_ui::search::{Recency, SearchIndex};
use amg_ui::select::SelItem;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use amg_ui::testing::{AppHarness, FakeFileDialog};
use serde::{Deserialize, Serialize};

const KCM: &str = "builtin:creality/ender-3-v3-ke-kcm";

// ---- fixtures ---------------------------------------------------------------------------------

static MESHES: Mutex<Vec<Arc<IndexedMesh>>> = Mutex::new(Vec::new());

/// A mesh source whose mesh lives in a process-wide table (small feature records).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct TestMesh {
    key: usize,
}

impl amg_doc::Feature for TestMesh {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let m = Arc::clone(&MESHES.lock().unwrap()[self.key]);
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(m),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(TestMesh, "uitest.am11_source", version = 1);

fn link() {
    amg_features::mesh::link();
    amg_features::io::link();
    amg_features::am::link();
}

fn newest_mesh_body(ev: &Evaluated) -> Option<BodyId> {
    ev.bodies()
        .filter(|(_, b)| matches!(b.geom, BodyGeom::Mesh(_)))
        .map(|(id, _)| id)
        .max()
}

fn add_mesh(h: &mut AppHarness, m: IndexedMesh) -> BodyId {
    link();
    let key = {
        let mut t = MESHES.lock().unwrap();
        t.push(Arc::new(m));
        t.len() - 1
    };
    let root = h.document().design().root();
    h.app_mut().submit(CommitBatch {
        label: "Mesh".into(),
        commands: vec![Command::AddFeature {
            at: None,
            feature: FeatureBox::new(TestMesh { key }),
            context: root,
            name: None,
        }],
    });
    h.drive_until_idle();
    newest_mesh_body(h.document().evaluated()).expect("a mesh body")
}

fn mesh(h: &AppHarness, body: BodyId) -> Arc<IndexedMesh> {
    match &h.document().evaluated().body(body).expect("body").geom {
        BodyGeom::Mesh(m) => Arc::clone(m),
        other => panic!("{other:?}"),
    }
}

/// An axis-aligned box `size` with its min corner at `min`, transformed by `t`.
fn cuboid(size: DVec3, t: Transform) -> IndexedMesh {
    let mut p = Vec::new();
    for i in 0..8u32 {
        let c = DVec3::new(
            if i & 1 == 0 { 0.0 } else { size.x },
            if i & 2 == 0 { 0.0 } else { size.y },
            if i & 4 == 0 { 0.0 } else { size.z },
        );
        p.push(t.transform_point3(c));
    }
    let tris = vec![
        [0, 2, 1],
        [1, 2, 3],
        [4, 5, 6],
        [5, 7, 6],
        [0, 1, 4],
        [1, 5, 4],
        [2, 6, 3],
        [3, 6, 7],
        [0, 4, 2],
        [2, 4, 6],
        [1, 3, 5],
        [3, 7, 5],
    ];
    IndexedMesh::new(p, tris).unwrap()
}

/// Binary STL bytes of `m`.
fn stl_bytes(m: &IndexedMesh) -> Vec<u8> {
    let mut out = vec![0u8; 80];
    out.extend_from_slice(&u32::try_from(m.tris.len()).unwrap().to_le_bytes());
    #[allow(clippy::cast_possible_truncation, reason = "STL is f32")]
    for (f, t) in m.tris.iter().enumerate() {
        let n = m.face_normal(FaceId(u32::try_from(f).unwrap()));
        for v in [n.x, n.y, n.z] {
            out.extend_from_slice(&(v as f32).to_le_bytes());
        }
        for &i in t {
            let p = m.positions[i as usize];
            for v in [p.x, p.y, p.z] {
                out.extend_from_slice(&(v as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

/// Imports an STL file into the open document as Insert Mesh does (S12 `io.import_mesh`).
fn import_stl(h: &mut AppHarness, bytes: &[u8], name: &str) -> BodyId {
    use amg_features::io::{ImportChoices, ImportSource, import_commands, raw_options};
    use amg_io::{Format, ImportOptions, Source};
    link();
    let ctx = amg_jobs::JobCtx::detached();
    let opts = ImportOptions::default();
    let imported = amg_io::import(
        Source::Bytes(bytes),
        Some(Format::Stl),
        &raw_options(&opts),
        &ctx,
    )
    .unwrap();
    let doc: &mut Document = &mut h.app_mut().active_document_mut().document;
    let choices = ImportChoices::from_options(doc.design().root(), &opts, &imported.report);
    let source = ImportSource::from_bytes(name, Format::Stl, imported.report.declared_unit, bytes);
    let plan = import_commands(imported, source, &choices).unwrap();
    plan.apply(doc).unwrap();
    h.drive_until_idle();
    newest_mesh_body(h.document().evaluated()).expect("imported mesh body")
}

/// A per-test folder under the system temp folder.
fn temp_dir(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("amg-am11-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// A stand-in "slicer" that exits at once (never the slicer GUI).
fn stand_in() -> SlicerConfig {
    #[cfg(windows)]
    let (exe, args) = {
        let root = std::env::var_os("SystemRoot").unwrap_or_else(|| "C:\\Windows".into());
        (
            Path::new(&root).join("System32").join("cmd.exe"),
            vec!["/C".to_owned(), "echo".to_owned(), "{file}".to_owned()],
        )
    };
    #[cfg(not(windows))]
    let (exe, args) = (PathBuf::from("/bin/echo"), vec!["{file}".to_owned()]);
    SlicerConfig {
        kind: SlicerKind::OrcaSlicer,
        exe,
        args,
        format: HandoffFormat::ThreeMf,
    }
}

/// Points the settings at `dir` with the stand-in slicer; returns the hand-off folder.
fn configure(dir: &Path) -> PathBuf {
    settings::set_settings_path(Some(dir.join("am-settings.json")));
    let handoff = dir.join("handoff");
    settings::save(AmSettings {
        slicer: Some(stand_in()),
        handoff_dir: Some(handoff.clone()),
        machines_dir: Some(dir.join("machines")),
    });
    handoff
}

fn select(h: &mut AppHarness, bodies: &[BodyId]) {
    let sel = &mut h.app_mut().active_document_mut().selection;
    sel.clear();
    for b in bodies {
        sel.add(
            SelItem::from_target(PickTarget::Body {
                body: *b,
                instance: 0,
                sub: None,
            })
            .unwrap(),
        );
    }
}

fn click(h: &mut AppHarness, label: &str) {
    h.kittest().step();
    h.kittest().get_by_label(label).click();
    h.kittest().step();
    h.drive_until_idle();
}

/// New Setup on the KCM machine with `bodies` selected; returns the new key.
fn new_setup(h: &mut AppHarness, bodies: &[BodyId]) -> ExtKey {
    select(h, bodies);
    let before = setups(h.document().design()).len();
    h.run_command(NEW_SETUP);
    h.kittest().step();
    {
        let d = h.app_mut().dialog_mut::<NewSetupDialog>().expect("dialog");
        d.machine = MachineKey(KCM.to_owned());
    }
    click(h, "OK");
    let all = setups(h.document().design());
    assert_eq!(all.len(), before + 1, "one setup added");
    all.last().unwrap().0.clone()
}

fn enter(h: &mut AppHarness) {
    h.app_mut().set_workspace(AM);
    h.drive_until_idle();
    h.kittest().step();
    h.drive_until_idle();
}

fn layer(h: &AppHarness) -> &AmLayer {
    h.app().doc_layer::<AmLayer>().expect("plate layer")
}

fn setup(h: &AppHarness) -> (ExtKey, PrintSetup) {
    let chosen = layer(h).shown_setup().cloned();
    active_setup(h.document().design(), chosen.as_ref()).expect("a setup")
}

fn status(h: &AppHarness, body: BodyId) -> ItemStatus {
    layer(h)
        .status()
        .iter()
        .find(|s| s.body == body)
        .cloned()
        .expect("item status")
}

fn undo_count(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

fn record(h: &AppHarness, key: &ExtKey) -> serde_json::Value {
    h.document()
        .design()
        .exts()
        .find(|(k, _)| *k == key)
        .map(|(_, r)| r.data.clone())
        .unwrap_or(serde_json::Value::Null)
}

/// Runs `edit`, then checks it added exactly one undo entry and that undo/redo restore the
/// records before/after.
fn one_undo_entry(
    h: &mut AppHarness,
    key: &ExtKey,
    what: &str,
    edit: impl FnOnce(&mut AppHarness),
) {
    let n = undo_count(h);
    let before = record(h, key);
    edit(h);
    h.drive_until_idle();
    assert_eq!(undo_count(h), n + 1, "{what}: one undo entry");
    let after = record(h, key);
    assert_ne!(before, after, "{what} changed the setup");
    let doc = h.app().active_document().id;
    h.app_mut().request_undo(doc);
    h.drive_until_idle();
    assert_eq!(record(h, key), before, "{what}: undo");
    h.app_mut().request_redo(doc);
    h.drive_until_idle();
    assert_eq!(record(h, key), after, "{what}: redo");
}

/// Picks the first face hit around the screen projection of the item's bounds centre; returns
/// the screen position and the face.
fn face_under_item(h: &mut AppHarness, body: BodyId) -> (egui::Pos2, u32) {
    let st = status(h, body);
    let b = st.fit.expect("fit").aabb;
    let view = h.camera_view();
    let c = (b.min + b.max) * 0.5;
    let centre = view.project(c).expect("item on screen");
    for r in 0..12 {
        for k in 0..8 {
            let a = f32::from(u8::try_from(k).unwrap()) * std::f32::consts::FRAC_PI_4;
            let d = f32::from(u8::try_from(r).unwrap()) * 3.0;
            let pos = centre + egui::vec2(a.cos(), a.sin()) * d;
            let hits = h.pick_ray(view.ray(pos), PickMode::Nearest);
            if let Some(PickTarget::Body {
                body: hb,
                sub: Some((amg_doc::TopoKind::Face, f)),
                ..
            }) = hits.first().map(|x| x.target)
                && hb == body
            {
                return (pos, f);
            }
        }
    }
    panic!("no face of the item under the cursor");
}

/// Place on Plate by clicking `pos`; returns the new transform.
fn place_face_down(h: &mut AppHarness, body: BodyId, pos: egui::Pos2) -> Transform {
    h.run_command(PLACE_FACE_DOWN);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(PLACE_FACE_DOWN));
    h.click_at(pos, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
    h.kittest().step();
    h.drive_until_idle();
    assert_eq!(
        h.app().active_tool(),
        None,
        "the tool finished after the pick"
    );
    status(h, body).transform.expect("transform")
}

/// The picked face's plane normal points at −Z and the item rests on the plate. With
/// `touches`, the face itself lies on the plate (true for convex items; on a non-convex one other
/// geometry may be lower, then the face is only horizontal).
fn assert_face_down(m: &IndexedMesh, t: &Transform, face: u32, touches: bool, what: &str) {
    let n = pick_plane_normal(m, FaceId(face));
    let down = t.matrix3 * n;
    assert!(
        (down.normalize() - DVec3::NEG_Z).length() < 1e-9,
        "{what}: picked normal at {down:?}"
    );
    let min_z = m
        .positions
        .iter()
        .map(|p| t.transform_point3(*p).z)
        .fold(f64::INFINITY, f64::min);
    assert!(min_z.abs() < 1e-9, "{what}: min z {min_z}");
    let zs: Vec<f64> = m.tris[face as usize]
        .iter()
        .map(|&v| t.transform_point3(m.positions[v as usize]).z)
        .collect();
    if touches {
        assert!(
            zs.iter().all(|z| z.abs() < 1e-6),
            "{what}: picked face at z {zs:?}"
        );
    } else if pick_plane_normal(m, FaceId(face)) == m.face_normal(FaceId(face)) {
        let spread = zs.iter().fold(f64::NEG_INFINITY, |a, b| a.max(*b))
            - zs.iter().fold(f64::INFINITY, |a, b| a.min(*b));
        assert!(spread < 1e-6, "{what}: picked face not horizontal: {zs:?}");
    }
}

fn tilted() -> Transform {
    Transform::from_rotation_translation(
        DQuat::from_axis_angle(DVec3::new(1.0, 2.0, 3.0).normalize(), 0.7),
        DVec3::new(500.0, -300.0, 77.0),
    )
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1")
}

// ---- toolbar and search -----------------------------------------------------------------------

#[test]
fn every_command_is_on_the_additive_toolbar_and_in_s_key_search() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let ids = command_ids();
    // The Interfaces list plus Remove and Settings.
    for id in [
        "am.new_setup",
        "am.add_bodies",
        "am.place_face_down",
        "am.drop",
        "am.move_rotate",
        "am.centre",
        "am.overhang_toggle",
        "am.auto_orient",
        "am.arrange",
        "am.send_to_slicer",
        "am.export_3mf",
        "am.remove_items",
        "am.settings",
    ] {
        assert!(ids.iter().any(|c| c.0 == id), "{id} registered");
    }
    let reg = h.app().registry();
    assert!(reg.workspace(AM).is_some());
    assert!(reg.all_tabs(AM).iter().any(|t| t.id == AM_TAB));
    let index = SearchIndex::build(reg);
    let recency = Recency::new();
    for id in &ids {
        let def = reg.command(*id).unwrap_or_else(|| panic!("{id}"));
        let groups: Vec<_> = def
            .placements
            .iter()
            .filter_map(|p| match p {
                Placement::Toolbar { group, .. } => Some(*group),
                _ => None,
            })
            .collect();
        assert!(!groups.is_empty(), "{id} has a toolbar placement");
        for g in groups {
            assert_eq!(reg.group(g).map(|g| g.tab), Some(AM_TAB), "{id} in {g}");
            assert!(reg.group_commands(g).any(|(_, c)| c.id == *id));
        }
        let title = def.label.trim_end_matches('…').to_owned();
        let hits = index.search(&title, &recency);
        assert_eq!(
            hits.first().map(|x| x.id),
            Some(*id),
            "search {title:?}: {hits:?}"
        );
        for k in def.keywords {
            assert!(
                index.search(k, &recency).iter().any(|x| x.id == *id),
                "keyword {k} finds {id}"
            );
        }
    }

    // The S key finds and runs a command in the Manufacture workspace.
    h.app_mut().set_workspace(AM);
    h.drive_until_idle();
    h.kittest().key_press(egui::Key::S);
    for _ in 0..6 {
        h.kittest().step();
    }
    assert!(h.app().search().is_open());
    h.kittest().event(egui::Event::Text("new setup".to_owned()));
    for _ in 0..6 {
        h.kittest().step();
    }
    assert_eq!(
        h.app().search().results().first().map(|x| x.id),
        Some(NEW_SETUP)
    );
    h.kittest().key_press(egui::Key::Enter);
    h.drive_until_idle();
    h.kittest().step();
    assert!(
        h.app().dialog::<NewSetupDialog>().is_some(),
        "New Setup opened from search"
    );
}

// ---- edits and undo ---------------------------------------------------------------------------

#[test]
fn every_setup_edit_is_one_undo_entry() {
    let dir = temp_dir("undo");
    configure(&dir);
    let mut h = AppHarness::new();
    let a = add_mesh(
        &mut h,
        cuboid(DVec3::new(30.0, 20.0, 10.0), Transform::IDENTITY),
    );
    let b = add_mesh(&mut h, cuboid(DVec3::new(10.0, 10.0, 40.0), tilted()));

    // New Setup: one AddExt.
    let n = undo_count(&h);
    let key = new_setup(&mut h, &[a]);
    assert_eq!(undo_count(&h), n + 1, "New Setup: one undo entry");
    assert_eq!(h.document().undo_label(), Some("New Print Setup"));
    enter(&mut h);
    assert_eq!(layer(&h).shown_setup(), Some(&key));

    one_undo_entry(&mut h, &key, "Add Bodies", |h| {
        select(h, &[b]);
        h.run_command(ADD_BODIES);
    });
    assert_eq!(setup(&h).1.plates[0].items.len(), 2);
    h.drive_until_idle();

    one_undo_entry(&mut h, &key, "Move/Rotate", |h| {
        select(h, &[b]);
        h.run_command(MOVE_ROTATE);
        h.kittest().step();
        {
            let d = h
                .app_mut()
                .dialog_mut::<MoveRotateDialog>()
                .expect("dialog");
            d.fields.x += 40.0;
            d.fields.rotate_z = 30.0;
            d.fields.z = 5.0;
        }
        click(h, "OK");
    });
    assert_eq!(h.app().dialog_titles(), Vec::<String>::new());
    assert_eq!(layer(&h).preview(), None, "preview cleared on OK");

    one_undo_entry(&mut h, &key, "Drop", |h| {
        select(h, &[b]);
        h.run_command(DROP);
    });
    let item_b = setup(&h).1.plates[0].items[1].clone();
    assert_eq!(item_b.placement.z, ZMode::OnPlate { elevation: 0.0 });

    one_undo_entry(&mut h, &key, "Centre", |h| {
        select(h, &[b]);
        h.run_command(CENTRE);
    });
    let item_b = setup(&h).1.plates[0].items[1].clone();
    assert_eq!(
        item_b.placement.xy,
        amg_core::glam::DVec2::new(110.0, 110.0)
    );

    h.drive_until_idle();
    let (pos, _) = face_under_item(&mut h, b);
    one_undo_entry(&mut h, &key, "Place on Plate", |h| {
        place_face_down(h, b, pos);
    });

    one_undo_entry(&mut h, &key, "Remove", |h| {
        select(h, &[a]);
        h.run_command(REMOVE_ITEMS);
    });
    let items: Vec<BodyId> = setup(&h).1.plates[0].items.iter().map(|i| i.body).collect();
    assert_eq!(items, vec![b]);
}

// ---- Place on Plate ---------------------------------------------------------------------------

#[test]
fn picking_a_face_of_an_imported_stl_puts_it_on_the_plate() {
    let dir = temp_dir("pick");
    configure(&dir);
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let src = cuboid(DVec3::new(30.0, 20.0, 10.0), tilted());
    let body = import_stl(&mut h, &stl_bytes(&src), "tilted-box.stl");
    new_setup(&mut h, &[body]);
    enter(&mut h);
    // Before: no rotation, resting on its lowest corner at the bed centre.
    let before = status(&h, body);
    assert!(before.fit.as_ref().unwrap().fits, "{before:?}");
    // Only the plate items are shown, at their plate transforms.
    let views = h
        .scene()
        .body_views()
        .expect("body views in the Manufacture workspace");
    assert_eq!(views.bodies.len(), 1);
    assert_eq!(views.bodies[&body].transform, before.transform.unwrap());

    let m = mesh(&h, body);
    let (pos, face) = face_under_item(&mut h, body);
    let t = place_face_down(&mut h, body, pos);
    assert_face_down(&m, &t, face, true, "tilted box");
    // Same anchor: the footprint centre stays where it was.
    let (_, s) = setup(&h);
    let p = s.plates[0].items[0].placement;
    assert_eq!(p.xy, amg_core::glam::DVec2::new(110.0, 110.0));
    assert_eq!(effective_transform(&m, &p), t);
    // The scene and picking follow the new transform.
    assert_eq!(h.scene().body_views().unwrap().bodies[&body].transform, t);

    // Leaving the workspace restores the design view.
    h.app_mut().set_workspace(amg_ui::design::DESIGN);
    h.drive_until_idle();
    assert!(h.scene().body_views().is_none());
}

// ---- out of volume ----------------------------------------------------------------------------

#[test]
fn an_item_moved_out_of_the_volume_turns_red_with_a_size_message() {
    let dir = temp_dir("fit");
    configure(&dir);
    let mut h = AppHarness::new();
    let small = add_mesh(
        &mut h,
        cuboid(DVec3::new(20.0, 20.0, 20.0), Transform::IDENTITY),
    );
    let tall = add_mesh(
        &mut h,
        cuboid(DVec3::new(20.0, 20.0, 250.0), Transform::IDENTITY),
    );
    new_setup(&mut h, &[small, tall]);
    enter(&mut h);

    // 250 mm on a 240 mm volume.
    let st = status(&h, tall);
    assert_eq!(st.message.as_deref(), Some("10.0 mm too tall"));
    let views = h.scene().body_views().unwrap().clone();
    assert_eq!(views.bodies[&tall].tint, Some(OUT_OF_VOLUME_TINT));
    assert_eq!(views.bodies[&small].tint, None);
    assert!(status(&h, small).message.is_none());

    // Move the small one 15 mm past the right edge of the 220 mm bed (x = 235 + 10).
    select(&mut h, &[small]);
    h.run_command(MOVE_ROTATE);
    h.kittest().step();
    h.app_mut()
        .dialog_mut::<MoveRotateDialog>()
        .unwrap()
        .fields
        .x = 225.0 + 10.0;
    h.kittest().step();
    h.drive_until_idle();
    // Live preview: red before OK.
    assert_eq!(
        status(&h, small).message.as_deref(),
        Some("25.0 mm outside the bed")
    );
    click(&mut h, "OK");
    let st = status(&h, small);
    assert_eq!(st.message.as_deref(), Some("25.0 mm outside the bed"));
    assert_eq!(
        h.scene().body_views().unwrap().bodies[&small].tint,
        Some(OUT_OF_VOLUME_TINT)
    );
    // The scene instance is drawn in the tint.
    let backend = h
        .scene()
        .backend()
        .as_any()
        .downcast_ref::<amg_ui::scene::CpuSceneBackend>()
        .expect("CPU backend");
    let tinted = backend
        .instances()
        .filter(|(_, d)| d.visible && d.material.base_color == OUT_OF_VOLUME_TINT)
        .count();
    assert_eq!(tinted, 2, "both out-of-volume items are red");
    // A label with the message next to each item.
    h.kittest().step();
    assert!(h.app().tool_overlays().label_count() >= 2);

    // Centre: fits again.
    h.run_command(CENTRE);
    h.drive_until_idle();
    assert!(status(&h, small).message.is_none());
    assert_eq!(h.scene().body_views().unwrap().bodies[&small].tint, None);
}

// ---- hand-off ---------------------------------------------------------------------------------

/// Plate-frame positions of every exported vertex vs the item transform.
fn check_export(path: &Path, m: &IndexedMesh, t: &Transform, tol: f64) {
    let back = amg_io::import(
        amg_io::Source::Path(path),
        None,
        &amg_io::ImportOptions::default(),
        &amg_jobs::JobCtx::detached(),
    )
    .unwrap()
    .scene;
    let inst = back.instances();
    assert_eq!(inst.len(), 1, "one object");
    let (o, wt) = inst[0];
    let obj = &back.objects[o as usize];
    assert_eq!(obj.mesh.positions.len(), m.positions.len());
    let mut worst: f64 = 0.0;
    for (a, b) in obj.mesh.positions.iter().zip(&m.positions) {
        let got = wt.transform_point3(*a);
        let want = t.transform_point3(*b);
        worst = worst.max((got - want).length());
    }
    assert!(worst < tol, "worst vertex error {worst} mm");
}

#[test]
fn send_to_slicer_and_export_3mf_write_the_plate() {
    let dir = temp_dir("send");
    let handoff = configure(&dir);
    let mut h = AppHarness::new();
    let fake = FakeFileDialog::new();
    h.app_mut().set_file_dialog_backend(fake.clone());
    let body = add_mesh(&mut h, cuboid(DVec3::new(30.0, 20.0, 10.0), tilted()));
    new_setup(&mut h, &[body]);
    enter(&mut h);
    let t = status(&h, body).transform.unwrap();
    let m = mesh(&h, body);

    // Send to Slicer: a job exports, prunes and starts the (stand-in) slicer.
    h.run_command(SEND_TO_SLICER);
    h.drive_until_idle();
    let out = layer(&h).last_output().cloned().expect("sent");
    assert_eq!(out.objects, 1);
    assert_eq!(out.slicer, Some(SlicerKind::OrcaSlicer));
    assert!(out.pid.is_some());
    assert!(out.path.starts_with(&handoff), "{:?}", out.path);
    assert_eq!(out.path.extension().and_then(|e| e.to_str()), Some("3mf"));
    check_export(&out.path, &m, &t, 1e-4);

    // A missing slicer fails with a toast and writes nothing.
    let mut s = settings::load();
    s.slicer = Some(SlicerConfig::new(
        SlicerKind::OrcaSlicer,
        dir.join("missing").join("orca-slicer.exe"),
    ));
    s.handoff_dir = Some(dir.join("handoff2"));
    settings::save(s);
    h.run_command(SEND_TO_SLICER);
    h.drive_until_idle();
    assert!(!dir.join("handoff2").exists(), "nothing written");
    assert!(
        h.app().notifications().items().iter().any(|t| {
            let n = &t.notification;
            n.body.contains("slicer")
                || n.hint.as_deref().is_some_and(|h| h.contains("Preferences"))
        }),
        "a toast names the problem"
    );

    // Export Plate 3MF: Save dialog, then a job.
    let target = dir.join("plate");
    fake.answer(Some(target.clone()));
    h.run_command(EXPORT_3MF);
    h.drive_until_idle();
    h.kittest().step();
    h.drive_until_idle();
    let file = target.with_extension("3mf");
    assert!(file.is_file(), "{file:?}");
    assert_eq!(
        layer(&h).last_output().map(|o| o.path.clone()),
        Some(file.clone())
    );
    check_export(&file, &m, &t, 1e-4);
}

// ---- machine picker, settings and browser -----------------------------------------------------

#[test]
fn the_machine_picker_lists_the_library_and_the_user_folder() {
    let dir = temp_dir("machines");
    configure(&dir);
    let machines = dir.join("machines");
    std::fs::create_dir_all(&machines).unwrap();
    let lib = MachineLibrary::builtin();
    let mut mine = lib.get(&MachineKey(KCM.to_owned())).unwrap().clone();
    mine.name = "My big printer".to_owned();
    mine.key = MachineKey("user:big".to_owned());
    mine.volume.height = 400.0;
    std::fs::write(machines.join("big.toml"), machine_to_toml(&mine)).unwrap();
    std::fs::write(
        machines.join("broken.toml"),
        "name = 3\nbogus_field = true\n",
    )
    .unwrap();

    let mut h = AppHarness::new();
    let body = add_mesh(
        &mut h,
        cuboid(DVec3::new(10.0, 10.0, 300.0), Transform::IDENTITY),
    );
    select(&mut h, &[body]);
    h.run_command(NEW_SETUP);
    h.kittest().step();
    let d = h.app().dialog::<NewSetupDialog>().expect("dialog");
    let names: Vec<String> = d.machines().map(|m| m.name.clone()).collect();
    assert!(names.iter().any(|n| n == "My big printer"), "{names:?}");
    assert!(names.len() > lib.len());
    assert_eq!(
        d.machine,
        MachineKey(KCM.to_owned()),
        "the KCM is preselected"
    );
    assert!(
        d.warnings().iter().any(|w| w.file.ends_with("broken.toml")),
        "{:?}",
        d.warnings()
    );
    let user_key = d
        .machines()
        .find(|m| m.name == "My big printer")
        .unwrap()
        .key
        .clone();
    h.app_mut().dialog_mut::<NewSetupDialog>().unwrap().machine = user_key;
    click(&mut h, "OK");
    enter(&mut h);
    let (_, s) = setup(&h);
    assert_eq!(s.machine.name, "My big printer");
    // 300 mm fits the 400 mm user machine.
    assert!(status(&h, body).message.is_none());

    // The browser lists setup → plate → item.
    let nodes = h.app_mut().with_cx(|cx| {
        let env = cx.env();
        let mut out = Vec::new();
        SetupBrowser.build(&env, &BrowserAnchor::Root, &mut out);
        out
    });
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0].label, "Print Setups");
    let setup_node = &nodes[0].children[0];
    assert!(
        setup_node.label.contains("My big printer"),
        "{}",
        setup_node.label
    );
    let item = &setup_node.children[0].children[0];
    assert_eq!(
        item.target.as_ref().and_then(|t| t.target.body()),
        Some(body)
    );
}

#[test]
fn the_settings_dialog_saves_the_slicer_and_folders() {
    let dir = temp_dir("settings");
    configure(&dir);
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h.run_command(SETTINGS);
    h.kittest().step();
    {
        let d = h.app_mut().dialog_mut::<SettingsDialog>().expect("dialog");
        assert!(!d.auto_detect);
        d.kind = SlicerKind::Chitubox;
        d.exe = "D:/CHITUBOX/CHITUBOX.exe".to_owned();
        d.args = "{file}".to_owned();
        d.format = HandoffFormat::Stl;
        d.handoff_dir = dir.join("h2").to_string_lossy().into_owned();
    }
    click(&mut h, "OK");
    assert!(h.app().dialog::<SettingsDialog>().is_none());
    settings::set_settings_path(Some(dir.join("am-settings.json")));
    let s = settings::load();
    let c = s.slicer.unwrap();
    assert_eq!(c.kind, SlicerKind::Chitubox);
    assert_eq!(c.exe, PathBuf::from("D:/CHITUBOX/CHITUBOX.exe"));
    assert_eq!(c.format, HandoffFormat::Stl);
    assert_eq!(s.handoff_dir, Some(dir.join("h2")));
}

// ---- timing -----------------------------------------------------------------------------------

/// Records how long each `amg_ui.frame` span was entered (the UI thread's frame time).
#[derive(Clone, Default)]
struct FrameTimes(Arc<Mutex<Vec<Duration>>>);

struct Entered(Instant);

impl<S> tracing_subscriber::Layer<S> for FrameTimes
where
    S: tracing::Subscriber + for<'a> tracing_subscriber::registry::LookupSpan<'a>,
{
    fn on_enter(&self, id: &tracing::span::Id, ctx: tracing_subscriber::layer::Context<'_, S>) {
        if let Some(span) = ctx.span(id)
            && span.name() == "amg_ui.frame"
        {
            span.extensions_mut().replace(Entered(Instant::now()));
        }
    }

    fn on_exit(&self, id: &tracing::span::Id, ctx: tracing_subscriber::layer::Context<'_, S>) {
        if let Some(span) = ctx.span(id)
            && let Some(Entered(t)) = span.extensions().get::<Entered>()
        {
            self.0.lock().unwrap().push(t.elapsed());
        }
    }
}

/// Steps frames like a continuously repainting event loop until the app is idle.
fn step_until_idle(h: &mut AppHarness) {
    let t0 = Instant::now();
    loop {
        h.kittest().step();
        let app = h.app();
        let idle = app.jobs().jobs().is_empty()
            && app.scene().pending() == 0
            && !app.input_busy()
            && matches!(
                app.active_document().document.recompute_status(),
                amg_doc::RecomputeStatus::Idle
            )
            && app.doc_layer::<AmLayer>().is_none_or(|l| !l.busy());
        if idle {
            h.kittest().step();
            return;
        }
        assert!(t0.elapsed() < Duration::from_secs(300), "did not settle");
        std::thread::sleep(Duration::from_millis(2));
    }
}

#[test]
fn timing_no_frame_over_16ms_while_placing_a_1m_tri_body() {
    use tracing_subscriber::layer::SubscriberExt as _;

    let dir = temp_dir("timing");
    configure(&dir);
    let mut h = AppHarness::new();
    let sphere =
        amg_mesh::corpus::bench::benchmark_sphere(amg_mesh::corpus::bench::BenchSize::M1, 1);
    let tris = sphere.face_count();
    let body = add_mesh(&mut h, sphere);
    new_setup(&mut h, &[body]);
    enter(&mut h);
    let (pos, _) = face_under_item(&mut h, body);

    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let _guard = tracing::subscriber::set_default(subscriber);
    let t0 = Instant::now();
    // Place on Plate (face pick + rotation job + fit check).
    h.run_command(PLACE_FACE_DOWN);
    step_until_idle(&mut h);
    h.click_at(pos, egui::PointerButton::Primary, egui::Modifiers::NONE);
    step_until_idle(&mut h);
    // Drop, Centre.
    h.app_mut().request_command(DROP);
    step_until_idle(&mut h);
    h.app_mut().request_command(CENTRE);
    step_until_idle(&mut h);
    // Move/Rotate with a live preview: a new placement every frame for 60 frames (a drag),
    // each superseding the running fit check, then OK.
    select(&mut h, &[body]);
    h.app_mut().request_command(MOVE_ROTATE);
    step_until_idle(&mut h);
    for i in 0..60 {
        if let Some(d) = h.app_mut().dialog_mut::<MoveRotateDialog>() {
            d.fields.x = 80.0 + f64::from(i);
            d.fields.rotate_z = f64::from(i) * 3.0;
        }
        h.kittest().step();
        std::thread::sleep(Duration::from_millis(4));
    }
    step_until_idle(&mut h);
    h.kittest().get_by_label("OK").click();
    step_until_idle(&mut h);
    let total = t0.elapsed();
    let st = status(&h, body);
    assert!(st.transform.is_some() && !st.pending);
    drop(_guard);

    let mut t = times.0.lock().unwrap().clone();
    t.sort();
    let max = *t.last().unwrap();
    let p50 = t[t.len() / 2];
    let p99 = t[t.len() * 99 / 100];
    let over = t.iter().filter(|d| **d > Duration::from_millis(16)).count();
    eprintln!(
        "AM-11 frames while placing a {tris}-tri body (place on plate, drop, centre, 60-frame move/rotate preview, OK): {} frames in {total:?}; p50 {p50:?}, p99 {p99:?}, max {max:?}, {over} over 16 ms; {} fit checks",
        t.len(),
        layer(&h).checks()
    );
    if strict_timing() {
        assert!(max < Duration::from_millis(16), "max frame {max:?}");
    }
}

// ---- Phase 1 script on corpus models ----------------------------------------------------------

const SCRIPT_MODELS: [&str; 3] = [
    "mesh/real/thingi10k/132062.stl",
    "mesh/real/thingi10k/37825.stl",
    "mesh/real/thingi10k/36371.stl",
];

fn orca_exe() -> Option<PathBuf> {
    std::env::var_os("AMG_ORCA_SLICER")
        .map(PathBuf::from)
        .filter(|p| p.is_file())
}

/// Open STL → New Setup on the Ender-3 V3 KE KCM → Place on Plate (face pick) → Send to Slicer
/// (stand-in), checking the hand-off file; with `AMG_ORCA_SLICER`, OrcaSlicer's CLI `--info`
/// reads it (the CLI only, never the GUI).
#[test]
fn phase1_script_on_three_corpus_models() {
    let dir = temp_dir("script");
    let handoff = configure(&dir);
    let mut ran = 0;
    for rel in SCRIPT_MODELS {
        let Some(path) = amg_testkit::testdata(rel) else {
            continue;
        };
        let bytes = std::fs::read(&path).unwrap();
        let mut h = AppHarness::new();
        h.drive_until_idle();
        let t0 = Instant::now();
        // 1. Open STL.
        let body = import_stl(&mut h, &bytes, rel);
        let m = mesh(&h, body);
        // 2. New setup on the KCM.
        new_setup(&mut h, &[body]);
        enter(&mut h);
        let (_, s) = setup(&h);
        assert_eq!(s.machine.key, MachineKey(KCM.to_owned()));
        // 3. Place face down.
        let (pos, face) = face_under_item(&mut h, body);
        let t = place_face_down(&mut h, body, pos);
        assert_face_down(&m, &t, face, false, rel);
        // 4. Send to Slicer.
        h.run_command(SEND_TO_SLICER);
        h.drive_until_idle();
        let out = layer(&h).last_output().cloned().expect("sent");
        assert!(out.path.starts_with(&handoff));
        let scale = (m.aabb().max - m.aabb().min).length().max(1.0);
        check_export(&out.path, &m, &t, 1e-6 * scale + 1e-4);
        let fit = status(&h, body).message;
        let mut line = format!(
            "{rel}: {} faces, face {face} down, fit {}, {} triangles in {} in {:?}",
            m.face_count(),
            fit.as_deref().unwrap_or("ok"),
            out.triangles,
            out.path.display(),
            t0.elapsed()
        );
        if let Some(exe) = orca_exe() {
            let info = std::process::Command::new(&exe)
                .arg("--info")
                .arg(&out.path)
                .output()
                .unwrap();
            let text = String::from_utf8_lossy(&info.stdout);
            let name = out.path.file_name().unwrap().to_string_lossy().into_owned();
            let n = text.matches(&format!("[{name}]")).count();
            assert_eq!(n, 1, "Orca --info lists one object: {text}");
            line.push_str("; OrcaSlicer --info: 1 object");
        }
        eprintln!("AM-11 Phase 1 script: {line}");
        ran += 1;
    }
    eprintln!("AM-11 Phase 1 script ran on {ran} corpus models");
}
