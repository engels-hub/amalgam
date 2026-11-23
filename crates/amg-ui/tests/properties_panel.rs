//! INS-03: the Properties panel in the real app (`AppHarness`, kittest).
//!
//! - The command is in the INSPECT tab and S-key search, and opens the panel; the viewport
//!   context menu of a mesh body offers it.
//! - A 1000 mm³ cube without a material shows 1.24 g (PLA, "(default material)"); an assigned
//!   material replaces the default and its density is used.
//! - An open mesh shows the area and the reason, no volume-based values.
//! - Several bodies show the total and a section per body.
//! - The centre-of-mass glyph (RND-07 overlay) follows its toggles.
//! - A changed selection recomputes; the result is computed in a job.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "test code"
)]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_core::{Point3, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, Dependency, Evaluated, FeatureBox,
    FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_materials::{LibrarySet, MaterialId};
use amg_mesh::IndexedMesh;
use amg_ui::CommitBatch;
use amg_ui::command::Placement;
use amg_ui::design::tabs::INSPECT;
use amg_ui::inspect::props::mass_text;
use amg_ui::inspect::{PROPERTIES_CMD, PROPERTIES_PANEL};
use amg_ui::marking::{ContextTarget, context_menu_commands};
use amg_ui::pick::PickTarget;
use amg_ui::search::{Recency, SearchIndex};
use amg_ui::select::SelItem;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use serde::{Deserialize, Serialize};

// ---- fixtures ---------------------------------------------------------------------------------

static MESHES: Mutex<Vec<Arc<IndexedMesh>>> = Mutex::new(Vec::new());

/// A mesh source whose mesh lives in a process-wide table (small feature, cheap previews).
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

amg_doc::register_feature!(TestMesh, "uitest.ins03_source", version = 1);

fn link() {
    amg_features::mesh::link();
    amg_features::io::link();
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

fn newest_mesh_body(ev: &Evaluated) -> Option<BodyId> {
    ev.bodies()
        .filter(|(_, b)| matches!(b.geom, BodyGeom::Mesh(_)))
        .map(|(id, _)| id)
        .max()
}

/// An axis-aligned cube with outward, counter-clockwise triangles.
fn cube(min: Point3, s: f64) -> IndexedMesh {
    let v = (0..8)
        .map(|i| {
            min + Vec3::new(
                f64::from(i & 1),
                f64::from((i >> 1) & 1),
                f64::from((i >> 2) & 1),
            ) * s
        })
        .collect();
    let t = vec![
        [0, 2, 3],
        [0, 3, 1],
        [4, 5, 7],
        [4, 7, 6],
        [0, 1, 5],
        [0, 5, 4],
        [2, 6, 7],
        [2, 7, 3],
        [0, 4, 6],
        [0, 6, 2],
        [1, 3, 7],
        [1, 7, 5],
    ];
    IndexedMesh::new(v, t).unwrap()
}

fn translated(mut m: IndexedMesh, d: Vec3) -> IndexedMesh {
    for p in &mut m.positions {
        *p += d;
    }
    IndexedMesh::new(m.positions.clone(), m.tris.clone()).unwrap()
}

fn select(h: &mut AppHarness, bodies: &[BodyId]) {
    let sel = &mut h.app_mut().active_document_mut().selection;
    sel.clear();
    for &body in bodies {
        sel.add(
            SelItem::from_target(PickTarget::Body {
                body,
                instance: 0,
                sub: None,
            })
            .unwrap(),
        );
    }
}

// ---- helpers ------------------------------------------------------------------------------------

/// Steps until a label containing `text` is shown (the panel's job finished), at most ~20 s.
fn wait_for(h: &mut AppHarness, text: &str) {
    for _ in 0..4000 {
        h.kittest().step();
        if h.kittest()
            .query_all_by_label_contains(text)
            .next()
            .is_some()
        {
            return;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("no label containing {text:?} appeared");
}

fn shown(h: &mut AppHarness, text: &str) -> bool {
    h.kittest().step();
    h.kittest()
        .query_all_by_label_contains(text)
        .next()
        .is_some()
}

fn open(h: &mut AppHarness) {
    h.run_command(PROPERTIES_CMD);
    h.drive_until_idle();
    h.kittest().step();
}

fn click_checkbox(h: &mut AppHarness, label: &str) {
    h.kittest().step();
    h.kittest()
        .get_by_role_and_label(egui::accesskit::Role::CheckBox, label)
        .click();
    h.kittest().step();
    h.kittest().step();
}

// ---- command ------------------------------------------------------------------------------------

#[test]
fn properties_is_in_the_inspect_tab_and_search_and_opens_its_panel() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    {
        let reg = h.app().registry();
        let def = reg.command(PROPERTIES_CMD).expect("registered");
        assert!(def.placements.iter().any(|p| matches!(
            p,
            Placement::Toolbar { group, .. } if reg.group(*group).map(|g| g.tab) == Some(INSPECT)
        )));
        assert!(reg.panel(PROPERTIES_PANEL).is_some());
        let index = SearchIndex::build(reg);
        for q in ["Properties", "mass", "inertia"] {
            assert!(
                index
                    .search(q, &Recency::new())
                    .iter()
                    .any(|x| x.id == PROPERTIES_CMD),
                "search {q}"
            );
        }
    }
    // Disabled without a mesh body.
    open(&mut h);
    assert!(!shown(&mut h, "Material:"));
    add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    // From its toolbar button (INSPECT tab).
    h.app_mut().select_tab(INSPECT);
    h.kittest().step();
    h.kittest().step();
    h.kittest().get_by_label("Properties").click();
    h.kittest().step();
    h.drive_until_idle();
    wait_for(&mut h, "Mass:");
}

#[test]
fn the_context_menu_of_a_mesh_body_offers_properties() {
    let mut h = AppHarness::new();
    let body = add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    let item = SelItem::from_target(PickTarget::Body {
        body,
        instance: 0,
        sub: None,
    })
    .unwrap();
    let cmds = h.app_mut().with_cx(|cx| {
        let env = cx.env();
        context_menu_commands(cx.registry(), &env, &ContextTarget::Viewport(Some(&item)))
    });
    assert!(cmds.contains(&PROPERTIES_CMD), "{cmds:?}");
    let none = h.app_mut().with_cx(|cx| {
        let env = cx.env();
        context_menu_commands(cx.registry(), &env, &ContextTarget::Viewport(None))
    });
    assert!(!none.contains(&PROPERTIES_CMD));
}

// ---- values ---------------------------------------------------------------------------------

/// Done-when 7: the mass of a 1000 mm³ PLA cube (no material: the default material) is 1.24 g.
#[test]
fn a_1000_mm3_cube_without_material_shows_1_24_g_of_default_pla() {
    let mut h = AppHarness::new();
    add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    open(&mut h);
    wait_for(&mut h, "Mass: 1.24 g (default material)");
    assert!(shown(&mut h, "Volume: 1000 mm³"));
    assert!(shown(&mut h, "Area: 600 mm²"));
    assert!(shown(&mut h, "Density: 1.24 g/cm³ (default material)"));
    assert!(shown(&mut h, "Centre of mass: (5, 5, 5) mm"));
    assert!(shown(&mut h, "Material: "));
    // The cube's inertia: m·(a²+a²)/12 = 1.24e-6 t · 200/12 mm² = 0.020667 kg·mm² per axis.
    assert!(shown(
        &mut h,
        "Inertia about CoM (Ixx, Iyy, Izz): 0.020667, 0.020667, 0.020667 kg·mm²"
    ));
}

#[test]
fn an_assigned_material_replaces_the_default() {
    let mut h = AppHarness::new();
    let body = add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    let libs = LibrarySet::builtin_only();
    let id = MaterialId::parse("amg.builtin/petg.fff").unwrap();
    let petg = libs.material(&id).expect("built-in PETG").clone();
    let commands =
        amg_doc::assign_material_by_id(h.document().design(), &libs, &[body], id.as_str()).unwrap();
    h.app_mut().submit(CommitBatch {
        label: "Assign".into(),
        commands,
    });
    h.drive_until_idle();
    open(&mut h);
    let mass = mass_text(petg.density.get() * 1000.0);
    wait_for(&mut h, &format!("Mass: {mass}"));
    assert!(!shown(&mut h, "(default material)"));
    assert!(shown(&mut h, &format!("Material: {}", petg.name)));
}

#[test]
fn an_open_mesh_shows_the_area_and_the_reason_but_no_volume() {
    let mut h = AppHarness::new();
    let c = cube(Point3::ZERO, 10.0);
    let mut tris = c.tris.clone();
    tris.pop();
    add_mesh(&mut h, IndexedMesh::new(c.positions.clone(), tris).unwrap());
    open(&mut h);
    wait_for(&mut h, "Area: 550 mm²");
    assert!(shown(&mut h, "Volume: – (not watertight)"));
    assert!(shown(&mut h, "need a closed mesh"));
    assert!(!shown(&mut h, "Mass:"));
    assert!(!shown(&mut h, "Centre of mass"));
}

#[test]
fn several_bodies_show_the_total_and_a_section_per_body() {
    let mut h = AppHarness::new();
    let a = add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    let b = add_mesh(
        &mut h,
        translated(cube(Point3::ZERO, 20.0), Vec3::new(100.0, 0.0, 0.0)),
    );
    select(&mut h, &[a, b]);
    open(&mut h);
    // Total volume 1000 + 8000, mass 9 cm³ of PLA = 11.16 g, CoM mass-weighted.
    wait_for(&mut h, "Volume: 9000 mm³");
    assert!(shown(&mut h, "Mass: 11.16 g (default material)"));
    // x = (5·1000 + 110·8000) / 9000 = 98.333, y = (5·1000 + 10·8000) / 9000 = 9.4444.
    assert!(shown(&mut h, "Centre of mass: (98.333, 9.4444"));
    assert!(shown(&mut h, "2 bodies"));
    assert!(shown(&mut h, &format!("Body {}.{}", a.origin.0, a.slot)));
    assert!(shown(&mut h, &format!("Body {}.{}", b.origin.0, b.slot)));
    // Selecting only the first body recomputes.
    select(&mut h, &[a]);
    wait_for(&mut h, "Volume: 1000 mm³");
    assert!(!shown(&mut h, "2 bodies"));
}

// ---- glyph and job --------------------------------------------------------------------------

#[test]
fn the_centre_of_mass_glyph_follows_its_toggle() {
    let mut h = AppHarness::new();
    let a = add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    open(&mut h);
    wait_for(&mut h, "Mass:");
    // Off by default.
    assert_eq!(h.app().tool_overlays().primitive_count(), 0);
    click_checkbox(&mut h, "Show centre of mass");
    // Three rings and a point, and a label.
    let o = h.app().tool_overlays();
    assert_eq!(o.primitive_count(), 4, "rings + point");
    assert_eq!(o.label_count(), 1);
    click_checkbox(&mut h, "Show centre of mass");
    assert_eq!(h.app().tool_overlays().primitive_count(), 0);
    // Per body with several bodies.
    let b = add_mesh(
        &mut h,
        translated(cube(Point3::ZERO, 10.0), Vec3::new(50.0, 0.0, 0.0)),
    );
    select(&mut h, &[a, b]);
    wait_for(&mut h, "2 bodies");
    h.kittest()
        .get_by_label(&format!("Body {}.{}", a.origin.0, a.slot))
        .click();
    h.kittest().step();
    click_checkbox(&mut h, "Show centre of mass of this body");
    assert_eq!(h.app().tool_overlays().primitive_count(), 4);
}

/// The result is computed in a background job: opening the panel on a large body returns to
/// the frame loop at once (timing assert only under `AMG_STRICT_TIMING=1`).
#[test]
fn large_bodies_are_computed_off_the_ui_thread() {
    let strict = std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1");
    let s = amg_testkit::shapes::icosphere(Point3::ZERO, 50.0, if strict { 8 } else { 6 });
    let m = IndexedMesh::new(s.positions, s.triangles).unwrap();
    let faces = m.face_count();
    let mut h = AppHarness::new();
    add_mesh(&mut h, m);
    h.run_command(PROPERTIES_CMD);
    let t0 = Instant::now();
    h.kittest().step();
    h.kittest().step();
    let frames = t0.elapsed();
    eprintln!("{faces} faces: two frames after opening took {frames:?}");
    if strict {
        assert!(frames < Duration::from_millis(32), "{frames:?}");
    }
    wait_for(&mut h, "Mass:");
}
