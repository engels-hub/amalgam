//! MESH-27: the Combine dialog of the MESH tab in the real app (`AppHarness`, kittest).
//!
//! - Combine is a MESH toolbar command (Modify group) found by S-key search;
//! - preselected bodies fill Target (first mesh body) and Tools; OK needs a tool, previews as
//!   a job and commits one undoable "Mesh Combine" feature;
//! - an implicit operand disables OK with "stays implicit" and never shows the "converted to
//!   mesh" note; a solid tool into a mesh target shows that note.
//!
//! The boolean itself needs the `manifold` feature; without it the committed feature reports
//! the backend error and the tests check that instead.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "test code"
)]

use std::sync::{Arc, Mutex};

use amg_core::Point3;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, Dependency, Evaluated, FeatureBox,
    FeatureError, FeatureOutput, Health, RecomputeCtx,
};
use amg_mesh::IndexedMesh;
use amg_ui::CommitBatch;
use amg_ui::command::Placement;
use amg_ui::mesh::combine::{CONVERTED_NOTE, IMPLICIT_NOTE};
use amg_ui::mesh::{MESH_TAB, MeshOp};
use amg_ui::pick::PickTarget;
use amg_ui::preview::PreviewState;
use amg_ui::search::{Recency, SearchIndex};
use amg_ui::select::SelItem;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use serde::{Deserialize, Serialize};

static MESHES: Mutex<Vec<Arc<IndexedMesh>>> = Mutex::new(Vec::new());

/// A body source: a mesh from a process-wide table, a data-less implicit body or a
/// placeholder solid.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
enum Src {
    Mesh(usize),
    Implicit,
    Solid,
}

impl amg_doc::Feature for Src {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let geom = match self {
            Src::Mesh(k) => BodyGeom::Mesh(Arc::clone(&MESHES.lock().unwrap()[*k])),
            Src::Implicit => BodyGeom::Implicit(Arc::new(amg_doc::stub::SdfGraph::default())),
            Src::Solid => BodyGeom::Solid(amg_doc::stub::Shape::new()),
        };
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created { geom, naming: None },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(Src, "uitest.mesh27_source", version = 1);

fn add(h: &mut AppHarness, src: Src) -> BodyId {
    amg_features::mesh::link();
    let root = h.document().design().root();
    h.app_mut().submit(CommitBatch {
        label: "Source".into(),
        commands: vec![Command::AddFeature {
            at: None,
            feature: FeatureBox::new(src),
            context: root,
            name: None,
        }],
    });
    h.drive_until_idle();
    let ev = h.document().evaluated();
    ev.bodies().map(|(id, _)| id).max().expect("a body")
}

fn add_mesh(h: &mut AppHarness, m: IndexedMesh) -> BodyId {
    let key = {
        let mut t = MESHES.lock().unwrap();
        t.push(Arc::new(m));
        t.len() - 1
    };
    add(h, Src::Mesh(key))
}

fn cube(min: Point3, size: f64) -> IndexedMesh {
    let s = amg_testkit::shapes::cube(min, size);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn mesh(ev: &Evaluated, body: BodyId) -> Option<Arc<IndexedMesh>> {
    match &ev.body(body)?.geom {
        BodyGeom::Mesh(m) => Some(Arc::clone(m)),
        _ => None,
    }
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

fn start(h: &mut AppHarness) {
    h.run_command(MeshOp::Combine.cmd());
    h.drive_until_idle();
    h.kittest().step();
    assert_eq!(h.app().active_tool(), Some(MeshOp::Combine.cmd()));
}

fn esc(h: &mut AppHarness) {
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None);
}

fn shown(h: &mut AppHarness, text: &str) -> bool {
    h.kittest()
        .query_all_by_label_contains(text)
        .next()
        .is_some()
}

#[test]
fn combine_is_in_the_modify_group_and_found_by_search() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let reg = h.app().registry();
    let id = MeshOp::Combine.cmd();
    let def = reg.command(id).expect("mesh.combine registered");
    assert_eq!(def.label, "Combine…");
    let group = def
        .placements
        .iter()
        .find_map(|p| match p {
            Placement::Toolbar { group, .. } => Some(*group),
            _ => None,
        })
        .expect("toolbar placement");
    assert_eq!(group, amg_ui::mesh::ops::GROUP_MODIFY);
    assert_eq!(reg.group(group).map(|g| g.tab), Some(MESH_TAB));
    let index = SearchIndex::build(reg);
    let recency = Recency::new();
    assert_eq!(
        index.search("Combine", &recency).first().map(|x| x.id),
        Some(id)
    );
    for k in ["boolean", "join", "subtract", "intersect"] {
        assert!(index.search(k, &recency).iter().any(|x| x.id == id), "{k}");
    }
}

#[test]
fn combine_dialog_needs_a_tool_previews_and_commits_one_undoable_feature() {
    let mut h = AppHarness::new();
    let a = add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    let b = add_mesh(&mut h, cube(Point3::new(5.0, 0.0, 0.0), 10.0));
    let nodes = h.document().design().timeline().nodes().count();

    // Only the target: OK is disabled with the reason.
    select(&mut h, &[a]);
    start(&mut h);
    assert!(shown(&mut h, "Combine"), "dialog titled");
    assert!(shown(&mut h, "Select the tool bodies"));
    assert!(!shown(&mut h, CONVERTED_NOTE));
    esc(&mut h);
    assert_eq!(h.document().design().timeline().nodes().count(), nodes);

    // Target and tool preselected: the preview runs as a job, OK commits.
    select(&mut h, &[a, b]);
    start(&mut h);
    assert!(
        matches!(
            h.app_mut().preview_driver().state(),
            PreviewState::Ready(_) | PreviewState::Failed(_)
        ),
        "the preview ran"
    );
    let v0 = mesh(h.document().evaluated(), a).unwrap().signed_volume();
    assert!(
        (v0 - 1000.0).abs() < 1e-9,
        "the document is untouched until OK: {v0}"
    );
    h.kittest().step();
    h.kittest().get_by_label("OK").click();
    h.kittest().step();
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(h.document().undo_label(), Some("Mesh Combine"));
    let doc = h.document();
    let ids: Vec<_> = doc.design().timeline().nodes().map(|n| n.id).collect();
    assert_eq!(ids.len(), nodes + 1);
    let health = doc
        .evaluated()
        .node(*ids.last().unwrap())
        .unwrap()
        .health
        .clone();
    if cfg!(feature = "manifold") {
        assert_eq!(health, Health::Ok);
        let v = mesh(doc.evaluated(), a).unwrap().signed_volume();
        assert!((v - 1500.0).abs() < 1e-9, "{v}");
        assert!(doc.evaluated().body(b).is_none(), "tool deleted");
    } else {
        assert!(
            matches!(&health, Health::Error(d) if d.message.contains("boolean backend unavailable")),
            "{health:?}"
        );
    }

    // Undo restores both bodies.
    let doc_id = h.app().active_document().id;
    h.app_mut().request_undo(doc_id);
    h.drive_until_idle();
    let ev = h.document().evaluated();
    let v1 = mesh(ev, a).unwrap().signed_volume();
    assert!((v1 - 1000.0).abs() < 1e-9, "{v1}");
    assert!(mesh(ev, b).is_some());
}

#[test]
fn an_implicit_operand_disables_ok_and_shows_no_conversion_note() {
    let mut h = AppHarness::new();
    let m = add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    let imp = add(&mut h, Src::Implicit);
    let solid = add(&mut h, Src::Solid);
    let nodes = h.document().design().timeline().nodes().count();
    assert!(IMPLICIT_NOTE.contains("stays implicit"));

    // Mesh target, implicit tool (and a solid one): stays implicit, no conversion note.
    select(&mut h, &[m, imp, solid]);
    start(&mut h);
    assert!(shown(&mut h, "stays implicit"));
    assert!(!shown(&mut h, CONVERTED_NOTE));
    assert!(
        !matches!(h.app_mut().preview_driver().state(), PreviewState::Ready(_)),
        "no preview for an implicit operand"
    );
    // OK does nothing (disabled).
    h.kittest().get_by_label("OK").click();
    h.kittest().step();
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(MeshOp::Combine.cmd()));
    esc(&mut h);
    assert_eq!(h.document().design().timeline().nodes().count(), nodes);
    assert!(matches!(
        h.document().evaluated().body(imp).unwrap().geom,
        BodyGeom::Implicit(_)
    ));
}

#[test]
fn a_solid_tool_into_a_mesh_target_shows_the_conversion_note() {
    let mut h = AppHarness::new();
    let m = add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    let solid = add(&mut h, Src::Solid);
    select(&mut h, &[m, solid]);
    start(&mut h);
    assert!(shown(&mut h, CONVERTED_NOTE));
    assert!(!shown(&mut h, "stays implicit"));
    esc(&mut h);
}
