//! UI-04 tests: window / crossing box selection, the selection-filter bar and select priority,
//! select-through, selection sets through save / reload / undo, and revalidation when a
//! selected body is deleted.
//!
//! Everything runs the real app in `AppHarness` (kittest) with `CpuPickService` and mesh bodies
//! built by a test feature, seen from above.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use amg_core::{Dir3, Point3};
use amg_doc::stub::IndexedMesh;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, Dependency, Document, EntityRef,
    FeatureBox, FeatureError, FeatureOutput, RecomputeCtx, TopoKind,
};
use amg_render::{Camera, Projection};
use amg_ui::CommitBatch;
use amg_ui::command::CmdId;
use amg_ui::file::FileOp;
use amg_ui::pick::PickTarget;
use amg_ui::scene::HighlightLayer;
use amg_ui::select::sets::{SELECTION_SETS_KIND, selection_sets};
use amg_ui::select::{EntityFilter, FILTER_GROUPS, LONG_PRESS, SelectPriority};
use amg_ui::testing::egui_kittest::kittest::Queryable;
use amg_ui::testing::{AppHarness, FakeFileDialog};
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- test feature ----------------------------------------------------------------------------

/// A flat `n × n` grid of 1 mm cells at height `z` from `(x0, y0)`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct SelGrid {
    n: u32,
    x0: f64,
    y0: f64,
    z: f64,
}

fn grid_mesh(g: &SelGrid) -> IndexedMesh {
    let mut pos = Vec::new();
    for j in 0..=g.n {
        for i in 0..=g.n {
            pos.push(Point3::new(g.x0 + f64::from(i), g.y0 + f64::from(j), g.z));
        }
    }
    let w = g.n + 1;
    let mut tris = Vec::new();
    for j in 0..g.n {
        for i in 0..g.n {
            let a = j * w + i;
            tris.push([a, a + 1, a + w + 1]);
            tris.push([a, a + w + 1, a + w]);
        }
    }
    IndexedMesh::new(pos, tris).expect("valid grid")
}

impl amg_doc::Feature for SelGrid {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(grid_mesh(self))),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(SelGrid, "uitest.sel_grid", version = 1);

fn grid(n: u32, x0: f64, y0: f64, z: f64) -> SelGrid {
    SelGrid { n, x0, y0, z }
}

fn add_feature(f: FeatureBox, doc: &Document) -> Command {
    Command::AddFeature {
        at: None,
        feature: f,
        context: doc.design().root(),
        name: None,
    }
}

fn top_down(x: f64, y: f64, height: f64) -> Camera {
    Camera {
        eye: Point3::new(x, y, height),
        target: Point3::new(x, y, 0.0),
        up: Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    }
}

/// A harness with `grids` (one feature each), seen from above at `(cx, cy)`. Returns the
/// bodies in feature order.
fn harness(grids: &[SelGrid], cx: f64, cy: f64, height: f64) -> (AppHarness, Vec<BodyId>) {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    for g in grids {
        let cmd = add_feature(FeatureBox::new(g.clone()), h.document());
        h.app_mut().submit(CommitBatch {
            label: "Grid".into(),
            commands: vec![cmd],
        });
        h.drive_until_idle();
    }
    h.app_mut().active_document_mut().camera = top_down(cx, cy, height);
    h.drive_until_idle();
    let bodies: Vec<BodyId> = h.scene().bodies().collect();
    assert_eq!(bodies.len(), grids.len());
    (h, bodies)
}

fn whole(body: BodyId) -> PickTarget {
    PickTarget::Body {
        body,
        instance: 0,
        sub: None,
    }
}

fn selected(h: &AppHarness) -> Vec<PickTarget> {
    h.app()
        .active_document()
        .selection
        .items()
        .iter()
        .map(|i| i.target)
        .collect()
}

fn selected_entities(h: &AppHarness) -> Vec<EntityRef> {
    h.app()
        .active_document()
        .selection
        .items()
        .iter()
        .map(|i| i.entity.clone())
        .collect()
}

fn screen(h: &AppHarness, x: f64, y: f64) -> egui::Pos2 {
    h.world_to_screen(Point3::new(x, y, 0.0))
}

fn no_errors(h: &AppHarness) -> bool {
    h.app()
        .notifications()
        .items()
        .iter()
        .all(|t| t.notification.severity != amg_core::Severity::Error)
}

/// Sets the filter bar's group toggles to `wanted` by clicking them (the bar shows while the
/// pointer is over the viewport).
fn set_filter_via_bar(h: &mut AppHarness, wanted: EntityFilter) {
    let c = h.camera_view().rect.center();
    h.hover_at(c);
    h.kittest().run();
    for (label, bits) in FILTER_GROUPS {
        let on = h
            .app()
            .active_document()
            .selection
            .filter()
            .intersects(*bits);
        if on != wanted.intersects(*bits) {
            h.kittest()
                .get_by_role_and_label(Role::Button, label)
                .click();
            h.kittest().run();
        }
    }
    let got = h.app().active_document().selection.filter();
    for (label, bits) in FILTER_GROUPS {
        assert_eq!(got.intersects(*bits), wanted.intersects(*bits), "{label}");
    }
}

// ---- Done-when 1: window and crossing boxes ----------------------------------------------------

#[test]
fn window_box_picks_enclosed_bodies_and_crossing_box_also_touching_ones() {
    // Three 4 × 4 mm bodies at x = 0, 6 and 12.
    let (mut h, b) = harness(
        &[
            grid(4, 0.0, 0.0, 0.0),
            grid(4, 6.0, 0.0, 0.0),
            grid(4, 12.0, 0.0, 0.0),
        ],
        8.0,
        2.0,
        40.0,
    );
    // The box spans x ∈ [-1, 8], y ∈ [-1, 5]: all of body 0, part of body 1, none of body 2.
    let top_left = screen(&h, -1.0, 5.0);
    let bottom_right = screen(&h, 8.0, -1.0);
    assert!(h.camera_view().rect.contains(top_left) && h.camera_view().rect.contains(bottom_right));
    let prim = egui::PointerButton::Primary;

    // Left to right: window.
    h.drag(prim, egui::Modifiers::NONE, top_left, bottom_right, 6);
    h.drive_until_idle();
    assert_eq!(selected(&h), [whole(b[0])], "window: fully enclosed only");
    assert_eq!(h.scene().highlight(HighlightLayer::Selected).len(), 1);

    // Right to left over the same rectangle: crossing.
    h.drag(prim, egui::Modifiers::NONE, bottom_right, top_left, 6);
    h.drive_until_idle();
    assert_eq!(
        selected(&h),
        [whole(b[0]), whole(b[1])],
        "crossing: touching too"
    );
    assert_eq!(h.scene().highlight(HighlightLayer::Selected).len(), 2);

    // A window around body 2 with Shift adds it; without a modifier a box replaces.
    let a = screen(&h, 11.0, 5.0);
    let z = screen(&h, 17.0, -1.0);
    h.drag(prim, egui::Modifiers::SHIFT, a, z, 6);
    h.drive_until_idle();
    assert_eq!(selected(&h), [whole(b[0]), whole(b[1]), whole(b[2])]);
    h.drag(prim, egui::Modifiers::NONE, a, z, 6);
    h.drive_until_idle();
    assert_eq!(selected(&h), [whole(b[2])]);

    // A box drag never moved the camera, and click rules still hold.
    let mid = screen(&h, 2.0, 2.5);
    h.click_at(mid, prim, egui::Modifiers::CTRL);
    h.drive_until_idle();
    assert_eq!(
        selected(&h),
        [whole(b[2]), whole(b[0])],
        "Ctrl+click toggles on"
    );
    h.click_at(mid, prim, egui::Modifiers::CTRL);
    h.drive_until_idle();
    assert_eq!(selected(&h), [whole(b[2])], "Ctrl+click toggles off");
    h.click_at(mid, prim, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert_eq!(selected(&h), [whole(b[0])], "click replaces");
    h.click_at(screen(&h, 5.0, 10.0), prim, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert!(selected(&h).is_empty(), "click on empty space clears");
}

// ---- Done-when 2: faces-only filter, a click on an edge selects the adjacent face ---------------

#[test]
fn faces_only_filter_turns_a_click_on_an_edge_into_the_adjacent_face() {
    let (mut h, b) = harness(&[grid(4, 0.0, 0.0, 0.0)], 2.0, 2.0, 30.0);
    let body = b[0];
    // The diagonal of cell (1, 1) runs from (1, 1) to (2, 2); click its midpoint.
    let (p0, p1) = (Point3::new(1.0, 1.0, 0.0), Point3::new(2.0, 2.0, 0.0));
    let on_edge = screen(&h, 1.5, 1.5);
    let prim = egui::PointerButton::Primary;

    // Edges only: the click selects that edge (so the point is on an edge).
    set_filter_via_bar(&mut h, EntityFilter::EDGE);
    h.click_at(on_edge, prim, egui::Modifiers::NONE);
    h.drive_until_idle();
    let edge = match selected(&h).as_slice() {
        [
            PickTarget::Body {
                body: got,
                sub: Some((TopoKind::Edge, e)),
                ..
            },
        ] if *got == body => *e,
        other => panic!("expected one edge, got {other:?}"),
    };
    let mesh = grid_mesh(&grid(4, 0.0, 0.0, 0.0));
    let (a, c) = amg_ui::select::naming::mesh_edge(&mesh, edge).unwrap();
    assert!(
        (a == p0 && c == p1) || (a == p1 && c == p0),
        "the diagonal of cell (1, 1)"
    );
    assert!(matches!(
        &selected_entities(&h)[0],
        EntityRef::Topo(r) if r.kind == TopoKind::Edge && r.body == body
    ));

    // Faces only: the same click selects a face adjacent to that edge.
    set_filter_via_bar(&mut h, EntityFilter::FACE);
    h.click_at(on_edge, prim, egui::Modifiers::NONE);
    h.drive_until_idle();
    let face = match selected(&h).as_slice() {
        [
            PickTarget::Body {
                body: got,
                sub: Some((TopoKind::Face, f)),
                ..
            },
        ] if *got == body => *f,
        other => panic!("expected one face, got {other:?}"),
    };
    let corners = mesh.tris[face as usize].map(|i| mesh.positions[i as usize]);
    assert!(
        corners.contains(&p0) && corners.contains(&p1),
        "face {face} is adjacent to the clicked edge"
    );
    assert!(matches!(
        &selected_entities(&h)[0],
        EntityRef::Topo(r) if r.kind == TopoKind::Face && r.body == body
    ));
    assert_eq!(
        h.scene().highlight(HighlightLayer::Selected).len(),
        1,
        "the face is highlighted"
    );

    // Face priority with every kind allowed does the same; Body priority selects the body.
    set_filter_via_bar(&mut h, EntityFilter::ALL);
    h.kittest()
        .get_by_role_and_label(Role::Button, "Face")
        .click();
    h.kittest().run();
    assert_eq!(
        h.app().active_document().selection.priority(),
        SelectPriority::Face
    );
    h.click_at(on_edge + egui::vec2(0.5, 0.0), prim, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert!(matches!(
        selected(&h).as_slice(),
        [PickTarget::Body {
            sub: Some((TopoKind::Face, f)),
            ..
        }] if *f == face || *f == (face ^ 1)
    ));
    h.kittest()
        .get_by_role_and_label(Role::Button, "Body")
        .click();
    h.kittest().run();
    h.click_at(on_edge, prim, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert_eq!(selected(&h), [whole(body)]);
    h.kittest().get_by_label("Reset filter").click();
    h.kittest().run();
    let sel = &h.app().active_document().selection;
    assert_eq!(
        (sel.filter(), sel.priority()),
        (EntityFilter::ALL, SelectPriority::Default)
    );
}

// ---- Done-when 3: select-through -----------------------------------------------------------------

#[test]
fn long_press_lists_all_three_stacked_bodies_in_depth_order() {
    let (mut h, b) = harness(
        &[
            grid(2, 0.0, 0.0, 0.0),
            grid(2, 0.0, 0.0, 4.0),
            grid(2, 0.0, 0.0, 8.0),
        ],
        1.0,
        1.0,
        40.0,
    );
    // Away from the triangle edges, so the CPU pick reports faces.
    let p = h.world_to_screen(Point3::new(0.7, 0.3, 8.0));
    let prim = egui::PointerButton::Primary;
    h.hover_at(p);
    h.drive_until_idle();
    h.button(p, prim, true, egui::Modifiers::NONE);
    let pressed = Instant::now();
    // A short hold does nothing.
    h.kittest().step();
    assert!(h.app().select_through().is_none());
    std::thread::sleep(
        LONG_PRESS.saturating_sub(pressed.elapsed()) + std::time::Duration::from_millis(30),
    );
    let mut frames = 0;
    while h.app().select_through().is_none() {
        assert!(frames < 20, "the select-through list opens");
        h.kittest().step();
        frames += 1;
    }
    let list = h.app().select_through().unwrap().clone();
    let bodies: Vec<PickTarget> = list.entries.iter().map(|e| e.item.target).collect();
    assert_eq!(
        bodies,
        [whole(b[2]), whole(b[1]), whole(b[0])],
        "nearest (z = 8) first"
    );
    assert!(
        list.entries.windows(2).all(|w| w[0].depth < w[1].depth),
        "depth order"
    );
    // The release that ends the long press is not a click.
    h.button(p, prim, false, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert!(selected(&h).is_empty());
    // Every entry is in the list by label; choosing the middle one selects it.
    for e in &list.entries {
        assert!(
            h.kittest().query_by_label(&e.label).is_some(),
            "{}",
            e.label
        );
    }
    h.kittest().get_by_label(&list.entries[1].label).click();
    h.kittest().run();
    assert_eq!(selected(&h), [whole(b[1])]);
    assert!(h.app().select_through().is_none(), "the list closes");

    // Esc closes an open list without clearing the selection.
    h.hover_at(p);
    h.button(p, prim, true, egui::Modifiers::NONE);
    std::thread::sleep(LONG_PRESS + std::time::Duration::from_millis(30));
    for _ in 0..20 {
        if h.app().select_through().is_some() {
            break;
        }
        h.kittest().step();
    }
    h.button(p, prim, false, egui::Modifiers::NONE);
    assert!(h.app().select_through().is_some());
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    assert!(h.app().select_through().is_none());
    assert_eq!(selected(&h), [whole(b[1])]);
}

// ---- Done-when 4: selection sets survive save / reload, and undo removes them ------------------

fn temp_dir(tag: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("amg-ui04-{tag}-{}-{nanos}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn selection_set_round_trips_through_save_and_reload_and_undo_removes_it() {
    let (mut h, b) = harness(
        &[
            grid(4, 0.0, 0.0, 0.0),
            grid(4, 6.0, 0.0, 0.0),
            grid(4, 12.0, 0.0, 0.0),
        ],
        8.0,
        2.0,
        40.0,
    );
    let fake = FakeFileDialog::new();
    h.app_mut().set_file_dialog_backend(fake.clone());
    let prim = egui::PointerButton::Primary;
    // Body 0 whole, and a face of body 2 (face priority, Ctrl adds).
    h.click_at(screen(&h, 2.3, 1.6), prim, egui::Modifiers::NONE);
    h.drive_until_idle();
    h.app_mut()
        .active_document_mut()
        .selection
        .set_priority(SelectPriority::Face);
    h.click_at(screen(&h, 14.3, 2.6), prim, egui::Modifiers::CTRL);
    h.drive_until_idle();
    let saved = selected_entities(&h);
    assert_eq!(saved.len(), 2);
    assert_eq!(saved[0], EntityRef::Body(b[0]));
    assert!(matches!(&saved[1], EntityRef::Topo(r) if r.body == b[2] && r.kind == TopoKind::Face));

    // Save the set: one undo entry; undo removes it, redo brings it back.
    let undo_depth = h.document().history().stack().applied();
    h.run_command(CmdId("select.save_set"));
    h.drive_until_idle();
    assert_eq!(h.document().history().stack().applied(), undo_depth + 1);
    let sets = selection_sets(h.document().design());
    assert_eq!(sets.len(), 1);
    assert_eq!(sets[0].name, "Selection Set 1");
    assert_eq!(sets[0].items, saved);
    h.app_mut().with_cx(|cx| cx.undo());
    h.drive_until_idle();
    assert!(
        selection_sets(h.document().design()).is_empty(),
        "undo removes the set"
    );
    assert!(
        !h.document()
            .design()
            .exts()
            .any(|(k, _)| k.kind == SELECTION_SETS_KIND),
        "and its record"
    );
    h.app_mut().with_cx(|cx| cx.redo());
    h.drive_until_idle();
    assert_eq!(selection_sets(h.document().design()).len(), 1);

    // Save the file, close it, open it again.
    let dir = temp_dir("sets");
    let path = dir.join("sets.amg");
    fake.answer(Some(path.clone()));
    h.run_command(CmdId("file.save_as"));
    h.drive_until_idle();
    assert!(path.exists());
    h.run_command(CmdId("file.close"));
    h.drive_until_idle();
    h.app_mut().file_op(None, FileOp::OpenPath(path.clone()));
    h.drive_until_idle();
    assert_eq!(
        h.app().active_document().path.as_deref(),
        Some(path.as_path())
    );
    assert!(h.app().active_document().selection.is_empty());
    assert_eq!(h.scene().bodies().count(), 3);

    // Restore gives the same references, mapped to displayed targets.
    let missing = h.app_mut().restore_selection_set("Selection Set 1");
    assert_eq!(missing, Some(0));
    assert_eq!(selected_entities(&h), saved);
    let targets = selected(&h);
    assert_eq!(targets[0], whole(b[0]));
    assert!(matches!(
        targets[1],
        PickTarget::Body { body, sub: Some((TopoKind::Face, _)), .. } if body == b[2]
    ));
    h.drive_until_idle();
    assert_eq!(h.scene().highlight(HighlightLayer::Selected).len(), 2);
    assert!(h.app_mut().restore_selection_set("No such set").is_none());
    assert!(no_errors(&h));
    let _ = std::fs::remove_dir_all(dir);
}

// ---- Done-when 5: deleting a selected body drops it ------------------------------------------------

#[test]
fn deleting_a_selected_body_drops_it_from_the_selection_without_error() {
    let (mut h, b) = harness(
        &[
            grid(4, 0.0, 0.0, 0.0),
            grid(4, 6.0, 0.0, 0.0),
            grid(4, 12.0, 0.0, 0.0),
        ],
        8.0,
        2.0,
        40.0,
    );
    let prim = egui::PointerButton::Primary;
    // Crossing over everything selects the three bodies; a face of body 1 is added too.
    let a = screen(&h, 17.0, -1.0);
    let z = screen(&h, -1.0, 5.0);
    h.drag(prim, egui::Modifiers::NONE, a, z, 6);
    h.drive_until_idle();
    assert_eq!(selected(&h), [whole(b[0]), whole(b[1]), whole(b[2])]);
    h.app_mut()
        .active_document_mut()
        .selection
        .set_priority(SelectPriority::Face);
    h.click_at(screen(&h, 6.3, 1.6), prim, egui::Modifiers::CTRL);
    h.drive_until_idle();
    assert_eq!(selected(&h).len(), 4);

    // Delete body 1's feature: it and its face leave the selection.
    h.app_mut().submit(CommitBatch {
        label: "Delete".into(),
        commands: vec![Command::DeleteFeatures {
            ids: vec![b[1].origin],
        }],
    });
    h.drive_until_idle();
    assert_eq!(h.scene().bodies().count(), 2);
    assert_eq!(selected(&h), [whole(b[0]), whole(b[2])]);
    assert_eq!(h.scene().highlight(HighlightLayer::Selected).len(), 2);
    assert!(no_errors(&h), "no error toast");

    // Undo brings the body back but not into the selection; re-opening clears it.
    h.app_mut().with_cx(|cx| cx.undo());
    h.drive_until_idle();
    assert_eq!(h.scene().bodies().count(), 3);
    assert_eq!(selected(&h), [whole(b[0]), whole(b[2])]);
    let design = h.document().design().clone();
    h.app_mut().active_document_mut().document = Document::from_design(design);
    h.drive_until_idle();
    assert!(
        selected(&h).is_empty(),
        "DocumentReplaced clears the selection"
    );
    assert!(no_errors(&h));
}
