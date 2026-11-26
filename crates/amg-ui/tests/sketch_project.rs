//! UI-17: Project / Include / Intersect in sketch mode. Kittest on the real app with the
//! `NullViewport` and `CpuPickService`.
//!
//! - Done-when 1 (`occt` module, needs AMG_OCCT_DIR): the Project tool picks the front face of an
//!   extruded box (SOL-01/02), the face's four edges come in as locked lines, a circle is
//!   dimensioned to them with Sketch Dimension, and editing the extrude (while the sketch is
//!   open, then through the Extrude dialog after Finish Sketch) moves the projected edges and
//!   the circle.
//! - Done-when 2: projected entities cannot be dragged (both builds; the default build projects
//!   a line of an earlier sketch, which needs no geometry kernel), and they are drawn in the
//!   locked colour. The open session also follows an upstream sketch edit there.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test code"
)]

use amg_core::{Dir3, Point2, Point3};
use amg_doc::stub::SketchEntityKey;
use amg_doc::{
    Command, CreatedId, Document, EntityRef, Expr, ExprSrc, FeatureBox, FeatureId, SketchId,
};
use amg_features::sketch::{
    PlaneInput, ProjectionMode, SketchEdit, SketchFeature, project_reference,
};
use amg_render::{Camera, Projection};
use amg_sketch::{
    ConstraintKind, Dimension, DimensionKind, EntityId, EntityKind, EntityOrigin, HvTarget, Sketch,
};
use amg_ui::CommitBatch;
use amg_ui::sketch::view::{self, colors};
use amg_ui::sketch::{SKETCH_SELECT_CMD, SketchLayer, SketchSession};
use amg_ui::testing::AppHarness;

const PRIM: egui::PointerButton = egui::PointerButton::Primary;
const NONE: egui::Modifiers = egui::Modifiers::NONE;
const TOL: f64 = 1e-6;

// ---- helpers --------------------------------------------------------------------------------

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn inline(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn add(doc: &mut Document, f: FeatureBox) -> FeatureId {
    let root = doc.design().root();
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context: root,
            name: None,
        })
        .unwrap();
    out.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap()
}

fn jobs() -> &'static amg_jobs::JobSystem {
    static JOBS: std::sync::OnceLock<amg_jobs::JobSystem> = std::sync::OnceLock::new();
    JOBS.get_or_init(|| amg_jobs::JobSystem::new(amg_jobs::JobSystemConfig::default()).unwrap())
}

fn harness(doc: Document) -> AppHarness {
    let mut h = AppHarness::builder()
        .size(egui::vec2(1400.0, 900.0))
        .build();
    let id = h.app_mut().add_document("ui17".into(), doc);
    h.app_mut().activate(id);
    h.drive_until_idle();
    h
}

fn session(h: &AppHarness) -> &SketchSession {
    h.app()
        .doc_layer::<SketchLayer>()
        .expect("sketch layer")
        .session()
        .expect("an open sketch session")
}

fn applied(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

/// Opens sketch `id` (Edit Sketch) and sets `camera`.
fn edit_sketch(h: &mut AppHarness, id: FeatureId, camera: Camera) {
    h.app_mut().edit_feature(id);
    h.drive_until_idle();
    assert_eq!(session(h).feature(), id);
    assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));
    h.app_mut().active_document_mut().camera = camera;
    h.drive_until_idle();
}

/// The ends of line `l`, sorted by (x, y).
fn ends(s: &Sketch, l: EntityId) -> (Point2, Point2) {
    let EntityKind::Line { start, end } = s.entity(l).unwrap().kind else {
        panic!("{l:?} is not a line");
    };
    let (a, b) = (s.point(start).unwrap(), s.point(end).unwrap());
    if (a.x, a.y) <= (b.x, b.y) {
        (a, b)
    } else {
        (b, a)
    }
}

#[track_caller]
fn close(a: Point2, b: Point2) {
    assert!(a.distance(b) < TOL, "{a:?} vs {b:?}");
}

#[track_caller]
fn assert_line(s: &Sketch, l: EntityId, a: Point2, b: Point2) {
    let (u, v) = ends(s, l);
    close(u, a);
    close(v, b);
}

/// The projected lines of `s`.
fn projected_lines(s: &Sketch) -> Vec<EntityId> {
    s.entities()
        .filter(|(_, e)| {
            matches!(e.origin, EntityOrigin::Projected(_))
                && matches!(e.kind, EntityKind::Line { .. })
        })
        .map(|(id, _)| id)
        .collect()
}

/// The end points of line `l`.
fn line_points(s: &Sketch, l: EntityId) -> [EntityId; 2] {
    let EntityKind::Line { start, end } = s.entity(l).unwrap().kind else {
        panic!("{l:?} is not a line");
    };
    [start, end]
}

/// Drags from sketch point `from` to `to` on a sketch in plane `to_world` with the select
/// tool; returns the number of undo entries it made.
fn drag(h: &mut AppHarness, from: Point3, to: Point3) -> usize {
    assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));
    let before = applied(h);
    let (a, b) = (h.world_to_screen(from), h.world_to_screen(to));
    h.drag(PRIM, NONE, a, b, 6);
    h.drive_until_idle();
    applied(h) - before
}

// ---- Done-when 2 without a kernel: a projected line of an earlier sketch --------------------

/// A sketch on XY with one line from the fixed origin, horizontal, of length `len` (a user
/// parameter), and a second sketch on XY that projects it. Returns (doc, upstream, line,
/// sketch, length param).
fn sketch_line_doc() -> (Document, FeatureId, EntityId, FeatureId, amg_doc::ParamId) {
    let mut doc = Document::new("ui17-sketch");
    let o = doc
        .apply(Command::AddParam {
            name: "len".to_owned(),
            expr: "30 mm".to_owned(),
            unit: None,
            comment: String::new(),
        })
        .unwrap();
    let len = o
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Param(p) => Some(*p),
            _ => None,
        })
        .unwrap();
    let mut a = SketchFeature::new(PlaneInput::OriginXY);
    let line = a
        .apply(SketchEdit::AddLine {
            a: p(0.0, 0.0),
            b: p(30.0, 0.0),
        })
        .unwrap()
        .created[0];
    let [start, _] = line_points(&a.sketch, line);
    for (kind, expr) in [
        (ConstraintKind::Fix { entity: start }, None),
        (ConstraintKind::Horizontal(HvTarget::Line(line)), None),
        (
            ConstraintKind::Dimension(Dimension {
                kind: DimensionKind::Length { line },
                value: 30.0,
                driving: true,
                label_pos: p(15.0, -5.0),
            }),
            Some(inline("len")),
        ),
    ] {
        a.apply(SketchEdit::AddConstraint { kind, expr }).unwrap();
    }
    let upstream = add(&mut doc, FeatureBox::new(a));
    doc.recompute_blocking(jobs()).unwrap();
    let b = add(
        &mut doc,
        FeatureBox::new(SketchFeature::new(PlaneInput::OriginXY)),
    );
    doc.recompute_blocking(jobs()).unwrap();
    // Project the line into the second sketch, as the tool's job does.
    let source = EntityRef::SketchCurve {
        sketch: SketchId(upstream),
        curve: SketchEntityKey(u64::from(line.0)),
    };
    let plane = doc.evaluated().sketch(SketchId(b)).unwrap().plane;
    let geometry = project_reference(
        doc.design(),
        doc.evaluated(),
        b,
        &plane,
        &source,
        ProjectionMode::Project,
    )
    .unwrap();
    let (_, node) = doc.design().timeline().node(b).unwrap();
    let mut f = node
        .feature
        .downcast_ref::<SketchFeature>()
        .unwrap()
        .clone();
    f.apply(SketchEdit::AddProjection {
        source,
        mode: ProjectionMode::Project,
        geometry,
    })
    .unwrap();
    doc.apply(Command::EditFeature {
        id: b,
        feature: FeatureBox::new(f),
    })
    .unwrap();
    (doc, upstream, line, b, len)
}

fn top_down() -> Camera {
    Camera {
        eye: Point3::new(15.0, 5.0, 90.0),
        target: Point3::new(15.0, 5.0, 0.0),
        up: Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    }
}

#[test]
fn dw2_projected_sketch_line_is_locked_coloured_and_follows_its_source() {
    let (doc, _upstream, _line, sketch, len) = sketch_line_doc();
    let mut h = harness(doc);
    edit_sketch(&mut h, sketch, top_down());
    let s = session(&h);
    let lines = projected_lines(s.display());
    assert_eq!(lines.len(), 1);
    let pl = lines[0];
    assert_line(s.display(), pl, p(0.0, 0.0), p(30.0, 0.0));
    assert!(s.is_projected(pl));
    assert_eq!(view::color_of(s, pl), Some(colors::LOCKED));
    for e in line_points(s.display(), pl) {
        assert!(s.is_projected(e));
        assert_eq!(view::color_of(s, e), Some(colors::LOCKED));
    }

    // Done-when 2: neither the line nor its end point can be dragged.
    let commits = session(&h).stats().commits;
    assert_eq!(
        drag(
            &mut h,
            Point3::new(15.0, 0.0, 0.0),
            Point3::new(15.0, 8.0, 0.0)
        ),
        0,
        "dragging the projected line made no undo entry"
    );
    assert_eq!(
        drag(
            &mut h,
            Point3::new(30.0, 0.0, 0.0),
            Point3::new(36.0, 6.0, 0.0)
        ),
        0,
        "dragging its end made no undo entry"
    );
    let s = session(&h);
    assert_eq!(s.stats().commits, commits);
    assert_eq!(s.stats().drag_frames, 0, "no drag started");
    assert!(!s.is_dragging());
    assert!(
        s.notice.as_deref().is_some_and(|n| n.contains("locked")),
        "{:?}",
        s.notice
    );
    assert_line(s.display(), pl, p(0.0, 0.0), p(30.0, 0.0));

    // An upstream edit while the sketch is open: the session takes the refreshed line.
    let refreshes = s.stats().projection_refreshes;
    h.app_mut().submit(CommitBatch {
        label: "Change Parameters".into(),
        commands: vec![Command::SetParamExpr {
            id: len,
            expr: "42 mm".into(),
        }],
    });
    h.drive_until_idle();
    let s = session(&h);
    assert_line(s.display(), pl, p(0.0, 0.0), p(42.0, 0.0));
    assert_line(&s.working().sketch, pl, p(0.0, 0.0), p(42.0, 0.0));
    assert!(s.stats().projection_refreshes > refreshes);
    assert_eq!(s.stats().commits, commits, "the refresh is no undo entry");
}

// ---- Done-when 1: an extruded box (needs the OCCT kernel) -----------------------------------

#[cfg(feature = "occt")]
mod occt {
    use super::*;
    use amg_doc::stub::ProfileKey;
    use amg_features::solid::{BodyOp, Extent, Extrude, OpTarget};
    use amg_sketch::build;
    use amg_ui::sketch::{
        CreateKind, DIMENSION_DEF, DimensionEditDialog, FINISH_SKETCH_CMD, ProjectKind,
        ProjectOutcome, ProjectTool,
    };
    use amg_ui::testing::egui_kittest::kittest::Queryable as _;

    /// A 40 × 20 rectangle on XY extruded by 30 mm, and an empty sketch on XZ (the plane of
    /// the box's front face). Returns (doc, extrude, sketch).
    fn box_doc() -> (Document, FeatureId, FeatureId) {
        amg_features::solid::link();
        let mut doc = Document::new("ui17-box");
        let mut rect = SketchFeature::new(PlaneInput::OriginXY);
        rect.edit_sketch(|s| build::rectangle_2pt(s, p(0.0, 0.0), p(40.0, 20.0)))
            .unwrap();
        let rect = add(&mut doc, FeatureBox::new(rect));
        doc.recompute_blocking(jobs()).unwrap();
        let g = doc.evaluated().sketch(SketchId(rect)).unwrap();
        let pr = &g.profiles.profiles[0];
        let profile = EntityRef::Profile {
            sketch: SketchId(rect),
            profile: ProfileKey(pr.id.0),
            fingerprint: Some(amg_doc::naming::ProfileFp(pr.fingerprint.clone())),
        };
        let e = Extrude::distance(
            vec![profile],
            "30 mm",
            OpTarget {
                op: BodyOp::NewBody,
                participants: Vec::new(),
            },
        );
        let extrude = add(&mut doc, FeatureBox::new(e));
        doc.recompute_blocking(jobs()).unwrap();
        assert_eq!(
            doc.evaluated().node(extrude).unwrap().health,
            amg_doc::Health::Ok
        );
        let sketch = add(
            &mut doc,
            FeatureBox::new(SketchFeature::new(PlaneInput::OriginXZ)),
        );
        doc.recompute_blocking(jobs()).unwrap();
        (doc, extrude, sketch)
    }

    /// Looks at the front face (y = 0) from −Y.
    fn front() -> Camera {
        Camera {
            eye: Point3::new(20.0, -110.0, 18.0),
            target: Point3::new(20.0, 0.0, 18.0),
            up: Dir3::Z,
            projection: Projection::Perspective { fov_y: 0.8 },
        }
    }

    /// Sketch point (x, z) of the XZ sketch in world coordinates.
    fn xz(x: f64, z: f64) -> Point3 {
        Point3::new(x, 0.0, z)
    }

    fn click(h: &mut AppHarness, at: Point3, mods: egui::Modifiers) {
        let pos = h.world_to_screen(at);
        h.click_at(pos, PRIM, mods);
        h.drive_until_idle();
    }

    /// Steps until no solver work is left (the dimension editor's focused field keeps asking
    /// for frames, so `drive_until_idle` cannot be used while it is open).
    fn settle(h: &mut AppHarness) {
        let start = std::time::Instant::now();
        loop {
            h.kittest().step();
            if !session(h).is_solving()
                && h.app().jobs().jobs().is_empty()
                && h.document().is_evaluated()
            {
                for _ in 0..3 {
                    h.kittest().step();
                }
                return;
            }
            assert!(start.elapsed().as_secs() < 30, "settle: still solving");
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }

    /// Sketch Dimension on the picks `on`, label at `label`; types `value` into the editor.
    fn dimension(h: &mut AppHarness, on: &[Point3], label: Point3, value: &str) {
        if h.app().active_tool() != Some(DIMENSION_DEF.id) {
            h.run_command(DIMENSION_DEF.id);
            h.drive_until_idle();
        }
        for at in on {
            click(h, *at, NONE);
        }
        let pos = h.world_to_screen(label);
        h.click_at(pos, PRIM, NONE);
        settle(h);
        assert!(
            h.app().dialog::<DimensionEditDialog>().is_some(),
            "the editor opens on a new driving dimension: notice {:?}, offer {:?}, dialogs {:?},              dims {:?}, selected {:?}",
            session(h).notice,
            session(h).driven_offer,
            h.app().dialog_titles(),
            session(h)
                .working()
                .sketch
                .constraints()
                .map(|(_, c)| c.kind.clone())
                .collect::<Vec<_>>(),
            session(h).selected,
        );
        h.kittest()
            .key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
        h.kittest().step();
        h.kittest()
            .get_by_label(amg_ui::sketch::dimension::EDIT_FIELD_NAME)
            .type_text(value);
        h.kittest().step();
        h.key(egui::Key::Enter);
        h.drive_until_idle();
        assert!(h.app().dialog::<DimensionEditDialog>().is_none());
    }

    fn to_select(h: &mut AppHarness) {
        for _ in 0..3 {
            if h.app().active_tool() == Some(SKETCH_SELECT_CMD) {
                return;
            }
            h.key(egui::Key::Escape);
            h.drive_until_idle();
        }
        assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));
    }

    /// The projected edges sorted bottom, right, top, left (by the midpoint).
    fn edges(s: &Sketch) -> [EntityId; 4] {
        let mut lines = projected_lines(s);
        assert_eq!(lines.len(), 4, "the four edges of the face");
        let mid = |l: EntityId| {
            let (a, b) = ends(s, l);
            p((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)
        };
        lines.sort_by(|a, b| mid(*a).y.total_cmp(&mid(*b).y));
        let (bottom, top) = (lines[0], lines[3]);
        let (mut l, mut r) = (lines[1], lines[2]);
        if mid(l).x > mid(r).x {
            std::mem::swap(&mut l, &mut r);
        }
        [bottom, r, top, l]
    }

    /// The circle of `s` (the only user curve) and its centre and radius.
    fn circle(s: &Sketch) -> (EntityId, Point2, f64) {
        let (id, e) = s
            .entities()
            .find(|(_, e)| matches!(e.kind, EntityKind::Circle { .. }))
            .expect("the circle");
        let EntityKind::Circle { center, radius } = e.kind else {
            unreachable!()
        };
        (id, s.point(center).unwrap(), s.params().get(radius))
    }

    /// The box face edges for width `w` and height `t`, and the circle 10 mm left of the right
    /// edge and 8 mm below the top edge (radius 3).
    #[track_caller]
    fn expect(s: &Sketch, w: f64, t: f64) {
        let [bottom, right, top, left] = edges(s);
        assert_line(s, bottom, p(0.0, 0.0), p(w, 0.0));
        assert_line(s, right, p(w, 0.0), p(w, t));
        assert_line(s, top, p(0.0, t), p(w, t));
        assert_line(s, left, p(0.0, 0.0), p(0.0, t));
        let (_, c, r) = circle(s);
        assert!(c.distance(p(w - 10.0, t - 8.0)) < 1e-5, "centre {c:?}");
        assert!((r - 3.0).abs() < 1e-5, "radius {r}");
    }

    fn set_height(h: &mut AppHarness, extrude: FeatureId, height: &str) {
        let (_, node) = h.document().design().timeline().node(extrude).unwrap();
        let mut e = node.feature.downcast_ref::<Extrude>().unwrap().clone();
        e.side1.extent = Extent::Distance(inline(height));
        h.app_mut().submit(CommitBatch {
            label: "Edit Extrude".into(),
            commands: vec![Command::EditFeature {
                id: extrude,
                feature: FeatureBox::new(e),
            }],
        });
        h.drive_until_idle();
    }

    fn evaluated(h: &AppHarness, sketch: FeatureId) -> Sketch {
        (*h.document()
            .evaluated()
            .sketch(SketchId(sketch))
            .unwrap()
            .sketch)
            .clone()
    }

    fn retype(h: &mut AppHarness, label: &str, text: &str) {
        h.kittest().get_by_label(label).focus();
        h.kittest().step();
        h.kittest()
            .key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
        h.kittest().step();
        h.kittest().get_by_label(label).type_text(text);
        h.kittest().step();
        let ctx = h.kittest().ctx.clone();
        ctx.memory_mut(|m| {
            if let Some(id) = m.focused() {
                m.surrender_focus(id);
            }
        });
        h.kittest().step();
    }

    #[test]
    fn dw1_projected_box_face_edges_and_dimensioned_geometry_follow_the_extrude() {
        let (doc, extrude, sketch) = box_doc();
        let mut h = harness(doc);
        edit_sketch(&mut h, sketch, front());

        // Project: click the middle of the front face.
        let before = applied(&h);
        h.run_command(ProjectKind::Project.def().id);
        h.drive_until_idle();
        assert_eq!(h.app().active_tool(), Some(ProjectKind::Project.def().id));
        click(&mut h, xz(20.0, 15.0), NONE);
        let tool = h.app_mut().active_tool_as::<ProjectTool>().unwrap();
        let Some(ProjectOutcome::Added(created)) = tool.last().cloned() else {
            panic!("projected: {:?}", tool.last());
        };
        assert_eq!(created.len(), 4, "one line per face edge");
        assert_eq!(applied(&h) - before, 1, "one undo entry");
        assert_eq!(h.document().undo_label(), Some("Project"));
        // Projecting the same face again is refused.
        click(&mut h, xz(20.0, 15.0), NONE);
        let tool = h.app_mut().active_tool_as::<ProjectTool>().unwrap();
        assert!(
            matches!(tool.last(), Some(ProjectOutcome::Refused(m)) if m.contains("already")),
            "{:?}",
            tool.last()
        );
        assert_eq!(applied(&h) - before, 1);
        h.key(egui::Key::Escape);
        h.drive_until_idle();
        to_select(&mut h);
        {
            let s = session(&h);
            let f = s.working();
            assert_eq!(f.links.len(), 1);
            let link = f.links.values().next().unwrap();
            assert_eq!(link.mode, ProjectionMode::Project);
            assert!(
                matches!(&link.source, EntityRef::Topo(t) if t.kind == amg_doc::TopoKind::Face)
            );
            let [bottom, right, top, left] = edges(s.display());
            for e in [bottom, right, top, left] {
                assert_eq!(view::color_of(s, e), Some(colors::LOCKED));
            }
            assert_line(s.display(), top, p(0.0, 30.0), p(40.0, 30.0));
            assert_line(s.display(), right, p(40.0, 0.0), p(40.0, 30.0));
        }

        // A circle (Ctrl: no inference), dimensioned to the projected top and right edges.
        h.run_command(CreateKind::CircleCenter.def().id);
        h.drive_until_idle();
        click(&mut h, xz(29.0, 21.0), egui::Modifiers::CTRL);
        click(&mut h, xz(32.0, 21.0), egui::Modifiers::CTRL);
        to_select(&mut h);
        let centre = {
            let (_, c, _) = circle(session(&h).display());
            xz(c.x, c.y)
        };
        // 8 mm below the top edge, 10 mm left of the right edge, diameter 6 mm.
        dimension(&mut h, &[centre, xz(20.0, 30.0)], xz(12.0, 26.0), "8");
        let (_, c, _) = circle(session(&h).display());
        dimension(
            &mut h,
            &[xz(c.x, c.y), xz(40.0, 10.0)],
            xz(36.0, 14.0),
            "10",
        );
        let (_, c, _) = circle(session(&h).display());
        dimension(&mut h, &[xz(c.x + 3.0, c.y)], xz(c.x + 6.0, c.y + 4.0), "6");
        to_select(&mut h);
        let dims = session(&h)
            .working()
            .sketch
            .constraints()
            .filter(|(_, c)| c.kind.as_dimension().is_some())
            .count();
        assert_eq!(dims, 3);
        h.drive_until_idle();
        {
            let s = session(&h);
            assert_eq!(
                s.dof().map(|d| d.dof),
                Some(0),
                "the circle is fully constrained"
            );
            expect(s.display(), 40.0, 30.0);
        }
        expect(&evaluated(&h, sketch), 40.0, 30.0);

        // Edit the extrude while the sketch is open: 30 → 45 mm.
        let commits = session(&h).stats().commits;
        set_height(&mut h, extrude, "45 mm");
        {
            let s = session(&h);
            expect(s.display(), 40.0, 45.0);
            expect(&s.working().sketch, 40.0, 45.0);
            assert!(s.stats().projection_refreshes >= 1);
            assert_eq!(s.stats().commits, commits);
        }
        expect(&evaluated(&h, sketch), 40.0, 45.0);

        // Done-when 2 on the box: the projected top edge and a corner do not drag.
        let top = edges(session(&h).display())[2];
        assert_eq!(drag(&mut h, xz(20.0, 45.0), xz(20.0, 52.0)), 0);
        assert_eq!(drag(&mut h, xz(40.0, 45.0), xz(46.0, 50.0)), 0);
        assert_line(session(&h).display(), top, p(0.0, 45.0), p(40.0, 45.0));

        // Finish, then edit the extrude through its dialog: 45 → 12 mm.
        h.run_command(FINISH_SKETCH_CMD);
        h.drive_until_idle();
        assert!(
            h.app()
                .doc_layer::<SketchLayer>()
                .unwrap()
                .session()
                .is_none()
        );
        h.app_mut().edit_feature(extrude);
        h.drive_until_idle();
        retype(&mut h, "Distance", "12 mm");
        h.drive_until_idle();
        h.kittest().get_by_label("OK").click();
        h.drive_until_idle();
        assert_eq!(h.app().active_tool(), None);
        expect(&evaluated(&h, sketch), 40.0, 12.0);
        // Opening the sketch again starts from the refreshed projections.
        edit_sketch(&mut h, sketch, front());
        let s = session(&h);
        expect(s.display(), 40.0, 12.0);
        expect(&s.working().sketch, 40.0, 12.0);
        assert_eq!(s.dof().map(|d| d.dof), Some(0));
    }

    #[test]
    fn include_takes_an_edge_in_the_sketch_plane() {
        let (doc, _extrude, sketch) = box_doc();
        let mut h = harness(doc);
        edit_sketch(&mut h, sketch, front());
        // Include an edge in the plane: the front face's bottom edge.
        h.run_command(ProjectKind::Include.def().id);
        h.drive_until_idle();
        click(&mut h, xz(20.0, 0.1), NONE);
        let tool = h.app_mut().active_tool_as::<ProjectTool>().unwrap();
        assert!(
            matches!(tool.last(), Some(ProjectOutcome::Added(v)) if v.len() == 1),
            "{:?}",
            tool.last()
        );
        h.key(egui::Key::Escape);
        h.drive_until_idle();
        to_select(&mut h);
        let s = session(&h);
        let lines = projected_lines(s.display());
        assert_eq!(lines.len(), 1);
        assert_line(s.display(), lines[0], p(0.0, 0.0), p(40.0, 0.0));
        assert_eq!(view::color_of(s, lines[0]), Some(colors::LOCKED));
    }
}
