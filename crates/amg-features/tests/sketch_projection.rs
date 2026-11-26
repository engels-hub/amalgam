//! SK-09: projected geometry in sketches. Done-when 1 is the three golden models of the `occt`
//! module (`tests/golden/sketch/sk09_*.json`; `AMG_BLESS=1` rewrites them), Done-when 2 the
//! `dw2_*` tests. Sources that are lines or points of an earlier sketch need no geometry
//! kernel, so those tests run everywhere.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

mod solid_common;

use amg_core::{Point2, Transform, Vec3};
use amg_doc::stub::{SketchEntityKey, SketchGeom};
use amg_doc::{
    Command, CreatedId, Document, EntityRef, Expr, ExprSrc, FeatureBox, FeatureId, Health, ParamId,
    SaveOptions, SketchId,
};
use amg_features::sketch::{
    PlaneInput, ProjectionError, ProjectionMode, SketchEdit, SketchFeature, project_reference,
};
use amg_sketch::{
    ConstraintKind, Dimension, DimensionKind, EntityId, EntityKind, EntityOrigin, ExternalLinkId,
    HvTarget, ProjectedGeom, SketchError,
};
use solid_common::*;

const TOL: f64 = 1e-6;

fn inline(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

#[track_caller]
fn close(a: Point2, b: Point2) {
    assert!(a.distance(b) < TOL, "{a:?} vs {b:?}");
}

fn add_param(doc: &mut Document, name: &str, expr: &str) -> ParamId {
    let o = doc
        .apply(Command::AddParam {
            name: name.to_owned(),
            expr: expr.to_owned(),
            unit: None,
            comment: String::new(),
        })
        .unwrap();
    o.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Param(p) => Some(*p),
            _ => None,
        })
        .unwrap()
}

fn set_param(doc: &mut Document, id: ParamId, expr: &str) {
    doc.apply(Command::SetParamExpr {
        id,
        expr: expr.to_owned(),
    })
    .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
}

fn current(doc: &Document, id: FeatureId) -> SketchFeature {
    let (_, node) = doc.design().timeline().node(id).unwrap();
    node.feature
        .downcast_ref::<SketchFeature>()
        .unwrap()
        .clone()
}

fn geom(doc: &Document, id: FeatureId) -> SketchGeom {
    (**doc.evaluated().sketch(SketchId(id)).unwrap()).clone()
}

/// Applies `e` to sketch `id`, commits it and recomputes. Returns the created entities.
fn apply(doc: &mut Document, id: FeatureId, e: SketchEdit) -> Vec<EntityId> {
    let mut f = current(doc, id);
    let r = f.apply(e).unwrap();
    edit(doc, id, FeatureBox::new(f));
    r.created
}

fn constrain(doc: &mut Document, id: FeatureId, kind: ConstraintKind, expr: Option<&str>) {
    apply(
        doc,
        id,
        SketchEdit::AddConstraint {
            kind,
            expr: expr.map(inline),
        },
    );
}

fn dim(kind: DimensionKind, value: f64) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving: true,
        label_pos: Point2::ZERO,
    })
}

/// Adds the projection of `source` to sketch `id`; returns the projected entities.
fn project(
    doc: &mut Document,
    id: FeatureId,
    source: EntityRef,
    mode: ProjectionMode,
) -> Vec<EntityId> {
    let geometry = preview(doc, id, &source, mode).unwrap();
    apply(
        doc,
        id,
        SketchEdit::AddProjection {
            source,
            mode,
            geometry,
        },
    )
}

fn preview(
    doc: &Document,
    id: FeatureId,
    source: &EntityRef,
    mode: ProjectionMode,
) -> Result<Vec<ProjectedGeom>, ProjectionError> {
    let plane = geom(doc, id).plane;
    project_reference(doc.design(), doc.evaluated(), id, &plane, source, mode)
}

/// The end points of line `l` in the solved sketch.
fn ends(g: &SketchGeom, l: EntityId) -> (Point2, Point2) {
    let EntityKind::Line { start, end } = g.sketch.entity(l).unwrap().kind else {
        panic!("{l:?} is not a line");
    };
    (g.sketch.point(start).unwrap(), g.sketch.point(end).unwrap())
}

fn point_ids(g: &SketchGeom, l: EntityId) -> (EntityId, EntityId) {
    let EntityKind::Line { start, end } = g.sketch.entity(l).unwrap().kind else {
        panic!("{l:?} is not a line");
    };
    (start, end)
}

#[cfg(feature = "occt")]
fn circle(g: &SketchGeom, c: EntityId) -> (Point2, f64) {
    let EntityKind::Circle { center, radius } = g.sketch.entity(c).unwrap().kind else {
        panic!("{c:?} is not a circle");
    };
    (
        g.sketch.point(center).unwrap(),
        g.sketch.params().get(radius),
    )
}

/// The ends of a line sorted by (x, y), so that a test does not depend on its direction.
fn sorted_ends(g: &SketchGeom, l: EntityId) -> (Point2, Point2) {
    let (a, b) = ends(g, l);
    if (a.x, a.y) <= (b.x, b.y) {
        (a, b)
    } else {
        (b, a)
    }
}

#[track_caller]
fn assert_line(g: &SketchGeom, l: EntityId, a: Point2, b: Point2) {
    let (u, v) = sorted_ends(g, l);
    close(u, a);
    close(v, b);
}

#[track_caller]
fn assert_ok(doc: &Document, id: FeatureId) {
    assert_eq!(health(doc, id), Health::Ok, "{id}");
}

#[track_caller]
fn warnings(doc: &Document, id: FeatureId) -> Vec<amg_doc::Diagnostic> {
    match health(doc, id) {
        Health::Warning(w) => w,
        other => panic!("{id}: expected a warning, got {other:?}"),
    }
}

fn is_locked(g: &SketchGeom, e: EntityId) -> bool {
    let entity = g.sketch.entity(e).unwrap();
    entity
        .kind
        .own_params()
        .into_iter()
        .chain(
            entity
                .kind
                .children()
                .into_iter()
                .flat_map(|c| g.sketch.entity(c).unwrap().kind.own_params()),
        )
        .all(|p| g.sketch.params().flags(p).locked)
}

// ---- sketch sources (no geometry kernel) ------------------------------------------------------

/// An upstream sketch on XY with one line from (0, 0) to (`w`, 10) and a sketch on XY that
/// projects it, with a user line hanging 5 mm below the projected line's far end.
struct LineModel {
    doc: Document,
    w: ParamId,
    upstream: FeatureId,
    source_line: EntityId,
    sketch: FeatureId,
    projected: EntityId,
    hanging: EntityId,
}

fn line_model() -> LineModel {
    let mut doc = Document::new("sk09-lines");
    let w = add_param(&mut doc, "w", "30 mm");
    let mut u = SketchFeature::new(PlaneInput::OriginXY);
    let source_line = u
        .edit_sketch(|s| {
            let line = s.add_line(p(0.0, 0.0), p(30.0, 10.0))?;
            let EntityKind::Line { start, end } = s.entity(line).unwrap().kind else {
                unreachable!()
            };
            s.add_constraint(ConstraintKind::Fix { entity: start })?;
            s.add_constraint(dim(
                DimensionKind::HorizontalDistance {
                    a: start,
                    b: end,
                    side: amg_sketch::Side::Positive,
                },
                30.0,
            ))?;
            s.add_constraint(dim(
                DimensionKind::VerticalDistance {
                    a: start,
                    b: end,
                    side: amg_sketch::Side::Positive,
                },
                10.0,
            ))?;
            Ok(line)
        })
        .unwrap();
    let width = *u.dims.keys().next().unwrap();
    u.apply(SketchEdit::SetDimension {
        id: width,
        expr: inline("w"),
    })
    .unwrap();
    let upstream = add(&mut doc, FeatureBox::new(u));
    assert_ok(&doc, upstream);

    let sketch = add(
        &mut doc,
        FeatureBox::new(SketchFeature::new(PlaneInput::OriginXY)),
    );
    let source = EntityRef::SketchCurve {
        sketch: SketchId(upstream),
        curve: SketchEntityKey(u64::from(source_line.0)),
    };
    let projected = project(&mut doc, sketch, source, ProjectionMode::Project)[0];
    let created = apply(
        &mut doc,
        sketch,
        SketchEdit::AddLine {
            a: p(31.0, 9.0),
            b: p(31.0, 4.0),
        },
    );
    let hanging = created[0];
    let g = geom(&doc, sketch);
    let far = point_ids(&g, projected).1;
    let top = point_ids(&g, hanging).0;
    constrain(
        &mut doc,
        sketch,
        ConstraintKind::Coincident { a: top, b: far },
        None,
    );
    constrain(
        &mut doc,
        sketch,
        ConstraintKind::Vertical(HvTarget::Line(hanging)),
        None,
    );
    constrain(
        &mut doc,
        sketch,
        dim(DimensionKind::Length { line: hanging }, 5.0),
        Some("5 mm"),
    );
    assert_ok(&doc, sketch);
    LineModel {
        doc,
        w,
        upstream,
        source_line,
        sketch,
        projected,
        hanging,
    }
}

#[track_caller]
fn expect_line_model(g: &SketchGeom, m_projected: EntityId, hanging: EntityId, w: f64) {
    let (a, b) = ends(g, m_projected);
    close(a, p(0.0, 0.0));
    close(b, p(w, 10.0));
    let (c, d) = ends(g, hanging);
    close(c, p(w, 10.0));
    close(d, p(w, 5.0));
}

#[test]
fn editing_the_source_sketch_moves_the_projected_line_and_what_hangs_on_it() {
    let mut m = line_model();
    let g = geom(&m.doc, m.sketch);
    assert!(is_locked(&g, m.projected));
    assert!(matches!(
        g.sketch.entity(m.projected).unwrap().origin,
        EntityOrigin::Projected(ExternalLinkId(0))
    ));
    expect_line_model(&g, m.projected, m.hanging, 30.0);
    set_param(&mut m.doc, m.w, "48 mm");
    assert_ok(&m.doc, m.sketch);
    expect_line_model(&geom(&m.doc, m.sketch), m.projected, m.hanging, 48.0);
    // The stored geometry is untouched by recompute: the refresh is part of the output.
    let stored = current(&m.doc, m.sketch);
    let EntityKind::Line { end, .. } = stored.sketch.entity(m.projected).unwrap().kind else {
        panic!()
    };
    close(stored.sketch.point(end).unwrap(), p(30.0, 10.0));
}

#[test]
fn dw2_a_deleted_source_curve_warns_and_keeps_the_last_geometry() {
    let mut m = line_model();
    set_param(&mut m.doc, m.w, "48 mm");
    // What the application does after a recompute: keep the refreshed geometry.
    let commit = current(&m.doc, m.sketch).commit_solved(&geom(&m.doc, m.sketch));
    apply(&mut m.doc, m.sketch, commit);

    let mut u = current(&m.doc, m.upstream);
    u.apply(SketchEdit::RemoveEntities(vec![m.source_line]))
        .unwrap();
    edit(&mut m.doc, m.upstream, FeatureBox::new(u));

    let w = warnings(&m.doc, m.sketch);
    assert_eq!(w.len(), 1, "{w:?}");
    assert!(w[0].message.contains("projection 0"), "{}", w[0].message);
    assert!(w[0].message.contains("not available"), "{}", w[0].message);
    assert!(matches!(
        w[0].entities[0],
        EntityRef::SketchCurve { sketch, .. } if sketch == SketchId(m.upstream)
    ));
    assert!(w[0].entities.contains(&EntityRef::SketchCurve {
        sketch: SketchId(m.sketch),
        curve: SketchEntityKey(u64::from(m.projected.0)),
    }));
    let g = geom(&m.doc, m.sketch);
    assert!(is_locked(&g, m.projected));
    expect_line_model(&g, m.projected, m.hanging, 48.0);
}

#[test]
fn dw2_a_suppressed_or_deleted_source_feature_warns_and_the_sketch_stays_editable() {
    let mut m = line_model();
    let before = geom(&m.doc, m.sketch);
    m.doc
        .apply(Command::SetSuppressed {
            ids: vec![m.upstream],
            suppressed: true,
        })
        .unwrap();
    m.doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(warnings(&m.doc, m.sketch).len(), 1);
    expect_line_model(&geom(&m.doc, m.sketch), m.projected, m.hanging, 30.0);
    assert_eq!(
        geom(&m.doc, m.sketch).sketch.params().as_slice(),
        before.sketch.params().as_slice()
    );

    m.doc
        .apply(Command::SetSuppressed {
            ids: vec![m.upstream],
            suppressed: false,
        })
        .unwrap();
    m.doc.recompute_blocking(jobs()).unwrap();
    assert_ok(&m.doc, m.sketch);

    m.doc
        .apply(Command::DeleteFeatures {
            ids: vec![m.upstream],
        })
        .unwrap();
    m.doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(warnings(&m.doc, m.sketch).len(), 1);
    expect_line_model(&geom(&m.doc, m.sketch), m.projected, m.hanging, 30.0);

    // The sketch can still be edited, saved and loaded with the dangling source ...
    apply(
        &mut m.doc,
        m.sketch,
        SketchEdit::AddPoint { at: p(1.0, 1.0) },
    );
    assert_eq!(warnings(&m.doc, m.sketch).len(), 1);
    let dir = tempfile_dir("dangling");
    let path = dir.join("p.amg");
    let loaded = save_and_load(&m.doc, &path);
    assert_eq!(warnings(&loaded, m.sketch).len(), 1);
    expect_line_model(&geom(&loaded, m.sketch), m.projected, m.hanging, 30.0);
    std::fs::remove_dir_all(dir).unwrap();

    // ... and removing the projection heals it.
    apply(
        &mut m.doc,
        m.sketch,
        SketchEdit::RemoveProjection(ExternalLinkId(0)),
    );
    assert_ok(&m.doc, m.sketch);
    let f = current(&m.doc, m.sketch);
    assert!(f.links.is_empty());
    assert!(f.sketch.entity(m.projected).is_none());
}

/// Saves `doc` as `.amg`, loads it again and recomputes.
fn save_and_load(doc: &Document, path: &std::path::Path) -> Document {
    amg_doc::save(path, &doc.snapshot(), &SaveOptions::default()).unwrap();
    let loaded = amg_doc::load(path).unwrap();
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    let mut doc = Document::from_design(loaded.design);
    doc.recompute_blocking(jobs()).unwrap();
    doc
}

fn tempfile_dir(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("amg-sk09-{tag}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn a_source_that_changes_shape_warns_and_keeps_the_geometry() {
    let mut m = line_model();
    // The source line now seen end on: its projection is a point, not a line.
    let mut u = current(&m.doc, m.upstream);
    u.apply(SketchEdit::SetPlane(PlaneInput::OriginXZ)).unwrap();
    edit(&mut m.doc, m.upstream, FeatureBox::new(u));
    // On XZ the line projects onto XY as (0,0)-(30,0): still a line, so it follows.
    assert_ok(&m.doc, m.sketch);
    assert_line(
        &geom(&m.doc, m.sketch),
        m.projected,
        p(0.0, 0.0),
        p(30.0, 0.0),
    );

    // A point link whose source becomes unrelated geometry: two pieces instead of one.
    let mut f = current(&m.doc, m.sketch);
    let link = f.links.get_mut(&ExternalLinkId(0)).unwrap();
    link.entities.push(EntityId(999));
    edit(&mut m.doc, m.sketch, FeatureBox::new(f));
    let w = warnings(&m.doc, m.sketch);
    assert!(w[0].message.contains("changed shape"), "{}", w[0].message);
    expect_line_model(&geom(&m.doc, m.sketch), m.projected, m.hanging, 30.0);
}

#[test]
fn point_sources_modes_and_construction_of_links() {
    let mut m = line_model();
    let g = geom(&m.doc, m.upstream);
    let (_, far) = point_ids(&g, m.source_line);
    let point = EntityRef::SketchPoint {
        sketch: SketchId(m.upstream),
        point: SketchEntityKey(u64::from(far.0)),
    };
    // A second sketch on XZ.
    let side = add(
        &mut m.doc,
        FeatureBox::new(SketchFeature::new(PlaneInput::OriginXZ)),
    );
    assert_eq!(
        preview(&m.doc, side, &point, ProjectionMode::Project).unwrap(),
        vec![ProjectedGeom::Point(p(30.0, 0.0))]
    );
    // (30, 10, 0) is 10 mm off XZ.
    assert!(matches!(
        preview(&m.doc, side, &point, ProjectionMode::Include),
        Err(ProjectionError::NotInPlane(d)) if (d - 10.0).abs() < TOL
    ));
    assert!(matches!(
        preview(&m.doc, side, &point, ProjectionMode::IntersectionCurve),
        Err(ProjectionError::NotApplicable { .. })
    ));
    // The line crosses XZ only at its start.
    let line = EntityRef::SketchCurve {
        sketch: SketchId(m.upstream),
        curve: SketchEntityKey(u64::from(m.source_line.0)),
    };
    assert_eq!(
        preview(&m.doc, side, &line, ProjectionMode::Intersect).unwrap(),
        vec![ProjectedGeom::Point(p(0.0, 0.0))]
    );
    let dot = project(&mut m.doc, side, point.clone(), ProjectionMode::Project)[0];
    set_param(&mut m.doc, m.w, "12 mm");
    assert_ok(&m.doc, side);
    close(geom(&m.doc, side).sketch.point(dot).unwrap(), p(12.0, 0.0));

    // Link ids are allocated past the ones in use, and a batch keeps entities apart.
    let geometry = preview(&m.doc, side, &point, ProjectionMode::Project).unwrap();
    let add_again = SketchEdit::AddProjection {
        source: point,
        mode: ProjectionMode::Project,
        geometry,
    };
    apply(
        &mut m.doc,
        side,
        SketchEdit::Batch(vec![add_again.clone(), add_again]),
    );
    let f = current(&m.doc, side);
    let links: Vec<_> = f
        .links
        .iter()
        .map(|(id, l)| (id.0, l.entities.len()))
        .collect();
    assert_eq!(links, [(0, 1), (1, 1), (2, 1)]);
    f.sketch.entities().for_each(|(_, e)| {
        assert!(matches!(e.origin, EntityOrigin::Projected(_)));
    });
}

#[test]
fn projected_geometry_is_locked_against_edits() {
    let m = line_model();
    let mut f = current(&m.doc, m.sketch);
    let before = f.clone();
    assert!(matches!(
        f.edit_sketch(|s| amg_sketch::edit::break_at(s, m.projected, p(15.0, 5.0))),
        Err(SketchError::Locked(l)) if l == m.projected
    ));
    assert!(matches!(
        f.edit_sketch(|s| amg_sketch::transform::transform(
            s,
            &[m.projected],
            amg_sketch::transform::translation(p(1.0, 0.0)),
            false
        )),
        Err(SketchError::Locked(_))
    ));
    assert_eq!(
        f.sketch.params().as_slice(),
        before.sketch.params().as_slice()
    );
    // An empty projection is refused, and nothing changes.
    assert!(
        f.apply(SketchEdit::AddProjection {
            source: before.links[&ExternalLinkId(0)].source.clone(),
            mode: ProjectionMode::Project,
            geometry: Vec::new(),
        })
        .is_err()
    );
    assert_eq!(f.links, before.links);
}

#[test]
fn removing_projected_geometry_cleans_up_its_points_and_link() {
    let m = line_model();
    let mut f = current(&m.doc, m.sketch);
    let EntityKind::Line { start, end } = f.sketch.entity(m.projected).unwrap().kind else {
        panic!()
    };
    let r = f
        .apply(SketchEdit::RemoveEntities(vec![m.projected]))
        .unwrap();
    assert!(r.removed.contains(&m.projected));
    // The unused start point goes; the end point carries the coincident constraint, so it
    // stays as ordinary (unlocked) geometry.
    assert!(f.sketch.entity(start).is_none() && r.removed.contains(&start));
    let kept = f.sketch.entity(end).unwrap();
    assert_eq!(kept.origin, EntityOrigin::User);
    assert!(
        kept.kind
            .own_params()
            .into_iter()
            .all(|p| !f.sketch.params().flags(p).locked)
    );
    assert!(f.links.is_empty(), "the last entity takes the link with it");

    // Removing the whole link does the same.
    let mut g = current(&m.doc, m.sketch);
    g.apply(SketchEdit::RemoveProjection(ExternalLinkId(0)))
        .unwrap();
    assert!(g.links.is_empty());
    assert!(g.sketch.entity(m.projected).is_none() && g.sketch.entity(start).is_none());
    assert!(
        g.apply(SketchEdit::RemoveProjection(ExternalLinkId(0)))
            .is_err()
    );
}

#[test]
fn projections_undo_and_round_trip_through_the_amg_format() {
    let mut m = line_model();
    let f = current(&m.doc, m.sketch);
    let json = serde_json::to_value(&f).unwrap();
    let back: SketchFeature = serde_json::from_value(json.clone()).unwrap();
    assert_eq!(serde_json::to_value(&back).unwrap(), json);
    assert_eq!(back.links, f.links);

    let dir = tempfile_dir("roundtrip");
    let path = dir.join("p.amg");
    let loaded = save_and_load(&m.doc, &path);
    assert_ok(&loaded, m.sketch);
    expect_line_model(&geom(&loaded, m.sketch), m.projected, m.hanging, 30.0);
    std::fs::remove_dir_all(dir).unwrap();

    // Undo back past the projection: no link, no projected entity.
    while !current(&m.doc, m.sketch).links.is_empty() {
        assert!(m.doc.undo());
    }
    assert_eq!(current(&m.doc, m.sketch).sketch.entities().count(), 0);
}

#[test]
fn a_link_listing_foreign_geometry_is_refused() {
    let m = line_model();
    let mut f = current(&m.doc, m.sketch);
    let hanging = m.hanging;
    f.links
        .get_mut(&ExternalLinkId(0))
        .unwrap()
        .entities
        .push(hanging);
    assert!(amg_doc::Feature::validate(&f).is_err());
}

#[test]
fn an_in_context_source_follows_its_occurrence() {
    let mut doc = Document::new("sk09-context");
    let o = doc
        .apply(Command::NewComponent {
            parent: doc.design().root(),
            name: "Part".to_owned(),
            transform: Transform::from_translation(Vec3::new(100.0, 0.0, 0.0)),
        })
        .unwrap();
    let component = o
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Component(c) => Some(*c),
            _ => None,
        })
        .unwrap();
    let occ = *doc.design().occurrence_paths(component)[0].last().unwrap();
    let mut u = SketchFeature::new(PlaneInput::OriginXY);
    let line = u
        .edit_sketch(|s| s.add_line(p(0.0, 0.0), p(10.0, 5.0)))
        .unwrap();
    let o = doc
        .apply(Command::AddFeature {
            at: None,
            feature: FeatureBox::new(u),
            context: component,
            name: None,
        })
        .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    let upstream = created_feature(&o);
    let sketch = add(
        &mut doc,
        FeatureBox::new(SketchFeature::new(PlaneInput::OriginXY)),
    );
    let source = EntityRef::InContext {
        path: vec![occ],
        target: Box::new(EntityRef::SketchCurve {
            sketch: SketchId(upstream),
            curve: SketchEntityKey(u64::from(line.0)),
        }),
    };
    let projected = project(&mut doc, sketch, source, ProjectionMode::Project)[0];
    assert_ok(&doc, sketch);
    assert_line(&geom(&doc, sketch), projected, p(100.0, 0.0), p(110.0, 5.0));

    // A quarter turn about Z, then 50 mm along Y.
    doc.apply(Command::SetOccurrenceTransform {
        occ,
        transform: Transform::from_translation(Vec3::new(0.0, 50.0, 0.0))
            * Transform::from_rotation_z(std::f64::consts::FRAC_PI_2),
    })
    .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    assert_ok(&doc, sketch);
    assert_line(&geom(&doc, sketch), projected, p(-5.0, 60.0), p(0.0, 50.0));
}

// ---- B-rep sources ----------------------------------------------------------------------------

#[cfg(feature = "occt")]
mod occt {
    use amg_core::Point3;
    use amg_doc::naming::{GeomSource, KernelGeom, default_kernel, topo_ref_at};
    use amg_doc::{BodyId, ErrorCause, TopoKind};
    use amg_features::solid::{BodyOp, Extrude, OpTarget};
    use amg_sketch::{Sketch, build};
    use serde::{Deserialize, Serialize};

    use super::*;

    /// A reference to the sub-entity of `body` whose centroid is nearest to `near`.
    fn pick(doc: &Document, body: BodyId, kind: TopoKind, near: [f64; 3]) -> EntityRef {
        let ev = doc.evaluated();
        let view: &amg_doc::Evaluated = ev;
        let source = KernelGeom::new(view, default_kernel());
        let near = Point3::from_array(near);
        let mut found: Vec<(f64, u32)> = GeomSource::all(&source, body, kind)
            .into_iter()
            .map(|(i, fp)| ((fp.centroid - near).length(), i))
            .collect();
        found.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert!(
            found.len() == 1 || found[1].0 - found[0].0 > 1e-6,
            "tied pick near {near:?}"
        );
        let mut r = topo_ref_at(view, default_kernel(), body, kind, found[0].1).unwrap();
        r.key = 1;
        EntityRef::Topo(r)
    }

    /// An extruded sketch with the user parameters its dimensions and distance use.
    struct Solid {
        doc: Document,
        params: Vec<ParamId>,
        upstream: FeatureId,
        extrude: FeatureId,
        body: BodyId,
    }

    impl Solid {
        fn set(&mut self, param: usize, expr: &str) {
            set_param(&mut self.doc, self.params[param], expr);
            assert_ok(&self.doc, self.upstream);
            assert_ok(&self.doc, self.extrude);
        }

        fn sketch_on(&mut self, plane: PlaneInput) -> FeatureId {
            let id = add(&mut self.doc, FeatureBox::new(SketchFeature::new(plane)));
            assert_ok(&self.doc, id);
            id
        }
    }

    /// Extrudes `u` by the parameter `tall` (the profile with the most holes).
    fn solid(mut doc: Document, params: Vec<ParamId>, u: SketchFeature) -> Solid {
        let upstream = add(&mut doc, FeatureBox::new(u));
        assert_ok(&doc, upstream);
        let g = geom(&doc, upstream);
        let ring = g
            .profiles
            .profiles
            .iter()
            .enumerate()
            .max_by_key(|(_, p)| p.holes.len())
            .unwrap()
            .0;
        let profile = profiles(&doc, upstream).swap_remove(ring);
        let e = Extrude::distance(
            vec![profile],
            "tall",
            OpTarget {
                op: BodyOp::NewBody,
                participants: Vec::new(),
            },
        );
        let extrude = add(&mut doc, FeatureBox::new(e));
        assert_ok(&doc, extrude);
        Solid {
            doc,
            params,
            upstream,
            extrude,
            body: body0(extrude),
        }
    }

    /// A rectangle from `(x0, y0)` with a fixed corner, width `w` and depth `d`.
    fn dimensioned_rect(s: &mut Sketch, a: Point2, b: Point2) -> Result<(), SketchError> {
        let built = build::rectangle_2pt(s, a, b)?;
        s.add_constraint(ConstraintKind::Fix {
            entity: built.entities[0],
        })?;
        s.add_constraint(dim(
            DimensionKind::Length {
                line: built.entities[4],
            },
            b.x - a.x,
        ))?;
        s.add_constraint(dim(
            DimensionKind::Length {
                line: built.entities[5],
            },
            b.y - a.y,
        ))?;
        Ok(())
    }

    fn bind(f: &mut SketchFeature, exprs: &[&str]) {
        let ids: Vec<_> = f.dims.keys().copied().collect();
        assert_eq!(ids.len(), exprs.len());
        for (id, e) in ids.into_iter().zip(exprs) {
            f.apply(SketchEdit::SetDimension {
                id,
                expr: inline(e),
            })
            .unwrap();
        }
    }

    /// `b` sits `v` above `a`: a signed dimension, so it keeps its side however far the
    /// projection under it moves.
    fn above(a: EntityId, b: EntityId, v: f64) -> ConstraintKind {
        dim(
            DimensionKind::VerticalDistance {
                a,
                b,
                side: amg_sketch::Side::Positive,
            },
            v,
        )
    }

    // ---- goldens ----------------------------------------------------------------------------

    /// One entity of a sketch: its kind and the numbers that place it.
    #[derive(Debug, Serialize, Deserialize)]
    struct Item {
        id: u32,
        kind: String,
        projected: bool,
        values: Vec<f64>,
    }

    #[derive(Debug, Serialize, Deserialize)]
    struct State {
        name: String,
        sketches: Vec<Vec<Item>>,
    }

    fn items(g: &SketchGeom) -> Vec<Item> {
        g.sketch
            .entities()
            .filter(|(_, e)| !matches!(e.kind, EntityKind::Point { .. }))
            .chain(
                g.sketch
                    .entities()
                    .filter(|(_, e)| matches!(e.kind, EntityKind::Point { .. })),
            )
            .map(|(id, e)| {
                let mut values = Vec::new();
                for c in e.kind.children() {
                    let q = g.sketch.point(c).unwrap();
                    values.extend([q.x, q.y]);
                }
                values.extend(
                    e.kind
                        .own_params()
                        .into_iter()
                        .map(|p| g.sketch.params().get(p)),
                );
                // Nine decimals keep the file stable; the comparison allows 1e-6.
                let values = values
                    .into_iter()
                    .map(|v| (v * 1e9).round() / 1e9 + 0.0)
                    .collect();
                Item {
                    id: id.0,
                    kind: e.kind.name().to_owned(),
                    projected: matches!(e.origin, EntityOrigin::Projected(_)),
                    values,
                }
            })
            .collect()
    }

    fn state(name: &str, doc: &Document, sketches: &[FeatureId]) -> State {
        State {
            name: name.to_owned(),
            sketches: sketches.iter().map(|&s| items(&geom(doc, s))).collect(),
        }
    }

    /// Compares with `tests/golden/sketch/sk09_<case>.json`; `AMG_BLESS=1` (never under CI)
    /// writes it instead.
    #[track_caller]
    fn check_golden(case: &str, got: &[State]) {
        let path = amg_testkit::golden_dir()
            .join("sketch")
            .join(format!("sk09_{case}.json"));
        if std::env::var("AMG_BLESS").is_ok_and(|v| v == "1") {
            assert!(
                std::env::var_os("CI").is_none_or(|v| v.is_empty()),
                "blessing is refused under CI"
            );
            std::fs::write(&path, serde_json::to_string_pretty(got).unwrap() + "\n").unwrap();
            return;
        }
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("{}: {e}; bless it with AMG_BLESS=1", path.display()));
        let want: Vec<State> = serde_json::from_str(&text).unwrap();
        assert_eq!(got.len(), want.len(), "{case}: states");
        for (g, w) in got.iter().zip(&want) {
            assert_eq!(g.name, w.name, "{case}");
            assert_eq!(g.sketches.len(), w.sketches.len(), "{case}/{}", g.name);
            for (gs, ws) in g.sketches.iter().zip(&w.sketches) {
                assert_eq!(gs.len(), ws.len(), "{case}/{}: entities", g.name);
                for (a, b) in gs.iter().zip(ws) {
                    assert_eq!(
                        (a.id, &a.kind, a.projected, a.values.len()),
                        (b.id, &b.kind, b.projected, b.values.len()),
                        "{case}/{}",
                        g.name
                    );
                    for (x, y) in a.values.iter().zip(&b.values) {
                        assert!(
                            (x - y).abs() <= TOL,
                            "{case}/{} entity {}: {:?} vs {:?}",
                            g.name,
                            a.id,
                            a.values,
                            b.values
                        );
                    }
                }
            }
        }
    }

    // ---- Done-when 1, model 1: box, projected edges -----------------------------------------

    /// Box `w × d × h` with its front top edge and right front edge projected onto XZ, a user
    /// line standing 5 mm on the top edge's far end and a circle on its upper end.
    struct Box1 {
        s: Solid,
        sketch: FeatureId,
        top: EntityId,
        upright: EntityId,
        post: EntityId,
        circle: EntityId,
    }

    fn box_model() -> Box1 {
        let mut doc = Document::new("sk09-box");
        let params = vec![
            add_param(&mut doc, "wide", "40 mm"),
            add_param(&mut doc, "deep", "20 mm"),
            add_param(&mut doc, "tall", "30 mm"),
        ];
        let mut u = SketchFeature::new(PlaneInput::OriginXY);
        u.edit_sketch(|s| dimensioned_rect(s, p(0.0, 0.0), p(40.0, 20.0)))
            .unwrap();
        bind(&mut u, &["wide", "deep"]);
        let mut s = solid(doc, params, u);
        let sketch = s.sketch_on(PlaneInput::OriginXZ);
        let top_ref = pick(&s.doc, s.body, TopoKind::Edge, [20.0, 0.0, 30.0]);
        let top = project(&mut s.doc, sketch, top_ref, ProjectionMode::Project)[0];
        let up_ref = pick(&s.doc, s.body, TopoKind::Edge, [40.0, 0.0, 15.0]);
        let upright = project(&mut s.doc, sketch, up_ref, ProjectionMode::Include)[0];

        let post = apply(
            &mut s.doc,
            sketch,
            SketchEdit::AddLine {
                a: p(41.0, 31.0),
                b: p(41.0, 36.0),
            },
        )[0];
        let g = geom(&s.doc, sketch);
        let (a, b) = point_ids(&g, top);
        let far = if g.sketch.point(a).unwrap().x > g.sketch.point(b).unwrap().x {
            a
        } else {
            b
        };
        let (foot, head) = point_ids(&g, post);
        constrain(
            &mut s.doc,
            sketch,
            ConstraintKind::Coincident { a: foot, b: far },
            None,
        );
        constrain(
            &mut s.doc,
            sketch,
            ConstraintKind::Vertical(HvTarget::Line(post)),
            None,
        );
        constrain(&mut s.doc, sketch, above(foot, head, 5.0), Some("5 mm"));
        let circle = apply(
            &mut s.doc,
            sketch,
            SketchEdit::AddCircle {
                center: p(40.5, 35.5),
                radius: 2.0,
            },
        )
        .pop()
        .unwrap();
        let g = geom(&s.doc, sketch);
        let EntityKind::Circle { center, .. } = g.sketch.entity(circle).unwrap().kind else {
            panic!()
        };
        constrain(
            &mut s.doc,
            sketch,
            ConstraintKind::Coincident { a: center, b: head },
            None,
        );
        constrain(
            &mut s.doc,
            sketch,
            dim(DimensionKind::Radius { curve: circle }, 2.0),
            Some("2 mm"),
        );
        assert_ok(&s.doc, sketch);
        Box1 {
            s,
            sketch,
            top,
            upright,
            post,
            circle,
        }
    }

    #[track_caller]
    fn expect_box(m: &Box1, w: f64, h: f64) {
        assert_ok(&m.s.doc, m.sketch);
        let g = geom(&m.s.doc, m.sketch);
        assert_line(&g, m.top, p(0.0, h), p(w, h));
        assert_line(&g, m.upright, p(w, 0.0), p(w, h));
        assert_line(&g, m.post, p(w, h), p(w, h + 5.0));
        let (c, r) = circle(&g, m.circle);
        close(c, p(w, h + 5.0));
        assert!((r - 2.0).abs() < TOL);
        assert!(is_locked(&g, m.top) && is_locked(&g, m.upright));
    }

    #[test]
    fn dw1_golden_box_edges_follow_height_and_sketch_edits() {
        let mut m = box_model();
        expect_box(&m, 40.0, 30.0);
        let mut states = vec![state("base", &m.s.doc, &[m.sketch])];
        m.s.set(2, "45 mm");
        expect_box(&m, 40.0, 45.0);
        states.push(state("height 45", &m.s.doc, &[m.sketch]));
        m.s.set(0, "55 mm");
        expect_box(&m, 55.0, 45.0);
        states.push(state("width 55", &m.s.doc, &[m.sketch]));
        m.s.set(2, "12 mm");
        expect_box(&m, 55.0, 12.0);
        states.push(state("height 12", &m.s.doc, &[m.sketch]));
        // Each link has its own selection key, whatever the picked reference carried.
        let f = current(&m.s.doc, m.sketch);
        let keys: Vec<u32> = f
            .links
            .values()
            .map(|l| match &l.source {
                EntityRef::Topo(t) => t.key,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(keys, [1, 2]);
        // The references resolved by name every time.
        let node = m.s.doc.evaluated().node(m.sketch).unwrap();
        assert_eq!(node.resolutions.len(), 2);
        for r in node.resolutions.iter() {
            let found = r.result.as_ref().unwrap();
            assert_eq!(found[0].quality, amg_doc::MatchQuality::Exact);
        }
        check_golden("box", &states);
    }

    // ---- Done-when 1, model 2: cylinder, projected circle and section lines -----------------

    #[test]
    fn dw1_golden_cylinder_circle_and_side_lines_follow_radius_and_height() {
        let mut doc = Document::new("sk09-cylinder");
        let params = vec![
            add_param(&mut doc, "radius", "6 mm"),
            add_param(&mut doc, "tall", "30 mm"),
        ];
        let mut u = SketchFeature::new(PlaneInput::OriginXY);
        u.edit_sketch(|s| {
            let c = s.add_circle(p(10.0, 5.0), 6.0)?;
            let EntityKind::Circle { center, .. } = s.entity(c).unwrap().kind else {
                unreachable!()
            };
            s.add_constraint(ConstraintKind::Fix { entity: center })?;
            s.add_constraint(dim(DimensionKind::Radius { curve: c }, 6.0))?;
            Ok(())
        })
        .unwrap();
        bind(&mut u, &["radius"]);
        let mut s = solid(doc, params, u);

        // The top circle on XY, with a concentric half-size ring dimension and a point on it.
        let flat = s.sketch_on(PlaneInput::OriginXY);
        let rim = pick(&s.doc, s.body, TopoKind::Edge, [10.0, 5.0, 30.0]);
        let projected = project(&mut s.doc, flat, rim, ProjectionMode::Project)[0];
        let ring = apply(
            &mut s.doc,
            flat,
            SketchEdit::AddCircle {
                center: p(10.2, 5.1),
                radius: 3.0,
            },
        )
        .pop()
        .unwrap();
        constrain(
            &mut s.doc,
            flat,
            ConstraintKind::Concentric {
                a: ring,
                b: projected,
            },
            None,
        );
        constrain(
            &mut s.doc,
            flat,
            dim(DimensionKind::Radius { curve: ring }, 3.0),
            Some("3 mm"),
        );
        let marker = apply(&mut s.doc, flat, SketchEdit::AddPoint { at: p(16.0, 5.0) })[0];
        let g = geom(&s.doc, flat);
        let EntityKind::Circle { center, .. } = g.sketch.entity(projected).unwrap().kind else {
            panic!("the rim projects to a circle")
        };
        constrain(
            &mut s.doc,
            flat,
            ConstraintKind::PointOnCurve {
                point: marker,
                curve: projected,
            },
            None,
        );
        constrain(
            &mut s.doc,
            flat,
            ConstraintKind::Horizontal(HvTarget::Points(center, marker)),
            None,
        );

        // The section of the lateral face with XZ: two lines, and a point halfway up one.
        let front = s.sketch_on(PlaneInput::OriginXZ);
        let side = pick(&s.doc, s.body, TopoKind::Face, [10.0, 5.0, 15.0]);
        let lines = project(&mut s.doc, front, side, ProjectionMode::IntersectionCurve);
        assert_eq!(lines.len(), 2);
        let mid = apply(&mut s.doc, front, SketchEdit::AddPoint { at: p(7.0, 14.0) })[0];
        constrain(
            &mut s.doc,
            front,
            ConstraintKind::Midpoint {
                point: mid,
                line: lines[0],
            },
            None,
        );

        let expect = |s: &Solid, r: f64, h: f64| {
            assert_ok(&s.doc, flat);
            assert_ok(&s.doc, front);
            let g = geom(&s.doc, flat);
            let (c, radius) = circle(&g, projected);
            close(c, p(10.0, 5.0));
            assert!((radius - r).abs() < TOL, "{radius}");
            let (c, radius) = circle(&g, ring);
            close(c, p(10.0, 5.0));
            assert!((radius - 3.0).abs() < TOL);
            close(g.sketch.point(marker).unwrap(), p(10.0 + r, 5.0));
            let g = geom(&s.doc, front);
            let dx = (r * r - 25.0).sqrt();
            let mut xs: Vec<f64> = lines.iter().map(|&l| ends(&g, l).0.x).collect();
            xs.sort_by(f64::total_cmp);
            assert!((xs[0] - (10.0 - dx)).abs() < TOL, "{xs:?}");
            assert!((xs[1] - (10.0 + dx)).abs() < TOL, "{xs:?}");
            for &l in &lines {
                let x = ends(&g, l).0.x;
                assert_line(&g, l, p(x, 0.0), p(x, h));
            }
            let x = ends(&g, lines[0]).0.x;
            close(g.sketch.point(mid).unwrap(), p(x, 0.5 * h));
        };
        let both = [flat, front];
        expect(&s, 6.0, 30.0);
        let mut states = vec![state("base", &s.doc, &both)];
        let first = ends(&geom(&s.doc, front), lines[0]).0.x;
        s.set(1, "18 mm");
        expect(&s, 6.0, 18.0);
        states.push(state("height 18", &s.doc, &both));
        s.set(0, "8.5 mm");
        expect(&s, 8.5, 18.0);
        states.push(state("radius 8.5", &s.doc, &both));
        // The same entity stays the same side of the cylinder.
        let now = ends(&geom(&s.doc, front), lines[0]).0.x;
        assert_eq!(first < 10.0, now < 10.0);
        check_golden("cylinder", &states);
    }

    // ---- Done-when 1, model 3: plate with a hole, body section ------------------------------

    #[test]
    fn dw1_golden_plate_section_follows_width_hole_and_thickness() {
        let mut doc = Document::new("sk09-plate");
        let params = vec![
            add_param(&mut doc, "wide", "40 mm"),
            add_param(&mut doc, "deep", "20 mm"),
            add_param(&mut doc, "radius", "4 mm"),
            add_param(&mut doc, "tall", "8 mm"),
        ];
        let mut u = SketchFeature::new(PlaneInput::OriginXY);
        u.edit_sketch(|s| {
            dimensioned_rect(s, p(0.0, -10.0), p(40.0, 10.0))?;
            // Off the XZ plane, so that the section misses the seam of the hole.
            let c = s.add_circle(p(20.0, 1.0), 4.0)?;
            let EntityKind::Circle { center, .. } = s.entity(c).unwrap().kind else {
                unreachable!()
            };
            s.add_constraint(ConstraintKind::Fix { entity: center })?;
            s.add_constraint(dim(DimensionKind::Radius { curve: c }, 4.0))?;
            Ok(())
        })
        .unwrap();
        bind(&mut u, &["wide", "deep", "radius"]);
        let mut s = solid(doc, params, u);
        let front = s.sketch_on(PlaneInput::OriginXZ);
        let lines = project(
            &mut s.doc,
            front,
            EntityRef::Body(s.body),
            ProjectionMode::IntersectionCurve,
        );
        assert_eq!(lines.len(), 8, "two rectangles");

        // The expected segments for width `w`, hole radius `r` and thickness `h`.
        let segments = |w: f64, r: f64, h: f64| {
            let dx = (r * r - 1.0).sqrt();
            let (a, b) = (20.0 - dx, 20.0 + dx);
            let mut v = Vec::new();
            for (x0, x1) in [(0.0, a), (b, w)] {
                v.push((p(x0, 0.0), p(x1, 0.0)));
                v.push((p(x0, h), p(x1, h)));
            }
            for x in [0.0, a, b, w] {
                v.push((p(x, 0.0), p(x, h)));
            }
            v
        };
        // Which expected segment each entity is.
        let which = |g: &SketchGeom, w: f64, r: f64, h: f64| -> Vec<usize> {
            let all = segments(w, r, h);
            lines
                .iter()
                .map(|&l| {
                    let (u, v) = sorted_ends(g, l);
                    all.iter()
                        .position(|(a, b)| a.distance(u) < TOL && b.distance(v) < TOL)
                        .unwrap_or_else(|| panic!("{l:?}: {u:?}-{v:?} is not a segment"))
                })
                .collect()
        };
        let base = which(&geom(&s.doc, front), 40.0, 4.0, 8.0);
        let mut seen = base.clone();
        seen.sort_unstable();
        assert_eq!(seen, (0..8).collect::<Vec<_>>(), "all eight, once each");

        // The section closes into two profiles, and a user line dimensioned above the entity
        // that is the top of the right rectangle follows it.
        assert_eq!(geom(&s.doc, front).profiles.profiles.len(), 2);
        let top_right = lines[base.iter().position(|&i| i == 3).unwrap()];
        let pin = apply(&mut s.doc, front, SketchEdit::AddPoint { at: p(32.0, 8.0) })[0];
        constrain(
            &mut s.doc,
            front,
            ConstraintKind::Midpoint {
                point: pin,
                line: top_right,
            },
            None,
        );
        let post = apply(
            &mut s.doc,
            front,
            SketchEdit::AddLine {
                a: p(32.1, 8.1),
                b: p(32.1, 11.0),
            },
        )[0];
        let (foot, head) = point_ids(&geom(&s.doc, front), post);
        constrain(
            &mut s.doc,
            front,
            ConstraintKind::Coincident { a: foot, b: pin },
            None,
        );
        constrain(
            &mut s.doc,
            front,
            ConstraintKind::Vertical(HvTarget::Line(post)),
            None,
        );
        constrain(&mut s.doc, front, above(foot, head, 3.0), Some("3 mm"));

        let expect = |s: &Solid, w: f64, r: f64, h: f64| {
            assert_ok(&s.doc, front);
            let g = geom(&s.doc, front);
            assert_eq!(which(&g, w, r, h), base, "every entity keeps its segment");
            let mid = 0.5 * (20.0 + (r * r - 1.0).sqrt() + w);
            close(g.sketch.point(pin).unwrap(), p(mid, h));
            assert_line(&g, post, p(mid, h), p(mid, h + 3.0));
        };
        expect(&s, 40.0, 4.0, 8.0);
        let mut states = vec![state("base", &s.doc, &[front])];
        for (index, value, w, r, h, name) in [
            (3, "12 mm", 40.0, 4.0, 12.0, "thickness 12"),
            (2, "6 mm", 40.0, 6.0, 12.0, "hole 6"),
            (0, "52 mm", 52.0, 6.0, 12.0, "width 52"),
        ] {
            s.set(index, value);
            expect(&s, w, r, h);
            states.push(state(name, &s.doc, &[front]));
        }
        check_golden("plate_section", &states);
    }

    // ---- Done-when 2 with solid sources -----------------------------------------------------

    #[test]
    fn dw2_an_edge_cut_away_warns_and_keeps_the_last_geometry() {
        let mut m = box_model();
        m.s.set(2, "45 mm");
        let commit = current(&m.s.doc, m.sketch).commit_solved(&geom(&m.s.doc, m.sketch));
        apply(&mut m.s.doc, m.sketch, commit);
        expect_box(&m, 40.0, 45.0);
        let before = geom(&m.s.doc, m.sketch);

        // A cut before the sketch removes the right front corner, and the upright edge with it.
        let corner = add(
            &mut m.s.doc,
            FeatureBox::new(rect_sketch(&[[30.0, -5.0, 50.0, 5.0]])),
        );
        let cut = Extrude::distance(
            profiles(&m.s.doc, corner),
            "100 mm",
            OpTarget {
                op: BodyOp::Cut,
                participants: vec![m.s.body],
            },
        );
        let cut = add(&mut m.s.doc, FeatureBox::new(cut));
        assert_ok(&m.s.doc, cut);
        m.s.doc
            .apply(Command::MoveFeature { id: corner, to: 2 })
            .unwrap();
        m.s.doc
            .apply(Command::MoveFeature { id: cut, to: 3 })
            .unwrap();
        m.s.doc.recompute_blocking(jobs()).unwrap();
        assert_ok(&m.s.doc, cut);

        let w = warnings(&m.s.doc, m.sketch);
        let lost = w
            .iter()
            .find(|d| d.message.starts_with("projection 1 "))
            .unwrap_or_else(|| panic!("{w:?}"));
        assert!(
            matches!(lost.cause, Some(ErrorCause::Resolve(_))),
            "{lost:?}"
        );
        let g = geom(&m.s.doc, m.sketch);
        assert_line(&g, m.upright, p(40.0, 0.0), p(40.0, 45.0));
        let (a, b) = ends(&before, m.upright);
        assert_eq!(ends(&g, m.upright), (a, b), "bit for bit");
        // The other link is still live: the top edge now ends at the cut.
        assert_line(&g, m.top, p(0.0, 45.0), p(30.0, 45.0));
    }

    #[test]
    fn dw2_a_suppressed_or_deleted_extrude_warns_and_keeps_the_last_geometry() {
        let mut m = box_model();
        m.s.set(0, "50 mm");
        let commit = current(&m.s.doc, m.sketch).commit_solved(&geom(&m.s.doc, m.sketch));
        apply(&mut m.s.doc, m.sketch, commit);
        // Something downstream of the sketch must keep computing.
        let after = add(
            &mut m.s.doc,
            FeatureBox::new(rect_sketch(&[[0.0, 0.0, 1.0, 1.0]])),
        );
        let kept = geom(&m.s.doc, m.sketch);
        for command in [
            Command::SetSuppressed {
                ids: vec![m.s.extrude],
                suppressed: true,
            },
            Command::DeleteFeatures {
                ids: vec![m.s.extrude],
            },
        ] {
            m.s.doc.apply(command).unwrap();
            m.s.doc.recompute_blocking(jobs()).unwrap();
            let w = warnings(&m.s.doc, m.sketch);
            assert_eq!(w.len(), 2, "one per link: {w:?}");
            let g = geom(&m.s.doc, m.sketch);
            assert_eq!(
                g.sketch.params().as_slice(),
                kept.sketch.params().as_slice()
            );
            assert_line(&g, m.top, p(0.0, 30.0), p(50.0, 30.0));
            assert_line(&g, m.post, p(50.0, 30.0), p(50.0, 35.0));
            assert_ok(&m.s.doc, after);
        }
    }

    // ---- modes and curve kinds --------------------------------------------------------------

    #[test]
    fn modes_on_solid_sources() {
        let mut m = box_model();
        let flat = m.s.sketch_on(PlaneInput::OriginXY);
        let doc = &m.s.doc;
        let upright = pick(doc, m.s.body, TopoKind::Edge, [40.0, 20.0, 15.0]);
        // Seen end on, an edge is a point; it crosses XY at its foot.
        for mode in [ProjectionMode::Project, ProjectionMode::Intersect] {
            let got = preview(doc, flat, &upright, mode).unwrap();
            let [ProjectedGeom::Point(q)] = got.as_slice() else {
                panic!("{mode:?}: {got:?}")
            };
            close(*q, p(40.0, 20.0));
        }
        assert!(matches!(
            preview(doc, flat, &upright, ProjectionMode::Include),
            Err(ProjectionError::NotInPlane(d)) if (d - 30.0).abs() < TOL
        ));
        // A face gives its four edges; a vertex a point; the body only a section.
        let top = pick(doc, m.s.body, TopoKind::Face, [20.0, 10.0, 30.0]);
        let got = preview(doc, flat, &top, ProjectionMode::Project).unwrap();
        assert_eq!(got.len(), 4, "{got:?}");
        assert!(got.iter().all(|g| g.name() == "line"));
        assert!(matches!(
            preview(doc, flat, &top, ProjectionMode::Include),
            Err(ProjectionError::NotInPlane(_))
        ));
        let bottom = pick(doc, m.s.body, TopoKind::Face, [20.0, 10.0, 0.0]);
        assert_eq!(
            preview(doc, flat, &bottom, ProjectionMode::Include)
                .unwrap()
                .len(),
            4
        );
        let vertex = pick(doc, m.s.body, TopoKind::Vertex, [40.0, 20.0, 30.0]);
        assert_eq!(
            preview(doc, flat, &vertex, ProjectionMode::Project)
                .unwrap()
                .len(),
            1
        );
        let body = EntityRef::Body(m.s.body);
        assert!(matches!(
            preview(doc, flat, &body, ProjectionMode::Project),
            Err(ProjectionError::NotApplicable { .. })
        ));
        // XY only touches the bottom of the box; the top face does not meet it.
        assert!(matches!(
            preview(doc, flat, &top, ProjectionMode::IntersectionCurve),
            Err(ProjectionError::Empty(_))
        ));
    }

    #[test]
    fn curved_sketch_sources_project_and_refresh() {
        let mut doc = Document::new("sk09-curves");
        let mut u = SketchFeature::new(PlaneInput::OriginXY);
        let ids = u
            .edit_sketch(|s| {
                Ok([
                    s.add_circle(p(0.0, 0.0), 5.0)?,
                    s.add_arc(p(20.0, 0.0), p(24.0, 0.0), p(20.0, 4.0))?,
                    s.add_ellipse(p(40.0, 0.0), p(46.0, 0.0), 2.0)?,
                    s.add_elliptic_arc(p(60.0, 0.0), p(62.0, 0.0), 5.0, 0.3, 2.0)?,
                    s.add_spline(
                        3,
                        &[
                            p(80.0, 0.0),
                            p(82.0, 6.0),
                            p(86.0, -3.0),
                            p(90.0, 2.0),
                            p(94.0, 0.0),
                        ],
                        None,
                        None,
                    )?,
                ])
            })
            .unwrap();
        let upstream = add(&mut doc, FeatureBox::new(u));
        let flat = add(
            &mut doc,
            FeatureBox::new(SketchFeature::new(PlaneInput::OriginXY)),
        );
        let source = |e: EntityId| EntityRef::SketchCurve {
            sketch: SketchId(upstream),
            curve: SketchEntityKey(u64::from(e.0)),
        };
        let made: Vec<EntityId> = ids
            .iter()
            .map(|&e| project(&mut doc, flat, source(e), ProjectionMode::Include)[0])
            .collect();
        let same = |doc: &Document| {
            assert_ok(doc, flat);
            let (a, b) = (geom(doc, upstream), geom(doc, flat));
            for (&from, &to) in ids.iter().zip(&made) {
                let (x, y) = (a.sketch.curve(from).unwrap(), b.sketch.curve(to).unwrap());
                assert_eq!(
                    a.sketch.entity(from).unwrap().kind.name(),
                    b.sketch.entity(to).unwrap().kind.name()
                );
                let (x1, y1) = (x.param_range().1, y.param_range().1);
                for i in 0..=8 {
                    let f = f64::from(i) / 8.0;
                    let q = x.eval(f * x1);
                    // The projection may run the other way along the curve.
                    let d = y
                        .eval(f * y1)
                        .distance(q)
                        .min(y.eval((1.0 - f) * y1).distance(q));
                    assert!(d < TOL, "{from:?} at {f}: {d}");
                }
                assert!(is_locked(&b, to));
            }
        };
        same(&doc);
        // Move every source: the projections follow, with the same entities.
        let mut u = current(&doc, upstream);
        u.edit_sketch(|s| {
            amg_sketch::transform::transform(
                s,
                &ids,
                amg_sketch::transform::translation(p(3.0, -2.0)),
                false,
            )
        })
        .unwrap();
        edit(&mut doc, upstream, FeatureBox::new(u));
        same(&doc);

        // Seen edge-on, the circle is two segments (it turns back at its ends).
        let front = add(
            &mut doc,
            FeatureBox::new(SketchFeature::new(PlaneInput::OriginXZ)),
        );
        let got = preview(&doc, front, &source(ids[0]), ProjectionMode::Project).unwrap();
        assert!(got.iter().all(|g| g.name() == "line"), "{got:?}");
        assert!(got.len() >= 2, "{got:?}");
        // And it meets XZ in two points.
        let got = preview(&doc, front, &source(ids[0]), ProjectionMode::Intersect).unwrap();
        let mut xs: Vec<f64> = got
            .iter()
            .map(|g| match g {
                ProjectedGeom::Point(q) => q.x,
                other => panic!("{other:?}"),
            })
            .collect();
        xs.sort_by(f64::total_cmp);
        let dx = (25.0_f64 - 4.0).sqrt();
        assert_eq!(xs.len(), 2);
        assert!((xs[0] - (3.0 - dx)).abs() < TOL && (xs[1] - (3.0 + dx)).abs() < TOL);
    }
}
