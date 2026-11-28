//! A self-made design that stands in for an owner design: its STEP file is written with our own
//! exporter into a temporary `fusion/` folder, so the whole compare path (folder walk, STEP
//! import, rebuild, metrics, bounds) runs without private data. The geometry is also checked
//! against its analytic volume and bbox, so the reference is not only compared with itself.

use std::f64::consts::PI;

use amg_core::{Point2, Point3};
use amg_doc::naming::{GeomSource, KernelGeom, default_kernel, topo_ref_at};
use amg_doc::{
    BodyId, Document, EntityRef, Evaluated, Expr, ExprSrc, FeatureBox, TopoKind, TopoRef,
};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{BodyOp, Extent, Extrude, Fillet, OpTarget, Start};

use super::super::solid_common::{add, body0, extrude_rects, profiles};
use super::compare::Metrics;

fn inline(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

/// The reference of the edge whose centroid is `near`.
fn edge_at(doc: &Document, body: BodyId, near: [f64; 3], key: u32) -> TopoRef {
    let ev: &Evaluated = doc.evaluated();
    let near = Point3::from_array(near);
    let (_, index) = GeomSource::all(&KernelGeom::new(ev, default_kernel()), body, TopoKind::Edge)
        .into_iter()
        .map(|(i, fp)| ((fp.centroid - near).length(), i))
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .expect("the body has edges");
    let mut r = topo_ref_at(ev, default_kernel(), body, TopoKind::Edge, index).expect("a name");
    r.key = key;
    r
}

fn circles(centres: &[[f64; 2]], r: f64) -> SketchFeature {
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    f.edit_sketch(|s| {
        for c in centres {
            s.add_circle(Point2::new(c[0], c[1]), r)?;
        }
        Ok(())
    })
    .expect("circles");
    f
}

fn circle_profiles(doc: &mut Document, centres: &[[f64; 2]], r: f64) -> Vec<EntityRef> {
    let s = add(doc, FeatureBox::new(circles(centres, r)));
    profiles(doc, s)
}

/// A 60 × 40 × 10 plate with filleted vertical edges (r 5), a boss (Ø16) from the top face up
/// to `boss_top`, a through hole of radius `bore` in the boss and two Ø5 mounting holes.
fn bracket_with(doc: &mut Document, boss_top: f64, bore: f64) {
    let (_, base) = extrude_rects(
        doc,
        &[[0.0, 0.0, 60.0, 40.0]],
        10.0,
        BodyOp::NewBody,
        Vec::new(),
    );
    let body = body0(base);
    let edges = [[0.0, 0.0], [60.0, 0.0], [60.0, 40.0], [0.0, 40.0]]
        .into_iter()
        .zip(1..)
        .map(|(p, key)| edge_at(doc, body, [p[0], p[1], 5.0], key))
        .collect();
    add(doc, FeatureBox::new(Fillet::constant(edges, "5 mm")));

    let target = |op| OpTarget {
        op,
        participants: vec![body],
    };
    let boss = circle_profiles(doc, &[[30.0, 20.0]], 8.0);
    let mut e = Extrude::distance(
        boss,
        &format!("{} mm", boss_top - 10.0),
        target(BodyOp::Join),
    );
    e.start = Start::Offset(inline("10 mm"));
    add(doc, FeatureBox::new(e));

    let centre = circle_profiles(doc, &[[30.0, 20.0]], bore);
    let mut e = Extrude::distance(centre, "1 mm", target(BodyOp::Cut));
    e.side1.extent = Extent::All;
    add(doc, FeatureBox::new(e));

    let mounting = circle_profiles(doc, &[[10.0, 20.0], [50.0, 20.0]], 2.5);
    let mut e = Extrude::distance(mounting, "1 mm", target(BodyOp::Cut));
    e.side1.extent = Extent::All;
    add(doc, FeatureBox::new(e));
}

/// The reference design.
pub fn bracket(doc: &mut Document) {
    bracket_with(doc, 22.0, 3.0);
}

/// A script that is 0.01 mm too tall: only the bbox bound can catch it.
pub fn bracket_taller(doc: &mut Document) {
    bracket_with(doc, 22.01, 3.0);
}

/// A script with a bore 0.05 mm too wide: the bbox is the same, volume is not.
pub fn bracket_wider_bore(doc: &mut Document) {
    bracket_with(doc, 22.0, 3.05);
}

/// A script whose last feature fails (a cut that touches nothing).
pub fn bracket_broken(doc: &mut Document) {
    bracket(doc);
    let body = body0(
        doc.design()
            .timeline()
            .nodes()
            .nth(1)
            .expect("the base extrude")
            .id,
    );
    let far = circle_profiles(doc, &[[500.0, 500.0]], 1.0);
    let e = Extrude::distance(
        far,
        "1 mm",
        OpTarget {
            op: BodyOp::Cut,
            participants: vec![body],
        },
    );
    add(doc, FeatureBox::new(e));
}

/// The analytic metrics of [`bracket`] that do not depend on how faces are split: volume and
/// bbox (area and tolerance are left at zero).
pub fn bracket_analytic() -> Metrics {
    let plate = 60.0 * 40.0 * 10.0 - 4.0 * (25.0 - PI * 25.0 / 4.0) * 10.0;
    let boss = PI * 64.0 * 12.0;
    let bore = PI * 9.0 * 22.0;
    let mounting = 2.0 * PI * 6.25 * 10.0;
    Metrics {
        solids: 1,
        volume: plate + boss - bore - mounting,
        area: 0.0,
        min: [0.0, 0.0, 0.0],
        max: [60.0, 40.0, 22.0],
        tolerance: 0.0,
    }
}
