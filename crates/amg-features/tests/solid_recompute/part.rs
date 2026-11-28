//! The 50-feature part of SOL-29 Done-when 2, shared by `tests/solid_recompute_50.rs` and
//! `benches/sol_recompute.rs`: a filleted, shelled enclosure with bosses, bored holes, ribs, a
//! vent-hole pattern, a pin pattern, wall cut-outs (one sketched on a model face), fillets,
//! chamfers and a rim notch. The first sketch drives the enclosure's width through a dimension,
//! so editing it changes the body every later feature works on.
//!
//! `solid.hole` (SOL-12) does not exist yet: the holes are circle cut-extrudes.

#![allow(dead_code, reason = "shared by a test and a bench")]

use amg_core::{Point2, Point3};
use amg_doc::naming::{GeomSource, KernelGeom, default_kernel, topo_ref_at};
use amg_doc::{
    BodyId, Command, Document, EntityRef, Evaluated, Expr, ExprSrc, FeatureBox, FeatureId, Health,
    TopoKind, TopoRef,
};
use amg_features::construct::{OriginEntity, RefInput};
use amg_features::sketch::{PlaneInput, SketchEdit, SketchFeature};
use amg_features::solid::{
    BodyOp, Chamfer, ComputeType, Direction, Extent, Extrude, Fillet, OpTarget, Pattern,
    PatternDir, PatternKind, PatternObjects, Shell, Spacing, Start,
};
use amg_sketch::{ConstraintId, ConstraintKind, Dimension, DimensionKind, build};

use crate::solid_common::{body0, created_feature, jobs, profiles};

/// The number of timeline features of the part.
pub const FEATURES: usize = 50;
/// The width the part is built with, mm.
pub const WIDTH: f64 = 100.0;

/// The built part.
pub struct Part {
    /// The document, recomputed.
    pub doc: Document,
    /// Every feature with its label, in timeline order.
    pub features: Vec<(FeatureId, String)>,
    /// The enclosure body.
    pub body: BodyId,
    /// The width dimension of the first sketch.
    width: ConstraintId,
}

fn inline(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

struct Builder {
    doc: Document,
    features: Vec<(FeatureId, String)>,
}

impl Builder {
    /// Adds a feature, recomputes and requires it to be healthy.
    fn add(&mut self, label: &str, feature: FeatureBox) -> FeatureId {
        let root = self.doc.design().root();
        let o = self
            .doc
            .apply(Command::AddFeature {
                at: None,
                feature,
                context: root,
                name: Some(label.to_owned()),
            })
            .unwrap_or_else(|e| panic!("{label}: {e}"));
        self.doc.recompute_blocking(jobs()).expect("recompute");
        let id = created_feature(&o);
        let health = &self.doc.evaluated().node(id).expect("evaluated").health;
        assert_eq!(*health, Health::Ok, "{label}");
        self.features.push((id, label.to_owned()));
        id
    }

    /// A sketch feature plus an extrude of all its profiles, shaped by `shape`.
    fn extruded(
        &mut self,
        label: &str,
        sketch: SketchFeature,
        target: OpTarget,
        distance: f64,
        shape: impl FnOnce(&mut Extrude),
    ) -> FeatureId {
        let s = self.add(&format!("{label} sketch"), FeatureBox::new(sketch));
        let mut e = Extrude::distance(profiles(&self.doc, s), &format!("{distance} mm"), target);
        shape(&mut e);
        self.add(label, FeatureBox::new(e))
    }

    /// The reference of the `kind` entity whose centroid is nearest to `near`. Of concentric
    /// entities (a boss rim and its bore) the larger one is picked.
    fn pick(&self, body: BodyId, kind: TopoKind, near: [f64; 3], key: u32) -> TopoRef {
        let ev: &Evaluated = self.doc.evaluated();
        let at = Point3::from_array(near);
        let mut found: Vec<(f64, f64, u32)> =
            GeomSource::all(&KernelGeom::new(ev, default_kernel()), body, kind)
                .into_iter()
                .map(|(i, fp)| ((fp.centroid - at).length(), fp.measure, i))
                .collect();
        found.sort_by(|a, b| a.0.total_cmp(&b.0));
        let nearest = found.first().expect("the body has entities").0;
        let (_, _, index) = found
            .into_iter()
            .take_while(|f| f.0 - nearest <= 1e-6)
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .expect("the nearest");
        let mut r = topo_ref_at(ev, default_kernel(), body, kind, index).expect("a name");
        r.key = key;
        r
    }

    fn edges(&self, body: BodyId, near: &[[f64; 3]]) -> Vec<TopoRef> {
        near.iter()
            .zip(1..)
            .map(|(p, key)| self.pick(body, TopoKind::Edge, *p, key))
            .collect()
    }
}

fn rect(plane: PlaneInput, r: [f64; 4]) -> SketchFeature {
    let mut f = SketchFeature::new(plane);
    f.edit_sketch(|s| build::rectangle_2pt(s, Point2::new(r[0], r[1]), Point2::new(r[2], r[3])))
        .expect("rectangle");
    f
}

fn circle(plane: PlaneInput, c: [f64; 2], r: f64) -> SketchFeature {
    let mut f = SketchFeature::new(plane);
    f.edit_sketch(|s| s.add_circle(Point2::new(c[0], c[1]), r))
        .expect("circle");
    f
}

/// The base outline: `WIDTH` × 60 with the origin corner fixed and a driving width dimension.
fn outline() -> (SketchFeature, ConstraintId) {
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    let built = f
        .edit_sketch(|s| build::rectangle_2pt(s, Point2::new(0.0, 0.0), Point2::new(WIDTH, 60.0)))
        .expect("rectangle");
    // Entities: four corner points, then the lines bottom, right, top, left.
    let (corner, bottom) = (built.entities[0], built.entities[4]);
    f.apply(SketchEdit::AddConstraint {
        kind: ConstraintKind::Fix { entity: corner },
        expr: None,
    })
    .expect("fixed corner");
    f.apply(SketchEdit::AddConstraint {
        kind: ConstraintKind::Dimension(Dimension {
            kind: DimensionKind::Length { line: bottom },
            value: WIDTH,
            driving: true,
            label_pos: Point2::new(0.0, 0.0),
        }),
        expr: Some(inline(&format!("{WIDTH} mm"))),
    })
    .expect("width dimension");
    let width = *f.dims.keys().next().expect("one dimension");
    (f, width)
}

fn pattern(seed: FeatureId, spacing: f64, nx: usize, ny: usize) -> Pattern {
    let dir = |axis, count: usize| PatternDir {
        dir: RefInput::Origin(axis),
        count: inline(&count.to_string()),
        spacing: Spacing::Spacing(inline(&format!("{spacing} mm"))),
        flip: false,
        symmetric: false,
    };
    Pattern {
        objects: PatternObjects::Features(vec![seed]),
        kind: PatternKind::Rectangular {
            d1: dir(OriginEntity::XAxis, nx),
            d2: Some(dir(OriginEntity::YAxis, ny)),
        },
        suppressed: Vec::new(),
        compute: ComputeType::Identical,
        target: None,
    }
}

/// Builds the part.
///
/// # Panics
/// If any feature is not healthy.
pub fn build_part() -> Part {
    let mut b = Builder {
        doc: Document::new("sol-recompute-50"),
        features: Vec::new(),
    };
    let on_xy = || PlaneInput::OriginXY;

    // The shell of the enclosure: 100 × 60 × 20, corners r6, 2 mm walls, open at the top.
    let (sketch, width) = outline();
    let s = b.add("outline sketch", FeatureBox::new(sketch));
    let base = b.add(
        "base extrude",
        FeatureBox::new(Extrude::distance(
            profiles(&b.doc, s),
            "20 mm",
            OpTarget::default(),
        )),
    );
    let body = body0(base);
    let corners = b.edges(
        body,
        &[
            [0.0, 0.0, 10.0],
            [WIDTH, 0.0, 10.0],
            [WIDTH, 60.0, 10.0],
            [0.0, 60.0, 10.0],
        ],
    );
    b.add(
        "corner fillet",
        FeatureBox::new(Fillet::constant(corners, "6 mm")),
    );
    let top = b.pick(body, TopoKind::Face, [WIDTH / 2.0, 30.0, 20.0], 1);
    b.add("shell", FeatureBox::new(Shell::inside(vec![top], "2 mm")));

    let on = |op| OpTarget {
        op,
        participants: vec![body],
    };
    let from = |z: f64| move |e: &mut Extrude| e.start = Start::Offset(inline(&format!("{z} mm")));

    // Four bosses on the floor, each with a blind bore.
    let bosses = [[12.0, 12.0], [88.0, 12.0], [12.0, 48.0], [88.0, 48.0]];
    for (i, c) in bosses.into_iter().enumerate() {
        let label = format!("boss {}", i + 1);
        let sk = circle(on_xy(), c, 4.0);
        b.extruded(&label, sk, on(BodyOp::Join), 11.0, from(1.0));
    }
    for (i, c) in bosses.into_iter().enumerate() {
        let label = format!("boss hole {}", i + 1);
        let sk = circle(on_xy(), c, 1.5);
        b.extruded(&label, sk, on(BodyOp::Cut), 10.0, from(4.0));
    }

    // A 6 × 4 pattern of vent holes through the floor.
    let all = |e: &mut Extrude| e.side1.extent = Extent::All;
    let vent = b.extruded(
        "vent hole",
        circle(on_xy(), [30.0, 20.0], 1.5),
        on(BodyOp::Cut),
        1.0,
        all,
    );
    b.add("vent pattern", FeatureBox::new(pattern(vent, 8.0, 6, 4)));

    // Four ribs.
    let ribs = [
        [18.0, 18.0, 19.5, 42.0],
        [80.5, 18.0, 82.0, 42.0],
        [30.0, 8.0, 70.0, 9.5],
        [30.0, 50.5, 70.0, 52.0],
    ];
    for (i, r) in ribs.into_iter().enumerate() {
        let label = format!("rib {}", i + 1);
        b.extruded(&label, rect(on_xy(), r), on(BodyOp::Join), 5.0, from(1.0));
    }

    // Cut-outs: two in the front wall, one in the back wall (sketched on that face), one in
    // the left wall.
    let through = |e: &mut Extrude| e.direction = Direction::Symmetric { whole_length: true };
    for (i, u) in [30.0, 60.0].into_iter().enumerate() {
        let label = format!("front cut-out {}", i + 1);
        let sk = rect(PlaneInput::OriginXZ, [u, 8.0, u + 10.0, 14.0]);
        b.extruded(&label, sk, on(BodyOp::Cut), 6.0, through);
    }
    let back = b.pick(body, TopoKind::Face, [WIDTH / 2.0, 60.0, 10.0], 1);
    // The face frame of the back wall has x along world X and y along world −Z.
    let sk = rect(
        PlaneInput::Ref(EntityRef::Topo(back)),
        [40.0, -14.0, 60.0, -8.0],
    );
    b.extruded("back cut-out", sk, on(BodyOp::Cut), 4.0, |e| e.flip = true);
    b.extruded(
        "side hole",
        circle(PlaneInput::OriginYZ, [30.0, 11.0], 3.0),
        on(BodyOp::Cut),
        6.0,
        through,
    );

    // Fillets at the boss roots and round the bottom, chamfers on the boss tops and the rim.
    for (i, pair) in bosses.chunks(2).enumerate() {
        let near: Vec<[f64; 3]> = pair.iter().map(|c| [c[0], c[1], 2.0]).collect();
        let edges = b.edges(body, &near);
        b.add(
            &format!("boss fillet {}", i + 1),
            FeatureBox::new(Fillet::constant(edges, "1 mm")),
        );
    }
    let bottom = b.edges(body, &[[WIDTH / 2.0, 0.0, 0.0]]);
    b.add(
        "bottom fillet",
        FeatureBox::new(Fillet::constant(bottom, "1.5 mm")),
    );
    for (i, pair) in bosses.chunks(2).enumerate() {
        let near: Vec<[f64; 3]> = pair.iter().map(|c| [c[0], c[1], 12.0]).collect();
        let edges = b.edges(body, &near);
        b.add(
            &format!("boss chamfer {}", i + 1),
            FeatureBox::new(Chamfer::equal(edges, "0.5 mm")),
        );
    }
    let rim = b.edges(body, &[[WIDTH / 2.0, 0.0, 20.0]]);
    b.add(
        "rim chamfer",
        FeatureBox::new(Chamfer::equal(rim, "0.5 mm")),
    );

    // A second pattern: pins between the vent holes.
    let pin = b.extruded(
        "pin",
        circle(on_xy(), [34.0, 24.0], 1.5),
        on(BodyOp::Join),
        4.0,
        from(1.0),
    );
    b.add("pin pattern", FeatureBox::new(pattern(pin, 8.0, 5, 3)));

    // A notch in the front rim.
    b.extruded(
        "rim notch",
        rect(on_xy(), [45.0, -1.0, 55.0, 3.0]),
        on(BodyOp::Cut),
        4.0,
        from(18.0),
    );

    Part {
        doc: b.doc,
        features: b.features,
        body,
        width,
    }
}

impl Part {
    /// Edits the first sketch's width dimension and recomputes: the measured operation.
    pub fn set_width(&mut self, width: f64) {
        let first = self.features[0].0;
        let (_, node) = self
            .doc
            .design()
            .timeline()
            .node(first)
            .expect("the first sketch");
        let mut sketch = node
            .feature
            .downcast_ref::<SketchFeature>()
            .expect("a sketch")
            .clone();
        sketch
            .apply(SketchEdit::SetDimension {
                id: self.width,
                expr: inline(&format!("{width} mm")),
            })
            .expect("the dimension");
        self.doc
            .apply(Command::EditFeature {
                id: first,
                feature: FeatureBox::new(sketch),
            })
            .expect("edit");
        self.doc.recompute_blocking(jobs()).expect("recompute");
    }

    /// The features that are not healthy, with their health.
    pub fn unhealthy(&self) -> Vec<(&str, Health)> {
        self.features
            .iter()
            .filter_map(|(id, label)| {
                let h = &self.doc.evaluated().node(*id).expect("evaluated").health;
                (*h != Health::Ok).then(|| (label.as_str(), h.clone()))
            })
            .collect()
    }
}
