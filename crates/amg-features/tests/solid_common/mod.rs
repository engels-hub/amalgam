//! Shared helpers of the SOL-01 tests: documents with sketches and extrudes, metrics, goldens.

#![allow(dead_code, reason = "shared test module")]

#[cfg(feature = "occt")]
pub mod failing;

use std::sync::OnceLock;

use amg_core::Point2;
use amg_doc::stub::ProfileKey;
use amg_doc::{
    BodyId, Command, CommandOutcome, CreatedId, Document, EntityRef, FeatureBox, FeatureId, Health,
    SketchId,
};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{BodyOp, Extrude, OpTarget};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_sketch::{EntityId, build};

pub fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

pub fn created_feature(o: &CommandOutcome) -> FeatureId {
    o.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .expect("a feature was created")
}

/// Adds `f` to the root component and recomputes.
pub fn add(doc: &mut Document, f: FeatureBox) -> FeatureId {
    let root = doc.design().root();
    let o = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context: root,
            name: None,
        })
        .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    created_feature(&o)
}

/// Replaces feature `id` and recomputes.
pub fn edit(doc: &mut Document, id: FeatureId, f: FeatureBox) {
    doc.apply(Command::EditFeature { id, feature: f }).unwrap();
    doc.recompute_blocking(jobs()).unwrap();
}

pub fn health(doc: &Document, id: FeatureId) -> Health {
    doc.evaluated().node(id).unwrap().health.clone()
}

/// A sketch on XY with axis-aligned rectangles `[x0, y0, x1, y1]`.
pub fn rect_sketch(rects: &[[f64; 4]]) -> SketchFeature {
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    for r in rects {
        f.edit_sketch(|s| {
            build::rectangle_2pt(s, Point2::new(r[0], r[1]), Point2::new(r[2], r[3]))
        })
        .unwrap();
    }
    f
}

/// A closed polygon sketch on XY: one point entity per vertex, lines between neighbours.
/// Returns the feature, the point ids and the line ids (line `i` runs from point `i` to `i + 1`).
pub fn polygon_sketch(points: &[[f64; 2]]) -> (SketchFeature, Vec<EntityId>, Vec<EntityId>) {
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    let (pts, lines) = f
        .edit_sketch(|s| {
            let pts: Vec<EntityId> = points
                .iter()
                .map(|p| s.add_point(Point2::new(p[0], p[1])))
                .collect::<Result<_, _>>()?;
            let n = pts.len();
            let lines = (0..n)
                .map(|i| s.add_line_between(pts[i], pts[(i + 1) % n]))
                .collect::<Result<Vec<_>, _>>()?;
            Ok((pts, lines))
        })
        .unwrap();
    (f, pts, lines)
}

/// Every profile of a sketch as `EntityRef::Profile`, in profile-set order.
pub fn profiles(doc: &Document, sketch: FeatureId) -> Vec<EntityRef> {
    let g = doc
        .evaluated()
        .sketch(SketchId(sketch))
        .expect("the sketch is evaluated");
    g.profiles
        .profiles
        .iter()
        .map(|p| EntityRef::Profile {
            sketch: SketchId(sketch),
            profile: ProfileKey(p.id.0),
            fingerprint: Some(amg_doc::naming::ProfileFp(p.fingerprint.clone())),
        })
        .collect()
}

/// A sketch of `rects` plus a one-sided extrude of all its profiles by `h`.
pub fn extrude_rects(
    doc: &mut Document,
    rects: &[[f64; 4]],
    h: f64,
    op: BodyOp,
    participants: Vec<BodyId>,
) -> (FeatureId, FeatureId) {
    let s = add(doc, FeatureBox::new(rect_sketch(rects)));
    let ext = Extrude::distance(
        profiles(doc, s),
        &format!("{h} mm"),
        OpTarget { op, participants },
    );
    let e = add(doc, FeatureBox::new(ext));
    (s, e)
}

pub fn body0(f: FeatureId) -> BodyId {
    BodyId { origin: f, slot: 0 }
}

/// The B-rep of a live body.
#[cfg(feature = "occt")]
pub fn brep(doc: &Document, b: BodyId) -> amg_brep::Shape {
    match &doc.evaluated().body(b).expect("body exists").geom {
        amg_doc::BodyGeom::Solid(s) => s.brep().expect("a B-rep").clone(),
        other => panic!("{b:?} is not a solid: {other:?}"),
    }
}

#[cfg(feature = "occt")]
pub fn kernel() -> &'static amg_brep::OcctKernel {
    amg_doc::naming::default_kernel()
}

#[cfg(feature = "occt")]
pub fn volume(s: &amg_brep::Shape) -> f64 {
    use amg_brep::BrepKernel;
    kernel().mass_props(s).unwrap().volume
}

#[cfg(feature = "occt")]
pub fn faces(s: &amg_brep::Shape) -> u32 {
    s.count(amg_brep::SubKind::Face)
}

#[track_caller]
pub fn assert_rel(got: f64, want: f64, rel: f64) {
    assert!(
        (got - want).abs() <= rel * want.abs().max(1.0),
        "got {got}, want {want} (rel {rel})"
    );
}

// ---- goldens (tests/golden/solid/sol01_*.toml) ------------------------------------------------

#[cfg(feature = "occt")]
#[allow(unused_imports, reason = "each test binary uses one of them")]
pub use golden::{check_golden, check_golden_as};

#[cfg(feature = "occt")]
mod golden {
    use amg_brep::{BrepKernel, SubKind};

    #[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    struct Golden {
        tolerances: Tolerances,
        metrics: Metrics,
    }

    #[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    struct Tolerances {
        rel: f64,
    }

    #[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    struct Metrics {
        volume: f64,
        area: f64,
        faces: u32,
        edges: u32,
        vertices: u32,
        bbox_min: [f64; 3],
        bbox_max: [f64; 3],
    }

    fn metrics(s: &amg_brep::Shape) -> Metrics {
        let m = super::kernel().mass_props(s).unwrap();
        let b = s.bbox();
        Metrics {
            volume: m.volume,
            area: m.area,
            faces: s.count(SubKind::Face),
            edges: s.count(SubKind::Edge),
            vertices: s.count(SubKind::Vertex),
            bbox_min: b.min.to_array(),
            bbox_max: b.max.to_array(),
        }
    }

    /// Compares `s` with `tests/golden/solid/sol01_<case>.toml`; `AMG_BLESS=1` (never under CI)
    /// writes it instead.
    #[track_caller]
    pub fn check_golden(case: &str, s: &amg_brep::Shape) {
        check_golden_as("sol01", 1e-9, case, s);
    }

    /// Compares `s` with `tests/golden/solid/<wp>_<case>.toml` (`wp` lower case, e.g. `sol05`);
    /// `AMG_BLESS=1` (never under CI) writes it instead, with tolerance `rel`.
    #[track_caller]
    pub fn check_golden_as(wp: &str, rel: f64, case: &str, s: &amg_brep::Shape) {
        let got = metrics(s);
        let path = amg_testkit::golden_dir()
            .join("solid")
            .join(format!("{wp}_{case}.toml"));
        if std::env::var("AMG_BLESS").is_ok_and(|v| v == "1") {
            assert!(
                std::env::var_os("CI").is_none_or(|v| v.is_empty()),
                "blessing is refused under CI"
            );
            let g = Golden {
                tolerances: Tolerances { rel },
                metrics: got,
            };
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            let wp_id = wp.to_uppercase().replacen("SOL", "SOL-", 1);
            let text = format!(
                "# B-rep metrics of solid/{wp}_{case} (S04 {wp_id}). Written by `AMG_BLESS=1`.\n\n{}",
                toml::to_string(&g).unwrap()
            );
            std::fs::write(&path, text).unwrap();
            return;
        }
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("{}: {e}; bless it with AMG_BLESS=1", path.display()));
        let g: Golden = toml::from_str(&text).unwrap();
        let w = &g.metrics;
        assert_eq!(
            (got.faces, got.edges, got.vertices),
            (w.faces, w.edges, w.vertices),
            "{case}: topology"
        );
        let rel = g.tolerances.rel;
        super::assert_rel(got.volume, w.volume, rel);
        super::assert_rel(got.area, w.area, rel);
        for i in 0..3 {
            assert!(
                (got.bbox_min[i] - w.bbox_min[i]).abs() <= 1e-6,
                "{case}: bbox"
            );
            assert!(
                (got.bbox_max[i] - w.bbox_max[i]).abs() <= 1e-6,
                "{case}: bbox"
            );
        }
    }
}
