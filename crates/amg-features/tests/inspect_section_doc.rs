//! INS-09 `inspect.section` record: create / edit / hide / delete are one undo step each,
//! `.amg` save → load → save is byte-identical, and two visible analyses combine into a
//! box-corner cut (caps on both planes, each clipped by the other).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::sync::Arc;

use amg_core::{Plane, Point3, Tolerance, Transform};
use amg_doc::{CreatedId, Document, ExtKey, SaveOptions, load, save};
use amg_features::inspect::doc::{
    CapColor, HatchPattern, OriginPlane, SECTION_KIND, SectionAnalysis, SectionBase, analyses,
    combine, delete_command, is_kept, next_name,
};
use amg_features::inspect::section::{CapOpts, section_mesh};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, MeshBody, Rgba8};

fn get(doc: &Document, key: &ExtKey) -> Option<SectionAnalysis> {
    doc.design()
        .ext(key)
        .map(|r| SectionAnalysis::from_record(r).unwrap())
}

fn xy10() -> SectionAnalysis {
    let mut a = SectionAnalysis::new("Section 1", SectionBase::Origin(OriginPlane::XY));
    a.offset = 10.0;
    a
}

/// Create, edit, hide and delete: each is exactly one undo entry, and undo / redo restore
/// equal records.
#[test]
fn create_edit_hide_delete_are_one_undo_step_each() {
    let mut doc = Document::new("s");
    let applied = |d: &Document| d.history().stack().applied();
    let n0 = applied(&doc);
    assert_eq!(next_name(doc.design()), "Section 1");

    // Create.
    let a = xy10();
    let out = doc.apply(a.add_command()).unwrap();
    let CreatedId::Ext(key) = out.created[0].clone() else {
        panic!("AddExt creates an ext key")
    };
    assert_eq!(key.kind, SECTION_KIND);
    assert_eq!(applied(&doc), n0 + 1);
    assert_eq!(get(&doc, &key), Some(a.clone()));
    assert_eq!(next_name(doc.design()), "Section 2");

    // Edit (offset, flip, hatch, colour).
    let mut b = a.clone();
    b.offset = -3.25;
    b.flip = true;
    b.hatch.pattern = HatchPattern::Lines45;
    b.cap_color = CapColor::Uniform(Rgba8([10, 200, 30, 255]));
    doc.apply(b.command(key.clone())).unwrap();
    assert_eq!(applied(&doc), n0 + 2);

    // Hide.
    let mut c = b.clone();
    c.visible = false;
    doc.apply(c.command(key.clone())).unwrap();
    assert_eq!(applied(&doc), n0 + 3);

    // Delete.
    doc.apply(delete_command(key.clone())).unwrap();
    assert_eq!(applied(&doc), n0 + 4);
    assert_eq!(get(&doc, &key), None);
    assert!(analyses(doc.design()).is_empty());

    // Each undo steps back exactly one of them.
    assert!(doc.undo());
    assert_eq!(get(&doc, &key), Some(c.clone()));
    assert!(doc.undo());
    assert_eq!(get(&doc, &key), Some(b.clone()));
    assert!(doc.undo());
    assert_eq!(get(&doc, &key), Some(a.clone()));
    assert!(doc.undo());
    assert_eq!(get(&doc, &key), None);
    assert_eq!(applied(&doc), n0);
    // And redo forward again.
    assert!(doc.redo());
    assert_eq!(get(&doc, &key), Some(a));
    assert!(doc.redo());
    assert_eq!(get(&doc, &key), Some(b));
    assert!(doc.redo());
    assert_eq!(get(&doc, &key), Some(c));
    assert!(doc.redo());
    assert_eq!(get(&doc, &key), None);
}

fn temp_dir(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("amg-ins09-{tag}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Two analyses (one with every field set) survive save → load → save: `design.json` and the
/// whole archive are byte-identical, and the records decode to the same values.
#[test]
fn save_load_save_is_byte_identical() {
    let dir = temp_dir("save");
    let mut doc = Document::new("s");
    let a = xy10();
    let mut b = SectionAnalysis::new(
        "Tilted",
        SectionBase::Free(Plane::from_normal(
            Point3::new(1.0 / 3.0, -2.5, 7.125),
            amg_core::Dir3::new_unchecked(amg_core::Vec3::new(0.3, -0.4, 0.866).normalize()),
        )),
    );
    b.offset = 0.1 + 0.2;
    b.tilt = [0.123_456_789, -1.0e-7];
    b.flip = true;
    b.visible = false;
    b.hatch.pattern = HatchPattern::Cross;
    b.hatch.spacing_px = 12.5;
    b.cap_color = CapColor::Uniform(Rgba8([1, 2, 3, 4]));
    b.scope = amg_features::inspect::doc::SectionScope::Bodies(vec![amg_doc::BodyId {
        origin: amg_doc::FeatureId(42),
        slot: 3,
    }]);
    doc.apply(a.add_command()).unwrap();
    doc.apply(b.add_command()).unwrap();
    let json = |d: &amg_doc::Design| serde_json::to_vec(d).unwrap();
    let before = json(doc.design());
    let (p1, p2) = (dir.join("a.amg"), dir.join("b.amg"));
    save(
        &p1,
        &Arc::new(doc.design().clone()),
        &SaveOptions::default(),
    )
    .unwrap();
    let loaded = load(&p1).unwrap();
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    assert_eq!(json(&loaded.design), before, "design.json bytes");
    let back: Vec<SectionAnalysis> = analyses(&loaded.design)
        .into_iter()
        .map(|(_, r)| r.unwrap())
        .collect();
    assert_eq!(back, [a, b]);
    save(
        &p2,
        &Arc::new(loaded.design.clone()),
        &SaveOptions::default(),
    )
    .unwrap();
    assert_eq!(
        std::fs::read(&p1).unwrap(),
        std::fs::read(&p2).unwrap(),
        "the whole archive is byte-identical"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

fn cube() -> MeshBody {
    let c = amg_testkit::shapes::cube(Point3::ZERO, 10.0);
    MeshBody::new(IndexedMesh::new(c.positions, c.triangles).unwrap())
}

/// Two visible analyses (XY at z = 5, YZ at x = 5) on a 10 mm cube give a box-corner cut:
/// the kept region is z ≤ 5 ∧ x ≤ 5, each cap is a 5 × 10 rectangle (area 50) inside the
/// other plane's kept half-space, and hiding one leaves a single full 100 mm² cap.
#[test]
fn two_visible_analyses_combine_into_a_box_corner_cut() {
    let mut a = SectionAnalysis::new("A", SectionBase::Origin(OriginPlane::XY));
    a.offset = 5.0;
    let mut b = SectionAnalysis::new("B", SectionBase::Origin(OriginPlane::YZ));
    b.offset = 5.0;
    let c = combine(&[&a, &b], &|_| None);
    assert_eq!(c.owners, [0, 1]);
    assert!(c.dropped.is_empty());
    let body = cube();
    let s = section_mesh(
        &body,
        &Transform::IDENTITY,
        &c.planes,
        &CapOpts::default(),
        &JobCtx::detached(),
    )
    .unwrap();
    assert_eq!(s.caps.len(), 2);
    for (cap, st) in s.caps.iter().zip(&s.stats) {
        assert!((st.area.get() - 50.0).abs() < 1e-9, "{st:?}");
        assert!(!cap.tris.is_empty());
        let tol = Tolerance {
            linear: 1e-9,
            ..Tolerance::mesh()
        };
        for p in &cap.positions {
            assert!(is_kept(&c.planes, *p, &tol), "{p:?} is cut away");
        }
    }
    // Corner points: kept, removed by one, removed by both.
    let t = Tolerance::mesh();
    assert!(is_kept(&c.planes, Point3::new(2.0, 5.0, 2.0), &t));
    assert!(!is_kept(&c.planes, Point3::new(7.0, 5.0, 2.0), &t));
    assert!(!is_kept(&c.planes, Point3::new(2.0, 5.0, 7.0), &t));
    // Hiding B leaves A alone: one full cap.
    b.visible = false;
    let c1 = combine(&[&a, &b], &|_| None);
    assert_eq!(c1.owners, [0]);
    let s1 = section_mesh(
        &body,
        &Transform::IDENTITY,
        &c1.planes,
        &CapOpts::default(),
        &JobCtx::detached(),
    )
    .unwrap();
    assert!((s1.stats[0].area.get() - 100.0).abs() < 1e-9);
}
