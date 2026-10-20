//! SK-01 serde: fixture sketches round-trip bit-identically, the format is pinned by checked-in
//! JSON, and invalid files are rejected on load.
//!
//! Re-bless the fixtures after an intentional format change with
//! `AMG_BLESS=1 cargo nextest run -p amg-sketch`.

#![allow(
    clippy::unwrap_used,
    reason = "test helpers; a failure is a test failure"
)]

use std::path::PathBuf;

use amg_core::{Dir3, Plane, Point2, Point3, Tolerance, Vec3};
use amg_sketch::{
    AngleBranch, ConstraintKind, Dimension, DimensionKind, End, EntityKind, EntityOrigin,
    ExternalLinkId, HvTarget, ParamFlags, Side, Sketch, TangentKind,
};
use proptest::prelude::*;

fn dim(kind: DimensionKind, value: f64, driving: bool) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving,
        label_pos: Point2::new(value * 0.5, -1.25),
    })
}

fn children(s: &Sketch, id: amg_sketch::EntityId) -> Vec<amg_sketch::EntityId> {
    s.entity(id).unwrap().kind.children().to_vec()
}

/// 1. Rectangle on XY: four lines sharing corners, H/V, fixed corner, two driving and one
///    driven dimension.
fn rectangle() -> Sketch {
    let mut s = Sketch::new(Plane::XY);
    let c = [
        s.add_point(Point2::new(0.0, 0.0)).unwrap(),
        s.add_point(Point2::new(40.0, 0.0)).unwrap(),
        s.add_point(Point2::new(40.0, 25.0)).unwrap(),
        s.add_point(Point2::new(0.0, 25.0)).unwrap(),
    ];
    let l: Vec<_> = (0..4)
        .map(|i| s.add_line_between(c[i], c[(i + 1) % 4]).unwrap())
        .collect();
    use ConstraintKind as C;
    s.add_constraint(C::Horizontal(HvTarget::Line(l[0])))
        .unwrap();
    s.add_constraint(C::Horizontal(HvTarget::Line(l[2])))
        .unwrap();
    s.add_constraint(C::Vertical(HvTarget::Line(l[1]))).unwrap();
    s.add_constraint(C::Vertical(HvTarget::Line(l[3]))).unwrap();
    s.add_constraint(C::Fix { entity: c[0] }).unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: l[0] }, 40.0, true))
        .unwrap();
    s.add_constraint(dim(DimensionKind::Length { line: l[1] }, 25.0, true))
        .unwrap();
    s.add_constraint(dim(
        DimensionKind::Distance {
            a: c[0],
            b: c[2],
            side: Side::Positive,
        },
        (40.0_f64.powi(2) + 25.0_f64.powi(2)).sqrt(),
        false,
    ))
    .unwrap();
    s
}

/// 2. Slot on an offset, tilted plane: two arcs and two lines, tangent at endpoints, equal
///    radii, a concentric hole, awkward float values.
fn slot() -> Sketch {
    let n = Dir3::new(Vec3::new(1.0, -2.0, 3.0), &Tolerance::DEFAULT).unwrap();
    let plane = Plane::from_normal(Point3::new(0.1 + 0.2, -1.0 / 3.0, 1e-300), n);
    let mut s = Sketch::new(plane);
    let r = std::f64::consts::PI;
    let a1 = s
        .add_arc(
            Point2::new(0.0, 0.0),
            Point2::new(0.0, r),
            Point2::new(0.0, -r),
        )
        .unwrap();
    let a2 = s
        .add_arc(
            Point2::new(17.0, 0.0),
            Point2::new(17.0, -r),
            Point2::new(17.0, r),
        )
        .unwrap();
    let (c1, c2) = (children(&s, a1), children(&s, a2));
    let top = s.add_line_between(c2[2], c1[1]).unwrap();
    let bottom = s.add_line_between(c1[2], c2[1]).unwrap();
    let hole = s.add_circle(Point2::new(17.0, 0.0), 1.0 / 7.0).unwrap();
    use ConstraintKind as C;
    let tan = |a, a_end, b, b_end| C::Tangent {
        a,
        b,
        kind: TangentKind::AtEndpoint {
            a_end,
            b_end,
            reversed: false,
        },
    };
    // `top` runs from a2's end to a1's start, `bottom` from a1's end to a2's start.
    s.add_constraint(tan(a1, End::Start, top, End::End))
        .unwrap();
    s.add_constraint(tan(a1, End::End, bottom, End::Start))
        .unwrap();
    s.add_constraint(tan(a2, End::End, top, End::Start))
        .unwrap();
    s.add_constraint(tan(a2, End::Start, bottom, End::End))
        .unwrap();
    s.add_constraint(C::Equal { a: a1, b: a2 }).unwrap();
    s.add_constraint(C::Concentric { a: hole, b: a2 }).unwrap();
    s.add_constraint(C::Horizontal(HvTarget::Points(c1[0], c2[0])))
        .unwrap();
    s.add_constraint(dim(DimensionKind::Radius { curve: a1 }, r, true))
        .unwrap();
    s.add_constraint(dim(
        DimensionKind::Diameter { curve: hole },
        2.0 / 7.0,
        true,
    ))
    .unwrap();
    s.add_constraint(dim(
        DimensionKind::HorizontalDistance {
            a: c1[0],
            b: c2[0],
            side: Side::Positive,
        },
        17.0,
        true,
    ))
    .unwrap();
    s.add_constraint(dim(DimensionKind::ArcLength { arc: a1 }, r * r, false))
        .unwrap();
    s
}

/// 3. Everything else: every remaining constraint/dimension variant, construction, projected
///    (locked) and imported entities, removed entities (gaps in ids), an unowned param slot,
///    and special floats (−0.0, subnormal, huge).
fn kitchen_sink() -> Sketch {
    let mut s = Sketch::new(Plane::XZ);
    let p = s.add_point(Point2::new(-0.0, 5e-324)).unwrap();
    let q = s.add_point(Point2::new(1e300, -2.5)).unwrap();
    let l1 = s
        .add_line(Point2::new(0.0, 0.0), Point2::new(10.0, 0.1))
        .unwrap();
    let l2 = s
        .add_line(Point2::new(0.0, 4.0), Point2::new(10.0, 6.0))
        .unwrap();
    let gone = s
        .add_line(Point2::new(9.0, 9.0), Point2::new(8.0, 8.0))
        .unwrap();
    let c = s.add_circle(Point2::new(30.0, 0.0), 3.0).unwrap();
    let a = s
        .add_arc(
            Point2::new(0.0, 20.0),
            Point2::new(4.0, 20.0),
            Point2::new(-4.0, 20.0),
        )
        .unwrap();
    let link = ExternalLinkId(7);
    let px = s.alloc_param(1.5, ParamFlags::default()).unwrap();
    let py = s.alloc_param(-2.5, ParamFlags::default()).unwrap();
    let proj = s
        .add_entity(
            EntityKind::Point { x: px, y: py },
            false,
            EntityOrigin::Projected(link),
        )
        .unwrap();
    let ix = s.alloc_param(3.0, ParamFlags::default()).unwrap();
    let iy = s.alloc_param(4.0, ParamFlags::default()).unwrap();
    let imp = s
        .add_entity(
            EntityKind::Point { x: ix, y: iy },
            true,
            EntityOrigin::Imported,
        )
        .unwrap();
    let axis = s.add_line_between(proj, imp).unwrap();
    s.set_construction(axis, true).unwrap();
    let _unowned = s
        .alloc_param(0.1 + 0.2, ParamFlags { locked: true })
        .unwrap();
    s.remove_entities(&[gone]).unwrap();

    use ConstraintKind as C;
    use DimensionKind as D;
    let br = AngleBranch {
        reverse_a: true,
        reverse_b: false,
    };
    for k in [
        C::Coincident { a: p, b: q },
        C::PointOnCurve { point: p, curve: c },
        C::Midpoint { point: q, line: l1 },
        C::Parallel { a: l1, b: l2 },
        C::Perpendicular { a: l1, b: axis },
        C::Collinear { a: l2, b: l1 },
        C::Tangent {
            a: l1,
            b: c,
            kind: TangentKind::External,
        },
        C::Tangent {
            a: c,
            b: a,
            kind: TangentKind::Internal,
        },
        C::Symmetric { a: p, b: q, axis },
        C::Vertical(HvTarget::Line(axis)),
        C::Fix { entity: proj },
        dim(
            D::Distance {
                a: p,
                b: l1,
                side: Side::Negative,
            },
            0.0,
            true,
        ),
        dim(
            D::VerticalDistance {
                a: p,
                b: q,
                side: Side::Negative,
            },
            2.5,
            true,
        ),
        dim(
            D::Angle {
                a: l1,
                b: l2,
                branch: br,
            },
            -std::f64::consts::FRAC_PI_6,
            true,
        ),
        dim(D::Radius { curve: c }, 3.0, false),
    ] {
        s.add_constraint(k).unwrap();
    }
    let removed = s.add_constraint(C::Equal { a: l1, b: l2 }).unwrap();
    s.remove_constraint(removed).unwrap();
    s
}

fn fixtures() -> [(&'static str, Sketch); 3] {
    [
        ("rectangle", rectangle()),
        ("slot", slot()),
        ("kitchen_sink", kitchen_sink()),
    ]
}

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/serde")
        .join(format!("{name}.json"))
}

/// Bit-level equality of everything a sketch stores.
fn assert_bit_identical(a: &Sketch, b: &Sketch) {
    let bits = |s: &Sketch| {
        s.params()
            .as_slice()
            .iter()
            .map(|v| v.to_bits())
            .collect::<Vec<_>>()
    };
    assert_eq!(bits(a), bits(b));
    let flags = |s: &Sketch| {
        (0..s.params().len())
            .map(|i| s.params().flags(amg_sketch::ParamId(i as u32)))
            .collect::<Vec<_>>()
    };
    assert_eq!(flags(a), flags(b));
    let plane = |s: &Sketch| {
        let p = s.plane();
        [p.origin, p.x.get(), p.y.get(), p.normal.get()].map(|v| v.to_array().map(f64::to_bits))
    };
    assert_eq!(plane(a), plane(b));
    assert_eq!(
        a.entities().collect::<Vec<_>>(),
        b.entities().collect::<Vec<_>>()
    );
    let cons = |s: &Sketch| {
        s.constraints()
            .map(|(id, c)| {
                let d = c
                    .kind
                    .as_dimension()
                    .map(|d| (d.value.to_bits(), d.label_pos.to_array().map(f64::to_bits)));
                (id, c.clone(), d)
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(cons(a), cons(b));
}

#[test]
fn fixtures_round_trip_bit_identically() {
    let bless = std::env::var_os("AMG_BLESS").is_some();
    for (name, sketch) in fixtures() {
        let json = serde_json::to_string_pretty(&sketch).unwrap() + "\n";
        let path = fixture_path(name);
        if bless || !path.exists() {
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(&path, &json).unwrap();
        }
        let on_disk = std::fs::read_to_string(&path)
            .unwrap()
            .replace("\r\n", "\n");
        assert_eq!(
            on_disk, json,
            "{name}: serialised form changed (AMG_BLESS=1 to accept)"
        );

        let loaded: Sketch = serde_json::from_str(&on_disk).unwrap();
        assert_bit_identical(&sketch, &loaded);
        let again = serde_json::to_string_pretty(&loaded).unwrap() + "\n";
        assert_eq!(again, on_disk, "{name}: re-serialisation differs");

        // Loaded sketches keep allocating fresh ids and get fresh revisions.
        assert_ne!(loaded.structure_rev(), sketch.structure_rev());
        let mut loaded = loaded;
        let max = sketch.entities().map(|(id, _)| id).max().unwrap();
        assert!(loaded.add_point(Point2::ZERO).unwrap() > max);
    }
}

#[test]
fn kitchen_sink_keeps_flags_and_origins() {
    let s = kitchen_sink();
    let projected: Vec<_> = s
        .entities()
        .filter(|(_, e)| matches!(e.origin, EntityOrigin::Projected(_)))
        .collect();
    assert_eq!(projected.len(), 1);
    for p in projected[0].1.kind.own_params() {
        assert!(s.params().flags(p).locked);
    }
    assert!(
        s.entities()
            .any(|(_, e)| e.origin == EntityOrigin::Imported && e.construction)
    );
    // The removed line's three entities leave a gap in the ids.
    let ids: Vec<u32> = s.entities().map(|(id, _)| id.0).collect();
    assert!(ids.windows(2).any(|w| w[1] - w[0] > 1));
}

/// Mutates the rectangle fixture's JSON and expects the load to fail.
fn rejects(edit: impl Fn(&mut serde_json::Value)) -> String {
    let mut v = serde_json::to_value(rectangle()).unwrap();
    edit(&mut v);
    serde_json::from_value::<Sketch>(v).unwrap_err().to_string()
}

#[test]
fn invalid_values_are_rejected_on_load() {
    let mut s = Sketch::new(Plane::XY);
    let c = s.add_circle(Point2::ZERO, 2.0).unwrap(); // params 0, 1 (centre), 2 (radius)
    let a = s
        .add_arc(Point2::ZERO, Point2::new(3.0, 0.0), Point2::new(0.0, 3.0))
        .unwrap(); // params 3..=8
    let gone = s.add_point(Point2::new(7.0, 7.0)).unwrap(); // params 9, 10
    s.remove_entities(&[gone]).unwrap();
    let load = |edit: &dyn Fn(&mut serde_json::Value)| {
        let mut v = serde_json::to_value(&s).unwrap();
        edit(&mut v);
        serde_json::from_value::<Sketch>(v).map_err(|e| e.to_string())
    };
    assert!(load(&|_| {}).is_ok());
    let e = load(&|v| v["params"]["values"][2] = (-3.0).into()).unwrap_err();
    assert!(e.contains("circle radius"), "{e}");
    let e = load(&|v| v["params"]["values"][2] = 0.0.into()).unwrap_err();
    assert!(e.contains("circle radius"), "{e}");
    // Arc end moved off the circle through its start.
    let e = load(&|v| v["params"]["values"][8] = 3.5.into()).unwrap_err();
    assert!(e.contains("different distances"), "{e}");
    // A tiny inconsistency left by a solve is accepted.
    assert!(load(&|v| v["params"]["values"][8] = (3.0 + 1e-9).into()).is_ok());
    // Non-finite values cannot even be written as JSON numbers.
    assert!(load(&|v| v["params"]["values"][0] = serde_json::Value::Null).is_err());
    // A retired slot that an entity owns, and one out of range.
    let e = load(&|v| v["retired"] = serde_json::json!([0])).unwrap_err();
    assert!(e.contains("owned twice"), "{e}");
    let e = load(&|v| v["retired"] = serde_json::json!([500])).unwrap_err();
    assert!(e.contains("unknown parameter"), "{e}");
    let _ = (c, a);
}

#[test]
fn invalid_files_are_rejected() {
    // Dangling entity reference in a constraint.
    let e = rejects(|v| v["constraints"]["0"]["kind"]["Horizontal"]["Line"] = 999.into());
    assert!(e.contains("unknown entity"), "{e}");
    // Wrong entity kind: a line used as a line end point.
    let e = rejects(|v| v["entities"]["4"]["kind"]["Line"]["start"] = 5.into());
    assert!(e.contains("constraint needs point"), "{e}");
    // Parameter out of range.
    let e = rejects(|v| v["entities"]["0"]["kind"]["Point"]["x"] = 10_000.into());
    assert!(e.contains("unknown parameter"), "{e}");
    // Two points sharing a parameter.
    let e = rejects(|v| v["entities"]["1"]["kind"]["Point"]["x"] = 0.into());
    assert!(e.contains("owned twice"), "{e}");
    // Id counter behind existing ids (would reuse ids).
    let e = rejects(|v| v["next_entity"] = 2.into());
    assert!(e.contains("id counter"), "{e}");
    let e = rejects(|v| v["next_constraint"] = 0.into());
    assert!(e.contains("id counter"), "{e}");
    // Values and flags out of sync.
    let e = rejects(|v| {
        v["params"]["flags"].as_array_mut().unwrap().pop();
    });
    assert!(e.contains("flags"), "{e}");
}

proptest! {
    /// Any finite parameter value (subnormals, −0.0, extremes) survives a JSON round trip
    /// bit for bit.
    #[test]
    fn params_round_trip_bitwise(
        xs in proptest::collection::vec(proptest::num::f64::NORMAL | proptest::num::f64::SUBNORMAL
            | proptest::num::f64::ZERO | proptest::num::f64::NEGATIVE | proptest::num::f64::POSITIVE, 2..40),
    ) {
        let mut s = Sketch::new(Plane::XY);
        for pair in xs.chunks_exact(2) {
            s.add_point(Point2::new(pair[0], pair[1])).unwrap();
        }
        let loaded: Sketch = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        let bits = |s: &Sketch| s.params().as_slice().iter().map(|v| v.to_bits()).collect::<Vec<_>>();
        prop_assert_eq!(bits(&s), bits(&loaded));
    }
}
