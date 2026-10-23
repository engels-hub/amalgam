//! SK-11 acceptance: DXF/SVG import of the ten fixtures under `tests/fixtures/import/`
//! (bbox within 1e-6 relative, expected profile counts), plus units, scale, layers,
//! endpoint merge and skip behaviour on small inline files.

#![allow(
    clippy::unwrap_used,
    reason = "test helpers; a failure is a test failure"
)]

use std::f64::consts::PI;
use std::path::PathBuf;

use amg_core::{Aabb2, LengthUnit, Plane, Point2};
use amg_sketch::import::{
    ImportError, ImportOptions, ImportReport, UnitSource, import_dxf, import_dxf_file, import_svg,
    import_svg_file,
};
use amg_sketch::{
    ConstraintKind, Curve2, EntityKind, EntityOrigin, ProfileIssue, ProfileOptions, ProfileSet,
    Sketch, SplineDef, detect_profiles,
};

const PX_MM: f64 = 25.4 / 96.0;

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/import")
        .join(name)
}

fn dxf(name: &str, opts: &ImportOptions) -> (Sketch, ImportReport) {
    let mut s = Sketch::new(Plane::XY);
    let r = import_dxf_file(&mut s, &fixture(name), opts).unwrap();
    (s, r)
}

fn svg(name: &str, opts: &ImportOptions) -> (Sketch, ImportReport) {
    let mut s = Sketch::new(Plane::XY);
    let r = import_svg_file(&mut s, &fixture(name), opts).unwrap();
    (s, r)
}

fn profiles(s: &Sketch) -> ProfileSet {
    detect_profiles(s, &ProfileOptions::default())
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

#[track_caller]
fn assert_bbox(r: &ImportReport, x: (f64, f64), y: (f64, f64)) {
    let want = Aabb2::new(Point2::new(x.0, y.0), Point2::new(x.1, y.1));
    for (got, exp, what) in [
        (r.bbox.min.x, want.min.x, "min x"),
        (r.bbox.min.y, want.min.y, "min y"),
        (r.bbox.max.x, want.max.x, "max x"),
        (r.bbox.max.y, want.max.y, "max y"),
    ] {
        assert!(
            rel(got, exp) < 1e-6,
            "{what}: got {got}, expected {exp} (bbox {:?})",
            r.bbox
        );
    }
}

fn kinds(s: &Sketch, r: &ImportReport) -> Vec<&'static str> {
    r.entities
        .iter()
        .map(|&id| s.entity(id).unwrap().kind.name())
        .collect()
}

fn count(kinds: &[&str], name: &str) -> usize {
    kinds.iter().filter(|&&k| k == name).count()
}

fn sorted_areas(p: &ProfileSet) -> Vec<f64> {
    let mut a: Vec<f64> = p.profiles.iter().map(|q| q.fingerprint.area).collect();
    a.sort_by(f64::total_cmp);
    a
}

// ---- the ten fixtures ---------------------------------------------------------------

#[test]
fn dxf_r12_rectangle_with_hole_and_polyline_triangle() {
    let (s, r) = dxf("rect_hole_r12.dxf", &ImportOptions::default());
    assert_eq!(r.unit.source, UnitSource::AssumedMm);
    assert!(rel(r.unit.scale_to_mm, 1.0) < 1e-15);
    assert_bbox(&r, (10.0, 230.0), (20.0, 80.0));
    assert_eq!(r.layers["PART"].len(), 5);
    assert_eq!(r.layers["TRI"].len(), 3);
    let p = profiles(&s);
    // Plate with a hole, the disc in the hole, the triangle.
    assert_eq!(p.profiles.len(), 3, "{:?}", p.issues);
    assert!(p.open_chains.is_empty());
    let mut by_area: Vec<(f64, usize)> = p
        .profiles
        .iter()
        .map(|q| (q.fingerprint.area, q.holes.len()))
        .collect();
    by_area.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert!(rel(by_area[0].0, PI * 100.0) < 1e-9 && by_area[0].1 == 0);
    assert!(rel(by_area[1].0, 375.0) < 1e-9 && by_area[1].1 == 0);
    assert!(rel(by_area[2].0, 6000.0 - PI * 100.0) < 1e-9 && by_area[2].1 == 1);
    assert!(s.entities().all(|(_, e)| {
        e.origin == EntityOrigin::Imported || matches!(e.kind, EntityKind::Point { .. })
    }));
}

#[test]
fn dxf_r2000_bulge_polyline_and_arc() {
    let (s, r) = dxf("slot_lwpolyline_r2000.dxf", &ImportOptions::default());
    assert_eq!(r.unit.source, UnitSource::Declared);
    assert_eq!(r.unit.declared_code, Some(4));
    assert_bbox(&r, (-10.0, 110.0), (0.0, 20.0));
    let k = kinds(&s, &r);
    assert_eq!((count(&k, "line"), count(&k, "arc")), (3, 3), "{k:?}");
    let p = profiles(&s);
    assert_eq!(p.profiles.len(), 2, "{:?}", p.issues);
    let areas = sorted_areas(&p);
    assert!(rel(areas[0], PI * 50.0) < 1e-9);
    assert!(rel(areas[1], 800.0 + PI * 100.0) < 1e-9);
}

#[test]
fn dxf_r2000_inch_ellipse_and_spline() {
    let (s, r) = dxf("ellipse_spline_inch_r2000.dxf", &ImportOptions::default());
    assert_eq!(r.unit.source, UnitSource::Declared);
    assert!(rel(r.unit.scale_to_mm, 25.4) < 1e-15);
    assert_bbox(&r, (0.5 * 25.4, 6.0 * 25.4), (0.0, 1.75 * 25.4));
    let k = kinds(&s, &r);
    assert_eq!(
        (count(&k, "ellipse"), count(&k, "spline"), count(&k, "line")),
        (1, 1, 1),
        "{k:?}"
    );
    let p = profiles(&s);
    assert_eq!(p.profiles.len(), 2, "{:?}", p.issues);
    // Area between the Bezier (4,0) (4.5,1) (5.5,1) (6,0) and its chord: the integral of
    // y dx with y = 3 (1-t) t and x' = 3 (0.5 (1-t)^2 + 2 (1-t) t + 0.5 t^2).
    let n = 200_000;
    let mut cap = 0.0;
    for i in 0..n {
        let t = (f64::from(i) + 0.5) / f64::from(n);
        let u = 1.0 - t;
        let dx = 3.0 * (0.5 * u * u + 2.0 * u * t + 0.5 * t * t);
        cap += 3.0 * u * t * dx / f64::from(n);
    }
    let mm2 = 25.4 * 25.4;
    let areas: Vec<f64> = sorted_areas(&p).iter().map(|a| a / mm2).collect();
    assert!(rel(areas[0], cap) < 1e-6, "{areas:?} vs {cap}");
    assert!(rel(areas[1], PI * 1.5 * 0.75) < 1e-9);
}

#[test]
fn dxf_r2000_layers_gaps_and_mirrored_arc() {
    let (s, r) = dxf("layers_mirrored_cm_r2000.dxf", &ImportOptions::default());
    assert!(rel(r.unit.scale_to_mm, 10.0) < 1e-15);
    assert_bbox(&r, (-90.0, 50.0), (0.0, 30.0));
    assert_eq!(r.layers["OUTLINE"].len(), 6);
    // Four 5e-5 mm corner gaps and the two arc-line joints.
    assert_eq!(r.merged, 6, "{r:?}");
    assert!(r.snapped >= 4);
    assert_eq!(r.constraints.len(), 6);
    for &c in &r.constraints {
        assert!(matches!(
            s.constraint(c).unwrap().kind,
            ConstraintKind::Coincident { .. }
        ));
    }
    // The hidden layer is construction; CONSTR is real by default and splits the rectangle.
    assert!(s.entity(r.layers["HIDDEN"][0]).unwrap().construction);
    assert!(!s.entity(r.layers["CONSTR"][0]).unwrap().construction);
    let p = profiles(&s);
    assert_eq!(p.profiles.len(), 3, "{:?}", p.issues);
    assert!(
        p.issues
            .iter()
            .all(|i| !matches!(i, ProfileIssue::NearGap { .. }))
    );

    // Layers to construction: CONSTR off the profile set, rectangle + half disc remain.
    let opts = ImportOptions {
        construction_layers: vec!["constr".to_owned()],
        ..ImportOptions::default()
    };
    let (s2, r2) = dxf("layers_mirrored_cm_r2000.dxf", &opts);
    assert!(s2.entity(r2.layers["CONSTR"][0]).unwrap().construction);
    let p2 = profiles(&s2);
    assert_eq!(p2.profiles.len(), 2);
    let areas = sorted_areas(&p2);
    assert!(rel(areas[0], PI * 100.0 / 2.0) < 1e-9);
    assert!(rel(areas[1], 50.0 * 30.0) < 1e-6, "{areas:?}");

    // Hidden layers kept real, no construction layer: both guide lines split the rectangle.
    let opts = ImportOptions {
        hidden_layers_construction: false,
        ..ImportOptions::default()
    };
    let (s3, _) = dxf("layers_mirrored_cm_r2000.dxf", &opts);
    assert_eq!(profiles(&s3).profiles.len(), 4);

    // Dropping a layer removes its geometry.
    let opts = ImportOptions {
        skip_layers: vec!["CONSTR".to_owned(), "HIDDEN".to_owned()],
        ..ImportOptions::default()
    };
    let (s4, r4) = dxf("layers_mirrored_cm_r2000.dxf", &opts);
    assert_eq!(r4.entities.len(), 6);
    assert_eq!(profiles(&s4).profiles.len(), 2);
}

#[test]
fn dxf_r2010_unitless_elliptic_arc_and_fit_spline() {
    let (s, r) = dxf(
        "half_ellipse_fit_spline_r2010.dxf",
        &ImportOptions::default(),
    );
    assert_eq!(r.unit.source, UnitSource::AssumedMm);
    assert_eq!(r.unit.declared_code, Some(0));
    assert_bbox(&r, (-10.0, 10.0), (-6.0, 5.0));
    let k = kinds(&s, &r);
    assert_eq!(
        (
            count(&k, "elliptical arc"),
            count(&k, "spline"),
            count(&k, "line")
        ),
        (1, 1, 2),
        "{k:?}"
    );
    assert!(r.entities.iter().any(|&id| matches!(
        s.entity(id).unwrap().kind,
        EntityKind::Spline(SplineDef::Fit { .. })
    )));
    let p = profiles(&s);
    assert_eq!(p.profiles.len(), 2, "{:?}", p.issues);
    assert!(
        p.profiles
            .iter()
            .any(|q| rel(q.fingerprint.area, PI * 10.0 * 5.0 / 2.0) < 1e-9)
    );
}

#[test]
fn svg_group_transform_rectangles_and_circle() {
    let (s, r) = svg("rect_circle_group_transform.svg", &ImportOptions::default());
    assert_eq!(r.unit.source, UnitSource::SvgPixels);
    assert_bbox(&r, (12.5, 70.0), (10.0, 47.5));
    let k = kinds(&s, &r);
    assert_eq!((count(&k, "line"), count(&k, "circle")), (8, 1), "{k:?}");
    let p = profiles(&s);
    // Plate with a hole, the square in the hole, the circle.
    assert_eq!(p.profiles.len(), 3, "{:?}", p.issues);
    assert!(p.profiles.iter().any(|q| q.holes.len() == 1));
    let areas = sorted_areas(&p);
    for (got, want) in areas.iter().zip([25.0, 20.0 * 15.0 - 25.0, PI * 100.0]) {
        assert!(rel(*got, want) < 1e-6, "{areas:?}");
    }
}

#[test]
fn svg_rotated_path_and_ellipse_in_pixels() {
    let (s, r) = svg("rotated_rect_ellipse_px.svg", &ImportOptions::default());
    let th = 30f64.to_radians();
    let (rx, ry) = (20.0, 10.0);
    let hx = (rx * rx * th.cos().powi(2) + ry * ry * th.sin().powi(2)).sqrt();
    assert_bbox(
        &r,
        ((60.0 - hx) * PX_MM, 150.0 * PX_MM),
        (10.0 * PX_MM, 110.0 * PX_MM),
    );
    let k = kinds(&s, &r);
    assert_eq!((count(&k, "line"), count(&k, "ellipse")), (4, 1), "{k:?}");
    let ell = r
        .entities
        .iter()
        .find_map(|&id| match s.entity(id).unwrap().kind {
            EntityKind::Ellipse {
                center,
                major,
                minor_radius,
            } => Some((center, major, minor_radius)),
            _ => None,
        })
        .unwrap();
    let c = s.point(ell.0).unwrap();
    let m = s.point(ell.1).unwrap();
    assert!(rel((m - c).length(), 20.0 * PX_MM) < 1e-5);
    assert!(rel(s.params().get(ell.2), 10.0 * PX_MM) < 1e-5);
    assert!(rel(c.x, 60.0 * PX_MM) < 1e-5 && rel(c.y, 60.0 * PX_MM) < 1e-5);
    let p = profiles(&s);
    assert_eq!(p.profiles.len(), 2, "{:?}", p.issues);
    let want = 100.0 * 50.0 * PX_MM * PX_MM;
    assert!(
        p.profiles
            .iter()
            .any(|q| rel(q.fingerprint.area, want) < 1e-5)
    );
}

#[test]
fn svg_layers_arcs_and_construction_toggle() {
    let (s, r) = svg("layers_arcs_guides.svg", &ImportOptions::default());
    assert_bbox(&r, (10.0, 50.0), (20.0, 40.0));
    assert_eq!(r.layers["outline"].len(), 5, "{:?}", r.layers);
    assert_eq!(r.layers["guides"].len(), 1);
    let k = kinds(&s, &r);
    assert_eq!((count(&k, "line"), count(&k, "arc")), (4, 2), "{k:?}");
    // The guide crosses the D: two profiles; as construction, one.
    assert_eq!(profiles(&s).profiles.len(), 2);
    let opts = ImportOptions {
        construction_layers: vec!["guides".to_owned()],
        ..ImportOptions::default()
    };
    let (s2, r2) = svg("layers_arcs_guides.svg", &opts);
    assert!(s2.entity(r2.layers["guides"][0]).unwrap().construction);
    let p = profiles(&s2);
    assert_eq!(p.profiles.len(), 1);
    assert!(rel(p.profiles[0].fingerprint.area, 600.0 + PI * 50.0) < 1e-6);
}

#[test]
fn svg_matrix_group_scaled_circle_and_open_bezier() {
    let (s, r) = svg("nested_matrix_bezier.svg", &ImportOptions::default());
    let k_mm = 10.0 * PX_MM;
    assert_bbox(&r, (9.0 * k_mm, 36.0 * k_mm), (6.0 * k_mm, 28.0 * k_mm));
    let k = kinds(&s, &r);
    assert_eq!(
        (count(&k, "line"), count(&k, "circle"), count(&k, "spline")),
        (3, 1, 1),
        "{k:?}"
    );
    let p = profiles(&s);
    assert_eq!(p.profiles.len(), 2, "{:?}", p.issues);
    assert_eq!(p.open_chains.len(), 1);
    let areas = sorted_areas(&p);
    assert!(rel(areas[0], 0.5 * 10.0 * 8.0 * k_mm * k_mm) < 1e-5);
    assert!(rel(areas[1], PI * 36.0 * k_mm * k_mm) < 1e-5);
}

#[test]
fn svg_rounded_rect_in_inches() {
    let (s, r) = svg("rounded_rect_inch.svg", &ImportOptions::default());
    assert_bbox(&r, (2.0 * 2.54, 22.0 * 2.54), (6.0 * 2.54, 18.0 * 2.54));
    let k = kinds(&s, &r);
    assert_eq!((count(&k, "line"), count(&k, "arc")), (4, 4), "{k:?}");
    let p = profiles(&s);
    assert_eq!(p.profiles.len(), 1, "{:?}", p.issues);
    let want = (20.0 * 12.0 - (4.0 - PI) * 9.0) * 2.54 * 2.54;
    assert!(rel(p.profiles[0].fingerprint.area, want) < 1e-5);
}

// ---- inline behaviour ---------------------------------------------------------------

fn dxf_text(version: &str, insunits: Option<i16>, entities: &str) -> Vec<u8> {
    let mut t = format!("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\n{version}\n");
    if let Some(u) = insunits {
        t += &format!("9\n$INSUNITS\n70\n{u}\n");
    }
    t += "0\nENDSEC\n0\nSECTION\n2\nENTITIES\n";
    t += entities;
    t += "0\nENDSEC\n0\nEOF\n";
    t.into_bytes()
}

const LINE: &str = "0\nLINE\n8\n0\n10\n0\n20\n0\n30\n0\n11\n100\n21\n0\n31\n0\n";

fn first_length(s: &Sketch, r: &ImportReport) -> f64 {
    s.curve(r.entities[0]).unwrap().length()
}

#[test]
fn units_override_scale_and_unitless_default() {
    let bytes = dxf_text("AC1009", None, LINE);
    let go = |bytes: &[u8], opts: &ImportOptions| {
        let mut s = Sketch::new(Plane::XY);
        let r = import_dxf(&mut s, bytes, opts).unwrap();
        (first_length(&s, &r), r.unit)
    };
    let (len, unit) = go(&bytes, &ImportOptions::default());
    assert!(rel(len, 100.0) < 1e-12 && unit.source == UnitSource::AssumedMm);
    let (len, unit) = go(
        &bytes,
        &ImportOptions {
            unit: Some(LengthUnit::Inch),
            ..ImportOptions::default()
        },
    );
    assert!(rel(len, 2540.0) < 1e-12 && unit.source == UnitSource::Override);
    let (len, _) = go(
        &bytes,
        &ImportOptions {
            scale: 0.5,
            ..ImportOptions::default()
        },
    );
    assert!(rel(len, 50.0) < 1e-12);
    // A declared unit applies; an override wins over it.
    let bytes = dxf_text("AC1015", Some(6), LINE);
    let (len, _) = go(&bytes, &ImportOptions::default());
    assert!(rel(len, 100_000.0) < 1e-12);
    let (len, _) = go(
        &bytes,
        &ImportOptions {
            unit: Some(LengthUnit::Centimetre),
            ..ImportOptions::default()
        },
    );
    assert!(rel(len, 1000.0) < 1e-12);
}

#[test]
fn unsupported_entities_are_counted_and_tilted_planes_skipped() {
    let text = "0\nTEXT\n8\n0\n10\n0\n20\n0\n30\n0\n40\n2\n1\nhi\n\
                0\nCIRCLE\n8\n0\n10\n0\n20\n0\n30\n0\n40\n5\n210\n1\n220\n0\n230\n0\n\
                0\nLINE\n8\n0\n10\n0\n20\n0\n30\n0\n11\n0\n21\n0\n31\n0\n";
    let mut s = Sketch::new(Plane::XY);
    let bytes = dxf_text("AC1015", None, text);
    let r = import_dxf(&mut s, &bytes, &ImportOptions::default()).unwrap();
    assert!(r.entities.is_empty() && r.bbox.is_empty());
    assert_eq!(r.skipped["TEXT"], 1);
    assert_eq!(r.skipped["CIRCLE (not in the XY plane)"], 1);
    assert_eq!(r.skipped["degenerate"], 1);
}

#[test]
fn gaps_merge_only_within_tolerance() {
    // Two lines with a 5e-3 mm gap: kept apart by default (warn only, QUESTIONS #17),
    // healed when the tolerance is raised.
    let text = "0\nLINE\n8\n0\n10\n0\n20\n0\n30\n0\n11\n10\n21\n0\n31\n0\n\
                0\nLINE\n8\n0\n10\n10.005\n20\n0\n30\n0\n11\n20\n21\n0\n31\n0\n";
    let bytes = dxf_text("AC1015", Some(4), text);
    let mut s = Sketch::new(Plane::XY);
    let r = import_dxf(&mut s, &bytes, &ImportOptions::default()).unwrap();
    assert_eq!((r.merged, r.snapped), (0, 0));
    let mut s = Sketch::new(Plane::XY);
    let opts = ImportOptions {
        merge_tol: 1e-2,
        ..ImportOptions::default()
    };
    let r = import_dxf(&mut s, &bytes, &opts).unwrap();
    assert_eq!((r.merged, r.snapped), (1, 1));
    let (Curve2::Line(a), Curve2::Line(b)) = (
        s.curve(r.entities[0]).unwrap(),
        s.curve(r.entities[1]).unwrap(),
    ) else {
        panic!("lines expected");
    };
    assert_eq!(a.b, b.a);
}

#[test]
fn unclamped_spline_is_sampled() {
    // Uniform quadratic B-spline, one span: C(0.5) = (P0 + 6 P1 + P2) / 8 = (10, 15).
    let text = "0\nSPLINE\n8\n0\n210\n0\n220\n0\n230\n1\n70\n8\n71\n2\n72\n6\n73\n3\n74\n0\n\
                40\n0\n40\n1\n40\n2\n40\n3\n40\n4\n40\n5\n\
                10\n0\n20\n0\n30\n0\n10\n10\n20\n20\n30\n0\n10\n20\n20\n0\n30\n0\n";
    let mut s = Sketch::new(Plane::XY);
    let bytes = dxf_text("AC1015", Some(4), text);
    let r = import_dxf(&mut s, &bytes, &ImportOptions::default()).unwrap();
    assert_eq!(r.entities.len(), 1);
    assert!(r.warnings.iter().any(|w| w.contains("approximated")));
    let c = s.curve(r.entities[0]).unwrap();
    // The domain runs from (P0 + P1) / 2 = (5, 10) to (P1 + P2) / 2 = (15, 10).
    for (want, tol) in [
        ((10.0, 15.0), 1e-4),
        ((5.0, 10.0), 1e-9),
        ((15.0, 10.0), 1e-9),
    ] {
        let w = Point2::new(want.0, want.1);
        assert!(c.closest(w).1.distance(w) < tol, "{want:?}");
    }
}

#[test]
fn bad_input_is_an_error() {
    let mut s = Sketch::new(Plane::XY);
    assert!(matches!(
        import_svg(&mut s, "not svg", &ImportOptions::default()),
        Err(ImportError::Svg(_))
    ));
    assert!(matches!(
        import_dxf(
            &mut s,
            b"0\nSECTION\n2\nENTITIES\n0\nLINE\n10\nabc",
            &ImportOptions::default()
        ),
        Err(ImportError::Dxf(_))
    ));
    for opts in [
        ImportOptions {
            scale: 0.0,
            ..ImportOptions::default()
        },
        ImportOptions {
            merge_tol: -1.0,
            ..ImportOptions::default()
        },
        ImportOptions {
            svg_dpi: f64::NAN,
            ..ImportOptions::default()
        },
    ] {
        assert!(matches!(
            import_svg(&mut s, "<svg xmlns=\"http://www.w3.org/2000/svg\"/>", &opts),
            Err(ImportError::Options(_))
        ));
    }
    assert!(matches!(
        import_dxf_file(&mut s, &fixture("missing.dxf"), &ImportOptions::default()),
        Err(ImportError::Io { .. })
    ));
    assert_eq!(s.entities().count(), 0);
}

#[test]
fn svg_override_unit() {
    let text = "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"100\" height=\"100\">\
                <path d=\"M0 0 L50 0\" stroke=\"black\"/></svg>";
    let mut s = Sketch::new(Plane::XY);
    let r = import_svg(&mut s, text, &ImportOptions::default()).unwrap();
    assert!(rel(first_length(&s, &r), 50.0 * PX_MM) < 1e-6);
    let mut s = Sketch::new(Plane::XY);
    let opts = ImportOptions {
        unit: Some(LengthUnit::Millimetre),
        ..ImportOptions::default()
    };
    let r = import_svg(&mut s, text, &opts).unwrap();
    assert!(rel(first_length(&s, &r), 50.0) < 1e-6);
    assert_eq!(r.unit.source, UnitSource::Override);
}
