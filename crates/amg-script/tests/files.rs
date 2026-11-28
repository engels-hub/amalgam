//! SCR-07 Done-when 2 (export), 3 (convert), 4 (properties) and 5 (report schemas).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    reason = "test helpers; a failure is a test failure"
)]

mod common;

use std::path::Path;
use std::sync::Arc;

use amg_core::units::LengthUnit;
use amg_core::{Point3, Transform, Vec3};
use amg_doc::testing::TestBox;
use amg_doc::{BodyProps, Command, CreatedId, Document, FeatureBox};
use amg_io::{CoordPrecision, Format, IoError, Sink, StlWriteOptions};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::IndexedMesh;
use amg_script::ScriptError;
use amg_script::host::ScriptDoc;
use amg_script::ops::{
    BodyFilter, ConvertRequest, ExportRequest, convert, export, properties, recompute,
};

fn open(doc: &Document, dir: &Path, name: &str) -> ScriptDoc {
    let path = common::save(doc, dir, name);
    ScriptDoc::open(&path, common::jobs()).unwrap()
}

#[test]
fn export_reimports_with_the_same_triangles_and_volume() {
    let dir = tempfile::tempdir().unwrap();
    let mut doc = open(&common::mesh_only(), dir.path(), "export");
    let cancel = CancelToken::new();
    let source = properties(&mut doc, &BodyFilter::AllVisible, &cancel).unwrap();
    let volume = source.total.as_ref().unwrap().volume_mm3.unwrap();
    let triangles = 12 + 1280;

    for file in ["all.stl", "all.3mf"] {
        let path = dir.path().join(file);
        let summary = export(&mut doc, &ExportRequest::new(&path), &cancel).unwrap();
        let back = common::measure_file(&path);
        let error = common::relative(back.volume, volume);
        eprintln!(
            "[export] {file}: {} bodies, {} triangles (re-import {}), {} bytes, volume {volume} \
             -> {} (relative {error:e}), {:.2} ms",
            summary.bodies,
            summary.triangles,
            back.triangles,
            summary.bytes,
            back.volume,
            summary.millis
        );
        assert_eq!(summary.schema, 1);
        assert_eq!(summary.path, path);
        assert_eq!(summary.format, path.extension().unwrap().to_str().unwrap());
        assert_eq!(summary.bodies, 2);
        assert_eq!(summary.triangles, triangles);
        assert_eq!(summary.bytes, std::fs::metadata(&path).unwrap().len());
        assert_eq!(back.triangles, triangles);
        assert!(error <= 1e-9, "{file}: {error:e}");
    }
}

/// 3MF keeps full `f64` coordinates with the request's lossless default; STL is `f32` by
/// format, so only a mesh that already is `f32` survives it to 1e-9.
#[test]
fn full_precision_coordinates_survive_3mf_but_not_stl() {
    amg_script::link().unwrap();
    let dir = tempfile::tempdir().unwrap();
    let mut source = Document::new("f64");
    let root = source.design().root();
    common::insert(
        &mut source,
        root,
        "ball",
        common::sphere(Point3::new(0.1, 0.2, 0.3), 1.7),
    );
    let mut doc = open(&source, dir.path(), "f64");
    let cancel = CancelToken::new();
    let volume = properties(&mut doc, &BodyFilter::AllVisible, &cancel)
        .unwrap()
        .total
        .unwrap()
        .volume_mm3
        .unwrap();
    let mut error = |file: &str, precision: CoordPrecision| {
        let mut req = ExportRequest::new(dir.path().join(file));
        req.options.precision = precision;
        export(&mut doc, &req, &cancel).unwrap();
        common::relative(common::measure_file(&req.path).volume, volume)
    };
    let lossless = error("a.3mf", CoordPrecision::Roundtrip);
    let six = error("b.3mf", CoordPrecision::Decimals(6));
    let stl = error("c.stl", CoordPrecision::Roundtrip);
    eprintln!(
        "[export] f64 sphere volume error: 3MF roundtrip {lossless:e}, 3MF 6 decimals {six:e}, \
         STL {stl:e}"
    );
    assert!(lossless <= 1e-9);
    assert!(stl < 1e-5);
}

#[test]
fn export_selects_bodies_by_name_and_id() {
    let dir = tempfile::tempdir().unwrap();
    let mut doc = open(&common::mesh_only(), dir.path(), "select");
    let cancel = CancelToken::new();
    let path = dir.path().join("one.stl");
    let mut req = ExportRequest::new(&path);
    req.bodies = BodyFilter::Names(vec!["cube".to_owned()]);
    let summary = export(&mut doc, &req, &cancel).unwrap();
    assert_eq!((summary.bodies, summary.triangles), (1, 12));
    let back = common::measure_file(&path);
    assert_eq!(back.triangles, 12);
    assert!(common::relative(back.volume, 1000.0) <= 1e-9);

    let rows = properties(&mut doc, &req.bodies, &cancel).unwrap();
    let id = rows.bodies[0].id.unwrap();
    req.bodies = BodyFilter::Ids(vec![id, id]);
    assert_eq!(export(&mut doc, &req, &cancel).unwrap().bodies, 1);

    // Unknown names and ids, also next to known ones, and nothing is written.
    let missing = dir.path().join("missing.stl");
    let mut req = ExportRequest::new(&missing);
    req.bodies = BodyFilter::Names(vec!["cube".to_owned(), "nope".to_owned()]);
    let err = export(&mut doc, &req, &cancel).unwrap_err();
    assert!(
        matches!(&err, ScriptError::NotFound { what: "body", name } if name == "nope"),
        "{err:?}"
    );
    assert_eq!(err.to_string(), "no body named `nope`");
    req.bodies = BodyFilter::Ids(vec![amg_doc::BodyId { slot: 7, ..id }]);
    assert!(matches!(
        export(&mut doc, &req, &cancel),
        Err(ScriptError::NotFound { what: "body", .. })
    ));
    assert!(!missing.exists());

    let err = export(
        &mut doc,
        &ExportRequest::new(dir.path().join("x.unknown")),
        &cancel,
    )
    .unwrap_err();
    assert!(matches!(err, ScriptError::Io(IoError::UnknownFormat)));
    let mut req = ExportRequest::new(dir.path().join("explicit.bin"));
    req.format = Some(Format::Stl);
    assert_eq!(export(&mut doc, &req, &cancel).unwrap().format, "stl");
}

/// Writes a binary STL with awkward `f32` coordinates: large, tiny and fractional.
fn source_stl(path: &Path) {
    let mut ball: IndexedMesh = common::sphere_f32(Point3::new(0.013, -251.7, 1034.3), 0.0371);
    let cube = common::cube(Point3::new(-0.015625, 0.001953125, 3.0), 7.5);
    ball.append(&cube).unwrap();
    let scene = common::scene(vec![("part", ball)]);
    let options = amg_io::ExportOptions {
        stl: StlWriteOptions::default(),
        ..amg_io::ExportOptions::default()
    };
    amg_io::export(
        &scene,
        Sink::Path(path),
        Format::Stl,
        &options,
        &JobCtx::detached(),
    )
    .unwrap();
}

#[test]
fn convert_stl_3mf_stl_keeps_triangles_and_bbox_exactly() {
    let dir = tempfile::tempdir().unwrap();
    let a = dir.path().join("a.stl");
    let b = dir.path().join("b.3mf");
    let c = dir.path().join("c.stl");
    source_stl(&a);
    let jobs = common::jobs();
    let cancel = CancelToken::new();
    let first = convert(&ConvertRequest::new(&a, &b), &jobs, &cancel).unwrap();
    let second = convert(&ConvertRequest::new(&b, &c), &jobs, &cancel).unwrap();
    let (before, middle, after) = (
        common::measure_file(&a),
        common::measure_file(&b),
        common::measure_file(&c),
    );
    eprintln!(
        "[convert] triangles {} -> {} -> {}; bbox {:?} -> {:?} -> {:?}; {:.2} + {:.2} ms",
        before.triangles,
        middle.triangles,
        after.triangles,
        before.bbox,
        middle.bbox,
        after.bbox,
        first.millis,
        second.millis
    );
    assert_eq!(first.format, "3mf");
    assert_eq!(second.format, "stl");
    assert_eq!((first.bodies, second.bodies), (1, 1));
    assert_eq!(before.triangles, 12 + 1280);
    for (summary, m) in [(&first, &middle), (&second, &after)] {
        assert_eq!(summary.triangles, before.triangles);
        assert_eq!(m.triangles, before.triangles);
        // Bit-exact.
        assert_eq!(m.bbox, before.bbox);
    }
    // Every coordinate, not only the box.
    let (x, y) = (common::read_scene(&a), common::read_scene(&c));
    assert_eq!(x.objects[0].mesh.positions, y.objects[0].mesh.positions);
    assert_eq!(x.objects[0].mesh.tris, y.objects[0].mesh.tris);
}

#[test]
fn convert_applies_the_unit_and_reports_failures() {
    let dir = tempfile::tempdir().unwrap();
    let a = dir.path().join("a.stl");
    source_stl(&a);
    let jobs = common::jobs();
    let cancel = CancelToken::new();

    let mut req = ConvertRequest::new(&a, dir.path().join("inch.3mf"));
    req.unit = Some(LengthUnit::Inch);
    convert(&req, &jobs, &cancel).unwrap();
    let (mm, inch) = (common::measure_file(&a), common::measure_file(&req.dst));
    let size = |m: &common::FileMeasure| m.bbox.1 - m.bbox.0;
    assert!((size(&inch) - size(&mm) * 25.4).abs().max_element() < 1e-9);

    let missing = ConvertRequest::new(dir.path().join("no.stl"), dir.path().join("no.3mf"));
    let err = convert(&missing, &jobs, &cancel).unwrap_err();
    assert!(
        matches!(err, ScriptError::Io(IoError::Read { .. })),
        "{err:?}"
    );
    let unknown = ConvertRequest::new(&a, dir.path().join("a.nope"));
    assert!(matches!(
        convert(&unknown, &jobs, &cancel),
        Err(ScriptError::Io(IoError::UnknownFormat))
    ));
    cancel.cancel();
    let stopped = ConvertRequest::new(&a, dir.path().join("stopped.3mf"));
    assert!(matches!(
        convert(&stopped, &jobs, &cancel),
        Err(ScriptError::Cancelled)
    ));
    assert!(!stopped.dst.exists() && !missing.dst.exists());
}

#[test]
fn properties_of_a_10_mm_cube() {
    amg_script::link().unwrap();
    let dir = tempfile::tempdir().unwrap();
    let mut source = Document::new("cube");
    let root = source.design().root();
    common::insert(
        &mut source,
        root,
        "cube",
        common::cube(Point3::new(1.0, 2.0, 3.0), 10.0),
    );
    let mut doc = open(&source, dir.path(), "cube");
    let report = properties(&mut doc, &BodyFilter::AllVisible, &CancelToken::new()).unwrap();
    assert_eq!(report.schema, 1);
    assert_eq!(report.bodies.len(), 1);
    let total = report.total.as_ref().unwrap();
    for row in [&report.bodies[0], total] {
        let (v, a) = (row.volume_mm3.unwrap(), row.area_mm2);
        eprintln!(
            "[props] {}: volume {v} mm3 (error {:e}), area {a} mm2 (error {:e})",
            row.name,
            (v - 1000.0).abs(),
            (a - 600.0).abs()
        );
        assert!((v - 1000.0).abs() <= 1e-9);
        assert!((a - 600.0).abs() <= 1e-9);
        let c = Vec3::from_array(row.centroid.unwrap());
        assert!((c - Vec3::new(6.0, 7.0, 8.0)).length() < 1e-9);
        assert_eq!(row.bbox_min, Some([1.0, 2.0, 3.0]));
        assert_eq!(row.bbox_max, Some([11.0, 12.0, 13.0]));
        assert_eq!(row.watertight, Some(true));
        assert_eq!(row.mass_g, None, "no material assigned");
    }
    assert_eq!(report.bodies[0].name, "cube");
    assert_eq!(report.bodies[0].kind, "mesh");
    assert_eq!((total.id, total.kind), (None, "total"));
}

/// A component placed twice, with a material on its body: one row per placement, masses and
/// totals, and the export applies the placements.
#[test]
fn placements_and_materials_are_applied() {
    amg_script::link().unwrap();
    let dir = tempfile::tempdir().unwrap();
    let mut source = Document::new("assembly");
    let root = source.design().root();
    let at = |x: f64| Transform::from_translation(Vec3::new(x, 0.0, 0.0));
    let made = source
        .apply(Command::NewComponent {
            parent: root,
            name: "part".to_owned(),
            transform: at(0.0),
        })
        .unwrap();
    let component = made
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Component(id) => Some(*id),
            _ => None,
        })
        .unwrap();
    source
        .apply(Command::AddOccurrence {
            parent: root,
            component,
            transform: at(100.0),
            name: None,
        })
        .unwrap();
    let body = common::insert(
        &mut source,
        component,
        "cube",
        common::cube(Point3::ZERO, 10.0),
    );
    // 8 g/cm3.
    let material: Command = serde_json::from_value(serde_json::json!({
        "SetMaterial": { "material": {
            "id": "test/dense", "revision": 1, "name": "Dense", "category": "metal",
            "density": 8.0e-9
        }}
    }))
    .unwrap();
    source.apply(material).unwrap();
    let mut props: BodyProps = source.design().body_props(body);
    props.material = Some(amg_doc::stub::MaterialKey("test/dense".to_owned()));
    source.apply(Command::SetBodyProps { body, props }).unwrap();

    let mut doc = open(&source, dir.path(), "assembly");
    let cancel = CancelToken::new();
    let report = properties(&mut doc, &BodyFilter::AllVisible, &cancel).unwrap();
    assert_eq!(report.bodies.len(), 2);
    for row in &report.bodies {
        assert!((row.mass_g.unwrap() - 8.0).abs() < 1e-9, "{row:?}");
    }
    let total = report.total.unwrap();
    assert!((total.volume_mm3.unwrap() - 2000.0).abs() < 1e-9);
    assert!((total.mass_g.unwrap() - 16.0).abs() < 1e-9);
    let c = Vec3::from_array(total.centroid.unwrap());
    assert!((c - Vec3::new(55.0, 5.0, 5.0)).length() < 1e-9, "{c:?}");
    assert_eq!(total.bbox_min, Some([0.0, 0.0, 0.0]));
    assert_eq!(total.bbox_max, Some([110.0, 10.0, 10.0]));

    let path = dir.path().join("assembly.3mf");
    export(&mut doc, &ExportRequest::new(&path), &cancel).unwrap();
    let back = common::measure_file(&path);
    assert_eq!(back.triangles, 24);
    assert!((back.volume - 2000.0).abs() < 1e-9);
    assert_eq!(back.bbox.1, Point3::new(110.0, 10.0, 10.0));
}

#[test]
fn open_empty_hidden_and_solid_bodies() {
    amg_script::link().unwrap();
    let dir = tempfile::tempdir().unwrap();
    let mut source = Document::new("odd");
    let root = source.design().root();
    let cube = common::cube(Point3::ZERO, 10.0);
    let mut faces = cube.tris.clone();
    faces.pop();
    let open_cube = IndexedMesh::new(cube.positions.clone(), faces).unwrap();
    let open_body = common::insert(&mut source, root, "open", open_cube);
    let hidden = common::insert(
        &mut source,
        root,
        "hidden",
        common::cube(Point3::new(20.0, 0.0, 0.0), 1.0),
    );
    let mut props = source.design().body_props(hidden);
    props.visible = false;
    source
        .apply(Command::SetBodyProps {
            body: hidden,
            props,
        })
        .unwrap();
    // The test feature adds an empty, unnamed mesh body.
    let probe = common::add_feature(&mut source, common::counter(open_body, 0, false), "probe");
    let mut doc = open(&source, dir.path(), "odd");
    let cancel = CancelToken::new();

    let all = properties(&mut doc, &BodyFilter::AllVisible, &cancel).unwrap();
    let names: Vec<&str> = all.bodies.iter().map(|b| b.name.as_str()).collect();
    let unnamed = format!("Body {}.0", probe.0);
    assert_eq!(names, ["open", unnamed.as_str()]);
    let row = &all.bodies[0];
    assert_eq!(row.watertight, Some(false));
    assert_eq!((row.volume_mm3, row.centroid), (None, None));
    assert!(row.area_mm2 > 0.0);
    assert_eq!(
        (all.bodies[1].bbox_min, all.bodies[1].bbox_max),
        (None, None)
    );
    let total = all.total.as_ref().unwrap();
    assert_eq!((total.volume_mm3, total.watertight), (None, Some(false)));

    // A hidden body is selected by name.
    let filter = BodyFilter::Names(vec!["hidden".to_owned()]);
    assert_eq!(
        properties(&mut doc, &filter, &cancel).unwrap().bodies.len(),
        1
    );
    // The fallback name works as a filter.
    let filter = BodyFilter::Names(vec![unnamed]);
    assert_eq!(
        properties(&mut doc, &filter, &cancel).unwrap().bodies.len(),
        1
    );
    // No selected body: no total.
    let none = properties(&mut doc, &BodyFilter::Ids(Vec::new()), &cancel).unwrap();
    assert!(none.bodies.is_empty() && none.total.is_none());

    let root = doc.document().design().root();
    let solid = FeatureBox::new(TestBox { size: [1.0; 3] });
    doc.apply("Box", vec![common::add_command(root, solid, "box")])
        .unwrap();
    let err = properties(&mut doc, &BodyFilter::AllVisible, &cancel).unwrap_err();
    assert!(
        matches!(
            err,
            ScriptError::NotFound {
                what: "solid properties",
                ..
            }
        ),
        "{err:?}"
    );
}

/// Done-when 5.
#[test]
fn report_json_matches_the_schema_snapshots() {
    let dir = tempfile::tempdir().unwrap();
    let cancel = CancelToken::new();
    let jobs = common::jobs();
    let schema = |name: &str| common::schema::load(name);
    let valid = |name: &str, value: &serde_json::Value| {
        let problems = common::schema::check(&schema(name), value);
        assert!(problems.is_empty(), "{name}: {problems:#?}\n{value:#}");
        assert_eq!(value["schema"], 1);
    };

    let fixtures = [
        (common::mesh_only(), "mesh-only"),
        (common::mixed().0, "mixed"),
        (common::failing().0, "failing"),
    ];
    let mut checked = 0;
    for (source, name) in &fixtures {
        let path = common::save(source, dir.path(), name);
        let mut doc = ScriptDoc::open(&path, Arc::clone(&jobs)).unwrap();
        let r = recompute(&mut doc, &cancel).unwrap();
        valid("recompute_report", &serde_json::to_value(&r).unwrap());
        let p = properties(&mut doc, &BodyFilter::AllVisible, &cancel).unwrap();
        valid("props_report", &serde_json::to_value(&p).unwrap());
        let mut req = ExportRequest::new(dir.path().join(format!("{name}.stl")));
        req.bodies = BodyFilter::Names(vec!["cube".to_owned()]);
        let e = export(&mut doc, &req, &cancel).unwrap();
        valid("export_summary", &serde_json::to_value(&e).unwrap());
        checked += 3;
    }
    let none = properties(
        &mut ScriptDoc::new("empty", Arc::clone(&jobs)),
        &BodyFilter::AllVisible,
        &cancel,
    )
    .unwrap();
    valid("props_report", &serde_json::to_value(&none).unwrap());
    let converted = convert(
        &ConvertRequest::new(
            dir.path().join("mesh-only.stl"),
            dir.path().join("converted.obj"),
        ),
        &jobs,
        &cancel,
    )
    .unwrap();
    let value = serde_json::to_value(&converted).unwrap();
    valid("export_summary", &value);
    eprintln!("[schema] {} reports validated", checked + 2);

    // The check really rejects drift: a new, a missing, a mistyped and an unknown value.
    let export_schema = schema("export_summary");
    let broken = |edit: &dyn Fn(&mut serde_json::Map<String, serde_json::Value>)| {
        let mut v = value.clone();
        edit(v.as_object_mut().unwrap());
        common::schema::check(&export_schema, &v)
    };
    for problems in [
        broken(&|o| {
            o.insert("extra".to_owned(), 1.into());
        }),
        broken(&|o| {
            o.remove("bytes");
        }),
        broken(&|o| {
            o.insert("triangles".to_owned(), "many".into());
        }),
        broken(&|o| {
            o.insert("schema".to_owned(), 2.into());
        }),
        broken(&|o| {
            o.insert("format".to_owned(), "step".into());
        }),
    ] {
        assert_eq!(problems.len(), 1, "{problems:?}");
    }
}
