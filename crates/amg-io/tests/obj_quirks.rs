//! IO-05 Done-when 2: synthetic OBJ quirk cases through the public `import`. Each case
//! checks the geometry and the exact warnings.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test code; a failed expectation is a test failure"
)]

use amg_core::{Point3, units::LengthUnit};
use amg_io::{
    Format, ImportOptions, Imported, IoError, IoWarning, Location, NO_SLOT, ObjGroups, ReadLimits,
    Source, UnitChoice, UpAxis, import,
};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::Rgba8;

fn run_with(text: &str, opts: &ImportOptions) -> Result<Imported, IoError> {
    import(
        Source::Bytes(text.as_bytes()),
        Some(Format::Obj),
        opts,
        &JobCtx::detached(),
    )
}

fn run(text: &str) -> Result<Imported, IoError> {
    run_with(text, &ImportOptions::default())
}

fn faces(i: &Imported) -> usize {
    i.scene.objects.iter().map(|o| o.mesh.face_count()).sum()
}

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

const QUAD_VERTS: &str = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n";

fn scratch_dir() -> tempfile::TempDir {
    let base = std::env::var_os("CARGO_TARGET_DIR")
        .map_or_else(std::env::temp_dir, std::path::PathBuf::from);
    std::fs::create_dir_all(&base).expect("base");
    tempfile::Builder::new()
        .prefix("obj-quirks")
        .tempdir_in(base)
        .expect("tempdir")
}

#[test]
fn q01_negative_indices_resolve_relative_to_the_vertices_so_far() {
    // The second face is written after more vertices, so -1 means a different vertex.
    let t = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\nv 5 5 5\nv 6 5 5\nv 5 6 5\nf -3 -2 -1\n";
    let i = run(t).expect("read");
    let m = &i.scene.objects[0].mesh;
    assert_eq!(m.tris, vec![[0, 1, 2], [3, 4, 5]]);
    assert_eq!(m.positions[3], p(5., 5., 5.));
    assert!(i.report.warnings.is_empty(), "{:?}", i.report.warnings);
    assert_eq!(i.report.triangles_read, 2);
    assert_eq!(i.report.vertices, 6);
}

#[test]
fn q02_every_face_token_form_and_mixed_uvs() {
    let t = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\nvt 0 0\nvt 1 0\nvt 0 1\nvn 0 0 1\n\
             f 1 2 3\nf 1/1 2/2 3/3\nf 1//1 2//1 3//1\nf 2/-2/1 4/-1/1 3/-3/1\n";
    let i = run(t).expect("read");
    let m = &i.scene.objects[0].mesh;
    assert_eq!(m.face_count(), 4);
    let uv = m.attrs.corner_uvs.as_ref().expect("uvs");
    assert_eq!(uv.len(), 4);
    // Face 1 has no uvs, face 2 has them, face 3 none, face 4 relative uvs.
    assert_eq!(uv[0][0], amg_core::Point2::ZERO);
    assert_eq!(uv[1][1], amg_core::Point2::new(1.0, 0.0));
    assert_eq!(uv[3][0], amg_core::Point2::new(1.0, 0.0));
    assert_eq!(uv[3][2], amg_core::Point2::new(0.0, 0.0));
    assert!(i.report.warnings.is_empty());
}

#[test]
fn q03_concave_polygon_is_ear_clipped_not_fanned() {
    // An L-shaped hexagon: the fan from vertex 0 leaves the polygon.
    let t = "v 0 0 0\nv 2 0 0\nv 2 1 0\nv 1 1 0\nv 1 2 0\nv 0 2 0\nf 1 2 3 4 5 6\n";
    let i = run(t).expect("read");
    let m = &i.scene.objects[0].mesh;
    assert_eq!(m.face_count(), 4);
    assert!((m.area() - 3.0).abs() < 1e-12);
    for f in 0..4 {
        assert!(m.face_normal(amg_mesh::FaceId(f)).z > 0.0, "flipped {f}");
    }
    assert_eq!(
        i.report.warnings,
        vec![IoWarning::PolygonTriangulated {
            polygons: 1,
            fallback_fan: 0
        }]
    );
}

#[test]
fn q04_degenerate_polygon_uses_the_fan_and_says_so() {
    let t = "v 0 0 0\nv 1 0 0\nv 2 0 0\nv 3 0 0\nf 1 2 3 4\n";
    let i = run(t).expect("read");
    assert_eq!(i.scene.objects[0].mesh.tris, vec![[0, 1, 2], [0, 2, 3]]);
    assert_eq!(
        i.report.warnings,
        vec![IoWarning::PolygonTriangulated {
            polygons: 1,
            fallback_fan: 1
        }]
    );
    assert_eq!(i.report.degenerate_triangles, 2);
}

#[test]
fn q05_missing_mtllib_warns_and_uses_the_default_colour() {
    let dir = scratch_dir();
    let path = dir.path().join("a.obj");
    std::fs::write(
        &path,
        "mtllib nothing.mtl\nusemtl Red\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
    )
    .expect("write");
    let i = import(
        Source::Path(&path),
        None,
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .expect("read");
    assert_eq!(i.scene.materials.len(), 1);
    assert_eq!(i.scene.materials[0].name, "Red");
    assert_eq!(i.scene.materials[0].color, Rgba8([204, 204, 204, 255]));
    assert_eq!(i.report.warnings.len(), 1, "{:?}", i.report.warnings);
    assert!(matches!(
        &i.report.warnings[0],
        IoWarning::MissingReference { what } if what.contains("nothing.mtl")
    ));
    let fm = i.scene.objects[0].face_materials.as_ref().expect("slots");
    assert_eq!(&fm[..], &[0]);
}

#[test]
fn q06_line_continuation_joins_statements() {
    let t = "v 0 0 \\\n0\nv 1 0 0\nv 0 1 0\nf 1 \\\n  2 \\\r\n 3\n";
    let i = run(t).expect("read");
    let m = &i.scene.objects[0].mesh;
    assert_eq!(m.tris, vec![[0, 1, 2]]);
    assert_eq!(m.positions[0], p(0., 0., 0.));
    assert!(i.report.warnings.is_empty());
}

#[test]
fn q07_crlf_and_bom_and_tabs() {
    let t = "\u{FEFF}# c\r\nv\t0 0 0\r\nv 1  0 0\r\nv 0 1 0\r\n\r\nf 1 2 3\r\n";
    let i = run(t).expect("read");
    assert_eq!(faces(&i), 1);
    assert_eq!(i.scene.objects[0].mesh.positions.len(), 3);
    assert!(i.report.warnings.is_empty());
}

#[test]
fn q08_vertex_colours_as_floats_and_as_bytes() {
    let f = "v 0 0 0 1 0 0\nv 1 0 0 0 0.5 1\nv 0 1 0\nf 1 2 3\n";
    let i = run(f).expect("read");
    let c = i.scene.objects[0]
        .mesh
        .attrs
        .vertex_colors
        .clone()
        .expect("colours");
    assert_eq!(c[0], Rgba8([255, 0, 0, 255]));
    assert_eq!(c[1], Rgba8([0, 128, 255, 255]));
    assert_eq!(c[2], Rgba8([255, 255, 255, 255]));
    // 0-255 integers (some exporters) are detected from the range.
    let b = "v 0 0 0 255 0 0\nv 1 0 0 0 128 255 128\nv 0 1 0 10 20 30\nf 1 2 3\n";
    let i = run(b).expect("read");
    let c = i.scene.objects[0]
        .mesh
        .attrs
        .vertex_colors
        .clone()
        .expect("colours");
    assert_eq!(c[0], Rgba8([255, 0, 0, 255]));
    assert_eq!(c[1], Rgba8([0, 128, 255, 128]));
    assert_eq!(c[2], Rgba8([10, 20, 30, 255]));
}

#[test]
fn q09_object_without_faces_is_dropped_with_a_warning() {
    let t = format!("o Empty\n{QUAD_VERTS}o Real\nf 1 2 3\nf 1 3 4\no Last\n");
    let i = run(&t).expect("read");
    assert_eq!(i.scene.objects.len(), 1);
    assert_eq!(i.scene.objects[0].name.as_deref(), Some("Real"));
    assert_eq!(
        i.report.warnings,
        vec![IoWarning::UnsupportedContentIgnored {
            what: "object without faces".into(),
            count: 2
        }]
    );
}

#[test]
fn q10_out_of_range_and_zero_indices_drop_the_face() {
    let t = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\nf 1 2 9\nf 0 1 2\nf -9 1 2\nf 1 2 3 7\n";
    let i = run(t).expect("read");
    assert_eq!(faces(&i), 1);
    assert_eq!(i.report.warnings, vec![IoWarning::InvalidIndexDropped(4)]);
}

#[test]
fn q11_faces_before_their_vertices_are_fine() {
    let t = "f 1 2 3\nv 0 0 0\nv 1 0 0\nv 0 1 0\n";
    let i = run(t).expect("read");
    assert_eq!(i.scene.objects[0].mesh.tris, vec![[0, 1, 2]]);
}

#[test]
fn q12_non_finite_vertices_drop_their_faces() {
    let t = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv nan 0 0\nv 1 1 1\nf 1 2 3\nf 1 2 4\nf 1 2 3 4 5\n";
    let i = run(t).expect("read");
    assert_eq!(faces(&i), 1);
    assert_eq!(i.report.warnings, vec![IoWarning::NonFiniteDropped(2)]);
}

#[test]
fn q13_unsupported_statements_are_counted() {
    let t = "v 0 0 0\nv 1 0 0\nv 0 1 0\nl 1 2\np 3\ncstype bspline\ncurv 0 1 1 2\nend\nbogus 1\n\
             s off\nf 1 2\nf 1 2 3\n";
    let i = run(t).expect("read");
    assert_eq!(faces(&i), 1);
    let w = &i.report.warnings;
    for (what, count) in [
        ("face with fewer than three vertices", 1),
        ("line elements (l)", 1),
        ("point elements (p)", 1),
        ("curves and surfaces", 3),
        ("unrecognised statements", 1),
    ] {
        assert!(
            w.contains(&IoWarning::UnsupportedContentIgnored {
                what: what.into(),
                count
            }),
            "{what}: {w:?}"
        );
    }
}

#[test]
fn q14_objects_groups_and_the_three_modes() {
    let t = format!("{QUAD_VERTS}o A\nf 1 2 3\ng top\nf 1 3 4\ng bottom\nf 1 2 4\no B\nf 2 3 4\n");
    // Objects: o and g both start objects.
    let i = run(&t).expect("read");
    let names: Vec<_> = i.scene.objects.iter().map(|o| o.name.clone()).collect();
    assert_eq!(
        names,
        vec![
            Some("A".to_owned()),
            Some("top".to_owned()),
            Some("bottom".to_owned()),
            Some("B".to_owned())
        ]
    );
    // Each object keeps only the vertices it uses, in file order.
    assert_eq!(
        i.scene.objects[3].mesh.positions,
        vec![p(1., 0., 0.), p(1., 1., 0.), p(0., 1., 0.)]
    );
    assert_eq!(i.scene.objects[3].mesh.tris, vec![[0, 1, 2]]);

    // FaceGroups: g becomes a face group, o still splits.
    let mut o = ImportOptions::default();
    o.obj.groups_as = ObjGroups::FaceGroups;
    let i = run_with(&t, &o).expect("read");
    assert_eq!(i.scene.objects.len(), 2);
    let a = &i.scene.objects[0].mesh;
    assert_eq!(a.face_count(), 3);
    let names: Vec<_> = a
        .groups
        .ids()
        .map(|g| a.groups.info(g).and_then(|x| x.name.clone()))
        .collect();
    assert_eq!(
        names,
        vec![None, Some("top".to_owned()), Some("bottom".to_owned())]
    );
    let g1 = a.groups.group_of(amg_mesh::FaceId(1));
    assert_eq!(
        a.groups.info(g1).expect("info").name.as_deref(),
        Some("top")
    );
    assert!(matches!(
        a.groups.info(g1).expect("info").origin,
        amg_mesh::GroupOrigin::Import { object: 0 }
    ));

    // Ignore: one object.
    o.obj.groups_as = ObjGroups::Ignore;
    let i = run_with(&t, &o).expect("read");
    assert_eq!(i.scene.objects.len(), 1);
    assert_eq!(i.scene.objects[0].mesh.face_count(), 4);
    assert_eq!(i.scene.objects[0].name, None);
}

#[test]
fn q15_usemtl_slots_with_an_mtl_file() {
    let dir = scratch_dir();
    std::fs::write(
        dir.path().join("m.mtl"),
        "newmtl Red\nKd 1 0 0\nnewmtl Glass\nKd 0.2 0.4 0.6\nd 0.5\nnewmtl Unused\nKd 0 0 0\n",
    )
    .expect("mtl");
    let path = dir.path().join("a.obj");
    std::fs::write(
        &path,
        format!(
            "mtllib m.mtl\n{QUAD_VERTS}f 1 2 3\nusemtl Red\nf 1 3 4\nusemtl Glass\nf 1 2 4\n\
             usemtl off\nf 2 3 4\nusemtl Red\nf 1 2 3\n"
        ),
    )
    .expect("obj");
    let i = import(
        Source::Path(&path),
        None,
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .expect("read");
    let s = &i.scene;
    assert_eq!(
        s.materials
            .iter()
            .map(|m| m.name.as_str())
            .collect::<Vec<_>>(),
        ["Red", "Glass"]
    );
    assert_eq!(s.materials[0].color, Rgba8([255, 0, 0, 255]));
    assert_eq!(s.materials[1].color, Rgba8([51, 102, 153, 128]));
    let fm = s.objects[0].face_materials.as_ref().expect("slots");
    assert_eq!(&fm[..], &[NO_SLOT, 0, 1, NO_SLOT, 0]);
    assert!(i.report.warnings.is_empty(), "{:?}", i.report.warnings);

    // load_mtl = false keeps the slots but not the colours.
    let mut o = ImportOptions::default();
    o.obj.load_mtl = false;
    let i = import(Source::Path(&path), None, &o, &JobCtx::detached()).expect("read");
    assert_eq!(i.scene.materials[0].color, Rgba8([204, 204, 204, 255]));
}

#[test]
fn q16_mtllib_never_leaves_the_obj_folder() {
    let dir = scratch_dir();
    std::fs::write(dir.path().join("secret.mtl"), "newmtl Red\nKd 1 0 0\n").expect("mtl");
    let sub = dir.path().join("sub");
    std::fs::create_dir_all(&sub).expect("dir");
    let path = sub.join("a.obj");
    std::fs::write(
        &path,
        "mtllib ../secret.mtl\nusemtl Red\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
    )
    .expect("obj");
    let i = import(
        Source::Path(&path),
        None,
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .expect("read");
    assert_eq!(i.scene.materials[0].color, Rgba8([204, 204, 204, 255]));
    assert!(matches!(
        &i.report.warnings[0],
        IoWarning::MissingReference { .. }
    ));
}

#[test]
fn q17_empty_and_comment_only_files() {
    assert!(matches!(run(""), Err(IoError::Empty)));
    assert!(matches!(run("# nothing\n\n"), Err(IoError::Empty)));
    assert!(matches!(run("v 0 0 0\nv 1 0 0\n"), Err(IoError::Empty)));
    assert!(matches!(
        run("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 7\n"),
        Err(IoError::Empty)
    ));
}

#[test]
fn q18_malformed_lines_report_their_line_number() {
    let t = "v 0 0 0\nv 1 0 0\n# c\nv 0 x 0\nf 1 2 3\n";
    match run(t) {
        Err(IoError::Malformed {
            format: Format::Obj,
            location: Location::Line(4),
            ..
        }) => {}
        o => panic!("{o:?}"),
    }
    // Same line after a continuation and in a later parallel block.
    let t = "v 0 0 0\r\nv 1 0 0\r\nv 0 1 0\r\nf 1 2 \\\r\n 3\r\nf 1 two 3\r\n";
    match run(t) {
        Err(IoError::Malformed {
            location: Location::Line(6),
            ..
        }) => {}
        o => panic!("{o:?}"),
    }
    assert!(matches!(
        run("v 1 2\nf 1 1 1\n"),
        Err(IoError::Malformed {
            location: Location::Line(1),
            ..
        })
    ));
}

#[test]
fn q19_limits_are_enforced() {
    let t = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\nf 1 2 3\n";
    let mut o = ImportOptions {
        limits: ReadLimits {
            max_vertices: 2,
            ..ReadLimits::default()
        },
        ..ImportOptions::default()
    };
    assert!(matches!(
        run_with(t, &o),
        Err(IoError::LimitExceeded {
            what: "vertices",
            ..
        })
    ));
    o.limits = ReadLimits {
        max_triangles: 1,
        ..ReadLimits::default()
    };
    assert!(matches!(
        run_with(t, &o),
        Err(IoError::LimitExceeded {
            what: "triangles",
            ..
        })
    ));
}

#[test]
fn q20_unit_up_axis_and_merge_go_through_import() {
    let t = "o A\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\no B\nv 0 0 1\nv 1 0 1\nv 0 1 1\nf 4 5 6\n";
    let mut o = ImportOptions {
        unit: UnitChoice::Assume(LengthUnit::Inch),
        up_axis: UpAxis::Y,
        ..ImportOptions::default()
    };
    let i = run_with(t, &o).expect("read");
    assert_eq!(i.report.applied_unit, Some(LengthUnit::Inch));
    let m = &i.scene.objects[1].mesh;
    assert!((m.positions[1].x - 25.4).abs() < 1e-12);
    assert_ne!(i.scene.roots[0].transform, amg_core::Transform::IDENTITY);
    o.merge_objects = true;
    let i = run_with(t, &o).expect("read");
    assert_eq!(i.scene.objects.len(), 1);
    assert_eq!(i.scene.objects[0].mesh.face_count(), 2);
}

#[test]
fn q21_sniffing_picks_obj_and_flags_a_wrong_extension() {
    let dir = scratch_dir();
    let path = dir.path().join("model.stl");
    std::fs::write(&path, "# exported\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n").expect("write");
    let i = import(
        Source::Path(&path),
        None,
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .expect("read");
    assert_eq!(i.report.format, Some(Format::Obj));
    assert!(matches!(
        i.report.warnings.as_slice(),
        [IoWarning::ExtensionMismatch {
            detected: Format::Obj,
            ..
        }]
    ));
}

#[test]
fn q22_cancel_returns_cancelled() {
    let token = CancelToken::new();
    token.cancel();
    let ctx = JobCtx::with_token(token);
    let r = import(
        Source::Bytes(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"),
        Some(Format::Obj),
        &ImportOptions::default(),
        &ctx,
    );
    assert!(matches!(r, Err(IoError::Cancelled)));
}

#[test]
fn q23_object_and_group_with_default_names() {
    // `g default` is the implicit group and does not name anything.
    let t = format!("g default\n{QUAD_VERTS}f 1 2 3\ng\nf 1 3 4\n");
    let i = run(&t).expect("read");
    assert_eq!(i.scene.objects.len(), 2);
    assert!(i.scene.objects.iter().all(|o| o.name.is_none()));
}

#[test]
fn q24_o_names_survive_a_following_g() {
    let t = format!("o Body\ng part\n{QUAD_VERTS}f 1 2 3\n");
    let i = run(&t).expect("read");
    assert_eq!(i.scene.objects.len(), 1);
    assert_eq!(i.scene.objects[0].name.as_deref(), Some("Body"));
}

#[test]
fn q25_names_with_spaces_and_trailing_comments_of_usemtl() {
    let t = format!("usemtl  Brushed Steel  \n{QUAD_VERTS}f 1 2 3\n");
    let i = run(&t).expect("read");
    assert_eq!(i.scene.materials[0].name, "Brushed Steel");
}

#[test]
fn q26_registry_reports_obj_read_and_write() {
    let info = amg_io::registry()
        .iter()
        .find(|i| i.format == Format::Obj)
        .expect("obj");
    assert!(info.can_import && info.can_export);
    assert!(info.multi_object && info.colors);
}
