//! IO-02 Done-when 2 (and parts of 4): synthetic STL quirk cases through the public
//! `import` / `export`. Each case checks the geometry and the exact warnings.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test code; a failed expectation is a test failure"
)]

use amg_core::units::LengthUnit;
use amg_io::{
    ExportOptions, Format, ImportOptions, ImportReport, Imported, IoError, IoWarning, Location,
    NO_SLOT, ReadLimits, Sink, Source, StlWeld, UnitChoice, UpAxis, export, import,
};
use amg_jobs::JobCtx;

type Tri = [f32; 9];

const T0: Tri = [0., 0., 0., 1., 0., 0., 0., 1., 0.];
const T1: Tri = [1., 0., 0., 1., 1., 0., 0., 1., 0.];
const T2: Tri = [0., 0., 1., 1., 0., 1., 0., 1., 1.];

fn binary(header: &[u8], declared: u32, tris: &[Tri], attrs: &[u16]) -> Vec<u8> {
    let mut v = vec![b' '; 80];
    v[..header.len()].copy_from_slice(header);
    v.extend_from_slice(&declared.to_le_bytes());
    for (i, t) in tris.iter().enumerate() {
        v.extend_from_slice(&[0u8; 12]);
        for f in t {
            v.extend_from_slice(&f.to_le_bytes());
        }
        v.extend_from_slice(&attrs.get(i).copied().unwrap_or(0).to_le_bytes());
    }
    v
}

fn facet(t: &Tri) -> String {
    format!(
        "facet normal 0 0 1\n outer loop\n  vertex {} {} {}\n  vertex {} {} {}\n  vertex {} {} {}\n endloop\nendfacet\n",
        t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8]
    )
}

fn run_with(bytes: &[u8], opts: &ImportOptions) -> Result<Imported, IoError> {
    import(
        Source::Bytes(bytes),
        Some(Format::Stl),
        opts,
        &JobCtx::detached(),
    )
}

fn run(bytes: &[u8]) -> Result<Imported, IoError> {
    run_with(bytes, &ImportOptions::default())
}

fn faces(i: &Imported) -> usize {
    i.scene.objects.iter().map(|o| o.mesh.face_count()).sum()
}

fn no_warnings(r: &ImportReport) {
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
}

#[test]
fn q01_solid_headed_binary_is_binary() {
    let f = binary(b"solid facet exported by CAD", 2, &[T0, T1], &[]);
    let i = run(&f).expect("read");
    assert_eq!(faces(&i), 2);
    assert_eq!(i.scene.objects[0].mesh.vertex_count(), 4);
    no_warnings(&i.report);
}

#[test]
fn q02_count_zero_uses_the_file_size() {
    let f = binary(b"lazy writer", 0, &[T0, T1, T2], &[]);
    let i = run(&f).expect("read");
    assert_eq!(faces(&i), 3);
    assert_eq!(
        i.report.warnings,
        vec![IoWarning::HeaderCountMismatch {
            header: 0,
            actual: 3
        }]
    );
}

#[test]
fn q03_wrong_nonzero_count_uses_the_file_size() {
    let f = binary(b"x", 7, &[T0, T1], &[]);
    let i = run(&f).expect("read");
    assert_eq!(faces(&i), 2);
    assert_eq!(
        i.report.warnings,
        vec![IoWarning::HeaderCountMismatch {
            header: 7,
            actual: 2
        }]
    );
}

#[test]
fn q04_truncated_binary_reads_the_complete_triangles() {
    let mut f = binary(b"x", 5, &[T0, T1], &[]);
    f.extend_from_slice(&[0u8; 20]);
    let i = run(&f).expect("read");
    assert_eq!(faces(&i), 2);
    assert_eq!(
        i.report.warnings,
        vec![IoWarning::Truncated {
            expected: 5,
            read: 2
        }]
    );
}

#[test]
fn q05_trailing_bytes_are_ignored_with_a_warning() {
    let mut f = binary(b"x", 2, &[T0, T1], &[]);
    f.extend_from_slice(&[7u8; 7]);
    let i = run(&f).expect("read");
    assert_eq!(faces(&i), 2);
    assert_eq!(i.report.warnings, vec![IoWarning::TrailingBytes(7)]);
}

#[test]
fn q06_nan_triangle_is_dropped_and_counted() {
    let bad: Tri = [f32::NAN, 0., 0., 1., 0., 0., 0., 1., 0.];
    let inf: Tri = [0., 0., f32::INFINITY, 1., 0., 0., 0., 1., 0.];
    let f = binary(b"x", 4, &[T0, bad, T1, inf], &[]);
    let i = run(&f).expect("read");
    assert_eq!(faces(&i), 2);
    assert_eq!(i.report.warnings, vec![IoWarning::NonFiniteDropped(2)]);
}

#[test]
fn q07_all_triangles_non_finite_is_empty() {
    let bad: Tri = [f32::NAN; 9];
    let f = binary(b"x", 1, &[bad], &[]);
    assert!(matches!(run(&f), Err(IoError::Empty)));
}

#[test]
fn q08_empty_inputs() {
    assert!(matches!(run(b""), Err(IoError::Empty)));
    // 84 bytes, count 0: nothing to read.
    assert!(matches!(
        run(&binary(b"x", 0, &[], &[])),
        Err(IoError::Empty)
    ));
    // ASCII with a solid and no facet.
    assert!(matches!(
        run(b"solid empty\nendsolid empty\n"),
        Err(IoError::Empty)
    ));
    // ASCII whose only facet has no vertices.
    assert!(matches!(
        run(b"solid x\nfacet normal 0 0 1\nouter loop\nendloop\nendfacet\nendsolid x\n"),
        Err(IoError::Empty)
    ));
}

#[test]
fn q09_crlf_uppercase_and_tabs() {
    let text = "SOLID part\r\n\tFACET NORMAL 0 0 1\r\n\t\tOUTER LOOP\r\n\t\t\tVERTEX 0 0 0\r\n\t\t\tVERTEX\t1.0E0\t0\t0\r\n\t\t\tVERTEX 0 1 0\r\n\t\tENDLOOP\r\n\tENDFACET\r\nENDSOLID part\r\n";
    let i = run(text.as_bytes()).expect("read");
    assert_eq!(faces(&i), 1);
    assert_eq!(i.scene.objects[0].name.as_deref(), Some("part"));
    no_warnings(&i.report);
}

#[test]
fn q10_missing_endsolid() {
    let text = format!("solid a\n{}", facet(&T0));
    let i = run(text.as_bytes()).expect("read");
    assert_eq!(faces(&i), 1);
    no_warnings(&i.report);
}

#[test]
fn q11_multi_solid_merged_or_split() {
    let text = format!(
        "solid first\n{}endsolid first\nsolid second part\n{}{}endsolid second part\n",
        facet(&T0),
        facet(&T1),
        facet(&T2)
    );
    let merged = run(text.as_bytes()).expect("read");
    assert_eq!(merged.scene.objects.len(), 1);
    assert_eq!(faces(&merged), 3);
    assert_eq!(merged.scene.objects[0].name.as_deref(), Some("first"));

    let mut o = ImportOptions::default();
    o.stl.split_solids = true;
    let split = run_with(text.as_bytes(), &o).expect("read");
    let names: Vec<_> = split
        .scene
        .objects
        .iter()
        .map(|o| (o.name.clone().unwrap(), o.mesh.face_count()))
        .collect();
    assert_eq!(
        names,
        vec![("first".to_owned(), 1), ("second part".to_owned(), 2)]
    );
    assert_eq!(split.scene.roots.len(), 2);

    // merge_objects joins them again.
    o.merge_objects = true;
    let joined = run_with(text.as_bytes(), &o).expect("read");
    assert_eq!(joined.scene.objects.len(), 1);
    assert_eq!(faces(&joined), 3);
}

#[test]
fn q12_utf8_bom_and_leading_whitespace() {
    let text = format!("\u{feff}  \n solid b\n{}endsolid b\n", facet(&T0));
    let i = run(text.as_bytes()).expect("read");
    assert_eq!(faces(&i), 1);
    no_warnings(&i.report);
}

#[test]
fn q13_quad_in_outer_loop_is_fanned() {
    let text = "solid q\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 1 1 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid q\n";
    let i = run(text.as_bytes()).expect("read");
    assert_eq!(faces(&i), 2);
    assert_eq!(
        i.report.warnings,
        vec![IoWarning::PolygonTriangulated {
            polygons: 1,
            fallback_fan: 1
        }]
    );
    // Fan around the first vertex keeps the winding.
    assert_eq!(i.scene.objects[0].mesh.tris, vec![[0, 1, 2], [0, 2, 3]]);
}

#[test]
fn q14_loop_with_two_vertices_is_dropped() {
    let text = format!(
        "solid q\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\n{}endsolid q\n",
        facet(&T0)
    );
    let i = run(text.as_bytes()).expect("read");
    assert_eq!(faces(&i), 1);
    assert!(matches!(
        i.report.warnings.as_slice(),
        [IoWarning::UnsupportedContentIgnored { count: 1, .. }]
    ));
}

#[test]
fn q15_ascii_nan_vertex_is_dropped() {
    let text = format!(
        "solid n\n{}facet normal 0 0 1\nouter loop\nvertex nan 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid n\n",
        facet(&T0)
    );
    let i = run(text.as_bytes()).expect("read");
    assert_eq!(faces(&i), 1);
    assert_eq!(i.report.warnings, vec![IoWarning::NonFiniteDropped(1)]);
}

#[test]
fn q16_garbage_is_malformed_at_byte_zero() {
    let e = run(b"hello").expect_err("must fail");
    assert!(matches!(
        e,
        IoError::Malformed {
            format: Format::Stl,
            location: Location::Byte(0),
            ..
        }
    ));
}

#[test]
fn q17_bad_number_reports_its_line() {
    let text = format!(
        "solid n\n{}facet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 oops 0\nvertex 0 1 0\nendloop\nendfacet\n",
        facet(&T0)
    );
    match run(text.as_bytes()) {
        Err(IoError::Malformed {
            location: Location::Line(l),
            ..
        }) => assert_eq!(l, 12),
        other => panic!("{other:?}"),
    }
}

#[test]
fn q18_negative_zero_welds_with_positive_zero() {
    let a: Tri = [-0.0, 0., 0., 1., 0., 0., 0., 1., 0.];
    let b: Tri = [0.0, -0.0, 0.0, 0., 1., 0., 1., 1., 0.];
    let f = binary(b"x", 2, &[a, b], &[]);
    let i = run(&f).expect("read");
    let m = &i.scene.objects[0].mesh;
    assert_eq!(m.vertex_count(), 4);
    assert!(m.positions.iter().all(|p| p.x.is_sign_positive()));
}

#[test]
fn q19_degenerate_triangles_are_kept_and_counted() {
    let zero_area: Tri = [0., 0., 0., 1., 0., 0., 2., 0., 0.];
    let repeated: Tri = [0., 0., 0., 0., 0., 0., 0., 1., 0.];
    let f = binary(b"x", 3, &[T0, zero_area, repeated], &[]);
    let i = run(&f).expect("read");
    assert_eq!(faces(&i), 3);
    assert_eq!(i.report.degenerate_triangles, 2);
    no_warnings(&i.report);
}

#[test]
fn q20_weld_modes() {
    let near: Tri = [1.0, 0., 1e-6, 1., 1., 0., 0., 1., 0.];
    let f = binary(b"x", 2, &[T0, near], &[]);
    let mut o = ImportOptions::default();
    let exact = run_with(&f, &o).expect("read");
    assert_eq!(exact.scene.objects[0].mesh.vertex_count(), 5);
    o.stl.weld = StlWeld::None;
    let soup = run_with(&f, &o).expect("read");
    assert_eq!(soup.scene.objects[0].mesh.vertex_count(), 6);
    assert_eq!(soup.scene.objects[0].mesh.tris[1], [3, 4, 5]);
    o.stl.weld = StlWeld::Tolerance(1e-3);
    let tol = run_with(&f, &o).expect("read");
    assert_eq!(tol.scene.objects[0].mesh.vertex_count(), 4);
}

#[test]
fn q21_viscam_face_colours() {
    // bit 15 valid, red 31, green 0, blue 0; second face has no colour.
    let red = 0x8000 | (31 << 10);
    let f = binary(b"x", 2, &[T0, T1], &[red, 0]);
    let mut o = ImportOptions::default();
    // Off by default: the attribute word is ignored.
    let off = run_with(&f, &o).expect("read");
    assert!(off.scene.objects[0].face_materials.is_none());
    o.stl.face_colors = true;
    let i = run_with(&f, &o).expect("read");
    assert_eq!(i.scene.materials.len(), 1);
    assert_eq!(i.scene.materials[0].color.0, [255, 0, 0, 255]);
    let fm = i.scene.objects[0].face_materials.as_ref().expect("slots");
    assert_eq!(&fm[..], &[0, NO_SLOT]);
    i.scene.validate().expect("valid scene");
}

#[test]
fn q22_magics_colour_uses_the_inverse_bit_and_the_header_default() {
    let mut header = b"COLOR=".to_vec();
    header.extend_from_slice(&[0, 0, 255, 255]); // default blue
    // Face 0: bit 15 clear: red 0, green 31, blue 0. Face 1: bit 15 set: the default.
    let green = 31u16 << 5;
    let f = binary(&header, 2, &[T0, T1], &[green, 0x8000]);
    let mut o = ImportOptions::default();
    o.stl.face_colors = true;
    let i = run_with(&f, &o).expect("read");
    let cols: Vec<_> = i.scene.materials.iter().map(|m| m.color.0).collect();
    assert_eq!(cols, vec![[0, 255, 0, 255], [0, 0, 255, 255]]);
    let fm = i.scene.objects[0].face_materials.as_ref().expect("slots");
    assert_eq!(&fm[..], &[0, 1]);
}

#[test]
fn q23_unit_up_axis_and_merge_are_applied_by_import() {
    let f = binary(b"x", 1, &[[0., 0., 0., 1., 0., 0., 0., 0., 1.]], &[]);
    let o = ImportOptions {
        unit: UnitChoice::Assume(LengthUnit::Inch),
        up_axis: UpAxis::Y,
        ..ImportOptions::default()
    };
    let i = run_with(&f, &o).expect("read");
    assert_eq!(i.report.applied_unit, Some(LengthUnit::Inch));
    let p = i.scene.objects[0].mesh.positions[1];
    assert!((p.x - 25.4).abs() < 1e-12, "{p:?}");
    assert_eq!(i.scene.roots[0].transform, UpAxis::Y.to_z_up());
    // The world box is Z-up: the file's +Z edge lies along -Y.
    let bb = i.scene.world_aabb();
    assert!((bb.min.y + 25.4).abs() < 1e-12, "{bb:?}");
}

#[test]
fn q24_extension_mismatch_warns_when_sniffing() {
    let dir = tempfile::tempdir().expect("tempdir");
    let p = dir.path().join("part.obj");
    std::fs::write(&p, binary(b"solid x", 1, &[T0], &[])).expect("write");
    let i = import(
        Source::Path(&p),
        None,
        &ImportOptions::default(),
        &JobCtx::detached(),
    );
    // Sniffing needs the whole first record, which the 134-byte file has.
    let i = i.expect("read");
    assert!(
        i.report
            .warnings
            .iter()
            .any(|w| matches!(w, IoWarning::ExtensionMismatch { .. })),
        "{:?}",
        i.report.warnings
    );
}

#[test]
fn q25_triangle_limit() {
    let f = binary(b"x", 3, &[T0, T1, T2], &[]);
    let o = ImportOptions {
        limits: ReadLimits {
            max_triangles: 2,
            ..ReadLimits::default()
        },
        ..ImportOptions::default()
    };
    assert!(matches!(
        run_with(&f, &o),
        Err(IoError::LimitExceeded {
            what: "triangles",
            ..
        })
    ));
}

#[test]
fn q26_cancelled_before_start() {
    let token = amg_jobs::CancelToken::new();
    token.cancel();
    let f = binary(b"x", 1, &[T0], &[]);
    let r = import(
        Source::Bytes(&f),
        Some(Format::Stl),
        &ImportOptions::default(),
        &JobCtx::with_token(token),
    );
    assert!(matches!(r, Err(IoError::Cancelled)));
}

#[test]
fn registry_reports_stl_read_and_write() {
    let info = amg_io::registry()
        .iter()
        .find(|i| i.format == Format::Stl)
        .expect("stl");
    assert!(info.can_import && info.can_export);
}

#[test]
fn export_is_memory_or_path_and_flavours_stay_unsupported() {
    let f = binary(b"x", 1, &[T0], &[]);
    let i = run(&f).expect("read");
    let mut out = Vec::new();
    let r = export(
        &i.scene,
        Sink::Memory(&mut out),
        Format::Stl,
        &ExportOptions::default(),
        &JobCtx::detached(),
    )
    .expect("write");
    assert_eq!((r.triangles, r.bytes), (1, 134));
    assert_eq!(out.len(), 134);
    assert!(!out.starts_with(b"solid"));
    // Other formats still say so.
    let e = export(
        &i.scene,
        Sink::Memory(&mut Vec::new()),
        Format::Obj,
        &ExportOptions::default(),
        &JobCtx::detached(),
    );
    assert!(matches!(e, Err(IoError::Unsupported { .. })));
}
