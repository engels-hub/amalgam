//! IO-13 Done-when 2: the quirk cases of the PLY reader (S12 note §6, §9.2), each as an
//! individual test, plus the registry/dispatch checks.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test code; a failed expectation is a test failure"
)]

mod ply_common;

use amg_core::units::LengthUnit;
use amg_io::{
    Format, ImportOptions, IoError, IoWarning, NO_SLOT, ReadLimits, Source, UnitChoice, import,
};
use ply_common::*;

fn tris(i: &amg_io::Imported) -> Vec<[u32; 3]> {
    i.scene.objects[0].mesh.tris.clone()
}

fn assert_tetra(i: &amg_io::Imported) {
    let m = &i.scene.objects[0].mesh;
    assert_eq!((m.vertex_count(), m.face_count()), (4, 4));
    assert_eq!(m.tris, TETRA_F.to_vec());
    for (p, v) in m.positions.iter().zip(TETRA_V) {
        assert_eq!(p.to_array(), v.map(f64::from));
    }
    assert!((m.signed_volume().abs() - 1.0 / 6.0).abs() < 1e-12);
    assert!(i.report.warnings.is_empty(), "{:?}", i.report.warnings);
    assert_eq!(i.report.format, Some(Format::Ply));
    assert_eq!(i.report.triangles_read, 4);
}

// 1
#[test]
fn big_endian_binary() {
    for (c, x) in [("uchar", "int"), ("ushort", "uint"), ("uint", "int")] {
        assert_tetra(&run(&binary_tetra(true, c, x)).expect(c));
    }
}

// 2
#[test]
fn ascii_body() {
    assert_tetra(&run(&ascii_tetra()).expect("ascii"));
}

// 3, 4, 5
#[test]
fn list_counts_uchar_ushort_uint() {
    for c in ["uchar", "ushort", "uint"] {
        assert_tetra(&run(&binary_tetra(false, c, "int")).expect(c));
    }
}

// 6
#[test]
fn index_types_int_uint_short_ushort_uchar() {
    for x in ["int", "uint", "short", "ushort", "uchar", "char"] {
        assert_tetra(&run(&binary_tetra(false, "uchar", x)).expect(x));
    }
}

// 7
#[test]
fn extra_elements_are_skipped_by_size() {
    for be in [false, true] {
        let mut out = format!(
            "ply\nformat {} 1.0\nelement material 2\nproperty uchar r\nproperty float shine\n\
             element vertex 4\nproperty float x\nproperty float y\nproperty float z\n\
             element edge 3\nproperty int a\nproperty int b\nproperty list uchar ushort tags\n\
             element face 4\nproperty list uchar int vertex_indices\n\
             element extra 2\nproperty double q\nend_header\n",
            if be {
                "binary_big_endian"
            } else {
                "binary_little_endian"
            }
        )
        .into_bytes();
        for _ in 0..2 {
            out.push(7);
            out.extend_from_slice(&f32b(be, 1.5));
        }
        for v in TETRA_V {
            for x in v {
                out.extend_from_slice(&f32b(be, x));
            }
        }
        for k in 0..3u32 {
            out.extend_from_slice(&u32b(be, k));
            out.extend_from_slice(&u32b(be, k + 1));
            out.push(2);
            out.extend_from_slice(&u16b(be, 9));
            out.extend_from_slice(&u16b(be, 8));
        }
        for f in TETRA_F {
            out.push(3);
            for i in f {
                out.extend_from_slice(&u32b(be, i));
            }
        }
        out.extend_from_slice(&f64b(be, 1.0));
        out.extend_from_slice(&f64b(be, 2.0));
        let i = run(&out).expect("extra elements");
        assert_eq!(tris(&i), TETRA_F.to_vec());
        let ignored: Vec<_> = i
            .report
            .warnings
            .iter()
            .filter_map(|w| match w {
                IoWarning::UnsupportedContentIgnored { what, count } => {
                    Some((what.clone(), *count))
                }
                _ => None,
            })
            .collect();
        assert_eq!(
            ignored,
            vec![
                ("PLY element 'material'".to_owned(), 2),
                ("PLY element 'edge'".to_owned(), 3),
                ("PLY element 'extra'".to_owned(), 2)
            ]
        );
        assert_eq!(i.report.warnings.len(), 3, "{:?}", i.report.warnings);
    }
}

fn colored(be: bool, ty: &str, alpha: bool) -> Vec<u8> {
    let mut out = format!(
        "ply\nformat {} 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\n\
         property {ty} red\nproperty {ty} green\nproperty {ty} blue\n{}\
         element face 1\nproperty list uchar int vertex_indices\nend_header\n",
        if be {
            "binary_big_endian"
        } else {
            "binary_little_endian"
        },
        if alpha {
            format!("property {ty} alpha\n")
        } else {
            String::new()
        }
    )
    .into_bytes();
    for (k, v) in TETRA_V.iter().enumerate() {
        for x in v {
            out.extend_from_slice(&f32b(be, *x));
        }
        let chans: [f32; 4] = [k as f32 / 3.0, 1.0 - k as f32 / 3.0, 0.5, 0.0];
        for c in chans.iter().take(if alpha { 4 } else { 3 }) {
            match ty {
                "uchar" => out.push((c * 255.0).round() as u8),
                "float" => out.extend_from_slice(&f32b(be, *c)),
                "ushort" => out.extend_from_slice(&u16b(be, (c * 65535.0).round() as u16)),
                _ => unreachable!(),
            }
        }
    }
    out.push(3);
    for i in [0u32, 1, 2] {
        out.extend_from_slice(&u32b(be, i));
    }
    out
}

// 8
#[test]
fn vertex_colors_uchar_with_and_without_alpha() {
    let i = run(&colored(false, "uchar", true)).expect("rgba");
    let c = i.scene.objects[0]
        .mesh
        .attrs
        .vertex_colors
        .clone()
        .expect("colours");
    assert_eq!(c[0].0, [0, 255, 128, 0]);
    assert_eq!(c[3].0, [255, 0, 128, 0]);
    let i = run(&colored(true, "uchar", false)).expect("rgb");
    let c = i.scene.objects[0]
        .mesh
        .attrs
        .vertex_colors
        .clone()
        .expect("colours");
    assert_eq!(c[1].0, [85, 170, 128, 255]);
    // The oracle agrees.
    let o = oracle(&colored(false, "uchar", true));
    assert_eq!(o.colors.expect("oracle colours")[3], [255, 0, 128, 0]);
}

// 9
#[test]
fn vertex_colors_float_and_ushort() {
    for ty in ["float", "ushort"] {
        let i = run(&colored(false, ty, true)).expect(ty);
        let c = i.scene.objects[0]
            .mesh
            .attrs
            .vertex_colors
            .clone()
            .expect("colours");
        assert_eq!(c[0].0, [0, 255, 128, 0], "{ty}");
        assert_eq!(c[3].0, [255, 0, 128, 0], "{ty}");
    }
}

// 10
#[test]
fn face_colors_become_slots() {
    let mut s = String::from(
        "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\n\
         element face 4\nproperty list uchar int vertex_indices\nproperty uchar red\n\
         property uchar green\nproperty uchar blue\nproperty uchar alpha\nend_header\n",
    );
    for v in TETRA_V {
        s.push_str(&format!("{} {} {}\n", v[0], v[1], v[2]));
    }
    let cols = [
        [255, 0, 0, 255],
        [0, 255, 0, 255],
        [255, 0, 0, 255],
        [1, 2, 3, 128],
    ];
    for (f, c) in TETRA_F.iter().zip(cols) {
        s.push_str(&format!(
            "3 {} {} {} {} {} {} {}\n",
            f[0], f[1], f[2], c[0], c[1], c[2], c[3]
        ));
    }
    let i = run(s.as_bytes()).expect("face colours");
    let o = &i.scene.objects[0];
    assert_eq!(&**o.face_materials.as_ref().expect("slots"), &[0, 1, 0, 2]);
    let names: Vec<_> = i.scene.materials.iter().map(|m| m.name.as_str()).collect();
    assert_eq!(names, ["#FF0000", "#00FF00", "#01020380"]);
    assert_eq!(i.scene.materials[2].color.0, [1, 2, 3, 128]);
    assert!(o.mesh.attrs.vertex_colors.is_none());
    assert_ne!(NO_SLOT, 0);
}

// 11
#[test]
fn no_faces_is_a_point_cloud() {
    let no_element =
        b"ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\n\
        property float z\nend_header\n0 0 0\n1 1 1\n";
    assert!(matches!(run(no_element), Err(IoError::PointCloud)));
    let zero_faces = b"ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\n\
        property float z\nelement face 0\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 1 1\n";
    assert!(matches!(run(zero_faces), Err(IoError::PointCloud)));
    let zero_vertices =
        b"ply\nformat ascii 1.0\nelement vertex 0\nproperty float x\nproperty float y\n\
        property float z\nend_header\n";
    assert!(matches!(run(zero_vertices), Err(IoError::Empty)));
}

// 12
#[test]
fn truncated_body() {
    let full = binary_tetra(false, "uchar", "int");
    // Cut in the middle of the third face: two whole faces survive.
    let body_start = full.windows(11).position(|w| w == b"end_header\n").unwrap() + 11;
    let faces_start = body_start + 4 * 12;
    let cut = faces_start + 13 * 2 + 5;
    let i = run(&full[..cut]).expect("truncated faces");
    assert_eq!(tris(&i), TETRA_F[..2].to_vec());
    assert!(
        i.report.warnings.contains(&IoWarning::Truncated {
            expected: 4,
            read: 2
        }),
        "{:?}",
        i.report.warnings
    );
    // Cut inside the vertices: nothing can reference them, so it is an error.
    let e = run(&full[..body_start + 30]).expect_err("truncated vertices");
    assert!(
        matches!(&e, IoError::Malformed { format: Format::Ply, message, .. } if message.contains("vertex")),
        "{e:?}"
    );
    // The same for ASCII.
    let a = ascii_tetra();
    let text = String::from_utf8(a).unwrap();
    let cut = text.rfind("3 1 2 3").unwrap();
    let i = run(&text.as_bytes()[..cut]).expect("ascii truncated faces");
    assert_eq!(i.scene.triangle_count(), 3);
    assert!(i.report.warnings.contains(&IoWarning::Truncated {
        expected: 4,
        read: 3
    }));
    // Cut in the middle of a face record: that face is not read.
    let i = run(&text.as_bytes()[..cut + 4]).expect("ascii cut mid-record");
    assert_eq!(i.scene.triangle_count(), 3);
}

// 13
#[test]
fn header_without_end_header() {
    let e =
        run(b"ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\n").expect_err("no end");
    assert!(
        matches!(
            e,
            IoError::Malformed {
                format: Format::Ply,
                ..
            }
        ),
        "{e:?}"
    );
    // A header that never ends and is longer than the limit.
    let mut big = b"ply\nformat ascii 1.0\n".to_vec();
    while big.len() < 100_000 {
        big.extend_from_slice(b"comment padding padding padding padding padding\n");
    }
    let e = run(&big).expect_err("header limit");
    assert!(
        matches!(
            e,
            IoError::LimitExceeded {
                what: "PLY header bytes",
                limit: 65_536,
                ..
            }
        ),
        "{e:?}"
    );
    // A long header that does end inside the limit is fine.
    let mut ok = b"ply\nformat ascii 1.0\n".to_vec();
    while ok.len() < 60_000 {
        ok.extend_from_slice(b"comment padding padding padding padding padding\n");
    }
    ok.extend_from_slice(&ascii_tetra()[b"ply\nformat ascii 1.0\n".len()..]);
    assert_eq!(run(&ok).expect("60 KB header").scene.triangle_count(), 4);
}

#[test]
fn bad_headers() {
    for (name, text) in [
        ("not ply", "solid x\nendsolid\n"),
        (
            "no format",
            "ply\nelement vertex 1\nproperty float x\nend_header\n",
        ),
        ("bad format", "ply\nformat utf16 1.0\nend_header\n"),
        ("bad version", "ply\nformat ascii 2.0\nend_header\n"),
        (
            "property first",
            "ply\nformat ascii 1.0\nproperty float x\nend_header\n",
        ),
        (
            "bad type",
            "ply\nformat ascii 1.0\nelement vertex 1\nproperty quad x\nend_header\n",
        ),
        (
            "bad count",
            "ply\nformat ascii 1.0\nelement vertex lots\nend_header\n",
        ),
        (
            "float list count",
            "ply\nformat ascii 1.0\nelement face 1\nproperty list float int v\nend_header\n",
        ),
        (
            "no xyz",
            "ply\nformat ascii 1.0\nelement vertex 1\nproperty float a\nelement face 0\nend_header\n1\n",
        ),
    ] {
        let e = run(text.as_bytes()).expect_err(name);
        assert!(
            matches!(
                e,
                IoError::Malformed {
                    format: Format::Ply,
                    ..
                }
            ),
            "{name}: {e:?}"
        );
    }
    assert!(matches!(run(b""), Err(IoError::Empty)));
}

// 14
#[test]
fn polygons_are_triangulated() {
    let text = "ply\nformat ascii 1.0\nelement vertex 8\nproperty float x\nproperty float y\n\
        property float z\nelement face 3\nproperty list uchar int vertex_indices\nend_header\n\
        0 0 0\n2 0 0\n2 2 0\n0 2 0\n3 0 0\n4 0 0\n4 1 0\n3.5 2 0\n\
        4 0 1 2 3\n3 4 5 6\n4 4 5 6 7\n";
    let i = run(text.as_bytes()).expect("polygons");
    let m = &i.scene.objects[0].mesh;
    assert_eq!(m.face_count(), 2 + 1 + 2);
    assert!(
        (m.area() - (4.0 + 0.5 + 1.0 + 0.25)).abs() < 1e-9,
        "{}",
        m.area()
    );
    assert!(i.report.warnings.contains(&IoWarning::PolygonTriangulated {
        polygons: 2,
        fallback_fan: 0
    }));
    // A concave quad is clipped correctly (a fan from the reflex corner would overlap).
    let concave = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\n\
        property float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n\
        0 0 0\n4 0 0\n1 1 0\n0 4 0\n4 0 1 2 3\n";
    let i = run(concave.as_bytes()).expect("concave");
    assert!((i.scene.objects[0].mesh.area() - 4.0).abs() < 1e-9);
}

// 15
#[test]
fn invalid_indices_are_dropped() {
    let text = "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\n\
        property float z\nelement face 5\nproperty list uchar int vertex_indices\nend_header\n\
        0 0 0\n1 0 0\n0 1 0\n\
        3 0 1 2\n3 0 1 3\n3 -1 1 2\n4 0 1 2 99\n2 0 1\n";
    let i = run(text.as_bytes()).expect("dropped");
    assert_eq!(tris(&i), vec![[0, 1, 2]]);
    assert!(
        i.report
            .warnings
            .contains(&IoWarning::InvalidIndexDropped(3))
    );
    assert!(
        i.report
            .warnings
            .contains(&IoWarning::UnsupportedContentIgnored {
                what: "PLY face with fewer than 3 corners".into(),
                count: 1
            })
    );
    // Every face invalid: nothing usable.
    let all_bad = "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\n\
        property float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n\
        0 0 0\n1 0 0\n0 1 0\n3 0 1 7\n";
    assert!(matches!(run(all_bad.as_bytes()), Err(IoError::Empty)));
}

// 16
#[test]
fn non_finite_vertices_drop_their_faces() {
    let text = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\n\
        property float z\nelement face 2\nproperty list uchar int vertex_indices\nend_header\n\
        0 0 0\n1 0 0\n0 1 0\nnan 0 0\n3 0 1 2\n3 0 1 3\n";
    let i = run(text.as_bytes()).expect("nan");
    assert_eq!(tris(&i), vec![[0, 1, 2]]);
    assert!(i.report.warnings.contains(&IoWarning::NonFiniteDropped(1)));
}

// 17
#[test]
fn texture_coordinates() {
    // Vertex s/t.
    let text = "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\n\
        property float z\nproperty float s\nproperty float t\nelement face 1\n\
        property list uchar int vertex_indices\nend_header\n\
        0 0 0 0 0\n1 0 0 1 0\n0 1 0 0 1\n3 0 1 2\n";
    let i = run(text.as_bytes()).expect("st");
    let uv = i.scene.objects[0]
        .mesh
        .attrs
        .corner_uvs
        .clone()
        .expect("uvs");
    assert_eq!(uv[0][1].to_array(), [1.0, 0.0]);
    assert_eq!(uv[0][2].to_array(), [0.0, 1.0]);
    // Face texcoord lists (MeshLab): per corner, also on a quad.
    let text = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\n\
        property float z\nelement face 2\nproperty list uchar int vertex_indices\n\
        property list uchar float texcoord\nend_header\n\
        0 0 0\n1 0 0\n1 1 0\n0 1 0\n\
        3 0 1 2 6 0 0 1 0 1 1\n4 0 1 2 3 8 0 0 1 0 1 1 0 1\n";
    let i = run(text.as_bytes()).expect("texcoord");
    let m = &i.scene.objects[0].mesh;
    assert_eq!(m.face_count(), 3);
    let uv = m.attrs.corner_uvs.clone().expect("uvs");
    assert_eq!(uv[0][2].to_array(), [1.0, 1.0]);
    // The quad's triangles take the uv of their own corners (here uv == xy).
    for (t, c) in m.tris[1..].iter().zip(&uv[1..]) {
        for k in 0..3 {
            let p = m.positions[t[k] as usize];
            assert_eq!(c[k].to_array(), [p.x, p.y]);
        }
    }
}

// 18
#[test]
fn crlf_line_endings() {
    let text = String::from_utf8(ascii_tetra())
        .unwrap()
        .replace('\n', "\r\n");
    assert_tetra(&run(text.as_bytes()).expect("crlf"));
    // Binary with a CRLF header.
    let b = binary_tetra(false, "uchar", "int");
    let split = b.windows(11).position(|w| w == b"end_header\n").unwrap() + 11;
    let head = String::from_utf8(b[..split].to_vec())
        .unwrap()
        .replace('\n', "\r\n");
    let mut crlf = head.into_bytes();
    crlf.extend_from_slice(&b[split..]);
    assert_tetra(&run(&crlf).expect("binary crlf"));
}

// 19
#[test]
fn double_positions_and_other_vertex_properties() {
    let be = true;
    let mut out = String::from(
        "ply\nformat binary_big_endian 1.0\nobj_info made by hand\nelement vertex 4\nproperty double x\n\
         property double y\nproperty double z\nproperty float nx\nproperty float ny\nproperty float nz\n\
         property int flags\nelement face 4\nproperty list uchar int vertex_index\nend_header\n",
    )
    .into_bytes();
    for v in TETRA_V {
        for x in v {
            out.extend_from_slice(&f64b(be, f64::from(x) + 1e-9));
        }
        for _ in 0..3 {
            out.extend_from_slice(&f32b(be, 0.0));
        }
        out.extend_from_slice(&u32b(be, 5));
    }
    for f in TETRA_F {
        out.push(3);
        for i in f {
            out.extend_from_slice(&u32b(be, i));
        }
    }
    let i = run(&out).expect("double");
    let m = &i.scene.objects[0].mesh;
    assert_eq!(m.positions[1].x, 1.0 + 1e-9);
    assert_eq!(m.tris, TETRA_F.to_vec());
    assert!(i.report.warnings.is_empty());
    // Integer coordinates are fine too.
    let ints = "ply\nformat ascii 1.0\nelement vertex 3\nproperty short x\nproperty short y\n\
        property uchar z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n\
        -3 4 5\n1 0 0\n0 1 0\n3 0 1 2\n";
    let i = run(ints.as_bytes()).expect("ints");
    assert_eq!(
        i.scene.objects[0].mesh.positions[0].to_array(),
        [-3.0, 4.0, 5.0]
    );
}

// 20
#[test]
fn trailing_bytes_are_reported() {
    let mut b = binary_tetra(false, "uchar", "int");
    b.extend_from_slice(&[0xAB; 7]);
    let i = run(&b).expect("trailing");
    assert_eq!(i.scene.triangle_count(), 4);
    assert_eq!(i.report.warnings, vec![IoWarning::TrailingBytes(7)]);
    let mut a = ascii_tetra();
    a.extend_from_slice(b"\n\n  \n");
    assert_tetra(&run(&a).expect("ascii whitespace is not trailing"));
    a.extend_from_slice(b"junk\n");
    assert_eq!(
        run(&a).expect("ascii junk").report.warnings,
        vec![IoWarning::TrailingBytes(4)]
    );
}

// 21
#[test]
fn faces_may_come_before_vertices() {
    let text = "ply\nformat ascii 1.0\nelement face 1\nproperty list uchar int vertex_indices\n\
        element vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n\
        3 0 1 2\n0 0 0\n1 0 0\n0 1 0\n";
    let i = run(text.as_bytes()).expect("order");
    assert_eq!(tris(&i), vec![[0, 1, 2]]);
    // Also with a polygon (triangulated once all vertices are known).
    let text = "ply\nformat ascii 1.0\nelement face 1\nproperty list uchar int vertex_indices\n\
        element vertex 4\nproperty float x\nproperty float y\nproperty float z\nend_header\n\
        4 0 1 2 3\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n";
    assert_eq!(
        run(text.as_bytes()).expect("quad").scene.triangle_count(),
        2
    );
}

// 22
#[test]
fn hostile_counts_do_not_allocate_or_hang() {
    let text = b"ply\nformat binary_little_endian 1.0\nelement vertex 4000000000\nproperty float x\n\
        property float y\nproperty float z\nelement face 4000000000\nproperty list uint uint vertex_indices\n\
        end_header\n0123456789";
    // Beyond the vertex limit.
    let e = run(text).expect_err("limit");
    assert!(
        matches!(
            e,
            IoError::LimitExceeded {
                what: "vertices",
                ..
            }
        ),
        "{e:?}"
    );
    let opts = ImportOptions {
        limits: ReadLimits {
            max_vertices: u64::MAX,
            max_triangles: u64::MAX,
            ..ReadLimits::default()
        },
        ..ImportOptions::default()
    };
    // Within the limits, the file simply ends.
    let e = run_with(text, &opts).expect_err("truncated");
    assert!(
        matches!(
            e,
            IoError::Malformed {
                format: Format::Ply,
                ..
            }
        ),
        "{e:?}"
    );
    // A list that claims 4 G corners.
    let mut b = String::from(
        "ply\nformat binary_little_endian 1.0\nelement vertex 3\nproperty float x\nproperty float y\n\
         property float z\nelement face 2\nproperty list uint int vertex_indices\nend_header\n"
    )
    .into_bytes();
    for _ in 0..9 {
        b.extend_from_slice(&f32b(false, 0.5));
    }
    b.extend_from_slice(&u32b(false, 3));
    for i in 0..3u32 {
        b.extend_from_slice(&u32b(false, i));
    }
    b.extend_from_slice(&u32b(false, 4_000_000_000));
    b.extend_from_slice(&[0; 16]);
    let i = run(&b).expect("one good face, then a lie");
    assert_eq!(i.scene.triangle_count(), 1);
    assert!(i.report.warnings.contains(&IoWarning::Truncated {
        expected: 2,
        read: 1
    }));
    // Triangle limit.
    let o = ImportOptions {
        limits: ReadLimits {
            max_triangles: 3,
            ..ReadLimits::default()
        },
        ..ImportOptions::default()
    };
    let e = run_with(&ascii_tetra(), &o).expect_err("triangle limit");
    assert!(
        matches!(
            e,
            IoError::LimitExceeded {
                what: "triangles",
                ..
            }
        ),
        "{e:?}"
    );
    // A value without end.
    let mut long = b"ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\n\
        property float z\nend_header\n"
        .to_vec();
    long.extend(std::iter::repeat_n(b'1', 100_000));
    assert!(matches!(run(&long), Err(IoError::Malformed { .. })));
    // Garbage where a number should be.
    let e = run(
        b"ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\n\
        property float z\nend_header\n1 two 3\n",
    )
    .expect_err("bad number");
    assert!(
        matches!(&e, IoError::Malformed { message, .. } if message.contains("two")),
        "{e:?}"
    );
}

// 23
#[test]
fn import_applies_unit_up_axis_and_sniffs_the_content() {
    let ascii = ascii_tetra();
    let opts = ImportOptions {
        unit: UnitChoice::Assume(LengthUnit::Inch),
        ..ImportOptions::default()
    };
    let i = import(Source::Bytes(&ascii), None, &opts, &ctx()).expect("sniffed");
    assert_eq!(i.report.format, Some(Format::Ply));
    assert_eq!(i.scene.objects[0].mesh.positions[1].x, 25.4);
    assert_eq!(i.report.applied_unit, Some(LengthUnit::Inch));
    assert!(Format::sniff(b"ply\r\nformat", 100) == Some(Format::Ply));
    // Y-up files get the rotation on the root.
    let y = ImportOptions {
        up_axis: amg_io::UpAxis::Y,
        ..ImportOptions::default()
    };
    let i = import(Source::Bytes(&ascii), Some(Format::Ply), &y, &ctx()).expect("up");
    assert_ne!(i.scene.roots[0].transform, amg_core::Transform::IDENTITY);
    let info = amg_io::registry()
        .iter()
        .find(|f| f.format == Format::Ply)
        .expect("ply in registry");
    assert!(info.can_import && info.can_export && info.colors);
}

// 24
#[test]
fn a_path_source_reads_the_same_as_memory() {
    let dir = tempfile::tempdir().expect("tempdir");
    let p = dir.path().join("t.ply");
    for bytes in [binary_tetra(true, "uchar", "int"), ascii_tetra()] {
        std::fs::write(&p, &bytes).expect("write");
        let a = import(Source::Path(&p), None, &ImportOptions::default(), &ctx()).expect("path");
        let b = run(&bytes).expect("memory");
        assert_eq!(a.scene.objects[0].mesh.tris, b.scene.objects[0].mesh.tris);
        assert_eq!(a.report.bytes, bytes.len() as u64);
    }
    let missing = dir.path().join("missing.ply");
    assert!(matches!(
        import(
            Source::Path(&missing),
            Some(Format::Ply),
            &ImportOptions::default(),
            &ctx()
        ),
        Err(IoError::Read { .. })
    ));
}
