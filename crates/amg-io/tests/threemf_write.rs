//! IO-11: the Generic 3MF writer (S12 §6).
//!
//! - Done-when 1: 50 generated scenes round-trip write → read (`Decimals(6)` within 1e-6 mm,
//!   `Roundtrip` bit-exact; identical indices, names, slots, filaments, transforms within 1e-12).
//! - Done-when 2: every fixture passes `scripts/io/validate_3mf.py` (lib3mf strict + XSD) when
//!   `AMG_3MF_PYTHON` names a Python with `lib3mf` and `xmlschema` (required with
//!   `AMG_REQUIRE_3MF_VALIDATOR=1`); the fixtures also pass our own strict reader.
//! - Done-when 3: byte-identical output across runs and with 1 and 16 threads.
//! - Done-when 4: a non-watertight object fails with `InvalidForFormat` (object and edge counts);
//!   `AllowInvalid` writes it with a warning.
//! - Done-when 5 (manual): `slicer_check_fixture` writes the 3-object, 2-material file to
//!   `AMG_IO11_FIXTURE_DIR` when set.
#![allow(clippy::unwrap_used, clippy::expect_used, reason = "tests")]

use std::io::Read;
use std::sync::Arc;

use amg_core::units::LengthUnit;
use amg_core::{Point3, Transform, Vec3};
use amg_io::{
    Compression, CoordPrecision, ExportOptions, Format, ImportOptions, IoError, IoWarning,
    MaterialSlot, NO_SLOT, NodeContent, ObjectKind, Scene, SceneNode, SceneObject, Sink, Source,
    Thumbnail, ValidityPolicy, export, import,
};
use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::geodesic_sphere;
use amg_mesh::{IndexedMesh, Rgba8};
use proptest::prelude::*;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn write(scene: &Scene, opts: &ExportOptions) -> Vec<u8> {
    let mut out = Vec::new();
    export(scene, Sink::Memory(&mut out), Format::ThreeMf, opts, &ctx()).unwrap();
    out
}

fn read(bytes: &[u8]) -> Scene {
    import(
        Source::Bytes(bytes),
        Some(Format::ThreeMf),
        &ImportOptions::default(),
        &ctx(),
    )
    .unwrap()
    .scene
}

fn read_strict(bytes: &[u8]) -> Scene {
    let mut o = ImportOptions::default();
    o.threemf.strict = true;
    import(Source::Bytes(bytes), Some(Format::ThreeMf), &o, &ctx())
        .unwrap()
        .scene
}

// ------------------------------------------------------------------------------ scene builders

/// Small deterministic generator (splitmix64).
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.unit()
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    fn chance(&mut self, p: f64) -> bool {
        self.unit() < p
    }
}

/// An axis-aligned box: 8 vertices, 12 outward triangles.
fn cuboid(size: [f64; 3]) -> IndexedMesh {
    let [x, y, z] = size;
    let p = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(x, 0.0, 0.0),
        Point3::new(x, y, 0.0),
        Point3::new(0.0, y, 0.0),
        Point3::new(0.0, 0.0, z),
        Point3::new(x, 0.0, z),
        Point3::new(x, y, z),
        Point3::new(0.0, y, z),
    ];
    let t = vec![
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
    ];
    IndexedMesh::new(p, t).unwrap()
}

/// A closed mesh with "awkward" coordinates: a geodesic sphere stretched and moved by a
/// positive-determinant affine map.
fn blob(r: &mut Rng) -> IndexedMesh {
    let mut m = geodesic_sphere(1 + r.below(3) as u32, r.range(1.0, 30.0), None);
    let s = [r.range(0.3, 3.0), r.range(0.3, 3.0), r.range(0.3, 3.0)];
    let o = [
        r.range(-50.0, 50.0),
        r.range(-50.0, 50.0),
        r.range(-50.0, 50.0),
    ];
    for p in &mut m.positions {
        p.x = p.x * s[0] + o[0];
        p.y = p.y * s[1] + o[1];
        p.z = p.z * s[2] + o[2];
    }
    m
}

fn rotation(r: &mut Rng) -> Transform {
    let axis = Vec3::new(
        r.range(-1.0, 1.0),
        r.range(-1.0, 1.0),
        r.range(-1.0, 1.0) + 2.0,
    );
    let rot = Transform::from_axis_angle(axis.normalize(), r.range(-3.0, 3.0));
    let s = if r.chance(0.3) {
        Transform::from_scale(Vec3::splat(r.range(0.5, 2.0)))
    } else {
        Transform::IDENTITY
    };
    let t = Transform::from_translation(Vec3::new(
        r.range(-200.0, 200.0),
        r.range(-200.0, 200.0),
        r.range(0.0, 100.0),
    ));
    t * rot * s
}

const NAMES: [&str; 6] = [
    "Part",
    "Bracket & <clip>",
    "Halter \"groß\"",
    "  spaced  ",
    "日本語",
    "a'b",
];

fn node(r: &mut Rng, scene: &Scene, depth: u32) -> SceneNode {
    if depth < 2 && r.chance(0.35) {
        let n = 1 + r.below(3) as usize;
        let children = (0..n).map(|_| node(r, scene, depth + 1)).collect();
        let mut g = SceneNode::group(rotation(r), children);
        g.name = r
            .chance(0.7)
            .then(|| format!("Group {}", NAMES[r.below(6) as usize]));
        g
    } else {
        let i = r.below(scene.objects.len() as u64) as u32;
        let mut m = SceneNode::mesh(i, rotation(r));
        m.name = scene.objects[i as usize].name.clone();
        m
    }
}

/// A random scene: objects, groups, materials, per-face slots, transforms, filaments.
fn random_scene(seed: u64) -> Scene {
    let r = &mut Rng(seed);
    let mut s = Scene::default();
    for k in 0..r.below(4) {
        s.materials.push(MaterialSlot {
            name: format!("{} {k}", NAMES[r.below(6) as usize]),
            color: Rgba8([
                r.below(256) as u8,
                r.below(256) as u8,
                r.below(256) as u8,
                if r.chance(0.3) {
                    r.below(256) as u8
                } else {
                    255
                },
            ]),
            filament: None,
        });
    }
    let nslots = s.materials.len() as u64;
    for _ in 0..1 + r.below(4) {
        let mesh = blob(r);
        let n = mesh.face_count();
        let mut o = SceneObject::new(Arc::new(mesh));
        o.name = r.chance(0.8).then(|| NAMES[r.below(6) as usize].to_owned());
        if r.chance(0.15) {
            o.kind = ObjectKind::SolidSupport;
        }
        if nslots > 0 && r.chance(0.7) {
            o.material = Some(r.below(nslots) as u16);
            if r.chance(0.6) {
                let faces: Vec<u16> = (0..n)
                    .map(|_| {
                        if r.chance(0.5) {
                            NO_SLOT
                        } else {
                            r.below(nslots) as u16
                        }
                    })
                    .collect();
                o.face_materials = Some(faces.into());
            }
        }
        o.filament = r.chance(0.4).then(|| 1 + r.below(8) as u8);
        s.objects.push(o);
    }
    for _ in 0..1 + r.below(4) {
        let n = node(r, &s, 0);
        s.roots.push(n);
    }
    if r.chance(0.5) {
        s.metadata
            .insert("Title".into(), format!("Scene {seed} & <x>"));
        s.metadata.insert("Designer".into(), "Lane G".into());
    }
    s
}

// ------------------------------------------------------------------------------- comparisons

fn bits(x: f64) -> u64 {
    if x == 0.0 { 0 } else { x.to_bits() }
}

fn transform_close(a: &Transform, b: &Transform, what: &str) {
    let fa = a.to_cols_array();
    let fb = b.to_cols_array();
    for (x, y) in fa.iter().zip(&fb) {
        assert!((x - y).abs() <= 1e-12, "{what}: transform {fa:?} vs {fb:?}");
    }
}

fn slot(s: &Scene, i: Option<u16>) -> Option<MaterialSlot> {
    i.filter(|&i| i != NO_SLOT)
        .map(|i| s.materials[usize::from(i)].clone())
}

fn resolved_faces(s: &Scene, o: &SceneObject) -> Vec<Option<MaterialSlot>> {
    (0..o.mesh.face_count())
        .map(|f| {
            let own = o.face_materials.as_ref().map_or(NO_SLOT, |v| v[f]);
            slot(
                s,
                if own == NO_SLOT {
                    o.material
                } else {
                    Some(own)
                },
            )
        })
        .collect()
}

fn object_eq(a: &Scene, x: &SceneObject, b: &Scene, y: &SceneObject, exact: bool, what: &str) {
    assert_eq!(x.name, y.name, "{what}: name");
    assert_eq!(x.kind, y.kind, "{what}: kind");
    assert_eq!(x.filament, y.filament, "{what}: filament");
    assert_eq!(x.mesh.tris, y.mesh.tris, "{what}: indices");
    assert_eq!(
        x.mesh.positions.len(),
        y.mesh.positions.len(),
        "{what}: vertex count"
    );
    for (p, q) in x.mesh.positions.iter().zip(&y.mesh.positions) {
        for (u, v) in p.to_array().iter().zip(q.to_array()) {
            if exact {
                assert_eq!(bits(*u), bits(v), "{what}: {p:?} vs {q:?}");
            } else {
                assert!((u - v).abs() <= 1e-6, "{what}: {p:?} vs {q:?}");
            }
        }
    }
    assert_eq!(
        slot(a, x.material),
        slot(b, y.material),
        "{what}: default slot"
    );
    assert_eq!(
        resolved_faces(a, x),
        resolved_faces(b, y),
        "{what}: face slots"
    );
}

fn node_eq(a: &Scene, n: &SceneNode, b: &Scene, m: &SceneNode, exact: bool, what: &str) {
    assert_eq!(n.name, m.name, "{what}: node name");
    transform_close(&n.transform, &m.transform, what);
    match (&n.content, &m.content) {
        (NodeContent::Mesh(i), NodeContent::Mesh(j)) => object_eq(
            a,
            &a.objects[*i as usize],
            b,
            &b.objects[*j as usize],
            exact,
            what,
        ),
        (NodeContent::Group(c), NodeContent::Group(d)) => {
            assert_eq!(c.len(), d.len(), "{what}: children");
            for (k, (x, y)) in c.iter().zip(d).enumerate() {
                node_eq(a, x, b, y, exact, &format!("{what}/{k}"));
            }
        }
        _ => panic!("{what}: mesh vs group"),
    }
}

fn scene_eq(a: &Scene, b: &Scene, exact: bool) {
    assert_eq!(a.roots.len(), b.roots.len(), "root count");
    for (k, (n, m)) in a.roots.iter().zip(&b.roots).enumerate() {
        node_eq(a, n, b, m, exact, &format!("root {k}"));
    }
    for key in ["Title", "Designer"] {
        assert_eq!(a.metadata.get(key), b.metadata.get(key), "metadata {key}");
    }
    assert_eq!(
        b.metadata.get("Application"),
        Some(&amg_io::threemf::application())
    );
}

// -------------------------------------------------------------------------------- Done-when 1

proptest! {
    #![proptest_config(ProptestConfig { cases: 50, ..ProptestConfig::default() })]

    #[test]
    fn generated_scenes_round_trip(seed in any::<u64>()) {
        let scene = random_scene(seed);
        let six = write(&scene, &ExportOptions::default());
        scene_eq(&scene, &read(&six), false);
        let rt = ExportOptions { precision: CoordPrecision::Roundtrip, ..ExportOptions::default() };
        let exact = write(&scene, &rt);
        scene_eq(&scene, &read(&exact), true);
        // Our own strict reader accepts what we write.
        scene_eq(&scene, &read_strict(&exact), true);
    }
}

#[test]
fn faces_without_a_default_get_a_neutral_default_base() {
    let mut s = Scene::default();
    s.materials.push(MaterialSlot {
        name: "Red".into(),
        color: Rgba8([255, 0, 0, 255]),
        filament: None,
    });
    let mut o = SceneObject::new(Arc::new(cuboid([10.0, 10.0, 10.0])));
    let mut faces = vec![NO_SLOT; 12];
    faces[3] = 0;
    o.face_materials = Some(faces.into());
    s.objects.push(o);
    s.roots.push(SceneNode::mesh(0, Transform::IDENTITY));
    let back = read_strict(&write(&s, &ExportOptions::default()));
    let o = &back.objects[0];
    assert_eq!(slot(&back, o.material).unwrap().name, "Default");
    let f = resolved_faces(&back, o);
    assert_eq!(f[3].as_ref().unwrap().name, "Red");
    assert!(
        f.iter()
            .enumerate()
            .all(|(k, s)| k == 3 || s.as_ref().unwrap().name == "Default")
    );
}

#[test]
fn units_and_precision_warnings() {
    let s = random_scene(7);
    let opts = ExportOptions {
        unit: LengthUnit::Inch,
        precision: CoordPrecision::Roundtrip,
        ..ExportOptions::default()
    };
    let bytes = write(&s, &opts);
    let model = entry(&bytes, "3D/3dmodel.model");
    assert!(model.contains("unit=\"inch\""));
    let back = read(&bytes);
    let (a, b) = (s.instances(), back.instances());
    assert_eq!(a.len(), b.len());
    for ((i, ta), (j, tb)) in a.iter().zip(&b) {
        let (x, y) = (&s.objects[*i as usize], &back.objects[*j as usize]);
        for (p, q) in x.mesh.positions.iter().zip(&y.mesh.positions) {
            assert!((*p - *q).length() <= 1e-9 * (1.0 + p.length()));
        }
        assert!(
            (ta.translation - tb.translation).length() <= 1e-9 * (1.0 + ta.translation.length())
        );
    }
    // Two decimals of an inch: a precision-loss warning.
    let mut report_opts = opts.clone();
    report_opts.precision = CoordPrecision::Decimals(2);
    let mut out = Vec::new();
    let r = export(
        &s,
        Sink::Memory(&mut out),
        Format::ThreeMf,
        &report_opts,
        &ctx(),
    )
    .unwrap();
    assert!(
        r.warnings
            .iter()
            .any(|w| matches!(w, IoWarning::PrecisionLoss { .. }))
    );
}

#[test]
fn non_core_metadata_and_foreign_application_are_not_written() {
    let mut s = random_scene(11);
    s.metadata
        .insert("Application".into(), "BambuStudio-02.06.00.51".into());
    s.metadata.insert("OrcaSlicer".into(), "2.4.0".into());
    s.metadata
        .insert("BambuStudio:3mfVersion".into(), "1".into());
    let mut out = Vec::new();
    let r = export(
        &s,
        Sink::Memory(&mut out),
        Format::ThreeMf,
        &ExportOptions::default(),
        &ctx(),
    )
    .unwrap();
    let model = entry(&out, "3D/3dmodel.model");
    assert!(model.contains(&format!(
        "<metadata name=\"Application\">{}</metadata>",
        amg_io::threemf::application()
    )));
    assert!(!model.contains("Bambu") && !model.contains("OrcaSlicer"));
    assert!(
        r.warnings
            .iter()
            .any(|w| matches!(w, IoWarning::UnsupportedContentIgnored { count: 2, .. }))
    );
}

#[test]
fn thumbnails_are_downscaled_png_with_the_package_relationship() {
    let mut s = random_scene(3);
    let (w, h) = (1024u32, 768u32);
    let px: Vec<u8> = (0..w * h)
        .flat_map(|i| [(i % 256) as u8, 40, 200, 255])
        .collect();
    s.thumbnail = Some(Thumbnail::Rgba8 {
        width: w,
        height: h,
        pixels: px.into(),
    });
    let bytes = write(&s, &ExportOptions::default());
    let rels = entry(&bytes, "_rels/.rels");
    assert!(rels.contains("/Metadata/thumbnail.png"));
    assert!(entry(&bytes, "[Content_Types].xml").contains("image/png"));
    let back = read_strict(&bytes);
    let Some(Thumbnail::Png(png)) = &back.thumbnail else {
        panic!("thumbnail")
    };
    let d = png::Decoder::new(std::io::Cursor::new(png.to_vec()));
    let r = d.read_info().unwrap();
    assert_eq!((r.info().width, r.info().height), (512, 384));
    // PNG input is kept byte for byte.
    s.thumbnail = Some(Thumbnail::Png(png.clone()));
    let again = read(&write(&s, &ExportOptions::default()));
    assert!(matches!(&again.thumbnail, Some(Thumbnail::Png(p)) if p == png));
}

#[test]
fn flavours_other_than_generic_are_not_yet_written() {
    let s = random_scene(5);
    let mut o = ExportOptions::default();
    o.threemf.flavor = amg_io::SlicerFlavor::Orca;
    let mut out = Vec::new();
    let e = export(&s, Sink::Memory(&mut out), Format::ThreeMf, &o, &ctx()).unwrap_err();
    assert!(matches!(e, IoError::Unsupported { .. }));
    assert!(out.is_empty());
}

#[test]
fn non_finite_numbers_are_refused_before_writing() {
    let mut s = random_scene(9);
    let mut m = (*s.objects[0].mesh).clone();
    m.positions[0].x = f64::NAN;
    s.objects[0].mesh = Arc::new(m);
    let o = ExportOptions {
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    };
    let mut out = Vec::new();
    let e = export(&s, Sink::Memory(&mut out), Format::ThreeMf, &o, &ctx()).unwrap_err();
    assert!(matches!(e, IoError::InvalidScene { .. }), "{e:?}");
    assert!(out.is_empty());
}

// -------------------------------------------------------------------------------- Done-when 3

fn big_scene() -> Scene {
    let mut s = random_scene(42);
    let sphere = geodesic_sphere(120, 25.0, None); // 288 000 triangles
    let n = sphere.face_count();
    let mut o = SceneObject::new(Arc::new(sphere));
    o.name = Some("big".into());
    s.materials.push(MaterialSlot {
        name: "A".into(),
        color: Rgba8([1, 2, 3, 255]),
        filament: None,
    });
    s.materials.push(MaterialSlot {
        name: "B".into(),
        color: Rgba8([4, 5, 6, 255]),
        filament: None,
    });
    let a = (s.materials.len() - 2) as u16;
    o.material = Some(a);
    o.face_materials = Some(
        (0..n)
            .map(|f| if f % 7 == 0 { a + 1 } else { NO_SLOT })
            .collect(),
    );
    s.objects.push(o);
    s.roots.push(SceneNode::mesh(
        (s.objects.len() - 1) as u32,
        rotation(&mut Rng(1)),
    ));
    s.thumbnail = Some(Thumbnail::Rgba8 {
        width: 64,
        height: 64,
        pixels: vec![200u8; 64 * 64 * 4].into(),
    });
    s
}

#[test]
fn output_is_byte_identical_across_runs_and_thread_counts() {
    let s = big_scene();
    let opts = ExportOptions::default();
    let on = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| write(&s, &opts))
    };
    let one = on(1);
    let sixteen = on(16);
    let again = on(16);
    assert!(one.len() > 1_000_000);
    assert!(one == sixteen, "1 vs 16 threads differ");
    assert!(sixteen == again, "two runs differ");
    let stored = ExportOptions {
        threemf: amg_io::ThreeMfWriteOptions {
            compression: Compression::Stored,
            ..amg_io::ThreeMfWriteOptions::default()
        },
        ..ExportOptions::default()
    };
    assert!(on(1) == one);
    let st1 = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap()
        .install(|| write(&s, &stored));
    let st16 = rayon::ThreadPoolBuilder::new()
        .num_threads(16)
        .build()
        .unwrap()
        .install(|| write(&s, &stored));
    assert!(st1 == st16);
}

#[test]
fn package_layout_is_fixed() {
    let bytes = write(&big_scene(), &ExportOptions::default());
    let mut z = zip::ZipArchive::new(std::io::Cursor::new(&bytes)).unwrap();
    let names: Vec<String> = (0..z.len())
        .map(|i| z.by_index(i).unwrap().name().unwrap().into_owned())
        .collect();
    assert_eq!(
        names,
        [
            "[Content_Types].xml",
            "_rels/.rels",
            "3D/3dmodel.model",
            "Metadata/thumbnail.png"
        ]
    );
    for i in 0..z.len() {
        let f = z.by_index(i).unwrap();
        let t = f.last_modified().unwrap();
        assert_eq!(
            (t.year(), t.month(), t.day(), t.hour(), t.minute()),
            (1980, 1, 1, 0, 0)
        );
        assert_eq!(f.compression(), zip::CompressionMethod::Deflated);
    }
    // Production UUIDs on build, items, objects and components: present and unique.
    let model = entry(
        &write(&grouped_scene(), &ExportOptions::default()),
        "3D/3dmodel.model",
    );
    let uuids: Vec<&str> = model.split("p:UUID=\"").skip(1).map(|r| &r[..36]).collect();
    let distinct: std::collections::BTreeSet<_> = uuids.iter().collect();
    assert!(
        uuids.len() >= 10 && distinct.len() == uuids.len(),
        "{uuids:?}"
    );
    assert!(!model.contains("requiredextensions"));
    // Plain zip: no ZIP64 end-of-central-directory record.
    assert!(!bytes.windows(4).any(|w| w == b"PK\x06\x06"));
}

// -------------------------------------------------------------------------------- Done-when 4

fn open_box_scene() -> Scene {
    let mut m = cuboid([10.0, 20.0, 5.0]);
    m.tris.pop();
    m.groups = amg_mesh::FaceGroups::new(m.tris.len());
    let mut o = SceneObject::new(Arc::new(m));
    o.name = Some("open box".into());
    let mut ok = SceneObject::new(Arc::new(cuboid([1.0, 1.0, 1.0])));
    ok.name = Some("closed box".into());
    let mut s = Scene::default();
    s.objects.push(ok);
    s.objects.push(o);
    s.roots.push(SceneNode::mesh(0, Transform::IDENTITY));
    s.roots.push(SceneNode::mesh(
        1,
        Transform::from_translation(Vec3::new(30.0, 0.0, 0.0)),
    ));
    s
}

#[test]
fn non_watertight_objects_fail_with_the_counts() {
    let s = open_box_scene();
    let mut out = Vec::new();
    let e = export(
        &s,
        Sink::Memory(&mut out),
        Format::ThreeMf,
        &ExportOptions::default(),
        &ctx(),
    )
    .unwrap_err();
    let IoError::InvalidForFormat {
        format,
        count,
        check,
    } = e
    else {
        panic!("expected InvalidForFormat, got {e:?}")
    };
    assert_eq!((format, count), (Format::ThreeMf, 1));
    let bad: Vec<_> = check.objects.iter().filter(|o| !o.is_valid()).collect();
    assert_eq!(bad.len(), 1);
    assert_eq!(bad[0].name.as_deref(), Some("open box"));
    assert_eq!(bad[0].boundary_edges, 3);
    assert_eq!(bad[0].non_manifold_edges, 0);
    assert!(out.is_empty(), "nothing written");
    assert!(e_message_names_it(&s));

    for policy in [ValidityPolicy::AllowInvalid, ValidityPolicy::Warn] {
        let opts = ExportOptions {
            validity: Some(policy),
            ..ExportOptions::default()
        };
        let mut out = Vec::new();
        let r = export(&s, Sink::Memory(&mut out), Format::ThreeMf, &opts, &ctx()).unwrap();
        assert!(r.warnings.iter().any(|w| matches!(
            w,
            IoWarning::NotManifold { object, boundary_edges: 3, non_manifold_edges: 0 }
                if object == "open box"
        )));
        let back = read(&out);
        assert_eq!(back.objects.len(), 2);
        assert!(entry(&out, "3D/3dmodel.model").contains("name=\"open box\""));
        assert!(!entry(&out, "3D/3dmodel.model").contains("type=\"surface\""));
    }
}

fn e_message_names_it(s: &Scene) -> bool {
    let c = amg_io::check_export(s, Format::ThreeMf, &ctx()).unwrap();
    c.warnings.iter().any(|w| w.message().contains("open box"))
}

#[test]
fn only_model_and_solidsupport_objects_must_be_closed() {
    let mut s = open_box_scene();
    s.objects[1].kind = ObjectKind::Surface;
    let out = write(&s, &ExportOptions::default());
    assert!(entry(&out, "3D/3dmodel.model").contains("type=\"surface\""));
    s.objects[1].kind = ObjectKind::SolidSupport;
    let mut buf = Vec::new();
    assert!(matches!(
        export(
            &s,
            Sink::Memory(&mut buf),
            Format::ThreeMf,
            &ExportOptions::default(),
            &ctx()
        ),
        Err(IoError::InvalidForFormat { .. })
    ));
}

// ------------------------------------------------------------------------------- fixtures

/// The 3-object, 2-material scene for the manual slicer check (Done-when 5): a red sphere, a
/// blue box and a bar with blue ends on red, standing on the bed in a row along x.
fn slicer_check_scene() -> Scene {
    let mut s = Scene::default();
    s.metadata
        .insert("Title".into(), "IO-11 slicer check".into());
    s.materials.push(MaterialSlot {
        name: "PLA Red".into(),
        color: Rgba8([0xD9, 0x3B, 0x2B, 255]),
        filament: None,
    });
    s.materials.push(MaterialSlot {
        name: "PLA Blue".into(),
        color: Rgba8([0x2B, 0x6C, 0xD9, 255]),
        filament: None,
    });
    let mut sphere = SceneObject::new(Arc::new(geodesic_sphere(16, 10.0, None)));
    sphere.name = Some("Red sphere".into());
    sphere.material = Some(0);
    sphere.filament = Some(1);
    let mut cube = SceneObject::new(Arc::new(cuboid([20.0, 20.0, 20.0])));
    cube.name = Some("Blue cube".into());
    cube.material = Some(1);
    cube.filament = Some(2);
    let bar_mesh = cuboid([40.0, 10.0, 10.0]);
    // End caps (faces 6,7 at x = 40 and 10,11 at x = 0) blue, the rest red.
    let faces: Vec<u16> = (0..12)
        .map(|f| {
            if matches!(f, 6 | 7 | 10 | 11) {
                1
            } else {
                NO_SLOT
            }
        })
        .collect();
    let mut bar = SceneObject::new(Arc::new(bar_mesh));
    bar.name = Some("Two-colour bar".into());
    bar.material = Some(0);
    bar.face_materials = Some(faces.into());
    bar.filament = Some(1);
    s.objects.extend([sphere, cube, bar]);
    let at = |x: f64, y: f64, z: f64| Transform::from_translation(Vec3::new(x, y, z));
    s.roots.push(SceneNode::mesh(0, at(80.0, 128.0, 10.0)));
    s.roots.push(SceneNode::mesh(1, at(110.0, 118.0, 0.0)));
    s.roots.push(SceneNode::mesh(2, at(150.0, 123.0, 0.0)));
    for (k, r) in s.roots.iter_mut().enumerate() {
        r.name = s.objects[k].name.clone();
    }
    s
}

/// Grouped (components) variant: two copies of one part in an assembly, plus a support.
fn grouped_scene() -> Scene {
    let mut s = slicer_check_scene();
    let mut sup = SceneObject::new(Arc::new(cuboid([5.0, 5.0, 2.0])));
    sup.name = Some("support pad".into());
    sup.kind = ObjectKind::Support;
    s.objects.push(sup);
    let child =
        |i: u32, x: f64| SceneNode::mesh(i, Transform::from_translation(Vec3::new(x, 0.0, 0.0)));
    let mut g = SceneNode::group(
        Transform::from_translation(Vec3::new(100.0, 60.0, 0.0)),
        vec![child(1, 0.0), child(1, 30.0), child(3, 60.0)],
    );
    g.name = Some("Assembly".into());
    s.roots.push(g);
    s
}

/// Every fixture the validator checks, with its options.
fn fixtures() -> Vec<(&'static str, Scene, ExportOptions)> {
    let d = ExportOptions::default();
    let rt = ExportOptions {
        precision: CoordPrecision::Roundtrip,
        ..ExportOptions::default()
    };
    let inch = ExportOptions {
        unit: LengthUnit::Inch,
        ..ExportOptions::default()
    };
    let stored = ExportOptions {
        threemf: amg_io::ThreeMfWriteOptions {
            compression: Compression::Stored,
            ..amg_io::ThreeMfWriteOptions::default()
        },
        ..ExportOptions::default()
    };
    let mut thumb = slicer_check_scene();
    thumb.thumbnail = Some(Thumbnail::Rgba8 {
        width: 300,
        height: 200,
        pixels: (0..300 * 200)
            .flat_map(|i| [(i % 251) as u8, (i % 13) as u8 * 19, 128, 255])
            .collect::<Vec<u8>>()
            .into(),
    });
    let mut plain = Scene::default();
    plain
        .objects
        .push(SceneObject::new(Arc::new(cuboid([10.0, 10.0, 10.0]))));
    plain.roots.push(SceneNode::mesh(0, Transform::IDENTITY));
    let mut v = vec![
        ("io11-plain-cube.3mf", plain, d.clone()),
        ("io11-slicer-check.3mf", slicer_check_scene(), d.clone()),
        ("io11-grouped.3mf", grouped_scene(), d.clone()),
        ("io11-roundtrip.3mf", random_scene(1), rt),
        ("io11-inch.3mf", random_scene(2), inch),
        ("io11-stored.3mf", random_scene(3), stored),
        ("io11-thumbnail.3mf", thumb, d.clone()),
        ("io11-big.3mf", big_scene(), d.clone()),
    ];
    for k in 0..8u64 {
        let name: &'static str = Box::leak(format!("io11-random-{k}.3mf").into_boxed_str());
        v.push((name, random_scene(1000 + k), d.clone()));
    }
    v
}

fn entry(zip_bytes: &[u8], name: &str) -> String {
    let mut z = zip::ZipArchive::new(std::io::Cursor::new(zip_bytes)).unwrap();
    let mut s = String::new();
    z.by_name(name).unwrap().read_to_string(&mut s).unwrap();
    s
}

#[test]
fn fixtures_pass_the_strict_reader() {
    for (name, scene, opts) in fixtures() {
        let bytes = write(&scene, &opts);
        let back = read_strict(&bytes);
        assert_eq!(
            back.instances().len(),
            scene
                .instances()
                .iter()
                .filter(|(i, _)| scene.objects[*i as usize].kind != ObjectKind::Other)
                .count(),
            "{name}"
        );
    }
}

/// Done-when 2: `scripts/io/validate_3mf.py` (lib3mf strict, no warnings, plus the Consortium
/// consolidated XSD) on every fixture.
#[test]
fn fixtures_pass_validate_3mf_py() {
    let python = std::env::var_os("AMG_3MF_PYTHON");
    let required = std::env::var("AMG_REQUIRE_3MF_VALIDATOR").is_ok_and(|v| v == "1");
    let Some(python) = python else {
        assert!(
            !required,
            "AMG_REQUIRE_3MF_VALIDATOR=1 but AMG_3MF_PYTHON is not set"
        );
        eprintln!("skipped: set AMG_3MF_PYTHON to a Python with lib3mf and xmlschema");
        return;
    };
    let dir = tempfile::tempdir().unwrap();
    let mut files = Vec::new();
    for (name, scene, opts) in fixtures() {
        let p = dir.path().join(name);
        export(&scene, Sink::Path(&p), Format::ThreeMf, &opts, &ctx()).unwrap();
        files.push(p);
    }
    let script =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scripts/io/validate_3mf.py");
    let out = std::process::Command::new(python)
        .arg("-I")
        .arg(&script)
        .args(&files)
        .output()
        .unwrap();
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    eprintln!("{text}");
    assert!(out.status.success(), "validate_3mf.py failed:\n{text}");
    assert_eq!(
        text.matches("OK ").count(),
        files.len(),
        "every file reported OK:\n{text}"
    );
}

/// Done-when 5 (manual): writes the slicer-check fixture to `AMG_IO11_FIXTURE_DIR`.
#[test]
fn slicer_check_fixture() {
    let scene = slicer_check_scene();
    let bytes = write(&scene, &ExportOptions::default());
    let back = read_strict(&bytes);
    assert_eq!(back.objects.len(), 3);
    assert_eq!(back.materials.len(), 2);
    let names: Vec<_> = back
        .objects
        .iter()
        .map(|o| o.name.clone().unwrap())
        .collect();
    assert_eq!(names, ["Red sphere", "Blue cube", "Two-colour bar"]);
    if let Some(d) = std::env::var_os("AMG_IO11_FIXTURE_DIR") {
        let d = std::path::PathBuf::from(d);
        std::fs::create_dir_all(&d).unwrap();
        for (name, scene, opts) in fixtures()
            .into_iter()
            .filter(|(n, ..)| matches!(*n, "io11-slicer-check.3mf" | "io11-grouped.3mf"))
        {
            let p = d.join(name);
            export(&scene, Sink::Path(&p), Format::ThreeMf, &opts, &ctx()).unwrap();
        }
    }
}

#[test]
fn path_sink_is_atomic_and_reports_the_file() {
    let dir = tempfile::tempdir().unwrap();
    let p = dir.path().join("out.3mf");
    let r = export(
        &slicer_check_scene(),
        Sink::Path(&p),
        Format::ThreeMf,
        &ExportOptions::default(),
        &ctx(),
    )
    .unwrap();
    assert_eq!(r.files, std::slice::from_ref(&p));
    assert_eq!(r.bytes, std::fs::metadata(&p).unwrap().len());
    assert_eq!(r.triangles, 20 * 16 * 16 + 12 + 12);
    assert!(!dir.path().join("out.3mf.tmp").exists());
    assert!(
        amg_io::registry()
            .iter()
            .any(|f| f.format == Format::ThreeMf && f.can_export)
    );
}

#[test]
fn cancellation_stops_the_write() {
    let s = big_scene();
    let token = amg_jobs::CancelToken::new();
    token.cancel();
    let c = JobCtx::with_token(token.clone());
    let mut out = Vec::new();
    let e = export(
        &s,
        Sink::Memory(&mut out),
        Format::ThreeMf,
        &ExportOptions::default(),
        &c,
    )
    .unwrap_err();
    assert!(matches!(e, IoError::Cancelled));
    assert!(out.is_empty());
}
