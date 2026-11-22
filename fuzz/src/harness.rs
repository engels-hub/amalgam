//! IO-16 fuzz target bodies (S12 §12, note `import-export-formats.md` §9.3).
//!
//! Every function takes one raw fuzz input and panics when it finds a bug. The fuzz targets
//! in `fuzz/fuzz_targets/` call them, and `crates/amg-io/tests/fuzz_replay.rs` compiles this
//! file by path to replay the committed artifacts on stable, so it only uses crates that are
//! normal dependencies of `amg-io`.
//!
//! Oracles:
//! - no panic, arithmetic overflow (fuzz builds have debug assertions), OOM or timeout;
//! - a successful import gives a [`Scene`] that passes [`Scene::validate`], with finite
//!   positions and transforms;
//! - a read with tiny chunks (the parallel chunk-boundary paths) agrees with the default read;
//! - the 3MF scanner agrees with the pure quick-xml reference path;
//! - `roundtrip_scene`: export → import gives the scene back (IO-11 Done-when 1).

use std::sync::{Arc, Once};

use amg_core::units::LengthUnit;
use amg_core::{Transform, Vec3};
use amg_io::{
    Compression, CoordPrecision, ExportOptions, Format, ImportOptions, Imported, IoError,
    MaterialSlot, NO_SLOT, NodeContent, ObjGroups, ObjectKind, ReadLimits, Scene, SceneNode,
    SceneObject, Sink, Source, StlWeld, UnitChoice, UpAxis, ValidityPolicy, export, import, obj,
    paint, ply, stl, threemf,
};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, Rgba8};

/// A target body.
pub type Target = fn(&[u8]);

/// Every target, by the name of its `fuzz_targets/<name>.rs` and its artifact directory
/// `testdata/io/fuzz/<name>/`.
pub const TARGETS: &[(&str, Target)] = &[
    ("stl", stl),
    ("obj", obj),
    ("mtl", mtl),
    ("ply", ply),
    ("threemf_package", threemf_package),
    ("threemf_model", threemf_model),
    ("paint_decode", paint_decode),
    ("roundtrip_scene", roundtrip_scene),
];

/// One rayon thread: deterministic, and libFuzzer's RSS accounting stays per input.
fn init() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        // Already built (another test in the same process) is fine.
        let _ = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build_global();
    });
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

// ------------------------------------------------------------------------------- checks

/// What every successful import must satisfy.
fn check_scene(scene: &Scene, what: &str) {
    if let Err(e) = scene.validate() {
        panic!("{what}: the reader returned an invalid scene: {e}");
    }
    for (k, o) in scene.objects.iter().enumerate() {
        assert!(
            o.mesh
                .positions
                .iter()
                .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()),
            "{what}: object {k} has a non-finite position"
        );
    }
    for (i, t) in scene.instances() {
        assert!(
            t.to_cols_array().iter().all(|x| x.is_finite()),
            "{what}: an instance of object {i} has a non-finite transform"
        );
    }
}

fn check_import(r: &Result<Imported, IoError>, what: &str) {
    if let Ok(i) = r {
        check_scene(&i.scene, what);
    }
}

/// Everything a read produces except the wall time.
fn fingerprint(i: &Imported) -> String {
    let r = &i.report;
    format!(
        "{:?}\n{:?} {:?} {} {} {} {:?}",
        i.scene,
        r.format,
        r.declared_unit,
        r.triangles_read,
        r.vertices,
        r.degenerate_triangles,
        r.warnings
    )
}

/// The default read and a tiny-chunk read must agree.
fn same_read(a: &Result<Imported, IoError>, b: &Result<Imported, IoError>, what: &str) {
    match (a, b) {
        (Ok(x), Ok(y)) => {
            let (fx, fy) = (fingerprint(x), fingerprint(y));
            assert!(
                fx == fy,
                "{what}: chunked read differs\n--- default\n{fx}\n--- chunked\n{fy}"
            );
        }
        (Err(_), Err(_)) => {}
        (Ok(_), Err(e)) => panic!("{what}: only the chunked read failed: {e}"),
        (Err(e), Ok(_)) => panic!("{what}: only the default read failed: {e}"),
    }
}

/// Options for a second import that takes the other code paths: merged objects, Y-up and a
/// non-mm fallback unit.
fn variant_opts() -> ImportOptions {
    ImportOptions {
        unit: UnitChoice::FromFile {
            fallback: LengthUnit::Inch,
        },
        up_axis: UpAxis::Y,
        merge_objects: true,
        limits: ReadLimits::default(),
        ..ImportOptions::default()
    }
}

fn import_as(data: &[u8], format: Format, opts: &ImportOptions) -> Result<Imported, IoError> {
    import(Source::Bytes(data), Some(format), opts, &ctx())
}

/// Sniffing and probing take any bytes.
fn probe(data: &[u8]) {
    let _ = amg_io::probe(Source::Bytes(data), &ReadLimits::default());
    let _ = Format::sniff(data, data.len() as u64);
}

// ------------------------------------------------------------------------------ readers

/// STL: detection, both parsers, the exact weld and the streaming chunk boundaries.
pub fn stl(data: &[u8]) {
    init();
    probe(data);
    let head = &data[..data.len().min(4096)];
    let _ = stl::detect(head, data.len() as u64);
    let o = ImportOptions::default();
    check_import(&import_as(data, Format::Stl, &o), "stl");
    let mut v = variant_opts();
    v.stl.split_solids = true;
    v.stl.face_colors = true;
    check_import(&import_as(data, Format::Stl, &v), "stl variant");
    let mut soup = ImportOptions::default();
    soup.stl.weld = StlWeld::None;
    check_import(&import_as(data, Format::Stl, &soup), "stl soup");

    let src = Source::Bytes(data);
    let a = stl::read(src, &o, &ctx());
    let b = stl::read_tuned(src, &o, &ctx(), stl::Tuning { chunk_bytes: 61 });
    same_read(&a, &b, "stl");
}

/// OBJ: the two-pass parser, polygons, groups and the block boundaries.
pub fn obj(data: &[u8]) {
    init();
    probe(data);
    let o = ImportOptions::default();
    check_import(&import_as(data, Format::Obj, &o), "obj");
    let mut v = variant_opts();
    v.obj.groups_as = ObjGroups::FaceGroups;
    check_import(&import_as(data, Format::Obj, &v), "obj variant");

    let src = Source::Bytes(data);
    let a = obj::read(src, &o, &ctx());
    let b = obj::read_tuned(
        src,
        &o,
        &ctx(),
        obj::Tuning {
            chunk_bytes: 61,
            min_block: 7,
        },
    );
    same_read(&a, &b, "obj");
}

/// MTL: names, colours and line numbers.
pub fn mtl(data: &[u8]) {
    init();
    let m = obj::parse_mtl(data);
    let mut last = 0;
    for mat in &m.materials {
        assert!(mat.line >= 1 && mat.line >= last, "mtl: line {}", mat.line);
        last = mat.line;
        assert!(
            // MTL/OBJ whitespace is space, tab and CR (not form feed).
            mat.name.trim_matches([' ', '\t', '\r']) == mat.name && !mat.name.contains('\n'),
            "mtl: name {:?} is not trimmed",
            mat.name
        );
    }
    assert!(obj::parse_mtl(data) == m, "mtl: not deterministic");
}

/// PLY: the header, ASCII and both binary byte orders, and the chunk/block boundaries.
pub fn ply(data: &[u8]) {
    init();
    probe(data);
    let o = ImportOptions::default();
    check_import(&import_as(data, Format::Ply, &o), "ply");
    check_import(
        &import_as(data, Format::Ply, &variant_opts()),
        "ply variant",
    );

    let src = Source::Bytes(data);
    let a = ply::read(src, &o, &ctx());
    let b = ply::read_tuned(
        src,
        &o,
        &ctx(),
        ply::Tuning {
            chunk_bytes: 7,
            block_records: 3,
        },
    );
    same_read(&a, &b, "ply");
}

/// The 3MF reader on a whole package, in normal and strict mode, with the scanner checked
/// against the quick-xml reference path.
fn threemf_read(pkg: &[u8], what: &str) {
    let o = ImportOptions::default();
    check_import(&import_as(pkg, Format::ThreeMf, &o), what);
    let mut v = variant_opts();
    v.threemf.keep_components = true;
    check_import(&import_as(pkg, Format::ThreeMf, &v), what);
    let mut strict = ImportOptions::default();
    strict.threemf.strict = true;
    check_import(&import_as(pkg, Format::ThreeMf, &strict), what);

    let src = Source::Bytes(pkg);
    let fast = threemf::read_with(
        src,
        &o,
        &ctx(),
        &threemf::Tuning {
            chunk_bytes: 64,
            reference_only: false,
        },
    );
    let reference = threemf::read_with(
        src,
        &o,
        &ctx(),
        &threemf::Tuning {
            chunk_bytes: 64,
            reference_only: true,
        },
    );
    let default = threemf::read(src, &o, &ctx());
    same_read(&default, &fast, what);
    same_read(&default, &reference, what);
}

/// A whole 3MF package (zip, OPC, relationships, then the models).
pub fn threemf_package(data: &[u8]) {
    init();
    probe(data);
    threemf_read(data, "threemf_package");
}

const CONTENT_TYPES: &str = concat!(
    r#"<?xml version="1.0" encoding="UTF-8"?>"#,
    r#"<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">"#,
    r#"<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>"#,
    r#"<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>"#,
    r#"<Default Extension="config" ContentType="text/xml"/>"#,
    "</Types>"
);

const RELS: &str = concat!(
    r#"<?xml version="1.0" encoding="UTF-8"?>"#,
    r#"<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">"#,
    r#"<Relationship Target="/3D/3dmodel.model" Id="rel0" "#,
    r#"Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>"#,
    "</Relationships>"
);

/// Wraps model XML into a minimal valid package: `[Content_Types].xml`, `_rels/.rels` and
/// `3D/3dmodel.model`, stored.
#[must_use]
pub fn stub_package(model: &[u8]) -> Vec<u8> {
    use std::io::Write as _;
    let mut zw = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
    let opt = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Stored)
        .last_modified_time(zip::DateTime::default());
    let mut put = |name: &str, bytes: &[u8]| {
        zw.start_file(name, opt).expect("stub zip entry");
        zw.write_all(bytes).expect("stub zip write");
    };
    put("[Content_Types].xml", CONTENT_TYPES.as_bytes());
    put("_rels/.rels", RELS.as_bytes());
    put("3D/3dmodel.model", model);
    zw.finish().expect("stub zip").into_inner()
}

/// One model part's XML inside a stub package: the XML reader, the scanner, materials,
/// components and slicer attributes, without the zip layer in the way.
pub fn threemf_model(data: &[u8]) {
    init();
    threemf_read(&stub_package(data), "threemf_model");
}

// -------------------------------------------------------------------------------- paint

const HEX: &[u8; 16] = b"0123456789ABCDEF";

/// The `paint_color` / `mmu_segmentation` decoder (note §4).
pub fn paint_decode(data: &[u8]) {
    init();
    let s = String::from_utf8_lossy(data);
    if let Ok((state, _)) = paint::decode_dominant(&s) {
        assert!(state <= paint::MAX_STATE, "paint: state {state}");
    }
    // The same bytes as hex digits always reach the tree decoder.
    let hex: String = data
        .iter()
        .map(|b| char::from(HEX[usize::from(b & 15)]))
        .collect();
    match paint::decode_dominant(&hex) {
        Ok((state, _)) => assert!(state <= paint::MAX_STATE, "paint: state {state}"),
        Err(e) => assert!(e.pos <= hex.len(), "paint: error position {}", e.pos),
    }
    // Whole-triangle states round-trip.
    if let Some(&b) = data.first() {
        let st = b % (paint::MAX_STATE + 1);
        assert_eq!(
            paint::decode_dominant(&paint::encode_state(st)),
            Ok((st, false)),
            "paint: state {st}"
        );
    }
}

// ----------------------------------------------------------------------- roundtrip_scene

/// Reads the fuzz input as a stream of choices; past the end every choice is 0.
struct Choices<'a> {
    data: &'a [u8],
    at: usize,
}

impl Choices<'_> {
    fn u8(&mut self) -> u8 {
        let v = self.data.get(self.at).copied().unwrap_or(0);
        self.at += 1;
        v
    }
    fn u16(&mut self) -> u16 {
        u16::from_le_bytes([self.u8(), self.u8()])
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes([self.u8(), self.u8(), self.u8(), self.u8()])
    }
    /// `0..n` (`n > 0`).
    fn below(&mut self, n: u32) -> u32 {
        if n <= 256 {
            u32::from(self.u8()) % n
        } else {
            self.u32() % n
        }
    }
    /// True with probability about `percent` %.
    fn chance(&mut self, percent: u8) -> bool {
        self.u8() % 100 < percent
    }
    /// A coordinate in mm, |x| < 2.1e6, with exact zeros, integers, fine fractions and tiny
    /// values. (Larger magnitudes cannot meet the 1e-6 mm `Decimals(6)` bound, see IO-11.)
    fn coord(&mut self) -> f64 {
        let raw = f64::from(self.u32().cast_signed());
        match self.u8() % 5 {
            0 => 0.0,
            1 => f64::from(self.u16().cast_signed()),
            2 => raw / 1024.0,
            3 => raw * 1e-9,
            _ => raw / 3.0e3,
        }
    }
    /// Any UTF-8 text, up to 15 bytes of input.
    fn text(&mut self) -> String {
        let n = usize::from(self.u8() % 16);
        let bytes: Vec<u8> = (0..n).map(|_| self.u8()).collect();
        String::from_utf8_lossy(&bytes).into_owned()
    }
    fn name(&mut self) -> Option<String> {
        self.chance(80).then(|| self.text())
    }
}

fn mesh(c: &mut Choices<'_>) -> IndexedMesh {
    let nv = 3 + c.below(30);
    let positions = (0..nv)
        .map(|_| amg_core::Point3::new(c.coord(), c.coord(), c.coord()))
        .collect();
    let nt = 1 + c.below(40);
    let tris = (0..nt)
        .map(|_| {
            let a = c.below(nv);
            if c.chance(3) {
                // A repeated index (degenerate face): readers keep these (note §9.2).
                return [a, a, c.below(nv)];
            }
            let b = (a + 1 + c.below(nv - 1)) % nv;
            let (lo, hi) = (a.min(b), a.max(b));
            let mut x = c.below(nv - 2);
            if x >= lo {
                x += 1;
            }
            if x >= hi {
                x += 1;
            }
            [a, b, x]
        })
        .collect();
    IndexedMesh::new(positions, tris).expect("indices are in range")
}

fn transform(c: &mut Choices<'_>) -> Transform {
    let axis = Vec3::new(
        f64::from(c.u8()) - 127.5,
        f64::from(c.u8()) - 127.5,
        f64::from(c.u8()) - 127.5,
    )
    .normalize();
    let angle = (f64::from(c.u16()) / 65535.0 - 0.5) * std::f64::consts::TAU;
    let rot = Transform::from_axis_angle(axis, angle);
    let scale = match c.u8() % 8 {
        0 => Transform::from_scale(Vec3::splat(0.5 + f64::from(c.u8()) / 170.0)),
        1 => Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0)),
        2 => Transform::from_scale(Vec3::new(
            0.25 + f64::from(c.u8()) / 64.0,
            0.25 + f64::from(c.u8()) / 64.0,
            0.25 + f64::from(c.u8()) / 64.0,
        )),
        _ => Transform::IDENTITY,
    };
    let t = Transform::from_translation(Vec3::new(
        f64::from(c.u16().cast_signed()) / 64.0,
        f64::from(c.u16().cast_signed()) / 64.0,
        f64::from(c.u16().cast_signed()) / 64.0,
    ));
    t * rot * scale
}

fn node(c: &mut Choices<'_>, scene: &Scene, depth: u32) -> SceneNode {
    if depth < 3 && c.chance(30) {
        let n = 1 + c.below(3);
        let children = (0..n).map(|_| node(c, scene, depth + 1)).collect();
        let mut g = SceneNode::group(transform(c), children);
        g.name = c.name();
        g
    } else {
        let i = c.below(scene.objects.len() as u32);
        let mut m = SceneNode::mesh(i, transform(c));
        // 3MF has no instance names: a placed object carries the object's name.
        m.name = scene.objects[i as usize].name.clone();
        m
    }
}

/// A scene from the input: objects of every written kind, open or closed meshes, materials,
/// per-face slots, filaments, nested groups, mirrored and scaled transforms, any UTF-8
/// names.
#[must_use]
pub fn scene_from(data: &[u8]) -> Scene {
    let c = &mut Choices { data, at: 0 };
    let mut s = Scene::default();
    for _ in 0..c.below(5) {
        s.materials.push(MaterialSlot {
            name: c.text(),
            color: Rgba8([c.u8(), c.u8(), c.u8(), c.u8()]),
            filament: None,
        });
    }
    let nslots = s.materials.len() as u32;
    for _ in 0..1 + c.below(4) {
        let m = mesh(c);
        let n = m.face_count();
        let mut o = SceneObject::new(Arc::new(m));
        o.name = c.name();
        // `Other` is not written (IO-11 deviation 7).
        o.kind = match c.u8() % 8 {
            0 => ObjectKind::SolidSupport,
            1 => ObjectKind::Support,
            2 => ObjectKind::Surface,
            _ => ObjectKind::Model,
        };
        if nslots > 0 && c.chance(70) {
            o.material = Some(c.below(nslots) as u16);
        }
        if nslots > 0 && c.chance(50) {
            let faces: Vec<u16> = (0..n)
                .map(|_| {
                    if c.chance(50) {
                        NO_SLOT
                    } else {
                        c.below(nslots) as u16
                    }
                })
                .collect();
            o.face_materials = Some(faces.into());
        }
        o.filament = c.chance(40).then(|| 1 + c.u8() % 255);
        s.objects.push(o);
    }
    for _ in 0..1 + c.below(4) {
        let n = node(c, &s, 0);
        s.roots.push(n);
    }
    if c.chance(50) {
        s.metadata.insert("Title".into(), c.text());
    }
    if c.chance(50) {
        s.metadata.insert("Designer".into(), c.text());
    }
    s
}

fn bits(x: f64) -> u64 {
    if x == 0.0 { 0 } else { x.to_bits() }
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
    let (fa, fb) = (n.transform.to_cols_array(), m.transform.to_cols_array());
    for (x, y) in fa.iter().zip(&fb) {
        assert!((x - y).abs() <= 1e-12, "{what}: transform {fa:?} vs {fb:?}");
    }
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

/// IO-11 Done-when 1: placements, objects, slots, names and metadata come back.
fn scene_eq(a: &Scene, b: &Scene, exact: bool) {
    assert_eq!(a.roots.len(), b.roots.len(), "root count");
    for (k, (n, m)) in a.roots.iter().zip(&b.roots).enumerate() {
        node_eq(a, n, b, m, exact, &format!("root {k}"));
    }
    for key in ["Title", "Designer"] {
        assert_eq!(a.metadata.get(key), b.metadata.get(key), "metadata {key}");
    }
}

fn write_read(scene: &Scene, opts: &ExportOptions) -> Scene {
    let mut out = Vec::new();
    if let Err(e) = export(scene, Sink::Memory(&mut out), Format::ThreeMf, opts, &ctx()) {
        panic!("export failed: {e}\n{scene:?}");
    }
    match import(
        Source::Bytes(&out),
        Some(Format::ThreeMf),
        &ImportOptions::default(),
        &ctx(),
    ) {
        Ok(i) => {
            check_scene(&i.scene, "roundtrip_scene");
            i.scene
        }
        Err(e) => panic!("re-import failed: {e}\n{scene:?}"),
    }
}

/// Text as the 3MF writer stores it: characters XML 1.0 cannot carry are dropped (IO-11
/// `escape`).
fn xml_text(s: &str) -> String {
    s.chars()
        .filter(|&c| {
            !((c as u32) < 0x20 && !matches!(c, '\t' | '\n' | '\r'))
                && !matches!(c, '\u{FFFE}' | '\u{FFFF}')
        })
        .collect()
}

fn normalise_node(n: &mut SceneNode) {
    n.name = n.name.as_deref().map(xml_text);
    if let NodeContent::Group(c) = &mut n.content {
        c.iter_mut().for_each(normalise_node);
    }
}

/// What a 3MF round trip keeps of `scene` by design (IO-11): XML-safe text, and no empty
/// metadata values (the writer skips them).
#[must_use]
pub fn expected_after_3mf(scene: &Scene) -> Scene {
    let mut s = scene.clone();
    for m in &mut s.materials {
        m.name = xml_text(&m.name);
    }
    for o in &mut s.objects {
        o.name = o.name.as_deref().map(xml_text);
    }
    // An object with a face slot but no default slot gets the writer's synthetic grey
    // "Default" base material as its default (IO-11 deviation 4).
    let synthetic = MaterialSlot {
        name: "Default".into(),
        color: Rgba8([128, 128, 128, 255]),
        filament: None,
    };
    let index = s.materials.len() as u16;
    let mut used = false;
    for o in &mut s.objects {
        let has_faces = o
            .face_materials
            .as_ref()
            .is_some_and(|f| f.iter().any(|&s| s != NO_SLOT));
        if o.material.is_none() && has_faces {
            o.material = Some(index);
            used = true;
        }
    }
    if used {
        s.materials.push(synthetic);
    }
    s.roots.iter_mut().for_each(normalise_node);
    s.metadata = s
        .metadata
        .iter()
        .map(|(k, v)| (k.clone(), xml_text(v)))
        .filter(|(_, v)| !v.is_empty())
        .collect();
    s
}

/// Scene → 3MF (`Decimals(6)` and `Roundtrip`) → scene, compared as in IO-11 Done-when 1.
pub fn roundtrip_scene(data: &[u8]) {
    init();
    let generated = scene_from(data);
    let scene = expected_after_3mf(&generated);
    let compression = match data.last().copied().unwrap_or(0) % 3 {
        0 => Compression::Stored,
        1 => Compression::Deflate { level: 1 },
        _ => Compression::Deflate { level: 9 },
    };
    let mut six = ExportOptions {
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    };
    six.threemf.compression = compression;
    scene_eq(&scene, &write_read(&generated, &six), false);
    let rt = ExportOptions {
        precision: CoordPrecision::Roundtrip,
        ..six
    };
    scene_eq(&scene, &write_read(&generated, &rt), true);
}
