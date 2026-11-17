//! IO-14: `io.import_mesh` in a real `Document` (S12 Done-when 1-5): one transaction per
//! import with undo/redo, exact unit edits that recompute dependants only, `.amg` save/load of
//! the baked payloads, `scene_from_document` back to the imported scene, and cache keys that
//! are stable across processes.
//!
//! Corpus files skip when they are un-fetched git-LFS pointers, unless
//! `AMG_REQUIRE_TESTDATA=1`.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, OnceLock};

use amg_core::Transform;
use amg_core::units::LengthUnit;
use amg_doc::{
    BodyGeom, BodyId, Command, CreatedId, Document, FeatureBox, FeatureId, SaveOptions, load, save,
};
use amg_features::io::{
    ExportPrep, ExportSelection, ImportChoices, ImportMeshFeature, ImportSource, import_commands,
    raw_options, scene_from_document,
};
use amg_features::mesh::MeshReverseNormalFeature;
use amg_io::{
    CoordPrecision, ExportOptions, Format, ImportOptions, Imported, Scene, Sink, Source,
    ValidityPolicy,
};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
use amg_mesh::IndexedMesh;

// ----------------------------------------------------------------------------- fixtures

const STL: &str = "mesh/real/thingi10k/132062.stl";
const THREE_MF: &str = "io/slicer/orca-three.3mf";

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn link() {
    amg_features::io::link();
    amg_features::mesh::link();
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

/// A corpus file's bytes (`None`: un-fetched LFS pointer, skipped).
fn corpus(rel: &str) -> Option<(PathBuf, Vec<u8>)> {
    let p = amg_testkit::testdata(rel)?;
    let bytes = std::fs::read(&p).unwrap();
    Some((p, bytes))
}

fn read(bytes: &[u8], format: Format, opts: &ImportOptions) -> Imported {
    amg_io::import(Source::Bytes(bytes), Some(format), opts, &ctx()).unwrap()
}

/// An OBJ made from the corpus STL (the mesh corpus has no OBJ): exact `Roundtrip` numbers.
fn obj_from(stl: &[u8]) -> Vec<u8> {
    let scene = read(stl, Format::Stl, &ImportOptions::default()).scene;
    let mut out = Vec::new();
    let opts = ExportOptions {
        precision: CoordPrecision::Roundtrip,
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    };
    amg_io::export(&scene, Sink::Memory(&mut out), Format::Obj, &opts, &ctx()).unwrap();
    out
}

/// Imports `bytes` into `doc` (raw read, then one transaction). Returns the feature.
fn import_into(doc: &mut Document, name: &str, format: Format, bytes: &[u8]) -> FeatureId {
    let opts = ImportOptions::default();
    let imported = read(bytes, format, &raw_options(&opts));
    let choices = ImportChoices::from_options(doc.design().root(), &opts, &imported.report);
    let source = ImportSource::from_bytes(name, format, imported.report.declared_unit, bytes);
    let plan = import_commands(imported, source, &choices).unwrap();
    plan.apply(doc).unwrap()
}

fn recompute(doc: &mut Document) {
    doc.recompute_blocking(jobs()).unwrap();
}

fn mesh(doc: &Document, id: BodyId) -> Arc<IndexedMesh> {
    match &doc.evaluated().body(id).unwrap().geom {
        BodyGeom::Mesh(m) => Arc::clone(m),
        other => panic!("{other:?}"),
    }
}

fn import_feature(doc: &Document, id: FeatureId) -> ImportMeshFeature {
    let (_, node) = doc.design().timeline().node(id).unwrap();
    node.feature
        .downcast_ref::<ImportMeshFeature>()
        .unwrap()
        .clone()
}

/// `(body, content_hash)` of every evaluated body of `origin`, in id order.
fn hashes(doc: &Document, origin: FeatureId) -> Vec<(BodyId, u64)> {
    doc.evaluated()
        .bodies()
        .filter(|(id, _)| id.origin == origin)
        .map(|(id, b)| match &b.geom {
            BodyGeom::Mesh(m) => (id, m.content_hash()),
            other => panic!("{other:?}"),
        })
        .collect()
}

struct TempFile(PathBuf);

impl TempFile {
    fn new() -> TempFile {
        static N: AtomicU32 = AtomicU32::new(0);
        TempFile(std::env::temp_dir().join(format!(
            "amg-features-io14-{}-{}.amg",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        )))
    }
}

impl Drop for TempFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
        let _ = std::fs::remove_file(self.0.with_extension("amg.bak"));
    }
}

// ----------------------------------------------------------------------------- Done-when 1

/// One case per format: file name, format, bytes, expected body count.
fn cases() -> Vec<(&'static str, Format, Vec<u8>, usize)> {
    let mut v = Vec::new();
    if let Some((_, stl)) = corpus(STL) {
        v.push(("132062.obj", Format::Obj, obj_from(&stl), 1));
        v.push(("132062.stl", Format::Stl, stl, 1));
    }
    if let Some((_, tmf)) = corpus(THREE_MF) {
        v.push(("orca-three.3mf", Format::ThreeMf, tmf, 3));
    }
    v
}

#[test]
fn import_is_one_feature_and_undo_redo_restore_it() {
    link();
    for (name, format, bytes, bodies) in cases() {
        let mut doc = Document::new("io14");
        let undo_before = doc.history().stack().applied();
        let fid = import_into(&mut doc, name, format, &bytes);
        assert_eq!(doc.design().timeline().len(), 1, "{name}");
        assert_eq!(
            doc.history().stack().applied(),
            undo_before + 1,
            "{name}: one undo entry"
        );
        assert_eq!(
            doc.history().undo_label(),
            Some(format!("Insert {name}").as_str())
        );
        assert_eq!(doc.design().baked_bodies().count(), bodies, "{name}");
        recompute(&mut doc);
        let before = hashes(&doc, fid);
        assert_eq!(before.len(), bodies, "{name}");
        for (id, _) in &before {
            assert!(
                doc.design().body_props(*id).name.is_some(),
                "{name}: body name"
            );
        }
        assert!(doc.undo());
        assert_eq!(doc.design().timeline().len(), 0, "{name}");
        assert_eq!(doc.design().baked_bodies().count(), 0, "{name}");
        recompute(&mut doc);
        assert_eq!(doc.evaluated().bodies().count(), 0, "{name}");
        assert!(doc.redo());
        recompute(&mut doc);
        assert_eq!(
            hashes(&doc, fid),
            before,
            "{name}: redo restores identical meshes"
        );
    }
}

#[test]
fn three_mf_bodies_keep_names_and_build_transforms() {
    link();
    let Some((_, tmf)) = corpus(THREE_MF) else {
        return;
    };
    let mut doc = Document::new("io14");
    let fid = import_into(&mut doc, "orca-three.3mf", Format::ThreeMf, &tmf);
    recompute(&mut doc);
    let f = import_feature(&doc, fid);
    let names: Vec<&str> = f.bodies.iter().map(|b| b.name.as_str()).collect();
    assert_eq!(names, ["97160.stl", "97162.stl", "217025.stl"]);
    // Instance transforms (build item · component), exactly as read: the file is in mm.
    let original = read(&tmf, Format::ThreeMf, &ImportOptions::default()).scene;
    let want = original.instances();
    assert_eq!(want.len(), f.bodies.len());
    for (b, (_, t)) in f.bodies.iter().zip(want) {
        assert_eq!(b.transform, t);
    }
}

// ----------------------------------------------------------------------------- Done-when 2

#[test]
fn unit_edit_rescales_exactly_and_recomputes_dependants_only() {
    link();
    let Some((_, stl)) = corpus(STL) else { return };
    let mut doc = Document::new("io14");
    let a = import_into(&mut doc, "a.stl", Format::Stl, &stl);
    let b = import_into(&mut doc, "b.obj", Format::Obj, &obj_from(&stl));
    let body_a = BodyId { origin: a, slot: 0 };
    let root = doc.design().root();
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: FeatureBox::new(MeshReverseNormalFeature {
                body: body_a,
                groups: None,
            }),
            context: root,
            name: None,
        })
        .unwrap();
    let CreatedId::Feature(rev) = out.created[0] else {
        panic!()
    };
    recompute(&mut doc);
    let mm = mesh(&doc, body_a).aabb();

    let mut f = import_feature(&doc, a);
    assert_eq!(f.unit, LengthUnit::Millimetre);
    f.unit = LengthUnit::Inch;
    doc.apply(Command::EditFeature {
        id: a,
        feature: FeatureBox::new(f),
    })
    .unwrap();
    recompute(&mut doc);
    let run = doc.last_run().unwrap();
    assert_eq!(
        run.computed,
        vec![a, rev],
        "only the import and its dependant recompute"
    );
    assert_eq!(
        run.cached,
        vec![b],
        "the unrelated import is served from the cache"
    );
    let inch = mesh(&doc, body_a).aabb();
    for (x, y) in [(mm.min, inch.min), (mm.max, inch.max)] {
        assert_eq!(y.x, x.x * 25.4);
        assert_eq!(y.y, x.y * 25.4);
        assert_eq!(y.z, x.z * 25.4);
    }
    // The raw payload was not touched: back to mm is the original mesh.
    let mut f = import_feature(&doc, a);
    f.unit = LengthUnit::Millimetre;
    doc.apply(Command::EditFeature {
        id: a,
        feature: FeatureBox::new(f),
    })
    .unwrap();
    recompute(&mut doc);
    assert_eq!(mesh(&doc, body_a).aabb(), mm);
}

// ----------------------------------------------------------------------------- Done-when 3

#[test]
fn save_load_restores_bit_identical_meshes() {
    link();
    let cases = cases();
    if cases.is_empty() {
        return;
    }
    let mut doc = Document::new("io14");
    let fids: Vec<FeatureId> = cases
        .iter()
        .map(|(name, format, bytes, _)| import_into(&mut doc, name, *format, bytes))
        .collect();
    recompute(&mut doc);
    let file = TempFile::new();
    save(&file.0, &doc.snapshot(), &SaveOptions::default()).unwrap();
    let mut reopened = load(&file.0).unwrap().into_document();
    for (id, b) in doc.design().baked_bodies() {
        let r = reopened.design().baked(id).expect("baked body survives");
        assert_eq!(r.blob, b.blob);
        let (Some(BodyGeom::Mesh(x)), Some(BodyGeom::Mesh(y))) = (&b.geom, &r.geom) else {
            panic!("mesh payload expected")
        };
        assert_eq!(x.positions, y.positions);
        assert_eq!(x.tris, y.tris);
        assert_eq!(x.content_hash(), y.content_hash());
    }
    recompute(&mut reopened);
    for fid in fids {
        let a = hashes(&doc, fid);
        let b = hashes(&reopened, fid);
        assert_eq!(a, b);
        for (id, _) in a {
            assert_eq!(mesh(&doc, id).positions, mesh(&reopened, id).positions);
            assert_eq!(
                doc.design().body_props(id).name,
                reopened.design().body_props(id).name
            );
        }
    }
}

// ----------------------------------------------------------------------------- Done-when 4

fn assert_transform_close(a: &Transform, b: &Transform, what: &str) {
    let d = (a.translation - b.translation).abs().max_element().max(
        (a.matrix3 - b.matrix3)
            .to_cols_array()
            .iter()
            .fold(0.0f64, |m, x| m.max(x.abs())),
    );
    assert!(d <= 1e-12, "{what}: transforms differ by {d}");
}

/// Same placed objects (mesh, world transform within 1e-12, default and per-face material
/// slots resolved through the material tables) in the same order.
fn assert_scene_eq(original: &Scene, got: &Scene) {
    let a = original.instances();
    let b = got.instances();
    assert_eq!(a.len(), b.len(), "instance count");
    let slot = |s: &Scene, i: Option<u16>| {
        i.filter(|&i| i != amg_io::NO_SLOT)
            .map(|i| s.materials[usize::from(i)].clone())
    };
    for (k, ((oa, ta), (ob, tb))) in a.iter().zip(&b).enumerate() {
        let (x, y) = (&original.objects[*oa as usize], &got.objects[*ob as usize]);
        assert_eq!(
            x.mesh.positions, y.mesh.positions,
            "instance {k}: object mesh"
        );
        assert_eq!(x.mesh.tris, y.mesh.tris, "instance {k}: object faces");
        assert_transform_close(ta, tb, &format!("instance {k}"));
        assert_eq!(
            slot(original, x.material),
            slot(got, y.material),
            "instance {k}: slot"
        );
        assert_eq!(x.filament, y.filament, "instance {k}: filament");
        let faces = |s: &Scene, o: &amg_io::SceneObject| {
            o.face_materials
                .as_ref()
                .map(|f| f.iter().map(|&i| slot(s, Some(i))).collect::<Vec<_>>())
        };
        assert_eq!(
            faces(original, x),
            faces(got, y),
            "instance {k}: face slots"
        );
    }
}

fn exported(doc: &Document) -> Scene {
    scene_from_document(
        doc.design(),
        doc.evaluated(),
        &ExportSelection::AllVisible,
        &ExportPrep::default(),
        &ctx(),
    )
    .unwrap()
}

#[test]
fn scene_from_document_reproduces_the_imported_3mf() {
    link();
    let Some((_, tmf)) = corpus(THREE_MF) else {
        return;
    };
    let original = read(&tmf, Format::ThreeMf, &ImportOptions::default()).scene;
    let mut doc = Document::new("io14");
    import_into(&mut doc, "orca-three.3mf", Format::ThreeMf, &tmf);
    recompute(&mut doc);
    let scene = exported(&doc);
    assert_eq!(scene.objects.len(), 3);
    assert_scene_eq(&original, &scene);
}

/// Re-export through the mesh writers that exist (PLY double, OBJ round-trip precision) and
/// read back: the same placed geometry as the original import.
#[test]
fn reexport_round_trips_through_ply_and_obj() {
    link();
    let Some((_, stl)) = corpus(STL) else { return };
    let original = read(&stl, Format::Stl, &ImportOptions::default()).scene;
    let mut doc = Document::new("io14");
    import_into(&mut doc, "132062.stl", Format::Stl, &stl);
    recompute(&mut doc);
    let scene = exported(&doc);
    assert_scene_eq(&original, &scene);
    let mut opts = ExportOptions {
        precision: CoordPrecision::Roundtrip,
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    };
    opts.ply.double = true;
    for format in [Format::Ply, Format::Obj] {
        let mut out = Vec::new();
        amg_io::export(&scene, Sink::Memory(&mut out), format, &opts, &ctx()).unwrap();
        let back = read(&out, format, &ImportOptions::default()).scene;
        assert_scene_eq(&original, &back);
    }
}

/// Done-when 4 as written: re-export through the IO-11 3MF writer and read back. IO-11 (the
/// 3MF writer) is not on this base yet (`amg_io::export` answers `Unsupported` for 3MF);
/// IO-11 must un-ignore this test.
#[test]
#[ignore = "needs the IO-11 3MF writer"]
fn reexport_3mf_round_trips() {
    link();
    let Some((_, tmf)) = corpus(THREE_MF) else {
        return;
    };
    let original = read(&tmf, Format::ThreeMf, &ImportOptions::default()).scene;
    let mut doc = Document::new("io14");
    import_into(&mut doc, "orca-three.3mf", Format::ThreeMf, &tmf);
    recompute(&mut doc);
    let scene = exported(&doc);
    let mut out = Vec::new();
    amg_io::export(
        &scene,
        Sink::Memory(&mut out),
        Format::ThreeMf,
        &ExportOptions::default(),
        &ctx(),
    )
    .unwrap();
    let back = read(&out, Format::ThreeMf, &ImportOptions::default()).scene;
    assert_scene_eq(&original, &back);
}

// ----------------------------------------------------------------------------- Done-when 5

const CHILD_ENV: &str = "AMG_IO14_KEY_CHILD";

/// Cache keys of a fixed import design (hex, one per line).
fn keys_of_fixed_design() -> Option<String> {
    let (_, stl) = corpus(STL)?;
    let mut doc = Document::new("io14-keys");
    let mut fids = vec![import_into(&mut doc, "132062.stl", Format::Stl, &stl)];
    if let Some((_, tmf)) = corpus(THREE_MF) {
        fids.push(import_into(
            &mut doc,
            "orca-three.3mf",
            Format::ThreeMf,
            &tmf,
        ));
    }
    recompute(&mut doc);
    let mut s = String::new();
    for f in fids {
        let key = doc
            .evaluated()
            .node(f)
            .unwrap()
            .key
            .expect("computed node has a key");
        for b in key.0 {
            s.push_str(&format!("{b:02x}"));
        }
        s.push('\n');
    }
    Some(s)
}

/// Child half of `cache_key_is_identical_across_processes` (does nothing unless spawned).
#[test]
fn cache_key_child() {
    if std::env::var_os(CHILD_ENV).is_none() {
        return;
    }
    link();
    let keys = keys_of_fixed_design().unwrap_or_default();
    println!("KEYS-BEGIN\n{keys}KEYS-END");
}

#[test]
fn cache_key_is_identical_across_processes() {
    link();
    let Some(mine) = keys_of_fixed_design() else {
        return;
    };
    let out = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "cache_key_child",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(CHILD_ENV, "1")
        .output()
        .unwrap();
    assert!(out.status.success(), "child failed: {out:?}");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let start = stdout.find("KEYS-BEGIN\n").expect("child printed keys") + "KEYS-BEGIN\n".len();
    let end = stdout.find("KEYS-END").unwrap();
    let theirs = &stdout[start..end];
    assert!(!mine.is_empty());
    assert_eq!(mine, theirs, "cache keys differ between processes");
}
