//! IO-06: STEP files in a real `Document` (needs `--features occt`).
//!
//! - An assembly (BREP-13 fixtures `bolt_pattern`, `nested_assembly`) imports as components and
//!   occurrences (ASM-01): one component per assembly node, one per part with an
//!   `io.import_brep` feature, shared parts as several occurrences of one component; one undo
//!   entry removes it all and redo restores it.
//! - A file without assemblies (`two_roots`, `coloured_box_ap214`) imports as bodies of one
//!   feature in the target component.
//! - Document → `brep_scene_from_document` → AP242 → import gives the original tree, names,
//!   colours, volumes (1e-9 relative) and bboxes (1e-6 mm) (Done-when 2 through the document;
//!   also on the NIST models and the owner's Fusion exports when present).
//! - Save → load restores the baked shapes; the feature serde round-trips.
#![cfg(feature = "occt")]
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::print_stdout,
    clippy::print_stderr,
    missing_docs,
    reason = "tests"
)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use amg_core::Transform;
use amg_doc::{BodyGeom, Document, SaveOptions, load, save};
use amg_features::io::{
    BrepImportOutcome, ExportSelection, ImportBrepFeature, ImportChoices, ImportSource,
    brep_scene_from_document, import_brep_commands,
};
use amg_io::Format;
use amg_io::brep::{
    BrepContent, BrepNode, BrepScene, StepSchema, export_step, import_step, import_step_with,
};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
use amg_mesh::Rgba8;

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn fixture(name: &str) -> Option<PathBuf> {
    amg_features::io::link();
    amg_testkit::testdata(&format!("brep/step/{name}.step"))
}

fn tmp(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("io-step-doc");
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

/// Imports `path` into a new document (components when `as_components`), recomputed.
fn import_into(path: &Path, as_components: bool) -> (Document, BrepImportOutcome, BrepScene) {
    let imported = import_step_with(path, &Default::default(), &ctx()).unwrap();
    let scene = imported.scene.clone();
    let source =
        ImportSource::from_path(path, Format::Step, imported.report.declared_unit, &ctx()).unwrap();
    let mut doc = Document::new("step");
    let root = doc.design().root();
    let mut choices = ImportChoices::new(root);
    choices.as_components = as_components;
    let plan = import_brep_commands(imported, source, &choices).unwrap();
    let out = plan.apply(&mut doc, root).unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    (doc, out, scene)
}

fn export_scene(doc: &Document) -> BrepScene {
    brep_scene_from_document(doc.design(), doc.evaluated(), &ExportSelection::AllVisible).unwrap()
}

// ---------------------------------------------------------------- comparisons

fn same_transform(a: &Transform, b: &Transform) -> bool {
    a.to_cols_array()
        .iter()
        .zip(b.to_cols_array().iter())
        .all(|(x, y)| (x - y).abs() <= 1e-9)
}

fn effective(s: &BrepScene, n: &BrepNode) -> Option<Rgba8> {
    match &n.content {
        BrepContent::Shape(o) => n.color.or(s.objects[*o as usize].color),
        BrepContent::Group(_) => n.color,
    }
}

/// Same tree, names, effective colours, part names, face colours, sharing, topology, volume
/// (1e-9 relative) and bbox (1e-6 mm).
fn assert_same(a: &BrepScene, b: &BrepScene, what: &str) {
    assert_eq!(a.roots.len(), b.roots.len(), "{what}: roots");
    let mut map = BTreeMap::new();
    fn walk(
        a: &BrepScene,
        b: &BrepScene,
        x: &BrepNode,
        y: &BrepNode,
        path: &str,
        map: &mut BTreeMap<u32, u32>,
    ) {
        same_name(&x.name, &y.name, path);
        assert!(
            same_transform(&x.transform, &y.transform),
            "{path}: transform"
        );
        assert_eq!(effective(a, x), effective(b, y), "{path}: colour");
        match (&x.content, &y.content) {
            (BrepContent::Shape(i), BrepContent::Shape(j)) => {
                // `b` may unshare a part (instances with their own colour become their own
                // component) but never merges two parts.
                assert!(
                    map.insert(*j, *i).is_none_or(|p| p == *i),
                    "{path}: merged parts"
                );
                let (p, q) = (&a.objects[*i as usize], &b.objects[*j as usize]);
                same_name(&p.name, &q.name, path);
                assert_eq!(p.face_colors, q.face_colors, "{path}: face colours");
                same_shape(&p.shape, &q.shape, path);
            }
            (BrepContent::Group(cx), BrepContent::Group(cy)) => {
                assert_eq!(cx.len(), cy.len(), "{path}: children");
                for (k, (p, q)) in cx.iter().zip(cy).enumerate() {
                    walk(a, b, p, q, &format!("{path}/{k}"), map);
                }
            }
            other => panic!("{path}: kind {other:?}"),
        }
    }
    for (i, (x, y)) in a.roots.iter().zip(&b.roots).enumerate() {
        walk(a, b, x, y, &format!("{what}:{i}"), &mut map);
    }
}

/// Equal names. An unnamed node or part may come back named: the document names every
/// component (`<file stem> <n>`), and OCCT's STEP writer names every product (`Product <n>`;
/// BREP-13 leftover).
fn same_name(a: &Option<String>, b: &Option<String>, path: &str) {
    assert!(a == b || a.is_none(), "{path}: name {a:?} vs {b:?}");
}

/// Same solids and faces, volume within 1e-9 relative and bbox within 1e-6 mm. For a part
/// with tolerances above the kernel's (1e-6 mm; STEP does not store them, so the reader
/// rebuilds them) the bounds are `area · tolerance` and the tolerance (see the amg-io IO-06
/// tests).
fn same_shape(p: &amg_brep::Shape, q: &amg_brep::Shape, what: &str) {
    assert_eq!(p.solids().len(), q.solids().len(), "{what}: solids");
    assert_eq!(
        p.count(amg_brep::SubKind::Face),
        q.count(amg_brep::SubKind::Face),
        "{what}: faces"
    );
    let tol = p.max_tolerance().max(q.max_tolerance());
    let strict = tol <= 1e-6;
    if !p.solids().is_empty() {
        // Adaptive integration (the plain query is fixed-order quadrature, ~1e-4 relative).
        let (vp, vq) = (
            p.volume_adaptive(1e-12).unwrap(),
            q.volume_adaptive(1e-12).unwrap(),
        );
        let bound = if strict {
            1e-9 * vp.abs()
        } else {
            let area: f64 = p.faces().map(|f| p.face_area(f)).sum();
            (1e-9 * vp.abs()).max(area * tol)
        };
        assert!(
            (vp - vq).abs() <= bound,
            "{what}: volume {vp} vs {vq} (tol {tol})"
        );
    }
    let bbox_bound = if strict { 1e-6 } else { tol.max(1e-6) };
    let (a, b) = (p.bbox(), q.bbox());
    if a != b {
        for (x, y) in [a.min, a.max].iter().zip([b.min, b.max].iter()) {
            assert!(
                (*x - *y).abs().max_element() <= bbox_bound,
                "{what}: bbox {a:?} vs {b:?}"
            );
        }
    }
}

/// Exports the document, writes AP242, reads it again and compares with `original`.
fn assert_document_round_trip(doc: &Document, original: &BrepScene, what: &str) {
    let exported = export_scene(doc);
    assert_same(original, &exported, &format!("{what} (document)"));
    let out = tmp(&format!("{what}.step"));
    export_step(&exported, StepSchema::Ap242, &out, &ctx()).unwrap();
    let (back, _) = import_step(&out, &ctx()).unwrap();
    assert_same(original, &back, &format!("{what} (file)"));
}

// ---------------------------------------------------------------- tests

#[test]
fn assembly_imports_as_components_and_round_trips() {
    for name in ["bolt_pattern", "nested_assembly"] {
        let Some(path) = fixture(name) else { continue };
        let (mut doc, out, scene) = import_into(&path, true);
        let design = doc.design();
        // One feature per distinct (part, instance colour).
        let parts: std::collections::BTreeSet<_> = scene.instances().iter().map(|i| i.0).collect();
        assert!(out.features.len() >= parts.len(), "{name}: {out:?}");
        assert_eq!(
            out.occurrences.len(),
            scene.instances().len() + groups(&scene),
            "{name}"
        );
        // Every feature is an io.import_brep in a new component, with healthy bodies.
        for f in &out.features {
            let (_, node) = design.timeline().node(*f).unwrap();
            let feat = node.feature.downcast_ref::<ImportBrepFeature>().unwrap();
            assert_eq!(feat.bodies.len(), 1);
            assert!(out.components.contains(&node.context), "{name}");
        }
        assert_eq!(doc.evaluated().bodies().count(), out.features.len());
        for (_, b) in doc.evaluated().bodies() {
            let BodyGeom::Solid(s) = &b.geom else {
                panic!("solid")
            };
            let kernel = amg_brep::OcctKernel::new(amg_brep::KernelConfig::default());
            let report =
                amg_brep::BrepKernel::check(&kernel, s.brep().unwrap(), &Default::default());
            assert!(report.valid, "{name}: {report:?}");
        }
        // Shared parts: the bolt (4 instances, one with its own colour) is 2 components.
        if name == "bolt_pattern" {
            let bolt_paths: usize = out
                .components
                .iter()
                .filter(|c| design.component(**c).unwrap().name == "bolt")
                .map(|c| design.occurrence_paths(*c).len())
                .sum();
            assert_eq!(bolt_paths, 4);
        }
        assert_document_round_trip(&doc, &scene, name);
        // One undo entry.
        assert_eq!(
            doc.undo_label(),
            Some(format!("Insert {name}.step").as_str())
        );
        let before = doc.design().components().count();
        assert!(doc.undo());
        assert_eq!(doc.design().components().count(), 1);
        assert!(doc.redo());
        assert_eq!(doc.design().components().count(), before);
        doc.recompute_blocking(jobs()).unwrap();
        assert_document_round_trip(&doc, &scene, &format!("{name}-redo"));
        println!(
            "{name}: {} components, {} occurrences, {} features",
            out.components.len(),
            out.occurrences.len(),
            out.features.len()
        );
    }
}

fn groups(s: &BrepScene) -> usize {
    fn walk(n: &BrepNode) -> usize {
        match &n.content {
            BrepContent::Group(c) => 1 + c.iter().map(walk).sum::<usize>(),
            BrepContent::Shape(_) => 0,
        }
    }
    s.roots.iter().map(walk).sum()
}

#[test]
fn files_without_assemblies_import_as_bodies() {
    for name in ["two_roots", "coloured_box_ap214", "filleted_box"] {
        let Some(path) = fixture(name) else { continue };
        let (doc, out, scene) = import_into(&path, true);
        assert!(
            out.components.is_empty() && out.occurrences.is_empty(),
            "{name}"
        );
        assert_eq!(out.features.len(), 1);
        assert_eq!(doc.evaluated().bodies().count(), scene.instances().len());
        assert_document_round_trip(&doc, &scene, name);
    }
}

#[test]
fn assemblies_can_be_flattened_into_bodies() {
    let Some(path) = fixture("bolt_pattern") else {
        return;
    };
    let (doc, out, scene) = import_into(&path, false);
    assert!(out.components.is_empty());
    assert_eq!(doc.evaluated().bodies().count(), 5);
    let exported = export_scene(&doc);
    assert_eq!(exported.roots.len(), 5);
    // World placements and colours of the instances are kept.
    let mut want: Vec<(Option<Rgba8>, [i64; 3])> = Vec::new();
    let colors = {
        let mut v = Vec::new();
        fn walk(s: &BrepScene, n: &BrepNode, c: Option<Rgba8>, out: &mut Vec<Option<Rgba8>>) {
            let c = n.color.or(c);
            match &n.content {
                BrepContent::Shape(o) => out.push(c.or(s.objects[*o as usize].color)),
                BrepContent::Group(ch) => ch.iter().for_each(|x| walk(s, x, c, out)),
            }
        }
        scene
            .roots
            .iter()
            .for_each(|r| walk(&scene, r, None, &mut v));
        v
    };
    let key = |s: &amg_brep::Shape| {
        let c = s.bbox().center();
        [
            (c.x * 1e3).round() as i64,
            (c.y * 1e3).round() as i64,
            (c.z * 1e3).round() as i64,
        ]
    };
    for ((o, t, _), c) in scene.instances().iter().zip(colors) {
        let placed = scene.objects[*o as usize].shape.transformed(t).unwrap();
        want.push((c, key(&placed)));
    }
    let mut got: Vec<(Option<Rgba8>, [i64; 3])> = exported
        .roots
        .iter()
        .map(|n| {
            let BrepContent::Shape(o) = n.content else {
                panic!("part")
            };
            let obj = &exported.objects[o as usize];
            (obj.color, key(&obj.shape))
        })
        .collect();
    want.sort_by_key(|x| x.1);
    got.sort_by_key(|x| x.1);
    assert_eq!(want, got);
}

#[test]
fn save_load_restores_the_baked_shapes() {
    let Some(path) = fixture("nested_assembly") else {
        return;
    };
    let (doc, _, scene) = import_into(&path, true);
    let file = tmp("nested.amg");
    save(&file, &doc.snapshot(), &SaveOptions::default()).unwrap();
    let mut reopened = load(&file).unwrap().into_document();
    reopened.recompute_blocking(jobs()).unwrap();
    assert_eq!(
        reopened.evaluated().bodies().count(),
        doc.evaluated().bodies().count()
    );
    assert_document_round_trip(&reopened, &scene, "nested-reloaded");
}

#[test]
fn feature_serde_round_trips() {
    let Some(path) = fixture("coloured_box_ap214") else {
        return;
    };
    let (doc, out, _) = import_into(&path, true);
    let (_, node) = doc.design().timeline().node(out.features[0]).unwrap();
    let f = node.feature.downcast_ref::<ImportBrepFeature>().unwrap();
    assert!(!f.face_colors.is_empty() && !f.colors.is_empty());
    let json = serde_json::to_string(f).unwrap();
    let back: ImportBrepFeature = serde_json::from_str(&json).unwrap();
    assert_eq!(&back, f);
    let mut broken = f.clone();
    broken.bodies[0].material = Some(99);
    assert!(amg_doc::Feature::validate(&broken).is_err());
}

/// Done-when 1-3 through the document: the NIST models and the owner's Fusion exports
/// (skipped when absent).
#[test]
fn external_models_round_trip_through_the_document() {
    for rel in ["io/step/nist-mbe-pmi", "fusion"] {
        let Some(root) = std::env::var_os("AMG_TESTDATA_DIR").map(PathBuf::from) else {
            eprintln!("SKIP: AMG_TESTDATA_DIR is not set");
            return;
        };
        let Ok(entries) = std::fs::read_dir(root.join(rel)) else {
            eprintln!("SKIP: {rel} is absent");
            continue;
        };
        let mut files: Vec<PathBuf> = entries
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| {
                let s = p.to_string_lossy().to_ascii_lowercase();
                (s.ends_with(".step") || s.ends_with(".stp")) && !s.ends_with("-tg.stp")
            })
            .collect();
        files.sort();
        for path in &files {
            amg_features::io::link();
            let (doc, _, scene) = import_into(path, true);
            let stem = path.file_stem().unwrap().to_string_lossy().into_owned();
            assert_document_round_trip(&doc, &scene, &format!("doc-{stem}"));
        }
        println!("{rel}: {} files through the document", files.len());
    }
}
