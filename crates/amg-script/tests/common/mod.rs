//! Fixture designs of SCR-07, built through the public API, and measurement helpers.

#![allow(dead_code, reason = "each test binary uses a part of the fixtures")]

pub mod schema;

use std::path::{Path, PathBuf};
use std::sync::Arc;

use amg_core::{Density, Point3, Transform};
use amg_doc::testing::TestCounter;
use amg_doc::{
    BodyId, Command, ComponentId, CreatedId, Dependency, Document, FeatureBox, FeatureId,
    SaveOptions,
};
use amg_features::inspect::props::mesh_mass_props;
use amg_features::io::{ImportChoices, ImportSource, import_commands};
use amg_io::{
    Format, ImportOptions, ImportReport, Imported, Scene, SceneNode, SceneObject, Source,
};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
use amg_mesh::{IndexedMesh, MeshBody};
use amg_testkit::{TriMesh, shapes};

pub fn jobs() -> Arc<JobSystem> {
    Arc::new(JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn indexed(m: TriMesh) -> IndexedMesh {
    IndexedMesh::new(m.positions, m.triangles).unwrap()
}

/// An axis-aligned cube of edge `size` with its minimum corner at `min`.
pub fn cube(min: Point3, size: f64) -> IndexedMesh {
    indexed(shapes::cube(min, size))
}

/// A faceted sphere with full `f64` coordinates.
pub fn sphere(center: Point3, r: f64) -> IndexedMesh {
    indexed(shapes::icosphere(center, r, 3))
}

/// [`sphere`] with every coordinate rounded to `f32`, like a mesh that came from STL.
pub fn sphere_f32(center: Point3, r: f64) -> IndexedMesh {
    let mut m = sphere(center, r);
    for p in &mut m.positions {
        *p = Point3::new(
            f64::from(p.x as f32),
            f64::from(p.y as f32),
            f64::from(p.z as f32),
        );
    }
    m
}

/// A scene with one named object per mesh, placed as it is.
pub fn scene(objects: Vec<(&str, IndexedMesh)>) -> Scene {
    let mut scene = Scene::default();
    for (i, (name, mesh)) in objects.into_iter().enumerate() {
        let mut o = SceneObject::new(Arc::new(mesh));
        o.name = Some(name.to_owned());
        scene.objects.push(o);
        scene
            .roots
            .push(SceneNode::mesh(i as u32, Transform::IDENTITY));
    }
    scene
}

/// "Insert mesh" of one object into `target`; the body is slot 0 of the new feature.
pub fn insert(doc: &mut Document, target: ComponentId, name: &str, mesh: IndexedMesh) -> BodyId {
    let imported = Imported {
        scene: scene(vec![(name, mesh)]),
        report: ImportReport::default(),
    };
    let source =
        ImportSource::from_bytes(format!("{name}.stl"), Format::Stl, None, name.as_bytes());
    let plan = import_commands(imported, source, &ImportChoices::new(target)).unwrap();
    let origin = plan.apply(doc).unwrap();
    BodyId { origin, slot: 0 }
}

pub fn add_command(context: ComponentId, feature: FeatureBox, name: &str) -> Command {
    Command::AddFeature {
        at: None,
        feature,
        context,
        name: Some(name.to_owned()),
    }
}

pub fn add_feature(doc: &mut Document, feature: FeatureBox, name: &str) -> FeatureId {
    let command = add_command(doc.design().root(), feature, name);
    let out = doc.apply(command).unwrap();
    match out.created.first() {
        Some(CreatedId::Feature(id)) => *id,
        other => panic!("no feature: {other:?}"),
    }
}

/// The lane-E test feature: reads `body`, passes it through and adds an empty mesh body.
pub fn counter(body: BodyId, value: i64, fail: bool) -> FeatureBox {
    FeatureBox::new(TestCounter {
        deps: vec![Dependency::Body(body)],
        value,
        fail,
    })
}

fn first_body(doc: &Document) -> BodyId {
    BodyId {
        origin: doc.design().timeline().nodes().next().unwrap().id,
        slot: 0,
    }
}

/// Fixture 1: two imported meshes, "cube" (10 mm, at the origin) and "ball".
pub fn mesh_only() -> Document {
    amg_script::link().unwrap();
    let mut doc = Document::new("mesh only");
    let root = doc.design().root();
    insert(&mut doc, root, "cube", cube(Point3::ZERO, 10.0));
    insert(
        &mut doc,
        root,
        "ball",
        sphere_f32(Point3::new(30.0, 5.0, 5.0), 7.25),
    );
    doc
}

/// Fixture 2: fixture 1 plus a test feature on the cube. Returns the test feature and the cube.
pub fn mixed() -> (Document, FeatureId, BodyId) {
    let mut doc = mesh_only();
    let cube = first_body(&doc);
    let test = add_feature(&mut doc, counter(cube, 1, false), "probe");
    (doc, test, cube)
}

/// Fixture 3: fixture 1 plus a failing test feature. Returns the failing feature.
pub fn failing() -> (Document, FeatureId) {
    let mut doc = mesh_only();
    let cube = first_body(&doc);
    let bad = add_feature(&mut doc, counter(cube, 1, true), "broken");
    (doc, bad)
}

/// Saves `doc` compact (no geometry cache) as `dir/name.amg`.
pub fn save(doc: &Document, dir: &Path, name: &str) -> PathBuf {
    let path = dir.join(format!("{name}.amg"));
    amg_doc::save(&path, &doc.snapshot(), &SaveOptions::default()).unwrap();
    path
}

/// What a mesh file holds, measured over its placed objects.
#[derive(Debug)]
pub struct FileMeasure {
    pub triangles: u64,
    pub volume: f64,
    pub bbox: (Point3, Point3),
}

pub fn read_scene(path: &Path) -> Scene {
    amg_io::import(
        Source::Path(path),
        None,
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .unwrap()
    .scene
}

pub fn measure_file(path: &Path) -> FileMeasure {
    let scene = read_scene(path);
    let bbox = scene.world_aabb();
    let mut volume = 0.0;
    for (i, world) in scene.instances() {
        let body = MeshBody::from_arc(Arc::clone(&scene.objects[i as usize].mesh));
        let p = mesh_mass_props(&body, &world, Density::ZERO, &JobCtx::detached()).unwrap();
        volume += p.volume.expect("closed mesh").get();
    }
    FileMeasure {
        triangles: scene.triangle_count(),
        volume,
        bbox: (bbox.min, bbox.max),
    }
}

pub fn relative(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}
