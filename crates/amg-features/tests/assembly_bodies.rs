//! ASM-09: create components from bodies, move bodies between components (S06).
//!
//! Mesh bodies run everywhere; B-rep solids need the `occt` feature (AMG_OCCT_DIR).

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

use amg_core::{Aabb3, Point3, Transform, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, ComponentId, ConstructionGeom,
    CreatedId, Dependency, Design, Document, Feature, FeatureBox, FeatureError, FeatureId,
    FeatureOutput, Health, OccurrenceId, RecomputeCtx, SaveOptions, load, register_feature, save,
};
use amg_features::assembly::{
    AsmPrepError, BodiesToComponentsFeature, BodyMove, components_from_bodies, move_bodies,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_mesh::IndexedMesh;
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------- test features

/// Creates mesh body slot 0 (stands in for an importer).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Asm09Mesh {
    mesh: IndexedMesh,
}

impl Feature for Asm09Mesh {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(self.mesh.clone())),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

register_feature!(Asm09Mesh, "test.asm09_mesh", version = 1);

/// Reads a body and outputs the centre of its own-space bounding box as a construction point.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Asm09Reader {
    body: BodyId,
}

impl Feature for Asm09Reader {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let c = match ctx.body(self.body)? {
            BodyGeom::Mesh(m) => m.aabb().center(),
            #[cfg(feature = "occt")]
            BodyGeom::Solid(s) => s.brep().map_or(Point3::ZERO, |b| b.bbox().center()),
            _ => Point3::ZERO,
        };
        Ok(FeatureOutput {
            construction: Some(ConstructionGeom::Point(c)),
            ..FeatureOutput::default()
        })
    }
}

register_feature!(Asm09Reader, "test.asm09_reader", version = 1);

// ---------------------------------------------------------------- helpers

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

/// Closed, outward-oriented box `[min, min + size]`.
fn cube(min: [f64; 3], size: [f64; 3]) -> IndexedMesh {
    let p = |i: u32| {
        Point3::new(
            min[0] + if i & 1 != 0 { size[0] } else { 0.0 },
            min[1] + if i & 2 != 0 { size[1] } else { 0.0 },
            min[2] + if i & 4 != 0 { size[2] } else { 0.0 },
        )
    };
    let positions = (0..8).map(p).collect();
    let quads = [
        [0, 2, 3, 1], // z-
        [4, 5, 7, 6], // z+
        [0, 1, 5, 4], // y-
        [2, 6, 7, 3], // y+
        [0, 4, 6, 2], // x-
        [1, 3, 7, 5], // x+
    ];
    let mut tris = Vec::new();
    for [a, b, c, d] in quads {
        tris.push([a, b, c]);
        tris.push([a, c, d]);
    }
    IndexedMesh::new(positions, tris).unwrap()
}

fn created_feature(created: &[CreatedId]) -> FeatureId {
    created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap()
}

fn add(doc: &mut Document, f: FeatureBox, context: ComponentId) -> FeatureId {
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context,
            name: None,
        })
        .unwrap();
    created_feature(&out.created)
}

fn add_mesh(doc: &mut Document, context: ComponentId, mesh: IndexedMesh) -> BodyId {
    let f = add(doc, FeatureBox::new(Asm09Mesh { mesh }), context);
    BodyId { origin: f, slot: 0 }
}

fn new_component(
    doc: &mut Document,
    parent: ComponentId,
    t: Transform,
) -> (ComponentId, OccurrenceId) {
    let out = doc
        .apply(Command::NewComponent {
            parent,
            name: "C".to_owned(),
            transform: t,
        })
        .unwrap();
    let mut c = None;
    let mut o = None;
    for id in out.created {
        match id {
            CreatedId::Component(x) => c = Some(x),
            CreatedId::Occurrence(x) => o = Some(x),
            _ => {}
        }
    }
    (c.unwrap(), o.unwrap())
}

fn recompute(doc: &mut Document) {
    doc.recompute_blocking(jobs()).unwrap();
}

fn health(doc: &Document, f: FeatureId) -> Health {
    doc.evaluated().node(f).unwrap().health.clone()
}

/// The only world transform of `body`.
fn world(doc: &Document, body: BodyId) -> Transform {
    let inst = doc.evaluated().instances(body);
    assert_eq!(inst.len(), 1, "{body:?} has one instance");
    inst[0].world
}

/// World bounding box and volume of a body's only instance.
fn world_bbox_volume(doc: &Document, body: BodyId) -> (Aabb3, f64) {
    let w = world(doc, body);
    match &doc.evaluated().body(body).unwrap().geom {
        BodyGeom::Mesh(m) => {
            let mut m = (**m).clone();
            m.transform(&w);
            (m.aabb(), m.signed_volume())
        }
        #[cfg(feature = "occt")]
        BodyGeom::Solid(s) => {
            let s = s.brep().expect("a real solid").transformed(&w).unwrap();
            (s.bbox(), s.volume().unwrap())
        }
        other => panic!("{other:?}"),
    }
}

fn assert_bbox_close(a: &Aabb3, b: &Aabb3, tol: f64) {
    let d = (a.min - b.min)
        .abs()
        .max_element()
        .max((a.max - b.max).abs().max_element());
    assert!(d <= tol, "bbox moved by {d:e} mm: {a:?} vs {b:?}");
}

fn component_of(doc: &Document, body: BodyId) -> ComponentId {
    doc.evaluated().body(body).unwrap().component
}

fn run_transaction(doc: &mut Document, label: &str, cmds: Vec<Command>) -> Vec<CreatedId> {
    let mut tx = doc.transaction(label);
    let mut created = Vec::new();
    for c in cmds {
        created.extend(tx.apply(c).unwrap().created);
    }
    tx.commit();
    created
}

fn design_json(d: &Design) -> String {
    serde_json::to_string(d).unwrap()
}

/// A rotation that keeps no axis aligned, plus a translation.
fn skew_placement() -> Transform {
    Transform::from_translation(Vec3::new(12.5, -7.0, 3.25))
        * Transform::from_axis_angle(Vec3::new(1.0, 2.0, 3.0).normalize(), 0.7)
}

// ---------------------------------------------------------------- Done-when 1

#[test]
fn the_kind_is_registered() {
    amg_features::assembly::link();
    let reg = amg_doc::try_registry().unwrap();
    assert_eq!(reg.get("assembly.bodies_to_components").unwrap().version, 1);
}

#[test]
fn three_root_bodies_become_three_components_in_one_undo_entry() {
    amg_features::assembly::link();
    let mut doc = Document::new("asm09");
    let root = doc.design().root();
    let bodies: Vec<BodyId> = (0..3)
        .map(|i| {
            add_mesh(
                &mut doc,
                root,
                cube([f64::from(i) * 20.0, 1.0, -2.0], [5.0, 6.0, 7.0]),
            )
        })
        .collect();
    recompute(&mut doc);
    let before: Vec<_> = bodies.iter().map(|b| world_bbox_volume(&doc, *b)).collect();
    let json_before = design_json(doc.design());
    let nodes_before = doc.design().timeline().len();
    let comps_before = doc.design().components().count();

    let cmds = components_from_bodies(doc.design(), doc.evaluated(), &bodies).unwrap();
    assert_eq!(cmds.len(), 4, "3 NewComponent + 1 AddFeature");
    let created = run_transaction(&mut doc, "Create components from bodies", cmds);
    recompute(&mut doc);

    let new_comps: Vec<ComponentId> = created
        .iter()
        .filter_map(|c| match c {
            CreatedId::Component(c) => Some(*c),
            _ => None,
        })
        .collect();
    assert_eq!(new_comps.len(), 3);
    assert_eq!(doc.design().components().count(), comps_before + 3);
    assert_eq!(
        doc.design().timeline().len(),
        nodes_before + 1,
        "one feature"
    );
    let feature = created_feature(&created);
    assert_eq!(health(&doc, feature), Health::Ok);
    for (i, b) in bodies.iter().enumerate() {
        let eb = doc.evaluated().body(*b).expect("same BodyId");
        assert_eq!(eb.component, new_comps[i]);
        assert_eq!(doc.design().occurrence_paths(new_comps[i]).len(), 1);
        let (bb, vol) = world_bbox_volume(&doc, *b);
        assert_bbox_close(&bb, &before[i].0, 1e-9);
        assert!((vol - before[i].1).abs() <= 1e-9 * before[i].1.abs());
    }
    assert_eq!(doc.evaluated().bodies().count(), 3);

    // One undo entry restores the design exactly; redo brings it back.
    assert_eq!(doc.undo_label(), Some("Create components from bodies"));
    assert!(doc.undo());
    assert_eq!(design_json(doc.design()), json_before);
    recompute(&mut doc);
    for b in &bodies {
        assert_eq!(component_of(&doc, *b), root);
    }
    assert!(doc.redo());
    recompute(&mut doc);
    for (i, b) in bodies.iter().enumerate() {
        assert_eq!(component_of(&doc, *b), new_comps[i]);
    }
}

#[test]
fn a_body_in_a_placed_sub_component_gets_its_new_component_inside_it() {
    amg_features::assembly::link();
    let mut doc = Document::new("asm09-sub");
    let root = doc.design().root();
    let (sub, _) = new_component(&mut doc, root, skew_placement());
    let b = add_mesh(&mut doc, sub, cube([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]));
    recompute(&mut doc);
    let (bb0, v0) = world_bbox_volume(&doc, b);
    let cmds = components_from_bodies(doc.design(), doc.evaluated(), &[b, b]).unwrap();
    assert_eq!(cmds.len(), 2, "duplicates are ignored");
    let created = run_transaction(&mut doc, "Create components from bodies", cmds);
    recompute(&mut doc);
    let c = component_of(&doc, b);
    assert!(created.contains(&CreatedId::Component(c)));
    let path = &doc.design().occurrence_paths(c)[0];
    assert_eq!(doc.design().occurrence(path[1]).unwrap().parent, sub);
    let (bb1, v1) = world_bbox_volume(&doc, b);
    assert_bbox_close(&bb0, &bb1, 1e-9);
    assert!((v0 - v1).abs() <= 1e-9 * v0);
}

// ---------------------------------------------------------------- Done-when 2 and 3

/// Root body moved into a component nested two levels deep, both placed off-axis.
fn moved_fixture() -> (
    Document,
    BodyId,
    ComponentId,
    OccurrenceId,
    (Aabb3, f64),
    FeatureId,
) {
    amg_features::assembly::link();
    let mut doc = Document::new("asm09-move");
    let root = doc.design().root();
    let (outer, _) = new_component(&mut doc, root, skew_placement());
    let (target, occ) = new_component(
        &mut doc,
        outer,
        Transform::from_translation(Vec3::new(-3.0, 40.0, 1.0)) * Transform::from_rotation_z(2.1),
    );
    let b = add_mesh(&mut doc, root, cube([10.0, 20.0, 30.0], [3.0, 4.0, 5.0]));
    recompute(&mut doc);
    let before = world_bbox_volume(&doc, b);
    let cmds = move_bodies(doc.design(), doc.evaluated(), &[b], target).unwrap();
    assert_eq!(cmds.len(), 1);
    let out = doc.apply(cmds.into_iter().next().unwrap()).unwrap();
    let f = created_feature(&out.created);
    recompute(&mut doc);
    (doc, b, target, occ, before, f)
}

#[test]
fn moving_a_mesh_body_into_a_placed_component_keeps_world_bbox_and_volume() {
    let (doc, b, target, _, (bb0, v0), f) = moved_fixture();
    assert_eq!(health(&doc, f), Health::Ok);
    assert_eq!(component_of(&doc, b), target);
    let (bb1, v1) = world_bbox_volume(&doc, b);
    assert_bbox_close(&bb0, &bb1, 1e-9);
    assert!(v0 > 0.0);
    assert!((v1 - v0).abs() <= 1e-9 * v0, "volume {v0} -> {v1}");
    // The geometry really is in the target's space (not just re-tagged).
    let BodyGeom::Mesh(m) = &doc.evaluated().body(b).unwrap().geom else {
        panic!()
    };
    assert!((m.aabb().min - bb0.min).length() > 1.0);
}

#[test]
fn a_mirrored_target_keeps_mesh_volume_positive() {
    amg_features::assembly::link();
    let mut doc = Document::new("asm09-mirror");
    let root = doc.design().root();
    let mirror = Transform::from_translation(Vec3::new(5.0, 0.0, 0.0))
        * Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0));
    let (target, _) = new_component(&mut doc, root, mirror);
    let b = add_mesh(&mut doc, root, cube([1.0, 2.0, 3.0], [2.0, 3.0, 4.0]));
    recompute(&mut doc);
    let (bb0, v0) = world_bbox_volume(&doc, b);
    let cmd = move_bodies(doc.design(), doc.evaluated(), &[b], target).unwrap();
    let f = created_feature(&doc.apply(cmd[0].clone()).unwrap().created);
    recompute(&mut doc);
    assert_eq!(health(&doc, f), Health::Ok);
    let (bb1, v1) = world_bbox_volume(&doc, b);
    assert_bbox_close(&bb0, &bb1, 1e-9);
    assert!((v1 - v0).abs() <= 1e-9 * v0, "volume {v0} -> {v1}");
}

#[test]
fn a_later_root_feature_still_reads_the_moved_body() {
    let (mut doc, b, target, _, _, mover) = moved_fixture();
    let root = doc.design().root();
    let reader = add(&mut doc, FeatureBox::new(Asm09Reader { body: b }), root);
    recompute(&mut doc);
    assert_eq!(health(&doc, mover), Health::Ok);
    assert_eq!(health(&doc, reader), Health::Ok, "same BodyId resolves");
    assert_eq!(
        component_of(&doc, b),
        target,
        "the body reports its new component"
    );
    assert_eq!(doc.evaluated().instances(b)[0].path.len(), 2);
}

#[test]
fn moving_the_target_occurrence_moves_the_body_and_recomputes_nothing() {
    let (mut doc, b, _, occ, (bb0, _), _) = moved_fixture();
    let shift = Vec3::new(0.0, 0.0, 10.0);
    let local = doc.design().occurrence(occ).unwrap().transform;
    let outer_world = doc
        .evaluated()
        .world_transform(&doc.evaluated().instances(b)[0].path[..1])
        .unwrap();
    // Shift the target by +10 mm in world z: local' = outer⁻¹ · T(shift) · outer · local.
    let moved = outer_world.inverse() * Transform::from_translation(shift) * outer_world * local;
    doc.apply(Command::SetOccurrenceTransforms {
        moves: vec![(occ, moved)],
    })
    .unwrap();
    recompute(&mut doc);
    assert!(
        doc.last_run().unwrap().computed.is_empty(),
        "no feature re-ran"
    );
    let (bb1, _) = world_bbox_volume(&doc, b);
    assert_bbox_close(&Aabb3::new(bb0.min + shift, bb0.max + shift), &bb1, 1e-9);
}

#[test]
fn bodies_already_in_the_target_give_no_commands() {
    let (doc, b, target, _, _, _) = moved_fixture();
    assert!(
        move_bodies(doc.design(), doc.evaluated(), &[b], target)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn builder_errors() {
    amg_features::assembly::link();
    let mut doc = Document::new("asm09-errors");
    let root = doc.design().root();
    let (twice, _) = new_component(&mut doc, root, Transform::IDENTITY);
    doc.apply(Command::AddOccurrence {
        parent: root,
        component: twice,
        transform: Transform::from_translation(Vec3::X),
        name: None,
    })
    .unwrap();
    let (single, _) = new_component(&mut doc, root, Transform::IDENTITY);
    let in_twice = add_mesh(&mut doc, twice, cube([0.0; 3], [1.0; 3]));
    let in_root = add_mesh(&mut doc, root, cube([0.0; 3], [1.0; 3]));
    recompute(&mut doc);
    let (d, e) = (doc.design(), doc.evaluated());
    let ghost = BodyId {
        origin: FeatureId(9999),
        slot: 0,
    };
    assert_eq!(
        move_bodies(d, e, &[in_root], twice).unwrap_err(),
        AsmPrepError::AmbiguousTarget(2)
    );
    assert_eq!(
        move_bodies(d, e, &[in_twice], single).unwrap_err(),
        AsmPrepError::AmbiguousSource {
            body: in_twice,
            placements: 2
        }
    );
    assert_eq!(
        move_bodies(d, e, &[ghost], single).unwrap_err(),
        AsmPrepError::NoBody(ghost)
    );
    assert_eq!(
        move_bodies(d, e, &[in_root], ComponentId(4242)).unwrap_err(),
        AsmPrepError::NoComponent(ComponentId(4242))
    );
    assert_eq!(
        components_from_bodies(d, e, &[ghost]).unwrap_err(),
        AsmPrepError::NoBody(ghost)
    );
    assert!(components_from_bodies(d, e, &[]).unwrap().is_empty());
    // A component placed twice can still get components from its bodies: the new one is
    // placed inside it, so the body keeps both instances.
    let cmds = components_from_bodies(d, e, &[in_twice]).unwrap();
    run_transaction(&mut doc, "Create components from bodies", cmds);
    recompute(&mut doc);
    assert_eq!(doc.evaluated().instances(in_twice).len(), 2);
}

#[test]
fn invalid_feature_data_is_refused() {
    amg_features::assembly::link();
    let mut doc = Document::new("asm09-invalid");
    let root = doc.design().root();
    let b = add_mesh(&mut doc, root, cube([0.0; 3], [1.0; 3]));
    let (c, _) = new_component(&mut doc, root, Transform::IDENTITY);
    let try_add = |doc: &mut Document, moves: Vec<BodyMove>| {
        doc.apply(Command::AddFeature {
            at: None,
            feature: FeatureBox::new(BodiesToComponentsFeature { moves }),
            context: root,
            name: None,
        })
    };
    let mv = |t: Transform| BodyMove {
        body: b,
        to: c,
        transform: t,
    };
    assert!(try_add(&mut doc, vec![]).is_err(), "empty");
    assert!(
        try_add(&mut doc, vec![mv(Transform::from_scale(Vec3::splat(2.0)))]).is_err(),
        "scale"
    );
    assert!(
        try_add(
            &mut doc,
            vec![mv(Transform::IDENTITY), mv(Transform::IDENTITY)]
        )
        .is_err(),
        "twice"
    );
    // A deleted target makes the node fail; the body stays where it was.
    let f = created_feature(
        &try_add(&mut doc, vec![mv(Transform::IDENTITY)])
            .unwrap()
            .created,
    );
    let occ = doc.design().occurrence_paths(c)[0][0];
    doc.apply(Command::DeleteOccurrences { occs: vec![occ] })
        .unwrap();
    recompute(&mut doc);
    assert!(matches!(health(&doc, f), Health::Error(_)));
    assert_eq!(component_of(&doc, b), root);
}

// ---------------------------------------------------------------- Done-when 4

struct TempFile(PathBuf);

impl TempFile {
    fn new() -> TempFile {
        static N: AtomicU32 = AtomicU32::new(0);
        TempFile(std::env::temp_dir().join(format!(
            "amg-features-asm09-{}-{}.amg",
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

#[test]
fn save_and_load_keep_the_tree_and_each_bodys_component() {
    let (mut doc, moved, target, _, _, _) = moved_fixture();
    let root = doc.design().root();
    let other = add_mesh(&mut doc, root, cube([-9.0; 3], [2.0; 3]));
    recompute(&mut doc);
    let cmds = components_from_bodies(doc.design(), doc.evaluated(), &[other]).unwrap();
    run_transaction(&mut doc, "Create components from bodies", cmds);
    recompute(&mut doc);
    let comps: Vec<(BodyId, ComponentId)> = doc
        .evaluated()
        .bodies()
        .map(|(id, b)| (id, b.component))
        .collect();
    assert!(comps.contains(&(moved, target)));
    let json = design_json(doc.design());
    let bb_moved = world_bbox_volume(&doc, moved).0;

    for with_cache in [false, true] {
        let f = TempFile::new();
        let opts = SaveOptions {
            cache: with_cache.then(|| doc.cache_export()),
            ..SaveOptions::default()
        };
        save(&f.0, &doc.snapshot(), &opts).unwrap();
        let mut back = load(&f.0).unwrap().into_document();
        assert_eq!(
            design_json(back.design()),
            json,
            "tree, occurrences and timeline"
        );
        recompute(&mut back);
        if with_cache {
            assert!(
                back.last_run().unwrap().computed.is_empty(),
                "the moved bodies come from the persisted cache"
            );
        }
        let back_comps: Vec<(BodyId, ComponentId)> = back
            .evaluated()
            .bodies()
            .map(|(id, b)| (id, b.component))
            .collect();
        assert_eq!(back_comps, comps, "cache: {with_cache}");
        assert_bbox_close(&world_bbox_volume(&back, moved).0, &bb_moved, 0.0);
    }
}

// ---------------------------------------------------------------- B-rep solids (OCCT)

#[cfg(feature = "occt")]
mod solid {
    use super::*;

    /// Creates solid body slot 0 from `.brep` text.
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct Asm09Solid {
        brep: String,
    }

    impl Feature for Asm09Solid {
        fn inputs(&self) -> Vec<Dependency> {
            Vec::new()
        }

        fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
            let s = amg_brep::Shape::read_text(self.brep.as_bytes())
                .map_err(|e| FeatureError::Kernel(e.to_string()))?;
            Ok(FeatureOutput {
                bodies: vec![BodyChange {
                    id: ctx.new_body_id(0),
                    change: BodyChangeKind::Created {
                        geom: BodyGeom::Solid(amg_doc::stub::Shape::from_brep(s)),
                        naming: None,
                    },
                }],
                ..FeatureOutput::default()
            })
        }
    }

    register_feature!(Asm09Solid, "test.asm09_solid", version = 1);

    fn fixture(name: &str) -> String {
        let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/brep")
            .join(name);
        std::fs::read_to_string(p).unwrap()
    }

    fn solid_doc(name: &str, target_at: Transform) -> (Document, BodyId, ComponentId) {
        amg_features::assembly::link();
        let mut doc = Document::new("asm09-solid");
        let root = doc.design().root();
        let (outer, _) = new_component(&mut doc, root, skew_placement());
        let (target, _) = new_component(&mut doc, outer, target_at);
        let f = add(
            &mut doc,
            FeatureBox::new(Asm09Solid {
                brep: fixture(name),
            }),
            root,
        );
        recompute(&mut doc);
        (doc, BodyId { origin: f, slot: 0 }, target)
    }

    #[test]
    fn moving_a_solid_keeps_world_bbox_and_volume() {
        for name in ["box.brep", "plate_with_holes.brep", "filleted_box.brep"] {
            let (mut doc, b, target) = solid_doc(
                name,
                Transform::from_translation(Vec3::new(-3.0, 40.0, 1.0))
                    * Transform::from_rotation_z(2.1),
            );
            let (bb0, v0) = world_bbox_volume(&doc, b);
            assert!(v0 > 0.0, "{name}");
            let cmd = move_bodies(doc.design(), doc.evaluated(), &[b], target).unwrap();
            let f = created_feature(&doc.apply(cmd[0].clone()).unwrap().created);
            let root = doc.design().root();
            let reader = add(&mut doc, FeatureBox::new(Asm09Reader { body: b }), root);
            recompute(&mut doc);
            assert_eq!(health(&doc, f), Health::Ok, "{name}");
            assert_eq!(health(&doc, reader), Health::Ok, "{name}");
            assert_eq!(component_of(&doc, b), target);
            let (bb1, v1) = world_bbox_volume(&doc, b);
            println!(
                "{name}: volume {v0} -> {v1} (rel {:e}); bbox min {:?} -> {:?}",
                ((v1 - v0) / v0).abs(),
                bb0.min,
                bb1.min
            );
            assert_bbox_close(&bb0, &bb1, 1e-9);
            assert!(((v1 - v0) / v0).abs() <= 1e-9, "{name}: {v0} -> {v1}");
        }
    }

    #[test]
    fn a_solid_cannot_move_into_a_mirrored_placement() {
        let (mut doc, b, target) =
            solid_doc("box.brep", Transform::from_scale(Vec3::new(1.0, -1.0, 1.0)));
        let cmd = move_bodies(doc.design(), doc.evaluated(), &[b], target).unwrap();
        let f = created_feature(&doc.apply(cmd[0].clone()).unwrap().created);
        recompute(&mut doc);
        assert!(
            matches!(health(&doc, f), Health::Error(_)),
            "{:?}",
            health(&doc, f)
        );
        assert_eq!(component_of(&doc, b), doc.design().root());
    }
}
