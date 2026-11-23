//! AM-06 glue: `resolve_plate` / `handoff_plate` against a real `Document`, and the hand-off
//! file re-imported through IO-03 (positions in the plate frame, names, filaments).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use std::sync::{Arc, OnceLock};

use amg_am::handoff::{HandoffFormat, export_plate};
use amg_am::machine::{MachineKey, MachineLibrary, SlicerKind};
use amg_am::place::Placement;
use amg_core::glam::{DQuat, DVec2, DVec3};
use amg_doc::{BodyId, BodyProps, Command, Document, FeatureId};
use amg_features::am::{ItemGeom, PrintSetup, UnresolvedReason, handoff_plate, resolve_plate};
use amg_features::io::{ImportChoices, ImportSource, import_commands, raw_options};
use amg_io::{ExportOptions, Format, ImportOptions, Sink, Source};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
use amg_mesh::IndexedMesh;

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn box_stl(x: f64, y: f64, z: f64) -> Vec<u8> {
    let t = amg_testkit::shapes::cube(DVec3::ZERO, 1.0);
    let positions = t
        .positions
        .iter()
        .map(|p| DVec3::new(p.x * x, p.y * y, p.z * z))
        .collect();
    let mesh = IndexedMesh::new(positions, t.triangles).unwrap();
    let mut scene = amg_io::Scene::default();
    scene.objects.push(amg_io::SceneObject::new(Arc::new(mesh)));
    scene
        .roots
        .push(amg_io::SceneNode::mesh(0, amg_core::Transform::IDENTITY));
    let mut out = Vec::new();
    amg_io::export(
        &scene,
        Sink::Memory(&mut out),
        Format::Stl,
        &ExportOptions::default(),
        &JobCtx::detached(),
    )
    .unwrap();
    out
}

fn import_box(doc: &mut Document, name: &str, size: (f64, f64, f64)) -> BodyId {
    amg_features::io::link();
    let bytes = box_stl(size.0, size.1, size.2);
    let opts = ImportOptions::default();
    let imported = amg_io::import(
        Source::Bytes(&bytes),
        Some(Format::Stl),
        &raw_options(&opts),
        &JobCtx::detached(),
    )
    .unwrap();
    let choices = ImportChoices::from_options(doc.design().root(), &opts, &imported.report);
    let source = ImportSource::from_bytes(name, Format::Stl, imported.report.declared_unit, &bytes);
    let plan = import_commands(imported, source, &choices).unwrap();
    let fid: FeatureId = plan.apply(doc).unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    doc.evaluated()
        .bodies()
        .find(|(id, _)| id.origin == fid)
        .map(|(id, _)| id)
        .unwrap()
}

fn centre_of(mesh: &IndexedMesh, t: &amg_core::Transform) -> (DVec3, DVec3) {
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = DVec3::splat(f64::NEG_INFINITY);
    for p in &mesh.positions {
        let q = t.transform_point3(*p);
        lo = lo.min(q);
        hi = hi.max(q);
    }
    ((lo + hi) * 0.5, lo)
}

#[test]
fn handoff_plate_resolves_places_names_and_skips() {
    let mut doc = Document::new("AM-06 design");
    let a = import_box(&mut doc, "a.stl", (20.0, 10.0, 8.0));
    let b = import_box(&mut doc, "b.stl", (12.0, 12.0, 15.0));
    let c = import_box(&mut doc, "c.stl", (8.0, 30.0, 5.0));
    let hidden = import_box(&mut doc, "h.stl", (5.0, 5.0, 5.0));
    let machine = MachineLibrary::builtin()
        .get(&MachineKey("builtin:creality/ender-3-v3-ke-kcm".into()))
        .unwrap()
        .clone();
    let mut setup = PrintSetup::new("Setup", machine);
    let plate = setup.plates[0].id;
    let rot = DQuat::from_axis_angle(DVec3::Z, 0.4);
    let place = |r: DQuat, x: f64, y: f64| Placement::on_plate(r, DVec2::new(x, y));
    let ia = setup.add_body(plate, a, place(rot, 100.0, 90.0));
    let ib = setup.add_body(plate, b, place(DQuat::IDENTITY, 140.0, 120.0));
    let ic = setup.add_body(plate, c, place(DQuat::IDENTITY, 60.0, 150.0));
    let ih = setup.add_body(plate, hidden, place(DQuat::IDENTITY, 30.0, 30.0));
    let gone = BodyId {
        origin: FeatureId(9999),
        slot: 0,
    };
    let ig = setup.add_body(plate, gone, place(DQuat::IDENTITY, 0.0, 0.0));
    setup.item_mut(ia).unwrap().filament = Some(1);
    setup.item_mut(ib).unwrap().filament = Some(2);
    setup.item_mut(ic).unwrap().filament = Some(1);
    doc.apply(Command::SetBodyProps {
        body: a,
        props: BodyProps {
            name: Some("Alpha".into()),
            ..BodyProps::default()
        },
    })
    .unwrap();
    doc.apply(Command::SetBodyProps {
        body: hidden,
        props: BodyProps {
            visible: false,
            ..BodyProps::default()
        },
    })
    .unwrap();
    doc.recompute_blocking(jobs()).unwrap();

    let resolved = resolve_plate(&setup, plate, doc.design(), doc.evaluated());
    assert_eq!(resolved.len(), 5);
    assert!(matches!(resolved[0].geom, ItemGeom::Mesh(_)));
    assert!(matches!(
        resolved[3].geom,
        ItemGeom::Unresolved(UnresolvedReason::Hidden)
    ));
    assert!(matches!(
        resolved[4].geom,
        ItemGeom::Unresolved(UnresolvedReason::BodyMissing)
    ));
    assert_eq!(resolved[0].name, "Alpha");

    let (hp, skipped) = handoff_plate(&setup, plate, doc.design(), doc.evaluated());
    assert_eq!(
        skipped,
        [
            (ih, UnresolvedReason::Hidden),
            (ig, UnresolvedReason::BodyMissing)
        ]
    );
    assert_eq!(hp.items.len(), 3);
    assert_eq!(hp.design_name, "AM-06 design");
    assert_eq!(hp.plate_name, setup.plates[0].name);
    assert_eq!(hp.items[0].name, "Alpha");
    assert_eq!(hp.items[1].filament, Some(2));
    // The effective transform puts each item on the plate at its xy centre.
    for (it, xy) in hp
        .items
        .iter()
        .zip([(100.0, 90.0), (140.0, 120.0), (60.0, 150.0)])
    {
        let (c, lo) = centre_of(&it.mesh, &it.transform);
        assert!((c.x - xy.0).abs() < 1e-9 && (c.y - xy.1).abs() < 1e-9);
        assert!(lo.z.abs() < 1e-9);
    }

    // The file re-imported through IO-03 has the plate-frame positions (origin offset undone).
    let dir = std::env::temp_dir().join(format!("amg-am06-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("h.3mf");
    export_plate(
        &hp,
        HandoffFormat::ThreeMf,
        SlicerKind::OrcaSlicer,
        &path,
        &JobCtx::detached(),
    )
    .unwrap();
    let scene = amg_io::import(
        Source::Path(&path),
        None,
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .unwrap()
    .scene;
    let origin = setup.machine.volume.slicer_origin;
    let mut got: Vec<(String, Option<u8>, DVec3)> = scene
        .instances()
        .iter()
        .map(|(i, t)| {
            let o = &scene.objects[*i as usize];
            let (c, _) = centre_of(&o.mesh, t);
            let c = c + DVec3::new(origin.x, origin.y, 0.0);
            (o.name.clone().unwrap(), o.filament, c)
        })
        .collect();
    got.sort_by(|x, y| x.0.cmp(&y.0));
    assert_eq!(got.len(), 3);
    assert_eq!(got[0].0, "Alpha");
    assert!((got[0].2.x - 100.0).abs() < 1e-6 && (got[0].2.y - 90.0).abs() < 1e-6);
    assert_eq!(
        got.iter().map(|g| g.1).collect::<Vec<_>>(),
        [Some(1), Some(2), Some(1)]
    );
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn unknown_plate_resolves_to_nothing() {
    let doc = Document::new("x");
    let machine = MachineLibrary::builtin()
        .get(&MachineKey("builtin:creality/ender-3-v3-ke-kcm".into()))
        .unwrap()
        .clone();
    let setup = PrintSetup::new("S", machine);
    let bogus = amg_features::am::PlateId(77);
    assert!(resolve_plate(&setup, bogus, doc.design(), doc.evaluated()).is_empty());
    let (hp, skipped) = handoff_plate(&setup, bogus, doc.design(), doc.evaluated());
    assert!(hp.items.is_empty() && skipped.is_empty());
}
