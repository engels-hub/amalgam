//! INS-10: the Properties panel on assemblies and solid bodies, and "Display Component
//! Colours", in the real app (`AppHarness`, kittest, CPU scene backend).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "test code"
)]

use std::collections::BTreeMap;
use std::sync::{Arc, OnceLock};
use std::time::Duration;

use amg_core::{Point3, Transform, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, BodyProps, Command, ComponentId, CreatedId,
    Dependency, Document, EntityRef, Feature, FeatureBox, FeatureError, FeatureOutput,
    OccurrenceId, RecomputeCtx, register_feature,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_mesh::IndexedMesh;
use amg_render::Material;
use amg_ui::CommitBatch;
use amg_ui::command::Placement;
use amg_ui::design::tabs::INSPECT;
use amg_ui::inspect::{COMPONENT_COLORS_CMD, PROPERTIES_CMD};
use amg_ui::marking::{ContextTarget, context_menu_commands};
use amg_ui::pick::PickTarget;
use amg_ui::scene::{CpuSceneBackend, RenderEntityKey};
use amg_ui::search::{Recency, SearchIndex};
use amg_ui::select::SelItem;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- fixtures ---------------------------------------------------------------------------------

/// An axis-aligned 10 mm cube mesh body. `serial` tells two cubes of one component apart:
/// the recompute cache key does not contain the feature id, so identical features would share
/// one output.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Cube {
    serial: u32,
}

impl Feature for Cube {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(body(ctx, BodyGeom::Mesh(Arc::new(cube(10.0)))))
    }
}

register_feature!(Cube, "uitest.ins10_cube", version = 1);

/// A solid body without B-rep geometry (this crate's tests run without the kernel).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Placeholder {}

impl Feature for Placeholder {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(body(ctx, BodyGeom::Solid(amg_doc::stub::Shape::new())))
    }
}

register_feature!(Placeholder, "uitest.ins10_solid", version = 1);

fn body(ctx: &mut RecomputeCtx<'_>, geom: BodyGeom) -> FeatureOutput {
    FeatureOutput {
        bodies: vec![BodyChange {
            id: ctx.new_body_id(0),
            change: BodyChangeKind::Created { geom, naming: None },
        }],
        ..FeatureOutput::default()
    }
}

fn cube(s: f64) -> IndexedMesh {
    let v = (0..8)
        .map(|i| {
            Point3::new(
                f64::from(i & 1),
                f64::from((i >> 1) & 1),
                f64::from((i >> 2) & 1),
            ) * s
        })
        .collect();
    let t = vec![
        [0, 2, 3],
        [0, 3, 1],
        [4, 5, 7],
        [4, 7, 6],
        [0, 1, 5],
        [0, 5, 4],
        [2, 6, 7],
        [2, 7, 3],
        [0, 4, 6],
        [0, 6, 2],
        [1, 3, 7],
        [1, 7, 5],
    ];
    IndexedMesh::new(v, t).unwrap()
}

fn add_command(feature: FeatureBox, context: ComponentId) -> Command {
    Command::AddFeature {
        at: None,
        feature,
        context,
        name: None,
    }
}

fn add(doc: &mut Document, feature: FeatureBox, context: ComponentId) -> BodyId {
    let out = doc.apply(add_command(feature, context)).unwrap();
    let f = out
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap();
    BodyId { origin: f, slot: 0 }
}

fn recompute(doc: &mut Document) {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    let jobs = JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap());
    doc.recompute_blocking(jobs).unwrap();
}

/// A component "Part" with one cube, placed at x = 100 and x = 200.
struct Assembly {
    doc: Document,
    part: ComponentId,
    body: BodyId,
    occurrences: [OccurrenceId; 2],
}

fn assembly() -> Assembly {
    let mut doc = Document::new("Rig");
    let root = doc.design().root();
    let at = |x: f64| Transform::from_translation(Vec3::new(x, 0.0, 0.0));
    let out = doc
        .apply(Command::NewComponent {
            parent: root,
            name: "Part".into(),
            transform: at(100.0),
        })
        .unwrap();
    let (mut part, mut first) = (None, None);
    for id in out.created {
        match id {
            CreatedId::Component(c) => part = Some(c),
            CreatedId::Occurrence(o) => first = Some(o),
            _ => {}
        }
    }
    let (part, first) = (part.unwrap(), first.unwrap());
    let body = add(&mut doc, FeatureBox::new(Cube { serial: 0 }), part);
    let out = doc
        .apply(Command::AddOccurrence {
            parent: root,
            component: part,
            transform: at(200.0),
            name: None,
        })
        .unwrap();
    let second = out
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Occurrence(o) => Some(*o),
            _ => None,
        })
        .unwrap();
    recompute(&mut doc);
    Assembly {
        doc,
        part,
        body,
        occurrences: [first, second],
    }
}

// ---- helpers ------------------------------------------------------------------------------------

/// Steps until a label containing `text` is shown (the panel's job finished), at most ~20 s.
fn wait_for(h: &mut AppHarness, text: &str) {
    for _ in 0..4000 {
        if shown(h, text) {
            return;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("no label containing {text:?} appeared");
}

fn shown(h: &mut AppHarness, text: &str) -> bool {
    h.kittest().step();
    h.kittest()
        .query_all_by_label_contains(text)
        .next()
        .is_some()
}

fn open(h: &mut AppHarness) {
    h.drive_until_idle();
    h.run_command(PROPERTIES_CMD);
    h.drive_until_idle();
    h.kittest().step();
}

fn settle(h: &mut AppHarness) {
    h.drive_until_idle();
    h.kittest().step();
    h.kittest().step();
}

fn select(h: &mut AppHarness, item: SelItem) {
    let sel = &mut h.app_mut().active_document_mut().selection;
    sel.clear();
    sel.add(item);
}

fn materials(h: &AppHarness) -> BTreeMap<RenderEntityKey, Material> {
    h.scene()
        .backend()
        .as_any()
        .downcast_ref::<CpuSceneBackend>()
        .expect("default backend")
        .instances()
        .map(|(_, i)| (i.key, i.material))
        .collect()
}

fn colors_of(h: &AppHarness, body: BodyId) -> Vec<[f32; 4]> {
    let all = materials(h);
    h.scene()
        .pick_map()
        .body_keys(body)
        .iter()
        .map(|k| all[k].base_color)
        .collect()
}

// ---- assemblies ---------------------------------------------------------------------------------

#[test]
fn an_assembly_shows_its_total_and_the_occurrence_and_component_totals() {
    let a = assembly();
    let [o0, o1] = a.occurrences;
    let mut h = AppHarness::with_document(a.doc);
    open(&mut h);
    // Two placements of a 1000 mm³ cube, centred at x = 105 and 205.
    wait_for(&mut h, "Volume: 2000 mm³");
    assert!(shown(&mut h, "1 body, 2 body instances"));
    assert!(shown(&mut h, "Centre of mass: (155, 5, 5) mm"));
    assert!(shown(&mut h, "Mass: 2.48 g (default material)"));
    assert!(shown(&mut h, "Occurrence Part:1 (1 body instance)"));
    assert!(shown(&mut h, "Occurrence Part:2 (1 body instance)"));
    assert!(shown(&mut h, "Component Part (2 body instances)"));
    assert!(shown(&mut h, "Component Rig (2 body instances)"));
    h.kittest()
        .get_by_label("Occurrence Part:2 (1 body instance)")
        .click();
    settle(&mut h);
    assert!(shown(&mut h, "Centre of mass: (205, 5, 5) mm"));

    // A selected occurrence (component pick priority, browser) measures that placement.
    select(
        &mut h,
        SelItem {
            target: PickTarget::Occurrence(o1),
            entity: EntityRef::InContext {
                path: vec![o1],
                target: Box::new(EntityRef::Body(a.body)),
            },
        },
    );
    wait_for(&mut h, "Volume: 1000 mm³");
    assert!(shown(&mut h, "Centre of mass: (205, 5, 5) mm"));
    assert!(!shown(&mut h, "Occurrence Part:1"));

    // A picked body instance measures the placement it was picked through.
    for (occurrence, x) in [(o1, "205"), (o0, "105")] {
        let instance = h
            .scene()
            .pick_map()
            .instances_of(a.body)
            .into_iter()
            .find(|i| h.scene().occurrence_path(*i) == Some(&vec![occurrence]))
            .expect("displayed instance");
        let item = SelItem::from_target(PickTarget::Body {
            body: a.body,
            instance,
            sub: None,
        })
        .unwrap();
        select(&mut h, item);
        wait_for(&mut h, &format!("Centre of mass: ({x}, 5, 5) mm"));
    }
}

#[test]
fn moving_an_occurrence_recomputes_the_totals() {
    let a = assembly();
    let o1 = a.occurrences[1];
    let mut h = AppHarness::with_document(a.doc);
    open(&mut h);
    wait_for(&mut h, "Centre of mass: (155, 5, 5) mm");
    h.app_mut().submit(CommitBatch {
        label: "Move".into(),
        commands: vec![Command::SetOccurrenceTransform {
            occ: o1,
            transform: Transform::from_translation(Vec3::new(300.0, 0.0, 0.0)),
        }],
    });
    h.drive_until_idle();
    wait_for(&mut h, "Centre of mass: (205, 5, 5) mm");
}

// ---- solids -------------------------------------------------------------------------------------

#[test]
fn a_solid_without_kernel_is_reported_and_left_out_of_the_totals() {
    let mut doc = Document::new("Mixed");
    let root = doc.design().root();
    let solid = add(&mut doc, FeatureBox::new(Placeholder {}), root);
    recompute(&mut doc);
    let mut h = AppHarness::with_document(doc);
    h.drive_until_idle();
    // Offered for a solid body.
    let item = SelItem::from_target(PickTarget::Body {
        body: solid,
        instance: 0,
        sub: None,
    })
    .unwrap();
    let cmds = h.app_mut().with_cx(|cx| {
        let env = cx.env();
        context_menu_commands(cx.registry(), &env, &ContextTarget::Viewport(Some(&item)))
    });
    assert!(cmds.contains(&PROPERTIES_CMD), "{cmds:?}");
    open(&mut h);
    // The default build says "backend unavailable"; with the kernel linked in, the
    // placeholder has no geometry. Either way no value is shown.
    let reason =
        |h: &mut AppHarness| shown(h, "backend unavailable") || shown(h, "has no B-rep geometry");
    wait_for(&mut h, &format!("Body {}.{}: ", solid.origin.0, solid.slot));
    assert!(reason(&mut h));
    assert!(!shown(&mut h, "Volume:"));
    // With a mesh body next to it the mesh is measured and the totals say what they miss.
    h.app_mut().submit(CommitBatch {
        label: "Cube".into(),
        commands: vec![add_command(FeatureBox::new(Cube { serial: 0 }), root)],
    });
    h.drive_until_idle();
    wait_for(&mut h, "Volume: 1000 mm³");
    assert!(shown(&mut h, "The totals leave out 1 body instance."));
    assert!(reason(&mut h));
}

// ---- component colours ----------------------------------------------------------------------

/// Done-when 4: the toggle colours each component distinctly and restores the appearances.
#[test]
fn component_colours_are_applied_per_component_and_restored() {
    let mut a = assembly();
    let root = a.doc.design().root();
    let own = add(&mut a.doc, FeatureBox::new(Cube { serial: 0 }), root);
    a.doc
        .apply(Command::SetBodyProps {
            body: own,
            props: BodyProps {
                opacity: 0.5,
                ..BodyProps::default()
            },
        })
        .unwrap();
    recompute(&mut a.doc);
    let mut h = AppHarness::with_document(a.doc);
    settle(&mut h);
    let before = materials(&h);
    assert_eq!(before.len(), 3, "one root body and two placements");
    let checked = |h: &mut AppHarness| {
        h.app_mut().with_cx(|cx| {
            let env = cx.env();
            cx.registry()
                .command(COMPONENT_COLORS_CMD)
                .unwrap()
                .checked(&env)
        })
    };
    assert!(!checked(&mut h));
    let undo = h.document().history().stack().applied();

    h.run_command(COMPONENT_COLORS_CMD);
    settle(&mut h);
    assert!(checked(&mut h));
    let part = colors_of(&h, a.body);
    let rig = colors_of(&h, own);
    assert_eq!((part.len(), rig.len()), (2, 1));
    assert_eq!(part[0], part[1], "one colour per component");
    assert_ne!(part[0][..3], rig[0][..3], "components differ");
    assert_eq!((part[0][3], rig[0][3]), (1.0, 0.5), "opacity is kept");
    for (key, m) in materials(&h) {
        assert_ne!(m.base_color, before[&key].base_color);
    }

    // A body added while the colours are on takes its component's colour.
    h.app_mut().submit(CommitBatch {
        label: "Cube".into(),
        commands: vec![add_command(FeatureBox::new(Cube { serial: 1 }), a.part)],
    });
    // The recompute and the upload run on jobs.
    for _ in 0..4000 {
        settle(&mut h);
        if materials(&h).len() == 5 {
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(
        materials(&h).len(),
        5,
        "the new body is shown in both placements"
    );
    let newest = h.document().evaluated().bodies().map(|(id, _)| id).max();
    let newest = newest.unwrap();
    assert_ne!(newest, own);
    assert_eq!(colors_of(&h, newest), part);

    h.run_command(COMPONENT_COLORS_CMD);
    settle(&mut h);
    assert!(!checked(&mut h));
    let after = materials(&h);
    for (key, m) in &before {
        assert_eq!(&after[key], m, "restored");
    }
    assert!(h.scene().body_tints().is_none());
    // View state only: no undo entry besides the added body's.
    assert_eq!(h.document().history().stack().applied(), undo + 1);
}

#[test]
fn the_colour_command_and_the_panel_checkbox_share_the_state() {
    let a = assembly();
    let mut h = AppHarness::with_document(a.doc);
    h.drive_until_idle();
    {
        let reg = h.app().registry();
        let def = reg.command(COMPONENT_COLORS_CMD).expect("registered");
        assert!(def.placements.iter().any(|p| matches!(
            p,
            Placement::Toolbar { group, .. } if reg.group(*group).map(|g| g.tab) == Some(INSPECT)
        )));
        let index = SearchIndex::build(reg);
        assert!(
            index
                .search("component colour", &Recency::new())
                .iter()
                .any(|x| x.id == COMPONENT_COLORS_CMD)
        );
    }
    open(&mut h);
    wait_for(&mut h, "Volume:");
    let before = materials(&h);
    let click = |h: &mut AppHarness| {
        h.kittest()
            .get_by_role_and_label(Role::CheckBox, "Display component colours")
            .click();
        settle(h);
    };
    click(&mut h);
    assert!(h.scene().body_tints().is_some());
    assert_ne!(materials(&h), before);
    // The command turns off what the checkbox turned on.
    h.run_command(COMPONENT_COLORS_CMD);
    settle(&mut h);
    assert_eq!(materials(&h), before);
    // Another document starts with its materials.
    click(&mut h);
    h.app_mut().new_document();
    settle(&mut h);
    assert!(h.scene().body_tints().is_none());
}
