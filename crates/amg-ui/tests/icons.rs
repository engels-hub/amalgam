//! UI-23 tests: the SVG icon registry, the per-scale raster cache and the icons drawn in the
//! toolbar, the marking menu, the browser and the timeline (real app in `AppHarness`).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::time::Instant;

use amg_doc::{Command, Dependency, FeatureBox, FeatureError, FeatureOutput, RecomputeCtx};
use amg_ui::command::{CmdAction, CommandDef, Placement};
use amg_ui::icons::{self, IconCache, ids};
use amg_ui::mesh::{MESH_TAB, MeshOp};
use amg_ui::module::{IconId, Registrar, UiModule};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use amg_ui::timeline::{EditAction, FeatureUiInfo};
use amg_ui::{CommitBatch, UiRegistry};
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- test module: two features (one with an icon) and commands with a missing icon -----------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Quiet;

impl amg_doc::Feature for Quiet {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }
    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(Quiet, "uitest.ic_known", version = 1);

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Plain;

impl amg_doc::Feature for Plain {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }
    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(Plain, "uitest.ic_plain", version = 1);

const NO_ICON_CMD: &str = "uitest.icon_none";
const BAD_ICON_CMD: &str = "uitest.icon_unknown";

fn register_test_module(r: &mut Registrar<'_>) {
    r.feature_ui(FeatureUiInfo {
        kind: "uitest.ic_known",
        display_name: "Known",
        icon: ids::CUBE_PLUS,
        edit: EditAction::NotEditable,
    });
    let place = |order| Placement::Toolbar {
        group: amg_ui::mesh::ops::GROUP_CREATE,
        pinned: true,
        order,
    };
    r.command(
        CommandDef::new(NO_ICON_CMD, "Plain Label Cmd", CmdAction::Run(|_| {})).placed(place(900)),
    );
    r.command(
        CommandDef::new(BAD_ICON_CMD, "Unknown Icon Cmd", CmdAction::Run(|_| {}))
            .with_icon(IconId("does-not-exist"))
            .placed(place(910)),
    );
}

inventory::submit! { UiModule { id: "zz_icons_test", register: register_test_module } }

// ---- helpers ------------------------------------------------------------------------------------

fn harness() -> AppHarness {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h
}

fn cache(h: &mut AppHarness) -> IconCache {
    icons::cache(&h.kittest().ctx).expect("the app installs the icon cache")
}

fn collect_textures(shape: &egui::Shape, out: &mut Vec<egui::TextureId>) {
    match shape {
        egui::Shape::Vec(v) => v.iter().for_each(|s| collect_textures(s, out)),
        egui::Shape::Mesh(m) => out.push(m.texture_id),
        egui::Shape::Rect(r) => {
            out.push(r.fill_texture_id());
        }
        _ => {}
    }
}

/// Whether the last frame painted a quad textured with (any size of) icon `id`.
fn drawn(h: &mut AppHarness, id: IconId) -> bool {
    let want = cache(h).texture_ids(id);
    assert!(!want.is_empty(), "{} was never rasterised", id.0);
    let mut used = Vec::new();
    for s in &h.kittest().output().shapes {
        collect_textures(&s.shape, &mut used);
    }
    want.iter().any(|t| used.contains(t))
}

fn add(h: &mut AppHarness, f: FeatureBox) {
    let cmd = Command::AddFeature {
        at: None,
        feature: f,
        context: h.document().design().root(),
        name: None,
    };
    h.app_mut().submit(CommitBatch {
        label: "Add".into(),
        commands: vec![cmd],
    });
    h.drive_until_idle();
}

fn svg_of(id: IconId) -> &'static [u8] {
    icons::BUILTIN.iter().find(|(i, _)| *i == id).unwrap().1
}

// ---- Done-when 1: icons draw on every surface; commands without an icon are listed -----------

#[test]
fn every_registered_command_has_an_icon_that_exists() {
    let reg = UiRegistry::build();
    let own = |c: &&CommandDef| !c.id.0.starts_with("uitest.");
    let missing: Vec<&str> = reg
        .commands()
        .filter(own)
        .filter(|c| c.icon == IconId::NONE)
        .map(|c| c.id.0)
        .collect();
    assert!(
        missing.is_empty(),
        "commands without an IconId: {missing:?}"
    );
    let unregistered: Vec<_> = reg
        .commands()
        .filter(own)
        .filter(|c| !reg.has_icon(c.icon))
        .map(|c| (c.id.0, c.icon.0))
        .collect();
    assert!(
        unregistered.is_empty(),
        "icons not in the registry: {unregistered:?}"
    );
}

#[test]
fn toolbar_icons_are_drawn_with_their_texture() {
    let mut h = harness();
    h.app_mut().select_tab(MESH_TAB);
    h.drive_until_idle();
    assert!(
        drawn(&mut h, MeshOp::Repair.icon()),
        "Repair icon in the toolbar"
    );
    assert!(
        drawn(&mut h, MeshOp::PlaneCut.icon()),
        "Plane Cut icon in the toolbar"
    );
    // The label stays the accessible name.
    assert!(
        h.kittest()
            .query_by_role_and_label(Role::Button, &MeshOp::Repair.label())
            .is_some()
    );
}

#[test]
fn marking_menu_wedges_draw_their_icon() {
    let mut h = harness();
    let c = h.camera_view().rect.center();
    h.app_mut().open_marking_menu(c);
    h.drive_until_idle();
    assert!(h.app().marking_menu().is_some());
    // The framework's default W wedge is Undo.
    assert!(drawn(&mut h, ids::UNDO), "undo wedge icon");
    assert!(
        h.kittest().query_all_by_label("Undo").count() >= 1,
        "the wedge label stays"
    );
}

#[test]
fn browser_nodes_draw_their_icon() {
    let mut h = harness();
    assert!(drawn(&mut h, ids::ORIGIN), "Origin node icon");
    assert!(drawn(&mut h, ids::SETTINGS), "Document Settings node icon");
    assert!(h.kittest().query_by_label("Origin").is_some());
}

#[test]
fn timeline_nodes_draw_registered_and_generic_icons() {
    let mut h = harness();
    add(&mut h, FeatureBox::new(Quiet));
    add(&mut h, FeatureBox::new(Plain));
    assert!(drawn(&mut h, ids::CUBE_PLUS), "node with a registered icon");
    assert!(
        drawn(&mut h, ids::GENERIC),
        "unknown kind uses the generic icon"
    );
}

// ---- Done-when 2: rasterised per scale, cached --------------------------------------------------

#[test]
fn rasterising_happens_only_on_a_scale_change() {
    let mut h = harness();
    h.app_mut().select_tab(MESH_TAB);
    h.kittest().set_pixels_per_point(1.0);
    h.drive_until_idle();
    let n1 = cache(&mut h).raster_count();
    assert!(n1 > 0);
    h.kittest().run_steps(5);
    assert_eq!(
        cache(&mut h).raster_count(),
        n1,
        "no re-rasterising at the same scale"
    );
    h.kittest().set_pixels_per_point(1.5);
    h.drive_until_idle();
    let n2 = cache(&mut h).raster_count();
    assert!(n2 > n1, "a scale change re-rasterises");
    h.kittest().run_steps(5);
    assert_eq!(cache(&mut h).raster_count(), n2);
    h.kittest().set_pixels_per_point(2.0);
    h.drive_until_idle();
    let n3 = cache(&mut h).raster_count();
    assert!(n3 > n2);
    assert!(drawn(&mut h, MeshOp::Repair.icon()));
}

#[test]
fn rasters_have_the_pixel_size_of_the_scale() {
    for (ppp, px) in [(1.0_f32, 16_u32), (1.5, 24), (2.0, 32)] {
        let want = (icons::SIZE_PT * ppp).round() as u32;
        assert_eq!(want, px);
        let img = icons::rasterise(svg_of(ids::SEARCH), px).unwrap();
        assert_eq!(img.size, [px as usize, px as usize]);
    }
}

// ---- Done-when 3: unknown / NONE fall back to the label ---------------------------------------

#[test]
fn unknown_and_none_icons_fall_back_to_the_label() {
    let mut h = harness();
    h.app_mut().select_tab(MESH_TAB);
    h.drive_until_idle();
    for label in ["Plain Label Cmd", "Unknown Icon Cmd"] {
        assert!(
            h.kittest()
                .query_by_role_and_label(Role::Button, label)
                .is_some(),
            "{label} still shows as a labelled button"
        );
    }
    let c = cache(&mut h);
    let ctx = h.kittest().ctx.clone();
    assert!(c.texture(&ctx, IconId::NONE, 16.0).is_none());
    assert!(c.texture(&ctx, IconId("does-not-exist"), 16.0).is_none());
    assert!(!c.contains(IconId("does-not-exist")));
}

// ---- Done-when 4: cold start --------------------------------------------------------------------

#[test]
fn rasterising_every_icon_cold_is_fast() {
    let reg = UiRegistry::build();
    let ctx = egui::Context::default();
    let cache = IconCache::new(&reg);
    let ids: Vec<IconId> = icons::BUILTIN.iter().map(|(i, _)| *i).collect();
    let t = Instant::now();
    for id in &ids {
        assert!(
            cache.texture(&ctx, *id, icons::SIZE_PT).is_some(),
            "{}",
            id.0
        );
    }
    let ms = t.elapsed().as_secs_f64() * 1e3;
    eprintln!(
        "UI-23 cold start: {} icons rasterised in {ms:.2} ms",
        ids.len()
    );
    if std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1") {
        assert!(ms < 20.0, "cold start {ms:.2} ms >= 20 ms");
    }
}

// ---- Done-when 5: licence ------------------------------------------------------------------------

#[test]
fn icon_set_licence_is_vendored_and_listed() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let lic = std::fs::read_to_string(root.join("assets/icons/LICENSE-tabler.txt")).unwrap();
    assert!(lic.contains("MIT License") && lic.contains("Permission is hereby granted"));
    let third = std::fs::read_to_string(root.join("../../THIRD_PARTY.md")).unwrap();
    assert!(
        third.contains("Tabler Icons"),
        "THIRD_PARTY.md lists the icon set"
    );
}
