//! Construction geometry UI (lane A, S05 §7, CON-04).
//!
//! - **Commands.** The 20 Fusion Construct variants ([`COMMANDS`]) are in SOLID ▸ Construct,
//!   SURFACE ▸ Construct and S-key search. Each opens the generic dialog (UI-05) of its feature
//!   kind on that variant, filled from the selection; the schemas are in
//!   `amg_features::construct::dialog`. Editing a construction feature (timeline, browser
//!   double-click) opens the same dialog through the generated `feature.construct.*` command.
//! - **Display.** [`ConstructionLayer`] draws every visible construction plane, axis and point
//!   (sized by `construct::display::display_extent`) as RND-07 overlays with the geometry's
//!   pick id as handle, for every visible occurrence of its component, and hands the shapes to
//!   the CPU pick service. While a dialog previews, the previewed geometry is drawn too.
//! - **Browser.** The framework lists them in each component's "Construction" folder (eye,
//!   rename, select, edit); this module adds "Delete" to their row menu.
//! - [`base_plane`] and [`world_ref`] let other tools (Section Analysis) use a picked
//!   construction plane.

use amg_core::Plane;
use amg_doc::{
    Command, ConstructionGeom, ConstructionId, Design, DocEvent, EntityRef, Evaluated, FeatureId,
};
use amg_features::construct::dialog::{self as schema, AXIS_KIND, PLANE_KIND, POINT_KIND};
use amg_features::construct::display::{ConstructionDisplay, display_extent, placed};
use amg_features::construct::{RefInput, math, resolve};
use amg_features::dialog::{DialogInit, registration};
use amg_render::{DepthMode, LineStyle, OverlayList, PickId, PointStyle};

use crate::app::{AppCx, CmdEnv, CommitBatch};
use crate::browser::{BrowserAnchor, BrowserNode, BrowserProvider, NodeKey, visibility};
use crate::command::{CmdAction, CmdId, CommandDef, Placement};
use crate::feature_dialog::{self, GenericFeatureTool};
use crate::icons::ids;
use crate::layer::{DocLayer, DocLayerDef, LayerOut};
use crate::module::{GroupId, IconId, Registrar, UiModule};
use crate::pick::PickTarget;
use crate::scene::ConstructionPick;
use crate::tool::{Tool, ToolInit};

inventory::submit! { UiModule { id: "construct", register } }

/// Id of the construction display layer.
pub const LAYER_ID: &str = "construct.display";

/// A menu command: one variant of a construction dialog.
#[derive(Clone, Copy, Debug)]
pub struct ConstructCommand {
    /// Command id.
    pub id: CmdId,
    /// Feature kind.
    pub kind: &'static str,
    /// Variant id in the dialog's "Type" chooser.
    pub variant: &'static str,
    /// Extra search words.
    pub keywords: &'static [&'static str],
}

macro_rules! commands {
    ($(($id:literal, $kind:ident, $variant:literal, [$($word:literal),*])),+ $(,)?) => {
        [$(ConstructCommand {
            id: CmdId($id),
            kind: $kind,
            variant: $variant,
            keywords: &["construct" $(, $word)*],
        }),+]
    };
}

/// The 20 construction commands in menu order (their labels are the variants' labels).
pub const COMMANDS: [ConstructCommand; 20] = commands![
    (
        "construct.offset_plane",
        PLANE_KIND,
        "offset",
        ["plane", "offset", "datum"]
    ),
    (
        "construct.plane_at_angle",
        PLANE_KIND,
        "at_angle",
        ["plane", "angle", "rotate"]
    ),
    (
        "construct.tangent_plane",
        PLANE_KIND,
        "tangent",
        ["plane", "tangent", "cylinder"]
    ),
    (
        "construct.midplane",
        PLANE_KIND,
        "midplane",
        ["plane", "middle", "between"]
    ),
    (
        "construct.plane_two_edges",
        PLANE_KIND,
        "two_edges",
        ["plane", "edges", "lines"]
    ),
    (
        "construct.plane_three_points",
        PLANE_KIND,
        "three_points",
        ["plane", "points"]
    ),
    (
        "construct.plane_tangent_at_point",
        PLANE_KIND,
        "tangent_at_point",
        ["plane", "tangent", "point"]
    ),
    (
        "construct.plane_along_path",
        PLANE_KIND,
        "along_path",
        ["plane", "path", "curve"]
    ),
    (
        "construct.axis_revolved",
        AXIS_KIND,
        "revolved",
        ["axis", "cylinder", "cone", "torus", "centre"]
    ),
    (
        "construct.axis_perpendicular_at_point",
        AXIS_KIND,
        "perpendicular_at_point",
        ["axis", "normal"]
    ),
    (
        "construct.axis_two_planes",
        AXIS_KIND,
        "two_planes",
        ["axis", "planes", "intersection"]
    ),
    (
        "construct.axis_two_points",
        AXIS_KIND,
        "two_points",
        ["axis", "points"]
    ),
    (
        "construct.axis_through_edge",
        AXIS_KIND,
        "edge",
        ["axis", "edge", "line"]
    ),
    (
        "construct.axis_perpendicular_to_face",
        AXIS_KIND,
        "perpendicular_to_face",
        ["axis", "normal", "face"]
    ),
    (
        "construct.point_at_vertex",
        POINT_KIND,
        "vertex",
        ["point", "vertex"]
    ),
    (
        "construct.point_two_edges",
        POINT_KIND,
        "two_edges",
        ["point", "edges", "intersection"]
    ),
    (
        "construct.point_three_planes",
        POINT_KIND,
        "three_planes",
        ["point", "planes", "intersection"]
    ),
    (
        "construct.point_at_center",
        POINT_KIND,
        "center",
        ["point", "center", "centre", "circle", "sphere"]
    ),
    (
        "construct.point_edge_plane",
        POINT_KIND,
        "edge_plane",
        ["point", "edge", "plane", "intersection"]
    ),
    (
        "construct.point_along_path",
        POINT_KIND,
        "along_path",
        ["point", "path", "curve"]
    ),
];

/// The menu command `id`.
pub fn command(id: CmdId) -> Option<&'static ConstructCommand> {
    COMMANDS.iter().find(|c| c.id == id)
}

fn icon_of(kind: &str) -> IconId {
    match kind {
        PLANE_KIND => ids::RECT_3PT,
        AXIS_KIND => ids::ORIGIN,
        _ => ids::POINT,
    }
}

/// The `StartTool` factory of every construction command: the dialog of its kind on its
/// variant, with the preselection in the inputs it fits.
fn start(init: &ToolInit<'_>) -> Box<dyn Tool> {
    let dialog = command(init.command)
        .filter(|_| init.edit.is_none())
        .and_then(|c| {
            let refs: Vec<EntityRef> = init.preselection.iter().map(|i| i.entity.clone()).collect();
            let values = schema::variant_values(
                c.kind,
                c.variant,
                &DialogInit {
                    preselection: &refs,
                    design: init.env.design,
                    evaluated: init.env.evaluated,
                },
                schema::Unfilled::Pick,
            )?;
            GenericFeatureTool::with_defaults(registration(c.kind)?, init, values).ok()
        });
    match dialog {
        Some(t) => Box::new(t),
        // Edits and anything unknown: the generic factory explains what is wrong.
        None => feature_dialog::start(init),
    }
}

/// Registers the commands, the display layer and the browser row menu.
pub fn register(r: &mut Registrar<'_>) {
    for (order, c) in (0..).zip(&COMMANDS) {
        let label = schema::variant(c.kind, c.variant).map_or(c.id.0, |v| v.label);
        let place = |group| Placement::Toolbar {
            group: GroupId(group),
            // Not pinned: a pinned button would change the UI-01 shell snapshots.
            pinned: false,
            order,
        };
        r.command(
            CommandDef::new(c.id.0, label, CmdAction::StartTool(start))
                .with_icon(icon_of(c.kind))
                .with_tooltip(label)
                .with_keywords(c.keywords)
                .placed(place("design.solid.construct"))
                .placed(place("design.surface.construct")),
        );
    }
    r.doc_layer(DocLayerDef {
        id: LAYER_ID,
        create: || Box::new(ConstructionLayer::default()),
    });
    r.browser_provider(Box::new(RowMenu));
}

// ---- other tools ------------------------------------------------------------------------------

/// The reference a tool outside the recompute should store for construction geometry picked in
/// the view: with the occurrence path when its component is placed exactly once below the root,
/// so it resolves where it is seen.
pub fn world_ref(design: &Design, c: ConstructionId) -> EntityRef {
    let plain = EntityRef::Construction(c);
    let Some((_, node)) = design.timeline().node(c.0) else {
        return plain;
    };
    match design.occurrence_paths(node.context).as_slice() {
        [path] if !path.is_empty() => EntityRef::InContext {
            path: path.clone(),
            target: Box::new(plain),
        },
        _ => plain,
    }
}

/// Whether `e` is construction geometry, possibly seen through occurrences.
fn is_construction(e: &EntityRef) -> bool {
    match e {
        EntityRef::Construction(_) => true,
        EntityRef::InContext { target, .. } => is_construction(target),
        _ => false,
    }
}

/// The world plane of a construction plane reference; `None` if it does not resolve to a plane
/// or is not construction geometry (faces need a kernel query: S05 INS-11). Cheap enough for
/// the UI thread.
pub fn base_plane(design: &Design, eval: &Evaluated, e: &EntityRef) -> Option<Plane> {
    if !is_construction(e) {
        return None;
    }
    let resolved = resolve::resolve_in(eval, design, &RefInput::Ref(e.clone())).ok()?;
    math::plane_of(&resolved.geom).ok()
}

// ---- display ----------------------------------------------------------------------------------

/// One construction display of the current frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shown {
    /// The construction geometry.
    pub id: ConstructionId,
    /// What is drawn, world space.
    pub shape: ConstructionDisplay,
    /// The pick id of its overlay handle (`None` for a preview, which is not pickable).
    pub handle: Option<PickId>,
}

/// The per-document construction display (see the module docs).
#[derive(Debug, Default)]
pub struct ConstructionLayer {
    shown: Vec<Shown>,
    preview: Vec<Shown>,
}

impl ConstructionLayer {
    /// What was drawn in the last frame, in id order (one entry per visible occurrence).
    pub fn shown(&self) -> &[Shown] {
        &self.shown
    }

    /// The previewed geometry drawn in the last frame.
    pub fn preview(&self) -> &[Shown] {
        &self.preview
    }

    /// The first display of `id`.
    pub fn display_of(&self, id: ConstructionId) -> Option<ConstructionDisplay> {
        self.shown.iter().find(|s| s.id == id).map(|s| s.shape)
    }
}

/// The world geometry of `geom` of component-space feature `id`: one per occurrence path of its
/// component whose occurrences are all visible.
fn instances(
    design: &Design,
    eval: &Evaluated,
    context: amg_doc::ComponentId,
    geom: &ConstructionGeom,
) -> Vec<ConstructionGeom> {
    design
        .occurrence_paths(context)
        .iter()
        .filter(|path| {
            path.iter()
                .all(|o| design.occurrence(*o).is_some_and(|o| o.visible))
        })
        .filter_map(|path| {
            if path.is_empty() {
                Some(*geom)
            } else {
                Some(placed(geom, &eval.world_transform(path)?))
            }
        })
        .collect()
}

/// Linear RGB of the theme's construction orange and of the preview blue.
const COLOR: [f32; 3] = [0.871, 0.305, 0.021];
const PREVIEW_COLOR: [f32; 3] = [0.051, 0.216, 0.888];

fn draw(out: &mut OverlayList, s: &Shown, rgb: [f32; 3]) {
    let color = |a| [rgb[0], rgb[1], rgb[2], a];
    let handle = s.handle.map(|id| (id, 0));
    // Axes and points stay visible, dimmed, behind bodies; planes are hidden by them.
    let through = DepthMode::XRay { hidden_alpha: 0.35 };
    match s.shape {
        ConstructionDisplay::PlaneSquare { frame, half_size } => out.plane_quad(
            &frame,
            [half_size; 2],
            color(0.18),
            LineStyle {
                width_px: 1.5,
                color: color(0.9),
                dash: None,
                depth: DepthMode::Test,
            },
            handle,
        ),
        ConstructionDisplay::AxisSegment(seg) => out.polyline(
            &[seg.a, seg.b],
            LineStyle {
                width_px: 2.0,
                color: color(1.0),
                dash: Some((10.0, 4.0)),
                depth: through,
            },
            handle,
        ),
        ConstructionDisplay::PointMarker(p) => out.points(
            &[p],
            PointStyle {
                size_px: 9.0,
                color: color(1.0),
                depth: through,
            },
            handle,
        ),
    }
}

impl DocLayer for ConstructionLayer {
    fn update(&mut self, cx: &mut AppCx<'_>, out: &mut LayerOut<'_>) {
        self.shown.clear();
        self.preview.clear();
        let env = cx.env();
        let (design, eval) = (env.design, env.evaluated);
        // Both only when there is something to draw.
        let (mut hidden, mut bounds) = (None, None);
        let scene = out.scene;
        for (id, geom) in eval.constructions() {
            let hidden = hidden.get_or_insert_with(|| visibility::hidden(design));
            if hidden.contains(&visibility::construction_key(id)) {
                continue;
            }
            let Some((_, node)) = design.timeline().node(id.0) else {
                continue;
            };
            let handle = out
                .scene
                .pick_map()
                .render(&PickTarget::Construction(id))
                .map(|(key, _)| key.pick_id());
            let bounds = bounds.get_or_insert_with(|| scene.bounds());
            for g in instances(design, eval, node.context, geom) {
                self.shown.push(Shown {
                    id,
                    shape: display_extent(&g, bounds),
                    handle,
                });
            }
        }
        if let Some(preview) = out.preview {
            for (id, geom) in preview.constructions() {
                if eval.construction(id) == Some(geom) {
                    continue;
                }
                // A new feature is not in the document yet: it goes to the active component.
                let context = design
                    .timeline()
                    .node(id.0)
                    .map_or(env.active_component, |(_, n)| n.context);
                let bounds = bounds.get_or_insert_with(|| scene.bounds());
                for g in instances(design, eval, context, geom) {
                    self.preview.push(Shown {
                        id,
                        shape: display_extent(&g, bounds),
                        handle: None,
                    });
                }
            }
        }
        for s in &self.shown {
            draw(out.overlays, s, COLOR);
            out.construction_picks.push(ConstructionPick {
                id: s.id,
                shape: s.shape,
            });
        }
        for s in &self.preview {
            draw(out.overlays, s, PREVIEW_COLOR);
        }
    }
}

// ---- browser ------------------------------------------------------------------------------------

/// The feature of a framework construction row key (`construction/<feature>[@path]`).
fn row_feature(node: &NodeKey) -> Option<FeatureId> {
    let rest = node.as_str().strip_prefix("construction/")?;
    rest.split('@').next()?.parse().ok().map(FeatureId)
}

/// Adds "Delete" to the row menu of construction geometry. Builds no nodes.
struct RowMenu;

impl BrowserProvider for RowMenu {
    fn id(&self) -> &'static str {
        "construct"
    }

    fn affected_by(&self, _ev: &DocEvent) -> bool {
        false
    }

    fn build(&self, _env: &CmdEnv<'_>, _anchor: &BrowserAnchor, _out: &mut Vec<BrowserNode>) {}

    fn row_menu(&self, node: &NodeKey, ui: &mut egui::Ui, cx: &mut AppCx<'_>) {
        let Some(id) = row_feature(node) else {
            return;
        };
        if ui.button("Delete").clicked() {
            cx.submit(CommitBatch {
                label: "Delete".into(),
                commands: vec![Command::DeleteFeatures { ids: vec![id] }],
            });
            ui.close();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn there_is_one_command_per_variant_in_the_same_order() {
        assert_eq!(COMMANDS.len(), schema::VARIANTS.len());
        for (c, v) in COMMANDS.iter().zip(&schema::VARIANTS) {
            assert_eq!((c.kind, c.variant), (v.kind, v.id));
        }
        let mut ids: Vec<&str> = COMMANDS.iter().map(|c| c.id.0).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), 20);
    }

    #[test]
    fn construction_row_keys_give_the_feature() {
        let key = |s: &str| row_feature(&NodeKey::owned(s.to_owned()));
        assert_eq!(key("construction/12"), Some(FeatureId(12)));
        assert_eq!(key("construction/12@3.4"), Some(FeatureId(12)));
        assert_eq!(key("constructions/1"), None);
        assert_eq!(key("origin/xy"), None);
    }
}
