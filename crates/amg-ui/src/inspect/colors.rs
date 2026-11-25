//! "Display Component Colours" (S05 §4, INS-10): every component's bodies are drawn in their
//! own colour while the document's [`COMPONENT_COLORS_FLAG`] view flag is on, and in their
//! materials again when it is off. It is view state: nothing is saved or put on the undo stack.
//!
//! The colours reach the view through a [`DocLayer`] and [`LayerOut::body_tints`]; the scene
//! adapter replaces the base colour of the instances (opacity kept, no upload).

use std::sync::Arc;

use amg_doc::{ComponentId, Design, Evaluated};

use crate::app::{AppCx, CmdEnv};
use crate::command::{CmdAction, CmdId, CommandDef, Enablement, Enabler, Placement};
use crate::layer::{DocLayer, DocLayerDef, LayerOut};
use crate::module::{GroupId, Registrar};
use crate::scene::BodyTints;

/// Toggles the component colours.
pub const COMPONENT_COLORS_CMD: CmdId = CmdId("inspect.component_colors");
/// The view flag that is on while component colours are shown.
pub const COMPONENT_COLORS_FLAG: &str = "inspect.component_colors";

/// The colour (linear RGB) of the component at `index` in id order: hues a golden-ratio step
/// apart (so neighbours differ strongly and none repeats), with alternating brightness.
pub fn component_color(index: usize) -> [f32; 3] {
    const GOLDEN: f64 = 0.618_033_988_749_894_9;
    #[allow(clippy::cast_precision_loss)]
    let hue = (0.58 + index as f64 * GOLDEN).fract() * 6.0;
    let value = if index.is_multiple_of(2) { 0.92 } else { 0.72 };
    let saturation = 0.68;
    let chroma = value * saturation;
    let x = chroma * (1.0 - (hue % 2.0 - 1.0).abs());
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let rgb = match hue as u8 {
        0 => [chroma, x, 0.0],
        1 => [x, chroma, 0.0],
        2 => [0.0, chroma, x],
        3 => [0.0, x, chroma],
        4 => [x, 0.0, chroma],
        _ => [chroma, 0.0, x],
    };
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let [r, g, b] = rgb.map(|c| ((c + value - chroma) * 255.0).round() as u8);
    let linear = egui::Rgba::from(egui::Color32::from_rgb(r, g, b));
    [linear.r(), linear.g(), linear.b()]
}

/// The colour of `component` in `design` (`None` if it is not a component of it).
pub fn color_of(design: &Design, component: ComponentId) -> Option<[f32; 3]> {
    design
        .components()
        .position(|(id, _)| id == component)
        .map(component_color)
}

/// The tint of every evaluated body: the colour of its component.
pub fn component_tints(design: &Design, evaluated: &Evaluated) -> BodyTints {
    evaluated
        .bodies()
        .filter_map(|(id, b)| Some((id, color_of(design, b.component)?)))
        .collect()
}

/// The per-document layer; it rebuilds the tints only when the evaluated state or the
/// component list changed.
#[derive(Default)]
struct ComponentColorLayer {
    cache: Option<(Arc<Evaluated>, usize, Arc<BodyTints>)>,
}

impl DocLayer for ComponentColorLayer {
    fn update(&mut self, cx: &mut AppCx<'_>, out: &mut LayerOut<'_>) {
        let env = cx.env();
        if !is_on(&env) {
            self.cache = None;
            return;
        }
        let components = env.design.components().count();
        let fresh = self
            .cache
            .as_ref()
            .is_some_and(|(e, n, _)| Arc::ptr_eq(e, env.evaluated) && *n == components);
        if !fresh {
            self.cache = Some((
                Arc::clone(env.evaluated),
                components,
                Arc::new(component_tints(env.design, env.evaluated)),
            ));
        }
        *out.body_tints = self.cache.as_ref().map(|(_, _, t)| Arc::clone(t));
    }
}

fn create_layer() -> Box<dyn DocLayer> {
    Box::new(ComponentColorLayer::default())
}

fn is_on(env: &CmdEnv<'_>) -> bool {
    env.view_flags.contains(COMPONENT_COLORS_FLAG)
}

fn set_on(cx: &mut AppCx<'_>, on: bool) {
    cx.set_view_flag(COMPONENT_COLORS_FLAG, on);
}

fn enabled(env: &CmdEnv<'_>) -> Enablement {
    if is_on(env) || env.evaluated.bodies().next().is_some() {
        Enablement::Enabled
    } else {
        Enablement::disabled("The design has no body")
    }
}

/// Registers the command and the layer.
pub(crate) fn register(r: &mut Registrar<'_>) {
    r.command(
        CommandDef::new(
            COMPONENT_COLORS_CMD.0,
            "Display Component Colours",
            CmdAction::Toggle {
                get: is_on,
                set: set_on,
            },
        )
        .with_tooltip("Draw the bodies of each component in their own colour")
        .with_keywords(&[
            "inspect",
            "component",
            "colour",
            "color",
            "display component colors",
            "assembly",
        ])
        .with_icon(crate::icons::ids::COMPONENT)
        .with_shortcut("Shift+N")
        .with_enabler(Enabler::When(enabled))
        .placed(Placement::Toolbar {
            group: GroupId("design.inspect.inspect"),
            pinned: false,
            order: 20,
        })
        .placed(Placement::Toolbar {
            group: GroupId("design.solid.inspect"),
            pinned: false,
            order: 20,
        }),
    );
    r.doc_layer(DocLayerDef {
        id: "inspect.component_colors",
        create: create_layer,
    });
}
