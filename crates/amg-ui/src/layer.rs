//! Per-document view layers (additive, INS-09; S11 Interface changes 2026-11-23).
//!
//! A [`DocLayer`] is module state that lives with one open document and contributes to its
//! view every frame, independent of the active tool and of open panels: persistent section
//! analyses (S05 INS-09) clip the view whether or not their dialog or stats panel is open.
//!
//! - Modules register a [`DocLayerDef`] with
//!   [`Registrar::doc_layer`](crate::module::Registrar::doc_layer). Each document creates its
//!   layers the first time it is the active document.
//! - Once per frame, for the active document, after the active tool's `update` and the panel
//!   overlays, the router calls [`DocLayer::update`] in registration order with an [`AppCx`] on
//!   the document and a [`LayerOut`]: layers add RND-07 overlays and the view's section planes
//!   ([`ViewSections`]), which the viewport hands to RND-06 and the scene adapter applies per body
//!   (`Instance::sectioned`, cap colours).
//! - Tools and panels reach a layer of the active document through
//!   [`AppCx::doc_layer`](crate::app::AppCx::doc_layer) (downcast by type).

use std::any::Any;

use crate::app::AppCx;
use crate::scene::SectionStyle;

/// What a layer contributes to the frame.
#[derive(Debug)]
pub struct LayerOut<'a> {
    /// The frame's overlay list (the active tool's and the panels' overlays are already in it).
    pub overlays: &'a mut amg_render::OverlayList,
    /// The view's section planes and per-body section style.
    pub sections: &'a mut ViewSections,
    /// Per-body view overrides of the committed bodies (additive, AM-11): `None` (the
    /// default each frame) shows the design placements; the last layer that sets it wins.
    pub body_views: &'a mut Option<std::sync::Arc<crate::scene::BodyViews>>,
}

/// The section planes of the view (RND-06) and which bodies they clip.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ViewSections {
    /// The planes, in order; RND-06 applies the first `MAX_SECTIONS` enabled ones.
    pub planes: Vec<amg_render::SectionPlane>,
    /// Which bodies are clipped and their cap colours.
    pub style: SectionStyle,
}

/// Module state attached to one document (see the module docs).
pub trait DocLayer: Any {
    /// Once per frame while the document is active.
    fn update(&mut self, cx: &mut AppCx<'_>, out: &mut LayerOut<'_>);
}

/// A layer registration.
#[derive(Clone, Copy, Debug)]
pub struct DocLayerDef {
    /// Unique id (`<module>.<name>`).
    pub id: &'static str,
    /// Creates the layer state for a document.
    pub create: fn() -> Box<dyn DocLayer>,
}

/// The layers of one document, in registration order. A slot is `None` while its layer runs.
#[derive(Default)]
pub(crate) struct DocLayers {
    pub(crate) list: Vec<(&'static str, Option<Box<dyn DocLayer>>)>,
    pub(crate) created: bool,
}

impl std::fmt::Debug for DocLayers {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_list()
            .entries(self.list.iter().map(|(id, _)| id))
            .finish()
    }
}

impl DocLayers {
    /// Creates the registered layers once.
    pub(crate) fn ensure(&mut self, defs: &[DocLayerDef]) {
        if self.created {
            return;
        }
        self.created = true;
        self.list = defs.iter().map(|d| (d.id, Some((d.create)()))).collect();
    }

    /// The layer of type `T`, if it exists and is not running.
    pub(crate) fn get_mut<T: DocLayer>(&mut self) -> Option<&mut T> {
        self.list.iter_mut().find_map(|(_, l)| {
            let l: &mut dyn Any = l.as_deref_mut()?;
            l.downcast_mut::<T>()
        })
    }

    /// The layer of type `T`, if it exists and is not running.
    pub(crate) fn get<T: DocLayer>(&self) -> Option<&T> {
        self.list.iter().find_map(|(_, l)| {
            let l: &dyn Any = l.as_deref()?;
            l.downcast_ref::<T>()
        })
    }
}
