//! SVG icon registry and raster cache (UI-23).
//!
//! * **Set**: [Tabler Icons](https://tabler.io/icons) (MIT, `assets/icons/LICENSE-tabler.txt`),
//!   outline style, only the SVGs the shell uses, vendored (no build-time download). Each is a
//!   24x24 stroke icon on `currentColor`; the cache swaps that for white and the draw sites
//!   tint with egui's text colour, so dark and light themes both work.
//! * **Registry**: [`Registrar::icon`](crate::module::Registrar::icon) maps an [`IconId`] to
//!   embedded SVG bytes (`include_bytes!`); the framework registers the vendored set through
//!   [`register_builtin`], domain modules may add their own.
//! * **Cache**: [`IconCache`] lives in the egui context (installed by [`install`]) so every draw
//!   site that has a `Ui` can reach it. Icons are rasterised per pixel size (points x
//!   pixels-per-point) into textures; a scale change drops the old textures and re-rasterises
//!   lazily, nothing else does ([`IconCache::raster_count`] counts rasterisations).
//! * **Fallback**: an unknown or [`IconId::NONE`] icon draws nothing and the caller shows its
//!   text label (the label stays the accessible name everywhere).

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use egui::{Color32, ColorImage, Context, Id, Rect, SizeHint, TextureHandle, TextureOptions};

use crate::module::{IconId, Registrar};
use crate::registry::UiRegistry;

/// Default icon size in points (toolbar, menus, browser rows).
pub const SIZE_PT: f32 = 16.0;

macro_rules! icon_set {
    ($( $(#[$m:meta])* $name:ident = $file:literal ),* $(,)?) => {
        /// Ids of the vendored icons (the Tabler file stem).
        pub mod ids {
            use crate::module::IconId;
            $( $(#[$m])* pub const $name: IconId = IconId($file); )*
        }

        /// Every vendored icon: id and SVG source.
        pub const BUILTIN: &[(IconId, &[u8])] = &[
            $( (ids::$name, include_bytes!(concat!("../assets/icons/", $file, ".svg")) as &[u8]), )*
        ];
    };
}

icon_set! {
    /// Command search.
    SEARCH = "search",
    /// Undo.
    UNDO = "arrow-back-up",
    /// Redo.
    REDO = "arrow-forward-up",
    /// Save selection set / bookmark.
    BOOKMARK = "bookmark",
    /// Selection sets folder.
    BOOKMARKS = "bookmarks",
    /// Clear selection.
    CLEAR_SELECTION = "square-off",
    /// New document.
    FILE_NEW = "file-plus",
    /// Open.
    FOLDER_OPEN = "folder-open",
    /// Save.
    SAVE = "device-floppy",
    /// Save as.
    SAVE_AS = "file-download",
    /// Save compact.
    SAVE_COMPACT = "file-zip",
    /// Close.
    CLOSE = "x",
    /// Measure.
    MEASURE = "ruler-measure",
    /// Physical properties.
    PROPERTIES = "list-details",
    /// Section analysis.
    SECTION = "section",
    /// Section statistics.
    SECTION_STATS = "chart-area",
    /// Insert mesh / import.
    IMPORT = "file-import",
    /// Export.
    EXPORT = "file-export",
    /// Repair.
    REPAIR = "tool",
    /// Remesh.
    REMESH = "grid-pattern",
    /// Reduce.
    REDUCE = "arrows-minimize",
    /// Smooth.
    SMOOTH = "wave-sine",
    /// Plane cut.
    CUT = "cut",
    /// Shell.
    SHELL = "box-model-2",
    /// Combine.
    COMBINE = "layers-union",
    /// Separate shells.
    SEPARATE = "arrows-split-2",
    /// Reverse normals.
    REVERSE_NORMAL = "arrows-exchange",
    /// Erase and fill.
    ERASE = "eraser",
    /// Wrap.
    WRAP = "cube-3d-sphere",
    /// Face groups.
    FACE_GROUPS = "category",
    /// Mesh analysis.
    ANALYSIS = "stethoscope",
    /// Add / new.
    PLUS = "plus",
    /// Add bodies.
    CUBE_PLUS = "cube-plus",
    /// Delete.
    DELETE = "trash",
    /// Place face down.
    FACE_DOWN = "arrow-bar-to-down",
    /// Drop to plate.
    DROP = "arrow-bar-down",
    /// Move / rotate.
    MOVE_ROTATE = "rotate-3d",
    /// Centre.
    CENTRE = "focus-centered",
    /// Overhang overlay.
    OVERHANG = "triangle-inverted",
    /// Settings (sliders).
    ADJUSTMENTS = "adjustments",
    /// Auto-orient.
    AUTO_ORIENT = "rotate-360",
    /// Arrange.
    ARRANGE = "layout-grid",
    /// Send to slicer.
    PRINTER = "printer",
    /// Export plate.
    EXPORT_PLATE = "package-export",
    /// Settings (cog).
    SETTINGS = "settings",
    /// Sketch.
    SKETCH = "pencil",
    /// Sketch line (UI-09).
    LINE = "line",
    /// Finish Sketch (UI-09).
    SKETCH_FINISH = "circle-check",
    /// Select tool.
    SELECT = "pointer",
    /// Folder node.
    FOLDER = "folder",
    /// Units / ruler node.
    RULER = "ruler",
    /// Named views.
    CAMERA = "camera",
    /// Origin.
    ORIGIN = "axis-x",
    /// Body.
    BODY = "box",
    /// Component.
    COMPONENT = "components",
    /// Analysis folder.
    ANALYSIS_FOLDER = "chart-dots",
    /// Generic feature / unknown kind.
    GENERIC = "diamond",
    /// Edit.
    EDIT = "edit",
}

/// Registers the vendored set (the framework's `app` module calls this).
pub fn register_builtin(r: &mut Registrar<'_>) {
    for &(id, svg) in BUILTIN {
        r.icon(id, svg);
    }
}

/// Pixels per point as a stable integer key (milli-points).
fn scale_key(ppp: f32) -> u32 {
    (ppp * 1000.0).round().max(1.0) as u32
}

/// Rasterises `svg` (a `currentColor` stroke icon) to white-on-transparent at `px` x `px`.
///
/// # Errors
/// The SVG does not parse.
pub fn rasterise(svg: &[u8], px: u32) -> Result<ColorImage, String> {
    let text = String::from_utf8_lossy(svg).replace("currentColor", "#ffffff");
    egui_extras::image::load_svg_bytes_with_size(
        text.as_bytes(),
        SizeHint::Size {
            width: px,
            height: px,
            maintain_aspect_ratio: true,
        },
        &Default::default(),
    )
}

#[derive(Default)]
struct Inner {
    svgs: HashMap<&'static str, &'static [u8]>,
    /// Pixels per point (milli) the textures were rasterised for.
    scale: u32,
    textures: HashMap<(&'static str, u32), Option<TextureHandle>>,
    rasters: usize,
}

impl std::fmt::Debug for IconCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let g = self.lock();
        f.debug_struct("IconCache")
            .field("icons", &g.svgs.len())
            .field("textures", &g.textures.len())
            .field("rasters", &g.rasters)
            .finish()
    }
}

/// The icon textures of one egui context (cheap to clone; shared).
#[derive(Clone, Default)]
pub struct IconCache(Arc<Mutex<Inner>>);

impl IconCache {
    /// A cache over the registry's icons.
    pub fn new(registry: &UiRegistry) -> IconCache {
        let c = IconCache::default();
        c.lock().svgs = registry.icons().collect();
        c
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// How many SVGs were rasterised so far (a scale change makes the next draws count again).
    pub fn raster_count(&self) -> usize {
        self.lock().rasters
    }

    /// The textures of `id` at the current scale (one per size drawn so far; empty if none).
    pub fn texture_ids(&self, id: IconId) -> Vec<egui::TextureId> {
        let g = self.lock();
        g.textures
            .iter()
            .filter(|((i, _), _)| *i == id.0)
            .filter_map(|(_, t)| t.as_ref().map(TextureHandle::id))
            .collect()
    }

    /// Whether the icon is registered.
    pub fn contains(&self, id: IconId) -> bool {
        !id.0.is_empty() && self.lock().svgs.contains_key(id.0)
    }

    /// The texture of `id` at `size_pt` for the context's pixels per point; rasterised on the
    /// first request per scale. `None` for [`IconId::NONE`], an unknown id or a broken SVG.
    pub fn texture(
        &self,
        ctx: &Context,
        id: IconId,
        size_pt: f32,
    ) -> Option<egui::load::SizedTexture> {
        if id.0.is_empty() {
            return None;
        }
        let ppp = ctx.pixels_per_point();
        let scale = scale_key(ppp);
        let px = (size_pt * ppp).round().max(1.0) as u32;
        let mut g = self.lock();
        if g.scale != scale {
            g.textures.clear();
            g.scale = scale;
        }
        let key = (id.0, px);
        if !g.textures.contains_key(&key) {
            let tex = g
                .svgs
                .get(id.0)
                .copied()
                .and_then(|svg| match rasterise(svg, px) {
                    Ok(img) => Some(ctx.load_texture(
                        format!("amg_icon_{}_{px}", id.0),
                        img,
                        TextureOptions::LINEAR,
                    )),
                    Err(e) => {
                        tracing::warn!(icon = id.0, error = %e, "icon SVG does not parse");
                        None
                    }
                });
            if g.svgs.contains_key(id.0) {
                g.rasters += 1;
            }
            g.textures.insert(key, tex);
        }
        g.textures
            .get(&key)
            .and_then(|t| t.as_ref())
            .map(|t| egui::load::SizedTexture::new(t.id(), egui::vec2(size_pt, size_pt)))
    }
}

fn cache_id() -> Id {
    Id::new("amg_icon_cache")
}

/// Installs the cache for `registry` into the context (idempotent; call every frame).
pub fn install(ctx: &Context, registry: &UiRegistry) {
    if ctx.data(|d| d.get_temp::<IconCache>(cache_id())).is_none() {
        ctx.data_mut(|d| d.insert_temp(cache_id(), IconCache::new(registry)));
    }
}

/// The context's cache, if [`install`] ran.
pub fn cache(ctx: &Context) -> Option<IconCache> {
    ctx.data(|d| d.get_temp::<IconCache>(cache_id()))
}

/// The icon as an image tinted with the UI's text colour (weak when the `ui` is disabled);
/// `None` when it cannot be drawn (the caller shows text instead).
pub fn image(
    ui: &egui::Ui,
    id: IconId,
    size_pt: f32,
    enabled: bool,
) -> Option<egui::Image<'static>> {
    let tex = cache(ui.ctx())?.texture(ui.ctx(), id, size_pt)?;
    let tint = if enabled && ui.is_enabled() {
        ui.visuals().text_color()
    } else {
        ui.visuals().weak_text_color()
    };
    Some(egui::Image::new(tex).tint(tint))
}

/// A button with the icon left of the label, or the plain label button without an icon.
/// The label stays the accessible name.
pub fn button(
    ui: &egui::Ui,
    id: IconId,
    label: impl Into<egui::WidgetText>,
    enabled: bool,
) -> egui::Button<'static> {
    match image(ui, id, SIZE_PT, enabled) {
        Some(img) => egui::Button::image_and_text(img, label),
        None => egui::Button::new(label),
    }
}

/// Paints the icon into `rect` (centred, square) with `color`; false if it cannot be drawn.
pub fn paint(
    ctx: &Context,
    painter: &egui::Painter,
    rect: Rect,
    id: IconId,
    color: Color32,
) -> bool {
    let Some(cache) = cache(ctx) else {
        return false;
    };
    let Some(tex) = cache.texture(ctx, id, rect.width().min(rect.height()).round()) else {
        return false;
    };
    painter.image(
        tex.id,
        Rect::from_center_size(rect.center(), tex.size),
        Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
        color,
    );
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_vendored_svg_parses_and_has_ink() {
        for &(id, svg) in BUILTIN {
            let img = rasterise(svg, 24).unwrap_or_else(|e| panic!("{}: {e}", id.0));
            assert!(img.pixels.iter().any(|p| p.a() > 0), "{} is blank", id.0);
        }
    }

    #[test]
    fn builtin_ids_are_unique() {
        let mut ids: Vec<_> = BUILTIN.iter().map(|(i, _)| i.0).collect();
        ids.sort_unstable();
        let n = ids.len();
        ids.dedup();
        assert_eq!(ids.len(), n);
    }
}
