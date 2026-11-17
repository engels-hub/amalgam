//! The default tool (spec §8): click rules, box selection and select-through.
//!
//! - **Click** replaces the selection with what the hit selects ([`super::item_for_hit`]
//!   under the filter bar and the select priority). Ctrl / Shift / Cmd + click toggles the
//!   item. A click on empty space clears the selection (with a modifier it does nothing).
//! - **Box:** a primary drag draws a rectangle. Left to right is a window (fully inside),
//!   right to left a crossing (touching). The rectangle goes to the pick service's region pick
//!   (S10's window / crossing rectangles in the app). Without a modifier the result replaces
//!   the selection; with Ctrl / Shift / Cmd it is added.
//! - **Select-through:** holding the primary button for [`LONG_PRESS`] with less than
//!   [`LONG_PRESS_SLOP_PT`] of movement picks everything along the ray
//!   ([`PickMode::AllAlongRay`]) and lists it front to back next to the cursor; choosing an entry
//!   selects it. The release that ends the long press does not click.

use std::borrow::Cow;
use std::time::{Duration, Instant};

use amg_doc::{Design, TopoKind};

use crate::app::CommitBatch;
use crate::command::CmdId;
use crate::input::ViewportEvent;
use crate::pick::{PickMode, PickTarget, PickTicket, RegionMode};
use crate::select::{EntityFilter, SelItem, Selection};
use crate::tool::{FinishReason, InputResult, Tool, ToolCx};

/// Id of the select tool.
pub const SELECT_TOOL: CmdId = CmdId("app.select");
/// How long a press must be held for select-through.
pub const LONG_PRESS: Duration = Duration::from_millis(500);
/// A long press moves less than this, points.
pub const LONG_PRESS_SLOP_PT: f32 = 4.0;
/// A hover answer this close to a click is used for the click without a new pick, points.
const CLICK_REUSES_HOVER_PT: f32 = 1.0;

/// One entry of the select-through list.
#[derive(Clone, Debug, PartialEq)]
pub struct SelectThroughEntry {
    /// What choosing it selects.
    pub item: SelItem,
    /// Distance along the pick ray, mm.
    pub depth: f32,
    /// Its label in the list.
    pub label: String,
}

/// The open select-through list: everything under the long-pressed point, front to back.
#[derive(Clone, Debug, PartialEq)]
pub struct SelectThrough {
    /// Where the press was, points.
    pub at: egui::Pos2,
    /// The entries, nearest first.
    pub entries: Vec<SelectThroughEntry>,
}

#[derive(Copy, Clone, Debug)]
struct Press {
    pos: egui::Pos2,
    at: Instant,
    moved: bool,
}

#[derive(Copy, Clone, Debug)]
struct BoxDrag {
    from: egui::Pos2,
    to: egui::Pos2,
    mods: egui::Modifiers,
}

/// The default tool: click / Ctrl / Shift selection, window and crossing boxes and
/// select-through (see the module docs).
#[derive(Debug, Default)]
pub struct SelectTool {
    click: Option<(PickTicket, egui::Modifiers)>,
    region: Option<(PickTicket, egui::Modifiers)>,
    through_pick: Option<(PickTicket, egui::Pos2)>,
    press: Option<Press>,
    swallow_click: bool,
    boxing: Option<BoxDrag>,
    through: Option<SelectThrough>,
}

fn toggles(mods: egui::Modifiers) -> bool {
    mods.ctrl || mods.shift || mods.command
}

/// A short label for a selected item (select-through list).
pub(crate) fn describe(item: &SelItem, design: &Design) -> String {
    match item.target {
        PickTarget::Body { body, sub, .. } => {
            let name = design.body_props(body).name.unwrap_or_else(|| {
                if body.slot == 0 {
                    format!("Body {}", body.origin.0)
                } else {
                    format!("Body {}.{}", body.origin.0, body.slot)
                }
            });
            match sub {
                None => name,
                Some((kind, i)) => {
                    let k = match kind {
                        TopoKind::Face => "face",
                        TopoKind::Edge => "edge",
                        TopoKind::Vertex => "vertex",
                        TopoKind::FaceGroup => "face group",
                    };
                    format!("{name} · {k} {i}")
                }
            }
        }
        PickTarget::Sketch(s) => format!("Sketch {}", s.0.0),
        PickTarget::Construction(c) => format!("Construction {}", c.0.0),
        PickTarget::Occurrence(o) => format!("Component {}", o.0),
    }
}

impl SelectTool {
    fn apply(item: Option<SelItem>, mods: egui::Modifiers, sel: &mut Selection) {
        match (item, toggles(mods)) {
            (Some(i), true) => sel.toggle(i),
            (Some(i), false) => sel.set(vec![i]),
            (None, true) => {}
            (None, false) => sel.clear(),
        }
    }

    /// Picks waiting for an answer (idle detection).
    pub(crate) fn picks_pending(&self) -> bool {
        self.click.is_some() || self.region.is_some() || self.through_pick.is_some()
    }

    /// The box being dragged and its mode (drawn by the router).
    pub fn box_rect(&self) -> Option<(egui::Rect, RegionMode)> {
        self.boxing.map(|b| {
            (
                egui::Rect::from_two_pos(b.from, b.to),
                region_mode(b.from, b.to),
            )
        })
    }

    /// The open select-through list.
    pub fn select_through(&self) -> Option<&SelectThrough> {
        self.through.as_ref()
    }

    /// Closes the select-through list; true if it was open.
    pub(crate) fn close_through(&mut self) -> bool {
        self.through.take().is_some()
    }

    /// Cancels everything in progress (another tool starts).
    pub(crate) fn reset(&mut self) {
        *self = SelectTool::default();
    }
}

/// Left to right: window; right to left: crossing.
fn region_mode(from: egui::Pos2, to: egui::Pos2) -> RegionMode {
    if to.x >= from.x {
        RegionMode::Window
    } else {
        RegionMode::Crossing
    }
}

impl Tool for SelectTool {
    fn id(&self) -> CmdId {
        SELECT_TOOL
    }

    fn label(&self) -> Cow<'static, str> {
        Cow::Borrowed("Select")
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        let primary = egui::PointerButton::Primary;
        match ev {
            ViewportEvent::Press { button, pos, .. } if *button == primary => {
                self.through = None;
                self.swallow_click = false;
                self.press = Some(Press {
                    pos: *pos,
                    at: Instant::now(),
                    moved: false,
                });
                InputResult::Ignored
            }
            ViewportEvent::Hover { pos, .. } => {
                if let Some(p) = &mut self.press
                    && p.pos.distance(*pos) >= LONG_PRESS_SLOP_PT
                {
                    p.moved = true;
                }
                InputResult::Ignored
            }
            ViewportEvent::DragStart { button, from, mods } if *button == primary => {
                self.press = None;
                self.boxing = Some(BoxDrag {
                    from: *from,
                    to: *from,
                    mods: *mods,
                });
                cx.set_cursor(egui::CursorIcon::Crosshair);
                InputResult::Consumed
            }
            ViewportEvent::Drag { button, pos, .. } if *button == primary => match &mut self.boxing
            {
                Some(b) => {
                    b.to = *pos;
                    cx.app().request_repaint();
                    InputResult::Consumed
                }
                None => InputResult::Ignored,
            },
            ViewportEvent::DragEnd { button, pos, .. } if *button == primary => {
                let Some(b) = self.boxing.take() else {
                    return InputResult::Ignored;
                };
                let rect = egui::Rect::from_two_pos(b.from, *pos);
                if rect.width() >= 1.0 || rect.height() >= 1.0 {
                    let t = cx.pick_region(rect, region_mode(b.from, *pos));
                    self.region = Some((t, b.mods));
                }
                InputResult::Consumed
            }
            ViewportEvent::Release { button, .. } if *button == primary => {
                self.press = None;
                InputResult::Ignored
            }
            ViewportEvent::Click {
                button, pos, mods, ..
            } if *button == primary => {
                if std::mem::take(&mut self.swallow_click) {
                    return InputResult::Consumed;
                }
                if let Some(h) = cx.hover()
                    && h.at.distance(*pos) <= CLICK_REUSES_HOVER_PT
                {
                    let item = h.item.clone();
                    Self::apply(Some(item), *mods, cx.selection_mut());
                } else {
                    let t = cx.pick_point(*pos, PickMode::Nearest);
                    self.click = Some((t, *mods));
                }
                InputResult::Consumed
            }
            _ => InputResult::Ignored,
        }
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        if let Some((t, mods)) = self.click
            && let Some(hits) = cx.poll_pick(t)
        {
            self.click = None;
            let item = hits.into_iter().find_map(|h| cx.item_for(h.target));
            Self::apply(item, mods, cx.selection_mut());
        }
        if let Some((t, mods)) = self.region
            && let Some(hits) = cx.poll_pick(t)
        {
            self.region = None;
            let mut items: Vec<SelItem> = Vec::new();
            for item in hits.into_iter().filter_map(|h| cx.item_for(h.target)) {
                if !items.iter().any(|i| i.target == item.target) {
                    items.push(item);
                }
            }
            if toggles(mods) {
                cx.selection_mut().extend(items);
            } else {
                cx.selection_mut().set(items);
            }
        }
        if let Some(p) = self.press
            && !p.moved
            && self.through_pick.is_none()
        {
            let held = p.at.elapsed();
            if held >= LONG_PRESS {
                self.press = None;
                self.swallow_click = true;
                let t = cx.pick_point(p.pos, PickMode::AllAlongRay);
                self.through_pick = Some((t, p.pos));
            } else {
                cx.app().request_repaint_after(LONG_PRESS - held);
            }
        }
        if let Some((t, at)) = self.through_pick
            && let Some(mut hits) = cx.poll_pick(t)
        {
            self.through_pick = None;
            hits.sort_by(|a, b| a.depth.total_cmp(&b.depth));
            let mut entries: Vec<SelectThroughEntry> = Vec::new();
            for h in hits {
                let Some(item) = cx.item_for(h.target) else {
                    continue;
                };
                if entries.iter().any(|e| e.item.target == item.target) {
                    continue;
                }
                let label = describe(&item, cx.env().design);
                entries.push(SelectThroughEntry {
                    item,
                    depth: h.depth,
                    label,
                });
            }
            self.through = (!entries.is_empty()).then_some(SelectThrough { at, entries });
            cx.app().request_repaint();
        }
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::ALL
    }

    fn finish(&mut self, _reason: FinishReason, _cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        self.reset();
        None
    }
}
