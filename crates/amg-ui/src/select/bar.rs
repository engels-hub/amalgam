//! The selection-filter bar (spec §8 "Filters"): one toggle per [`FILTER_GROUPS`] entry and
//! the select priority, at the top of the viewport.
//!
//! It shows while the pointer is over the viewport, and always while the filter or the
//! priority differs from the default (so a restricted filter is never invisible). Changes go
//! straight to the active document's [`Selection`](super::Selection); the router intersects
//! them with the tool's filter from the next pick on.

use crate::app::AmalgamApp;
use crate::select::{EntityFilter, FILTER_GROUPS, SelectPriority};

/// Kinds the bar never switches off (manipulator handles).
const ALWAYS: EntityFilter = EntityFilter::HANDLE;

/// Draws the bar over the active document's viewport.
pub(crate) fn show(app: &mut AmalgamApp, ctx: &egui::Context) {
    let active = app.active;
    let Some(rect) = app.docs[active].view.as_ref().map(|v| v.rect) else {
        return;
    };
    let sel = &app.docs[active].selection;
    let custom = sel.filter() != EntityFilter::ALL || sel.priority() != SelectPriority::Default;
    let pointer_in = ctx
        .input(|i| i.pointer.latest_pos())
        .is_some_and(|p| rect.contains(p));
    if !custom && !pointer_in {
        return;
    }
    let sel = &mut app.docs[active].selection;
    let mut filter = sel.filter();
    let mut priority = sel.priority();
    egui::Area::new(egui::Id::new("amg_selection_filter_bar"))
        .order(egui::Order::Middle)
        .pivot(egui::Align2::CENTER_TOP)
        .fixed_pos(rect.center_top() + egui::vec2(0.0, 6.0))
        .constrain_to(rect)
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Select");
                    for (label, bits) in FILTER_GROUPS {
                        let on = filter.intersects(*bits);
                        let r = ui
                            .selectable_label(on, *label)
                            .on_hover_text(format!("Pick {}", label.to_lowercase()));
                        if r.clicked() {
                            filter = if on {
                                filter.without(*bits)
                            } else {
                                filter | *bits
                            };
                        }
                    }
                    if ui
                        .small_button("Reset filter")
                        .on_hover_text("Select every kind, automatic priority")
                        .clicked()
                    {
                        filter = EntityFilter::ALL;
                        priority = SelectPriority::Default;
                    }
                    ui.separator();
                    ui.label("Priority");
                    for p in SelectPriority::ALL {
                        if ui.selectable_label(priority == p, p.label()).clicked() {
                            priority = p;
                        }
                    }
                });
            });
        });
    sel.set_filter(filter | ALWAYS);
    sel.set_priority(priority);
}
