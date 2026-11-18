//! [`SelectionField`] (spec §9): "N selected", a clear button, the active state and the pick
//! filter of the active field.
//!
//! Only one field is active at a time, app-wide. The active field is recorded in egui memory
//! together with its filter and the pass it was drawn in; [`active_selection_filter`] reads it
//! back, and the tool router intersects the active tool's pick filter with it (so hover,
//! clicks and boxes only pick what the field accepts). A field becomes active when it is
//! clicked or when its `active` flag is set to true by the caller (auto-focus); every other
//! field then turns its flag off on its next draw.
//!
//! With [`SelectionField::mirror`] the field drives UI-04's [`Selection`]: on activation the
//! selection is set to the field's items, and while it is active the field takes the selected
//! items its filter admits (up to its maximum). The selection itself is edited by UI-04's
//! select rules (a tool forwards its events to a [`SelectTool`](crate::select::SelectTool)).

use crate::select::{EntityFilter, SelItem, Selection};

/// Where the active field is recorded.
fn active_key() -> egui::Id {
    egui::Id::new("amg_ui.widgets.active_selection_field")
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct ActiveField {
    id: egui::Id,
    filter: EntityFilter,
    pass: u64,
}

/// The pick filter of the [`SelectionField`] that is active and was drawn in this pass (or
/// the previous one), if any. The tool router intersects the active tool's filter with it.
pub fn active_selection_filter(ctx: &egui::Context) -> Option<EntityFilter> {
    let rec: ActiveField = ctx.data(|d| d.get_temp(active_key()))?;
    (ctx.cumulative_pass_nr() <= rec.pass + 1).then_some(rec.filter)
}

/// True if `n` items satisfy the limits (`min ..= max`).
pub fn selection_count_valid(n: usize, min: u32, max: Option<u32>) -> bool {
    let n = u64::try_from(n).unwrap_or(u64::MAX);
    n >= u64::from(min) && max.is_none_or(|m| n <= u64::from(m))
}

/// A selection input of a dialog.
#[derive(Debug)]
#[must_use = "call `show`"]
pub struct SelectionField<'a> {
    label: &'a str,
    items: &'a mut Vec<SelItem>,
    filter: EntityFilter,
    active: &'a mut bool,
    min: u32,
    max: Option<u32>,
    selection: Option<&'a mut Selection>,
}

impl<'a> SelectionField<'a> {
    /// A field labelled `label` holding `items`, accepting `filter`; `active` is its state
    /// (set it to true to activate the field programmatically).
    pub fn new(
        label: &'a str,
        items: &'a mut Vec<SelItem>,
        filter: EntityFilter,
        active: &'a mut bool,
    ) -> Self {
        SelectionField {
            label,
            items,
            filter,
            active,
            min: 1,
            max: None,
            selection: None,
        }
    }

    /// How many items the field needs (default `1..`).
    pub fn limits(mut self, min: u32, max: Option<u32>) -> Self {
        self.min = min;
        self.max = max;
        self
    }

    /// Mirror the document selection while active (additive; see the module docs).
    pub fn mirror(mut self, selection: &'a mut Selection) -> Self {
        self.selection = Some(selection);
        self
    }

    /// Draws the field. The response is the field button's (clicked = activated).
    pub fn show(self, ui: &mut egui::Ui) -> egui::Response {
        let SelectionField {
            label,
            items,
            filter,
            active,
            min,
            max,
            selection,
        } = self;
        let id = ui.make_persistent_id(("amg_selection_field", label));
        let was_id = id.with("was_active");
        let ctx = ui.ctx().clone();
        let pass = ctx.cumulative_pass_nr();
        // State from the previous draw only counts if that draw was in the last pass (a
        // reopened dialog starts fresh).
        let was_active = ctx
            .data(|d| d.get_temp::<(bool, u64)>(was_id))
            .is_some_and(|(a, p)| a && p + 1 >= pass);
        let rec: Option<ActiveField> = ctx
            .data(|d| d.get_temp::<ActiveField>(active_key()))
            .filter(|r| r.pass + 1 >= pass);
        let claim = |ctx: &egui::Context| {
            ctx.data_mut(|d| {
                d.insert_temp(active_key(), ActiveField { id, filter, pass });
            });
        };
        let mut activated = false;
        if *active && !was_active {
            // Activated by the caller (auto-focus).
            claim(&ctx);
            activated = true;
        } else if *active && rec.is_some_and(|r| r.id != id) {
            // Another field took over.
            *active = false;
        }

        let response = ui
            .horizontal(|ui| {
                // Clicking the label activates the field too.
                let label_resp = ui.add(
                    egui::Label::new(label)
                        .selectable(false)
                        .sense(egui::Sense::click()),
                );
                let text = format!("{} selected", items.len());
                let mut r = ui
                    .add(egui::Button::new(text).selected(*active))
                    .labelled_by(label_resp.id);
                if (r.clicked() || label_resp.clicked()) && !*active {
                    *active = true;
                    claim(ui.ctx());
                    activated = true;
                    r.mark_changed();
                }
                let clear_label = format!("Clear {label}");
                let clear = ui
                    .add_enabled(!items.is_empty(), egui::Button::new("✕").small())
                    .on_hover_text(clear_label.as_str());
                clear.widget_info(|| {
                    egui::WidgetInfo::labeled(
                        egui::WidgetType::Button,
                        !items.is_empty(),
                        clear_label.as_str(),
                    )
                });
                let cleared = clear.clicked();
                if !selection_count_valid(items.len(), min, max) && !items.is_empty() {
                    ui.colored_label(ui.visuals().warn_fg_color, limits_text(min, max));
                }
                (r, cleared)
            })
            .inner;
        let (mut r, cleared) = response;

        match selection {
            Some(sel) if *active => {
                if activated {
                    sel.set(items.clone());
                } else {
                    let mut picked: Vec<SelItem> = sel
                        .items()
                        .iter()
                        .filter(|i| filter.admits(&i.target))
                        .cloned()
                        .collect();
                    if let Some(m) = max.and_then(|m| usize::try_from(m).ok()) {
                        // Keep the newest picks.
                        let drop = picked.len().saturating_sub(m);
                        picked.drain(..drop);
                    }
                    if picked != *items {
                        *items = picked;
                        r.mark_changed();
                    }
                }
                if cleared {
                    sel.clear();
                }
            }
            _ => {}
        }
        if cleared && !items.is_empty() {
            items.clear();
            r.mark_changed();
        }
        if *active {
            // Refresh the record: the filter may change and the pass shows the field is drawn.
            claim(&ctx);
        }
        ctx.data_mut(|d| d.insert_temp(was_id, (*active, pass)));
        r
    }
}

fn limits_text(min: u32, max: Option<u32>) -> String {
    match max {
        Some(m) if m == min => format!("needs {m}"),
        Some(m) => format!("needs {min} to {m}"),
        None => format!("needs at least {min}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn limits_check_counts() {
        assert!(!selection_count_valid(0, 1, None));
        assert!(selection_count_valid(3, 1, None));
        assert!(selection_count_valid(1, 1, Some(1)));
        assert!(!selection_count_valid(2, 1, Some(1)));
        assert!(selection_count_valid(0, 0, Some(2)));
    }
}
