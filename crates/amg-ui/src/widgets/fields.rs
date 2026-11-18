//! [`ChoiceField`], [`toggle`], [`IntField`] and [`flip`] (spec §9).

/// A choice between a few values: a dropdown, or a row of (icon) buttons.
#[derive(Debug)]
#[must_use = "call `show`"]
pub struct ChoiceField<'a, T> {
    label: &'a str,
    value: &'a mut T,
    options: &'a [(T, &'a str)],
    buttons: bool,
}

impl<'a, T: Clone + PartialEq> ChoiceField<'a, T> {
    /// A dropdown labelled `label` over `options` (value, text).
    pub fn new(label: &'a str, value: &'a mut T, options: &'a [(T, &'a str)]) -> Self {
        ChoiceField {
            label,
            value,
            options,
            buttons: false,
        }
    }

    /// Show the options as a row of buttons (icon text such as `⬆`) instead of a dropdown.
    pub fn buttons(mut self) -> Self {
        self.buttons = true;
        self
    }

    /// Draws the field; `changed()` when the value changed.
    pub fn show(self, ui: &mut egui::Ui) -> egui::Response {
        let ChoiceField {
            label,
            value,
            options,
            buttons,
        } = self;
        ui.horizontal(|ui| {
            let label_id = ui.label(label).id;
            if buttons {
                let mut changed = None::<egui::Response>;
                let mut row: Option<egui::Response> = None;
                for (v, text) in options {
                    let r = ui.selectable_label(*value == *v, *text);
                    if r.clicked() && *value != *v {
                        *value = v.clone();
                        let mut r = r.clone();
                        r.mark_changed();
                        changed = Some(r);
                    }
                    row = Some(match row {
                        Some(acc) => acc.union(r),
                        None => r,
                    });
                }
                let row = changed.or(row).unwrap_or_else(|| ui.label(""));
                row.labelled_by(label_id)
            } else {
                let current = options
                    .iter()
                    .find(|(v, _)| *v == *value)
                    .map_or("", |(_, t)| *t);
                let mut changed = false;
                let inner = egui::ComboBox::from_id_salt(("amg_choice", label))
                    .selected_text(current)
                    .show_ui(ui, |ui| {
                        for (v, text) in options {
                            if ui.selectable_label(*value == *v, *text).clicked() && *value != *v {
                                *value = v.clone();
                                changed = true;
                            }
                        }
                    });
                let mut r = inner.response.labelled_by(label_id);
                if changed {
                    r.mark_changed();
                }
                r
            }
        })
        .inner
    }
}

/// A labelled check box.
pub fn toggle(ui: &mut egui::Ui, label: &str, value: &mut bool) -> egui::Response {
    ui.checkbox(value, label)
}

/// An integer field with optional bounds (drag or type).
#[derive(Debug)]
#[must_use = "call `show`"]
pub struct IntField<'a> {
    label: &'a str,
    value: &'a mut i64,
    min: i64,
    max: i64,
}

impl<'a> IntField<'a> {
    /// An unbounded integer field labelled `label`.
    pub fn new(label: &'a str, value: &'a mut i64) -> Self {
        IntField {
            label,
            value,
            min: i64::MIN,
            max: i64::MAX,
        }
    }

    /// Clamp to `min ..= max`.
    pub fn range(mut self, min: i64, max: i64) -> Self {
        self.min = min;
        self.max = max.max(min);
        self
    }

    /// Draws the field.
    pub fn show(self, ui: &mut egui::Ui) -> egui::Response {
        let IntField {
            label,
            value,
            min,
            max,
        } = self;
        ui.horizontal(|ui| {
            let label_id = ui.label(label).id;
            let r = ui.add(egui::DragValue::new(value).range(min..=max).speed(0.1));
            r.labelled_by(label_id)
        })
        .inner
    }
}

/// A direction flip button (`⇄ label`), shown pressed while `flipped`.
pub fn flip(ui: &mut egui::Ui, label: &str, flipped: &mut bool) -> egui::Response {
    let mut r = ui
        .add(egui::Button::new(format!("⇄ {label}")).selected(*flipped))
        .on_hover_text("Flip the direction");
    r.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::Button, true, *flipped, label));
    if r.clicked() {
        *flipped = !*flipped;
        r.mark_changed();
    }
    r
}
