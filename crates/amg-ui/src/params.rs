//! The Parameters dialog (spec S11 §17, UI-12; Fusion "Change Parameters").
//!
//! - [`PARAMETERS_CMD`] ("Change Parameters", SOLID ▸ Modify, SURFACE ▸ Modify and S-key
//!   search) opens one non-modal [`ParametersDialog`] per document.
//! - [`ParametersView`] is the dialog's content: a filter, the row that adds a user parameter
//!   and a virtualised table (favourite, name, unit, expression, value, comment, delete).
//!   Favourites are pinned on top, then the user parameters, then the model parameters grouped
//!   by feature in timeline order.
//! - Every edit is one S01 command in its own [`CommitBatch`], so one undo entry (`AddParam`,
//!   `RenameParam`, `SetParamExpr`, `SetParamMeta`, `DeleteParam`). A text cell commits on
//!   `Enter` or when it loses the focus; `Esc` reverts it.
//! - The command is first tried on a copy of the design ([`amg_doc::apply_command`]; copies
//!   are cheap). A rejected edit (cycle, unit mismatch, name in use, parameter in use, ...)
//!   is never submitted: the text stays in the cell and the message is shown in the row's
//!   value column. Expression cells show these errors while the user types.

use std::borrow::Cow;
use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};

use amg_core::{Diagnostic as _, FormatSpec, FormatStyle, Quantity, Unit};
use amg_doc::model::Design;
use amg_doc::{
    Command, CommandError, ExprError, FeatureId, Param, ParamId, ParamKind, Revision, UnitPrefs,
};
use egui::{Id, RichText};
use egui_extras::{Column, TableBuilder, TableRow};

use crate::app::{AppCx, CommitBatch};
use crate::command::{CmdAction, CmdId, CommandDef, Placement};
use crate::dialog::AppDialog;
use crate::module::{GroupId, Registrar, UiModule};
use crate::registry::UiRegistry;
use crate::widgets::{DISPLAY_PRECISION, ExprCheck, ExprInput, check_expr, format_quantity};

/// Opens the Parameters dialog.
pub const PARAMETERS_CMD: CmdId = CmdId("params.show");
/// The dialog's window title.
pub const DIALOG_TITLE: &str = "Parameters";
/// The view flag that is on while the document's dialog is open.
const OPEN_FLAG: &str = "params.open";

/// Group title of the pinned favourites.
pub const FAVOURITES_GROUP: &str = "Favourites";
/// Group title of the user parameters.
pub const USER_GROUP: &str = "User Parameters";
/// Prefix of a model parameter group title; the feature's timeline label follows.
pub const MODEL_GROUP: &str = "Model Parameters";

/// The units offered for a new user parameter (besides "No unit").
const NEW_UNITS: &[&str] = &[
    "um", "mm", "cm", "m", "in", "ft", "deg", "rad", "g", "kg", "N", "MPa", "GPa", "s", "K",
    "degC", "mm^2", "mm^3",
];
/// A valid, unlikely name to try a new expression with before the user typed a name.
const PLACEHOLDER_NAME: &str = "__amg_new_parameter";

inventory::submit! { UiModule { id: "params", register } }

/// Registers the module's command.
pub fn register(r: &mut Registrar<'_>) {
    let placed = |group| Placement::Toolbar {
        group: GroupId(group),
        pinned: false,
        order: 100,
    };
    r.command(
        CommandDef::new(PARAMETERS_CMD.0, "Change Parameters", CmdAction::Run(open))
            .with_icon(crate::icons::ids::ADJUSTMENTS)
            .with_tooltip("Edit the user and model parameters of the design")
            .with_keywords(&["parameters", "variables", "expressions", "dimensions"])
            .placed(placed("design.solid.modify"))
            .placed(placed("design.surface.modify")),
    );
}

/// Opens the dialog of the document, unless it is open already.
fn open(cx: &mut AppCx<'_>) {
    if cx.env().view_flags.contains(OPEN_FLAG) {
        return;
    }
    cx.set_view_flag(OPEN_FLAG, true);
    cx.open_dialog(Box::new(ParametersDialog::default()));
}

/// The non-modal Parameters window of one document.
#[derive(Debug, Default)]
pub struct ParametersDialog {
    view: ParametersView,
}

impl ParametersDialog {
    /// The table.
    pub fn view(&self) -> &ParametersView {
        &self.view
    }
}

impl AppDialog for ParametersDialog {
    fn title(&self) -> Cow<'static, str> {
        Cow::Borrowed(DIALOG_TITLE)
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        // An O(1) copy, so the table can read the design while it submits to `cx`.
        let design = cx.env().design.clone();
        let revision = cx.doc_tab().document.history().revision();
        let mut open = true;
        let mut batches = Vec::new();
        egui::Window::new(DIALOG_TITLE)
            .id(Id::new(("amg_parameters", cx.doc_tab().id)))
            .open(&mut open)
            .default_pos([230.0, 110.0])
            .default_size([880.0, 440.0])
            .show(ctx, |ui| {
                batches = self.view.show(ui, &design, revision, cx.registry());
            });
        if !open {
            // Closing the window keeps what was typed into the focused cell.
            batches.extend(self.view.flush(&design));
            cx.set_view_flag(OPEN_FLAG, false);
        }
        for b in batches {
            cx.submit(b);
        }
        open
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

/// One line of the table.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ParameterRow {
    /// A group title.
    Group(String),
    /// A parameter.
    Param(ParamId),
}

/// What in a row an edit changes.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum Cell {
    Name,
    Expression,
    Comment,
    Favourite,
    Delete,
}

/// Why an edit was rejected.
#[derive(Clone, Debug, PartialEq)]
struct Rejection {
    message: String,
    hint: Option<String>,
}

/// Text typed into a cell that the document does not have (yet).
#[derive(Clone, Debug)]
struct Draft {
    text: String,
    /// The document's text when the edit started; the draft is dropped if that changes.
    base: String,
    /// The text field, once drawn (`None` for button actions).
    widget: Option<Id>,
    /// The field had the focus at the end of the last frame.
    focused: bool,
    /// The edit was tried and rejected.
    error: Option<Rejection>,
}

/// The user parameter being typed into the "add" row.
#[derive(Debug, Default)]
struct NewParam {
    name: String,
    /// `None`: not chosen, the document length unit; `Some(None)`: no unit.
    unit: Option<Option<Unit>>,
    expr: String,
    comment: String,
    /// The last try of these fields: (name, unit, expression) and its outcome.
    checked: Option<(NewKey, Result<Quantity, Rejection>)>,
}

type NewKey = (String, Option<Unit>, String);

/// The edit state of the table.
#[derive(Debug, Default)]
struct Edits {
    drafts: BTreeMap<(ParamId, Cell), Draft>,
    /// The last check of each edited expression (cleared when the document changes).
    checks: RefCell<HashMap<ParamId, (String, ExprCheck)>>,
}

/// The content of the Parameters dialog (see the [module docs](self)). It only reads the
/// design: [`show`](Self::show) returns the batches to submit.
#[derive(Debug, Default)]
pub struct ParametersView {
    filter: String,
    rows: Vec<ParameterRow>,
    /// The document revision and filter the rows were built for.
    built_for: Option<(Revision, String)>,
    rebuilds: u64,
    edits: Edits,
    new: NewParam,
}

impl ParametersView {
    /// The table lines of the last frame (all of them, also the ones scrolled out of view).
    pub fn rows(&self) -> &[ParameterRow] {
        &self.rows
    }

    /// How often the table lines were rebuilt (only when the document or the filter changes).
    pub fn rebuilds(&self) -> u64 {
        self.rebuilds
    }

    /// Draws the filter, the add row and the table. `revision` must change whenever `design`
    /// does ([`amg_doc::History::revision`]). Returns one batch per edit finished this frame.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        design: &Design,
        revision: Revision,
        registry: &UiRegistry,
    ) -> Vec<CommitBatch> {
        // Read before a field consumes it.
        let esc = ui.input(|i| i.key_pressed(egui::Key::Escape));
        ui.horizontal(|ui| {
            let label = ui.label("Filter parameters").id;
            ui.add(
                egui::TextEdit::singleline(&mut self.filter)
                    .id(Id::new("amg_params_filter"))
                    .hint_text("name, expression or comment")
                    .desired_width(240.0),
            )
            .labelled_by(label);
        });
        self.sync(design, revision, registry);
        // Edits are tried on this copy in order, so a second edit of the same frame sees the
        // first.
        let mut work = design.clone();
        let mut batches = Vec::new();
        self.add_row(ui, &mut work, &mut batches, registry);
        ui.separator();
        let clicks = self.table(ui, design);

        // Text cells that lost the focus (Enter, Tab, a click elsewhere, scrolled away).
        let keys: Vec<(ParamId, Cell)> = self.edits.drafts.keys().copied().collect();
        for key in keys {
            let Some(d) = self.edits.drafts.get_mut(&key) else {
                continue;
            };
            let Some(widget) = d.widget else {
                continue;
            };
            let focused = ui.memory(|m| m.has_focus(widget));
            let was_focused = std::mem::replace(&mut d.focused, focused);
            if focused {
                continue;
            }
            if esc && was_focused {
                self.edits.drafts.remove(&key);
                ui.ctx().request_repaint();
            } else if d.error.is_none() {
                self.edits.commit(key, &mut work, &mut batches, registry);
                ui.ctx().request_repaint();
            }
        }
        for (id, cell) in clicks {
            let Some(p) = work.params().get(id) else {
                continue;
            };
            let (label, command) = match cell {
                Cell::Favourite => (
                    "Favourite parameter",
                    Command::SetParamMeta {
                        id,
                        comment: None,
                        favorite: Some(!p.favorite),
                    },
                ),
                _ => ("Delete parameter", Command::DeleteParam { id }),
            };
            match try_command(&work, command.clone()) {
                Ok(next) => {
                    work = next;
                    batches.push(batch(label, command));
                    self.edits.drafts.remove(&(id, cell));
                }
                Err(e) => {
                    self.edits.drafts.insert(
                        (id, cell),
                        Draft {
                            text: String::new(),
                            base: String::new(),
                            widget: None,
                            focused: false,
                            error: Some(rejection(&work, registry, &e)),
                        },
                    );
                }
            }
            ui.ctx().request_repaint();
        }
        batches
    }

    /// Commits every typed text that was not rejected, focused or not (the dialog closes).
    pub fn flush(&mut self, design: &Design) -> Vec<CommitBatch> {
        let mut work = design.clone();
        let mut batches = Vec::new();
        let registry = UiRegistry::default();
        let keys: Vec<(ParamId, Cell)> = self
            .edits
            .drafts
            .iter()
            .filter(|(_, d)| d.widget.is_some() && d.error.is_none())
            .map(|(&k, _)| k)
            .collect();
        for key in keys {
            self.edits.commit(key, &mut work, &mut batches, &registry);
        }
        batches
    }

    /// Rebuilds the rows and drops stale edit state after a document or filter change.
    fn sync(&mut self, design: &Design, revision: Revision, registry: &UiRegistry) {
        if let Some((r, f)) = &self.built_for
            && *r == revision
            && *f == self.filter
        {
            return;
        }
        let new_revision = self.built_for.as_ref().is_none_or(|(r, _)| *r != revision);
        if new_revision {
            self.edits.checks.borrow_mut().clear();
            self.new.checked = None;
            self.edits.drafts.retain(|&(id, cell), d| {
                d.widget.is_some()
                    && design
                        .params()
                        .get(id)
                        .is_some_and(|p| cell_text(p, cell) == d.base)
            });
        }
        let _span = tracing::debug_span!("amg_ui.params.rebuild").entered();
        self.rows = build_rows(design, registry, &self.filter);
        self.built_for = Some((revision, self.filter.clone()));
        self.rebuilds += 1;
    }

    /// The row that adds a user parameter, with the preview or the reason it can't be added.
    fn add_row(
        &mut self,
        ui: &mut egui::Ui,
        work: &mut Design,
        batches: &mut Vec<CommitBatch>,
        registry: &UiRegistry,
    ) {
        let prefs = work.meta().default_units.clone();
        let design = work.clone();
        let new = &mut self.new;
        let unit = *new.unit.get_or_insert(Some(prefs.length.into()));
        let mut enter = false;
        let mut add = false;
        ui.horizontal(|ui| {
            enter |= new_field(ui, &mut new.name, "New parameter name", "name", 130.0);
            let combo = egui::ComboBox::from_id_salt("amg_params_new_unit")
                .width(80.0)
                .selected_text(unit.map_or("No unit", Unit::symbol))
                .show_ui(ui, |ui| {
                    let chosen = &mut new.unit;
                    ui.selectable_value(chosen, Some(None), "No unit");
                    for u in NEW_UNITS.iter().filter_map(|s| Unit::from_symbol(s)) {
                        ui.selectable_value(chosen, Some(Some(u)), u.symbol());
                    }
                });
            set_label(ui, combo.response.id, "New parameter unit".to_owned());
            let unit = new.unit.flatten();
            let check = |src: &str| {
                let command = Command::AddParam {
                    name: PLACEHOLDER_NAME.to_owned(),
                    expr: src.to_owned(),
                    unit,
                    comment: String::new(),
                };
                expr_check(&design, registry, src, command, |d| {
                    d.params().by_name(PLACEHOLDER_NAME)
                })
            };
            let expected = unit.map_or(amg_core::Dimension::NONE, Unit::dimension);
            let r = ExprInput::new(&mut new.expr, expected, design.params(), &prefs)
                .id(Id::new("amg_params_new_expr"))
                .accessible_name("New parameter expression")
                .show_value(false)
                .checker(&check)
                .desired_width(160.0)
                .show(ui);
            enter |= r.committed && ui.input(|i| i.key_pressed(egui::Key::Enter));
            enter |= new_field(
                ui,
                &mut new.comment,
                "New parameter comment",
                "comment",
                150.0,
            );
            let ready = !new.name.trim().is_empty() && !new.expr.trim().is_empty();
            let outcome = ready.then(|| new_outcome(new, &design, registry));
            let valid = matches!(outcome, Some(Ok(_)));
            add = ui
                .add_enabled(valid, egui::Button::new("Add parameter"))
                .on_disabled_hover_text(match &outcome {
                    Some(Err(e)) => e.message.clone(),
                    _ => "Enter a name and an expression".to_owned(),
                })
                .clicked()
                || (enter && valid);
        });
        let ready = !new.name.trim().is_empty() && !new.expr.trim().is_empty();
        match ready.then(|| new_outcome(new, &design, registry)) {
            Some(Ok(q)) => {
                let unit = new.unit.flatten();
                ui.weak(format!("= {}", value_text(unit, q, &prefs)));
            }
            Some(Err(e)) => error_label(ui, &e),
            None => {
                ui.weak("Add a user parameter: name, unit and expression.");
            }
        }
        if add {
            let command = Command::AddParam {
                name: new.name.trim().to_owned(),
                expr: new.expr.clone(),
                unit: new.unit.flatten(),
                comment: new.comment.clone(),
            };
            if let Ok(next) = try_command(work, command.clone()) {
                *work = next;
                batches.push(batch("Add parameter", command));
                new.name.clear();
                new.expr.clear();
                new.comment.clear();
                new.checked = None;
                ui.ctx().request_repaint();
            }
        }
    }

    /// The virtualised table. Returns the buttons clicked.
    fn table(&mut self, ui: &mut egui::Ui, design: &Design) -> Vec<(ParamId, Cell)> {
        let mut clicks = Vec::new();
        if self.rows.is_empty() {
            ui.weak(if design.params().is_empty() {
                "The design has no parameters yet."
            } else {
                "No parameter matches the filter."
            });
            return clicks;
        }
        let prefs = &design.meta().default_units;
        let row_height = ui.spacing().interact_size.y + 4.0;
        let header_height = ui.spacing().interact_size.y;
        let height = ui.available_height().max(120.0);
        let rows = &self.rows;
        let edits = &mut self.edits;
        TableBuilder::new(ui)
            .id_salt("amg_params_table")
            .striped(true)
            .resizable(true)
            .auto_shrink([false, false])
            .min_scrolled_height(0.0)
            .max_scroll_height(height)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(Column::exact(24.0))
            .column(Column::initial(190.0).at_least(80.0).clip(true))
            .column(Column::initial(56.0).at_least(40.0).clip(true))
            .column(Column::initial(180.0).at_least(80.0).clip(true))
            .column(Column::initial(170.0).at_least(80.0).clip(true))
            .column(Column::remainder().at_least(80.0).clip(true))
            .column(Column::exact(56.0))
            .header(header_height, |mut h| {
                for title in ["★", "Name", "Unit", "Expression", "Value", "Comment", ""] {
                    h.col(|ui| {
                        ui.strong(title);
                    });
                }
            })
            .body(|body| {
                body.rows(row_height, rows.len(), |mut row| {
                    match rows.get(row.index()) {
                        Some(ParameterRow::Group(title)) => {
                            row.col(|_| {});
                            row.col(|ui| {
                                ui.strong(title.as_str()).on_hover_text(title.as_str());
                            });
                        }
                        Some(&ParameterRow::Param(id)) => {
                            if let Some(p) = design.params().get(id) {
                                edits.param_row(&mut row, design, prefs, id, p, &mut clicks);
                            }
                        }
                        None => {}
                    }
                });
            });
        clicks
    }
}

impl Edits {
    /// The cells of one parameter.
    fn param_row(
        &mut self,
        row: &mut TableRow<'_, '_>,
        design: &Design,
        prefs: &UnitPrefs,
        id: ParamId,
        p: &Param,
        clicks: &mut Vec<(ParamId, Cell)>,
    ) {
        let name = p.name.as_str();
        row.col(|ui| {
            let color = if p.favorite {
                ui.visuals().warn_fg_color
            } else {
                ui.visuals().weak_text_color()
            };
            let r = ui
                .add(egui::Button::new(RichText::new("★").color(color)).frame(false))
                .on_hover_text(if p.favorite {
                    "Remove from the favourites"
                } else {
                    "Add to the favourites"
                });
            r.widget_info(|| {
                egui::WidgetInfo::selected(
                    egui::WidgetType::Checkbox,
                    true,
                    p.favorite,
                    format!("Favourite {name}"),
                )
            });
            if r.clicked() {
                clicks.push((id, Cell::Favourite));
            }
        });
        row.col(|ui| self.text_cell(ui, id, Cell::Name, name, format!("Name {name}")));
        row.col(|ui| match p.unit {
            Some(u) => {
                ui.label(u.symbol());
            }
            None => {
                ui.weak("-").on_hover_text("No unit: any dimension");
            }
        });
        row.col(|ui| self.expr_cell(ui, design, prefs, id, p));
        row.col(|ui| self.value_cell(ui, design, prefs, id, p));
        row.col(|ui| {
            self.text_cell(ui, id, Cell::Comment, &p.comment, format!("Comment {name}"));
        });
        row.col(|ui| {
            // Model parameters go away with their feature.
            if p.kind == ParamKind::User {
                let r = ui.small_button("Delete");
                set_label(ui, r.id, format!("Delete {name}"));
                if r.clicked() {
                    clicks.push((id, Cell::Delete));
                }
            }
        });
    }

    fn text_cell(
        &mut self,
        ui: &mut egui::Ui,
        id: ParamId,
        cell: Cell,
        current: &str,
        label: String,
    ) {
        let key = (id, cell);
        let widget = Id::new(("amg_params_cell", id, cell));
        let draft = self.drafts.get(&key);
        let mut text = draft.map_or_else(|| current.to_owned(), |d| d.text.clone());
        let mut edit = egui::TextEdit::singleline(&mut text)
            .id(widget)
            .desired_width(ui.available_width());
        if draft.is_some_and(|d| d.error.is_some()) {
            edit = edit.text_color(ui.visuals().error_fg_color);
        }
        let r = ui.add(edit);
        set_label(ui, r.id, label);
        if r.changed() {
            self.typed(key, text, current, widget);
        }
    }

    fn expr_cell(
        &mut self,
        ui: &mut egui::Ui,
        design: &Design,
        prefs: &UnitPrefs,
        id: ParamId,
        p: &Param,
    ) {
        let key = (id, Cell::Expression);
        let widget = Id::new(("amg_params_cell", id, Cell::Expression));
        let current = p.expr.as_str();
        let mut text = self
            .drafts
            .get(&key)
            .map_or_else(|| current.to_owned(), |d| d.text.clone());
        let dim = design
            .params()
            .value(id)
            .map_or(amg_core::Dimension::NONE, |q| q.dim);
        let checks = &self.checks;
        let check = |src: &str| {
            if src == current
                && let Some(q) = design.params().value(id)
            {
                return ok_check(q);
            }
            if let Some((s, c)) = checks.borrow().get(&id)
                && s == src
            {
                return c.clone();
            }
            // Without the UI registry only feature labels in messages would differ, and an
            // expression edit names none.
            let command = Command::SetParamExpr {
                id,
                expr: src.to_owned(),
            };
            let c = expr_check(design, &UiRegistry::default(), src, command, |_| Some(id));
            checks.borrow_mut().insert(id, (src.to_owned(), c.clone()));
            c
        };
        let r = ExprInput::new(&mut text, dim, design.params(), prefs)
            .id(widget)
            .accessible_name(format!("Expression {}", p.name))
            .show_value(false)
            .checker(&check)
            .desired_width(ui.available_width())
            .show(ui);
        if r.changed {
            self.typed(key, text, current, widget);
        }
    }

    /// The value, or why the row's edit was (or would be) rejected.
    fn value_cell(
        &self,
        ui: &mut egui::Ui,
        design: &Design,
        prefs: &UnitPrefs,
        id: ParamId,
        p: &Param,
    ) {
        let rejected = self
            .drafts
            .range((id, Cell::Name)..=(id, Cell::Delete))
            .find_map(|(_, d)| d.error.as_ref());
        if let Some(e) = rejected {
            error_label(ui, e);
            return;
        }
        // While an expression is typed: its value, or what is wrong with it.
        if let Some(d) = self.drafts.get(&(id, Cell::Expression))
            && let Some((src, c)) = self.checks.borrow().get(&id)
            && *src == d.text
        {
            match &c.value {
                Ok(q) => {
                    ui.weak(value_text(p.unit, *q, prefs));
                }
                Err(_) => error_label(
                    ui,
                    &Rejection {
                        message: c.message.clone().unwrap_or_default(),
                        hint: c.hint.clone(),
                    },
                ),
            }
            return;
        }
        if let Some(q) = design.params().value(id) {
            ui.label(value_text(p.unit, q, prefs));
        }
    }

    /// Records typed text; an earlier rejection no longer applies to it.
    fn typed(&mut self, key: (ParamId, Cell), text: String, current: &str, widget: Id) {
        let d = self.drafts.entry(key).or_insert_with(|| Draft {
            text: String::new(),
            base: current.to_owned(),
            widget: Some(widget),
            focused: true,
            error: None,
        });
        d.text = text;
        d.widget = Some(widget);
        d.error = None;
    }

    /// Finishes a text edit: drops it if nothing changed, submits it if the command is
    /// accepted, else keeps the text with the reason.
    fn commit(
        &mut self,
        key: (ParamId, Cell),
        work: &mut Design,
        batches: &mut Vec<CommitBatch>,
        registry: &UiRegistry,
    ) {
        let (id, cell) = key;
        let Some(d) = self.drafts.get_mut(&key) else {
            return;
        };
        let Some(current) = work.params().get(id).map(|p| cell_text(p, cell).to_owned()) else {
            self.drafts.remove(&key);
            return;
        };
        let text = match cell {
            Cell::Name => d.text.trim().to_owned(),
            _ => d.text.clone(),
        };
        if text == current {
            self.drafts.remove(&key);
            return;
        }
        let (label, command) = match cell {
            Cell::Name => ("Rename parameter", Command::RenameParam { id, name: text }),
            Cell::Expression => ("Edit parameter", Command::SetParamExpr { id, expr: text }),
            Cell::Comment => (
                "Edit parameter comment",
                Command::SetParamMeta {
                    id,
                    comment: Some(text),
                    favorite: None,
                },
            ),
            Cell::Favourite | Cell::Delete => return,
        };
        match try_command(work, command.clone()) {
            Ok(next) => {
                *work = next;
                batches.push(batch(label, command));
                self.drafts.remove(&key);
            }
            Err(e) => d.error = Some(rejection(work, registry, &e)),
        }
    }
}

/// The document text a cell edits (empty for the buttons).
fn cell_text(p: &Param, cell: Cell) -> &str {
    match cell {
        Cell::Name => &p.name,
        Cell::Expression => p.expr.as_str(),
        Cell::Comment => &p.comment,
        Cell::Favourite | Cell::Delete => "",
    }
}

fn batch(label: &str, command: Command) -> CommitBatch {
    CommitBatch {
        label: label.to_owned(),
        commands: vec![command],
    }
}

/// `design` with `command` applied, or why the document would refuse it.
fn try_command(design: &Design, command: Command) -> Result<Design, CommandError> {
    let mut next = design.clone();
    amg_doc::apply_command(&mut next, command)?;
    Ok(next)
}

fn ok_check(q: Quantity) -> ExprCheck {
    ExprCheck {
        value: Ok(q),
        span: None,
        message: None,
        hint: None,
    }
}

/// Checks the expression `src` by trying `command`; `result` finds the parameter whose value
/// it sets. Unlike [`check_expr`] this applies the parameter's own unit and finds cycles.
fn expr_check(
    design: &Design,
    registry: &UiRegistry,
    src: &str,
    command: Command,
    result: impl FnOnce(&Design) -> Option<ParamId>,
) -> ExprCheck {
    let error = match try_command(design, command) {
        Ok(next) => match result(&next).and_then(|id| next.params().value(id)) {
            Some(q) => return ok_check(q),
            None => CommandError::Invalid("the expression has no value".to_owned()),
        },
        Err(e) => e,
    };
    let start = src.len() - src.trim_start().len();
    let end = src.trim_end().len();
    let whole = (start < end).then_some(start..end);
    // Blank text, syntax errors and the decimal comma: the field's own messages and spans.
    if src.trim().is_empty() || matches!(error, CommandError::Expr(ExprError::Parse { .. })) {
        let c = check_expr(
            src,
            amg_core::Dimension::NONE,
            design.params(),
            &design.meta().default_units,
        );
        if c.value.is_err() {
            return c;
        }
    }
    let r = rejection(design, registry, &error);
    let (value, span) = match error {
        CommandError::Expr(e) => {
            let span = e
                .span()
                .filter(|s| {
                    s.start < s.end && src.is_char_boundary(s.start) && src.is_char_boundary(s.end)
                })
                .filter(|s| s.end <= src.len())
                .or(whole);
            (e, span)
        }
        _ => (
            ExprError::Parse {
                span: whole.clone().unwrap_or(0..0),
                msg: r.message.clone(),
            },
            whole,
        ),
    };
    ExprCheck {
        value: Err(value),
        span,
        message: Some(r.message),
        hint: r.hint,
    }
}

/// The message for a refused command, with parameter and feature names instead of ids.
fn rejection(design: &Design, registry: &UiRegistry, e: &CommandError) -> Rejection {
    let param_name = |id: ParamId| {
        design
            .params()
            .get(id)
            .map_or_else(|| id.to_string(), |p| p.name.clone())
    };
    let (message, hint) = match e {
        CommandError::Expr(ExprError::Cycle(path)) => {
            let names: Vec<String> = path.iter().map(|&id| param_name(id)).collect();
            (
                format!("circular reference: {}", names.join(" → ")),
                Some("A parameter can't depend on itself.".to_owned()),
            )
        }
        CommandError::Expr(x) => {
            let message = match x {
                ExprError::WrongDimension { expected, got } => {
                    format!("unit mismatch: expected {expected}, got {got}")
                }
                ExprError::Parse { msg, .. } => format!("syntax error: {msg}"),
                other => other.to_string(),
            };
            (message, x.hint())
        }
        CommandError::InUse { by, .. } => {
            let labels = feature_labels(design, registry);
            let users: Vec<String> = by
                .iter()
                .map(|&raw| {
                    labels
                        .get(&FeatureId(raw))
                        .map_or_else(|| param_name(ParamId(raw)), Clone::clone)
                })
                .collect();
            (
                format!("in use by {}", users.join(", ")),
                Some("Change the expressions that use it first.".to_owned()),
            )
        }
        CommandError::NameTaken(_) => (e.to_string(), Some("Choose a different name.".to_owned())),
        other => (other.to_string(), other.hint()),
    };
    Rejection { message, hint }
}

/// The timeline labels of the features (`Extrude2` or the node's name).
fn feature_labels(design: &Design, registry: &UiRegistry) -> HashMap<FeatureId, String> {
    let mut ordinals = HashMap::new();
    design
        .timeline()
        .nodes()
        .map(|n| {
            let (_, label) = crate::timeline::node_label(n, registry, &mut ordinals);
            (n.id, label)
        })
        .collect()
}

/// The table lines: favourites, user parameters, then model parameters per feature in
/// timeline order. `filter` matches the name, expression or comment, ignoring case.
fn build_rows(design: &Design, registry: &UiRegistry, filter: &str) -> Vec<ParameterRow> {
    let needle = filter.trim().to_lowercase();
    let matches = |p: &Param| {
        needle.is_empty()
            || [p.name.as_str(), p.expr.as_str(), p.comment.as_str()]
                .iter()
                .any(|s| s.to_lowercase().contains(&needle))
    };
    let mut favourites = Vec::new();
    let mut user = Vec::new();
    let mut model: BTreeMap<FeatureId, Vec<ParamId>> = BTreeMap::new();
    for (id, p) in design.params().iter().filter(|(_, p)| matches(p)) {
        match p.kind {
            _ if p.favorite => favourites.push(id),
            ParamKind::User => user.push(id),
            ParamKind::Model { owner } => model.entry(owner).or_default().push(id),
        }
    }
    let mut rows = Vec::new();
    let mut group = |title: String, ids: Vec<ParamId>| {
        if !ids.is_empty() {
            rows.push(ParameterRow::Group(title));
            rows.extend(ids.into_iter().map(ParameterRow::Param));
        }
    };
    group(FAVOURITES_GROUP.to_owned(), favourites);
    group(USER_GROUP.to_owned(), user);
    if !model.is_empty() {
        let labels = feature_labels(design, registry);
        for node in design.timeline().nodes() {
            if let Some(ids) = model.remove(&node.id) {
                let label = labels.get(&node.id).map_or("", String::as_str);
                group(format!("{MODEL_GROUP}: {label}"), ids);
            }
        }
        // Owners outside the timeline (not produced by the commands) are still listed.
        for (owner, ids) in model {
            group(format!("{MODEL_GROUP}: {owner}"), ids);
        }
    }
    rows
}

/// A value in the parameter's unit, else like every dialog value ([`format_quantity`]).
fn value_text(unit: Option<Unit>, q: Quantity, prefs: &UnitPrefs) -> String {
    let spec = FormatSpec {
        style: FormatStyle::Decimal,
        precision: DISPLAY_PRECISION,
        trim_zeros: false,
        show_unit: true,
    };
    unit.and_then(|u| amg_core::format_quantity(q, u, &spec).ok())
        .unwrap_or_else(|| format_quantity(q, prefs))
}

/// The outcome of adding the typed parameter, cached until the fields or the document change.
fn new_outcome(
    new: &mut NewParam,
    design: &Design,
    registry: &UiRegistry,
) -> Result<Quantity, Rejection> {
    let key: NewKey = (
        new.name.trim().to_owned(),
        new.unit.flatten(),
        new.expr.clone(),
    );
    if let Some((k, outcome)) = &new.checked
        && *k == key
    {
        return outcome.clone();
    }
    let command = Command::AddParam {
        name: key.0.clone(),
        expr: key.2.clone(),
        unit: key.1,
        comment: String::new(),
    };
    let outcome = match try_command(design, command) {
        Ok(next) => next
            .params()
            .by_name(&key.0)
            .and_then(|id| next.params().value(id))
            .ok_or_else(|| Rejection {
                message: "the parameter has no value".to_owned(),
                hint: None,
            }),
        Err(e) => Err(rejection(design, registry, &e)),
    };
    new.checked = Some((key, outcome.clone()));
    outcome
}

/// A plain text field of the add row; true if `Enter` was pressed in it.
fn new_field(ui: &mut egui::Ui, text: &mut String, name: &str, hint: &str, width: f32) -> bool {
    let r = ui.add(
        egui::TextEdit::singleline(text)
            .id(Id::new(("amg_params_new", name)))
            .hint_text(hint)
            .desired_width(width),
    );
    set_label(ui, r.id, name.to_owned());
    r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter))
}

fn error_label(ui: &mut egui::Ui, e: &Rejection) {
    let tip = match &e.hint {
        Some(h) => format!("{}\n{h}", e.message),
        None => e.message.clone(),
    };
    let text = RichText::new(e.message.as_str()).color(ui.visuals().error_fg_color);
    ui.add(egui::Label::new(text).truncate()).on_hover_text(tip);
}

/// The AccessKit name of a widget without a visible label of its own.
fn set_label(ui: &egui::Ui, id: Id, label: String) {
    ui.ctx()
        .accesskit_node_builder(id, |node| node.set_label(label));
}
