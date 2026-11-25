//! [`ExprInput`]: the expression field (spec §9).

use std::ops::Range;

use amg_core::{Diagnostic as _, ParseQuantityError, Quantity, Unit};
use amg_doc::expr::{self, Dim, ExprError};
use amg_doc::param::ParamTable;
use amg_doc::{ParamId, UnitPrefs};
use egui::text::{CCursor, CCursorRange, LayoutJob, TextFormat};

use super::{DEFAULT_NUDGE_STEP, display_unit, format_quantity};

/// At most this many function / unit names are offered (every matching parameter is).
pub const MAX_RESERVED_COMPLETIONS: usize = 12;

/// The outcome of checking an expression for a field.
#[derive(Clone, Debug, PartialEq)]
pub struct ExprCheck {
    /// The value (canonical units) or why there is none.
    pub value: Result<Quantity, ExprError>,
    /// Byte span to underline: the error's own span, the decimal comma, or the whole
    /// (trimmed) expression for errors without a span (wrong result dimension, cycles).
    pub span: Option<Range<usize>>,
    /// The user-facing message (`None` when the value is fine).
    pub message: Option<String>,
    /// What to do about it.
    pub hint: Option<String>,
}

/// Checks `src` for a field of dimension `expected`: [`ParamTable::try_eval`] plus the
/// display details. A `,` between two digits outside a function call is reported as
/// [`ParseQuantityError::DecimalComma`] (QUESTIONS #11: `.` is the only decimal separator).
pub fn check_expr(src: &str, expected: Dim, params: &ParamTable, prefs: &UnitPrefs) -> ExprCheck {
    let e = match params.try_eval(src, Some(expected), prefs) {
        Ok(q) => {
            return ExprCheck {
                value: Ok(q),
                span: None,
                message: None,
                hint: None,
            };
        }
        Err(e) => e,
    };
    if let Some(comma) = decimal_comma(src) {
        let qe = ParseQuantityError::DecimalComma {
            span: comma.clone(),
        };
        return ExprCheck {
            value: Err(ExprError::Parse {
                span: comma.clone(),
                msg: qe.to_string(),
            }),
            span: Some(comma),
            message: Some(qe.to_string()),
            hint: qe.hint(),
        };
    }
    let message = if src.trim().is_empty() {
        "enter a value".to_owned()
    } else {
        match &e {
            ExprError::WrongDimension { expected, got } => {
                format!("unit mismatch: expected {expected}, got {got}")
            }
            ExprError::Parse { msg, .. } => format!("syntax error: {msg}"),
            other => other.to_string(),
        }
    };
    let span = whole(src).and_then(|w| e.span().and_then(|s| clamp_span(src, s)).or(Some(w)));
    ExprCheck {
        hint: e.hint(),
        value: Err(e),
        span,
        message: Some(message),
    }
}

/// A `,` with a digit on both sides at parenthesis depth 0 (`12,5`; `max(1,5)` is fine).
fn decimal_comma(src: &str) -> Option<Range<usize>> {
    let b = src.as_bytes();
    let mut depth = 0i32;
    for (i, &c) in b.iter().enumerate() {
        match c {
            b'(' => depth += 1,
            b')' => depth -= 1,
            b',' if depth <= 0
                && i > 0
                && b[i - 1].is_ascii_digit()
                && b.get(i + 1).is_some_and(u8::is_ascii_digit) =>
            {
                return Some(i..i + 1);
            }
            _ => {}
        }
    }
    None
}

/// `s` clamped into `src` on char boundaries; an empty span (end of input) covers the char
/// before it.
fn clamp_span(src: &str, s: Range<usize>) -> Option<Range<usize>> {
    let end = s.end.min(src.len());
    let mut start = s.start.min(end);
    if start == end {
        start = src[..end].char_indices().next_back().map(|(i, _)| i)?;
    }
    (src.is_char_boundary(start) && src.is_char_boundary(end)).then_some(start..end)
}

/// The trimmed expression's span (`None` when blank).
fn whole(src: &str) -> Option<Range<usize>> {
    let start = src.len() - src.trim_start().len();
    let end = src.trim_end().len();
    (start < end).then_some(start..end)
}

/// What a completion inserts.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CompletionKind {
    /// A parameter of the design.
    Param,
    /// A function (`sin`, `max`); completing adds `(`.
    Function,
    /// `PI`, `pi`, `E`.
    Constant,
    /// A unit spelling (`mm`, `deg`).
    Unit,
}

impl CompletionKind {
    /// Shown next to the name in the list.
    pub fn label(self) -> &'static str {
        match self {
            CompletionKind::Param => "parameter",
            CompletionKind::Function => "function",
            CompletionKind::Constant => "constant",
            CompletionKind::Unit => "unit",
        }
    }
}

/// One autocomplete entry.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Completion {
    /// The name.
    pub name: String,
    /// What it is.
    pub kind: CompletionKind,
}

impl Completion {
    /// The text that replaces the word at the cursor.
    pub fn insert_text(&self) -> String {
        match self.kind {
            CompletionKind::Function => format!("{}(", self.name),
            _ => self.name.clone(),
        }
    }
}

fn ident_start(c: char) -> bool {
    c.is_alphabetic() || c == '_'
}

fn ident_continue(c: char) -> bool {
    c.is_alphabetic() || c.is_ascii_digit() || c == '_'
}

/// Autocomplete for the word before byte `cursor` in `src`: the byte range it replaces (the
/// whole word under the cursor) and the candidates. Every parameter whose name starts with the
/// word comes first (by name), then up to [`MAX_RESERVED_COMPLETIONS`] functions, constants
/// and unit names. `None` if the cursor is not after an identifier start or nothing but the
/// word itself matches.
pub fn completions(
    src: &str,
    cursor: usize,
    params: &ParamTable,
) -> Option<(Range<usize>, Vec<Completion>)> {
    let mut cursor = cursor.min(src.len());
    while !src.is_char_boundary(cursor) {
        cursor -= 1;
    }
    // The identifier-character run ending at the cursor; the word starts at its first
    // identifier-start character (`12mm` completes `mm`).
    let run_start = src[..cursor]
        .char_indices()
        .rev()
        .take_while(|&(_, c)| ident_continue(c))
        .last()
        .map_or(cursor, |(i, _)| i);
    let start = src[run_start..cursor]
        .char_indices()
        .find(|&(_, c)| ident_start(c))
        .map(|(i, _)| run_start + i)?;
    let end = cursor
        + src[cursor..]
            .char_indices()
            .find(|&(_, c)| !ident_continue(c))
            .map_or(src.len() - cursor, |(i, _)| i);
    let word = &src[start..cursor];
    let mut names: Vec<&str> = params
        .iter()
        .map(|(_, p)| p.name.as_str())
        .filter(|n| n.starts_with(word))
        .collect();
    names.sort_unstable();
    let mut out: Vec<Completion> = names
        .into_iter()
        .map(|n| Completion {
            name: n.to_owned(),
            kind: CompletionKind::Param,
        })
        .collect();
    let mut reserved: Vec<Completion> = expr::reserved_names()
        .into_iter()
        .filter(|n| n.starts_with(word))
        .map(|n| Completion {
            name: n.to_owned(),
            kind: reserved_kind(n),
        })
        .collect();
    reserved.sort_by_key(|c| match c.kind {
        CompletionKind::Function => 0,
        CompletionKind::Constant => 1,
        _ => 2,
    });
    reserved.truncate(MAX_RESERVED_COMPLETIONS);
    out.extend(reserved);
    let whole_word = &src[start..end];
    if out.is_empty() || (out.len() == 1 && out[0].name == whole_word) {
        return None;
    }
    Some((start..end, out))
}

fn reserved_kind(name: &str) -> CompletionKind {
    if Unit::from_symbol(name).is_some() {
        CompletionKind::Unit
    } else if matches!(name, "PI" | "pi" | "E") {
        CompletionKind::Constant
    } else {
        CompletionKind::Function
    }
}

/// Per-field state kept in egui memory.
#[derive(Clone, Debug, Default)]
struct ExprState {
    /// Text when the field got focus (`Esc` restores it).
    original: Option<String>,
    /// Cursor byte offset after the last frame.
    cursor: usize,
    /// Highlighted completion.
    pick: usize,
    /// The completion list was dismissed (`Esc`) for this text.
    dismissed: Option<String>,
    /// The next focus loss is a revert, not a commit.
    suppress_commit: bool,
    /// The field had the focus at the end of its last draw.
    had_focus: bool,
}

/// What [`ExprInput::show`] reports.
#[derive(Debug)]
pub struct ExprResponse {
    /// The text field's response.
    pub response: egui::Response,
    /// The value (canonical units) or the error.
    pub value: Result<Quantity, ExprError>,
    /// `Enter` or focus loss committed the text this frame (not after `Esc`).
    pub committed: bool,
    /// The text changed this frame (typing, completion, nudge or revert; additive).
    pub changed: bool,
    /// The underlined byte span (additive).
    pub error_span: Option<Range<usize>>,
    /// The error message shown (additive).
    pub message: Option<String>,
    /// The hint shown under the field (additive).
    pub hint: Option<String>,
}

impl ExprResponse {
    /// True if the expression evaluated.
    pub fn is_valid(&self) -> bool {
        self.value.is_ok()
    }
}

/// An expression field: text, evaluated value and errors (see the [module docs](super)).
#[must_use = "call `show`"]
pub struct ExprInput<'a> {
    src: &'a mut String,
    expected: Dim,
    params: &'a ParamTable,
    prefs: &'a UnitPrefs,
    bound: Option<ParamId>,
    step: f64,
    label: Option<&'a str>,
    id_salt: Option<egui::Id>,
    width: f32,
    widget_id: Option<egui::Id>,
    show_value: bool,
    name: Option<String>,
    checker: Option<&'a dyn Fn(&str) -> ExprCheck>,
}

impl std::fmt::Debug for ExprInput<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ExprInput")
            .field("src", &self.src)
            .field("expected", &self.expected)
            .finish_non_exhaustive()
    }
}

impl<'a> ExprInput<'a> {
    /// A field editing `src`, whose value must have dimension `expected`.
    pub fn new(
        src: &'a mut String,
        expected: Dim,
        params: &'a ParamTable,
        prefs: &'a UnitPrefs,
    ) -> Self {
        ExprInput {
            src,
            expected,
            params,
            prefs,
            bound: None,
            step: DEFAULT_NUDGE_STEP,
            label: None,
            id_salt: None,
            width: 120.0,
            widget_id: None,
            show_value: true,
            name: None,
            checker: None,
        }
    }

    /// The model parameter this field edits: its name is shown with the value (`d3 = 20.00
    /// mm`).
    pub fn bound_param(mut self, id: Option<ParamId>) -> Self {
        self.bound = id;
        self
    }

    /// Up/Down step in the display unit (default [`DEFAULT_NUDGE_STEP`]).
    pub fn nudge_step(mut self, step: f64) -> Self {
        self.step = step;
        self
    }

    /// The label shown before the field and given to AccessKit (additive).
    pub fn label(mut self, label: &'a str) -> Self {
        self.label = Some(label);
        self
    }

    /// Identity of the field when the label is not unique (additive).
    pub fn id_salt(mut self, salt: impl std::hash::Hash + std::fmt::Debug) -> Self {
        self.id_salt = Some(egui::Id::new(("amg_expr_salt", salt)));
        self
    }

    /// Width of the text field, points (additive; default 120).
    pub fn desired_width(mut self, width: f32) -> Self {
        self.width = width;
        self
    }

    /// The text field's widget id, for a field whose position in the layout changes, such as
    /// a table cell (additive, UI-12; overrides [`id_salt`](Self::id_salt)).
    pub fn id(mut self, id: egui::Id) -> Self {
        self.widget_id = Some(id);
        self
    }

    /// Whether the value or the error message is drawn next to the field (default true). A
    /// table with its own value column turns it off; the underline and the error tooltip
    /// stay (additive, UI-12).
    pub fn show_value(mut self, show: bool) -> Self {
        self.show_value = show;
        self
    }

    /// The AccessKit name of a field without a visible [`label`](Self::label) (additive,
    /// UI-12).
    pub fn accessible_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Replaces [`check_expr`] for the underline, the message and the returned value: for
    /// fields with other rules than "a value of dimension `expected`" (a parameter's own
    /// unit, cycles). It may run several times per frame (additive, UI-12).
    pub fn checker(mut self, checker: &'a dyn Fn(&str) -> ExprCheck) -> Self {
        self.checker = Some(checker);
        self
    }

    /// Draws the field.
    pub fn show(self, ui: &mut egui::Ui) -> ExprResponse {
        let ExprInput {
            src,
            expected,
            params,
            prefs,
            bound,
            step,
            label,
            id_salt,
            width,
            widget_id,
            show_value,
            name,
            checker,
        } = self;
        let salt = id_salt.unwrap_or_else(|| egui::Id::new(label.unwrap_or("expr")));
        let text_id = widget_id.unwrap_or_else(|| ui.make_persistent_id(("amg_expr_input", salt)));
        let run_check = |s: &str| match checker {
            Some(c) => c(s),
            None => check_expr(s, expected, params, prefs),
        };
        let state_id = text_id.with("state");
        let mut st: ExprState = ui.data(|d| d.get_temp(state_id)).unwrap_or_default();
        let focused = ui.memory(|m| m.has_focus(text_id));
        let mut changed = false;
        let mut new_cursor: Option<usize> = None;

        if focused {
            let comps = if st.dismissed.as_deref() == Some(src.as_str()) {
                None
            } else {
                completions(src, st.cursor, params)
            };
            let key = |ui: &egui::Ui, k: egui::Key| {
                ui.input_mut(|i| i.consume_key(egui::Modifiers::NONE, k))
            };
            if let Some((range, list)) = comps {
                st.pick = st.pick.min(list.len() - 1);
                if key(ui, egui::Key::Tab) {
                    let text = list[st.pick].insert_text();
                    src.replace_range(range.clone(), &text);
                    new_cursor = Some(range.start + text.len());
                    changed = true;
                    st.pick = 0;
                } else if key(ui, egui::Key::ArrowDown) {
                    st.pick = (st.pick + 1) % list.len();
                } else if key(ui, egui::Key::ArrowUp) {
                    st.pick = (st.pick + list.len() - 1) % list.len();
                } else if key(ui, egui::Key::Escape) {
                    st.dismissed = Some(src.clone());
                }
            } else if key(ui, egui::Key::Escape) {
                if let Some(o) = st.original.clone()
                    && *o != **src
                {
                    *src = o;
                    changed = true;
                }
                st.suppress_commit = true;
                ui.memory_mut(|m| m.surrender_focus(text_id));
            } else {
                let dir = if key(ui, egui::Key::ArrowUp) {
                    1.0
                } else if key(ui, egui::Key::ArrowDown) {
                    -1.0
                } else {
                    0.0
                };
                if dir != 0.0 && nudge(src, dir * step, expected, params, prefs) {
                    changed = true;
                    new_cursor = Some(src.len());
                }
            }
        } else {
            st.original = Some(src.clone());
            st.dismissed = None;
        }
        if let Some(b) = new_cursor {
            let chars = src[..b].chars().count();
            let mut ts =
                egui::text_edit::TextEditState::load(ui.ctx(), text_id).unwrap_or_default();
            ts.cursor
                .set_char_range(Some(CCursorRange::one(CCursor::new(chars))));
            ts.store(ui.ctx(), text_id);
        }

        let mut layouter = |ui: &egui::Ui, text: &dyn egui::TextBuffer, wrap: f32| {
            let s = text.as_str();
            let span = run_check(s).span;
            let mut job = highlight_job(ui, s, span);
            job.wrap.max_width = wrap;
            ui.fonts_mut(|f| f.layout_job(job))
        };

        let inner = ui.horizontal(|ui| {
            let label_id = label.map(|l| ui.label(l).id);
            let out = egui::TextEdit::singleline(&mut *src)
                .id(text_id)
                .desired_width(width)
                .layouter(&mut layouter)
                .event_filter(egui::EventFilter {
                    tab: true,
                    horizontal_arrows: true,
                    vertical_arrows: true,
                    escape: true,
                })
                .show(ui);
            let mut response = out.response.response;
            if let Some(l) = label_id {
                response = response.labelled_by(l);
            }
            if let Some(r) = out.cursor_range {
                st.cursor = src
                    .char_indices()
                    .nth(r.primary.index.0)
                    .map_or(src.len(), |(i, _)| i);
            }
            response
        });
        let mut response = inner.inner;
        if response.changed() {
            changed = true;
            st.pick = 0;
        }
        // Focus changes are tracked here rather than with `Response::lost_focus`, which a
        // repeated (discarded) pass reports again: a commit is reported exactly once.
        let now_focused = ui.memory(|m| m.has_focus(text_id));
        let committed = st.had_focus && !now_focused && !st.suppress_commit;
        if now_focused && !st.had_focus {
            st.suppress_commit = false;
        }
        st.had_focus = now_focused;

        if let Some(n) = name {
            ui.ctx()
                .accesskit_node_builder(response.id, |node| node.set_label(n));
        }
        let check = run_check(src);
        let bound_name = bound.and_then(|id| params.get(id)).map(|p| p.name.as_str());
        let error_color = ui.visuals().error_fg_color;
        match &check.value {
            Ok(q) if show_value => {
                let v = format_quantity(*q, prefs);
                let text = match bound_name {
                    Some(n) => format!("{n} = {v}"),
                    None => v,
                };
                ui.weak(text);
            }
            Ok(_) => {}
            Err(_) => {
                let msg = check.message.clone().unwrap_or_default();
                let tip = match &check.hint {
                    Some(h) => format!("{msg}\n{h}"),
                    None => msg.clone(),
                };
                response = response.on_hover_text(tip);
                if show_value {
                    if let Some(n) = bound_name {
                        ui.weak(format!("{n} = ?"));
                    }
                    ui.colored_label(error_color, msg);
                    if let Some(h) = &check.hint {
                        ui.small(h.as_str());
                    }
                }
            }
        }

        // The completion list, under the field while it has focus.
        if ui.memory(|m| m.has_focus(text_id))
            && st.dismissed.as_deref() != Some(src.as_str())
            && let Some((range, list)) = completions(src, st.cursor, params)
        {
            st.pick = st.pick.min(list.len() - 1);
            let mut chosen = None;
            egui::Area::new(text_id.with("completions"))
                .order(egui::Order::Foreground)
                .fixed_pos(response.rect.left_bottom())
                .show(ui.ctx(), |ui| {
                    egui::Frame::popup(ui.style()).show(ui, |ui| {
                        for (i, c) in list.iter().enumerate() {
                            ui.horizontal(|ui| {
                                if ui.selectable_label(i == st.pick, c.name.as_str()).clicked() {
                                    chosen = Some(i);
                                }
                                ui.weak(c.kind.label());
                            });
                        }
                    });
                });
            if let Some(i) = chosen {
                let text = list[i].insert_text();
                src.replace_range(range.clone(), &text);
                let b = range.start + text.len();
                st.cursor = b;
                st.pick = 0;
                changed = true;
                let chars = src[..b].chars().count();
                let mut ts =
                    egui::text_edit::TextEditState::load(ui.ctx(), text_id).unwrap_or_default();
                ts.cursor
                    .set_char_range(Some(CCursorRange::one(CCursor::new(chars))));
                ts.store(ui.ctx(), text_id);
                ui.memory_mut(|m| m.request_focus(text_id));
            }
        }
        ui.data_mut(|d| d.insert_temp(state_id, st));
        // A completion click changed the text after the check above.
        let check = if changed { run_check(src) } else { check };
        if changed {
            response.mark_changed();
        }
        ExprResponse {
            response,
            value: check.value,
            committed,
            changed,
            error_span: check.span,
            message: check.message,
            hint: check.hint,
        }
    }
}

/// Up/Down on a single literal (`12 mm`, `1/4 in`): adds `delta` in the display unit and
/// writes the value back with its unit. Expressions are left alone (nudging would replace
/// them). Returns true if the text changed.
fn nudge(
    src: &mut String,
    delta: f64,
    expected: Dim,
    params: &ParamTable,
    prefs: &UnitPrefs,
) -> bool {
    if amg_core::parse_quantity(src.trim()).is_err() {
        return false;
    }
    let Ok(q) = params.try_eval(src, Some(expected), prefs) else {
        return false;
    };
    let text = match display_unit(q.dim, prefs) {
        Some(u) => {
            let Ok(v) = q.in_unit(u) else {
                return false;
            };
            match expr::format_value(Quantity::from_unit(v + delta, u), u, 6) {
                Ok(t) => t,
                Err(_) => return false,
            }
        }
        None => {
            let t = format!("{:.6}", q.value + delta);
            t.trim_end_matches('0').trim_end_matches('.').to_owned()
        }
    };
    if text == *src {
        return false;
    }
    *src = text;
    true
}

/// The text as one layout job; `span` is drawn in the error colour and underlined.
fn highlight_job(ui: &egui::Ui, s: &str, span: Option<Range<usize>>) -> LayoutJob {
    let font = egui::TextStyle::Body.resolve(ui.style());
    let normal = TextFormat::simple(font.clone(), ui.visuals().text_color());
    let mut job = LayoutJob::default();
    match span {
        Some(r) if r.end <= s.len() && r.start < r.end => {
            let err = ui.visuals().error_fg_color;
            let bad = TextFormat {
                underline: egui::Stroke::new(1.5, err),
                color: err,
                ..TextFormat::simple(font, err)
            };
            job.append(&s[..r.start], 0.0, normal.clone());
            job.append(&s[r.clone()], 0.0, bad);
            job.append(&s[r.end..], 0.0, normal);
        }
        _ => job.append(s, 0.0, normal),
    }
    job
}

#[cfg(test)]
mod tests {
    use amg_core::Dimension;
    use amg_doc::model::Design;

    use super::*;

    const L: Dim = Dimension::LENGTH;

    fn design() -> Design {
        let mut d = Design::new("t");
        d.add_param("d1", "10 mm", Some(Unit::Millimetre), "")
            .unwrap();
        d.add_param("depth", "3 mm", Some(Unit::Millimetre), "")
            .unwrap();
        d.add_param("width", "40 mm", Some(Unit::Millimetre), "")
            .unwrap();
        d
    }

    #[test]
    fn check_reports_values_spans_and_the_decimal_comma() {
        let d = design();
        let p = UnitPrefs::default();
        let ok = check_expr("d1 + 5", L, d.params(), &p);
        assert_eq!(ok.value, Ok(Quantity::new(15.0, L)));
        assert_eq!(ok.span, None);

        let kg = check_expr("12 kg", L, d.params(), &p);
        assert!(matches!(kg.value, Err(ExprError::WrongDimension { .. })));
        assert_eq!(kg.span, Some(0..5));
        assert!(kg.message.unwrap().starts_with("unit mismatch"));

        let mix = check_expr("d1 + 2 kg", L, d.params(), &p);
        assert!(matches!(mix.value, Err(ExprError::UnitMismatch { .. })));
        assert_eq!(mix.span, Some(0..9));

        let unknown = check_expr("2 * dx", L, d.params(), &p);
        assert_eq!(unknown.span, Some(4..6));

        let comma = check_expr("12,5", L, d.params(), &p);
        assert_eq!(comma.span, Some(2..3));
        assert_eq!(comma.hint.as_deref(), Some("Write 12.5, not 12,5."));
        // A comma between function arguments is not a decimal comma.
        let max = check_expr("max(1 mm,5 mm)", L, d.params(), &p);
        assert!(max.value.is_ok());

        let blank = check_expr("  ", L, d.params(), &p);
        assert!(blank.value.is_err());
        assert_eq!(blank.span, None);
    }

    #[test]
    fn completions_list_params_first_and_replace_the_word() {
        let d = design();
        let (range, list) = completions("d", 1, d.params()).unwrap();
        assert_eq!(range, 0..1);
        let params: Vec<&str> = list
            .iter()
            .filter(|c| c.kind == CompletionKind::Param)
            .map(|c| c.name.as_str())
            .collect();
        assert_eq!(params, ["d1", "depth"]);
        assert_eq!(list[0].name, "d1");
        assert!(
            list.iter()
                .any(|c| c.kind == CompletionKind::Unit && c.name == "deg")
        );

        // Mid-expression, units after a number, functions get a parenthesis.
        let (r, l) = completions("2 * wi + 1", 6, d.params()).unwrap();
        assert_eq!(r, 4..6);
        assert_eq!(l[0].name, "width");
        let (r, l) = completions("12m", 3, d.params()).unwrap();
        assert_eq!(r, 2..3);
        assert!(l.iter().all(|c| c.name.starts_with('m')));
        let (_, l) = completions("sq", 2, d.params()).unwrap();
        assert_eq!(l[0].insert_text(), "sqrt(");
        // Nothing to complete after a number or a complete unique name.
        assert!(completions("12", 2, d.params()).is_none());
        assert!(completions("width", 5, d.params()).is_none());
    }

    #[test]
    fn nudge_changes_literals_only() {
        let d = design();
        let p = UnitPrefs::default();
        let mut s = "12 mm".to_owned();
        assert!(nudge(&mut s, 1.0, L, d.params(), &p));
        assert_eq!(s, "13 mm");
        let mut s = "5".to_owned();
        assert!(nudge(&mut s, -0.5, L, d.params(), &p));
        assert_eq!(s, "4.5 mm");
        let mut s = "d1 + 5".to_owned();
        assert!(!nudge(&mut s, 1.0, L, d.params(), &p));
    }
}
