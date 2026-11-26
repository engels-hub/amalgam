//! [`GenericFeatureTool`]: one dialog tool for every `DialogRegistration` (see the module docs).

use std::borrow::Cow;
use std::collections::BTreeMap;

use amg_doc::{
    Command as DocCommand, Design, Diagnostic, EntityRef, Evaluated, FeatureBox, FeatureId, Health,
    ParamTable, UnitPrefs,
};
use amg_features::dialog::{
    DialogInit, DialogRegistration, DialogSchema, FieldKey, FieldKind, FieldSpec, FieldValue,
    FieldValues, ManipAnchor, ManipKind,
};

use super::profiles;
use crate::app::{CmdEnv, CommitBatch};
use crate::command::{CmdId, Enablement};
use crate::input::ViewportEvent;
use crate::manip::{self, DragValue, GizmoPrim, ManipEvent, Manipulators};
use crate::notify::Notification;
use crate::pick::PickTarget;
use crate::preview::PreviewState;
use crate::select::{EntityFilter, SelItem, SelectTool};
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};
use crate::widgets::{
    ChoiceField, DialogAction, DialogFrame, ExprInput, IntField, SelectionField, check_expr, flip,
    group, selection_count_valid, toggle,
};

/// The feature being edited.
#[derive(Debug)]
struct Editing {
    id: FeatureId,
    old: FeatureBox,
    /// The values `read` gave (after the selection items were resolved).
    initial: Option<FieldValues>,
}

/// One selection field's items and active flag.
#[derive(Clone, Debug, Default)]
struct SelState {
    items: Vec<SelItem>,
    active: bool,
}

/// A field problem: the message and whether it came from the registration's `build`
/// (expression fields show their own parse errors).
#[derive(Clone, Debug, PartialEq)]
struct FieldError {
    message: String,
    from_build: bool,
}

/// What the current values give: the feature, or the problems.
#[derive(Debug)]
struct Checked {
    errors: BTreeMap<FieldKey, FieldError>,
    /// The first problem as `Label: message` (OK's disabled reason).
    reason: Option<String>,
    feature: Option<FeatureBox>,
}

/// A schema field driven by a manipulator (UI-06): its anchor from `anchors`. The gizmo id is
/// the binding's index.
#[derive(Clone, Debug)]
struct Binding {
    field: FieldKey,
    kind: ManipKind,
    dim: amg_doc::Dim,
    anchor: ManipAnchor,
}

/// The drag in progress: the bound field and its value and text before the drag.
#[derive(Clone, Debug)]
struct FieldDrag {
    id: u32,
    field: FieldKey,
    start: f64,
    start_src: String,
    /// Cursor, points (the heads-up value follows it).
    pos: egui::Pos2,
}

/// Offset of the heads-up value from the cursor, points.
const HEADS_UP_OFFSET: egui::Vec2 = egui::vec2(18.0, 18.0);

/// The dialog tool of one feature kind, generated from its [`DialogRegistration`].
pub struct GenericFeatureTool {
    cmd: CmdId,
    reg: &'static DialogRegistration,
    schema: &'static DialogSchema,
    edit: Option<Editing>,
    values: FieldValues,
    sel: BTreeMap<FieldKey, SelState>,
    /// Selection refs from `defaults` / `read`, resolved to items in `start`.
    pending_refs: Option<FieldValues>,
    picker: SelectTool,
    /// The values the preview was last requested for.
    previewed: Option<FieldValues>,
    /// Auto-focus ran.
    focused: bool,
    hover_profile: Option<EntityRef>,
    /// OK's state from the last update.
    ok: Enablement,
    /// The gizmos of the fields that declare a manipulator (UI-06).
    manips: Manipulators,
    bindings: Vec<Binding>,
    field_drag: Option<FieldDrag>,
}

impl std::fmt::Debug for GenericFeatureTool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GenericFeatureTool")
            .field("cmd", &self.cmd)
            .field("kind", &self.reg.kind)
            .field("edit", &self.edit.as_ref().map(|e| e.id))
            .field("values", &self.values)
            .finish_non_exhaustive()
    }
}

/// Whether a field is shown for `v`.
fn visible(spec: &FieldSpec, v: &FieldValues) -> bool {
    spec.visible_if.as_ref().is_none_or(|c| c.holds(v))
}

/// Whether a field is editable for `v`.
fn enabled(spec: &FieldSpec, v: &FieldValues) -> bool {
    spec.enabled_if.as_ref().is_none_or(|c| c.holds(v))
}

/// The selection item of a stored reference: its displayed target, else a best guess (a body
/// without display, a profile's sketch).
fn item_for_ref(
    design: &Design,
    eval: &Evaluated,
    scene: &crate::scene::SceneAdapter,
    e: &EntityRef,
) -> Option<SelItem> {
    if profiles::is_profile(e) {
        return profiles::item(e.clone());
    }
    let target = crate::select::target_for_entity(design, eval, scene, e).or(match e {
        EntityRef::Body(b) => Some(PickTarget::Body {
            body: *b,
            instance: 0,
            sub: None,
        }),
        EntityRef::Topo(t) => Some(PickTarget::Body {
            body: t.body,
            instance: 0,
            sub: None,
        }),
        EntityRef::Construction(c) => Some(PickTarget::Construction(*c)),
        EntityRef::InContext { path, .. } => path.first().map(|o| PickTarget::Occurrence(*o)),
        EntityRef::SketchCurve { sketch, .. } | EntityRef::SketchPoint { sketch, .. } => {
            Some(PickTarget::Sketch(*sketch))
        }
        EntityRef::Profile { .. } => None,
    })?;
    Some(SelItem {
        target,
        entity: e.clone(),
    })
}

impl GenericFeatureTool {
    /// The dialog of `reg`, for a new feature (`init.edit == None`) or for editing one.
    ///
    /// # Errors
    /// A user-facing message when the edited feature is gone or `read` does not take it.
    pub fn new(
        reg: &'static DialogRegistration,
        init: &ToolInit<'_>,
    ) -> Result<GenericFeatureTool, String> {
        let schema = (reg.schema)();
        let design = init.env.design;
        let (values, edit) = match init.edit {
            Some(id) => {
                let Some((_, node)) = design.timeline().node(id) else {
                    return Err("The feature to edit no longer exists.".into());
                };
                let Some(v) = (reg.read)(node.feature.get(), design) else {
                    return Err(format!("This feature can't be edited as {}.", schema.title));
                };
                (
                    v,
                    Some(Editing {
                        id,
                        old: node.feature.clone(),
                        initial: None,
                    }),
                )
            }
            None => {
                let refs: Vec<EntityRef> =
                    init.preselection.iter().map(|i| i.entity.clone()).collect();
                let v = (reg.defaults)(&DialogInit {
                    preselection: &refs,
                    design,
                    evaluated: init.env.evaluated,
                });
                (v, None)
            }
        };
        let mut values = values;
        let mut sel = BTreeMap::new();
        for f in &schema.fields {
            match &f.kind {
                FieldKind::Selection { .. } => {
                    sel.insert(f.key, SelState::default());
                }
                FieldKind::Expr { default, .. } => {
                    values
                        .0
                        .entry(f.key)
                        .or_insert_with(|| FieldValue::expr(default));
                }
                _ => {}
            }
        }
        Ok(GenericFeatureTool {
            cmd: init.command,
            reg,
            schema,
            edit,
            pending_refs: Some(values.clone()),
            values,
            sel,
            picker: SelectTool::default(),
            previewed: None,
            focused: false,
            hover_profile: None,
            ok: Enablement::disabled("starting"),
            manips: Manipulators::new(),
            bindings: Vec::new(),
            field_drag: None,
        })
    }

    /// The registration.
    pub fn registration(&self) -> &'static DialogRegistration {
        self.reg
    }

    /// The current field values (tests and scripting read them).
    pub fn values(&self) -> &FieldValues {
        &self.values
    }

    /// The current field values, mutable (tests and scripting drive the dialog through
    /// them; selection fields are set with [`Self::set_selection`]).
    pub fn values_mut(&mut self) -> &mut FieldValues {
        &mut self.values
    }

    /// Replaces the items of selection field `key` (tests, scripting). False if there is no
    /// such field.
    pub fn set_selection(&mut self, key: &str, items: Vec<SelItem>) -> bool {
        match self.sel.iter_mut().find(|(k, _)| k.0 == key) {
            Some((_, st)) => {
                st.items = items;
                self.sync_values();
                true
            }
            None => false,
        }
    }

    /// The selection field that is active, if any.
    pub fn active_field(&self) -> Option<FieldKey> {
        self.sel.iter().find(|(_, s)| s.active).map(|(k, _)| *k)
    }

    /// The feature being edited (`None` for a new one).
    pub fn editing(&self) -> Option<FeatureId> {
        self.edit.as_ref().map(|e| e.id)
    }

    /// Whether OK is enabled, with the reason when not (as of the last frame).
    pub fn ok_state(&self) -> &Enablement {
        &self.ok
    }

    /// The manipulators of this frame (UI-06; tests and scripting read their gizmos).
    pub fn manipulators(&self) -> &Manipulators {
        &self.manips
    }

    /// The gizmo that drives field `key`, as placed for the current value.
    pub fn gizmo_of(&self, key: &str) -> Option<GizmoPrim> {
        let id = self.bindings.iter().position(|b| b.field.0 == key)?;
        let id = u32::try_from(id).ok()?;
        self.manips.gizmos().into_iter().find(|g| g.id() == id)
    }

    /// The field a manipulator drag is changing, if one is in progress.
    pub fn dragged_field(&self) -> Option<FieldKey> {
        self.field_drag.as_ref().map(|d| d.field)
    }

    /// The value (canonical units) of expression field `key` as typed now.
    fn expr_value(&self, env: &CmdEnv<'_>, key: FieldKey, dim: amg_doc::Dim) -> Option<f64> {
        let Some(FieldValue::Expr { src, .. }) = self.values.0.get(&key) else {
            return None;
        };
        let prefs = &env.design.meta().default_units;
        env.design
            .params()
            .try_eval(src, Some(dim), prefs)
            .ok()
            .map(|q| q.value)
    }

    /// Re-reads the anchors of the fields that declare a manipulator (on the latest preview,
    /// else the document) and places their gizmos at the current values.
    fn refresh_manipulators(&mut self, cx: &mut ToolCx<'_>) {
        let preview = match cx.preview().state() {
            PreviewState::Ready(ev) => Some(std::sync::Arc::clone(ev)),
            _ => None,
        };
        let env = cx.env();
        let anchors = self.check(&env, &self.values).feature.map(|f| {
            let mut a = preview
                .as_ref()
                .map(|ev| (self.reg.anchors)(f.get(), env.design, ev))
                .unwrap_or_default();
            if a.is_empty() {
                a = (self.reg.anchors)(f.get(), env.design, env.evaluated);
            }
            a
        });
        let mut bindings = Vec::new();
        for a in anchors.unwrap_or_default() {
            let Some(spec) = self.spec(a.field) else {
                continue;
            };
            let (Some(kind), FieldKind::Expr { dim, .. }) = (spec.manipulator, &spec.kind) else {
                continue;
            };
            if !visible(spec, &self.values) || !enabled(spec, &self.values) {
                continue;
            }
            bindings.push(Binding {
                field: a.field,
                kind,
                dim: *dim,
                anchor: a,
            });
        }
        // A drag keeps its binding while the values cannot be built for a moment.
        if bindings.is_empty() && self.field_drag.is_some() {
            bindings.clone_from(&self.bindings);
        }
        let mut gizmos = Vec::new();
        for (i, b) in bindings.iter().enumerate() {
            let Ok(id) = u32::try_from(i) else {
                break;
            };
            if let Some(v) = self.expr_value(&env, b.field, b.dim) {
                gizmos.push(place(b, v, id));
            }
        }
        self.bindings = bindings;
        self.manips.set(gizmos);
    }

    /// Applies a gizmo event to its bound field: a drag writes the formatted value (replacing
    /// any expression), `Esc` restores the text from before the drag.
    fn on_manip(&mut self, ev: ManipEvent, cx: &mut ToolCx<'_>) {
        match ev {
            ManipEvent::Begin { id, .. } => {
                let Some(b) = usize::try_from(id).ok().and_then(|i| self.bindings.get(i)) else {
                    return;
                };
                let (field, dim, base) = (b.field, b.dim, b.anchor.base);
                let start = self.expr_value(&cx.env(), field, dim).unwrap_or(base);
                let start_src = match self.values.0.get(&field) {
                    Some(FieldValue::Expr { src, .. }) => src.clone(),
                    _ => String::new(),
                };
                let pos = self.manips.cursor().unwrap_or_default();
                self.field_drag = Some(FieldDrag {
                    id,
                    field,
                    start,
                    start_src,
                    pos,
                });
            }
            ManipEvent::Drag {
                id,
                value,
                snap,
                pos,
            } => {
                if let Some(d) = &mut self.field_drag {
                    d.pos = pos;
                }
                self.apply_drag(id, value, snap, cx);
            }
            ManipEvent::End { id, value, snap } => {
                if let Some(v) = value {
                    self.apply_drag(id, v, snap, cx);
                }
                self.field_drag = None;
            }
            ManipEvent::Cancel { .. } => {
                if let Some(d) = self.field_drag.take() {
                    self.set_field_text(d.field, d.start_src);
                }
            }
        }
        cx.app().request_repaint();
    }

    /// Writes `start + change` (snapped with `Ctrl`) into the dragged field.
    fn apply_drag(&mut self, id: u32, value: DragValue, snap: bool, cx: &mut ToolCx<'_>) {
        let Some(d) = self.field_drag.as_ref().filter(|d| d.id == id) else {
            return;
        };
        let Some(b) = usize::try_from(id).ok().and_then(|i| self.bindings.get(i)) else {
            return;
        };
        let change = if b.dim == amg_doc::Dim::ANGLE {
            match value {
                DragValue::Angle(a) => Some(a),
                _ => None,
            }
        } else {
            value.along(drive_dir(b))
        };
        let Some(change) = change else {
            return;
        };
        let env = cx.env();
        let prefs = &env.design.meta().default_units;
        let mut v = d.start + change;
        if snap {
            v = manip::snap(v, manip::snap_step(b.dim, prefs));
        }
        let text = manip::field_text(v, b.dim, prefs);
        let field = d.field;
        self.set_field_text(field, text);
    }

    /// Replaces the text of expression field `key` (a bound model parameter stays bound).
    fn set_field_text(&mut self, key: FieldKey, text: String) {
        if let Some(FieldValue::Expr { src, .. }) = self.values.0.get_mut(&key) {
            *src = text;
        }
    }

    /// The heads-up value next to the cursor while a manipulator is dragged.
    fn heads_up(&mut self, ctx: &egui::Context, params: &ParamTable, prefs: &UnitPrefs) {
        let Some(d) = self.field_drag.clone() else {
            return;
        };
        let Some(spec) = self.spec(d.field) else {
            return;
        };
        let FieldKind::Expr { dim, .. } = spec.kind else {
            return;
        };
        let Some(FieldValue::Expr { src, .. }) = self.values.0.get_mut(&d.field) else {
            return;
        };
        egui::Area::new(egui::Id::new(("amg_heads_up", d.field.0)))
            .order(egui::Order::Tooltip)
            .fixed_pos(d.pos + HEADS_UP_OFFSET)
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    let _ = ExprInput::new(src, dim, params, prefs)
                        .accessible_name(format!("Heads-up {}", spec.label))
                        .id_salt(("amg_heads_up_field", d.field.0))
                        .desired_width(90.0)
                        .show(ui);
                });
            });
    }

    /// Writes the selection items into the values.
    fn sync_values(&mut self) {
        for (k, st) in &self.sel {
            let refs = st.items.iter().map(|i| i.entity.clone()).collect();
            self.values.0.insert(*k, FieldValue::Selection(refs));
        }
    }

    /// The pick filter of a selection field.
    fn filter_of(spec: &FieldSpec) -> EntityFilter {
        match &spec.kind {
            FieldKind::Selection { filter, .. } => EntityFilter::from(*filter),
            _ => EntityFilter::NONE,
        }
    }

    fn spec(&self, key: FieldKey) -> Option<&'static FieldSpec> {
        self.schema.field(key)
    }

    /// The active selection field's spec, if it is shown.
    fn active_spec(&self) -> Option<&'static FieldSpec> {
        let k = self.active_field()?;
        self.spec(k).filter(|s| visible(s, &self.values))
    }

    /// Whether a shown selection field takes profiles.
    fn shows_profiles(&self) -> bool {
        self.schema
            .fields
            .iter()
            .any(|f| visible(f, &self.values) && Self::filter_of(f).contains(EntityFilter::PROFILE))
    }

    /// Checks the values: field rules of the shown fields first, then `build`.
    fn check(&self, env: &CmdEnv<'_>, values: &FieldValues) -> Checked {
        let mut errors: BTreeMap<FieldKey, FieldError> = BTreeMap::new();
        let params = env.design.params();
        let prefs = &env.design.meta().default_units;
        for f in &self.schema.fields {
            if !visible(f, values) || !enabled(f, values) {
                continue;
            }
            let msg = match (&f.kind, values.0.get(&f.key)) {
                (FieldKind::Expr { dim, .. }, Some(FieldValue::Expr { src, .. })) => {
                    check_expr(src, *dim, params, prefs).message
                }
                (FieldKind::Expr { .. }, _) => Some("enter a value".to_owned()),
                (FieldKind::Selection { min, max, .. }, v) => {
                    let n = match v {
                        Some(FieldValue::Selection(s)) => s.len(),
                        _ => 0,
                    };
                    (!selection_count_valid(n, *min, *max)).then(|| match max {
                        Some(m) if *m == *min => format!("select {m}"),
                        Some(m) if selection_count_valid(n, 0, Some(*m)) => {
                            format!("select at least {min}")
                        }
                        Some(m) => format!("select at most {m}"),
                        None => format!("select at least {min}"),
                    })
                }
                _ => None,
            };
            if let Some(message) = msg {
                errors.insert(
                    f.key,
                    FieldError {
                        message,
                        from_build: false,
                    },
                );
            }
        }
        let mut feature = None;
        if errors.is_empty() {
            match (self.reg.build)(values, self.edit.as_ref().map(|e| e.old.get())) {
                Ok(f) => feature = Some(f),
                Err(fe) => {
                    for (k, m) in fe.0 {
                        errors.entry(k).or_insert(FieldError {
                            message: m,
                            from_build: true,
                        });
                    }
                    if errors.is_empty() {
                        errors.insert(
                            FieldKey(""),
                            FieldError {
                                message: "the values don't describe a feature".into(),
                                from_build: true,
                            },
                        );
                    }
                }
            }
        }
        // The first problem in field order (unknown keys last).
        let reason = self
            .schema
            .fields
            .iter()
            .find_map(|f| {
                errors
                    .get(&f.key)
                    .map(|e| format!("{}: {}", f.label, e.message))
            })
            .or_else(|| errors.values().next().map(|e| e.message.clone()));
        Checked {
            errors,
            reason,
            feature,
        }
    }

    /// Expression fields bound to a model parameter whose text differs from the parameter.
    fn changed_params(&self, params: &ParamTable) -> Vec<(amg_doc::ParamId, String)> {
        let mut out = Vec::new();
        for f in &self.schema.fields {
            if let (
                FieldKind::Expr { .. },
                Some(FieldValue::Expr {
                    src,
                    param: Some(p),
                }),
            ) = (&f.kind, self.values.0.get(&f.key))
            {
                let current = params.get(*p).map(|q| q.expr.as_str().trim().to_owned());
                if current.as_deref() != Some(src.trim()) {
                    out.push((*p, src.trim().to_owned()));
                }
            }
        }
        out
    }

    /// The values to preview in edit mode: bound fields whose text changed are inline, so one
    /// `EditFeature` shows the result of `EditFeature` + `SetParamExpr`.
    fn preview_values(&self, params: &ParamTable) -> FieldValues {
        let changed = self.changed_params(params);
        let mut v = self.values.clone();
        for val in v.0.values_mut() {
            if let FieldValue::Expr { param, .. } = val
                && param.is_some_and(|p| changed.iter().any(|(q, _)| *q == p))
            {
                *param = None;
            }
        }
        v
    }

    /// The command to preview now, or why there is none.
    fn preview_command(&self, env: &CmdEnv<'_>) -> Option<DocCommand> {
        match &self.edit {
            None => {
                let feature = self.check(env, &self.values).feature?;
                Some(DocCommand::AddFeature {
                    at: None,
                    feature,
                    context: env.active_component,
                    name: None,
                })
            }
            Some(e) => {
                if e.initial.as_ref() == Some(&self.values) {
                    // Nothing changed: the document already shows it.
                    return None;
                }
                // Field rules on the values as typed, `build` on the inline ones.
                self.check(env, &self.values).feature?;
                let v = self.preview_values(env.design.params());
                let feature = self.check(env, &v).feature?;
                Some(DocCommand::EditFeature { id: e.id, feature })
            }
        }
    }

    /// The health of the previewed feature, or the preview's failure.
    fn preview_health(&self, env: &CmdEnv<'_>, state: &PreviewState) -> Option<Health> {
        match state {
            PreviewState::Ready(ev) => {
                let id = match &self.edit {
                    Some(e) => Some(e.id),
                    None => {
                        let timeline = env.design.timeline();
                        ev.nodes()
                            .find(|(id, _)| timeline.node(*id).is_none())
                            .map(|(id, _)| id)
                    }
                };
                id.and_then(|id| ev.node(id)).map(|n| n.health.clone())
            }
            PreviewState::Failed(m) => Some(Health::Error(Diagnostic {
                message: m.clone(),
                cause: None,
                entities: Vec::new(),
            })),
            _ => None,
        }
    }

    /// The active field takes the document selection (what UI-04's rules picked) plus its own
    /// profile picks.
    fn mirror_selection(&mut self, sel: &crate::select::Selection) {
        let Some(spec) = self.active_spec() else {
            return;
        };
        let FieldKind::Selection { max, .. } = &spec.kind else {
            return;
        };
        let filter = Self::filter_of(spec);
        let Some(st) = self.sel.get_mut(&spec.key) else {
            return;
        };
        let mut items: Vec<SelItem> = st
            .items
            .iter()
            .filter(|i| profiles::is_profile(&i.entity))
            .cloned()
            .collect();
        items.extend(
            sel.items()
                .iter()
                .filter(|i| filter.admits(&i.target) && !profiles::is_profile(&i.entity))
                .cloned(),
        );
        if let Some(m) = max.and_then(|m| usize::try_from(m).ok()) {
            let drop = items.len().saturating_sub(m);
            items.drain(..drop);
        }
        if items != st.items {
            st.items = items;
        }
    }

    /// Adds or removes a profile pick in the active field (a single-pick field replaces).
    fn toggle_profile(&mut self, e: EntityRef) -> bool {
        let Some(spec) = self.active_spec() else {
            return false;
        };
        let FieldKind::Selection { max, .. } = &spec.kind else {
            return false;
        };
        let Some(st) = self.sel.get_mut(&spec.key) else {
            return false;
        };
        if let Some(i) = st.items.iter().position(|i| profiles::same(&i.entity, &e)) {
            st.items.remove(i);
        } else {
            if *max == Some(1) {
                st.items.clear();
            }
            if let Some(item) = profiles::item(e) {
                st.items.push(item);
            }
        }
        true
    }
}

/// The direction a length drag is read along: the anchor axis (arrow, triad), or the in-plane
/// reference direction (plane handle).
fn drive_dir(b: &Binding) -> amg_core::Vec3 {
    match b.kind {
        ManipKind::Plane => manip::frame_from(b.anchor.axis, b.anchor.ref_dir).x_axis,
        _ => b.anchor.axis.get(),
    }
}

/// The gizmo of binding `b` for the field value `v` (canonical units): the anchor shows
/// `anchor.base` at `anchor.origin`.
fn place(b: &Binding, v: f64, id: u32) -> GizmoPrim {
    let a = &b.anchor;
    let d = v - a.base;
    let angle = b.dim == amg_doc::Dim::ANGLE;
    let frame = manip::frame_from(a.axis, a.ref_dir);
    match b.kind {
        ManipKind::Distance => GizmoPrim::Arrow {
            origin: a.origin + a.axis.get() * d,
            dir: a.axis,
            id,
            hot: false,
        },
        ManipKind::Angle => GizmoPrim::Ring {
            center: a.origin,
            axis: a.axis,
            ref_dir: amg_core::Dir3::new_unchecked(manip::rotate(
                frame.x_axis,
                a.axis.get(),
                if angle { d } else { 0.0 },
            )),
            id,
            hot: false,
        },
        ManipKind::Triad => GizmoPrim::Triad {
            origin: if angle {
                a.origin
            } else {
                a.origin + a.axis.get() * d
            },
            frame: if angle {
                let r = |c: amg_core::Vec3| manip::rotate(c, a.axis.get(), d);
                amg_core::Mat3::from_cols(r(frame.x_axis), r(frame.y_axis), frame.z_axis)
            } else {
                frame
            },
            id,
            hot_part: None,
        },
        ManipKind::Plane => GizmoPrim::PlaneHandle {
            origin: if angle {
                a.origin
            } else {
                a.origin + frame.x_axis * d
            },
            normal: a.axis,
            id,
            hot: false,
        },
    }
}

/// Draws the fields. Returns the selection fields that became active and those whose clear
/// button emptied them while active.
struct FieldsUi<'a> {
    schema: &'static DialogSchema,
    values: &'a mut FieldValues,
    sel: &'a mut BTreeMap<FieldKey, SelState>,
    errors: &'a BTreeMap<FieldKey, FieldError>,
    params: &'a ParamTable,
    prefs: &'a UnitPrefs,
    activated: Option<FieldKey>,
    cleared: bool,
}

impl FieldsUi<'_> {
    fn all(&mut self, ui: &mut egui::Ui) {
        let schema = self.schema;
        let mut groups: Vec<&'static str> = Vec::new();
        for spec in &schema.fields {
            if !visible(spec, self.values) {
                continue;
            }
            match &spec.group {
                None => self.field(ui, spec),
                Some(g) if groups.contains(&g.as_str()) => {}
                Some(g) => {
                    groups.push(g.as_str());
                    group(ui, g, |ui| {
                        for s in schema
                            .fields
                            .iter()
                            .filter(|s| s.group.as_deref() == Some(g.as_str()))
                        {
                            if visible(s, self.values) {
                                self.field(ui, s);
                            }
                        }
                    });
                }
            }
        }
    }

    fn field(&mut self, ui: &mut egui::Ui, spec: &'static FieldSpec) {
        let on = enabled(spec, self.values);
        ui.add_enabled_ui(on, |ui| {
            let response = self.widget(ui, spec);
            if let (Some(r), Some(t)) = (response, &spec.tooltip) {
                r.on_hover_text(t.as_str());
            }
        });
        if let Some(e) = self.errors.get(&spec.key)
            && (e.from_build || !matches!(spec.kind, FieldKind::Expr { .. }))
        {
            ui.colored_label(ui.visuals().error_fg_color, e.message.as_str());
        }
    }

    fn widget(&mut self, ui: &mut egui::Ui, spec: &'static FieldSpec) -> Option<egui::Response> {
        let label = spec.label.as_str();
        match &spec.kind {
            FieldKind::Selection { filter, min, max } => {
                let st = self.sel.entry(spec.key).or_default();
                let (was_active, had) = (st.active, !st.items.is_empty());
                let r = SelectionField::new(
                    label,
                    &mut st.items,
                    EntityFilter::from(*filter),
                    &mut st.active,
                )
                .limits(*min, *max)
                .show(ui);
                if st.active && !was_active {
                    self.activated = Some(spec.key);
                }
                if st.active && had && st.items.is_empty() {
                    self.cleared = true;
                }
                let refs = st.items.iter().map(|i| i.entity.clone()).collect();
                self.values.0.insert(spec.key, FieldValue::Selection(refs));
                Some(r)
            }
            FieldKind::Expr { dim, default } => {
                let v = self
                    .values
                    .0
                    .entry(spec.key)
                    .or_insert_with(|| FieldValue::expr(default));
                if !matches!(v, FieldValue::Expr { .. }) {
                    *v = FieldValue::expr(default);
                }
                let FieldValue::Expr { src, param } = v else {
                    return None;
                };
                let r = ExprInput::new(src, *dim, self.params, self.prefs)
                    .label(label)
                    .bound_param(*param)
                    .id_salt(("amg_feature_field", spec.key.0))
                    .show(ui);
                Some(r.response)
            }
            FieldKind::Choice { options, style } => {
                let opts: Vec<(&'static str, &str)> =
                    options.iter().map(|o| (o.id, o.label.as_str())).collect();
                let mut cur = match self.values.0.get(&spec.key) {
                    Some(FieldValue::Choice(c)) => *c,
                    _ => options.first().map_or("", |o| o.id),
                };
                let mut field = ChoiceField::new(label, &mut cur, &opts);
                if *style == amg_features::dialog::ChoiceStyle::IconRow {
                    field = field.buttons();
                }
                let r = field.show(ui);
                self.values.0.insert(spec.key, FieldValue::Choice(cur));
                Some(r)
            }
            FieldKind::Bool => {
                let mut b = matches!(self.values.0.get(&spec.key), Some(FieldValue::Bool(true)));
                let r = toggle(ui, label, &mut b);
                self.values.0.insert(spec.key, FieldValue::Bool(b));
                Some(r)
            }
            FieldKind::Int { min, max } => {
                let mut i = match self.values.0.get(&spec.key) {
                    Some(FieldValue::Int(i)) => *i,
                    _ => *min,
                };
                let r = IntField::new(label, &mut i).range(*min, *max).show(ui);
                self.values.0.insert(spec.key, FieldValue::Int(i));
                Some(r)
            }
            FieldKind::Text => {
                let mut s = match self.values.0.get(&spec.key) {
                    Some(FieldValue::Text(s)) => s.clone(),
                    _ => String::new(),
                };
                let r = ui
                    .horizontal(|ui| {
                        let l = ui.label(label).id;
                        ui.text_edit_singleline(&mut s).labelled_by(l)
                    })
                    .inner;
                self.values.0.insert(spec.key, FieldValue::Text(s));
                Some(r)
            }
            FieldKind::FlipDirection => {
                let mut b = matches!(self.values.0.get(&spec.key), Some(FieldValue::Flip(true)));
                let r = flip(ui, label, &mut b);
                self.values.0.insert(spec.key, FieldValue::Flip(b));
                Some(r)
            }
        }
    }
}

impl Tool for GenericFeatureTool {
    fn id(&self) -> CmdId {
        self.cmd
    }

    fn label(&self) -> Cow<'static, str> {
        Cow::Borrowed(self.schema.title.as_str())
    }

    fn start(&mut self, cx: &mut ToolCx<'_>) {
        // Stored references → items with display targets (needs the scene).
        if let Some(v) = self.pending_refs.take() {
            let (design, eval) = {
                let env = cx.env();
                (env.design.clone(), std::sync::Arc::clone(env.evaluated))
            };
            for (k, st) in &mut self.sel {
                if let Some(FieldValue::Selection(refs)) = v.0.get(k) {
                    st.items = refs
                        .iter()
                        .filter_map(|e| item_for_ref(&design, &eval, cx.scene(), e))
                        .collect();
                }
            }
            self.sync_values();
        }
        if let Some(e) = &mut self.edit {
            e.initial = Some(self.values.clone());
        }
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        // Manipulators first (routing priority: manipulator → tool).
        if let Some(cam) = cx.camera().cloned() {
            let r = self.manips.input(ev, &cam);
            if r.changed {
                cx.app().request_repaint();
            }
            if let Some(e) = r.event {
                self.on_manip(e, cx);
            }
            if r.result == InputResult::Consumed {
                return InputResult::Consumed;
            }
        }
        match ev {
            ViewportEvent::Click {
                button: egui::PointerButton::Primary,
                ray,
                ..
            } if self
                .active_spec()
                .is_some_and(|s| Self::filter_of(s).contains(EntityFilter::PROFILE)) =>
            {
                let hit = {
                    let env = cx.env();
                    profiles::at_ray(env.design, env.evaluated, ray)
                };
                if let Some(e) = hit
                    && self.toggle_profile(e)
                {
                    self.sync_values();
                    cx.app().request_repaint();
                    return InputResult::Consumed;
                }
            }
            ViewportEvent::Hover { ray, .. } if self.shows_profiles() => {
                let hit = {
                    let env = cx.env();
                    profiles::at_ray(env.design, env.evaluated, ray)
                };
                if hit != self.hover_profile {
                    self.hover_profile = hit;
                    cx.app().request_repaint();
                }
            }
            ViewportEvent::Key {
                key: egui::Key::Enter,
                pressed: true,
                ..
            } if !self.ok.is_enabled() => {
                // Enter commits only a valid dialog.
                return InputResult::Consumed;
            }
            _ => {}
        }
        self.picker.input(ev, cx)
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        // The forwarded select tool polls its click and box picks.
        self.picker.update(cx);
        // Hidden fields cannot stay active.
        let hidden: Vec<FieldKey> = self
            .sel
            .keys()
            .copied()
            .filter(|k| self.spec(*k).is_none_or(|s| !visible(s, &self.values)))
            .collect();
        for k in hidden {
            if let Some(st) = self.sel.get_mut(&k) {
                st.active = false;
            }
        }
        let sel = cx.selection().clone();
        self.mirror_selection(&sel);
        self.sync_values();
        let (cmd, ok) = {
            let env = cx.env();
            let checked = self.check(&env, &self.values);
            let ok = match checked.reason {
                Some(r) => Enablement::Disabled(r.into()),
                None => Enablement::Enabled,
            };
            (self.preview_command(&env), ok)
        };
        self.ok = ok;
        self.refresh_manipulators(cx);
        if self.manips.dragging().is_some() {
            cx.set_cursor(egui::CursorIcon::Grabbing);
        } else if self.manips.hot().is_some() {
            cx.set_cursor(egui::CursorIcon::Grab);
        }
        match cmd {
            Some(cmd) => {
                if self.previewed.as_ref() != Some(&self.values) {
                    self.previewed = Some(self.values.clone());
                    cx.preview().set(cmd);
                }
            }
            None => {
                if self.previewed.take().is_some() {
                    cx.preview().clear();
                }
            }
        }
    }

    fn dialog_ui(&mut self, ui: &mut egui::Ui, cx: &mut ToolCx<'_>) -> bool {
        if !self.focused {
            self.focused = true;
            // Auto-focus: the first shown, editable, empty selection field.
            let first = self.schema.fields.iter().find(|f| {
                matches!(f.kind, FieldKind::Selection { .. })
                    && visible(f, &self.values)
                    && enabled(f, &self.values)
                    && self.sel.get(&f.key).is_none_or(|s| s.items.is_empty())
            });
            if let Some(f) = first
                && let Some(st) = self.sel.get_mut(&f.key)
            {
                st.active = true;
            }
        }
        let state = cx.preview().state().clone();
        let (params, prefs, health, checked) = {
            let env = cx.env();
            (
                env.design.params().clone(),
                env.design.meta().default_units.clone(),
                self.preview_health(&env, &state),
                self.check(&env, &self.values),
            )
        };
        let ok = match &checked.reason {
            Some(r) => Enablement::Disabled(r.clone().into()),
            None => Enablement::Enabled,
        };
        let computing = matches!(state, PreviewState::Pending | PreviewState::Computing);
        let mut fields = FieldsUi {
            schema: self.schema,
            values: &mut self.values,
            sel: &mut self.sel,
            errors: &checked.errors,
            params: &params,
            prefs: &prefs,
            activated: None,
            cleared: false,
        };
        let action = DialogFrame::new(self.schema.title.as_str())
            .banner(health.as_ref())
            .ok_enabled(ok)
            .show_inside(ui, |ui| {
                fields.all(ui);
                if computing {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.label("Computing preview…");
                    });
                }
            });
        let (activated, cleared) = (fields.activated, fields.cleared);
        self.heads_up(ui.ctx(), &params, &prefs);
        if let Some(k) = activated {
            // The newly active field shows its items as the document selection.
            let items: Vec<SelItem> = self
                .sel
                .get(&k)
                .map(|s| {
                    s.items
                        .iter()
                        .filter(|i| !profiles::is_profile(&i.entity))
                        .cloned()
                        .collect()
                })
                .unwrap_or_default();
            cx.selection_mut().set(items);
        } else if cleared {
            cx.selection_mut().clear();
        }
        match action {
            DialogAction::Ok => cx.request_finish(FinishReason::Commit),
            DialogAction::Cancel => cx.request_finish(FinishReason::Cancel),
            DialogAction::None => {}
        }
        true
    }

    fn pick_filter(&self) -> EntityFilter {
        self.schema
            .fields
            .iter()
            .filter(|f| visible(f, &self.values))
            .fold(EntityFilter::NONE, |acc, f| acc | Self::filter_of(f))
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        let title = &self.schema.title;
        Some(match self.active_spec() {
            Some(f) => format!("{title}: pick {}, then OK (Enter) or Esc", f.label).into(),
            None => format!("{title}: set the options, then OK (Enter) or Esc").into(),
        })
    }

    fn overlay(&mut self, out: &mut amg_render::OverlayList, cx: &ToolCx<'_>) {
        if let Some(cam) = cx.camera() {
            self.manips.draw(out, cam);
        }
        if !self.shows_profiles() {
            return;
        }
        let picked: Vec<&EntityRef> = self
            .sel
            .values()
            .flat_map(|s| s.items.iter().map(|i| &i.entity))
            .filter(|e| profiles::is_profile(e))
            .collect();
        let env = cx.env();
        profiles::draw(
            out,
            env.design,
            env.evaluated,
            &picked,
            self.hover_profile.as_ref(),
        );
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn finish(&mut self, reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        self.manips.reset();
        self.field_drag = None;
        if reason != FinishReason::Commit {
            return None;
        }
        self.sync_values();
        let env = cx.env();
        let feature = self.check(&env, &self.values).feature?;
        let title = &self.schema.title;
        match &self.edit {
            None => Some(CommitBatch {
                label: title.clone(),
                commands: vec![DocCommand::AddFeature {
                    at: None,
                    feature,
                    context: env.active_component,
                    name: None,
                }],
            }),
            Some(e) => {
                let mut commands = Vec::new();
                if feature != e.old {
                    commands.push(DocCommand::EditFeature { id: e.id, feature });
                }
                for (id, expr) in self.changed_params(env.design.params()) {
                    commands.push(DocCommand::SetParamExpr { id, expr });
                }
                (!commands.is_empty()).then(|| CommitBatch {
                    label: format!("Edit {title}"),
                    commands,
                })
            }
        }
    }
}

/// A tool that only explains why the command cannot run, then ends.
pub(crate) struct Unavailable {
    cmd: CmdId,
    message: String,
}

impl Unavailable {
    pub(crate) fn new(cmd: CmdId, message: String) -> Unavailable {
        Unavailable { cmd, message }
    }
}

impl Tool for Unavailable {
    fn id(&self) -> CmdId {
        self.cmd
    }

    fn label(&self) -> Cow<'static, str> {
        Cow::Borrowed(self.cmd.0)
    }

    fn start(&mut self, cx: &mut ToolCx<'_>) {
        cx.app().notify(Notification::info(
            "Cannot open the dialog",
            self.message.clone(),
        ));
        cx.request_finish(FinishReason::Cancel);
    }

    fn input(&mut self, _ev: &ViewportEvent, _cx: &mut ToolCx<'_>) -> InputResult {
        InputResult::Ignored
    }

    fn finish(&mut self, _reason: FinishReason, _cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        None
    }
}
