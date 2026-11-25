//! Declarative feature dialogs (S11 Interfaces §B, adopted by S04 SOL-01; lane A owns the module).
//!
//! A feature kind registers a [`DialogRegistration`] with `inventory`: its [`DialogSchema`]
//! (fields, conditions, manipulators), how to fill the fields from the preselection
//! (`defaults`) or from an existing feature (`read`), how to build the feature from the field
//! values (`build`) and where its manipulators sit (`anchors`). S11 UI-05 renders every
//! registration generically (dialog, command, toolbar and timeline entry) and S18 builds the
//! Python bindings from the same data. A [`DispatchRegistration`] is a command without a schema
//! that opens another kind's dialog chosen from the selection (press-pull, S04 SOL-09).
//!
//! The types live below `amg-ui` so every lane's features can describe their dialogs. Other
//! lanes register their own schemas here; the module only holds the shared shape.

use std::collections::BTreeMap;

use amg_doc::{Design, Dim, EntityRef, Evaluated, Expr, ExprSrc, Feature, FeatureBox, ParamId};
use serde::{Deserialize, Serialize};

/// Which kinds of entity a selection field accepts (a bit set; S11 §B layout).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct EntityFilter(pub u32);

impl EntityFilter {
    /// Nothing.
    pub const NONE: EntityFilter = EntityFilter(0);
    /// Whole bodies.
    pub const BODY: EntityFilter = EntityFilter(1);
    /// B-rep faces.
    pub const FACE: EntityFilter = EntityFilter(1 << 1);
    /// Edges.
    pub const EDGE: EntityFilter = EntityFilter(1 << 2);
    /// Vertices.
    pub const VERTEX: EntityFilter = EntityFilter(1 << 3);
    /// Mesh face groups.
    pub const MESH_FACE_GROUP: EntityFilter = EntityFilter(1 << 4);
    /// Whole sketches.
    pub const SKETCH: EntityFilter = EntityFilter(1 << 5);
    /// Sketch curves.
    pub const SKETCH_CURVE: EntityFilter = EntityFilter(1 << 6);
    /// Sketch points.
    pub const SKETCH_POINT: EntityFilter = EntityFilter(1 << 7);
    /// Closed sketch profiles.
    pub const PROFILE: EntityFilter = EntityFilter(1 << 8);
    /// Construction planes.
    pub const CONSTRUCTION_PLANE: EntityFilter = EntityFilter(1 << 9);
    /// Construction axes.
    pub const CONSTRUCTION_AXIS: EntityFilter = EntityFilter(1 << 10);
    /// Construction points.
    pub const CONSTRUCTION_POINT: EntityFilter = EntityFilter(1 << 11);
    /// Occurrences.
    pub const OCCURRENCE: EntityFilter = EntityFilter(1 << 12);
    /// Planar faces only (narrows [`EntityFilter::FACE`]).
    pub const PLANAR_FACE: EntityFilter = EntityFilter(1 << 13);
    /// Everything.
    pub const ALL: EntityFilter = EntityFilter(0x3fff);

    /// The raw bits.
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// Whether every kind of `other` is in the set.
    pub const fn contains(self, other: EntityFilter) -> bool {
        self.0 & other.0 == other.0
    }

    /// Whether the sets share a kind.
    pub const fn intersects(self, other: EntityFilter) -> bool {
        self.0 & other.0 != 0
    }

    /// Kinds in either set.
    #[must_use]
    pub const fn union(self, other: EntityFilter) -> EntityFilter {
        EntityFilter(self.0 | other.0)
    }

    /// Whether `e` is of a kind in the set (`PLANAR_FACE` admits faces; the planarity check
    /// is the feature's).
    pub fn admits(self, e: &EntityRef) -> bool {
        use amg_doc::TopoKind;
        let bit = match e {
            EntityRef::Body(_) => EntityFilter::BODY,
            EntityRef::Topo(t) => match t.kind {
                TopoKind::Face => EntityFilter::FACE.union(EntityFilter::PLANAR_FACE),
                TopoKind::Edge => EntityFilter::EDGE,
                TopoKind::Vertex => EntityFilter::VERTEX,
                TopoKind::FaceGroup => EntityFilter::MESH_FACE_GROUP,
            },
            EntityRef::SketchCurve { .. } => EntityFilter::SKETCH_CURVE,
            EntityRef::SketchPoint { .. } => EntityFilter::SKETCH_POINT,
            EntityRef::Profile { .. } => EntityFilter::PROFILE,
            EntityRef::Construction(_) => EntityFilter::CONSTRUCTION_PLANE
                .union(EntityFilter::CONSTRUCTION_AXIS)
                .union(EntityFilter::CONSTRUCTION_POINT),
            EntityRef::InContext { target, .. } => return self.admits(target),
        };
        self.intersects(bit)
    }
}

impl std::ops::BitOr for EntityFilter {
    type Output = EntityFilter;
    fn bitor(self, rhs: EntityFilter) -> EntityFilter {
        self.union(rhs)
    }
}

/// Key of one dialog field, unique within a schema (`"profiles"`, `"distance"`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FieldKey(pub &'static str);

/// The dialog of one feature kind.
#[derive(Clone, Debug, PartialEq)]
pub struct DialogSchema {
    /// The feature kind (`"solid.extrude"`).
    pub kind: &'static str,
    /// Dialog title.
    pub title: String,
    /// Help topic id.
    pub help: Option<&'static str>,
    /// Fields in display order.
    pub fields: Vec<FieldSpec>,
}

impl DialogSchema {
    /// The field with `key`.
    pub fn field(&self, key: FieldKey) -> Option<&FieldSpec> {
        self.fields.iter().find(|f| f.key == key)
    }
}

/// One field of a dialog.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldSpec {
    /// Key (unique in the schema).
    pub key: FieldKey,
    /// Label.
    pub label: String,
    /// Tooltip.
    pub tooltip: Option<String>,
    /// Collapsible group the field is shown in (`None` = top level).
    pub group: Option<String>,
    /// Widget and value type.
    pub kind: FieldKind,
    /// Shown only while this holds.
    pub visible_if: Option<Condition>,
    /// Editable only while this holds.
    pub enabled_if: Option<Condition>,
    /// The viewport manipulator that drives the field.
    pub manipulator: Option<ManipKind>,
}

impl FieldSpec {
    /// A field with only key, label and kind set.
    pub fn new(key: &'static str, label: &str, kind: FieldKind) -> FieldSpec {
        FieldSpec {
            key: FieldKey(key),
            label: label.to_owned(),
            tooltip: None,
            group: None,
            kind,
            visible_if: None,
            enabled_if: None,
            manipulator: None,
        }
    }
}

/// Widget and value type of a field.
#[derive(Clone, Debug, PartialEq)]
pub enum FieldKind {
    /// Entity picks (`FieldValue::Selection`).
    Selection {
        /// Accepted kinds.
        filter: EntityFilter,
        /// Fewest picks for OK.
        min: u32,
        /// Most picks (`None` = any number).
        max: Option<u32>,
    },
    /// A parameter expression (`FieldValue::Expr`).
    Expr {
        /// Required dimension.
        dim: Dim,
        /// Initial text.
        default: &'static str,
    },
    /// One of several options (`FieldValue::Choice`).
    Choice {
        /// The options.
        options: Vec<ChoiceOption>,
        /// Dropdown or icon row.
        style: ChoiceStyle,
    },
    /// A check box (`FieldValue::Bool`).
    Bool,
    /// An integer (`FieldValue::Int`).
    Int {
        /// Smallest value.
        min: i64,
        /// Largest value.
        max: i64,
    },
    /// Free text (`FieldValue::Text`).
    Text,
    /// A flip-direction toggle (`FieldValue::Flip`).
    FlipDirection,
}

/// One option of a [`FieldKind::Choice`].
#[derive(Clone, Debug, PartialEq)]
pub struct ChoiceOption {
    /// Stable id stored in [`FieldValue::Choice`].
    pub id: &'static str,
    /// Label.
    pub label: String,
    /// Icon (SVG bytes).
    pub icon_svg: Option<&'static [u8]>,
}

impl ChoiceOption {
    /// An option without an icon.
    pub fn new(id: &'static str, label: &str) -> ChoiceOption {
        ChoiceOption {
            id,
            label: label.to_owned(),
            icon_svg: None,
        }
    }
}

/// How a choice is shown.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ChoiceStyle {
    /// A dropdown.
    Dropdown,
    /// A row of icon buttons.
    IconRow,
}

/// A predicate over the current field values.
#[derive(Clone, Debug, PartialEq)]
pub enum Condition {
    /// The field has this value.
    Is(FieldKey, FieldValue),
    /// The selection field is not empty.
    NonEmpty(FieldKey),
    /// Negation.
    Not(Box<Condition>),
    /// Conjunction.
    All(Vec<Condition>),
    /// Disjunction.
    Any(Vec<Condition>),
}

impl Condition {
    /// Evaluates the condition against `v` (a missing field is neither equal nor non-empty).
    pub fn holds(&self, v: &FieldValues) -> bool {
        match self {
            Condition::Is(k, x) => v.0.get(k) == Some(x),
            Condition::NonEmpty(k) => {
                matches!(v.0.get(k), Some(FieldValue::Selection(s)) if !s.is_empty())
            }
            Condition::Not(c) => !c.holds(v),
            Condition::All(cs) => cs.iter().all(|c| c.holds(v)),
            Condition::Any(cs) => cs.iter().any(|c| c.holds(v)),
        }
    }
}

/// The value of one field.
#[derive(Clone, Debug, PartialEq)]
pub enum FieldValue {
    /// Picked entities, in pick order.
    Selection(Vec<EntityRef>),
    /// Expression text; `param` is the bound model parameter in edit mode.
    Expr {
        /// The text.
        src: String,
        /// The model parameter the field is bound to.
        param: Option<ParamId>,
    },
    /// The chosen option id.
    Choice(&'static str),
    /// A check box.
    Bool(bool),
    /// An integer.
    Int(i64),
    /// Text.
    Text(String),
    /// Flip toggle.
    Flip(bool),
}

impl FieldValue {
    /// An unbound expression field.
    pub fn expr(src: &str) -> FieldValue {
        FieldValue::Expr {
            src: src.to_owned(),
            param: None,
        }
    }

    /// The field value of a feature's `Expr` (bound to its parameter, text from `design`).
    pub fn from_expr(e: &Expr, design: &Design) -> FieldValue {
        match e {
            Expr::Inline(s) => FieldValue::expr(s.as_str()),
            Expr::Param(id) => FieldValue::Expr {
                src: design
                    .params()
                    .get(*id)
                    .map(|p| p.expr.as_str().to_owned())
                    .unwrap_or_default(),
                param: Some(*id),
            },
        }
    }
}

/// Every field value of one dialog.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FieldValues(pub BTreeMap<FieldKey, FieldValue>);

impl FieldValues {
    /// Sets a field (builder style).
    #[must_use]
    pub fn with(mut self, key: &'static str, v: FieldValue) -> FieldValues {
        self.0.insert(FieldKey(key), v);
        self
    }

    /// The value of `key`.
    pub fn get(&self, key: &'static str) -> Option<&FieldValue> {
        self.0.get(&FieldKey(key))
    }

    /// A selection field (`[]` when missing).
    ///
    /// # Errors
    /// The field error when the value has another type.
    pub fn selection(&self, key: &'static str) -> Result<&[EntityRef], (FieldKey, String)> {
        match self.get(key) {
            None => Ok(&[]),
            Some(FieldValue::Selection(s)) => Ok(s),
            Some(_) => Err((FieldKey(key), "expected a selection".into())),
        }
    }

    /// An expression field as a feature `Expr`: the bound parameter, else the inline text.
    ///
    /// # Errors
    /// The field error when it is missing, empty or has another type.
    pub fn expr(&self, key: &'static str) -> Result<Expr, (FieldKey, String)> {
        match self.get(key) {
            Some(FieldValue::Expr { param: Some(p), .. }) => Ok(Expr::Param(*p)),
            Some(FieldValue::Expr { src, param: None }) if !src.trim().is_empty() => {
                Ok(Expr::Inline(ExprSrc(src.clone())))
            }
            Some(FieldValue::Expr { .. }) | None => {
                Err((FieldKey(key), "an expression is required".into()))
            }
            Some(_) => Err((FieldKey(key), "expected an expression".into())),
        }
    }

    /// A choice field (`default` when missing).
    ///
    /// # Errors
    /// The field error when the value has another type.
    pub fn choice(
        &self,
        key: &'static str,
        default: &'static str,
    ) -> Result<&'static str, (FieldKey, String)> {
        match self.get(key) {
            None => Ok(default),
            Some(FieldValue::Choice(c)) => Ok(c),
            Some(_) => Err((FieldKey(key), "expected a choice".into())),
        }
    }

    /// A flip or bool field (`false` when missing).
    ///
    /// # Errors
    /// The field error when the value has another type.
    pub fn flag(&self, key: &'static str) -> Result<bool, (FieldKey, String)> {
        match self.get(key) {
            None => Ok(false),
            Some(FieldValue::Flip(b) | FieldValue::Bool(b)) => Ok(*b),
            Some(_) => Err((FieldKey(key), "expected a toggle".into())),
        }
    }
}

/// Per-field problems found by [`DialogRegistration::build`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FieldErrors(pub Vec<(FieldKey, String)>);

impl FieldErrors {
    /// Whether there are none.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl From<(FieldKey, String)> for FieldErrors {
    fn from(e: (FieldKey, String)) -> FieldErrors {
        FieldErrors(vec![e])
    }
}

/// Kind of viewport manipulator.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ManipKind {
    /// Arrow along an axis.
    Distance,
    /// Arc about an axis.
    Angle,
    /// Move/rotate triad.
    Triad,
    /// Plane placement.
    Plane,
}

/// Where a manipulator of a field sits.
#[derive(Clone, Debug, PartialEq)]
pub struct ManipAnchor {
    /// The driven field.
    pub field: FieldKey,
    /// Anchor point (world, mm).
    pub origin: amg_core::Point3,
    /// Drag axis (distance) or rotation axis (angle).
    pub axis: amg_core::Dir3,
    /// Zero direction of an angle manipulator.
    pub ref_dir: Option<amg_core::Dir3>,
    /// Value shown at `origin`.
    pub base: f64,
}

/// What a dialog opens with.
#[derive(Clone, Copy, Debug)]
pub struct DialogInit<'a> {
    /// The selection when the command started.
    pub preselection: &'a [EntityRef],
    /// The design.
    pub design: &'a Design,
    /// Its evaluated state.
    pub evaluated: &'a Evaluated,
}

/// Command, toolbar and search data of a registration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandMeta {
    /// Workspace id (`"design"`).
    pub workspace: &'static str,
    /// Toolbar tab (`"solid"`).
    pub tab: &'static str,
    /// Toolbar group (`"create"`).
    pub group: &'static str,
    /// Order within the group.
    pub order: i32,
    /// Shown directly in the toolbar (not only in the dropdown).
    pub pinned: bool,
    /// Icon (SVG bytes; empty = generic icon).
    pub icon_svg: &'static [u8],
    /// Default shortcut (`"E"`).
    pub shortcut: Option<&'static str>,
    /// Extra search words.
    pub keywords: &'static [&'static str],
}

/// The dialog registration of one feature kind (submitted with `inventory::submit!`).
pub struct DialogRegistration {
    /// The feature kind.
    pub kind: &'static str,
    /// The schema (built once).
    pub schema: fn() -> &'static DialogSchema,
    /// Field values when the command starts, from the preselection.
    pub defaults: fn(&DialogInit<'_>) -> FieldValues,
    /// Field values of an existing feature (edit); `None` if it is not this kind.
    pub read: fn(&dyn Feature, &Design) -> Option<FieldValues>,
    /// The feature from the field values; `Some(old)` when editing.
    pub build: fn(&FieldValues, Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors>,
    /// Manipulator anchors of a computed feature.
    pub anchors: fn(&dyn Feature, &Design, &Evaluated) -> Vec<ManipAnchor>,
    /// Command placement.
    pub command: CommandMeta,
    /// S04 addition: `Some(f)` and `f(values) == Some(name)` means the user chose "New
    /// component": UI-05 commits `AddComponent { name }` and `AddFeature` with the new
    /// component as context in one transaction. `None` for every registration until SOL-24.
    pub new_component: Option<fn(&FieldValues) -> Option<String>>,
}

impl std::fmt::Debug for DialogRegistration {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DialogRegistration")
            .field("kind", &self.kind)
            .field("command", &self.command)
            .finish_non_exhaustive()
    }
}

inventory::collect!(DialogRegistration);

/// S04 addition (SOL-09): a command that starts another kind's dialog chosen from the
/// selection (press-pull). UI-05 renders it like a registration without a schema.
pub struct DispatchRegistration {
    /// Command id (`"solid.press_pull"`).
    pub command_id: &'static str,
    /// The feature kind whose dialog opens for this selection (`None` = disabled).
    pub dispatch: fn(&[EntityRef], &Design) -> Option<&'static str>,
    /// Command placement.
    pub command: CommandMeta,
}

impl std::fmt::Debug for DispatchRegistration {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DispatchRegistration")
            .field("command_id", &self.command_id)
            .field("command", &self.command)
            .finish_non_exhaustive()
    }
}

inventory::collect!(DispatchRegistration);

/// The dialog registration of `kind`.
pub fn registration(kind: &str) -> Option<&'static DialogRegistration> {
    registrations().find(|r| r.kind == kind)
}

/// Every dialog registration linked into the binary.
pub fn registrations() -> impl Iterator<Item = &'static DialogRegistration> {
    crate::solid::link();
    inventory::iter::<DialogRegistration>.into_iter()
}

/// Every dispatch registration linked into the binary.
pub fn dispatches() -> impl Iterator<Item = &'static DispatchRegistration> {
    crate::solid::link();
    inventory::iter::<DispatchRegistration>.into_iter()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn filter_bits_follow_the_s11_layout() {
        assert_eq!(EntityFilter::PROFILE.bits(), 256);
        assert_eq!(EntityFilter::PLANAR_FACE.bits(), 1 << 13);
        assert!(EntityFilter::ALL.contains(EntityFilter::OCCURRENCE | EntityFilter::BODY));
        assert!(!EntityFilter::BODY.intersects(EntityFilter::FACE));
    }

    #[test]
    fn conditions_evaluate() {
        let v = FieldValues::default()
            .with("op", FieldValue::Choice("cut"))
            .with("bodies", FieldValue::Selection(Vec::new()));
        let is_cut = Condition::Is(FieldKey("op"), FieldValue::Choice("cut"));
        assert!(is_cut.holds(&v));
        assert!(!Condition::NonEmpty(FieldKey("bodies")).holds(&v));
        assert!(
            Condition::All(vec![
                is_cut.clone(),
                Condition::Not(Box::new(Condition::NonEmpty(FieldKey("bodies"))))
            ])
            .holds(&v)
        );
        assert!(Condition::Any(vec![Condition::NonEmpty(FieldKey("x")), is_cut]).holds(&v));
    }

    #[test]
    fn expr_fields_prefer_the_bound_param() {
        let v = FieldValues::default()
            .with("a", FieldValue::expr("10 mm"))
            .with(
                "b",
                FieldValue::Expr {
                    src: "d1".into(),
                    param: Some(ParamId(7)),
                },
            )
            .with("c", FieldValue::expr("  "));
        assert_eq!(v.expr("a").unwrap(), Expr::Inline(ExprSrc("10 mm".into())));
        assert_eq!(v.expr("b").unwrap(), Expr::Param(ParamId(7)));
        assert!(v.expr("c").is_err());
        assert!(v.expr("missing").is_err());
        assert_eq!(v.choice("op", "new").unwrap(), "new");
        assert!(!v.flag("flip").unwrap());
    }
}
