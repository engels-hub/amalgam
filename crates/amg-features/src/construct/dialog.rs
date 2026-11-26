//! Dialog schemas of the construction features (CON-04; UI-05 renders them).
//!
//! One [`DialogRegistration`] per feature kind (`construct.plane`, `construct.axis`,
//! `construct.point`). Its "Type" field chooses one of the 20 variants ([`VARIANTS`]); only
//! that variant's inputs are shown. A menu command opens the dialog on one variant with
//! [`variant_values`].
//!
//! Every reference input is two fields: where it comes from (picked geometry, or an origin
//! entity of the component, which is not an `EntityRef` and so cannot be picked) and, for
//! picked geometry, the selection field.

use std::sync::OnceLock;

use amg_core::Dir3;
use amg_doc::{
    ConstructionGeom, Design, Dim, EntityRef, Evaluated, Expr, ExprSrc, Feature, FeatureBox,
    TopoKind,
};

use super::resolve::resolve_in;
use super::{
    AxisDef, ConstructAxis, ConstructPlane, ConstructPoint, OriginEntity, PathPosition, PlaneDef,
    PointDef, RefInput, math,
};
use crate::dialog::{
    ChoiceOption, ChoiceStyle, CommandMeta, Condition, DialogInit, DialogRegistration,
    DialogSchema, EntityFilter, FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue,
    FieldValues, ManipAnchor, ManipKind,
};

/// Feature kind of construction planes.
pub const PLANE_KIND: &str = "construct.plane";
/// Feature kind of construction axes.
pub const AXIS_KIND: &str = "construct.axis";
/// Feature kind of construction points.
pub const POINT_KIND: &str = "construct.point";

/// Key of the variant chooser.
pub const TYPE: &str = "type";
/// "Picked geometry" in an input's source chooser.
pub const PICK: &str = "pick";
/// "Default" in the optional zero plane's source chooser.
pub const DEFAULT: &str = "default";

/// Offset distance (offset plane).
pub const OFFSET_DISTANCE: &str = "offset_distance";
/// Angle (plane at angle).
pub const AT_ANGLE_ANGLE: &str = "at_angle_angle";
/// Angle (tangent plane).
pub const TANGENT_ANGLE: &str = "tangent_angle";
/// "Fraction" or "distance" (along path).
pub const POSITION_BY: &str = "position_by";
/// Path position as a fraction of the length.
pub const FRACTION: &str = "fraction";
/// Path position as a length from the start.
pub const PATH_DISTANCE: &str = "path_distance";

const DEFAULT_OFFSET: &str = "10 mm";
const DEFAULT_AT_ANGLE: &str = "45 deg";
const DEFAULT_TANGENT_ANGLE: &str = "0 deg";
const DEFAULT_FRACTION: &str = "0.5";
const DEFAULT_PATH_DISTANCE: &str = "10 mm";

/// What a reference input accepts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Accepts {
    /// A plane: construction plane, planar face or face group, origin plane.
    Plane,
    /// A straight line: construction axis, edge, sketch line, origin axis.
    Line,
    /// A point: construction point, vertex, sketch point, origin point.
    Point,
    /// A plane or a point (midplane).
    PlaneOrPoint,
    /// A B-rep face.
    Face,
    /// A revolved face or a circular edge or sketch curve.
    Revolved,
    /// Something with a centre: circle edge or curve, sphere or torus face.
    Center,
    /// An edge or sketch curve (an infinite line is not a path).
    Path,
}

impl Accepts {
    /// The kinds the selection field takes.
    pub fn filter(self) -> EntityFilter {
        use EntityFilter as F;
        let plane = F::CONSTRUCTION_PLANE | F::FACE | F::PLANAR_FACE | F::MESH_FACE_GROUP;
        let point = F::CONSTRUCTION_POINT | F::VERTEX | F::SKETCH_POINT;
        match self {
            Accepts::Plane => plane,
            Accepts::Line => F::CONSTRUCTION_AXIS | F::EDGE | F::SKETCH_CURVE,
            Accepts::Point => point,
            Accepts::PlaneOrPoint => plane | point,
            Accepts::Face => F::FACE,
            Accepts::Revolved | Accepts::Center => F::FACE | F::EDGE | F::SKETCH_CURVE,
            Accepts::Path => F::EDGE | F::SKETCH_CURVE,
        }
    }

    /// The origin entities that can stand for the input.
    pub fn origins(self) -> &'static [OriginEntity] {
        use OriginEntity as O;
        match self {
            Accepts::Plane => &[O::XYPlane, O::XZPlane, O::YZPlane],
            Accepts::Line => &[O::XAxis, O::YAxis, O::ZAxis],
            Accepts::Point => &[O::Point],
            Accepts::PlaneOrPoint => &[O::XYPlane, O::XZPlane, O::YZPlane, O::Point],
            Accepts::Face | Accepts::Revolved | Accepts::Center | Accepts::Path => &[],
        }
    }

    /// Whether preselected entity `e` can fill the input.
    pub fn fits(self, e: &EntityRef, eval: &Evaluated) -> bool {
        use Accepts as A;
        match e {
            EntityRef::Construction(c) => match eval.construction(*c) {
                Some(ConstructionGeom::Plane(_)) => matches!(self, A::Plane | A::PlaneOrPoint),
                Some(ConstructionGeom::Axis(_)) => matches!(self, A::Line),
                Some(ConstructionGeom::Point(_)) => matches!(self, A::Point | A::PlaneOrPoint),
                None => false,
            },
            EntityRef::Topo(t) => match t.kind {
                TopoKind::Face => matches!(
                    self,
                    A::Plane | A::PlaneOrPoint | A::Face | A::Revolved | A::Center
                ),
                TopoKind::FaceGroup => matches!(self, A::Plane | A::PlaneOrPoint),
                TopoKind::Edge => {
                    matches!(self, A::Line | A::Revolved | A::Center | A::Path)
                }
                TopoKind::Vertex => matches!(self, A::Point | A::PlaneOrPoint),
            },
            EntityRef::SketchCurve { .. } => {
                matches!(self, A::Line | A::Revolved | A::Center | A::Path)
            }
            EntityRef::SketchPoint { .. } => matches!(self, A::Point | A::PlaneOrPoint),
            EntityRef::InContext { target, .. } => self.fits(target, eval),
            EntityRef::Body(_) | EntityRef::Profile { .. } => false,
        }
    }
}

/// The id of an origin entity in a source chooser.
pub fn origin_id(o: OriginEntity) -> &'static str {
    match o {
        OriginEntity::Point => "origin_point",
        OriginEntity::XAxis => "origin_x",
        OriginEntity::YAxis => "origin_y",
        OriginEntity::ZAxis => "origin_z",
        OriginEntity::XYPlane => "origin_xy",
        OriginEntity::YZPlane => "origin_yz",
        OriginEntity::XZPlane => "origin_xz",
    }
}

fn origin_label(o: OriginEntity) -> &'static str {
    match o {
        OriginEntity::Point => "Origin point",
        OriginEntity::XAxis => "Origin X axis",
        OriginEntity::YAxis => "Origin Y axis",
        OriginEntity::ZAxis => "Origin Z axis",
        OriginEntity::XYPlane => "Origin XY plane",
        OriginEntity::YZPlane => "Origin YZ plane",
        OriginEntity::XZPlane => "Origin XZ plane",
    }
}

/// One reference input of a variant.
#[derive(Clone, Copy, Debug)]
pub struct Input {
    /// Key of the selection field.
    pub key: &'static str,
    /// Key of the source chooser.
    pub source: &'static str,
    /// Label of the selection field (the chooser adds " from").
    pub label: &'static str,
    /// What it takes.
    pub accepts: Accepts,
    /// May be left at its default (the zero plane of "at angle").
    pub optional: bool,
}

const fn input(
    key: &'static str,
    source: &'static str,
    label: &'static str,
    accepts: Accepts,
) -> Input {
    Input {
        key,
        source,
        label,
        accepts,
        optional: false,
    }
}

/// One of the 20 construction variants.
#[derive(Clone, Copy, Debug)]
pub struct Variant {
    /// The feature kind.
    pub kind: &'static str,
    /// Id in the "Type" chooser.
    pub id: &'static str,
    /// Menu and chooser label (Fusion's names).
    pub label: &'static str,
    /// Reference inputs in field order.
    pub inputs: &'static [Input],
}

macro_rules! variant {
    ($kind:ident, $id:literal, $label:literal, [$($input:expr),+ $(,)?]) => {
        Variant {
            kind: $kind,
            id: $id,
            label: $label,
            inputs: &[$($input),+],
        }
    };
}

/// Every variant, in menu order: 8 planes, 6 axes, 6 points.
pub const VARIANTS: [Variant; 20] = [
    variant!(
        PLANE_KIND,
        "offset",
        "Offset Plane",
        [input(
            "offset_base",
            "offset_base_from",
            "Plane",
            Accepts::Plane
        )]
    ),
    variant!(
        PLANE_KIND,
        "at_angle",
        "Plane at Angle",
        [
            input("at_angle_line", "at_angle_line_from", "Line", Accepts::Line),
            Input {
                key: "at_angle_zero",
                source: "at_angle_zero_from",
                label: "Zero plane",
                accepts: Accepts::Plane,
                optional: true,
            },
        ]
    ),
    variant!(
        PLANE_KIND,
        "tangent",
        "Tangent Plane",
        [
            input("tangent_face", "tangent_face_from", "Face", Accepts::Face),
            input(
                "tangent_reference",
                "tangent_reference_from",
                "Reference plane",
                Accepts::Plane
            ),
        ]
    ),
    variant!(
        PLANE_KIND,
        "midplane",
        "Midplane",
        [
            input("mid_a", "mid_a_from", "First side", Accepts::PlaneOrPoint),
            input("mid_b", "mid_b_from", "Second side", Accepts::PlaneOrPoint),
        ]
    ),
    variant!(
        PLANE_KIND,
        "two_edges",
        "Plane Through Two Edges",
        [
            input("edges_a", "edges_a_from", "First edge", Accepts::Line),
            input("edges_b", "edges_b_from", "Second edge", Accepts::Line),
        ]
    ),
    variant!(
        PLANE_KIND,
        "three_points",
        "Plane Through Three Points",
        [
            input("points_a", "points_a_from", "First point", Accepts::Point),
            input("points_b", "points_b_from", "Second point", Accepts::Point),
            input("points_c", "points_c_from", "Third point", Accepts::Point),
        ]
    ),
    variant!(
        PLANE_KIND,
        "tangent_at_point",
        "Plane Tangent to Face at Point",
        [
            input("tap_face", "tap_face_from", "Face", Accepts::Face),
            input("tap_point", "tap_point_from", "Point", Accepts::Point),
        ]
    ),
    variant!(
        PLANE_KIND,
        "along_path",
        "Plane Along Path",
        [input(
            "along_path",
            "along_path_from",
            "Path",
            Accepts::Path
        )]
    ),
    variant!(
        AXIS_KIND,
        "revolved",
        "Axis Through Cylinder, Cone or Torus",
        [input(
            "rev_entity",
            "rev_entity_from",
            "Face or circle",
            Accepts::Revolved
        )]
    ),
    variant!(
        AXIS_KIND,
        "perpendicular_at_point",
        "Axis Perpendicular at Point",
        [
            input("perp_face", "perp_face_from", "Face", Accepts::Face),
            input("perp_point", "perp_point_from", "Point", Accepts::Point),
        ]
    ),
    variant!(
        AXIS_KIND,
        "two_planes",
        "Axis Through Two Planes",
        [
            input("planes_a", "planes_a_from", "First plane", Accepts::Plane),
            input("planes_b", "planes_b_from", "Second plane", Accepts::Plane),
        ]
    ),
    variant!(
        AXIS_KIND,
        "two_points",
        "Axis Through Two Points",
        [
            input("line_a", "line_a_from", "Start point", Accepts::Point),
            input("line_b", "line_b_from", "End point", Accepts::Point),
        ]
    ),
    variant!(
        AXIS_KIND,
        "edge",
        "Axis Through Edge",
        [input("axis_edge", "axis_edge_from", "Edge", Accepts::Line)]
    ),
    variant!(
        AXIS_KIND,
        "perpendicular_to_face",
        "Axis Perpendicular to Face at Point",
        [
            input("perpf_face", "perpf_face_from", "Face", Accepts::Face),
            input("perpf_point", "perpf_point_from", "Point", Accepts::Point),
        ]
    ),
    variant!(
        POINT_KIND,
        "vertex",
        "Point at Vertex",
        [input("vertex", "vertex_from", "Point", Accepts::Point)]
    ),
    variant!(
        POINT_KIND,
        "two_edges",
        "Point Through Two Edges",
        [
            input("cross_a", "cross_a_from", "First edge", Accepts::Line),
            input("cross_b", "cross_b_from", "Second edge", Accepts::Line),
        ]
    ),
    variant!(
        POINT_KIND,
        "three_planes",
        "Point Through Three Planes",
        [
            input("three_a", "three_a_from", "First plane", Accepts::Plane),
            input("three_b", "three_b_from", "Second plane", Accepts::Plane),
            input("three_c", "three_c_from", "Third plane", Accepts::Plane),
        ]
    ),
    variant!(
        POINT_KIND,
        "center",
        "Point at Center of Circle, Sphere or Torus",
        [input(
            "center_entity",
            "center_entity_from",
            "Circle or face",
            Accepts::Center
        )]
    ),
    variant!(
        POINT_KIND,
        "edge_plane",
        "Point at Edge and Plane",
        [
            // Curved edges and sketch curves too, and the origin axes.
            input("ep_edge", "ep_edge_from", "Edge", Accepts::Line),
            input("ep_plane", "ep_plane_from", "Plane", Accepts::Plane),
        ]
    ),
    variant!(
        POINT_KIND,
        "along_path",
        "Point Along Path",
        [input(
            "point_path",
            "point_path_from",
            "Path",
            Accepts::Path
        )]
    ),
];

/// The variants of one feature kind.
pub fn variants_of(kind: &str) -> impl Iterator<Item = &'static Variant> {
    VARIANTS.iter().filter(move |v| v.kind == kind)
}

/// The variant `id` of `kind`.
pub fn variant(kind: &str, id: &str) -> Option<&'static Variant> {
    VARIANTS.iter().find(|v| v.kind == kind && v.id == id)
}

/// The variant a registration starts on when no command chose one.
fn default_variant(kind: &str) -> &'static str {
    match kind {
        PLANE_KIND => "offset",
        AXIS_KIND => "edge",
        _ => "vertex",
    }
}

/// What an input without a preselected entity starts with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unfilled {
    /// "Picked geometry" with an empty selection (menu commands: the user picks next).
    Pick,
    /// The first origin entity that fits, so the dialog is complete (a generic dialog opened
    /// without a variant).
    Origin,
}

// ---- schemas --------------------------------------------------------------------------------

fn is(key: &'static str, v: &'static str) -> Condition {
    Condition::Is(FieldKey(key), FieldValue::Choice(v))
}

fn choice(key: &'static str, label: &str, options: Vec<ChoiceOption>) -> FieldSpec {
    FieldSpec::new(
        key,
        label,
        FieldKind::Choice {
            options,
            style: ChoiceStyle::Dropdown,
        },
    )
}

fn expr(key: &'static str, label: &str, dim: Dim, default: &'static str) -> FieldSpec {
    FieldSpec::new(key, label, FieldKind::Expr { dim, default })
}

/// The source chooser (if there is anything to choose) and the selection field of one input of
/// variant `id`.
fn input_fields(id: &'static str, i: &Input) -> Vec<FieldSpec> {
    let chooses = i.optional || !i.accepts.origins().is_empty();
    let mut fields = Vec::new();
    if chooses {
        let mut options = Vec::new();
        if i.optional {
            options.push(ChoiceOption::new(DEFAULT, "Default"));
        }
        options.push(ChoiceOption::new(PICK, "Picked geometry"));
        for &o in i.accepts.origins() {
            options.push(ChoiceOption::new(origin_id(o), origin_label(o)));
        }
        let mut source = choice(i.source, &format!("{} from", i.label), options);
        source.visible_if = Some(is(TYPE, id));
        fields.push(source);
    }
    let mut pick = FieldSpec::new(
        i.key,
        i.label,
        FieldKind::Selection {
            filter: i.accepts.filter(),
            min: 1,
            max: Some(1),
        },
    );
    pick.visible_if = Some(if chooses {
        Condition::All(vec![is(TYPE, id), is(i.source, PICK)])
    } else {
        is(TYPE, id)
    });
    fields.push(pick);
    fields
}

fn position_fields(id: &'static str) -> [FieldSpec; 3] {
    let mut by = choice(
        POSITION_BY,
        "Position by",
        vec![
            ChoiceOption::new(FRACTION, "Fraction of length"),
            ChoiceOption::new(PATH_DISTANCE, "Length from start"),
        ],
    );
    by.visible_if = Some(is(TYPE, id));
    let mut fraction = expr(FRACTION, "Fraction", Dim::NONE, DEFAULT_FRACTION);
    fraction.tooltip = Some("0 is the start of the path, 1 its end.".into());
    fraction.visible_if = Some(Condition::All(vec![
        is(TYPE, id),
        is(POSITION_BY, FRACTION),
    ]));
    let mut distance = expr(
        PATH_DISTANCE,
        "Distance",
        Dim::LENGTH,
        DEFAULT_PATH_DISTANCE,
    );
    distance.visible_if = Some(Condition::All(vec![
        is(TYPE, id),
        is(POSITION_BY, PATH_DISTANCE),
    ]));
    [by, fraction, distance]
}

fn make_schema(kind: &'static str, title: &str) -> DialogSchema {
    let types = variants_of(kind)
        .map(|v| ChoiceOption::new(v.id, v.label))
        .collect();
    let mut fields = vec![choice(TYPE, "Type", types)];
    for v in variants_of(kind) {
        for i in v.inputs {
            fields.extend(input_fields(v.id, i));
        }
        let angle = |key, default| {
            let mut f = expr(key, "Angle", Dim::ANGLE, default);
            f.visible_if = Some(is(TYPE, v.id));
            f
        };
        match (v.kind, v.id) {
            (PLANE_KIND, "offset") => {
                let mut d = expr(OFFSET_DISTANCE, "Distance", Dim::LENGTH, DEFAULT_OFFSET);
                d.visible_if = Some(is(TYPE, v.id));
                d.manipulator = Some(ManipKind::Distance);
                fields.push(d);
            }
            (PLANE_KIND, "at_angle") => {
                let mut a = angle(AT_ANGLE_ANGLE, DEFAULT_AT_ANGLE);
                a.manipulator = Some(ManipKind::Angle);
                fields.push(a);
            }
            (PLANE_KIND, "tangent") => {
                fields.push(angle(TANGENT_ANGLE, DEFAULT_TANGENT_ANGLE));
            }
            (_, "along_path") => fields.extend(position_fields(v.id)),
            _ => {}
        }
    }
    DialogSchema {
        kind,
        title: title.into(),
        help: Some(kind),
        fields,
    }
}

fn plane_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| make_schema(PLANE_KIND, "Construction Plane"))
}

fn axis_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| make_schema(AXIS_KIND, "Construction Axis"))
}

fn point_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| make_schema(POINT_KIND, "Construction Point"))
}

// ---- values ---------------------------------------------------------------------------------

/// Variant `id` with every chooser and expression of its kind at the default.
fn base_values(kind: &str, id: &'static str) -> FieldValues {
    let mut v = FieldValues::default().with(TYPE, FieldValue::Choice(id));
    for i in variants_of(kind).flat_map(|var| var.inputs) {
        let first = if i.optional { DEFAULT } else { PICK };
        v = v.with(i.source, FieldValue::Choice(first));
    }
    match kind {
        PLANE_KIND => v
            .with(OFFSET_DISTANCE, FieldValue::expr(DEFAULT_OFFSET))
            .with(AT_ANGLE_ANGLE, FieldValue::expr(DEFAULT_AT_ANGLE))
            .with(TANGENT_ANGLE, FieldValue::expr(DEFAULT_TANGENT_ANGLE))
            .with(POSITION_BY, FieldValue::Choice(FRACTION))
            .with(FRACTION, FieldValue::expr(DEFAULT_FRACTION))
            .with(PATH_DISTANCE, FieldValue::expr(DEFAULT_PATH_DISTANCE)),
        POINT_KIND => v
            .with(POSITION_BY, FieldValue::Choice(FRACTION))
            .with(FRACTION, FieldValue::expr(DEFAULT_FRACTION))
            .with(PATH_DISTANCE, FieldValue::expr(DEFAULT_PATH_DISTANCE)),
        _ => v,
    }
}

/// Writes input `i` of a definition (`None`: the optional input is unset).
fn put(v: FieldValues, i: &Input, r: Option<&RefInput>) -> FieldValues {
    match r {
        Some(RefInput::Origin(o)) => v.with(i.source, FieldValue::Choice(origin_id(*o))),
        Some(RefInput::Ref(e)) => v
            .with(i.source, FieldValue::Choice(PICK))
            .with(i.key, FieldValue::Selection(vec![e.clone()])),
        None => v.with(i.source, FieldValue::Choice(DEFAULT)),
    }
}

/// The field values that open the dialog of `kind` on variant `id`: preselected entities fill
/// the inputs they fit, in field order; the others start as `unfilled` says.
pub fn variant_values(
    kind: &str,
    id: &str,
    init: &DialogInit<'_>,
    unfilled: Unfilled,
) -> Option<FieldValues> {
    let var = variant(kind, id)?;
    let mut v = base_values(kind, var.id);
    let mut free: Vec<&EntityRef> = init.preselection.iter().collect();
    for i in var.inputs {
        let found = free
            .iter()
            .position(|e| i.accepts.fits(e, init.evaluated))
            .map(|k| free.remove(k).clone());
        v = match found {
            Some(e) => put(v, i, Some(&RefInput::Ref(e))),
            None if i.optional => put(v, i, None),
            None => match (unfilled, i.accepts.origins().first()) {
                (Unfilled::Origin, Some(&o)) => put(v, i, Some(&RefInput::Origin(o))),
                _ => v.with(i.source, FieldValue::Choice(PICK)),
            },
        };
    }
    Some(v)
}

fn defaults_for(kind: &str, init: &DialogInit<'_>) -> FieldValues {
    variant_values(kind, default_variant(kind), init, Unfilled::Origin).unwrap_or_default()
}

fn plane_defaults(init: &DialogInit<'_>) -> FieldValues {
    defaults_for(PLANE_KIND, init)
}

fn axis_defaults(init: &DialogInit<'_>) -> FieldValues {
    defaults_for(AXIS_KIND, init)
}

fn point_defaults(init: &DialogInit<'_>) -> FieldValues {
    defaults_for(POINT_KIND, init)
}

/// The reference inputs of a definition, in the order of its variant's inputs (`None`: the
/// optional input is unset).
fn parts(v: &mut FieldValues, id: &'static str, kind: &str, refs: &[Option<&RefInput>]) {
    let Some(var) = variant(kind, id) else {
        return;
    };
    let mut out = std::mem::take(v);
    for (i, r) in var.inputs.iter().zip(refs) {
        out = put(out, i, *r);
    }
    *v = out;
}

fn position(v: FieldValues, at: &PathPosition, design: &Design) -> FieldValues {
    let (by, e) = match at {
        PathPosition::Fraction(e) => (FRACTION, e),
        PathPosition::Distance(e) => (PATH_DISTANCE, e),
    };
    v.with(POSITION_BY, FieldValue::Choice(by))
        .with(by, FieldValue::from_expr(e, design))
}

fn plane_values(f: &ConstructPlane, design: &Design) -> FieldValues {
    use PlaneDef as P;
    let (id, refs): (&'static str, Vec<Option<&RefInput>>) = match &f.def {
        P::Offset { base, .. } => ("offset", vec![Some(base)]),
        P::AtAngle { line, zero, .. } => ("at_angle", vec![Some(line), zero.as_ref()]),
        P::Tangent {
            face, reference, ..
        } => ("tangent", vec![Some(face), Some(reference)]),
        P::Midplane { a, b } => ("midplane", vec![Some(a), Some(b)]),
        P::TwoEdges { a, b } => ("two_edges", vec![Some(a), Some(b)]),
        P::ThreePoints { a, b, c } => ("three_points", vec![Some(a), Some(b), Some(c)]),
        P::TangentAtPoint { face, point } => ("tangent_at_point", vec![Some(face), Some(point)]),
        P::AlongPath { path, .. } => ("along_path", vec![Some(path)]),
    };
    let mut v = base_values(PLANE_KIND, id);
    parts(&mut v, id, PLANE_KIND, &refs);
    match &f.def {
        P::Offset { distance, .. } => v.with(OFFSET_DISTANCE, from(distance, design)),
        P::AtAngle { angle, .. } => v.with(AT_ANGLE_ANGLE, from(angle, design)),
        P::Tangent { angle, .. } => v.with(TANGENT_ANGLE, from(angle, design)),
        P::AlongPath { at, .. } => position(v, at, design),
        _ => v,
    }
}

fn from(e: &Expr, design: &Design) -> FieldValue {
    FieldValue::from_expr(e, design)
}

fn axis_values(f: &ConstructAxis) -> FieldValues {
    use AxisDef as A;
    let (id, refs): (&'static str, Vec<Option<&RefInput>>) = match &f.def {
        A::ThroughRevolved { face_or_edge } => ("revolved", vec![Some(face_or_edge)]),
        A::PerpendicularAtPoint { face, point } => {
            ("perpendicular_at_point", vec![Some(face), Some(point)])
        }
        A::ThroughTwoPlanes { a, b } => ("two_planes", vec![Some(a), Some(b)]),
        A::ThroughTwoPoints { a, b } => ("two_points", vec![Some(a), Some(b)]),
        A::ThroughEdge { edge } => ("edge", vec![Some(edge)]),
        A::PerpendicularToFaceAtPoint { face, point } => {
            ("perpendicular_to_face", vec![Some(face), Some(point)])
        }
    };
    let mut v = base_values(AXIS_KIND, id);
    parts(&mut v, id, AXIS_KIND, &refs);
    v
}

fn point_values(f: &ConstructPoint, design: &Design) -> FieldValues {
    use PointDef as P;
    let (id, refs): (&'static str, Vec<Option<&RefInput>>) = match &f.def {
        P::AtVertex { point } => ("vertex", vec![Some(point)]),
        P::ThroughTwoEdges { a, b } => ("two_edges", vec![Some(a), Some(b)]),
        P::ThroughThreePlanes { a, b, c } => ("three_planes", vec![Some(a), Some(b), Some(c)]),
        P::AtCenter { entity } => ("center", vec![Some(entity)]),
        P::EdgeAndPlane { edge, plane } => ("edge_plane", vec![Some(edge), Some(plane)]),
        P::AlongPath { path, .. } => ("along_path", vec![Some(path)]),
    };
    let mut v = base_values(POINT_KIND, id);
    parts(&mut v, id, POINT_KIND, &refs);
    match &f.def {
        P::AlongPath { at, .. } => position(v, at, design),
        _ => v,
    }
}

fn read_plane(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    Some(plane_values(
        f.as_any().downcast_ref::<ConstructPlane>()?,
        design,
    ))
}

fn read_axis(f: &dyn Feature, _design: &Design) -> Option<FieldValues> {
    Some(axis_values(f.as_any().downcast_ref::<ConstructAxis>()?))
}

fn read_point(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    Some(point_values(
        f.as_any().downcast_ref::<ConstructPoint>()?,
        design,
    ))
}

// ---- build ----------------------------------------------------------------------------------

/// Reads the inputs and expressions of one variant, collecting field errors.
struct Reader<'a> {
    v: &'a FieldValues,
    var: &'static Variant,
    errors: FieldErrors,
}

/// Stands in for an input that could not be read; the errors keep it from being used.
const MISSING: RefInput = RefInput::Origin(OriginEntity::Point);

impl Reader<'_> {
    /// Input `n` of the variant; `None` for an optional input left at its default.
    fn optional(&mut self, n: usize) -> Option<RefInput> {
        let i = &self.var.inputs[n];
        // An input without origin entities has no chooser to read.
        let source = if i.accepts.origins().is_empty() && !i.optional {
            PICK
        } else {
            let first = if i.optional { DEFAULT } else { PICK };
            self.v.choice(i.source, first).unwrap_or_else(|e| {
                self.errors.0.push(e);
                first
            })
        };
        if source == DEFAULT && i.optional {
            return None;
        }
        if let Some(&o) = i
            .accepts
            .origins()
            .iter()
            .find(|o| origin_id(**o) == source)
        {
            return Some(RefInput::Origin(o));
        }
        if source != PICK {
            self.errors
                .0
                .push((FieldKey(i.source), "not a valid choice".into()));
            return Some(MISSING);
        }
        match self.v.selection(i.key) {
            Ok([e]) => Some(RefInput::Ref(e.clone())),
            Ok(_) => {
                self.errors
                    .0
                    .push((FieldKey(i.key), "select one object".into()));
                Some(MISSING)
            }
            Err(e) => {
                self.errors.0.push(e);
                Some(MISSING)
            }
        }
    }

    fn input(&mut self, n: usize) -> RefInput {
        self.optional(n).unwrap_or(MISSING)
    }

    fn expr(&mut self, key: &'static str) -> Expr {
        self.v.expr(key).unwrap_or_else(|e| {
            self.errors.0.push(e);
            Expr::Inline(ExprSrc(String::new()))
        })
    }

    fn position(&mut self) -> PathPosition {
        let by = self.v.choice(POSITION_BY, FRACTION).unwrap_or_else(|e| {
            self.errors.0.push(e);
            FRACTION
        });
        if by == PATH_DISTANCE {
            PathPosition::Distance(self.expr(PATH_DISTANCE))
        } else {
            PathPosition::Fraction(self.expr(FRACTION))
        }
    }

    fn finish(self, f: impl Feature) -> Result<FeatureBox, FieldErrors> {
        if self.errors.is_empty() {
            Ok(FeatureBox::new(f))
        } else {
            Err(self.errors)
        }
    }
}

fn reader<'a>(kind: &str, v: &'a FieldValues) -> Result<Reader<'a>, FieldErrors> {
    let id = v.choice(TYPE, default_variant(kind))?;
    let var = variant(kind, id)
        .ok_or_else(|| FieldErrors(vec![(FieldKey(TYPE), "unknown type".into())]))?;
    Ok(Reader {
        v,
        var,
        errors: FieldErrors::default(),
    })
}

fn build_plane(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    use PlaneDef as P;
    let mut r = reader(PLANE_KIND, v)?;
    let def = match r.var.id {
        "offset" => P::Offset {
            base: r.input(0),
            distance: r.expr(OFFSET_DISTANCE),
        },
        "at_angle" => P::AtAngle {
            line: r.input(0),
            zero: r.optional(1),
            angle: r.expr(AT_ANGLE_ANGLE),
        },
        "tangent" => P::Tangent {
            face: r.input(0),
            reference: r.input(1),
            angle: r.expr(TANGENT_ANGLE),
        },
        "midplane" => P::Midplane {
            a: r.input(0),
            b: r.input(1),
        },
        "two_edges" => P::TwoEdges {
            a: r.input(0),
            b: r.input(1),
        },
        "three_points" => P::ThreePoints {
            a: r.input(0),
            b: r.input(1),
            c: r.input(2),
        },
        "tangent_at_point" => P::TangentAtPoint {
            face: r.input(0),
            point: r.input(1),
        },
        _ => P::AlongPath {
            path: r.input(0),
            at: r.position(),
        },
    };
    r.finish(ConstructPlane { def })
}

fn build_axis(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    use AxisDef as A;
    let mut r = reader(AXIS_KIND, v)?;
    let def = match r.var.id {
        "revolved" => A::ThroughRevolved {
            face_or_edge: r.input(0),
        },
        "perpendicular_at_point" => A::PerpendicularAtPoint {
            face: r.input(0),
            point: r.input(1),
        },
        "two_planes" => A::ThroughTwoPlanes {
            a: r.input(0),
            b: r.input(1),
        },
        "two_points" => A::ThroughTwoPoints {
            a: r.input(0),
            b: r.input(1),
        },
        "edge" => A::ThroughEdge { edge: r.input(0) },
        _ => A::PerpendicularToFaceAtPoint {
            face: r.input(0),
            point: r.input(1),
        },
    };
    r.finish(ConstructAxis { def })
}

fn build_point(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    use PointDef as P;
    let mut r = reader(POINT_KIND, v)?;
    let def = match r.var.id {
        "vertex" => P::AtVertex { point: r.input(0) },
        "two_edges" => P::ThroughTwoEdges {
            a: r.input(0),
            b: r.input(1),
        },
        "three_planes" => P::ThroughThreePlanes {
            a: r.input(0),
            b: r.input(1),
            c: r.input(2),
        },
        "center" => P::AtCenter { entity: r.input(0) },
        "edge_plane" => P::EdgeAndPlane {
            edge: r.input(0),
            plane: r.input(1),
        },
        _ => P::AlongPath {
            path: r.input(0),
            at: r.position(),
        },
    };
    r.finish(ConstructPoint { def })
}

// ---- manipulators ---------------------------------------------------------------------------

/// Resolves an input that needs no geometry query (origin entities and construction
/// geometry): `anchors` runs on the UI thread.
fn cheap(r: &RefInput, design: &Design, eval: &Evaluated) -> Option<super::ResolvedRef> {
    match r {
        RefInput::Origin(_) | RefInput::Ref(EntityRef::Construction(_)) => {
            resolve_in(eval, design, r).ok()
        }
        RefInput::Ref(_) => None,
    }
}

/// The offset arrow (at the base plane, along its normal) and the angle ring (about the line).
/// Bases that need a geometry query (faces, edges) have no manipulator.
fn plane_anchors(f: &dyn Feature, design: &Design, eval: &Evaluated) -> Vec<ManipAnchor> {
    let Some(p) = f.as_any().downcast_ref::<ConstructPlane>() else {
        return Vec::new();
    };
    let anchor = match &p.def {
        PlaneDef::Offset { base, .. } => cheap(base, design, eval)
            .and_then(|r| math::plane_of(&r.geom).ok())
            .map(|plane| ManipAnchor {
                field: FieldKey(OFFSET_DISTANCE),
                origin: plane.origin,
                axis: plane.normal,
                ref_dir: None,
                base: 0.0,
            }),
        PlaneDef::AtAngle { line, zero, .. } => {
            let tol = design.meta().tolerance;
            let zero = zero
                .as_ref()
                .and_then(|z| cheap(z, design, eval))
                .and_then(|r| math::plane_of(&r.geom).ok());
            cheap(line, design, eval).and_then(|r| {
                let l = math::line_of(&r.geom, &tol).ok()?;
                let at_zero = math::plane_at_angle(&l, zero.as_ref(), 0.0, &tol).ok()?;
                Some(ManipAnchor {
                    field: FieldKey(AT_ANGLE_ANGLE),
                    origin: l.point,
                    axis: l.dir,
                    ref_dir: Some(Dir3::new_unchecked(at_zero.y.get())),
                    base: 0.0,
                })
            })
        }
        _ => None,
    };
    anchor.into_iter().collect()
}

fn no_anchors(_f: &dyn Feature, _design: &Design, _eval: &Evaluated) -> Vec<ManipAnchor> {
    Vec::new()
}

const fn meta(order: i32, keywords: &'static [&'static str]) -> CommandMeta {
    CommandMeta {
        workspace: "design",
        tab: "solid",
        group: "construct",
        order,
        pinned: false,
        icon_svg: &[],
        shortcut: None,
        keywords,
    }
}

inventory::submit! {
    DialogRegistration {
        kind: PLANE_KIND,
        schema: plane_schema,
        defaults: plane_defaults,
        read: read_plane,
        build: build_plane,
        anchors: plane_anchors,
        command: meta(200, &["construct", "plane", "datum", "work plane"]),
        new_component: None,
    }
}

inventory::submit! {
    DialogRegistration {
        kind: AXIS_KIND,
        schema: axis_schema,
        defaults: axis_defaults,
        read: read_axis,
        build: build_axis,
        anchors: no_anchors,
        command: meta(201, &["construct", "axis", "datum", "work axis"]),
        new_component: None,
    }
}

inventory::submit! {
    DialogRegistration {
        kind: POINT_KIND,
        schema: point_schema,
        defaults: point_defaults,
        read: read_point,
        build: build_point,
        anchors: no_anchors,
        command: meta(202, &["construct", "point", "datum", "work point"]),
        new_component: None,
    }
}
