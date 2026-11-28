//! Dialog registrations of `solid.pattern` and `solid.mirror` (S11 §B schema, rendered by
//! UI-05).
//!
//! S11 §B has no field kind that picks timeline features, so seed features are a text field
//! of feature ids (`"12, 15"`), and suppressed instances a text field of instance numbers;
//! UI-05 can replace both with timeline / viewport picks later (S11 request in the S04 log).
//! Directions, axes and the mirror plane are a choice of origin entity or "Selected …" with
//! a selection field shown for the latter.

use std::sync::OnceLock;

use amg_doc::{BodyId, Design, Dim, EntityRef, Evaluated, Feature, FeatureBox, FeatureId};

use super::{
    AngleSpan, ComputeType, Mirror, PathOrientation, Pattern, PatternDir, PatternKind,
    PatternObjects, Spacing,
};
use crate::construct::{OriginEntity, RefInput};
use crate::dialog::{
    ChoiceOption, ChoiceStyle, CommandMeta, Condition, DialogInit, DialogRegistration,
    DialogSchema, EntityFilter, FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue,
    FieldValues, ManipAnchor, ManipKind,
};
use crate::solid::common::BodyOp;

// ---- shared ---------------------------------------------------------------------------------

const OBJECT_TYPE: &str = "object_type";
const BODIES: &str = "bodies";
const FEATURES: &str = "features";
const FACES: &str = "faces";

const OBJECT_TYPES: [(&str, &str); 2] = [("bodies", "Bodies"), ("features", "Features")];
/// Patterns also take the faces of a boss or pocket (SOL-23); mirrors don't.
const PATTERN_OBJECT_TYPES: [(&str, &str); 3] = [
    ("bodies", "Bodies"),
    ("features", "Features"),
    ("faces", "Faces"),
];
pub(crate) const AXES: [(&str, &str); 4] = [
    ("x", "X axis"),
    ("y", "Y axis"),
    ("z", "Z axis"),
    ("picked", "Selected axis"),
];

fn choice(key: &'static str, label: &str, options: &[(&'static str, &str)]) -> FieldSpec {
    FieldSpec::new(
        key,
        label,
        FieldKind::Choice {
            options: options
                .iter()
                .map(|(id, l)| ChoiceOption::new(id, l))
                .collect(),
            style: ChoiceStyle::Dropdown,
        },
    )
}

fn is(key: &'static str, id: &'static str) -> Condition {
    Condition::Is(FieldKey(key), FieldValue::Choice(id))
}

fn flag_on(key: &'static str) -> Condition {
    Condition::Is(FieldKey(key), FieldValue::Bool(true))
}

fn expr_field(key: &'static str, label: &str, dim: Dim, default: &'static str) -> FieldSpec {
    FieldSpec::new(key, label, FieldKind::Expr { dim, default })
}

fn single_pick(key: &'static str, label: &str, filter: EntityFilter) -> FieldSpec {
    FieldSpec::new(
        key,
        label,
        FieldKind::Selection {
            filter,
            min: 1,
            max: Some(1),
        },
    )
}

/// The object fields: type, bodies (shown for bodies), feature ids (shown for features), and
/// with `faces` the picked faces (shown for faces).
fn object_fields(verb: &str, faces: bool) -> Vec<FieldSpec> {
    let types: &[(&'static str, &str)] = if faces {
        &PATTERN_OBJECT_TYPES
    } else {
        &OBJECT_TYPES
    };
    let ty = choice(OBJECT_TYPE, "Object type", types);
    let mut bodies = FieldSpec::new(
        BODIES,
        &format!("Bodies to {verb}"),
        FieldKind::Selection {
            filter: EntityFilter::BODY,
            min: 1,
            max: None,
        },
    );
    bodies.visible_if = Some(is(OBJECT_TYPE, "bodies"));
    let mut features = FieldSpec::new(FEATURES, &format!("Features to {verb}"), FieldKind::Text);
    features.visible_if = Some(is(OBJECT_TYPE, "features"));
    features.tooltip =
        Some("Timeline ids of extrudes and other tool features, e.g. `12, 15`.".into());
    let mut v = vec![ty, bodies, features];
    if faces {
        let mut f = FieldSpec::new(
            FACES,
            &format!("Faces to {verb}"),
            FieldKind::Selection {
                filter: EntityFilter::FACE,
                min: 1,
                max: None,
            },
        );
        f.visible_if = Some(is(OBJECT_TYPE, "faces"));
        f.tooltip = Some("The faces of a boss or a pocket on a planar face.".into());
        v.push(f);
    }
    v
}

fn preselected_bodies(init: &DialogInit<'_>) -> Vec<EntityRef> {
    init.preselection
        .iter()
        .filter(|e| matches!(e, EntityRef::Body(_)))
        .cloned()
        .collect()
}

fn ids_text<T: Copy>(ids: &[T], f: impl Fn(T) -> u64) -> String {
    ids.iter()
        .map(|i| f(*i).to_string())
        .collect::<Vec<_>>()
        .join(", ")
}

fn parse_ids(text: &str, key: &'static str) -> Result<Vec<u64>, (FieldKey, String)> {
    text.split(|c: char| c == ',' || c.is_whitespace())
        .filter(|s| !s.is_empty())
        .map(|s| {
            s.parse::<u64>()
                .map_err(|_| (FieldKey(key), format!("`{s}` is not a number")))
        })
        .collect()
}

fn text<'v>(v: &'v FieldValues, key: &'static str) -> Result<&'v str, (FieldKey, String)> {
    match v.get(key) {
        None => Ok(""),
        Some(FieldValue::Text(t)) => Ok(t),
        Some(_) => Err((FieldKey(key), "expected text".into())),
    }
}

/// Reads the object fields; errors go to `errs`.
fn read_objects(v: &FieldValues, errs: &mut FieldErrors) -> PatternObjects {
    match v.choice(OBJECT_TYPE, "bodies") {
        Ok("faces") => {
            let mut faces = Vec::new();
            match v.selection(FACES) {
                Ok(sel) => {
                    for e in sel {
                        match e {
                            EntityRef::Topo(t) => faces.push(t.clone()),
                            _ => errs.0.push((FieldKey(FACES), "select faces".into())),
                        }
                    }
                }
                Err(e) => errs.0.push(e),
            }
            PatternObjects::Faces(faces)
        }
        Ok("features") => {
            let ids = text(v, FEATURES).and_then(|t| parse_ids(t, FEATURES));
            match ids {
                Ok(ids) => PatternObjects::Features(ids.into_iter().map(FeatureId).collect()),
                Err(e) => {
                    errs.0.push(e);
                    PatternObjects::Features(Vec::new())
                }
            }
        }
        Ok(_) => {
            let mut bodies = Vec::new();
            match v.selection(BODIES) {
                Ok(sel) => {
                    for e in sel {
                        match e {
                            EntityRef::Body(b) => bodies.push(*b),
                            _ => errs.0.push((FieldKey(BODIES), "select bodies".into())),
                        }
                    }
                }
                Err(e) => errs.0.push(e),
            }
            PatternObjects::Bodies(bodies)
        }
        Err(e) => {
            errs.0.push(e);
            PatternObjects::Bodies(Vec::new())
        }
    }
}

/// The object field values; `faces` adds the face selection (patterns only).
fn object_values(o: &PatternObjects, faces: bool) -> FieldValues {
    let mut picked: Vec<EntityRef> = Vec::new();
    let (ty, bodies, features): (&'static str, Vec<EntityRef>, String) = match o {
        PatternObjects::Features(f) => ("features", Vec::new(), ids_text(f, |x: FeatureId| x.0)),
        PatternObjects::Bodies(b) => (
            "bodies",
            b.iter().map(|b: &BodyId| EntityRef::Body(*b)).collect(),
            String::new(),
        ),
        PatternObjects::Faces(t) if faces => {
            picked = t.iter().cloned().map(EntityRef::Topo).collect();
            ("faces", Vec::new(), String::new())
        }
        // Occurrences (SOL-24) have no dialog fields yet; mirrors have no face fields.
        PatternObjects::Faces(_) | PatternObjects::Occurrences(_) => {
            ("bodies", Vec::new(), String::new())
        }
    };
    let v = FieldValues::default()
        .with(OBJECT_TYPE, FieldValue::Choice(ty))
        .with(BODIES, FieldValue::Selection(bodies))
        .with(FEATURES, FieldValue::Text(features));
    if faces {
        v.with(FACES, FieldValue::Selection(picked))
    } else {
        v
    }
}

/// An axis choice plus its selection field: `RefInput`.
pub(crate) fn read_axis(
    v: &FieldValues,
    choice_key: &'static str,
    ref_key: &'static str,
    default: &'static str,
) -> Result<RefInput, (FieldKey, String)> {
    match v.choice(choice_key, default)? {
        "x" => Ok(RefInput::Origin(OriginEntity::XAxis)),
        "y" => Ok(RefInput::Origin(OriginEntity::YAxis)),
        "z" => Ok(RefInput::Origin(OriginEntity::ZAxis)),
        "xy" => Ok(RefInput::Origin(OriginEntity::XYPlane)),
        "yz" => Ok(RefInput::Origin(OriginEntity::YZPlane)),
        "xz" => Ok(RefInput::Origin(OriginEntity::XZPlane)),
        "picked" => v
            .selection(ref_key)?
            .first()
            .cloned()
            .map(RefInput::Ref)
            .ok_or_else(|| (FieldKey(ref_key), "select an entity".into())),
        other => Err((FieldKey(choice_key), format!("unknown option `{other}`"))),
    }
}

/// The choice id and selection of a `RefInput` (`default` and none for an origin entity the
/// choice doesn't list).
pub(crate) fn axis_values(r: &RefInput, default: &'static str) -> (&'static str, Vec<EntityRef>) {
    match r {
        RefInput::Origin(OriginEntity::XAxis) => ("x", Vec::new()),
        RefInput::Origin(OriginEntity::YAxis) => ("y", Vec::new()),
        RefInput::Origin(OriginEntity::ZAxis) => ("z", Vec::new()),
        RefInput::Origin(OriginEntity::XYPlane) => ("xy", Vec::new()),
        RefInput::Origin(OriginEntity::YZPlane) => ("yz", Vec::new()),
        RefInput::Origin(OriginEntity::XZPlane) => ("xz", Vec::new()),
        RefInput::Origin(OriginEntity::Point) => (default, Vec::new()),
        RefInput::Ref(e) => ("picked", vec![e.clone()]),
    }
}

fn push<T>(r: Result<T, (FieldKey, String)>, errs: &mut FieldErrors) -> Option<T> {
    match r {
        Ok(x) => Some(x),
        Err(e) => {
            errs.0.push(e);
            None
        }
    }
}

// ---- pattern --------------------------------------------------------------------------------

const PATTERN_TYPE: &str = "pattern_type";
const SUPPRESSED: &str = "suppressed";
const AXIS: &str = "axis";
const AXIS_REF: &str = "axis_ref";
const COUNT: &str = "count";
const SPAN: &str = "span";
const ANGLE: &str = "angle";
const SYMMETRIC: &str = "symmetric";
const D2_ON: &str = "d2_enabled";
const PATH: &str = "path";
const PATH_COUNT: &str = "path_count";
const PATH_SPACING: &str = "path_spacing_type";
const PATH_DISTANCE: &str = "path_distance";
const PATH_START: &str = "path_start";
const PATH_ORIENTATION: &str = "path_orientation";
const PATH_FLIP: &str = "path_flip";
const COMPUTE: &str = "compute";

const PATTERN_TYPES: [(&str, &str); 3] = [
    ("rectangular", "Rectangular"),
    ("circular", "Circular"),
    ("path", "Path"),
];
const ORIENTATIONS: [(&str, &str); 2] = [
    ("identical", "Identical"),
    ("path_direction", "Path direction"),
];
const COMPUTE_TYPES: [(&str, &str); 2] = [("identical", "Identical"), ("adjust", "Adjust")];
const DEFAULT_PATH_COUNT: &str = "3";
const DEFAULT_PATH_START: &str = "0";
const SPACING_TYPES: [(&str, &str); 2] = [("spacing", "Spacing"), ("extent", "Extent")];
const SPANS: [(&str, &str); 3] = [
    ("full", "Full turn"),
    ("extent", "Partial"),
    ("spacing", "Angle between"),
];

const DEFAULT_COUNT: &str = "2";
const DEFAULT_DISTANCE: &str = "10 mm";
const DEFAULT_CIRCULAR_COUNT: &str = "3";
const DEFAULT_ANGLE: &str = "90 deg";

/// The field keys of one rectangular direction.
struct DirKeys {
    axis: &'static str,
    axis_ref: &'static str,
    count: &'static str,
    spacing: &'static str,
    distance: &'static str,
    flip: &'static str,
    symmetric: &'static str,
    default_axis: &'static str,
    n: u8,
}

const D1: DirKeys = DirKeys {
    axis: "d1_axis",
    axis_ref: "d1_ref",
    count: "d1_count",
    spacing: "d1_spacing_type",
    distance: "d1_distance",
    flip: "d1_flip",
    symmetric: "d1_symmetric",
    default_axis: "x",
    n: 1,
};
const D2: DirKeys = DirKeys {
    axis: "d2_axis",
    axis_ref: "d2_ref",
    count: "d2_count",
    spacing: "d2_spacing_type",
    distance: "d2_distance",
    flip: "d2_flip",
    symmetric: "d2_symmetric",
    default_axis: "y",
    n: 2,
};

fn dir_fields(d: &DirKeys, shown: &Condition) -> Vec<FieldSpec> {
    let n = d.n;
    let mut axis = choice(d.axis, &format!("Direction {n}"), &AXES);
    axis.visible_if = Some(shown.clone());
    let mut r = single_pick(
        d.axis_ref,
        &format!("Direction {n} reference"),
        EntityFilter::EDGE
            | EntityFilter::CONSTRUCTION_AXIS
            | EntityFilter::SKETCH_CURVE
            | EntityFilter::FACE,
    );
    r.visible_if = Some(Condition::All(vec![shown.clone(), is(d.axis, "picked")]));
    let mut count = expr_field(d.count, &format!("Quantity {n}"), Dim::NONE, DEFAULT_COUNT);
    count.visible_if = Some(shown.clone());
    let mut spacing = choice(d.spacing, &format!("Distance type {n}"), &SPACING_TYPES);
    spacing.visible_if = Some(shown.clone());
    let mut distance = expr_field(
        d.distance,
        &format!("Distance {n}"),
        Dim::LENGTH,
        DEFAULT_DISTANCE,
    );
    distance.manipulator = Some(ManipKind::Distance);
    distance.visible_if = Some(shown.clone());
    let mut flip = FieldSpec::new(
        d.flip,
        &format!("Flip direction {n}"),
        FieldKind::FlipDirection,
    );
    flip.visible_if = Some(shown.clone());
    let mut sym = FieldSpec::new(d.symmetric, &format!("Symmetric {n}"), FieldKind::Bool);
    sym.visible_if = Some(shown.clone());
    vec![axis, r, count, spacing, distance, flip, sym]
}

/// The path-pattern fields (SOL-23), shown when `shown` holds.
fn path_fields(shown: &Condition) -> Vec<FieldSpec> {
    let mut path = FieldSpec::new(
        PATH,
        "Path curves",
        FieldKind::Selection {
            filter: EntityFilter::EDGE | EntityFilter::SKETCH_CURVE,
            min: 1,
            max: None,
        },
    );
    path.tooltip = Some("Edges or sketch curves, connected end to end.".into());
    let count = expr_field(
        PATH_COUNT,
        "Quantity along path",
        Dim::NONE,
        DEFAULT_PATH_COUNT,
    );
    let spacing = choice(PATH_SPACING, "Path distance type", &SPACING_TYPES);
    let mut distance = expr_field(
        PATH_DISTANCE,
        "Path distance",
        Dim::LENGTH,
        DEFAULT_DISTANCE,
    );
    distance.manipulator = Some(ManipKind::Distance);
    let mut start = expr_field(PATH_START, "Start position", Dim::NONE, DEFAULT_PATH_START);
    start.tooltip = Some("Where the original sits, as a fraction of the path (0 to 1).".into());
    let orientation = choice(PATH_ORIENTATION, "Orientation", &ORIENTATIONS);
    let flip = FieldSpec::new(PATH_FLIP, "Flip path", FieldKind::FlipDirection);
    let mut v = vec![path, count, spacing, distance, start, orientation, flip];
    for f in &mut v {
        f.visible_if = Some(shown.clone());
    }
    v
}

fn path_defaults(v: FieldValues) -> FieldValues {
    v.with(PATH, FieldValue::Selection(Vec::new()))
        .with(PATH_COUNT, FieldValue::expr(DEFAULT_PATH_COUNT))
        .with(PATH_SPACING, FieldValue::Choice("spacing"))
        .with(PATH_DISTANCE, FieldValue::expr(DEFAULT_DISTANCE))
        .with(PATH_START, FieldValue::expr(DEFAULT_PATH_START))
        .with(PATH_ORIENTATION, FieldValue::Choice("identical"))
        .with(PATH_FLIP, FieldValue::Flip(false))
}

fn compute_value(c: ComputeType) -> FieldValue {
    FieldValue::Choice(match c {
        ComputeType::Identical => "identical",
        ComputeType::Adjust => "adjust",
    })
}

fn read_path(v: &FieldValues, errs: &mut FieldErrors) -> Option<PatternKind> {
    let path = push(v.selection(PATH), errs).map(<[EntityRef]>::to_vec);
    let count = push(v.expr(PATH_COUNT), errs);
    let dist = push(v.expr(PATH_DISTANCE), errs);
    let spacing = match push(v.choice(PATH_SPACING, "spacing"), errs) {
        Some("extent") => dist.map(Spacing::Extent),
        Some(_) => dist.map(Spacing::Spacing),
        None => None,
    };
    let start = push(v.expr(PATH_START), errs);
    let orientation = match push(v.choice(PATH_ORIENTATION, "identical"), errs) {
        Some("path_direction") => Some(PathOrientation::PathDirection),
        Some(_) => Some(PathOrientation::Identical),
        None => None,
    };
    let flip = push(v.flag(PATH_FLIP), errs);
    Some(PatternKind::Path {
        path: path?,
        count: count?,
        spacing: spacing?,
        start: start?,
        orientation: orientation?,
        flip: flip?,
    })
}

fn pattern_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let rect = is(PATTERN_TYPE, "rectangular");
        let circ = is(PATTERN_TYPE, "circular");
        let mut fields = vec![choice(PATTERN_TYPE, "Pattern type", &PATTERN_TYPES)];
        fields.extend(object_fields("pattern", true));
        fields.extend(dir_fields(&D1, &rect));
        let mut d2_on = FieldSpec::new(D2_ON, "Second direction", FieldKind::Bool);
        d2_on.visible_if = Some(rect.clone());
        fields.push(d2_on);
        fields.extend(dir_fields(
            &D2,
            &Condition::All(vec![rect.clone(), flag_on(D2_ON)]),
        ));
        let mut axis = choice(AXIS, "Axis", &AXES);
        axis.visible_if = Some(circ.clone());
        let mut axis_ref = single_pick(
            AXIS_REF,
            "Axis reference",
            EntityFilter::EDGE
                | EntityFilter::CONSTRUCTION_AXIS
                | EntityFilter::SKETCH_CURVE
                | EntityFilter::FACE,
        );
        axis_ref.visible_if = Some(Condition::All(vec![circ.clone(), is(AXIS, "picked")]));
        let mut count = expr_field(COUNT, "Quantity", Dim::NONE, DEFAULT_CIRCULAR_COUNT);
        count.visible_if = Some(circ.clone());
        let mut span = choice(SPAN, "Angle type", &SPANS);
        span.visible_if = Some(circ.clone());
        let partial = Condition::All(vec![
            circ.clone(),
            Condition::Not(Box::new(is(SPAN, "full"))),
        ]);
        let mut angle = expr_field(ANGLE, "Angle", Dim::ANGLE, DEFAULT_ANGLE);
        angle.manipulator = Some(ManipKind::Angle);
        angle.visible_if = Some(partial.clone());
        let mut sym = FieldSpec::new(SYMMETRIC, "Symmetric", FieldKind::Bool);
        sym.visible_if = Some(partial);
        fields.extend([axis, axis_ref, count, span, angle, sym]);
        fields.extend(path_fields(&is(PATTERN_TYPE, "path")));
        let mut compute = choice(COMPUTE, "Compute type", &COMPUTE_TYPES);
        compute.visible_if = Some(is(OBJECT_TYPE, "features"));
        compute.tooltip = Some(
            "Identical moves the features' result; Adjust rebuilds each instance, so extents \
             to objects are recomputed per instance."
                .into(),
        );
        fields.push(compute);
        let mut suppressed = FieldSpec::new(SUPPRESSED, "Suppressed instances", FieldKind::Text);
        suppressed.tooltip = Some(
            "Instance numbers to leave out: i + 65536 j for rectangular, the position for \
             circular and path patterns."
                .into(),
        );
        fields.push(suppressed);
        DialogSchema {
            kind: "solid.pattern",
            title: "Pattern".into(),
            help: Some("solid.pattern"),
            fields,
        }
    })
}

fn dir_defaults(mut v: FieldValues, d: &DirKeys) -> FieldValues {
    v = v
        .with(d.axis, FieldValue::Choice(d.default_axis))
        .with(d.axis_ref, FieldValue::Selection(Vec::new()))
        .with(d.count, FieldValue::expr(DEFAULT_COUNT))
        .with(d.spacing, FieldValue::Choice("spacing"))
        .with(d.distance, FieldValue::expr(DEFAULT_DISTANCE))
        .with(d.flip, FieldValue::Flip(false))
        .with(d.symmetric, FieldValue::Bool(false));
    v
}

fn circular_defaults(v: FieldValues) -> FieldValues {
    v.with(AXIS, FieldValue::Choice("z"))
        .with(AXIS_REF, FieldValue::Selection(Vec::new()))
        .with(COUNT, FieldValue::expr(DEFAULT_CIRCULAR_COUNT))
        .with(SPAN, FieldValue::Choice("full"))
        .with(ANGLE, FieldValue::expr(DEFAULT_ANGLE))
        .with(SYMMETRIC, FieldValue::Bool(false))
}

fn merge(mut a: FieldValues, b: FieldValues) -> FieldValues {
    a.0.extend(b.0);
    a
}

fn pattern_defaults(init: &DialogInit<'_>) -> FieldValues {
    let mut v = FieldValues::default().with(PATTERN_TYPE, FieldValue::Choice("rectangular"));
    v = merge(v, object_values(&PatternObjects::Bodies(Vec::new()), true));
    v = v.with(BODIES, FieldValue::Selection(preselected_bodies(init)));
    v = dir_defaults(v, &D1);
    v = v.with(D2_ON, FieldValue::Bool(false));
    v = dir_defaults(v, &D2);
    v = circular_defaults(v);
    v = path_defaults(v);
    v.with(COMPUTE, compute_value(ComputeType::Identical))
        .with(SUPPRESSED, FieldValue::Text(String::new()))
}

fn dir_values(mut v: FieldValues, d: &DirKeys, p: &PatternDir, design: &Design) -> FieldValues {
    let (axis, sel) = axis_values(&p.dir, d.default_axis);
    let (ty, e) = match &p.spacing {
        Spacing::Spacing(e) => ("spacing", e),
        Spacing::Extent(e) => ("extent", e),
    };
    v = v
        .with(d.axis, FieldValue::Choice(axis))
        .with(d.axis_ref, FieldValue::Selection(sel))
        .with(d.count, FieldValue::from_expr(&p.count, design))
        .with(d.spacing, FieldValue::Choice(ty))
        .with(d.distance, FieldValue::from_expr(e, design))
        .with(d.flip, FieldValue::Flip(p.flip))
        .with(d.symmetric, FieldValue::Bool(p.symmetric));
    v
}

fn pattern_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    let f = f.as_any().downcast_ref::<Pattern>()?;
    let mut v = FieldValues::default();
    v = merge(v, object_values(&f.objects, true));
    v = dir_defaults(v, &D1);
    v = v.with(D2_ON, FieldValue::Bool(false));
    v = dir_defaults(v, &D2);
    v = circular_defaults(v);
    v = path_defaults(v);
    v = v.with(COMPUTE, compute_value(f.compute));
    match &f.kind {
        PatternKind::Rectangular { d1, d2 } => {
            v = v.with(PATTERN_TYPE, FieldValue::Choice("rectangular"));
            v = dir_values(v, &D1, d1, design);
            if let Some(d2) = d2 {
                v = v.with(D2_ON, FieldValue::Bool(true));
                v = dir_values(v, &D2, d2, design);
            }
        }
        PatternKind::Circular {
            axis,
            count,
            span,
            symmetric,
        } => {
            let (a, sel) = axis_values(axis, "z");
            v = v
                .with(PATTERN_TYPE, FieldValue::Choice("circular"))
                .with(AXIS, FieldValue::Choice(a))
                .with(AXIS_REF, FieldValue::Selection(sel))
                .with(COUNT, FieldValue::from_expr(count, design))
                .with(SYMMETRIC, FieldValue::Bool(*symmetric));
            match span {
                AngleSpan::Full => v = v.with(SPAN, FieldValue::Choice("full")),
                AngleSpan::Extent(e) => {
                    v = v
                        .with(SPAN, FieldValue::Choice("extent"))
                        .with(ANGLE, FieldValue::from_expr(e, design));
                }
                AngleSpan::Spacing(e) => {
                    v = v
                        .with(SPAN, FieldValue::Choice("spacing"))
                        .with(ANGLE, FieldValue::from_expr(e, design));
                }
            }
        }
        PatternKind::Path {
            path,
            count,
            spacing,
            start,
            orientation,
            flip,
        } => {
            let (ty, e) = match spacing {
                Spacing::Spacing(e) => ("spacing", e),
                Spacing::Extent(e) => ("extent", e),
            };
            let o = match orientation {
                PathOrientation::Identical => "identical",
                PathOrientation::PathDirection => "path_direction",
            };
            v = v
                .with(PATTERN_TYPE, FieldValue::Choice("path"))
                .with(PATH, FieldValue::Selection(path.clone()))
                .with(PATH_COUNT, FieldValue::from_expr(count, design))
                .with(PATH_SPACING, FieldValue::Choice(ty))
                .with(PATH_DISTANCE, FieldValue::from_expr(e, design))
                .with(PATH_START, FieldValue::from_expr(start, design))
                .with(PATH_ORIENTATION, FieldValue::Choice(o))
                .with(PATH_FLIP, FieldValue::Flip(*flip));
        }
    }
    Some(v.with(
        SUPPRESSED,
        FieldValue::Text(ids_text(&f.suppressed, u64::from)),
    ))
}

fn read_dir(v: &FieldValues, d: &DirKeys, errs: &mut FieldErrors) -> Option<PatternDir> {
    let dir = push(read_axis(v, d.axis, d.axis_ref, d.default_axis), errs);
    let count = push(v.expr(d.count), errs);
    let dist = push(v.expr(d.distance), errs);
    let spacing = match push(v.choice(d.spacing, "spacing"), errs)? {
        "extent" => dist.map(Spacing::Extent),
        _ => dist.map(Spacing::Spacing),
    };
    let flip = push(v.flag(d.flip), errs);
    let symmetric = push(v.flag(d.symmetric), errs);
    Some(PatternDir {
        dir: dir?,
        count: count?,
        spacing: spacing?,
        flip: flip?,
        symmetric: symmetric?,
    })
}

fn pattern_build(v: &FieldValues, old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let old = old.and_then(|o| o.as_any().downcast_ref::<Pattern>());
    let objects = read_objects(v, &mut errs);
    let kind = match push(v.choice(PATTERN_TYPE, "rectangular"), &mut errs) {
        Some("path") => read_path(v, &mut errs),
        Some("circular") => {
            let axis = push(read_axis(v, AXIS, AXIS_REF, "z"), &mut errs);
            let count = push(v.expr(COUNT), &mut errs);
            let span = match push(v.choice(SPAN, "full"), &mut errs) {
                Some("extent") => push(v.expr(ANGLE), &mut errs).map(AngleSpan::Extent),
                Some("spacing") => push(v.expr(ANGLE), &mut errs).map(AngleSpan::Spacing),
                Some(_) => Some(AngleSpan::Full),
                None => None,
            };
            let symmetric = push(v.flag(SYMMETRIC), &mut errs);
            match (axis, count, span, symmetric) {
                (Some(axis), Some(count), Some(span), Some(symmetric)) => {
                    Some(PatternKind::Circular {
                        axis,
                        count,
                        span,
                        symmetric,
                    })
                }
                _ => None,
            }
        }
        Some(_) => {
            let d1 = read_dir(v, &D1, &mut errs);
            let d2 = match push(v.flag(D2_ON), &mut errs) {
                Some(true) => read_dir(v, &D2, &mut errs).map(Some),
                Some(false) => Some(None),
                None => None,
            };
            match (d1, d2) {
                (Some(d1), Some(d2)) => Some(PatternKind::Rectangular { d1, d2 }),
                _ => None,
            }
        }
        None => None,
    };
    let suppressed = text(v, SUPPRESSED)
        .and_then(|t| parse_ids(t, SUPPRESSED))
        .and_then(|ids| {
            ids.into_iter()
                .map(|k| {
                    u32::try_from(k)
                        .map_err(|_| (FieldKey(SUPPRESSED), format!("{k} is out of range")))
                })
                .collect::<Result<Vec<u32>, _>>()
        });
    let suppressed = push(suppressed, &mut errs);
    let compute = match push(v.choice(COMPUTE, "identical"), &mut errs) {
        Some("adjust") => ComputeType::Adjust,
        _ => ComputeType::Identical,
    };
    match (kind, suppressed) {
        (Some(kind), Some(suppressed)) if errs.is_empty() => {
            let has_target = matches!(
                objects,
                PatternObjects::Features(_) | PatternObjects::Faces(_)
            );
            Ok(FeatureBox::new(Pattern {
                objects,
                kind,
                suppressed,
                compute,
                target: old.and_then(|o| o.target.clone()).filter(|_| has_target),
            }))
        }
        _ => Err(errs),
    }
}

fn no_anchors(_f: &dyn Feature, _design: &Design, _eval: &Evaluated) -> Vec<ManipAnchor> {
    Vec::new()
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.pattern",
        schema: pattern_schema,
        defaults: pattern_defaults,
        read: pattern_read,
        build: pattern_build,
        anchors: no_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "create",
            order: 60,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["pattern", "array", "rectangular", "circular", "polar", "copy"],
        },
        new_component: None,
    }
}

// ---- mirror ---------------------------------------------------------------------------------

const PLANE: &str = "plane";
const PLANE_REF: &str = "plane_ref";
const OPERATION: &str = "operation";

const PLANES: [(&str, &str); 4] = [
    ("xy", "XY plane"),
    ("yz", "YZ plane"),
    ("xz", "XZ plane"),
    ("picked", "Selected plane"),
];
const OPERATIONS: [(&str, &str); 2] = [("new_body", "New body"), ("join", "Join")];

fn mirror_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut fields: Vec<FieldSpec> = object_fields("mirror", false);
        fields.push(choice(PLANE, "Mirror plane", &PLANES));
        let mut r = single_pick(
            PLANE_REF,
            "Plane reference",
            EntityFilter::FACE | EntityFilter::CONSTRUCTION_PLANE,
        );
        r.visible_if = Some(is(PLANE, "picked"));
        fields.push(r);
        let mut op = choice(OPERATION, "Operation", &OPERATIONS);
        op.visible_if = Some(is(OBJECT_TYPE, "bodies"));
        fields.push(op);
        DialogSchema {
            kind: "solid.mirror",
            title: "Mirror".into(),
            help: Some("solid.mirror"),
            fields,
        }
    })
}

fn mirror_defaults(init: &DialogInit<'_>) -> FieldValues {
    merge(
        object_values(&PatternObjects::Bodies(Vec::new()), false),
        FieldValues::default()
            .with(BODIES, FieldValue::Selection(preselected_bodies(init)))
            .with(PLANE, FieldValue::Choice("yz"))
            .with(PLANE_REF, FieldValue::Selection(Vec::new()))
            .with(OPERATION, FieldValue::Choice("new_body")),
    )
}

fn mirror_read(f: &dyn Feature, _design: &Design) -> Option<FieldValues> {
    let f = f.as_any().downcast_ref::<Mirror>()?;
    let (plane, sel) = axis_values(&f.plane, "yz");
    let op = if f.op == BodyOp::Join {
        "join"
    } else {
        "new_body"
    };
    Some(merge(
        object_values(&f.objects, false),
        FieldValues::default()
            .with(PLANE, FieldValue::Choice(plane))
            .with(PLANE_REF, FieldValue::Selection(sel))
            .with(OPERATION, FieldValue::Choice(op)),
    ))
}

fn mirror_build(v: &FieldValues, old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let old = old.and_then(|o| o.as_any().downcast_ref::<Mirror>());
    let objects = read_objects(v, &mut errs);
    let plane = push(read_axis(v, PLANE, PLANE_REF, "yz"), &mut errs);
    let op = match push(v.choice(OPERATION, "new_body"), &mut errs) {
        Some("join") => BodyOp::Join,
        _ => BodyOp::NewBody,
    };
    match plane {
        Some(plane) if errs.is_empty() => {
            let is_features = matches!(objects, PatternObjects::Features(_));
            Ok(FeatureBox::new(Mirror {
                objects,
                plane,
                op: if is_features { BodyOp::NewBody } else { op },
                target: old.and_then(|o| o.target.clone()).filter(|_| is_features),
            }))
        }
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.mirror",
        schema: mirror_schema,
        defaults: mirror_defaults,
        read: mirror_read,
        build: mirror_build,
        anchors: no_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "create",
            order: 61,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["mirror", "reflect", "symmetry", "copy"],
        },
        new_component: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialog::registration;

    fn labels(s: &DialogSchema) -> Vec<String> {
        s.fields.iter().map(|f| f.label.clone()).collect()
    }

    #[test]
    fn labels_are_unique_and_differ_from_option_labels() {
        for s in [pattern_schema(), mirror_schema()] {
            let mut l = labels(s);
            let n = l.len();
            l.sort();
            l.dedup();
            assert_eq!(l.len(), n, "{}: duplicate field label", s.kind);
            for f in &s.fields {
                if let FieldKind::Choice { options, .. } = &f.kind {
                    for o in options {
                        assert!(
                            !l.contains(&o.label),
                            "{}: option `{}` repeats a field label",
                            s.kind,
                            o.label
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn ids_parse_with_commas_and_spaces() {
        assert_eq!(parse_ids("12, 15 3", FEATURES).unwrap(), vec![12, 15, 3]);
        assert!(parse_ids("", FEATURES).unwrap().is_empty());
        assert!(parse_ids("x", FEATURES).is_err());
    }

    #[test]
    fn both_kinds_are_registered() {
        assert!(registration("solid.pattern").is_some());
        assert!(registration("solid.mirror").is_some());
    }
}
