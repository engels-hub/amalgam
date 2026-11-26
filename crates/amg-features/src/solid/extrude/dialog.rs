//! The `solid.extrude` dialog (UI-05 renders it from the schema): profiles, start, direction,
//! extent and taper per side, flip, operation. Conditional fields are hidden for the default
//! values (one side, from the profile plane, by a distance), so the default dialog shows the
//! SOL-01 fields plus the start, direction, extent and taper choosers.

use std::sync::OnceLock;

use amg_core::{Dir3, Point3};
use amg_doc::{BodyId, Design, Dim, EntityRef, Evaluated, Expr, Feature, FeatureBox};

use super::Extrude;
use crate::dialog::{
    ChoiceOption, ChoiceStyle, CommandMeta, Condition, DialogInit, DialogRegistration,
    DialogSchema, EntityFilter, FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue,
    FieldValues, ManipAnchor, ManipKind,
};
use crate::solid::common::{BodyOp, OpTarget};
use crate::solid::extent::{Direction, Extent, ExtentSide, Start};

const PROFILES: &str = "profiles";
const START: &str = "start";
const START_OFFSET: &str = "start_offset";
const START_OBJECT: &str = "start_object";
const DIRECTION: &str = "direction";
const WHOLE_LENGTH: &str = "whole_length";
const EXTENT: &str = "extent";
const DISTANCE: &str = "distance";
const OBJECT: &str = "object";
const OBJECT_OFFSET: &str = "object_offset";
const EXTEND: &str = "extend";
const TAPER: &str = "taper";
const EXTENT2: &str = "side2_extent";
const DISTANCE2: &str = "side2_distance";
const OBJECT2: &str = "side2_object";
const OBJECT_OFFSET2: &str = "side2_object_offset";
const EXTEND2: &str = "side2_extend";
const TAPER2: &str = "side2_taper";
const FLIP: &str = "flip";
const OPERATION: &str = "operation";
const BODIES: &str = "bodies";

pub(crate) const OPS: [(&str, BodyOp, &str); 4] = [
    ("new_body", BodyOp::NewBody, "New body"),
    ("join", BodyOp::Join, "Join"),
    ("cut", BodyOp::Cut, "Cut"),
    ("intersect", BodyOp::Intersect, "Intersect"),
];

// Option labels differ from every field label (the UI harness finds fields by label).
const STARTS: [(&str, &str); 3] = [
    ("profile", "Profile plane"),
    ("offset", "Offset plane"),
    ("object", "From object"),
];
const DIRECTIONS: [(&str, &str); 3] = [
    ("one_side", "One side"),
    ("two_sides", "Two sides"),
    ("symmetric", "Symmetric"),
];
const EXTENTS: [(&str, &str); 3] = [
    ("distance", "Fixed distance"),
    ("to_object", "To object"),
    ("all", "All"),
];

const DEFAULT_DISTANCE: &str = "10 mm";
const ZERO_MM: &str = "0 mm";
const ZERO_DEG: &str = "0 deg";

pub(crate) fn is(key: &'static str, v: &'static str) -> Condition {
    Condition::Is(FieldKey(key), FieldValue::Choice(v))
}

pub(crate) fn not(c: Condition) -> Condition {
    Condition::Not(Box::new(c))
}

pub(crate) fn choice(key: &'static str, label: &str, opts: &[(&'static str, &str)]) -> FieldSpec {
    FieldSpec::new(
        key,
        label,
        FieldKind::Choice {
            options: opts
                .iter()
                .map(|(id, label)| ChoiceOption::new(id, label))
                .collect(),
            style: ChoiceStyle::Dropdown,
        },
    )
}

pub(crate) fn expr(key: &'static str, label: &str, dim: Dim, default: &'static str) -> FieldSpec {
    FieldSpec::new(key, label, FieldKind::Expr { dim, default })
}

pub(crate) fn target_filter() -> EntityFilter {
    EntityFilter::CONSTRUCTION_PLANE
        | EntityFilter::FACE
        | EntityFilter::PLANAR_FACE
        | EntityFilter::BODY
        | EntityFilter::VERTEX
        | EntityFilter::CONSTRUCTION_POINT
        | EntityFilter::SKETCH_POINT
}

/// The fields of one side's extent: chooser, distance, object, offset, extend, taper.
#[allow(
    clippy::too_many_arguments,
    reason = "one key and label per field of a side"
)]
fn side_fields(
    keys: [&'static str; 6],
    labels: [&str; 6],
    shown: Option<Condition>,
    dist_enabled: Condition,
    extent_enabled: Option<Condition>,
) -> Vec<FieldSpec> {
    let [k_ext, k_dist, k_obj, k_off, k_extend, k_taper] = keys;
    let both = |c: Condition| -> Condition {
        match &shown {
            Some(s) => Condition::All(vec![s.clone(), c]),
            None => c,
        }
    };
    let mut ext = choice(k_ext, labels[0], &EXTENTS);
    ext.visible_if.clone_from(&shown);
    ext.enabled_if = extent_enabled;
    let mut dist = expr(k_dist, labels[1], Dim::LENGTH, DEFAULT_DISTANCE);
    dist.manipulator = Some(ManipKind::Distance);
    dist.visible_if.clone_from(&shown);
    dist.enabled_if = Some(dist_enabled);
    let to_object = both(is(k_ext, "to_object"));
    let mut obj = FieldSpec::new(
        k_obj,
        labels[2],
        FieldKind::Selection {
            filter: target_filter(),
            min: 1,
            max: Some(1),
        },
    );
    obj.tooltip = Some("Plane, face, body or vertex the side extends to.".into());
    obj.visible_if = Some(to_object.clone());
    let mut off = expr(k_off, labels[3], Dim::LENGTH, ZERO_MM);
    off.visible_if = Some(to_object.clone());
    let mut extend = FieldSpec::new(k_extend, labels[4], FieldKind::Bool);
    extend.tooltip = Some("Extend the target face to its untrimmed surface.".into());
    extend.visible_if = Some(to_object);
    let mut taper = expr(k_taper, labels[5], Dim::ANGLE, ZERO_DEG);
    taper.visible_if = shown;
    vec![ext, dist, obj, off, extend, taper]
}

fn schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut profiles = FieldSpec::new(
            PROFILES,
            "Profiles",
            FieldKind::Selection {
                filter: EntityFilter::PROFILE | EntityFilter::PLANAR_FACE,
                min: 1,
                max: None,
            },
        );
        profiles.tooltip = Some("Sketch profiles, text or planar faces to extrude.".into());
        let start = choice(START, "Start", &STARTS);
        let mut start_offset = expr(START_OFFSET, "Start offset", Dim::LENGTH, ZERO_MM);
        start_offset.visible_if = Some(not(is(START, "profile")));
        let mut start_object = FieldSpec::new(
            START_OBJECT,
            "Start object",
            FieldKind::Selection {
                filter: EntityFilter::CONSTRUCTION_PLANE
                    | EntityFilter::PLANAR_FACE
                    | EntityFilter::VERTEX
                    | EntityFilter::CONSTRUCTION_POINT
                    | EntityFilter::SKETCH_POINT,
                min: 1,
                max: Some(1),
            },
        );
        start_object.tooltip =
            Some("Plane or planar face parallel to the profile, or a point.".into());
        start_object.visible_if = Some(is(START, "object"));
        let direction = choice(DIRECTION, "Direction", &DIRECTIONS);
        let mut whole = FieldSpec::new(WHOLE_LENGTH, "Whole length", FieldKind::Bool);
        whole.tooltip = Some("Each side gets the whole distance (else half).".into());
        whole.visible_if = Some(is(DIRECTION, "symmetric"));
        let side1 = side_fields(
            [EXTENT, DISTANCE, OBJECT, OBJECT_OFFSET, EXTEND, TAPER],
            [
                "Extent",
                "Distance",
                "Object",
                "Object offset",
                "Extend faces",
                "Taper angle",
            ],
            None,
            Condition::Any(vec![is(EXTENT, "distance"), is(DIRECTION, "symmetric")]),
            Some(not(is(DIRECTION, "symmetric"))),
        );
        let side2 = side_fields(
            [EXTENT2, DISTANCE2, OBJECT2, OBJECT_OFFSET2, EXTEND2, TAPER2],
            [
                "Side 2 extent",
                "Side 2 distance",
                "Side 2 object",
                "Side 2 object offset",
                "Side 2 extend faces",
                "Side 2 taper angle",
            ],
            Some(is(DIRECTION, "two_sides")),
            is(EXTENT2, "distance"),
            None,
        );
        let operation = choice(
            OPERATION,
            "Operation",
            &OPS.map(|(id, _, label)| (id, label)),
        );
        let mut bodies = FieldSpec::new(
            BODIES,
            "Objects",
            FieldKind::Selection {
                filter: EntityFilter::BODY,
                min: 0,
                max: None,
            },
        );
        bodies.tooltip = Some("Bodies the operation applies to.".into());
        bodies.visible_if = Some(not(is(OPERATION, "new_body")));
        let mut fields = vec![
            profiles,
            start,
            start_offset,
            start_object,
            direction,
            whole,
        ];
        fields.extend(side1);
        fields.extend(side2);
        fields.extend([
            FieldSpec::new(FLIP, "Flip", FieldKind::FlipDirection),
            operation,
            bodies,
        ]);
        DialogSchema {
            kind: "solid.extrude",
            title: "Extrude".into(),
            help: Some("solid.extrude"),
            fields,
        }
    })
}

/// The values of one side for `s` (defaults where a field doesn't apply).
fn side_values(
    v: FieldValues,
    keys: [&'static str; 6],
    s: Option<&ExtentSide>,
    design: &Design,
) -> FieldValues {
    let [k_ext, k_dist, k_obj, k_off, k_extend, k_taper] = keys;
    let mut ext = "distance";
    let mut dist = FieldValue::expr(DEFAULT_DISTANCE);
    let mut obj = Vec::new();
    let mut off = FieldValue::expr(ZERO_MM);
    let mut extend = false;
    let mut taper = FieldValue::expr(ZERO_DEG);
    if let Some(s) = s {
        match &s.extent {
            Extent::Distance(d) => dist = FieldValue::from_expr(d, design),
            Extent::ToObject {
                target,
                offset,
                extend: e,
            } => {
                ext = "to_object";
                obj = vec![target.clone()];
                off = FieldValue::from_expr(offset, design);
                extend = *e;
            }
            Extent::All | Extent::Full => ext = "all",
        }
        taper = FieldValue::from_expr(&s.taper, design);
    }
    v.with(k_ext, FieldValue::Choice(ext))
        .with(k_dist, dist)
        .with(k_obj, FieldValue::Selection(obj))
        .with(k_off, off)
        .with(k_extend, FieldValue::Bool(extend))
        .with(k_taper, taper)
}

const SIDE1: [&str; 6] = [EXTENT, DISTANCE, OBJECT, OBJECT_OFFSET, EXTEND, TAPER];
const SIDE2: [&str; 6] = [EXTENT2, DISTANCE2, OBJECT2, OBJECT_OFFSET2, EXTEND2, TAPER2];

fn values_of(e: &Extrude, design: &Design) -> FieldValues {
    let (start, start_offset, start_object) = match &e.start {
        Start::Profile => ("profile", FieldValue::expr(ZERO_MM), Vec::new()),
        Start::Offset(o) => ("offset", FieldValue::from_expr(o, design), Vec::new()),
        Start::Object { target, offset } => (
            "object",
            FieldValue::from_expr(offset, design),
            vec![target.clone()],
        ),
    };
    let (direction, whole) = match e.direction {
        Direction::OneSide => ("one_side", false),
        Direction::TwoSides => ("two_sides", false),
        Direction::Symmetric { whole_length } => ("symmetric", whole_length),
    };
    let op = OPS
        .iter()
        .find(|(_, o, _)| *o == e.target.op)
        .map_or("new_body", |(id, _, _)| id);
    let v = FieldValues::default()
        .with(PROFILES, FieldValue::Selection(e.profiles.clone()))
        .with(START, FieldValue::Choice(start))
        .with(START_OFFSET, start_offset)
        .with(START_OBJECT, FieldValue::Selection(start_object))
        .with(DIRECTION, FieldValue::Choice(direction))
        .with(WHOLE_LENGTH, FieldValue::Bool(whole));
    let v = side_values(v, SIDE1, Some(&e.side1), design);
    let side2 = if e.direction == Direction::TwoSides {
        e.side2.as_ref()
    } else {
        None
    };
    side_values(v, SIDE2, side2, design)
        .with(FLIP, FieldValue::Flip(e.flip))
        .with(OPERATION, FieldValue::Choice(op))
        .with(
            BODIES,
            FieldValue::Selection(
                e.target
                    .participants
                    .iter()
                    .map(|b| EntityRef::Body(*b))
                    .collect(),
            ),
        )
}

fn defaults(init: &DialogInit<'_>) -> FieldValues {
    let profiles: Vec<EntityRef> = init
        .preselection
        .iter()
        .filter(|e| match e {
            EntityRef::Profile { .. } => true,
            EntityRef::Topo(t) => t.kind == amg_doc::TopoKind::Face,
            _ => false,
        })
        .cloned()
        .collect();
    let f = Extrude::distance(profiles, DEFAULT_DISTANCE, OpTarget::default());
    values_of(&f, init.design)
}

fn read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    let e = f.as_any().downcast_ref::<Extrude>()?;
    Some(values_of(e, design))
}

pub(crate) fn one_ref(
    v: &FieldValues,
    key: &'static str,
    errs: &mut FieldErrors,
) -> Option<EntityRef> {
    match v.selection(key) {
        Ok([r]) => Some(r.clone()),
        Ok(_) => {
            errs.0.push((FieldKey(key), "select one object".into()));
            None
        }
        Err(e) => {
            errs.0.push(e);
            None
        }
    }
}

pub(crate) fn take_expr(
    v: &FieldValues,
    key: &'static str,
    errs: &mut FieldErrors,
) -> Option<Expr> {
    v.expr(key).map_err(|e| errs.0.push(e)).ok()
}

pub(crate) fn take_choice(
    v: &FieldValues,
    key: &'static str,
    default: &'static str,
    errs: &mut FieldErrors,
) -> &'static str {
    v.choice(key, default).unwrap_or_else(|e| {
        errs.0.push(e);
        default
    })
}

/// One side from its fields; `None` (with errors) when a used field is invalid.
fn build_side(
    v: &FieldValues,
    keys: [&'static str; 6],
    errs: &mut FieldErrors,
) -> Option<ExtentSide> {
    let [k_ext, k_dist, k_obj, k_off, k_extend, k_taper] = keys;
    let taper = take_expr(v, k_taper, errs);
    let extent = match take_choice(v, k_ext, "distance", errs) {
        "distance" => take_expr(v, k_dist, errs).map(Extent::Distance),
        "to_object" => {
            let target = one_ref(v, k_obj, errs);
            let offset = take_expr(v, k_off, errs);
            let extend = v
                .flag(k_extend)
                .map_err(|e| errs.0.push(e))
                .unwrap_or(false);
            target.zip(offset).map(|(target, offset)| Extent::ToObject {
                target,
                offset,
                extend,
            })
        }
        "all" => Some(Extent::All),
        other => {
            errs.0
                .push((FieldKey(k_ext), format!("unknown extent `{other}`")));
            None
        }
    };
    Some(ExtentSide {
        extent: extent?,
        taper: taper?,
    })
}

fn build(v: &FieldValues, old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let mut f = old
        .and_then(|o| o.as_any().downcast_ref::<Extrude>())
        .cloned()
        .unwrap_or_else(|| Extrude::distance(Vec::new(), DEFAULT_DISTANCE, OpTarget::default()));
    match v.selection(PROFILES) {
        Ok([]) => errs.0.push((FieldKey(PROFILES), "select a profile".into())),
        Ok(p) => f.profiles = p.to_vec(),
        Err(e) => errs.0.push(e),
    }
    match take_choice(v, START, "profile", &mut errs) {
        "profile" => f.start = Start::Profile,
        "offset" => {
            if let Some(o) = take_expr(v, START_OFFSET, &mut errs) {
                f.start = Start::Offset(o);
            }
        }
        "object" => {
            let target = one_ref(v, START_OBJECT, &mut errs);
            let offset = take_expr(v, START_OFFSET, &mut errs);
            if let Some((target, offset)) = target.zip(offset) {
                f.start = Start::Object { target, offset };
            }
        }
        other => errs
            .0
            .push((FieldKey(START), format!("unknown start `{other}`"))),
    }
    let direction = match take_choice(v, DIRECTION, "one_side", &mut errs) {
        "one_side" => Direction::OneSide,
        "two_sides" => Direction::TwoSides,
        "symmetric" => Direction::Symmetric {
            whole_length: v
                .flag(WHOLE_LENGTH)
                .map_err(|e| errs.0.push(e))
                .unwrap_or(false),
        },
        other => {
            errs.0
                .push((FieldKey(DIRECTION), format!("unknown direction `{other}`")));
            Direction::OneSide
        }
    };
    f.direction = direction;
    if matches!(direction, Direction::Symmetric { .. }) {
        // Symmetric: the distance and taper of side 1, both ways.
        let d = take_expr(v, DISTANCE, &mut errs);
        let t = take_expr(v, TAPER, &mut errs);
        if let Some((d, t)) = d.zip(t) {
            f.side1 = ExtentSide {
                extent: Extent::Distance(d),
                taper: t,
            };
        }
    } else if let Some(s) = build_side(v, SIDE1, &mut errs) {
        f.side1 = s;
    }
    f.side2 = if direction == Direction::TwoSides {
        build_side(v, SIDE2, &mut errs)
    } else {
        None
    };
    match v.flag(FLIP) {
        Ok(b) => f.flip = b,
        Err(e) => errs.0.push(e),
    }
    let id = take_choice(v, OPERATION, "new_body", &mut errs);
    match OPS.iter().find(|(o, _, _)| *o == id) {
        Some((_, op, _)) => f.target.op = *op,
        None => errs
            .0
            .push((FieldKey(OPERATION), format!("unknown operation `{id}`"))),
    }
    match v.selection(BODIES) {
        Ok(b) => {
            let mut ids: Vec<BodyId> = Vec::new();
            for e in b {
                match e {
                    EntityRef::Body(id) => ids.push(*id),
                    _ => errs.0.push((FieldKey(BODIES), "select bodies".into())),
                }
            }
            f.target.participants = ids;
        }
        Err(e) => errs.0.push(e),
    }
    if errs.is_empty() {
        Ok(FeatureBox::new(f))
    } else {
        Err(errs)
    }
}

/// The distance arrows: at the first profile region's centroid, along side 1's direction (and
/// against it for side 2 of a two-sided extrude).
fn anchors(f: &dyn Feature, _design: &Design, eval: &Evaluated) -> Vec<ManipAnchor> {
    let Some(e) = f.as_any().downcast_ref::<Extrude>() else {
        return Vec::new();
    };
    let Some(EntityRef::Profile {
        sketch, profile, ..
    }) = e.profiles.first()
    else {
        return Vec::new();
    };
    let Some(g) = eval.sketch(*sketch) else {
        return Vec::new();
    };
    let Some(p) = g.profiles.get(amg_sketch::ProfileId(profile.0)) else {
        return Vec::new();
    };
    let c = p.fingerprint.centroid;
    let pl = g.plane;
    let origin: Point3 = pl.origin + pl.x.get() * c.x + pl.y.get() * c.y;
    let axis = if e.flip { -pl.normal } else { pl.normal };
    let mut v = vec![ManipAnchor {
        field: FieldKey(DISTANCE),
        origin,
        axis: Dir3::new_unchecked(axis.get()),
        ref_dir: None,
        base: 0.0,
    }];
    if e.direction == Direction::TwoSides {
        v.push(ManipAnchor {
            field: FieldKey(DISTANCE2),
            origin,
            axis: Dir3::new_unchecked(-axis.get()),
            ref_dir: None,
            base: 0.0,
        });
    }
    v
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.extrude",
        schema,
        defaults,
        read,
        build,
        anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "create",
            order: 10,
            pinned: true,
            icon_svg: &[],
            shortcut: Some("E"),
            keywords: &["extrude", "boss", "pocket", "pull", "push", "taper", "text"],
        },
        new_component: None,
    }
}
