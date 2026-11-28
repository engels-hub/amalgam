//! The `solid.revolve` dialog (UI-05 renders it from the schema): profiles, axis (origin axis or
//! a selected sketch line, edge or construction axis), direction, extent per side (fixed angle,
//! to object, full turn), flip, operation. Conditional fields are hidden for the defaults (one
//! side, full turn about Z), as for the extrude.

use std::sync::OnceLock;

use amg_core::{Dir3, Point3};
use amg_doc::{BodyId, Design, Dim, EntityRef, Evaluated, Expr, Feature, FeatureBox};

use super::Revolve;
use crate::construct::{OriginEntity, RefInput};
use crate::dialog::{
    CommandMeta, Condition, DialogInit, DialogRegistration, DialogSchema, EntityFilter,
    FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue, FieldValues, ManipAnchor, ManipKind,
};
use crate::solid::common::OpTarget;
use crate::solid::extent::{Direction, Extent, ExtentSide};
use crate::solid::extrude::dialog::{
    OPS, choice, expr, is, new_component_of, not, one_ref, op_of, op_options, take_choice,
    take_expr, target_filter, uses_participants,
};
use crate::solid::pattern::dialog::{AXES, axis_values, read_axis};

const PROFILES: &str = "profiles";
const AXIS: &str = "axis";
const AXIS_REF: &str = "axis_ref";
const DIRECTION: &str = "direction";
const WHOLE_LENGTH: &str = "whole_length";
const EXTENT: &str = "extent";
const ANGLE: &str = "angle";
const OBJECT: &str = "object";
const OBJECT_OFFSET: &str = "object_offset";
const EXTEND: &str = "extend";
const EXTENT2: &str = "side2_extent";
const ANGLE2: &str = "side2_angle";
const OBJECT2: &str = "side2_object";
const OBJECT_OFFSET2: &str = "side2_object_offset";
const EXTEND2: &str = "side2_extend";
const FLIP: &str = "flip";
const OPERATION: &str = "operation";
const BODIES: &str = "bodies";

// Option labels differ from every field label (the UI harness finds fields by label).
const DIRECTIONS: [(&str, &str); 3] = [
    ("one_side", "One side"),
    ("two_sides", "Two sides"),
    ("symmetric", "Symmetric"),
];
const EXTENTS: [(&str, &str); 3] = [
    ("full", "Full turn"),
    ("angle", "Fixed angle"),
    ("to_object", "To object"),
];
const EXTENTS2: [(&str, &str); 2] = [("angle", "Fixed angle"), ("to_object", "To object")];

const DEFAULT_ANGLE: &str = "360 deg";
const DEFAULT_ANGLE2: &str = "90 deg";
const ZERO_MM: &str = "0 mm";
const ZERO_DEG: &str = "0 deg";

/// The fields of one side: chooser, angle, object, offset, extend.
fn side_fields(
    keys: [&'static str; 5],
    labels: [&str; 5],
    extents: &[(&'static str, &str)],
    default_angle: &'static str,
    shown: Option<Condition>,
    angle_enabled: Condition,
    extent_enabled: Option<Condition>,
) -> Vec<FieldSpec> {
    let [k_ext, k_angle, k_obj, k_off, k_extend] = keys;
    let mut ext = choice(k_ext, labels[0], extents);
    ext.visible_if.clone_from(&shown);
    ext.enabled_if = extent_enabled;
    let mut angle = expr(k_angle, labels[1], Dim::ANGLE, default_angle);
    angle.manipulator = Some(ManipKind::Angle);
    angle.visible_if.clone_from(&shown);
    angle.enabled_if = Some(angle_enabled);
    let to_object = match &shown {
        Some(s) => Condition::All(vec![s.clone(), is(k_ext, "to_object")]),
        None => is(k_ext, "to_object"),
    };
    let mut obj = FieldSpec::new(
        k_obj,
        labels[2],
        FieldKind::Selection {
            filter: target_filter(),
            min: 1,
            max: Some(1),
        },
    );
    obj.tooltip = Some("Plane, face, body or vertex the side turns to.".into());
    obj.visible_if = Some(to_object.clone());
    let mut off = expr(k_off, labels[3], Dim::LENGTH, ZERO_MM);
    off.visible_if = Some(to_object.clone());
    let mut extend = FieldSpec::new(k_extend, labels[4], FieldKind::Bool);
    extend.tooltip = Some("Extend the target face to its untrimmed surface.".into());
    extend.visible_if = Some(to_object);
    vec![ext, angle, obj, off, extend]
}

const SIDE1: [&str; 5] = [EXTENT, ANGLE, OBJECT, OBJECT_OFFSET, EXTEND];
const SIDE2: [&str; 5] = [EXTENT2, ANGLE2, OBJECT2, OBJECT_OFFSET2, EXTEND2];

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
        profiles.tooltip = Some("Sketch profiles or planar faces to revolve.".into());
        let axis = choice(AXIS, "Axis", &AXES);
        let mut axis_ref = FieldSpec::new(
            AXIS_REF,
            "Axis reference",
            FieldKind::Selection {
                filter: EntityFilter::EDGE
                    | EntityFilter::CONSTRUCTION_AXIS
                    | EntityFilter::SKETCH_CURVE
                    | EntityFilter::FACE,
                min: 1,
                max: Some(1),
            },
        );
        axis_ref.tooltip =
            Some("Sketch line, straight edge or construction axis to revolve about.".into());
        axis_ref.visible_if = Some(is(AXIS, "picked"));
        let direction = choice(DIRECTION, "Direction", &DIRECTIONS);
        let mut whole = FieldSpec::new(WHOLE_LENGTH, "Whole angle", FieldKind::Bool);
        whole.tooltip = Some("Each side gets the whole angle (else half).".into());
        whole.visible_if = Some(is(DIRECTION, "symmetric"));
        let side1 = side_fields(
            SIDE1,
            ["Extent", "Angle", "Object", "Object offset", "Extend faces"],
            &EXTENTS,
            DEFAULT_ANGLE,
            None,
            Condition::Any(vec![is(EXTENT, "angle"), is(DIRECTION, "symmetric")]),
            Some(not(is(DIRECTION, "symmetric"))),
        );
        let side2 = side_fields(
            SIDE2,
            [
                "Side 2 extent",
                "Side 2 angle",
                "Side 2 object",
                "Side 2 object offset",
                "Side 2 extend faces",
            ],
            &EXTENTS2,
            DEFAULT_ANGLE2,
            Some(is(DIRECTION, "two_sides")),
            is(EXTENT2, "angle"),
            None,
        );
        let operation = choice(OPERATION, "Operation", &op_options());
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
        bodies.visible_if = Some(uses_participants());
        let mut fields = vec![profiles, axis, axis_ref, direction, whole];
        fields.extend(side1);
        fields.extend(side2);
        fields.extend([
            FieldSpec::new(FLIP, "Flip", FieldKind::FlipDirection),
            operation,
            bodies,
        ]);
        DialogSchema {
            kind: "solid.revolve",
            title: "Revolve".into(),
            help: Some("solid.revolve"),
            fields,
        }
    })
}

/// The values of one side for `s` (defaults where a field doesn't apply).
fn side_values(
    v: FieldValues,
    keys: [&'static str; 5],
    default_angle: &'static str,
    s: Option<&ExtentSide>,
    design: &Design,
) -> FieldValues {
    let [k_ext, k_angle, k_obj, k_off, k_extend] = keys;
    let mut ext = "angle";
    let mut angle = FieldValue::expr(default_angle);
    let mut obj = Vec::new();
    let mut off = FieldValue::expr(ZERO_MM);
    let mut extend = false;
    if let Some(s) = s {
        match &s.extent {
            Extent::Distance(d) => angle = FieldValue::from_expr(d, design),
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
            Extent::Full | Extent::All => ext = "full",
        }
    }
    v.with(k_ext, FieldValue::Choice(ext))
        .with(k_angle, angle)
        .with(k_obj, FieldValue::Selection(obj))
        .with(k_off, off)
        .with(k_extend, FieldValue::Bool(extend))
}

fn values_of(r: &Revolve, design: &Design) -> FieldValues {
    let (axis, axis_ref) = axis_values(&r.axis, "z");
    let (direction, whole) = match r.direction {
        Direction::OneSide => ("one_side", false),
        Direction::TwoSides => ("two_sides", false),
        Direction::Symmetric { whole_length } => ("symmetric", whole_length),
    };
    let op = OPS
        .iter()
        .find(|(_, o, _)| *o == r.target.op)
        .map_or("new_body", |(id, _, _)| id);
    let v = FieldValues::default()
        .with(PROFILES, FieldValue::Selection(r.profiles.clone()))
        .with(AXIS, FieldValue::Choice(axis))
        .with(AXIS_REF, FieldValue::Selection(axis_ref))
        .with(DIRECTION, FieldValue::Choice(direction))
        .with(WHOLE_LENGTH, FieldValue::Bool(whole));
    let v = side_values(v, SIDE1, DEFAULT_ANGLE, Some(&r.side1), design);
    let side2 = if r.direction == Direction::TwoSides {
        r.side2.as_ref()
    } else {
        None
    };
    side_values(v, SIDE2, DEFAULT_ANGLE2, side2, design)
        .with(FLIP, FieldValue::Flip(r.flip))
        .with(OPERATION, FieldValue::Choice(op))
        .with(
            BODIES,
            FieldValue::Selection(
                r.target
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
    // A preselected sketch line, edge or construction axis becomes the axis.
    let axis = init
        .preselection
        .iter()
        .find(|e| match e {
            EntityRef::SketchCurve { .. } | EntityRef::Construction(_) => true,
            EntityRef::Topo(t) => t.kind == amg_doc::TopoKind::Edge,
            _ => false,
        })
        .cloned()
        .map_or(RefInput::Origin(OriginEntity::ZAxis), RefInput::Ref);
    let f = Revolve::full(profiles, axis, OpTarget::default());
    values_of(&f, init.design)
}

fn read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    let r = f.as_any().downcast_ref::<Revolve>()?;
    Some(values_of(r, design))
}

fn zero_deg() -> Expr {
    Expr::Inline(amg_doc::ExprSrc(ZERO_DEG.into()))
}

/// One side from its fields; `None` (with errors) when a used field is invalid.
fn build_side(
    v: &FieldValues,
    keys: [&'static str; 5],
    errs: &mut FieldErrors,
) -> Option<ExtentSide> {
    let [k_ext, k_angle, k_obj, k_off, k_extend] = keys;
    let extent = match take_choice(v, k_ext, "angle", errs) {
        "angle" => take_expr(v, k_angle, errs).map(Extent::Distance),
        "full" => Some(Extent::Full),
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
        other => {
            errs.0
                .push((FieldKey(k_ext), format!("unknown extent `{other}`")));
            None
        }
    };
    Some(ExtentSide {
        extent: extent?,
        taper: zero_deg(),
    })
}

fn build(v: &FieldValues, old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let mut f = old
        .and_then(|o| o.as_any().downcast_ref::<Revolve>())
        .cloned()
        .unwrap_or_else(|| {
            Revolve::full(
                Vec::new(),
                RefInput::Origin(OriginEntity::ZAxis),
                OpTarget::default(),
            )
        });
    match v.selection(PROFILES) {
        Ok([]) => errs.0.push((FieldKey(PROFILES), "select a profile".into())),
        Ok(p) => f.profiles = p.to_vec(),
        Err(e) => errs.0.push(e),
    }
    match read_axis(v, AXIS, AXIS_REF, "z") {
        Ok(a) => f.axis = a,
        Err(e) => errs.0.push(e),
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
        // Symmetric: the angle of side 1, both ways.
        if let Some(a) = take_expr(v, ANGLE, &mut errs) {
            f.side1 = ExtentSide {
                extent: Extent::Distance(a),
                taper: zero_deg(),
            };
        }
    } else if let Some(s) = build_side(v, SIDE1, &mut errs) {
        if direction == Direction::TwoSides && s.extent == Extent::Full {
            errs.0.push((
                FieldKey(EXTENT),
                "a full turn is one-sided; choose an angle".into(),
            ));
        }
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
    match op_of(id) {
        Some(op) => f.target.op = op,
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

/// The angle arcs: about an origin axis (other axes resolve only inside a recompute), starting
/// at the first profile region's centroid; side 2 of a two-sided revolve turns the other way.
fn anchors(f: &dyn Feature, _design: &Design, eval: &Evaluated) -> Vec<ManipAnchor> {
    let Some(r) = f.as_any().downcast_ref::<Revolve>() else {
        return Vec::new();
    };
    let dir = match r.axis {
        RefInput::Origin(OriginEntity::XAxis) => amg_core::Vec3::X,
        RefInput::Origin(OriginEntity::YAxis) => amg_core::Vec3::Y,
        RefInput::Origin(OriginEntity::ZAxis) => amg_core::Vec3::Z,
        _ => return Vec::new(),
    };
    let dir = if r.flip { -dir } else { dir };
    let Some(EntityRef::Profile {
        sketch, profile, ..
    }) = r.profiles.first()
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
    let at: Point3 = pl.origin + pl.x.get() * c.x + pl.y.get() * c.y;
    let radial = at - dir * at.dot(dir);
    let Some(ref_dir) = radial.try_normalize() else {
        return Vec::new();
    };
    let anchor = |field, axis: amg_core::Vec3| ManipAnchor {
        field: FieldKey(field),
        origin: Point3::ZERO,
        axis: Dir3::new_unchecked(axis),
        ref_dir: Some(Dir3::new_unchecked(ref_dir)),
        base: 0.0,
    };
    let mut v = vec![anchor(ANGLE, dir)];
    if r.direction == Direction::TwoSides {
        v.push(anchor(ANGLE2, -dir));
    }
    v
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.revolve",
        schema,
        defaults,
        read,
        build,
        anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "create",
            order: 20,
            pinned: true,
            icon_svg: &[],
            shortcut: None,
            keywords: &["revolve", "revolution", "lathe", "turn", "spin", "axis"],
        },
        new_component: Some(new_component_of),
    }
}
