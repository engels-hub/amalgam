//! Dialog registrations of `solid.move` and `solid.align` (S11 §B schemas, rendered by
//! UI-05). Free moves come from the triad manipulator (UI-06): the dialog keeps the stored
//! matrix of an edited free move and starts a new one at the identity.

use std::sync::OnceLock;

use amg_doc::{BodyId, Design, Dim, EntityRef, Evaluated, Feature, FeatureBox};

use super::{Align, IDENTITY_MATRIX, Move, MoveSpec};
use crate::construct::RefInput;
use crate::dialog::{
    CommandMeta, Condition, DialogInit, DialogRegistration, DialogSchema, EntityFilter,
    FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue, FieldValues, ManipAnchor, ManipKind,
};
use crate::solid::extrude::dialog::{choice, expr, is, one_ref, take_choice, take_expr};
use crate::solid::pattern::dialog::{AXES, axis_values, read_axis};

const BODIES: &str = "bodies";
const MOVE_TYPE: &str = "move_type";
const DX: &str = "dx";
const DY: &str = "dy";
const DZ: &str = "dz";
const AXIS: &str = "axis";
const AXIS_REF: &str = "axis_ref";
const ANGLE: &str = "angle";
const FROM_POINT: &str = "from_point";
const TO_POINT: &str = "to_point";
const COPY: &str = "copy";

const MOVE_TYPES: [(&str, &str); 4] = [
    ("free", "Free move"),
    ("translate", "Translate"),
    ("rotate", "Rotate"),
    ("point", "Point to point"),
];

const ZERO_MM: &str = "0 mm";
const ZERO_DEG: &str = "0 deg";

const POINT_FILTER: EntityFilter = EntityFilter::VERTEX
    .union(EntityFilter::SKETCH_POINT)
    .union(EntityFilter::CONSTRUCTION_POINT)
    .union(EntityFilter::EDGE)
    .union(EntityFilter::SKETCH_CURVE);

fn pick(key: &'static str, label: &str, filter: EntityFilter, max: Option<u32>) -> FieldSpec {
    FieldSpec::new(
        key,
        label,
        FieldKind::Selection {
            filter,
            min: 1,
            max,
        },
    )
}

fn shown(mut f: FieldSpec, c: Condition) -> FieldSpec {
    f.visible_if = Some(c);
    f
}

fn manip(mut f: FieldSpec, m: ManipKind) -> FieldSpec {
    f.manipulator = Some(m);
    f
}

fn push<T>(r: Result<T, (FieldKey, String)>, errs: &mut FieldErrors) -> Option<T> {
    r.map_err(|e| errs.0.push(e)).ok()
}

fn no_anchors(_f: &dyn Feature, _design: &Design, _eval: &Evaluated) -> Vec<ManipAnchor> {
    Vec::new()
}

fn preselected_bodies(init: &DialogInit<'_>) -> Vec<BodyId> {
    init.preselection
        .iter()
        .filter_map(|e| match e {
            EntityRef::Body(b) => Some(*b),
            _ => None,
        })
        .collect()
}

fn bodies_value(b: &[BodyId]) -> FieldValue {
    FieldValue::Selection(b.iter().copied().map(EntityRef::Body).collect())
}

fn read_bodies(v: &FieldValues, errs: &mut FieldErrors) -> Vec<BodyId> {
    let mut out = Vec::new();
    for e in push(v.selection(BODIES), errs).unwrap_or_default() {
        match e {
            EntityRef::Body(b) => out.push(*b),
            _ => errs.0.push((FieldKey(BODIES), "select bodies".into())),
        }
    }
    out
}

fn optional_ref(v: &FieldValues, key: &'static str, errs: &mut FieldErrors) -> Option<EntityRef> {
    match v.selection(key) {
        Ok([]) => None,
        _ => one_ref(v, key, errs),
    }
}

// ---- move -------------------------------------------------------------------------------------

fn move_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let tr = || is(MOVE_TYPE, "translate");
        let rot = || is(MOVE_TYPE, "rotate");
        let pt = || is(MOVE_TYPE, "point");
        let mut copy = FieldSpec::new(COPY, "Create copy", FieldKind::Bool);
        copy.tooltip = Some("Leave the bodies in place and move copies of them.".into());
        let mut ty = choice(MOVE_TYPE, "Move type", &MOVE_TYPES);
        ty.manipulator = Some(ManipKind::Triad);
        DialogSchema {
            kind: "solid.move",
            title: "Move/Copy".into(),
            help: Some("solid.move"),
            fields: vec![
                pick(BODIES, "Bodies to move", EntityFilter::BODY, None),
                ty,
                shown(
                    manip(
                        expr(DX, "X distance", Dim::LENGTH, ZERO_MM),
                        ManipKind::Distance,
                    ),
                    tr(),
                ),
                shown(
                    manip(
                        expr(DY, "Y distance", Dim::LENGTH, ZERO_MM),
                        ManipKind::Distance,
                    ),
                    tr(),
                ),
                shown(
                    manip(
                        expr(DZ, "Z distance", Dim::LENGTH, ZERO_MM),
                        ManipKind::Distance,
                    ),
                    tr(),
                ),
                shown(choice(AXIS, "Rotation axis", &AXES), rot()),
                shown(
                    pick(
                        AXIS_REF,
                        "Axis reference",
                        EntityFilter::EDGE
                            | EntityFilter::SKETCH_CURVE
                            | EntityFilter::CONSTRUCTION_AXIS
                            | EntityFilter::FACE,
                        Some(1),
                    ),
                    Condition::All(vec![rot(), is(AXIS, "picked")]),
                ),
                shown(
                    manip(expr(ANGLE, "Angle", Dim::ANGLE, ZERO_DEG), ManipKind::Angle),
                    rot(),
                ),
                shown(pick(FROM_POINT, "From point", POINT_FILTER, Some(1)), pt()),
                shown(pick(TO_POINT, "To point", POINT_FILTER, Some(1)), pt()),
                copy,
            ],
        }
    })
}

fn move_values(m: &Move, design: &Design) -> FieldValues {
    let x = |e: &amg_doc::Expr| FieldValue::from_expr(e, design);
    let mm = || FieldValue::expr(ZERO_MM);
    let (ty, d, axis, axis_ref, angle, from, to) = match &m.spec {
        MoveSpec::Free { .. } => (
            "free",
            [mm(), mm(), mm()],
            "z",
            Vec::new(),
            FieldValue::expr(ZERO_DEG),
            Vec::new(),
            Vec::new(),
        ),
        MoveSpec::Translate(d) => (
            "translate",
            [x(&d[0]), x(&d[1]), x(&d[2])],
            "z",
            Vec::new(),
            FieldValue::expr(ZERO_DEG),
            Vec::new(),
            Vec::new(),
        ),
        MoveSpec::Rotate { axis, angle } => {
            let (a, r) = axis_values(axis, "z");
            (
                "rotate",
                [mm(), mm(), mm()],
                a,
                r,
                x(angle),
                Vec::new(),
                Vec::new(),
            )
        }
        MoveSpec::PointToPoint { from, to } => (
            "point",
            [mm(), mm(), mm()],
            "z",
            Vec::new(),
            FieldValue::expr(ZERO_DEG),
            from.entity().cloned().into_iter().collect(),
            to.entity().cloned().into_iter().collect(),
        ),
    };
    let [dx, dy, dz] = d;
    FieldValues::default()
        .with(BODIES, bodies_value(&m.bodies))
        .with(MOVE_TYPE, FieldValue::Choice(ty))
        .with(DX, dx)
        .with(DY, dy)
        .with(DZ, dz)
        .with(AXIS, FieldValue::Choice(axis))
        .with(AXIS_REF, FieldValue::Selection(axis_ref))
        .with(ANGLE, angle)
        .with(FROM_POINT, FieldValue::Selection(from))
        .with(TO_POINT, FieldValue::Selection(to))
        .with(COPY, FieldValue::Bool(m.copy))
}

fn move_defaults(init: &DialogInit<'_>) -> FieldValues {
    let m = Move::translate(preselected_bodies(init), [ZERO_MM, ZERO_MM, ZERO_MM]);
    move_values(&m, init.design)
}

fn move_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<Move>()
        .map(|m| move_values(m, design))
}

fn point_ref(v: &FieldValues, key: &'static str, errs: &mut FieldErrors) -> Option<RefInput> {
    match optional_ref(v, key, errs) {
        Some(r) => Some(RefInput::Ref(r)),
        None => {
            errs.0.push((FieldKey(key), "select a point".into()));
            None
        }
    }
}

fn move_build(v: &FieldValues, old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let bodies = read_bodies(v, &mut errs);
    let copy = push(v.flag(COPY), &mut errs).unwrap_or(false);
    let spec = match take_choice(v, MOVE_TYPE, "translate", &mut errs) {
        "free" => {
            let matrix = old
                .and_then(|f| f.as_any().downcast_ref::<Move>())
                .and_then(|m| match m.spec {
                    MoveSpec::Free { matrix } => Some(matrix),
                    _ => None,
                })
                .unwrap_or(IDENTITY_MATRIX);
            Some(MoveSpec::Free { matrix })
        }
        "rotate" => {
            let axis = push(read_axis(v, AXIS, AXIS_REF, "z"), &mut errs);
            let angle = take_expr(v, ANGLE, &mut errs);
            axis.zip(angle)
                .map(|(axis, angle)| MoveSpec::Rotate { axis, angle })
        }
        "point" => {
            let from = point_ref(v, FROM_POINT, &mut errs);
            let to = point_ref(v, TO_POINT, &mut errs);
            from.zip(to)
                .map(|(from, to)| MoveSpec::PointToPoint { from, to })
        }
        "translate" => {
            let d = [DX, DY, DZ].map(|k| take_expr(v, k, &mut errs));
            match d {
                [Some(x), Some(y), Some(z)] => Some(MoveSpec::Translate([x, y, z])),
                _ => None,
            }
        }
        other => {
            errs.0
                .push((FieldKey(MOVE_TYPE), format!("unknown option `{other}`")));
            None
        }
    };
    match spec {
        Some(spec) if errs.is_empty() => Ok(FeatureBox::new(Move { bodies, spec, copy })),
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.move",
        schema: move_schema,
        defaults: move_defaults,
        read: move_read,
        build: move_build,
        anchors: no_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 50,
            pinned: true,
            icon_svg: &[],
            shortcut: Some("M"),
            keywords: &["move", "copy", "translate", "rotate", "transform", "position"],
        },
        new_component: None,
    }
}

// ---- align ------------------------------------------------------------------------------------

const FROM: &str = "from";
const TO: &str = "to";
const TO_REF: &str = "to_ref";
const FLIP: &str = "flip";

const TARGETS: [(&str, &str); 7] = [
    ("xy", "XY plane"),
    ("yz", "YZ plane"),
    ("xz", "XZ plane"),
    ("x", "X axis"),
    ("y", "Y axis"),
    ("z", "Z axis"),
    ("picked", "Selected target"),
];

const ALIGN_FILTER: EntityFilter = EntityFilter::FACE
    .union(EntityFilter::PLANAR_FACE)
    .union(EntityFilter::EDGE)
    .union(EntityFilter::VERTEX)
    .union(EntityFilter::SKETCH_CURVE)
    .union(EntityFilter::SKETCH_POINT)
    .union(EntityFilter::CONSTRUCTION_PLANE)
    .union(EntityFilter::CONSTRUCTION_AXIS)
    .union(EntityFilter::CONSTRUCTION_POINT);

fn align_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut flip = FieldSpec::new(FLIP, "Flip", FieldKind::Bool);
        flip.tooltip =
            Some("Planes face the same way instead of each other; lines run the other way.".into());
        DialogSchema {
            kind: "solid.align",
            title: "Align".into(),
            help: Some("solid.align"),
            fields: vec![
                pick(BODIES, "Bodies to align", EntityFilter::BODY, None),
                pick(FROM, "From", ALIGN_FILTER, Some(1)),
                choice(TO, "To", &TARGETS),
                shown(
                    pick(TO_REF, "Target reference", ALIGN_FILTER, Some(1)),
                    is(TO, "picked"),
                ),
                flip,
                manip(expr(ANGLE, "Angle", Dim::ANGLE, ZERO_DEG), ManipKind::Angle),
            ],
        }
    })
}

fn align_values(a: &Align, design: &Design) -> FieldValues {
    let (to, to_ref) = axis_values(&a.to, "xy");
    FieldValues::default()
        .with(BODIES, bodies_value(&a.bodies))
        .with(
            FROM,
            FieldValue::Selection(
                a.from
                    .as_ref()
                    .and_then(RefInput::entity)
                    .cloned()
                    .into_iter()
                    .collect(),
            ),
        )
        .with(TO, FieldValue::Choice(to))
        .with(TO_REF, FieldValue::Selection(to_ref))
        .with(FLIP, FieldValue::Bool(a.flip))
        .with(ANGLE, FieldValue::from_expr(&a.angle, design))
}

fn align_defaults(init: &DialogInit<'_>) -> FieldValues {
    let a = Align {
        bodies: preselected_bodies(init),
        from: None,
        to: RefInput::Origin(crate::construct::OriginEntity::XYPlane),
        flip: false,
        angle: amg_doc::Expr::Inline(amg_doc::ExprSrc(ZERO_DEG.into())),
    };
    align_values(&a, init.design)
}

fn align_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<Align>()
        .map(|a| align_values(a, design))
}

fn align_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let bodies = read_bodies(v, &mut errs);
    let from = optional_ref(v, FROM, &mut errs).map(RefInput::Ref);
    let to = push(read_axis(v, TO, TO_REF, "xy"), &mut errs);
    let flip = push(v.flag(FLIP), &mut errs).unwrap_or(false);
    let angle = take_expr(v, ANGLE, &mut errs);
    match (to, angle) {
        (Some(to), Some(angle)) if errs.is_empty() => Ok(FeatureBox::new(Align {
            bodies,
            from,
            to,
            flip,
            angle,
        })),
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.align",
        schema: align_schema,
        defaults: align_defaults,
        read: align_read,
        build: align_build,
        anchors: no_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 51,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["align", "mate", "snap", "coplanar", "position"],
        },
        new_component: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::construct::OriginEntity;
    use amg_doc::{Document, FeatureId};

    fn body(origin: u64) -> BodyId {
        BodyId {
            origin: FeatureId(origin),
            slot: 0,
        }
    }

    #[test]
    fn every_move_type_round_trips() {
        let doc = Document::new("move");
        let edge = EntityRef::Body(body(9));
        let mut free = IDENTITY_MATRIX;
        free[1][3] = 4.0;
        for m in [
            Move::translate(vec![body(1)], ["1 mm", "2 mm", "3 mm"]),
            Move::rotate(
                vec![body(1)],
                RefInput::Origin(OriginEntity::XAxis),
                "30 deg",
            ),
            Move::rotate(vec![body(1)], RefInput::Ref(edge.clone()), "30 deg").copied(),
            Move {
                bodies: vec![body(1), body(2)],
                spec: MoveSpec::PointToPoint {
                    from: RefInput::Ref(edge.clone()),
                    to: RefInput::Ref(EntityRef::Body(body(8))),
                },
                copy: true,
            },
        ] {
            let v = move_values(&m, doc.design());
            let f = move_build(&v, None).unwrap();
            assert_eq!(f.get().as_any().downcast_ref::<Move>(), Some(&m));
            assert_eq!(move_read(f.get(), doc.design()).unwrap(), v);
        }
        // A free move keeps its matrix on edit; a new one starts at the identity.
        let m = Move {
            bodies: vec![body(1)],
            spec: MoveSpec::Free { matrix: free },
            copy: false,
        };
        let v = move_values(&m, doc.design());
        let f = move_build(&v, Some(&m)).unwrap();
        assert_eq!(f.get().as_any().downcast_ref::<Move>(), Some(&m));
        let f = move_build(&v, None).unwrap();
        let Some(Move {
            spec: MoveSpec::Free { matrix },
            ..
        }) = f.get().as_any().downcast_ref::<Move>()
        else {
            panic!("free")
        };
        assert_eq!(*matrix, IDENTITY_MATRIX);
        // Point to point without picks is a field error.
        let v = v.with(MOVE_TYPE, FieldValue::Choice("point"));
        let e = move_build(&v, None).unwrap_err();
        assert!(e.0.iter().any(|(k, _)| *k == FieldKey(FROM_POINT)));
    }

    #[test]
    fn every_align_target_round_trips() {
        let doc = Document::new("align");
        let face = EntityRef::Body(body(5));
        for to in [
            RefInput::Origin(OriginEntity::XYPlane),
            RefInput::Origin(OriginEntity::ZAxis),
            RefInput::Ref(face.clone()),
        ] {
            let mut a = Align::new(vec![body(1)], RefInput::Ref(face.clone()), to);
            a.flip = true;
            let v = align_values(&a, doc.design());
            let f = align_build(&v, None).unwrap();
            assert_eq!(f.get().as_any().downcast_ref::<Align>(), Some(&a));
            assert_eq!(align_read(f.get(), doc.design()).unwrap(), v);
        }
    }
}
