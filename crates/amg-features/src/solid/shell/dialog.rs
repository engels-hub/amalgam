//! Dialog registrations of `solid.shell` and `solid.thicken` (S11 §B schema, rendered by
//! UI-05), plus small field helpers shared with the draft and scale dialogs.

use std::sync::OnceLock;

use amg_doc::{BodyId, Design, Dim, EntityRef, Evaluated, Feature, FeatureBox, TopoKind, TopoRef};

use super::{Shell, ShellDirection, Thicken};
use crate::dialog::{
    ChoiceOption, ChoiceStyle, CommandMeta, Condition, DialogInit, DialogRegistration,
    DialogSchema, EntityFilter, FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue,
    FieldValues, ManipAnchor, ManipKind,
};
use crate::solid::common::{BodyOp, OpTarget};
use crate::solid::extent::ThinSide;

// ---- shared helpers -------------------------------------------------------------------------

pub(crate) fn choice(
    key: &'static str,
    label: &str,
    options: &[(&'static str, &str)],
) -> FieldSpec {
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

pub(crate) fn when(key: &'static str, ids: &[&'static str]) -> Condition {
    Condition::Any(
        ids.iter()
            .map(|id| Condition::Is(FieldKey(key), FieldValue::Choice(id)))
            .collect(),
    )
}

pub(crate) fn expr_field(
    key: &'static str,
    label: &str,
    dim: Dim,
    default: &'static str,
) -> FieldSpec {
    let mut f = FieldSpec::new(key, label, FieldKind::Expr { dim, default });
    if dim == Dim::LENGTH {
        f.manipulator = Some(ManipKind::Distance);
    }
    f
}

pub(crate) fn pick(
    key: &'static str,
    label: &str,
    filter: EntityFilter,
    min: u32,
    max: Option<u32>,
) -> FieldSpec {
    FieldSpec::new(key, label, FieldKind::Selection { filter, min, max })
}

pub(crate) fn push<T>(r: Result<T, (FieldKey, String)>, errs: &mut FieldErrors) -> Option<T> {
    match r {
        Ok(x) => Some(x),
        Err(e) => {
            errs.0.push(e);
            None
        }
    }
}

pub(crate) fn face_refs(
    v: &FieldValues,
    key: &'static str,
    errs: &mut FieldErrors,
) -> Vec<TopoRef> {
    let mut out = Vec::new();
    if let Some(sel) = push(v.selection(key), errs) {
        for e in sel {
            match e {
                EntityRef::Topo(t) if t.kind == TopoKind::Face => out.push(t.clone()),
                _ => errs.0.push((FieldKey(key), "select faces".into())),
            }
        }
    }
    out
}

pub(crate) fn body_ids(v: &FieldValues, key: &'static str, errs: &mut FieldErrors) -> Vec<BodyId> {
    let mut out = Vec::new();
    if let Some(sel) = push(v.selection(key), errs) {
        for e in sel {
            match e {
                EntityRef::Body(b) => out.push(*b),
                _ => errs.0.push((FieldKey(key), "select bodies".into())),
            }
        }
    }
    out
}

pub(crate) fn faces_value(faces: &[TopoRef]) -> FieldValue {
    FieldValue::Selection(faces.iter().cloned().map(EntityRef::Topo).collect())
}

pub(crate) fn bodies_value(bodies: &[BodyId]) -> FieldValue {
    FieldValue::Selection(bodies.iter().copied().map(EntityRef::Body).collect())
}

pub(crate) fn preselected_faces(init: &DialogInit<'_>) -> Vec<TopoRef> {
    init.preselection
        .iter()
        .filter_map(|e| match e {
            EntityRef::Topo(t) if t.kind == TopoKind::Face => Some(t.clone()),
            _ => None,
        })
        .collect()
}

pub(crate) fn preselected_bodies(init: &DialogInit<'_>) -> Vec<BodyId> {
    init.preselection
        .iter()
        .filter_map(|e| match e {
            EntityRef::Body(b) => Some(*b),
            _ => None,
        })
        .collect()
}

pub(crate) fn no_anchors(_f: &dyn Feature, _d: &Design, _e: &Evaluated) -> Vec<ManipAnchor> {
    Vec::new()
}

// ---- shell ----------------------------------------------------------------------------------

const FACES: &str = "faces";
const BODIES: &str = "bodies";
const TANGENT_CHAIN: &str = "tangent_chain";
const DIRECTION: &str = "direction";
const INSIDE: &str = "inside";
const OUTSIDE: &str = "outside";
const DEFAULT_THICKNESS: &str = "1 mm";

const DIRECTIONS: [(&str, ShellDirection, &str); 3] = [
    ("inside", ShellDirection::Inside, "Inside"),
    ("outside", ShellDirection::Outside, "Outside"),
    ("both", ShellDirection::Both, "Both"),
];

fn shell_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut faces = pick(FACES, "Faces to remove", EntityFilter::FACE, 0, None);
        faces.tooltip = Some("Open faces. Pick a body instead to hollow it.".into());
        let mut bodies = pick(BODIES, "Bodies to hollow", EntityFilter::BODY, 0, None);
        bodies.tooltip = Some("Bodies shelled with a closed inner void.".into());
        let mut chain = FieldSpec::new(TANGENT_CHAIN, "Tangent chain", FieldKind::Bool);
        chain.tooltip = Some("Also remove the curved faces tangent to the picked ones.".into());
        let opts: Vec<(&'static str, &str)> = DIRECTIONS.iter().map(|(i, _, l)| (*i, *l)).collect();
        let mut inside = expr_field(INSIDE, "Inside thickness", Dim::LENGTH, DEFAULT_THICKNESS);
        inside.visible_if = Some(when(DIRECTION, &["inside", "both"]));
        let mut outside = expr_field(OUTSIDE, "Outside thickness", Dim::LENGTH, DEFAULT_THICKNESS);
        outside.visible_if = Some(when(DIRECTION, &["outside", "both"]));
        DialogSchema {
            kind: "solid.shell",
            title: "Shell".into(),
            help: Some("solid.shell"),
            fields: vec![
                faces,
                bodies,
                chain,
                choice(DIRECTION, "Direction", &opts),
                inside,
                outside,
            ],
        }
    })
}

fn shell_values(s: &Shell, design: &Design) -> FieldValues {
    let dir = DIRECTIONS
        .iter()
        .find(|(_, d, _)| *d == s.direction)
        .map_or("inside", |(i, _, _)| *i);
    FieldValues::default()
        .with(FACES, faces_value(&s.remove))
        .with(BODIES, bodies_value(&s.bodies))
        .with(TANGENT_CHAIN, FieldValue::Bool(s.tangent_chain))
        .with(DIRECTION, FieldValue::Choice(dir))
        .with(INSIDE, FieldValue::from_expr(&s.inside, design))
        .with(OUTSIDE, FieldValue::from_expr(&s.outside, design))
}

fn shell_defaults(init: &DialogInit<'_>) -> FieldValues {
    let mut s = Shell::inside(preselected_faces(init), DEFAULT_THICKNESS);
    s.bodies = preselected_bodies(init);
    shell_values(&s, init.design)
}

fn shell_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<Shell>()
        .map(|s| shell_values(s, design))
}

fn shell_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let remove = face_refs(v, FACES, &mut errs);
    let bodies = body_ids(v, BODIES, &mut errs);
    let chain = push(v.flag(TANGENT_CHAIN), &mut errs).unwrap_or(true);
    let direction = match push(v.choice(DIRECTION, "inside"), &mut errs) {
        Some(id) => match DIRECTIONS.iter().find(|(i, _, _)| *i == id) {
            Some((_, d, _)) => *d,
            None => {
                errs.0
                    .push((FieldKey(DIRECTION), format!("unknown option `{id}`")));
                ShellDirection::Inside
            }
        },
        None => ShellDirection::Inside,
    };
    let inside = push(v.expr(INSIDE), &mut errs);
    let outside = push(v.expr(OUTSIDE), &mut errs);
    match (inside, outside) {
        (Some(inside), Some(outside)) if errs.is_empty() => Ok(FeatureBox::new(Shell {
            bodies,
            remove,
            inside,
            outside,
            direction,
            tangent_chain: chain,
        })),
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.shell",
        schema: shell_schema,
        defaults: shell_defaults,
        read: shell_read,
        build: shell_build,
        anchors: no_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 24,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["shell", "hollow", "wall", "thickness"],
        },
        new_component: None,
    }
}

// ---- thicken --------------------------------------------------------------------------------

const THICKNESS: &str = "thickness";
const SIDE: &str = "side";
const OPERATION: &str = "operation";
const PARTICIPANTS: &str = "participants";

const SIDES: [(&str, ThinSide, &str); 3] = [
    ("side1", ThinSide::Side1, "Along the normal"),
    ("side2", ThinSide::Side2, "Against the normal"),
    ("center", ThinSide::Center, "Symmetric"),
];

const OPS: [(&str, BodyOp, &str); 4] = [
    ("new_body", BodyOp::NewBody, "New body"),
    ("join", BodyOp::Join, "Join"),
    ("cut", BodyOp::Cut, "Cut"),
    ("intersect", BodyOp::Intersect, "Intersect"),
];

fn thicken_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let sides: Vec<(&'static str, &str)> = SIDES.iter().map(|(i, _, l)| (*i, *l)).collect();
        let ops: Vec<(&'static str, &str)> = OPS.iter().map(|(i, _, l)| (*i, *l)).collect();
        let mut parts = pick(PARTICIPANTS, "Objects", EntityFilter::BODY, 0, None);
        parts.visible_if = Some(when(OPERATION, &["join", "cut", "intersect"]));
        DialogSchema {
            kind: "solid.thicken",
            title: "Thicken".into(),
            help: Some("solid.thicken"),
            fields: vec![
                pick(FACES, "Faces", EntityFilter::FACE, 1, None),
                expr_field(THICKNESS, "Thickness", Dim::LENGTH, DEFAULT_THICKNESS),
                choice(SIDE, "Direction", &sides),
                choice(OPERATION, "Operation", &ops),
                parts,
            ],
        }
    })
}

fn thicken_values(t: &Thicken, design: &Design) -> FieldValues {
    let side = SIDES
        .iter()
        .find(|(_, s, _)| *s == t.side)
        .map_or("side1", |(i, _, _)| *i);
    let op = OPS
        .iter()
        .find(|(_, o, _)| *o == t.target.op)
        .map_or("new_body", |(i, _, _)| *i);
    FieldValues::default()
        .with(FACES, faces_value(&t.faces))
        .with(THICKNESS, FieldValue::from_expr(&t.thickness, design))
        .with(SIDE, FieldValue::Choice(side))
        .with(OPERATION, FieldValue::Choice(op))
        .with(PARTICIPANTS, bodies_value(&t.target.participants))
}

fn thicken_defaults(init: &DialogInit<'_>) -> FieldValues {
    thicken_values(
        &Thicken::new_body(preselected_faces(init), DEFAULT_THICKNESS),
        init.design,
    )
}

fn thicken_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<Thicken>()
        .map(|t| thicken_values(t, design))
}

fn thicken_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let faces = face_refs(v, FACES, &mut errs);
    let thickness = push(v.expr(THICKNESS), &mut errs);
    let side = push(v.choice(SIDE, "side1"), &mut errs)
        .and_then(|id| SIDES.iter().find(|(i, _, _)| *i == id))
        .map_or(ThinSide::Side1, |(_, s, _)| *s);
    let op = push(v.choice(OPERATION, "new_body"), &mut errs)
        .and_then(|id| OPS.iter().find(|(i, _, _)| *i == id))
        .map_or(BodyOp::NewBody, |(_, o, _)| *o);
    let participants = body_ids(v, PARTICIPANTS, &mut errs);
    match thickness {
        Some(thickness) if errs.is_empty() => Ok(FeatureBox::new(Thicken {
            faces,
            thickness,
            side,
            target: OpTarget { op, participants },
        })),
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.thicken",
        schema: thicken_schema,
        defaults: thicken_defaults,
        read: thicken_read,
        build: thicken_build,
        anchors: no_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 27,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["thicken", "offset", "surface", "solidify"],
        },
        new_component: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid::shell::inline;
    use amg_doc::Document;

    #[test]
    fn shell_and_thicken_round_trip_every_choice() {
        let doc = Document::new("sol06");
        for (id, d, _) in DIRECTIONS {
            let mut s = Shell::inside(Vec::new(), "2 mm");
            s.direction = d;
            s.outside = inline("3 mm");
            let v = shell_values(&s, doc.design());
            assert_eq!(v.get(DIRECTION), Some(&FieldValue::Choice(id)));
            let f = shell_build(&v, None).unwrap();
            assert_eq!(shell_read(f.get(), doc.design()).unwrap(), v);
        }
        for (_, side, _) in SIDES {
            for (_, op, _) in OPS {
                let mut t = Thicken::new_body(Vec::new(), "1 mm");
                t.side = side;
                t.target.op = op;
                let v = thicken_values(&t, doc.design());
                let f = thicken_build(&v, None).unwrap();
                assert_eq!(thicken_read(f.get(), doc.design()).unwrap(), v);
            }
        }
    }
}
