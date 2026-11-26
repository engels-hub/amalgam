//! Dialog registration of `solid.draft` (S11 §B schema, rendered by UI-05): neutral plane
//! (origin plane or a picked planar face / construction plane), faces, type, angles, flip,
//! tangent chain.

use std::sync::OnceLock;

use amg_doc::{Design, Dim, Feature, FeatureBox};

use super::{Draft, DraftSides};
use crate::dialog::{
    CommandMeta, DialogInit, DialogRegistration, DialogSchema, EntityFilter, FieldErrors, FieldKey,
    FieldKind, FieldSpec, FieldValue, FieldValues,
};
use crate::solid::pattern::dialog::{axis_values, read_axis};
use crate::solid::shell::dialog::{
    choice, expr_field, face_refs, faces_value, no_anchors, pick, preselected_faces, push, when,
};

const PLANE: &str = "plane";
const PLANE_REF: &str = "plane_ref";
const FACES: &str = "faces";
const TYPE: &str = "draft_type";
const ANGLE: &str = "angle";
const ANGLE2: &str = "angle2";
const FLIP: &str = "flip";
const TANGENT_CHAIN: &str = "tangent_chain";
const DEFAULT_ANGLE: &str = "3 deg";

const PLANES: [(&str, &str); 4] = [
    ("xy", "XY plane"),
    ("yz", "YZ plane"),
    ("xz", "XZ plane"),
    ("picked", "Selected plane"),
];
const TYPES: [(&str, &str); 3] = [
    ("one_side", "One side"),
    ("two_sides", "Two sides"),
    ("symmetric", "Symmetric"),
];

fn schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut plane_ref = pick(
            PLANE_REF,
            "Plane reference",
            EntityFilter::PLANAR_FACE | EntityFilter::CONSTRUCTION_PLANE,
            1,
            Some(1),
        );
        plane_ref.visible_if = Some(when(PLANE, &["picked"]));
        let mut faces = pick(FACES, "Faces", EntityFilter::FACE, 1, None);
        faces.tooltip = Some("Planar, cylindrical or conical faces to tilt.".into());
        let mut angle2 = expr_field(ANGLE2, "Second angle", Dim::ANGLE, DEFAULT_ANGLE);
        angle2.visible_if = Some(when(TYPE, &["two_sides"]));
        let mut chain = FieldSpec::new(TANGENT_CHAIN, "Tangent chain", FieldKind::Bool);
        chain.tooltip = Some("Also draft the faces tangent to the picked ones.".into());
        DialogSchema {
            kind: "solid.draft",
            title: "Draft".into(),
            help: Some("solid.draft"),
            fields: vec![
                choice(PLANE, "Pull plane", &PLANES),
                plane_ref,
                faces,
                choice(TYPE, "Draft type", &TYPES),
                expr_field(ANGLE, "Angle", Dim::ANGLE, DEFAULT_ANGLE),
                angle2,
                FieldSpec::new(FLIP, "Flip pull direction", FieldKind::FlipDirection),
                chain,
            ],
        }
    })
}

fn values(d: &Draft, design: &Design) -> FieldValues {
    let (plane, sel) = axis_values(&d.plane, "xy");
    let (ty, angle2) = match &d.sides {
        DraftSides::OneSide => ("one_side", FieldValue::from_expr(&d.angle, design)),
        DraftSides::TwoSides { angle2 } => ("two_sides", FieldValue::from_expr(angle2, design)),
        DraftSides::Symmetric => ("symmetric", FieldValue::from_expr(&d.angle, design)),
    };
    FieldValues::default()
        .with(PLANE, FieldValue::Choice(plane))
        .with(PLANE_REF, FieldValue::Selection(sel))
        .with(FACES, faces_value(&d.faces))
        .with(TYPE, FieldValue::Choice(ty))
        .with(ANGLE, FieldValue::from_expr(&d.angle, design))
        .with(ANGLE2, angle2)
        .with(FLIP, FieldValue::Flip(d.flip))
        .with(TANGENT_CHAIN, FieldValue::Bool(d.tangent_chain))
}

fn defaults(init: &DialogInit<'_>) -> FieldValues {
    let d = Draft {
        faces: preselected_faces(init),
        ..Draft::default()
    };
    values(&d, init.design)
}

fn read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<Draft>()
        .map(|d| values(d, design))
}

fn flip(v: &FieldValues) -> Result<bool, (FieldKey, String)> {
    match v.get(FLIP) {
        Some(FieldValue::Flip(b)) => Ok(*b),
        None => Ok(false),
        Some(_) => Err((FieldKey(FLIP), "expected a flip toggle".into())),
    }
}

fn build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let plane = push(read_axis(v, PLANE, PLANE_REF, "xy"), &mut errs);
    let faces = face_refs(v, FACES, &mut errs);
    let angle = push(v.expr(ANGLE), &mut errs);
    let sides = match push(v.choice(TYPE, "one_side"), &mut errs) {
        Some("two_sides") => push(v.expr(ANGLE2), &mut errs)
            .map(|angle2| DraftSides::TwoSides { angle2 })
            .unwrap_or_default(),
        Some("symmetric") => DraftSides::Symmetric,
        Some("one_side") | None => DraftSides::OneSide,
        Some(other) => {
            errs.0
                .push((FieldKey(TYPE), format!("unknown option `{other}`")));
            DraftSides::OneSide
        }
    };
    let flip = push(flip(v), &mut errs).unwrap_or(false);
    let chain = push(v.flag(TANGENT_CHAIN), &mut errs).unwrap_or(true);
    match (plane, angle) {
        (Some(plane), Some(angle)) if errs.is_empty() => Ok(FeatureBox::new(Draft {
            plane,
            faces,
            angle,
            flip,
            sides,
            tangent_chain: chain,
        })),
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.draft",
        schema,
        defaults,
        read,
        build,
        anchors: no_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 25,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["draft", "taper", "angle", "mould", "mold"],
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
    fn every_type_round_trips() {
        let doc = Document::new("draft");
        for sides in [
            DraftSides::OneSide,
            DraftSides::TwoSides {
                angle2: inline("5 deg"),
            },
            DraftSides::Symmetric,
        ] {
            let d = Draft {
                sides,
                flip: true,
                ..Draft::default()
            };
            let v = values(&d, doc.design());
            let f = build(&v, None).unwrap();
            assert_eq!(f.get().as_any().downcast_ref::<Draft>().unwrap(), &d);
            assert_eq!(read(f.get(), doc.design()).unwrap(), v);
        }
    }
}
