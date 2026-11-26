//! Dialog registration of `solid.scale` (S11 §B schema, rendered by UI-05): bodies, point
//! (origin or a picked vertex / sketch / construction point), uniform or per-axis factors.

use std::sync::OnceLock;

use amg_doc::{Design, Dim, Feature, FeatureBox};

use super::{Scale, ScaleKind};
use crate::construct::{OriginEntity, RefInput};
use crate::dialog::{
    CommandMeta, DialogInit, DialogRegistration, DialogSchema, EntityFilter, FieldErrors, FieldKey,
    FieldValue, FieldValues,
};
use crate::solid::shell::dialog::{
    bodies_value, body_ids, choice, expr_field, no_anchors, pick, preselected_bodies, push, when,
};

const BODIES: &str = "bodies";
const POINT: &str = "point";
const POINT_REF: &str = "point_ref";
const TYPE: &str = "scale_type";
const SCALE: &str = "scale";
const SX: &str = "scale_x";
const SY: &str = "scale_y";
const SZ: &str = "scale_z";
const ONE: &str = "1";

const POINTS: [(&str, &str); 2] = [("origin", "Origin"), ("picked", "Selected point")];
const TYPES: [(&str, &str); 2] = [("uniform", "Uniform"), ("non_uniform", "Non-uniform")];

fn schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut point_ref = pick(
            POINT_REF,
            "Point reference",
            EntityFilter::VERTEX | EntityFilter::SKETCH_POINT | EntityFilter::CONSTRUCTION_POINT,
            1,
            Some(1),
        );
        point_ref.visible_if = Some(when(POINT, &["picked"]));
        let mut uniform = expr_field(SCALE, "Scale factor", Dim::NONE, ONE);
        uniform.visible_if = Some(when(TYPE, &["uniform"]));
        let axis = |key, label| {
            let mut f = expr_field(key, label, Dim::NONE, ONE);
            f.visible_if = Some(when(TYPE, &["non_uniform"]));
            f
        };
        DialogSchema {
            kind: "solid.scale",
            title: "Scale".into(),
            help: Some("solid.scale"),
            fields: vec![
                pick(BODIES, "Bodies", EntityFilter::BODY, 1, None),
                choice(POINT, "Point", &POINTS),
                point_ref,
                choice(TYPE, "Scale type", &TYPES),
                uniform,
                axis(SX, "X factor"),
                axis(SY, "Y factor"),
                axis(SZ, "Z factor"),
            ],
        }
    })
}

fn values(s: &Scale, design: &Design) -> FieldValues {
    let (point, sel) = match &s.point {
        RefInput::Ref(e) => ("picked", vec![e.clone()]),
        RefInput::Origin(_) => ("origin", Vec::new()),
    };
    let one = || FieldValue::expr(ONE);
    let (ty, u, [x, y, z]) = match &s.kind {
        ScaleKind::Uniform(e) => (
            "uniform",
            FieldValue::from_expr(e, design),
            [one(), one(), one()],
        ),
        ScaleKind::NonUniform(v) => (
            "non_uniform",
            one(),
            [0, 1, 2].map(|i| FieldValue::from_expr(&v[i], design)),
        ),
    };
    FieldValues::default()
        .with(BODIES, bodies_value(&s.bodies))
        .with(POINT, FieldValue::Choice(point))
        .with(POINT_REF, FieldValue::Selection(sel))
        .with(TYPE, FieldValue::Choice(ty))
        .with(SCALE, u)
        .with(SX, x)
        .with(SY, y)
        .with(SZ, z)
}

fn defaults(init: &DialogInit<'_>) -> FieldValues {
    values(&Scale::uniform(preselected_bodies(init), ONE), init.design)
}

fn read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<Scale>()
        .map(|s| values(s, design))
}

fn read_point(v: &FieldValues) -> Result<RefInput, (FieldKey, String)> {
    match v.choice(POINT, "origin")? {
        "origin" => Ok(RefInput::Origin(OriginEntity::Point)),
        "picked" => v
            .selection(POINT_REF)?
            .first()
            .cloned()
            .map(RefInput::Ref)
            .ok_or_else(|| (FieldKey(POINT_REF), "select a point".into())),
        other => Err((FieldKey(POINT), format!("unknown option `{other}`"))),
    }
}

fn build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let bodies = body_ids(v, BODIES, &mut errs);
    let point = push(read_point(v), &mut errs);
    let kind = match push(v.choice(TYPE, "uniform"), &mut errs) {
        Some("non_uniform") => {
            let x = push(v.expr(SX), &mut errs);
            let y = push(v.expr(SY), &mut errs);
            let z = push(v.expr(SZ), &mut errs);
            match (x, y, z) {
                (Some(x), Some(y), Some(z)) => Some(ScaleKind::NonUniform([x, y, z])),
                _ => None,
            }
        }
        Some("uniform") | None => push(v.expr(SCALE), &mut errs).map(ScaleKind::Uniform),
        Some(other) => {
            errs.0
                .push((FieldKey(TYPE), format!("unknown option `{other}`")));
            None
        }
    };
    match (point, kind) {
        (Some(point), Some(kind)) if errs.is_empty() => Ok(FeatureBox::new(Scale {
            bodies,
            point,
            kind,
        })),
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.scale",
        schema,
        defaults,
        read,
        build,
        anchors: no_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 26,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["scale", "resize", "grow", "shrink", "stretch"],
        },
        new_component: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_doc::{BodyId, Document, EntityRef, FeatureId};

    #[test]
    fn uniform_and_non_uniform_round_trip() {
        let doc = Document::new("scale");
        let b = BodyId {
            origin: FeatureId(3),
            slot: 0,
        };
        let mut picked = Scale::non_uniform(vec![b], ["1", "2", "3"]);
        picked.point = RefInput::Ref(EntityRef::Body(b));
        for s in [Scale::uniform(vec![b], "2"), picked] {
            let v = values(&s, doc.design());
            let f = build(&v, None).unwrap();
            assert_eq!(f.get().as_any().downcast_ref::<Scale>().unwrap(), &s);
            assert_eq!(read(f.get(), doc.design()).unwrap(), v);
        }
    }
}
