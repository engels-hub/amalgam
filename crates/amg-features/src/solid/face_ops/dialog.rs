//! Dialog registrations of `solid.offset_face`, `solid.replace_face` and `solid.delete_face`
//! (S11 §B schemas, rendered by UI-05), and the press-pull command (`solid.press_pull`, a
//! [`DispatchRegistration`]): a face selection opens Offset Face, an edge selection Fillet, a
//! profile selection Extrude (S04 §9).

use std::sync::OnceLock;

use amg_doc::{Design, Dim, EntityRef, Evaluated, Feature, FeatureBox, TopoKind, TopoRef};

use super::{DeleteFace, OffsetFace, ReplaceFace};
use crate::construct::{OriginEntity, RefInput};
use crate::dialog::{
    CommandMeta, DialogInit, DialogRegistration, DialogSchema, DispatchRegistration, EntityFilter,
    FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue, FieldValues, ManipAnchor, ManipKind,
};
use crate::solid::extrude::dialog::{choice, expr, is, take_expr};
use crate::solid::pattern::dialog::{axis_values, read_axis};

const FACES: &str = "faces";
const DISTANCE: &str = "distance";
const WITH: &str = "with";
const WITH_REF: &str = "with_ref";
const TANGENT_CHAIN: &str = "tangent_chain";
const HEAL: &str = "heal";

const DEFAULT_DISTANCE: &str = "1 mm";

const SURFACES: [(&str, &str); 4] = [
    ("xy", "XY plane"),
    ("yz", "YZ plane"),
    ("xz", "XZ plane"),
    ("picked", "Selected surface"),
];

fn faces_field(label: &str) -> FieldSpec {
    FieldSpec::new(
        FACES,
        label,
        FieldKind::Selection {
            filter: EntityFilter::FACE,
            min: 1,
            max: None,
        },
    )
}

fn faces_value(faces: &[TopoRef]) -> FieldValue {
    FieldValue::Selection(faces.iter().cloned().map(EntityRef::Topo).collect())
}

fn preselected_faces(init: &DialogInit<'_>) -> Vec<TopoRef> {
    init.preselection
        .iter()
        .filter_map(|e| match e {
            EntityRef::Topo(t) if t.kind == TopoKind::Face => Some(t.clone()),
            _ => None,
        })
        .collect()
}

fn read_faces(v: &FieldValues, errs: &mut FieldErrors) -> Vec<TopoRef> {
    let mut out = Vec::new();
    match v.selection(FACES) {
        Ok(sel) => {
            for e in sel {
                match e {
                    EntityRef::Topo(t) if t.kind == TopoKind::Face => out.push(t.clone()),
                    _ => errs.0.push((FieldKey(FACES), "select faces".into())),
                }
            }
        }
        Err(e) => errs.0.push(e),
    }
    out
}

fn flag(v: &FieldValues, key: &'static str, errs: &mut FieldErrors) -> bool {
    v.flag(key).unwrap_or_else(|e| {
        errs.0.push(e);
        false
    })
}

fn no_anchors(_f: &dyn Feature, _design: &Design, _eval: &Evaluated) -> Vec<ManipAnchor> {
    Vec::new()
}

const fn modify(order: i32, keywords: &'static [&'static str]) -> CommandMeta {
    CommandMeta {
        workspace: "design",
        tab: "solid",
        group: "modify",
        order,
        pinned: false,
        icon_svg: &[],
        shortcut: None,
        keywords,
    }
}

// ---- offset face ------------------------------------------------------------------------------

fn offset_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut d = expr(DISTANCE, "Distance", Dim::LENGTH, DEFAULT_DISTANCE);
        d.manipulator = Some(ManipKind::Distance);
        d.tooltip = Some("Along the outward normal; negative moves the faces inwards.".into());
        DialogSchema {
            kind: "solid.offset_face",
            title: "Offset Face".into(),
            help: Some("solid.offset_face"),
            fields: vec![faces_field("Faces to offset"), d],
        }
    })
}

fn offset_values(o: &OffsetFace, design: &Design) -> FieldValues {
    FieldValues::default()
        .with(FACES, faces_value(&o.faces))
        .with(DISTANCE, FieldValue::from_expr(&o.distance, design))
}

fn offset_defaults(init: &DialogInit<'_>) -> FieldValues {
    offset_values(
        &OffsetFace::new(preselected_faces(init), DEFAULT_DISTANCE),
        init.design,
    )
}

fn offset_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<OffsetFace>()
        .map(|o| offset_values(o, design))
}

fn offset_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let faces = read_faces(v, &mut errs);
    let distance = take_expr(v, DISTANCE, &mut errs);
    match distance {
        Some(distance) if errs.is_empty() => Ok(FeatureBox::new(OffsetFace { faces, distance })),
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.offset_face",
        schema: offset_schema,
        defaults: offset_defaults,
        read: offset_read,
        build: offset_build,
        anchors: no_anchors,
        command: modify(53, &["offset", "face", "push", "pull", "move face"]),
        new_component: None,
    }
}

// ---- replace face -----------------------------------------------------------------------------

fn replace_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut r = FieldSpec::new(
            WITH_REF,
            "Surface reference",
            FieldKind::Selection {
                filter: EntityFilter::FACE
                    | EntityFilter::PLANAR_FACE
                    | EntityFilter::CONSTRUCTION_PLANE,
                min: 1,
                max: Some(1),
            },
        );
        r.visible_if = Some(is(WITH, "picked"));
        let mut chain = FieldSpec::new(TANGENT_CHAIN, "Tangent chain", FieldKind::Bool);
        chain.tooltip = Some("Also replace the faces tangent to the selected ones.".into());
        DialogSchema {
            kind: "solid.replace_face",
            title: "Replace Face".into(),
            help: Some("solid.replace_face"),
            fields: vec![
                faces_field("Faces to replace"),
                choice(WITH, "New surface", &SURFACES),
                r,
                chain,
            ],
        }
    })
}

fn replace_values(r: &ReplaceFace) -> FieldValues {
    let (with, sel) = axis_values(&r.with, "xy");
    FieldValues::default()
        .with(FACES, faces_value(&r.faces))
        .with(WITH, FieldValue::Choice(with))
        .with(WITH_REF, FieldValue::Selection(sel))
        .with(TANGENT_CHAIN, FieldValue::Bool(r.tangent_chain))
}

fn replace_defaults(init: &DialogInit<'_>) -> FieldValues {
    replace_values(&ReplaceFace::new(
        preselected_faces(init),
        RefInput::Origin(OriginEntity::XYPlane),
    ))
}

fn replace_read(f: &dyn Feature, _design: &Design) -> Option<FieldValues> {
    f.as_any().downcast_ref::<ReplaceFace>().map(replace_values)
}

fn replace_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let faces = read_faces(v, &mut errs);
    let with = read_axis(v, WITH, WITH_REF, "xy")
        .map_err(|e| errs.0.push(e))
        .ok();
    let tangent_chain = flag(v, TANGENT_CHAIN, &mut errs);
    match with {
        Some(with) if errs.is_empty() => Ok(FeatureBox::new(ReplaceFace {
            faces,
            with,
            tangent_chain,
        })),
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.replace_face",
        schema: replace_schema,
        defaults: replace_defaults,
        read: replace_read,
        build: replace_build,
        anchors: no_anchors,
        command: modify(54, &["replace", "face", "surface", "substitute"]),
        new_component: None,
    }
}

// ---- delete face ------------------------------------------------------------------------------

fn delete_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut heal = FieldSpec::new(HEAL, "Heal", FieldKind::Bool);
        heal.tooltip = Some(
            "Close the gap by extending the neighbouring faces. Off: remove the faces and leave \
             an open surface body."
                .into(),
        );
        DialogSchema {
            kind: "solid.delete_face",
            title: "Delete Face".into(),
            help: Some("solid.delete_face"),
            fields: vec![faces_field("Faces to delete"), heal],
        }
    })
}

fn delete_values(d: &DeleteFace) -> FieldValues {
    FieldValues::default()
        .with(FACES, faces_value(&d.faces))
        .with(HEAL, FieldValue::Bool(d.heal))
}

fn delete_defaults(init: &DialogInit<'_>) -> FieldValues {
    delete_values(&DeleteFace::new(preselected_faces(init), true))
}

fn delete_read(f: &dyn Feature, _design: &Design) -> Option<FieldValues> {
    f.as_any().downcast_ref::<DeleteFace>().map(delete_values)
}

fn delete_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let faces = read_faces(v, &mut errs);
    let heal = flag(v, HEAL, &mut errs);
    if errs.is_empty() {
        Ok(FeatureBox::new(DeleteFace { faces, heal }))
    } else {
        Err(errs)
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.delete_face",
        schema: delete_schema,
        defaults: delete_defaults,
        read: delete_read,
        build: delete_build,
        anchors: no_anchors,
        command: modify(55, &["delete", "remove", "face", "defeature", "heal"]),
        new_component: None,
    }
}

// ---- press-pull -------------------------------------------------------------------------------

/// The kind whose dialog press-pull opens for `selection` (S04 §9): faces → Offset Face,
/// edges → Fillet, sketch profiles → Extrude. A mixed or empty selection (or anything else)
/// disables the command (`None`).
pub fn press_pull(selection: &[EntityRef], _design: &Design) -> Option<&'static str> {
    let class = |e: &EntityRef| -> Option<&'static str> {
        match e {
            EntityRef::Topo(t) if t.kind == TopoKind::Face => Some("solid.offset_face"),
            EntityRef::Topo(t) if t.kind == TopoKind::Edge => Some("solid.fillet"),
            EntityRef::Profile { .. } => Some("solid.extrude"),
            _ => None,
        }
    };
    let (first, rest) = selection.split_first()?;
    let kind = class(first)?;
    rest.iter().all(|e| class(e) == Some(kind)).then_some(kind)
}

inventory::submit! {
    DispatchRegistration {
        command_id: "solid.press_pull",
        dispatch: press_pull,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 10,
            pinned: true,
            icon_svg: &[],
            shortcut: Some("Q"),
            keywords: &["press", "pull", "push", "offset", "fillet", "extrude"],
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_doc::{BodyId, Document, FeatureId};

    fn face(key: u32, kind: TopoKind) -> TopoRef {
        TopoRef {
            body: BodyId {
                origin: FeatureId(1),
                slot: 0,
            },
            kind,
            name: amg_doc::naming::TopoName::default(),
            fingerprint: None,
            key,
        }
    }

    #[test]
    fn the_three_dialogs_round_trip_with_preselected_faces() {
        let doc = Document::new("faces");
        let pre = [
            EntityRef::Topo(face(1, TopoKind::Face)),
            EntityRef::Topo(face(2, TopoKind::Edge)),
        ];
        let init = DialogInit {
            preselection: &pre,
            design: doc.design(),
            evaluated: doc.evaluated(),
        };
        let v = offset_defaults(&init);
        let f = offset_build(&v, None).unwrap();
        let o = f.get().as_any().downcast_ref::<OffsetFace>().unwrap();
        assert_eq!(
            o.faces,
            vec![face(1, TopoKind::Face)],
            "edges are not faces"
        );
        assert_eq!(offset_read(f.get(), doc.design()).unwrap(), v);
        for r in [
            ReplaceFace::new(
                vec![face(1, TopoKind::Face)],
                RefInput::Origin(OriginEntity::YZPlane),
            ),
            ReplaceFace {
                faces: vec![face(1, TopoKind::Face)],
                with: RefInput::Ref(EntityRef::Topo(face(3, TopoKind::Face))),
                tangent_chain: true,
            },
        ] {
            let v = replace_values(&r);
            let f = replace_build(&v, None).unwrap();
            assert_eq!(f.get().as_any().downcast_ref::<ReplaceFace>(), Some(&r));
            assert_eq!(replace_read(f.get(), doc.design()).unwrap(), v);
        }
        for heal in [true, false] {
            let d = DeleteFace::new(vec![face(4, TopoKind::Face)], heal);
            let v = delete_values(&d);
            let f = delete_build(&v, None).unwrap();
            assert_eq!(f.get().as_any().downcast_ref::<DeleteFace>(), Some(&d));
        }
        let bad = delete_values(&DeleteFace::new(vec![], true)).with(
            FACES,
            FieldValue::Selection(vec![EntityRef::Topo(face(5, TopoKind::Edge))]),
        );
        assert_eq!(
            delete_build(&bad, None).unwrap_err().0[0].0,
            FieldKey(FACES)
        );
    }

    #[test]
    fn press_pull_needs_one_kind_of_selection() {
        let doc = Document::new("pp");
        let f = EntityRef::Topo(face(1, TopoKind::Face));
        let e = EntityRef::Topo(face(2, TopoKind::Edge));
        assert_eq!(press_pull(&[], doc.design()), None);
        assert_eq!(
            press_pull(&[f.clone(), f.clone()], doc.design()),
            Some("solid.offset_face")
        );
        assert_eq!(
            press_pull(std::slice::from_ref(&e), doc.design()),
            Some("solid.fillet")
        );
        assert_eq!(press_pull(&[f, e], doc.design()), None, "mixed");
        assert_eq!(
            press_pull(
                &[EntityRef::Body(BodyId {
                    origin: FeatureId(1),
                    slot: 0
                })],
                doc.design()
            ),
            None
        );
    }
}
