//! Dialog registrations of `solid.split_body` and `solid.split_face` (S11 §B schema, rendered
//! by UI-05). A tool is an origin plane choice or "Selected tool" with a pick shown for the
//! latter (as the mirror plane, SOL-08).

use std::sync::OnceLock;

use amg_doc::{BodyId, Design, EntityRef, Evaluated, Feature, FeatureBox, TopoRef};

use super::{SplitBody, SplitFace, SplitFaceTool};
use crate::construct::{OriginEntity, RefInput};
use crate::dialog::{
    CommandMeta, Condition, DialogInit, DialogRegistration, DialogSchema, EntityFilter,
    FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue, FieldValues, ManipAnchor,
};
use crate::solid::extrude::dialog::{choice, is};
use crate::solid::pattern::dialog::{axis_values, read_axis};

const TOOL: &str = "tool";
const TOOL_REF: &str = "tool_ref";
const EXTEND: &str = "extend_tool";

const TOOLS: [(&str, &str); 4] = [
    ("xy", "XY plane"),
    ("yz", "YZ plane"),
    ("xz", "XZ plane"),
    ("picked", "Selected tool"),
];

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

/// Tool choice, tool pick (shown for "Selected tool") and "Extend tool", each shown when
/// `shown` holds.
fn tool_fields(tool_filter: EntityFilter, shown: Option<Condition>) -> [FieldSpec; 3] {
    let mut tool = choice(TOOL, "Splitting tool", &TOOLS);
    tool.visible_if.clone_from(&shown);
    let mut r = pick(TOOL_REF, "Tool reference", tool_filter, Some(1));
    r.visible_if = Some(match &shown {
        Some(c) => Condition::All(vec![c.clone(), is(TOOL, "picked")]),
        None => is(TOOL, "picked"),
    });
    let mut extend = FieldSpec::new(EXTEND, "Extend tool", FieldKind::Bool);
    extend.visible_if = Some(match &shown {
        Some(c) => Condition::All(vec![c.clone(), is(TOOL, "picked")]),
        None => is(TOOL, "picked"),
    });
    extend.tooltip = Some("Use the whole surface of a face tool, not only the face.".into());
    [tool, r, extend]
}

fn tool_values(tool: &RefInput, extend: bool) -> FieldValues {
    let (choice, sel) = axis_values(tool, "yz");
    FieldValues::default()
        .with(TOOL, FieldValue::Choice(choice))
        .with(TOOL_REF, FieldValue::Selection(sel))
        .with(EXTEND, FieldValue::Bool(extend))
}

fn merge(mut a: FieldValues, b: FieldValues) -> FieldValues {
    a.0.extend(b.0);
    a
}

fn push<T>(r: Result<T, (FieldKey, String)>, errs: &mut FieldErrors) -> Option<T> {
    r.map_err(|e| errs.0.push(e)).ok()
}

fn no_anchors(_f: &dyn Feature, _design: &Design, _eval: &Evaluated) -> Vec<ManipAnchor> {
    Vec::new()
}

const SPLIT_FILTER: EntityFilter = EntityFilter::FACE
    .union(EntityFilter::PLANAR_FACE)
    .union(EntityFilter::CONSTRUCTION_PLANE);

// ---- split body -----------------------------------------------------------------------------

const BODY: &str = "body";

fn body_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut fields = vec![pick(BODY, "Body to split", EntityFilter::BODY, Some(1))];
        fields.extend(tool_fields(SPLIT_FILTER.union(EntityFilter::BODY), None));
        DialogSchema {
            kind: "solid.split_body",
            title: "Split Body".into(),
            help: Some("solid.split_body"),
            fields,
        }
    })
}

fn body_values(s: &SplitBody) -> FieldValues {
    merge(
        FieldValues::default().with(
            BODY,
            FieldValue::Selection(s.body.map(EntityRef::Body).into_iter().collect()),
        ),
        tool_values(&s.tool, s.extend_tool),
    )
}

fn body_defaults(init: &DialogInit<'_>) -> FieldValues {
    let body = init.preselection.iter().find_map(|e| match e {
        EntityRef::Body(b) => Some(*b),
        _ => None,
    });
    body_values(&SplitBody {
        body,
        ..SplitBody::default()
    })
}

fn body_read(f: &dyn Feature, _design: &Design) -> Option<FieldValues> {
    f.as_any().downcast_ref::<SplitBody>().map(body_values)
}

fn body_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let body: Option<BodyId> = match push(v.selection(BODY), &mut errs) {
        Some([]) | None => None,
        Some([EntityRef::Body(b)]) => Some(*b),
        Some(_) => {
            errs.0.push((FieldKey(BODY), "select one body".into()));
            None
        }
    };
    let tool = push(read_axis(v, TOOL, TOOL_REF, "yz"), &mut errs);
    let extend = push(v.flag(EXTEND), &mut errs).unwrap_or(false);
    match tool {
        Some(tool) if errs.is_empty() => Ok(FeatureBox::new(SplitBody {
            body,
            tool,
            extend_tool: extend,
        })),
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.split_body",
        schema: body_schema,
        defaults: body_defaults,
        read: body_read,
        build: body_build,
        anchors: no_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 42,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["split", "body", "cut", "divide", "slice"],
        },
        new_component: None,
    }
}

// ---- split face -----------------------------------------------------------------------------

const FACES: &str = "faces";
const TOOL_TYPE: &str = "tool_type";
const CURVES: &str = "curves";
const DIRECTION: &str = "direction";
const DIRECTION_REF: &str = "direction_ref";

const TOOL_TYPES: [(&str, &str); 2] = [
    ("surface", "Plane, face or body"),
    ("curves", "Sketch curves"),
];
const DIRECTIONS: [(&str, &str); 5] = [
    ("normal", "Sketch normal"),
    ("x", "X axis"),
    ("y", "Y axis"),
    ("z", "Z axis"),
    ("picked", "Selected direction"),
];

fn face_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut fields = vec![
            pick(FACES, "Faces to split", EntityFilter::FACE, None),
            choice(TOOL_TYPE, "Tool type", &TOOL_TYPES),
        ];
        fields.extend(tool_fields(
            SPLIT_FILTER.union(EntityFilter::BODY),
            Some(is(TOOL_TYPE, "surface")),
        ));
        let mut curves = pick(CURVES, "Curves", EntityFilter::SKETCH_CURVE, None);
        curves.visible_if = Some(is(TOOL_TYPE, "curves"));
        fields.push(curves);
        let mut dir = choice(DIRECTION, "Direction", &DIRECTIONS);
        dir.visible_if = Some(is(TOOL_TYPE, "curves"));
        fields.push(dir);
        let mut dir_ref = pick(
            DIRECTION_REF,
            "Direction reference",
            EntityFilter::EDGE
                .union(EntityFilter::SKETCH_CURVE)
                .union(EntityFilter::CONSTRUCTION_AXIS),
            Some(1),
        );
        dir_ref.visible_if = Some(Condition::All(vec![
            is(TOOL_TYPE, "curves"),
            is(DIRECTION, "picked"),
        ]));
        fields.push(dir_ref);
        DialogSchema {
            kind: "solid.split_face",
            title: "Split Face".into(),
            help: Some("solid.split_face"),
            fields,
        }
    })
}

fn face_values(f: &SplitFace) -> FieldValues {
    let faces = FieldValue::Selection(f.faces.iter().cloned().map(EntityRef::Topo).collect());
    let base = FieldValues::default().with(FACES, faces);
    match &f.tool {
        SplitFaceTool::Surface(r) => merge(
            base.with(TOOL_TYPE, FieldValue::Choice("surface"))
                .with(CURVES, FieldValue::Selection(Vec::new()))
                .with(DIRECTION, FieldValue::Choice("normal"))
                .with(DIRECTION_REF, FieldValue::Selection(Vec::new())),
            tool_values(r, f.extend_tool),
        ),
        SplitFaceTool::CurvesAlong { curves, dir } => {
            let (d, sel) = match dir {
                None => ("normal", Vec::new()),
                Some(r) => axis_values(r, "normal"),
            };
            merge(
                base.with(TOOL_TYPE, FieldValue::Choice("curves"))
                    .with(CURVES, FieldValue::Selection(curves.clone()))
                    .with(DIRECTION, FieldValue::Choice(d))
                    .with(DIRECTION_REF, FieldValue::Selection(sel)),
                tool_values(&RefInput::Origin(OriginEntity::YZPlane), f.extend_tool),
            )
        }
    }
}

fn face_defaults(init: &DialogInit<'_>) -> FieldValues {
    let faces: Vec<TopoRef> = init
        .preselection
        .iter()
        .filter_map(|e| match e {
            EntityRef::Topo(t) if EntityFilter::FACE.admits(e) => Some(t.clone()),
            _ => None,
        })
        .collect();
    face_values(&SplitFace::by_surface(
        faces,
        RefInput::Origin(OriginEntity::YZPlane),
    ))
}

fn face_read(f: &dyn Feature, _design: &Design) -> Option<FieldValues> {
    f.as_any().downcast_ref::<SplitFace>().map(face_values)
}

fn face_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let mut faces = Vec::new();
    for e in push(v.selection(FACES), &mut errs).unwrap_or_default() {
        match e {
            EntityRef::Topo(t) => faces.push(t.clone()),
            _ => errs.0.push((FieldKey(FACES), "select faces".into())),
        }
    }
    let extend = push(v.flag(EXTEND), &mut errs).unwrap_or(false);
    let tool = match push(v.choice(TOOL_TYPE, "surface"), &mut errs) {
        Some("curves") => {
            let curves = push(v.selection(CURVES), &mut errs)
                .unwrap_or_default()
                .to_vec();
            let dir = match push(v.choice(DIRECTION, "normal"), &mut errs) {
                Some("normal") | None => None,
                Some(_) => push(read_axis(v, DIRECTION, DIRECTION_REF, "z"), &mut errs),
            };
            Some(SplitFaceTool::CurvesAlong { curves, dir })
        }
        Some(_) => push(read_axis(v, TOOL, TOOL_REF, "yz"), &mut errs).map(SplitFaceTool::Surface),
        None => None,
    };
    match tool {
        Some(tool) if errs.is_empty() => Ok(FeatureBox::new(SplitFace {
            faces,
            tool,
            extend_tool: extend,
        })),
        _ => Err(errs),
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.split_face",
        schema: face_schema,
        defaults: face_defaults,
        read: face_read,
        build: face_build,
        anchors: no_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 43,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["split", "face", "imprint", "divide", "partition"],
        },
        new_component: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialog::registration;
    use amg_doc::stub::SketchEntityKey;
    use amg_doc::{FeatureId, SketchId};

    fn labels_unique(s: &DialogSchema) {
        let mut l: Vec<&str> = s.fields.iter().map(|f| f.label.as_str()).collect();
        let n = l.len();
        l.sort_unstable();
        l.dedup();
        assert_eq!(l.len(), n, "{}: duplicate field label", s.kind);
        for f in &s.fields {
            if let FieldKind::Choice { options, .. } = &f.kind {
                for o in options {
                    assert!(!l.contains(&o.label.as_str()), "{}: {}", s.kind, o.label);
                }
            }
        }
    }

    #[test]
    fn labels_are_unique_and_both_kinds_are_registered() {
        labels_unique(body_schema());
        labels_unique(face_schema());
        assert!(registration("solid.split_body").is_some());
        assert!(registration("solid.split_face").is_some());
    }

    #[test]
    fn every_tool_kind_round_trips() {
        let doc = amg_doc::Document::new("split");
        let b = BodyId {
            origin: FeatureId(3),
            slot: 0,
        };
        let face = EntityRef::Body(BodyId {
            origin: FeatureId(4),
            slot: 0,
        });
        for (tool, extend) in [
            (RefInput::Origin(OriginEntity::XYPlane), false),
            (RefInput::Origin(OriginEntity::XZPlane), false),
            (RefInput::Ref(face.clone()), true),
        ] {
            let s = SplitBody {
                body: Some(b),
                tool,
                extend_tool: extend,
            };
            let v = body_values(&s);
            let f = body_build(&v, None).unwrap();
            assert_eq!(f.get().as_any().downcast_ref::<SplitBody>(), Some(&s));
            assert_eq!(body_read(f.get(), doc.design()).unwrap(), v);
        }
        let curve = EntityRef::SketchCurve {
            sketch: SketchId(FeatureId(2)),
            curve: SketchEntityKey(7),
        };
        for f in [
            SplitFace::by_curves(Vec::new(), vec![curve.clone()]),
            SplitFace {
                faces: Vec::new(),
                tool: SplitFaceTool::CurvesAlong {
                    curves: vec![curve.clone()],
                    dir: Some(RefInput::Origin(OriginEntity::XAxis)),
                },
                extend_tool: false,
            },
            SplitFace {
                faces: Vec::new(),
                tool: SplitFaceTool::CurvesAlong {
                    curves: vec![curve.clone()],
                    dir: Some(RefInput::Ref(curve.clone())),
                },
                extend_tool: false,
            },
            SplitFace::by_surface(Vec::new(), RefInput::Ref(face.clone())),
        ] {
            let v = face_values(&f);
            let back = face_build(&v, None).unwrap();
            assert_eq!(back.get().as_any().downcast_ref::<SplitFace>(), Some(&f));
            assert_eq!(face_read(back.get(), doc.design()).unwrap(), v);
        }
    }
}
