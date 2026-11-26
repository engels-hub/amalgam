//! Dialog registration of `solid.combine` (S11 §B schema, rendered by UI-05): target body,
//! tool bodies, operation, keep tools.

use std::sync::OnceLock;

use amg_doc::{BodyId, Design, EntityRef, Evaluated, Feature, FeatureBox};

use super::Combine;
use crate::dialog::{
    ChoiceOption, ChoiceStyle, CommandMeta, DialogInit, DialogRegistration, DialogSchema,
    EntityFilter, FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue, FieldValues,
    ManipAnchor,
};
use crate::solid::common::BodyOp;

const TARGET: &str = "target";
const TOOLS: &str = "tools";
const OPERATION: &str = "operation";
const KEEP_TOOLS: &str = "keep_tools";

const OPS: [(&str, BodyOp, &str); 3] = [
    ("join", BodyOp::Join, "Join"),
    ("cut", BodyOp::Cut, "Cut"),
    ("intersect", BodyOp::Intersect, "Intersect"),
];

fn bodies(key: &'static str, label: &str, max: Option<u32>) -> FieldSpec {
    FieldSpec::new(
        key,
        label,
        FieldKind::Selection {
            filter: EntityFilter::BODY,
            min: 1,
            max,
        },
    )
}

fn schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut keep = FieldSpec::new(KEEP_TOOLS, "Keep tools", FieldKind::Bool);
        keep.tooltip =
            Some("Keep the tool bodies instead of combining them into the target.".into());
        DialogSchema {
            kind: "solid.combine",
            title: "Combine".into(),
            help: Some("solid.combine"),
            fields: vec![
                bodies(TARGET, "Target body", Some(1)),
                bodies(TOOLS, "Tool bodies", None),
                FieldSpec::new(
                    OPERATION,
                    "Operation",
                    FieldKind::Choice {
                        options: OPS
                            .iter()
                            .map(|(id, _, l)| ChoiceOption::new(id, l))
                            .collect(),
                        style: ChoiceStyle::Dropdown,
                    },
                ),
                keep,
            ],
        }
    })
}

fn body_refs(ids: impl IntoIterator<Item = BodyId>) -> Vec<EntityRef> {
    ids.into_iter().map(EntityRef::Body).collect()
}

fn values(target: Option<BodyId>, tools: &[BodyId], op: BodyOp, keep: bool) -> FieldValues {
    let op = OPS
        .iter()
        .find(|(_, o, _)| *o == op)
        .map_or("join", |(id, _, _)| *id);
    FieldValues::default()
        .with(TARGET, FieldValue::Selection(body_refs(target)))
        .with(
            TOOLS,
            FieldValue::Selection(body_refs(tools.iter().copied())),
        )
        .with(OPERATION, FieldValue::Choice(op))
        .with(KEEP_TOOLS, FieldValue::Bool(keep))
}

/// The first preselected body is the target, the others are tools.
fn defaults(init: &DialogInit<'_>) -> FieldValues {
    let picked: Vec<BodyId> = init
        .preselection
        .iter()
        .filter_map(|e| match e {
            EntityRef::Body(b) => Some(*b),
            _ => None,
        })
        .collect();
    let (target, tools) = match picked.split_first() {
        Some((t, rest)) => (Some(*t), rest.to_vec()),
        None => (None, Vec::new()),
    };
    values(target, &tools, BodyOp::Join, false)
}

fn read(f: &dyn Feature, _design: &Design) -> Option<FieldValues> {
    let c = f.as_any().downcast_ref::<Combine>()?;
    Some(values(c.target, &c.tools, c.op, c.keep_tools))
}

fn read_bodies(v: &FieldValues, key: &'static str, errs: &mut FieldErrors) -> Vec<BodyId> {
    let mut out = Vec::new();
    match v.selection(key) {
        Ok(sel) => {
            for e in sel {
                match e {
                    EntityRef::Body(b) => out.push(*b),
                    _ => errs.0.push((FieldKey(key), "select bodies".into())),
                }
            }
        }
        Err(e) => errs.0.push(e),
    }
    out
}

fn build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let target = read_bodies(v, TARGET, &mut errs);
    if target.len() > 1 {
        errs.0
            .push((FieldKey(TARGET), "select one target body".into()));
    }
    let tools = read_bodies(v, TOOLS, &mut errs);
    if let Some(t) = target.first()
        && tools.contains(t)
    {
        errs.0.push((
            FieldKey(TOOLS),
            "the target body can't also be a tool".into(),
        ));
    }
    let op = match v.choice(OPERATION, "join") {
        Ok(id) => match OPS.iter().find(|(o, _, _)| *o == id) {
            Some((_, op, _)) => Some(*op),
            None => {
                errs.0
                    .push((FieldKey(OPERATION), format!("unknown option `{id}`")));
                None
            }
        },
        Err(e) => {
            errs.0.push(e);
            None
        }
    };
    let keep = match v.flag(KEEP_TOOLS) {
        Ok(b) => b,
        Err(e) => {
            errs.0.push(e);
            false
        }
    };
    match op {
        Some(op) if errs.is_empty() => Ok(FeatureBox::new(Combine {
            target: target.first().copied(),
            tools,
            op,
            keep_tools: keep,
        })),
        _ => Err(errs),
    }
}

fn anchors(_f: &dyn Feature, _design: &Design, _eval: &Evaluated) -> Vec<ManipAnchor> {
    Vec::new()
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.combine",
        schema,
        defaults,
        read,
        build,
        anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 40,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["combine", "join", "cut", "intersect", "boolean", "union", "subtract"],
        },
        new_component: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_doc::{Document, FeatureId};

    fn body(origin: u64) -> BodyId {
        BodyId {
            origin: FeatureId(origin),
            slot: 0,
        }
    }

    #[test]
    fn preselected_bodies_become_target_and_tools_and_round_trip() {
        let doc = Document::new("combine");
        let pre = [EntityRef::Body(body(1)), EntityRef::Body(body(2))];
        let init = DialogInit {
            preselection: &pre,
            design: doc.design(),
            evaluated: doc.evaluated(),
        };
        let v = defaults(&init);
        let f = build(&v, None).unwrap();
        let c = f.get().as_any().downcast_ref::<Combine>().unwrap();
        assert_eq!(c, &Combine::new(body(1), vec![body(2)], BodyOp::Join));
        assert_eq!(read(f.get(), doc.design()).unwrap(), v);
        for (id, op, _) in OPS {
            let v =
                values(Some(body(1)), &[body(2)], op, true).with(OPERATION, FieldValue::Choice(id));
            let f = build(&v, None).unwrap();
            assert_eq!(read(f.get(), doc.design()).unwrap(), v);
        }
    }

    #[test]
    fn the_target_as_tool_is_a_field_error() {
        let v = values(Some(body(1)), &[body(1)], BodyOp::Cut, false);
        let e = build(&v, None).unwrap_err();
        assert_eq!(e.0[0].0, FieldKey(TOOLS));
    }
}
