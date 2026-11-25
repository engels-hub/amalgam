//! Dialog registrations of `solid.fillet` and `solid.chamfer` (S11 §B schema, rendered by
//! UI-05). The dialog edits the feature's **first** set; further sets (from scripts or later
//! dialog versions) are kept unchanged when an existing feature is edited. An empty edge pick
//! builds (the field's `min: 1` keeps OK disabled); such a feature fails to compute.

use std::sync::OnceLock;

use amg_core::{Dir3, Point3};
use amg_doc::naming::KernelGeom;
use amg_doc::{Design, Dim, EntityRef, Evaluated, Feature, FeatureBox, TopoKind, TopoRef};

use super::{
    Chamfer, ChamferKind, ChamferSet, CornerType, Fillet, FilletRadius, FilletSet, VarRadius,
    inline,
};
use crate::dialog::{
    ChoiceOption, ChoiceStyle, CommandMeta, Condition, DialogInit, DialogRegistration,
    DialogSchema, EntityFilter, FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue,
    FieldValues, ManipAnchor, ManipKind,
};

const EDGES: &str = "edges";
const TANGENT_CHAIN: &str = "tangent_chain";
const RADIUS_TYPE: &str = "radius_type";
const RADIUS: &str = "radius";
const START_RADIUS: &str = "start_radius";
const END_RADIUS: &str = "end_radius";
const CHAMFER_TYPE: &str = "chamfer_type";
const DISTANCE: &str = "distance";
const DISTANCE2: &str = "distance2";
const ANGLE: &str = "angle";
const FACE: &str = "face";

const DEFAULT_LEN: &str = "1 mm";
const DEFAULT_ANGLE: &str = "45 deg";

fn edges_field() -> FieldSpec {
    let mut f = FieldSpec::new(
        EDGES,
        "Edges",
        FieldKind::Selection {
            filter: EntityFilter::EDGE,
            min: 1,
            max: None,
        },
    );
    f.tooltip = Some("Edges to blend; they may lie on several bodies.".into());
    f
}

fn chain_field() -> FieldSpec {
    let mut f = FieldSpec::new(TANGENT_CHAIN, "Tangent chain", FieldKind::Bool);
    f.tooltip = Some("Also blend the edges tangent to the picked ones.".into());
    f
}

fn length(key: &'static str, label: &str) -> FieldSpec {
    let mut f = FieldSpec::new(
        key,
        label,
        FieldKind::Expr {
            dim: Dim::LENGTH,
            default: DEFAULT_LEN,
        },
    );
    f.manipulator = Some(ManipKind::Distance);
    f
}

fn when(key: &'static str, ids: &[&'static str]) -> Condition {
    Condition::Any(
        ids.iter()
            .map(|id| Condition::Is(FieldKey(key), FieldValue::Choice(id)))
            .collect(),
    )
}

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

fn edge_refs(sel: &[EntityRef], errs: &mut FieldErrors) -> Vec<TopoRef> {
    let mut v = Vec::new();
    for e in sel {
        match e {
            EntityRef::Topo(t) if t.kind == TopoKind::Edge => v.push(t.clone()),
            _ => errs.0.push((FieldKey(EDGES), "select edges".into())),
        }
    }
    v
}

fn preselected_edges(init: &DialogInit<'_>) -> Vec<EntityRef> {
    init.preselection
        .iter()
        .filter(|e| matches!(e, EntityRef::Topo(t) if t.kind == TopoKind::Edge))
        .cloned()
        .collect()
}

fn selection_of(edges: &[TopoRef]) -> FieldValue {
    FieldValue::Selection(edges.iter().cloned().map(EntityRef::Topo).collect())
}

/// The manipulator anchor of `field`: at the first edge's centroid (the drag axis is world Z;
/// UI-05 projects the drag onto it).
fn edge_anchor(edges: &[TopoRef], field: &'static str, eval: &Evaluated) -> Vec<ManipAnchor> {
    let Some(first) = edges.first() else {
        return Vec::new();
    };
    let Ok(r) = eval.resolve(first) else {
        return Vec::new();
    };
    let geom = KernelGeom::new(eval, amg_doc::naming::default_kernel());
    let Some((_, fp)) = amg_doc::naming::GeomSource::all(&geom, r.body, TopoKind::Edge)
        .into_iter()
        .find(|(i, _)| *i == r.index)
    else {
        return Vec::new();
    };
    let origin: Point3 = fp.centroid;
    vec![ManipAnchor {
        field: FieldKey(field),
        origin,
        axis: Dir3::new_unchecked(amg_core::Vec3::Z),
        ref_dir: None,
        base: 0.0,
    }]
}

// ---- fillet ---------------------------------------------------------------------------------

const RADIUS_TYPES: [(&str, &str); 3] = [
    ("constant", "Constant radius"),
    ("chord", "Chord length"),
    ("variable", "Variable radius"),
];

fn fillet_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut radius = length(RADIUS, "Radius");
        radius.visible_if = Some(when(RADIUS_TYPE, &["constant", "chord"]));
        radius.tooltip = Some("Radius, or the chord length for a chord fillet.".into());
        let mut start = length(START_RADIUS, "Start radius");
        start.visible_if = Some(when(RADIUS_TYPE, &["variable"]));
        let mut end = length(END_RADIUS, "End radius");
        end.visible_if = Some(when(RADIUS_TYPE, &["variable"]));
        DialogSchema {
            kind: "solid.fillet",
            title: "Fillet".into(),
            help: Some("solid.fillet"),
            fields: vec![
                edges_field(),
                choice(RADIUS_TYPE, "Radius type", &RADIUS_TYPES),
                radius,
                start,
                end,
                chain_field(),
            ],
        }
    })
}

fn fillet_defaults(init: &DialogInit<'_>) -> FieldValues {
    FieldValues::default()
        .with(EDGES, FieldValue::Selection(preselected_edges(init)))
        .with(RADIUS_TYPE, FieldValue::Choice("constant"))
        .with(RADIUS, FieldValue::expr(DEFAULT_LEN))
        .with(START_RADIUS, FieldValue::expr(DEFAULT_LEN))
        .with(END_RADIUS, FieldValue::expr(DEFAULT_LEN))
        .with(TANGENT_CHAIN, FieldValue::Bool(true))
}

fn fillet_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    let f = f.as_any().downcast_ref::<Fillet>()?;
    let set = f.sets.first()?;
    let def = || FieldValue::expr(DEFAULT_LEN);
    let (ty, radius, start, end) = match &set.radius {
        FilletRadius::Constant(e) => ("constant", FieldValue::from_expr(e, design), def(), def()),
        FilletRadius::Chord(e) => ("chord", FieldValue::from_expr(e, design), def(), def()),
        FilletRadius::Variable(v) => (
            "variable",
            def(),
            v.first()
                .map_or_else(def, |p| FieldValue::from_expr(&p.radius, design)),
            v.last()
                .map_or_else(def, |p| FieldValue::from_expr(&p.radius, design)),
        ),
    };
    Some(
        FieldValues::default()
            .with(EDGES, selection_of(&set.edges))
            .with(RADIUS_TYPE, FieldValue::Choice(ty))
            .with(RADIUS, radius)
            .with(START_RADIUS, start)
            .with(END_RADIUS, end)
            .with(TANGENT_CHAIN, FieldValue::Bool(f.tangent_chain)),
    )
}

fn fillet_build(v: &FieldValues, old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let mut f = old
        .and_then(|o| o.as_any().downcast_ref::<Fillet>())
        .cloned()
        .unwrap_or_else(|| Fillet {
            sets: Vec::new(),
            tangent_chain: true,
            corner: CornerType::RollingBall,
        });
    let edges = match v.selection(EDGES) {
        Ok(s) => edge_refs(s, &mut errs),
        Err(e) => {
            errs.0.push(e);
            Vec::new()
        }
    };
    let radius = match v.choice(RADIUS_TYPE, "constant") {
        Ok("constant") => v.expr(RADIUS).map(FilletRadius::Constant),
        Ok("chord") => v.expr(RADIUS).map(FilletRadius::Chord),
        Ok("variable") => v.expr(START_RADIUS).and_then(|s| {
            v.expr(END_RADIUS).map(|e| {
                FilletRadius::Variable(vec![
                    VarRadius {
                        at: inline("0"),
                        radius: s,
                    },
                    VarRadius {
                        at: inline("1"),
                        radius: e,
                    },
                ])
            })
        }),
        Ok(other) => Err((
            FieldKey(RADIUS_TYPE),
            format!("unknown radius type `{other}`"),
        )),
        Err(e) => Err(e),
    };
    match v.flag(TANGENT_CHAIN) {
        Ok(b) => f.tangent_chain = b,
        Err(e) => errs.0.push(e),
    }
    match radius {
        Ok(radius) if errs.is_empty() => {
            let set = FilletSet { edges, radius };
            if f.sets.is_empty() {
                f.sets.push(set);
            } else {
                f.sets[0] = set;
            }
            Ok(FeatureBox::new(f))
        }
        Ok(_) => Err(errs),
        Err(e) => {
            errs.0.push(e);
            Err(errs)
        }
    }
}

fn fillet_anchors(f: &dyn Feature, _design: &Design, eval: &Evaluated) -> Vec<ManipAnchor> {
    let Some(f) = f.as_any().downcast_ref::<Fillet>() else {
        return Vec::new();
    };
    let Some(set) = f.sets.first() else {
        return Vec::new();
    };
    let field = match set.radius {
        FilletRadius::Variable(_) => START_RADIUS,
        _ => RADIUS,
    };
    edge_anchor(&set.edges, field, eval)
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.fillet",
        schema: fillet_schema,
        defaults: fillet_defaults,
        read: fillet_read,
        build: fillet_build,
        anchors: fillet_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 20,
            pinned: true,
            icon_svg: &[],
            shortcut: Some("F"),
            keywords: &["fillet", "round", "blend", "radius"],
        },
        new_component: None,
    }
}

// ---- chamfer --------------------------------------------------------------------------------

const CHAMFER_TYPES: [(&str, &str); 3] = [
    ("equal", "Equal distance"),
    ("two_distances", "Two distances"),
    ("distance_angle", "Distance and angle"),
];

fn chamfer_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| {
        let mut d2 = length(DISTANCE2, "Distance 2");
        d2.visible_if = Some(when(CHAMFER_TYPE, &["two_distances"]));
        let mut angle = FieldSpec::new(
            ANGLE,
            "Angle",
            FieldKind::Expr {
                dim: Dim::ANGLE,
                default: DEFAULT_ANGLE,
            },
        );
        angle.manipulator = Some(ManipKind::Angle);
        angle.visible_if = Some(when(CHAMFER_TYPE, &["distance_angle"]));
        let mut face = FieldSpec::new(
            FACE,
            "Reference face",
            FieldKind::Selection {
                filter: EntityFilter::FACE,
                min: 0,
                max: Some(1),
            },
        );
        face.tooltip = Some("The face the first distance is measured on.".into());
        face.visible_if = Some(when(CHAMFER_TYPE, &["two_distances", "distance_angle"]));
        DialogSchema {
            kind: "solid.chamfer",
            title: "Chamfer".into(),
            help: Some("solid.chamfer"),
            fields: vec![
                edges_field(),
                choice(CHAMFER_TYPE, "Chamfer type", &CHAMFER_TYPES),
                length(DISTANCE, "Distance"),
                d2,
                angle,
                face,
                chain_field(),
            ],
        }
    })
}

fn chamfer_defaults(init: &DialogInit<'_>) -> FieldValues {
    FieldValues::default()
        .with(EDGES, FieldValue::Selection(preselected_edges(init)))
        .with(CHAMFER_TYPE, FieldValue::Choice("equal"))
        .with(DISTANCE, FieldValue::expr(DEFAULT_LEN))
        .with(DISTANCE2, FieldValue::expr(DEFAULT_LEN))
        .with(ANGLE, FieldValue::expr(DEFAULT_ANGLE))
        .with(FACE, FieldValue::Selection(Vec::new()))
        .with(TANGENT_CHAIN, FieldValue::Bool(true))
}

fn chamfer_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    let f = f.as_any().downcast_ref::<Chamfer>()?;
    let set = f.sets.first()?;
    let x = |e: &amg_doc::Expr| FieldValue::from_expr(e, design);
    let (ty, d, d2, angle, face) = match &set.kind {
        ChamferKind::Equal(d) => (
            "equal",
            x(d),
            FieldValue::expr(DEFAULT_LEN),
            FieldValue::expr(DEFAULT_ANGLE),
            Vec::new(),
        ),
        ChamferKind::TwoDistances { d1, d2, face } => (
            "two_distances",
            x(d1),
            x(d2),
            FieldValue::expr(DEFAULT_ANGLE),
            vec![EntityRef::Topo(face.clone())],
        ),
        ChamferKind::DistanceAngle { d, angle, face } => (
            "distance_angle",
            x(d),
            FieldValue::expr(DEFAULT_LEN),
            x(angle),
            vec![EntityRef::Topo(face.clone())],
        ),
    };
    Some(
        FieldValues::default()
            .with(EDGES, selection_of(&set.edges))
            .with(CHAMFER_TYPE, FieldValue::Choice(ty))
            .with(DISTANCE, d)
            .with(DISTANCE2, d2)
            .with(ANGLE, angle)
            .with(FACE, FieldValue::Selection(face))
            .with(TANGENT_CHAIN, FieldValue::Bool(f.tangent_chain)),
    )
}

fn chamfer_build(v: &FieldValues, old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let mut f = old
        .and_then(|o| o.as_any().downcast_ref::<Chamfer>())
        .cloned()
        .unwrap_or_else(|| Chamfer {
            sets: Vec::new(),
            tangent_chain: true,
        });
    let edges = match v.selection(EDGES) {
        Ok(s) => edge_refs(s, &mut errs),
        Err(e) => {
            errs.0.push(e);
            Vec::new()
        }
    };
    let face = || -> Result<TopoRef, (FieldKey, String)> {
        match v.selection(FACE)? {
            [EntityRef::Topo(t)] if t.kind == TopoKind::Face => Ok(t.clone()),
            _ => Err((
                FieldKey(FACE),
                "pick the face the first distance is measured on".into(),
            )),
        }
    };
    let kind = match v.choice(CHAMFER_TYPE, "equal") {
        Ok("equal") => v.expr(DISTANCE).map(ChamferKind::Equal),
        Ok("two_distances") => v.expr(DISTANCE).and_then(|d1| {
            v.expr(DISTANCE2)
                .and_then(|d2| face().map(|face| ChamferKind::TwoDistances { d1, d2, face }))
        }),
        Ok("distance_angle") => v.expr(DISTANCE).and_then(|d| {
            v.expr(ANGLE)
                .and_then(|angle| face().map(|face| ChamferKind::DistanceAngle { d, angle, face }))
        }),
        Ok(other) => Err((
            FieldKey(CHAMFER_TYPE),
            format!("unknown chamfer type `{other}`"),
        )),
        Err(e) => Err(e),
    };
    match v.flag(TANGENT_CHAIN) {
        Ok(b) => f.tangent_chain = b,
        Err(e) => errs.0.push(e),
    }
    match kind {
        Ok(kind) if errs.is_empty() => {
            let set = ChamferSet { edges, kind };
            if f.sets.is_empty() {
                f.sets.push(set);
            } else {
                f.sets[0] = set;
            }
            Ok(FeatureBox::new(f))
        }
        Ok(_) => Err(errs),
        Err(e) => {
            errs.0.push(e);
            Err(errs)
        }
    }
}

fn chamfer_anchors(f: &dyn Feature, _design: &Design, eval: &Evaluated) -> Vec<ManipAnchor> {
    let Some(f) = f.as_any().downcast_ref::<Chamfer>() else {
        return Vec::new();
    };
    f.sets
        .first()
        .map(|s| edge_anchor(&s.edges, DISTANCE, eval))
        .unwrap_or_default()
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.chamfer",
        schema: chamfer_schema,
        defaults: chamfer_defaults,
        read: chamfer_read,
        build: chamfer_build,
        anchors: chamfer_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "modify",
            order: 21,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &["chamfer", "bevel"],
        },
        new_component: None,
    }
}
