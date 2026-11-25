//! `solid.extrude` (S04 §4; SOL-01 minimal: one side, a distance, starting at the profile).
//!
//! Naming (naming note §5.2): laterals `Gen { self, Lateral#0, SketchCurve(sketch, entity) }`
//! from the builder tags, caps `StartCap#0` / `EndCap#0` from `Region(p)`. Regions of one sketch
//! that share curves are fused (and unified) into one tool solid first (note §3.4).

use std::sync::OnceLock;

use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, InputSub, OpResult, OpRole, PrismExtent, PrismOpts, Relation,
    ShapeHistory, SubKind, SubShape,
};
use amg_core::{Dir3, Point3};
use amg_doc::naming::{OperandNaming, Role, TopoKind};
use amg_doc::{
    BodyId, BodyNaming, Dependency, Design, Dim, EntityRef, Evaluated, Expr, ExprSrc, Feature,
    FeatureBox, FeatureError, FeatureOutput, RecomputeCtx, register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{
    BodyOp, OpTarget, SolidTool, ToolBody, apply_body_op, finish_naming, op_kernel, simplify,
    with_job,
};
use super::error::{SolidError, kerr};
use super::extent::{Direction, Extent, ExtentSide, Start, ThinSpec, extent_refs, side_exprs};
use super::profile::{ProfileFace, ProfileSource, cap_source, profile_faces, tag_source};
use crate::dialog::{
    ChoiceOption, ChoiceStyle, CommandMeta, Condition, DialogInit, DialogRegistration,
    DialogSchema, EntityFilter, FieldErrors, FieldKey, FieldKind, FieldSpec, FieldValue,
    FieldValues, ManipAnchor, ManipKind,
};

/// `solid.extrude`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Extrude {
    /// Profile regions (`EntityRef::Profile`) or planar faces (`EntityRef::Topo`, SOL-02).
    pub profiles: Vec<EntityRef>,
    /// Where the extent starts.
    #[serde(default)]
    pub start: Start,
    /// One side, two sides or symmetric.
    #[serde(default)]
    pub direction: Direction,
    /// The first (or only) side.
    pub side1: ExtentSide,
    /// The second side; `Some` iff `direction == TwoSides`.
    #[serde(default)]
    pub side2: Option<ExtentSide>,
    /// Extrude against the profile normal.
    #[serde(default)]
    pub flip: bool,
    /// Thin extrude (SOL-18).
    #[serde(default)]
    pub thin: Option<ThinSpec>,
    /// Body operation.
    #[serde(default)]
    pub target: OpTarget,
}

impl Extrude {
    /// A one-sided extrude of `profiles` by `distance` (an expression such as `"30 mm"`).
    pub fn distance(profiles: Vec<EntityRef>, distance: &str, target: OpTarget) -> Extrude {
        Extrude {
            profiles,
            start: Start::Profile,
            direction: Direction::OneSide,
            side1: ExtentSide::distance(Expr::Inline(ExprSrc(distance.to_owned()))),
            side2: None,
            flip: false,
            thin: None,
            target,
        }
    }

    /// The options SOL-01 computes; anything else is `NotYet`.
    fn supported(&self, ctx: &RecomputeCtx<'_>) -> Result<&Expr, FeatureError> {
        let not_yet = |what| Err(SolidError::NotYet { what }.into());
        if self.thin.is_some() {
            return not_yet("thin extrude");
        }
        if self.start != Start::Profile {
            return not_yet("extrude start offsets and start objects");
        }
        if self.direction != Direction::OneSide {
            return not_yet("two-sided and symmetric extrudes");
        }
        let Extent::Distance(d) = &self.side1.extent else {
            return not_yet("to-object and through-all extents");
        };
        if ctx.eval_angle(&self.side1.taper)? != 0.0 {
            return not_yet("tapered extrudes");
        }
        Ok(d)
    }
}

/// The prism of one profile face along `v`, named (laterals from tags, caps from the region).
fn prism_one(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    f: &ProfileFace,
    v: amg_core::Vec3,
    out: &mut FeatureOutput,
) -> Result<(OpResult, BodyNaming), FeatureError> {
    let r = with_job(ctx, |job| {
        k.prism(
            &f.face,
            &PrismExtent::Vector(v),
            &PrismOpts {
                taper: 0.0,
                merge_coplanar: false,
            },
            job,
        )
    })
    .map_err(kerr)?;
    let mut b = ctx.naming_builder();
    b.from_history(&[(0, OperandNaming::Tagged(&f.edge_tags))], &r.history);
    if let ProfileSource::Region { sketch, .. } = &f.source {
        for (_, tag) in &f.edge_tags {
            b.tag(*tag, tag_source(*sketch, *tag));
        }
    }
    let cap = cap_source(ctx, f)?;
    for (sub, role) in &r.history.roles {
        let role = match role {
            OpRole::StartCap => Role::new("StartCap", 0),
            OpRole::EndCap => Role::new("EndCap", 0),
            _ => continue,
        };
        if sub.kind == SubKind::Face {
            b.generated(TopoKind::Face, sub.index, role, Some(cap.clone()));
        }
    }
    let naming = finish_naming(b, k, &r.shape, out)?;
    Ok((r, naming))
}

/// The prisms of one group fused and unified into one solid (shared laterals vanish).
fn fuse_group(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    parts: Vec<(OpResult, BodyNaming)>,
    out: &mut FeatureOutput,
) -> Result<(amg_brep::Shape, BodyNaming), FeatureError> {
    let mut parts = parts;
    if parts.len() == 1
        && let Some((r, n)) = parts.pop()
    {
        return Ok((r.shape, n));
    }
    let shapes: Vec<amg_brep::Shape> = parts.iter().map(|(r, _)| r.shape.clone()).collect();
    let r = with_job(ctx, |job| {
        k.boolean(BoolOp::Fuse, &shapes, &[], &BoolOpts::default(), job)
    })
    .and_then(|r| simplify(k, r))
    .map_err(kerr)?;
    let operands: Vec<(u16, OperandNaming<'_>)> = parts
        .iter()
        .enumerate()
        .map(|(i, (_, n))| {
            (
                u16::try_from(i).unwrap_or(u16::MAX),
                OperandNaming::Named(n),
            )
        })
        .collect();
    let mut b = ctx.naming_builder();
    b.from_history(&operands, &r.history);
    let naming = finish_naming(b, k, &r.shape, out)?;
    Ok((r.shape, naming))
}

/// Disjoint solids as one compound, named through an index-offset history: a compound's
/// sub-shape indices are its parts' in order (`make_compound` follows `TopExp::MapShapes`).
pub(crate) fn compound_of(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    parts: Vec<(amg_brep::Shape, BodyNaming)>,
    out: &mut FeatureOutput,
) -> Result<(amg_brep::Shape, BodyNaming), FeatureError> {
    let mut parts = parts;
    if parts.len() == 1
        && let Some(p) = parts.pop()
    {
        return Ok(p);
    }
    let shapes: Vec<amg_brep::Shape> = parts.iter().map(|(s, _)| s.clone()).collect();
    let shape = k.make_compound(&shapes).map_err(kerr)?;
    let kinds = [SubKind::Face, SubKind::Edge, SubKind::Vertex];
    let mut offset = [0u32; 3];
    let mut entries = Vec::new();
    let mut operand_counts = Vec::new();
    for (i, (s, _)) in parts.iter().enumerate() {
        let operand = u16::try_from(i).unwrap_or(u16::MAX);
        let counts = kinds.map(|kd| s.count(kd));
        for (j, kind) in kinds.into_iter().enumerate() {
            for index in 0..counts[j] {
                let sub = SubShape { kind, index };
                let to = SubShape {
                    kind,
                    index: offset[j] + index,
                };
                entries.push((InputSub { operand, sub }, Relation::Kept(to)));
            }
            offset[j] += counts[j];
        }
        operand_counts.push(counts);
    }
    if kinds.map(|kd| shape.count(kd)) != offset {
        return Err(FeatureError::Kernel(
            "profile solids share sub-shapes; can't name their compound".into(),
        ));
    }
    entries.sort_by_key(|(i, _)| *i);
    let history = ShapeHistory {
        entries,
        roles: Vec::new(),
        tags: Vec::new(),
        operand_counts,
    };
    let operands: Vec<(u16, OperandNaming<'_>)> = parts
        .iter()
        .enumerate()
        .map(|(i, (_, n))| {
            (
                u16::try_from(i).unwrap_or(u16::MAX),
                OperandNaming::Named(n),
            )
        })
        .collect();
    let mut b = ctx.naming_builder();
    b.from_history(&operands, &history);
    let naming = finish_naming(b, k, &shape, out)?;
    Ok((shape, naming))
}

impl SolidTool for Extrude {
    fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError> {
        let distance = self.supported(ctx)?;
        let mut d = ctx.eval_length(distance)?;
        if !d.is_finite() || d.abs() <= ctx.tolerance().linear {
            return Err(SolidError::Invalid {
                field: "distance",
                msg: "the distance must not be zero".into(),
            }
            .into());
        }
        let faces = profile_faces(ctx, &self.profiles)?;
        let mut n = faces[0].plane.normal.get();
        if self.flip {
            n = -n;
        }
        if d < 0.0 {
            n = -n;
            d = -d;
        }
        let k = op_kernel(ctx);
        // Naming warnings of the tool are not feature warnings (the tool's history is complete
        // by construction); a partial history shows up again in the body op.
        let mut scratch = FeatureOutput::default();
        let mut groups: Vec<Vec<(OpResult, BodyNaming)>> = Vec::new();
        for f in &faces {
            ctx.check_cancelled()?;
            let g = f.group as usize;
            if groups.len() <= g {
                groups.resize_with(g + 1, Vec::new);
            }
            groups[g].push(prism_one(ctx, k, f, n * d, &mut scratch)?);
        }
        let mut solids = Vec::with_capacity(groups.len());
        for g in groups {
            solids.push(fuse_group(ctx, k, g, &mut scratch)?);
        }
        let (shape, naming) = compound_of(ctx, k, solids, &mut scratch)?;
        Ok(ToolBody {
            shape,
            naming,
            target: self.target.clone(),
        })
    }
}

impl Feature for Extrude {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d: Vec<Dependency> = Vec::new();
        for p in &self.profiles {
            d.push(Dependency::Entity(p.clone()));
            if let EntityRef::Profile { sketch, .. } = p {
                let s = Dependency::Sketch(*sketch);
                if !d.contains(&s) {
                    d.push(s);
                }
            }
        }
        d.extend(
            extent_refs(&self.start, std::iter::once(&self.side1).chain(&self.side2))
                .map(Dependency::Entity),
        );
        d.extend(self.target.inputs());
        d
    }

    fn exprs(&self) -> Vec<&Expr> {
        // The fields of `exprs_mut`, in the same order.
        let mut v: Vec<&Expr> = Vec::new();
        match &self.start {
            Start::Offset(e) | Start::Object { offset: e, .. } => v.push(e),
            Start::Profile => {}
        }
        for s in std::iter::once(&self.side1).chain(&self.side2) {
            match &s.extent {
                Extent::Distance(e) | Extent::ToObject { offset: e, .. } => v.push(e),
                Extent::All | Extent::Full => {}
            }
            v.push(&s.taper);
        }
        if let Some(t) = &self.thin {
            v.push(&t.thickness);
        }
        v
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        let mut v = Vec::new();
        match &mut self.start {
            Start::Offset(e) | Start::Object { offset: e, .. } => v.push((e, Dim::LENGTH)),
            Start::Profile => {}
        }
        v.extend(side_exprs(&mut self.side1, Dim::LENGTH));
        if let Some(s) = &mut self.side2 {
            v.extend(side_exprs(s, Dim::LENGTH));
        }
        if let Some(t) = &mut self.thin {
            v.push((&mut t.thickness, Dim::LENGTH));
        }
        v
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self.profiles.is_empty() {
            return Err(SolidError::Invalid {
                field: "profiles",
                msg: "select at least one profile".into(),
            }
            .into());
        }
        if self.side2.is_some() != (self.direction == Direction::TwoSides) {
            return Err(SolidError::Invalid {
                field: "side2",
                msg: "a second side is set exactly for two-sided extrudes".into(),
            }
            .into());
        }
        if std::iter::once(&self.side1)
            .chain(&self.side2)
            .any(|s| s.extent == Extent::Full)
        {
            return Err(SolidError::Invalid {
                field: "extent",
                msg: "a full turn is a revolve extent".into(),
            }
            .into());
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let tool = self.tool(ctx)?;
        let mut out = FeatureOutput::default();
        apply_body_op(ctx, tool, &mut out)?;
        Ok(out)
    }
}

register_feature!(Extrude, "solid.extrude", version = 1);
crate::register_tool!(Extrude, "solid.extrude");

// ---- dialog ---------------------------------------------------------------------------------

const PROFILES: &str = "profiles";
const DISTANCE: &str = "distance";
const FLIP: &str = "flip";
const OPERATION: &str = "operation";
const BODIES: &str = "bodies";

const OPS: [(&str, BodyOp, &str); 4] = [
    ("new_body", BodyOp::NewBody, "New body"),
    ("join", BodyOp::Join, "Join"),
    ("cut", BodyOp::Cut, "Cut"),
    ("intersect", BodyOp::Intersect, "Intersect"),
];

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
        profiles.tooltip = Some("Sketch profiles or planar faces to extrude.".into());
        let mut distance = FieldSpec::new(
            DISTANCE,
            "Distance",
            FieldKind::Expr {
                dim: Dim::LENGTH,
                default: "10 mm",
            },
        );
        distance.manipulator = Some(ManipKind::Distance);
        let operation = FieldSpec::new(
            OPERATION,
            "Operation",
            FieldKind::Choice {
                options: OPS
                    .iter()
                    .map(|(id, _, label)| ChoiceOption::new(id, label))
                    .collect(),
                style: ChoiceStyle::Dropdown,
            },
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
        bodies.visible_if = Some(Condition::Not(Box::new(Condition::Is(
            FieldKey(OPERATION),
            FieldValue::Choice("new_body"),
        ))));
        DialogSchema {
            kind: "solid.extrude",
            title: "Extrude".into(),
            help: Some("solid.extrude"),
            fields: vec![
                profiles,
                distance,
                FieldSpec::new(FLIP, "Flip", FieldKind::FlipDirection),
                operation,
                bodies,
            ],
        }
    })
}

fn defaults(init: &DialogInit<'_>) -> FieldValues {
    let profiles: Vec<EntityRef> = init
        .preselection
        .iter()
        .filter(|e| matches!(e, EntityRef::Profile { .. }))
        .cloned()
        .collect();
    FieldValues::default()
        .with(PROFILES, FieldValue::Selection(profiles))
        .with(DISTANCE, FieldValue::expr("10 mm"))
        .with(FLIP, FieldValue::Flip(false))
        .with(OPERATION, FieldValue::Choice("new_body"))
        .with(BODIES, FieldValue::Selection(Vec::new()))
}

fn read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    let e = f.as_any().downcast_ref::<Extrude>()?;
    let distance = match &e.side1.extent {
        Extent::Distance(d) => FieldValue::from_expr(d, design),
        _ => FieldValue::expr("10 mm"),
    };
    let op = OPS
        .iter()
        .find(|(_, o, _)| *o == e.target.op)
        .map_or("new_body", |(id, _, _)| id);
    Some(
        FieldValues::default()
            .with(PROFILES, FieldValue::Selection(e.profiles.clone()))
            .with(DISTANCE, distance)
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
            ),
    )
}

fn build(v: &FieldValues, old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let mut errs = FieldErrors::default();
    let mut f = old
        .and_then(|o| o.as_any().downcast_ref::<Extrude>())
        .cloned()
        .unwrap_or_else(|| Extrude::distance(Vec::new(), "10 mm", OpTarget::default()));
    match v.selection(PROFILES) {
        Ok([]) => errs.0.push((FieldKey(PROFILES), "select a profile".into())),
        Ok(p) => f.profiles = p.to_vec(),
        Err(e) => errs.0.push(e),
    }
    match v.expr(DISTANCE) {
        Ok(d) => f.side1.extent = Extent::Distance(d),
        Err(e) => errs.0.push(e),
    }
    match v.flag(FLIP) {
        Ok(b) => f.flip = b,
        Err(e) => errs.0.push(e),
    }
    match v.choice(OPERATION, "new_body") {
        Ok(id) => match OPS.iter().find(|(o, _, _)| *o == id) {
            Some((_, op, _)) => f.target.op = *op,
            None => errs
                .0
                .push((FieldKey(OPERATION), format!("unknown operation `{id}`"))),
        },
        Err(e) => errs.0.push(e),
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

/// The distance arrow: at the first profile's centroid, along its (flipped) normal.
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
    vec![ManipAnchor {
        field: FieldKey(DISTANCE),
        origin,
        axis: Dir3::new_unchecked(axis.get()),
        ref_dir: None,
        base: 0.0,
    }]
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
            keywords: &["extrude", "boss", "pocket", "pull", "push"],
        },
        new_component: None,
    }
}
