//! `solid.fillet` and `solid.chamfer` (S04 SOL-05) on raw BREP-06.
//!
//! A feature holds edge **sets**, each with its own radius (constant, chord or variable) or
//! chamfer kind. Picked edges resolve per body (`SetPolicy::DropMissing`); with
//! `tangent_chain` each pick grows along its tangent chain ([`tangent_chain_edges`],
//! R-S04-1). Every body gets one kernel call with all its edges, in pick order (OCCT numbers
//! contours in that order). A kernel failure highlights the offending edges (`kernel_failure`).
//!
//! Naming (naming note §5.2): blend faces are `Blend#0` (chamfer faces `Chamfer#0`) from
//! `Selection(key)` for picked edges and from the propagated edge's own name otherwise;
//! corner patches `Corner#k` by the role-only rule; neighbours keep their names (`Modified`
//! 1:1).
//!
//! G1 only (G2 is S20 KH-10, QUESTIONS #55); setback corners are SOL-21; the S20 robust chain
//! and `FilletDiagnosis` are SOL-20.

mod dialog;
pub mod harness;

use amg_brep::{ChamferSpec, EdgeId, FaceId, FilletOpts, FilletSpec, OpResult, SubKind, SubShape};
use amg_doc::naming::OperandNaming;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, BodyNaming, Dependency, Dim, EntityRef, Expr,
    ExprSrc, Feature, FeatureError, FeatureOutput, RecomputeCtx, SetPolicy, TopoKind, TopoRef,
    register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{
    finish_naming, input_solid, op_kernel, resolve_subs, tangent_chain_edges, with_job,
};
use super::error::{SolidError, kernel_failure, kerr};

/// Largest angle (rad) between the tangents of two edges at a shared vertex for the tangent
/// chain to continue across it (~0.06°; sketch tangencies are solved far tighter).
pub const CHAIN_ANGLE: f64 = 1e-3;

fn inline(src: &str) -> Expr {
    Expr::Inline(ExprSrc(src.to_owned()))
}

fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

// ---- data -----------------------------------------------------------------------------------

/// The radius of one fillet set.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum FilletRadius {
    /// Constant radius (length).
    Constant(Expr),
    /// Constant chord length between the blend's two tangent lines (length), converted to a
    /// radius per edge by the kernel.
    Chord(Expr),
    /// Radius varying along each tangent chain (contour): samples by fraction of its arc
    /// length. Missing ends are held at the nearest sample's radius.
    Variable(Vec<VarRadius>),
}

/// One sample of a variable radius.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VarRadius {
    /// Fraction 0..1 of the chain's arc length.
    pub at: Expr,
    /// Radius there (length).
    pub radius: Expr,
}

/// Edges filleted with one radius.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FilletSet {
    /// Picked edges (set-valued; may span bodies).
    pub edges: Vec<TopoRef>,
    /// Their radius.
    pub radius: FilletRadius,
}

/// How fillets meet at a vertex.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CornerType {
    /// OCCT's rolling-ball corner patch.
    #[default]
    RollingBall,
    /// Setback corners (SOL-21; `NotYet` here).
    Setback,
}

fn yes() -> bool {
    true
}

/// `solid.fillet`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Fillet {
    /// Edge sets, each with its radius.
    pub sets: Vec<FilletSet>,
    /// Grow each pick along its tangent chain.
    #[serde(default = "yes")]
    pub tangent_chain: bool,
    /// Corner type.
    #[serde(default)]
    pub corner: CornerType,
}

impl Fillet {
    /// A constant-radius fillet of `edges` (`radius` an expression such as `"5 mm"`), tangent
    /// chains on.
    pub fn constant(edges: Vec<TopoRef>, radius: &str) -> Fillet {
        Fillet {
            sets: vec![FilletSet {
                edges,
                radius: FilletRadius::Constant(inline(radius)),
            }],
            tangent_chain: true,
            corner: CornerType::RollingBall,
        }
    }

    /// The edges this fillet passes to the kernel, per body (tangent chains grown).
    ///
    /// # Errors
    /// Resolution errors; [`SolidError::Invalid`] for picks that are not edges.
    pub fn plan(&self, ctx: &RecomputeCtx<'_>) -> Result<Vec<BodyEdges>, FeatureError> {
        let sets: Vec<&[TopoRef]> = self.sets.iter().map(|s| s.edges.as_slice()).collect();
        plan_edges(ctx, &sets, self.tangent_chain)
    }
}

/// The kind and size of one chamfer set.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ChamferKind {
    /// The same distance on both faces (length).
    Equal(Expr),
    /// `d1` on `face`, `d2` on the other face of each edge.
    TwoDistances {
        /// Distance on `face` (length).
        d1: Expr,
        /// Distance on the other face (length).
        d2: Expr,
        /// The reference face; it must border every edge of the set.
        face: TopoRef,
    },
    /// `d` on `face`, the chamfer plane at `angle` from it.
    DistanceAngle {
        /// Distance on `face` (length).
        d: Expr,
        /// Angle from `face`, in (0°, 90°).
        angle: Expr,
        /// The reference face; it must border every edge of the set.
        face: TopoRef,
    },
}

impl ChamferKind {
    fn face(&self) -> Option<&TopoRef> {
        match self {
            ChamferKind::Equal(_) => None,
            ChamferKind::TwoDistances { face, .. } | ChamferKind::DistanceAngle { face, .. } => {
                Some(face)
            }
        }
    }
}

/// Edges chamfered with one kind and size.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChamferSet {
    /// Picked edges (set-valued; may span bodies).
    pub edges: Vec<TopoRef>,
    /// Kind and size.
    pub kind: ChamferKind,
}

/// `solid.chamfer`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Chamfer {
    /// Edge sets, each with its kind.
    pub sets: Vec<ChamferSet>,
    /// Grow each pick along its tangent chain.
    #[serde(default = "yes")]
    pub tangent_chain: bool,
}

impl Chamfer {
    /// An equal-distance chamfer of `edges` (`distance` such as `"2 mm"`), tangent chains on.
    pub fn equal(edges: Vec<TopoRef>, distance: &str) -> Chamfer {
        Chamfer {
            sets: vec![ChamferSet {
                edges,
                kind: ChamferKind::Equal(inline(distance)),
            }],
            tangent_chain: true,
        }
    }

    /// The edges this chamfer passes to the kernel, per body (tangent chains grown).
    ///
    /// # Errors
    /// As [`Fillet::plan`].
    pub fn plan(&self, ctx: &RecomputeCtx<'_>) -> Result<Vec<BodyEdges>, FeatureError> {
        let sets: Vec<&[TopoRef]> = self.sets.iter().map(|s| s.edges.as_slice()).collect();
        plan_edges(ctx, &sets, self.tangent_chain)
    }
}

// ---- edge plan ------------------------------------------------------------------------------

/// One edge of a blend, as passed to the kernel.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PlannedEdge {
    /// The edge (input index space of the body).
    pub edge: EdgeId,
    /// The set it belongs to.
    pub set: usize,
    /// The selection key of a picked edge; `None` for an edge added by tangent propagation.
    pub key: Option<u32>,
}

/// The blend edges of one body: picks first (in pick order), then propagated edges.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BodyEdges {
    /// The body (as it is just before the feature).
    pub body: BodyId,
    /// Its edges.
    pub edges: Vec<PlannedEdge>,
}

impl BodyEdges {
    /// Just the edge ids, in kernel order.
    pub fn edge_ids(&self) -> Vec<EdgeId> {
        self.edges.iter().map(|e| e.edge).collect()
    }
}

fn plan_edges(
    ctx: &RecomputeCtx<'_>,
    sets: &[&[TopoRef]],
    tangent_chain: bool,
) -> Result<Vec<BodyEdges>, FeatureError> {
    let mut plan: Vec<BodyEdges> = Vec::new();
    for (set, refs) in sets.iter().enumerate() {
        for (body, subs) in resolve_subs(ctx, refs, SetPolicy::DropMissing)? {
            let slot = match plan.iter().position(|p| p.body == body) {
                Some(i) => i,
                None => {
                    plan.push(BodyEdges {
                        body,
                        edges: Vec::new(),
                    });
                    plan.len() - 1
                }
            };
            for (sub, key) in subs {
                if sub.kind != SubKind::Edge {
                    return Err(invalid("edges", "select edges"));
                }
                let edges = &mut plan[slot].edges;
                if !edges.iter().any(|e| e.edge.0 == sub.index) {
                    edges.push(PlannedEdge {
                        edge: EdgeId(sub.index),
                        set,
                        key: Some(key),
                    });
                }
            }
        }
    }
    if tangent_chain {
        let k = op_kernel(ctx);
        for p in &mut plan {
            let (shape, _) = input_solid(ctx, p.body)?;
            let picked = p.edges.clone();
            for seed in picked {
                let chain =
                    tangent_chain_edges(k, shape, &[seed.edge], CHAIN_ANGLE).map_err(kerr)?;
                for e in chain {
                    if !p.edges.iter().any(|x| x.edge == e) {
                        p.edges.push(PlannedEdge {
                            edge: e,
                            set: seed.set,
                            key: None,
                        });
                    }
                }
            }
        }
    }
    plan.retain(|p| !p.edges.is_empty());
    if plan.is_empty() {
        return Err(invalid("edges", "no edge of the selection is left"));
    }
    Ok(plan)
}

// ---- compute --------------------------------------------------------------------------------

/// Names a blend result (picked edges by selection key) and records the body change.
fn emit(
    ctx: &RecomputeCtx<'_>,
    p: &BodyEdges,
    naming: &BodyNaming,
    r: OpResult,
    out: &mut FeatureOutput,
) -> Result<(), FeatureError> {
    let k = op_kernel(ctx);
    let mut b = ctx.naming_builder();
    b.from_history(&[(0, OperandNaming::Named(naming))], &r.history);
    for e in &p.edges {
        if let Some(key) = e.key {
            b.selection(
                0,
                SubShape {
                    kind: SubKind::Edge,
                    index: e.edge.0,
                },
                key,
            );
        }
    }
    let naming = finish_naming(b, k, &r.shape, out)?;
    out.bodies.push(BodyChange {
        id: p.body,
        change: BodyChangeKind::Modified {
            geom: BodyGeom::Solid(amg_doc::stub::Shape::from_brep(r.shape)),
            naming: Some(naming),
        },
    });
    Ok(())
}

fn finite_positive(field: &'static str, v: f64) -> Result<f64, FeatureError> {
    if v.is_finite() && v > 0.0 {
        Ok(v)
    } else {
        Err(invalid(field, format!("must be larger than 0, got {v}")))
    }
}

/// The kernel spec of one fillet radius.
fn fillet_spec(ctx: &RecomputeCtx<'_>, r: &FilletRadius) -> Result<FilletSpec, FeatureError> {
    Ok(match r {
        FilletRadius::Constant(e) => {
            FilletSpec::Constant(finite_positive("radius", ctx.eval_length(e)?)?)
        }
        FilletRadius::Chord(e) => FilletSpec::Chord(finite_positive("chord", ctx.eval_length(e)?)?),
        FilletRadius::Variable(v) => {
            let mut law: Vec<(f64, f64)> = Vec::with_capacity(v.len() + 2);
            for s in v {
                let t = ctx.eval_scalar(&s.at)?;
                if !(0.0..=1.0).contains(&t) {
                    return Err(invalid("radius", format!("position {t} is outside 0..1")));
                }
                law.push((t, finite_positive("radius", ctx.eval_length(&s.radius)?)?));
            }
            law.sort_by(|a, b| a.0.total_cmp(&b.0));
            if law.windows(2).any(|w| w[1].0 - w[0].0 <= 1e-9) {
                return Err(invalid("radius", "two radius positions coincide"));
            }
            let (Some(&first), Some(&last)) = (law.first(), law.last()) else {
                return Err(invalid("radius", "a variable radius needs a position"));
            };
            if first.0 > 1e-9 {
                law.insert(0, (0.0, first.1));
            } else {
                law[0].0 = 0.0;
            }
            if last.0 < 1.0 - 1e-9 {
                law.push((1.0, last.1));
            } else if let Some(l) = law.last_mut() {
                l.0 = 1.0;
            }
            FilletSpec::Variable(law)
        }
    })
}

fn set_exprs_fillet(r: &FilletRadius) -> Vec<&Expr> {
    match r {
        FilletRadius::Constant(e) | FilletRadius::Chord(e) => vec![e],
        FilletRadius::Variable(v) => v.iter().flat_map(|s| [&s.at, &s.radius]).collect(),
    }
}

/// Body and B-rep entity inputs are optional (SOL-07): a body consumed by a combine or split
/// off since the reference was made has no producer of its own any more, but its faces live
/// on in the successor body, which DOC-06 resolution finds through body lineage (TNP-23).
pub(crate) fn lineage_optional(dep: &Dependency) -> bool {
    matches!(
        dep,
        Dependency::Body(_) | Dependency::Entity(EntityRef::Topo(_))
    )
}

fn edge_inputs<'a>(refs: impl Iterator<Item = &'a TopoRef>) -> Vec<Dependency> {
    let mut d: Vec<Dependency> = Vec::new();
    for r in refs {
        let b = Dependency::Body(r.body);
        if !d.contains(&b) {
            d.push(b);
        }
        d.push(Dependency::Entity(EntityRef::Topo(r.clone())));
    }
    d
}

impl Feature for Fillet {
    fn inputs(&self) -> Vec<Dependency> {
        edge_inputs(self.sets.iter().flat_map(|s| &s.edges))
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        lineage_optional(dep)
    }

    fn exprs(&self) -> Vec<&Expr> {
        self.sets
            .iter()
            .flat_map(|s| set_exprs_fillet(&s.radius))
            .collect()
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        let mut v = Vec::new();
        for s in &mut self.sets {
            match &mut s.radius {
                FilletRadius::Constant(e) | FilletRadius::Chord(e) => v.push((e, Dim::LENGTH)),
                FilletRadius::Variable(pts) => {
                    for p in pts {
                        v.push((&mut p.at, Dim::NONE));
                        v.push((&mut p.radius, Dim::LENGTH));
                    }
                }
            }
        }
        v
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self
            .sets
            .iter()
            .any(|s| matches!(&s.radius, FilletRadius::Variable(v) if v.is_empty()))
        {
            return Err(invalid("radius", "a variable radius needs a position"));
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        if self.corner == CornerType::Setback {
            return Err(SolidError::NotYet {
                what: "setback fillet corners",
            }
            .into());
        }
        let specs = self
            .sets
            .iter()
            .map(|s| fillet_spec(ctx, &s.radius))
            .collect::<Result<Vec<_>, _>>()?;
        let plan = self.plan(ctx)?;
        let k = op_kernel(ctx);
        let mut out = FeatureOutput::default();
        for p in &plan {
            ctx.check_cancelled()?;
            let (shape, naming) = input_solid(ctx, p.body)?;
            let edges: Vec<(EdgeId, FilletSpec)> = p
                .edges
                .iter()
                .map(|e| (e.edge, specs[e.set].clone()))
                .collect();
            let r = with_job(ctx, |job| {
                k.fillet(shape, &edges, &FilletOpts::default(), job)
            })
            .map_err(|e| kernel_failure(ctx, Some(p.body), e, &mut out))?;
            emit(ctx, p, naming, r, &mut out)?;
        }
        Ok(out)
    }
}

register_feature!(Fillet, "solid.fillet", version = 1);

/// The reference face of a two-distance or distance-angle set, checked against the set's
/// edges on `body`.
fn chamfer_face(
    ctx: &RecomputeCtx<'_>,
    face: &TopoRef,
    body: BodyId,
    shape: &amg_brep::Shape,
    edges: &[EdgeId],
) -> Result<FaceId, FeatureError> {
    let r = ctx.resolve(face)?;
    if r.kind != TopoKind::Face || r.body != body {
        return Err(invalid(
            "face",
            "the reference face must be a face of the chamfered body",
        ));
    }
    let f = FaceId(r.index);
    if let Some(e) = edges.iter().find(|e| !shape.edge_faces(**e).contains(&f)) {
        return Err(invalid(
            "face",
            format!("the reference face does not border edge {}", e.0),
        ));
    }
    Ok(f)
}

impl Feature for Chamfer {
    fn inputs(&self) -> Vec<Dependency> {
        edge_inputs(
            self.sets
                .iter()
                .flat_map(|s| s.edges.iter().chain(s.kind.face())),
        )
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        lineage_optional(dep)
    }

    fn exprs(&self) -> Vec<&Expr> {
        let mut v = Vec::new();
        for s in &self.sets {
            match &s.kind {
                ChamferKind::Equal(d) => v.push(d),
                ChamferKind::TwoDistances { d1, d2, .. } => v.extend([d1, d2]),
                ChamferKind::DistanceAngle { d, angle, .. } => v.extend([d, angle]),
            }
        }
        v
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        let mut v = Vec::new();
        for s in &mut self.sets {
            match &mut s.kind {
                ChamferKind::Equal(d) => v.push((d, Dim::LENGTH)),
                ChamferKind::TwoDistances { d1, d2, .. } => {
                    v.push((d1, Dim::LENGTH));
                    v.push((d2, Dim::LENGTH));
                }
                ChamferKind::DistanceAngle { d, angle, .. } => {
                    v.push((d, Dim::LENGTH));
                    v.push((angle, Dim::ANGLE));
                }
            }
        }
        v
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self
            .sets
            .iter()
            .filter_map(|s| s.kind.face())
            .any(|f| f.kind != TopoKind::Face)
        {
            return Err(invalid("face", "the reference must be a face"));
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let plan = self.plan(ctx)?;
        let k = op_kernel(ctx);
        let mut out = FeatureOutput::default();
        for p in &plan {
            ctx.check_cancelled()?;
            let (shape, naming) = input_solid(ctx, p.body)?;
            let mut specs: Vec<Option<ChamferSpec>> = vec![None; self.sets.len()];
            for (i, s) in self.sets.iter().enumerate() {
                let set_edges: Vec<EdgeId> = p
                    .edges
                    .iter()
                    .filter(|e| e.set == i)
                    .map(|e| e.edge)
                    .collect();
                if set_edges.is_empty() {
                    continue;
                }
                specs[i] = Some(match &s.kind {
                    ChamferKind::Equal(d) => {
                        ChamferSpec::Equal(finite_positive("distance", ctx.eval_length(d)?)?)
                    }
                    ChamferKind::TwoDistances { d1, d2, face } => ChamferSpec::TwoDistances {
                        d1: finite_positive("distance", ctx.eval_length(d1)?)?,
                        d2: finite_positive("distance2", ctx.eval_length(d2)?)?,
                        face: chamfer_face(ctx, face, p.body, shape, &set_edges)?,
                    },
                    ChamferKind::DistanceAngle { d, angle, face } => {
                        let a = ctx.eval_angle(angle)?;
                        if !a.is_finite() || a <= 0.0 || a >= std::f64::consts::FRAC_PI_2 {
                            return Err(invalid("angle", "the angle must be between 0° and 90°"));
                        }
                        ChamferSpec::DistanceAngle {
                            d: finite_positive("distance", ctx.eval_length(d)?)?,
                            angle: a,
                            face: chamfer_face(ctx, face, p.body, shape, &set_edges)?,
                        }
                    }
                });
            }
            let edges: Vec<(EdgeId, ChamferSpec)> = p
                .edges
                .iter()
                .filter_map(|e| specs[e.set].clone().map(|s| (e.edge, s)))
                .collect();
            let r = with_job(ctx, |job| k.chamfer(shape, &edges, job))
                .map_err(|e| kernel_failure(ctx, Some(p.body), e, &mut out))?;
            emit(ctx, p, naming, r, &mut out)?;
        }
        Ok(out)
    }
}

register_feature!(Chamfer, "solid.chamfer", version = 1);

#[cfg(test)]
mod tests {
    use super::*;
    use amg_doc::{ComputeMode, Document, FeatureId};

    fn spec(r: &FilletRadius) -> Result<FilletSpec, FeatureError> {
        let doc = Document::new("spec");
        let ctx = RecomputeCtx::new(
            doc.design(),
            doc.evaluated(),
            FeatureId(1),
            doc.design().root(),
            ComputeMode::Full,
        );
        fillet_spec(&ctx, r)
    }

    fn pt(at: &str, r: &str) -> VarRadius {
        VarRadius {
            at: inline(at),
            radius: inline(r),
        }
    }

    #[test]
    fn variable_laws_are_sorted_and_held_at_the_ends() {
        let law = spec(&FilletRadius::Variable(vec![
            pt("0.75", "3 mm"),
            pt("0.25", "2 mm"),
        ]));
        assert_eq!(
            law.unwrap(),
            FilletSpec::Variable(vec![(0.0, 2.0), (0.25, 2.0), (0.75, 3.0), (1.0, 3.0)])
        );
        let one = spec(&FilletRadius::Variable(vec![pt("0.5", "2 mm")])).unwrap();
        assert_eq!(
            one,
            FilletSpec::Variable(vec![(0.0, 2.0), (0.5, 2.0), (1.0, 2.0)])
        );
        let exact = spec(&FilletRadius::Variable(vec![
            pt("0", "2 mm"),
            pt("1", "4 mm"),
        ]));
        assert_eq!(
            exact.unwrap(),
            FilletSpec::Variable(vec![(0.0, 2.0), (1.0, 4.0)])
        );
    }

    #[test]
    fn bad_radii_are_field_errors() {
        for r in [
            FilletRadius::Constant(inline("0 mm")),
            FilletRadius::Chord(inline("-2 mm")),
            FilletRadius::Variable(vec![pt("1.5", "2 mm")]),
            FilletRadius::Variable(Vec::new()),
        ] {
            assert!(
                matches!(spec(&r), Err(FeatureError::InvalidValue { .. })),
                "{r:?}"
            );
        }
        assert_eq!(
            spec(&FilletRadius::Chord(inline("3 mm"))).unwrap(),
            FilletSpec::Chord(3.0)
        );
    }
}
