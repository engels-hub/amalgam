//! `solid.extrude` (S04 §4, note §3): every start (profile plane, offset, planar object),
//! direction (one side, two sides, symmetric) and extent (distance, to object, all), a taper
//! per side, sketch-region, text and planar-face profiles. Thin extrudes are SOL-18.
//!
//! Naming (naming note §5.2): laterals `Gen { self, Lateral#side, SketchCurve(sketch, entity) }`
//! from the builder tags (face profiles: the body edge's name), caps `StartCap#side` /
//! `EndCap#side` from `Region(p)` (face profiles: `Sel(key)`); `side` is 1 for the second side
//! of two-sided and symmetric extrudes. Regions of one sketch that share curves, and the two
//! sides of a profile, are fused (and unified) into one tool solid first (note §3.4).

pub(crate) mod dialog;
pub(crate) mod sides;

use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, InputSub, Relation, Shape, ShapeHistory, SubKind, SubShape,
};
use amg_core::{Transform, Vec3};
use amg_doc::naming::OperandNaming;
use amg_doc::{
    BodyNaming, ContextBodies, Dependency, Dim, EntityRef, Expr, ExprSrc, Feature, FeatureError,
    FeatureOutput, RecomputeCtx, register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{OpTarget, SolidTool, ToolBody, apply_body_op, finish_naming, op_kernel};
use super::common::{simplify, with_job};
use super::error::{SolidError, kerr};
use super::extent::{
    Direction, Extent, ExtentSide, Start, Target, ThinSpec, extent_refs, resolve_target, side_exprs,
};
use super::profile::{ProfileFace, profile_faces};
use sides::{Side, SideExtent, side_solid};

/// `solid.extrude`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Extrude {
    /// Profile regions (`EntityRef::Profile`) or planar faces (`EntityRef::Topo` faces).
    pub profiles: Vec<EntityRef>,
    /// Where the extent starts.
    #[serde(default)]
    pub start: Start,
    /// One side, two sides or symmetric.
    #[serde(default)]
    pub direction: Direction,
    /// The first (or only) side; for symmetric extrudes its distance and taper apply to both.
    pub side1: ExtentSide,
    /// The second side; `Some` iff `direction == TwoSides`.
    #[serde(default)]
    pub side2: Option<ExtentSide>,
    /// Extrude against the profile normal (side 1; side 2 goes the other way).
    #[serde(default)]
    pub flip: bool,
    /// Thin extrude (SOL-18).
    #[serde(default)]
    pub thin: Option<ThinSpec>,
    /// Body operation.
    #[serde(default)]
    pub target: OpTarget,
}

fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
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

    /// The evaluated sides; `u` is side 1's direction.
    fn sides<'a>(&self, ctx: &RecomputeCtx<'a>, u: Vec3) -> Result<Vec<Side<'a>>, FeatureError> {
        match self.direction {
            Direction::OneSide => Ok(vec![side_of(ctx, &self.side1, 0, u)?]),
            Direction::TwoSides => {
                let s2 = self
                    .side2
                    .as_ref()
                    .ok_or_else(|| invalid("side2", "a two-sided extrude needs a second side"))?;
                Ok(vec![
                    side_of(ctx, &self.side1, 0, u)?,
                    side_of(ctx, s2, 1, -u)?,
                ])
            }
            Direction::Symmetric { whole_length } => {
                let Extent::Distance(e) = &self.side1.extent else {
                    return Err(invalid("extent", "a symmetric extrude needs a distance"));
                };
                let d = distance_value(ctx, e)?.abs();
                let h = if whole_length { d } else { d / 2.0 };
                let taper = ctx.eval_angle(&self.side1.taper)?;
                Ok([(0, u), (1, -u)]
                    .into_iter()
                    .map(|(index, dir)| Side {
                        index,
                        dir,
                        extent: SideExtent::Length(h),
                        taper,
                    })
                    .collect())
            }
        }
    }

    /// The signed distance (along `n0`) from each profile's plane to the start plane.
    fn start_offset(
        &self,
        ctx: &RecomputeCtx<'_>,
        f: &ProfileFace,
        n0: Vec3,
        target: Option<&Target<'_>>,
    ) -> Result<f64, FeatureError> {
        match (&self.start, target) {
            (Start::Profile, _) => Ok(0.0),
            (Start::Offset(e), _) => ctx.eval_length(e),
            (Start::Object { offset, target: r }, Some(t)) => {
                let off = ctx.eval_length(offset)?;
                let at = match t {
                    Target::Point(p) => *p,
                    Target::Plane { plane, .. }
                        if plane.normal.get().dot(n0).abs() >= 1.0 - 1e-9 =>
                    {
                        plane.origin
                    }
                    _ => {
                        ctx.highlight([r.clone()]);
                        return Err(invalid(
                            "start",
                            "the start object must be a plane or planar face parallel to the profile, or a point",
                        ));
                    }
                };
                Ok((at - f.plane.origin).dot(n0) + off)
            }
            (Start::Object { .. }, None) => Err(invalid("start", "the start object is missing")),
        }
    }
}

fn distance_value(ctx: &RecomputeCtx<'_>, e: &Expr) -> Result<f64, FeatureError> {
    let d = ctx.eval_length(e)?;
    if !d.is_finite() || d.abs() <= ctx.tolerance().linear {
        return Err(invalid("distance", "the distance must not be zero"));
    }
    Ok(d)
}

/// One stored side, evaluated: a negative distance reverses the side (as in SOL-01).
fn side_of<'a>(
    ctx: &RecomputeCtx<'a>,
    s: &ExtentSide,
    index: u32,
    dir: Vec3,
) -> Result<Side<'a>, FeatureError> {
    let taper = ctx.eval_angle(&s.taper)?;
    let (dir, extent) = match &s.extent {
        Extent::Distance(e) => {
            let d = distance_value(ctx, e)?;
            if d < 0.0 {
                (-dir, SideExtent::Length(-d))
            } else {
                (dir, SideExtent::Length(d))
            }
        }
        Extent::ToObject {
            target,
            offset,
            extend,
        } => (
            dir,
            SideExtent::ToObject {
                target: resolve_target(ctx, target, "extent")?,
                r: target.clone(),
                offset: ctx.eval_length(offset)?,
                extend: *extend,
            },
        ),
        Extent::All => (dir, SideExtent::All),
        Extent::Full => return Err(invalid("extent", "a full turn is a revolve extent")),
    };
    Ok(Side {
        index,
        dir,
        extent,
        taper,
    })
}

/// `f` moved along `n0` by `t` (shares TShapes, keeps every index and tag).
fn moved(f: &ProfileFace, n0: Vec3, t: f64) -> Result<ProfileFace, FeatureError> {
    let mut g = f.clone();
    g.face = f
        .face
        .transformed(&Transform::from_translation(n0 * t))
        .map_err(kerr)?;
    g.plane.origin += n0 * t;
    Ok(g)
}

/// The solids of one group fused and unified into one solid (shared caps and laterals
/// vanish; coplanar laterals of two sides merge, primary side 0).
pub(crate) fn fuse_group(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    parts: Vec<(Shape, BodyNaming)>,
    out: &mut FeatureOutput,
) -> Result<(Shape, BodyNaming), FeatureError> {
    let mut parts = parts;
    if parts.len() == 1
        && let Some(p) = parts.pop()
    {
        return Ok(p);
    }
    let shapes: Vec<Shape> = parts.iter().map(|(s, _)| s.clone()).collect();
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
    parts: Vec<(Shape, BodyNaming)>,
    out: &mut FeatureOutput,
) -> Result<(Shape, BodyNaming), FeatureError> {
    let mut parts = parts;
    if parts.len() == 1
        && let Some(p) = parts.pop()
    {
        return Ok(p);
    }
    let shapes: Vec<Shape> = parts.iter().map(|(s, _)| s.clone()).collect();
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
        if self.thin.is_some() {
            return Err(SolidError::NotYet {
                what: "thin extrude",
            }
            .into());
        }
        let faces = profile_faces(ctx, &self.profiles)?;
        let n0 = faces[0].plane.normal.get();
        let u = if self.flip { -n0 } else { n0 };
        let sides = self.sides(ctx, u)?;
        let start_target = match &self.start {
            Start::Object { target, .. } => Some(resolve_target(ctx, target, "start")?),
            Start::Profile | Start::Offset(_) => None,
        };
        let k = op_kernel(ctx);
        // Naming warnings of the tool are not feature warnings (the tool's history is complete
        // by construction); a partial history shows up again in the body op.
        let mut scratch = FeatureOutput::default();
        let mut groups: Vec<Vec<(Shape, BodyNaming)>> = Vec::new();
        for f in &faces {
            ctx.check_cancelled()?;
            let t = self.start_offset(ctx, f, n0, start_target.as_ref())?;
            let at = if t.abs() > ctx.tolerance().linear {
                moved(f, n0, t)?
            } else {
                f.clone()
            };
            let g = f.group as usize;
            if groups.len() <= g {
                groups.resize_with(g + 1, Vec::new);
            }
            for s in &sides {
                groups[g].push(side_solid(ctx, k, &at, s, &mut scratch)?);
            }
        }
        let mut solids = Vec::with_capacity(groups.len());
        for g in groups {
            if !g.is_empty() {
                solids.push(fuse_group(ctx, k, g, &mut scratch)?);
            }
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

    /// "All" measures every solid body of the context component, participant or not.
    fn context_reads(&self) -> Option<ContextBodies> {
        std::iter::once(&self.side1)
            .chain(&self.side2)
            .any(|s| s.extent == Extent::All)
            .then_some(ContextBodies::Context)
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
            return Err(invalid("profiles", "select at least one profile"));
        }
        if self.side2.is_some() != (self.direction == Direction::TwoSides) {
            return Err(invalid(
                "side2",
                "a second side is set exactly for two-sided extrudes",
            ));
        }
        if std::iter::once(&self.side1)
            .chain(&self.side2)
            .any(|s| s.extent == Extent::Full)
        {
            return Err(invalid("extent", "a full turn is a revolve extent"));
        }
        if matches!(self.direction, Direction::Symmetric { .. })
            && !matches!(self.side1.extent, Extent::Distance(_))
        {
            return Err(invalid("extent", "a symmetric extrude needs a distance"));
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
