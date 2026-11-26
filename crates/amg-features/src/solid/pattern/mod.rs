//! `solid.pattern` and `solid.mirror` (S04 §5, SOL-08): rectangular and circular patterns of
//! bodies and features with suppressed instances and "identical" compute, and mirrors of bodies
//! (new body or join) and features.
//!
//! - **Body patterns** copy each source body by [`Shape::transformed`] (shared TShapes) as a new
//!   body named `instanced(self, k)` (naming note §4.6).
//! - **Feature patterns** build each seed's tool **once** ([`tool_of`], with the seed as the
//!   naming feature, against the state just before the pattern), place instance `k` with
//!   [`Shape::transformed`], rename it `instanced(self, k)`, and apply the body operation once
//!   with all instances in one compound: one boolean per participant ([`apply_body_op`]).
//! - **Mirrors** reflect through [`BrepKernel::mirrored`] (R-S04-6: a geometry copy with face
//!   orientations fixed), named through its 1:1 history and then `Inst { self, 1 }`.
//!
//! Instance numbers (naming note §5.2): rectangular `k = i + (j << 16)` with `i`, `j` the
//! positions along direction 1 and 2; circular `k` = the position along the pattern. The seed
//! is instance 0 and is never produced again (the source body or seed feature stays). A
//! position on the negative side of a symmetric pattern sets the field's top bit
//! (`0x8000 | p` per rectangular field, `0x8000_0000 | p` for circular), so instance numbers
//! stay stable when the count changes (TNP-17/18).

mod dialog;
mod mirror;

use amg_brep::{BrepKernel, Shape};
use amg_core::{Line3, Transform, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, BodyNaming, Dependency, Dim, EntityRef, Expr,
    Feature, FeatureError, FeatureId, FeatureOutput, OccurrenceId, RecomputeCtx, TopoRef,
    register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{OpTarget, ToolBody, apply_body_op, input_solid, op_kernel, tool_of};
use super::error::{SolidError, kerr};
use super::extrude::compound_of;
use crate::construct::RefInput;
use crate::construct::math::{axis_of, line_of};
use crate::construct::resolve::resolve_input;

pub use mirror::Mirror;

/// Most instances one pattern may produce (a guard against typos such as a count of 1e6).
pub const MAX_INSTANCES: usize = 10_000;

// ---- data -------------------------------------------------------------------------------------

/// What a pattern or mirror copies.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum PatternObjects {
    /// Solid bodies, copied as new bodies.
    Bodies(Vec<BodyId>),
    /// Additive/subtractive features (their tools, [`tool_of`]).
    Features(Vec<FeatureId>),
    /// Faces of a body (SOL-23).
    Faces(Vec<TopoRef>),
    /// Component occurrences (SOL-24).
    Occurrences(Vec<OccurrenceId>),
}

/// How the instances of one direction are spaced.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Spacing {
    /// The distance from the seed to the last instance (mm); instances are spread evenly.
    Extent(Expr),
    /// The distance between neighbours (mm).
    Spacing(Expr),
}

/// One direction of a rectangular pattern.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PatternDir {
    /// The direction: a straight edge, axis, sketch line, or the axis of a cylindrical face.
    pub dir: RefInput,
    /// Instances along it, the seed included (per side when `symmetric`); ≥ 1, integral.
    pub count: Expr,
    /// Their spacing.
    pub spacing: Spacing,
    /// Reverse the direction.
    #[serde(default)]
    pub flip: bool,
    /// `count` instances on both sides of the seed (`2 count − 1` in all).
    #[serde(default)]
    pub symmetric: bool,
}

/// The angular span of a circular pattern.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum AngleSpan {
    /// A full turn: `count` instances `360° / count` apart.
    Full,
    /// The angle from the seed to the last instance; instances are spread evenly.
    Extent(Expr),
    /// The angle between neighbours.
    Spacing(Expr),
}

/// The orientation of instances along a path (SOL-23).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum PathOrientation {
    /// Every instance keeps the seed's orientation.
    #[default]
    Identical,
    /// Instances follow the path's frame.
    PathDirection,
}

/// The pattern's layout.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[allow(
    clippy::large_enum_variant,
    reason = "the S04 Interfaces shape; one value per feature, never in bulk"
)]
pub enum PatternKind {
    /// One or two linear directions.
    Rectangular {
        /// Direction 1.
        d1: PatternDir,
        /// Direction 2 (`None`: a one-dimensional pattern).
        #[serde(default)]
        d2: Option<PatternDir>,
    },
    /// Rotation about an axis.
    Circular {
        /// The axis (an axis, straight edge, or a cylindrical/conical face or circular edge).
        axis: RefInput,
        /// Instances, the seed included (per side when `symmetric`); ≥ 1, integral.
        count: Expr,
        /// The angular span.
        span: AngleSpan,
        /// `count` instances on both sides of the seed (ignored for a full turn).
        #[serde(default)]
        symmetric: bool,
    },
    /// Along a path (SOL-23).
    Path {
        /// The path curves.
        path: Vec<EntityRef>,
        /// Instances.
        count: Expr,
        /// Their spacing.
        spacing: Spacing,
        /// Start position as a fraction of the path.
        start: Expr,
        /// Instance orientation.
        orientation: PathOrientation,
        /// Reverse the path.
        flip: bool,
    },
}

/// How feature instances are computed.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ComputeType {
    /// The seed's tool is built once and moved (fast; SOL-08).
    #[default]
    Identical,
    /// The seed's inputs are moved per instance and each tool is rebuilt (SOL-23).
    Adjust,
}

/// `solid.pattern`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pattern {
    /// What is patterned.
    pub objects: PatternObjects,
    /// The layout.
    pub kind: PatternKind,
    /// Instance numbers left out (`0`, the seed, is ignored).
    #[serde(default)]
    pub suppressed: Vec<u32>,
    /// How feature instances are computed.
    #[serde(default)]
    pub compute: ComputeType,
    /// Feature patterns: the body operation of the instances (`None`: the seeds' own targets,
    /// which must then agree). Must be `None` for body patterns.
    #[serde(default)]
    pub target: Option<OpTarget>,
}

pub(crate) fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

pub(crate) fn not_yet(what: &'static str) -> FeatureError {
    SolidError::NotYet { what }.into()
}

/// `InvalidValue` for an empty body or feature selection.
pub(crate) fn require_objects(o: &PatternObjects) -> Result<(), FeatureError> {
    match o {
        PatternObjects::Bodies(b) if b.is_empty() => {
            Err(invalid("bodies", "select at least one body"))
        }
        PatternObjects::Features(f) if f.is_empty() => {
            Err(invalid("features", "select at least one feature"))
        }
        _ => Ok(()),
    }
}

/// The dependency of a reference input (as `construct` does for its features).
pub(crate) fn ref_dependency(r: &RefInput) -> Option<Dependency> {
    match r {
        RefInput::Origin(_) => None,
        RefInput::Ref(EntityRef::Construction(c)) => Some(Dependency::Construction(*c)),
        RefInput::Ref(other) => Some(Dependency::Entity(other.clone())),
    }
}

// ---- instance layout ------------------------------------------------------------------------

/// One evaluated direction: unit vector, step (mm), positions.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct LinearDir {
    pub(crate) dir: Vec3,
    pub(crate) step: f64,
    pub(crate) count: u32,
    pub(crate) symmetric: bool,
}

/// An evaluated layout, before suppression.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Layout {
    Rectangular {
        d1: LinearDir,
        d2: Option<LinearDir>,
    },
    Circular {
        axis: Line3,
        step: f64,
        count: u32,
        symmetric: bool,
    },
}

/// Signed positions `0, 1, …, n−1` (and `−1, …, −(n−1)` when symmetric), seed first.
fn positions(count: u32, symmetric: bool) -> Vec<i64> {
    let n = i64::from(count);
    let mut v: Vec<i64> = (0..n).collect();
    if symmetric {
        v.extend((1..n).map(|p| -p));
    }
    v
}

/// The instance-number field of position `p`: `p`, or `flag | −p` on the negative side.
fn field(p: i64, flag: u32) -> u32 {
    let m = u32::try_from(p.unsigned_abs()).unwrap_or(u32::MAX);
    if p < 0 { flag | m } else { m }
}

impl Layout {
    /// Every instance `(k, transform)` but the seed, in pattern order (direction 1 fastest).
    pub(crate) fn instances(&self) -> Vec<(u32, Transform)> {
        let mut v = Vec::new();
        match self {
            Layout::Rectangular { d1, d2 } => {
                let p1 = positions(d1.count, d1.symmetric);
                let p2 = d2
                    .as_ref()
                    .map_or_else(|| vec![0], |d| positions(d.count, d.symmetric));
                for &j in &p2 {
                    for &i in &p1 {
                        if i == 0 && j == 0 {
                            continue;
                        }
                        let mut t = d1.dir * (d1.step * i as f64);
                        if let Some(d) = d2 {
                            t += d.dir * (d.step * j as f64);
                        }
                        let k = field(i, 0x8000) | (field(j, 0x8000) << 16);
                        v.push((k, Transform::from_translation(t)));
                    }
                }
            }
            Layout::Circular {
                axis,
                step,
                count,
                symmetric,
            } => {
                for p in positions(*count, *symmetric) {
                    if p == 0 {
                        continue;
                    }
                    let a = step * p as f64;
                    let o = axis.point;
                    let t = Transform::from_translation(o)
                        * Transform::from_axis_angle(axis.dir.get(), a)
                        * Transform::from_translation(-o);
                    v.push((field(p, 0x8000_0000), t));
                }
            }
        }
        v
    }

    /// How many instances the layout has, the seed included.
    pub(crate) fn len(&self) -> usize {
        let n = |c: u32, s: bool| {
            let c = c as usize;
            if s { 2 * c - 1 } else { c }
        };
        match self {
            Layout::Rectangular { d1, d2 } => {
                n(d1.count, d1.symmetric) * d2.as_ref().map_or(1, |d| n(d.count, d.symmetric))
            }
            Layout::Circular {
                count, symmetric, ..
            } => n(*count, *symmetric),
        }
    }
}

/// An instance count: integral and ≥ 1.
fn count_of(ctx: &RecomputeCtx<'_>, e: &Expr, field: &'static str) -> Result<u32, FeatureError> {
    let x = ctx.eval_scalar(e)?;
    let r = x.round();
    if !x.is_finite() || (x - r).abs() > 1e-9 || r < 1.0 {
        return Err(invalid(
            field,
            format!("the count must be a whole number of at least 1, not {x}"),
        ));
    }
    if r > MAX_INSTANCES as f64 {
        return Err(invalid(field, format!("at most {MAX_INSTANCES} instances")));
    }
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "checked integral, 1 ≤ r ≤ MAX_INSTANCES"
    )]
    Ok(r as u32)
}

/// The line of a direction or axis input: a straight edge, line or axis first, else the axis
/// of a revolved face or circular edge. A failure highlights the reference.
fn line_input(
    ctx: &RecomputeCtx<'_>,
    r: &RefInput,
    field: &'static str,
) -> Result<Line3, FeatureError> {
    let g = resolve_input(ctx, r)?.geom;
    line_of(&g, ctx.tolerance())
        .or_else(|_| axis_of(&g))
        .map_err(|e| {
            ctx.highlight(r.entity().cloned());
            invalid(field, format!("not a direction: {e}"))
        })
}

fn linear_dir(
    ctx: &RecomputeCtx<'_>,
    d: &PatternDir,
    field: &'static str,
) -> Result<LinearDir, FeatureError> {
    let line = line_input(ctx, &d.dir, field)?;
    let count = count_of(ctx, &d.count, field)?;
    let step = match &d.spacing {
        Spacing::Spacing(e) => ctx.eval_length(e)?,
        Spacing::Extent(e) => {
            let ext = ctx.eval_length(e)?;
            if count > 1 {
                ext / f64::from(count - 1)
            } else {
                0.0
            }
        }
    };
    if !step.is_finite() {
        return Err(invalid(field, "the spacing is not finite"));
    }
    let dir = if d.flip {
        -line.dir.get()
    } else {
        line.dir.get()
    };
    Ok(LinearDir {
        dir,
        step,
        count,
        symmetric: d.symmetric,
    })
}

impl Pattern {
    /// The evaluated layout (directions resolved, counts and spacings evaluated).
    pub(crate) fn layout(&self, ctx: &RecomputeCtx<'_>) -> Result<Layout, FeatureError> {
        let layout = match &self.kind {
            PatternKind::Rectangular { d1, d2 } => Layout::Rectangular {
                d1: linear_dir(ctx, d1, "direction1")?,
                d2: d2
                    .as_ref()
                    .map(|d| linear_dir(ctx, d, "direction2"))
                    .transpose()?,
            },
            PatternKind::Circular {
                axis,
                count,
                span,
                symmetric,
            } => {
                let axis = line_input(ctx, axis, "axis")?;
                let count = count_of(ctx, count, "count")?;
                let (step, symmetric) = match span {
                    AngleSpan::Full => (std::f64::consts::TAU / f64::from(count), false),
                    AngleSpan::Spacing(e) => (ctx.eval_angle(e)?, *symmetric),
                    AngleSpan::Extent(e) => {
                        let a = ctx.eval_angle(e)?;
                        let step = if count > 1 {
                            a / f64::from(count - 1)
                        } else {
                            0.0
                        };
                        (step, *symmetric)
                    }
                };
                if !step.is_finite() {
                    return Err(invalid("angle", "the angle is not finite"));
                }
                Layout::Circular {
                    axis,
                    step,
                    count,
                    symmetric,
                }
            }
            PatternKind::Path { .. } => return Err(not_yet("pattern along a path")),
        };
        if layout.len() > MAX_INSTANCES {
            return Err(invalid(
                "count",
                format!("at most {MAX_INSTANCES} instances"),
            ));
        }
        Ok(layout)
    }

    /// Every produced instance `(k, transform)`: the layout's, minus the suppressed ones.
    ///
    /// # Errors
    /// Resolution and evaluation errors of the directions, counts and spacings.
    pub fn instances(&self, ctx: &RecomputeCtx<'_>) -> Result<Vec<(u32, Transform)>, FeatureError> {
        let mut v = self.layout(ctx)?.instances();
        v.retain(|(k, _)| !self.suppressed.contains(k));
        Ok(v)
    }

    fn body_pattern(
        &self,
        ctx: &RecomputeCtx<'_>,
        bodies: &[BodyId],
        out: &mut FeatureOutput,
    ) -> Result<(), FeatureError> {
        let inst = self.instances(ctx)?;
        let me = ctx.feature_id();
        let single = bodies.len() == 1;
        let mut seq = 0u32;
        for &b in bodies {
            let (shape, naming) = input_solid(ctx, b)?;
            for (k, t) in &inst {
                ctx.check_cancelled()?;
                let s = shape.transformed(t).map_err(kerr)?;
                let n = naming
                    .instanced(me, *k, &s)
                    .map_err(|e| FeatureError::Kernel(format!("naming: {e}")))?;
                // One source body: the body slot is the instance number, so references to a
                // copy follow its instance when the count changes. Several: in order.
                let slot = if single { *k } else { seq };
                seq += 1;
                out.bodies.push(BodyChange {
                    id: ctx.new_body_id(slot),
                    change: BodyChangeKind::Created {
                        geom: BodyGeom::Solid(amg_doc::stub::Shape::from_brep(s)),
                        naming: Some(n),
                    },
                });
            }
        }
        if inst.is_empty() {
            out.warnings.push(super::common::warning(
                "The pattern has no instances besides the original.".into(),
            ));
        }
        Ok(())
    }

    fn feature_pattern(
        &self,
        ctx: &RecomputeCtx<'_>,
        seeds: &[FeatureId],
        out: &mut FeatureOutput,
    ) -> Result<(), FeatureError> {
        if self.compute == ComputeType::Adjust {
            return Err(not_yet("adjust compute"));
        }
        let inst = self.instances(ctx)?;
        let tools = seed_tools(ctx, seeds)?;
        let target = common_target(self.target.as_ref(), &tools)?;
        if inst.is_empty() {
            out.warnings.push(super::common::warning(
                "The pattern has no instances besides the original.".into(),
            ));
            return Ok(());
        }
        let k = op_kernel(ctx);
        let me = ctx.feature_id();
        let mut parts: Vec<(Shape, BodyNaming)> = Vec::with_capacity(inst.len() * tools.len());
        for (kk, t) in &inst {
            ctx.check_cancelled()?;
            for tool in &tools {
                let s = tool.shape.transformed(t).map_err(kerr)?;
                let n = tool
                    .naming
                    .instanced(me, *kk, &s)
                    .map_err(|e| FeatureError::Kernel(format!("naming: {e}")))?;
                parts.push((s, n));
            }
        }
        let (shape, naming) = instances_compound(ctx, k, parts)?;
        apply_body_op(
            ctx,
            ToolBody {
                shape,
                naming,
                target,
            },
            out,
        )
    }
}

/// The instance solids as one compound with their names (no copy: the compound shares the
/// instances' TShapes).
pub(crate) fn instances_compound(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    parts: Vec<(Shape, BodyNaming)>,
) -> Result<(Shape, BodyNaming), FeatureError> {
    // Naming warnings of the compound are not feature warnings (its history is complete by
    // construction); a partial history shows up again in the body operation.
    let mut scratch = FeatureOutput::default();
    compound_of(ctx, k, parts, &mut scratch)
}

/// The tool of every seed, built against the state just before the computing feature, named
/// by the seed's own rules (naming feature = the seed). A failure's highlights are copied to
/// `ctx`.
pub(crate) fn seed_tools(
    ctx: &RecomputeCtx<'_>,
    seeds: &[FeatureId],
) -> Result<Vec<ToolBody>, FeatureError> {
    if seeds.is_empty() {
        return Err(invalid("features", "select at least one feature"));
    }
    let design = ctx.design();
    let me = design.timeline().node(ctx.feature_id()).map(|(i, _)| i);
    let mut v = Vec::with_capacity(seeds.len());
    for &id in seeds {
        let Some((at, node)) = design.timeline().node(id) else {
            return Err(FeatureError::MissingInput(Box::new(Dependency::Feature(
                id,
            ))));
        };
        if me.is_some_and(|m| at >= m) {
            return Err(invalid(
                "features",
                format!("feature {id:?} comes after this one in the timeline"),
            ));
        }
        if node.suppressed {
            return Err(invalid("features", format!("feature {id:?} is suppressed")));
        }
        let sub = RecomputeCtx::new(design, ctx.before(), id, node.context, ctx.mode())
            .with_kernel(ctx.kernel());
        let sub = match ctx.job() {
            Some(j) => sub.with_job(j),
            None => sub,
        };
        let res = tool_of(node.feature.get(), &sub);
        ctx.highlight(sub.highlights());
        v.push(res?);
    }
    Ok(v)
}

/// The body operation of all instances: the override, else the seeds' (which must agree).
pub(crate) fn common_target(
    over: Option<&OpTarget>,
    tools: &[ToolBody],
) -> Result<OpTarget, FeatureError> {
    if let Some(t) = over {
        return Ok(t.clone());
    }
    let first = &tools[0].target;
    if tools.iter().any(|t| t.target != *first) {
        return Err(invalid(
            "target",
            "the features have different body operations; set the operation of the pattern",
        ));
    }
    Ok(first.clone())
}

impl Feature for Pattern {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d = Vec::new();
        match &self.objects {
            PatternObjects::Bodies(b) => d.extend(b.iter().map(|b| Dependency::Body(*b))),
            PatternObjects::Features(f) => d.extend(f.iter().map(|f| Dependency::Feature(*f))),
            PatternObjects::Faces(t) => {
                d.extend(
                    t.iter()
                        .cloned()
                        .map(|t| Dependency::Entity(EntityRef::Topo(t))),
                );
            }
            PatternObjects::Occurrences(o) => {
                d.extend(o.iter().map(|o| Dependency::Occurrence(*o)))
            }
        }
        match &self.kind {
            PatternKind::Rectangular { d1, d2 } => {
                d.extend(
                    std::iter::once(d1)
                        .chain(d2)
                        .filter_map(|x| ref_dependency(&x.dir)),
                );
            }
            PatternKind::Circular { axis, .. } => d.extend(ref_dependency(axis)),
            PatternKind::Path { path, .. } => {
                d.extend(path.iter().cloned().map(Dependency::Entity));
            }
        }
        if let Some(t) = &self.target {
            d.extend(t.inputs());
        }
        d
    }

    fn exprs(&self) -> Vec<&Expr> {
        // The fields of `exprs_mut`, in the same order.
        let mut v: Vec<&Expr> = Vec::new();
        match &self.kind {
            PatternKind::Rectangular { d1, d2 } => {
                for d in std::iter::once(d1).chain(d2) {
                    v.push(&d.count);
                    match &d.spacing {
                        Spacing::Extent(e) | Spacing::Spacing(e) => v.push(e),
                    }
                }
            }
            PatternKind::Circular { count, span, .. } => {
                v.push(count);
                match span {
                    AngleSpan::Full => {}
                    AngleSpan::Extent(e) | AngleSpan::Spacing(e) => v.push(e),
                }
            }
            PatternKind::Path {
                count,
                spacing,
                start,
                ..
            } => {
                v.push(count);
                match spacing {
                    Spacing::Extent(e) | Spacing::Spacing(e) => v.push(e),
                }
                v.push(start);
            }
        }
        v
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        let mut v: Vec<(&mut Expr, Dim)> = Vec::new();
        match &mut self.kind {
            PatternKind::Rectangular { d1, d2 } => {
                for d in std::iter::once(d1).chain(d2.as_mut()) {
                    v.push((&mut d.count, Dim::NONE));
                    match &mut d.spacing {
                        Spacing::Extent(e) | Spacing::Spacing(e) => v.push((e, Dim::LENGTH)),
                    }
                }
            }
            PatternKind::Circular { count, span, .. } => {
                v.push((count, Dim::NONE));
                match span {
                    AngleSpan::Full => {}
                    AngleSpan::Extent(e) | AngleSpan::Spacing(e) => v.push((e, Dim::ANGLE)),
                }
            }
            PatternKind::Path {
                count,
                spacing,
                start,
                ..
            } => {
                v.push((count, Dim::NONE));
                match spacing {
                    Spacing::Extent(e) | Spacing::Spacing(e) => v.push((e, Dim::LENGTH)),
                }
                v.push((start, Dim::NONE));
            }
        }
        v
    }

    fn validate(&self) -> Result<(), FeatureError> {
        // Empty selections build (the dialog's `min: 1` keeps OK disabled) and fail in
        // `compute`, as for fillet edges.
        match &self.objects {
            PatternObjects::Bodies(_) if self.target.is_some() => Err(invalid(
                "target",
                "a body pattern creates new bodies; it has no body operation",
            )),
            _ => Ok(()),
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        require_objects(&self.objects)?;
        match &self.objects {
            PatternObjects::Bodies(b) => self.body_pattern(ctx, b, &mut out)?,
            PatternObjects::Features(f) => self.feature_pattern(ctx, f, &mut out)?,
            PatternObjects::Faces(_) => return Err(not_yet("face pattern")),
            PatternObjects::Occurrences(_) => return Err(not_yet("component pattern")),
        }
        Ok(out)
    }
}

register_feature!(Pattern, "solid.pattern", version = 1);

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::{Dir3, Point3};

    fn lin(dir: Vec3, step: f64, count: u32, symmetric: bool) -> LinearDir {
        LinearDir {
            dir,
            step,
            count,
            symmetric,
        }
    }

    #[test]
    fn rectangular_numbers_follow_i_plus_j_shift_16() {
        let l = Layout::Rectangular {
            d1: lin(Vec3::X, 10.0, 3, false),
            d2: Some(lin(Vec3::Y, 5.0, 4, false)),
        };
        let v = l.instances();
        assert_eq!(l.len(), 12);
        assert_eq!(v.len(), 11, "the seed is not produced");
        let (k, t) = v.iter().find(|(k, _)| *k == 2 + (3 << 16)).unwrap();
        assert_eq!(*k, 0x0003_0002);
        let p = t.transform_point3(Point3::ZERO);
        assert!((p - Vec3::new(20.0, 15.0, 0.0)).length() < 1e-12);
    }

    #[test]
    fn symmetric_positions_set_the_top_bit_and_stay_stable() {
        let a = Layout::Rectangular {
            d1: lin(Vec3::X, 10.0, 2, true),
            d2: None,
        };
        let b = Layout::Rectangular {
            d1: lin(Vec3::X, 10.0, 3, true),
            d2: None,
        };
        let ka: Vec<u32> = a.instances().iter().map(|x| x.0).collect();
        let kb: Vec<u32> = b.instances().iter().map(|x| x.0).collect();
        assert_eq!(ka, vec![1, 0x8001]);
        assert_eq!(kb, vec![1, 2, 0x8001, 0x8002]);
        let neg = b.instances().into_iter().find(|x| x.0 == 0x8002).unwrap().1;
        assert!((neg.transform_point3(Point3::ZERO).x + 20.0).abs() < 1e-12);
        assert_eq!(b.len(), 5);
    }

    #[test]
    fn circular_full_turn_spreads_evenly() {
        let l = Layout::Circular {
            axis: Line3 {
                point: Point3::ZERO,
                dir: Dir3::Z,
            },
            step: std::f64::consts::TAU / 6.0,
            count: 6,
            symmetric: false,
        };
        let v = l.instances();
        assert_eq!(
            v.iter().map(|x| x.0).collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5]
        );
        let p = v[2].1.transform_point3(Point3::new(30.0, 0.0, 10.0));
        assert!((p - Point3::new(-30.0, 0.0, 10.0)).length() < 1e-9, "{p}");
        let sym = Layout::Circular {
            axis: Line3 {
                point: Point3::new(1.0, 0.0, 0.0),
                dir: Dir3::Z,
            },
            step: 0.5,
            count: 2,
            symmetric: true,
        };
        assert_eq!(
            sym.instances().iter().map(|x| x.0).collect::<Vec<_>>(),
            vec![1, 0x8000_0001]
        );
        let p = sym.instances()[0]
            .1
            .transform_point3(Point3::new(1.0, 0.0, 0.0));
        assert!(
            (p - Point3::new(1.0, 0.0, 0.0)).length() < 1e-12,
            "on the axis"
        );
    }

    #[test]
    fn instance_numbers_are_unique() {
        let l = Layout::Rectangular {
            d1: lin(Vec3::X, 1.0, 5, true),
            d2: Some(lin(Vec3::Y, 1.0, 4, true)),
        };
        let mut ks: Vec<u32> = l.instances().iter().map(|x| x.0).collect();
        let n = ks.len();
        ks.sort_unstable();
        ks.dedup();
        assert_eq!(ks.len(), n);
        assert_eq!(n + 1, l.len());
    }
}
