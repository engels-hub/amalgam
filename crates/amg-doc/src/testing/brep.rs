//! Lane-E B-rep test features for the persistent-naming framework (DOC-06; schemas of the
//! naming note §10, used by the TNP corpus §11). Built directly on `BrepKernel`; they need the
//! OCCT backend (`amg-doc` feature `occt`) and fail with a kernel error without it.
//!
//! - `test.brep_box` `{ size }`: a box at the origin; faces `XMin#0 … ZMax#0`.
//! - `test.brep_prism` `{ points: [[id, x, y], …], z0, height, op, target }`: a polygon prism;
//!   edge `i` (point i → i+1) is tagged with point `i`'s id, laterals are
//!   `Gen { self, Lateral#0, SketchCurve { SketchId(self), id } }`, caps `StartCap#0` /
//!   `EndCap#0`. `Cut`/`Join` modify `target` and simplify the result with UnifySameDomain
//!   (note §5.1), its history composed into the boolean's.
//! - `test.brep_fillet` `{ body, edges, radius, policy }`: the S04 fillet rule (selection keys
//!   for picked edges, corners by the role-only rule).
//! - `test.probe` `{ body, refs, policy }`: resolves and records its refs, changes nothing.

use amg_brep::geom::{Curve2d, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, FilletOpts, FilletSpec, OpResult, PrismExtent, PrismOpts,
    SubKind, SubShape,
};
use amg_core::{Plane, Point2, Point3, Vec3};
use amg_jobs::JobCtx;
use serde::{Deserialize, Serialize};

use crate::eval::BodyGeom;
use crate::feature::{
    BodyChange, BodyChangeKind, Dependency, Feature, FeatureError, FeatureOutput, RecomputeCtx,
};
use crate::id::{BodyId, SketchId};
use crate::naming::{
    BodyNaming, NameSource, NamingBuilder, NamingError, OperandNaming, Role, SetPolicy, TopoKind,
    TopoRef,
};
use crate::stub::{Shape, SketchEntityKey};

/// Kinds registered by this module.
pub const KINDS: [&str; 4] = [
    "test.brep_box",
    "test.brep_fillet",
    "test.brep_prism",
    "test.probe",
];

fn kerr(e: impl std::fmt::Display) -> FeatureError {
    FeatureError::Kernel(e.to_string())
}

fn naming_err(e: &NamingError) -> FeatureError {
    FeatureError::Kernel(format!("naming: {e}"))
}

fn with_job<T>(ctx: &RecomputeCtx<'_>, f: impl FnOnce(&JobCtx) -> T) -> T {
    match ctx.job() {
        Some(j) => f(j),
        None => f(&JobCtx::detached()),
    }
}

/// The real B-rep of a solid input body and its element map.
fn solid_input<'a>(
    ctx: &RecomputeCtx<'a>,
    id: BodyId,
) -> Result<(&'a amg_brep::Shape, &'a BodyNaming), FeatureError> {
    let BodyGeom::Solid(s) = ctx.body(id)? else {
        return Err(FeatureError::InvalidValue {
            field: "body",
            msg: format!("{id:?} is not a solid"),
        });
    };
    let shape = s.brep().ok_or_else(|| FeatureError::InvalidValue {
        field: "body",
        msg: format!("{id:?} has no B-rep geometry"),
    })?;
    let naming = ctx.naming(id).ok_or_else(|| FeatureError::InvalidValue {
        field: "body",
        msg: format!("{id:?} has no element map"),
    })?;
    Ok((shape, naming))
}

fn finish(
    b: NamingBuilder,
    kernel: &dyn BrepKernel,
    shape: &amg_brep::Shape,
    out: &mut FeatureOutput,
) -> Result<BodyNaming, FeatureError> {
    let (naming, report) = b.finish_report(kernel, shape).map_err(|e| naming_err(&e))?;
    if report.partial_history() {
        out.warnings.push(crate::eval::Diagnostic {
            message: format!(
                "Kernel history is incomplete for {} face(s) (PartialHistory); references to them are weak.",
                report.unexplained.len()
            ),
            cause: None,
            entities: Vec::new(),
        });
    }
    Ok(naming)
}

// ---- box ------------------------------------------------------------------------------------

/// `test.brep_box`: a box `size` (mm) with one corner at the origin.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TestBrepBox {
    /// Edge lengths along x, y, z, mm.
    pub size: [f64; 3],
}

/// The fixed role of a box face from its outward normal.
fn box_role(axis: Option<Vec3>) -> Role {
    let a = axis.unwrap_or(Vec3::ZERO);
    let (i, v) = a
        .to_array()
        .into_iter()
        .enumerate()
        .max_by(|x, y| x.1.abs().total_cmp(&y.1.abs()))
        .unwrap_or((0, 0.0));
    let tag = match (i, v > 0.0) {
        (0, false) => "XMin",
        (0, true) => "XMax",
        (1, false) => "YMin",
        (1, true) => "YMax",
        (_, false) => "ZMin",
        (_, true) => "ZMax",
    };
    Role::new(tag, 0)
}

impl Feature for TestBrepBox {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let k = ctx.kernel();
        let r = k
            .make_box(&Plane::XY, Vec3::from_array(self.size))
            .map_err(kerr)?;
        let mut b = ctx.naming_builder();
        b.from_history(&[], &r.history);
        for f in 0..r.shape.count(SubKind::Face) {
            let g = k
                .sub_geom(
                    &r.shape,
                    SubShape {
                        kind: SubKind::Face,
                        index: f,
                    },
                )
                .map_err(kerr)?;
            b.generated(TopoKind::Face, f, box_role(g.axis), None);
        }
        let mut out = FeatureOutput::default();
        let naming = finish(b, k, &r.shape, &mut out)?;
        out.bodies.push(BodyChange {
            id: ctx.new_body_id(0),
            change: BodyChangeKind::Created {
                geom: BodyGeom::Solid(Shape::from_brep(r.shape)),
                naming: Some(naming),
            },
        });
        Ok(out)
    }
}

crate::register_feature!(TestBrepBox, "test.brep_box", version = 1);

// ---- prism ----------------------------------------------------------------------------------

/// Boolean mode of `test.brep_prism`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PrismOp {
    /// A new body.
    New,
    /// Cut from `target`.
    Cut,
    /// Fuse into `target`.
    Join,
}

/// `test.brep_prism`: a CCW polygon `points` (`[id, x, y]`) in the plane `z = z0`, swept
/// `height` along +Z.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TestBrepPrism {
    /// `[id, x, y]` per vertex, counter-clockwise; ids are stable sketch-like keys.
    pub points: Vec<[f64; 3]>,
    /// Plane height, mm.
    pub z0: f64,
    /// Sweep along +Z, mm.
    pub height: f64,
    /// Boolean mode.
    pub op: PrismOp,
    /// Target of `Cut`/`Join`.
    pub target: Option<BodyId>,
}

#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "ids are small non-negative integers written as JSON numbers"
)]
fn point_id(p: &[f64; 3]) -> u64 {
    p[0] as u64
}

impl TestBrepPrism {
    fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<(OpResult, BodyNaming), FeatureError> {
        let k = ctx.kernel();
        let n = self.points.len();
        if n < 3 {
            return Err(FeatureError::InvalidValue {
                field: "points",
                msg: "need at least 3 points".into(),
            });
        }
        let edges = (0..n)
            .map(|i| {
                let (a, b) = (&self.points[i], &self.points[(i + 1) % n]);
                ProfileEdge2 {
                    curve: Curve2d::Line {
                        a: Point2::new(a[1], a[2]),
                        b: Point2::new(b[1], b[2]),
                    },
                    tag: Tag(point_id(a)),
                }
            })
            .collect();
        let plane = Plane {
            origin: Point3::new(0.0, 0.0, self.z0),
            ..Plane::XY
        };
        let profile = PlanarProfile {
            plane,
            outer: Loop2 { edges },
            holes: Vec::new(),
        };
        let fb = k
            .make_planar_faces(&[profile], ctx.tolerance())
            .map_err(kerr)?
            .pop()
            .ok_or_else(|| kerr("no face built"))?;
        let r = with_job(ctx, |job| {
            k.prism(
                &fb.face,
                &PrismExtent::Vector(Vec3::new(0.0, 0.0, self.height)),
                &PrismOpts {
                    taper: 0.0,
                    merge_coplanar: false,
                },
                job,
            )
        })
        .map_err(kerr)?;
        let mut b = ctx.naming_builder();
        b.from_history(&[(0, OperandNaming::Tagged(&fb.edge_tags))], &r.history);
        let sketch = SketchId(ctx.feature_id());
        for p in &self.points {
            b.tag(
                Tag(point_id(p)),
                NameSource::SketchCurve {
                    sketch,
                    curve: SketchEntityKey(point_id(p)),
                },
            );
        }
        for (sub, role) in &r.history.roles {
            let cap = match role {
                amg_brep::OpRole::StartCap => Role::new("StartCap", 0),
                amg_brep::OpRole::EndCap => Role::new("EndCap", 0),
                _ => continue,
            };
            if sub.kind == SubKind::Face {
                b.generated(TopoKind::Face, sub.index, cap, None);
            }
        }
        let mut scratch = FeatureOutput::default();
        let naming = finish(b, k, &r.shape, &mut scratch)?;
        Ok((r, naming))
    }
}

impl Feature for TestBrepPrism {
    fn inputs(&self) -> Vec<Dependency> {
        self.target.map(Dependency::Body).into_iter().collect()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let k = ctx.kernel();
        let (tool, tool_naming) = self.tool(ctx)?;
        let mut out = FeatureOutput::default();
        let bool_op = match self.op {
            PrismOp::New => {
                out.bodies.push(BodyChange {
                    id: ctx.new_body_id(0),
                    change: BodyChangeKind::Created {
                        geom: BodyGeom::Solid(Shape::from_brep(tool.shape)),
                        naming: Some(tool_naming),
                    },
                });
                return Ok(out);
            }
            PrismOp::Cut => BoolOp::Cut,
            PrismOp::Join => BoolOp::Fuse,
        };
        let target = self.target.ok_or(FeatureError::InvalidValue {
            field: "target",
            msg: "Cut and Join need a target".into(),
        })?;
        let (shape, naming) = solid_input(ctx, target)?;
        let r1 = with_job(ctx, |job| {
            k.boolean(
                bool_op,
                std::slice::from_ref(shape),
                std::slice::from_ref(&tool.shape),
                &BoolOpts::default(),
                job,
            )
        })
        .map_err(kerr)?;
        // Every BodyOp result is simplified (note §5.1); the unify history is composed in.
        let r2 = crate::naming::default_kernel()
            .unify_coplanar(&r1.shape)
            .map_err(kerr)?;
        let mut history = r1.history.compose(&r2.history);
        history.order_splits(&r2.shape).map_err(kerr)?;
        let mut b = ctx.naming_builder();
        b.from_history(
            &[
                (0, OperandNaming::Named(naming)),
                (1, OperandNaming::Named(&tool_naming)),
            ],
            &history,
        );
        let naming = finish(b, k, &r2.shape, &mut out)?;
        out.bodies.push(BodyChange {
            id: target,
            change: BodyChangeKind::Modified {
                geom: BodyGeom::Solid(Shape::from_brep(r2.shape)),
                naming: Some(naming),
            },
        });
        Ok(out)
    }
}

crate::register_feature!(TestBrepPrism, "test.brep_prism", version = 1);

// ---- fillet ---------------------------------------------------------------------------------

/// `test.brep_fillet`: a constant-radius fillet of `edges` of `body`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TestBrepFillet {
    /// The body.
    pub body: BodyId,
    /// Picked edges (set-valued).
    pub edges: Vec<TopoRef>,
    /// Radius, mm.
    pub radius: f64,
    /// How lost members are treated.
    #[serde(default)]
    pub policy: SetPolicy,
}

impl Feature for TestBrepFillet {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let k = ctx.kernel();
        let members = ctx.resolve_set(&self.edges, self.policy)?;
        let (shape, naming) = solid_input(ctx, self.body)?;
        let mut edges: Vec<(amg_brep::EdgeId, FilletSpec)> = Vec::new();
        let mut keys: Vec<(u32, u32)> = Vec::new();
        for (pos, (r, member)) in self.edges.iter().zip(&members).enumerate() {
            let key = if r.key != 0 {
                r.key
            } else {
                u32::try_from(pos + 1).unwrap_or(u32::MAX)
            };
            for x in member {
                if x.body != self.body || x.kind != TopoKind::Edge {
                    return Err(FeatureError::InvalidValue {
                        field: "edges",
                        msg: format!("{x:?} is not an edge of {:?}", self.body),
                    });
                }
                if !keys.iter().any(|(e, _)| *e == x.index) {
                    keys.push((x.index, key));
                    edges.push((amg_brep::EdgeId(x.index), FilletSpec::Constant(self.radius)));
                }
            }
        }
        let r = with_job(ctx, |job| {
            k.fillet(shape, &edges, &FilletOpts::default(), job)
        })
        .map_err(kerr)?;
        let mut b = ctx.naming_builder();
        b.from_history(&[(0, OperandNaming::Named(naming))], &r.history);
        for (e, key) in keys {
            b.selection(
                0,
                SubShape {
                    kind: SubKind::Edge,
                    index: e,
                },
                key,
            );
        }
        let mut out = FeatureOutput::default();
        let naming = finish(b, k, &r.shape, &mut out)?;
        out.bodies.push(BodyChange {
            id: self.body,
            change: BodyChangeKind::Modified {
                geom: BodyGeom::Solid(Shape::from_brep(r.shape)),
                naming: Some(naming),
            },
        });
        Ok(out)
    }
}

crate::register_feature!(TestBrepFillet, "test.brep_fillet", version = 1);

// ---- probe ----------------------------------------------------------------------------------

/// `test.probe`: resolves `refs` against `body` and records them; outputs nothing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TestProbe {
    /// The body the refs live in.
    pub body: BodyId,
    /// The references.
    pub refs: Vec<TopoRef>,
    /// How failing members are treated.
    #[serde(default)]
    pub policy: SetPolicy,
}

impl Feature for TestProbe {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        ctx.resolve_set(&self.refs, self.policy)?;
        Ok(FeatureOutput::default())
    }
}

crate::register_feature!(TestProbe, "test.probe", version = 1);
