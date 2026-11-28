//! Primitives (S04 SOL-04): `solid.box`, `solid.cylinder`, `solid.sphere` and `solid.torus`,
//! each a kernel primitive (BREP-14) placed on a [`PrimitiveFrame`] and applied with an
//! [`OpTarget`]. All are [`SolidTool`]s, so pattern and mirror can use them as seeds.
//!
//! **Placement.** The frame's plane is an origin plane, a construction plane or a planar face,
//! resolved by CON-03 (`resolve_plane_input`; a face gives its outward normal). `u` and `v` are
//! offsets along that plane's x and y axes from its origin, and `angle` turns the frame about
//! the plane normal (counter-clockwise).
//!
//! - Box: `size` is length (frame x), width (frame y) and height (frame normal). One corner
//!   sits at the frame origin, or the footprint is centred on it with `centered`. Length and
//!   width are positive; a negative height goes below the plane.
//! - Cylinder: base centre at the frame origin, axis along the normal; a negative height goes
//!   below the plane.
//! - Sphere and torus: centred at the frame origin; the torus axis is the normal and its major
//!   radius must exceed the minor one.
//!
//! Naming (naming note §5.2 "Primitives"): fixed role tags with no source. Box faces are
//! `XMin#0 … ZMax#0` by their outward normal in the box frame; cylinder faces `Lateral#0`,
//! `StartCap#0` (on the placement plane) and `EndCap#0`; sphere and torus `Surface#0`. Size
//! and placement edits therefore keep every name.

mod dialog;

use std::f64::consts::TAU;

use amg_brep::{BrepKernel, KResult, OpResult, OpRole, SubKind, SubShape};
use amg_core::{Dir3, Plane, Vec3};
use amg_doc::naming::Role;
use amg_doc::{
    Dependency, Dim, Expr, ExprSrc, Feature, FeatureError, FeatureOutput, RecomputeCtx, TopoKind,
    register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{OpTarget, SolidTool, ToolBody, apply_body_op, finish_naming, op_kernel};
use super::error::{SolidError, kernel_failure, kerr};
use crate::construct::resolve::resolve_plane_input;
use crate::construct::{OriginEntity, RefInput};

/// Where a primitive sits: a plane (origin plane, construction plane or planar face), an
/// in-plane origin and a rotation about the plane normal.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PrimitiveFrame {
    /// The placement plane.
    pub plane: RefInput,
    /// Offset of the frame origin along the plane's x axis, mm.
    pub u: Expr,
    /// Offset of the frame origin along the plane's y axis, mm.
    pub v: Expr,
    /// Rotation of the frame about the plane normal, rad.
    pub angle: Expr,
}

fn inline(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

/// The frame `base` moved by `(u, v)` in its own axes and turned by `angle` about its normal.
fn placed(base: &Plane, u: f64, v: f64, angle: f64) -> Plane {
    let (x, y) = (base.x.get(), base.y.get());
    let (s, c) = angle.sin_cos();
    Plane {
        origin: base.origin + x * u + y * v,
        x: Dir3::new_unchecked((x * c + y * s).normalize()),
        y: Dir3::new_unchecked((y * c - x * s).normalize()),
        normal: base.normal,
    }
}

impl PrimitiveFrame {
    /// A frame on `plane` with no offset or rotation.
    pub fn on(plane: RefInput) -> PrimitiveFrame {
        PrimitiveFrame {
            plane,
            u: inline("0 mm"),
            v: inline("0 mm"),
            angle: inline("0 deg"),
        }
    }

    /// The world XY plane at the origin.
    pub fn xy() -> PrimitiveFrame {
        PrimitiveFrame::on(RefInput::Origin(OriginEntity::XYPlane))
    }

    /// The placed frame, in the context component's space.
    ///
    /// # Errors
    /// Expression errors; a plane reference that doesn't resolve to a plane.
    pub fn resolve(&self, ctx: &RecomputeCtx<'_>) -> Result<Plane, FeatureError> {
        let base = resolve_plane_input(ctx, &self.plane)?;
        let u = finite("offset_x", ctx.eval_length(&self.u)?)?;
        let v = finite("offset_y", ctx.eval_length(&self.v)?)?;
        let angle = finite("rotation", ctx.eval_angle(&self.angle)?)?;
        Ok(placed(&base, u, v, angle))
    }

    fn exprs(&self) -> [&Expr; 3] {
        [&self.u, &self.v, &self.angle]
    }

    fn exprs_mut(&mut self) -> [(&mut Expr, Dim); 3] {
        [
            (&mut self.u, Dim::LENGTH),
            (&mut self.v, Dim::LENGTH),
            (&mut self.angle, Dim::ANGLE),
        ]
    }
}

/// `solid.box`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoxPrim {
    /// Placement.
    pub frame: PrimitiveFrame,
    /// Length (frame x), width (frame y) and height (frame normal).
    pub size: [Expr; 3],
    /// Centre the footprint on the frame origin (else one corner sits there).
    #[serde(default)]
    pub centered: bool,
    /// Body operation.
    #[serde(default)]
    pub target: OpTarget,
}

/// `solid.cylinder`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CylinderPrim {
    /// Placement of the base centre.
    pub frame: PrimitiveFrame,
    /// Radius.
    pub radius: Expr,
    /// Height along the frame normal (negative: below the plane).
    pub height: Expr,
    /// Body operation.
    #[serde(default)]
    pub target: OpTarget,
}

/// `solid.sphere`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SpherePrim {
    /// Placement of the centre.
    pub frame: PrimitiveFrame,
    /// Radius.
    pub radius: Expr,
    /// Body operation.
    #[serde(default)]
    pub target: OpTarget,
}

/// `solid.torus`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TorusPrim {
    /// Placement of the centre; the axis is the frame normal.
    pub frame: PrimitiveFrame,
    /// Radius of the tube's centre circle.
    pub major: Expr,
    /// Radius of the tube.
    pub minor: Expr,
    /// Body operation.
    #[serde(default)]
    pub target: OpTarget,
}

impl BoxPrim {
    /// A box of `size` (expressions such as `"10 mm"`) with a corner at the frame origin.
    pub fn new(frame: PrimitiveFrame, size: [&str; 3], target: OpTarget) -> BoxPrim {
        BoxPrim {
            frame,
            size: size.map(inline),
            centered: false,
            target,
        }
    }
}

impl CylinderPrim {
    /// A cylinder of `radius` and `height` (expressions).
    pub fn new(
        frame: PrimitiveFrame,
        radius: &str,
        height: &str,
        target: OpTarget,
    ) -> CylinderPrim {
        CylinderPrim {
            frame,
            radius: inline(radius),
            height: inline(height),
            target,
        }
    }
}

impl SpherePrim {
    /// A sphere of `radius` (an expression).
    pub fn new(frame: PrimitiveFrame, radius: &str, target: OpTarget) -> SpherePrim {
        SpherePrim {
            frame,
            radius: inline(radius),
            target,
        }
    }
}

impl TorusPrim {
    /// A torus of `major` and `minor` radius (expressions).
    pub fn new(frame: PrimitiveFrame, major: &str, minor: &str, target: OpTarget) -> TorusPrim {
        TorusPrim {
            frame,
            major: inline(major),
            minor: inline(minor),
            target,
        }
    }
}

// ---- evaluation -----------------------------------------------------------------------------

fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

fn finite(field: &'static str, v: f64) -> Result<f64, FeatureError> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(invalid(field, "must be a finite number"))
    }
}

/// A length larger than the linear tolerance.
fn positive(ctx: &RecomputeCtx<'_>, field: &'static str, e: &Expr) -> Result<f64, FeatureError> {
    let v = finite(field, ctx.eval_length(e)?)?;
    if v <= ctx.tolerance().linear {
        return Err(invalid(field, "must be greater than zero"));
    }
    Ok(v)
}

/// A signed length that isn't zero.
fn nonzero(ctx: &RecomputeCtx<'_>, field: &'static str, e: &Expr) -> Result<f64, FeatureError> {
    let v = finite(field, ctx.eval_length(e)?)?;
    if v.abs() <= ctx.tolerance().linear {
        return Err(invalid(field, "must not be zero"));
    }
    Ok(v)
}

/// The fixed role tag of a box face whose outward normal is `n`, in the box frame.
fn box_tag(frame: &Plane, n: Vec3) -> &'static str {
    let local = [
        n.dot(frame.x.get()),
        n.dot(frame.y.get()),
        n.dot(frame.normal.get()),
    ];
    let mut axis = 0;
    for i in 1..3 {
        if local[i].abs() > local[axis].abs() {
            axis = i;
        }
    }
    match (axis, local[axis] > 0.0) {
        (0, false) => "XMin",
        (0, true) => "XMax",
        (1, false) => "YMin",
        (1, true) => "YMax",
        (_, false) => "ZMin",
        (_, true) => "ZMax",
    }
}

/// Builds a primitive with `make` and names every face with the tag `tag_of` gives it.
fn tool_body(
    ctx: &RecomputeCtx<'_>,
    target: &OpTarget,
    make: impl FnOnce(&dyn BrepKernel) -> KResult<OpResult>,
    tag_of: impl Fn(&dyn BrepKernel, &OpResult, u32) -> KResult<&'static str>,
) -> Result<ToolBody, FeatureError> {
    ctx.check_cancelled()?;
    let k = op_kernel(ctx);
    // A primitive has no input, so its naming never warns.
    let mut scratch = FeatureOutput::default();
    let r = make(k).map_err(|e| kernel_failure(ctx, None, e, &mut scratch))?;
    let mut b = ctx.naming_builder();
    b.from_history(&[], &r.history);
    for f in 0..r.shape.count(SubKind::Face) {
        let tag = tag_of(k, &r, f).map_err(kerr)?;
        b.generated(TopoKind::Face, f, Role::new(tag, 0), None);
    }
    let naming = finish_naming(b, k, &r.shape, &mut scratch)?;
    Ok(ToolBody {
        shape: r.shape,
        naming,
        target: target.clone(),
    })
}

impl SolidTool for BoxPrim {
    fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError> {
        let frame = self.frame.resolve(ctx)?;
        let length = positive(ctx, "length", &self.size[0])?;
        let width = positive(ctx, "width", &self.size[1])?;
        let height = nonzero(ctx, "height", &self.size[2])?;
        let mut corner = frame;
        if self.centered {
            corner.origin -= frame.x.get() * (length / 2.0) + frame.y.get() * (width / 2.0);
        }
        corner.origin += frame.normal.get() * height.min(0.0);
        let size = Vec3::new(length, width, height.abs());
        tool_body(
            ctx,
            &self.target,
            |k| k.make_box(&corner, size),
            |k, r, f| {
                let sub = SubShape {
                    kind: SubKind::Face,
                    index: f,
                };
                let n = k.sub_geom(&r.shape, sub)?.axis.unwrap_or(Vec3::ZERO);
                Ok(box_tag(&corner, n))
            },
        )
    }
}

impl SolidTool for CylinderPrim {
    fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError> {
        let mut frame = self.frame.resolve(ctx)?;
        let radius = positive(ctx, "radius", &self.radius)?;
        let height = nonzero(ctx, "height", &self.height)?;
        if height < 0.0 {
            // The frame turned over about x: the start cap stays on the placement plane.
            frame.y = -frame.y;
            frame.normal = -frame.normal;
        }
        tool_body(
            ctx,
            &self.target,
            |k| k.make_cylinder(&frame, radius, height.abs(), TAU),
            |_, r, f| {
                let sub = SubShape {
                    kind: SubKind::Face,
                    index: f,
                };
                Ok(match r.history.role_of(sub) {
                    Some(OpRole::StartCap) => "StartCap",
                    Some(OpRole::EndCap) => "EndCap",
                    _ => "Lateral",
                })
            },
        )
    }
}

impl SolidTool for SpherePrim {
    fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError> {
        let frame = self.frame.resolve(ctx)?;
        let radius = positive(ctx, "radius", &self.radius)?;
        tool_body(
            ctx,
            &self.target,
            |k| k.make_sphere(&frame, radius),
            |_, _, _| Ok("Surface"),
        )
    }
}

impl SolidTool for TorusPrim {
    fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError> {
        let frame = self.frame.resolve(ctx)?;
        let major = positive(ctx, "major_radius", &self.major)?;
        let minor = positive(ctx, "minor_radius", &self.minor)?;
        if major - minor <= ctx.tolerance().linear {
            return Err(invalid(
                "minor_radius",
                "the minor radius must be smaller than the major radius",
            ));
        }
        tool_body(
            ctx,
            &self.target,
            |k| k.make_torus(&frame, major, minor, TAU),
            |_, _, _| Ok("Surface"),
        )
    }
}

// ---- features -------------------------------------------------------------------------------

/// The size expressions of each kind (all lengths), in dialog order.
trait Sizes {
    fn frame(&self) -> &PrimitiveFrame;
    fn target(&self) -> &OpTarget;
    fn sizes(&self) -> Vec<&Expr>;
    /// The frame expressions and then the sizes, mutably.
    fn all_mut(&mut self) -> Vec<(&mut Expr, Dim)>;
}

macro_rules! sizes {
    ($ty:ident, |$s:ident| $get:expr, |$m:ident| $get_mut:expr) => {
        impl Sizes for $ty {
            fn frame(&self) -> &PrimitiveFrame {
                &self.frame
            }

            fn target(&self) -> &OpTarget {
                &self.target
            }

            fn sizes(&self) -> Vec<&Expr> {
                let $s = self;
                $get
            }

            fn all_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
                let $m = self;
                let mut v: Vec<(&mut Expr, Dim)> = $m.frame.exprs_mut().into();
                v.extend($get_mut.into_iter().map(|e| (e, Dim::LENGTH)));
                v
            }
        }
    };
}

sizes!(BoxPrim, |s| s.size.iter().collect(), |s| s.size.iter_mut());
sizes!(CylinderPrim, |s| vec![&s.radius, &s.height], |s| [
    &mut s.radius,
    &mut s.height
]);
sizes!(SpherePrim, |s| vec![&s.radius], |s| [&mut s.radius]);
sizes!(TorusPrim, |s| vec![&s.major, &s.minor], |s| [
    &mut s.major,
    &mut s.minor
]);

/// `Feature` and tool registration of a primitive.
macro_rules! primitive_feature {
    ($ty:ident, $kind:literal) => {
        impl Feature for $ty {
            fn inputs(&self) -> Vec<Dependency> {
                let mut d: Vec<Dependency> = Vec::new();
                d.extend(super::pattern::ref_dependency(&self.frame().plane));
                d.extend(self.target().inputs());
                d
            }

            fn exprs(&self) -> Vec<&Expr> {
                // The fields of `exprs_mut`, in the same order.
                let mut v: Vec<&Expr> = self.frame().exprs().to_vec();
                v.extend(self.sizes());
                v
            }

            fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
                self.all_mut()
            }

            fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
                let tool = self.tool(ctx)?;
                let mut out = FeatureOutput::default();
                apply_body_op(ctx, tool, &mut out)?;
                Ok(out)
            }
        }

        register_feature!($ty, $kind, version = 1);
        crate::register_tool!($ty, $kind);
    };
}

primitive_feature!(BoxPrim, "solid.box");
primitive_feature!(CylinderPrim, "solid.cylinder");
primitive_feature!(SpherePrim, "solid.sphere");
primitive_feature!(TorusPrim, "solid.torus");
