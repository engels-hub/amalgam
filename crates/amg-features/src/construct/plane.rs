//! The construction plane feature (`construct.plane` v1, CON-01).

use amg_core::{Dimension, Plane};
use amg_doc::{
    ConstructionGeom, Dependency, Diagnostic, Expr, Feature, FeatureError, FeatureOutput,
    RecomputeCtx, register_feature,
};
use serde::{Deserialize, Serialize};

use super::curves::curve_of;
use super::{
    ConstructInputs, CtxInputs, PathPosition, RefInput, ResolvedRef, dependency, math,
    resolve_tracked,
};

/// How a construction plane is defined (Fusion's Construct menu; conventions in note §6).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum PlaneDef {
    /// `base` moved along its normal by `distance` (length).
    Offset {
        /// A plane (planar face, construction or origin plane).
        base: RefInput,
        /// Offset, mm; may be negative.
        distance: Expr,
    },
    /// The plane containing `line`, rotated by `angle` from the zero plane.
    AtAngle {
        /// A straight edge, sketch line or axis.
        line: RefInput,
        /// Zero plane; `None` uses the line's context plane, else the note §6 default.
        zero: Option<RefInput>,
        /// Rotation about the line direction (right-hand rule), rad.
        angle: Expr,
    },
    /// Tangent to a cylinder, cone or sphere face, at `angle` from `reference`.
    Tangent {
        /// The face.
        face: RefInput,
        /// The reference plane (angle zero).
        reference: RefInput,
        /// Rotation about the face axis, rad.
        angle: Expr,
    },
    /// Midway between two planes or two points.
    Midplane {
        /// First plane or point.
        a: RefInput,
        /// Second plane or point.
        b: RefInput,
    },
    /// Through two coplanar straight edges.
    TwoEdges {
        /// First edge.
        a: RefInput,
        /// Second edge.
        b: RefInput,
    },
    /// Through three points.
    ThreePoints {
        /// Origin of the plane.
        a: RefInput,
        /// Sets the x axis.
        b: RefInput,
        /// Sets the normal side.
        c: RefInput,
    },
    /// Tangent to a face at the face point closest to `point`.
    TangentAtPoint {
        /// The face.
        face: RefInput,
        /// The point.
        point: RefInput,
    },
    /// Normal to a path at a position along it.
    AlongPath {
        /// An edge or sketch curve.
        path: RefInput,
        /// Fraction or distance along the path.
        at: PathPosition,
    },
}

impl PlaneDef {
    /// Every reference input, in field order.
    pub fn refs(&self) -> Vec<&RefInput> {
        match self {
            PlaneDef::Offset { base, .. } => vec![base],
            PlaneDef::AtAngle { line, zero, .. } => {
                let mut v = vec![line];
                v.extend(zero.as_ref());
                v
            }
            PlaneDef::Tangent {
                face, reference, ..
            } => vec![face, reference],
            PlaneDef::Midplane { a, b } | PlaneDef::TwoEdges { a, b } => vec![a, b],
            PlaneDef::ThreePoints { a, b, c } => vec![a, b, c],
            PlaneDef::TangentAtPoint { face, point } => vec![face, point],
            PlaneDef::AlongPath { path, .. } => vec![path],
        }
    }

    /// Every parameter expression with its dimension.
    fn exprs_with_dim(&self) -> Vec<(&Expr, Dimension)> {
        match self {
            PlaneDef::Offset { distance, .. } => vec![(distance, Dimension::LENGTH)],
            PlaneDef::AtAngle { angle, .. } | PlaneDef::Tangent { angle, .. } => {
                vec![(angle, Dimension::ANGLE)]
            }
            PlaneDef::AlongPath { at, .. } => vec![position_expr(at)],
            PlaneDef::Midplane { .. }
            | PlaneDef::TwoEdges { .. }
            | PlaneDef::ThreePoints { .. }
            | PlaneDef::TangentAtPoint { .. } => Vec::new(),
        }
    }

    /// Computes the plane from resolved inputs. Returns the plane and the health warnings
    /// (references matched by fingerprint only).
    pub fn evaluate(
        &self,
        inputs: &dyn ConstructInputs,
    ) -> Result<(Plane, Vec<Diagnostic>), FeatureError> {
        let tol = inputs.tolerance();
        let mut warnings = Vec::new();
        let mut res = |r: &RefInput| -> Result<ResolvedRef, FeatureError> {
            resolve_tracked(inputs, r, &mut warnings)
        };
        let plane = match self {
            PlaneDef::Offset { base, distance } => {
                let base = math::plane_of(&res(base)?.geom)?;
                math::plane_offset(&base, inputs.length(distance)?)
            }
            PlaneDef::AtAngle { line, zero, angle } => {
                let line = res(line)?;
                let zero = match zero {
                    Some(z) => Some(math::plane_of(&res(z)?.geom)?),
                    None => line.context_plane,
                };
                let l = math::line_of(&line.geom, &tol)?;
                math::plane_at_angle(&l, zero.as_ref(), inputs.angle(angle)?, &tol)?
            }
            PlaneDef::Tangent {
                face,
                reference,
                angle,
            } => {
                let face = res(face)?;
                let reference = math::plane_of(&res(reference)?.geom)?;
                math::plane_tangent(&face.geom, &reference, inputs.angle(angle)?, &tol)?
            }
            PlaneDef::Midplane { a, b } => {
                let (a, b) = (res(a)?, res(b)?);
                math::midplane(&a.geom, &b.geom, &tol)?
            }
            PlaneDef::TwoEdges { a, b } => {
                let a = math::line_of(&res(a)?.geom, &tol)?;
                let b = math::line_of(&res(b)?.geom, &tol)?;
                math::plane_two_lines(&a, &b, &tol)?
            }
            PlaneDef::ThreePoints { a, b, c } => {
                let a = math::point_of(&res(a)?.geom)?;
                let b = math::point_of(&res(b)?.geom)?;
                let c = math::point_of(&res(c)?.geom)?;
                math::plane_three_points(a, b, c, &tol)?
            }
            PlaneDef::TangentAtPoint { face, point } => {
                let face = res(face)?;
                let p = math::point_of(&res(point)?.geom)?;
                math::plane_tangent_at_point(&face.geom, p, &tol)?
            }
            PlaneDef::AlongPath { path, at } => {
                let curve = curve_of(&res(path)?.geom)?;
                let (v, by_distance) = match at {
                    PathPosition::Fraction(e) => (inputs.scalar(e)?, false),
                    PathPosition::Distance(e) => (inputs.length(e)?, true),
                };
                let t = math::path_param(&*curve, v, by_distance)?;
                math::plane_along_path(&*curve, t, &tol)?
            }
        };
        Ok((plane, warnings))
    }
}

fn position_expr(at: &PathPosition) -> (&Expr, Dimension) {
    match at {
        PathPosition::Fraction(e) => (e, Dimension::NONE),
        PathPosition::Distance(e) => (e, Dimension::LENGTH),
    }
}

/// A construction plane: a timeline feature whose output is `ConstructionGeom::Plane`, with
/// `ConstructionId` = its `FeatureId`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConstructPlane {
    /// The definition.
    pub def: PlaneDef,
}

impl Feature for ConstructPlane {
    fn inputs(&self) -> Vec<Dependency> {
        self.def.refs().into_iter().filter_map(dependency).collect()
    }

    fn exprs(&self) -> Vec<&Expr> {
        self.def
            .exprs_with_dim()
            .into_iter()
            .map(|(e, _)| e)
            .collect()
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        match &mut self.def {
            PlaneDef::Offset { distance, .. } => vec![(distance, Dimension::LENGTH)],
            PlaneDef::AtAngle { angle, .. } | PlaneDef::Tangent { angle, .. } => {
                vec![(angle, Dimension::ANGLE)]
            }
            PlaneDef::AlongPath { at, .. } => match at {
                PathPosition::Fraction(e) => vec![(e, Dimension::NONE)],
                PathPosition::Distance(e) => vec![(e, Dimension::LENGTH)],
            },
            PlaneDef::Midplane { .. }
            | PlaneDef::TwoEdges { .. }
            | PlaneDef::ThreePoints { .. }
            | PlaneDef::TangentAtPoint { .. } => Vec::new(),
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let (plane, warnings) = self.def.evaluate(&CtxInputs { ctx })?;
        Ok(FeatureOutput {
            construction: Some(ConstructionGeom::Plane(plane)),
            warnings,
            ..FeatureOutput::default()
        })
    }
}

register_feature!(ConstructPlane, "construct.plane", version = 1);
