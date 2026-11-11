//! The construction axis feature (`construct.axis` v1, CON-02).

use amg_core::{Dimension, Line3};
use amg_doc::{
    ConstructionGeom, Dependency, Diagnostic, Expr, Feature, FeatureError, FeatureOutput,
    RecomputeCtx, register_feature,
};
use serde::{Deserialize, Serialize};

use super::{ConstructInputs, CtxInputs, RefInput, ResolvedRef, dependency, math, resolve_tracked};

/// How a construction axis is defined (Fusion's Construct menu; conventions in note §6).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum AxisDef {
    /// The axis of a cylinder, cone or torus face, or of a circular edge (its normal through
    /// the centre).
    ThroughRevolved {
        /// The face or edge.
        face_or_edge: RefInput,
    },
    /// Along the face normal, through a point that lies on the face.
    PerpendicularAtPoint {
        /// The face.
        face: RefInput,
        /// A point on the face.
        point: RefInput,
    },
    /// The intersection line of two planes.
    ThroughTwoPlanes {
        /// First plane (the direction is `a.normal × b.normal`).
        a: RefInput,
        /// Second plane.
        b: RefInput,
    },
    /// Through two points, directed `a → b`.
    ThroughTwoPoints {
        /// Start point.
        a: RefInput,
        /// Sets the direction.
        b: RefInput,
    },
    /// Along a straight edge, sketch line or axis, directed start → end.
    ThroughEdge {
        /// The edge.
        edge: RefInput,
    },
    /// Along the face normal, through the face point closest to `point`.
    PerpendicularToFaceAtPoint {
        /// The face.
        face: RefInput,
        /// Any point.
        point: RefInput,
    },
}

impl AxisDef {
    /// Every reference input, in field order.
    pub fn refs(&self) -> Vec<&RefInput> {
        match self {
            AxisDef::ThroughRevolved { face_or_edge } => vec![face_or_edge],
            AxisDef::PerpendicularAtPoint { face, point }
            | AxisDef::PerpendicularToFaceAtPoint { face, point } => vec![face, point],
            AxisDef::ThroughTwoPlanes { a, b } | AxisDef::ThroughTwoPoints { a, b } => {
                vec![a, b]
            }
            AxisDef::ThroughEdge { edge } => vec![edge],
        }
    }

    /// Computes the axis from resolved inputs. Returns the axis and the health warnings
    /// (references matched by fingerprint only).
    pub fn evaluate(
        &self,
        inputs: &dyn ConstructInputs,
    ) -> Result<(Line3, Vec<Diagnostic>), FeatureError> {
        let tol = inputs.tolerance();
        let mut warnings = Vec::new();
        let mut res = |r: &RefInput| -> Result<ResolvedRef, FeatureError> {
            resolve_tracked(inputs, r, &mut warnings)
        };
        let axis = match self {
            AxisDef::ThroughRevolved { face_or_edge } => math::axis_of(&res(face_or_edge)?.geom)?,
            AxisDef::PerpendicularAtPoint { face, point } => {
                let face = res(face)?;
                let p = math::point_of(&res(point)?.geom)?;
                math::axis_perpendicular_at_point(&face.geom, p, true, &tol)?
            }
            AxisDef::PerpendicularToFaceAtPoint { face, point } => {
                let face = res(face)?;
                let p = math::point_of(&res(point)?.geom)?;
                math::axis_perpendicular_at_point(&face.geom, p, false, &tol)?
            }
            AxisDef::ThroughTwoPlanes { a, b } => {
                let a = math::plane_of(&res(a)?.geom)?;
                let b = math::plane_of(&res(b)?.geom)?;
                math::axis_two_planes(&a, &b, &tol)?
            }
            AxisDef::ThroughTwoPoints { a, b } => {
                let a = math::point_of(&res(a)?.geom)?;
                let b = math::point_of(&res(b)?.geom)?;
                math::axis_two_points(a, b, &tol)?
            }
            AxisDef::ThroughEdge { edge } => math::line_of(&res(edge)?.geom, &tol)?,
        };
        Ok((axis, warnings))
    }
}

/// A construction axis: a timeline feature whose output is `ConstructionGeom::Axis`, with
/// `ConstructionId` = its `FeatureId`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConstructAxis {
    /// The definition.
    pub def: AxisDef,
}

impl Feature for ConstructAxis {
    fn inputs(&self) -> Vec<Dependency> {
        self.def.refs().into_iter().filter_map(dependency).collect()
    }

    /// No axis variant has a parameter.
    fn exprs(&self) -> Vec<&Expr> {
        Vec::new()
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let (axis, warnings) = self.def.evaluate(&CtxInputs { ctx })?;
        Ok(FeatureOutput {
            construction: Some(ConstructionGeom::Axis(axis)),
            warnings,
            ..FeatureOutput::default()
        })
    }
}

register_feature!(ConstructAxis, "construct.axis", version = 1);
