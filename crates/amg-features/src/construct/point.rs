//! The construction point feature (`construct.point` v1, CON-02).

use amg_core::{Dimension, Point3};
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

/// How a construction point is defined (Fusion's Construct menu; conventions in note §6).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum PointDef {
    /// At a vertex, sketch point or construction point.
    AtVertex {
        /// The point.
        point: RefInput,
    },
    /// Where two coplanar straight edges (or axes) cross.
    ThroughTwoEdges {
        /// First edge.
        a: RefInput,
        /// Second edge.
        b: RefInput,
    },
    /// The common point of three planes.
    ThroughThreePlanes {
        /// First plane.
        a: RefInput,
        /// Second plane.
        b: RefInput,
        /// Third plane.
        c: RefInput,
    },
    /// The centre of a circle / arc, sphere or torus.
    AtCenter {
        /// The edge or face.
        entity: RefInput,
    },
    /// Where an edge, sketch curve or axis crosses a plane.
    EdgeAndPlane {
        /// The edge.
        edge: RefInput,
        /// The plane.
        plane: RefInput,
    },
    /// At a position along a path.
    AlongPath {
        /// An edge or sketch curve.
        path: RefInput,
        /// Fraction or distance along the path.
        at: PathPosition,
    },
}

impl PointDef {
    /// Every reference input, in field order.
    pub fn refs(&self) -> Vec<&RefInput> {
        match self {
            PointDef::AtVertex { point } => vec![point],
            PointDef::ThroughTwoEdges { a, b } => vec![a, b],
            PointDef::ThroughThreePlanes { a, b, c } => vec![a, b, c],
            PointDef::AtCenter { entity } => vec![entity],
            PointDef::EdgeAndPlane { edge, plane } => vec![edge, plane],
            PointDef::AlongPath { path, .. } => vec![path],
        }
    }

    /// Computes the point from resolved inputs. Returns the point and the health warnings
    /// (references matched by fingerprint only).
    pub fn evaluate(
        &self,
        inputs: &dyn ConstructInputs,
    ) -> Result<(Point3, Vec<Diagnostic>), FeatureError> {
        let tol = inputs.tolerance();
        let mut warnings = Vec::new();
        let mut res = |r: &RefInput| -> Result<ResolvedRef, FeatureError> {
            resolve_tracked(inputs, r, &mut warnings)
        };
        let point = match self {
            PointDef::AtVertex { point } => math::point_of(&res(point)?.geom)?,
            PointDef::ThroughTwoEdges { a, b } => {
                let a = math::line_of(&res(a)?.geom, &tol)?;
                let b = math::line_of(&res(b)?.geom, &tol)?;
                math::point_two_lines(&a, &b, &tol)?
            }
            PointDef::ThroughThreePlanes { a, b, c } => {
                let a = math::plane_of(&res(a)?.geom)?;
                let b = math::plane_of(&res(b)?.geom)?;
                let c = math::plane_of(&res(c)?.geom)?;
                math::point_three_planes(&a, &b, &c, &tol)?
            }
            PointDef::AtCenter { entity } => math::center_of(&res(entity)?.geom)?,
            PointDef::EdgeAndPlane { edge, plane } => {
                let edge = res(edge)?;
                let plane = math::plane_of(&res(plane)?.geom)?;
                math::point_curve_plane(&edge.geom, &plane, &tol)?
            }
            PointDef::AlongPath { path, at } => {
                let curve = curve_of(&res(path)?.geom)?;
                let (v, by_distance) = match at {
                    PathPosition::Fraction(e) => (inputs.scalar(e)?, false),
                    PathPosition::Distance(e) => (inputs.length(e)?, true),
                };
                curve.eval(math::path_param(&*curve, v, by_distance)?)
            }
        };
        Ok((point, warnings))
    }
}

/// A construction point: a timeline feature whose output is `ConstructionGeom::Point`, with
/// `ConstructionId` = its `FeatureId`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConstructPoint {
    /// The definition.
    pub def: PointDef,
}

impl Feature for ConstructPoint {
    fn inputs(&self) -> Vec<Dependency> {
        self.def.refs().into_iter().filter_map(dependency).collect()
    }

    fn exprs(&self) -> Vec<&Expr> {
        match &self.def {
            PointDef::AlongPath {
                at: PathPosition::Fraction(e) | PathPosition::Distance(e),
                ..
            } => vec![e],
            _ => Vec::new(),
        }
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        match &mut self.def {
            PointDef::AlongPath { at, .. } => match at {
                PathPosition::Fraction(e) => vec![(e, Dimension::NONE)],
                PathPosition::Distance(e) => vec![(e, Dimension::LENGTH)],
            },
            _ => Vec::new(),
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let (point, warnings) = self.def.evaluate(&CtxInputs { ctx })?;
        Ok(FeatureOutput {
            construction: Some(ConstructionGeom::Point(point)),
            warnings,
            ..FeatureOutput::default()
        })
    }
}

register_feature!(ConstructPoint, "construct.point", version = 1);
