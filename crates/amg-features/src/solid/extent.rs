//! Extents of extrude, revolve, hole and rib (S04 §4, note §3): the data, and the resolution
//! of start and extent targets ([`resolve_target`]) shared by the features that use them.

use amg_brep::{FaceId, Shape, SubKind, SubShape};
use amg_core::{Plane, Point3};
use amg_doc::{Dim, EntityRef, Expr, ExprSrc, FeatureError, RecomputeCtx, TopoKind};
use serde::{Deserialize, Serialize};

use super::common::input_solid;
use super::error::SolidError;
use crate::construct::resolve::resolve_input;
use crate::construct::{RefGeom, RefInput};

/// How far a side goes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Extent {
    /// A distance (mm, extrude) or an angle (rad, revolve).
    Distance(Expr),
    /// Up to a plane, face, body or vertex, with an offset; `extend` uses the target's
    /// untrimmed surface.
    ToObject {
        /// The target.
        target: EntityRef,
        /// Offset from the target (mm).
        offset: Expr,
        /// Extend the target face.
        extend: bool,
    },
    /// Through everything in the context component.
    All,
    /// Revolve only: 360°.
    Full,
}

/// One side of an extent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExtentSide {
    /// How far.
    pub extent: Extent,
    /// Taper angle (rad; revolve ignores it).
    pub taper: Expr,
}

impl ExtentSide {
    /// A distance side without taper.
    pub fn distance(d: Expr) -> ExtentSide {
        ExtentSide {
            extent: Extent::Distance(d),
            taper: Expr::Inline(ExprSrc("0 deg".into())),
        }
    }
}

/// Which sides of the profile an extent covers.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Direction {
    /// Along the profile normal (or against it with `flip`).
    #[default]
    OneSide,
    /// Side 1 along the normal, side 2 against it, each with its own extent.
    TwoSides,
    /// Both ways by half the distance (or the whole distance each way).
    Symmetric {
        /// Each side gets the whole distance.
        whole_length: bool,
    },
}

/// Where the extent starts.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum Start {
    /// At the profile plane.
    #[default]
    Profile,
    /// Offset along the normal (mm).
    Offset(Expr),
    /// At a planar object, with an offset.
    Object {
        /// The plane or planar face.
        target: EntityRef,
        /// Offset (mm).
        offset: Expr,
    },
}

/// Which side of an open chain a thin feature grows to.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ThinSide {
    /// Side 1.
    #[default]
    Side1,
    /// Side 2.
    Side2,
    /// Centred.
    Center,
}

/// Thin-feature options (SOL-18).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThinSpec {
    /// Wall thickness (mm).
    pub thickness: Expr,
    /// Side.
    pub side: ThinSide,
}

/// Every expression of an extent side with its dimension (`length` for distances: mm for
/// extrude, rad for revolve).
pub(crate) fn side_exprs(s: &mut ExtentSide, length: Dim) -> Vec<(&mut Expr, Dim)> {
    let mut v = Vec::new();
    match &mut s.extent {
        Extent::Distance(e) => v.push((e, length)),
        Extent::ToObject { offset, .. } => v.push((offset, Dim::LENGTH)),
        Extent::All | Extent::Full => {}
    }
    v.push((&mut s.taper, Dim::ANGLE));
    v
}

/// The entity references of a start and its sides.
pub(crate) fn extent_refs<'a>(
    start: &'a Start,
    sides: impl Iterator<Item = &'a ExtentSide> + 'a,
) -> impl Iterator<Item = EntityRef> + 'a {
    let s = match start {
        Start::Object { target, .. } => Some(target.clone()),
        Start::Profile | Start::Offset(_) => None,
    };
    s.into_iter().chain(sides.filter_map(|x| match &x.extent {
        Extent::ToObject { target, .. } => Some(target.clone()),
        _ => None,
    }))
}

// ---- target resolution (SOL-02) ---------------------------------------------------------------

/// A face of an input body.
#[derive(Clone, Copy, Debug)]
pub(crate) struct BodyFace<'a> {
    /// The body's B-rep.
    pub shape: &'a Shape,
    /// The face.
    pub face: FaceId,
}

impl BodyFace<'_> {
    /// The face as a shape of its own (shares TShapes with the body, keeps its orientation).
    pub fn sheet(&self) -> Result<Shape, FeatureError> {
        self.shape
            .sub_shape(SubShape {
                kind: SubKind::Face,
                index: self.face.0,
            })
            .map_err(super::error::kerr)
    }
}

/// A resolved start or extent target.
#[derive(Clone, Debug)]
pub(crate) enum Target<'a> {
    /// A plane with its outward normal: a construction or origin plane (`face: None`,
    /// unbounded) or a planar face.
    Plane {
        /// The plane (a face's plane points out of its material).
        plane: Plane,
        /// The planar face, if the target is one.
        face: Option<BodyFace<'a>>,
    },
    /// A vertex, sketch point or construction point.
    Point(Point3),
    /// A non-planar face.
    Face(BodyFace<'a>),
    /// A whole body.
    Body {
        /// The body's B-rep.
        shape: &'a Shape,
    },
}

fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

/// Resolves a start or extent target (S04 §2): B-rep faces, vertices and bodies through
/// DOC-06 naming, everything else through CON-03 (`resolve_input`). A failure highlights the
/// reference.
///
/// # Errors
/// The resolution error; [`SolidError::Invalid`] for a target that is not a plane, face, body
/// or point (an edge, a sketch curve, an axis).
pub(crate) fn resolve_target<'a>(
    ctx: &RecomputeCtx<'a>,
    r: &EntityRef,
    field: &'static str,
) -> Result<Target<'a>, FeatureError> {
    let res = resolve_target_inner(ctx, r, field);
    if res.is_err() {
        ctx.highlight([r.clone()]);
    }
    res
}

fn resolve_target_inner<'a>(
    ctx: &RecomputeCtx<'a>,
    r: &EntityRef,
    field: &'static str,
) -> Result<Target<'a>, FeatureError> {
    match r {
        EntityRef::Topo(t) => {
            let res = ctx.resolve(t)?;
            let (shape, _) = input_solid(ctx, res.body)?;
            match res.kind {
                TopoKind::Face => {
                    let f = BodyFace {
                        shape,
                        face: FaceId(res.index),
                    };
                    match shape.surface(f.face) {
                        amg_brep::geom::Surface::Plane(p) => Ok(Target::Plane {
                            plane: if shape.face_reversed(f.face) {
                                p.flipped()
                            } else {
                                p
                            },
                            face: Some(f),
                        }),
                        _ => Ok(Target::Face(f)),
                    }
                }
                TopoKind::Vertex => Ok(Target::Point(
                    shape.vertex_point(amg_brep::VertexId(res.index)),
                )),
                _ => Err(invalid(
                    field,
                    "the target must be a plane, a face, a body or a point",
                )),
            }
        }
        EntityRef::Body(b) => {
            let (shape, _) = input_solid(ctx, *b)?;
            Ok(Target::Body { shape })
        }
        other => {
            let g = resolve_input(ctx, &RefInput::Ref(other.clone()))?;
            match g.geom {
                RefGeom::Plane(p) => Ok(Target::Plane {
                    plane: p,
                    face: None,
                }),
                RefGeom::Point(p) => Ok(Target::Point(p)),
                g => Err(invalid(
                    field,
                    format!(
                        "the target must be a plane, a face, a body or a point, not a {}",
                        g.kind_name()
                    ),
                )),
            }
        }
    }
}
