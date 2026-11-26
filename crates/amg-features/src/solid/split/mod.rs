//! `solid.split_body` and `solid.split_face` (S04, SOL-07; naming note §5.2 "Split body" and
//! "Split face" rows).
//!
//! The split tool of both is resolved here ([`split_tool`]): an origin or construction plane
//! becomes a square face covering the body; a planar face is used as is or, with
//! `extend_tool`, as its whole plane over the body; a curved face as is or extended to its
//! untrimmed surface (SOL-02's `extended_face`); a body (split body, or the surface tool of a
//! split face) as is. Split face can also cut along sketch curves swept along a direction
//! ([`curves_sheet`], a prism sheet from each curve's wire, R-S04 "prism sheet from a wire").

mod body;
mod dialog;
mod face;

use amg_brep::geom::{Curve3, CurveSeg, Tag};
use amg_brep::{BrepKernel, PrismExtent, PrismOpts, Shape};
use amg_core::{Aabb3, Dir3, Plane, Vec3};
use amg_doc::naming::NameSource;
use amg_doc::{BodyNaming, Dependency, EntityRef, FeatureError, RecomputeCtx};

pub use body::SplitBody;
pub use face::{SplitFace, SplitFaceTool};

use super::common::{input_solid, with_job};
use super::error::{SolidError, kerr, with_hint};
use super::extent::{Target, resolve_target};
use super::extrude::sides::{extended_face, plane_face};
use crate::construct::RefInput;
use crate::construct::resolve::{resolve_input, resolve_plane_input};

fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

/// [`SolidError::NoSplit`] with its "extend the tool" hint, the tool highlighted.
pub(crate) fn no_split(ctx: &RecomputeCtx<'_>, tool: &RefInput) -> FeatureError {
    ctx.highlight(tool.entity().cloned());
    with_hint(&SolidError::NoSplit)
}

/// The dependency of a split tool: `Body` for a body tool (read whole), else as the pattern
/// inputs.
pub(crate) fn tool_dependency(r: &RefInput) -> Option<Dependency> {
    match r {
        RefInput::Ref(EntityRef::Body(b)) => Some(Dependency::Body(*b)),
        other => super::pattern::ref_dependency(other),
    }
}

/// A resolved split tool.
pub(crate) struct SplitTool<'a> {
    /// A face (bounded or covering the work box) or a solid.
    pub shape: Shape,
    /// The tool's plane for planar tools (pieces are ordered along its normal).
    pub plane: Option<Plane>,
    /// The tool body's names (body tools only; its faces keep them on the cut).
    pub naming: Option<&'a BodyNaming>,
    /// Name source of the faces the tool leaves on the pieces (face and plane tools).
    pub source: Option<NameSource>,
}

/// The tool of a split (module docs) over the work box `w`.
///
/// # Errors
/// Resolution errors (tool highlighted); [`SolidError::ExtendUnsupported`];
/// [`SolidError::Invalid`] for a point, axis or curve.
pub(crate) fn split_tool<'a>(
    ctx: &RecomputeCtx<'a>,
    k: &dyn BrepKernel,
    r: &RefInput,
    extend: bool,
    w: &Aabb3,
) -> Result<SplitTool<'a>, FeatureError> {
    let tol = ctx.tolerance().linear;
    let e = match r {
        RefInput::Origin(_) => {
            let plane = resolve_plane_input(ctx, r)?;
            return Ok(SplitTool {
                shape: plane_face(k, &plane, w, tol)?,
                plane: Some(plane),
                naming: None,
                source: None,
            });
        }
        RefInput::Ref(e) => e,
    };
    if let EntityRef::Body(b) = e {
        let res = input_solid(ctx, *b);
        if res.is_err() {
            ctx.highlight([e.clone()]);
        }
        let (shape, naming) = res?;
        return Ok(SplitTool {
            shape: shape.clone(),
            plane: None,
            naming: Some(naming),
            source: None,
        });
    }
    let source = match e {
        EntityRef::Topo(t) => Some(NameSource::Selection {
            key: if t.key == 0 { 1 } else { t.key },
        }),
        EntityRef::Construction(c) => Some(NameSource::Construction(*c)),
        _ => None,
    };
    match resolve_target(ctx, e, "tool")? {
        Target::Plane { plane, face } => {
            let shape = match face {
                Some(f) if !extend => f.sheet()?,
                _ => plane_face(k, &plane, w, tol)?,
            };
            Ok(SplitTool {
                shape,
                plane: Some(plane),
                naming: None,
                source,
            })
        }
        Target::Face(f) => Ok(SplitTool {
            shape: if extend {
                extended_face(ctx, k, &f, w, e)?
            } else {
                f.sheet()?
            },
            plane: None,
            naming: None,
            source,
        }),
        Target::Body { shape } => Ok(SplitTool {
            shape: shape.clone(),
            plane: None,
            naming: None,
            source: None,
        }),
        Target::Point(_) => {
            ctx.highlight([e.clone()]);
            Err(invalid(
                "tool",
                "the split tool must be a plane, a face or a body",
            ))
        }
    }
}

/// One sketch curve as a wire segment, moved by `shift`.
fn curve_seg(
    ctx: &RecomputeCtx<'_>,
    r: &EntityRef,
    shift: Vec3,
) -> Result<(CurveSeg, Option<Plane>), FeatureError> {
    let res = resolve_input(ctx, &RefInput::Ref(r.clone()))?;
    let tag = Tag(0);
    let seg = match res.geom {
        crate::construct::RefGeom::Segment(s) => {
            let d = s.b - s.a;
            let len = d.length();
            let Some(dir) = Dir3::new(d, ctx.tolerance()) else {
                ctx.highlight([r.clone()]);
                return Err(invalid("curves", "a curve has zero length"));
            };
            CurveSeg {
                curve: Curve3::Line {
                    origin: s.a + shift,
                    dir,
                },
                t0: 0.0,
                t1: len,
                tag,
            }
        }
        crate::construct::RefGeom::Circle {
            center,
            normal,
            radius,
            arc,
        } => {
            let (t0, mut t1) = arc.unwrap_or((0.0, std::f64::consts::TAU));
            if t1 <= t0 {
                t1 += std::f64::consts::TAU;
            }
            CurveSeg {
                curve: Curve3::Circle {
                    frame: Plane::from_normal(center + shift, normal),
                    radius,
                },
                t0,
                t1,
                tag,
            }
        }
        crate::construct::RefGeom::Curve(_) => {
            ctx.highlight([r.clone()]);
            return Err(SolidError::NotYet {
                what: "split face along free-form sketch curves",
            }
            .into());
        }
        g => {
            ctx.highlight([r.clone()]);
            return Err(invalid(
                "curves",
                format!("pick sketch lines or arcs, not a {}", g.kind_name()),
            ));
        }
    };
    Ok((seg, res.context_plane))
}

/// A sheet swept from each of `curves` along `dir` (`None` = the first curve's sketch normal)
/// across the work box `w` both ways: one prism of each curve's wire, all in one compound.
///
/// # Errors
/// [`SolidError::Invalid`] for an empty pick, a non-curve, or no direction; resolution and
/// kernel errors.
pub(crate) fn curves_sheet(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    curves: &[EntityRef],
    dir: Option<&RefInput>,
    w: &Aabb3,
) -> Result<Shape, FeatureError> {
    if curves.is_empty() {
        return Err(invalid("curves", "select the sketch curves to split along"));
    }
    let explicit = match dir {
        Some(d) => Some(super::pattern::line_input(ctx, d, "direction")?.dir),
        None => None,
    };
    let mut normal = explicit;
    let mut starts = Vec::with_capacity(curves.len());
    for c in curves {
        let (seg, plane) = curve_seg(ctx, c, Vec3::ZERO)?;
        if normal.is_none() {
            normal = plane.map(|p| p.normal);
        }
        starts.push(
            seg.start()
                .ok_or_else(|| invalid("curves", "a curve can't be evaluated"))?,
        );
    }
    let Some(d) = normal else {
        return Err(invalid(
            "direction",
            "the curves have no sketch plane; pick a direction",
        ));
    };
    let d = d.get();
    // Each sheet runs from `reach` before the work box's centre (measured along `d`) to
    // `reach` past it, so it crosses every face of the box.
    let reach = w.diagonal() + 1.0;
    let mut sheets = Vec::with_capacity(curves.len());
    for (c, p0) in curves.iter().zip(starts) {
        let along = (w.center() - p0).dot(d);
        let (seg, _) = curve_seg(ctx, c, d * (along - reach))?;
        let wire = k
            .make_wire(std::slice::from_ref(&seg), ctx.tolerance())
            .map_err(kerr)?;
        let sheet = with_job(ctx, |job| {
            k.prism(
                &wire.shape,
                &PrismExtent::Vector(d * (2.0 * reach)),
                &PrismOpts {
                    taper: 0.0,
                    merge_coplanar: false,
                },
                job,
            )
        })
        .map_err(|e| {
            ctx.highlight([c.clone()]);
            kerr(e)
        })?;
        sheets.push(sheet.shape);
    }
    if sheets.len() == 1 {
        return Ok(sheets.remove(0));
    }
    k.make_compound(&sheets).map_err(kerr)
}
