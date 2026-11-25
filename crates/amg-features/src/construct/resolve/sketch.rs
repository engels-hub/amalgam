//! Sketch curves and points → [`RefGeom`] on the sketch plane (S03 world mapping).

use std::sync::Arc;

use amg_core::{Plane, Segment3, Transform};
use amg_doc::stub::{SketchEntityKey, SketchGeom};
use amg_sketch::{Curve2, EntityId};

use super::super::curves::FreeCurve;
use super::super::{RefGeom, ResolvedRef, math};
use super::{Fail, brep};

fn entity_id(key: SketchEntityKey) -> Result<EntityId, Fail> {
    u32::try_from(key.0)
        .map(EntityId)
        .map_err(|_| Fail::Missing(format!("sketch entity {}", key.0)))
}

fn on_plane(geom: RefGeom, plane: Plane) -> ResolvedRef {
    ResolvedRef {
        geom,
        context_plane: Some(plane),
        quality: None,
    }
}

/// Sketch point `key` of `g`, placed by `t`.
pub(super) fn point(
    g: &SketchGeom,
    key: SketchEntityKey,
    t: &Transform,
) -> Result<ResolvedRef, Fail> {
    let plane = math::transform_plane(t, &g.plane);
    let p = g
        .sketch
        .point(entity_id(key)?)
        .ok_or_else(|| Fail::Missing(format!("sketch point {}", key.0)))?;
    Ok(on_plane(RefGeom::Point(plane.to_world(p)), plane))
}

/// Sketch curve `key` of `g`, placed by `t`.
pub(super) fn curve(
    g: &SketchGeom,
    key: SketchEntityKey,
    t: &Transform,
) -> Result<ResolvedRef, Fail> {
    let plane = math::transform_plane(t, &g.plane);
    let curve = g
        .sketch
        .curve(entity_id(key)?)
        .ok_or_else(|| Fail::Missing(format!("sketch curve {}", key.0)))?;
    let geom = match curve {
        Curve2::Line(s) => RefGeom::Segment(Segment3 {
            a: plane.to_world(s.a),
            b: plane.to_world(s.b),
        }),
        Curve2::Circle(c) => RefGeom::Circle {
            center: plane.to_world(c.center),
            normal: plane.normal,
            radius: c.radius,
            arc: None,
        },
        Curve2::Arc(a) => {
            // A clockwise arc runs counter-clockwise about the opposite normal.
            let normal = if a.sweep < 0.0 {
                -plane.normal
            } else {
                plane.normal
            };
            let (sin, cos) = a.start_angle.sin_cos();
            brep::circle(
                plane.to_world(a.center),
                normal,
                a.radius,
                plane.x.get() * cos + plane.y.get() * sin,
                a.sweep.abs(),
            )
        }
        other @ (Curve2::Ellipse(_) | Curve2::BSpline(_)) => RefGeom::Curve(Arc::new(
            FreeCurve::planar(other, plane).map_err(|e| Fail::Geometry(e.to_string()))?,
        )),
    };
    Ok(on_plane(geom, plane))
}
