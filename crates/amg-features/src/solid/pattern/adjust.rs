//! "Adjust" compute of feature patterns (S04 §5, SOL-23).
//!
//! Each instance rebuilds every seed's tool with the seed's **sketches moved** by the instance
//! transform, against the unmoved state just before the pattern. Everything else the seed
//! reads (extent targets, "All" bodies, start planes) stays where it is, so extents and
//! terminations are recomputed per instance: a to-object extrude reaches a sloped target at
//! each instance's own height (Fusion's "Adjust"). The tools are then named
//! `instanced(self, k)` and applied with one body operation, as for "identical" compute.
//!
//! A seed without a sketch (a planar-face profile) can't be moved this way and fails with a
//! clear error; "identical" compute still handles it.

use std::sync::Arc;

use amg_core::{Dir3, Plane, Tolerance, Transform};
use amg_doc::{Dependency, Evaluated, FeatureError, FeatureId, RecomputeCtx, SketchId};

use super::{invalid, seed_tool_in};
use crate::solid::common::ToolBody;

/// `p` moved by the rigid transform `t` (orthonormal, right-handed under rounding).
pub(crate) fn move_plane(t: &Transform, p: &Plane) -> Plane {
    let tol = Tolerance::DEFAULT;
    let map = |d: Dir3| Dir3::new(t.transform_vector3(d.get()), &tol).unwrap_or(d);
    let x = map(p.x);
    let y_img = map(p.y);
    let normal = Dir3::new(x.get().cross(y_img.get()), &tol).unwrap_or(p.normal);
    let y = Dir3::new(normal.get().cross(x.get()), &tol).unwrap_or(y_img);
    Plane {
        origin: t.transform_point3(p.origin),
        x,
        y,
        normal,
    }
}

/// The sketches every seed reads, per seed (in seed order).
fn seed_sketches(
    ctx: &RecomputeCtx<'_>,
    seeds: &[FeatureId],
) -> Result<Vec<SketchId>, FeatureError> {
    let design = ctx.design();
    let mut all = Vec::new();
    for &id in seeds {
        let Some((_, node)) = design.timeline().node(id) else {
            return Err(FeatureError::MissingInput(Box::new(Dependency::Feature(
                id,
            ))));
        };
        let mut found = false;
        for d in node.feature.get().inputs() {
            if let Dependency::Sketch(s) = d {
                found = true;
                if !all.contains(&s) {
                    all.push(s);
                }
            }
        }
        if !found {
            return Err(invalid(
                "compute",
                format!(
                    "adjust compute moves the features' sketches, and feature {} has none; \
                     use identical compute",
                    id.0
                ),
            ));
        }
    }
    Ok(all)
}

/// The tools of every instance `(k, t)`: each seed rebuilt with its sketches moved by `t`.
/// Returns `(k, tools in seed order)` per instance.
pub(crate) fn adjusted_tools(
    ctx: &RecomputeCtx<'_>,
    seeds: &[FeatureId],
    inst: &[(u32, Transform)],
) -> Result<Vec<(u32, Vec<ToolBody>)>, FeatureError> {
    if seeds.is_empty() {
        return Err(invalid("features", "select at least one feature"));
    }
    let sketches = seed_sketches(ctx, seeds)?;
    let mut out = Vec::with_capacity(inst.len());
    for (k, t) in inst {
        ctx.check_cancelled()?;
        let mut moved: Evaluated = ctx.before().clone();
        for &s in &sketches {
            let g = ctx.sketch(s)?;
            let mut g2 = g.clone();
            g2.plane = move_plane(t, &g.plane);
            moved.set_sketch(s, Arc::new(g2));
        }
        let mut tools = Vec::with_capacity(seeds.len());
        for &id in seeds {
            tools.push(seed_tool_in(ctx, &moved, id)?);
        }
        out.push((*k, tools));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::{Point3, Vec3};

    #[test]
    fn moved_planes_stay_orthonormal_and_follow_the_transform() {
        let p = Plane::from_normal(Point3::new(1.0, 2.0, 3.0), Dir3::Z);
        let t = Transform::from_translation(Vec3::new(5.0, 0.0, 0.0))
            * Transform::from_axis_angle(Vec3::Y, 0.3);
        let q = move_plane(&t, &p);
        assert!((q.origin - t.transform_point3(p.origin)).length() < 1e-12);
        assert!((q.normal.get() - t.transform_vector3(Vec3::Z)).length() < 1e-12);
        assert!(q.x.get().dot(q.normal.get()).abs() < 1e-15);
        assert!((q.x.get().cross(q.y.get()) - q.normal.get()).length() < 1e-15);
        let uv = amg_core::Point2::new(3.0, -4.0);
        assert!((q.to_world(uv) - t.transform_point3(p.to_world(uv))).length() < 1e-12);
        assert_eq!(move_plane(&Transform::IDENTITY, &p).origin, p.origin);
    }
}
