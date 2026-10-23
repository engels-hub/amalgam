//! Offset of splines and conics: a fit-point spline approximating the exact offset curve.
//!
//! The exact offset `O(t) = C(t) ± d·n(t)` of a spline is not a spline. Each selected spline or
//! conic is sampled on its knot spans, the samples are interpolated by a fit-point spline whose
//! end tangents are the offset's end tangents, and the sampling is refined (the number of
//! samples per span doubles, up to six times) until the *measured* distance of the fitted curve
//! from the source stays within [`OffsetOptions::spline_tol`](super::OffsetOptions) of `d`. The
//! copy is a new unconstrained fit-point spline. It is not trimmed against itself or other
//! curves, and it is refused where `d` reaches the radius of curvature (the offset would have a
//! cusp).

use amg_core::{Point2, Vec2};

use super::{MIN_LEN, OffsetOptions, OffsetResult};
use crate::error::SketchError;
use crate::geom::{BSpline2, Curve2, fit_spline};
use crate::ids::EntityId;
use crate::model::Sketch;

/// Smallest allowed `1 − d·κ` along the offset: below it the offset is (nearly) singular.
const MIN_SPEED_FACTOR: f64 = 0.02;
/// Largest number of sample doublings.
const MAX_REFINE: usize = 6;

struct Plan {
    points: Vec<Point2>,
    start_tangent: Vec2,
    end_tangent: Vec2,
    construction: bool,
}

pub(super) fn offset_splines(
    s: &mut Sketch,
    ids: &[EntityId],
    distance: f64,
    side: Point2,
    constrain: bool,
    opts: &OffsetOptions,
) -> Result<OffsetResult, SketchError> {
    if constrain {
        return Err(SketchError::NotApplicable(
            "a constrained offset of splines and conics is not supported (the copy is an approximation)",
        ));
    }
    if !(opts.spline_tol.is_finite() && opts.spline_tol > 0.0) {
        return Err(SketchError::InvalidValue {
            what: "spline offset tolerance",
            value: opts.spline_tol,
        });
    }
    let mut plans = Vec::new();
    let mut seen = std::collections::BTreeSet::new();
    for &id in ids {
        if !seen.insert(id) {
            return Err(SketchError::NotApplicable(
                "an entity is listed twice in the chain",
            ));
        }
        let ent = s.entity(id).ok_or(SketchError::UnknownEntity(id))?;
        let Some(Curve2::BSpline(b)) = s.curve(id) else {
            return Err(SketchError::NotApplicable(
                "a selection that mixes splines or conics with other curves cannot be offset yet",
            ));
        };
        plans.push(plan(ent.construction, &b, distance, side, opts.spline_tol)?);
    }
    let before: std::collections::BTreeSet<EntityId> = s.entities().map(|(i, _)| i).collect();
    let mut out = OffsetResult::default();
    let res = plans.iter().try_for_each(|p| {
        let id = s.add_spline_fit(&p.points, Some(p.start_tangent), Some(p.end_tangent))?;
        s.set_construction(id, p.construction)?;
        out.copies.push(id);
        Ok::<(), SketchError>(())
    });
    out.created = s
        .entities()
        .map(|(i, _)| i)
        .filter(|i| !before.contains(i))
        .collect();
    if let Err(e) = res {
        let _ = s.remove_entities(&out.created);
        return Err(e);
    }
    Ok(out)
}

fn plan(
    construction: bool,
    c: &BSpline2,
    d: f64,
    side: Point2,
    tol: f64,
) -> Result<Plan, SketchError> {
    // Side: left or right of the direction of travel at the point nearest to `side`.
    let (t0, q) = c.closest(side);
    let to_side = side - q;
    if to_side.length() <= MIN_LEN {
        return Err(SketchError::NotApplicable(
            "the side point lies on the spline",
        ));
    }
    let tan = c.derivative(t0);
    let sgn = if tan.perp_dot(to_side) >= 0.0 {
        1.0
    } else {
        -1.0
    };

    // The offset must not reach the centre of curvature anywhere.
    let span = c.span();
    let n_check = 200 * c.spans().len();
    for i in 0..=n_check {
        let t = span * i as f64 / n_check as f64;
        let f = 1.0 - sgn * d * c.curvature_at(t);
        if f < MIN_SPEED_FACTOR {
            return Err(SketchError::Degenerate(
                "the offset distance reaches the radius of curvature of the spline",
            ));
        }
    }

    let normal = |t: f64| {
        let v = c.derivative(t);
        let len = v.length();
        if len > 0.0 {
            v.perp() / len * sgn
        } else {
            Vec2::ZERO
        }
    };
    let point = |t: f64| c.eval(t) + normal(t) * d;
    let tangent_dir = |t: f64| {
        let v = c.derivative(t);
        let len = v.length();
        if len > 0.0 { v / len } else { Vec2::X }
    };
    for k in 0..=MAX_REFINE {
        let per_span = (8 + 4 * c.degree()) << k;
        let mut ts = Vec::new();
        for (lo, hi) in c.spans() {
            for i in 0..per_span {
                ts.push(lo + (hi - lo) * i as f64 / per_span as f64);
            }
        }
        ts.push(span);
        let mut pts: Vec<Point2> = Vec::with_capacity(ts.len());
        for &t in &ts {
            let p = point(t);
            if pts.last().is_none_or(|q| q.distance(p) > MIN_LEN) {
                pts.push(p);
            }
        }
        if pts.len() < 2 {
            return Err(SketchError::Degenerate("the offset curve is empty"));
        }
        // Sampling must not be coarser than the offset's own features: a closing last point
        // that was dropped is restored.
        let end_pt = point(span);
        if pts.last().is_some_and(|q| q.distance(end_pt) > 0.0) {
            pts.push(end_pt);
        }
        let chord0 = pts[0].distance(pts[1]);
        let chord1 = pts[pts.len() - 1].distance(pts[pts.len() - 2]);
        let (v0, v1) = (tangent_dir(0.0) * chord0, tangent_dir(span) * chord1);
        let fit = fit_spline(&pts, Some(v0), Some(v1))?;
        // Measure the real property: distance from the source minus `d`.
        let m = 8 * pts.len();
        let mut err = 0.0_f64;
        for i in 0..=m {
            let p = fit.eval(fit.span() * i as f64 / m as f64);
            let dist = c.closest(p).1.distance(p);
            err = err.max((dist - d).abs());
        }
        if err <= tol {
            return Ok(Plan {
                points: pts,
                start_tangent: v0,
                end_tangent: v1,
                construction,
            });
        }
    }
    Err(SketchError::Degenerate(
        "the offset of the spline could not be approximated within the tolerance",
    ))
}
