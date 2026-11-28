//! Path patterns (S04 §5, SOL-23): instances along a chain of edges or sketch curves.
//!
//! The path curves are resolved through CON-03 (`resolve_input` + [`curve_of`]) and chained
//! end to end in the given order (each curve reversed where needed). Positions are arc lengths:
//! the seed sits at `start · L` (instance 0), instance `k` at `s_0 ± k · step` (`−` when
//! `flip`), with `step` the spacing or `extent / (count − 1)`. A closed path wraps around; on
//! an open path, positions beyond its ends are dropped with a warning (their numbers stay
//! reserved, so references to the others don't move).
//!
//! Orientation: `Identical` translates by `C(s_k) − C(s_0)`; `PathDirection` maps the path
//! frame at `s_0` onto the frame at `s_k`, with frames from
//! [`plane_along_path`](crate::construct::math::plane_along_path) (normal = unit tangent, x =
//! principal normal on curved stretches).

use std::sync::Arc;

use amg_core::{Plane, Point3, Tolerance, Transform, Vec3};
use amg_doc::{EntityRef, FeatureError, RecomputeCtx};

use super::{PathOrientation, Spacing, count_of, invalid};
use crate::construct::curves::curve_of;
use crate::construct::math::plane_along_path;
use crate::construct::resolve::resolve_input;
use crate::construct::{CurveEval, RefInput};

/// `c` run backwards over the same parameter range.
#[derive(Debug)]
struct Reversed(Arc<dyn CurveEval>);

impl Reversed {
    fn flip(&self, t: f64) -> f64 {
        let (t0, t1) = self.0.range();
        t0 + t1 - t
    }
}

impl CurveEval for Reversed {
    fn range(&self) -> (f64, f64) {
        self.0.range()
    }
    fn eval(&self, t: f64) -> Point3 {
        self.0.eval(self.flip(t))
    }
    fn d1(&self, t: f64) -> Vec3 {
        -self.0.d1(self.flip(t))
    }
    fn d2(&self, t: f64) -> Vec3 {
        self.0.d2(self.flip(t))
    }
    fn length(&self) -> f64 {
        self.0.length()
    }
    fn param_at_length(&self, s: f64) -> f64 {
        self.flip(self.0.param_at_length(self.0.length() - s))
    }
    fn closest_param(&self, p: Point3) -> f64 {
        self.flip(self.0.closest_param(p))
    }
}

fn start_point(c: &dyn CurveEval) -> Point3 {
    c.eval(c.range().0)
}

fn end_point(c: &dyn CurveEval) -> Point3 {
    c.eval(c.range().1)
}

/// The path curves chained head to tail, with cumulative arc lengths.
#[derive(Debug)]
pub struct PathChain {
    curves: Vec<Arc<dyn CurveEval>>,
    /// `cum[i]` = arc length at the start of curve `i`; `cum[n]` = the total length.
    cum: Vec<f64>,
    closed: bool,
}

/// Curves whose ends are closer than this meet (mm). B-rep edges meet within their vertex
/// tolerance, sketch curves exactly; 1e-4 mm covers both.
const GAP: f64 = 1e-4;

impl PathChain {
    /// Chains `curves` in order, reversing each where needed so that it starts where the
    /// previous one ends.
    ///
    /// # Errors
    /// `Invalid { field: "path" }` for an empty list, a zero-length path or curves that don't
    /// meet end to end.
    pub fn new(curves: Vec<Arc<dyn CurveEval>>) -> Result<PathChain, FeatureError> {
        if curves.is_empty() {
            return Err(invalid("path", "select at least one path curve"));
        }
        let near = |a: Point3, b: Point3| (a - b).length() <= GAP;
        let mut out: Vec<Arc<dyn CurveEval>> = Vec::with_capacity(curves.len());
        for (i, c) in curves.into_iter().enumerate() {
            if i == 0 {
                out.push(c);
                continue;
            }
            if i == 1 {
                // Orient the first curve towards the second.
                let first = &out[0];
                let (a, b) = (start_point(&*c), end_point(&*c));
                let e0 = end_point(&**first);
                let s0 = start_point(&**first);
                if !near(e0, a) && !near(e0, b) && (near(s0, a) || near(s0, b)) {
                    let r: Arc<dyn CurveEval> = Arc::new(Reversed(Arc::clone(first)));
                    out[0] = r;
                }
            }
            let prev = end_point(&*out[out.len() - 1]);
            if near(prev, start_point(&*c)) {
                out.push(c);
            } else if near(prev, end_point(&*c)) {
                out.push(Arc::new(Reversed(c)));
            } else {
                return Err(invalid(
                    "path",
                    format!("path curve {} doesn't start where curve {i} ends", i + 1),
                ));
            }
        }
        let mut cum = Vec::with_capacity(out.len() + 1);
        let mut total = 0.0;
        for c in &out {
            cum.push(total);
            total += c.length();
        }
        cum.push(total);
        if !(total.is_finite() && total > GAP) {
            return Err(invalid("path", "the path has no length"));
        }
        // A full circle, or a chain whose last curve ends where the first starts.
        let closed = near(start_point(&*out[0]), end_point(&*out[out.len() - 1]));
        Ok(PathChain {
            curves: out,
            cum,
            closed,
        })
    }

    /// The total arc length, mm.
    pub fn length(&self) -> f64 {
        self.cum[self.cum.len() - 1]
    }

    /// Whether the chain ends where it starts.
    pub fn is_closed(&self) -> bool {
        self.closed
    }

    /// The curve and its parameter at arc length `s ∈ [0, length]`.
    fn locate(&self, s: f64) -> (&dyn CurveEval, f64) {
        let n = self.curves.len();
        let i = self.cum[1..n].partition_point(|&c| c <= s).min(n - 1);
        let c = &*self.curves[i];
        let local = (s - self.cum[i]).clamp(0.0, c.length());
        let (t0, t1) = c.range();
        (c, c.param_at_length(local).clamp(t0, t1))
    }

    /// The point at arc length `s`.
    pub fn point_at(&self, s: f64) -> Point3 {
        let (c, t) = self.locate(s);
        c.eval(t)
    }

    /// The path frame at arc length `s` ([`plane_along_path`]).
    ///
    /// # Errors
    /// `Invalid { field: "path" }` where the tangent vanishes.
    pub fn frame_at(&self, s: f64, tol: &Tolerance) -> Result<Plane, FeatureError> {
        let (c, t) = self.locate(s);
        plane_along_path(c, t, tol).map_err(|e| invalid("path", format!("no path frame: {e}")))
    }
}

/// Resolves and chains the path curves of `refs`. A failure highlights the reference.
pub(crate) fn resolve_path(
    ctx: &RecomputeCtx<'_>,
    refs: &[EntityRef],
) -> Result<PathChain, FeatureError> {
    let mut curves = Vec::with_capacity(refs.len());
    for r in refs {
        let input = RefInput::Ref(r.clone());
        let g = resolve_input(ctx, &input)?.geom;
        let c = curve_of(&g).map_err(|e| {
            ctx.highlight([r.clone()]);
            invalid("path", format!("not a path curve: {e}"))
        })?;
        curves.push(c);
    }
    PathChain::new(curves).inspect_err(|_| ctx.highlight(refs.iter().cloned()))
}

/// The evaluated settings of a path pattern.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PathSpec {
    pub(crate) count: u32,
    pub(crate) step: f64,
    pub(crate) start: f64,
    pub(crate) orientation: PathOrientation,
    pub(crate) flip: bool,
}

/// Placed instances `(k, transform)`.
type Instances = Vec<(u32, Transform)>;

/// Instance `(k, transform)` along `chain` for `k = 1 … count − 1`, and the instance numbers
/// that fell beyond the ends of an open path.
pub(crate) fn path_instances(
    chain: &PathChain,
    spec: &PathSpec,
    tol: &Tolerance,
) -> Result<(Instances, Vec<u32>), FeatureError> {
    let len = chain.length();
    let s0 = spec.start * len;
    let f0 = match spec.orientation {
        PathOrientation::PathDirection => Some(chain.frame_at(s0, tol)?.to_transform().inverse()),
        PathOrientation::Identical => None,
    };
    let p0 = chain.point_at(s0);
    let sign = if spec.flip { -1.0 } else { 1.0 };
    let mut v = Vec::new();
    let mut dropped = Vec::new();
    for k in 1..spec.count {
        let mut s = s0 + sign * spec.step * f64::from(k);
        if chain.is_closed() {
            s = s.rem_euclid(len);
        } else if s < -GAP || s > len + GAP {
            dropped.push(k);
            continue;
        }
        let s = s.clamp(0.0, len);
        let t = match &f0 {
            Some(inv) => chain.frame_at(s, tol)?.to_transform() * *inv,
            None => Transform::from_translation(chain.point_at(s) - p0),
        };
        v.push((k, t));
    }
    Ok((v, dropped))
}

/// Evaluates count, spacing and start of a path pattern.
pub(crate) fn path_spec(
    ctx: &RecomputeCtx<'_>,
    count: &amg_doc::Expr,
    spacing: &Spacing,
    start: &amg_doc::Expr,
    orientation: PathOrientation,
    flip: bool,
) -> Result<PathSpec, FeatureError> {
    let count = count_of(ctx, count, "count")?;
    let start = ctx.eval_scalar(start)?;
    if !(0.0..=1.0).contains(&start) {
        return Err(invalid(
            "start",
            format!("the start is a fraction of the path in [0, 1], not {start}"),
        ));
    }
    let step = match spacing {
        Spacing::Spacing(e) => ctx.eval_length(e)?,
        Spacing::Extent(e) => {
            let ext = ctx.eval_length(e)?;
            if count > 1 {
                ext / f64::from(count - 1)
            } else {
                0.0
            }
        }
    };
    if !step.is_finite() || step < 0.0 {
        return Err(invalid(
            "spacing",
            "the spacing must be a finite length ≥ 0 (use flip to reverse)",
        ));
    }
    Ok(PathSpec {
        count,
        step,
        start,
        orientation,
        flip,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::construct::curves::{CircleCurve, SegmentCurve};
    use amg_core::{Dir3, Segment3};

    fn seg(a: [f64; 3], b: [f64; 3]) -> Arc<dyn CurveEval> {
        Arc::new(
            SegmentCurve::new(&Segment3 {
                a: Point3::from_array(a),
                b: Point3::from_array(b),
            })
            .unwrap(),
        )
    }

    fn spec(count: u32, step: f64, start: f64, o: PathOrientation, flip: bool) -> PathSpec {
        PathSpec {
            count,
            step,
            start,
            orientation: o,
            flip,
        }
    }

    #[test]
    fn chains_reverse_curves_that_run_backwards() {
        // First curve given backwards, second forwards, third backwards.
        let c = PathChain::new(vec![
            seg([10.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            seg([10.0, 0.0, 0.0], [10.0, 10.0, 0.0]),
            seg([20.0, 10.0, 0.0], [10.0, 10.0, 0.0]),
        ])
        .unwrap();
        assert!((c.length() - 30.0).abs() < 1e-12);
        assert!(!c.is_closed());
        assert!((c.point_at(0.0) - Point3::ZERO).length() < 1e-12);
        assert!((c.point_at(15.0) - Point3::new(10.0, 5.0, 0.0)).length() < 1e-12);
        assert!((c.point_at(30.0) - Point3::new(20.0, 10.0, 0.0)).length() < 1e-12);
        let err = PathChain::new(vec![
            seg([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            seg([5.0, 0.0, 0.0], [6.0, 0.0, 0.0]),
        ])
        .unwrap_err();
        assert!(err.to_string().contains("doesn't start"), "{err}");
    }

    #[test]
    fn identical_orientation_translates_and_drops_beyond_the_end() {
        let c = PathChain::new(vec![seg([0.0, 0.0, 0.0], [40.0, 0.0, 0.0])]).unwrap();
        let (v, dropped) = path_instances(
            &c,
            &spec(6, 10.0, 0.25, PathOrientation::Identical, false),
            &Tolerance::DEFAULT,
        )
        .unwrap();
        assert_eq!(v.iter().map(|x| x.0).collect::<Vec<_>>(), vec![1, 2, 3]);
        assert_eq!(dropped, vec![4, 5]);
        let p = v[2].1.transform_point3(Point3::new(10.0, 3.0, 0.0));
        assert!((p - Point3::new(40.0, 3.0, 0.0)).length() < 1e-12);
        let (v, _) = path_instances(
            &c,
            &spec(2, 10.0, 0.25, PathOrientation::Identical, true),
            &Tolerance::DEFAULT,
        )
        .unwrap();
        assert!((v[0].1.transform_point3(Point3::ZERO).x + 10.0).abs() < 1e-12);
    }

    #[test]
    fn path_direction_follows_a_circle_and_wraps() {
        let circle: Arc<dyn CurveEval> =
            Arc::new(CircleCurve::new(Point3::ZERO, Dir3::Z, 10.0, None).unwrap());
        let c = PathChain::new(vec![circle]).unwrap();
        assert!(c.is_closed());
        let quarter = c.length() / 4.0;
        let (v, dropped) = path_instances(
            &c,
            &spec(5, quarter, 0.0, PathOrientation::PathDirection, false),
            &Tolerance::DEFAULT,
        )
        .unwrap();
        assert!(dropped.is_empty());
        assert_eq!(v.len(), 4);
        // A quarter turn about Z: (12, 0, 0) → (0, 12, 0); the full turn returns home.
        let p = v[0].1.transform_point3(Point3::new(12.0, 0.0, 0.0));
        assert!((p - Point3::new(0.0, 12.0, 0.0)).length() < 1e-9, "{p}");
        let p = v[3].1.transform_point3(Point3::new(12.0, 0.0, 1.0));
        assert!((p - Point3::new(12.0, 0.0, 1.0)).length() < 1e-9, "{p}");
    }
}
