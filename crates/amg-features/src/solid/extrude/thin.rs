//! Thin profiles (SOL-18): the walls of a thin extrude or revolve, as planar faces ready to
//! sweep like any other profile.
//!
//! - **Chains.** A sketch region gives one closed chain per loop (outer and holes). Picked
//!   sketch curves are joined where their end points meet (within [`WELD`]) into open or closed
//!   chains; more than two curves at a point is an error. Curves that only cross are separate
//!   chains.
//! - **Orientation.** A closed chain runs counter-clockwise about the sketch normal. An open
//!   chain starts at its end on the end curve with the lower entity id (a single curve runs
//!   from its start to its end). Neither depends on dimensions or on the order of the picks.
//! - **Sides.** [`ThinSide::Side1`] widens a chain to its left (into a closed chain),
//!   [`ThinSide::Side2`] to its right (outwards), [`ThinSide::Center`] half each way.
//! - **Naming** (naming note §5.2): the wall face left of the chain is `ThinInner#side` and the
//!   one right of it `ThinOuter#side`, both from their sketch curve; the end faces of an open
//!   chain are `ThinEnd#(2·side + end)` from the sketch point at that end (`end` 0 at the
//!   chain's start); caps are `StartCap#side` / `EndCap#side` from `Profile { sketch, chain
//!   curves }`. No name depends on the thickness or the side mode.

use amg_brep::geom::{ProfileEdge2, Tag, ThinProfile};
use amg_core::{Plane, Point2};
use amg_doc::naming::NameSource;
use amg_doc::stub::SketchEntityKey;
use amg_doc::{EntityRef, FeatureError, RecomputeCtx, SketchId};
use amg_sketch::{Curve2, EntityId, EntityKind, Sketch};

use crate::solid::common::op_kernel;
use crate::solid::error::{SolidError, kerr};
use crate::solid::extent::{ThinSide, ThinSpec};
use crate::solid::profile::{
    ProfileFace, ProfileSource, Wall, WallBoundary, WallEdge, parallel, resolve_region, to_curve2d,
};

/// Curve end points closer than this are joined, mm (the weld distance of profile detection).
const WELD: f64 = 1e-6;

/// Samples per curve for the orientation of a closed chain.
const AREA_SAMPLES: u32 = 32;

fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

/// One edge of a chain, oriented along it.
#[derive(Clone, Debug)]
struct Link {
    entity: EntityId,
    curve: Curve2,
}

/// An oriented chain of sketch curves.
#[derive(Clone, Debug)]
struct Chain {
    links: Vec<Link>,
    /// The sketch points at the start and end of an open chain; `None` for a closed one.
    ends: Option<[EntityId; 2]>,
    /// The sorted curve ids of the chain (its cap name source).
    sources: Vec<EntityId>,
}

/// A picked curve with its end points.
#[derive(Clone, Debug)]
struct Picked {
    entity: EntityId,
    curve: Curve2,
    /// Point entity and position of the start and end; `None` for a circle or full ellipse.
    ends: Option<[(EntityId, Point2); 2]>,
}

fn picked(sketch: &Sketch, entity: EntityId) -> Result<Picked, FeatureError> {
    let gone = || invalid("profiles", "a selected sketch curve no longer exists");
    let kind = &sketch.entity(entity).ok_or_else(gone)?.kind;
    let points = match kind {
        EntityKind::Point { .. } => {
            return Err(invalid("profiles", "select sketch curves, not points"));
        }
        EntityKind::Circle { .. } | EntityKind::Ellipse { .. } => None,
        EntityKind::Line { start, end }
        | EntityKind::Arc { start, end, .. }
        | EntityKind::EllipticArc { start, end, .. }
        | EntityKind::Conic { start, end, .. } => Some((*start, *end)),
        EntityKind::Spline(d) => Some(d.ends().ok_or_else(gone)?),
    };
    let curve = sketch
        .curve(entity)
        .ok_or_else(|| invalid("profiles", "a selected sketch curve can't be evaluated"))?;
    let (t0, t1) = curve.param_range();
    let ends = points.map(|(a, b)| [(a, curve.eval(t0)), (b, curve.eval(t1))]);
    Ok(Picked {
        entity,
        curve,
        ends,
    })
}

/// Twice the signed area of a closed chain (positive when counter-clockwise).
fn signed_area2(links: &[Link]) -> f64 {
    let mut points = Vec::new();
    for l in links {
        let (t0, t1) = l.curve.param_range();
        for i in 0..AREA_SAMPLES {
            let s = f64::from(i) / f64::from(AREA_SAMPLES);
            points.push(l.curve.eval(t0 + (t1 - t0) * s));
        }
    }
    let n = points.len();
    (0..n)
        .map(|i| {
            let (a, b) = (points[i], points[(i + 1) % n]);
            a.x * b.y - b.x * a.y
        })
        .sum()
}

fn sorted_sources(links: &[Link]) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = links.iter().map(|l| l.entity).collect();
    v.sort();
    v.dedup();
    v
}

/// A closed chain turned counter-clockwise. A chain of one full circle stays a circle (the
/// kernel runs it counter-clockwise).
fn closed_chain(mut links: Vec<Link>, sources: Vec<EntityId>) -> Chain {
    if let [l] = links.as_mut_slice()
        && let Curve2::Arc(a) = l.curve
        && a.sweep.abs() >= std::f64::consts::TAU - 1e-12
    {
        l.curve = Curve2::Circle(amg_sketch::Circle2 {
            center: a.center,
            radius: a.radius,
        });
    }
    let lone_circle = matches!(links.as_slice(), [l] if matches!(l.curve, Curve2::Circle(_)));
    if !lone_circle && signed_area2(&links) < 0.0 {
        links.reverse();
        for l in &mut links {
            l.curve = l.curve.reversed();
        }
    }
    Chain {
        links,
        ends: None,
        sources,
    }
}

/// Joins the picked curves of one sketch into chains, ordered by their lowest curve id.
fn chains(mut curves: Vec<Picked>) -> Result<Vec<Chain>, FeatureError> {
    curves.sort_by_key(|c| c.entity);
    // Weld the end points into nodes.
    let mut nodes: Vec<Point2> = Vec::new();
    let mut node_of = |p: Point2| match nodes.iter().position(|q| (*q - p).length() <= WELD) {
        Some(i) => i,
        None => {
            nodes.push(p);
            nodes.len() - 1
        }
    };
    let at: Vec<Option<[usize; 2]>> = curves
        .iter()
        .map(|c| c.ends.map(|e| [node_of(e[0].1), node_of(e[1].1)]))
        .collect();
    // The curve ends at every node.
    let mut incident: Vec<Vec<(usize, usize)>> = vec![Vec::new(); nodes.len()];
    for (c, n) in at.iter().enumerate() {
        if let Some(n) = n {
            for (end, node) in n.iter().enumerate() {
                incident[*node].push((c, end));
            }
        }
    }
    if incident.iter().any(|v| v.len() > 2) {
        return Err(invalid(
            "profiles",
            "more than two selected curves meet at one point; a thin chain can't branch",
        ));
    }
    let mut used = vec![false; curves.len()];
    // Walks from end `entry` of curve `c`; returns the links and the entry/exit curve ends.
    let walk = |used: &mut Vec<bool>, c: usize, entry: usize| {
        let mut links = Vec::new();
        let (mut c, mut entry) = (c, entry);
        let first = (c, entry);
        loop {
            used[c] = true;
            let p = &curves[c];
            links.push(Link {
                entity: p.entity,
                curve: if entry == 0 {
                    p.curve.clone()
                } else {
                    p.curve.reversed()
                },
            });
            let exit = 1 - entry;
            let next = at[c].and_then(|n| incident[n[exit]].iter().find(|(d, _)| !used[*d]));
            match next {
                Some(&(d, e)) => (c, entry) = (d, e),
                None => return (links, first, (c, exit)),
            }
        }
    };
    let mut out = Vec::new();
    // Open chains: from the free ends, in curve id order.
    for (c, n) in at.iter().enumerate() {
        let Some(n) = n else { continue };
        for (end, node) in n.iter().enumerate() {
            if incident[*node].len() != 1 || used[c] {
                continue;
            }
            let (links, first, last) = walk(&mut used, c, end);
            let point =
                |(c, end): (usize, usize)| curves[c].ends.map_or(curves[c].entity, |e| e[end].0);
            let sources = sorted_sources(&links);
            out.push(Chain {
                ends: Some([point(first), point(last)]),
                links,
                sources,
            });
        }
    }
    // What is left is closed.
    for c in 0..curves.len() {
        if !used[c] {
            let (links, _, _) = walk(&mut used, c, 0);
            let sources = sorted_sources(&links);
            out.push(closed_chain(links, sources));
        }
    }
    out.sort_by(|a, b| a.sources.cmp(&b.sources));
    Ok(out)
}

/// The same curves over the same pieces (a loop shared by two selected regions).
fn same_wall(a: &Chain, b: &Chain) -> bool {
    let mids = |c: &Chain| {
        let mut v: Vec<(EntityId, Point2)> = c
            .links
            .iter()
            .map(|l| {
                let (t0, t1) = l.curve.param_range();
                // A full circle's pieces can start anywhere: compare its centre.
                match &l.curve {
                    Curve2::Circle(circle) => (l.entity, circle.center),
                    curve => (l.entity, curve.eval(0.5 * (t0 + t1))),
                }
            })
            .collect();
        v.sort_by(|x, y| {
            (x.0, x.1.x, x.1.y)
                .partial_cmp(&(y.0, y.1.x, y.1.y))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        v
    };
    if a.ends.is_some() != b.ends.is_some() || a.links.len() != b.links.len() {
        return false;
    }
    mids(a)
        .iter()
        .zip(mids(b))
        .all(|(x, y)| x.0 == y.0 && (x.1 - y.1).length() <= WELD)
}

/// The widths left and right of a chain.
fn widths(side: ThinSide, t: f64) -> (f64, f64) {
    match side {
        ThinSide::Side1 => (t, 0.0),
        ThinSide::Side2 => (0.0, t),
        ThinSide::Center => (0.5 * t, 0.5 * t),
    }
}

/// The chains selected in one sketch.
struct SketchPicks {
    sketch: SketchId,
    plane: Plane,
    loops: Vec<Chain>,
    curves: Vec<Picked>,
}

fn collect(ctx: &RecomputeCtx<'_>, refs: &[EntityRef]) -> Result<Vec<SketchPicks>, FeatureError> {
    let mut out: Vec<SketchPicks> = Vec::new();
    let slot = |out: &mut Vec<SketchPicks>, sketch: SketchId| -> Result<usize, FeatureError> {
        if let Some(i) = out.iter().position(|s| s.sketch == sketch) {
            return Ok(i);
        }
        out.push(SketchPicks {
            sketch,
            plane: ctx.sketch(sketch)?.plane,
            loops: Vec::new(),
            curves: Vec::new(),
        });
        Ok(out.len() - 1)
    };
    for r in refs {
        match r {
            EntityRef::Profile {
                sketch,
                profile,
                fingerprint,
            } => {
                let p = resolve_region(ctx, r, *sketch, *profile, fingerprint.as_ref())?;
                let i = slot(&mut out, *sketch)?;
                for (n, lp) in std::iter::once(&p.outer).chain(&p.holes).enumerate() {
                    let links: Vec<Link> = lp
                        .edges
                        .iter()
                        .map(|e| Link {
                            entity: e.source,
                            curve: e.curve.clone(),
                        })
                        .collect();
                    // The outer loop's cap source is the region's (`Region(p)`).
                    let sources = if n == 0 {
                        let mut v = p.outer_sources().to_vec();
                        v.sort();
                        v.dedup();
                        v
                    } else {
                        sorted_sources(&links)
                    };
                    let chain = closed_chain(links, sources);
                    if !out[i].loops.iter().any(|c| same_wall(c, &chain)) {
                        out[i].loops.push(chain);
                    }
                }
            }
            EntityRef::SketchCurve { sketch, curve } => {
                let i = slot(&mut out, *sketch)?;
                let entity = u32::try_from(curve.0)
                    .map(EntityId)
                    .map_err(|_| invalid("profiles", "unknown sketch curve"))?;
                if !out[i].curves.iter().any(|c| c.entity == entity) {
                    let p = picked(&ctx.sketch(*sketch)?.sketch, entity);
                    if p.is_err() {
                        ctx.highlight([r.clone()]);
                    }
                    out[i].curves.push(p?);
                }
            }
            _ => {
                return Err(invalid(
                    "profiles",
                    "a thin feature takes sketch regions or sketch curves",
                ));
            }
        }
    }
    Ok(out)
}

/// The wall faces of a thin feature's selection, in first-reference sketch order. All walls
/// form one group, so overlapping walls are fused into one tool.
///
/// # Errors
/// [`SolidError::Invalid`] for a zero or negative thickness, an unsupported reference, a lost
/// curve or region or a branching selection; [`SolidError::NonParallelProfiles`]; the kernel's
/// wall errors (a wall wider than the chain allows).
pub(crate) fn thin_faces(
    ctx: &RecomputeCtx<'_>,
    refs: &[EntityRef],
    spec: &ThinSpec,
) -> Result<Vec<ProfileFace>, FeatureError> {
    if refs.is_empty() {
        return Err(invalid("profiles", "select at least one profile or curve"));
    }
    let t = ctx.eval_length(&spec.thickness)?;
    if !t.is_finite() || t <= 2.0 * ctx.tolerance().linear {
        return Err(invalid("thickness", "the thickness must be positive"));
    }
    let (left, right) = widths(spec.side, t);
    let picks = collect(ctx, refs)?;
    if let Some(first) = picks.first()
        && picks.iter().any(|s| !parallel(&s.plane, &first.plane))
    {
        return Err(SolidError::NonParallelProfiles.into());
    }
    let k = op_kernel(ctx);
    let mut out = Vec::new();
    for s in picks {
        let mut walls = s.loops;
        for c in chains(s.curves)? {
            if !walls.iter().any(|w| same_wall(w, &c)) {
                walls.push(c);
            }
        }
        for chain in walls {
            ctx.check_cancelled()?;
            let n = chain.links.len();
            // Tag `i < n` is chain edge `i`; `n` and `n + 1` are the end caps.
            let tag = |i: usize| Tag(i as u64);
            let profile = ThinProfile {
                plane: s.plane,
                chain: chain
                    .links
                    .iter()
                    .enumerate()
                    .map(|(i, l)| ProfileEdge2 {
                        curve: to_curve2d(&l.curve),
                        tag: tag(i),
                    })
                    .collect(),
                closed: chain.ends.is_none(),
                left,
                right,
                end_tags: [tag(n), tag(n + 1)],
            };
            let built = k.make_thin_face(&profile, ctx.tolerance()).map_err(kerr)?;
            let mut edges = Vec::with_capacity(built.build.edge_tags.len());
            for (edge, t) in &built.build.edge_tags {
                let i = usize::try_from(t.0).unwrap_or(usize::MAX);
                let (boundary, source) = if let Some(link) = chain.links.get(i) {
                    let boundary = if built.left.contains(edge) {
                        WallBoundary::Inner
                    } else {
                        WallBoundary::Outer
                    };
                    let source = NameSource::SketchCurve {
                        sketch: s.sketch,
                        curve: SketchEntityKey(u64::from(link.entity.0)),
                    };
                    (boundary, source)
                } else {
                    let end = u32::from(i != n);
                    let Some(ends) = chain.ends else {
                        continue;
                    };
                    let source = NameSource::SketchPoint {
                        sketch: s.sketch,
                        point: SketchEntityKey(u64::from(ends[end as usize].0)),
                    };
                    (WallBoundary::End(end), source)
                };
                edges.push(WallEdge {
                    edge: *edge,
                    boundary,
                    source,
                });
            }
            let cap = NameSource::Profile {
                sketch: s.sketch,
                outer: chain
                    .sources
                    .iter()
                    .map(|e| SketchEntityKey(u64::from(e.0)))
                    .collect(),
            };
            let curve_refs: Vec<EntityRef> = refs
                .iter()
                .filter(|r| match r {
                    EntityRef::Profile { sketch, .. } => *sketch == s.sketch,
                    EntityRef::SketchCurve { sketch, curve } => {
                        *sketch == s.sketch
                            && chain.sources.iter().any(|e| u64::from(e.0) == curve.0)
                    }
                    _ => false,
                })
                .cloned()
                .collect();
            out.push(ProfileFace {
                face: built.build.face,
                plane: s.plane,
                edge_tags: Vec::new(),
                source: ProfileSource::Wall(Wall {
                    cap,
                    edges,
                    refs: curve_refs,
                }),
                group: 0,
            });
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::{Segment2, Vec2};
    use amg_sketch::Circle2;

    fn line(entity: u32, a: (u32, f64, f64), b: (u32, f64, f64)) -> Picked {
        let (pa, pb) = (Point2::new(a.1, a.2), Point2::new(b.1, b.2));
        Picked {
            entity: EntityId(entity),
            curve: Curve2::Line(Segment2 { a: pa, b: pb }),
            ends: Some([(EntityId(a.0), pa), (EntityId(b.0), pb)]),
        }
    }

    fn start(c: &Chain, i: usize) -> Point2 {
        c.links[i].curve.eval(c.links[i].curve.param_range().0)
    }

    #[test]
    fn an_open_chain_starts_at_the_lower_end_curve_in_any_pick_order() {
        // 10: (10,0)->(0,0) [drawn backwards]; 11: (10,0)->(10,5).
        let a = line(10, (1, 10.0, 0.0), (2, 0.0, 0.0));
        let b = line(11, (1, 10.0, 0.0), (3, 10.0, 5.0));
        for picks in [vec![a.clone(), b.clone()], vec![b, a]] {
            let c = chains(picks).unwrap();
            assert_eq!(c.len(), 1);
            // Starts at point 2, the free end of curve 10.
            assert_eq!(c[0].ends, Some([EntityId(2), EntityId(3)]));
            assert_eq!(start(&c[0], 0), Point2::new(0.0, 0.0));
            assert_eq!(start(&c[0], 1), Point2::new(10.0, 0.0));
            assert_eq!(c[0].sources, [EntityId(10), EntityId(11)]);
        }
    }

    #[test]
    fn a_single_curve_runs_from_its_start() {
        let c = chains(vec![line(5, (1, 4.0, 0.0), (2, 0.0, 0.0))]).unwrap();
        assert_eq!(c[0].ends, Some([EntityId(1), EntityId(2)]));
        assert_eq!(start(&c[0], 0), Point2::new(4.0, 0.0));
    }

    #[test]
    fn closed_chains_run_counter_clockwise_and_separate_chains_stay_apart() {
        // A clockwise triangle, a separate line and a circle.
        let circle = Picked {
            entity: EntityId(2),
            curve: Curve2::Circle(Circle2 {
                center: Point2::ZERO + Vec2::new(50.0, 0.0),
                radius: 1.0,
            }),
            ends: None,
        };
        let picks = vec![
            line(7, (1, 0.0, 0.0), (2, 0.0, 4.0)),
            line(8, (2, 0.0, 4.0), (3, 4.0, 0.0)),
            line(9, (3, 4.0, 0.0), (1, 0.0, 0.0)),
            line(12, (20, 20.0, 0.0), (21, 30.0, 0.0)),
            circle,
        ];
        let c = chains(picks).unwrap();
        assert_eq!(c.len(), 3);
        assert_eq!(c[0].sources, [EntityId(2)]);
        assert!(c[0].ends.is_none());
        assert_eq!(c[1].sources, [EntityId(7), EntityId(8), EntityId(9)]);
        assert!(c[1].ends.is_none());
        assert!(signed_area2(&c[1].links) > 0.0);
        assert!(c[2].ends.is_some());
    }

    #[test]
    fn a_branch_is_an_error() {
        let picks = vec![
            line(1, (1, 0.0, 0.0), (2, 1.0, 0.0)),
            line(2, (1, 0.0, 0.0), (3, 0.0, 1.0)),
            line(3, (1, 0.0, 0.0), (4, -1.0, 0.0)),
        ];
        assert!(chains(picks).unwrap_err().to_string().contains("branch"));
    }

    #[test]
    fn side_modes_split_the_thickness() {
        assert_eq!(widths(ThinSide::Side1, 2.0), (2.0, 0.0));
        assert_eq!(widths(ThinSide::Side2, 2.0), (0.0, 2.0));
        assert_eq!(widths(ThinSide::Center, 2.0), (1.0, 1.0));
    }
}
