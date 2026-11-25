//! Profiles → planar faces (S04 §2, note §3.4).

use amg_brep::geom::{BSpline2dData, Curve2d, Loop2, PlanarProfile, ProfileEdge2, Tag};
use amg_brep::{EdgeId, FaceId};
use amg_core::Plane;
use amg_doc::naming::NameSource;
use amg_doc::stub::{ProfileKey, SketchEntityKey};
use amg_doc::{BodyId, Diagnostic, EntityRef, FeatureError, RecomputeCtx, SketchId};
use amg_sketch::{Curve2, EntityId, Profile, ProfileId, ProfileMatch};

use super::error::{SolidError, kerr};

/// Where a profile face came from.
#[derive(Clone, Debug)]
pub enum ProfileSource {
    /// A closed sketch region.
    Region {
        /// The sketch.
        sketch: SketchId,
        /// The region as resolved (its key in the current profile set).
        profile: ProfileKey,
    },
    /// A planar face of a body.
    Face {
        /// The body.
        body: BodyId,
        /// The face.
        face: FaceId,
        /// The selection key of the pick.
        key: u32,
    },
}

/// One profile as a planar face, ready to sweep.
#[derive(Clone, Debug)]
pub struct ProfileFace {
    /// The planar face (world space).
    pub face: amg_brep::Shape,
    /// The profile plane; its normal is the face normal and the extrude direction.
    pub plane: Plane,
    /// The tag of every edge: the sketch entity id (note §3.4).
    pub edge_tags: Vec<(EdgeId, Tag)>,
    /// Where it came from.
    pub source: ProfileSource,
    /// Regions of one sketch that share boundary curves share a group (they are fused into one
    /// tool solid); groups are numbered from 0 in profile order.
    pub group: u32,
}

/// The builder tag of a sketch entity.
pub fn tag_of(e: EntityId) -> Tag {
    Tag(u64::from(e.0))
}

/// The name source of a builder tag made by [`tag_of`] in `sketch`.
pub fn tag_source(sketch: SketchId, tag: Tag) -> NameSource {
    NameSource::SketchCurve {
        sketch,
        curve: SketchEntityKey(tag.0),
    }
}

/// A sketch curve as a kernel profile curve (total match, S02 §4).
pub fn to_curve2d(c: &Curve2) -> Curve2d {
    match c {
        Curve2::Line(s) => Curve2d::Line { a: s.a, b: s.b },
        Curve2::Circle(c) => Curve2d::Circle {
            center: c.center,
            radius: c.radius,
        },
        Curve2::Arc(a) => Curve2d::Arc {
            center: a.center,
            radius: a.radius,
            start_angle: a.start_angle,
            sweep: a.sweep,
        },
        Curve2::Ellipse(e) => Curve2d::Ellipse {
            center: e.center,
            major: e.major,
            minor_radius: e.minor_radius,
            t0: e.t0,
            t1: e.t1,
        },
        Curve2::BSpline(b) => Curve2d::BSpline(BSpline2dData {
            degree: u8::try_from(b.degree()).unwrap_or(u8::MAX),
            knots: b.knots().to_vec(),
            poles: b.poles().to_vec(),
            weights: b.is_rational().then(|| b.weights().to_vec()),
        }),
    }
}

/// `Region(p)`: the cap name source of a profile region, its sorted outer-loop source ids
/// (naming note §5.4).
pub fn region_source(sketch: SketchId, p: &Profile) -> NameSource {
    let mut outer: Vec<SketchEntityKey> = p
        .outer_sources()
        .iter()
        .map(|e| SketchEntityKey(u64::from(e.0)))
        .collect();
    outer.sort();
    outer.dedup();
    NameSource::Profile { sketch, outer }
}

fn loop2(l: &amg_sketch::Loop) -> Loop2 {
    Loop2 {
        edges: l
            .edges
            .iter()
            .map(|e| ProfileEdge2 {
                curve: to_curve2d(&e.curve),
                tag: tag_of(e.source),
            })
            .collect(),
    }
}

/// The sketch entities on a region's boundary (for grouping).
fn boundary_sources(p: &Profile) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = std::iter::once(&p.outer)
        .chain(&p.holes)
        .flat_map(|l| &l.edges)
        .flat_map(|e| std::iter::once(e.source).chain(e.also.iter().copied()))
        .collect();
    v.sort();
    v.dedup();
    v
}

/// The region a profile reference names in the current solve: by key, else by fingerprint
/// (with a `FingerprintOnly` warning on this feature, note §5.4).
fn resolve_region<'a>(
    ctx: &RecomputeCtx<'a>,
    r: &EntityRef,
    sketch: SketchId,
    key: ProfileKey,
    fp: Option<&amg_doc::naming::ProfileFp>,
) -> Result<&'a Profile, FeatureError> {
    let set = &ctx.sketch(sketch)?.profiles;
    if let Some(p) = set.get(ProfileId(key.0)) {
        return Ok(p);
    }
    if let Some(fp) = fp
        && let ProfileMatch::Fingerprint { new, score } = set.match_fingerprint(&fp.0)
        && let Some(p) = set.get(new)
    {
        ctx.warn(Diagnostic {
            message: format!(
                "Profile was found only by its shape (match {:.0} %); check it (FingerprintOnly).",
                score * 100.0
            ),
            cause: None,
            entities: vec![r.clone()],
        });
        return Ok(p);
    }
    Err(SolidError::Invalid {
        field: "profiles",
        msg: format!("the profile no longer exists in sketch {}", sketch.0),
    }
    .into())
}

fn parallel(a: &Plane, b: &Plane) -> bool {
    a.normal.get().dot(b.normal.get()).abs() >= 1.0 - 1e-9
}

/// Resolves profile regions (and, from SOL-02, planar faces), converts their loops
/// ([`to_curve2d`], tags [`tag_of`]) and builds the faces with one `make_planar_faces` call per
/// sketch. Faces come back in reference order; a region listed twice is used once.
///
/// # Errors
/// [`SolidError::Invalid`] for an empty list, an unsupported reference or a lost region;
/// [`SolidError::NonParallelProfiles`]; [`SolidError::NotYet`] for planar-face profiles
/// (SOL-02); the kernel's face-builder errors.
pub fn profile_faces(
    ctx: &RecomputeCtx<'_>,
    refs: &[EntityRef],
) -> Result<Vec<ProfileFace>, FeatureError> {
    if refs.is_empty() {
        return Err(SolidError::Invalid {
            field: "profiles",
            msg: "select at least one profile".into(),
        }
        .into());
    }
    // Resolved regions per sketch, in first-reference order.
    let mut sketches: Vec<(SketchId, Plane, Vec<&Profile>)> = Vec::new();
    for r in refs {
        match r {
            EntityRef::Profile {
                sketch,
                profile,
                fingerprint,
            } => {
                let p = resolve_region(ctx, r, *sketch, *profile, fingerprint.as_ref())?;
                let plane = ctx.sketch(*sketch)?.plane;
                match sketches.iter_mut().find(|(s, _, _)| s == sketch) {
                    Some((_, _, v)) => {
                        if !v.iter().any(|q| q.id == p.id) {
                            v.push(p);
                        }
                    }
                    None => sketches.push((*sketch, plane, vec![p])),
                }
            }
            EntityRef::Topo(_) => {
                return Err(SolidError::NotYet {
                    what: "planar-face profiles",
                }
                .into());
            }
            _ => {
                return Err(SolidError::Invalid {
                    field: "profiles",
                    msg: "a profile must be a sketch region or a planar face".into(),
                }
                .into());
            }
        }
    }
    let first_plane = sketches[0].1;
    if sketches.iter().any(|(_, p, _)| !parallel(p, &first_plane)) {
        return Err(SolidError::NonParallelProfiles.into());
    }
    let k = super::common::op_kernel(ctx);
    let mut out = Vec::new();
    let mut next_group = 0u32;
    for (sketch, plane, regions) in sketches {
        let profiles: Vec<PlanarProfile> = regions
            .iter()
            .map(|p| PlanarProfile {
                plane,
                outer: loop2(&p.outer),
                holes: p.holes.iter().map(loop2).collect(),
            })
            .collect();
        let builds = k
            .make_planar_faces(&profiles, ctx.tolerance())
            .map_err(kerr)?;
        let groups = group_regions(&regions);
        let base = next_group;
        next_group += groups.iter().copied().max().map_or(0, |g| g + 1);
        for ((p, fb), g) in regions.iter().zip(builds).zip(groups) {
            out.push(ProfileFace {
                face: fb.face,
                plane,
                edge_tags: fb.edge_tags,
                source: ProfileSource::Region {
                    sketch,
                    profile: ProfileKey(p.id.0),
                },
                group: base + g,
            });
        }
    }
    Ok(out)
}

/// Connected components of "shares a boundary curve", numbered in region order.
fn group_regions(regions: &[&Profile]) -> Vec<u32> {
    let n = regions.len();
    let sources: Vec<Vec<EntityId>> = regions.iter().map(|p| boundary_sources(p)).collect();
    let mut parent: Vec<usize> = (0..n).collect();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    for i in 0..n {
        for j in (i + 1)..n {
            let shared = sources[i]
                .iter()
                .any(|e| sources[j].binary_search(e).is_ok());
            if shared {
                let (a, b) = (root(&mut parent, i), root(&mut parent, j));
                parent[a.max(b)] = a.min(b);
            }
        }
    }
    let mut ids: Vec<Option<u32>> = vec![None; n];
    let mut next = 0u32;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let r = root(&mut parent, i);
        let g = *ids[r].get_or_insert_with(|| {
            next += 1;
            next - 1
        });
        out.push(g);
    }
    out
}

/// The cap name source of a profile face: `Region(p)` for a sketch region.
///
/// # Errors
/// `MissingInput` if the sketch is gone.
pub fn cap_source(ctx: &RecomputeCtx<'_>, f: &ProfileFace) -> Result<NameSource, FeatureError> {
    match &f.source {
        ProfileSource::Region { sketch, profile } => {
            let set = &ctx.sketch(*sketch)?.profiles;
            let p = set
                .get(ProfileId(profile.0))
                .ok_or_else(|| SolidError::Invalid {
                    field: "profiles",
                    msg: "the profile no longer exists".into(),
                })?;
            Ok(region_source(*sketch, p))
        }
        ProfileSource::Face { key, .. } => Ok(NameSource::Selection { key: *key }),
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "values are copied, not computed")]
mod tests {
    use super::*;
    use amg_core::{Point2, Segment2, Vec2};
    use amg_sketch::{Arc2, Circle2, EllipseArc2};

    #[test]
    fn curves_convert_field_for_field() {
        let l = to_curve2d(&Curve2::Line(Segment2 {
            a: Point2::new(0.0, 1.0),
            b: Point2::new(2.0, 3.0),
        }));
        assert_eq!(
            l,
            Curve2d::Line {
                a: Point2::new(0.0, 1.0),
                b: Point2::new(2.0, 3.0)
            }
        );
        let c = to_curve2d(&Curve2::Circle(Circle2 {
            center: Point2::new(1.0, 1.0),
            radius: 2.0,
        }));
        assert!(matches!(c, Curve2d::Circle { radius, .. } if radius == 2.0));
        let a = to_curve2d(&Curve2::Arc(Arc2 {
            center: Point2::ZERO,
            radius: 3.0,
            start_angle: 0.5,
            sweep: -1.0,
        }));
        assert!(
            matches!(a, Curve2d::Arc { radius, start_angle, sweep, .. } if radius == 3.0 && start_angle == 0.5 && sweep == -1.0)
        );
        let e = to_curve2d(&Curve2::Ellipse(EllipseArc2 {
            center: Point2::ZERO,
            major: Vec2::new(4.0, 0.0),
            minor_radius: 1.0,
            t0: 0.0,
            t1: 2.0,
        }));
        assert!(
            matches!(e, Curve2d::Ellipse { minor_radius, t1, .. } if minor_radius == 1.0 && t1 == 2.0)
        );
        let b = amg_sketch::BSpline2::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![Point2::ZERO, Point2::new(1.0, 1.0), Point2::new(2.0, 0.0)],
            vec![1.0; 3],
        )
        .unwrap();
        let Curve2d::BSpline(d) = to_curve2d(&Curve2::BSpline(b)) else {
            panic!("a B-spline");
        };
        assert_eq!(d.degree, 2);
        assert_eq!(d.poles.len(), 3);
        assert!(d.weights.is_none());
    }

    #[test]
    fn tags_are_entity_ids() {
        assert_eq!(tag_of(EntityId(42)), Tag(42));
        assert_eq!(
            tag_source(SketchId(amg_doc::FeatureId(3)), Tag(42)),
            NameSource::SketchCurve {
                sketch: SketchId(amg_doc::FeatureId(3)),
                curve: SketchEntityKey(42)
            }
        );
    }
}
