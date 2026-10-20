//! Persistent references to topology (S01 §4). **Provisional** until DOC-11 is accepted.
//!
//! These are data types only, so S03/S04/S06/S13 can code against them. The `NameSeg`
//! grammar, fingerprint fields and quantisation may change with DOC-11; callers treat
//! [`TopoName`] as opaque. `NamingBuilder` and the resolver arrive with DOC-06.

use std::borrow::Cow;
use std::hash::{Hash, Hasher};
use std::sync::Arc;

use amg_core::{Aabb3, Point3, Vec3};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::id::{BodyId, ConstructionId, OccurrenceId, SketchId};
use crate::stub::{ProfileKey, SketchEntityKey};

/// Anything a feature can reference as an input.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EntityRef {
    /// A whole body.
    Body(BodyId),
    /// A face, edge, vertex or mesh face group of a body.
    Topo(TopoRef),
    /// A sketch curve.
    SketchCurve {
        /// The sketch.
        sketch: SketchId,
        /// The curve.
        curve: SketchEntityKey,
    },
    /// A sketch point.
    SketchPoint {
        /// The sketch.
        sketch: SketchId,
        /// The point.
        point: SketchEntityKey,
    },
    /// A closed sketch profile.
    Profile {
        /// The sketch.
        sketch: SketchId,
        /// The profile.
        profile: ProfileKey,
    },
    /// A construction plane, axis or point.
    Construction(ConstructionId),
    /// Entity seen through an occurrence path (assemblies, S06 joint origins).
    InContext {
        /// Occurrences from the context component down to the target's component.
        path: Vec<OccurrenceId>,
        /// The referenced entity inside that component.
        target: Box<EntityRef>,
    },
}

/// Reference to a sub-entity of a body.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TopoRef {
    /// The body.
    pub body: BodyId,
    /// Kind of sub-entity.
    pub kind: TopoKind,
    /// Persistent name (opaque).
    pub name: TopoName,
    /// Geometric fallback for resolution.
    pub fingerprint: Option<GeomFingerprint>,
}

/// Kind of a body sub-entity.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TopoKind {
    /// B-rep face.
    Face,
    /// B-rep edge.
    Edge,
    /// B-rep vertex.
    Vertex,
    /// Face group of a mesh body.
    FaceGroup,
}

/// Persistent topological name. Opaque to callers; grammar defined by DOC-11.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TopoName(Arc<[NameSeg]>);

impl TopoName {
    /// A name from its segments (naming builders only).
    pub fn new(segments: Vec<NameSeg>) -> TopoName {
        TopoName(segments.into())
    }

    /// The segments, outermost history last.
    pub fn segments(&self) -> &[NameSeg] {
        &self.0
    }
}

impl Serialize for TopoName {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        self.0.serialize(s)
    }
}

impl<'de> Deserialize<'de> for TopoName {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        Vec::<NameSeg>::deserialize(d).map(TopoName::new)
    }
}

/// One step of a name's history.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum NameSeg {
    /// Created by `feature` in `role`, optionally from a source entity.
    Generated {
        /// Creating feature.
        feature: crate::id::FeatureId,
        /// Role within the feature.
        role: Role,
        /// Entity it was generated from.
        from: Option<NameSource>,
    },
    /// Modified (but still identifiable) by `feature`.
    Modified {
        /// Modifying feature.
        feature: crate::id::FeatureId,
    },
    /// The `index`-th piece after a split.
    Split {
        /// Piece index.
        index: u32,
    },
    /// Merged with other named entities.
    Merged {
        /// The other names.
        with: Vec<TopoName>,
    },
}

/// Feature-specific role, e.g. `("Lateral", 0)` or `("EndCap", 0)`.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Role {
    /// Role tag.
    pub tag: Cow<'static, str>,
    /// Index among entities with the same tag.
    pub index: u32,
}

/// What a generated entity came from.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum NameSource {
    /// A sketch curve (extrude laterals, ...).
    SketchCurve {
        /// The sketch.
        sketch: SketchId,
        /// The curve.
        curve: SketchEntityKey,
    },
    /// Another named entity.
    Topo(TopoName),
}

/// Geometric fingerprint of a sub-entity.
///
/// `Eq`/`Hash` compare values quantised to [`GeomFingerprint::QUANTUM`] so refs are
/// hashable; resolution compares with real tolerances (DOC-06).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeomFingerprint {
    /// Surface or curve class.
    pub class: GeomClass,
    /// Area (faces), length (edges) or 0 (vertices).
    pub measure: f64,
    /// Centroid, mm.
    pub centroid: Point3,
    /// Axis or normal, when the class has one.
    pub axis: Option<Vec3>,
    /// Bounding box, mm.
    pub bbox: Aabb3,
}

impl GeomFingerprint {
    /// Quantisation step for `Eq`/`Hash`. Provisional (DOC-11 fixes it).
    pub const QUANTUM: f64 = 1e-6;

    fn quantised(&self) -> (GeomClass, Vec<i64>) {
        let q = |x: f64| {
            #[allow(
                clippy::cast_possible_truncation,
                reason = "saturating cast is the intent; magnitudes are far below i64::MAX"
            )]
            let i = (x / Self::QUANTUM).round() as i64;
            i
        };
        let mut v = vec![q(self.measure)];
        v.extend(self.centroid.to_array().map(q));
        match self.axis {
            Some(a) => {
                v.push(1);
                v.extend(a.to_array().map(q));
            }
            None => v.push(0),
        }
        v.extend(self.bbox.min.to_array().map(q));
        v.extend(self.bbox.max.to_array().map(q));
        (self.class, v)
    }
}

impl PartialEq for GeomFingerprint {
    fn eq(&self, other: &Self) -> bool {
        self.quantised() == other.quantised()
    }
}
impl Eq for GeomFingerprint {}
impl Hash for GeomFingerprint {
    fn hash<H: Hasher>(&self, h: &mut H) {
        self.quantised().hash(h);
    }
}

/// Geometric class of a sub-entity.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[allow(missing_docs, reason = "class names are self-explanatory")]
pub enum GeomClass {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    BSpline,
    Other,
    Line,
    Circle,
    Ellipse,
    Curve,
    Point,
}

/// Element map of one evaluated body: index = `amg-brep` in-shape index of that sub-shape.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyNaming {
    /// Face names.
    pub faces: Vec<TopoName>,
    /// Edge names.
    pub edges: Vec<TopoName>,
    /// Vertex names.
    pub vertices: Vec<TopoName>,
}

/// Result quality of a resolution (DOC-06).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MatchQuality {
    /// Name matched exactly.
    Exact,
    /// Name matched several candidates; fingerprint picked one.
    Disambiguated,
    /// Only the fingerprint matched (always a warning).
    FingerprintOnly,
}

/// A resolved reference.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Resolved {
    /// The body.
    pub body: BodyId,
    /// Kind of sub-entity.
    pub kind: TopoKind,
    /// In-shape index.
    pub index: u32,
    /// How confident the match is.
    pub quality: MatchQuality,
}

/// Why a reference did not resolve.
#[derive(thiserror::Error, Clone, Debug, PartialEq)]
pub enum ResolveError {
    /// The body does not exist at this point in the timeline.
    #[error("body {0:?} does not exist at this point in the timeline")]
    BodyMissing(BodyId),
    /// The entity is gone.
    #[error("referenced {kind:?} no longer exists")]
    NotFound {
        /// Kind of the referenced entity.
        kind: TopoKind,
    },
    /// Several candidates and no confident pick.
    #[error("reference is ambiguous ({candidates} candidates)")]
    Ambiguous {
        /// Number of candidates.
        candidates: u32,
    },
    /// The body kind has no sub-entity naming.
    #[error("body has no sub-entity naming (implicit/form body)")]
    Unsupported,
}

impl amg_core::Diagnostic for ResolveError {
    fn hint(&self) -> Option<String> {
        Some("Edit the feature and pick the reference again.".to_owned())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::id::FeatureId;

    fn fp(x: f64) -> GeomFingerprint {
        GeomFingerprint {
            class: GeomClass::Plane,
            measure: 100.0,
            centroid: Point3::new(x, 0.0, 0.0),
            axis: Some(Vec3::Z),
            bbox: Aabb3::new(Point3::ZERO, Point3::splat(10.0)),
        }
    }

    #[test]
    fn fingerprint_eq_is_quantised() {
        assert_eq!(fp(1.0), fp(1.0 + 1e-9));
        assert_ne!(fp(1.0), fp(1.0 + 1e-3));
        let hash = |f: &GeomFingerprint| {
            let mut h = std::collections::hash_map::DefaultHasher::new();
            f.hash(&mut h);
            h.finish()
        };
        assert_eq!(hash(&fp(2.0)), hash(&fp(2.0 + 1e-9)));
    }

    #[test]
    fn entity_ref_round_trips() {
        let name = TopoName::new(vec![
            NameSeg::Generated {
                feature: FeatureId(3),
                role: Role {
                    tag: Cow::Borrowed("Lateral"),
                    index: 0,
                },
                from: Some(NameSource::SketchCurve {
                    sketch: SketchId(FeatureId(2)),
                    curve: SketchEntityKey(7),
                }),
            },
            NameSeg::Split { index: 1 },
        ]);
        let r = EntityRef::InContext {
            path: vec![OccurrenceId(5)],
            target: Box::new(EntityRef::Topo(TopoRef {
                body: BodyId {
                    origin: FeatureId(3),
                    slot: 0,
                },
                kind: TopoKind::Face,
                name,
                fingerprint: Some(fp(1.0)),
            })),
        };
        let s = serde_json::to_string(&r).unwrap();
        let back: EntityRef = serde_json::from_str(&s).unwrap();
        assert_eq!(back, r);
        assert_eq!(serde_json::to_string(&back).unwrap(), s);
    }
}
