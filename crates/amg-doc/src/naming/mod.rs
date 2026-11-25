//! Persistent references to topology (S01 §4; scheme in `docs/notes/persistent-naming.md`,
//! DOC-11; framework DOC-06).
//!
//! - **Names.** A face name starts with its generator ([`NameSeg::Gen`]) and records the
//!   structural events that happen to it later ([`NameSeg::Split`], [`NameSeg::Merge`],
//!   [`NameSeg::Inst`]). Edges and vertices are named by their adjacent faces
//!   ([`NameSeg::Bound`]), recomputed after every op. Indices in names are hints only.
//! - **Element maps.** [`BodyNaming`] holds the name of every face, edge and vertex of an
//!   evaluated body (index = `amg-brep` in-shape index) or of every mesh face group, plus
//!   tombstones of removed faces.
//! - **Building.** Features build element maps with [`NamingBuilder`] from S02's
//!   [`amg_brep::ShapeHistory`] (note §4).
//! - **Resolving.** [`resolve`] maps a stored [`TopoRef`] to an in-shape index with a
//!   [`MatchQuality`] and an optional [`ResolveWarning`], or fails with a [`ResolveError`]
//!   (note §6). It never returns a wrong match silently: anything uncertain is a warning or an
//!   error.
//!
//! Callers treat [`TopoName`] as opaque and compare references only by resolving them.

mod builder;
mod intern;
mod resolve;
mod topo;

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::sync::{Arc, OnceLock};

use amg_core::{Aabb3, Point3, Tolerance, Vec3};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::id::{BodyId, ConstructionId, FeatureId, OccurrenceId, SketchId};
use crate::stub::{ProfileKey, SketchEntityKey};

pub use builder::{NamingBuilder, NamingError, NamingReport, OperandNaming, SplitOrder};
pub(crate) use intern::WireNaming;
pub(crate) use resolve::error_message;
pub use resolve::{
    GeomSource, KernelGeom, NamingView, default_kernel, resolve, resolve_set, resolve_with,
    topo_ref_at, warning_message,
};
pub use topo::{NamingTopology, ShapeTopo, SyntheticTopology};

/// Version of the naming grammar, rules and thresholds. Part of `ENGINE_SALT`: a change
/// invalidates every cached element map (note §8.2).
pub const NAMING_VERSION: u32 = 1;

/// Nested `Topo` sources deeper than this are replaced by [`NameSource::Digest`] (note §3.1).
pub const MAX_NAME_DEPTH: usize = 8;

// ---- references ---------------------------------------------------------------------------

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
        /// Geometric fallback when the key no longer exists (note §5.4).
        #[serde(default)]
        fingerprint: Option<ProfileFp>,
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

/// S03 [`amg_sketch::ProfileFingerprint`] with `Eq`/`Hash` quantised like
/// [`GeomFingerprint`] and a serde form, so profile references stay hashable.
#[derive(Clone, Debug)]
pub struct ProfileFp(pub amg_sketch::ProfileFingerprint);

#[derive(Serialize, Deserialize)]
struct ProfileFpWire {
    area: f64,
    centroid: [f64; 2],
    bbox: [f64; 4],
    sources: Vec<u32>,
    outer_sources: Vec<u32>,
}

impl ProfileFp {
    fn wire(&self) -> ProfileFpWire {
        let f = &self.0;
        ProfileFpWire {
            area: f.area,
            centroid: [f.centroid.x, f.centroid.y],
            bbox: [f.bbox.min.x, f.bbox.min.y, f.bbox.max.x, f.bbox.max.y],
            sources: f.sources.iter().map(|e| e.0).collect(),
            outer_sources: f.outer_sources.iter().map(|e| e.0).collect(),
        }
    }

    fn quantised(&self) -> Vec<i64> {
        let w = self.wire();
        let mut v = vec![q(w.area), q(w.centroid[0]), q(w.centroid[1])];
        v.extend(w.bbox.map(q));
        v.push(i64::MAX);
        v.extend(w.sources.iter().map(|&s| i64::from(s)));
        v.push(i64::MAX);
        v.extend(w.outer_sources.iter().map(|&s| i64::from(s)));
        v
    }
}

impl Serialize for ProfileFp {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        self.wire().serialize(s)
    }
}

impl<'de> Deserialize<'de> for ProfileFp {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let w = ProfileFpWire::deserialize(d)?;
        let ids = |v: Vec<u32>| v.into_iter().map(amg_sketch::EntityId).collect();
        Ok(ProfileFp(amg_sketch::ProfileFingerprint {
            area: w.area,
            centroid: amg_core::Point2::new(w.centroid[0], w.centroid[1]),
            bbox: amg_core::Aabb2::new(
                amg_core::Point2::new(w.bbox[0], w.bbox[1]),
                amg_core::Point2::new(w.bbox[2], w.bbox[3]),
            ),
            sources: ids(w.sources),
            outer_sources: ids(w.outer_sources),
        }))
    }
}

impl PartialEq for ProfileFp {
    fn eq(&self, other: &Self) -> bool {
        self.quantised() == other.quantised()
    }
}
impl Eq for ProfileFp {}
impl Hash for ProfileFp {
    fn hash<H: Hasher>(&self, h: &mut H) {
        self.quantised().hash(h);
    }
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
    /// Per-selection key, allocated by whoever edits the feature data (max + 1 within the
    /// feature, never reused) and kept by Accept match and Re-pick (note §5.1). `0` means
    /// "position + 1" (old or hand-written data).
    #[serde(default)]
    pub key: u32,
}

/// Kind of a body sub-entity.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
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

impl TopoKind {
    /// The `amg-brep` index space of a B-rep kind (`None` for face groups).
    pub fn sub_kind(self) -> Option<amg_brep::SubKind> {
        match self {
            TopoKind::Face => Some(amg_brep::SubKind::Face),
            TopoKind::Edge => Some(amg_brep::SubKind::Edge),
            TopoKind::Vertex => Some(amg_brep::SubKind::Vertex),
            TopoKind::FaceGroup => None,
        }
    }

    /// The kind of an `amg-brep` index space.
    pub fn from_sub_kind(k: amg_brep::SubKind) -> TopoKind {
        match k {
            amg_brep::SubKind::Face => TopoKind::Face,
            amg_brep::SubKind::Edge => TopoKind::Edge,
            amg_brep::SubKind::Vertex => TopoKind::Vertex,
        }
    }
}

// ---- names ----------------------------------------------------------------------------------

/// 64-bit FNV-1a, written field by field so the value is the same on every platform and Rust
/// version (used for [`NameSource::Digest`] and the cached name hash).
#[derive(Clone, Copy)]
pub(crate) struct Fnv(u64);

impl Fnv {
    pub(crate) const fn new() -> Fnv {
        Fnv(0xcbf2_9ce4_8422_2325)
    }
    pub(crate) fn bytes(&mut self, b: &[u8]) {
        for &x in b {
            self.0 ^= u64::from(x);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    pub(crate) fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }
    pub(crate) fn u32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }
    pub(crate) fn tag(&mut self, t: u8) {
        self.bytes(&[t]);
    }
    pub(crate) fn finish(self) -> u64 {
        self.0
    }
}

struct NameInner {
    hash: u64,
    depth: u8,
    segs: Box<[NameSeg]>,
}

/// Persistent topological name: an origin segment followed by events, oldest first (note §3).
///
/// Cheap to clone (one `Arc`). The 64-bit hash of the segments is cached, so `Eq` rejects
/// unequal names quickly; `Ord` and serde (a list of [`NameSeg`]) see the segments only.
#[derive(Clone)]
pub struct TopoName(Arc<NameInner>);

impl TopoName {
    /// A name from its segments (naming builders only).
    pub fn new(segments: Vec<NameSeg>) -> TopoName {
        let mut h = Fnv::new();
        let mut depth = 0u8;
        for s in &segments {
            hash_seg(s, &mut h);
            depth = depth.max(seg_depth(s));
        }
        TopoName(Arc::new(NameInner {
            hash: h.finish(),
            depth,
            segs: segments.into_boxed_slice(),
        }))
    }

    /// The segments, oldest first.
    pub fn segments(&self) -> &[NameSeg] {
        &self.0.segs
    }

    /// Stable 64-bit hash of the segments.
    pub fn stable_hash(&self) -> u64 {
        self.0.hash
    }

    /// Nesting depth: 0 for a name without nested names, else 1 + the deepest nested name.
    pub fn depth(&self) -> usize {
        usize::from(self.0.depth)
    }

    /// True if both are the same allocation (shared, unchanged names).
    pub fn ptr_eq(&self, other: &TopoName) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    /// This name with `seg` appended.
    #[must_use]
    pub fn with(&self, seg: NameSeg) -> TopoName {
        let mut v = self.segments().to_vec();
        v.push(seg);
        TopoName::new(v)
    }

    /// The origin segment ([`NameSeg::Gen`] or [`NameSeg::Bound`]), if any.
    pub fn origin(&self) -> Option<&NameSeg> {
        self.segments().first()
    }
}

fn seg_depth(s: &NameSeg) -> u8 {
    let nested = |names: &[TopoName]| {
        names
            .iter()
            .map(|n| n.0.depth.saturating_add(1))
            .max()
            .unwrap_or(0)
    };
    match s {
        NameSeg::Gen {
            from: Some(NameSource::Topo(n)),
            ..
        } => n.0.depth.saturating_add(1),
        NameSeg::Bound { faces, .. } => nested(faces),
        NameSeg::Merge { with, .. } => nested(with),
        _ => 0,
    }
}

fn hash_role(r: &Role, h: &mut Fnv) {
    h.bytes(r.tag.as_bytes());
    h.tag(0xff);
    h.u32(r.index);
}

fn hash_source(s: &NameSource, h: &mut Fnv) {
    match s {
        NameSource::SketchCurve { sketch, curve } => {
            h.tag(1);
            h.u64(sketch.0.0);
            h.u64(curve.0);
        }
        NameSource::SketchPoint { sketch, point } => {
            h.tag(2);
            h.u64(sketch.0.0);
            h.u64(point.0);
        }
        NameSource::Profile { sketch, outer } => {
            h.tag(3);
            h.u64(sketch.0.0);
            h.u64(outer.len() as u64);
            for o in outer {
                h.u64(o.0);
            }
        }
        NameSource::Selection { key } => {
            h.tag(4);
            h.u32(*key);
        }
        NameSource::Construction(c) => {
            h.tag(5);
            h.u64(c.0.0);
        }
        NameSource::Topo(n) => {
            h.tag(6);
            h.u64(n.stable_hash());
        }
        NameSource::Digest(d) => {
            h.tag(7);
            h.u64(*d);
        }
    }
}

fn hash_seg(s: &NameSeg, h: &mut Fnv) {
    match s {
        NameSeg::Gen {
            feature,
            role,
            from,
        } => {
            h.tag(1);
            h.u64(feature.0);
            hash_role(role, h);
            match from {
                Some(src) => hash_source(src, h),
                None => h.tag(0),
            }
        }
        NameSeg::Bound {
            faces,
            index,
            count,
        } => {
            h.tag(2);
            h.u64(faces.len() as u64);
            for f in faces {
                h.u64(f.stable_hash());
            }
            h.u32(*index);
            h.u32(*count);
        }
        NameSeg::Split {
            feature,
            index,
            count,
        } => {
            h.tag(3);
            h.u64(feature.0);
            h.u32(*index);
            h.u32(*count);
        }
        NameSeg::Merge { feature, with } => {
            h.tag(4);
            h.u64(feature.0);
            h.u64(with.len() as u64);
            for w in with {
                h.u64(w.stable_hash());
            }
        }
        NameSeg::Inst { feature, instance } => {
            h.tag(5);
            h.u64(feature.0);
            h.u32(*instance);
        }
    }
}

impl PartialEq for TopoName {
    fn eq(&self, other: &TopoName) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
            || (self.0.hash == other.0.hash && self.0.segs == other.0.segs)
    }
}
impl Eq for TopoName {}
impl Hash for TopoName {
    fn hash<H: Hasher>(&self, h: &mut H) {
        h.write_u64(self.0.hash);
    }
}
impl PartialOrd for TopoName {
    fn partial_cmp(&self, other: &TopoName) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for TopoName {
    fn cmp(&self, other: &TopoName) -> std::cmp::Ordering {
        if Arc::ptr_eq(&self.0, &other.0) {
            return std::cmp::Ordering::Equal;
        }
        self.0.segs.cmp(&other.0.segs)
    }
}
impl fmt::Debug for TopoName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(self.segments()).finish()
    }
}

impl Serialize for TopoName {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        self.segments().serialize(s)
    }
}

impl<'de> Deserialize<'de> for TopoName {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        Vec::<NameSeg>::deserialize(d).map(TopoName::new)
    }
}

/// One segment of a name (note §3.1). `Gen` and `Bound` only come first; events follow in
/// timeline order.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum NameSeg {
    /// Created by `feature` in `role`, optionally from a source.
    Gen {
        /// Creating feature.
        feature: FeatureId,
        /// Role within the feature.
        role: Role,
        /// What it was generated from.
        from: Option<NameSource>,
    },
    /// An edge or vertex named by the sorted multiset of its adjacent face names; `index` of
    /// `count` among the sub-shapes with the same multiset (a hint, never identity).
    Bound {
        /// Adjacent face names, sorted (a seam edge lists its face twice).
        faces: Vec<TopoName>,
        /// Position in the group (geometric order).
        index: u32,
        /// Group size.
        count: u32,
    },
    /// One face became `count` pieces in `feature`; this is piece `index` (a hint).
    Split {
        /// Splitting feature.
        feature: FeatureId,
        /// Piece position (split order).
        index: u32,
        /// Number of pieces.
        count: u32,
    },
    /// This (primary, smallest) name absorbed the faces `with` in `feature`.
    Merge {
        /// Merging feature.
        feature: FeatureId,
        /// The other merged names, sorted.
        with: Vec<TopoName>,
    },
    /// Copy `instance` of a pattern or mirror `feature` (instance 0, the seed, has none).
    Inst {
        /// Copying feature.
        feature: FeatureId,
        /// Instance number (`i + (j << 16)` for rectangular patterns).
        instance: u32,
    },
}

/// Feature-specific role, e.g. `Lateral#0` or `EndCap#0`. Interpreted only together with the
/// creating feature's kind; a shipped tag never changes meaning (note §8.2).
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Role {
    /// Role tag (ASCII identifier, at most 24 characters).
    pub tag: Cow<'static, str>,
    /// Index among entities with the same tag.
    pub index: u32,
}

impl Role {
    /// A role with a static tag.
    pub const fn new(tag: &'static str, index: u32) -> Role {
        Role {
            tag: Cow::Borrowed(tag),
            index,
        }
    }
}

/// What a generated entity came from (note §3.1).
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum NameSource {
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
    /// Outer-loop source ids of a profile region, sorted and deduplicated (note §5.4).
    Profile {
        /// The sketch.
        sketch: SketchId,
        /// Source entity ids on the region's outer loop.
        outer: Vec<SketchEntityKey>,
    },
    /// `TopoRef.key` of a picked input, scoped by the enclosing `Gen.feature` (note §5.1).
    Selection {
        /// The selection key.
        key: u32,
    },
    /// A construction plane, axis or point.
    Construction(ConstructionId),
    /// Another named entity (propagated, not picked).
    Topo(TopoName),
    /// Identity hash of a nested name deeper than [`MAX_NAME_DEPTH`].
    Digest(u64),
}

// ---- identity and events (note §6.2) ----------------------------------------------------------

/// The identity of a name: its origin with nested names reduced to their identities, plus its
/// `Inst` segments in order; `Split`/`Merge` events and `Bound` indices are dropped. Two names
/// with equal identity name the same generated entity, possibly before or after splits and
/// merges.
pub(crate) fn identity(n: &TopoName) -> TopoName {
    let mut out = Vec::with_capacity(2);
    let mut segs = n.segments().iter();
    if let Some(first) = segs.next() {
        out.push(match first {
            NameSeg::Gen {
                feature,
                role,
                from,
            } => NameSeg::Gen {
                feature: *feature,
                role: role.clone(),
                from: from.as_ref().map(|s| match s {
                    NameSource::Topo(t) => NameSource::Topo(identity(t)),
                    other => other.clone(),
                }),
            },
            NameSeg::Bound { faces, .. } => {
                let mut f: Vec<TopoName> = faces.iter().map(identity).collect();
                f.sort();
                NameSeg::Bound {
                    faces: f,
                    index: 0,
                    count: 0,
                }
            }
            other => other.clone(),
        });
    }
    out.extend(segs.filter(|s| matches!(s, NameSeg::Inst { .. })).cloned());
    TopoName::new(out)
}

/// Stable 64-bit hash of a name's identity ([`NameSource::Digest`]).
pub(crate) fn identity_digest(n: &TopoName) -> u64 {
    identity(n).stable_hash()
}

/// A structural event of a name, compared by feature only (indices are wildcards).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum Event {
    Split(FeatureId),
    Merge(FeatureId),
}

pub(crate) fn events(n: &TopoName) -> Vec<Event> {
    n.segments()
        .iter()
        .filter_map(|s| match s {
            NameSeg::Split { feature, .. } => Some(Event::Split(*feature)),
            NameSeg::Merge { feature, .. } => Some(Event::Merge(*feature)),
            _ => None,
        })
        .collect()
}

// ---- fingerprints ---------------------------------------------------------------------------

fn q(x: f64) -> i64 {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "saturating cast is the intent; magnitudes are far below i64::MAX"
    )]
    let i = (x / GeomFingerprint::QUANTUM).round() as i64;
    i
}

/// Geometric fingerprint of a sub-entity (note §3.2).
///
/// `Eq`/`Hash` compare values quantised to [`GeomFingerprint::QUANTUM`] so refs are hashable;
/// resolution never compares fingerprints for equality, it uses [`GeomFingerprint::distance`].
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeomFingerprint {
    /// Surface or curve class.
    pub class: GeomClass,
    /// Area (faces), length (edges) or 0 (vertices).
    pub measure: f64,
    /// Area- or length-weighted centroid; a vertex's point. mm.
    pub centroid: Point3,
    /// Plane normal (oriented), cylinder/cone/torus axis, line direction or circle normal.
    pub axis: Option<Vec3>,
    /// Cylinder/sphere radius, cone reference radius, torus minor radius, circle radius. mm.
    #[serde(default)]
    pub radius: Option<f64>,
    /// Bounding box, mm.
    pub bbox: Aabb3,
}

impl GeomFingerprint {
    /// Quantisation step for `Eq`/`Hash` (mm, mm², unit vectors).
    pub const QUANTUM: f64 = 1e-6;

    fn quantised(&self) -> (GeomClass, Vec<i64>) {
        let mut v = vec![q(self.measure)];
        v.extend(self.centroid.to_array().map(q));
        match self.axis {
            Some(a) => {
                v.push(1);
                v.extend(a.to_array().map(q));
            }
            None => v.push(0),
        }
        match self.radius {
            Some(r) => {
                v.push(1);
                v.push(q(r));
            }
            None => v.push(0),
        }
        v.extend(self.bbox.min.to_array().map(q));
        v.extend(self.bbox.max.to_array().map(q));
        (self.class, v)
    }

    /// The fingerprint of an S02 sub-shape query, with the radius read from the surface or
    /// curve (`Shape::surface` / `Shape::curve`).
    pub fn from_sub_geom(g: &amg_brep::SubShapeGeom, radius: Option<f64>) -> GeomFingerprint {
        GeomFingerprint {
            class: g.class.into(),
            measure: g.measure,
            centroid: g.centroid,
            axis: g.axis,
            radius,
            bbox: g.bbox,
        }
    }

    /// The fingerprint of an S07 mesh face group: class from the fitted primitive (else
    /// `Other`), measure = area, axis = mean normal.
    pub fn from_group(g: &amg_mesh::GroupFingerprint) -> GeomFingerprint {
        use amg_mesh::PrimitiveKind as P;
        let class = match g.primitive {
            Some(P::Plane) => GeomClass::Plane,
            Some(P::Sphere) => GeomClass::Sphere,
            Some(P::Cylinder) => GeomClass::Cylinder,
            Some(P::Cone) => GeomClass::Cone,
            Some(P::Torus) => GeomClass::Torus,
            None => GeomClass::Other,
        };
        GeomFingerprint {
            class,
            measure: g.area,
            centroid: g.centroid,
            axis: (g.mean_normal.length_squared() > 0.0).then_some(g.mean_normal),
            radius: None,
            bbox: g.aabb,
        }
    }

    /// The metric of note §6.3: `None` when the classes differ (never ∞), else
    /// `sqrt(W_C·dc² + W_M·dm² + W_A·da² + W_R·dr²)` with distances scaled by the size of the
    /// referenced entity (`self`).
    pub fn distance(&self, other: &GeomFingerprint, tol: &Tolerance) -> Option<f64> {
        use thresholds::{W_AXIS, W_CENTROID, W_MEASURE, W_RADIUS};
        if self.class != other.class {
            return None;
        }
        let diag = (self.bbox.max - self.bbox.min).length();
        let s = if diag.is_finite() {
            diag.max(10.0 * tol.linear)
        } else {
            10.0 * tol.linear
        };
        let dc = (self.centroid - other.centroid).length() / s;
        let dm = (self.measure - other.measure).abs()
            / self.measure.abs().max(other.measure.abs()).max(1e-12);
        let oriented = self.class == GeomClass::Plane;
        let da = match (self.axis, other.axis) {
            (None, None) => 0.0,
            (Some(a), Some(b)) => {
                let (a, b) = (a.normalize_or_zero(), b.normalize_or_zero());
                let c = a.dot(b);
                let c = if oriented { c } else { c.abs() };
                c.clamp(-1.0, 1.0).acos() / std::f64::consts::FRAC_PI_2
            }
            _ => 1.0,
        };
        let dr = match (self.radius, other.radius) {
            (None, None) => 0.0,
            (Some(a), Some(b)) => (a - b).abs() / s,
            _ => 1.0,
        };
        let d =
            (W_CENTROID * dc * dc + W_MEASURE * dm * dm + W_AXIS * da * da + W_RADIUS * dr * dr)
                .sqrt();
        d.is_finite().then_some(d)
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

impl From<amg_brep::geom::GeomClass> for GeomClass {
    fn from(c: amg_brep::geom::GeomClass) -> GeomClass {
        use amg_brep::geom::GeomClass as B;
        match c {
            B::Plane => GeomClass::Plane,
            B::Cylinder => GeomClass::Cylinder,
            B::Cone => GeomClass::Cone,
            B::Sphere => GeomClass::Sphere,
            B::Torus => GeomClass::Torus,
            B::BSpline => GeomClass::BSpline,
            B::Line => GeomClass::Line,
            B::Circle => GeomClass::Circle,
            B::Ellipse => GeomClass::Ellipse,
            B::Curve => GeomClass::Curve,
            B::Point => GeomClass::Point,
            _ => GeomClass::Other,
        }
    }
}

/// Resolution metric weights and thresholds (note §6.3; part of [`NAMING_VERSION`]).
pub mod thresholds {
    /// Stage 2 picks `best` only if `d_best ≤ PICK_RATIO · d_second`.
    pub const PICK_RATIO: f64 = 0.5;
    /// Above this ratio a pick carries a `LowMargin` warning.
    pub const QUIET_RATIO: f64 = 0.25;
    /// `d_best > FAR` carries a `FarMatch` warning.
    pub const FAR: f64 = 1.0;
    /// Stage 3 accepts only `d_best ≤ FP_ONLY_MAX`.
    pub const FP_ONLY_MAX: f64 = 0.05;
    /// Stage 3 needs `d_second ≥ FP_ONLY_UNIQUE · d_best` (and `d_second > FP_ONLY_MAX`).
    pub const FP_ONLY_UNIQUE: f64 = 4.0;
    /// Weight of the centroid distance.
    pub const W_CENTROID: f64 = 1.0;
    /// Weight of the relative measure difference.
    pub const W_MEASURE: f64 = 0.5;
    /// Weight of the axis angle.
    pub const W_AXIS: f64 = 1.0;
    /// Weight of the radius difference.
    pub const W_RADIUS: f64 = 1.0;
}

// ---- element maps -----------------------------------------------------------------------------

/// Element map of one evaluated body (note §3, §5.3): index = `amg-brep` in-shape index
/// (solids) or `FaceGroupId` (meshes, [`BodyNaming::groups`]).
///
/// The lookup index the resolver uses is built lazily on the first resolution and dropped by
/// `clone`; don't mutate the fields of a `BodyNaming` that has been resolved against (clone it).
#[derive(Clone, Default, Serialize, Deserialize)]
pub struct BodyNaming {
    /// Face names.
    pub faces: Vec<TopoName>,
    /// Edge names.
    pub edges: Vec<TopoName>,
    /// Vertex names.
    pub vertices: Vec<TopoName>,
    /// Mesh face-group names (empty for solids).
    #[serde(default)]
    pub groups: BTreeMap<u32, TopoName>,
    /// Tombstones: face (or group) names removed by a feature (note §4.4).
    #[serde(default, with = "removed_serde")]
    pub removed: Arc<BTreeMap<TopoName, FeatureId>>,
    #[serde(skip)]
    index: IndexCell,
}

/// Lazily built resolver index (not part of equality or serde).
#[derive(Default)]
struct IndexCell(OnceLock<Arc<resolve::NamingIndex>>);

impl Clone for IndexCell {
    fn clone(&self) -> IndexCell {
        IndexCell::default()
    }
}

mod removed_serde {
    use std::collections::BTreeMap;
    use std::sync::Arc;

    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    use super::TopoName;
    use crate::id::FeatureId;

    #[allow(clippy::ref_option, reason = "serde `with` signature")]
    pub(super) fn serialize<S: Serializer>(
        m: &Arc<BTreeMap<TopoName, FeatureId>>,
        s: S,
    ) -> Result<S::Ok, S::Error> {
        let v: Vec<(&TopoName, &FeatureId)> = m.iter().collect();
        v.serialize(s)
    }

    pub(super) fn deserialize<'de, D: Deserializer<'de>>(
        d: D,
    ) -> Result<Arc<BTreeMap<TopoName, FeatureId>>, D::Error> {
        let v = Vec::<(TopoName, FeatureId)>::deserialize(d)?;
        Ok(Arc::new(v.into_iter().collect()))
    }
}

impl PartialEq for BodyNaming {
    fn eq(&self, other: &BodyNaming) -> bool {
        self.faces == other.faces
            && self.edges == other.edges
            && self.vertices == other.vertices
            && self.groups == other.groups
            && self.removed == other.removed
    }
}
impl Eq for BodyNaming {}

impl fmt::Debug for BodyNaming {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BodyNaming")
            .field("faces", &self.faces)
            .field("edges", &self.edges)
            .field("vertices", &self.vertices)
            .field("groups", &self.groups)
            .field("removed", &self.removed)
            .finish()
    }
}

impl BodyNaming {
    /// An element map of a solid from its face, edge and vertex names (no tombstones).
    pub fn new(faces: Vec<TopoName>, edges: Vec<TopoName>, vertices: Vec<TopoName>) -> Self {
        BodyNaming {
            faces,
            edges,
            vertices,
            ..BodyNaming::default()
        }
    }

    /// The names of one kind (`None` for face groups, which are keyed by id).
    pub fn list(&self, kind: TopoKind) -> Option<&[TopoName]> {
        match kind {
            TopoKind::Face => Some(&self.faces),
            TopoKind::Edge => Some(&self.edges),
            TopoKind::Vertex => Some(&self.vertices),
            TopoKind::FaceGroup => None,
        }
    }

    /// The name of sub-entity `index` of `kind` (a group id for face groups).
    pub fn name_of(&self, kind: TopoKind, index: u32) -> Option<&TopoName> {
        match kind {
            TopoKind::FaceGroup => self.groups.get(&index),
            _ => self.list(kind)?.get(index as usize),
        }
    }

    /// Estimated heap bytes of this element map: every name allocation counted once (shared
    /// names by pointer), plus the vectors. With `base`, allocations shared with `base` are not
    /// counted (the cost of an op whose result reuses its input's names, note §9).
    pub fn estimated_heap_bytes(&self, base: Option<&BodyNaming>) -> u64 {
        use std::collections::HashSet;
        fn walk(n: &TopoName, seen: &mut HashSet<usize>, base: &HashSet<usize>) -> u64 {
            let p = Arc::as_ptr(&n.0) as usize;
            if base.contains(&p) || !seen.insert(p) {
                return 0;
            }
            // Arc header (2 × 8) + hash + depth (padded) + boxed slice.
            let mut b = 16 + 8 + 8 + 16 + size_of_val(&*n.0.segs) as u64;
            for s in n.segments() {
                let nested: &[TopoName] = match s {
                    NameSeg::Bound { faces, .. } => faces,
                    NameSeg::Merge { with, .. } => with,
                    NameSeg::Gen {
                        from: Some(NameSource::Topo(t)),
                        ..
                    } => std::slice::from_ref(t),
                    _ => &[],
                };
                if matches!(s, NameSeg::Bound { .. } | NameSeg::Merge { .. }) {
                    b += 8 * nested.len() as u64;
                }
                for t in nested {
                    b += walk(t, seen, base);
                }
            }
            b
        }
        let mut base_ptrs = HashSet::new();
        if let Some(bn) = base {
            let mut seen = HashSet::new();
            for n in bn.faces.iter().chain(&bn.edges).chain(&bn.vertices) {
                walk(n, &mut seen, &HashSet::new());
            }
            base_ptrs = seen;
        }
        let mut seen = HashSet::new();
        let lists = 8 * (self.faces.len() + self.edges.len() + self.vertices.len()) as u64;
        lists
            + self
                .faces
                .iter()
                .chain(&self.edges)
                .chain(&self.vertices)
                .chain(self.groups.values())
                .map(|n| walk(n, &mut seen, &base_ptrs))
                .sum::<u64>()
    }

    pub(crate) fn index(&self) -> &resolve::NamingIndex {
        self.index
            .0
            .get_or_init(|| Arc::new(resolve::NamingIndex::build(self)))
    }

    /// The element map of a rigid copy `instance` of this body by `feature` (pattern, mirror
    /// through history, copy; note §4.6): `Inst { feature, instance }` is appended to every face
    /// (and group) name and the edge and vertex names are re-derived from them. `shape` is the
    /// copy; its face/edge/vertex counts must match (a proper rigid transform keeps the
    /// sub-shape order, S02 Q1).
    ///
    /// # Errors
    /// [`NamingError::CountMismatch`] when the counts differ.
    pub fn instanced(
        &self,
        feature: FeatureId,
        instance: u32,
        shape: &amg_brep::Shape,
    ) -> Result<BodyNaming, NamingError> {
        use amg_brep::SubKind;
        let counts = [
            (TopoKind::Face, self.faces.len(), shape.count(SubKind::Face)),
            (TopoKind::Edge, self.edges.len(), shape.count(SubKind::Edge)),
            (
                TopoKind::Vertex,
                self.vertices.len(),
                shape.count(SubKind::Vertex),
            ),
        ];
        for (kind, naming, shape) in counts {
            if naming != shape as usize {
                return Err(NamingError::CountMismatch {
                    kind,
                    naming: u32::try_from(naming).unwrap_or(u32::MAX),
                    shape,
                });
            }
        }
        Ok(self.instanced_names(feature, instance))
    }

    /// [`BodyNaming::instanced`] without the count check (synthetic and mesh bodies).
    #[must_use]
    pub fn instanced_names(&self, feature: FeatureId, instance: u32) -> BodyNaming {
        let inst = NameSeg::Inst { feature, instance };
        let faces: Vec<TopoName> = self.faces.iter().map(|n| n.with(inst.clone())).collect();
        let map: std::collections::HashMap<&TopoName, &TopoName> =
            self.faces.iter().zip(&faces).collect();
        let rename = |n: &TopoName| -> TopoName {
            match n.segments() {
                [
                    NameSeg::Bound {
                        faces: fs,
                        index,
                        count,
                    },
                    rest @ ..,
                ] => {
                    let mut nf: Vec<TopoName> = fs
                        .iter()
                        .map(|f| {
                            map.get(f)
                                .map_or_else(|| f.with(inst.clone()), |x| (*x).clone())
                        })
                        .collect();
                    nf.sort();
                    let mut segs = vec![NameSeg::Bound {
                        faces: nf,
                        index: *index,
                        count: *count,
                    }];
                    segs.extend(rest.iter().cloned());
                    TopoName::new(segs)
                }
                _ => n.with(inst.clone()),
            }
        };
        BodyNaming {
            edges: self.edges.iter().map(rename).collect(),
            vertices: self.vertices.iter().map(rename).collect(),
            groups: self
                .groups
                .iter()
                .map(|(k, n)| (*k, n.with(inst.clone())))
                .collect(),
            removed: Arc::new(
                self.removed
                    .iter()
                    .map(|(n, f)| (n.with(inst.clone()), *f))
                    .collect(),
            ),
            faces,
            index: IndexCell::default(),
        }
    }

    /// Index naming of an imported or baked body (note §4.6): face `i` is
    /// `Gen { feature, Imported#i, None }`, edges and vertices derived from them (group order
    /// by index; the shape is frozen, so the order is stable).
    ///
    /// # Errors
    /// [`NamingError::Kernel`] if a topology query fails.
    pub fn for_import(
        feature: FeatureId,
        shape: &amg_brep::Shape,
    ) -> Result<BodyNaming, NamingError> {
        let topo = ShapeTopo::new(shape, None);
        let nf = topo.counts()[0];
        let faces: Vec<TopoName> = (0..nf)
            .map(|i| {
                TopoName::new(vec![NameSeg::Gen {
                    feature,
                    role: Role {
                        tag: Cow::Borrowed("Imported"),
                        index: i,
                    },
                    from: None,
                }])
            })
            .collect();
        let (edges, vertices) =
            topo::derive_edges_and_vertices(&topo, feature, &faces, &|_, _| None)?;
        Ok(BodyNaming::new(faces, edges, vertices))
    }

    /// Names of the face groups of a mesh body (note §5.3). A group whose source B-rep face is
    /// known (`from`: the solid's naming and a map from group id to its source face index,
    /// `solid_to_mesh`) takes that face's name; every other group is
    /// `Gen { feature, Group#<id>, None }`.
    pub fn for_mesh(
        feature: FeatureId,
        groups: &amg_mesh::FaceGroups,
        from: Option<MeshSource<'_>>,
    ) -> BodyNaming {
        let mut out = BTreeMap::new();
        for g in groups.ids() {
            let name = from
                .and_then(|(naming, map)| map(g.0).and_then(|f| naming.faces.get(f as usize)))
                .cloned()
                .unwrap_or_else(|| {
                    TopoName::new(vec![NameSeg::Gen {
                        feature,
                        role: Role {
                            tag: Cow::Borrowed("Group"),
                            index: g.0,
                        },
                        from: None,
                    }])
                });
            out.insert(g.0, name);
        }
        BodyNaming {
            groups: out,
            ..BodyNaming::default()
        }
    }
}

/// The B-rep naming a tessellated mesh's groups come from, and the source face index of each
/// group id ([`BodyNaming::for_mesh`]).
pub type MeshSource<'a> = (&'a BodyNaming, &'a dyn Fn(u32) -> Option<u32>);

// ---- resolution results -----------------------------------------------------------------------

/// How confident a resolution is (note §6.4).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MatchQuality {
    /// Name and lineage match exactly.
    Exact,
    /// Several candidates (or a wildcard match); the fingerprint picked one.
    Disambiguated,
    /// The match is larger than what was referenced (a split went away, or a merge absorbed
    /// it).
    Widened,
    /// Only the fingerprint matched (always a warning).
    FingerprintOnly,
}

/// Why a successful resolution needs the user's attention (note §6.4 precedence order).
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ResolveWarning {
    /// The reference predates a split of the entity; `pieces` candidates were split pieces.
    NewSplit {
        /// The splitting feature (`None` when faces gained or lost shared edges).
        feature: Option<FeatureId>,
        /// Number of pieces among the candidates.
        pieces: u32,
    },
    /// The reference was a merge that no longer happens.
    Unmerged,
    /// The match is larger than the reference.
    Widened {
        /// Why.
        cause: WidenCause,
    },
    /// Split pieces changed order; matched by position, not by index.
    IndexDrift,
    /// Some candidates changed geometric class.
    ClassChanged,
    /// The best candidate wins with a small margin.
    LowMargin {
        /// `d_best / d_second`.
        ratio: f32,
    },
    /// The match is far from the reference's fingerprint.
    FarMatch {
        /// `d_best`.
        distance: f32,
    },
    /// The entity now lives in another body (split off).
    BodyChanged {
        /// The body it was found in.
        now_in: BodyId,
    },
    /// Only the fingerprint matched.
    FingerprintOnly {
        /// The fingerprint distance.
        distance: f32,
    },
    /// Several candidates and no fingerprint; the name-equal one was taken.
    NoFingerprint,
}

/// Cause of a [`ResolveWarning::Widened`].
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum WidenCause {
    /// The referenced face was a piece of a split by `feature` that no longer happens.
    SplitGone {
        /// The former splitting feature.
        feature: FeatureId,
    },
    /// The referenced face was merged with neighbours by `feature`.
    Merged {
        /// The merging feature.
        feature: FeatureId,
    },
}

/// A resolved reference.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Resolved {
    /// The body (may differ from the reference's, through body lineage).
    pub body: BodyId,
    /// Kind of sub-entity.
    pub kind: TopoKind,
    /// In-shape index (group id for face groups).
    pub index: u32,
    /// How confident the match is.
    pub quality: MatchQuality,
    /// What the user should check, if anything.
    pub warning: Option<ResolveWarning>,
}

/// Why a reference did not resolve.
#[derive(thiserror::Error, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ResolveError {
    /// The body does not exist at this point in the timeline.
    #[error("body {0:?} does not exist at this point in the timeline")]
    BodyMissing(BodyId),
    /// The entity is gone.
    #[error("referenced {kind:?} no longer exists")]
    NotFound {
        /// Kind of the referenced entity.
        kind: TopoKind,
        /// The feature that removed it, when known (tombstone).
        removed_by: Option<FeatureId>,
    },
    /// Several candidates and no confident pick.
    #[error("reference is ambiguous ({} candidates)", candidates.len())]
    Ambiguous {
        /// The candidates `(body, index)`.
        candidates: Vec<(BodyId, u32)>,
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

/// How a set-valued reference (fillet edges, shell faces) treats members that fail (note §5.1).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum SetPolicy {
    /// Any failing member fails the whole set.
    #[default]
    AllOrError,
    /// Lost or ambiguous members are dropped with a warning; members whose candidates are all
    /// new split pieces expand to every piece. Fails only if no member resolves.
    DropMissing,
}

/// One resolution done by a feature, kept in `NodeEval::resolutions` (UI re-pick, Accept match,
/// the corpus harness). A set member expanded to all its split pieces has several `resolved` /
/// `current` entries; a dropped member keeps its error.
#[derive(Clone, Debug, PartialEq)]
pub struct ResolutionRecord {
    /// The reference as stored.
    pub r: EntityRef,
    /// The outcome (more than one entry only for expanded set members).
    pub result: Result<Vec<Resolved>, ResolveError>,
    /// References to the chosen entities at the feature's input state, `key` kept (Accept
    /// match).
    pub current: Vec<EntityRef>,
}

impl ResolutionRecord {
    /// The first resolved entity, if any.
    pub fn first(&self) -> Option<&Resolved> {
        self.result.as_ref().ok().and_then(|v| v.first())
    }

    /// The warning of the first resolved entity, if any.
    pub fn warning(&self) -> Option<ResolveWarning> {
        self.first().and_then(|r| r.warning)
    }
}

#[cfg(test)]
mod tests;
