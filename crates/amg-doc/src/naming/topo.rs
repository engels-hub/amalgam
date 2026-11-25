//! The topology queries naming needs (adjacency, fingerprints of grouped sub-shapes), and the
//! derivation of edge and vertex names from face names (note §4.3).

use std::collections::{BTreeMap, HashMap};

use amg_brep::{BrepKernel, EdgeId, FaceId, SubKind, SubShape, VertexId};

use super::{GeomFingerprint, NameSeg, NamingError, Role, TopoKind, TopoName};
use crate::id::FeatureId;

/// What [`NamingBuilder::finish_topo`](super::NamingBuilder::finish_topo) and the edge
/// derivation read from a shape. [`ShapeTopo`] implements it over `amg-brep`;
/// [`SyntheticTopology`] over plain tables (unit tests and benches without OCCT).
pub trait NamingTopology {
    /// Faces, edges and vertices.
    fn counts(&self) -> [u32; 3];
    /// The faces of edge `e`, ascending, a seam edge's face listed twice.
    fn edge_faces(&self, e: u32, out: &mut Vec<u32>);
    /// The edges meeting at vertex `v`.
    fn vertex_edges(&self, v: u32, out: &mut Vec<u32>);
    /// The fingerprint of one sub-shape, `None` when it can't be computed (no kernel). Called
    /// only for sub-shapes whose order must be decided (split pieces, dedup and `Bound` groups).
    fn fingerprint(&self, kind: TopoKind, index: u32) -> Option<GeomFingerprint>;
}

/// [`NamingTopology`] of an `amg-brep` shape; fingerprints through the kernel (none without
/// one: groups are then ordered by index).
#[derive(Clone, Copy)]
pub struct ShapeTopo<'a> {
    shape: &'a amg_brep::Shape,
    kernel: Option<&'a dyn BrepKernel>,
}

impl std::fmt::Debug for ShapeTopo<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ShapeTopo")
            .field("kernel", &self.kernel.is_some())
            .finish_non_exhaustive()
    }
}

impl<'a> ShapeTopo<'a> {
    /// Topology of `shape`.
    pub fn new(shape: &'a amg_brep::Shape, kernel: Option<&'a dyn BrepKernel>) -> ShapeTopo<'a> {
        ShapeTopo { shape, kernel }
    }
}

/// The radius of a sub-shape's surface or curve, for [`GeomFingerprint::radius`]: cylinder and
/// sphere radius, cone reference radius, torus minor radius, circle radius.
pub(crate) fn radius_of(shape: &amg_brep::Shape, sub: SubShape) -> Option<f64> {
    use amg_brep::geom::{Curve3, Surface};
    match sub.kind {
        SubKind::Face => match shape.surface(FaceId(sub.index)) {
            Surface::Cylinder { radius, .. } | Surface::Sphere { radius, .. } => Some(radius),
            Surface::Cone { ref_radius, .. } => Some(ref_radius),
            Surface::Torus { minor, .. } => Some(minor),
            _ => None,
        },
        SubKind::Edge => match shape.curve(EdgeId(sub.index)) {
            Some((Curve3::Circle { radius, .. }, _, _)) => Some(radius),
            _ => None,
        },
        SubKind::Vertex => None,
    }
}

/// The fingerprint of `sub` of `shape` through `kernel`.
pub(crate) fn shape_fingerprint(
    kernel: &dyn BrepKernel,
    shape: &amg_brep::Shape,
    sub: SubShape,
) -> Option<GeomFingerprint> {
    let g = kernel.sub_geom(shape, sub).ok()?;
    Some(GeomFingerprint::from_sub_geom(&g, radius_of(shape, sub)))
}

impl NamingTopology for ShapeTopo<'_> {
    fn counts(&self) -> [u32; 3] {
        [
            self.shape.count(SubKind::Face),
            self.shape.count(SubKind::Edge),
            self.shape.count(SubKind::Vertex),
        ]
    }

    fn edge_faces(&self, e: u32, out: &mut Vec<u32>) {
        let faces = self.shape.edge_faces(EdgeId(e));
        for f in faces {
            out.push(f.0);
            if faces.len() == 1 && self.shape.is_seam(EdgeId(e), *f) {
                out.push(f.0);
            }
        }
    }

    fn vertex_edges(&self, v: u32, out: &mut Vec<u32>) {
        out.extend(self.shape.vertex_edges(VertexId(v)).iter().map(|e| e.0));
    }

    fn fingerprint(&self, kind: TopoKind, index: u32) -> Option<GeomFingerprint> {
        let sub = SubShape {
            kind: kind.sub_kind()?,
            index,
        };
        shape_fingerprint(self.kernel?, self.shape, sub)
    }
}

/// A topology given as tables (tests, benches, and callers without a B-rep shape).
#[derive(Clone, Debug, Default)]
pub struct SyntheticTopology {
    /// Number of faces.
    pub faces: u32,
    /// Faces of each edge (a seam face twice).
    pub edge_faces: Vec<Vec<u32>>,
    /// Edges of each vertex.
    pub vertex_edges: Vec<Vec<u32>>,
    /// Fingerprints of the sub-shapes that have one.
    pub fingerprints: HashMap<(TopoKind, u32), GeomFingerprint>,
}

impl NamingTopology for SyntheticTopology {
    fn counts(&self) -> [u32; 3] {
        let n = |l: usize| u32::try_from(l).unwrap_or(u32::MAX);
        [
            self.faces,
            n(self.edge_faces.len()),
            n(self.vertex_edges.len()),
        ]
    }

    fn edge_faces(&self, e: u32, out: &mut Vec<u32>) {
        if let Some(f) = self.edge_faces.get(e as usize) {
            out.extend_from_slice(f);
        }
    }

    fn vertex_edges(&self, v: u32, out: &mut Vec<u32>) {
        if let Some(e) = self.vertex_edges.get(v as usize) {
            out.extend_from_slice(e);
        }
    }

    fn fingerprint(&self, kind: TopoKind, index: u32) -> Option<GeomFingerprint> {
        self.fingerprints.get(&(kind, index)).cloned()
    }
}

/// The total sort key of note §4.3: `(−q(measure), q(cx), q(cy), q(cz), index)`, quantised to
/// `amg_brep::SPLIT_QUANTUM`, so the order is total (S02 Q5 uses the same key). Without a
/// fingerprint only the index decides.
pub(crate) fn order_key(fp: Option<&GeomFingerprint>, index: u32) -> (i64, i64, i64, i64, u32) {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "saturating cast; no part is 9e12 mm across"
    )]
    let q = |x: f64| (x / amg_brep::SPLIT_QUANTUM).round() as i64;
    match fp {
        Some(f) => (
            -q(f.measure),
            q(f.centroid.x),
            q(f.centroid.y),
            q(f.centroid.z),
            index,
        ),
        None => (0, 0, 0, 0, index),
    }
}

/// Sorts `members` (sub-shapes of `kind`) into the geometric order, fetching only their
/// fingerprints.
pub(crate) fn sort_geometric(topo: &dyn NamingTopology, kind: TopoKind, members: &mut [u32]) {
    if members.len() < 2 {
        return;
    }
    let mut keyed: Vec<_> = members
        .iter()
        .map(|&i| (order_key(topo.fingerprint(kind, i).as_ref(), i), i))
        .collect();
    keyed.sort_unstable();
    for (slot, (_, i)) in members.iter_mut().zip(keyed) {
        *slot = i;
    }
}

/// `count` names equal to `base` get `Split { feature, i, count }` in the order of `members`.
pub(crate) fn dedup_names(
    names: &mut [TopoName],
    topo: &dyn NamingTopology,
    kind: TopoKind,
    feature: FeatureId,
) {
    let mut groups: HashMap<TopoName, Vec<u32>> = HashMap::new();
    for (i, n) in names.iter().enumerate() {
        groups
            .entry(n.clone())
            .or_default()
            .push(u32::try_from(i).unwrap_or(u32::MAX));
    }
    let mut dups: Vec<Vec<u32>> = groups.into_values().filter(|g| g.len() > 1).collect();
    dups.sort_unstable();
    for mut g in dups {
        sort_geometric(topo, kind, &mut g);
        let count = u32::try_from(g.len()).unwrap_or(u32::MAX);
        for (index, &m) in (0u32..).zip(&g) {
            let n = &mut names[m as usize];
            *n = n.with(NameSeg::Split {
                feature,
                index,
                count,
            });
        }
    }
}

/// Edge and vertex names derived from `faces` (note §4.3).
///
/// - An edge with adjacent faces is `Bound { faces: sorted multiset of their names, index,
///   count }`; `index`/`count` order the edges with the same multiset geometrically. A vertex
///   likewise with the set of faces of its edges.
/// - An edge or vertex without faces (wire bodies) gets `prior(kind, i)` (its history name) or
///   `Gen { feature, Edge#0 | Vertex#0, None }`, deduplicated.
/// - When the derived name equals `prior(kind, i)` (an unchanged sub-shape), the prior `Arc` is
///   reused.
///
/// # Errors
/// [`NamingError::CountMismatch`] when an adjacency refers to a face beyond `faces`.
pub(crate) fn derive_edges_and_vertices(
    topo: &dyn NamingTopology,
    feature: FeatureId,
    faces: &[TopoName],
    prior: &dyn Fn(TopoKind, u32) -> Option<TopoName>,
) -> Result<(Vec<TopoName>, Vec<TopoName>), NamingError> {
    let [nf, ne, nv] = topo.counts();
    if faces.len() != nf as usize {
        return Err(NamingError::CountMismatch {
            kind: TopoKind::Face,
            naming: u32::try_from(faces.len()).unwrap_or(u32::MAX),
            shape: nf,
        });
    }
    let mut buf = Vec::new();
    let mut edge_keys: Vec<Vec<u32>> = Vec::with_capacity(ne as usize);
    for e in 0..ne {
        buf.clear();
        topo.edge_faces(e, &mut buf);
        if let Some(&bad) = buf.iter().find(|&&f| f >= nf) {
            return Err(NamingError::CountMismatch {
                kind: TopoKind::Face,
                naming: nf,
                shape: bad + 1,
            });
        }
        let mut k = buf.clone();
        k.sort_unstable();
        edge_keys.push(k);
    }
    let edges = name_groups(topo, TopoKind::Edge, feature, faces, &edge_keys, prior);

    let mut edge_buf = Vec::new();
    let mut vertex_keys: Vec<Vec<u32>> = Vec::with_capacity(nv as usize);
    for v in 0..nv {
        edge_buf.clear();
        topo.vertex_edges(v, &mut edge_buf);
        let mut k: Vec<u32> = edge_buf
            .iter()
            .filter_map(|&e| edge_keys.get(e as usize))
            .flatten()
            .copied()
            .collect();
        k.sort_unstable();
        k.dedup();
        vertex_keys.push(k);
    }
    let vertices = name_groups(topo, TopoKind::Vertex, feature, faces, &vertex_keys, prior);
    Ok((edges, vertices))
}

fn name_groups(
    topo: &dyn NamingTopology,
    kind: TopoKind,
    feature: FeatureId,
    faces: &[TopoName],
    keys: &[Vec<u32>],
    prior: &dyn Fn(TopoKind, u32) -> Option<TopoName>,
) -> Vec<TopoName> {
    // Group on integer keys; only then turn them into names (note §4.3).
    let mut groups: HashMap<&[u32], Vec<u32>> = HashMap::new();
    let mut loose = Vec::new();
    for (i, k) in (0u32..).zip(keys) {
        if k.is_empty() {
            loose.push(i);
        } else {
            groups.entry(k.as_slice()).or_default().push(i);
        }
    }
    let mut out: Vec<Option<TopoName>> = vec![None; keys.len()];
    let mut names: Vec<TopoName> = Vec::new();
    for (k, mut members) in groups {
        sort_geometric(topo, kind, &mut members);
        names.clear();
        names.extend(k.iter().map(|&f| faces[f as usize].clone()));
        names.sort();
        let count = u32::try_from(members.len()).unwrap_or(u32::MAX);
        for (index, &m) in (0u32..).zip(&members) {
            // An unchanged sub-shape reuses its input name without building a new one.
            let name = match prior(kind, m) {
                Some(p) if is_bound(&p, &names, index, count) => p,
                _ => TopoName::new(vec![NameSeg::Bound {
                    faces: names.clone(),
                    index,
                    count,
                }]),
            };
            out[m as usize] = Some(name);
        }
    }
    if !loose.is_empty() {
        let tag = if kind == TopoKind::Vertex {
            "Vertex"
        } else {
            "Edge"
        };
        let mut names: Vec<TopoName> = loose
            .iter()
            .map(|&i| {
                prior(kind, i).unwrap_or_else(|| {
                    TopoName::new(vec![NameSeg::Gen {
                        feature,
                        role: Role::new(tag, 0),
                        from: None,
                    }])
                })
            })
            .collect();
        // Dedup within the loose set only (Bound names can't collide with them).
        let sub = Subset {
            topo,
            members: &loose,
        };
        dedup_names(&mut names, &sub, kind, feature);
        for (i, n) in loose.iter().zip(names) {
            out[*i as usize] = Some(n);
        }
    }
    out.into_iter().map(Option::unwrap_or_default).collect()
}

/// `n` is exactly `Bound { faces, index, count }`.
fn is_bound(n: &TopoName, faces: &[TopoName], index: u32, count: u32) -> bool {
    matches!(
        n.segments(),
        [NameSeg::Bound { faces: f, index: i, count: c }]
            if *i == index && *c == count && f.as_slice() == faces
    )
}

/// A view of `members` of a topology as indices `0..members.len()` (for dedup of a subset).
struct Subset<'a> {
    topo: &'a dyn NamingTopology,
    members: &'a [u32],
}

impl NamingTopology for Subset<'_> {
    fn counts(&self) -> [u32; 3] {
        self.topo.counts()
    }
    fn edge_faces(&self, e: u32, out: &mut Vec<u32>) {
        self.topo.edge_faces(e, out);
    }
    fn vertex_edges(&self, v: u32, out: &mut Vec<u32>) {
        self.topo.vertex_edges(v, out);
    }
    fn fingerprint(&self, kind: TopoKind, index: u32) -> Option<GeomFingerprint> {
        let real = *self.members.get(index as usize)?;
        self.topo.fingerprint(kind, real)
    }
}

impl Default for TopoName {
    fn default() -> TopoName {
        TopoName::new(Vec::new())
    }
}

/// Faces next to each face (through shared edges), for the role-only rule (note §4.2 rule 4).
pub(crate) fn face_neighbours(topo: &dyn NamingTopology) -> BTreeMap<u32, Vec<u32>> {
    let [_, ne, _] = topo.counts();
    let mut out: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    let mut buf = Vec::new();
    for e in 0..ne {
        buf.clear();
        topo.edge_faces(e, &mut buf);
        for &a in &buf {
            for &b in &buf {
                if a != b {
                    out.entry(a).or_default().push(b);
                }
            }
        }
    }
    for v in out.values_mut() {
        v.sort_unstable();
        v.dedup();
    }
    out
}
