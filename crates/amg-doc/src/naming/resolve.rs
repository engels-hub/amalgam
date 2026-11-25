//! The resolver (note §6): candidate sets by lineage, the fingerprint ratio test, tombstones,
//! the fingerprint-only fallback, and the warnings and errors of every outcome.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::{Arc, Mutex, OnceLock, PoisonError};

use amg_brep::{BrepKernel, SubShape};
use amg_core::Tolerance;

use super::thresholds::{FAR, FP_ONLY_MAX, FP_ONLY_UNIQUE, PICK_RATIO, QUIET_RATIO};
use super::topo::shape_fingerprint;
use super::{
    BodyNaming, EntityRef, Event, GeomFingerprint, MatchQuality, NameSeg, ResolutionRecord,
    ResolveError, ResolveWarning, Resolved, SetPolicy, TopoKind, TopoName, TopoRef, WidenCause,
    events, identity,
};
use crate::eval::BodyGeom;
use crate::id::{BodyId, FeatureId};

/// Body lookup used by the resolver: the input state of a feature, or the final evaluated
/// state ([`crate::Evaluated`] implements it).
pub trait NamingView {
    /// Geometry of a body that exists in this state.
    fn body_geom(&self, id: BodyId) -> Option<&BodyGeom>;
    /// Element map of a body, if it has one.
    fn naming(&self, id: BodyId) -> Option<&BodyNaming>;
    /// Bodies that `id` was consumed into or split into (`BodyChangeKind::Consumed` /
    /// `CreatedFrom`), ascending.
    fn successors(&self, id: BodyId) -> &[BodyId];
    /// The body `id` was split off from (`CreatedFrom`), if any.
    fn derived_from(&self, _id: BodyId) -> Option<BodyId> {
        None
    }
    /// Timeline position of a feature (tombstone reports prefer the latest one); `None` falls
    /// back to id order.
    fn feature_order(&self, _f: FeatureId) -> Option<usize> {
        None
    }
}

/// Fingerprints of sub-entities, for the ratio test and the fingerprint-only scan.
pub trait GeomSource {
    /// The fingerprint of one sub-entity, `None` if it can't be computed.
    fn fingerprint(&self, body: BodyId, kind: TopoKind, index: u32) -> Option<GeomFingerprint>;
    /// Every sub-entity of `kind` with a fingerprint (stage 3).
    fn all(&self, body: BodyId, kind: TopoKind) -> Vec<(u32, GeomFingerprint)>;
}

/// The process-wide kernel the engine and [`crate::Evaluated::resolve`] use when no other is
/// given (`OcctKernel` with the default configuration; without the `occt` feature every query
/// fails, so references resolve by name only).
pub fn default_kernel() -> &'static amg_brep::OcctKernel {
    static KERNEL: OnceLock<amg_brep::OcctKernel> = OnceLock::new();
    KERNEL.get_or_init(|| amg_brep::OcctKernel::new(amg_brep::KernelConfig::default()))
}

type AllCache = HashMap<(BodyId, TopoKind), Arc<Vec<(u32, GeomFingerprint)>>>;

/// [`GeomSource`] over the bodies of a [`NamingView`]: B-rep fingerprints through a kernel,
/// mesh group fingerprints from the mesh. Caches the stage-3 scans.
pub struct KernelGeom<'a> {
    view: &'a dyn NamingView,
    kernel: &'a dyn BrepKernel,
    all: Mutex<AllCache>,
}

impl std::fmt::Debug for KernelGeom<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KernelGeom").finish_non_exhaustive()
    }
}

impl<'a> KernelGeom<'a> {
    /// Fingerprints of `view`'s bodies through `kernel`.
    pub fn new(view: &'a dyn NamingView, kernel: &'a dyn BrepKernel) -> KernelGeom<'a> {
        KernelGeom {
            view,
            kernel,
            all: Mutex::new(HashMap::new()),
        }
    }
}

impl GeomSource for KernelGeom<'_> {
    fn fingerprint(&self, body: BodyId, kind: TopoKind, index: u32) -> Option<GeomFingerprint> {
        match (self.view.body_geom(body)?, kind) {
            (BodyGeom::Solid(s), k) if k != TopoKind::FaceGroup => {
                let sub = SubShape {
                    kind: k.sub_kind()?,
                    index,
                };
                shape_fingerprint(self.kernel, s.brep()?, sub)
            }
            (BodyGeom::Mesh(m), TopoKind::FaceGroup) => {
                let g = m.groups.fingerprint(m, amg_mesh::FaceGroupId(index))?;
                Some(GeomFingerprint::from_group(&g))
            }
            _ => None,
        }
    }

    fn all(&self, body: BodyId, kind: TopoKind) -> Vec<(u32, GeomFingerprint)> {
        let key = (body, kind);
        if let Some(v) = self
            .all
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get(&key)
        {
            return v.as_ref().clone();
        }
        let v: Vec<(u32, GeomFingerprint)> = match (self.view.body_geom(body), kind) {
            (Some(BodyGeom::Solid(s)), k) if k != TopoKind::FaceGroup => {
                let (Some(shape), Some(sk)) = (s.brep(), k.sub_kind()) else {
                    return Vec::new();
                };
                let Ok(all) = self.kernel.sub_geom_all(shape) else {
                    return Vec::new();
                };
                let slot = match sk {
                    amg_brep::SubKind::Face => 0,
                    amg_brep::SubKind::Edge => 1,
                    amg_brep::SubKind::Vertex => 2,
                };
                (0u32..)
                    .zip(&all[slot])
                    .map(|(i, g)| {
                        let sub = SubShape { kind: sk, index: i };
                        let r = super::topo::radius_of(shape, sub);
                        (i, GeomFingerprint::from_sub_geom(g, r))
                    })
                    .collect()
            }
            (Some(BodyGeom::Mesh(m)), TopoKind::FaceGroup) => m
                .groups
                .ids()
                .filter_map(|g| {
                    let fp = m.groups.fingerprint(m, g)?;
                    Some((g.0, GeomFingerprint::from_group(&fp)))
                })
                .collect(),
            _ => Vec::new(),
        };
        self.all
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(key, Arc::new(v.clone()));
        v
    }
}

// ---- the per-naming index ---------------------------------------------------------------------

/// Names of one kind by identity, with their events and merge members.
#[derive(Default)]
struct NameSet {
    ident: HashMap<TopoName, Vec<u32>>,
    /// Identity of a `Merge.with` member → (entity that absorbed it, merging feature).
    merged: HashMap<TopoName, Vec<(u32, FeatureId)>>,
    events: HashMap<u32, Vec<Event>>,
}

impl NameSet {
    fn add(&mut self, i: u32, n: &TopoName) {
        self.ident.entry(identity(n)).or_default().push(i);
        for s in n.segments() {
            if let NameSeg::Merge { feature, with } = s {
                for w in with {
                    self.merged
                        .entry(identity(w))
                        .or_default()
                        .push((i, *feature));
                }
            }
        }
        self.events.insert(i, events(n));
    }
}

/// Adjacency of derived (`Bound`) names: entity → faces and face → entities.
#[derive(Default)]
struct Bounds {
    faces_of: Vec<Vec<u32>>,
    of_face: Vec<Vec<u32>>,
    /// `Gen`-origin entities (wire edges and vertices).
    loose: NameSet,
}

/// Lookup index of one [`BodyNaming`], built lazily on the first resolution.
pub(crate) struct NamingIndex {
    faces: NameSet,
    edges: Bounds,
    vertices: Bounds,
    groups: NameSet,
    /// (identity, events, removing feature).
    tombstones: Vec<(TopoName, Vec<Event>, FeatureId)>,
}

impl NamingIndex {
    pub(crate) fn build(n: &BodyNaming) -> NamingIndex {
        let mut faces = NameSet::default();
        let mut by_name: HashMap<&TopoName, u32> = HashMap::with_capacity(n.faces.len());
        for (i, f) in (0u32..).zip(&n.faces) {
            faces.add(i, f);
            by_name.insert(f, i);
        }
        let bounds = |list: &[TopoName]| {
            let mut b = Bounds {
                faces_of: Vec::with_capacity(list.len()),
                of_face: vec![Vec::new(); n.faces.len()],
                loose: NameSet::default(),
            };
            for (i, e) in (0u32..).zip(list) {
                match e.origin() {
                    Some(NameSeg::Bound { faces: fs, .. }) => {
                        let ids: Vec<u32> =
                            fs.iter().filter_map(|f| by_name.get(f).copied()).collect();
                        let mut uniq = ids.clone();
                        uniq.sort_unstable();
                        uniq.dedup();
                        for f in uniq {
                            b.of_face[f as usize].push(i);
                        }
                        b.faces_of.push(ids);
                    }
                    _ => {
                        b.faces_of.push(Vec::new());
                        b.loose.add(i, e);
                    }
                }
            }
            b
        };
        let edges = bounds(&n.edges);
        let vertices = bounds(&n.vertices);
        let mut groups = NameSet::default();
        for (g, name) in &n.groups {
            groups.add(*g, name);
        }
        let tombstones = n
            .removed
            .iter()
            .map(|(name, f)| (identity(name), events(name), *f))
            .collect();
        NamingIndex {
            faces,
            edges,
            vertices,
            groups,
            tombstones,
        }
    }
}

// ---- classification (note §6.2) ---------------------------------------------------------------

/// The classes of one candidate against the reference.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Classes {
    new_split: Option<Option<FeatureId>>,
    unmerged: bool,
    widened: Option<WidenCause>,
}

impl Classes {
    fn same(&self) -> bool {
        self.new_split.is_none() && !self.unmerged && self.widened.is_none()
    }

    fn union(&mut self, o: Classes) {
        if self.new_split.is_none() {
            self.new_split = o.new_split;
        }
        self.unmerged |= o.unmerged;
        if self.widened.is_none() {
            self.widened = o.widened;
        }
    }
}

fn counts(ev: &[Event]) -> BTreeMap<Event, usize> {
    let mut m = BTreeMap::new();
    for e in ev {
        *m.entry(*e).or_insert(0) += 1;
    }
    m
}

/// Compares events by feature (indices and counts are wildcards).
fn classify(r: &[Event], f: &[Event]) -> Classes {
    let (rc, fc) = (counts(r), counts(f));
    let n = |m: &BTreeMap<Event, usize>, e: &Event| m.get(e).copied().unwrap_or(0);
    let mut c = Classes::default();
    for e in f {
        if n(&fc, e) > n(&rc, e) {
            match e {
                Event::Split(feat) if c.new_split.is_none() => c.new_split = Some(Some(*feat)),
                Event::Merge(feat) if c.widened.is_none() => {
                    c.widened = Some(WidenCause::Merged { feature: *feat });
                }
                _ => {}
            }
        }
    }
    for e in r {
        if n(&rc, e) > n(&fc, e) {
            match e {
                Event::Split(feat) if c.widened.is_none() => {
                    c.widened = Some(WidenCause::SplitGone { feature: *feat });
                }
                Event::Merge(_) => c.unmerged = true,
                _ => {}
            }
        }
    }
    c
}

/// One events list is a prefix of the other's (tombstone lookup by lineage, note §4.4).
fn prefix_related(a: &[Event], b: &[Event]) -> bool {
    let n = a.len().min(b.len());
    a[..n] == b[..n]
}

/// Candidates for name `r` among the names of `set` (note §6.2 sources (a), (b), (c)).
fn name_candidates(set: &NameSet, r: &TopoName) -> BTreeMap<u32, Classes> {
    let rid = identity(r);
    let rev = events(r);
    let mut out: BTreeMap<u32, Classes> = BTreeMap::new();
    let empty = Vec::new();
    for &f in set.ident.get(&rid).unwrap_or(&empty) {
        let fev = set.events.get(&f).map_or(&[][..], Vec::as_slice);
        out.entry(f).or_default().union(classify(&rev, fev));
    }
    if let Some(ms) = set.merged.get(&rid) {
        for &(f, feature) in ms {
            out.entry(f).or_default().union(Classes {
                widened: Some(WidenCause::Merged { feature }),
                ..Classes::default()
            });
        }
    }
    for s in r.segments() {
        if let NameSeg::Merge { with, .. } = s {
            for w in with {
                for &f in set.ident.get(&identity(w)).unwrap_or(&empty) {
                    out.entry(f).or_default().union(Classes {
                        unmerged: true,
                        ..Classes::default()
                    });
                }
            }
        }
    }
    out
}

/// One-to-one assignment of the reference faces to the candidate's faces; returns the union of
/// the classes of the first assignment found.
fn assign(sets: &[BTreeMap<u32, Classes>], faces: &[u32]) -> Option<Classes> {
    fn go(
        sets: &[BTreeMap<u32, Classes>],
        faces: &[u32],
        used: &mut Vec<bool>,
        acc: Classes,
    ) -> Option<Classes> {
        let Some((first, rest)) = sets.split_first() else {
            return Some(acc);
        };
        for (j, f) in faces.iter().enumerate() {
            if used[j] {
                continue;
            }
            if let Some(c) = first.get(f) {
                used[j] = true;
                let mut a = acc;
                a.union(*c);
                if let Some(done) = go(rest, faces, used, a) {
                    return Some(done);
                }
                used[j] = false;
            }
        }
        None
    }
    if sets.len() != faces.len() {
        return None;
    }
    go(
        sets,
        faces,
        &mut vec![false; faces.len()],
        Classes::default(),
    )
}

/// What a search over one body found.
struct Found {
    cands: Vec<(u32, Classes)>,
    /// For a `Bound` reference: the reference faces without candidates.
    missing_faces: Vec<TopoName>,
}

fn find_in(naming: &BodyNaming, kind: TopoKind, r: &TopoName) -> Found {
    let ix = naming.index();
    let none = Found {
        cands: Vec::new(),
        missing_faces: Vec::new(),
    };
    match kind {
        TopoKind::Face => Found {
            cands: name_candidates(&ix.faces, r).into_iter().collect(),
            missing_faces: Vec::new(),
        },
        TopoKind::FaceGroup => Found {
            cands: name_candidates(&ix.groups, r).into_iter().collect(),
            missing_faces: Vec::new(),
        },
        TopoKind::Edge | TopoKind::Vertex => {
            let (b, list) = if kind == TopoKind::Edge {
                (&ix.edges, &naming.edges)
            } else {
                (&ix.vertices, &naming.vertices)
            };
            let Some(NameSeg::Bound {
                faces: rfaces,
                count: rcount,
                ..
            }) = r.origin()
            else {
                return Found {
                    cands: name_candidates(&b.loose, r).into_iter().collect(),
                    missing_faces: Vec::new(),
                };
            };
            let sets: Vec<BTreeMap<u32, Classes>> = rfaces
                .iter()
                .map(|x| name_candidates(&ix.faces, x))
                .collect();
            let missing: Vec<TopoName> = rfaces
                .iter()
                .zip(&sets)
                .filter(|(_, s)| s.is_empty())
                .map(|(x, _)| x.clone())
                .collect();
            if !missing.is_empty() || sets.is_empty() {
                return Found {
                    missing_faces: missing,
                    ..none
                };
            }
            let mut seen = BTreeSet::new();
            for f in sets[0].keys() {
                if let Some(es) = b.of_face.get(*f as usize) {
                    seen.extend(es.iter().copied());
                }
            }
            let mut cands = Vec::new();
            for e in seen {
                let ef = &b.faces_of[e as usize];
                let Some(mut c) = assign(&sets, ef) else {
                    continue;
                };
                if let Some(NameSeg::Bound { count, .. }) = list[e as usize].origin()
                    && count != rcount
                    && c.new_split.is_none()
                {
                    c.new_split = Some(None);
                }
                cands.push((e, c));
            }
            Found {
                cands,
                missing_faces: Vec::new(),
            }
        }
    }
}

// ---- stages (note §6.4) -----------------------------------------------------------------------

#[derive(Clone, Debug)]
struct Cand {
    body: BodyId,
    index: u32,
    classes: Classes,
    name_equal: bool,
    /// Found in a body split off from the referenced one.
    now_in: Option<BodyId>,
}

/// The result of resolving one reference, with the candidates (set expansion).
struct Outcome {
    result: Result<Resolved, ResolveError>,
    cands: Vec<Cand>,
}

fn kind_ok(geom: &BodyGeom, kind: TopoKind) -> Result<(), ResolveError> {
    match (geom, kind) {
        (BodyGeom::Implicit(_) | BodyGeom::Form(_), _) => Err(ResolveError::Unsupported),
        (BodyGeom::Solid(_), TopoKind::FaceGroup)
        | (BodyGeom::Mesh(_), TopoKind::Face | TopoKind::Edge | TopoKind::Vertex) => {
            Err(ResolveError::NotFound {
                kind,
                removed_by: None,
            })
        }
        _ => Ok(()),
    }
}

fn successors_dfs(view: &dyn NamingView, start: BodyId) -> Vec<BodyId> {
    fn go(view: &dyn NamingView, id: BodyId, seen: &mut BTreeSet<BodyId>, out: &mut Vec<BodyId>) {
        let mut next: Vec<BodyId> = view.successors(id).to_vec();
        next.sort();
        for s in next {
            if seen.insert(s) {
                out.push(s);
                go(view, s, seen, out);
            }
        }
    }
    let mut seen = BTreeSet::from([start]);
    let mut out = Vec::new();
    go(view, start, &mut seen, &mut out);
    out
}

/// Picks the first warning in precedence order.
fn first_warning(ws: impl IntoIterator<Item = Option<ResolveWarning>>) -> Option<ResolveWarning> {
    ws.into_iter().flatten().next()
}

#[allow(
    clippy::cast_possible_truncation,
    reason = "warning payloads are display values"
)]
fn f32_of(x: f64) -> f32 {
    x as f32
}

#[allow(clippy::too_many_lines, reason = "the stages of note §6.4 in order")]
fn resolve_inner(
    view: &dyn NamingView,
    geom: &dyn GeomSource,
    tol: &Tolerance,
    r: &TopoRef,
) -> Outcome {
    let fail = |e: ResolveError| Outcome {
        result: Err(e),
        cands: Vec::new(),
    };
    let kind = r.kind;
    let mut searched: Vec<(BodyId, Option<BodyId>)> = Vec::new();
    let mut cands: Vec<Cand> = Vec::new();
    let mut missing_faces: Vec<TopoName> = Vec::new();
    let mut search = |body: BodyId, now_in: Option<BodyId>, cands: &mut Vec<Cand>| {
        let Some(naming) = view.naming(body) else {
            return;
        };
        let found = find_in(naming, kind, &r.name);
        missing_faces.extend(found.missing_faces);
        for (index, classes) in found.cands {
            let name_equal = naming.name_of(kind, index) == Some(&r.name);
            cands.push(Cand {
                body,
                index,
                classes,
                name_equal,
                now_in,
            });
        }
    };
    let primary = view.body_geom(r.body);
    if let Some(g) = primary {
        if let Err(e) = kind_ok(g, kind) {
            return fail(e);
        }
        searched.push((r.body, None));
        search(r.body, None, &mut cands);
    }
    if cands.is_empty() {
        let succ = successors_dfs(view, r.body);
        if primary.is_none() && succ.is_empty() {
            return fail(ResolveError::BodyMissing(r.body));
        }
        for s in succ {
            let Some(g) = view.body_geom(s) else {
                continue;
            };
            if kind_ok(g, kind).is_err() {
                continue;
            }
            let now_in = view.derived_from(s).map(|_| s);
            searched.push((s, now_in));
            search(s, now_in, &mut cands);
        }
        if searched.is_empty() {
            return fail(ResolveError::BodyMissing(r.body));
        }
    }
    let rfp = r.fingerprint.as_ref();
    let resolved = |c: &Cand, quality: MatchQuality, warning: Option<ResolveWarning>| Resolved {
        body: c.body,
        kind,
        index: c.index,
        quality,
        warning,
    };

    // Stage 1: a unique candidate.
    if let [c] = cands.as_slice() {
        let cl = c.classes;
        let wildcard = !cl.same() || !c.name_equal;
        let (mut ws_class, mut ws_far) = (None, None);
        if wildcard && let Some(p) = rfp {
            match geom
                .fingerprint(c.body, kind, c.index)
                .and_then(|q| p.distance(&q, tol))
            {
                None => ws_class = Some(ResolveWarning::ClassChanged),
                Some(d) if d > FAR => {
                    ws_far = Some(ResolveWarning::FarMatch {
                        distance: f32_of(d),
                    });
                }
                Some(_) => {}
            }
        }
        let warning = first_warning([
            cl.new_split
                .map(|feature| ResolveWarning::NewSplit { feature, pieces: 1 }),
            cl.unmerged.then_some(ResolveWarning::Unmerged),
            cl.widened.map(|cause| ResolveWarning::Widened { cause }),
            ws_class,
            ws_far,
            c.now_in
                .map(|now_in| ResolveWarning::BodyChanged { now_in }),
        ]);
        let quality = if cl.new_split.is_some() || cl.unmerged {
            MatchQuality::Disambiguated
        } else if cl.widened.is_some() {
            MatchQuality::Widened
        } else if c.name_equal && warning.is_none() {
            MatchQuality::Exact
        } else {
            MatchQuality::Disambiguated
        };
        let res = resolved(c, quality, warning);
        return Outcome {
            result: Ok(res),
            cands,
        };
    }

    // Stage 2: several candidates.
    if cands.len() >= 2 {
        let ambiguous = |cands: &[Cand]| ResolveError::Ambiguous {
            candidates: cands.iter().map(|c| (c.body, c.index)).collect(),
        };
        let Some(p) = rfp else {
            let eq: Vec<&Cand> = cands.iter().filter(|c| c.name_equal).collect();
            if let [c] = eq.as_slice()
                && cands.iter().all(|c| c.classes.same())
            {
                let res = resolved(
                    c,
                    MatchQuality::Disambiguated,
                    Some(ResolveWarning::NoFingerprint),
                );
                return Outcome {
                    result: Ok(res),
                    cands,
                };
            }
            let e = ambiguous(&cands);
            return Outcome {
                result: Err(e),
                cands,
            };
        };
        let mut finite: Vec<(f64, &Cand)> = Vec::new();
        let mut dropped = false;
        for c in &cands {
            match geom
                .fingerprint(c.body, kind, c.index)
                .and_then(|q| p.distance(&q, tol))
            {
                Some(d) => finite.push((d, c)),
                None => dropped = true,
            }
        }
        finite.sort_by(|a, b| {
            a.0.total_cmp(&b.0)
                .then((a.1.body, a.1.index).cmp(&(b.1.body, b.1.index)))
        });
        if finite.is_empty() {
            let e = ambiguous(&cands);
            return Outcome {
                result: Err(e),
                cands,
            };
        }
        let ratio = if finite.len() >= 2 {
            let d1 = finite[1].0;
            if d1 <= 1e-12 { 1.0 } else { finite[0].0 / d1 }
        } else {
            0.0
        };
        if finite.len() >= 2 && ratio > PICK_RATIO {
            let e = ambiguous(&cands);
            return Outcome {
                result: Err(e),
                cands,
            };
        }
        let (d_best, best) = finite[0];
        let pieces = cands
            .iter()
            .filter(|c| c.classes.new_split.is_some())
            .count();
        let split_feature = best
            .classes
            .new_split
            .or_else(|| cands.iter().find_map(|c| c.classes.new_split));
        let index_drift = cands.iter().any(|c| c.name_equal) && !best.name_equal;
        let warning = first_warning([
            split_feature.map(|feature| ResolveWarning::NewSplit {
                feature,
                pieces: u32::try_from(pieces).unwrap_or(u32::MAX),
            }),
            cands
                .iter()
                .any(|c| c.classes.unmerged)
                .then_some(ResolveWarning::Unmerged),
            best.classes
                .widened
                .or_else(|| cands.iter().find_map(|c| c.classes.widened))
                .map(|cause| ResolveWarning::Widened { cause }),
            index_drift.then_some(ResolveWarning::IndexDrift),
            (dropped || finite.len() == 1).then_some(ResolveWarning::ClassChanged),
            (ratio > QUIET_RATIO).then_some(ResolveWarning::LowMargin {
                ratio: f32_of(ratio),
            }),
            (d_best > FAR).then_some(ResolveWarning::FarMatch {
                distance: f32_of(d_best),
            }),
            best.now_in
                .map(|now_in| ResolveWarning::BodyChanged { now_in }),
        ]);
        let quality = if best.name_equal && best.classes.same() && !dropped && warning.is_none() {
            MatchQuality::Exact
        } else {
            MatchQuality::Disambiguated
        };
        let res = resolved(best, quality, warning);
        return Outcome {
            result: Ok(res),
            cands,
        };
    }

    // Stage 3: no candidates.
    let not_found = |removed_by| ResolveError::NotFound { kind, removed_by };
    let tombstone_of = |name: &TopoName| -> Option<FeatureId> {
        let id = identity(name);
        let ev = events(name);
        let mut best: Option<FeatureId> = None;
        for (body, _) in &searched {
            let Some(n) = view.naming(*body) else {
                continue;
            };
            for (tid, tev, f) in &n.index().tombstones {
                if *tid == id && prefix_related(tev, &ev) {
                    let later = |a: FeatureId, b: FeatureId| match (
                        view.feature_order(a),
                        view.feature_order(b),
                    ) {
                        (Some(x), Some(y)) => x > y,
                        _ => a > b,
                    };
                    if best.is_none_or(|b| later(*f, b)) {
                        best = Some(*f);
                    }
                }
            }
        }
        best
    };
    match (kind, r.name.origin()) {
        (TopoKind::Face | TopoKind::FaceGroup, _) => {
            if let Some(f) = tombstone_of(&r.name) {
                return fail(not_found(Some(f)));
            }
        }
        (_, Some(NameSeg::Bound { .. })) => {
            if missing_faces.is_empty() {
                // The faces exist but are no longer adjacent: no geometric guess.
                return fail(not_found(None));
            }
            if let Some(f) = missing_faces.iter().find_map(tombstone_of) {
                return fail(not_found(Some(f)));
            }
        }
        _ => {}
    }
    let Some(p) = rfp else {
        return fail(not_found(None));
    };
    let mut scan: Vec<(f64, BodyId, u32, Option<BodyId>)> = Vec::new();
    for (body, now_in) in &searched {
        for (i, q) in geom.all(*body, kind) {
            if let Some(d) = p.distance(&q, tol) {
                scan.push((d, *body, i, *now_in));
            }
        }
    }
    scan.sort_by(|a, b| a.0.total_cmp(&b.0).then((a.1, a.2).cmp(&(b.1, b.2))));
    if let Some(&(d, body, index, now_in)) = scan.first() {
        let unique = scan
            .get(1)
            .is_none_or(|s| s.0 >= FP_ONLY_UNIQUE * d && s.0 > FP_ONLY_MAX);
        if d <= FP_ONLY_MAX && unique {
            let _ = now_in;
            return Outcome {
                result: Ok(Resolved {
                    body,
                    kind,
                    index,
                    quality: MatchQuality::FingerprintOnly,
                    warning: Some(ResolveWarning::FingerprintOnly {
                        distance: f32_of(d),
                    }),
                }),
                cands: Vec::new(),
            };
        }
    }
    fail(not_found(None))
}

fn current_ref(
    view: &dyn NamingView,
    geom: &dyn GeomSource,
    r: &TopoRef,
    x: &Resolved,
) -> EntityRef {
    let name = view
        .naming(x.body)
        .and_then(|n| n.name_of(x.kind, x.index))
        .cloned()
        .unwrap_or_else(|| r.name.clone());
    EntityRef::Topo(TopoRef {
        body: x.body,
        kind: x.kind,
        name,
        fingerprint: geom.fingerprint(x.body, x.kind, x.index),
        key: r.key,
    })
}

/// Resolves `r` against `view` with fingerprints from `kernel` (note §6).
pub fn resolve(
    view: &dyn NamingView,
    kernel: &dyn BrepKernel,
    tol: &Tolerance,
    r: &TopoRef,
) -> ResolutionRecord {
    resolve_with(view, &KernelGeom::new(view, kernel), tol, r)
}

/// [`resolve`] with fingerprints from any [`GeomSource`].
pub fn resolve_with(
    view: &dyn NamingView,
    geom: &dyn GeomSource,
    tol: &Tolerance,
    r: &TopoRef,
) -> ResolutionRecord {
    let out = resolve_inner(view, geom, tol, r);
    let current = match &out.result {
        Ok(x) => vec![current_ref(view, geom, r, x)],
        Err(_) => Vec::new(),
    };
    ResolutionRecord {
        r: EntityRef::Topo(r.clone()),
        result: out.result.map(|x| vec![x]),
        current,
    }
}

/// Resolves a set-valued reference (note §5.1, §6.4): one record per member, and the members'
/// entities (an expanded member contributes all its split pieces, a dropped member nothing).
///
/// # Errors
/// With `AllOrError`, the first member's error (later members are not resolved); with
/// `DropMissing`, the first error when no member resolves.
pub fn resolve_set(
    view: &dyn NamingView,
    geom: &dyn GeomSource,
    tol: &Tolerance,
    refs: &[TopoRef],
    policy: SetPolicy,
) -> (
    Vec<ResolutionRecord>,
    Result<Vec<Vec<Resolved>>, ResolveError>,
) {
    let mut records = Vec::with_capacity(refs.len());
    let mut members = Vec::with_capacity(refs.len());
    let mut first_err = None;
    for r in refs {
        let out = resolve_inner(view, geom, tol, r);
        let result = match out.result {
            Ok(x) => Ok(vec![x]),
            Err(ResolveError::Ambiguous { .. })
                if policy == SetPolicy::DropMissing
                    && !out.cands.is_empty()
                    && out.cands.iter().all(|c| c.classes.new_split.is_some()) =>
            {
                let mut cs = out.cands.clone();
                cs.sort_by_key(|c| (c.body, c.index));
                let pieces = u32::try_from(cs.len()).unwrap_or(u32::MAX);
                let feature = cs.iter().find_map(|c| c.classes.new_split).flatten();
                Ok(cs
                    .iter()
                    .map(|c| Resolved {
                        body: c.body,
                        kind: r.kind,
                        index: c.index,
                        quality: MatchQuality::Disambiguated,
                        warning: Some(ResolveWarning::NewSplit { feature, pieces }),
                    })
                    .collect())
            }
            Err(e) => Err(e),
        };
        let current = match &result {
            Ok(xs) => xs.iter().map(|x| current_ref(view, geom, r, x)).collect(),
            Err(_) => Vec::new(),
        };
        records.push(ResolutionRecord {
            r: EntityRef::Topo(r.clone()),
            result: result.clone(),
            current,
        });
        match result {
            Ok(xs) => members.push(xs),
            Err(e) if policy == SetPolicy::AllOrError => return (records, Err(e)),
            Err(e) => {
                first_err.get_or_insert(e);
                members.push(Vec::new());
            }
        }
    }
    if policy == SetPolicy::DropMissing
        && !refs.is_empty()
        && members.iter().all(Vec::is_empty)
        && let Some(e) = first_err
    {
        return (records, Err(e));
    }
    (records, Ok(members))
}

/// S11 R1: a reference to sub-entity `(kind, index)` of `body` as it is in `view`, with its
/// fingerprint (`key` 0; the selection widget allocates it). `None` if the body or the name
/// doesn't exist.
pub fn topo_ref_at(
    view: &dyn NamingView,
    kernel: &dyn BrepKernel,
    body: BodyId,
    kind: TopoKind,
    index: u32,
) -> Option<TopoRef> {
    let name = view.naming(body)?.name_of(kind, index)?.clone();
    let fingerprint = KernelGeom::new(view, kernel).fingerprint(body, kind, index);
    Some(TopoRef {
        body,
        kind,
        name,
        fingerprint,
        key: 0,
    })
}

fn kind_word(kind: TopoKind) -> &'static str {
    match kind {
        TopoKind::Face => "Face",
        TopoKind::Edge => "Edge",
        TopoKind::Vertex => "Vertex",
        TopoKind::FaceGroup => "Face group",
    }
}

/// The user-facing message of a warning (note §7).
pub fn warning_message(w: &ResolveWarning, kind: TopoKind) -> String {
    let k = kind_word(kind);
    match w {
        ResolveWarning::NewSplit {
            feature: Some(f),
            pieces,
        } => format!("{k} was split by feature {f} into {pieces} pieces; using the closest piece."),
        ResolveWarning::NewSplit {
            feature: None,
            pieces,
        } => format!(
            "The faces now share {pieces} {}s; using the closest.",
            k.to_lowercase()
        ),
        ResolveWarning::Unmerged => {
            format!("{k} was a merge that no longer happens; using the closest part.")
        }
        ResolveWarning::Widened {
            cause: WidenCause::SplitGone { .. },
        } => format!(
            "{k} was a piece of a split that no longer happens; using the whole {}.",
            k.to_lowercase()
        ),
        ResolveWarning::Widened {
            cause: WidenCause::Merged { feature },
        } => format!(
            "{k} was merged with neighbouring faces by feature {feature}; using the merged face."
        ),
        ResolveWarning::IndexDrift => {
            "Pieces of the split face changed order; matched by position.".to_owned()
        }
        ResolveWarning::ClassChanged => {
            "Some candidates changed shape type (e.g. plane to cone); check the match.".to_owned()
        }
        ResolveWarning::LowMargin { .. } | ResolveWarning::FarMatch { .. } => {
            "Match is uncertain.".to_owned()
        }
        ResolveWarning::BodyChanged { now_in } => {
            format!(
                "{k} is now in body {}/{} (split off).",
                now_in.origin, now_in.slot
            )
        }
        ResolveWarning::FingerprintOnly { .. } => format!(
            "Original {} is gone; matched a {} with the same shape and position.",
            k.to_lowercase(),
            k.to_lowercase()
        ),
        ResolveWarning::NoFingerprint => {
            "Reference has no fingerprint; matched by name only.".to_owned()
        }
    }
}

/// The user-facing message of a resolution error (note §7).
pub(crate) fn error_message(e: &ResolveError, kind: TopoKind) -> String {
    let k = kind_word(kind);
    match e {
        ResolveError::NotFound {
            removed_by: Some(f),
            ..
        } => format!("{k} was removed by feature {f}."),
        ResolveError::NotFound { .. } => format!("{k} no longer exists."),
        ResolveError::Ambiguous { candidates } => format!(
            "Reference matches {} {}s equally well.",
            candidates.len(),
            k.to_lowercase()
        ),
        ResolveError::BodyMissing(b) => {
            format!("Body {}/{} does not exist here.", b.origin, b.slot)
        }
        ResolveError::Unsupported => "This body has no sub-entity references.".to_owned(),
    }
}
