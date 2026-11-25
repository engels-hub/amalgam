//! [`NamingBuilder`]: element maps from S02 op histories (note §4).

use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::Arc;

use amg_brep::geom::Tag;
use amg_brep::{BrepKernel, EdgeId, InputSub, OpRole, Relation, ShapeHistory, SubKind, SubShape};

use super::topo::{
    NamingTopology, ShapeTopo, dedup_names, derive_edges_and_vertices, face_neighbours, order_key,
    sort_geometric,
};
use super::{
    BodyNaming, MAX_NAME_DEPTH, NameSeg, NameSource, Role, TopoKind, TopoName, identity_digest,
};
use crate::id::FeatureId;

/// How one operand of a kernel op is named (note §4.1).
#[derive(Debug)]
pub enum OperandNaming<'a> {
    /// An existing body with its element map (boolean target, an earlier step's result).
    Named(&'a BodyNaming),
    /// A freshly built face or wire whose edges carry builder tags (S02
    /// `FaceBuild::edge_tags`); [`NamingBuilder::tag`] gives each tag a [`NameSource`].
    Tagged(&'a [(EdgeId, Tag)]),
    /// No names: faces generated from it get role-only names.
    Anonymous,
}

/// Order of the pieces of a split face (note §4.2 rule 2).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum SplitOrder {
    /// Measure descending, then centroid (the quantised key of note §4.3).
    #[default]
    Geometric,
    /// By the piece slot of each output face (split body), then geometric.
    PieceSlot,
}

/// Why naming failed. A bug in the feature or the kernel history, never a user error.
#[derive(thiserror::Error, Clone, Debug, PartialEq)]
pub enum NamingError {
    /// The history refers to an operand the feature did not pass.
    #[error("history refers to operand {0} that was not supplied")]
    MissingOperand(u16),
    /// The history, an operand's naming or the shape disagree on a count.
    #[error("{kind:?} count mismatch: naming {naming}, shape {shape}")]
    CountMismatch {
        /// Index space.
        kind: TopoKind,
        /// Count (or index + 1) on the naming/history side.
        naming: u32,
        /// Count on the shape side.
        shape: u32,
    },
    /// Two sub-shapes ended up with the same name after dedup.
    #[error("duplicate {kind:?} name after dedup")]
    Duplicate {
        /// Index space.
        kind: TopoKind,
    },
    /// A kernel query failed.
    #[error("kernel query failed: {0}")]
    Kernel(String),
}

impl amg_core::Diagnostic for NamingError {}

/// Side information of a [`NamingBuilder::finish_report`] run.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct NamingReport {
    /// Output faces with neither a source nor a role (`OpRole::Unexplained`): their names are
    /// weak and the feature should report `KernelWarning::PartialHistory` (note §4.2 rule 5).
    pub unexplained: Vec<u32>,
}

impl NamingReport {
    /// True if some face was named by rule 5.
    pub fn partial_history(&self) -> bool {
        !self.unexplained.is_empty()
    }
}

enum Operand {
    Named(BodyNaming),
    Tagged(BTreeMap<u32, Tag>),
    Anonymous,
}

/// Builds the element map of one kernel op's result (note §4). One builder per op; a
/// multi-step feature passes an earlier step's naming as a `Named` operand of the next.
pub struct NamingBuilder {
    feature: FeatureId,
    tags: BTreeMap<u64, NameSource>,
    roles: Vec<(OpRole, Role)>,
    explicit: BTreeMap<(TopoKind, u32), (Role, Option<NameSource>)>,
    operands: BTreeMap<u16, Operand>,
    history: ShapeHistory,
    split_order: SplitOrder,
    piece_of_face: Option<Vec<u32>>,
    selections: BTreeMap<(u16, SubShape), u32>,
}

impl std::fmt::Debug for NamingBuilder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NamingBuilder")
            .field("feature", &self.feature)
            .field("operands", &self.operands.len())
            .field("explicit", &self.explicit.len())
            .finish_non_exhaustive()
    }
}

/// The default `OpRole → Role` mapping (note §4.2 rule 6).
fn default_role(r: &OpRole) -> Role {
    match r {
        OpRole::StartCap => Role::new("StartCap", 0),
        OpRole::EndCap => Role::new("EndCap", 0),
        OpRole::Lateral => Role::new("Lateral", 0),
        OpRole::FilletBlend { .. } => Role::new("Blend", 0),
        OpRole::ChamferFace => Role::new("Chamfer", 0),
        OpRole::CornerPatch => Role::new("Corner", 0),
        OpRole::ShellInner => Role::new("ShellInner", 0),
        OpRole::OffsetFace => Role::new("Offset", 0),
        OpRole::SectionEdge => Role::new("Section", 0),
        OpRole::LoftSection { index: 0 } => Role::new("LoftCap", 0),
        OpRole::LoftSection { .. } => Role::new("LoftCap", 1),
        OpRole::SplitPiece { index } => Role::new("SplitPiece", *index),
        _ => Role::new("Unexplained", 0),
    }
}

fn kind_of(k: SubKind) -> TopoKind {
    TopoKind::from_sub_kind(k)
}

/// A nested name as a source, replaced by its identity digest past [`MAX_NAME_DEPTH`].
fn topo_source(n: TopoName) -> NameSource {
    if n.depth() + 1 > MAX_NAME_DEPTH {
        NameSource::Digest(identity_digest(&n))
    } else {
        NameSource::Topo(n)
    }
}

/// `names` (≥ 1) as one name: the smallest, plus `Merge { feature, with: the others }`.
fn merged(mut names: Vec<TopoName>, feature: FeatureId) -> TopoName {
    names.sort();
    names.dedup();
    let primary = names.remove(0);
    if names.is_empty() {
        primary
    } else {
        primary.with(NameSeg::Merge {
            feature,
            with: names,
        })
    }
}

impl NamingBuilder {
    /// A builder for an op run by `feature`.
    pub fn new(feature: FeatureId) -> NamingBuilder {
        NamingBuilder {
            feature,
            tags: BTreeMap::new(),
            roles: Vec::new(),
            explicit: BTreeMap::new(),
            operands: BTreeMap::new(),
            history: ShapeHistory::default(),
            split_order: SplitOrder::Geometric,
            piece_of_face: None,
            selections: BTreeMap::new(),
        }
    }

    /// The feature this builder names for.
    pub fn feature(&self) -> FeatureId {
        self.feature
    }

    /// Gives builder tag `tag` (on edges of a `Tagged` operand) its name source, e.g. a sketch
    /// curve.
    pub fn tag(&mut self, tag: Tag, source: NameSource) -> &mut Self {
        self.tags.insert(tag.0, source);
        self
    }

    /// Overrides the default role of output faces with `op_role`.
    pub fn role(&mut self, op_role: OpRole, role: Role) -> &mut Self {
        self.roles.retain(|(r, _)| *r != op_role);
        self.roles.push((op_role, role));
        self
    }

    /// The op's history and its operands in kernel input order (one entry per operand).
    pub fn from_history(
        &mut self,
        operands: &[(u16, OperandNaming<'_>)],
        history: &ShapeHistory,
    ) -> &mut Self {
        self.operands = operands
            .iter()
            .map(|(i, o)| {
                let op = match o {
                    OperandNaming::Named(n) => Operand::Named((*n).clone()),
                    OperandNaming::Tagged(t) => {
                        Operand::Tagged(t.iter().map(|(e, tag)| (e.0, *tag)).collect())
                    }
                    OperandNaming::Anonymous => Operand::Anonymous,
                };
                (*i, op)
            })
            .collect();
        self.history = history.clone();
        self
    }

    /// Explicit name for one output face (or a face-less edge or vertex); wins over history.
    pub fn generated(
        &mut self,
        kind: TopoKind,
        index: u32,
        role: Role,
        from: Option<NameSource>,
    ) -> &mut Self {
        self.explicit.insert((kind, index), (role, from));
        self
    }

    /// The split order, and for [`SplitOrder::PieceSlot`] the piece slot of every output face.
    pub fn split_order(&mut self, order: SplitOrder, piece_of_face: Option<Vec<u32>>) -> &mut Self {
        self.split_order = order;
        self.piece_of_face = piece_of_face;
        self
    }

    /// Input sub-shape `sub` of operand `operand` is the resolution of a picked reference with
    /// selection key `key`: faces generated from it get `from: Selection { key }` (note §5.1).
    pub fn selection(&mut self, operand: u16, sub: SubShape, key: u32) -> &mut Self {
        self.selections.insert((operand, sub), key);
        self
    }

    /// Names the op's result `shape` (note §4.2–4.4). Calls `kernel.sub_geom` only for
    /// sub-shapes whose order must be decided.
    ///
    /// # Errors
    /// See [`NamingError`].
    pub fn finish(
        self,
        kernel: &dyn BrepKernel,
        shape: &amg_brep::Shape,
    ) -> Result<BodyNaming, NamingError> {
        self.finish_report(kernel, shape).map(|(n, _)| n)
    }

    /// [`NamingBuilder::finish`] plus the [`NamingReport`].
    ///
    /// # Errors
    /// See [`NamingError`].
    pub fn finish_report(
        self,
        kernel: &dyn BrepKernel,
        shape: &amg_brep::Shape,
    ) -> Result<(BodyNaming, NamingReport), NamingError> {
        self.finish_topo(&ShapeTopo::new(shape, Some(kernel)))
    }

    fn input_name(&self, inp: InputSub) -> Result<Option<TopoName>, NamingError> {
        match self.operands.get(&inp.operand) {
            None => Err(NamingError::MissingOperand(inp.operand)),
            Some(Operand::Named(n)) => {
                let kind = kind_of(inp.sub.kind);
                match n.name_of(kind, inp.sub.index) {
                    Some(name) => Ok(Some(name.clone())),
                    None => Err(NamingError::CountMismatch {
                        kind,
                        naming: u32::try_from(n.list(kind).map_or(0, <[TopoName]>::len))
                            .unwrap_or(u32::MAX),
                        shape: inp.sub.index.saturating_add(1),
                    }),
                }
            }
            Some(_) => Ok(None),
        }
    }

    /// The name source of input `g` for a generated face (note §4.2 rule 3).
    fn source_of(&self, g: InputSub) -> Result<Option<NameSource>, NamingError> {
        if let Some(key) = self.selections.get(&(g.operand, g.sub)) {
            return Ok(Some(NameSource::Selection { key: *key }));
        }
        if let Some(Operand::Tagged(tags)) = self.operands.get(&g.operand) {
            if g.sub.kind == SubKind::Edge
                && let Some(tag) = tags.get(&g.sub.index)
                && let Some(src) = self.tags.get(&tag.0)
            {
                return Ok(Some(src.clone()));
            }
            return Ok(None);
        }
        Ok(self.input_name(g)?.map(topo_source))
    }

    fn role_for(&self, r: &OpRole) -> Role {
        self.roles
            .iter()
            .find(|(o, _)| o == r)
            .map_or_else(|| default_role(r), |(_, role)| role.clone())
    }

    /// Names a result given as a [`NamingTopology`] (`finish` uses `amg-brep`; tests and
    /// benches use [`super::SyntheticTopology`]).
    ///
    /// # Errors
    /// See [`NamingError`].
    #[allow(clippy::too_many_lines, reason = "the rules of note §4.2 in order")]
    pub fn finish_topo(
        self,
        topo: &dyn NamingTopology,
    ) -> Result<(BodyNaming, NamingReport), NamingError> {
        let feature = self.feature;
        let [nf, ne, nv] = topo.counts();
        let count_of = |k: SubKind| match k {
            SubKind::Face => nf,
            SubKind::Edge => ne,
            SubKind::Vertex => nv,
        };
        let check_out = |o: &SubShape| {
            if o.index >= count_of(o.kind) {
                Err(NamingError::CountMismatch {
                    kind: kind_of(o.kind),
                    naming: o.index.saturating_add(1),
                    shape: count_of(o.kind),
                })
            } else {
                Ok(())
            }
        };

        // Sources of every output face, and the face outputs of every input face.
        let mut mod_src: Vec<Vec<InputSub>> = vec![Vec::new(); nf as usize];
        let mut gen_src: Vec<Vec<InputSub>> = vec![Vec::new(); nf as usize];
        let mut face_outs: BTreeMap<InputSub, Vec<u32>> = BTreeMap::new();
        let mut edge_src: HashMap<(SubKind, u32), Vec<InputSub>> = HashMap::new();
        for (inp, rel) in &self.history.entries {
            if !self.operands.contains_key(&inp.operand) {
                return Err(NamingError::MissingOperand(inp.operand));
            }
            let (outs, modified): (&[SubShape], bool) = match rel {
                Relation::Modified(o) => (o.as_slice(), true),
                Relation::Kept(o) => (std::slice::from_ref(o), true),
                Relation::Generated(o) => (o.as_slice(), false),
                Relation::Deleted => (&[], false),
            };
            if inp.sub.kind == SubKind::Face {
                face_outs.entry(*inp).or_default();
            }
            for o in outs {
                check_out(o)?;
                match (o.kind, modified && inp.sub.kind == o.kind) {
                    (SubKind::Face, true) => {
                        mod_src[o.index as usize].push(*inp);
                        face_outs.entry(*inp).or_default().push(o.index);
                    }
                    (SubKind::Face, false) => gen_src[o.index as usize].push(*inp),
                    (k, true) => edge_src.entry((k, o.index)).or_default().push(*inp),
                    (_, false) => {}
                }
            }
        }
        for (o, _) in &self.history.roles {
            check_out(o)?;
        }

        // Split order of every input face with several pieces.
        let mut piece_pos: HashMap<(InputSub, u32), (u32, u32)> = HashMap::new();
        for (inp, outs) in &face_outs {
            if outs.len() < 2 {
                continue;
            }
            let mut ordered = outs.clone();
            ordered.sort_unstable();
            ordered.dedup();
            match (self.split_order, &self.piece_of_face) {
                (SplitOrder::PieceSlot, Some(slots)) => {
                    let mut keyed: Vec<_> = ordered
                        .iter()
                        .map(|&o| {
                            let slot = slots.get(o as usize).copied().unwrap_or(u32::MAX);
                            let fp = topo.fingerprint(TopoKind::Face, o);
                            ((slot, order_key(fp.as_ref(), o)), o)
                        })
                        .collect();
                    keyed.sort_unstable();
                    ordered = keyed.into_iter().map(|(_, o)| o).collect();
                }
                _ => sort_geometric(topo, TopoKind::Face, &mut ordered),
            }
            let k = u32::try_from(ordered.len()).unwrap_or(u32::MAX);
            for (i, o) in (0u32..).zip(&ordered) {
                piece_pos.insert((*inp, *o), (i, k));
            }
        }

        // Rules 1-3.
        let mut names: Vec<Option<TopoName>> = vec![None; nf as usize];
        let mut deferred: Vec<(u32, Role, bool)> = Vec::new();
        for o in 0..nf {
            let oi = o as usize;
            if let Some((role, from)) = self.explicit.get(&(TopoKind::Face, o)) {
                names[oi] = Some(TopoName::new(vec![NameSeg::Gen {
                    feature,
                    role: role.clone(),
                    from: from.clone(),
                }]));
                continue;
            }
            let mut pieces = Vec::new();
            for s in &mod_src[oi] {
                let Some(n) = self.input_name(*s)? else {
                    continue;
                };
                pieces.push(match piece_pos.get(&(*s, o)) {
                    Some(&(index, count)) => n.with(NameSeg::Split {
                        feature,
                        index,
                        count,
                    }),
                    None => n,
                });
            }
            if !pieces.is_empty() {
                names[oi] = Some(merged(pieces, feature));
                continue;
            }
            let op_role = self.history.role_of(SubShape {
                kind: SubKind::Face,
                index: o,
            });
            let role = op_role.map_or_else(|| Role::new("Face", 0), |r| self.role_for(r));
            let mut gens = Vec::new();
            for g in &gen_src[oi] {
                if let Some(src) = self.source_of(*g)? {
                    gens.push(TopoName::new(vec![NameSeg::Gen {
                        feature,
                        role: role.clone(),
                        from: Some(src),
                    }]));
                }
            }
            if !gens.is_empty() {
                names[oi] = Some(merged(gens, feature));
                continue;
            }
            match op_role {
                Some(r) if *r != OpRole::Unexplained => deferred.push((o, role, false)),
                _ => deferred.push((o, Role::new("Unexplained", 0), true)),
            }
        }

        // Rules 4-5: named from the faces next to them that rules 1-3 named.
        let mut report = NamingReport::default();
        if !deferred.is_empty() {
            let neighbours = face_neighbours(topo);
            let first: Vec<Option<TopoName>> = names.clone();
            for (o, role, unexplained) in deferred {
                let mut adj: Vec<TopoName> = neighbours
                    .get(&o)
                    .into_iter()
                    .flatten()
                    .filter_map(|&a| first[a as usize].clone())
                    .collect();
                adj.sort();
                let from = (!adj.is_empty()).then(|| {
                    topo_source(TopoName::new(vec![NameSeg::Bound {
                        faces: adj,
                        index: 0,
                        count: 1,
                    }]))
                });
                names[o as usize] = Some(TopoName::new(vec![NameSeg::Gen {
                    feature,
                    role,
                    from,
                }]));
                if unexplained {
                    report.unexplained.push(o);
                }
            }
        }
        let mut faces: Vec<TopoName> = names.into_iter().map(Option::unwrap_or_default).collect();
        dedup_names(&mut faces, topo, TopoKind::Face, feature);
        unique(&faces, TopoKind::Face)?;

        // Tombstones (note §4.4): input faces deleted or only generated from.
        let mut removed: Option<Arc<BTreeMap<TopoName, FeatureId>>> = None;
        for op in self.operands.values() {
            if let Operand::Named(n) = op {
                match &mut removed {
                    None => removed = Some(Arc::clone(&n.removed)),
                    Some(r) if Arc::ptr_eq(r, &n.removed) || n.removed.is_empty() => {}
                    Some(r) => {
                        Arc::make_mut(r).extend(n.removed.iter().map(|(k, v)| (k.clone(), *v)))
                    }
                }
            }
        }
        let mut removed = removed.unwrap_or_default();
        for (inp, outs) in &face_outs {
            if outs.is_empty()
                && let Some(name) = self.input_name(*inp)?
            {
                Arc::make_mut(&mut removed).insert(name, feature);
            }
        }

        // Edges and vertices (note §4.3).
        let mut prior_edges: HashMap<(TopoKind, u32), TopoName> = HashMap::new();
        for ((k, o), srcs) in &edge_src {
            if let [s] = srcs.as_slice()
                && let Some(n) = self.input_name(*s)?
            {
                prior_edges.insert((kind_of(*k), *o), n);
            }
        }
        for (sub, tag) in &self.history.tags {
            if sub.kind != SubKind::Face
                && let Some(src) = self.tags.get(&tag.0)
            {
                let tag_name = if sub.kind == SubKind::Edge {
                    "Edge"
                } else {
                    "Vertex"
                };
                prior_edges.insert(
                    (kind_of(sub.kind), sub.index),
                    TopoName::new(vec![NameSeg::Gen {
                        feature,
                        role: Role::new(tag_name, 0),
                        from: Some(src.clone()),
                    }]),
                );
            }
        }
        for ((kind, i), (role, from)) in &self.explicit {
            if *kind != TopoKind::Face {
                prior_edges.insert(
                    (*kind, *i),
                    TopoName::new(vec![NameSeg::Gen {
                        feature,
                        role: role.clone(),
                        from: from.clone(),
                    }]),
                );
            }
        }
        let prior = |k: TopoKind, i: u32| prior_edges.get(&(k, i)).cloned();
        let (edges, vertices) = derive_edges_and_vertices(topo, feature, &faces, &prior)?;
        unique(&edges, TopoKind::Edge)?;
        unique(&vertices, TopoKind::Vertex)?;

        let mut naming = BodyNaming::new(faces, edges, vertices);
        naming.removed = removed;
        Ok((naming, report))
    }
}

fn unique(names: &[TopoName], kind: TopoKind) -> Result<(), NamingError> {
    let mut seen = HashSet::with_capacity(names.len());
    if names.iter().all(|n| seen.insert(n)) {
        Ok(())
    } else {
        Err(NamingError::Duplicate { kind })
    }
}
