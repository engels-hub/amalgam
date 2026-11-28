//! Op results and their history data (S02 §6).
//!
//! BREP-03 landed the data types of the Interfaces. BREP-05 adds the framework every op goes
//! through: extraction from OCCT (in the shim, `occt_history.hxx`), [`ShapeHistory::compose`],
//! the deterministic split order ([`split_order_key`], [`ShapeHistory::order_splits`]), the
//! completeness rule (every output face has a source or a role, else
//! [`OpRole::Unexplained`] plus [`KernelWarning::PartialHistory`]) and per-solid views of a
//! result ([`OpResult::per_solid`]).

use std::collections::{BTreeMap, BTreeSet, HashMap};

use smallvec::SmallVec;

use crate::error::KResult;
use crate::geom::Tag;
use crate::query::SubShapeGeom;
use crate::shape::{FaceId, Shape, SubKind, SubShape};

/// Quantum of the split-order key: mm for centroids, mm² or mm for measures (persistent-naming
/// note §4.3).
pub const SPLIT_QUANTUM: f64 = 1e-6;

/// The split-order key of one sub-shape (see [`split_order_key`]).
pub type SplitKey = (i64, i64, i64, i64, u32);

/// The total order of split pieces (S02 §6, naming note §4.3 and request Q5): measure
/// descending, then centroid x, y, z ascending, then the in-shape index, all compared as
/// integers after quantising to [`SPLIT_QUANTUM`] (`q(x) = round(x / 1e-6)`), so the order is
/// total and never depends on a tolerance comparison. [`ShapeHistory::order_splits`] sorts
/// with it, and S01 DOC-06 can order its `Bound` groups with the same key.
pub fn split_order_key(g: &SubShapeGeom, index: u32) -> SplitKey {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "saturating cast; no part is 9e12 mm across"
    )]
    let q = |x: f64| (x / SPLIT_QUANTUM).round() as i64;
    (
        -q(g.measure),
        q(g.centroid.x),
        q(g.centroid.y),
        q(g.centroid.z),
        index,
    )
}

/// An input sub-shape: `operand` is the position in the op's input list.
#[derive(
    Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct InputSub {
    /// Index of the input shape.
    pub operand: u16,
    /// The sub-shape in that input's index space.
    pub sub: SubShape,
}

/// What became of one input sub-shape.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Relation {
    /// Same kind; split order is area descending, then centroid lexicographic.
    Modified(SmallVec<[SubShape; 2]>),
    /// May cross kinds (edge → face, vertex → edge).
    Generated(SmallVec<[SubShape; 2]>),
    /// The same TShape passed through.
    Kept(SubShape),
    /// Gone from the result.
    Deleted,
}

/// Op-semantic labels on output sub-shapes that OCCT's generic history does not express.
#[derive(Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum OpRole {
    /// Prism/revol start cap.
    StartCap,
    /// Prism/revol end cap.
    EndCap,
    /// A swept side face.
    Lateral,
    /// A fillet face of contour `contour`.
    FilletBlend {
        /// Contour index.
        contour: u32,
    },
    /// A chamfer face.
    ChamferFace,
    /// A corner patch of a fillet or chamfer.
    CornerPatch,
    /// An inner face of a shell.
    ShellInner,
    /// An offset face.
    OffsetFace,
    /// A section edge.
    SectionEdge,
    /// A loft section face.
    LoftSection {
        /// Section index.
        index: u32,
    },
    /// A piece of a split.
    SplitPiece {
        /// Piece index.
        index: u32,
    },
    /// Created with no source and no other role.
    Unexplained,
}

/// History of one op: relations from input sub-shapes, roles and builder tags on outputs.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ShapeHistory {
    /// Sorted by [`InputSub`]; one entry per input sub-shape and relation.
    pub entries: Vec<(InputSub, Relation)>,
    /// Roles of output sub-shapes.
    pub roles: Vec<(SubShape, OpRole)>,
    /// Builder tags on output edges and faces, sorted by sub-shape.
    pub tags: Vec<(SubShape, Tag)>,
    /// Faces, edges and vertices per operand (validation).
    pub operand_counts: Vec<[u32; 3]>,
}

impl ShapeHistory {
    /// The relations recorded for input `i`.
    pub fn of_input(&self, i: InputSub) -> impl Iterator<Item = &Relation> + '_ {
        self.entries
            .iter()
            .filter(move |(input, _)| *input == i)
            .map(|(_, relation)| relation)
    }

    /// Inverse view: every input relation that produced output sub-shape `o`.
    pub fn sources_of(&self, o: SubShape) -> Vec<(InputSub, &Relation)> {
        self.entries
            .iter()
            .filter(|(_, relation)| match relation {
                Relation::Modified(outs) | Relation::Generated(outs) => outs.contains(&o),
                Relation::Kept(out) => *out == o,
                Relation::Deleted => false,
            })
            .map(|(input, relation)| (*input, relation))
            .collect()
    }

    /// The role of output `o`, if any.
    pub fn role_of(&self, o: SubShape) -> Option<&OpRole> {
        self.roles
            .iter()
            .find(|(sub, _)| *sub == o)
            .map(|(_, role)| role)
    }

    /// The builder tag of output `o`, if any.
    pub fn tag_of(&self, o: SubShape) -> Option<Tag> {
        self.tags
            .iter()
            .find(|(sub, _)| *sub == o)
            .map(|(_, tag)| *tag)
    }

    /// Composes `self` (A → B) with `next` (B → C), where B is `next`'s operand 0 (S20 request
    /// R8): one history from A (and `next`'s other operands) to C, as if one op had run.
    ///
    /// - Relations chain per output: `Kept` then `Kept` is `Kept`; `Modified` or `Kept`
    ///   followed by `Modified` or `Kept` is `Modified`; anything involving `Generated` is
    ///   `Generated`. An input whose every image is gone is one `Deleted` entry. A B sub-shape
    ///   with no entry in `next` counts as deleted (every op lists each operand sub-shape; a
    ///   section lists only the ones in its result).
    /// - Operands: `self`'s, then `next`'s operands 1.. renumbered to follow them;
    ///   `operand_counts` likewise.
    /// - Roles and tags: `next`'s, plus `self`'s carried through `Modified`/`Kept` (not through
    ///   `Generated`: a face generated from a cap is not a cap). `next`'s win on a conflict.
    /// - Output lists are in index order. [`ShapeHistory::order_splits`] on C's shape restores
    ///   the split order; the kernel does that for every history it returns.
    pub fn compose(&self, next: &ShapeHistory) -> ShapeHistory {
        let mut by_b: HashMap<SubShape, Vec<&Relation>> = HashMap::new();
        for (input, relation) in &next.entries {
            if input.operand == 0 {
                by_b.entry(input.sub).or_default().push(relation);
            }
        }
        let images = |b: SubShape| by_b.get(&b).map(Vec::as_slice).unwrap_or(&[]);

        let mut entries: Vec<(InputSub, Relation)> = Vec::with_capacity(self.entries.len());
        let mut i = 0;
        while i < self.entries.len() {
            let input = self.entries[i].0;
            let mut modified = BTreeSet::new();
            let mut generated = BTreeSet::new();
            let mut kept = BTreeSet::new();
            while i < self.entries.len() && self.entries[i].0 == input {
                let (outs, first) = step_of(&self.entries[i].1);
                for &b in outs {
                    for r2 in images(b) {
                        let (cs, second) = step_of(r2);
                        let target = match (first, second) {
                            (Step::Deleted, _) | (_, Step::Deleted) => continue,
                            (Step::Generated, _) | (_, Step::Generated) => &mut generated,
                            (Step::Kept, Step::Kept) => &mut kept,
                            _ => &mut modified,
                        };
                        target.extend(cs.iter().copied());
                    }
                }
                i += 1;
            }
            let before = entries.len();
            if !modified.is_empty() {
                entries.push((input, Relation::Modified(modified.into_iter().collect())));
            }
            if !generated.is_empty() {
                entries.push((input, Relation::Generated(generated.into_iter().collect())));
            }
            for c in kept {
                entries.push((input, Relation::Kept(c)));
            }
            if entries.len() == before {
                entries.push((input, Relation::Deleted));
            }
        }
        let shift = u16::try_from(self.operand_counts.len()).unwrap_or(u16::MAX);
        let others = next
            .entries
            .iter()
            .filter(|(input, _)| input.operand > 0)
            .map(|(input, relation)| {
                let operand = (input.operand - 1).saturating_add(shift);
                (InputSub { operand, ..*input }, relation.clone())
            });
        entries.extend(others);
        // Stable: per input, Modified before Generated before Kept, as built.
        entries.sort_by_key(|(input, _)| *input);

        let mut operand_counts = self.operand_counts.clone();
        operand_counts.extend(next.operand_counts.iter().skip(1).copied());

        ShapeHistory {
            entries,
            roles: carried(&self.roles, &next.roles, &images),
            tags: carried(&self.tags, &next.tags, &images),
            operand_counts,
        }
    }

    /// Sorts the outputs of every `Modified` or `Generated` relation that has several outputs
    /// of one kind into the split order of `result`, the op's result shape: by kind, then
    /// [`split_order_key`]. Lists with at most one output per kind are left as they are (the
    /// kernel produces them in index order). Fingerprints are computed only for the sub-shapes
    /// that need them. This is the one place the split order is applied (S02 §6).
    ///
    /// # Errors
    /// `InvalidInput` when an output index is out of range for `result`; `Occt` if a
    /// fingerprint query raises.
    pub fn order_splits(&mut self, result: &Shape) -> KResult<()> {
        let mut needed: [BTreeSet<u32>; 3] = Default::default();
        for (_, relation) in &self.entries {
            if let Relation::Modified(outs) | Relation::Generated(outs) = relation {
                for kind in KINDS {
                    let of_kind = || outs.iter().filter(move |o| o.kind == kind);
                    if of_kind().count() > 1 {
                        needed[kind_slot(kind)].extend(of_kind().map(|o| o.index));
                    }
                }
            }
        }
        if needed.iter().all(BTreeSet::is_empty) {
            return Ok(());
        }
        let mut keys: HashMap<SubShape, SplitKey> = HashMap::new();
        for kind in KINDS {
            let indices: Vec<u32> = needed[kind_slot(kind)].iter().copied().collect();
            let geoms = crate::query::sub_geoms(result, kind, &indices)?;
            for (index, g) in indices.into_iter().zip(&geoms) {
                keys.insert(SubShape { kind, index }, split_order_key(g, index));
            }
        }
        for (_, relation) in &mut self.entries {
            if let Relation::Modified(outs) | Relation::Generated(outs) = relation {
                outs.sort_by_key(|o| (o.kind, keys.get(o).copied(), o.index));
            }
        }
        Ok(())
    }

    /// The output faces (of a result with `face_count` faces) that are neither `Modified`,
    /// `Generated` nor `Kept` from any input and have no role other than
    /// [`OpRole::Unexplained`]: what the completeness rule (S02 §6) forbids.
    pub fn unexplained_faces(&self, face_count: u32) -> Vec<FaceId> {
        let mut explained = vec![false; face_count as usize];
        let mut mark = |o: &SubShape| {
            if o.kind == SubKind::Face
                && let Some(slot) = explained.get_mut(o.index as usize)
            {
                *slot = true;
            }
        };
        for (_, relation) in &self.entries {
            step_of(relation).0.iter().for_each(&mut mark);
        }
        for (o, role) in &self.roles {
            if *role != OpRole::Unexplained {
                mark(o);
            }
        }
        (0..face_count)
            .filter(|&f| !explained[f as usize])
            .map(FaceId)
            .collect()
    }

    /// The first index that is out of range, described: an input beyond its operand's
    /// `operand_counts` (or of an operand without counts), or an output, role or tag beyond
    /// `output_counts` (faces, edges, vertices of the result). `None` when all are in range.
    pub fn range_error(&self, output_counts: [u32; 3]) -> Option<String> {
        let slot = |s: &SubShape| kind_slot(s.kind);
        for (input, relation) in &self.entries {
            let Some(counts) = self.operand_counts.get(usize::from(input.operand)) else {
                return Some(format!(
                    "{input:?}: operand {} has no counts",
                    input.operand
                ));
            };
            if input.sub.index >= counts[slot(&input.sub)] {
                return Some(format!("input {input:?} out of range ({counts:?})"));
            }
            let outs = step_of(relation).0;
            if let Some(bad) = outs.iter().find(|o| o.index >= output_counts[slot(o)]) {
                return Some(format!("output {bad:?} of {input:?} out of range"));
            }
        }
        let mut labelled = self
            .roles
            .iter()
            .map(|(o, _)| o)
            .chain(self.tags.iter().map(|(o, _)| o));
        labelled
            .find(|o| o.index >= output_counts[slot(o)])
            .map(|bad| format!("role or tag on {bad:?} out of range"))
    }
}

const KINDS: [SubKind; 3] = [SubKind::Face, SubKind::Edge, SubKind::Vertex];

fn kind_slot(kind: SubKind) -> usize {
    match kind {
        SubKind::Face => 0,
        SubKind::Edge => 1,
        SubKind::Vertex => 2,
    }
}

/// A relation's kind, for `compose`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Modified,
    Generated,
    Kept,
    Deleted,
}

/// A relation's outputs and kind.
fn step_of(relation: &Relation) -> (&[SubShape], Step) {
    match relation {
        Relation::Modified(o) => (o.as_slice(), Step::Modified),
        Relation::Generated(o) => (o.as_slice(), Step::Generated),
        Relation::Kept(o) => (std::slice::from_ref(o), Step::Kept),
        Relation::Deleted => (&[], Step::Deleted),
    }
}

/// `next`'s labels plus `first`'s carried through `Modified`/`Kept` images (`compose`), sorted
/// by sub-shape, one per sub-shape.
fn carried<'a, T: Clone>(
    first: &[(SubShape, T)],
    next: &[(SubShape, T)],
    images: &impl Fn(SubShape) -> &'a [&'a Relation],
) -> Vec<(SubShape, T)> {
    let mut out: BTreeMap<SubShape, T> = BTreeMap::new();
    for (c, label) in next {
        out.entry(*c).or_insert_with(|| label.clone());
    }
    for (b, label) in first {
        for r in images(*b) {
            let (cs, step) = step_of(r);
            if matches!(step, Step::Modified | Step::Kept) {
                for c in cs {
                    out.entry(*c).or_insert_with(|| label.clone());
                }
            }
        }
    }
    out.into_iter().collect()
}

/// The kernel's last step on every op's history (BREP-05): the split order, then the
/// completeness rule (S02 §6, naming note Q4): an output face without a source or role gets
/// [`OpRole::Unexplained`] and the result [`KernelWarning::PartialHistory`]. A failing
/// fingerprint query leaves the index order (logged): the history itself is still right.
pub(crate) fn finish(
    history: &mut ShapeHistory,
    result: &Shape,
    warnings: &mut Vec<KernelWarning>,
) {
    if let Err(e) = history.order_splits(result) {
        tracing::warn!("split order not applied: {e}");
    }
    let unexplained = history.unexplained_faces(result.count(SubKind::Face));
    if unexplained.is_empty() {
        return;
    }
    for f in unexplained {
        let o = SubShape {
            kind: SubKind::Face,
            index: f.0,
        };
        if history.role_of(o).is_none() {
            history.roles.push((o, OpRole::Unexplained));
        }
    }
    history.roles.sort_by_key(|(o, _)| *o);
    if !warnings.contains(&KernelWarning::PartialHistory) {
        warnings.push(KernelWarning::PartialHistory);
    }
}

/// Non-fatal notes on an op's result.
#[derive(Clone, Debug, PartialEq)]
pub enum KernelWarning {
    /// Healing applied `fixes` fixes.
    Healed {
        /// Number of fixes.
        fixes: u32,
    },
    /// A tolerance grew to `max` mm.
    ToleranceGrown {
        /// The new largest tolerance, mm.
        max: f64,
    },
    /// A fuzzy value was used, mm.
    FuzzyUsed(f64),
    /// The history is incomplete.
    PartialHistory,
}

/// A shape-producing op's result.
#[derive(Clone, Debug)]
pub struct OpResult {
    /// The result.
    pub shape: Shape,
    /// How it relates to the inputs.
    pub history: ShapeHistory,
    /// Non-fatal notes.
    pub warnings: Vec<KernelWarning>,
}

impl OpResult {
    /// One result per solid of this result's shape (R-S04-2, naming note Q3), in index order
    /// ([`Shape::solids`]), each with this history restricted to the solid and reindexed into
    /// its own index space: outputs in other solids are dropped (an input with no output left in
    /// the solid is `Deleted` there), roles and tags follow their sub-shapes, and the split
    /// order is applied again. Operands and warnings are unchanged. A shape without solids
    /// gives an empty list.
    ///
    /// # Errors
    /// `Occt` if a query raises.
    pub fn per_solid(&self) -> KResult<Vec<OpResult>> {
        let mut out = Vec::new();
        for solid in self.shape.solids() {
            let map = self.shape.occ().sub_index_map(solid.occ())?;
            let mut entries = Vec::new();
            for (k, kind) in KINDS.into_iter().enumerate() {
                for (local, whole) in map[k].iter().enumerate() {
                    let (Some(whole), Ok(local)) = (whole, u32::try_from(local)) else {
                        continue;
                    };
                    let b = SubShape {
                        kind,
                        index: *whole,
                    };
                    let c = SubShape { kind, index: local };
                    entries.push((InputSub { operand: 0, sub: b }, Relation::Kept(c)));
                }
            }
            entries.sort_by_key(|(input, _)| *input);
            let restriction = ShapeHistory {
                entries,
                operand_counts: vec![crate::ops::counts(&self.shape)],
                ..ShapeHistory::default()
            };
            let mut history = self.history.compose(&restriction);
            history.order_splits(&solid)?;
            out.push(OpResult {
                shape: solid,
                history,
                warnings: self.warnings.clone(),
            });
        }
        Ok(out)
    }
}

/// A face built by a face builder, with the tag of each created edge.
#[derive(Clone, Debug)]
pub struct FaceBuild {
    /// The face.
    pub face: Shape,
    /// `(edge, tag)` for every edge made from a tagged input curve, in input order. Empty for
    /// bounds without tags (`Natural`, `UvRect`, `Polylines`).
    pub edge_tags: Vec<(crate::EdgeId, Tag)>,
}

/// A wall face built by [`crate::BrepKernel::make_thin_face`].
#[derive(Clone, Debug)]
pub struct ThinFace {
    /// The face and the tag of every edge: an offset edge keeps its chain edge's tag, the end
    /// caps carry [`crate::geom::ThinProfile::end_tags`]. In order: the left boundary along the
    /// chain, the right boundary along the chain, the start cap, the end cap.
    pub build: FaceBuild,
    /// The edges of the boundary left of the chain (the chain itself where `left` is 0).
    pub left: Vec<crate::EdgeId>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::SubKind;

    fn edge(index: u32) -> SubShape {
        SubShape {
            kind: SubKind::Edge,
            index,
        }
    }

    #[test]
    fn lookups_find_their_entries() {
        let input = InputSub {
            operand: 0,
            sub: edge(3),
        };
        let history = ShapeHistory {
            entries: vec![(input, Relation::Generated(SmallVec::from_slice(&[edge(1)])))],
            roles: vec![(edge(2), OpRole::SectionEdge)],
            tags: vec![(edge(1), Tag(7))],
            operand_counts: vec![[0, 4, 0]],
        };
        assert_eq!(history.of_input(input).count(), 1);
        assert_eq!(history.sources_of(edge(1)).len(), 1);
        assert!(history.sources_of(edge(2)).is_empty());
        assert_eq!(history.role_of(edge(2)), Some(&OpRole::SectionEdge));
        assert_eq!(history.tag_of(edge(1)), Some(Tag(7)));
        assert_eq!(history.tag_of(edge(0)), None);
        let json = serde_json::to_string(&history).unwrap();
        let back: ShapeHistory = serde_json::from_str(&json).unwrap();
        assert_eq!(back, history);
    }

    fn face(index: u32) -> SubShape {
        SubShape {
            kind: SubKind::Face,
            index,
        }
    }

    fn input(operand: u16, sub: SubShape) -> InputSub {
        InputSub { operand, sub }
    }

    fn outs(subs: &[SubShape]) -> SmallVec<[SubShape; 2]> {
        SmallVec::from_slice(subs)
    }

    /// A → B: face 0 split into B faces 0 and 1, face 1 kept as B face 2, face 2 deleted, edge 0
    /// generating B face 3 (a lateral) and modified into B edge 0. Role StartCap on B face 2.
    fn first() -> ShapeHistory {
        ShapeHistory {
            entries: vec![
                (
                    input(0, face(0)),
                    Relation::Modified(outs(&[face(0), face(1)])),
                ),
                (input(0, face(1)), Relation::Kept(face(2))),
                (input(0, face(2)), Relation::Deleted),
                (input(0, edge(0)), Relation::Modified(outs(&[edge(0)]))),
                (input(0, edge(0)), Relation::Generated(outs(&[face(3)]))),
            ],
            roles: vec![(face(2), OpRole::StartCap), (face(3), OpRole::Lateral)],
            tags: vec![(edge(0), Tag(5))],
            operand_counts: vec![[3, 1, 0]],
        }
    }

    #[test]
    fn compose_chains_relations_operands_roles_and_tags() {
        // B → C with a second operand (a tool): B faces 0 and 1 merge into C face 0, B face 2 is
        // kept as C face 1, B face 3 is modified into C face 2, B edge 0 is missing (deleted),
        // tool face 0 generates C face 3 (role from `next`), tool face 1 is kept as C face 4.
        let next = ShapeHistory {
            entries: vec![
                (input(0, face(0)), Relation::Modified(outs(&[face(0)]))),
                (input(0, face(1)), Relation::Modified(outs(&[face(0)]))),
                (input(0, face(2)), Relation::Kept(face(1))),
                (input(0, face(3)), Relation::Modified(outs(&[face(2)]))),
                (input(1, face(0)), Relation::Generated(outs(&[face(3)]))),
                (input(1, face(1)), Relation::Kept(face(4))),
            ],
            roles: vec![(face(3), OpRole::SectionEdge)],
            tags: vec![],
            operand_counts: vec![[4, 1, 0], [2, 0, 0]],
        };
        let c = first().compose(&next);
        assert_eq!(
            c.entries,
            vec![
                (input(0, face(0)), Relation::Modified(outs(&[face(0)]))),
                (input(0, face(1)), Relation::Kept(face(1))),
                (input(0, face(2)), Relation::Deleted),
                (input(0, edge(0)), Relation::Generated(outs(&[face(2)]))),
                (input(1, face(0)), Relation::Generated(outs(&[face(3)]))),
                (input(1, face(1)), Relation::Kept(face(4))),
            ]
        );
        assert_eq!(c.operand_counts, vec![[3, 1, 0], [2, 0, 0]]);
        // StartCap follows the kept face, Lateral its modified image; `next`'s own role stays.
        assert_eq!(
            c.roles,
            vec![
                (face(1), OpRole::StartCap),
                (face(2), OpRole::Lateral),
                (face(3), OpRole::SectionEdge),
            ]
        );
        // The tagged edge is gone in C.
        assert!(c.tags.is_empty());
        assert_eq!(c.range_error([5, 0, 0]), None);
    }

    #[test]
    fn compose_with_identity_is_identity() {
        let h = first();
        let mut entries = Vec::new();
        for f in 0..4 {
            entries.push((input(0, face(f)), Relation::Kept(face(f))));
        }
        entries.push((input(0, edge(0)), Relation::Kept(edge(0))));
        let identity = ShapeHistory {
            entries,
            operand_counts: vec![[4, 1, 0]],
            ..ShapeHistory::default()
        };
        assert_eq!(h.compose(&identity), h);
    }

    #[test]
    fn composing_a_generated_image_with_a_split_stays_generated() {
        let a = ShapeHistory {
            entries: vec![(input(0, edge(0)), Relation::Generated(outs(&[face(0)])))],
            operand_counts: vec![[0, 1, 0]],
            ..ShapeHistory::default()
        };
        let b = ShapeHistory {
            entries: vec![(
                input(0, face(0)),
                Relation::Modified(outs(&[face(4), face(1)])),
            )],
            operand_counts: vec![[1, 0, 0]],
            ..ShapeHistory::default()
        };
        let c = a.compose(&b);
        assert_eq!(
            c.entries,
            vec![(
                input(0, edge(0)),
                Relation::Generated(outs(&[face(1), face(4)]))
            )]
        );
        assert_eq!(c.operand_counts, vec![[0, 1, 0]]);
    }

    #[test]
    fn split_key_orders_by_measure_then_centroid_then_index() {
        use amg_core::{Aabb3, Point3};
        let g = |measure: f64, x: f64, y: f64| SubShapeGeom {
            class: crate::geom::GeomClass::Plane,
            measure,
            centroid: Point3::new(x, y, 0.0),
            axis: None,
            bbox: Aabb3::EMPTY,
        };
        let big = split_order_key(&g(60.0, 5.0, 7.0), 4);
        let small = split_order_key(&g(20.0, 5.0, 1.0), 1);
        assert!(big < small, "area descending");
        // Equal within the quantum: the centroid decides, then the index.
        let left = split_order_key(&g(20.0, 1.0, 0.0), 9);
        let right = split_order_key(&g(20.0 + 1e-8, 2.0, 0.0), 0);
        assert!(left < right);
        let twin_a = split_order_key(&g(20.0, 1.0, 0.0), 2);
        let twin_b = split_order_key(&g(20.0, 1.0 + 1e-8, 0.0), 3);
        assert!(twin_a < twin_b);
        assert_eq!(twin_a.0, twin_b.0);
        assert_eq!(twin_a.1, twin_b.1);
    }

    #[test]
    fn unexplained_faces_and_ranges_are_reported() {
        let h = first();
        // B has faces 0..3 explained (face 3 by its Generated relation and its role); a fifth
        // face has nothing.
        assert!(h.unexplained_faces(4).is_empty());
        assert_eq!(h.unexplained_faces(5), vec![FaceId(4)]);
        let mut labelled = h.clone();
        labelled.roles.push((face(4), OpRole::Unexplained));
        assert_eq!(labelled.unexplained_faces(5), vec![FaceId(4)]);
        assert_eq!(h.range_error([4, 1, 0]), None);
        assert!(h.range_error([3, 1, 0]).unwrap().contains("Face"));
        assert!(h.range_error([4, 0, 0]).is_some());
        let mut bad_input = h.clone();
        bad_input.operand_counts = vec![[1, 1, 0]];
        assert!(bad_input.range_error([4, 1, 0]).unwrap().contains("input"));
        bad_input.operand_counts.clear();
        assert!(
            bad_input
                .range_error([4, 1, 0])
                .unwrap()
                .contains("operand")
        );
    }

    #[test]
    fn composed_histories_round_trip_through_serde() {
        let h = first().compose(&first());
        let json = serde_json::to_string(&h).unwrap();
        assert_eq!(serde_json::from_str::<ShapeHistory>(&json).unwrap(), h);
    }
}
