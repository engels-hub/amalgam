//! Op results and their history data (S02 §6).
//!
//! BREP-03 lands the data types of the Interfaces, because `make_wire` returns an [`OpResult`]
//! with per-edge tags. BREP-05 adds the extraction from OCCT, `compose` and the completeness
//! guarantees.

use smallvec::SmallVec;

use crate::geom::Tag;
use crate::shape::{Shape, SubShape};

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

/// A face built by a face builder, with the tag of each created edge.
#[derive(Clone, Debug)]
pub struct FaceBuild {
    /// The face.
    pub face: Shape,
    /// `(edge, tag)` for every edge made from a tagged input curve, in input order. Empty for
    /// bounds without tags (`Natural`, `UvRect`, `Polylines`).
    pub edge_tags: Vec<(crate::EdgeId, Tag)>,
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
}
