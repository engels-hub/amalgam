//! Hover and selection highlight (RND-04, design note §7).
//!
//! - A whole pick owner (`sub == None`) is tinted through its instance record (`flags`), and its
//!   edges and points take the highlight colour.
//! - Faces and mesh triangles are redrawn after the opaque meshes with the same vertex stage and
//!   `depth_compare: Equal` (no depth write), so the highlight lands on exactly the pixels the
//!   face drew, without z-fighting. B-rep faces map to their triangles through the mesh's
//!   [`crate::FaceRange`]s.
//! - Edges and points are tinted through a state buffer indexed by their sub-id (one `u32` of
//!   layer bits per edge and per point of a highlighted instance).
//!
//! The highlight colour replaces the base colour (shading still applies). Precedence:
//! Error > Selected > Hover > Preview; Selected together with Hover is drawn lighter.

use std::collections::HashMap;
use std::ops::Range;

use crate::upload::FaceRange;
use crate::{PickId, SubKind};

/// Highlight layers, in the order of their bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum HighlightLayer {
    /// Under the cursor.
    Hover,
    /// In the selection.
    Selected,
    /// Failed or invalid (drawn over everything else).
    Error,
    /// Previewed by the active tool.
    Preview,
}

impl HighlightLayer {
    /// All layers.
    pub const ALL: [HighlightLayer; 4] = [
        HighlightLayer::Hover,
        HighlightLayer::Selected,
        HighlightLayer::Error,
        HighlightLayer::Preview,
    ];

    /// Bit of the layer (`HL_*` in `shaders/scene.wgsl`).
    pub(crate) fn bit(self) -> u32 {
        1 << (self as u32)
    }

    /// Default colour (linear RGBA).
    pub(crate) fn default_color(self) -> [f32; 4] {
        match self {
            HighlightLayer::Hover => [0.32, 0.62, 1.0, 1.0],
            HighlightLayer::Selected => [0.05, 0.36, 1.0, 1.0],
            HighlightLayer::Error => [1.0, 0.12, 0.08, 1.0],
            HighlightLayer::Preview => [0.95, 0.55, 0.1, 1.0],
        }
    }
}

/// Precedence of a set of layer bits: the redraw order (higher drawn last).
fn precedence(bits: u32) -> u8 {
    if bits & HighlightLayer::Error.bit() != 0 {
        3
    } else if bits & HighlightLayer::Selected.bit() != 0 {
        2
    } else if bits & HighlightLayer::Hover.bit() != 0 {
        1
    } else {
        0
    }
}

/// Half-open sub-id range (a B-rep face `f..f+1`, a run of mesh triangles, one edge).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Range32 {
    /// First sub-id.
    pub start: u32,
    /// One past the last.
    pub end: u32,
}

impl Range32 {
    /// `start..end`.
    pub fn new(start: u32, end: u32) -> Range32 {
        Range32 { start, end }
    }

    /// The single sub-id `id`.
    pub fn one(id: u32) -> Range32 {
        Range32 {
            start: id,
            end: id.saturating_add(1),
        }
    }
}

/// One highlighted thing: a whole pick owner (`sub == None`) or a run of its sub-ids.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct HighlightItem {
    /// The pick owner.
    pub pick: PickId,
    /// Faces / triangles, edges or vertices; `None` = the whole owner.
    pub sub: Option<(SubKind, Range32)>,
}

/// Edge and point state regions larger than this are not allocated (the ids are not
/// highlighted).
const MAX_STATE_REGION: u32 = 1 << 24;

/// What the resolution needs to know about one instance slot.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct SlotGeometry<'a> {
    pub pick: PickId,
    /// Triangle count and B-rep face ranges of its mesh (`None` without a live mesh).
    pub mesh: Option<(u32, &'a [FaceRange])>,
    /// One past the largest edge id / vertex id of its edge and point sets.
    pub edge_ids: u32,
    pub point_ids: u32,
}

/// A face-range redraw: triangles `tris` of the instance in `slot`, with layer `bits`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct FaceHighlight {
    pub slot: u32,
    pub bits: u32,
    pub tris: Range<u32>,
}

/// The highlight of a scene: per slot `[flags, states index + 1]`, the edge and point state
/// buffer and the face redraws (ordered by precedence, highest last).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct Resolved {
    pub slots: Vec<[u32; 2]>,
    pub states: Vec<u32>,
    pub faces: Vec<FaceHighlight>,
}

/// Resolves the highlight layers against the instance slots.
pub(crate) fn resolve(slots: &[SlotGeometry<'_>], layers: &[Vec<HighlightItem>; 4]) -> Resolved {
    let mut out = Resolved {
        slots: vec![[0, 0]; slots.len()],
        ..Resolved::default()
    };
    if layers.iter().all(Vec::is_empty) {
        return out;
    }
    let mut by_pick: HashMap<PickId, Vec<usize>> = HashMap::new();
    for (i, s) in slots.iter().enumerate() {
        if s.pick.0 != 0 {
            by_pick.entry(s.pick).or_default().push(i);
        }
    }
    let mut faces: HashMap<(u32, u32, u32), u32> = HashMap::new();
    for (layer, items) in HighlightLayer::ALL.iter().zip(layers) {
        let bit = layer.bit();
        for item in items {
            let Some(found) = by_pick.get(&item.pick) else {
                continue;
            };
            for &slot in found {
                let geometry = slots[slot];
                match item.sub {
                    None => out.slots[slot][0] |= bit,
                    Some((SubKind::Face, range)) => {
                        let Some((triangles, ranges)) = geometry.mesh else {
                            continue;
                        };
                        let mut add = |tris: Range<u32>| {
                            if tris.start < tris.end {
                                *faces
                                    .entry((slot as u32, tris.start, tris.end))
                                    .or_default() |= bit;
                            }
                        };
                        if ranges.is_empty() {
                            add(range.start.min(triangles)..range.end.min(triangles));
                        } else {
                            for r in ranges
                                .iter()
                                .filter(|r| r.face >= range.start && r.face < range.end)
                            {
                                let end = r.first_tri.saturating_add(r.tri_count).min(triangles);
                                add(r.first_tri.min(end)..end);
                            }
                        }
                    }
                    Some((kind @ (SubKind::Edge | SubKind::Vertex), range)) => {
                        let (edges, points) = (geometry.edge_ids, geometry.point_ids);
                        if edges > MAX_STATE_REGION || points > MAX_STATE_REGION {
                            continue;
                        }
                        if out.slots[slot][1] == 0 {
                            out.slots[slot][1] = out.states.len() as u32 + 1;
                            out.states.push(edges);
                            out.states.push(points);
                            out.states
                                .extend(std::iter::repeat_n(0, (edges + points) as usize));
                        }
                        let base = out.slots[slot][1] as usize - 1 + 2;
                        let (offset, len) = if kind == SubKind::Edge {
                            (0, edges)
                        } else {
                            (edges as usize, points)
                        };
                        for id in range.start.min(len)..range.end.min(len) {
                            out.states[base + offset + id as usize] |= bit;
                        }
                    }
                    Some((SubKind::Handle, _)) => {}
                }
            }
        }
    }
    out.faces = faces
        .into_iter()
        .map(|((slot, start, end), bits)| FaceHighlight {
            slot,
            bits,
            tris: start..end,
        })
        .collect();
    out.faces.sort_by(|a, b| {
        precedence(a.bits)
            .cmp(&precedence(b.bits))
            .then(a.slot.cmp(&b.slot))
            .then(a.tris.start.cmp(&b.tris.start))
            .then(a.tris.end.cmp(&b.tris.end))
    });
    out
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
mod tests {
    use super::*;

    fn layers(items: [Vec<HighlightItem>; 4]) -> [Vec<HighlightItem>; 4] {
        items
    }

    #[test]
    fn whole_owner_sets_flags_on_every_instance_of_the_pick() {
        let slots = [
            SlotGeometry {
                pick: PickId(3),
                ..Default::default()
            },
            SlotGeometry {
                pick: PickId(4),
                ..Default::default()
            },
            SlotGeometry {
                pick: PickId(3),
                ..Default::default()
            },
        ];
        let item = HighlightItem {
            pick: PickId(3),
            sub: None,
        };
        let r = resolve(&slots, &layers([vec![item], vec![item], vec![], vec![]]));
        assert_eq!(r.slots, [[3, 0], [0, 0], [3, 0]]);
        assert!(r.faces.is_empty() && r.states.is_empty());
    }

    #[test]
    fn brep_faces_map_to_their_triangles_and_precedence_orders_redraws() {
        let ranges = [
            FaceRange {
                first_tri: 0,
                tri_count: 4,
                face: 0,
            },
            FaceRange {
                first_tri: 4,
                tri_count: 2,
                face: 1,
            },
            FaceRange {
                first_tri: 6,
                tri_count: 6,
                face: 2,
            },
        ];
        let slots = [SlotGeometry {
            pick: PickId(1),
            mesh: Some((12, &ranges)),
            ..Default::default()
        }];
        let face = |f| HighlightItem {
            pick: PickId(1),
            sub: Some((SubKind::Face, Range32::one(f))),
        };
        let r = resolve(
            &slots,
            &layers([vec![face(1), face(2)], vec![face(1)], vec![face(0)], vec![]]),
        );
        let got: Vec<_> = r.faces.iter().map(|f| (f.bits, f.tris.clone())).collect();
        // Hover-only face 2, then face 1 hovered and selected, then the error face last.
        assert_eq!(got, [(1, 6..12), (3, 4..6), (4, 0..4)]);
    }

    #[test]
    fn plain_mesh_ranges_are_triangles_clamped_to_the_mesh() {
        let slots = [SlotGeometry {
            pick: PickId(1),
            mesh: Some((10, &[])),
            ..Default::default()
        }];
        let item = HighlightItem {
            pick: PickId(1),
            sub: Some((SubKind::Face, Range32::new(8, 20))),
        };
        let r = resolve(&slots, &layers([vec![], vec![item], vec![], vec![]]));
        assert_eq!(
            r.faces,
            [FaceHighlight {
                slot: 0,
                bits: 2,
                tris: 8..10
            }]
        );
    }

    #[test]
    fn edge_and_point_states_get_one_region_per_instance() {
        let slots = [
            SlotGeometry {
                pick: PickId(1),
                edge_ids: 4,
                point_ids: 3,
                ..Default::default()
            },
            SlotGeometry {
                pick: PickId(2),
                edge_ids: 2,
                point_ids: 0,
                ..Default::default()
            },
        ];
        let edge = |p, e| HighlightItem {
            pick: PickId(p),
            sub: Some((SubKind::Edge, Range32::one(e))),
        };
        let vertex = HighlightItem {
            pick: PickId(1),
            sub: Some((SubKind::Vertex, Range32::new(1, 9))),
        };
        let r = resolve(
            &slots,
            &layers([
                vec![edge(1, 2), vertex],
                vec![edge(1, 2), edge(2, 1)],
                vec![],
                vec![],
            ]),
        );
        assert_eq!(r.slots, [[0, 1], [0, 10]]);
        assert_eq!(r.states, [4, 3, 0, 0, 3, 0, 0, 1, 1, 2, 0, 0, 2]);
    }

    #[test]
    fn unknown_picks_and_empty_layers_resolve_to_nothing() {
        let slots = [SlotGeometry {
            pick: PickId(1),
            ..Default::default()
        }];
        let r = resolve(&slots, &Default::default());
        assert_eq!(r.slots, [[0, 0]]);
        let item = HighlightItem {
            pick: PickId(9),
            sub: None,
        };
        let r = resolve(&slots, &layers([vec![item], vec![], vec![], vec![]]));
        assert_eq!(r.slots, [[0, 0]]);
    }
}
