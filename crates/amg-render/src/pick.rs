//! GPU picking (RND-04, design note §7): request and result types, the CPU resolution of a
//! point pick's id region, the per-pick-id flag tables of rectangle selection and the readbacks
//! in flight.
//!
//! A pick renders an id pass (`shaders/pick.wgsl`) into an `Rg32Uint` id target (pick id,
//! `kind << 30 | sub`) and an `R32Float` depth target on the next [`crate::Renderer::encode`]
//! of its view:
//! - a **point** pick is scissored to the `(2r + 1)²` square around the cursor, with the
//!   projection shifted by the cursor's sub-pixel offset so the centre texel samples the exact
//!   cursor ray. Faces, then edges, then points are drawn with the depth test; the CPU then
//!   takes the nearest vertex within `r`, else the nearest edge within `r - 1`, else the face
//!   under the cursor ([`resolve_point`]), and reconstructs the world point in f64;
//! - a **rectangle** renders the whole view, and a compute pass ORs 1 (inside) or 2 (outside)
//!   into per-entity flags (`shaders/pick_rect.wgsl`): window = only inside, crossing = some
//!   inside.

use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};

use amg_core::{Point3, Vec3};

use crate::camera::Camera;
use crate::section::{SectionPlane, section_cap_point};
use crate::{CAP_SUB, InstanceKey, PickId, RenderError, ViewId};

/// Sub-id in a [`PickResult::Rect`] entry that stands for the whole pick owner (filter
/// [`PickFilter::BODIES`]).
pub const BODY_SUB: u32 = u32::MAX;

/// Mask of the sub-id bits of an id texel (`kind << 30 | sub`).
pub(crate) const SUB_MASK: u32 = (1 << 30) - 1;

/// Which pick-shader path reports mesh triangle indices (B-rep meshes always use their face-id
/// attribute).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PickBackend {
    /// `PrimitiveIndex` when the device has `PRIMITIVE_INDEX`, else `VertexPulling`.
    #[default]
    Auto,
    /// `@builtin(primitive_index)` in the fragment stage (falls back to vertex pulling, with a
    /// warning, when the device lacks the feature).
    PrimitiveIndex,
    /// A non-indexed draw that reads indices and positions from storage buffers and passes
    /// `vertex_index / 3`.
    VertexPulling,
}

/// Handle of a pick request; [`crate::Renderer::poll_picks`] returns each exactly once.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PickTicket(pub u64);

/// What to pick and what may be hit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PickRequest {
    /// Point or rectangle.
    pub region: PickRegion,
    /// Which kinds are drawn into the pick pass and reported.
    pub filter: PickFilter,
}

impl PickRequest {
    /// A point pick at `px` (physical pixels, origin top-left, y down) with every kind.
    pub fn point(px: [f32; 2]) -> PickRequest {
        PickRequest {
            region: PickRegion::Point { px },
            filter: PickFilter::ALL,
        }
    }

    /// A rectangle pick of the texels `min..max` (exclusive max).
    pub fn rect(min: [u32; 2], max: [u32; 2], mode: RectMode, filter: PickFilter) -> PickRequest {
        PickRequest {
            region: PickRegion::Rect { min, max, mode },
            filter,
        }
    }
}

/// Where to pick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PickRegion {
    /// Under the cursor, within `RendererConfig::pick_radius_px`.
    Point {
        /// Cursor position, physical pixels (origin top-left, y down; pixel centres at `.5`).
        px: [f32; 2],
    },
    /// Rectangle selection of the texels `min..max`.
    Rect {
        /// Top-left texel (inclusive).
        min: [u32; 2],
        /// Bottom-right texel (exclusive).
        max: [u32; 2],
        /// Window or crossing.
        mode: RectMode,
    },
}

/// Rectangle selection semantics. Only visible texels count (occluded entities are never
/// selected).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RectMode {
    /// Entities whose every visible texel lies inside the rectangle.
    Window,
    /// Entities with at least one visible texel inside the rectangle.
    Crossing,
}

/// Kinds a pick draws and reports (a bit set). Kinds left out are not drawn, so they do not
/// hide others: without `EDGES`, a click on an edge reports the face under it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct PickFilter(u8);

impl PickFilter {
    /// Whole pick owners (rectangle results with [`BODY_SUB`]; point picks report the face).
    pub const BODIES: PickFilter = PickFilter(1);
    /// Faces (B-rep faces, mesh triangles, section caps).
    pub const FACES: PickFilter = PickFilter(2);
    /// Edges.
    pub const EDGES: PickFilter = PickFilter(4);
    /// Vertices (points).
    pub const VERTICES: PickFilter = PickFilter(8);
    /// Overlay handles (RND-07).
    pub const HANDLES: PickFilter = PickFilter(16);
    /// Everything.
    pub const ALL: PickFilter = PickFilter(31);
    /// Nothing.
    pub const NONE: PickFilter = PickFilter(0);

    /// The raw bits.
    pub fn bits(self) -> u8 {
        self.0
    }

    /// Whether every kind of `other` is in the set.
    pub fn contains(self, other: PickFilter) -> bool {
        self.0 & other.0 == other.0
    }

    /// Whether the sets share a kind.
    pub fn intersects(self, other: PickFilter) -> bool {
        self.0 & other.0 != 0
    }

    /// Faces are drawn (bodies or faces asked for).
    pub(crate) fn draws_faces(self) -> bool {
        self.intersects(PickFilter::BODIES.union(PickFilter::FACES))
    }

    /// The union of both sets.
    #[must_use]
    pub const fn union(self, other: PickFilter) -> PickFilter {
        PickFilter(self.0 | other.0)
    }
}

impl std::ops::BitOr for PickFilter {
    type Output = PickFilter;
    fn bitor(self, rhs: PickFilter) -> PickFilter {
        self.union(rhs)
    }
}

/// The kind of a pick sub-id.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SubKind {
    /// A B-rep face, a mesh triangle or a section cap ([`CAP_SUB`]).
    Face,
    /// An edge (`Polyline::id`).
    Edge,
    /// A vertex (`PointData::ids`).
    Vertex,
    /// An overlay handle (RND-07).
    Handle,
}

impl SubKind {
    pub(crate) fn from_code(code: u32) -> SubKind {
        match code >> 30 {
            0 => SubKind::Face,
            1 => SubKind::Edge,
            2 => SubKind::Vertex,
            _ => SubKind::Handle,
        }
    }

    /// Resolution rank: handles, then vertices, then edges, then faces.
    fn rank(self) -> u8 {
        match self {
            SubKind::Handle => 0,
            SubKind::Vertex => 1,
            SubKind::Edge => 2,
            SubKind::Face => 3,
        }
    }

    fn in_filter(self, filter: PickFilter) -> bool {
        match self {
            SubKind::Face => filter.draws_faces(),
            SubKind::Edge => filter.contains(PickFilter::EDGES),
            SubKind::Vertex => filter.contains(PickFilter::VERTICES),
            SubKind::Handle => filter.contains(PickFilter::HANDLES),
        }
    }
}

/// One hit of a point pick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PickHit {
    /// The pick owner.
    pub pick: PickId,
    /// The instance with this pick id when the result was polled (`None` if it is gone).
    pub instance: Option<InstanceKey>,
    /// What was hit.
    pub kind: SubKind,
    /// The sub-id: face id or triangle index ([`CAP_SUB`] for a section cap), edge id, vertex id.
    pub sub: u32,
    /// Reversed-Z depth of the hit texel (for a cap, of the cap point on its plane).
    pub depth: f32,
    /// World position, f64 mm: the surface point under the hit texel, the nearest point of an
    /// edge's centre line, a vertex's centre, or for a cap the entry of the texel's ray into the
    /// kept region ([`section_cap_point`], exact in f64).
    pub world: Point3,
    /// Distance in pixels from the cursor to the hit texel (0 for the face under the cursor).
    pub distance_px: f32,
}

/// The answer to a [`PickRequest`].
#[derive(Clone, Debug, PartialEq)]
pub enum PickResult {
    /// Best hit first (handle > vertex > edge > face, then distance, then nearer), then the other
    /// distinct hits that qualify: vertices and handles within the radius, edges within the
    /// radius minus one pixel, and the face under the cursor. Empty: nothing there.
    Point(Vec<PickHit>),
    /// Selected `(pick, kind, sub)`, sorted; whole owners use `(pick, SubKind::Face, BODY_SUB)`.
    Rect(Vec<(PickId, SubKind, u32)>),
}

/// One qualifying hit of [`resolve_point`] (offset in texels from the cursor texel).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Candidate {
    /// Index of the texel in the region.
    pub texel: usize,
    pub pick: PickId,
    pub kind: SubKind,
    pub sub: u32,
    pub depth: f32,
    pub offset: [i32; 2],
    pub distance: f32,
}

/// Resolves a point pick from its id region (`ids[y * width + x]` = (pick, code), `depths` the
/// same layout); `centre` is the cursor texel inside the region. Vertices and handles count
/// within `radius` px, edges within `radius - 1` (at least the cursor texel), faces only under
/// the cursor. Returns the distinct qualifying hits, best first.
pub(crate) fn resolve_point(
    ids: &[[u32; 2]],
    depths: &[f32],
    width: usize,
    centre: [usize; 2],
    radius: u32,
    filter: PickFilter,
) -> Vec<Candidate> {
    let r = radius as f32;
    let r_edge = r.max(1.0) - 1.0;
    let mut found: Vec<Candidate> = Vec::new();
    for (i, (&[pick, code], &depth)) in ids.iter().zip(depths).enumerate() {
        if pick == 0 {
            continue;
        }
        let dx = (i % width) as i32 - centre[0] as i32;
        let dy = (i / width) as i32 - centre[1] as i32;
        let distance = ((dx * dx + dy * dy) as f32).sqrt();
        let kind = SubKind::from_code(code);
        let limit = match kind {
            SubKind::Handle | SubKind::Vertex => r,
            SubKind::Edge => r_edge,
            SubKind::Face => 0.0,
        };
        if distance > limit + 1e-4 || !kind.in_filter(filter) {
            continue;
        }
        let c = Candidate {
            texel: i,
            pick: PickId(pick),
            kind,
            sub: code & SUB_MASK,
            depth,
            offset: [dx, dy],
            distance,
        };
        match found
            .iter_mut()
            .find(|f| (f.pick, f.kind, f.sub) == (c.pick, c.kind, c.sub))
        {
            Some(f) => {
                if (c.distance, -c.depth) < (f.distance, -f.depth) {
                    *f = c;
                }
            }
            None => found.push(c),
        }
    }
    found.sort_by(|a, b| {
        a.kind
            .rank()
            .cmp(&b.kind.rank())
            .then(a.distance.total_cmp(&b.distance))
            .then(b.depth.total_cmp(&a.depth))
            .then(a.pick.cmp(&b.pick))
            .then(a.sub.cmp(&b.sub))
    });
    found
}

/// Flags of one kind of one pick owner in a rectangle pick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct RectEntry {
    pub pick: PickId,
    /// `None` = the whole owner.
    pub kind: Option<SubKind>,
    pub base: u32,
    pub count: u32,
}

/// The GPU table (8 words per pick id: base and count of the body, face, edge and vertex
/// flags) and the CPU entries of a rectangle pick.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct RectTable {
    pub table: Vec<u32>,
    pub entries: Vec<RectEntry>,
    /// Number of flag words.
    pub flags: u32,
}

/// Sizes of one pick owner's sub-id spaces (faces, edges, vertices).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct OwnerSizes {
    pub faces: u32,
    pub edges: u32,
    pub vertices: u32,
}

/// Builds the flag table for `owners` (pick id → sizes; ids of 0 are skipped) under `filter`.
/// Owners listed twice keep the larger sizes.
pub(crate) fn build_rect_table(owners: &[(PickId, OwnerSizes)], filter: PickFilter) -> RectTable {
    let mut merged: Vec<(PickId, OwnerSizes)> = Vec::with_capacity(owners.len());
    let mut sorted: Vec<_> = owners.iter().filter(|(p, _)| p.0 != 0).copied().collect();
    sorted.sort_by_key(|(p, _)| *p);
    for (p, s) in sorted {
        match merged.last_mut() {
            Some((q, t)) if *q == p => {
                t.faces = t.faces.max(s.faces);
                t.edges = t.edges.max(s.edges);
                t.vertices = t.vertices.max(s.vertices);
            }
            _ => merged.push((p, s)),
        }
    }
    let picks = merged.last().map_or(1, |(p, _)| p.0 as usize + 1);
    let mut out = RectTable {
        table: vec![0; picks * 8],
        entries: Vec::new(),
        flags: 0,
    };
    for (pick, sizes) in merged {
        let row = pick.0 as usize * 8;
        let kinds = [
            (None, 1, filter.contains(PickFilter::BODIES)),
            (
                Some(SubKind::Face),
                sizes.faces,
                filter.contains(PickFilter::FACES),
            ),
            (
                Some(SubKind::Edge),
                sizes.edges,
                filter.contains(PickFilter::EDGES),
            ),
            (
                Some(SubKind::Vertex),
                sizes.vertices,
                filter.contains(PickFilter::VERTICES),
            ),
        ];
        for (k, (kind, count, wanted)) in kinds.into_iter().enumerate() {
            if !wanted || count == 0 {
                continue;
            }
            out.table[row + 2 * k] = out.flags;
            out.table[row + 2 * k + 1] = count;
            out.entries.push(RectEntry {
                pick,
                kind,
                base: out.flags,
                count,
            });
            out.flags += count;
        }
    }
    out
}

/// The selected entities of a rectangle pick from its flags (1 = a texel inside, 2 = a texel
/// outside), sorted.
pub(crate) fn rect_selection(
    entries: &[RectEntry],
    flags: &[u32],
    mode: RectMode,
) -> Vec<(PickId, SubKind, u32)> {
    let mut out = Vec::new();
    for e in entries {
        for i in 0..e.count {
            let f = flags.get((e.base + i) as usize).copied().unwrap_or(0);
            let selected = match mode {
                RectMode::Window => f == 1,
                RectMode::Crossing => f & 1 != 0,
            };
            if selected {
                out.push(match e.kind {
                    None => (e.pick, SubKind::Face, BODY_SUB),
                    Some(kind) => (e.pick, kind, i),
                });
            }
        }
    }
    out.sort_unstable();
    out
}

pub(crate) const MAP_PENDING: u8 = 0;
pub(crate) const MAP_DONE: u8 = 1;
pub(crate) const MAP_FAILED: u8 = 2;

/// A pick waiting for its view's next `encode`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct QueuedPick {
    pub ticket: PickTicket,
    pub view: ViewId,
    pub request: PickRequest,
}

/// What a recorded point pick needs to resolve its region.
#[derive(Clone, Debug)]
pub(crate) struct PointReadback {
    /// Cursor, physical px.
    pub px: [f64; 2],
    /// Region size and the cursor texel inside it.
    pub width: u32,
    pub height: u32,
    pub centre: [u32; 2],
    pub radius: u32,
    /// Byte offset of the position rows in the staging buffer; rows are `ids_row` / `pos_row`
    /// bytes (padded to `COPY_BYTES_PER_ROW_ALIGNMENT`).
    pub pos_offset: u64,
    pub ids_row: u32,
    pub pos_row: u32,
    pub camera: Camera,
    pub size: [u32; 2],
    pub sections: Vec<SectionPlane>,
}

/// What a recorded rectangle pick needs to read its flags.
#[derive(Clone, Debug)]
pub(crate) struct RectReadback {
    pub entries: Vec<RectEntry>,
    pub mode: RectMode,
}

#[derive(Clone, Debug)]
pub(crate) enum ReadbackKind {
    Point(PointReadback),
    Rect(RectReadback),
    /// Nothing to read (no flags): the result is ready.
    Empty(PickResult),
}

/// A pick recorded into an encoder whose staging buffer maps once the work is submitted.
#[derive(Debug)]
pub(crate) struct InFlight {
    pub ticket: PickTicket,
    pub filter: PickFilter,
    pub staging: Option<wgpu::Buffer>,
    pub state: Arc<AtomicU8>,
    pub kind: ReadbackKind,
}

impl InFlight {
    /// Whether the readback finished (mapped or failed).
    pub fn ready(&self) -> bool {
        self.staging.is_none() || self.state.load(Ordering::Acquire) != MAP_PENDING
    }

    /// Reads the mapped staging buffer and resolves the result. `instance` maps a pick id to
    /// its instance.
    pub fn finish(
        self,
        instance: &dyn Fn(PickId) -> Option<InstanceKey>,
    ) -> Result<PickResult, RenderError> {
        let bytes = match &self.staging {
            None => Vec::new(),
            Some(staging) => {
                if self.state.load(Ordering::Acquire) != MAP_DONE {
                    return Err(RenderError::InvalidData(
                        "the pick readback could not be mapped".into(),
                    ));
                }
                let bytes = staging
                    .get_mapped_range(..)
                    .map_err(|e| RenderError::InvalidData(format!("pick readback: {e}")))?
                    .to_vec();
                staging.unmap();
                bytes
            }
        };
        match self.kind {
            ReadbackKind::Empty(r) => Ok(r),
            ReadbackKind::Rect(rect) => {
                let flags: Vec<u32> = bytemuck::pod_collect_to_vec(&bytes);
                Ok(PickResult::Rect(rect_selection(
                    &rect.entries,
                    &flags,
                    rect.mode,
                )))
            }
            ReadbackKind::Point(p) => {
                let (w, h) = (p.width as usize, p.height as usize);
                let mut ids = Vec::with_capacity(w * h);
                let mut positions: Vec<[f32; 4]> = Vec::with_capacity(w * h);
                for y in 0..h {
                    let row = &bytes[y * p.ids_row as usize..][..w * 8];
                    ids.extend(
                        bytemuck::pod_collect_to_vec::<u8, u32>(row)
                            .chunks_exact(2)
                            .map(|c| [c[0], c[1]]),
                    );
                    let row = &bytes[p.pos_offset as usize + y * p.pos_row as usize..][..w * 16];
                    positions.extend(
                        bytemuck::pod_collect_to_vec::<u8, f32>(row)
                            .chunks_exact(4)
                            .map(|c| [c[0], c[1], c[2], c[3]]),
                    );
                }
                let depths: Vec<f32> = positions.iter().map(|v| v[3]).collect();
                let centre = [p.centre[0] as usize, p.centre[1] as usize];
                let hits = resolve_point(&ids, &depths, w, centre, p.radius, self.filter)
                    .into_iter()
                    .map(|c| point_hit(&p, &c, positions[c.texel], instance))
                    .collect();
                Ok(PickResult::Point(hits))
            }
        }
    }
}

/// The world position of a candidate: the eye plus the eye-relative point its texel stored
/// (surface point, nearest centre-line point of an edge, centre of a vertex), or for a cap the
/// f64 entry of the texel's ray into the kept region ([`section_cap_point`]).
fn point_hit(
    p: &PointReadback,
    c: &Candidate,
    pos: [f32; 4],
    instance: &dyn Fn(PickId) -> Option<InstanceKey>,
) -> PickHit {
    let px = [
        p.px[0] + f64::from(c.offset[0]),
        p.px[1] + f64::from(c.offset[1]),
    ];
    let cap = (c.kind == SubKind::Face && c.sub == CAP_SUB)
        .then(|| section_cap_point(&p.sections, &p.camera.ray(px, p.size)))
        .flatten();
    let rte = Vec3::new(f64::from(pos[0]), f64::from(pos[1]), f64::from(pos[2]));
    let world = match cap {
        Some((_, point)) => point,
        None => p.camera.eye + rte,
    };
    PickHit {
        pick: c.pick,
        instance: instance(c.pick),
        kind: c.kind,
        sub: c.sub,
        depth: c.depth,
        world,
        distance_px: c.distance,
    }
}

/// The cursor texel (clamped to the view) and the view-pixel origin of the `(2 r + 1)²` square
/// centred on it (negative or past the view at its borders; those texels stay empty).
pub(crate) fn point_region(px: [f64; 2], radius: u32, size: [u32; 2]) -> ([u32; 2], [i64; 2]) {
    let cx = (px[0].floor().max(0.0) as u32).min(size[0].saturating_sub(1));
    let cy = (px[1].floor().max(0.0) as u32).min(size[1].saturating_sub(1));
    let r = i64::from(radius);
    ([cx, cy], [i64::from(cx) - r, i64::from(cy) - r])
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
#[allow(clippy::float_cmp, reason = "exact texel distances")]
mod tests {
    use super::*;

    const FACE: u32 = 0;
    const EDGE: u32 = 1 << 30;
    const VERTEX: u32 = 2 << 30;

    /// An 11×11 region (radius 5) filled with face 9 of pick 1, cursor at the centre.
    fn region() -> (Vec<[u32; 2]>, Vec<f32>) {
        (vec![[1, FACE | 9]; 121], vec![0.5; 121])
    }

    fn set(ids: &mut [[u32; 2]], x: usize, y: usize, v: [u32; 2]) {
        ids[y * 11 + x] = v;
    }

    #[test]
    fn the_face_under_the_cursor_wins_without_edges_or_vertices() {
        let (ids, depths) = region();
        let hits = resolve_point(&ids, &depths, 11, [5, 5], 5, PickFilter::ALL);
        assert_eq!(hits.len(), 1);
        assert_eq!(
            (hits[0].pick, hits[0].kind, hits[0].sub),
            (PickId(1), SubKind::Face, 9)
        );
        assert_eq!(hits[0].distance, 0.0);
    }

    #[test]
    fn vertex_beats_edge_beats_face_within_their_radii() {
        let (mut ids, depths) = region();
        // Edge 3 four texels to the right (within r - 1 = 4), vertex 2 five up (within r = 5).
        set(&mut ids, 9, 5, [1, EDGE | 3]);
        set(&mut ids, 5, 0, [1, VERTEX | 2]);
        let hits = resolve_point(&ids, &depths, 11, [5, 5], 5, PickFilter::ALL);
        let kinds: Vec<_> = hits.iter().map(|h| (h.kind, h.sub)).collect();
        assert_eq!(
            kinds,
            [(SubKind::Vertex, 2), (SubKind::Edge, 3), (SubKind::Face, 9)]
        );
        assert_eq!(hits[0].offset, [0, -5]);
        // An edge exactly r px away does not count (edges need r - 1).
        let (mut ids, _) = region();
        set(&mut ids, 10, 5, [1, EDGE | 3]);
        let hits = resolve_point(&ids, &depths, 11, [5, 5], 5, PickFilter::ALL);
        assert!(hits.iter().all(|h| h.kind != SubKind::Edge));
    }

    #[test]
    fn nearest_texel_of_each_edge_and_nearest_edge_first() {
        let (mut ids, depths) = region();
        for x in 0..11 {
            set(&mut ids, x, 8, [2, EDGE | 4]);
            set(&mut ids, x, 3, [2, EDGE | 5]);
        }
        let hits = resolve_point(&ids, &depths, 11, [5, 5], 5, PickFilter::ALL);
        assert_eq!(hits[0].sub, 5, "two texels away beats three");
        assert_eq!(hits[0].distance, 2.0);
        assert_eq!(hits[1].sub, 4);
        assert_eq!(hits[1].distance, 3.0);
        assert_eq!(hits.len(), 3);
    }

    #[test]
    fn filter_drops_kinds_and_background_gives_nothing() {
        let (mut ids, depths) = region();
        set(&mut ids, 6, 5, [1, EDGE | 3]);
        let faces = resolve_point(&ids, &depths, 11, [5, 5], 5, PickFilter::FACES);
        assert_eq!(faces.len(), 1);
        assert_eq!(faces[0].kind, SubKind::Face);
        let empty = vec![[0, 0]; 121];
        assert!(resolve_point(&empty, &depths, 11, [5, 5], 5, PickFilter::ALL).is_empty());
        // A face next to the cursor (not under it) is not a hit.
        let mut ids = empty;
        set(&mut ids, 6, 5, [1, FACE | 1]);
        assert!(resolve_point(&ids, &depths, 11, [5, 5], 5, PickFilter::ALL).is_empty());
    }

    #[test]
    fn equal_distance_prefers_the_nearer_depth() {
        let (mut ids, mut depths) = region();
        set(&mut ids, 7, 5, [3, VERTEX | 1]);
        set(&mut ids, 3, 5, [4, VERTEX | 1]);
        depths[5 * 11 + 3] = 0.9;
        let hits = resolve_point(&ids, &depths, 11, [5, 5], 5, PickFilter::ALL);
        assert_eq!(
            hits[0].pick,
            PickId(4),
            "reversed-Z: larger depth is nearer"
        );
    }

    #[test]
    fn point_region_is_centred_on_the_clamped_cursor_texel() {
        assert_eq!(
            point_region([50.5, 60.5], 5, [100, 100]),
            ([50, 60], [45, 55])
        );
        assert_eq!(point_region([1.2, 0.0], 5, [100, 100]), ([1, 0], [-4, -5]));
        assert_eq!(
            point_region([99.9, 99.9], 5, [100, 100]),
            ([99, 99], [94, 94])
        );
        assert_eq!(
            point_region([-3.0, 250.0], 2, [100, 100]),
            ([0, 99], [-2, 97])
        );
    }

    #[test]
    fn rect_tables_and_window_vs_crossing() {
        let owners = [
            (
                PickId(2),
                OwnerSizes {
                    faces: 6,
                    edges: 12,
                    vertices: 8,
                },
            ),
            (
                PickId(5),
                OwnerSizes {
                    faces: 6,
                    edges: 0,
                    vertices: 0,
                },
            ),
            (PickId(0), OwnerSizes::default()),
        ];
        let t = build_rect_table(&owners, PickFilter::BODIES | PickFilter::FACES);
        assert_eq!(t.table.len(), 6 * 8);
        assert_eq!(t.flags, 1 + 6 + 1 + 6);
        assert_eq!(&t.table[16..20], &[0, 1, 1, 6]);
        assert_eq!(&t.table[40..44], &[7, 1, 8, 6]);
        assert_eq!(t.table[16 + 5], 0, "edges not asked for");
        let mut flags = vec![0u32; t.flags as usize];
        flags[0] = 1; // body 2: only inside
        flags[1 + 2] = 3; // its face 2: inside and outside
        flags[7] = 3; // body 5: crossing
        flags[8 + 4] = 1;
        let window = rect_selection(&t.entries, &flags, RectMode::Window);
        assert_eq!(
            window,
            [
                (PickId(2), SubKind::Face, BODY_SUB),
                (PickId(5), SubKind::Face, 4)
            ]
        );
        let crossing = rect_selection(&t.entries, &flags, RectMode::Crossing);
        assert_eq!(crossing.len(), 4);
        assert!(crossing.contains(&(PickId(5), SubKind::Face, BODY_SUB)));
        assert!(crossing.contains(&(PickId(2), SubKind::Face, 2)));
    }

    #[test]
    fn filters_compose() {
        let f = PickFilter::FACES | PickFilter::EDGES;
        assert!(f.contains(PickFilter::EDGES));
        assert!(!f.contains(PickFilter::VERTICES));
        assert!(f.draws_faces());
        assert!(!PickFilter::EDGES.draws_faces());
        assert!(PickFilter::BODIES.draws_faces());
        assert_eq!(PickFilter::ALL.bits(), 31);
        assert_eq!(SubKind::from_code(EDGE | 7), SubKind::Edge);
        assert_eq!(SubKind::from_code(3 << 30), SubKind::Handle);
    }
}
