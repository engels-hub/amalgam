//! [`Bvh`]: binned-SAH bounding-volume hierarchy with 32-byte flattened nodes.
//!
//! Build follows Wald 2007 (binned SAH, 16 bins). Subtrees larger than [`PAR_MIN`] primitives
//! are built with `rayon::join`; the result does not depend on the thread count because every
//! split decision is a pure function of the primitives in the subtree (bin counts and bounds
//! combine with exact, order-independent min/max/integer-add).
//!
//! Layout: nodes are stored depth first. The left child of the inner node `i` is `i + 1`; the
//! right child index is stored in the node. Bounds are `f32`, rounded *outward*, so f64 tests
//! against them are conservative.

use amg_core::{Aabb3, Point3};
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::error::{MeshError, MeshResult};

/// Subtrees with at least this many primitives are built in parallel.
const PAR_MIN: usize = 16 * 1024;
/// Above this many primitives the binning pass itself is parallel.
const PAR_BIN_MIN: usize = 64 * 1024;
/// Nodes at this depth or deeper split at the median, which bounds the depth (and so the
/// traversal stack) at about `FORCE_MEDIAN_DEPTH + log2(n)`.
const FORCE_MEDIAN_DEPTH: u32 = 48;
/// Largest supported bin count.
const MAX_BINS: usize = 64;
/// Size of the fixed traversal stacks used by the queries.
pub(crate) const STACK: usize = 128;

/// Build options.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BvhOpts {
    /// Maximum primitives per leaf (default 4).
    pub max_leaf_size: u32,
    /// SAH bins per axis (default 16, `2..=64`).
    pub sah_bins: u32,
}

impl Default for BvhOpts {
    fn default() -> Self {
        Self {
            max_leaf_size: 4,
            sah_bins: 16,
        }
    }
}

/// 32-byte flattened node. `count == 0` marks an inner node (`left_or_first` is the right
/// child index, the left child is the next node); otherwise a leaf covering
/// `prim_indices[left_or_first .. left_or_first + count]`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct RawNode {
    pub min: [f32; 3],
    pub left_or_first: u32,
    pub max: [f32; 3],
    pub count: u32,
}

const _: () = assert!(size_of::<RawNode>() == 32);

impl RawNode {
    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.count != 0
    }

    #[inline]
    pub fn min64(&self) -> Point3 {
        Point3::new(
            f64::from(self.min[0]),
            f64::from(self.min[1]),
            f64::from(self.min[2]),
        )
    }

    #[inline]
    pub fn max64(&self) -> Point3 {
        Point3::new(
            f64::from(self.max[0]),
            f64::from(self.max[1]),
            f64::from(self.max[2]),
        )
    }
}

/// What a [`BvhNode`] holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BvhNodeKind {
    /// Inner node with two children (node indices).
    Inner {
        /// Left child index.
        left: u32,
        /// Right child index.
        right: u32,
    },
    /// Leaf covering `count` slots starting at `first` (see [`Bvh::prim`]).
    Leaf {
        /// First slot.
        first: u32,
        /// Number of slots.
        count: u32,
    },
}

/// Read-only view of a node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BvhNode {
    /// Bounds (f32 values rounded outward, widened to f64).
    pub bounds: Aabb3,
    /// Node kind.
    pub kind: BvhNodeKind,
}

/// Bounding-volume hierarchy over a set of primitive boxes.
#[derive(Clone, Debug)]
pub struct Bvh {
    pub(crate) nodes: Vec<RawNode>,
    pub(crate) prim_indices: Vec<u32>,
}

/// f32 value `<= v` (or `-inf` when `v` is below the f32 range).
#[inline]
fn round_down(v: f64) -> f32 {
    let f = v as f32;
    if f64::from(f) > v { f.next_down() } else { f }
}

/// f32 value `>= v`.
#[inline]
fn round_up(v: f64) -> f32 {
    let f = v as f32;
    if f64::from(f) < v { f.next_up() } else { f }
}

/// Primitive reference used during the build: outward f32 bounds and the original index.
#[derive(Clone, Copy)]
struct PrimRef {
    min: [f32; 3],
    max: [f32; 3],
    idx: u32,
}

impl PrimRef {
    #[inline]
    fn centroid(&self, axis: usize) -> f32 {
        0.5 * (self.min[axis] + self.max[axis])
    }
}

/// f32 box used while building.
#[derive(Clone, Copy)]
struct B3 {
    min: [f32; 3],
    max: [f32; 3],
}

impl B3 {
    const EMPTY: B3 = B3 {
        min: [f32::INFINITY; 3],
        max: [f32::NEG_INFINITY; 3],
    };

    #[inline]
    fn add(&mut self, min: &[f32; 3], max: &[f32; 3]) {
        for a in 0..3 {
            self.min[a] = self.min[a].min(min[a]);
            self.max[a] = self.max[a].max(max[a]);
        }
    }

    #[inline]
    fn union(mut self, o: &B3) -> B3 {
        self.add(&o.min, &o.max);
        self
    }

    /// Half the surface area (monotone in the area; the constant is irrelevant for SAH).
    #[inline]
    fn half_area(&self) -> f64 {
        if self.min[0] > self.max[0] {
            return 0.0;
        }
        let dx = f64::from(self.max[0]) - f64::from(self.min[0]);
        let dy = f64::from(self.max[1]) - f64::from(self.min[1]);
        let dz = f64::from(self.max[2]) - f64::from(self.min[2]);
        dx * dy + dy * dz + dz * dx
    }
}

#[derive(Clone, Copy)]
struct Bin {
    bounds: B3,
    count: u32,
}

impl Bin {
    const EMPTY: Bin = Bin {
        bounds: B3::EMPTY,
        count: 0,
    };
}

type Bins = [[Bin; MAX_BINS]; 3];

fn new_bins() -> Box<Bins> {
    Box::new([[Bin::EMPTY; MAX_BINS]; 3])
}

/// Bounds of the prims and of their centroids.
fn extents(prims: &[PrimRef]) -> (B3, B3) {
    let mut b = B3::EMPTY;
    let mut c = B3::EMPTY;
    for p in prims {
        b.add(&p.min, &p.max);
        let ct = [p.centroid(0), p.centroid(1), p.centroid(2)];
        c.add(&ct, &ct);
    }
    (b, c)
}

#[inline]
fn bin_of(c: f32, cmin: f32, scale: f32, bins: usize) -> usize {
    (((c - cmin) * scale) as usize).min(bins - 1)
}

fn bin_scale(cb: &B3, nbins: usize) -> [f32; 3] {
    let mut scale = [0.0f32; 3];
    for (a, s) in scale.iter_mut().enumerate() {
        let e = cb.max[a] - cb.min[a];
        *s = if e > 0.0 { nbins as f32 / e } else { 0.0 };
    }
    scale
}

fn bin_into(prims: &[PrimRef], cb: &B3, nbins: usize, out: &mut Bins) {
    let scale = bin_scale(cb, nbins);
    for p in prims {
        for a in 0..3 {
            let k = bin_of(p.centroid(a), cb.min[a], scale[a], nbins);
            let bin = &mut out[a][k];
            bin.bounds.add(&p.min, &p.max);
            bin.count += 1;
        }
    }
}

fn merge_bins(mut a: Box<Bins>, b: &Bins) -> Box<Bins> {
    for ax in 0..3 {
        for k in 0..MAX_BINS {
            a[ax][k].bounds = a[ax][k].bounds.union(&b[ax][k].bounds);
            a[ax][k].count += b[ax][k].count;
        }
    }
    a
}

/// Best SAH split `(axis, first bin of the right side)`; ties go to the lowest axis, then bin.
fn best_split(bins: &Bins, nbins: usize) -> Option<(usize, usize)> {
    let mut best: Option<(f64, usize, usize)> = None;
    for (a, axis_bins) in bins.iter().enumerate() {
        let mut right_area = [0.0f64; MAX_BINS];
        let mut right_cnt = [0u32; MAX_BINS];
        let mut acc = B3::EMPTY;
        let mut cnt = 0u32;
        for k in (1..nbins).rev() {
            acc = acc.union(&axis_bins[k].bounds);
            cnt += axis_bins[k].count;
            right_area[k] = acc.half_area();
            right_cnt[k] = cnt;
        }
        let mut acc = B3::EMPTY;
        let mut cnt = 0u32;
        for k in 1..nbins {
            acc = acc.union(&axis_bins[k - 1].bounds);
            cnt += axis_bins[k - 1].count;
            if cnt == 0 || right_cnt[k] == 0 {
                continue;
            }
            let cost = acc.half_area() * f64::from(cnt) + right_area[k] * f64::from(right_cnt[k]);
            if best.is_none_or(|(c, _, _)| cost < c) {
                best = Some((cost, a, k));
            }
        }
    }
    best.map(|(_, a, k)| (a, k))
}

/// Splits `prims` in place; returns the size of the left part (`1..len`).
fn split(prims: &mut [PrimRef], cb: &B3, depth: u32, opts: &BvhOpts) -> usize {
    let n = prims.len();
    let nbins = opts.sah_bins as usize;
    let degenerate = (0..3).all(|a| cb.max[a] <= cb.min[a]);
    if degenerate || depth >= FORCE_MEDIAN_DEPTH {
        return median_split(prims, cb);
    }
    let bins = if n >= PAR_BIN_MIN {
        prims
            .par_chunks(16 * 1024)
            .map(|c| {
                let mut b = new_bins();
                bin_into(c, cb, nbins, &mut b);
                b
            })
            .reduce_with(|a, b| merge_bins(a, &b))
            .unwrap_or_else(new_bins)
    } else {
        let mut b = new_bins();
        bin_into(prims, cb, nbins, &mut b);
        b
    };
    let Some((axis, k)) = best_split(&bins, nbins) else {
        return median_split(prims, cb);
    };
    let scale = bin_scale(cb, nbins)[axis];
    let cmin = cb.min[axis];
    let mid = partition(prims, |p| bin_of(p.centroid(axis), cmin, scale, nbins) < k);
    if mid == 0 || mid == n {
        median_split(prims, cb)
    } else {
        mid
    }
}

/// Median split on the longest centroid axis (by index when all centroids coincide).
fn median_split(prims: &mut [PrimRef], cb: &B3) -> usize {
    let n = prims.len();
    let mut axis = 0;
    let mut ext = cb.max[0] - cb.min[0];
    for a in 1..3 {
        let e = cb.max[a] - cb.min[a];
        if e > ext {
            ext = e;
            axis = a;
        }
    }
    if ext > 0.0 {
        // Deterministic: ties on the centroid are ordered by the original index.
        prims.select_nth_unstable_by(n / 2, |p, q| {
            p.centroid(axis)
                .total_cmp(&q.centroid(axis))
                .then(p.idx.cmp(&q.idx))
        });
    }
    n / 2
}

/// In-place partition; returns the number of elements for which `pred` holds (moved first).
fn partition(prims: &mut [PrimRef], pred: impl Fn(&PrimRef) -> bool) -> usize {
    let (mut i, mut j) = (0usize, prims.len());
    while i < j {
        if pred(&prims[i]) {
            i += 1;
        } else {
            j -= 1;
            prims.swap(i, j);
        }
    }
    i
}

struct Cx<'a> {
    opts: BvhOpts,
    job: &'a JobCtx,
}

fn raw(b: &B3, left_or_first: u32, count: u32) -> RawNode {
    RawNode {
        min: b.min,
        left_or_first,
        max: b.max,
        count,
    }
}

/// Sequential build of a subtree into `out`. `first` is the global slot of `prims[0]`; inner
/// nodes store indices relative to `out`.
fn build_seq(
    prims: &mut [PrimRef],
    first: usize,
    depth: u32,
    cx: &Cx<'_>,
    out: &mut Vec<RawNode>,
) -> MeshResult<()> {
    let n = prims.len();
    if n >= 256 {
        cx.job.checkpoint()?;
    }
    let (bounds, cb) = extents(prims);
    if n <= cx.opts.max_leaf_size as usize {
        out.push(raw(&bounds, first as u32, n as u32));
        return Ok(());
    }
    let mid = split(prims, &cb, depth, &cx.opts);
    let me = out.len();
    out.push(raw(&bounds, 0, 0));
    let (l, r) = prims.split_at_mut(mid);
    build_seq(l, first, depth + 1, cx, out)?;
    out[me].left_or_first = out.len() as u32;
    build_seq(r, first + mid, depth + 1, cx, out)
}

/// Parallel build; the returned nodes use indices relative to the returned vector.
fn build_par(
    prims: &mut [PrimRef],
    first: usize,
    depth: u32,
    cx: &Cx<'_>,
) -> MeshResult<Vec<RawNode>> {
    let n = prims.len();
    if n < PAR_MIN {
        let mut out = Vec::with_capacity(n / 2 + 1);
        build_seq(prims, first, depth, cx, &mut out)?;
        return Ok(out);
    }
    cx.job.checkpoint()?;
    let (bounds, cb) = extents(prims);
    let mid = split(prims, &cb, depth, &cx.opts);
    let (l, r) = prims.split_at_mut(mid);
    let (left, right) = rayon::join(
        || build_par(l, first, depth + 1, cx),
        || build_par(r, first + mid, depth + 1, cx),
    );
    let (left, right) = (left?, right?);
    let right_at = 1 + left.len();
    let mut out = Vec::with_capacity(1 + left.len() + right.len());
    out.push(raw(&bounds, right_at as u32, 0));
    out.extend(left.into_iter().map(|mut nd| {
        if nd.count == 0 {
            nd.left_or_first += 1;
        }
        nd
    }));
    out.extend(right.into_iter().map(|mut nd| {
        if nd.count == 0 {
            nd.left_or_first += right_at as u32;
        }
        nd
    }));
    Ok(out)
}

impl Bvh {
    /// Builds a BVH over `prim_bounds` (primitive `i` has box `prim_bounds[i]`).
    ///
    /// Deterministic: the result is bit-identical for any rayon thread count. Checks
    /// cancellation regularly; reports no progress (the build is sub-second at 1 M
    /// primitives).
    ///
    /// # Errors
    /// [`MeshError::TooLarge`] beyond the `u32` range, [`MeshError::InvalidOption`] for bad
    /// options or non-finite / empty boxes, [`MeshError::Cancelled`].
    pub fn build(prim_bounds: &[Aabb3], opts: &BvhOpts, ctx: &JobCtx) -> MeshResult<Self> {
        if opts.max_leaf_size == 0 {
            return Err(MeshError::InvalidOption {
                name: "max_leaf_size",
                reason: "must be at least 1".into(),
            });
        }
        if !(2..=MAX_BINS as u32).contains(&opts.sah_bins) {
            return Err(MeshError::InvalidOption {
                name: "sah_bins",
                reason: format!("must be in 2..={MAX_BINS}"),
            });
        }
        let n = prim_bounds.len();
        if n >= u32::MAX as usize / 2 {
            return Err(MeshError::TooLarge { count: n });
        }
        let to_ref = |(i, b): (usize, &Aabb3)| PrimRef {
            min: [
                round_down(b.min.x),
                round_down(b.min.y),
                round_down(b.min.z),
            ],
            max: [round_up(b.max.x), round_up(b.max.y), round_up(b.max.z)],
            idx: i as u32,
        };
        let bad = |p: &PrimRef| {
            (0..3).any(|a| !(p.min[a].is_finite() && p.max[a].is_finite() && p.min[a] <= p.max[a]))
        };
        let (mut prims, any_bad): (Vec<PrimRef>, bool) = if n >= PAR_MIN {
            let v: Vec<PrimRef> = prim_bounds.par_iter().enumerate().map(to_ref).collect();
            let bad_any = v.par_iter().any(bad);
            (v, bad_any)
        } else {
            let v: Vec<PrimRef> = prim_bounds.iter().enumerate().map(to_ref).collect();
            let bad_any = v.iter().any(bad);
            (v, bad_any)
        };
        if any_bad {
            return Err(MeshError::InvalidOption {
                name: "prim_bounds",
                reason: "boxes must be finite and non-empty".into(),
            });
        }
        let cx = Cx {
            opts: *opts,
            job: ctx,
        };
        let nodes = if n == 0 {
            Vec::new()
        } else {
            build_par(&mut prims, 0, 0, &cx)?
        };
        let prim_indices = prims.iter().map(|p| p.idx).collect();
        Ok(Self {
            nodes,
            prim_indices,
        })
    }

    /// Number of nodes (0 for an empty set).
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Number of primitives.
    pub fn prim_count(&self) -> usize {
        self.prim_indices.len()
    }

    /// Root node index (always 0); `None` when empty.
    pub fn root(&self) -> Option<u32> {
        (!self.nodes.is_empty()).then_some(0)
    }

    /// Read-only view of node `i`.
    ///
    /// # Panics
    /// If `i >= node_count()`.
    pub fn node(&self, i: u32) -> BvhNode {
        let n = &self.nodes[i as usize];
        BvhNode {
            bounds: Aabb3 {
                min: n.min64(),
                max: n.max64(),
            },
            kind: if n.is_leaf() {
                BvhNodeKind::Leaf {
                    first: n.left_or_first,
                    count: n.count,
                }
            } else {
                BvhNodeKind::Inner {
                    left: i + 1,
                    right: n.left_or_first,
                }
            },
        }
    }

    /// Primitive index stored in leaf slot `slot`.
    pub fn prim(&self, slot: u32) -> u32 {
        self.prim_indices[slot as usize]
    }

    /// Visits every primitive of every leaf whose (outward-rounded) bounds overlap `b`. This
    /// is conservative at primitive level: callers filter with their exact test. The order is
    /// deterministic (depth first, left before right).
    pub fn query_aabb(&self, b: &Aabb3, visit: &mut dyn FnMut(u32)) {
        if self.nodes.is_empty() || b.is_empty() {
            return;
        }
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let i = stack[sp];
            let n = &self.nodes[i as usize];
            if n.min64().cmpgt(b.max).any() || n.max64().cmplt(b.min).any() {
                continue;
            }
            if n.is_leaf() {
                let f = n.left_or_first as usize;
                for s in f..f + n.count as usize {
                    visit(self.prim_indices[s]);
                }
            } else {
                stack[sp] = n.left_or_first;
                stack[sp + 1] = i + 1;
                sp += 2;
            }
        }
    }

    /// Maximum node depth (root = 1); 0 when empty. For tests and diagnostics.
    pub fn depth(&self) -> usize {
        fn go(b: &Bvh, i: u32) -> usize {
            match b.node(i).kind {
                BvhNodeKind::Leaf { .. } => 1,
                BvhNodeKind::Inner { left, right } => 1 + go(b, left).max(go(b, right)),
            }
        }
        if self.nodes.is_empty() {
            0
        } else {
            go(self, 0)
        }
    }
}
