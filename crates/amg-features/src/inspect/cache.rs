//! Per-mesh derived data for measurement: the `MeshSpatial` BVH and the "closed and
//! consistently oriented" flag.
//!
//! S07 plans `MeshBody::spatial()` (a `OnceLock` on the body); until it exists this module keeps
//! a small process-wide cache keyed by the mesh's `Arc` address. An entry is dropped once the
//! cache holds the only reference to its mesh, so meshes are not kept alive.

use std::sync::{Arc, Mutex, OnceLock, PoisonError, Weak};

use amg_jobs::JobCtx;
use amg_mesh::{Bvh, BvhNodeKind, BvhOpts, IndexedMesh, MeshBody, MeshSpatial};
use rayon::prelude::*;

use super::InspectError;

/// Derived data of one mesh.
#[derive(Debug)]
pub(crate) struct Derived {
    pub(crate) spatial: MeshSpatial,
    watertight: OnceLock<bool>,
    min_prims: OnceLock<Vec<u32>>,
}

impl Derived {
    /// Smallest primitive (face) id under each BVH node.
    pub(crate) fn min_prims(&self) -> &[u32] {
        self.min_prims
            .get_or_init(|| node_min_prims(self.spatial.bvh()))
    }

    /// The mesh is closed and every edge is used once in each direction.
    pub(crate) fn watertight(&self, ctx: &JobCtx) -> Result<bool, InspectError> {
        if let Some(w) = self.watertight.get() {
            return Ok(*w);
        }
        let w = closed_oriented(self.spatial.mesh(), ctx)?;
        Ok(*self.watertight.get_or_init(|| w))
    }
}

static CACHE: Mutex<Vec<Arc<Derived>>> = Mutex::new(Vec::new());

/// Derived data of `body`, built on first use.
pub(crate) fn derived(body: &MeshBody, ctx: &JobCtx) -> Result<Arc<Derived>, InspectError> {
    let mesh = body.mesh();
    let find = |c: &[Arc<Derived>]| {
        c.iter()
            .find(|d| Arc::ptr_eq(d.spatial.mesh(), mesh))
            .cloned()
    };
    if let Some(d) = find(&CACHE.lock().unwrap_or_else(PoisonError::into_inner)) {
        return Ok(d);
    }
    // Built outside the lock; a concurrent build of the same mesh just loses the race.
    let spatial = MeshSpatial::build(Arc::clone(mesh), &BvhOpts::default(), ctx)?;
    let built = Arc::new(Derived {
        spatial,
        watertight: OnceLock::new(),
        min_prims: OnceLock::new(),
    });
    let mut c = CACHE.lock().unwrap_or_else(PoisonError::into_inner);
    // Drop entries whose mesh is referenced only by the cache (the entry's own spatial).
    c.retain(|d| Arc::strong_count(d.spatial.mesh()) > 1 || Arc::strong_count(d) > 1);
    if let Some(d) = find(&c) {
        return Ok(d);
    }
    c.push(Arc::clone(&built));
    Ok(built)
}

/// Closed-and-oriented flags of meshes that have no [`Derived`] entry (section caps need only
/// the flag, not a BVH). Keyed by the mesh `Arc`; dead entries are dropped on insert.
static ORIENTED: Mutex<Vec<(Weak<IndexedMesh>, bool)>> = Mutex::new(Vec::new());

/// Whether `body`'s mesh is closed and every edge is used once in each direction, cached per
/// mesh. Reuses the measurement cache's flag when that entry exists.
pub(crate) fn oriented(body: &MeshBody, ctx: &JobCtx) -> Result<bool, InspectError> {
    let mesh = body.mesh();
    let existing = CACHE
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .iter()
        .find(|d| Arc::ptr_eq(d.spatial.mesh(), mesh))
        .cloned();
    if let Some(d) = existing {
        return d.watertight(ctx);
    }
    let find = |c: &[(Weak<IndexedMesh>, bool)]| {
        c.iter()
            .find(|(w, _)| std::ptr::eq(w.as_ptr(), Arc::as_ptr(mesh)))
            .map(|&(_, f)| f)
    };
    if let Some(f) = find(&ORIENTED.lock().unwrap_or_else(PoisonError::into_inner)) {
        return Ok(f);
    }
    let flag = closed_oriented(mesh, ctx)?;
    let mut c = ORIENTED.lock().unwrap_or_else(PoisonError::into_inner);
    c.retain(|(w, _)| w.strong_count() > 0);
    if find(&c).is_none() {
        c.push((Arc::downgrade(mesh), flag));
    }
    Ok(flag)
}

/// Every directed edge appears exactly once and its reverse exactly once.
fn closed_oriented(m: &IndexedMesh, ctx: &JobCtx) -> Result<bool, InspectError> {
    if m.tris.is_empty() {
        return Ok(false);
    }
    let mut edges: Vec<u64> = m
        .tris
        .par_iter()
        .flat_map_iter(|t| {
            [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]
                .into_iter()
                .map(|(a, b)| (u64::from(a) << 32) | u64::from(b))
        })
        .collect();
    ctx.checkpoint()?;
    edges.par_sort_unstable();
    ctx.checkpoint()?;
    if edges.windows(2).any(|w| w[0] == w[1]) {
        return Ok(false);
    }
    Ok(edges.par_iter().all(|&e| {
        let rev = e.rotate_right(32);
        e >> 32 != e & 0xffff_ffff && edges.binary_search(&rev).is_ok()
    }))
}

/// Smallest primitive id under each node of `bvh` (indexed by node).
pub(crate) fn node_min_prims(bvh: &Bvh) -> Vec<u32> {
    let mut out = vec![u32::MAX; bvh.node_count()];
    let Some(root) = bvh.root() else {
        return out;
    };
    // Post-order with an explicit stack: (node, children done).
    let mut stack = vec![(root, false)];
    while let Some((n, done)) = stack.pop() {
        match bvh.node(n).kind {
            BvhNodeKind::Leaf { first, count } => {
                out[n as usize] = (first..first + count)
                    .map(|s| bvh.prim(s))
                    .min()
                    .unwrap_or(u32::MAX);
            }
            BvhNodeKind::Inner { left, right } => {
                if done {
                    out[n as usize] = out[left as usize].min(out[right as usize]);
                } else {
                    stack.push((n, true));
                    stack.push((left, false));
                    stack.push((right, false));
                }
            }
        }
    }
    out
}
