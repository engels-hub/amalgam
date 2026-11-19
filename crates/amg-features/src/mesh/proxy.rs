//! Preview proxies (S07 UI, MESH-23): in [`ComputeMode::Preview`] a mesh feature whose input
//! has more than [`PREVIEW_PROXY_FACES`] faces runs on a QEM-decimated copy of about
//! [`PREVIEW_PROXY_TARGET`] faces, so an interactive preview of a large scan stays fast. Full
//! computes (OK in the dialog, recompute, export) always see the real mesh.
//!
//! The proxy of the last few inputs is kept (by content hash), so changing a dialog parameter
//! does not decimate again. The limits are a preference (UI-11); tests lower them.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use amg_doc::{ComputeMode, FeatureError, RecomputeCtx};
use amg_mesh::{DecimateOpts, DecimateTarget, IndexedMesh};

use super::base::with_job;

/// Inputs with more faces than this are previewed on a proxy (S07: "proxy > 2 M tris").
pub const PREVIEW_PROXY_FACES: usize = 2_000_000;
/// Face count the proxy is decimated to.
pub const PREVIEW_PROXY_TARGET: usize = 500_000;
/// Proxies kept for reuse.
const CACHED_PROXIES: usize = 2;

static THRESHOLD: AtomicUsize = AtomicUsize::new(PREVIEW_PROXY_FACES);
static TARGET: AtomicUsize = AtomicUsize::new(PREVIEW_PROXY_TARGET);
/// `(input content hash, input face count, proxy)`, newest last.
static CACHE: Mutex<Vec<(u64, usize, Arc<IndexedMesh>)>> = Mutex::new(Vec::new());

/// Changes the proxy limits: previews of inputs above `threshold` faces run on a proxy of
/// about `target` faces (`target` is clamped to `4..=threshold`). Process-wide; for the
/// preferences and for tests. Previews already in a document's preview cache keep their
/// result (the cache key does not include the limits).
pub fn set_preview_proxy_limits(threshold: usize, target: usize) {
    THRESHOLD.store(threshold, Ordering::Relaxed);
    TARGET.store(target.clamp(4, threshold.max(4)), Ordering::Relaxed);
    if let Ok(mut c) = CACHE.lock() {
        c.clear();
    }
}

/// The current `(threshold, target)` proxy limits.
pub fn preview_proxy_limits() -> (usize, usize) {
    (
        THRESHOLD.load(Ordering::Relaxed),
        TARGET.load(Ordering::Relaxed),
    )
}

/// True if a preview of a mesh with `faces` faces runs on a proxy.
pub fn uses_preview_proxy(faces: usize) -> bool {
    faces > THRESHOLD.load(Ordering::Relaxed)
}

/// The mesh a feature computes on: `mesh` itself, or its proxy in a preview of a large input.
/// A decimation failure other than cancellation falls back to the full mesh.
pub(super) fn for_compute(
    ctx: &RecomputeCtx<'_>,
    mesh: Arc<IndexedMesh>,
) -> Result<Arc<IndexedMesh>, FeatureError> {
    if ctx.mode() != ComputeMode::Preview || !uses_preview_proxy(mesh.face_count()) {
        return Ok(mesh);
    }
    let hash = mesh.content_hash();
    let n = mesh.face_count();
    if let Some(p) = cached(hash, n) {
        return Ok(p);
    }
    let opts = DecimateOpts {
        target: DecimateTarget::FaceCount(TARGET.load(Ordering::Relaxed)),
        ..DecimateOpts::default()
    };
    match with_job(ctx, |job| amg_mesh::decimate(&mesh, &opts, job)) {
        Ok((proxy, _)) => {
            let proxy = Arc::new(proxy);
            if let Ok(mut c) = CACHE.lock() {
                c.retain(|(h, f, _)| (*h, *f) != (hash, n));
                c.push((hash, n, Arc::clone(&proxy)));
                let excess = c.len().saturating_sub(CACHED_PROXIES);
                c.drain(..excess);
            }
            Ok(proxy)
        }
        Err(e) if amg_core::Diagnostic::is_cancelled(&e) => Err(FeatureError::Cancelled),
        // E.g. a mesh QEM cannot collapse: preview the full mesh instead.
        Err(_) => Ok(mesh),
    }
}

fn cached(hash: u64, faces: usize) -> Option<Arc<IndexedMesh>> {
    let c = CACHE.lock().ok()?;
    c.iter()
        .find(|(h, f, _)| (*h, *f) == (hash, faces))
        .map(|(_, _, p)| Arc::clone(p))
}
