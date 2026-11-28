//! Robustness and diagnostics (RND-13): the uncaptured-error log, VRAM accounting by category
//! and the data of the diagnostics overlay.

use std::collections::HashSet;
use std::sync::{Arc, Mutex, PoisonError};

use crate::{FrameStats, GpuTimings};

/// Distinct error messages remembered (and logged) before further ones are only counted. Some
/// messages embed sizes or ids, so an error that repeats every frame must not grow without end.
const MAX_DISTINCT_ERRORS: usize = 64;

/// Routes the errors of a device that no error scope catches to `tracing`, each distinct message
/// once (with the labels wgpu puts in it), instead of wgpu's default handler, which panics.
///
/// Install it with [`crate::Renderer::capture_errors`] (`egui_glue::install` does). Error
/// scopes ([`wgpu::Device::push_error_scope`]) still take their errors first. A device has one
/// handler: whatever is installed last receives the errors. Clones share the counters.
#[derive(Clone, Debug, Default)]
pub struct GpuErrorLog {
    state: Arc<Mutex<ErrorState>>,
}

#[derive(Debug, Default)]
struct ErrorState {
    seen: HashSet<String>,
    total: u64,
}

impl GpuErrorLog {
    /// Makes this log the uncaptured-error handler of `device`.
    pub fn install(device: &wgpu::Device) -> GpuErrorLog {
        let log = GpuErrorLog::default();
        let handler = log.clone();
        device.on_uncaptured_error(Arc::new(move |e: wgpu::Error| {
            let kind = match e {
                wgpu::Error::Validation { .. } => "validation",
                wgpu::Error::OutOfMemory { .. } => "out of memory",
                wgpu::Error::Internal { .. } => "internal",
            };
            handler.record(kind, &e.to_string());
        }));
        log
    }

    /// Errors received, repeats included.
    pub fn total(&self) -> u64 {
        self.lock().total
    }

    /// Distinct messages logged (at most 64; later ones are only counted).
    pub fn distinct(&self) -> usize {
        self.lock().seen.len()
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, ErrorState> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn record(&self, kind: &'static str, message: &str) {
        let (first, last_slot) = {
            let mut state = self.lock();
            state.total += 1;
            let first =
                state.seen.len() < MAX_DISTINCT_ERRORS && state.seen.insert(message.to_owned());
            (first, first && state.seen.len() == MAX_DISTINCT_ERRORS)
        };
        // Logged outside the lock: a subscriber may take its time.
        if first {
            tracing::error!(kind, "GPU error: {message}");
        }
        if last_slot {
            tracing::warn!(
                "{MAX_DISTINCT_ERRORS} distinct GPU errors logged; further ones are only counted"
            );
        }
    }
}

/// VRAM of the [`crate::RenderScene`]'s resources by category: the sums of their `gpu_bytes`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SceneVram {
    /// Meshes ([`crate::GpuMesh::gpu_bytes`]).
    pub meshes: u64,
    /// Edge sets ([`crate::GpuEdges::gpu_bytes`]).
    pub edges: u64,
    /// Point sets ([`crate::GpuPoints::gpu_bytes`]).
    pub points: u64,
    /// Scalar and vector fields ([`crate::GpuField::gpu_bytes`]).
    pub fields: u64,
    /// Overlay images ([`crate::GpuImage::gpu_bytes`]).
    pub images: u64,
}

impl SceneVram {
    /// All scene resources ([`crate::RenderScene::gpu_bytes`]).
    pub fn total(&self) -> u64 {
        self.meshes + self.edges + self.points + self.fields + self.images
    }
}

/// VRAM a [`crate::Renderer`] holds, by category, as requested from wgpu (driver padding,
/// pipelines and shaders are not included, nor are egui's resources, the caller's external
/// layer, or the buffers and targets that live only while a capture or pick is in flight).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VramUsage {
    /// The scene's resources.
    pub scene: SceneVram,
    /// Offscreen colour, depth and normal targets of the views.
    pub view_targets: u64,
    /// The point-pick targets the views keep.
    pub pick_targets: u64,
    /// The scene-wide instance, line, analysis and highlight-state buffers.
    pub instance_buffers: u64,
    /// Per-view buffers: uniforms, visibility lists, overlay geometry and timers.
    pub view_buffers: u64,
    /// The colormap LUT and other fixed resources.
    pub other: u64,
}

impl VramUsage {
    /// Every category with its label, in display order; their sum is [`VramUsage::total`].
    pub fn categories(&self) -> [(&'static str, u64); 10] {
        [
            ("meshes", self.scene.meshes),
            ("edges", self.scene.edges),
            ("points", self.scene.points),
            ("fields", self.scene.fields),
            ("images", self.scene.images),
            ("view targets", self.view_targets),
            ("pick targets", self.pick_targets),
            ("instance buffers", self.instance_buffers),
            ("view buffers", self.view_buffers),
            ("other", self.other),
        ]
    }

    /// All categories.
    pub fn total(&self) -> u64 {
        self.categories().iter().map(|&(_, b)| b).sum()
    }
}

/// What the diagnostics overlay shows for one view ([`crate::Renderer::diagnostics`]).
#[derive(Clone, Debug, Default)]
pub struct Diagnostics {
    /// Adapter name.
    pub adapter: String,
    /// Graphics backend.
    pub backend: String,
    /// What the view's last successful [`crate::Renderer::encode`] drew.
    pub frame: FrameStats,
    /// Pass timings of the newest finished frame; `None` without `TIMESTAMP_QUERY` or before
    /// the first frame finishes.
    pub timings: Option<GpuTimings>,
    /// VRAM by category.
    pub vram: VramUsage,
    /// Errors the renderer's [`GpuErrorLog`] received (0 when none is installed).
    pub gpu_errors: u64,
}

impl Diagnostics {
    /// The overlay's text lines.
    pub fn lines(&self) -> Vec<String> {
        let mut out = vec![
            format!("{} ({})", self.adapter, self.backend),
            format!(
                "triangles {}  instances {}  culled {}",
                self.frame.triangles, self.frame.instances_drawn, self.frame.culled
            ),
            format!("edge segments {}", self.frame.edge_segments),
        ];
        match &self.timings {
            Some(t) => {
                out.push(format!("GPU {:.3} ms", t.total_ms));
                out.extend(
                    t.passes
                        .iter()
                        .map(|(name, ms)| format!("  {name} {ms:.3} ms")),
                );
            }
            None => out.push("GPU timings unavailable".to_owned()),
        }
        out.push(format!("VRAM {}", mib(self.vram.total())));
        out.extend(
            self.vram
                .categories()
                .iter()
                .filter(|(_, bytes)| *bytes > 0)
                .map(|&(name, bytes)| format!("  {name} {}", mib(bytes))),
        );
        if self.gpu_errors > 0 {
            out.push(format!("GPU errors {}", self.gpu_errors));
        }
        out
    }
}

/// `bytes` in MiB with two decimals.
fn mib(bytes: u64) -> String {
    format!("{:.2} MiB", bytes as f64 / (1024.0 * 1024.0))
}

/// Bytes of a single-mip 2D (array) texture of an uncompressed format, all samples.
pub(crate) fn texture_bytes(t: &wgpu::Texture) -> u64 {
    // Depth formats have no copy size without an aspect; the renderer's are 32-bit.
    let texel = u64::from(t.format().block_copy_size(None).unwrap_or(4));
    u64::from(t.width())
        * u64::from(t.height())
        * u64::from(t.depth_or_array_layers())
        * u64::from(t.sample_count())
        * texel
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_log_counts_repeats_and_bounds_distinct_messages() {
        let log = GpuErrorLog::default();
        log.record("validation", "a");
        log.record("validation", "a");
        log.record("validation", "b");
        assert_eq!((log.total(), log.distinct()), (3, 2));
        for i in 0..200 {
            log.record("validation", &format!("buffer {i}"));
        }
        assert_eq!((log.total(), log.distinct()), (203, MAX_DISTINCT_ERRORS));
    }

    #[test]
    fn categories_sum_to_the_total_and_list_in_the_lines() {
        let vram = VramUsage {
            scene: SceneVram {
                meshes: 1 << 20,
                edges: 2,
                points: 3,
                fields: 4,
                images: 5,
            },
            view_targets: 6,
            pick_targets: 7,
            instance_buffers: 8,
            view_buffers: 9,
            other: 10,
        };
        assert_eq!(vram.scene.total(), (1 << 20) + 14);
        assert_eq!(vram.total(), (1 << 20) + 54);
        let d = Diagnostics {
            vram,
            timings: Some(GpuTimings {
                total_ms: 1.5,
                passes: vec![("opaque", 1.0), ("final", 0.5)],
            }),
            frame: FrameStats {
                triangles: 12,
                ..FrameStats::default()
            },
            gpu_errors: 2,
            ..Diagnostics::default()
        };
        let text = d.lines().join("\n");
        for want in [
            "triangles 12",
            "opaque 1.000 ms",
            "final 0.500 ms",
            "VRAM 1.00 MiB",
            "meshes 1.00 MiB",
            "view targets",
            "GPU errors 2",
        ] {
            assert!(text.contains(want), "{want:?} in {text}");
        }
        let none = Diagnostics::default().lines().join("\n");
        assert!(none.contains("GPU timings unavailable"), "{none}");
        assert!(!none.contains("GPU errors"), "{none}");
    }
}
