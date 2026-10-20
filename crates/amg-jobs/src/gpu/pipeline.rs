//! Compute-pipeline cache keyed by (WGSL source, entry point, override constants).

use std::collections::HashMap;
use std::sync::Arc;

use parking_lot::Mutex;

use super::GpuError;

/// Describes a compute pipeline for [`super::Gpu::compute_pipeline`]. The pipeline uses the
/// layout derived from the shader (`get_bind_group_layout(i)`).
#[derive(Clone, Copy, Debug)]
pub struct ComputePipelineDesc<'a> {
    /// Debug label (shader module and pipeline). Not part of the cache key.
    pub label: &'a str,
    /// WGSL source.
    pub wgsl: &'a str,
    /// Compute entry point.
    pub entry_point: &'a str,
    /// Values for WGSL `override` constants, by name or `@id`.
    pub constants: &'a [(&'a str, f64)],
}

impl<'a> ComputePipelineDesc<'a> {
    /// A desc without override constants.
    pub const fn new(label: &'a str, wgsl: &'a str, entry_point: &'a str) -> Self {
        ComputePipelineDesc {
            label,
            wgsl,
            entry_point,
            constants: &[],
        }
    }
}

/// Full key (not just its hash), so a hash collision can never return the wrong pipeline.
/// Constants are compared by bit pattern; their order is significant, as in wgpu (the last
/// value of a repeated name wins).
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct Key {
    wgsl: String,
    entry_point: String,
    constants: Vec<(String, u64)>,
}

impl Key {
    fn new(desc: &ComputePipelineDesc<'_>) -> Key {
        Key {
            wgsl: desc.wgsl.to_owned(),
            entry_point: desc.entry_point.to_owned(),
            constants: desc
                .constants
                .iter()
                .map(|&(name, v)| (name.to_owned(), v.to_bits()))
                .collect(),
        }
    }
}

#[derive(Debug, Default)]
pub(super) struct PipelineCache {
    map: Mutex<HashMap<Key, Arc<wgpu::ComputePipeline>>>,
}

impl PipelineCache {
    pub(super) fn get_or_create(
        &self,
        device: &wgpu::Device,
        desc: &ComputePipelineDesc<'_>,
    ) -> Result<Arc<wgpu::ComputePipeline>, GpuError> {
        let key = Key::new(desc);
        if let Some(p) = self.map.lock().get(&key) {
            return Ok(Arc::clone(p));
        }
        // Compile outside the lock; if another thread raced us, keep the first pipeline so
        // equal descs always share one `Arc`.
        let created = Arc::new(create(device, desc)?);
        Ok(Arc::clone(self.map.lock().entry(key).or_insert(created)))
    }
}

/// Creates the pipeline inside error scopes, so invalid WGSL becomes `GpuError::Shader`
/// instead of reaching the device's uncaptured-error handler (which panics by default).
fn create(
    device: &wgpu::Device,
    desc: &ComputePipelineDesc<'_>,
) -> Result<wgpu::ComputePipeline, GpuError> {
    // Error scopes are thread-local, so other threads' work is not captured here.
    let internal = device.push_error_scope(wgpu::ErrorFilter::Internal);
    let validation = device.push_error_scope(wgpu::ErrorFilter::Validation);
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some(desc.label),
        source: wgpu::ShaderSource::Wgsl(desc.wgsl.into()),
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some(desc.label),
        layout: None,
        module: &module,
        entry_point: Some(desc.entry_point),
        compilation_options: wgpu::PipelineCompilationOptions {
            constants: desc.constants,
            zero_initialize_workgroup_memory: true,
        },
        cache: None,
    });
    let validation = validation.pop();
    let internal = internal.pop();
    #[allow(
        clippy::disallowed_methods,
        reason = "native wgpu resolves error-scope futures immediately; nothing waits on the GPU"
    )]
    let error = pollster::block_on(async { validation.await.or(internal.await) });
    match error {
        None => Ok(pipeline),
        Some(e) => Err(GpuError::Shader {
            label: desc.label.to_owned(),
            message: e.to_string(),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn key_distinguishes_entry_point_and_constants() {
        let a = ComputePipelineDesc::new("a", "src", "main");
        let b = ComputePipelineDesc { label: "b", ..a };
        assert_eq!(Key::new(&a), Key::new(&b), "label is not part of the key");
        let c = ComputePipelineDesc::new("a", "src", "other");
        assert_ne!(Key::new(&a), Key::new(&c));
        let d = ComputePipelineDesc {
            constants: &[("n", 1.0)],
            ..a
        };
        let e = ComputePipelineDesc {
            constants: &[("n", 2.0)],
            ..a
        };
        assert_ne!(Key::new(&d), Key::new(&e));
        assert_ne!(Key::new(&a), Key::new(&d));
    }
}
