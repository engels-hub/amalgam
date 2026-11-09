//! Compiled kernels and the WGSL prelude.

use std::sync::Arc;

use amg_jobs::gpu::{ComputePipelineDesc, Gpu, wgpu};

use crate::precision::{Precision, device_precision};
use crate::{ComputeError, ComputeResult};

const PRELUDE_F32: &str = concat!("alias real = f32;\n", include_str!("prelude.wgsl"));
const PRELUDE_F64: &str = concat!("alias real = f64;\n", include_str!("prelude.wgsl"));

/// Default of the `WG` override in the prelude.
const DEFAULT_WG: u32 = 256;

/// `amg_linear_id(gid, nwg) -> u32`, the `WG` override, `alias real = f32;` (or `f64`) and shared
/// constants. Numeric kernels write constants as `real(…)` conversions, because f64 literals
/// have their own suffix.
pub fn prelude_wgsl(precision: Precision) -> &'static str {
    match precision {
        Precision::F32 => PRELUDE_F32,
        Precision::F64 => PRELUDE_F64,
    }
}

/// A compiled kernel: pipeline + workgroup size (from the `WG` override, default 256).
#[derive(Clone, Debug)]
pub struct Kernel {
    pub(super) pipeline: Arc<wgpu::ComputePipeline>,
    workgroup: [u32; 3],
    pub(super) label: Arc<str>,
    precision: Precision,
}

/// The value of the `WG` override; like wgpu, the last value of a repeated name wins.
fn workgroup_x(label: &str, constants: &[(&str, f64)]) -> ComputeResult<u32> {
    let Some(&(_, value)) = constants.iter().rev().find(|(name, _)| *name == "WG") else {
        return Ok(DEFAULT_WG);
    };
    let wg = value as u32; // saturating; the round trip below rejects anything inexact
    if wg == 0 || f64::from(wg).to_bits() != value.to_bits() {
        return Err(ComputeError::Config(format!(
            "kernel `{label}`: WG = {value} is not a positive integer"
        )));
    }
    Ok(wg)
}

impl Kernel {
    /// Prepends `prelude_wgsl(Precision::F32)` to `wgsl`; cached through `Gpu::compute_pipeline`.
    ///
    /// # Errors
    /// As [`Kernel::with_precision`].
    pub fn new(
        gpu: &Gpu,
        label: &str,
        wgsl: &str,
        entry: &str,
        constants: &[(&str, f64)],
    ) -> ComputeResult<Self> {
        Self::with_precision(gpu, label, wgsl, entry, constants, Precision::F32)
    }

    /// Same with the prelude for `precision` (`alias real = f64;` for F64). The pipeline cache
    /// keys on the full source, so both variants of one kernel are cached side by side.
    ///
    /// # Errors
    /// [`ComputeError::Unsupported`] for F64 on a device without `SHADER_F64`,
    /// [`ComputeError::Config`] when `WG` is not a positive integer,
    /// [`ComputeError::Gpu`] when the kernel does not compile or the device is lost.
    pub fn with_precision(
        gpu: &Gpu,
        label: &str,
        wgsl: &str,
        entry: &str,
        constants: &[(&str, f64)],
        precision: Precision,
    ) -> ComputeResult<Self> {
        if precision == Precision::F64 && device_precision(gpu) != Precision::F64 {
            return Err(ComputeError::Unsupported(format!(
                "kernel `{label}` needs f64, but adapter `{}` ({:?}) has no SHADER_F64",
                gpu.info().name,
                gpu.info().backend
            )));
        }
        let wg = workgroup_x(label, constants)?;
        let source = format!("{}\n{wgsl}", prelude_wgsl(precision));
        let pipeline = gpu.compute_pipeline(&ComputePipelineDesc {
            label,
            wgsl: &source,
            entry_point: entry,
            constants,
        })?;
        Ok(Kernel {
            pipeline,
            workgroup: [wg, 1, 1],
            label: Arc::from(label),
            precision,
        })
    }

    /// The precision the kernel was compiled for.
    pub fn precision(&self) -> Precision {
        self.precision
    }

    /// Threads per workgroup along each axis.
    pub fn workgroup(&self) -> [u32; 3] {
        self.workgroup
    }

    /// `entries[i]` is bound at `@binding(i)` of `@group(group)`.
    pub fn bind_group(
        &self,
        gpu: &Gpu,
        group: u32,
        entries: &[wgpu::BindingResource<'_>],
    ) -> wgpu::BindGroup {
        let entries: Vec<wgpu::BindGroupEntry<'_>> = entries
            .iter()
            .enumerate()
            .map(|(i, resource)| wgpu::BindGroupEntry {
                binding: i as u32,
                resource: resource.clone(),
            })
            .collect();
        gpu.device().create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(&self.label),
            layout: &self.pipeline.get_bind_group_layout(group),
            entries: &entries,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prelude_binds_real_per_precision() {
        assert!(prelude_wgsl(Precision::F32).starts_with("alias real = f32;\n"));
        assert!(prelude_wgsl(Precision::F64).starts_with("alias real = f64;\n"));
        for p in [Precision::F32, Precision::F64] {
            let s = prelude_wgsl(p);
            assert!(s.contains("fn amg_linear_id(gid: vec3<u32>, nwg: vec3<u32>) -> u32"));
            assert!(s.contains("override WG: u32 = 256;"));
        }
    }

    #[test]
    fn workgroup_comes_from_the_wg_override() {
        assert_eq!(workgroup_x("k", &[]).unwrap(), 256);
        assert_eq!(workgroup_x("k", &[("other", 3.0)]).unwrap(), 256);
        assert_eq!(workgroup_x("k", &[("WG", 64.0)]).unwrap(), 64);
        assert_eq!(workgroup_x("k", &[("WG", 64.0), ("WG", 32.0)]).unwrap(), 32);
    }

    #[test]
    fn invalid_wg_is_a_config_error() {
        for wg in [0.0, -1.0, 1.5, f64::NAN, f64::INFINITY, 1e12] {
            let e = workgroup_x("k", &[("WG", wg)]).unwrap_err();
            assert!(matches!(e, ComputeError::Config(_)), "WG = {wg}: {e:?}");
        }
    }
}
