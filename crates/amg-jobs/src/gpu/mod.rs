//! The shared wgpu device for rendering and compute (S00 §4, FND-09, ADR-004).
//!
//! One [`Gpu`] is created at startup and shared as `Arc<Gpu>`. It owns the wgpu instance,
//! adapter, device and queue, so the viewport (egui-wgpu) and the compute kernels use the same
//! device. Helpers cover typed buffer upload, storage allocation, non-blocking readback
//! ([`Readback`]) and a compute-pipeline cache.
//!
//! ```no_run
//! use amg_jobs::gpu::{ComputePipelineDesc, Gpu, GpuConfig};
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let gpu = Gpu::new_blocking(&GpuConfig::from_env())?;
//! let wgsl = "@group(0) @binding(0) var<storage, read_write> v: array<f32>;
//!             @compute @workgroup_size(64) fn main(@builtin(global_invocation_id) i: vec3<u32>) {
//!                 if (i.x < arrayLength(&v)) { v[i.x] = v[i.x] * 2.0; }
//!             }";
//! let pipeline = gpu.compute_pipeline(&ComputePipelineDesc::new("double", wgsl, "main"))?;
//! let buf = gpu.upload("v", &[1.0f32, 2.0, 3.0], wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC);
//! let bind = gpu.device().create_bind_group(&wgpu::BindGroupDescriptor {
//!     label: Some("double"),
//!     layout: &pipeline.get_bind_group_layout(0),
//!     entries: &[wgpu::BindGroupEntry { binding: 0, resource: buf.as_entire_binding() }],
//! });
//! let mut enc = gpu.device().create_command_encoder(&Default::default());
//! {
//!     let mut pass = enc.begin_compute_pass(&Default::default());
//!     pass.set_pipeline(&pipeline);
//!     pass.set_bind_group(0, &bind, &[]);
//!     pass.dispatch_workgroups(1, 1, 1);
//! }
//! gpu.queue().submit([enc.finish()]);
//! let out: Vec<f32> = gpu.readback(&buf, 0..12).wait()?; // job threads only
//! assert_eq!(out, [2.0, 4.0, 6.0]);
//! # Ok(()) }
//! ```
//!
//! # Sharing the device with egui-wgpu
//! `amg-app` hands the same handles to eframe (egui-wgpu 0.37), so render and compute use one
//! device (egui-wgpu is not a workspace dependency yet, so this is not compiled):
//!
//! ```text
//! let gpu = Gpu::new_blocking(&cfg)?;          // before the event loop starts
//! let wgpu_options = egui_wgpu::WgpuConfiguration {
//!     wgpu_setup: egui_wgpu::WgpuSetup::Existing(egui_wgpu::WgpuSetupExisting {
//!         instance: gpu.instance().clone(),
//!         adapter: gpu.adapter().clone(),
//!         device: gpu.device().clone(),
//!         queue: gpu.queue().clone(),
//!     }),
//!     ..Default::default()
//! };
//! let native = eframe::NativeOptions { wgpu_options, ..Default::default() };
//! ```

mod backend;
mod pipeline;
mod readback;

use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use parking_lot::Mutex;
use wgpu::util::DeviceExt as _;

pub use backend::{BackendSelection, UnknownBackend, WGPU_BACKEND_ENV};
pub use pipeline::ComputePipelineDesc;
pub use readback::Readback;

/// Re-export of the pinned `wgpu`, so dependants use exactly this crate's version.
pub use wgpu;

/// How [`Gpu::new`] picks the adapter and creates the device.
#[derive(Clone, Debug)]
pub struct GpuConfig {
    /// Backends the instance enables. Default: DX12 on Windows, primary backends elsewhere.
    pub backends: wgpu::Backends,
    /// Default: [`wgpu::PowerPreference::HighPerformance`].
    pub power_preference: wgpu::PowerPreference,
    /// Only a software adapter (WARP on Windows; used by CI). Default: `false`.
    pub force_fallback_adapter: bool,
    /// Features the device must have; device creation fails otherwise. Default: empty.
    pub required_features: wgpu::Features,
    /// Features enabled when the adapter has them; query with [`Gpu::has`].
    /// Default: `SHADER_F64 | TIMESTAMP_QUERY`. Callers OR in their own (for example
    /// `amg_render::OPTIONAL_FEATURES`).
    pub optional_features: wgpu::Features,
    /// Floor the adapter's limits must meet. The device is created with the adapter's own
    /// (better) limits when they meet it; otherwise device creation fails.
    /// Default: [`wgpu::Limits::downlevel_defaults`].
    pub required_limits: wgpu::Limits,
    /// DX12 shader compiler. Default: [`wgpu::Dx12Compiler::Fxc`] (no extra DLL, deterministic;
    /// wgpu's own default `Auto` depends on what is on `PATH`).
    pub dx12_compiler: wgpu::Dx12Compiler,
    /// Instance flags (validation, debug labels). Default: wgpu's build-config defaults plus
    /// the `WGPU_*` environment overrides (`WGPU_VALIDATION=0`, ...).
    pub instance_flags: wgpu::InstanceFlags,
}

impl Default for GpuConfig {
    fn default() -> Self {
        let sel = BackendSelection::platform_default();
        GpuConfig {
            backends: sel.backends,
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: sel.force_fallback_adapter,
            required_features: wgpu::Features::empty(),
            optional_features: wgpu::Features::SHADER_F64 | wgpu::Features::TIMESTAMP_QUERY,
            required_limits: wgpu::Limits::downlevel_defaults(),
            dx12_compiler: wgpu::Dx12Compiler::Fxc,
            instance_flags: wgpu::InstanceFlags::from_env_or_default(),
        }
    }
}

impl GpuConfig {
    /// Applies `AMG_WGPU_BACKEND=dx12|vulkan|warp` on top of [`GpuConfig::default`].
    ///
    /// # Panics
    /// If `AMG_WGPU_BACKEND` has another value; use [`GpuConfig::try_from_env`] to report it
    /// instead.
    pub fn from_env() -> GpuConfig {
        match GpuConfig::try_from_env() {
            Ok(cfg) => cfg,
            Err(e) => panic!("{e}"),
        }
    }

    /// Like [`GpuConfig::from_env`], but an unknown `AMG_WGPU_BACKEND` is an error.
    ///
    /// # Errors
    /// [`UnknownBackend`] when the variable is set to something other than `dx12`, `vulkan`
    /// or `warp`.
    pub fn try_from_env() -> Result<GpuConfig, UnknownBackend> {
        Ok(GpuConfig::default().with_backend(BackendSelection::from_env()?))
    }

    /// Replaces the backend and fallback choice.
    #[must_use]
    pub fn with_backend(mut self, sel: BackendSelection) -> GpuConfig {
        self.backends = sel.backends;
        self.force_fallback_adapter = sel.force_fallback_adapter;
        self
    }
}

/// Errors from [`Gpu`] and its helpers.
#[derive(Debug, thiserror::Error)]
pub enum GpuError {
    /// No adapter matches the configured backends / fallback choice.
    #[error("no compatible GPU adapter found")]
    NoAdapter,
    /// The adapter refused the device (missing required features or limits).
    #[error("GPU device request failed: {0}")]
    RequestDevice(#[from] wgpu::RequestDeviceError),
    /// The device was lost (driver reset, removal, or `destroy`).
    #[error("GPU device lost: {0}")]
    DeviceLost(String),
    /// Mapping a readback buffer failed.
    #[error("buffer mapping failed: {0}")]
    Map(#[from] wgpu::BufferAsyncError),
    /// WGSL parsing/validation or pipeline creation failed.
    #[error("shader `{label}` failed to compile: {message}")]
    Shader {
        /// The pipeline label.
        label: String,
        /// The wgpu validation message (with source location).
        message: String,
    },
    /// `Device::poll` failed.
    #[error("GPU poll failed: {0}")]
    Poll(String),
    /// The operation was cancelled.
    #[error(transparent)]
    Cancelled(#[from] amg_core::Cancelled),
}

impl amg_core::Diagnostic for GpuError {
    fn user_message(&self) -> String {
        match self {
            GpuError::NoAdapter => "No compatible graphics adapter was found.".to_owned(),
            GpuError::RequestDevice(_) => "The graphics adapter could not be opened.".to_owned(),
            GpuError::DeviceLost(_) => "The graphics device was lost.".to_owned(),
            GpuError::Map(_) | GpuError::Poll(_) => "A GPU operation failed.".to_owned(),
            GpuError::Shader { label, .. } => format!("GPU shader `{label}` failed to compile."),
            GpuError::Cancelled(c) => c.to_string(),
        }
    }

    fn hint(&self) -> Option<String> {
        let backend = format!(
            "set {WGPU_BACKEND_ENV}=dx12, vulkan or warp (software) to try another backend"
        );
        match self {
            GpuError::NoAdapter | GpuError::RequestDevice(_) => {
                Some(format!("Update the graphics driver, or {backend}."))
            }
            GpuError::DeviceLost(_) => Some(format!(
                "Restart Amalgam; if it happens again, update the graphics driver or {backend}."
            )),
            GpuError::Shader { .. } => {
                Some("This is a bug in Amalgam; the log has the details.".to_owned())
            }
            GpuError::Map(_) | GpuError::Poll(_) => Some("Update the graphics driver.".to_owned()),
            GpuError::Cancelled(_) => None,
        }
    }

    fn severity(&self) -> amg_core::Severity {
        match self {
            GpuError::Cancelled(_) => amg_core::Severity::Info,
            _ => amg_core::Severity::Error,
        }
    }

    fn is_cancelled(&self) -> bool {
        matches!(self, GpuError::Cancelled(_))
    }
}

/// Device-lost state shared with the wgpu callback and every [`Readback`].
#[derive(Debug, Default)]
pub(crate) struct LostState {
    lost: AtomicBool,
    message: Mutex<String>,
}

impl LostState {
    fn set(&self, reason: wgpu::DeviceLostReason, message: &str) {
        *self.message.lock() = format!("{reason:?}: {message}");
        self.lost.store(true, Ordering::Release);
    }

    pub(crate) fn is_lost(&self) -> bool {
        self.lost.load(Ordering::Acquire)
    }

    /// `Err(DeviceLost)` once the device is lost.
    pub(crate) fn check(&self) -> Result<(), GpuError> {
        if self.is_lost() {
            Err(GpuError::DeviceLost(self.message.lock().clone()))
        } else {
            Ok(())
        }
    }
}

/// Shared device for render + compute. Create once; share as `Arc<Gpu>`.
///
/// `Gpu` is `Send + Sync`; the wgpu handles it returns are cheap reference-counted clones.
pub struct Gpu {
    instance: wgpu::Instance,
    adapter: wgpu::Adapter,
    device: wgpu::Device,
    queue: wgpu::Queue,
    info: wgpu::AdapterInfo,
    features: wgpu::Features,
    lost: Arc<LostState>,
    pipelines: pipeline::PipelineCache,
}

impl fmt::Debug for Gpu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Gpu")
            .field("adapter", &self.info.name)
            .field("backend", &self.info.backend)
            .field("features", &self.features)
            .field("lost", &self.lost.is_lost())
            .finish_non_exhaustive()
    }
}

impl Gpu {
    /// Creates the instance, picks the adapter and opens the device.
    ///
    /// Features: `required_features` plus whichever `optional_features` the adapter has.
    /// Limits: the adapter's, provided they meet `required_limits`.
    ///
    /// # Errors
    /// [`GpuError::NoAdapter`] when no adapter matches, [`GpuError::RequestDevice`] when the
    /// adapter lacks a required feature or limit.
    pub async fn new(cfg: &GpuConfig) -> Result<Arc<Gpu>, GpuError> {
        let mut desc = wgpu::InstanceDescriptor::new_without_display_handle();
        desc.backends = cfg.backends;
        desc.flags = cfg.instance_flags;
        desc.backend_options.dx12.shader_compiler = cfg.dx12_compiler.clone();
        let instance = wgpu::Instance::new(desc);

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: cfg.power_preference,
                force_fallback_adapter: cfg.force_fallback_adapter,
                compatible_surface: None,
                apply_limit_buckets: false,
            })
            .await
            .map_err(|_| GpuError::NoAdapter)?;

        let adapter_limits = adapter.limits();
        // Ask for the adapter's limits when they meet the floor; otherwise ask for the floor so
        // wgpu reports which limit is missing.
        let required_limits = if cfg.required_limits.check_limits(&adapter_limits) {
            adapter_limits
        } else {
            cfg.required_limits.clone()
        };
        let required_features =
            cfg.required_features | (cfg.optional_features & adapter.features());
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("amalgam"),
                required_features,
                required_limits,
                experimental_features: wgpu::ExperimentalFeatures::disabled(),
                memory_hints: wgpu::MemoryHints::Performance,
                trace: wgpu::Trace::Off,
            })
            .await?;

        let lost = Arc::new(LostState::default());
        let flag = Arc::clone(&lost);
        device.set_device_lost_callback(move |reason, message| flag.set(reason, &message));

        Ok(Arc::new(Gpu {
            info: adapter.get_info(),
            features: device.features(),
            instance,
            adapter,
            device,
            queue,
            lost,
            pipelines: pipeline::PipelineCache::default(),
        }))
    }

    /// Blocking [`Gpu::new`] (pollster). Allowed on the UI thread *before* the event loop
    /// starts (it is not guarded), but must not be called from egui frames.
    ///
    /// # Errors
    /// As [`Gpu::new`].
    pub fn new_blocking(cfg: &GpuConfig) -> Result<Arc<Gpu>, GpuError> {
        #[allow(
            clippy::disallowed_methods,
            reason = "S00 §4: the one sanctioned blocking GPU call, used at startup"
        )]
        pollster::block_on(Gpu::new(cfg))
    }

    /// The wgpu instance (egui-wgpu needs it to create surfaces).
    pub fn instance(&self) -> &wgpu::Instance {
        &self.instance
    }

    /// The adapter the device was opened on.
    pub fn adapter(&self) -> &wgpu::Adapter {
        &self.adapter
    }

    /// The shared device.
    pub fn device(&self) -> &wgpu::Device {
        &self.device
    }

    /// The device's queue.
    pub fn queue(&self) -> &wgpu::Queue {
        &self.queue
    }

    /// Adapter name, vendor, backend and driver.
    pub fn info(&self) -> &wgpu::AdapterInfo {
        &self.info
    }

    /// True when the device has every feature in `f`.
    pub fn has(&self, f: wgpu::Features) -> bool {
        self.features.contains(f)
    }

    /// True once the device-lost callback has fired.
    pub fn is_lost(&self) -> bool {
        self.lost.is_lost()
    }

    /// Creates a buffer initialised with `data`. `COPY_DST` is added to `usage`.
    pub fn upload<T: bytemuck::Pod>(
        &self,
        label: &str,
        data: &[T],
        usage: wgpu::BufferUsages,
    ) -> wgpu::Buffer {
        self.device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents: bytemuck::cast_slice(data),
                usage: usage | wgpu::BufferUsages::COPY_DST,
            })
    }

    /// Creates a zero-initialised buffer of `len` elements of `T`. `STORAGE` is added to
    /// `usage`.
    ///
    /// # Panics
    /// If `len * size_of::<T>()` overflows `u64`.
    pub fn storage<T: bytemuck::Pod>(
        &self,
        label: &str,
        len: u64,
        usage: wgpu::BufferUsages,
    ) -> wgpu::Buffer {
        let size = len
            .checked_mul(size_of::<T>() as u64)
            .unwrap_or_else(|| panic!("storage buffer `{label}`: {len} elements overflow u64"));
        self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: usage | wgpu::BufferUsages::STORAGE,
            mapped_at_creation: false,
        })
    }

    /// Copies `range` (bytes) of `src` to a staging buffer and starts mapping it. `src` needs
    /// `COPY_SRC`. Drive it with [`Readback::try_get`] (UI) or [`Readback::wait`] (jobs).
    ///
    /// # Panics
    /// If the range is empty, reversed, not a multiple of 4 bytes (start and length), or its
    /// length is not a multiple of `size_of::<T>()`.
    pub fn readback<T: bytemuck::Pod>(
        &self,
        src: &wgpu::Buffer,
        range: std::ops::Range<u64>,
    ) -> Readback<T> {
        Readback::start(self, src, range)
    }

    /// Returns the compute pipeline for `desc`, compiling it on first use. Pipelines are cached
    /// by (WGSL source, entry point, override constants), so equal descs give the same `Arc`.
    /// Compilation runs on the calling thread; prefer a job thread for large shaders.
    ///
    /// # Errors
    /// [`GpuError::Shader`] with wgpu's message when the WGSL does not parse or validate, or
    /// the pipeline cannot be created; [`GpuError::DeviceLost`] after device loss. Failures
    /// are not cached.
    pub fn compute_pipeline(
        &self,
        desc: &ComputePipelineDesc<'_>,
    ) -> Result<Arc<wgpu::ComputePipeline>, GpuError> {
        self.lost.check()?;
        self.pipelines.get_or_create(&self.device, desc)
    }

    /// Non-blocking device poll; runs completed map callbacks.
    ///
    /// # Errors
    /// [`GpuError::DeviceLost`] after device loss, [`GpuError::Poll`] if wgpu reports one.
    pub fn poll(&self) -> Result<(), GpuError> {
        poll_device(&self.device, &self.lost)
    }

    pub(crate) fn lost_state(&self) -> &Arc<LostState> {
        &self.lost
    }
}

/// One non-blocking poll, then the lost check.
pub(crate) fn poll_device(device: &wgpu::Device, lost: &LostState) -> Result<(), GpuError> {
    lost.check()?;
    device
        .poll(wgpu::PollType::Poll)
        .map_err(|e| GpuError::Poll(e.to_string()))?;
    lost.check()
}

#[cfg(test)]
mod tests {
    use amg_core::Diagnostic as _;

    use super::*;

    #[test]
    fn default_config_matches_the_spec() {
        let c = GpuConfig::default();
        assert_eq!(c.power_preference, wgpu::PowerPreference::HighPerformance);
        assert!(!c.force_fallback_adapter);
        assert!(c.required_features.is_empty());
        assert_eq!(
            c.optional_features,
            wgpu::Features::SHADER_F64 | wgpu::Features::TIMESTAMP_QUERY
        );
        assert!(matches!(c.dx12_compiler, wgpu::Dx12Compiler::Fxc));
        if cfg!(windows) {
            assert_eq!(c.backends, wgpu::Backends::DX12);
        }
    }

    #[test]
    fn with_backend_selects_warp() {
        let warp = BackendSelection::parse(Some("warp")).unwrap();
        let c = GpuConfig::default().with_backend(warp);
        assert_eq!(c.backends, wgpu::Backends::DX12);
        assert!(c.force_fallback_adapter);
    }

    #[test]
    fn errors_have_hints() {
        let e = GpuError::NoAdapter;
        assert!(e.hint().unwrap().contains(WGPU_BACKEND_ENV));
        let s = GpuError::Shader {
            label: "add".into(),
            message: "bad".into(),
        };
        assert_eq!(s.to_string(), "shader `add` failed to compile: bad");
        let c = GpuError::from(amg_core::Cancelled);
        assert!(c.is_cancelled());
        assert_eq!(c.severity(), amg_core::Severity::Info);
    }

    #[test]
    fn lost_state_reports_the_message() {
        let l = LostState::default();
        assert!(l.check().is_ok());
        l.set(wgpu::DeviceLostReason::Destroyed, "gone");
        let Err(GpuError::DeviceLost(m)) = l.check() else {
            panic!("expected DeviceLost");
        };
        assert!(m.contains("gone"), "{m}");
    }
}
