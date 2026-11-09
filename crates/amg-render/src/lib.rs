//! wgpu renderer: scene mirror, picking, analysis shaders and result plots. Fed via `RenderScene`, never `amg-doc`.
//!
//! Spec: S10. Lane: D.
//!
//! RND-01 is the Phase-0 spike: [`Renderer`] with per-view offscreen targets, the f64
//! [`Camera`] with relative-to-eye (RTE) matrices, one Lambert + headlight pipeline,
//! [`upload_mesh`], headless [`Renderer::capture`] and the `egui_glue` paint callback. The
//! remaining S10 API arrives with the later work packages.

#![warn(missing_docs)]

mod camera;
mod error;
mod renderer;
mod scene;
mod upload;

#[cfg(feature = "egui")]
pub mod egui_glue;

pub use wgpu;

pub use camera::{Camera, CameraUniform, Projection};
pub use error::RenderError;
pub use renderer::{
    Background, Capture, FrameStats, GpuTimings, PipelineTiming, Renderer, RendererConfig,
    RgbaImage, View, ViewStyle,
};
pub use scene::{Instance, Material, RenderScene};
pub use upload::{GpuMesh, MeshData, upload_mesh};

/// OR these into `GpuConfig::optional_features` before `Gpu::new` (all have fallbacks).
pub const OPTIONAL_FEATURES: wgpu::Features = wgpu::Features::PRIMITIVE_INDEX
    .union(wgpu::Features::POLYGON_MODE_LINE)
    .union(wgpu::Features::TIMESTAMP_QUERY);

amg_core::define_id! {
    /// A view (viewport) of the [`Renderer`].
    pub struct ViewId;
    /// A mesh in the [`RenderScene`].
    pub struct MeshKey;
    /// An instance in the [`RenderScene`].
    pub struct InstanceKey;
}
