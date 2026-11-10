//! wgpu renderer: scene mirror, picking, analysis shaders and result plots. Fed via `RenderScene`, never `amg-doc`.
//!
//! Spec: S10. Lane: D.
//!
//! RND-01 is the Phase-0 spike: [`Renderer`] with per-view offscreen targets, the f64
//! [`Camera`] with relative-to-eye (RTE) matrices, one Lambert + headlight pipeline,
//! [`upload_mesh`], headless [`Renderer::capture`] and the `egui_glue` paint callback.
//!
//! RND-03 adds the [`RenderScene`]: shared meshes, instances whose 128-byte records are
//! rewritten only when changed, layers, CPU frustum culling with a front-to-back draw plan,
//! several views over one scene ([`Renderer::capture_many`]) and the [`ExternalLayer`]
//! composite. The remaining S10 API arrives with the later work packages.

#![warn(missing_docs)]

mod camera;
mod cull;
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
pub use scene::{ExternalLayer, Instance, Layer, LayerMask, Material, RenderScene};
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

/// Dense id chosen by the scene adapter, written to the id buffer by picking (RND-04).
/// 0 means nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct PickId(pub u32);
