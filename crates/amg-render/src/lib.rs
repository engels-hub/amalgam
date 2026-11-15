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
//! composite.
//!
//! RND-02 adds the shading framework: PBR-lite [`Lighting::Studio`] (GGX key and fill lights
//! plus a hemisphere ambient), a procedural [`Lighting::Matcap`] and the headlight
//! [`Lighting::Flat`], all with CPU references ([`shade_reference`]); octahedral `Snorm16x2`
//! normals ([`oct_encode`]); a view-normal target; MSAA 1/4, optional FXAA and gradient
//! backgrounds ([`ViewStyle`]); pipeline variants from WGSL `override` constants, created lazily
//! on jobs ([`RendererConfig::jobs`]); an optional depth pre-pass; [`Material`] metallic and
//! roughness.
//!
//! RND-12 adds lines and points: [`upload_edges`] / [`upload_points`], screen-space wide lines
//! with a capsule distance (any width, antialiasing, round joins, dashes) and a view-space depth
//! offset, hidden-edge passes, point sprites, a silhouettes post pass and Fusion's six
//! [`VisualStyle`]s, with the triangle wireframe of mesh bodies drawn through
//! `POLYGON_MODE_LINE` (feature edges without it).
//!
//! RND-06 adds section planes ([`SectionPlane`], up to [`MAX_SECTIONS`]): clipped pipeline
//! variants that `discard` in the mesh, depth, line, point and wireframe passes, back-face caps
//! by ray–plane intersection with a per-instance cap colour and screen-constant hatching, drawn
//! only for closed meshes, and a cap depth pass that puts cap depth on the plane.
//!
//! RND-08 adds analyses and fields ([`Shading`] per instance, design note §11): zebra, draft and
//! overhang shader modes computed live from the normal, per-vertex scalar fields
//! ([`upload_scalar_field`]) through a colormap LUT texture array ([`ColorMap`], with
//! [`colormap_rgb`] as the CPU reference) with bands, isolines and out-of-range colours, the
//! [`legend`] ticks, and vector-field deformation ([`upload_vector_field`], [`Deformation`],
//! [`Animation`]). Switching shading rewrites two 128-byte records, never geometry. The
//! remaining S10 API arrives with the later work packages.

#![warn(missing_docs)]

mod analysis;
mod camera;
mod colormap_data;
mod cull;
mod error;
mod lines;
mod pipelines;
mod renderer;
mod scene;
mod section;
mod shading;
mod upload;

#[cfg(feature = "egui")]
pub mod egui_glue;

pub use wgpu;

pub use analysis::testing as analysis_testing;
pub use analysis::{
    Analysis, AnalysisOutput, Animation, CONTACT_COLOR, ColorMap, DRAFT_INSUFFICIENT_COLOR,
    Deformation, FieldBinding, FieldKind, GpuField, ISOLINE_ALPHA, ISOLINE_COLOR, LUT_SIZE, Legend,
    NAN_COLOR, OVERHANG_HIGH_COLOR, OVERHANG_LOW_COLOR, OutOfRange, Shading, ZEBRA_DARK,
    ZEBRA_LIGHT, class, colormap_lut, colormap_rgb, decode_class, draft_angle, legend,
    overhang_class, overhang_severity, upload_scalar_field, upload_vector_field, zebra_t,
};
pub use camera::{Camera, CameraUniform, Projection};
pub use error::RenderError;
pub use lines::{
    DepthMode, EdgeData, GpuEdges, GpuPoints, HIDDEN_ALPHA, HIDDEN_DASH_PX, LineStyle, PointData,
    PointStyle, Polyline, VisualStyle, upload_edges, upload_points,
};
pub use renderer::{
    Background, Capture, FrameStats, GpuTimings, PipelineTiming, Renderer, RendererConfig,
    RgbaImage, View, ViewStyle,
};
pub use scene::{ExternalLayer, Instance, Layer, LayerMask, Material, RenderScene};
pub use section::{Hatch, MAX_SECTIONS, SectionPlane, section_cap_point};
pub use shading::{Lighting, oct_decode, oct_encode, shade_reference};
pub use upload::{GpuMesh, MeshData, geometry_bytes_uploaded, upload_mesh};

/// OR these into `GpuConfig::optional_features` before `Gpu::new` (all have fallbacks).
pub const OPTIONAL_FEATURES: wgpu::Features = wgpu::Features::PRIMITIVE_INDEX
    .union(wgpu::Features::POLYGON_MODE_LINE)
    .union(wgpu::Features::TIMESTAMP_QUERY);

amg_core::define_id! {
    /// A view (viewport) of the [`Renderer`].
    pub struct ViewId;
    /// A mesh in the [`RenderScene`].
    pub struct MeshKey;
    /// An edge set in the [`RenderScene`].
    pub struct EdgesKey;
    /// A point set in the [`RenderScene`].
    pub struct PointsKey;
    /// A per-vertex field in the [`RenderScene`] (RND-08).
    pub struct FieldKey;
    /// An instance in the [`RenderScene`].
    pub struct InstanceKey;
    /// A section plane in the [`RenderScene`].
    pub struct SectionKey;
}

/// Pick sub-id of a section cap (kind face): picking (RND-04) reports a cap hit with this
/// sub-id and the world point from [`section_cap_point`].
pub const CAP_SUB: u32 = (1 << 30) - 1;

/// Dense id chosen by the scene adapter, written to the id buffer by picking (RND-04).
/// 0 means nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct PickId(pub u32);
