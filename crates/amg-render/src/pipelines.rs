//! Render pipeline variants (design note §4, §16): keys, the factory that builds them from WGSL
//! `override` constants, and lazy creation on job threads.
//!
//! The *core set* of a configuration (sample count, depth pre-pass) is built synchronously by
//! `Renderer::new`; every other variant is requested on first need and built on the
//! Interactive pool of an attached [`JobSystem`] (or a short-lived thread without one). Until a
//! variant is ready, draws fall back to the core Studio pipeline of the same geometry kind.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use amg_jobs::{Cancelled, JobSystem, Priority, SpawnOptions};

use crate::analysis::{AnalysisOutput, ShadeKey, ShadeMode, common_wgsl};
use crate::lines::LinePass;
use crate::renderer::PipelineTiming;
use crate::shading::{
    AMBIENT_SPECULAR, FILL_DIR, FILL_RGB, GROUND_RGB, KEY_DIR, KEY_RGB, Lighting, MATCAP_DIR,
    MIN_ROUGHNESS, SKY_RGB,
};

pub(crate) const HDR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
/// Octahedral view normal of the opaque pass (design note §2).
pub(crate) const NORMAL_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rg16Float;
pub(crate) const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
pub(crate) const FINAL_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;

const CAMERA_WGSL: &str = include_str!("shaders/camera.wgsl");
const SHADING_WGSL: &str = include_str!("shaders/shading.wgsl");
const SCENE_WGSL: &str = include_str!("shaders/scene.wgsl");
const MESH_WGSL: &str = include_str!("shaders/mesh.wgsl");
const LINES_WGSL: &str = include_str!("shaders/lines.wgsl");
const SILHOUETTE_WGSL: &str = include_str!("shaders/silhouette.wgsl");
const EXTERNAL_WGSL: &str = include_str!("shaders/external.wgsl");
const POST_WGSL: &str = include_str!("shaders/post.wgsl");
const SECTION_WGSL: &str = include_str!("shaders/section.wgsl");
const CAP_DEPTH_WGSL: &str = include_str!("shaders/cap_depth.wgsl");
pub(crate) const FULLSCREEN_WGSL: &str = include_str!("shaders/fullscreen.wgsl");

/// What a pipeline draws.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum PipelineKind {
    /// Opaque meshes into colour + normal.
    Mesh {
        lighting: Lighting,
        /// Facet normals from derivatives (`vs_flat`, positions only).
        flat: bool,
        /// No back-face culling (open meshes).
        double_sided: bool,
        /// Depth test `Equal` without depth write, after a depth pre-pass.
        equal_depth: bool,
        /// Discards what the section planes remove (RND-06). A clipped closed mesh is drawn
        /// without culling and its back faces become section caps (`CAPS`).
        clipped: bool,
        /// Material (the RND-02 programme) or an analysis / field programme (RND-08,
        /// `shaders/analysis.wgsl`).
        shade: ShadeKey,
    },
    /// Depth pre-pass (no fragment stage unless clipped).
    Depth {
        flat: bool,
        double_sided: bool,
        clipped: bool,
    },
    /// Front faces of cut closed meshes past the exit of a bounded kept region, depth only
    /// (colour writes masked; drawn in the opaque pass before the meshes).
    CapOccluder,
    /// Moves the depth of cap samples onto their plane (fullscreen, per sample, depth only).
    CapDepth,
    /// Gradient background at the far plane.
    Background,
    /// `ExternalLayer` composite.
    External,
    /// FXAA into the final image (always single-sampled).
    Fxaa,
    /// Wide-line edge segments (RND-12); `clipped` honours the section planes.
    Line { pass: LinePass, clipped: bool },
    /// Point sprites.
    Point { pass: LinePass, clipped: bool },
    /// Triangle wireframe through `POLYGON_MODE_LINE`.
    Wire { pass: LinePass, clipped: bool },
    /// Silhouettes over the resolved image; `samples` is the sample count of the targets it
    /// reads (it always draws single-sampled).
    Silhouette,
}

/// A pipeline variant: what it draws and at which sample count.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct PipelineKey {
    pub kind: PipelineKind,
    pub samples: u32,
}

impl PipelineKey {
    pub fn mesh(
        lighting: Lighting,
        flat: bool,
        double_sided: bool,
        equal_depth: bool,
        samples: u32,
    ) -> Self {
        PipelineKey {
            kind: PipelineKind::Mesh {
                lighting,
                flat,
                double_sided,
                equal_depth,
                clipped: false,
                shade: ShadeKey::default(),
            },
            samples,
        }
    }

    /// This mesh key with another shading programme (other kinds are returned unchanged).
    #[must_use]
    pub fn with_shade(mut self, key: ShadeKey) -> Self {
        if let PipelineKind::Mesh { shade, .. } = &mut self.kind {
            *shade = key;
        }
        self
    }

    /// Whether this is an analysis / field pipeline (layout groups 1-3).
    pub fn is_analysis(&self) -> bool {
        matches!(self.kind, PipelineKind::Mesh { shade, .. } if !shade.is_material())
    }

    /// This key with section clipping on or off (mesh, depth, line, point and wireframe kinds;
    /// other kinds are returned unchanged).
    #[must_use]
    pub fn with_clip(mut self, on: bool) -> Self {
        match &mut self.kind {
            PipelineKind::Mesh { clipped, .. }
            | PipelineKind::Depth { clipped, .. }
            | PipelineKind::Line { clipped, .. }
            | PipelineKind::Point { clipped, .. }
            | PipelineKind::Wire { clipped, .. } => *clipped = on,
            _ => {}
        }
        self
    }

    /// Family name for [`PipelineTiming::label`].
    pub fn family(&self) -> &'static str {
        match self.kind {
            PipelineKind::Mesh { shade, .. } if !shade.is_material() => "analysis",
            PipelineKind::Mesh { .. } => "mesh",
            PipelineKind::Depth { .. } => "depth prepass",
            PipelineKind::Background => "background",
            PipelineKind::External => "external layer",
            PipelineKind::Fxaa => "fxaa",
            PipelineKind::Line { .. } => "line",
            PipelineKind::Point { .. } => "point",
            PipelineKind::Wire { .. } => "wireframe",
            PipelineKind::Silhouette => "silhouettes",
            PipelineKind::CapOccluder => "cap occluders",
            PipelineKind::CapDepth => "cap depth",
        }
    }

    /// Full description for [`PipelineTiming::variant`].
    pub fn describe(&self) -> String {
        format!("{:?} x{}", self.kind, self.samples)
    }
}

/// The pipelines a configuration needs before it can draw anything: Studio meshes of every
/// geometry kind, the background and the external-layer composite (plus the depth-only
/// pipelines when the pre-pass is on).
pub(crate) fn core_keys(samples: u32, prepass: bool) -> Vec<PipelineKey> {
    let mut keys = Vec::with_capacity(10);
    for flat in [false, true] {
        for double_sided in [false, true] {
            keys.push(PipelineKey::mesh(
                Lighting::Studio,
                flat,
                double_sided,
                prepass,
                samples,
            ));
            if prepass {
                keys.push(PipelineKey {
                    kind: PipelineKind::Depth {
                        flat,
                        double_sided,
                        clipped: false,
                    },
                    samples,
                });
            }
        }
    }
    keys.push(PipelineKey {
        kind: PipelineKind::Background,
        samples,
    });
    keys.push(PipelineKey {
        kind: PipelineKind::External,
        samples,
    });
    keys
}

/// WGSL constants generated from [`crate::shading`], so the shader and its CPU reference share
/// one definition.
fn shading_constants() -> String {
    let v3 = |name: &str, v: [f32; 3]| {
        format!(
            "const {name}: vec3<f32> = vec3<f32>({:?}, {:?}, {:?});\n",
            v[0], v[1], v[2]
        )
    };
    let mut out = String::new();
    out += &v3("KEY_DIR", KEY_DIR);
    out += &v3("KEY_RGB", KEY_RGB);
    out += &v3("FILL_DIR", FILL_DIR);
    out += &v3("FILL_RGB", FILL_RGB);
    out += &v3("SKY_RGB", SKY_RGB);
    out += &v3("GROUND_RGB", GROUND_RGB);
    out += &v3("MATCAP_DIR", MATCAP_DIR);
    out += &format!("const AMBIENT_SPECULAR: f32 = {AMBIENT_SPECULAR:?};\n");
    out += &format!("const MIN_ROUGHNESS: f32 = {MIN_ROUGHNESS:?};\n");
    out
}

/// Full WGSL of the mesh module (camera + shading + sections + mesh passes and caps).
pub(crate) fn mesh_wgsl() -> String {
    format!(
        "{CAMERA_WGSL}\n{}\n{SHADING_WGSL}\n{SCENE_WGSL}\n{SECTION_WGSL}\n{MESH_WGSL}",
        shading_constants()
    )
}

/// Full WGSL of the analysis module: the mesh module plus the shared analysis formulas and the
/// analysis entry points.
pub(crate) fn analysis_module_wgsl() -> String {
    format!(
        "{}\n{}\n{}",
        mesh_wgsl(),
        common_wgsl(),
        crate::analysis::ANALYSIS_WGSL
    )
}

/// Full WGSL of the line, point and wireframe module.
pub(crate) fn lines_wgsl() -> String {
    format!("{CAMERA_WGSL}\n{SCENE_WGSL}\n{SECTION_WGSL}\n{LINES_WGSL}")
}

/// WGSL of the cap depth pass reading marks of `samples` samples.
pub(crate) fn cap_depth_wgsl(samples: u32) -> String {
    let marks = if samples > 1 {
        "texture_multisampled_2d<f32>"
    } else {
        "texture_2d<f32>"
    };
    format!(
        "{CAMERA_WGSL}\n{SCENE_WGSL}\n{SECTION_WGSL}\n{}",
        CAP_DEPTH_WGSL.replace("MARK_TEXTURE", marks)
    )
}

/// WGSL of the silhouette pass reading targets of `samples` samples.
pub(crate) fn silhouette_wgsl(samples: u32) -> String {
    let (normal, depth) = if samples > 1 {
        (
            "texture_multisampled_2d<f32>",
            "texture_depth_multisampled_2d",
        )
    } else {
        ("texture_2d<f32>", "texture_depth_2d")
    };
    SILHOUETTE_WGSL
        .replace("NORMAL_TEXTURE", normal)
        .replace("DEPTH_TEXTURE", depth)
}

/// Bind group layouts the factory builds pipeline layouts from.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Layouts<'a> {
    pub view: &'a wgpu::BindGroupLayout,
    pub external: &'a wgpu::BindGroupLayout,
    pub post: &'a wgpu::BindGroupLayout,
    /// Group 1 of the line and point pipelines: one read-only storage buffer.
    pub geometry: &'a wgpu::BindGroupLayout,
    /// Silhouette inputs at one sample and at several.
    pub silhouette: [&'a wgpu::BindGroupLayout; 2],
    /// Group 1 of the cap depth pass (the normal target) at one sample and at several.
    pub cap_marks: [&'a wgpu::BindGroupLayout; 2],
    /// Group 1 of the analysis pipelines (records, LUT, sampler, globals); groups 2 and 3 are
    /// `geometry`-shaped (one storage buffer: scalar and displacement fields).
    pub analysis: &'a wgpu::BindGroupLayout,
}

/// Everything needed to build any variant; cheap to share with job threads.
#[derive(Debug)]
pub(crate) struct PipelineFactory {
    device: wgpu::Device,
    mesh_shader: wgpu::ShaderModule,
    mesh_layout: wgpu::PipelineLayout,
    external_shader: wgpu::ShaderModule,
    external_layout: wgpu::PipelineLayout,
    post_shader: wgpu::ShaderModule,
    post_layout: wgpu::PipelineLayout,
    lines_shader: wgpu::ShaderModule,
    lines_layout: wgpu::PipelineLayout,
    /// Silhouette modules and layouts at one sample (`[0]`) and several (`[1]`).
    silhouette_shader: [wgpu::ShaderModule; 2],
    silhouette_layout: [wgpu::PipelineLayout; 2],
    /// Cap depth modules and layouts at one sample (`[0]`) and several (`[1]`).
    cap_depth_shader: [wgpu::ShaderModule; 2],
    cap_depth_layout: [wgpu::PipelineLayout; 2],
    /// The analysis module, compiled on first use (it is never in the core set).
    analysis_shader: OnceLock<wgpu::ShaderModule>,
    analysis_layout: wgpu::PipelineLayout,
}

impl PipelineFactory {
    pub fn new(device: &wgpu::Device, layouts: Layouts<'_>) -> PipelineFactory {
        let module = |label, source: String| {
            device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some(label),
                source: wgpu::ShaderSource::Wgsl(source.into()),
            })
        };
        let layout = |label, bind: &wgpu::BindGroupLayout| {
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some(label),
                bind_group_layouts: &[Some(bind)],
                immediate_size: 0,
            })
        };
        PipelineFactory {
            device: device.clone(),
            mesh_shader: module("amg mesh", mesh_wgsl()),
            mesh_layout: layout("amg mesh", layouts.view),
            external_shader: module("amg external layer", EXTERNAL_WGSL.to_owned()),
            external_layout: layout("amg external layer", layouts.external),
            post_shader: module("amg post", format!("{FULLSCREEN_WGSL}\n{POST_WGSL}")),
            post_layout: layout("amg post", layouts.post),
            lines_shader: module("amg lines", lines_wgsl()),
            lines_layout: device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("amg lines"),
                bind_group_layouts: &[Some(layouts.view), Some(layouts.geometry)],
                immediate_size: 0,
            }),
            silhouette_shader: [
                module("amg silhouettes x1", silhouette_wgsl(1)),
                module("amg silhouettes msaa", silhouette_wgsl(4)),
            ],
            silhouette_layout: [
                layout("amg silhouettes x1", layouts.silhouette[0]),
                layout("amg silhouettes msaa", layouts.silhouette[1]),
            ],
            cap_depth_shader: [
                module("amg cap depth x1", cap_depth_wgsl(1)),
                module("amg cap depth msaa", cap_depth_wgsl(4)),
            ],
            cap_depth_layout: [0, 1].map(|i| {
                device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("amg cap depth"),
                    bind_group_layouts: &[Some(layouts.view), Some(layouts.cap_marks[i])],
                    immediate_size: 0,
                })
            }),
            analysis_shader: OnceLock::new(),
            analysis_layout: device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("amg analysis"),
                bind_group_layouts: &[
                    Some(layouts.view),
                    Some(layouts.analysis),
                    Some(layouts.geometry),
                    Some(layouts.geometry),
                ],
                immediate_size: 0,
            }),
        }
    }

    fn analysis_shader(&self) -> &wgpu::ShaderModule {
        self.analysis_shader.get_or_init(|| {
            self.device
                .create_shader_module(wgpu::ShaderModuleDescriptor {
                    label: Some("amg analysis"),
                    source: wgpu::ShaderSource::Wgsl(analysis_module_wgsl().into()),
                })
        })
    }

    pub fn build(&self, key: PipelineKey) -> wgpu::RenderPipeline {
        let label = key.describe();
        let multisample = wgpu::MultisampleState {
            count: key.samples,
            ..Default::default()
        };
        let depth = |write: bool, compare: wgpu::CompareFunction| {
            Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(write),
                depth_compare: Some(compare),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            })
        };
        // Colour + normal, as every pipeline of the opaque pass must declare.
        let opaque_targets = |blend: Option<wgpu::BlendState>, normal: bool| {
            [
                Some(wgpu::ColorTargetState {
                    format: HDR_FORMAT,
                    blend,
                    write_mask: wgpu::ColorWrites::ALL,
                }),
                Some(wgpu::ColorTargetState {
                    format: NORMAL_FORMAT,
                    blend: None,
                    write_mask: if normal {
                        wgpu::ColorWrites::ALL
                    } else {
                        wgpu::ColorWrites::empty()
                    },
                }),
            ]
        };
        let positions = [wgpu::VertexAttribute {
            format: wgpu::VertexFormat::Float32x3,
            offset: 0,
            shader_location: 0,
        }];
        let normals = [wgpu::VertexAttribute {
            format: wgpu::VertexFormat::Snorm16x2,
            offset: 0,
            shader_location: 1,
        }];
        let smooth_buffers = [
            Some(wgpu::VertexBufferLayout {
                array_stride: 12,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &positions,
            }),
            Some(wgpu::VertexBufferLayout {
                array_stride: 4,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &normals,
            }),
        ];
        let clip_constant = |on: bool| ("CLIP", f64::from(u8::from(on)));
        let geometry = |flat: bool, double_sided: bool| {
            (
                wgpu::VertexState {
                    module: &self.mesh_shader,
                    entry_point: Some(if flat { "vs_flat" } else { "vs_smooth" }),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    buffers: if flat {
                        &smooth_buffers[..1]
                    } else {
                        &smooth_buffers[..]
                    },
                },
                wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    front_face: wgpu::FrontFace::Ccw,
                    cull_mode: (!double_sided).then_some(wgpu::Face::Back),
                    ..Default::default()
                },
            )
        };
        let device = &self.device;
        match key.kind {
            PipelineKind::Mesh {
                lighting,
                flat,
                double_sided,
                equal_depth: _,
                clipped,
                shade,
            } if !shade.is_material() => {
                // Analysis / field programme: the geometry state of the material pipeline.
                // Analysis groups never use the depth pre-pass, so the depth test is always
                // `GreaterEqual` with writes.
                let caps = clipped && !double_sided;
                let (_, primitive) = geometry(flat, double_sided || caps);
                let module = self.analysis_shader();
                let flag = |on: bool| f64::from(u8::from(on));
                let out = match shade.out {
                    AnalysisOutput::Lit => 0.0,
                    AnalysisOutput::Unlit => 1.0,
                    AnalysisOutput::Classes => 2.0,
                };
                let mode = match shade.mode {
                    ShadeMode::Material | ShadeMode::Field => 1.0,
                    ShadeMode::Zebra => 2.0,
                    ShadeMode::Draft => 3.0,
                    ShadeMode::Overhang => 4.0,
                };
                let vertex_constants = [
                    ("HAS_SCALAR", flag(shade.scalar)),
                    ("HAS_DEFORM", flag(shade.deform)),
                ];
                let fragment_constants = [
                    ("LIGHTING", f64::from(lighting.shader_id())),
                    clip_constant(clipped),
                    ("CAPS", flag(caps)),
                    ("MODE", mode),
                    ("OUT", out),
                    ("HAS_SCALAR", flag(shade.scalar)),
                ];
                let targets = opaque_targets(None, true);
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(&label),
                    layout: Some(&self.analysis_layout),
                    vertex: wgpu::VertexState {
                        module,
                        entry_point: Some(if flat {
                            "vs_analysis_flat"
                        } else {
                            "vs_analysis_smooth"
                        }),
                        compilation_options: wgpu::PipelineCompilationOptions {
                            constants: &vertex_constants,
                            ..Default::default()
                        },
                        buffers: if flat {
                            &smooth_buffers[..1]
                        } else {
                            &smooth_buffers[..]
                        },
                    },
                    primitive,
                    depth_stencil: depth(true, wgpu::CompareFunction::GreaterEqual),
                    multisample,
                    fragment: Some(wgpu::FragmentState {
                        module,
                        entry_point: Some("fs_analysis"),
                        compilation_options: wgpu::PipelineCompilationOptions {
                            constants: &fragment_constants,
                            ..Default::default()
                        },
                        targets: &targets,
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            }
            PipelineKind::Mesh {
                lighting,
                flat,
                double_sided,
                equal_depth,
                clipped,
                shade: _,
            } => {
                let caps = clipped && !double_sided;
                let (mut vertex, primitive) = geometry(flat, double_sided || caps);
                // After the clipped depth pre-pass (`Equal`) the shader needs no discard: the
                // clipped fragments were never written, so they fail the depth test.
                let constants = [
                    ("LIGHTING", f64::from(lighting.shader_id())),
                    clip_constant(clipped && !equal_depth),
                    ("CAPS", f64::from(u8::from(caps))),
                ];
                let vertex_constants = [("CAPS", f64::from(u8::from(caps)))];
                vertex.compilation_options = wgpu::PipelineCompilationOptions {
                    constants: &vertex_constants,
                    ..Default::default()
                };
                let targets = opaque_targets(None, true);
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(&label),
                    layout: Some(&self.mesh_layout),
                    vertex,
                    primitive,
                    depth_stencil: if equal_depth {
                        depth(false, wgpu::CompareFunction::Equal)
                    } else {
                        depth(true, wgpu::CompareFunction::GreaterEqual)
                    },
                    multisample,
                    fragment: Some(wgpu::FragmentState {
                        module: &self.mesh_shader,
                        entry_point: Some("fs_main"),
                        compilation_options: wgpu::PipelineCompilationOptions {
                            constants: &constants,
                            ..Default::default()
                        },
                        targets: &targets,
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            }
            PipelineKind::Depth {
                flat,
                double_sided,
                clipped,
            } => {
                // A clipped closed mesh keeps its back faces that are section-cap candidates.
                let caps = clipped && !double_sided;
                let (mut vertex, primitive) = geometry(flat, double_sided || caps);
                let caps_constant = [("CAPS", f64::from(u8::from(caps)))];
                vertex.compilation_options = wgpu::PipelineCompilationOptions {
                    constants: &caps_constant,
                    ..Default::default()
                };
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(&label),
                    layout: Some(&self.mesh_layout),
                    vertex,
                    primitive,
                    depth_stencil: depth(true, wgpu::CompareFunction::GreaterEqual),
                    multisample,
                    fragment: clipped.then(|| wgpu::FragmentState {
                        module: &self.mesh_shader,
                        entry_point: Some("fs_depth_clip"),
                        compilation_options: wgpu::PipelineCompilationOptions {
                            constants: &caps_constant,
                            ..Default::default()
                        },
                        targets: &[],
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            }
            PipelineKind::CapOccluder => {
                let masked = |format| {
                    Some(wgpu::ColorTargetState {
                        format,
                        blend: None,
                        write_mask: wgpu::ColorWrites::empty(),
                    })
                };
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(&label),
                    layout: Some(&self.mesh_layout),
                    vertex: wgpu::VertexState {
                        module: &self.mesh_shader,
                        entry_point: Some("vs_cap"),
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                        buffers: &smooth_buffers[..1],
                    },
                    primitive: wgpu::PrimitiveState {
                        topology: wgpu::PrimitiveTopology::TriangleList,
                        front_face: wgpu::FrontFace::Ccw,
                        cull_mode: Some(wgpu::Face::Back),
                        ..Default::default()
                    },
                    depth_stencil: depth(true, wgpu::CompareFunction::GreaterEqual),
                    multisample,
                    fragment: Some(wgpu::FragmentState {
                        module: &self.mesh_shader,
                        entry_point: Some("fs_cap_occluder"),
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                        targets: &[masked(HDR_FORMAT), masked(NORMAL_FORMAT)],
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            }
            PipelineKind::CapDepth => {
                let i = usize::from(key.samples > 1);
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(&label),
                    layout: Some(&self.cap_depth_layout[i]),
                    vertex: wgpu::VertexState {
                        module: &self.cap_depth_shader[i],
                        entry_point: Some("vs_cap_depth"),
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                        buffers: &[],
                    },
                    primitive: wgpu::PrimitiveState::default(),
                    depth_stencil: depth(true, wgpu::CompareFunction::Always),
                    multisample,
                    fragment: Some(wgpu::FragmentState {
                        module: &self.cap_depth_shader[i],
                        entry_point: Some("fs_cap_depth"),
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                        targets: &[],
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            }
            PipelineKind::Background => {
                let targets = opaque_targets(None, false);
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(&label),
                    layout: Some(&self.mesh_layout),
                    vertex: wgpu::VertexState {
                        module: &self.mesh_shader,
                        entry_point: Some("vs_background"),
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                        buffers: &[],
                    },
                    primitive: wgpu::PrimitiveState::default(),
                    depth_stencil: depth(false, wgpu::CompareFunction::Always),
                    multisample,
                    fragment: Some(wgpu::FragmentState {
                        module: &self.mesh_shader,
                        entry_point: Some("fs_background"),
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                        targets: &targets,
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            }
            PipelineKind::External => {
                let premultiplied = wgpu::BlendComponent {
                    src_factor: wgpu::BlendFactor::One,
                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                    operation: wgpu::BlendOperation::Add,
                };
                let targets = opaque_targets(
                    Some(wgpu::BlendState {
                        color: premultiplied,
                        alpha: premultiplied,
                    }),
                    false,
                );
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(&label),
                    layout: Some(&self.external_layout),
                    vertex: wgpu::VertexState {
                        module: &self.external_shader,
                        entry_point: Some("vs_external"),
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                        buffers: &[],
                    },
                    primitive: wgpu::PrimitiveState::default(),
                    depth_stencil: depth(true, wgpu::CompareFunction::GreaterEqual),
                    multisample,
                    fragment: Some(wgpu::FragmentState {
                        module: &self.external_shader,
                        entry_point: Some("fs_external"),
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                        targets: &targets,
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            }
            PipelineKind::Line { pass, clipped }
            | PipelineKind::Point { pass, clipped }
            | PipelineKind::Wire { pass, clipped } => {
                let alpha = wgpu::BlendState {
                    color: wgpu::BlendComponent {
                        src_factor: wgpu::BlendFactor::SrcAlpha,
                        dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                        operation: wgpu::BlendOperation::Add,
                    },
                    alpha: wgpu::BlendComponent {
                        src_factor: wgpu::BlendFactor::One,
                        dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                        operation: wgpu::BlendOperation::Add,
                    },
                };
                let targets = opaque_targets(Some(alpha), false);
                let compare = match pass {
                    LinePass::Hidden => wgpu::CompareFunction::Less,
                    LinePass::Visible => wgpu::CompareFunction::GreaterEqual,
                    LinePass::Always => wgpu::CompareFunction::Always,
                };
                let constants = [
                    ("HIDDEN", f64::from(u8::from(pass == LinePass::Hidden))),
                    clip_constant(clipped),
                ];
                let wire = matches!(key.kind, PipelineKind::Wire { .. });
                let (vs, fs) = match key.kind {
                    PipelineKind::Line { .. } => ("vs_line", "fs_line"),
                    PipelineKind::Point { .. } => ("vs_point", "fs_point"),
                    _ => ("vs_wire", "fs_wire"),
                };
                let options = wgpu::PipelineCompilationOptions {
                    constants: &constants,
                    ..Default::default()
                };
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(&label),
                    layout: Some(if wire {
                        &self.mesh_layout
                    } else {
                        &self.lines_layout
                    }),
                    vertex: wgpu::VertexState {
                        module: &self.lines_shader,
                        entry_point: Some(vs),
                        compilation_options: options.clone(),
                        buffers: if wire { &smooth_buffers[..1] } else { &[] },
                    },
                    primitive: wgpu::PrimitiveState {
                        topology: wgpu::PrimitiveTopology::TriangleList,
                        cull_mode: None,
                        polygon_mode: if wire {
                            wgpu::PolygonMode::Line
                        } else {
                            wgpu::PolygonMode::Fill
                        },
                        ..Default::default()
                    },
                    depth_stencil: depth(false, compare),
                    multisample,
                    fragment: Some(wgpu::FragmentState {
                        module: &self.lines_shader,
                        entry_point: Some(fs),
                        compilation_options: options,
                        targets: &targets,
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            }
            PipelineKind::Silhouette => {
                let i = usize::from(key.samples > 1);
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(&label),
                    layout: Some(&self.silhouette_layout[i]),
                    vertex: wgpu::VertexState {
                        module: &self.silhouette_shader[i],
                        entry_point: Some("vs_silhouette"),
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                        buffers: &[],
                    },
                    primitive: wgpu::PrimitiveState::default(),
                    depth_stencil: None,
                    multisample: wgpu::MultisampleState::default(),
                    fragment: Some(wgpu::FragmentState {
                        module: &self.silhouette_shader[i],
                        entry_point: Some("fs_silhouette"),
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                        targets: &[Some(wgpu::ColorTargetState {
                            format: HDR_FORMAT,
                            blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            }
            PipelineKind::Fxaa => device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(&label),
                layout: Some(&self.post_layout),
                vertex: wgpu::VertexState {
                    module: &self.post_shader,
                    entry_point: Some("vs_fullscreen"),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    buffers: &[],
                },
                primitive: wgpu::PrimitiveState::default(),
                depth_stencil: None,
                multisample: wgpu::MultisampleState::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &self.post_shader,
                    entry_point: Some("fs_fxaa"),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: FINAL_FORMAT,
                        blend: None,
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                multiview_mask: None,
                cache: None,
            }),
        }
    }
}

/// A finished background build; `pipeline` is `None` when the build panicked.
struct Built {
    key: PipelineKey,
    pipeline: Option<wgpu::RenderPipeline>,
    ms: f64,
}

type Inbox = Arc<Mutex<Vec<Built>>>;

/// Reports the build's result, or its failure if the build unwinds before finishing.
struct Report {
    key: PipelineKey,
    inbox: Inbox,
    result: Option<(wgpu::RenderPipeline, f64)>,
}

impl Drop for Report {
    fn drop(&mut self) {
        let (pipeline, ms) = match self.result.take() {
            Some((p, ms)) => (Some(p), ms),
            None => (None, 0.0),
        };
        self.inbox
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push(Built {
                key: self.key,
                pipeline,
                ms,
            });
    }
}

/// Built pipelines plus the variants being built in the background.
#[derive(Debug)]
pub(crate) struct Variants {
    factory: Arc<PipelineFactory>,
    ready: HashMap<PipelineKey, wgpu::RenderPipeline>,
    pending: HashSet<PipelineKey>,
    failed: HashSet<PipelineKey>,
    inbox: Inbox,
    jobs: Option<JobSystem>,
    timings: Vec<PipelineTiming>,
}

impl std::fmt::Debug for Built {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Built")
            .field("key", &self.key)
            .field("ok", &self.pipeline.is_some())
            .finish()
    }
}

impl Variants {
    pub fn new(factory: PipelineFactory, jobs: Option<JobSystem>) -> Variants {
        Variants {
            factory: Arc::new(factory),
            ready: HashMap::new(),
            pending: HashSet::new(),
            failed: HashSet::new(),
            inbox: Arc::default(),
            jobs,
            timings: Vec::new(),
        }
    }

    /// Builds `key` on this thread (core pipelines) and returns its timing.
    pub fn build_now(&mut self, key: PipelineKey) -> PipelineTiming {
        let start = Instant::now();
        let pipeline = self.factory.build(key);
        let timing = PipelineTiming {
            label: key.family(),
            variant: key.describe(),
            ms: start.elapsed().as_secs_f64() * 1e3,
        };
        self.ready.insert(key, pipeline);
        timing
    }

    pub fn get(&self, key: PipelineKey) -> Option<&wgpu::RenderPipeline> {
        self.ready.get(&key)
    }

    pub fn has(&self, key: PipelineKey) -> bool {
        self.ready.contains_key(&key)
    }

    pub fn failed(&self, key: PipelineKey) -> bool {
        self.failed.contains(&key)
    }

    /// Starts building `key` in the background unless it is ready, pending or failed.
    pub fn request(&mut self, key: PipelineKey) {
        if self.ready.contains_key(&key) || self.failed.contains(&key) || !self.pending.insert(key)
        {
            return;
        }
        let factory = Arc::clone(&self.factory);
        let mut report = Report {
            key,
            inbox: Arc::clone(&self.inbox),
            result: None,
        };
        let work = move || {
            let start = Instant::now();
            let pipeline = factory.build(key);
            report.result = Some((pipeline, start.elapsed().as_secs_f64() * 1e3));
            drop(report);
        };
        tracing::debug!(variant = %key.describe(), "building pipeline variant in the background");
        match &self.jobs {
            Some(jobs) => {
                let opts = SpawnOptions {
                    priority: Priority::Interactive,
                    hidden: true,
                };
                drop(
                    jobs.spawn_fn("amg-render pipeline variant", opts, move |_ctx| {
                        work();
                        Ok::<(), Cancelled>(())
                    }),
                );
            }
            None => {
                let spawned = std::thread::Builder::new()
                    .name("amg-render variant".into())
                    .spawn(work);
                if let Err(e) = spawned {
                    // The closure (and its `Report`) was dropped: the failure is reported.
                    tracing::error!(error = %e, "cannot spawn a pipeline build thread");
                }
            }
        }
    }

    /// Moves finished background builds into the ready set. Non-blocking.
    pub fn poll(&mut self) {
        let finished =
            std::mem::take(&mut *self.inbox.lock().unwrap_or_else(PoisonError::into_inner));
        for built in finished {
            self.pending.remove(&built.key);
            match built.pipeline {
                Some(p) => {
                    self.timings.push(PipelineTiming {
                        label: built.key.family(),
                        variant: built.key.describe(),
                        ms: built.ms,
                    });
                    self.ready.insert(built.key, p);
                }
                None => {
                    tracing::error!(variant = %built.key.describe(), "pipeline variant build failed");
                    self.failed.insert(built.key);
                }
            }
        }
    }

    pub fn pending(&self) -> usize {
        self.pending.len()
    }

    /// Blocks until nothing is pending or `timeout` passes; true when nothing is pending.
    pub fn wait(&mut self, timeout: Duration) -> bool {
        let start = Instant::now();
        loop {
            self.poll();
            if self.pending.is_empty() {
                return true;
            }
            if start.elapsed() >= timeout {
                return false;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Timings of the variants built in the background so far.
    pub fn timings(&self) -> &[PipelineTiming] {
        &self.timings
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
mod tests {
    use super::*;

    #[test]
    fn core_set_covers_every_geometry_kind_with_studio_lighting() {
        let keys = core_keys(4, false);
        assert_eq!(keys.len(), 6);
        for flat in [false, true] {
            for double_sided in [false, true] {
                assert!(keys.contains(&PipelineKey::mesh(
                    Lighting::Studio,
                    flat,
                    double_sided,
                    false,
                    4
                )));
            }
        }
        assert!(keys.iter().all(|k| k.samples == 4));
        let with_prepass = core_keys(1, true);
        assert_eq!(with_prepass.len(), 10);
        assert!(with_prepass.contains(&PipelineKey {
            kind: PipelineKind::Depth {
                flat: true,
                double_sided: false,
                clipped: false,
            },
            samples: 1
        }));
        assert!(with_prepass.contains(&PipelineKey::mesh(Lighting::Studio, true, true, true, 1)));
    }

    #[test]
    fn silhouette_wgsl_picks_the_texture_types_by_sample_count() {
        let one = silhouette_wgsl(1);
        assert!(one.contains("var normals: texture_2d<f32>;"));
        assert!(one.contains("var depths: texture_depth_2d;"));
        let four = silhouette_wgsl(4);
        assert!(four.contains("var normals: texture_multisampled_2d<f32>;"));
        assert!(four.contains("var depths: texture_depth_multisampled_2d;"));
        assert!(!four.contains("NORMAL_TEXTURE") && !four.contains("DEPTH_TEXTURE"));
    }

    #[test]
    fn generated_constants_match_the_cpu_reference() {
        let wgsl = mesh_wgsl();
        assert!(wgsl.contains(&format!(
            "const KEY_DIR: vec3<f32> = vec3<f32>({:?}, {:?}, {:?});",
            KEY_DIR[0], KEY_DIR[1], KEY_DIR[2]
        )));
        assert!(wgsl.contains("const MIN_ROUGHNESS: f32 = 0.045;"));
        // Every float literal parses as WGSL (a `{:?}` float always has a dot or exponent).
        assert!(!wgsl.contains("vec3<f32>(1, "));
    }
}
