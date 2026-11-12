//! Render pipeline variants (design note §4, §16): keys, the factory that builds them from WGSL
//! `override` constants, and lazy creation on job threads.
//!
//! The *core set* of a configuration (sample count, depth pre-pass) is built synchronously by
//! `Renderer::new`; every other variant is requested on first need and built on the
//! Interactive pool of an attached [`JobSystem`] (or a short-lived thread without one). Until a
//! variant is ready, draws fall back to the core Studio pipeline of the same geometry kind.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use amg_jobs::{Cancelled, JobSystem, Priority, SpawnOptions};

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
const MESH_WGSL: &str = include_str!("shaders/mesh.wgsl");
const EXTERNAL_WGSL: &str = include_str!("shaders/external.wgsl");
const POST_WGSL: &str = include_str!("shaders/post.wgsl");
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
    },
    /// Depth pre-pass (no fragment stage).
    Depth { flat: bool, double_sided: bool },
    /// Gradient background at the far plane.
    Background,
    /// `ExternalLayer` composite.
    External,
    /// FXAA into the final image (always single-sampled).
    Fxaa,
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
            },
            samples,
        }
    }

    /// Family name for [`PipelineTiming::label`].
    pub fn family(&self) -> &'static str {
        match self.kind {
            PipelineKind::Mesh { .. } => "mesh",
            PipelineKind::Depth { .. } => "depth prepass",
            PipelineKind::Background => "background",
            PipelineKind::External => "external layer",
            PipelineKind::Fxaa => "fxaa",
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
                    kind: PipelineKind::Depth { flat, double_sided },
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

/// Full WGSL of the mesh module (camera + shading + mesh passes).
pub(crate) fn mesh_wgsl() -> String {
    format!(
        "{CAMERA_WGSL}\n{}\n{SHADING_WGSL}\n{MESH_WGSL}",
        shading_constants()
    )
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
}

impl PipelineFactory {
    pub fn new(
        device: &wgpu::Device,
        view_bind_layout: &wgpu::BindGroupLayout,
        external_bind_layout: &wgpu::BindGroupLayout,
        post_bind_layout: &wgpu::BindGroupLayout,
    ) -> PipelineFactory {
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
            mesh_layout: layout("amg mesh", view_bind_layout),
            external_shader: module("amg external layer", EXTERNAL_WGSL.to_owned()),
            external_layout: layout("amg external layer", external_bind_layout),
            post_shader: module("amg post", format!("{FULLSCREEN_WGSL}\n{POST_WGSL}")),
            post_layout: layout("amg post", post_bind_layout),
        }
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
                equal_depth,
            } => {
                let (vertex, primitive) = geometry(flat, double_sided);
                let constants = [("LIGHTING", f64::from(lighting.shader_id()))];
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
            PipelineKind::Depth { flat, double_sided } => {
                let (vertex, primitive) = geometry(flat, double_sided);
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(&label),
                    layout: Some(&self.mesh_layout),
                    vertex,
                    primitive,
                    depth_stencil: depth(true, wgpu::CompareFunction::GreaterEqual),
                    multisample,
                    fragment: None,
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
                double_sided: false
            },
            samples: 1
        }));
        assert!(with_prepass.contains(&PipelineKey::mesh(Lighting::Studio, true, true, true, 1)));
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
