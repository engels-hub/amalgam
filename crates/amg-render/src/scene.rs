//! The renderer-owned scene mirror. RND-01 keeps the minimum the spike draws; RND-03 adds the
//! rest of the S10 surface (edges, fields, sections, dirty tracking, culling).

use amg_core::{Aabb3, Point3, SlotMap, Transform};

use crate::upload::GpuMesh;
use crate::{InstanceKey, MeshKey};

/// A mesh placed in the world.
#[derive(Clone, Debug)]
pub struct Instance {
    /// The mesh to draw.
    pub mesh: Option<MeshKey>,
    /// World-from-mesh-local (the mesh origin is handled internally), f64.
    pub transform: Transform,
    /// Whether the instance is drawn.
    pub visible: bool,
    /// Surface appearance.
    pub material: Material,
}

impl Instance {
    /// A visible instance of `mesh` with the default material.
    pub fn new(mesh: MeshKey, transform: Transform) -> Instance {
        Instance {
            mesh: Some(mesh),
            transform,
            visible: true,
            material: Material::default(),
        }
    }
}

/// Surface appearance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    /// Linear RGBA.
    pub base_color: [f32; 4],
}

impl Default for Material {
    fn default() -> Self {
        Material {
            base_color: [0.62, 0.66, 0.72, 1.0],
        }
    }
}

/// Renderer-owned mirror of what to draw. Mutations are cheap (no geometry work).
#[derive(Debug, Default)]
pub struct RenderScene {
    meshes: SlotMap<MeshKey, GpuMesh>,
    instances: SlotMap<InstanceKey, Instance>,
    revision: u64,
}

impl RenderScene {
    /// Adds a finished upload.
    pub fn insert_mesh(&mut self, m: GpuMesh) -> MeshKey {
        self.revision += 1;
        self.meshes.insert(m)
    }

    /// Removes a mesh; instances that use it stop drawing. Returns whether it existed.
    pub fn remove_mesh(&mut self, k: MeshKey) -> bool {
        self.revision += 1;
        self.meshes.remove(k).is_some()
    }

    /// Adds an instance.
    pub fn add_instance(&mut self, inst: Instance) -> InstanceKey {
        self.revision += 1;
        self.instances.insert(inst)
    }

    /// The instance, if it exists.
    pub fn instance(&self, k: InstanceKey) -> Option<&Instance> {
        self.instances.get(k)
    }

    /// Mutable access to the instance.
    pub fn instance_mut(&mut self, k: InstanceKey) -> Option<&mut Instance> {
        self.revision += 1;
        self.instances.get_mut(k)
    }

    /// Removes an instance. Returns whether it existed.
    pub fn remove_instance(&mut self, k: InstanceKey) -> bool {
        self.revision += 1;
        self.instances.remove(k).is_some()
    }

    /// World bounds of the visible instances.
    pub fn visible_bounds(&self) -> Aabb3 {
        let mut out = Aabb3::EMPTY;
        for inst in self.instances.values().filter(|i| i.visible) {
            let Some(mesh) = inst.mesh.and_then(|k| self.meshes.get(k)) else {
                continue;
            };
            let b = mesh.bounds();
            for x in [b.min.x, b.max.x] {
                for y in [b.min.y, b.max.y] {
                    for z in [b.min.z, b.max.z] {
                        out.include_point(inst.transform.transform_point3(Point3::new(x, y, z)));
                    }
                }
            }
        }
        out
    }

    /// VRAM used by the meshes.
    pub fn gpu_bytes(&self) -> u64 {
        self.meshes.values().map(GpuMesh::gpu_bytes).sum()
    }

    /// Bumped on every mutation.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub(crate) fn mesh(&self, k: MeshKey) -> Option<&GpuMesh> {
        self.meshes.get(k)
    }

    pub(crate) fn instances(&self) -> impl Iterator<Item = &Instance> {
        self.instances.values()
    }
}
