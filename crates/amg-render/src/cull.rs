//! CPU frustum culling and the per-view draw plan (design note §8): visible instances are
//! grouped by (pipeline, mesh) so each mesh is one instanced draw, instances are ordered front
//! to back inside a group and groups are ordered front to back inside a pipeline.

use amg_core::glam::DVec4;
use amg_core::{Mat4, Vec3};

use crate::MeshKey;
use crate::scene::{Instance, InstanceTable, LayerMask};

/// View frustum as inward-facing planes `n·p + d >= 0` in world space, f64.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Frustum {
    planes: [DVec4; 5],
    count: usize,
}

impl Frustum {
    /// Planes of the clip volume `-w <= x,y <= w`, `0 <= z <= w` (reversed-Z) of `view_proj`
    /// (Gribb–Hartmann). Planes with a zero normal (the infinite far plane) are dropped.
    pub fn from_view_proj(view_proj: &Mat4) -> Frustum {
        let r = |i| view_proj.row(i);
        let candidates = [
            r(3) + r(0),
            r(3) - r(0),
            r(3) + r(1),
            r(3) - r(1),
            r(3) - r(2), // depth <= 1: the near side
            r(2),        // depth >= 0: the far side
        ];
        let mut planes = [DVec4::ZERO; 5];
        let mut count = 0;
        for p in candidates {
            let len = p.truncate().length();
            if len > 1e-12 && count < planes.len() {
                planes[count] = p / len;
                count += 1;
            }
        }
        Frustum { planes, count }
    }

    /// False only when the sphere lies completely outside one plane.
    pub fn intersects_sphere(&self, center: Vec3, radius: f64) -> bool {
        self.planes[..self.count]
            .iter()
            .all(|p| p.truncate().dot(center) + p.w >= -radius)
    }
}

/// A run of instances of one mesh drawn with one pipeline.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct DrawGroup {
    pub mesh: MeshKey,
    /// The mesh is shaded from derivatives (no normals, or the view forces flat).
    pub flat: bool,
    /// First entry of [`DrawPlan::slots`].
    pub first: u32,
    pub count: u32,
}

/// The visible instances of one view for one frame.
#[derive(Debug, Default)]
pub(crate) struct DrawPlan {
    pub groups: Vec<DrawGroup>,
    /// Instance slots in draw order; the GPU visibility list.
    pub slots: Vec<u32>,
    /// Drawable instances rejected by the frustum.
    pub culled: u32,
    items: Vec<Item>,
    order: Vec<(usize, usize)>,
}

#[derive(Clone, Copy, Debug)]
struct Item {
    flat: bool,
    mesh: MeshKey,
    dist: f32,
    slot: u32,
}

impl DrawPlan {
    /// Rebuilds the plan. `mesh_has_normals` maps a mesh key to whether it has normals (`None`:
    /// the mesh is gone or has nothing to draw).
    pub fn build(
        &mut self,
        table: &InstanceTable,
        mesh_has_normals: impl Fn(MeshKey) -> Option<bool>,
        frustum: &Frustum,
        eye: Vec3,
        layers: LayerMask,
        force_flat: bool,
    ) {
        self.groups.clear();
        self.slots.clear();
        self.items.clear();
        self.culled = 0;

        let instances: &[Instance] = table.instances();
        for (slot, (inst, info)) in instances.iter().zip(table.info()).enumerate() {
            if !inst.visible || !layers.contains(inst.layer) {
                continue;
            }
            let (Some(mesh), Some(sphere)) = (inst.mesh, info.sphere) else {
                continue;
            };
            let Some(has_normals) = mesh_has_normals(mesh) else {
                continue;
            };
            if !frustum.intersects_sphere(sphere.center, sphere.radius) {
                self.culled += 1;
                continue;
            }
            self.items.push(Item {
                flat: force_flat || !has_normals,
                mesh,
                dist: (sphere.center.distance(eye) - sphere.radius).max(0.0) as f32,
                slot: slot as u32,
            });
        }

        self.items.sort_unstable_by(|a, b| {
            (a.flat, a.mesh)
                .cmp(&(b.flat, b.mesh))
                .then(a.dist.total_cmp(&b.dist))
                .then(a.slot.cmp(&b.slot))
        });

        // Runs of equal (flat, mesh) in `items`, then ordered by pipeline and nearest instance.
        self.order.clear();
        let mut start = 0;
        for i in 1..=self.items.len() {
            let boundary = i == self.items.len()
                || (self.items[i].flat, self.items[i].mesh)
                    != (self.items[start].flat, self.items[start].mesh);
            if boundary {
                if i > start {
                    self.order.push((start, i));
                }
                start = i;
            }
        }
        let items = &self.items;
        self.order.sort_unstable_by(|&(a, _), &(b, _)| {
            items[a]
                .flat
                .cmp(&items[b].flat)
                .then(items[a].dist.total_cmp(&items[b].dist))
                .then(items[a].slot.cmp(&items[b].slot))
        });
        for &(from, to) in &self.order {
            self.groups.push(DrawGroup {
                mesh: items[from].mesh,
                flat: items[from].flat,
                first: self.slots.len() as u32,
                count: (to - from) as u32,
            });
            self.slots.extend(items[from..to].iter().map(|i| i.slot));
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
mod tests {
    use amg_core::{Aabb3, Dir3, Point3, SlotMap, Transform};

    use super::*;
    use crate::{Camera, Projection};

    const SIZE: [u32; 2] = [800, 600];

    fn camera(projection: Projection) -> Camera {
        Camera {
            eye: Point3::new(0.0, -100.0, 0.0),
            target: Point3::ZERO,
            up: Dir3::Z,
            projection,
        }
    }

    fn frustum(cam: &Camera) -> Frustum {
        Frustum::from_view_proj(&(cam.proj_matrix(SIZE) * cam.view_matrix()))
    }

    #[test]
    fn perspective_frustum_accepts_the_view_axis_and_rejects_the_rest() {
        let cam = camera(Projection::Perspective { fov_y: 0.8 });
        let f = frustum(&cam);
        assert!(f.intersects_sphere(Point3::ZERO, 1.0));
        assert!(
            f.intersects_sphere(Point3::new(0.0, 5000.0, 0.0), 1.0),
            "far plane is infinite"
        );
        assert!(
            !f.intersects_sphere(Point3::new(0.0, -200.0, 0.0), 1.0),
            "behind the eye"
        );
        assert!(
            !f.intersects_sphere(Point3::new(500.0, 0.0, 0.0), 1.0),
            "far to the right"
        );
        assert!(
            !f.intersects_sphere(Point3::new(0.0, 0.0, 500.0), 1.0),
            "far above"
        );
        assert!(
            f.intersects_sphere(Point3::new(500.0, 0.0, 0.0), 480.0),
            "radius reaches in"
        );
    }

    #[test]
    fn orthographic_frustum_is_a_box() {
        let cam = camera(Projection::Orthographic { height: 60.0 });
        let f = frustum(&cam);
        assert!(f.intersects_sphere(Point3::new(0.0, 0.0, 29.0), 0.5));
        assert!(!f.intersects_sphere(Point3::new(0.0, 0.0, 31.0), 0.5));
        assert!(
            !f.intersects_sphere(Point3::new(41.0, 0.0, 0.0), 0.5),
            "aspect 4:3, half width 40"
        );
        assert!(f.intersects_sphere(Point3::new(39.0, 0.0, 0.0), 0.5));
        assert!(
            !f.intersects_sphere(Point3::new(0.0, -400.0, 0.0), 1.0),
            "behind the eye"
        );
    }

    #[test]
    fn frustum_far_from_the_origin_keeps_its_precision() {
        let far = Vec3::new(1e6, -2e6, 3e5);
        let cam = Camera {
            eye: far + Vec3::new(0.0, -100.0, 0.0),
            target: far,
            up: Dir3::Z,
            projection: Projection::Perspective { fov_y: 0.8 },
        };
        let f = frustum(&cam);
        assert!(f.intersects_sphere(far, 0.01));
        assert!(!f.intersects_sphere(far + Vec3::new(100.0, 0.0, 0.0), 0.01));
    }

    fn table_of(
        n: usize,
        mesh: impl Fn(usize) -> MeshKey,
        pos: impl Fn(usize) -> Vec3,
    ) -> InstanceTable {
        let mut t = InstanceTable::default();
        for i in 0..n {
            t.insert(Instance::new(mesh(i), Transform::from_translation(pos(i))));
        }
        let mut runs = Vec::new();
        t.flush(
            |_| {
                Some((
                    Point3::ZERO,
                    Aabb3::new(Point3::splat(-0.5), Point3::splat(0.5)),
                ))
            },
            &mut runs,
        );
        t
    }

    fn keys(n: usize) -> Vec<MeshKey> {
        let mut m = SlotMap::<MeshKey, ()>::with_key();
        (0..n).map(|_| m.insert(())).collect()
    }

    #[test]
    fn plan_groups_by_mesh_and_sorts_front_to_back() {
        let meshes = keys(2);
        // Instances 0..6 alternate between the two meshes at increasing depth along +Y.
        let table = table_of(
            6,
            |i| meshes[i % 2],
            |i| Vec3::new(0.0, 10.0 * i as f64, 0.0),
        );
        let cam = camera(Projection::Perspective { fov_y: 0.8 });
        let mut plan = DrawPlan::default();
        plan.build(
            &table,
            |_| Some(true),
            &frustum(&cam),
            cam.eye,
            LayerMask::ALL,
            false,
        );
        assert_eq!(plan.groups.len(), 2);
        assert_eq!(plan.culled, 0);
        for g in &plan.groups {
            assert!(!g.flat);
            let slots = &plan.slots[g.first as usize..(g.first + g.count) as usize];
            assert_eq!(slots.len(), 3);
            assert!(
                slots.windows(2).all(|w| w[0] < w[1]),
                "front to back: {slots:?}"
            );
        }
        // The group holding the nearest instance (slot 0) is drawn first.
        assert_eq!(plan.slots[0], 0);
        assert_eq!(plan.slots.len(), 6);
    }

    #[test]
    fn plan_skips_hidden_wrong_layer_meshless_and_culled_instances() {
        let meshes = keys(2);
        let gone = meshes[1];
        let mut table = table_of(
            6,
            |i| if i == 2 { gone } else { meshes[0] },
            |i| {
                if i == 4 {
                    Vec3::new(5000.0, 0.0, 0.0)
                } else {
                    Vec3::ZERO
                }
            },
        );
        // Slot 0 hidden, slot 1 on the preview layer (masked out), slot 2's mesh is gone,
        // slot 4 is outside the frustum; slots 3 and 5 are drawn.
        table.instance_at_mut(0).visible = false;
        table.instance_at_mut(1).layer = crate::Layer::Preview;
        let cam = camera(Projection::Perspective { fov_y: 0.8 });
        let mut plan = DrawPlan::default();
        plan.build(
            &table,
            |k| (k != gone).then_some(true),
            &frustum(&cam),
            cam.eye,
            LayerMask::ALL.without(crate::Layer::Preview),
            false,
        );
        assert_eq!(plan.slots, vec![3, 5]);
        assert_eq!(plan.culled, 1);
    }

    #[test]
    fn flat_pipeline_groups_come_after_smooth_ones() {
        let meshes = keys(2);
        let table = table_of(
            4,
            |i| meshes[i % 2],
            |i| Vec3::new(0.0, 10.0 * i as f64, 0.0),
        );
        let cam = camera(Projection::Perspective { fov_y: 0.8 });
        let mut plan = DrawPlan::default();
        let flat_mesh = meshes[0];
        plan.build(
            &table,
            |k| Some(k != flat_mesh),
            &frustum(&cam),
            cam.eye,
            LayerMask::ALL,
            false,
        );
        assert_eq!(plan.groups.len(), 2);
        assert!(!plan.groups[0].flat && plan.groups[1].flat);
        plan.build(
            &table,
            |_| Some(true),
            &frustum(&cam),
            cam.eye,
            LayerMask::ALL,
            true,
        );
        assert!(
            plan.groups.iter().all(|g| g.flat),
            "the view forces flat shading"
        );
    }
}
