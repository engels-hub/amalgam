//! CPU frustum culling and the per-view draw plan (design note §8): visible instances are
//! grouped by (pipeline, mesh) so each mesh is one instanced draw, instances are ordered front
//! to back inside a group and groups are ordered front to back inside a pipeline.

use amg_core::glam::DVec4;
use amg_core::{Mat4, Vec3};

use crate::MeshKey;
use crate::analysis::GroupShade;
use crate::scene::{Instance, InstanceTable, LayerMask};
use crate::section::{SectionPlane, SectionState, classify};

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

/// What the plan needs to know about a mesh.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct MeshDraw {
    /// Per-vertex normals were uploaded.
    pub has_normals: bool,
    /// Watertight: back faces are culled. Open meshes are drawn double-sided.
    pub closed: bool,
}

/// A bare `has_normals` flag describes a closed mesh.
impl From<bool> for MeshDraw {
    fn from(has_normals: bool) -> Self {
        MeshDraw {
            has_normals,
            closed: true,
        }
    }
}

/// A run of instances of one mesh drawn with one pipeline.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct DrawGroup {
    pub mesh: MeshKey,
    /// The mesh is shaded from derivatives (no normals, or the view forces flat).
    pub flat: bool,
    /// The mesh is open: no back-face culling.
    pub double_sided: bool,
    /// The instances cross a section plane: drawn with the clipped pipelines (and, when
    /// closed, capped).
    pub clipped: bool,
    /// Material or analysis programme, and the field buffers it binds (RND-08).
    pub shade: GroupShade,
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
    /// Drawable instances entirely removed by a section plane.
    pub sectioned_away: u32,
    items: Vec<Item>,
    order: Vec<(usize, usize)>,
}

#[derive(Clone, Copy, Debug)]
struct Item {
    flat: bool,
    double_sided: bool,
    clipped: bool,
    shade: GroupShade,
    mesh: MeshKey,
    dist: f32,
    slot: u32,
}

impl Item {
    /// Pipeline order: smooth before flat, closed before double-sided, unclipped before
    /// clipped, material before analyses.
    fn pipeline(&self) -> (bool, bool, bool, crate::analysis::ShadeKey) {
        (self.flat, self.double_sided, self.clipped, self.shade.key)
    }

    /// What one instanced draw shares: the pipeline, the mesh and the bound fields.
    fn draw_key(&self) -> (bool, bool, bool, GroupShade, MeshKey) {
        (
            self.flat,
            self.double_sided,
            self.clipped,
            self.shade,
            self.mesh,
        )
    }
}

impl DrawPlan {
    /// Rebuilds the plan. `mesh_info` describes a mesh ([`MeshDraw`], or a bare `has_normals`
    /// flag for a closed mesh); `None`: the mesh is gone or has nothing to draw.
    #[cfg(test)]
    pub fn build<M: Into<MeshDraw>>(
        &mut self,
        table: &InstanceTable,
        mesh_info: impl Fn(MeshKey) -> Option<M>,
        frustum: &Frustum,
        eye: Vec3,
        layers: LayerMask,
        force_flat: bool,
    ) {
        self.build_sectioned(
            table,
            mesh_info,
            |_| GroupShade::default(),
            frustum,
            eye,
            layers,
            force_flat,
            &[],
        );
    }

    /// [`DrawPlan::build`] with the view's active section planes: sectioned instances whose
    /// bounding sphere lies entirely on the removed side of a plane are skipped, those that
    /// cross a plane go into clipped groups. `shade_of` resolves an instance's shading (RND-08).
    #[allow(
        clippy::too_many_arguments,
        reason = "plain inputs of one planning step"
    )]
    pub fn build_sectioned<M: Into<MeshDraw>>(
        &mut self,
        table: &InstanceTable,
        mesh_info: impl Fn(MeshKey) -> Option<M>,
        shade_of: impl Fn(&Instance) -> GroupShade,
        frustum: &Frustum,
        eye: Vec3,
        layers: LayerMask,
        force_flat: bool,
        sections: &[SectionPlane],
    ) {
        self.groups.clear();
        self.slots.clear();
        self.items.clear();
        self.culled = 0;
        self.sectioned_away = 0;

        let instances: &[Instance] = table.instances();
        for (slot, (inst, info)) in instances.iter().zip(table.info()).enumerate() {
            if !inst.visible || !layers.contains(inst.layer) {
                continue;
            }
            let (Some(mesh), Some(sphere)) = (inst.mesh, info.sphere) else {
                continue;
            };
            let Some(draw) = mesh_info(mesh).map(Into::into) else {
                continue;
            };
            if !frustum.intersects_sphere(sphere.center, sphere.radius) {
                self.culled += 1;
                continue;
            }
            let state = if inst.sectioned {
                classify(sections, sphere.center, sphere.radius)
            } else {
                SectionState::Kept
            };
            if state == SectionState::Removed {
                self.sectioned_away += 1;
                continue;
            }
            self.items.push(Item {
                flat: force_flat || !draw.has_normals,
                double_sided: !draw.closed,
                clipped: state == SectionState::Cut,
                shade: shade_of(inst),
                mesh,
                dist: (sphere.center.distance(eye) - sphere.radius).max(0.0) as f32,
                slot: slot as u32,
            });
        }

        self.items.sort_unstable_by(|a, b| {
            a.pipeline()
                .cmp(&b.pipeline())
                .then(a.mesh.cmp(&b.mesh))
                .then(a.shade.fields.cmp(&b.shade.fields))
                .then(a.dist.total_cmp(&b.dist))
                .then(a.slot.cmp(&b.slot))
        });

        // Runs of equal (flat, mesh) in `items`, then ordered by pipeline and nearest instance.
        self.order.clear();
        let mut start = 0;
        for i in 1..=self.items.len() {
            let boundary =
                i == self.items.len() || self.items[i].draw_key() != self.items[start].draw_key();
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
                .pipeline()
                .cmp(&items[b].pipeline())
                .then(items[a].dist.total_cmp(&items[b].dist))
                .then(items[a].slot.cmp(&items[b].slot))
        });
        for &(from, to) in &self.order {
            self.groups.push(DrawGroup {
                mesh: items[from].mesh,
                flat: items[from].flat,
                double_sided: items[from].double_sided,
                clipped: items[from].clipped,
                shade: items[from].shade,
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
    fn instances_of_one_mesh_split_by_shading_and_fields() {
        use crate::FieldKey;
        use crate::analysis::{GroupShade, ShadeKey, ShadeMode};

        let meshes = keys(1);
        let mut fields = SlotMap::<FieldKey, ()>::with_key();
        let (fa, fb) = (fields.insert(()), fields.insert(()));
        let table = table_of(6, |_| meshes[0], |i| Vec3::new(0.0, 10.0 * i as f64, 0.0));
        let field = |k| GroupShade {
            key: ShadeKey {
                mode: ShadeMode::Field,
                scalar: true,
                ..ShadeKey::default()
            },
            fields: [Some(k), None],
        };
        // Slots 0 and 3: material; 1 and 4: field A; 2: field B; 5: zebra.
        let shade = |slot: usize| match slot {
            1 | 4 => field(fa),
            2 => field(fb),
            5 => GroupShade {
                key: ShadeKey {
                    mode: ShadeMode::Zebra,
                    ..ShadeKey::default()
                },
                fields: [None, None],
            },
            _ => GroupShade::default(),
        };
        let cam = camera(Projection::Perspective { fov_y: 0.8 });
        let mut plan = DrawPlan::default();
        let instances = table.instances();
        plan.build_sectioned(
            &table,
            |_| Some(true),
            |inst| {
                let slot = instances
                    .iter()
                    .position(|i| std::ptr::eq(i, inst))
                    .unwrap();
                shade(slot)
            },
            &frustum(&cam),
            cam.eye,
            LayerMask::ALL,
            false,
            &[],
        );
        let groups: Vec<(ShadeMode, [Option<FieldKey>; 2], Vec<u32>)> = plan
            .groups
            .iter()
            .map(|g| {
                (
                    g.shade.key.mode,
                    g.shade.fields,
                    plan.slots[g.first as usize..(g.first + g.count) as usize].to_vec(),
                )
            })
            .collect();
        assert_eq!(groups.len(), 4, "{groups:?}");
        assert_eq!(groups[0], (ShadeMode::Material, [None, None], vec![0, 3]));
        assert!(groups.contains(&(ShadeMode::Field, [Some(fa), None], vec![1, 4])));
        assert!(groups.contains(&(ShadeMode::Field, [Some(fb), None], vec![2])));
        assert!(groups.contains(&(ShadeMode::Zebra, [None, None], vec![5])));
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
    fn section_planes_split_kept_cut_and_removed_instances() {
        use amg_core::Plane;

        let meshes = keys(1);
        // Instances at z = -10, 0 and +10 (unit spheres); the plane z = 0 removes z > 0.
        let mut table = table_of(
            4,
            |_| meshes[0],
            |i| Vec3::new(0.0, 0.0, 10.0 * (i as f64 - 1.0)),
        );
        // Slot 3 sits above the plane too but is not sectioned.
        table.instance_at_mut(3).sectioned = false;
        let cam = camera(Projection::Orthographic { height: 100.0 });
        let plane = SectionPlane::new(Plane::XY);
        let mut plan = DrawPlan::default();
        plan.build_sectioned(
            &table,
            |_| Some(true),
            |_| GroupShade::default(),
            &frustum(&cam),
            cam.eye,
            LayerMask::ALL,
            false,
            &[plane],
        );
        assert_eq!(plan.sectioned_away, 1, "slot 2 is entirely removed");
        let clipped: Vec<u32> = plan
            .groups
            .iter()
            .filter(|g| g.clipped)
            .flat_map(|g| plan.slots[g.first as usize..(g.first + g.count) as usize].to_vec())
            .collect();
        let kept: Vec<u32> = plan
            .groups
            .iter()
            .filter(|g| !g.clipped)
            .flat_map(|g| plan.slots[g.first as usize..(g.first + g.count) as usize].to_vec())
            .collect();
        assert_eq!(clipped, vec![1]);
        let mut kept = kept;
        kept.sort_unstable();
        assert_eq!(kept, vec![0, 3]);
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
