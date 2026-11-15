//! The renderer-owned scene mirror: meshes, edge and point sets, instances with dirty
//! tracking, layers and the external layer. RND-03 covers the mesh path, RND-12 the edges and
//! points, RND-06 the section planes, RND-08 the fields and per-instance [`Shading`]; overlays
//! arrive with their own work package.

use amg_core::{Aabb3, Mat3, Point3, SlotMap, Transform, Vec3};

use crate::analysis::{
    AnalysisOutput, AnalysisRecord, Animation, GpuField, GroupShade, Shading,
    build_analysis_record, deformation_of, group_shade,
};
use crate::lines::{GpuEdges, GpuPoints, LineRecord, LineStyle, PointStyle, build_line_record};
use crate::section::{MAX_SECTIONS, SectionPlane};
use crate::upload::GpuMesh;
use crate::{EdgesKey, FieldKey, InstanceKey, MeshKey, PickId, PointsKey, SectionKey};

/// What an instance belongs to. Views choose which layers they draw ([`LayerMask`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Layer {
    /// Regular model geometry.
    #[default]
    Model,
    /// A preview of an operation result.
    Preview,
    /// Translucent context geometry (drawn by the transparency pass, RND-05).
    Ghost,
    /// Construction geometry.
    Construction,
}

impl Layer {
    fn bit(self) -> u8 {
        1 << (self as u8)
    }
}

/// The set of layers a view draws.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LayerMask(u8);

impl LayerMask {
    /// Every layer.
    pub const ALL: LayerMask = LayerMask(0b1111);
    /// No layer.
    pub const NONE: LayerMask = LayerMask(0);

    /// Only `layer`.
    pub fn only(layer: Layer) -> LayerMask {
        LayerMask(layer.bit())
    }

    /// This set plus `layer`.
    #[must_use]
    pub fn with(self, layer: Layer) -> LayerMask {
        LayerMask(self.0 | layer.bit())
    }

    /// This set minus `layer`.
    #[must_use]
    pub fn without(self, layer: Layer) -> LayerMask {
        LayerMask(self.0 & !layer.bit())
    }

    /// Whether `layer` is in the set.
    pub fn contains(self, layer: Layer) -> bool {
        self.0 & layer.bit() != 0
    }
}

impl Default for LayerMask {
    fn default() -> Self {
        LayerMask::ALL
    }
}

/// A body placed in the world: a mesh, its edges and its points, any of them optional.
#[derive(Clone, Debug)]
pub struct Instance {
    /// The mesh to draw. Several instances may share one mesh.
    pub mesh: Option<MeshKey>,
    /// Edge polylines (B-rep edges, or a mesh body's feature edges), drawn per the view's
    /// [`crate::VisualStyle`] with [`Instance::edge_style`].
    pub edges: Option<EdgesKey>,
    /// Points (B-rep vertices), drawn in every visual style with [`Instance::point_style`].
    pub points: Option<PointsKey>,
    /// World-from-mesh-local (the mesh origin is handled internally), f64.
    pub transform: Transform,
    /// Whether the instance is drawn.
    pub visible: bool,
    /// The layer it belongs to.
    pub layer: Layer,
    /// Surface appearance.
    pub material: Material,
    /// Style of the edges (and of the triangle wireframe).
    pub edge_style: LineStyle,
    /// Style of the points.
    pub point_style: PointStyle,
    /// A mesh body: the wireframe styles draw the triangle wireframe of `mesh`
    /// (`POLYGON_MODE_LINE`), falling back to `edges` (feature edges) without the feature.
    /// False (B-rep bodies): the wireframe styles draw `edges`.
    pub mesh_wireframe: bool,
    /// Clipped by the scene's section planes (RND-06). Default true. Closed meshes cut by a
    /// plane show caps.
    pub sectioned: bool,
    /// How the surface is coloured (RND-08): material, a scalar field (with optional
    /// deformation) or a live analysis. Changing it rewrites the instance's records only.
    pub shading: Shading,
}

impl Instance {
    /// A visible instance of `mesh` on the model layer with the default material and styles.
    pub fn new(mesh: MeshKey, transform: Transform) -> Instance {
        Instance {
            mesh: Some(mesh),
            ..Instance::empty(transform)
        }
    }

    /// A visible instance of an edge set only (wire bodies, edge previews).
    pub fn new_edges(edges: EdgesKey, transform: Transform) -> Instance {
        Instance {
            edges: Some(edges),
            ..Instance::empty(transform)
        }
    }

    /// A visible instance of a point set only.
    pub fn new_points(points: PointsKey, transform: Transform) -> Instance {
        Instance {
            points: Some(points),
            ..Instance::empty(transform)
        }
    }

    fn empty(transform: Transform) -> Instance {
        Instance {
            mesh: None,
            edges: None,
            points: None,
            transform,
            visible: true,
            layer: Layer::Model,
            material: Material::default(),
            edge_style: LineStyle::default(),
            point_style: PointStyle::default(),
            mesh_wireframe: false,
            sectioned: true,
            shading: Shading::Material,
        }
    }
}

/// Surface appearance (PBR-lite, design note §4). Changing it rewrites only the instance's
/// 128-byte record, never geometry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    /// Linear RGBA. Alpha below 1 is translucent (drawn by the transparency pass, RND-05).
    pub base_color: [f32; 4],
    /// 0 = dielectric, 1 = metal (clamped to `[0, 1]`).
    pub metallic: f32,
    /// Perceptual roughness, `[0, 1]`; values below 0.045 are treated as 0.045.
    pub roughness: f32,
    /// Section cap colour (RND-06); `None` = darkened base colour.
    pub cap_color: Option<[f32; 3]>,
}

impl Default for Material {
    fn default() -> Self {
        Material {
            base_color: [0.62, 0.66, 0.72, 1.0],
            metallic: 0.0,
            roughness: 0.5,
            cap_color: None,
        }
    }
}

impl Material {
    /// `pack2x16unorm(metallic, roughness)` as the shader unpacks it.
    pub(crate) fn packed_metallic_roughness(&self) -> u32 {
        let q = |v: f32| (v.clamp(0.0, 1.0) * 65535.0).round() as u32;
        q(self.metallic) | (q(self.roughness) << 16)
    }
}

/// Foreign colour and depth composited into the opaque pass (IMP-05 ray march; design note §14).
///
/// Both textures have the view's size. Pixels whose depth is 0 are empty.
#[derive(Clone, Debug)]
pub struct ExternalLayer {
    /// `Rgba16Float`, premultiplied linear colour, with `TEXTURE_BINDING`.
    pub color: wgpu::TextureView,
    /// `R32Float` reversed-Z NDC depth of the view's projection (0 = empty), with
    /// `TEXTURE_BINDING`.
    pub depth: wgpu::TextureView,
    /// Size of both textures. A layer whose size differs from the view is skipped.
    pub size_px: [u32; 2],
    /// The pick id the adapter registered for the layer (used by picking, RND-04).
    pub pick: PickId,
}

/// One 128-byte record of the instance storage buffer (`shaders/mesh.wgsl`).
///
/// Camera independent: the eye is subtracted in the shader from the split translation, so a
/// moving camera never rewrites it, and a changed instance rewrites exactly this record.
/// The `w` of the three normal-matrix columns is reserved (pick id and flags, RND-04/05).
/// RND-02 put the packed metallic/roughness in the former `t_lo.w`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct InstanceRecord {
    /// Columns of the world-from-local linear part; `w` is the high part of the translation.
    pub lin: [[f32; 4]; 3],
    /// Low part of the translation.
    pub t_lo: [f32; 3],
    /// `pack2x16unorm(metallic, roughness)`.
    pub material: u32,
    /// Columns of the normal matrix (`xyz`), `w` reserved.
    pub nrm: [[f32; 4]; 3],
    /// Linear RGBA.
    pub color: [f32; 4],
}

/// Size of one instance record, bytes.
pub(crate) const RECORD_BYTES: u64 = size_of::<InstanceRecord>() as u64;

/// World bounding sphere of an instance, f64.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Sphere {
    pub center: Vec3,
    pub radius: f64,
}

/// What culling and sorting need per instance.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct DrawInfo {
    /// `None` when the instance has no live non-empty mesh.
    pub sphere: Option<Sphere>,
}

/// Origin and bounds (origin applied) of the geometry an instance references; `None` for a
/// missing key.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Geometry {
    pub mesh: Option<(Point3, Aabb3)>,
    pub edges: Option<(Point3, Aabb3)>,
    pub points: Option<(Point3, Aabb3)>,
    /// Largest local displacement of the instance's deformation (`|scale| · max |d|`), mm.
    pub deform_pad: f64,
}

/// Builds the GPU record and world sphere of `inst`. `mesh` supplies the origin and bounds.
#[cfg(test)]
pub(crate) fn build_record(
    inst: &Instance,
    mesh: Option<(Point3, Aabb3)>,
) -> (InstanceRecord, DrawInfo) {
    let (record, _, _, info) = build_records(
        inst,
        Geometry {
            mesh,
            ..Geometry::default()
        },
    );
    (record, info)
}

/// Builds the instance record, the line record, the analysis record and the world sphere of
/// `inst`. The record is built around the mesh origin (else the edge-set, else the point-set
/// origin); the line record holds the other sets' offsets from it. The sphere encloses all
/// three, grown by the largest deformation.
pub(crate) fn build_records(
    inst: &Instance,
    geometry: Geometry,
) -> (InstanceRecord, LineRecord, AnalysisRecord, DrawInfo) {
    let origin = geometry
        .mesh
        .or(geometry.edges)
        .or(geometry.points)
        .map_or(Point3::ZERO, |(o, _)| o);
    let bounds = [geometry.mesh, geometry.edges, geometry.points]
        .into_iter()
        .flatten()
        .map(|(_, b)| b)
        .reduce(|a, b| a.union(&b));
    let m = inst.transform.matrix3;
    let translation = inst.transform.transform_point3(origin);
    let hi = translation.as_vec3();
    let lo = (translation - hi.as_dvec3()).as_vec3();
    let col = |c: Vec3, h: f32| [c.x as f32, c.y as f32, c.z as f32, h];
    let normal = Mat3::from(m).inverse().transpose();
    let ncol = |c: Vec3| [c.x as f32, c.y as f32, c.z as f32, 0.0];
    let record = InstanceRecord {
        lin: [
            col(m.x_axis, hi.x),
            col(m.y_axis, hi.y),
            col(m.z_axis, hi.z),
        ],
        t_lo: [lo.x, lo.y, lo.z],
        material: inst.material.packed_metallic_roughness(),
        nrm: [
            ncol(normal.x_axis),
            ncol(normal.y_axis),
            ncol(normal.z_axis),
        ],
        color: inst.material.base_color,
    };
    let lines = build_line_record(
        inst,
        origin,
        geometry.edges.map(|(o, _)| o),
        geometry.points.map(|(o, _)| o),
    );
    let analysis = build_analysis_record(&inst.shading, &inst.transform, translation);
    // A deformed vertex moves at most `deform_pad` in local units; the linear part stretches
    // that by at most its largest column length (an upper bound of its norm for these uses).
    let stretch = [m.x_axis, m.y_axis, m.z_axis]
        .iter()
        .map(|c| c.length())
        .fold(0.0, f64::max)
        * 3f64.sqrt();
    let pad = geometry.deform_pad * stretch;
    let sphere = bounds.filter(|b| !b.is_empty()).map(|b| {
        let mut world = Aabb3::EMPTY;
        for x in [b.min.x, b.max.x] {
            for y in [b.min.y, b.max.y] {
                for z in [b.min.z, b.max.z] {
                    world.include_point(inst.transform.transform_point3(Point3::new(x, y, z)));
                }
            }
        }
        let center = world.center();
        let radius = [world.min, world.max]
            .iter()
            .map(|c| c.distance(center))
            .fold(0.0, f64::max)
            + pad;
        Sphere { center, radius }
    });
    (record, lines, analysis, DrawInfo { sphere })
}

/// Dense instance storage: slot `i` of every vector describes the same instance, and the slot
/// index is the record's position in the GPU buffer. Removal swaps the last slot in.
#[derive(Debug, Default)]
pub(crate) struct InstanceTable {
    index: SlotMap<InstanceKey, u32>,
    keys: Vec<InstanceKey>,
    instances: Vec<Instance>,
    records: Vec<InstanceRecord>,
    line_records: Vec<LineRecord>,
    analysis_records: Vec<AnalysisRecord>,
    info: Vec<DrawInfo>,
    dirty_flag: Vec<bool>,
    dirty: Vec<u32>,
}

impl InstanceTable {
    pub fn len(&self) -> usize {
        self.keys.len()
    }

    pub fn insert(&mut self, inst: Instance) -> InstanceKey {
        let slot = self.keys.len() as u32;
        let key = self.index.insert(slot);
        self.keys.push(key);
        self.instances.push(inst);
        self.records.push(InstanceRecord::default());
        self.line_records.push(LineRecord::default());
        self.analysis_records.push(AnalysisRecord::default());
        self.info.push(DrawInfo::default());
        self.dirty_flag.push(false);
        self.mark(slot);
        key
    }

    /// The GPU record slot of `k`.
    pub fn slot(&self, k: InstanceKey) -> Option<u32> {
        self.index.get(k).copied()
    }

    pub fn get(&self, k: InstanceKey) -> Option<&Instance> {
        self.index.get(k).map(|&s| &self.instances[s as usize])
    }

    /// Marks the instance dirty and returns it.
    pub fn get_mut(&mut self, k: InstanceKey) -> Option<&mut Instance> {
        let slot = *self.index.get(k)?;
        self.mark(slot);
        Some(&mut self.instances[slot as usize])
    }

    pub fn remove(&mut self, k: InstanceKey) -> bool {
        let Some(slot) = self.index.remove(k) else {
            return false;
        };
        let slot = slot as usize;
        let last = self.keys.len() - 1;
        self.keys.swap_remove(slot);
        self.instances.swap_remove(slot);
        self.records.swap_remove(slot);
        self.line_records.swap_remove(slot);
        self.analysis_records.swap_remove(slot);
        self.info.swap_remove(slot);
        self.dirty_flag.swap_remove(slot);
        if slot != last {
            // The former last instance now lives in `slot`; its record must be re-sent.
            if let Some(entry) = self.index.get_mut(self.keys[slot]) {
                *entry = slot as u32;
            }
            self.dirty_flag[slot] = false;
            self.mark(slot as u32);
        }
        true
    }

    fn mark(&mut self, slot: u32) {
        if !std::mem::replace(&mut self.dirty_flag[slot as usize], true) {
            // Removals leave stale entries behind; drop them before the list outgrows the
            // scene (a long run of add/remove between two frames).
            if self.dirty.len() >= 64 && self.dirty.len() > 2 * self.keys.len() {
                self.compact_dirty();
            }
            self.dirty.push(slot);
        }
    }

    /// Marks every instance matching `pred` dirty.
    pub fn mark_where(&mut self, pred: impl Fn(&Instance) -> bool) {
        for slot in 0..self.instances.len() {
            if pred(&self.instances[slot]) {
                self.mark(slot as u32);
            }
        }
    }

    /// Keeps each still-dirty slot once, in ascending order.
    fn compact_dirty(&mut self) {
        let (flags, len) = (&self.dirty_flag, self.keys.len());
        self.dirty
            .retain(|&s| (s as usize) < len && flags[s as usize]);
        self.dirty.sort_unstable();
        self.dirty.dedup();
    }

    /// [`InstanceTable::flush_geometry`] for instances with a mesh only.
    #[cfg(test)]
    pub fn flush(
        &mut self,
        mesh: impl Fn(MeshKey) -> Option<(Point3, Aabb3)>,
        runs: &mut Vec<std::ops::Range<u32>>,
    ) {
        self.flush_geometry(
            |inst| Geometry {
                mesh: inst.mesh.and_then(&mesh),
                ..Geometry::default()
            },
            runs,
        );
    }

    /// Recomputes the records of the dirty slots and writes the contiguous runs of changed
    /// slots to `runs` (sorted, as slot ranges). Clears the dirty set.
    pub fn flush_geometry(
        &mut self,
        geometry: impl Fn(&Instance) -> Geometry,
        runs: &mut Vec<std::ops::Range<u32>>,
    ) {
        runs.clear();
        self.compact_dirty();
        for &slot in &self.dirty {
            let i = slot as usize;
            let inst = &self.instances[i];
            let (record, lines, analysis, info) = build_records(inst, geometry(inst));
            self.records[i] = record;
            self.line_records[i] = lines;
            self.analysis_records[i] = analysis;
            self.info[i] = info;
            self.dirty_flag[i] = false;
            match runs.last_mut() {
                Some(r) if r.end == slot => r.end += 1,
                _ => runs.push(slot..slot + 1),
            }
        }
        self.dirty.clear();
    }

    #[cfg(test)]
    pub fn instance_at_mut(&mut self, slot: usize) -> &mut Instance {
        &mut self.instances[slot]
    }

    pub fn records(&self) -> &[InstanceRecord] {
        &self.records
    }

    pub fn line_records(&self) -> &[LineRecord] {
        &self.line_records
    }

    pub fn analysis_records(&self) -> &[AnalysisRecord] {
        &self.analysis_records
    }

    pub fn instances(&self) -> &[Instance] {
        &self.instances
    }

    pub fn info(&self) -> &[DrawInfo] {
        &self.info
    }
}

/// Renderer-owned mirror of what to draw. Mutations are cheap (no geometry work); the GPU
/// instance records of changed instances are rewritten by the next `encode`.
#[derive(Debug, Default)]
pub struct RenderScene {
    meshes: SlotMap<MeshKey, GpuMesh>,
    edges: SlotMap<EdgesKey, GpuEdges>,
    points: SlotMap<PointsKey, GpuPoints>,
    fields: SlotMap<FieldKey, GpuField>,
    table: InstanceTable,
    external: Option<ExternalLayer>,
    sections: SlotMap<SectionKey, SectionPlane>,
    /// Section keys in the order they were added (which planes are active when more than
    /// [`MAX_SECTIONS`] are enabled).
    section_order: Vec<SectionKey>,
    revision: u64,
}

impl RenderScene {
    /// Adds a finished upload. Constant time; no GPU work.
    pub fn insert_mesh(&mut self, m: GpuMesh) -> MeshKey {
        self.revision += 1;
        self.meshes.insert(m)
    }

    /// Removes a mesh; instances that use it stop drawing. Returns whether it existed. The VRAM
    /// is freed once frames in flight finish.
    pub fn remove_mesh(&mut self, k: MeshKey) -> bool {
        self.revision += 1;
        self.meshes.remove(k).is_some()
    }

    /// The mesh, if it exists.
    pub fn mesh_ref(&self, k: MeshKey) -> Option<&GpuMesh> {
        self.meshes.get(k)
    }

    /// Adds a finished edge upload. Constant time; no GPU work.
    pub fn insert_edges(&mut self, e: GpuEdges) -> EdgesKey {
        self.revision += 1;
        self.edges.insert(e)
    }

    /// Removes an edge set; instances that use it stop drawing edges. Returns whether it
    /// existed.
    pub fn remove_edges(&mut self, k: EdgesKey) -> bool {
        self.revision += 1;
        self.edges.remove(k).is_some()
    }

    /// The edge set, if it exists.
    pub fn edges_ref(&self, k: EdgesKey) -> Option<&GpuEdges> {
        self.edges.get(k)
    }

    /// Adds a finished point upload. Constant time; no GPU work.
    pub fn insert_points(&mut self, p: GpuPoints) -> PointsKey {
        self.revision += 1;
        self.points.insert(p)
    }

    /// Removes a point set. Returns whether it existed.
    pub fn remove_points(&mut self, k: PointsKey) -> bool {
        self.revision += 1;
        self.points.remove(k).is_some()
    }

    /// The point set, if it exists.
    pub fn points_ref(&self, k: PointsKey) -> Option<&GpuPoints> {
        self.points.get(k)
    }

    /// Adds a finished field upload ([`crate::upload_scalar_field`],
    /// [`crate::upload_vector_field`]). Constant time; no GPU work.
    pub fn insert_field(&mut self, f: GpuField) -> FieldKey {
        self.revision += 1;
        self.fields.insert(f)
    }

    /// Removes a field; instances bound to it fall back (a missing scalar field shows the
    /// material, a missing displacement field draws undeformed). Returns whether it existed.
    pub fn remove_field(&mut self, k: FieldKey) -> bool {
        self.revision += 1;
        if self.fields.remove(k).is_none() {
            return false;
        }
        // Deformation pads the bounding sphere: re-derive it for the instances that used it.
        self.table
            .mark_where(|i| deformation_of(&i.shading).is_some_and(|d| d.field == k));
        true
    }

    /// The field, if it exists.
    pub fn field_ref(&self, k: FieldKey) -> Option<&GpuField> {
        self.fields.get(k)
    }

    /// Number of live fields.
    pub fn field_count(&self) -> usize {
        self.fields.len()
    }

    /// Whether a visible instance has an animated deformation ([`Animation::Sine`] or
    /// [`Animation::Ramp`]): the app should keep repainting and advancing
    /// [`crate::Renderer::set_animation_time`].
    pub fn is_animating(&self) -> bool {
        self.table.instances().iter().any(|i| {
            i.visible
                && deformation_of(&i.shading).is_some_and(|d| {
                    d.animation != Animation::Static && self.fields.contains_key(d.field)
                })
        })
    }

    /// How `inst` is drawn: its shading resolved against its mesh and the live fields.
    pub(crate) fn group_shade(&self, inst: &Instance, out: AnalysisOutput) -> GroupShade {
        let vertices = inst
            .mesh
            .and_then(|k| self.meshes.get(k))
            .map_or(0, GpuMesh::vertex_count);
        group_shade(
            &inst.shading,
            vertices,
            |k| self.fields.get(k).map(|f| (f.kind(), f.vertex_count())),
            out,
        )
    }

    pub(crate) fn field(&self, k: FieldKey) -> Option<&GpuField> {
        self.fields.get(k)
    }

    /// Adds an instance. Its GPU record is written by the next `encode`.
    pub fn add_instance(&mut self, inst: Instance) -> InstanceKey {
        self.revision += 1;
        self.table.insert(inst)
    }

    /// The instance, if it exists.
    pub fn instance(&self, k: InstanceKey) -> Option<&Instance> {
        self.table.get(k)
    }

    /// Mutable access to the instance. Marks it dirty: the next `encode` rewrites its 128 B
    /// record only.
    pub fn instance_mut(&mut self, k: InstanceKey) -> Option<&mut Instance> {
        self.revision += 1;
        self.table.get_mut(k)
    }

    /// Removes an instance. Returns whether it existed.
    pub fn remove_instance(&mut self, k: InstanceKey) -> bool {
        self.revision += 1;
        self.table.remove(k)
    }

    /// Number of live instances.
    pub fn instance_count(&self) -> usize {
        self.table.len()
    }

    /// Number of live meshes.
    pub fn mesh_count(&self) -> usize {
        self.meshes.len()
    }

    /// Sets or clears the external layer (composited after the opaque meshes).
    pub fn set_external_layer(&mut self, layer: Option<ExternalLayer>) {
        self.revision += 1;
        self.external = layer;
    }

    /// Adds a section plane. At most [`MAX_SECTIONS`] enabled planes apply at once: the first
    /// ones added; further enabled planes are ignored until one of those is disabled or removed.
    pub fn add_section(&mut self, s: SectionPlane) -> SectionKey {
        self.revision += 1;
        let key = self.sections.insert(s);
        self.section_order.push(key);
        key
    }

    /// The section plane, if it exists.
    pub fn section(&self, k: SectionKey) -> Option<&SectionPlane> {
        self.sections.get(k)
    }

    /// Mutable access to a section plane (takes effect on the next `encode`).
    pub fn section_mut(&mut self, k: SectionKey) -> Option<&mut SectionPlane> {
        self.revision += 1;
        self.sections.get_mut(k)
    }

    /// Removes a section plane. Returns whether it existed.
    pub fn remove_section(&mut self, k: SectionKey) -> bool {
        self.revision += 1;
        self.section_order.retain(|&o| o != k);
        self.sections.remove(k).is_some()
    }

    /// Number of section planes (enabled or not).
    pub fn section_count(&self) -> usize {
        self.sections.len()
    }

    /// The planes that apply: the enabled ones in the order they were added, at most
    /// [`MAX_SECTIONS`].
    pub fn active_sections(&self) -> Vec<SectionPlane> {
        self.section_order
            .iter()
            .filter_map(|&k| self.sections.get(k))
            .filter(|s| s.enabled)
            .take(MAX_SECTIONS)
            .copied()
            .collect()
    }

    /// The external layer, if any.
    pub fn external_layer(&self) -> Option<&ExternalLayer> {
        self.external.as_ref()
    }

    /// World bounds of the visible instances (meshes, edges and points).
    pub fn visible_bounds(&self) -> Aabb3 {
        let mut out = Aabb3::EMPTY;
        for inst in self.table.instances().iter().filter(|i| i.visible) {
            let g = self.geometry(inst);
            let Some(b) = [g.mesh, g.edges, g.points]
                .into_iter()
                .flatten()
                .map(|(_, b)| b)
                .reduce(|a, b| a.union(&b))
            else {
                continue;
            };
            if b.is_empty() {
                continue;
            }
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

    /// VRAM used by the meshes, edge sets, point sets and fields. The renderer's own instance and
    /// visibility buffers are not included.
    pub fn gpu_bytes(&self) -> u64 {
        self.meshes.values().map(GpuMesh::gpu_bytes).sum::<u64>()
            + self.edges.values().map(GpuEdges::gpu_bytes).sum::<u64>()
            + self.points.values().map(GpuPoints::gpu_bytes).sum::<u64>()
            + self.fields.values().map(GpuField::gpu_bytes).sum::<u64>()
    }

    /// Number of live edge sets.
    pub fn edges_count(&self) -> usize {
        self.edges.len()
    }

    /// Number of live point sets.
    pub fn points_count(&self) -> usize {
        self.points.len()
    }

    fn geometry(&self, inst: &Instance) -> Geometry {
        geometry_of(inst, &self.meshes, &self.edges, &self.points, &self.fields)
    }

    /// Bumped on every mutation.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub(crate) fn mesh(&self, k: MeshKey) -> Option<&GpuMesh> {
        self.meshes.get(k)
    }

    pub(crate) fn table(&self) -> &InstanceTable {
        &self.table
    }

    pub(crate) fn edge_set(&self, k: EdgesKey) -> Option<&GpuEdges> {
        self.edges.get(k)
    }

    pub(crate) fn point_set(&self, k: PointsKey) -> Option<&GpuPoints> {
        self.points.get(k)
    }

    /// Recomputes dirty records; see [`InstanceTable::flush_geometry`].
    pub(crate) fn flush_instances(&mut self, runs: &mut Vec<std::ops::Range<u32>>) {
        let Self {
            meshes,
            edges,
            points,
            fields,
            table,
            ..
        } = self;
        table.flush_geometry(
            |inst| geometry_of(inst, meshes, edges, points, fields),
            runs,
        );
    }
}

fn geometry_of(
    inst: &Instance,
    meshes: &SlotMap<MeshKey, GpuMesh>,
    edges: &SlotMap<EdgesKey, GpuEdges>,
    points: &SlotMap<PointsKey, GpuPoints>,
    fields: &SlotMap<FieldKey, GpuField>,
) -> Geometry {
    let deform_pad = deformation_of(&inst.shading)
        .and_then(|d| {
            fields
                .get(d.field)
                .map(|f| f64::from(d.scale.abs()) * f64::from(f.max_displacement()))
        })
        .unwrap_or(0.0);
    Geometry {
        deform_pad,
        mesh: inst
            .mesh
            .and_then(|k| meshes.get(k))
            .map(|m| (m.origin, m.bounds())),
        edges: inst
            .edges
            .and_then(|k| edges.get(k))
            .map(|e| (e.origin, e.bounds())),
        points: inst
            .points
            .and_then(|k| points.get(k))
            .map(|p| (p.origin, p.bounds())),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
#[allow(
    clippy::float_cmp,
    reason = "exact translations survive the table bookkeeping"
)]
mod tests {
    use amg_core::Aabb3;

    use super::*;

    fn key() -> MeshKey {
        // A mesh key never dereferenced: the table only stores it.
        SlotMap::<MeshKey, ()>::with_key().insert(())
    }

    fn unit_mesh(origin: Point3) -> Option<(Point3, Aabb3)> {
        Some((
            origin,
            Aabb3::new(origin - Vec3::splat(0.5), origin + Vec3::splat(0.5)),
        ))
    }

    fn flush(t: &mut InstanceTable) -> Vec<std::ops::Range<u32>> {
        let mut runs = Vec::new();
        t.flush(|_| unit_mesh(Point3::ZERO), &mut runs);
        runs
    }

    #[test]
    fn record_is_128_bytes() {
        assert_eq!(size_of::<InstanceRecord>(), 128);
        assert_eq!(RECORD_BYTES, 128);
    }

    #[test]
    fn new_instances_are_dirty_once_then_clean() {
        let mut t = InstanceTable::default();
        for _ in 0..3 {
            t.insert(Instance::new(key(), Transform::IDENTITY));
        }
        assert_eq!(flush(&mut t), vec![0..3]);
        assert!(flush(&mut t).is_empty());
    }

    #[test]
    fn mutating_one_instance_dirties_only_its_slot() {
        let mut t = InstanceTable::default();
        let keys: Vec<_> = (0..5)
            .map(|_| t.insert(Instance::new(key(), Transform::IDENTITY)))
            .collect();
        flush(&mut t);
        t.get_mut(keys[3]).unwrap().transform = Transform::from_translation(Vec3::X);
        t.get_mut(keys[3]).unwrap().visible = false; // a second touch does not duplicate
        assert_eq!(flush(&mut t), vec![3..4]);
        assert!(flush(&mut t).is_empty());
    }

    #[test]
    fn adjacent_dirty_slots_coalesce_into_runs() {
        let mut t = InstanceTable::default();
        let keys: Vec<_> = (0..8)
            .map(|_| t.insert(Instance::new(key(), Transform::IDENTITY)))
            .collect();
        flush(&mut t);
        for i in [6, 1, 2, 7, 4] {
            t.get_mut(keys[i]).unwrap();
        }
        assert_eq!(flush(&mut t), vec![1..3, 4..5, 6..8]);
    }

    #[test]
    fn removal_swaps_the_last_instance_in_and_resends_it() {
        let mut t = InstanceTable::default();
        let m = key();
        let keys: Vec<_> = (0..4)
            .map(|i| {
                t.insert(Instance::new(
                    m,
                    Transform::from_translation(Vec3::new(f64::from(i), 0.0, 0.0)),
                ))
            })
            .collect();
        flush(&mut t);
        assert!(t.remove(keys[1]));
        assert!(!t.remove(keys[1]), "second removal reports false");
        assert!(t.get(keys[1]).is_none());
        // The former last instance (x = 3) moved into slot 1 and is dirty.
        assert_eq!(flush(&mut t), vec![1..2]);
        assert_eq!(t.len(), 3);
        for (k, x) in [(keys[0], 0.0), (keys[2], 2.0), (keys[3], 3.0)] {
            let slot = *t.index.get(k).unwrap() as usize;
            assert_eq!(t.keys[slot], k);
            assert_eq!(t.instances[slot].transform.translation.x, x);
            assert_eq!(
                t.records[slot].lin[0][3], x as f32,
                "record follows the key"
            );
        }
    }

    #[test]
    fn removing_the_last_instance_dirties_nothing() {
        let mut t = InstanceTable::default();
        let a = t.insert(Instance::new(key(), Transform::IDENTITY));
        let b = t.insert(Instance::new(key(), Transform::IDENTITY));
        flush(&mut t);
        t.remove(b);
        assert!(flush(&mut t).is_empty());
        t.remove(a);
        assert_eq!(t.len(), 0);
    }

    #[test]
    fn add_remove_churn_between_frames_leaves_one_clean_record_per_survivor() {
        let mut t = InstanceTable::default();
        let m = key();
        let keep = t.insert(Instance::new(m, Transform::IDENTITY));
        flush(&mut t);
        for cycle in 0..2000 {
            let ks: Vec<_> = (0..=cycle % 3)
                .map(|_| t.insert(Instance::new(m, Transform::IDENTITY)))
                .collect();
            for k in ks {
                t.remove(k);
            }
            assert!(
                t.dirty.len() <= 2 * t.len() + 64,
                "dirty list grew to {}",
                t.dirty.len()
            );
        }
        assert!(t.get(keep).is_some());
        assert_eq!(t.len(), 1);
        // Slot 0 was never touched again: nothing to send.
        assert!(flush(&mut t).is_empty());
        let k = t.insert(Instance::new(m, Transform::IDENTITY));
        t.remove(k);
        let k = t.insert(Instance::new(m, Transform::IDENTITY));
        assert_eq!(
            flush(&mut t),
            vec![1..2],
            "one slot, once, despite stale entries"
        );
        t.remove(k);
    }

    #[test]
    fn dirty_slots_that_vanish_are_dropped() {
        let mut t = InstanceTable::default();
        let a = t.insert(Instance::new(key(), Transform::IDENTITY));
        let b = t.insert(Instance::new(key(), Transform::IDENTITY));
        flush(&mut t);
        t.get_mut(b).unwrap();
        t.remove(b);
        t.remove(a);
        assert!(flush(&mut t).is_empty());
    }

    #[test]
    fn translation_is_split_into_high_and_low_parts() {
        let inst = Instance::new(
            key(),
            Transform::from_translation(Vec3::new(1.0e5 + 0.123_456_789, -2.0e5, 3.0)),
        );
        let (r, _) = build_record(&inst, unit_mesh(Point3::new(0.01, 0.0, 0.0)));
        let hi = Vec3::new(
            f64::from(r.lin[0][3]),
            f64::from(r.lin[1][3]),
            f64::from(r.lin[2][3]),
        );
        let lo = Vec3::new(
            f64::from(r.t_lo[0]),
            f64::from(r.t_lo[1]),
            f64::from(r.t_lo[2]),
        );
        let want = Vec3::new(1.0e5 + 0.123_456_789 + 0.01, -2.0e5, 3.0);
        assert!(
            (hi + lo).distance(want) < 1e-6,
            "{}",
            (hi + lo).distance(want)
        );
        assert!(
            lo.x.abs() > 0.0,
            "the low part carries the digits f32 loses"
        );
    }

    #[test]
    fn sphere_encloses_the_transformed_bounds() {
        let mut inst = Instance::new(
            key(),
            Transform::from_scale_rotation_translation(
                Vec3::new(2.0, 1.0, 1.0),
                amg_core::Quat::from_rotation_z(0.7),
                Vec3::new(10.0, 20.0, 30.0),
            ),
        );
        inst.visible = true;
        let (_, info) = build_record(&inst, unit_mesh(Point3::ZERO));
        let s = info.sphere.unwrap();
        for x in [-0.5, 0.5] {
            for y in [-0.5, 0.5] {
                for z in [-0.5, 0.5] {
                    let p = inst.transform.transform_point3(Point3::new(x, y, z));
                    assert!(p.distance(s.center) <= s.radius + 1e-9);
                }
            }
        }
        let (_, none) = build_record(&inst, None);
        assert!(none.sphere.is_none(), "no mesh, nothing to draw");
    }

    #[test]
    fn material_packs_metallic_and_roughness_like_unpack2x16unorm() {
        let m = Material {
            metallic: 1.0,
            roughness: 0.25,
            ..Material::default()
        };
        let p = m.packed_metallic_roughness();
        assert_eq!(p & 0xffff, 65535);
        assert_eq!(p >> 16, (0.25f32 * 65535.0).round() as u32);
        let clamped = Material {
            metallic: -3.0,
            roughness: 7.0,
            ..Material::default()
        };
        assert_eq!(clamped.packed_metallic_roughness(), 0xffff_0000);
        let (r, _) = build_record(
            &Instance {
                material: m,
                ..Instance::new(key(), Transform::IDENTITY)
            },
            unit_mesh(Point3::ZERO),
        );
        assert_eq!(r.material, p);
    }

    #[test]
    fn layer_masks_select_layers() {
        let m = LayerMask::ALL.without(Layer::Ghost);
        assert!(m.contains(Layer::Model) && m.contains(Layer::Preview));
        assert!(!m.contains(Layer::Ghost));
        assert!(m.with(Layer::Ghost).contains(Layer::Ghost));
        assert!(LayerMask::only(Layer::Preview).contains(Layer::Preview));
        assert!(!LayerMask::only(Layer::Preview).contains(Layer::Model));
        assert!(!LayerMask::NONE.contains(Layer::Model));
    }
}
