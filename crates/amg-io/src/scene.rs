//! The format-independent scene every reader produces and every writer consumes (S12 §2).

use std::collections::BTreeMap;
use std::sync::Arc;

use amg_core::{Aabb3, Transform, units::LengthUnit};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, Rgba8};

use crate::error::IoError;
use crate::units::convert_length;

/// "No slot": the face uses the object's default material.
pub const NO_SLOT: u16 = u16::MAX;
/// Index into [`Scene::materials`].
pub type SlotIndex = u16;

/// Deepest group nesting a [`Scene`] may have (STEP assemblies rarely exceed a dozen).
pub const MAX_GROUP_DEPTH: usize = 256;

/// 3MF base material, colour group entry, OBJ MTL entry or slicer filament slot.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MaterialSlot {
    /// "PLA Red", "Material 2", the 3MF base name or the MTL name.
    pub name: String,
    /// sRGB plus alpha.
    pub color: Rgba8,
    /// 1-based slicer filament or extruder; `None` = by slot order.
    pub filament: Option<u8>,
}

/// Role of an object.
#[derive(
    Copy, Clone, Debug, PartialEq, Eq, Hash, Default, serde::Serialize, serde::Deserialize,
)]
pub enum ObjectKind {
    /// A printable model.
    #[default]
    Model,
    /// 3MF `solidsupport`.
    SolidSupport,
    /// 3MF `support`.
    Support,
    /// 3MF `surface`.
    Surface,
    /// 3MF `other`.
    Other,
}

/// Slicer part role (Orca/Prusa).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum PartSubtype {
    /// Ordinary geometry.
    #[default]
    NormalPart,
    /// Subtracts material.
    NegativePart,
    /// Changes settings in its volume.
    Modifier,
    /// Forces supports.
    SupportEnforcer,
    /// Blocks supports.
    SupportBlocker,
}

/// One mesh with its attributes.
#[derive(Clone, Debug)]
pub struct SceneObject {
    /// Name.
    pub name: Option<String>,
    /// Geometry in object space, mm.
    pub mesh: Arc<IndexedMesh>,
    /// Role.
    pub kind: ObjectKind,
    /// Default material of the object.
    pub material: Option<SlotIndex>,
    /// 1-based slicer extruder; overrides `materials[material].filament`.
    pub filament: Option<u8>,
    /// Material per face (`len == mesh.face_count()`); [`NO_SLOT`] = default.
    pub face_materials: Option<Arc<[SlotIndex]>>,
    /// Slicer part role.
    pub part_subtype: PartSubtype,
    /// 3MF object id, `p:UUID` or OBJ object name, for diagnostics.
    pub source_id: Option<String>,
}

impl SceneObject {
    /// An unnamed model object with no materials.
    pub fn new(mesh: Arc<IndexedMesh>) -> Self {
        Self {
            name: None,
            mesh,
            kind: ObjectKind::Model,
            material: None,
            filament: None,
            face_materials: None,
            part_subtype: PartSubtype::NormalPart,
            source_id: None,
        }
    }
}

/// A placed object or a group of nodes.
#[derive(Clone, Debug)]
pub struct SceneNode {
    /// Name.
    pub name: Option<String>,
    /// Parent ← node transform, mm.
    pub transform: Transform,
    /// Content.
    pub content: NodeContent,
}

impl SceneNode {
    /// A node that places object `object` with `transform`.
    pub fn mesh(object: u32, transform: Transform) -> Self {
        Self {
            name: None,
            transform,
            content: NodeContent::Mesh(object),
        }
    }

    /// A group of `children` placed with `transform`.
    pub fn group(transform: Transform, children: Vec<SceneNode>) -> Self {
        Self {
            name: None,
            transform,
            content: NodeContent::Group(children),
        }
    }
}

/// What a node holds.
#[derive(Clone, Debug)]
pub enum NodeContent {
    /// An instance of `objects[index]`.
    Mesh(u32),
    /// Child nodes.
    Group(Vec<SceneNode>),
}

/// A build plate of a slicer project.
#[derive(Clone, Debug, Default)]
pub struct Plate {
    /// Name.
    pub name: Option<String>,
    /// Indices into [`Scene::roots`].
    pub roots: Vec<u32>,
}

/// Preview image of a file.
#[derive(Clone, Debug)]
pub enum Thumbnail {
    /// An encoded PNG.
    Png(Arc<[u8]>),
    /// Raw RGBA8 pixels.
    Rgba8 {
        /// Width in pixels.
        width: u32,
        /// Height in pixels.
        height: u32,
        /// `4 * width * height` bytes.
        pixels: Arc<[u8]>,
    },
}

/// Everything in a mesh file, in mm.
#[derive(Clone, Debug, Default)]
pub struct Scene {
    /// Distinct meshes.
    pub objects: Vec<SceneObject>,
    /// Build items / top-level occurrences.
    pub roots: Vec<SceneNode>,
    /// Material slots.
    pub materials: Vec<MaterialSlot>,
    /// Core names, or `"ns:name"` for unknown ones.
    pub metadata: BTreeMap<String, String>,
    /// Empty = everything on plate 1.
    pub plates: Vec<Plate>,
    /// Preview image.
    pub thumbnail: Option<Thumbnail>,
}

fn invalid(message: impl Into<String>) -> IoError {
    IoError::InvalidScene {
        message: message.into(),
    }
}

impl Scene {
    /// Every (object, world transform) pair, depth-first in declaration order.
    ///
    /// Indices are not checked here; call [`Scene::validate`] on untrusted scenes.
    pub fn instances(&self) -> Vec<(u32, Transform)> {
        let mut out = Vec::new();
        // Explicit stack: depth is data, not recursion.
        let mut stack: Vec<(&SceneNode, Transform)> = self
            .roots
            .iter()
            .rev()
            .map(|n| (n, Transform::IDENTITY))
            .collect();
        while let Some((node, parent)) = stack.pop() {
            let world = parent * node.transform;
            match &node.content {
                NodeContent::Mesh(i) => out.push((*i, world)),
                NodeContent::Group(children) => {
                    stack.extend(children.iter().rev().map(|c| (c, world)));
                }
            }
        }
        out
    }

    /// Bounding box of all instances in world space. Instances of missing objects are skipped.
    pub fn world_aabb(&self) -> Aabb3 {
        let mut b = Aabb3::EMPTY;
        for (i, t) in self.instances() {
            if let Some(o) = self.objects.get(i as usize) {
                for p in &o.mesh.positions {
                    b.include_point(t.transform_point3(*p));
                }
            }
        }
        b
    }

    /// Triangles over all instances (an object placed twice counts twice).
    pub fn triangle_count(&self) -> u64 {
        self.instances()
            .iter()
            .filter_map(|(i, _)| self.objects.get(*i as usize))
            .map(|o| o.mesh.face_count() as u64)
            .sum()
    }

    /// Merges all instances into one mesh in world space (STL write, `merge_objects`), plus
    /// the material slot of every face if any object has a material.
    ///
    /// The slot of a face is its `face_materials` entry, or the object's `material` if that
    /// is [`NO_SLOT`] or absent, or [`NO_SLOT`].
    ///
    /// # Errors
    /// Any [`Scene::validate`] error, [`IoError::Mesh`] if the result outgrows `u32`, or
    /// [`IoError::Cancelled`].
    pub fn flatten(&self, ctx: &JobCtx) -> Result<(IndexedMesh, Option<Vec<SlotIndex>>), IoError> {
        self.validate()?;
        let instances = self.instances();
        let want_slots = instances.iter().any(|(i, _)| {
            let o = &self.objects[*i as usize];
            o.material.is_some() || o.face_materials.is_some()
        });
        let mut out = IndexedMesh::default();
        let mut slots: Vec<SlotIndex> = Vec::new();
        let n = instances.len().max(1) as f32;
        for (k, (i, t)) in instances.iter().enumerate() {
            ctx.checkpoint()?;
            ctx.progress(k as f32 / n);
            let o = &self.objects[*i as usize];
            if *t == Transform::IDENTITY {
                out.append(&o.mesh)?;
            } else {
                let mut m = (*o.mesh).clone();
                m.transform(t);
                out.append(&m)?;
            }
            if want_slots {
                let default = o.material.unwrap_or(NO_SLOT);
                match &o.face_materials {
                    Some(fm) => {
                        slots.extend(fm.iter().map(|&s| if s == NO_SLOT { default } else { s }))
                    }
                    None => slots.extend(std::iter::repeat_n(default, o.mesh.face_count())),
                }
            }
        }
        ctx.progress(1.0);
        Ok((out, want_slots.then_some(slots)))
    }

    /// Exact uniform rescale of every mesh and translation for a unit change: each coordinate
    /// `v` becomes `convert_length(v, from, to)`.
    pub fn rescale(&mut self, from: LengthUnit, to: LengthUnit) {
        if from == to {
            return;
        }
        let f = |v: f64| convert_length(v, from, to);
        for o in &mut self.objects {
            let mesh = Arc::make_mut(&mut o.mesh);
            for p in &mut mesh.positions {
                p.x = f(p.x);
                p.y = f(p.y);
                p.z = f(p.z);
            }
        }
        fn nodes(ns: &mut [SceneNode], f: &dyn Fn(f64) -> f64) {
            // Explicit stack, like `instances`.
            let mut stack: Vec<&mut SceneNode> = ns.iter_mut().collect();
            while let Some(n) = stack.pop() {
                let t = &mut n.transform.translation;
                t.x = f(t.x);
                t.y = f(t.y);
                t.z = f(t.z);
                if let NodeContent::Group(c) = &mut n.content {
                    stack.extend(c.iter_mut());
                }
            }
        }
        nodes(&mut self.roots, &f);
    }

    /// Checks the invariants every writer relies on:
    /// - every mesh is valid and every instance refers to an existing object;
    /// - material and slot indices exist, and `face_materials` has one entry per face;
    /// - plates refer to existing roots;
    /// - transforms are finite and groups nest at most [`MAX_GROUP_DEPTH`] deep.
    ///
    /// Cycles cannot be expressed in the owned tree; the depth limit is what stops a reader
    /// that unrolls a cyclic component reference.
    ///
    /// # Errors
    /// [`IoError::InvalidScene`], [`IoError::LimitExceeded`] or [`IoError::Mesh`].
    pub fn validate(&self) -> Result<(), IoError> {
        let slots = self.materials.len();
        for (k, o) in self.objects.iter().enumerate() {
            let label = || o.name.clone().unwrap_or_else(|| format!("#{k}"));
            o.mesh.validate()?;
            if let Some(m) = o.material
                && usize::from(m) >= slots
            {
                return Err(invalid(format!(
                    "object {} uses material {m} but there are {slots} slots",
                    label()
                )));
            }
            if let Some(fm) = &o.face_materials {
                if fm.len() != o.mesh.face_count() {
                    return Err(invalid(format!(
                        "object {} has {} face materials for {} faces",
                        label(),
                        fm.len(),
                        o.mesh.face_count()
                    )));
                }
                if let Some(bad) = fm
                    .iter()
                    .find(|&&s| s != NO_SLOT && usize::from(s) >= slots)
                {
                    return Err(invalid(format!(
                        "object {} has a face with slot {bad} but there are {slots} slots",
                        label()
                    )));
                }
            }
        }
        // (node, depth); roots have depth 1.
        let mut stack: Vec<(&SceneNode, usize)> = self.roots.iter().map(|n| (n, 1)).collect();
        while let Some((n, depth)) = stack.pop() {
            if !n.transform.is_finite() {
                return Err(invalid(format!(
                    "node {} has a non-finite transform",
                    n.name.as_deref().unwrap_or("(unnamed)")
                )));
            }
            match &n.content {
                NodeContent::Mesh(i) => {
                    if *i as usize >= self.objects.len() {
                        return Err(invalid(format!(
                            "node {} refers to object {i} but there are {} objects",
                            n.name.as_deref().unwrap_or("(unnamed)"),
                            self.objects.len()
                        )));
                    }
                }
                NodeContent::Group(children) => {
                    if depth >= MAX_GROUP_DEPTH {
                        return Err(IoError::LimitExceeded {
                            what: "group nesting depth",
                            limit: MAX_GROUP_DEPTH as u64,
                            actual: depth as u64 + 1,
                        });
                    }
                    stack.extend(children.iter().map(|c| (c, depth + 1)));
                }
            }
        }
        for (p, plate) in self.plates.iter().enumerate() {
            if let Some(r) = plate
                .roots
                .iter()
                .find(|&&r| r as usize >= self.roots.len())
            {
                return Err(invalid(format!(
                    "plate {p} refers to root {r} but there are {} roots",
                    self.roots.len()
                )));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "exact unit conversion is the property under test"
)]
mod tests {
    use super::*;
    use amg_core::{Point3, Vec3};

    /// Two triangles of a unit square in the XY plane.
    fn quad() -> Arc<IndexedMesh> {
        Arc::new(
            IndexedMesh::new(
                vec![
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(1.0, 1.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                ],
                vec![[0, 1, 2], [0, 2, 3]],
            )
            .expect("quad"),
        )
    }

    fn translate(x: f64, y: f64, z: f64) -> Transform {
        Transform::from_translation(Vec3::new(x, y, z))
    }

    fn scene_of(roots: Vec<SceneNode>) -> Scene {
        Scene {
            objects: vec![SceneObject::new(quad())],
            roots,
            ..Scene::default()
        }
    }

    fn assert_invalid(s: &Scene, needle: &str) {
        match s.validate() {
            Err(IoError::InvalidScene { message }) => {
                assert!(message.contains(needle), "{message}")
            }
            other => panic!("expected InvalidScene({needle}), got {other:?}"),
        }
    }

    #[test]
    fn validate_accepts_a_good_scene() {
        let mut s = scene_of(vec![
            SceneNode::mesh(0, Transform::IDENTITY),
            SceneNode::group(
                translate(1.0, 0.0, 0.0),
                vec![SceneNode::mesh(0, translate(0.0, 2.0, 0.0))],
            ),
        ]);
        s.materials.push(MaterialSlot {
            name: "PLA".into(),
            color: Rgba8([255, 0, 0, 255]),
            filament: Some(1),
        });
        s.objects[0].material = Some(0);
        s.objects[0].face_materials = Some(Arc::from(vec![0u16, NO_SLOT]));
        s.plates.push(Plate {
            name: None,
            roots: vec![0, 1],
        });
        s.validate().expect("valid");
        assert!(Scene::default().validate().is_ok());
    }

    #[test]
    fn validate_rejects_an_out_of_range_object() {
        let s = scene_of(vec![SceneNode::group(
            Transform::IDENTITY,
            vec![SceneNode::mesh(1, Transform::IDENTITY)],
        )]);
        assert_invalid(&s, "object 1");
    }

    #[test]
    fn validate_rejects_out_of_range_slots() {
        // Object default material.
        let mut s = scene_of(vec![SceneNode::mesh(0, Transform::IDENTITY)]);
        s.objects[0].material = Some(0);
        assert_invalid(&s, "material 0");
        // Face slot.
        let mut s = scene_of(vec![SceneNode::mesh(0, Transform::IDENTITY)]);
        s.materials.push(MaterialSlot {
            name: "a".into(),
            color: Rgba8::WHITE,
            filament: None,
        });
        s.objects[0].face_materials = Some(Arc::from(vec![0u16, 1]));
        assert_invalid(&s, "slot 1");
        // Wrong length.
        s.objects[0].face_materials = Some(Arc::from(vec![0u16]));
        assert_invalid(&s, "1 face materials for 2 faces");
    }

    #[test]
    fn validate_rejects_runaway_nesting_the_stand_in_for_cycles() {
        // A reader that unrolls a cyclic component reference produces an ever deeper chain.
        let mut node = SceneNode::mesh(0, Transform::IDENTITY);
        for _ in 0..MAX_GROUP_DEPTH + 10 {
            node = SceneNode::group(Transform::IDENTITY, vec![node]);
        }
        let s = scene_of(vec![node]);
        assert!(matches!(
            s.validate(),
            Err(IoError::LimitExceeded {
                what: "group nesting depth",
                ..
            })
        ));
        // Just inside the limit is fine.
        let mut node = SceneNode::mesh(0, Transform::IDENTITY);
        for _ in 0..MAX_GROUP_DEPTH - 2 {
            node = SceneNode::group(Transform::IDENTITY, vec![node]);
        }
        scene_of(vec![node])
            .validate()
            .expect("depth within the limit");
    }

    #[test]
    fn validate_rejects_bad_plates_transforms_and_meshes() {
        let mut s = scene_of(vec![SceneNode::mesh(0, Transform::IDENTITY)]);
        s.plates.push(Plate {
            name: None,
            roots: vec![3],
        });
        assert_invalid(&s, "root 3");

        let s = scene_of(vec![SceneNode::mesh(0, translate(f64::NAN, 0.0, 0.0))]);
        assert_invalid(&s, "non-finite");

        let mut s = scene_of(vec![SceneNode::mesh(0, Transform::IDENTITY)]);
        let mut bad = (*s.objects[0].mesh).clone();
        bad.tris.push([0, 1, 99]);
        s.objects[0].mesh = Arc::new(bad);
        assert!(matches!(s.validate(), Err(IoError::Mesh(_))));
    }

    #[test]
    fn instances_are_depth_first_in_declaration_order() {
        let mut s = scene_of(vec![
            SceneNode::group(
                translate(10.0, 0.0, 0.0),
                vec![
                    SceneNode::mesh(0, translate(1.0, 0.0, 0.0)),
                    SceneNode::group(
                        translate(0.0, 5.0, 0.0),
                        vec![SceneNode::mesh(1, translate(0.0, 0.0, 2.0))],
                    ),
                    SceneNode::mesh(0, translate(3.0, 0.0, 0.0)),
                ],
            ),
            SceneNode::mesh(1, Transform::IDENTITY),
        ]);
        s.objects.push(SceneObject::new(quad()));
        let got: Vec<(u32, Vec3)> = s
            .instances()
            .into_iter()
            .map(|(i, t)| (i, t.translation))
            .collect();
        assert_eq!(
            got,
            vec![
                (0, Vec3::new(11.0, 0.0, 0.0)),
                (1, Vec3::new(10.0, 5.0, 2.0)),
                (0, Vec3::new(13.0, 0.0, 0.0)),
                (1, Vec3::ZERO),
            ]
        );
        assert_eq!(s.triangle_count(), 8);
    }

    #[test]
    fn flatten_of_three_instances_equals_the_hand_built_mesh() {
        let ts = [
            Transform::IDENTITY,
            translate(5.0, 0.0, 0.0),
            Transform::from_rotation_translation(
                amg_core::Quat::from_rotation_z(std::f64::consts::FRAC_PI_2),
                Vec3::new(0.0, 7.0, 1.0),
            ),
        ];
        let mut s = scene_of(ts.iter().map(|t| SceneNode::mesh(0, *t)).collect());
        // The object is shared: one mesh, three instances.
        assert_eq!(s.objects.len(), 1);
        let (flat, slots) = s.flatten(&JobCtx::detached()).expect("flatten");
        assert!(slots.is_none());

        let base = quad();
        let mut positions = Vec::new();
        let mut tris = Vec::new();
        for t in &ts {
            let off = positions.len() as u32;
            positions.extend(base.positions.iter().map(|p| t.transform_point3(*p)));
            tris.extend(base.tris.iter().map(|f| f.map(|i| i + off)));
        }
        let expected = IndexedMesh::new(positions, tris).expect("hand built");
        assert_eq!(flat.positions, expected.positions);
        assert_eq!(flat.tris, expected.tris);
        assert_eq!(flat.face_count(), 6);
        assert_eq!(s.triangle_count(), 6);

        // With materials: per-face slot, then the object default, then NO_SLOT.
        s.materials = vec![
            MaterialSlot {
                name: "a".into(),
                color: Rgba8::WHITE,
                filament: None,
            },
            MaterialSlot {
                name: "b".into(),
                color: Rgba8::WHITE,
                filament: None,
            },
        ];
        s.objects[0].material = Some(1);
        s.objects[0].face_materials = Some(Arc::from(vec![0u16, NO_SLOT]));
        let (_, slots) = s.flatten(&JobCtx::detached()).expect("flatten");
        assert_eq!(slots, Some(vec![0, 1, 0, 1, 0, 1]));
    }

    #[test]
    fn flatten_keeps_orientation_under_a_mirror_and_reports_cancel() {
        let mirror = Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0));
        let s = scene_of(vec![SceneNode::mesh(0, mirror)]);
        let (flat, _) = s.flatten(&JobCtx::detached()).expect("flatten");
        let n = flat.face_normal(amg_mesh::FaceId(0));
        assert!(n.z > 0.0, "mirrored face must still point up, got {n:?}");

        let token = amg_jobs::CancelToken::new();
        let ctx = JobCtx::with_token(token.clone());
        token.cancel();
        assert!(matches!(s.flatten(&ctx), Err(IoError::Cancelled)));
    }

    #[test]
    fn rescale_is_exact_and_reversible_in_the_exact_cases() {
        let mut s = scene_of(vec![SceneNode::group(
            translate(2.0, 3.0, 4.0),
            vec![SceneNode::mesh(0, translate(1.0, 0.5, 0.25))],
        )]);
        let before = s.objects[0].mesh.positions.clone();
        s.rescale(LengthUnit::Inch, LengthUnit::Millimetre);
        for (a, b) in before.iter().zip(&s.objects[0].mesh.positions) {
            assert_eq!(b.x.to_bits(), (a.x * 25.4).to_bits());
            assert_eq!(b.y.to_bits(), (a.y * 25.4).to_bits());
            assert_eq!(b.z.to_bits(), (a.z * 25.4).to_bits());
        }
        let NodeContent::Group(c) = &s.roots[0].content else {
            panic!("group expected")
        };
        assert_eq!(
            s.roots[0].transform.translation.x.to_bits(),
            (2.0f64 * 25.4).to_bits()
        );
        assert_eq!(
            c[0].transform.translation.y.to_bits(),
            (0.5f64 * 25.4).to_bits()
        );
        // Same unit: untouched. cm -> mm -> cm for exact values.
        let snapshot = s.objects[0].mesh.clone();
        s.rescale(LengthUnit::Metre, LengthUnit::Metre);
        assert!(Arc::ptr_eq(&snapshot, &s.objects[0].mesh));
        let mut t = scene_of(vec![SceneNode::mesh(0, translate(1.0, 0.0, 0.0))]);
        t.rescale(LengthUnit::Centimetre, LengthUnit::Millimetre);
        t.rescale(LengthUnit::Millimetre, LengthUnit::Centimetre);
        assert_eq!(t.objects[0].mesh.positions, quad().positions);
        assert_eq!(t.roots[0].transform.translation.x, 1.0);
    }

    #[test]
    fn rescale_does_not_touch_other_scenes_sharing_the_mesh() {
        let a = scene_of(vec![SceneNode::mesh(0, Transform::IDENTITY)]);
        let mut b = a.clone();
        b.rescale(LengthUnit::Inch, LengthUnit::Millimetre);
        assert_eq!(a.objects[0].mesh.positions, quad().positions);
        assert_ne!(b.objects[0].mesh.positions, quad().positions);
    }

    #[test]
    fn world_aabb_covers_all_instances() {
        let s = scene_of(vec![
            SceneNode::mesh(0, Transform::IDENTITY),
            SceneNode::mesh(0, translate(4.0, 5.0, 6.0)),
        ]);
        let b = s.world_aabb();
        assert_eq!((b.min, b.max), (Point3::ZERO, Point3::new(5.0, 6.0, 6.0)));
        assert!(Scene::default().world_aabb().is_empty());
    }
}
