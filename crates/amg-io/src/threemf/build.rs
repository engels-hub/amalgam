//! Parsed model parts → [`Scene`]: build items become roots, components become groups, mesh
//! objects become [`SceneObject`]s (shared between instances), property groups become
//! [`MaterialSlot`]s and slicer paint becomes per-face filament slots.

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

use amg_core::units::LengthUnit;
use amg_core::{Point2, Transform};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, MeshAttrs, Rgba8};
use rayon::prelude::*;

use super::config::{OrcaConfig, PrusaConfig, PrusaObject};
use super::model::{Content, MeshData, ModelPart, Object, PropGroup};
use super::scan::{NONE, TriProp};
use crate::error::{IoError, IoWarning};
use crate::paint;
use crate::scene::{
    MaterialSlot, NO_SLOT, NodeContent, ObjectKind, Plate, Scene, SceneNode, SceneObject, SlotIndex,
};
use crate::units::convert_length;

/// A hostile file can make a component DAG expand exponentially; this bounds the tree.
pub(crate) const MAX_NODES: u64 = 10_000_000;

/// Colours of filament slots when the project does not say (sRGB, distinct).
const PALETTE: [Rgba8; 8] = [
    Rgba8([0xE6, 0xE6, 0xE6, 255]),
    Rgba8([0xD9, 0x3B, 0x2B, 255]),
    Rgba8([0x2B, 0x6C, 0xD9, 255]),
    Rgba8([0x3B, 0xB0, 0x4A, 255]),
    Rgba8([0xF2, 0xC2, 0x1B, 255]),
    Rgba8([0x8E, 0x44, 0xAD, 255]),
    Rgba8([0xF2, 0x8C, 0x28, 255]),
    Rgba8([0x33, 0x33, 0x33, 255]),
];

/// Slicer data the builder applies.
#[derive(Default)]
pub(crate) struct Extras {
    pub orca: Option<OrcaConfig>,
    pub prusa: Option<PrusaConfig>,
    pub filament_colors: Vec<Rgba8>,
    pub read_paint: bool,
}

/// Counters that become warnings.
#[derive(Default)]
pub(crate) struct Counts {
    pub paint_approx: u64,
    pub paint_invalid: u64,
    pub paint_clamped: u64,
    pub textured: u64,
    pub composite: u64,
    pub multi: u64,
    pub gradient: u64,
    pub other_items: u64,
    pub missing_default: u64,
}

pub(crate) struct Builder<'a> {
    pub parts: &'a BTreeMap<String, ModelPart>,
    pub root: &'a str,
    pub root_unit: LengthUnit,
    pub strict: bool,
    pub keep_components: bool,
    pub max_depth: u32,
    pub extras: &'a Extras,
    pub ctx: &'a JobCtx,
    pub scene: Scene,
    pub counts: Counts,
    /// (part, object id, Orca root object) → scene object.
    objects: HashMap<(String, u32, Option<u32>), u32>,
    /// (part, group id, index) → slot.
    slots: HashMap<(String, u32, u32), SlotIndex>,
    filament_slots: HashMap<u8, SlotIndex>,
    stack: Vec<(String, u32)>,
    nodes: u64,
}

fn limit(what: &'static str, limit: u64, actual: u64) -> IoError {
    IoError::LimitExceeded {
        what,
        limit,
        actual,
    }
}

/// Resolved material of one corner/face.
#[derive(Copy, Clone, PartialEq)]
enum Res {
    None,
    Slot(SlotIndex),
    Uv(Point2),
}

impl<'a> Builder<'a> {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        parts: &'a BTreeMap<String, ModelPart>,
        root: &'a str,
        root_unit: LengthUnit,
        strict: bool,
        keep_components: bool,
        max_depth: u32,
        extras: &'a Extras,
        ctx: &'a JobCtx,
    ) -> Self {
        Builder {
            parts,
            root,
            root_unit,
            strict,
            keep_components,
            max_depth,
            extras,
            ctx,
            scene: Scene::default(),
            counts: Counts::default(),
            objects: HashMap::new(),
            slots: HashMap::new(),
            filament_slots: HashMap::new(),
            stack: Vec::new(),
            nodes: 0,
        }
    }

    fn part(&self, key: &str) -> Result<&'a ModelPart, IoError> {
        self.parts.get(key).ok_or_else(|| {
            super::xml::malformed_part(key, "the referenced model part does not exist")
        })
    }

    /// Scale from a part's unit to the root unit.
    fn scale_of(&self, p: &ModelPart) -> Option<(LengthUnit, LengthUnit)> {
        let u = p.unit.unwrap_or(LengthUnit::Millimetre);
        (u != self.root_unit).then_some((u, self.root_unit))
    }

    fn transform_in(&self, p: &ModelPart, mut t: Transform) -> Transform {
        if let Some((from, to)) = self.scale_of(p) {
            t.translation.x = convert_length(t.translation.x, from, to);
            t.translation.y = convert_length(t.translation.y, from, to);
            t.translation.z = convert_length(t.translation.z, from, to);
        }
        t
    }

    /// Builds every root from the build items of the root part.
    pub(crate) fn run(&mut self) -> Result<(), IoError> {
        let root = self.part(self.root)?;
        let mut item_of_object: HashMap<u32, u32> = HashMap::new();
        // (object id, instance index) → root index, for plates.
        let mut instance_roots: HashMap<(u32, u32), u32> = HashMap::new();
        for item in &root.build {
            self.ctx.checkpoint()?;
            let part_key = item.path.clone().unwrap_or_else(|| self.root.to_owned());
            let p = self.part(&part_key)?;
            let obj = p.objects.get(&item.objectid).ok_or_else(|| {
                root.err(
                    item.offset,
                    format!("build item refers to a missing object {}", item.objectid),
                )
            })?;
            if obj.kind_other {
                if self.strict {
                    return Err(root.err(
                        item.offset,
                        "a build item refers to an object of type other",
                    ));
                }
                self.counts.other_items += 1;
                continue;
            }
            let orca_root = (part_key == self.root).then_some(item.objectid);
            let t = self.transform_in(p, item.transform);
            let nodes = self.instance(&part_key, obj, t, 0, orca_root)?;
            let inst = item_of_object.entry(item.objectid).or_insert(0);
            for n in nodes {
                instance_roots.insert((item.objectid, *inst), self.scene.roots.len() as u32);
                self.scene.roots.push(n);
            }
            *inst += 1;
        }
        if let Some(cfg) = &self.extras.orca
            && cfg.plates.len() > 1
        {
            for p in &cfg.plates {
                let mut roots: Vec<u32> = p
                    .instances
                    .iter()
                    .filter_map(|k| instance_roots.get(k).copied())
                    .collect();
                roots.sort_unstable();
                roots.dedup();
                self.scene.plates.push(Plate {
                    name: p.name.clone(),
                    roots,
                });
            }
        }
        Ok(())
    }

    /// The node(s) for one placement of `obj`: one node, or one per mesh instance when
    /// components are flattened.
    fn instance(
        &mut self,
        part_key: &str,
        obj: &'a Object,
        t: Transform,
        depth: u32,
        orca_root: Option<u32>,
    ) -> Result<Vec<SceneNode>, IoError> {
        let node = self.node(part_key, obj, t, depth, orca_root, None)?;
        if self.keep_components {
            return Ok(vec![node]);
        }
        let mut out = Vec::new();
        flatten(&node, Transform::IDENTITY, &mut out);
        Ok(out)
    }

    fn node(
        &mut self,
        part_key: &str,
        obj: &'a Object,
        t: Transform,
        depth: u32,
        orca_root: Option<u32>,
        orca_part: Option<u32>,
    ) -> Result<SceneNode, IoError> {
        self.nodes += 1;
        if self.nodes > MAX_NODES {
            return Err(limit("scene nodes", MAX_NODES, self.nodes));
        }
        if depth > self.max_depth {
            return Err(limit(
                "component depth",
                u64::from(self.max_depth),
                u64::from(depth),
            ));
        }
        let key = (part_key.to_owned(), obj.id);
        let p = self.part(part_key)?;
        if self.stack.contains(&key) {
            return Err(p.err(
                obj.offset,
                format!("component cycle through object {}", obj.id),
            ));
        }
        match &obj.content {
            Content::Components(comps) => {
                self.stack.push(key);
                let mut children = Vec::with_capacity(comps.len());
                for c in comps {
                    self.ctx.checkpoint()?;
                    let cpart = c.path.clone().unwrap_or_else(|| part_key.to_owned());
                    let cp = self.part(&cpart)?;
                    let child = cp.objects.get(&c.objectid).ok_or_else(|| {
                        p.err(
                            c.offset,
                            format!("component refers to a missing object {}", c.objectid),
                        )
                    })?;
                    if self.strict && child.kind_other {
                        return Err(
                            p.err(c.offset, "a component refers to an object of type other")
                        );
                    }
                    let ct = self.transform_in(p, c.transform);
                    let part_id = orca_root.map(|_| c.objectid);
                    children.push(self.node(&cpart, child, ct, depth + 1, orca_root, part_id)?);
                }
                self.stack.pop();
                let mut n = SceneNode::group(t, children);
                n.name = self.group_name(obj, orca_root, depth);
                Ok(n)
            }
            Content::Mesh(_) => {
                if let Some(pr) = self.prusa_volumes(part_key, obj) {
                    return self.prusa_group(part_key, obj, t, pr);
                }
                let idx = self.mesh_object(part_key, obj, orca_root, orca_part)?;
                let mut n = SceneNode::mesh(idx, t);
                n.name = self.scene.objects[idx as usize].name.clone();
                Ok(n)
            }
            Content::Empty => {
                if self.strict {
                    return Err(p.err(obj.offset, "object without content"));
                }
                Ok(SceneNode::group(t, Vec::new()))
            }
        }
    }

    fn group_name(&self, obj: &Object, orca_root: Option<u32>, depth: u32) -> Option<String> {
        if depth == 0
            && let (Some(cfg), Some(r)) = (&self.extras.orca, orca_root)
            && let Some(o) = cfg.objects.get(&r)
            && o.name.is_some()
        {
            return o.name.clone();
        }
        obj.name.clone()
    }

    fn prusa_volumes(&self, part_key: &str, obj: &Object) -> Option<&'a PrusaObject> {
        if part_key != self.root {
            return None;
        }
        let o = self.extras.prusa.as_ref()?.objects.get(&obj.id)?;
        (o.volumes.len() > 1).then_some(o)
    }

    /// The scene object for a mesh object, created on first use.
    fn mesh_object(
        &mut self,
        part_key: &str,
        obj: &'a Object,
        orca_root: Option<u32>,
        orca_part: Option<u32>,
    ) -> Result<u32, IoError> {
        let ckey = orca_root.filter(|_| self.extras.orca.is_some());
        let key = (part_key.to_owned(), obj.id, ckey);
        if let Some(&i) = self.objects.get(&key) {
            return Ok(i);
        }
        let p = self.part(part_key)?;
        let Content::Mesh(m) = &obj.content else {
            return Err(p.err(obj.offset, "not a mesh object"));
        };
        let mut so = self.convert(p, obj, m, None)?;
        // Slicer names and extruders.
        if let (Some(cfg), Some(r)) = (&self.extras.orca, orca_root)
            && let Some(o) = cfg.objects.get(&r)
        {
            let part = orca_part.and_then(|id| o.parts.get(&id)).or_else(|| {
                // A root object with a local mesh is its own single part.
                (orca_part.is_none())
                    .then(|| o.parts.values().next())
                    .flatten()
            });
            if let Some(ps) = part {
                if ps.name.is_some() {
                    so.name = ps.name.clone();
                }
                so.part_subtype = ps.subtype;
                so.filament = ps.extruder.or(o.extruder);
            } else {
                so.filament = o.extruder;
                if so.name.is_none() {
                    so.name = o.name.clone();
                }
            }
        }
        if part_key == self.root
            && let Some(cfg) = &self.extras.prusa
            && let Some(o) = cfg.objects.get(&obj.id)
        {
            if o.name.is_some() {
                so.name = o.name.clone();
            }
            let v = o.volumes.first();
            so.filament = v.and_then(|v| v.extruder).or(o.extruder);
            if let Some(v) = v {
                so.part_subtype = v.subtype;
            }
        }
        let i = self.scene.objects.len() as u32;
        self.scene.objects.push(so);
        self.objects.insert(key, i);
        Ok(i)
    }

    /// A PrusaSlicer object with several volumes: one scene object per volume.
    fn prusa_group(
        &mut self,
        part_key: &str,
        obj: &'a Object,
        t: Transform,
        pr: &'a PrusaObject,
    ) -> Result<SceneNode, IoError> {
        let p = self.part(part_key)?;
        let Content::Mesh(m) = &obj.content else {
            return Err(p.err(obj.offset, "not a mesh object"));
        };
        let mut children = Vec::new();
        for (k, v) in pr.volumes.iter().enumerate() {
            let key = (part_key.to_owned(), obj.id, Some(u32::MAX - k as u32));
            let idx = match self.objects.get(&key) {
                Some(&i) => i,
                None => {
                    let mut so = self.convert(p, obj, m, Some((v.first, v.last)))?;
                    so.name = v.name.clone().or_else(|| pr.name.clone());
                    so.filament = v.extruder.or(pr.extruder);
                    so.part_subtype = v.subtype;
                    let i = self.scene.objects.len() as u32;
                    self.scene.objects.push(so);
                    self.objects.insert(key, i);
                    i
                }
            };
            let mut n = SceneNode::mesh(idx, Transform::IDENTITY);
            n.name = self.scene.objects[idx as usize].name.clone();
            children.push(n);
        }
        let mut n = SceneNode::group(t, children);
        n.name = pr.name.clone().or_else(|| obj.name.clone());
        Ok(n)
    }

    fn slot_for(&mut self, part: &ModelPart, gid: u32, idx: u32) -> Option<SlotIndex> {
        let key = (part.key.clone(), gid, idx);
        if let Some(&s) = self.slots.get(&key) {
            return Some(s);
        }
        let (name, color) = match part.groups.get(&gid)? {
            PropGroup::Base(b) => {
                let (n, c) = b.get(idx as usize)?;
                (n.clone(), *c)
            }
            PropGroup::Colors(c) => {
                let c = *c.get(idx as usize)?;
                (
                    format!(
                        "Color #{:02X}{:02X}{:02X}{:02X}",
                        c.0[0], c.0[1], c.0[2], c.0[3]
                    ),
                    c,
                )
            }
            _ => return None,
        };
        if self.scene.materials.len() >= usize::from(NO_SLOT) {
            return None;
        }
        let s = self.scene.materials.len() as SlotIndex;
        self.scene.materials.push(MaterialSlot {
            name,
            color,
            filament: None,
        });
        self.slots.insert(key, s);
        Some(s)
    }

    fn filament_slot(&mut self, f: u8) -> SlotIndex {
        if let Some(&s) = self.filament_slots.get(&f) {
            return s;
        }
        let color = self
            .extras
            .filament_colors
            .get(usize::from(f) - 1)
            .copied()
            .unwrap_or(PALETTE[(usize::from(f) - 1) % PALETTE.len()]);
        let s = self.scene.materials.len().min(usize::from(NO_SLOT) - 1) as SlotIndex;
        self.scene.materials.push(MaterialSlot {
            name: format!("Filament {f}"),
            color,
            filament: Some(f),
        });
        self.filament_slots.insert(f, s);
        s
    }

    /// What property `(gid, idx)` resolves to (Phase 1 flattening, note §2.4).
    fn resolve(&mut self, part: &ModelPart, gid: u32, idx: u32) -> Res {
        match part.groups.get(&gid) {
            Some(PropGroup::Base(_) | PropGroup::Colors(_)) => {
                self.slot_for(part, gid, idx).map_or(Res::None, Res::Slot)
            }
            Some(PropGroup::TexCoords { uvs, .. }) => uvs
                .get(idx as usize)
                .map_or(Res::None, |uv| Res::Uv(Point2::new(uv[0], uv[1]))),
            Some(PropGroup::Composite {
                matid, matindices, ..
            }) => {
                self.counts.composite += 1;
                match matindices.first() {
                    Some(&m) => self.slot_for(part, *matid, m).map_or(Res::None, Res::Slot),
                    None => Res::None,
                }
            }
            Some(PropGroup::Multi { pids, multis }) => {
                self.counts.multi += 1;
                let Some(row) = multis.get(idx as usize) else {
                    return Res::None;
                };
                let mut uv = Res::None;
                for (k, &layer) in pids.iter().enumerate() {
                    let Some(&li) = row.get(k) else { break };
                    match part.groups.get(&layer) {
                        Some(PropGroup::Base(_) | PropGroup::Colors(_)) => {
                            if let Some(s) = self.slot_for(part, layer, li) {
                                return Res::Slot(s);
                            }
                        }
                        Some(PropGroup::Composite {
                            matid, matindices, ..
                        }) => {
                            if let Some(&m) = matindices.first()
                                && let Some(s) = self.slot_for(part, *matid, m)
                            {
                                return Res::Slot(s);
                            }
                        }
                        Some(PropGroup::TexCoords { uvs, .. }) if uv == Res::None => {
                            if let Some(u) = uvs.get(li as usize) {
                                uv = Res::Uv(Point2::new(u[0], u[1]));
                            }
                        }
                        _ => {}
                    }
                }
                uv
            }
            _ => Res::None,
        }
    }

    /// Converts a mesh object (optionally only the triangle range `tri_range`, inclusive).
    fn convert(
        &mut self,
        p: &ModelPart,
        obj: &Object,
        m: &MeshData,
        tri_range: Option<(u32, u32)>,
    ) -> Result<SceneObject, IoError> {
        self.ctx.checkpoint()?;
        let nv = m.positions.len();
        let all = &m.tris.tris;
        let (lo, hi) = match tri_range {
            Some((a, b)) => ((a as usize).min(all.len()), (b as usize + 1).min(all.len())),
            None => (0, all.len()),
        };
        // Index range (always) and distinct corners (strict).
        let bad = all[lo..hi]
            .par_iter()
            .position_first(|t| t.iter().any(|&i| i as usize >= nv));
        if let Some(k) = bad {
            return Err(p.err(
                obj.offset,
                format!(
                    "triangle {} of object {} refers to a vertex beyond {}",
                    lo + k,
                    obj.id,
                    nv
                ),
            ));
        }
        if self.strict {
            let dup = all[lo..hi]
                .par_iter()
                .position_first(|t| t[0] == t[1] || t[1] == t[2] || t[0] == t[2]);
            if let Some(k) = dup {
                return Err(p.err(
                    obj.offset,
                    format!("triangle {} of object {} repeats a vertex", lo + k, obj.id),
                ));
            }
        }
        // Vertices: the whole array, or the ones a volume uses.
        let (positions, tris) = if tri_range.is_some() {
            let mut remap = vec![u32::MAX; nv];
            let mut pos = Vec::new();
            let mut tris = Vec::with_capacity(hi - lo);
            for t in &all[lo..hi] {
                let mut o = [0u32; 3];
                for (k, &i) in t.iter().enumerate() {
                    if remap[i as usize] == u32::MAX {
                        remap[i as usize] = pos.len() as u32;
                        pos.push(m.positions[i as usize]);
                    }
                    o[k] = remap[i as usize];
                }
                tris.push(o);
            }
            (pos, tris)
        } else {
            (m.positions.clone(), all.clone())
        };
        let mut positions = positions;
        if let Some((from, to)) = self.scale_of(p) {
            positions.par_iter_mut().for_each(|q| {
                q.x = convert_length(q.x, from, to);
                q.y = convert_length(q.y, from, to);
                q.z = convert_length(q.z, from, to);
            });
        }

        // Materials.
        let default = match obj.pid {
            Some(pid) => self.resolve(p, pid, obj.pindex.unwrap_or(0)),
            None => Res::None,
        };
        let default_slot = match default {
            Res::Slot(s) => Some(s),
            _ => None,
        };
        let n = hi - lo;
        let mut face: Option<Vec<SlotIndex>> = None;
        let mut uvs: Option<Vec<[Point2; 3]>> = None;
        if let Res::Uv(_) = default {
            self.counts.textured += 1;
        }
        let props: Option<&[TriProp]> = m.tris.props.as_deref().map(|v| &v[lo..hi]);
        if self.strict
            && obj.pid.is_none()
            && let Some(f) = props.and_then(|v| v.iter().position(|t| *t != TriProp::EMPTY))
        {
            return Err(p.err(
                obj.offset,
                format!(
                    "triangle {} has properties but object {} has no default pid",
                    lo + f,
                    obj.id
                ),
            ));
        }
        if props.is_some() || matches!(default, Res::Uv(_)) {
            for f in 0..n {
                let tp = props.map_or(TriProp::EMPTY, |v| v[f]);
                let (r, corners) = if tp.p[0] == NONE {
                    if tp.pid != 0 && obj.pid.is_none() && self.strict {
                        return Err(p.err(
                            obj.offset,
                            format!(
                                "triangle {} has a pid but object {} has no default property",
                                lo + f,
                                obj.id
                            ),
                        ));
                    }
                    (default, [default; 3])
                } else {
                    let pid = if tp.pid != 0 { Some(tp.pid) } else { obj.pid };
                    let Some(pid) = pid else {
                        if self.strict {
                            return Err(p.err(
                                obj.offset,
                                format!(
                                    "triangle {} has properties but object {} has no pid",
                                    lo + f,
                                    obj.id
                                ),
                            ));
                        }
                        self.counts.missing_default += 1;
                        continue;
                    };
                    if self.strict {
                        let len = p.groups.get(&pid).map(PropGroup::len);
                        let Some(len) = len.filter(|_| {
                            !matches!(p.groups.get(&pid), Some(PropGroup::Texture2d { .. }))
                        }) else {
                            return Err(p.err(
                                obj.offset,
                                format!(
                                    "triangle {} refers to a missing property group {pid}",
                                    lo + f
                                ),
                            ));
                        };
                        if tp.p.iter().any(|&i| i != NONE && i as usize >= len) {
                            return Err(p.err(
                                obj.offset,
                                format!("triangle {} has a property index out of range", lo + f),
                            ));
                        }
                    }
                    let p1 = tp.p[0];
                    let p2 = if tp.p[1] == NONE { p1 } else { tp.p[1] };
                    let p3 = if tp.p[2] == NONE { p1 } else { tp.p[2] };
                    let r1 = self.resolve(p, pid, p1);
                    let c = if p2 == p1 && p3 == p1 {
                        [r1; 3]
                    } else {
                        let c = [r1, self.resolve(p, pid, p2), self.resolve(p, pid, p3)];
                        if matches!(r1, Res::Slot(_)) && (c[1] != r1 || c[2] != r1) {
                            self.counts.gradient += 1;
                        }
                        c
                    };
                    (r1, c)
                };
                if let Res::Slot(s) = r
                    && Some(s) != default_slot
                {
                    face.get_or_insert_with(|| vec![NO_SLOT; n])[f] = s;
                }
                if corners.iter().any(|c| matches!(c, Res::Uv(_))) {
                    let u = |c: Res| match c {
                        Res::Uv(u) => u,
                        _ => Point2::ZERO,
                    };
                    uvs.get_or_insert_with(|| vec![[Point2::ZERO; 3]; n])[f] =
                        [u(corners[0]), u(corners[1]), u(corners[2])];
                }
            }
            if uvs.is_some() && !matches!(default, Res::Uv(_)) {
                self.counts.textured += 1;
            }
        }

        // Slicer paint wins over XML properties for the faces it paints.
        if self.extras.read_paint {
            let painted = if m.tris.paint.is_empty() {
                &m.tris.mmu
            } else {
                &m.tris.paint
            };
            for (i, s) in painted {
                let i = *i as usize;
                if i < lo || i >= hi {
                    continue;
                }
                match paint::decode_dominant(s) {
                    Ok((state, split)) => {
                        if split {
                            self.counts.paint_approx += 1;
                        }
                        let state = if state > paint::MAX_STATE {
                            self.counts.paint_clamped += 1;
                            0
                        } else {
                            state
                        };
                        if state > 0 {
                            let slot = self.filament_slot(state);
                            face.get_or_insert_with(|| vec![NO_SLOT; n])[i - lo] = slot;
                        }
                    }
                    Err(_) => self.counts.paint_invalid += 1,
                }
            }
        }

        let mesh = IndexedMesh {
            positions,
            groups: amg_mesh::FaceGroups::new(tris.len()),
            tris,
            attrs: MeshAttrs {
                corner_uvs: uvs,
                ..MeshAttrs::default()
            },
        };
        mesh.validate()?;
        let mut so = SceneObject::new(Arc::new(mesh));
        so.name = obj.name.clone();
        so.kind = obj.kind;
        so.material = default_slot;
        so.face_materials = face.map(Arc::from);
        so.source_id = Some(match (&obj.uuid, p.key == self.root) {
            (Some(u), _) => u.clone(),
            (None, true) => obj.id.to_string(),
            (None, false) => format!("{}#{}", p.key, obj.id),
        });
        Ok(so)
    }

    /// Warnings for everything that was approximated or dropped.
    pub(crate) fn warnings(&self) -> Vec<IoWarning> {
        let c = &self.counts;
        let mut w = Vec::new();
        if c.paint_approx > 0 {
            w.push(IoWarning::PaintApproximated {
                triangles: c.paint_approx,
            });
        }
        let mut ignored = |what: &str, n: u64| {
            if n > 0 {
                w.push(IoWarning::UnsupportedContentIgnored {
                    what: what.into(),
                    count: n,
                });
            }
        };
        ignored(
            "texture images (UVs kept, image dropped until IO-19)",
            c.textured,
        );
        ignored(
            "composite materials (flattened to their first base material)",
            c.composite,
        );
        ignored(
            "multiproperties (flattened to their first colour layer)",
            c.multi,
        );
        ignored(
            "per-vertex colour gradients (the first corner's colour is used)",
            c.gradient,
        );
        ignored("build items of type other", c.other_items);
        ignored(
            "triangle properties without an object default",
            c.missing_default,
        );
        ignored("invalid paint strings (left unpainted)", c.paint_invalid);
        ignored(
            "paint states above the slicer range (left unpainted)",
            c.paint_clamped,
        );
        w
    }
}

/// Flattens a node tree into one mesh node per instance with world transforms.
fn flatten(n: &SceneNode, parent: Transform, out: &mut Vec<SceneNode>) {
    let t = parent * n.transform;
    match &n.content {
        NodeContent::Mesh(i) => {
            let mut m = SceneNode::mesh(*i, t);
            m.name = n.name.clone();
            out.push(m);
        }
        NodeContent::Group(c) => {
            for k in c {
                flatten(k, t, out);
            }
        }
    }
}

/// Strict-mode checks on the finished scene: model meshes are closed, oriented and outward;
/// no placement mirrors (negative determinant).
pub(crate) fn strict_scene_checks(scene: &Scene, ctx: &JobCtx) -> Result<(), IoError> {
    for (i, t) in scene.instances() {
        if t.matrix3.determinant() < 0.0 {
            let o = &scene.objects[i as usize];
            return Err(super::xml::malformed_part(
                "",
                format!(
                    "object {} is placed with a mirroring transform (negative determinant)",
                    o.source_id.as_deref().unwrap_or("?")
                ),
            ));
        }
    }
    let used: std::collections::BTreeSet<u32> =
        scene.instances().into_iter().map(|(i, _)| i).collect();
    for i in used {
        ctx.checkpoint()?;
        let o = &scene.objects[i as usize];
        if !matches!(o.kind, ObjectKind::Model | ObjectKind::SolidSupport) {
            continue;
        }
        let id = o.source_id.as_deref().unwrap_or("?");
        if o.mesh.face_count() < 4 {
            return Err(super::xml::malformed_part(
                "",
                format!("model object {id} has fewer than 4 triangles"),
            ));
        }
        let a = amg_mesh::analyze(&o.mesh, &amg_mesh::AnalyzeOpts::default(), ctx).map_err(
            |e| match e {
                amg_mesh::MeshError::Cancelled => IoError::Cancelled,
                e => IoError::Mesh(e),
            },
        )?;
        if a.boundary_edges > 0 || !a.non_manifold_edges.is_empty() {
            return Err(super::xml::malformed_part(
                "",
                format!(
                    "model object {id} is not manifold ({} boundary, {} non-manifold edges)",
                    a.boundary_edges,
                    a.non_manifold_edges.len()
                ),
            ));
        }
        if !a.is_oriented {
            return Err(super::xml::malformed_part(
                "",
                format!("model object {id} is not consistently oriented"),
            ));
        }
        if a.is_inverted {
            return Err(super::xml::malformed_part(
                "",
                format!("model object {id} faces inward (negative volume)"),
            ));
        }
    }
    Ok(())
}

/// Boundary and non-manifold edge counts (normal mode). Undirected edges are hashed into
/// buckets that are sorted independently, so the cancel check runs every bucket.
pub(crate) fn edge_counts(m: &IndexedMesh, ctx: &JobCtx) -> Result<(u64, u64), IoError> {
    const BUCKETS: usize = 256;
    let bucket = |k: u64| (k.wrapping_mul(0x9E37_79B9_7F4A_7C15) >> 56) as usize;
    let parts: Vec<Vec<Vec<u64>>> = m
        .tris
        .par_chunks(1 << 16)
        .map(|c| {
            if ctx.is_cancelled() {
                return Vec::new();
            }
            let mut b: Vec<Vec<u64>> = (0..BUCKETS)
                .map(|_| Vec::with_capacity(c.len() * 3 / BUCKETS + 8))
                .collect();
            for t in c {
                for (a, z) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                    let k = (u64::from(a.min(z)) << 32) | u64::from(a.max(z));
                    b[bucket(k)].push(k);
                }
            }
            b
        })
        .collect();
    ctx.checkpoint()?;
    let (boundary, nonman) = (0..BUCKETS)
        .into_par_iter()
        .map(|k| {
            if ctx.is_cancelled() {
                return (0, 0);
            }
            let mut e: Vec<u64> = parts
                .iter()
                .filter_map(|p| p.get(k))
                .flatten()
                .copied()
                .collect();
            e.sort_unstable();
            let (mut boundary, mut nonman) = (0u64, 0u64);
            let mut i = 0;
            while i < e.len() {
                let mut j = i + 1;
                while j < e.len() && e[j] == e[i] {
                    j += 1;
                }
                match j - i {
                    1 => boundary += 1,
                    2 => {}
                    _ => nonman += 1,
                }
                i = j;
            }
            (boundary, nonman)
        })
        .reduce(|| (0, 0), |a, b| (a.0 + b.0, a.1 + b.1));
    ctx.checkpoint()?;
    Ok((boundary, nonman))
}
