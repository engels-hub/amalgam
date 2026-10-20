//! Face groups: a per-face [`FaceGroupId`] plus a group table. Group ids are the persistent
//! names of mesh sub-entities (S01) and are never reused within one body lineage.

use std::collections::BTreeMap;

use amg_core::{Aabb3, Point3, Vec3};

use crate::error::{MeshError, MeshResult};
use crate::{FaceId, IndexedMesh};

/// Face-group id, allocated from a monotonic per-mesh counter.
#[derive(
    Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
#[repr(transparent)]
pub struct FaceGroupId(pub u32);

/// Which side of the surface a shell / offset goes to.
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ShellSide {
    /// Towards the inside (against the normals).
    Inside,
    /// Towards the outside (along the normals).
    Outside,
    /// Both sides, half the thickness each.
    Both,
}

/// Where a face group came from.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum GroupOrigin {
    /// An object of an imported file.
    Import {
        /// Object index in the file.
        object: u32,
    },
    /// Face-group segmentation.
    Segmentation,
    /// Re-allocated from a boolean operand.
    Boolean {
        /// Operand index (0 = A).
        operand: u8,
        /// The group id in the operand.
        source: FaceGroupId,
    },
    /// Plane-cut cap.
    Cap,
    /// Hole fill.
    Fill,
    /// Shell side wall or offset surface.
    Shell {
        /// Which offset surface.
        side: ShellSide,
    },
    /// Created by the user or by code without a more specific origin (the default group).
    User,
}

/// Kind of a fitted analytic primitive.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum PrimitiveKind {
    /// Plane.
    Plane,
    /// Sphere.
    Sphere,
    /// Cylinder.
    Cylinder,
    /// Cone.
    Cone,
    /// Torus.
    Torus,
}

/// A fitted analytic primitive (mm, radians).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[allow(missing_docs)]
pub enum Primitive {
    Plane {
        origin: Point3,
        normal: Vec3,
    },
    Sphere {
        center: Point3,
        radius: f64,
    },
    Cylinder {
        origin: Point3,
        axis: Vec3,
        radius: f64,
    },
    Cone {
        apex: Point3,
        axis: Vec3,
        half_angle: f64,
    },
    Torus {
        center: Point3,
        axis: Vec3,
        major: f64,
        minor: f64,
    },
}

impl Primitive {
    /// The kind of this primitive.
    pub fn kind(&self) -> PrimitiveKind {
        match self {
            Primitive::Plane { .. } => PrimitiveKind::Plane,
            Primitive::Sphere { .. } => PrimitiveKind::Sphere,
            Primitive::Cylinder { .. } => PrimitiveKind::Cylinder,
            Primitive::Cone { .. } => PrimitiveKind::Cone,
            Primitive::Torus { .. } => PrimitiveKind::Torus,
        }
    }
}

/// Group table entry.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FaceGroupInfo {
    /// Optional user-visible name.
    pub name: Option<String>,
    /// Provenance.
    pub origin: GroupOrigin,
    /// Fitted primitive, if any.
    pub primitive: Option<Primitive>,
}

impl FaceGroupInfo {
    /// Unnamed group without primitive.
    pub fn new(origin: GroupOrigin) -> Self {
        Self {
            name: None,
            origin,
            primitive: None,
        }
    }
}

/// Geometric fingerprint of a group: the fallback match when a group id no longer resolves.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GroupFingerprint {
    /// Total area, mm².
    pub area: f64,
    /// Area-weighted centroid.
    pub centroid: Point3,
    /// Area-weighted mean normal (unit, or zero if it cancels out).
    pub mean_normal: Vec3,
    /// Bounds of the group's faces.
    pub aabb: Aabb3,
    /// Kind of the fitted primitive, if any.
    pub primitive: Option<PrimitiveKind>,
}

/// Per-face group ids plus the group table.
///
/// Deserialisation validates the table against `per_face.len()` ([`FaceGroups::validate`]).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(try_from = "FaceGroupsRaw")]
pub struct FaceGroups {
    per_face: Vec<FaceGroupId>,
    info: BTreeMap<FaceGroupId, FaceGroupInfo>,
    next_id: u32,
}

/// Unvalidated wire form of [`FaceGroups`].
#[derive(serde::Deserialize)]
struct FaceGroupsRaw {
    per_face: Vec<FaceGroupId>,
    info: BTreeMap<FaceGroupId, FaceGroupInfo>,
    next_id: u32,
}

impl TryFrom<FaceGroupsRaw> for FaceGroups {
    type Error = MeshError;

    fn try_from(r: FaceGroupsRaw) -> Result<Self, MeshError> {
        let g = FaceGroups {
            per_face: r.per_face,
            info: r.info,
            next_id: r.next_id,
        };
        g.validate(g.per_face.len())?;
        Ok(g)
    }
}

/// The group every face of a freshly constructed mesh belongs to.
pub const DEFAULT_GROUP: FaceGroupId = FaceGroupId(0);

impl FaceGroups {
    /// `face_count` faces, all in [`DEFAULT_GROUP`] (origin [`GroupOrigin::User`]).
    pub fn new(face_count: usize) -> Self {
        let mut g = FaceGroups::default();
        let id = g.alloc(FaceGroupInfo::new(GroupOrigin::User));
        g.per_face = vec![id; face_count];
        g
    }

    /// Group of face `f`. Panics if `f` is out of range.
    pub fn group_of(&self, f: FaceId) -> FaceGroupId {
        self.per_face[f.0 as usize]
    }

    /// Moves face `f` into group `g`. Panics if `f` is out of range.
    pub fn set(&mut self, f: FaceId, g: FaceGroupId) {
        debug_assert!(self.info.contains_key(&g), "unknown face group {g:?}");
        self.per_face[f.0 as usize] = g;
    }

    /// Allocates a fresh, never-used group id.
    ///
    /// # Panics
    /// When all `u32::MAX - 1` ids are used; see [`FaceGroups::try_alloc`].
    pub fn alloc(&mut self, info: FaceGroupInfo) -> FaceGroupId {
        match self.try_alloc(info) {
            Ok(id) => id,
            Err(e) => panic!("{e}"),
        }
    }

    /// Allocates a fresh group id; `TooLarge` when the id counter is exhausted (`u32::MAX`
    /// is never handed out).
    pub fn try_alloc(&mut self, info: FaceGroupInfo) -> MeshResult<FaceGroupId> {
        // Invariant (checked by `validate`): `next_id < u32::MAX`, so the counter never wraps
        // and `u32::MAX` (`INVALID`) is never an id.
        if self.next_id >= u32::MAX - 1 {
            return Err(MeshError::TooLarge {
                count: self.next_id as usize,
            });
        }
        let id = FaceGroupId(self.next_id);
        self.next_id += 1;
        self.info.insert(id, info);
        Ok(id)
    }

    /// Ids still available to [`FaceGroups::try_alloc`].
    pub fn remaining_ids(&self) -> u32 {
        (u32::MAX - 1).saturating_sub(self.next_id)
    }

    /// Table entry of `g`.
    pub fn info(&self, g: FaceGroupId) -> Option<&FaceGroupInfo> {
        self.info.get(&g)
    }

    /// Mutable table entry of `g`.
    pub fn info_mut(&mut self, g: FaceGroupId) -> Option<&mut FaceGroupInfo> {
        self.info.get_mut(&g)
    }

    /// All group ids in the table, ascending.
    pub fn ids(&self) -> impl Iterator<Item = FaceGroupId> + '_ {
        self.info.keys().copied()
    }

    /// Faces of group `g`, ascending.
    pub fn faces_of(&self, g: FaceGroupId) -> impl Iterator<Item = FaceId> + '_ {
        self.per_face
            .iter()
            .enumerate()
            .filter(move |(_, x)| **x == g)
            .map(|(i, _)| FaceId(i as u32))
    }

    /// The per-face group ids.
    pub fn per_face(&self) -> &[FaceGroupId] {
        &self.per_face
    }

    /// The next id [`FaceGroups::alloc`] will hand out.
    pub fn next_id(&self) -> u32 {
        self.next_id
    }

    /// Fingerprint of group `g` on `mesh`; `None` if `g` is unknown or has no faces.
    pub fn fingerprint(&self, mesh: &IndexedMesh, g: FaceGroupId) -> Option<GroupFingerprint> {
        let info = self.info.get(&g)?;
        let mut area = 0.0;
        let mut centroid = Vec3::ZERO;
        let mut normal = Vec3::ZERO;
        let mut aabb = Aabb3::EMPTY;
        let mut any = false;
        for f in self.faces_of(g) {
            let [a, b, c] = mesh.triangle(f);
            let n = (b - a).cross(c - a);
            let fa = 0.5 * n.length();
            area += fa;
            centroid += (a + b + c) * (fa / 3.0);
            normal += n;
            for p in [a, b, c] {
                aabb.include_point(p);
            }
            any = true;
        }
        if !any {
            return None;
        }
        let centroid = if area > 0.0 {
            centroid / area
        } else {
            aabb.center()
        };
        Some(GroupFingerprint {
            area,
            centroid,
            mean_normal: normal.normalize_or_zero(),
            aabb,
            primitive: info.primitive.as_ref().map(Primitive::kind),
        })
    }

    /// Checks the per-face length and that every used id is in the table.
    pub fn validate(&self, faces: usize) -> MeshResult<()> {
        if self.per_face.len() != faces {
            return Err(MeshError::AttributeLength {
                name: "groups",
                got: self.per_face.len(),
                expected: faces,
            });
        }
        if self.next_id == u32::MAX {
            return Err(MeshError::Decode("face group id counter overflowed".into()));
        }
        if let Some((&last, _)) = self.info.last_key_value()
            && last.0 >= self.next_id
        {
            return Err(MeshError::Decode(format!(
                "face group {} is not below the id counter {}",
                last.0, self.next_id
            )));
        }
        // Runs of equal ids are common; check each run once.
        let mut prev = None;
        for &g in &self.per_face {
            if prev != Some(g) {
                if !self.info.contains_key(&g) {
                    return Err(MeshError::Decode(format!(
                        "face references unknown face group {}",
                        g.0
                    )));
                }
                prev = Some(g);
            }
        }
        Ok(())
    }

    pub(crate) fn push(&mut self, g: FaceGroupId) {
        self.per_face.push(g);
    }

    pub(crate) fn retain(&mut self, keep_f: &[bool]) {
        let mut i = 0;
        self.per_face.retain(|_| {
            i += 1;
            keep_f[i - 1]
        });
    }

    /// Ensures a group exists for `face_count` faces created without groups.
    pub(crate) fn ensure_len(&mut self, face_count: usize) {
        if self.per_face.len() != face_count {
            if self.info.is_empty() {
                *self = FaceGroups::new(face_count);
            } else {
                let g = self.ids().next().unwrap_or(DEFAULT_GROUP);
                self.per_face.resize(face_count, g);
            }
        }
    }

    /// Appends `other`'s faces, re-allocating each of its groups to a fresh id here (in id
    /// order). Returns the old → new id pairs.
    /// Fails with `TooLarge` (leaving `self` unchanged) if the ids would run out.
    pub(crate) fn append(
        &mut self,
        other: &FaceGroups,
    ) -> MeshResult<Vec<(FaceGroupId, FaceGroupId)>> {
        if (self.remaining_ids() as usize) < other.info.len() {
            return Err(MeshError::TooLarge {
                count: self.next_id as usize + other.info.len(),
            });
        }
        let map: Vec<(FaceGroupId, FaceGroupId)> = other
            .info
            .iter()
            .map(|(&old, info)| (old, self.alloc(info.clone())))
            .collect();
        self.per_face.extend(other.per_face.iter().map(|g| {
            // Every id in `per_face` is in the table for a validated mesh; unknown ids are
            // passed through unchanged.
            map.binary_search_by_key(g, |(o, _)| *o)
                .map_or(*g, |i| map[i].1)
        }));
        Ok(map)
    }

    /// Raw parts, for the binary codec.
    pub(crate) fn raw(&self) -> (&[FaceGroupId], &BTreeMap<FaceGroupId, FaceGroupInfo>, u32) {
        (&self.per_face, &self.info, self.next_id)
    }

    pub(crate) fn from_raw(
        per_face: Vec<FaceGroupId>,
        info: BTreeMap<FaceGroupId, FaceGroupInfo>,
        next_id: u32,
    ) -> Self {
        Self {
            per_face,
            info,
            next_id,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alloc_is_monotonic_and_append_reallocates() {
        let mut a = FaceGroups::new(2);
        assert_eq!(a.ids().collect::<Vec<_>>(), vec![DEFAULT_GROUP]);
        let g1 = a.alloc(FaceGroupInfo::new(GroupOrigin::Cap));
        assert_eq!(g1, FaceGroupId(1));
        a.set(FaceId(1), g1);
        assert_eq!(a.faces_of(g1).collect::<Vec<_>>(), vec![FaceId(1)]);
        let b = FaceGroups::new(3);
        let map = a.append(&b).unwrap();
        assert_eq!(map, vec![(FaceGroupId(0), FaceGroupId(2))]);
        assert_eq!(a.per_face().len(), 5);
        assert_eq!(a.group_of(FaceId(4)), FaceGroupId(2));
        assert_eq!(a.next_id(), 3);
        assert!(a.validate(5).is_ok());
        assert!(a.validate(4).is_err());
    }

    #[test]
    fn id_counter_exhaustion_is_an_error() {
        let mut a = FaceGroups::new(1);
        a.next_id = u32::MAX - 2;
        assert_eq!(a.remaining_ids(), 1);
        let id = a.try_alloc(FaceGroupInfo::new(GroupOrigin::Cap)).unwrap();
        assert_eq!(id, FaceGroupId(u32::MAX - 2));
        assert_eq!(a.remaining_ids(), 0);
        let before = a.clone();
        assert!(matches!(
            a.try_alloc(FaceGroupInfo::new(GroupOrigin::Cap)),
            Err(MeshError::TooLarge { .. })
        ));
        assert_eq!(a, before);
        // `append` checks for room up front and leaves `a` untouched.
        let b = FaceGroups::new(2);
        assert!(matches!(a.append(&b), Err(MeshError::TooLarge { .. })));
        assert_eq!(a, before);
        // A counter at `u32::MAX` is not a valid state.
        a.next_id = u32::MAX;
        assert!(a.validate(1).is_err());
    }

    #[test]
    #[should_panic(expected = "too large")]
    fn alloc_panics_when_exhausted() {
        let mut a = FaceGroups::new(0);
        a.next_id = u32::MAX - 1;
        a.alloc(FaceGroupInfo::new(GroupOrigin::Cap));
    }

    #[test]
    fn deserialisation_validates() {
        let a = FaceGroups::new(3);
        let json = serde_json::to_value(&a).unwrap();
        assert_eq!(
            serde_json::from_value::<FaceGroups>(json.clone()).unwrap(),
            a
        );
        let mut bad = json.clone();
        bad["per_face"][1] = 7.into(); // group not in the table
        assert!(serde_json::from_value::<FaceGroups>(bad).is_err());
        let mut bad = json;
        bad["next_id"] = 0.into(); // counter not above the table's ids
        assert!(serde_json::from_value::<FaceGroups>(bad).is_err());
    }
}
