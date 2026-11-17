//! `io.import_mesh` v1: imported meshes as baked raw payloads plus unit, up axis and placement.

use std::collections::BTreeSet;
use std::sync::Arc;

use amg_core::Transform;
use amg_core::units::LengthUnit;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, Dependency, Diagnostic, Feature, FeatureError,
    FeatureOutput, RecomputeCtx,
};
use amg_io::{Format, MaterialSlot, UpAxis, scale_to_mm};
use amg_mesh::{IndexedMesh, TriProperty};
use serde::{Deserialize, Serialize};

/// Registered kind of [`ImportMeshFeature`].
pub const IMPORT_MESH_KIND: &str = "io.import_mesh";

/// `TriProperty::group` that marks per-face material slots of an import: `index[0]` is the
/// slot in [`ImportMeshFeature::materials`] (all three corners equal). S07 mesh ops propagate
/// `tri_properties`, so the slots follow the faces until S07 `property_groups` gives the slot
/// table a home on the mesh itself.
pub const SLOT_PROPERTY_GROUP: u32 = 1;

/// Where an import came from. Display and provenance only; geometry is in the baked payloads.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImportSource {
    /// File name, display only.
    pub file_name: String,
    /// Last known absolute path (for "Reload", Phase 2).
    pub path_hint: Option<String>,
    /// Detected format.
    pub format: Format,
    /// blake3 of the source bytes, lower-case hex.
    pub blake3: String,
    /// Unit the file declared, if any.
    pub declared_unit: Option<LengthUnit>,
}

impl ImportSource {
    /// A source description for `bytes` read from a file called `file_name`.
    pub fn from_bytes(
        file_name: impl Into<String>,
        format: Format,
        declared_unit: Option<LengthUnit>,
        bytes: &[u8],
    ) -> ImportSource {
        ImportSource {
            file_name: file_name.into(),
            path_hint: None,
            format,
            blake3: blake3::hash(bytes).to_hex().to_string(),
            declared_unit,
        }
    }
}

/// One output body of an import: body slot `slot` of the feature, read from the baked payload
/// of the same slot.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImportedBodySpec {
    /// Body slot (also the baked payload slot).
    pub slot: u32,
    /// Body name at import time.
    pub name: String,
    /// Build-item (instance) transform, in file units.
    pub transform: Transform,
    /// Default material slot of the object, in [`ImportMeshFeature::materials`].
    pub material: Option<u16>,
    /// 1-based slicer extruder of the object (3MF slicer flavours), if the file set one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub filament: Option<u8>,
}

/// Timeline feature `io.import_mesh` v1. Holds the raw imported meshes as baked bodies (slot
/// `i` = placed object `i` of the import) and computes
/// `placement · up_axis · scale(unit) · transform` for each.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImportMeshFeature {
    /// The file.
    pub source: ImportSource,
    /// Unit of the file numbers (editable after import).
    pub unit: LengthUnit,
    /// Up axis of the file (editable after import).
    pub up_axis: UpAxis,
    /// Placement in the context component, mm (the move gizmo writes it).
    pub placement: Transform,
    /// Output bodies.
    pub bodies: Vec<ImportedBodySpec>,
    /// Material slot table (until S07 `property_groups` lands).
    pub materials: Vec<MaterialSlot>,
}

impl ImportMeshFeature {
    /// The object-to-component transform of `body` in mm, applied *after* the raw positions
    /// are scaled to mm: `placement · up_axis · T_mm`, where `T_mm` is the build transform
    /// with its translation scaled. A uniform scale commutes with the linear part, so this
    /// equals `placement · up_axis · scale · transform`.
    pub fn object_transform(&self, body: &ImportedBodySpec) -> Transform {
        let mut t = body.transform;
        t.translation.x = scale_to_mm(t.translation.x, self.unit);
        t.translation.y = scale_to_mm(t.translation.y, self.unit);
        t.translation.z = scale_to_mm(t.translation.z, self.unit);
        self.placement * self.up_axis.to_z_up() * t
    }

    /// The raw mesh with every coordinate scaled to mm (object space, exact factors).
    pub fn object_mesh(&self, raw: &IndexedMesh) -> IndexedMesh {
        let mut m = raw.clone();
        if self.unit != LengthUnit::Millimetre {
            for p in &mut m.positions {
                p.x = scale_to_mm(p.x, self.unit);
                p.y = scale_to_mm(p.y, self.unit);
                p.z = scale_to_mm(p.z, self.unit);
            }
        }
        m
    }

    /// The body mesh as `compute` emits it: [`object_mesh`](Self::object_mesh) moved by
    /// [`object_transform`](Self::object_transform) (skipped when that is the identity, so a
    /// unit change alone is an exact per-coordinate scale).
    pub fn body_mesh(&self, raw: &IndexedMesh, body: &ImportedBodySpec) -> IndexedMesh {
        let mut m = self.object_mesh(raw);
        let t = self.object_transform(body);
        if t != Transform::IDENTITY {
            m.transform(&t);
        }
        m
    }

    /// True if any baked mesh carries per-face slots (health warning).
    fn has_face_slots(raw: &IndexedMesh) -> bool {
        raw.attrs
            .tri_properties
            .as_ref()
            .is_some_and(|tp| tp.iter().any(|p| p.group == SLOT_PROPERTY_GROUP))
    }
}

fn bad(field: &'static str, msg: impl Into<String>) -> FeatureError {
    FeatureError::InvalidValue {
        field,
        msg: msg.into(),
    }
}

fn finite(t: &Transform) -> bool {
    t.matrix3.is_finite() && t.translation.is_finite()
}

impl Feature for ImportMeshFeature {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if !finite(&self.placement) {
            return Err(bad("placement", "must be finite"));
        }
        let mut seen = BTreeSet::new();
        for b in &self.bodies {
            if !seen.insert(b.slot) {
                return Err(bad("bodies", format!("slot {} is listed twice", b.slot)));
            }
            if !finite(&b.transform) {
                return Err(bad(
                    "bodies",
                    format!("transform of slot {} is not finite", b.slot),
                ));
            }
            if let Some(m) = b.material
                && usize::from(m) >= self.materials.len()
            {
                return Err(bad(
                    "bodies",
                    format!("material {m} of slot {} is not in the table", b.slot),
                ));
            }
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        let mut face_slots = false;
        let n = self.bodies.len().max(1);
        for (i, b) in self.bodies.iter().enumerate() {
            ctx.check_cancelled()?;
            #[allow(clippy::cast_precision_loss, reason = "progress fraction")]
            ctx.progress(i as f32 / n as f32, "placing imported meshes");
            let raw = match ctx.baked(b.slot)? {
                BodyGeom::Mesh(m) => Arc::clone(m),
                _ => return Err(bad("bodies", format!("slot {} is not a mesh", b.slot))),
            };
            face_slots |= Self::has_face_slots(&raw);
            out.bodies.push(BodyChange {
                id: ctx.new_body_id(b.slot),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(self.body_mesh(&raw, b))),
                    naming: None,
                },
            });
        }
        if face_slots {
            out.warnings.push(Diagnostic {
                message: "per-face materials are kept on the imported bodies and on bodies \
                          modified in place; bodies made by other features lose them on export \
                          (until S07 property groups)"
                    .to_owned(),
                cause: None,
                entities: Vec::new(),
            });
        }
        Ok(out)
    }
}

amg_doc::register_feature!(ImportMeshFeature, "io.import_mesh", version = 1);

/// Writes per-face slots into `mesh.attrs.tri_properties` ([`SLOT_PROPERTY_GROUP`]).
/// [`amg_io::NO_SLOT`] faces get [`TriProperty::NONE`].
pub(crate) fn set_face_slots(mesh: &mut IndexedMesh, slots: &[u16]) {
    mesh.attrs.tri_properties = Some(
        slots
            .iter()
            .map(|&s| {
                if s == amg_io::NO_SLOT {
                    TriProperty::NONE
                } else {
                    TriProperty {
                        group: SLOT_PROPERTY_GROUP,
                        index: [u32::from(s); 3],
                    }
                }
            })
            .collect(),
    );
}

/// Reads the per-face slots back (`None` if the mesh has none of this group).
pub(crate) fn face_slots(mesh: &IndexedMesh) -> Option<Vec<u16>> {
    let tp = mesh.attrs.tri_properties.as_ref()?;
    if !tp.iter().any(|p| p.group == SLOT_PROPERTY_GROUP) {
        return None;
    }
    Some(
        tp.iter()
            .map(|p| {
                if p.group == SLOT_PROPERTY_GROUP {
                    u16::try_from(p.index[0]).unwrap_or(amg_io::NO_SLOT)
                } else {
                    amg_io::NO_SLOT
                }
            })
            .collect(),
    )
}
