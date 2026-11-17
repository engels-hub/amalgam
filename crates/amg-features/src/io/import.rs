//! Import side: `Imported` scene → one undoable "Insert <file>" transaction.

use std::sync::Arc;

use amg_core::Transform;
use amg_core::units::LengthUnit;
use amg_doc::{
    BodyGeom, BodyProps, Command, ComponentId, CreatedId, Document, FeatureBox, FeatureId,
};
use amg_io::{
    ImportOptions, ImportReport, Imported, NodeContent, Scene, SceneNode, UnitChoice, UpAxis,
};

use super::ImportPrepError;
use super::feature::{ImportMeshFeature, ImportSource, ImportedBodySpec, set_face_slots};

/// What the user chose in the import dialog.
#[derive(Clone, Debug)]
pub struct ImportChoices {
    /// Unit of the file numbers.
    pub unit: LengthUnit,
    /// Up axis of the file.
    pub up_axis: UpAxis,
    /// Component the feature is created in.
    pub target: ComponentId,
    /// One component per scene root (Phase 2, ASM-01). Not implemented yet: ignored.
    pub as_components: bool,
    /// Feature name in the timeline (`None`: "Insert <file name>").
    pub name: Option<String>,
}

impl ImportChoices {
    /// Millimetres, Z up, into `target`.
    pub fn new(target: ComponentId) -> ImportChoices {
        ImportChoices {
            unit: LengthUnit::Millimetre,
            up_axis: UpAxis::Z,
            target,
            as_components: false,
            name: None,
        }
    }

    /// The choices the dialog's `opts` stand for, given what the raw read reported: the unit
    /// resolves against the declared unit, the up axis is taken as is.
    pub fn from_options(target: ComponentId, opts: &ImportOptions, report: &ImportReport) -> Self {
        ImportChoices {
            unit: opts.unit.resolve(report.declared_unit),
            up_axis: opts.up_axis,
            ..ImportChoices::new(target)
        }
    }
}

/// `opts` changed so the reader keeps the file numbers: unit assumed mm (no scale), Z up.
/// [`import_commands`] needs a scene read this way; the dialog's unit and up axis go into
/// [`ImportChoices`] instead (see [`ImportChoices::from_options`]).
pub fn raw_options(opts: &ImportOptions) -> ImportOptions {
    ImportOptions {
        unit: UnitChoice::Assume(LengthUnit::Millimetre),
        up_axis: UpAxis::Z,
        ..opts.clone()
    }
}

/// Everything one import adds to a document, applied as one undo entry by
/// [`ImportPlan::apply`].
#[derive(Debug)]
pub struct ImportPlan {
    /// Undo label ("Insert <file>").
    pub label: String,
    /// The `AddFeature` command of the [`ImportMeshFeature`].
    pub add: Command,
    /// Baked raw meshes by body slot.
    pub baked: Vec<(u32, BodyGeom)>,
    /// Body properties (names) by body slot.
    pub props: Vec<(u32, BodyProps)>,
}

impl ImportPlan {
    /// The feature this plan adds.
    pub fn feature(&self) -> Option<&ImportMeshFeature> {
        match &self.add {
            Command::AddFeature { feature, .. } => feature.downcast_ref(),
            _ => None,
        }
    }

    /// Applies the plan as one undo entry; returns the new feature's id.
    ///
    /// # Errors
    /// [`ImportPrepError::Command`] if the document refuses it (for example in direct mode).
    pub fn apply(self, doc: &mut Document) -> Result<FeatureId, ImportPrepError> {
        let out = doc.add_feature_baked(&self.label, self.add, self.baked, self.props)?;
        match out.created.first() {
            Some(CreatedId::Feature(id)) => Ok(*id),
            _ => Err(ImportPrepError::Command(amg_doc::CommandError::Invalid(
                "the import created no feature".to_owned(),
            ))),
        }
    }
}

/// One placed object of the scene: object index, transform from the file root (file units)
/// and the best name found on the way.
struct Placed {
    object: u32,
    transform: Transform,
    name: Option<String>,
}

/// Depth-first, in declaration order (the order of [`Scene::instances`]).
fn placed(scene: &Scene) -> Vec<Placed> {
    let mut out = Vec::new();
    let mut stack: Vec<(&SceneNode, Transform, Option<&str>)> = scene
        .roots
        .iter()
        .rev()
        .map(|n| (n, Transform::IDENTITY, None))
        .collect();
    while let Some((node, parent, inherited)) = stack.pop() {
        let world = parent * node.transform;
        let name = node.name.as_deref().or(inherited);
        match &node.content {
            NodeContent::Mesh(i) => out.push(Placed {
                object: *i,
                transform: world,
                name: name.map(str::to_owned),
            }),
            NodeContent::Group(children) => {
                stack.extend(children.iter().rev().map(|c| (c, world, name)));
            }
        }
    }
    out
}

/// Builds the "Insert <file>" transaction for `imported`, which must have been read with
/// [`raw_options`] (file numbers, Z up). Each placed object becomes body slot `i` of one
/// [`ImportMeshFeature`] with its raw mesh baked (objects placed more than once share the
/// mesh allocation), the material table and the body name.
///
/// # Errors
/// [`ImportPrepError::NotRaw`] if the scene was scaled on read, [`ImportPrepError::Io`] if it
/// is invalid or has no placed object.
pub fn import_commands(
    imported: Imported,
    source: ImportSource,
    choices: &ImportChoices,
) -> Result<ImportPlan, ImportPrepError> {
    let Imported { scene, report } = imported;
    if let Some(u) = report.applied_unit
        && u != LengthUnit::Millimetre
    {
        return Err(ImportPrepError::NotRaw(u));
    }
    scene.validate()?;
    let placed = placed(&scene);
    if placed.is_empty() {
        return Err(amg_io::IoError::Empty.into());
    }
    let stem = std::path::Path::new(&source.file_name)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("Import")
        .to_owned();
    // One baked mesh per object (with its face slots), shared by its placements.
    let meshes: Vec<Arc<amg_mesh::IndexedMesh>> = scene
        .objects
        .iter()
        .map(|o| match &o.face_materials {
            Some(fm) => {
                let mut m = (*o.mesh).clone();
                set_face_slots(&mut m, fm);
                Arc::new(m)
            }
            None => Arc::clone(&o.mesh),
        })
        .collect();
    let multi = placed.len() > 1;
    let mut bodies = Vec::with_capacity(placed.len());
    let mut baked = Vec::with_capacity(placed.len());
    let mut props = Vec::with_capacity(placed.len());
    for (k, p) in placed.into_iter().enumerate() {
        let slot = u32::try_from(k).map_err(|_| amg_io::IoError::LimitExceeded {
            what: "bodies per import",
            limit: u64::from(u32::MAX),
            actual: k as u64,
        })?;
        let obj = &scene.objects[p.object as usize];
        let name = p.name.or_else(|| obj.name.clone()).unwrap_or_else(|| {
            if multi {
                format!("{stem} {}", k + 1)
            } else {
                stem.clone()
            }
        });
        bodies.push(ImportedBodySpec {
            slot,
            name: name.clone(),
            transform: p.transform,
            material: obj.material,
            filament: obj.filament,
        });
        baked.push((slot, BodyGeom::Mesh(Arc::clone(&meshes[p.object as usize]))));
        props.push((
            slot,
            BodyProps {
                name: Some(name),
                ..BodyProps::default()
            },
        ));
    }
    let label = format!("Insert {}", source.file_name);
    let feature = ImportMeshFeature {
        source,
        unit: choices.unit,
        up_axis: choices.up_axis,
        placement: Transform::IDENTITY,
        bodies,
        materials: scene.materials,
    };
    let add = Command::AddFeature {
        at: None,
        feature: FeatureBox::new(feature),
        context: choices.target,
        name: Some(choices.name.clone().unwrap_or_else(|| label.clone())),
    };
    Ok(ImportPlan {
        label,
        add,
        baked,
        props,
    })
}
