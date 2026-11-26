//! `io.import_brep` v1 and the STEP document adapter (S12 §8, IO-06).
//!
//! - [`ImportBrepFeature`]: the parts of one imported STEP file as baked `amg_brep::Shape`s
//!   (mm), each placed by its instance transform and the editable `placement`.
//! - [`import_brep_commands`] turns an [`amg_io::brep::BrepImport`] into a [`BrepImportPlan`]:
//!   - a file with an assembly structure (and [`ImportChoices::as_components`]) becomes
//!     components and occurrences (ASM-01): one component per assembly node, one component per
//!     part with one `ImportBrepFeature` holding its body, and one occurrence per instance
//!     (instances of a shared part are occurrences of one component);
//!   - otherwise one `ImportBrepFeature` in the target component, one body per part instance.
//!
//!   [`BrepImportPlan::apply`] adds everything as one undo entry
//!   ([`amg_doc::Document::baked_transaction`]).
//! - [`brep_scene_from_document`] turns the component tree and its solid bodies back into an
//!   [`amg_io::brep::BrepScene`] for [`amg_io::brep::export_step`]: components become groups,
//!   a component with exactly one solid body and no sub-components becomes a part (shared
//!   between its occurrences), and the colours an import stored come back.

use std::collections::{BTreeMap, BTreeSet};

use amg_brep::Shape;
use amg_core::Transform;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, BodyProps, Command, ComponentId, CreatedId,
    Dependency, Design, Document, Evaluated, Feature, FeatureBox, FeatureError, FeatureId,
    FeatureOutput, OccurrenceId, RecomputeCtx,
};
use amg_io::brep::{BrepContent, BrepImport, BrepNode, BrepObject, BrepScene, HealthReport};
use amg_mesh::Rgba8;
use serde::{Deserialize, Serialize};

use super::ImportPrepError;
use super::export::ExportSelection;
use super::feature::{ImportSource, ImportedBodySpec};
use super::import::ImportChoices;

/// Registered kind of [`ImportBrepFeature`].
pub const IMPORT_BREP_KIND: &str = "io.import_brep";

/// Face colours of one body of an [`ImportBrepFeature`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BrepFaceColors {
    /// Body slot.
    pub slot: u32,
    /// `(face index of the part shape, index into ImportBrepFeature::colors)`, ascending.
    pub faces: Vec<(u32, u16)>,
}

/// Timeline feature `io.import_brep` v1: imported B-rep parts as baked shapes (units already
/// mm). Body slot `i` is baked payload `i`; `compute` places it by
/// `placement · body.transform`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImportBrepFeature {
    /// The file.
    pub source: ImportSource,
    /// Placement in the context component, mm (the move gizmo writes it).
    pub placement: Transform,
    /// Output bodies. `transform` is the instance transform in mm; `material` indexes
    /// [`colors`](Self::colors) (the body colour).
    pub bodies: Vec<ImportedBodySpec>,
    /// sRGB colour table of the bodies and faces (until MAT-02 appearance overrides).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub colors: Vec<Rgba8>,
    /// Per-face colours, by body slot.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub face_colors: Vec<BrepFaceColors>,
}

impl ImportBrepFeature {
    /// The part-to-component transform of `body`.
    pub fn body_transform(&self, body: &ImportedBodySpec) -> Transform {
        self.placement * body.transform
    }

    /// The colour the import stored for body `slot`.
    pub fn body_color(&self, slot: u32) -> Option<Rgba8> {
        let b = self.bodies.iter().find(|b| b.slot == slot)?;
        self.colors.get(usize::from(b.material?)).copied()
    }

    /// The face colours the import stored for body `slot` (face index of the part shape).
    pub fn body_face_colors(&self, slot: u32) -> Vec<(u32, Rgba8)> {
        self.face_colors
            .iter()
            .filter(|f| f.slot == slot)
            .flat_map(|f| f.faces.iter())
            .filter_map(|&(face, c)| Some((face, *self.colors.get(usize::from(c))?)))
            .collect()
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

impl Feature for ImportBrepFeature {
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
                && usize::from(m) >= self.colors.len()
            {
                return Err(bad(
                    "bodies",
                    format!("colour {m} of slot {} is not in the table", b.slot),
                ));
            }
        }
        for f in &self.face_colors {
            if !seen.contains(&f.slot) {
                return Err(bad("face_colors", format!("slot {} is not a body", f.slot)));
            }
            if f.faces
                .iter()
                .any(|&(_, c)| usize::from(c) >= self.colors.len())
            {
                return Err(bad(
                    "face_colors",
                    format!("a face colour of slot {} is not in the table", f.slot),
                ));
            }
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        let n = self.bodies.len().max(1);
        for (i, b) in self.bodies.iter().enumerate() {
            ctx.check_cancelled()?;
            #[allow(clippy::cast_precision_loss, reason = "progress fraction")]
            ctx.progress(i as f32 / n as f32, "placing imported parts");
            let shape = match ctx.baked(b.slot)? {
                BodyGeom::Solid(s) => s.clone(),
                _ => return Err(bad("bodies", format!("slot {} is not a B-rep", b.slot))),
            };
            let t = self.body_transform(b);
            // The identity keeps the baked handle itself (cheap, and the export can tell an
            // unchanged body by `ptr_eq`).
            if shape.brep().is_none() {
                return Err(bad(
                    "bodies",
                    format!("slot {} has no B-rep geometry", b.slot),
                ));
            }
            let shape = if t == Transform::IDENTITY {
                shape
            } else {
                shape
                    .transformed(&t)
                    .map_err(|e| FeatureError::Kernel(e.to_string()))?
            };
            out.bodies.push(BodyChange {
                id: ctx.new_body_id(b.slot),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Solid(shape),
                    naming: None,
                },
            });
        }
        Ok(out)
    }
}

amg_doc::register_feature!(ImportBrepFeature, "io.import_brep", version = 1);

// ---------------------------------------------------------------- import plan

/// A component of the plan: the target, or the `i`-th component the plan creates.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Ctx {
    Target,
    New(usize),
}

#[derive(Debug)]
enum Step {
    /// Creates component `key` with its first occurrence.
    NewComponent {
        parent: Ctx,
        name: String,
        transform: Transform,
        occ_name: Option<String>,
    },
    /// Another occurrence of component `component`.
    AddOccurrence {
        parent: Ctx,
        component: usize,
        transform: Transform,
        name: Option<String>,
    },
    /// An `ImportBrepFeature` with its baked shapes in `context`.
    Feature {
        context: Ctx,
        add: Box<ImportBrepFeature>,
        name: String,
        baked: Vec<(u32, BodyGeom)>,
        props: Vec<(u32, BodyProps)>,
    },
}

/// Everything one STEP import adds to a document, applied as one undo entry by
/// [`BrepImportPlan::apply`].
#[derive(Debug)]
pub struct BrepImportPlan {
    /// Undo label ("Insert <file>").
    pub label: String,
    /// Whether the plan creates components (the file has an assembly structure).
    pub as_components: bool,
    /// The import's health report (healed and failed parts).
    pub health: HealthReport,
    steps: Vec<Step>,
}

/// What [`BrepImportPlan::apply`] created.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BrepImportOutcome {
    /// The `ImportBrepFeature`s, one per part component (or one in total).
    pub features: Vec<FeatureId>,
    /// The new components, in creation order.
    pub components: Vec<ComponentId>,
    /// The new occurrences, in creation order.
    pub occurrences: Vec<OccurrenceId>,
}

impl BrepImportPlan {
    /// How many `ImportBrepFeature`s the plan adds.
    pub fn feature_count(&self) -> usize {
        self.steps
            .iter()
            .filter(|s| matches!(s, Step::Feature { .. }))
            .count()
    }

    /// The features the plan adds.
    pub fn features(&self) -> impl Iterator<Item = &ImportBrepFeature> {
        self.steps.iter().filter_map(|s| match s {
            Step::Feature { add, .. } => Some(&**add),
            _ => None,
        })
    }

    /// Applies the plan as one undo entry into the target of the [`ImportChoices`].
    ///
    /// # Errors
    /// [`ImportPrepError::Command`] if the document refuses a step (for example in direct
    /// mode); nothing changes then.
    pub fn apply(
        self,
        doc: &mut Document,
        target: ComponentId,
    ) -> Result<BrepImportOutcome, ImportPrepError> {
        let mut tx = doc.baked_transaction(&self.label);
        let mut out = BrepImportOutcome::default();
        let resolve = |c: Ctx, comps: &[ComponentId]| match c {
            Ctx::Target => target,
            Ctx::New(i) => comps[i],
        };
        for step in self.steps {
            match step {
                Step::NewComponent {
                    parent,
                    name,
                    transform,
                    occ_name,
                } => {
                    let created = tx.apply(Command::NewComponent {
                        parent: resolve(parent, &out.components),
                        name,
                        transform,
                    })?;
                    for c in created {
                        match c {
                            CreatedId::Component(c) => out.components.push(c),
                            CreatedId::Occurrence(o) => {
                                if occ_name.is_some() {
                                    tx.apply(Command::SetOccurrenceName {
                                        occ: o,
                                        name: occ_name.clone(),
                                    })?;
                                }
                                out.occurrences.push(o);
                            }
                            _ => {}
                        }
                    }
                }
                Step::AddOccurrence {
                    parent,
                    component,
                    transform,
                    name,
                } => {
                    let created = tx.apply(Command::AddOccurrence {
                        parent: resolve(parent, &out.components),
                        component: out.components[component],
                        transform,
                        name,
                    })?;
                    out.occurrences
                        .extend(created.into_iter().filter_map(|c| match c {
                            CreatedId::Occurrence(o) => Some(o),
                            _ => None,
                        }));
                }
                Step::Feature {
                    context,
                    add,
                    name,
                    baked,
                    props,
                } => {
                    let created = tx.add_feature_baked(
                        Command::AddFeature {
                            at: None,
                            feature: FeatureBox::new(*add),
                            context: resolve(context, &out.components),
                            name: Some(name),
                        },
                        baked,
                        props,
                    )?;
                    out.features
                        .extend(created.into_iter().filter_map(|c| match c {
                            CreatedId::Feature(f) => Some(f),
                            _ => None,
                        }));
                }
            }
        }
        tx.commit();
        Ok(out)
    }
}

/// The colour table of one feature being built.
#[derive(Default)]
struct Colors {
    table: Vec<Rgba8>,
}

impl Colors {
    fn index(&mut self, c: Rgba8) -> u16 {
        if let Some(i) = self.table.iter().position(|&t| t == c) {
            return u16::try_from(i).unwrap_or(u16::MAX);
        }
        self.table.push(c);
        u16::try_from(self.table.len() - 1).unwrap_or(u16::MAX)
    }
}

/// One feature under construction.
struct FeatureDraft {
    bodies: Vec<ImportedBodySpec>,
    baked: Vec<(u32, BodyGeom)>,
    props: Vec<(u32, BodyProps)>,
    face_colors: Vec<BrepFaceColors>,
    colors: Colors,
}

impl FeatureDraft {
    fn new() -> FeatureDraft {
        FeatureDraft {
            bodies: Vec::new(),
            baked: Vec::new(),
            props: Vec::new(),
            face_colors: Vec::new(),
            colors: Colors::default(),
        }
    }

    fn push(
        &mut self,
        object: &BrepObject,
        name: String,
        transform: Transform,
        color: Option<Rgba8>,
    ) -> Result<(), ImportPrepError> {
        let k = self.bodies.len();
        let slot = u32::try_from(k).map_err(|_| amg_io::IoError::LimitExceeded {
            what: "bodies per import",
            limit: u64::from(u32::MAX),
            actual: k as u64,
        })?;
        let material = color.map(|c| self.colors.index(c));
        if !object.face_colors.is_empty() {
            let faces = object
                .face_colors
                .iter()
                .map(|&(f, c)| (f, self.colors.index(c)))
                .collect();
            self.face_colors.push(BrepFaceColors { slot, faces });
        }
        self.bodies.push(ImportedBodySpec {
            slot,
            name: name.clone(),
            transform,
            material,
            filament: None,
        });
        self.baked.push((
            slot,
            BodyGeom::Solid(amg_doc::stub::Shape::from_brep(object.shape.clone())),
        ));
        self.props.push((
            slot,
            BodyProps {
                name: Some(name),
                ..BodyProps::default()
            },
        ));
        Ok(())
    }

    fn step(self, context: Ctx, source: &ImportSource, name: String) -> Step {
        Step::Feature {
            context,
            add: Box::new(ImportBrepFeature {
                source: source.clone(),
                placement: Transform::IDENTITY,
                bodies: self.bodies,
                colors: self.colors.table,
                face_colors: self.face_colors,
            }),
            name,
            baked: self.baked,
            props: self.props,
        }
    }
}

/// Builds the "Insert <file>" transaction for a STEP import.
///
/// With [`ImportChoices::as_components`] and a file that has an assembly structure, every
/// assembly node becomes a component (name, transform) and every part a component with one
/// `ImportBrepFeature` and one body; instances of one part (with the same instance colour)
/// are occurrences of the same component, and an instance name that differs from the part's
/// becomes the occurrence name. Otherwise every part instance becomes a body of one feature
/// in the target, placed by its transform from the file root. Body names are part names
/// (instance names in the flat case), colours go into the feature's colour table.
/// `choices.unit` and `choices.up_axis` are ignored: STEP declares its unit, and the geometry
/// is already mm.
///
/// # Errors
/// [`ImportPrepError::Io`] if the scene is invalid or places nothing.
pub fn import_brep_commands(
    imported: BrepImport,
    source: ImportSource,
    choices: &ImportChoices,
) -> Result<BrepImportPlan, ImportPrepError> {
    let BrepImport { scene, health, .. } = imported;
    scene.validate()?;
    let instances = scene.instances();
    if instances.is_empty() {
        return Err(amg_io::IoError::Empty.into());
    }
    let stem = std::path::Path::new(&source.file_name)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("Import")
        .to_owned();
    let label = format!("Insert {}", source.file_name);
    let as_components = choices.as_components && scene.has_assemblies();
    let mut steps = Vec::new();
    if as_components {
        let mut b = TreeBuilder {
            scene: &scene,
            source: &source,
            stem: &stem,
            steps: &mut steps,
            components: 0,
            parts: BTreeMap::new(),
        };
        for root in &scene.roots {
            b.node(root, Ctx::Target)?;
        }
    } else {
        let multi = instances.len() > 1;
        let mut draft = FeatureDraft::new();
        let colors = instance_colors(&scene);
        for (k, ((object, transform, node_name), color)) in
            instances.into_iter().zip(colors).enumerate()
        {
            let obj = &scene.objects[object as usize];
            let name = node_name
                .map(str::to_owned)
                .or_else(|| obj.name.clone())
                .unwrap_or_else(|| {
                    if multi {
                        format!("{stem} {}", k + 1)
                    } else {
                        stem.clone()
                    }
                });
            draft.push(obj, name, transform, color)?;
        }
        let name = choices.name.clone().unwrap_or_else(|| label.clone());
        steps.push(draft.step(Ctx::Target, &source, name));
    }
    Ok(BrepImportPlan {
        label,
        as_components,
        health,
        steps,
    })
}

/// The effective colour of every instance, in [`BrepScene::instances`] order: the nearest
/// node colour on the way down, else the part's.
fn instance_colors(scene: &BrepScene) -> Vec<Option<Rgba8>> {
    fn walk(s: &BrepScene, n: &BrepNode, inherited: Option<Rgba8>, out: &mut Vec<Option<Rgba8>>) {
        let c = n.color.or(inherited);
        match &n.content {
            BrepContent::Shape(o) => out.push(c.or(s.objects[*o as usize].color)),
            BrepContent::Group(children) => {
                for ch in children {
                    walk(s, ch, c, out);
                }
            }
        }
    }
    let mut out = Vec::new();
    for r in &scene.roots {
        walk(scene, r, None, &mut out);
    }
    out
}

/// Builds the component steps of an assembly import.
struct TreeBuilder<'a> {
    scene: &'a BrepScene,
    source: &'a ImportSource,
    stem: &'a str,
    steps: &'a mut Vec<Step>,
    components: usize,
    /// `(object, instance colour)` → its part component.
    parts: BTreeMap<(u32, Option<[u8; 4]>), usize>,
}

impl TreeBuilder<'_> {
    fn new_component(
        &mut self,
        parent: Ctx,
        name: String,
        transform: Transform,
        occ_name: Option<String>,
    ) -> usize {
        self.steps.push(Step::NewComponent {
            parent,
            name,
            transform,
            occ_name,
        });
        self.components += 1;
        self.components - 1
    }

    fn node(&mut self, n: &BrepNode, parent: Ctx) -> Result<(), ImportPrepError> {
        match &n.content {
            BrepContent::Group(children) => {
                let name = n
                    .name
                    .clone()
                    .unwrap_or_else(|| format!("{} {}", self.stem, self.components + 1));
                let key = self.new_component(parent, name, n.transform, None);
                for c in children {
                    self.node(c, Ctx::New(key))?;
                }
            }
            BrepContent::Shape(o) => {
                let obj = &self.scene.objects[*o as usize];
                let comp_name = obj
                    .name
                    .clone()
                    .or_else(|| n.name.clone())
                    .unwrap_or_else(|| format!("{} {}", self.stem, self.components + 1));
                let occ_name = n.name.clone().filter(|nm| *nm != comp_name);
                let color = n.color.or(obj.color);
                let key = (*o, n.color.map(|c| c.0));
                if let Some(&component) = self.parts.get(&key) {
                    self.steps.push(Step::AddOccurrence {
                        parent,
                        component,
                        transform: n.transform,
                        name: occ_name,
                    });
                } else {
                    let c = self.new_component(parent, comp_name.clone(), n.transform, occ_name);
                    self.parts.insert(key, c);
                    let mut draft = FeatureDraft::new();
                    draft.push(obj, comp_name.clone(), Transform::IDENTITY, color)?;
                    self.steps.push(draft.step(
                        Ctx::New(c),
                        self.source,
                        format!("Insert {comp_name}"),
                    ));
                }
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------- export

/// The `ImportBrepFeature` that created `id`, if any.
fn import_feature(design: &Design, id: BodyId) -> Option<&ImportBrepFeature> {
    let (_, node) = design.timeline().node(id.origin)?;
    node.feature.downcast_ref::<ImportBrepFeature>()
}

/// Document → [`BrepScene`] for [`amg_io::brep::export_step`].
///
/// The tree starts at the root (or at the component of [`ExportSelection::Component`]): its
/// solid bodies are parts placed at the identity, and every occurrence below it is a group
/// named after the occurrence, with the occurrence's local transform. An occurrence whose
/// component holds exactly one solid body and no occurrences is a part instance instead, so
/// every occurrence of one component shares one part. [`ExportSelection::AllVisible`] skips
/// hidden bodies and occurrences; [`ExportSelection::Bodies`] keeps only those bodies. Empty
/// groups are dropped. Mesh, implicit and form bodies are skipped.
///
/// Colours: a body an `ImportBrepFeature` created keeps the colour the import stored; its face
/// colours are kept while the body still has the imported face count (a later feature that
/// changed the faces drops them).
///
/// # Errors
/// [`ImportPrepError::EmptySelection`] when no solid body is selected.
pub fn brep_scene_from_document(
    design: &Design,
    eval: &Evaluated,
    sel: &ExportSelection,
) -> Result<BrepScene, ImportPrepError> {
    let keep: Option<BTreeSet<BodyId>> = match sel {
        ExportSelection::Bodies(ids) => Some(ids.iter().copied().collect()),
        _ => None,
    };
    let visible_only = matches!(sel, ExportSelection::AllVisible);
    let start = match sel {
        ExportSelection::Component(c) => *c,
        _ => design.root(),
    };
    // Solid bodies per component, in id order.
    let mut by_component: BTreeMap<ComponentId, Vec<(BodyId, Shape)>> = BTreeMap::new();
    for (id, body) in eval.bodies() {
        let Some(shape) = (match &body.geom {
            BodyGeom::Solid(s) => s.brep(),
            _ => None,
        }) else {
            continue;
        };
        if keep.as_ref().is_some_and(|k| !k.contains(&id)) {
            continue;
        }
        if visible_only && !design.body_props(id).visible {
            continue;
        }
        by_component
            .entry(body.component)
            .or_default()
            .push((id, shape.clone()));
    }
    let mut w = ExportWalk {
        design,
        eval,
        by_component: &by_component,
        visible_only,
        objects: Vec::new(),
        object_of: BTreeMap::new(),
    };
    let roots = w.component(start, 0);
    if roots.is_empty() {
        return Err(ImportPrepError::EmptySelection);
    }
    let mut scene = BrepScene {
        objects: w.objects,
        roots,
        ..BrepScene::default()
    };
    scene
        .metadata
        .insert("Title".to_owned(), design.meta().name.clone());
    Ok(scene)
}

struct ExportWalk<'a> {
    design: &'a Design,
    eval: &'a Evaluated,
    by_component: &'a BTreeMap<ComponentId, Vec<(BodyId, Shape)>>,
    visible_only: bool,
    objects: Vec<BrepObject>,
    object_of: BTreeMap<BodyId, u32>,
}

impl ExportWalk<'_> {
    fn object(&mut self, id: BodyId, shape: &Shape) -> u32 {
        if let Some(&o) = self.object_of.get(&id) {
            return o;
        }
        let f = import_feature(self.design, id);
        let face_colors = f.map(|f| f.body_face_colors(id.slot)).unwrap_or_default();
        let faces = shape.count(amg_brep::SubKind::Face);
        let face_colors = if face_colors.iter().all(|&(face, _)| face < faces)
            && self
                .design
                .baked(id)
                .and_then(|b| b.geom.as_ref())
                .is_none_or(|g| match g {
                    BodyGeom::Solid(s) => s
                        .brep()
                        .is_some_and(|s| s.count(amg_brep::SubKind::Face) == faces),
                    _ => false,
                }) {
            face_colors
        } else {
            Vec::new()
        };
        let o = u32::try_from(self.objects.len()).unwrap_or(u32::MAX);
        self.objects.push(BrepObject {
            name: self.design.body_props(id).name.clone(),
            shape: shape.clone(),
            color: f.and_then(|f| f.body_color(id.slot)),
            face_colors,
        });
        self.object_of.insert(id, o);
        o
    }

    fn component(&mut self, c: ComponentId, depth: usize) -> Vec<BrepNode> {
        let mut out = Vec::new();
        if depth > amg_io::MAX_GROUP_DEPTH {
            return out;
        }
        for (id, shape) in self.by_component.get(&c).into_iter().flatten() {
            let o = self.object(*id, shape);
            out.push(BrepNode {
                name: self.design.body_props(*id).name.clone(),
                transform: Transform::IDENTITY,
                color: None,
                content: BrepContent::Shape(o),
            });
        }
        let Some(comp) = self.design.component(c) else {
            return out;
        };
        for &occ_id in &comp.children {
            let Some(occ) = self.design.occurrence(occ_id) else {
                continue;
            };
            if self.visible_only && !occ.visible {
                continue;
            }
            let Some(child) = self.design.component(occ.component) else {
                continue;
            };
            let transform = self
                .eval
                .placements()
                .local(occ_id)
                .unwrap_or(occ.transform);
            let name = Some(occ.name.clone().unwrap_or_else(|| child.name.clone()));
            let bodies = self.by_component.get(&occ.component);
            if child.children.is_empty()
                && let Some([(id, shape)]) = bodies.map(Vec::as_slice)
            {
                let o = self.object(*id, shape);
                out.push(BrepNode {
                    name,
                    transform,
                    color: None,
                    content: BrepContent::Shape(o),
                });
                continue;
            }
            let children = self.component(occ.component, depth + 1);
            if !children.is_empty() {
                out.push(BrepNode {
                    name,
                    transform,
                    color: None,
                    content: BrepContent::Group(children),
                });
            }
        }
        out
    }
}
