//! STEP and IGES models as plain data (S02 §9, BREP-13): `ForeignModel`, its conversion to and
//! from the shim's flat node list, and the file unit tables, in plain Rust (unit-tested without
//! OCCT). The file I/O is `crate::io`; the OCCT calls are in `backend/occt/xde.rs` and
//! `cpp/occt_xde.cpp`.

use amg_core::Transform;

use crate::error::{KResult, KernelError};
use crate::shape::{FaceId, Shape, SubKind, is_rigid};

/// A STEP or IGES file's content: a tree of nodes over shared part prototypes, in mm.
///
/// XCAF stays inside `amg-brep`: S12 maps this to components (IO-06) without seeing it.
#[derive(Clone, Debug)]
pub struct ForeignModel {
    /// The top-level nodes (XCAF free shapes), in file order.
    pub roots: Vec<ForeignNode>,
    /// The shared-prototype table that [`ForeignNode::instance_of`] indexes: one entry per part
    /// (XCAF simple-shape label), in first-use order. Nodes showing a prototype carry a clone of
    /// its shape (the same TShapes), so all instances of one part share their geometry.
    pub prototypes: Vec<ForeignPrototype>,
    /// The file's declared length unit in mm (25.4 for inches). Geometry is already in mm. A
    /// write uses it as the file's unit when it is mm, cm, m, inch, foot or micron, else mm.
    pub source_unit_mm: f64,
    /// Non-fatal notes of the read (healing that failed, edges whose missing 3D curve was
    /// built, an unknown unit).
    pub warnings: Vec<String>,
}

impl Default for ForeignModel {
    fn default() -> Self {
        ForeignModel {
            roots: Vec::new(),
            prototypes: Vec::new(),
            source_unit_mm: 1.0,
            warnings: Vec::new(),
        }
    }
}

impl ForeignModel {
    /// A model in mm with these roots and no prototype table (a write makes one prototype per
    /// node with a shape and no `instance_of`).
    pub fn new(roots: Vec<ForeignNode>) -> ForeignModel {
        ForeignModel {
            roots,
            ..ForeignModel::default()
        }
    }

    /// Every node, depth first, parents before children.
    pub fn nodes(&self) -> Vec<&ForeignNode> {
        fn walk<'a>(node: &'a ForeignNode, out: &mut Vec<&'a ForeignNode>) {
            out.push(node);
            for child in &node.children {
                walk(child, out);
            }
        }
        let mut out = Vec::new();
        for root in &self.roots {
            walk(root, &mut out);
        }
        out
    }
}

/// One node of a [`ForeignModel`]: a part instance (with `shape`) or an assembly (with
/// `children`).
#[derive(Clone, Debug)]
pub struct ForeignNode {
    /// The instance's name, else its part's or assembly's name.
    pub name: Option<String>,
    /// Placement relative to the parent (rigid). Roots read from a file have the identity.
    pub transform: Transform,
    /// Linear RGBA from XCAF: the instance's colour, else its part's or assembly's.
    pub color: Option<[f32; 4]>,
    /// The part shape (solid, shell or compound) of a part instance.
    pub shape: Option<Shape>,
    /// Colours of single faces of `shape` (indices in `shape`'s own index space).
    pub face_colors: Vec<(FaceId, [f32; 4])>,
    /// The components of an assembly.
    pub children: Vec<ForeignNode>,
    /// Index into [`ForeignModel::prototypes`]: instances of one part share it. A write uses
    /// that prototype's shape and face colours (this node's `shape` and `face_colors` are then
    /// ignored); without it, a node's `shape` becomes a prototype of its own.
    pub instance_of: Option<u32>,
}

impl ForeignNode {
    /// An instance of `shape` placed by `transform`, with no prototype index.
    pub fn part(name: Option<&str>, shape: Shape, transform: Transform) -> ForeignNode {
        ForeignNode {
            name: name.map(str::to_owned),
            transform,
            color: None,
            shape: Some(shape),
            face_colors: Vec::new(),
            children: Vec::new(),
            instance_of: None,
        }
    }

    /// An assembly of `children` placed by `transform`.
    pub fn assembly(
        name: Option<&str>,
        children: Vec<ForeignNode>,
        transform: Transform,
    ) -> ForeignNode {
        ForeignNode {
            name: name.map(str::to_owned),
            transform,
            color: None,
            shape: None,
            face_colors: Vec::new(),
            children,
            instance_of: None,
        }
    }
}

/// One shared part of a [`ForeignModel`].
#[derive(Clone, Debug)]
pub struct ForeignPrototype {
    /// The part's (product's) name.
    pub name: Option<String>,
    /// The part shape, healed when the import asked for it.
    pub shape: Shape,
    /// The part's colour (linear RGBA).
    pub color: Option<[f32; 4]>,
    /// Colours of single faces of `shape`.
    pub face_colors: Vec<(FaceId, [f32; 4])>,
}

/// What a STEP or IGES import reads and does.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ImportOpts {
    /// Run [`crate::BrepKernel::heal`] (default options) on every prototype; face colours follow
    /// the healing history. A prototype that fails to heal is kept as read, with a warning.
    pub heal: bool,
    /// Read part, instance and face colours.
    pub read_colors: bool,
    /// Read part, assembly and instance names.
    pub read_names: bool,
}

impl Default for ImportOpts {
    fn default() -> Self {
        ImportOpts {
            heal: true,
            read_colors: true,
            read_names: true,
        }
    }
}

/// The STEP application protocol a write uses.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum StepSchema {
    /// AP203 (configuration-controlled design; OCCT adds `SHAPE_APPEARANCE_LAYER_MIM` for colours).
    Ap203,
    /// AP214 IS (automotive design).
    Ap214,
    /// AP242 DIS (managed model-based 3D engineering), the S12 default.
    #[default]
    Ap242,
}

impl StepSchema {
    /// The shim's schema code.
    pub(crate) fn code(self) -> u8 {
        match self {
            StepSchema::Ap203 => 0,
            StepSchema::Ap214 => 1,
            StepSchema::Ap242 => 2,
        }
    }
}

/// One node as the shim reads or writes it (pre-order, parents first).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FlatNode {
    pub parent: Option<u32>,
    pub name: Option<String>,
    pub transform: Transform,
    pub color: Option<[f32; 4]>,
    pub proto: Option<u32>,
}

/// One prototype's record (the shape travels separately).
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct FlatProto {
    pub name: Option<String>,
    pub color: Option<[f32; 4]>,
}

/// A whole model as the shim reads or writes it.
#[derive(Clone, Debug, Default)]
pub(crate) struct FlatModel {
    pub nodes: Vec<FlatNode>,
    pub protos: Vec<FlatProto>,
    pub shapes: Vec<Shape>,
    /// `(prototype, face, colour)`.
    pub face_colors: Vec<(u32, u32, [f32; 4])>,
}

fn invalid(what: &'static str, reason: impl Into<String>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: Vec::new(),
    }
}

fn index_u32(i: usize) -> KResult<u32> {
    u32::try_from(i).map_err(|_| invalid("model", "too many nodes"))
}

/// Lowers `model` for a write: its prototype table, then one new prototype per node with a shape
/// and no `instance_of`; nodes in pre-order.
pub(crate) fn flatten(model: &ForeignModel) -> KResult<FlatModel> {
    let mut flat = FlatModel::default();
    for (p, proto) in model.prototypes.iter().enumerate() {
        let p = index_u32(p)?;
        flat.protos.push(FlatProto {
            name: proto.name.clone(),
            color: proto.color,
        });
        push_face_colors(&mut flat, p, &proto.shape, &proto.face_colors)?;
        flat.shapes.push(proto.shape.clone());
    }
    for root in &model.roots {
        flatten_node(root, None, model.prototypes.len(), &mut flat)?;
    }
    Ok(flat)
}

fn push_face_colors(
    flat: &mut FlatModel,
    proto: u32,
    shape: &Shape,
    colors: &[(FaceId, [f32; 4])],
) -> KResult<()> {
    let faces = shape.count(SubKind::Face);
    for &(face, color) in colors {
        if face.0 >= faces {
            return Err(invalid(
                "model",
                format!(
                    "face colour on face {} of a shape with {faces} faces",
                    face.0
                ),
            ));
        }
        check_color(color)?;
        flat.face_colors.push((proto, face.0, color));
    }
    Ok(())
}

fn check_color(color: [f32; 4]) -> KResult<()> {
    if color
        .iter()
        .all(|c| c.is_finite() && (0.0..=1.0).contains(c))
    {
        Ok(())
    } else {
        Err(invalid(
            "model",
            format!("colour {color:?} is outside [0, 1]"),
        ))
    }
}

fn flatten_node(
    node: &ForeignNode,
    parent: Option<u32>,
    table: usize,
    flat: &mut FlatModel,
) -> KResult<()> {
    if !is_rigid(&node.transform) {
        return Err(invalid(
            "model",
            format!("node {:?} has a transform that is not rigid", node.name),
        ));
    }
    if let Some(color) = node.color {
        check_color(color)?;
    }
    let proto = match (&node.shape, node.instance_of) {
        (_, Some(p)) => {
            if p as usize >= table {
                return Err(invalid(
                    "model",
                    format!(
                        "node {:?} names prototype {p}, which does not exist",
                        node.name
                    ),
                ));
            }
            Some(p)
        }
        (Some(shape), None) => {
            if shape.is_null() {
                return Err(invalid(
                    "model",
                    format!("node {:?} has a null shape", node.name),
                ));
            }
            let p = index_u32(flat.shapes.len())?;
            flat.protos.push(FlatProto {
                name: node.name.clone(),
                color: node.color,
            });
            push_face_colors(flat, p, shape, &node.face_colors)?;
            flat.shapes.push(shape.clone());
            Some(p)
        }
        (None, None) => None,
    };
    if proto.is_some() && !node.children.is_empty() {
        return Err(invalid(
            "model",
            format!(
                "node {:?} has both a shape and children; put the shape in a child",
                node.name
            ),
        ));
    }
    let me = index_u32(flat.nodes.len())?;
    flat.nodes.push(FlatNode {
        parent,
        name: node.name.clone(),
        transform: node.transform,
        color: node.color,
        proto,
    });
    for child in &node.children {
        flatten_node(child, Some(me), table, flat)?;
    }
    Ok(())
}

/// Builds the model tree of a read. `protos` and `face_colors` describe the (possibly healed)
/// prototype shapes in `flat.shapes`.
pub(crate) fn unflatten(
    flat: FlatModel,
    source_unit_mm: f64,
    warnings: Vec<String>,
) -> ForeignModel {
    let mut prototypes: Vec<ForeignPrototype> = flat
        .protos
        .iter()
        .zip(&flat.shapes)
        .map(|(p, shape)| ForeignPrototype {
            name: p.name.clone(),
            shape: shape.clone(),
            color: p.color,
            face_colors: Vec::new(),
        })
        .collect();
    for &(p, face, color) in &flat.face_colors {
        if let Some(proto) = prototypes.get_mut(p as usize) {
            proto.face_colors.push((FaceId(face), color));
        }
    }
    let mut children: Vec<Vec<usize>> = vec![Vec::new(); flat.nodes.len()];
    let mut roots = Vec::new();
    for (i, node) in flat.nodes.iter().enumerate() {
        match node.parent {
            Some(p) if (p as usize) < i => children[p as usize].push(i),
            _ => roots.push(i),
        }
    }
    fn build(
        i: usize,
        flat: &FlatModel,
        children: &[Vec<usize>],
        prototypes: &[ForeignPrototype],
    ) -> ForeignNode {
        let node = &flat.nodes[i];
        let proto = node.proto.and_then(|p| prototypes.get(p as usize));
        ForeignNode {
            name: node.name.clone(),
            transform: node.transform,
            color: node.color,
            shape: proto.map(|p| p.shape.clone()),
            face_colors: proto.map(|p| p.face_colors.clone()).unwrap_or_default(),
            children: children[i]
                .iter()
                .map(|&c| build(c, flat, children, prototypes))
                .collect(),
            instance_of: proto.and(node.proto),
        }
    }
    let roots = roots
        .into_iter()
        .map(|r| build(r, &flat, &children, &prototypes))
        .collect();
    ForeignModel {
        roots,
        prototypes,
        source_unit_mm,
        warnings,
    }
}

/// The length in mm of a STEP length unit name as `STEPControl_Reader::FileUnits` reports it
/// (`MILLIMETRE`, `INCH`, ...), ignoring case, spaces, dots and underscores.
pub(crate) fn step_unit_mm(name: &str) -> Option<f64> {
    let key: String = name
        .chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .map(|c| c.to_ascii_uppercase())
        .collect();
    let mm = match key.as_str() {
        "MM" | "MILLIMETRE" | "MILLIMETER" => 1.0,
        "CM" | "CENTIMETRE" | "CENTIMETER" => 10.0,
        "M" | "METRE" | "METER" => 1000.0,
        "KM" | "KILOMETRE" | "KILOMETER" => 1.0e6,
        "UM" | "MICRON" | "MICROMETRE" | "MICROMETER" => 1.0e-3,
        "IN" | "INCH" | "INCHES" => 25.4,
        "FT" | "FOOT" | "FEET" => 304.8,
        "MIL" | "MILS" => 0.0254,
        "MI" | "MILE" | "MILES" => 1_609_344.0,
        "UIN" | "MICROINCH" => 2.54e-5,
        _ => return None,
    };
    Some(mm)
}

/// The length in mm of an IGES global-section unit flag (IGES 5.3 §2.2.4.3.15).
pub(crate) fn iges_unit_mm(flag: i32) -> Option<f64> {
    let mm = match flag {
        1 => 25.4,
        2 => 1.0,
        4 => 304.8,
        5 => 1_609_344.0,
        6 => 1000.0,
        7 => 1.0e6,
        8 => 0.0254,
        9 => 1.0e-3,
        10 => 10.0,
        11 => 2.54e-5,
        _ => return None,
    };
    Some(mm)
}

/// The declared unit of a STEP file from its length unit names: the first one, with a warning
/// when it is unknown (mm assumed) or when the file declares several different ones.
pub(crate) fn step_source_unit(names: &[String], warnings: &mut Vec<String>) -> f64 {
    let Some(first) = names.first() else {
        return 1.0;
    };
    let unit = step_unit_mm(first).unwrap_or_else(|| {
        warnings.push(format!("unknown STEP length unit `{first}`; mm assumed"));
        1.0
    });
    let others: Vec<&String> = names
        .iter()
        .filter(|n| step_unit_mm(n).is_none_or(|mm| (mm - unit).abs() > 1e-12 * unit))
        .filter(|n| *n != first)
        .collect();
    if !others.is_empty() {
        warnings.push(format!(
            "the file declares several length units ({first}, {}); the first is reported",
            others
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    unit
}

/// The unit a write uses for `source_unit_mm`: a unit the writers know, else mm.
pub(crate) fn write_unit_mm(source_unit_mm: f64) -> f64 {
    const KNOWN: [f64; 6] = [1.0, 10.0, 1000.0, 25.4, 304.8, 1.0e-3];
    KNOWN
        .into_iter()
        .find(|&mm| (source_unit_mm - mm).abs() <= 1e-12 * mm)
        .unwrap_or(1.0)
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "the unit tables are exact")]
mod tests {
    use super::*;

    #[test]
    fn step_unit_names_map_to_mm() {
        assert_eq!(step_unit_mm("MILLIMETRE"), Some(1.0));
        assert_eq!(step_unit_mm("milli metre"), Some(1.0));
        assert_eq!(step_unit_mm("INCH"), Some(25.4));
        assert_eq!(step_unit_mm("METRE"), Some(1000.0));
        assert_eq!(step_unit_mm("FOOT"), Some(304.8));
        assert_eq!(step_unit_mm("CENTIMETRE"), Some(10.0));
        assert_eq!(step_unit_mm("furlong"), None);
    }

    #[test]
    fn iges_unit_flags_map_to_mm() {
        assert_eq!(iges_unit_mm(2), Some(1.0));
        assert_eq!(iges_unit_mm(1), Some(25.4));
        assert_eq!(iges_unit_mm(6), Some(1000.0));
        assert_eq!(iges_unit_mm(3), None);
        assert_eq!(iges_unit_mm(0), None);
    }

    #[test]
    fn source_unit_warns_on_unknown_and_mixed_units() {
        let mut warnings = Vec::new();
        assert_eq!(step_source_unit(&[], &mut warnings), 1.0);
        assert_eq!(step_source_unit(&["INCH".into()], &mut warnings), 25.4);
        assert!(warnings.is_empty());
        assert_eq!(step_source_unit(&["PARSEC".into()], &mut warnings), 1.0);
        assert_eq!(warnings.len(), 1);
        assert_eq!(
            step_source_unit(&["INCH".into(), "MILLIMETRE".into()], &mut warnings),
            25.4
        );
        assert_eq!(warnings.len(), 2);
        assert_eq!(
            step_source_unit(&["INCH".into(), "INCH".into()], &mut warnings),
            25.4
        );
        assert_eq!(warnings.len(), 2);
    }

    #[test]
    fn write_unit_falls_back_to_mm() {
        assert_eq!(write_unit_mm(25.4), 25.4);
        assert_eq!(write_unit_mm(1000.0), 1000.0);
        assert_eq!(write_unit_mm(0.0254), 1.0);
        assert_eq!(write_unit_mm(f64::NAN), 1.0);
    }

    #[test]
    fn defaults() {
        assert_eq!(
            ImportOpts::default(),
            ImportOpts {
                heal: true,
                read_colors: true,
                read_names: true
            }
        );
        assert_eq!(StepSchema::default(), StepSchema::Ap242);
        assert_eq!(ForeignModel::default().source_unit_mm, 1.0);
    }

    #[test]
    fn empty_flat_model_gives_an_empty_tree() {
        let model = unflatten(FlatModel::default(), 1.0, vec!["w".into()]);
        assert!(model.roots.is_empty());
        assert!(model.prototypes.is_empty());
        assert_eq!(model.warnings, vec!["w".to_owned()]);
    }

    #[test]
    fn assemblies_without_shapes_flatten_in_pre_order() {
        let model = ForeignModel::new(vec![ForeignNode::assembly(
            Some("top"),
            vec![
                ForeignNode::assembly(Some("a"), Vec::new(), Transform::IDENTITY),
                ForeignNode::assembly(
                    Some("b"),
                    vec![ForeignNode::assembly(
                        Some("c"),
                        Vec::new(),
                        Transform::IDENTITY,
                    )],
                    Transform::from_translation(amg_core::Vec3::new(1.0, 0.0, 0.0)),
                ),
            ],
            Transform::IDENTITY,
        )]);
        let flat = flatten(&model).expect("flatten");
        let names: Vec<_> = flat.nodes.iter().map(|n| n.name.as_deref()).collect();
        assert_eq!(names, [Some("top"), Some("a"), Some("b"), Some("c")]);
        let parents: Vec<_> = flat.nodes.iter().map(|n| n.parent).collect();
        assert_eq!(parents, [None, Some(0), Some(0), Some(2)]);
        let back = unflatten(flat, 1.0, Vec::new());
        assert_eq!(back.nodes().len(), 4);
        assert_eq!(
            back.roots[0].children[1].children[0].name.as_deref(),
            Some("c")
        );
    }

    #[test]
    fn bad_nodes_are_invalid_input() {
        let scaled = ForeignModel::new(vec![ForeignNode::assembly(
            None,
            Vec::new(),
            Transform::from_scale(amg_core::Vec3::splat(2.0)),
        )]);
        assert!(matches!(
            flatten(&scaled),
            Err(KernelError::InvalidInput { what: "model", .. })
        ));
        let mut missing = ForeignNode::assembly(None, Vec::new(), Transform::IDENTITY);
        missing.instance_of = Some(0);
        assert!(matches!(
            flatten(&ForeignModel::new(vec![missing])),
            Err(KernelError::InvalidInput { what: "model", .. })
        ));
        let mut red = ForeignNode::assembly(None, Vec::new(), Transform::IDENTITY);
        red.color = Some([2.0, 0.0, 0.0, 1.0]);
        assert!(matches!(
            flatten(&ForeignModel::new(vec![red])),
            Err(KernelError::InvalidInput { what: "model", .. })
        ));
    }
}
