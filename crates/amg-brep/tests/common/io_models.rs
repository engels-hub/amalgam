//! The STEP and IGES fixture models (BREP-13), built with the kernel. Shared by `tests/io.rs`
//! and the `gen_brep_io` generator, which writes them to `testdata/brep/{step,iges}/`.
#![allow(
    dead_code,
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test code"
)]

use std::f64::consts::FRAC_PI_2;
use std::path::{Path, PathBuf};

use amg_brep::{
    BrepKernel, FaceId, ForeignModel, ForeignNode, ForeignPrototype, KernelConfig, OcctKernel,
    Plane, Shape, StepSchema, SubKind, Transform, Validate, Vec3,
};
use amg_core::Mat3;

/// STEP fixtures: `testdata/brep/step/<name>.step`, with the schema they are written in.
pub const STEP_FIXTURES: [(&str, StepSchema); 10] = [
    ("box_ap242", StepSchema::Ap242),
    ("cylinder_ap203", StepSchema::Ap203),
    ("coloured_box_ap214", StepSchema::Ap214),
    ("inch_block", StepSchema::Ap214),
    ("bolt_pattern", StepSchema::Ap242),
    ("nested_assembly", StepSchema::Ap214),
    ("two_roots", StepSchema::Ap242),
    ("filleted_box", StepSchema::Ap242),
    ("plate_with_holes", StepSchema::Ap214),
    ("open_shapes", StepSchema::Ap242),
];

/// IGES fixtures: `testdata/brep/iges/<name>.igs`.
pub const IGES_FIXTURES: [&str; 3] = ["box_iges", "coloured_cylinder", "filleted_box_iges"];

pub const RED: [f32; 4] = [0.8, 0.1, 0.1, 1.0];
pub const BLUE: [f32; 4] = [0.1, 0.2, 0.9, 1.0];
pub const GREEN: [f32; 4] = [0.1, 0.7, 0.2, 1.0];
pub const GREY: [f32; 4] = [0.5, 0.5, 0.5, 1.0];
pub const YELLOW: [f32; 4] = [0.9, 0.8, 0.1, 1.0];

pub fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Always,
        ..KernelConfig::default()
    })
}

pub fn repo_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

pub fn step_path(name: &str) -> PathBuf {
    repo_dir().join(format!("testdata/brep/step/{name}.step"))
}

pub fn iges_path(name: &str) -> PathBuf {
    repo_dir().join(format!("testdata/brep/iges/{name}.igs"))
}

/// A `testdata/brep/<name>.brep` fixture.
pub fn brep_fixture(name: &str) -> Shape {
    let path = repo_dir().join(format!("testdata/brep/{name}.brep"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    Shape::read_text(std::io::BufReader::new(file)).expect("read fixture")
}

fn at(x: f64, y: f64, z: f64) -> Transform {
    Transform::from_translation(Vec3::new(x, y, z))
}

fn bx(k: &OcctKernel, x: f64, y: f64, z: f64) -> Shape {
    k.make_box(&Plane::XY, Vec3::new(x, y, z)).unwrap().shape
}

fn named(mut node: ForeignNode, color: Option<[f32; 4]>) -> ForeignNode {
    node.color = color;
    node
}

fn proto(name: &str, shape: Shape, color: Option<[f32; 4]>) -> ForeignPrototype {
    ForeignPrototype {
        name: Some(name.to_owned()),
        shape,
        color,
        face_colors: Vec::new(),
    }
}

/// An instance of prototype `p` of `model`.
fn instance(model: &ForeignModel, p: u32, name: &str, transform: Transform) -> ForeignNode {
    let proto = &model.prototypes[p as usize];
    ForeignNode {
        name: Some(name.to_owned()),
        transform,
        color: proto.color,
        shape: Some(proto.shape.clone()),
        face_colors: proto.face_colors.clone(),
        children: Vec::new(),
        instance_of: Some(p),
    }
}

/// The STEP fixture model `name`.
pub fn step_model(k: &OcctKernel, name: &str) -> ForeignModel {
    match name {
        "box_ap242" => ForeignModel::new(vec![ForeignNode::part(
            Some("box"),
            bx(k, 10.0, 20.0, 30.0),
            Transform::IDENTITY,
        )]),
        "cylinder_ap203" => ForeignModel::new(vec![ForeignNode::part(
            Some("cylinder"),
            k.make_cylinder(&Plane::XY, 5.0, 12.0, std::f64::consts::TAU)
                .unwrap()
                .shape,
            Transform::IDENTITY,
        )]),
        "coloured_box_ap214" => {
            let mut node = named(
                ForeignNode::part(
                    Some("coloured_box"),
                    bx(k, 20.0, 20.0, 10.0),
                    Transform::IDENTITY,
                ),
                Some(RED),
            );
            node.face_colors = vec![(FaceId(0), BLUE), (FaceId(3), GREEN)];
            ForeignModel::new(vec![node])
        }
        "inch_block" => {
            // 1 × 2 × 3 inches, written in inches.
            let mut model = ForeignModel::new(vec![ForeignNode::part(
                Some("inch_block"),
                bx(k, 25.4, 50.8, 76.2),
                Transform::IDENTITY,
            )]);
            model.source_unit_mm = 25.4;
            model
        }
        "bolt_pattern" => {
            let bolt = k
                .make_cylinder(&Plane::XY, 2.0, 15.0, std::f64::consts::TAU)
                .unwrap()
                .shape;
            let mut model = ForeignModel {
                prototypes: vec![
                    proto("bolt", bolt, None),
                    proto("base", bx(k, 50.0, 50.0, 5.0), Some(GREY)),
                ],
                ..ForeignModel::default()
            };
            let mut children = vec![instance(&model, 1, "base", at(-25.0, -25.0, -5.0))];
            for (i, (x, y)) in [(-20.0, -20.0), (20.0, -20.0), (20.0, 20.0), (-20.0, 20.0)]
                .into_iter()
                .enumerate()
            {
                let mut bolt = instance(&model, 0, &format!("bolt_{}", i + 1), at(x, y, 0.0));
                if i == 2 {
                    bolt.color = Some(YELLOW);
                }
                children.push(bolt);
            }
            model.roots = vec![ForeignNode::assembly(
                Some("bolt_pattern"),
                children,
                Transform::IDENTITY,
            )];
            model
        }
        "nested_assembly" => {
            let link = bx(k, 40.0, 6.0, 6.0);
            let joint = k.make_sphere(&Plane::XY, 5.0).unwrap().shape;
            let mut model = ForeignModel {
                prototypes: vec![
                    proto("link", link, Some(BLUE)),
                    proto("joint", joint, Some(RED)),
                ],
                ..ForeignModel::default()
            };
            let arm = |model: &ForeignModel, name: &str, t: Transform| {
                ForeignNode::assembly(
                    Some(name),
                    vec![
                        instance(model, 0, "link", at(0.0, -3.0, -3.0)),
                        instance(model, 1, "joint", Transform::IDENTITY),
                    ],
                    t,
                )
            };
            let turn = Transform::from_mat3_translation(
                Mat3::from_rotation_z(FRAC_PI_2),
                Vec3::new(0.0, 0.0, 30.0),
            );
            let left = arm(&model, "arm_left", Transform::IDENTITY);
            let right = arm(&model, "arm_right", turn);
            model.roots = vec![ForeignNode::assembly(
                Some("robot"),
                vec![left, right],
                Transform::IDENTITY,
            )];
            model
        }
        "two_roots" => ForeignModel::new(vec![
            named(
                ForeignNode::part(
                    Some("ball"),
                    k.make_sphere(&Plane::XY, 8.0).unwrap().shape,
                    Transform::IDENTITY,
                ),
                Some(GREEN),
            ),
            ForeignNode::part(
                Some("ring"),
                k.make_torus(&Plane::XY, 20.0, 3.0, std::f64::consts::TAU)
                    .unwrap()
                    .shape,
                Transform::IDENTITY,
            ),
        ]),
        "filleted_box" => {
            let shape = brep_fixture("filleted_box");
            let mut node = named(
                ForeignNode::part(Some("filleted_box"), shape, Transform::IDENTITY),
                Some(GREY),
            );
            // A fillet face: the first non-planar face.
            let f = node
                .shape
                .as_ref()
                .unwrap()
                .faces()
                .find(|&f| {
                    node.shape.as_ref().unwrap().surface(f).class()
                        != amg_brep::geom::GeomClass::Plane
                })
                .unwrap();
            node.face_colors = vec![(f, YELLOW)];
            ForeignModel::new(vec![node])
        }
        "plate_with_holes" => ForeignModel::new(vec![ForeignNode::part(
            Some("plate_with_holes"),
            brep_fixture("plate_with_holes"),
            Transform::IDENTITY,
        )]),
        "open_shapes" => ForeignModel::new(vec![
            ForeignNode::part(
                Some("bspline_patch"),
                brep_fixture("bspline_patch"),
                Transform::IDENTITY,
            ),
            ForeignNode::part(
                Some("revolved_profile"),
                brep_fixture("revolved_profile"),
                Transform::IDENTITY,
            ),
        ]),
        other => panic!("no STEP fixture {other}"),
    }
}

/// The IGES fixture model `name`.
pub fn iges_model(k: &OcctKernel, name: &str) -> ForeignModel {
    match name {
        "box_iges" => ForeignModel::new(vec![ForeignNode::part(
            Some("box"),
            bx(k, 10.0, 20.0, 30.0),
            Transform::IDENTITY,
        )]),
        "coloured_cylinder" => ForeignModel::new(vec![named(
            ForeignNode::part(
                Some("cylinder"),
                k.make_cylinder(&Plane::XY, 6.0, 10.0, std::f64::consts::TAU)
                    .unwrap()
                    .shape,
                Transform::IDENTITY,
            ),
            Some(BLUE),
        )]),
        "filleted_box_iges" => ForeignModel::new(vec![ForeignNode::part(
            Some("filleted_box"),
            brep_fixture("filleted_box"),
            Transform::IDENTITY,
        )]),
        other => panic!("no IGES fixture {other}"),
    }
}

/// Whether `path` is an un-fetched git-LFS pointer (`*.step` is LFS-tracked).
pub fn is_lfs_pointer(path: &Path) -> bool {
    std::fs::read(path)
        .map(|bytes| bytes.starts_with(b"version https://git-lfs"))
        .unwrap_or(false)
}

pub fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

/// Volumes of each solid of `shape`, in index order.
pub fn solid_volumes(shape: &Shape) -> Vec<f64> {
    shape
        .solids()
        .iter()
        .map(|s| s.volume_adaptive(1e-12).unwrap())
        .collect()
}

pub fn area(shape: &Shape) -> f64 {
    shape.faces().map(|f| shape.face_area(f)).sum()
}

/// Shape facts compared across a round trip.
pub fn shape_facts(shape: &Shape) -> (usize, u32, Vec<f64>, f64) {
    (
        shape.solids().len(),
        shape.count(SubKind::Face),
        solid_volumes(shape),
        area(shape),
    )
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * a.abs().max(b.abs()).max(1e-300)
}

fn transforms_close(a: &Transform, b: &Transform) -> bool {
    let (a, b) = (a.to_cols_array(), b.to_cols_array());
    a.iter().zip(b).all(|(x, y)| (x - y).abs() <= 1e-9)
}

fn colors_close(a: Option<[f32; 4]>, b: Option<[f32; 4]>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => a.iter().zip(b).all(|(x, y)| (x - y).abs() <= 1e-4),
        _ => false,
    }
}

/// What a comparison checks.
#[derive(Clone, Copy, Debug)]
pub struct Same {
    /// Relative tolerance of solid volumes and areas.
    pub rel: f64,
    pub names: bool,
    pub colors: bool,
    pub face_colors: bool,
    /// Compare `instance_of` (absent on models built in code).
    pub instances: bool,
}

pub const STRICT: Same = Same {
    rel: 1e-9,
    names: true,
    colors: true,
    face_colors: true,
    instances: true,
};

/// Asserts that `b` has the tree of `a`: structure, names, transforms, colours, prototype
/// sharing (a consistent bijection of `instance_of`), and per part the solid count, face count,
/// per-solid volume and area.
pub fn assert_same_model(a: &ForeignModel, b: &ForeignModel, same: Same, what: &str) {
    assert_eq!(a.roots.len(), b.roots.len(), "{what}: root count");
    assert!(
        close(a.source_unit_mm, b.source_unit_mm, 1e-12),
        "{what}: unit {} vs {}",
        a.source_unit_mm,
        b.source_unit_mm
    );
    let mut protos = std::collections::BTreeMap::new();
    for (i, (x, y)) in a.roots.iter().zip(&b.roots).enumerate() {
        same_node(x, y, same, &format!("{what}/root{i}"), &mut protos);
    }
    let mut targets: Vec<u32> = protos.values().copied().collect();
    targets.sort_unstable();
    targets.dedup();
    assert_eq!(
        targets.len(),
        protos.len(),
        "{what}: prototype sharing differs"
    );
}

fn same_node(
    a: &ForeignNode,
    b: &ForeignNode,
    same: Same,
    what: &str,
    protos: &mut std::collections::BTreeMap<u32, u32>,
) {
    if same.names {
        assert_eq!(a.name, b.name, "{what}: name");
    }
    assert!(
        transforms_close(&a.transform, &b.transform),
        "{what}: transform {:?} vs {:?}",
        a.transform,
        b.transform
    );
    if same.colors {
        assert!(
            colors_close(a.color, b.color),
            "{what}: colour {:?} vs {:?}",
            a.color,
            b.color
        );
    }
    assert_eq!(
        a.shape.is_some(),
        b.shape.is_some(),
        "{what}: shape presence"
    );
    if let (Some(sa), Some(sb)) = (&a.shape, &b.shape) {
        let (fa, fb) = (shape_facts(sa), shape_facts(sb));
        assert_eq!(fa.0, fb.0, "{what}: solid count");
        assert_eq!(fa.1, fb.1, "{what}: face count");
        for (va, vb) in fa.2.iter().zip(&fb.2) {
            assert!(close(*va, *vb, same.rel), "{what}: volume {va} vs {vb}");
        }
        assert!(
            close(fa.3, fb.3, same.rel),
            "{what}: area {} vs {}",
            fa.3,
            fb.3
        );
        if same.face_colors {
            let mut ca = a.face_colors.clone();
            let mut cb = b.face_colors.clone();
            ca.sort_by_key(|c| c.0);
            cb.sort_by_key(|c| c.0);
            assert_eq!(ca.len(), cb.len(), "{what}: face colours {ca:?} vs {cb:?}");
            for (x, y) in ca.iter().zip(&cb) {
                assert_eq!(x.0, y.0, "{what}: coloured face {ca:?} vs {cb:?}");
                assert!(colors_close(Some(x.1), Some(y.1)), "{what}: face colour");
            }
        }
    }
    match (a.instance_of, b.instance_of) {
        _ if !same.instances => {}
        (Some(x), Some(y)) => {
            let previous = protos.insert(x, y);
            assert!(
                previous.is_none_or(|p| p == y),
                "{what}: prototype {x} maps to {previous:?} and {y}"
            );
        }
        (None, None) => {}
        (x, y) => panic!("{what}: instance_of {x:?} vs {y:?}"),
    }
    assert_eq!(a.children.len(), b.children.len(), "{what}: child count");
    for (i, (x, y)) in a.children.iter().zip(&b.children).enumerate() {
        same_node(x, y, same, &format!("{what}/{i}"), protos);
    }
}
