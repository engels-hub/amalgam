//! MESH-26: the Phase 1 mesh op features (remesh, reduce, smooth, plane cut, quick / robust
//! shell, separate, erase & fill) in a real `Document`: undo / redo, parameter edits that
//! recompute the feature and its dependents only, serialisation, health and goldens.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, OnceLock};

use amg_core::Point3;
use amg_doc::{
    BodyGeom, BodyId, Command, CommandOutcome, ConstructionId, CreatedId, Dependency, Document,
    Expr, ExprSrc, Feature, FeatureBox, FeatureError, FeatureId, FeatureOutput, Health, ParamId,
    RecomputeCtx, SaveOptions, load, save,
};
use amg_features::construct::{ConstructPlane, OriginEntity, PlaneDef, RefInput};
use amg_features::mesh::{
    CutPlane, FairingParam, KeepSideParam, MeshEntityRef, MeshEraseFillFeature,
    MeshPlaneCutFeature, MeshReduceFeature, MeshRemeshFeature, MeshReverseNormalFeature,
    MeshSeparateFeature, MeshShellFeature, MeshSmoothFeature, MeshWrapFeature, OriginPlane,
    OutputMode, ReduceTarget, SmoothMethodParam,
};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
use amg_mesh::{FaceGroupInfo, FaceId, GroupOrigin, IndexedMesh, ShellSide};
use amg_testkit::{TriSoup, hausdorff, shapes};
use serde::{Deserialize, Serialize};

// ----------------------------------------------------------------------------- fixtures

/// Test-only mesh source (stands in for the S12 importers): creates body slot 0.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct OpsMeshSource {
    mesh: IndexedMesh,
}

impl Feature for OpsMeshSource {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![amg_doc::BodyChange {
                id: ctx.new_body_id(0),
                change: amg_doc::BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(self.mesh.clone())),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(OpsMeshSource, "test.mesh_ops_source", version = 1);

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn link() {
    amg_features::mesh::link();
    amg_features::construct::link();
}

fn inline(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

fn body(origin: FeatureId) -> BodyId {
    BodyId { origin, slot: 0 }
}

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

fn watertight(m: &IndexedMesh) -> bool {
    amg_mesh::analyze(m, &Default::default(), &JobCtx::detached())
        .unwrap()
        .is_watertight()
}

fn icosphere(radius: f64, sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(Point3::ZERO, radius, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

/// Outward torus, `nu` x `nv` quads split in two.
fn torus(nu: u32, nv: u32, major: f64, minor: f64) -> IndexedMesh {
    let mut p = Vec::new();
    for i in 0..nu {
        let u = f64::from(i) / f64::from(nu) * std::f64::consts::TAU;
        for j in 0..nv {
            let v = f64::from(j) / f64::from(nv) * std::f64::consts::TAU;
            let r = major + minor * v.cos();
            p.push(Point3::new(r * u.cos(), r * u.sin(), minor * v.sin()));
        }
    }
    let id = |i: u32, j: u32| (i % nu) * nv + (j % nv);
    let mut t = Vec::new();
    for i in 0..nu {
        for j in 0..nv {
            t.push([id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
            t.push([id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
        }
    }
    let m = IndexedMesh::new(p, t).unwrap();
    assert!(m.signed_volume() > 0.0);
    m
}

/// An icosphere with every vertex moved radially by a deterministic pseudo-random amount up
/// to `amp` mm.
fn noisy_sphere(radius: f64, sub: u32, amp: f64) -> IndexedMesh {
    let mut m = icosphere(radius, sub);
    let mut s = 0x9e37_79b9_7f4a_7c15u64;
    for p in &mut m.positions {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        #[allow(clippy::cast_precision_loss)]
        let r = (s >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0;
        *p *= 1.0 + amp * r / radius;
    }
    m
}

/// Spheres of radius 6, 4 and 2 side by side, as one mesh with three shells.
fn three_spheres() -> IndexedMesh {
    let mut positions = Vec::new();
    let mut tris = Vec::new();
    for (r, x) in [(6.0, 0.0), (4.0, 15.0), (2.0, 25.0)] {
        let s = shapes::icosphere(Point3::new(x, 0.0, 0.0), r, 3);
        let base = positions.len() as u32;
        positions.extend(s.positions);
        tris.extend(s.triangles.iter().map(|t| t.map(|i| i + base)));
    }
    IndexedMesh::new(positions, tris).unwrap()
}

/// Two 6 mm cubes 1 mm apart (an outward 1 mm quick shell makes them cross).
fn two_cubes() -> IndexedMesh {
    let mut positions = Vec::new();
    let mut tris = Vec::new();
    for x in [0.0, 7.0] {
        let s = shapes::cube(Point3::new(x, 0.0, 0.0), 6.0);
        let base = positions.len() as u32;
        positions.extend(s.positions);
        tris.extend(s.triangles.iter().map(|t| t.map(|i| i + base)));
    }
    IndexedMesh::new(positions, tris).unwrap()
}

/// An icosphere (radius 10, subdivision 4) whose faces above z = 7 are one `User` group.
fn sphere_with_cap() -> (IndexedMesh, amg_mesh::FaceGroupId) {
    let mut m = icosphere(10.0, 4);
    let cap = m.groups.alloc(FaceGroupInfo::new(GroupOrigin::User));
    for f in 0..m.face_count() {
        let id = FaceId(f as u32);
        let c = m.triangle(id).iter().map(|p| p.z).sum::<f64>() / 3.0;
        if c > 7.0 {
            m.groups.set(id, cap);
        }
    }
    (m, cap)
}

fn add_feature(doc: &mut Document, f: FeatureBox) -> FeatureId {
    let root = doc.design().root();
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context: root,
            name: None,
        })
        .unwrap();
    match out.created[0] {
        CreatedId::Feature(id) => id,
        ref other => panic!("{other:?}"),
    }
}

fn created_param(out: CommandOutcome) -> ParamId {
    match out.created[0] {
        CreatedId::Param(id) => id,
        ref other => panic!("{other:?}"),
    }
}

fn source(m: &IndexedMesh) -> FeatureBox {
    FeatureBox::new(OpsMeshSource { mesh: m.clone() })
}

fn reverse(b: BodyId) -> FeatureBox {
    FeatureBox::new(MeshReverseNormalFeature {
        body: b,
        groups: None,
    })
}

fn recompute(doc: &mut Document) {
    doc.recompute_blocking(jobs()).unwrap();
}

fn mesh(doc: &Document, id: BodyId) -> Arc<IndexedMesh> {
    match &doc.evaluated().body(id).unwrap().geom {
        BodyGeom::Mesh(m) => Arc::clone(m),
        other => panic!("{other:?}"),
    }
}

fn try_mesh(doc: &Document, id: BodyId) -> Option<Arc<IndexedMesh>> {
    match &doc.evaluated().body(id)?.geom {
        BodyGeom::Mesh(m) => Some(Arc::clone(m)),
        _ => None,
    }
}

fn health(doc: &Document, id: FeatureId) -> Health {
    doc.evaluated().node(id).unwrap().health.clone()
}

fn computed(doc: &Document) -> Vec<FeatureId> {
    doc.last_run().unwrap().computed.clone()
}

fn cached(doc: &Document) -> Vec<FeatureId> {
    doc.last_run().unwrap().cached.clone()
}

/// The model parameter behind the first expression of feature `id`.
fn first_param(doc: &Document, id: FeatureId) -> ParamId {
    let (_, node) = doc.design().timeline().node(id).unwrap();
    match node.feature.get().exprs().first() {
        Some(Expr::Param(p)) => *p,
        other => panic!("inline expressions should have become parameters: {other:?}"),
    }
}

struct TempFile(PathBuf);

impl TempFile {
    fn new() -> TempFile {
        static N: AtomicU32 = AtomicU32::new(0);
        TempFile(std::env::temp_dir().join(format!(
            "amg-features-mesh26-{}-{}.amg",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        )))
    }
}

impl Drop for TempFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
        let _ = std::fs::remove_file(self.0.with_extension("amg.bak"));
    }
}

// ----------------------------------------------------------------------------- op cases

/// How a case changes its feature after the first compute.
enum Edit {
    /// New source for the feature's first expression parameter.
    Param(&'static str),
    /// Replace the feature.
    Feature(FeatureBox),
}

struct Case {
    name: &'static str,
    input: IndexedMesh,
    feature: fn(BodyId) -> FeatureBox,
    edit: fn(BodyId) -> Edit,
    /// The op's own checks on (input, result body, extra body slot 0 of the op if any).
    check: fn(&IndexedMesh, &IndexedMesh, Option<&IndexedMesh>),
}

fn remesh_case() -> Case {
    Case {
        name: "remesh",
        input: icosphere(10.0, 4),
        feature: |b| FeatureBox::new(MeshRemeshFeature::new(b, "1.5 mm")),
        edit: |_| Edit::Param("1.2 mm"),
        check: |input, out, _| {
            assert!(watertight(out));
            let mean = mean_edge(out);
            assert!((mean - 1.5).abs() < 0.25, "mean edge {mean}");
            let h = hausdorff(soup(input), soup(out), 20_000);
            assert!(h < 0.1, "hausdorff to input {h}");
            assert!(out.face_count() < input.face_count());
        },
    }
}

fn reduce_case() -> Case {
    Case {
        name: "reduce",
        input: icosphere(10.0, 4),
        feature: |b| FeatureBox::new(MeshReduceFeature::ratio(b, "0.25")),
        edit: |_| Edit::Param("0.5"),
        check: |input, out, _| {
            assert!(watertight(out));
            let want = input.face_count() / 4;
            assert!(out.face_count().abs_diff(want) <= 2, "{}", out.face_count());
            let h = hausdorff(soup(input), soup(out), 20_000);
            assert!(h < 0.15, "hausdorff to input {h}");
        },
    }
}

fn smooth_case() -> Case {
    Case {
        name: "smooth",
        input: noisy_sphere(10.0, 4, 0.2),
        feature: |b| FeatureBox::new(MeshSmoothFeature::new(b)),
        edit: |_| Edit::Param("0.4"),
        check: |input, out, _| {
            assert_eq!(out.tris, input.tris, "topology unchanged");
            assert_eq!(out.groups, input.groups);
            // Closer to the ideal sphere than the noisy input, volume kept within 1 %.
            let ideal = icosphere(10.0, 4);
            let before = hausdorff(soup(&ideal), soup(input), 20_000);
            let after = hausdorff(soup(&ideal), soup(out), 20_000);
            assert!(after < 0.8 * before, "{after} vs {before}");
            let dv = (out.signed_volume() - input.signed_volume()).abs() / input.signed_volume();
            assert!(dv < 0.01, "{dv}");
        },
    }
}

fn plane_cut_case() -> Case {
    Case {
        name: "plane_cut",
        input: torus(96, 48, 10.0, 4.0),
        feature: |b| {
            let mut f = MeshPlaneCutFeature::origin(b, OriginPlane::XY, "1 mm");
            f.keep = KeepSideParam::Both;
            FeatureBox::new(f)
        },
        edit: |_| Edit::Param("2 mm"),
        check: |input, pos, neg| {
            let neg = neg.expect("keep Both makes a second body");
            assert!(watertight(pos) && watertight(neg));
            let v = pos.signed_volume() + neg.signed_volume();
            assert!((v - input.signed_volume()).abs() < 1e-9 * input.signed_volume());
            assert!(pos.aabb().min.z >= 1.0 - 1e-12, "{:?}", pos.aabb());
            assert!(neg.aabb().max.z <= 1.0 + 1e-12, "{:?}", neg.aabb());
            assert!(neg.signed_volume() > pos.signed_volume());
        },
    }
}

fn quick_shell_case() -> Case {
    Case {
        name: "shell_quick",
        input: icosphere(10.0, 4),
        feature: |b| FeatureBox::new(MeshShellFeature::quick(b, "1 mm")),
        edit: |_| Edit::Param("1.5 mm"),
        check: |input, out, _| {
            assert!(watertight(out));
            // Polyhedral sphere: the shell volume is the input minus its 1 mm inner copy.
            let r: f64 = 10.0;
            let ratio = out.signed_volume() / input.signed_volume();
            let want = 1.0 - ((r - 1.0) / r).powi(3);
            assert!((ratio - want).abs() < 0.01, "{ratio} vs {want}");
        },
    }
}

fn robust_shell_case() -> Case {
    Case {
        name: "shell_robust",
        input: icosphere(10.0, 3),
        feature: |b| {
            let mut f = MeshShellFeature::robust(b, "2 mm");
            f.voxel = Some(inline("0.5 mm"));
            FeatureBox::new(f)
        },
        edit: |_| Edit::Param("2.5 mm"),
        check: |input, out, _| {
            assert!(watertight(out));
            let s = amg_mesh::MeshSpatial::build(
                Arc::new(out.clone()),
                &amg_mesh::BvhOpts::default(),
                &JobCtx::detached(),
            )
            .unwrap();
            let si = amg_mesh::self_intersections(
                &s,
                &amg_mesh::SelfIntersectOpts::default(),
                &JobCtx::detached(),
            )
            .unwrap();
            assert!(si.is_empty(), "{} SI pairs", si.len());
            let ratio = out.signed_volume() / input.signed_volume();
            let want = 1.0 - (8.0f64 / 10.0).powi(3);
            assert!((ratio - want).abs() < 0.05, "{ratio} vs {want}");
        },
    }
}

/// MESH-18: a soup of two overlapping cubes, one inside out, wrapped into one solid.
fn wrap_input() -> IndexedMesh {
    let mut positions = Vec::new();
    let mut tris = Vec::new();
    for (k, min) in [Point3::ZERO, Point3::new(3.0, 1.0, 1.0)]
        .into_iter()
        .enumerate()
    {
        let s = shapes::cube(min, 6.0);
        let base = positions.len() as u32;
        positions.extend(s.positions);
        tris.extend(s.triangles.iter().map(|t| {
            let t = t.map(|i| i + base);
            if k == 1 { [t[0], t[2], t[1]] } else { t }
        }));
    }
    IndexedMesh::new(positions, tris).unwrap()
}

fn wrap_case() -> Case {
    Case {
        name: "wrap",
        input: wrap_input(),
        feature: |b| FeatureBox::new(MeshWrapFeature::new(b, "0.5 mm")),
        edit: |_| Edit::Param("0.4 mm"),
        check: |input, out, _| {
            assert!(watertight(out));
            let s = amg_mesh::MeshSpatial::build(
                Arc::new(out.clone()),
                &amg_mesh::BvhOpts::default(),
                &JobCtx::detached(),
            )
            .unwrap();
            let si = amg_mesh::self_intersections(
                &s,
                &amg_mesh::SelfIntersectOpts::default(),
                &JobCtx::detached(),
            )
            .unwrap();
            assert!(si.is_empty(), "{} SI pairs", si.len());
            assert_eq!(amg_mesh::separate_shells(out).len(), 1);
            // Union of the cubes (216 + 216 - 3 * 5 * 5 = 357 mm³) grown by about one voxel.
            let v = out.signed_volume();
            assert!(v > 357.0 && v < 357.0 * 1.5, "volume {v}");
            let h = amg_testkit::one_sided_hausdorff(soup(out), soup(input), 20_000);
            assert!(h <= 2.0 * (0.75 + 0.5), "hausdorff to input {h}");
        },
    }
}

fn erase_fill_case() -> Case {
    let (m, _) = sphere_with_cap();
    Case {
        name: "erase_fill",
        input: m,
        feature: |b| {
            let (m, cap) = sphere_with_cap();
            FeatureBox::new(MeshEraseFillFeature {
                body: b,
                groups: vec![MeshEntityRef::new(&m, cap).unwrap()],
                fairing: FairingParam::Biharmonic,
            })
        },
        edit: |b| {
            let (m, cap) = sphere_with_cap();
            Edit::Feature(FeatureBox::new(MeshEraseFillFeature {
                body: b,
                groups: vec![MeshEntityRef::new(&m, cap).unwrap()],
                fairing: FairingParam::None,
            }))
        },
        check: |input, out, _| {
            assert!(watertight(out));
            let (_, cap) = sphere_with_cap();
            assert_eq!(out.groups.faces_of(cap).count(), 0, "cap erased");
            let fill = out
                .groups
                .ids()
                .find(|g| {
                    out.groups.info(*g).map(|i| &i.origin) == Some(&GroupOrigin::Fill)
                        && out.groups.faces_of(*g).next().is_some()
                })
                .expect("a Fill group");
            assert!(out.groups.faces_of(fill).count() > 10);
            // The patch stays near the sphere it replaces (bi-harmonic, a smooth bulge).
            let h = hausdorff(soup(input), soup(out), 20_000);
            assert!(h < 1.0, "{h}");
        },
    }
}

fn cases() -> Vec<Case> {
    vec![
        remesh_case(),
        reduce_case(),
        smooth_case(),
        plane_cut_case(),
        quick_shell_case(),
        robust_shell_case(),
        erase_fill_case(),
        wrap_case(),
    ]
}

fn mean_edge(m: &IndexedMesh) -> f64 {
    let mut sum = 0.0;
    let mut n = 0u32;
    for t in &m.tris {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if a < b {
                sum += (m.positions[a as usize] - m.positions[b as usize]).length();
                n += 1;
            }
        }
    }
    sum / f64::from(n)
}

/// The full lifecycle of one op: compute + own checks, undo / redo of the add, a parameter
/// edit that recomputes the op and its dependent only, undo / redo of the edit, JSON
/// round-trip, save / load.
fn lifecycle(case: &Case) {
    link();
    let name = case.name;
    let mut doc = Document::new(name);
    let a = add_feature(&mut doc, source(&case.input));
    recompute(&mut doc);
    let h0 = mesh(&doc, body(a)).content_hash();

    // Compute: health Ok, the op's own checks.
    let op = add_feature(&mut doc, (case.feature)(body(a)));
    recompute(&mut doc);
    assert_eq!(health(&doc, op), Health::Ok, "{name}");
    let out1 = mesh(&doc, body(a));
    let extra1 = try_mesh(&doc, body(op));
    (case.check)(&case.input, &out1, extra1.as_deref());
    let h1 = out1.content_hash();
    assert_ne!(h1, h0, "{name}: the op changed the body");

    // Undo / redo of the add restore the exact meshes.
    assert!(doc.undo());
    recompute(&mut doc);
    assert_eq!(mesh(&doc, body(a)).content_hash(), h0, "{name}: undo");
    assert!(doc.redo());
    recompute(&mut doc);
    assert_eq!(mesh(&doc, body(a)).content_hash(), h1, "{name}: redo");

    // A dependent of the op, and an independent chain.
    let rev = add_feature(&mut doc, reverse(body(a)));
    let b = add_feature(&mut doc, source(&icosphere(5.0, 2)));
    let rev_b = add_feature(&mut doc, reverse(body(b)));
    recompute(&mut doc);
    assert_eq!(computed(&doc), vec![rev, b, rev_b], "{name}");
    let after_rev = mesh(&doc, body(a)).content_hash();

    // Edit: only the op and its dependent recompute.
    match (case.edit)(body(a)) {
        Edit::Param(src) => {
            let p = first_param(&doc, op);
            doc.apply(Command::SetParamExpr {
                id: p,
                expr: src.to_owned(),
            })
            .unwrap();
        }
        Edit::Feature(f) => {
            doc.apply(Command::EditFeature { id: op, feature: f })
                .unwrap();
        }
    }
    recompute(&mut doc);
    assert_eq!(computed(&doc), vec![op, rev], "{name}: dependents only");
    assert_eq!(
        cached(&doc),
        vec![a, b, rev_b],
        "{name}: the rest is cached"
    );
    assert_eq!(health(&doc, op), Health::Ok, "{name}");
    let h2 = mesh(&doc, body(a)).content_hash();
    assert_ne!(h2, after_rev, "{name}: the edit changed the result");

    // Undo / redo of the edit.
    assert!(doc.undo());
    recompute(&mut doc);
    assert_eq!(mesh(&doc, body(a)).content_hash(), after_rev, "{name}");
    assert!(doc.redo());
    recompute(&mut doc);
    assert_eq!(mesh(&doc, body(a)).content_hash(), h2, "{name}");

    // JSON round-trip of the stored feature (expressions are parameters by now).
    let (_, node) = doc.design().timeline().node(op).unwrap();
    let f = node.feature.clone();
    let s = serde_json::to_string(&f).unwrap();
    let back: FeatureBox = serde_json::from_str(&s).unwrap();
    assert!(!back.is_unknown(), "{name}");
    assert_eq!(back, f, "{name}");
    assert_eq!(serde_json::to_string(&back).unwrap(), s, "{name}");

    // Save / load: the reopened document recomputes to the identical meshes.
    let all: Vec<(BodyId, u64)> = doc
        .evaluated()
        .bodies()
        .filter_map(|(id, b)| match &b.geom {
            BodyGeom::Mesh(m) => Some((id, m.content_hash())),
            _ => None,
        })
        .collect();
    let file = TempFile::new();
    save(&file.0, &doc.snapshot(), &SaveOptions::default()).unwrap();
    let mut reopened = load(&file.0).unwrap().into_document();
    reopened.recompute_blocking(jobs()).unwrap();
    for (id, h) in &all {
        assert_eq!(mesh(&reopened, *id).content_hash(), *h, "{name}: {id:?}");
    }
    assert_eq!(health(&reopened, op), Health::Ok, "{name}");
}

#[test]
fn remesh_lifecycle() {
    lifecycle(&remesh_case());
}

#[test]
fn reduce_lifecycle() {
    lifecycle(&reduce_case());
}

#[test]
fn smooth_lifecycle() {
    lifecycle(&smooth_case());
}

#[test]
fn plane_cut_lifecycle() {
    lifecycle(&plane_cut_case());
}

#[test]
fn quick_shell_lifecycle() {
    lifecycle(&quick_shell_case());
}

#[test]
fn robust_shell_lifecycle() {
    lifecycle(&robust_shell_case());
}

#[test]
fn erase_fill_lifecycle() {
    lifecycle(&erase_fill_case());
}

#[test]
fn wrap_lifecycle() {
    lifecycle(&wrap_case());
}

// ----------------------------------------------------------------------------- separate

#[test]
fn separate_lifecycle_and_upstream_edit_recomputes_dependents_only() {
    link();
    let input = three_spheres();
    let mut doc = Document::new("separate");
    let a = add_feature(&mut doc, source(&input));
    let red = add_feature(
        &mut doc,
        FeatureBox::new(MeshReduceFeature::ratio(body(a), "0.5")),
    );
    let sep = add_feature(
        &mut doc,
        FeatureBox::new(MeshSeparateFeature { body: body(a) }),
    );
    recompute(&mut doc);
    assert_eq!(health(&doc, sep), Health::Ok);
    let big = mesh(&doc, body(a));
    let mid = mesh(
        &doc,
        BodyId {
            origin: sep,
            slot: 0,
        },
    );
    let small = mesh(
        &doc,
        BodyId {
            origin: sep,
            slot: 1,
        },
    );
    assert!(
        try_mesh(
            &doc,
            BodyId {
                origin: sep,
                slot: 2
            }
        )
        .is_none()
    );
    for m in [&big, &mid, &small] {
        assert!(watertight(m));
    }
    assert!(big.area() > mid.area() && mid.area() > small.area());
    assert!((big.aabb().center().x - 0.0).abs() < 0.5);
    assert!((mid.aabb().center().x - 15.0).abs() < 0.5);
    assert!((small.aabb().center().x - 25.0).abs() < 0.5);
    let hs = [big.content_hash(), mid.content_hash(), small.content_hash()];

    // Undo / redo of the separate.
    assert!(doc.undo());
    recompute(&mut doc);
    assert!(
        try_mesh(
            &doc,
            BodyId {
                origin: sep,
                slot: 0
            }
        )
        .is_none()
    );
    assert!(doc.redo());
    recompute(&mut doc);
    assert_eq!(
        mesh(
            &doc,
            BodyId {
                origin: sep,
                slot: 1
            }
        )
        .content_hash(),
        hs[2]
    );

    // A dependent of a split-off body and an independent chain; then an upstream parameter
    // edit recomputes reduce, separate and that dependent only.
    let rev = add_feature(
        &mut doc,
        reverse(BodyId {
            origin: sep,
            slot: 0,
        }),
    );
    let b = add_feature(&mut doc, source(&icosphere(5.0, 2)));
    let rev_b = add_feature(&mut doc, reverse(body(b)));
    recompute(&mut doc);
    let p = first_param(&doc, red);
    doc.apply(Command::SetParamExpr {
        id: p,
        expr: "0.3".to_owned(),
    })
    .unwrap();
    recompute(&mut doc);
    assert_eq!(computed(&doc), vec![red, sep, rev]);
    assert_eq!(cached(&doc), vec![a, b, rev_b]);
    assert_ne!(mesh(&doc, body(a)).content_hash(), hs[0]);

    // JSON round-trip and save / load.
    let f = FeatureBox::new(MeshSeparateFeature { body: body(a) });
    let back: FeatureBox = serde_json::from_str(&serde_json::to_string(&f).unwrap()).unwrap();
    assert_eq!(back, f);
    let small_now = mesh(
        &doc,
        BodyId {
            origin: sep,
            slot: 1,
        },
    )
    .content_hash();
    let file = TempFile::new();
    save(&file.0, &doc.snapshot(), &SaveOptions::default()).unwrap();
    let mut reopened = load(&file.0).unwrap().into_document();
    reopened.recompute_blocking(jobs()).unwrap();
    assert_eq!(
        mesh(
            &reopened,
            BodyId {
                origin: sep,
                slot: 1
            }
        )
        .content_hash(),
        small_now
    );
}

// ----------------------------------------------------------------------------- health

#[test]
fn quick_shell_self_intersections_are_a_warning_and_robust_shell_has_none() {
    link();
    let mut doc = Document::new("shell-health");
    let a = add_feature(&mut doc, source(&two_cubes()));
    let mut quick = MeshShellFeature::quick(body(a), "1 mm");
    quick.side = ShellSide::Outside;
    quick.output = OutputMode::NewBody;
    let q = add_feature(&mut doc, FeatureBox::new(quick));
    recompute(&mut doc);
    match health(&doc, q) {
        Health::Warning(d) => {
            assert_eq!(d.len(), 1);
            assert!(
                d[0].message.contains("self-intersecting"),
                "{}",
                d[0].message
            );
            assert!(d[0].message.contains("has 88 "), "{}", d[0].message);
        }
        other => panic!("{other:?}"),
    }
    // NewBody: the input stays.
    assert_eq!(
        mesh(&doc, body(a)).content_hash(),
        two_cubes().content_hash()
    );
    assert!(try_mesh(&doc, body(q)).is_some());

    let mut robust = MeshShellFeature::robust(body(a), "1 mm");
    robust.side = ShellSide::Outside;
    robust.voxel = Some(inline("0.25 mm"));
    robust.output = OutputMode::NewBody;
    let r = add_feature(&mut doc, FeatureBox::new(robust));
    recompute(&mut doc);
    assert_eq!(health(&doc, r), Health::Ok);
    assert!(watertight(&mesh(&doc, body(r))));
}

#[test]
fn bad_parameters_and_missing_sides_fail_with_a_reason() {
    link();
    let mut doc = Document::new("errors");
    let a = add_feature(&mut doc, source(&icosphere(10.0, 3)));
    let bad = [
        FeatureBox::new(MeshRemeshFeature::new(body(a), "-1 mm")),
        FeatureBox::new(MeshReduceFeature::ratio(body(a), "1.5")),
        {
            let mut f = MeshSmoothFeature::new(body(a));
            f.params.method = SmoothMethodParam::Taubin {
                lambda: inline("0.5"),
                mu: inline("-0.2"),
            };
            FeatureBox::new(f)
        },
        FeatureBox::new(MeshShellFeature::quick(body(a), "0 mm")),
        // The plane is above the sphere and the positive side is kept: nothing remains.
        {
            let mut f = MeshPlaneCutFeature::origin(body(a), OriginPlane::XY, "20 mm");
            f.keep = KeepSideParam::Positive;
            FeatureBox::new(f)
        },
        FeatureBox::new(MeshPlaneCutFeature {
            body: body(a),
            plane: CutPlane::Custom {
                origin: [inline("0 mm"), inline("0 mm"), inline("0 mm")],
                normal: [inline("0"), inline("0"), inline("0")],
            },
            keep: KeepSideParam::Negative,
            cap: true,
            snap_tolerance: inline("0 mm"),
        }),
    ];
    let ids: Vec<_> = bad.into_iter().map(|f| add_feature(&mut doc, f)).collect();
    recompute(&mut doc);
    for id in ids {
        assert!(
            matches!(health(&doc, id), Health::Error(_)),
            "{id:?}: {:?}",
            health(&doc, id)
        );
    }
    // The body is untouched by the failed features.
    assert_eq!(
        mesh(&doc, body(a)).content_hash(),
        icosphere(10.0, 3).content_hash()
    );

    // An empty erase list is refused before it enters the design.
    let root = doc.design().root();
    assert!(
        doc.apply(Command::AddFeature {
            at: None,
            feature: FeatureBox::new(MeshEraseFillFeature {
                body: body(a),
                groups: Vec::new(),
                fairing: FairingParam::default(),
            }),
            context: root,
            name: None,
        })
        .is_err()
    );
}

#[test]
fn reduce_reports_a_target_it_cannot_reach() {
    link();
    let mut doc = Document::new("reduce-health");
    // A 4-face target with a 0.9 minimum triangle quality: the quality check stops the
    // collapses long before the target.
    let a = add_feature(&mut doc, source(&torus(48, 24, 10.0, 4.0)));
    let mut f = MeshReduceFeature::ratio(body(a), "0.5");
    f.params.target = ReduceTarget::FaceCount(inline("4"));
    f.params.min_triangle_quality = inline("0.9");
    let r = add_feature(&mut doc, FeatureBox::new(f));
    recompute(&mut doc);
    match health(&doc, r) {
        Health::Warning(d) => assert!(d[0].message.contains("stopped"), "{}", d[0].message),
        other => panic!("{other:?}"),
    }
}

#[test]
fn plane_cut_by_a_construction_plane_follows_the_plane_parameter() {
    link();
    let mut doc = Document::new("cut-construction");
    let h = created_param(
        doc.apply(Command::AddParam {
            name: "cut_height".to_owned(),
            expr: "1 mm".to_owned(),
            unit: None,
            comment: String::new(),
        })
        .unwrap(),
    );
    let a = add_feature(&mut doc, source(&torus(96, 48, 10.0, 4.0)));
    let plane = add_feature(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: inline("cut_height"),
            },
        }),
    );
    let cut = add_feature(
        &mut doc,
        FeatureBox::new(MeshPlaneCutFeature {
            body: body(a),
            plane: CutPlane::Construction {
                plane: ConstructionId(plane),
                offset: inline("0 mm"),
            },
            keep: KeepSideParam::Negative,
            cap: true,
            snap_tolerance: inline("0 mm"),
        }),
    );
    let rev = add_feature(&mut doc, reverse(body(a)));
    let b = add_feature(&mut doc, source(&icosphere(5.0, 2)));
    recompute(&mut doc);
    assert_eq!(health(&doc, cut), Health::Ok);
    let top = mesh(&doc, body(a)).aabb().max.z;
    assert!((top - 1.0).abs() < 1e-9, "{top}");

    doc.apply(Command::SetParamExpr {
        id: h,
        expr: "-2 mm".to_owned(),
    })
    .unwrap();
    recompute(&mut doc);
    assert_eq!(computed(&doc), vec![plane, cut, rev]);
    assert_eq!(cached(&doc), vec![a, b]);
    let top = mesh(&doc, body(a)).aabb().max.z;
    assert!((top + 2.0).abs() < 1e-9, "{top}");
}

#[test]
fn every_op_kind_is_registered() {
    link();
    let reg = amg_doc::try_registry().unwrap();
    for kind in [
        "mesh.remesh",
        "mesh.reduce",
        "mesh.smooth",
        "mesh.plane_cut",
        "mesh.shell",
        "mesh.separate",
        "mesh.erase_fill",
        "mesh.wrap",
    ] {
        assert_eq!(reg.get(kind).unwrap().version, 1, "{kind}");
    }
}

// ----------------------------------------------------------------------------- goldens

/// Done-when: golden volume, area, bbox and Hausdorff (to the blessed reference mesh) of every
/// op's feature output.
#[test]
fn golden_mesh_op_features() {
    link();
    for case in cases() {
        let mut doc = Document::new(case.name);
        let a = add_feature(&mut doc, source(&case.input));
        let op = add_feature(&mut doc, (case.feature)(body(a)));
        recompute(&mut doc);
        assert_eq!(health(&doc, op), Health::Ok, "{}", case.name);
        let out = mesh(&doc, body(a));
        amg_testkit::assert_golden!("mesh", &format!("feature_{}", case.name), soup(&out));
        if let Some(extra) = try_mesh(&doc, body(op)) {
            amg_testkit::assert_golden!(
                "mesh",
                &format!("feature_{}_slot0", case.name),
                soup(&extra)
            );
        }
    }
    let mut doc = Document::new("separate");
    let a = add_feature(&mut doc, source(&three_spheres()));
    let sep = add_feature(
        &mut doc,
        FeatureBox::new(MeshSeparateFeature { body: body(a) }),
    );
    recompute(&mut doc);
    for (i, id) in [
        body(a),
        BodyId {
            origin: sep,
            slot: 0,
        },
        BodyId {
            origin: sep,
            slot: 1,
        },
    ]
    .into_iter()
    .enumerate()
    {
        amg_testkit::assert_golden!(
            "mesh",
            &format!("feature_separate_{i}"),
            soup(&mesh(&doc, id))
        );
    }
}
