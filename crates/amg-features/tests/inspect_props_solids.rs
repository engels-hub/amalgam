//! INS-10: mass properties of B-rep solids, mixed selections and component / occurrence
//! totals of assemblies.
//!
//! Mesh bodies and the "backend unavailable" path run everywhere; B-rep solids need the `occt`
//! feature (AMG_OCCT_DIR).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "test code"
)]

use std::sync::{Arc, OnceLock};

use amg_core::{Density, Mat3, Point3, Transform, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, ComponentId, CreatedId, Dependency,
    Document, Feature, FeatureBox, FeatureError, FeatureOutput, OccurrenceId, RecomputeCtx,
    register_feature,
};
use amg_features::inspect::InspectError;
use amg_features::inspect::doc::assembly::{
    PlacedBody, PropsPick, PropsScope, occurrence_name, scopes_of, selected_instances,
};
use amg_features::inspect::props::{
    MassProps, PropsGeom, PropsItem, combine, group_totals, items_mass_props, mesh_mass_props,
    transform_mass_props,
};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};
use amg_mesh::{IndexedMesh, MeshBody};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------- helpers

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn rho() -> Density {
    Density::g_per_cm3(1.24)
}

/// A placement that keeps no axis aligned.
fn skew(t: [f64; 3], angle: f64) -> Transform {
    Transform::from_translation(Vec3::from(t))
        * Transform::from_axis_angle(Vec3::new(1.0, -2.0, 3.0).normalize(), angle)
}

fn close(got: f64, want: f64, scale: f64, what: &str) {
    let e = (got - want).abs() / scale;
    assert!(e <= 1e-9, "{what}: {got} vs {want} (relative {e:e})");
}

fn norm(m: &Mat3) -> f64 {
    m.to_cols_array().iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn close_point(got: Point3, want: Point3, scale: f64, what: &str) {
    let e = (got - want).length() / scale;
    assert!(e <= 1e-9, "{what}: {got} vs {want} (relative {e:e})");
}

fn close_mat(got: &Mat3, want: &Mat3, what: &str) {
    let e = norm(&(*got - *want)) / norm(want);
    assert!(e <= 1e-9, "{what}: {got:?} vs {want:?} (relative {e:e})");
}

/// `m (|d|² 𝟙 − d dᵀ)`.
fn point_inertia(m: f64, d: Vec3) -> Mat3 {
    let outer = Mat3::from_cols(d * d.x, d * d.y, d * d.z);
    (Mat3::IDENTITY * d.length_squared() - outer) * m
}

/// Closed, outward-oriented box `[min, min + size]`.
fn cube(min: [f64; 3], size: [f64; 3]) -> IndexedMesh {
    let p = |i: u32| {
        Point3::new(
            min[0] + if i & 1 != 0 { size[0] } else { 0.0 },
            min[1] + if i & 2 != 0 { size[1] } else { 0.0 },
            min[2] + if i & 4 != 0 { size[2] } else { 0.0 },
        )
    };
    let positions = (0..8).map(p).collect();
    let quads = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];
    let mut tris = Vec::new();
    for [a, b, c, d] in quads {
        tris.push([a, b, c]);
        tris.push([a, c, d]);
    }
    IndexedMesh::new(positions, tris).unwrap()
}

/// Analytic mass properties of a box `size` whose corner is at the frame origin of `world`.
struct Analytic {
    volume: f64,
    mass: f64,
    com: Point3,
    inertia: Mat3,
}

fn analytic_box(size: Vec3, world: &Transform) -> Analytic {
    let volume = size.x * size.y * size.z;
    let mass = rho().get() * volume;
    let s = size * size;
    let local = Mat3::from_diagonal(Vec3::new(s.y + s.z, s.x + s.z, s.x + s.y) * (mass / 12.0));
    let r = world.matrix3;
    Analytic {
        volume,
        mass,
        com: world.transform_point3(size * 0.5),
        inertia: r * local * r.transpose(),
    }
}

fn assert_matches(p: &MassProps, a: &Analytic, length: f64, what: &str) {
    assert_eq!(p.not_watertight, None, "{what}");
    close(p.volume.unwrap().get(), a.volume, a.volume, what);
    close(p.mass.unwrap().get(), a.mass, a.mass, what);
    close_point(p.center_of_mass.unwrap(), a.com, length, what);
    close_mat(&p.inertia_com.unwrap(), &a.inertia, what);
    close_mat(
        &p.inertia_origin.unwrap(),
        &(a.inertia + point_inertia(a.mass, a.com)),
        what,
    );
    // Each principal pair is an eigenpair of the analytic tensor (a residual, so repeated
    // moments need no special case).
    let moments = p.principal_moments.unwrap();
    let axes = p.principal_axes.unwrap();
    assert!(
        moments[0] <= moments[1] && moments[1] <= moments[2],
        "{what}"
    );
    for (k, (m, axis)) in moments.iter().zip(axes).enumerate() {
        let v = axis.get();
        let residual = (a.inertia * v - v * *m).length() / norm(&a.inertia);
        assert!(residual <= 1e-9, "{what}: principal pair {k}: {residual:e}");
        for other in &axes[k + 1..] {
            assert!(
                v.dot(other.get()).abs() <= 1e-9,
                "{what}: axes not orthogonal"
            );
        }
    }
}

// ---------------------------------------------------------------- document fixtures

/// Mesh body slot 0 in the context component.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Ins10Mesh {
    mesh: IndexedMesh,
}

impl Feature for Ins10Mesh {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(body(ctx, BodyGeom::Mesh(Arc::new(self.mesh.clone()))))
    }
}

register_feature!(Ins10Mesh, "test.ins10_mesh", version = 1);

/// A solid body without B-rep geometry (what a build without the kernel holds).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Ins10Placeholder {}

impl Feature for Ins10Placeholder {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(body(ctx, BodyGeom::Solid(amg_doc::stub::Shape::new())))
    }
}

register_feature!(Ins10Placeholder, "test.ins10_placeholder", version = 1);

fn body(ctx: &mut RecomputeCtx<'_>, geom: BodyGeom) -> FeatureOutput {
    FeatureOutput {
        bodies: vec![BodyChange {
            id: ctx.new_body_id(0),
            change: BodyChangeKind::Created { geom, naming: None },
        }],
        ..FeatureOutput::default()
    }
}

fn jobs() -> &'static JobSystem {
    static JOBS: OnceLock<JobSystem> = OnceLock::new();
    JOBS.get_or_init(|| JobSystem::new(JobSystemConfig::default()).unwrap())
}

fn add(doc: &mut Document, feature: FeatureBox, context: ComponentId) -> BodyId {
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature,
            context,
            name: None,
        })
        .unwrap();
    let f = out
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap();
    BodyId { origin: f, slot: 0 }
}

fn new_component(
    doc: &mut Document,
    parent: ComponentId,
    transform: Transform,
) -> (ComponentId, OccurrenceId) {
    let out = doc
        .apply(Command::NewComponent {
            parent,
            name: "Part".to_owned(),
            transform,
        })
        .unwrap();
    let (mut c, mut o) = (None, None);
    for id in out.created {
        match id {
            CreatedId::Component(x) => c = Some(x),
            CreatedId::Occurrence(x) => o = Some(x),
            _ => {}
        }
    }
    (c.unwrap(), o.unwrap())
}

fn place_again(
    doc: &mut Document,
    parent: ComponentId,
    component: ComponentId,
    transform: Transform,
) -> OccurrenceId {
    let out = doc
        .apply(Command::AddOccurrence {
            parent,
            component,
            transform,
            name: None,
        })
        .unwrap();
    out.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Occurrence(o) => Some(*o),
            _ => None,
        })
        .unwrap()
}

/// The items of the placed instances, all with [`rho`].
fn items(doc: &Document, placed: &[PlacedBody]) -> Vec<PropsItem> {
    placed
        .iter()
        .map(|p| PropsItem {
            geom: match &doc.evaluated().body(p.body).unwrap().geom {
                BodyGeom::Mesh(m) => PropsGeom::Mesh(MeshBody::from_arc(Arc::clone(m))),
                BodyGeom::Solid(s) => PropsGeom::solid(s.brep().cloned()),
                other => panic!("{other:?}"),
            },
            to_world: p.world,
            density: rho(),
        })
        .collect()
}

/// A part (a 10 x 20 x 30 body built by `part`) in a component placed twice.
struct TwoPlacements {
    doc: Document,
    body: BodyId,
    component: ComponentId,
    occurrences: [OccurrenceId; 2],
    worlds: [Transform; 2],
}

fn two_placements(part: FeatureBox) -> TwoPlacements {
    let mut doc = Document::new("assembly");
    let root = doc.design().root();
    let worlds = [skew([40.0, -5.0, 8.0], 0.7), skew([-60.0, 33.0, 2.0], 2.1)];
    let (component, first) = new_component(&mut doc, root, worlds[0]);
    let body = add(&mut doc, part, component);
    let second = place_again(&mut doc, root, component, worlds[1]);
    doc.recompute_blocking(jobs()).unwrap();
    TwoPlacements {
        doc,
        body,
        component,
        occurrences: [first, second],
        worlds,
    }
}

const PART: Vec3 = Vec3::new(10.0, 20.0, 30.0);

/// Done-when 3: the total of a two-occurrence assembly equals the sum of the two placed parts
/// by the parallel-axis rule, and the occurrence and component totals agree with it.
fn check_two_occurrence_assembly(t: &TwoPlacements) {
    let ev = t.doc.evaluated();
    let placed = selected_instances(ev, &[]);
    assert_eq!(placed.len(), 2);
    assert!(placed.iter().all(|p| p.body == t.body));
    let results = items_mass_props(&items(&t.doc, &placed), &ctx()).unwrap();
    let parts: Vec<MassProps> = results.into_iter().map(Result::unwrap).collect();
    let total = combine(&parts);

    // By hand: each placed box analytically, summed by the parallel-axis rule.
    let a = t.worlds.map(|w| analytic_box(PART, &w));
    let mass = a[0].mass + a[1].mass;
    let com = (a[0].com * a[0].mass + a[1].com * a[1].mass) / mass;
    let inertia = a[0].inertia
        + point_inertia(a[0].mass, a[0].com - com)
        + a[1].inertia
        + point_inertia(a[1].mass, a[1].com - com);
    let want = Analytic {
        volume: a[0].volume + a[1].volume,
        mass,
        com,
        inertia,
    };
    eprintln!(
        "assembly: volume {:.1e}, com {:.1e}, inertia {:.1e} (relative)",
        (total.volume.unwrap().get() - want.volume).abs() / want.volume,
        (total.center_of_mass.unwrap() - want.com).length() / 100.0,
        norm(&(total.inertia_com.unwrap() - want.inertia)) / norm(&want.inertia),
    );
    assert_matches(&total, &want, 100.0, "assembly total");

    // Per occurrence and per component.
    let keyed: Vec<(Vec<PropsScope>, Option<&MassProps>)> = placed
        .iter()
        .zip(&parts)
        .map(|(p, m)| (scopes_of(t.doc.design(), &p.path), Some(m)))
        .collect();
    let groups = group_totals(&keyed);
    let group = |s: &PropsScope| groups.iter().find(|g| &g.key == s).unwrap();
    assert_eq!(groups.len(), 4, "2 occurrences, the part and the root");
    for (o, w) in t.occurrences.iter().zip(&t.worlds) {
        let g = group(&PropsScope::Occurrence(vec![*o]));
        assert_eq!((g.members.len(), g.skipped), (1, 0));
        assert_matches(
            g.total.as_ref().unwrap(),
            &analytic_box(PART, w),
            100.0,
            "occurrence",
        );
        // Selecting the occurrence picks exactly its instance.
        let sel = selected_instances(ev, &[PropsPick::Occurrence(*o)]);
        assert_eq!(sel.len(), 1);
        assert_eq!((sel[0].path.clone(), sel[0].world), (vec![*o], *w));
    }
    for c in [t.component, t.doc.design().root()] {
        let g = group(&PropsScope::Component(c));
        assert_eq!(g.members, [0, 1]);
        assert_matches(g.total.as_ref().unwrap(), &want, 100.0, "component");
    }
}

#[test]
fn a_two_occurrence_mesh_assembly_obeys_the_parallel_axis_rule() {
    let mesh = cube([0.0; 3], PART.to_array());
    check_two_occurrence_assembly(&two_placements(FeatureBox::new(Ins10Mesh { mesh })));
}

#[test]
fn selections_name_instances_occurrences_or_the_whole_design() {
    let mesh = cube([0.0; 3], PART.to_array());
    let mut t = two_placements(FeatureBox::new(Ins10Mesh { mesh: mesh.clone() }));
    let root = t.doc.design().root();
    let own = add(
        &mut t.doc,
        FeatureBox::new(Ins10Mesh { mesh: mesh.clone() }),
        root,
    );
    // A nested component inside the part, placed once in it: seen twice from the root.
    let (inner, inner_occ) = new_component(&mut t.doc, t.component, skew([0.0, 0.0, 50.0], 0.2));
    let nested = add(&mut t.doc, FeatureBox::new(Ins10Mesh { mesh }), inner);
    t.doc.recompute_blocking(jobs()).unwrap();
    let ev = t.doc.evaluated();
    let [o0, o1] = t.occurrences;

    let all = selected_instances(ev, &[]);
    assert_eq!(all.len(), 5, "1 root body, 2 parts, 2 nested");
    assert_eq!(all.iter().filter(|p| p.body == nested).count(), 2);
    // An occurrence takes everything below it.
    let under = selected_instances(ev, &[PropsPick::Occurrence(o1)]);
    assert_eq!(
        under
            .iter()
            .map(|p| (p.body, p.path.clone()))
            .collect::<Vec<_>>(),
        [(t.body, vec![o1]), (nested, vec![o1, inner_occ])]
    );
    // An occurrence with nothing below it gives nothing, not the whole design.
    assert!(selected_instances(ev, &[PropsPick::Occurrence(OccurrenceId(999_999))]).is_empty());
    // A body instance, an invalid or missing path (first placement), and duplicates once.
    let one = |path| {
        selected_instances(ev, &[PropsPick::Body { body: t.body, path }])
            .into_iter()
            .map(|p| p.path)
            .collect::<Vec<_>>()
    };
    assert_eq!(one(Some(vec![o1])), [vec![o1]]);
    assert_eq!(one(None), [vec![o0]]);
    assert_eq!(one(Some(Vec::new())), [vec![o0]]);
    let mixed = selected_instances(
        ev,
        &[
            PropsPick::Occurrence(o0),
            PropsPick::Body {
                body: t.body,
                path: Some(vec![o0]),
            },
            PropsPick::Body {
                body: own,
                path: None,
            },
        ],
    );
    assert_eq!(mixed.len(), 3);
    // Scopes of a nested instance: both prefixes and all three components.
    let design = t.doc.design();
    assert_eq!(
        scopes_of(design, &[o1, inner_occ]),
        [
            PropsScope::Component(root),
            PropsScope::Occurrence(vec![o1]),
            PropsScope::Component(t.component),
            PropsScope::Occurrence(vec![o1, inner_occ]),
            PropsScope::Component(inner),
        ]
    );
    assert_eq!(occurrence_name(design, o0), "Part:1");
    assert_eq!(occurrence_name(design, o1), "Part:2");
}

#[test]
fn a_placement_moves_the_properties_like_the_geometry() {
    let m = MeshBody::new(cube([1.0, 2.0, 3.0], PART.to_array()));
    let local = mesh_mass_props(&m, &Transform::IDENTITY, rho(), &ctx()).unwrap();
    // Rigid and mirrored placements.
    let mirror = Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0));
    for t in [
        skew([5.0, 6.0, -7.0], 1.3),
        skew([1.0, 0.0, 9.0], 0.4) * mirror,
    ] {
        let moved = transform_mass_props(&local, &t).unwrap();
        let direct = mesh_mass_props(&m, &t, rho(), &ctx()).unwrap();
        close(
            moved.volume.unwrap().get(),
            direct.volume.unwrap().get(),
            direct.volume.unwrap().get(),
            "volume",
        );
        close_point(
            moved.center_of_mass.unwrap(),
            direct.center_of_mass.unwrap(),
            100.0,
            "com",
        );
        close_mat(
            &moved.inertia_com.unwrap(),
            &direct.inertia_com.unwrap(),
            "inertia",
        );
        close_mat(
            &moved.inertia_origin.unwrap(),
            &direct.inertia_origin.unwrap(),
            "inertia about the origin",
        );
    }
    let scale = Transform::from_scale(Vec3::splat(2.0));
    assert!(matches!(
        transform_mass_props(&local, &scale),
        Err(InspectError::NonRigidTransform)
    ));
}

/// A solid without B-rep geometry gives its own error ("backend unavailable" in a build
/// without the kernel) and leaves the mesh bodies of a mixed selection alone.
#[test]
fn a_solid_without_geometry_fails_alone() {
    let mut doc = Document::new("mixed");
    let root = doc.design().root();
    let solid = add(&mut doc, FeatureBox::new(Ins10Placeholder {}), root);
    let mesh = cube([0.0; 3], PART.to_array());
    add(&mut doc, FeatureBox::new(Ins10Mesh { mesh }), root);
    doc.recompute_blocking(jobs()).unwrap();
    let placed = selected_instances(doc.evaluated(), &[]);
    assert_eq!(placed.len(), 2);
    assert_eq!(placed[0].body, solid);
    let r = items_mass_props(&items(&doc, &placed), &ctx()).unwrap();
    let err = r[0].as_ref().unwrap_err();
    if cfg!(feature = "occt") {
        assert!(matches!(err, InspectError::Brep(_)), "{err}");
    } else {
        assert!(matches!(err, InspectError::BackendUnavailable), "{err}");
        assert!(err.to_string().contains("backend unavailable"), "{err}");
    }
    let mesh = r[1].as_ref().unwrap();
    close(mesh.volume.unwrap().get(), 6000.0, 6000.0, "mesh volume");
    let keyed = vec![(vec![0], r[0].as_ref().ok()), (vec![0], r[1].as_ref().ok())];
    let g = group_totals(&keyed);
    assert_eq!((g[0].members.clone(), g[0].skipped), (vec![1], 1));
}

#[test]
fn a_cancelled_run_reports_cancellation() {
    let token = amg_jobs::CancelToken::new();
    token.cancel();
    let ctx = JobCtx::with_token(token);
    let items = [PropsItem {
        geom: PropsGeom::Mesh(MeshBody::new(cube([0.0; 3], [1.0; 3]))),
        to_world: Transform::IDENTITY,
        density: rho(),
    }];
    assert!(matches!(
        items_mass_props(&items, &ctx),
        Err(InspectError::Cancelled)
    ));
}

// ---------------------------------------------------------------- B-rep solids

#[cfg(feature = "occt")]
mod solid {
    use super::*;
    use amg_brep::{BrepKernel, KernelConfig, OcctKernel, Shape, TessQuality};
    use amg_core::{Dir3, Plane, Tolerance};
    use amg_features::inspect::props::solid_mass_props;
    use std::f64::consts::{PI, TAU};

    fn kernel() -> &'static OcctKernel {
        static K: OnceLock<OcctKernel> = OnceLock::new();
        K.get_or_init(|| OcctKernel::new(KernelConfig::default()))
    }

    /// A frame tilted off every axis, and the same frame as a transform from local space.
    fn frame() -> (Plane, Transform) {
        let tol = Tolerance::mesh();
        let normal = Dir3::new(Vec3::new(1.0, 2.0, 3.0), &tol).unwrap();
        let f = Plane::from_normal_x(Point3::new(12.0, -7.0, 4.0), normal, Vec3::X, &tol).unwrap();
        let t = Transform {
            matrix3: Mat3::from_cols(f.x.get(), f.y.get(), f.normal.get()),
            translation: f.origin,
        };
        (f, t)
    }

    /// Done-when 1: box, cylinder and sphere solids match the analytic volume, centre of mass
    /// and inertia within 1e-9 relative, also seen through an occurrence placement.
    #[test]
    fn box_cylinder_and_sphere_match_the_analytic_values() {
        let (f, local) = frame();
        let k = kernel();
        let size = Vec3::new(10.0, 20.0, 30.0);
        let (r, h) = (5.0, 12.0);
        let rs = 7.0;
        let analytic = |volume: f64, com: Vec3, diag: Vec3, t: &Transform| {
            let mass = rho().get() * volume;
            let m = t.matrix3;
            Analytic {
                volume,
                mass,
                com: t.transform_point3(com),
                inertia: m * Mat3::from_diagonal(diag * mass) * m.transpose(),
            }
        };
        let shapes: [(&str, Shape); 3] = [
            ("box", k.make_box(&f, size).unwrap().shape),
            ("cylinder", k.make_cylinder(&f, r, h, TAU).unwrap().shape),
            ("sphere", k.make_sphere(&f, rs).unwrap().shape),
        ];
        let mirror = Transform::from_scale(Vec3::new(1.0, -1.0, 1.0));
        for placement in [
            Transform::IDENTITY,
            skew([-30.0, 14.0, 9.0], 1.1),
            skew([3.0, 3.0, 3.0], 0.3) * mirror,
        ] {
            let t = placement * local;
            let s = size * size;
            let radial = (3.0 * r * r + h * h) / 12.0;
            let want = [
                analytic(
                    size.x * size.y * size.z,
                    size * 0.5,
                    Vec3::new(s.y + s.z, s.x + s.z, s.x + s.y) / 12.0,
                    &t,
                ),
                analytic(
                    PI * r * r * h,
                    Vec3::new(0.0, 0.0, h / 2.0),
                    Vec3::new(radial, radial, r * r / 2.0),
                    &t,
                ),
                analytic(
                    4.0 / 3.0 * PI * rs * rs * rs,
                    Vec3::ZERO,
                    Vec3::splat(0.4 * rs * rs),
                    &t,
                ),
            ];
            let areas = [
                2.0 * (size.x * size.y + size.x * size.z + size.y * size.z),
                TAU * r * (r + h),
                4.0 * PI * rs * rs,
            ];
            for (((name, shape), want), area) in shapes.iter().zip(&want).zip(areas) {
                let p = solid_mass_props(shape, &placement, rho(), &ctx()).unwrap();
                eprintln!(
                    "{name}: volume {:.1e}, com {:.1e}, inertia {:.1e} (relative)",
                    (p.volume.unwrap().get() - want.volume).abs() / want.volume,
                    (p.center_of_mass.unwrap() - want.com).length() / 50.0,
                    norm(&(p.inertia_com.unwrap() - want.inertia)) / norm(&want.inertia),
                );
                assert_matches(&p, want, 50.0, name);
                close(p.area.get(), area, area, name);
            }
        }
    }

    /// Done-when 2: the same part as a fine tessellation agrees with the solid within the
    /// tessellation error bound: a surface moved by at most the deflection `d` changes the
    /// volume by at most `A·d`, a first moment by `A·d·R` and an inertia entry by
    /// `ρ·A·d·R²` (`R`: the largest distance from the centre of mass).
    #[test]
    fn a_fine_tessellation_agrees_within_the_deflection_bound() {
        let (f, _) = frame();
        let k = kernel();
        let parts: [(&str, Shape, f64); 3] = [
            (
                "cylinder",
                k.make_cylinder(&f, 5.0, 12.0, TAU).unwrap().shape,
                7.9,
            ),
            ("sphere", k.make_sphere(&f, 7.0).unwrap().shape, 7.0),
            (
                "torus",
                k.make_torus(&f, 10.0, 3.0, TAU).unwrap().shape,
                13.0,
            ),
        ];
        let placement = skew([8.0, -2.0, 5.0], 0.9);
        for (name, shape, radius) in parts {
            let d = 0.002;
            let q = TessQuality {
                linear: d,
                angular: 0.1,
                relative: false,
                min_size: 0.0,
                max_linear_abs: None,
            };
            let tess = k.tessellate(&shape, &q, &ctx()).unwrap();
            let (positions, tris, _) = tess.to_indexed_parts();
            let mesh = MeshBody::new(IndexedMesh::new(positions, tris).unwrap());
            let exact = solid_mass_props(&shape, &placement, rho(), &ctx()).unwrap();
            let approx = mesh_mass_props(&mesh, &placement, rho(), &ctx()).unwrap();
            assert_eq!(approx.not_watertight, None, "{name}");
            let area = exact.area.get();
            let volume = exact.volume.unwrap().get();
            let dv = (approx.volume.unwrap().get() - volume).abs();
            assert!(
                dv > 0.0 && dv <= area * d,
                "{name}: volume {dv} > {}",
                area * d
            );
            assert!((approx.area.get() - area).abs() <= area * d, "{name}: area");
            let dc = (approx.center_of_mass.unwrap() - exact.center_of_mass.unwrap()).length();
            let com_bound = area * d * radius / (volume - area * d);
            assert!(dc <= com_bound, "{name}: com {dc} > {com_bound}");
            let di = approx.inertia_com.unwrap() - exact.inertia_com.unwrap();
            let bound = rho().get() * area * d * radius * radius;
            let worst = di
                .to_cols_array()
                .iter()
                .fold(0.0f64, |m, x| m.max(x.abs()));
            assert!(worst <= bound, "{name}: inertia {worst:e} > {bound:e}");
            eprintln!(
                "{name}: {} triangles; volume {dv:.4} <= {:.4} mm3, com {dc:.2e} <= \
                 {com_bound:.2e} mm, inertia {worst:.3e} <= {bound:.3e} t mm2; relative \
                 volume error {:.2e}",
                mesh.mesh().tris.len(),
                area * d,
                dv / volume
            );
        }
    }

    /// A solid box built by a feature.
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct Ins10Box {
        size: [f64; 3],
    }

    impl Feature for Ins10Box {
        fn inputs(&self) -> Vec<Dependency> {
            Vec::new()
        }

        fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
            let s = kernel()
                .make_box(&Plane::XY, Vec3::from(self.size))
                .map_err(|e| FeatureError::Kernel(e.to_string()))?
                .shape;
            Ok(body(
                ctx,
                BodyGeom::Solid(amg_doc::stub::Shape::from_brep(s)),
            ))
        }
    }

    register_feature!(Ins10Box, "test.ins10_box", version = 1);

    #[test]
    fn a_two_occurrence_solid_assembly_obeys_the_parallel_axis_rule() {
        let part = FeatureBox::new(Ins10Box {
            size: PART.to_array(),
        });
        check_two_occurrence_assembly(&two_placements(part));
    }

    /// A mixed selection (a solid and a mesh of the same box) totals twice one box.
    #[test]
    fn a_mixed_solid_and_mesh_selection_adds_up() {
        let mut doc = Document::new("mixed");
        let root = doc.design().root();
        add(
            &mut doc,
            FeatureBox::new(Ins10Box {
                size: PART.to_array(),
            }),
            root,
        );
        let mesh = cube([0.0; 3], PART.to_array());
        add(&mut doc, FeatureBox::new(Ins10Mesh { mesh }), root);
        doc.recompute_blocking(jobs()).unwrap();
        let placed = selected_instances(doc.evaluated(), &[]);
        let parts: Vec<MassProps> = items_mass_props(&items(&doc, &placed), &ctx())
            .unwrap()
            .into_iter()
            .map(Result::unwrap)
            .collect();
        let mut want = analytic_box(PART, &Transform::IDENTITY);
        want.volume *= 2.0;
        want.mass *= 2.0;
        want.inertia *= 2.0;
        assert_matches(&combine(&parts), &want, 50.0, "mixed total");
    }

    #[test]
    fn a_surface_body_has_an_area_but_no_volume() {
        let (f, _) = frame();
        let k = kernel();
        let solid = k.make_box(&f, Vec3::new(1.0, 2.0, 3.0)).unwrap().shape;
        let face = solid
            .sub_shape(amg_brep::SubShape {
                kind: amg_brep::SubKind::Face,
                index: 0,
            })
            .unwrap();
        let p = solid_mass_props(&face, &Transform::IDENTITY, rho(), &ctx()).unwrap();
        assert!(p.volume.is_none() && p.mass.is_none());
        assert!(p.area.get() > 0.0);
        assert!(p.not_watertight.unwrap().contains("not a solid"));
        assert!(matches!(
            solid_mass_props(
                &solid,
                &Transform::from_scale(Vec3::splat(2.0)),
                rho(),
                &ctx()
            ),
            Err(InspectError::NonRigidTransform)
        ));
    }
}
