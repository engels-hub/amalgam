//! INS-11: section analysis on solids and on reference bases.
//!
//! - Done-when 1: the INS-02 watertight test on the section tessellation of 20+ solid parts
//!   (`testdata/brep` fixtures, STEP fixtures and parts built with the kernel).
//! - Done-when 2: a construction-plane base follows a parameter edit (runs everywhere).
//! - Done-when 3 (resolution half; the app half is `amg-ui/tests/section_solids.rs`): a face
//!   base follows its face and stops resolving when the face is deleted.
//!
//! The solid tests need the `occt` feature (AMG_OCCT_DIR).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stderr,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

// Shared with the SOL-01 tests; this binary doesn't use their goldens.
#[allow(unused_imports, reason = "shared test module")]
mod solid_common;

use amg_core::{Plane, Point3, Vec3};
use amg_doc::{Command, ConstructionId, Document, EntityRef, Expr, ExprSrc, FeatureBox};
use amg_features::construct::{ConstructPlane, OriginEntity, PlaneDef, RefInput};
use amg_features::inspect::doc::base::{BaseError, is_construction, resolve_base};
use amg_features::inspect::doc::{SectionAnalysis, SectionBase};
use solid_common::*;

/// The section plane of `a` with its reference resolved in `doc`, and whether it fell back.
fn plane_in(doc: &Document, a: &SectionAnalysis) -> (Plane, bool) {
    a.plane(&|e| {
        resolve_base(doc.evaluated(), doc.design(), e)
            .ok()
            .map(|b| b.plane)
    })
}

#[test]
fn a_construction_plane_base_follows_a_parameter_edit() {
    amg_features::construct::link();
    let mut doc = Document::new("ins11-param");
    let out = doc
        .apply(Command::AddParam {
            name: "height".into(),
            expr: "10 mm".into(),
            unit: None,
            comment: String::new(),
        })
        .unwrap();
    let param = out
        .created
        .iter()
        .find_map(|c| match c {
            amg_doc::CreatedId::Param(p) => Some(*p),
            _ => None,
        })
        .expect("a parameter");
    let plane = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: Expr::Inline(ExprSrc("height".to_owned())),
            },
        }),
    );
    let entity = EntityRef::Construction(ConstructionId(plane));
    assert!(is_construction(&entity));
    let first = resolve_base(doc.evaluated(), doc.design(), &entity).unwrap();
    assert!(!first.uncertain);
    let mut a = SectionAnalysis::new(
        "s",
        SectionBase::Ref {
            entity: entity.clone(),
            last: first.plane,
        },
    );
    a.offset = 2.0;
    assert_eq!(plane_in(&doc, &a).0.origin, Point3::new(0.0, 0.0, 12.0));

    for (expr, z) in [("25 mm", 27.0), ("height_is_gone * 0 + 0.5 in", 14.7)] {
        // The second expression is refused (unknown name); the plane stays.
        let ok = doc
            .apply(Command::SetParamExpr {
                id: param,
                expr: expr.into(),
            })
            .is_ok();
        doc.recompute_blocking(jobs()).unwrap();
        let (p, fallback) = plane_in(&doc, &a);
        assert!(!fallback);
        let want = if ok { z } else { 27.0 };
        assert!((p.origin.z - want).abs() < 1e-12, "{expr}: {}", p.origin.z);
    }

    // A deleted plane: the stored plane is used.
    doc.apply(Command::DeleteFeatures { ids: vec![plane] })
        .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    assert!(matches!(
        resolve_base(doc.evaluated(), doc.design(), &entity),
        Err(BaseError::Resolve(_))
    ));
    let (p, fallback) = plane_in(&doc, &a);
    assert!(fallback);
    assert_eq!(p.origin, Point3::new(0.0, 0.0, 12.0));
    assert_eq!(p.normal.get(), Vec3::Z);
}

#[cfg(feature = "occt")]
mod solid {
    use amg_brep::geom::Surface;
    use amg_brep::{BoolOp, BoolOpts, BrepKernel, FaceId as BrepFace, Shape, SubKind};
    use amg_core::{Dir3, Point2, Tolerance, Transform};
    use amg_doc::naming::{self, TopoKind};
    use amg_doc::{BodyId, Health};
    use amg_features::inspect::section::{
        CapOpts, SectionPlanes, section_mesh, section_quality, tessellate_solid,
    };
    use amg_features::sketch::{PlaneInput, SketchFeature};
    use amg_features::solid::{BodyOp, Extrude, OpTarget};
    use amg_jobs::{CancelToken, JobCtx};
    use amg_mesh::{
        AnalyzeOpts, IndexedMesh, KeepSide, MeshBody, PlaneCutOpts, WeldOpts, analyze, plane_cut,
    };

    use super::*;

    fn ctx() -> JobCtx {
        JobCtx::detached()
    }

    // ---- the 20 parts ----------------------------------------------------------------------

    /// `testdata/brep` fixtures with a solid (`bspline_patch` and `revolved_profile` are
    /// surfaces).
    const BREP: [&str; 9] = [
        "box",
        "cone",
        "cylinder",
        "cylinder_mirrored",
        "filleted_box",
        "located_compound",
        "plate_with_holes",
        "sphere",
        "torus",
    ];

    /// STEP fixtures with solids.
    const STEP: [&str; 9] = [
        "bolt_pattern",
        "box_ap242",
        "coloured_box_ap214",
        "cylinder_ap203",
        "filleted_box",
        "inch_block",
        "nested_assembly",
        "plate_with_holes",
        "two_roots",
    ];

    fn built() -> Vec<(String, Shape)> {
        let k = kernel();
        let at = |x: f64, y: f64, z: f64| Plane {
            origin: Point3::new(x, y, z),
            ..Plane::XY
        };
        let boxed = |p: Plane, s: Vec3| k.make_box(&p, s).unwrap().shape;
        let cylinder = |p: Plane, r: f64, h: f64| {
            k.make_cylinder(&p, r, h, std::f64::consts::TAU)
                .unwrap()
                .shape
        };
        let sphere = |p: Plane, r: f64| k.make_sphere(&p, r).unwrap().shape;
        let boolean = |op, a: Shape, b: Shape| {
            k.boolean(op, &[a], &[b], &BoolOpts::default(), &ctx())
                .unwrap()
                .shape
        };
        let across = Plane::from_normal(Point3::new(-10.0, 0.0, 0.0), Dir3::X);
        vec![
            (
                "blind_hole".into(),
                boolean(
                    BoolOp::Cut,
                    boxed(at(0.0, 0.0, 0.0), Vec3::new(20.0, 20.0, 10.0)),
                    cylinder(at(10.0, 10.0, 4.0), 4.0, 8.0),
                ),
            ),
            (
                "steinmetz".into(),
                boolean(
                    BoolOp::Common,
                    cylinder(at(0.0, 0.0, -10.0), 5.0, 20.0),
                    cylinder(across, 5.0, 20.0),
                ),
            ),
            (
                "sphere_union".into(),
                boolean(
                    BoolOp::Fuse,
                    sphere(at(0.0, 0.0, 0.0), 6.0),
                    sphere(at(7.0, 1.0, 2.0), 5.0),
                ),
            ),
            (
                "box_with_cavity".into(),
                boolean(
                    BoolOp::Cut,
                    boxed(at(0.0, 0.0, 0.0), Vec3::new(20.0, 16.0, 12.0)),
                    boxed(at(4.0, 4.0, 3.0), Vec3::new(10.0, 8.0, 6.0)),
                ),
            ),
        ]
    }

    /// Every part: its name and the solids of its file (or the built shape).
    fn parts() -> Vec<(String, Shape)> {
        let mut out = Vec::new();
        for name in BREP {
            let Some(path) = amg_testkit::testdata(&format!("brep/{name}.brep")) else {
                continue;
            };
            let bytes = std::fs::read(path).unwrap();
            out.push((
                format!("{name}.brep"),
                Shape::read_text(bytes.as_slice()).unwrap(),
            ));
        }
        for name in STEP {
            let Some(path) = amg_testkit::testdata(&format!("brep/step/{name}.step")) else {
                continue;
            };
            let imported =
                amg_io::brep::import_step_with(&path, &Default::default(), &ctx()).unwrap();
            let solids: Vec<Shape> = imported
                .scene
                .objects
                .iter()
                .map(|o| o.shape.clone())
                .filter(|s| !s.solids().is_empty())
                .collect();
            assert!(!solids.is_empty(), "{name}.step has no solid");
            for (i, s) in solids.into_iter().enumerate() {
                out.push((format!("{name}.step#{i}"), s));
            }
        }
        out.extend(built());
        out
    }

    // ---- the INS-02 watertight test (note §3.4) --------------------------------------------

    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }

        fn dir(&mut self) -> Vec3 {
            loop {
                let v = Vec3::new(self.next(), self.next(), self.next()) * 2.0 - Vec3::ONE;
                let l = v.length();
                if l > 0.1 && l <= 1.0 {
                    return v / l;
                }
            }
        }
    }

    fn snap() -> f64 {
        Tolerance::mesh().linear
    }

    fn cut(m: &IndexedMesh, p: Plane, cap: bool) -> amg_mesh::PlaneCutResult {
        plane_cut(
            m,
            &PlaneCutOpts {
                plane: p,
                keep: KeepSide::Negative,
                cap,
                snap_tolerance: snap(),
            },
            &ctx(),
        )
        .unwrap()
    }

    /// `plane_cut(keep = Negative, cap = false)` + the INS-02 cap, welded at the snap
    /// tolerance: `Ok` when closed, manifold and oriented (or nothing is kept).
    fn capped_half(body: &MeshBody, p: Plane) -> Result<bool, String> {
        let Some(half) = cut(body.mesh(), p, false).negative else {
            return Ok(false);
        };
        let s = section_mesh(
            body,
            &Transform::IDENTITY,
            &SectionPlanes(vec![p]),
            &CapOpts::default(),
            &ctx(),
        )
        .unwrap();
        let cap = &s.caps[0];
        let mut positions = half.positions.clone();
        let mut tris = half.tris.clone();
        let off = positions.len() as u32;
        positions.extend_from_slice(&cap.positions);
        tris.extend(cap.tris.iter().map(|t| t.map(|i| i + off)));
        let mut joined = IndexedMesh::new(positions, tris).unwrap();
        joined.weld(WeldOpts { tolerance: snap() }).unwrap();
        let r = analyze(&joined, &AnalyzeOpts::default(), &ctx()).unwrap();
        if r.is_closed && r.is_manifold && r.is_oriented && cap.open.is_empty() {
            Ok(!cap.tris.is_empty())
        } else {
            Err(format!(
                "closed {} manifold {} oriented {} (boundary edges {}, non-manifold edges {}, \
                 non-manifold vertices {}, open curves {})",
                r.is_closed,
                r.is_manifold,
                r.is_oriented,
                r.boundary_edges,
                r.non_manifold_edges.len(),
                r.non_manifold_vertices.len(),
                cap.open.len()
            ))
        }
    }

    /// Whether MESH-09's own capped half of the same plane is closed and manifold.
    fn mesh09_ok(m: &IndexedMesh, p: Plane) -> bool {
        let Some(h) = cut(m, p, true).negative else {
            return true;
        };
        let r = analyze(&h, &AnalyzeOpts::default(), &ctx()).unwrap();
        r.is_closed && r.is_manifold
    }

    /// Done-when 1: for 20+ solid parts, 3 seeded random planes, a plane exactly through a
    /// vertex, one containing an edge and the three axis planes through the middle of the part:
    /// the kept half plus the cap welds into a closed, manifold, oriented mesh. As in INS-02, a
    /// cut that fails for MESH-09's own capped cut too (a pinch vertex) is reported, not failed.
    #[test]
    fn section_caps_of_solid_parts_are_watertight() {
        let parts = parts();
        let require = std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1");
        let fetched = amg_testkit::testdata("brep/box.brep").is_some();
        if fetched || require {
            assert!(parts.len() >= 20, "only {} solid parts", parts.len());
        }
        let (mut cuts, mut capped, mut exceptions, mut failures) =
            (0usize, 0usize, Vec::new(), Vec::new());
        let (mut triangles, mut tess_s, mut cap_s) = (0usize, 0.0f64, 0.0f64);
        for (n, (name, shape)) in parts.iter().enumerate() {
            let t0 = std::time::Instant::now();
            let body = tessellate_solid(shape, &ctx()).unwrap();
            tess_s += t0.elapsed().as_secs_f64();
            let m = body.mesh();
            triangles += m.tris.len();
            eprintln!(
                "  {name}: {} triangles, tessellated in {:.3} s",
                m.tris.len(),
                t0.elapsed().as_secs_f64()
            );
            let r = analyze(m, &AnalyzeOpts::default(), &ctx()).unwrap();
            assert!(
                r.is_closed && r.is_manifold && r.is_oriented && !r.is_inverted,
                "{name}: the tessellation of a solid is watertight and outward"
            );
            let bb = m.aabb();
            let mut rng = Rng(0x1115_0000 ^ (n as u64 + 1) << 20 ^ m.tris.len() as u64);
            let mut planes = Vec::new();
            for _ in 0..3 {
                let o = bb.min + (bb.max - bb.min) * Vec3::new(rng.next(), rng.next(), rng.next());
                planes.push(Plane::from_normal(o, Dir3::new_unchecked(rng.dir())));
            }
            let t = m.tris[(rng.next() * m.tris.len() as f64) as usize % m.tris.len()];
            let (a, b) = (m.positions[t[0] as usize], m.positions[t[1] as usize]);
            planes.push(Plane::from_normal(a, Dir3::new_unchecked(rng.dir())));
            let d = (b - a).normalize();
            let normal = d.cross(rng.dir()).normalize();
            planes.push(Plane::from_normal(a, Dir3::new_unchecked(normal)));
            for axis in [Dir3::X, Dir3::Y, Dir3::Z] {
                planes.push(Plane::from_normal(bb.center(), axis));
            }
            for (k, p) in planes.into_iter().enumerate() {
                cuts += 1;
                let t0 = std::time::Instant::now();
                let result = capped_half(&body, p);
                cap_s += t0.elapsed().as_secs_f64();
                match result {
                    Ok(had_cap) => capped += usize::from(had_cap),
                    Err(e) => {
                        let what = format!("{name}, plane {k}: {e}");
                        if mesh09_ok(m, p) {
                            failures.push(what);
                        } else {
                            exceptions.push(what);
                        }
                    }
                }
            }
        }
        eprintln!(
            "solid section caps: {} parts, {triangles} triangles, {cuts} cuts ({capped} with a \
             cap), {} exceptions (also fail in MESH-09), {} failures; tessellation {tess_s:.2} s, \
             cuts with checks {cap_s:.2} s",
            parts.len(),
            exceptions.len(),
            failures.len()
        );
        for e in &exceptions {
            eprintln!("  exception: {e}");
        }
        assert!(failures.is_empty(), "{failures:#?}");
        assert!(
            exceptions.len() * 50 <= cuts,
            "more than 2 % exceptions: {exceptions:#?}"
        );
        assert!(capped * 2 > cuts, "most cuts go through the part");
    }

    /// Cap areas of analytic solids are within the tessellation's deflection of the exact
    /// values, through a placement too; a surface body gives open curves and no cap.
    #[test]
    fn cap_areas_match_the_analytic_sections() {
        let k = kernel();
        let shape = k
            .make_cylinder(&Plane::XY, 8.0, 20.0, std::f64::consts::TAU)
            .unwrap()
            .shape;
        let body = tessellate_solid(&shape, &ctx()).unwrap();
        let q = section_quality();
        let deflection = (q.linear * shape.bbox().diagonal()).min(q.max_linear_abs.unwrap());
        let moved = Transform::from_translation(Vec3::new(30.0, -4.0, 100.0));
        for (to_world, z) in [(Transform::IDENTITY, 7.0), (moved, 107.0)] {
            let plane = Plane {
                origin: Point3::new(0.0, 0.0, z),
                ..Plane::XY
            };
            let s = section_mesh(
                &body,
                &to_world,
                &SectionPlanes(vec![plane]),
                &CapOpts::default(),
                &ctx(),
            )
            .unwrap();
            let (area, perimeter) = (s.stats[0].area.get(), s.stats[0].perimeter.get());
            let exact = std::f64::consts::PI * 64.0;
            let around = std::f64::consts::TAU * 8.0;
            assert!(
                area <= exact && exact - area <= around * deflection,
                "{area}"
            );
            assert!(
                perimeter <= around && around - perimeter < 0.05,
                "{perimeter}"
            );
            assert_eq!(s.stats[0].loops, 1);
        }
        let Some(path) = amg_testkit::testdata("brep/bspline_patch.brep") else {
            return;
        };
        let patch = Shape::read_text(std::fs::read(path).unwrap().as_slice()).unwrap();
        let body = tessellate_solid(&patch, &ctx()).unwrap();
        let middle = Plane::from_normal(body.mesh().aabb().center(), Dir3::X);
        let s = section_mesh(
            &body,
            &Transform::IDENTITY,
            &SectionPlanes(vec![middle]),
            &CapOpts::default(),
            &ctx(),
        )
        .unwrap();
        assert!(s.stats[0].open_loops >= 1 && s.caps[0].tris.is_empty());
    }

    #[test]
    fn a_cancelled_tessellation_stops() {
        let shape = kernel().make_sphere(&Plane::XY, 5.0).unwrap().shape;
        let token = CancelToken::new();
        token.cancel();
        let ctx = JobCtx::with_token(token);
        assert!(matches!(
            tessellate_solid(&shape, &ctx),
            Err(amg_features::inspect::InspectError::Cancelled)
        ));
    }

    // ---- face bases ------------------------------------------------------------------------

    /// The planar face of `body` at height `z`, facing up.
    fn top_face(doc: &Document, body: BodyId, z: f64) -> EntityRef {
        let shape = brep(doc, body);
        let found: Vec<BrepFace> = shape
            .faces()
            .filter(|&f| {
                matches!(shape.surface(f), Surface::Plane(_))
                    && shape.face_edges(f).iter().all(|&e| {
                        shape
                            .edge_vertices(e)
                            .iter()
                            .all(|&v| (shape.vertex_point(v).z - z).abs() < 1e-9)
                    })
            })
            .collect();
        assert_eq!(found.len(), 1);
        topo(doc, body, TopoKind::Face, found[0].0)
    }

    fn topo(doc: &Document, body: BodyId, kind: TopoKind, index: u32) -> EntityRef {
        EntityRef::Topo(
            naming::topo_ref_at(&**doc.evaluated(), kernel(), body, kind, index)
                .expect("the face is named"),
        )
    }

    #[test]
    fn a_face_base_follows_its_face_and_stops_resolving_when_it_is_deleted() {
        amg_features::sketch::link();
        let mut doc = Document::new("ins11-face");
        let (_, base) = extrude_rects(
            &mut doc,
            &[[0.0, 0.0, 40.0, 30.0]],
            10.0,
            BodyOp::NewBody,
            Vec::new(),
        );
        let body = body0(base);
        // A boss on the top face.
        let (boss_sketch, boss) = extrude_rects(
            &mut doc,
            &[[5.0, 5.0, 15.0, 17.0]],
            16.0,
            BodyOp::Join,
            vec![body],
        );
        assert_eq!(health(&doc, boss), Health::Ok);
        let entity = top_face(&doc, body, 16.0);
        assert!(!is_construction(&entity));
        let found = resolve_base(doc.evaluated(), doc.design(), &entity).unwrap();
        assert!(!found.uncertain);
        assert!((found.plane.origin.z - 16.0).abs() < 1e-9);
        assert!((found.plane.normal.get() - Vec3::Z).length() < 1e-12);

        // The face follows an edit of the boss.
        let higher = Extrude::distance(
            profiles(&doc, boss_sketch),
            "22.5 mm",
            OpTarget {
                op: BodyOp::Join,
                participants: vec![body],
            },
        );
        edit(&mut doc, boss, FeatureBox::new(higher));
        let found = resolve_base(doc.evaluated(), doc.design(), &entity).unwrap();
        assert!((found.plane.origin.z - 22.5).abs() < 1e-9, "{found:?}");

        // Deleting the boss deletes the face.
        doc.apply(Command::DeleteFeatures { ids: vec![boss] })
            .unwrap();
        doc.recompute_blocking(jobs()).unwrap();
        assert_eq!(brep(&doc, body).count(SubKind::Face), 6);
        let lost = resolve_base(doc.evaluated(), doc.design(), &entity);
        assert!(matches!(lost, Err(BaseError::Resolve(_))), "{lost:?}");
    }

    #[test]
    fn a_curved_face_is_not_a_base() {
        let mut doc = Document::new("ins11-curved");
        amg_features::sketch::link();
        let mut f = SketchFeature::new(PlaneInput::OriginXY);
        f.edit_sketch(|s| s.add_circle(Point2::new(0.0, 0.0), 4.0))
            .unwrap();
        let sketch = add(&mut doc, FeatureBox::new(f));
        let circle = Extrude::distance(
            profiles(&doc, sketch),
            "5 mm",
            OpTarget {
                op: BodyOp::NewBody,
                participants: Vec::new(),
            },
        );
        let extrude = add(&mut doc, FeatureBox::new(circle));
        let body = body0(extrude);
        let shape = brep(&doc, body);
        let side = shape
            .faces()
            .find(|&f| matches!(shape.surface(f), Surface::Cylinder { .. }))
            .unwrap();
        let entity = topo(&doc, body, TopoKind::Face, side.0);
        assert_eq!(
            resolve_base(doc.evaluated(), doc.design(), &entity),
            Err(BaseError::NotPlanar)
        );
    }
}
