//! Unit tests of INS-03 (Done-when 1–5 and 7 at the core level; thread determinism and the
//! corpus are in `tests/inspect_props.rs`).

#![allow(clippy::float_cmp, clippy::cast_precision_loss)]

use amg_core::{Density, Mat3, Point3, Quat, Transform, Vec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::{IndexedMesh, MeshBody};
use amg_testkit::shapes;

use super::*;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

/// SplitMix64.
struct Rng(u64);
impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
    fn rotation(&mut self) -> Quat {
        loop {
            let q = Quat::from_xyzw(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            if (0.1..=1.0).contains(&q.length()) {
                return q.normalize();
            }
        }
    }
}

/// Axis-aligned box `[0, dims]` as a body.
fn box_body(dims: Vec3) -> MeshBody {
    box_at(Point3::ZERO, dims)
}

fn box_at(min: Point3, dims: Vec3) -> MeshBody {
    let c = shapes::cube(Point3::ZERO, 1.0);
    let pos = c.positions.iter().map(|p| min + *p * dims).collect();
    MeshBody::new(IndexedMesh::new(pos, c.triangles).unwrap())
}

fn sphere_body(r: f64, sub: u32) -> MeshBody {
    let s = shapes::icosphere(Point3::ZERO, r, sub);
    MeshBody::new(IndexedMesh::new(s.positions, s.triangles).unwrap())
}

fn rho(g_per_cm3: f64) -> Density {
    Density::g_per_cm3(g_per_cm3)
}

fn mat_err(a: &Mat3, b: &Mat3) -> f64 {
    let d = (0..9)
        .map(|i| (a.to_cols_array()[i] - b.to_cols_array()[i]).powi(2))
        .sum::<f64>()
        .sqrt();
    d / b.to_cols_array().iter().map(|x| x * x).sum::<f64>().sqrt()
}

#[test]
fn rotated_translated_box_matches_the_analytic_values() {
    let mut rng = Rng(7);
    let (a, b, c) = (10.0, 20.0, 30.0);
    let body = box_body(Vec3::new(a, b, c));
    let density = rho(2.7);
    let m = density.get() * 6000.0;
    let local_com = Vec3::new(a, b, c) / 2.0;
    let diag = Mat3::from_diagonal(Vec3::new(
        m * (b * b + c * c) / 12.0,
        m * (a * a + c * c) / 12.0,
        m * (a * a + b * b) / 12.0,
    ));
    for _ in 0..20 {
        let q = rng.rotation();
        let t = Transform::from_rotation_translation(
            q,
            Vec3::new(
                rng.range(-500.0, 500.0),
                rng.range(-500.0, 500.0),
                rng.range(-500.0, 500.0),
            ),
        );
        let p = mesh_mass_props(&body, &t, density, &ctx()).unwrap();
        assert!((p.volume.unwrap().get() - 6000.0).abs() <= 1e-9);
        assert!((p.area.get() - 2.0 * (200.0 + 300.0 + 600.0)).abs() <= 1e-9);
        let com = t.transform_point3(local_com);
        assert!((p.center_of_mass.unwrap() - com).length() <= 1e-9);
        let r = Mat3::from_quat(q);
        let want = r * diag * r.transpose();
        assert!(mat_err(&p.inertia_com.unwrap(), &want) <= 1e-9);
        // About the origin: parallel axis from the CoM.
        let want_o = want + parallel_axis(m, com);
        assert!(mat_err(&p.inertia_origin.unwrap(), &want_o) <= 1e-9);
        // Principal moments are the sorted diagonal; axes are right-handed and orthonormal.
        let mut mom = [diag.x_axis.x, diag.y_axis.y, diag.z_axis.z];
        mom.sort_by(f64::total_cmp);
        let got = p.principal_moments.unwrap();
        for (g, w) in got.iter().zip(mom) {
            assert!((g - w).abs() <= 1e-9 * w);
        }
        let ax = p.principal_axes.unwrap().map(Dir3::get);
        assert!(ax[0].cross(ax[1]).dot(ax[2]) > 1.0 - 1e-12);
        assert!(ax[0].dot(ax[1]).abs() <= 1e-12);
    }
}

#[test]
fn principal_axes_have_the_documented_sign_rule() {
    let p = mesh_mass_props(
        &box_body(Vec3::new(10.0, 20.0, 30.0)),
        &Transform::IDENTITY,
        rho(1.0),
        &ctx(),
    )
    .unwrap();
    // The moment about z is the smallest, then y, then x: the first two axes are +Z and +Y
    // (largest component positive) and the third is Z × Y = −X (right-handed).
    let ax = p.principal_axes.unwrap().map(Dir3::get);
    assert!((ax[0] - Vec3::Z).length() <= 1e-12);
    assert!((ax[1] - Vec3::Y).length() <= 1e-12);
    assert!((ax[2] + Vec3::X).length() <= 1e-12);
}

#[test]
fn icosphere_converges_to_the_analytic_sphere_by_at_least_3_5x_per_level() {
    let r: f64 = 10.0;
    let density = rho(1.0);
    let v_ref = 4.0 / 3.0 * std::f64::consts::PI * r.powi(3);
    let i_ref = 0.4 * density.get() * v_ref * r * r;
    let mut prev: Option<(f64, f64)> = None;
    for sub in 1..=5 {
        let p =
            mesh_mass_props(&sphere_body(r, sub), &Transform::IDENTITY, density, &ctx()).unwrap();
        let ev = (p.volume.unwrap().get() - v_ref).abs() / v_ref;
        let ei = (p.inertia_com.unwrap().x_axis.x - i_ref).abs() / i_ref;
        if let Some((pv, pi)) = prev {
            assert!(pv / ev >= 3.5, "volume level {sub}: {pv} -> {ev}");
            assert!(pi / ei >= 3.5, "inertia level {sub}: {pi} -> {ei}");
        }
        prev = Some((ev, ei));
        assert!(p.center_of_mass.unwrap().length() <= 1e-10);
    }
}

#[test]
fn translating_a_body_changes_inertia_com_by_less_than_1e_9() {
    let body = sphere_body(12.0, 3);
    let density = rho(1.24);
    let a = mesh_mass_props(&body, &Transform::IDENTITY, density, &ctx()).unwrap();
    for shift in [
        Vec3::new(1.0e3, -2.0e3, 3.0e3),
        Vec3::new(-5.0e4, 1.0e4, 7.0e4),
    ] {
        let b =
            mesh_mass_props(&body, &Transform::from_translation(shift), density, &ctx()).unwrap();
        assert!(mat_err(&b.inertia_com.unwrap(), &a.inertia_com.unwrap()) < 1e-9);
        assert!(
            (b.center_of_mass.unwrap() - a.center_of_mass.unwrap() - shift).length() < 1e-9 * 1e5
        );
    }
}

#[test]
fn an_open_mesh_has_no_volume_based_values_but_gives_the_area() {
    let c = shapes::cube(Point3::ZERO, 10.0);
    let mut tris = c.triangles.clone();
    tris.pop();
    let body = MeshBody::new(IndexedMesh::new(c.positions, tris).unwrap());
    let p = mesh_mass_props(&body, &Transform::IDENTITY, rho(1.0), &ctx()).unwrap();
    assert!((p.area.get() - 550.0).abs() <= 1e-9);
    assert!(p.volume.is_none() && p.mass.is_none() && p.center_of_mass.is_none());
    assert!(p.inertia_com.is_none() && p.inertia_origin.is_none());
    assert!(p.principal_moments.is_none() && p.principal_axes.is_none());
    assert!(
        p.not_watertight
            .as_deref()
            .is_some_and(|r| r.contains("not closed"))
    );
}

#[test]
fn an_inverted_mesh_reports_a_reason() {
    let c = shapes::cube(Point3::ZERO, 10.0);
    let tris = c.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect();
    let body = MeshBody::new(IndexedMesh::new(c.positions, tris).unwrap());
    let p = mesh_mass_props(&body, &Transform::IDENTITY, rho(1.0), &ctx()).unwrap();
    assert!(p.volume.is_none());
    assert!(p.not_watertight.unwrap().contains("inverted"));
    assert!((p.area.get() - 600.0).abs() <= 1e-9);
}

#[test]
fn a_mirroring_transform_keeps_the_body_valid() {
    let body = box_body(Vec3::new(10.0, 20.0, 30.0));
    let t = Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0));
    let p = mesh_mass_props(&body, &t, rho(1.0), &ctx()).unwrap();
    assert!((p.volume.unwrap().get() - 6000.0).abs() <= 1e-9);
    assert!((p.center_of_mass.unwrap() - Vec3::new(-5.0, 10.0, 15.0)).length() <= 1e-9);
}

#[test]
fn combine_of_two_halves_equals_the_whole() {
    let density = rho(1.24);
    let t = Transform::from_rotation_translation(
        Quat::from_rotation_z(0.3) * Quat::from_rotation_x(1.1),
        Vec3::new(40.0, -12.0, 7.0),
    );
    let whole =
        mesh_mass_props(&box_body(Vec3::new(30.0, 20.0, 10.0)), &t, density, &ctx()).unwrap();
    let left =
        mesh_mass_props(&box_body(Vec3::new(12.0, 20.0, 10.0)), &t, density, &ctx()).unwrap();
    let right = mesh_mass_props(
        &box_at(Point3::new(12.0, 0.0, 0.0), Vec3::new(18.0, 20.0, 10.0)),
        &t,
        density,
        &ctx(),
    )
    .unwrap();
    let sum = combine(&[left, right]);
    let rel = |a: f64, b: f64| (a - b).abs() / b.abs();
    assert!(rel(sum.volume.unwrap().get(), whole.volume.unwrap().get()) <= 1e-9);
    assert!(rel(sum.mass.unwrap().get(), whole.mass.unwrap().get()) <= 1e-9);
    // Two halves have 2·(200 + 100) extra area at the cut.
    assert!((sum.area.get() - whole.area.get() - 400.0).abs() <= 1e-9);
    assert!(
        (sum.center_of_mass.unwrap() - whole.center_of_mass.unwrap()).length()
            <= 1e-9 * whole.center_of_mass.unwrap().length()
    );
    assert!(mat_err(&sum.inertia_com.unwrap(), &whole.inertia_com.unwrap()) <= 1e-9);
    assert!(mat_err(&sum.inertia_origin.unwrap(), &whole.inertia_origin.unwrap()) <= 1e-9);
    assert!(rel(sum.density.get(), whole.density.get()) <= 1e-12);
    for (a, b) in sum
        .principal_moments
        .unwrap()
        .iter()
        .zip(whole.principal_moments.unwrap())
    {
        assert!(rel(*a, b) <= 1e-9);
    }
}

#[test]
fn combine_edge_cases() {
    let e = combine(&[]);
    assert_eq!(e.area.get(), 0.0);
    assert!(e.volume.is_none() && e.not_watertight.is_some());
    let c = shapes::cube(Point3::ZERO, 10.0);
    let mut tris = c.triangles.clone();
    tris.pop();
    let open = MeshBody::new(IndexedMesh::new(c.positions, tris).unwrap());
    let good = mesh_mass_props(
        &box_body(Vec3::splat(10.0)),
        &Transform::IDENTITY,
        rho(1.0),
        &ctx(),
    )
    .unwrap();
    let bad = mesh_mass_props(&open, &Transform::IDENTITY, rho(1.0), &ctx()).unwrap();
    let both = combine(&[good.clone(), bad.clone()]);
    assert!(both.volume.is_none());
    assert!((both.area.get() - 1150.0).abs() <= 1e-9);
    assert!(both.not_watertight.unwrap().contains("1 of 2"));
    // A single part is returned as it is (area, mass, CoM, inertia).
    let one = combine(std::slice::from_ref(&good));
    assert_eq!(one.volume, good.volume);
    assert!(mat_err(&one.inertia_com.unwrap(), &good.inertia_com.unwrap()) <= 1e-12);
}

#[test]
fn a_1000_mm3_pla_cube_weighs_1_24_g() {
    let p = mesh_mass_props(
        &box_body(Vec3::splat(10.0)),
        &Transform::IDENTITY,
        rho(1.24),
        &ctx(),
    )
    .unwrap();
    assert!((p.mass.unwrap().to_g() - 1.24).abs() <= 1e-12);
    assert!((p.mass.unwrap().get() - 1.24e-6).abs() <= 1e-18);
}

#[test]
fn bad_input_and_cancellation() {
    let body = box_body(Vec3::splat(1.0));
    let nan = Transform::from_translation(Vec3::new(f64::NAN, 0.0, 0.0));
    assert!(matches!(
        mesh_mass_props(&body, &nan, rho(1.0), &ctx()),
        Err(InspectError::Degenerate(_))
    ));
    let token = CancelToken::new();
    token.cancel();
    let cancelled = JobCtx::with_token(token);
    assert!(matches!(
        mesh_mass_props(
            &sphere_body(1.0, 5),
            &Transform::IDENTITY,
            rho(1.0),
            &cancelled
        ),
        Err(InspectError::Cancelled)
    ));
}

#[test]
fn principal_handles_degenerate_tensors() {
    // A cube (and a sphere) have three equal moments: any frame is valid, but it must be
    // orthonormal, right-handed and deterministic.
    let p = mesh_mass_props(
        &box_body(Vec3::splat(10.0)),
        &Transform::from_rotation_translation(Quat::from_rotation_y(0.4), Vec3::ONE),
        rho(1.0),
        &ctx(),
    )
    .unwrap();
    let m = p.principal_moments.unwrap();
    assert!((m[0] - m[2]).abs() <= 1e-9 * m[2]);
    let ax = p.principal_axes.unwrap().map(Dir3::get);
    assert!(ax[0].cross(ax[1]).dot(ax[2]) > 1.0 - 1e-12);
    let again = mesh_mass_props(
        &box_body(Vec3::splat(10.0)),
        &Transform::from_rotation_translation(Quat::from_rotation_y(0.4), Vec3::ONE),
        rho(1.0),
        &ctx(),
    )
    .unwrap();
    assert_eq!(p, again);
}
