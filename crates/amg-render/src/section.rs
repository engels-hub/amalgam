//! Section planes (RND-06, design note §10): the public [`SectionPlane`] and [`Hatch`], the
//! per-view GPU uniform, the CPU classification of instances against the planes, and the f64
//! cap point of a ray ([`section_cap_point`], the CPU twin of `section_cap` in
//! `shaders/section.wgsl`, for picking).
//!
//! A plane keeps the side opposite its normal: a point `p` is clipped when
//! `normal · (p − origin) > 0` for any active plane, so several planes keep the intersection of
//! their kept half-spaces (a box section with six).

use amg_core::{Plane, Point3, Ray3, Vec3};

use crate::camera::Camera;

/// Planes a view applies at once (QUESTIONS #29). Further enabled planes are ignored, in the
/// order they were added.
pub const MAX_SECTIONS: usize = 6;

/// Hatching drawn over section caps: parallel lines (or a cross pattern) with a constant
/// spacing on screen.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hatch {
    /// Angle of the lines from the plane's `x` axis, radians.
    pub angle: f64,
    /// Distance between neighbouring lines, pixels (at the depth of the camera target's
    /// projection onto the plane; exact for a plane facing the camera).
    pub spacing_px: f32,
    /// Linear RGBA of the lines; alpha 0 draws no hatch.
    pub color: [f32; 4],
    /// A second set of lines at `angle + 90°`.
    pub cross: bool,
}

impl Hatch {
    /// No hatching (plain cap colour).
    pub const NONE: Hatch = Hatch {
        angle: 0.0,
        spacing_px: 8.0,
        color: [0.0; 4],
        cross: false,
    };
}

impl Default for Hatch {
    /// 45° lines every 8 px, dark and slightly translucent.
    fn default() -> Self {
        Hatch {
            angle: std::f64::consts::FRAC_PI_4,
            spacing_px: 8.0,
            color: [0.02, 0.02, 0.025, 0.85],
            cross: false,
        }
    }
}

/// A section plane of the [`crate::RenderScene`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SectionPlane {
    /// The cut. Geometry on the side `plane.normal` points to is removed; the plane's `x` and
    /// `y` axes orient the hatch.
    pub plane: Plane,
    /// Draw caps where closed instances are cut by this plane. Without caps their inside (back
    /// faces) is shown shaded.
    pub caps: bool,
    /// Hatching of the caps.
    pub hatch: Hatch,
    /// Disabled planes are kept but clip nothing.
    pub enabled: bool,
}

impl SectionPlane {
    /// An enabled plane with caps and the default hatch.
    pub fn new(plane: Plane) -> SectionPlane {
        SectionPlane {
            plane,
            caps: true,
            hatch: Hatch::default(),
            enabled: true,
        }
    }

    /// Signed distance of `p` past the plane, mm: positive on the removed side.
    pub fn distance(&self, p: Point3) -> f64 {
        self.plane.normal.get().dot(p - self.plane.origin)
    }
}

/// Where the ray enters the kept region of `planes` (the intersection of their kept
/// half-spaces), in f64: `Some((index into planes, point on that plane))` when the ray starts on
/// the removed side of at least one plane and reaches the kept region; `None` when it starts
/// inside the kept region or never reaches it.
///
/// This is the cap point of design note §10: when the ray then hits the back face of a closed
/// body, that point lies on the body's cap. Picking (RND-04) uses it for the world point of a
/// `CAP_SUB` hit, so the point is exactly on the plane up to f64 rounding.
pub fn section_cap_point(planes: &[SectionPlane], ray: &Ray3) -> Option<(usize, Point3)> {
    let v = ray.dir.get();
    let mut best: Option<(usize, f64)> = None;
    for (i, s) in planes.iter().enumerate() {
        let so = s.distance(ray.origin);
        if so <= 0.0 {
            continue;
        }
        let nv = s.plane.normal.get().dot(v);
        if nv >= 0.0 {
            // Starts on the removed side and moves away: never enters.
            return None;
        }
        let t = -so / nv;
        if best.is_none_or(|(_, bt)| t > bt) {
            best = Some((i, t));
        }
    }
    let (i, t) = best?;
    let p = ray.at(t);
    // The entry point must be kept by every other plane.
    let kept = planes
        .iter()
        .enumerate()
        .all(|(j, s)| j == i || s.distance(p) <= 1e-9 * (1.0 + t.abs()));
    if !kept {
        return None;
    }
    // Project onto the plane so the point is on it to f64 rounding of the projection.
    let s = &planes[i];
    let n = s.plane.normal.get();
    Some((i, p - n * s.distance(p)))
}

/// One plane of [`SectionUniform`] (`SectionPlaneGpu` in `shaders/section.wgsl`), 80 bytes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct SectionPlaneGpu {
    /// Unit normal (world axes) and `d`: an eye-relative point `x` is clipped when `n·x > d`.
    pub plane: [f32; 4],
    /// Hatch line coordinate `u = dot(x, xyz) + w` in periods (direction across the lines over
    /// the spacing in mm, and the phase at the eye).
    pub hatch_u: [f32; 4],
    /// The crossing lines (zero without `cross`).
    pub hatch_v: [f32; 4],
    /// Hatch colour, linear RGBA.
    pub color: [f32; 4],
    /// `x` = caps (0/1), `y` = cross (0/1).
    pub params: [f32; 4],
}

/// Per-view section uniform (`SectionUniform` in `shaders/section.wgsl`), 496 bytes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct SectionUniform {
    /// `x` = number of active planes.
    pub info: [u32; 4],
    pub planes: [SectionPlaneGpu; MAX_SECTIONS],
}

/// Bytes of [`SectionUniform`].
pub(crate) const SECTION_UNIFORM_BYTES: u64 = size_of::<SectionUniform>() as u64;

/// Hatch coordinate across lines at `angle` in `plane`: `(direction / spacing_mm, phase at the
/// eye)`, the phase reduced to `[0, 1)` in f64.
fn hatch_axis(plane: &Plane, angle: f64, spacing_mm: f64, eye: Point3) -> [f32; 4] {
    // Lines run along `cos a · x + sin a · y`; the coordinate grows across them.
    let across = plane.y.get() * angle.cos() - plane.x.get() * angle.sin();
    let scaled = across / spacing_mm;
    let phase = scaled.dot(eye - plane.origin);
    let phase = phase - phase.floor();
    [
        scaled.x as f32,
        scaled.y as f32,
        scaled.z as f32,
        phase as f32,
    ]
}

impl SectionUniform {
    /// The uniform of `planes` (already the active ones, at most [`MAX_SECTIONS`]) for a view
    /// with `camera` and `size_px`. The hatch spacing in mm is the requested pixel spacing at
    /// the camera target's projection onto each plane.
    pub fn new(planes: &[SectionPlane], camera: &Camera, size_px: [u32; 2]) -> SectionUniform {
        let mut out = SectionUniform::default();
        let eye = camera.eye;
        for (gpu, s) in out.planes.iter_mut().zip(planes.iter().take(MAX_SECTIONS)) {
            let n = s.plane.normal.get();
            let d = n.dot(s.plane.origin - eye);
            let reference = camera.target - n * s.distance(camera.target);
            let spacing_mm = f64::from(s.hatch.spacing_px.max(1.0))
                * camera.pixel_size_at(reference, size_px).max(1e-300);
            let h = s.hatch;
            gpu.plane = [n.x as f32, n.y as f32, n.z as f32, d as f32];
            gpu.hatch_u = hatch_axis(&s.plane, h.angle, spacing_mm, eye);
            gpu.hatch_v = if h.cross {
                hatch_axis(
                    &s.plane,
                    h.angle + std::f64::consts::FRAC_PI_2,
                    spacing_mm,
                    eye,
                )
            } else {
                [0.0; 4]
            };
            gpu.color = h.color;
            gpu.params = [
                f32::from(u8::from(s.caps)),
                f32::from(u8::from(h.cross)),
                0.0,
                0.0,
            ];
        }
        out.info[0] = planes.len().min(MAX_SECTIONS) as u32;
        out
    }
}

/// Where an instance's bounding sphere lies relative to the active planes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SectionState {
    /// Entirely kept: drawn without clipping.
    Kept,
    /// Crosses at least one plane and is not entirely removed: drawn clipped (and capped).
    Cut,
    /// Entirely on the removed side of a plane: not drawn.
    Removed,
}

/// Classifies a world bounding sphere against `planes`.
pub(crate) fn classify(planes: &[SectionPlane], center: Vec3, radius: f64) -> SectionState {
    let mut cut = false;
    for s in planes {
        let d = s.distance(center);
        if d > radius {
            return SectionState::Removed;
        }
        if d > -radius {
            cut = true;
        }
    }
    if cut {
        SectionState::Cut
    } else {
        SectionState::Kept
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
#[allow(
    clippy::float_cmp,
    reason = "exactly representable plane and flag values"
)]
mod tests {
    use amg_core::{Dir3, Tolerance};

    use super::*;
    use crate::Projection;

    fn plane(origin: Point3, normal: Vec3) -> SectionPlane {
        SectionPlane::new(Plane::from_normal(
            origin,
            Dir3::new(normal, &Tolerance::default()).unwrap(),
        ))
    }

    #[test]
    fn uniform_is_496_bytes_and_matches_the_wgsl_struct() {
        assert_eq!(size_of::<SectionPlaneGpu>(), 80);
        assert_eq!(size_of::<SectionUniform>(), 496);
        let wgsl = include_str!("shaders/section.wgsl");
        for field in [
            "plane: vec4<f32>",
            "hatch_u: vec4<f32>",
            "hatch_v: vec4<f32>",
            "color: vec4<f32>",
            "params: vec4<f32>",
            "info: vec4<u32>",
            "planes: array<SectionPlaneGpu, 6>",
        ] {
            assert!(wgsl.contains(field), "{field}");
        }
        assert!(wgsl.contains(&format!("const MAX_SECTIONS: u32 = {MAX_SECTIONS}u;")));
    }

    #[test]
    fn classify_spheres() {
        let p = [plane(Point3::ZERO, Vec3::Z)];
        assert_eq!(
            classify(&p, Vec3::new(0.0, 0.0, -5.0), 1.0),
            SectionState::Kept
        );
        assert_eq!(
            classify(&p, Vec3::new(0.0, 0.0, 5.0), 1.0),
            SectionState::Removed
        );
        assert_eq!(classify(&p, Vec3::ZERO, 1.0), SectionState::Cut);
        assert_eq!(classify(&[], Vec3::ZERO, 1.0), SectionState::Kept);
        // Removed by one plane wins over cut by another.
        let two = [plane(Point3::ZERO, Vec3::X), plane(Point3::ZERO, Vec3::Z)];
        assert_eq!(
            classify(&two, Vec3::new(0.0, 0.0, 9.0), 2.0),
            SectionState::Removed
        );
    }

    #[test]
    fn cap_point_is_on_the_entry_plane() {
        // Kept: x <= 0 and z <= 0.
        let planes = [
            plane(Point3::new(0.0, 0.0, 0.0), Vec3::X),
            plane(Point3::new(0.0, 0.0, 0.0), Vec3::Z),
        ];
        let tol = Tolerance::default();
        let ray = |o: Point3, d: Vec3| Ray3 {
            origin: o,
            dir: Dir3::new(d, &tol).unwrap(),
        };
        // From above, straight down at x = -3: enters through z = 0.
        let (i, p) =
            section_cap_point(&planes, &ray(Point3::new(-3.0, 1.0, 50.0), -Vec3::Z)).unwrap();
        assert_eq!(i, 1);
        assert!((p - Point3::new(-3.0, 1.0, 0.0)).length() < 1e-12);
        // Diagonal: enters through whichever plane it crosses last.
        let (i, p) = section_cap_point(
            &planes,
            &ray(Point3::new(10.0, 0.0, 1.0), Vec3::new(-1.0, 0.0, -0.5)),
        )
        .unwrap();
        assert_eq!(i, 0);
        assert!(planes[0].distance(p).abs() < 1e-12);
        assert!(planes[1].distance(p) <= 0.0);
        // Already inside: no cap point.
        assert!(section_cap_point(&planes, &ray(Point3::new(-1.0, 0.0, -1.0), -Vec3::Z)).is_none());
        // On the removed side moving away: none.
        assert!(section_cap_point(&planes, &ray(Point3::new(-1.0, 0.0, 5.0), Vec3::Z)).is_none());
        // Far from the origin, still on the plane to f64 rounding (≤ 1e-6 mm).
        let far = [plane(Point3::new(1e5, 2e5, 3e4), Vec3::new(1.0, 2.0, 3.0))];
        let (_, p) = section_cap_point(
            &far,
            &ray(Point3::new(2e5, 3e5, 1e5), Vec3::new(-0.3, -0.5, -0.8)),
        )
        .unwrap();
        assert!(far[0].distance(p).abs() < 1e-6, "{}", far[0].distance(p));
    }

    #[test]
    fn uniform_encodes_planes_relative_to_the_eye_and_hatch_in_pixels() {
        let cam = Camera {
            eye: Point3::new(0.0, 0.0, 100.0),
            target: Point3::ZERO,
            up: Dir3::Y,
            projection: Projection::Orthographic { height: 64.0 },
        };
        let size = [256, 256];
        let s = SectionPlane {
            hatch: Hatch {
                angle: 0.0,
                spacing_px: 8.0,
                color: [1.0; 4],
                cross: true,
            },
            ..plane(Point3::new(0.0, 0.0, 10.0), Vec3::Z)
        };
        let u = SectionUniform::new(&[s], &cam, size);
        assert_eq!(u.info[0], 1);
        let g = u.planes[0];
        // n = +Z, d = n·(origin − eye) = 10 − 100.
        assert_eq!(g.plane, [0.0, 0.0, 1.0, -90.0]);
        // 0.25 mm/px → 2 mm spacing: |hatch_u| = 0.5 periods per mm, in the plane.
        let hu = Vec3::new(
            f64::from(g.hatch_u[0]),
            f64::from(g.hatch_u[1]),
            f64::from(g.hatch_u[2]),
        );
        assert!((hu.length() - 0.5).abs() < 1e-6);
        assert!(hu.z.abs() < 1e-9);
        assert!((0.0..1.0).contains(&g.hatch_u[3]));
        let hv = Vec3::new(
            f64::from(g.hatch_v[0]),
            f64::from(g.hatch_v[1]),
            f64::from(g.hatch_v[2]),
        );
        assert!(hu.dot(hv).abs() < 1e-9 && (hv.length() - 0.5).abs() < 1e-6);
        assert_eq!(g.params, [1.0, 1.0, 0.0, 0.0]);
        // Only MAX_SECTIONS planes are encoded.
        let many = vec![s; 9];
        assert_eq!(SectionUniform::new(&many, &cam, size).info[0], 6);
    }
}
