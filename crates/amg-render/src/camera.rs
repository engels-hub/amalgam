//! The f64 camera and its relative-to-eye (RTE) GPU uniform (design note §3).

use amg_core::glam::DVec4;
use amg_core::{Dir3, Mat4, Point3, Ray3, Vec3};

/// Nearest near-plane distance a perspective camera uses, mm (design note §3).
const MIN_NEAR_MM: f64 = 1e-4;
/// Near plane as a fraction of the eye-to-target distance (design note §3).
const NEAR_FRACTION: f64 = 1e-3;
/// Orthographic far distance as a multiple of the eye-to-target distance. RND-11 replaces this
/// with the near-plane policy based on the visible scene bounds.
const ORTHO_DEPTH_FACTOR: f64 = 1e3;

/// A viewpoint in world space, mm. World up is +Z by convention (spec §Design 10).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Camera {
    /// Eye position.
    pub eye: Point3,
    /// Point looked at; also the pivot for the near-plane distance.
    pub target: Point3,
    /// Approximate up direction.
    pub up: Dir3,
    /// Lens.
    pub projection: Projection,
}

/// The camera lens.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Projection {
    /// Perspective with an infinite far plane.
    Perspective {
        /// Vertical field of view, radians.
        fov_y: f64,
    },
    /// Orthographic.
    Orthographic {
        /// Height of the view volume, mm.
        height: f64,
    },
}

/// GPU camera block; the layout is fixed in `shaders/camera.wgsl` (design note §14).
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    /// World-to-view rotation (the eye is the origin of RTE space).
    pub view_rte: [[f32; 4]; 4],
    /// View-to-clip, reversed-Z.
    pub proj: [[f32; 4]; 4],
    /// Clip-to-RTE-world.
    pub inv_view_proj_rte: [[f32; 4]; 4],
    /// High part of the eye position, `w` unused.
    pub eye_hi: [f32; 4],
    /// Low part of the eye position, `w` unused.
    pub eye_lo: [f32; 4],
    /// Target size in pixels.
    pub size_px: [f32; 2],
    /// World size of one pixel at 1 mm depth (the orthographic pixel size if `ortho != 0`).
    pub pixel_size_at_1mm: f32,
    /// Non-zero for an orthographic camera.
    pub ortho: u32,
}

impl Camera {
    /// Unit viewing direction.
    pub fn forward(&self) -> Dir3 {
        Dir3::new_unchecked((self.target - self.eye).normalize())
    }

    /// Ray through pixel `px` (origin top-left, y down, pixel centres at `.5`) for a view of
    /// `size_px`.
    pub fn ray(&self, px: [f64; 2], size_px: [u32; 2]) -> Ray3 {
        let (right, up, forward) = self.basis();
        let (x, y) = ndc_xy(px, size_px);
        match self.projection {
            Projection::Perspective { fov_y } => {
                let half_h = (fov_y * 0.5).tan();
                let half_w = half_h * aspect(size_px);
                let dir = forward + right * (x * half_w) + up * (y * half_h);
                Ray3 {
                    origin: self.eye,
                    dir: Dir3::new_unchecked(dir.normalize()),
                }
            }
            Projection::Orthographic { height } => {
                let half_h = height * 0.5;
                let half_w = half_h * aspect(size_px);
                Ray3 {
                    origin: self.eye + right * (x * half_w) + up * (y * half_h),
                    dir: Dir3::new_unchecked(forward),
                }
            }
        }
    }

    /// Pixel position (origin top-left, y down) and reversed-Z depth; `None` if `p` is at or
    /// behind the eye (perspective only).
    pub fn project(&self, p: Point3, size_px: [u32; 2]) -> Option<[f64; 3]> {
        // In eye-relative f64, so a part far from the world origin loses no digits.
        let view = self.view_rotation().transform_point3(p - self.eye);
        if matches!(self.projection, Projection::Perspective { .. }) && view.z >= 0.0 {
            return None;
        }
        let clip = self.proj_matrix(size_px) * view.extend(1.0);
        let ndc = clip / clip.w;
        Some([
            (ndc.x + 1.0) * 0.5 * f64::from(size_px[0]),
            (1.0 - ndc.y) * 0.5 * f64::from(size_px[1]),
            ndc.z,
        ])
    }

    /// World point at pixel `px` and reversed-Z `depth`. The result is f64, but the f32 `depth`
    /// limits it to about 1e-7 of the distance (design note §3).
    pub fn unproject(&self, px: [f64; 2], depth: f32, size_px: [u32; 2]) -> Point3 {
        self.unproject_f64(px, f64::from(depth), size_px)
    }

    /// [`Camera::unproject`] with an f64 depth.
    fn unproject_f64(&self, px: [f64; 2], depth: f64, size_px: [u32; 2]) -> Point3 {
        let (x, y) = ndc_xy(px, size_px);
        let view = self
            .proj_matrix(size_px)
            .inverse()
            .project_point3(Vec3::new(x, y, depth));
        self.eye + self.view_rotation().inverse().transform_point3(view)
    }

    /// World size of one pixel at the depth of `p`, mm/px.
    pub fn pixel_size_at(&self, p: Point3, size_px: [u32; 2]) -> f64 {
        let h = f64::from(size_px[1].max(1));
        match self.projection {
            Projection::Perspective { fov_y } => {
                let depth = (p - self.eye).dot(self.forward().get());
                2.0 * (fov_y * 0.5).tan() * depth.abs() / h
            }
            Projection::Orthographic { height } => height / h,
        }
    }

    /// The default near-plane distance: 1e-3 of the eye-to-target distance, at least 1e-4 mm.
    pub fn default_near(&self) -> f64 {
        (self.eye.distance(self.target) * NEAR_FRACTION).max(MIN_NEAR_MM)
    }

    /// The RTE uniform for a view of `size_px` with near-plane distance `near`.
    pub fn uniform(&self, size_px: [u32; 2], near: f64) -> CameraUniform {
        let rotation = self.view_rotation();
        let proj = self.proj_with_near(size_px, near);
        let hi = self.eye.as_vec3();
        let lo = (self.eye - hi.as_dvec3()).as_vec3();
        let h = f64::from(size_px[1].max(1));
        let pixel_size_at_1mm = match self.projection {
            Projection::Perspective { fov_y } => 2.0 * (fov_y * 0.5).tan() / h,
            Projection::Orthographic { height } => height / h,
        };
        CameraUniform {
            view_rte: rotation.as_mat4().to_cols_array_2d(),
            proj: proj.as_mat4().to_cols_array_2d(),
            inv_view_proj_rte: (proj * rotation).inverse().as_mat4().to_cols_array_2d(),
            eye_hi: hi.extend(0.0).to_array(),
            eye_lo: lo.extend(0.0).to_array(),
            size_px: [size_px[0] as f32, size_px[1] as f32],
            pixel_size_at_1mm: pixel_size_at_1mm as f32,
            ortho: u32::from(matches!(self.projection, Projection::Orthographic { .. })),
        }
    }

    /// f64 world-to-view matrix.
    pub fn view_matrix(&self) -> Mat4 {
        self.view_rotation() * Mat4::from_translation(-self.eye)
    }

    /// f64 view-to-clip matrix (reversed-Z, depth in `[0, 1]`) with [`Camera::default_near`].
    pub fn proj_matrix(&self, size_px: [u32; 2]) -> Mat4 {
        self.proj_with_near(size_px, self.default_near())
    }

    fn proj_with_near(&self, size_px: [u32; 2], near: f64) -> Mat4 {
        let aspect = aspect(size_px);
        match self.projection {
            Projection::Perspective { fov_y } => {
                // Infinite far plane, reversed-Z: depth = near / -z_view.
                let f = 1.0 / (fov_y * 0.5).tan();
                Mat4::from_cols(
                    DVec4::new(f / aspect, 0.0, 0.0, 0.0),
                    DVec4::new(0.0, f, 0.0, 0.0),
                    DVec4::new(0.0, 0.0, 0.0, -1.0),
                    DVec4::new(0.0, 0.0, near, 0.0),
                )
            }
            Projection::Orthographic { height } => {
                let half_h = height * 0.5;
                let half_w = half_h * aspect;
                let dist = self.eye.distance(self.target);
                let far_plane = dist * ORTHO_DEPTH_FACTOR + height;
                // Negative: geometry between the eye and this plane is still visible.
                let near_plane = -dist.max(height);
                // Reversed-Z: depth = (z_view + far) / (far - near), so near -> 1 and far -> 0.
                let z = 1.0 / (far_plane - near_plane);
                Mat4::from_cols(
                    DVec4::new(1.0 / half_w, 0.0, 0.0, 0.0),
                    DVec4::new(0.0, 1.0 / half_h, 0.0, 0.0),
                    DVec4::new(0.0, 0.0, z, 0.0),
                    DVec4::new(0.0, 0.0, far_plane * z, 1.0),
                )
            }
        }
    }

    /// World-to-view with the translation removed; the eye is the origin of RTE space.
    fn view_rotation(&self) -> Mat4 {
        let (right, up, forward) = self.basis();
        Mat4::from_cols(
            DVec4::new(right.x, up.x, -forward.x, 0.0),
            DVec4::new(right.y, up.y, -forward.y, 0.0),
            DVec4::new(right.z, up.z, -forward.z, 0.0),
            DVec4::W,
        )
    }

    /// World-space right, up and forward unit vectors.
    fn basis(&self) -> (Vec3, Vec3, Vec3) {
        let forward = self.forward().get();
        let right = forward.cross(self.up.get()).normalize();
        (right, right.cross(forward), forward)
    }
}

fn aspect(size_px: [u32; 2]) -> f64 {
    f64::from(size_px[0].max(1)) / f64::from(size_px[1].max(1))
}

fn ndc_xy(px: [f64; 2], size_px: [u32; 2]) -> (f64, f64) {
    (
        px[0] / f64::from(size_px[0].max(1)) * 2.0 - 1.0,
        1.0 - px[1] / f64::from(size_px[1].max(1)) * 2.0,
    )
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "the RTE uniforms must match bit for bit")]
mod tests {
    use super::*;

    const SIZE: [u32; 2] = [1920, 1080];

    fn cameras() -> Vec<Camera> {
        let far = Point3::new(1e5, -2e5, 3e4);
        let mut out = Vec::new();
        for origin in [Point3::ZERO, far] {
            for projection in [
                Projection::Perspective { fov_y: 0.8 },
                Projection::Orthographic { height: 120.0 },
            ] {
                out.push(Camera {
                    eye: origin + Vec3::new(120.0, -90.0, 60.0),
                    target: origin,
                    up: Dir3::Z,
                    projection,
                });
            }
        }
        out
    }

    #[test]
    fn project_unproject_round_trip() {
        for cam in cameras() {
            let dist = cam.eye.distance(cam.target);
            for p in [
                cam.target,
                cam.target + Vec3::new(10.0, 5.0, -3.0),
                cam.target + Vec3::new(-20.0, 15.0, 8.0),
            ] {
                let [x, y, depth] = cam.project(p, SIZE).unwrap();
                let back = cam.unproject_f64([x, y], depth, SIZE);
                assert!(
                    back.distance(p) <= 1e-9 * dist,
                    "{cam:?}: {p} -> {back}, error {}",
                    back.distance(p)
                );
                // The public f32-depth path keeps the pixel, and the point to the depth ulp.
                let back32 = cam.unproject([x, y], depth as f32, SIZE);
                // Orthographic depth is linear over a range of ORTHO_DEPTH_FACTOR * dist, so its
                // f32 step scales with that range until RND-11 fits it to the scene bounds.
                let range = match cam.projection {
                    Projection::Perspective { .. } => 1.0,
                    Projection::Orthographic { .. } => ORTHO_DEPTH_FACTOR,
                };
                assert!(back32.distance(p) <= 1e-6 * dist * range);
                let again = cam.project(back32, SIZE).unwrap();
                assert!((again[0] - x).abs() <= 1e-6 && (again[1] - y).abs() <= 1e-6);
            }
        }
    }

    #[test]
    fn centre_ray_is_the_view_axis() {
        for cam in cameras() {
            let ray = cam.ray([960.0, 540.0], SIZE);
            assert!(ray.dir.get().distance(cam.forward().get()) < 1e-12);
            let hit = ray.at(cam.eye.distance(cam.target));
            assert!(hit.distance(cam.target) < 1e-9 * cam.eye.distance(cam.target));
        }
    }

    #[test]
    fn rays_pass_through_their_pixels() {
        for cam in cameras() {
            for px in [[0.5, 0.5], [1919.5, 0.5], [100.25, 1000.75]] {
                let p = cam.ray(px, SIZE).at(cam.eye.distance(cam.target));
                let q = cam.project(p, SIZE).unwrap();
                assert!(
                    (q[0] - px[0]).abs() < 1e-6 && (q[1] - px[1]).abs() < 1e-6,
                    "{px:?}"
                );
            }
        }
    }

    #[test]
    fn behind_the_eye_projects_to_none() {
        let cam = cameras()[0];
        assert!(cam.project(cam.eye - cam.forward().get(), SIZE).is_none());
    }

    #[test]
    fn reversed_z_near_is_one_and_far_tends_to_zero() {
        let cam = cameras()[0];
        let near = cam.eye + cam.forward().get() * cam.default_near();
        assert!((cam.project(near, SIZE).unwrap()[2] - 1.0).abs() < 1e-9);
        let far = cam.eye + cam.forward().get() * 1e9;
        assert!(cam.project(far, SIZE).unwrap()[2] < 1e-6);
    }

    #[test]
    fn orthographic_depth_is_reversed() {
        let cam = cameras()[1];
        let d = cam.eye.distance(cam.target);
        let closer = cam.eye + cam.forward().get() * (d * 0.5);
        let farther = cam.eye + cam.forward().get() * (d * 2.0);
        let (a, b) = (
            cam.project(closer, SIZE).unwrap()[2],
            cam.project(farther, SIZE).unwrap()[2],
        );
        assert!(a > b && (0.0..=1.0).contains(&a) && (0.0..=1.0).contains(&b));
    }

    #[test]
    fn pixel_size_matches_the_projection() {
        for cam in cameras() {
            let p = cam.target;
            let step = cam.pixel_size_at(p, SIZE);
            let a = cam.project(p, SIZE).unwrap();
            let (right, _, _) = cam.basis();
            let b = cam.project(p + right * step, SIZE).unwrap();
            assert!(((b[0] - a[0]) - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn rte_view_drops_the_translation_and_keeps_the_pose() {
        let cams = cameras();
        // Same relative pose at the origin and far away.
        let (near, far) = (cams[0].uniform(SIZE, 0.1), cams[2].uniform(SIZE, 0.1));
        assert_eq!(near.view_rte, far.view_rte);
        assert_eq!(near.proj, far.proj);
        assert_eq!(near.view_rte[3], [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn eye_high_and_low_parts_sum_to_the_eye() {
        let cam = cameras()[2];
        let u = cam.uniform(SIZE, 0.1);
        for i in 0..3 {
            let sum = f64::from(u.eye_hi[i]) + f64::from(u.eye_lo[i]);
            assert!((sum - cam.eye.to_array()[i]).abs() < 1e-6);
        }
    }

    #[test]
    fn uniform_is_240_bytes() {
        assert_eq!(size_of::<CameraUniform>(), 240);
    }

    /// The offsets `shaders/camera.wgsl` documents and IMP-05 copies (WGSL alignment rules:
    /// `mat4x4` 64, `vec4` 16, `vec2` 8, scalars 4).
    #[test]
    fn uniform_field_offsets_match_the_wgsl_struct() {
        use std::mem::offset_of;
        assert_eq!(offset_of!(CameraUniform, view_rte), 0);
        assert_eq!(offset_of!(CameraUniform, proj), 64);
        assert_eq!(offset_of!(CameraUniform, inv_view_proj_rte), 128);
        assert_eq!(offset_of!(CameraUniform, eye_hi), 192);
        assert_eq!(offset_of!(CameraUniform, eye_lo), 208);
        assert_eq!(offset_of!(CameraUniform, size_px), 224);
        assert_eq!(offset_of!(CameraUniform, pixel_size_at_1mm), 232);
        assert_eq!(offset_of!(CameraUniform, ortho), 236);
        let wgsl = include_str!("shaders/camera.wgsl");
        for field in [
            "view_rte: mat4x4<f32>",
            "proj: mat4x4<f32>",
            "inv_view_proj_rte: mat4x4<f32>",
            "eye_hi: vec4<f32>",
            "eye_lo: vec4<f32>",
            "size_px: vec2<f32>",
            "pixel_size_at_1mm: f32",
            "ortho: u32",
        ] {
            assert!(wgsl.contains(field), "camera.wgsl lacks `{field}`");
        }
    }
}
