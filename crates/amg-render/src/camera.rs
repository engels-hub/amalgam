//! The f64 camera and its relative-to-eye (RTE) GPU uniform (design note §3).

use amg_core::glam::DVec4;
use amg_core::{Aabb3, Dir3, Mat4, Point3, Ray3, Vec3};

/// Nearest near-plane distance a perspective camera uses, mm (design note §3).
const MIN_NEAR_MM: f64 = 1e-4;
/// Near plane as a fraction of the eye-to-target distance (design note §3).
const NEAR_FRACTION: f64 = 1e-3;
/// Orthographic far distance as a multiple of the eye-to-target distance when the scene extent
/// is unknown ([`Camera::default_depth_range`]); [`Camera::depth_range_for`] fits it instead.
const ORTHO_DEPTH_FACTOR: f64 = 1e3;
/// Relative margin around a fitted orthographic depth range (design note §3).
const ORTHO_FIT_MARGIN: f64 = 0.01;
/// Extra fitted orthographic margin as a fraction of the view height: room for overlays sized in
/// pixels (arrows, rings), whose extent is not in the fitted bounds.
const ORTHO_FIT_HEIGHT_MARGIN: f64 = 0.25;

/// Clip planes along the view axis, mm from the eye (RND-11 near-plane policy, design note §3).
///
/// Perspective: `near` is the near plane and `far` is infinite. Orthographic: the depth range
/// `[near, far]`; `near` may be negative (geometry behind the eye plane is still visible).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DepthRange {
    /// Near plane distance, mm.
    pub near: f64,
    /// Far plane distance, mm (`f64::INFINITY` for perspective).
    pub far: f64,
}

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

    /// The depth range without scene knowledge: perspective near = [`Camera::default_near`];
    /// orthographic from `max(distance, height)` behind the eye to about 1000 × the
    /// eye-to-target distance in front of it.
    pub fn default_depth_range(&self) -> DepthRange {
        match self.projection {
            Projection::Perspective { .. } => DepthRange {
                near: self.default_near(),
                far: f64::INFINITY,
            },
            Projection::Orthographic { height } => {
                let dist = self.eye.distance(self.target);
                DepthRange {
                    near: -dist.max(height),
                    far: dist * ORTHO_DEPTH_FACTOR + height,
                }
            }
        }
    }

    /// The near-plane policy (RND-11, design note §3) for visible content inside `bounds`
    /// (world, mm).
    ///
    /// Perspective: near = 1e-3 of the eye-to-target distance (at least 1e-4 mm), far infinite;
    /// the navigation keeps the target at the depth of what is looked at. Orthographic: the
    /// bounds projected on the view direction plus a 1 % margin (and a quarter of the view
    /// height for overlays sized in pixels), so the f32 depth step follows the scene, not the
    /// eye distance. Empty bounds give [`Camera::default_depth_range`].
    pub fn depth_range_for(&self, bounds: &Aabb3) -> DepthRange {
        if bounds.is_empty() {
            return self.default_depth_range();
        }
        let forward = self.forward().get();
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for x in [bounds.min.x, bounds.max.x] {
            for y in [bounds.min.y, bounds.max.y] {
                for z in [bounds.min.z, bounds.max.z] {
                    let d = (Point3::new(x, y, z) - self.eye).dot(forward);
                    lo = lo.min(d);
                    hi = hi.max(d);
                }
            }
        }
        self.depth_range_for_extent(lo, hi)
    }

    /// [`Camera::depth_range_for`] from the view-axis extent `[lo, hi]` (mm in front of the eye)
    /// of the visible content.
    pub(crate) fn depth_range_for_extent(&self, lo: f64, hi: f64) -> DepthRange {
        match self.projection {
            Projection::Perspective { .. } => self.default_depth_range(),
            Projection::Orthographic { height } => {
                if !(lo.is_finite() && hi.is_finite() && lo <= hi) {
                    return self.default_depth_range();
                }
                // Never zero width: a flat part seen edge-on still needs a depth slab.
                let margin = ORTHO_FIT_MARGIN * (hi - lo)
                    + ORTHO_FIT_HEIGHT_MARGIN * height
                    + 1e-9 * lo.abs().max(hi.abs()).max(1.0);
                DepthRange {
                    near: lo - margin,
                    far: hi + margin,
                }
            }
        }
    }

    /// The RTE uniform for a view of `size_px` with near-plane distance `near` (orthographic
    /// cameras use [`Camera::default_depth_range`]).
    pub fn uniform(&self, size_px: [u32; 2], near: f64) -> CameraUniform {
        self.uniform_in(size_px, self.range_with_near(near))
    }

    /// The RTE uniform for a view of `size_px` with the depth range `range`.
    pub fn uniform_in(&self, size_px: [u32; 2], range: DepthRange) -> CameraUniform {
        let rotation = self.view_rotation();
        let proj = self.proj_in(size_px, range);
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

    /// [`Camera::uniform`] with the image shifted by `shift_px` pixels (x right, y down): a pick
    /// pass uses it to put the centre of a texel exactly on the cursor (RND-04).
    pub(crate) fn uniform_shifted(
        &self,
        size_px: [u32; 2],
        range: DepthRange,
        shift_px: [f64; 2],
    ) -> CameraUniform {
        let mut u = self.uniform_in(size_px, range);
        let shift = Mat4::from_translation(Vec3::new(
            2.0 * shift_px[0] / f64::from(size_px[0].max(1)),
            -2.0 * shift_px[1] / f64::from(size_px[1].max(1)),
            0.0,
        ));
        let proj = shift * self.proj_in(size_px, range);
        u.proj = proj.as_mat4().to_cols_array_2d();
        u.inv_view_proj_rte = (proj * self.view_rotation())
            .inverse()
            .as_mat4()
            .to_cols_array_2d();
        u
    }

    /// f64 world-to-view matrix.
    pub fn view_matrix(&self) -> Mat4 {
        self.view_rotation() * Mat4::from_translation(-self.eye)
    }

    /// f64 view-to-clip matrix (reversed-Z, depth in `[0, 1]`) with
    /// [`Camera::default_depth_range`].
    pub fn proj_matrix(&self, size_px: [u32; 2]) -> Mat4 {
        self.proj_in(size_px, self.default_depth_range())
    }

    /// f64 view-to-clip matrix (reversed-Z) with the depth range `range`.
    pub fn proj_matrix_in(&self, size_px: [u32; 2], range: DepthRange) -> Mat4 {
        self.proj_in(size_px, range)
    }

    /// [`Camera::default_depth_range`] with the perspective near plane at `near`.
    fn range_with_near(&self, near: f64) -> DepthRange {
        match self.projection {
            Projection::Perspective { .. } => DepthRange {
                near,
                far: f64::INFINITY,
            },
            Projection::Orthographic { .. } => self.default_depth_range(),
        }
    }

    fn proj_in(&self, size_px: [u32; 2], range: DepthRange) -> Mat4 {
        let aspect = aspect(size_px);
        let near = range.near;
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
                let far_plane = range.far;
                // May be negative: geometry between the eye and this plane is still visible.
                let near_plane = range.near;
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
    pub(crate) fn basis(&self) -> (Vec3, Vec3, Vec3) {
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

    /// RND-11 near-plane policy: orthographic depth fits the bounds (+ margins) whatever the
    /// eye distance; perspective near is 1e-3 of the distance, at least 1e-4 mm.
    #[test]
    fn depth_range_policy() {
        let cams = cameras();
        let persp = cams[0];
        let r = persp.depth_range_for(&Aabb3::new(Point3::splat(-5.0), Point3::splat(5.0)));
        assert_eq!(r.near, persp.default_near());
        assert!(r.far.is_infinite());
        let tiny = Camera {
            eye: Point3::new(0.0, -0.02, 0.0),
            ..persp
        };
        assert_eq!(tiny.default_near(), MIN_NEAR_MM);

        let ortho = cams[3]; // 1e5 mm from the origin
        let b = Aabb3::new(
            ortho.target - Vec3::splat(5.0),
            ortho.target + Vec3::splat(5.0),
        );
        let fitted = ortho.depth_range_for(&b);
        let d = ortho.eye.distance(ortho.target);
        // The box spans ±5√3 along any direction at most.
        assert!(
            fitted.near <= d - 5.0 && fitted.far >= d + 5.0,
            "{fitted:?}"
        );
        assert!(fitted.far - fitted.near <= 20.0 * 1.01 + 0.5 * 120.0 + 1e-3);
        // Every box corner projects inside [0, 1] depth, the nearer one higher.
        let near_pt = ortho.target - ortho.forward().get() * 5.0;
        let far_pt = ortho.target + ortho.forward().get() * 5.0;
        let proj = ortho.proj_matrix_in(SIZE, fitted) * ortho.view_matrix();
        let (a, z) = (
            proj.project_point3(near_pt).z,
            proj.project_point3(far_pt).z,
        );
        assert!(a > z && (0.0..=1.0).contains(&a) && (0.0..=1.0).contains(&z));
        // The f32 depth step is set by the fitted range, not by 1000 × the eye distance.
        let placeholder = ortho.default_depth_range();
        assert!(placeholder.far - placeholder.near > 50.0 * (fitted.far - fitted.near));
        assert_eq!(ortho.depth_range_for(&Aabb3::EMPTY), placeholder);
        // The uniform uses the range; `uniform` keeps the placeholder for orthographic views.
        let u = ortho.uniform_in(SIZE, fitted);
        assert_eq!(
            u.proj,
            ortho
                .proj_matrix_in(SIZE, fitted)
                .as_mat4()
                .to_cols_array_2d()
        );
        assert_eq!(
            ortho.uniform(SIZE, 0.1).proj,
            ortho.uniform_in(SIZE, placeholder).proj
        );
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
