// Layout matches `amg_render::CameraUniform` (240 bytes, design note §14). External producers
// (IMP-05) copy this struct verbatim; field offsets are 0, 64, 128, 192, 208, 224, 232, 236.
//
// Space conventions:
//   * "RTE" = relative to eye: world position minus the eye. `view_rte` is a pure rotation, so
//     an RTE point goes straight into `view_rte`, then `proj`.
//   * `proj` is reversed-Z: depth 1 at the near plane, 0 at infinity (perspective) or at the far
//     plane (orthographic). Depth compare is GreaterEqual.
//   * The eye is split into `eye_hi + eye_lo` (f64 -> 2 x f32); see `eye_relative`.
struct CameraUniform {
    view_rte: mat4x4<f32>,
    proj: mat4x4<f32>,
    inv_view_proj_rte: mat4x4<f32>,
    eye_hi: vec4<f32>,
    eye_lo: vec4<f32>,
    size_px: vec2<f32>,
    pixel_size_at_1mm: f32,
    ortho: u32,
}

// World position given as `hi + lo` minus the eye, in f32 without losing the digits the
// subtraction of two large numbers would drop: `hi - eye_hi` is exact when both are close.
fn eye_relative(camera: CameraUniform, hi: vec3<f32>, lo: vec3<f32>) -> vec3<f32> {
    return (hi - camera.eye_hi.xyz) + (lo - camera.eye_lo.xyz);
}
