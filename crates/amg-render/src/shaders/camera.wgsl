// Layout matches `amg_render::CameraUniform` (design note §14).
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
