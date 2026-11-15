//! Analyses and scalar fields (RND-08, design note §11): [`Shading`] per instance, the zebra,
//! draft and overhang shader modes, per-vertex scalar fields drawn through a colormap LUT with
//! bands, isolines and out-of-range colours, vector-field deformation with [`Animation`], the
//! [`legend`] ticks and the CPU colormap reference [`colormap_rgb`].
//!
//! Everything here is a shader variant of the mesh pass (`shaders/analysis.wgsl`): switching an
//! instance's shading rewrites its 128-byte instance record and its 128-byte analysis record,
//! never geometry. Fields are uploaded once per mesh ([`upload_scalar_field`],
//! [`upload_vector_field`]) and read by vertex index from storage buffers.

use std::sync::OnceLock;

use amg_core::{Dir3, Mat3, Point3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::Gpu;

use crate::colormap_data::{TURBO, VIRIDIS};
use crate::upload::{GpuMesh, check_size, upload_chunked};
use crate::{FieldKey, RenderError};

/// How an instance's surface is coloured.
#[derive(Clone, Debug, PartialEq, Default)]
pub enum Shading {
    /// The instance's [`crate::Material`].
    #[default]
    Material,
    /// A per-vertex scalar field through a colormap, optionally deformed by a vector field.
    Field(FieldBinding),
    /// A normal-based analysis computed live in the shader.
    Analysis(Analysis),
}

/// A scalar field (and optional deformation) bound to an instance.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldBinding {
    /// The scalar field ([`upload_scalar_field`] of the instance's mesh). `None`: the surface
    /// keeps its material colour (deformation only). A missing key, a vector field or a field of
    /// another vertex count is treated as `None`.
    pub scalar: Option<FieldKey>,
    /// Colormap.
    pub map: ColorMap,
    /// Values mapped to the ends of the colormap, `(min, max)`.
    pub range: (f32, f32),
    /// Number of discrete colour bands; `None` (or 0) for a continuous map.
    pub bands: Option<u32>,
    /// Number of isoline intervals over the range; `None` (or 0) for no isolines.
    pub isolines: Option<u32>,
    /// Values below `range.0`.
    pub below: OutOfRange,
    /// Values above `range.1`.
    pub above: OutOfRange,
    /// Vector-field deformation.
    pub deformation: Option<Deformation>,
}

impl FieldBinding {
    /// A continuous Viridis map of `scalar` over `range`, clamped, without deformation.
    pub fn new(scalar: Option<FieldKey>, range: (f32, f32)) -> FieldBinding {
        FieldBinding {
            scalar,
            map: ColorMap::default(),
            range,
            bands: None,
            isolines: None,
            below: OutOfRange::Clamp,
            above: OutOfRange::Clamp,
            deformation: None,
        }
    }
}

/// What a value outside the field range shows.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OutOfRange {
    /// The colour of the nearest end of the map.
    Clamp,
    /// A fixed linear RGBA colour. Alpha below 0.5 is transparent (the fragment is discarded).
    Color([f32; 4]),
}

/// `p' = p + scale · A(τ) · d` with the per-vertex displacement `d` (design note §11).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Deformation {
    /// The vector field ([`upload_vector_field`] of the instance's mesh), mm, mesh-local frame.
    pub field: FieldKey,
    /// Display scale (1 = true scale).
    pub scale: f32,
    /// How `A(τ)` follows the animation time ([`crate::Renderer::set_animation_time`]).
    pub animation: Animation,
}

/// The deformation amplitude `A(τ)` over the animation time τ.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Animation {
    /// `A = 1`.
    Static,
    /// `A = sin(2πτ / T)` (mode shapes).
    Sine {
        /// Period `T`, seconds.
        period_s: f32,
    },
    /// `A = min(τ / T, 1)` (a static result growing in once).
    Ramp {
        /// Ramp duration `T`, seconds.
        period_s: f32,
    },
}

impl Animation {
    /// `A(τ)`, the CPU twin of the shader's `deform_amplitude`.
    pub fn amplitude(self, time_s: f32) -> f32 {
        match self {
            Animation::Static => 1.0,
            Animation::Sine { period_s } => {
                (std::f32::consts::TAU * time_s / period_s.max(1e-6)).sin()
            }
            Animation::Ramp { period_s } => (time_s / period_s.max(1e-6)).min(1.0),
        }
    }

    fn code(self) -> (f32, f32) {
        match self {
            Animation::Static => (0.0, 1.0),
            Animation::Sine { period_s } => (1.0, period_s),
            Animation::Ramp { period_s } => (2.0, period_s),
        }
    }
}

/// Live normal-based analyses (formulas: design note §11, the definition shared with S05 and
/// S16). Directions are in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Analysis {
    /// Reflection stripes: `t = acos(r·a) / π × stripes` with `r = reflect(−v, n)`.
    Zebra {
        /// Stripe count `N` of the formula.
        stripes: u32,
        /// Vertical stripes (axis = camera right, or world X when world locked) instead of
        /// horizontal ones (axis = camera up, or world Z).
        vertical: bool,
        /// Stripes fixed to the world instead of the view.
        world_locked: bool,
    },
    /// Draft angle `β = asin(n·p)` on a blue–white–red map over `[−max_angle, max_angle]`;
    /// `|β| < required` is highlighted ([`DRAFT_INSUFFICIENT_COLOR`]). Radians.
    Draft {
        /// Pull direction `p`.
        pull: Dir3,
        /// `β` at the ends of the map, radians.
        max_angle: f64,
        /// Minimum draft, radians.
        required: f64,
    },
    /// Overhang classes of S16 AM-02 (`docs/notes/am-build-prep.md` §1): with `c = −n·b`, a
    /// fragment is *contact* when `c > 0` and its height above the plate is at most
    /// `plate_tol`, else *overhang* when `c > cos critical` (strict), else *supported*. Uses the
    /// flat face normal.
    Overhang {
        /// Build direction `b` (world).
        build_up: Dir3,
        /// Critical self-supporting angle `α` from the horizontal, radians.
        critical: f64,
        /// Height of the plate along `b` (`dot(x, b)` of a point on the plate), mm.
        plate_height: f64,
        /// Contact slab thickness above the plate, mm.
        plate_tol: f64,
    },
}

/// Colormaps of the scalar-field LUT (one texture-array layer each).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub enum ColorMap {
    /// matplotlib's Viridis (CC0), the default for analyses.
    #[default]
    Viridis,
    /// Google's Turbo (Apache-2.0).
    Turbo,
    /// Fusion-like blue → cyan → green → yellow → red, the usual FEA fringe map.
    Rainbow,
    /// Diverging blue → white → red.
    BlueWhiteRed,
    /// Black → white.
    Grey,
}

impl ColorMap {
    /// Every map, in texture-array layer order.
    pub const ALL: [ColorMap; 5] = [
        ColorMap::Viridis,
        ColorMap::Turbo,
        ColorMap::Rainbow,
        ColorMap::BlueWhiteRed,
        ColorMap::Grey,
    ];

    /// Layer of the LUT texture array.
    pub(crate) fn layer(self) -> u32 {
        self as u32
    }

    /// sRGB colour of the map at `t ∈ [0, 1]`, before quantisation.
    fn srgb(self, t: f64) -> [f64; 3] {
        let t = t.clamp(0.0, 1.0);
        let table = |data: &[[f32; 3]; 256]| {
            let x = t * 255.0;
            let i = (x.floor() as usize).min(254);
            let f = x - i as f64;
            let (a, b) = (data[i], data[i + 1]);
            [0, 1, 2].map(|k| f64::from(a[k]) * (1.0 - f) + f64::from(b[k]) * f)
        };
        let stops = |s: &[[f64; 3]]| {
            let x = t * (s.len() - 1) as f64;
            let i = (x.floor() as usize).min(s.len() - 2);
            let f = x - i as f64;
            [0, 1, 2].map(|k| s[i][k] * (1.0 - f) + s[i + 1][k] * f)
        };
        match self {
            ColorMap::Viridis => table(&VIRIDIS),
            ColorMap::Turbo => table(&TURBO),
            ColorMap::Rainbow => stops(&[
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 1.0],
                [0.0, 1.0, 0.0],
                [1.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
            ]),
            // Moreland's cool-warm end points with a light grey centre.
            ColorMap::BlueWhiteRed => stops(&[
                [59.0 / 255.0, 76.0 / 255.0, 192.0 / 255.0],
                [221.0 / 255.0, 221.0 / 255.0, 221.0 / 255.0],
                [180.0 / 255.0, 4.0 / 255.0, 38.0 / 255.0],
            ]),
            ColorMap::Grey => [t, t, t],
        }
    }
}

/// Entries of one LUT layer.
pub const LUT_SIZE: usize = 256;

/// The `Rgba8UnormSrgb` LUT layer of `map` (entry `i` is the map at `i / 255`), exactly the
/// bytes uploaded to the GPU. Viridis and Turbo are their reference tables rounded to 8 bits.
pub fn colormap_lut(map: ColorMap) -> [[u8; 4]; LUT_SIZE] {
    let mut out = [[0u8; 4]; LUT_SIZE];
    for (i, px) in out.iter_mut().enumerate() {
        let c = map.srgb(i as f64 / 255.0);
        let q = |v: f64| (v.clamp(0.0, 1.0) * 255.0).round() as u8;
        *px = [q(c[0]), q(c[1]), q(c[2]), 255];
    }
    out
}

fn srgb_decode(byte: u8) -> f32 {
    let s = f32::from(byte) / 255.0;
    if s <= 0.040_45 {
        s / 12.92
    } else {
        ((s + 0.055) / 1.055).powf(2.4)
    }
}

/// Linear RGB of `map` at `t ∈ [0, 1]` (clamped; NaN gives [`NAN_COLOR`]): the LUT entries
/// decoded to linear and interpolated linearly, as the GPU's filtered sample of the
/// `Rgba8UnormSrgb` texture does. The CPU reference for legends and tests.
pub fn colormap_rgb(map: ColorMap, t: f32) -> [f32; 3] {
    if t.is_nan() {
        return NAN_COLOR;
    }
    let lut = lut_linear(map);
    let x = t.clamp(0.0, 1.0) * 255.0;
    let i = (x.floor() as usize).min(LUT_SIZE - 2);
    let f = x - i as f32;
    let (a, b) = (lut[i], lut[i + 1]);
    [0, 1, 2].map(|k| a[k] * (1.0 - f) + b[k] * f)
}

fn lut_linear(map: ColorMap) -> &'static [[f32; 3]; LUT_SIZE] {
    static TABLES: OnceLock<Vec<[[f32; 3]; LUT_SIZE]>> = OnceLock::new();
    let tables = TABLES.get_or_init(|| {
        ColorMap::ALL
            .iter()
            .map(|&m| {
                colormap_lut(m).map(|p| [srgb_decode(p[0]), srgb_decode(p[1]), srgb_decode(p[2])])
            })
            .collect()
    });
    &tables[map.layer() as usize]
}

/// Linear colour of NaN field values.
pub const NAN_COLOR: [f32; 3] = [0.35, 0.35, 0.35];
/// Linear colour isolines are blended towards.
pub const ISOLINE_COLOR: [f32; 3] = [0.0, 0.0, 0.0];
/// Opacity of isolines at their centre.
pub const ISOLINE_ALPHA: f32 = 0.85;
/// Dark zebra stripe (linear, drawn unlit).
pub const ZEBRA_DARK: [f32; 3] = [0.02, 0.02, 0.02];
/// Light zebra stripe (linear, drawn unlit).
pub const ZEBRA_LIGHT: [f32; 3] = [0.9, 0.9, 0.9];
/// Draft below the required angle.
pub const DRAFT_INSUFFICIENT_COLOR: [f32; 3] = [1.0, 0.85, 0.0];
/// Overhang at severity 0 (just past the critical angle).
pub const OVERHANG_LOW_COLOR: [f32; 3] = [1.0, 0.55, 0.1];
/// Overhang at severity 1 (a horizontal ceiling).
pub const OVERHANG_HIGH_COLOR: [f32; 3] = [0.8, 0.02, 0.02];
/// Plate contact.
pub const CONTACT_COLOR: [f32; 3] = [0.25, 0.4, 0.75];

/// Overhang class codes (the values of `docs/notes/am-build-prep.md` §1.2 and the AM-02 golden
/// file), as returned by [`overhang_class`] and written by [`AnalysisOutput::Classes`].
pub mod class {
    /// Supported (upward faces, walls, self-supporting slopes).
    pub const SUPPORTED: u32 = 0;
    /// Overhang.
    pub const OVERHANG: u32 = 1;
    /// Plate contact.
    pub const CONTACT: u32 = 2;
    /// Zero normal.
    pub const DEGENERATE: u32 = 3;
    /// Draft `β ≥ required`.
    pub const DRAFT_POSITIVE: u32 = 0;
    /// Draft `β ≤ −required`.
    pub const DRAFT_NEGATIVE: u32 = 1;
    /// `|β| < required`.
    pub const DRAFT_INSUFFICIENT: u32 = 2;
    /// Dark zebra stripe.
    pub const ZEBRA_DARK: u32 = 0;
    /// Light zebra stripe.
    pub const ZEBRA_LIGHT: u32 = 1;
}

/// The overhang rule of the shader in f64 (normal and direction are normalised first; a zero
/// normal is degenerate): `c = −n·b`; contact when `c > 0 ∧ height ≤ tol`, overhang when
/// `c > cos_critical`, else supported. Matches `amg_am::overhang::classify_face` up to its
/// guards (`ANGLE_GUARD`, `LINEAR_GUARD`).
pub fn overhang_class(
    normal: Vec3,
    build_dir: Vec3,
    cos_critical: f64,
    height: f64,
    tol: f64,
) -> u32 {
    if normal == Vec3::ZERO {
        return class::DEGENERATE;
    }
    let c = -normal.normalize().dot(build_dir.normalize());
    if c > 0.0 && height <= tol {
        class::CONTACT
    } else if c > cos_critical {
        class::OVERHANG
    } else {
        class::SUPPORTED
    }
}

/// Overhang severity `clamp((c − cos α) / (1 − cos α), 0, 1)` (S16 `severity`).
pub fn overhang_severity(c: f64, cos_critical: f64) -> f64 {
    ((c - cos_critical) / (1.0 - cos_critical).max(1e-12)).clamp(0.0, 1.0)
}

/// Draft angle `β = asin(clamp(n·p, −1, 1))`, radians (unit vectors).
pub fn draft_angle(normal: Vec3, pull: Vec3) -> f64 {
    normal.dot(pull).clamp(-1.0, 1.0).asin()
}

/// Zebra stripe parameter `t = acos(clamp(r·a, −1, 1)) / π × stripes` with
/// `r = reflect(−v, n)`; `v` points from the surface to the eye. The stripe is light where
/// `fract(t) ≥ 0.5`.
pub fn zebra_t(normal: Vec3, to_eye: Vec3, axis: Vec3, stripes: u32) -> f64 {
    let v = -to_eye;
    let r = v - 2.0 * v.dot(normal) * normal;
    r.dot(axis).clamp(-1.0, 1.0).acos() / std::f64::consts::PI * f64::from(stripes)
}

/// Debug output of the analysis and field pipelines (test mode, [`crate::Renderer::set_analysis_output`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub enum AnalysisOutput {
    /// Analysis colours lit by the view's lighting model (zebra is always unlit).
    #[default]
    Lit,
    /// Analysis colours without lighting.
    Unlit,
    /// The class of each fragment in the red channel as linear `class / 4` (overhang classes,
    /// draft classes, zebra stripes; see [`class`]), unlit; decode with [`decode_class`].
    Classes,
}

/// Inverse of [`AnalysisOutput::Classes`] on a captured sRGB pixel: the nearest class, or
/// `None` when the pixel matches none within 6/255 (background, edges, antialiased pixels).
pub fn decode_class(pixel: [u8; 4]) -> Option<u32> {
    (0..4u32)
        .map(|k| {
            let linear = k as f32 / 4.0;
            let s = if linear <= 0.003_130_8 {
                12.92 * linear
            } else {
                1.055 * linear.powf(1.0 / 2.4) - 0.055
            };
            (k, (s * 255.0).round() as u8)
        })
        .find(|&(_, r)| pixel[0].abs_diff(r) <= 6 && pixel[1] <= 6 && pixel[2] <= 6)
        .map(|(k, _)| k)
}

/// A colour-bar description for `amg-ui` to paint.
#[derive(Clone, Debug, PartialEq)]
pub struct Legend {
    /// The colormap.
    pub map: ColorMap,
    /// Value at the bottom of the bar.
    pub min: f32,
    /// Value at the top of the bar.
    pub max: f32,
    /// "Nice" tick values inside `[min, max]`, ascending.
    pub ticks: Vec<f32>,
    /// Discrete bands, if any.
    pub bands: Option<u32>,
}

/// "Nice" ticks (1, 2, 2.5, 5 × 10^k) inside `[min, max]` of `binding.range`, at most
/// `max_ticks` (and at least one when the range is a point), using the smallest step that fits.
pub fn legend(binding: &FieldBinding, max_ticks: usize) -> Legend {
    let (a, b) = binding.range;
    let (min, max) = if a <= b { (a, b) } else { (b, a) };
    Legend {
        map: binding.map,
        min: binding.range.0,
        max: binding.range.1,
        ticks: nice_ticks(f64::from(min), f64::from(max), max_ticks)
            .into_iter()
            .map(|v| v as f32)
            .collect(),
        bands: binding.bands.filter(|&n| n > 0),
    }
}

fn nice_ticks(min: f64, max: f64, max_ticks: usize) -> Vec<f64> {
    if max_ticks == 0 || !min.is_finite() || !max.is_finite() {
        return Vec::new();
    }
    let span = max - min;
    if span <= 0.0 {
        return vec![min];
    }
    // Start one decade below the step that would give one tick per span.
    let mut exp = (span / max_ticks as f64).log10().floor() as i32 - 1;
    loop {
        for mantissa in [1.0, 2.0, 2.5, 5.0] {
            let step = mantissa * 10f64.powi(exp);
            let eps = step * 1e-9;
            let first = ((min - eps) / step).ceil();
            let last = ((max + eps) / step).floor();
            let count = (last - first + 1.0).max(0.0);
            if count <= max_ticks as f64 {
                return (0..count as usize)
                    .map(|i| {
                        let v = (first + i as f64) * step;
                        // Round away binary noise (0.30000000000000004) at the step's precision.
                        let digits = (-step.log10().floor() as i32 + 1).clamp(0, 15);
                        let scale = 10f64.powi(digits);
                        let v = (v * scale).round() / scale;
                        v + 0.0 // no negative zero
                    })
                    .collect();
            }
        }
        exp += 1;
    }
}

/// What a [`GpuField`] holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldKind {
    /// One `f32` per vertex.
    Scalar,
    /// One `[f32; 3]` displacement per vertex.
    Vector,
}

/// A per-vertex field on the GPU (storage buffer). Immutable and `Send + Sync`.
#[derive(Debug)]
pub struct GpuField {
    pub(crate) buffer: wgpu::Buffer,
    kind: FieldKind,
    vertex_count: u32,
    /// Finite min/max of a scalar field; largest displacement length of a vector field (max, max).
    extent: (f32, f32),
    /// Group 2/3 bind group of the analysis pipelines (built on first draw).
    pub(crate) bind: OnceLock<wgpu::BindGroup>,
}

impl GpuField {
    /// Scalar or vector.
    pub fn kind(&self) -> FieldKind {
        self.kind
    }

    /// Number of vertices (values).
    pub fn vertex_count(&self) -> u32 {
        self.vertex_count
    }

    /// Smallest and largest finite value of a scalar field (`None` when no value is finite);
    /// `None` for a vector field.
    pub fn value_range(&self) -> Option<(f32, f32)> {
        (self.kind == FieldKind::Scalar && self.extent.0 <= self.extent.1).then_some(self.extent)
    }

    /// Largest displacement length of a vector field (0 for a scalar field).
    pub fn max_displacement(&self) -> f32 {
        if self.kind == FieldKind::Vector {
            self.extent.1
        } else {
            0.0
        }
    }

    /// VRAM used.
    pub fn gpu_bytes(&self) -> u64 {
        self.buffer.size()
    }
}

/// Storage buffers must not be empty; the buffer is padded to at least 16 bytes.
fn padded<T: bytemuck::Pod + Default>(values: &[T]) -> Vec<T> {
    let min = 16usize.div_ceil(size_of::<T>().max(1));
    let mut v = values.to_vec();
    if v.len() < min {
        v.resize(min, T::default());
    }
    v
}

/// Uploads one scalar per vertex of `mesh` (curvature, thickness, stress, …). NaN is allowed
/// (drawn [`NAN_COLOR`]). Job threads; checkpoints like [`crate::upload_mesh`].
///
/// # Errors
/// [`RenderError::FieldLength`] unless `values.len() == mesh.vertex_count()`;
/// [`RenderError::TooLarge`]; [`RenderError::Cancelled`].
pub fn upload_scalar_field(
    gpu: &Gpu,
    mesh: &GpuMesh,
    values: &[f32],
    ctx: &JobCtx,
) -> Result<GpuField, RenderError> {
    check_length(values.len(), mesh)?;
    check_size(
        "scalar field",
        (values.len() * 4) as u64,
        gpu.device().limits().max_storage_buffer_binding_size,
    )?;
    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for chunk in values.chunks(1 << 18) {
        for &v in chunk.iter().filter(|v| v.is_finite()) {
            lo = lo.min(v);
            hi = hi.max(v);
        }
        ctx.checkpoint()?;
    }
    let buffer = upload_chunked(
        gpu,
        "amg scalar field",
        &padded(values),
        wgpu::BufferUsages::STORAGE,
        ctx,
    )?;
    Ok(GpuField {
        buffer,
        kind: FieldKind::Scalar,
        vertex_count: mesh.vertex_count(),
        extent: (lo, hi),
        bind: OnceLock::new(),
    })
}

/// Uploads one displacement per vertex of `mesh`, in mm, in the mesh's local frame.
///
/// # Errors
/// [`RenderError::FieldLength`] unless `values.len() == mesh.vertex_count()`;
/// [`RenderError::InvalidData`] for a non-finite displacement; [`RenderError::TooLarge`];
/// [`RenderError::Cancelled`].
pub fn upload_vector_field(
    gpu: &Gpu,
    mesh: &GpuMesh,
    values: &[[f32; 3]],
    ctx: &JobCtx,
) -> Result<GpuField, RenderError> {
    check_length(values.len(), mesh)?;
    check_size(
        "vector field",
        (values.len() * 12) as u64,
        gpu.device().limits().max_storage_buffer_binding_size,
    )?;
    let mut longest = 0f32;
    for (c, chunk) in values.chunks(1 << 18).enumerate() {
        for (i, d) in chunk.iter().enumerate() {
            if !d.iter().all(|v| v.is_finite()) {
                return Err(RenderError::InvalidData(format!(
                    "vertex {} has a non-finite displacement",
                    c * (1 << 18) + i
                )));
            }
            longest = longest.max((d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt());
        }
        ctx.checkpoint()?;
    }
    let flat: &[f32] = bytemuck::cast_slice(values);
    let buffer = upload_chunked(
        gpu,
        "amg vector field",
        &padded(flat),
        wgpu::BufferUsages::STORAGE,
        ctx,
    )?;
    Ok(GpuField {
        buffer,
        kind: FieldKind::Vector,
        vertex_count: mesh.vertex_count(),
        extent: (longest, longest),
        bind: OnceLock::new(),
    })
}

fn check_length(got: usize, mesh: &GpuMesh) -> Result<(), RenderError> {
    let expected = mesh.vertex_count() as usize;
    if got == expected {
        Ok(())
    } else {
        Err(RenderError::FieldLength { got, expected })
    }
}

/// The shading part of a pipeline key: which analysis program and which field streams.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub(crate) struct ShadeKey {
    pub mode: ShadeMode,
    /// A scalar field is bound (mode `Field`).
    pub scalar: bool,
    /// A displacement field is bound.
    pub deform: bool,
    /// Debug output (only for non-material modes).
    pub out: AnalysisOutput,
}

impl ShadeKey {
    /// Plain material shading (the RND-02 mesh pipelines).
    pub fn is_material(&self) -> bool {
        self.mode == ShadeMode::Material
    }
}

/// Shader program of the analysis pipelines (WGSL override `MODE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub(crate) enum ShadeMode {
    #[default]
    Material = 0,
    Field = 1,
    Zebra = 2,
    Draft = 3,
    Overhang = 4,
}

/// How a draw group is shaded: the key plus the field buffers to bind.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub(crate) struct GroupShade {
    pub key: ShadeKey,
    /// Scalar field (group 2) and displacement field (group 3).
    pub fields: [Option<FieldKey>; 2],
}

/// Resolves an instance's shading against the live fields of a mesh with `vertex_count`
/// vertices. Fields that are missing, of the wrong kind or length are dropped.
pub(crate) fn group_shade(
    shading: &Shading,
    vertex_count: u32,
    field: impl Fn(FieldKey) -> Option<(FieldKind, u32)>,
    out: AnalysisOutput,
) -> GroupShade {
    let valid = |k: FieldKey, kind: FieldKind| {
        field(k).is_some_and(|(fk, n)| fk == kind && n == vertex_count)
    };
    let mut g = GroupShade::default();
    match shading {
        Shading::Material => {}
        Shading::Field(b) => {
            let scalar = b.scalar.filter(|&k| valid(k, FieldKind::Scalar));
            let deform = b
                .deformation
                .map(|d| d.field)
                .filter(|&k| valid(k, FieldKind::Vector));
            if scalar.is_some() || deform.is_some() {
                g.key.mode = ShadeMode::Field;
                g.key.scalar = scalar.is_some();
                g.key.deform = deform.is_some();
                g.fields = [scalar, deform];
            }
        }
        Shading::Analysis(a) => {
            g.key.mode = match a {
                Analysis::Zebra { .. } => ShadeMode::Zebra,
                Analysis::Draft { .. } => ShadeMode::Draft,
                Analysis::Overhang { .. } => ShadeMode::Overhang,
            };
        }
    }
    if !g.key.is_material() {
        g.key.out = out;
    }
    g
}

/// Flag bits of `AnalysisRecord::mode.y`.
mod flags {
    pub const ZEBRA_VERTICAL: u32 = 1;
    pub const ZEBRA_WORLD: u32 = 2;
    pub const BELOW_CLAMP: u32 = 4;
    pub const ABOVE_CLAMP: u32 = 8;
}

/// One 128-byte record per instance, parallel to the instance records (`shaders/analysis.wgsl`
/// `AnalysisRecord`). Camera independent.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct AnalysisRecord {
    /// x = mode, y = flags, z = colormap layer, w = bands (0 = continuous).
    pub mode: [u32; 4],
    /// World direction (draft pull, overhang build direction); w = zebra stripes or overhang
    /// `cos α`.
    pub dir: [f32; 4],
    /// Overhang: `Lᵀ b` (height per local unit) and w = height of the local origin above the
    /// plate (f64 on the CPU), so `height = dot(local, xyz) + w`.
    pub height: [f32; 4],
    /// Field: (min, max, isolines, 0). Draft: (max angle, required, 0, 0). Overhang:
    /// (plate tolerance, 0, 0, 0).
    pub params: [f32; 4],
    /// Below-range colour; w < 0 = clamp.
    pub below: [f32; 4],
    /// Above-range colour; w < 0 = clamp.
    pub above: [f32; 4],
    /// Deformation (scale, animation kind 0/1/2, period s, 0).
    pub deform: [f32; 4],
    pub reserved: [f32; 4],
}

/// Size of one analysis record, bytes.
pub(crate) const ANALYSIS_RECORD_BYTES: u64 = size_of::<AnalysisRecord>() as u64;

/// Builds the analysis record of an instance whose mesh-local origin sits at `translation`
/// (world) with world-from-local linear part `linear`.
pub(crate) fn build_analysis_record(
    shading: &Shading,
    transform: &Transform,
    translation: Point3,
) -> AnalysisRecord {
    let mut r = AnalysisRecord::default();
    let v4 = |v: Vec3, w: f32| [v.x as f32, v.y as f32, v.z as f32, w];
    match shading {
        Shading::Material => {}
        Shading::Field(b) => {
            r.mode = [
                ShadeMode::Field as u32,
                0,
                b.map.layer(),
                b.bands.unwrap_or(0),
            ];
            r.params = [b.range.0, b.range.1, b.isolines.unwrap_or(0) as f32, 0.0];
            let side = |o: OutOfRange, flag: u32, out: &mut [f32; 4], f: &mut u32| match o {
                OutOfRange::Clamp => {
                    *f |= flag;
                    *out = [0.0, 0.0, 0.0, -1.0];
                }
                OutOfRange::Color(c) => *out = c,
            };
            let mut f = 0;
            side(b.below, flags::BELOW_CLAMP, &mut r.below, &mut f);
            side(b.above, flags::ABOVE_CLAMP, &mut r.above, &mut f);
            r.mode[1] = f;
            if let Some(d) = b.deformation {
                let (kind, period) = d.animation.code();
                r.deform = [d.scale, kind, period, 0.0];
            }
        }
        Shading::Analysis(Analysis::Zebra {
            stripes,
            vertical,
            world_locked,
        }) => {
            let mut f = 0;
            if *vertical {
                f |= flags::ZEBRA_VERTICAL;
            }
            if *world_locked {
                f |= flags::ZEBRA_WORLD;
            }
            r.mode = [ShadeMode::Zebra as u32, f, 0, 0];
            r.dir = [0.0, 0.0, 0.0, *stripes as f32];
        }
        Shading::Analysis(Analysis::Draft {
            pull,
            max_angle,
            required,
        }) => {
            r.mode = [
                ShadeMode::Draft as u32,
                0,
                ColorMap::BlueWhiteRed.layer(),
                0,
            ];
            r.dir = v4(pull.get(), 0.0);
            r.params = [*max_angle as f32, *required as f32, 0.0, 0.0];
        }
        Shading::Analysis(Analysis::Overhang {
            build_up,
            critical,
            plate_height,
            plate_tol,
        }) => {
            let b = build_up.get();
            r.mode = [ShadeMode::Overhang as u32, 0, 0, 0];
            r.dir = v4(b, critical.cos() as f32);
            // height(x) = dot(L·local + t, b) − plate = dot(local, Lᵀb) + (dot(t, b) − plate).
            let lt_b = Mat3::from(transform.matrix3).transpose() * b;
            let h0 = translation.dot(b) - plate_height;
            r.height = v4(lt_b, h0 as f32);
            r.params = [*plate_tol as f32, 0.0, 0.0, 0.0];
        }
    }
    r
}

/// The instance's deformation and its field, if it has one.
pub(crate) fn deformation_of(shading: &Shading) -> Option<Deformation> {
    match shading {
        Shading::Field(b) => b.deformation,
        _ => None,
    }
}

/// Creates the colormap LUT: a 256 × 1 `Rgba8UnormSrgb` texture array, one layer per
/// [`ColorMap`] in [`ColorMap::ALL`] order.
pub(crate) fn create_lut(gpu: &Gpu) -> wgpu::Texture {
    let device = gpu.device();
    let layers = ColorMap::ALL.len() as u32;
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("amg colormap LUT"),
        size: wgpu::Extent3d {
            width: LUT_SIZE as u32,
            height: 1,
            depth_or_array_layers: layers,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_DST
            | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let mut bytes = Vec::with_capacity(LUT_SIZE * 4 * ColorMap::ALL.len());
    for map in ColorMap::ALL {
        bytes.extend(colormap_lut(map).iter().flatten());
    }
    gpu.queue().write_texture(
        texture.as_image_copy(),
        &bytes,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(LUT_SIZE as u32 * 4),
            rows_per_image: Some(1),
        },
        wgpu::Extent3d {
            width: LUT_SIZE as u32,
            height: 1,
            depth_or_array_layers: layers,
        },
    );
    texture
}

/// The LUT sampler: linear, clamped.
pub(crate) fn lut_sampler(device: &wgpu::Device) -> wgpu::Sampler {
    device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("amg colormap LUT"),
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        ..Default::default()
    })
}

/// Group 1 layout of the analysis pipelines: analysis records, LUT, sampler, globals.
pub(crate) fn analysis_bind_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("amg analysis"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2Array,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    })
}

/// WGSL shared by the analysis pipelines and the GPU test kernels (`testing`).
pub(crate) const ANALYSIS_COMMON_WGSL: &str = include_str!("shaders/analysis_common.wgsl");
/// Bindings and entry points of the analysis pipelines (appended to the mesh module).
pub(crate) const ANALYSIS_WGSL: &str = include_str!("shaders/analysis.wgsl");

/// Constants generated into the analysis WGSL from this module (one definition).
pub(crate) fn analysis_constants() -> String {
    let v3 = |name: &str, v: [f32; 3]| {
        format!(
            "const {name}: vec3<f32> = vec3<f32>({:?}, {:?}, {:?});\n",
            v[0], v[1], v[2]
        )
    };
    let mut out = String::new();
    out += &v3("NAN_COLOR", NAN_COLOR);
    out += &v3("ISOLINE_COLOR", ISOLINE_COLOR);
    out += &format!("const ISOLINE_ALPHA: f32 = {ISOLINE_ALPHA:?};\n");
    out += &v3("ZEBRA_DARK", ZEBRA_DARK);
    out += &v3("ZEBRA_LIGHT", ZEBRA_LIGHT);
    out += &v3("DRAFT_INSUFFICIENT_COLOR", DRAFT_INSUFFICIENT_COLOR);
    out += &v3("OVERHANG_LOW_COLOR", OVERHANG_LOW_COLOR);
    out += &v3("OVERHANG_HIGH_COLOR", OVERHANG_HIGH_COLOR);
    out += &v3("CONTACT_COLOR", CONTACT_COLOR);
    out += &format!(
        "const FLAG_ZEBRA_VERTICAL: u32 = {}u;\nconst FLAG_ZEBRA_WORLD: u32 = {}u;\nconst FLAG_BELOW_CLAMP: u32 = {}u;\nconst FLAG_ABOVE_CLAMP: u32 = {}u;\n",
        flags::ZEBRA_VERTICAL,
        flags::ZEBRA_WORLD,
        flags::BELOW_CLAMP,
        flags::ABOVE_CLAMP
    );
    out
}

/// Common WGSL with its generated constants.
pub(crate) fn common_wgsl() -> String {
    format!("{}\n{ANALYSIS_COMMON_WGSL}", analysis_constants())
}

/// GPU evaluation of the analysis WGSL functions (`shaders/analysis_common.wgsl`) for tests:
/// the S16 AM-02 golden classification file, LUT sampling and the deformation readback
/// ([`crate::Renderer::deformed_positions`]). Blocking; job and test threads only.
pub mod testing {
    use amg_jobs::gpu::Gpu;

    use super::{ColorMap, common_wgsl, create_lut, lut_sampler};
    use crate::RenderError;

    const TEST_WGSL: &str = include_str!("shaders/analysis_test.wgsl");

    /// One overhang case: the inputs of the WGSL `overhang_class`.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct OverhangCase {
        /// Face normal (any length; zero is degenerate).
        pub normal: [f32; 3],
        /// Build direction (any non-zero length).
        pub build_dir: [f32; 3],
        /// `cos α`.
        pub cos_critical: f32,
        /// Height above the plate, mm.
        pub height: f32,
        /// Contact tolerance, mm.
        pub tol: f32,
    }

    /// One field sample: the inputs of the WGSL `field_t`.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct FieldSample {
        /// Field value.
        pub value: f32,
        /// Range minimum.
        pub min: f32,
        /// Range maximum.
        pub max: f32,
        /// Bands (0 = continuous).
        pub bands: u32,
    }

    /// The WGSL of one kernel (`OVERHANG`, `LUT`, `DEFORM`): the common functions, the
    /// `Params` struct and that kernel's section of `analysis_test.wgsl`.
    pub(crate) fn kernel_wgsl(name: &str) -> String {
        let head = TEST_WGSL.split("//#if").next().unwrap_or_default();
        let start = format!("//#if {name}\n");
        let body = TEST_WGSL
            .split_once(&start)
            .and_then(|(_, rest)| rest.split_once("//#endif"))
            .map_or("", |(body, _)| body);
        format!("{}\n{head}\n{body}", common_wgsl())
    }

    /// `Params` of `analysis_test.wgsl`.
    #[repr(C)]
    #[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
    pub(crate) struct Params {
        pub count: u32,
        pub slot: u32,
        pub layer: u32,
        pub pad: u32,
    }

    pub(crate) fn storage_init(gpu: &Gpu, label: &str, bytes: &[u8]) -> wgpu::Buffer {
        let size = (bytes.len() as u64).max(16).next_multiple_of(4);
        let buffer = gpu.device().create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        if !bytes.is_empty() {
            gpu.queue().write_buffer(&buffer, 0, bytes);
        }
        buffer
    }

    pub(crate) fn uniform_init(gpu: &Gpu, params: Params) -> wgpu::Buffer {
        let buffer = gpu.device().create_buffer(&wgpu::BufferDescriptor {
            label: Some("amg analysis test params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        gpu.queue()
            .write_buffer(&buffer, 0, bytemuck::bytes_of(&params));
        buffer
    }

    /// Runs `entry` of kernel `name` over `count` invocations with `resources` at bindings
    /// 0.., then reads back `out_bytes` of `out`.
    pub(crate) fn run(
        gpu: &Gpu,
        name: &str,
        entry: &str,
        resources: &[wgpu::BindingResource<'_>],
        count: u32,
        out: &wgpu::Buffer,
        out_bytes: u64,
    ) -> Result<Vec<u8>, RenderError> {
        let device = gpu.device();
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some(entry),
            source: wgpu::ShaderSource::Wgsl(kernel_wgsl(name).into()),
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(entry),
            layout: None,
            module: &module,
            entry_point: Some(entry),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        });
        let entries: Vec<_> = resources
            .iter()
            .enumerate()
            .map(|(i, r)| wgpu::BindGroupEntry {
                binding: i as u32,
                resource: r.clone(),
            })
            .collect();
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(entry),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &entries,
        });
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some(entry) });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some(entry),
                timestamp_writes: None,
            });
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(count.div_ceil(64).max(1), 1, 1);
        }
        gpu.queue().submit([encoder.finish()]);
        Ok(gpu.readback::<u8>(out, 0..out_bytes.max(4)).wait()?)
    }

    /// The WGSL `overhang_class` of every case (the shader's rule, evaluated on the GPU).
    ///
    /// # Errors
    /// [`RenderError::Gpu`] on device loss.
    pub fn overhang_classes(gpu: &Gpu, cases: &[OverhangCase]) -> Result<Vec<u32>, RenderError> {
        let mut input = Vec::with_capacity(cases.len() * 12);
        for c in cases {
            input.extend_from_slice(&c.normal);
            input.extend_from_slice(&c.build_dir);
            input.extend_from_slice(&[c.cos_critical, c.height, c.tol, 0.0, 0.0, 0.0]);
        }
        let n = cases.len() as u32;
        let input = storage_init(gpu, "amg overhang cases", bytemuck::cast_slice(&input));
        let out = storage_init(gpu, "amg overhang classes", &vec![0u8; cases.len() * 4]);
        let params = uniform_init(
            gpu,
            Params {
                count: n,
                slot: 0,
                layer: 0,
                pad: 0,
            },
        );
        let bytes = run(
            gpu,
            "OVERHANG",
            "overhang_kernel",
            &[
                input.as_entire_binding(),
                out.as_entire_binding(),
                params.as_entire_binding(),
            ],
            n,
            &out,
            u64::from(n) * 4,
        )?;
        Ok(bytemuck::pod_collect_to_vec::<u8, u32>(&bytes)
            .into_iter()
            .take(cases.len())
            .collect())
    }

    /// The analysis pipelines' colour of each sample: `field_t`, then a filtered sample of
    /// `map` in a LUT built like the renderer's. Returns linear RGB and the map parameter `t`.
    ///
    /// # Errors
    /// [`RenderError::Gpu`] on device loss.
    pub fn field_colors(
        gpu: &Gpu,
        map: ColorMap,
        samples: &[FieldSample],
    ) -> Result<Vec<([f32; 3], f32)>, RenderError> {
        let mut input = Vec::with_capacity(samples.len() * 4);
        for s in samples {
            input.extend_from_slice(&[s.value, s.min, s.max, s.bands as f32]);
        }
        let n = samples.len() as u32;
        let input = storage_init(gpu, "amg lut cases", bytemuck::cast_slice(&input));
        let out = storage_init(gpu, "amg lut colours", &vec![0u8; samples.len() * 16]);
        let params = uniform_init(
            gpu,
            Params {
                count: n,
                slot: 0,
                layer: map.layer(),
                pad: 0,
            },
        );
        let lut = create_lut(gpu);
        let view = lut.create_view(&wgpu::TextureViewDescriptor {
            dimension: Some(wgpu::TextureViewDimension::D2Array),
            ..Default::default()
        });
        let sampler = lut_sampler(gpu.device());
        let bytes = run(
            gpu,
            "LUT",
            "lut_kernel",
            &[
                input.as_entire_binding(),
                out.as_entire_binding(),
                params.as_entire_binding(),
                wgpu::BindingResource::TextureView(&view),
                wgpu::BindingResource::Sampler(&sampler),
            ],
            n,
            &out,
            u64::from(n) * 16,
        )?;
        Ok(bytemuck::pod_collect_to_vec::<u8, [f32; 4]>(&bytes)
            .into_iter()
            .take(samples.len())
            .map(|c| ([c[0], c[1], c[2]], c[3]))
            .collect())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, reason = "test code")]
#[allow(clippy::float_cmp, reason = "exact table values")]
mod tests {
    use super::*;

    #[test]
    fn analysis_record_is_128_bytes() {
        assert_eq!(ANALYSIS_RECORD_BYTES, 128);
    }

    #[test]
    fn viridis_and_turbo_luts_are_the_reference_tables_rounded() {
        for (map, table) in [(ColorMap::Viridis, &VIRIDIS), (ColorMap::Turbo, &TURBO)] {
            let lut = colormap_lut(map);
            for (i, (px, want)) in lut.iter().zip(table.iter()).enumerate() {
                for k in 0..3 {
                    assert_eq!(
                        px[k],
                        (f64::from(want[k]) * 255.0).round() as u8,
                        "{map:?} entry {i} channel {k}"
                    );
                }
                assert_eq!(px[3], 255);
            }
        }
        // Spot values of the published tables.
        assert_eq!(VIRIDIS[0], [0.267_004, 0.004_874, 0.329_415]);
        assert_eq!(VIRIDIS[255], [0.993_248, 0.906_157, 0.143_936]);
        assert_eq!(TURBO[0], [0.189_95, 0.071_76, 0.232_17]);
        assert_eq!(TURBO[255], [0.479_6, 0.015_83, 0.010_55]);
    }

    #[test]
    fn code_defined_maps_have_their_stops() {
        let rainbow = colormap_lut(ColorMap::Rainbow);
        assert_eq!(rainbow[0], [0, 0, 255, 255]);
        assert_eq!(rainbow[255], [255, 0, 0, 255]);
        let mid = colormap_lut(ColorMap::Rainbow)[128];
        assert!(mid[1] == 255 && mid[0] < 5 && mid[2] < 5, "{mid:?}");
        let bwr = colormap_lut(ColorMap::BlueWhiteRed);
        assert_eq!(bwr[0], [59, 76, 192, 255]);
        assert_eq!(bwr[255], [180, 4, 38, 255]);
        let grey = colormap_lut(ColorMap::Grey);
        for (i, px) in grey.iter().enumerate() {
            assert_eq!(px[..3], [i as u8; 3]);
        }
    }

    #[test]
    fn colormap_rgb_interpolates_the_decoded_lut() {
        for map in ColorMap::ALL {
            let lut = colormap_lut(map);
            for i in [0usize, 1, 100, 254, 255] {
                let c = colormap_rgb(map, i as f32 / 255.0);
                for k in 0..3 {
                    assert!((c[k] - srgb_decode(lut[i][k])).abs() < 1e-5);
                }
            }
            let a = colormap_rgb(map, 10.0 / 255.0);
            let b = colormap_rgb(map, 11.0 / 255.0);
            let m = colormap_rgb(map, 10.5 / 255.0);
            for k in 0..3 {
                assert!((m[k] - 0.5 * (a[k] + b[k])).abs() < 1e-5);
            }
            assert_eq!(colormap_rgb(map, -3.0), colormap_rgb(map, 0.0));
            assert_eq!(colormap_rgb(map, 7.0), colormap_rgb(map, 1.0));
        }
        assert_eq!(colormap_rgb(ColorMap::Turbo, f32::NAN), NAN_COLOR);
    }

    fn binding(range: (f32, f32)) -> FieldBinding {
        FieldBinding::new(None, range)
    }

    #[test]
    fn legend_ticks_are_nice_numbers_table() {
        #[allow(clippy::type_complexity, reason = "test table")]
        let table: &[((f32, f32), usize, &[f32])] = &[
            ((0.0, 1.0), 6, &[0.0, 0.2, 0.4, 0.6, 0.8, 1.0]),
            ((0.0, 1.0), 5, &[0.0, 0.25, 0.5, 0.75, 1.0]),
            ((0.0, 1.0), 3, &[0.0, 0.5, 1.0]),
            (
                (0.0, 1.0),
                11,
                &[0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
            ),
            ((0.0, 100.0), 5, &[0.0, 25.0, 50.0, 75.0, 100.0]),
            ((-3.7, 12.2), 6, &[-2.5, 0.0, 2.5, 5.0, 7.5, 10.0]),
            ((-3.7, 12.2), 4, &[0.0, 5.0, 10.0]),
            (
                (-3.7, 12.2),
                10,
                &[-2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0],
            ),
            ((0.013, 0.087), 5, &[0.02, 0.04, 0.06, 0.08]),
            ((-1.0, 1.0), 5, &[-1.0, -0.5, 0.0, 0.5, 1.0]),
            ((250.0, 1000.0), 4, &[400.0, 600.0, 800.0, 1000.0]),
            ((250.0, 1000.0), 3, &[500.0, 1000.0]),
            ((1.0e6, 3.0e6), 3, &[1.0e6, 2.0e6, 3.0e6]),
            ((2.0, 2.0), 5, &[2.0]),
            ((7.0, 3.0), 3, &[4.0, 6.0]),
        ];
        for &(range, max, want) in table {
            let l = legend(&binding(range), max);
            assert_eq!(l.ticks, want, "range {range:?}, max {max}");
            assert!(l.ticks.len() <= max.max(1));
        }
        let mut b = binding((0.0, 1.0));
        b.map = ColorMap::Rainbow;
        b.bands = Some(8);
        let l = legend(&b, 6);
        assert_eq!(
            (l.map, l.min, l.max, l.bands),
            (ColorMap::Rainbow, 0.0, 1.0, Some(8))
        );
        assert!(legend(&binding((0.0, 1.0)), 0).ticks.is_empty());
        assert!(legend(&binding((0.0, f32::NAN)), 5).ticks.is_empty());
    }

    #[test]
    fn every_legend_tick_is_a_nice_multiple_inside_the_range() {
        let mut seed = 0x1234_5678_u64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        for _ in 0..2000 {
            let a = (next() - 0.5) * 10f64.powf(next() * 8.0 - 3.0);
            let b = a + next() * 10f64.powf(next() * 8.0 - 3.0) + 1e-9;
            let max = 2 + (next() * 10.0) as usize;
            let t = nice_ticks(a, b, max);
            assert!(!t.is_empty() && t.len() <= max, "{a} {b} {max}: {t:?}");
            for w in t.windows(2) {
                let step = w[1] - w[0];
                let e = step.log10().floor();
                let m = step / 10f64.powf(e);
                assert!(
                    [1.0, 2.0, 2.5, 5.0, 10.0]
                        .iter()
                        .any(|n| (m - n).abs() < 1e-6),
                    "step {step}"
                );
            }
            let span = b - a;
            assert!(
                t.iter()
                    .all(|&v| v >= a - span * 1e-6 && v <= b + span * 1e-6)
            );
        }
    }

    #[test]
    fn cpu_overhang_rule_follows_the_am02_order() {
        let b = Vec3::Z;
        let cos45 = std::f64::consts::FRAC_PI_4.cos();
        assert_eq!(
            overhang_class(Vec3::ZERO, b, cos45, 5.0, 0.1),
            class::DEGENERATE
        );
        assert_eq!(
            overhang_class(-Vec3::Z, b, cos45, 0.05, 0.1),
            class::CONTACT
        );
        assert_eq!(
            overhang_class(-Vec3::Z, b, cos45, 5.0, 0.1),
            class::OVERHANG
        );
        assert_eq!(
            overhang_class(Vec3::Z, b, cos45, 0.0, 0.1),
            class::SUPPORTED
        );
        assert_eq!(
            overhang_class(Vec3::X, b, cos45, 0.0, 0.1),
            class::SUPPORTED,
            "walls never contact"
        );
        // 40° from horizontal is an overhang at α = 45°, 50° is not.
        let at = |deg: f64| {
            let r = deg.to_radians();
            Vec3::new(r.sin(), 0.0, -r.cos())
        };
        assert_eq!(
            overhang_class(at(40.0), b, cos45, 5.0, 0.1),
            class::OVERHANG
        );
        assert_eq!(
            overhang_class(at(50.0), b, cos45, 5.0, 0.1),
            class::SUPPORTED
        );
        assert!((overhang_severity(1.0, cos45) - 1.0).abs() < 1e-12);
        assert_eq!(overhang_severity(cos45, cos45), 0.0);
    }

    #[test]
    fn group_shade_drops_missing_or_mismatched_fields() {
        use amg_core::SlotMap;
        let mut keys = SlotMap::<FieldKey, (FieldKind, u32)>::with_key();
        let s = keys.insert((FieldKind::Scalar, 10));
        let v = keys.insert((FieldKind::Vector, 10));
        let short = keys.insert((FieldKind::Scalar, 9));
        let lookup = |k| keys.get(k).copied();
        let mut b = FieldBinding::new(Some(s), (0.0, 1.0));
        let g = group_shade(&Shading::Field(b.clone()), 10, lookup, AnalysisOutput::Lit);
        assert_eq!(
            (g.key.mode, g.key.scalar, g.key.deform),
            (ShadeMode::Field, true, false)
        );
        b.deformation = Some(Deformation {
            field: v,
            scale: 2.0,
            animation: Animation::Static,
        });
        let g = group_shade(
            &Shading::Field(b.clone()),
            10,
            lookup,
            AnalysisOutput::Unlit,
        );
        assert_eq!(g.fields, [Some(s), Some(v)]);
        assert_eq!(g.key.out, AnalysisOutput::Unlit);
        b.scalar = Some(short);
        let g = group_shade(&Shading::Field(b.clone()), 10, lookup, AnalysisOutput::Lit);
        assert_eq!(g.fields, [None, Some(v)], "wrong length dropped");
        b.scalar = Some(v);
        b.deformation = None;
        let g = group_shade(&Shading::Field(b), 10, lookup, AnalysisOutput::Unlit);
        assert!(g.key.is_material(), "a vector field is not a scalar");
        assert_eq!(
            g.key.out,
            AnalysisOutput::Lit,
            "material keys ignore the debug output"
        );
        let g = group_shade(
            &Shading::Analysis(Analysis::Zebra {
                stripes: 8,
                vertical: false,
                world_locked: false,
            }),
            10,
            lookup,
            AnalysisOutput::Lit,
        );
        assert_eq!(g.key.mode, ShadeMode::Zebra);
    }

    #[test]
    fn overhang_record_measures_height_above_the_plate_in_local_coordinates() {
        let t = Transform::from_scale_rotation_translation(
            Vec3::splat(1.0),
            amg_core::Quat::from_rotation_x(0.6),
            Vec3::new(1.0e5, 2.0, 30.0),
        );
        let origin = Point3::new(0.5, 0.25, -1.0);
        let translation = t.transform_point3(origin);
        let shading = Shading::Analysis(Analysis::Overhang {
            build_up: Dir3::Z,
            critical: 45f64.to_radians(),
            plate_height: 10.0,
            plate_tol: 0.1,
        });
        let r = build_analysis_record(&shading, &t, translation);
        for local in [
            Vec3::ZERO,
            Vec3::new(3.0, -4.0, 5.0),
            Vec3::new(-1.0, 2.0, 0.5),
        ] {
            let world = t.transform_point3(origin + local);
            let want = world.z - 10.0;
            let got = f64::from(r.height[0]) * local.x
                + f64::from(r.height[1]) * local.y
                + f64::from(r.height[2]) * local.z
                + f64::from(r.height[3]);
            assert!((got - want).abs() < 1e-5, "{got} vs {want}");
        }
        assert!((f64::from(r.dir[3]) - 45f64.to_radians().cos()).abs() < 1e-7);
        assert_eq!(r.params[0], 0.1);
    }

    #[test]
    fn field_record_packs_range_bands_out_of_range_and_deformation() {
        let mut b = FieldBinding::new(None, (-2.0, 5.0));
        b.map = ColorMap::Turbo;
        b.bands = Some(7);
        b.isolines = Some(4);
        b.below = OutOfRange::Color([0.0, 0.0, 1.0, 1.0]);
        b.deformation = Some(Deformation {
            field: amg_core::SlotMap::<FieldKey, ()>::with_key().insert(()),
            scale: 3.5,
            animation: Animation::Sine { period_s: 2.0 },
        });
        let r = build_analysis_record(&Shading::Field(b), &Transform::IDENTITY, Point3::ZERO);
        assert_eq!(r.mode, [1, flags::ABOVE_CLAMP, 1, 7]);
        assert_eq!(r.params, [-2.0, 5.0, 4.0, 0.0]);
        assert_eq!(r.below, [0.0, 0.0, 1.0, 1.0]);
        assert_eq!(r.above[3], -1.0);
        assert_eq!(r.deform, [3.5, 1.0, 2.0, 0.0]);
    }

    #[test]
    fn animation_amplitudes() {
        assert_eq!(Animation::Static.amplitude(12.0), 1.0);
        let s = Animation::Sine { period_s: 2.0 };
        assert!((s.amplitude(0.5) - 1.0).abs() < 1e-6);
        assert!(s.amplitude(1.0).abs() < 1e-6);
        let r = Animation::Ramp { period_s: 4.0 };
        assert_eq!(r.amplitude(1.0), 0.25);
        assert_eq!(r.amplitude(9.0), 1.0);
    }

    #[test]
    fn decode_class_inverts_the_class_output() {
        for k in 0..4u32 {
            let linear = k as f32 / 4.0;
            let s = if linear <= 0.003_130_8 {
                12.92 * linear
            } else {
                1.055 * linear.powf(1.0 / 2.4) - 0.055
            };
            let r = (s * 255.0).round() as u8;
            assert_eq!(decode_class([r, 0, 0, 255]), Some(k));
        }
        assert_eq!(decode_class([60, 0, 0, 255]), None);
        assert_eq!(decode_class([0, 120, 0, 255]), None);
    }

    #[test]
    fn generated_constants_are_valid_wgsl_literals() {
        let c = analysis_constants();
        assert!(c.contains("const ZEBRA_LIGHT: vec3<f32> = vec3<f32>(0.9, 0.9, 0.9);"));
        assert!(c.contains("const FLAG_ABOVE_CLAMP: u32 = 8u;"));
    }
}
