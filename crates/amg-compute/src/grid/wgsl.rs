//! Generated WGSL: the brick grid accessors for user kernels and the `allocate` kernels.

use std::fmt::Write as _;

use super::GpuEncoding;

/// WGSL declarations for the 4 bindings of a [`GpuBrickGrid`](super::GpuBrickGrid) at
/// `@group(group) @binding(first_binding..first_binding + 3)` (meta uniform, root, tiles, data,
/// in the order of `GpuBrickGrid::bindings`) plus:
/// - `bg_slot(brick: vec3<i32>) -> u32`: brick index or fill code (`BG_FILL0`, `BG_FILL1`;
///   `BG_FILL0` outside the grid);
/// - `bg_candidate_brick(i: u32) -> vec3<i32>`: the brick of candidate index `i`
///   ([`TileBox::candidate_index`](super::TileBox::candidate_index)), for producer kernels;
/// - `bg_load(brick_index: u32, local: u32) -> f32`: sample `local` (`x + s (y + s z)`) of a brick;
/// - `bg_sample(ijk: vec3<i32>) -> f32`: like `BrickGrid::sample`;
/// - `bg_trilinear(p: vec3<f32>) -> f32`: like `BrickGrid::trilinear`, in f32;
/// - `bg_store(brick_index: u32, local: u32, v: f32)` (only when `writable`): writes one sample.
///   Packed encodings update their word with `atomicAnd` + `atomicOr`, so threads may write
///   neighbouring samples concurrently; the result does not depend on their order.
///
/// `writable` declares `data` read-write: bind [`GpuBrickGrid::data_mut_binding`] there. One
/// grid per shader module (the names are fixed). With wgpu's automatic layouts a binding the
/// kernel never reads is dropped, so a kernel must use `bg_sample`, `bg_trilinear` or `bg_load`
/// (or `bg_store`) for the four bindings to exist.
///
/// [`GpuBrickGrid::data_mut_binding`]: super::GpuBrickGrid::data_mut_binding
pub fn brick_grid_wgsl(group: u32, first_binding: u32, enc: GpuEncoding, writable: bool) -> String {
    let atomic = writable && enc != GpuEncoding::F32;
    let (access, elem) = match (writable, atomic) {
        (false, _) => ("read", "u32"),
        (true, false) => ("read_write", "u32"),
        (true, true) => ("read_write", "atomic<u32>"),
    };
    let b = first_binding;
    let mut s = String::new();
    let _ = write!(
        s,
        "
// amg-compute brick grid ({enc:?}{w})
struct BgMeta {{
    tile_origin: vec3<i32>,
    spa: u32,
    tile_dims: vec3<u32>,
    stride: u32,
    fill: vec2<f32>,
    apron: u32,
    brick_count: u32,
}}
@group({group}) @binding({b0}) var<uniform> bg_meta: BgMeta;
@group({group}) @binding({b1}) var<storage, read> bg_root: array<u32>;
@group({group}) @binding({b2}) var<storage, read> bg_tiles: array<u32>;
@group({group}) @binding({b3}) var<storage, {access}> bg_data: array<{elem}>;
const BG_FILL0: u32 = 0xffffffffu;
const BG_FILL1: u32 = 0xfffffffeu;

fn bg_slot(brick: vec3<i32>) -> u32 {{
    let t = (brick >> vec3<u32>(3u)) - bg_meta.tile_origin;
    if (any(t < vec3<i32>(0)) || any(vec3<u32>(t) >= bg_meta.tile_dims)) {{
        return BG_FILL0;
    }}
    let d = bg_meta.tile_dims;
    let node = bg_root[(u32(t.z) * d.y + u32(t.y)) * d.x + u32(t.x)];
    if (node >= BG_FILL1) {{
        return node;
    }}
    let l = vec3<u32>(brick & vec3<i32>(7));
    return bg_tiles[node * 512u + l.x + 8u * l.y + 64u * l.z];
}}

// The brick of candidate index `i` (`TileBox::candidate_index`: tile in root order, then the
// 9-bit Morton code inside the tile). Producer kernels launched over candidates use it with
// `bg_slot` to find their brick.
fn bg_candidate_brick(i: u32) -> vec3<i32> {{
    let d = bg_meta.tile_dims;
    let t = i >> 9u;
    let tl = vec3<u32>(t % d.x, (t / d.x) % d.y, t / (d.x * d.y));
    let m = i & 511u;
    let l = vec3<u32>(
        (m & 1u) | ((m >> 2u) & 2u) | ((m >> 4u) & 4u),
        ((m >> 1u) & 1u) | ((m >> 3u) & 2u) | ((m >> 5u) & 4u),
        ((m >> 2u) & 1u) | ((m >> 4u) & 2u) | ((m >> 6u) & 4u));
    return (bg_meta.tile_origin + vec3<i32>(tl)) * 8 + vec3<i32>(l);
}}

fn bg_fill(code: u32) -> f32 {{
    return select(bg_meta.fill.x, bg_meta.fill.y, code == BG_FILL1);
}}

fn bg_word(i: u32) -> u32 {{
    return {word};
}}
",
        w = if writable { ", writable" } else { "" },
        b0 = b,
        b1 = b + 1,
        b2 = b + 2,
        b3 = b + 3,
        word = if atomic {
            "atomicLoad(&bg_data[i])"
        } else {
            "bg_data[i]"
        },
    );
    s.push_str(match enc {
        GpuEncoding::F32 => {
            "
fn bg_load(brick_index: u32, local: u32) -> f32 {
    return bitcast<f32>(bg_word(brick_index * bg_meta.stride + local));
}
"
        }
        GpuEncoding::F16Packed => {
            "
fn bg_load(brick_index: u32, local: u32) -> f32 {
    let w = bg_word(brick_index * bg_meta.stride + (local >> 1u));
    return unpack2x16float(w)[local & 1u];
}
"
        }
        GpuEncoding::U8Packed => {
            "
fn bg_load(brick_index: u32, local: u32) -> f32 {
    let w = bg_word(brick_index * bg_meta.stride + (local >> 2u));
    return f32((w >> ((local & 3u) * 8u)) & 0xffu);
}
"
        }
    });
    s.push_str(
        "
fn bg_sample(ijk: vec3<i32>) -> f32 {
    let code = bg_slot(ijk >> vec3<u32>(3u));
    if (code >= BG_FILL1) {
        return bg_fill(code);
    }
    let l = vec3<u32>(ijk & vec3<i32>(7));
    let s = bg_meta.spa;
    return bg_load(code, l.x + s * (l.y + s * l.z));
}

fn bg_lerp8(c0: f32, c1: f32, c2: f32, c3: f32, c4: f32, c5: f32, c6: f32, c7: f32,
            t: vec3<f32>) -> f32 {
    let x00 = c0 * (1.0 - t.x) + c1 * t.x;
    let x10 = c2 * (1.0 - t.x) + c3 * t.x;
    let x01 = c4 * (1.0 - t.x) + c5 * t.x;
    let x11 = c6 * (1.0 - t.x) + c7 * t.x;
    let y0 = x00 * (1.0 - t.y) + x10 * t.y;
    let y1 = x01 * (1.0 - t.y) + x11 * t.y;
    return y0 * (1.0 - t.z) + y1 * t.z;
}

fn bg_trilinear(p: vec3<f32>) -> f32 {
    let f = floor(p);
    let i = vec3<i32>(f);
    let t = p - f;
    if (bg_meta.apron == 0u) {
        return bg_lerp8(
            bg_sample(i), bg_sample(i + vec3<i32>(1, 0, 0)),
            bg_sample(i + vec3<i32>(0, 1, 0)), bg_sample(i + vec3<i32>(1, 1, 0)),
            bg_sample(i + vec3<i32>(0, 0, 1)), bg_sample(i + vec3<i32>(1, 0, 1)),
            bg_sample(i + vec3<i32>(0, 1, 1)), bg_sample(i + vec3<i32>(1, 1, 1)), t);
    }
    let code = bg_slot(i >> vec3<u32>(3u));
    if (code >= BG_FILL1) {
        return bg_fill(code);
    }
    let l = vec3<u32>(i & vec3<i32>(7));
    let s = bg_meta.spa;
    let ss = s * s;
    let o = l.x + s * (l.y + s * l.z);
    return bg_lerp8(
        bg_load(code, o), bg_load(code, o + 1u),
        bg_load(code, o + s), bg_load(code, o + s + 1u),
        bg_load(code, o + ss), bg_load(code, o + ss + 1u),
        bg_load(code, o + ss + s), bg_load(code, o + ss + s + 1u), t);
}
",
    );
    if writable {
        s.push_str(match enc {
            GpuEncoding::F32 => {
                "
fn bg_store(brick_index: u32, local: u32, v: f32) {
    bg_data[brick_index * bg_meta.stride + local] = bitcast<u32>(v);
}
"
            }
            GpuEncoding::F16Packed => {
                "
fn bg_store(brick_index: u32, local: u32, v: f32) {
    let w = brick_index * bg_meta.stride + (local >> 1u);
    let sh = (local & 1u) * 16u;
    let bits = pack2x16float(vec2<f32>(v, 0.0)) & 0xffffu;
    atomicAnd(&bg_data[w], ~(0xffffu << sh));
    atomicOr(&bg_data[w], bits << sh);
}
"
            }
            GpuEncoding::U8Packed => {
                "
fn bg_store(brick_index: u32, local: u32, v: f32) {
    let w = brick_index * bg_meta.stride + (local >> 2u);
    let sh = (local & 3u) * 8u;
    let bits = u32(clamp(round(v), 0.0, 255.0));
    atomicAnd(&bg_data[w], ~(0xffu << sh));
    atomicOr(&bg_data[w], bits << sh);
}
"
            }
        });
    }
    s
}

/// Parameters of the allocate kernels (8 words, like the `prim` kernels).
const PARAMS: &str = "
struct P { n_tiles: u32, n_cand: u32, p2: u32, p3: u32, p4: u32, p5: u32, p6: u32, p7: u32 }
@group(0) @binding(0) var<uniform> prm: P;
const FILL0: u32 = 0xffffffffu;
const FILL1: u32 = 0xfffffffeu;
fn fill_code(unmarked: u32) -> u32 { return select(FILL0, FILL1, unmarked != 0u); }
";

/// Per tile: 1 if any of its 512 candidates is marked.
pub(super) fn tile_flags() -> String {
    format!(
        "{PARAMS}
@group(0) @binding(1) var<storage, read> mask: array<u32>;
@group(0) @binding(2) var<storage, read_write> flags: array<u32>;
@compute @workgroup_size(WG)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {{
    let t = amg_linear_id(gid, nwg);
    if (t >= prm.n_tiles) {{ return; }}
    var hit = 0u;
    for (var k = 0u; k < 512u; k++) {{
        if (mask[t * 512u + k] != 0u) {{ hit = 1u; break; }}
    }}
    flags[t] = hit;
}}
"
    )
}

/// Per tile: root entry = node index (flagged) or the fill code of `unmarked`.
pub(super) fn write_root() -> String {
    format!(
        "{PARAMS}
@group(0) @binding(1) var<storage, read> flags: array<u32>;
@group(0) @binding(2) var<storage, read> node_of: array<u32>;
@group(0) @binding(3) var<storage, read> unmarked: array<u32>;
@group(0) @binding(4) var<storage, read_write> root: array<u32>;
@compute @workgroup_size(WG)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {{
    let t = amg_linear_id(gid, nwg);
    if (t >= prm.n_tiles) {{ return; }}
    root[t] = select(fill_code(unmarked[t]), node_of[t], flags[t] != 0u);
}}
"
    )
}

/// Per candidate of a flagged tile: its slot = brick index (marked) or the tile's fill code.
pub(super) fn write_tiles() -> String {
    format!(
        "{PARAMS}
@group(0) @binding(1) var<storage, read> mask: array<u32>;
@group(0) @binding(2) var<storage, read> flags: array<u32>;
@group(0) @binding(3) var<storage, read> node_of: array<u32>;
@group(0) @binding(4) var<storage, read> brick_of: array<u32>;
@group(0) @binding(5) var<storage, read> unmarked: array<u32>;
@group(0) @binding(6) var<storage, read_write> tiles: array<u32>;
@compute @workgroup_size(WG)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {{
    let i = amg_linear_id(gid, nwg);
    if (i >= prm.n_cand) {{ return; }}
    let t = i >> 9u;
    if (flags[t] == 0u) {{ return; }}
    let m = i & 511u;
    // Morton bits: x = 0, 3, 6; y = 1, 4, 7; z = 2, 5, 8.
    let x = (m & 1u) | ((m >> 2u) & 2u) | ((m >> 4u) & 4u);
    let y = ((m >> 1u) & 1u) | ((m >> 3u) & 2u) | ((m >> 5u) & 4u);
    let z = ((m >> 2u) & 1u) | ((m >> 4u) & 2u) | ((m >> 6u) & 4u);
    let code = select(fill_code(unmarked[t]), brick_of[i], mask[i] != 0u);
    tiles[node_of[t] * 512u + x + 8u * y + 64u * z] = code;
}}
"
    )
}
