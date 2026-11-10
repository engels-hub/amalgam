//! WGSL sources of the primitive kernels. Every kernel runs 256 threads per workgroup and
//! finds its workgroup as `wid.x + wid.y * nwg.x` (launches fold into 2-D past 65 535
//! workgroups); `groups` in the parameters masks the surplus workgroups of the last row.

use super::scalar::ReduceOp;

/// The `op` function of a reduction over `T`.
fn op_body(op: ReduceOp) -> &'static str {
    match op {
        ReduceOp::Sum => "return a + b;",
        ReduceOp::Min => "return select(a, b, b < a);",
        ReduceOp::Max => "return select(a, b, a < b);",
    }
}

/// One tree pass over `src` (see `prim` module docs). Parameters:
/// `[n, groups, out_index, xform]`; writes `dst[out_index + group]`. With `flags`, an element is
/// read as `u32(src[i] != 0)` (used on the flags of a compaction).
pub(super) fn reduce(ty: &str, op: ReduceOp, identity: &str, flags: bool) -> String {
    let load = if flags {
        "select(0u, 1u, src[i] != 0u)"
    } else {
        "src[i]"
    };
    let body = op_body(op);
    format!(
        r"
alias T = {ty};
struct Params {{ n: u32, groups: u32, out_index: u32, p0: u32 }}
@group(0) @binding(0) var<uniform> prm: Params;
@group(0) @binding(1) var<storage, read> src: array<T>;
@group(0) @binding(2) var<storage, read_write> dst: array<T>;
var<workgroup> sh: array<T, 256>;

fn op(a: T, b: T) -> T {{ {body} }}

@compute @workgroup_size(256)
fn main(@builtin(local_invocation_id) lid: vec3<u32>,
        @builtin(workgroup_id) wid: vec3<u32>,
        @builtin(num_workgroups) nwg: vec3<u32>) {{
    let t = lid.x;
    let g = wid.x + wid.y * nwg.x;
    let live = g < prm.groups;
    let base = g * 2048u;
    var acc: T = {identity};
    for (var k = 0u; k < 8u; k++) {{
        let i = base + k * 256u + t;
        if (live && i < prm.n) {{
            acc = op(acc, {load});
        }}
    }}
    sh[t] = acc;
    workgroupBarrier();
    for (var s = 128u; s > 0u; s = s >> 1u) {{
        if (t < s) {{
            sh[t] = op(sh[t], sh[t + s]);
        }}
        workgroupBarrier();
    }}
    if (t == 0u && live) {{
        dst[prm.out_index + g] = sh[0];
    }}
}}
"
    )
}

/// `out[i] = a[i] * b[i]` in `real`. Parameters: `[n, 0, 0, 0]`.
pub(super) fn multiply() -> String {
    r"
struct Params { n: u32, p0: u32, p1: u32, p2: u32 }
@group(0) @binding(0) var<uniform> prm: Params;
@group(0) @binding(1) var<storage, read> a: array<real>;
@group(0) @binding(2) var<storage, read> b: array<real>;
@group(0) @binding(3) var<storage, read_write> dst: array<real>;

@compute @workgroup_size(256)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    let i = amg_linear_id(gid, nwg);
    if (i < prm.n) {
        dst[i] = a[i] * b[i];
    }
}
"
    .to_owned()
}

/// One workgroup per segment. Parameters: `[segments, values_len, groups, 0]`.
pub(super) fn segmented(ty: &str, op: ReduceOp, identity: &str) -> String {
    let body = op_body(op);
    format!(
        r"
alias T = {ty};
struct Params {{ segments: u32, values_len: u32, p0: u32, p1: u32 }}
@group(0) @binding(0) var<uniform> prm: Params;
@group(0) @binding(1) var<storage, read> values: array<T>;
@group(0) @binding(2) var<storage, read> offsets: array<u32>;
@group(0) @binding(3) var<storage, read_write> dst: array<T>;
var<workgroup> sh: array<T, 256>;

fn op(a: T, b: T) -> T {{ {body} }}

@compute @workgroup_size(256)
fn main(@builtin(local_invocation_id) lid: vec3<u32>,
        @builtin(workgroup_id) wid: vec3<u32>,
        @builtin(num_workgroups) nwg: vec3<u32>) {{
    let t = lid.x;
    let seg = wid.x + wid.y * nwg.x;
    let live = seg < prm.segments;
    var start = 0u;
    var end = 0u;
    if (live) {{
        end = min(offsets[seg + 1u], prm.values_len);
        start = min(offsets[seg], end);
    }}
    var acc: T = {identity};
    for (var i = start + t; i < end; i += 256u) {{
        acc = op(acc, values[i]);
    }}
    sh[t] = acc;
    workgroupBarrier();
    for (var s = 128u; s > 0u; s = s >> 1u) {{
        if (t < s) {{
            sh[t] = op(sh[t], sh[t + s]);
        }}
        workgroupBarrier();
    }}
    if (t == 0u && live) {{
        dst[seg] = sh[0];
    }}
}}
"
    )
}

/// Scan of one tile of 2 048 `u32`, plus an optional per-tile offset.
/// Parameters: `[n, groups, inclusive, use_off, xform, write_total, total_index, 0]`.
pub(super) fn scan_tiles() -> String {
    r"
struct Params {
    n: u32, groups: u32, inclusive: u32, use_off: u32,
    xform: u32, write_total: u32, total_index: u32, p0: u32,
}
@group(0) @binding(0) var<uniform> prm: Params;
@group(0) @binding(1) var<storage, read> src: array<u32>;
@group(0) @binding(2) var<storage, read_write> dst: array<u32>;
@group(0) @binding(3) var<storage, read> offs: array<u32>;
@group(0) @binding(4) var<storage, read_write> total: array<u32>;
var<workgroup> tile: array<u32, 2304>;
var<workgroup> sc: array<u32, 256>;

// One padding word per 8 words: thread t's 8 contiguous words then start in different banks.
fn pad(i: u32) -> u32 { return i + (i >> 3u); }

@compute @workgroup_size(256)
fn main(@builtin(local_invocation_id) lid: vec3<u32>,
        @builtin(workgroup_id) wid: vec3<u32>,
        @builtin(num_workgroups) nwg: vec3<u32>) {
    let t = lid.x;
    let g = wid.x + wid.y * nwg.x;
    let live = g < prm.groups;
    let base = g * 2048u;
    for (var k = 0u; k < 8u; k++) {
        let i = base + k * 256u + t;
        var v = 0u;
        if (live && i < prm.n) {
            v = src[i];
            if (prm.xform != 0u) { v = select(0u, 1u, v != 0u); }
        }
        tile[pad(k * 256u + t)] = v;
    }
    workgroupBarrier();
    var run: array<u32, 8>;
    var s = 0u;
    for (var j = 0u; j < 8u; j++) {
        s += tile[pad(t * 8u + j)];
        run[j] = s;
    }
    sc[t] = s;
    workgroupBarrier();
    for (var d = 1u; d < 256u; d = d * 2u) {
        let a = sc[t];
        var b = a;
        if (t >= d) { b = a + sc[t - d]; }
        workgroupBarrier();
        sc[t] = b;
        workgroupBarrier();
    }
    var before = sc[t] - s;
    if (live && prm.use_off != 0u) { before += offs[g]; }
    for (var j = 0u; j < 8u; j++) {
        let v = tile[pad(t * 8u + j)];
        let inc = before + run[j];
        tile[pad(t * 8u + j)] = select(inc - v, inc, prm.inclusive != 0u);
    }
    workgroupBarrier();
    for (var k = 0u; k < 8u; k++) {
        let i = base + k * 256u + t;
        if (live && i < prm.n) {
            dst[i] = tile[pad(k * 256u + t)];
        }
    }
    if (t == 255u && live && prm.write_total != 0u) {
        total[prm.total_index] = sc[255];
    }
}
"
    .to_owned()
}

/// `out[pos[i]] = values[i]` where `flags[i] != 0`. Parameters: `[n, 0, 0, 0]`.
pub(super) fn compact_scatter() -> String {
    r"
struct Params { n: u32, p0: u32, p1: u32, p2: u32 }
@group(0) @binding(0) var<uniform> prm: Params;
@group(0) @binding(1) var<storage, read> flags: array<u32>;
@group(0) @binding(2) var<storage, read> values: array<u32>;
@group(0) @binding(3) var<storage, read> pos: array<u32>;
@group(0) @binding(4) var<storage, read_write> dst: array<u32>;

@compute @workgroup_size(256)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    let i = amg_linear_id(gid, nwg);
    if (i < prm.n && flags[i] != 0u) {
        dst[pos[i]] = values[i];
    }
}
"
    .to_owned()
}

/// Elements per thread in the sort kernels (the sort tile is `256 * SORT_PT`).
pub(super) const SORT_PT: u32 = 4;

/// Digit histogram per sort tile, digit-major: `hist[digit * tiles + tile]`.
/// Parameters: `[n, tiles, shift, mask]`.
pub(super) fn sort_histogram() -> String {
    r"
struct Params { n: u32, tiles: u32, shift: u32, mask: u32 }
@group(0) @binding(0) var<uniform> prm: Params;
@group(0) @binding(1) var<storage, read> keys: array<u32>;
@group(0) @binding(2) var<storage, read_write> hist: array<u32>;
var<workgroup> h: array<atomic<u32>, 256>;

@compute @workgroup_size(256)
fn main(@builtin(local_invocation_id) lid: vec3<u32>,
        @builtin(workgroup_id) wid: vec3<u32>,
        @builtin(num_workgroups) nwg: vec3<u32>) {
    let t = lid.x;
    let g = wid.x + wid.y * nwg.x;
    let live = g < prm.tiles;
    atomicStore(&h[t], 0u);
    workgroupBarrier();
    let base = g * TILE_u;
    for (var k = 0u; k < PT_u; k++) {
        let i = base + k * 256u + t;
        if (live && i < prm.n) {
            atomicAdd(&h[(keys[i] >> prm.shift) & prm.mask], 1u);
        }
    }
    workgroupBarrier();
    if (live && t <= prm.mask) {
        hist[t * prm.tiles + g] = atomicLoad(&h[t]);
    }
}
"
    .replace("TILE_u", &format!("{}u", 256 * SORT_PT))
    .replace("PT_u", &format!("{SORT_PT}u"))
}

/// Stable in-tile sort by digit (one stable 4-way split per two digit bits on packed
/// `digit << 11 | index` words, `SORT_PT` per thread), then a scatter to
/// `offs[digit * tiles + tile] + rank within the digit`.
/// Parameters: `[n, tiles, shift, mask, bits, has_vals, 0, 0]`.
pub(super) fn sort_scatter() -> String {
    r"
struct Params {
    n: u32, tiles: u32, shift: u32, mask: u32,
    bits: u32, has_vals: u32, p0: u32, p1: u32,
}
@group(0) @binding(0) var<uniform> prm: Params;
@group(0) @binding(1) var<storage, read> keys_in: array<u32>;
@group(0) @binding(2) var<storage, read> vals_in: array<u32>;
@group(0) @binding(3) var<storage, read> offs: array<u32>;
@group(0) @binding(4) var<storage, read_write> keys_out: array<u32>;
@group(0) @binding(5) var<storage, read_write> vals_out: array<u32>;
var<workgroup> pk: array<u32, TILE_u>;
var<workgroup> sc: array<vec4<u32>, 256>;
var<workgroup> sd: array<vec4<u32>, 256>;
var<workgroup> first: array<u32, 256>;

fn onehot(q: u32) -> vec4<u32> {
    return vec4<u32>(select(0u, 1u, q == 0u), select(0u, 1u, q == 1u),
                     select(0u, 1u, q == 2u), select(0u, 1u, q == 3u));
}

@compute @workgroup_size(256)
fn main(@builtin(local_invocation_id) lid: vec3<u32>,
        @builtin(workgroup_id) wid: vec3<u32>,
        @builtin(num_workgroups) nwg: vec3<u32>) {
    let t = lid.x;
    let g = wid.x + wid.y * nwg.x;
    let live = g < prm.tiles;
    let base = g * TILE_u;

    // Slots past the end sort last (digit = mask) and are never written.
    for (var k = 0u; k < PT_u; k++) {
        let li = k * 256u + t;
        let i = base + li;
        var digit = prm.mask;
        if (live && i < prm.n) {
            digit = (keys_in[i] >> prm.shift) & prm.mask;
        }
        pk[li] = (digit << 11u) | li;
    }
    workgroupBarrier();

    var w: array<u32, PT_u>;
    for (var j = 0u; j < PT_u; j++) { w[j] = pk[t * PT_u + j]; }
    workgroupBarrier();

    // Stable 4-way split by each pair of digit bits, least significant first.
    for (var b = 0u; b < prm.bits; b += 2u) {
        let sh = 11u + b;
        var cnt = vec4<u32>(0u);
        for (var j = 0u; j < PT_u; j++) { cnt += onehot((w[j] >> sh) & 3u); }
        sc[t] = cnt;
        workgroupBarrier();
        // Hillis-Steele scan, ping-ponging between `sc` and `sd`: one barrier per step.
        for (var d = 1u; d < 256u; d = d * 4u) {
            var v = sc[t];
            if (t >= d) { v += sc[t - d]; }
            sd[t] = v;
            workgroupBarrier();
            var u = sd[t];
            if (t >= d * 2u) { u += sd[t - d * 2u]; }
            sc[t] = u;
            workgroupBarrier();
        }
        let before = sc[t] - cnt;
        let tot = sc[255];
        var pos = before + vec4<u32>(0u, tot.x, tot.x + tot.y, tot.x + tot.y + tot.z);
        for (var j = 0u; j < PT_u; j++) {
            let oh = onehot((w[j] >> sh) & 3u);
            let at = pos.x * oh.x + pos.y * oh.y + pos.z * oh.z + pos.w * oh.w;
            pk[at] = w[j];
            pos += oh;
        }
        workgroupBarrier();
        for (var j = 0u; j < PT_u; j++) { w[j] = pk[t * PT_u + j]; }
        workgroupBarrier();
    }

    // First slot of every digit run.
    for (var j = 0u; j < PT_u; j++) {
        let p = t * PT_u + j;
        let d = w[j] >> 11u;
        var prev = 0xffffffffu;
        if (p > 0u) { prev = pk[p - 1u] >> 11u; }
        if (d != prev) { first[d] = p; }
    }
    workgroupBarrier();

    for (var j = 0u; j < PT_u; j++) {
        let p = t * PT_u + j;
        let d = w[j] >> 11u;
        let i = base + (w[j] & 2047u);
        if (live && i < prm.n) {
            let at = offs[d * prm.tiles + g] + (p - first[d]);
            keys_out[at] = keys_in[i];
            if (prm.has_vals != 0u) { vals_out[at] = vals_in[i]; }
        }
    }
}
"
    .replace("TILE_u", &format!("{}u", 256 * SORT_PT))
    .replace("PT_u", &format!("{SORT_PT}u"))
}

/// Sets `c[0] = 0` (the reset of an `AtomicCounter`).
pub(super) fn counter_reset() -> String {
    r"
@group(0) @binding(0) var<storage, read_write> c: array<atomic<u32>>;
@compute @workgroup_size(1)
fn main() { atomicStore(&c[0], 0u); }
"
    .to_owned()
}
