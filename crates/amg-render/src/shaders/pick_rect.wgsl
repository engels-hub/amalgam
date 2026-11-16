// Rectangle select (RND-04, design note §7): one invocation per id texel of the full-view pick
// render. Every texel ORs 1 (inside the rectangle) or 2 (outside) into the flags of its pick
// owner and of its sub-entity. Window = flags == 1 (every visible texel inside), crossing =
// flags & 1.
//
// `table` has 8 words per pick id: (base, count) of the body, face, edge and vertex flags;
// a count of 0 means the kind is not asked for. Sub-ids at or above the count (a section cap,
// an unassigned face) only mark the body.

struct RectUniform {
    // Inclusive minimum and exclusive maximum texel of the rectangle.
    min: vec2<u32>,
    max: vec2<u32>,
    // View size in texels.
    size: vec2<u32>,
    // Number of pick ids in `table`.
    picks: u32,
    pad: u32,
}

@group(0) @binding(0) var pick_ids: texture_2d<u32>;
@group(0) @binding(1) var<uniform> rect: RectUniform;
@group(0) @binding(2) var<storage, read> table: array<u32>;
@group(0) @binding(3) var<storage, read_write> flags: array<atomic<u32>>;

fn mark(index: u32, bit: u32) {
    // Most texels repeat a known bit; a plain load avoids the atomic traffic.
    if (atomicLoad(&flags[index]) & bit) == 0u {
        atomicOr(&flags[index], bit);
    }
}

@compute @workgroup_size(8, 8)
fn rect_flags(@builtin(global_invocation_id) gid: vec3<u32>) {
    if gid.x >= rect.size.x || gid.y >= rect.size.y {
        return;
    }
    let id = textureLoad(pick_ids, vec2<i32>(gid.xy), 0);
    let pick = id.x;
    if pick == 0u || pick >= rect.picks {
        return;
    }
    let inside = all(gid.xy >= rect.min) && all(gid.xy < rect.max);
    let bit = select(2u, 1u, inside);
    let row = pick * 8u;
    if table[row + 1u] > 0u {
        mark(table[row], bit);
    }
    let kind = id.y >> 30u;
    if kind > 2u {
        return;
    }
    let sub = id.y & 0x3fffffffu;
    let base = table[row + 2u + 2u * kind];
    let count = table[row + 3u + 2u * kind];
    if sub < count {
        mark(base + sub, bit);
    }
}
