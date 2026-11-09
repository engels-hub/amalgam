// amg-compute WGSL prelude (shared part). `alias real` is prepended per precision.

// Threads per workgroup along x; kernels declare `@workgroup_size(WG)`.
override WG: u32 = 256;

// "No index" marker.
const AMG_NONE: u32 = 0xffffffffu;

// Flat element index of a thread launched by `ComputeCtx::dispatch_1d`: `gid` is
// `@builtin(global_invocation_id)` and `nwg` is `@builtin(num_workgroups)`. The launch rounds
// up to whole workgroups, so kernels must ignore an index >= their element count.
fn amg_linear_id(gid: vec3<u32>, nwg: vec3<u32>) -> u32 {
    return gid.x + gid.y * (nwg.x * WG);
}
