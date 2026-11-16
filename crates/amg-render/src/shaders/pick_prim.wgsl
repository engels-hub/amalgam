// Face pick with `@builtin(primitive_index)` (RND-04): appended to the pick module, which then
// starts with `enable primitive_index;`. Built only on devices with `PRIMITIVE_INDEX`.

@fragment
fn fs_pick_prim(
    in: PickVs,
    @builtin(front_facing) front: bool,
    @builtin(primitive_index) primitive: u32,
) -> PickOut {
    let normal = cross(dpdx(in.rte), dpdy(in.rte));
    return face_out(in, front, primitive, normal);
}
