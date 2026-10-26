//! Raw FFI to Manifold's C API (`manifoldc`, Manifold 3.5.4 built by vcpkg with oneTBB), plus a
//! small safe layer that `amg-mesh::boolean` uses (S07 MESH-05, ADR-003).
//!
//! This is the documented FFI boundary: all `unsafe` for booleans lives in this crate.
//!
//! - [`ffi`] holds hand-written declarations for the subset of `manifoldc.h` we use. `build.rs`
//!   checks every name against the pinned header.
//! - [`Manifold`] and [`MeshGL64`] own one C object each and free it on drop.
//! - [`MeshData`] is the plain-Rust copy of a `MeshGL64`.
//!
//! Without the `native` feature nothing is linked; the safe layer still compiles, but calling
//! it fails at link time, so only `amg-mesh` with its `manifold` feature uses it.
//!
//! Memory model of the C API: every constructor takes a caller-allocated `mem` block
//! (`manifold_alloc_*`) and placement-constructs into it; `manifold_delete_*` runs the
//! destructor and frees the block. Accessors that return arrays copy into caller memory.

#![allow(unsafe_code)]

use std::fmt;

/// Raw declarations (subset of `manifoldc.h`, Manifold 3.5.4).
#[allow(missing_docs, non_camel_case_types)]
pub mod ffi {
    use std::ffi::c_void;

    /// Opaque `ManifoldManifold`.
    #[derive(Debug)]
    #[repr(C)]
    pub struct ManifoldManifold {
        _p: [u8; 0],
    }
    /// Opaque `ManifoldMeshGL64`.
    #[derive(Debug)]
    #[repr(C)]
    pub struct ManifoldMeshGL64 {
        _p: [u8; 0],
    }
    /// Opaque `ManifoldManifoldVec`.
    #[derive(Debug)]
    #[repr(C)]
    pub struct ManifoldManifoldVec {
        _p: [u8; 0],
    }

    /// `ManifoldMeshGL64Options` (`types.h`).
    #[derive(Debug)]
    #[repr(C)]
    pub struct ManifoldMeshGL64Options {
        pub run_indices: *mut u64,
        pub run_indices_length: usize,
        pub run_original_ids: *mut u32,
        pub run_original_ids_length: usize,
        pub merge_from_vert: *mut u64,
        pub merge_to_vert: *mut u64,
        pub merge_verts_length: usize,
        pub halfedge_tangents: *mut f64,
    }

    /// `ManifoldOpType`.
    pub const MANIFOLD_ADD: i32 = 0;
    pub const MANIFOLD_SUBTRACT: i32 = 1;
    pub const MANIFOLD_INTERSECT: i32 = 2;

    unsafe extern "C" {
        // Allocation / destruction.
        pub fn manifold_alloc_manifold() -> *mut ManifoldManifold;
        pub fn manifold_alloc_meshgl64() -> *mut ManifoldMeshGL64;
        pub fn manifold_alloc_manifold_vec() -> *mut ManifoldManifoldVec;
        pub fn manifold_delete_manifold(m: *mut ManifoldManifold);
        pub fn manifold_delete_meshgl64(m: *mut ManifoldMeshGL64);
        pub fn manifold_delete_manifold_vec(v: *mut ManifoldManifoldVec);

        // Construction.
        pub fn manifold_meshgl64(
            mem: *mut c_void,
            vert_props: *const f64,
            n_verts: usize,
            n_props: usize,
            tri_verts: *const u64,
            n_tris: usize,
        ) -> *mut ManifoldMeshGL64;
        pub fn manifold_meshgl64_w_options(
            mem: *mut c_void,
            vert_props: *const f64,
            n_verts: usize,
            n_props: usize,
            tri_verts: *const u64,
            n_tris: usize,
            options: *mut ManifoldMeshGL64Options,
        ) -> *mut ManifoldMeshGL64;
        pub fn manifold_empty(mem: *mut c_void) -> *mut ManifoldManifold;
        pub fn manifold_of_meshgl64(
            mem: *mut c_void,
            mesh: *const ManifoldMeshGL64,
        ) -> *mut ManifoldManifold;
        pub fn manifold_reserve_ids(n: u32) -> u32;
        pub fn manifold_status(m: *const ManifoldManifold) -> i32;
        pub fn manifold_original_id(m: *const ManifoldManifold) -> i32;
        pub fn manifold_num_tri(m: *const ManifoldManifold) -> usize;

        // Booleans.
        pub fn manifold_boolean(
            mem: *mut c_void,
            a: *const ManifoldManifold,
            b: *const ManifoldManifold,
            op: i32,
        ) -> *mut ManifoldManifold;
        pub fn manifold_manifold_empty_vec(mem: *mut c_void) -> *mut ManifoldManifoldVec;
        pub fn manifold_manifold_vec_push_back(
            ms: *mut ManifoldManifoldVec,
            m: *const ManifoldManifold,
        );
        pub fn manifold_batch_boolean(
            mem: *mut c_void,
            ms: *const ManifoldManifoldVec,
            op: i32,
        ) -> *mut ManifoldManifold;

        // Output.
        pub fn manifold_get_meshgl64(
            mem: *mut c_void,
            m: *const ManifoldManifold,
        ) -> *mut ManifoldMeshGL64;
        pub fn manifold_meshgl64_num_prop(m: *const ManifoldMeshGL64) -> usize;
        pub fn manifold_meshgl64_num_vert(m: *const ManifoldMeshGL64) -> usize;
        pub fn manifold_meshgl64_num_tri(m: *const ManifoldMeshGL64) -> usize;
        pub fn manifold_meshgl64_vert_properties_length(m: *const ManifoldMeshGL64) -> usize;
        pub fn manifold_meshgl64_tri_length(m: *const ManifoldMeshGL64) -> usize;
        pub fn manifold_meshgl64_merge_length(m: *const ManifoldMeshGL64) -> usize;
        pub fn manifold_meshgl64_run_index_length(m: *const ManifoldMeshGL64) -> usize;
        pub fn manifold_meshgl64_run_original_id_length(m: *const ManifoldMeshGL64) -> usize;
        pub fn manifold_meshgl64_face_id_length(m: *const ManifoldMeshGL64) -> usize;
        pub fn manifold_meshgl64_vert_properties(
            mem: *mut c_void,
            m: *const ManifoldMeshGL64,
        ) -> *mut f64;
        pub fn manifold_meshgl64_tri_verts(
            mem: *mut c_void,
            m: *const ManifoldMeshGL64,
        ) -> *mut u64;
        pub fn manifold_meshgl64_merge_from_vert(
            mem: *mut c_void,
            m: *const ManifoldMeshGL64,
        ) -> *mut u64;
        pub fn manifold_meshgl64_merge_to_vert(
            mem: *mut c_void,
            m: *const ManifoldMeshGL64,
        ) -> *mut u64;
        pub fn manifold_meshgl64_run_index(
            mem: *mut c_void,
            m: *const ManifoldMeshGL64,
        ) -> *mut u64;
        pub fn manifold_meshgl64_run_original_id(
            mem: *mut c_void,
            m: *const ManifoldMeshGL64,
        ) -> *mut u32;
        pub fn manifold_meshgl64_face_id(mem: *mut c_void, m: *const ManifoldMeshGL64) -> *mut u64;
    }
}

/// Boolean operation (`ManifoldOpType`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpType {
    /// Union.
    Add,
    /// Difference.
    Subtract,
    /// Intersection.
    Intersect,
}

impl OpType {
    fn raw(self) -> i32 {
        match self {
            OpType::Add => ffi::MANIFOLD_ADD,
            OpType::Subtract => ffi::MANIFOLD_SUBTRACT,
            OpType::Intersect => ffi::MANIFOLD_INTERSECT,
        }
    }
}

/// `ManifoldError` status of a [`Manifold`]; anything but [`Status::NoError`] is a failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// Valid.
    NoError,
    /// The input is not an oriented 2-manifold.
    NotManifold,
    /// A triangle references a vertex out of range.
    VertexIndexOutOfBounds,
    /// Properties contain NaN.
    NonFiniteVertex,
    /// Any other code (raw value).
    Other(i32),
}

impl Status {
    fn from_raw(v: i32) -> Status {
        match v {
            0 => Status::NoError,
            1 => Status::NonFiniteVertex,
            2 => Status::NotManifold,
            3 => Status::VertexIndexOutOfBounds,
            x => Status::Other(x),
        }
    }
}

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Status::NoError => f.write_str("no error"),
            Status::NotManifold => f.write_str("input is not an oriented 2-manifold"),
            Status::VertexIndexOutOfBounds => f.write_str("vertex index out of bounds"),
            Status::NonFiniteVertex => f.write_str("non-finite vertex"),
            Status::Other(x) => write!(f, "manifold error code {x}"),
        }
    }
}

/// An owned `ManifoldMeshGL64`.
pub struct MeshGL64(*mut ffi::ManifoldMeshGL64);

// SAFETY: a MeshGL64 is a plain value type (vectors); nothing is shared between instances.
unsafe impl Send for MeshGL64 {}

impl fmt::Debug for MeshGL64 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MeshGL64").finish_non_exhaustive()
    }
}

impl Drop for MeshGL64 {
    fn drop(&mut self) {
        // SAFETY: `self.0` came from `manifold_alloc_meshgl64` + a constructor and is freed once.
        unsafe { ffi::manifold_delete_meshgl64(self.0) }
    }
}

impl MeshGL64 {
    /// Builds a mesh from interleaved vertex properties (`n_props ≥ 3`, xyz first) and
    /// triangle vertex indices.
    ///
    /// # Panics
    /// If `n_props < 3` or the slice lengths are not multiples of `n_props` / 3.
    pub fn new(vert_props: &[f64], n_props: usize, tri_verts: &[u64]) -> MeshGL64 {
        assert!(n_props >= 3, "n_props must be at least 3");
        assert_eq!(vert_props.len() % n_props, 0, "vert_props length");
        assert_eq!(tri_verts.len() % 3, 0, "tri_verts length");
        // SAFETY: the pointers and lengths describe live slices; the C side copies them into
        // the freshly allocated block (`n_verts * n_props` doubles, `n_tris * 3` indices).
        let p = unsafe {
            let mem = ffi::manifold_alloc_meshgl64();
            ffi::manifold_meshgl64(
                mem.cast(),
                vert_props.as_ptr(),
                vert_props.len() / n_props,
                n_props,
                tri_verts.as_ptr(),
                tri_verts.len() / 3,
            )
        };
        MeshGL64(p)
    }

    /// As [`MeshGL64::new`], plus merge vectors (property vertex `merge_from[i]` is the same
    /// topological vertex as `merge_to[i]`, for vertices duplicated per property) and one run
    /// whose `runOriginalID` is `original_id` (from [`reserve_ids`]). Output runs of a boolean
    /// report that id for the triangles that came from this mesh.
    ///
    /// # Panics
    /// As [`MeshGL64::new`], or if the merge vectors differ in length.
    pub fn with_options(
        vert_props: &[f64],
        n_props: usize,
        tri_verts: &[u64],
        merge_from: &[u64],
        merge_to: &[u64],
        original_id: u32,
    ) -> MeshGL64 {
        assert!(n_props >= 3, "n_props must be at least 3");
        assert_eq!(vert_props.len() % n_props, 0, "vert_props length");
        assert_eq!(tri_verts.len() % 3, 0, "tri_verts length");
        assert_eq!(merge_from.len(), merge_to.len(), "merge vector lengths");
        let mut run_index = [0, tri_verts.len() as u64];
        let mut run_original_id = [original_id];
        let mut opts = ffi::ManifoldMeshGL64Options {
            run_indices: run_index.as_mut_ptr(),
            run_indices_length: 2,
            run_original_ids: run_original_id.as_mut_ptr(),
            run_original_ids_length: 1,
            merge_from_vert: merge_from.as_ptr().cast_mut(),
            merge_to_vert: merge_to.as_ptr().cast_mut(),
            merge_verts_length: merge_from.len(),
            halfedge_tangents: std::ptr::null_mut(),
        };
        // SAFETY: as in `new`; the options only point at live slices of the stated lengths,
        // which the C side copies (it never writes through them), and null fields are skipped.
        let p = unsafe {
            let mem = ffi::manifold_alloc_meshgl64();
            ffi::manifold_meshgl64_w_options(
                mem.cast(),
                vert_props.as_ptr(),
                vert_props.len() / n_props,
                n_props,
                tri_verts.as_ptr(),
                tri_verts.len() / 3,
                &mut opts,
            )
        };
        MeshGL64(p)
    }

    /// Copies the mesh out into plain Rust vectors.
    pub fn to_data(&self) -> MeshData {
        let m = self.0;
        // SAFETY: `m` is valid. Each `*_length` call gives the element count of the array the
        // matching accessor copies into `mem`; the vectors are allocated with exactly that
        // capacity and their length is set only after the copy.
        unsafe {
            let n_props = ffi::manifold_meshgl64_num_prop(m);
            MeshData {
                n_props,
                vert_props: copy_out(ffi::manifold_meshgl64_vert_properties_length(m), |mem| {
                    ffi::manifold_meshgl64_vert_properties(mem, m)
                }),
                tri_verts: copy_out(ffi::manifold_meshgl64_tri_length(m), |mem| {
                    ffi::manifold_meshgl64_tri_verts(mem, m)
                }),
                merge_from: copy_out(ffi::manifold_meshgl64_merge_length(m), |mem| {
                    ffi::manifold_meshgl64_merge_from_vert(mem, m)
                }),
                merge_to: copy_out(ffi::manifold_meshgl64_merge_length(m), |mem| {
                    ffi::manifold_meshgl64_merge_to_vert(mem, m)
                }),
                run_index: copy_out(ffi::manifold_meshgl64_run_index_length(m), |mem| {
                    ffi::manifold_meshgl64_run_index(mem, m)
                }),
                run_original_id: copy_out(
                    ffi::manifold_meshgl64_run_original_id_length(m),
                    |mem| ffi::manifold_meshgl64_run_original_id(mem, m),
                ),
                face_id: copy_out(ffi::manifold_meshgl64_face_id_length(m), |mem| {
                    ffi::manifold_meshgl64_face_id(mem, m)
                }),
            }
        }
    }
}

/// Reserves `n` consecutive fresh original ids (process-wide counter) and returns the first.
pub fn reserve_ids(n: u32) -> u32 {
    // SAFETY: no pointers; the counter is atomic on the C++ side.
    unsafe { ffi::manifold_reserve_ids(n) }
}

/// Copies `len` elements written by `fill(mem)` into a new `Vec`.
///
/// # Safety
/// `fill` must write exactly `len` initialised `T`s to `mem` (which has room for `len`).
unsafe fn copy_out<T: Copy>(
    len: usize,
    fill: impl FnOnce(*mut std::ffi::c_void) -> *mut T,
) -> Vec<T> {
    let mut v: Vec<T> = Vec::with_capacity(len);
    if len > 0 {
        fill(v.as_mut_ptr().cast());
        // SAFETY: the caller guarantees `fill` initialised `len` elements.
        unsafe { v.set_len(len) };
    }
    v
}

/// Plain copy of a `MeshGL64` (see Manifold's `MeshGL` docs for the field meanings).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeshData {
    /// Properties per vertex (xyz first).
    pub n_props: usize,
    /// `num_vert * n_props` interleaved properties.
    pub vert_props: Vec<f64>,
    /// `num_tri * 3` vertex indices, counter-clockwise from outside.
    pub tri_verts: Vec<u64>,
    /// Vertices that are topologically the same as `merge_to[i]`.
    pub merge_from: Vec<u64>,
    /// See `merge_from`.
    pub merge_to: Vec<u64>,
    /// Triangle-index (×3) boundaries of runs; `run_index.len() == run_original_id.len() + 1`.
    pub run_index: Vec<u64>,
    /// Original id (see [`Manifold::original_id`]) of the input each run came from.
    pub run_original_id: Vec<u32>,
    /// Per triangle: the face id it came from.
    pub face_id: Vec<u64>,
}

/// An owned `ManifoldManifold`.
pub struct Manifold(*mut ffi::ManifoldManifold);

// SAFETY: Manifold objects are immutable after construction and Manifold documents them as
// safe to use from several threads (internal lazy evaluation is guarded by a mutex).
unsafe impl Send for Manifold {}
// SAFETY: see `Send`; every operation here takes `&self` and returns a new object.
unsafe impl Sync for Manifold {}

impl fmt::Debug for Manifold {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Manifold").finish_non_exhaustive()
    }
}

impl Drop for Manifold {
    fn drop(&mut self) {
        // SAFETY: `self.0` came from `manifold_alloc_manifold` + a constructor and is freed once.
        unsafe { ffi::manifold_delete_manifold(self.0) }
    }
}

impl Manifold {
    fn wrap(p: *mut ffi::ManifoldManifold) -> Result<Manifold, Status> {
        let m = Manifold(p);
        match m.status() {
            Status::NoError => Ok(m),
            s => Err(s),
        }
    }

    /// Converts a mesh; fails when it is not an oriented, closed 2-manifold. Output runs trace
    /// back to the mesh's `runOriginalID` (see [`MeshGL64::with_options`]); without one,
    /// Manifold reserves a fresh id that the caller cannot see.
    pub fn from_mesh(mesh: &MeshGL64) -> Result<Manifold, Status> {
        // SAFETY: `mesh.0` is valid; the result is constructed into fresh memory.
        let p = unsafe {
            let mem = ffi::manifold_alloc_manifold();
            ffi::manifold_of_meshgl64(mem.cast(), mesh.0)
        };
        Manifold::wrap(p)
    }

    /// The empty manifold.
    pub fn empty() -> Manifold {
        // SAFETY: constructed into fresh memory.
        Manifold(unsafe { ffi::manifold_empty(ffi::manifold_alloc_manifold().cast()) })
    }

    /// Status of this manifold.
    pub fn status(&self) -> Status {
        // SAFETY: `self.0` is valid.
        Status::from_raw(unsafe { ffi::manifold_status(self.0) })
    }

    /// Manifold's own original id: −1 for anything built from a mesh or by an operation (the
    /// input mesh's `runOriginalID` is the original, see [`MeshGL64::with_options`]).
    pub fn original_id(&self) -> i32 {
        // SAFETY: `self.0` is valid.
        unsafe { ffi::manifold_original_id(self.0) }
    }

    /// Triangle count.
    pub fn num_tri(&self) -> usize {
        // SAFETY: `self.0` is valid.
        unsafe { ffi::manifold_num_tri(self.0) }
    }

    /// `self op other`.
    pub fn boolean(&self, other: &Manifold, op: OpType) -> Result<Manifold, Status> {
        // SAFETY: both operands are valid; the result is constructed into fresh memory.
        let p = unsafe {
            let mem = ffi::manifold_alloc_manifold();
            ffi::manifold_boolean(mem.cast(), self.0, other.0, op.raw())
        };
        Manifold::wrap(p)
    }

    /// `ms[0] op ms[1] op …` with Manifold's batch boolean (balanced tree for union).
    pub fn batch_boolean(ms: &[&Manifold], op: OpType) -> Result<Manifold, Status> {
        // SAFETY: the vector is created, filled with valid manifolds (copied by value on the C
        // side), used and freed here; the result is constructed into fresh memory.
        let p = unsafe {
            let vec = ffi::manifold_manifold_empty_vec(ffi::manifold_alloc_manifold_vec().cast());
            for m in ms {
                ffi::manifold_manifold_vec_push_back(vec, m.0);
            }
            let mem = ffi::manifold_alloc_manifold();
            let r = ffi::manifold_batch_boolean(mem.cast(), vec, op.raw());
            ffi::manifold_delete_manifold_vec(vec);
            r
        };
        Manifold::wrap(p)
    }

    /// The result mesh.
    pub fn to_mesh(&self) -> MeshGL64 {
        // SAFETY: `self.0` is valid; the result is constructed into fresh memory.
        let p = unsafe {
            let mem = ffi::manifold_alloc_meshgl64();
            ffi::manifold_get_meshgl64(mem.cast(), self.0)
        };
        MeshGL64(p)
    }
}

#[cfg(all(test, feature = "native"))]
mod tests {
    use super::*;

    /// Unit cube `[0,1]³` (+ `off`), outward CCW.
    fn cube(off: f64) -> MeshGL64 {
        let mut p = Vec::new();
        for i in 0..8u32 {
            p.extend([
                f64::from(i & 1) + off,
                f64::from((i >> 1) & 1) + off,
                f64::from((i >> 2) & 1) + off,
            ]);
        }
        let t: [u64; 36] = [
            0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4, 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4,
            6, 1, 3, 5, 3, 7, 5,
        ];
        MeshGL64::new(&p, 3, &t)
    }

    fn volume(d: &MeshData) -> f64 {
        let p = |i: u64| {
            let i = i as usize * d.n_props;
            [d.vert_props[i], d.vert_props[i + 1], d.vert_props[i + 2]]
        };
        d.tri_verts
            .chunks(3)
            .map(|t| {
                let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
                (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0]))
                    / 6.0
            })
            .sum()
    }

    #[test]
    fn cube_union_and_round_trip() {
        let id = reserve_ids(2);
        let tagged = |off: f64, id: u32| {
            let d = cube(off).to_data();
            MeshGL64::with_options(&d.vert_props, 3, &d.tri_verts, &[], &[], id)
        };
        let a = Manifold::from_mesh(&tagged(0.0, id)).unwrap();
        let b = Manifold::from_mesh(&tagged(0.5, id + 1)).unwrap();
        assert_eq!(a.num_tri(), 12);
        assert_eq!(a.original_id(), -1);
        let u = a.boolean(&b, OpType::Add).unwrap();
        let d = u.to_mesh().to_data();
        assert!((volume(&d) - (2.0 - 0.125)).abs() < 1e-12);
        assert_eq!(d.run_index.len(), d.run_original_id.len() + 1);
        assert_eq!(d.run_original_id, vec![id, id + 1]);
        assert_eq!(d.face_id.len(), d.tri_verts.len() / 3);
        let i = Manifold::batch_boolean(&[&a, &b], OpType::Intersect).unwrap();
        assert!((volume(&i.to_mesh().to_data()) - 0.125).abs() < 1e-12);
    }

    #[test]
    fn open_mesh_is_rejected() {
        let mut d = cube(0.0).to_data();
        d.tri_verts.truncate(33);
        let r = Manifold::from_mesh(&MeshGL64::new(&d.vert_props, 3, &d.tri_verts));
        assert_eq!(r.err(), Some(Status::NotManifold));
    }
}
