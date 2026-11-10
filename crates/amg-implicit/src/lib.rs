//! Implicit modeling: SDF expression graph, GPU evaluation, lattices and meshing.
//!
//! Spec: S08. Lane: C.
//!
//! Implemented so far:
//! - [`sdf::MeshSdf`] (IMP-03): exact narrow-band signed distance to a triangle mesh, an
//!   [`amg_mesh::iso::ScalarField`].

#![deny(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]

pub mod sdf;

pub use sdf::MeshSdf;
