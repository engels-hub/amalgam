//! Timeline features implementing `Feature`. Split by module; each domain lane owns its module.
//!
//! Spec: S04/S05/S06 + domain specs. Lane: shared by module.

pub mod am;
pub mod assembly;
pub mod construct;
pub mod dialog;
pub mod inspect;
pub mod io;
pub mod mesh;
pub mod sketch;
pub mod solid;

#[doc(hidden)]
pub mod __private {
    pub use amg_doc;
    pub use inventory;
}
