//! OBJ and MTL (IO-05): the reader, the writer and the MTL parser.
//!
//! Spec S12 §7, note `import-export-formats.md` §6. The reader is our own two-pass parallel
//! parser; `tobj` is only a dev-dependency oracle.

mod mtl;
mod parse;
mod read;
mod triangulate;
mod write;

pub use mtl::{Mtl, MtlMaterial, parse_mtl};
pub use read::read;
pub use write::write;

// Shared with the PLY reader (IO-13).
pub(crate) use triangulate::{Method, triangulate};

#[cfg(test)]
pub(crate) use read::{Tuning, read_tuned};

#[cfg(test)]
mod tests;
