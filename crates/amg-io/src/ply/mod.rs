//! PLY (IO-13): the reader and the writer.
//!
//! Spec S12 §7, note `import-export-formats.md` §6. Both are our own code; `ply-rs-bw` is only
//! a dev-dependency oracle.
//!
//! Reader: ASCII and both binary byte orders, generic elements and properties, a header limit
//! of 64 KiB, vertex and face colours, UVs, polygons (ear clipped). A point cloud (vertices
//! without faces) is [`IoError::PointCloud`](crate::IoError::PointCloud). Writer:
//! `binary_little_endian` (default) or ASCII, `float` or `double` positions, `uchar` RGBA vertex
//! colours when present.

mod header;
mod read;
mod write;

pub use header::MAX_HEADER_BYTES;
pub use read::read;
pub use write::write;

#[cfg(test)]
pub(crate) use read::{Tuning, read_tuned};

#[cfg(test)]
mod tests;
