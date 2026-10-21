//! Broken-mesh corpus (MESH-28, S07 §Testing, `docs/notes/mesh-processing.md` §6).
//!
//! Test support, behind the `corpus` feature:
//! - [`Manifest`] / [`Entry`]: `testdata/mesh/corpus.toml`, one entry per real model with SPDX
//!   licence, source URL, sha256, face count and defect flags. [`Entry::locate`] finds the file
//!   (committed through git-LFS, or fetched by `fetch.sh` / `fetch.ps1`) and skips what is absent.
//! - [`read_mesh`]: a small STL / PLY reader so the corpus can be loaded before `amg-io` exists.
//! - [`quick_stats`]: independent topology counts used to cross-check the manifest's defect flags.
//! - [`synth`]: seeded synthetic defect generator with ground truth, reproducible from
//!   `(shape, defects, seed)`.
//! - [`bench`]: displaced geodesic spheres at 100 k / 1 M / 10 M triangles and the offset sphere
//!   pair for the 1 M x 1 M boolean scenario.

mod manifest;
mod read;
mod stats;

pub mod bench;
pub mod synth;

pub use manifest::{
    CorpusError, Defect, Entry, FileStatus, MAX_COMMITTED_FILE_BYTES, MAX_COMMITTED_TOTAL_BYTES,
    Manifest, corpus_root, sha256_hex,
};
pub use read::{ReadError, read_mesh, read_ply, read_stl};
pub use stats::{QuickStats, quick_stats};
