//! Additive manufacturing: build orientation, overhang, supports, packing and slicer hand-off.
//!
//! Spec: S16. Lane: G.
//!
//! AM-01 provides [`machine`] (machine profiles, build volumes, the library and its TOML
//! format) plus the minimal [`place::Placement`] and [`overhang::AnalysisSettings`] types that
//! the print-setup record in `amg-features::am` stores. AM-02 and AM-03 extend those modules.
//! AM-10 adds [`import`] (machine profiles from OrcaSlicer, PrusaSlicer and Chitubox).

mod error;
pub mod handoff;
pub mod import;
pub mod machine;
pub mod overhang;
pub mod place;

pub use error::AmError;
