//! Additive manufacturing: build orientation, overhang, supports, packing and slicer hand-off.
//!
//! Spec: S16. Lane: G.
//!
//! AM-01 provides [`machine`] (machine profiles, build volumes, the library and its TOML
//! format) plus the minimal [`place::Placement`] and [`overhang::AnalysisSettings`] types that
//! the print-setup record in `amg-features::am` stores. AM-02 and AM-03 extend those modules.

mod error;
pub mod machine;
pub mod overhang;
pub mod place;

pub use error::AmError;
