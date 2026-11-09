//! Physical and appearance material library.
//!
//! Spec: S17. Lane: G. Design: `docs/notes/materials-data.md`.
//!
//! MAT-01 core: ids, [`Material`] and its sub-types (three elastic models with admissibility
//! checks, directional values, strength, plasticity, thermal, AM data), per-value provenance,
//! a minimal [`Appearance`], the TOML library format v1 ([`Library`]) and the embedded
//! [`builtin`] library. Values are `amg_core::units` typed quantities in the canonical
//! mm–t–s–K system; library files hold strings with units (`"1.24 g/cm^3"`).
//!
//! ```
//! use amg_materials::{builtin, default_material_id};
//! let pla = builtin().material(&default_material_id()).expect("built-in PLA");
//! assert!(pla.density.to_g_per_cm3() > 1.0);
//! ```

#![warn(missing_docs)]

mod appearance;
mod builtin;
mod elastic;
mod error;
mod format;
mod id;
mod library;
mod material;
mod provenance;

pub use appearance::{Appearance, Srgba8};
pub use builtin::{BUILTIN_LIBRARY_ID, builtin, default_material_id};
pub use elastic::{Elastic, EngineeringConstants, IsoPolicy};
pub use error::MaterialError;
pub use id::{AppearanceId, AppearanceKey, MaterialId, MaterialKey};
pub use library::{FORMAT_VERSION, Library, LibraryMeta};
pub use material::{
    AmInfo, AmProcess, Category, Cost, Directional, Hardness, Material, MaterialFamily, Mechanical,
    Plasticity, Strength, TempRange, Thermal,
};
pub use provenance::{
    PropPath, Provenance, Quality, Source, SourceKey, SourceKind, SpecimenOrientation, ValueProv,
};
