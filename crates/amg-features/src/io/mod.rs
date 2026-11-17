//! Import/export document adapter (S12, lane G): the only place that sees both `amg-io` and
//! `amg-doc`.
//!
//! - [`ImportMeshFeature`] (`io.import_mesh` v1): the raw meshes of one imported file, kept as
//!   baked bodies (file numbers, no scale), plus the editable `unit`, `up_axis` and
//!   `placement`. `compute` applies them, so a unit change is an exact recompute with no
//!   re-read (Fusion's "Insert Mesh").
//! - [`import_commands`] builds an [`ImportPlan`] from an [`amg_io::Imported`] scene;
//!   [`ImportPlan::apply`] adds the feature, its baked payloads and the body names as one undo
//!   entry ([`amg_doc::Document::add_feature_baked`]).
//! - [`scene_from_document`] turns the selected bodies of a design back into an
//!   [`amg_io::Scene`] for `amg_io::export`.
//!
//! Call [`link`] from a binary or test so the feature registration is linked in.

mod export;
mod feature;
mod import;

pub use export::{ExportPrep, ExportRefinement, ExportSelection, scene_from_document};
pub use feature::{
    IMPORT_MESH_KIND, ImportMeshFeature, ImportSource, ImportedBodySpec, SLOT_PROPERTY_GROUP,
};
pub use import::{ImportChoices, ImportPlan, import_commands, raw_options};

use amg_doc::{BodyId, CommandError, FeatureError};

/// Why an import could not be turned into commands, or a design into a scene.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ImportPrepError {
    /// An `amg-io` error (scene validation, cancellation).
    #[error(transparent)]
    Io(#[from] amg_io::IoError),
    /// Solid bodies are selected; tessellated export is IO-09.
    #[error("{0} solid bodies need a tessellation quality (IO-09)")]
    NeedsTessellation(usize),
    /// An implicit or form body is selected.
    #[error("body {0:?} is an implicit/form body; convert it to a mesh first")]
    UnsupportedBody(BodyId),
    /// The selection has no exportable body.
    #[error("nothing selected to export")]
    EmptySelection,
    /// A feature error.
    #[error(transparent)]
    Feature(#[from] FeatureError),
    /// The scene was not read with [`raw_options`]: its numbers are already scaled.
    #[error(
        "the import was read in {0:?}; read it with raw_options so the feature keeps file numbers"
    )]
    NotRaw(amg_core::units::LengthUnit),
    /// The document refused the import transaction.
    #[error(transparent)]
    Command(#[from] CommandError),
}

impl amg_core::Diagnostic for ImportPrepError {
    fn is_cancelled(&self) -> bool {
        matches!(
            self,
            ImportPrepError::Io(amg_io::IoError::Cancelled)
                | ImportPrepError::Feature(FeatureError::Cancelled)
        )
    }
}

/// Forces the registrations of this module to be linked (registry `link()` convention).
pub fn link() {}

#[cfg(test)]
mod tests;
