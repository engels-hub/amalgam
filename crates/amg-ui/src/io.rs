//! Import/export UI (lane G, S12 IO-01): File ▸ Import/Export entries.
//!
//! Stub created by UI-01 (lane D). The owning lane fills in `register`; the framework never
//! needs to change for it (note §5).

use crate::module::{Registrar, UiModule};

inventory::submit! { UiModule { id: "io", register } }

/// Registers the module's workspaces, tabs, groups and panels.
pub fn register(_r: &mut Registrar<'_>) {}
