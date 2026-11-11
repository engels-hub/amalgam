//! Inspection UI (lane A, S05): measure, analyses. The INSPECT tab itself is registered by the framework (`design`).
//!
//! Stub created by UI-01 (lane D). The owning lane fills in `register`; the framework never
//! needs to change for it (note §5).

use crate::module::{Registrar, UiModule};

inventory::submit! { UiModule { id: "inspect", register } }

/// Registers the module's workspaces, tabs, groups and panels.
pub fn register(_r: &mut Registrar<'_>) {}
