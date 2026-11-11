//! Overhang analysis (S16 AM-02). AM-01 only defines [`AnalysisSettings`], which the print
//! setup record stores; AM-02 adds the classification and metrics.

use serde::{Deserialize, Serialize};

use crate::machine::Machine;

/// User-facing analysis settings stored in a print setup.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AnalysisSettings {
    /// Critical overhang angle α, rad.
    pub critical_angle: f64,
    /// Contact tolerance, mm; `None` → [`Machine::default_contact_tolerance`].
    pub contact_tolerance: Option<f64>,
    /// Support-estimate cell size, mm.
    pub support_cell: f64,
    /// Maximum support-estimate ray count.
    pub max_rays: u32,
}

impl AnalysisSettings {
    /// Defaults for `m`: its critical angle, automatic tolerance, 0.5 mm cells, 262 144 rays.
    pub fn for_machine(m: &Machine) -> Self {
        AnalysisSettings {
            critical_angle: m.default_critical_angle,
            contact_tolerance: None,
            support_cell: 0.5,
            max_rays: 262_144,
        }
    }
}
