//! Material preferences (MAT-06): the default material for unassigned bodies.

use crate::builtin::default_material_id;
use crate::id::MaterialId;

/// User preferences of the materials lane. The application stores it in its preference file
/// (serde; missing fields take the defaults) and hands it to mass-property callers, which
/// resolve a body through `amg_doc::Design::mass_material`.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MaterialPrefs {
    /// Material used for the mass properties of a body without one (S17 decision 9,
    /// QUESTIONS #28). FEA never applies it: it refuses unassigned bodies.
    pub default_material: MaterialId,
}

impl Default for MaterialPrefs {
    /// `amg.builtin/pla.fff`.
    fn default() -> MaterialPrefs {
        MaterialPrefs {
            default_material: default_material_id(),
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "tests")]

    use super::*;

    #[test]
    fn defaults_to_builtin_pla_and_fills_missing_fields() {
        let p = MaterialPrefs::default();
        assert_eq!(p.default_material.as_str(), "amg.builtin/pla.fff");
        let back: MaterialPrefs = serde_json::from_str("{}").unwrap();
        assert_eq!(back, p);
        let s = serde_json::to_string(&p).unwrap();
        assert_eq!(s, r#"{"default_material":"amg.builtin/pla.fff"}"#);
        let other: MaterialPrefs =
            serde_json::from_str(r#"{"default_material":"amg.builtin/petg.fff"}"#).unwrap();
        assert_eq!(other.default_material.local(), "petg.fff");
        assert!(serde_json::from_str::<MaterialPrefs>(r#"{"default_material":"PLA"}"#).is_err());
    }
}
