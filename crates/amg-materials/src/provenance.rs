//! Per-value provenance (note §3.2).

use std::borrow::Borrow;
use std::collections::BTreeMap;

/// Key of a `[[source]]` record.
#[derive(
    Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct SourceKey(pub String);

/// Dotted property path, e.g. `density`, `mechanical.elastic.youngs`,
/// `mechanical.strength.ultimate_tensile.z`. Orthotropic arrays use `e.1`, `nu.12`, `g.23`.
#[derive(
    Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct PropPath(pub String);

impl Borrow<str> for PropPath {
    fn borrow(&self) -> &str {
        &self.0
    }
}

impl Borrow<str> for SourceKey {
    fn borrow(&self) -> &str {
        &self.0
    }
}

/// Where a material's values come from.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Provenance {
    /// Source of every value without its own entry (quality `measured`).
    pub default_source: Option<SourceKey>,
    /// Per-value overrides, keyed by property path.
    pub values: BTreeMap<PropPath, ValueProv>,
    /// Copies of the `Source` records used, so a document copy keeps its citations.
    pub sources: BTreeMap<SourceKey, Source>,
}

/// Provenance of one value.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ValueProv {
    /// Source record; `None` means the material's `default_source`.
    pub source: Option<SourceKey>,
    /// How trustworthy the value is.
    pub quality: Quality,
    /// ISO/ASTM 52921 orientation of the test specimen.
    pub specimen: Option<SpecimenOrientation>,
    /// Test standard, e.g. `ISO 527-2`, `ASTM D638`.
    pub standard: Option<String>,
    /// Condition or derivation note.
    pub note: Option<String>,
}

impl ValueProv {
    /// True if this entry says nothing beyond the default (plain string in the file).
    pub fn is_default(&self) -> bool {
        self.source.is_none()
            && self.quality == Quality::Measured
            && self.specimen.is_none()
            && self.standard.is_none()
            && self.note.is_none()
    }
}

/// Quality flag of a value.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Quality {
    /// Verbatim from the source, unit-converted only.
    Measured,
    /// Computed from other values; the note gives the formula.
    Derived,
    /// Handbook or range value; the note gives the range.
    Typical,
    /// Engineering judgement (warning in the UI).
    Estimate,
}

impl Quality {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Quality::Measured => "measured",
            Quality::Derived => "derived",
            Quality::Typical => "typical",
            Quality::Estimate => "estimate",
        }
    }

    pub(crate) fn from_str(s: &str) -> Option<Quality> {
        Some(match s {
            "measured" => Quality::Measured,
            "derived" => Quality::Derived,
            "typical" => Quality::Typical,
            "estimate" => Quality::Estimate,
            _ => return None,
        })
    }
}

/// ISO/ASTM 52921 orthogonal orientation notation; the first letter is the specimen's long
/// (load) axis. Serialized as `"XYZ"`, `"ZXY"`, ...
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum SpecimenOrientation {
    /// Long axis X, then Y, then Z.
    Xyz,
    /// X, Z, Y.
    Xzy,
    /// Y, X, Z.
    Yxz,
    /// Y, Z, X.
    Yzx,
    /// Z, X, Y.
    Zxy,
    /// Z, Y, X.
    Zyx,
}

impl SpecimenOrientation {
    /// True if the load axis is Z (the only orientation that may fill a `z` slot).
    pub fn is_z(self) -> bool {
        matches!(self, SpecimenOrientation::Zxy | SpecimenOrientation::Zyx)
    }

    /// `"XYZ"`, `"ZXY"`, ...
    pub fn as_str(self) -> &'static str {
        match self {
            SpecimenOrientation::Xyz => "XYZ",
            SpecimenOrientation::Xzy => "XZY",
            SpecimenOrientation::Yxz => "YXZ",
            SpecimenOrientation::Yzx => "YZX",
            SpecimenOrientation::Zxy => "ZXY",
            SpecimenOrientation::Zyx => "ZYX",
        }
    }

    /// Inverse of [`as_str`](SpecimenOrientation::as_str).
    pub fn parse(s: &str) -> Option<SpecimenOrientation> {
        Some(match s {
            "XYZ" => SpecimenOrientation::Xyz,
            "XZY" => SpecimenOrientation::Xzy,
            "YXZ" => SpecimenOrientation::Yxz,
            "YZX" => SpecimenOrientation::Yzx,
            "ZXY" => SpecimenOrientation::Zxy,
            "ZYX" => SpecimenOrientation::Zyx,
            _ => return None,
        })
    }
}

/// A cited document (datasheet, standard, handbook table, paper).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Source {
    /// Key used by `src` fields.
    pub key: SourceKey,
    /// Kind of document.
    pub kind: SourceKind,
    /// Document title.
    pub title: String,
    /// Publisher or manufacturer.
    pub publisher: Option<String>,
    /// Document version.
    pub version: Option<String>,
    /// Publication date.
    pub date: Option<String>,
    /// Where it was found.
    pub url: Option<String>,
    /// Access date.
    pub accessed: Option<String>,
    /// Licence of the cited data, if any.
    pub licence: Option<String>,
}

/// Kind of a [`Source`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceKind {
    /// Manufacturer datasheet.
    Datasheet,
    /// A standard.
    Standard,
    /// Handbook table.
    Handbook,
    /// A paper.
    Paper,
    /// Open community database with a verified licence.
    CommunityDb,
    /// Measured by the user.
    UserMeasured,
    /// Engineering judgement.
    Estimate,
}

impl SourceKind {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            SourceKind::Datasheet => "datasheet",
            SourceKind::Standard => "standard",
            SourceKind::Handbook => "handbook",
            SourceKind::Paper => "paper",
            SourceKind::CommunityDb => "community_db",
            SourceKind::UserMeasured => "user_measured",
            SourceKind::Estimate => "estimate",
        }
    }

    pub(crate) fn from_str(s: &str) -> Option<SourceKind> {
        Some(match s {
            "datasheet" => SourceKind::Datasheet,
            "standard" => SourceKind::Standard,
            "handbook" => SourceKind::Handbook,
            "paper" => SourceKind::Paper,
            "community_db" => SourceKind::CommunityDb,
            "user_measured" => SourceKind::UserMeasured,
            "estimate" => SourceKind::Estimate,
            _ => return None,
        })
    }
}
