//! Validity checks (S02 §9) and the validation policy's report type.

use crate::shape::SubShape;

/// Options of [`crate::BrepKernel::check`].
#[derive(Clone, Debug, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct CheckOpts {
    /// Also run `BOPAlgo_ArgumentAnalyzer`'s self-interference pass (slower; S20 request R5).
    /// BREP-10 adds that pass; until then it is ignored.
    pub self_interference: bool,
}

/// The outcome of a validity check.
#[derive(Clone, Debug, PartialEq)]
pub struct CheckReport {
    /// Whether the shape is valid.
    pub valid: bool,
    /// The problems found, per sub-shape (`None` for a shell, a solid or the whole shape). The
    /// text is the `BRepCheck_Status` name without its prefix, for example `NotClosed`, or a
    /// `BOPAlgo` check name. Problems of a wire are reported on its face.
    pub problems: Vec<(Option<SubShape>, String)>,
}

impl CheckReport {
    /// A report with no problems.
    pub fn valid() -> CheckReport {
        CheckReport {
            valid: true,
            problems: Vec::new(),
        }
    }

    /// The distinct problem names, sorted.
    pub fn problem_kinds(&self) -> Vec<&str> {
        let mut kinds: Vec<&str> = self.problems.iter().map(|(_, k)| k.as_str()).collect();
        kinds.sort_unstable();
        kinds.dedup();
        kinds
    }
}
