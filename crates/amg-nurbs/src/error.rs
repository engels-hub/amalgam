//! [`NurbsError`]: every failure of the crate, with user hints through [`Diagnostic`].

use amg_core::{Cancelled, Diagnostic};

use crate::MAX_DEGREE;

/// Errors raised by `amg-nurbs`.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum NurbsError {
    /// The knot vector is empty, not finite, decreasing, unclamped or has a knot whose
    /// multiplicity is too high.
    #[error("invalid knot vector: {0}")]
    InvalidKnots(&'static str),
    /// `knots.len() != poles + degree + 1`, or too few poles for the degree.
    #[error("{poles} poles and {knots} knots do not fit degree {degree}")]
    CountMismatch {
        /// Degree of the curve or surface direction.
        degree: usize,
        /// Number of poles (in that direction).
        poles: usize,
        /// Number of knots.
        knots: usize,
    },
    /// Degree outside `1..=MAX_DEGREE`.
    #[error("degree {0} outside 1..={MAX_DEGREE}")]
    Degree(usize),
    /// A weight is zero, negative or not finite.
    #[error("weight {index} is not positive")]
    Weight {
        /// Flat index of the weight (row-major for surfaces).
        index: usize,
    },
    /// A parameter lies outside the domain.
    #[error("parameter {0} is outside the curve domain")]
    OutOfDomain(f64),
    /// Geometry that cannot be represented (non-finite poles, zero radius, empty sweep, …).
    #[error("degenerate geometry: {0}")]
    Degenerate(&'static str),
    /// A linear system could not be solved.
    #[error("singular system in {0}")]
    Singular(&'static str),
    /// An approximation did not reach its tolerance.
    #[error("approximation reached {achieved:e} mm, needed {tol:e} mm")]
    ToleranceNotMet {
        /// Requested tolerance, mm.
        tol: f64,
        /// Best achieved error, mm.
        achieved: f64,
    },
    /// The result would need more poles than allowed.
    #[error("needs {needed} poles, limit is {limit}")]
    TooManyPoles {
        /// Poles needed.
        needed: usize,
        /// Configured limit.
        limit: usize,
    },
    /// Curves or sections cannot be combined (different degree, knots, …).
    #[error("sections are incompatible: {0}")]
    Incompatible(String),
    /// A curve network profile and guide do not intersect.
    #[error("profile {profile} and guide {guide} miss by {distance:e} mm")]
    NetworkMiss {
        /// Profile index.
        profile: usize,
        /// Guide index.
        guide: usize,
        /// Closest distance, mm.
        distance: f64,
    },
    /// A profile crosses a guide more than once.
    #[error("profile {profile} crosses guide {guide} more than once")]
    NetworkMultiple {
        /// Profile index.
        profile: usize,
        /// Guide index.
        guide: usize,
    },
    /// The operation was cancelled.
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
}

impl Diagnostic for NurbsError {
    fn hint(&self) -> Option<String> {
        let h = match self {
            NurbsError::InvalidKnots(_) | NurbsError::CountMismatch { .. } => {
                "the spline data is malformed; re-create the curve or surface"
            }
            NurbsError::Degree(_) => "use a degree between 1 and 25",
            NurbsError::Weight { .. } => "all rational weights must be positive",
            NurbsError::OutOfDomain(_) => "pick a parameter inside the curve domain",
            NurbsError::Degenerate(_) => "check for zero-length or zero-radius input",
            NurbsError::Singular(_) | NurbsError::ToleranceNotMet { .. } => {
                "reduce section complexity"
            }
            NurbsError::TooManyPoles { .. } => {
                "reduce section complexity or loosen the fit tolerance"
            }
            NurbsError::Incompatible(_) => "rebuild the sections with matching segments",
            NurbsError::NetworkMiss { profile, guide, .. } => {
                return Some(format!("rail {guide} misses section {profile}"));
            }
            NurbsError::NetworkMultiple { profile, guide } => {
                return Some(format!("rail {guide} crosses section {profile} twice"));
            }
            NurbsError::Cancelled(_) => return None,
        };
        Some(h.to_owned())
    }

    fn severity(&self) -> amg_core::Severity {
        match self {
            NurbsError::Cancelled(_) => amg_core::Severity::Info,
            _ => amg_core::Severity::Error,
        }
    }

    fn is_cancelled(&self) -> bool {
        matches!(self, NurbsError::Cancelled(_))
    }
}
