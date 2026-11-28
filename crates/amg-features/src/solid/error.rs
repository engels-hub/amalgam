//! S04 failures (§7, note §6): [`SolidError`] and the kernel-failure highlight.

use amg_brep::KernelError;
use amg_core::Diagnostic as _;
use amg_doc::naming::topo_ref_at;
use amg_doc::{BodyId, Diagnostic, EntityRef, FeatureError, FeatureOutput, RecomputeCtx, TopoKind};

/// Every S04 failure. Input problems become [`FeatureError::InvalidValue`], the rest
/// [`FeatureError::Kernel`].
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum SolidError {
    /// A participant or input body is not a B-rep solid.
    #[error("body is not a solid")]
    WrongBodyKind,
    /// A participant is in another component than the feature.
    #[error("body {0:?} is in another component")]
    OtherComponent(BodyId),
    /// A participant in a sub-component placed more than once below the feature's component,
    /// so the tool's placement relative to it is ambiguous (SOL-24).
    #[error(
        "body {body:?} is in a component placed {placements} times below the feature's component"
    )]
    AmbiguousPlacement {
        /// The body.
        body: BodyId,
        /// Its number of placements below the feature's component.
        placements: usize,
    },
    /// A participant seen through a mirrored occurrence (SOL-24): a reflected tool would need
    /// new geometry with flipped faces.
    #[error("body {0:?} is placed through a mirrored occurrence")]
    MirroredPlacement(BodyId),
    /// A join whose touched participants are in different components (SOL-24).
    #[error("a join can't fuse bodies of different components")]
    MixedComponents,
    /// A cut whose tool touches no participant.
    #[error("nothing to cut: the tool touches no participant")]
    NothingToCut,
    /// An intersect whose every common part is empty.
    #[error("nothing to intersect: every common part is empty")]
    NothingToIntersect,
    /// Profiles of one feature on non-parallel planes.
    #[error("profiles are not parallel")]
    NonParallelProfiles,
    /// A to-object target that does not cover the profile.
    #[error("the target does not cover the profile")]
    TargetMissed,
    /// "Extend faces" on a face type that can't be extended.
    #[error("this face type can't be extended")]
    ExtendUnsupported,
    /// A revolve profile crossing its axis.
    #[error("the profile crosses the axis")]
    AxisCrossesProfile,
    /// A split tool that does not split the body.
    #[error("the tool does not split the body")]
    NoSplit,
    /// A pattern seed without a [`crate::solid::ToolRegistration`].
    #[error("feature kind `{0}` can't be patterned")]
    NotPatternable(String),
    /// An option a later WP implements.
    #[error("{what} is not available in this build yet")]
    NotYet {
        /// The option.
        what: &'static str,
    },
    /// A thread designation the standards tables don't have.
    #[error("unknown thread `{0}`")]
    UnknownThread(String),
    /// A field value the feature can't use.
    #[error("{field}: {msg}")]
    Invalid {
        /// The field.
        field: &'static str,
        /// Why.
        msg: String,
    },
}

impl SolidError {
    /// The field an input problem is about (`None` for kernel-side failures).
    fn field(&self) -> Option<&'static str> {
        match self {
            SolidError::WrongBodyKind
            | SolidError::OtherComponent(_)
            | SolidError::AmbiguousPlacement { .. }
            | SolidError::MirroredPlacement(_)
            | SolidError::MixedComponents => Some("participants"),
            SolidError::NonParallelProfiles => Some("profiles"),
            SolidError::ExtendUnsupported | SolidError::TargetMissed => Some("extent"),
            SolidError::AxisCrossesProfile => Some("axis"),
            SolidError::NotPatternable(_) => Some("objects"),
            SolidError::NotYet { .. } => Some("options"),
            SolidError::UnknownThread(_) => Some("size"),
            SolidError::Invalid { field, .. } => Some(field),
            SolidError::NoSplit => Some("tool"),
            SolidError::NothingToCut | SolidError::NothingToIntersect => None,
        }
    }
}

impl amg_core::Diagnostic for SolidError {
    fn hint(&self) -> Option<String> {
        let h = match self {
            SolidError::WrongBodyKind => {
                "Convert the mesh to a solid first, or use a mesh boolean in the Mesh workspace."
            }
            SolidError::OtherComponent(_) => {
                "Activate the component that holds the body, or one of its parents."
            }
            SolidError::AmbiguousPlacement { .. } => {
                "Activate the component that holds the body, so the placement is unique."
            }
            SolidError::MirroredPlacement(_) => {
                "Activate the mirrored component and run the feature there."
            }
            SolidError::MixedComponents => {
                "Join bodies of one component at a time, or move them into one component."
            }
            SolidError::NothingToCut => {
                "Move the profile or the distance so the tool overlaps a body."
            }
            SolidError::NothingToIntersect => "The tool and the bodies don't overlap.",
            SolidError::TargetMissed => "Turn on Extend faces.",
            SolidError::AxisCrossesProfile => "The profile crosses the axis; move the axis.",
            SolidError::NoSplit => "Extend the tool so it crosses the body.",
            _ => return None,
        };
        Some(h.to_owned())
    }
}

impl From<SolidError> for FeatureError {
    fn from(e: SolidError) -> FeatureError {
        match e.field() {
            Some(field) => FeatureError::InvalidValue {
                field,
                msg: e.to_string(),
            },
            None => FeatureError::Kernel(e.to_string()),
        }
    }
}

/// A kernel failure as a feature error (§7): [`FeatureError::Cancelled`] for a cancel, else
/// [`FeatureError::Kernel`] with the kernel's message and hint. The offending sub-shapes, in the
/// index space of `body`'s input shape, become `EntityRef`s (`topo_ref_at`), are marked for the
/// node's error highlight ([`RecomputeCtx::highlight`]) and are listed in a diagnostic pushed
/// to `out` (for callers that turn the failure into a warning).
pub fn kernel_failure(
    ctx: &RecomputeCtx<'_>,
    body: Option<BodyId>,
    e: KernelError,
    out: &mut FeatureOutput,
) -> FeatureError {
    if matches!(e, KernelError::Cancelled) {
        return FeatureError::Cancelled;
    }
    let entities: Vec<EntityRef> = body
        .map(|b| {
            e.offending()
                .iter()
                .filter_map(|s| {
                    topo_ref_at(
                        ctx.before(),
                        ctx.kernel(),
                        b,
                        TopoKind::from_sub_kind(s.kind),
                        s.index,
                    )
                })
                .map(EntityRef::Topo)
                .collect()
        })
        .unwrap_or_default();
    let message = match e.hint() {
        Some(h) => format!("{e}. {h}"),
        None => e.to_string(),
    };
    ctx.highlight(entities.iter().cloned());
    out.warnings.push(Diagnostic {
        message: message.clone(),
        cause: None,
        entities,
    });
    FeatureError::Kernel(message)
}

/// An input problem as [`FeatureError::InvalidValue`] with the error's hint appended to the
/// message (`TargetMissed` → "… Turn on Extend faces."), so the dialog shows the fix.
pub(crate) fn with_hint(e: &SolidError) -> FeatureError {
    let field = e.field().unwrap_or("options");
    let msg = match e.hint() {
        Some(h) => format!("{e}. {h}"),
        None => e.to_string(),
    };
    FeatureError::InvalidValue { field, msg }
}

/// `e` with its offending sub-shapes, given in the result space of an earlier step whose
/// history is `h`, mapped back to that step's input (operand 0) space, so [`kernel_failure`]
/// can highlight them on the input body (multi-step features: shell both, two-sided draft,
/// thicken). A sub-shape with no input is dropped.
pub(crate) fn remap_offending(e: KernelError, h: &amg_brep::ShapeHistory) -> KernelError {
    let map = |subs: Vec<amg_brep::SubShape>| -> Vec<amg_brep::SubShape> {
        let mut out: Vec<amg_brep::SubShape> = Vec::new();
        for s in subs {
            for (i, _) in h.sources_of(s) {
                if i.operand == 0 && i.sub.kind == s.kind && !out.contains(&i.sub) {
                    out.push(i.sub);
                }
            }
        }
        out
    };
    match e {
        KernelError::InvalidInput {
            what,
            reason,
            offending,
        } => KernelError::InvalidInput {
            what,
            reason,
            offending: map(offending),
        },
        KernelError::OpFailed {
            op,
            message,
            offending,
        } => KernelError::OpFailed {
            op,
            message,
            offending: map(offending),
        },
        KernelError::LocalOpFailed {
            op,
            message,
            offending,
            partial,
        } => KernelError::LocalOpFailed {
            op,
            message,
            offending: map(offending),
            partial,
        },
        other => other,
    }
}

/// A kernel error without sub-shapes worth highlighting.
pub(crate) fn kerr(e: KernelError) -> FeatureError {
    if matches!(e, KernelError::Cancelled) {
        return FeatureError::Cancelled;
    }
    FeatureError::Kernel(e.to_string())
}
