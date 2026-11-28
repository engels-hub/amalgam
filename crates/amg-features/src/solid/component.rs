//! Component operations (S04 SOL-24): "New component", component (occurrence) patterns and
//! mirrors. Components and occurrences are authored state (S01 §6), so these are transaction
//! builders, not features: each returns the commands of one undo entry, checked by running
//! them on a copy of the design (id allocation is deterministic, as in S06 ASM-09's
//! builders). Apply them in order in one [`Transaction`](amg_doc::Transaction).
//!
//! - [`new_component_commands`]: `NewComponent` (placed once, at the identity, in the active
//!   component) + `AddFeature` with the new component as context. A dialog whose
//!   [`DialogRegistration::new_component`](crate::dialog::DialogRegistration) says so is
//!   committed this way by [`dialog::new_feature_commands`](crate::dialog::new_feature_commands).
//! - [`pattern_occurrences`] / [`mirror_occurrences`]: a [`Pattern`] or [`Mirror`] whose
//!   objects are [`PatternObjects::Occurrences`] becomes one `AddOccurrence` per instance (same
//!   component, same parent): placement `T_k · L` for a source with local transform `L`, where
//!   `T_k` is instance `k`'s transform of the pattern layout (resolved like the feature's, in
//!   the active component's space) or the reflection about the mirror plane. The occurrences
//!   share their component, so editing it changes every instance (Fusion's component pattern).
//!
//! Cross-component participants (a body operation in an assembly context on a body of a
//! sub-component) are handled by [`apply_body_op`](super::apply_body_op) itself.

use amg_core::{Mat3, Plane, Transform, Vec3};
use amg_doc::{
    Command, CommandError, ComponentId, ComputeMode, CreatedId, Design, Evaluated, FeatureBox,
    FeatureError, FeatureId, OccurrenceId, RecomputeCtx, apply_command,
};

use super::pattern::{Mirror, Pattern, PatternObjects};
use crate::construct::resolve::resolve_plane_input;

/// Why a component operation built no commands.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum ComponentOpError {
    /// The active component does not exist.
    #[error("component {0} does not exist")]
    NoComponent(ComponentId),
    /// The pattern or mirror does not copy occurrences.
    #[error("the objects are not occurrences")]
    NotOccurrences,
    /// No occurrence was selected.
    #[error("select at least one occurrence")]
    NoOccurrences,
    /// A selected occurrence does not exist.
    #[error("occurrence {0} does not exist")]
    NoOccurrence(OccurrenceId),
    /// A selected occurrence is not placed directly in the active component.
    #[error("occurrence {occ} is not in the active component {context}")]
    NotInContext {
        /// The occurrence.
        occ: OccurrenceId,
        /// The active component.
        context: ComponentId,
    },
    /// The layout or plane could not be evaluated.
    #[error(transparent)]
    Feature(#[from] FeatureError),
    /// The design refused a built command.
    #[error(transparent)]
    Command(#[from] CommandError),
}

impl amg_core::Diagnostic for ComponentOpError {}

/// `base` if no component has that name, else `base` followed by the smallest free number
/// (`"Component"`, `"Component1"`, …). An empty `base` is `"Component"`.
pub fn unique_component_name(design: &Design, base: &str) -> String {
    let base = if base.trim().is_empty() {
        "Component"
    } else {
        base.trim()
    };
    let taken = |n: &str| design.components().any(|(_, c)| c.name == n);
    if !taken(base) {
        return base.to_owned();
    }
    (1u64..)
        .map(|i| format!("{base}{i}"))
        .find(|n| !taken(n))
        .unwrap_or_else(|| base.to_owned())
}

/// The commands of "New component" (S04 §3): a new component named `name` (made unique with
/// [`unique_component_name`]) placed once, at the identity, in `parent`, then `feature` added
/// at the marker with the new component as its context. Because the placement is the
/// identity, profiles and references of `parent` keep their coordinates in the new component.
///
/// # Errors
/// [`ComponentOpError::NoComponent`] for a missing parent, [`ComponentOpError::Command`] when
/// the design refuses a command (for example in direct mode).
pub fn new_component_commands(
    design: &Design,
    parent: ComponentId,
    name: &str,
    feature: FeatureBox,
    feature_name: Option<String>,
) -> Result<Vec<Command>, ComponentOpError> {
    if design.component(parent).is_none() {
        return Err(ComponentOpError::NoComponent(parent));
    }
    let mut work = design.clone();
    let add_component = Command::NewComponent {
        parent,
        name: unique_component_name(design, name),
        transform: Transform::IDENTITY,
    };
    let created = apply_command(&mut work, add_component.clone())?;
    let component = created
        .iter()
        .find_map(|c| match c {
            CreatedId::Component(c) => Some(*c),
            _ => None,
        })
        .ok_or_else(|| CommandError::Invalid("NewComponent created no component".to_owned()))?;
    let add_feature = Command::AddFeature {
        at: None,
        feature,
        context: component,
        name: feature_name,
    };
    apply_command(&mut work, add_feature.clone())?;
    Ok(vec![add_component, add_feature])
}

/// A context for evaluating a layout outside recompute: the state `eval` (the latest
/// evaluation), `context` as the active component, captured positions at the end of the
/// timeline. The feature id is a placeholder that no timeline node has.
fn layout_ctx<'a>(
    design: &'a Design,
    eval: &'a Evaluated,
    context: ComponentId,
) -> RecomputeCtx<'a> {
    RecomputeCtx::new(
        design,
        eval,
        FeatureId(u64::MAX),
        context,
        ComputeMode::Full,
    )
}

/// The selected occurrences with their captured local transforms; each must be placed
/// directly in `context`.
fn sources(
    design: &Design,
    ctx: &RecomputeCtx<'_>,
    objects: &PatternObjects,
    context: ComponentId,
) -> Result<Vec<(OccurrenceId, ComponentId, Transform)>, ComponentOpError> {
    if design.component(context).is_none() {
        return Err(ComponentOpError::NoComponent(context));
    }
    let PatternObjects::Occurrences(occs) = objects else {
        return Err(ComponentOpError::NotOccurrences);
    };
    if occs.is_empty() {
        return Err(ComponentOpError::NoOccurrences);
    }
    let mut v: Vec<(OccurrenceId, ComponentId, Transform)> = Vec::with_capacity(occs.len());
    for &o in occs {
        if v.iter().any(|(x, _, _)| *x == o) {
            continue;
        }
        let occ = design
            .occurrence(o)
            .ok_or(ComponentOpError::NoOccurrence(o))?;
        if occ.parent != context {
            return Err(ComponentOpError::NotInContext { occ: o, context });
        }
        let local = ctx.occurrence_transform(o).unwrap_or(occ.transform);
        v.push((o, occ.component, local));
    }
    Ok(v)
}

/// `AddOccurrence` commands for every source and placement, checked on a copy.
fn add_occurrences(
    design: &Design,
    context: ComponentId,
    sources: &[(OccurrenceId, ComponentId, Transform)],
    placements: &[Transform],
) -> Result<Vec<Command>, ComponentOpError> {
    let mut work = design.clone();
    let mut commands = Vec::with_capacity(sources.len() * placements.len());
    for (_, component, local) in sources {
        for t in placements {
            let cmd = Command::AddOccurrence {
                parent: context,
                component: *component,
                transform: *t * *local,
                name: None,
            };
            apply_command(&mut work, cmd.clone())?;
            commands.push(cmd);
        }
    }
    Ok(commands)
}

/// The instance transforms of a component pattern (S04 §5, SOL-24) in `context`'s space, in
/// pattern order, without the seed and the suppressed instances: the same layout
/// ([`Pattern::instances`]) a body or feature pattern uses, evaluated against `eval`.
///
/// # Errors
/// [`ComponentOpError::Feature`] for an unresolved direction or axis or a bad count/spacing.
pub fn occurrence_pattern_transforms(
    design: &Design,
    eval: &Evaluated,
    pattern: &Pattern,
    context: ComponentId,
) -> Result<Vec<(u32, Transform)>, ComponentOpError> {
    let ctx = layout_ctx(design, eval, context);
    Ok(pattern.instances(&ctx)?)
}

/// The commands of a component pattern (S04 §5, SOL-24): for each selected occurrence (placed
/// in `context`) and each produced instance `k`, one `AddOccurrence` of the same component in
/// `context` at `T_k · L` (`L` its captured local transform). One undo entry; no timeline
/// feature. An empty list when the layout has no instance besides the seed.
///
/// # Errors
/// [`ComponentOpError`]: not occurrences, an empty or foreign selection, an unresolved layout,
/// or a refused command.
pub fn pattern_occurrences(
    design: &Design,
    eval: &Evaluated,
    pattern: &Pattern,
    context: ComponentId,
) -> Result<Vec<Command>, ComponentOpError> {
    let ctx = layout_ctx(design, eval, context);
    let src = sources(design, &ctx, &pattern.objects, context)?;
    let placements: Vec<Transform> = pattern
        .instances(&ctx)?
        .into_iter()
        .map(|(_, t)| t)
        .collect();
    add_occurrences(design, context, &src, &placements)
}

/// The reflection about `plane` (determinant −1).
pub fn reflection(plane: &Plane) -> Transform {
    let n: Vec3 = plane.normal.get();
    let col = |e: Vec3, c: f64| e - n * (2.0 * c);
    let m = Mat3::from_cols(col(Vec3::X, n.x), col(Vec3::Y, n.y), col(Vec3::Z, n.z));
    let p: Vec3 = plane.origin;
    Transform::from_mat3_translation(m, n * (2.0 * n.dot(p)))
}

/// The commands of a component mirror (S04 §5, SOL-24): for each selected occurrence (placed
/// in `context`), one `AddOccurrence` of the same component at `M · L`, `M` the reflection
/// about the mirror plane (resolved in `context`'s space). A mirrored occurrence is a
/// placement with determinant −1 (S01 `AddOccurrence` accepts "rigid or mirrored"); the
/// component's geometry is shared, not copied. `op` and `target` don't apply.
///
/// # Errors
/// As [`pattern_occurrences`], and an unresolved or non-planar plane.
pub fn mirror_occurrences(
    design: &Design,
    eval: &Evaluated,
    mirror: &Mirror,
    context: ComponentId,
) -> Result<Vec<Command>, ComponentOpError> {
    let ctx = layout_ctx(design, eval, context);
    let src = sources(design, &ctx, &mirror.objects, context)?;
    let plane = resolve_plane_input(&ctx, &mirror.plane)?;
    add_occurrences(design, context, &src, &[reflection(&plane)])
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::{Dir3, Point3};

    #[test]
    fn reflection_maps_points_across_the_plane_and_flips_orientation() {
        let n = Vec3::new(1.0, 2.0, -0.5).normalize();
        let plane = Plane {
            origin: Point3::new(3.0, -1.0, 2.0),
            x: Dir3::new_unchecked(n.any_orthonormal_vector()),
            y: Dir3::new_unchecked(n.cross(n.any_orthonormal_vector())),
            normal: Dir3::new_unchecked(n),
        };
        let m = reflection(&plane);
        assert!((m.matrix3.determinant() + 1.0).abs() < 1e-12);
        let q = Point3::new(7.0, 4.0, -9.0);
        let r = m.transform_point3(q);
        // The midpoint lies on the plane and the segment is along the normal.
        let mid = (q + r) * 0.5;
        assert!((mid - plane.origin).dot(n).abs() < 1e-12);
        assert!((r - q).cross(n).length() < 1e-12);
        // An involution.
        assert!((m.transform_point3(r) - q).length() < 1e-12);
    }

    #[test]
    fn unique_names_count_up() {
        let mut d = Design::new("names");
        assert_eq!(unique_component_name(&d, ""), "Component");
        let root = d.root();
        apply_command(
            &mut d,
            Command::NewComponent {
                parent: root,
                name: "Component".into(),
                transform: Transform::IDENTITY,
            },
        )
        .unwrap();
        assert_eq!(unique_component_name(&d, "Component"), "Component1");
        assert_eq!(unique_component_name(&d, " Bracket "), "Bracket");
    }
}
