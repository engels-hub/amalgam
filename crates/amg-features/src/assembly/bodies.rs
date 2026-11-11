//! ASM-09: create components from bodies, move bodies between components.

use std::collections::BTreeSet;
use std::sync::Arc;

use amg_core::Transform;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, CommandError, ComponentId, CreatedId,
    Dependency, Design, Evaluated, Feature, FeatureBox, FeatureError, FeatureOutput, OccurrenceId,
    RecomputeCtx, apply_command, register_feature,
};
use serde::{Deserialize, Serialize};

/// Registered kind of [`BodiesToComponentsFeature`].
pub const BODIES_TO_COMPONENTS_KIND: &str = "assembly.bodies_to_components";

/// Tolerance of the orthonormality and unit-determinant checks (same as occurrence transforms).
const RIGID_TOL: f64 = 1e-9;

/// One body re-homed by [`BodiesToComponentsFeature`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BodyMove {
    /// The body; it keeps its id.
    pub body: BodyId,
    /// The component it belongs to from this feature on.
    pub to: ComponentId,
    /// Maps the body's geometry from its old component space into `to`'s space (rigid or
    /// mirrored). Captured by the builder from the placements at that moment:
    /// `world(to)⁻¹ · world(old)`.
    pub transform: Transform,
}

/// "assembly.bodies_to_components" v1 (S06 ASM-09): moves existing bodies into other
/// components. Every body keeps its [`BodyId`], so later features that read it still resolve,
/// and its geometry is re-expressed in the new component's space.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BodiesToComponentsFeature {
    /// The moves, applied together.
    pub moves: Vec<BodyMove>,
}

impl Feature for BodiesToComponentsFeature {
    fn inputs(&self) -> Vec<Dependency> {
        self.moves
            .iter()
            .map(|m| Dependency::Body(m.body))
            .collect()
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self.moves.is_empty() {
            return Err(FeatureError::InvalidValue {
                field: "moves",
                msg: "no body to move".to_owned(),
            });
        }
        let mut seen = BTreeSet::new();
        for m in &self.moves {
            if !seen.insert(m.body) {
                return Err(FeatureError::InvalidValue {
                    field: "moves",
                    msg: format!("body {:?} is moved twice", m.body),
                });
            }
            check_rigid_or_mirror(&m.transform)?;
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut bodies = Vec::with_capacity(self.moves.len());
        for m in &self.moves {
            ctx.check_cancelled()?;
            if ctx.design().component(m.to).is_none() {
                return Err(FeatureError::InvalidValue {
                    field: "moves",
                    msg: format!("target component {} does not exist", m.to),
                });
            }
            let geom = transform_geom(ctx.body(m.body)?, &m.transform)?;
            bodies.push(BodyChange {
                id: m.body,
                change: BodyChangeKind::Moved {
                    component: m.to,
                    geom,
                    naming: ctx.naming(m.body).cloned(),
                },
            });
        }
        Ok(FeatureOutput {
            bodies,
            ..FeatureOutput::default()
        })
    }
}

register_feature!(
    BodiesToComponentsFeature,
    "assembly.bodies_to_components",
    version = 1
);

/// Finite, orthonormal linear part, `|det| = 1` within [`RIGID_TOL`].
fn check_rigid_or_mirror(t: &Transform) -> Result<(), FeatureError> {
    let bad = |msg: &str| {
        Err(FeatureError::InvalidValue {
            field: "transform",
            msg: msg.to_owned(),
        })
    };
    if !t.is_finite() {
        return bad("contains NaN or infinity");
    }
    let m = t.matrix3;
    let cols = [m.x_axis, m.y_axis, m.z_axis];
    for (i, a) in cols.iter().enumerate() {
        for (j, b) in cols.iter().enumerate().skip(i) {
            let want = if i == j { 1.0 } else { 0.0 };
            if (a.dot(*b) - want).abs() > RIGID_TOL {
                return bad("not rigid: the linear part must be orthonormal (no scale or shear)");
            }
        }
    }
    if (m.determinant().abs() - 1.0).abs() > RIGID_TOL {
        return bad("not rigid: |determinant| must be 1");
    }
    Ok(())
}

/// `geom` mapped by `t`. The identity keeps the same allocation (no geometry change event).
fn transform_geom(geom: &BodyGeom, t: &Transform) -> Result<BodyGeom, FeatureError> {
    if *t == Transform::IDENTITY {
        return Ok(geom.clone());
    }
    match geom {
        BodyGeom::Solid(s) => s
            .transformed(t)
            .map(BodyGeom::Solid)
            .map_err(|e| FeatureError::Kernel(e.to_string())),
        BodyGeom::Mesh(m) => {
            let mut m = (**m).clone();
            m.transform(t);
            Ok(BodyGeom::Mesh(Arc::new(m)))
        }
        BodyGeom::Implicit(_) | BodyGeom::Form(_) => Err(FeatureError::InvalidValue {
            field: "moves",
            msg: "implicit and form bodies can only move between components placed alike"
                .to_owned(),
        }),
    }
}

/// Why [`components_from_bodies`] or [`move_bodies`] could not build their commands.
#[derive(thiserror::Error, Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum AsmPrepError {
    /// The body is not in the evaluated state.
    #[error("body {0:?} does not exist")]
    NoBody(BodyId),
    /// The component is not in the design.
    #[error("component {0} does not exist")]
    NoComponent(ComponentId),
    /// The target component is placed several times (or not at all), so the world placement
    /// to keep is ambiguous.
    #[error("target component is placed {0} times; pick one occurrence")]
    AmbiguousTarget(usize),
    /// The body's component is placed several times, so the instance to keep is ambiguous.
    #[error("body {body:?} is shown {placements} times; its component must be placed once")]
    AmbiguousSource {
        /// The body.
        body: BodyId,
        /// Number of placements of its component.
        placements: usize,
    },
    /// A reference did not resolve.
    #[error(transparent)]
    Resolve(#[from] amg_doc::ResolveError),
    /// The design refused one of the built commands.
    #[error(transparent)]
    Command(#[from] CommandError),
}

impl amg_core::Diagnostic for AsmPrepError {}

/// Commands for one "Create Components from Bodies" transaction (S06 ASM-09).
///
/// Each body gets a new component placed once, at the identity, in the component the body is
/// in now (Fusion places it in the body's parent), named after the body when it has a name.
/// A single `assembly.bodies_to_components` node at the marker (root context) then moves
/// every body into its new component. Bodies keep their ids and world placements.
///
/// Apply the commands in order in one [`Transaction`](amg_doc::Transaction) on the same
/// design: the ids of the new components are predicted by running the commands on a copy
/// (id allocation is deterministic). Duplicate bodies are ignored; no bodies give no commands.
///
/// # Errors
/// [`AsmPrepError::NoBody`] for a body that is not evaluated, [`AsmPrepError::Command`] when
/// the design refuses a command (for example in direct mode).
pub fn components_from_bodies(
    design: &Design,
    eval: &Evaluated,
    bodies: &[BodyId],
) -> Result<Vec<Command>, AsmPrepError> {
    let bodies = dedup(bodies);
    let mut work = design.clone();
    let mut commands = Vec::with_capacity(bodies.len() + 1);
    let mut moves = Vec::with_capacity(bodies.len());
    let existing = design.components().count();
    for (i, body) in bodies.into_iter().enumerate() {
        let parent = eval.body(body).ok_or(AsmPrepError::NoBody(body))?.component;
        if design.component(parent).is_none() {
            return Err(AsmPrepError::NoComponent(parent));
        }
        let name = design
            .body_props(body)
            .name
            .unwrap_or_else(|| format!("Component{}", existing + i));
        let cmd = Command::NewComponent {
            parent,
            name,
            transform: Transform::IDENTITY,
        };
        let created = apply_command(&mut work, cmd.clone())?;
        let to = created
            .iter()
            .find_map(|c| match c {
                CreatedId::Component(c) => Some(*c),
                _ => None,
            })
            .ok_or_else(|| {
                AsmPrepError::Command(CommandError::Invalid(
                    "NewComponent created no component".to_owned(),
                ))
            })?;
        commands.push(cmd);
        moves.push(BodyMove {
            body,
            to,
            transform: Transform::IDENTITY,
        });
    }
    if moves.is_empty() {
        return Ok(Vec::new());
    }
    let add = add_feature(design, moves);
    apply_command(&mut work, add.clone())?;
    commands.push(add);
    Ok(commands)
}

/// Commands for "Move to Component" (S06 ASM-09): one `assembly.bodies_to_components` node at
/// the marker that moves `bodies` into `to`, each keeping its world placement as currently
/// evaluated (`Evaluated::placements`). Bodies already in `to` are skipped; if none is left
/// there are no commands.
///
/// # Errors
/// [`AsmPrepError::NoComponent`] if `to` does not exist, [`AsmPrepError::AmbiguousTarget`] if
/// it is not placed exactly once, [`AsmPrepError::NoBody`] / [`AsmPrepError::AmbiguousSource`]
/// for a body that is not evaluated or whose component is placed more than once, and
/// [`AsmPrepError::Command`] when the design refuses the command.
pub fn move_bodies(
    design: &Design,
    eval: &Evaluated,
    bodies: &[BodyId],
    to: ComponentId,
) -> Result<Vec<Command>, AsmPrepError> {
    if design.component(to).is_none() {
        return Err(AsmPrepError::NoComponent(to));
    }
    let to_world = single_world(design, eval, to).map_err(AsmPrepError::AmbiguousTarget)?;
    let to_inv = to_world.inverse();
    let mut moves = Vec::new();
    for body in dedup(bodies) {
        let from = eval.body(body).ok_or(AsmPrepError::NoBody(body))?.component;
        if from == to {
            continue;
        }
        if design.component(from).is_none() {
            return Err(AsmPrepError::NoComponent(from));
        }
        let from_world = single_world(design, eval, from)
            .map_err(|placements| AsmPrepError::AmbiguousSource { body, placements })?;
        moves.push(BodyMove {
            body,
            to,
            transform: to_inv * from_world,
        });
    }
    if moves.is_empty() {
        return Ok(Vec::new());
    }
    let add = add_feature(design, moves);
    apply_command(&mut design.clone(), add.clone())?;
    Ok(vec![add])
}

fn add_feature(design: &Design, moves: Vec<BodyMove>) -> Command {
    Command::AddFeature {
        at: None,
        feature: FeatureBox::new(BodiesToComponentsFeature { moves }),
        context: design.root(),
        name: None,
    }
}

/// World transform of the only placement of `c` (solved placements, else authored), or the
/// number of placements when it is not exactly one.
fn single_world(design: &Design, eval: &Evaluated, c: ComponentId) -> Result<Transform, usize> {
    let paths: Vec<Vec<OccurrenceId>> = design.occurrence_paths(c);
    let [path] = paths.as_slice() else {
        return Err(paths.len());
    };
    eval.world_transform(path)
        .or_else(|| design.authored_world(path))
        .ok_or(0)
}

fn dedup(bodies: &[BodyId]) -> Vec<BodyId> {
    let mut seen = BTreeSet::new();
    bodies.iter().copied().filter(|b| seen.insert(*b)).collect()
}
