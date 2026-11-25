//! What the assembly commands and browser menus do, on explicit targets. Every edit is one
//! [`CommitBatch`] (one undo entry) of `amg-doc` / `amg-features::assembly` commands.

use std::collections::BTreeSet;

use amg_core::Transform;
use amg_doc::{BodyId, Command, ComponentId, Design, DesignMode, OccurrenceId};
use amg_features::assembly::{components_from_bodies, move_bodies};

use super::state::{AssemblyLayer, Clip};
use crate::app::{AppCx, CmdEnv, CommitBatch};
use crate::notify::Notification;
use crate::pick::PickTarget;

/// The selected occurrences, in selection order.
pub fn selected_occurrences(env: &CmdEnv<'_>) -> Vec<OccurrenceId> {
    env.selection
        .items()
        .iter()
        .filter_map(|i| match i.target {
            PickTarget::Occurrence(o) => env.design.occurrence(o).map(|_| o),
            _ => None,
        })
        .collect()
}

/// The selected whole bodies, in selection order.
pub fn selected_bodies(env: &CmdEnv<'_>) -> Vec<BodyId> {
    let mut seen = BTreeSet::new();
    env.selection
        .items()
        .iter()
        .filter_map(|i| match i.target {
            PickTarget::Body {
                body, sub: None, ..
            } => env.evaluated.body(body).map(|_| body),
            _ => None,
        })
        .filter(|b| seen.insert(*b))
        .collect()
}

/// What a command on `clicked` acts on: the whole selection if the clicked item is part of
/// it, else the clicked item alone.
pub(super) fn targets<T: PartialEq + Copy>(selected: Vec<T>, clicked: T) -> Vec<T> {
    if selected.contains(&clicked) {
        selected
    } else {
        vec![clicked]
    }
}

/// The display name of a component (the design name for the root).
pub(super) fn component_name(design: &Design, c: ComponentId) -> String {
    if c == design.root() {
        return design.meta().name.clone();
    }
    design
        .component(c)
        .map(|k| k.name.clone())
        .unwrap_or_default()
}

/// True if `target` is `from` or is placed anywhere inside it (so placing `from` in `target`
/// would make a component contain itself).
fn reaches(design: &Design, from: ComponentId, target: ComponentId) -> bool {
    let mut seen = BTreeSet::new();
    let mut stack = vec![from];
    while let Some(c) = stack.pop() {
        if c == target {
            return true;
        }
        if !seen.insert(c) {
            continue;
        }
        if let Some(def) = design.component(c) {
            stack.extend(
                def.children
                    .iter()
                    .filter_map(|o| design.occurrence(*o))
                    .map(|o| o.component),
            );
        }
    }
    false
}

fn submit(cx: &mut AppCx<'_>, label: &str, commands: Vec<Command>) {
    if !commands.is_empty() {
        cx.submit(CommitBatch {
            label: label.to_owned(),
            commands,
        });
    }
}

/// True if the evaluated state matches the design; otherwise says so. The builders that read
/// positions or bodies must not run on an older state.
fn require_evaluated(cx: &mut AppCx<'_>, what: &str) -> bool {
    let current = cx.doc_tab().document.is_evaluated();
    if !current {
        cx.notify(Notification::info(
            what,
            "The design is still computing; try again when it is done.",
        ));
    }
    current
}

/// New Component: an empty component placed in `parent` at the identity, named "Component<n>"
/// with the smallest free `n`.
pub(super) fn new_component(cx: &mut AppCx<'_>, parent: ComponentId) {
    let name = {
        let design = cx.env().design;
        let used: BTreeSet<&str> = design.components().map(|(_, c)| c.name.as_str()).collect();
        let mut n = 1_usize;
        while used.contains(format!("Component{n}").as_str()) {
            n += 1;
        }
        format!("Component{n}")
    };
    submit(
        cx,
        "New Component",
        vec![Command::NewComponent {
            parent,
            name,
            transform: Transform::IDENTITY,
        }],
    );
}

/// Makes `component` the active one (session state: no undo entry).
pub(super) fn activate(cx: &mut AppCx<'_>, component: ComponentId) {
    if cx.env().active_component == component {
        return;
    }
    // No command is applied, so this adds nothing to the undo stack.
    cx.apply_document("Activate Component", move |doc| {
        doc.set_active_component(component)
    });
}

/// Grounds or ungrounds those of `occs` that are not already in that state.
pub(super) fn set_grounded(cx: &mut AppCx<'_>, occs: &[OccurrenceId], grounded: bool) {
    let commands = {
        let design = cx.env().design;
        occs.iter()
            .filter(|o| {
                design
                    .occurrence(**o)
                    .is_some_and(|d| d.grounded != grounded)
            })
            .map(|&occ| Command::SetOccurrenceGrounded { occ, grounded })
            .collect()
    };
    let label = if grounded { "Ground" } else { "Unground" };
    submit(cx, label, commands);
}

/// Copies the placements of `occs` to the document's clipboard.
pub(super) fn copy(cx: &mut AppCx<'_>, occs: &[OccurrenceId]) {
    let clips: Vec<Clip> = {
        let design = cx.env().design;
        occs.iter()
            .filter_map(|o| design.occurrence(*o))
            .map(|o| Clip {
                component: o.component,
                transform: o.transform,
            })
            .collect()
    };
    if clips.is_empty() {
        return;
    }
    if let Some(layer) = cx.doc_layer::<AssemblyLayer>() {
        layer.set_clipboard(clips);
    }
}

/// The clipboard entries that can be pasted into `target`: the component still exists and
/// does not contain `target`.
pub(super) fn pastable(env: &CmdEnv<'_>, target: ComponentId) -> Vec<Clip> {
    let Some(layer) = env.doc_layer::<AssemblyLayer>() else {
        return Vec::new();
    };
    layer
        .clipboard()
        .iter()
        .filter(|c| {
            env.design.component(c.component).is_some() && !reaches(env.design, c.component, target)
        })
        .copied()
        .collect()
}

/// Paste: another occurrence of each copied component in `target`, at the copied placement.
pub(super) fn paste(cx: &mut AppCx<'_>, target: ComponentId) {
    let commands = pastable(&cx.env(), target)
        .into_iter()
        .map(|c| Command::AddOccurrence {
            parent: target,
            component: c.component,
            transform: c.transform,
            name: None,
        })
        .collect();
    submit(cx, "Paste", commands);
}

/// Deletes the occurrences (with the S06 cascade) as one command.
pub(super) fn delete(cx: &mut AppCx<'_>, occs: Vec<OccurrenceId>) {
    if !occs.is_empty() {
        submit(cx, "Delete", vec![Command::DeleteOccurrences { occs }]);
    }
}

/// Why bodies cannot be moved between components now.
pub(super) fn bodies_blocked(env: &CmdEnv<'_>) -> Option<&'static str> {
    (env.design.meta().mode == DesignMode::Direct).then_some("Needs a design with history")
}

/// Create Components from Bodies.
pub(super) fn create_components(cx: &mut AppCx<'_>, bodies: &[BodyId]) {
    const LABEL: &str = "Create Components from Bodies";
    if !require_evaluated(cx, LABEL) {
        return;
    }
    let built = {
        let env = cx.env();
        components_from_bodies(env.design, env.evaluated, bodies)
    };
    match built {
        Ok(commands) => submit(cx, LABEL, commands),
        Err(e) => cx.notify(Notification::error(LABEL, e.to_string())),
    }
}

/// Moves bodies into component `to`.
pub(super) fn move_bodies_to(cx: &mut AppCx<'_>, bodies: &[BodyId], to: ComponentId) {
    const LABEL: &str = "Move to Component";
    if !require_evaluated(cx, LABEL) {
        return;
    }
    let built = {
        let env = cx.env();
        move_bodies(env.design, env.evaluated, bodies, to)
    };
    match built {
        Ok(commands) => submit(cx, LABEL, commands),
        Err(e) => cx.notify(Notification::error(LABEL, e.to_string())),
    }
}

/// A "Move to Component" destination and, if it cannot be used, why.
pub(super) struct Destination {
    pub(super) component: ComponentId,
    pub(super) refused: Option<String>,
}

/// Where the occurrences can be re-parented: every component but their common parent and
/// those inside the moved ones. World positions are kept, which needs unambiguous parents.
pub(super) fn occurrence_destinations(env: &CmdEnv<'_>, occs: &[OccurrenceId]) -> Vec<Destination> {
    let design = env.design;
    let moved: Vec<_> = occs.iter().filter_map(|o| design.occurrence(*o)).collect();
    design
        .components()
        .map(|(c, _)| c)
        .filter(|&c| {
            moved.iter().any(|o| o.parent != c)
                && !moved.iter().any(|o| reaches(design, o.component, c))
        })
        .map(|component| {
            let mut trial = design.clone();
            let refused = occs
                .iter()
                .try_for_each(|&o| trial.move_occurrence(o, component, true))
                .err()
                .map(|e| e.to_string());
            Destination { component, refused }
        })
        .collect()
}

/// Where the bodies can go: every component that is not already the home of all of them.
pub(super) fn body_destinations(env: &CmdEnv<'_>, bodies: &[BodyId]) -> Vec<Destination> {
    let homes: BTreeSet<ComponentId> = bodies
        .iter()
        .filter_map(|b| env.evaluated.body(*b))
        .map(|b| b.component)
        .collect();
    env.design
        .components()
        .map(|(c, _)| c)
        .filter(|c| homes.iter().any(|h| h != c))
        .map(|component| Destination {
            component,
            refused: move_bodies(env.design, env.evaluated, bodies, component)
                .err()
                .map(|e| e.to_string()),
        })
        .collect()
}

/// Re-parents the occurrences into `to`, keeping their world positions.
pub(super) fn move_occurrences_to(cx: &mut AppCx<'_>, occs: &[OccurrenceId], to: ComponentId) {
    let commands = {
        let design = cx.env().design;
        occs.iter()
            .filter(|o| design.occurrence(**o).is_some_and(|d| d.parent != to))
            .map(|&occ| Command::MoveOccurrence {
                occ,
                parent: to,
                keep_world: true,
            })
            .collect()
    };
    submit(cx, "Move to Component", commands);
}

/// Capture Position: one `CapturePositions` for the pending and out-of-date occurrences.
pub(super) fn capture(cx: &mut AppCx<'_>) {
    const LABEL: &str = "Capture Position";
    if !require_evaluated(cx, LABEL) {
        return;
    }
    let command = {
        let env = cx.env();
        amg_doc::joints::capture_command(env.design, env.evaluated, None)
    };
    match command {
        Some(c) => submit(cx, LABEL, vec![c]),
        None => cx.notify(Notification::info(
            LABEL,
            "There is no position to capture.",
        )),
    }
}
