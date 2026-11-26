//! The Move / Copy tool for occurrences (S06 §2, §7, ASM-02).
//!
//! - **Targets:** the occurrences selected when the tool starts (occurrence rows, or bodies
//!   picked in the viewport: the occurrence that places the body). A click on a body in the
//!   viewport makes its occurrence the target (Shift / Ctrl adds or removes it). Bodies of the
//!   root component are not occurrences and cannot be moved here. A grounded target blocks
//!   the move; the dialog says which one and why.
//! - **Free move:** a world-aligned triad ([`crate::manip`]) at the first target's origin:
//!   arms move along an axis, squares in a plane, the centre in the view plane, arcs rotate
//!   about the triad's centre. `Ctrl` snaps to 1 mm / 5°. The numeric fields X / Y / Z
//!   (distance of the triad centre) and X / Y / Z angle (rotation, applied Z·Y·X about the
//!   triad centre) show the move and edit it.
//! - **Point to point:** click the point to move from, then the point to move to; both snap
//!   to the nearest mesh vertex within [`VERTEX_SNAP_PT`] of the click.
//! - **Preview:** every change shows the moved instances through the scene's placement
//!   overlay ([`AppCx::set_placement_overlay`]); nothing is recomputed or uploaded.
//! - **OK** commits one `SetPendingPositions` (History mode) or `SetOccurrenceTransforms`
//!   (Direct mode) for all targets: one undo entry. With *Create copy* it adds one
//!   occurrence of each target's component at the moved place instead (`AddOccurrence`).
//!
//! The move is kept as one world transform `delta` about the triad's start point; each target
//! gets `local' = P⁻¹ · delta · P · local`, with `P` its parent's world transform.

use std::borrow::Cow;
use std::sync::Arc;

use amg_core::math::glam::EulerRot;
use amg_core::{Dimension, Mat3, Point3, Transform, Vec3};
use amg_doc::{BodyGeom, Command, DesignMode, OccurrenceId, Placements};

use super::state::AssemblyLayer;
use crate::app::{CmdEnv, CommitBatch};
use crate::command::{CmdId, Enablement};
use crate::input::ViewportEvent;
use crate::manip::{self, DragValue, GizmoPart, GizmoPrim, ManipEvent, Manipulators, TriadPart};
use crate::pick::{PickHitUi, PickMode, PickTarget, PickTicket};
use crate::select::EntityFilter;
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};
use crate::widgets::{DialogAction, DialogFrame, ExprInput};

/// The Move / Copy command.
pub const MOVE_CMD: CmdId = CmdId("assembly.move");

/// Gizmo id of the triad.
const TRIAD_ID: u32 = 1;

/// Point-to-point clicks snap to a mesh vertex within this distance, points.
pub const VERTEX_SNAP_PT: f64 = 10.0;

/// Accessible names of the dialog's fields and buttons.
pub mod field_labels {
    /// Distance along world X.
    pub const X: &str = "X Distance";
    /// Distance along world Y.
    pub const Y: &str = "Y Distance";
    /// Distance along world Z.
    pub const Z: &str = "Z Distance";
    /// Rotation about world X.
    pub const RX: &str = "X Angle";
    /// Rotation about world Y.
    pub const RY: &str = "Y Angle";
    /// Rotation about world Z.
    pub const RZ: &str = "Z Angle";
    /// The six fields in order.
    pub const ALL: [&str; 6] = [X, Y, Z, RX, RY, RZ];
    /// Free move mode.
    pub const FREE: &str = "Free Move";
    /// Point to point mode.
    pub const POINT_TO_POINT: &str = "Point to Point";
    /// Create copy check box.
    pub const COPY: &str = "Create copy";
    /// Reset button.
    pub const RESET: &str = "Reset";
}

/// How the move is made.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MoveMode {
    /// Triad and fields.
    Free,
    /// From one picked point to another.
    PointToPoint,
}

/// Which point a point-to-point click picks.
#[derive(Copy, Clone, Debug, PartialEq)]
enum P2p {
    From,
    To(Point3),
}

/// A pending viewport click.
#[derive(Copy, Clone, Debug, PartialEq)]
enum Click {
    /// Choose the moved occurrence (`toggle`: Shift / Ctrl).
    Target { toggle: bool },
    /// A point-to-point point, clicked at `pos`.
    Point { pos: egui::Pos2 },
}

/// One moved occurrence.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Target {
    occ: OccurrenceId,
    /// World transform of its parent (first path from the root).
    parent_world: Transform,
    /// Its local placement when the tool started (or when it was chosen).
    local: Transform,
}

/// The Move / Copy tool (see the module docs).
#[derive(Debug)]
pub struct MoveTool {
    targets: Vec<Target>,
    /// Triad centre before the move (world, mm).
    pivot: Point3,
    /// The move so far (world).
    delta: Transform,
    /// The move as X, Y, Z (mm) and X, Y, Z angles (rad), exact.
    values: [f64; 6],
    /// The six field texts.
    texts: [String; 6],
    mode: MoveMode,
    p2p: P2p,
    copy: bool,
    manips: Manipulators,
    /// The dragged triad part and `delta` when the drag started.
    drag: Option<(TriadPart, Transform)>,
    click: Option<(PickTicket, Click)>,
    /// `delta` changed since the overlay was last sent.
    dirty: bool,
    /// An overlay was sent and not cleared yet.
    shown: bool,
    /// Placements of the last overlay (tests).
    last_overlay: Option<Arc<Placements>>,
    units: amg_doc::UnitPrefs,
}

impl MoveTool {
    fn new(init: &ToolInit<'_>) -> MoveTool {
        let occs = occurrences_of(&init.env, init.preselection.iter().map(|i| i.target));
        let mut t = MoveTool {
            targets: Vec::new(),
            pivot: Point3::ZERO,
            delta: Transform::IDENTITY,
            values: [0.0; 6],
            texts: Default::default(),
            mode: MoveMode::Free,
            p2p: P2p::From,
            copy: false,
            manips: Manipulators::new(),
            drag: None,
            click: None,
            dirty: false,
            shown: false,
            last_overlay: None,
            units: init.env.design.meta().default_units.clone(),
        };
        t.set_targets(&init.env, &occs);
        t
    }

    /// The moved occurrences.
    pub fn targets(&self) -> Vec<OccurrenceId> {
        self.targets.iter().map(|t| t.occ).collect()
    }

    /// The move so far, a world transform.
    pub fn delta(&self) -> Transform {
        self.delta
    }

    /// The triad centre before the move (world, mm).
    pub fn pivot(&self) -> Point3 {
        self.pivot
    }

    /// The triad as drawn now (centre moved by the move so far, world-aligned).
    pub fn triad(&self) -> GizmoPrim {
        GizmoPrim::Triad {
            origin: self.delta.transform_point3(self.pivot),
            frame: Mat3::IDENTITY,
            id: TRIAD_ID,
            hot_part: None,
        }
    }

    /// The mode.
    pub fn mode(&self) -> MoveMode {
        self.mode
    }

    /// Switches the mode (point to point starts with the "from" point).
    pub fn set_mode(&mut self, mode: MoveMode) {
        self.mode = mode;
        self.p2p = P2p::From;
    }

    /// Create a copy instead of moving.
    pub fn set_copy(&mut self, copy: bool) {
        self.copy = copy;
    }

    /// The placements of the last preview sent to the scene.
    pub fn preview_placements(&self) -> Option<&Arc<Placements>> {
        self.last_overlay.as_ref()
    }

    /// The point-to-point "from" point once picked.
    pub fn from_point(&self) -> Option<Point3> {
        match self.p2p {
            P2p::To(p) => Some(p),
            P2p::From => None,
        }
    }

    /// Applies a triad drag of `part` by `value` as if dragged with the pointer (scripts and
    /// tests): `Move` for arms, squares and the centre, `Angle` for the arcs.
    pub fn drag_part(&mut self, part: TriadPart, value: DragValue, snap: bool) {
        let base = self.delta;
        self.apply_drag(part, base, value, snap);
    }

    /// Sets the move from the field values (X, Y, Z in mm; angles in rad).
    pub fn set_values(&mut self, values: [f64; 6]) {
        self.values = values;
        self.delta = delta_of(self.pivot, values);
        self.texts_from_values();
        self.dirty = true;
    }

    /// The field values (X, Y, Z in mm; angles in rad).
    pub fn values(&self) -> [f64; 6] {
        self.values
    }

    /// Why the targets cannot be moved, if they cannot.
    pub fn blocked(&self, env: &CmdEnv<'_>) -> Option<String> {
        blocked_reason(env, &self.targets(), self.copy)
    }

    /// The commit for the move so far (`None`: nothing moved or blocked).
    pub fn commit_batch(&self, env: &CmdEnv<'_>) -> Option<CommitBatch> {
        if self.targets.is_empty()
            || self.blocked(env).is_some()
            || self.delta.abs_diff_eq(Transform::IDENTITY, 0.0)
        {
            return None;
        }
        let moves = self.moves();
        if self.copy {
            let commands = moves
                .iter()
                .filter_map(|(o, local)| {
                    let d = env.design.occurrence(*o)?;
                    Some(Command::AddOccurrence {
                        parent: d.parent,
                        component: d.component,
                        transform: *local,
                        name: None,
                    })
                })
                .collect();
            return Some(CommitBatch {
                label: "Copy".into(),
                commands,
            });
        }
        Some(CommitBatch {
            label: "Move".into(),
            commands: vec![move_command(env.design.meta().mode, moves)],
        })
    }

    /// The new local placement of every target.
    fn moves(&self) -> Vec<(OccurrenceId, Transform)> {
        self.targets
            .iter()
            .map(|t| {
                (
                    t.occ,
                    t.parent_world.inverse() * self.delta * t.parent_world * t.local,
                )
            })
            .collect()
    }

    fn set_targets(&mut self, env: &CmdEnv<'_>, occs: &[OccurrenceId]) {
        let placements = env.evaluated.placements();
        self.targets = occs
            .iter()
            .filter_map(|&occ| {
                let o = env.design.occurrence(occ)?;
                let parent_world = placements
                    .paths_of(o.parent)
                    .first()
                    .and_then(|p| placements.world(p))?;
                let local = placements
                    .local(occ)
                    .or_else(|| env.design.seed_transform(occ))?;
                Some(Target {
                    occ,
                    parent_world,
                    local,
                })
            })
            .collect();
        self.pivot = self
            .targets
            .first()
            .map_or(Point3::ZERO, |t| (t.parent_world * t.local).translation);
        self.delta = Transform::IDENTITY;
        self.values = [0.0; 6];
        self.texts_from_values();
        self.p2p = P2p::From;
        self.dirty = true;
    }

    fn texts_from_values(&mut self) {
        for (i, v) in self.values.iter().enumerate() {
            let dim = if i < 3 {
                Dimension::LENGTH
            } else {
                Dimension::ANGLE
            };
            self.texts[i] = manip::field_text(*v, dim, &self.units);
        }
    }

    fn values_from_delta(&mut self) {
        let t = self.delta.transform_point3(self.pivot) - self.pivot;
        let q = amg_core::Quat::from_mat3(&self.delta.matrix3);
        let (rz, ry, rx) = q.to_euler(EulerRot::ZYX);
        self.values = [t.x, t.y, t.z, rx, ry, rz];
        self.texts_from_values();
    }

    /// `delta = step · base` for a drag of `part` by `value` that started at `base`.
    fn apply_drag(&mut self, part: TriadPart, base: Transform, value: DragValue, snap: bool) {
        let step = match value {
            DragValue::Move(v) => {
                let v = if snap {
                    let s = manip::snap_step(Dimension::LENGTH, &self.units);
                    Vec3::new(
                        manip::snap(v.x, s),
                        manip::snap(v.y, s),
                        manip::snap(v.z, s),
                    )
                } else {
                    v
                };
                Transform::from_translation(v)
            }
            DragValue::Along(d) => {
                let axis = match part {
                    TriadPart::AxisY => Vec3::Y,
                    TriadPart::AxisZ => Vec3::Z,
                    _ => Vec3::X,
                };
                let d = if snap {
                    manip::snap(d, manip::snap_step(Dimension::LENGTH, &self.units))
                } else {
                    d
                };
                Transform::from_translation(axis * d)
            }
            DragValue::Angle(a) => {
                let axis = match part {
                    TriadPart::RotX => Vec3::X,
                    TriadPart::RotY => Vec3::Y,
                    _ => Vec3::Z,
                };
                let a = if snap {
                    manip::snap(a, manip::SNAP_ANGLE)
                } else {
                    a
                };
                let c = base.transform_point3(self.pivot);
                Transform::from_translation(c)
                    * Transform::from_axis_angle(axis, a)
                    * Transform::from_translation(-c)
            }
        };
        self.delta = step * base;
        self.values_from_delta();
        self.dirty = true;
    }

    fn on_manip(&mut self, ev: ManipEvent) {
        match ev {
            ManipEvent::Begin {
                part: GizmoPart::Triad(p),
                ..
            } => self.drag = Some((p, self.delta)),
            ManipEvent::Drag { value, snap, .. } => {
                if let Some((part, base)) = self.drag {
                    self.apply_drag(part, base, value, snap);
                }
            }
            ManipEvent::End { value, snap, .. } => {
                if let (Some((part, base)), Some(v)) = (self.drag, value) {
                    self.apply_drag(part, base, v, snap);
                }
                self.drag = None;
            }
            ManipEvent::Cancel { .. } => {
                if let Some((_, base)) = self.drag.take() {
                    self.delta = base;
                    self.values_from_delta();
                    self.dirty = true;
                }
            }
            ManipEvent::Begin { .. } => {}
        }
    }

    /// Sends the preview placements when the move changed.
    fn push_overlay(&mut self, cx: &mut ToolCx<'_>) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let _span = tracing::debug_span!("amg_ui.assembly.move_preview").entered();
        let overlay = if self.targets.is_empty() || self.blocked(&cx.env()).is_some() {
            None
        } else {
            let env = cx.env();
            preview_placements(env.design, self.moves())
        };
        if overlay.is_none() && !self.shown {
            return;
        }
        self.shown = overlay.is_some();
        self.last_overlay.clone_from(&overlay);
        cx.app().set_placement_overlay(overlay);
    }

    fn on_hits(&mut self, click: Click, hits: &[PickHitUi], cx: &mut ToolCx<'_>) {
        let hit = hits
            .iter()
            .find(|h| matches!(h.target, PickTarget::Body { .. }));
        match click {
            Click::Target { toggle } => {
                let Some(hit) = hit else {
                    if !toggle {
                        let env = cx.env();
                        self.set_targets(&env, &[]);
                    }
                    return;
                };
                let env = cx.env();
                let Some(o) = occurrence_of_target(&env, hit.target) else {
                    return;
                };
                let mut occs = self.targets();
                if toggle {
                    if let Some(i) = occs.iter().position(|x| *x == o) {
                        occs.remove(i);
                    } else {
                        occs.push(o);
                    }
                } else {
                    occs = vec![o];
                }
                self.set_targets(&env, &occs);
            }
            Click::Point { pos } => {
                let Some(hit) = hit else { return };
                let p = snap_point(cx, hit, pos);
                self.pick_point(p);
            }
        }
    }

    /// A point-to-point pick: the first is the "from" point, the second moves it onto
    /// itself (scripts and tests may call it with exact points).
    pub fn pick_point(&mut self, p: Point3) {
        match self.p2p {
            P2p::From => self.p2p = P2p::To(p),
            P2p::To(from) => {
                self.delta = Transform::from_translation(p - from) * self.delta;
                self.values_from_delta();
                self.dirty = true;
                self.p2p = P2p::From;
            }
        }
    }

    fn fields_ui(&mut self, ui: &mut egui::Ui, params: &amg_doc::param::ParamTable) {
        let mut edited = None;
        egui::Grid::new("amg_move_fields")
            .num_columns(2)
            .show(ui, |ui| {
                for (i, label) in field_labels::ALL.iter().enumerate() {
                    let dim = if i < 3 {
                        Dimension::LENGTH
                    } else {
                        Dimension::ANGLE
                    };
                    ui.label(*label);
                    let r = ExprInput::new(&mut self.texts[i], dim, params, &self.units)
                        .accessible_name(*label)
                        .id_salt(("amg_move_field", i))
                        .desired_width(110.0)
                        .show(ui);
                    if (r.changed || r.committed)
                        && let Ok(q) = r.value
                    {
                        edited = Some((i, q.value));
                    }
                    ui.end_row();
                }
            });
        if let Some((i, v)) = edited
            && (self.values[i] - v).abs() > 0.0
        {
            self.values[i] = v;
            self.delta = delta_of(self.pivot, self.values);
            self.dirty = true;
        }
    }
}

/// `delta` from X, Y, Z (the triad centre's move) and the Z·Y·X rotation about the centre.
fn delta_of(pivot: Point3, v: [f64; 6]) -> Transform {
    let r = Transform::from_mat3(Mat3::from_euler(EulerRot::ZYX, v[5], v[4], v[3]));
    Transform::from_translation(pivot + Vec3::new(v[0], v[1], v[2]))
        * r
        * Transform::from_translation(-pivot)
}

/// The move command of the design's mode.
pub(super) fn move_command(mode: DesignMode, moves: Vec<(OccurrenceId, Transform)>) -> Command {
    match mode {
        DesignMode::Direct => Command::SetOccurrenceTransforms { moves },
        DesignMode::History => Command::SetPendingPositions {
            moves,
            joint_states: Vec::new(),
        },
    }
}

/// The placements the design would have after `moves`, for the scene overlay (`None` if the
/// design refuses them). Before the joint solver (ASM-04) placements equal the seeds.
fn preview_placements(
    design: &amg_doc::Design,
    moves: Vec<(OccurrenceId, Transform)>,
) -> Option<Arc<Placements>> {
    let mut trial = design.clone();
    let command = move_command(trial.meta().mode, moves);
    amg_doc::apply_command(&mut trial, command).ok()?;
    Some(Arc::new(Placements::from_design(&trial)))
}

/// Why `occs` cannot be moved (`copy`: copies are made, so grounding does not matter).
fn blocked_reason(env: &CmdEnv<'_>, occs: &[OccurrenceId], copy: bool) -> Option<String> {
    if copy {
        return None;
    }
    occs.iter().find_map(|o| {
        let d = env.design.occurrence(*o)?;
        d.grounded.then(|| {
            let name = d
                .name
                .clone()
                .unwrap_or_else(|| super::actions::component_name(env.design, d.component));
            format!("{name} is grounded and cannot be moved. Unground it first.")
        })
    })
}

/// The occurrence a picked target stands for: an occurrence, or the occurrence that places a
/// picked body (the last of its path). `None` for root bodies and other targets.
fn occurrence_of_target(env: &CmdEnv<'_>, t: PickTarget) -> Option<OccurrenceId> {
    match t {
        PickTarget::Occurrence(o) => env.design.occurrence(o).map(|_| o),
        PickTarget::Body { instance, .. } => {
            let path = env.instance_paths.get(instance as usize)?;
            path.last().copied()
        }
        _ => None,
    }
}

/// The distinct occurrences of `targets`, in order.
fn occurrences_of(
    env: &CmdEnv<'_>,
    targets: impl Iterator<Item = PickTarget>,
) -> Vec<OccurrenceId> {
    let mut out = Vec::new();
    for t in targets {
        if let Some(o) = occurrence_of_target(env, t)
            && !out.contains(&o)
        {
            out.push(o);
        }
    }
    out
}

/// The picked world point, snapped to the nearest mesh vertex of the picked instance within
/// [`VERTEX_SNAP_PT`] of the click (exact f64 vertex positions).
fn snap_point(cx: &ToolCx<'_>, hit: &PickHitUi, pos: egui::Pos2) -> Point3 {
    let PickTarget::Body { body, instance, .. } = hit.target else {
        return hit.world;
    };
    let env = cx.env();
    let (Some(b), Some(world), Some(cam)) = (
        env.evaluated.body(body),
        cx.scene().instance_world(body, instance),
        cx.camera(),
    ) else {
        return hit.world;
    };
    let BodyGeom::Mesh(mesh) = &b.geom else {
        return hit.world;
    };
    let mut best: Option<(f32, Point3)> = None;
    for p in &mesh.positions {
        let w = world.transform_point3(*p);
        let Some(s) = cam.project(w) else { continue };
        let d = s.distance(pos);
        if best.is_none_or(|(bd, _)| d < bd) {
            best = Some((d, w));
        }
    }
    match best {
        #[allow(clippy::cast_possible_truncation, reason = "a small radius in points")]
        Some((d, w)) if d <= VERTEX_SNAP_PT as f32 => w,
        _ => hit.world,
    }
}

impl Tool for MoveTool {
    fn id(&self) -> CmdId {
        MOVE_CMD
    }

    fn label(&self) -> Cow<'static, str> {
        "Move/Copy".into()
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        if self.mode == MoveMode::Free
            && let Some(cam) = cx.camera().cloned()
        {
            let r = self.manips.input(ev, &cam);
            if let Some(e) = r.event {
                self.on_manip(e);
            }
            if r.changed {
                cx.app().request_repaint();
            }
            if r.result == InputResult::Consumed {
                return InputResult::Consumed;
            }
        }
        match ev {
            ViewportEvent::Click {
                button: egui::PointerButton::Primary,
                pos,
                mods,
                ..
            } => {
                let click = match self.mode {
                    MoveMode::Free => Click::Target {
                        toggle: mods.shift || mods.ctrl,
                    },
                    MoveMode::PointToPoint => Click::Point { pos: *pos },
                };
                let t = cx.pick_point(*pos, PickMode::Nearest);
                self.click = Some((t, click));
                InputResult::Consumed
            }
            _ => InputResult::Ignored,
        }
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        let _span = tracing::debug_span!("amg_ui.assembly.move").entered();
        if let Some((t, click)) = self.click
            && let Some(hits) = cx.poll_pick(t)
        {
            self.click = None;
            self.on_hits(click, &hits, cx);
        }
        let gizmos = if self.mode == MoveMode::Free
            && !self.targets.is_empty()
            && self.blocked(&cx.env()).is_none()
        {
            vec![self.triad()]
        } else {
            Vec::new()
        };
        self.manips.set(gizmos);
        if self.manips.dragging().is_some() {
            cx.set_cursor(egui::CursorIcon::Grabbing);
        } else if self.manips.hot().is_some() {
            cx.set_cursor(egui::CursorIcon::Grab);
        }
        self.push_overlay(cx);
    }

    fn overlay(&mut self, out: &mut amg_render::OverlayList, cx: &ToolCx<'_>) {
        if let Some(cam) = cx.camera() {
            self.manips.draw(out, cam);
        }
        if let P2p::To(p) = self.p2p
            && self.mode == MoveMode::PointToPoint
        {
            out.points(
                &[p],
                amg_render::PointStyle {
                    size_px: 8.0,
                    color: [1.0, 0.72, 0.05, 1.0],
                    depth: amg_render::DepthMode::AlwaysOnTop,
                },
                None,
            );
        }
    }

    fn dialog_ui(&mut self, ui: &mut egui::Ui, cx: &mut ToolCx<'_>) -> bool {
        let params = cx.env().design.params().clone();
        let blocked = self.blocked(&cx.env());
        let ok = if let Some(b) = &blocked {
            Enablement::disabled(b.clone())
        } else if self.targets.is_empty() {
            Enablement::disabled("Select the components to move")
        } else {
            Enablement::Enabled
        };
        let count = self.targets.len();
        let action = DialogFrame::new("Move/Copy")
            .ok_enabled(ok)
            .show_inside(ui, |ui| {
                ui.label(match count {
                    0 => "No component selected: click a component".to_owned(),
                    1 => "1 component".to_owned(),
                    n => format!("{n} components"),
                });
                if let Some(b) = &blocked {
                    ui.colored_label(ui.visuals().error_fg_color, b);
                }
                ui.horizontal(|ui| {
                    for (m, label) in [
                        (MoveMode::Free, field_labels::FREE),
                        (MoveMode::PointToPoint, field_labels::POINT_TO_POINT),
                    ] {
                        if ui.selectable_label(self.mode == m, label).clicked() {
                            self.set_mode(m);
                        }
                    }
                });
                match self.mode {
                    MoveMode::Free => self.fields_ui(ui, &params),
                    MoveMode::PointToPoint => {
                        ui.weak(match self.p2p {
                            P2p::From => "Click the point to move from",
                            P2p::To(_) => "Click the point to move to",
                        });
                    }
                }
                ui.checkbox(&mut self.copy, field_labels::COPY);
                if ui.button(field_labels::RESET).clicked() {
                    self.set_values([0.0; 6]);
                    self.p2p = P2p::From;
                }
            });
        self.push_overlay(cx);
        match action {
            DialogAction::Ok => cx.request_finish(FinishReason::Commit),
            DialogAction::Cancel => cx.request_finish(FinishReason::Cancel),
            DialogAction::None => {}
        }
        true
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::ALL
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        Some(match (self.mode, self.p2p) {
            (MoveMode::Free, _) => {
                "Move/Copy: drag the triad or type values (Ctrl snaps); Enter applies, Esc cancels"
                    .into()
            }
            (MoveMode::PointToPoint, P2p::From) => "Move/Copy: click the point to move from".into(),
            (MoveMode::PointToPoint, P2p::To(_)) => "Move/Copy: click the point to move to".into(),
        })
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn finish(&mut self, reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        self.manips.reset();
        let batch = if reason == FinishReason::Commit {
            self.commit_batch(&cx.env())
        } else {
            None
        };
        if self.shown {
            self.shown = false;
            if batch.is_some() {
                // Keep showing the moved place until the evaluation has caught up.
                let revision = cx.app().doc_tab().document.history().revision();
                match cx.app().doc_layer::<AssemblyLayer>() {
                    Some(l) => l.hold_overlay(revision),
                    None => cx.app().set_placement_overlay(None),
                }
            } else {
                cx.app().set_placement_overlay(None);
            }
        }
        batch
    }
}

/// The tool factory of [`MOVE_CMD`].
pub(super) fn start(init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(MoveTool::new(init))
}

/// Enabled with a design that has occurrences.
pub(super) fn can_move(env: &CmdEnv<'_>) -> Enablement {
    if env.design.occurrences().next().is_some() {
        Enablement::Enabled
    } else {
        Enablement::disabled("The design has no components to move")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn field_values_round_trip_through_the_delta() {
        let pivot = Point3::new(3.0, -2.0, 7.0);
        let v = [12.5, -4.0, 0.25, 0.3, -0.7, 1.2];
        let d = delta_of(pivot, v);
        // The triad centre moves by X, Y, Z.
        let moved = d.transform_point3(pivot) - pivot;
        assert!(moved.distance(Vec3::new(v[0], v[1], v[2])) < 1e-12);
        // The angles come back.
        let q = amg_core::Quat::from_mat3(&d.matrix3);
        let (rz, ry, rx) = q.to_euler(EulerRot::ZYX);
        for (a, b) in [(rx, v[3]), (ry, v[4]), (rz, v[5])] {
            assert!((a - b).abs() < 1e-12, "{a} vs {b}");
        }
    }

    #[test]
    fn the_move_command_follows_the_design_mode() {
        let m = vec![(OccurrenceId(1), Transform::IDENTITY)];
        assert!(matches!(
            move_command(DesignMode::History, m.clone()),
            Command::SetPendingPositions { .. }
        ));
        assert!(matches!(
            move_command(DesignMode::Direct, m),
            Command::SetOccurrenceTransforms { .. }
        ));
    }
}
