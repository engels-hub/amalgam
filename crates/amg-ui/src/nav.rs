//! Navigation presets (spec §6 "Navigation presets", Interfaces §A `amg_ui::nav`).
//!
//! A [`NavPreset`] is a data table ([`NavBindings`]) that maps (button, modifiers) to
//! [`NavAction::Orbit`] / `Pan` / `Zoom`, plus the wheel direction. The [`Navigator`] feeds the
//! matching [`ViewportEvent`]s into S10's `nav::NavController` (RND-11), which does all camera
//! maths; nothing here computes a camera itself. View-cube and nav-bar [`CameraAction`]s go
//! through `egui_glue::apply_camera_action`.
//!
//! UI-03 ships the Fusion preset (orbit `Shift+MMB`, pan `MMB`, zoom with the wheel; orbit
//! about the cursor pick point and zoom to the cursor). UI-11 adds the preferences page; the
//! SolidWorks and Blender tables are here already.

use std::time::Duration;

use amg_core::{Aabb3, Plane, Point3};
use amg_render::egui_glue::{CameraAction, apply_camera_action};
use amg_render::nav::{NavController, NavMode};
use amg_render::{Camera, Projection};
use serde::{Deserialize, Serialize};

use crate::input::{CameraView, POINTS_PER_LINE, ViewportEvent};
use crate::tool::InputResult;

/// A navigation preset.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum NavPreset {
    /// Fusion: orbit `Shift+MMB`, pan `MMB`, wheel zoom.
    #[default]
    Fusion,
    /// SolidWorks: orbit `MMB`, pan `Ctrl+MMB`, zoom `Shift+MMB` and wheel.
    SolidWorks,
    /// Blender: orbit `MMB`, pan `Shift+MMB`, zoom `Ctrl+MMB` and wheel.
    Blender,
    /// The user's own table (UI-11); until then the Fusion table.
    Custom,
}

/// What a drag does to the camera.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavAction {
    /// Rotate about the pivot.
    Orbit,
    /// Move parallel to the view plane.
    Pan,
    /// Dolly / scale.
    Zoom,
}

impl NavAction {
    fn mode(self) -> NavMode {
        match self {
            NavAction::Orbit => NavMode::Orbit,
            NavAction::Pan => NavMode::Pan,
            NavAction::Zoom => NavMode::Zoom,
        }
    }
}

/// Serialisable stand-in for `egui::PointerButton` (egui's type has no serde here).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavButton {
    /// Left.
    Primary,
    /// Right.
    Secondary,
    /// Wheel button.
    Middle,
}

impl NavButton {
    /// The egui button.
    pub fn egui(self) -> egui::PointerButton {
        match self {
            NavButton::Primary => egui::PointerButton::Primary,
            NavButton::Secondary => egui::PointerButton::Secondary,
            NavButton::Middle => egui::PointerButton::Middle,
        }
    }
}

/// Modifier keys of a binding (exact match on Shift, Ctrl/Cmd and Alt).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct NavMods {
    /// Shift held.
    pub shift: bool,
    /// Ctrl (Cmd on macOS) held.
    pub ctrl: bool,
    /// Alt held.
    pub alt: bool,
}

impl NavMods {
    /// No modifier.
    pub const NONE: NavMods = NavMods {
        shift: false,
        ctrl: false,
        alt: false,
    };
    /// Shift only.
    pub const SHIFT: NavMods = NavMods {
        shift: true,
        ctrl: false,
        alt: false,
    };
    /// Ctrl only.
    pub const CTRL: NavMods = NavMods {
        shift: false,
        ctrl: true,
        alt: false,
    };

    /// The modifiers of an egui event.
    pub fn of(m: egui::Modifiers) -> NavMods {
        NavMods {
            shift: m.shift,
            ctrl: m.ctrl || m.command || m.mac_cmd,
            alt: m.alt,
        }
    }
}

/// A preset's table: drag bindings and wheel behaviour.
///
/// Spec shape: `drags: Vec<(egui::PointerButton, egui::Modifiers, NavAction)>`; egui's types
/// are not serde-enabled in this workspace, so the table uses [`NavButton`] / [`NavMods`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NavBindings {
    /// (button, modifiers) → action; the first exact match wins.
    pub drags: Vec<(NavButton, NavMods, NavAction)>,
    /// Turning the wheel up (away from the user) zooms in.
    pub wheel_zoom_in_is_up: bool,
    /// Orbit about the world point under the cursor (else the scene centre).
    pub orbit_about_cursor: bool,
    /// Zoom about the world point under the cursor (else the view centre).
    pub zoom_to_cursor: bool,
}

impl NavBindings {
    /// The action bound to a drag with `button` and `mods`.
    pub fn action_for(
        &self,
        button: egui::PointerButton,
        mods: egui::Modifiers,
    ) -> Option<NavAction> {
        let mods = NavMods::of(mods);
        self.drags
            .iter()
            .find(|(b, m, _)| b.egui() == button && *m == mods)
            .map(|(_, _, a)| *a)
    }
}

impl NavPreset {
    /// The preset's table.
    pub fn bindings(self) -> NavBindings {
        let drags = match self {
            NavPreset::Fusion | NavPreset::Custom => vec![
                (NavButton::Middle, NavMods::SHIFT, NavAction::Orbit),
                (NavButton::Middle, NavMods::NONE, NavAction::Pan),
            ],
            NavPreset::SolidWorks => vec![
                (NavButton::Middle, NavMods::NONE, NavAction::Orbit),
                (NavButton::Middle, NavMods::CTRL, NavAction::Pan),
                (NavButton::Middle, NavMods::SHIFT, NavAction::Zoom),
            ],
            NavPreset::Blender => vec![
                (NavButton::Middle, NavMods::NONE, NavAction::Orbit),
                (NavButton::Middle, NavMods::SHIFT, NavAction::Pan),
                (NavButton::Middle, NavMods::CTRL, NavAction::Zoom),
            ],
        };
        NavBindings {
            drags,
            wheel_zoom_in_is_up: true,
            orbit_about_cursor: true,
            zoom_to_cursor: true,
        }
    }
}

/// Moves the camera from viewport events and camera actions (one per document).
#[derive(Clone, Debug)]
pub struct Navigator {
    bindings: NavBindings,
    controller: NavController,
    /// The button of the drag navigation owns.
    dragging: Option<egui::PointerButton>,
    /// Nav-bar mode: the next primary drag orbits / pans / zooms.
    sticky: Option<NavMode>,
}

impl Default for Navigator {
    fn default() -> Self {
        Navigator::new(NavPreset::Fusion.bindings())
    }
}

impl Navigator {
    /// A navigator with `bindings`.
    pub fn new(bindings: NavBindings) -> Navigator {
        Navigator {
            bindings,
            controller: NavController::new(),
            dragging: None,
            sticky: None,
        }
    }

    /// The table in use.
    pub fn bindings(&self) -> &NavBindings {
        &self.bindings
    }

    /// Replaces the table (preferences, UI-11).
    pub fn set_bindings(&mut self, b: NavBindings) {
        self.bindings = b;
    }

    /// S10's controller (pivot, orbit mode, transitions).
    pub fn controller(&self) -> &NavController {
        &self.controller
    }

    /// True while a navigation drag runs.
    pub fn is_dragging(&self) -> bool {
        self.dragging.is_some()
    }

    /// True while a view transition animates.
    pub fn is_animating(&self) -> bool {
        self.controller.is_animating()
    }

    /// Handles a viewport event. `anchor` = world point under the cursor (the newest hover
    /// pick near the event), `bounds` = the visible scene. Consumes drags it owns and the
    /// wheel.
    pub fn input(
        &mut self,
        ev: &ViewportEvent,
        cam: &mut Camera,
        view: &CameraView,
        anchor: Option<Point3>,
        bounds: Aabb3,
    ) -> InputResult {
        match ev {
            ViewportEvent::DragStart { button, from, mods } => {
                let action = self
                    .bindings
                    .action_for(*button, *mods)
                    .map(NavAction::mode)
                    .or(match (button, self.sticky) {
                        (egui::PointerButton::Primary, Some(m)) => Some(m),
                        _ => None,
                    });
                let Some(mode) = action else {
                    return InputResult::Ignored;
                };
                let anchor = match mode {
                    NavMode::Orbit if self.bindings.orbit_about_cursor => anchor,
                    NavMode::Pan => anchor,
                    _ => None,
                };
                self.controller
                    .begin(mode, cam, view.to_px(*from), anchor, bounds);
                self.dragging = Some(*button);
                InputResult::Consumed
            }
            ViewportEvent::Drag { button, pos, .. } if self.dragging == Some(*button) => {
                self.controller.drag(cam, view.to_px(*pos), view.size_px);
                InputResult::Consumed
            }
            ViewportEvent::DragEnd { button, pos, .. } if self.dragging == Some(*button) => {
                self.controller.drag(cam, view.to_px(*pos), view.size_px);
                self.controller.end();
                self.dragging = None;
                if *button == egui::PointerButton::Primary {
                    self.sticky = None;
                }
                InputResult::Consumed
            }
            ViewportEvent::Release { button, .. } | ViewportEvent::Press { button, .. }
                if self.dragging == Some(*button) =>
            {
                InputResult::Consumed
            }
            ViewportEvent::Scroll { delta, pos, .. } => {
                if delta.y == 0.0 {
                    return InputResult::Ignored;
                }
                let sign = if self.bindings.wheel_zoom_in_is_up {
                    1.0
                } else {
                    -1.0
                };
                let steps = sign * f64::from(delta.y / POINTS_PER_LINE);
                let (at, anchor) = if self.bindings.zoom_to_cursor {
                    (view.to_px(*pos), anchor)
                } else {
                    (view.to_px(view.rect.center()), None)
                };
                self.controller.zoom(cam, steps, at, anchor, view.size_px);
                InputResult::Consumed
            }
            _ => InputResult::Ignored,
        }
    }

    /// Applies a view-cube / nav-bar action. `StartMode` arms the next primary drag; the
    /// visual-style action is returned for the shell.
    pub fn apply(
        &mut self,
        action: CameraAction,
        cam: &mut Camera,
        bounds: Aabb3,
        size_px: [u32; 2],
    ) -> Option<CameraAction> {
        match apply_camera_action(&mut self.controller, cam, action, bounds, size_px) {
            Some(CameraAction::StartMode(m)) => {
                self.sticky = Some(m);
                None
            }
            other => other,
        }
    }

    /// Advances a running transition; true while it animates (ask for another frame).
    pub fn tick(&mut self, cam: &mut Camera, dt: Duration) -> bool {
        self.controller.tick(cam, dt)
    }

    /// Cancels a drag (the tool or the window took the pointer).
    pub fn cancel_drag(&mut self) {
        if self.dragging.take().is_some() {
            self.controller.end();
        }
    }
}

/// Camera access for modules (spec `CameraControl`), implemented over S10's camera and
/// controller.
pub trait CameraControl {
    /// Orbits by a screen delta (points) about `pivot` (`None`: the target).
    fn orbit(&mut self, delta: egui::Vec2, pivot: Option<Point3>);
    /// Pans by a screen delta (points).
    fn pan(&mut self, delta: egui::Vec2);
    /// Zooms by `factor` (> 1 = closer) about the screen position `at`.
    fn zoom(&mut self, factor: f64, at: egui::Pos2);
    /// Fits `bounds` (`None`: the whole scene).
    fn fit(&mut self, bounds: Option<Aabb3>);
    /// Looks at a plane (sketch "look at").
    fn look_at_plane(&mut self, plane: &Plane, animate: bool);
    /// Switches between orthographic and perspective.
    fn set_ortho(&mut self, ortho: bool);
}

/// [`CameraControl`] over one document's camera and navigator.
pub(crate) struct DocCamera<'a> {
    pub(crate) camera: &'a mut Camera,
    pub(crate) nav: &'a mut Navigator,
    pub(crate) view: CameraView,
    pub(crate) bounds: Aabb3,
}

impl DocCamera<'_> {
    fn drag(&mut self, mode: NavMode, delta: egui::Vec2, anchor: Option<Point3>) {
        let c = self.view.rect.center();
        let from = self.view.to_px(c);
        let to = self.view.to_px(c + delta);
        let n = &mut self.nav.controller;
        n.begin(mode, self.camera, from, anchor, self.bounds);
        n.drag(self.camera, to, self.view.size_px);
        n.end();
    }
}

impl CameraControl for DocCamera<'_> {
    fn orbit(&mut self, delta: egui::Vec2, pivot: Option<Point3>) {
        let pivot = pivot.or(Some(self.camera.target));
        self.drag(NavMode::Orbit, delta, pivot);
    }

    fn pan(&mut self, delta: egui::Vec2) {
        self.drag(NavMode::Pan, delta, None);
    }

    fn zoom(&mut self, factor: f64, at: egui::Pos2) {
        if factor <= 0.0 {
            return;
        }
        // factor = 0.9^-steps  →  steps = -ln(factor)/ln(0.9)
        let steps = factor.ln() / -amg_render::nav::ZOOM_STEP.ln();
        let at = self.view.to_px(at);
        self.nav
            .controller
            .zoom(self.camera, steps, at, None, self.view.size_px);
    }

    fn fit(&mut self, bounds: Option<Aabb3>) {
        let b = bounds.unwrap_or(self.bounds);
        self.nav
            .controller
            .fit(self.camera, b, self.view.size_px, true);
    }

    fn look_at_plane(&mut self, plane: &Plane, animate: bool) {
        self.nav.controller.look_at_plane(
            self.camera,
            plane,
            self.bounds,
            self.view.size_px,
            animate,
        );
    }

    fn set_ortho(&mut self, ortho: bool) {
        let p = if ortho {
            Projection::Orthographic { height: 1.0 }
        } else {
            Projection::Perspective {
                fov_y: amg_render::nav::DEFAULT_FOV_Y,
            }
        };
        self.nav.controller.set_projection(self.camera, p);
    }
}

/// The camera a new document starts with: Fusion's home direction (front-right-top, Z up)
/// looking at the origin from 300 mm.
pub fn home_camera() -> Camera {
    let dir = amg_render::nav::StandardView::Home
        .eye_direction()
        .normalize();
    Camera {
        eye: dir * 300.0,
        target: Point3::ZERO,
        up: amg_core::Dir3::Z,
        projection: Projection::Perspective {
            fov_y: amg_render::nav::DEFAULT_FOV_Y,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fusion_table_orbits_with_shift_middle_and_pans_with_middle() {
        let b = NavPreset::Fusion.bindings();
        let mid = egui::PointerButton::Middle;
        assert_eq!(
            b.action_for(mid, egui::Modifiers::SHIFT),
            Some(NavAction::Orbit)
        );
        assert_eq!(
            b.action_for(mid, egui::Modifiers::NONE),
            Some(NavAction::Pan)
        );
        assert_eq!(b.action_for(mid, egui::Modifiers::CTRL), None);
        assert_eq!(
            b.action_for(egui::PointerButton::Primary, egui::Modifiers::NONE),
            None
        );
        assert!(b.wheel_zoom_in_is_up && b.orbit_about_cursor && b.zoom_to_cursor);
        let blender = NavPreset::Blender.bindings();
        assert_eq!(
            blender.action_for(mid, egui::Modifiers::NONE),
            Some(NavAction::Orbit)
        );
        let json = serde_json::to_string(&b).unwrap();
        let back: NavBindings = serde_json::from_str(&json).unwrap();
        assert_eq!(back, b);
    }
}
