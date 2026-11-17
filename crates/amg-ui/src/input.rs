//! Viewport input (spec §6, Interfaces §A `amg_ui::input`): [`ViewportEvent`]s synthesised
//! from egui's raw events, and the per-frame [`CameraView`].
//!
//! [`InputSynth`] turns the frame's `egui::Event`s into press / release / click / drag /
//! scroll / hover events for the viewport rectangle, in the order egui received them. A press
//! that moves more than [`DRAG_THRESHOLD_PT`] becomes a drag; a release without a drag is a
//! click. A drag that started in the viewport keeps reporting outside it until release.

use amg_core::{Mat4, Point3, Ray3};
use amg_render::Camera;

/// Pointer travel (points) after which a press becomes a drag.
pub const DRAG_THRESHOLD_PT: f32 = 3.0;
/// Two clicks within this time and distance are a double click.
pub const DOUBLE_CLICK_S: f64 = 0.3;
/// Maximum distance (points) between the clicks of a double click.
pub const DOUBLE_CLICK_PT: f32 = 6.0;
/// Points per wheel line (a `MouseWheelUnit::Line` step; the navigator zooms one step per line).
pub const POINTS_PER_LINE: f32 = 50.0;

/// One viewport input event, in egui logical points.
#[derive(Clone, Debug, PartialEq)]
pub enum ViewportEvent {
    /// The pointer moved with no button held.
    Hover {
        /// Cursor.
        pos: egui::Pos2,
        /// World ray through the cursor.
        ray: Ray3,
    },
    /// A button went down.
    Press {
        /// Button.
        button: egui::PointerButton,
        /// Cursor.
        pos: egui::Pos2,
        /// World ray through the cursor.
        ray: Ray3,
        /// Modifiers at the press.
        mods: egui::Modifiers,
    },
    /// A button went up (after `Press`; followed by `Click` or `DragEnd`).
    Release {
        /// Button.
        button: egui::PointerButton,
        /// Cursor.
        pos: egui::Pos2,
        /// World ray through the cursor.
        ray: Ray3,
        /// Modifiers at the release.
        mods: egui::Modifiers,
    },
    /// A press and release without a drag.
    Click {
        /// Button.
        button: egui::PointerButton,
        /// Cursor.
        pos: egui::Pos2,
        /// World ray through the cursor.
        ray: Ray3,
        /// Modifiers at the release.
        mods: egui::Modifiers,
        /// 1 for a single click, 2 for a double click.
        count: u8,
    },
    /// A press moved past the drag threshold.
    DragStart {
        /// Button.
        button: egui::PointerButton,
        /// Where the button went down.
        from: egui::Pos2,
        /// Modifiers at the press.
        mods: egui::Modifiers,
    },
    /// The pointer moved during a drag.
    Drag {
        /// Button.
        button: egui::PointerButton,
        /// Cursor.
        pos: egui::Pos2,
        /// Movement since the previous drag event (or the press).
        delta: egui::Vec2,
        /// World ray through the cursor.
        ray: Ray3,
        /// Modifiers held.
        mods: egui::Modifiers,
    },
    /// The button of a drag went up.
    DragEnd {
        /// Button.
        button: egui::PointerButton,
        /// Cursor.
        pos: egui::Pos2,
        /// World ray through the cursor.
        ray: Ray3,
        /// Modifiers at the release.
        mods: egui::Modifiers,
    },
    /// Mouse wheel or trackpad scroll, in points (egui sign: positive y = content moves down,
    /// i.e. the wheel turned up / away from the user).
    Scroll {
        /// Scroll amount, points.
        delta: egui::Vec2,
        /// Cursor.
        pos: egui::Pos2,
        /// Modifiers held.
        mods: egui::Modifiers,
    },
    /// A key went down or up.
    Key {
        /// The key.
        key: egui::Key,
        /// Pressed (true) or released.
        pressed: bool,
        /// Modifiers held.
        mods: egui::Modifiers,
    },
    /// Typed text (heads-up numeric input).
    Text(String),
}

impl ViewportEvent {
    /// The cursor position of a pointer event.
    pub fn pos(&self) -> Option<egui::Pos2> {
        match self {
            ViewportEvent::Hover { pos, .. }
            | ViewportEvent::Press { pos, .. }
            | ViewportEvent::Release { pos, .. }
            | ViewportEvent::Click { pos, .. }
            | ViewportEvent::Drag { pos, .. }
            | ViewportEvent::DragEnd { pos, .. }
            | ViewportEvent::Scroll { pos, .. } => Some(*pos),
            ViewportEvent::DragStart { from, .. } => Some(*from),
            ViewportEvent::Key { .. } | ViewportEvent::Text(_) => None,
        }
    }

    /// The button of a button or drag event.
    pub fn button(&self) -> Option<egui::PointerButton> {
        match self {
            ViewportEvent::Press { button, .. }
            | ViewportEvent::Release { button, .. }
            | ViewportEvent::Click { button, .. }
            | ViewportEvent::DragStart { button, .. }
            | ViewportEvent::Drag { button, .. }
            | ViewportEvent::DragEnd { button, .. } => Some(*button),
            _ => None,
        }
    }
}

/// Camera snapshot for the frame (from S10). Points are egui logical points.
#[derive(Clone, Debug, PartialEq)]
pub struct CameraView {
    /// The viewport rectangle, points.
    pub rect: egui::Rect,
    /// World-to-view matrix.
    pub view: Mat4,
    /// View-to-clip matrix (S10 reversed-Z, placeholder depth range).
    pub proj: Mat4,
    /// Orthographic lens.
    pub ortho: bool,
    /// The S10 camera the matrices come from (additive).
    pub camera: Camera,
    /// The viewport size, physical pixels (additive).
    pub size_px: [u32; 2],
    /// Points to pixels (additive).
    pub pixels_per_point: f32,
}

impl CameraView {
    /// The snapshot of `camera` drawn into `rect` at `pixels_per_point`.
    pub fn new(camera: Camera, rect: egui::Rect, pixels_per_point: f32) -> CameraView {
        let ppp = pixels_per_point.max(1e-3);
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "viewport sizes are small positive numbers"
        )]
        let size_px = [
            (rect.width() * ppp).round().max(1.0) as u32,
            (rect.height() * ppp).round().max(1.0) as u32,
        ];
        CameraView {
            rect,
            view: camera.view_matrix(),
            proj: camera.proj_matrix(size_px),
            ortho: matches!(
                camera.projection,
                amg_render::Projection::Orthographic { .. }
            ),
            camera,
            size_px,
            pixels_per_point: ppp,
        }
    }

    /// A screen position in the view's physical pixels (origin top-left of the viewport).
    pub fn to_px(&self, pos: egui::Pos2) -> [f64; 2] {
        let d = pos - self.rect.min;
        [
            f64::from(d.x * self.pixels_per_point),
            f64::from(d.y * self.pixels_per_point),
        ]
    }

    /// Physical view pixels back to a screen position.
    #[allow(
        clippy::cast_possible_truncation,
        reason = "screen coordinates are f32"
    )]
    pub fn from_px(&self, px: [f64; 2]) -> egui::Pos2 {
        self.rect.min
            + egui::vec2(
                px[0] as f32 / self.pixels_per_point,
                px[1] as f32 / self.pixels_per_point,
            )
    }

    /// Where a world point appears (`None` behind the camera).
    pub fn project(&self, p: Point3) -> Option<egui::Pos2> {
        let px = self.camera.project(p, self.size_px)?;
        Some(self.from_px([px[0], px[1]]))
    }

    /// The world ray through a screen position.
    pub fn ray(&self, pos: egui::Pos2) -> Ray3 {
        self.camera.ray(self.to_px(pos), self.size_px)
    }

    /// World length of one screen point at `p` (for constant-screen-size gizmos, pick radii).
    pub fn world_per_point(&self, p: Point3) -> f64 {
        self.camera.pixel_size_at(p, self.size_px) * f64::from(self.pixels_per_point)
    }
}

/// A button held down in the viewport.
#[derive(Copy, Clone, Debug)]
struct Down {
    button: egui::PointerButton,
    pos: egui::Pos2,
    last: egui::Pos2,
    mods: egui::Modifiers,
    dragging: bool,
}

/// Turns egui's raw events into [`ViewportEvent`]s; keeps press / drag state across frames.
#[derive(Clone, Debug, Default)]
pub struct InputSynth {
    down: Vec<Down>,
    last_click: Option<(egui::PointerButton, egui::Pos2, f64)>,
    pos: Option<egui::Pos2>,
}

impl InputSynth {
    /// A synthesiser with no button held.
    pub fn new() -> InputSynth {
        InputSynth::default()
    }

    /// True while a button that went down in the viewport is held.
    pub fn is_pressed(&self) -> bool {
        !self.down.is_empty()
    }

    /// The pointer events of this frame for the viewport `view`. A press counts only if
    /// `accept(pos)` (the pointer is over the viewport and not over a widget drawn on it).
    pub fn pointer_events(
        &mut self,
        input: &egui::InputState,
        view: &CameraView,
        accept: impl Fn(egui::Pos2) -> bool,
    ) -> Vec<ViewportEvent> {
        let mut out = Vec::new();
        let time = input.time;
        for e in &input.events {
            match e {
                egui::Event::PointerMoved(pos) => {
                    self.moved(*pos, input.modifiers, view, &accept, &mut out)
                }
                egui::Event::PointerButton {
                    pos,
                    button,
                    pressed,
                    modifiers,
                } => {
                    self.moved(*pos, *modifiers, view, &accept, &mut out);
                    if *pressed {
                        if accept(*pos) && !self.down.iter().any(|d| d.button == *button) {
                            self.down.push(Down {
                                button: *button,
                                pos: *pos,
                                last: *pos,
                                mods: *modifiers,
                                dragging: false,
                            });
                            out.push(ViewportEvent::Press {
                                button: *button,
                                pos: *pos,
                                ray: view.ray(*pos),
                                mods: *modifiers,
                            });
                        }
                    } else if let Some(i) = self.down.iter().position(|d| d.button == *button) {
                        let d = self.down.remove(i);
                        let ray = view.ray(*pos);
                        out.push(ViewportEvent::Release {
                            button: *button,
                            pos: *pos,
                            ray,
                            mods: *modifiers,
                        });
                        if d.dragging {
                            out.push(ViewportEvent::DragEnd {
                                button: *button,
                                pos: *pos,
                                ray,
                                mods: *modifiers,
                            });
                        } else {
                            let count = match self.last_click {
                                Some((b, p, t))
                                    if b == *button
                                        && time - t <= DOUBLE_CLICK_S
                                        && p.distance(*pos) <= DOUBLE_CLICK_PT =>
                                {
                                    2
                                }
                                _ => 1,
                            };
                            self.last_click = (count == 1).then_some((*button, *pos, time));
                            out.push(ViewportEvent::Click {
                                button: *button,
                                pos: *pos,
                                ray,
                                mods: *modifiers,
                                count,
                            });
                        }
                    }
                }
                egui::Event::MouseWheel {
                    unit,
                    delta,
                    modifiers,
                    ..
                } => {
                    let Some(pos) = self.pos.or(input.pointer.latest_pos()) else {
                        continue;
                    };
                    if !accept(pos) {
                        continue;
                    }
                    let scale = match unit {
                        egui::MouseWheelUnit::Point => 1.0,
                        egui::MouseWheelUnit::Line => POINTS_PER_LINE,
                        egui::MouseWheelUnit::Page => view.rect.height(),
                    };
                    out.push(ViewportEvent::Scroll {
                        delta: *delta * scale,
                        pos,
                        mods: *modifiers,
                    });
                }
                egui::Event::PointerGone => self.pos = None,
                _ => {}
            }
        }
        out
    }

    fn moved(
        &mut self,
        pos: egui::Pos2,
        mods: egui::Modifiers,
        view: &CameraView,
        accept: &impl Fn(egui::Pos2) -> bool,
        out: &mut Vec<ViewportEvent>,
    ) {
        if self.pos == Some(pos) {
            return;
        }
        self.pos = Some(pos);
        if self.down.is_empty() {
            if accept(pos) {
                out.push(ViewportEvent::Hover {
                    pos,
                    ray: view.ray(pos),
                });
            }
            return;
        }
        for d in &mut self.down {
            if !d.dragging && d.pos.distance(pos) > DRAG_THRESHOLD_PT {
                d.dragging = true;
                out.push(ViewportEvent::DragStart {
                    button: d.button,
                    from: d.pos,
                    mods: d.mods,
                });
            }
            if d.dragging {
                out.push(ViewportEvent::Drag {
                    button: d.button,
                    pos,
                    delta: pos - d.last,
                    ray: view.ray(pos),
                    mods,
                });
                d.last = pos;
            }
        }
    }
}

/// The key and text events of this frame (routed to the active tool before shortcuts).
pub fn key_events(input: &egui::InputState) -> Vec<ViewportEvent> {
    input
        .events
        .iter()
        .filter_map(|e| match e {
            egui::Event::Key {
                key,
                pressed,
                repeat: false,
                modifiers,
                ..
            } => Some(ViewportEvent::Key {
                key: *key,
                pressed: *pressed,
                mods: *modifiers,
            }),
            egui::Event::Text(t) => Some(ViewportEvent::Text(t.clone())),
            _ => None,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::{Dir3, Vec3};

    fn view() -> CameraView {
        let camera = Camera {
            eye: Point3::new(0.0, -100.0, 0.0),
            target: Point3::ZERO,
            up: Dir3::Z,
            projection: amg_render::Projection::Perspective { fov_y: 0.8 },
        };
        CameraView::new(
            camera,
            egui::Rect::from_min_size(egui::pos2(10.0, 20.0), egui::vec2(400.0, 300.0)),
            1.5,
        )
    }

    #[test]
    fn camera_view_projects_and_casts_consistently() {
        let v = view();
        let p = Point3::new(3.0, 5.0, -2.0);
        let s = v.project(p).unwrap();
        let ray = v.ray(s);
        let off = (p - ray.origin).cross(ray.dir.get()).length();
        assert!(off < 1e-4, "{off}");
        let centre = v.project(Point3::ZERO).unwrap();
        assert!((centre - v.rect.center()).length() < 1e-3);
        let wpp = v.world_per_point(Point3::ZERO);
        let one = v.project(Point3::new(wpp, 0.0, 0.0)).unwrap();
        assert!(((one - centre).length() - 1.0).abs() < 1e-3);
        let _ = Vec3::X;
    }

    fn input(events: Vec<egui::Event>, time: f64) -> egui::InputState {
        let ctx = egui::Context::default();
        let mut raw = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(800.0, 600.0),
            )),
            time: Some(time),
            ..Default::default()
        };
        raw.events = events.clone();
        let mut out = None;
        let mut full = ctx.run_ui(raw, |ui| out = Some(ui.input(|i| i.clone())));
        full.textures_delta.clear();
        out.unwrap()
    }

    fn button(pos: egui::Pos2, button: egui::PointerButton, pressed: bool) -> egui::Event {
        egui::Event::PointerButton {
            pos,
            button,
            pressed,
            modifiers: egui::Modifiers::default(),
        }
    }

    #[test]
    fn press_move_release_becomes_a_drag_and_a_still_press_a_click() {
        let v = view();
        let mut s = InputSynth::new();
        let p = egui::pos2(100.0, 100.0);
        let mid = egui::PointerButton::Middle;
        let ev = s.pointer_events(
            &input(
                vec![
                    egui::Event::PointerMoved(p),
                    button(p, mid, true),
                    egui::Event::PointerMoved(p + egui::vec2(1.0, 0.0)),
                    egui::Event::PointerMoved(p + egui::vec2(10.0, 0.0)),
                    button(p + egui::vec2(10.0, 0.0), mid, false),
                ],
                1.0,
            ),
            &v,
            |_| true,
        );
        let kinds: Vec<&str> = ev
            .iter()
            .map(|e| match e {
                ViewportEvent::Hover { .. } => "hover",
                ViewportEvent::Press { .. } => "press",
                ViewportEvent::DragStart { .. } => "drag_start",
                ViewportEvent::Drag { .. } => "drag",
                ViewportEvent::Release { .. } => "release",
                ViewportEvent::DragEnd { .. } => "drag_end",
                ViewportEvent::Click { .. } => "click",
                _ => "other",
            })
            .collect();
        assert_eq!(
            kinds,
            [
                "hover",
                "press",
                "drag_start",
                "drag",
                "release",
                "drag_end"
            ]
        );

        let prim = egui::PointerButton::Primary;
        let ev = s.pointer_events(
            &input(vec![button(p, prim, true), button(p, prim, false)], 2.0),
            &v,
            |_| true,
        );
        assert!(matches!(
            ev.last(),
            Some(ViewportEvent::Click { count: 1, .. })
        ));
        let ev = s.pointer_events(
            &input(vec![button(p, prim, true), button(p, prim, false)], 2.1),
            &v,
            |_| true,
        );
        assert!(matches!(
            ev.last(),
            Some(ViewportEvent::Click { count: 2, .. })
        ));
        // A press outside the accepted area is not the viewport's.
        let ev = s.pointer_events(
            &input(vec![button(p, prim, true), button(p, prim, false)], 5.0),
            &v,
            |_| false,
        );
        assert!(ev.is_empty());
    }
}
