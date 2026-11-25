//! The UI-13 creation tool: one [`Tool`] for every [`CreateKind`]. It collects picks (click,
//! or type values in the heads-up input and press Enter), infers each pick like the Line tool
//! (glyphs, guides, auto-constraints; Ctrl suppresses), previews the shape, and commits the
//! SK-21 / SK-22 builder with its inferred constraints and typed driving dimensions as one
//! undo entry. The tool stays active for the next shape; Esc closes the heads-up input, then
//! drops the picks, then leaves the tool.

use std::borrow::Cow;

use amg_core::{Point2, Vec2};
use amg_doc::UnitPrefs;
use amg_render::{DepthMode, LabelStyle, LineStyle, OverlayList, PointStyle};
use amg_sketch::query::{self, PickFilter};
use amg_sketch::{EditResult, End, EntityId, EntityKind, Sketch};

use super::infer::{Anchor, InferInput, Inference, SNAP_RADIUS_PT, infer};
use super::session::with_session;
use super::shapes::{CreateKind, Pick, PolygonOptions, Typed, build_shape, follow_ups, resolve};
use super::tools::{
    Cached, HeadsUp, PICK_RADIUS_PT, cursor_on, eval_typed, mark_construction, mods_ctrl, nice_step,
};
use super::view::{self, colors};
use crate::app::CommitBatch;
use crate::command::CmdId;
use crate::input::ViewportEvent;
use crate::select::EntityFilter;
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};

/// A sketch creation tool (see [`CreateKind`]).
#[derive(Debug)]
pub struct CreateTool {
    kind: CreateKind,
    cache: Cached,
    picks: Vec<Pick>,
    /// The inferred cursor pick (not yet placed).
    cursor: Option<Pick>,
    mmpp: f64,
    ctrl: bool,
    heads_up: Option<HeadsUp>,
    /// Signed angle the cursor has turned about an arc's centre since its start pick.
    turn: f64,
    polygon: PolygonOptions,
    /// Why the last pick was refused (status bar).
    problem: Option<String>,
    shapes: u32,
}

impl CreateTool {
    /// A new tool of `kind`.
    pub fn new(kind: CreateKind, _init: &ToolInit<'_>) -> CreateTool {
        CreateTool {
            kind,
            cache: Cached::default(),
            picks: Vec::new(),
            cursor: None,
            mmpp: 1.0,
            ctrl: false,
            heads_up: None,
            turn: 0.0,
            polygon: PolygonOptions::default(),
            problem: None,
            shapes: 0,
        }
    }

    fn reset(&mut self) {
        self.picks.clear();
        self.heads_up = None;
        self.turn = 0.0;
        self.problem = None;
    }

    fn anchor(&self) -> Option<Anchor> {
        if !self.kind.directional(self.picks.len()) {
            return None;
        }
        self.picks.first().map(|p| Anchor {
            at: p.at,
            entity: None,
        })
    }

    fn infer_at(&self, sketch: &Sketch, p: Point2) -> Inference {
        let grid = self.cache.grid.then(|| nice_step(self.mmpp * 20.0));
        infer(
            sketch,
            p,
            &InferInput {
                anchor: self.anchor(),
                radius: self.mmpp * SNAP_RADIUS_PT,
                suppress: self.ctrl,
                grid,
                exclude: &[],
            },
        )
    }

    /// The pick the cursor position `p` stands for at the current stage; `None` if the stage
    /// needs a curve and there is none under the cursor.
    fn pick_at(&self, sketch: &Sketch, p: Point2) -> Option<Pick> {
        let stage = self.picks.len();
        let radius = self.mmpp * PICK_RADIUS_PT;
        if self.kind.picks_curve(stage) {
            let first = self.picks.first().and_then(|q| q.curve);
            let curve = query::pick(sketch, p, radius, PickFilter::ALL)
                .into_iter()
                .map(|h| h.entity)
                .find(|e| {
                    Some(*e) != first
                        && matches!(
                            sketch.entity(*e).map(|x| &x.kind),
                            Some(
                                EntityKind::Line { .. }
                                    | EntityKind::Circle { .. }
                                    | EntityKind::Arc { .. }
                            )
                        )
                })?;
            return Some(Pick {
                curve: Some(curve),
                ..Pick::at(p)
            });
        }
        if self.kind == CreateKind::ArcTangent && stage == 0 {
            let (curve, end, at) = curve_end(sketch, p, radius * 1.5)?;
            return Some(Pick {
                curve: Some(curve),
                end: Some(end),
                ..Pick::at(at)
            });
        }
        let inference = self.infer_at(sketch, p);
        Some(Pick {
            at: inference.point,
            inference,
            clockwise: self.turn < 0.0,
            ..Pick::default()
        })
    }

    /// Tracks which way the cursor goes round an arc's centre.
    fn track_turn(&mut self, p: Point2) {
        if !matches!(self.kind, CreateKind::ArcCenter | CreateKind::SlotArc) {
            return;
        }
        let [c, start] = [self.picks.first(), self.picks.get(1)].map(|q| q.map(|q| q.at));
        let (Some(c), Some(start), 2) = (c, start, self.picks.len()) else {
            return;
        };
        let (a, b): (Vec2, Vec2) = (start - c, p - c);
        let now = a.perp_dot(b).atan2(a.dot(b));
        // Unwrap: the nearest angle to the previous one, kept within one turn.
        let tau = std::f64::consts::TAU;
        let mut turn = now + ((self.turn - now) / tau).round() * tau;
        if turn.abs() >= tau {
            turn = now;
        }
        self.turn = turn;
    }

    fn update_cursor(&mut self, cx: &mut ToolCx<'_>, ray: &amg_core::Ray3) {
        let (Some(plane), Some(sketch)) = (self.cache.plane, self.cache.sketch.clone()) else {
            return;
        };
        let Some((p, mmpp)) = cursor_on(&plane, cx.camera(), ray) else {
            return;
        };
        self.mmpp = mmpp;
        self.track_turn(p);
        self.cursor = self.pick_at(&sketch, p);
        // The curve a curve-picking stage would take is highlighted.
        let hovered = self.cursor.as_ref().and_then(|c| c.curve);
        with_session(cx.app(), |s, _| s.hovered = hovered);
    }

    /// Adds `pick`; commits the shape when it was the last one. A degenerate pick is refused
    /// and nothing changes.
    fn place(&mut self, cx: &mut ToolCx<'_>, pick: Pick) {
        let repeated = self.picks.last().is_some_and(|q| {
            q.curve.is_none() && q.at.distance(pick.at) <= 1e-9 * self.mmpp.max(1.0)
        });
        if repeated {
            return;
        }
        self.problem = None;
        self.picks.push(pick);
        if self.picks.len() < self.kind.picks() {
            self.heads_up = None;
            return;
        }
        let (kind, polygon, picks) = (self.kind, self.polygon, self.picks.clone());
        let res = with_session(cx.app(), |s, app| {
            let construction = s.construction;
            s.commit_op(
                app,
                kind.def().label,
                |f| {
                    f.edit_sketch(|sk| {
                        let built = build_shape(kind, sk, &picks, polygon)?;
                        if construction {
                            mark_construction(sk, &built.entities)?;
                        }
                        Ok(EditResult {
                            created: built.entities,
                            constraints_added: built.constraints,
                            ..EditResult::default()
                        })
                    })
                },
                |res, sk| {
                    follow_ups(
                        kind,
                        &picks,
                        &res.created,
                        &res.constraints_added,
                        sk,
                        polygon,
                    )
                },
            )
        });
        match res {
            Some(Ok(_)) => {
                self.shapes += 1;
                self.reset();
                self.cursor = None;
            }
            Some(Err(e)) => {
                self.picks.pop();
                self.problem = Some(e.to_string());
            }
            None => {
                self.picks.pop();
            }
        }
    }

    /// Places the pick described by the heads-up fields. Fields that do not evaluate keep the
    /// input open.
    fn place_typed(&mut self, cx: &mut ToolCx<'_>) {
        let Some(hu) = self.heads_up.clone() else {
            return;
        };
        let fields = self.kind.fields(self.picks.len());
        let params = cx.env().design.params().clone();
        let prefs = UnitPrefs::default();
        let mut typed: [Option<Typed>; 2] = [None, None];
        for (i, field) in fields.iter().enumerate() {
            let src = hu.fields[i].trim();
            if src.is_empty() {
                continue;
            }
            match eval_typed(&params, &prefs, src, field.quantity()) {
                Some((value, expr)) if !field.positive() || value > 0.0 => {
                    typed[i] = Some(Typed { value, expr });
                }
                _ => {
                    self.problem = Some(format!("{} {src} is not a valid value", field.name()));
                    return;
                }
            }
        }
        let Some(sketch) = self.cache.sketch.clone() else {
            return;
        };
        let cursor = self.cursor.clone().unwrap_or_else(|| {
            let base = self.picks.last().map_or(Point2::ZERO, |p| p.at);
            Pick::at(base + Vec2::new(1.0, 1.0))
        });
        let values = [0, 1].map(|i| typed[i].as_ref().map(|t| t.value));
        if values.iter().all(Option::is_none) {
            self.heads_up = None;
            self.place(cx, cursor);
            return;
        }
        let Some(at) = resolve(self.kind, &self.picks, &cursor, values, &sketch) else {
            self.problem = Some("no shape has these values".to_owned());
            return;
        };
        let pick = Pick {
            typed,
            clockwise: cursor.clockwise,
            ..Pick::at(at)
        };
        let before = (self.picks.len(), self.shapes);
        self.place(cx, pick);
        if before != (self.picks.len(), self.shapes) {
            self.heads_up = None;
        }
    }

    /// Mirrors the inference and heads-up input into the session (tests, scripts).
    fn mirror(&self, cx: &mut ToolCx<'_>) {
        let inf = self.cursor.as_ref().map(|c| c.inference.clone());
        let hu = self.heads_up.clone();
        with_session(cx.app(), |s, _| {
            s.inference = inf;
            s.heads_up = hu;
        });
    }

    fn handle(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        self.ctrl = match ev {
            ViewportEvent::Press { mods, .. }
            | ViewportEvent::Release { mods, .. }
            | ViewportEvent::Click { mods, .. }
            | ViewportEvent::Drag { mods, .. }
            | ViewportEvent::DragStart { mods, .. }
            | ViewportEvent::Key { mods, .. } => mods_ctrl(*mods),
            _ => mods_ctrl(cx.modifiers()),
        };
        let fields = self.kind.fields(self.picks.len()).len();
        match ev {
            ViewportEvent::Hover { ray, .. } => {
                self.update_cursor(cx, ray);
                InputResult::Consumed
            }
            ViewportEvent::Click {
                button: egui::PointerButton::Primary,
                ray,
                ..
            } => {
                if self.cache.busy {
                    return InputResult::Consumed;
                }
                self.update_cursor(cx, ray);
                if let Some(pick) = self.cursor.clone() {
                    self.place(cx, pick);
                }
                InputResult::Consumed
            }
            ViewportEvent::Text(t) if fields > 0 => {
                let starts_number = t
                    .chars()
                    .all(|c| c.is_ascii_digit() || ".-+*/() ".contains(c))
                    || self.heads_up.is_some();
                if !starts_number {
                    return InputResult::Ignored;
                }
                let hu = self.heads_up.get_or_insert_with(HeadsUp::default);
                hu.fields[hu.active].push_str(t);
                InputResult::Consumed
            }
            ViewportEvent::Key {
                key, pressed: true, ..
            } => match key {
                egui::Key::Tab if self.heads_up.is_some() => {
                    if let Some(hu) = &mut self.heads_up {
                        hu.active = (hu.active + 1) % fields.max(1);
                    }
                    InputResult::Consumed
                }
                egui::Key::Backspace if self.heads_up.is_some() => {
                    if let Some(hu) = &mut self.heads_up {
                        hu.fields[hu.active].pop();
                    }
                    InputResult::Consumed
                }
                egui::Key::Enter if self.heads_up.is_some() => {
                    if !self.cache.busy {
                        self.place_typed(cx);
                    }
                    InputResult::Consumed
                }
                egui::Key::Escape if self.heads_up.is_some() => {
                    self.heads_up = None;
                    self.problem = None;
                    InputResult::Consumed
                }
                egui::Key::Escape if !self.picks.is_empty() => {
                    self.reset();
                    InputResult::Consumed
                }
                egui::Key::Escape => InputResult::Ignored,
                // Typing owns the keyboard: no shortcut fires from a heads-up expression.
                _ if self.heads_up.is_some() => InputResult::Consumed,
                _ => InputResult::Ignored,
            },
            ViewportEvent::Key { .. } if self.heads_up.is_some() => InputResult::Consumed,
            _ => InputResult::Ignored,
        }
    }

    /// The shape as it would be committed with the cursor pick, on a scratch sketch.
    fn preview(&self, cursor: &Pick) -> Option<(Sketch, Vec<EntityId>)> {
        if self.picks.len() + 1 != self.kind.picks() {
            return None;
        }
        let display = self.cache.sketch.as_ref()?;
        let mut scratch = if self.kind.needs_sketch() {
            (**display).clone()
        } else {
            Sketch::new(*display.plane())
        };
        let mut picks = self.picks.clone();
        picks.push(cursor.clone());
        let built = build_shape(self.kind, &mut scratch, &picks, self.polygon).ok()?;
        Some((scratch, built.entities))
    }
}

/// The end of a line or arc at the point under `p`: curve, end and the point's position.
fn curve_end(s: &Sketch, p: Point2, radius: f64) -> Option<(EntityId, End, Point2)> {
    for hit in query::pick(s, p, radius, PickFilter::ALL) {
        let Some(at) = s.point(hit.entity) else {
            continue;
        };
        for (id, e) in s.entities() {
            if !matches!(e.kind, EntityKind::Line { .. } | EntityKind::Arc { .. }) {
                continue;
            }
            for end in [End::End, End::Start] {
                if e.kind.endpoint(end) == Some(hit.entity) {
                    return Some((id, end, at));
                }
            }
        }
    }
    None
}

impl Tool for CreateTool {
    fn id(&self) -> CmdId {
        self.kind.def().id
    }

    fn label(&self) -> Cow<'static, str> {
        self.kind.def().label.into()
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::NONE
    }

    fn start(&mut self, cx: &mut ToolCx<'_>) {
        // `X` now sets how new geometry is drawn rather than changing an old selection.
        with_session(cx.app(), |s, _| {
            s.selected.clear();
            s.hovered = None;
        });
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        let step = self.kind.step_hint(self.picks.len());
        let typed = if self.kind.fields(self.picks.len()).is_empty() {
            ""
        } else {
            " or type values"
        };
        let construction = if self.cache.construction {
            " · construction"
        } else {
            ""
        };
        let problem = self
            .problem
            .as_ref()
            .map(|p| format!(" · {p}"))
            .unwrap_or_default();
        Some(
            format!(
                "{} · {}: {step}{typed} (Ctrl: no inference){construction}{problem}",
                self.cache.dof_text(),
                self.kind.def().label,
            )
            .into(),
        )
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        if !self.cache.refresh(cx) {
            cx.request_finish(FinishReason::Cancel);
            return;
        }
        self.mirror(cx);
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        let r = self.handle(ev, cx);
        self.mirror(cx);
        r
    }

    fn dialog_ui(&mut self, ui: &mut egui::Ui, _cx: &mut ToolCx<'_>) -> bool {
        if self.kind != CreateKind::Polygon {
            return false;
        }
        ui.horizontal(|ui| {
            ui.label("Sides");
            ui.add(egui::DragValue::new(&mut self.polygon.sides).range(3..=128));
        });
        ui.radio_value(&mut self.polygon.inscribed, true, "Inscribed");
        ui.radio_value(&mut self.polygon.inscribed, false, "Circumscribed");
        true
    }

    fn overlay(&mut self, out: &mut OverlayList, _cx: &ToolCx<'_>) {
        let (Some(plane), Some(sketch)) = (self.cache.plane, self.cache.sketch.clone()) else {
            return;
        };
        let Some(cursor) = &self.cursor else {
            return;
        };
        let preview = LineStyle {
            width_px: view::CURVE_PX,
            color: colors::PREVIEW,
            dash: self.cache.construction.then_some((6.0, 4.0)),
            depth: DepthMode::AlwaysOnTop,
        };
        let guide = LineStyle {
            width_px: 1.0,
            color: colors::INFER,
            dash: Some((4.0, 4.0)),
            depth: DepthMode::AlwaysOnTop,
        };
        match self.preview(cursor) {
            Some((scratch, new)) => {
                for e in new {
                    let Some(c) = scratch.curve(e) else { continue };
                    let construction = scratch.entity(e).is_some_and(|x| x.construction);
                    let style = LineStyle {
                        dash: if construction {
                            Some((6.0, 4.0))
                        } else {
                            preview.dash
                        },
                        ..preview
                    };
                    view::polyline(out, &plane, &view::sample(&c), style);
                }
            }
            None => {
                // Not enough picks for the shape yet: a rubber band through them.
                let mut line: Vec<Point2> = self
                    .picks
                    .iter()
                    .filter(|p| !(p.curve.is_some() && p.end.is_none()))
                    .map(|p| p.at)
                    .collect();
                if !line.is_empty() {
                    line.push(cursor.at);
                    view::polyline(out, &plane, &line, preview);
                }
            }
        }
        for (a, b) in cursor.inference.guides(&sketch, self.anchor()) {
            view::polyline(out, &plane, &[a, b], guide);
        }
        out.points(
            &[plane.to_world(cursor.at)],
            PointStyle {
                size_px: view::POINT_PX,
                color: colors::PREVIEW,
                depth: DepthMode::AlwaysOnTop,
            },
            None,
        );
        if let Some(g) = cursor.inference.glyph() {
            out.label(
                plane.to_world(cursor.at),
                g,
                LabelStyle {
                    size_pt: 11.0,
                    color: colors::INFER,
                    offset_px: [18.0, -16.0],
                    background: Some(colors::LABEL_BG),
                },
            );
        }
        if let Some(hu) = &self.heads_up {
            for (i, field) in self.kind.fields(self.picks.len()).iter().enumerate() {
                let caret = if hu.active == i { "▏" } else { "" };
                out.label(
                    plane.to_world(cursor.at),
                    format!("{} {}{caret}", field.name(), hu.fields[i]),
                    LabelStyle {
                        size_pt: 12.0,
                        color: if hu.active == i {
                            colors::FULL
                        } else {
                            colors::UNDER
                        },
                        offset_px: [70.0, 18.0 + 20.0 * i as f32],
                        background: Some(colors::LABEL_BG),
                    },
                );
            }
        }
    }

    fn finish(&mut self, _reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        with_session(cx.app(), |s, _| {
            s.inference = None;
            s.heads_up = None;
            s.hovered = None;
        });
        None
    }
}
