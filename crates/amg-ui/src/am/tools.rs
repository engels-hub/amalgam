//! Place on Plate: pick a face of a plate item; the item turns so that face lies on the plate.
//!
//! The click is a point pick with the face filter. The rotation (`pick_plane_normal` over the
//! face's group, then `face_down_rotation`) runs as a job because a face group of a 1 M-triangle
//! mesh can hold every triangle; the job's result is applied to the setup current at that time
//! as one `SetExt` (one undo entry). The tool finishes after the pick.

use std::borrow::Cow;

use amg_am::place::{ZMode, face_down_rotation, pick_plane_normal};
use amg_doc::TopoKind;
use amg_jobs::SpawnOptions;
use amg_mesh::FaceId;

use super::{
    FnJob, PLACE_FACE_DOWN, active_items, current_setup, drop_elevation, mesh_of, submit_setup,
};
use crate::app::CommitBatch;
use crate::command::CmdId;
use crate::input::ViewportEvent;
use crate::notify::Notification;
use crate::pick::{PickMode, PickTarget, PickTicket};
use crate::select::EntityFilter;
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};

/// The Place on Plate tool.
#[derive(Debug, Default)]
pub struct PlaceOnPlateTool {
    click: Option<PickTicket>,
    hint: Option<&'static str>,
}

impl PlaceOnPlateTool {
    /// A new tool.
    pub fn new(_init: &ToolInit<'_>) -> PlaceOnPlateTool {
        PlaceOnPlateTool::default()
    }

    fn on_hit(&mut self, target: PickTarget, cx: &mut ToolCx<'_>) {
        let PickTarget::Body {
            body,
            sub: Some((TopoKind::Face, face)),
            ..
        } = target
        else {
            self.hint = Some("Pick a face of a plate item");
            return;
        };
        let app = cx.app();
        let Some((key, s)) = current_setup(app) else {
            return;
        };
        let Some(item) = active_items(&s).iter().find(|i| i.body == body).cloned() else {
            self.hint = Some("That body is not on the plate: Add Bodies first");
            return;
        };
        let Some(mesh) = mesh_of(app.env().evaluated, body) else {
            self.hint = Some("Only mesh bodies can be placed");
            return;
        };
        let current = item.placement.rotation;
        let job = FnJob::new("Place on plate", move |ctx: &amg_jobs::JobCtx| {
            ctx.checkpoint()?;
            let n = pick_plane_normal(&mesh, FaceId(face));
            Ok(face_down_rotation(current, n))
        });
        let id = item.id;
        app.spawn_tracked(job, SpawnOptions::default(), move |res, cx| {
            let Ok(rotation) = res else { return };
            // Apply to the setup as it is now (it may have changed while the job ran).
            let Some((k, mut s)) = current_setup(cx) else {
                return;
            };
            if k != key {
                return;
            }
            let elevation = match s.item_mut(id).map(|i| i.placement.z) {
                Some(ZMode::OnPlate { elevation }) => elevation,
                Some(ZMode::Fixed { .. }) => drop_elevation(&s),
                None => return,
            };
            if let Some(it) = s.item_mut(id) {
                it.placement.rotation = rotation;
                it.placement.z = ZMode::OnPlate { elevation };
                it.placement = it.placement.normalized();
            }
            submit_setup(cx, "Place on Plate", k, &s);
        });
        cx.request_finish(FinishReason::Commit);
    }
}

impl Tool for PlaceOnPlateTool {
    fn id(&self) -> CmdId {
        PLACE_FACE_DOWN
    }

    fn label(&self) -> Cow<'static, str> {
        Cow::Borrowed("Place on Plate")
    }

    fn start(&mut self, cx: &mut ToolCx<'_>) {
        if current_setup(cx.app()).is_none() {
            cx.app().notify(Notification::info(
                "Place on Plate",
                "Create a print setup first (New Setup).",
            ));
            cx.request_finish(FinishReason::Cancel);
        }
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        match ev {
            ViewportEvent::Click {
                button: egui::PointerButton::Primary,
                pos,
                ..
            } => {
                self.click = Some(cx.pick_point(*pos, PickMode::Nearest));
                InputResult::Consumed
            }
            _ => InputResult::Ignored,
        }
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        cx.set_cursor(egui::CursorIcon::Crosshair);
        if let Some(t) = self.click
            && let Some(hits) = cx.poll_pick(t)
        {
            self.click = None;
            match hits.into_iter().next() {
                Some(h) => self.on_hit(h.target, cx),
                None => self.hint = Some("Nothing there: pick a face of a plate item"),
            }
        }
    }

    fn pick_filter(&self) -> EntityFilter {
        EntityFilter::FACE
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        Some(Cow::Borrowed(self.hint.unwrap_or(
            "Place on Plate: pick the face that goes down on the plate (Esc cancels)",
        )))
    }

    fn finish(&mut self, _reason: FinishReason, _cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        // The edit is submitted when the rotation job finishes.
        None
    }
}
