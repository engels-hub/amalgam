//! The module's per-document view state: the occurrence clipboard, the "positions out of
//! date" list, the Position panel and prompt flags and the Move tool's held preview.

use std::cell::Cell;
use std::rc::Rc;
use std::sync::Arc;

use amg_core::Transform;
use amg_doc::{ComponentId, Design, DesignMode, Evaluated, OccurrenceId, Revision};

use crate::app::AppCx;
use crate::layer::{DocLayer, LayerOut};

/// Id of the module's document layer.
pub const LAYER_ID: &str = "assembly.state";

/// How the out-of-date occurrences are found ([`amg_doc::joints::out_of_date`]).
pub type OutOfDateFn = fn(&Design, &Evaluated) -> Vec<OccurrenceId>;

/// A copied occurrence: what Paste places again. Session state, never saved.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Clip {
    /// The placed component.
    pub component: ComponentId,
    /// The copied placement, relative to its parent.
    pub transform: Transform,
}

/// The assembly state of one open document.
#[derive(Debug)]
pub struct AssemblyLayer {
    clipboard: Vec<Clip>,
    out_of_date: Vec<OccurrenceId>,
    /// The evaluated state `out_of_date` was found in.
    seen: Option<(Revision, Arc<Evaluated>)>,
    source: OutOfDateFn,
    /// The Position panel is open (shared with the panel, which clears it on close).
    pub(super) panel_open: Rc<Cell<bool>>,
    /// A Capture / Revert / Cancel prompt is open (shared with the prompt).
    pub(super) prompt_open: Rc<Cell<bool>>,
    /// The Move tool's preview stays shown until an evaluation after this revision arrives
    /// (`frames`: updates seen without a new revision, for a refused commit).
    held: Option<(Revision, u32)>,
}

impl Default for AssemblyLayer {
    fn default() -> AssemblyLayer {
        AssemblyLayer {
            clipboard: Vec::new(),
            out_of_date: Vec::new(),
            seen: None,
            source: amg_doc::joints::out_of_date,
            panel_open: Rc::new(Cell::new(false)),
            prompt_open: Rc::new(Cell::new(false)),
            held: None,
        }
    }
}

impl AssemblyLayer {
    /// The copied occurrences (empty before the first Copy).
    pub fn clipboard(&self) -> &[Clip] {
        &self.clipboard
    }

    pub(super) fn set_clipboard(&mut self, clips: Vec<Clip>) {
        self.clipboard = clips;
    }

    /// Occurrences whose solved position differs from the authored one, as of the last
    /// evaluation that matched the design.
    pub fn out_of_date(&self) -> &[OccurrenceId] {
        &self.out_of_date
    }

    /// Keeps the scene's placement overlay (the Move tool's preview) until the document has
    /// been evaluated after `revision`, so a committed move does not flash back to the old
    /// place while the placements are recomputed.
    pub(super) fn hold_overlay(&mut self, revision: Revision) {
        self.held = Some((revision, 0));
    }

    /// The Position panel is open.
    pub fn position_panel_open(&self) -> bool {
        self.panel_open.get()
    }

    /// A Capture / Revert / Cancel prompt is open.
    pub fn prompt_open(&self) -> bool {
        self.prompt_open.get()
    }

    /// Releases a held overlay once the evaluation caught up (or the commit was refused).
    fn release_overlay(&mut self, cx: &mut AppCx<'_>) {
        /// Updates after which a commit that changed nothing releases the overlay.
        const REFUSED_AFTER: u32 = 3;
        let Some((at, frames)) = self.held else {
            return;
        };
        let document = &cx.doc_tab().document;
        let moved_on = document.history().revision() != at;
        let release = if moved_on {
            document.is_evaluated()
        } else {
            frames >= REFUSED_AFTER
        };
        if release {
            self.held = None;
            cx.set_placement_overlay(None);
            cx.request_repaint();
        } else {
            self.held = Some((at, frames + 1));
            cx.request_repaint();
        }
    }

    /// Opens the Position panel while moves are pending (History mode).
    fn open_panel(&mut self, cx: &mut AppCx<'_>) {
        let design = cx.doc_tab().document.design();
        let wanted = design.meta().mode == DesignMode::History && design.has_pending_positions();
        if wanted && !self.panel_open.get() {
            self.panel_open.set(true);
            cx.open_dialog(Box::new(super::position::PositionPanel::new(
                Rc::clone(&self.panel_open),
                Rc::clone(&self.prompt_open),
            )));
        }
    }

    /// Replaces [`amg_doc::joints::out_of_date`]. Test seam: before the joint solver (ASM-04)
    /// solved placements always equal the authored ones, so nothing is ever out of date.
    #[cfg(feature = "testing")]
    pub fn set_out_of_date_source(&mut self, source: OutOfDateFn) {
        self.source = source;
        self.seen = None;
    }
}

impl DocLayer for AssemblyLayer {
    fn update(&mut self, cx: &mut AppCx<'_>, _out: &mut LayerOut<'_>) {
        self.release_overlay(cx);
        self.open_panel(cx);
        let document = &cx.doc_tab().document;
        // While a recompute is pending the placements are older than the design (a move that
        // was just made would look out of date), so the last answer is kept.
        if !document.is_evaluated() {
            return;
        }
        let revision = document.history().revision();
        let evaluated = document.evaluated();
        if self
            .seen
            .as_ref()
            .is_some_and(|(r, e)| *r == revision && Arc::ptr_eq(e, evaluated))
        {
            return;
        }
        let design = document.design();
        let list = if design.meta().mode == DesignMode::Direct {
            Vec::new()
        } else {
            (self.source)(design, evaluated)
        };
        self.seen = Some((revision, Arc::clone(evaluated)));
        if list != self.out_of_date {
            self.out_of_date = list;
            cx.request_repaint();
        }
    }
}
