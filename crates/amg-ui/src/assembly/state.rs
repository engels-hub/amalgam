//! The module's per-document view state: the occurrence clipboard and the "positions out of
//! date" list.

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
}

impl Default for AssemblyLayer {
    fn default() -> AssemblyLayer {
        AssemblyLayer {
            clipboard: Vec::new(),
            out_of_date: Vec::new(),
            seen: None,
            source: amg_doc::joints::out_of_date,
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
