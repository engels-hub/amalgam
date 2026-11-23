//! App-level dialogs of domain modules (additive, IO-15).
//!
//! A tool's dialog (UI-03) lives and dies with the tool. A module dialog that is not a tool
//! (the Insert Mesh import dialog, the export dialog) implements [`AppDialog`] and is opened
//! with [`AppCx::open_dialog`]. The shell draws every open dialog of the active document once
//! per frame, after the tool dialog; `show` runs with an [`AppCx`] on that document, so it can
//! submit commands, spawn jobs and open file pickers like a panel. A dialog of another
//! document stays open and is shown again when its document becomes active; closing the
//! document drops it.

use std::borrow::Cow;

use crate::app::AppCx;

/// A dialog opened by a module. Lives on the UI thread.
pub trait AppDialog: 'static {
    /// The window title.
    fn title(&self) -> Cow<'static, str>;
    /// Draws the dialog and polls its jobs. Returns false to close it.
    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool;
    /// For tests and `AmalgamApp::dialog`.
    fn as_any(&self) -> &dyn std::any::Any;
    /// Mutable access for tests and `AmalgamApp::dialog_mut`.
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}

/// A file-drop handler of a module: called with the active document's context for every
/// dropped path that is not an Amalgam design. Returns true if it took the file; the first
/// handler (in module id order) that does ends the search.
pub type DropHandler = fn(&mut AppCx<'_>, &std::path::Path) -> bool;

/// A boxed dialog that can sit in the (`Debug`) shell request queue.
pub(crate) struct BoxedDialog(pub(crate) Box<dyn AppDialog>);

impl std::fmt::Debug for BoxedDialog {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("AppDialog").field(&self.0.title()).finish()
    }
}

impl crate::app::AmalgamApp {
    /// Draws the open dialogs of the active document and drops the ones that closed or whose
    /// document is gone.
    pub(crate) fn draw_dialogs(&mut self, ctx: &egui::Context) {
        if self.dialogs.is_empty() {
            return;
        }
        let ids: Vec<crate::app::DocTabId> = self.docs.iter().map(|d| d.id).collect();
        self.dialogs.retain(|(doc, _)| ids.contains(doc));
        let active = self.docs[self.active].id;
        let mut open = std::mem::take(&mut self.dialogs);
        open.retain_mut(|(doc, dialog)| {
            if *doc != active {
                return true;
            }
            self.with_cx(|cx| dialog.show(ctx, cx))
        });
        // Dialogs opened while drawing went to the request queue, not here.
        open.append(&mut self.dialogs);
        self.dialogs = open;
    }

    /// Offers every dropped file that is not an Amalgam design to the modules' drop handlers
    /// (`.amg` files open as documents). Called once per frame from `logic`.
    pub(crate) fn poll_dropped_files(&mut self, ctx: &egui::Context) {
        let paths: Vec<std::path::PathBuf> = ctx.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .map(|f| f.path().to_owned())
                .collect()
        });
        if paths.is_empty() {
            return;
        }
        let handlers = self.registry.drop_handlers();
        for path in paths {
            let is_design = path
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case(crate::file::AMG_EXTENSION));
            if is_design {
                self.open_path(path);
                continue;
            }
            let taken = self.with_cx(|cx| handlers.iter().any(|h| h(cx, &path)));
            if !taken {
                self.notify(crate::notify::Notification::warning(
                    "Cannot open the file",
                    format!("{}: no importer for this file", path.display()),
                ));
            }
        }
        ctx.request_repaint();
    }

    /// The first open dialog of type `T` (tests and modules).
    pub fn dialog<T: 'static>(&self) -> Option<&T> {
        self.dialogs
            .iter()
            .find_map(|(_, d)| d.as_any().downcast_ref::<T>())
    }

    /// Titles of the open dialogs.
    pub fn dialog_titles(&self) -> Vec<String> {
        self.dialogs
            .iter()
            .map(|(_, d)| d.title().into_owned())
            .collect()
    }
}

impl crate::app::AmalgamApp {
    /// The first open dialog of type `T`, mutable (tests drive dialog state through it).
    pub fn dialog_mut<T: 'static>(&mut self) -> Option<&mut T> {
        self.dialogs
            .iter_mut()
            .find_map(|(_, d)| d.as_any_mut().downcast_mut::<T>())
    }
}
