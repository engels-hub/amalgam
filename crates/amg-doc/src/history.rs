//! Snapshot undo/redo and transactions (S01 §6, ADR-014).
//!
//! [`History`] owns the current [`DesignSnapshot`] and an [`UndoStack`] of entries
//! `{label, before, after, commands}`. Undo and redo swap `current` for `before` / `after`
//! (an `Arc` clone, O(1)); there are no inverse commands. Evaluated state is not touched here:
//! the document's recompute layer (DOC-03) reacts to [`History::revision`] or to
//! `Arc::ptr_eq` on [`History::snapshot`] changing and hits its cache.
//!
//! A [`Transaction`] edits a private scratch design and publishes it as one entry on
//! [`Transaction::commit`]; dropping or [`Transaction::rollback`]-ing it discards everything,
//! so `?` inside a transaction rolls it back.

use std::collections::VecDeque;
use std::sync::Arc;

use crate::command::{Command, CommandError, CommandOutcome, apply_command};
use crate::id::Revision;
use crate::model::{Design, DesignSnapshot};

/// Default undo depth (S01 §6).
pub const DEFAULT_UNDO_DEPTH: usize = 500;

/// One undoable step.
#[derive(Clone, Debug)]
pub struct UndoEntry {
    /// Menu label ("Add feature"), or the transaction label.
    pub label: String,
    /// State before the step.
    pub before: DesignSnapshot,
    /// State after the step.
    pub after: DesignSnapshot,
    /// The commands that produced it, for the crash journal, macros and scripting history.
    pub commands: Vec<Command>,
    seq: u64,
}

/// The undo stack: applied entries followed by redoable ones, bounded by a depth limit.
#[derive(Debug)]
pub struct UndoStack {
    entries: VecDeque<UndoEntry>,
    /// Number of applied entries; `entries[cursor..]` can be redone.
    cursor: usize,
    limit: usize,
}

impl UndoStack {
    fn new(limit: usize) -> UndoStack {
        UndoStack {
            entries: VecDeque::new(),
            cursor: 0,
            limit: limit.max(1),
        }
    }

    /// All entries, oldest first (applied ones are `..applied()`).
    pub fn entries(&self) -> impl Iterator<Item = &UndoEntry> + '_ {
        self.entries.iter()
    }

    /// Number of applied (undoable) entries.
    pub fn applied(&self) -> usize {
        self.cursor
    }

    /// Number of undone (redoable) entries.
    pub fn redoable(&self) -> usize {
        self.entries.len() - self.cursor
    }

    /// Maximum number of entries kept.
    pub fn limit(&self) -> usize {
        self.limit
    }
}

/// Observer of committed history changes (the crash journal, DOC-14). Called on the thread that
/// mutates the [`History`] (the UI thread), after the change; implementations must not block.
pub trait HistoryObserver: Send + Sync {
    /// A new undo entry was committed with these commands and label.
    fn on_commit(&self, label: &str, commands: &[Command]);
    /// A step was undone.
    fn on_undo(&self);
    /// A step was redone.
    fn on_redo(&self);
    /// A new undo entry was committed whose commands cannot be replayed on the design alone
    /// (they needed evaluated geometry: entering Direct mode, a Direct-mode bake, DOC-08).
    /// The default treats it like any commit; the crash journal overrides it to snapshot.
    fn on_opaque_commit(&self, label: &str, commands: &[Command]) {
        self.on_commit(label, commands);
    }
}

/// Holder of the optional observer (keeps `History: Debug`).
#[derive(Clone, Default)]
struct ObserverSlot(Option<Arc<dyn HistoryObserver>>);

impl std::fmt::Debug for ObserverSlot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.0.is_some() {
            "Observer(set)"
        } else {
            "Observer(none)"
        })
    }
}

/// The current design plus its undo history.
#[derive(Debug)]
pub struct History {
    observer: ObserverSlot,
    current: DesignSnapshot,
    stack: UndoStack,
    revision: Revision,
    next_seq: u64,
    /// `seq` of the state after the oldest dropped entry (0 initially).
    base_seq: u64,
    saved_seq: Option<u64>,
}

impl History {
    /// History for a design with the default depth.
    pub fn new(design: Design) -> History {
        History::with_limit(design, DEFAULT_UNDO_DEPTH)
    }

    /// History with a custom depth limit (at least 1).
    pub fn with_limit(design: Design, limit: usize) -> History {
        History {
            observer: ObserverSlot::default(),
            current: Arc::new(design),
            stack: UndoStack::new(limit),
            revision: Revision(0),
            next_seq: 1,
            base_seq: 0,
            saved_seq: Some(0),
        }
    }

    /// The current state, O(1).
    pub fn snapshot(&self) -> DesignSnapshot {
        Arc::clone(&self.current)
    }

    /// The current design.
    pub fn design(&self) -> &Design {
        &self.current
    }

    /// Monotonic counter: increases on every apply, commit, undo and redo that changes the
    /// design (unlike `Design::revision`, which goes back with undo). Runtime only.
    pub fn revision(&self) -> Revision {
        self.revision
    }

    /// Installs (or removes) the observer told about commits, undo and redo.
    pub fn set_observer(&mut self, observer: Option<Arc<dyn HistoryObserver>>) {
        self.observer = ObserverSlot(observer);
    }

    /// The undo stack.
    pub fn stack(&self) -> &UndoStack {
        &self.stack
    }

    /// Changes the depth limit, dropping the oldest entries if needed.
    pub fn set_limit(&mut self, limit: usize) {
        self.stack.limit = limit.max(1);
        self.enforce_limit();
    }

    /// Applies one command as one undo entry. A command that changes nothing creates no
    /// entry. On error nothing changes.
    pub fn apply(&mut self, cmd: Command) -> Result<CommandOutcome, CommandError> {
        let label = cmd.label();
        let mut work = (*self.current).clone();
        let created = apply_command(&mut work, cmd.clone())?;
        let revision = self.publish(label.to_owned(), work, vec![cmd]);
        Ok(CommandOutcome { created, revision })
    }

    /// Publishes `work` (derived from the current state outside `apply_command`, e.g. by
    /// direct-mode baking) as one undo entry. Returns the new revision.
    pub(crate) fn push_design(
        &mut self,
        label: String,
        work: Design,
        commands: Vec<Command>,
    ) -> Revision {
        self.publish_as(label, work, commands, false)
    }

    /// Starts a transaction: several commands, one undo entry.
    pub fn transaction(&mut self, label: &str) -> Transaction<'_> {
        Transaction {
            work: (*self.current).clone(),
            h: self,
            label: label.to_owned(),
            commands: Vec::new(),
        }
    }

    /// Steps back. Returns false if there is nothing to undo.
    pub fn undo(&mut self) -> bool {
        if self.stack.cursor == 0 {
            return false;
        }
        self.stack.cursor -= 1;
        self.current = Arc::clone(&self.stack.entries[self.stack.cursor].before);
        self.revision = self.revision.next();
        if let Some(o) = &self.observer.0 {
            o.on_undo();
        }
        true
    }

    /// Steps forward. Returns false if there is nothing to redo.
    pub fn redo(&mut self) -> bool {
        let Some(e) = self.stack.entries.get(self.stack.cursor) else {
            return false;
        };
        self.current = Arc::clone(&e.after);
        self.stack.cursor += 1;
        self.revision = self.revision.next();
        if let Some(o) = &self.observer.0 {
            o.on_redo();
        }
        true
    }

    /// Label of the step `undo` would revert.
    pub fn undo_label(&self) -> Option<&str> {
        let i = self.stack.cursor.checked_sub(1)?;
        self.stack.entries.get(i).map(|e| e.label.as_str())
    }

    /// Label of the step `redo` would reapply.
    pub fn redo_label(&self) -> Option<&str> {
        self.stack
            .entries
            .get(self.stack.cursor)
            .map(|e| e.label.as_str())
    }

    /// True if `undo` would do something.
    pub fn can_undo(&self) -> bool {
        self.stack.cursor > 0
    }

    /// True if `redo` would do something.
    pub fn can_redo(&self) -> bool {
        self.stack.cursor < self.stack.entries.len()
    }

    /// Forgets all history, keeping the current state.
    pub fn clear_history(&mut self) {
        self.stack.entries.clear();
        self.stack.cursor = 0;
        self.base_seq = self.next_seq;
        self.next_seq += 1;
        self.saved_seq = None;
    }

    /// Records that the current state was saved.
    pub fn mark_saved(&mut self) {
        self.saved_seq = Some(self.current_seq());
    }

    /// Forgets the saved marker, so the state counts as unsaved (a recovered document).
    pub fn mark_dirty(&mut self) {
        self.saved_seq = None;
    }

    /// True if the current state differs from the last saved one (or none was saved).
    pub fn is_dirty(&self) -> bool {
        self.saved_seq != Some(self.current_seq())
    }

    fn current_seq(&self) -> u64 {
        match self.stack.cursor.checked_sub(1) {
            Some(i) => self.stack.entries[i].seq,
            None => self.base_seq,
        }
    }

    /// Pushes `work` as the new state unless it equals the current one by revision.
    fn publish(&mut self, label: String, work: Design, commands: Vec<Command>) -> Revision {
        self.publish_as(label, work, commands, true)
    }

    /// [`Self::publish`]; `replayable` is false when `commands` cannot rebuild `work` from the
    /// design alone (see [`HistoryObserver::on_opaque_commit`]).
    fn publish_as(
        &mut self,
        label: String,
        work: Design,
        commands: Vec<Command>,
        replayable: bool,
    ) -> Revision {
        if work.revision() == self.current.revision() {
            return self.revision;
        }
        let before = Arc::clone(&self.current);
        let after = Arc::new(work);
        if let Some(o) = &self.observer.0 {
            if replayable {
                o.on_commit(&label, &commands);
            } else {
                o.on_opaque_commit(&label, &commands);
            }
        }
        let st = &mut self.stack;
        st.entries.truncate(st.cursor);
        st.entries.push_back(UndoEntry {
            label,
            before,
            after: Arc::clone(&after),
            commands,
            seq: self.next_seq,
        });
        self.next_seq += 1;
        st.cursor = st.entries.len();
        self.current = after;
        self.revision = self.revision.next();
        self.enforce_limit();
        self.revision
    }

    fn enforce_limit(&mut self) {
        let st = &mut self.stack;
        while st.entries.len() > st.limit {
            if let Some(e) = st.entries.pop_front() {
                self.base_seq = e.seq;
            }
            st.cursor = st.cursor.saturating_sub(1);
        }
    }
}

/// Several commands applied as one undo entry. Edits go to a private copy; nothing is
/// visible in the [`History`] until [`Transaction::commit`].
#[derive(Debug)]
pub struct Transaction<'h> {
    h: &'h mut History,
    label: String,
    work: Design,
    commands: Vec<Command>,
}

impl Transaction<'_> {
    /// Applies a command to the working copy. A failing command changes nothing and leaves
    /// the transaction usable; return the error (or call [`Transaction::rollback`]) to
    /// abandon it. The outcome's revision is the one `commit` will publish.
    pub fn apply(&mut self, cmd: Command) -> Result<CommandOutcome, CommandError> {
        let created = apply_command(&mut self.work, cmd.clone())?;
        self.commands.push(cmd);
        Ok(CommandOutcome {
            created,
            revision: self.h.revision.next(),
        })
    }

    /// The working design, including the commands applied so far.
    pub fn design(&self) -> &Design {
        &self.work
    }

    /// Publishes everything as one undo entry. A transaction that changed nothing adds no
    /// entry. Returns the new revision.
    pub fn commit(self) -> Revision {
        let Transaction {
            h,
            label,
            work,
            commands,
        } = self;
        h.publish(label, work, commands)
    }

    /// Discards everything (same as dropping it).
    pub fn rollback(self) {}
}
