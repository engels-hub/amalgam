//! Selection sets (spec §8): named lists of [`EntityRef`]s kept in one extension record of
//! kind [`SELECTION_SETS_KIND`] per design.
//!
//! The record is written with `DocCommand::AddExt` (first set) or `DocCommand::SetExt` (later
//! changes), so saving a set is one undo entry and the sets are saved with the file. Restoring
//! maps every reference back to a displayed target ([`super::target_for_entity`]); references
//! that no longer resolve are skipped and counted.

use amg_doc::{Command, Design, EntityRef, ExtKey, ExtRecord};
use serde::{Deserialize, Serialize};

use crate::app::{AmalgamApp, AppCx, CommitBatch};
use crate::notify::Notification;

/// Extension kind of the selection-set record.
pub const SELECTION_SETS_KIND: &str = "ui.selection_sets";
/// Schema version of the record's `data`.
pub const SELECTION_SETS_VERSION: u32 = 1;

/// One named selection set.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SelectionSet {
    /// Unique name within the design.
    pub name: String,
    /// The references, in selection order.
    pub items: Vec<EntityRef>,
}

#[derive(Debug, Default, Serialize, Deserialize)]
struct SetsData {
    sets: Vec<SelectionSet>,
}

/// The key of the design's selection-set record (the first of its kind).
pub fn sets_key(design: &Design) -> Option<ExtKey> {
    design
        .exts()
        .find(|(k, _)| k.kind == SELECTION_SETS_KIND)
        .map(|(k, _)| k.clone())
}

/// The design's selection sets, in creation order. A record of an unknown version or shape
/// reads as no sets.
pub fn selection_sets(design: &Design) -> Vec<SelectionSet> {
    let Some(key) = sets_key(design) else {
        return Vec::new();
    };
    design
        .ext(&key)
        .filter(|r| r.version == SELECTION_SETS_VERSION)
        .and_then(|r| serde_json::from_value::<SetsData>(r.data.clone()).ok())
        .map(|d| d.sets)
        .unwrap_or_default()
}

/// The set called `name`.
pub fn selection_set(design: &Design, name: &str) -> Option<SelectionSet> {
    selection_sets(design).into_iter().find(|s| s.name == name)
}

/// The first free default name: "Selection Set 1", "Selection Set 2", …
pub fn next_set_name(design: &Design) -> String {
    let sets = selection_sets(design);
    (1..)
        .map(|n| format!("Selection Set {n}"))
        .find(|n| !sets.iter().any(|s| s.name == *n))
        .unwrap_or_default()
}

fn record(sets: Vec<SelectionSet>) -> Result<ExtRecord, serde_json::Error> {
    Ok(ExtRecord {
        kind: SELECTION_SETS_KIND.to_owned(),
        version: SELECTION_SETS_VERSION,
        data: serde_json::to_value(SetsData { sets })?,
        blobs: Vec::new(),
    })
}

fn write(design: &Design, sets: Vec<SelectionSet>) -> Result<Command, serde_json::Error> {
    let record = record(sets)?;
    Ok(match sets_key(design) {
        Some(key) => Command::SetExt {
            key,
            record: Some(record),
        },
        None => Command::AddExt { record },
    })
}

/// The command that stores `set` (replacing a set of the same name).
///
/// # Errors
/// If the references cannot be serialized (they always can; `serde_json` refuses only
/// non-finite floats, and fingerprints of finite geometry are finite).
pub fn save_set_command(design: &Design, set: SelectionSet) -> Result<Command, serde_json::Error> {
    let mut sets = selection_sets(design);
    match sets.iter_mut().find(|s| s.name == set.name) {
        Some(s) => *s = set,
        None => sets.push(set),
    }
    write(design, sets)
}

/// The command that removes the set called `name` (`None` if there is none).
///
/// # Errors
/// As [`save_set_command`].
pub fn remove_set_command(
    design: &Design,
    name: &str,
) -> Option<Result<Command, serde_json::Error>> {
    let mut sets = selection_sets(design);
    let before = sets.len();
    sets.retain(|s| s.name != name);
    if sets.len() == before {
        return None;
    }
    if sets.is_empty() {
        return sets_key(design).map(|key| Ok(Command::SetExt { key, record: None }));
    }
    Some(write(design, sets))
}

/// Saves the current selection as a set called `name` (default: [`next_set_name`]). One undo
/// entry, applied at the end of the frame. False if nothing is selected.
pub fn save_selection(cx: &mut AppCx<'_>, name: Option<String>) -> bool {
    let cmd = {
        let env = cx.env();
        if env.selection.is_empty() {
            return false;
        }
        let set = SelectionSet {
            name: name.unwrap_or_else(|| next_set_name(env.design)),
            items: env
                .selection
                .items()
                .iter()
                .map(|i| i.entity.clone())
                .collect(),
        };
        save_set_command(env.design, set)
    };
    match cmd {
        Ok(cmd) => {
            cx.submit(CommitBatch {
                label: "Save selection set".to_owned(),
                commands: vec![cmd],
            });
            true
        }
        Err(e) => {
            cx.notify(Notification::error(
                "Cannot save the selection set",
                e.to_string(),
            ));
            false
        }
    }
}

impl AmalgamApp {
    /// Replaces the active document's selection with the set called `name`. References that
    /// no longer resolve (or are not displayed) are skipped. Returns how many were skipped, or
    /// `None` if there is no such set (UI-04).
    pub fn restore_selection_set(&mut self, name: &str) -> Option<usize> {
        let doc = &mut self.docs[self.active];
        let set = selection_set(doc.document.design(), name)?;
        let total = set.items.len();
        let mut items = Vec::with_capacity(set.items.len());
        for entity in set.items {
            if let Some(target) = super::target_for_entity(
                doc.document.design(),
                doc.document.evaluated(),
                &self.scene,
                &entity,
            ) {
                items.push(super::SelItem { target, entity });
            }
        }
        let missing = total - items.len();
        doc.selection.set(items);
        if let Some(ctx) = &self.ctx {
            ctx.request_repaint();
        }
        Some(missing)
    }
}
