//! `ui.visibility`: browser visibility of things S01 has no visibility field for (sketches,
//! construction geometry, the origin).
//!
//! One `ExtRecord` per design, kind [`VISIBILITY_KIND`], `data = {"hidden": [node keys]}`
//! (sorted, unique). Written through `AddExt` (first hidden item) / `SetExt` (later; `None`
//! once nothing is hidden), so it is undoable and saved with the document. The overlay
//! renderers (RND-07, UI-09) skip hidden sketches and construction geometry through
//! [`is_hidden`].

use std::collections::BTreeSet;

use amg_doc::{Command, ConstructionId, Design, ExtKey, ExtRecord, SketchId};
use serde::{Deserialize, Serialize};

/// Extension kind of the record.
pub const VISIBILITY_KIND: &str = "ui.visibility";
/// Schema version of the record's `data`.
pub const VISIBILITY_VERSION: u32 = 1;

#[derive(Debug, Default, Serialize, Deserialize)]
struct Data {
    hidden: BTreeSet<String>,
}

/// Key of the design's visibility record, if it has one.
pub fn visibility_key(design: &Design) -> Option<ExtKey> {
    design
        .exts()
        .find(|(k, _)| k.kind == VISIBILITY_KIND)
        .map(|(k, _)| k.clone())
}

/// The hidden item keys (`"sketch/<feature>"`, `"construction/<feature>"`, `"origin"`).
pub fn hidden(design: &Design) -> BTreeSet<String> {
    let Some(key) = visibility_key(design) else {
        return BTreeSet::new();
    };
    design
        .ext(&key)
        .and_then(|r| serde_json::from_value::<Data>(r.data.clone()).ok())
        .map(|d| d.hidden)
        .unwrap_or_default()
}

/// The visibility key of a sketch.
pub fn sketch_key(s: SketchId) -> String {
    format!("sketch/{}", s.0.0)
}

/// The visibility key of construction geometry.
pub fn construction_key(c: ConstructionId) -> String {
    format!("construction/{}", c.0.0)
}

/// The visibility key of the origin.
pub const ORIGIN_KEY: &str = "origin";

/// Whether the item with visibility key `item` is hidden.
pub fn is_hidden(design: &Design, item: &str) -> bool {
    hidden(design).contains(item)
}

/// The command that shows or hides `items` (`None` if nothing changes).
pub fn set_hidden_command(design: &Design, items: &[String], hide: bool) -> Option<Command> {
    let mut set = hidden(design);
    let before = set.clone();
    for i in items {
        if hide {
            set.insert(i.clone());
        } else {
            set.remove(i);
        }
    }
    if set == before {
        return None;
    }
    let key = visibility_key(design);
    if set.is_empty() {
        return key.map(|key| Command::SetExt { key, record: None });
    }
    let record = ExtRecord {
        kind: VISIBILITY_KIND.to_owned(),
        version: VISIBILITY_VERSION,
        data: serde_json::to_value(Data { hidden: set }).ok()?,
        blobs: Vec::new(),
    };
    Some(match key {
        Some(key) => Command::SetExt {
            key,
            record: Some(record),
        },
        None => Command::AddExt { record },
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "test code")]
    use super::*;
    use amg_doc::{Document, FeatureId};

    #[test]
    fn hide_show_round_trip_adds_sets_and_removes_the_record() {
        let mut doc = Document::new("v");
        let s = sketch_key(SketchId(FeatureId(7)));
        let c = construction_key(ConstructionId(FeatureId(9)));
        let cmd = set_hidden_command(doc.design(), std::slice::from_ref(&s), true).unwrap();
        assert!(matches!(cmd, Command::AddExt { .. }));
        doc.apply(cmd).unwrap();
        assert!(is_hidden(doc.design(), &s));
        assert!(set_hidden_command(doc.design(), std::slice::from_ref(&s), true).is_none());
        let cmd = set_hidden_command(doc.design(), std::slice::from_ref(&c), true).unwrap();
        assert!(matches!(
            cmd,
            Command::SetExt {
                record: Some(_),
                ..
            }
        ));
        doc.apply(cmd).unwrap();
        assert_eq!(hidden(doc.design()).len(), 2);
        let cmd = set_hidden_command(doc.design(), &[s.clone(), c.clone()], false).unwrap();
        assert!(matches!(cmd, Command::SetExt { record: None, .. }));
        doc.apply(cmd).unwrap();
        assert!(visibility_key(doc.design()).is_none());
        assert!(!is_hidden(doc.design(), &s));
    }
}
