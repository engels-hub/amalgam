//! Additive-manufacturing features (S16, lane AM): the print-setup extension record.
//!
//! A design holds any number of print setups as extension records `am.print_setup`
//! (`ExtKey { kind, id }`). The record is one JSON document, [`PrintSetup`]; its only mutation
//! path is whole-record replace through `Command::SetExt` ([`PrintSetup::command`]), so
//! undo/redo restores equal records. Setup creation uses `Command::AddExt`
//! ([`PrintSetup::add_command`]).
//!
//! `amg-doc` has no extension-registration table yet, so the version check and the migration
//! hook ([`migrate_print_setup`]) are applied by [`PrintSetup::from_record`] itself. Resolving
//! items to meshes and the hand-off glue (`resolve_plate`, `handoff_plate`) belong to AM-06.
//!
//! Call [`link`] from a binary or test so the module is linked in (registry convention).

use amg_am::AmError;
use amg_am::machine::Machine;
use amg_am::overhang::AnalysisSettings;
use amg_am::place::Placement;
use amg_doc::{BodyId, Command, Design, ExtKey, ExtRecord};
use serde::{Deserialize, Serialize};

/// Extension kind of a print setup.
pub const PRINT_SETUP_KIND: &str = "am.print_setup";
/// Current record version (2 after AM-04 adds cosmetic face groups).
pub const PRINT_SETUP_VERSION: u32 = 1;

/// Forces this module to be linked (registry `link()` convention).
pub fn link() {}

/// Id of a plate within a setup, allocated from `next_plate_id`, never reused.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct PlateId(pub u32);

/// Id of an item within a setup, allocated from `next_item_id`, never reused.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ItemId(pub u32);

/// A body placed on a plate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlateItem {
    /// Item id.
    pub id: ItemId,
    /// The body.
    pub body: BodyId,
    /// Where it sits.
    pub placement: Placement,
    /// Filament slot (multi-material), if assigned.
    pub filament: Option<u8>,
}

/// One build plate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Plate {
    /// Plate id.
    pub id: PlateId,
    /// Display name.
    pub name: String,
    /// Items in insertion order.
    pub items: Vec<PlateItem>,
}

/// A print setup: machine snapshot, analysis settings and plates.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PrintSetup {
    /// Display name.
    pub name: String,
    /// Snapshot of the machine at setup creation (the library may change later).
    pub machine: Machine,
    /// Overhang-analysis settings.
    pub settings: AnalysisSettings,
    /// Plates (at least one).
    pub plates: Vec<Plate>,
    /// The plate shown.
    pub active_plate: PlateId,
    /// Next plate id.
    pub next_plate_id: u32,
    /// Next item id.
    pub next_item_id: u32,
}

/// Errors of the print-setup record.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum AmFeatureError {
    /// The record does not decode.
    #[error("print setup record is invalid: {0}")]
    Decode(String),
    /// The record is newer than this build supports.
    #[error("print setup v{0} is newer than supported")]
    TooNew(u32),
    /// A machine or other AM error.
    #[error(transparent)]
    Am(#[from] AmError),
}

impl PrintSetup {
    /// A setup on `machine` with one empty plate named "Plate 1".
    pub fn new(name: &str, machine: Machine) -> Self {
        let settings = AnalysisSettings::for_machine(&machine);
        PrintSetup {
            name: name.to_owned(),
            machine,
            settings,
            plates: vec![Plate {
                id: PlateId(1),
                name: "Plate 1".to_owned(),
                items: Vec::new(),
            }],
            active_plate: PlateId(1),
            next_plate_id: 2,
            next_item_id: 1,
        }
    }

    /// Adds `body` to `plate` (the active plate if `plate` does not exist) and returns the new
    /// item's id.
    pub fn add_body(&mut self, plate: PlateId, body: BodyId, placement: Placement) -> ItemId {
        let id = ItemId(self.next_item_id);
        self.next_item_id += 1;
        let idx = self
            .plates
            .iter()
            .position(|p| p.id == plate)
            .or_else(|| self.plates.iter().position(|p| p.id == self.active_plate))
            .unwrap_or(0);
        if let Some(p) = self.plates.get_mut(idx) {
            p.items.push(PlateItem {
                id,
                body,
                placement,
                filament: None,
            });
        }
        id
    }

    /// Removes an item; returns whether it existed. Ids are not reused.
    pub fn remove_item(&mut self, id: ItemId) -> bool {
        for p in &mut self.plates {
            if let Some(i) = p.items.iter().position(|it| it.id == id) {
                p.items.remove(i);
                return true;
            }
        }
        false
    }

    /// Mutable access to an item.
    pub fn item_mut(&mut self, id: ItemId) -> Option<&mut PlateItem> {
        self.plates
            .iter_mut()
            .flat_map(|p| p.items.iter_mut())
            .find(|it| it.id == id)
    }

    /// Adds an empty plate and returns its id.
    pub fn add_plate(&mut self, name: &str) -> PlateId {
        let id = PlateId(self.next_plate_id);
        self.next_plate_id += 1;
        self.plates.push(Plate {
            id,
            name: name.to_owned(),
            items: Vec::new(),
        });
        id
    }

    /// The record (deterministic JSON: struct order, sorted maps).
    pub fn to_record(&self) -> ExtRecord {
        ExtRecord {
            kind: PRINT_SETUP_KIND.to_owned(),
            version: PRINT_SETUP_VERSION,
            // Plain data with finite numbers; the fallback only guards a broken Machine.
            data: serde_json::to_value(self).unwrap_or(serde_json::Value::Null),
            blobs: Vec::new(),
        }
    }

    /// Decodes a record. Version 0 and other malformed records give `Decode`, versions above
    /// [`PRINT_SETUP_VERSION`] give `TooNew`; older supported versions are migrated first.
    pub fn from_record(r: &ExtRecord) -> Result<Self, AmFeatureError> {
        if r.kind != PRINT_SETUP_KIND {
            return Err(AmFeatureError::Decode(format!(
                "record kind `{}` is not `{PRINT_SETUP_KIND}`",
                r.kind
            )));
        }
        if r.version > PRINT_SETUP_VERSION {
            return Err(AmFeatureError::TooNew(r.version));
        }
        if r.version == 0 {
            return Err(AmFeatureError::Decode("version 0 never existed".to_owned()));
        }
        let data = if r.version < PRINT_SETUP_VERSION {
            migrate_print_setup(r.version, r.data.clone())
                .map_err(|e| AmFeatureError::Decode(e.to_string()))?
        } else {
            r.data.clone()
        };
        let s: PrintSetup =
            serde_json::from_value(data).map_err(|e| AmFeatureError::Decode(e.to_string()))?;
        s.check()?;
        Ok(s)
    }

    fn check(&self) -> Result<(), AmFeatureError> {
        let bad = |m: &str| Err(AmFeatureError::Decode(m.to_owned()));
        if self.plates.is_empty() {
            return bad("a setup needs at least one plate");
        }
        if !self.plates.iter().any(|p| p.id == self.active_plate) {
            return bad("active_plate is not a plate of the setup");
        }
        let mut seen = std::collections::BTreeSet::new();
        for p in &self.plates {
            if p.id.0 >= self.next_plate_id {
                return bad("plate id is not below next_plate_id");
            }
            for it in &p.items {
                if it.id.0 >= self.next_item_id || !seen.insert(it.id) {
                    return bad("item id is duplicated or not below next_item_id");
                }
            }
        }
        let mut ids: Vec<_> = self.plates.iter().map(|p| p.id).collect();
        ids.sort();
        ids.dedup();
        if ids.len() != self.plates.len() {
            return bad("duplicate plate id");
        }
        self.machine.validate()?;
        Ok(())
    }

    /// The one mutation path: replace the whole record under `key`.
    pub fn command(&self, key: ExtKey) -> Command {
        Command::SetExt {
            key,
            record: Some(self.to_record()),
        }
    }

    /// Creates this setup as a new record; the key is the design's [`next_setup_key`].
    pub fn add_command(&self) -> Command {
        Command::AddExt {
            record: self.to_record(),
        }
    }
}

/// Deletes the setup under `key`.
pub fn delete_setup_command(key: ExtKey) -> Command {
    Command::SetExt { key, record: None }
}

/// The key the next `Command::AddExt` of a print setup will get. Ids come from the design's
/// monotonic counter, so a deleted setup's key is never handed out again.
pub fn next_setup_key(d: &Design) -> ExtKey {
    let mut scratch = d.clone();
    match scratch.new_ext_key(PRINT_SETUP_KIND) {
        Ok(k) => k,
        // Id space exhausted (2^53 ids): no valid key exists; hand out an invalid one that
        // `SetExt` rejects instead of panicking.
        Err(_) => ExtKey {
            kind: PRINT_SETUP_KIND.to_owned(),
            id: 0,
        },
    }
}

/// Every print setup of `d` in key order, each decoded or with its error.
pub fn setups(d: &Design) -> Vec<(ExtKey, Result<PrintSetup, AmFeatureError>)> {
    d.exts()
        .filter(|(k, _)| k.kind == PRINT_SETUP_KIND)
        .map(|(k, r)| (k.clone(), PrintSetup::from_record(r)))
        .collect()
}

/// Migrates a record's data from `from` to [`PRINT_SETUP_VERSION`]. Version 1 is current, so
/// there is nothing to do yet; AM-04 adds the v1→v2 step here.
pub fn migrate_print_setup(
    from: u32,
    v: serde_json::Value,
) -> Result<serde_json::Value, amg_doc::MigrationError> {
    match from {
        1 => Ok(v),
        _ => Err(amg_doc::MigrationError(format!(
            "no migration from print setup v{from}"
        ))),
    }
}
