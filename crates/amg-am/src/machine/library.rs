//! The machine library and the TOML (de)serialisation.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use super::{
    BuildVolume, Machine, MachineKey, MachineSource, ProcessParams, SlicerKind, Technology,
};
use crate::AmError;

/// A user profile that could not be loaded; the library still loads the rest.
#[derive(Clone, Debug)]
pub struct LoadWarning {
    /// The offending file.
    pub file: PathBuf,
    /// What is wrong, naming the field.
    pub message: String,
}

const BUILTIN: [(&str, &str); 5] = [
    (
        "creality-ender-3-v3-ke.toml",
        include_str!("../../data/machines/creality-ender-3-v3-ke.toml"),
    ),
    (
        "creality-ender-3-v3-ke-kcm.toml",
        include_str!("../../data/machines/creality-ender-3-v3-ke-kcm.toml"),
    ),
    (
        "elegoo-mars-3-pro.toml",
        include_str!("../../data/machines/elegoo-mars-3-pro.toml"),
    ),
    (
        "generic-fff-220.toml",
        include_str!("../../data/machines/generic-fff-220.toml"),
    ),
    (
        "generic-msla.toml",
        include_str!("../../data/machines/generic-msla.toml"),
    ),
];

/// Machines by key, in deterministic (key) order.
#[derive(Clone, Debug, Default)]
pub struct MachineLibrary {
    machines: BTreeMap<MachineKey, Machine>,
}

impl MachineLibrary {
    /// The built-in profiles (embedded TOML).
    pub fn builtin() -> Self {
        let mut lib = MachineLibrary::default();
        for (_file, src) in BUILTIN {
            // The embedded files are covered by a unit test; a broken one is skipped.
            if let Ok(m) = machine_from_toml(src) {
                let _ = lib.insert(m);
            }
        }
        lib
    }

    /// Loads every `*.toml` in `dir` as a user profile (key `user:<file stem>`). Files that
    /// do not parse or validate become warnings; a missing directory is not an error.
    pub fn load_user_dir(&mut self, dir: &Path) -> Vec<LoadWarning> {
        let mut warnings = Vec::new();
        let entries = match std::fs::read_dir(dir) {
            Ok(e) => e,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return warnings,
            Err(e) => {
                warnings.push(LoadWarning {
                    file: dir.to_owned(),
                    message: format!("cannot read directory: {e}"),
                });
                return warnings;
            }
        };
        let mut files: Vec<PathBuf> = entries
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| {
                p.extension()
                    .is_some_and(|x| x.eq_ignore_ascii_case("toml"))
            })
            .collect();
        files.sort();
        for file in files {
            let stem = file
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_default();
            let loaded = std::fs::read_to_string(&file)
                .map_err(|e| format!("cannot read file: {e}"))
                .and_then(|s| machine_from_toml(&s).map_err(|e| e.to_string()))
                .map(|mut m| {
                    m.key = MachineKey(format!("user:{stem}"));
                    m.source = MachineSource::User { path: file.clone() };
                    m
                })
                .and_then(|m| self.insert(m).map(|_| ()).map_err(|e| e.to_string()));
            if let Err(message) = loaded {
                warnings.push(LoadWarning { file, message });
            }
        }
        warnings
    }

    /// Validates and inserts `m`; returns the machine it replaced.
    pub fn insert(&mut self, m: Machine) -> Result<Option<Machine>, AmError> {
        m.validate()?;
        Ok(self.machines.insert(m.key.clone(), m))
    }

    /// The machine with `key`.
    pub fn get(&self, key: &MachineKey) -> Option<&Machine> {
        self.machines.get(key)
    }

    /// All machines in key order.
    pub fn iter(&self) -> impl Iterator<Item = &Machine> + '_ {
        self.machines.values()
    }

    /// Number of machines.
    pub fn len(&self) -> usize {
        self.machines.len()
    }

    /// True if the library has no machines.
    pub fn is_empty(&self) -> bool {
        self.machines.is_empty()
    }

    /// Writes a user copy of `key` to `<dir>/<stem>.toml` (creating `dir`); returns the path.
    /// The copy has key `user:<stem>`, the stem being the key's own for user keys and a
    /// sanitised form of the key otherwise. Does not modify the library.
    pub fn save_user(&self, key: &MachineKey, dir: &Path) -> Result<PathBuf, AmError> {
        let m = self.get(key).ok_or_else(|| AmError::Machine {
            key: key.0.clone(),
            reason: "no such machine in the library".into(),
        })?;
        let raw = key.0.strip_prefix("user:").unwrap_or(&key.0);
        let stem: String = raw
            .chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                    c
                } else {
                    '-'
                }
            })
            .collect();
        let path = dir.join(format!("{stem}.toml"));
        let mut copy = m.clone();
        copy.key = MachineKey(format!("user:{stem}"));
        copy.source = MachineSource::User { path: path.clone() };
        std::fs::create_dir_all(dir).map_err(|e| AmError::Profile {
            file: dir.to_owned(),
            reason: e.to_string(),
        })?;
        std::fs::write(&path, machine_to_toml(&copy)).map_err(|e| AmError::Profile {
            file: path.clone(),
            reason: e.to_string(),
        })?;
        Ok(path)
    }
}

/// The on-disk document. `key` and `source` are optional in hand-written files.
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct MachineDoc {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    key: Option<MachineKey>,
    name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    vendor: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    model: Option<String>,
    technology: Technology,
    min_feature: f64,
    default_critical_angle: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    slicer: Option<SlicerKind>,
    volume: BuildVolume,
    process: ProcessParams,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    source: Option<MachineSource>,
}

/// Parses and validates a machine from TOML. A missing `key` becomes `user:<slug of name>` and
/// a missing `source` a [`MachineSource::User`] with an empty path.
pub fn machine_from_toml(s: &str) -> Result<Machine, AmError> {
    let doc: MachineDoc = toml::from_str(s).map_err(|e| AmError::Machine {
        key: "<toml>".into(),
        reason: e.to_string(),
    })?;
    let key = doc.key.unwrap_or_else(|| {
        let slug: String = doc
            .name
            .chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() {
                    c.to_ascii_lowercase()
                } else {
                    '-'
                }
            })
            .collect();
        MachineKey(format!("user:{slug}"))
    });
    let m = Machine {
        key,
        name: doc.name,
        vendor: doc.vendor,
        model: doc.model,
        technology: doc.technology,
        volume: doc.volume,
        process: doc.process,
        min_feature: doc.min_feature,
        default_critical_angle: doc.default_critical_angle,
        slicer: doc.slicer,
        source: doc.source.unwrap_or(MachineSource::User {
            path: PathBuf::new(),
        }),
    };
    m.validate()?;
    Ok(m)
}

/// Serialises `m` to TOML (lossless: f64 values round-trip exactly).
pub fn machine_to_toml(m: &Machine) -> String {
    let doc = MachineDoc {
        key: Some(m.key.clone()),
        name: m.name.clone(),
        vendor: m.vendor.clone(),
        model: m.model.clone(),
        technology: m.technology,
        min_feature: m.min_feature,
        default_critical_angle: m.default_critical_angle,
        slicer: m.slicer,
        volume: m.volume.clone(),
        process: m.process.clone(),
        source: Some(m.source.clone()),
    };
    toml::to_string(&doc).unwrap_or_else(|e| format!("# cannot serialise machine: {e}\n"))
}
