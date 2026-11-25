//! Remembered import and export choices, per format (S12 §4, IO-15).
//!
//! One JSON file (`io-prefs.json` in the user's config folder). The store is process-wide
//! because commands and drop handlers are plain functions; tests point it at a temporary file
//! with [`set_prefs_path`]. Reads and writes never fail the UI: a missing or damaged file
//! gives the defaults, a failed write is logged.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Mutex;

use amg_core::units::LengthUnit;
use amg_features::io::ExportRefinement;
use amg_io::{Format, SlicerFlavor, UpAxis};
use serde::{Deserialize, Serialize};

/// What the import dialog remembers for one format.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImportPref {
    /// The unit last chosen; `None` = use the unit the file declares.
    pub unit: Option<LengthUnit>,
    /// Up axis.
    pub up_axis: UpAxis,
    /// One body for the whole file.
    pub merge: bool,
}

impl Default for ImportPref {
    fn default() -> Self {
        ImportPref {
            unit: None,
            up_axis: UpAxis::Z,
            merge: false,
        }
    }
}

/// What the export dialog remembers for one format.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExportPref {
    /// Unit of the written numbers.
    pub unit: LengthUnit,
    /// 3MF slicer flavour.
    pub flavor: SlicerFlavor,
    /// Selected bodies only (else every visible body).
    pub selection_only: bool,
    /// STL: ASCII instead of binary.
    pub ascii: bool,
    /// PLY: binary instead of ASCII.
    pub binary: bool,
    /// OBJ: write the `.mtl`.
    pub mtl: bool,
    /// Tessellation preset of the solid bodies (IO-09).
    #[serde(default)]
    pub refinement: ExportRefinement,
}

impl Default for ExportPref {
    fn default() -> Self {
        ExportPref {
            unit: LengthUnit::Millimetre,
            flavor: SlicerFlavor::Generic,
            selection_only: false,
            ascii: false,
            binary: true,
            mtl: true,
            refinement: ExportRefinement::Medium,
        }
    }
}

/// Everything remembered.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct IoPrefs {
    /// Import choices by format.
    #[serde(default)]
    pub import: BTreeMap<Format, ImportPref>,
    /// Export choices by format.
    #[serde(default)]
    pub export: BTreeMap<Format, ExportPref>,
    /// The format of the last export.
    #[serde(default)]
    pub last_export_format: Option<Format>,
    /// Folder of the last import or export.
    #[serde(default)]
    pub last_dir: Option<PathBuf>,
    /// Build volume (mm) used for the bed-fit line when the design has no print setup.
    #[serde(default = "default_bed")]
    pub bed_mm: [f64; 3],
}

fn default_bed() -> [f64; 3] {
    [256.0, 256.0, 256.0]
}

impl Default for IoPrefs {
    fn default() -> Self {
        IoPrefs {
            import: BTreeMap::new(),
            export: BTreeMap::new(),
            last_export_format: None,
            last_dir: None,
            bed_mm: default_bed(),
        }
    }
}

impl IoPrefs {
    /// The import choices of `format`.
    pub fn import_for(&self, format: Format) -> ImportPref {
        self.import.get(&format).copied().unwrap_or_default()
    }

    /// The export choices of `format`.
    pub fn export_for(&self, format: Format) -> ExportPref {
        self.export.get(&format).copied().unwrap_or_default()
    }
}

struct Store {
    path: Option<PathBuf>,
    loaded: Option<IoPrefs>,
}

static STORE: Mutex<Store> = Mutex::new(Store {
    path: None,
    loaded: None,
});

fn store() -> std::sync::MutexGuard<'static, Store> {
    STORE
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// `<config dir>/amalgam/io-prefs.json`.
pub fn default_prefs_path() -> Option<PathBuf> {
    directories::BaseDirs::new().map(|d| d.config_dir().join("amalgam").join("io-prefs.json"))
}

/// Where the choices are kept (`None`: in memory only). Forgets what was loaded.
pub fn set_prefs_path(path: Option<PathBuf>) {
    let mut s = store();
    s.path = path;
    s.loaded = None;
}

fn read(path: &std::path::Path) -> IoPrefs {
    match std::fs::read(path) {
        Ok(bytes) => serde_json::from_slice(&bytes).unwrap_or_else(|e| {
            tracing::warn!(
                "{}: {e}; using the default import/export choices",
                path.display()
            );
            IoPrefs::default()
        }),
        Err(_) => IoPrefs::default(),
    }
}

/// The remembered choices (loaded on first use).
pub fn load() -> IoPrefs {
    let mut s = store();
    if s.loaded.is_none() {
        let path = s.path.clone().or_else(default_prefs_path);
        s.loaded = Some(path.map(|p| read(&p)).unwrap_or_default());
    }
    s.loaded.clone().unwrap_or_default()
}

/// Changes the remembered choices and writes them.
pub fn update(f: impl FnOnce(&mut IoPrefs)) {
    let mut prefs = load();
    f(&mut prefs);
    let mut s = store();
    let path = s.path.clone().or_else(default_prefs_path);
    if let Some(path) = path {
        let write = || -> std::io::Result<()> {
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir)?;
            }
            let tmp = path.with_extension("json.tmp");
            let json = serde_json::to_vec_pretty(&prefs).map_err(std::io::Error::other)?;
            std::fs::write(&tmp, json)?;
            std::fs::rename(&tmp, &path)
        };
        if let Err(e) = write() {
            tracing::warn!("cannot save {}: {e}", path.display());
        }
    }
    s.loaded = Some(prefs);
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact literals")]
mod tests {
    use super::*;

    #[test]
    fn defaults_have_a_bed_and_round_trip() {
        let p = IoPrefs::default();
        assert_eq!(p.bed_mm, [256.0, 256.0, 256.0]);
        let mut q = p.clone();
        q.import.insert(
            Format::Stl,
            ImportPref {
                unit: Some(LengthUnit::Inch),
                up_axis: UpAxis::Y,
                merge: true,
            },
        );
        q.export.insert(
            Format::ThreeMf,
            ExportPref {
                flavor: SlicerFlavor::Orca,
                refinement: ExportRefinement::Custom {
                    surface_deviation_mm: 0.05,
                    normal_deviation_rad: 0.2,
                },
                ..ExportPref::default()
            },
        );
        let json = serde_json::to_string(&q).unwrap();
        let back: IoPrefs = serde_json::from_str(&json).unwrap();
        assert_eq!(back, q);
        // A file written before a field existed still loads.
        let old: IoPrefs = serde_json::from_str("{}").unwrap();
        assert_eq!(old, p);
        // So does a format's entry from before the refinement preset (IO-09).
        let mut v = serde_json::to_value(ExportPref::default()).unwrap();
        v.as_object_mut().unwrap().remove("refinement");
        let before: ExportPref = serde_json::from_value(v).unwrap();
        assert_eq!(before.refinement, ExportRefinement::Medium);
    }
}
