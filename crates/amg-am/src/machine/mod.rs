//! Machine profiles (S16 AM-01): [`Machine`], [`BuildVolume`], [`MachineLibrary`] and their
//! TOML format.
//!
//! Built-in profiles are embedded TOML (`data/machines/*.toml`); user profiles are
//! `<config>/machines/*.toml`, a directory the UI passes in (this crate never hard-codes app
//! paths). TOML goes through the `toml` crate with serde: machine files are tiny, so the
//! custom fast-path reader of `amg-materials` is not needed here.

mod library;
mod volume;

use std::path::PathBuf;

use serde::{Deserialize, Serialize};

pub use library::{LoadWarning, MachineLibrary, machine_from_toml, machine_to_toml};
pub use volume::{BedShape, BuildVolume};

use crate::AmError;

/// Additive-manufacturing technology.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Technology {
    /// Fused filament fabrication.
    Fff,
    /// Masked stereolithography (LCD resin).
    Msla,
    /// Laser stereolithography.
    Sla,
    /// Selective laser sintering (polymer powder).
    Sls,
    /// Metal powder bed fusion.
    Mpbf,
}

impl Technology {
    /// Parts print directly on the plate (FFF).
    pub fn is_on_plate(self) -> bool {
        matches!(self, Technology::Fff)
    }

    /// Resin technologies (MSLA, SLA).
    pub fn is_resin(self) -> bool {
        matches!(self, Technology::Msla | Technology::Sla)
    }
}

/// FFF process parameters.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FffParams {
    /// Nozzle diameter per physical extruder, mm.
    pub nozzle_diameters: Vec<f64>,
    /// Filament slots, at least the nozzle count (MMU / KCM: 4 slots, 1 nozzle).
    pub filament_slots: u8,
    /// Default layer height, mm.
    pub layer_height: f64,
    /// First layer height, mm.
    pub first_layer_height: f64,
    /// Allowed layer height range (min, max), mm.
    pub layer_height_range: (f64, f64),
}

/// Resin (MSLA/SLA) process parameters.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResinParams {
    /// LCD resolution in pixels (x, y).
    pub resolution: [u32; 2],
    /// Pixel pitch (x, y), mm.
    pub pixel_size: amg_core::glam::DVec2,
    /// Default layer height, mm.
    pub layer_height: f64,
    /// Default elevation of a part above the plate (raft / supports base), mm.
    pub default_elevation: f64,
}

/// Technology-specific process parameters.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ProcessParams {
    /// FFF.
    Fff(FffParams),
    /// Resin.
    Resin(ResinParams),
    /// Powder bed.
    Powder {
        /// Powder layer height, mm.
        layer_height: f64,
    },
}

/// Slicer a machine hands off to by default.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SlicerKind {
    /// OrcaSlicer.
    OrcaSlicer,
    /// PrusaSlicer.
    PrusaSlicer,
    /// Bambu Studio.
    BambuStudio,
    /// Chitubox.
    Chitubox,
    /// A user-configured slicer.
    Custom,
}

/// Stable library key: `builtin:<vendor>/<model>`, `user:<file stem>`, `orca:<preset name>`,
/// `chitubox:<machinetype>` or `prusa:<preset name>`.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct MachineKey(pub String);

/// Where a machine profile came from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum MachineSource {
    /// Embedded in the application.
    Builtin,
    /// A user `*.toml` file.
    User {
        /// The file.
        path: PathBuf,
    },
    /// Imported from an Orca preset (AM-10).
    Orca {
        /// Preset name.
        preset: String,
        /// Preset file.
        file: PathBuf,
        /// `inherits` chain, child to root.
        chain: Vec<String>,
    },
    /// Imported from a Chitubox `.cfgx` (AM-10).
    Chitubox {
        /// The file.
        file: PathBuf,
    },
    /// Imported from a PrusaSlicer INI (AM-10).
    Prusa {
        /// Preset name.
        preset: String,
        /// The file.
        file: PathBuf,
    },
}

/// A printer: technology, build volume, process defaults and hand-off preference.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Machine {
    /// Library key.
    pub key: MachineKey,
    /// Display name.
    pub name: String,
    /// Manufacturer.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vendor: Option<String>,
    /// Model.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
    /// Technology.
    pub technology: Technology,
    /// Build volume.
    pub volume: BuildVolume,
    /// Process parameters; the variant matches `technology`.
    pub process: ProcessParams,
    /// Smallest printable feature, mm (FFF default: nozzle; resin: 2 px).
    pub min_feature: f64,
    /// Default critical overhang angle, rad (45 degrees default).
    pub default_critical_angle: f64,
    /// Preferred hand-off target.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub slicer: Option<SlicerKind>,
    /// Origin of the profile.
    pub source: MachineSource,
}

fn pos(name: &str, v: f64) -> Result<(), String> {
    if v.is_finite() && v > 0.0 {
        Ok(())
    } else {
        Err(format!("`{name}` must be a positive number, got {v}"))
    }
}

impl Machine {
    /// Half the first-layer height (FFF) or layer height (resin); 0.1 mm otherwise.
    pub fn default_contact_tolerance(&self) -> f64 {
        match &self.process {
            ProcessParams::Fff(f) => f.first_layer_height * 0.5,
            ProcessParams::Resin(r) => r.layer_height * 0.5,
            ProcessParams::Powder { .. } => 0.1,
        }
    }

    /// Checks sizes, bed points, slot counts and technology/process agreement. Reasons name
    /// the offending field.
    pub fn validate(&self) -> Result<(), AmError> {
        self.validate_inner().map_err(|reason| AmError::Machine {
            key: self.key.0.clone(),
            reason,
        })
    }

    fn validate_inner(&self) -> Result<(), String> {
        if self.key.0.trim().is_empty() {
            return Err("`key` must not be empty".into());
        }
        if self.name.trim().is_empty() {
            return Err("`name` must not be empty".into());
        }
        self.volume.validate()?;
        pos("min_feature", self.min_feature)?;
        let a = self.default_critical_angle;
        if !(a.is_finite() && a > 0.0 && a <= std::f64::consts::FRAC_PI_2) {
            return Err(format!(
                "`default_critical_angle` must be in (0, pi/2] rad, got {a}"
            ));
        }
        match (&self.process, self.technology) {
            (ProcessParams::Fff(f), Technology::Fff) => {
                if f.nozzle_diameters.is_empty() {
                    return Err("`process.nozzle_diameters` must not be empty".into());
                }
                for d in &f.nozzle_diameters {
                    pos("process.nozzle_diameters", *d)?;
                }
                if f.filament_slots == 0 || usize::from(f.filament_slots) < f.nozzle_diameters.len()
                {
                    return Err(format!(
                        "`process.filament_slots` ({}) must be >= 1 and >= the nozzle count ({})",
                        f.filament_slots,
                        f.nozzle_diameters.len()
                    ));
                }
                pos("process.layer_height", f.layer_height)?;
                pos("process.first_layer_height", f.first_layer_height)?;
                pos("process.layer_height_range", f.layer_height_range.0)?;
                pos("process.layer_height_range", f.layer_height_range.1)?;
                if f.layer_height_range.0 > f.layer_height_range.1 {
                    return Err("`process.layer_height_range` must be (min, max)".into());
                }
                Ok(())
            }
            (ProcessParams::Resin(r), Technology::Msla | Technology::Sla) => {
                if r.resolution.contains(&0) {
                    return Err("`process.resolution` must be non-zero".into());
                }
                pos("process.pixel_size", r.pixel_size.x)?;
                pos("process.pixel_size", r.pixel_size.y)?;
                pos("process.layer_height", r.layer_height)?;
                if !(r.default_elevation.is_finite() && r.default_elevation >= 0.0) {
                    return Err(format!(
                        "`process.default_elevation` must be >= 0, got {}",
                        r.default_elevation
                    ));
                }
                Ok(())
            }
            (ProcessParams::Powder { layer_height }, Technology::Sls | Technology::Mpbf) => {
                pos("process.layer_height", *layer_height)
            }
            (p, t) => Err(format!(
                "`process.kind` ({}) does not match `technology` ({t:?})",
                match p {
                    ProcessParams::Fff(_) => "fff",
                    ProcessParams::Resin(_) => "resin",
                    ProcessParams::Powder { .. } => "powder",
                }
            )),
        }
    }
}
