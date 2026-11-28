//! Chitubox machine files: `<VENDOR>/<uuid>.cfgx`, JSON with the printer values under
//! `printerinfo.<name>.currentvalue` and the layer height in the current material's current
//! slice profile.

use std::path::{Path, PathBuf};

use amg_core::glam::DVec2;
use serde_json::Value;

use super::{Collector, ImportResult, files_with_extension, profile_error, read_text};
use crate::AmError;
use crate::machine::{
    BedShape, BuildVolume, Machine, MachineKey, MachineSource, ProcessParams, ResinParams,
    SlicerKind, Technology,
};

/// Index files of the same folder; they are YAML, not machines.
const LISTS: [&str; 2] = ["addMachineList", "HistoryList"];

/// Chitubox stores its numbers as `f32` (143.42999267578125 is 143.43): such a value becomes
/// the shortest decimal that gives the same `f32`; any other value is kept.
fn from_f32(x: f64) -> f64 {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "compared with the original below"
    )]
    let narrow = x as f32;
    if (f64::from(narrow) - x).abs() > x.abs() * 1e-12 {
        return x;
    }
    narrow.to_string().parse().unwrap_or(x)
}

fn machine(file: &Path, doc: &Value) -> Result<Machine, String> {
    let info = doc.get("printerinfo").ok_or("no `printerinfo` object")?;
    let value = |k: &str| info.get(k).and_then(|v| v.get("currentvalue"));
    let size = |k: &str| {
        value(k)
            .and_then(Value::as_f64)
            .filter(|x| x.is_finite() && *x > 0.0)
            .map(from_f32)
            .ok_or(format!("`printerinfo.{k}` is missing or not positive"))
    };
    let pixels = |k: &str| {
        value(k)
            .and_then(Value::as_u64)
            .and_then(|n| u32::try_from(n).ok())
            .filter(|n| *n > 0)
            .ok_or(format!("`printerinfo.{k}` is missing or not positive"))
    };
    let string = |k: &str| {
        value(k)
            .and_then(Value::as_str)
            .map(str::trim)
            .filter(|s| !s.is_empty())
    };
    let machine_type = string("machinetype").ok_or("`printerinfo.machinetype` is missing")?;
    let bed = DVec2::new(size("machinewidth")?, size("machinedepth")?);
    let resolution = [pixels("resolutionx")?, pixels("resolutiony")?];
    let pixel_size = bed / DVec2::new(f64::from(resolution[0]), f64::from(resolution[1]));
    let index = |v: &Value, k: &str| {
        v.get(k)
            .and_then(Value::as_u64)
            .and_then(|n| usize::try_from(n).ok())
            .unwrap_or(0)
    };
    let layer_height = doc
        .get("material")
        .and_then(|m| m.get(index(doc, "current_material")))
        .and_then(|m| m.get("slice")?.get(index(m, "current_slice")))
        .and_then(|s| s.get("print")?.get("layerheight")?.get("currentvalue"))
        .and_then(Value::as_f64)
        .filter(|x| x.is_finite() && *x > 0.0)
        .map_or(0.05, from_f32);
    Ok(Machine {
        key: MachineKey(format!("chitubox:{machine_type}")),
        name: string("machinename").unwrap_or(machine_type).to_owned(),
        vendor: info
            .get("vendor")
            .and_then(|v| v.as_str().or_else(|| v.get("currentvalue")?.as_str()))
            .map(str::to_owned),
        model: Some(machine_type.to_owned()),
        technology: Technology::Msla,
        volume: BuildVolume {
            bed: BedShape::Rect { size: bed },
            height: size("machineheight")?,
            exclude: Vec::new(),
            slicer_origin: DVec2::ZERO,
        },
        process: ProcessParams::Resin(ResinParams {
            resolution,
            pixel_size,
            layer_height,
            default_elevation: 5.0,
        }),
        min_feature: 2.0 * pixel_size.max_element(),
        default_critical_angle: 30f64.to_radians(),
        slicer: Some(SlicerKind::Chitubox),
        source: MachineSource::Chitubox {
            file: file.to_owned(),
        },
    })
}

fn load(file: &Path) -> Result<Machine, AmError> {
    let doc: Value = serde_json::from_str(&read_text(file)?).map_err(|e| profile_error(file, e))?;
    machine(file, &doc).map_err(|reason| profile_error(file, reason))
}

/// Imports every `.cfgx` machine in the given folders (and their vendor folders) as
/// `chitubox:<machine type>`. A second, different machine of the same type gets a ` (2)`
/// suffix. Chitubox configures LCD printers, so the technology is MSLA.
pub fn import_chitubox(dirs: &[PathBuf]) -> ImportResult {
    let mut out = Collector::default();
    for dir in dirs {
        for file in files_with_extension(dir, "cfgx", 2) {
            if file
                .file_stem()
                .is_some_and(|s| LISTS.iter().any(|l| s.eq_ignore_ascii_case(l)))
            {
                continue;
            }
            match load(&file) {
                Ok(mut m) => {
                    let base = m.key.0.clone();
                    let name = m.name.clone();
                    let mut n = 1;
                    // Skip suffixes taken by a different machine; stop at an identical one.
                    while let Some(other) = out.machines.get(&m.key) {
                        if other.volume == m.volume && other.process == m.process {
                            break;
                        }
                        n += 1;
                        m.key = MachineKey(format!("{base} ({n})"));
                        m.name = format!("{name} ({n})");
                    }
                    out.add(&file, m);
                }
                Err(e) => out.warn(&file, e),
            }
        }
    }
    out.finish()
}
