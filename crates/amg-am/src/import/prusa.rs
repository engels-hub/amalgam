//! PrusaSlicer printer presets: `printer/*.ini` (one preset per file) and the
//! `[printer:<name>]` sections of the `vendor/*.ini` bundles. Read for behaviour only: no
//! PrusaSlicer code is used.

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};

use super::{
    Collector, Extruders, ImportResult, MAX_INHERITS_DEPTH, build_volume, fff_params,
    files_with_extension, parse_points, profile_error, read_text,
};
use crate::AmError;
use crate::machine::{Machine, MachineKey, MachineSource, ProcessParams, SlicerKind, Technology};

type Values = BTreeMap<String, String>;

struct Preset {
    name: String,
    file: PathBuf,
    /// The bundle's vendor name (file stem of a vendor bundle).
    vendor: Option<String>,
    values: Values,
}

impl Preset {
    /// `*name*` sections are bases shared by other presets.
    fn is_abstract(&self) -> bool {
        self.name.starts_with('*') && self.name.ends_with('*')
    }
}

/// The `key = value` lines of an INI file, by section (`""` before the first header).
fn parse_ini(s: &str) -> Vec<(String, Values)> {
    let mut sections = vec![(String::new(), Values::new())];
    for line in s.lines().map(str::trim) {
        if line.is_empty() || line.starts_with(['#', ';']) {
            continue;
        }
        if let Some(name) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            sections.push((name.trim().to_owned(), Values::new()));
        } else if let Some((k, v)) = line.split_once('=')
            && let Some(last) = sections.last_mut()
        {
            last.1.insert(k.trim().to_owned(), v.trim().to_owned());
        }
    }
    sections
}

/// The printer presets of one file. A file without printer sections is a single preset named
/// after the file.
fn presets(file: &Path, bundle: bool) -> Result<Vec<Preset>, AmError> {
    let sections = parse_ini(&read_text(file)?);
    let stem = file
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    if !bundle {
        let values = sections.into_iter().next().map(|s| s.1).unwrap_or_default();
        return Ok(vec![Preset {
            name: stem,
            file: file.to_owned(),
            vendor: None,
            values,
        }]);
    }
    Ok(sections
        .into_iter()
        .filter_map(|(section, values)| {
            let name = section.strip_prefix("printer:")?.trim().to_owned();
            Some(Preset {
                name,
                file: file.to_owned(),
                vendor: Some(stem.clone()),
                values,
            })
        })
        .collect())
}

struct Library {
    presets: Vec<Preset>,
    by_name: HashMap<String, Vec<usize>>,
}

impl Library {
    /// The preset's values over those of its parents (`inherits = a; b`: later parents win).
    /// A parent is looked up in the child's own file first. `path` is the chain so far.
    fn resolve(&self, i: usize, path: &mut Vec<usize>) -> Result<Values, String> {
        let p = &self.presets[i];
        if path.contains(&i) {
            return Err(format!("`{}` is part of an `inherits` cycle", p.name));
        }
        if path.len() > MAX_INHERITS_DEPTH {
            return Err(format!(
                "more than {MAX_INHERITS_DEPTH} levels of `inherits`"
            ));
        }
        path.push(i);
        let mut out = Values::new();
        let parents = p.values.get("inherits").map(String::as_str).unwrap_or("");
        for name in parents.split(';').map(str::trim).filter(|s| !s.is_empty()) {
            let all = self.by_name.get(name).map(Vec::as_slice).unwrap_or(&[]);
            // A user preset may share the name of the vendor preset it was derived from.
            let free = || all.iter().copied().filter(|j| !path.contains(j));
            let parent = free()
                .find(|&j| self.presets[j].file == p.file)
                .or_else(|| free().next())
                .or_else(|| all.first().copied())
                .ok_or_else(|| format!("parent preset `{name}` of `{}` was not found", p.name))?;
            out.extend(self.resolve(parent, path)?);
        }
        path.pop();
        out.extend(p.values.clone());
        Ok(out)
    }
}

fn number(s: &str) -> Option<f64> {
    s.trim().parse::<f64>().ok().filter(|x| x.is_finite())
}

fn machine(p: &Preset, v: &Values) -> Result<Machine, String> {
    let get = |k: &str| v.get(k).map(String::as_str).filter(|s| !s.is_empty());
    if let Some(t) = get("printer_technology")
        && t != "FFF"
    {
        return Err(format!("printer technology `{t}` is not imported"));
    }
    let bed = get("bed_shape")
        .and_then(|s| parse_points(s.split(',')))
        .ok_or("`bed_shape` is missing or not a list of `XxY` points")?;
    let height = get("max_print_height")
        .and_then(number)
        .ok_or("`max_print_height` is missing or not a number")?;
    let nozzles = get("nozzle_diameter")
        .and_then(|s| s.split(',').map(number).collect::<Option<Vec<_>>>())
        .ok_or("`nozzle_diameter` is missing or not a list of numbers")?;
    let first = |k: &str| get(k).and_then(|s| s.split(',').next()).and_then(number);
    let (fff, min_nozzle) = fff_params(Extruders {
        nozzles,
        colours: get("extruder_colour").map_or(0, |s| s.split(';').count()),
        single_nozzle_mmu: get("single_extruder_multi_material") == Some("1"),
        min_layer: first("min_layer_height"),
        max_layer: first("max_layer_height"),
    })?;
    Ok(Machine {
        key: MachineKey(format!("prusa:{}", p.name)),
        name: p.name.clone(),
        vendor: p.vendor.clone(),
        model: get("printer_model").map(str::to_owned),
        technology: Technology::Fff,
        volume: build_volume(bed, Vec::new(), height)?,
        process: ProcessParams::Fff(fff),
        min_feature: min_nozzle,
        default_critical_angle: 45f64.to_radians(),
        slicer: Some(SlicerKind::PrusaSlicer),
        source: MachineSource::Prusa {
            preset: p.name.clone(),
            file: p.file.clone(),
        },
    })
}

/// Imports the FFF printer presets of PrusaSlicer data folders as `prusa:<preset name>`:
/// `printer/*.ini` first (they hide vendor presets of the same name), then every concrete
/// `[printer:…]` section of `vendor/*.ini`. A broken `inherits` chain gives a warning with
/// [`AmError::Inherits`]; a user preset file, which PrusaSlicer writes complete, is still
/// imported from its own values when they suffice. Resin presets give a warning.
pub fn import_prusa(dirs: &[PathBuf]) -> ImportResult {
    let mut out = Collector::default();
    let mut all = Vec::new();
    for (sub, bundle) in [("printer", false), ("vendor", true)] {
        for dir in dirs {
            for file in files_with_extension(&dir.join(sub), "ini", 0) {
                match presets(&file, bundle) {
                    Ok(p) => all.extend(p),
                    Err(e) => out.warn(&file, e),
                }
            }
        }
    }
    let mut by_name: HashMap<String, Vec<usize>> = HashMap::new();
    for (i, p) in all.iter().enumerate() {
        by_name.entry(p.name.clone()).or_default().push(i);
    }
    let lib = Library {
        presets: all,
        by_name,
    };
    for (i, p) in lib.presets.iter().enumerate() {
        // The first of a name (user before vendor) is the one listed.
        if p.is_abstract() || lib.by_name.get(&p.name).and_then(|v| v.first()) != Some(&i) {
            continue;
        }
        let (values, broken) = match lib.resolve(i, &mut Vec::new()) {
            Ok(v) => (v, false),
            Err(reason) => {
                out.warn(
                    &p.file,
                    AmError::Inherits {
                        preset: p.name.clone(),
                        reason,
                    },
                );
                if p.vendor.is_some() {
                    continue;
                }
                (p.values.clone(), true)
            }
        };
        match machine(p, &values) {
            Ok(m) => out.add(&p.file, m),
            // The broken chain already explains why values are missing.
            Err(_) if broken => {}
            Err(reason) => out.warn(&p.file, profile_error(&p.file, reason)),
        }
    }
    out.finish()
}
