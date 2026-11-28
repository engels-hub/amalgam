//! OrcaSlicer machine presets (design note §6.2). Read for behaviour only: no OrcaSlicer code
//! is used.

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::rc::Rc;

use serde_json::{Map, Value};

use super::{
    Collector, Extruders, ImportResult, MAX_INHERITS_DEPTH, build_volume, entries, fff_params,
    files_with_extension, parse_points, profile_error, read_text,
};
use crate::AmError;
use crate::machine::{Machine, MachineKey, MachineSource, ProcessParams, SlicerKind, Technology};

type Object = Map<String, Value>;

/// One preset file.
struct Preset {
    name: String,
    file: PathBuf,
    /// The vendor folder (system and install presets).
    vendor: Option<String>,
    values: Object,
}

impl Preset {
    fn inherits(&self) -> Option<&str> {
        text(&self.values, "inherits")
    }

    /// An abstract base, by its own `instantiation` (children do not take it over).
    fn is_abstract(&self) -> bool {
        self.values
            .get("instantiation")
            .is_some_and(|v| v.as_str() == Some("false") || v.as_bool() == Some(false))
    }
}

/// A non-empty string value (or the first element of an array of strings).
fn text<'a>(o: &'a Object, key: &str) -> Option<&'a str> {
    let v = o.get(key)?;
    v.as_str()
        .or_else(|| v.as_array()?.first()?.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
}

/// An array of strings; a single string is split at commas.
fn list<'a>(o: &'a Object, key: &str) -> Vec<&'a str> {
    match o.get(key) {
        Some(Value::Array(a)) => a.iter().filter_map(Value::as_str).collect(),
        Some(Value::String(s)) => s.split(',').collect(),
        _ => Vec::new(),
    }
}

fn number(s: &str) -> Option<f64> {
    s.trim().parse::<f64>().ok().filter(|x| x.is_finite())
}

/// The folders of one level of the search order, with the vendor each belongs to.
struct Folder {
    dir: PathBuf,
    vendor: Option<String>,
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum Level {
    User,
    System,
    Install,
}

/// The presets of one folder.
#[derive(Default)]
struct Loaded {
    /// In file order.
    all: Vec<Rc<Preset>>,
    by_name: HashMap<String, Vec<Rc<Preset>>>,
}

/// The search roots. Folders are read when first needed, so the several thousand install
/// presets are only opened when a parent is not in the data folder.
struct Roots {
    user: Vec<Folder>,
    system: Vec<Folder>,
    install: Vec<Folder>,
    cache: HashMap<PathBuf, Rc<Loaded>>,
}

/// `<root>/*/machine`, one per vendor.
fn vendor_folders(root: &Path) -> impl Iterator<Item = Folder> + use<> {
    entries(root).into_iter().filter_map(|v| {
        let dir = v.join("machine");
        dir.is_dir().then(|| Folder {
            vendor: v.file_name().map(|n| n.to_string_lossy().into_owned()),
            dir,
        })
    })
}

/// `<data>/user/*/machine`, the `default` user first.
fn user_folders(data: &Path) -> Vec<Folder> {
    let mut users = entries(&data.join("user"));
    users.sort_by_key(|u| u.file_name().is_none_or(|n| n != "default"));
    users
        .into_iter()
        .map(|u| u.join("machine"))
        .filter(|d| d.is_dir())
        .map(|dir| Folder { dir, vendor: None })
        .collect()
}

fn parse_preset(file: &Path, vendor: Option<&str>) -> Result<Option<Preset>, AmError> {
    let s = read_text(file)?;
    let value: Value = serde_json::from_str(&s).map_err(|e| profile_error(file, e))?;
    let Value::Object(values) = value else {
        return Err(profile_error(file, "not a JSON object"));
    };
    // `machine_model` files describe a product, not a preset.
    if text(&values, "type").is_some_and(|t| t != "machine") {
        return Ok(None);
    }
    let name = text(&values, "name").map(str::to_owned).unwrap_or_else(|| {
        file.file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default()
    });
    Ok(Some(Preset {
        name,
        file: file.to_owned(),
        vendor: vendor.map(str::to_owned),
        values,
    }))
}

impl Roots {
    fn new(data_dirs: &[PathBuf], install_dirs: &[PathBuf]) -> Roots {
        Roots {
            user: data_dirs.iter().flat_map(|d| user_folders(d)).collect(),
            system: data_dirs
                .iter()
                .flat_map(|d| vendor_folders(&d.join("system")))
                .collect(),
            install: install_dirs
                .iter()
                .flat_map(|d| vendor_folders(&d.join("resources").join("profiles")))
                .collect(),
            cache: HashMap::new(),
        }
    }

    fn folders(&self, level: Level) -> &[Folder] {
        match level {
            Level::User => &self.user,
            Level::System => &self.system,
            Level::Install => &self.install,
        }
    }

    fn load(&mut self, level: Level, i: usize, out: &mut Collector) -> Rc<Loaded> {
        let folder = &self.folders(level)[i];
        let (dir, vendor) = (folder.dir.clone(), folder.vendor.clone());
        if let Some(c) = self.cache.get(&dir) {
            return Rc::clone(c);
        }
        let mut loaded = Loaded::default();
        // User printers without a parent live in `machine/base/`.
        for file in files_with_extension(&dir, "json", 2) {
            match parse_preset(&file, vendor.as_deref()) {
                Ok(Some(p)) => {
                    let p = Rc::new(p);
                    loaded
                        .by_name
                        .entry(p.name.clone())
                        .or_default()
                        .push(Rc::clone(&p));
                    loaded.all.push(p);
                }
                Ok(None) => {}
                Err(e) => out.warn(&file, e),
            }
        }
        let loaded = Rc::new(loaded);
        self.cache.insert(dir, Rc::clone(&loaded));
        loaded
    }

    /// The preset `name` for a child of `vendor`, skipping files already in the chain: user
    /// folders first, then the child's own vendor (every vendor has its own
    /// `fdm_machine_common`), then any system and install vendor. The flag of the error is true
    /// when the only matches are in the chain already (a cycle).
    fn find(
        &mut self,
        name: &str,
        vendor: Option<&str>,
        visited: &HashSet<PathBuf>,
        out: &mut Collector,
    ) -> Result<Rc<Preset>, bool> {
        let mut seen = false;
        let passes: [(Level, bool); 5] = [
            (Level::User, false),
            (Level::System, true),
            (Level::Install, true),
            (Level::System, false),
            (Level::Install, false),
        ];
        for (level, same_vendor) in passes {
            for i in 0..self.folders(level).len() {
                if same_vendor
                    && (vendor.is_none() || self.folders(level)[i].vendor.as_deref() != vendor)
                {
                    continue;
                }
                let loaded = self.load(level, i, out);
                for p in loaded.by_name.get(name).into_iter().flatten() {
                    if visited.contains(&p.file) {
                        seen = true;
                    } else {
                        return Ok(Rc::clone(p));
                    }
                }
            }
        }
        Err(seen)
    }

    /// The `inherits` chain of `child`, child first.
    fn chain(
        &mut self,
        child: &Rc<Preset>,
        out: &mut Collector,
    ) -> Result<Vec<Rc<Preset>>, AmError> {
        let broken = |reason: String| AmError::Inherits {
            preset: child.name.clone(),
            reason,
        };
        let mut chain = vec![Rc::clone(child)];
        let mut visited = HashSet::from([child.file.clone()]);
        let mut last = Rc::clone(child);
        while let Some(parent) = last.inherits().map(str::to_owned) {
            if chain.len() > MAX_INHERITS_DEPTH {
                return Err(broken(format!(
                    "more than {MAX_INHERITS_DEPTH} levels of `inherits`"
                )));
            }
            last = self
                .find(&parent, last.vendor.as_deref(), &visited, out)
                .map_err(|cycle| {
                    broken(if cycle {
                        format!("`{}` inherits `{parent}`, which is a cycle", last.name)
                    } else {
                        format!("parent preset `{parent}` of `{}` was not found", last.name)
                    })
                })?;
            visited.insert(last.file.clone());
            chain.push(Rc::clone(&last));
        }
        Ok(chain)
    }
}

/// The machine of a resolved chain (child first); child values replace parent values whole.
fn machine(chain: &[Rc<Preset>]) -> Result<Machine, String> {
    let child = &chain[0];
    let mut v = Object::new();
    for p in chain.iter().rev() {
        v.extend(p.values.clone());
    }
    if let Some(t) = text(&v, "printer_technology")
        && t != "FFF"
    {
        return Err(format!("printer technology `{t}` is not imported"));
    }
    let bed = parse_points(list(&v, "printable_area").into_iter())
        .filter(|p| !p.is_empty())
        .ok_or("`printable_area` is missing or not a list of `XxY` points")?;
    let height = text(&v, "printable_height")
        .and_then(number)
        .ok_or("`printable_height` is missing or not a number")?;
    let exclude = parse_points(list(&v, "bed_exclude_area").into_iter())
        .ok_or("`bed_exclude_area` is not a list of `XxY` points")?;
    let nozzles = list(&v, "nozzle_diameter")
        .into_iter()
        .map(number)
        .collect::<Option<Vec<_>>>()
        .ok_or("`nozzle_diameter` is not a list of numbers")?;
    let (fff, min_nozzle) = fff_params(Extruders {
        nozzles,
        colours: list(&v, "extruder_colour").len(),
        single_nozzle_mmu: text(&v, "single_extruder_multi_material") == Some("1"),
        min_layer: text(&v, "min_layer_height").and_then(number),
        max_layer: text(&v, "max_layer_height").and_then(number),
    })?;
    Ok(Machine {
        key: MachineKey(format!("orca:{}", child.name)),
        name: child.name.clone(),
        vendor: chain.iter().find_map(|p| p.vendor.clone()),
        model: text(&v, "printer_model").map(str::to_owned),
        technology: Technology::Fff,
        volume: build_volume(bed, vec![exclude], height)?,
        process: ProcessParams::Fff(fff),
        min_feature: min_nozzle,
        default_critical_angle: 45f64.to_radians(),
        slicer: Some(SlicerKind::OrcaSlicer),
        source: MachineSource::Orca {
            preset: child.name.clone(),
            file: child.file.clone(),
            chain: chain.iter().map(|p| p.name.clone()).collect(),
        },
    })
}

/// Imports the machine presets of OrcaSlicer data folders: every preset under
/// `user/*/machine` and `system/*/machine` that is not abstract (`instantiation: false`), as
/// `orca:<preset name>` (a user preset hides a system preset of the same name). `inherits` is
/// resolved by name through the user and system folders and then the profiles shipped in
/// `install_dirs` (`<install>/resources/profiles`), whose own presets are not listed. A broken
/// chain (missing parent, cycle, more than 16 levels) gives a warning with
/// [`AmError::Inherits`] and no machine.
pub fn import_orca(data_dirs: &[PathBuf], install_dirs: &[PathBuf]) -> ImportResult {
    let mut roots = Roots::new(data_dirs, install_dirs);
    let mut out = Collector::default();
    // Names already handled: the first in search order wins, even if it fails.
    let mut done = HashSet::new();
    for level in [Level::User, Level::System] {
        for i in 0..roots.folders(level).len() {
            let loaded = roots.load(level, i, &mut out);
            for p in &loaded.all {
                if p.is_abstract() || !done.insert(p.name.clone()) {
                    continue;
                }
                match roots.chain(p, &mut out) {
                    Ok(chain) => match machine(&chain) {
                        Ok(m) => out.add(&p.file, m),
                        Err(reason) => out.warn(&p.file, profile_error(&p.file, reason)),
                    },
                    Err(e) => out.warn(&p.file, e),
                }
            }
        }
    }
    out.finish()
}

/// The critical overhang angle (rad) of an Orca process preset: `support_threshold_angle` in
/// degrees, if set and in (0, 90] (0 means "automatic"). Pass a resolved preset; this does not
/// follow `inherits`.
pub fn orca_process_critical_angle(json: &Value) -> Option<f64> {
    let o = json.as_object()?;
    let v = o.get("support_threshold_angle")?;
    let degrees = v
        .as_f64()
        .or_else(|| text(o, "support_threshold_angle").and_then(number))?;
    (degrees > 0.0 && degrees <= 90.0).then(|| degrees.to_radians())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn critical_angle_values() {
        let a = |v: Value| orca_process_critical_angle(&v);
        let thirty = Some(30f64.to_radians());
        assert_eq!(
            a(serde_json::json!({"support_threshold_angle": "30"})),
            thirty
        );
        assert_eq!(
            a(serde_json::json!({"support_threshold_angle": 30})),
            thirty
        );
        assert_eq!(
            a(serde_json::json!({"support_threshold_angle": ["30"]})),
            thirty
        );
        for none in [
            serde_json::json!({"support_threshold_angle": "0"}),
            serde_json::json!({"support_threshold_angle": "-5"}),
            serde_json::json!({"support_threshold_angle": "120"}),
            serde_json::json!({"support_threshold_angle": "auto"}),
            serde_json::json!({"layer_height": "0.2"}),
            serde_json::json!([30]),
        ] {
            assert_eq!(a(none.clone()), None, "{none}");
        }
    }
}
