//! Importing machine profiles from installed slicers (S16 AM-10, design note §6.2).
//!
//! - [`import_orca`]: OrcaSlicer JSON presets with their `inherits` chain, looked up in the
//!   user, system and install roots;
//! - [`import_prusa`]: PrusaSlicer INI printer presets and vendor bundles;
//! - [`import_chitubox`]: Chitubox `.cfgx` (JSON) machine files.
//!
//! Slicer folders are only ever read. A profile that cannot be imported becomes a
//! [`LoadWarning`]; it never stops the others. Imported machines are not stored: keep one by
//! saving it as a user profile ([`crate::machine::MachineLibrary::save_user_new`]).

mod chitubox;
mod orca;
mod prusa;

use std::collections::BTreeMap;
use std::ffi::OsString;
use std::path::{Path, PathBuf};

use amg_core::glam::DVec2;

use crate::AmError;
use crate::machine::{BedShape, BuildVolume, FffParams, LoadWarning, Machine, MachineKey};

pub use chitubox::import_chitubox;
pub use orca::{import_orca, orca_process_critical_angle};
pub use prusa::import_prusa;

/// The slicer data folders of this machine.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SlicerDataDirs {
    /// OrcaSlicer data folders (with `user/` and `system/`).
    pub orca: Vec<PathBuf>,
    /// PrusaSlicer data folders (with `printer/` and `vendor/`).
    pub prusa: Vec<PathBuf>,
    /// Chitubox machine-config folders.
    pub chitubox: Vec<PathBuf>,
}

impl SlicerDataDirs {
    /// `%APPDATA%\OrcaSlicer`, `%APPDATA%\PrusaSlicer` and
    /// `%LOCALAPPDATA%\chitubox2_0\Conf\MachineConfig`, plus the XDG (Linux) and
    /// `~/Library/Application Support` (macOS) equivalents; only folders that exist.
    pub fn discover() -> Self {
        Self::discover_with(|name| std::env::var_os(name))
    }

    /// [`Self::discover`] with the environment given by `var` (for tests).
    pub fn discover_with(var: impl Fn(&str) -> Option<OsString>) -> Self {
        let dir = |name: &str| var(name).filter(|v| !v.is_empty()).map(PathBuf::from);
        let home = dir("HOME");
        let under_home = |rel: &str| home.as_ref().map(|h| h.join(rel));
        // Settings roots (Orca, Prusa) and data roots (Chitubox keeps its configs there).
        let mut config: Vec<PathBuf> = dir("APPDATA").into_iter().collect();
        config.extend(dir("XDG_CONFIG_HOME").or_else(|| under_home(".config")));
        let mut data: Vec<PathBuf> = dir("LOCALAPPDATA").into_iter().collect();
        data.extend(dir("XDG_DATA_HOME").or_else(|| under_home(".local/share")));
        if cfg!(target_os = "macos")
            && let Some(p) = under_home("Library/Application Support")
        {
            config.push(p.clone());
            data.push(p);
        }
        let existing = |roots: &[PathBuf], rel: &[&str]| {
            let mut out: Vec<PathBuf> = Vec::new();
            for root in roots {
                let p = rel.iter().fold(root.clone(), |p, part| p.join(part));
                if p.is_dir() && !out.contains(&p) {
                    out.push(p);
                }
            }
            out
        };
        SlicerDataDirs {
            orca: existing(&config, &["OrcaSlicer"]),
            prusa: existing(&config, &["PrusaSlicer"]),
            chitubox: existing(&data, &["chitubox2_0", "Conf", "MachineConfig"]),
        }
    }
}

/// What an import found.
#[derive(Clone, Debug, Default)]
pub struct ImportResult {
    /// The imported machines, valid and in key order.
    pub machines: Vec<Machine>,
    /// Profiles that were skipped (or imported with a reservation), with the reason.
    pub warnings: Vec<LoadWarning>,
}

/// Collects machines (the first of a key wins) and warnings.
#[derive(Default)]
struct Collector {
    machines: BTreeMap<MachineKey, Machine>,
    warnings: Vec<LoadWarning>,
}

impl Collector {
    fn warn(&mut self, file: &Path, message: impl std::fmt::Display) {
        let message = message.to_string();
        if !self
            .warnings
            .iter()
            .any(|w| w.file == file && w.message == message)
        {
            self.warnings.push(LoadWarning {
                file: file.to_owned(),
                message,
            });
        }
    }

    fn add(&mut self, file: &Path, m: Machine) {
        match m.validate() {
            Ok(()) => {
                self.machines.entry(m.key.clone()).or_insert(m);
            }
            Err(e) => self.warn(file, e),
        }
    }

    fn finish(self) -> ImportResult {
        ImportResult {
            machines: self.machines.into_values().collect(),
            warnings: self.warnings,
        }
    }
}

fn profile_error(file: &Path, reason: impl std::fmt::Display) -> AmError {
    AmError::Profile {
        file: file.to_owned(),
        reason: reason.to_string(),
    }
}

/// The text of a profile file, without a UTF-8 byte-order mark.
fn read_text(file: &Path) -> Result<String, AmError> {
    let mut s = std::fs::read_to_string(file).map_err(|e| profile_error(file, e))?;
    if s.starts_with('\u{feff}') {
        s.remove(0);
    }
    Ok(s)
}

/// The entries of `dir`, sorted; empty if it cannot be read (slicer folders are optional).
fn entries(dir: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = std::fs::read_dir(dir)
        .map(|it| it.filter_map(Result::ok).map(|e| e.path()).collect())
        .unwrap_or_default();
    out.sort();
    out
}

/// Files with extension `ext` in `dir` and up to `depth` levels of folders below it, sorted.
fn files_with_extension(dir: &Path, ext: &str, depth: u32) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for p in entries(dir) {
        if p.is_dir() {
            if depth > 0 {
                out.extend(files_with_extension(&p, ext, depth - 1));
            }
        } else if p.extension().is_some_and(|x| x.eq_ignore_ascii_case(ext)) {
            out.push(p);
        }
    }
    out
}

/// The most `inherits` steps a preset may have.
const MAX_INHERITS_DEPTH: usize = 16;

/// A slicer's `"<x>x<y>"` point list as a polygon (a repeated closing point is dropped).
fn parse_points<'a>(items: impl Iterator<Item = &'a str>) -> Option<Vec<DVec2>> {
    let mut out = Vec::new();
    for item in items.map(str::trim).filter(|s| !s.is_empty()) {
        let (x, y) = item.split_once(['x', 'X'])?;
        let p = DVec2::new(x.trim().parse().ok()?, y.trim().parse().ok()?);
        if !p.is_finite() {
            return None;
        }
        out.push(p);
    }
    if out.len() > 1 && out.first() == out.last() {
        out.pop();
    }
    Some(out)
}

fn twice_signed_area(p: &[DVec2]) -> f64 {
    (0..p.len())
        .map(|i| p[i].perp_dot(p[(i + 1) % p.len()]))
        .sum()
}

/// A build volume from slicer coordinates: the plate frame starts at the bed's lower-left
/// corner, so everything is shifted by `-min` and `slicer_origin` records where the slicer's
/// (0, 0) ends up. Four axis-aligned corners give a rectangle. Exclude areas without area
/// (Orca writes `["0x0"]` for none) are dropped.
fn build_volume(
    bed: Vec<DVec2>,
    exclude: Vec<Vec<DVec2>>,
    height: f64,
) -> Result<BuildVolume, String> {
    if bed.len() < 3 {
        return Err(format!("the bed outline has only {} points", bed.len()));
    }
    let (lo, hi) = bed
        .iter()
        .fold((DVec2::INFINITY, DVec2::NEG_INFINITY), |(lo, hi), p| {
            (lo.min(*p), hi.max(*p))
        });
    // `0.0 - lo` rather than `-lo`: no negative zeros in saved profiles.
    let origin = DVec2::ZERO - lo;
    let mut points: Vec<DVec2> = bed.iter().map(|p| *p + origin).collect();
    if twice_signed_area(&points) < 0.0 {
        points.reverse();
    }
    let size = hi - lo;
    let corners = [
        DVec2::ZERO,
        DVec2::new(size.x, 0.0),
        size,
        DVec2::new(0.0, size.y),
    ];
    let near = |a: DVec2, b: DVec2| a.abs_diff_eq(b, 1e-6);
    // In order: counter-clockwise and starting anywhere, so a bow-tie is not a rectangle.
    let is_rect = points.len() == 4
        && points
            .iter()
            .position(|p| near(*p, DVec2::ZERO))
            .is_some_and(|s| (0..4).all(|i| near(points[(s + i) % 4], corners[i])));
    Ok(BuildVolume {
        bed: if is_rect {
            BedShape::Rect { size }
        } else {
            BedShape::Polygon { points }
        },
        height,
        exclude: exclude
            .into_iter()
            .filter(|e| e.len() >= 3 && twice_signed_area(e).abs() > 1e-9)
            .map(|e| e.into_iter().map(|p| p + origin).collect())
            .collect(),
        slicer_origin: origin,
    })
}

/// The extruder values of an Orca or Prusa printer preset.
struct Extruders {
    nozzles: Vec<f64>,
    colours: usize,
    /// One nozzle fed by several filaments.
    single_nozzle_mmu: bool,
    min_layer: Option<f64>,
    max_layer: Option<f64>,
}

/// FFF parameters from a printer preset. The filament slot count is the number of nozzle
/// entries, or of extruder colours when there are more (design note §6.1). A single-nozzle
/// multi-material printer with equal entries has one physical nozzle. Printer presets carry no
/// layer height, so 0.2 mm is clamped into the printer's range; a limit of 0 means automatic.
fn fff_params(e: Extruders) -> Result<(FffParams, f64), String> {
    let Some(&first) = e.nozzles.first() else {
        return Err("`nozzle_diameter` is empty".into());
    };
    if e.nozzles.iter().any(|d| !(d.is_finite() && *d > 0.0)) {
        return Err("`nozzle_diameter` must be positive".into());
    }
    let slots = u8::try_from(e.nozzles.len().max(e.colours))
        .map_err(|_| "more than 255 filament slots".to_owned())?;
    let min_nozzle = e.nozzles.iter().copied().fold(first, f64::min);
    let nozzles = if e.single_nozzle_mmu && e.nozzles.iter().all(|d| d.to_bits() == first.to_bits())
    {
        vec![first]
    } else {
        e.nozzles
    };
    let set = |v: Option<f64>| v.filter(|x| x.is_finite() && *x > 0.0);
    let hi = set(e.max_layer).unwrap_or(0.75 * min_nozzle);
    let lo = set(e.min_layer).unwrap_or(0.07).min(hi);
    let layer = 0.2_f64.clamp(lo, hi);
    Ok((
        FffParams {
            nozzle_diameters: nozzles,
            filament_slots: slots,
            layer_height: layer,
            first_layer_height: layer,
            layer_height_range: (lo, hi),
        },
        min_nozzle,
    ))
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    fn pts(v: &[&str]) -> Vec<DVec2> {
        parse_points(v.iter().copied()).unwrap()
    }

    #[test]
    fn a_centred_polygon_bed_is_shifted_into_the_plate_frame() {
        // Clockwise triangle around the slicer origin.
        let v = build_volume(pts(&["-10x-10", "0x20", "10x-10", "-10x-10"]), vec![], 50.0).unwrap();
        assert_eq!(v.slicer_origin, DVec2::new(10.0, 10.0));
        let BedShape::Polygon { points } = &v.bed else {
            panic!("{:?}", v.bed);
        };
        assert_eq!(points.len(), 3);
        assert!(twice_signed_area(points) > 0.0);
        assert!(points.contains(&DVec2::new(10.0, 30.0)));
    }

    #[test]
    fn rectangles_are_detected_and_empty_excludes_dropped() {
        let v = build_volume(
            pts(&["0x220", "0x0", "200x0", "200x220"]),
            vec![pts(&["0x0"]), pts(&["0x0", "18x0", "18x28", "0x28"])],
            240.0,
        )
        .unwrap();
        assert_eq!(
            v.bed,
            BedShape::Rect {
                size: DVec2::new(200.0, 220.0)
            }
        );
        assert_eq!(v.exclude.len(), 1);
        assert_eq!(v.slicer_origin, DVec2::ZERO);
        assert!(v.slicer_origin.x.is_sign_positive());
        let bow_tie = build_volume(pts(&["0x0", "10x10", "10x0", "0x10"]), vec![], 1.0).unwrap();
        assert!(matches!(bow_tie.bed, BedShape::Polygon { .. }));
        assert!(build_volume(pts(&["0x0", "1x0"]), vec![], 1.0).is_err());
        assert!(parse_points(["0x0", "nonsense"].into_iter()).is_none());
    }

    #[test]
    fn extruder_rules() {
        let e = |nozzles: &[f64], colours, mmu| Extruders {
            nozzles: nozzles.to_vec(),
            colours,
            single_nozzle_mmu: mmu,
            min_layer: Some(0.0),
            max_layer: Some(0.0),
        };
        let (mmu, min) = fff_params(e(&[0.4; 5], 5, true)).unwrap();
        assert_eq!(
            (mmu.nozzle_diameters.len(), mmu.filament_slots, min),
            (1, 5, 0.4)
        );
        let (idex, _) = fff_params(e(&[0.4, 0.4], 2, false)).unwrap();
        assert_eq!((idex.nozzle_diameters.len(), idex.filament_slots), (2, 2));
        let (colours, _) = fff_params(e(&[0.4], 4, false)).unwrap();
        assert_eq!(
            (colours.nozzle_diameters.len(), colours.filament_slots),
            (1, 4)
        );
        assert_eq!(colours.layer_height_range, (0.07, 0.75 * 0.4));
        assert_eq!(colours.layer_height, 0.2);
        let (fine, _) = fff_params(e(&[0.15], 1, false)).unwrap();
        assert!(fine.layer_height <= 0.15 * 0.75 + 1e-12);
        assert!(fff_params(e(&[], 1, false)).is_err());
        assert!(fff_params(e(&[0.0], 1, false)).is_err());
    }
}
