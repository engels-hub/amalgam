//! Finding installed slicers: registry (Windows, read-only), well-known directories, `PATH`.

use std::ffi::OsString;
use std::path::{Path, PathBuf};

use crate::machine::SlicerKind;

/// Where an install was found. The order is the preference order when the same exe is found
/// twice (the better source is kept).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum DetectSource {
    /// Windows Uninstall registry entry.
    Registry,
    /// A well-known install directory.
    KnownDir,
    /// A directory on `PATH`.
    Path,
    /// Chosen by the user.
    User,
}

/// One detected slicer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SlicerInstall {
    /// Which slicer.
    pub kind: SlicerKind,
    /// The executable (an existing file).
    pub exe: PathBuf,
    /// Version, when the source knows it (the registry's `DisplayVersion`).
    pub version: Option<String>,
    /// Where it was found.
    pub source: DetectSource,
}

/// What [`detect_slicers_with`] looks at. [`DetectOptions::system`] is the real machine; tests
/// build their own so that they do not depend on what is installed.
#[derive(Clone, Debug, Default)]
pub struct DetectOptions {
    /// Read the Uninstall registry keys (Windows only; ignored elsewhere).
    pub registry: bool,
    /// Directories whose children (`<root>/<slicer folder>/<exe>`) are checked.
    pub known_roots: Vec<PathBuf>,
    /// The `PATH` value to search.
    pub path: Option<OsString>,
}

impl DetectOptions {
    /// The registry, `%ProgramFiles%`, `%ProgramFiles(x86)%`, `%LOCALAPPDATA%\Programs` and the
    /// process `PATH`.
    pub fn system() -> Self {
        let mut known_roots = Vec::new();
        for var in ["ProgramFiles", "ProgramFiles(x86)", "ProgramW6432"] {
            if let Some(p) = std::env::var_os(var) {
                known_roots.push(PathBuf::from(p));
            }
        }
        if let Some(p) = std::env::var_os("LOCALAPPDATA") {
            known_roots.push(PathBuf::from(p).join("Programs"));
        }
        if cfg!(target_os = "macos") {
            known_roots.push(PathBuf::from("/Applications"));
        }
        known_roots.sort();
        known_roots.dedup();
        Self {
            registry: true,
            known_roots,
            path: std::env::var_os("PATH"),
        }
    }
}

/// Executable file names per slicer (checked case-insensitively on Windows by the file system).
fn exe_names(kind: SlicerKind) -> &'static [&'static str] {
    match kind {
        SlicerKind::OrcaSlicer => &["orca-slicer.exe", "orca-slicer"],
        SlicerKind::PrusaSlicer => &["prusa-slicer.exe", "prusa-slicer"],
        SlicerKind::BambuStudio => &["bambu-studio.exe", "bambu-studio"],
        SlicerKind::Chitubox => &[
            "CHITUBOX.exe",
            "CHITUBOX64.exe",
            "CHITUBOX Basic.exe",
            "CHITUBOXBasic.exe",
            "CHITUBOX_Basic.exe",
        ],
        SlicerKind::Custom => &[],
    }
}

/// Folder names under an install root.
fn folder_names(kind: SlicerKind) -> &'static [&'static str] {
    match kind {
        SlicerKind::OrcaSlicer => &["OrcaSlicer", "Orca Slicer", "orca-slicer"],
        SlicerKind::PrusaSlicer => &["PrusaSlicer", "Prusa3D\\PrusaSlicer", "Prusa Slicer"],
        SlicerKind::BambuStudio => &["Bambu Studio", "BambuStudio"],
        SlicerKind::Chitubox => &[
            "CHITUBOX",
            "CHITUBOX64",
            "CHITUBOX Basic",
            "CHITUBOXBasic",
            "CHITUBOX Pro",
            "CBD/CHITUBOX_Basic",
            "CBD/CHITUBOX Basic",
        ],
        SlicerKind::Custom => &[],
    }
}

const DETECTABLE: [SlicerKind; 4] = [
    SlicerKind::OrcaSlicer,
    SlicerKind::BambuStudio,
    SlicerKind::PrusaSlicer,
    SlicerKind::Chitubox,
];

fn kind_rank(k: SlicerKind) -> u8 {
    match k {
        SlicerKind::OrcaSlicer => 0,
        SlicerKind::BambuStudio => 1,
        SlicerKind::PrusaSlicer => 2,
        SlicerKind::Chitubox => 3,
        SlicerKind::Custom => 4,
    }
}

/// Which slicer a registry `DisplayName` (or a folder name) denotes. Ignores case, spaces,
/// dashes and underscores. Console helpers and uninstallers do not match.
pub fn classify_name(name: &str) -> Option<SlicerKind> {
    let n: String = name
        .chars()
        .filter(|c| c.is_alphanumeric())
        .flat_map(char::to_lowercase)
        .collect();
    if n.contains("uninstall") {
        return None;
    }
    if n.contains("orcaslicer") {
        Some(SlicerKind::OrcaSlicer)
    } else if n.contains("bambustudio") {
        Some(SlicerKind::BambuStudio)
    } else if n.contains("prusaslicer") {
        Some(SlicerKind::PrusaSlicer)
    } else if n.contains("chitubox") {
        Some(SlicerKind::Chitubox)
    } else {
        None
    }
}

/// Which slicer an executable path is, from its file name (`orca-slicer.exe`, …). The
/// `-console` and `-gcodeviewer` helpers are not slicers.
pub fn classify_exe(exe: &Path) -> Option<SlicerKind> {
    let stem = exe.file_stem()?.to_str()?.to_lowercase();
    if stem.ends_with("console") || stem.contains("gcodeviewer") || stem.contains("uninst") {
        return None;
    }
    classify_name(&stem)
}

/// Cleans a registry value into an `.exe` path candidate (no file-system access): quotes,
/// trailing `,<icon index>` and command-line arguments are dropped, and a path that repeats
/// itself behind a second drive prefix (`D:\X\D:\X\app.exe`, seen in the owner's OrcaSlicer
/// entry) is cut to its last drive-rooted part.
fn exe_candidate(value: &str) -> Option<PathBuf> {
    let v = value.trim();
    let v = v.strip_prefix('"').unwrap_or(v);
    // Up to the first `.exe` (any case) that ends a path component, which drops `,0`,
    // closing quotes and arguments alike.
    let lower = v.to_ascii_lowercase();
    let mut end = None;
    let mut from = 0;
    while let Some(i) = lower[from..].find(".exe") {
        let e = from + i + 4;
        let next = lower[e..].chars().next();
        if next.is_none_or(|c| matches!(c, '"' | ',' | ' ' | '\t')) {
            end = Some(e);
            break;
        }
        from = e;
    }
    let mut s = &v[..end?];
    // The last position that starts a drive-rooted path (`X:\` or `X:/`) wins.
    let b = s.as_bytes();
    let mut start = 0;
    for i in 1..b.len().saturating_sub(2) {
        if b[i].is_ascii_alphabetic()
            && b[i + 1] == b':'
            && matches!(b[i + 2], b'\\' | b'/')
            && matches!(b[i - 1], b'\\' | b'/')
        {
            start = i;
        }
    }
    s = &s[start..];
    let s = s.trim();
    (s.len() > 4).then(|| PathBuf::from(s))
}

/// Extracts an `.exe` path from a registry `DisplayIcon` / `UninstallString` value, if the
/// file exists. Handles quotes, `,0` icon indices, arguments and the doubled
/// `"D:\X\D:\X\app.exe"` form. See [`parse_registry_exe_with`] to supply the existence test.
pub fn parse_registry_exe(value: &str) -> Option<PathBuf> {
    parse_registry_exe_with(value, Path::is_file)
}

/// [`parse_registry_exe`] with a caller-supplied existence check (unit tests use a fixed set).
pub fn parse_registry_exe_with(value: &str, exists: impl Fn(&Path) -> bool) -> Option<PathBuf> {
    let p = exe_candidate(value)?;
    exists(&p).then_some(p)
}

/// Detects slicers on this machine: registry, well-known directories, `PATH`. Sorted (Orca,
/// Bambu, Prusa, Chitubox; registry before directories before `PATH`; then by path) and
/// de-duplicated by executable. Reads only.
pub fn detect_slicers() -> Vec<SlicerInstall> {
    detect_slicers_with(&DetectOptions::system())
}

/// [`detect_slicers`] against explicit sources.
pub fn detect_slicers_with(opts: &DetectOptions) -> Vec<SlicerInstall> {
    let mut found: Vec<SlicerInstall> = Vec::new();
    if opts.registry {
        found.extend(registry_installs());
    }
    for root in &opts.known_roots {
        for kind in DETECTABLE {
            for folder in folder_names(kind) {
                let dir = root.join(folder);
                for name in exe_names(kind) {
                    let exe = dir.join(name);
                    if exe.is_file() {
                        found.push(SlicerInstall {
                            kind,
                            exe,
                            version: None,
                            source: DetectSource::KnownDir,
                        });
                    }
                }
            }
        }
    }
    if let Some(path) = &opts.path {
        for dir in std::env::split_paths(path) {
            for kind in DETECTABLE {
                for name in exe_names(kind) {
                    let exe = dir.join(name);
                    if exe.is_file() {
                        found.push(SlicerInstall {
                            kind,
                            exe,
                            version: None,
                            source: DetectSource::Path,
                        });
                    }
                }
            }
        }
    }
    found.sort_by(|a, b| {
        (kind_rank(a.kind), a.source, &a.exe).cmp(&(kind_rank(b.kind), b.source, &b.exe))
    });
    let mut seen: Vec<String> = Vec::new();
    found.retain(|i| {
        let key = i.exe.to_string_lossy().replace('/', "\\").to_lowercase();
        if seen.contains(&key) {
            false
        } else {
            seen.push(key);
            true
        }
    });
    found
}

#[cfg(windows)]
fn registry_installs() -> Vec<SlicerInstall> {
    use winreg::RegKey;
    use winreg::enums::{
        HKEY_CURRENT_USER, HKEY_LOCAL_MACHINE, KEY_READ, KEY_WOW64_32KEY, KEY_WOW64_64KEY,
    };

    const UNINSTALL: &str = r"SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall";
    let mut out = Vec::new();
    for (hive, view) in [
        (HKEY_LOCAL_MACHINE, KEY_WOW64_64KEY),
        (HKEY_LOCAL_MACHINE, KEY_WOW64_32KEY),
        (HKEY_CURRENT_USER, KEY_WOW64_64KEY),
        (HKEY_CURRENT_USER, KEY_WOW64_32KEY),
    ] {
        let Ok(root) = RegKey::predef(hive).open_subkey_with_flags(UNINSTALL, KEY_READ | view)
        else {
            continue;
        };
        for sub in root.enum_keys().flatten() {
            let Ok(key) = root.open_subkey_with_flags(&sub, KEY_READ) else {
                continue;
            };
            let name: String = key.get_value("DisplayName").unwrap_or_default();
            let icon: String = key.get_value("DisplayIcon").unwrap_or_default();
            let location: String = key.get_value("InstallLocation").unwrap_or_default();
            let version: Option<String> = key.get_value("DisplayVersion").ok();
            // The key name (`OrcaSlicer`, `{GUID}_is1`) is a second hint after DisplayName.
            let by_name = classify_name(&name).or_else(|| classify_name(&sub));
            let icon_exe = parse_registry_exe(&icon);
            // Uninstallers (`Uninstall.exe`, the DisplayIcon of most installers) are not the
            // slicer, but their folder is its install folder.
            let mut exe = icon_exe
                .clone()
                .filter(|p| classify_exe(p).is_some_and(|k| by_name.is_none_or(|n| n == k)));
            if exe.is_none()
                && let Some(kind) = by_name
            {
                let mut dirs: Vec<PathBuf> = Vec::new();
                if !location.trim().is_empty() {
                    dirs.push(PathBuf::from(location.trim().trim_matches('"')));
                }
                dirs.extend(
                    icon_exe
                        .as_deref()
                        .and_then(Path::parent)
                        .map(Path::to_path_buf),
                );
                exe = dirs.iter().find_map(|dir| {
                    exe_names(kind)
                        .iter()
                        .map(|n| dir.join(n))
                        .find(|p| p.is_file())
                });
            }
            let Some(exe) = exe else { continue };
            let Some(kind) = by_name.or_else(|| classify_exe(&exe)) else {
                continue;
            };
            out.push(SlicerInstall {
                kind,
                exe,
                version: version.filter(|v| !v.trim().is_empty()),
                source: DetectSource::Registry,
            });
        }
    }
    out
}

#[cfg(not(windows))]
fn registry_installs() -> Vec<SlicerInstall> {
    Vec::new()
}
