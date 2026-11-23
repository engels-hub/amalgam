//! Manufacture settings (Phase 1 stand-in for the UI-11 "Manufacture" preferences page): the
//! slicer to hand off to, the hand-off folder and the user machine folder.
//!
//! One JSON file (`am-settings.json` in the user's config folder), process-wide like the IO-15
//! choices because commands are plain functions; tests point it at a temporary file with
//! [`set_settings_path`]. A missing or damaged file gives the defaults; a failed write is logged.

use std::borrow::Cow;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use amg_am::handoff::{
    HandoffFormat, SlicerConfig, SlicerInstall, default_handoff_dir, detect_slicers,
};
use amg_am::machine::SlicerKind;
use amg_jobs::JobHandle;
use serde::{Deserialize, Serialize};

use crate::app::AppCx;
use crate::dialog::AppDialog;
use crate::file::{FileDialogKind, FileDialogRequest};
use crate::widgets::{DialogAction, DialogFrame};

/// The remembered Manufacture settings.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AmSettings {
    /// The slicer to start; `None` = detect one (registry, known folders, `PATH`) at each send.
    #[serde(default)]
    pub slicer: Option<SlicerConfig>,
    /// Where hand-off files go; `None` = `%LOCALAPPDATA%\Amalgam\handoff`.
    #[serde(default)]
    pub handoff_dir: Option<PathBuf>,
    /// The user machine folder (`*.toml`); `None` = `<config>/Amalgam/machines`.
    #[serde(default)]
    pub machines_dir: Option<PathBuf>,
}

impl AmSettings {
    /// The hand-off folder in force.
    pub fn handoff_dir(&self) -> PathBuf {
        self.handoff_dir.clone().unwrap_or_else(default_handoff_dir)
    }

    /// The user machine folder in force (`None` without a config folder).
    pub fn machines_dir(&self) -> Option<PathBuf> {
        self.machines_dir.clone().or_else(default_machines_dir)
    }
}

/// `<config dir>/Amalgam/machines`.
pub fn default_machines_dir() -> Option<PathBuf> {
    directories::BaseDirs::new().map(|d| d.config_dir().join("Amalgam").join("machines"))
}

/// `<config dir>/Amalgam/am-settings.json`.
pub fn default_settings_path() -> Option<PathBuf> {
    directories::BaseDirs::new().map(|d| d.config_dir().join("Amalgam").join("am-settings.json"))
}

struct Store {
    path: Option<PathBuf>,
    loaded: Option<AmSettings>,
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

/// Where the settings are kept (`None`: the default path). Forgets what was loaded.
pub fn set_settings_path(path: Option<PathBuf>) {
    let mut s = store();
    s.path = path;
    s.loaded = None;
}

fn read(path: &Path) -> AmSettings {
    match std::fs::read(path) {
        Ok(bytes) => serde_json::from_slice(&bytes).unwrap_or_else(|e| {
            tracing::warn!(
                "{}: {e}; using the default Manufacture settings",
                path.display()
            );
            AmSettings::default()
        }),
        Err(_) => AmSettings::default(),
    }
}

/// The settings (loaded on first use).
pub fn load() -> AmSettings {
    let mut s = store();
    if s.loaded.is_none() {
        let path = s.path.clone().or_else(default_settings_path);
        s.loaded = Some(path.map(|p| read(&p)).unwrap_or_default());
    }
    s.loaded.clone().unwrap_or_default()
}

/// Replaces the settings and writes them.
pub fn save(settings: AmSettings) {
    let mut s = store();
    let path = s.path.clone().or_else(default_settings_path);
    if let Some(path) = path {
        let write = || -> std::io::Result<()> {
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir)?;
            }
            let tmp = path.with_extension("json.tmp");
            let json = serde_json::to_vec_pretty(&settings).map_err(std::io::Error::other)?;
            std::fs::write(&tmp, json)?;
            std::fs::rename(&tmp, &path)
        };
        if let Err(e) = write() {
            tracing::warn!("cannot save {}: {e}", path.display());
        }
    }
    s.loaded = Some(settings);
}

/// Display name of a slicer kind.
pub fn slicer_name(k: SlicerKind) -> &'static str {
    match k {
        SlicerKind::OrcaSlicer => "OrcaSlicer",
        SlicerKind::PrusaSlicer => "PrusaSlicer",
        SlicerKind::BambuStudio => "Bambu Studio",
        SlicerKind::Chitubox => "Chitubox",
        SlicerKind::Custom => "Custom",
    }
}

const KINDS: [SlicerKind; 5] = [
    SlicerKind::OrcaSlicer,
    SlicerKind::PrusaSlicer,
    SlicerKind::BambuStudio,
    SlicerKind::Chitubox,
    SlicerKind::Custom,
];

/// Which path a pending file picker fills.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum PickFor {
    Exe,
    Handoff,
    Machines,
}

/// The Manufacture Settings dialog.
pub struct SettingsDialog {
    /// Detect at each send instead of using `exe`.
    pub auto_detect: bool,
    /// Slicer kind (3MF flavour).
    pub kind: SlicerKind,
    /// The slicer executable.
    pub exe: String,
    /// Arguments, space-separated; `{file}` is the hand-off file.
    pub args: String,
    /// Hand-off format.
    pub format: HandoffFormat,
    /// Hand-off folder (empty: the default).
    pub handoff_dir: String,
    /// User machine folder (empty: the default).
    pub machines_dir: String,
    detect: Option<JobHandle<Vec<SlicerInstall>, amg_am::AmError>>,
    found: Option<Vec<SlicerInstall>>,
    picking: Option<(PickFor, std::sync::mpsc::Receiver<Option<PathBuf>>)>,
}

impl std::fmt::Debug for SettingsDialog {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SettingsDialog")
            .field("kind", &self.kind)
            .field("exe", &self.exe)
            .finish_non_exhaustive()
    }
}

impl Default for SettingsDialog {
    fn default() -> Self {
        Self::new()
    }
}

impl SettingsDialog {
    /// The dialog over the current settings.
    pub fn new() -> SettingsDialog {
        let s = load();
        let (auto, cfg) = match &s.slicer {
            Some(c) => (false, c.clone()),
            None => (
                true,
                SlicerConfig::new(SlicerKind::OrcaSlicer, PathBuf::new()),
            ),
        };
        SettingsDialog {
            auto_detect: auto,
            kind: cfg.kind,
            exe: cfg.exe.to_string_lossy().into_owned(),
            args: cfg.args.join(" "),
            format: cfg.format,
            handoff_dir: s
                .handoff_dir
                .as_ref()
                .map(|p| p.to_string_lossy().into_owned())
                .unwrap_or_default(),
            machines_dir: s
                .machines_dir
                .as_ref()
                .map(|p| p.to_string_lossy().into_owned())
                .unwrap_or_default(),
            detect: None,
            found: None,
            picking: None,
        }
    }

    /// The settings the dialog would save.
    pub fn settings(&self) -> AmSettings {
        let path = |s: &str| {
            let t = s.trim();
            (!t.is_empty()).then(|| PathBuf::from(t))
        };
        let slicer = (!self.auto_detect).then(|| {
            let args: Vec<String> = self.args.split_whitespace().map(str::to_owned).collect();
            SlicerConfig {
                kind: self.kind,
                exe: PathBuf::from(self.exe.trim()),
                args: if args.is_empty() {
                    vec!["{file}".to_owned()]
                } else {
                    args
                },
                format: self.format,
            }
        });
        AmSettings {
            slicer,
            handoff_dir: path(&self.handoff_dir),
            machines_dir: path(&self.machines_dir),
        }
    }

    /// Uses a detected install.
    pub fn use_install(&mut self, i: &SlicerInstall) {
        let cfg = SlicerConfig::new(i.kind, i.exe.clone());
        self.auto_detect = false;
        self.kind = cfg.kind;
        self.exe = cfg.exe.to_string_lossy().into_owned();
        self.args = cfg.args.join(" ");
        self.format = cfg.format;
    }

    /// The installs found by *Detect* (once its job finished).
    pub fn detected(&self) -> Option<&[SlicerInstall]> {
        self.found.as_deref()
    }

    fn poll(&mut self) {
        if let Some(h) = &mut self.detect
            && let Some(r) = h.try_take()
        {
            self.detect = None;
            self.found = Some(r.unwrap_or_default());
        }
        if let Some((what, rx)) = &self.picking
            && let Ok(answer) = rx.try_recv()
        {
            let what = *what;
            self.picking = None;
            if let Some(p) = answer {
                let p = p.to_string_lossy().into_owned();
                match what {
                    PickFor::Exe => {
                        self.auto_detect = false;
                        if let Some(k) = amg_am::handoff::classify_exe(Path::new(&p)) {
                            self.kind = k;
                        }
                        self.exe = p;
                    }
                    PickFor::Handoff => self.handoff_dir = p,
                    PickFor::Machines => self.machines_dir = p,
                }
            }
        }
    }

    fn pick(&mut self, cx: &mut AppCx<'_>, what: PickFor) {
        let title = match what {
            PickFor::Exe => "Slicer executable",
            PickFor::Handoff => "Pick a file in the hand-off folder",
            PickFor::Machines => "Pick a file in the machine folder",
        };
        let rx = cx.pick_file(FileDialogRequest {
            kind: FileDialogKind::Open,
            title: title.to_owned(),
            filters: Vec::new(),
            directory: None,
            file_name: None,
        });
        self.picking = Some((what, rx));
    }

    fn form(&mut self, ui: &mut egui::Ui, cx: &mut AppCx<'_>) {
        ui.label(egui::RichText::new("Slicer").strong());
        ui.checkbox(&mut self.auto_detect, "Detect the slicer at each send");
        ui.add_enabled_ui(!self.auto_detect, |ui| {
            egui::ComboBox::from_label("Slicer")
                .selected_text(slicer_name(self.kind))
                .show_ui(ui, |ui| {
                    for k in KINDS {
                        ui.selectable_value(&mut self.kind, k, slicer_name(k));
                    }
                });
            ui.horizontal(|ui| {
                ui.label("Executable");
                ui.text_edit_singleline(&mut self.exe);
                if ui.button("Browse…").clicked() {
                    self.pick(cx, PickFor::Exe);
                }
            });
            ui.horizontal(|ui| {
                ui.label("Arguments");
                ui.text_edit_singleline(&mut self.args);
            });
            ui.horizontal(|ui| {
                ui.label("Format");
                ui.radio_value(&mut self.format, HandoffFormat::ThreeMf, "3MF");
                ui.radio_value(&mut self.format, HandoffFormat::Stl, "STL");
            });
        });
        ui.horizontal(|ui| {
            let busy = self.detect.is_some();
            if ui
                .add_enabled(!busy, egui::Button::new("Detect installed slicers"))
                .clicked()
            {
                self.detect = Some(cx.jobs().spawn_fn(
                    "Detect slicers",
                    amg_jobs::SpawnOptions::default(),
                    |_| Ok::<_, amg_am::AmError>(detect_slicers()),
                ));
            }
            if busy {
                ui.spinner();
            }
        });
        if let Some(found) = self.found.clone() {
            if found.is_empty() {
                ui.weak("No slicer found.");
            }
            for i in &found {
                let text = format!(
                    "Use {} {} — {}",
                    slicer_name(i.kind),
                    i.version.as_deref().unwrap_or(""),
                    i.exe.display()
                );
                if ui.button(text).clicked() {
                    self.use_install(i);
                }
            }
        }
        ui.separator();
        ui.label(egui::RichText::new("Folders").strong());
        ui.horizontal(|ui| {
            ui.label("Hand-off folder");
            ui.text_edit_singleline(&mut self.handoff_dir);
            if ui.button("Browse…").clicked() {
                self.pick(cx, PickFor::Handoff);
            }
        });
        ui.weak(format!("Default: {}", default_handoff_dir().display()));
        ui.horizontal(|ui| {
            ui.label("Machine folder");
            ui.text_edit_singleline(&mut self.machines_dir);
            if ui.button("Browse…").clicked() {
                self.pick(cx, PickFor::Machines);
            }
        });
        if let Some(d) = default_machines_dir() {
            ui.weak(format!("Default: {}", d.display()));
        }
    }
}

impl AppDialog for SettingsDialog {
    fn title(&self) -> Cow<'static, str> {
        Cow::Borrowed("Manufacture Settings")
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        self.poll();
        // A picked file stands for its folder.
        for dir in [&mut self.handoff_dir, &mut self.machines_dir] {
            let p = PathBuf::from(dir.trim());
            if p.is_file()
                && let Some(parent) = p.parent()
            {
                *dir = parent.to_string_lossy().into_owned();
            }
        }
        let action = DialogFrame::new("Manufacture Settings").show(ctx, |ui| self.form(ui, cx));
        match action {
            DialogAction::Ok => {
                save(self.settings());
                false
            }
            DialogAction::Cancel => false,
            DialogAction::None => true,
        }
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

impl SettingsDialog {
    #[cfg(test)]
    fn empty() -> SettingsDialog {
        SettingsDialog {
            auto_detect: true,
            kind: SlicerKind::OrcaSlicer,
            exe: String::new(),
            args: String::new(),
            format: HandoffFormat::ThreeMf,
            handoff_dir: String::new(),
            machines_dir: String::new(),
            detect: None,
            found: None,
            picking: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn settings_round_trip_and_defaults() {
        let s = AmSettings::default();
        assert!(s.slicer.is_none());
        assert_eq!(s.handoff_dir(), default_handoff_dir());
        let t = AmSettings {
            slicer: Some(SlicerConfig::new(
                SlicerKind::OrcaSlicer,
                "D:/x/orca-slicer.exe",
            )),
            handoff_dir: Some(PathBuf::from("D:/h")),
            machines_dir: None,
        };
        let json = serde_json::to_string(&t).unwrap_or_default();
        let back: AmSettings = serde_json::from_str(&json).unwrap_or_default();
        assert_eq!(back, t);
        // Unknown or missing fields are tolerated (forward compatible).
        let old: AmSettings = serde_json::from_str("{}").unwrap_or_default();
        assert_eq!(old, AmSettings::default());
    }

    #[test]
    fn dialog_turns_fields_into_settings() {
        let mut d = SettingsDialog::empty();
        assert!(d.settings().slicer.is_none());
        d.auto_detect = false;
        d.kind = SlicerKind::Chitubox;
        d.exe = " D:/CHITUBOX/CHITUBOX.exe ".into();
        d.args = String::new();
        d.format = HandoffFormat::Stl;
        d.handoff_dir = "D:/hand".into();
        let s = d.settings();
        let c = s
            .slicer
            .unwrap_or_else(|| SlicerConfig::new(SlicerKind::Custom, ""));
        assert_eq!(c.kind, SlicerKind::Chitubox);
        assert_eq!(c.exe, PathBuf::from("D:/CHITUBOX/CHITUBOX.exe"));
        assert_eq!(c.args, vec!["{file}".to_owned()]);
        assert_eq!(s.handoff_dir, Some(PathBuf::from("D:/hand")));
        assert_eq!(s.machines_dir, None);
    }
}
