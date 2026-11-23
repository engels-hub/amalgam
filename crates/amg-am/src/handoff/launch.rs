//! Starting the slicer, and the job that exports and starts it.

use std::borrow::Cow;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::SystemTime;

use amg_jobs::{Job, JobCtx};
use serde::{Deserialize, Serialize};

use super::export::{
    HandoffFormat, HandoffItem, HandoffPlate, HandoffReport, export_plate, handoff_path,
    prune_handoff_dir,
};
use crate::AmError;
use crate::machine::{Machine, SlicerKind};

/// How to start a slicer.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SlicerConfig {
    /// Which slicer (picks the 3MF flavour).
    pub kind: SlicerKind,
    /// The executable.
    pub exe: PathBuf,
    /// Arguments; `{file}` is replaced by the hand-off file. Without a `{file}` the file is
    /// appended as the last argument. Default: `["{file}"]`.
    pub args: Vec<String>,
    /// File format to hand over.
    pub format: HandoffFormat,
}

impl SlicerConfig {
    /// Default arguments and format for `kind`: 3MF, except Chitubox, which gets STL until the
    /// owner's AM-06 check shows that its 3MF import keeps positions (QUESTIONS #32).
    pub fn new(kind: SlicerKind, exe: impl Into<PathBuf>) -> Self {
        Self {
            kind,
            exe: exe.into(),
            args: vec!["{file}".to_owned()],
            format: match kind {
                SlicerKind::Chitubox => HandoffFormat::Stl,
                _ => HandoffFormat::ThreeMf,
            },
        }
    }

    fn command_args(&self, file: &Path) -> Vec<String> {
        let f = file.to_string_lossy();
        let mut used = false;
        let mut args: Vec<String> = self
            .args
            .iter()
            .map(|a| {
                if a.contains("{file}") {
                    used = true;
                }
                a.replace("{file}", &f)
            })
            .collect();
        if !used {
            args.push(f.into_owned());
        }
        args
    }
}

/// Starts the slicer on `file` and returns at once with the child's pid. The process is
/// detached (own process group, no console, no inherited handles) and never waited on by the
/// caller; a helper thread only reaps it.
///
/// # Errors
/// [`AmError::SlicerNotFound`] if the executable does not exist, [`AmError::Launch`] if the
/// OS refuses to start it.
pub fn launch(cfg: &SlicerConfig, file: &Path) -> Result<u32, AmError> {
    if !cfg.exe.is_file() {
        return Err(AmError::SlicerNotFound(cfg.exe.clone()));
    }
    let mut cmd = Command::new(&cfg.exe);
    cmd.args(cfg.command_args(file))
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        const DETACHED_PROCESS: u32 = 0x0000_0008;
        const CREATE_NEW_PROCESS_GROUP: u32 = 0x0000_0200;
        cmd.creation_flags(DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP);
    }
    let mut child = cmd
        .spawn()
        .map_err(|e| AmError::Launch(format!("{}: {e}", cfg.exe.display())))?;
    let pid = child.id();
    // Reap on a helper thread so a finished slicer does not linger as a zombie; the caller
    // never waits. If the thread cannot start, the child is simply left alone.
    let _ = std::thread::Builder::new()
        .name("amg-am-reap".into())
        .spawn(move || {
            let _ = child.wait();
        });
    Ok(pid)
}

/// Result of [`send_to_slicer`].
#[derive(Clone, Debug)]
pub struct SendReport {
    /// The exported file.
    pub handoff: HandoffReport,
    /// Pid of the started slicer.
    pub pid: u32,
    /// Old hand-off files deleted.
    pub pruned: usize,
}

/// Files kept in the hand-off directory.
pub const KEEP_HANDOFF_FILES: usize = 20;

/// Export into `dir`, prune it to the newest [`KEEP_HANDOFF_FILES`], start the slicer.
/// Blocking: call it from a job ([`SendToSlicerJob`]), never from the UI thread.
///
/// The slicer is checked first, so a wrong path fails before any file is written; a cancel
/// after the export deletes nothing but also starts nothing.
///
/// # Errors
/// Anything [`export_plate`] or [`launch`] returns.
pub fn send_to_slicer(
    plate: &HandoffPlate<'_>,
    cfg: &SlicerConfig,
    dir: &Path,
    ctx: &JobCtx,
) -> Result<SendReport, AmError> {
    if !cfg.exe.is_file() {
        return Err(AmError::SlicerNotFound(cfg.exe.clone()));
    }
    let path = handoff_path(
        dir,
        &plate.design_name,
        &plate.plate_name,
        SystemTime::now(),
        cfg.format,
    );
    let handoff = export_plate(plate, cfg.format, cfg.kind, &path, &ctx.sub(0.0, 0.95))?;
    // A prune failure (a file held open by the slicer) must not stop the hand-off.
    let pruned = prune_handoff_dir(dir, KEEP_HANDOFF_FILES).unwrap_or(0);
    ctx.checkpoint()?;
    let pid = launch(cfg, &handoff.path)?;
    ctx.progress(1.0);
    Ok(SendReport {
        handoff,
        pid,
        pruned,
    })
}

/// The "Send to Slicer" job: owns what it needs (the machine is cloned, meshes are shared
/// `Arc`s), so the UI can spawn it and return immediately.
#[derive(Debug)]
pub struct SendToSlicerJob {
    design_name: String,
    plate_name: String,
    machine: Machine,
    items: Vec<HandoffItem>,
    thumbnail_png: Option<Vec<u8>>,
    cfg: SlicerConfig,
    dir: PathBuf,
}

impl SendToSlicerJob {
    /// Captures `plate` for a job.
    pub fn new(plate: &HandoffPlate<'_>, cfg: SlicerConfig, dir: PathBuf) -> Self {
        Self {
            design_name: plate.design_name.clone(),
            plate_name: plate.plate_name.clone(),
            machine: plate.machine.clone(),
            items: plate.items.clone(),
            thumbnail_png: plate.thumbnail_png.clone(),
            cfg,
            dir,
        }
    }
}

impl Job for SendToSlicerJob {
    type Output = SendReport;
    type Error = AmError;

    fn label(&self) -> Cow<'static, str> {
        Cow::Owned(format!("Send to {:?}", self.cfg.kind))
    }

    fn run(self, ctx: &JobCtx) -> Result<SendReport, AmError> {
        let plate = HandoffPlate {
            design_name: self.design_name,
            plate_name: self.plate_name,
            machine: &self.machine,
            items: self.items,
            thumbnail_png: self.thumbnail_png,
        };
        send_to_slicer(&plate, &self.cfg, &self.dir, ctx)
    }
}
