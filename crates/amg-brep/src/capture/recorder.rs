//! Capture configuration, the dump format and `FailureRecorder` (dedup, retention, pending
//! write-ahead dumps).

use std::collections::HashMap;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime};

use amg_core::Tolerance;

use crate::capture::info::{Attempt, FailureInfo};
use crate::capture::op::KernelOp;
use crate::capture::{CaptureError, time};
use crate::shape::Shape;

/// Environment variable overriding [`CaptureConfig::mode`]: `off`, `on_failure`, `write_ahead`.
pub const ENV_CAPTURE: &str = "AMG_KERNEL_CAPTURE";
/// Environment variable overriding [`CaptureConfig::dir`].
pub const ENV_CAPTURE_DIR: &str = "AMG_KERNEL_CAPTURE_DIR";
/// The manifest schema this build writes and reads.
pub const MANIFEST_SCHEMA: u32 = 1;

const PENDING: &str = "pending";
const MANIFEST: &str = "manifest.json";
const LOCK: &str = "lock";

/// When [`crate::capture::RecordingKernel`] writes dumps.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptureMode {
    /// Never.
    Off,
    /// When an op fails; nothing on success (owner default).
    #[default]
    OnFailure,
    /// Also before every op, to `pending/`, deleted when the op returns; catches crashes and
    /// hangs (debug).
    WriteAhead,
}

impl CaptureMode {
    /// Parses `off`, `on_failure` or `write_ahead` (ASCII case-insensitive; `-` for `_`).
    pub fn parse(text: &str) -> Option<CaptureMode> {
        match text.trim().to_ascii_lowercase().replace('-', "_").as_str() {
            "off" | "0" | "false" => Some(CaptureMode::Off),
            "on_failure" | "on" | "1" => Some(CaptureMode::OnFailure),
            "write_ahead" => Some(CaptureMode::WriteAhead),
            _ => None,
        }
    }
}

/// Where and how dumps are written.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CaptureConfig {
    /// The mode.
    pub mode: CaptureMode,
    /// The capture directory.
    pub dir: PathBuf,
    /// Retention: total size of all dumps, bytes (default 1 GiB).
    pub max_bytes: u64,
    /// Retention: number of dumps (default 1000).
    pub max_dumps: u32,
}

impl CaptureConfig {
    /// `OnFailure` into `dir` with the default limits.
    pub fn in_dir(dir: impl Into<PathBuf>) -> CaptureConfig {
        CaptureConfig {
            mode: CaptureMode::OnFailure,
            dir: dir.into(),
            max_bytes: 1 << 30,
            max_dumps: 1000,
        }
    }

    /// The default directory (`%LOCALAPPDATA%\Amalgam\kernel-failures`; elsewhere
    /// `$XDG_STATE_HOME/amalgam/kernel-failures` or `~/.local/state/amalgam/kernel-failures`),
    /// overridden by [`ENV_CAPTURE_DIR`], and the mode from [`ENV_CAPTURE`] (an unknown value
    /// is logged and ignored).
    pub fn from_env() -> CaptureConfig {
        let dir = std::env::var_os(ENV_CAPTURE_DIR)
            .filter(|d| !d.is_empty())
            .map_or_else(default_dir, PathBuf::from);
        let mut cfg = CaptureConfig::in_dir(dir);
        if let Ok(text) = std::env::var(ENV_CAPTURE) {
            match CaptureMode::parse(&text) {
                Some(mode) => cfg.mode = mode,
                None => tracing::warn!("{ENV_CAPTURE}={text:?} is not a capture mode; ignored"),
            }
        }
        cfg
    }
}

fn default_dir() -> PathBuf {
    let var = |k: &str| {
        std::env::var_os(k)
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
    };
    if let Some(local) = var("LOCALAPPDATA") {
        return local.join("Amalgam").join("kernel-failures");
    }
    if let Some(state) = var("XDG_STATE_HOME") {
        return state.join("amalgam").join("kernel-failures");
    }
    if let Some(home) = var("HOME") {
        return home.join(".local/state/amalgam/kernel-failures");
    }
    std::env::temp_dir().join("amalgam-kernel-failures")
}

/// A dump's directory name.
pub type DumpId = String;

/// Where a failure came from in the document (no file paths, no names).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DumpContext {
    /// The feature kind, for example `fillet`.
    pub feature: Option<String>,
    /// The feature's `PersistentId`, as text.
    pub persistent_id: Option<String>,
}

/// `manifest.json`, schema 1 (design note §1.1). Unknown fields are rejected.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    /// Always [`MANIFEST_SCHEMA`].
    pub schema: u32,
    /// blake3 (hex) over the canonical op JSON and the input `.brep` bytes: the dedup key.
    pub hash: String,
    /// `amg-brep` package version.
    pub amalgam_version: String,
    /// The build's git commit (`AMG_GIT_SHA` at build time), if known.
    pub git_sha: Option<String>,
    /// `occt-<version>` or `occt-none`.
    pub occt_version: String,
    /// [`crate::kernel_version`].
    pub kernel_version: String,
    /// [`crate::POLICY_VERSION`].
    pub policy_version: u32,
    /// `std::env::consts::OS`.
    pub os: String,
    /// First occurrence, RFC 3339 UTC.
    pub created_utc: String,
    /// Latest occurrence, RFC 3339 UTC.
    pub last_utc: String,
    /// The mode that wrote it.
    pub mode: CaptureMode,
    /// How long the failing call ran, ms (0 for crashes).
    pub duration_ms: u64,
    /// How often this failure (same hash) occurred.
    pub count: u32,
    /// The kernel tolerance in force, when the recording kernel knows it.
    pub tolerance: Option<Tolerance>,
    /// The `RetryPolicy` in force (KH-17; `None` for raw calls).
    pub retry_policy: Option<String>,
    /// The op, with shape arguments as [`crate::capture::InputRef`]s.
    pub op: KernelOp,
    /// The input files, relative to the dump: `inputs/<n>.brep`.
    pub inputs: Vec<String>,
    /// The failure.
    pub error: FailureInfo,
    /// Strategy attempts (KH-17); empty for a direct call.
    pub attempts: Vec<Attempt>,
    /// Document context, if the caller supplied it.
    pub context: Option<DumpContext>,
}

/// Reads and validates `<dump_dir>/manifest.json` against schema 1.
///
/// # Errors
/// `Io`; `Version` for another schema number; `Schema` for anything that does not match.
pub fn read_manifest(dump_dir: &Path) -> Result<Manifest, CaptureError> {
    let text = fs::read_to_string(dump_dir.join(MANIFEST))?;
    let value: serde_json::Value =
        serde_json::from_str(&text).map_err(|e| CaptureError::Schema(e.to_string()))?;
    let schema = value
        .get("schema")
        .and_then(serde_json::Value::as_u64)
        .ok_or_else(|| CaptureError::Schema("`schema` is missing".into()))?;
    if schema != u64::from(MANIFEST_SCHEMA) {
        return Err(CaptureError::Version {
            found: u32::try_from(schema).unwrap_or(u32::MAX),
        });
    }
    let manifest: Manifest =
        serde_json::from_value(value).map_err(|e| CaptureError::Schema(e.to_string()))?;
    if manifest.inputs.len() != manifest.op.inputs().len() {
        return Err(CaptureError::Schema(format!(
            "{} input files for an op with {} shape arguments",
            manifest.inputs.len(),
            manifest.op.inputs().len()
        )));
    }
    Ok(manifest)
}

/// An op and its inputs, serialised once.
pub(crate) struct Captured {
    pub(crate) op: KernelOp,
    pub(crate) inputs: Vec<Vec<u8>>,
    pub(crate) hash: String,
}

fn to_io(e: impl std::fmt::Display) -> io::Error {
    io::Error::other(e.to_string())
}

impl Captured {
    /// Serialises `op` and writes every input as text `.brep`.
    pub(crate) fn new(op: KernelOp, inputs: &[&Shape]) -> io::Result<Captured> {
        let mut bytes = Vec::with_capacity(inputs.len());
        for s in inputs {
            let mut data = Vec::new();
            s.write_text(&mut data).map_err(to_io)?;
            bytes.push(data);
        }
        Captured::from_parts(op, bytes)
    }

    fn from_parts(op: KernelOp, inputs: Vec<Vec<u8>>) -> io::Result<Captured> {
        let json = serde_json::to_vec(&op).map_err(to_io)?;
        let mut h = blake3::Hasher::new();
        h.update(&(json.len() as u64).to_le_bytes());
        h.update(&json);
        for data in &inputs {
            h.update(&(data.len() as u64).to_le_bytes());
            h.update(data);
        }
        Ok(Captured {
            op,
            inputs,
            hash: h.finalize().to_hex().to_string(),
        })
    }

    pub(crate) fn manifest(
        &self,
        error: FailureInfo,
        attempts: &[Attempt],
        mode: CaptureMode,
        duration: Duration,
        tolerance: Option<Tolerance>,
    ) -> Manifest {
        let now = time::rfc3339(time::now());
        Manifest {
            schema: MANIFEST_SCHEMA,
            hash: self.hash.clone(),
            amalgam_version: env!("CARGO_PKG_VERSION").into(),
            git_sha: option_env!("AMG_GIT_SHA").map(str::to_owned),
            occt_version: crate::backend::occt_version(),
            kernel_version: crate::kernel_version().into(),
            policy_version: crate::POLICY_VERSION,
            os: std::env::consts::OS.into(),
            created_utc: now.clone(),
            last_utc: now,
            mode,
            duration_ms: u64::try_from(duration.as_millis()).unwrap_or(u64::MAX),
            count: 1,
            tolerance,
            retry_policy: None,
            op: self.op.clone(),
            inputs: (0..self.inputs.len())
                .map(|i| format!("inputs/{i}.brep"))
                .collect(),
            error,
            attempts: attempts.to_vec(),
            context: None,
        }
    }
}

/// Writes dumps: dedup by hash (a repeat increments `count`), retention (oldest first, by
/// the manifest's modification time, so a recurring failure stays), write-ahead pending dumps.
/// `Send + Sync`; writes are serialised by an internal lock.
#[derive(Debug)]
pub struct FailureRecorder {
    cfg: CaptureConfig,
    /// hash → dump id, loaded from the directory on the first write.
    index: Mutex<Option<HashMap<String, DumpId>>>,
}

static_assertions::assert_impl_all!(FailureRecorder: Send, Sync);

/// Distinguishes pending dumps of concurrent calls.
static PENDING_SEQ: AtomicU64 = AtomicU64::new(0);

fn write_json(path: &Path, m: &Manifest) -> io::Result<()> {
    let text = serde_json::to_vec_pretty(m).map_err(to_io)?;
    let tmp = path.with_extension("json.tmp");
    fs::write(&tmp, text)?;
    fs::rename(&tmp, path)
}

fn dir_size(path: &Path) -> u64 {
    let Ok(entries) = fs::read_dir(path) else {
        return 0;
    };
    entries
        .flatten()
        .map(|e| match e.metadata() {
            Ok(m) if m.is_dir() => dir_size(&e.path()),
            Ok(m) => m.len(),
            Err(_) => 0,
        })
        .sum()
}

impl FailureRecorder {
    /// A recorder for `cfg`. Touches no file: the directory is created by the first dump.
    ///
    /// # Errors
    /// `InvalidInput` for an empty directory path.
    pub fn new(cfg: CaptureConfig) -> io::Result<FailureRecorder> {
        if cfg.dir.as_os_str().is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "capture directory is empty",
            ));
        }
        Ok(FailureRecorder {
            cfg,
            index: Mutex::new(None),
        })
    }

    /// The configuration.
    pub fn config(&self) -> &CaptureConfig {
        &self.cfg
    }

    /// Records a failure of `op` on `inputs` (in [`crate::capture::InputRef`] order) and
    /// returns the dump's id; a failure with the same hash as an existing dump increments its
    /// `count` instead.
    ///
    /// # Errors
    /// File system errors; `Other` when an input cannot be serialised.
    pub fn record(
        &self,
        op: &KernelOp,
        inputs: &[&Shape],
        err: &FailureInfo,
        attempts: &[Attempt],
    ) -> io::Result<DumpId> {
        let captured = Captured::new(op.clone(), inputs)?;
        let manifest =
            captured.manifest(err.clone(), attempts, self.cfg.mode, Duration::ZERO, None);
        self.store(&captured, manifest)
    }

    /// The dump directories (not `pending/`), sorted by name.
    ///
    /// # Errors
    /// File system errors other than a missing capture directory.
    pub fn dumps(&self) -> io::Result<Vec<PathBuf>> {
        let entries = match fs::read_dir(&self.cfg.dir) {
            Ok(entries) => entries,
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(Vec::new()),
            Err(e) => return Err(e),
        };
        let mut dumps: Vec<PathBuf> = entries
            .flatten()
            .filter(|e| e.file_type().is_ok_and(|t| t.is_dir()))
            .filter(|e| {
                let name = e.file_name();
                let name = name.to_string_lossy();
                name != PENDING && !name.starts_with('.')
            })
            .map(|e| e.path())
            .collect();
        dumps.sort();
        Ok(dumps)
    }

    fn load_index(&self) -> HashMap<String, DumpId> {
        let mut index = HashMap::new();
        for dir in self.dumps().unwrap_or_default() {
            let Ok(text) = fs::read_to_string(dir.join(MANIFEST)) else {
                continue;
            };
            let Ok(value) = serde_json::from_str::<serde_json::Value>(&text) else {
                continue;
            };
            if let (Some(hash), Some(name)) = (
                value.get("hash").and_then(serde_json::Value::as_str),
                dir.file_name(),
            ) {
                index.insert(hash.to_owned(), name.to_string_lossy().into_owned());
            }
        }
        index
    }

    /// Writes `m` (and the inputs of `c`) as a new dump, or bumps the count of the dump with
    /// the same hash.
    pub(crate) fn store(&self, c: &Captured, m: Manifest) -> io::Result<DumpId> {
        let mut guard = self
            .index
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let index = guard.get_or_insert_with(|| self.load_index());
        if let Some(id) = index.get(&c.hash).cloned() {
            let path = self.cfg.dir.join(&id).join(MANIFEST);
            if let Ok(mut existing) = read_manifest(&self.cfg.dir.join(&id)) {
                existing.count = existing.count.saturating_add(m.count.max(1));
                existing.last_utc = m.last_utc;
                write_json(&path, &existing)?;
                return Ok(id);
            }
            index.remove(&c.hash);
        }
        fs::create_dir_all(&self.cfg.dir)?;
        let stem = format!(
            "{}_{}_{}",
            time::compact(time::now()),
            c.op.kind(),
            &c.hash[..8]
        );
        let mut name = stem.clone();
        let mut n = 1;
        while self.cfg.dir.join(&name).exists() {
            name = format!("{stem}-{n}");
            n += 1;
        }
        let tmp = self
            .cfg
            .dir
            .join(format!(".tmp-{name}-{}", std::process::id()));
        write_dump(&tmp, c, &m)?;
        fs::rename(&tmp, self.cfg.dir.join(&name))?;
        index.insert(c.hash.clone(), name.clone());
        self.prune(index, &name);
        Ok(name)
    }

    /// Removes the least recently seen dumps until both limits hold; `keep` is never removed.
    fn prune(&self, index: &mut HashMap<String, DumpId>, keep: &str) {
        let Ok(dumps) = self.dumps() else { return };
        let mut entries: Vec<(SystemTime, String, u64)> = dumps
            .iter()
            .filter_map(|d| {
                let name = d.file_name()?.to_string_lossy().into_owned();
                let mtime = fs::metadata(d.join(MANIFEST))
                    .and_then(|m| m.modified())
                    .unwrap_or(SystemTime::UNIX_EPOCH);
                Some((mtime, name, dir_size(d)))
            })
            .collect();
        entries.sort();
        let mut count = entries.len() as u64;
        let mut bytes: u64 = entries.iter().map(|e| e.2).sum();
        for (_, name, size) in entries {
            if count <= u64::from(self.cfg.max_dumps) && bytes <= self.cfg.max_bytes {
                break;
            }
            if name == keep {
                continue;
            }
            match fs::remove_dir_all(self.cfg.dir.join(&name)) {
                Ok(()) => {
                    count -= 1;
                    bytes = bytes.saturating_sub(size);
                    index.retain(|_, id| *id != name);
                }
                Err(e) => tracing::warn!("kernel capture: cannot prune {name}: {e}"),
            }
        }
    }

    /// Writes a write-ahead dump to `pending/` and holds its lock until the returned guard is
    /// finished or dropped.
    pub(crate) fn begin(&self, c: Captured, m: Manifest) -> io::Result<Pending> {
        let seq = PENDING_SEQ.fetch_add(1, Ordering::Relaxed);
        let dir = self.cfg.dir.join(PENDING).join(format!(
            "{}-{}-{seq}",
            &c.hash[..8],
            std::process::id()
        ));
        fs::create_dir_all(&dir)?;
        let lock = fs::File::create(dir.join(LOCK))?;
        lock.lock()?;
        // The manifest goes last: a pending dump with a manifest is complete.
        write_dump(&dir, &c, &m)?;
        Ok(Pending {
            dir,
            lock: Some(lock),
            captured: c,
        })
    }

    /// Moves write-ahead dumps left behind by a process that ended inside an op (a crash, a
    /// kill during a hang) into the capture directory as `Crash` dumps; call at startup.
    /// Pending dumps whose lock is still held (an op running in another process) are left
    /// alone. Returns how many were recovered.
    ///
    /// # Errors
    /// File system errors other than a missing `pending/` directory.
    pub fn recover_pending(&self) -> io::Result<usize> {
        let pending = self.cfg.dir.join(PENDING);
        let entries = match fs::read_dir(&pending) {
            Ok(entries) => entries,
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(0),
            Err(e) => return Err(e),
        };
        let mut recovered = 0;
        for entry in entries.flatten() {
            let dir = entry.path();
            if !dir.is_dir() {
                continue;
            }
            if let Ok(lock) = fs::OpenOptions::new().write(true).open(dir.join(LOCK)) {
                match lock.try_lock() {
                    Ok(()) => drop(lock),
                    Err(fs::TryLockError::WouldBlock) => continue,
                    Err(fs::TryLockError::Error(e)) => return Err(e),
                }
            }
            match read_pending(&dir) {
                Ok((c, m)) => {
                    self.store(&c, m)?;
                    recovered += 1;
                }
                Err(e) => tracing::warn!(
                    "kernel capture: dropping incomplete pending dump {}: {e}",
                    dir.display()
                ),
            }
            if let Err(e) = fs::remove_dir_all(&dir) {
                tracing::warn!("kernel capture: cannot remove {}: {e}", dir.display());
            }
        }
        Ok(recovered)
    }
}

fn write_dump(dir: &Path, c: &Captured, m: &Manifest) -> io::Result<()> {
    fs::create_dir_all(dir.join("inputs"))?;
    for (file, data) in m.inputs.iter().zip(&c.inputs) {
        fs::write(dir.join(file), data)?;
    }
    write_json(&dir.join(MANIFEST), m)
}

fn read_pending(dir: &Path) -> Result<(Captured, Manifest), CaptureError> {
    let m = read_manifest(dir)?;
    let inputs = m
        .inputs
        .iter()
        .map(|f| fs::read(dir.join(f)))
        .collect::<io::Result<Vec<_>>>()?;
    let c = Captured::from_parts(m.op.clone(), inputs)?;
    Ok((c, m))
}

/// A write-ahead dump of a running call.
pub(crate) struct Pending {
    dir: PathBuf,
    lock: Option<fs::File>,
    pub(crate) captured: Captured,
}

impl Pending {
    /// Deletes the pending dump.
    pub(crate) fn finish(mut self) {
        self.remove();
    }

    fn remove(&mut self) {
        // Windows cannot delete a directory with an open file in it.
        drop(self.lock.take());
        if let Err(e) = fs::remove_dir_all(&self.dir) {
            tracing::warn!(
                "kernel capture: cannot remove pending {}: {e}",
                self.dir.display()
            );
        }
    }
}

impl Drop for Pending {
    fn drop(&mut self) {
        if self.lock.is_some() {
            self.remove();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modes_parse() {
        assert_eq!(CaptureMode::parse("off"), Some(CaptureMode::Off));
        assert_eq!(
            CaptureMode::parse(" On_Failure "),
            Some(CaptureMode::OnFailure)
        );
        assert_eq!(
            CaptureMode::parse("write-ahead"),
            Some(CaptureMode::WriteAhead)
        );
        assert_eq!(CaptureMode::parse("sometimes"), None);
        assert_eq!(CaptureMode::default(), CaptureMode::OnFailure);
        assert_eq!(
            serde_json::to_string(&CaptureMode::WriteAhead).unwrap(),
            "\"write_ahead\""
        );
    }

    #[test]
    fn default_limits() {
        let cfg = CaptureConfig::in_dir("x");
        assert_eq!(cfg.max_bytes, 1 << 30);
        assert_eq!(cfg.max_dumps, 1000);
        assert_eq!(cfg.mode, CaptureMode::OnFailure);
        assert!(FailureRecorder::new(CaptureConfig::in_dir("")).is_err());
    }
}
