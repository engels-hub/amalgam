//! Kernel failure capture (S20 KH-01, design note `kernel-robustness.md` §1).
//!
//! [`RecordingKernel`] wraps any [`crate::BrepKernel`] and forwards every call. When a
//! shape-producing op fails it writes a **dump** through a [`FailureRecorder`]:
//!
//! ```text
//! <capture dir>/                        # default %LOCALAPPDATA%\Amalgam\kernel-failures
//!   20261020T140311Z_boolean_3f2a9c1e/  # UTC time, op kind, first 8 hex of the dump hash
//!     manifest.json                     # schema 1, see [`Manifest`]
//!     inputs/0.brep inputs/1.brep …     # BRepTools text, no triangulation
//!   pending/                            # write-ahead dumps of calls still running
//! ```
//!
//! [`replay`] re-runs a dump and classifies the result ([`ReplayOutcome`]); [`replay_in_child`]
//! does so in a separate process so that a crash is a result, not the end of the run;
//! [`promote`] turns a dump into a corpus case (`case.toml` + `op.json` + `inputs/`).
//!
//! Modes ([`CaptureMode`]): `OnFailure` (the default) does nothing on success: the inputs are
//! borrowed for the call anyway, so no clone, no serialisation and no file I/O happens unless
//! the op fails. `WriteAhead` writes the op and its inputs to `pending/` before the call and
//! deletes them after it, so a hard crash or a killed hang leaves a dump behind that
//! [`FailureRecorder::recover_pending`] turns into a `Crash` dump on the next start.

mod info;
mod op;
mod recorder;
mod recording;
mod replay;
mod time;

pub use info::{Attempt, FailureClass, FailureInfo};
pub use op::{EdgeRecord, InputRef, KernelOp, SubShapeGeom};
pub use recorder::{
    CaptureConfig, CaptureMode, DumpContext, DumpId, ENV_CAPTURE, ENV_CAPTURE_DIR, FailureRecorder,
    MANIFEST_SCHEMA, Manifest, read_manifest,
};
pub use recording::RecordingKernel;
pub use replay::{
    REPORT_PREFIX, ReplayOpts, ReplayOutcome, ReplayReport, child_report_line, promote, replay,
    replay_in_child,
};

/// Why reading, replaying or promoting a dump failed.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum CaptureError {
    /// File system access failed.
    #[error("capture I/O: {0}")]
    Io(#[from] std::io::Error),
    /// `manifest.json` (or `op.json`) does not match the schema.
    #[error("dump does not match the manifest schema: {0}")]
    Schema(String),
    /// The manifest has a schema version this build cannot read.
    #[error("manifest schema {found} is not supported (this build reads {MANIFEST_SCHEMA})")]
    Version {
        /// The manifest's `schema`.
        found: u32,
    },
    /// Loading an input shape failed.
    #[error("kernel: {0}")]
    Kernel(#[from] crate::KernelError),
    /// An edge of the op neither keeps its index nor matches a unique edge by fingerprint.
    #[error("edge {edge} of input {input} has no unique match in the reloaded shape")]
    Unmatched {
        /// The input.
        input: u32,
        /// The recorded edge index.
        edge: u32,
    },
    /// A replay child process gave no report.
    #[error("replay child: {0}")]
    Child(String),
    /// A corpus id is empty or has components other than `[A-Za-z0-9._-]+`.
    #[error("invalid corpus id `{0}`")]
    InvalidId(String),
}

/// The current time as RFC 3339 UTC (`2026-11-28T14:03:11Z`), for reports.
pub(crate) fn time_rfc3339_now() -> String {
    time::rfc3339(time::now())
}
