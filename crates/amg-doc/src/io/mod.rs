//! The `.amg` container (S01 §8, DOC-07): a zip with `manifest.json`, `design.json`,
//! `bodies/…`, `ext/…` and an optional `thumbnail.png`.
//!
//! - [`save`] / [`load`] are synchronous; [`save_job`] / [`load_job`] run them on the
//!   [`amg_jobs::JobSystem`] (progress and cancellation).
//! - Saving is safe: the file is written to `<name>.amg.tmp`, flushed and `sync_all`ed, the old
//!   file is copied to `<name>.amg.bak`, and only then is the temporary renamed over the target.
//!   A crash at any point leaves the previous file intact ([`SavePhase`] is the test hook).
//! - `design.json` is deterministic (sorted maps, no timestamps): saving, loading and saving again
//!   gives byte-identical bytes.
//! - Loading checks, in order: it is a zip with an Amalgam manifest ([`FormatError::NotAmalgam`]),
//!   the file is not newer than this build ([`FormatError::TooNew`]), every entry's CRC and
//!   manifest blake3 ([`FormatError::Corrupt`]), then migrates the raw JSON through the document
//!   chain ([`DOC_MIGRATIONS`]) and deserializes it with the JSON path of any error
//!   ([`FormatError::Json`]). Feature data migrates per kind in the registry.
//! - Geometry is read and written through a [`BodyCodec`]. Until the stub geometry types are
//!   replaced by the real ones, [`StubBodyCodec`] is used; see the trait for the seam.
//!
//! Not here: geometry-cache persistence and autosave/recovery (DOC-14).

mod read;
mod write;

use std::collections::BTreeMap;
use std::fmt::Debug;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use amg_jobs::{Cancelled, JobHandle, JobSystem, SpawnOptions};
use serde::{Deserialize, Serialize};

use crate::eval::BodyGeom;
use crate::feature::MigrationError;
use crate::model::{BlobRef, Design, DesignSnapshot};
use crate::stub::{IndexedMesh, SdfGraph, Shape, SubdMesh};

pub use read::{load, load_in_job, load_with_codec, migrate_document};
pub use write::{save, save_in_job};

/// Document format version written by this build. `design.json` of older versions is migrated
/// by [`DOC_MIGRATIONS`]; newer ones are refused with [`FormatError::TooNew`].
pub const FORMAT_VERSION: u32 = 1;

/// Value of `manifest.json`'s `format` field.
pub const FORMAT_NAME: &str = "amalgam";

/// One document-level migration step: raw `design.json` of version `i` to version `i + 1`.
pub type DocMigration = fn(serde_json::Value) -> Result<serde_json::Value, MigrationError>;

/// The document migration chain. Element `i` migrates version `i` to `i + 1`, so the length is
/// [`FORMAT_VERSION`]. Every step has a never-edited fixture `tests/fixtures/format/v<i>.amg`.
pub const DOC_MIGRATIONS: &[DocMigration] = &[migrate_v0_to_v1];

/// v0 (pre-freeze) kept the default units under `meta.units`; v1 calls them
/// `meta.default_units`.
fn migrate_v0_to_v1(mut raw: serde_json::Value) -> Result<serde_json::Value, MigrationError> {
    let meta = raw
        .get_mut("meta")
        .and_then(serde_json::Value::as_object_mut)
        .ok_or_else(|| MigrationError("v0 design has no `meta` object".to_owned()))?;
    let units = meta
        .remove("units")
        .ok_or_else(|| MigrationError("v0 `meta` has no `units`".to_owned()))?;
    meta.insert("default_units".to_owned(), units);
    Ok(raw)
}

/// Why a `.amg` file could not be saved or loaded.
#[derive(thiserror::Error, Debug)]
#[non_exhaustive]
pub enum FormatError {
    /// Not a zip, or a zip without an Amalgam manifest.
    #[error("not an Amalgam file")]
    NotAmalgam,
    /// Written by a newer build. The file is left untouched.
    #[error("file format {file} is newer than supported {supported}; update Amalgam")]
    TooNew {
        /// Version in the file.
        file: u32,
        /// Version this build reads.
        supported: u32,
    },
    /// Truncated or damaged: zip directory, CRC or blake3 mismatch, or a missing entry.
    #[error("file is corrupt ({entry})")]
    Corrupt {
        /// The damaged or missing entry (`"zip directory"` for a truncated archive).
        entry: String,
    },
    /// `design.json` does not parse or does not fit the model.
    #[error("invalid design data at {path}: {msg}")]
    Json {
        /// JSON path of the failing value (`timeline.nodes[12].data.extent`; `.` for the root or
        /// a syntax error).
        path: String,
        /// The parser's message.
        msg: String,
    },
    /// A document migration step failed.
    #[error("migration v{from}→v{to} failed: {msg}")]
    Migration {
        /// Version being migrated from.
        from: u32,
        /// Version it should become.
        to: u32,
        /// The step's message.
        msg: String,
    },
    /// A [`BlobRef`] is not a valid container path of its kind.
    #[error("invalid blob path `{0}`")]
    InvalidBlobPath(String),
    /// A referenced blob has no data (not in the container on load, not provided on save).
    #[error("missing data for blob `{0}`")]
    MissingBlob(String),
    /// The [`BodyCodec`] refused a body.
    #[error("body `{blob}`: {msg}")]
    Codec {
        /// The body's blob path.
        blob: String,
        /// The codec's message.
        msg: String,
    },
    /// The job was cancelled.
    #[error("cancelled")]
    Cancelled,
    /// The save was stopped by the test hook ([`SaveOptions::stop_after`]).
    #[error("save stopped after {0:?} (test hook)")]
    Stopped(SavePhase),
    /// File-system error.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// Zip error not covered above.
    #[error(transparent)]
    Zip(#[from] zip::result::ZipError),
}

impl From<Cancelled> for FormatError {
    fn from(_: Cancelled) -> Self {
        FormatError::Cancelled
    }
}

impl amg_core::Diagnostic for FormatError {}

/// Error of a [`BodyCodec`].
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error("{0}")]
pub struct BodyCodecError(pub String);

/// Serializes baked body geometry to and from `bodies/` entries.
///
/// **Seam.** `BodyGeom` still holds `amg-doc::stub` placeholders, so [`StubBodyCodec`] only
/// records the geometry kind. When MESH-02 replaces the stub `IndexedMesh`, the real codec
/// writes meshes with `amg_mesh::MeshBody::write_binary` and reads them with `read_binary`
/// (versioned, checksummed, bit-exact; the container round trip of those bytes is tested in
/// `tests/format.rs`), and `.brep` for solids (BREP-02).
pub trait BodyCodec: Send + Sync + Debug {
    /// Encodes `geom`, to be stored at `blob`.
    fn encode(&self, blob: &BlobRef, geom: &BodyGeom) -> Result<Vec<u8>, BodyCodecError>;
    /// Decodes the bytes stored at `blob`.
    fn decode(&self, blob: &BlobRef, bytes: &[u8]) -> Result<BodyGeom, BodyCodecError>;
}

/// Codec for the stub geometry types: stores a tag per kind and nothing else.
#[derive(Clone, Copy, Debug, Default)]
pub struct StubBodyCodec;

impl BodyCodec for StubBodyCodec {
    fn encode(&self, _blob: &BlobRef, geom: &BodyGeom) -> Result<Vec<u8>, BodyCodecError> {
        Ok(match geom {
            BodyGeom::Solid(_) => b"stub:solid".to_vec(),
            BodyGeom::Mesh(_) => b"stub:mesh".to_vec(),
            BodyGeom::Implicit(_) => b"stub:implicit".to_vec(),
            BodyGeom::Form(_) => b"stub:form".to_vec(),
        })
    }

    fn decode(&self, _blob: &BlobRef, bytes: &[u8]) -> Result<BodyGeom, BodyCodecError> {
        match bytes {
            b"stub:solid" => Ok(BodyGeom::Solid(Shape::new())),
            b"stub:mesh" => Ok(BodyGeom::Mesh(Arc::new(IndexedMesh::default()))),
            b"stub:implicit" => Ok(BodyGeom::Implicit(Arc::new(SdfGraph::default()))),
            b"stub:form" => Ok(BodyGeom::Form(Arc::new(SubdMesh::default()))),
            _ => Err(BodyCodecError("unknown stub geometry tag".to_owned())),
        }
    }
}

/// Binary payloads of extension records, by their container path (`ext/…`).
pub type BlobStore = BTreeMap<BlobRef, Arc<[u8]>>;

/// A point in [`save`] where the test hook can stop it. All of them leave the previous file
/// intact; this is what "killed during save" means for the on-disk state.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SavePhase {
    /// The temporary file is written but not yet synced.
    TmpWritten,
    /// The temporary file is synced.
    TmpSynced,
    /// The `.bak` copy exists; the rename has not happened.
    BackupMade,
}

/// What [`SaveOptions::stop_after`] does when its phase is reached.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum StopAction {
    /// Return [`FormatError::Stopped`] without cleaning up (as if the process had died).
    Return,
    /// `std::process::abort()`; for tests that run the save in a child process.
    Abort,
}

/// Options of [`save`].
#[derive(Clone, Debug)]
pub struct SaveOptions {
    /// Keep the previous file as `<name>.amg.bak` (default on).
    pub keep_backup: bool,
    /// PNG for `thumbnail.png`.
    pub thumbnail: Option<Vec<u8>>,
    /// Payloads for the blobs of extension records. Every [`BlobRef`] of every record must be here.
    pub blobs: BlobStore,
    /// Geometry encoder for baked bodies.
    pub codec: Arc<dyn BodyCodec>,
    /// Written to the manifest's `app_version` and `created_by`.
    pub app_version: String,
    /// Test hook: stop at this phase (see [`StopAction`]). `None` in production.
    pub stop_after: Option<(SavePhase, StopAction)>,
}

impl Default for SaveOptions {
    fn default() -> Self {
        SaveOptions {
            keep_backup: true,
            thumbnail: None,
            blobs: BlobStore::new(),
            codec: Arc::new(StubBodyCodec),
            app_version: env!("CARGO_PKG_VERSION").to_owned(),
            stop_after: None,
        }
    }
}

/// Result of [`load`].
#[derive(Debug)]
pub struct LoadedDocument {
    /// The design. Baked bodies have their geometry filled in.
    pub design: Design,
    /// Payloads of extension blobs, by path.
    pub blobs: BlobStore,
    /// `thumbnail.png`, if present.
    pub thumbnail: Option<Vec<u8>>,
    /// Format version found in the file (before migration).
    pub file_version: u32,
    /// Things that loaded but deserve the user's attention (unknown feature kinds).
    pub warnings: Vec<String>,
}

/// `manifest.json`.
#[derive(Serialize, Deserialize)]
pub(crate) struct Manifest {
    pub format: String,
    pub format_version: u32,
    #[serde(default)]
    pub app_version: String,
    #[serde(default)]
    pub created_by: String,
    /// Entry name → blake3 hex of its (uncompressed) content. Every entry but the manifest.
    #[serde(default)]
    pub entries: BTreeMap<String, String>,
}

pub(crate) const MANIFEST: &str = "manifest.json";
pub(crate) const DESIGN: &str = "design.json";
pub(crate) const THUMBNAIL: &str = "thumbnail.png";

/// Checks a blob path: relative, `/`-separated, under `prefix`, no `.`/`..`/empty segments.
pub(crate) fn check_blob_path(blob: &BlobRef, prefix: &str) -> Result<(), FormatError> {
    let p = blob.0.as_str();
    let ok = p.starts_with(prefix)
        && p.len() > prefix.len()
        && !p.contains(['\\', ':', '\0'])
        && !p.ends_with('/')
        && p.split('/').all(|s| !s.is_empty() && s != "." && s != "..");
    if ok {
        Ok(())
    } else {
        Err(FormatError::InvalidBlobPath(p.to_owned()))
    }
}

/// `<path>` + `suffix` (".tmp", ".bak") as a sibling path.
pub(crate) fn sibling(path: &Path, suffix: &str) -> PathBuf {
    let mut s = path.as_os_str().to_owned();
    s.push(suffix);
    PathBuf::from(s)
}

/// Runs [`save`] on the job system against `snap` (the UI never blocks). Progress is reported;
/// cancelling before the final rename leaves the previous file intact.
pub fn save_job(
    jobs: &JobSystem,
    path: PathBuf,
    snap: DesignSnapshot,
    opts: SaveOptions,
) -> JobHandle<(), FormatError> {
    jobs.spawn_fn("Save", SpawnOptions::default(), move |ctx| {
        save_in_job(&path, &snap, &opts, ctx)
    })
}

/// Runs [`load`] on the job system.
pub fn load_job(
    jobs: &JobSystem,
    path: PathBuf,
    codec: Arc<dyn BodyCodec>,
) -> JobHandle<LoadedDocument, FormatError> {
    jobs.spawn_fn("Open", SpawnOptions::default(), move |ctx| {
        load_in_job(&path, &*codec, ctx)
    })
}
