//! `testdata/mesh/corpus.toml`: types, parsing, validation, file lookup.

use std::collections::BTreeSet;
use std::io::Read;
use std::path::{Path, PathBuf};

use serde::Deserialize;
use sha2::{Digest, Sha256};

/// The 11 defect categories used to stratify the corpus.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Defect {
    /// Boundary edges (open surface).
    Holes,
    /// Edges shared by more than two faces.
    NonManifoldEdges,
    /// Vertices whose faces form more than one fan.
    NonManifoldVertices,
    /// Neighbouring faces with opposite winding.
    InconsistentOrientation,
    /// Closed shells with negative volume.
    Inverted,
    /// Zero-area triangles or triangles with repeated vertices.
    Degenerate,
    /// Faces repeated with the same vertex set.
    Duplicate,
    /// Triangles that cross each other.
    SelfIntersections,
    /// More than one connected component.
    MultipleShells,
    /// Seams whose boundary edges disappear when vertices within 1e-4 of the diagonal are welded.
    Unwelded,
    /// Scan soups that only a wrap can fix (excluded from the repair targets).
    Hopeless,
}

impl Defect {
    /// All categories, in manifest order.
    pub const ALL: [Defect; 11] = [
        Defect::Holes,
        Defect::NonManifoldEdges,
        Defect::NonManifoldVertices,
        Defect::InconsistentOrientation,
        Defect::Inverted,
        Defect::Degenerate,
        Defect::Duplicate,
        Defect::SelfIntersections,
        Defect::MultipleShells,
        Defect::Unwelded,
        Defect::Hopeless,
    ];
}

/// Errors from reading the corpus.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum CorpusError {
    /// File system error.
    #[error("{path}: {source}")]
    Io {
        /// File involved.
        path: PathBuf,
        /// Underlying error.
        source: std::io::Error,
    },
    /// The manifest is not valid TOML for the schema.
    #[error("corpus.toml: {0}")]
    Toml(#[from] toml::de::Error),
    /// The manifest breaks a rule (see [`Manifest::validate`]).
    #[error("corpus manifest: {0}")]
    Invalid(String),
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CorpusHeader {
    format: u32,
}

/// One real model.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Entry {
    /// Unique id, `<source>-<n>`.
    pub id: String,
    /// Path relative to the corpus root: `real/...` (committed) or `fetch/...` (fetch-only).
    pub file: String,
    /// `stl` or `ply`.
    pub format: String,
    /// Page the licence applies to.
    pub source_url: String,
    /// SPDX id, or `LicenseRef-*` where none exists.
    pub license: String,
    /// Attribution.
    pub author: String,
    /// Model title.
    #[serde(default)]
    pub title: String,
    /// sha256 (hex) of the model file itself.
    pub sha256: String,
    /// Triangle count stored in the file.
    pub faces: usize,
    /// Defect ground truth (provisional until MESH-02 reproduces it).
    pub defects: Vec<Defect>,
    /// `false`: scan-like soup, excluded from the 95 % watertight target.
    pub expect_repairable: bool,
    /// Not in the repo; downloaded by `fetch.sh` / `fetch.ps1`.
    #[serde(default)]
    pub fetch: bool,
    /// Download URL of the fetch-only file or archive.
    #[serde(default)]
    pub fetch_url: Option<String>,
    /// sha256 (hex) of the downloaded file or archive.
    #[serde(default)]
    pub fetch_sha256: Option<String>,
    /// `none`, `gz` or `tar.gz`.
    #[serde(default)]
    pub fetch_archive: Option<String>,
    /// Member to extract from a `tar.gz`.
    #[serde(default)]
    pub fetch_member: Option<String>,
    /// Dataset id (for example the Thingi10K file id).
    #[serde(default)]
    pub dataset_id: Option<String>,
    /// Free text.
    #[serde(default)]
    pub notes: Option<String>,
}

/// Where an entry's file is.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FileStatus {
    /// Real file present.
    Ready(PathBuf),
    /// A git-LFS pointer (clone without `git lfs pull`).
    LfsPointer(PathBuf),
    /// Fetch-only entry that has not been fetched.
    NotFetched(PathBuf),
    /// A committed entry whose file is missing (a bug, not a skip).
    Missing(PathBuf),
}

/// The whole manifest.
#[derive(Clone, Debug)]
pub struct Manifest {
    /// Entries in file order.
    pub meshes: Vec<Entry>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestRaw {
    corpus: CorpusHeader,
    mesh: Vec<Entry>,
}

/// `<workspace>/testdata/mesh`.
pub fn corpus_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("testdata")
        .join("mesh")
}

/// Lower-case hex sha256 of a file, streamed.
///
/// # Errors
/// If the file cannot be read.
pub fn sha256_hex(path: &Path) -> Result<String, CorpusError> {
    let io = |source| CorpusError::Io {
        path: path.to_path_buf(),
        source,
    };
    let mut f = std::fs::File::open(path).map_err(io)?;
    let mut h = Sha256::new();
    let mut buf = vec![0u8; 1 << 16];
    loop {
        let n = f.read(&mut buf).map_err(io)?;
        if n == 0 {
            break;
        }
        h.update(&buf[..n]);
    }
    let mut s = String::with_capacity(64);
    for b in h.finalize() {
        s.push_str(&format!("{b:02x}"));
    }
    Ok(s)
}

const LFS_PREFIX: &[u8] = b"version https://git-lfs.github.com/spec/v1";

fn is_lfs_pointer(path: &Path) -> bool {
    let Ok(meta) = std::fs::metadata(path) else {
        return false;
    };
    if meta.len() > 1024 {
        return false;
    }
    std::fs::read(path).is_ok_and(|b| b.starts_with(LFS_PREFIX))
}

/// Licences that may be committed (ADR-012, S07 note §6): redistribution allowed.
const COMMITTABLE: [&str; 4] = [
    "CC0-1.0",
    "CC-BY-3.0",
    "CC-BY-SA-3.0",
    "LicenseRef-PublicDomain",
];

/// Per-file size cap for committed models (note §6).
pub const MAX_COMMITTED_FILE_BYTES: u64 = 20 * 1024 * 1024;
/// Total size cap for committed models (note §6).
pub const MAX_COMMITTED_TOTAL_BYTES: u64 = 300 * 1024 * 1024;

fn is_sha256(s: &str) -> bool {
    s.len() == 64
        && s.bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

impl Manifest {
    /// Parses and validates manifest text.
    ///
    /// # Errors
    /// On TOML errors or when [`Manifest::validate`] fails.
    pub fn parse(text: &str) -> Result<Manifest, CorpusError> {
        let raw: ManifestRaw = toml::from_str(text)?;
        if raw.corpus.format != 1 {
            return Err(CorpusError::Invalid(format!(
                "unsupported corpus format {}",
                raw.corpus.format
            )));
        }
        let m = Manifest { meshes: raw.mesh };
        m.validate()?;
        Ok(m)
    }

    /// Loads `<corpus_root>/corpus.toml`.
    ///
    /// # Errors
    /// On I/O, TOML or validation errors.
    pub fn load() -> Result<Manifest, CorpusError> {
        let path = corpus_root().join("corpus.toml");
        let text = std::fs::read_to_string(&path).map_err(|source| CorpusError::Io {
            path: path.clone(),
            source,
        })?;
        Manifest::parse(&text)
    }

    /// Entries committed through git-LFS.
    pub fn committed(&self) -> impl Iterator<Item = &Entry> {
        self.meshes.iter().filter(|e| !e.fetch)
    }

    /// Fetch-only entries.
    pub fn fetch_only(&self) -> impl Iterator<Item = &Entry> {
        self.meshes.iter().filter(|e| e.fetch)
    }

    /// Checks the rules of MESH-28: unique ids and files, well-formed hashes, licence and source
    /// on every entry, committed entries only with a redistributable licence, `real/` versus
    /// `fetch/` paths consistent with `fetch`, complete fetch metadata, and the corpus-level
    /// requirements (at least 150 committed models, all 11 defect categories covered).
    ///
    /// # Errors
    /// [`CorpusError::Invalid`] naming the first broken rule.
    pub fn validate(&self) -> Result<(), CorpusError> {
        let bad = |s: String| Err(CorpusError::Invalid(s));
        let mut ids = BTreeSet::new();
        let mut files = BTreeSet::new();
        let mut covered = BTreeSet::new();
        for e in &self.meshes {
            if !ids.insert(e.id.as_str()) {
                return bad(format!("duplicate id {}", e.id));
            }
            if !files.insert(e.file.as_str()) {
                return bad(format!("duplicate file {}", e.file));
            }
            if !is_sha256(&e.sha256) {
                return bad(format!("{}: sha256 is not 64 lower-case hex digits", e.id));
            }
            if e.license.trim().is_empty() || e.source_url.trim().is_empty() {
                return bad(format!("{}: licence and source_url are required", e.id));
            }
            if e.license != "CC0-1.0"
                && e.license != "LicenseRef-PublicDomain"
                && e.author.trim().is_empty()
            {
                return bad(format!("{}: attribution (author) is required", e.id));
            }
            if !matches!(e.format.as_str(), "stl" | "ply") || !e.file.ends_with(&e.format) {
                return bad(format!("{}: format {} does not match file", e.id, e.format));
            }
            if e.file.contains("..") || e.file.starts_with('/') || e.file.contains('\\') {
                return bad(format!("{}: file must be a relative `/` path", e.id));
            }
            if e.faces == 0 {
                return bad(format!("{}: faces must be > 0", e.id));
            }
            if e.defects.contains(&Defect::Hopeless) == e.expect_repairable {
                return bad(format!(
                    "{}: `hopeless` must coincide with expect_repairable = false",
                    e.id
                ));
            }
            if e.fetch {
                if !e.file.starts_with("fetch/") {
                    return bad(format!("{}: fetch-only files live under fetch/", e.id));
                }
                let archive = e.fetch_archive.as_deref().unwrap_or("");
                if e.fetch_url
                    .as_deref()
                    .is_none_or(|u| !u.starts_with("https://"))
                    || !e.fetch_sha256.as_deref().is_some_and(is_sha256)
                    || !matches!(archive, "none" | "gz" | "tar.gz")
                    || (archive == "tar.gz") != e.fetch_member.is_some()
                {
                    return bad(format!("{}: incomplete or invalid fetch metadata", e.id));
                }
            } else {
                if !e.file.starts_with("real/") {
                    return bad(format!("{}: committed files live under real/", e.id));
                }
                if !COMMITTABLE.contains(&e.license.as_str()) {
                    return bad(format!(
                        "{}: licence {} may not be committed (make it fetch-only)",
                        e.id, e.license
                    ));
                }
                covered.extend(e.defects.iter().copied());
            }
        }
        let committed = self.committed().count();
        if committed < 150 {
            return bad(format!(
                "{committed} committed models, at least 150 required"
            ));
        }
        for d in Defect::ALL {
            if !covered.contains(&d) {
                return bad(format!("no committed model covers defect category {d:?}"));
            }
        }
        Ok(())
    }
}

impl Entry {
    /// Locates the file under `root` and classifies it.
    pub fn locate(&self, root: &Path) -> FileStatus {
        let path = root.join(&self.file);
        if !path.is_file() {
            return if self.fetch {
                FileStatus::NotFetched(path)
            } else {
                FileStatus::Missing(path)
            };
        }
        if is_lfs_pointer(&path) {
            FileStatus::LfsPointer(path)
        } else {
            FileStatus::Ready(path)
        }
    }
}
