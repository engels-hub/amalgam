//! Geometry-cache persistence in the `.amg` container (S01 §8, DOC-14).
//!
//! # Layout
//! `cache/index.json` holds the [`engine_salt`] the entries were computed under and the list of
//! keys; each output is `cache/<key-hex>.out` (bytes made by an [`OutputCodec`]). Entries are
//! covered by the manifest's blake3 list like every other entry. Only outputs referenced by the
//! evaluated timeline are written ([`Document::cache_export`]), so the cache does not grow with
//! stale LRU leftovers. "Save compact" simply passes no [`CacheExport`].
//!
//! # Opening
//! [`load`](super::load) reads and verifies only `cache/index.json` and returns a [`CacheSeed`].
//! [`CacheSeed::attach_to`] checks the stored salt against the running one and, if equal, installs
//! a [`CacheBacking`] on the [`GeomCache`]: an entry is read from the zip on its first hit. A
//! missing, unreadable or damaged cache never fails a load; the features just recompute.

use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::fmt::Debug;
use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, PoisonError};

use amg_core::{Line3, Plane};
use amg_jobs::JobCtx;
use serde::{Deserialize, Serialize};
use zip::ZipArchive;

use super::read::read_entry;
use super::{FormatError, StubBodyCodec};
use crate::cache::{CacheBacking, GeomCache};
use crate::document::Document;
use crate::eval::{CacheKey, ConstructionGeom, Diagnostic};
use crate::feature::{BodyChange, BodyChangeKind, FeatureOutput};
use crate::id::BodyId;
use crate::io::BodyCodec;
use crate::model::{BlobRef, Design};
use crate::naming::{BodyNaming, EntityRef};
use crate::recompute::engine_salt;
use crate::stub::SketchGeom;

/// Name of the index entry.
pub const CACHE_INDEX: &str = "cache/index.json";
/// Version of the index and entry layout written here.
pub const CACHE_FORMAT: u32 = 1;

/// Error of an [`OutputCodec`].
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error("{0}")]
pub struct OutputCodecError(pub String);

/// Serializes a [`FeatureOutput`] to and from a `cache/<key>.out` entry.
///
/// **Seam.** Geometry types are `amg-doc::stub` placeholders until BREP-02/MESH-02 land, so
/// [`StubOutputCodec`] records structure only. The real codec writes solids with `amg-brep`'s
/// serialization and meshes with `MeshBody::write_binary`, and returns an error for an output it
/// cannot represent; such an entry is left out of the file (the feature recomputes on open).
pub trait OutputCodec: Send + Sync + Debug {
    /// Encodes `out`.
    fn encode(&self, out: &FeatureOutput) -> Result<Vec<u8>, OutputCodecError>;
    /// Decodes bytes made by [`OutputCodec::encode`].
    fn decode(&self, bytes: &[u8]) -> Result<FeatureOutput, OutputCodecError>;
}

/// Output codec for the stub geometry types (kind tags via [`StubBodyCodec`]).
#[derive(Clone, Copy, Debug, Default)]
pub struct StubOutputCodec;

#[derive(Serialize, Deserialize)]
struct WireOutput {
    bodies: Vec<WireBody>,
    sketch: bool,
    construction: Option<WireConstruction>,
    warnings: Vec<WireDiagnostic>,
}

#[derive(Serialize, Deserialize)]
struct WireBody {
    id: BodyId,
    change: WireChange,
}

#[derive(Serialize, Deserialize)]
enum WireChange {
    Created {
        geom: Vec<u8>,
        naming: Option<BodyNaming>,
    },
    Modified {
        geom: Vec<u8>,
        naming: Option<BodyNaming>,
    },
    Deleted,
}

#[derive(Serialize, Deserialize)]
enum WireConstruction {
    Plane(Plane),
    Axis(Line3),
    Point([f64; 3]),
}

#[derive(Serialize, Deserialize)]
struct WireDiagnostic {
    message: String,
    entities: Vec<EntityRef>,
}

fn stub_blob() -> BlobRef {
    BlobRef("cache/stub".to_owned())
}

impl OutputCodec for StubOutputCodec {
    fn encode(&self, out: &FeatureOutput) -> Result<Vec<u8>, OutputCodecError> {
        let geom = |g: &crate::eval::BodyGeom| {
            StubBodyCodec
                .encode(&stub_blob(), g)
                .map_err(|e| OutputCodecError(e.0))
        };
        let bodies = out
            .bodies
            .iter()
            .map(|b| {
                let change = match &b.change {
                    BodyChangeKind::Created { geom: g, naming: n } => WireChange::Created {
                        geom: geom(g)?,
                        naming: n.clone(),
                    },
                    BodyChangeKind::Modified { geom: g, naming: n } => WireChange::Modified {
                        geom: geom(g)?,
                        naming: n.clone(),
                    },
                    BodyChangeKind::Deleted => WireChange::Deleted,
                };
                Ok(WireBody { id: b.id, change })
            })
            .collect::<Result<Vec<_>, OutputCodecError>>()?;
        let warnings = out
            .warnings
            .iter()
            .map(|w| {
                if w.cause.is_some() {
                    return Err(OutputCodecError(
                        "a warning with a structured cause is not persisted".to_owned(),
                    ));
                }
                Ok(WireDiagnostic {
                    message: w.message.clone(),
                    entities: w.entities.clone(),
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let wire = WireOutput {
            bodies,
            sketch: out.sketch.is_some(),
            construction: out.construction.map(|c| match c {
                ConstructionGeom::Plane(p) => WireConstruction::Plane(p),
                ConstructionGeom::Axis(l) => WireConstruction::Axis(l),
                ConstructionGeom::Point(p) => WireConstruction::Point(p.to_array()),
            }),
            warnings,
        };
        serde_json::to_vec(&wire).map_err(|e| OutputCodecError(e.to_string()))
    }

    fn decode(&self, bytes: &[u8]) -> Result<FeatureOutput, OutputCodecError> {
        let wire: WireOutput =
            serde_json::from_slice(bytes).map_err(|e| OutputCodecError(e.to_string()))?;
        let geom = |b: &[u8]| {
            StubBodyCodec
                .decode(&stub_blob(), b)
                .map_err(|e| OutputCodecError(e.0))
        };
        let mut bodies = Vec::with_capacity(wire.bodies.len());
        for b in wire.bodies {
            let change = match b.change {
                WireChange::Created { geom: g, naming } => BodyChangeKind::Created {
                    geom: geom(&g)?,
                    naming,
                },
                WireChange::Modified { geom: g, naming } => BodyChangeKind::Modified {
                    geom: geom(&g)?,
                    naming,
                },
                WireChange::Deleted => BodyChangeKind::Deleted,
            };
            bodies.push(BodyChange { id: b.id, change });
        }
        Ok(FeatureOutput {
            bodies,
            sketch: wire.sketch.then(|| Arc::new(SketchGeom::default())),
            construction: wire.construction.map(|c| match c {
                WireConstruction::Plane(p) => ConstructionGeom::Plane(p),
                WireConstruction::Axis(l) => ConstructionGeom::Axis(l),
                WireConstruction::Point(p) => ConstructionGeom::Point(p.into()),
            }),
            warnings: wire
                .warnings
                .into_iter()
                .map(|w| Diagnostic {
                    message: w.message,
                    cause: None,
                    entities: w.entities,
                })
                .collect(),
        })
    }
}

/// What to persist: the cache, the keys the timeline uses and the codec for the outputs.
/// Made by [`Document::cache_export`]; passed through [`SaveOptions::cache`](super::SaveOptions).
#[derive(Clone, Debug)]
pub struct CacheExport {
    /// Where the outputs are.
    pub cache: Arc<GeomCache>,
    /// Keys to write (outputs absent from `cache` are skipped).
    pub keys: Vec<CacheKey>,
    /// Output serializer.
    pub codec: Arc<dyn OutputCodec>,
}

impl CacheExport {
    /// Export of `keys` with the stub codec.
    pub fn new(cache: Arc<GeomCache>, keys: Vec<CacheKey>) -> CacheExport {
        CacheExport {
            cache,
            keys,
            codec: Arc::new(StubOutputCodec),
        }
    }
}

impl Document {
    /// The persistent cache with the keys of the current evaluated timeline, for
    /// [`SaveOptions::cache`](super::SaveOptions). Cheap (no serialization happens here).
    pub fn cache_export(&self) -> CacheExport {
        let keys = self
            .evaluated()
            .nodes()
            .filter_map(|(_, n)| n.key)
            .collect();
        CacheExport::new(Arc::clone(self.cache()), keys)
    }
}

#[derive(Serialize, Deserialize)]
struct CacheIndex {
    version: u32,
    /// Hex of the 32-byte engine salt the keys were derived under.
    engine_salt: String,
    /// Hex keys; each has a `cache/<key>.out` entry.
    keys: Vec<String>,
}

fn entry_name(key: &CacheKey) -> String {
    format!("cache/{}.out", key.to_hex())
}

fn hex32(s: &str) -> Option<[u8; 32]> {
    if s.len() != 64 || !s.is_ascii() {
        return None;
    }
    let mut out = [0u8; 32];
    for (i, b) in out.iter_mut().enumerate() {
        *b = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).ok()?;
    }
    Some(out)
}

fn to_hex(b: &[u8; 32]) -> String {
    CacheKey(*b).to_hex()
}

/// Encoded cache entries (`index.json` first), ready to be written, in key order.
pub(super) fn entries(
    design: &Design,
    export: &CacheExport,
    ctx: &JobCtx,
) -> Result<Vec<(String, Vec<u8>)>, FormatError> {
    let salt = engine_salt(&design.meta().tolerance);
    let keys: BTreeSet<CacheKey> = export.keys.iter().copied().collect();
    let mut written = Vec::new();
    let mut files = Vec::new();
    for key in keys {
        ctx.checkpoint()?;
        let Some(out) = export.cache.peek(&key) else {
            continue;
        };
        // An output the codec cannot represent is skipped: it recomputes on open.
        let Ok(bytes) = export.codec.encode(&out) else {
            continue;
        };
        written.push(key.to_hex());
        files.push((entry_name(&key), bytes));
    }
    let index = CacheIndex {
        version: CACHE_FORMAT,
        engine_salt: to_hex(&salt),
        keys: written,
    };
    let index_json = serde_json::to_vec_pretty(&index).map_err(|e| FormatError::Json {
        path: CACHE_INDEX.to_owned(),
        msg: e.to_string(),
    })?;
    let mut all = vec![(CACHE_INDEX.to_owned(), index_json)];
    all.extend(files);
    Ok(all)
}

/// The persisted cache of an opened file, not yet read. See [`CacheSeed::attach_to`].
#[derive(Clone, Debug)]
pub struct CacheSeed {
    stored_salt: Option<[u8; 32]>,
    keys: Arc<HashSet<CacheKey>>,
    hashes: Arc<BTreeMap<String, String>>,
    path: Option<PathBuf>,
    codec: Arc<dyn OutputCodec>,
}

impl CacheSeed {
    /// A seed with nothing in it (file without a cache, or compact save).
    pub fn empty() -> CacheSeed {
        CacheSeed {
            stored_salt: None,
            keys: Arc::new(HashSet::new()),
            hashes: Arc::new(BTreeMap::new()),
            path: None,
            codec: Arc::new(StubOutputCodec),
        }
    }

    /// Number of persisted outputs.
    pub fn len(&self) -> usize {
        self.keys.len()
    }

    /// True if the file had no (usable) cache.
    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }

    /// The salt the entries were computed under.
    pub fn stored_salt(&self) -> Option<[u8; 32]> {
        self.stored_salt
    }

    /// Uses `codec` to read entries (the default is [`StubOutputCodec`]).
    #[must_use]
    pub fn with_codec(mut self, codec: Arc<dyn OutputCodec>) -> CacheSeed {
        self.codec = codec;
        self
    }

    /// Lets `cache` read this file's entries lazily, if the stored salt equals `current_salt`
    /// (`engine_salt(&design.meta().tolerance)`). Returns whether it did. With another salt the
    /// keys would never match anyway; nothing is attached and everything recomputes.
    pub fn attach_to(&self, cache: &GeomCache, current_salt: &[u8; 32]) -> bool {
        let (Some(path), Some(salt)) = (&self.path, &self.stored_salt) else {
            return false;
        };
        if salt != current_salt || self.keys.is_empty() {
            return false;
        }
        cache.set_backing(Some(Arc::new(LazyZipCache {
            path: path.clone(),
            keys: Arc::clone(&self.keys),
            hashes: Arc::clone(&self.hashes),
            codec: Arc::clone(&self.codec),
            zip: Mutex::new(None),
        })));
        true
    }
}

/// Reads `cache/<key>.out` from the file on first use. The archive handle opens on the first
/// request and stays open.
struct LazyZipCache {
    path: PathBuf,
    keys: Arc<HashSet<CacheKey>>,
    hashes: Arc<BTreeMap<String, String>>,
    codec: Arc<dyn OutputCodec>,
    zip: Mutex<Option<ZipArchive<BufReader<File>>>>,
}

impl CacheBacking for LazyZipCache {
    fn load(&self, key: &CacheKey) -> Option<Arc<FeatureOutput>> {
        if !self.keys.contains(key) {
            return None;
        }
        let name = entry_name(key);
        let hash = self.hashes.get(&name)?;
        let mut g = self.zip.lock().unwrap_or_else(PoisonError::into_inner);
        if g.is_none() {
            let file = File::open(&self.path).ok()?;
            *g = ZipArchive::new(BufReader::new(file)).ok();
        }
        let zip = g.as_mut()?;
        let bytes = read_entry(zip, &name, Some(Some(hash.as_str()))).ok()?;
        drop(g);
        self.codec.decode(&bytes).ok().map(Arc::new)
    }
}

/// Reads the index (the only cache entry read at open). Any problem gives an empty seed and a
/// warning: a bad cache is never a load error.
pub(super) fn read_seed<R: std::io::Read + std::io::Seek>(
    zip: &mut ZipArchive<R>,
    path: &std::path::Path,
    entries: &BTreeMap<String, String>,
    warnings: &mut Vec<String>,
) -> CacheSeed {
    let Some(hash) = entries.get(CACHE_INDEX) else {
        return CacheSeed::empty();
    };
    let index = read_entry(zip, CACHE_INDEX, Some(Some(hash.as_str())))
        .map_err(|e| e.to_string())
        .and_then(|b| serde_json::from_slice::<CacheIndex>(&b).map_err(|e| e.to_string()));
    let index = match index {
        Ok(i) if i.version == CACHE_FORMAT => i,
        Ok(_) => return CacheSeed::empty(), // another layout: just recompute
        Err(e) => {
            warnings.push(format!("geometry cache ignored: {e}"));
            return CacheSeed::empty();
        }
    };
    let Some(salt) = hex32(&index.engine_salt) else {
        warnings.push("geometry cache ignored: bad salt".to_owned());
        return CacheSeed::empty();
    };
    let keys: HashSet<CacheKey> = index
        .keys
        .iter()
        .filter_map(|k| hex32(k).map(CacheKey))
        .filter(|k| entries.contains_key(&entry_name(k)))
        .collect();
    let hashes = entries
        .iter()
        .filter(|(n, _)| n.starts_with("cache/"))
        .map(|(n, h)| (n.clone(), h.clone()))
        .collect();
    CacheSeed {
        stored_salt: Some(salt),
        keys: Arc::new(keys),
        hashes: Arc::new(hashes),
        path: Some(std::path::absolute(path).unwrap_or_else(|_| path.to_path_buf())),
        codec: Arc::new(StubOutputCodec),
    }
}
