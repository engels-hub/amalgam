//! Loading and migrating `.amg` files.

use std::fs::File;
use std::io::{BufReader, Read, Seek};
use std::path::Path;

use amg_jobs::JobCtx;
use zip::ZipArchive;
use zip::result::ZipError;

use super::{
    BlobStore, BodyCodec, DESIGN, DOC_MIGRATIONS, DocMigration, FORMAT_NAME, FORMAT_VERSION,
    FormatError, LoadedDocument, MANIFEST, Manifest, StubBodyCodec, THUMBNAIL, check_blob_path,
};
use crate::model::{BlobRef, Design};

/// Reads `path` with the stub geometry codec. Run inside a job ([`super::load_job`]).
pub fn load(path: &Path) -> Result<LoadedDocument, FormatError> {
    load_with_codec(path, &StubBodyCodec)
}

/// Reads `path`, decoding baked bodies with `codec`.
pub fn load_with_codec(path: &Path, codec: &dyn BodyCodec) -> Result<LoadedDocument, FormatError> {
    load_in_job(path, codec, &JobCtx::detached())
}

/// Applies `chain` (step `i` is version `i` → `i + 1`) to take `raw` from version `from` to `to`.
pub fn migrate_document(
    chain: &[DocMigration],
    from: u32,
    to: u32,
    mut raw: serde_json::Value,
) -> Result<serde_json::Value, FormatError> {
    for v in from..to {
        let step = chain
            .get(v as usize)
            .ok_or_else(|| FormatError::Migration {
                from: v,
                to: v + 1,
                msg: "no migration step registered".to_owned(),
            })?;
        raw = step(raw).map_err(|e| FormatError::Migration {
            from: v,
            to: v + 1,
            msg: e.0,
        })?;
    }
    Ok(raw)
}

/// [`load_with_codec`] with a job context for progress and cancellation (what [`load_job`] runs).
pub fn load_in_job(
    path: &Path,
    codec: &dyn BodyCodec,
    ctx: &JobCtx,
) -> Result<LoadedDocument, FormatError> {
    ctx.progress_msg(0.0, "Opening");
    let mut file = File::open(path)?;
    let mut magic = [0u8; 2];
    if file.read_exact(&mut magic).is_err() || &magic != b"PK" {
        return Err(FormatError::NotAmalgam);
    }
    file.rewind()?;
    let mut zip = ZipArchive::new(BufReader::new(file)).map_err(|e| match e {
        ZipError::Io(io) => FormatError::Io(io),
        _ => FormatError::Corrupt {
            entry: "zip directory".to_owned(),
        },
    })?;

    // Manifest, then the version gate, before anything else is trusted.
    if zip.index_for_name(MANIFEST).is_none() {
        return Err(FormatError::NotAmalgam);
    }
    let manifest_bytes = read_entry(&mut zip, MANIFEST, None)?;
    let manifest: Manifest =
        serde_json::from_slice(&manifest_bytes).map_err(|_| FormatError::Corrupt {
            entry: MANIFEST.to_owned(),
        })?;
    if manifest.format != FORMAT_NAME {
        return Err(FormatError::NotAmalgam);
    }
    if manifest.format_version > FORMAT_VERSION {
        return Err(FormatError::TooNew {
            file: manifest.format_version,
            supported: FORMAT_VERSION,
        });
    }
    ctx.checkpoint()?;

    // Every listed entry must exist and match its hash, even ones we do not read below.
    ctx.progress_msg(0.1, "Verifying");
    let hash = |name: &str| manifest.entries.get(name).map(String::as_str);
    let design_bytes = read_entry(&mut zip, DESIGN, Some(hash(DESIGN)))?;
    ctx.checkpoint()?;

    ctx.progress_msg(0.3, "Reading design");
    let raw: serde_json::Value =
        serde_json::from_slice(&design_bytes).map_err(|e| FormatError::Json {
            path: ".".to_owned(),
            msg: e.to_string(),
        })?;
    let raw = migrate_document(DOC_MIGRATIONS, manifest.format_version, FORMAT_VERSION, raw)?;
    let mut design: Design = serde_path_to_error::deserialize(raw).map_err(|e| {
        let path = e.path().to_string();
        FormatError::Json {
            path,
            msg: e.into_inner().to_string(),
        }
    })?;
    ctx.checkpoint()?;

    ctx.progress_msg(0.6, "Reading bodies");
    let baked: Vec<_> = design
        .baked_bodies()
        .map(|(id, b)| (id, b.blob.clone()))
        .collect();
    for (id, blob) in baked {
        check_blob_path(&blob, "bodies/")?;
        let bytes = read_entry(&mut zip, &blob.0, Some(hash(&blob.0)))?;
        let geom = codec
            .decode(&blob, &bytes)
            .map_err(|e| FormatError::Codec {
                blob: blob.0.clone(),
                msg: e.0,
            })?;
        design.fill_baked_geom(id, geom);
        ctx.checkpoint()?;
    }

    let mut blobs = BlobStore::new();
    for (_, rec) in design.exts() {
        for b in &rec.blobs {
            check_blob_path(b, "ext/")?;
            if !blobs.contains_key(b) {
                let bytes = read_entry(&mut zip, &b.0, Some(hash(&b.0)))?;
                blobs.insert(BlobRef(b.0.clone()), bytes.into());
            }
        }
    }

    let thumbnail = if zip.index_for_name(THUMBNAIL).is_some() {
        Some(read_entry(&mut zip, THUMBNAIL, Some(hash(THUMBNAIL)))?)
    } else {
        None
    };

    // Entries we did not need still have to be intact.
    for name in manifest.entries.keys() {
        if name != DESIGN
            && name != THUMBNAIL
            && !name.starts_with("bodies/")
            && !name.starts_with("ext/")
        {
            read_entry(&mut zip, name, Some(hash(name)))?;
        }
    }

    let warnings = design
        .timeline()
        .nodes()
        .filter_map(|n| {
            n.feature.as_unknown().map(|u| {
                format!(
                    "feature `{}` (v{}) is unknown to this build; it is kept as is",
                    u.kind, u.version
                )
            })
        })
        .collect();
    ctx.progress_msg(1.0, "Opened");
    Ok(LoadedDocument {
        design,
        blobs,
        thumbnail,
        file_version: manifest.format_version,
        warnings,
    })
}

/// Reads and checks one entry. `expected` is `Some(manifest hash)` for entries the manifest
/// covers (`Some(None)` = covered by nothing, which is itself corrupt for non-manifest entries).
fn read_entry<R: Read + Seek>(
    zip: &mut ZipArchive<R>,
    name: &str,
    expected: Option<Option<&str>>,
) -> Result<Vec<u8>, FormatError> {
    let corrupt = || FormatError::Corrupt {
        entry: name.to_owned(),
    };
    let mut entry = zip.by_name(name).map_err(|e| match e {
        ZipError::Io(io) => FormatError::Io(io),
        _ => corrupt(),
    })?;
    let mut data = Vec::with_capacity(usize::try_from(entry.size()).unwrap_or(0).min(1 << 28));
    // A bad CRC or a damaged deflate stream surfaces as an I/O error from the decoder.
    entry.read_to_end(&mut data).map_err(|_| corrupt())?;
    if let Some(expected) = expected
        && expected != Some(blake3::hash(&data).to_hex().as_str())
    {
        return Err(corrupt());
    }
    Ok(data)
}
