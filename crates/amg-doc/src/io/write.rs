//! Safe save (S01 §8): tmp + fsync + backup + rename.

use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;

use amg_jobs::JobCtx;
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, DateTime, ZipWriter};

use super::{
    DESIGN, FORMAT_NAME, FORMAT_VERSION, FormatError, MANIFEST, Manifest, SaveOptions, SavePhase,
    StopAction, THUMBNAIL, check_blob_path, sibling,
};
use crate::model::{BlobRef, Design, DesignSnapshot};

/// Writes `snap` to `path` (see the module docs for the sequence). Run inside a job
/// ([`super::save_job`]) from the UI.
///
/// The previous file at `path` stays valid until the final rename: any failure, cancellation or
/// crash before it leaves that file untouched.
pub fn save(path: &Path, snap: &DesignSnapshot, opts: &SaveOptions) -> Result<(), FormatError> {
    save_in_job(path, snap, opts, &JobCtx::detached())
}

/// One zip entry to write.
struct Entry {
    name: String,
    data: Vec<u8>,
    method: CompressionMethod,
}

/// [`save`] with a job context for progress and cancellation (what [`save_job`] runs).
pub fn save_in_job(
    path: &Path,
    snap: &DesignSnapshot,
    opts: &SaveOptions,
    ctx: &JobCtx,
) -> Result<(), FormatError> {
    let tmp = sibling(path, ".tmp");
    let result = save_inner(path, &tmp, snap, opts, ctx);
    if !matches!(result, Ok(()) | Err(FormatError::Stopped(_))) {
        // A real failure: don't leave our temporary behind. (A "killed" save leaves it, like a
        // dead process would; the next save overwrites it.)
        let _ = fs::remove_file(&tmp);
    }
    result
}

fn stop(opts: &SaveOptions, phase: SavePhase) -> Result<(), FormatError> {
    match opts.stop_after {
        Some((p, StopAction::Return)) if p == phase => Err(FormatError::Stopped(phase)),
        Some((p, StopAction::Abort)) if p == phase => std::process::abort(),
        _ => Ok(()),
    }
}

fn save_inner(
    path: &Path,
    tmp: &Path,
    snap: &DesignSnapshot,
    opts: &SaveOptions,
    ctx: &JobCtx,
) -> Result<(), FormatError> {
    ctx.progress_msg(0.0, "Serializing");
    let entries = collect_entries(snap, opts, ctx)?;
    ctx.checkpoint()?;

    let manifest = Manifest {
        format: FORMAT_NAME.to_owned(),
        format_version: FORMAT_VERSION,
        app_version: opts.app_version.clone(),
        created_by: format!("Amalgam {}", opts.app_version),
        entries: entries
            .iter()
            .map(|e| (e.name.clone(), blake3::hash(&e.data).to_hex().to_string()))
            .collect(),
    };
    let manifest_json = serde_json::to_vec_pretty(&manifest).map_err(|e| FormatError::Json {
        path: MANIFEST.to_owned(),
        msg: e.to_string(),
    })?;

    ctx.progress_msg(0.3, "Writing");
    let mut zip = ZipWriter::new(BufWriter::new(File::create(tmp)?));
    write_entry(
        &mut zip,
        MANIFEST,
        &manifest_json,
        CompressionMethod::Deflated,
    )?;
    for (i, e) in entries.iter().enumerate() {
        write_entry(&mut zip, &e.name, &e.data, e.method)?;
        ctx.checkpoint()?;
        ctx.progress(0.3 + 0.5 * (i + 1) as f32 / entries.len() as f32);
    }
    let mut out = zip.finish()?;
    out.flush()?;
    let file = out.into_inner().map_err(|e| e.into_error())?;
    stop(opts, SavePhase::TmpWritten)?;

    ctx.progress_msg(0.8, "Flushing to disk");
    file.sync_all()?;
    drop(file);
    stop(opts, SavePhase::TmpSynced)?;
    ctx.checkpoint()?;

    if opts.keep_backup && path.exists() {
        // A copy, not a rename: until the final rename the target is never absent.
        let bak = sibling(path, ".bak");
        fs::copy(path, &bak)?;
        // `FlushFileBuffers` needs write access on Windows.
        fs::OpenOptions::new().write(true).open(&bak)?.sync_all()?;
    }
    stop(opts, SavePhase::BackupMade)?;
    // Last chance to cancel; after the rename the new file is the document.
    ctx.checkpoint()?;

    // `rename` replaces an existing file atomically on NTFS and ext4.
    fs::rename(tmp, path)?;
    sync_parent(path);
    ctx.progress_msg(1.0, "Saved");
    Ok(())
}

/// Makes the rename itself durable where the platform allows (directory fsync on Unix).
fn sync_parent(path: &Path) {
    #[cfg(unix)]
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty())
        && let Ok(d) = File::open(dir)
    {
        let _ = d.sync_all();
    }
    #[cfg(not(unix))]
    let _ = path;
}

fn write_entry(
    zip: &mut ZipWriter<BufWriter<File>>,
    name: &str,
    data: &[u8],
    method: CompressionMethod,
) -> Result<(), FormatError> {
    // Fixed timestamps keep the archive deterministic.
    let options = SimpleFileOptions::default()
        .compression_method(method)
        .last_modified_time(DateTime::default())
        .large_file(data.len() as u64 >= u64::from(u32::MAX));
    zip.start_file(name, options)?;
    zip.write_all(data)?;
    Ok(())
}

/// `design.json`, baked bodies, extension blobs and the thumbnail, in a fixed order.
fn collect_entries(
    snap: &DesignSnapshot,
    opts: &SaveOptions,
    ctx: &JobCtx,
) -> Result<Vec<Entry>, FormatError> {
    let design: &Design = snap;
    let mut entries = vec![Entry {
        name: DESIGN.to_owned(),
        data: serde_json::to_vec_pretty(design).map_err(|e| FormatError::Json {
            path: ".".to_owned(),
            msg: e.to_string(),
        })?,
        method: CompressionMethod::Deflated,
    }];

    let mut seen: BTreeMap<&str, ()> = BTreeMap::new();
    for (_, baked) in design.baked_bodies() {
        check_blob_path(&baked.blob, "bodies/")?;
        if seen.insert(&baked.blob.0, ()).is_some() {
            return Err(FormatError::InvalidBlobPath(baked.blob.0.clone()));
        }
        let geom = baked
            .geom
            .as_ref()
            .ok_or_else(|| FormatError::MissingBlob(baked.blob.0.clone()))?;
        let data = opts
            .codec
            .encode(&baked.blob, geom)
            .map_err(|e| FormatError::Codec {
                blob: baked.blob.0.clone(),
                msg: e.0,
            })?;
        entries.push(Entry {
            name: baked.blob.0.clone(),
            data,
            method: CompressionMethod::Deflated,
        });
        ctx.checkpoint()?;
    }

    let mut ext_blobs: BTreeMap<&BlobRef, ()> = BTreeMap::new();
    for (_, rec) in design.exts() {
        for b in &rec.blobs {
            check_blob_path(b, "ext/")?;
            ext_blobs.insert(b, ());
        }
    }
    for b in ext_blobs.into_keys() {
        let data = opts
            .blobs
            .get(b)
            .ok_or_else(|| FormatError::MissingBlob(b.0.clone()))?;
        entries.push(Entry {
            name: b.0.clone(),
            data: data.to_vec(),
            method: CompressionMethod::Deflated,
        });
    }

    if let Some(png) = &opts.thumbnail {
        entries.push(Entry {
            name: THUMBNAIL.to_owned(),
            data: png.clone(),
            method: CompressionMethod::Stored,
        });
    }

    if let Some(export) = &opts.cache {
        for (name, data) in super::cachefile::entries(design, export, ctx)? {
            entries.push(Entry {
                name,
                data,
                method: CompressionMethod::Deflated,
            });
        }
    }
    Ok(entries)
}
