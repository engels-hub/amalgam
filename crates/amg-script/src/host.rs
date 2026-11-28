//! The document a script works on.
//!
//! [`ScriptDoc`] is a real [`amg_doc::Document`] plus what a headless run needs around it: the
//! file path, the job system and the file's extension blobs and thumbnail (so a save keeps
//! them). The mirror to the UI document is SCR-03.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use amg_doc::io::BlobStore;
use amg_doc::{Command, CommandOutcome, DesignMode, Document, Evaluated, SaveOptions};
use amg_jobs::{CancelToken, JobSystem};

use crate::ScriptError;

/// A script's document.
///
/// All methods that compute block the calling thread: never call them on the UI thread.
#[derive(Debug)]
pub struct ScriptDoc {
    doc: Document,
    path: Option<PathBuf>,
    jobs: Arc<JobSystem>,
    blobs: BlobStore,
    thumbnail: Option<Vec<u8>>,
    warnings: Vec<String>,
}

impl ScriptDoc {
    /// An empty design called `name`.
    pub fn new(name: &str, jobs: Arc<JobSystem>) -> ScriptDoc {
        // A broken registry is a build error that `open` reports and the registry's first use
        // panics on; an empty design cannot hit it.
        let _ = crate::link();
        ScriptDoc {
            doc: Document::new(name),
            path: None,
            jobs,
            blobs: BlobStore::new(),
            thumbnail: None,
            warnings: Vec::new(),
        }
    }

    /// Opens a `.amg` file. The saved geometry cache is used, so a fully cached design
    /// recomputes nothing.
    ///
    /// # Errors
    /// [`ScriptError::Format`] for a missing, corrupt or too new file;
    /// [`ScriptError::Registry`] for a broken build.
    pub fn open(path: &Path, jobs: Arc<JobSystem>) -> Result<ScriptDoc, ScriptError> {
        crate::link().map_err(Clone::clone)?;
        let mut loaded = amg_doc::load(path)?;
        let blobs = std::mem::take(&mut loaded.blobs);
        let thumbnail = loaded.thumbnail.take();
        let warnings = std::mem::take(&mut loaded.warnings);
        Ok(ScriptDoc {
            doc: loaded.into_document(),
            path: Some(path.to_owned()),
            jobs,
            blobs,
            thumbnail,
            warnings,
        })
    }

    /// The document.
    pub fn document(&self) -> &Document {
        &self.doc
    }

    /// The file the document was opened from or last saved to.
    pub fn path(&self) -> Option<&Path> {
        self.path.as_deref()
    }

    /// The job system computations run on.
    pub fn jobs(&self) -> &Arc<JobSystem> {
        &self.jobs
    }

    /// What loading the file complained about (unknown feature kinds, an unusable cache).
    pub fn warnings(&self) -> &[String] {
        &self.warnings
    }

    /// Applies `cmds` as **one** undo entry called `label`. Atomic: if one command fails,
    /// none is applied.
    ///
    /// A lone `SetDesignMode { Direct }` bakes the evaluated bodies, so the design is
    /// recomputed first; its undo entry has the command's own label.
    ///
    /// # Errors
    /// [`ScriptError::Command`] (or a recompute error before a switch to direct mode).
    pub fn apply(
        &mut self,
        label: &str,
        cmds: Vec<Command>,
    ) -> Result<Vec<CommandOutcome>, ScriptError> {
        if matches!(
            cmds.as_slice(),
            [Command::SetDesignMode {
                mode: DesignMode::Direct
            }]
        ) && self.doc.design().meta().mode == DesignMode::History
        {
            self.evaluated(&CancelToken::new())?;
            let mut out = Vec::with_capacity(1);
            for c in cmds {
                out.push(self.doc.apply(c)?);
            }
            return Ok(out);
        }
        let mut tx = self.doc.transaction(label);
        let mut out = Vec::with_capacity(cmds.len());
        for c in cmds {
            // Dropping the transaction on error discards the earlier commands.
            out.push(tx.apply(c)?);
        }
        let revision = tx.commit();
        for o in &mut out {
            o.revision = revision;
        }
        Ok(out)
    }

    /// Steps back one undo entry. False if there is nothing to undo.
    pub fn undo(&mut self) -> bool {
        self.doc.undo()
    }

    /// Steps forward one undo entry. False if there is nothing to redo.
    pub fn redo(&mut self) -> bool {
        self.doc.redo()
    }

    /// The evaluated state of the current revision, recomputing first (blocking) if it is
    /// stale. Failing features do not make this an error: see their health.
    ///
    /// # Errors
    /// [`ScriptError::Cancelled`] if `cancel` is cancelled while a recompute is needed (the
    /// document stays usable), or [`ScriptError::Recompute`] if the recompute panicked.
    pub fn evaluated(&mut self, cancel: &CancelToken) -> Result<Arc<Evaluated>, ScriptError> {
        self.doc.recompute_blocking_with(&self.jobs, cancel)?;
        Ok(Arc::clone(self.doc.evaluated()))
    }

    /// Saves to `path`, or to the current file, and returns the path written.
    ///
    /// The opened file's extension blobs and thumbnail are kept unless `opts` provides them.
    /// The geometry cache is written only if `opts.cache` is set (pass
    /// `Some(doc.document().cache_export())` after [`ScriptDoc::evaluated`]); otherwise the
    /// file is saved compact.
    ///
    /// # Errors
    /// [`ScriptError::Field`] if there is no path to save to; [`ScriptError::Format`].
    pub fn save(
        &mut self,
        path: Option<&Path>,
        opts: &SaveOptions,
    ) -> Result<PathBuf, ScriptError> {
        let path = path
            .map(Path::to_path_buf)
            .or_else(|| self.path.clone())
            .ok_or_else(|| ScriptError::Field {
                field: "path".to_owned(),
                msg: "the document has no file yet; give a path to save to".to_owned(),
            })?;
        let mut opts = opts.clone();
        for (blob, data) in &self.blobs {
            opts.blobs
                .entry(blob.clone())
                .or_insert_with(|| Arc::clone(data));
        }
        if opts.thumbnail.is_none() {
            opts.thumbnail.clone_from(&self.thumbnail);
        }
        amg_doc::save(&path, &self.doc.snapshot(), &opts)?;
        self.doc.history_mut().mark_saved();
        self.blobs = opts.blobs;
        self.path = Some(path.clone());
        Ok(path)
    }
}
