//! `Library`: an in-memory set of materials, appearances and sources, with TOML load/save.

use std::collections::BTreeMap;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use crate::appearance::Appearance;
use crate::error::MaterialError;
use crate::format::{reader, writer};
use crate::id::{AppearanceId, MaterialId};
use crate::material::Material;
use crate::provenance::{Source, SourceKey};

/// Current library file schema version.
pub const FORMAT_VERSION: u32 = 1;

/// The `[library]` table.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LibraryMeta {
    /// Library id (`[a-z0-9._-]+`; user libraries start with `user.`).
    pub id: String,
    /// Display name.
    pub name: String,
    /// Data version shown in the UI.
    pub version: String,
    /// Licence of the data in this file.
    pub data_licence: Option<String>,
    /// Free text.
    pub description: Option<String>,
}

/// One library file in memory.
#[derive(Clone, Debug)]
pub struct Library {
    pub(crate) meta: LibraryMeta,
    pub(crate) sources: BTreeMap<SourceKey, Source>,
    pub(crate) materials: BTreeMap<MaterialId, Arc<Material>>,
    pub(crate) appearances: BTreeMap<AppearanceId, Arc<Appearance>>,
    pub(crate) path: Option<PathBuf>,
    pub(crate) read_only: bool,
}

impl Library {
    /// An empty, editable library.
    pub fn new(meta: LibraryMeta) -> Library {
        Library {
            meta,
            sources: BTreeMap::new(),
            materials: BTreeMap::new(),
            appearances: BTreeMap::new(),
            path: None,
            read_only: false,
        }
    }

    /// Parses a library file. `origin` (a path, or `builtin`) is used in error messages.
    pub fn from_toml_str(src: &str, origin: &str) -> Result<Library, MaterialError> {
        reader::read_library(src, origin)
    }

    /// The canonical TOML text (a fixed point: re-saving a loaded file is byte-identical).
    pub fn to_toml_string(&self) -> String {
        writer::write_library(self)
    }

    /// Loads a user library file. Its id must start with `user.`.
    pub fn load(path: &Path) -> Result<Library, MaterialError> {
        let src = std::fs::read_to_string(path)?;
        let origin = path.display().to_string();
        let mut lib = Library::from_toml_str(&src, &origin)?;
        if !lib.meta.id.starts_with("user.") {
            return Err(MaterialError::Invalid {
                id: origin,
                property: "library.id".to_owned(),
                reason: format!(
                    "user libraries must start with `user.`, got `{}`",
                    lib.meta.id
                ),
            });
        }
        lib.path = Some(path.to_owned());
        Ok(lib)
    }

    /// Sets the file `save` writes to.
    pub fn set_path(&mut self, path: impl Into<PathBuf>) {
        self.path = Some(path.into());
    }

    /// Atomic save: write a temp file next to the target, fsync, rename over it.
    /// `Err(ReadOnly)` for the built-in library.
    pub fn save(&self) -> Result<(), MaterialError> {
        if self.read_only {
            return Err(MaterialError::ReadOnly(self.meta.id.clone()));
        }
        let path = self
            .path
            .as_deref()
            .ok_or_else(|| MaterialError::NoPath(self.meta.id.clone()))?;
        atomic_write(path, self.to_toml_string().as_bytes())?;
        Ok(())
    }

    /// The `[library]` table.
    pub fn meta(&self) -> &LibraryMeta {
        &self.meta
    }

    /// True for the embedded built-in library.
    pub fn is_read_only(&self) -> bool {
        self.read_only
    }

    /// The file this library was loaded from or will be saved to.
    pub fn path(&self) -> Option<&Path> {
        self.path.as_deref()
    }

    /// A material by id.
    pub fn material(&self, id: &MaterialId) -> Option<&Arc<Material>> {
        self.materials.get(id)
    }

    /// All materials, sorted by id.
    pub fn materials(&self) -> impl Iterator<Item = &Arc<Material>> + '_ {
        self.materials.values()
    }

    /// An appearance by id.
    pub fn appearance(&self, id: &AppearanceId) -> Option<&Arc<Appearance>> {
        self.appearances.get(id)
    }

    /// All appearances, sorted by id.
    pub fn appearances(&self) -> impl Iterator<Item = &Arc<Appearance>> + '_ {
        self.appearances.values()
    }

    /// A source record by key.
    pub fn source(&self, key: &SourceKey) -> Option<&Source> {
        self.sources.get(key)
    }

    /// All source records, sorted by key.
    pub fn sources(&self) -> impl Iterator<Item = &Source> + '_ {
        self.sources.values()
    }

    fn check_writable(&self) -> Result<(), MaterialError> {
        if self.read_only {
            Err(MaterialError::ReadOnly(self.meta.id.clone()))
        } else {
            Ok(())
        }
    }

    /// Validates, re-keys the id into this library, and sets `revision` to the stored
    /// revision plus one if the content hash changed; an unchanged upsert keeps the stored
    /// entry. The material's cited sources are copied into the library if absent.
    pub fn upsert_material(&mut self, mut m: Material) -> Result<&Arc<Material>, MaterialError> {
        self.check_writable()?;
        m.validate()?;
        m.id = MaterialId::new(&self.meta.id, m.id.local())?;
        for (k, s) in &m.provenance.sources {
            self.sources.entry(k.clone()).or_insert_with(|| s.clone());
        }
        let entry = self.materials.entry(m.id.clone());
        match entry {
            std::collections::btree_map::Entry::Occupied(mut o) => {
                if o.get().content_hash() != m.content_hash() {
                    m.revision = o.get().revision + 1;
                    o.insert(Arc::new(m));
                } else if o.get().provenance != m.provenance {
                    // Citations changed, values did not: keep the revision.
                    m.revision = o.get().revision;
                    o.insert(Arc::new(m));
                }
                Ok(o.into_mut())
            }
            std::collections::btree_map::Entry::Vacant(v) => Ok(v.insert(Arc::new(m))),
        }
    }

    /// Removes a material.
    pub fn remove_material(&mut self, id: &MaterialId) -> Option<Arc<Material>> {
        if self.read_only {
            return None;
        }
        self.materials.remove(id)
    }

    /// Validates, re-keys the id into this library and bumps `revision` on a content change.
    pub fn upsert_appearance(
        &mut self,
        mut a: Appearance,
    ) -> Result<&Arc<Appearance>, MaterialError> {
        self.check_writable()?;
        a.validate()?;
        a.id = AppearanceId::new(&self.meta.id, a.id.local())?;
        match self.appearances.entry(a.id.clone()) {
            std::collections::btree_map::Entry::Occupied(mut o) => {
                if o.get().content_hash() != a.content_hash() {
                    a.revision = o.get().revision + 1;
                    o.insert(Arc::new(a));
                }
                Ok(o.into_mut())
            }
            std::collections::btree_map::Entry::Vacant(v) => Ok(v.insert(Arc::new(a))),
        }
    }

    /// Removes an appearance.
    pub fn remove_appearance(&mut self, id: &AppearanceId) -> Option<Arc<Appearance>> {
        if self.read_only {
            return None;
        }
        self.appearances.remove(id)
    }

    /// Adds or replaces a source record.
    pub fn upsert_source(&mut self, s: Source) {
        if !self.read_only {
            self.sources.insert(s.key.clone(), s);
        }
    }
}

/// Writes `bytes` to `path` via a sibling temp file, fsync and rename.
fn atomic_write(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    let result = (|| {
        let mut f = std::fs::File::create(&tmp)?;
        f.write_all(bytes)?;
        f.sync_all()?;
        drop(f);
        std::fs::rename(&tmp, path)
    })();
    if result.is_err() {
        // Best effort: never leave a stale temp file behind a failed save.
        let _ = std::fs::remove_file(&tmp);
    }
    result
}
