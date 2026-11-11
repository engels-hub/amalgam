//! `LibrarySet`: the built-in library plus the user libraries, for picking, search, import
//! matching and drift detection. A document's own material copy stays authoritative.

use std::path::Path;
use std::sync::Arc;

use crate::appearance::Appearance;
use crate::builtin::builtin;
use crate::error::MaterialError;
use crate::id::{AppearanceId, MaterialId};
use crate::library::{Library, LibraryMeta};
use crate::material::{AmProcess, Category, Material, MaterialFamily};

/// Search filters; every `Some` must match. `text` is a case-insensitive substring of the
/// name, id, description or a tag.
#[derive(Clone, Debug, Default)]
pub struct MaterialQuery {
    /// Free text.
    pub text: Option<String>,
    /// Broad class.
    pub category: Option<Category>,
    /// AM process.
    pub process: Option<AmProcess>,
    /// Material family.
    pub family: Option<MaterialFamily>,
}

/// How a document's material copy relates to the library entry with the same id.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Drift {
    /// Same values as the library entry.
    UpToDate,
    /// The library entry has a higher revision and different values.
    LibraryNewer {
        /// The library's current revision.
        library_revision: u32,
    },
    /// The values differ at the same (or a higher) revision: the copy was edited.
    LocallyModified,
    /// No library entry has this id (any more).
    NotInLibrary,
}

/// The built-in library and the user libraries (sorted by library id).
///
/// Not used to resolve a document's materials (the document copy is authoritative).
#[derive(Debug)]
pub struct LibrarySet {
    builtin: &'static Library,
    user: Vec<Library>,
}

impl LibrarySet {
    /// Only the built-in library.
    pub fn builtin_only() -> LibrarySet {
        LibrarySet::with_builtin(builtin())
    }

    /// A set around another read-only base library (tests and tools).
    pub fn with_builtin(base: &'static Library) -> LibrarySet {
        LibrarySet {
            builtin: base,
            user: Vec::new(),
        }
    }

    /// Loads every `*.toml` in `dir` (created if missing), in file-name order. Bad files and
    /// libraries whose id is already loaded from another file are skipped and returned. A file
    /// that was loaded before replaces its library (reload).
    pub fn load_user_dir(&mut self, dir: &Path) -> Vec<MaterialError> {
        let mut errors = Vec::new();
        if let Err(e) = std::fs::create_dir_all(dir) {
            errors.push(e.into());
            return errors;
        }
        let mut files = match std::fs::read_dir(dir) {
            Ok(rd) => rd
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.is_file() && p.extension().is_some_and(|x| x == "toml"))
                .collect::<Vec<_>>(),
            Err(e) => {
                errors.push(e.into());
                return errors;
            }
        };
        files.sort();
        for path in files {
            match Library::load(&path) {
                Ok(lib) => {
                    self.user.retain(|u| u.path() != Some(path.as_path()));
                    if self.user.iter().any(|u| u.meta.id == lib.meta.id) {
                        errors.push(MaterialError::Duplicate {
                            origin: path.display().to_string(),
                            id: lib.meta.id.clone(),
                        });
                    } else {
                        self.user.push(lib);
                    }
                }
                Err(e) => errors.push(e),
            }
        }
        self.user.sort_by(|a, b| a.meta.id.cmp(&b.meta.id));
        errors
    }

    /// Creates `<dir>/<id>.toml` (an empty library, saved at once) and adds it to the set.
    /// `id` must start with `user.`.
    pub fn create_user_library(
        &mut self,
        dir: &Path,
        id: &str,
        name: &str,
    ) -> Result<&mut Library, MaterialError> {
        MaterialId::new(id, "x")?;
        if !id.starts_with("user.") {
            return Err(MaterialError::InvalidId(id.to_owned()));
        }
        let path = dir.join(format!("{id}.toml"));
        if self.user.iter().any(|u| u.meta.id == id) {
            return Err(MaterialError::Duplicate {
                origin: path.display().to_string(),
                id: id.to_owned(),
            });
        }
        std::fs::create_dir_all(dir)?;
        if path.exists() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::AlreadyExists,
                format!("{} already exists", path.display()),
            )
            .into());
        }
        let mut lib = Library::new(LibraryMeta {
            id: id.to_owned(),
            name: name.to_owned(),
            version: "1".to_owned(),
            data_licence: None,
            description: None,
        });
        lib.set_path(path);
        lib.save()?;
        let at = self.user.partition_point(|u| u.meta.id.as_str() < id);
        self.user.insert(at, lib);
        Ok(&mut self.user[at])
    }

    /// The built-in library first, then the user libraries by id.
    pub fn libraries(&self) -> impl Iterator<Item = &Library> + '_ {
        std::iter::once(self.builtin).chain(self.user.iter())
    }

    /// A user library for editing; `None` for the built-in library and unknown ids.
    pub fn library_mut(&mut self, id: &str) -> Option<&mut Library> {
        self.user.iter_mut().find(|u| u.meta.id == id)
    }

    fn library(&self, id: &str) -> Option<&Library> {
        if id == self.builtin.meta.id {
            Some(self.builtin)
        } else {
            self.user.iter().find(|u| u.meta.id == id)
        }
    }

    /// A material by id, from whichever library the id names.
    pub fn material(&self, id: &MaterialId) -> Option<&Arc<Material>> {
        self.library(id.library())?.material(id)
    }

    /// An appearance by id.
    pub fn appearance(&self, id: &AppearanceId) -> Option<&Arc<Appearance>> {
        self.library(id.library())?.appearance(id)
    }

    /// Materials matching `q`, ordered by library (built-in first), then name, then id.
    pub fn search(&self, q: &MaterialQuery) -> Vec<&Arc<Material>> {
        let needle = q.text.as_deref().map(str::to_ascii_lowercase);
        let needle = needle.as_deref().filter(|n| !n.is_empty());
        let mut out = Vec::new();
        for lib in self.libraries() {
            let start = out.len();
            out.extend(lib.materials().filter(|m| matches_query(m, q, needle)));
            out[start..].sort_unstable_by(|a, b| a.name.cmp(&b.name).then_with(|| a.id.cmp(&b.id)));
        }
        out
    }

    /// Import matching (3MF `base/@name`, STEP names). In order: case-insensitive exact name,
    /// then `am.slicer_type` equal to the name or one of its words, then a family keyword among
    /// the words (`PETG Basic` finds a PETG material). Within a step user libraries win over the
    /// built-in one; ties go to the lowest id.
    pub fn match_name(&self, name: &str) -> Option<&Arc<Material>> {
        let name = name.trim();
        if name.is_empty() {
            return None;
        }
        // User libraries first, built-in last.
        let find = |pred: &dyn Fn(&Material) -> bool| {
            self.user
                .iter()
                .chain(std::iter::once(self.builtin))
                .find_map(|lib| lib.materials().find(|m| pred(m)))
        };
        if let Some(m) = find(&|m| m.name.eq_ignore_ascii_case(name)) {
            return Some(m);
        }
        let full = squash(name);
        let words: Vec<String> = name
            .split(|c: char| !c.is_ascii_alphanumeric() && c != '-')
            .map(squash)
            .filter(|w| !w.is_empty())
            .collect();
        let keys = || std::iter::once(&full).chain(words.iter());
        for key in keys() {
            let hit = find(&|m| {
                m.am.as_ref()
                    .and_then(|a| a.slicer_type.as_deref())
                    .is_some_and(|t| squash(t) == *key)
            });
            if hit.is_some() {
                return hit;
            }
        }
        for key in keys() {
            let hit = find(&|m| {
                m.family
                    .as_ref()
                    .is_some_and(|f| squash(&f.as_str()) == *key)
            });
            if hit.is_some() {
                return hit;
            }
        }
        None
    }

    /// Compares a document's copy with the library entry of the same id.
    pub fn drift(&self, doc_copy: &Material) -> Drift {
        let Some(lib) = self.material(&doc_copy.id) else {
            return Drift::NotInLibrary;
        };
        if lib.content_hash() == doc_copy.content_hash() {
            Drift::UpToDate
        } else if lib.revision > doc_copy.revision {
            Drift::LibraryNewer {
                library_revision: lib.revision,
            }
        } else {
            Drift::LocallyModified
        }
    }
}

/// Lower-case alphanumerics only (`PA-CF`, `pa_cf` and `PACF` compare equal).
fn squash(s: &str) -> String {
    s.chars()
        .filter(char::is_ascii_alphanumeric)
        .map(|c| c.to_ascii_lowercase())
        .collect()
}

/// ASCII case-insensitive substring test; `needle` is already lower-case.
fn contains_ci(hay: &str, needle: &str) -> bool {
    let (h, n) = (hay.as_bytes(), needle.as_bytes());
    n.len() <= h.len() && h.windows(n.len()).any(|w| w.eq_ignore_ascii_case(n))
}

fn matches_query(m: &Material, q: &MaterialQuery, needle: Option<&str>) -> bool {
    if q.category.is_some_and(|c| c != m.category) {
        return false;
    }
    if let Some(p) = q.process
        && m.am.as_ref().map(|a| a.process) != Some(p)
    {
        return false;
    }
    if let Some(f) = &q.family
        && m.family.as_ref() != Some(f)
    {
        return false;
    }
    needle.is_none_or(|n| {
        contains_ci(&m.name, n)
            || contains_ci(m.id.as_str(), n)
            || m.description.as_deref().is_some_and(|d| contains_ci(d, n))
            || m.tags.iter().any(|t| contains_ci(t, n))
    })
}
