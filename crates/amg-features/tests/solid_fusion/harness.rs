//! The folder walk of the Fusion-parity harness: every `<name>.step` in the folder is matched
//! with the design script of the same name, rebuilt and compared (`compare`). See `README.md`
//! next to this file for the folder layout and the script format.

use std::path::{Path, PathBuf};

use amg_doc::Document;

use super::compare::Comparison;

/// How a design is rebuilt.
#[derive(Copy, Clone)]
pub enum Script {
    /// Adds the design's features to an empty document (and recomputes, as `add` does).
    Build(fn(&mut Document)),
    /// Not rebuildable yet: which feature is missing.
    Pending(&'static str),
}

/// One owner design.
#[derive(Copy, Clone)]
pub struct Design {
    /// The STEP file's stem.
    pub name: &'static str,
    /// The rebuild script.
    pub script: Script,
    /// Why a known difference from Fusion is expected. A difference is then reported instead of
    /// failing the test; the same reason goes into the S04 refinement log.
    pub explained: Option<&'static str>,
}

/// What happened to one design.
#[derive(Clone, Debug)]
pub enum Outcome {
    /// Rebuilt within the bounds.
    Matched(Comparison),
    /// Rebuilt, different, and the script explains why.
    Explained(Comparison, &'static str),
    /// Different without an explanation, or the file or script failed.
    Failed(String),
    /// The script says a feature is missing.
    Pending(&'static str),
    /// A STEP file without a script.
    Unscripted,
    /// A script without a STEP file.
    MissingStep,
    /// The file is an un-fetched git-LFS pointer.
    NotFetched,
}

/// The outcome of every design of one folder.
#[derive(Clone, Debug, Default)]
pub struct Report {
    /// `(design name, outcome)`: STEP files in name order, then scripts without a file.
    pub designs: Vec<(String, Outcome)>,
}

impl Report {
    /// The outcome of `name`.
    pub fn outcome(&self, name: &str) -> Option<&Outcome> {
        self.designs.iter().find(|d| d.0 == name).map(|d| &d.1)
    }

    /// One line per design.
    pub fn lines(&self) -> Vec<String> {
        self.designs
            .iter()
            .map(|(name, o)| match o {
                Outcome::Matched(c) => format!("{name}: MATCH, {}", c.summary()),
                Outcome::Explained(c, why) => format!(
                    "{name}: DIFFERS (explained: {why}), {}; {}",
                    c.summary(),
                    c.mismatches.join("; ")
                ),
                Outcome::Failed(why) => format!("{name}: FAIL, {why}"),
                Outcome::Pending(why) => format!("{name}: SKIP, not rebuildable yet: {why}"),
                Outcome::Unscripted => format!("{name}: SKIP, no design script"),
                Outcome::MissingStep => format!("{name}: SKIP, no {name}.step in the folder"),
                Outcome::NotFetched => format!("{name}: SKIP, un-fetched git-LFS pointer"),
            })
            .collect()
    }

    /// The designs that fail the test. With `strict` (`AMG_REQUIRE_TESTDATA=1`) a STEP file
    /// without a script, a script without a file and an LFS pointer fail too.
    pub fn failures(&self, strict: bool) -> Vec<&str> {
        self.designs
            .iter()
            .filter(|(_, o)| match o {
                Outcome::Failed(_) => true,
                Outcome::Unscripted | Outcome::MissingStep | Outcome::NotFetched => strict,
                Outcome::Matched(_) | Outcome::Explained(..) | Outcome::Pending(_) => false,
            })
            .map(|(n, _)| n.as_str())
            .collect()
    }
}

/// Whether `AMG_REQUIRE_TESTDATA` is set (the workspace's strict corpus switch).
pub fn strict() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| {
        matches!(
            v.trim().to_ascii_lowercase().as_str(),
            "1" | "true" | "yes" | "on"
        )
    })
}

/// The `.step`/`.stp` files directly in `dir`, as `(stem, path)` in name order. `None` if
/// `dir` is not a folder (the harness then skips).
///
/// # Panics
/// If the folder exists but cannot be listed.
pub fn step_files(dir: &Path) -> Option<Vec<(String, PathBuf)>> {
    if !dir.is_dir() {
        return None;
    }
    let mut files: Vec<(String, PathBuf)> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
        .map(|e| {
            e.unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
                .path()
        })
        .filter(|p| {
            p.is_file()
                && p.extension().is_some_and(|x| {
                    x.eq_ignore_ascii_case("step") || x.eq_ignore_ascii_case("stp")
                })
        })
        .filter_map(|p| Some((p.file_stem()?.to_string_lossy().into_owned(), p)))
        .collect();
    files.sort();
    Some(files)
}

#[cfg(feature = "occt")]
pub use occt::{run, step_metrics};

#[cfg(feature = "occt")]
mod occt {
    use std::io::Read;
    use std::panic::{AssertUnwindSafe, catch_unwind};
    use std::path::Path;

    use amg_brep::BrepKernel;
    use amg_doc::{Document, Health};
    use amg_features::io::{ExportSelection, brep_scene_from_document};
    use amg_io::brep::{BrepScene, import_step};
    use amg_jobs::JobCtx;

    use super::super::compare::{Metrics, compare};
    use super::{Design, Outcome, Report, Script, step_files};

    /// Volume, area, bbox and tolerance summed over every part instance of `scene`, placed in
    /// the root frame. Volume and area use the kernel's adaptive integration.
    ///
    /// # Errors
    /// An empty scene, or a kernel refusal.
    pub fn scene_metrics(scene: &BrepScene) -> Result<Metrics, String> {
        let mut m = Metrics {
            solids: 0,
            volume: 0.0,
            area: 0.0,
            min: [f64::INFINITY; 3],
            max: [f64::NEG_INFINITY; 3],
            tolerance: 0.0,
        };
        let instances = scene.instances();
        if instances.is_empty() {
            return Err("no parts".to_owned());
        }
        for (object, transform, _) in instances {
            let shape = scene.objects[object as usize]
                .shape
                .transformed(&transform)
                .map_err(|e| e.to_string())?;
            let props = amg_doc::naming::default_kernel()
                .mass_props(&shape)
                .map_err(|e| e.to_string())?;
            let solids = shape.solids().len();
            m.solids += solids;
            if solids > 0 {
                m.volume += props.volume;
            }
            m.area += props.area;
            let b = shape.bbox();
            for (i, (lo, hi)) in b
                .min
                .to_array()
                .into_iter()
                .zip(b.max.to_array())
                .enumerate()
            {
                m.min[i] = m.min[i].min(lo);
                m.max[i] = m.max[i].max(hi);
            }
            m.tolerance = m.tolerance.max(shape.max_tolerance());
        }
        Ok(m)
    }

    /// The metrics of a STEP file.
    ///
    /// # Errors
    /// An unreadable file, or [`scene_metrics`].
    pub fn step_metrics(path: &Path) -> Result<Metrics, String> {
        let (scene, _) =
            import_step(path, &JobCtx::detached()).map_err(|e| format!("STEP import: {e}"))?;
        scene_metrics(&scene)
    }

    /// Runs a script on an empty document and measures its visible solid bodies.
    ///
    /// # Errors
    /// A panicking script, a feature in error, or [`scene_metrics`].
    pub fn rebuild(name: &str, script: fn(&mut Document)) -> Result<Metrics, String> {
        let mut doc = Document::new(name);
        catch_unwind(AssertUnwindSafe(|| script(&mut doc))).map_err(|p| {
            let msg = p
                .downcast_ref::<String>()
                .map(String::as_str)
                .or_else(|| p.downcast_ref::<&str>().copied())
                .unwrap_or("(no message)");
            format!("script panicked: {msg}")
        })?;
        let errors: Vec<String> = doc
            .design()
            .timeline()
            .nodes()
            .filter_map(|n| match &doc.evaluated().node(n.id)?.health {
                Health::Error(d) => Some(format!("feature {}: {}", n.id.0, d.message)),
                _ => None,
            })
            .collect();
        if !errors.is_empty() {
            return Err(errors.join("; "));
        }
        let scene =
            brep_scene_from_document(doc.design(), doc.evaluated(), &ExportSelection::AllVisible)
                .map_err(|e| e.to_string())?;
        scene_metrics(&scene)
    }

    fn is_lfs_pointer(path: &Path) -> bool {
        const PREFIX: &[u8] = b"version https://git-lfs.github.com/spec/v1";
        let mut head = [0_u8; PREFIX.len()];
        std::fs::File::open(path)
            .and_then(|mut f| f.read_exact(&mut head))
            .is_ok_and(|()| head == PREFIX)
    }

    fn check(path: &Path, design: &Design) -> Outcome {
        if is_lfs_pointer(path) {
            return Outcome::NotFetched;
        }
        let script = match design.script {
            Script::Build(f) => f,
            Script::Pending(why) => return Outcome::Pending(why),
        };
        let compared = step_metrics(path)
            .and_then(|reference| Ok(compare(&reference, &rebuild(design.name, script)?)));
        match compared {
            Err(why) => Outcome::Failed(why),
            Ok(c) if c.mismatches.is_empty() => Outcome::Matched(c),
            Ok(c) => match design.explained {
                Some(why) => Outcome::Explained(c, why),
                None => Outcome::Failed(c.mismatches.join("; ")),
            },
        }
    }

    /// Rebuilds and compares every design of `dir`; `None` if the folder is missing.
    pub fn run(dir: &Path, designs: &[Design]) -> Option<Report> {
        amg_features::io::link();
        let files = step_files(dir)?;
        let mut report = Report::default();
        for (i, (name, path)) in files.iter().enumerate() {
            let outcome = if i > 0 && files[i - 1].0 == *name {
                Outcome::Failed(format!("two STEP files are named {name}"))
            } else {
                match designs.iter().find(|d| d.name == name) {
                    Some(d) => check(path, d),
                    None => Outcome::Unscripted,
                }
            };
            report.designs.push((name.clone(), outcome));
        }
        for d in designs {
            if !files.iter().any(|f| f.0 == d.name) {
                let outcome = match d.script {
                    Script::Build(_) => Outcome::MissingStep,
                    Script::Pending(why) => Outcome::Pending(why),
                };
                report.designs.push((d.name.to_owned(), outcome));
            }
        }
        Some(report)
    }
}
