//! The corpus case schema (design note §5.2), `seeds.toml` seed lists and corpus loading.
//!
//! A **case directory** holds `case.toml` ([`CaseToml`], schema 1), `op.json` (a
//! [`KernelOp`] whose `InputRef(n)` is `inputs/<n>.brep`) and `inputs/`. `capture::promote`
//! writes the same layout. A **seed list** (`seeds.toml`, [`SeedFile`]) names synthetic cases
//! as `(generator, seeds)`; they are rebuilt by [`super::fillet`] and never stored as files.
//!
//! ```toml
//! schema = 1
//! id = "fillet/handmade/cube-edge-r1"
//! op = "op.json"
//! source = "handmade"            # handmade | synthetic | owner | harvested:<url> | promoted:<hash>
//! licence = "LicenseRef-Amalgam-Personal"
//! tags = ["analytic", "convex", "smoke"]
//!
//! [expect]
//! outcome = "success"            # success | clean_failure
//! volume = 997.854               # optional reference volume, mm³
//! volume_rel_tol = 1e-6
//! volume_bounds = [990.0, 1000.0] # optional
//! faces = 7                      # optional
//! failure_class = "RadiusTooLarge" # clean_failure only
//! max_radius = 10.0              # optional analytic max radius, mm (KH-03)
//! ```

use std::fs;
use std::io::BufReader;
use std::path::{Path, PathBuf};

use super::fillet::{FilletGenCase, FilletGenerator};
use crate::capture::{EdgeRecord, FailureClass, InputRef, KernelOp};
use crate::error::KernelError;
use crate::kernel::BrepKernel;
use crate::shape::Shape;

/// The `case.toml` and `seeds.toml` schema this build reads and writes.
pub const CASE_SCHEMA: u32 = 1;

/// The default relative volume tolerance when `volume` is given without `volume_rel_tol`.
pub const DEFAULT_VOLUME_REL_TOL: f64 = 1e-6;

/// The env var naming the external corpus root (S00, `amg_testkit::external_testdata`); the
/// fillet owner and harvested cases live in `$AMG_TESTDATA_DIR/kernel/fillet/{owner,harvested}`.
pub const ENV_TESTDATA_DIR: &str = "AMG_TESTDATA_DIR";

/// What a case should give.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExpectOutcome {
    /// A valid result.
    Success,
    /// A classified error: the request is geometrically impossible.
    CleanFailure,
}

/// The `[expect]` table.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Expect {
    /// Success or clean failure.
    pub outcome: ExpectOutcome,
    /// Reference volume, mm³.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub volume: Option<f64>,
    /// Relative tolerance of `volume` (default [`DEFAULT_VOLUME_REL_TOL`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub volume_rel_tol: Option<f64>,
    /// `[lo, hi]` bounds on the result volume, mm³.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub volume_bounds: Option<[f64; 2]>,
    /// Face count of the result.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub faces: Option<u32>,
    /// The failure class a hardened op should report (clean failures).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub failure_class: Option<FailureClass>,
    /// The analytic max workable radius, mm (KH-03 Done-when 4).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_radius: Option<f64>,
}

impl Expect {
    /// `outcome = "success"` and nothing else.
    pub fn success() -> Expect {
        Expect {
            outcome: ExpectOutcome::Success,
            volume: None,
            volume_rel_tol: None,
            volume_bounds: None,
            faces: None,
            failure_class: None,
            max_radius: None,
        }
    }

    /// `outcome = "clean_failure"` with `class`.
    pub fn clean_failure(class: FailureClass) -> Expect {
        Expect {
            outcome: ExpectOutcome::CleanFailure,
            failure_class: Some(class),
            ..Expect::success()
        }
    }

    /// The volume range a valid result must fall in: `volume ± rel_tol·|volume|` intersected
    /// with `volume_bounds`; `None` without either.
    pub fn volume_range(&self) -> Option<(f64, f64)> {
        let exact = self.volume.map(|v| {
            let tol = self.volume_rel_tol.unwrap_or(DEFAULT_VOLUME_REL_TOL) * v.abs().max(1e-9);
            (v - tol, v + tol)
        });
        match (exact, self.volume_bounds) {
            (Some((a, b)), Some([c, d])) => Some((a.max(c), b.min(d))),
            (Some(r), None) => Some(r),
            (None, Some([c, d])) => Some((c, d)),
            (None, None) => None,
        }
    }
}

fn default_op() -> String {
    "op.json".to_owned()
}

/// `case.toml` (design note §5.2, schema 1). `recorded_class` and `kernel_version` are written by
/// `capture::promote`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CaseToml {
    /// [`CASE_SCHEMA`].
    pub schema: u32,
    /// `<corpus>/<kind>/<name>`, for example `fillet/handmade/cube-edge-r1`.
    pub id: String,
    /// The op file in the case directory.
    #[serde(default = "default_op")]
    pub op: String,
    /// `handmade`, `synthetic`, `owner`, `harvested:<url>` or `promoted:<dump hash>`.
    pub source: String,
    /// SPDX id or `LicenseRef-…`.
    pub licence: String,
    /// Subset filters for the harness (`smoke` selects the CI subset).
    #[serde(default)]
    pub tags: Vec<String>,
    /// What the case exercises.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// The failure class recorded in the promoted dump.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub recorded_class: Option<String>,
    /// The kernel version the dump was recorded with.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kernel_version: Option<String>,
    /// The expectation.
    pub expect: Expect,
}

/// Where a case's op comes from.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaseInput {
    /// A case directory with its op file.
    Dir {
        /// The case directory.
        dir: PathBuf,
        /// The op file name in it.
        op: String,
    },
    /// A fillet generator case.
    Fillet(FilletGenCase),
}

/// A fault the harness worker injects instead of (or into) running the case, to test the
/// harness itself (KH-02 Done-when 4 and 5). Injections that end the process are meant for
/// isolated runs only.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Inject {
    /// The worker ends with `STATUS_ACCESS_VIOLATION` (0xC0000005), the exit status of an
    /// unhandled access violation inside OCCT, in the middle of the case.
    AccessViolation,
    /// The worker aborts (`std::process::abort`).
    Abort,
    /// The worker never answers (the parent's timeout kills it).
    Hang,
    /// The op panics.
    Panic,
    /// The op fails with an injected `OpFailed` (a "broken" case).
    NotDone,
}

/// A loaded case, serialisable so the harness can send it to a worker process.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Case {
    /// The id.
    pub id: String,
    /// The source (see [`CaseToml::source`]).
    pub source: String,
    /// Tags.
    pub tags: Vec<String>,
    /// The expectation. For generator cases, [`Prepared::expect`] may complete it.
    pub expect: Expect,
    /// The op's origin.
    pub input: CaseInput,
    /// A harness self-test fault.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub inject: Option<Inject>,
}

/// A case ready to run.
#[derive(Clone, Debug)]
pub struct Prepared {
    /// The op.
    pub op: KernelOp,
    /// Its inputs (`InputRef(n)` = `inputs[n]`).
    pub inputs: Vec<Shape>,
    /// The final expectation.
    pub expect: Expect,
}

/// Why loading or preparing a case failed.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum CaseError {
    /// File system access failed.
    #[error("{path}: {source}")]
    Io {
        /// The file or directory.
        path: PathBuf,
        /// The error.
        source: std::io::Error,
    },
    /// A file does not match its schema.
    #[error("{path}: {message}")]
    Schema {
        /// The file.
        path: PathBuf,
        /// What is wrong.
        message: String,
    },
    /// Two cases have the same id.
    #[error("duplicate case id `{0}`")]
    Duplicate(String),
    /// Building the inputs failed.
    #[error("kernel: {0}")]
    Kernel(#[from] KernelError),
    /// An edge argument of `op.json` cannot be found in the reloaded input.
    #[error("capture: {0}")]
    Capture(#[from] crate::capture::CaptureError),
}

fn io(path: &Path) -> impl FnOnce(std::io::Error) -> CaseError + '_ {
    move |source| CaseError::Io {
        path: path.to_path_buf(),
        source,
    }
}

fn schema(path: &Path, message: impl std::fmt::Display) -> CaseError {
    CaseError::Schema {
        path: path.to_path_buf(),
        message: message.to_string(),
    }
}

impl Case {
    /// Whether the case has `tag`.
    pub fn has_tag(&self, tag: &str) -> bool {
        self.tags.iter().any(|t| t == tag)
    }

    /// The synthetic case of `gen`, with its generator's tags and expectation.
    pub fn synthetic(gen_case: FilletGenCase) -> Case {
        Case {
            id: gen_case.id(),
            source: "synthetic".to_owned(),
            tags: gen_case.tags(),
            expect: gen_case.expect(),
            input: CaseInput::Fillet(gen_case),
            inject: None,
        }
    }

    /// Builds or loads the op and its inputs.
    ///
    /// # Errors
    /// `Io`/`Schema` for an unreadable case directory, `Kernel` when an input cannot be built
    /// or read, `Capture` when an edge of `op.json` has no match in the reloaded input.
    pub fn prepare(&self, k: &dyn BrepKernel) -> Result<Prepared, CaseError> {
        match &self.input {
            CaseInput::Fillet(g) => {
                let built = g.build(k)?;
                let op = KernelOp::Fillet {
                    shape: InputRef(0),
                    edges: built
                        .edges
                        .iter()
                        .map(|(e, spec)| (EdgeRecord::of(&built.input, *e), spec.clone()))
                        .collect(),
                    opts: built.opts,
                };
                Ok(Prepared {
                    op,
                    inputs: vec![built.input],
                    expect: built.expect,
                })
            }
            CaseInput::Dir { dir, op } => {
                let path = dir.join(op);
                let text = fs::read_to_string(&path).map_err(io(&path))?;
                let mut op: KernelOp = serde_json::from_str(&text).map_err(|e| schema(&path, e))?;
                let count = op.inputs().iter().map(|r| r.0 + 1).max().unwrap_or(0);
                let inputs = (0..count)
                    .map(|n| {
                        let p = dir.join("inputs").join(format!("{n}.brep"));
                        let f = fs::File::open(&p).map_err(io(&p))?;
                        Ok(Shape::read_text(BufReader::new(f))?)
                    })
                    .collect::<Result<Vec<_>, CaseError>>()?;
                op.reindex(&inputs)?;
                Ok(Prepared {
                    op,
                    inputs,
                    expect: self.expect.clone(),
                })
            }
        }
    }
}

/// Reads the case directory `dir` (its `case.toml`).
///
/// # Errors
/// `Io` or `Schema` (including a schema version other than [`CASE_SCHEMA`]).
pub fn load_case_dir(dir: &Path) -> Result<Case, CaseError> {
    let path = dir.join("case.toml");
    let text = fs::read_to_string(&path).map_err(io(&path))?;
    let t: CaseToml = toml::from_str(&text).map_err(|e| schema(&path, e))?;
    if t.schema != CASE_SCHEMA {
        return Err(schema(
            &path,
            format!("schema {} (this build reads {CASE_SCHEMA})", t.schema),
        ));
    }
    Ok(Case {
        id: t.id,
        source: t.source,
        tags: t.tags,
        expect: t.expect,
        input: CaseInput::Dir {
            dir: dir.to_path_buf(),
            op: t.op,
        },
        inject: None,
    })
}

/// Every case directory under `root` (a directory with a `case.toml`; not descended into
/// further), in path order. A missing `root` gives no cases.
///
/// # Errors
/// `Io` or `Schema` of the first unreadable case.
pub fn discover(root: &Path) -> Result<Vec<Case>, CaseError> {
    let mut out = Vec::new();
    if root.is_dir() {
        walk(root, &mut out)?;
    }
    Ok(out)
}

fn walk(dir: &Path, out: &mut Vec<Case>) -> Result<(), CaseError> {
    if dir.join("case.toml").is_file() {
        out.push(load_case_dir(dir)?);
        return Ok(());
    }
    let mut subdirs: Vec<PathBuf> = fs::read_dir(dir)
        .map_err(io(dir))?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    subdirs.sort();
    for d in subdirs {
        walk(&d, out)?;
    }
    Ok(())
}

/// One `[[generator]]` entry of `seeds.toml`: seeds `first .. first + count`, the first
/// `smoke` of them tagged `smoke`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SeedEntry {
    /// The generator's name ([`FilletGenerator::name`]).
    pub name: String,
    /// The first seed.
    #[serde(default)]
    pub first: u64,
    /// How many seeds.
    pub count: u64,
    /// How many of the first seeds are in the smoke subset.
    #[serde(default)]
    pub smoke: u64,
    /// Extra tags for every case of this entry.
    #[serde(default)]
    pub tags: Vec<String>,
}

/// `seeds.toml`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SeedFile {
    /// [`CASE_SCHEMA`].
    pub schema: u32,
    /// The generator entries.
    pub generator: Vec<SeedEntry>,
}

impl SeedFile {
    /// The cases of every entry, in file order.
    ///
    /// # Errors
    /// `Schema` for an unknown generator name (with `path` as the file).
    pub fn cases(&self, path: &Path) -> Result<Vec<Case>, CaseError> {
        let mut out = Vec::new();
        for entry in &self.generator {
            let generator = FilletGenerator::from_name(&entry.name)
                .ok_or_else(|| schema(path, format!("unknown generator `{}`", entry.name)))?;
            for seed in entry.first..entry.first + entry.count {
                let mut case = Case::synthetic(FilletGenCase { generator, seed });
                case.tags.extend(entry.tags.iter().cloned());
                if seed < entry.first + entry.smoke {
                    case.tags.push("smoke".to_owned());
                }
                out.push(case);
            }
        }
        Ok(out)
    }
}

/// Reads `seeds.toml` and expands it.
///
/// # Errors
/// `Io` or `Schema`.
pub fn load_seeds(path: &Path) -> Result<Vec<Case>, CaseError> {
    let text = fs::read_to_string(path).map_err(io(path))?;
    let f: SeedFile = toml::from_str(&text).map_err(|e| schema(path, e))?;
    if f.schema != CASE_SCHEMA {
        return Err(schema(
            path,
            format!("schema {} (this build reads {CASE_SCHEMA})", f.schema),
        ));
    }
    f.cases(path)
}

/// Where a corpus lives.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CorpusSource {
    /// Roots searched for case directories.
    pub dirs: Vec<PathBuf>,
    /// Seed lists.
    pub seeds: Vec<PathBuf>,
}

impl CorpusSource {
    /// The committed fillet corpus of this source checkout:
    /// `tests/golden/kernel/fillet/{handmade/, seeds.toml}`, plus
    /// `$AMG_TESTDATA_DIR/kernel/fillet/{owner,harvested}` when that variable is set (missing
    /// directories give no cases).
    pub fn fillet() -> CorpusSource {
        let root = fillet_corpus_dir();
        let mut dirs = vec![root.join("handmade")];
        if let Some(ext) = std::env::var_os(ENV_TESTDATA_DIR).filter(|v| !v.is_empty()) {
            let ext = PathBuf::from(ext).join("kernel").join("fillet");
            dirs.push(ext.join("owner"));
            dirs.push(ext.join("harvested"));
        }
        CorpusSource {
            dirs,
            seeds: vec![root.join("seeds.toml")],
        }
    }

    /// Every case: the seed lists' cases, then the directories' cases, ids unique.
    ///
    /// # Errors
    /// The first loading error, or `Duplicate`.
    pub fn load(&self) -> Result<Vec<Case>, CaseError> {
        let mut cases = Vec::new();
        for s in &self.seeds {
            cases.extend(load_seeds(s)?);
        }
        for d in &self.dirs {
            cases.extend(discover(d)?);
        }
        let mut ids: Vec<&str> = cases.iter().map(|c| c.id.as_str()).collect();
        ids.sort_unstable();
        if let Some(w) = ids.windows(2).find(|w| w[0] == w[1]) {
            return Err(CaseError::Duplicate(w[0].to_owned()));
        }
        Ok(cases)
    }
}

/// `<workspace>/tests/golden/kernel/fillet` of the source checkout this crate was built from.
pub fn fillet_corpus_dir() -> PathBuf {
    golden_kernel_dir().join("fillet")
}

/// `<workspace>/tests/golden/kernel`, holding the corpora and the `*-baseline.toml` files.
pub fn golden_kernel_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("tests")
        .join("golden")
        .join("kernel")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn promoted_case_toml_parses() {
        // The text `capture::promote` writes.
        let text = "schema = 1\n\
            id = \"fillet/promoted/x\"\n\
            op = \"op.json\"\n\
            source = \"promoted:abc\"\n\
            licence = \"LicenseRef-Amalgam-Personal\"\n\
            tags = [\"promoted\", \"fillet\"]\n\
            recorded_class = \"NotDone\"\n\
            kernel_version = \"occt-8.0.1;amg-brep-1;policy-2\"\n\
            \n[expect]\noutcome = \"clean_failure\"\nfailure_class = \"NotDone\"\n";
        let t: CaseToml = toml::from_str(text).unwrap();
        assert_eq!(t.expect.outcome, ExpectOutcome::CleanFailure);
        assert_eq!(t.expect.failure_class, Some(FailureClass::NotDone));
        let back: CaseToml = toml::from_str(&toml::to_string(&t).unwrap()).unwrap();
        assert_eq!(back, t);
    }

    #[test]
    fn unknown_keys_are_rejected() {
        let text = "schema = 1\nid = \"a\"\nsource = \"handmade\"\nlicence = \"x\"\nbogus = 1\n\
                    [expect]\noutcome = \"success\"\n";
        assert!(toml::from_str::<CaseToml>(text).is_err());
    }

    #[test]
    fn volume_range_combines_oracle_and_bounds() {
        let mut e = Expect::success();
        assert_eq!(e.volume_range(), None);
        e.volume = Some(100.0);
        let (lo, hi) = e.volume_range().unwrap();
        assert!((lo - (100.0 - 1e-4)).abs() < 1e-12 && (hi - (100.0 + 1e-4)).abs() < 1e-12);
        e.volume_bounds = Some([99.0, 100.0]);
        assert!((e.volume_range().unwrap().1 - 100.0).abs() < 1e-12);
        e.volume = None;
        assert_eq!(e.volume_range(), Some((99.0, 100.0)));
    }

    #[test]
    fn seed_lists_expand_with_smoke_tags() {
        let f: SeedFile = toml::from_str(
            "schema = 1\n[[generator]]\nname = \"prism_vertical_edges\"\nfirst = 5\ncount = 3\nsmoke = 2\n\
             [[generator]]\nname = \"radius_too_large\"\ncount = 1\ntags = [\"x\"]\n",
        )
        .unwrap();
        let cases = f.cases(Path::new("seeds.toml")).unwrap();
        assert_eq!(cases.len(), 4);
        assert_eq!(cases[0].id, "fillet/synthetic/prism_vertical_edges-0005");
        assert!(cases[0].has_tag("smoke") && cases[1].has_tag("smoke"));
        assert!(!cases[2].has_tag("smoke"));
        assert!(cases[3].has_tag("x") && cases[3].has_tag("radius_too_large"));
        assert_eq!(cases[3].expect.outcome, ExpectOutcome::CleanFailure);
        let json = serde_json::to_string(&cases[0]).unwrap();
        // serde_json may round the last bit of a float; the worker rebuilds generator
        // expectations from the seed anyway.
        let back = serde_json::from_str::<Case>(&json).unwrap();
        assert_eq!(
            (&back.id, &back.tags, &back.input),
            (&cases[0].id, &cases[0].tags, &cases[0].input)
        );
        let bad: SeedFile =
            toml::from_str("schema = 1\n[[generator]]\nname = \"nope\"\ncount = 1\n").unwrap();
        assert!(matches!(
            bad.cases(Path::new("s")),
            Err(CaseError::Schema { .. })
        ));
    }
}
