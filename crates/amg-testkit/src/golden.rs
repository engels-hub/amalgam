//! Golden geometry files: `tests/golden/<suite>/<case>.toml`.
//!
//! A file holds the expected [`MeshMetrics`] and the [`GoldenTolerances`] used to compare
//! them. When `hausdorff_abs` is set, the blessed mesh is also stored next to it as
//! `<case>.stl` (binary STL, git-LFS) and the current mesh must stay within that distance of
//! it. `AMG_BLESS=1` rewrites both files from the current result and keeps the existing
//! tolerances, so they can be tuned by hand once and survive later blesses.
//!
//! Unknown keys are errors, so a typo (`hausdorff_ab`, `[tolerance]`) fails loudly instead of
//! silently disabling a check. Blessing is refused when the `CI` variable is set.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use crate::metrics::{MeshMetrics, metrics};
use crate::{TriSoup, env, hausdorff, stl, testdata};

/// Surface samples per mesh for the golden Hausdorff check.
const HAUSDORFF_SAMPLES: usize = 20_000;

/// Per-metric tolerances of a golden file. Counts (vertices, triangles, edges, Euler) are
/// always compared exactly.
///
/// Volume and area pass when `|actual - expected| <= rel * max(|expected|, scale)`, where
/// `scale` is `d³` for volume and `d²` for area and `d` is the expected bbox diagonal. The
/// floor keeps near-zero values (open or flat soups) from turning round-off into a huge
/// relative error.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GoldenTolerances {
    /// Relative volume tolerance (see the type docs for the near-zero floor).
    pub volume_rel: f64,
    /// Relative area tolerance (see the type docs for the near-zero floor).
    pub area_rel: f64,
    /// Absolute tolerance on every bbox coordinate, mm.
    pub bbox_abs: f64,
    /// If set, the maximum Hausdorff distance to the blessed reference mesh, mm. The
    /// reference is stored in f32, so values below ~1e-6 × model size are not meaningful.
    pub hausdorff_abs: Option<f64>,
}

impl Default for GoldenTolerances {
    /// 1e-6 relative on volume and area, 1e-6 mm on the bbox, no Hausdorff check.
    fn default() -> Self {
        GoldenTolerances {
            volume_rel: 1e-6,
            area_rel: 1e-6,
            bbox_abs: 1e-6,
            hausdorff_abs: None,
        }
    }
}

/// A golden comparison failed (or the golden file is missing or unreadable).
#[derive(Debug, thiserror::Error)]
#[error("golden mismatch {suite}/{case}: {details}")]
pub struct GoldenMismatch {
    /// Suite (sub-directory of `tests/golden`).
    pub suite: String,
    /// Case (file stem).
    pub case: String,
    /// Human-readable list of the differences.
    pub details: String,
}

const TOLERANCES_KEY: &str = "tolerances";
const METRICS_KEY: &str = "metrics";

#[derive(Debug, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct GoldenFile {
    #[serde(default)]
    tolerances: GoldenTolerances,
    metrics: MeshMetrics,
}

/// How [`check_in`] runs; [`check_golden`] fills it from the environment.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Mode {
    /// `AMG_BLESS=1`.
    pub(crate) bless: bool,
    /// `AMG_REQUIRE_TESTDATA=1`: an un-fetched reference mesh fails instead of skipping.
    pub(crate) require_reference: bool,
    /// `CI` is set: blessing is refused.
    pub(crate) ci: bool,
}

/// `<workspace>/tests/golden`.
pub fn golden_dir() -> PathBuf {
    crate::workspace_root().join("tests").join("golden")
}

/// Compares `m` against `tests/golden/<suite>/<case>.toml`. With `AMG_BLESS=1` it writes the
/// file (and the `.stl` reference if a Hausdorff tolerance is set, deleting a stale one if
/// not) and returns `Ok`.
///
/// Prefer [`assert_golden!`](crate::assert_golden) in tests. Test functions that call it
/// should be named `golden_*` so CI's `golden` job (`-E 'test(golden_)'`) runs them.
///
/// # Errors
/// [`GoldenMismatch`] listing every metric outside its tolerance, or explaining why the file
/// could not be read (including unknown keys).
///
/// # Panics
/// If `suite` or `case` is not a plain name, or when blessing: if `CI` is set, if the
/// existing file's `[tolerances]` (or its top level) is invalid, or if writing fails.
pub fn check_golden(suite: &str, case: &str, m: TriSoup<'_>) -> Result<(), GoldenMismatch> {
    check_in(
        &golden_dir(),
        suite,
        case,
        m,
        Mode {
            bless: env::flag(env::BLESS),
            require_reference: env::flag(env::REQUIRE_TESTDATA),
            ci: std::env::var_os("CI").is_some_and(|v| !v.is_empty()),
        },
    )
}

pub(crate) fn check_in(
    root: &Path,
    suite: &str,
    case: &str,
    m: TriSoup<'_>,
    mode: Mode,
) -> Result<(), GoldenMismatch> {
    for name in [suite, case] {
        assert!(
            !name.is_empty()
                && name
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '.'))
                && !name.starts_with('.'),
            "golden suite/case must be a plain name ([A-Za-z0-9_.-]), got {name:?}"
        );
    }
    let dir = root.join(suite);
    let toml_path = dir.join(format!("{case}.toml"));
    let stl_path = dir.join(format!("{case}.stl"));
    let mismatch = |details: String| GoldenMismatch {
        suite: suite.to_owned(),
        case: case.to_owned(),
        details,
    };

    if mode.bless {
        bless(&dir, &toml_path, &stl_path, suite, case, m, mode.ci);
        return Ok(());
    }

    let expected = match read(&toml_path) {
        Ok(Some(f)) => f,
        Ok(None) => {
            return Err(mismatch(format!(
                "{} does not exist; run the test with AMG_BLESS=1 to create it",
                toml_path.display()
            )));
        }
        Err(e) => return Err(mismatch(e)),
    };

    let actual = metrics(m);
    let mut diffs = compare(&expected.metrics, &actual, &expected.tolerances);
    if let Some(tol) = expected.tolerances.hausdorff_abs {
        if !stl_path.is_file() {
            diffs.push(format!(
                "reference mesh {} is missing; re-bless with AMG_BLESS=1",
                stl_path.display()
            ));
        } else if testdata::is_lfs_pointer(&stl_path) {
            assert!(
                !mode.require_reference,
                "{} is an un-fetched git-LFS pointer and {}=1 is set; run `git lfs pull`",
                stl_path.display(),
                env::REQUIRE_TESTDATA
            );
            eprintln!(
                "SKIP: Hausdorff check of {suite}/{case}: {} is an un-fetched git-LFS pointer",
                stl_path.display()
            );
        } else {
            match stl::read(&stl_path) {
                Ok(reference) => {
                    let h = hausdorff(m, reference.soup(), HAUSDORFF_SAMPLES);
                    if !within(h, tol) {
                        diffs.push(format!("hausdorff to reference: {h:e} mm > {tol:e} mm"));
                    }
                }
                Err(e) => diffs.push(format!("cannot read reference mesh: {e}")),
            }
        }
    }

    if diffs.is_empty() {
        Ok(())
    } else {
        let mut details = String::new();
        for d in &diffs {
            let _ = write!(details, "\n  {d}");
        }
        let _ = write!(
            details,
            "\n  (file: {}; re-bless with AMG_BLESS=1 if the change is intended)",
            toml_path.display()
        );
        Err(mismatch(details))
    }
}

fn bless(
    dir: &Path,
    toml_path: &Path,
    stl_path: &Path,
    suite: &str,
    case: &str,
    m: TriSoup<'_>,
    ci: bool,
) {
    assert!(
        !ci,
        "refusing to bless {suite}/{case} on CI (the CI variable is set); bless locally and \
         commit the reviewed files"
    );
    let tolerances = match read_tolerances(toml_path) {
        Ok(t) => t,
        Err(e) => panic!("refusing to bless {suite}/{case}: {e}"),
    };
    std::fs::create_dir_all(dir).expect("create golden directory");
    if tolerances.hausdorff_abs.is_some() {
        stl::write(stl_path, m).expect("write golden reference mesh");
    } else if stl_path.exists() {
        std::fs::remove_file(stl_path).expect("remove stale golden reference mesh");
        eprintln!("REMOVED stale reference: {}", stl_path.display());
    }
    let file = GoldenFile {
        tolerances,
        metrics: metrics(m),
    };
    let body = toml::to_string_pretty(&file).expect("serialise golden file");
    let text = format!(
        "# Golden metrics for {suite}/{case}. Written by `AMG_BLESS=1`; tolerances are kept\n\
         # across blesses and may be edited by hand. Spec: S00 §5.\n\n{body}"
    );
    std::fs::write(toml_path, text).expect("write golden file");
    eprintln!("BLESSED: {}", toml_path.display());
}

/// The tolerances to keep when blessing, read independently of `[metrics]` so a change of the
/// metrics schema cannot drop hand-tuned values. Defaults if the file or the table is absent.
/// `Err` if the file is not TOML, has unknown top-level keys or an invalid `[tolerances]`.
fn read_tolerances(path: &Path) -> Result<GoldenTolerances, String> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Ok(GoldenTolerances::default());
        }
        Err(e) => return Err(format!("cannot read {}: {e}", path.display())),
    };
    let mut table: toml::Table = toml::from_str(&text).map_err(|e| {
        format!(
            "{} is not valid TOML ({e}); fix or delete it first",
            path.display()
        )
    })?;
    if let Some(k) = table
        .keys()
        .find(|k| *k != TOLERANCES_KEY && *k != METRICS_KEY)
    {
        return Err(format!(
            "{} has an unknown top-level key {k:?} (expected [{TOLERANCES_KEY}] and \
             [{METRICS_KEY}])",
            path.display()
        ));
    }
    match table.remove(TOLERANCES_KEY) {
        None => Ok(GoldenTolerances::default()),
        Some(v) => v
            .try_into()
            .map_err(|e| format!("invalid [{TOLERANCES_KEY}] in {}: {e}", path.display())),
    }
}

/// `Ok(None)` if the file does not exist.
fn read(path: &Path) -> Result<Option<GoldenFile>, String> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(format!("cannot read {}: {e}", path.display())),
    };
    toml::from_str(&text)
        .map(Some)
        .map_err(|e| format!("cannot parse {}: {e}", path.display()))
}

/// `value <= tol`; false for NaN, so a NaN metric is always a mismatch.
fn within(value: f64, tol: f64) -> bool {
    value <= tol
}

fn compare(e: &MeshMetrics, a: &MeshMetrics, t: &GoldenTolerances) -> Vec<String> {
    let mut out = Vec::new();
    let d = if e.bbox.is_empty() {
        0.0
    } else {
        e.bbox.diagonal()
    };
    let rel = |name: &str, exp: f64, act: f64, tol: f64, scale: f64, out: &mut Vec<String>| {
        let allowed = tol * exp.abs().max(scale);
        let dev = (act - exp).abs();
        if !within(dev, allowed) {
            out.push(format!(
                "{name}: expected {exp}, got {act} (deviation {dev:e} > {tol:e} × \
                 max(|expected|, {scale:e}) = {allowed:e})"
            ));
        }
    };
    rel(
        "volume",
        e.volume,
        a.volume,
        t.volume_rel,
        d * d * d,
        &mut out,
    );
    rel("area", e.area, a.area, t.area_rel, d * d, &mut out);

    if e.bbox.is_empty() != a.bbox.is_empty() {
        out.push(format!("bbox: expected {:?}, got {:?}", e.bbox, a.bbox));
    } else if !e.bbox.is_empty() {
        let dev = (e.bbox.min - a.bbox.min)
            .abs()
            .max((e.bbox.max - a.bbox.max).abs())
            .max_element();
        if !within(dev, t.bbox_abs) {
            out.push(format!(
                "bbox: expected {:?}..{:?}, got {:?}..{:?} (deviation {dev:e} > {:e} mm)",
                e.bbox.min.to_array(),
                e.bbox.max.to_array(),
                a.bbox.min.to_array(),
                a.bbox.max.to_array(),
                t.bbox_abs
            ));
        }
    }

    let counts = [
        ("vertices", e.vertices as i64, a.vertices as i64),
        ("triangles", e.triangles as i64, a.triangles as i64),
        (
            "boundary_edges",
            e.boundary_edges as i64,
            a.boundary_edges as i64,
        ),
        (
            "non_manifold_edges",
            e.non_manifold_edges as i64,
            a.non_manifold_edges as i64,
        ),
        ("euler", e.euler, a.euler),
    ];
    for (name, exp, act) in counts {
        if exp != act {
            out.push(format!("{name}: expected {exp}, got {act}"));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use amg_core::{Aabb3, Point3};

    use super::*;
    use crate::shapes;
    use crate::testdata::tests::{POINTER, scratch};

    const CHECK: Mode = Mode {
        bless: false,
        require_reference: false,
        ci: false,
    };
    const BLESS: Mode = Mode {
        bless: true,
        require_reference: false,
        ci: false,
    };
    const REQUIRE: Mode = Mode {
        bless: false,
        require_reference: true,
        ci: false,
    };

    const VALID_METRICS: &str = "[metrics]\nvolume = 0.0\narea = 0.0\nvertices = 0\n\
        triangles = 0\nboundary_edges = 0\nnon_manifold_edges = 0\neuler = 0\n\
        [metrics.bbox]\nmin = [0.0, 0.0, 0.0]\nmax = [0.0, 0.0, 0.0]\n";

    fn panic_message(r: std::thread::Result<()>) -> String {
        let e = r.expect_err("expected a panic");
        e.downcast_ref::<String>()
            .cloned()
            .or_else(|| e.downcast_ref::<&str>().map(|s| (*s).to_owned()))
            .unwrap_or_default()
    }

    #[test]
    fn bless_then_pass_then_detect_change() {
        let root = scratch("golden-bless");
        let cube = shapes::cube(Point3::ZERO, 10.0);

        let missing = check_in(&root, "s", "cube", cube.soup(), CHECK).unwrap_err();
        assert!(missing.details.contains("AMG_BLESS=1"), "{missing}");

        check_in(&root, "s", "cube", cube.soup(), BLESS).unwrap();
        let text = std::fs::read_to_string(root.join("s/cube.toml")).unwrap();
        assert!(text.contains("volume = 1000.0"), "{text}");
        check_in(&root, "s", "cube", cube.soup(), CHECK).unwrap();

        let bigger = shapes::cube(Point3::ZERO, 10.1);
        let err = check_in(&root, "s", "cube", bigger.soup(), CHECK).unwrap_err();
        for field in ["volume", "area", "bbox"] {
            assert!(err.details.contains(field), "{err}");
        }
        assert!(!err.details.contains("euler"), "{err}");
        assert_eq!((err.suite.as_str(), err.case.as_str()), ("s", "cube"));
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn tolerances_survive_bless_and_drive_comparison() {
        let root = scratch("golden-tolerances");
        let cube = shapes::cube(Point3::ZERO, 10.0);
        check_in(&root, "s", "c", cube.soup(), BLESS).unwrap();
        // Loosen tolerances by hand, as a reviewer would.
        let path = root.join("s/c.toml");
        let mut f: GoldenFile = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        f.tolerances = GoldenTolerances {
            volume_rel: 0.05,
            area_rel: 0.05,
            bbox_abs: 0.2,
            hausdorff_abs: None,
        };
        std::fs::write(&path, toml::to_string(&f).unwrap()).unwrap();
        let close = shapes::cube(Point3::ZERO, 10.1);
        check_in(&root, "s", "c", close.soup(), CHECK).unwrap();
        check_in(&root, "s", "c", close.soup(), BLESS).unwrap();
        let again: GoldenFile = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(again.tolerances, f.tolerances, "bless keeps tolerances");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn bless_keeps_tolerances_when_metrics_schema_changed() {
        let root = scratch("golden-schema");
        std::fs::create_dir_all(root.join("s")).unwrap();
        let path = root.join("s/c.toml");
        // `[metrics]` from an older schema (an extra field, missing ones): unparsable as a
        // whole, but the tolerances must survive the re-bless.
        std::fs::write(
            &path,
            "[tolerances]\nvolume_rel = 0.25\n\n[metrics]\nvolume = 1.0\nold_field = 3\n",
        )
        .unwrap();
        let cube = shapes::cube(Point3::ZERO, 1.0);
        let err = check_in(&root, "s", "c", cube.soup(), CHECK).unwrap_err();
        assert!(err.details.contains("cannot parse"), "{err}");
        check_in(&root, "s", "c", cube.soup(), BLESS).unwrap();
        let f: GoldenFile = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(
            f.tolerances,
            GoldenTolerances {
                volume_rel: 0.25,
                ..GoldenTolerances::default()
            }
        );
        check_in(&root, "s", "c", cube.soup(), CHECK).unwrap();
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn typos_fail_loudly() {
        let root = scratch("golden-typos");
        std::fs::create_dir_all(root.join("s")).unwrap();
        let cube = shapes::cube(Point3::ZERO, 1.0);
        for (case, text, needle) in [
            (
                "field",
                format!("[tolerances]\nhausdorff_ab = 0.1\n\n{VALID_METRICS}"),
                "hausdorff_ab",
            ),
            (
                "table",
                format!("[tolerance]\nhausdorff_abs = 0.1\n\n{VALID_METRICS}"),
                "tolerance",
            ),
        ] {
            let path = root.join(format!("s/{case}.toml"));
            std::fs::write(&path, &text).unwrap();
            let err = check_in(&root, "s", case, cube.soup(), CHECK).unwrap_err();
            assert!(err.details.contains("unknown"), "{err}");
            assert!(err.details.contains(needle), "{err}");
            // Blessing must not paper over the typo either.
            let msg = panic_message(std::panic::catch_unwind(|| {
                let _ = check_in(&root, "s", case, cube.soup(), BLESS);
            }));
            assert!(msg.contains("refusing to bless"), "{msg}");
            assert_eq!(
                std::fs::read_to_string(&path).unwrap(),
                text,
                "file untouched"
            );
        }
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn bless_refuses_invalid_toml_and_ci() {
        let root = scratch("golden-refuse");
        std::fs::create_dir_all(root.join("s")).unwrap();
        std::fs::write(root.join("s/x.toml"), "not = [valid").unwrap();
        let cube = shapes::cube(Point3::ZERO, 1.0);
        let msg = panic_message(std::panic::catch_unwind(|| {
            let _ = check_in(&root, "s", "x", cube.soup(), BLESS);
        }));
        assert!(msg.contains("not valid TOML"), "{msg}");

        let ci = Mode { ci: true, ..BLESS };
        let msg = panic_message(std::panic::catch_unwind(|| {
            let _ = check_in(&root, "s", "new", cube.soup(), ci);
        }));
        assert!(msg.contains("on CI"), "{msg}");
        assert!(!root.join("s/new.toml").exists());
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn hausdorff_reference() {
        let root = scratch("golden-hausdorff");
        let dir = root.join("s");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            dir.join("sph.toml"),
            format!("[tolerances]\nhausdorff_abs = 0.01\n\n{VALID_METRICS}"),
        )
        .unwrap();
        let sphere = shapes::icosphere(Point3::ZERO, 5.0, 3);
        check_in(&root, "s", "sph", sphere.soup(), BLESS).unwrap();
        assert!(dir.join("sph.stl").is_file(), "reference written on bless");
        check_in(&root, "s", "sph", sphere.soup(), CHECK).unwrap();

        // Moved by 0.1 mm: same volume and area, but the reference check catches it.
        let mut moved = sphere.clone();
        for p in &mut moved.positions {
            p.x += 0.1;
        }
        let err = check_in(&root, "s", "sph", moved.soup(), CHECK).unwrap_err();
        assert!(err.details.contains("hausdorff"), "{err}");

        // Un-fetched reference: skipped locally, fails when required.
        std::fs::write(dir.join("sph.stl"), POINTER).unwrap();
        check_in(&root, "s", "sph", sphere.soup(), CHECK).unwrap();
        let required =
            std::panic::catch_unwind(|| check_in(&root, "s", "sph", sphere.soup(), REQUIRE));
        assert!(required.is_err());

        std::fs::remove_file(dir.join("sph.stl")).unwrap();
        let err = check_in(&root, "s", "sph", sphere.soup(), CHECK).unwrap_err();
        assert!(err.details.contains("missing"), "{err}");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn bless_removes_stale_reference() {
        let root = scratch("golden-stale");
        let dir = root.join("s");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("c.toml");
        std::fs::write(
            &path,
            format!("[tolerances]\nhausdorff_abs = 0.01\n\n{VALID_METRICS}"),
        )
        .unwrap();
        let cube = shapes::cube(Point3::ZERO, 1.0);
        check_in(&root, "s", "c", cube.soup(), BLESS).unwrap();
        assert!(dir.join("c.stl").is_file());
        // The reviewer drops the Hausdorff check; the next bless removes the reference.
        let text = std::fs::read_to_string(&path).unwrap();
        std::fs::write(&path, text.replace("hausdorff_abs = 0.01\n", "")).unwrap();
        check_in(&root, "s", "c", cube.soup(), BLESS).unwrap();
        assert!(!dir.join("c.stl").exists(), "stale reference removed");
        check_in(&root, "s", "c", cube.soup(), CHECK).unwrap();
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn near_zero_values_use_the_bbox_scale() {
        let mut e = metrics(shapes::cube(Point3::ZERO, 10.0).soup());
        // A flat soup: expected volume is round-off, the actual one different round-off.
        e.volume = 1e-20;
        let mut a = e.clone();
        a.volume = -3e-14;
        let t = GoldenTolerances::default();
        assert!(compare(&e, &a, &t).is_empty(), "{:?}", compare(&e, &a, &t));
        // d = sqrt(300), d³ ≈ 5196: a deviation of 0.01 mm³ is above 1e-6 × d³.
        a.volume = 0.01;
        let diffs = compare(&e, &a, &t);
        assert!(
            diffs.len() == 1 && diffs[0].starts_with("volume"),
            "{diffs:?}"
        );
        // Cube: the d³ floor (≈ 5196) dominates, so 1000 ± 1e-4 passes and ± 1e-2 does not.
        let mut big = e.clone();
        big.volume = 1000.0;
        let mut b = big.clone();
        b.volume = 1000.0 + 1e-4;
        b.bbox = Aabb3::new(Point3::ZERO, Point3::splat(10.0));
        assert!(compare(&big, &b, &t).is_empty());
        b.volume = 1000.0 + 1e-2;
        assert_eq!(compare(&big, &b, &t).len(), 1);
    }

    #[test]
    fn unparsable_file_is_a_mismatch() {
        let root = scratch("golden-bad");
        std::fs::create_dir_all(root.join("s")).unwrap();
        std::fs::write(root.join("s/x.toml"), "not = [valid").unwrap();
        let cube = shapes::cube(Point3::ZERO, 1.0);
        let err = check_in(&root, "s", "x", cube.soup(), CHECK).unwrap_err();
        assert!(err.details.contains("cannot parse"), "{err}");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    #[should_panic(expected = "plain name")]
    fn path_like_names_are_rejected() {
        let cube = shapes::cube(Point3::ZERO, 1.0);
        let _ = check_in(Path::new("."), "../x", "c", cube.soup(), CHECK);
    }

    #[test]
    fn golden_dir_is_in_workspace() {
        assert!(golden_dir().ends_with("tests/golden"));
        assert!(golden_dir().is_dir());
    }

    /// The checked-in golden file `tests/golden/testkit/cube10.toml`.
    #[test]
    fn golden_testkit_cube10() {
        let cube = shapes::cube(Point3::ZERO, 10.0);
        crate::assert_golden!("testkit", "cube10", cube.soup());
    }

    /// The checked-in golden file `tests/golden/testkit/icosphere_r10_s4.toml`.
    #[test]
    fn golden_testkit_icosphere() {
        let s = shapes::icosphere(Point3::ZERO, 10.0, 4);
        crate::assert_golden!("testkit", "icosphere_r10_s4", s.soup());
    }
}
