//! Golden geometry files: `tests/golden/<suite>/<case>.toml`.
//!
//! A file holds the expected [`MeshMetrics`] and the [`GoldenTolerances`] used to compare
//! them. When `hausdorff_abs` is set, the blessed mesh is also stored next to it as
//! `<case>.stl` (binary STL, git-LFS) and the current mesh must stay within that distance of
//! it. `AMG_BLESS=1` rewrites both files from the current result and keeps the existing
//! tolerances, so they can be tuned by hand once and survive later blesses.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use crate::metrics::{MeshMetrics, metrics};
use crate::{TriSoup, env, hausdorff, stl, testdata};

/// Surface samples per mesh for the golden Hausdorff check.
const HAUSDORFF_SAMPLES: usize = 20_000;

/// Per-metric tolerances of a golden file. Counts (vertices, triangles, edges, Euler) are
/// always compared exactly.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct GoldenTolerances {
    /// Relative volume tolerance (absolute when the expected volume is 0).
    pub volume_rel: f64,
    /// Relative area tolerance (absolute when the expected area is 0).
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

#[derive(Debug, serde::Serialize, serde::Deserialize)]
struct GoldenFile {
    #[serde(default)]
    tolerances: GoldenTolerances,
    metrics: MeshMetrics,
}

/// `<workspace>/tests/golden`.
pub fn golden_dir() -> PathBuf {
    crate::workspace_root().join("tests").join("golden")
}

/// Compares `m` against `tests/golden/<suite>/<case>.toml`. With `AMG_BLESS=1` it writes the
/// file (and the `.stl` reference if a Hausdorff tolerance is set) and returns `Ok`.
///
/// Prefer [`assert_golden!`](crate::assert_golden) in tests. Test functions that call it
/// should be named `golden_*` so CI's `golden` job (`-E 'test(golden_)'`) runs them.
///
/// # Errors
/// [`GoldenMismatch`] listing every metric outside its tolerance, or explaining why the file
/// could not be read.
///
/// # Panics
/// If `suite` or `case` is not a plain name, or blessing cannot write the file.
pub fn check_golden(suite: &str, case: &str, m: TriSoup<'_>) -> Result<(), GoldenMismatch> {
    check_in(
        &golden_dir(),
        suite,
        case,
        m,
        env::flag(env::BLESS),
        env::flag(env::REQUIRE_TESTDATA),
    )
}

pub(crate) fn check_in(
    root: &Path,
    suite: &str,
    case: &str,
    m: TriSoup<'_>,
    bless: bool,
    require_reference: bool,
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

    let existing = read(&toml_path);
    let actual = metrics(m);

    if bless {
        let tolerances = match existing {
            Ok(Some(f)) => f.tolerances,
            _ => GoldenTolerances::default(),
        };
        std::fs::create_dir_all(&dir).expect("create golden directory");
        if tolerances.hausdorff_abs.is_some() {
            stl::write(&stl_path, m).expect("write golden reference mesh");
        }
        let file = GoldenFile {
            tolerances,
            metrics: actual,
        };
        let body = toml::to_string_pretty(&file).expect("serialise golden file");
        let text = format!(
            "# Golden metrics for {suite}/{case}. Written by `AMG_BLESS=1`; tolerances are kept\n\
             # across blesses and may be edited by hand. Spec: S00 §5.\n\n{body}"
        );
        std::fs::write(&toml_path, text).expect("write golden file");
        eprintln!("BLESSED: {}", toml_path.display());
        return Ok(());
    }

    let expected = match existing {
        Ok(Some(f)) => f,
        Ok(None) => {
            return Err(mismatch(format!(
                "{} does not exist; run the test with AMG_BLESS=1 to create it",
                toml_path.display()
            )));
        }
        Err(e) => return Err(mismatch(e)),
    };

    let mut diffs = compare(&expected.metrics, &actual, &expected.tolerances);
    if let Some(tol) = expected.tolerances.hausdorff_abs {
        if !stl_path.is_file() {
            diffs.push(format!(
                "reference mesh {} is missing; re-bless with AMG_BLESS=1",
                stl_path.display()
            ));
        } else if testdata::is_lfs_pointer(&stl_path) {
            assert!(
                !require_reference,
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
    let rel = |name: &str, exp: f64, act: f64, tol: f64, out: &mut Vec<String>| {
        let err = if exp == 0.0 {
            (act - exp).abs()
        } else {
            ((act - exp) / exp).abs()
        };
        if !within(err, tol) {
            out.push(format!(
                "{name}: expected {exp}, got {act} (relative error {err:e} > {tol:e})"
            ));
        }
    };
    rel("volume", e.volume, a.volume, t.volume_rel, &mut out);
    rel("area", e.area, a.area, t.area_rel, &mut out);

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
    use amg_core::Point3;

    use super::*;
    use crate::shapes;
    use crate::testdata::tests::{POINTER, scratch};

    #[test]
    fn bless_then_pass_then_detect_change() {
        let root = scratch("golden-bless");
        let cube = shapes::cube(Point3::ZERO, 10.0);

        let missing = check_in(&root, "s", "cube", cube.soup(), false, false).unwrap_err();
        assert!(missing.details.contains("AMG_BLESS=1"), "{missing}");

        check_in(&root, "s", "cube", cube.soup(), true, false).unwrap();
        let text = std::fs::read_to_string(root.join("s/cube.toml")).unwrap();
        assert!(text.contains("volume = 1000.0"), "{text}");
        check_in(&root, "s", "cube", cube.soup(), false, false).unwrap();

        let bigger = shapes::cube(Point3::ZERO, 10.1);
        let err = check_in(&root, "s", "cube", bigger.soup(), false, false).unwrap_err();
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
        check_in(&root, "s", "c", cube.soup(), true, false).unwrap();
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
        check_in(&root, "s", "c", close.soup(), false, false).unwrap();
        check_in(&root, "s", "c", close.soup(), true, false).unwrap();
        let again: GoldenFile = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(again.tolerances, f.tolerances, "bless keeps tolerances");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn hausdorff_reference() {
        let root = scratch("golden-hausdorff");
        let dir = root.join("s");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            dir.join("sph.toml"),
            "[tolerances]\nhausdorff_abs = 0.01\n\n[metrics]\nvolume = 0.0\narea = 0.0\n\
             vertices = 0\ntriangles = 0\nboundary_edges = 0\nnon_manifold_edges = 0\n\
             euler = 0\n[metrics.bbox]\nmin = [0.0, 0.0, 0.0]\nmax = [0.0, 0.0, 0.0]\n",
        )
        .unwrap();
        let sphere = shapes::icosphere(Point3::ZERO, 5.0, 3);
        check_in(&root, "s", "sph", sphere.soup(), true, false).unwrap();
        assert!(dir.join("sph.stl").is_file(), "reference written on bless");
        check_in(&root, "s", "sph", sphere.soup(), false, false).unwrap();

        // Moved by 0.1 mm: same volume and area, but the reference check catches it.
        let mut moved = sphere.clone();
        for p in &mut moved.positions {
            p.x += 0.1;
        }
        let err = check_in(&root, "s", "sph", moved.soup(), false, false).unwrap_err();
        assert!(err.details.contains("hausdorff"), "{err}");

        // Un-fetched reference: skipped locally, fails when required.
        std::fs::write(dir.join("sph.stl"), POINTER).unwrap();
        check_in(&root, "s", "sph", sphere.soup(), false, false).unwrap();
        let required =
            std::panic::catch_unwind(|| check_in(&root, "s", "sph", sphere.soup(), false, true));
        assert!(required.is_err());

        std::fs::remove_file(dir.join("sph.stl")).unwrap();
        let err = check_in(&root, "s", "sph", sphere.soup(), false, false).unwrap_err();
        assert!(err.details.contains("missing"), "{err}");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn unparsable_file_is_a_mismatch() {
        let root = scratch("golden-bad");
        std::fs::create_dir_all(root.join("s")).unwrap();
        std::fs::write(root.join("s/x.toml"), "not = [valid").unwrap();
        let cube = shapes::cube(Point3::ZERO, 1.0);
        let err = check_in(&root, "s", "x", cube.soup(), false, false).unwrap_err();
        assert!(err.details.contains("cannot parse"), "{err}");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    #[should_panic(expected = "plain name")]
    fn path_like_names_are_rejected() {
        let cube = shapes::cube(Point3::ZERO, 1.0);
        let _ = check_in(Path::new("."), "../x", "c", cube.soup(), false, false);
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
