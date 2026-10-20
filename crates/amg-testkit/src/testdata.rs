//! Access to test data: the curated git-LFS corpus in `<workspace>/testdata/` and the large or
//! private corpus outside the repo under `AMG_TESTDATA_DIR` (S00 open question 2).

use std::io::Read;
use std::path::{Path, PathBuf};

use crate::env;

/// First line of every git-LFS pointer file.
const LFS_POINTER_PREFIX: &[u8] = b"version https://git-lfs.github.com/spec/v1";
/// Pointer files are a few lines; anything bigger is real content.
const LFS_POINTER_MAX_LEN: u64 = 1024;

/// `<workspace>/testdata`, the git-LFS corpus.
pub fn testdata_dir() -> PathBuf {
    crate::workspace_root().join("testdata")
}

/// Resolves `testdata/<rel>`.
///
/// Returns `None` and prints a skip notice to stderr if the file is an un-fetched git-LFS
/// pointer (run `git lfs pull`). With `AMG_REQUIRE_TESTDATA=1` (CI's `golden` job) it panics
/// instead, so missing LFS objects cannot pass silently.
///
/// ```no_run
/// let Some(path) = amg_testkit::testdata("mesh/broken/hole.stl") else { return };
/// ```
///
/// # Panics
/// If the file does not exist (a typo is a bug, not a skip), or as described above.
pub fn testdata(rel: &str) -> Option<PathBuf> {
    resolve(&testdata_dir(), rel, env::flag(env::REQUIRE_TESTDATA))
}

/// Resolves `$AMG_TESTDATA_DIR/<rel>`, the corpus that is too large or not redistributable
/// for the repo (scans, big STEP assemblies, owner files; listed in `testdata/external.toml`).
///
/// Returns `None` with a skip notice when `AMG_TESTDATA_DIR` is unset or the file is absent.
/// It never fails the test: CI does not have this corpus.
pub fn external_testdata(rel: &str) -> Option<PathBuf> {
    let root = std::env::var_os(env::TESTDATA_DIR).map(PathBuf::from);
    resolve_external(root.as_deref(), rel)
}

pub(crate) fn resolve(root: &Path, rel: &str, require: bool) -> Option<PathBuf> {
    let path = root.join(rel);
    assert!(
        path.is_file(),
        "test data file {} does not exist",
        path.display()
    );
    if is_lfs_pointer(&path) {
        assert!(
            !require,
            "{} is an un-fetched git-LFS pointer and {}=1 is set; run `git lfs pull`",
            path.display(),
            env::REQUIRE_TESTDATA
        );
        eprintln!(
            "SKIP: {} is an un-fetched git-LFS pointer; run `git lfs pull` \
             (set {}=1 to fail instead)",
            path.display(),
            env::REQUIRE_TESTDATA
        );
        return None;
    }
    Some(path)
}

fn resolve_external(root: Option<&Path>, rel: &str) -> Option<PathBuf> {
    let Some(root) = root else {
        eprintln!(
            "SKIP: external test data {rel} needs {} (unset)",
            env::TESTDATA_DIR
        );
        return None;
    };
    let path = root.join(rel);
    if !path.is_file() {
        eprintln!("SKIP: external test data {} is absent", path.display());
        return None;
    }
    Some(path)
}

/// True if `path` is a git-LFS pointer file rather than the object it points to.
pub(crate) fn is_lfs_pointer(path: &Path) -> bool {
    let Ok(meta) = std::fs::metadata(path) else {
        return false;
    };
    if meta.len() > LFS_POINTER_MAX_LEN {
        return false;
    }
    let mut head = [0u8; LFS_POINTER_PREFIX.len()];
    std::fs::File::open(path)
        .and_then(|mut f| f.read_exact(&mut head))
        .is_ok_and(|()| head == LFS_POINTER_PREFIX)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) const POINTER: &str = "version https://git-lfs.github.com/spec/v1\n\
        oid sha256:4d7a214614ab2935c943f9e0ff69d22eadbb8f32b1258daaa5e2ca24d17e2393\n\
        size 12345\n";

    /// A fresh, empty scratch directory for one test.
    pub(crate) fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("amg-testkit-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn pointer_detection() {
        let dir = scratch("pointer-detection");
        std::fs::write(dir.join("p.stl"), POINTER).unwrap();
        std::fs::write(dir.join("real.stl"), b"solid x\nendsolid x\n").unwrap();
        let mut big = POINTER.as_bytes().to_vec();
        big.resize(4096, b' ');
        std::fs::write(dir.join("big.stl"), &big).unwrap();
        assert!(is_lfs_pointer(&dir.join("p.stl")));
        assert!(!is_lfs_pointer(&dir.join("real.stl")));
        assert!(!is_lfs_pointer(&dir.join("big.stl")));
        assert!(!is_lfs_pointer(&dir.join("missing.stl")));

        assert_eq!(resolve(&dir, "real.stl", true), Some(dir.join("real.stl")));
        assert_eq!(resolve(&dir, "p.stl", false), None, "skipped locally");
        let required = std::panic::catch_unwind(|| resolve(&dir, "p.stl", true));
        assert!(
            required.is_err(),
            "AMG_REQUIRE_TESTDATA=1 turns the skip into a failure"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    #[should_panic(expected = "does not exist")]
    fn missing_file_panics() {
        let dir = scratch("missing-file");
        let _ = resolve(&dir, "nope.stl", false);
    }

    #[test]
    fn external_skips_when_unset_or_absent() {
        let dir = scratch("external");
        std::fs::write(dir.join("scan.ply"), b"ply\n").unwrap();
        assert_eq!(resolve_external(None, "scan.ply"), None);
        assert_eq!(resolve_external(Some(&dir), "other.ply"), None);
        assert_eq!(
            resolve_external(Some(&dir), "scan.ply"),
            Some(dir.join("scan.ply"))
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn testdata_dir_has_readme() {
        assert!(testdata_dir().join("README.md").is_file());
        assert_eq!(
            testdata("README.md"),
            Some(testdata_dir().join("README.md"))
        );
    }

    /// Child half of `require_testdata_env_fails_in_a_child_process`: resolves a pointer
    /// file through the env-reading path. Does nothing unless the parent sets the variable.
    #[test]
    fn child_resolve_pointer_from_env() {
        let Some(dir) = std::env::var_os("AMG_TESTKIT_CHILD_DIR") else {
            return;
        };
        let got = resolve(Path::new(&dir), "p.stl", env::flag(env::REQUIRE_TESTDATA));
        assert_eq!(got, None);
    }

    #[test]
    fn require_testdata_env_fails_in_a_child_process() {
        let dir = scratch("require-env");
        std::fs::write(dir.join("p.stl"), POINTER).unwrap();
        let run = |require: Option<&str>| {
            let mut cmd = std::process::Command::new(std::env::current_exe().unwrap());
            cmd.args([
                "--exact",
                "testdata::tests::child_resolve_pointer_from_env",
                "--nocapture",
            ])
            .env("AMG_TESTKIT_CHILD_DIR", &dir)
            .env_remove(env::REQUIRE_TESTDATA);
            if let Some(v) = require {
                cmd.env(env::REQUIRE_TESTDATA, v);
            }
            cmd.output().unwrap()
        };
        let local = run(None);
        let stderr = String::from_utf8_lossy(&local.stderr);
        assert!(local.status.success(), "{stderr}");
        assert!(stderr.contains("SKIP:"), "{stderr}");

        let ci = run(Some("1"));
        let out = String::from_utf8_lossy(&ci.stdout) + String::from_utf8_lossy(&ci.stderr);
        assert!(
            !ci.status.success(),
            "must fail with AMG_REQUIRE_TESTDATA=1:\n{out}"
        );
        assert!(out.contains("un-fetched git-LFS pointer"), "{out}");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
