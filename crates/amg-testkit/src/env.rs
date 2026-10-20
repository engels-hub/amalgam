//! Environment switches read by the testkit.

/// `AMG_BLESS=1`: golden checks rewrite their files instead of comparing.
pub(crate) const BLESS: &str = "AMG_BLESS";
/// `AMG_REQUIRE_TESTDATA=1`: an un-fetched LFS pointer fails the test instead of skipping.
pub(crate) const REQUIRE_TESTDATA: &str = "AMG_REQUIRE_TESTDATA";
/// `AMG_REQUIRE_GPU=1`: a missing GPU adapter fails the test instead of skipping.
pub(crate) const REQUIRE_GPU: &str = "AMG_REQUIRE_GPU";
/// `AMG_TESTDATA_DIR=<dir>`: root of the large / private corpus kept outside the repo.
pub(crate) const TESTDATA_DIR: &str = "AMG_TESTDATA_DIR";
/// `AMG_WGPU_BACKEND=dx12|vulkan|warp`: backend override (shared with `amg_jobs::gpu`).
pub(crate) const WGPU_BACKEND: &str = "AMG_WGPU_BACKEND";

/// True when `name` is set to `1`, `true` or `yes` (case-insensitive).
pub(crate) fn flag(name: &str) -> bool {
    std::env::var(name).is_ok_and(|v| is_truthy(&v))
}

pub(crate) fn is_truthy(v: &str) -> bool {
    matches!(
        v.trim().to_ascii_lowercase().as_str(),
        "1" | "true" | "yes" | "on"
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn truthy_values() {
        for v in ["1", "true", "TRUE", " yes ", "on"] {
            assert!(is_truthy(v), "{v}");
        }
        for v in ["", "0", "false", "no", "2"] {
            assert!(!is_truthy(v), "{v}");
        }
    }

    #[test]
    fn unset_flag_is_false() {
        assert!(!flag("AMG_TESTKIT_SURELY_UNSET_VARIABLE"));
    }
}
