//! Dev-only test helpers: golden-geometry metrics, test data access and budget benches.
//!
//! Spec: S00 §5 (FND-06, FND-08). Lane: E.
//!
//! Every crate may list `amg-testkit` under `[dev-dependencies]`. It never depends on a domain
//! crate: geometry comes in as a borrowed [`TriSoup`], so a mesh, a B-rep tessellation or an
//! implicit surface adapts with a one-line conversion.
//!
//! - [`metrics`], [`signed_volume`], [`area`] and [`hausdorff`] measure a soup.
//! - [`assert_golden!`] / [`check_golden`] compare against `tests/golden/<suite>/<case>.toml`;
//!   `AMG_BLESS=1` rewrites the file instead.
//! - [`testdata`] resolves files in the git-LFS corpus under `testdata/` and skips on
//!   un-fetched pointer files; [`external_testdata`] resolves files under `AMG_TESTDATA_DIR`.
//! - [`gpu_available`] decides whether a GPU test runs.
//! - [`shapes`] builds reference meshes (cube, icosphere) for tests.
//! - [`budget::BUDGETS`] lists the vision's performance budgets; their criterion benches are
//!   named `budget/<id>`.

#![warn(missing_docs)]
#![allow(
    clippy::expect_used,
    reason = "test helpers; a failure is a test failure"
)]

pub mod budget;
mod bvh;
mod env;
mod golden;
mod gpu;
mod hausdorff;
mod metrics;
pub mod shapes;
mod stl;
mod testdata;

pub use golden::{GoldenMismatch, GoldenTolerances, check_golden, golden_dir};
pub use gpu::gpu_available;
pub use hausdorff::{hausdorff, one_sided_hausdorff};
pub use metrics::{MeshMetrics, area, metrics, signed_volume};
pub use shapes::TriMesh;
pub use testdata::{external_testdata, testdata, testdata_dir};

/// Borrowed triangle soup in mm. Domain crates adapt their meshes to this.
///
/// Topology metrics ([`MeshMetrics::euler`], edge counts) use the indices as given: weld
/// coincident vertices first if the source mesh does not share them.
#[derive(Clone, Copy, Debug)]
pub struct TriSoup<'a> {
    /// Vertex positions, mm.
    pub positions: &'a [amg_core::Point3],
    /// Triangles as indices into `positions`, counter-clockwise seen from outside.
    pub triangles: &'a [[u32; 3]],
}

impl TriSoup<'_> {
    /// The three corners of triangle `t`.
    ///
    /// # Panics
    /// If `t` or one of its indices is out of range.
    #[inline]
    pub fn corners(&self, t: usize) -> [amg_core::Point3; 3] {
        let [a, b, c] = self.triangles[t];
        [
            self.positions[a as usize],
            self.positions[b as usize],
            self.positions[c as usize],
        ]
    }
}

/// Compares `$soup` against `tests/golden/<$suite>/<$case>.toml` and panics with the
/// differences. With `AMG_BLESS=1` it (re)writes the file instead.
///
/// ```no_run
/// let cube = amg_testkit::shapes::cube(amg_core::Point3::ZERO, 10.0);
/// amg_testkit::assert_golden!("mesh", "cube10", cube.soup());
/// ```
#[macro_export]
macro_rules! assert_golden {
    ($suite:expr, $case:expr, $soup:expr $(,)?) => {
        if let ::std::result::Result::Err(e) = $crate::check_golden($suite, $case, $soup) {
            ::std::panic!("{e}");
        }
    };
}

/// Workspace root (two levels above this crate's manifest).
pub(crate) fn workspace_root() -> std::path::PathBuf {
    let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    manifest
        .parent()
        .and_then(std::path::Path::parent)
        .expect("amg-testkit lives in <workspace>/crates/amg-testkit")
        .to_path_buf()
}
