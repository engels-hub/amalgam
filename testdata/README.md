# Test data

Model files used by tests (S00 §5, FND-06). Two tiers (S00 open question 2):

1. **In the repo, git-LFS (`testdata/`).** A curated, redistributable set: STL, 3MF, STEP,
   OBJ, PLY and `.amg` files (patterns in `/.gitattributes`). Soft cap **2 GiB** in total,
   including golden reference meshes under `tests/golden/`. Budget: S07 mesh corpus ≤ 300 MB,
   S20 kernel corpus, golden files. Every file needs a licence that allows redistribution;
   note the source and licence in the README of its sub-directory.
2. **Outside the repo (`AMG_TESTDATA_DIR`).** Large, private or non-redistributable data:
   scans, big STEP assemblies, the owner's own files. Point `AMG_TESTDATA_DIR` at a local
   folder; tests reach it with `amg_testkit::external_testdata("kernel/loft/owner/x.step")`
   and skip when it is unset or the file is absent. List each file with its sha256 in
   [`external.toml`](external.toml) so others can rebuild the folder.

## Layout

One sub-directory per spec area, for example `mesh/` (S07), `kernel/` (S20), `io/`.

## Using it from a test

```rust
#[test]
fn golden_mesh_repair_hole() {
    let Some(path) = amg_testkit::testdata("mesh/broken/hole.stl") else { return };
    // ...
}
```

`testdata()` panics if the file does not exist. If it is an un-fetched LFS pointer (cloned
without `git lfs pull`), it prints a `SKIP:` notice and returns `None`. CI's `golden` job checks
out with LFS and sets `AMG_REQUIRE_TESTDATA=1`, so a missing object fails there.

## Setup

```sh
git lfs install      # once per machine
git lfs pull         # fetch objects after cloning
```
