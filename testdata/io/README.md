# Import/export test data (S12, `amg-io`)

- `corpus.toml`: the manifest (schema in its header). `amg-io`'s `tests/corpus.rs` iterates it
  under nextest.
- `heads/*.fixture`: byte-exact format-sniffing fixtures, a few hundred bytes each (the first
  4 KiB at most). All are synthetic and CC0-1.0: hand-built bytes, nothing copied from other
  projects. They cover binary STL starting with `solid`, STL with a wrong count, ASCII STL with
  BOM/CRLF/tabs, OBJ/PLY/STEP/IGES/DXF/SVG variants, 3MF packages, an Amalgam `.amg` container
  and several non-model files that must not be recognised.
- Fixtures use the `.fixture` extension because the repository's root `.gitattributes` would
  otherwise convert line endings and send `*.stl`/`*.3mf` to git-LFS. `.gitattributes` in this
  directory switches both off for them.
- Corpus tests skip fetch-only entries that are absent and un-fetched git-LFS pointer files,
  unless `AMG_REQUIRE_TESTDATA=1` (then a pointer fails the test).

Larger corpora are added by the reader WPs: the MESH-28 mesh corpus (`testdata/mesh/`), the
3MF Consortium suites and slicer fixtures (IO-03), STEP files (IO-06). See
`docs/notes/import-export-formats.md` §11.

3MF (IO-03):
- `fetch-3mf-suites.ps1` checks out the 3MF Consortium conformance suites 2, 3, 5 and 6
  (BSD-2-Clause, about 600 MB) at a pinned commit into `fetch/3mf-test-suites/` (git-ignored) and
  verifies every case file against `3mf-suites.sha256`. `AMG_3MF_SUITES` points the tests at
  another checkout.
- `3mf-conformance.toml`: the out-of-scope cases, each with its reason.
- `slicer/fixtures.toml`: slicer-produced fixtures made by `scripts/io/make_slicer_fixtures.ps1`
  (CC0-1.0 outputs of CC0 inputs).
