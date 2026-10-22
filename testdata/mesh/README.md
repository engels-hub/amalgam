# Broken-mesh corpus (MESH-28, S07)

Real models with real defects (holes, non-manifold edges and vertices, bad orientation, inverted
shells, degenerate and duplicate faces, self-intersections, several shells, unwelded seams, scan
soups), plus the manifest the mesh tests iterate. Test data only: nothing here is compiled,
linked or shipped.

```
corpus.toml           manifest, one [[mesh]] entry per model (schema in its header comment)
fetch.tsv             pinned URL + sha256 of every fetch-only model (generated from corpus.toml)
fetch.sh, fetch.ps1   download + verify the fetch-only models into fetch/
real/<source>/        committed models (git-LFS: *.stl, *.ply, *.3mf, *.obj are LFS patterns)
fetch/                NOT committed (git-ignored): output of the fetch scripts
```

## What is where

| Set | Entries | Where | Licence |
|---|---|---|---|
| `real/thingi10k/` | 165 | committed, git-LFS, about 70 MB | CC0-1.0, LicenseRef-PublicDomain, CC-BY-3.0, CC-BY-SA-3.0 (per model, in `corpus.toml` and `real/thingi10k/NOTICE`) |
| `fetch/thingi10k/` | 8 | fetch-only | CC-BY-NC(-SA)-3.0: may not be redistributed commercially or relicensed, so never committed |
| `fetch/stanford/` | 3 | fetch-only | Stanford 3D Scanning Repository terms (research use, no redistribution): bunny reconstruction, dragon (871 k faces), armadillo (346 k faces) |

Budget (note `docs/notes/mesh-processing.md` §6): at most 300 MB and 20 MB per committed file;
`Manifest::validate` and `tests/corpus.rs` enforce the manifest rules, the sizes and the hashes.
Benchmark meshes (100 k / 1 M / 10 M triangles) and the synthetic defect cases are not stored:
`amg_mesh::corpus::bench` and `amg_mesh::corpus::synth` generate them from `(shape, defects, seed)`.

## Fetching the rest

```sh
cd testdata/mesh
CURL_OPTS=--ssl-no-revoke ./fetch.sh          # Git Bash on Windows (schannel); plain ./fetch.sh elsewhere
./fetch.ps1 -CurlOpts '--ssl-no-revoke'       # PowerShell
./fetch.sh stanford-armadillo                 # only some ids
```

Every download is checked against the pinned sha256 before it is unpacked, and the unpacked file
again; downloaded content is never executed. The corpus test skips entries that are absent and
prints which ones; `AMG_REQUIRE_TESTDATA=1` (CI `golden` job) makes an un-pulled LFS pointer a
failure but never a missing fetch-only file.

## Adding a model

1. Redistributable (a licence that allows copying with attribution): copy the file unmodified to
   `real/<source>/`, add a `[[mesh]]` entry with SPDX `license`, `source_url`, `author`, `sha256`,
   `faces` and `defects`, and extend that folder's `NOTICE`. Add a THIRD_PARTY.md row for a new
   source. Otherwise make it fetch-only (`fetch = true`, `fetch_*` fields, a line in `fetch.tsv`).
2. `cargo nextest run -p amg-mesh --test corpus` checks the entry (hash, face count, defect
   flags that can be derived from topology alone).

`defects` are provisional ground truth: derived with exact-weld edge statistics (boundary,
non-manifold, orientation, degenerate, duplicate, components, signed volume), Thingi10K's
self-intersection counts and a 1e-4 x diagonal weld test for `unwelded`. MESH-02 must report the
same flags on 100 % of entries, and the entries where it disagrees get corrected here.

MESH-02 (2026-10-23) reproduces every flag except `self_intersections` (MESH-19) and `hopeless`
(a repair verdict) on all committed entries, after correcting `non_manifold_vertices` (added to
12 entries that have non-manifold edges) and `unwelded` (added to 6, removed from 1). The
evidence is in the S07 refinement log. `cargo nextest run -p amg-mesh --test analysis` keeps it
true.
