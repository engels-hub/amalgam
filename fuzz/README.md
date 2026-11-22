# amg-io fuzzing (S12 IO-16)

A cargo-fuzz project with its own workspace, outside the main one. Design: S12 §12 and
`docs/notes/import-export-formats.md` §9.3.

| Target | Input | Checks |
|---|---|---|
| `stl` | an STL file | detection, ASCII and binary parsers, exact weld, split solids, face colours, soup; 61-byte chunks agree with the default read |
| `obj` | an OBJ file | two-pass parser, polygons, groups as objects and as face groups; 61-byte chunks / 7-byte blocks agree |
| `mtl` | an MTL file | names trimmed, lines ordered, deterministic |
| `ply` | a PLY file | header, ASCII and both binary orders; 7-byte chunks / 3-record blocks agree |
| `threemf_package` | a 3MF zip | OPC, relationships, models, in normal, strict and keep-components mode; the parallel scanner agrees with the quick-xml reference path |
| `threemf_model` | one model part's XML | as above, inside a stub package (`harness::stub_package`) |
| `paint_decode` | a `paint_color` string | decoder state range; the same bytes as hex digits; `encode_state` round trip |
| `roundtrip_scene` | choices for a scene | scene → 3MF (`Decimals(6)` and `Roundtrip`) → scene equals the original as in IO-11 Done-when 1 |

Every successful import must also give a scene that passes `Scene::validate`, with finite
positions and transforms.

The bodies are in `src/harness.rs`. `crates/amg-io/tests/fuzz_replay.rs` compiles that file by
path and replays `testdata/io/fuzz/<target>/*` (minimised findings) and `fuzz/seeds/<target>/*`
under stable `cargo nextest`, so a fixed bug stays fixed without nightly.

## Running

Needs a nightly toolchain and `cargo install cargo-fuzz`.

```sh
fuzz/run.sh stl 300            # 5 minutes, the nightly CI settings
FUZZ_TARGET_DIR=D:/coding/amalgam-targets/IO-16-fuzz fuzz/run.sh roundtrip_scene 3600
FUZZ_SANITIZER=none fuzz/run.sh paint_decode 600 -runs=10000000
```

`run.sh` passes `-rss_limit_mb=2048 -timeout=2`, the working corpus `fuzz/corpus/<target>/`
(git-ignored), the committed seeds and the existing fixture directories (`testdata/io/heads`,
`testdata/io/slicer`, the fetched 3MF Consortium suites) by path.

### Windows

libFuzzer works on MSVC, but ASan needs the VS "C++ AddressSanitizer" component. Without it,
build with `FUZZ_SANITIZER=none` (debug assertions stay on). `src/lib.rs` has two `cfg(windows)`
shims for that case: the SanitizerCoverage section markers that only the ASan runtime defines on
MSVC, and a panic-to-`ud2` wrapper, because `process::abort` (`__fastfail`) bypasses libFuzzer's
handler and no crash file would be written. On a busy machine `-timeout=2` can fire on host
stalls; re-time the input (`<target>.exe -runs=20 <file>`) before treating it as a finding.

## A finding

1. Minimise it: `cargo +nightly fuzz tmin --fuzz-dir fuzz -a <target> fuzz/artifacts/<target>/crash-…`.
2. Fix the bug in `amg-io` and add a unit test in the reader's own tests if the case is
   interesting on its own.
3. Commit the minimised input as `testdata/io/fuzz/<target>/<short-name>` (no format
   extension, so the git-LFS rules don't apply) and note it in that directory's README.

## Seeds

`fuzz/seeds/` is written by `python fuzz/make_seeds.py` (deterministic, 41 files, 14 kB).
