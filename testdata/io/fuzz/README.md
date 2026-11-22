# Minimised fuzz findings (IO-16)

One directory per fuzz target (`fuzz/fuzz_targets/<target>.rs`); each file is a minimised
libFuzzer input that once crashed, overflowed, timed out or broke an oracle, committed together
with its fix. `crates/amg-io/tests/fuzz_replay.rs` replays every file under stable
`cargo nextest`, so a fixed bug stays fixed. Files have no format extension (the repository's
git-LFS rules would otherwise apply) and are byte-exact (`.gitattributes`).

| File | Bug | Fix |
|---|---|---|
| `stl/endfacet-substring` | The ASCII chunk cutter cut after any `endfacet` substring (`xendfacet`, or inside a `solid` name), so a chunked read parsed a different file than the whole-file read. | `stl::ascii::is_cut`: only whole `endfacet` tokens outside `solid`/`endsolid` name lines are cuts. |
| `threemf_package/percent-utf8` | `percent_decode` sliced the `str` inside a multi-byte character after `%` (panic: "byte index is not a char boundary"); it also took `%+1` as an escape. | Hex digits are read from the bytes. |
| `ply/nan-vertex` | Faces on NaN/inf vertices were dropped, but the vertices stayed in `positions` (also OBJ single-object files). | PLY drops non-finite vertices and remaps; OBJ keeps only used vertices when any is non-finite. |
| `roundtrip_scene/control-char-metadata` | The 3MF writer skipped empty metadata before dropping XML-invalid characters, so a value of only control characters was written as `<metadata …></metadata>` and read back as `""`. | Emptiness is checked after escaping. |
| `obj/inf-vertex` | Same leak as `ply/nan-vertex` in single-object OBJ files (found by reading the code after the PLY finding; written by hand). | See `ply/nan-vertex`. |
| `threemf_model/late-xmlns` | The fast vertex/triangle scanner reported a bad value (`y="h"`) before it had read the whole start tag; a later `xmlns` makes the element foreign, which the reference parser skips. So the result depended on the chunking (raw fuzzer input, 1.4 kB). | `threemf::scan`: value errors wait until the element is known to be plain. |
| `ply/inch-overflow` | A finite double near `f64::MAX`, imported with an inch (or cm/m/ft) unit, became infinite in `Scene::rescale`, after the reader's NaN/inf filter. Any format with doubles (PLY, OBJ, 3MF `unit="inch"`). | `import`: after a non-mm rescale, non-finite coordinates give `IoError::InvalidScene`. |
| `ply/empty-element-count` | An ASCII element without properties and a count of 90 M (a zero-byte record, so the allocation rule bounds nothing) was walked record by record: over 15 s per read (raw fuzzer input, 239 bytes). | Property-less elements are skipped at once. |
