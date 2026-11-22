#!/usr/bin/env bash
# Runs one IO-16 fuzz target with the nightly CI settings (note import-export-formats.md §9.3).
#
#   fuzz/run.sh <target> [seconds=300] [extra libFuzzer flags...]
#
# Corpus: fuzz/corpus/<target>/ (working corpus, git-ignored; CI caches it between nights),
# then the committed seeds in fuzz/seeds/<target>/ and the existing fixture directories, read
# by path. Findings land in fuzz/artifacts/<target>/; minimise them with
# `cargo +nightly fuzz tmin <target> <file>` and commit the result to
# testdata/io/fuzz/<target>/ together with the fix (the stable replay test runs it).
#
# Environment: FUZZ_TOOLCHAIN (default `nightly`), FUZZ_SANITIZER (default `address`; `none`
# is faster, amg-io has no unsafe code), FUZZ_TARGET_DIR (cargo target dir; default
# fuzz/target), FUZZ_BUILD_ONLY=1 to build without running.
#
# One in-process fuzzer (no -jobs/-workers): the memory of `-jobs=1` without its forked worker
# and fuzz-0.log, so the exit code and final stats come straight back.
set -euo pipefail

target=${1:?usage: fuzz/run.sh <target> [seconds] [libFuzzer flags...]}
seconds=${2:-300}
shift $(( $# >= 2 ? 2 : 1 ))

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/.." && pwd)
cd "$repo"

toolchain=${FUZZ_TOOLCHAIN:-nightly}
sanitizer=${FUZZ_SANITIZER:-address}
target_dir=${FUZZ_TARGET_DIR:-$here/target}

seeds=("fuzz/seeds/$target")
max_len=()
case "$target" in
  stl | obj | ply) seeds+=(testdata/io/heads) ;;
  threemf_package)
    seeds+=(testdata/io/slicer)
    # 3MF Consortium suites, when fetched (testdata/io/fetch-3mf-suites.ps1).
    [[ -d testdata/io/fetch ]] && seeds+=(testdata/io/fetch)
    max_len=(-max_len=262144)
    ;;
  mtl | threemf_model | paint_decode | roundtrip_scene) ;;
  *) echo "unknown target: $target" >&2; exit 2 ;;
esac

mkdir -p "fuzz/corpus/$target" "fuzz/artifacts/$target"

build=(cargo "+$toolchain" fuzz build --fuzz-dir fuzz --debug-assertions
  --sanitizer "$sanitizer" --target-dir "$target_dir" "$target")
"${build[@]}"
if [[ "${FUZZ_BUILD_ONLY:-0}" == 1 ]]; then
  exit 0
fi

exec cargo "+$toolchain" fuzz run --fuzz-dir fuzz --debug-assertions \
  --sanitizer "$sanitizer" --target-dir "$target_dir" "$target" \
  "fuzz/corpus/$target" "${seeds[@]}" -- \
  -rss_limit_mb=2048 -timeout=2 \
  -max_total_time="$seconds" -print_final_stats=1 \
  -artifact_prefix="fuzz/artifacts/$target/" "${max_len[@]}" "$@"
