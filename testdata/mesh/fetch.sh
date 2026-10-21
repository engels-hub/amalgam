#!/usr/bin/env bash
# Downloads the fetch-only models of the MESH-28 corpus into testdata/mesh/fetch/ (git-ignored).
#
# Every entry of fetch.tsv has a pinned URL and sha256. The download is verified before it is
# unpacked, the unpacked file is verified again, and nothing that was downloaded is executed.
# Re-running skips files that are already present and correct.
#
#   ./fetch.sh            fetch everything
#   ./fetch.sh <id>...    fetch only the named corpus ids
#
# Environment: CURL_OPTS (extra curl flags, for example "--ssl-no-revoke" on Windows/schannel).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
list="$root/fetch.tsv"
curl_opts=${CURL_OPTS:-}

sha256() {
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | cut -d' ' -f1
  else shasum -a 256 "$1" | cut -d' ' -f1; fi
}

want() {
  [ "$#" -eq 1 ] && return 0
  local id="$1"; shift
  for w in "$@"; do [ "$w" = "$id" ] && return 0; done
  return 1
}

fail=0
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
while IFS=$'\t' read -r id file url fetch_sha archive member sha; do
  case "$id" in ''|'#'*) continue ;; esac
  want "$id" "$@" || continue
  dest="$root/$file"
  if [ -f "$dest" ] && [ "$(sha256 "$dest")" = "$sha" ]; then
    echo "ok       $id"
    continue
  fi
  rm -rf "${tmp:?}"/*
  echo "fetching $id"
  # shellcheck disable=SC2086
  if ! curl --fail --location --silent --show-error $curl_opts -o "$tmp/download" "$url"; then
    echo "FAILED   $id: download error" >&2; fail=1; continue
  fi
  if [ "$(sha256 "$tmp/download")" != "$fetch_sha" ]; then
    echo "FAILED   $id: download sha256 mismatch (source changed?)" >&2; fail=1; continue
  fi
  mkdir -p "$(dirname "$dest")"
  case "$archive" in
    none)   cp "$tmp/download" "$tmp/out" ;;
    gz)     gzip -dc "$tmp/download" > "$tmp/out" ;;
    tar.gz) tar -xzOf "$tmp/download" "$member" > "$tmp/out" ;;
    *) echo "FAILED   $id: unknown archive kind $archive" >&2; fail=1; continue ;;
  esac
  if [ "$(sha256 "$tmp/out")" != "$sha" ]; then
    echo "FAILED   $id: extracted sha256 mismatch" >&2; fail=1; continue
  fi
  mv "$tmp/out" "$dest"
  echo "done     $id"
done < "$list"
exit "$fail"
