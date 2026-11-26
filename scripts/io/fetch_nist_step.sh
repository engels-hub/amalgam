#!/usr/bin/env sh
# Fetches the NIST MBE PMI STEP models (S12 IO-06, Done-when 1) into
# $AMG_TESTDATA_DIR/io/step/nist-mbe-pmi/. They are not committed (fetch-only, like the owner's
# Fusion exports), so the IO-06 tests skip them when the folder is absent.
#
# Source: NIST MBE PMI Validation and Conformance Testing project, "NIST PMI STEP Files"
# (https://www.nist.gov/document/nist-pmi-step-files). NIST states the test cases, CAD models
# and STEP files "can be used without any restrictions" (US-government work; acknowledgement
# requested). The archive is pinned by SHA-256; a changed archive fails instead of silently
# changing the corpus.
#
# Usage: AMG_TESTDATA_DIR=D:/coding/amalgam-testdata sh scripts/io/fetch_nist_step.sh
set -eu

URL="https://www.nist.gov/system/files/documents/noindex/2024/06/19/NIST-PMI-STEP-Files.zip"
SHA256="8fa78429e6d8d9b0d7681d223b6aa9ec98c3772185c55b1a0e3679b21c181911"

: "${AMG_TESTDATA_DIR:?set AMG_TESTDATA_DIR to the external test data folder}"
DEST="$AMG_TESTDATA_DIR/io/step/nist-mbe-pmi"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "downloading $URL"
curl -fsSL --retry 3 -o "$WORK/nist.zip" "$URL"
GOT="$(sha256sum "$WORK/nist.zip" | cut -d' ' -f1)"
if [ "$GOT" != "$SHA256" ]; then
    echo "SHA-256 mismatch: got $GOT, want $SHA256" >&2
    exit 1
fi
mkdir -p "$WORK/x" "$DEST"
unzip -q "$WORK/nist.zip" -d "$WORK/x"
# The AP242 files (with semantic PMI) and the AP203 geometry-only files; the AP203 files with
# graphical PMI and the PDFs are not needed.
cp "$WORK/x/NIST-PMI-STEP-Files/"*.stp "$DEST/"
cp "$WORK/x/NIST-PMI-STEP-Files/AP203 geometry only/"*.stp "$DEST/"
echo "$(ls "$DEST" | wc -l) STEP files in $DEST"
