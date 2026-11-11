<#
Checks out the 3MF Consortium conformance suites used by IO-03 (S12) at a pinned commit into
testdata/io/fetch/3mf-test-suites/ (git-ignored), or into -Dest.

Source: https://github.com/3MFConsortium/test_suites (BSD-2-Clause). Only suites 2, 3, 5 and 6
(core, production, materials; positive and negative cases, about 600 MB) are checked out, with
a blob-less partial clone and a sparse checkout, so the 1.6 GB repository is never downloaded
whole. The commit is pinned; git verifies every object against it. Afterwards the file list is
compared with 3mf-suites.sha256 (path and sha256 of every case file).

  ./fetch-3mf-suites.ps1                       fetch into testdata/io/fetch/3mf-test-suites
  ./fetch-3mf-suites.ps1 -Dest D:\data\3mf     fetch elsewhere (then set AMG_3MF_SUITES)
  ./fetch-3mf-suites.ps1 -Verify               only verify an existing checkout
#>
param(
  [string]$Dest = '',
  [switch]$Verify
)
$ErrorActionPreference = 'Stop'
$commit = 'f483d3beee069e7adbac125d1f7baa36b5b3e5bf'
$url = 'https://github.com/3MFConsortium/test_suites'
$dirs = @(
  '/suite2_core_prod_matl/positive_test_cases/', '/suite2_core_prod_matl/negative_test_cases/',
  '/suite3_core/positive_test_cases/', '/suite3_core/negative_test_cases/',
  '/suite5_core_prod/positive_test_cases/', '/suite5_core_prod/negative_test_cases/',
  '/suite5_core_prod/positive_test_cases_prod_alt/', '/suite5_core_prod/negative_test_cases_prod_alt/',
  '/suite6_core_matl/positive_test_cases/', '/suite6_core_matl/negative_test_cases/',
  '/LICENSE', '/README.md'
)
if ($Dest -eq '') { $Dest = Join-Path $PSScriptRoot 'fetch/3mf-test-suites' }

if (-not $Verify) {
  if (-not (Test-Path -LiteralPath (Join-Path $Dest '.git'))) {
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Dest) | Out-Null
    & git clone --filter=blob:none --no-checkout --sparse $url $Dest
    if ($LASTEXITCODE -ne 0) { throw 'git clone failed' }
  }
  & git -C $Dest sparse-checkout set --no-cone @dirs
  if ($LASTEXITCODE -ne 0) { throw 'git sparse-checkout failed' }
  & git -C $Dest -c advice.detachedHead=false checkout $commit
  if ($LASTEXITCODE -ne 0) { throw "git checkout $commit failed" }
}

# Verify every case file against the manifest.
$manifest = Join-Path $PSScriptRoot '3mf-suites.sha256'
$bad = 0
$n = 0
foreach ($line in Get-Content -LiteralPath $manifest) {
  if ($line -eq '' -or $line.StartsWith('#')) { continue }
  $sha, $rel = $line -split '  ', 2
  $p = Join-Path $Dest $rel
  $n++
  if (-not (Test-Path -LiteralPath $p)) { Write-Host -ForegroundColor Red "missing  $rel"; $bad++; continue }
  $h = (Get-FileHash -Algorithm SHA256 -LiteralPath $p).Hash.ToLowerInvariant()
  if ($h -ne $sha) { Write-Host -ForegroundColor Red "sha256   $rel"; $bad++ }
}
if ($bad -gt 0) { throw "$bad of $n files are missing or differ" }
Write-Host "ok: $n case files verified at $Dest"
