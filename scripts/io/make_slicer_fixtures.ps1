<#
Builds the slicer-produced 3MF fixtures of S12 IO-03 Done-when 2 into testdata/io/slicer/.

Inputs are CC0-1.0 models of the MESH-28 corpus (thingi10k 97160, 97162, 217025). Each slicer
found on PATH (or given by parameter) exports them through its CLI:

  - OrcaSlicer 2.4.x:      orca-slicer --export-3mf <out> <in...>
  - Bambu Studio 2.8.x:    bambu-studio --export-3mf <out> <in...>
  - PrusaSlicer 2.9.x:     prusa-slicer-console --export-3mf -o <out> <in...>

Per slicer it writes: one object (`<slicer>-single.3mf`), three objects in one project
(`<slicer>-three.3mf`, Production multi-part for Orca/Bambu) and, with -Arrange, an arranged
copy. The CLIs cannot paint, so the painted fixtures (`<slicer>-painted.3mf`: whole-triangle
filament states on the 97160 model, plus one split triangle) are made once by hand in the GUI
from `<slicer>-single.3mf` and saved next to them; the script lists which are missing.

Afterwards run the reader over every fixture and paste the printed TOML into
testdata/io/slicer/fixtures.toml (objects, names, build transforms, painted-face filament
histogram), after checking the values against the slicer's own object list:

  cargo test -p amg-io --test threemf_slicer_fixtures -- --nocapture print_manifest_entries

All outputs are our own exports of CC0 inputs and are committed as CC0-1.0 (note §11).
#>
param(
  [string]$Orca = 'orca-slicer',
  [string]$Bambu = 'bambu-studio',
  [string]$Prusa = 'prusa-slicer-console',
  [switch]$Arrange
)
$ErrorActionPreference = 'Stop'
$repo = Resolve-Path (Join-Path $PSScriptRoot '../..')
$out = Join-Path $repo 'testdata/io/slicer'
$inputs = @('97160', '97162', '217025') | ForEach-Object { Join-Path $repo "testdata/mesh/real/thingi10k/$_.stl" }
foreach ($i in $inputs) {
  if (-not (Test-Path -LiteralPath $i)) { throw "missing input $i (git lfs pull)" }
  if ((Get-Content -LiteralPath $i -TotalCount 1) -like 'version https://git-lfs*') { throw "$i is an LFS pointer (git lfs pull)" }
}
New-Item -ItemType Directory -Force -Path $out | Out-Null

function Export([string]$name, [string]$exe, [string[]]$argsBefore) {
  if (-not (Get-Command $exe -ErrorAction SilentlyContinue)) { Write-Host "skip     $name ($exe not found)"; return }
  $ver = (& $exe --help 2>&1 | Select-Object -First 1)
  Write-Host "using    $name : $ver"
  $single = Join-Path $out "$name-single.3mf"
  $three = Join-Path $out "$name-three.3mf"
  $extra = @(); if ($Arrange) { $extra = @('--arrange', '1') }
  & $exe @argsBefore @extra $single $inputs[0]
  if ($LASTEXITCODE -ne 0) { throw "$name single export failed" }
  & $exe @argsBefore @extra $three @inputs
  if ($LASTEXITCODE -ne 0) { throw "$name three-object export failed" }
  $painted = Join-Path $out "$name-painted.3mf"
  if (-not (Test-Path -LiteralPath $painted)) { Write-Host "TODO     ${painted}: paint in the GUI (see header)" }
}

Export 'orca' $Orca @('--export-3mf')
Export 'bambu' $Bambu @('--export-3mf')
Export 'prusa' $Prusa @('--export-3mf', '-o')
Write-Host "done; now update testdata/io/slicer/fixtures.toml"
