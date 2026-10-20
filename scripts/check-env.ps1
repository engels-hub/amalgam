# Checks the Amalgam dev toolchain against the pins in docs/specs/S00-foundation.md §1.
# Prints every tool version. A missing tool is an error (exit 1); a version that differs
# from its pin is a warning only. Runs in Windows PowerShell 5.1 and PowerShell 7.
#
# Usage: powershell -ExecutionPolicy Bypass -File scripts\check-env.ps1

$ErrorActionPreference = 'Continue'
$script:errors = 0
$script:warnings = 0

function Write-Row([string]$status, [string]$tool, [string]$found, [string]$want) {
    $color = switch ($status) { 'ok' { 'Green' } 'warn' { 'Yellow' } default { 'Red' } }
    Write-Host ('[{0,-4}] {1,-14} {2,-28} (pin: {3})' -f $status, $tool, $found, $want) -ForegroundColor $color
    if ($status -eq 'warn') { $script:warnings++ }
    if ($status -eq 'FAIL') { $script:errors++ }
}

# Runs a command and returns the first regex capture from its output, or $null.
function Get-Version([string]$exe, [string[]]$cmdArgs, [string]$pattern) {
    $cmd = Get-Command $exe -ErrorAction SilentlyContinue
    if (-not $cmd) { return $null }
    try { $out = (& $cmd.Source @cmdArgs 2>&1 | Out-String) } catch { return $null }
    if ($out -match $pattern) { return $Matches[1] }
    return $null
}

# Compares a found version against a pin. $mode: 'exact', 'prefix' (e.g. 4.3 matches 4.3.4)
# or 'min' (found >= pin).
function Test-Pin([string]$tool, [string]$found, [string]$pin, [string]$mode) {
    if (-not $found) { Write-Row 'FAIL' $tool 'not found' $pin; return }
    $ok = switch ($mode) {
        'exact'  { $found -eq $pin }
        'prefix' { $found -eq $pin -or $found.StartsWith("$pin.") }
        'min'    { [version]($found -replace '^(\d+\.\d+(\.\d+)?).*', '$1') -ge [version]$pin }
    }
    $label = if ($mode -eq 'min') { ">= $pin" } else { $pin }
    if ($ok) { Write-Row 'ok' $tool $found $label } else { Write-Row 'warn' $tool $found $label }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$toolchainFile = Join-Path $repoRoot 'rust-toolchain.toml'
$rustPin = '1.97.1'
if (Test-Path $toolchainFile) {
    $m = Select-String -Path $toolchainFile -Pattern 'channel\s*=\s*"([^"]+)"' | Select-Object -First 1
    if ($m) { $rustPin = $m.Matches[0].Groups[1].Value }
}

Write-Host 'Amalgam environment check' -ForegroundColor Cyan
Write-Host ''

# Rust: run outside the repo too, so the pin check also works before rust-toolchain.toml exists.
Test-Pin 'rustc'   (Get-Version 'rustc' @('--version') 'rustc (\S+)') $rustPin 'exact'
Test-Pin 'cargo'   (Get-Version 'cargo' @('--version') 'cargo (\S+)') $rustPin 'exact'
Test-Pin 'rustup'  (Get-Version 'rustup' @('--version') 'rustup (\S+)') '1.28' 'min'
Test-Pin 'clippy'  (Get-Version 'cargo' @('clippy', '--version') 'clippy (\S+)') '0.1.97' 'prefix'
Test-Pin 'rustfmt' (Get-Version 'rustfmt' @('--version') 'rustfmt (\S+)') '1.0' 'min'
Test-Pin 'nextest' (Get-Version 'cargo' @('nextest', '--version') 'cargo-nextest (\S+)') '0.9' 'prefix'
Test-Pin 'cargo-deny' (Get-Version 'cargo' @('deny', '--version') 'cargo-deny (\S+)') '0.18' 'min'

# MSVC: cl.exe is usually not on PATH outside a Developer shell, so locate it with vswhere.
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$vsVersion = $null; $msvcVersion = $null; $sdkFound = $false
if (Test-Path $vswhere) {
    $inst = & $vswhere -products * -version '[17.0,18.0)' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -latest -format json | ConvertFrom-Json
    if ($inst) {
        $vsVersion = $inst[0].catalog.productDisplayVersion
        $msvcFile = Join-Path $inst[0].installationPath 'VC\Auxiliary\Build\Microsoft.VCToolsVersion.default.txt'
        if (Test-Path $msvcFile) { $msvcVersion = (Get-Content $msvcFile -TotalCount 1).Trim() }
    }
}
Test-Pin 'VS 2022'  $vsVersion '17.14' 'prefix'
Test-Pin 'MSVC'     $msvcVersion '14.44' 'prefix'
$sdkRoot = Join-Path ${env:ProgramFiles(x86)} 'Windows Kits\10\Include\10.0.26100.0'
if (Test-Path $sdkRoot) { Write-Row 'ok' 'Win SDK' '10.0.26100.0' '10.0.26100' }
else { Write-Row 'FAIL' 'Win SDK' 'not found' '10.0.26100' }

Test-Pin 'cmake'   (Get-Version 'cmake' @('--version') 'cmake version (\S+)') '4.3' 'prefix'
Test-Pin 'ninja'   (Get-Version 'ninja' @('--version') '^(\S+)') '1.13.2' 'exact'

# vcpkg: tool version plus the registry tag/commit of VCPKG_ROOT.
$vcpkgPin = '2026.07.29'
$vcpkgBaseline = '9e593bb18ea69cc5095e012465dcd675a822ed0d'
if ($env:VCPKG_ROOT -and (Test-Path (Join-Path $env:VCPKG_ROOT 'vcpkg.exe'))) {
    $vcpkgExe = Join-Path $env:VCPKG_ROOT 'vcpkg.exe'
    $tool = (& $vcpkgExe version 2>&1 | Out-String)
    $toolVer = if ($tool -match 'version (\S+)') { $Matches[1] } else { '?' }
    $head = (& git -C $env:VCPKG_ROOT rev-parse HEAD 2>$null)
    if ($head -eq $vcpkgBaseline) { Write-Row 'ok' 'vcpkg' "$vcpkgPin (tool $toolVer)" $vcpkgPin }
    else { Write-Row 'warn' 'vcpkg' "HEAD $head" "$vcpkgPin = $vcpkgBaseline" }
    if (-not $env:VCPKG_DEFAULT_BINARY_CACHE) { Write-Row 'warn' 'vcpkg cache' 'VCPKG_DEFAULT_BINARY_CACHE unset' 'set' }
} else {
    Write-Row 'FAIL' 'vcpkg' 'VCPKG_ROOT unset or no vcpkg.exe' $vcpkgPin
}

Test-Pin 'git'     (Get-Version 'git' @('--version') 'git version (\d+\.\d+\.\d+)') '2.50' 'min'
Test-Pin 'git-lfs' (Get-Version 'git-lfs' @('--version') 'git-lfs/(\S+)') '3.0' 'min'
$longpaths = (& git config --global core.longpaths 2>$null)
if ($longpaths -eq 'true') { Write-Row 'ok' 'core.longpaths' 'true' 'true' }
else { Write-Row 'warn' 'core.longpaths' "$longpaths" 'true' }
$lp = (Get-ItemProperty 'HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem' -Name LongPathsEnabled -ErrorAction SilentlyContinue).LongPathsEnabled
if ($lp -eq 1) { Write-Row 'ok' 'LongPaths reg' '1' '1' } else { Write-Row 'warn' 'LongPaths reg' "$lp" '1' }

# Python is optional until S18 (pyo3), so a missing or old one is only a warning.
$py = Get-Version 'python' @('--version') 'Python (\S+)'
if ($py) { Test-Pin 'python' $py '3.12' 'min' } else { Write-Row 'warn' 'python' 'not found (optional)' '>= 3.12' }

Write-Host ''
Write-Host ("{0} error(s), {1} warning(s)" -f $script:errors, $script:warnings)
if ($script:errors -gt 0) { exit 1 } else { exit 0 }
