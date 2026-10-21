<#
Downloads the fetch-only models of the MESH-28 corpus into testdata/mesh/fetch/ (git-ignored).

Every entry of fetch.tsv has a pinned URL and sha256. The download is verified before it is
unpacked, the unpacked file is verified again, and nothing that was downloaded is executed.
Re-running skips files that are already present and correct.

  ./fetch.ps1                 fetch everything
  ./fetch.ps1 -Id a,b         fetch only the named corpus ids
  ./fetch.ps1 -CurlOpts '--ssl-no-revoke'   extra curl.exe flags (Windows/schannel behind some proxies)
#>
param(
  [string[]]$Id = @(),
  [string]$CurlOpts = ''
)
$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$failed = $false

function Get-Sha([string]$path) { (Get-FileHash -Algorithm SHA256 -LiteralPath $path).Hash.ToLowerInvariant() }

foreach ($line in Get-Content -LiteralPath (Join-Path $root 'fetch.tsv')) {
  if ($line -eq '' -or $line.StartsWith('#')) { continue }
  $f = $line -split "`t"
  $cid, $file, $url, $fetchSha, $archive, $member, $sha = $f
  if ($Id.Count -gt 0 -and $Id -notcontains $cid) { continue }
  $dest = Join-Path $root $file
  if ((Test-Path -LiteralPath $dest) -and (Get-Sha $dest) -eq $sha) { Write-Host "ok       $cid"; continue }

  $tmp = Join-Path ([IO.Path]::GetTempPath()) ([IO.Path]::GetRandomFileName())
  New-Item -ItemType Directory -Path $tmp | Out-Null
  try {
    Write-Host "fetching $cid"
    $dl = Join-Path $tmp 'download'
    $curlArgs = @('--fail', '--location', '--silent', '--show-error') + @($CurlOpts -split ' ' | Where-Object { $_ }) + @('-o', $dl, $url)
    & curl.exe @curlArgs
    if ($LASTEXITCODE -ne 0) { Write-Host -ForegroundColor Red "FAILED   ${cid}: download error"; $failed = $true; continue }
    if ((Get-Sha $dl) -ne $fetchSha) { Write-Host -ForegroundColor Red "FAILED   ${cid}: download sha256 mismatch (source changed?)"; $failed = $true; continue }
    $out = Join-Path $tmp 'out'
    switch ($archive) {
      'none' { Copy-Item -LiteralPath $dl -Destination $out }
      'gz' {
        $in = [IO.File]::OpenRead($dl)
        $gz = New-Object IO.Compression.GZipStream($in, [IO.Compression.CompressionMode]::Decompress)
        $o = [IO.File]::Create($out)
        try { $gz.CopyTo($o) } finally { $o.Dispose(); $gz.Dispose(); $in.Dispose() }
      }
      'tar.gz' {
        $x = Join-Path $tmp 'x'
        New-Item -ItemType Directory -Path $x | Out-Null
        & tar.exe -xzf $dl -C $x $member
        if ($LASTEXITCODE -ne 0) { Write-Host -ForegroundColor Red "FAILED   ${cid}: tar error"; $failed = $true; continue }
        Copy-Item -LiteralPath (Join-Path $x $member) -Destination $out
      }
      default { Write-Host -ForegroundColor Red "FAILED   ${cid}: unknown archive kind $archive"; $failed = $true; continue }
    }
    if ((Get-Sha $out) -ne $sha) { Write-Host -ForegroundColor Red "FAILED   ${cid}: extracted sha256 mismatch"; $failed = $true; continue }
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $dest) | Out-Null
    Move-Item -LiteralPath $out -Destination $dest -Force
    Write-Host "done     $cid"
  } finally {
    Remove-Item -Recurse -Force -LiteralPath $tmp -ErrorAction SilentlyContinue
  }
}
if ($failed) { exit 1 }
