# Developer setup (Windows x64)

This is how to get from a clean Windows 11 machine to a building Amalgam workspace. The pins
come from [S00 §1](specs/S00-foundation.md). The table below records what was actually
installed on the owner's machine when FND-01 ran (2026-10-20).

| Tool | Pin | Installed (owner machine, 2026-10-20) |
|---|---|---|
| Rust | `1.97.1` via `rust-toolchain.toml` | 1.97.1 (rustup 1.29.1, clippy 0.1.97, rustfmt 1.9.0) |
| VS Build Tools 2022 | 17.14, MSVC v143 / 14.44, Win 11 SDK 10.0.26100 | 17.14.42, MSVC 14.44.35207, SDK 10.0.26100.0 |
| CMake | 4.3.x | 4.3.4 |
| Ninja | 1.13.2 | 1.13.2 |
| vcpkg | tag `2026.07.29` | tag `2026.07.29`, tool 2026-07-27 |
| Git + LFS | Git ≥ 2.50, git-lfs | Git 2.44.0 (upgrade pending, see §8), git-lfs 3.4.1 |
| cargo-nextest | latest 0.9.x | 0.9.148 |
| cargo-deny | latest 0.18.x+ | 0.20.2 |
| Python | 3.12+ (optional until S18) | 3.12.2 |

**vcpkg baseline:** tag `2026.07.29` = commit `9e593bb18ea69cc5095e012465dcd675a822ed0d`.
This SHA is the `builtin-baseline` that FND-05 writes into `vcpkg.json`.

## 1. Prerequisites

- Windows 11 x64.
- About 40 GB free disk on the drive that holds vcpkg. The OCCT build tree is large.
- Admin rights for the Visual Studio installer and the long-paths registry setting.

## 2. One-shot install

From PowerShell:

```powershell
winget install --source winget Rustlang.Rustup Kitware.CMake Ninja-build.Ninja Git.Git
winget install --source winget Microsoft.VisualStudio.2022.BuildTools --override "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 --add Microsoft.VisualStudio.Component.Windows11SDK.26100 --add Microsoft.VisualStudio.Component.VC.CMake.Project"
```

`--source winget` skips the Microsoft Store source. On some networks that source fails with
`0x8a15005e : The server certificate did not match any of the expected values`.

Open a **new** terminal afterwards so the updated `PATH` is picked up.

## 3. Long paths

```powershell
git config --global core.longpaths true
# admin PowerShell:
New-ItemProperty -Path 'HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem' -Name LongPathsEnabled -Value 1 -PropertyType DWORD -Force
```

Keep vcpkg in a short root such as `C:\dev\vcpkg` or `D:\dev\vcpkg`. If a port still fails
with MAX_PATH errors, pass `--x-buildtrees-root=C:\vb` (or another short path) to vcpkg.

## 4. vcpkg

Pick a drive with enough space. The owner machine uses `D:\dev` because C: is small. Any
short path works.

```powershell
git clone https://github.com/microsoft/vcpkg.git D:\dev\vcpkg
git -C D:\dev\vcpkg checkout 2026.07.29
D:\dev\vcpkg\bootstrap-vcpkg.bat -disableMetrics
setx VCPKG_ROOT D:\dev\vcpkg
setx VCPKG_DEFAULT_BINARY_CACHE D:\dev\vcpkg-cache
mkdir D:\dev\vcpkg-cache
```

A full clone is used locally so the baseline can be bumped with a `git checkout`. CI uses a
shallow clone of the tag (FND-05).

Smoke test (classic mode, outside the repo):

```powershell
cd D:\dev
vcpkg install zlib:x64-windows --classic
```

If it succeeds, the MSVC toolchain and CMake both work.

## 5. Rust

`rustup` reads `rust-toolchain.toml` in the repo root, so the first `cargo` command inside the
repo installs the pinned toolchain and components. To set it as the global default, too:

```powershell
rustup default 1.97.1
rustup component add clippy rustfmt
cargo install --locked cargo-nextest cargo-deny
```

## 6. Optional speed-ups

Windows Defender real-time scanning slows Rust builds a lot. From an admin PowerShell:

```powershell
Add-MpPreference -ExclusionPath "D:\coding\amalgam\target", "$env:USERPROFILE\.cargo", "$env:VCPKG_ROOT", "$env:VCPKG_DEFAULT_BINARY_CACHE"
```

Adjust the first path to wherever the repo is checked out.

## 7. Verify

```powershell
powershell -ExecutionPolicy Bypass -File scripts\check-env.ps1
```

It prints the version of every pinned tool. A version that differs from its pin is a
**warning**. A missing tool is an **error**, and the script then exits 1. When it exits 0, run:

```powershell
cargo build --workspace
cargo nextest run --workspace
```

(These two need the workspace from FND-02.)

## 8. Troubleshooting

- **`link.exe` not found / `cl.exe` errors:** the VC tools component is missing. Re-run the
  Build Tools install from §2, or build from a "Developer PowerShell for VS 2022". rustc finds
  MSVC through vswhere, so normally no Developer shell is needed.
- **MAX_PATH errors in vcpkg:** see §3 (`core.longpaths`, `LongPathsEnabled`, short
  buildtrees root).
- **vcpkg hash mismatch after a baseline bump:** clear the folder in
  `VCPKG_DEFAULT_BINARY_CACHE` and rebuild.
- **GPU tests skipped:** no wgpu adapter was found. Locally, check the GPU driver. In CI,
  `AMG_WGPU_BACKEND=warp` uses the WARP software adapter.
- **`git push` / `git clone` fails with `SSL certificate problem: unable to get local issuer
  certificate`:** git's OpenSSL backend doesn't trust the certificate chain the network
  presents. Use the Windows certificate store instead:
  `git config --global http.sslBackend schannel`.
- **Git upgrade aborts:** the Git for Windows installer refuses to run while any `bash.exe`
  from Git is running (Git Bash windows, terminal tabs, editor shells). Close them and run
  `winget upgrade --id Git.Git --source winget`.
- **New tools not found right after install:** open a new terminal. Running shells keep their
  old `PATH`.

## 9. Bumping pins

1. Pick the new version. For Rust, take the newest stable that is at least two weeks old.
2. Change it in **one PR** together with everything that depends on it:
   - Rust: `rust-toolchain.toml` and `rust-version` in the root `Cargo.toml`.
   - vcpkg: the tag *and* `builtin-baseline` in `vcpkg.json`, plus the clone tag in
     `.github/workflows/ci.yml`.
   - The pins in `scripts/check-env.ps1`, the table at the top of this file, and S00 §1.
3. Title it `FND-xx: bump <tool> to <version>`.
4. CI must be green, including a full vcpkg rebuild when the baseline changes.
