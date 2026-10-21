# S00 — Foundation, build & tooling

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P0 |
| **Crates** | workspace root, `amg-core`, `amg-jobs`, `amg-cli` (skeleton; subcommands filled by S18), `amg-testkit` (`amg-app` belongs to lane D) |
| **Lane** | E |
| **Depends on** | — (FND-09 is gated by ADR-004, see WP table) |
| **Parity** | — (enabler) |

## Summary
This spec covers the Cargo workspace, the toolchain, CI, and the foundation types every crate
uses: math, units, tolerances, ids, geometric primitives and errors. It also covers the job
system: a CPU pool with progress and cancellation, plus the shared GPU device. Nothing else
can start until FND-01..FND-04 are done, so this spec is kept small and its APIs stay stable.

## Scope
**In:**
- Windows x64 MSVC toolchain pinning and the `docs/dev-setup.md` outline.
- Workspace layout with every architecture crate as a stub.
- Lint, format and test config, plus a layering check.
- CI on GitHub Actions with vcpkg caching. The vcpkg build story: manifest mode, overlay
  triplet and binary cache.
- `amg-core`: math aliases, tolerance, ids, primitives with intersection tests, units and
  quantities, errors.
- `amg-jobs`: job system, progress, cancellation, child jobs, non-blocking handles, shared
  `Gpu`, diagnostics (logging, panic hook, crash report).
- `amg-testkit`: golden-geometry helpers, `testdata/` with git-LFS, budget benches.
- `amg-cli` skeleton: `--version`, `info`.

**Out:**
- Analytic curves such as circles, arcs and conics. They get a later FND WP once S02/S03
  refinement shows what they need (see open question 3).
- Any domain geometry. It lives in domain crates.
- The expression language with compound units. It belongs to S01 (DOC-04).
- CLI subcommands `recompute`, `export` and `study run` (S18, SCR-05).
- GPU kernels, reductions and solvers (S15, `amg-compute`).
- The renderer (S10).
- OCCT/Manifold build scripts (BREP-01 and MESH-05 own their `-sys` crates).

## Design

### 1. Toolchain (Windows x64 MSVC, owner machine has no Rust yet)

Pinned versions. FND-01 re-checks each pin on the day it runs and records the actual
versions in `docs/dev-setup.md`. Bumping a pin later is a one-line PR, `FND-xx: bump …`.

| Tool | Pin | Why / evidence |
|---|---|---|
| Rust | `rust-toolchain.toml` `channel = "1.97.1"` (newest stable verified at refinement: 1.97.0 on 2026-07-09, .1 on 2026-07-16). FND-01 may bump it to the newest stable that is at least 2 weeks old. | Pinning an exact version, not `stable`, keeps CI and local builds identical and is required for the determinism principle. Edition 2024 (stable since 1.85). |
| Target | `x86_64-pc-windows-msvc` | ADR-001, vision (Windows primary) |
| C++ toolchain | VS Build Tools 2022 **17.14** (final 2022 feature release), MSVC **v143 / 14.44**, Windows 11 SDK **10.0.26100**. winget component IDs: `Microsoft.VisualStudio.Workload.VCTools`, `Microsoft.VisualStudio.Component.VC.Tools.x86.x64`, `Microsoft.VisualStudio.Component.Windows11SDK.26100`, `Microsoft.VisualStudio.Component.VC.CMake.Project` | Needed by `cc`/`cxx` (BREP-01) and the vcpkg ports. Since June 2026 the CI image `windows-2025` ships VS 2026 (MSVC 14.5x, toolset v145). It is binary-compatible with v143, but local and CI compilers now differ. See open question 1 (owner). |
| CMake | **4.3.x** standalone, plus VS's bundled CMake as fallback. vcpkg uses its own pinned CMake. | CMake 4.x removed compatibility with `cmake_minimum_required` < 3.5. vcpkg handles this for its ports. Our own `-sys` crates must declare ≥ 3.21. |
| Ninja | **1.13.2** | Used by CMake-driven `-sys` builds. Joins the cargo/GNU-make jobserver since 1.13. |
| vcpkg | registry baseline = tag **`2026.07.29`** (2,858 ports). `builtin-baseline` = that tag's commit SHA, resolved in FND-01. vcpkg-tool comes from `bootstrap-vcpkg.bat` at the same tag. | Manifest mode (`vcpkg.json` at the workspace root, created empty-of-deps by FND-05 and first populated by BREP-01). Note: since vcpkg 2026.05.25 the `opencascade` port is **8.0.0**. ADR-002 says "7.8+", so BREP-01 either accepts 8.0 or adds an `overrides` entry pinning 7.9.x. **Owner decision 2026-10-20 (QUESTIONS #27):** FND-13 bumps the baseline to a tag shipping 8.0.1 before BREP-01; the tag above stays until FND-13 is Done. |
| vcpkg triplet | overlay triplet `triplets/x64-windows-amg.cmake`: `VCPKG_TARGET_ARCHITECTURE x64`, `VCPKG_CRT_LINKAGE dynamic`, `VCPKG_LIBRARY_LINKAGE static`, `VCPKG_BUILD_TYPE release` | rustc always links the release CRT (`/MD`), even in debug builds, so debug C++ libs (`/MDd`) would mismatch. Release-only also halves the OCCT build time. Static libs avoid copying DLLs next to every test binary. LGPL-2.1+exception (OCCT) permits static linking. BREP-01 may revisit. |
| Git + LFS | Git for Windows ≥ 2.50, `git lfs install` | `testdata/` (FND-06) |
| cargo tools | `cargo-nextest` (latest 0.9.x), `cargo-deny` (latest 0.18.x+), installed with `cargo install --locked` | AGENTS.md test command; ADR-012 licence check |
| Python | 3.12+ (optional until S18) | pyo3 (ADR-011) |

**`docs/dev-setup.md` outline (FND-01 deliverable):**
1. *Prerequisites*: Windows 11 x64, about 40 GB free disk (the vcpkg OCCT build tree is
   large), admin rights for the VS installer.
2. *One-shot install* (PowerShell, winget):
   `winget install Rustlang.Rustup Kitware.CMake Ninja-build.Ninja Git.Git` and
   `winget install Microsoft.VisualStudio.2022.BuildTools --override "--quiet --wait --add <the four IDs above>"`.
3. *Long paths*: `git config --global core.longpaths true`. Set `LongPathsEnabled=1`
   (registry command shown). Keep vcpkg in a short root (`C:\dev\vcpkg`) and use
   `--x-buildtrees-root=C:\vb` if MAX_PATH errors appear.
4. *vcpkg*: clone at the pinned tag, `bootstrap-vcpkg.bat -disableMetrics`, `setx VCPKG_ROOT`,
   and `setx VCPKG_DEFAULT_BINARY_CACHE C:\dev\vcpkg-cache`.
5. *Rust*: `rustup` reads `rust-toolchain.toml` automatically. Then
   `cargo install --locked cargo-nextest cargo-deny`.
6. *Optional speed-ups*: Defender exclusions for `target\`, `%USERPROFILE%\.cargo` and the
   vcpkg root.
7. *Verify*: `scripts/check-env.ps1`. It prints rustc/cargo/cl/cmake/ninja/vcpkg/git-lfs
   versions, compares them with the pins (warning only), and exits 0 or 1. Then run
   `cargo build --workspace` and `cargo nextest run --workspace`.
8. *Troubleshooting*: `link.exe` not found (Developer PowerShell or a missing VC component),
   MAX_PATH, vcpkg hash mismatch after a baseline bump (clear `VCPKG_DEFAULT_BINARY_CACHE`),
   GPU tests skipped (no adapter).
9. *Bumping pins*: the procedure, and that the toolchain, vcpkg baseline and CI must change
   in the same PR.

### 2. Workspace layout (FND-02)

```
amalgam/
├─ Cargo.toml              # virtual manifest: [workspace], [workspace.package], [workspace.dependencies], [workspace.lints], profiles
├─ rust-toolchain.toml  rustfmt.toml  clippy.toml  deny.toml  .gitattributes  .editorconfig
├─ .cargo/config.toml      # no target-cpu=native (bit-stability across machines); alias `xt = "nextest run --workspace"`
├─ .config/nextest.toml    # profiles `default`, `ci`; test-group `gpu` (max-threads = 1)
├─ .github/workflows/ci.yml
├─ vcpkg.json  vcpkg-configuration.json  triplets/x64-windows-amg.cmake   # added by FND-05
├─ scripts/check-env.ps1
├─ crates/
│   ├─ amg-core  amg-jobs  amg-compute  amg-testkit                     # foundation
│   ├─ amg-occt-sys  amg-nurbs  amg-brep  amg-sketch  amg-mesh  amg-implicit
│   ├─ amg-subd  amg-materials  amg-io  amg-fem  amg-topopt  amg-am       # domains
│   ├─ amg-doc  amg-features                                              # document
│   ├─ amg-render  amg-ui  amg-script                                     # interaction / scripting
│   └─ amg-app  amg-cli                                                   # binaries
├─ tests/golden/           # golden expectation files (FND-06), read via amg_testkit::golden_dir()
└─ testdata/               # git-LFS corpus (FND-06)
```

That is 23 crates: the 22 from the architecture table plus `amg-testkit`. Other `-sys` crates
(Manifold, fTetWild) are created by their owning WPs, not by FND-02. Every stub crate gets:
- `lib.rs` with a crate doc comment naming its spec and lane.
- `[lints] workspace = true`.
- Its **intra-workspace dependencies exactly as the architecture table allows**. The layering
  is then visible in `cargo metadata` and cycles cannot compile.

`amg-occt-sys` and `amg-script` start as pure-Rust stubs without `cxx`, a C++ build or pyo3.
BREP-01 and SCR-01 add those.

**Layering check:** `crates/amg-testkit/tests/layering.rs` runs `cargo metadata` and compares
every `amg-*` → `amg-*` edge against an allow-list table copied from architecture §Crates.
An edge that isn't on the list fails the test. Changing the allow-list needs an ADR, since
crate boundaries are architecture.

**`[workspace.package]`:** `edition = "2024"`, `rust-version = "1.97"`, `license = "LicenseRef-Amalgam-Personal"`,
`publish = false`.

**`[workspace.dependencies]` (versions = current major lines at refinement; FND-02 pins
exact minors):**
`glam = { version = "0.33", features = ["serde", "approx-05", "bytemuck"] }` (0.34; see 2026-10-20 log), `approx = "0.5"`,
`slotmap = "1"`, `uuid = { version = "1", features = ["v5", "v7", "serde"] }`,
`thiserror = "2"`, `anyhow = "1"`, `serde = { version = "1", features = ["derive"] }`,
`serde_json = "1"`, `rayon = "1.11"`, `crossbeam-channel = "0.5"`, `parking_lot = "0.12"`,
`tracing = "0.1"`, `tracing-subscriber = { version = "0.3", features = ["env-filter"] }`,
`tracing-appender = "0.2"`, `wgpu = "29"` (see FND-09 for egui alignment),
`bytemuck = { version = "1", features = ["derive"] }`, `pollster = "0.4"`,
`clap = { version = "4", features = ["derive"] }`, `toml = "0.9"`, `criterion = "0.7"`, `proptest = "1"`.
The glam feature `fast-math` is **forbidden**: it breaks determinism, and `deny.toml` carries
a comment saying so.

**`[workspace.lints]`:**
```toml
[workspace.lints.rust]
unsafe_op_in_unsafe_fn = "deny"
unsafe_code = "deny"                 # -sys crates and documented FFI/GPU modules use #![allow(unsafe_code)] / #[allow] with a SAFETY doc
missing_debug_implementations = "warn"
rust_2018_idioms = { level = "warn", priority = -1 }
unused_qualifications = "warn"

[workspace.lints.clippy]
all = { level = "warn", priority = -1 }
undocumented_unsafe_blocks = "deny"
dbg_macro = "warn"
todo = "warn"
unwrap_used = "warn"                 # allowed in tests via clippy.toml
expect_used = "warn"                 # ditto; use `?` or document the invariant with #[allow] + reason
float_cmp = "warn"                   # geometry compares through Tolerance
print_stdout = "warn"                # binaries allow at crate level
large_types_passed_by_value = "warn"
```
CI runs `cargo clippy --workspace --all-targets -- -D warnings`, so every `warn` is an error
in CI.

**`rust-toolchain.toml`:**
```toml
[toolchain]
channel = "1.97.1"
components = ["rustfmt", "clippy", "rust-src", "rust-analyzer"]
targets = ["x86_64-pc-windows-msvc"]
profile = "minimal"
```

**`rustfmt.toml`** (stable options only, no nightly needed): `edition = "2024"`,
`max_width = 100`, `use_field_init_shorthand = true`, `use_try_shorthand = true`,
`newline_style = "Unix"`. `.gitattributes` sets `* text=auto eol=lf`, and FND-06 adds the LFS
patterns.

**`clippy.toml`:** `allow-unwrap-in-tests = true`, `allow-expect-in-tests = true`,
`allow-dbg-in-tests = true`, `allow-print-in-tests = true`,
`disallowed-methods = [{ path = "pollster::block_on", reason = "use amg_jobs; never block the UI thread" }]`
(`amg-jobs::gpu` allows it locally).

**Profiles:**
- `[profile.dev] opt-level = 1` with `[profile.dev.package."*"] opt-level = 2`. Geometry and
  OCCT-adjacent code is unusably slow at `-O0`.
- `[profile.release] lto = "thin"`, `debug = "line-tables-only"` (symbolised crash reports),
  `panic = "unwind"` (required: jobs catch panics, and pyo3 needs unwinding).
- `[profile.bench] inherits = "release"`, `debug = true`.

**`deny.toml`:**
- `licenses.allow` = MIT, Apache-2.0, Apache-2.0 WITH LLVM-exception, BSD-2-Clause,
  BSD-3-Clause, ISC, Zlib, Unicode-3.0, BSL-1.0, CC0-1.0, MIT-0, MPL-2.0.
- GPL/AGPL/LGPL Rust crates are only allowed through per-crate `exceptions` that are listed
  in `THIRD_PARTY.md` and feature-gated.
- `bans.multiple-versions = "warn"`, `sources.unknown-git = "deny"`, `advisories` on.
- CI runs `cargo deny check` twice: once with default features (this must pass with zero
  copyleft, which is the ADR-012 permissive build) and once with `--all-features`.
- C++ deps from vcpkg are invisible to cargo-deny and stay tracked by hand in
  `THIRD_PARTY.md`.

### 3. `amg-core` (FND-03 math/tolerance/ids/primitives/errors; FND-10 units)

- **Math:** type aliases over `glam` f64 types. They are aliases, not newtypes, to keep glam's
  operator ergonomics and zero conversion cost. Points and vectors share `DVec3`, and the
  naming distinguishes them. f32 glam types are *not* re-exported. Code converts to f32 only
  at the GPU boundary through `amg_core::gpu::to_f32*`.
- **Tolerance:** an explicit value that is never global. The defaults are linear 1e-6 mm and
  angular 1e-9 rad. They are deliberately coarser than OCCT's `Precision::Confusion()`
  (1e-7) and `Precision::Angular()` (1e-12), so values that come back from OCCT compare
  equal. A `Tolerance` travels inside the doc/recompute context (S01). Mesh code may use
  `Tolerance::mesh()` (linear 1e-5 mm) for welding.
- **Ids:**
  - Runtime ids are `slotmap` generational keys, declared per crate with
    `amg_core::define_id!`.
  - File-persistent ids are `PersistentId(Uuid)`. `new()` creates a v7 id (time-ordered).
    Only commands call it, so recompute stays deterministic.
  - `derive(parent, role)` creates a v5 id (SHA-1 namespace hash). Recompute uses it when an
    output needs a stable id derived from its generator.
  - Slotmap keys are never written to files.
- **Primitives:** `Dir3` (a unit vector, checked once), `Aabb3`/`Aabb2`, `Plane` (a full
  frame: origin, x, y, normal, because sketch planes need an orientation), `Ray3`, `Line3`,
  `Segment3`, `Line2`, `Segment2`, `Triangle3`.
- **Intersection and distance:** ray–plane, ray–AABB (slab method with NaN-safe handling of
  0-components), ray–triangle (watertight, Woop–Benthin–Wald 2013, so picking never slips
  through shared edges), point–segment, segment–segment closest points (Ericson,
  *Real-Time Collision Detection* §5.1.9), plane–plane line, and AABB–AABB.
- **Units:** see [docs/notes/units-and-quantities.md](../notes/units-and-quantities.md).
  Canonical values use the coherent mm–t–s–K–rad system (N, MPa, mJ). The note covers
  `Dimension`, `Quantity`, the typed newtypes, the `Unit` table, the single-literal grammar
  (`12.5 mm`, `1/4 in`, `1 1/4 in`, `5' 3"`, `30 deg`), and the formatter with its round-trip
  contract. Compound expressions belong to S01.
- **Errors:**
  - `Display` is the short user-facing message. `source()` and `Debug` carry the developer
    context.
  - `amg_core::Error` covers core failures.
  - Every library crate defines its own `thiserror` enum and implements `Diagnostic`, so the
    UI (UI-10) can show the message, the hint and the source chain uniformly.
  - `Cancelled` is a shared zero-size error. Every crate's error type implements
    `From<Cancelled>`, so `ctx.checkpoint()?` works everywhere.

### 4. `amg-jobs` (FND-04 CPU jobs; FND-09 GPU; FND-07 diagnostics)

- **Pools:** two `rayon::ThreadPool`s.
  - `Interactive` is for recompute, previews and picking-adjacent work. It has
    `available_parallelism()` threads.
  - `Background` is for studies, large remeshes and lattice meshing. It has
    `max(1, n − 1)` threads, and the OS priority of its threads is set to below-normal.
  - Recompute (< 500 ms budget) therefore never queues behind a 5-minute topopt run.
  - Inside `run`, jobs may use `rayon` parallel iterators. They execute on the job's own pool.
- **Job lifecycle:** `Queued → Running → Finished(Ok | Failed | Cancelled | Panicked)`.
  - `run` executes under `catch_unwind`. A panic becomes `JobError::Panicked(msg)`; the
    global panic hook (FND-07) also logs it.
  - If the job returns `Err` *after* its token was cancelled, the result is classified as
    `JobError::Cancelled`, whatever the error value.
- **Cancellation:** a `CancelToken` (`Arc<AtomicBool>` plus a parent link). Cancelling a
  parent cancels all children. Jobs must call `ctx.checkpoint()?` at least every 10 ms of
  work (the convention for domain crates). `JobSystem::cancel_all()` exists for shutdown.
- **Progress:** each `JobCtx` covers a sub-range `[lo, hi]` of the root job's `[0, 1]`.
  - `ctx.sub(a, b)` narrows it. This is how nested steps report progress without knowing
    about each other.
  - The fraction is stored as `AtomicU32` (f32 bits) and is monotone (it only rises).
  - The message is held in a `parking_lot::Mutex<Arc<str>>`.
  - Updates cost one atomic store. Message updates are rate-limited to 30 Hz.
- **Child jobs:** `ctx.spawn_child(job)` links the child's cancel token to the parent and
  records `parent` in `JobInfo`. `ctx.wait(handle)` blocks only job threads, using
  `rayon::yield_now()` work-stealing, so a pool of size 1 can't deadlock. It panics in debug
  if called on the UI thread.
- **UI integration:** `JobHandle::try_take()` and `status()` never block (atomics plus a
  `crossbeam_channel::Receiver::try_recv`). `JobSystemConfig::notifier` is called after every
  state change and rate-limited progress change. The app passes
  `egui::Context::request_repaint` there, so the UI polls only when something changed.
- **UI-thread guard:** `amg_jobs::mark_ui_thread()` is called once by `amg-app`. Every
  blocking API (`wait_blocking`, `Readback::wait`, `JobSystem::install`) has
  `debug_assert!(!is_ui_thread())`. The one exception is `Gpu::new_blocking`, which may run
  at startup before the event loop starts.
- **GPU (FND-09):** `Gpu` owns `wgpu::Instance`, `Adapter`, `Device` and `Queue` (wgpu
  handles are `Clone` and internally ref-counted) and hands out clones.
  - **Sharing with the viewport:** `amg-app` builds the egui-wgpu renderer from
    `egui_wgpu::WgpuSetup::Existing` with these same handles. This gives one device for render
    and compute, as the architecture §Threading requires. Consequence: the `wgpu` major
    version must equal the one used by the egui-wgpu release that S11/S10 pick. FND-09 starts
    with the newest wgpu major that the latest `egui-wgpu` supports (≤ 29 at refinement).
  - **Backend:** DX12 on Windows by default. `AMG_WGPU_BACKEND=dx12|vulkan|warp` overrides it.
    `warp` sets `force_fallback_adapter` (Microsoft WARP software rasterizer), which is what CI
    uses because GitHub runners have no GPU.
  - **Features:** `SHADER_F64` and `TIMESTAMP_QUERY` are *requested if available* and exposed
    via `Gpu::has()`. They are never required.
  - **Helpers:** typed upload, storage-buffer allocation, non-blocking `Readback` (staging
    copy, `map_async`, polled through `device.poll(Poll)`), and a compute-pipeline cache keyed
    by a hash of (WGSL source, entry point, override constants).
  - **Device loss:** a device-lost callback flips `Gpu::is_lost()`, and callers receive
    `GpuError::DeviceLost`. Re-creating the device is out of scope (S10/S11 decide the UX).
- **Diagnostics (FND-07):**
  - `amg_jobs::diag::init(&DiagConfig)` installs `tracing-subscriber`: an env-filter
    (`AMG_LOG`), stderr output, and a daily rolling file under `%LOCALAPPDATA%\Amalgam\logs`
    (`tracing-appender`, keep 7 files). An optional `tracy` cargo feature adds
    `tracing-tracy`.
  - A panic hook logs the panic and a backtrace. It writes `crash-<utc>.txt` (version, git
    SHA, adapter info, last 200 log lines from an in-memory ring layer) and calls the
    registered `RecoveryHook`s. S01's autosave registers one that flushes the recovery file
    and returns its path. The hook prints `Recovery file: <path>`.
  - Native crashes (an access violation inside OCCT) are not panics. Catching them needs an
    out-of-process minidump writer (`crash-handler` + `minidumper`, MIT/Apache). That is a
    separate P2 WP (FND-12) and is not needed for Phase 1.

### 5. `amg-testkit` (FND-06, FND-08)
A dev-only crate (`publish = false`, used under `[dev-dependencies]`). It **does not depend on
any domain crate**, because domain crates use it. Geometry is passed as a borrowed
triangle-soup view, so `amg-mesh`, `amg-brep` tessellations and `amg-implicit` output all
adapt with a one-line conversion.
- **Metrics:** signed volume (divergence theorem), area, bbox, Euler characteristic and
  boundary/non-manifold edge counts, and the sampled two-sided Hausdorff distance (area-
  weighted surface samples with a fixed seed, nearest-triangle queries through an internal
  BVH).
- **Golden files:** `tests/golden/<suite>/<case>.toml` holds expected metrics and per-metric
  tolerances. `assert_golden!(suite, case, soup)` compares against the file. With
  `AMG_BLESS=1` it writes the file instead (the diff is reviewed in the PR).
- **`testdata/`:** git-LFS for `*.stl *.3mf *.step *.stp *.obj *.ply *.amg`.
  `amg_testkit::testdata(path)` resolves the path and *skips with a message* if the LFS
  object is an un-fetched pointer file. In CI the `golden` job has `lfs: true`, so it fails
  there instead of skipping.
- **Budgets:** `amg_testkit::budget::BUDGETS` is one `Budget` per vision row. Each budget's
  criterion bench lives in the owning crate and is named `budget/<id>`. FND-08 adds
  placeholder benches in `amg-testkit/benches/budgets.rs` that owners later move or replace.

### 6. Binaries
- `amg-cli` (FND-02 skeleton, lane E): `clap` derive.
  - `amg-cli --version` prints the crate version and git SHA.
  - `amg-cli info` prints the build profile, thread counts and (after FND-09) the GPU
    adapter.
  - S18 (SCR-05) adds `recompute`, `export`, `study run` and `convert`.
- `amg-app`: an empty `main` stub created by FND-02 so the workspace builds. Lane D owns it
  from RND-01/UI-01 on.

### 7. CI (FND-05) — `.github/workflows/ci.yml`

- **Triggers:** `push` to `main`, `pull_request`, `workflow_dispatch`. A `concurrency` group
  per ref with `cancel-in-progress: true`.
- **Runner:** `runs-on: windows-2025`, named explicitly. `windows-latest` points to the same
  image today but moves without notice. Since June 2026 that image has VS 2026. See open
  question 1.
- **Toolchain:** rustup is preinstalled. The first `cargo` call installs the toolchain from
  `rust-toolchain.toml`, so no third-party toolchain action is needed. The cargo cache is
  `Swatinem/rust-cache@v2`, with key including `rust-toolchain.toml` and `Cargo.lock`. Tools
  are installed with `taiki-e/install-action` (`cargo-nextest`, `cargo-deny`).
- **vcpkg:**
  - Clone `microsoft/vcpkg` at the pinned tag into `${{ runner.temp }}\vcpkg` with
    `--depth 1`. The tag commit is the baseline, so a shallow clone is enough. Bootstrap it
    and set `VCPKG_ROOT`.
  - **Binary caching:** the GitHub-Actions provider `x-gha` was removed from vcpkg in
    2025.06.13. We use `VCPKG_BINARY_SOURCES=clear;files,${{ runner.temp }}\vcpkg-bin,readwrite`
    and persist that directory with `actions/cache@v4`. The key is
    `vcpkg-${{ hashFiles('vcpkg.json','vcpkg-configuration.json','triplets/**') }}`, with
    restore-key prefix `vcpkg-`.
  - vcpkg ABI hashes include the compiler version, so stale entries are simply unused.
  - All vcpkg steps are skipped while `vcpkg.json` has no dependencies (they are no-ops until
    BREP-01).
  - Upgrade path if per-port granularity matters: NuGet on GitHub Packages, which is
    Microsoft's recommendation. It needs a `packages: write` token and is deferred until OCCT
    rebuilds become painful.
- **Jobs** (in parallel; `fmt` is the fastest gate):
  1. `fmt`: `cargo fmt --all --check`
  2. `clippy`: `cargo clippy --workspace --all-targets -- -D warnings`
  3. `test`: `cargo nextest run --workspace --profile ci` with `AMG_WGPU_BACKEND=warp` and
     `AMG_REQUIRE_GPU=1`, so GPU tests run on WARP and fail rather than skip. JUnit is
     uploaded as an artifact.
  4. `deny`: `cargo deny check` with default features and with `--all-features`.
  5. `golden` (needs `test`): checkout with `lfs: true`, plus `actions/cache` on `.git/lfs`
     keyed by the hash of `git lfs ls-files -l` output. GitHub Free includes 10 GiB/month of
     LFS bandwidth, and this cache keeps CI from consuming it. Then
     `cargo nextest run --workspace --profile ci -E 'test(golden_)'`.
- **Not in CI:** benches (`cargo bench` is noisy on shared runners). An optional
  `workflow_dispatch`-only `bench` job uploads criterion reports.

## Interfaces

All signatures are for the public API other crates use. Bodies are omitted. Every public item
gets a doc comment, and `amg-core` and `amg-jobs` set `#![warn(missing_docs)]`.

### `amg-core`
```rust
// ---- amg_core::math ---------------------------------------------------------------------
pub use glam; // escape hatch; prefer the aliases below
pub type Point3 = glam::DVec3;     // position in mm
pub type Vec3 = glam::DVec3;       // displacement / direction (not necessarily unit)
pub type Point2 = glam::DVec2;
pub type Vec2 = glam::DVec2;
pub type Mat3 = glam::DMat3;
pub type Mat4 = glam::DMat4;
pub type Quat = glam::DQuat;
pub type Transform = glam::DAffine3;   // rigid or affine 3D transform, mm
pub type Transform2 = glam::DAffine2;

pub mod gpu {
    /// Lossy f64→f32 conversion; the only sanctioned path to GPU data.
    pub fn to_f32(v: super::Vec3) -> glam::Vec3;
    pub fn to_f32_slice(src: &[super::Point3], dst: &mut Vec<[f32; 3]>);
    pub fn transform_to_f32(t: &super::Transform) -> glam::Mat4;
}

/// Unit-length direction; invariant checked once at construction.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(try_from = "Vec3", into = "Vec3")]
pub struct Dir3(Vec3);
impl Dir3 {
    pub const X: Dir3; pub const Y: Dir3; pub const Z: Dir3;
    pub const NEG_X: Dir3; pub const NEG_Y: Dir3; pub const NEG_Z: Dir3;
    /// `None` if |v| <= tol.linear (cannot normalise).
    pub fn new(v: Vec3, tol: &Tolerance) -> Option<Dir3>;
    pub fn new_unchecked(v: Vec3) -> Dir3; // debug_assert normalised
    pub fn get(self) -> Vec3;
    pub fn any_orthogonal(self) -> Dir3;
}
impl std::ops::Neg for Dir3 { type Output = Dir3; fn neg(self) -> Dir3; }
impl TryFrom<Vec3> for Dir3 { type Error = Error; fn try_from(v: Vec3) -> Result<Dir3>; }
impl From<Dir3> for Vec3 { fn from(d: Dir3) -> Vec3; }

// ---- amg_core::tolerance ----------------------------------------------------------------
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Tolerance {
    /// Distance below which two points coincide, mm. Default 1e-6.
    pub linear: f64,
    /// Angle below which two directions are parallel, rad. Default 1e-9.
    pub angular: f64,
}
impl Tolerance {
    pub const DEFAULT: Tolerance = Tolerance { linear: 1e-6, angular: 1e-9 };
    pub const fn mesh() -> Tolerance; // linear 1e-5 mm, angular 1e-9
    pub fn eq_len(&self, a: f64, b: f64) -> bool;
    pub fn is_zero_len(&self, a: f64) -> bool;
    pub fn eq_point(&self, a: Point3, b: Point3) -> bool;
    pub fn eq_angle(&self, a: f64, b: f64) -> bool;
    pub fn is_parallel(&self, a: Dir3, b: Dir3) -> bool;      // same or opposite
    pub fn is_codirectional(&self, a: Dir3, b: Dir3) -> bool;
    pub fn is_perpendicular(&self, a: Dir3, b: Dir3) -> bool;
}
impl Default for Tolerance { fn default() -> Self { Tolerance::DEFAULT } }

// ---- amg_core::id -----------------------------------------------------------------------
pub use slotmap::{DenseSlotMap, Key, KeyData, SecondaryMap, SlotMap, SparseSecondaryMap};
/// Declares a runtime generational id (slotmap key). Never serialised to files.
/// `amg_core::define_id! { pub struct BodyId; }`
#[macro_export]
macro_rules! define_id {
    ($($(#[$m:meta])* $vis:vis struct $name:ident;)+) => {
        $crate::__private::slotmap::new_key_type! { $($(#[$m])* $vis struct $name;)+ }
    };
}
#[doc(hidden)] pub mod __private { pub use slotmap; }

/// Stable id stored in `.amg` files and exposed to Python.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
#[serde(transparent)]
pub struct PersistentId(pub uuid::Uuid);
impl PersistentId {
    /// UUID v7 (time-ordered). Only commands may call this, never recompute (determinism).
    pub fn new() -> PersistentId;
    /// UUID v5 in namespace `parent`: deterministic id for recompute outputs.
    pub fn derive(parent: PersistentId, role: &[u8]) -> PersistentId;
    pub const NIL: PersistentId;
}
impl std::fmt::Display for PersistentId { /* hyphenated */ }
impl std::str::FromStr for PersistentId { type Err = Error; /* … */ }

// ---- amg_core::geom ---------------------------------------------------------------------
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Aabb3 { pub min: Point3, pub max: Point3 }
impl Aabb3 {
    pub const EMPTY: Aabb3; // min = +inf, max = -inf
    pub fn new(a: Point3, b: Point3) -> Aabb3; // component-wise min/max
    pub fn from_points<I: IntoIterator<Item = Point3>>(pts: I) -> Aabb3;
    pub fn is_empty(&self) -> bool;
    pub fn center(&self) -> Point3;
    pub fn size(&self) -> Vec3;
    pub fn diagonal(&self) -> f64;
    pub fn union(&self, o: &Aabb3) -> Aabb3;
    pub fn include_point(&mut self, p: Point3);
    pub fn expanded(&self, by: f64) -> Aabb3;
    pub fn contains(&self, p: Point3, tol: &Tolerance) -> bool;
    pub fn intersects(&self, o: &Aabb3, tol: &Tolerance) -> bool;
    /// Conservative bounds of the transformed box (Arvo's method).
    pub fn transformed(&self, t: &Transform) -> Aabb3;
}
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Aabb2 { pub min: Point2, pub max: Point2 }
// Aabb2: same method set as Aabb3 minus `transformed` (plus `transformed2(&Transform2)`).

/// Oriented plane = right-handed frame; local (u, v) ↔ world.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Plane { pub origin: Point3, pub x: Dir3, pub y: Dir3, pub normal: Dir3 }
impl Plane {
    pub const XY: Plane; pub const YZ: Plane; pub const XZ: Plane;
    /// x chosen deterministically from `normal` (same input → same frame).
    pub fn from_normal(origin: Point3, normal: Dir3) -> Plane;
    /// `x_hint` projected into the plane; Err if parallel to normal.
    pub fn from_normal_x(origin: Point3, normal: Dir3, x_hint: Vec3, tol: &Tolerance) -> Result<Plane>;
    pub fn from_points(a: Point3, b: Point3, c: Point3, tol: &Tolerance) -> Result<Plane>;
    pub fn signed_distance(&self, p: Point3) -> f64;
    pub fn project(&self, p: Point3) -> Point3;
    pub fn to_local(&self, p: Point3) -> Point2;
    pub fn to_world(&self, uv: Point2) -> Point3;
    pub fn to_transform(&self) -> Transform; // local (u,v,w) → world
    pub fn flipped(&self) -> Plane;
}
#[derive(Clone, Copy, Debug, PartialEq)] pub struct Ray3 { pub origin: Point3, pub dir: Dir3 }
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Line3 { pub point: Point3, pub dir: Dir3 }
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Segment3 { pub a: Point3, pub b: Point3 }
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Line2 { pub point: Point2, pub dir: Vec2 /* unit */ }
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Segment2 { pub a: Point2, pub b: Point2 }
#[derive(Clone, Copy, Debug, PartialEq)] pub struct Triangle3 { pub a: Point3, pub b: Point3, pub c: Point3 }

impl Ray3 { pub fn at(&self, t: f64) -> Point3; }
impl Segment3 { pub fn length(&self) -> f64; pub fn closest_point(&self, p: Point3) -> (f64 /*t∈[0,1]*/, Point3); }
impl Triangle3 { pub fn normal(&self, tol: &Tolerance) -> Option<Dir3>; pub fn area(&self) -> f64; }

pub mod intersect {
    use super::*;
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct RayHit { pub t: f64, pub point: Point3 }
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct TriHit { pub t: f64, pub bary: [f64; 3] }
    pub fn ray_plane(r: &Ray3, p: &Plane, tol: &Tolerance) -> Option<RayHit>; // None if parallel
    /// Slab test; returns (t_enter, t_exit) clipped to t >= 0.
    pub fn ray_aabb(r: &Ray3, b: &Aabb3) -> Option<(f64, f64)>;
    /// Watertight ray/triangle (Woop, Benthin, Wald 2013); both sides hit.
    pub fn ray_triangle(r: &Ray3, tri: &Triangle3) -> Option<TriHit>;
    pub fn plane_plane(a: &Plane, b: &Plane, tol: &Tolerance) -> Option<Line3>;
    pub fn line_plane(l: &Line3, p: &Plane, tol: &Tolerance) -> Option<Point3>;
    /// Closest points between segments (Ericson 5.1.9): (s, t, pa, pb).
    pub fn segment_segment(a: &Segment3, b: &Segment3) -> (f64, f64, Point3, Point3);
    pub fn line_line(a: &Line3, b: &Line3, tol: &Tolerance) -> Option<(Point3, Point3)>; // None if parallel
    pub fn point_segment_distance(p: Point3, s: &Segment3) -> f64;
    pub fn point_triangle_closest(p: Point3, tri: &Triangle3) -> Point3;
    pub fn segment2_segment2(a: &Segment2, b: &Segment2, tol: &Tolerance) -> Seg2Intersection;
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum Seg2Intersection { None, Point { t: f64, u: f64, p: Point2 }, Overlap { t0: f64, t1: f64 } }
}

// ---- amg_core::units  (FND-10; semantics in docs/notes/units-and-quantities.md) ----------
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default, serde::Serialize, serde::Deserialize)]
pub struct Dimension { pub length: i8, pub mass: i8, pub time: i8, pub temperature: i8, pub angle: i8 }
impl Dimension {
    pub const NONE: Dimension; pub const LENGTH: Dimension; pub const AREA: Dimension;
    pub const VOLUME: Dimension; pub const ANGLE: Dimension; pub const MASS: Dimension;
    pub const TIME: Dimension; pub const FORCE: Dimension; pub const PRESSURE: Dimension;
    pub const DENSITY: Dimension; pub const ENERGY: Dimension; pub const TEMPERATURE: Dimension;
    pub fn pow(self, n: i8) -> Dimension;
}
impl std::ops::Mul for Dimension { type Output = Dimension; fn mul(self, o: Dimension) -> Dimension; }
impl std::ops::Div for Dimension { type Output = Dimension; fn div(self, o: Dimension) -> Dimension; }
impl std::fmt::Display for Dimension { /* "mm^2", "N", "MPa", "mm*t/s^2" fallback */ }

/// Value in canonical coherent units (mm, t, s, K, rad) with its dimension.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Quantity { pub value: f64, pub dim: Dimension }
impl Quantity {
    pub const fn new(value: f64, dim: Dimension) -> Quantity;
    pub fn dimensionless(v: f64) -> Quantity;
    pub fn in_unit(&self, u: Unit) -> Result<f64>;            // Err(DimensionMismatch)
    pub fn from_unit(v: f64, u: Unit) -> Quantity;
    pub fn checked_add(self, o: Quantity) -> Result<Quantity>;
    pub fn checked_sub(self, o: Quantity) -> Result<Quantity>;
    pub fn powi(self, n: i8) -> Quantity;
    pub fn expect_dim(self, d: Dimension) -> Result<f64>;
}
impl std::ops::Mul for Quantity { type Output = Quantity; fn mul(self, o: Quantity) -> Quantity; }
impl std::ops::Div for Quantity { type Output = Quantity; fn div(self, o: Quantity) -> Quantity; }
impl std::ops::Neg for Quantity { type Output = Quantity; fn neg(self) -> Quantity; }

/// Typed quantities (repr(transparent) over canonical f64). Generated by an internal macro;
/// each has: ZERO, `get()`/`from_canonical()` (canonical raw), named constructors/accessors,
/// Add/Sub/Neg/Mul<f64>/Div<f64>/Div<Self>→f64, PartialOrd, Serialize as canonical f64,
/// From<T> for Quantity, TryFrom<Quantity> for T.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default, serde::Serialize, serde::Deserialize)]
#[repr(transparent)] #[serde(transparent)]
pub struct Length(f64);
impl Length {
    pub const ZERO: Length;
    pub const fn mm(v: f64) -> Length; pub fn um(v: f64) -> Length; pub fn cm(v: f64) -> Length;
    pub fn m(v: f64) -> Length; pub fn inches(v: f64) -> Length; pub fn feet(v: f64) -> Length;
    pub const fn get(self) -> f64; // mm
    pub fn to_unit(self, u: LengthUnit) -> f64;
}
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default, serde::Serialize, serde::Deserialize)]
#[repr(transparent)] #[serde(transparent)]
pub struct Angle(f64);
impl Angle {
    pub const ZERO: Angle;
    pub const fn rad(v: f64) -> Angle; pub fn deg(v: f64) -> Angle;
    pub const fn get(self) -> f64; pub fn to_deg(self) -> f64;
    pub fn normalized_0_2pi(self) -> Angle; pub fn normalized_pm_pi(self) -> Angle;
}
// Same pattern (constructors in brackets; canonical unit in parentheses):
// Area (mm²) [mm2, cm2, m2, in2]   Volume (mm³) [mm3, cm3, litres, ml, m3, in3]
// Mass (t) [g, kg, tonnes, lb]   Time (s) [ms, s, min, h]   Force (N) [newtons, kn, lbf]
// Pressure (MPa) [pa, kpa, mpa, gpa, bar, psi]   Density (t/mm³) [kg_per_m3, g_per_cm3]
// Energy (mJ) [mj, j]   Temperature (K) [kelvin, celsius, fahrenheit]
// Cross-type ops: Length*Length=Area, Area*Length=Volume, Force/Area=Pressure,
// Mass/Volume=Density, Force*Length=Energy.

/// Every parse/display unit (table: docs/notes/units-and-quantities.md §6).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum Unit {
    Micrometre, Millimetre, Centimetre, Metre, Inch, Foot, Thou,
    Radian, Degree, Gradian, Turn,
    SquareMillimetre, SquareCentimetre, SquareMetre, SquareInch,
    CubicMillimetre, CubicCentimetre, CubicMetre, Litre, Millilitre, CubicInch,
    Gram, Kilogram, Tonne, Pound, Ounce,
    Millisecond, Second, Minute, Hour,
    Newton, Kilonewton, PoundForce,
    Pascal, Kilopascal, Megapascal, Gigapascal, Bar, Psi, Ksi,
    KilogramPerCubicMetre, GramPerCubicCentimetre,
    Millijoule, Joule,
    Kelvin, DegreeCelsius, DegreeFahrenheit, DeltaCelsius,
}
impl Unit {
    pub fn dimension(self) -> Dimension;
    pub fn symbol(self) -> &'static str;          // canonical printed symbol
    pub fn aliases(self) -> &'static [&'static str];
    pub fn to_canonical(self, v: f64) -> f64;      // scale + offset
    pub fn from_canonical(self, v: f64) -> f64;
    pub fn from_symbol(s: &str) -> Option<Unit>;
    pub fn all() -> &'static [Unit];
}
/// Subset used for document length unit / display preferences.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default, serde::Serialize, serde::Deserialize)]
pub enum LengthUnit { Micrometre, #[default] Millimetre, Centimetre, Metre, Inch, Foot }
impl From<LengthUnit> for Unit { fn from(u: LengthUnit) -> Unit; }

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParsedQuantity { pub value: f64 /* in `unit` (not canonical) */, pub unit: Option<Unit> }
impl ParsedQuantity {
    /// Apply a context default unit when none was typed; returns canonical Quantity.
    pub fn resolve(self, default: Unit) -> Quantity;
}
/// Parse ONE quantity literal ("12.5 mm", "1/4 in", "1 1/4\"", "5' 3 1/2\"", "30 deg").
pub fn parse_quantity(input: &str) -> std::result::Result<ParsedQuantity, ParseQuantityError>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FormatStyle { Decimal, Fraction { max_denominator: u32 }, FeetInch { max_denominator: u32 }, Exact }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FormatSpec { pub style: FormatStyle, pub precision: u8, pub trim_zeros: bool, pub show_unit: bool }
impl FormatSpec { pub fn for_unit(u: Unit) -> FormatSpec; /* 3 decimals length, 2 angle, … */ }
/// Err(DimensionMismatch) if `q.dim != unit.dimension()`.
pub fn format_quantity(q: Quantity, unit: Unit, spec: &FormatSpec) -> Result<String>;

#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum ParseQuantityError {
    #[error("empty value")] Empty,
    #[error("`{text}` is not a number")] InvalidNumber { span: std::ops::Range<usize>, text: String },
    #[error("unknown unit `{text}`")] UnknownUnit { span: std::ops::Range<usize>, text: String },
    #[error("division by zero in fraction")] ZeroDenominator { span: std::ops::Range<usize> },
    #[error("use `.` as decimal separator")] DecimalComma { span: std::ops::Range<usize> },
    #[error("unexpected `{text}`; compound expressions belong in parameter expressions")]
    Trailing { span: std::ops::Range<usize>, text: String },
}
impl ParseQuantityError { pub fn span(&self) -> Option<std::ops::Range<usize>>; }

// ---- amg_core::error --------------------------------------------------------------------
/// Shared cancellation marker; every crate's error implements `From<Cancelled>`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
#[error("operation cancelled")]
pub struct Cancelled;

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    #[error(transparent)] Parse(#[from] ParseQuantityError),
    #[error("unit mismatch: expected {expected}, got {found}")]
    DimensionMismatch { expected: Dimension, found: Dimension },
    #[error("degenerate geometry: {what}")] Degenerate { what: &'static str },
    #[error("invalid argument `{name}`: {reason}")] InvalidArgument { name: &'static str, reason: String },
    #[error("invalid id `{0}`")] InvalidId(String),
    #[error(transparent)] Cancelled(#[from] Cancelled),
}
pub type Result<T, E = Error> = std::result::Result<T, E>;

/// Uniform user-facing reporting for all library error enums (UI-10, CLI, Python).
pub trait Diagnostic: std::error::Error + Send + Sync + 'static {
    /// Short sentence for the user. Default: `to_string()`.
    fn user_message(&self) -> String { self.to_string() }
    /// Optional actionable hint ("Check that the sketch profile is closed").
    fn hint(&self) -> Option<String> { None }
    fn severity(&self) -> Severity { Severity::Error }
    fn is_cancelled(&self) -> bool { false }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity { Info, Warning, Error }
impl Diagnostic for Error { /* is_cancelled for Cancelled; hints for Parse/DimensionMismatch */ }
/// Renders message + hint + `source()` chain (debug context) for logs / crash reports.
pub fn report(e: &(dyn std::error::Error + 'static)) -> String;
```

### `amg-jobs`
```rust
use std::{borrow::Cow, num::NonZeroUsize, sync::Arc, time::{Duration, Instant}};
pub use amg_core::Cancelled;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct JobId(pub u64); // monotonically increasing per JobSystem

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Priority { #[default] Interactive, Background }

pub struct JobSystemConfig {
    pub interactive_threads: Option<NonZeroUsize>, // default: available_parallelism
    pub background_threads: Option<NonZeroUsize>,  // default: max(1, n-1), below-normal OS priority
    /// Called (from worker threads) after state changes and rate-limited progress updates.
    pub notifier: Option<Arc<dyn Fn() + Send + Sync>>,
}
impl Default for JobSystemConfig { fn default() -> Self; }

#[derive(Debug, thiserror::Error)]
pub enum JobSystemError {
    #[error("failed to build thread pool: {0}")] Pool(#[from] rayon::ThreadPoolBuildError),
}

/// Cheap to clone (Arc inside); Send + Sync.
#[derive(Clone)]
pub struct JobSystem { /* Arc<Inner> */ }
impl JobSystem {
    pub fn new(cfg: JobSystemConfig) -> Result<JobSystem, JobSystemError>;
    pub fn spawn<J: Job>(&self, job: J) -> JobHandle<J::Output, J::Error>;
    pub fn spawn_with<J: Job>(&self, job: J, opts: SpawnOptions) -> JobHandle<J::Output, J::Error>;
    /// Closure convenience.
    pub fn spawn_fn<T, E, F>(&self, label: impl Into<Cow<'static, str>>, opts: SpawnOptions, f: F) -> JobHandle<T, E>
    where
        T: Send + 'static,
        E: std::error::Error + From<Cancelled> + Send + Sync + 'static,
        F: FnOnce(&JobCtx) -> Result<T, E> + Send + 'static;
    /// Snapshot for the job panel (UI-10). Non-blocking (short lock).
    pub fn jobs(&self) -> Vec<JobInfo>;
    pub fn cancel(&self, id: JobId) -> bool;
    pub fn cancel_all(&self);
    /// Run `f` inside a pool so its rayon iterators use that pool. Blocking — not on UI thread.
    pub fn install<R: Send>(&self, priority: Priority, f: impl FnOnce() -> R + Send) -> R;
    /// Cancels everything and waits up to `timeout` for workers to finish.
    pub fn shutdown(&self, timeout: Duration) -> ShutdownReport;
}
#[derive(Clone, Debug, Default)]
pub struct SpawnOptions {
    pub priority: Priority,
    /// Hidden jobs (e.g. hover picking helpers) don't appear in `jobs()`.
    pub hidden: bool,
}
#[derive(Clone, Debug)]
pub struct ShutdownReport { pub cancelled: usize, pub still_running: Vec<JobInfo> }

pub trait Job: Send + 'static {
    type Output: Send + 'static;
    type Error: std::error::Error + From<Cancelled> + Send + Sync + 'static;
    fn label(&self) -> Cow<'static, str>;
    fn run(self, ctx: &JobCtx) -> Result<Self::Output, Self::Error>;
}

/// Passed to running work. Send + Sync + Clone: may be shared into rayon par_iter closures.
#[derive(Clone)]
pub struct JobCtx { /* Arc<JobState>, CancelToken, progress range */ }
impl JobCtx {
    /// For synchronous callers (tests, CLI, direct calls): never cancelled, progress discarded.
    pub fn detached() -> JobCtx;
    /// Like `detached` but cancellable via the returned token (tests of cancel paths).
    pub fn with_token(token: CancelToken) -> JobCtx;
    pub fn id(&self) -> Option<JobId>;
    pub fn is_cancelled(&self) -> bool;
    /// `Err(Cancelled)` once cancelled. Call at least every ~10 ms of work.
    pub fn checkpoint(&self) -> Result<(), Cancelled>;
    /// `fraction` in [0,1] relative to this ctx's range; clamped; monotone.
    pub fn progress(&self, fraction: f32);
    pub fn progress_msg(&self, fraction: f32, message: &str);
    /// Sub-range [start, end] of this ctx's range, same cancellation.
    pub fn sub(&self, start: f32, end: f32) -> JobCtx;
    pub fn spawn_child<J: Job>(&self, job: J) -> JobHandle<J::Output, J::Error>;
    /// Waits for a child by work-stealing (rayon::yield_now). Job threads only.
    pub fn wait<T, E>(&self, handle: JobHandle<T, E>) -> Result<T, JobError<E>>;
    pub fn cancel_token(&self) -> CancelToken;
}

#[derive(Clone, Debug, Default)]
pub struct CancelToken { /* Arc<AtomicBool> + parent */ }
impl CancelToken {
    pub fn new() -> CancelToken;
    pub fn child(&self) -> CancelToken;
    pub fn cancel(&self);
    pub fn is_cancelled(&self) -> bool;
}

#[derive(Clone, Debug, PartialEq)]
pub struct Progress { pub fraction: f32, pub message: Option<Arc<str>> }

#[derive(Clone, Debug, PartialEq)]
pub enum JobState { Queued, Running(Progress), Succeeded, Failed(String), Cancelled, Panicked(String) }

#[derive(Clone, Debug)]
pub struct JobInfo {
    pub id: JobId, pub label: Cow<'static, str>, pub priority: Priority,
    pub parent: Option<JobId>, pub state: JobState, pub created: Instant, pub started: Option<Instant>,
}

/// Owned by the spawner. Dropping it detaches (job keeps running, result discarded);
/// call `cancel()` first to stop it. All methods except `wait_blocking` are non-blocking.
pub struct JobHandle<T, E> { /* … */ }
impl<T: Send + 'static, E: Send + 'static> JobHandle<T, E> {
    pub fn id(&self) -> JobId;
    pub fn state(&self) -> JobState;
    pub fn progress(&self) -> Progress;
    pub fn is_finished(&self) -> bool;
    /// `Some` exactly once, when finished.
    pub fn try_take(&mut self) -> Option<Result<T, JobError<E>>>;
    pub fn cancel(&self);
    pub fn cancel_token(&self) -> CancelToken;
    /// Blocks the calling thread. debug_assert!(!is_ui_thread()). CLI / tests only.
    pub fn wait_blocking(self) -> Result<T, JobError<E>>;
}

#[derive(Debug, thiserror::Error)]
pub enum JobError<E> {
    #[error("cancelled")] Cancelled,
    #[error("internal error (panic): {0}")] Panicked(String),
    #[error(transparent)] Failed(E),
}
impl<E: amg_core::Diagnostic> amg_core::Diagnostic for JobError<E> { /* delegates */ }

/// Marks the current thread as the UI thread (called once by amg-app).
pub fn mark_ui_thread();
pub fn is_ui_thread() -> bool;

// ---- amg_jobs::gpu  (FND-09, cargo feature "gpu", default on) ---------------------------
pub mod gpu {
    use std::{ops::Range, sync::Arc};

    #[derive(Clone, Debug)]
    pub struct GpuConfig {
        pub backends: wgpu::Backends,               // default: DX12 on Windows, PRIMARY elsewhere
        pub power_preference: wgpu::PowerPreference,// default: HighPerformance
        pub force_fallback_adapter: bool,           // WARP on Windows (CI)
        pub required_features: wgpu::Features,      // default: empty
        pub optional_features: wgpu::Features,      // default: SHADER_F64 | TIMESTAMP_QUERY
        pub required_limits: wgpu::Limits,          // default: adapter limits (downlevel-safe floor)
    }
    impl Default for GpuConfig { fn default() -> Self; }
    impl GpuConfig {
        /// Applies AMG_WGPU_BACKEND=dx12|vulkan|warp on top of `default()`.
        pub fn from_env() -> GpuConfig;
    }

    /// Shared device for render + compute. Create once; share as Arc<Gpu>.
    pub struct Gpu { /* instance, adapter, device, queue, info, pipeline cache, lost flag */ }
    impl Gpu {
        pub async fn new(cfg: &GpuConfig) -> Result<Arc<Gpu>, GpuError>;
        /// pollster-based; debug_assert!(!is_ui_thread()) is NOT applied (startup before the
        /// event loop is allowed), but it must not be called from egui frames.
        pub fn new_blocking(cfg: &GpuConfig) -> Result<Arc<Gpu>, GpuError>;
        pub fn instance(&self) -> &wgpu::Instance;
        pub fn adapter(&self) -> &wgpu::Adapter;
        pub fn device(&self) -> &wgpu::Device;
        pub fn queue(&self) -> &wgpu::Queue;
        pub fn info(&self) -> &wgpu::AdapterInfo;
        pub fn has(&self, f: wgpu::Features) -> bool;
        pub fn is_lost(&self) -> bool;
        /// Creates a buffer initialised with `data` (COPY_DST added).
        pub fn upload<T: bytemuck::Pod>(&self, label: &str, data: &[T], usage: wgpu::BufferUsages) -> wgpu::Buffer;
        pub fn storage<T: bytemuck::Pod>(&self, label: &str, len: u64, usage: wgpu::BufferUsages) -> wgpu::Buffer;
        /// Copies `range` (bytes) of `src` (needs COPY_SRC) to a staging buffer and starts mapping.
        pub fn readback<T: bytemuck::Pod>(&self, src: &wgpu::Buffer, range: Range<u64>) -> Readback<T>;
        /// Cached by hash(wgsl, entry_point, constants). Validation errors → GpuError::Shader.
        pub fn compute_pipeline(&self, desc: &ComputePipelineDesc<'_>) -> Result<Arc<wgpu::ComputePipeline>, GpuError>;
        /// Non-blocking device poll (drives map callbacks).
        pub fn poll(&self) -> Result<(), GpuError>;
    }

    #[derive(Clone, Copy, Debug)]
    pub struct ComputePipelineDesc<'a> {
        pub label: &'a str,
        pub wgsl: &'a str,
        pub entry_point: &'a str,
        pub constants: &'a [(&'a str, f64)], // WGSL `override` constants
    }

    /// In-flight GPU→CPU copy.
    pub struct Readback<T> { /* staging buffer, shared map status, PhantomData<T> */ }
    impl<T: bytemuck::Pod> Readback<T> {
        /// Non-blocking: polls the device once; `Some` when mapped (or failed).
        pub fn try_get(&mut self) -> Option<Result<Vec<T>, GpuError>>;
        /// Blocking wait. debug_assert!(!is_ui_thread()). Job threads only.
        pub fn wait(self) -> Result<Vec<T>, GpuError>;
    }

    #[derive(Debug, thiserror::Error)]
    pub enum GpuError {
        #[error("no compatible GPU adapter found")] NoAdapter,
        #[error("GPU device request failed: {0}")] RequestDevice(#[from] wgpu::RequestDeviceError),
        #[error("GPU device lost: {0}")] DeviceLost(String),
        #[error("buffer mapping failed: {0}")] Map(#[from] wgpu::BufferAsyncError),
        #[error("shader `{label}` failed to compile: {message}")] Shader { label: String, message: String },
        #[error("GPU poll failed: {0}")] Poll(String),
        #[error(transparent)] Cancelled(#[from] amg_core::Cancelled),
    }
    impl amg_core::Diagnostic for GpuError { /* hints: update driver / set AMG_WGPU_BACKEND */ }
}

// ---- amg_jobs::diag  (FND-07) ------------------------------------------------------------
pub mod diag {
    use std::path::PathBuf;
    #[derive(Clone, Debug)]
    pub struct DiagConfig {
        pub app_name: &'static str,       // "amalgam" / "amalgam-cli"
        pub log_dir: Option<PathBuf>,     // default %LOCALAPPDATA%\Amalgam\logs
        pub default_filter: &'static str, // overridden by AMG_LOG
        pub stderr: bool,
    }
    /// Keep the guard alive for the whole process (flushes file logs on drop).
    #[must_use]
    pub struct DiagGuard { /* tracing_appender WorkerGuard */ }
    #[derive(Debug, thiserror::Error)]
    pub enum DiagError {
        #[error("cannot create log directory {path}: {source}")]
        LogDir { path: PathBuf, #[source] source: std::io::Error },
        #[error("a global tracing subscriber is already installed")] AlreadyInitialised,
    }
    pub fn init(cfg: &DiagConfig) -> Result<DiagGuard, DiagError>;
    /// Called from the panic hook; returns the path of anything it saved (e.g. recovery file).
    pub type RecoveryHook = Box<dyn Fn() -> Option<PathBuf> + Send + Sync>;
    pub fn register_recovery_hook(name: &'static str, hook: RecoveryHook);
    /// Installs the panic hook (logs + crash-<utc>.txt + recovery hooks). Idempotent.
    pub fn install_panic_hook();
}
```

### `amg-testkit` (dev-dependency only)
```rust
/// Borrowed triangle soup in mm. Domain crates adapt their meshes to this.
#[derive(Clone, Copy, Debug)]
pub struct TriSoup<'a> { pub positions: &'a [amg_core::Point3], pub triangles: &'a [[u32; 3]] }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MeshMetrics {
    pub volume: f64, pub area: f64, pub bbox: amg_core::Aabb3,
    pub vertices: usize, pub triangles: usize,
    pub boundary_edges: usize, pub non_manifold_edges: usize, pub euler: i64,
}
pub fn metrics(m: TriSoup<'_>) -> MeshMetrics;
pub fn signed_volume(m: TriSoup<'_>) -> f64;
pub fn area(m: TriSoup<'_>) -> f64;
/// Symmetric Hausdorff distance by area-weighted sampling (`samples` per mesh, fixed seed).
pub fn hausdorff(a: TriSoup<'_>, b: TriSoup<'_>, samples: usize) -> f64;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GoldenTolerances { pub volume_rel: f64, pub area_rel: f64, pub bbox_abs: f64, pub hausdorff_abs: Option<f64> }
pub fn golden_dir() -> std::path::PathBuf;          // <workspace>/tests/golden
/// Compares against tests/golden/<suite>/<case>.toml; AMG_BLESS=1 rewrites it.
pub fn check_golden(suite: &str, case: &str, m: TriSoup<'_>) -> Result<(), GoldenMismatch>;
#[macro_export] macro_rules! assert_golden { ($suite:expr, $case:expr, $soup:expr) => { /* panics with diff */ }; }
#[derive(Debug, thiserror::Error)]
#[error("golden mismatch {suite}/{case}: {details}")]
pub struct GoldenMismatch { pub suite: String, pub case: String, pub details: String }

/// Resolves testdata/<rel>; returns None (and prints a skip notice) if the LFS object is
/// an un-fetched pointer, unless AMG_REQUIRE_TESTDATA=1, in which case it panics.
pub fn testdata(rel: &str) -> Option<std::path::PathBuf>;
/// True when a GPU test should run; panics if AMG_REQUIRE_GPU=1 and no adapter exists.
pub fn gpu_available() -> bool;

pub mod budget {
    #[derive(Clone, Copy, Debug)]
    pub struct Budget { pub id: &'static str, pub description: &'static str, pub limit: Limit }
    #[derive(Clone, Copy, Debug)]
    pub enum Limit { MaxMillis(u64), MinFps(u32) }
    /// One entry per row of docs/00-vision.md §Performance budgets.
    pub const BUDGETS: &[Budget];
}
```

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| FND-01 | Dev setup doc + toolchain install: rustup, VS Build Tools 2022 17.14 (4 components), CMake 4.3, Ninja 1.13.2, vcpkg at `2026.07.29`, git-lfs, nextest, deny. `scripts/check-env.ps1`. | — | P0 | Done | `docs/dev-setup.md` contains the 9 sections of §1. On the owner's machine `scripts/check-env.ps1` exits 0 and prints every pinned tool version. A throwaway `cargo new` binary builds and runs with the MSVC linker. `vcpkg install zlib:x64-windows` (classic mode, outside the repo) succeeds, which proves the C++ toolchain works. The vcpkg baseline SHA is recorded in the doc. |
| FND-02 | Cargo workspace: 23 crates from §2 as stubs with architecture-conformant intra-workspace deps, `[workspace.dependencies]`, `[workspace.lints]`, profiles, `rust-toolchain.toml`, `rustfmt.toml`, `clippy.toml`, `deny.toml`, `.gitattributes`, `.editorconfig`, `.config/nextest.toml`, layering test, `amg-cli` skeleton (`--version`, `info`) | FND-01 | P0 | Done | `cargo build --workspace`, `cargo fmt --all --check`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo nextest run --workspace` and `cargo deny check` all pass locally. `cargo metadata` lists exactly the 23 `amg-*` packages. The layering test passes, and fails when a forbidden edge such as `amg-render → amg-doc` is added (shown in the PR description). `amg-cli --version` prints version + git SHA. |
| FND-03 | `amg-core` (part 1): math aliases + `gpu` conversions, `Dir3`, `Tolerance`, ids (`define_id!`, `PersistentId` v7/v5), `Aabb3/Aabb2`, `Plane`, `Ray3`, `Line3/Line2`, `Segment3/Segment2`, `Triangle3`, `intersect` module, `Error`/`Result`/`Cancelled`/`Diagnostic`/`report` | FND-02 | P0 | Done | Every Interfaces item for these modules exists with doc comments (`missing_docs` clean). Tests pass: ≥ 1 unit test per pub fn. Proptests: `ray_aabb` agrees with brute-force sampling; `ray_triangle` gives no misses for rays through shared edges/vertices of a 1000-triangle fan (watertightness); `Plane::to_world(to_local(p))` round-trips within 1e-9 mm; `Aabb3::transformed` contains all 8 transformed corners. `PersistentId::derive` is stable across runs (a fixed-vector test). `cargo clippy -D warnings` is clean. |
| FND-04 | `amg-jobs` CPU job system: two pools, `Job`/`JobCtx`/`JobHandle`/`CancelToken`/`JobInfo`, progress sub-ranges, child jobs, panic capture, notifier, UI-thread guard (no GPU) | FND-02 | P0 | Done | Tests pass. (a) 1000 jobs spawned, `cancel_all` mid-way: every handle resolves (`Cancelled` or `Succeeded`) within 100 ms and none hang. (b) A panicking job yields `JobError::Panicked` and the system keeps working. (c) Progress is monotone and nested `sub()` ranges map correctly. (d) Cancelling a parent cancels its children. (e) A nested `ctx.wait` in a 1-thread pool completes (no deadlock). (f) `try_take`/`state` never block: criterion shows < 1 µs median. (g) Spawn-to-start latency on an idle pool is < 50 µs median. (h) A Background job saturating its pool does not delay an Interactive job by more than 5 ms. |
| FND-05 | CI: `.github/workflows/ci.yml` (windows-2025; fmt, clippy, test, deny, golden jobs), rust-cache, vcpkg clone at pinned tag + `files` binary cache via `actions/cache`, empty `vcpkg.json` + `vcpkg-configuration.json` + overlay triplet, nextest `ci` profile with JUnit | FND-02 | P1 | Review | The pipeline is green on a push to `main` and on a PR. A second run on unchanged inputs has cargo cache hits, and its wall time is < 15 min. vcpkg caching is proven on a throwaway branch that adds one small port: the second run restores it from cache (log shows "Restored … from …"). The branch is not merged. Once FND-09 is done, the `test` job runs GPU tests on WARP (not skipped). |
| FND-06 | Test data infrastructure: `testdata/` with git-LFS patterns, `amg-testkit` metrics (volume, area, bbox, Euler, edges, Hausdorff with BVH), golden files + bless workflow, `testdata()` / `gpu_available()` helpers | FND-03 | P0 | Done | Tests pass. A 10 mm cube soup gives volume 1000 ± 1e-9 mm³ and area 600 ± 1e-9 mm². Sphere tessellations converge to 4/3πr³ (error halves as the triangle count quadruples). Hausdorff of two concentric spheres r=10 and r=10.5 is 0.5 ± 0.01 mm, and 1M-triangle inputs finish in < 2 s. `AMG_BLESS=1` writes a golden file and the next run passes. An un-fetched LFS pointer is skipped locally and fails with `AMG_REQUIRE_TESTDATA=1`. |
| FND-07 | Diagnostics: `diag::init` (tracing, env-filter `AMG_LOG`, rolling file logs, optional `tracy` feature), panic hook with `crash-<utc>.txt`, `RecoveryHook` registry; wired into `amg-cli` | FND-04 | P1 | Review | An integration test runs a child process that panics. Afterwards the log file contains the panic message and a backtrace, `crash-*.txt` exists with version, git SHA and the last log lines, and the registered test recovery hook ran: stderr contains `Recovery file: <path>` and the file exists. A panic inside a job does not end the process. `amg-cli info` writes to the log. |
| FND-08 | Benchmark harness: `amg_testkit::budget::BUDGETS` (one per vision budget row), criterion placeholder benches `budget/<id>`, optional `workflow_dispatch` bench job | FND-06 | P1 | Todo | `cargo bench -p amg-testkit -- --test` runs one placeholder per vision budget row (8 rows today, after the QUESTIONS #33 STL-import row). A unit test asserts `BUDGETS.len()` equals the number of rows parsed from `docs/00-vision.md`. The bench job can be triggered manually and uploads the criterion HTML report. |
| FND-09 | `amg-jobs::gpu`: `GpuConfig` (+ `AMG_WGPU_BACKEND`), `Gpu` (instance/adapter/device/queue, optional features, device-lost flag), `upload`/`storage`/`Readback`, compute-pipeline cache; `amg-cli info` prints adapter. **Gate: ADR-004 (wgpu) accepted.** | FND-04 | P0 | Review | The smoke test (add two 1M-element f32 buffers in WGSL) matches the CPU result bit-exactly on the owner's RTX 5060 Ti (DX12) and on WARP in CI. `Readback::try_get` returns `None` at least once before `Some` in a polled test, and never blocks for more than 1 ms per call. Pipeline-cache tests: the same desc gives `Arc::ptr_eq`, and an invalid WGSL gives `GpuError::Shader` with a message. Without an adapter the GPU tests skip (and fail if `AMG_REQUIRE_GPU=1`). `Gpu::new` takes < 300 ms on the owner's machine. A doc example shows building `egui_wgpu::WgpuSetup::Existing` from a `Gpu`, compiled as a doctest only if egui-wgpu is already a workspace dep, otherwise as a `text` block. |
| FND-10 | `amg-core::units` (part 2 of core): `Dimension`, `Quantity`, typed quantities, `Unit` table, `LengthUnit`, `parse_quantity` (grammar in units note §7), `format_quantity` (Decimal/Fraction/FeetInch/Exact), `ParseQuantityError` with spans | FND-03 | P0 | Done | Tests pass. Every `Unit` symbol and alias parses (table-driven test generated from `Unit::all()`). Examples: `1/4 in` = 6.35 mm, `1 1/4"` = 31.75 mm, `5' 3 1/2"` = 1612.9 mm, `30 deg` = π/6, `20 degC` = 293.15 K, `7.85 g/cm^3` = 7.85e-9 t/mm³, `-.5e1 mm` = −5 mm. Errors carry correct spans for `12,5 mm`, `3 foo`, `1/0 in` and `1 m 20 cm`. Proptest round-trip: `Exact` style round-trips bit-for-bit, and `Decimal` stays within ½ ulp of the printed precision. `Fraction` prints `1 1/4 in` for 31.75 mm. Parsing a typical literal takes < 1 µs median (criterion). |
| FND-13 | Bump vcpkg baseline to a tag shipping opencascade 8.0.1 (dev-setup §9 procedure: vcpkg.json builtin-baseline, CI clone tag, check-env pin, dev-setup table) | FND-05 | P0 | Todo | CI green with the new baseline, and `vcpkg x-history opencascade` / the port file shows 8.0.1. |
| FND-12 | Native crash capture: out-of-process minidumps via `crash-handler` + `minidumper` (access violations inside C++ deps), wired into `amg-app` and `amg-cli` | FND-07, BREP-01 | P2 | Todo | A test binary that dereferences null through FFI produces a `.dmp` in the crash dir, and the recovery hooks ran. The dump opens in WinDbg/`minidump-stackwalk` with symbolised Rust frames. |

FND-11 is reserved for analytic curve primitives in `amg-core`. Create it when S02/S03
refinement specifies them (open question 3). The number is held so later IDs stay stable.

**Decision gates:** FND-01..08 and FND-10 have no gate. FND-09 needs ADR-004 (wgpu), the same
gate as RND-01. FND-12 depends on BREP-01, so it inherits ADR-002 indirectly.

## Testing
- **Unit tests** for every public function in `amg-core`, `amg-jobs` and `amg-testkit`.
  Golden-free: these crates produce no model geometry.
- **Property tests** (`proptest`):
  - Intersection routines against brute-force or sampled references.
  - Plane local/world round-trips.
  - Unit parse ↔ format round-trips (Exact bit-for-bit; Decimal within printed precision).
  - `Quantity` arithmetic dimension algebra (`(a*b)/b` has `a.dim`).
- **Concurrency tests:**
  - Cancellation storm (1000 jobs).
  - Nested waits in a 1-thread pool.
  - Panic isolation.
  - Background/Interactive starvation check.
  - A `loom` model of `CancelToken` + progress atomics is optional and only added if a race
    is ever found.
- **GPU tests:** named `gpu_*` and put in nextest test-group `gpu` (serial). They run on WARP
  in CI (`AMG_WGPU_BACKEND=warp`, `AMG_REQUIRE_GPU=1`) and on DX12 locally. They skip
  gracefully when no adapter exists and `AMG_REQUIRE_GPU` is unset.
- **Process-level tests:** the panic hook, crash file and recovery hook are tested by
  spawning the test binary as a child (`std::process::Command` with an env switch).
- **Layering test:** `cargo metadata` edges are checked against the architecture allow-list.
- **Benchmarks** (criterion, not run in CI): `JobHandle::try_take`, spawn latency,
  `parse_quantity`, `ray_triangle` throughput, `hausdorff` on 1M triangles.

## Performance targets
Tied to the vision budgets. The foundation must spend only a small slice of them.

| Item | Target | Vision budget it protects |
|---|---|---|
| `JobHandle::try_take` / `state` / `progress` | < 1 µs median, never blocks | UI never blocks > 16 ms |
| Spawn → job starts (idle pool) | < 50 µs median | recompute < 500 ms |
| Interactive job latency while Background pool is saturated | < 5 ms added | recompute < 500 ms, UI 16 ms |
| Cancellation observed by built-in jobs | ≤ 10 ms (checkpoint convention); 1000-job storm drains < 100 ms | responsiveness |
| `JobSystem::new` | < 5 ms | cold start < 2 s |
| `Gpu::new` (instance + adapter + device) | < 300 ms on RTX 5060 Ti / DX12 | cold start < 2 s |
| `Readback::try_get` | < 1 ms per call | UI 16 ms |
| `parse_quantity` typical literal | < 1 µs | interactive dialogs |
| `ray_triangle` | ≥ 50 M tests/s single core | picking, wall-thickness (MESH-11) |
| `amg_testkit::hausdorff`, 1M + 1M triangles, 100 k samples | < 2 s | test-suite time |
| CI warm run (fmt+clippy+test+deny) | < 15 min | developer loop |

## Open questions
1. **(owner, answered)** CI image `windows-2025` (and `windows-latest`) ships VS 2026 (MSVC 14.5x)
   since June 2026. The brief pins VS Build Tools **2022** locally. Should local stay on VS
   2022, or move to VS 2026 Build Tools so local and CI compilers match? *Default if
   unanswered:* keep VS 2022 locally and `windows-2025` in CI. The two toolsets are
   binary-compatible and each environment builds all C++ itself. Revisit if a compiler
   difference causes a bug.
   **Answered 2026-10-20:** keep VS Build Tools 2022 locally and `windows-2025` in CI (recommended default).
2. **(owner, answered)** `testdata/` on GitHub LFS: the free tier is 10 GiB storage and 10 GiB/month
   bandwidth. Uploads don't count against bandwidth, and CI caches LFS objects. How big do
   you expect the STEP/STL/scan corpus to get? If it may exceed ~5 GiB, we should keep large
   corpora outside the repo (a local folder referenced by `AMG_TESTDATA_DIR`) from the start.
   **Answered 2026-10-20:** hybrid. A curated, redistributable LFS set under `testdata/` with a
   soft cap of 2 GiB total (S07 corpus ≤ 300 MB, S20 kernel corpus, golden files). Large,
   non-redistributable or private data (scans, big STEP assemblies, owner files) lives outside
   the repo under `AMG_TESTDATA_DIR`, listed in a manifest with sha256, and tests skip it when
   absent. FND-06 supports `AMG_TESTDATA_DIR` from the start.
3. Architecture lists "2D/3D primitive curves" in `amg-core`. Circles, arcs and conics are
   deferred to FND-11 until S02 (BREP) and S03 (sketch) refinement shows whether they need
   shared analytic curves or keep their own. FND-03 already covers lines, segments and
   planes.
4. **(owner, answered)** Canonical units use the coherent **mm–t–s** system (forces in N, stresses in
   MPa, mass stored in tonnes, density in t/mm³), the standard FEA "mm system". The vision
   says "SI for everything else". Pure SI with mm length is not coherent, which breaks
   expression arithmetic and FEA assembly. OK to adopt? Rationale in
   [units note §2](../notes/units-and-quantities.md). *Default if unanswered:* adopt it. Only
   mass storage differs from plain SI, and typed accessors (`Mass::kg()`) hide it.
   **Answered 2026-10-20:** adopt coherent mm–t–s canonical units (recommended default).
5. **(owner, answered)** Accept `,` as a decimal separator in input fields (locale-aware), or `.`
   only? *Default:* `.` only, with a helpful error (`DecimalComma`). The `.amg` files and
   the Python API always use `.`.
   **Answered 2026-10-20:** `.` only, with the `DecimalComma` error (recommended default).
6. ~~GitHub remote for CI?~~ Resolved: remote `origin` = `github.com/engels-hub/amalgam`,
   default branch `main` (QUESTIONS #5). FND-05 targets GitHub Actions.
7. ~~`glam` vs `nalgebra` everywhere~~ Resolved 2026-10-19 (see Refinement log):
   - **glam f64 types are the only math types in cross-crate public APIs.**
   - `nalgebra` and `faer` are allowed *inside* domain crates for dense or sparse linear
     algebra (sketch solver, FEA, fitting), and they convert at the boundary.
   - Scalar-generic code, such as autodiff/dual numbers for the sketch Jacobians, stays
     private to the domain crate.

## Refinement log

### 2026-10-19 — S00 refinement (lane E agent)
**Researched (sources):**
- **Rust releases:** 1.96.0 (2026-05-28), 1.97.0 (2026-07-09, v0 mangling default), 1.97.1
  (2026-07-16). [Rust blog releases](https://blog.rust-lang.org/releases),
  [1.97.0 notes](https://www.linuxcompatible.org/story/rust-1970-drops-legacy-symbol-mangling-and-stabilizes-cargo-warning-controls/),
  [versionlog](https://versionlog.com/rust/).
- **GitHub runner images:** `windows-2025` and `windows-latest` moved to VS 2026 in June
  2026. `windows-2022` still has VS 2022.
  [runner-images releases](https://github.com/actions/runner-images/releases),
  [Depot changelog 2026-08-22](https://depot.dev/changelog/2026-08-22-windows-runner-images.md),
  [VS 2026 rollout notice](https://www.goodfirstissue.org/vercel/ncc/issues/1309).
- **vcpkg:** registry release 2026.07.29 (2,858 ports); 2026.05.25 bumped `opencascade` to
  8.0.0. [Wikipedia: vcpkg](https://en.wikipedia.org/wiki/Vcpkg),
  [What's new in vcpkg Jul 2026](https://devblogs.microsoft.com/cppblog/whats-new-in-vcpkg-jul-2026),
  [What's new May 2026](https://devblogs.microsoft.com/cppblog/whats-new-in-vcpkg-may-2026/),
  [2026.05.25 release](https://github.com/microsoft/vcpkg/releases/tag/2026.05.25).
- **vcpkg binary caching:** the `x-gha` provider was removed in 2025.06.13 (vcpkg-tool#1662).
  The recommended replacements are NuGet/GitHub Packages or `files` + `actions/cache`.
  [What's new in vcpkg June 2025](https://devblogs.microsoft.com/cppblog/whats-new-in-vcpkg-june-2025/),
  [GHA cache tutorial (removed)](https://learn.microsoft.com/en-us/vcpkg/consume/binary-caching-github-actions-cache).
- **CMake 4.2/4.3/4.4 and Ninja 1.13.x:**
  [CMake 4.2.3](https://www.kitware.com/cmake-4-2-3-available-for-download/),
  [CMake 4.3 release notes](https://cmake.org/cmake/help/v4.3/release/index.html),
  [PyPI cmake](https://pypi.org/project/cmake/),
  [Ninja 1.13 changes](https://src.opensuse.org/pool/ninja/commits/branch/factory/ninja.changes).
- **wgpu 29.x is current** (dependents on `^29` in Aug 2026):
  [wgpu releases](https://newreleases.io/project/github/gfx-rs/wgpu/release/v26.0.2),
  [wgpu_context 0.9.0](https://docs.rs/crate/wgpu_context/0.9.0).
- **egui-wgpu `WgpuSetup::Existing` / `WgpuSetupExisting`** shares an existing
  instance/adapter/device/queue:
  [WgpuSetup docs](https://ref.rerun.io/docs/rust/stable/rerun/external/eframe/egui_wgpu/enum.WgpuSetup.html),
  [egui-wgpu 0.35](https://docs.rs/egui-wgpu/=0.35.0/).
- **glam 0.33.x is current:** [docs.rs glam 0.33.12](https://docs.rs/crate/glam/0.33.12).
- **nalgebra ↔ glam conversions** exist only per glam version (`convert-glam0xx` features),
  so a public nalgebra API would lock glam versions:
  [nalgebra glam-0.25 conversion commit](https://git.m-labs.hk/M-Labs/nalgebra/commit/980dd68e6870fb85de575eed51ccab95dadcd352).
  Ecosystem direction: Parry moved its mesh input from nalgebra to glam
  ([users.rust-lang.org thread](https://users.rust-lang.org/t/what-is-the-status-of-nalgebra-vs-glam-in-robotics/140311)).
- **GitHub LFS quotas** (10 GiB storage / 10 GiB bandwidth free):
  [GitHub docs: Git LFS billing](https://docs.github.com/en/billing/concepts/product-billing/git-lfs).
- **Algorithms:** Woop, Benthin, Wald, *Watertight Ray/Triangle Intersection*, JCGT 2(1),
  2013. Ericson, *Real-Time Collision Detection* (2005) §5.1. Arvo, "Transforming axis-aligned
  bounding boxes", Graphics Gems (1990).
- **Units:** NIST SP 811 conversion factors; `uom` crate (rejected as the primary type, see the
  note); Fusion "units in expressions" behaviour; Abaqus/CalculiX consistent-units tables.

**Decided:**
- **glam vs nalgebra (OQ 2 → 7, resolved):**
  - glam f64 aliases are the only math types in cross-crate APIs.
  - nalgebra/faer stay crate-internal.
  - Reasons: compile time, no scalar generics needed in geometry APIs, operator ergonomics,
    the GPU ecosystem uses glam, and nalgebra↔glam conversion features are pinned per glam
    version. The sketch solver's possible need for scalar-generic code is local to S03.
  - ADR-006 stays valid; a clarifying sentence is proposed in the report.
- **Toolchain pins and their reasons:** see §1, including the overlay triplet
  `x64-windows-amg` (release-only, static libs, dynamic CRT), because rustc links `/MD` even
  in debug.
- **Coherent mm–t–s–K–rad canonical units**, angle as a tracked dimension, temperature
  affine handling, single-literal grammar incl. `1/4 in`, `1 1/4 in` and feet-inch. Owner
  confirmation is requested (OQ 4). Details in the new note
  [`docs/notes/units-and-quantities.md`](../notes/units-and-quantities.md).
- **Job system:**
  - Two rayon pools (Interactive/Background) so recompute never waits behind studies.
  - Jobs consume `self` and return `Result<Output, Error>` with `Error: From<Cancelled>`, so
    `ctx.checkpoint()?` works everywhere.
  - Non-blocking `JobHandle::try_take`. A notifier callback drives egui repaints.
  - Work-stealing `ctx.wait` for child jobs, and a debug-asserted UI-thread guard.
- **GPU:**
  - `Gpu` keeps instance and adapter too (needed by egui-wgpu `Existing`).
  - DX12 by default, WARP in CI.
  - Optional (not required) `SHADER_F64`/`TIMESTAMP_QUERY`.
  - Non-blocking `Readback`.
- **CI:** explicit `windows-2025`; `files` binary cache + `actions/cache` (x-gha is gone);
  LFS cache; cargo-deny run twice for the ADR-012 permissive build; no benches in CI.
- **Lints:** `[workspace.lints]` with `unsafe_code = deny` (FFI crates opt out locally),
  `float_cmp`, `unwrap_used`/`expect_used` warn, and CI `-D warnings`. Edition 2024.
- **Release profile:** `panic = "unwind"` (job panic capture, pyo3). No
  `target-cpu=native` (bit-stability).

**Changed:**
- Status Draft → Refined. Crates row now includes `amg-testkit`.
- Design rewritten with concrete layout and config. Interfaces made concrete for `amg-core`,
  `amg-jobs` (+`gpu`, `diag`) and `amg-testkit`.
- **FND-04 split:** the GPU part moved to the new **FND-09** (gated by ADR-004). FND-04 is now
  CPU-only and ungated, matching the roadmap's "FND-01..06 have no gate".
- **FND-03 split:** units/quantities moved to the new **FND-10**.
- **New FND-12** (P2): native minidumps (deps FND-07, BREP-01).
- **FND-11** reserved for analytic curves.
- All "Done when" criteria made measurable. Added Testing and Performance targets.

**Roadmap / cross-spec follow-ups (not edited here — for the coordinator):**
- The roadmap Phase 0 should become `FND-01 → FND-02 → (FND-03, FND-04)`, then
  `FND-03 → (FND-06, FND-10)` and `FND-04 → FND-09` (FND-09 gated by ADR-004, the same gate
  as the RND-01 spike).
- FND-12 goes in Phase 2 or later (after BREP-01).
- Add FND-09 to the ADR-004 row of the decision-gate table.
- No phase-order violation for existing WPs. Dependants that need the split-out parts should
  repoint their deps:
  - RND-01, CMP-01 and MESH-11 → **FND-09**. Each is in Phase 0/1/3, and FND-09 is Phase 0.
  - DOC-04, IO-01 and MAT-01 → **FND-10** (units). All are Phase 1+.
- Until those specs are edited, FND-04 is the nominal dep and the gap is only visible to
  someone who reads this log.

**New third-party dependencies recommended** (for `THIRD_PARTY.md`):

| Crate | Licence | Purpose |
|---|---|---|
| `crossbeam-channel` | MIT / Apache-2.0 | job results |
| `parking_lot` | MIT / Apache-2.0 | progress message lock |
| `bytemuck` | Zlib / MIT / Apache-2.0 | GPU buffer casts |
| `pollster` | MIT / Apache-2.0 | `Gpu::new_blocking` |
| `tracing-appender` | MIT | rolling log files |
| `tracing-tracy` | MIT / Apache-2.0 | optional feature `tracy` |
| `clap` | MIT / Apache-2.0 | `amg-cli` |
| `approx` | Apache-2.0 | float assertions, glam `approx` feature |
| `crash-handler`, `minidumper` | MIT / Apache-2.0 | FND-12 |
| `cargo-nextest`, `cargo-deny` | MIT / Apache-2.0 | dev tools |
| `toml` | MIT / Apache-2.0 | golden files in `amg-testkit` |

Already listed: glam, slotmap, uuid, rayon, thiserror, anyhow, tracing(-subscriber), serde,
wgpu, criterion, proptest.

### 2026-10-20 — FND-02 version pins (lane E)
Pinned the newest release of each `[workspace.dependencies]` entry on 2026-10-20. Changes from
the majors listed in §2: `glam` 0.34.1 (0.34 renamed the `approx` feature to `approx-05` and
dropped `fast-math`; `bytemuck` is still an implicit feature), `wgpu` 30.0.1 (egui-wgpu 0.36 and
0.37 both require wgpu 30, so FND-09's alignment rule holds), `pollster` 1.0.1, `toml` 1.1.8,
`criterion` 0.8.2. All entries were compiled together in a scratch crate to prove the features
resolve. The layering allow-list also admits `amg-mesh -> amg-manifold-sys` (ADR-003, MESH-05).

## Interface changes
### 2026-10-20 — FND-03 (`amg-core` part 1, lane E)
- **`Error::Parse` and `Error::DimensionMismatch` are deferred to FND-10.** Their payloads
  (`ParseQuantityError`, `Dimension`) live in `units`, which FND-10 owns. `Error` is
  `#[non_exhaustive]`, so FND-10 adds both variants (and their `Diagnostic` hints) additively.
- Additive: `impl Diagnostic for Cancelled` (severity `Info`, `is_cancelled() == true`);
  `Diagnostic for Error` reports `Cancelled` as `Severity::Info` and gives hints for
  `Degenerate` and `InvalidId`.
- Clarifications of behaviour the interface left open:
  - `Plane::XZ` is u = +X, v = +Z, normal = −Y (right-handed front view); `Plane::YZ` is
    u = +Y, v = +Z, normal = +X. `Plane::flipped` keeps `x` and negates `y` and `normal`.
  - `Dir3::try_from(Vec3)` (and so deserialisation) normalises with `Tolerance::DEFAULT`; a
    vector already unit to within 4 ulp keeps its exact bits, so files round-trip bit-for-bit.
  - `ray_plane` and `ray_triangle` return `None` for hits behind the origin (`t < 0`).
  - `gpu::to_f32_slice` clears `dst` before filling it (buffer reuse).
  - `PersistentId` deliberately has no `Default` (a random id must be asked for explicitly).
  - `Aabb3::center` is NaN for an empty box; `size`/`diagonal` are zero.
  - `Seg2Intersection::Overlap { t0, t1 }` are parameters on the first segment, `t0 < t1`.

### 2026-10-20 — FND-10 (`amg-core::units`, lane E)
- **`Error::Parse` and `Error::DimensionMismatch` added** as specified, with `Diagnostic`
  hints (`Parse` delegates to the new additive `impl Diagnostic for ParseQuantityError`).
  `ParseQuantityError` also derives `Eq`.
- **New unit `Unit::TonnePerCubicMillimetre` (`t/mm^3`, scale 1)**, not in the note's table.
  Every dimension now has a canonical unit, which `FormatStyle::Exact` needs for its
  fallback (below); FEA decks use t/mm³ directly. `Unit::all()` has 49 entries.
  `FormatSpec::for_unit(TonnePerCubicMillimetre)` is `FormatSpec::EXACT` (values ~1e-9).
- **Typed accessors are `to_*`** (`p.to_mpa()`, `m.to_kg()`, `l.to_inches()`), because the
  constructors already use the bare names (`Pressure::mpa(v)`); the note's `p.mpa()` would
  collide. `Length::to_mm`/`Angle::to_rad` exist besides `get()`. Additive on every typed
  quantity: `DIMENSION`, `in_unit(Unit) -> Result<f64>`, `AddAssign`/`SubAssign`,
  `f64 * T`. Additive cross-type ops beyond the listed ones: `Length*Area`, `Area/Length`,
  `Volume/Length`, `Volume/Area`, `Pressure*Area`, `Density*Volume`, `Energy/Length`.
- Other additive items: `Dimension::new`, `Unit::scale()`, `Display` for `Unit` (symbol)
  and `Quantity` (canonical value + `Dimension`), `LengthUnit::ALL`,
  `TryFrom<Unit> for LengthUnit` (`Err(unit)`), `FormatSpec::EXACT`.
- Clarifications of behaviour the interface left open:
  - Conversion is `canonical = (v + offset) · scale` with the offset in the unit itself
    (°C +273.15, °F +459.67, scale 5/9), equal to the note's figures. Units without an
    offset multiply only, so `-0.0` keeps its sign.
  - `Dimension` exponent arithmetic saturates at the `i8` range. `Display`: named units for
    the twelve constants (`1` for dimensionless), otherwise `mm*t/s^3`, `1/s^2/K`.
  - `checked_add`/`checked_sub` report `expected = self.dim`; `in_unit`/`expect_dim` report
    the unit's/requested dimension as `expected`.
  - `Angle::normalized_0_2pi` is `[0, 2π)`, `normalized_pm_pi` is `(-π, π]`.
  - Parser: feet-inch accepts any foot/inch alias and no space after `'` (`5'3"`); its value
    is returned in inches (`5' 3"` → 63, `Unit::Inch`). Mixed numbers do not require a
    proper fraction. Spans: `ZeroDenominator` covers the fraction (`1/0`), `DecimalComma`
    the comma, `UnknownUnit` the token up to the next whitespace, `Trailing` from the first
    unexpected byte to the end, `InvalidNumber` the token including its sign.
  - `FormatStyle::Fraction` uses binary denominators (1, 2, 4, … ≤ `max_denominator`) and
    matches within the printed precision `0.5 · 10^-precision`; otherwise it prints decimals.
    `FeetInch` requires a length (any target unit), ignores `show_unit`, prints `5' 3 1/2"`,
    `5'`, `3 1/4"`, `0"`, and carries rounded inches into feet.
  - `FormatStyle::Exact` prints the shortest round-trip decimal (`{}`; exponent form `{:e}`
    outside `[1e-5, 1e16)`). If no `f64` in the target unit converts back to the identical
    canonical bits (non-power-of-two scales leave gaps; overflow), it prints the canonical
    unit instead and always shows its symbol, so the bit-for-bit contract holds for every
    unit.
  - The `Decimal` round-trip bound also allows the float rounding of the unit conversion
    (a few ulp of the value), which the note's formula leaves out.
  - `format_quantity` returns `Err(InvalidArgument)` for NaN/±inf.
- `amg-core` gains `criterion` as a dev-dependency (bench `units`: `units/parse_quantity/*`,
  `units/format_quantity/*`).

### 2026-10-20 — review fixes for FND-03 and FND-10 (`amg-core`, lane E)
- **`line_line`** now computes `s = ((w × db)·n)/|n|²`, `t = ((w × da)·n)/|n|²` with
  `n = da × db`, `w = b.point − a.point`. The old `1 − (da·db)²` denominator cancelled to 0
  for angles below about 1.5e-8 rad that still pass the `tol.angular` check, giving NaN.
- **`point_triangle_closest`**: slivers with `|ab × ac| ≤ 1e-8 · max_edge²` (height below
  about 1e-8 of the longest edge) return the closest point of the three edges. Ericson's
  region tests gave wrong points for them (up to 22.9 mm off for 10 mm slivers). The edge
  answer is within the sliver height of the exact one.
- **`segment2_segment2`** now honours its documented endpoint tolerance in the non-parallel
  case: if the lines cross outside the segments, an endpoint within `tol.linear` of the
  other segment still gives `Point` (the closest of the four endpoint checks; `t`/`u` are
  the endpoint's and its projection's parameters, `p` is on the first segment).
- **`report` does not render hints** (a `dyn Error` has none); the Interfaces comment
  "message + hint + source chain" is met by the additive
  `report_diagnostic(e: &dyn Diagnostic) -> String` (user message, `  hint: …` line, then
  the `  caused by: …` chain), re-exported at the crate root.
- **`parse_quantity` rejects overflow**: a magnitude (decimal, fraction, mixed number) or
  feet-inch total that is not finite (`1e400`, 400-digit integers) is `InvalidNumber`
  spanning the number (the whole literal for feet-inch). Underflow (`1e-400`) parses as 0.
- **Whitespace** in quantity literals is space, tab, CR, LF, NBSP (U+00A0) and narrow NBSP
  (U+202F), both around the literal and between its parts.
- **`Decimal` and `Fraction` never print `inf`**: a finite value that overflows in the target
  unit (only units with scale < 1, e.g. `f64::MAX` mm in µm) is printed as `Exact` in the
  canonical unit with its symbol, so the output always parses back. The round-trip
  proptests now cover every finite `f64`.
- **`Temperature` is affine**: it no longer has `Add`, `AddAssign`, `SubAssign`, `Neg`,
  `Mul<f64>`, `f64 * Temperature`, `Div<f64>` or `Div<Self>`. `Temperature - Temperature`
  returns the difference in kelvin as `f64`, and `Temperature::offset_k(dk)` applies one.
  Nothing in the workspace used the removed operators. All other typed quantities keep the
  full linear set.
- New criterion bench `amg-core/benches/geom.rs` (`geom/ray_triangle`, throughput in
  ray/triangle tests per second; target ≥ 50 M/s). Measured 2026-10-20: about 63 M tests/s
  on one thread on a loaded dev machine (Windows 11, MSVC).

### 2026-10-20 — FND-04 (`amg-jobs` CPU job system, lane E)
Additive:
- `JobState::is_finished(&self) -> bool`.
- `JobSystemConfig` derives `Clone` and `Default` (the derived `Default` is the spec's).
  `JobSystem`, `JobSystemConfig`, `JobCtx` and `JobHandle` have hand-written `Debug` impls
  (the workspace lint `missing_debug_implementations` applies).

Behaviour the interface left open (now documented on the items):
- `JobSystem::jobs()` lists *unfinished* non-hidden jobs, sorted by id. A job leaves the list
  before its handle reports finished, and `cancel(id)` returns `false` for finished jobs.
- Cancelling a *queued* job through `JobHandle::cancel`, `JobSystem::cancel` or `cancel_all`
  finishes it as `Cancelled` immediately; the pool skips it later. A queued child cancelled only
  through its parent's token resolves when a pool thread reaches it (it does not run).
- `JobCtx::spawn_child` on a detached context (`detached` / `with_token`) runs the child inline,
  synchronously. Such children get ids from a separate counter starting at `1 << 63`.
- `JobCtx::wait` off a pool thread blocks (condvar); on a pool thread it work-steals with
  `rayon::yield_now` and, when idle, parks for 200 µs slices. `wait` and `wait_blocking` panic if
  the result was already taken with `try_take`.
- `JobSystem::shutdown`: jobs spawned afterwards are cancelled immediately.
- Progress messages arriving faster than 30 Hz are dropped (the fraction is never dropped).
  Progress notifications are rate-limited to 30 Hz per job; state-change notifications are not.
  Panics in the notifier are swallowed.
- Background below-normal priority uses a minimal `kernel32` FFI (`SetThreadPriority`) in
  `amg-jobs/src/os.rs`; it is a no-op on non-Windows targets.
- `JobInfo::started` is set when a pool thread picks the job up (always `Some` once `Running`).
- `Diagnostic for JobError<E>`: `Failed` delegates to `E` (so a failure that is itself a
  cancellation stays `is_cancelled`); `Cancelled` is `Severity::Info` with `is_cancelled()`;
  `Panicked` is an `Error` with a generic "internal error" user message and a hint, while the
  panic text stays in `Display` / `report()` for logs.
- `.config/nextest.toml`: tests named `timing_*` run alone (`threads-required =
  "num-test-threads"`); the `ci` profile also retries them twice.

### 2026-10-20 — FND-05 deviations from §7 (lane E)
CI files: `.github/workflows/ci.yml`, the local composite action `.github/actions/vcpkg/action.yml`,
`vcpkg.json` (no deps, `builtin-baseline` = `9e593bb1…` = tag `2026.07.29`),
`vcpkg-configuration.json` (overlay `./triplets`), `triplets/x64-windows-amg.cmake`. Where the
implementation differs from §7:
- **Action versions.** §7 says `actions/cache@v4`. The current majors on 2026-10-20 are used
  instead, each pinned by commit SHA with a version comment: `actions/checkout` v7.0.1,
  `actions/cache` v6.1.0, `actions/upload-artifact` v7.0.2, `Swatinem/rust-cache` v2.9.3,
  `taiki-e/install-action` v2.87.29, `actions/download-artifact` v8.0.2. Tools:
  `cargo-nextest@0.9`, `cargo-deny@0.20`.
- **Concurrency.** `cancel-in-progress` is `true` only for `pull_request` events. Pushes to
  `main` are never cancelled, so every `main` commit gets a complete run and saves caches.
- **Toolchain step.** Every job runs `rustup toolchain install --no-self-update` (reads
  `rust-toolchain.toml`) before rust-cache, instead of relying on the first `cargo` call. It
  makes the install explicit and ensures rust-cache keys on the pinned rustc.
- **vcpkg steps** live in one composite action (`.github/actions/vcpkg`) used by `clippy`,
  `test` and `bench`. `fmt`, `deny` and `golden` don't need it (`golden` runs prebuilt,
  statically linked binaries). The action fails fast if the cloned tag's commit differs from
  `builtin-baseline`. It sets `VCPKG_DEFAULT_TRIPLET` and `VCPKG_DEFAULT_HOST_TRIPLET` to
  `x64-windows-amg` (host tools are built once, release-only), and installs with
  `--x-buildtrees-root=<runner.temp>b` to keep OCCT build paths under MAX_PATH.
  BREP-01: if OCCT needs runtime resource files from `vcpkg_installed/`, add the action back to
  `golden`.
- **vcpkg cache key** is `vcpkg-<triplet>-<ImageVersion>-<hash(vcpkg.json, vcpkg-configuration.json,
  triplets/**)>`, with restore keys `vcpkg-<triplet>-<ImageVersion>-`, `vcpkg-<triplet>-` and
  `vcpkg-`. `ImageVersion` is a runner machine variable, not part of the `env` expression
  context, so a step reads `$env:ImageVersion` and passes it to the key as a step output. With the §7 key a runner-image compiler update would hit the exact key, so the
  rebuilt binaries would never be saved. Known limitation: the `files` directory grows with
  stale ABI entries until the key prefix changes.
- **golden LFS.** It checks out with `lfs: false`, writes `git lfs ls-files --long` object ids to
  `.lfs-assets-id`, restores `.git/lfs` from `actions/cache` keyed by that file's hash, and then
  runs `git lfs pull`. `lfs: true` on checkout would download everything before the cache is
  restored, so the cache would never save bandwidth. The job also sets
  `AMG_REQUIRE_TESTDATA=1` and runs nextest with `--no-tests=pass` (there are no `golden_`
  tests yet).
- **golden doesn't rebuild.** `test` runs `cargo nextest archive --profile ci` and uploads the
  archive (artifact `nextest-archive`, kept 1 day). `golden` downloads it and runs
  `cargo nextest run --archive-file … --workspace-remap . --extract-to <runner.temp>
extest
  --extract-overwrite -E 'test(golden_)'` on the LFS checkout. `test` runs
  `-E 'not test(golden_)'`, so golden tests run once, with their data. The archive round-trip
  and the remap were checked locally on Windows with nextest 0.9.148.
- **clippy** runs twice: default features and `--all-features`, so feature-gated code is linted.
- **WARP.** The local action `.github/actions/install-warp` downloads the
  `Microsoft.Direct3D.WARP` NuGet package, pinned to version 1.0.20 with its sha256 (the
  version wgpu's CI uses; 1.0.21 also exists). It copies `d3d10warp.dll` into
  `target/debug` and `target/debug/deps` (in `golden`, the extract tree). The loader searches
  the exe's directory before System32, so this DLL is used instead of the image's inbox copy,
  and `AMG_REQUIRE_GPU=1` doesn't depend on what the runner image ships. This is the same
  approach as wgpu's xtask `install-warp`. The DLL is CI-only and never shipped.
- **bench job** runs only on `workflow_dispatch` with the input `bench: true`, so a manual
  dispatch for an ordinary CI run doesn't also run benches. FND-08 adds the benches it runs.

### 2026-10-20 — FND-06 (`amg-testkit`, lane E)
- **`amg-testkit` depends on `amg-core`** (the interface's `TriSoup` borrows
  `amg_core::Point3` and `MeshMetrics` holds an `amg_core::Aabb3`). The FND-02 layering
  allow-list had `amg-testkit -> {}`; it is now `amg-testkit -> {amg-core}`. It still depends
  on no other workspace crate. Consequence for `amg-core` only: its *unit* tests
  (`#[cfg(test)]` inside the lib) cannot pass their `amg_core` types to testkit functions,
  because the dev-dependency cycle links a second copy of `amg-core` and the types do not
  unify (a compile error, not a silent bug). `amg-core` integration tests (`tests/*.rs`) see
  the same single copy as the testkit and may use it. The layering test therefore keeps
  allowing the `amg-core -> amg-testkit` dev edge.
- **Third-party deps:** `rayon` (parallel BVH build and queries), `toml` (golden files),
  `wgpu` + `pollster` (`gpu_available` probes one adapter per process; `AMG_WGPU_BACKEND`
  `dx12|vulkan|warp` is honoured, `warp` = DX12 fallback adapter), `criterion` (dev).
  `wgpu`/`pollster` sit behind the default-on cargo feature **`gpu`**; a crate that uses
  `amg-testkit` with `default-features = false` gets a `gpu_available()` that always skips
  (and panics under `AMG_REQUIRE_GPU=1`, since its GPU tests cannot run).
  `wgpu` is the workspace pin FND-09 also uses; it brings `cargo deny` duplicate *warnings*
  for `hashbrown` and `syn` (not errors).
- Additive API:
  - `TriSoup::corners(t)`; `one_sided_hausdorff(from, to, samples)`.
  - `shapes::{TriMesh, cube, icosphere}` (also `amg_testkit::TriMesh`): reference meshes for
    tests. `icosphere(c, r, k)` has `20·4^k` triangles, so each step quadruples the count.
  - `testdata_dir()` and `external_testdata(rel)`: resolves `$AMG_TESTDATA_DIR/<rel>` and
    skips (never fails, CI has no such corpus) when the variable is unset or the file is
    absent. `testdata/external.toml` is the manifest format (path, sha256, source).
  - `GoldenTolerances: Default` (1e-6 rel volume/area, 1e-6 mm bbox, no Hausdorff).
- Clarifications of behaviour the interface left open:
  - `MeshMetrics::vertices` counts distinct *referenced* indices; edges/Euler use indices as
    given (callers weld first if their mesh does not share vertices). Self-loop edges of
    degenerate triangles (`[a, a, b]`) are skipped, so they never count as boundary edges.
    `MeshMetrics` rejects unknown fields when deserialised.
  - `signed_volume` sums about the bbox centre to avoid cancellation far from the origin.
  - `hausdorff` queries every referenced vertex plus `samples` area-weighted points per
    direction (vertices catch corners and spikes; `samples == 0` means vertices only).
    Sample `i` comes from a SplitMix64 stream seeded with `SEED + i`; BVH queries stop early
    once a point cannot raise the running max, which keeps the max exact, so the result is
    bit-identical regardless of thread count. A direction *from* a soup without triangles
    contributes 0, a direction *to* one is `+inf`: both empty gives 0, exactly one empty
    `+inf`. A zero-area soup is measured by its vertices only.
  - `testdata(rel)` / `external_testdata(rel)` panic if `rel` is absolute or has `.`/`..`
    components; `testdata` also panics if the file does not exist (a typo is a bug, not a
    skip).
  - Golden files: unknown keys (in the file, `[tolerances]` or `[metrics]`) are errors, so a
    typo such as `hausdorff_ab` or `[tolerance]` fails instead of disabling a check. Bless
    keeps the existing `[tolerances]` table (hand-tuned once); it is read on its own, so a
    `[metrics]` schema change cannot drop it. Bless refuses (panics) if the file is not TOML,
    has unknown top-level keys or an invalid `[tolerances]`, and whenever the `CI` variable
    is set. When `hausdorff_abs` is set, bless also writes `<case>.stl` (binary STL, f32,
    LFS-tracked) next to the TOML, and deletes a stale one when it is not; the check computes
    the Hausdorff distance to it with 20 k samples, and an un-fetched reference skips that
    check locally and fails with `AMG_REQUIRE_TESTDATA=1`. Counts are compared exactly.
    Volume and area pass when `|a - e| <= rel * max(|e|, scale)` with `scale = d³` (volume)
    or `d²` (area), `d` the expected bbox diagonal, so near-zero values (open or flat soups)
    do not turn round-off into a huge relative error. Golden tests should be named
    `golden_*` for CI's `golden` job filter.
  - LFS patterns in `.gitattributes` are repo-wide (`*.stl *.3mf *.step *.stp *.obj *.ply
    *.amg`, plus upper-case `STL/STEP/STP`), so golden reference meshes are covered too.
- Measured: `hausdorff` on two 1 310 720-triangle icospheres, 100 k samples each, BVH builds
  included (vertex queries too): **~0.87 s** median (criterion, bench profile,
  `benches/hausdorff.rs`), target < 2 s.
- Leftovers for MESH-01 integration: `TriMesh::from_flat` (flat `[f64]` / `[u32]` buffers),
  `TriMesh::from_f32` (GPU/STL-style f32 positions) and a `welded(tol)` helper so unwelded
  soups give meaningful topology metrics. Not done in FND-06.

### 2026-10-20 — FND-07 (`amg-jobs::diag`, lane E)
Files: `crates/amg-jobs/src/diag/{mod,hook,ring,time}.rs`, `crates/amg-jobs/tests/diag.rs`,
`crates/amg-cli/{src/main.rs,tests/cli.rs}`.

Additive:
- **`DiagConfig` gains `version: &'static str` and `git_sha: &'static str`** (written into
  crash reports). The binary passes them (`amg-cli` uses `CARGO_PKG_VERSION` and its build.rs
  `AMG_GIT_SHA`), so `amg-jobs` has no build script. `DiagConfig: Default` (`amalgam`, this
  crate's version, SHA `unknown`, default dir, filter `info`, stderr on),
  `DiagConfig::resolved_log_dir()` and `default_log_dir()`.
- **Log dir override:** env `AMG_LOG_DIR` (`diag::LOG_DIR_ENV`) beats `DiagConfig::log_dir`,
  which beats the default `%LOCALAPPDATA%\Amalgam\logs` (non-Windows: `$XDG_STATE_HOME/amalgam/logs`,
  `~/.local/state/amalgam/logs`, temp dir). Crash reports go to the same directory.
  Constants `LOG_ENV` (`AMG_LOG`), `LOG_DIR_ENV`, `MAX_LOG_FILES` (7), `RING_LINES` (200),
  `HOOK_TIMEOUT` (10 s).
- `DiagGuard::log_dir()`. `impl Diagnostic for DiagError` (hint for `LogDir`).
- **Crash-info providers** (the pluggable "adapter info"):
  `pub type CrashInfoProvider = Box<dyn Fn() -> String + Send + Sync>;`
  `pub fn register_crash_info(name: &'static str, provider: CrashInfoProvider);`
  Each becomes a `name: text` line in the report's `[crash info]` section. It is a registry,
  not a `DiagConfig` field, because the GPU adapter exists only after `diag::init`. The report
  always has an `adapter:` line, `unknown` until FND-09 / the app registers a provider named
  `"adapter"`.
- `pub fn unregister_recovery_hook(name) -> bool` (a closed document drops its autosave hook);
  registering an existing name replaces the hook.
- **`pub fn catch_panic<R>(f: impl FnOnce() -> R + UnwindSafe) -> std::thread::Result<R>`**:
  `catch_unwind` that marks the panic as handled. The job system now runs jobs and the notifier
  through it (one-line changes in `system.rs` / `core.rs`). The hook logs a caught panic
  (`caught panic in thread … at …: msg` + backtrace) and does nothing else; any other panic is
  a crash (log, recovery hooks, crash report).
- `amg-jobs` feature `tracy` (`tracing-tracy` 0.12.0 with Tracy's own `crash-handler` feature
  off); `amg-cli` feature `tracy` forwards to it.

Behaviour the interface left open:
- `init` also calls `install_panic_hook` (idempotent). A second `init`, or one after another
  global subscriber / `log` logger, is `AlreadyInitialised`; a `LogDir` error installs nothing.
- Filter: `AMG_LOG` if set and valid, else `default_filter` (parsed lossily, base level
  `info`); an invalid `AMG_LOG` is logged as a warning. One filter applies to all layers.
- File layer: `tracing-appender` daily rotation, `<app_name>.<yyyy-mm-dd>.log`, 7 files kept,
  non-blocking writer in **non-lossy** mode (a burst blocks the logger instead of dropping
  lines), thread names on. Dropping the guard flushes it (also while a panic unwinds `main`);
  a `panic = "abort"` or `process::exit` path may lose the last buffered file lines, but not
  the crash report, which is written synchronously from the in-memory ring.
- Ring layer: a second `fmt` layer into a process-wide ring of the last 200 *lines*
  (multi-line events count per line). The crash report takes its snapshot before the panic
  itself is logged, so the backtrace does not push context out.
- Crash path order: log the panic + `Backtrace::force_capture()`, run the recovery hooks
  (stderr `Recovery file: <path>` for each `Some`), write `crash-<yyyymmdd>T<hhmmss>.<ms>Z.txt`
  (`-<n>` suffix on a clash; a header with app, version, SHA, UTC time, OS, thread, location and
  message, then `[crash info]`, `[recovery]`, `[backtrace]`, `[last N log lines]`), then print
  `Crash report: <path>`. Recovery hooks and crash-info providers each run on a helper thread
  with a 10 s timeout: a panic *inside* the panic hook would abort the process, and a hook that
  needs a lock the panicking thread holds must not hang it forever. The report records them as
  `saved <path>`, `nothing to save`, `panicked` or `timed out`.
- Before `init`, the hook still calls the previous hook (default stderr message) and writes
  crash reports to the resolved default directory. After `init` the previous hook is not
  called; with `stderr: false` a one-line `<app> panicked …` goes to stderr instead.
- `amg-cli` logs to the file always and to stderr only when `AMG_LOG` is set. `info` logs an
  `amg-cli info` event and prints the log dir. A failing `init` prints a warning and continues.
- Test: `tests/diag.rs` re-runs its own binary (`diag_child --exact`, env switch
  `AMG_DIAG_TEST_CHILD`, `AMG_LOG_DIR` = a temp dir). The child survives a job panic (no crash
  file), then panics; the parent checks the log (both panics, a backtrace with the child's
  frame), exactly one crash report (version, SHA, adapter, recovery, last lines) and the
  recovery file. `crates/amg-cli/tests/cli.rs` checks that `info` writes to the log.

Leftovers: FND-09 registers the `"adapter"` crash-info provider and adds the adapter to
`amg-cli info`; S01 autosave registers its recovery hook; native crashes stay FND-12.

### 2026-10-20 — FND-09 (`amg-jobs::gpu`, lane E)
- Additive on `GpuConfig`: `dx12_compiler` (default `Dx12Compiler::Fxc`; wgpu's own default
  `Auto` depends on `PATH`), `instance_flags` (default `InstanceFlags::from_env_or_default()`),
  `try_from_env() -> Result<_, UnknownBackend>` and `with_backend(BackendSelection)`.
  `GpuConfig::from_env` panics on an unknown `AMG_WGPU_BACKEND` value.
- New public items: `gpu::BackendSelection` (`platform_default`, `parse`, `from_env`),
  `gpu::UnknownBackend`, `WGPU_BACKEND_ENV`, `ComputePipelineDesc::new` (const), and
  `pub use wgpu` from `amg_jobs::gpu`.
- `GpuConfig::required_limits` is a floor: the device gets the adapter's limits if they meet
  it, otherwise the floor itself, so wgpu reports the missing limit.
- The `AMG_WGPU_BACKEND` parser lives only in `crates/amg-jobs/src/gpu/backend.rs`
  (wgpu + std only). `amg-testkit` compiles the same file via `#[path]`, so both agree. The
  testkit default on Windows is now DX12 (was PRIMARY) to match `GpuConfig`.
- `Gpu::readback` after device loss returns a `Readback` that yields `GpuError::DeviceLost`
  without calling into wgpu. `upload`/`storage` after loss still reach wgpu.
- The uncaptured-error handler is left at wgpu's default (panic). The app (lane D) may
  install its own.
- Wave-2 requests done: `dx12_compiler`, OR-able `optional_features`, the egui 0.37
  `WgpuSetupExisting` doc block (`text`, egui-wgpu is not yet a workspace dep).
- FYI for S10: `SHADER_F64` is not available on DX12 (RTX 5060 Ti, driver 32.0.16.1714).
