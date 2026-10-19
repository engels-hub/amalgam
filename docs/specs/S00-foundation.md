# S00 — Foundation, build & tooling

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 |
| **Crates** | workspace root, `amg-core`, `amg-jobs`, `amg-cli` (`amg-app` belongs to lane D) |
| **Lane** | E |
| **Depends on** | — |
| **Parity** | — (enabler) |

## Summary
Cargo workspace, toolchain, CI, the shared foundation types every crate uses (math, units,
tolerances, ids, errors), and the job system (CPU pool + shared GPU device, progress, cancel).
Nothing else can start in parallel until FND-01..FND-04 are done, so this is kept small.

## Scope
**In:** workspace layout, toolchain pinning, lint/format config, CI, C++ dependency build story
(vcpkg), `amg-core`, `amg-jobs`, empty binary crates, logging/tracing, crash reporting, test data setup.
**Out:** any domain geometry (lives in domain crates).

## Design

### Toolchain (owner's machine currently has no Rust installed)
- `rustup` stable, pinned in `rust-toolchain.toml`; MSVC target `x86_64-pc-windows-msvc`.
- Visual Studio Build Tools 2022 (C++ workload), CMake, Ninja, vcpkg (manifest mode, `vcpkg.json`)
  for OCCT, Manifold, fTetWild. `docs/dev-setup.md` is a deliverable of FND-01 (does not exist yet).
- `cargo-nextest` for tests, `criterion` for benches, `cargo-deny` for licence checks (ADR-012).

### `amg-core`
- Re-export `glam` f64 types: `Point3 = DVec3`, `Vec3`, `Transform` (`DAffine3`), `Point2`, `Mat3`, `Quat`.
- `Tolerance { linear: 1e-6 mm, angular: 1e-9 rad, ... }` passed explicitly, never global.
- Units: `Length`, `Angle`, `Mass`, `Force`, `Pressure`, `Temperature`… newtypes around f64
  in SI-internal-mm convention; `Unit` enum + parser/formatter (`"12.5 mm"`, `"1/4 in"`, `"30 deg"`).
  Used by parameters (S01) and UI.
- Ids: `slotmap`-style generational keys; `Uuid` (v7) for persistent ids in files.
- `Aabb3`, `Plane`, `Ray`, `Line3`, `Segment`, basic intersection tests.
- `Error` via `thiserror`; `Result<T>` alias; errors carry user-facing message + debug context.

### `amg-jobs`
- `JobSystem` over `rayon`; `Job` trait with `run(&self, ctx: &JobCtx)`; `JobCtx` gives
  `progress(f32, &str)`, `is_cancelled()`, child jobs.
- `JobHandle<T>` pollable from UI thread (no blocking), result via channel.
- `Gpu` — owns `wgpu::Instance/Adapter/Device/Queue`, shared by render and compute; helpers for
  buffer upload/readback and compute pipeline caching.
- Tracing (`tracing` + `tracing-subscriber`, optional `tracy` feature for profiling).

### Binaries
- `amg-app`: starts egui shell (S11). `amg-cli`: headless; subcommands `recompute`, `export`,
  `study run` (filled in by later specs).

## Interfaces
```rust
pub struct JobSystem; impl JobSystem { pub fn spawn<J: Job>(&self, job: J) -> JobHandle<J::Output>; }
pub trait Job: Send + 'static { type Output: Send; fn run(self, ctx: &JobCtx) -> Result<Self::Output>; }
pub struct Gpu { pub device: wgpu::Device, pub queue: wgpu::Queue, /* … */ }
```

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| FND-01 | Dev setup doc + toolchain install (rustup, VS Build Tools, CMake, vcpkg) | — | P0 | Todo | `cargo build` works on owner's machine; `docs/dev-setup.md` written |
| FND-02 | Cargo workspace with all crates from architecture as empty libs, lints, fmt, `rust-toolchain.toml` | FND-01 | P0 | Todo | `cargo build --workspace` and `cargo clippy -D warnings` pass |
| FND-03 | `amg-core`: math aliases, units + parser, tolerance, ids, errors, aabb/plane/ray | FND-02 | P0 | Todo | unit tests incl. unit parsing round-trips |
| FND-04 | `amg-jobs`: job system, progress, cancel, shared `Gpu` | FND-02 | P0 | Todo | test: 1000 jobs, cancel mid-way; GPU compute smoke test (add two buffers) |
| FND-05 | CI (GitHub Actions, windows-latest): build, clippy, nextest, cargo-deny; cache vcpkg | FND-02 | P1 | Todo | green pipeline on push |
| FND-06 | Test data infrastructure: create `testdata/` with git-LFS, golden-test helper crate (`amg-testkit`) | FND-03 | P0 | Todo | helper asserts volume/area/bbox/Hausdorff with tolerances |
| FND-07 | Logging, panic hook, crash dump + autosave hook point | FND-04 | P1 | Todo | panic writes log + recovery file path |
| FND-08 | Benchmark harness (`criterion`) wired for vision budgets | FND-06 | P1 | Todo | `cargo bench` runs a placeholder per budget |

## Testing
Unit tests for all core types; GPU smoke test skipped gracefully if no adapter.

## Open questions
1. ~~GitHub remote for CI?~~ Resolved: remote `origin` = `github.com/engels-hub/amalgam`, default branch `main` (QUESTIONS #5). FND-05 can target GitHub Actions.
2. `glam` vs `nalgebra` everywhere — ADR-006 proposes glam for geometry; refine if a domain needs generics.
