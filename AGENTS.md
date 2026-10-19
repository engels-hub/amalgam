# Working on Amalgam (for agents and humans)

Amalgam is a Rust, local-first, Fusion-style CAD/CAE app focused on additive manufacturing.
Read these first, in this order:
1. `docs/00-vision.md` — goals, non-goals, performance budgets
2. `docs/01-architecture.md` — crates, layering, data model, threading
3. `docs/02-decisions.md` — ADRs (don't contradict an Accepted ADR without proposing a new one)
4. `docs/04-roadmap.md` — phases and lanes
5. The spec(s) you are working on in `docs/specs/`

## Two kinds of tasks

### A. Refining a spec
Goal: turn a `Draft` spec into `Ready` work packages that an implementing agent can do
without guessing.
1. Read the spec + architecture. Research libraries, papers and prior art it mentions.
2. Resolve open questions you can resolve with evidence; record reasoning in the spec's
   **Refinement log** (dated). Questions only the owner can answer go to `docs/QUESTIONS.md`
   (append, with spec ID) — don't block on them unless they truly block.
3. Make the **Interfaces** section concrete (real Rust signatures for what other crates use).
4. Split or merge WPs so each is roughly 1–3 days of focused agent work, has explicit deps,
   and a measurable **Done when**. Keep IDs stable; new WPs get the next free number.
5. Larger design questions (persistent naming, solver choice) get a design note in
   `docs/notes/<topic>.md`, linked from the spec.
6. Set spec status to `Refined`; the owner flips it to `Ready` (or tells you to).

### B. Implementing a work package
1. Pick a WP whose status is `Ready`/`Todo` in a `Ready` spec, whose deps are all `Done`,
   which is in your lane, and whose decision gate (roadmap §Decision gates) is cleared. Set it to `In progress (<agent/branch>)` in the spec table.
2. Work on a branch `wp/<ID>-short-name` off `main` (or a worktree). One WP per branch/PR.
3. **Only touch crates and modules your lane owns** (roadmap §Lanes). `amg-ui` and
   `amg-features` are shared crates split by module: lane D owns the `amg-ui` framework, and
   each domain lane owns its own `amg-ui::<module>` and `amg-features::<module>`. If you need a
   change in another lane's code or in a shared API (`amg-core`, `amg-jobs`, `amg-doc`, the
   `amg-ui` framework), write it down under **Interface changes** in the owning spec and keep
   the change minimal and additive; flag it in your PR description.
4. Tests are part of the WP: unit tests, golden tests in `tests/golden/` where geometry is
   produced, benchmarks for anything with a budget in the vision doc.
5. Before finishing: `cargo fmt`, `cargo clippy --workspace -- -D warnings`,
   `cargo nextest run -p <your crates>` all pass. Set the WP to `Review`, summarize what was
   done and anything left over (new WPs if needed).

## Conventions
- Rust stable, edition 2021+, `#![deny(unsafe_op_in_unsafe_fn)]`; `unsafe` only in `-sys` crates
  and documented GPU/FFI boundaries.
- Errors: `thiserror` in libraries, `anyhow` only in binaries/tests.
- Geometry in f64 (`amg-core` aliases), millimetres, radians. f32 only on the GPU.
- Never block the UI thread; long work goes through `amg-jobs` with progress + cancel.
- No OCCT types outside `amg-brep`; no `amg-doc` types inside `amg-render`.
- New third-party dependency → check licence (ADR-012), list in `THIRD_PARTY.md`.
- Commit messages: `<WP-ID>: <summary>`.
- **Git authorship (strict):** all git actions (commits, merges, tags, PRs, comments) are made
  under the owner's own git identity only. Never add AI attribution: no `Co-Authored-By:` AI
  trailers, no "Generated with …" footers, no AI mentions in commit messages, PRs or branch names.
