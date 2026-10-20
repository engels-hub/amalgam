# S18 — Scripting API, add-ins & CLI

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P1 |
| **Crates** | `amg-script`, `amg-cli` |
| **Lane** | E |
| **Depends on** | S01 (commands), feature specs |
| **Parity** | Fusion API (Python), scripts & add-ins, text commands |

## Summary
Python API (pyo3, ADR-011) mirroring the command set: open/create designs, create sketches
and features, set parameters, run studies, export. Embedded interpreter in the app (script
manager + console) and a pip-installable module for headless use. CLI for batch tasks.
Also the main vehicle for parity regression tests (rebuild reference models by script).

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| SCR-01 | pyo3 module: document, parameters, bodies, export | DOC-05 | P1 | Todo | `import amalgam` works headless |
| SCR-02 | Auto-generated bindings for all registered features from dialog schema | SOL-01 | P1 | Todo | every feature scriptable without hand-written glue |
| SCR-03 | Embedded interpreter, script manager, console panel | UI-01 | P1 | Todo | |
| SCR-04 | Add-ins: custom commands/toolbar buttons from Python | SCR-03, UI-02 | P2 | Todo | |
| SCR-05 | CLI: recompute, export, run study, batch convert | DOC-07 | P1 | Todo | |
| SCR-06 | Type stubs (`.pyi`) + docs generation | SCR-02 | P2 | Todo | |

## Open questions
1. (from S05, 2026-10-20) Python/CLI wrappers `measure(a, b)`, `physical_properties(body)`, `section_stats(body, plane)`; suggested CLI `amg-cli props <file>`.
2. (from S11, 2026-10-20) SCR-03 and SCR-04 use `UiRegistry::register_dynamic` (`script.*` ids) and `PanelDef`.
3. (from S20, 2026-10-20) Thin `amg-cli` subcommands `kernel replay <dir>… [--robust] [--json]`, `kernel promote <dump> --corpus <dir> --id <name>` and `kernel harness --corpus … --strategy … --jobs N --report out.json`, each calling an `amg_brep::capture` / `robust::corpus` function; until then S20 ships them as `amg-brep` examples.
