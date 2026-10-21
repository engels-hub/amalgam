# Sketch solver corpus (SK-14)

Cases the sketch solver (SK-03), the ADR-007 benchmark (SK-13) and the DOF analysis (SK-04)
are measured against. Test data only: no crate compiles or links anything in this directory.

| Folder | Cases | Source | Licence |
|---|---|---|---|
| `solvespace/` | 61 | SolveSpace `test/constraint/*.slvs` (2D), unmodified | GPL-3.0-or-later (data, test-only) |
| `planegcs/` | 23 | scenarios after FreeCAD Sketcher tests, hand-ported JSON | LGPL-2.1-or-later |
| `ezpz/` | 27 | ezpz `test_cases/*/problem.md`, unmodified | MIT |
| `own/` | 24 | decision cases D1-D10 and hand-checkable cases | project |

Each folder has a `NOTICE` (source, commit, copyright, licence) and, where the data is third
party, the licence text.

## Formats

- `*.json` (`own/`, `planegcs/`): one `Case` (`amg_sketch::corpus::Case`, format 1):
  entities with start positions, constraints (entity references are indices into `entities`),
  and `expected` (`status`, `dof`, optional `solution`, `tolerance`, `groups`).
- `*.slvs`, `problem.md` (`solvespace/`, `ezpz/`): imported at load time by
  `amg_sketch::corpus::{slvs, ezpz}`. Their expectations are in the folder's `expected.json`
  (one entry per file; every file must be listed exactly once).

`amg_sketch::corpus::load_corpus(root)` loads everything; `runner::run_corpus` runs a
`CorpusBackend` over it and prints PASS/FAIL/SKIP per case.

## Expectations

- `status`: `converged`, `converged_redundant`, `inconsistent` (or `did_not_converge`, which an
  unsolvable case may also accept).
- `dof`: free parameters minus the rank of the constraint Jacobian at the solution.
- `solution`: `start` (the file is saved solved; a solver must keep it), `target` (the root a
  fully constrained case must reach from its start), `witness` (some valid configuration of an
  under-constrained case, only used to validate the corpus).

## Regenerating

`AMG_BLESS=1 cargo nextest run -p amg-sketch --test corpus_gen` rewrites the `own/` and
`planegcs/` JSON and both `expected.json` sidecars from `crates/amg-sketch/tests/corpus_gen.rs`.
Without `AMG_BLESS` the same tests fail if a checked-in file is stale.
