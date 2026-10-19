# S17 — Materials & appearances

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 (basic) / P1 (full) |
| **Crates** | `amg-materials`, `amg-ui::materials` (material browser) |
| **Lane** | G |
| **Depends on** | S00 |
| **Parity** | Physical material, appearance, material library |

## Summary
Physical materials (density, elastic, thermal, strength — incl. printing materials such as
PLA, PETG, ABS, ASA, PA12, PA-CF, TPU, standard resins, Ti6Al4V, AlSi10Mg, 316L) and
appearances (PBR parameters, colors) in user-editable libraries; assignment per body/
component; used by properties (S05), FEA (S13), generative (S14), 3MF colors (S12), rendering.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| MAT-01 | Physical material model + built-in library (TOML) incl. AM materials, user library | FND-03 | P0 | Todo | values sourced/cited |
| MAT-02 | Appearance model (PBR) + library, per-face overrides | FND-03 | P1 | Todo | |
| MAT-03 | Material browser UI, assign/drag-drop | UI-01 | P1 | Todo | |
| MAT-04 | Anisotropic/orthotropic printed-material models (layer direction) | MAT-01 | P2 | Todo | consumed by FEM |
