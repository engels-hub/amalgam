# S16 — Additive manufacturing prep

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 (overhang analysis, 3MF hand-off) / P1 (orientation, arrange) / P2 (supports) |
| **Crates** | `amg-am`, `amg-ui::am` (Manufacture (AM) workspace) |
| **Lane** | G |
| **Depends on** | S07, S08, S12, S10 |
| **Parity** | Additive section (Fusion Manufacturing Extension) |

## Summary
Everything between "design done" and "slicer": printer/machine profiles, build volume,
overhang and support-area analysis, automatic orientation, plate arrangement/nesting,
printability checks (thin walls, small features, trapped volumes for resin/powder), support
preview for SLA/MPBF, and a clean hand-off to OrcaSlicer/PrusaSlicer.

## Design
- **Machine library:** build volume, technology (FFF, SLA/MSLA, SLS, MPBF), nozzle/layer
  defaults, min feature size; import printer list from Orca/Prusa profile folders if present.
- **Print setup** (stored in document): machine, which bodies/components, per-object
  orientation & placement, plate(s).
- **Overhang analysis:** per-face angle vs build direction, threshold, highlights downward
  facing regions + support area estimate; live while rotating.
- **Orientation optimization:** sample orientations on a sphere (geodesic + local refine),
  score = weighted(support area, support volume, build height, contact area on plate,
  cosmetic faces penalty user-selected, stair-stepping on curved faces); GPU-parallel.
- **Arrange:** 2D nesting (polygon footprints, NFP or raster-based) for FFF; 3D packing for SLS.
- **Printability checks:** wall thickness (MESH-11), small features, islands per layer (sliced
  preview), trapped volumes (resin cups/powder), max bridge length.
- **Supports (P2):** tree supports (SLA/FFF preview), bar/volume supports for MPBF; mainly to
  evaluate orientation, since the slicer generates final supports for FFF.
- **Hand-off:** export 3MF with plate layout (IO-03/IO-04); "Send to slicer" button launches
  the configured slicer executable with the file.
- **Process simulation (P3):** metal PBF distortion via inherent strain on voxel model (S13/S15).

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| AM-01 | Machine library + print setup model | DOC-07 | P0 | Todo | |
| AM-02 | Overhang/support-area analysis overlay, live with orientation | MESH-02, RND-08 | P0 | Todo | |
| AM-03 | Build volume display, plate, place on plate (face-down tool) | RND-03 | P0 | Todo | |
| AM-04 | Orientation optimization (GPU scoring) | AM-02, CMP-01 | P1 | Todo | beats manual orientation on test set |
| AM-05 | Arrange: 2D nesting + 3D packing | AM-03 | P1 | Todo | |
| AM-06 | Slicer hand-off (3MF plates, launch Orca/Prusa) | IO-03 | P0 | Todo | one-click open in Orca |
| AM-07 | Printability checks (thin walls, small features, trapped volumes, layer islands) | MESH-11 | P1 | Todo | |
| AM-08 | Support generation preview (tree / bar / volume) | AM-02 | P2 | Todo | |
| AM-09 | Metal PBF distortion simulation (inherent strain) | FEM-04, CMP-05 | P3 | Todo | |

## Open questions
1. Owner: which printers/technologies do you use (FFF only? resin?) and which slicer — affects AM-06/AM-08 priority.
   **Answered 2026-10-20:** FFF on a Creality Ender 3 V3 KE with an MMU mod (multi-material)
   and an MSLA resin printer. Slicers: OrcaSlicer (FFF) and Chitubox (resin). AM-06 targets
   OrcaSlicer (multi-material 3MF plates) and Chitubox (STL/3MF + launch). Consequences for the
   next refinement: multi-material 3MF export (per-body/per-face filament) is a must-have for
   IO-03; resin checks in AM-07 (cups, trapped volumes, drain holes) and the resin support
   preview in AM-08 gain priority (AM-08 P2 → P1 proposed).
