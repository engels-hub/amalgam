# S12 — Import / export

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 |
| **Crates** | `amg-io`, `amg-ui::io` (import/export dialogs) |
| **Lane** | G |
| **Depends on** | S02 (STEP/IGES via OCCT), S07 (meshes), S03 (DXF/SVG), S17 (materials/colors) |
| **Parity** | Data section |

## Summary
Reliable interchange: STEP/IGES for B-rep (with assembly structure, names, colors),
STL/3MF/OBJ/PLY for meshes, DXF/SVG for 2D. **3MF is the primary AM export** and must
support multiple objects, colors, materials, build items/transforms, units, and the
OrcaSlicer/PrusaSlicer project extensions where practical.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| IO-01 | Format registry, import/export dialogs, unit handling, drag-drop | FND-03 | P0 | Todo | |
| IO-02 | STL read (ascii/binary, huge files streaming) + write | MESH-01 | P0 | Todo | 1 GB STL loads < 5 s |
| IO-03 | 3MF read/write: core spec, materials & colors, multiple objects, build items, thumbnails | MESH-01, MAT-01 | P0 | Todo | validates with 3MF consortium tooling; opens in Orca/Prusa |
| IO-04 | 3MF slicer extensions (per-object settings, plate assignment) for Orca/Prusa | IO-03, AM-06 | P1 | Todo | |
| IO-05 | OBJ (+mtl), PLY read/write | MESH-01 | P0 | Todo | |
| IO-06 | STEP AP203/214/242 import/export with assemblies, names, colors (XCAF) | BREP-10, ASM-01 | P0 | Todo | Fusion-exported STEP corpus round-trips |
| IO-07 | IGES import/export | BREP-10 | P1 | Todo | |
| IO-08 | DXF/SVG export of sketches & faces | SK-01 | P1 | Todo | |
| IO-09 | Tessellated export quality controls (refinement presets, per-body) | BREP-08 | P0 | Todo | |
| IO-10 | Image canvas import (calibrated reference images) | RND-03 | P1 | Todo | |

## Testing
Round-trip corpora per format; external validators (3MF consortium `lib3mf` test suite,
STEP validation via OCCT check); opening exported files in Orca/Prusa as manual check.
