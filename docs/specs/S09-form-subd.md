# S09 — Form workspace (subdivision surfaces)

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P2 |
| **Crates** | `amg-subd`, `amg-features::form`, `amg-ui::form` |
| **Lane** | C |
| **Depends on** | S07 (mesh), S02 (NURBS output), S10/S11 (editing UI) |
| **Parity** | Form / T-splines section (ADR-009) |

## Summary
Freeform modeling equivalent to Fusion's Form workspace using Catmull–Clark subdivision with
creases (OpenSubdiv semantics): primitives, edit form (move/rotate/scale faces/edges/vertices
with soft modification), insert/delete edges, subdivide, bridge, fill hole, crease/uncrease,
symmetry, pull to target, then convert to B-rep for downstream solid features.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| FRM-01 | Control cage data structure + Catmull–Clark evaluation with creases (limit surface) | MESH-01 | P2 | Todo | matches OpenSubdiv reference outputs |
| FRM-02 | Form primitives (box, plane, cylinder, sphere, torus, quadball, face) | FRM-01 | P2 | Todo | |
| FRM-03 | Edit form manipulator, soft selection, selection modes (loop/ring/grow) | FRM-01, UI-06 | P2 | Todo | |
| FRM-04 | Topology tools: insert edge/point, subdivide, bridge, fill hole, merge, weld, delete | FRM-01 | P2 | Todo | |
| FRM-05 | Crease/uncrease, sharpen, symmetry (mirror/circular) | FRM-01 | P2 | Todo | |
| FRM-06 | SubD → NURBS B-rep (regular patches as bicubic; extraordinary vertices via approximation) | FRM-01, BREP-10 | P2 | Todo | result passes BRepCheck, G1 within tol |
| FRM-07 | Pull to target body, flatten, match, thicken | FRM-04 | P3 | Todo | |
| FRM-08 | Quad remesh (mesh → SubD cage) for organic mesh→B-rep | FRM-06, MESH-06 | P3 | Todo | |

## Open questions
1. (from S02, 2026-10-20) FRM-06 should depend on BREP-03 (B-spline `make_face`) + BREP-10 (sew).
2. (from S20, 2026-10-20) FRM-06 should build on `amg-nurbs`; this needs the layering edge `amg-subd → amg-nurbs` (QUESTIONS #41).
