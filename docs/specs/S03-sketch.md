# S03 — Sketching & constraint solver

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 |
| **Crates** | `amg-sketch`, `amg-features::sketch` (sketch-mode UX in `amg-ui` is lane D's UI-09) |
| **Lane** | B |
| **Depends on** | S00; S01 for `EntityRef` (projection), S02 for projection/intersection |
| **Parity** | Sketch section of parity matrix |

## Summary
2D parametric sketches on planes/planar faces: entities, geometric constraints, dimensions,
a robust and fast constraint solver with DOF feedback, and profile/region detection that
produces faces for extrude/revolve.

## Scope
**In:** entity model, constraints, dimensions (driving/driven), solver, dragging under
constraints, profile detection, sketch editing ops (trim, extend, offset, fillet…), projection
of 3D geometry, text, DXF/SVG import, 3D sketch (P2).
**Out:** UI tool interaction details (S11 owns input handling, uses this API).

## Design
- **Entities:** point, line, arc, circle, ellipse, elliptical arc, spline (B-spline: control
  points or fit points), conic, text (as grouped curves). Construction flag. Each entity is
  defined by solver variables (point coords, radii, angles).
- **Constraints:** coincident, collinear, concentric, midpoint, fix, parallel, perpendicular,
  horizontal, vertical, tangent, curvature (G2), equal, symmetric, point-on-curve; dimensions:
  distance, horizontal/vertical distance, angle, radius, diameter, arc length, ellipse axes.
  Dimension values are `Expr` (S01 parameters).
- **Solver (ADR-007):**
  1. Build constraint graph → decompose into connected / rigid clusters (DR-planning or
     bipartite matching + Dulmage–Mendelsohn).
  2. Solve each cluster with Newton/LM (dense `nalgebra` for small, sparse for large), using
     analytic Jacobians.
  3. DOF analysis via Jacobian rank (SVD/QR) → per-entity status: under (blue), fully (black),
     over/conflicting (red) with the list of conflicting constraints.
  4. Dragging: solve with a soft constraint pulling the dragged point to the cursor, minimal
     movement of others (weighted least squares).
  Targets: 500-entity sketch solves in < 5 ms incremental.
- **Profiles:** planar arrangement of curves (with intersection splitting, tolerance) →
  half-edge graph → faces with holes → `Profile` list. Stable profile ids across edits via
  bounding curve ids (feeds persistent naming).
- **Projection/intersect:** via `amg-brep` queries; projected entities hold `EntityRef` and
  update on recompute.
- **Text:** `ttf-parser` / system fonts → outlines → splines.
- **Import:** DXF (`dxf` crate) and SVG (`usvg`) → entities.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| SK-01 | Entity model, sketch plane/transform, serialization | FND-03 | P0 | Todo | create/serialize all P0 entities |
| SK-02 | Constraint equations + analytic Jacobians for all constraints | SK-01 | P0 | Todo | finite-difference Jacobian check passes |
| SK-03 | Solver core: decomposition, Newton/LM, convergence, failure reporting | SK-02 | P0 | Todo | solver benchmark suite (SolveSpace/PlaneGCS test cases ported) |
| SK-04 | DOF analysis + over-constraint detection + diagnosis | SK-03 | P0 | Todo | correct status on 50 hand-made cases |
| SK-05 | Drag solving | SK-03 | P0 | Todo | smooth drag at 120 Hz on 200-entity sketch |
| SK-06 | Profile/region detection with holes, tolerance handling | SK-01 | P0 | Todo | nested profiles, overlapping curves, tiny gaps |
| SK-07 | Edit ops: trim, extend, break, offset, fillet, chamfer, mirror, patterns, scale | SK-06 | P0 | Todo | per-op tests |
| SK-08 | Splines (fit/control), conics, ellipses | SK-02 | P1 | Todo | constraints on splines work |
| SK-09 | Projection / include / intersect / intersection curve | SK-01, BREP-09, DOC-06 | P0 | Todo | projected edges follow model edits |
| SK-10 | Text (fonts, along path) | SK-08 | P1 | Todo | text extrudes correctly |
| SK-11 | DXF/SVG import | SK-08 | P1 | Todo | sample files import to scale |
| SK-12 | 3D sketch | SK-03 | P2 | Todo | 3D lines/splines with 3D constraints |

## Testing
Port test cases from SolveSpace and FreeCAD PlaneGCS; property test: random well-constrained
sketches perturbed → solver recovers; profile detection fuzzing.

## Open questions
1. Own solver vs `libslvs` FFI — refinement should prototype SK-03 on 10 cases and decide (ADR-007).
2. Auto-constraints while drawing (snap inference) — rules live in S11 but need solver hooks.
