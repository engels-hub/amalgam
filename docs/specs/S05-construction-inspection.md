# S05 — Construction geometry & inspection

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 / P1 |
| **Crates** | `amg-features::construct`, `amg-ui::inspect`; overlays come from lane D's RND-08 |
| **Lane** | A |
| **Depends on** | S01, S02, S07 (mesh analysis), S10 (overlay shaders) |
| **Parity** | Construction & inspection section |

## Summary
Construction planes/axes/points as timeline features, and inspection tools: measure,
section analysis, interference, physical properties, and surface analyses (zebra, curvature,
draft, curvature comb, min radius, accessibility). Overhang analysis lives in S16 but reuses
the same overlay mechanism.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| CON-01 | Construction planes (offset, angle, tangent, midplane, 2 edges, 3 points, tangent at point, along path) | SOL-01 | P0 | Todo | all variants, persistent refs |
| CON-02 | Construction axes (all variants) and points (all variants) | CON-01 | P0 | Todo | |
| INS-01 | Measure tool on meshes (distance, angle, area, length, min distance) + measure UI | MESH-02, MESH-03 | P0 | Todo | values match analytic |
| INS-07 | Extend measure to B-rep entities (faces, edges, vertices, construction geometry) | INS-01, BREP-09 | P0 | Todo | values match analytic |
| INS-02 | Section analysis (plane, with capping, hatching, multiple sections; saved as browser item) | RND-06 | P0 | Todo | caps watertight on meshes and solids |
| INS-03 | Properties: mass, volume, area, CoM, inertia tensor; CoM display | BREP-09, MAT-01 | P0 | Todo | |
| INS-04 | Interference detection between bodies/components | BREP-04, MESH-05 | P1 | Todo | |
| INS-05 | Zebra, curvature map, draft analysis, isocurves (shader overlays) | RND-08 | P1 | Todo | |
| INS-06 | Curvature comb, minimum radius, accessibility | BREP-09 | P1 | Todo | |

## Testing
Analytic checks (measure, properties); visual regression screenshots for overlays.
