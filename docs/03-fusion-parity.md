# Fusion Feature Parity Matrix

Maps Fusion features to Amalgam specs. **Priority** reflects the owner's AM-centric workflow
and is a first guess — the owner should adjust it (especially the rows marked `?`).

- **P0** — needed for the first usable versions (Phases 1–2 in the roadmap)
- **P1** — core parity (Phase 3–4)
- **P2** — advanced / paid-extension parity (Phase 4–5)
- **P3** — nice to have / later
- **—** — out of scope (see vision non-goals)

## Sketch (S03)

| Fusion feature | Pri | Notes |
|---|---|---|
| Line, rectangle (2pt/3pt/center), circle (all variants), arc (all variants), polygon, ellipse, slot (all variants), point | P0 | |
| Spline (fit point + control point), conic | P1 | |
| Text (with font, along path) | P1 | Needs font → outline (ttf-parser) |
| Fillet, chamfer (sketch), trim, extend, break, offset, mirror, circular/rectangular pattern, move/copy, scale | P0 | |
| Project / include, intersect, project to surface, intersection curve | P0 / P1 | project = P0 |
| Geometric constraints (all 12) + dimensions (linear, aligned, angular, radial, diameter) | P0 | |
| Construction lines, centerlines, fix/unfix, "show profile", sketch doctor | P0 | |
| Insert DXF / SVG, canvas (image) | P1 | |
| 3D sketch | P2 | |

## Solid modeling (S04)

| Fusion feature | Pri | Notes |
|---|---|---|
| Extrude (all extents, taper, thin, new/join/cut/intersect/new component) | P0 | |
| Revolve, sweep (path, path+guide, guide surface), loft (rails, centerline) | P0 / P1 | sweep/loft = P1; loft must be more stable than Fusion's (S20) |
| Rib, web, emboss, hole (simple/counterbore/countersink, tapped, hole-from-sketch), thread (modeled + cosmetic) | P1 | threads matter for printed parts |
| Box, cylinder, sphere, torus, coil, pipe | P0 | |
| Pattern (rectangular, circular, on path), mirror | P0 | |
| Thicken, boundary fill, create form (→ S09) | P1 | |
| Fillet (constant, variable, chord, setback, rule fillet*), chamfer | P0 | *rule fillet is Design Extension → P2; robustness in S20 |
| Shell, draft, scale, combine, replace face, split face, split body, silhouette split | P0 / P1 | |
| Move/copy, align, delete, press-pull, offset face, physical material, appearance | P0 | |
| Surface tools: patch, ruled, extend, stitch, unstitch, trim, untrim, reverse normal | P2 | ? |
| Sheet metal | P3 | ? |
| Configurations (tables of variants) | P2 | Design Extension |
| Derive (from another design) | P2 | |

## Construction & inspection (S05)

| Fusion feature | Pri |
|---|---|
| Offset plane, plane at angle, tangent plane, midplane, plane through 2 edges / 3 points / tangent at point, plane along path | P0 |
| Axis (all variants), point (all variants) | P0 |
| Measure, interference, curvature comb, zebra, draft analysis, curvature map, isocurve, accessibility, minimum radius | P0 measure; P1 rest |
| Section analysis | P0 |
| Center of mass, display component colors, properties (mass, volume, area) | P0 |

## Assemblies (S06)

| Fusion feature | Pri |
|---|---|
| New component, components/occurrences tree, activate component | P0 |
| Joints (rigid, revolute, slider, cylindrical, pin-slot, planar, ball), as-built joints, joint origins | P1 |
| Rigid group, ground, motion link, drive joints, motion study, contact sets | P2 |
| External references / linked designs (X-refs) | P3 |

## Mesh workspace (S07)

| Fusion feature | Pri |
|---|---|
| Insert mesh (STL, OBJ, 3MF, PLY), units on import | P0 |
| Repair (close holes, stitch, remove intersections, wrap), make closed mesh | P0 |
| Remesh (adaptive/uniform), reduce (decimate), smooth | P0 |
| Plane cut, combine (boolean), shell, separate, erase & fill, reverse normal | P0 |
| Generate face groups, direct edit (move faces), mesh section sketch | P1 |
| Convert mesh → B-rep (faceted, prismatic, organic) | P1 |
| Mesh analysis (watertight, normals, thickness) | P0 |

## Volumetric / implicit (S08)

| Fusion feature | Pri |
|---|---|
| Volumetric lattice (cell types, variable density/field-driven, skin) | P1 (Design Extension) |
| Variable-thickness shells, smooth booleans, offsets (beyond Fusion) | P1 |
| Automated modeling (connect bodies with generated shapes) | P2 (Design Extension) |

## Form / T-splines (S09)

| Fusion feature | Pri |
|---|---|
| Form primitives, edit form, insert/subdivide/bridge/fill hole/crease/uncrease, symmetry | P2 |
| Convert Form → B-rep | P2 |

## Simulation (S13) — paid / cloud in Fusion

| Fusion study | Pri |
|---|---|
| Static stress (linear) | P1 |
| Modal frequencies | P1 |
| Thermal (steady state), thermal stress | P2 |
| Structural buckling | P2 |
| Nonlinear static (material/geometric nonlinearity, contacts) | P2 |
| Event simulation (explicit dynamics, drop tests) | P3 |
| Shape optimization | P2 (→ S14) |
| Injection molding, electronics cooling | — |

## Generative design (S14) — cloud + paid in Fusion

| Fusion capability | Pri |
|---|---|
| Design space: preserve geometry, obstacles, starting shape | P1 |
| Load cases (structural loads, constraints), multiple load cases | P1 |
| Objectives: min mass, max stiffness; safety factor, displacement, modal constraints | P1 / P2 |
| Manufacturing methods: unrestricted, **additive (build direction, overhang angle)**, 2.5/3/5-axis milling, 2-axis cutting, die casting | P1 additive & unrestricted; P3 others |
| Multiple materials → multiple outcomes; outcome explorer (compare, filter, plot) | P2 |
| Convert outcome → editable body (Fusion: T-spline/B-rep; ours: mesh/SDF, SubD later) | P1 |

## Additive manufacturing (S16) — Manufacturing Extension in Fusion

| Fusion feature | Pri |
|---|---|
| Overhang / support-area analysis | P0 |
| Automatic orientation optimization | P1 |
| Arrange / nesting on build plate (2D and 3D) | P1 |
| Support generation (bar/tree/volume) | P2 — slicers do this well; ours is for preview & SLA/MPBF |
| Machine library, print setup, export to slicer (3MF project) | P1 |
| Metal PBF process simulation (distortion) | P3 |
| Toolpath generation / slicing | — (slicer hand-off) |

## Data & other (S12, S17, S18, S19)

| Fusion feature | Pri |
|---|---|
| Import STEP, IGES, STL, OBJ, 3MF; export STEP, STL, 3MF, OBJ, DXF | P0 |
| Export 3MF with colors/materials/multiple objects | P0 |
| Parameters (user/model, expressions, units) | P0 |
| Physical materials + appearances library | P0 basic / P1 full |
| Python API & add-ins, text commands | P1 |
| Drawings (2D sheets, views, dimensions, PDF/DXF) | P3 ? |
| Photoreal render workspace | P3 |
| Animation workspace (exploded views) | P3 |
| CAM, Electronics, cloud collaboration | — |
