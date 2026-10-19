# S14 — Generative design & shape optimization

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P1 (core) / P2 (outcome explorer, extra constraints) |
| **Crates** | `amg-topopt`, `amg-ui::generative` (Generative workspace) |
| **Lane** | F |
| **Depends on** | S13 (FE model, load cases), S15 (GPU compute), S08 (smoothing/meshing), S07 |
| **Parity** | Generative design section — Fusion's flagship cloud feature, local here |

## Summary
Local, GPU-accelerated topology optimization wrapped in Fusion's generative design workflow:
define design space (preserve geometry, obstacle geometry, starting shape, symmetry planes),
load cases, objectives & limits, manufacturing constraints (**additive with build direction
and overhang angle** first), materials; generate multiple outcomes in the background;
explore/compare; convert an outcome into a smooth, watertight, editable body.

## Design
- **Discretization:** regular voxel grid over the design-space bbox (hexahedral H8 elements,
  matrix-free), resolution user-selectable (e.g. 200k → 8M voxels). Preserves fixed solid,
  obstacles fixed void, voxels outside starting/design space void.
- **Method:** SIMP with density filtering + Heaviside projection (robust formulation,
  eroded/dilated) as the baseline; level-set method (closer to Fusion) as a later alternative.
  Optimizer: MMA (Svanberg) / OC; GPU implementation of matrix-free geometric multigrid PCG
  (as in Aage et al. "Giga-voxel computational morphogenesis" and Wu et al. GPU topopt).
- **Objectives / constraints:** minimize compliance s.t. volume (mass) fraction; minimize mass
  s.t. stress (p-norm aggregated, safety factor target) and displacement limits; multiple
  load cases (weighted/worst-case); frequency constraint (P2).
- **Manufacturing constraints:** additive overhang filter (Langelaar AM filter) for a chosen
  build direction and overhang angle; symmetry; minimum/maximum member size (robust formulation);
  milling/casting constraints P3.
- **Outcomes:** batch of runs over materials × manufacturing methods × parameter variations,
  scheduled in background, each producing density field + metrics (mass, max stress, max
  displacement, safety factor, volume). Outcome explorer: grid of thumbnails, scatter plots,
  filters (like Fusion).
- **Post-processing:** density field → smoothed SDF (S08) → dual contouring mesh (watertight)
  → optional remesh/smoothing (S07) → validation FEA (S13) on the final shape → editable body
  (mesh/implicit; SubD/B-rep later via S09).
- **Shape optimization (Fusion simulation feature):** same machinery restricted to an existing
  body (remove material only), result shown as an overlay of keep/remove regions.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| GEN-01 | Generative study model + workspace UI (design space, preserves, obstacles, symmetry) | FEM-01, UI-05 | P1 | Todo | |
| GEN-02 | Voxelization of design space/preserves/obstacles from mixed bodies | IMP-03 | P1 | Todo | |
| GEN-03 | GPU matrix-free H8 elasticity + multigrid PCG | CMP-05 | P1 | Todo | 1 M voxels one solve < 2 s |
| GEN-04 | SIMP + filters + Heaviside + OC/MMA (min compliance s.t. volume) | GEN-03 | P1 | Todo | MBB beam and cantilever classics reproduced |
| GEN-05 | Multiple load cases, mass objective with stress/displacement constraints | GEN-04, FEM-13 | P1 | Todo | |
| GEN-06 | Additive overhang filter (build dir + angle) | GEN-04 | P1 | Todo | outcomes print without supports in Orca at chosen angle |
| GEN-07 | Outcome → smooth watertight body + validation FEA | GEN-04, IMP-04, FEM-04 | P1 | Todo | |
| GEN-08 | Outcome batches, background scheduling, outcome explorer UI | GEN-07 | P2 | Todo | |
| GEN-09 | Min/max member size, frequency constraints | GEN-05 | P2 | Todo | |
| GEN-10 | Shape optimization on existing body | GEN-04 | P2 | Todo | |
| GEN-11 | Level-set method alternative | GEN-04 | P3 | Todo | |
| GEN-12 | Milling / 2-axis / casting constraints | GEN-04 | P3 | Todo | |

## Testing
Classic benchmarks (MBB, cantilever, L-bracket, bridge) vs published results; overhang
compliance check by computing max overhang angle on output mesh; performance benchmarks.
