# S06 — Components, assemblies & joints

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 (components) / P1 (joints) / P2 (motion) |
| **Crates** | `amg-doc` (component tree, `amg-doc::joints` solver), `amg-features::assembly` |
| **Lane** | E |
| **Depends on** | S01, S02 |
| **Parity** | Assemblies section |

## Summary
Fusion-style multi-component designs inside one file: components with occurrences,
activate-component editing context, joints with DOF, joint solving, drive/motion links,
contact sets. External references (linked designs) later.

## Design
- Component = definition; Occurrence = instance with transform. Bodies/sketches/features
  live in components; the timeline records which component was active.
- Joint types: rigid, revolute, slider, cylindrical, pin-slot, planar, ball; limits; as-built joints;
  joint origins (snap points on geometry via `EntityRef`).
- Joint solver: rigid-body kinematics (6-DOF per occurrence, constraints as equations) — reuse
  the S03 numeric solver core generalized to 3D (shared crate module or copy with SE(3) params).
- Motion: drive joints, motion links (ratios), simple motion study (keyframes → animation).
- Contact sets: collision via mesh BVH (S07), stop motion at contact.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| ASM-01 | Components/occurrences, activate component, new component from bodies | DOC-01 | P0 | Todo | |
| ASM-02 | Ground, move/rotate occurrences, capture position | ASM-01 | P0 | Todo | |
| ASM-03 | Joint origins + joint types + limits | ASM-01, DOC-06 | P1 | Todo | |
| ASM-04 | Joint solver (3D) and drag in assemblies | ASM-03, SK-03 | P1 | Todo | 4-bar linkage drags correctly |
| ASM-05 | As-built joints, rigid groups | ASM-04 | P1 | Todo | |
| ASM-06 | Motion links, drive joints, motion study | ASM-04 | P2 | Todo | |
| ASM-07 | Contact sets | ASM-04, MESH-03 | P2 | Todo | |
| ASM-08 | Linked external designs | DOC-07 | P3 | Todo | |

## Open questions
1. (from S05, S12, 2026-10-20) INS-04, INS-10 and IO-06 need occurrence transforms (`Evaluated::body_to_world(BodyId, occurrence path)`, requested from S01) and component creation from STEP `ForeignModel` assemblies (S12 maps `ForeignNode`s to components).
