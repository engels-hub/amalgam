# S19 — Later workspaces (drawings, render, animation)

| | |
|---|---|
| **Status** | Draft (placeholder — refine only when owner raises priority) |
| **Priority** | P3 |
| **Crates** | TBD (`amg-drawing`) |
| **Lane** | TBD |
| **Depends on** | S02, S10, S06 |

## Summary
Placeholders so the parity matrix is complete:
- **Drawings:** sheets, base/projected/section/detail views (hidden line removal via OCCT HLR),
  dimensions, annotations, BOM, PDF/DXF export.
- **Render workspace:** path-traced photoreal rendering (wgpu ray query or a CPU path tracer).
- **Animation workspace:** exploded views, storyboards, video export.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| LAT-01 | Drawings MVP (views + dims + PDF) | BREP-10 | P3 | Todo | |
| LAT-02 | Photoreal renderer | RND-10 | P3 | Todo | |
| LAT-03 | Exploded views & animation | ASM-06 | P3 | Todo | |
