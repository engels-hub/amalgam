# Open questions for the owner

Agents append here; the owner answers inline and the refining agent moves the answer into the spec.

| # | Spec | Question | Answer |
|---|---|---|---|
| 1 | Parity | Review priorities in `03-fusion-parity.md`, especially rows marked `?` (surface tools, sheet metal, drawings). | |
| 2 | S16 | Which printers / technologies (FFF, resin, SLS, metal) and which slicer (Orca, Prusa, Bambu Studio)? | |
| 3 | S13 | Which loads/constraints do you use most in simulation? Do you need orthotropic printed-material models? | |
| 4 | S04 | Can you supply 5–10 representative Fusion designs (STEP export + timeline screenshots) as parity references? | |
| 5 | S00 | Should the repo get a GitHub remote (enables CI)? | Yes — `github.com/engels-hub/amalgam` (2026-10-19). Moved into S00. |
| 6 | ADRs | Accept or change the `Proposed` ADRs (OCCT, Manifold, wgpu, egui, own sketch solver, SubD instead of T-splines, Python, own NURBS module). ADR-002 (OCCT), ADR-003, ADR-004/005 and ADR-007 gate specific work packages — see roadmap §Decision gates. Phase 0 (FND-01..04, FND-06) can start without them. | |
| 7 | S20 | Do you have old Fusion designs with failing lofts? Exporting their section sketches (DXF/STEP) gives the loft test set real cases. | |
