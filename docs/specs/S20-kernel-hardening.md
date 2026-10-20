# S20 — Kernel hardening (fillet & loft robustness, Rust rewrites)

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 (failure capture) / P1 (fillet + loft hardening) / P3 (Rust rewrites, last phase) |
| **Crates** | `amg-nurbs` (see ADR-013), `amg-brep` (strategies, fallbacks), `testdata/kernel/` |
| **Lane** | A |
| **Depends on** | S02 (OCCT backend), S04 (fillet/loft features) |
| **Parity** | Fillet and loft must be **more reliable than Fusion's**; loft instability was a specific pain point for the owner |

## Summary
OCCT's weakest operations for our use are **fillets** (legacy `ChFi3d` built on the old
`TopOpeBRep` boolean structures) and **lofts** (`BRepOffsetAPI_ThruSections`: fragile
section compatibility, no real rails or end conditions). This spec makes both measurably
robust, driven by failure corpora and success-rate metrics, in three stages:

1. **Now (Phases 1–3):** capture every kernel failure; smarter strategies around OCCT; our own
   loft surface construction in Rust; a fast-path fillet for analytic edges on modern booleans.
2. **Ongoing:** report/patch isolated bugs upstream in OCCT.
3. **Last (Phase 7, after all other parity work):** Rust rewrite of the fillet engine, then
   progressive replacement of other OCCT operations where the corpora show it pays off.

## Design

### Failure capture & corpora
- Every failed kernel operation can dump its inputs (`.brep` shapes + JSON op parameters +
  OCCT error) into `%LOCALAPPDATA%/Amalgam/kernel-failures/` (on by default for the owner).
  A CLI command replays a dump (`amg-cli kernel replay <dir>`).
- Curated corpora in `testdata/kernel/{fillet,loft}/`: owner's parts, cases harvested from
  FreeCAD/OCCT bug trackers, synthetic generators (random prismatic parts with random edge
  sets; random section stacks for lofts).
- Harness reports **success rate, validity (BRepCheck), deviation from expected volume, time**
  per strategy; tracked over time (`docs/notes/kernel-metrics.md`). Every hardening WP must
  move these numbers.

### Fillet hardening (stages 1 & 2 from the fillet discussion)
- **Strategy wrapper:** before reporting failure, try in order: unify same-domain faces
  (`ShapeUpgrade_UnifySameDomain`) → heal/adjust tolerances → regroup edges (by chain, by
  radius, largest first) → per-edge sequential fillets → isolate failing edge. Report the edge,
  the reason, and a max workable radius (bisection).
- **Analytic fast path:** constant-radius fillets between plane/cylinder/cone faces — build the
  exact blend surface (cylinder/cone/torus, or swept circle along the spine), trim and merge
  with the modern General Fuse booleans; corner patches for 3-edge vertices (setback patch,
  then GeomPlate / own N-sided patch). Falls back to `ChFi3d` when the case isn't covered.

### Loft hardening (owner priority)
Loft failures mostly come from *section preparation*, not the surface math, so we own both:

1. **Section preprocessing**
   - consistent orientation of all section wires; closed/open validation with clear errors;
   - start-vertex alignment minimizing total twist (search over cyclic shifts + direction);
   - **edge-count matching** by splitting edges at arc-length-proportional points instead of
     OCCT's ad-hoc splitting;
   - **merge tangent edge chains** into single curves (fewer, cleaner lateral faces);
   - point sections (apex) at start/end;
   - **user point mapping** override (drag correspondence lines in the viewport).
2. **Surface construction in Rust (`amg-nurbs`)**
   - make section curves compatible: common degree, then **approximate onto a shared knot
     vector** (bounded control-point count) rather than exact knot merging, which explodes;
   - skinning per *The NURBS Book* (Piegl & Tiller, ch. 10) with centripetal/chord
     parametrization across sections;
   - **end conditions:** free, direction, tangent to adjacent face (G1), smooth (G2), with weight;
   - **rails:** Gordon surface (network of section × rail curves) — the principled way to honour
     rails; **centerline:** frames from the centerline (rotation-minimizing), sections
     interpolated along it.
   - Output: one B-spline surface per lateral edge-chain → converted to OCCT
     `Geom_BSplineSurface` → faces sewn into a solid with caps (OCCT sewing + BRepCheck).
3. **Fallback chain:** own builder → OCCT `ThruSections` → OCCT `MakePipeShell` with multiple
   sections; the feature records which path succeeded.
4. **Diagnostics in the dialog:** correspondence lines between sections, twist warning,
   self-intersection check of the result, highlighted offending section.

### Rust rewrite (Phase 7 — only after everything else in the roadmap)
- **Fillet engine in Rust** on top of `amg-nurbs`: rolling-ball spine computation, blend
  surface generation (constant, variable, chord), vertex blends, trimming via our own or OCCT
  intersection routines, topology rebuild. Must beat stages 1–2 on the fillet corpus to ship.
- **Progressive kernel replacement:** use the corpora to decide which further OCCT operations
  (booleans, shell/offset, surface-surface intersection) are worth replacing with Rust
  implementations behind the `BrepKernel` trait. Long-term option: a full Rust kernel, possibly
  building on/contributing to `truck`.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| KH-01 | Kernel failure capture (input dump + replay CLI) | BREP-01 | P0 | Todo | any failed op reproducible from its dump |
| KH-02 | Fillet corpus + success-rate harness + metrics note | KH-01, BREP-06 | P1 | Todo | ≥ 200 cases; baseline OCCT rate recorded |
| KH-03 | Fillet strategy wrapper + diagnostics (failing edge, max radius) | KH-02, SOL-05 | P1 | Todo | measurable success-rate gain over baseline |
| KH-04 | Analytic fast-path fillet on modern booleans + 3-edge corner patches | KH-03 | P1 | Todo | ≥ 98% on analytic-edge subset of corpus |
| KH-05 | Loft corpus + harness (incl. owner's failing Fusion lofts as STEP sections) | KH-01, BREP-03 | P1 | Todo | ≥ 150 cases; baseline ThruSections rate recorded |
| KH-06 | Section preprocessing (orientation, twist alignment, edge matching, tangent merge, point sections, user mapping) | KH-05, SK-06 | P1 | Todo | eliminates the preprocessing failure class in corpus |
| KH-07 | `amg-nurbs`: curve/surface types, eval, degree elevation, knot ops, approximation onto shared knots, skinning, end conditions, Gordon surfaces | FND-03 | P1 | Todo | unit tests vs NURBS Book examples; G1/G2 checks |
| KH-08 | Loft builder (per-chain faces, sewing, caps, rails, centerline), fallback chain, diagnostics preview | KH-06, KH-07, SOL-01 | P1 | Todo | ≥ 98% loft corpus success, all results BRepCheck-valid |
| KH-09 | Upstream OCCT bug reports/patches for isolated issues found by the harnesses | KH-02 | P3 | Todo | ongoing |
| KH-10 | **Phase 7:** Rust fillet engine | KH-04, KH-07 | P3 | Todo | beats KH-03+KH-04 success rate on full corpus |
| KH-11 | **Phase 7:** corpus-driven replacement of further OCCT ops (booleans, offset, SSI) with Rust | KH-10 | P3 | Todo | each replacement beats OCCT on its corpus |

## Open questions
1. Owner: any old Fusion files whose lofts failed? Export their section sketches (DXF/STEP) for KH-05.
   **Answered 2026-10-20:** yes, private, supplied later (before KH-05). Section sketches and rails
   (DXF/STEP) plus a note on how Fusion failed go under `$AMG_TESTDATA_DIR/kernel/loft/owner/`
   (S00 Open question 2); not committed.
2. Own N-sided corner patch vs OCCT `GeomPlate` for KH-04 — decide on corpus results.
