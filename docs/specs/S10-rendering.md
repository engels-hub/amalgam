# S10 — Rendering & viewport

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 |
| **Crates** | `amg-render` |
| **Lane** | D |
| **Depends on** | S00 (Gpu) |
| **Parity** | Viewport, visual styles, view cube, section views, analysis displays |

## Summary
wgpu-based CAD viewport: fast shaded+edges rendering of large meshes and tessellated B-rep,
precise GPU picking, selection highlighting, section planes with caps, transparency,
analysis overlays, FEA result fringe plots, implicit ray-march compositing, and an
embeddable widget for egui.

## Design
- `RenderScene` — flat, renderer-owned mirror of what to draw: `GpuMesh` (positions, normals,
  per-face ids, per-face-range → entity id), `EdgeSet` (polylines with ids), instances with
  transforms, materials/appearances, overlays. Updated incrementally from `DocEvent`s by an
  adapter in `amg-ui` (renderer doesn't depend on `amg-doc`).
- Passes: depth pre-pass → opaque forward (PBR-lite; matcap option) → edges/silhouettes
  (geometry edges as lines with depth bias; silhouettes via normal discontinuity screen-space)
  → transparent (weighted blended OIT) → overlays (gizmos, sketch, dimensions) → MSAA resolve
  → FXAA/TAA optional.
- **Picking:** ID buffer (u32 entity id + u32 sub-id) rendered on demand at cursor region;
  supports faces, edges (wider pick lines), vertices, sketch entities; box/lasso selection via
  ID buffer region read-back.
- **Section planes:** clip planes + stencil-based capping with hatching; multiple sections.
- **Large meshes:** u32 indices, meshlets/LOD optional later; target 5 M tris ≥ 60 fps.
- **Visual styles:** shaded, shaded+visible edges, shaded+hidden edges, wireframe, x-ray.
- **Navigation:** orbit (about cursor pick point), pan, zoom-to-cursor, fit, view cube, named
  views, perspective/orthographic, Fusion-compatible mouse bindings + configurable presets.
- **Overlays:** per-vertex scalar → colormap (curvature, draft, overhang, thickness, FEA
  results, deformation scaling + animation), zebra stripes (environment-based), isocurves.
- **Implicit compositing:** ray-march pass writes color+depth that composites with mesh passes.
- **Headless rendering** for thumbnails and tests (offscreen target → PNG).

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| RND-01 | Renderer core, egui-wgpu integration, camera, navigation, view cube | FND-04 | P0 | Todo | spin a 1 M-tri mesh in the app |
| RND-02 | Shaded + edges + silhouettes, visual styles, MSAA | RND-01 | P0 | Todo | 5 M tris ≥ 60 fps on owner's GPU |
| RND-03 | `RenderScene` incremental updates, instancing, depth compositing hook | RND-01 | P0 | Todo | |
| RND-04 | GPU picking (faces/edges/vertices), hover/selection highlight, box select | RND-02 | P0 | Todo | edge picking reliable at 1 px lines |
| RND-05 | Transparency (WBOIT), per-body opacity, x-ray | RND-02 | P1 | Todo | |
| RND-06 | Section planes with capping + hatching | RND-02 | P0 | Todo | |
| RND-07 | Overlay layer: gizmos, sketch rendering, dimensions/text (MSDF fonts) | RND-02 | P0 | Todo | crisp text at all zooms |
| RND-08 | Scalar-field overlays, colormaps, legends, zebra, deformation animation | RND-02 | P0 | Todo | needed in Phase 1 by overhang analysis (AM-02) |
| RND-09 | Headless/offscreen rendering + visual regression harness | RND-02 | P1 | Todo | screenshot tests in CI |
| RND-10 | Appearance materials (PBR textures, environment lighting) | RND-02, MAT-02 | P2 | Todo | |
