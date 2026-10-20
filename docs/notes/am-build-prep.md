# Additive build preparation: conventions, overhang math, orientation and slicer hand-off

Design note for [S16](../specs/S16-additive-manufacturing.md). Written during S16 refinement
(2026-10-20). It covers:
- frames and angle conventions;
- the overhang and support-volume definitions that AM-02, AM-04 and the S10 overhang shader
  must all agree on;
- the orientation search;
- how profiles are imported from the owner's slicers and how the hand-off works.

Status: **Written** (refinement output; revisit after AM-02 and AM-06 land).

---

## 1. Frames and conventions

| Thing | Convention |
|---|---|
| Plate frame | Right-handed, mm. Origin at the **front-left corner of the printable area's bounding rectangle**. +X right, +Y back, +Z up (build direction). Same for every technology. |
| Build direction `d` | +Z of the plate frame. In body-local coordinates it is `d_local = R⁻¹·Z`, where `R` is the item's rotation. |
| Item transform | `T = translate(t) · R`, mapping body-local → plate. It is rigid (no scale, no mirror): `R` is a unit `DQuat`. |
| Placement anchor | `Placement.xy` is the plate position of the **centre of the rotated footprint AABB**. `ZMode::OnPlate { elevation }` puts the rotated minimum at `z = elevation` (0 for FFF; the support lift for resin). The translation is recomputed from the geometry every time (see §4), so the item stays on the plate and in place when an upstream feature changes the body. |
| Slicer frame | Each machine stores `slicer_origin`: the position of the slicer's (0,0) in the plate frame. OrcaSlicer and PrusaSlicer use the bed corner, so it is `printable_area.min`, normally (0,0). Chitubox centres the plate, so it is the plate centre. The hand-off applies `translate(-slicer_origin)`. |
| Angles | Radians internally (S00). Degrees only in the UI. |

### 1.1 Overhang angle convention
For a face with unit normal `n` (body-local) and build direction `d_local`, let

    c = −n · d_local          (cosine of the angle between n and "straight down")
    ψ = acos(clamp(c, −1, 1)) ∈ [0, π]      ("down angle": 0 = ceiling facing straight down,
                                              π/2 = vertical wall, π = facing straight up)

For a downward-facing face (`c > 0`), ψ is also the **slope of the face from the horizontal**.
The face needs support when ψ is less than the critical angle α:

    overhang  ⇔  c > cos α     (strict; c == cos α is self-supporting)

This is the **OrcaSlicer / PrusaSlicer convention**. Orca's "Threshold angle" tooltip says
supports are generated for overhangs whose slope angle is below the threshold; its default is
30° (`support_threshold_angle`, `PrintConfig.cpp`). Cura measures the "support overhang angle"
from the vertical. It converts as `α = 90° − α_cura`. The UI shows Amalgam's convention and
says so in the tooltip.

Default `α`:
- **45°** for FFF: the usual DfAM rule, and Fusion's default.
- **45°** for resin, until the owner says otherwise (QUESTIONS proposal in the S16 report).
- When an Orca *process* preset is imported (AM-10), its `support_threshold_angle` can be
  offered as the setup value. A value of 0 there means "use threshold overlap", so we ignore it.

### 1.2 Face classes
Computed per face. Heights `z` are in the plate frame after the effective transform.

| Class | Rule (evaluated in this order) |
|---|---|
| `Degenerate` | `face_normal == ZERO` (S07 `IndexedMesh::face_normal`) |
| `Contact` | `c > 0` and the face's **max** z − z_min ≤ `contact_tolerance` |
| `Overhang` | `c > cos α` |
| `Supported` | everything else (upward faces, walls, self-supporting slopes) |

`contact_tolerance` defaults to half the machine's first-layer height (0.1 mm for 0.2 mm). The
fallback is 0.1 mm. It is relative to the item's minimum z, so an elevated resin item still
gets `Contact` faces for the faces that touch the raft. Orca uses a similar first-layer slab
test (`z_max < min_z + FIRST_LAY_H`) to measure "bottom" area.

**Islands (point overhangs).** A vertex is a *local minimum* if, in every face that contains it,
no other vertex is lower than it by more than `1e-9` mm (one O(F) pass; ties are broken by
vertex index). It is an **island** if it is a local minimum and its height is > `contact_tolerance`.
A downward-pointing cone tip or the bottom of a sphere that is not on the plate is an island,
even though none of its faces is an overhang at a steep α. Downward ridges (edges) are not
detected in Phase 1. This is a documented limitation; AM-07's layer-island check covers them.

### 1.3 Shader parity (S10)
The live overlay is computed in a shader (S10 RND-08 "overhang mode") from the same inputs:
`d_local`, `cos α`, the contact plane (`dot(d_local, p) + w ≤ contact_tolerance`) and the face
normal. To keep shader and CPU in agreement, AM-02 ships `tests/golden/am/overhang_classes.toml`:
5 000 random (normal, d, α, height) tuples with the expected class. The RND-08 shader test
evaluates the WGSL function on them. Only tuples within 1e-6 of a threshold may disagree, and
they are flagged in the file.

---

## 2. Metrics (AM-02)

All metrics are for one item at one transform. Areas are in mm², volumes in mm³.

| Metric | Definition |
|---|---|
| `overhang_area` | Σ area of `Overhang` faces |
| `overhang_projected_area` | Σ area · c over `Overhang` faces (shadow on the plate) |
| `contact_area` | Σ area · c over `Contact` faces |
| `footprint_area` | area of the 2D convex hull of all transformed vertices projected to XY |
| `height` | z_max − z_min (+ elevation is reported separately) |
| `island_count` | number of island vertices (§1.2) |
| `stable` | `Some(true)` if the XY projection of the volume centroid lies inside the convex hull of the `Contact` vertices, with margin ≥ 0. `Some(false)` if outside. `None` if there is no contact (elevated / resin) or the mesh is not closed. `com_margin` gives the signed distance. |
| `support` | the support estimate (§3), optional because it costs rays |
| `approximate` | true when the mesh is not watertight (S07 `AnalysisReport::is_watertight`) and the support estimate fell back to the face-centroid method |

Rotating about the build axis must not change any metric beyond floating-point noise. The grid
metrics (§3) may change by at most the documented cell bound. Both are tested.

---

## 3. Support volume estimate

### 3.1 Column method (closed meshes)
Prior art:
- Ezair, Massarwi & Elber (2015) compute support volume on the GPU by depth layers along `d`,
  and show that volume is continuous but non-smooth in the orientation.
- PrusaSlicer's and Orca's FFF slicers compute the real supports by layer overlap. That is too
  expensive for orientation scoring.

We use an exact-per-column definition that works on the CPU (BVH rays, S07 MESH-03) and maps 1:1
to a GPU depth-peeling path (AM-13):

> Cast a ray along `+d` from below the plate through the cell centre of a square grid (cell `s`)
> covering the footprint AABB. Walk the hits sorted by t. Track `last_exit_z`, which starts at
> 0 (the plate, or the raft top for elevated items). At a hit on a **downward-facing** face
> (entry, c > 0), add `(z_entry − last_exit_z) · s²` to the support volume **if that face is
> `Overhang`**. At an upward-facing hit (exit), set `last_exit_z = z_exit`. For consecutive
> entries without an exit (open or non-manifold input), only the first entry counts.

Properties:
- The gap under a self-supporting downward slope (ψ ≥ α) is not counted. It needs no support.
- `on_part_volume` is the part of the volume where `last_exit_z > plate`: the support stands
  on the model, which is bad for cosmetic surfaces and hard to remove.
  `on_plate_volume = volume − on_part_volume`.
- `footprint_area` (support) = number of cells with any counted gap · s².
- Error: O(s · perimeter of the support footprint · max gap) from cell quantisation. Golden tests
  check convergence as s → 0.

Cell size: `support_cell` defaults to 0.5 mm, and grows automatically so that the ray count
stays ≤ `max_rays` (default 262 144 = 512²). The report carries the cell size actually used.
Rays use `MeshSpatial::ray_cast_all` in body-local coordinates (origin and direction transformed
by `T⁻¹`), so the BVH never has to be rebuilt when the item rotates.

### 3.2 Face-centroid fallback (open meshes)
For each `Overhang` face, cast one ray from the centroid along `−d`. If it hits, h = t;
otherwise h = centroid height above the plate. Add `area · c · h`. Faces larger than `4 s²` are
subsampled at their barycentric-grid points. The result is flagged `approximate`. This is the
approximation the PrusaSlicer SLA "least supports" score and Tweaker use (area-weighted
per-face terms with no visibility), and it is only used when columns are ill-defined.

### 3.3 Analytic check values (golden tests)
- **Sphere r = 10 tangent to the plate, α = 45°:**
  - overhang cap area = 2πr²(1 − cos α) = **184.030 mm²**;
  - projected area = π(r sin α)² = **157.080 mm²**;
  - support volume = π a² r − (2π/3)(r³ − (r² − a²)^{3/2}) with a = r sin α = **216.882 mm³**.

  The contact patch makes the discrete values slightly smaller. The tests assert convergence:
  the error at least halves when the triangle count quadruples and the cell halves.
- **T-shape** (stem 10×10×20, bar 30×10×5 centred on top): overhang area = projected area =
  200 mm²; support volume = 200 × 20 = **4 000 mm³**, all on the plate.
- **Shelf over a base** (base 30×30×5, post 5×5×15 at one corner, top plate 30×30×3 on the
  post): support under the top plate stands on the base. `on_part_volume` = (900 − 25) × 15 =
  13 125 mm³; `on_plate_volume` = 0.
- **10 mm cube on a face:** overhang 0; contact 100 mm²; support 0; height 10; stable.
- **Cube balanced on an edge** (45° about X): the two lower faces are at exactly ψ = 45°. At
  α = 45° they are `Supported` (strict rule); at α = 46° they are `Overhang`. `stable = false`.

---

## 4. Placement (AM-03)

- **Effective transform:** `T = translate(t) · R` with
  `t.xy = Placement.xy − centre_xy(AABB(R·V))` and `t.z = elevation − min_z(R·V)`, or `z` for
  `ZMode::Fixed`.
  - Cost: one pass over vertices (rayon, ordered min/max reductions; deterministic).
  - The result is memoised by the caller, keyed by (`content_hash`, placement).
- **Place face on plate** (Fusion "place on build plate"): the picked face's normal `n`, rotated
  by R, must map to −Z.
  - Use the shortest-arc quaternion `q = rotation_arc(R·n, −Z)` and set `R' = q·R`. This keeps
    the yaw that is not involved.
  - The anti-parallel case (`R·n ≈ +Z`) rotates 180° about plate X.
  - For a mesh, the picked "face" is the face group of the triangle if the group has a planar
    fitted primitive or a normal spread ≤ 1°, using the area-weighted mean normal. Otherwise
    the triangle's own normal is used.
- **Rotate about the build axis:** `R' = rot_z(θ)·R`. The anchor is unchanged, so the item spins
  in place.
- **Fit check:**
  - Height: `max_z ≤ volume.height`.
  - Footprint: the convex hull of the projected transformed vertices is computed with Andrew's
    monotone chain over the vertices, after an AABB early-out. Each hull vertex must lie inside
    the bed polygon, and no hull edge may cross it, which also handles non-convex beds. The hull
    must not intersect any `exclude` polygon.
  - Tolerance: 1e-6 mm (`Tolerance::DEFAULT.linear`).
  - The report gives the max outside distance, so the UI can say "3.2 mm too wide".
- **Item overlap:** convex hulls are tested with SAT, with a minimum gap (default 0). Overlap
  is a warning, not an error. Slicers accept touching items.
- **Build volume geometry** for display:
  - the plate as a thin polygon mesh;
  - a 10 mm grid as `Polyline3`s;
  - the volume edges as polylines;
  - exclude areas as separate meshes.

  amg-ui::am hands these to RND-03/RND-07 as non-pickable helper items. `amg-render` never sees
  `amg-am` types.

---

## 5. Orientation optimisation (AM-04, AM-13)

### 5.1 Prior art and what we take from it

| Source | Approach | Take / reject |
|---|---|---|
| Tweaker-3 (Schranz; Cura "Auto Orientation" plugin) | Candidates are the 10 most frequent face normals by area ("area cumulation"), plus 18 fixed supplements. Unprintability = `A(overhang + B) + F(overhang + C) / (D + contour·k + bottom·k')`, with constants fitted by an evolutionary algorithm. Lower is better. | **Take:** area-cumulated normal candidates and an explicit bottom/contact term. **Reject:** the opaque fitted rational score; we use a normalised weighted sum the user can understand. |
| OrcaSlicer / Bambu Studio `Orient.cpp` (AGPL-3.0, reference only) | Tweaker derivative: candidates from area cumulation (10 directions) + hull normals (14) + supplements. Adds a bottom convex-hull area term, a penalty on "appearance" faces (×3) and low-angle-face handling, and uses per-object overhang angle `ASCENT = cos(π − α)`. | **Take:** convex-hull candidates, the cosmetic-face penalty concept (×3 default weight), and a minimum bottom area guard. |
| PrusaSlicer SLA `Rotfinder.cpp` (AGPL-3.0, reference only) | "Least supports": if the object sits on the floor, the candidates are the 3D convex-hull face rotations; otherwise a brute-force grid over X/Y rotations (≈ √N × √N). Per-face score is `√area · (1 − ψ/π)³`. Also has "best misalignment" (avoid axis-aligned faces, against resin layer artefacts) and "fit Z to bed". | **Take:** hull faces = stable poses for on-plate machines, and the 2-DOF search space (yaw is irrelevant to supports). **Reject:** the √area heuristic in favour of the real support volume (§3). |
| Ezair, Massarwi & Elber 2015, *Computers & Graphics* 51:117–124 | GPU support volume per orientation by depth layers. Volume is continuous but non-smooth, so derivative-free search is needed. | **Take:** the volume definition (§3.1) and the GPU depth-layer path (AM-13). |
| Zhang et al. 2015, *Perceptual models of preference in 3D printing direction*, ACM TOG 34(6) | Learned visual-saliency penalty for supports on visible regions. | **Defer.** User-selected cosmetic face groups give the same control without a learned model. |
| Delfs, Tows & Schmid 2016, *Additive Manufacturing* 12 | Surface-quality (roughness) model per face as a function of build angle, plus build time. | **Take:** a stair-stepping / cusp term (below). |
| Pandey, Reddy & Dhande 2003; Thrimurthulu, Pande & Reddy 2004 (FDM orientation, cusp height) | Cusp height `h·|n·d|` per face; area-weighted average as a surface-finish objective; GA search. | **Take:** the cusp formula. **Reject:** GA (deterministic sphere sampling + local refine is cheaper and reproducible). |

### 5.2 Search
The search is over the build direction `d` on S² (2 DOF). Yaw is chosen afterwards: keep the
current one, or (AM-05) the one that minimises the footprint AABB.
1. **Stable poses** (on-plate technologies): faces of the 3D convex hull of the vertices, with
   coplanar hull triangles merged within 0.5°. Keep a pose only if the projection of the volume
   centroid lies inside the merged hull face. Rank by face area; keep up to 64.
2. **Normal clusters:** area-weighted histogram of face normals on a 2 562-cell geodesic grid
   (icosphere level 4). Take the 10 heaviest cells and use −n as the build direction (face
   down).
3. **Sphere sampling:** Fibonacci sphere, N = 1 024 by default (mean spacing ≈ 6.3°;
   4 096 ≈ 3.2° for "thorough"). It covers elevated / resin cases and non-hull optima.
4. **Local refinement:** take the top-k = 8 distinct candidates (≥ 10° apart). Run a
   compass/pattern search on the tangent plane of S²: step 4° halving to 0.25°, 4 directions
   per step, accept on strict improvement. On-plate items must stay on a stable pose. For them
   refinement only runs when the stable pose set is empty or `elevated` is set.
5. Deduplicate (< 2°) and return the best `n_results` (default 5) with full metrics, so the user
   can choose (like Fusion's orientation study results).

Determinism comes from fixed candidate order, scores compared with ties broken by candidate
index, and no RNG.

### 5.3 Score
Each term is normalised to be dimensionless and O(1) for typical parts:

    S = w_v · V_support / V_ref
      + w_a · A_overhang_proj / A_total
      + w_h · height / diag
      + w_c · (1 − min(A_contact, A_cref) / A_cref)            (FFF only; A_cref = largest merged hull face)
      + w_k · A_support_on_cosmetic / max(A_cosmetic, ε)       (cosmetic faces; ×3 Orca-style emphasis via w_k)
      + w_s · Σ_i A_i · |n_i·d| · [0.02 < |n_i·d| < 0.999] / A_total   (stair-stepping; cusp ∝ h·|n·d|,
                                                                 flats and walls excluded)
      + 100 · [contact_area < A_min]                           (FFF stability guard, A_min = 1 mm²)
      + 100 · [does not fit build volume in any yaw]           (fit guard)

`V_ref` is the part volume (or the AABB volume for open meshes), and `diag` is the AABB
diagonal of the part.

| Preset | w_v | w_a | w_h | w_c | w_k | w_s |
|---|---|---|---|---|---|---|
| Minimise supports (default FFF) | 1.0 | 0.5 | 0.1 | 0.3 | 1.0 | 0.0 |
| Minimise height / time | 0.3 | 0.2 | 1.0 | 0.3 | 0.5 | 0.0 |
| Surface quality | 0.5 | 0.3 | 0.1 | 0.2 | 3.0 | 0.5 |
| Resin (elevated) | 1.0 | 0.5 | 0.3 | 0 | 2.0 | 0.0 |

Resin-specific terms (peel force ∝ max layer cross-section, suction cups) come with AM-15 and
are added to the Resin preset then.

### 5.4 GPU path (AM-13)
For each candidate direction, rasterise the mesh orthographically along `d` into an A-buffer
(per-pixel fragment lists via an atomic counter and linked lists; fragment = depth + face
class). Then sort each pixel's fragments and run the §3.1 walk in a compute pass. Reduce with
CMP-01. Face classes per direction come from one dot-product dispatch.

The CPU path is the reference. The GPU result must agree within 1 % of support volume at equal
cell size.

---

## 6. Machine profiles and slicer hand-off (AM-01, AM-10, AM-06)

### 6.1 Owner's setup (inspected on the owner's machine, 2026-10-20)
- **OrcaSlicer 2.4.0** at `D:\OrcaSlicer\orca-slicer.exe`. Its registry `DisplayIcon` is
  malformed: `D:\OrcaSlicer\D:\OrcaSlicer\orca-slicer.exe`. Detection must take the **last**
  `X:\` segment. This is a unit-test fixture.
- **Orca data dir** `%APPDATA%\OrcaSlicer\`:
  - `user\default\{machine,process,filament}\*.json` (+ `.info` sidecars);
  - `system\<Vendor>\machine\*.json`;
  - `OrcaSlicer.conf` (JSON, `app.version`).
- **Owner's printer:** user preset `Creality Ender-3 V3 KE 0.4 nozzle - Connected KCM`.
  - It has `"from": "User"` and `inherits` = `Creality Ender-3 V3 KE 0.4 nozzle` (system).
  - That inherits `fdm_creality_common`, which inherits `fdm_machine_common`.
  - The user preset overrides `printable_area` and `printable_height` = 240. The system preset
    has 220×220, height 245, and `fdm_creality_common` has 250.
  - Per-extruder arrays have **4** entries (CoPrint KCM 4-slot filament changer, one nozzle).
    The filament slot count is the length of `nozzle_diameter`, or of `extruder_colour` when
    that differs.
- **Resin:** CHITUBOX v1.2.0 (`D:\CHITUBOX\CHITUBOX.exe`) and CHITUBOX Basic 2.x.
  `%LOCALAPPDATA%\chitubox2_0\Conf\MachineConfig\<VENDOR>\<uuid>.cfgx` is JSON with
  `printerinfo.{machinewidth, machinedepth, machineheight, resolutionx, resolutiony,
  machinetype}.currentvalue`.
  - The configured machine is an **ELEGOO Mars 3 Pro**: 143.43 × 89.60 × 175 mm,
    4098 × 2560 px (≈ 35 µm). Owner confirmation is requested.
  - `addMachineList.cfgx` lists the configured machines (YAML-like).
- The `.3mf` file association points at Creality Print 7.1, so the hand-off must launch the
  configured slicer by executable path, **never** via the shell association.
- PrusaSlicer is also installed (`%APPDATA%\PrusaSlicer`, INI bundles).

### 6.2 Orca profile resolution (AM-10)
1. Search roots, in order:
   - `user\*\machine\` (all user ids; `default` first);
   - `system\*\machine\`;
   - `<install>\resources\profiles\*\machine\`.
2. Parse JSON with `serde_json`. Values are strings (`"220x0"`) or arrays of strings.
3. Resolve `inherits` by name through the same roots. Child keys override parent keys (whole
   value, no array merging). The depth limit is 16, and cycles are an error.
4. Map to `Machine`:
   - `printable_area` → bed polygon (rect detection when it has 4 axis-aligned points);
   - `printable_height`;
   - `bed_exclude_area`;
   - `nozzle_diameter` → nozzles / slot count;
   - `printer_model`, `name`;
   - `min_layer_height` / `max_layer_height` (first element).
   
   The key is `orca:<preset name>`. `MachineSource::Orca` keeps the preset name, which the
   hand-off and IO-04 can write as the printer preset name in the project config.
5. Presets with `"instantiation": "false"` (abstract bases) are not listed.

Prusa INI (`bed_shape = 0x0,250x0,…`, `max_print_height`, `nozzle_diameter = 0.4,0.4`) and
Chitubox `.cfgx` map the same way.

### 6.3 Hand-off (AM-06)
1. Resolve the plate items to meshes and effective transforms (`amg-features::am`).
2. Build an `amg_io` 3MF model:
   - one object per item, with the mesh in body coordinates;
   - build-item transform = `translate(−slicer_origin) · T`;
   - object name = body name;
   - per-object extruder = item filament slot;
   - per-object colour from the body appearance;
   - unit = millimetre;
   - plate thumbnail when RND-09 is available.

   With `SlicerFlavor::Orca` / `Prusa`, IO-03/IO-04 add the per-object extruder metadata the
   slicer reads (see S16 requests to S12).
3. Write to `<handoff dir>\<design>-<plate>-<yyyymmdd-hhmmss>.3mf`. The default dir is
   `%LOCALAPPDATA%\Amalgam\handoff`; the 20 newest files are kept. Write to `.tmp` and rename.
4. Launch with `std::process::Command::new(exe).args(args with {file} substituted)`:
   - spawned detached, never waited on;
   - on Windows with `CREATE_NEW_PROCESS_GROUP | DETACHED_PROCESS`, so closing Amalgam does not
     kill the slicer.

   Orca's documented CLI is `orca-slicer [input files…] [options…]`, so a bare path opens the
   file in the GUI. Chitubox takes the file path as its first argument (verified in AM-06).
5. Chitubox format: 3MF if the AM-06 check shows Chitubox imports our 3MF with positions;
   otherwise STL (the plate's items merged in plate coordinates). Chitubox re-places objects on
   import anyway, so STL is no real loss for resin.

---

## 7. References
- B. Ezair, F. Massarwi, G. Elber. *Orientation analysis of 3D objects toward minimal support
  volume in 3D-printing.* Computers & Graphics 51 (2015) 117–124.
- X. Zhang, X. Le, A. Panotopoulou, E. Whiting, C. C. L. Wang. *Perceptual models of preference
  in 3D printing direction.* ACM TOG 34(6), 2015.
- J. Vaněk, J. A. G. Galicia, B. Beneš. *Clever Support: efficient support structure generation
  for digital fabrication.* CGF 33(5), 2014. (AM-08)
- R. Schmidt, N. Umetani. *Branching support structures for 3D printing.* SIGGRAPH 2014 Studio.
  (AM-08)
- P. Delfs, M. Tows, H.-J. Schmid. *Optimized build orientation of additive manufactured parts
  for improved surface quality and build time.* Additive Manufacturing 12 (2016).
- P. M. Pandey, N. V. Reddy, S. G. Dhande. *Improvement of surface finish by staircase machining
  in fused deposition modeling.* JMPT 132 (2003); K. Thrimurthulu, P. M. Pandey, N. V. Reddy.
  *Optimum part deposition orientation in FDM.* IJMTM 44 (2004).
- M. Langelaar. *An additive manufacturing filter for topology optimization of print-ready
  designs.* SMO 55 (2017). (S14 GEN-06 overhang constraint uses the same α convention.)
- E. K. Burke et al. *Complete and robust no-fit polygon generation for the irregular stock
  cutting problem.* EJOR 179 (2007). (AM-05)
- C. Schranz, *Tweaker-3* (GitHub `ChristophSchranz/Tweaker-3`), `MeshTweaker.py`.
- OrcaSlicer `src/libslic3r/Orient.cpp`, `src/libslic3r/SLA/Rotfinder.cpp`,
  `src/libslic3r/PrintConfig.cpp` (AGPL-3.0). Read for behaviour; **no code is copied**.
- OrcaSlicer wiki, *CLI mode* (`orca-slicer [input files...] [options...]`).
