# Inspection and construction geometry (S05)

**Status:** Written (S05 refinement, 2026-10-20). Needed before INS-01, INS-02, CON-01.
**Spec:** [`docs/specs/S05-construction-inspection.md`](../specs/S05-construction-inspection.md)

This note fixes the algorithms, conventions and tolerances behind measurement, section caps,
mass properties, analysis fields and construction geometry. Every WP's Done-when in S05 refers
to the conventions here, so implementers don't have to guess a sign or an angle range.

## 1. Where the code lives

| Code | Crate / module | Lane | Why |
|---|---|---|---|
| Measurement math, mass properties, section caps, analysis fields | `amg-features::inspect` (new module) | A | Needs `amg-mesh` now and `amg-brep` later. Domain crates may not depend on each other, and `amg-features` may depend on all of them. The module holds plain functions over domain types with no `amg-doc` types in their signatures, the same pattern as `amg-features::mesh::{shell, wrap}` (S07). `amg-script` and `amg-cli` reach it through `amg-features`. |
| Document glue: the section analysis `ExtRecord`, resolving `EntityRef` → geometry | `amg-features::inspect::doc`, `amg-features::construct` | A | Needs `amg-doc` types. |
| Construction features and construction math | `amg-features::construct` | A | Already lane A (roadmap). |
| Measure / section / properties / analysis tools, panels and overlays | `amg-ui::inspect` | A | Lane A owns it (roadmap). Built only on the lane-D framework API. |
| Construct commands and display | `amg-ui::construct` (proposed), fallback `amg-ui::inspect::construct` | A | See S05 report: the lane-A row needs `amg-features::inspect` and `amg-ui::construct` added. |

`amg-render` never sees `amg-mesh` or `amg-doc` types. `amg-ui::inspect` converts results into
the plain f32/f64 buffers RND-06/07/08 take.

## 2. Measurement on meshes (INS-01)

### 2.1 Operands become primitive sets
Every operand is lowered into world-space primitives:

| Element | Primitive set | Notes |
|---|---|---|
| free point, `Vertex`, `SurfacePoint` | 1 point | `SurfacePoint` = barycentric point on a face |
| `Edge {a, b}` | 1 segment | a triangle edge |
| `Polyline {verts, closed}` | segments | boundary loop or feature-edge chain |
| `Faces(ids)`, `FaceGroup` | triangles + ad-hoc `Bvh` | a `Bvh` is built over the subset when there are > 64 faces |
| `Body` | triangles + the body's `MeshSpatial` BVH | built once per `MeshBody` |

Each operand's `to_world` must be **rigid**: `|det(R) − 1| ≤ 1e-9` and `RᵀR = I` within 1e-9.
A non-rigid transform returns `InspectError::NonRigidTransform`, because distances can't be
computed in local frames under scaling. Rigid transforms let queries run in the local frame of
operand A: B is mapped by `T_ab = A.to_world⁻¹ · B.to_world`.

### 2.2 Minimum distance
- **Point–set:** `MeshSpatial::closest_point` (Body), or a best-first traversal over the ad-hoc
  BVH (subset).
- **Set–set:** best-first *simultaneous* BVH traversal, as in PQP (Larsen, Gottschalk, Lin &
  Manocha 2000) and Ericson 2005 §6.5:
  - A min-heap of node pairs is keyed by the lower bound
    `dist(boxA, Aabb3::transformed(boxB, T_ab))`. The transformed box is conservative, so the
    lower bound stays valid.
  - The upper bound `U` is initialised from the closest-point distance between the two root
    centroids' nearest vertices, which is cheap and valid.
  - A pair is popped and pruned if its bound ≥ U. Otherwise the larger node is split (by
    surface-area heuristic: the node with the larger box diagonal). Leaf × leaf pairs run the
    exact primitive test.
  - Ties are broken by `(bound, a_prim, b_prim)`, so the result and its witness points are
    deterministic. The traversal is single-threaded. For two bodies with > 200 k triangles
    each, the root's 4×4 grandchild pairs run in parallel with a shared atomic upper bound
    (f64 bits stored in an `AtomicU64`, min by compare-exchange). The final witness is the
    lexicographic minimum of `(distance, a_prim, b_prim)`, so results stay deterministic.
  - Cancellation is checked every 4096 node pairs.
- **Exact triangle–triangle distance** (also used for segment/point leaves):
  1. If any of the 6 edges of one triangle intersects the other triangle
     (`amg_core::intersect::ray_triangle` with `t ∈ [0, |edge|]`), the distance is 0 and the
     witness is that point. This covers every transversal intersection. Coplanar overlap is
     covered by step 2: crossing edges give edge–edge distance 0, and containment gives
     vertex–triangle distance 0.
  2. Otherwise take the minimum over 6 vertex–triangle distances
     (`point_triangle_closest`) and 9 edge–edge distances (`segment_segment`). Ericson §5.1.9
     and §5.2.7 prove the minimum lies on one of these feature pairs.
- `intersecting = distance ≤ tol.linear`.
- `contained`: if both operands are watertight bodies, the distance is > 0, and A's first vertex
  has winding number > 0.5 in B (or the reverse), the report says one body is inside the other.
  The value is still the *surface* distance, as in Fusion.
- **Requested from S07 (MESH-03, additive):** read-only `Bvh` node access (`root`, `node`,
  `prim`). **Fallback** if S07 declines: `amg-features::inspect` keeps a private ≈ 300-line f64
  BVH for subsets and pair traversal, and uses `MeshSpatial::closest_point` only for points.

### 2.3 Planar regions and smart selection
- **Plane fit** of a triangle set:
  - normal `n` = normalised area-weighted sum of face normals (exact for truly planar sets);
  - origin = area-weighted centroid;
  - `max_dev = max |n·(v − o)|` over its vertices.
  - The region is planar iff `max_dev ≤ planar_tolerance` (default **1e-3 mm**, which covers
    f32 STL quantisation up to ≈ 4 m from the origin).
  - If the area-weighted normal has norm < 1e-12 (for example a closed body), the set is not
    planar.
- **`grow_planar_region(seed)`** (default on in the measure tool; Alt-click picks a single
  triangle):
  - BFS over manifold edges, in ascending face-id order, adding a neighbour iff its normal is
    within `max_normal_angle` (default **0.5°**) of the seed normal and all its vertices are
    within `planar_tolerance` of the seed plane.
  - Optionally stops at face-group borders.
  - Capped at `max_faces` (default 5 M).
  - The result is sorted and deterministic.
- **`grow_feature_chain(a, b)`:** a feature edge has a dihedral angle ≥ `feature_angle`
  (default **30°**) or is a boundary edge. From the picked edge, walk both ways while the
  current vertex has exactly one other incident feature edge. The chain is closed if the walk
  returns to its start. This gives the full rim of a drilled hole in a CAD-exported STL.

### 2.4 Circle and line fits
- **`circle_through(a, b, c)`:** circumcircle in the plane of the three points. Errors if
  collinear (`|(b−a)×(c−a)| ≤ tol.linear · max(|b−a|, |c−a|)`).
- **`fit_circle(points)`** (closed chains with ≥ 5 points):
  1. Fit a plane by PCA (3×3 symmetric eigen, `nalgebra` crate-internal per ADR-006).
     Require max deviation ≤ `planar_tolerance`.
  2. Fit the circle in 2D: Kåsa algebraic fit, then ≤ 10 Gauss–Newton iterations on the
     geometric distance (Chernov 2010, ch. 4–5).
  3. Accept iff the RMS of the geometric residual ≤ `circle_tolerance` (default **0.01 mm**)
     **and** max residual ≤ 5 × that value.
  - For a regular n-gon with vertices on a circle of radius r, the result is r exactly (up to
    rounding), so Done-when values can be analytic.
- **Straightness:** a polyline is straight iff every vertex lies within `planar_tolerance` of
  the end-to-end segment. Then it has a direction and acts as a line for angles.

### 2.5 Angle conventions
Angles use `atan2(|u×v|, u·v)`, never `acos`, so precision holds near 0 and π.

| Pair | Value | Range | `supplement` |
|---|---|---|---|
| line–line (edges, straight chains, construction axes) | angle between *undirected* lines | [0, π/2] | π − value |
| plane–plane (planar regions, construction planes) | angle between *oriented outward* normals | [0, π] | π − value |
| line–plane | `asin(|d·n|)` via atan2 | [0, π/2] | π/2 − value |

Examples:
- two adjacent faces of a cube: 90° (supplement 90°);
- opposite faces of a cube: 180° (supplement 0°);
- a 45° chamfer face against the top face: 45° (supplement 135°).

The UI shows the value and puts the supplement in a tooltip.

### 2.6 "Center" distance
Each operand has a *center*: circle center if a circle fit was accepted, else the vertex or
point, else the area or length centroid. `center_distance = |c_b − c_a|`. This plays the role of
Fusion's "center to center".

## 3. Section caps (INS-02)

### 3.1 Convention
A section plane **removes the half-space its normal points into** (`signed_distance > 0`). This
matches Fusion's arrow pointing at the material that disappears. `flip` negates the normal.
With k visible planes (k ≤ 6), the kept region is the intersection of their negative
half-spaces. Section analyses combine (S05 Open question 2).

### 3.2 Per body, per plane
1. Map the plane into body-local coordinates.
2. Call `amg_mesh::section(mesh, plane_local, snap)` to get polylines.
3. **Orient closed loops** so material lies on the **left** when the plane is viewed from its
   +normal side (in the plane frame `(x, y)`: CCW outer boundaries, CW holes).
   - If S07 guarantees this orientation (request), use it directly.
   - Otherwise, when the mesh is oriented and closed, orient each loop by a winding-number
     probe: take the midpoint of its longest segment, offset it by `ε = 1e-4 · segment length`
     to the left in the plane, and evaluate `MeshSpatial::winding_number` there. Inside
     (> 0.5) keeps the loop; otherwise reverse it.
   - If the mesh is not oriented, loops are unoriented and the fill rule falls back to
     even–odd.
4. **CDT:** insert all closed-loop segments as constraints into
   `spade::ConstrainedDelaunayTriangulation` (2.15) with `add_constraint_and_split`.
   Overlapping shells give crossing constraints, which this splits.
5. **Classify** each CDT triangle by the winding number of its centroid with respect to the
   oriented loops (sum of signed crossings). Keep it if `w ≥ 1` (non-zero, oriented meshes) or
   if `w` is odd (even–odd fallback).
   - Overlapping closed shells (common in STLs) are capped as their union. Even–odd would wrongly
     leave the overlap empty.
6. **Clip by the other planes:** clip each kept triangle against the other k−1 kept
   half-spaces (Sutherland–Hodgman on a convex polygon, then fan triangulation). Clip each
   outline polyline the same way.
7. **Output** (world space): positions, triangles, plane-local `uv` in mm (for world-anchored
   hatching), outlines, open polylines and stats.

### 3.3 Stats per plane
- `area`: Σ kept cap triangle areas, cross-checked against the shoelace sum of oriented loops in
  tests;
- `perimeter`: Σ closed outline lengths after clipping;
- `loops` and `open_loops`;
- the 2D bounding box in the plane frame.

For AM this is the layer cross-section at the given height.

### 3.4 Watertight caps
**Definition:** take the part of the mesh kept by `amg_mesh::plane_cut(keep = Negative,
cap = false)`, append the cap triangles, and weld at `snap_tolerance`. For any closed, oriented
input, the result is closed and manifold (`AnalysisReport::is_closed && is_manifold`). This is
the INS-02 test. For open input, open polylines get no cap, and the panel reports "body is not
closed: N open section curves".

## 4. Mass properties of a closed triangle mesh (INS-03)

Divergence theorem over signed tetrahedra relative to a reference point `r`, the AABB center,
which reduces cancellation. Per triangle `(a, b, c)` with `A = [a−r, b−r, c−r]` (columns) and
`d = det A`:
- **Volume:** `V = Σ d/6`.
- **First moment:** `S = Σ d/24 · (a+b+c−3r)`, so `CoM = r + S/V`.
- **Second moment (covariance about r):** `C = Σ d · A·C₀·Aᵀ`, where
  `C₀ = 1/120 · [[2,1,1],[1,2,1],[1,1,2]]` (Tonon 2004; Eberly 2002/2009).
- **Inertia** with density ρ:
  - about r: `I_r = ρ(tr(C)·𝟙 − C)`;
  - about CoM: `I_c = I_r − m(|g|²𝟙 − g gᵀ)` with `g = CoM − r` and `m = ρV`.
- **Principal moments and axes:** eigen decomposition of `I_c` (Jacobi, `nalgebra`
  internally). Axes are sorted by ascending moment, and each axis is made right-handed with its
  largest component positive (deterministic sign).
- **Sums:** Kahan–Babuška (Neumaier) compensated summation in face order. The result is
  bit-identical across thread counts because the sum is sequential over chunks fixed by face
  index, then chunk sums are combined in order.
- **Preconditions:** body watertight (`AnalysisReport::is_watertight`) and not inverted.
  Otherwise volume, mass, CoM and inertia are `None` with a reason. Area is always available.
- **Units:**
  - mass in t (canonical), displayed in g by default (owner Q1);
  - inertia in t·mm², displayed in kg·mm² (S00 unit request).

## 5. Analysis fields (INS-05, INS-06)

Per-vertex `Vec<f64>` fields go to the RND-08 scalar overlay via `amg-ui::inspect`.
- **Draft angle** for pull direction `d` (unit): `draft(v) = π/2 − atan2(|n×d|, n·d)`, with
  angle-weighted vertex normals. The range is [−π/2, π/2]:
  - positive: the face looks toward the pull;
  - 0: vertical wall;
  - negative: undercut.

  The overlay classes are `> +θ` green, `|draft| ≤ θ` yellow and `< −θ` red, with θ = 1°
  by default. S16 overhang is the same field with `d` = −build direction; `amg-ui::am` may call
  `inspect::fields::draft_angle`.
- **Accessibility** from direction `d`: a vertex is accessible iff `n·d > 0` and the ray
  `p + ε n + t d`, `t ∈ (0, ∞)`, `ε = 1e-4 · bbox diagonal`, hits nothing (`MeshSpatial::ray_cast`).
  The field is 1 or 0.
- **Discrete curvature** on meshes:
  - mean curvature from the cotangent Laplace–Beltrami operator, and Gaussian curvature from
    the angle deficit, both over mixed Voronoi areas (Meyer, Desbrun, Schröder & Barr 2003);
  - principal curvatures `κ₁,₂ = H ± √max(H² − K, 0)`;
  - boundary vertices are excluded (NaN, drawn grey).
- **B-rep curvature (INS-06):** exact principal curvatures from S02 at tessellation vertices'
  `(u, v)`.
  - **Curvature comb:** sample κ(t) and the principal normal N(t) at ≥ 64 parameter values per
    edge, uniform in arc length, and draw the comb tip at `p − s·κ·N`. The scale s is
    auto-chosen so that the max tooth is 15 % of the curve length.
  - **Minimum radius:** `r_min = 1/max(|κ₁|, |κ₂|)` per face. Sample the tessellation
    vertices, then refine with ≤ 20 Newton steps on |κ|. Faces with radius below the threshold
    are flagged.

## 6. Construction geometry conventions (CON-01, CON-02)

All construction math is pure f64 over `RefGeom` (see S05 Interfaces), so it can be tested
without OCCT. The tolerance is the design `Tolerance` (`linear` 1e-6 mm, `angular` 1e-9 rad).
Every output frame is deterministic. When a plane's in-plane x-axis is not fixed by its inputs,
it is `Plane::from_normal(origin, n)` (S00), so the same input gives the same frame.

| Variant | Inputs | Result | Conventions / errors |
|---|---|---|---|
| Offset plane | plane P, distance d | `P.origin + d·P.normal`, same frame | d may be negative |
| Plane at angle | line L (+ optional zero plane Z) | plane containing L, rotated by θ about L.dir (right-hand rule) from Z | Z defaults to the line's `context_plane`. Without one, Z contains L and the first of world Z, Y, X that is not parallel to L. Error `Parallel` if Z doesn't contain L (`dist > tol`) |
| Tangent plane | cylinder / cone / sphere face F, reference plane R, angle θ | tangent to F | For a cylinder, `u₀` = R.normal projected ⟂ to the axis (error if parallel to the axis). The contact direction is u₀ rotated by θ about the axis, the plane normal is that direction, and the plane passes through `axis point + r·dir`. A cone uses the same direction at the reference-plane height. A sphere uses R.normal rotated by θ about R.x. Other surfaces give `Unsupported` |
| Midplane | two planes, or two points | parallel planes (normals within `angular`, same or opposite): halfway, normal = first plane's normal. Otherwise: bisector containing the intersection line, normal `n₁ − n₂` normalised for planes facing each other (n₁·n₂ < 0), else `n₁ + n₂`. Two points: normal along b − a at the midpoint | `Coincident` if identical |
| Plane through two edges | two lines | plane containing both. Parallel lines: normal `d × (p₂ − p₁)`. Intersecting lines: normal `d₁ × d₂` | `Skew { distance }` if not coplanar within `linear`. `Coincident` if they are the same line |
| Plane through three points | 3 points | `Plane::from_points(a, b, c)`. x along b − a, normal `(b−a)×(c−a)` | `Collinear` |
| Plane tangent to face at point | face F, point p | plane at `q = closest(F, p)` with normal = F's outward normal at q, x from `from_normal` | `NotOnFace { distance }` only if the closest-point query fails |
| Plane along path | curve C, position (fraction 0..1 or distance) | normal = unit tangent at t. x = principal normal if κ > 1e-12, else `from_normal` | `OutOfRange` if the position is outside [0, 1] or [0, length] |
| Axis through cylinder / cone / torus / circle | face or edge | its axis. Direction = the surface axis as stored. A circle's axis is its normal | — |
| Axis perpendicular at point | face F, point p on F | line through p along F's normal at p | `NotOnFace` if `dist(p, F) > linear` |
| Axis perpendicular to face at point | face F, point p | line through `closest(F, p)` along the normal | — |
| Axis through two planes | 2 planes | `intersect::plane_plane`. Direction `n₁ × n₂` | `Parallel` |
| Axis through two points | a, b | line a → b | `Coincident` |
| Axis through edge | linear edge / sketch line | its line, directed start → end | `NotStraight` if curved |
| Point at vertex | vertex / sketch point / construction point | that point | — |
| Point through two edges | 2 lines | `intersect::line_line` | `Parallel`, or `Skew { distance }` if the lines miss by > `linear` |
| Point through three planes | 3 planes | unique solution of the 3×3 system | `Degenerate` if `|det| ≤ angular` |
| Point at center | circle / arc / sphere / torus | its center | — |
| Point at edge and plane | curve / line, plane | the intersection. For a curve with several roots, the one with the smallest parameter | `NoIntersection` |
| Point along path | curve, position | `C(t)` at fraction or arc length | `OutOfRange` |

**Display sizing (CON-04):**
- A construction plane is drawn as a square in its own frame. The square is centred at the
  projection onto the plane of the center of the visible bodies' AABB `B`. Its side is
  `max(20 mm, 1.2 · max extent of B's 8 corners projected into the plane)`.
- An axis is drawn as a segment through the projection of B's center, with length
  `1.2 · B.diagonal()` (minimum 20 mm).
- This is a pure function, so tests can check it.

## 7. Prior art consulted
- **Fusion 360 documentation:**
  - Measure (selection filters, precision, Δx/Δy/Δz, angle, center-to-center);
  - Section Analysis (offset / angle manipulator, flip, hatch, "Analysis" browser folder);
  - Construct menu (8 plane, 6 axis, 6 point variants);
  - Physical Properties;
  - Zebra, Draft, Curvature Map, Curvature Comb, Minimum Radius, Accessibility.
- **Distance queries:**
  - Larsen, Gottschalk, Lin, Manocha, *Fast distance queries with rectangular swept sphere
    volumes*, ICRA 2000 (PQP);
  - Ericson, *Real-Time Collision Detection*, 2005, §5.1.5, §5.1.9, §5.2.7, §6.5;
  - Woop, Benthin, Wald 2013 (watertight ray/triangle, already in S00).
- **Mass properties:**
  - Mirtich, *Fast and accurate computation of polyhedral mass properties*, JGT 1996;
  - Eberly, *Polyhedral Mass Properties (Revisited)*, 2002/2009;
  - Tonon, *Explicit exact formulas for the 3-D tetrahedron inertia tensor*, J. Math. Stat. 2004.
- **Curvature:**
  - Meyer, Desbrun, Schröder, Barr, *Discrete differential-geometry operators for triangulated
    2-manifolds*, 2003;
  - Rusinkiewicz, *Estimating curvatures and their derivatives on triangle meshes*, 3DPVT 2004
    (an alternative, not used initially).
- **Circle fitting:**
  - Kåsa 1976;
  - Chernov, *Circular and Linear Regression: Fitting Circles and Lines by Least Squares*, 2010.
- **`spade` 2.15.1** (MIT OR Apache-2.0): `ConstrainedDelaunayTriangulation::add_constraint_and_split`
  splits crossing constraints (checked on docs.rs 2026-10-20). Already adopted by S07.
- **Polygon clipping:** Sutherland & Hodgman, *Reentrant polygon clipping*, CACM 1974.
