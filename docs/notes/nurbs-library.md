# `amg-nurbs` — design note

| | |
|---|---|
| **Spec** | [S20](../specs/S20-kernel-hardening.md) (KH-07, KH-12..KH-15) |
| **Decides** | build vs reuse for the NURBS layer (ADR-013); data model; algorithms; accuracy contract |
| **Produced by** | S20 refinement (2026-10-20) |
| **Needed before** | KH-07 |
| **Status** | Written (build decision taken with evidence; revisit only if §2's facts change) |

## 1. What the crate must do

`amg-nurbs` is the pure-Rust surface *construction* layer (ADR-013). OCCT keeps topology,
sewing, booleans and validation. The first consumer is the loft builder (KH-08, KH-18). Later
consumers are the Rust fillet engine (KH-10, Phase 7) and SubD→NURBS (S09 FRM-06).

Required capabilities, in dependency order:
1. B-spline / NURBS curves and tensor-product surfaces (2D and 3D control points, rational or
   not), with validation, evaluation and derivatives.
2. Knot operations: insertion, refinement, removal, degree elevation, Bézier decomposition,
   split, join, reverse and reparametrisation.
3. Exact conversion of analytic geometry (lines, circles, ellipses, planes, cylinders, cones,
   spheres, tori, revolutions, extrusions).
4. Fitting: parametrisation, global interpolation with end derivatives, constrained
   least-squares approximation, approximation to a tolerance, and approximation onto a given
   knot vector. Also arc length and closest point.
5. Compatibility of section curves: exact (degree elevation + knot union) when it stays
   small, approximated onto a shared knot vector otherwise (Piegl & Tiller 2002).
6. Skinning with end conditions (free, direction, G1 tangent, G2 smooth, point/apex, with
   weight), plus multi-patch skinning that shares v-parametrisation across edge chains.
7. Gordon surfaces from a curve network (rails), with network sorting and reparametrisation.
8. Rotation-minimising frames along a centerline.
9. Continuity and deviation measurement (G0/G1/G2 across boundaries, curve-to-surface).

Out of scope: trimming and B-rep topology (OCCT), surface–surface intersection (Phase 7,
KH-11), tessellation (OCCT `BRepMesh` after conversion in `amg-brep`), and rendering.

## 2. Build vs reuse (crates.io, checked 2026-10-20)

| Crate | Version / last release | Licence | Math types | Has | Missing for us / problems |
|---|---|---|---|---|---|
| `curvo` | 0.3.3 / 2026-10-19 (99 releases since 2024-04, 0.x API churn) | MIT | nalgebra, generic over `T` and `DimName` | eval, knot refine/remove, degree elevation, interpolation, loft, sweep, revolve, closest point, SSI, curve booleans, tessellation | Loft does **exact knot-vector union** (`try_unify_curve_knot_vectors`), the pole explosion S20 wants to avoid. No loft end conditions, no Gordon, no constrained least squares, no approximation to tolerance. `anyhow::Result` in the library API (AGENTS.md wants `thiserror`). About 430 `unwrap()`s outside tests. Pulls `geo`, `spade`, `argmin`, `robust`, `gauss-quad`. nalgebra types in the public API conflict with ADR-006 (glam only across crates). |
| `truck-geometry` | 0.5.0 / 2024-09-20 (repo active, crates.io stale for 2 years) | Apache-2.0 | cgmath (`truck-base`) | B-spline/NURBS, knot add/remove, degree elevation, `syncro_knots`, homotopy | No skinning with end conditions, no Gordon, no LSQ fitting API. Mutating `&mut Self` builders that panic on bad input. cgmath is a third math library. |
| `monstertruck-geometry` | 0.4.1 / 2026-09-19 (fork of truck, since 2026-03) | Apache-2.0 | cgmath | `skin`, `gordon`, interpolation, cubic approximation, rolling-ball fillet approximation | `gordon` puts intersection parameters on a uniform grid and builds the tensor term at degree 1, which is only correct for special networks. `skin`/`gordon` `assert!`/`expect` instead of returning errors. Small user base (2.5 k downloads). cgmath. |
| `brepkit-math` | 4.2.2 / 2026-10-15 | **AGPL-3.0-only** | own | NURBS, predicates | Licence: allowed by ADR-012 only behind a feature, which makes no sense for a core crate. Not considered. |
| `bspline`, `splines`, `enterpolation` | 1.1.0 (2022) / 5.0.0 / 0.3.0 | MIT / BSD-3 / MIT-Apache | generic | interpolation splines | No NURBS operations. |

Reference implementations to read (not to depend on):
- **TiGL** (DLR, Apache-2.0, active): `CTiglGordonSurfaceBuilder`, `CTiglInterpolateCurveNetwork`,
  `CTiglCurveNetworkSorter`, `CTiglBSplineApproxInterp`, `CTiglBSplineAlgorithms`. This is the
  reference for Gordon networks with reparametrisation (Siggel et al. 2019). Its
  `tests/TestData/CurveNetworks/` (Apache-2.0) is a ready-made network corpus for KH-15.
- **OCCT ≥ 8.0.0p1** `GeomFill_Gordon` / `GeomFill_NetworkSurface` (reworked in 8.0.0p1 and
  8.0.1, default `ApproximationMode::ExactOnly`). Useful as a differential oracle and as an
  extra fallback in `amg-brep`, if BREP-01 pins 8.0.1 (see the S20 report).
- **tinynurbs** (BSD-3, C++), **verb** (MIT, Haxe; curvo is a port of it), and OCCT
  `GeomFill_AppSurf` / `BRepFill_CompatibleWires` (what `ThruSections` does today).

**Decision: build `amg-nurbs` ourselves**, following *The NURBS Book* algorithms, on
`amg_core` glam f64 types with `thiserror` errors.
- The algorithms we need most (approximate compatibility, end-conditioned skinning, correct
  Gordon) are missing or wrong in every candidate, so a dependency would still need a fork.
- Determinism, serde and error conventions must match the workspace.
- The P&T core (eval, knots, elevation) is about 1.5 k lines of well-specified code. That is
  cheap compared with adapting a foreign math stack.

**Reuse as test oracles:**
- `curvo = "=0.3.3"` (MIT) is a **dev-dependency** of `amg-nurbs` for differential tests of
  evaluation, knot refinement, degree elevation and closest point. It never appears in a
  normal build.
- OCCT is the oracle in `amg-brep` tests (KH-16: `Geom_BSplineSurface` evaluation;
  `GeomFill_Gordon` on the TiGL networks).

## 3. Data model

- Control points are `Point2` / `Point3` (`amg_core` glam aliases) behind a small sealed trait
  `CtrlPt`. Rational weights are stored **separately** (`Option<Vec<f64>>`) rather than
  homogeneous. This matches OCCT's `Geom_BSplineCurve(poles, weights, knots, mults, degree)`,
  so conversion is a copy. Homogeneous `(w·P, w)` is used only inside algorithms.
- Knots are a flat, validated, non-decreasing `Vec<f64>` (`KnotVector`). It converts to and
  from OCCT's (knots, multiplicities) with a tolerance. Curves are always **clamped**
  (open). Closed sections are clamped curves whose end points coincide. "Smooth closed" is
  made by periodic interpolation followed by clamping (P&T §12.2), so OCCT always sees
  non-periodic splines. That avoids OCCT periodic-knot edge cases.
- Maximum degree is 25 (`Geom_BSplineCurve::MaxDegree()`). Loft output uses degree ≤ 5 in v
  and the section degree (≤ 9 after elevation, normally 3) in u.
- Surfaces store poles in a row-major `Grid<P>` (`nu × nv`, index `i·nv + j`, `i` along u).
- All types are `Clone + Debug + PartialEq + Send + Sync + serde`. serde lets KH-01 failure
  dumps and the corpus carry amg-nurbs inputs as JSON.

## 4. Algorithms (P&T = *The NURBS Book*, 2nd ed., 1997)

| Capability | Algorithm | WP |
|---|---|---|
| span search, basis, basis derivatives | A2.1 FindSpan (binary search, last span closed at `u = u_max`), A2.2 BasisFuns, A2.3 DersBasisFuns | KH-07 |
| curve point / derivatives | A4.1 / A3.2 non-rational, A4.2 RatCurveDerivs (via homogeneous derivatives) | KH-07 |
| surface point / derivatives | A4.3 / A3.6, A4.4 RatSurfaceDerivs | KH-07 |
| conics, analytic surfaces | A7.1 MakeNurbsCircle (≤ 90° per segment, `w = cos(θ/2)`), §7.6 ellipse, A8.1 MakeRevolvedSurf, §8.2–8.5 extrusion, ruled surface, cylinder, cone, sphere, torus | KH-07 |
| knot insertion / refinement | A5.1 CurveKnotIns, A5.3 SurfaceKnotIns, A5.4 RefineKnotVectCurve, A5.5 RefineKnotVectSurface | KH-12 |
| Bézier decomposition | A5.6 DecomposeCurve, A5.7 DecomposeSurface | KH-12 |
| knot removal | A5.8 RemoveCurveKnot with the TOL bound of eq. 5.30 (error bound in model units, rational-weight scaled) | KH-12 |
| degree elevation | A5.9 DegreeElevateCurve (Bézier-segment method), A5.10 surface | KH-12 |
| split / join / reverse | insertion to full multiplicity; join by knot concatenation with G0 check; reverse by knot mirror | KH-12 |
| parametrisation | uniform, chord length (eq. 9.4), centripetal (eq. 9.6, Lee 1989), default centripetal | KH-13 |
| interpolation knots | averaging (eq. 9.8); with end derivatives the variants of §9.2.2 (eq. 9.21–9.22) and §9.2.3 (second derivatives) | KH-13 |
| global interpolation | A9.1 GlobalCurveInterp (banded LU), with D1 / D2 end constraints as extra rows | KH-13 |
| least-squares approximation | §9.4.1 (eq. 9.63–9.67) with knots from eq. 9.68–9.69; constrained variant §9.4.2 A9.6 WCLeastSquaresCurve (Lagrange multipliers: fixed end points, end derivatives, data weights) | KH-13 |
| approximation to tolerance | start from `n = degree+1` poles, add knots in the span with largest error (bisect span, deterministic tie-break by lowest index), with Hoschek (1988) parameter correction after each solve, until `max_err ≤ tol` or `max_poles` | KH-13 |
| approximation onto given knots | sample the source curve (8 samples per target span, plus every source knot), solve constrained LSQ with fixed end points and optional end tangents, then 3 Hoschek corrections | KH-13 |
| arc length / inverse | adaptive Gauss–Legendre (5-point) per Bézier span; inverse by safeguarded Newton | KH-13 |
| closest point | P&T §6.1 Newton (eq. 6.3–6.8) seeded from the nearest control-polygon / grid sample, with domain clamping and a bisection fallback | KH-13 |
| compatibility | §10.3 (degree elevation to max, normalise domains, knot union) when the union stays ≤ `exact_pole_limit`, otherwise approximate onto a shared knot vector built by averaging the curves' parameter distributions (Piegl & Tiller, "Surface skinning revisited", *Vis. Comput.* 18, 2002) | KH-14 |
| skinning | §10.3: v-parameters averaged over control-point columns (centripetal by default), v-knots by averaging, each column interpolated with A9.1 plus end derivatives | KH-14 |
| Gordon | Gordon (1969) boolean sum `S = S_profiles + S_guides − T` after TiGL-style network preparation (§6) | KH-15 |
| rotation-minimising frames | double reflection (Wang, Jüttler, Zheng & Liu, *ACM TOG* 27(1), 2008) | KH-15 |

All dense solves use `nalgebra` **inside** the crate (ADR-006 allows it). Interpolation
systems are banded (bandwidth `p`) and use a banded LU with partial pivoting. LSQ normal
equations use Cholesky, falling back to QR when the condition estimate exceeds 1e12. Constraint
blocks use the Lagrange KKT system of A9.6, solved with LU.

## 5. Skinning and end conditions (KH-14)

### 5.1 Sections, patches and shared v-parametrisation
A loft section is either a **curve chain** (segments already matched by KH-06, so every curve
section has the same segment count and matching segment `k` across sections) or a
**point** (apex, only first or last).

`loft_surfaces` builds **one patch per segment index** (one lateral face per edge chain, as
S20 §Design requires):
1. For each segment index `k`, make the `k`-th segments of all sections compatible (§5.2).
2. Compute **one** set of v-parameters for the whole loft. Use the centripetal average over
   all columns of all patches, where each section's representative point set is the
   concatenation of its segments' poles. Compute one v-knot vector (averaging) from it.
3. Interpolate every control-point column of every patch with those same v-parameters, v-knots
   and degree.

Because patches `k` and `k+1` share the section vertex between them, the last column of `k`
and the first column of `k+1` interpolate the same points with the same basis. Their boundary
iso-curves are then **bit-identical**, so sewing has nothing to fix. End-derivative constraints
for those shared corner columns are computed **once per section vertex** and given to both
patches (§5.3). Computing them per patch from the left and right section tangents would open a
gap.

### 5.2 Compatibility rule
- Elevate to the maximum degree. Map every curve's domain linearly to `[0, 1]`. The KH-06
  matching has already made parameter ↔ arc length roughly proportional per segment.
- Union the knots (tolerance 1e-12 on `[0,1]`). If the resulting pole count is
  `≤ exact_pole_limit` (default `max(64, 2 × max input poles)`), use the exact curves
  (`CompatMethod::Exact`). Rational curves (arcs) stay exact on this path.
- Otherwise use `CompatMethod::Approximated`:
  - Build a shared knot vector from the merged normalised parameter distributions (averaging,
    eq. 9.8). Start at `n = max input poles` and grow until every curve fits within `fit_tol`
    (default **1e-5 mm**) or `max_poles` (default 200) is hit, which is an error.
  - End points are interpolated exactly. End tangents are constrained when the segment meets
    its neighbour G1 (tangent-merged chains), so G1 across patches survives approximation.
  - The approximation is non-rational. Least squares is affine-invariant: a planar section
    gives a planar approximation up to rounding, so planar caps stay planar.

### 5.3 End conditions
`EndCondition` applies to the first or last section. It turns into end-derivative rows for
every column's v-interpolation. The skin's cross-boundary derivative is
`S_v(u, 0) = Σ_i N_i(u) · D_i`. Target derivatives are therefore given as a field, sampled at
the section's Greville abscissae plus 4 samples per span. The per-column `D_i` (and `D2_i`) come
from fitting that field by LSQ in the section's u-basis.

| Condition | Meaning | `D_i` (and `D2_i`) |
|---|---|---|
| `Free` | natural interpolation | none |
| `Direction { dir, weight }` | leave the section along `dir` (Fusion "Direction"; the builder sets `dir` = profile normal rotated by the dialog angle) | `D = dir · weight · L`, where `L` is the mean chord to the next section |
| `Tangent { field, weight }` | G1 to an adjacent face | `field.d1(u, p, du)` lies in the face tangent plane, perpendicular to `du`, oriented toward the next section; magnitude `weight · L` |
| `Smooth { field, weight }` | G2 to an adjacent face | as `Tangent`, plus `D2` whose normal component equals `κ_n(D)·|D|²` (normal curvature of the face in the D direction); tangential part of `D2` from natural interpolation |
| `PointTangent { normal, weight }` (apex only) | dome-like apex tangent to the plane ⊥ `normal` | `D_i` = projection onto that plane of the chord from the apex to column `i` of the next section, times `weight` |

`CrossBoundaryField` is a trait, so `amg-nurbs` stays OCCT-free. `amg-brep` implements it for a
face by evaluating the OCCT surface normal and second fundamental form (KH-08).

Degree in v: default 3, reduced to `n_sections − 1 + n_end_constraints`. With exactly two
sections and no end conditions the result is ruled (degree 1), which is what users expect.

## 6. Gordon surfaces (KH-15)

Input: profiles `f_i(u)` (sections) and guides `g_j(v)` (rails). Preparation follows
TiGL/Siggel 2019:
1. **Sort the network.** Compute all profile × guide intersections (closest-point pairs with
   distance ≤ `net_tol`, default 1e-4 mm). Order profiles along the guides and guides along
   the profiles. Flip reversed curves. Reject a pair that misses (`NetworkError::NoIntersection
   { profile, guide, distance }`) or a curve that crosses another twice
   (`NetworkError::MultipleIntersections`).
2. **Reparametrise** every curve so its intersection parameters land on the **common** values
   `ū_j` (mean over profiles) and `v̄_i` (mean over guides). Use a monotone piecewise-cubic
   (Fritsch–Carlson) parameter map, then approximate the reparametrised curve onto a knot vector
   containing `ū_j` within `fit_tol`. Without this step `S_profiles` and `S_guides` disagree at
   the nodes and the boolean sum does not interpolate the network. This is the defect in
   `monstertruck-geometry::gordon`.
3. **Make compatible** each family (§5.2) and **move intersection points** to the average of
   the profile and guide point, so `T` is consistent.
4. **Skin** profiles over `v̄_i` and guides over `ū_j` with the same interpolation degree as
   the tensor term. **Interpolate the grid** of intersection points with a tensor-product
   B-spline on the same parameters and degrees. `T` must use the same scheme as both skins,
   or the identity `S|_{network} = network` fails.
5. Make the three surfaces compatible (degree elevation + knot union, exact). Then `S = S_p +
   S_g − T` pole-wise.

Closed networks (closed profiles) use closed skins and a periodic-then-clamped `T`. Rational
inputs are approximated to non-rational within `fit_tol` first, except exact circle networks,
which stay exact if every family already shares weights. Output reports `max_network_error`,
the maximum distance from input network curves (sampled) to `S`.

## 7. Centerline lofts and frames (KH-15)

`rotation_minimizing_frames` uses double reflection at caller-chosen parameters (default 64
per Bézier span), starting from `r0`. Tests: planar curves keep the frame normal constant; the
helix total rotation against Frenet matches `−∫τ ds` within 1e-6 rad (1000 steps).

For a centerline loft:
- Express each section in the RMF frame at its centerline foot point (closest point).
- Make the local section curves compatible.
- Insert `k` intermediate sections (default 4 per gap) at centerline parameters by
  interpolating the **local** pole sets with the same v-scheme as the skin, then map them back
  to world space with the frames there.
- Skin through original plus intermediate sections. The originals stay interpolated exactly.

Twist along the centerline is the RMF's natural twist. The builder can add a linear twist
correction so the end frame matches the last section's orientation.

## 8. Accuracy contract

`D` = bounding-box diagonal of the input poles (mm). `ε = 1e-12 · max(1, D)`. "Samples"
means 1000 deterministic parameters (stratified with a fixed-seed jitter).

| Operation | Target |
|---|---|
| Basis functions | P&T Ex. 2.3 (`U = {0,0,0,1,2,3,4,4,5,5,5}`, `p = 2`, `u = 5/2`): `N₂,₂ = 1/8`, `N₃,₂ = 6/8`, `N₄,₂ = 1/8` to 1e-15; partition of unity to 1e-14 on all samples |
| Evaluation | rational circle `|C(u)| − r ≤ 1e-14 · r`; sphere/torus surface points on the analytic surface ≤ 1e-13 · R; agrees with curvo (dev oracle) ≤ ε |
| Derivatives | analytic circle/helix tangents and curvatures ≤ 1e-12 relative; finite-difference check (central, h = 1e-6) ≤ 1e-6 relative |
| Insertion, refinement, elevation, decomposition, split + join | shape-preserving: max sample deviation ≤ ε; parameter mapping unchanged |
| Knot removal with `tol` | max deviation ≤ `tol` (eq. 5.30 bound), checked on samples |
| Interpolation | data points reproduced ≤ ε; end derivatives within 1e-10 relative |
| LSQ / to tolerance | reported `max_error` equals measured max (2000 samples + data points) within 1 %; `approximate_to_tolerance` result ≤ `tol` or `ToleranceNotMet` |
| Compatibility | `Exact`: each output ≡ input ≤ ε; `Approximated`: each output within `fit_tol` (default 1e-5 mm) |
| Skinning | sections reproduced at their v-parameter ≤ ε (exact path) / ≤ `fit_tol`; G1 end condition: angle between `S_v` and target ≤ 1e-8 rad on 100 boundary samples; G2: normal-curvature mismatch ≤ 1e-6 relative |
| Multi-patch skin | shared boundary columns bit-identical; G1 across tangent-merged segments ≤ 1e-8 rad |
| Gordon | every network curve reproduced within `max(fit_tol, ε)`; TiGL networks within 1e-4 mm of OCCT `GeomFill_Gordon` (KH-16 cross-check, when available) |
| Closest point | distance ≤ sampled-minimum distance + 1e-9 mm, for 10 k random queries (proptest) |
| RMF | orthonormal to 1e-12; planar curve normal drift ≤ 1e-12; helix ≤ 1e-6 rad |

## 9. Determinism, errors and performance

- **Determinism:** no `HashMap` iteration in results, no parallel floating-point reductions,
  and fixed tie-breaks (lowest index). The same input gives bit-identical output on one build.
  This is a proptest: run twice and compare `to_bits`.
- **Errors:**
  - `NurbsError` (`thiserror`, `#[non_exhaustive]`) implements `amg_core::Diagnostic`.
  - Nothing panics on user-reachable input. Invariants are checked in constructors, and
    `new_unchecked` is crate-private.
  - Network errors name the offending curve indices, so the loft dialog can highlight them.
- **Cancellation:** every operation is bounded by `max_poles` and iteration caps, so none
  needs a cancel token. The slowest target (Gordon 8×8 with reparametrisation) is < 50 ms.
  `amg-brep` checks cancellation between calls.

| Benchmark (criterion, owner machine, single thread) | Target |
|---|---|
| cubic curve point (A4.1) | < 60 ns |
| bicubic surface point + first derivatives | < 250 ns |
| rational surface point + normal | < 400 ns |
| refine 20 knots into a 50-pole curve | < 10 µs |
| degree elevation 3→5 of a 50-pole curve | < 20 µs |
| interpolate 200 points, cubic | < 100 µs |
| `approximate_to_tolerance` 1000 points to 1e-5 mm | < 5 ms |
| compat + skin, 6 sections × 8 segments, exact path | < 2 ms |
| compat + skin, 6 sections, approximated path | < 10 ms |
| Gordon 5 × 5 network | < 20 ms |

These keep the loft preview (S04: < 100 ms) dominated by OCCT face building and tessellation,
not by surface math.

## 10. References
- L. Piegl, W. Tiller, *The NURBS Book*, 2nd ed., Springer 1997 (algorithm numbers above).
- L. Piegl, W. Tiller, "Surface skinning revisited", *The Visual Computer* 18 (2002).
- E. T. Y. Lee, "Choosing nodes in parametric curve interpolation", *CAD* 21 (1989) (centripetal).
- J. Hoschek, "Intrinsic parametrization for approximation", *CAGD* 5 (1988) (parameter correction).
- W. J. Gordon, "Spline-blended surface interpolation through curve networks", *J. Math. Mech.* 18 (1969).
- M. Siggel, J. Kleinert, T. Stollenwerk, R. Maierl, "TiGL: an open source computational
  geometry library for parametric aircraft design", *Math. Comput. Sci.* 13 (2019); code
  github.com/DLR-SC/tigl (Apache-2.0).
- W. Wang, B. Jüttler, D. Zheng, Y. Liu, "Computation of rotation minimizing frames",
  *ACM TOG* 27(1) (2008).
- F. N. Fritsch, R. E. Carlson, "Monotone piecewise cubic interpolation", *SIAM J. Numer. Anal.* 17 (1980).
