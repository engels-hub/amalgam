# S05 — Construction geometry & inspection

| | |
|---|---|
| **Status** | Refined |
| **Priority** | P0 (measure, section, properties, construction) / P1 (interference, surface analyses) |
| **Crates** | `amg-features::inspect` (new module: measurement math, sections, properties, analysis fields), `amg-features::construct` (construction features + math), `amg-ui::inspect` (tools, panels, overlays), `amg-ui::construct` (proposed; fallback `amg-ui::inspect::construct`) |
| **Lane** | A |
| **Depends on** | S00 (FND-03 geometry + `intersect`, FND-04 jobs, FND-10 units/formatting); S01 (DOC-02/03/05 features, commands, `ExtRecord`; DOC-06 naming for Phase 2 refs); S07 (MESH-02 analysis, MESH-03 BVH/queries, MESH-09 `section`, MESH-16 mesh bodies in `Evaluated`, MESH-05 for interference); S02 (BREP-02/08/09 for B-rep measure/properties/construction, Phase 2); S03 (SK-01 sketch geometry as construction input); S10 (RND-04 picking, RND-06 section planes, RND-07 overlays, RND-08 scalar overlays); S11 (UI-02/03/04/07/10, UI-05 for Phase 2 construct dialogs); S17 (MAT-01 density) |
| **Parity** | Construction & inspection section |

## Summary
Construction planes, axes and points are timeline features. The inspection tools are:
- measure;
- section analysis;
- physical properties and centre of mass;
- interference;
- surface analyses: zebra, curvature map and comb, draft, isocurves, minimum radius,
  accessibility.

Phase 1 (Mesh MVP) ships **measure, section analysis and physical properties on mesh bodies
only**, with no OCCT dependency. B-rep entities, construction geometry and sketch entities join
the same tools in Phase 2. Algorithms, sign conventions and tolerances are in the design note
[`docs/notes/inspection-geometry.md`](../notes/inspection-geometry.md). Overhang analysis lives
in S16 and reuses the RND-08 overlay path and, optionally, `inspect::fields::draft_angle`.

## Scope
**In:**
- `amg-features::inspect`:
  - `measure`: operands, per-entity properties, minimum distance, angles, centres;
  - smart selection: planar region growth, feature-edge chains;
  - circle and plane fits;
  - result formatting;
  - `props`: mass properties of closed meshes, B-rep via S02 later;
  - `section`: section caps, multi-plane clipping, section stats;
  - `fields`: draft, accessibility, discrete curvature;
  - `interfere`: interference;
  - `doc`: the `SectionAnalysis` extension record and resolution helpers.
- `amg-features::construct`:
  - `ConstructPlane` / `ConstructAxis` / `ConstructPoint` features, with all Fusion variants;
  - `RefGeom` and the pure construction math;
  - `EntityRef` → `RefGeom` resolution;
  - display sizing.
- `amg-ui::inspect`:
  - the Measure tool, Section Analysis tool, Physical Properties panel, Interference dialog and
    analysis overlays (draft, curvature, zebra, accessibility, minimum radius, comb);
  - the browser "Analysis" folder entries for section analyses;
  - display of the centre of mass.
- `amg-ui::construct`: Construct menu commands and the construction-specific preview. Dialogs
  come from UI-05 schemas.

**Out:**
- Picking, clip-plane rendering, overlay rendering and colormaps → S10 (RND-04/06/07/08).
  This spec defines the data it hands over.
- Tool state machine, selection, browser tree, dialog framework → S11.
- Mesh queries and section polylines → S07. B-rep queries → S02.
- Overhang / support analysis → S16. Wall thickness → S07 MESH-11.
- Python and CLI wrappers → S18 (request in the report).
- Material densities → S17.

## Design

### 1. Code placement and layering
- **Measurement math belongs to `amg-features::inspect`.** It is lane A, and the
  `amg-features::inspect` module is proposed for the lane-A row of the roadmap.
  - It needs `amg-mesh` now and `amg-brep` later. A domain crate cannot depend on another
    domain crate, but `amg-features` may depend on all of them.
  - All functions are plain functions over `amg-core` / `amg-mesh` / `amg-brep` types, with no
    `amg-doc` types except in `inspect::doc`. They are unit-testable without a document, the
    same pattern as `amg-features::mesh::{shell, wrap}` in S07.
  - `amg-script` and `amg-cli` reach them through `amg-features`.
  - `amg-ui::am` (S16) may call `inspect::fields`.
- `amg-ui::inspect` only does these things:
  - collects picks;
  - builds `Operand`s from `Evaluated`;
  - spawns jobs;
  - formats results;
  - feeds S10 plain buffers.

  `amg-render` never sees `amg-mesh` or `amg-doc` types.
- Nothing in S05 runs geometry on the UI thread except operations proven < 0.1 ms
  (`measure::estimated_cost` returns `Inline`): point–point, a single triangle or edge, and
  circle-through-3-points. Everything else is an `Interactive` job (FND-04) that a newer
  selection supersedes (cancel + respawn).

### 2. Measure (INS-01 core, INS-08 tool, INS-07 B-rep/construction/sketch)
- **Operands** (Phase 1):
  - a free world point;
  - mesh elements: `Body`, `FaceGroup`, `Faces` (a picked triangle or a smart planar region),
    `Edge` (triangle edge), `Polyline` (boundary loop or feature-edge chain), `Vertex`,
    `SurfacePoint`.

  Each mesh operand carries a cheap-clone `MeshBody` and a **rigid** `to_world` (non-rigid →
  error).
- **Per-entity results:**

  | Element | Results |
  |---|---|
  | point / vertex | position |
  | edge / polyline | length, centroid, direction (if straight), circle (if a fit is accepted) |
  | faces / group | area, centroid, best-fit plane (if planar), face count |
  | body | area, volume (if watertight), world AABB, face count |
- **Pair results:**
  - minimum distance with witness points and features, Δ = b − a, `intersecting` and
    `contained` flags;
  - angle (line–line, plane–plane, line–plane, conventions in note §2.5) with its supplement;
  - centre-to-centre distance.
- **Smart selection, mesh-specific and key for STL parts:**
  - Clicking a triangle grows a coplanar region (0.5°, 1e-3 mm). Alt-click picks a single
    triangle.
  - Clicking a triangle edge on a feature edge grows the feature chain (30°). A closed chain
    gets a circle fit, so hole diameters on STLs measure in one click.
  - "3-point circle" mode: three snapped points give the circumcircle (centre, radius,
    diameter, normal).
- **Snapping** needs no BVH:
  1. RND-04 gives the picked face index and the world pick ray.
  2. The tool intersects the ray with that triangle (`intersect::ray_triangle`) in body-local
     f64.
  3. It snaps to a triangle vertex within `snap_px` pixels (default 8 px), else to an edge
     within 5 px, else keeps the surface point. Pixels convert to mm through RND-04's
     world-per-pixel at the hit depth.
- **UI (INS-08)** is a Fusion-style Measure dialog, hand-built on the lane-D framework like
  the S07 Phase 1 dialogs (QUESTIONS #22):
  - selection filter: Body / Face / Edge / Vertex / Point, plus the smart-selection toggle;
  - precision 0–6 decimals, default from `FormatSpec::for_unit`;
  - units, defaulting to the document's `UnitPrefs`;
  - Selection 1 and 2, Restart, and a results table with copy buttons.

  In the viewport, an RND-07 overlay draws a line between the witness points with a distance
  label, Δ components, an angle arc and the fitted circle. Measure is transient: no `Command`,
  nothing saved. Esc closes it. "Pin" keeps the overlay until the tool closes.
- **Formatting:**
  - `format_quantity` (S00) with the document display unit;
  - area and volume units follow the length unit (mm → mm²/mm³, in → in²/in³);
  - angles in degrees by default.

  Copy writes the displayed text.
- **Phase 2 (INS-07):** `Operand::Ref(ResolvedRef)` covers construction planes, axes and points,
  origin entities and sketch entities, through the CON-03 resolver. `Operand::Brep` covers
  faces, edges, vertices and bodies of solids.
  - B-rep–B-rep distance is exact via S02 `distance` (BRepExtrema), and length/area via S02
    mass properties.
  - Mixed mesh–B-rep tessellates the B-rep (BREP-08 fine preset) and sets
    `exactness = Tessellated { deflection }`, shown with "≈".
  - Analytic angles and radii come from `RefGeom` (a cylinder face gives radius/diameter; a
    circular edge gives a circle).

### 3. Section analysis (INS-02 core, INS-09 tool + browser, INS-11 solids)
- **Model.** A section analysis is **not a timeline feature**. It is an `ExtRecord` of kind
  `"inspect.section"`, v1, so it gets undo, save/load and `ExtChanged` events for free from S01.
  Each record holds:
  - `name`, `visible`;
  - `base`:
    - Phase 1: an origin plane, or a free plane taken from a picked planar region (stored by
      value; mesh STL faces have no persistent ids beyond group 0);
    - Phase 2: `Ref(EntityRef)` with the last resolved plane kept as a fallback, which shows a
      warning when used;
  - `offset` (mm), `tilt` (two rotations, rad), `flip`;
  - hatch style, cap colour;
  - scope: all visible bodies or a list of `BodyId`s.
- **Convention:** a plane removes the half-space its normal points into. All visible analyses
  combine (intersection of kept half-spaces), up to 6 planes (`MAX_SECTION_PLANES`, owner Q2).
- **Rendering split:**
  - The GPU clips every frame (RND-06 clip planes, so dragging stays at 60 fps).
  - Caps are geometric, computed by S05 as a job: `amg_mesh::section` polylines → oriented
    loops → `spade` CDT with `add_constraint_and_split` → non-zero winding classification
    (even–odd if the mesh is not oriented) → clipping by the other planes (note §3).
  - S05 hands RND-06 world-space cap triangles with plane-local uv in mm (world-anchored
    hatching) and outline polylines.
  - Until the cap job for the current plane position finishes, RND-06 shows its stencil cap if
    it has one, or the uncapped clip.
  - Geometric caps are exact, work for several planes and give **section statistics**: area,
    perimeter, loop counts and 2D extent. For AM, that is the layer cross-section area at a
    height.
- **Broken meshes:** open section curves are drawn as outlines without caps. The panel states
  "body is not closed: N open section curves".
- **UI (INS-09):**
  - Inspect ▸ Section Analysis: pick an origin plane or a planar region;
  - dialog with offset (expression input, slider, ± step), tilt X/Y, flip, hatch, cap colour,
    body scope;
  - OK adds the record (`Command::SetExt`, one undo entry); edits replace it;
  - browser "Analysis" folder: visibility eye, rename, edit, delete;
  - section stats panel.

  The on-canvas drag arrow comes later with UI-06 (Phase 2), added in INS-11.
- **Solids (INS-11, Phase 2):** caps come from the B-rep tessellation (BREP-08) through the same
  code. Bases can be construction planes and B-rep planar faces through CON-03.

### 4. Physical properties (INS-03 meshes, INS-10 solids + assemblies)
- **Closed meshes:** exact polyhedral mass properties (divergence theorem, Tonon / Eberly, note
  §4):
  - volume, area, mass, density used;
  - centre of mass;
  - inertia tensor about the CoM and about the world origin;
  - principal moments and axes.

  The result is deterministic, with compensated summation.
- **Density** comes from the body's material (`BodyProps.material` → S17 MAT-01). Without one,
  the design default material is used, marked "(default material)": PLA 1.24 g/cm³ is
  recommended (owner Q1).
- **Panel:** Inspect ▸ Properties, or a body context menu entry. A centre-of-mass glyph (RND-07)
  can be toggled per body.
- **INS-10** adds:
  - B-rep bodies via S02 `mass_props`;
  - per-component and per-occurrence totals (S06 transforms);
  - "Display component colours" (a per-component colour override through the lane-D scene
    API).

### 5. Interference (INS-04)
- Pairwise over the selected bodies and components, in world space:
  1. Broad phase: world AABBs.
  2. Mesh–mesh: BVH pair traversal with the exact triangle–triangle test (note §2.2) decides
     *touching* or *interfering*. The interference volume comes from Manifold intersection
     (MESH-05) when both bodies are watertight.
  3. Solid–solid: S02 `boolean(Common)` volume.
  4. Mixed: tessellate, and mark the result approximate.
- An "Include coincident faces" option (Fusion) treats contact with volume ≤ 1e-6 mm³ as
  touching.
- Results:
  - a table of pairs with volume;
  - highlighting;
  - "Create bodies" adds a `MeshCombineFeature` / S04 Combine (intersect, keep tools) per pair
    through normal commands, so it is undoable.

### 6. Surface analyses (INS-05, INS-06, INS-12)
- All analyses are per-vertex fields on the displayed tessellation, sent to RND-08 with a
  legend (note §5):
  - **INS-05:** draft (pull direction, ±θ classes), accessibility (direction, 0/1), discrete
    mesh curvature (mean, Gaussian, min, max).
  - **INS-06:** exact B-rep curvature map, curvature comb on edges and sketch curves (RND-07
    polylines), minimum radius (threshold flagging).
  - **INS-12:** zebra (RND-08 shader, normals only) and B-rep isocurves (S02 polylines, drawn as
    RND-07 lines).
- Analyses are view state: transient per session, listed in the "Analysis" folder while active.

### 7. Construction geometry (CON-01..04, Phase 2)
- **Features:** `construct.plane`, `construct.axis` and `construct.point` are S01 `Feature`s
  whose `compute` returns `FeatureOutput { construction: Some(ConstructionGeom::…) }`. Their
  `ConstructionId` equals the `FeatureId` (S01).
- **Inputs:**
  - `RefInput::Origin(OriginEntity)`: the context component's origin point, axes and planes.
    These need no feature.
  - `RefInput::Ref(EntityRef)`: B-rep faces, edges and vertices, sketch curves and points,
    other construction geometry, and mesh face groups with a fitted primitive (MESH-24, Phase 3;
    planar groups work from Phase 2 by plane fit).
- **All variants** (Fusion's Construct menu; exact conventions in note §6):
  - **Planes:** offset, at angle, tangent, midplane, through two edges, through three points,
    tangent to face at point, along path.
  - **Axes:** through cylinder/cone/torus, perpendicular at point, through two planes, through
    two points, through edge, perpendicular to face at point.
  - **Points:** at vertex, through two edges, through three planes, at centre of
    circle/sphere/torus, at edge and plane, along path.
- **Two layers:**
  - `construct::math`: pure functions over `RefGeom`, tested without OCCT.
  - `construct::resolve` (CON-03): `EntityRef` → `ResolvedRef { geom: RefGeom, context_plane }`
    through `RecomputeCtx::resolve` (S01) and S02 / S03 / S07 geometry queries.

  `resolve_plane_input` is also offered to S03 for sketch-plane resolution.
- **Health:** an unresolvable or degenerate input is a feature `Error` with a
  `Diagnostic { entities }`, so the timeline shows which reference broke. A fingerprint-only
  match (`MatchQuality::FingerprintOnly`) is a `Warning`.
- **Display:** planes are squares, axes are segments and points are markers. Sizing rules are
  in note §6 (pure `display_extent`). They are drawn by the lane-D scene adapter through RND-07
  with pick ids. Construction geometry is selectable as measure operands and section bases.

### 8. Determinism and tolerances
- Every result is bit-identical across rayon thread counts (ordered reductions; best-first
  traversals tie-broken by primitive ids, note §2.2).
- Defaults:
  - mesh queries: `Tolerance::mesh()` (1e-5 mm);
  - construction math: the design `Tolerance` (1e-6 mm, 1e-9 rad);
  - `planar_tolerance` 1e-3 mm, `circle_tolerance` 0.01 mm;
  - smart region 0.5°, feature edges 30°.

  All of these are in `MeasureOpts`.

## Interfaces
Types from S00: `Point3, Vec3, Point2, Dir3, Plane, Line3, Segment3, Ray3, Transform, Aabb3,
Aabb2, Tolerance, Length, Area, Volume, Angle, Mass, Quantity, Unit, LengthUnit, FormatSpec,
Diagnostic`. From S07: `MeshBody, IndexedMesh, MeshSpatial, Bvh, FaceId, FaceGroupId,
Polyline3, Rgba8`. From S01: `EntityRef, BodyId, ComponentId, Expr, Feature, RecomputeCtx,
ExtRecord, ConstructionGeom`.

```rust
// ════════════════ amg_features::inspect (lane A) ════════════════
pub mod inspect {
use std::sync::Arc;
use amg_core::*;
use amg_jobs::JobCtx;
use amg_mesh::{FaceGroupId, FaceId, IndexedMesh, MeshBody, Polyline3};

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum InspectError {
    #[error(transparent)] Mesh(#[from] amg_mesh::MeshError),
    #[error("measuring {what} between {a} and {b} is not supported")]
    Unsupported { what: &'static str, a: &'static str, b: &'static str },
    #[error("element {0} does not exist in this body")] BadElement(String),
    #[error("{0} needs a closed, oriented body")] NotWatertight(&'static str),
    #[error("transform is not rigid; measurements need rigid occurrence transforms")] NonRigidTransform,
    #[error("degenerate geometry: {0}")] Degenerate(&'static str),
    #[error("too many section planes ({0} > 6)")] TooManyPlanes(usize),
    #[error("B-rep query failed: {0}")] Brep(String),          // INS-07/10 (from amg_brep::KernelError)
    #[error("reference does not resolve: {0}")] Resolve(String),// INS-07 (from CON-03)
    #[error("operation cancelled")] Cancelled,
}
impl Diagnostic for InspectError { /* hints: NotWatertight → "Repair the mesh first (Mesh ▸ Repair)" */ }
impl From<amg_core::Cancelled> for InspectError { fn from(_: amg_core::Cancelled) -> Self { InspectError::Cancelled } }

// ─────────── measure (INS-01; INS-07 extends Operand) ───────────
pub mod measure {
    use super::*;

    /// Sub-element of a mesh body; indices are body-local.
    #[derive(Clone, Debug, PartialEq)]
    pub enum MeshElement {
        Body,
        FaceGroup(FaceGroupId),
        /// Sorted, deduplicated. One picked triangle, or a smart planar region.
        Faces(Arc<[FaceId]>),
        /// A triangle edge given by its two vertex indices.
        Edge { a: u32, b: u32 },
        /// Vertex chain: a boundary loop (`AnalysisReport::boundary_loops`) or a feature-edge chain.
        Polyline { verts: Arc<[u32]>, closed: bool },
        Vertex(u32),
        SurfacePoint { face: FaceId, bary: [f64; 3] },
    }

    #[derive(Clone)]
    pub struct MeshOperand { pub body: MeshBody, pub to_world: Transform /* rigid */, pub element: MeshElement }

    #[derive(Clone)]
    #[non_exhaustive]
    pub enum Operand {
        Point(Point3),
        Mesh(MeshOperand),
        /// INS-07: construction / origin / sketch / analytic B-rep geometry (CON-03).
        Ref(crate::construct::ResolvedRef),
        /// INS-07: exact B-rep sub-shape.
        Brep(BrepOperand),
    }
    /// INS-07. `sub == None` means the whole shape.
    #[derive(Clone)]
    pub struct BrepOperand { pub shape: amg_brep::Shape, pub sub: Option<amg_brep::SubShape /* S02 request */>, pub to_world: Transform }

    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct MeasureOpts {
        pub tol: Tolerance,              // Tolerance::mesh()
        pub planar_tolerance: f64,       // mm, 1e-3
        pub circle_tolerance: f64,       // mm RMS, 1e-2
    }
    impl Default for MeasureOpts { fn default() -> Self; }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum OperandKind { Point, Vertex, SurfacePoint, Edge, Polyline, Faces, FaceGroup, Body, Construction, Sketch, BrepFace, BrepEdge, BrepVertex, BrepBody }

    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct Circle3 { pub center: Point3, pub normal: Dir3, pub radius: f64 }
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct CircleFit { pub circle: Circle3, pub rms: f64, pub max_residual: f64 }
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct PlaneFit { pub plane: Plane, pub max_deviation: f64 }

    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum Exactness { Exact, Tessellated { deflection: f64 } }

    /// Single-entity results (world space). `None` = not applicable to this element.
    #[derive(Clone, Debug, PartialEq)]
    pub struct EntityMeasure {
        pub kind: OperandKind,
        pub position: Option<Point3>,
        pub length: Option<Length>,
        pub area: Option<Area>,
        pub volume: Option<Volume>,
        pub centroid: Option<Point3>,
        pub plane: Option<PlaneFit>,
        pub direction: Option<Dir3>,     // straight edge / chain, directed first → last vertex
        pub circle: Option<CircleFit>,
        pub radius: Option<Length>,      // INS-07: cylinder / sphere face, circular edge
        pub bbox: Option<Aabb3>,
        pub face_count: Option<usize>,
        pub exactness: Exactness,
    }

    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum WitnessFeature { Point, Vertex(u32), Edge(u32, u32), Face(FaceId), Brep /* INS-07 */ }
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct Witness { pub point: Point3, pub feature: WitnessFeature }

    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct MinDistance {
        pub distance: Length,
        pub a: Witness,
        pub b: Witness,
        pub intersecting: bool,          // distance ≤ tol.linear
        pub contained: Option<bool>,     // Some(true): one watertight body lies inside the other
        pub exactness: Exactness,
    }
    impl MinDistance { pub fn delta(&self) -> Vec3 { self.b.point - self.a.point } }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum AngleKind { LineLine, PlanePlane, LinePlane }
    /// Ranges and conventions: docs/notes/inspection-geometry.md §2.5.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct AngleMeasure { pub kind: AngleKind, pub angle: Angle, pub supplement: Angle }

    #[derive(Clone, Debug, PartialEq)]
    pub struct PairMeasure { pub min_distance: MinDistance, pub angle: Option<AngleMeasure>, pub center_distance: Length }

    #[derive(Clone, Debug, PartialEq)]
    pub struct MeasureReport { pub first: EntityMeasure, pub second: Option<EntityMeasure>, pub pair: Option<PairMeasure> }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Cost { Inline, Job }
    /// `Inline` only for work proven < 0.1 ms (no BVH traversal, ≤ 64 primitives per operand).
    pub fn estimated_cost(a: &Operand, b: Option<&Operand>) -> Cost;

    pub fn measure_one(a: &Operand, opts: &MeasureOpts, ctx: &JobCtx) -> Result<EntityMeasure, InspectError>;
    pub fn measure(a: &Operand, b: Option<&Operand>, opts: &MeasureOpts, ctx: &JobCtx) -> Result<MeasureReport, InspectError>;
    pub fn min_distance(a: &Operand, b: &Operand, opts: &MeasureOpts, ctx: &JobCtx) -> Result<MinDistance, InspectError>;
    pub fn angle_between(a: &EntityMeasure, b: &EntityMeasure) -> Option<AngleMeasure>;

    // smart selection & fits (note §2.3–2.4)
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct RegionOpts { pub max_normal_angle: f64 /* 0.5° */, pub planar_tolerance: f64 /* 1e-3 */, pub stop_at_group_borders: bool, pub max_faces: usize }
    pub fn grow_planar_region(m: &IndexedMesh, seed: FaceId, opts: &RegionOpts, ctx: &JobCtx) -> Result<Arc<[FaceId]>, InspectError>;
    pub fn grow_feature_chain(m: &IndexedMesh, a: u32, b: u32, feature_angle: f64 /* 30° */, ctx: &JobCtx) -> Result<MeshElement /* Polyline */, InspectError>;
    pub fn fit_plane_faces(m: &IndexedMesh, faces: &[FaceId]) -> Option<PlaneFit>;          // None if normals cancel
    pub fn circle_through(a: Point3, b: Point3, c: Point3, tol: &Tolerance) -> Result<Circle3, InspectError>;
    pub fn fit_circle(points: &[Point3], opts: &MeasureOpts) -> Option<CircleFit>;           // ≥ 5 points

    // picking support (no BVH needed)
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct SnapOpts { pub vertex_radius: f64 /* mm at the hit */, pub edge_radius: f64 }
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum Snapped { Vertex(u32), Edge { a: u32, b: u32, t: f64 }, Face { face: FaceId, bary: [f64; 3] } }
    /// `ray` in world space; `face` from the RND-04 ID buffer.
    pub fn snap_on_face(m: &IndexedMesh, to_world: &Transform, face: FaceId, ray: &Ray3, opts: &SnapOpts) -> Option<(Snapped, Point3)>;

    // display
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct DisplayPrefs { pub length: LengthUnit, pub angle: Unit /* Degree */, pub precision: u8 }
    pub fn area_unit(l: LengthUnit) -> Unit;     // mm→mm², cm→cm², m→m², in→in², ft→in² (no ft² unit)
    pub fn volume_unit(l: LengthUnit) -> Unit;
    #[derive(Clone, Debug, PartialEq)]
    pub struct MeasureRow { pub key: &'static str /* "distance", "dx", "angle", "area", … */, pub label: String, pub value: Quantity, pub text: String }
    /// Stable row order (tested): distance, dx, dy, dz, angle, center distance, then per-entity rows.
    pub fn report_rows(r: &MeasureReport, prefs: &DisplayPrefs) -> Vec<MeasureRow>;
}

// ─────────── section caps (INS-02) ───────────
pub mod section {
    use super::*;
    pub const MAX_SECTION_PLANES: usize = 6;
    /// Each plane removes the half-space its normal points into (signed_distance > 0).
    #[derive(Clone, Debug, PartialEq)]
    pub struct SectionPlanes(pub Vec<Plane>);   // world space, ≤ MAX_SECTION_PLANES
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum FillRule { Auto /* NonZero if the mesh is oriented, else EvenOdd */, NonZero, EvenOdd }
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct CapOpts { pub snap_tolerance: f64 /* Tolerance::mesh().linear */, pub fill_rule: FillRule }

    #[derive(Clone, Debug, PartialEq)]
    pub struct Cap {
        pub plane_index: u8,
        pub positions: Vec<Point3>,     // world
        pub uv: Vec<Point2>,            // plane-local mm (hatch anchoring)
        pub tris: Vec<[u32; 3]>,        // CCW seen from the removed side (+normal), i.e. facing the viewer
        pub outlines: Vec<Polyline3>,   // closed section curves after clipping by the other planes
        pub open: Vec<Polyline3>,       // open section curves (non-closed meshes); never capped
    }
    #[derive(Clone, Debug, PartialEq)]
    pub struct SectionStats { pub plane_index: u8, pub area: Area, pub perimeter: Length, pub loops: u32, pub open_loops: u32, pub extent: Aabb2 /* plane frame */ }
    #[derive(Clone, Debug, PartialEq)]
    pub struct BodySection { pub caps: Vec<Cap>, pub stats: Vec<SectionStats> }

    pub fn section_mesh(body: &MeshBody, to_world: &Transform, planes: &SectionPlanes, opts: &CapOpts, ctx: &JobCtx) -> Result<BodySection, InspectError>;
    /// The Phase 1 plane parameterisation shared by the record and the tool.
    pub fn section_plane(base: &Plane, offset: f64, tilt: [f64; 2], flip: bool) -> Plane;
}

// ─────────── mass properties (INS-03; INS-10 adds B-rep) ───────────
pub mod props {
    use super::*;
    #[derive(Clone, Debug, PartialEq)]
    pub struct MassProps {
        pub area: Area,
        pub volume: Option<Volume>,              // None if not watertight
        pub density: Density,
        pub mass: Option<Mass>,
        pub center_of_mass: Option<Point3>,      // world
        pub inertia_com: Option<Mat3>,           // t·mm², about the CoM, world axes
        pub inertia_origin: Option<Mat3>,        // about the world origin
        pub principal_moments: Option<[f64; 3]>, // ascending, t·mm²
        pub principal_axes: Option<[Dir3; 3]>,   // right-handed, sign rule in note §4
        pub not_watertight: Option<String>,      // reason when volume-based values are None
        pub exactness: measure::Exactness,
    }
    pub fn mesh_mass_props(body: &MeshBody, to_world: &Transform, density: Density, ctx: &JobCtx) -> Result<MassProps, InspectError>;
    /// Sum of several bodies (assembly / selection totals, INS-10). CoM and inertia are combined by the parallel-axis rule.
    pub fn combine(parts: &[MassProps]) -> MassProps;
}

// ─────────── analysis fields (INS-05/06) ───────────
pub mod fields {
    use super::*;
    pub fn draft_angle(m: &IndexedMesh, to_world: &Transform, pull: Dir3) -> Vec<f64>;              // rad per vertex
    pub fn accessibility(s: &amg_mesh::MeshSpatial, to_world: &Transform, from: Dir3, ctx: &JobCtx) -> Result<Vec<f64>, InspectError>; // 0/1
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum CurvatureKind { Mean, Gaussian, Min, Max }
    /// NaN on boundary vertices.
    pub fn mesh_curvature(m: &IndexedMesh, kind: CurvatureKind, ctx: &JobCtx) -> Result<Vec<f64>, InspectError>;
}

// ─────────── interference (INS-04) ───────────
pub mod interfere {
    use super::*;
    #[derive(Clone)]
    pub enum InterferenceBody { Mesh(MeshBody, Transform), Brep(amg_brep::Shape, Transform) }
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct InterferenceOpts { pub include_coincident: bool, pub touch_volume: f64 /* mm³, 1e-6 */, pub compute_volume: bool }
    #[derive(Clone, Debug, PartialEq)]
    pub enum Contact { Touching, Interfering { volume: Option<Volume>, exactness: measure::Exactness } }
    #[derive(Clone, Debug, PartialEq)]
    pub struct InterferencePair { pub a: usize, pub b: usize, pub contact: Contact }
    /// Sorted by (a, b). Pairs with no contact are omitted.
    pub fn interference(bodies: &[InterferenceBody], opts: &InterferenceOpts, ctx: &JobCtx) -> Result<Vec<InterferencePair>, InspectError>;
}

// ─────────── document glue (INS-09) ───────────
pub mod doc {
    use super::*;
    use amg_doc::{BodyId, EntityRef, ExtRecord};
    pub const SECTION_KIND: &str = "inspect.section";
    pub const SECTION_VERSION: u32 = 1;

    #[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum OriginPlane { XY, YZ, XZ }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum SectionBase {
        Origin(OriginPlane),
        Free(Plane),
        /// Phase 2 (INS-11). `last` is used, with a warning, when `entity` stops resolving.
        Ref { entity: EntityRef, last: Plane },
    }
    #[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum HatchPattern { None, Lines45, Cross }
    #[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct HatchStyle { pub pattern: HatchPattern, pub spacing_px: f32 /* 8 */, pub angle: f64 /* rad, π/4 */ }
    #[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum CapColor { BodyColor, Uniform(amg_mesh::Rgba8) }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum SectionScope { AllVisible, Bodies(Vec<BodyId>) }

    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct SectionAnalysis {
        pub name: String,
        pub visible: bool,
        pub base: SectionBase,
        pub offset: f64,           // mm along the base normal
        pub tilt: [f64; 2],        // rad: about base x, then base y
        pub flip: bool,
        pub hatch: HatchStyle,
        pub cap_color: CapColor,
        pub scope: SectionScope,
    }
    impl SectionAnalysis {
        pub fn to_record(&self) -> ExtRecord;                                   // kind/version as above
        pub fn from_record(r: &ExtRecord) -> Result<Self, serde_json::Error>;
        /// World plane. `resolve` maps a Ref base to its current plane (Phase 2); bool = fallback used.
        pub fn plane(&self, resolve: &dyn Fn(&EntityRef) -> Option<Plane>) -> (Plane, bool);
    }
    /// Registered with `inventory` (S01 ExtRegistration).
    pub fn section_migrate(from: u32, raw: serde_json::Value) -> Result<serde_json::Value, amg_doc::MigrationError>;
}
}

// ════════════════ amg_features::construct (lane A, CON-01..04) ════════════════
pub mod construct {
use std::sync::Arc;
use amg_core::*;
use amg_doc::{EntityRef, Expr};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum OriginEntity { Point, XAxis, YAxis, ZAxis, XYPlane, YZPlane, XZPlane }
#[derive(Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum RefInput { Origin(OriginEntity), Ref(EntityRef) }

pub trait CurveEval: Send + Sync + std::fmt::Debug {
    fn range(&self) -> (f64, f64);
    fn eval(&self, t: f64) -> Point3;
    fn d1(&self, t: f64) -> Vec3;
    fn d2(&self, t: f64) -> Vec3;
    fn length(&self) -> f64;
    fn param_at_length(&self, s: f64) -> f64;
    fn closest_param(&self, p: Point3) -> f64;
}
pub trait SurfaceEval: Send + Sync + std::fmt::Debug {
    fn eval(&self, uv: Point2) -> Point3;
    /// Outward normal (face orientation applied); None at singular points.
    fn normal(&self, uv: Point2) -> Option<Dir3>;
    fn closest_uv(&self, p: Point3) -> Option<Point2>;
}

/// World-space geometry of a resolved reference.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum RefGeom {
    Point(Point3),
    Line(Line3),
    Segment(Segment3),
    Circle { center: Point3, normal: Dir3, radius: f64, arc: Option<(f64, f64)> },
    Plane(Plane),
    Cylinder { axis: Line3, radius: f64 },
    Cone { apex: Point3, axis: Dir3, half_angle: f64 },
    Sphere { center: Point3, radius: f64 },
    Torus { axis: Line3, major: f64, minor: f64 },
    Curve(Arc<dyn CurveEval>),
    Surface(Arc<dyn SurfaceEval>),
}
#[derive(Clone, Debug)]
pub struct ResolvedRef {
    pub geom: RefGeom,
    /// The sketch plane for sketch entities, or the first adjacent planar face for B-rep edges.
    pub context_plane: Option<Plane>,
    pub quality: Option<amg_doc::MatchQuality>,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ConstructError {
    #[error("expected {expected}, got {got}")] WrongInput { expected: &'static str, got: &'static str },
    #[error("inputs are parallel")] Parallel,
    #[error("inputs coincide")] Coincident,
    #[error("points are collinear")] Collinear,
    #[error("lines are not coplanar (gap {distance} mm)")] Skew { distance: f64 },
    #[error("point is {distance} mm off the face")] NotOnFace { distance: f64 },
    #[error("edge is not straight")] NotStraight,
    #[error("no intersection")] NoIntersection,
    #[error("position is outside the path")] OutOfRange,
    #[error("unsupported surface for this construction")] Unsupported,
    #[error("degenerate input: {0}")] Degenerate(&'static str),
}
impl From<ConstructError> for amg_doc::FeatureError { fn from(e: ConstructError) -> Self { /* InvalidValue { field: "inputs", msg } */ } }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PathPosition { Fraction(Expr), Distance(Expr) }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PlaneDef {
    Offset { base: RefInput, distance: Expr },
    AtAngle { line: RefInput, zero: Option<RefInput>, angle: Expr },
    Tangent { face: RefInput, reference: RefInput, angle: Expr },
    Midplane { a: RefInput, b: RefInput },
    TwoEdges { a: RefInput, b: RefInput },
    ThreePoints { a: RefInput, b: RefInput, c: RefInput },
    TangentAtPoint { face: RefInput, point: RefInput },
    AlongPath { path: RefInput, at: PathPosition },
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum AxisDef {
    ThroughRevolved { face_or_edge: RefInput },        // cylinder / cone / torus face, circular edge
    PerpendicularAtPoint { face: RefInput, point: RefInput },
    ThroughTwoPlanes { a: RefInput, b: RefInput },
    ThroughTwoPoints { a: RefInput, b: RefInput },
    ThroughEdge { edge: RefInput },
    PerpendicularToFaceAtPoint { face: RefInput, point: RefInput },
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PointDef {
    AtVertex { point: RefInput },
    ThroughTwoEdges { a: RefInput, b: RefInput },
    ThroughThreePlanes { a: RefInput, b: RefInput, c: RefInput },
    AtCenter { entity: RefInput },
    EdgeAndPlane { edge: RefInput, plane: RefInput },
    AlongPath { path: RefInput, at: PathPosition },
}

/// kind "construct.plane" / "construct.axis" / "construct.point", version 1.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)] pub struct ConstructPlane { pub def: PlaneDef }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)] pub struct ConstructAxis  { pub def: AxisDef }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)] pub struct ConstructPoint { pub def: PointDef }
// impl amg_doc::Feature for each: inputs() = Dependency::Entity/Construction per RefInput::Ref;
// compute() resolves inputs (CON-03), calls `math`, returns FeatureOutput { construction: Some(..) }.

/// Pure math (note §6). All results in world space.
pub mod math {
    use super::*;
    pub fn origin_geom(e: OriginEntity, component_to_world: &Transform) -> RefGeom;
    pub fn plane_offset(base: &Plane, distance: f64) -> Plane;
    pub fn plane_at_angle(line: &Line3, zero: Option<&Plane>, angle: f64, tol: &Tolerance) -> Result<Plane, ConstructError>;
    pub fn plane_tangent(surface: &RefGeom, reference: &Plane, angle: f64, tol: &Tolerance) -> Result<Plane, ConstructError>;
    pub fn midplane(a: &RefGeom, b: &RefGeom, tol: &Tolerance) -> Result<Plane, ConstructError>;
    pub fn plane_two_lines(a: &Line3, b: &Line3, tol: &Tolerance) -> Result<Plane, ConstructError>;
    pub fn plane_three_points(a: Point3, b: Point3, c: Point3, tol: &Tolerance) -> Result<Plane, ConstructError>;
    pub fn plane_tangent_at_point(surface: &RefGeom, p: Point3, tol: &Tolerance) -> Result<Plane, ConstructError>;
    pub fn plane_along_path(curve: &dyn CurveEval, t: f64, tol: &Tolerance) -> Result<Plane, ConstructError>;
    pub fn path_param(curve: &dyn CurveEval, fraction_or_distance: f64, by_distance: bool) -> Result<f64, ConstructError>;
    pub fn axis_of(g: &RefGeom) -> Result<Line3, ConstructError>;
    pub fn axis_perpendicular_at_point(surface: &RefGeom, p: Point3, on_face: bool, tol: &Tolerance) -> Result<Line3, ConstructError>;
    pub fn axis_two_planes(a: &Plane, b: &Plane, tol: &Tolerance) -> Result<Line3, ConstructError>;
    pub fn axis_two_points(a: Point3, b: Point3, tol: &Tolerance) -> Result<Line3, ConstructError>;
    pub fn line_of(g: &RefGeom, tol: &Tolerance) -> Result<Line3, ConstructError>;     // Line / Segment / straight Curve
    pub fn plane_of(g: &RefGeom) -> Result<Plane, ConstructError>;
    pub fn point_of(g: &RefGeom) -> Result<Point3, ConstructError>;
    pub fn point_two_lines(a: &Line3, b: &Line3, tol: &Tolerance) -> Result<Point3, ConstructError>;
    pub fn point_three_planes(a: &Plane, b: &Plane, c: &Plane, tol: &Tolerance) -> Result<Point3, ConstructError>;
    pub fn center_of(g: &RefGeom) -> Result<Point3, ConstructError>;
    pub fn point_curve_plane(g: &RefGeom, p: &Plane, tol: &Tolerance) -> Result<Point3, ConstructError>;
}

/// CON-03. Called inside `Feature::compute`; also used by INS-07/INS-11 and offered to S03.
pub mod resolve {
    use super::*;
    pub fn resolve_input(ctx: &amg_doc::RecomputeCtx<'_>, r: &RefInput) -> Result<ResolvedRef, amg_doc::FeatureError>;
    /// For tools outside recompute (measure, section analysis): resolves against an `Evaluated`.
    pub fn resolve_in(eval: &amg_doc::Evaluated, design: &amg_doc::Design, r: &RefInput) -> Result<ResolvedRef, InspectResolveError>;
    #[derive(Debug, Clone, PartialEq, thiserror::Error)]
    pub enum InspectResolveError {
        #[error(transparent)] Resolve(#[from] amg_doc::ResolveError),
        #[error("entity kind not supported as construction input")] Unsupported,
    }
}

/// CON-04. Pure display sizing (note §6).
pub mod display {
    use super::*;
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum ConstructionDisplay { PlaneSquare { frame: Plane, half_size: f64 }, AxisSegment(Segment3), PointMarker(Point3) }
    pub fn display_extent(geom: &amg_doc::ConstructionGeom, visible_bounds: &Aabb3) -> ConstructionDisplay;
}
}
```

**Needed from other specs (requests, not edits; also listed in the refinement report):**
- **S07 (MESH-03, additive):** read-only BVH node access, and non-blocking getters on
  `MeshBody`. Section loop orientation is requested for MESH-09.
  ```rust
  pub enum BvhNodeKind { Inner { left: u32, right: u32 }, Leaf { first: u32, count: u32 } }
  pub struct BvhNode { pub bounds: Aabb3 /* f64 view of the outward-rounded bounds */, pub kind: BvhNodeKind }
  impl Bvh { pub fn root(&self) -> u32; pub fn node(&self, i: u32) -> BvhNode; pub fn prim(&self, slot: u32) -> u32; }
  impl MeshBody { pub fn try_spatial(&self) -> Option<&MeshSpatial>; pub fn try_analysis(&self) -> Option<&AnalysisReport>; }
  ```
  MESH-09: closed `section()` polylines are oriented with material on the left when viewed from
  the plane's +normal side (oriented closed input). Fallback without it: the note §3.2
  winding-number probe.
- **S01:**
  - `BodyGeom::Mesh` holds `amg_mesh::MeshBody` instead of `Arc<IndexedMesh>`, so BVH and
    analysis are built once per evaluated body (this also aligns S01 with S07);
  - `Command::AddExt { kind, record } → CreatedId::Ext(ExtKey)` (or a documented id-allocation
    rule for `ExtKey.id`);
  - `UnitPrefs` exposes the display length unit, angle unit and precision;
  - Phase 2: `Evaluated::body_to_world(BodyId, occurrence path) -> Transform`, and optionally
    `EntityRef::Origin { component, which }` to replace `RefInput::Origin`.
- **S00 (FND-10, additive):** inertia units `Unit::{KilogramSquareMillimetre,
  KilogramSquareMetre, GramSquareMillimetre, PoundSquareInch}` with `Dimension` mass·length².
- **S02 (BREP-02/08/09, Phase 2):**
  - `SubShape` handles and `Shape::sub(kind, index)`;
  - `distance(a, b) -> DistanceResult { distance, pa, pb }` (BRepExtrema);
  - `mass_props(shape, Volume | Surface | Linear) -> { measure, centroid, inertia_about_centroid }`;
  - `surface_geom(face)` and `curve_geom(edge)` (analytic type + params, orientation);
  - evaluation with first and second derivatives;
  - `closest_point(sub, p)`;
  - `surface_curvature(face, uv) -> (k1, k2, d1, d2)`;
  - `faces_of_edge`;
  - `iso_curves(face, nu, nv, deflection)`.

  All are `&self` on a `Sync` `Shape`.
- **S10 / S11:** see the refinement report. In short:
  - RND-06: CPU cap meshes with uv and hatch, ≤ 6 clip planes, per-body scope;
  - RND-04: world pick ray, face index, world-per-pixel;
  - RND-07: measurement dimensions, angle arcs, CoM glyph, construction-geometry display;
  - RND-08: 0/1 and NaN-aware fields;
  - UI-03: non-committing tools;
  - UI-04: mesh sub-element selection;
  - UI-07: "Analysis" folder registration for ExtRecord kinds.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| INS-01 | `amg-features::inspect::measure` core on meshes: operands, `measure_one`/`measure`, exact min distance (dual BVH, tri–tri), angles, centres, smart planar region, feature chains, plane/circle fits, `snap_on_face`, `report_rows` | MESH-02, MESH-03, FND-10 | P0 | Todo | Tests pass. 1. **Analytic:** a 10 mm cube gives face area 100 ± 1e-9 mm², edge length 10 ± 1e-12, opposite-face distance 10 ± 1e-12 and angle 180°, adjacent faces 90° ± 1e-12 rad, body volume 1000 ± 1e-9. A point 5 mm above the face centre gives distance 5 ± 1e-12. 2. **Brute force:** `min_distance` equals the O(n·m) brute force (≤ 1e-12 mm absolute, same witness features) on 1 000 random pairs of 1–2 k-tri meshes under random rigid transforms, including intersecting, touching and nested configurations. 3. A tessellated 128-segment hole r = 5 in a block: one feature-edge pick gives a closed 128-edge chain with fit radius 5 ± 1e-9 (vertices on the circle). A 64-segment chain with σ = 0.002 mm noise gives radius 5 ± 0.005. 4. `grow_planar_region` on a CAD-exported STL (corpus, f32) returns exactly the ground-truth face set of 10 planar faces. 5. Non-rigid transform → `NonRigidTransform`. 6. Bit-identical results with 1 and 16 threads. 7. Criterion: point → 1 M-tri body < 0.1 ms median; two separated 1 M-tri bodies (gap ≥ 1 mm) < 20 ms; concentric icospheres r = 50 / 50.1 (1 M each) < 3 s, cancellable within 50 ms. |
| INS-08 | Measure tool in `amg-ui::inspect`: hand-built dialog (filters, smart toggle, 3-point circle, precision, units, results with copy), snapping, job-backed results with supersede, RND-07 overlay (witness line, Δ, angle arc, circle) | INS-01, MESH-16, UI-02, UI-03, UI-04, UI-10, RND-04, RND-07 | P0 | Todo | Manual script on 3 corpus models: measure a hole diameter (1 click), a wall distance, an angle between two faces, a body volume; values equal the INS-01 API values for the same picks. Snap to vertex within 8 px and to edge within 5 px (automated test with a synthetic camera). No frame > 16 ms in the tracing profile while measuring two 1 M-tri bodies. A newer selection cancels the running job within 50 ms. Results use the document display unit and the chosen precision (`1/4 in` part shows inches). |
| INS-02 | `amg-features::inspect::section` caps on meshes: loop orientation, CDT with split constraints, non-zero / even–odd fill, multi-plane clipping (≤ 6), outlines, open curves, `SectionStats`, `section_plane` | MESH-09 | P0 | Todo | Tests pass. 1. **Watertight (note §3.4):** for every watertight MESH-28 corpus model and 3 seeded random planes, `plane_cut(keep = Negative, cap = false)` + caps welded at the snap tolerance is closed and manifold (100 %), including planes exactly through vertices and edges. 2. Cap area: 128-gon cylinder r = 10, perpendicular cut = ½·128·r²·sin(2π/128) within 1e-9 relative; axial cut = 2r·h exactly within 1e-9 relative. 3. Two overlapping closed cubes (one STL, two shells): cap area = area of the union of the squares (non-zero rule). 4. Box section with 6 planes: each cap lies inside the other 5 kept half-spaces (vertices ≤ 1e-9 mm outside). 5. Open cube (one face deleted): open curve reported, no cap through the gap. 6. Shoelace area of the oriented loops equals the cap triangle area within 1e-9 relative. 7. 1 M-tri body, one plane: < 50 ms; 10 M: < 500 ms. Deterministic across thread counts. |
| INS-09 | Section Analysis tool + persistence: `SectionAnalysis` ExtRecord (`inspect.section` v1, migration hook), hand-built dialog (origin plane / picked planar region, offset expression + slider, tilt, flip, hatch, cap colour, scope), browser "Analysis" folder entries, combining visible analyses, cap jobs feeding RND-06, stats panel | INS-02, INS-08, DOC-05, DOC-07, UI-07, RND-06 | P0 | Todo | Creating, editing, hiding and deleting a section are each one undo step and survive save/load byte-identically. Two visible analyses combine (box-corner cut visible in a screenshot test with RND-09, or a manual check before RND-09). Dragging the offset slider on a 1 M-tri body keeps ≥ 60 fps (GPU clip), and caps catch up ≤ 100 ms after the drag stops. Phase 1 manual script: open STL → section at XY + 10 mm → read the section area → flip → hatch on → save → reopen. |
| INS-03 | Physical properties of mesh bodies: `props::mesh_mass_props`, `combine`, density from material with default material, Properties panel, CoM glyph | INS-01, INS-08, MAT-01 | P0 | Todo | 1. A 10×20×30 box rotated by 20 random rotations and translated: volume 6000 ± 1e-9, CoM within 1e-9 mm, inertia = R·diag(m(b²+c²)/12, …)·Rᵀ within 1e-9 relative. 2. Icosphere r = 10: volume and inertia converge to the analytic sphere values (error falls ≥ 3.5× per subdivision level). 3. Translating a body changes `inertia_com` by < 1e-9 relative. 4. Non-watertight → volume-based fields `None` with a reason; area still given. 5. `combine` of two halves equals the whole within 1e-9 relative. 6. 1 M tris < 30 ms; 10 M < 300 ms; bit-identical across thread counts. 7. Mass of a 1000 mm³ PLA cube shows 1.24 g. |
| CON-01 | Construction framework + planes: `RefInput`/`OriginEntity`/`RefGeom`/`ConstructError`, `construct::math` plane functions (all 8 variants) and helpers, `ConstructPlane` feature registered (`construct.plane` v1) with inputs, `exprs`, compute against pre-resolved `RefGeom` (test resolver) | DOC-02, DOC-03, DOC-04 | P0 | Todo | Table-driven tests: ≥ 5 analytic cases per variant plus every error in note §6. Examples: offset XY by 5 → z = 5. At-angle 30° about +X from XY → normal (0, −sin 30°, cos 30°) (right-hand rule). Tangent to cylinder r = 3 on Z at 0° from XZ → normal +Y, through (0, 3, 0). Midplane of z = 0 and z = 10 → z = 5. Three collinear points → `Collinear`. Results within 1e-12. The same inputs give a bit-identical frame. The feature serialises round-trip, recomputes from the cache, and an `Expr` parameter edit recomputes only it and its dependants (DOC-03 counter). |
| CON-02 | Axes (6 variants) and points (6 variants): math + `ConstructAxis`/`ConstructPoint` features | CON-01 | P0 | Todo | Same table-driven standard as CON-01 (≥ 5 analytic cases per variant, all errors). Point through three planes on random non-degenerate triples matches a 3×3 solve within 1e-9. Line–line on skew lines with a 0.01 mm gap → `Skew { distance: 0.01 ± 1e-12 }`. The features register, serialise and recompute. |
| CON-03 | Reference resolution: `EntityRef`/`RefInput` → `ResolvedRef` for origin entities (with component transform), construction geometry, B-rep faces/edges/vertices (S02 analytic types, `CurveEval`/`SurfaceEval` adapters, context plane), sketch curves/points (S03 world mapping), planar mesh face groups (plane fit); `resolve_in` for tools; health mapping (FingerprintOnly → warning) | CON-01, DOC-06, BREP-09, SK-01, MESH-16 | P0 | Todo | On a test part (box with a hole, built with S04 or with direct `BrepKernel` calls): the planar face resolves to a `Plane` matching the face within 1e-9; the hole face resolves to a `Cylinder` with the exact radius; a circular edge resolves to a `Circle`; a sketch line resolves to a `Line` with `context_plane` = the sketch plane. Offset plane on a face survives an upstream extrude-distance edit (with DOC-06 naming) and moves with the face. A deleted referenced face gives a feature `Error` whose `Diagnostic.entities` contains the ref. |
| CON-04 | Construct UI: commands in the Construct menu (`amg-ui::construct`), UI-05 dialog schemas for all variants, construction display via `display::display_extent` + RND-07 with pick ids, browser "Construction" / "Origin" folders | CON-02, CON-03, UI-05, UI-07, RND-07 | P0 | Todo | Each of the 20 variants can be created from the menu and S-key search, edited and deleted, all undoable. `display_extent` unit tests follow the note §6 sizing rules. Construction planes are pickable as sketch planes and as section-analysis bases. Manual script: offset plane → sketch on it → extrude → edit the offset → the extrude follows. |
| INS-07 | Measure on B-rep, construction, origin and sketch entities: `Operand::Ref`/`Operand::Brep`, exact B-rep distance/length/area via S02, analytic radius/angle via `RefGeom`, mixed mesh–B-rep via tessellation (`Exactness::Tessellated`), tool filters extended | INS-01, INS-08, CON-03, BREP-08, BREP-09 | P0 | Todo | On analytic solids: box edge length, face area, distance between parallel faces within 1e-9; cylinder face radius exact; circular edge diameter exact; angle between two planar faces at 30° ± 1e-12 rad. Distance from a sphere (r = 10, centre 25 mm away) to a plane: 15 ± 1e-7. Distance between a construction plane and a vertex is exact. A mesh–solid distance is marked "≈" and lies within the tessellation deflection of the exact value. |
| INS-10 | Properties on solids + assemblies: B-rep `mass_props` (S02), per-component/occurrence totals with transforms, mixed selections, "Display component colours" | INS-03, BREP-09, ASM-01, RND-03 | P0 | Todo | Box, cylinder and sphere solids: volume, CoM and inertia within 1e-9 relative of analytic. The same part as a mesh (fine tessellation) agrees within the tessellation error bound. A two-occurrence assembly total equals the sum with the parallel-axis rule within 1e-9 relative. The component-colour toggle colours each component distinctly and restores the appearances. |
| INS-11 | Section analysis on solids + reference bases: caps from B-rep tessellation, `SectionBase::Ref` (construction planes, planar B-rep faces) with fallback warning, plane drag manipulator (UI-06) | INS-09, CON-03, BREP-08, UI-06 | P0 | Todo | The INS-02 watertight test passes on tessellations of 20 solid test parts. A section based on a construction plane follows a parameter edit of that plane. A deleted base face keeps the last plane and shows a warning. Dragging the manipulator updates the offset field both ways. |
| INS-04 | Interference: `interfere::interference` (broad phase, mesh BVH pair test, Manifold intersection volume, B-rep common volume, mixed via tessellation), dialog with pair table, highlight, include-coincident option, "Create bodies" via Combine features | INS-01, INS-08, MESH-05, BREP-04, ASM-01 | P1 | Todo | Two cubes overlapping by 2 mm: volume 2·10·10 = 200 ± 1e-6 mm³ (mesh and solid paths). Face-touching cubes → `Touching` (or omitted unless include-coincident). Disjoint → omitted. A 20-body random scene matches the brute-force pair list. "Create bodies" is one undo step. 50 bodies × 100 k tris < 2 s. |
| INS-05 | Mesh analysis fields + overlays: draft (pull direction, ±θ), accessibility (direction), discrete curvature (mean/Gaussian/min/max), legends, Analysis-folder entries | INS-08, MESH-03, RND-08 | P1 | Todo | Draft on a cylinder wall along its axis = 0 ± 1e-9 rad, and a 5° tapered frustum = 5° ± 0.01° away from edges. Accessibility from +Z of a box with a blind pocket: pocket floor accessible, undercut slot faces not, matching ray-cast ground truth exactly. Curvature on icosphere r = 10 (level 6): mean 0.1 ± 2 %, Gaussian 0.01 ± 3 %; plane 0 ± 1e-9. 1 M-vertex field < 200 ms (curvature), accessibility 1 M < 1 s. |
| INS-06 | B-rep curvature: exact curvature map at tessellation vertices, curvature comb on edges and sketch curves (RND-07 polylines, auto scale), minimum radius analysis with threshold | INS-05, INS-07, BREP-09 | P1 | Todo | Torus R = 10, r = 2: min radius 2 ± 1e-6, curvature map k1 = 1/2 ± 1e-9. Fillet r = 1 on a box is flagged with threshold 1.5 mm. Comb on a circle r = 5 has constant tooth length within 1e-9 relative. Comb on a B-spline matches finite-difference curvature within 1e-6. |
| INS-12 | Zebra stripes (RND-08) and B-rep isocurves (S02 `iso_curves`) as overlays, with stripe direction / density controls | INS-05, BREP-08, RND-08 | P1 | Todo | Zebra toggles per body with controls applied; a G1 vs G2 blend test pair shows the expected stripe kink (screenshot test via RND-09). Isocurves of a cylinder face give n_u straight lines and n_v circles within the tessellation deflection. |

## Testing
- **Unit and analytic tests** for every `measure`, `section`, `props`, `fields` and
  `construct::math` function. The cases are the ones in each WP's Done-when. Conventions come
  from the note, so the expected values are unambiguous.
- **Brute-force oracles:** the O(n·m) min distance (INS-01) and the pair lists (INS-04) run on
  random seeded meshes (`rand_chacha`, fixed seeds).
- **Property tests (`proptest`):**
  - min distance is invariant under a common rigid transform and symmetric (d(a, b) = d(b, a));
  - mass properties are invariant under translation and rotation (with tensor rotation);
  - section cap watertightness (INS-02);
  - construction outputs satisfy their defining relation (an offset plane is parallel at
    distance d; a three-point plane contains all three points).
- **Corpus:** INS-02 watertightness and INS-01 planar-region tests run on MESH-28 models through
  the S07 harness.
- **Golden tests** (`tests/golden/inspect/`): section stats and mass properties for 10 corpus
  models (area, perimeter, volume, CoM, principal moments) via `amg-testkit`.
- **Determinism:** 1 vs 16 rayon threads, run twice, bit-identical.
- **UI:** manual Phase 1 scripts (INS-08, INS-09). Screenshot tests via RND-09 when it lands.
- **Benchmarks (`criterion`):** every row of §Performance targets.

## Performance targets
Release build, reference machine, BVHs already built unless stated.

| Operation | Target |
|---|---|
| Inline measures (point–point, single triangle/edge, 3-point circle) | < 0.1 ms (runs on the UI thread) |
| Point → 1 M-tri body distance | < 0.1 ms median |
| Min distance, two 1 M-tri bodies ≥ 1 mm apart | < 20 ms |
| Min distance, worst case (concentric spheres, 0.1 mm gap, 1 M each) | < 3 s, job, cancel ≤ 50 ms |
| Planar region growth, 100 k-face region | < 30 ms |
| Section caps, one plane | 1 M tris < 50 ms; 10 M < 500 ms |
| Section drag (GPU clip) | ≥ 60 fps at 5 M tris (vision budget, RND-06) |
| Mesh mass properties | 1 M tris < 30 ms; 10 M < 300 ms |
| Draft / curvature field | 1 M vertices < 200 ms |
| Accessibility | 1 M vertices < 1 s |
| Construction feature compute (math only) | < 10 µs |

## Open questions
1. **(owner)** Default material for mass when a body has none: PLA 1.24 g/cm³ (recommended for
   an AM tool) or Steel (Fusion's default)? Should mass display in g by default (recommended)
   or kg? → QUESTIONS (proposed).
2. **(owner)** Several visible section analyses combine (kept region = intersection of their
   kept sides, up to 6 planes, which allows box sections; recommended), or only one is visible
   at a time? → QUESTIONS (proposed).
3. Resolved: measurement math lives in `amg-features::inspect`, not a new domain crate (no ADR
   needed). See Design §1 and the note §1.
4. Resolved: section caps are geometric (CPU, exact, multi-plane, with stats). GPU stencil
   capping is only an optional drag-time fallback in RND-06.
5. Resolved: section analyses are `ExtRecord`s, not timeline features (matches Fusion, where
   they are not in the timeline). Measure results are transient.
6. Resolved: mesh mass properties no longer wait for B-rep (INS-03 re-scoped; solids moved to
   the new INS-10), so they can ship in Phase 1.

## Refinement log
**2026-10-20 — S05 refinement (lane A agent).**

*Read:* AGENTS, vision, architecture, ADRs, parity, roadmap, QUESTIONS (binding answers,
especially #10 mm–t–s units, #11 `.` decimals, #22 hand-built Phase 1 dialogs), S00, S01, S07
(Ready), S02/S03/S04/S10/S11/S16/S17 drafts, notes `mesh-processing.md` and
`units-and-quantities.md`.

*Researched:*
- Fusion Measure / Section Analysis / Construct / Inspect tool behaviour;
- PQP-style dual-BVH distance (Larsen et al. 2000) and Ericson 2005 closest-feature proofs;
- polyhedral mass properties (Mirtich 1996, Eberly, Tonon 2004);
- discrete curvature (Meyer et al. 2003);
- circle fitting (Kåsa, Chernov 2010);
- Sutherland–Hodgman clipping;
- `spade` 2.15.1 `add_constraint_and_split`, checked on docs.rs (MIT OR Apache-2.0, already in
  S07's list).

*Decided:*
- Code placement: `amg-features::inspect` for the math (new module, lane A), plus
  `amg-ui::construct` (proposed).
- Phase 1 is mesh-only and OCCT-free: INS-01/02/08/09, plus INS-03 mesh properties.
- Smart planar regions and feature-edge chains with circle fit (hole diameters on STLs).
- Exact min distance with deterministic best-first traversal.
- Geometric caps with non-zero fill and multi-plane clipping.
- Section analyses stored as `ExtRecord`s.
- Angle, sign and frame conventions, written in the design note so Done-whens are analytic.
- Construction math is pure over `RefGeom`, separated from resolution.

*Changed WPs:*
- INS-01: core only (UI moved to the new INS-08). Deps add FND-10.
- INS-02: core caps only (UI and persistence moved to the new INS-09). Dep RND-06 → MESH-09.
  Solids moved to the new INS-11.
- INS-03: meshes only. Deps BREP-09 → INS-01, INS-08, MAT-01. B-rep and assemblies moved to the
  new INS-10, together with "display component colours" from the parity matrix.
- INS-04: deps add INS-01, INS-08, ASM-01.
- INS-05: re-scoped to mesh fields (draft, accessibility, curvature). Zebra and isocurves moved
  to the new INS-12. Deps add INS-08, MESH-03.
- INS-06: B-rep curvature, comb and minimum radius. Accessibility moved to INS-05. Deps add
  INS-05, INS-07.
- INS-07: deps add INS-08, CON-03, BREP-08.
- CON-01: deps SOL-01 → DOC-02, DOC-03, DOC-04. Math and framework are now separate from
  resolution (new CON-03) and UI (new CON-04).
- CON-02: axes and points.
- New WPs: INS-08, INS-09, INS-10, INS-11, INS-12, CON-03, CON-04.
- Every WP now has a measurable Done-when.

*Proposed roadmap placement (roadmap not edited):*
- Phase 1 lane A: INS-01, INS-08, INS-02, INS-09, and INS-03 (all deps are Phase 0/1: MESH-02/03/09/16,
  UI-02/03/04/07/10, RND-04/06/07, DOC-05/07, MAT-01).
- Phase 2 lane A: CON-01..04, INS-07, INS-10, INS-11 (deps DOC-06, BREP-08/09, UI-05/06, ASM-01
  are Phase 1–2).
- Phase 3 lane A: INS-04, INS-05, INS-06, INS-12.

The ADR-002 gate row should drop INS-03 (now OCCT-free) and add INS-10, INS-11 and INS-12. CON-01
and CON-02 no longer call OCCT directly; CON-03 does.

*Design note created:* `docs/notes/inspection-geometry.md`.

## Interface changes
None yet (spec not `Ready`).
