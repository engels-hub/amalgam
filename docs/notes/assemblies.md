# Assemblies: components, placements, joints and the joint solver

Design note for S06 (`docs/specs/S06-assemblies.md`). Written by the S06 refinement
(2026-11-08). Needed before ASM-01 (§1–§3) and ASM-03/ASM-04 (§4–§8).

## 0. Prior art

| Source | What it does | What we take | What we reject |
|---|---|---|---|
| Autodesk Fusion (help "Joint types", "Capture or Revert", tutorials on motion links, motion studies and contact sets) | Components + occurrences in one design. Seven joint types: rigid, revolute, slider, cylindrical, pin-slot, planar, ball. Joint origins snap to geometry. As-built joints, rigid groups, motion links (ratio between two joint values), drive joints, motion studies (per-joint keyed values over steps), contact sets (stop motion at contact; slow on cams and gears, Autodesk itself suggests limits or motion links instead). Moving a component leaves a *pending* position until Capture Position (a timeline entry) or Revert. | Data model, joint vocabulary and DOF table, joint-origin snaps, limits with rest value, motion links, motion study shape, contact sets as an opt-in, per-pair feature. | Pending positions and Capture Position timeline entries (§3; owner question QUESTIONS #59). |
| Onshape mates and mate connectors | Mates (fastened, revolute, slider, planar, cylindrical, pin slot, ball, parallel) between two mate connectors (frames on geometry, implicit or explicit). Mates live in an assembly feature list, not in the part history. | "A joint relates two frames" as the only primitive. Frames derived from geometry with a deterministic x-axis rule. | Separate assembly documents (we keep Fusion's single design). |
| FreeCAD 1.0 Assembly workbench + OndselSolver (LGPL-2.1, C++) | Joints between two "joint connector" placements, grounded parts, a multibody/assembly constraint solver with Euler parameters and Newton–Raphson. The FreeCAD wiki notes that long chains need help from the user to find "a sensible solution", which is a warm-start problem. | Joints as constraints on maximal coordinates (6 per part), grounded set, warm start from the current placements. | Taking OndselSolver as a dependency: C++, LGPL, another FFI, and we already have the S03 equation-system layer in Rust. |
| FreeCAD Assembly3 (uses SolveSpace `libslvs`) and SolveSpace | 3D constraints on points/normals with quaternion parameters plus a unit-norm constraint per quaternion. | Primitive constraints (coincident point, point on line, point on plane, parallel normals, locked rotation) as the residual vocabulary. | Quaternions with norm rows: we use increments on SE(3) instead (§6.1), so there are no redundant rows and no normalisation drift. `libslvs` is GPL and only an S03 test oracle (ADR-007). |
| FreeCAD Assembly4 | No solver: parts are attached to local coordinate systems of other parts; the tree is evaluated in order. | The observation that tree-shaped assemblies (most of the owner's designs) solve in closed form, which ASM-03 uses for the joint dialog snap (§5.3). | No loops and no drag, so no 4-bar. |
| Amalgam S03 solver (`amg_sketch::solve`, ADR-007) | Dimension-agnostic `EquationSystem` of `Residual` blocks with analytic Jacobians, damped min-norm LM, numeric rank, weighted min-norm drag. SK-12 already reuses it in 3D. | `Residual`, `EquationSystem`, `numeric_rank`, the drag formulation (§6.4). | `NativeSolver` itself: its pipeline (`Problem`, substitution, DM blocks) is crate-private and assumes additive updates of a flat parameter vector, while rotations need a retraction. |

## 1. Geometry lives in component space

- Bodies, sketches and construction geometry of a component are evaluated in that
  component's own coordinates. Every occurrence of the component shows the same geometry,
  placed by the occurrence transform (instancing, S10 "one instance per (occurrence path,
  body)").
- An occurrence transform is relative to its parent component. The world transform of an
  occurrence path `[o1, …, on]` is `P(o1) · … · P(on)`, where `P` is the *placement* (§2).
- This is already what the code does: `EvalBody::component` is the feature context, and
  DOC-03's cache key hashes the context id but not any occurrence transform. Moving an
  occurrence therefore never invalidates the geometry of the moved component.
- Features that read geometry from another component use `EntityRef::InContext { path, target }`.
  DOC-03 hashes the *authored* transforms of every occurrence on such a path. Such features see
  authored transforms, not solved placements (§2). This is deterministic and avoids a cycle
  (features → geometry → joint solve → placements → in-context features). After a commit, the
  two are equal unless a geometry edit moved a joint origin (§3).

**Transforms must be rigid or mirrored.** `amg_core::Transform` is a `DAffine3`, so it can hold
scale and shear. Occurrence transforms must have an orthonormal linear part
(`|det| = 1`, columns orthonormal within 1e-9). Scaled occurrences are refused with
`ModelError::InvalidValue`. A mirrored occurrence (det = −1, SOL-24 mirror) is allowed. The
solver moves occurrences by a rigid increment applied on the left (§6.1), so mirroring is kept.

## 2. Authored transforms vs solved placements

- **Authored** (`Design`): `Occurrence::transform` (last committed position), `grounded`, and
  the joint states of §5.5. Undo/redo works on these.
- **Evaluated** (`Evaluated::placements`): the placement of every occurrence after the
  assembly stage of recompute (§7). Display, picking, measure, mass properties (INS-10),
  interference (INS-04) and STEP export (IO-06) use placements.
- Without joints the two are equal: ASM-01 copies authored transforms into placements, and
  ASM-04 adds the solve.

## 3. Moving occurrences: no pending positions (QUESTIONS #59)

Fusion marks a moved component as pending until Capture Position adds a timeline entry or
Revert throws the move away. Autodesk's own advice is to use captures sparingly because they
"load up the timeline". We propose a simpler model:

- Every move (gizmo, drag, drive, motion-study "capture frame") ends in one
  `Command::SetOccurrenceTransforms` (one undo entry). Undo is the revert.
- There are no position entries in the timeline. Positions are authored state, like body
  visibility.
- A geometry edit that moves a joint origin changes the solved placements at the next
  recompute. The authored transforms stay as they were until the next move commits. The
  browser shows a "positions out of date" badge on the root when placements and authored
  transforms differ by more than the model tolerance. Its action, "Update positions", commits
  the placements (one command). This matters only for in-context features, which read authored
  transforms (§1).

The owner decides between this and Fusion's pending/capture model (QUESTIONS #59). The rest of
this note does not depend on the answer: capture would only add a "pending" flag in the UI
that holds the commit back.

## 4. Joints are timeline features

Joints, as-built joints, rigid groups, motion links and joint origins are timeline features
in `amg-features::assembly` (Fusion shows them in the timeline too). Reasons:
- Joint origins reference geometry (`EntityRef`s) that must be resolved through the naming
  framework (DOC-06) and CON-03 (`amg_features::construct::resolve`). Only features can do that
  during recompute, and `amg-doc` cannot call `amg-features`.
- The timeline ordering rule then gives Fusion's behaviour: a joint placed after a feature sees
  that feature's geometry. Rolling the marker back removes the joint.
- Health, suppression, editing and undo work with no new machinery.

A joint feature's `compute` resolves its origins to frames in component space and returns
an `AsmItem` (`FeatureOutput::assembly`, new, additive). It reads no placements, so its cache
key does not depend on positions: the feature lists `Dependency::Entity(target)` for the *inner*
target of each origin (not the `InContext` wrapper), so moving an occurrence does not re-run
it. The occurrence path is stored in the feature data and checked in compute.

`Component::joints: Vec<JointId>` (S01 provisional) stays in the struct, always empty, so
existing files load (`deny_unknown_fields`). It is dropped at the next `FORMAT_VERSION` bump.
`JointId` stays unused.

## 5. Frames, joint types, values and limits

### 5.1 Joint origin = frame on geometry
A joint origin is a right-handed orthonormal frame `F` (a rigid `Transform`: columns x, y, z,
translation = origin) in the component space of the occurrence it is attached to.
`JointOrigin { path, at: OriginAt, align: Align }`:
- `path`: occurrence path from the joint's context component to the occurrence whose geometry
  is used (empty = the context component itself, which never moves in its own context).
- `OriginAt` (the snap; Fusion keypoints):
  - `Vertex(EntityRef)`; `EdgeStart`, `EdgeMid`, `EdgeEnd` (line or curve); `CircleCenter`
    (circular or elliptic edge); `FaceCenter` (planar face: area centroid; cylinder/cone:
    axis point at the middle of the face's axial extent); `AxisPoint { face, at: Start | Mid |
    End }` (cylinder, cone, torus, sphere);
  - `Construction(ConstructionId)` (plane, axis or point), `ComponentOrigin` (identity);
  - `Point3InFace { face, uv }`: an arbitrary picked point, stored as surface parameters.
- **z axis:** face normal (planar) or axis (rotational face), circle normal, line direction,
  plane normal, construction axis direction. For a vertex: the z of the frame the dialog picked
  it from (stored as `z_hint`).
- **x axis (deterministic rule):** the projection of the component's +X onto the plane ⊥ z;
  if that is shorter than 1e-6, the projection of +Y. This is Onshape's convention for implicit
  mate connectors. Fusion lets the user re-orient; we do the same with `Align`.
- `Align { flip: bool, angle: Expr, offset: [Expr; 3], x_axis: Option<EntityRef> }`: flip turns
  the frame 180° about x; `x_axis` re-aims x along an edge/axis projected onto the plane;
  `angle` rotates about z; `offset` translates along the frame axes. Exprs become model
  parameters (SK-19 `exprs_mut` rule), so joint offsets are parametric.

Geometry comes from `construct::resolve::resolve_input` (CON-03) on the inner target. The
result is in the target body's component space (Interface request to S05).

### 5.2 Joint types
A joint relates frame `A` (on the "to" side, which stays put in the dialog) and frame `B` (on
the "from" side, which moves in the dialog, like Fusion's first selection). The relative pose
is `T = A⁻¹ · B`, written in A's axes. The joint motion axis is A's z unless the joint
chooses another axis (`JointAxis::{X, Y, Z}`); the frames are rotated so that the motion axis
is z before the residuals below are built.

| Type | Free DOF (in A's frame) | Residual rows (6 − DOF) | Joint values |
|---|---|---|---|
| Rigid | 0 | coincident origin (3), z ∥ (2), x locked (1) | — |
| Revolute | rot z | coincident origin (3), z ∥ (2) | angle θ |
| Slider | trans z | origin on A's z line (2), z ∥ (2), x locked (1) | distance d |
| Cylindrical | rot z, trans z | origin on A's z line (2), z ∥ (2) | θ, d |
| Pin-slot | rot z, trans along slide axis s ⊥ z | origin on the line through A along s (2), z ∥ (2) | θ, d |
| Planar | trans x, y; rot z | origin on A's xy plane (1), z ∥ (2) | θ, dx, dy |
| Ball | 3 rotations | coincident origin (3) | pitch, yaw, roll (ZYX) |

Rows: "coincident origin" `o_B − o_A`; "on line" the two components of `o_B − o_A` ⊥ the line;
"on plane" `z_A · (o_B − o_A)`; "z ∥" `z_A · x_B`, `z_A · y_B`; "x locked" `x_A · y_B`. Angular
rows are multiplied by the model length scale `L_c` (the S03 rule: rows are mm). "z ∥" is also
satisfied by antiparallel axes; the solver starts from aligned frames and the result is
checked (`z_A · z_B > 0`), otherwise the joint reports `Flipped`.

**Values.** θ = `atan2(y_A · x_B, x_A · x_B)` (rotation of B about A's z), unwrapped to the
value nearest the joint's stored state (§5.5). Slides are components of `o_B − o_A` in A's
axes: d = `z_A · (o_B − o_A)` (slider, cylindrical), `s_A · (o_B − o_A)` (pin-slot), x and y
(planar). Ball values are the ZYX angles of T's rotation. Values are what limits, drives,
motion links and the UI read.

### 5.3 Snap placement (closed form, ASM-03)
For a joint whose "from" occurrence is otherwise free, the dialog places it in closed form:
`P_from = P_to-side · A · J(values) · B⁻¹`, with J = identity for new joints (values 0). This is
exact for trees, needs no solver, and is what ASM-03 ships so joints are usable before ASM-04.
ASM-04 replaces it by a solve when the occurrence is in a loop or has other joints.

### 5.4 Limits and rest
`JointLimits { min, max, rest }` per joint value, each an optional `Expr` (mm or rad). Limits
are enforced by the solver as an active set (§6.3). `rest`: when a drag of the joint ends, the
drag tool drives the value back to `rest` before committing (Fusion "Rest").

### 5.5 Joint states (authored)
Revolute angles beyond ±π, limits such as −720°..720° and motion links (§8.1) need the
unwrapped value, which a transform does not hold. `Design::joint_states:
PMap<FeatureId, JointState>` stores the last committed unwrapped values. It is written by the
same `SetOccurrenceTransforms` command that commits a move. Missing entries are extracted from
the placements (wrapped to (−π, π]).

## 6. Joint solver (`amg-doc::joints`, ASM-04)

### 6.1 Unknowns and parameterisation
- Maximal coordinates: one rigid pose per unknown occurrence. The solver works on increments
  `ξ = (δ, ω) ∈ R⁶` around the current pose, applied on the left in the parent component's
  space: `P ← Exp(ξ) · P` with `Exp` the SE(3) exponential (rotation by `exp([ω]×)`, then
  translation). After each step the increments are folded into the poses and reset to 0
  (retraction). No quaternion norm rows, no gimbal problem.
- Jacobians are analytic and short: for a world point `q = P·l`, `∂q/∂δ = I`,
  `∂q/∂ω = −[q − t]×`; for a direction `d = R·u`, `∂d/∂ω = −[d]×`. Each joint is one
  `amg_sketch::solve::Residual` over the 12 increments of its two occurrences (6 if one side is
  fixed), FD-checked in tests like S03's residuals.
- The equation system is `amg_sketch::solve::EquationSystem` (row bookkeeping, dense Jacobian);
  the outer loop (LM with retraction) is ours, about 400 lines. Dense linear algebra uses
  `nalgebra` inside `amg-doc` (ADR-006: crate-internal), as `amg-sketch` does.

### 6.2 Decomposition
1. **Per context.** A joint's context is the component it was created in (its `FeatureNode::context`).
   Its unknowns are the *first* occurrences of its two paths, which are direct children of the
   context. Deeper occurrences on the paths are fixed relative to them, at their own
   context's solution. Contexts are solved bottom-up (deepest first, then component id order).
   A component placed several times is solved once, so all its copies move alike (limitation:
   two copies of a subassembly cannot be in different joint positions; Fusion has the same
   rule unless the copy is made independent).
2. **Rigid clusters.** Rigid joints and rigid groups are substituted, not solved: union-find
   over occurrences with the captured relative transforms gives clusters, each one unknown. A
   cluster with a grounded member is fixed. Inconsistent rigid loops (two rigid paths with
   different relative transforms) are reported `Overconstrained` on the later joint.
3. **Connected components** of the remaining joint graph are independent systems.
4. Fixed: the context's own geometry, grounded occurrences and clusters, and every occurrence
   with no joint (it simply keeps its authored transform).

### 6.3 Solve
- Seed: the authored transforms. LM with min-norm steps: under-constrained systems move as
  little as possible from the seed (the S03 rule), so free chains stay where the user left
  them. This is the warm start the FreeCAD wiki asks the user to help with.
- Convergence: max |row| ≤ 1e-9 mm (rows in mm); at most 50 iterations.
- **Limits** (active set): after convergence, every joint value outside its `[min, max]`
  gets an equality row at the violated bound and the system is re-solved; at most 8 rounds.
  A value that still violates gives `LimitViolated`.
- **DOF**: `numeric_rank` of the Jacobian at the solution gives the remaining DOF per system,
  shown by the drag tool and the joint glyphs.
- **Diagnostics** per joint feature (`AsmDiagnostic`): `NotConverged`, `Overconstrained`
  (conflicting rows: the residual stays above 1e-6 mm, the circuit found by the S03
  leave-one-out idea over joints), `Redundant` (rank deficit with zero residual: informational),
  `Flipped`, `LimitViolated`, `BadPath`. The engine turns them into `Health::Warning` on the
  joint's node; the joint has its effect as far as the solver got.
- **Determinism**: fixed ordering everywhere (component id, occurrence id, feature order), no
  parallel reductions, no hash iteration. The same input gives bit-identical placements
  (ASM-04 tests 100 runs).

### 6.4 Drag
`AsmDrag` keeps the system of the grabbed occurrence's connected component. Per frame:
1. The cursor target is a point (grab point dragged in the view plane) or a full pose
   (triad gizmo).
2. A weighted min-norm step moves the grab point toward the target inside the null space of
   the constraint Jacobian (the S03 drag formulation), then a Newton projection restores the
   constraints. Limits clamp with the active set.
3. Contact sets (§8.3) shorten the step if enabled.
4. The frame result is a `Placements` overlay for display only. Mouse-up gives one
   `SetOccurrenceTransforms` command (with joint states and, if set, rest values applied).

Drag runs synchronously on the UI thread: a 50-occurrence mechanism costs well under 1 ms per
frame (§9). Systems above 2 000 unknowns run each frame step as a job and show the last
finished frame (never blocks > 16 ms).

## 7. Assembly stage of recompute

After the timeline pass, `amg-doc` runs the assembly stage:
1. Collect the `AsmItem`s of every computed (not suppressed, not failed) node before the
   marker, in timeline order.
2. Key: blake3 over the items, authored transforms, grounded flags and joint states. If the key
   equals the previous evaluated key, reuse the previous placements.
3. Solve (§6) and store `Evaluated::placements` and `Evaluated::asm_diagnostics`.
4. `DocEvent::OccurrenceTransformChanged(o)` is emitted for every occurrence whose placement
   changed in the swap (in addition to authored changes, as today).

Before ASM-04 lands, step 3 is "placements = authored transforms" (ASM-01).

## 8. Motion and contact

### 8.1 Motion links
`assembly.motion_link { a: FeatureId, a_value: ValueSel, b: FeatureId, b_value: ValueSel,
ratio_a: Expr, ratio_b: Expr, reverse: bool }`. Fusion asks for two values (e.g. 360° and
10 mm). The residual is `(v_b − v_b0) − k · (v_a − v_a0)` with `k = ±ratio_b / ratio_a`, where
`v_a0`, `v_b0` are captured at creation and stored. Values are unwrapped (§5.5), so a gear
pair stays in phase over many turns.

### 8.2 Drive joints and motion studies
- **Drive joint** (tool): sets one joint value (field or slider), solves with a temporary
  equality row, commits on OK.
- **Motion study** (`ExtRecord` kind `assembly.motion_study`, not a timeline feature, as in
  Fusion): `{ name, steps: u32 (default 100), tracks: [{ joint, value: ValueSel, keys:
  [(step, Expr)] }] }`, linear between keys. Playback samples every step: drive rows for all
  tracks, solve warm-started from the previous step, contact if enabled. A job computes all
  frames into an in-memory `Vec<Placements>` (not saved); the panel plays them. "Capture this
  frame" commits that frame's transforms.

### 8.3 Contact sets
`ExtRecord` kind `assembly.contact`: `{ mode: Off | Sets | All, sets: [{ name, members:
Vec<OccPath> }] }`. Fusion's warnings apply: contact is slow on gears and cams, so it is off by
default and limited to pairs inside a set.
- Each member body is a mesh: mesh bodies as is; solids through BREP-08 tessellation (display
  "Fine" preset), cached per `BodyId` and cache key.
- After each drag/drive/playback step, test every pair in the set: BVH-vs-BVH overlap under the
  relative placement (S07 request: `MeshSpatial::overlaps`). Penetration means an actual
  triangle–triangle intersection; touching within the model tolerance is allowed.
- On penetration: bisect the step between the last free state and the target (re-solving at
  each midpoint) until the step is below 1e-3 mm / 1e-4 rad or 12 halvings; stop there. No
  sliding response (Fusion stops too).
- Accuracy is that of the tessellation (chordal deviation of the preset), which the drag tool
  states in its tooltip.

## 9. Budgets (reference machine)

| Case | Budget |
|---|---|
| Assembly stage, 50 occurrences, 60 joints incl. one closed loop | < 5 ms median |
| Assembly stage, 500 occurrences, mostly grounded/rigid | < 20 ms median |
| Assembly stage with unchanged key | < 0.2 ms |
| Drag frame, 4-bar or 50-occurrence mechanism | p95 < 2 ms (UI never blocks > 16 ms) |
| Drag frame with one contact pair, 2 × 100 k triangles | p95 < 8 ms |
| Motion study, 4-bar, 360 steps | < 200 ms total |
| Snap placement (closed form) | < 50 µs |

These sit inside the vision's "typical 50-feature part recompute < 500 ms" and "UI never
blocks > 16 ms".

## 10. Golden mechanisms (`tests/golden/asm/`)

Built from a lane-E test feature (`test.joint`) that emits frames directly, so the golden tests
need no OCCT. Each compares solved placements with analytic values within 1e-9 mm / 1e-9 rad:
1. Planar 4-bar, crank-rocker (Grashof): rocker angle vs the closed-form 4-bar position
   equation over 36 crank angles, both assembly branches kept (no flips).
2. Slider-crank: piston position `x = r cos θ + sqrt(l² − r² sin² θ)` over 36 angles.
3. Gear pair by motion link, ratio 2:1, 5 turns: phase error 0.
4. Pin-slot (Geneva-like) with limits: values stay in `[min, max]`.
5. Spatial RSSR (revolute–ball–ball–revolute) linkage: closure residual ≤ 1e-9 mm and output
   angle vs a reference computed by a dense Newton solve in the test.
6. Over-constrained rigid loop: `Overconstrained` reported on the right joint, no panic.
