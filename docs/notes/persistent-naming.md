# Persistent naming: name grammar, naming rules, resolver, golden corpus — design note

| | |
|---|---|
| **Spec** | [S01](../specs/S01-document-model.md) §4 (framework), DOC-11 |
| **Decides** | the final `TopoName`/`NameSeg` grammar and `GeomFingerprint`; how features build names (`NamingBuilder`); the naming rules for each S04 feature, mesh face groups and imported bodies; the resolver (candidate sets, fingerprint metric, thresholds, warnings); failure UX; storage and versioning in `.amg`; performance budgets; the golden TNP corpus format and its cases; the API DOC-06 implements |
| **Produced by** | DOC-11 (2026-10-20) |
| **Needed before** | DOC-06 (framework), DOC-13 (full corpus), S04 SOL-01..SOL-13 (naming rules), S07 mesh refs, S11 re-pick UI |
| **Status** | **Accepted** (owner, 2026-10-21; revised 2026-10-20 after review, §16). §15 answered: every recommendation taken. Thresholds marked *provisional* are re-tuned by DOC-06/DOC-13 runs and the change is recorded in §13 |

The roadmap lists persistent naming as project risk #2. This note fixes the scheme so DOC-06
can be implemented without design decisions of its own. The provisional types in
`crates/amg-doc/src/naming.rs` (DOC-01) are revised in §3 and §10. S01's Interface changes
section (2026-10-20, DOC-11) lists every change.

---

## 0. Decisions at a glance

1. **Faces carry history names. Edges and vertices are named by their adjacent faces.**
   A face name starts with its generator (`Gen { feature, role, from }`) and records only the
   structural events that happen to it later (`Split`, `Merge`, `Inst`). Edge and vertex names
   are recomputed after every operation from the sorted multiset of their adjacent face names
   (`Bound`). This follows Kripac (1997) and Bidarra et al. (2005). It survives anything that
   keeps face adjacency, and boolean section edges, which OCCT gives no history for, need no
   special case.
2. **There is no `Modified` segment.** A face that one input face maps onto one-to-one keeps
   its name unchanged, so inserting, suppressing or reordering a modifying feature (draft,
   fillet neighbours, boolean targets) never changes the names of untouched faces. This
   removes `NameSeg::Modified` from the provisional grammar.
3. **Generated names come from stable authored inputs, not evaluated indices.** Sketch entity
   ids (stable, never reused, S03), the outer-loop source set of a profile region, the
   per-selection `key` of each picked `TopoRef`, pattern instance numbers and `FeatureId`s are
   stable. In-shape indices, OCCT result order, `ProfileId`s (they change when a region gains a
   hole) and the *name* stored in a `TopoRef` (Accept match rewrites it) are not.
4. **Indices inside names are hints, never identity.** `Split.index` and `Bound.index` come
   from a geometric order that can permute. The resolver treats them as wildcards, much like
   Onshape's `unstableIdComponent`, and picks with the fingerprint. An index alone never
   decides when the fingerprint disagrees with it.
5. **Resolution is a pure function of authored state and the evaluated input state.** It
   never depends on edit history, previous evaluations or `HashMap` order (ADR-014
   determinism). Stale fingerprints are refreshed only by an explicit, undoable
   "Accept match" or re-pick command.
6. **Never a silent wrong match.** A match is quiet only when the candidate is unique after
   wildcarding, or when name and fingerprint agree with a clear margin. Every other success
   carries a `ResolveWarning`, and anything less certain is a `ResolveError`. Ties are errors.
   The rule holds *relative to the naming semantics* (an edge is "the boundary between faces A
   and B", a pattern ref is "instance k"); §6.7 lists the cases where those semantics can
   differ from what the user meant, and how each is bounded.
7. **Tombstones.** Each `BodyNaming` remembers which feature removed which face name, so a
   dangling reference fails with "removed by Fillet3" instead of falling back to a geometric
   guess.
8. **Body lineage.** `BodyChangeKind::Consumed { into }` and `Created { derived_from }` let
   references follow faces into the body they were combined into or split off to.

---

## 1. Requirements

| # | Requirement | Source |
|---|---|---|
| R1 | Features store `EntityRef`s, never indices; resolution happens at recompute | architecture §Core data model, S01 §4 |
| R2 | The resolver never returns a wrong entity silently: error or warning when unsure | S01 §4 hard rule; DOC-13 "0 silent wrong resolutions" |
| R3 | Deterministic: same `design.json` gives the same resolutions on every machine and run | ADR-014, S01 §3 cache keys |
| R4 | Features never mutate authored state during recompute | ADR-014 |
| R5 | Works on S02's index-based `ShapeHistory` (Modified/Generated/Kept/Deleted + roles + tags) without OCCT types | ADR-002, S02 §6 |
| R6 | Survives: dimension edits; profile edits that add/remove sketch curves; inserting, suppressing, deleting and reordering upstream features; pattern count changes; body combine/split | DOC-11 corpus categories |
| R7 | Covers mesh face groups (S07) and imported/baked bodies | S01 §4, DOC-11 (c) |
| R8 | Cheap: naming and resolution are a small fraction of the kernel op they follow | vision budgets ("50-feature edit < 500 ms") |

---

## 2. Prior art: what we take and what we reject (Done-when (a))

| Source | What it does | Take | Reject |
|---|---|---|---|
| **Kripac 1997**, *A mechanism for persistently naming topological entities in history-based parametric solid models*, CAD 29(2):113–122 | A topological ID system. Faces get ids from the generating operation and its input entity. Edges and vertices are identified by their adjacent faces. A face-graph match handles splits and merges, with extra info where face ids alone are ambiguous. | Faces primary; edges/vertices from adjacent faces (§3.3); explicit split/merge records | Kripac's matching heuristics over the whole old/new face graph (needs both graphs, so it is path-dependent; we match name→candidates only) |
| **Capoyleas, Chen, Hoffmann 1996**, *Generic naming in generative, constraint-based design*, CAD 28(1):17–26 (TR CSD-TR-94-011) | Names express how an entity was *generated* (the generator chain), not where it sits, so they are "generic" across re-evaluation. Matching after edits is a separate step. | Generator chains as the name (`Gen { feature, role, from }`); naming and matching as separate concerns (`NamingBuilder` vs resolver) | Their full constraint-based design formalism; we only need the naming half |
| **Marcheix & Pierra 2002**, *A survey of the persistent naming problem*, Proc. 7th ACM Symp. Solid Modeling and Applications, 13–22 (and Marcheix 2006, *A persistent naming of shells*) | A taxonomy of the problem: naming (identify), matching (re-find), and the split, merge and disappear cases. Pure topology names become ambiguous after splits, so a geometric complement is needed. | The taxonomy organises §6 and the corpus categories; the split/merge/disappear outcomes map to `NewSplit`, `Widened` and `NotFound` | Shell-level naming (we name faces/edges/vertices only) |
| **Bidarra, Nyirenda, Bronsvoort 2005**, *A feature-based solution to the persistent naming problem*, CAD&A 2(1–4):517–526 | References use persistent *feature faces* instead of boundary faces. Edges are identified by feature-face pairs, and leftover ambiguity is resolved with half-spaces defined by the features. | Edges named by face pairs (`Bound`); generated names derived from *feature* inputs (authored refs, sketch ids), not boundary indices | Feature-semantic half-space disambiguation in general (too feature-specific); we use a geometric fingerprint instead, plus a deterministic geometric index as a hint |
| **Farjana & Han 2018**, *Mechanisms of persistent identification of topological entities in CAD systems: a review*, Alexandria Eng. J. 57(4):2837–2849 | Review that classifies topology-based, geometry-based and hybrid methods. Concludes topology and geometry must be combined. | The hybrid design: a topological name first, a geometric fingerprint to disambiguate and as a last-resort fallback | Pure geometry-based identification (breaks under dimension edits, which are the most common edit) |
| **FreeCAD 1.0 element map** (realthunder's toponaming algorithm, merged in FreeCAD 1.0, Nov 2024; `src/App/ElementMap*`, `MappedName`; wiki "Topological Naming Algorithm") | Mapped names such as `Face6;:M2;FUS;:T1:5:F`, built from OCCT `Generated`/`Modified` history. Multiple sources give `;:M(…)` merge lists. Unnamed sub-elements are named from named parents (`;:U`) or children (`;:L(…)`). Long names are string-hashed. Splits are disambiguated by OCCT's reporting order. The wiki itself says the "related element" guess "may pick the wrong one" and needs user attention. Overhead on a 148-object model: recompute 40 s → 52 s; file 17.8 → 22.6 MiB uncompressed. | The element map shape (a name per in-shape index, `BodyNaming`); a core that doesn't know per-feature rules; merge lists; naming leftover elements from neighbours | Op codes and `;:M` index segments for every modification (names grow per feature; we drop `Modified`); relying on OCCT result order for split identity (we sort geometrically and still treat the index as a hint); string-encoded names (we use typed segments with `Arc` sharing) |
| **Onshape** (public FeatureScript docs, forum answers from Onshape developers; no public patent found) | Every operation gets a hierarchical `Id`, and "the full id hierarchy must reflect creation history". Downstream references are *queries* (e.g. `qCreatedBy(id + "fillet")`). `unstableIdComponent` marks id components (indices) that queries should treat as wildcards. `opHole`'s `identities` parameter "stabilizes references to the holes with respect to upstream changes". Transient ids are converted into stable queries when stored. | `FeatureId` + role + authored source as the creation id; *unstable components are wildcards* (`Split.index`, `Bound.index`, §6.2); naming holes by their sketch point (≈ `identities`); converting a pick into a stored name + fingerprint (`topo_ref_at`) | Arbitrary query language in features (S04 features hold resolved refs, not queries). Rule fillet (SOL-15, P2) can add queries later |
| **CadQuery / build123d** (selectors; CadQuery `.tag()`; build123d `Select.LAST` / `Select.NEW`, "Topology Selection" docs) | Avoid stored names entirely. Select by geometric properties (`>Z`, `sort_by(Axis.Z)`), by tags placed in the script, or by "created by the last operation". The build123d tutorial warns that list indices are not stable. | "Created by operation X" is our `Gen.feature`; property selectors are a model for SOL-15 rule fillets; the corpus harness uses property selectors (`$pick near/class`) to *create* refs | Selector-only references for interactive picks (they re-select a *different* entity when geometry moves, with no error, which breaks R2) |
| **OCCT TNaming / OCAF** (`TNaming_Builder`, evolutions `PRIMITIVE`, `GENERATED`, `MODIFY`, `DELETE`, `REPLACE`, `SELECTED`; `TNaming_Selector`, name types `IDENTITY`, `MODIFUNTIL`, `GENERATION`, `INTERSECTION`, `UNION`, `SUBSTRACTION`, `FILTERBYNEIGHBOURGS`, `ORIENTATION`, …) | Records shape evolution per label in an OCAF tree. A selected sub-shape is named by a solvable expression, e.g. an edge as the INTERSECTION of two named faces, filtered by neighbours when ambiguous. | The evolution vocabulary (our `Generated`/`Modified`/`Deleted`/`Kept` via S02); "edge = INTERSECTION of faces" (= `Bound`); neighbour filtering ≈ our fingerprint step | OCAF itself (an OCCT type leak, against ADR-002; a mutable label tree, against ADR-014) |
| **SolveSpace** (`src/sketch.h`: `EntityKey { input, copyNumber }`, `Group::Remap(hEntity in, int copyNumber)`, `REMAP_TOP`, `REMAP_BOTTOM`, `REMAP_LINE_TO_FACE`, `REMAP_LATHE_START`, …) | Each group (feature) maps every *input entity* plus a role-like copy number to a new entity handle, deterministically. An extruded line becomes a face via `(line, REMAP_LINE_TO_FACE)`. | Exactly our `Gen { feature, role, from: SketchCurve }`, with `role` ≈ copy number. Shows the scheme is enough for sketch-driven solids | Its mesh-level booleans don't name faces after a boolean (references into boolean results aren't supported), so it is no guide for splits |
| **Autodesk Fusion** (public API docs only) | `BRepFace.entityToken` is persisted. `Design.findEntityByToken(token)` returns an *array*, empty if lost, several if "subsequent modeling operations … cause the face to be split", with "the first face being the most logical match". Token strings for the same entity can differ over time. Users see lost-reference warnings and re-pick in the feature dialog. | Split references yield several candidates ranked by a best match; the UX of warning + re-pick; token ≠ identity (we compare by resolution, never by name string) | Silently taking "the most logical" candidate: we warn whenever the reference predates the split (§6.4) |

---

## 3. Name grammar and fingerprint (Done-when (b))

### 3.1 Grammar
```
TopoName  := Origin Event*                              (Arc<[NameSeg]>, oldest first)
Origin    := Gen   { feature, role, from: Source? }     faces; edges/vertices of wire bodies
           | Bound { faces: [TopoName] (sorted multiset), index, count }   edges, vertices
Event     := Split { feature, index, count }            one face became `count` pieces
           | Merge { feature, with: [TopoName] (sorted) }   this (primary) name absorbed others
           | Inst  { feature, instance }                copy `instance` of a pattern/mirror
Source    := SketchCurve { sketch, curve } | SketchPoint { sketch, point }
           | Profile { sketch, outer: [EntityId] (sorted, deduplicated) }
           | Selection { key: u32 } | Construction(id) | Topo(TopoName) | Digest(u64)
Role      := { tag: ASCII identifier ≤ 24 chars, index: u32 }
```
- `Gen` and `Bound` may only be the first segment. `Event`s are appended in timeline order.
  Within one op, a face's `Split` (from its own source) comes before its `Merge` (§4.2).
- **`Modified` is removed** (decision 2). Mod-only histories don't change names.
- **`Merge` primary**: the smallest input name by `Ord`. The others go in `with`, sorted.
- **`Profile.outer`** is the set of sketch `EntityId`s on the region's *outer* loop (each
  `ProfileEdge.source` plus its `also`), not the `ProfileId` (§5.4).
- **`Selection { key }`** is the per-selection key of a picked `TopoRef` (`TopoRef.key`, §5.1).
  It is scoped by the `Gen.feature` it appears in.
- **`Digest(u64)`** replaces a nested `Topo` source deeper than `MAX_NAME_DEPTH = 8`: a 64-bit
  FNV-1a hash of the nested name's *identity* (§6.2), so identity comparison still works and
  name size stays bounded (§9).
- **`Inst`**: instance `0` is the seed and gets no `Inst` segment. A rectangular pattern
  encodes `instance = i + (j << 16)`, with `i, j < 65 536`. Mirror copies are `instance = 1`.
- **`Bound.faces`** is the *multiset* of adjacent face names, sorted. A seam edge lists its
  face twice. A free edge of a sheet lists one face. `index`/`count` order the edges (or
  vertices) that share the same multiset (§4.3).
- **Role tags** are interpreted only together with the creating feature's kind. There is no
  global registry. A tag, once shipped, never changes meaning (§8.2).
- **Uniqueness invariant:** within one `BodyNaming`, every face name is unique, and every
  edge (vertex) name is unique among edges (vertices). `NamingBuilder::finish` enforces it
  (§4.4).

Example names (`E3` = extrude feature 3, sketch `S2`, rectangle lines 5–8, curve key 7):
```
lateral face        [Gen{E3, Lateral#0, SketchCurve(S2,7)}]
top cap             [Gen{E3, EndCap#0,  Profile(S2, outer {5,6,7,8})}]
… after cut C5 split it in two, right piece
                    [Gen{E3, EndCap#0, Profile(S2,…)}, Split{C5, 1, 2}]
top-front edge      [Bound{[ <lateral(S2,7)>, <top cap> ], 0, 1}]
fillet blend        [Gen{F6, Blend#0, Selection(1)}]          (edge picked with key 1)
fillet corner       [Gen{F6, Corner#0, Topo([Bound{[<blend 1>, <blend 2>, <blend 3>], 0, 1}])}]
pattern copy #3     [Gen{E8, Lateral#0, SketchCurve(S7,2)}, Inst{P9, 3}]
```

### 3.2 Fingerprint
```rust
pub struct GeomFingerprint {
    pub class: GeomClass,          // unchanged variants
    pub measure: f64,              // face area mm² | edge length mm | 0 for vertices
    pub centroid: Point3,          // area/length-weighted; vertex: its point
    pub axis: Option<Vec3>,        // plane normal (oriented) | cyl/cone/torus axis | line dir | circle normal
    #[serde(default)]
    pub radius: Option<f64>,       // NEW: cylinder/sphere radius, cone ref radius, torus minor, circle radius
    pub bbox: Aabb3,
}
```
- The fields come from S02 `sub_geom` (`SubShapeGeom`). `radius` comes from `Shape::surface()`
  or `Shape::curve()`. Both are existing S02 queries, so S02 needs no change.
- For mesh face groups the fields come from S07 `GroupFingerprint`: `class` from the fitted
  primitive kind (else `Other`), `measure` = area, `axis` = mean normal.
- **Quantisation for `Eq`/`Hash` stays `QUANTUM = 1e-6`** (mm, mm², unit vectors). It exists
  only so `TopoRef` and `Dependency` are hashable and deduplicable. The resolver never
  compares fingerprints for equality; it uses the metric in §6.3. The value is a
  deterministic function of the stored f64s, so platform noise never reaches it (the values
  are read back from JSON, not recomputed).
- A `TopoRef` **always** has a fingerprint when created by the UI, Python or the corpus
  harness (`topo_ref_at`). `None` is allowed (hand-written scripts, migrated files). Then
  stage 2 can't disambiguate (§6.4, `NoFingerprint`).

### 3.3 Why edges and vertices are derived
OCCT's edge history is incomplete for exactly the cases that matter. Boolean section edges are
new, with no source. Fillets replace one edge with two boundary edges. UnifySameDomain
re-creates edges. FreeCAD's algorithm needs a whole pass (`;:L(…)`/`;:U`) to name these from
neighbours. Kripac, Bidarra and OCCT's own `TNaming` `INTERSECTION` name edges by their faces
for the same reason.

Deriving every edge and vertex from its faces after each operation means one code path, no
edge bookkeeping in features, and edge names that change only when an adjacent face's name
changes. The cost is ambiguity when two faces share several edges. `Bound.index` plus the
fingerprint handle it (corpus TNP-04/05/06/07).

---

## 4. Building names (`NamingBuilder`)

### 4.1 Inputs
- `feature`: the `FeatureId` of the feature running the op.
- **Operands**, in the kernel op's input order (S02 `InputSub.operand`), each one of:
  - `Named(&BodyNaming)`: an existing body (boolean target, an earlier step's tool);
  - `Tagged(&[(EdgeId, Tag)])`: a freshly built face or wire (S02 `FaceBuild::edge_tags`).
    The meaning of each `Tag` is registered with `tag(tag, NameSource)`, e.g. sketch curve
    keys;
  - `Anonymous`: no names. Faces Generated from it get role-only names.
- `ShapeHistory` from S02, with roles and tags.
- Explicit names: `generated(kind, index, role, from)` overrides the history-derived name for
  one output sub-shape. Used where a feature knows more than OCCT, e.g. the extrude cap's
  `Profile` source.
- An optional split order (`SplitOrder::Geometric`, the default, or `PieceSlot` for
  split-body).

### 4.2 Face names (in `finish`)
For every output face `o`, in index order, the **first** rule that applies wins (the rules
are exclusive by construction, which settles outputs that are both split pieces and merge
results):
1. An explicit `generated(Face, o, …)`.
2. **Modified/Kept sources.** Let `S = {s : s is an input face with Kept(o) or Modified(…o…)}`.
   If `S` is non-empty:
   - For each `s ∈ S`, its *piece name* is `n_s = name(s)` if `s` has exactly one output, else
     `name(s) + Split{feature, i, k}`. `k` is the number of `s`'s outputs and `i` is `o`'s
     position among them in the split order (§4.3 sort key, or the piece slot under
     `PieceSlot`).
   - `|S| = 1`: `n_s`.
   - `|S| ≥ 2` (merge, including a coplanar overlap region that a fuse reports as Modified from
     both operands): primary `p = min(n_s)`, then `p + Merge{feature, with: the other n_s,
     sorted}`. So segment order is always Split (inside each piece name) before Merge.
   - Generated relations to `o` are ignored when `S` is non-empty (identity continues).
3. **Generated sources.** Let `G` be the input sub-shapes (any kind) with `Generated(…o…)`.
   For each `g ∈ G`, `n_g = Gen{feature, role(o), from: source(g)}`, where `source(g)` is the
   tag's `NameSource` if `g` is a tagged edge, `Selection{key}` if the feature registered `g`
   as the resolution of a picked ref (`NamingBuilder::selection`, §5.1), else
   `Topo(name(g))` (`Digest` past `MAX_NAME_DEPTH`).
   - `|G| = 1`: `n_g`. If `g` generates `k > 1` faces, they get `Split{feature, i, k}` from
     dedup (§4.4); this is the one-edge-several-blend-faces fillet case.
   - `|G| ≥ 2` (e.g. one fillet face spanning two collinear edges between the same two faces):
     primary `min(n_g)` plus `Merge{feature, with: others}`, as in rule 2.
4. **Role only**, no source (primitive faces, fillet corner patches, pipe transition faces):
   named in a second pass, after rules 1–3 have named every other face, from the faces next
   to it: `Gen{feature, role(o), from: Topo([Bound{faces: sorted names of the adjacent faces
   named by rules 1–3, 0, 1}])}`. Primitives instead use their fixed role tags with
   `from: None` (§5.2). A role-only face with no named neighbour gets `from: None`. This is
   how fillet corners are named without vertex history (S02 gives `CornerPatch` no source).
5. **No source and no role**, or `OpRole::Unexplained`: as rule 4 with role `Unexplained#0`,
   plus a `KernelWarning::PartialHistory` diagnostic on the feature.
6. The default `OpRole → Role` mapping is `StartCap→StartCap`, `EndCap→EndCap`,
   `Lateral→Lateral`, `FilletBlend{_}→Blend`, `ChamferFace→Chamfer`, `CornerPatch→Corner`,
   `ShellInner→ShellInner`, `OffsetFace→Offset`, `LoftSection{0}→LoftCap#0`,
   `LoftSection{last}→LoftCap#1`, `SplitPiece{_}`→ handled as `Split`, and
   `Unexplained→Unexplained`. Role index 0 unless a rule in §5 says otherwise. Features
   override the mapping with `role(op_role, Role)`.

### 4.3 Edge and vertex names
After all face names exist:
- An edge with ≥ 1 adjacent face (S02 `edge_faces`, with seam edges counted twice via
  `is_seam`) is named `Bound{faces: sorted multiset, index, count}`.
- An edge with 0 faces (wire bodies, sketch-derived paths) gets its history name: the tag
  source if tagged, else a source edge name, else `Gen{feature, Edge#0, None}`.
- A vertex is named `Bound{faces: sorted set of incident faces, index, count}` (faces of all
  incident edges, deduplicated).
- `index`/`count` group the edges (vertices) with an equal `faces` multiset. Groups are
  formed on the multiset of adjacent face *indices* (integer keys; face names are unique, so
  this equals grouping by names) and only then turned into names.
- **Sort key (total order).** Within a group, and for split pieces (§4.2), sub-shapes are
  sorted lexicographically by the integer tuple
  `(−q(measure), q(cx), q(cy), q(cz), in-shape index)` with `q(x) = round(x / 1e-6)` as
  `i64` (mm, mm²; vertices have measure 0). Quantising first makes the comparison a total
  order (no tolerance-based "equal within", which is not transitive and can make
  `sort_by` panic or permute). Values come from one deterministic kernel build (S02 §7), so
  the same document gives the same order on every machine; a kernel upgrade that moves a
  value across a quantum boundary is F13 (indices are hints). S02 uses the same key for its
  split order (request Q5, §12).
- That is the only place an index enters a name, and it is a hint (§6.2).
- **Reuse.** If an output edge (vertex) maps 1:1 to an input one (Kept/Modified) and its
  derived name equals the input name, the input `Arc` is reused, so unchanged edges cost one
  pointer per op (§9).

### 4.4 Dedup and validation
- If two faces still have equal names (e.g. one sketch curve generating two lateral faces),
  append `Split{feature, i, n}` to each, in split order.
- `finish` fails with `NamingError` if:
  - a count doesn't match the shape;
  - a name is still duplicated (a bug);
  - an operand index is out of range.
- **Tombstones:** for each input face whose relation is `Deleted`, or that has only Generated
  outputs (it was consumed), record `removed[name(s)] = feature`. Operand tombstones carry
  over. On a merge of operands, the union is taken (`Arc<BTreeMap>`, copy-on-write).
  Tombstones are stored under the full name, but **looked up by lineage**, like candidates
  (§6.2): a tombstone `t` matches a reference `R` if `identity(t) = identity(R)` and the
  events of one are a prefix of the other's, comparing `Split`/`Merge` by feature only
  (indices and counts are wildcards). So "piece 1 of the top face" hits the tombstone of the
  whole top face, and "the whole top face" hits the tombstone of its only remaining piece.
  When several tombstones match, the one with the latest feature in timeline order is reported.
- **Fingerprints** are needed only for split, dedup and `Bound` ordering, and only for
  groups of size > 1. `finish` calls `kernel.sub_geom` for those sub-shapes only. A group
  larger than 64 switches to one `sub_geom_all` call.

### 4.5 Multi-step features
A feature runs several kernel ops (build faces → prism → boolean into the target). It uses
one builder per op: the prism step's `finish` gives the tool's `BodyNaming`, which is operand
1 (`Named`) of the boolean step. Internal heal/unify steps are composed by S02
(`ShapeHistory::compose`) before naming, so the builder sees one step per op.

### 4.6 Copies, transforms, imports, meshes
- `BodyNaming::instanced(feature, instance, shape)` appends `Inst{feature, instance}` to every *face*
  name. Edges and vertices re-derive. Used by pattern and mirror for each copy.
- Proper rigid transforms (`Shape::transformed`, a `TopLoc_Location` on shared TShapes) keep
  the TShape structure, so in-shape indices and names are reused unchanged (S02 request Q1,
  §12). `instanced` relies on this, so it takes only such shapes and checks the face/edge/
  vertex counts (`NamingError::CountMismatch` otherwise).
- **Mirrors and non-rigid transforms** do not qualify: S02's `transformed` returns
  `Err(NonRigid)` for them, and OCCT (7.6+) rejects locations with a scaled or mirrored
  transformation by default. So a mirror is a geometry copy (`gtransformed`, `BRepBuilderAPI_Transform` with copy), whose
  index order is not contractual. Those copies are named through their 1:1 `Modified`
  history with `from_history`, then `Inst` is appended to every face name.
- `BodyNaming::for_import(feature, shape)` names face `i` `Gen{feature, Imported#i, None}`.
  Valid because the imported/baked shape is frozen and S02 guarantees index stability across
  binary `.brep` round-trips.
- `BodyNaming::for_mesh(feature, groups, from)` names the groups of a mesh body (§5.3).

---

## 5. Naming rules per feature (Done-when (c))

### 5.1 Common rules (every S04 feature)
- **BodyOp (new / join / cut / intersect / new component).**
  - Target-body faces keep their names via history; tool-body faces keep theirs.
  - Faces split by the boolean get `Split{feature, …}`; coplanar faces merged by
    `merge_coplanar`/unify get `Merge{feature, …}`.
  - The feature that runs the boolean is the `feature` in `Split`/`Merge`.
  - Join into a target keeps the target's `BodyId`.
  - Combine/join of a *separate* tool body emits `BodyChangeKind::Consumed { into: target }`
    for the tool (§6.1), unless `keep_tools`.
  - **Result simplification (contract for S04, request §12).** Every BodyOp result (join,
    cut, intersect, combine) is passed through `unify_same_domain { faces: true, edges: true }`
    with the default tolerances, and its history is composed into the op's
    (`ShapeHistory::compose`) before naming. Plain BOP fuse leaves two coplanar faces that
    touch along an edge as two faces; Fusion and Onshape show one, and the corpus (TNP-20)
    assumes one. The lane-E test features do the same, so DOC-06 and DOC-13 see the same
    topology.
- **Set-valued references** (fillet edges, shell faces, draft faces, pattern faces):
  resolved with `SetPolicy::DropMissing`.
  - Members that are `NotFound`/`BodyMissing` are dropped with a warning naming them.
  - An `Ambiguous` member whose candidates are all *new-split* pieces of the referenced entity
    (the reference predates the split, so the user meant the whole entity) is expanded to
    **all** those pieces, with a `NewSplit` warning. Any other `Ambiguous` member is dropped
    with a warning listing its candidates. (Owner question 2, §15.)
  - The feature fails only if *no* member resolves.
  - **Single-valued references** (sketch plane, extent face, axis, mirror plane) use
    `AllOrError`.
- **Selection-key rule** (replaces the earlier "authored-name rule"). When a feature generates
  faces from an entity the user picked, `from` is `Selection { key }`, where `key` is the
  picked `TopoRef`'s `key` field:
  - `TopoRef.key: u32` (`#[serde(default)]`) is allocated by whoever edits the feature data
    (UI selection widget, Python, corpus harness): `max(existing keys in that feature) + 1`,
    never reused within the feature. Single-valued fields use a fixed per-field key defined
    by the feature schema. Key `0` (old or hand-written data) means "use position + 1",
    which is weaker (removing a member renumbers the later ones) and gives a
    `NoSelectionKey` lint in the feature's diagnostics.
  - **Accept match and Re-pick keep the key** (§7), so they change neither the generated
    names nor any downstream reference into them. Removing a member and adding a new one
    allocates a new key.
  - The generated name is independent of how the input resolved *and* of the name stored in
    the ref. Before this rule, Accept match on a fillet's edge renamed every blend face and
    broke downstream refs into them (review MUST-FIX 3).
  - Propagated inputs that the user didn't pick (tangent-chain edges, the faces a shell
    offsets) still use `Topo*(e)`, the evaluated name; they change only if that upstream name
    changes.

### 5.2 Feature table
`self` = the feature's `FeatureId`. `Sel(r)` = `Selection { key: r.key }` of a picked input
ref `r` (§5.1). `Topo*(e)` = the evaluated name of a propagated (not user-picked) input `e`.
`Region(p)` = `Profile { sketch, outer }` of the profile region `p` the feature resolved
(§5.4).

| Feature (S04 WP) | Faces generated | Source (`from`) | Notes |
|---|---|---|---|
| **Extrude** (SOL-02) | `Lateral#side` per profile edge; `StartCap#side`, `EndCap#side` per profile | lateral: `SketchCurve(sketch, entity)` from the builder tag (`ProfileEdge.source`; projected curves use their sketch entity id too); caps: `Region(p)`; face-profile caps: `Sel(r)` | `side` = 0, or 1 for the second side of two-sided extents. If a two-sided result unifies the laterals, the name is `Merge`d (primary side 0). Thin: `ThinInner#0`/`ThinOuter#0` from the curve, `ThinEnd#0/1` from `SketchPoint` of the open chain ends. Taper and to-object extents don't change the rules. Extent faces (to-face) keep their target names via history |
| **Revolve** (SOL-03) | `Lateral#0` per profile edge; `StartCap#0`/`EndCap#0` if angle < 360° | like extrude | Full 360° has no caps, so refs to caps fail `NotFound` after an edit to 360° (TNP-22). An edge on the axis generates no face (degenerate) |
| **Primitives** (SOL-04) | box: `XMin#0 … ZMax#0` in the box frame; cylinder/cone: `Lateral#0`, `StartCap#0`, `EndCap#0`; sphere: `Surface#0`; torus: `Surface#0` | none | Coil and pipe use the sweep rules with a generated path (`from: None`) |
| **Fillet** (SOL-05) | `Blend#0` per filleted edge; `Corner#k` per vertex blend (`k` = order around the vertex by name) | `Sel(r)` for picked edges, `Topo*(e)` for tangent-chain-propagated edges; corners: role-only rule (§4.2 rule 4: named from the adjacent blends, no vertex history needed) | Variable/chord/setback use the same names. Neighbour faces keep their names (Modified 1:1). The picked edge's tombstone is implicit, since its faces stop being adjacent (TNP-11). One edge giving several blend faces (the blend crosses a seam, or OCCT splits the stripe) → `Split` dedup; one blend face spanning several edges between the same two faces → `Merge` (§4.2 rule 3). Corner case: TNP-27 |
| **Chamfer** (SOL-05) | `Chamfer#0` per edge; `Corner#k` | as fillet | Two-distance/distance-angle chamfers store the reference face as a `TopoRef` (single-valued) |
| **Shell** (SOL-06) | `ShellInner#0` per kept face; `ShellRim#0` per edge of each removed face; `ShellJoin#0` for arc-join faces | inner: `Topo*(outer face)`; rim and arc-join faces generated from an edge: `Topo*(edge)`; from a vertex or with no source: role-only rule | S02 has only the `ShellInner`/`OffsetFace` roles, so the feature tells inner from rim by the *source kind* (Generated from a face → inner, from an edge → rim/join). Removed faces get tombstones. Per-face thickness doesn't change names. S02 request Q6 |
| **Thicken / offset** (SOL-06/09) | `Offset#0` per face; `ThickenSide#0` per boundary edge | `Topo*(face)`, `Topo*(edge)` | |
| **Draft** (SOL-06) | none (Modified 1:1) | — | Drafted faces keep their names. A split-line draft splits faces (`Split`) |
| **Scale** (SOL-06) | none | — | `gtransformed` history is 1:1 |
| **Combine** (SOL-07) | none of its own | — | BodyOp rules. Tools consumed with `Consumed{into}` (TNP-23) |
| **Split face** (SOL-07) | `Split{self,i,n}` on split faces; edges re-derive | — | Default geometric order |
| **Split body** (SOL-07) | `SplitFace#0` per piece on the cut | `Sel(r)` of the tool face, or `Construction(id)` for a plane | `SplitOrder::PieceSlot`. Pieces are ordered by signed centroid distance along the tool normal (planar tools), else by (volume desc, centroid lex). Piece 0 keeps the original `BodyId`, piece *k* ≥ 1 is `BodyId{self, k}` with `Created{derived_from: original}` (TNP-21). Silhouette split follows split face |
| **Pattern** (SOL-08), bodies/faces | copies: `instanced(self, k)` of the source names | — | Rectangular `k = i + (j<<16)`, circular/path `k` = position along the pattern. References follow the **instance number** when the count changes (TNP-17, owner question 1); a vanished instance is `NotFound` (TNP-18) |
| **Pattern**, features | the seed feature's tool is rebuilt per instance and named by the seed's rules (feature = seed), then `instanced(self, k)`; booleaned with `feature = self` | as the seed | Identical-instance optimisation (SOL-08) reuses one tool shape; names are still per instance |
| **Mirror** (SOL-08) | copy: `instanced(self, 1)` | — | A mirrored *feature* follows feature-pattern rules with `k = 1` |
| **Move / copy / align** (SOL-09) | none | — | Move keeps names. Copy makes a new `BodyId{self, slot}` named `instanced(self, slot)`, so a later combine of original and copy gives distinct names instead of dedup `Split`s. A copy is **not** lineage |
| **Press-pull, offset face, replace face** (SOL-09) | none (Modified) | — | Names kept. Replace face keeps the replaced face's name |
| **Delete face** (SOL-09) | none | — | Tombstones for deleted faces; healed neighbours keep names |
| **Sweep** (SOL-10) | `Lateral#0` per profile edge; `StartCap#0`, `EndCap#0` | lateral: `SketchCurve`/`Sel(r)` of the profile edge; caps: `Region(p)`/`Sel(r)` | Path and guide rails generate no faces. Twist/scale don't change names. A path of `m` edges gives `m` faces per profile edge: they get dedup `Split{self, i, m}` (a hint) unless S02 tags each lateral with its spine edge (request Q2); then the role index is the spine edge's position in the path's sketch-id order. `RoundCorner`/`RightCorner` transition faces use the role-only rule |
| **Loft** (SOL-11) | `Lateral#k`, k = edge index in section 0's loop after S20 correspondence; `LoftCap#0` (first), `LoftCap#1` (last) | lateral: source of section 0's edge `k`; caps: `Region(p)`/`Sel(r)` of that section | Closed lofts have no caps. Rails and centerline generate no faces. If correspondence changes edge counts, lateral names change → corpus extension TNP-X4 |
| **Hole** (SOL-12) | per hole: `Bore#0`, `CBore#0`, `CBoreFloor#0`, `CSink#0`, `Tip#0`, `Thread#0` | `SketchPoint(sketch, point)`, or `Sel(r)` for a point/edge-placed hole; `Inst{self,k}` for multi-point holes is **not** used (each hole is named by its point, ≈ Onshape `identities`) | Adding or removing sketch points never renames other holes |
| **Thread, modeled** (SOL-12) | `Thread#k` (k = turn-face order along the axis) | `Sel(r)` of the cylinder face | Cosmetic threads are attributes and have no faces |
| **Rib / web** (SOL-13) | `Lateral#0` per curve; `RibSide#0/1` | `SketchCurve` | |
| **Emboss** (SOL-13) | `Lateral#0` per glyph curve; `EmbossFloor#0` | glyph `SketchCurve` (S03 text-block generated entities) | Editing the text regenerates glyph entity ids, so refs into embossed glyphs are weak (documented, not a corpus case) |
| **Boundary fill / patch / stitch** (SOL-14, P2) | patch: `Patch#0`; sewn faces keep names | `Sel(r)` of the boundary |
 Sheet bodies have free edges `Bound{[F]}` |
| **Base feature / direct mode** (S01 §7) | `for_import(self, shape)` | — | Bodies baked on History→Direct→History are renamed by index (refs into them come only from features after the base feature) |

### 5.3 Mesh face groups and imported bodies
- **Mesh bodies** (S07):
  - `BodyNaming.groups: BTreeMap<FaceGroupId, TopoName>` (empty for solids). `faces`,
    `edges` and `vertices` are empty, since mesh triangles are never referenced.
  - A group created by a mesh feature (segmentation, cap, fill, shell side, user group) is
    named `Gen{feature, Group#<FaceGroupId>, None}`. S07 guarantees group ids are monotonic
    and never reused within a body lineage, so the id is stable.
  - Mesh ops that propagate groups (S07 attribute policy, `GroupOrigin::Boolean`) keep names.
  - **`solid_to_mesh`** (tessellation, S02 `GroupOrigin::BrepFace`) names group `g` with the
    B-rep face name of its source face. A tessellated mesh then follows upstream solid edits
    by face name, not by face index (corpus extension TNP-X1).
  - S07's `MeshEntityRef { group, fingerprint }` is replaced by `TopoRef { kind: FaceGroup,
    name, fingerprint }`. The resolver uses the same stages, with `FaceGroupId` as the
    in-shape "index" (request to S07, §12).
- **Imported bodies** (STEP/IGES via S12, baked): `for_import` index naming. Refs survive
  save/load and recompute, because the shape is frozen. **Re-import of a changed file**
  (future S12 "update from source") must map old to new faces by fingerprint once, at import
  time, and store the mapping as the new body's names. Until that exists, refs into a
  re-imported body resolve `FingerprintOnly` (warning) or fail (corpus extension TNP-X2).
- **Implicit and Form bodies**: body-level references only (`ResolveError::Unsupported`), as in
  S01 §4.

### 5.4 Sketch interaction
- Sketch entity ids (`EntityId`) are stable and never reused (S03), and are the strongest
  sources in the scheme. Deleting a curve and drawing a new one in the same place is a *new*
  entity. The resolver can only rescue that with a fingerprint (TNP-13).
- **Profile references** (which region a feature uses): `EntityRef::Profile` gains
  `fingerprint: Option<ProfileFingerprint>` (S03 type, `#[serde(default)]`). The feature
  resolves its profile by `ProfileId`, else by `ProfileSet::match_fingerprint` (S03 request,
  §12), a pure function, unlike `match_previous`. A fingerprint match is a `FingerprintOnly`
  warning on *the feature that uses the profile* (e.g. the extrude in TNP-13), never quiet.
- **Profile-derived face names** (caps, `Region(p)`) do **not** use the `ProfileId`. S03
  SK-15 hashes *all* boundary edge keys, holes included, into the id (checked in the SK-06
  implementation, `ProfileId(hash of sorted edge keys)`), so the id changes whenever a region
  gains or loses a hole, a crossing curve is added, or an endpoint key changes. Cap names use
  `Profile { sketch, outer }`, the sorted set of source `EntityId`s on the region's outer
  loop. What stays stable, and what doesn't:

  | Sketch edit | `ProfileId` of an untouched region | `outer` source set | Cap name |
  |---|---|---|---|
  | Dimension edit, same arrangement | same | same | same |
  | Add/remove a curve disjoint from the region (TNP-14, TNP-16) | same | same | same |
  | Add/remove a hole inside the region (TNP-29) | **changes** | same | same |
  | Reorder the profile list in the feature (TNP-30) | same | same | same |
  | Re-pick the profiles in the feature dialog | (re-picked) | same | same |
  | A curve crosses the region (it is now split) | changes | changes for the pieces | lost → stage 3, or `NewSplit` if a later feature splits it instead |
  | An outer-loop curve is deleted and redrawn (TNP-13) | changes | changes | lost → stage 3 |

  Two regions with the same outer source set (the two crescents of overlapping circles) get
  dedup `Split`s, and the fingerprint tells them apart. A region's identity is independent of
  which profiles the feature lists and in what order, so re-picking can't rename anything.
- Projected geometry (SK-09) is refreshed from an `EntityRef` through the same resolver. A
  warning there becomes a sketch `Warning`.
- Sketch on a face (`PlaneInput::Ref(Topo)`) is a single-valued reference (TNP-15, TNP-20).

---

## 6. Resolution (Done-when (d))

### 6.1 Where to look (bodies)
1. Search `r.body` if it exists in the input state of the resolving feature.
2. If `r.body` doesn't exist, or stage 1/2 below finds no candidate in it, also search the
   body's lineage successors, depth-first in `BodyId` order. Successors come from
   `Consumed{into}` and `Created{derived_from}` (`Evaluated::successors(BodyId)`).
   Candidates from several bodies are pooled.
3. If `r.body` doesn't exist and has no successors: `BodyMissing`.
4. A result found through a `Consumed { into }` successor is quiet (the whole body moved into
   the target, TNP-23). A result found through a `Created { derived_from }` successor (a split
   piece other than piece 0) carries `BodyChanged` unless another warning applies, because
   the entity now lives in a different body than the feature was built against.

### 6.2 Candidate set (lineage match)
Define for each name:
- `identity(n)` = the origin with nested `Topo` sources reduced to *their* identity
  (recursively), plus the ordered list of `Inst` segments. For `Bound`, the identity is the
  multiset of the faces' identities.
- `events(n)` = the `Split` and `Merge` segments, with `index`/`count` treated as
  *unstable components*.

**Face candidates** for a reference face name `R` (all three sources are pooled):
- (a) every face `F` in the searched bodies with `identity(F) = identity(R)`;
- (b) every face `F` that has a `Merge` segment with a `with` member of identity
  `identity(R)` (`F` absorbed `R`);
- (c) if `R` has `Merge { f, with: W }` segments: every face `F` with `identity(F)` equal to
  the identity of a member of `W` (the faces `R` was merged from, now separate).

Each candidate is classified by comparing its events with `R`'s, matching `Split`/`Merge`
segments **by feature** (indices and counts are wildcards). The classes, and the warning each
leads to:

| Class | When | `F` vs `R` | Warning |
|---|---|---|---|
| **same** | the same `Split{f}`/`Merge{f}` features in the same order, from (a) | same entity | none |
| **new split** | `F` has a `Split{f}` that `R` lacks (the reference predates the split) | `F ⊂ R` | `NewSplit { feature: Some(f), pieces }` |
| **unmerged** | from (c), or from (a) when `R` has a `Merge{f}` that `F` lacks (the merge no longer happens) | `F ⊂ R` | `Unmerged` |
| **widened** | `R` has a `Split{f}` that `F` lacks (that split no longer happens), or from (b), or from (a) when `F` has a `Merge{f}` that `R` lacks (`F` absorbed other faces) | `F ⊃ R` | `Widened` |

A candidate can be in more than one class (e.g. new split + widened); it carries all of them,
and the warning is chosen by the precedence in §6.4. Class **same** requires no other class.

So TNP-20 follows: `R` = (block-1 cap) + `Merge{e2, with: [block-2 cap]}`. After the edit
the caps are separate. Block-1's cap is a candidate by (a) and block-2's by (c), both
**unmerged**, `|C| = 2`, and stage 2 compares their distances (0.413 vs 0.422, ratio 0.98 >
`PICK_RATIO`) → `Ambiguous`. Before this revision, rule (c) was missing, so `|C|` was 1 and the
class was undefined.

**Edge/vertex candidates** for `R = Bound{faces: [A, B, …]}`:
- Compute the face candidate set `M(X)` for each `X` in `faces`.
- Candidates are the edges (vertices) `e` of the same kind whose adjacent-face multiset has
  the same size as `faces` and admits a one-to-one assignment with each face in some `M(X)`.
- The classification is the union of the per-face classes. A `count` that differs from
  `R.count` counts as **new split** with `feature: None` (the faces gained or lost shared
  edges; usually a split of the edge, as in TNP-05, but the resolver can't name the feature).

### 6.3 Fingerprint distance *(provisional weights and thresholds)*
For a reference fingerprint `p` and a candidate `q` of the same `TopoKind`:
```
if p.class != q.class                                → d = ∞
s     = max(diag(p.bbox), 10 · tol.linear)           // scale: size of the referenced entity
dc    = |p.centroid − q.centroid| / s
dm    = |p.measure − q.measure| / max(p.measure, q.measure, 1e-12)   // 0 for vertices
da    = angle(p.axis, q.axis) / (π/2)   // oriented for planes, sign-free for axes; 0 if both None, 1 if one None
dr    = |p.radius − q.radius| / s       // 0 if both None, 1 if one None
d     = sqrt( 1.0·dc² + 0.5·dm² + 1.0·da² + 1.0·dr² )
```
The weights favour position over size. Dimension edits change sizes more than they move
pieces relative to each other (see the TNP-05/07 numbers).

**No ∞ arithmetic.** `distance` returns `None` on a class mismatch instead of ∞ (the API in
§10 returns `Option<f64>`). Candidates with `None` are *excluded* from the ratio test, so a
class change never takes part in `d_best ≤ PICK_RATIO · d_second`; §6.4 says what happens to
them. The ratio is `d_best / d_second`, defined as 1.0 (a tie) when `d_second ≤ 1e-12`.

Constants (`naming::thresholds`, part of `NAMING_VERSION`):

| Constant | Value | Meaning |
|---|---|---|
| `PICK_RATIO` | 0.5 | stage 2 picks `best` only if `d_best ≤ 0.5 · d_second` |
| `QUIET_RATIO` | 0.25 | above this ratio a pick is a `LowMargin` warning |
| `FAR` | 1.0 | `d_best > 1.0` → `FarMatch` warning |
| `FP_ONLY_MAX` | 0.05 | stage 3 accepts only `d_best ≤ 0.05` |
| `FP_ONLY_UNIQUE` | 4.0 | stage 3 needs `d_second ≥ 4 · d_best` and `d_second > FP_ONLY_MAX` |

### 6.4 Stages
Let `C` be the candidate set (§6.2). Then:

**Warning precedence** (one `Option<ResolveWarning>` per result; the first that applies):
`NewSplit` → `Unmerged` → `Widened` → `IndexDrift` → `ClassChanged` → `LowMargin` →
`FarMatch` → `BodyChanged` → `NoFingerprint`.

1. **Unique.** `|C| = 1`. Return it.
   - `quality = Exact` if its class is *same* and its full name equals `R` (indices
     included).
   - `Widened` quality + `Widened` warning if widened.
   - `Disambiguated` + `NewSplit{pieces: 1}` / `Unmerged` for a new split / unmerged
     candidate (one piece left).
   - **Sanity check** (cheap: one `sub_geom`). If the match used a wildcard (its class isn't
     *same*, or its name differs from `R` in an index or count) and `R` has a fingerprint:
     `distance = None` → `ClassChanged`, `d > FAR` → `FarMatch`, as the warning if no
     higher-precedence one applies, and an `Exact` quality becomes `Disambiguated`. A unique,
     name-equal, *same* match is not checked: names are the identity, and dimension and
     pattern edits legitimately move entities far (TNP-17 moves the chamfer edge by
     d = 2.03).
2. **Several candidates.** `|C| ≥ 2`.
   - With a reference fingerprint: compute `distance` for all of them. Let `C_f` be the
     candidates with a finite distance, sorted by it, ties broken by in-shape index for
     determinism (the index never matters when ratios decide).
     - `C_f = ∅` (every candidate changed class): `Ambiguous { candidates: C }`.
     - `|C_f| = 1`: return it with `ClassChanged` (unless a higher-precedence warning
       applies), `Disambiguated`. Never quiet: the entity the user meant may be one of the
       candidates that changed class (a taper turns a cylinder into a cone).
     - `|C_f| ≥ 2`: if `ratio > PICK_RATIO` (ties included): `Ambiguous { candidates }`.
       Otherwise return `best`. The *name-equal* candidate, if any, is the one whose full
       name, indices included, equals `R`.
       - `quality = Exact` if `best` is name-equal, its class is *same*, no candidate was
         dropped for a class mismatch, and no warning applies.
       - Otherwise `Disambiguated`.
       - Warning by the precedence above: `NewSplit{pieces}` if any candidate is a new
         split; `Unmerged`/`Widened` likewise; `IndexDrift` if a name-equal candidate exists
         and isn't `best`; `ClassChanged` if some candidate was dropped for a class
         mismatch; `LowMargin{ratio}` if `ratio > QUIET_RATIO`; `FarMatch{distance}` if
         `d_best > FAR`; `BodyChanged` (§6.1). Otherwise no warning.
   - Without a fingerprint: if exactly one candidate is name-equal and every class is *same*,
     return it as `Disambiguated` with `NoFingerprint`. Otherwise `Ambiguous`.
   - **Set-valued refs** (`resolve_all` with `DropMissing`): an `Ambiguous` result whose
     candidates are all *new split* pieces of `R` returns all of them (warning
     `NewSplit{pieces}`); see §5.1 and owner question 2.
3. **No candidates.** `C = ∅`.
   - If `R` (a face) is in a tombstone map of the searched bodies:
     `NotFound { removed_by: Some(f) }`. No geometric guess.
   - If `R` is `Bound` and *every* face in it has candidates (the faces exist but are no
     longer adjacent): `NotFound { removed_by: None }`.
   - Otherwise the lineage is lost (generator gone, sketch curve redrawn, naming rule
     version changed, re-imported body). With a fingerprint, scan all same-kind entities of
     the searched bodies (lazy `sub_geom_all`, cached per body for the recompute).
     - Accept `best` if `d_best ≤ FP_ONLY_MAX` and uniqueness holds (`d_second ≥
       FP_ONLY_UNIQUE · d_best` and `d_second > FP_ONLY_MAX`, over entities with a finite
       distance; class mismatches never count): `FingerprintOnly`, warning
       `FingerprintOnly{distance}`.
     - Otherwise `NotFound`.
4. `Implicit`/`Form` bodies: `Unsupported`. `FaceGroup` refs on a solid, or `Face` refs on a
   mesh: `NotFound`.

### 6.5 Failure modes and how each is handled
| # | Situation | Outcome | Corpus |
|---|---|---|---|
| F1 | Dimension edit, topology unchanged | Exact (names don't contain geometry) | TNP-01, 06, 10, 15, 19 |
| F2 | Upstream sketch/profile gains or loses curves or holes; profile list reordered or re-picked; index order shifts | Exact (names use sketch ids and outer-loop source sets) | TNP-02, 03, 12, 14, 16, 29, 30 |
| F3 | Upstream feature reordered | Exact (names use FeatureIds) | TNP-09 |
| F4 | Inserted feature splits the referenced entity; pieces distinguishable | Disambiguated + `NewSplit` warning | TNP-05 |
| F5 | Same, symmetric pieces | `Ambiguous` error, candidates highlighted | TNP-04 |
| F6 | Split already seen; piece sizes swap, so the index drifts | Disambiguated by fingerprint + `IndexDrift` warning (never trust the index) | TNP-07 |
| F7 | A split the reference saw disappears | `Widened` warning | TNP-08 |
| F8 | Merge seen by the reference disappears; halves tie | `Ambiguous` (both halves are *unmerged* candidates, §6.2 (c)) | TNP-20 |
| F9 | Referenced entity consumed by an inserted feature | `NotFound` (adjacency gone / tombstone) | TNP-11, 22 |
| F10 | Pattern count changes | Exact by instance number; vanished instance → `NotFound` | TNP-17, 18 |
| F11 | Sketch curve deleted and redrawn | `FingerprintOnly` warning if it is geometrically identical, else `NotFound` | TNP-13 |
| F12 | Body combined into another / split into pieces | Exact via lineage | TNP-23, 21 |
| F13 | Kernel upgrade changes split counts or order | Same as F4/F6 (indices are hints); `ENGINE_SALT` forces recompute | — |
| F14 | OCCT history incomplete (`Unexplained` faces) | Names exist but are weak; `PartialHistory` warning on the producing feature | — |
| F15 | Stale fingerprint after many edits (pieces moved far from the pick) | Ratio test, else `Ambiguous`/`FarMatch`; "Accept match" refreshes it | TNP-07 margin |
| F16 | Fingerprint coincidence in stage 3 (a different face exactly where the old one was) | Accepted with warning. This is the only *possibly wrong* path, so it is always a warning, never quiet | TNP-13 |
| F17 | Accept match / Re-pick on an input of a generating feature | Generated names keep their `Selection` key, so downstream refs stay Exact | TNP-24 |
| F18 | One member of a set-valued selection is lost | Member dropped with a warning; the rest resolve | TNP-25 |
| F19 | Split already seen; the pieces become congruent | Fingerprint + name-equal agree → Exact; the quantised sort makes the tie-break deterministic | TNP-26 |
| F20 | Fillet corner patch referenced; upstream dimension edit | Exact (corner named from adjacent blends) | TNP-27 |
| F21 | Save/load between base and edit | Same result as without (names and keys persist) | TNP-28 |
| F22 | Referenced candidate changed geometric class | Never quiet: `ClassChanged`, or `Ambiguous` if all candidates changed | unit tests (DOC-06) |

### 6.6 What the scheme needs from OCCT history, and what OCCT doesn't promise
The naming rules use only *face* history (Modified/Generated/Kept/Deleted of faces, plus the
edge→face Generated of sweeps and fillets). Edges and vertices are derived (§3.3). Where OCCT
gives less than the rule needs, the fallback is a weaker name plus a warning, never a wrong
name.

| Op | What OCCT provides (documented or observed) | Not promised by OCCT | How the scheme copes |
|---|---|---|---|
| Booleans (`BRepAlgoAPI_*`, `BOPAlgo_Builder` → `BRepTools_History`) | `Modified(face)` = its images (split pieces, or a trimmed copy); `IsDeleted`; untouched faces pass through as the same TShape (→ `Kept`). In non-destructive mode a face whose tolerance must grow is copied and reported `Modified` 1:1 | That *every* output face is reachable from some input (no documented completeness guarantee); what Glue and fuzzy modes report; `Generated` is defined only for section edges/vertices, which we don't use | S02's completeness test (BREP-05) covers the op corpus. An output face with no source gets `Unexplained` → named from its neighbours (§4.2 rule 5) + `PartialHistory`. Refs into it are weak (stage 2/3, warned), not wrong |
| Fuse of overlapping coplanar faces | The common region is an image of the faces of *both* operands (same-domain faces) | Which operand "owns" it if OCCT reports only one | Rule 2 makes it a `Merge` (primary = min name). If only one operand is reported, refs to the other face become stage 3 (warned) or `NotFound` |
| `ShapeUpgrade_UnifySameDomain` (history since 7.3, BREP-10 test 4) | `Modified` from each merged face to the unified face; removed internal edges | Which faces it merges depends on `lin_tol`/`ang_tol`; it can merge faces that came from different features | Merges become `Merge` segments; refs to any member resolve `Widened` (warned). The tolerances are fixed S04 defaults, part of `ENGINE_SALT` |
| Fillet / chamfer (`BRepFilletAPI_MakeFillet`, `ChFi3d_Builder`) | `Generated(edge)` → blend faces, `NewFaces(contour)`, `Modified(face)` for the trimmed neighbours | One face per edge: a stripe can give several faces for one edge (seams, support changes) or one face for several edges of a chain; any `Generated(vertex)` for corner patches | §4.2 rule 3 (`Split` dedup / `Merge`), rule 4 for corners (named from the adjacent blends; S02 gives `CornerPatch` no source). A blend found only via `NewFaces` gets role `FilletBlend{contour}` → role-only rule. S02 request Q6 adds history tests for these |
| Prism / revol (`BRepSweep`) | `Generated(edge)` → lateral face exact; `FirstShape`/`LastShape` caps | — | Fully relied on (BREP-05 test 2) |
| Pipe shell (`BRepOffsetAPI_MakePipeShell`) | `Generated` from profile edges to faces; `FirstShape`/`LastShape` | Coverage for several sections; one face per (profile edge × spine edge) with no spine attribution; transition faces have no source | Laterals named by profile edge + dedup `Split` in path order; transition faces role-only. S02 Q2 asks for a test of which sections are covered and for spine-edge tags |
| Thru-sections | `GeneratedFace(edge)` for edges of the first section (newer releases also `Generated`) | Coverage for later sections; a smooth loft of sections with unequal edge counts splits edges, so one section-0 edge may generate several faces | Laterals from section-0 edges only; extra faces → dedup `Split`; S20's builder replaces this for SOL-11 (TNP-X4) |
| Shell / thick solid (`BRepOffset_MakeOffset`) | Offset faces are images of the original faces; closing faces of removed faces come from their edges | Whether the image is reported as `Modified` or `Generated` (it differs by version and join type); arc-join faces at convex edges/vertices; extra faces with `Intersection` join | The feature assigns roles by *source kind* (§5.2 Shell), not by relation type; sourceless faces use the role-only rule. S02 Q6: BREP-07 history test on 5 cases |
| Sub-shape order (`TopExp::MapShapes`) | A DFS over the TShape graph, so a `TopLoc_Location` (rigid move) keeps it | It is implementation behaviour, not documented API; not stable across OCCT versions or across geometry copies (mirror, scale) | Only `instanced` (proper rigid copies) relies on it, behind S02 Q1's tested contract. Mirrors and scales go through history (§4.6). No name uses an index except as a hint |
| Output order with parallel BOP | Deterministic per build (S02 §7 requires it) | — | Names never depend on output order; the sort key (§4.3) breaks ties by index only for entities with equal length *and* centroid (coincident), which can't occur within one group |

### 6.7 Where "never a silent wrong resolution" can still surprise
Each case below was checked against §6.2–6.4. The resolver is silent only when name and
geometry agree, and in each case it is either still warned or the behaviour follows from the
naming semantics and is documented.

1. **Symmetric parts, mirrored twins.** Twins never compete by name: they differ in sketch
   ids or `Inst` segments. Stage 3 could pick a twin only if it sits exactly where the
   original was (d ≤ 0.05), which is F16 and always a warning.
2. **Patterns.** A count or spacing change keeps "instance k", which can move (TNP-17,
   d = 2.03) quietly. If the user meant "the hole at 180°", that is a silent semantic change.
   It is bounded to pattern refs, matches Fusion and Onshape, and is owner question 1 (option
   b adds a warning when the instance moved).
3. **Identical split pieces.** If the reference predates the split, congruent pieces tie →
   `Ambiguous` (TNP-04). If it saw the split, name and position must agree for a quiet pick
   (TNP-26); if they disagree, `IndexDrift`. The remaining case, where an edit moves the
   pieces so that the wrong piece is both nearest and name-equal, gives no signal at all;
   the scheme can't tell it from a correct follow, and nor can any of the prior-art systems.
4. **Coalesced coplanar faces.** A face that gets merged resolves `Widened` (warned), whether
   it is the primary or a member. A ref to a merged face whose merge goes away resolves
   `Unmerged` (warned) or `Ambiguous` (TNP-20). A dimension edit that makes two faces
   coplanar creates a new merge → `Widened` on refs to either. None of these is quiet.
5. **The boundary jumps.** `Bound{[A, B], 0, 1}` means "the edge between A and B". If an edit
   removes the original contact and A and B touch somewhere else, stage 1 returns the new
   edge quietly. Kripac's scheme and OCCT `TNaming` (`INTERSECTION`) behave the same way. A
   general stage-1 distance check was considered and rejected: it would warn on every large,
   legitimate move (TNP-17, large dimension edits). The DOC-13 fuzz set (§11.6) counts how
   often this happens.
6. **Re-pick with a different entity** keeps the selection key, so downstream refs into the
   generated faces follow the feature quietly. That is the intent (it is how a broken
   reference heals downstream), and it only follows an explicit user action.
7. **Unexplained faces** are named from their neighbours. If a kernel upgrade makes the
   history complete, their names change → stage 3 warnings, not silent.
8. **`Digest` collisions** (§3.1): 64-bit identity hashes of names deeper than 8 levels. For
   10⁴ deep names in one body the collision probability is about 3·10⁻¹². The cached hash
   in `TopoName` only speeds up `Eq` (equal hashes still compare segments), so it can't
   cause a false match.


---

## 7. Failure UX (Done-when (e))

- **Health mapping.**
  - Any `ResolveWarning` → the feature's `Health::Warning`. `RecomputeCtx::resolve` adds the
    diagnostic automatically, so features can't forget.
  - The diagnostic is `Diagnostic { message, cause: None, entities: [the ref] }`. Errors
    use `ErrorCause::Resolve(err)`.
  - `ResolveError` on a single-valued ref → `Health::Error` (no effect, dependants
    `Upstream`, S01 §3).
  - `DropMissing` sets: a warning per dropped member; an error only if none resolve.
- **Messages** (the pattern, then the `ResolveWarning`/`ResolveError` specifics):
  - `NewSplit`: "Edge was split by Cut 5 into 2 pieces; using the closest piece." With
    `feature: None`: "The two faces now share 2 edges; using the closest."
  - `Widened`: by cause, "Face was a piece of a split that no longer happens; using the whole
    face." or "Face was merged with neighbouring faces by Join 4; using the merged face."
  - `Unmerged`: "Face was a merge that no longer happens; using the closest part."
  - `ClassChanged`: "Some candidates changed shape type (e.g. plane → cone); check the match."
  - `BodyChanged`: "Edge is now in body 'Body 3' (split off by Split 7)."
  - `IndexDrift`: "Pieces of the split face changed order; matched by position."
  - `LowMargin`/`FarMatch`: "Match is uncertain."
  - `FingerprintOnly`: "Original face is gone; matched a face with the same shape and position."
  - `NotFound{removed_by}`: "Edge was removed by Fillet 3."
  - `Ambiguous`: "Reference matches 2 edges equally well."
- **Viewport** (S11/S10): hovering the feature or the warning highlights the chosen entity in
  the warning colour. Errors draw the reference's fingerprint bbox as a ghost box, and for
  `Ambiguous` all candidates are highlighted.
- **Actions** in the timeline context menu and the warning popover:
  - **Accept match**: one undoable `EditFeature` that replaces the ref's `name` and
    `fingerprint` with `ResolutionRecord.current` (at the feature's input state) and **keeps
    its `key`**. The warning goes away, and no generated name changes (§5.1), so nothing
    downstream is affected (TNP-24). This is the only way references are refreshed
    (decision 5).
  - **Re-pick**: opens the feature dialog with the selection field focused, rolled to the
    feature's input state (Fusion behaviour). For `Ambiguous`, clicking a highlighted
    candidate commits the re-pick. Re-picking a member *replaces* it and keeps its `key`;
    removing a member and adding another allocates a new key.
  - For a set member that `DropMissing` dropped, the popover offers Re-pick and "Remove from
    selection".

  - **Accept all** on a timeline group: one transaction re-anchors every warned ref in it.
- **Scripting** (S18): `feature.resolutions` exposes `ResolutionRecord`s;
  `doc.accept_matches(feature)` is the scripted Accept match.
- Warnings are **never** auto-accepted on recompute or save.

---

## 8. Storage and versioning

### 8.1 Where names live
| What | Where | Encoding |
|---|---|---|
| `TopoRef` in feature data (authored) | `design.json`, inside each feature's `data` | serde JSON, externally tagged enums: `[{"Gen":{"feature":3,"role":{"tag":"Lateral","index":0},"from":{"SketchCurve":{"sketch":2,"curve":7}}}}]`, plus `"key"`. Nested names are written in full. Because picked inputs generate `Selection` sources, nesting comes only from propagated inputs (tangent chains, shell, corners), and `Digest` caps the depth at 8 (§3.1), so a ref stays around 0.2–1 kB and never grows exponentially along fillet-of-fillet chains |
| `EntityRef::Profile.fingerprint` | same | `#[serde(default)]`, additive |
| `BodyNaming` (evaluated) | `GeomCache` `FeatureOutput` → `.amg` `cache/` (DOC-14) | **interned**: a table of unique `NameSeg`s in first-use order, with names as index lists into it. Shared sub-names are written once. Encoding chosen by DOC-14 (serde) |
| `NodeEval.resolutions` | evaluated only | not persisted (recomputed, or loaded with the cached output) |

### 8.2 Versioning
- `pub const NAMING_VERSION: u32 = 1` in `amg-doc::naming`. It is part of `ENGINE_SALT`, so a
  change of grammar, rules or thresholds invalidates every cache. Element maps are never
  reused across naming versions.
- A grammar change that alters the meaning of stored `TopoRef`s bumps `FORMAT_VERSION` and
  adds a document migration with a fixture (S01 §8). Adding a `NameSource` or `NameSeg`
  variant is additive: older builds then refuse the file with an unknown-variant error,
  following S01's wire policy.
- **Per-feature naming rules are frozen per feature schema version.** A rule change (new
  role, different source) is a feature `v` bump. The feature keeps the old rule for data
  migrated from older `v` by storing `naming_rule: u32` (default 1) in its data, so references
  created against the old rule keep resolving. Role tags are never reused with a new meaning.
- Lost lineage after a deliberate rule change degrades to stage 3 (`FingerprintOnly`
  warnings), never silently.

---

## 9. Performance

| Operation | Target | Notes |
|---|---|---|
| `NamingBuilder::finish`, 1 000 faces / 3 000 edges / 2 000 vertices, no duplicates | < 3 ms | Estimate, to be measured: faces ≈ one `Arc` clone each; edges/vertices grouped on integer face-index keys (no name comparisons), then one allocation and one or two name comparisons per *new* name, ≈ 0.3 µs → ≈ 1.5 ms; unchanged edges reuse the input `Arc` (§4.3) |
| `finish` with split/dedup groups | + `sub_geom` per grouped sub-shape (S02: `sub_geom_all` 1 000 faces < 20 ms) | only grouped entities are fingerprinted |
| `resolve`, stage 1 | < 20 µs | lazy per-`BodyNaming` index `identity → SmallVec<index>`, with `Merge.with` member identities indexed too for candidate rules (b)/(c) (`OnceLock`, a `HashMap` never iterated for output) |
| `resolve`, stage 2 | < 100 µs for ≤ 8 candidates | one `sub_geom` per candidate |
| `resolve`, stage 3 | < 2 ms for 1 000 faces + one cached `sub_geom_all` | rare path |
| `ResolutionRecord.current` | one `sub_geom` per resolved ref | needed for Accept match and the corpus |
| Memory | 8 B per element whose name is unchanged from the op's input (shared `Arc`); ≤ 192 B per newly named element, allocator overhead included | Estimate: `Arc` header 16 B + hash 8 B + one `NameSeg` (≈ 72 B as specified; DOC-06 may box `Gen.from` to shrink it) + the `Bound.faces` vector (16 B + overhead). The earlier "≤ 64 B" was not achievable for new names. A 1 000-face body's first naming is ≈ 1 MB; later features mostly reuse. Measured with a counting allocator in the DOC-06 bench |
| Naming share of a 50-feature recompute | ≤ 5 % | DOC-06 criterion bench `naming/finish-1000`, `naming/resolve` wired into FND-08 |

`TopoName` caches a 64-bit hash of its segments in its `Arc` allocation, so `Eq` rejects
unequal names quickly. Serde and `Ord` are unaffected.

All targets in this table are estimates, not measurements. DOC-06's bench gates them. If one
is missed, the binding budget is the last row (≤ 5 % of recompute), and the miss is recorded in
§13 with the measured value.


---

## 10. API for DOC-06 (final shapes; replaces S01's provisional naming block)

```rust
// ===== amg-doc::naming =====================================================================
pub const NAMING_VERSION: u32 = 1;

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EntityRef {
    Body(BodyId),
    Topo(TopoRef),
    SketchCurve { sketch: SketchId, curve: SketchEntityKey },
    SketchPoint { sketch: SketchId, point: SketchEntityKey },
    Profile { sketch: SketchId, profile: ProfileKey,
              #[serde(default)] fingerprint: Option<ProfileFp> },   // CHANGED: + fingerprint
    Construction(ConstructionId),
    InContext { path: Vec<OccurrenceId>, target: Box<EntityRef> },
}
/// S03 `ProfileFingerprint` with Eq/Hash quantised like `GeomFingerprint`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ProfileFp(pub amg_sketch::ProfileFingerprint);

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TopoRef { pub body: BodyId, pub kind: TopoKind, pub name: TopoName,
                     pub fingerprint: Option<GeomFingerprint>,
                     #[serde(default)] pub key: u32 }                  // + selection key (§5.1)
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TopoKind { Face, Edge, Vertex, FaceGroup }                    // unchanged

#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]          // serde = Vec<NameSeg>
pub struct TopoName(/* Arc<{ hash: u64, segs: [NameSeg] }> */);
impl TopoName {
    pub fn new(segments: Vec<NameSeg>) -> TopoName;
    pub fn segments(&self) -> &[NameSeg];
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum NameSeg {                                                     // CHANGED (§3.1)
    Gen   { feature: FeatureId, role: Role, from: Option<NameSource> },
    Bound { faces: Vec<TopoName>, index: u32, count: u32 },
    Split { feature: FeatureId, index: u32, count: u32 },
    Merge { feature: FeatureId, with: Vec<TopoName> },
    Inst  { feature: FeatureId, instance: u32 },
}
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Role { pub tag: Cow<'static, str>, pub index: u32 }         // unchanged
impl Role { pub const fn new(tag: &'static str, index: u32) -> Role; }
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum NameSource {                                                  // CHANGED: + 5 variants
    SketchCurve { sketch: SketchId, curve: SketchEntityKey },
    SketchPoint { sketch: SketchId, point: SketchEntityKey },
    /// Outer-loop source ids of a profile region, sorted and deduplicated (§5.4).
    Profile { sketch: SketchId, outer: Vec<SketchEntityKey> },
    /// `TopoRef.key` of a picked input, scoped by the enclosing `Gen.feature` (§5.1).
    Selection { key: u32 },
    Construction(ConstructionId),
    Topo(TopoName),
    /// Identity hash of a nested name deeper than `MAX_NAME_DEPTH` (§3.1).
    Digest(u64),
}
pub const MAX_NAME_DEPTH: usize = 8;

#[derive(Clone, Debug, Serialize, Deserialize)]                        // Eq/Hash quantised (QUANTUM)
pub struct GeomFingerprint { pub class: GeomClass, pub measure: f64, pub centroid: Point3,
                             pub axis: Option<Vec3>, #[serde(default)] pub radius: Option<f64>,  // + radius
                             pub bbox: Aabb3 }
impl GeomFingerprint {
    pub const QUANTUM: f64 = 1e-6;                                     // confirmed
    pub fn from_sub_geom(g: &amg_brep::SubShapeGeom, radius: Option<f64>) -> Self;
    pub fn from_group(g: &amg_mesh::GroupFingerprint) -> Self;
    /// §6.3 metric; `None` on a class mismatch (never ∞).
    pub fn distance(&self, other: &GeomFingerprint, tol: &Tolerance) -> Option<f64>;
}
impl From<amg_brep::geom::GeomClass> for GeomClass { /* 1:1 */ }

pub mod thresholds {
    pub const PICK_RATIO: f64 = 0.5;   pub const QUIET_RATIO: f64 = 0.25;
    pub const FAR: f64 = 1.0;          pub const FP_ONLY_MAX: f64 = 0.05;
    pub const FP_ONLY_UNIQUE: f64 = 4.0;
    pub const W_CENTROID: f64 = 1.0;   pub const W_MEASURE: f64 = 0.5;
    pub const W_AXIS: f64 = 1.0;       pub const W_RADIUS: f64 = 1.0;
}

/// Element map of one evaluated body; index = amg-brep in-shape index (solids)
/// or FaceGroupId (meshes).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyNaming {
    pub faces: Vec<TopoName>, pub edges: Vec<TopoName>, pub vertices: Vec<TopoName>,
    #[serde(default)] pub groups: BTreeMap<u32, TopoName>,            // NEW: mesh face groups
    #[serde(default)] pub removed: Arc<BTreeMap<TopoName, FeatureId>>, // NEW: tombstones (faces/groups)
}
impl BodyNaming {
    pub fn name_of(&self, kind: TopoKind, index: u32) -> Option<&TopoName>;
    pub fn instanced(&self, feature: FeatureId, instance: u32, shape: &amg_brep::Shape) -> BodyNaming;
    pub fn for_import(feature: FeatureId, shape: &amg_brep::Shape) -> BodyNaming;
    pub fn for_mesh(feature: FeatureId, groups: &amg_mesh::FaceGroups,
                    from: Option<(&BodyNaming, &dyn Fn(u32) -> Option<u32>)>) -> BodyNaming;
}

pub enum OperandNaming<'a> {
    Named(&'a BodyNaming),
    Tagged(&'a [(amg_brep::EdgeId, amg_brep::geom::Tag)]),
    Anonymous,
}
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum SplitOrder { #[default] Geometric, PieceSlot }

pub struct NamingBuilder { /* feature, tags, role overrides, explicit names, history */ }
impl NamingBuilder {
    pub fn new(feature: FeatureId) -> Self;
    pub fn tag(&mut self, tag: amg_brep::geom::Tag, source: NameSource) -> &mut Self;
    pub fn role(&mut self, op_role: amg_brep::OpRole, role: Role) -> &mut Self;
    /// CHANGED (S02 request): one entry per operand, in kernel input order.
    pub fn from_history(&mut self, operands: &[(u16, OperandNaming<'_>)],
                        history: &amg_brep::ShapeHistory) -> &mut Self;
    /// Explicit name for one output face (or a 0-face edge); wins over history.
    pub fn generated(&mut self, kind: TopoKind, index: u32, role: Role,
                     from: Option<NameSource>) -> &mut Self;
    pub fn split_order(&mut self, order: SplitOrder, piece_of_face: Option<Vec<u32>>) -> &mut Self;
    /// Input sub-shape `sub` of operand `operand` is the resolution of a picked ref with
    /// selection key `key`: faces Generated from it get `from: Selection { key }` (§4.2 rule 3).
    pub fn selection(&mut self, operand: u16, sub: amg_brep::SubShape, key: u32) -> &mut Self;
    /// §4.2–4.4. Calls `kernel.sub_geom` only for grouped sub-shapes.
    pub fn finish(self, kernel: &dyn amg_brep::BrepKernel, shape: &amg_brep::Shape)
        -> Result<BodyNaming, NamingError>;
}
#[derive(thiserror::Error, Clone, Debug, PartialEq)]
pub enum NamingError {
    #[error("history refers to operand {0} that was not supplied")] MissingOperand(u16),
    #[error("{kind:?} count mismatch: naming {naming}, shape {shape}")]
    CountMismatch { kind: TopoKind, naming: u32, shape: u32 },
    #[error("duplicate {kind:?} name after dedup")] Duplicate { kind: TopoKind },
    #[error("kernel query failed: {0}")] Kernel(String),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MatchQuality { Exact, Disambiguated, Widened, FingerprintOnly }   // + Widened
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ResolveWarning {                                                 // NEW; precedence §6.4
    NewSplit { feature: Option<FeatureId>, pieces: u32 }, Unmerged, Widened { cause: WidenCause },
    IndexDrift, ClassChanged, LowMargin { ratio: f32 }, FarMatch { distance: f32 },
    BodyChanged { now_in: BodyId }, FingerprintOnly { distance: f32 }, NoFingerprint,
}
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WidenCause { SplitGone { feature: FeatureId }, Merged { feature: FeatureId } }
#[derive(Copy, Clone, Debug, PartialEq)]                                  // CHANGED: no Eq (f32)
pub struct Resolved { pub body: BodyId, pub kind: TopoKind, pub index: u32,
                      pub quality: MatchQuality, pub warning: Option<ResolveWarning> }
#[derive(thiserror::Error, Clone, Debug, PartialEq)]
pub enum ResolveError {
    #[error("body {0:?} does not exist at this point in the timeline")] BodyMissing(BodyId),
    #[error("referenced {kind:?} no longer exists")]
    NotFound { kind: TopoKind, removed_by: Option<FeatureId> },          // + removed_by
    #[error("reference is ambiguous ({} candidates)", candidates.len())]
    Ambiguous { candidates: Vec<(BodyId, u32)> },                         // CHANGED: the candidates
    #[error("body has no sub-entity naming (implicit/form body)")] Unsupported,
}
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SetPolicy { AllOrError, DropMissing }

/// One resolution done by a feature, kept in `NodeEval` (UI, Python, corpus harness).
/// A set-valued `resolve_all` records one entry per member; a member expanded to all its
/// split pieces (§5.1) has several `resolved`/`current` entries; a dropped member keeps its
/// error here and becomes a warning diagnostic on the feature.
#[derive(Clone, Debug, PartialEq)]
pub struct ResolutionRecord {
    pub r: EntityRef,                                   // CHANGED: Topo or Profile refs
    pub result: Result<Vec<Resolved>, ResolveError>,    // > 1 only for expanded set members
    /// Refs to the chosen entities at the feature's input state, `key` kept (Accept match).
    pub current: Vec<EntityRef>,
}

/// Body lookup used by the resolver (input state of a feature, or final evaluated state).
pub trait NamingView {
    fn body_geom(&self, id: BodyId) -> Option<&BodyGeom>;
    fn naming(&self, id: BodyId) -> Option<&BodyNaming>;
    fn successors(&self, id: BodyId) -> &[BodyId];
}
pub fn resolve(view: &dyn NamingView, kernel: &dyn amg_brep::BrepKernel, tol: &Tolerance,
               r: &TopoRef) -> ResolutionRecord;
/// S11 R1: a ref for a picked entity (kernel needed for the fingerprint).
pub fn topo_ref_at(view: &dyn NamingView, kernel: &dyn amg_brep::BrepKernel,
                   body: BodyId, kind: TopoKind, index: u32) -> Option<TopoRef>;

// ===== changes elsewhere in amg-doc =========================================================
pub enum BodyChangeKind {
    Created { geom: BodyGeom, naming: Option<BodyNaming>,
              derived_from: Option<BodyId> },                 // + derived_from (split pieces)
    Modified { geom: BodyGeom, naming: Option<BodyNaming> },
    Consumed { into: BodyId },                                 // NEW (combine/join of a tool body)
    Deleted,
}
impl Evaluated { pub fn successors(&self, id: BodyId) -> &[BodyId]; }   // NEW (+ NamingView impl)
pub struct NodeEval { pub health: Health, pub key: Option<CacheKey>, pub duration: Duration,
                      pub resolutions: Arc<[ResolutionRecord]> }       // + resolutions
impl<'a> RecomputeCtx<'a> {
    /// Records a ResolutionRecord; a warning becomes a Diagnostic on this node automatically.
    /// Recording behind `&self` needs interior mutability: the ctx holds a
    /// `Mutex<Vec<ResolutionRecord>>` (it must stay `Sync`; features may use rayon). Features
    /// call `resolve*` from the thread running `compute`, never inside parallel closures, so
    /// the record order is the call order (deterministic, ADR-014; debug builds assert the
    /// thread). The engine moves the records into `NodeEval.resolutions` after `compute`.
    pub fn resolve(&self, r: &TopoRef) -> Result<Resolved, ResolveError>;          // unchanged signature
    /// Members in input order; an expanded member contributes all its pieces, in order.
    pub fn resolve_all(&self, refs: &[TopoRef], policy: SetPolicy)
        -> Result<Vec<Resolved>, ResolveError>;                                    // NEW
    /// By key, else by `ProfileSet::match_fingerprint` (FingerprintOnly warning, recorded).
    pub fn resolve_profile(&self, sketch: SketchId, key: ProfileKey,
                           fp: Option<&ProfileFp>) -> Result<(ProfileKey, MatchQuality), ResolveError>; // NEW
    /// `Profile { sketch, outer }` name source of a resolved region, for cap names (§5.4).
    pub fn region_source(&self, sketch: SketchId, key: ProfileKey) -> Result<NameSource, ResolveError>; // NEW
    pub fn naming_builder(&self) -> NamingBuilder;                                 // NEW = new(feature_id)
    pub fn kernel(&self) -> &dyn amg_brep::BrepKernel;                             // NEW (if not already)
}
```

**Compatibility with the DOC-01 code** (checked against `crates/amg-doc`): the only users of
the provisional types are `eval.rs` (`ErrorCause::Resolve(ResolveError)`, `EvalBody`'s
`naming: Option<Arc<BodyNaming>>`) and the `lib.rs` re-exports. `ErrorCause` and
`FeatureError` derive only `PartialEq`, which the new `ResolveError` still has. `Resolved`
loses `Eq` (its `f32` warning payloads); nothing in the workspace compares it with `Eq`.
`NameSeg::Generated`/`Modified`/`Merged` are renamed or removed, which only the `naming.rs`
unit test uses. The serialized form of provisional names changes, but no file with names
exists yet (`FORMAT_VERSION` stays; DOC-06 lands before any feature writes names).

**DOC-06 test features** (lane E, in `amg-doc` behind `cfg(any(test, feature =
"test-features"))`, built directly on `BrepKernel`; schemas used by the corpus, §11.3):

| Kind | Data | Naming |
|---|---|---|
| `test.brep_box` | `{ "size": [x, y, z] }`, at the origin, world frame | faces `XMin#0 … ZMax#0`, `from: None` |
| `test.brep_prism` | `{ "points": [[id, x, y], …] (CCW), "z0": f64, "height": f64, "op": "New"\|"Cut"\|"Join", "target": BodyId\|null }` | edge `i` (point i → i+1) is tagged with point `i`'s id; laterals `Gen{self, Lateral#0, SketchCurve{sketch: SketchId(self), curve: id}}`; caps `StartCap#0`/`EndCap#0`, `from: None`; `Cut`/`Join` modify `target` and are followed by `unify_same_domain` (§5.1) |
| `test.brep_fillet` | `{ "body": BodyId, "edges": [TopoRef], "radius": f64, "policy": "AllOrError"\|"DropMissing" }` (policy default `AllOrError`) | S04 fillet rule: `Selection` keys for picked edges, corners by §4.2 rule 4 |
| `test.probe` | `{ "body": BodyId, "refs": [TopoRef], "policy": "AllOrError"\|"DropMissing" }` | Resolves and records its refs, outputs no body change. Lets the corpus reference entities no test op can consume (fillet corner patches, TNP-27) |


---

## 11. Golden TNP corpus (Done-when (f))

### 11.1 Layout
```
tests/golden/tnp/
  README.md                      # points here
  bases/B1.json  B2.json …       # shared base command scripts
  cases/TNP-01.json …            # one case per file
  extended/TNP-X1.json …         # cases for features after Phase 2 (not CI-gated yet)
```
DOC-06 runs cases with `"phase": "DOC-06"` in `amg-doc/tests/tnp.rs`. DOC-13 runs all core
cases in `amg-features/tests/tnp.rs`. Both are CI-gated, and DOC-13 must finish in < 60 s.

### 11.2 Case format
```jsonc
{
  "id": "TNP-07",
  "title": "split already seen; piece areas swap, so the split index drifts",
  "phase": "DOC-06",                 // DOC-06 = test features only; DOC-13 = S04 features
  "naive_index_breaks": true,        // prediction: true | false | null (unknown); the harness measures it
  "base": "B1",                      // optional, a file in bases/
  "script": [ /* more base commands */ ],   // or a name ("S6") of a shared script in bases/
  "edit":   [ /* edit commands, applied after the base is recomputed */ ],
  "expect": {                        // or an array of such objects, all checked
    "subject": { "feature": "$fil", "ref": 0 },   // n-th ref (record) that feature resolves
    "health":  "Ok" | "Warning" | "Error",        // optional: the subject feature's health
    "outcome": "Ok" | "Warning" | "Error",
    "quality": "Exact" | "Disambiguated" | "Widened" | "FingerprintOnly",   // Ok/Warning
    "warning": "NewSplit" | "Widened" | "IndexDrift" | … | null,             // Warning
    "error":   "NotFound" | "Ambiguous" | "BodyMissing",                     // Error
    "entity":  { "kind": "Edge", "class": "Line", "centroid": [x,y,z], "measure": m },
    "body":    { "origin": "$e1", "slot": 0 },       // optional
    "intended": null                                 // Error cases: the entity the user meant, or null
  }
}
```
**Commands** are serialized S01 `Command`s (externally tagged serde), wrapped as
`{ "bind": "name", "cmd": { … } }`. The harness expands these placeholders and macros:
- `"$root"` is the root `ComponentId`. `"$name"` is the id created by the command bound to
  `name` (`CommandOutcome.created[0]`). `"$index_of:name"` is that feature's timeline index
  (for `AddFeature.at`, inserting before a feature).
- `{"$pick": {"kind", "near": [x,y,z], "class"}}` is replaced by
  `topo_ref_at(<state before the feature being added>, nearest entity of that kind and class
  to `near`)`. Picks are made when the command is applied, so a pick in an *edit* sees the
  edited upstream state. If the second-nearest entity is within 1e-6 mm of the nearest, the
  pick is a **harness error** (the case is broken, e.g. a pick on a shared edge of two faces
  that were expected to be merged), never a silent choice.
- **Selection keys.** The harness gives the `TopoRef`s of each feature field keys 1, 2, … in
  list order when a feature is added; a `patch` that adds members continues from the
  feature's largest key.
- `{"accept": "$name"}` expands to the Accept-match `EditFeature` for every ref of that
  feature (§7): each ref's name and fingerprint are replaced with `ResolutionRecord.current`,
  keys kept.
- `{"saveload": true}` saves the document to a temporary `.amg` (with caches, once DOC-14
  exists), closes it, reopens it and recomputes; later commands run on the reopened document.
- `{"patch": {"id": "$name", "merge": {…}}}` expands to `EditFeature` with the feature's
  current JSON merged with RFC 7396 JSON Merge Patch. Patches never touch `TopoRef`s.
- DOC-13 only:
  - `{"sketch": {"plane": "XY"|"XZ"|{"$pick": …}, "entities": [["L", id, [x0,y0], [x1,y1]],
    ["C", id, [cx,cy], r], ["P", id, [x,y]]]}}` adds a `sketch.sketch` feature with fully
    fixed geometry (no constraints). Coordinates are world coordinates for XY/XZ (x, z for
    XZ). For a `$pick` plane they are given as world x, y of points on the face, and the
    harness converts them once, at expansion, with the plane the sketch resolves to at that
    moment (`SketchGeom.plane.to_local`). The corpus only sketches on faces with normal +Z;
    under S03's planar-face frame rule (request §12: origin = projection of the world origin,
    x = projection of world X) local and world x, y then coincide, and the face can move
    along its normal (TNP-15) without moving the sketch in-plane.
  - `{"sketch_edit": {"id", "remove": [ids], "add": [entities], "move": [[id, …new
    coords]]}}` is the matching `EditFeature`.
  - `{"$profile": {"sketch": "$s", "at": [x,y]}}` gives `EntityRef::Profile` via
    `ProfileSet::at_point` plus the fingerprint.
  - Feature shorthands. Each expands to exactly one `AddFeature` with `at: null`,
    `context: "$root"`, `name: null`:

    | Macro | Arguments | Expands to (`feature`) |
    |---|---|---|
    | `"EXT"` | `[sketch, at, extent, op, participants?]` | `{"kind":"solid.extrude","v":1,"data":{"profiles":[{"$profile":{"sketch":sketch,"at":at}}],"start":"Profile","extent":E,"taper":"0 deg","thin":null,"op":op,"participants":P}}` with `E = "All"` if `extent == "All"`, else `{"Distance": extent}`, and `P` = each bound feature's `{"origin": id, "slot": 0}` (default `[]`) |
    | `"EXTN"` | `[sketch, [at…], extent, op, participants?]` | as `EXT`, one `$profile` per point |
    | `"FIL"` | `[pick, radius]` | `{"kind":"solid.fillet","v":1,"data":{"edges":[pick],"radius":radius}}` |
    | `"CHA"` | `[pick, distance]` | `{"kind":"solid.chamfer","v":1,"data":{"edges":[pick],"distance":distance}}` |

  - S04 feature `data` uses provisional field names that follow S04's `FeatureSpec` pattern:
    - `solid.revolve {profiles, axis, angle, op}`
    - `solid.pattern_circular {features, axis, count, angle}`
    - `solid.mirror {bodies, plane}`
    - `solid.combine {target, tools, op, keep_tools}`
    - `solid.split_body {body, tool}`

    DOC-13 renames them to the final SOL-* schemas without changing a case's intent, and
    updates the macro table in the corpus README.

**Checking.**
- The entity check uses `ResolutionRecord.current.fingerprint` against `expect.entity`:
  class equal, centroid within 1e-6 mm, measure within 1e-6 relative (fillet/revolve cases
  1e-4).
- `outcome`, `quality`, `warning` and `error` must match exactly.
- **Naive baseline (measured, every case).** At pick time the harness records the picked
  in-shape index `i` (`TopExp::MapShapes` order, via `amg-brep`) and body. After the edit it
  reads index `i` of the same body at the subject feature's input state. The naive scheme
  **breaks** if `i` is out of range, or if the entity there doesn't match `expect.entity`
  (same class/centroid/measure check as above). The measured flag is written to
  `target/tnp-report.json` next to the prediction. A wrong prediction is reported, not
  failed; the corpus file is corrected in the same PR. Only cases with an `expect.entity`
  count (on expected-error cases the naive scheme always "succeeds" silently, which says
  nothing). DOC-13 requires **≥ 5 cases measured as breaking**.
- **Silent wrong** (the DOC-06/DOC-13 "0 silent wrong" oracle): a subject that resolves `Ok`
  with no warning while its entity fails the `expect.entity` check, or while the case expects
  `Error` (an `intended` entity, if given, is reported in the failure message).
- **Held-out set.** The thresholds of §6.3 are tuned on the DOC-06 cases. The DOC-13-only
  cases and the fuzz set (§11.6) are not used for tuning; a threshold change must keep both
  green.
- Volume checks are not part of the oracle (geometry is S02/S04's golden tests).

### 11.3 Bases
```jsonc
// B1 — box 100 × 60 × 40 (x 0..100, y 0..60, z 0..40). Top face z = 40, front face y = 0.
[{"bind":"box","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_box","v":1,"data":{"size":[100,60,40]}},"context":"$root","name":null}}}]

// B2 — prism of polygon 1(0,0) 2(100,0) 3(100,60) 4(0,60), z 0..40.
//      Lateral tags: 1 = front (y=0), 2 = right (x=100), 3 = back (y=60), 4 = left (x=0).
[{"bind":"prism","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_prism","v":1,"data":{"points":[[1,0,0],[2,100,0],[3,100,60],[4,0,60]],"z0":0,"height":40,"op":"New","target":null}},"context":"$root","name":null}}}]
```

### 11.4 Core cases (30; ≥ 20 required; ≥ 5 must be measured to break naive index naming)
The fillet command template used below (radius `R`, pick `P`):
`{"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[P],"radius":R}},"context":"$root","name":null}}}`

| ID | Phase | Naive breaks | Category | Expected |
|---|---|---|---|---|
| TNP-01 | DOC-06 | no | dimension edit | Ok Exact |
| TNP-02 | DOC-06 | **yes** | profile gains a vertex | Ok Exact |
| TNP-03 | DOC-06 | **yes** | profile loses a vertex | Ok Exact |
| TNP-04 | DOC-06 | — | inserted split, symmetric | Error Ambiguous |
| TNP-05 | DOC-06 | — | inserted split, asymmetric | Warning NewSplit |
| TNP-06 | DOC-06 | no | seen split, dimension edit | Ok Exact |
| TNP-07 | DOC-06 | **yes** | seen split, pieces swap order | Warning IndexDrift |
| TNP-08 | DOC-06 | — | seen split suppressed | Warning Widened |
| TNP-09 | DOC-06 | **yes** | reorder | Ok Exact |
| TNP-10 | DOC-06 | no | fillet of fillet edge, radius edit | Ok Exact |
| TNP-11 | DOC-06 | — | inserted fillet consumes edge | Error NotFound |
| TNP-12 | DOC-06 | **yes** | tool profile gains a vertex | Ok Exact |
| TNP-13 | DOC-13 | — | sketch curve redrawn | Warning FingerprintOnly |
| TNP-14 | DOC-13 | **yes** | new profile added | Ok Exact |
| TNP-15 | DOC-13 | no | sketch on face, height edit | Ok Exact |
| TNP-16 | DOC-13 | **yes** | sketch circle deleted | Ok Exact |
| TNP-17 | DOC-13 | — | pattern count 6 → 8 | Ok Exact |
| TNP-18 | DOC-13 | — | pattern count 6 → 3 | Error NotFound |
| TNP-19 | DOC-13 | no | mirror copy, height edit | Ok Exact |
| TNP-20 | DOC-13 | — | seen merge disappears | Error Ambiguous |
| TNP-21 | DOC-13 | — | split body, size edit | Ok Exact |
| TNP-22 | DOC-13 | — | revolve to 360° removes caps | Error NotFound |
| TNP-23 | DOC-13 | — | combine inserted (body lineage) | Ok Exact |
| TNP-24 | DOC-06 | no | Accept match on a fillet input; fillet-of-fillet downstream | Ok Exact (no cascade) |
| TNP-25 | DOC-06 | no | set-valued fillet, one member consumed upstream | member Error NotFound (dropped), other Ok Exact, feature Warning |
| TNP-26 | DOC-06 | ? | seen split, pieces become congruent | Ok Exact |
| TNP-27 | DOC-06 | ? | fillet corner patch referenced, height edit | Ok Exact |
| TNP-28 | DOC-06 | **yes** | save/load, then TNP-07's edit | Warning IndexDrift |
| TNP-29 | DOC-13 | ? | hole added to the plate region, profile re-picked | Ok Exact |
| TNP-30 | DOC-13 | ? | profile list order reversed | Ok Exact |

"Naive breaks" is the prediction (`?` = `null`); the harness measures every case (§11.2).
DOC-06 Done-when 2 uses TNP-01, 02, 03, 05, 06, 07, 08, 09, 10, 12 as its 10 "resolve to
the intended entity" cases; TNP-24..28 are DOC-06 cases too (TNP-28 covers Done-when 4,
save/load). TNP-04, TNP-11 and TNP-25's dropped member count towards Done-when 3 (no silent
wrong). The expected-error cases record `"intended": null`: in TNP-04 and TNP-20 both
candidates are equally "meant", and in TNP-11/18/22 the entity no longer exists.


**TNP-01: fillet edge survives a box height edit.**
```json
{"id":"TNP-01","phase":"DOC-06","naive_index_breaks":false,"base":"B1",
 "script":[{"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[50,0,40],"class":"Line"}}],"radius":5}},"context":"$root","name":null}}}],
 "edit":[{"cmd":{"EditFeature":{"id":"$box","feature":{"kind":"test.brep_box","v":1,"data":{"size":[100,60,80]}}}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Line","centroid":[50,0,80],"measure":100}}}
```

**TNP-02: inserting a polygon vertex shifts every index; the front-right edge keeps its name.**
Edge between laterals 1 and 2 (x=100, y=0). The edit puts a new point 5 at (−20, 30) at the
*start* of the point list (between 4 and 1), so the wire, and with it every lateral face and
edge, starts with the new edge 5→1 and the indices of the existing ones shift. The face count
goes 6 → 7. (The first version inserted the point after the picked edge, which probably
didn't shift its index; review MUST-FIX 5.)
```json
{"id":"TNP-02","phase":"DOC-06","naive_index_breaks":true,"base":"B2",
 "script":[{"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$prism","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[100,0,20],"class":"Line"}}],"radius":5}},"context":"$root","name":null}}}],
 "edit":[{"cmd":{"EditFeature":{"id":"$prism","feature":{"kind":"test.brep_prism","v":1,"data":{"points":[[5,-20,30],[1,0,0],[2,100,0],[3,100,60],[4,0,60]],"z0":0,"height":40,"op":"New","target":null}}}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Line","centroid":[100,0,20],"measure":40}}}
```

**TNP-03: deleting a polygon vertex (2) keeps the back-left edge.**
After the edit the polygon is the triangle 1 (0,0), 3 (100,60), 4 (0,60).
```json
{"id":"TNP-03","phase":"DOC-06","naive_index_breaks":true,"base":"B2",
 "script":[{"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$prism","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[0,60,20],"class":"Line"}}],"radius":5}},"context":"$root","name":null}}}],
 "edit":[{"cmd":{"EditFeature":{"id":"$prism","feature":{"kind":"test.brep_prism","v":1,"data":{"points":[[1,0,0],[3,100,60],[4,0,60]],"z0":0,"height":40,"op":"New","target":null}}}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Line","centroid":[0,60,20],"measure":40}}}
```

**TNP-04: a centred notch inserted upstream splits the filleted edge symmetrically.**
The notch removes x 45..55, y 0..10, z 30..40, leaving two 45 mm pieces equidistant from the
picked centroid. Ratio 1.0 → Ambiguous.
```json
{"id":"TNP-04","phase":"DOC-06","naive_index_breaks":false,"base":"B1",
 "script":[{"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[50,0,40],"class":"Line"}}],"radius":3}},"context":"$root","name":null}}}],
 "edit":[{"bind":"notch","cmd":{"AddFeature":{"at":"$index_of:fil","feature":{"kind":"test.brep_prism","v":1,"data":{"points":[[41,45,-10],[42,55,-10],[43,55,10],[44,45,10]],"z0":30,"height":20,"op":"Cut","target":{"origin":"$box","slot":0}}},"context":"$root","name":null}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Error","error":"Ambiguous","intended":null}}
```

**TNP-05: an off-centre notch (x 20..30) leaves pieces of 20 mm and 70 mm.**
d(long) = 0.26, d(short) = 0.69, ratio 0.38 → picks the long piece, with a `NewSplit`
warning (the reference predates the split).
```json
{"id":"TNP-05","phase":"DOC-06","naive_index_breaks":false,"base":"B1",
 "script":[{"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[50,0,40],"class":"Line"}}],"radius":3}},"context":"$root","name":null}}}],
 "edit":[{"bind":"notch","cmd":{"AddFeature":{"at":"$index_of:fil","feature":{"kind":"test.brep_prism","v":1,"data":{"points":[[41,20,-10],[42,30,-10],[43,30,10],[44,20,10]],"z0":30,"height":20,"op":"Cut","target":{"origin":"$box","slot":0}}},"context":"$root","name":null}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Warning","quality":"Disambiguated","warning":"NewSplit","entity":{"kind":"Edge","class":"Line","centroid":[65,0,40],"measure":70}}}
```

**Shared script S6** (TNP-06/07/08): B1 + a groove x 60..70 across the full depth (z 30..40),
which splits the top face into A (x 0..60, Split index 0) and B (x 70..100, index 1). Then a
fillet on the B/front edge (centroid (85,0,40), length 30).
```json
[{"bind":"slot","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_prism","v":1,"data":{"points":[[51,60,-10],[52,70,-10],[53,70,70],[54,60,70]],"z0":30,"height":20,"op":"Cut","target":{"origin":"$box","slot":0}}},"context":"$root","name":null}}},
 {"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[85,0,40],"class":"Line"}}],"radius":3}},"context":"$root","name":null}}}]
```

**TNP-06: the groove moves to x 55..65; piece order is kept.**
```json
{"id":"TNP-06","phase":"DOC-06","naive_index_breaks":false,"base":"B1","script":"S6",
 "edit":[{"patch":{"id":"$slot","merge":{"points":[[51,55,-10],[52,65,-10],[53,65,70],[54,55,70]]}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Line","centroid":[82.5,0,40],"measure":35}}}
```

**TNP-07: the groove moves to x 35..45; B is now the larger piece, so the split indices swap.**
Name-equal (index 1) is now piece A. The fingerprint picks B: d = 0.53 vs 2.25, ratio 0.23.
They disagree → `IndexDrift` warning. A naive index ref, or one trusting `Split.index`,
would hit A.
```json
{"id":"TNP-07","phase":"DOC-06","naive_index_breaks":true,"base":"B1","script":"S6",
 "edit":[{"patch":{"id":"$slot","merge":{"points":[[51,35,-10],[52,45,-10],[53,45,70],[54,35,70]]}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Warning","quality":"Disambiguated","warning":"IndexDrift","entity":{"kind":"Edge","class":"Line","centroid":[72.5,0,40],"measure":55}}}
```

**TNP-08: suppressing the groove leaves the whole top-front edge.**
```json
{"id":"TNP-08","phase":"DOC-06","naive_index_breaks":false,"base":"B1","script":"S6",
 "edit":[{"cmd":{"SetSuppressed":{"ids":["$slot"],"suppressed":true}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Warning","quality":"Widened","warning":"Widened","entity":{"kind":"Edge","class":"Line","centroid":[50,0,40],"measure":100}}}
```

**TNP-09: reordering two independent cuts.**
CutA is a groove x 20..30. CutB is a pocket x 70..90, y 20..40. The fillet is on the pocket
rim at y = 20. The edit moves CutB before CutA.
```json
{"id":"TNP-09","phase":"DOC-06","naive_index_breaks":true,"base":"B1",
 "script":[{"bind":"cutA","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_prism","v":1,"data":{"points":[[31,20,-10],[32,30,-10],[33,30,70],[34,20,70]],"z0":30,"height":20,"op":"Cut","target":{"origin":"$box","slot":0}}},"context":"$root","name":null}}},
           {"bind":"cutB","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_prism","v":1,"data":{"points":[[21,70,20],[22,90,20],[23,90,40],[24,70,40]],"z0":30,"height":20,"op":"Cut","target":{"origin":"$box","slot":0}}},"context":"$root","name":null}}},
           {"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[80,20,40],"class":"Line"}}],"radius":2}},"context":"$root","name":null}}}],
 "edit":[{"cmd":{"MoveFeature":{"id":"$cutB","to":1}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Line","centroid":[80,20,40],"measure":20}}}
```

**TNP-10: a second fillet on the arc where blend 1 meets the right face; blend 1's radius goes 5 → 8.**
The arc is a quarter circle about (100, r, 40−r). Its centroid is
`(100, r − 0.6366r, 40 − r + 0.6366r)`, i.e. `0.9003·r` along the bisector. Tolerance 1e-4.
```json
{"id":"TNP-10","phase":"DOC-06","naive_index_breaks":false,"base":"B1",
 "script":[{"bind":"fil1","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[50,0,40],"class":"Line"}}],"radius":5}},"context":"$root","name":null}}},
           {"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[100,1.817,38.183],"class":"Circle"}}],"radius":1}},"context":"$root","name":null}}}],
 "edit":[{"patch":{"id":"$fil1","merge":{"radius":8}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Circle","centroid":[100,2.90704,37.09296],"measure":12.56637}}}
```

**TNP-11: a fillet inserted upstream on the same edge removes it.**
Top and Front no longer share an edge → `NotFound`, with no geometric guess (§6.4 stage 3,
second rule).
```json
{"id":"TNP-11","phase":"DOC-06","naive_index_breaks":false,"base":"B1",
 "script":[{"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[50,0,40],"class":"Line"}}],"radius":5}},"context":"$root","name":null}}}],
 "edit":[{"bind":"fil0","cmd":{"AddFeature":{"at":"$index_of:fil","feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[50,0,40],"class":"Line"}}],"radius":3}},"context":"$root","name":null}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Error","error":"NotFound","intended":null}}
```

**TNP-12: the pocket tool's polygon gains a vertex; the rim edge on tool lateral 12 keeps its name.**
```json
{"id":"TNP-12","phase":"DOC-06","naive_index_breaks":true,"base":"B2",
 "script":[{"bind":"pocket","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_prism","v":1,"data":{"points":[[11,30,20],[12,60,20],[13,60,40],[14,30,40]],"z0":30,"height":20,"op":"Cut","target":{"origin":"$prism","slot":0}}},"context":"$root","name":null}}},
           {"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$prism","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[60,30,40],"class":"Line"}}],"radius":2}},"context":"$root","name":null}}}],
 "edit":[{"patch":{"id":"$pocket","merge":{"points":[[11,30,20],[15,45,15],[12,60,20],[13,60,40],[14,30,40]]}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Line","centroid":[60,30,40],"measure":20}}}
```

The DOC-13 cases below use the `sketch`, `sketch_edit`, `patch`, `EXT`/`EXTN`/`FIL`/`CHA`
macros of §11.2. Each macro expands deterministically to one serialized `Command`.

**Shared sketch R1:** rectangle lines 1 (0,0)-(100,0), 2 (100,0)-(100,60), 3 (100,60)-(0,60),
4 (0,60)-(0,0) on XY. Written inline in each case that uses it.

**TNP-13: line 1 is deleted and redrawn as line 9 in the same place.**
Both adjacent faces lose their lineage: the lateral was generated from line 1, and the top
cap's outer source set changes from {1,2,3,4} to {2,3,4,9}. So the edge has no candidates,
not every face has candidates (no adjacency rule), and the stage-3 fingerprint scan finds the
identical edge (d ≈ 0; the next Line edge, the bottom-front one, is at d = 0.4). The extrude
itself also warns: its profile ref no longer matches by `ProfileId` and resolves by
fingerprint (§5.4). Only the fillet's ref is the subject.
```json
{"id":"TNP-13","phase":"DOC-13","naive_index_breaks":false,
 "script":[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[0,0],[100,0]],["L",2,[100,0],[100,60]],["L",3,[100,60],[0,60]],["L",4,[0,60],[0,0]]]}},
           {"bind":"e1","EXT":["$s1",[50,30],"40 mm","NewBody"]},
           {"bind":"fil","FIL":[{"$pick":{"kind":"Edge","near":[50,0,40],"class":"Line"}},"5 mm"]}],
 "edit":[{"sketch_edit":{"id":"$s1","remove":[1],"add":[["L",9,[0,0],[100,0]]]}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Warning","quality":"FingerprintOnly","warning":"FingerprintOnly","entity":{"kind":"Edge","class":"Line","centroid":[50,0,40],"measure":100}}}
```

**TNP-14: a third boss profile is added; the fillet stays on boss C2's top circle.**
The plate is R1 extruded 10 mm. Bosses C1 (20,20) r5 and C2 (60,20) r5 are joined 5 mm on a
sketch on the plate top. The edit adds C3 (40,45) r5 to the sketch and to the extrude. C3 is
disjoint from C1 and C2, so their regions keep both `ProfileId` and outer source set; the
re-picked profile list changes nothing in the cap names (`Profile{s2, outer {2}}`).
```json
{"id":"TNP-14","phase":"DOC-13","naive_index_breaks":true,
 "script":[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[0,0],[100,0]],["L",2,[100,0],[100,60]],["L",3,[100,60],[0,60]],["L",4,[0,60],[0,0]]]}},
           {"bind":"plate","EXT":["$s1",[50,30],"10 mm","NewBody"]},
           {"bind":"s2","sketch":{"plane":{"$pick":{"kind":"Face","near":[50,30,10],"class":"Plane"}},"entities":[["C",1,[20,20],5],["C",2,[60,20],5]]}},
           {"bind":"boss","EXTN":["$s2",[[20,20],[60,20]],"5 mm","Join",["$plate"]]},
           {"bind":"fil","FIL":[{"$pick":{"kind":"Edge","near":[65,20,15],"class":"Circle"}},"1 mm"]}],
 "edit":[{"sketch_edit":{"id":"$s2","add":[["C",3,[40,45],5]]}},
         {"patch":{"id":"$boss","merge":{"profiles":[{"$profile":{"sketch":"$s2","at":[20,20]}},{"$profile":{"sketch":"$s2","at":[60,20]}},{"$profile":{"sketch":"$s2","at":[40,45]}}]}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Circle","centroid":[60,20,15],"measure":31.41593}}}
```
(A `$profile` inside a patch is expanded like a pick. `EXTN` takes several profile points.)

**TNP-15: a sketch on the top face follows a height edit 40 → 25.**
```json
{"id":"TNP-15","phase":"DOC-13","naive_index_breaks":false,
 "script":[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[0,0],[100,0]],["L",2,[100,0],[100,60]],["L",3,[100,60],[0,60]],["L",4,[0,60],[0,0]]]}},
           {"bind":"e1","EXT":["$s1",[50,30],"40 mm","NewBody"]},
           {"bind":"s2","sketch":{"plane":{"$pick":{"kind":"Face","near":[50,30,40],"class":"Plane"}},"entities":[["C",1,[50,30],10]]}}],
 "edit":[{"patch":{"id":"$e1","merge":{"extent":{"Distance":"25 mm"}}}}],
 "expect":{"subject":{"feature":"$s2","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Face","class":"Plane","centroid":[50,30,25],"measure":6000}}}
```

**TNP-16: three through-holes A/B/C are cut from one sketch; circle A is deleted.**
The chamfer on hole C's top edge stays.
```json
{"id":"TNP-16","phase":"DOC-13","naive_index_breaks":true,
 "script":[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[0,0],[100,0]],["L",2,[100,0],[100,60]],["L",3,[100,60],[0,60]],["L",4,[0,60],[0,0]]]}},
           {"bind":"plate","EXT":["$s1",[50,30],"10 mm","NewBody"]},
           {"bind":"s2","sketch":{"plane":"XY","entities":[["C",11,[20,30],5],["C",12,[50,30],5],["C",13,[80,30],5]]}},
           {"bind":"holes","EXTN":["$s2",[[20,30],[50,30],[80,30]],"All","Cut",["$plate"]]},
           {"bind":"cha","CHA":[{"$pick":{"kind":"Edge","near":[85,30,10],"class":"Circle"}},"1 mm"]}],
 "edit":[{"sketch_edit":{"id":"$s2","remove":[11]}},
         {"patch":{"id":"$holes","merge":{"profiles":[{"$profile":{"sketch":"$s2","at":[50,30]}},{"$profile":{"sketch":"$s2","at":[80,30]}}]}}}],
 "expect":{"subject":{"feature":"$cha","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Circle","centroid":[80,30,10],"measure":31.41593}}}
```

**Shared script S17** (TNP-17/18): a plate (−50,−50)-(50,50), 10 mm. A through-hole r4 at
(30,0). A circular feature pattern of the hole about the origin Z axis, count 6. A chamfer on
instance 3's top edge, at angle 180°, i.e. (−30,0,10).
```json
[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[-50,-50],[50,-50]],["L",2,[50,-50],[50,50]],["L",3,[50,50],[-50,50]],["L",4,[-50,50],[-50,-50]]]}},
 {"bind":"plate","EXT":["$s1",[0,0],"10 mm","NewBody"]},
 {"bind":"s2","sketch":{"plane":"XY","entities":[["C",1,[30,0],4]]}},
 {"bind":"hole","EXT":["$s2",[30,0],"All","Cut",["$plate"]]},
 {"bind":"pat","cmd":{"AddFeature":{"at":null,"feature":{"kind":"solid.pattern_circular","v":1,"data":{"features":["$hole"],"axis":"OriginZ","count":"6","angle":"360 deg"}},"context":"$root","name":null}}},
 {"bind":"cha","CHA":[{"$pick":{"kind":"Edge","near":[-26,0,10],"class":"Circle"}},"0.5 mm"]}]
```
(The pick point is on the circle; nearest-entity selection then finds that edge.)

**TNP-17: pattern count 6 → 8.**
The reference follows instance 3, now at 135°: (−21.2132, 21.2132).
```json
{"id":"TNP-17","phase":"DOC-13","naive_index_breaks":false,"script":"S17",
 "edit":[{"patch":{"id":"$pat","merge":{"count":"8"}}}],
 "expect":{"subject":{"feature":"$cha","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Circle","centroid":[-21.21320,21.21320,10],"measure":25.13274}}}
```

**TNP-18: pattern count 6 → 3; instance 3 no longer exists.**
```json
{"id":"TNP-18","phase":"DOC-13","naive_index_breaks":false,"script":"S17",
 "edit":[{"patch":{"id":"$pat","merge":{"count":"3"}}}],
 "expect":{"subject":{"feature":"$cha","ref":0},"outcome":"Error","error":"NotFound","intended":null}}
```

**TNP-19: a fillet on a mirrored body copy survives a source height edit.**
```json
{"id":"TNP-19","phase":"DOC-13","naive_index_breaks":false,
 "script":[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[10,0],[50,0]],["L",2,[50,0],[50,60]],["L",3,[50,60],[10,60]],["L",4,[10,60],[10,0]]]}},
           {"bind":"e1","EXT":["$s1",[30,30],"30 mm","NewBody"]},
           {"bind":"mir","cmd":{"AddFeature":{"at":null,"feature":{"kind":"solid.mirror","v":1,"data":{"bodies":[{"origin":"$e1","slot":0}],"plane":"OriginYZ"}},"context":"$root","name":null}}},
           {"bind":"fil","FIL":[{"$pick":{"kind":"Edge","near":[-30,0,30],"class":"Line"}},"2 mm"]}],
 "edit":[{"patch":{"id":"$e1","merge":{"extent":{"Distance":"50 mm"}}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Line","centroid":[-30,0,50],"measure":40}}}
```

**TNP-20: a sketch on the merged top of two joined blocks; block 2's height goes 40 → 30.**
Requires the BodyOp unify contract (§5.1): the join merges the two coplanar tops into one
face, and the pick at (50,30,40) is inside it (without unify it would lie on the shared edge
of two faces, which the harness rejects as a tied pick). The reference is
(block-1 cap) + `Merge{e2, with: [block-2 cap]}`. After the edit the tops are separate. Both
are **unmerged** candidates (§6.2 (a) and (c)). d = 0.413 (block 1: centroid 25 mm away,
s = 116.6, half the area) vs 0.422 (block 2: 26.9 mm away), ratio 0.98 > 0.5 → Ambiguous.
```json
{"id":"TNP-20","phase":"DOC-13","naive_index_breaks":false,
 "script":[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[0,0],[50,0]],["L",2,[50,0],[50,60]],["L",3,[50,60],[0,60]],["L",4,[0,60],[0,0]],["L",5,[50,0],[100,0]],["L",6,[100,0],[100,60]],["L",7,[100,60],[50,60]]]}},
           {"bind":"e1","EXT":["$s1",[25,30],"40 mm","NewBody"]},
           {"bind":"e2","EXT":["$s1",[75,30],"40 mm","Join",["$e1"]]},
           {"bind":"s2","sketch":{"plane":{"$pick":{"kind":"Face","near":[50,30,40],"class":"Plane"}},"entities":[["C",1,[50,30],5]]}}],
 "edit":[{"patch":{"id":"$e2","merge":{"extent":{"Distance":"30 mm"}}}}],
 "expect":{"subject":{"feature":"$s2","ref":0},"outcome":"Error","error":"Ambiguous","intended":null}}
```

**TNP-21: split body by origin YZ; a fillet on the positive piece; the block grows to x = 80.**
```json
{"id":"TNP-21","phase":"DOC-13","naive_index_breaks":false,
 "script":[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[-50,0],[50,0]],["L",2,[50,0],[50,60]],["L",3,[50,60],[-50,60]],["L",4,[-50,60],[-50,0]]]}},
           {"bind":"e1","EXT":["$s1",[0,30],"40 mm","NewBody"]},
           {"bind":"split","cmd":{"AddFeature":{"at":null,"feature":{"kind":"solid.split_body","v":1,"data":{"body":{"origin":"$e1","slot":0},"tool":"OriginYZ"}},"context":"$root","name":null}}},
           {"bind":"fil","FIL":[{"$pick":{"kind":"Edge","near":[25,0,40],"class":"Line"}},"2 mm"]}],
 "edit":[{"sketch_edit":{"id":"$s1","move":[[1,[-50,0],[80,0]],[2,[80,0],[80,60]],[3,[80,60],[-50,60]]]}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","body":{"origin":"$split","slot":1},"entity":{"kind":"Edge","class":"Line","centroid":[40,0,40],"measure":80}}}
```

**TNP-22: revolve 270° → 360° removes the end cap the fillet referenced.**
The profile is the rectangle x 10..30, z 0..20 on XZ. The edge is between the end cap and the
outer lateral, at (0,−30,z). After the edit the only Line edges are the two seams at y = 0.
The nearest is the inner seam (x = 10), 31.6 mm away (d ≈ 1.58 > 0.05) → NotFound.
```json
{"id":"TNP-22","phase":"DOC-13","naive_index_breaks":false,
 "script":[{"bind":"s1","sketch":{"plane":"XZ","entities":[["L",1,[10,0],[30,0]],["L",2,[30,0],[30,20]],["L",3,[30,20],[10,20]],["L",4,[10,20],[10,0]]]}},
           {"bind":"rev","cmd":{"AddFeature":{"at":null,"feature":{"kind":"solid.revolve","v":1,"data":{"profiles":[{"$profile":{"sketch":"$s1","at":[20,10]}}],"axis":"OriginZ","angle":"270 deg","op":"NewBody"}},"context":"$root","name":null}}},
           {"bind":"fil","FIL":[{"$pick":{"kind":"Edge","near":[0,-30,10],"class":"Line"}},"1 mm"]}],
 "edit":[{"patch":{"id":"$rev","merge":{"angle":"360 deg"}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Error","error":"NotFound","intended":null}}

```

**TNP-23: a combine inserted upstream folds body B into A; B's fillet follows via lineage.**
```json
{"id":"TNP-23","phase":"DOC-13","naive_index_breaks":false,
 "script":[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[0,0],[50,0]],["L",2,[50,0],[50,60]],["L",3,[50,60],[0,60]],["L",4,[0,60],[0,0]],["L",5,[60,0],[100,0]],["L",6,[100,0],[100,60]],["L",7,[100,60],[60,60]],["L",8,[60,60],[60,0]]]}},
           {"bind":"eA","EXT":["$s1",[25,30],"40 mm","NewBody"]},
           {"bind":"eB","EXT":["$s1",[80,30],"20 mm","NewBody"]},
           {"bind":"fil","FIL":[{"$pick":{"kind":"Edge","near":[80,0,20],"class":"Line"}},"2 mm"]}],
 "edit":[{"bind":"comb","cmd":{"AddFeature":{"at":"$index_of:fil","feature":{"kind":"solid.combine","v":1,"data":{"target":{"origin":"$eA","slot":0},"tools":[{"origin":"$eB","slot":0}],"op":"Join","keep_tools":false}},"context":"$root","name":null}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","body":{"origin":"$eA","slot":0},"entity":{"kind":"Edge","class":"Line","centroid":[80,0,20],"measure":40}}}
```

**TNP-24: Accept match on a fillet's input doesn't rename its blend (review MUST-FIX 3).**
TNP-05's notch is inserted before fillet 1, so fillet 1's edge ref warns `NewSplit`; then
Accept match is run on fillet 1. Fillet 2 references the arc where blend 1 meets the right
face. Blend 1 is `Gen{fil1, Blend#0, Selection(1)}` before and after, so fillet 2 stays
Exact, and fillet 1's refreshed ref is now Exact too (name-equal long piece, ratio ≈ 0).
```json
{"id":"TNP-24","phase":"DOC-06","naive_index_breaks":false,"base":"B1",
 "script":[{"bind":"fil1","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[50,0,40],"class":"Line"}}],"radius":5}},"context":"$root","name":null}}},
           {"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[100,1.817,38.183],"class":"Circle"}}],"radius":1}},"context":"$root","name":null}}}],
 "edit":[{"bind":"notch","cmd":{"AddFeature":{"at":"$index_of:fil1","feature":{"kind":"test.brep_prism","v":1,"data":{"points":[[41,20,-10],[42,30,-10],[43,30,10],[44,20,10]],"z0":30,"height":20,"op":"Cut","target":{"origin":"$box","slot":0}}},"context":"$root","name":null}}},
         {"accept":"$fil1"}],
 "expect":[{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Circle","centroid":[100,1.81690,38.18310],"measure":7.85398}},
           {"subject":{"feature":"$fil1","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Line","centroid":[65,0,40],"measure":70}}]}
```

**TNP-25: a two-edge `DropMissing` fillet; a fillet inserted upstream consumes one edge.**
The back edge's member is dropped (`NotFound`, recorded and warned), the front edge still
resolves, and the feature is `Warning`.
```json
{"id":"TNP-25","phase":"DOC-06","naive_index_breaks":false,"base":"B1",
 "script":[{"bind":"fil","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[50,0,40],"class":"Line"}},{"$pick":{"kind":"Edge","near":[50,60,40],"class":"Line"}}],"radius":3,"policy":"DropMissing"}},"context":"$root","name":null}}}],
 "edit":[{"bind":"fil0","cmd":{"AddFeature":{"at":"$index_of:fil","feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[50,60,40],"class":"Line"}}],"radius":5}},"context":"$root","name":null}}}],
 "expect":[{"subject":{"feature":"$fil","ref":1},"health":"Warning","outcome":"Error","error":"NotFound","intended":null},
           {"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Line","centroid":[50,0,40],"measure":100}}]}
```

**TNP-26: seen split, and the groove moves to x 45..55, so the pieces become congruent.**
A' (x 0..45) and B' (x 55..100) both have area 2700, equal after quantisation (§4.3), so the
centroid decides the order: A' is index 0, B' index 1, and B' is name-equal. d(B') = 0.344
(7.5 mm away, s = 30, 30 → 45 mm), d(A') = 2.10, ratio 0.16 → quiet, Exact.
```json
{"id":"TNP-26","phase":"DOC-06","naive_index_breaks":null,"base":"B1","script":"S6",
 "edit":[{"patch":{"id":"$slot","merge":{"points":[[51,45,-10],[52,55,-10],[53,55,70],[54,45,70]]}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Line","centroid":[77.5,0,40],"measure":45}}}
```

**TNP-27: a three-edge fillet at the box corner (100, 0, 40); a probe references the corner patch; box height 40 → 50.**
The corner patch has no source in S02's history; it is named from its three adjacent blends
(§4.2 rule 4), which are named by selection keys 1–3. For equal radii the patch is a
spherical octant of radius 5 about (95, 5, h − 5), centroid offset r/2 on each axis, area
πr²/2. Tolerance 1e-4. If OCCT builds the corner as a B-spline patch instead, DOC-06 records
the observed class and centroid here and in §13.
```json
{"id":"TNP-27","phase":"DOC-06","naive_index_breaks":null,"base":"B1",
 "script":[{"bind":"fil3","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.brep_fillet","v":1,"data":{"body":{"origin":"$box","slot":0},"edges":[{"$pick":{"kind":"Edge","near":[50,0,40],"class":"Line"}},{"$pick":{"kind":"Edge","near":[100,30,40],"class":"Line"}},{"$pick":{"kind":"Edge","near":[100,0,20],"class":"Line"}}],"radius":5}},"context":"$root","name":null}}},
           {"bind":"probe","cmd":{"AddFeature":{"at":null,"feature":{"kind":"test.probe","v":1,"data":{"body":{"origin":"$box","slot":0},"refs":[{"$pick":{"kind":"Face","near":[97.5,2.5,37.5],"class":"Sphere"}}],"policy":"AllOrError"}},"context":"$root","name":null}}}],
 "edit":[{"cmd":{"EditFeature":{"id":"$box","feature":{"kind":"test.brep_box","v":1,"data":{"size":[100,60,50]}}}}}],
 "expect":{"subject":{"feature":"$probe","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Face","class":"Sphere","centroid":[97.5,2.5,47.5],"measure":39.26991}}}
```

**TNP-28: TNP-07 with a save/load between base and edit.**
Names, selection keys and fingerprints round-trip; the result equals TNP-07's.
```json
{"id":"TNP-28","phase":"DOC-06","naive_index_breaks":true,"base":"B1","script":"S6",
 "edit":[{"saveload":true},
         {"patch":{"id":"$slot","merge":{"points":[[51,35,-10],[52,45,-10],[53,45,70],[54,35,70]]}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Warning","quality":"Disambiguated","warning":"IndexDrift","entity":{"kind":"Edge","class":"Line","centroid":[72.5,0,40],"measure":55}}}
```

**TNP-29: a second hole is added to the plate region and the plate profile is re-picked; a sketch on the plate top stays (review MUST-FIX 2).**
The region's `ProfileId` changes (it hashes the hole edges too), but its outer source set
{1,2,3,4} doesn't, so the top cap keeps its name. Under the first version's rule (cap named
by the authored `ProfileKey`) this case came out `FingerprintOnly`. The top face after the
edit is the rectangle minus circles at (50,30) and (80,30), r5: area 6000 − 50π, centroid x
= (6000·50 − 25π·50 − 25π·80)/area.
```json
{"id":"TNP-29","phase":"DOC-13","naive_index_breaks":null,
 "script":[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[0,0],[100,0]],["L",2,[100,0],[100,60]],["L",3,[100,60],[0,60]],["L",4,[0,60],[0,0]],["C",11,[50,30],5]]}},
           {"bind":"plate","EXT":["$s1",[20,30],"10 mm","NewBody"]},
           {"bind":"s2","sketch":{"plane":{"$pick":{"kind":"Face","near":[20,15,10],"class":"Plane"}},"entities":[["C",1,[20,15],3]]}}],
 "edit":[{"sketch_edit":{"id":"$s1","add":[["C",12,[80,30],5]]}},
         {"patch":{"id":"$plate","merge":{"profiles":[{"$profile":{"sketch":"$s1","at":[20,30]}}]}}}],
 "expect":{"subject":{"feature":"$s2","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Face","class":"Plane","centroid":[49.59674,30,10],"measure":5842.92037}}}
```

**TNP-30: TNP-14's bosses with the profile list reversed.**
Cap names don't depend on the order in which the feature lists its profiles.
```json
{"id":"TNP-30","phase":"DOC-13","naive_index_breaks":null,
 "script":[{"bind":"s1","sketch":{"plane":"XY","entities":[["L",1,[0,0],[100,0]],["L",2,[100,0],[100,60]],["L",3,[100,60],[0,60]],["L",4,[0,60],[0,0]]]}},
           {"bind":"plate","EXT":["$s1",[50,30],"10 mm","NewBody"]},
           {"bind":"s2","sketch":{"plane":{"$pick":{"kind":"Face","near":[50,30,10],"class":"Plane"}},"entities":[["C",1,[20,20],5],["C",2,[60,20],5]]}},
           {"bind":"boss","EXTN":["$s2",[[20,20],[60,20]],"5 mm","Join",["$plate"]]},
           {"bind":"fil","FIL":[{"$pick":{"kind":"Edge","near":[65,20,15],"class":"Circle"}},"1 mm"]}],
 "edit":[{"patch":{"id":"$boss","merge":{"profiles":[{"$profile":{"sketch":"$s2","at":[60,20]}},{"$profile":{"sketch":"$s2","at":[20,20]}}]}}}],
 "expect":{"subject":{"feature":"$fil","ref":0},"outcome":"Ok","quality":"Exact","entity":{"kind":"Edge","class":"Circle","centroid":[60,20,15],"measure":31.41593}}}
```

### 11.5 Extended cases (written when their features land; not CI-gated in DOC-13)

| ID | Feature WP | Scenario | Expected |
|---|---|---|---|
| TNP-X1 | MESH (S07) | Box → `solid_to_mesh`; a mesh feature references the top face group; box height edit | Ok Exact (group named by the B-rep face name) |
| TNP-X2 | IO (S12) | STEP import, fillet on an edge, save/load → Ok Exact; re-import of a file with one extra hole | Warning FingerprintOnly for the unaffected edge |
| TNP-X3 | SOL-06 | Shell (top removed), fillet on the inner floor edge; box height edit | Ok Exact (`ShellInner` from `Topo(face)`) |
| TNP-X4 | SOL-11 | Loft square → circle; fillet on lateral #2's cap edge; square → pentagon | Error NotFound or Warning (correspondence-dependent; fixed when S20 KH-18 lands) |
| TNP-X5 | SOL-12 | Hole at sketch points P1..P3, chamfer on hole P3; delete P1 | Ok Exact (holes named by point) |
| TNP-X6 | SOL-06 | Draft 4 sides 3°, fillet a vertical edge; draft angle 3° → 5° | Ok Exact |

### 11.6 Held-out fuzz set (DOC-13)
Hand-written cases test what their author thought of, and the thresholds are tuned on the
same cases. DOC-13 adds a seeded fuzz run that nobody tunes on (`amg-features/tests/tnp_fuzz.rs`,
200 runs, fixed seed, part of the < 60 s budget or a nightly job if it doesn't fit):
- Bases B1/B2 with one fillet picked at a random top edge point.
- One random upstream edit: a dimension edit (±50 %), a notch inserted at a random position
  on the picked edge, a groove that splits the top face, a polygon vertex added or removed,
  a feature suppressed or reordered.
- **Oracle**, no hand-made expectation needed: the intended entity is the edge (piece) that
  contains the original pick point mapped through the edit (the point moves with the
  dimension edit; for notches and grooves it stays). If exactly one edge contains it, a quiet
  `Ok` must return that edge; if none or several do, the result must be a warning or an
  error. Any quiet `Ok` on another entity is a silent wrong and fails the run.
- The run reports the counts per outcome, so changes in how often each warning fires are
  visible in review (including the "boundary jumps" case of §6.7).

---

## 12. Requests to other specs (recorded here; owners confirm)

- **S02 (lane A).**
  - Q1: Contract and test that `Shape::transformed` (rigid) keeps the sub-shape index order of
    its input (§4.6).
  - Q2: BREP-15 `pipe` emits `StartCap`/`EndCap`/`Lateral` roles, with laterals `Generated`
    from profile edges. `thru_sections` caps are `LoftSection{0}` and `LoftSection{n−1}`.
    The BREP-15 history test records which sections' edges OCCT covers (pipe shell with
    several sections, thru-sections beyond section 0), and tags each pipe lateral with the
    tag of its spine edge where the spine is a tagged wire (§5.2 Sweep).
  - Q3: `OpResult::per_solid(&self) -> KResult<Vec<OpResult>>` splits a compound result into
    per-solid results with restricted, reindexed histories (split body, multi-lump outputs).
  - Q4: Any output face without a source gets `OpRole::Unexplained` *and* the op returns
    `KernelWarning::PartialHistory` (contract wording, BREP-05 test).
  - Q5: The split order (§6 "deterministic split order") uses the quantised integer key of
    §4.3 `(−q(area), q(centroid)…, index)` instead of a float comparison, so it is a total
    order. The sort lives in one function; DOC-06's `Bound` order uses the same key.
  - Q6: History tests for the cases §6.6 lists as not promised by OCCT: BREP-06 (a stripe
    with several faces for one edge, one face for two collinear edges, a 3-edge vertex blend
    whose `CornerPatch` faces are reported with the role) and BREP-07 (shell inner faces map
    to their source faces, rim faces to the removed faces' edges, arc-join faces reported
    with a role or an edge source), 5 cases each.
  - S01's existing request that `from_history` take several operands is adopted (§10).
- **S03 (lane B).**
  - `ProfileSet::match_fingerprint(&ProfileFingerprint) -> ProfileMatch`: a pure match
    without the previous profile set, so `EntityRef::Profile` resolution is deterministic
    (§5.4).
  - `Profile::outer_sources(&self) -> Vec<EntityId>`: sorted, deduplicated `source` + `also`
    of the outer loop (cap names, §5.4). It can also be computed in `amg-doc` from
    `Profile.outer`, so this is a convenience, not a blocker. No change to SK-15's id scheme
    is needed.
  - **Planar-face sketch frame**: `SketchFeature` on `PlaneInput::Ref(Topo face)` uses origin
    = the projection of the world origin onto the face plane, normal = the outward face
    normal, x = the projection of world X (world Y if X is parallel to the normal),
    y = normal × x. Deterministic, and a face that moves along its normal (a height edit)
    doesn't move the sketch in-plane (TNP-15, TNP-29).
- **S04 (lane A, Draft).**
  - Adopt §5 per feature (names via `ctx.naming_builder()`).
  - Use `SetPolicy::DropMissing` for set-valued selections and `AllOrError` for
    single-valued ones.
  - A rule change bumps the feature `v` and adds `naming_rule`.
  - Split body uses `PieceSlot` and the slot rule.
  - Combine emits `Consumed`.
  - **Every BodyOp result is simplified with `unify_same_domain` (faces + edges, fixed
    default tolerances), composed into the op's history** (§5.1; TNP-20 depends on it).
  - Picked refs carry selection keys; generating features call `NamingBuilder::selection`
    for them (§5.1). Copy (SOL-09) names its result `instanced(self, slot)`.
- **S07 (lane C).**
  - Replace `MeshEntityRef` with `TopoRef { kind: FaceGroup }`.
  - Mesh features return `BodyNaming::for_mesh`.
  - `solid_to_mesh` passes the B-rep face names.
- **S11 (lane D).**
  - `topo_ref_at` needs the kernel.
  - The Accept-match and Re-pick flows of §7 read `NodeEval.resolutions`.
  - Picks happen against the feature's input state.
  - Selection widgets allocate `TopoRef.key` (max + 1 per feature, never reused); Accept match
    and Re-pick keep it; remove + add allocates a new one (§5.1, §7).

## 13. Refinement log
- **2026-10-20 (DOC-11).** Written. Thresholds and weights in §6.3 are provisional. DOC-06
  re-tunes them on the corpus and records any change here (bumping `NAMING_VERSION`).
  Expected distances in TNP-05/07/20 were computed by hand with the §6.3 metric and are
  re-checked by the harness.
- **2026-10-20 (DOC-11, review response).** Revised after a critical review; the changes are
  listed in §16. Hand-checked again with the revised rules: TNP-01..23 keep their expected
  outcomes (TNP-02's edit changed, TNP-20 now follows from §6.2 rule (c), TNP-22's nearest
  seam corrected to d ≈ 1.58). New cases TNP-24..30 were computed by hand the same way.
- **2026-10-21 (owner).** Note Accepted; §15 answered with every recommendation (QUESTIONS #43–#49).


## 14. Sources
- D. A. Kripac, "A mechanism for persistently naming topological entities in history-based
  parametric solid models", *Computer-Aided Design* 29(2), 113–122, 1997.
- V. Capoyleas, X. Chen, C. M. Hoffmann, "Generic naming in generative, constraint-based
  design", *Computer-Aided Design* 28(1), 17–26, 1996; Purdue TR CSD-TR-94-011
  (https://docs.lib.purdue.edu/cstech/1114).
- D. Marcheix, G. Pierra, "A survey of the persistent naming problem", *Proc. 7th ACM Symp.
  on Solid Modeling and Applications*, 13–22, 2002; D. Marcheix, "A persistent naming of
  shells", CAD/CAM 2006 (https://www.lias-lab.fr/publications/7550/2006-CAD-CAM-Marcheix.pdf).
- R. Bidarra, P. J. Nyirenda, W. F. Bronsvoort, "A feature-based solution to the persistent
  naming problem", *Computer-Aided Design and Applications* 2(1–4), 517–526, 2005
  (https://publications.graphics.tudelft.nl/papers/626).
- S. H. Farjana, S. Han, "Mechanisms of persistent identification of topological entities in
  CAD systems: a review", *Alexandria Engineering Journal* 57(4), 2837–2849, 2018
  (https://doaj.org/article/bf42fdae16584d0ba2d824137aba6599).
- realthunder, "Topological Naming Algorithm", FreeCAD_assembly3 wiki
  (https://github.com/realthunder/FreeCAD_assembly3/wiki/Topological-Naming-Algorithm);
  FreeCAD 1.0 release, 2024-11-18 (https://wiki.freecad.org/Release_notes_1.0;
  https://hackster.io/news/freecad-hits-1-0-after-two-decades-finally-sees-an-end-to-the-toponaming-problem-7e7ac2df3e63).
- Onshape FeatureScript standard library docs: `Id`, `qCreatedBy`, `unstableIdComponent`,
  `opHole` `identities` (https://cad.onshape.com/FsDoc/library.html); Onshape forum answers on
  transient ids and `qCreatedBy` (https://forum.onshape.com/discussion/comment/23066,
  https://forum.onshape.com/discussion/comment/59713). No Onshape naming patent was found.
- build123d, "Topology Selection and Exploration" and "Selector Tutorial"
  (https://build123d.readthedocs.io/en/latest/topology_selection.html,
  https://build123d.readthedocs.io/en/latest/tutorial_selectors.html); CadQuery tags and
  string selectors.
- OCCT reference manual: `TNaming_Evolution` (PRIMITIVE, GENERATED, MODIFY, DELETE, REPLACE,
  SELECTED), `TNaming_NameType`, package TNaming
  (https://dev.opencascade.org/doc/refman/html/_t_naming___evolution_8hxx.html,
  https://dev.opencascade.org/doc/occt-7.4.0/refman/html/package_tnaming.html);
  `BRepTools_History`, `BRepBuilderAPI_MakeShape` (see `occt-integration.md` §7).
- SolveSpace source, `src/sketch.h` (`EntityKey`, `Group::Remap`, `REMAP_*`)
  (https://github.com/solvespace/solvespace/blob/master/src/sketch.h).
- Autodesk Fusion API: `Design.findEntityByToken`, `BRepFace.entityToken`
  (https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/Design_findEntityByToken.htm,
  https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/BRepFace_entityToken.htm).

## 15. Owner questions
Also listed in `docs/QUESTIONS.md` (#43–#49). **Answered 2026-10-21:** the owner took every
recommendation: 1 (a), 2 (c), 3 (a), 4 (a) now and (c) later, 5 (a), 6 (a), 7 (a). The note's
text already follows these options, so nothing below changes.

| # | Question | Options | Recommendation |
|---|---|---|---|
| 1 | **Pattern count change.** A ref into pattern instance *k*: when the count or spacing changes, what does it follow? | (a) The instance number, quietly; a vanished instance is `NotFound` (Fusion, Onshape). (b) The instance number, plus a `FarMatch` warning when the instance moved by d > `FAR`. (c) The geometric position: the instance nearest to the old one. | **(a).** It is what Fusion users expect and it is deterministic. (c) jumps between instances as the spacing changes. (b) would warn on every pattern edit (TNP-17 has d = 2.03). TNP-17/18 assume (a). |
| 2 | **Partial resolution of multi-entity selections** (fillet/chamfer edges, shell/draft faces). | (a) `DropMissing`: drop lost and ambiguous members with a warning; fail only if none resolve. (b) `AllOrError`: any lost member fails the feature. (c) As (a), but a member whose candidates are all new-split pieces of it (the reference predates the split) expands to *all* pieces, with a `NewSplit` warning. | **(c).** A user who filleted "this edge" means every piece of it after an upstream cut splits it. (b) makes long timelines brittle. The warning keeps it visible. §5.1 and §6.4 are written for (c). |
| 3 | **Should `Widened` and `NewSplit` warn**, or resolve quietly when the pick is clear? | (a) Both warn (current). (b) `Widened` quiet (the whole face contains what was picked), `NewSplit` warns. (c) Both quiet when the ratio test is clear. | **(a)** for now. Both change what the feature acts on (a fillet over the whole edge, or over one piece only), which is what R2 says not to hide. "Accept all" makes clearing them cheap. Revisit after real use; (b) only changes §6.4's precedence and TNP-08's expectation. |
| 4 | **Fingerprint-only rescue for redrawn sketch curves.** A curve is deleted and redrawn in the same place (new `EntityId`). | (a) Stage 3 accepts an identical entity (d ≤ 0.05, unique) with a `FingerprintOnly` warning (current, TNP-13). (b) Never rescue: `NotFound`. (c) (a), plus a later sketch tool "Replace curve" that keeps the old id, so redrawing doesn't break lineage at all. | **(a) now, (c) later** (an S03 P1 tool, not part of DOC-06). (b) makes a common user mistake fatal for every downstream feature. (a) is always visible, and it is the only path that can be wrong (F16). |
| 5 | **Threshold acceptance.** `PICK_RATIO` 0.5, `QUIET_RATIO` 0.25, `FAR` 1.0, `FP_ONLY_MAX` 0.05, `FP_ONLY_UNIQUE` 4, and the metric weights. | (a) Accept the provisional values now. DOC-06/DOC-13 re-tune them on the corpus (held-out cases and the fuzz set must stay green); every change bumps `NAMING_VERSION` and is logged in §13. (b) Don't accept the note until DOC-06 has measured them. (c) Make them user settings. | **(a).** The values only move where the corpus shows a reason, and versioning keeps old caches honest. (c) is ruled out: resolutions must be identical on every machine (ADR-014), so thresholds can't be per user. |
| 6 | **Simplify every boolean result** (`unify_same_domain` after join/cut/combine, §5.1). | (a) Always (Fusion behaviour). (b) Only for join. (c) Never; the user runs "merge faces" explicitly. | **(a).** Coplanar leftovers confuse picking and naming (TNP-20), and Fusion users expect merged faces. Cost: one unify pass per boolean, and `Widened` warnings on refs into newly merged faces. |
| 7 | **Re-pick keeps the selection key** (§5.1, §7): after a member is re-picked, downstream refs into the feature's generated faces follow the new entity. | (a) Keep the key (re-pick = replace). (b) New key (re-pick = remove + add; downstream refs into the old blend go to stage 3 or `NotFound`). | **(a).** Re-pick is how users repair a broken reference, and they expect the repair to flow downstream (Fusion does this). Remove + add still gives (b) when that is wanted. |

## 16. Revision 2026-10-20 (review response)
A critical review (9 must-fix and 6 nice-to-have findings, plus areas it hadn't covered) led to
these changes. Section numbers point to the new text.

**Must-fix findings**
1. *TNP-20 didn't follow from the candidate rules.* Added candidate source (c), a reference's
   merge partners, and the class **unmerged** with its own warning (§6.2). TNP-20's text now
   derives `Ambiguous` from the rules (§11.4).
2. *Profile keys change on topology edits, which would break TNP-13/14/16.* Caps are named by
   the region's outer-loop source set, not the `ProfileId` (§3.1, §5.4, with a table of what
   stays stable). Checked against the SK-06 implementation: `ProfileId` hashes all boundary
   edge keys, holes included, so it does change when a hole is added. TNP-14/16 were fine
   under both rules (their regions are disjoint from the edit). TNP-13 still gives
   `FingerprintOnly`, and the extrude warns too. New TNP-29/30 cover hole addition and
   profile reordering.
3. *Accept match cascaded renames.* Picked inputs now generate `Selection { key }` sources
   from a stable per-selection `TopoRef.key`, which Accept match and Re-pick keep (§5.1, §7,
   §10). New TNP-24.
4. *∞ arithmetic in the ratio test.* `distance` returns `None` on a class mismatch. Such
   candidates are excluded from the ratio; if all mismatch, the result is `Ambiguous`; one
   finite candidate gets `ClassChanged` and is never quiet; a 0/0 ratio is a tie (§6.3,
   §6.4). `DropMissing` handling of `Ambiguous` members is defined (§5.1, owner question 2).
5. *`naive_index_breaks` was asserted, not measured.* The harness measures the naive baseline
   on every case and records it; the flag is now a prediction; DOC-13 requires ≥ 5 *measured*
   breaks (§11.2, S01 DOC-13). TNP-02's edit now inserts the point at the start of the list.
6. *Fillet corner names needed vertex history that S02 doesn't give.* Role-only faces are
   named from their adjacent faces (§4.2 rule 4). New TNP-27 and S02 request Q6.
7. *Tolerance-based sort keys aren't a total order.* Quantised integer sort keys (§4.3), and
   S02 request Q5 for the split order. TNP-26 pins the tie-break for congruent pieces.
8. *Overlapping §4.2 rules; tombstone lookup unclear.* The rules are exclusive and ordered;
   Split comes before Merge; coplanar overlap regions are merges (§4.2). Tombstones are looked
   up by lineage with wildcards (§4.4).
9. *TNP-20's base needs a unify contract; face-plane sketch coordinates were undefined.* Every
   BodyOp result is unified (§5.1, S04 request, owner question 6). Tied `$pick`s are harness
   errors. The planar-face frame rule is an S03 request, and the harness converts world
   coordinates (§11.2).

**Nice-to-have findings applied**: a stage-1 sanity check for wildcard matches, and
`BodyChanged` for split-piece lineage (§6.1, §6.4); Copy uses `instanced` (§5.2); name depth
capped by `Digest`, plus interning and reuse, and the §9 targets restated as estimates with a
realistic memory figure; distinct labels (`Unmerged`, `Widened{cause}`,
`NewSplit{feature: None}`); corpus gaps closed by TNP-24..30 (Accept match, set-valued
`DropMissing`, seen split with congruent pieces, fillet corner, save/load, hole in a region,
profile reorder) and a held-out fuzz set (§11.6); error cases record `intended`, and "silent
wrong" is defined (§11.2).

**Previously uncovered areas**: OCCT history guarantees, op by op, and what the scheme does
where OCCT promises nothing (§6.6: boolean completeness, coplanar fuse, UnifySameDomain,
fillet stripes and corners, pipe/thru-sections coverage, shell images, sub-shape order under
transforms; mirrors go through history, §4.6). Counter-examples to "never a silent wrong
resolution" (§6.7: symmetric parts, patterns, congruent split pieces, coalesced coplanar
faces, jumping boundaries, re-pick, unexplained faces, digest collisions). The S01
`RecomputeCtx` recording needs a `Mutex` and sequential `resolve` calls; `ResolutionRecord`
covers profile refs and expanded set members; compatibility with the DOC-01 code (§10).

**Owner questions** are now an explicit section (§15) and in `docs/QUESTIONS.md`.
