# S01 — Document model, timeline & persistent naming

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P0 |
| **Crates** | `amg-doc` |
| **Depends on** | S00 (FND-03 ids/units/errors, FND-04 jobs, FND-07 crash hook); data types from S02 (BREP-02 `Shape`, BREP-05 history), S03 (SK-01 entity/profile keys), S07 (MESH-02 `IndexedMesh`), S08, S09 — stubbed in `amg-doc::stub` until those land |
| **Lane** | E |
| **Parity** | Timeline, parameters, components/bodies browser, undo, save/open |

## Summary
The heart of the app: the design tree (components, bodies, sketches, construction geometry),
the parametric timeline with incremental recompute, parameters with unit-aware expressions,
persistent references to topology (the "topological naming problem"), commands + undo/redo,
and the `.amg` file format.

## Scope
**In:** data model (authored state + evaluated state), ids, persistent snapshots, `Feature`
trait + registry + serialization, timeline (marker, suppression, groups, reorder), recompute
engine (dependency graph, cache keys, health propagation) and its job integration
(snapshot, cancel, supersede, atomic swap), `EntityRef`/`TopoName` types and the resolution
framework, parameters + expression language, `Command` enum + undo/redo + transactions,
direct (no-history) mode, `.amg` file format with versioning/migrations, geometry cache
persistence, autosave + crash journal + recovery, `DocEvent` change notifications,
configurations (P2), opaque extension records for studies/print setups.
**Out:** concrete features (S04, S05 and domain specs; `amg-features`), dialog schemas
(S04 SOL-01), UI (S11; the `DocEvent` → `RenderScene` adapter lives in `amg-ui`), joint
solving and assembly commands beyond the component tree (S06), study data models (S13/S14/S16
store their data through extension records), the persistent-naming *scheme* itself (DOC-11
design note; this spec fixes the framework and provisional types).

## Design

### 1. Authored state vs evaluated state
The draft put geometry, health and caches into the document tree. The refined model splits it:

- **Authored state** (`Design`): everything the user decided — timeline, parameters, component
  tree, occurrences, per-body properties, extension records. Small, serializable, immutable
  snapshots (`DesignSnapshot = Arc<Design>`), the unit of undo and of `design.json`.
- **Evaluated state** (`Evaluated`): everything recompute derives — body geometry, solved
  sketches, construction geometry, naming maps, per-feature health, cache keys. Produced by a
  recompute job from a snapshot, swapped in atomically on the UI thread, never undone (undo
  changes the authored state; recompute then hits the cache, so undo is instant).

```
Document (UI thread)
 ├─ current: DesignSnapshot (Arc<Design>)          ← commands produce new snapshots
 ├─ undo: UndoStack { entries: [ {label, before, after, commands} ], cursor }
 ├─ evaluated: Arc<Evaluated> (revision r)        ← swapped in when a recompute job finishes
 ├─ cache: Arc<GeomCache>                          ← shared with jobs; keyed by CacheKey
 └─ pending: Option<JobHandle<RecomputeResult>>
Design
 ├─ meta: DesignMeta { uuid, name, default units, mode: History|Direct, next_id }
 ├─ params: ParamTable
 ├─ timeline: Timeline { nodes: PVec<FeatureNode>, marker, groups }
 ├─ components: PMap<ComponentId, Arc<Component>>, root: ComponentId
 ├─ occurrences: PMap<OccurrenceId, Arc<Occurrence>>
 ├─ body_props: PMap<BodyId, Arc<BodyProps>>     (name, material, appearance, visible, opacity)
 ├─ baked: PMap<BodyId, BlobRef>                  (direct mode / base feature bodies)
 └─ extensions: PMap<ExtKey, Arc<ExtRecord>>      (studies, print setups … opaque JSON + blobs)
```

**Identity.** All authored ids (`FeatureId`, `ParamId`, `ComponentId`, `OccurrenceId`, `GroupId`,
`ConfigId`) are typed `u64`s allocated from `DesignMeta::next_id` (monotonic, never reused,
persisted in the file). This makes ids stable across save/load without a runtime↔file id map
and keeps recompute deterministic (no random UUIDs in anything that feeds a cache key). Because
ids are never reused, generations are unnecessary. `Uuid` (v7, amg-core) is used only for the
document identity (recovery folder, linked designs ASM-08).
**Derived ids** need no allocation: a body is identified by the feature that created it plus an
output slot (`BodyId { origin: FeatureId, slot: u32 }`), so recompute on a worker thread never
has to mutate authored state to name new bodies. Sketch and construction ids are the ids of
their creating features (`SketchId(FeatureId)`, `ConstructionId(FeatureId)`).

**Snapshots — open question 1 resolved: `imbl`.** Authored collections are persistent ordered
maps/vectors from `imbl` (MPL-2.0), wrapped in crate-private aliases `PMap = imbl::OrdMap`,
`PVec = imbl::Vector` so the backend can be swapped. Values are `Arc`ed so node copies are
pointer copies. Reasons:
- `im` is dead: its repository was archived on 2026-05-03 and RustSec RUSTSEC-2026-0250 (on the
  `im-rc` sibling) recommends the maintained fork **`imbl`** (7.0.x, MSRV 1.85), which also
  fixed OrdMap/Vector bugs and improved HashMap/OrdMap performance relative to `im`.
- `imbl` mutates in place when a node is uniquely owned (copy-on-write at node granularity),
  so the common case — mutate the working copy, snapshot rarely — costs about the same as a
  `BTreeMap`. `clone()` is O(1). `OrdMap::diff()` lets `DocEvent` generation skip shared
  subtrees.
- `rpds` (1.2.1, MPL-2.0, maintained) is a viable alternative (`new_sync()` Arc variants,
  `insert_mut`), but it has no ordered diff and no RRB vector with cheap split/concat (needed
  for timeline reorder). It stays as the fallback behind the alias.
- Per-entity `Arc` copy-on-write over plain `Vec<Arc<T>>` is O(n) per first mutation after a
  snapshot. That is fine at n ≈ 1 000 (≈ 5–20 µs) but grows linearly; the DOC-01 benchmark keeps
  it as the baseline.
- Ordered maps (not hash maps) give deterministic iteration order, so `design.json` output
  and cache keys are byte-stable. **Rule:** no `std::collections::HashMap`/`HashSet`
  iteration in anything that feeds serialization, cache keys or event order.

DOC-01 still ships a criterion benchmark (`imbl` vs `rpds` vs `Arc<Vec<Arc<T>>>`, 10³/10⁴/10⁵
entries; clone, 1 mutation after a clone, 100 mutations, ordered iteration, diff). If `imbl`
misses the < 1 µs snapshot / < 5 µs single-mutation targets at 10⁴ entries, switch the alias
and record why in the Refinement log.

**Geometry stubs.** `amg-doc` depends on domain crates for their data types only. Until a
domain type exists, `amg-doc::stub` defines a placeholder with the same name. Each stub is
replaced by a re-export when the owning WP lands: BREP-02 (`Shape`), SK-01 (`SketchGeom`,
`SketchEntityKey`, `ProfileKey`), MESH-02 (`IndexedMesh`), IMP-01 (`SdfGraph`), FRM-01
(`SubdMesh`).

### 2. Features, timeline, registry
- `Timeline { nodes: PVec<FeatureNode>, marker: usize, groups: PMap<GroupId, Group> }`.
  `marker` = number of nodes that are rolled forward; nodes at index ≥ marker are not computed
  (Fusion rollback). Nodes are immutable `Arc<dyn Feature>`; editing replaces the `Arc`.
- `FeatureNode { id, feature, suppressed, group, name, context: ComponentId }`. Health is
  **not** stored in the node: it is evaluated state (`NodeEval`), so undo never fights recompute.
- **Ordering rule:** a feature may depend only on features earlier in the timeline. `MoveFeature`
  validates this against `inputs()` and fails with `CommandError::WouldBreakDependency`.
- **Registry — `inventory` + explicit registration records, no `typetag`.** Each feature type
  submits a `FeatureRegistration` (kind string, current schema version, deserializer, migration
  fn) with `amg_doc::register_feature!`. `typetag` (which itself sits on `inventory`) was
  rejected because it gives no per-type version/migration hook and no way to keep an unknown
  kind as raw data. With the registry, a file containing a kind this build doesn't know loads
  as `UnknownFeature { kind, version, raw }` (health `Error(UnknownKind)`) and saves back
  byte-identical. Duplicate kinds are a startup error.
- **Linker caveat:** `inventory` only sees code that is linked. A crate whose symbols are never
  referenced can be dropped, and its features silently vanish. Every crate that registers
  features exports `pub fn link() {}`, and the binaries (`amg-app`, `amg-cli`, `amg-script`)
  call `amg_features::link()`. A test asserts the expected kinds are present.
- **Serialization:** `FeatureBox` (newtype over `Arc<dyn Feature>`) implements serde through
  the registry as `{"kind": "...", "v": 3, "data": {...}}`. Feature structs derive
  `Serialize/Deserialize` and get `to_json()` from a blanket impl in `register_feature!`.
  Feature structs must not contain hash maps (use `BTreeMap`); `f64` fields are allowed
  (`serde_json` uses shortest round-trip formatting via `ryu`, which is deterministic).

### 3. Recompute engine
**Dependencies.** A feature declares `inputs()`: bodies it reads (`Dependency::Body(BodyId)`),
entities (`Dependency::Entity(EntityRef)`), sketches, construction, occurrences, plus every
`Expr` it uses (parameter dependencies are derived from them). A body read by feature *F*
means "the state of that body just before *F*", i.e. the output of the last earlier
non-suppressed feature that wrote it. The engine builds this producer map in one linear pass.

**Cache keys (constructive traces).** For each node in timeline order:
```
key(F) = blake3( ENGINE_SALT ‖ kind ‖ schema_version ‖ canonical_json(F)
                ‖ for each Expr param p used: p.name ‖ value_bits(p)
                ‖ for each input: key(producer of that input state)
                ‖ context component transform bits ‖ compute mode )
```
- `ENGINE_SALT` = `amg-doc` geometry-format constant ‖ `amg_brep::kernel_version()` ‖
  tolerance settings. A kernel upgrade invalidates every cache; nothing is silently reused.
- `value_bits`: `f64::to_bits` after normalising `-0.0 → 0.0`; NaN is an evaluation error and
  never reaches a key.
- Upstream *keys*, not upstream *geometry content*, are chained (Mokhov, Mitchell & Peyton
  Jones, *Build systems à la carte*, ICFP 2018: "constructive traces"). Hashing B-rep content
  for early cutoff costs more than it saves at our feature counts. Early cutoff is a possible
  later optimisation (DOC-12 notes).
- blake3 output is 256 bits, used in full. A collision would mean silently wrong geometry, so
  64-bit hashes are not acceptable.

Editing feature *k* changes `key(k)`. Only nodes whose keys transitively include it miss the
cache; every other node hits. This is the incremental property DOC-03 tests with counters.

**Cache.** `GeomCache`: an in-memory map `CacheKey → Arc<FeatureOutput>` behind a `Mutex`, LRU by
estimated bytes (default 2 GiB budget, a preference). It is shared by the UI thread and jobs.
Entries are immutable, so readers never contend for long. Preview-mode outputs (`ComputeMode::Preview`)
use keys of their own and are never persisted.

**Evaluation pass.** The pass walks nodes `0..marker` in order:
- Suppressed node → `Health::Suppressed`; its outputs pass through unchanged.
- Cache hit → reuse the output.
- Otherwise call `compute(ctx)`.

Independent subgraphs may later run in parallel (an OCCT per-shape-graph lock exists in S02).
DOC-03 is sequential; parallel evaluation is a follow-up once profiles justify it.

**Health propagation (Fusion behaviour).** A failing feature never aborts the pass:
- `Health::Error`: the feature has **no effect**. Bodies it would have modified pass through
  unmodified, and bodies it would have created don't exist.
- A downstream feature whose input body doesn't exist, or whose `EntityRef` doesn't resolve,
  gets its own `Error` with `ErrorCause::Upstream(FeatureId)` or `ErrorCause::Resolve(..)`.
- Warnings (fingerprint-only match, healed shape, preview fallback) are `Health::Warning(Vec<Diagnostic>)`.
- The timeline shows the first failing feature as red and dependants as red-with-cause.

**Job integration (DOC-12).**
- `Document::poll()` runs once per UI frame and never blocks. If `current.revision` is newer than
  the evaluated revision, it cancels the running job (`JobHandle::cancel`), then spawns
  `RecomputeJob { snapshot, revision, cache, mode }` on `amg-jobs`.
- Cancellation is cooperative. `RecomputeCtx::check_cancelled()` is called by the engine between
  features and by features inside long loops. Outputs of features that finished before the
  cancel stay in the cache, so the superseding job reuses them.
- A finished job whose revision is no longer current is dropped. Otherwise `poll` swaps
  `evaluated` (one `Arc` store) and emits `DocEvent`s. Progress is per feature (`i / n` plus the
  feature name), plus whatever the feature itself reports.
- Headless callers (CLI, Python, tests) use `Document::recompute_blocking()`, which must not be
  called on the UI thread (debug assertion via a thread-local "UI thread" marker set by
  `amg-app`).

**Threading requirement on geometry.** `Evaluated` and `GeomCache` cross threads in `Arc`s, so
every geometry payload must be `Send + Sync`. S02 currently says OCCT objects are "Send but not
Sync" (tessellation mutates the shape). **Interface request to S02:** `amg_brep::Shape` must be
an immutable, `Sync` handle that serialises internally any operation that mutates OCCT state
(tessellation cache, BRepMesh). Until then, `BodyGeom::Solid` wraps it in a mutex.

### 4. Persistent naming (framework here, scheme in DOC-11)
Features never store face/edge/vertex indices. They store `EntityRef`s (see Interfaces), whose
topological variant is a `TopoRef { body, kind, name: TopoName, fingerprint }`.

- **Element map.** For every evaluated solid body the engine keeps `BodyNaming`: for each
  in-shape sub-shape index (S02 `TopTools_IndexedMapOfShape` order) the `TopoName` of that
  element. This is the same idea as FreeCAD 1.0's element map (realthunder's mitigation,
  merged in FreeCAD 1.0, Nov 2024). The framework stores and resolves names but does not know
  how a given feature generates them.
- **Ingestion.** Features build `BodyNaming` from `amg_brep::ShapeHistory`
  (generated/modified/deleted, BREP-05) through `NamingBuilder`, plus feature-specific roles
  (e.g. `Extrude/Lateral from SketchCurve`). Per-feature naming rules are a DOC-11 deliverable.
- **Resolution (`RecomputeCtx::resolve`).** Stages:
  1. Exact name match → `MatchQuality::Exact`.
  2. Name match with several candidates (split) → pick by fingerprint proximity →
     `Disambiguated`, a warning if the margin is small.
  3. Fingerprint-only match within tolerance → `FingerprintOnly` (always a warning).
  4. Otherwise `ResolveError` → feature error; the user re-picks.

  **Hard rule:** the resolver must never return a wrong match silently. When unsure, it returns
  an error or a warning.
- Mesh bodies are referenced by face-group id + fingerprint (`TopoKind::FaceGroup`). Implicit
  and form bodies have no sub-entity references (body-level only).
- **Provisional:** the types in Interfaces are enough for S03 (projection), S04 (feature
  inputs), S06 (joint origins) and S13 (BC mapping) to write code against. Their *contents* (the
  `NameSeg` grammar, fingerprint fields and tolerances) may change when DOC-11 is accepted.
  Callers treat `TopoName` as opaque and only go through `resolve()`.
- **Prior art for DOC-11:**
  - Kripac, *A mechanism for persistently naming topological entities in history-based
    parametric solid models*, CAD 29(2), 1997.
  - Marcheix & Pierra, *A survey of the persistent naming problem*, SMA 2002.
  - Farjana & Han, *Mechanisms of persistent identification of topological entities in CAD
    systems: a review*, Alexandria Eng. J. 2018.
  - FreeCAD 1.0 element map / `MappedName` (`src/App/ElementMap*`,
    realthunder/FreeCAD_assembly3 wiki "Topological Naming").
  - Onshape deterministic ids + queries.
  - CadQuery / build123d selectors.
  - OCCT `TNaming` (OCAF).

### 5. Parameters & expression language
**Parameter types.**
- `ParamTable` holds `Param { id, name, expr: ExprSrc, kind: User | Model { owner: FeatureId },
  unit: Option<UnitSpec>, comment, favorite }`.
- **Model parameters (Fusion behaviour).** Every `Expr` field of a feature becomes a model
  parameter `d<n>` when the feature is added. `AddFeature` walks `Feature::exprs_mut()`,
  creates a model parameter for each `Expr::Inline(src)`, and rewrites the field to
  `Expr::Param(id)`. So every dimension is listed and editable in the Parameters dialog
  (UI-12), and features only ever hold `Expr::Param`.
- Deleting a feature turns its model parameters into user parameters if anything else still
  references them. Otherwise they are deleted with the feature.
- Evaluated values (`ParamValues`) are computed on the UI thread at commit time (cheap; target
  below). A bad expression is rejected by the command immediately and never reaches recompute.

**Decision: a hand-written lexer + Pratt parser, no crate.** Reasons:
- `evalexpr` is AGPL-3.0 since its recent major versions (13.x; older versions were MIT), and
  it has no units.
- `meval` and `fasteval` are unmaintained and have no dimensional analysis.
- None of them supports Fusion-style unit literals, reserved unit identifiers or rename
  refactoring.
- The grammar is about 300 lines of Rust and is fully under our control (error spans for the
  expression input widget, UI-05).

**Grammar (EBNF).** Whitespace is insignificant except as a token separator.
```
expr     = sum ;
sum      = product , { ("+" | "-") , product } ;
product  = unary , { ("*" | "/") , unary } ;
unary    = ("-" | "+") , unary | power ;
power    = quantity , [ "^" , unary ] ;                (* right-associative *)
quantity = fraction , [ unit_expr ]                     (* "10 mm", "1/4 in", "2.5e3 N" *)
         | primary ;
fraction = number , [ "/" , number ] ;                  (* literal/literal only, binds before the unit *)
unit_expr= unit , { ("*" | "/") , unit } , [ "^" , int ] ;   (* "N/mm^2", "kg*m" *)
primary  = number | ident | call | "(" , expr , ")" ;
call     = ident , "(" , [ expr , { "," , expr } ] , ")" ;
number   = digit , { digit } , [ "." , { digit } ] , [ ("e" | "E") , [ "+" | "-" ] , digit , { digit } ] ;
ident    = (letter | "_") , { letter | digit | "_" } ;
```
**Semantics.**
- Unit names (`mm cm m um in ft deg rad kg g N kN MPa GPa Pa s K degC …`, taken from the
  amg-core unit table) are **reserved identifiers**. They can't be parameter names, and a bare
  unit evaluates to one of that unit, so `(a + b) * 1 mm` and `d1 / 5 mm` work.
- `1/4 in` is the literal 0.25 in, not 1/(4 in).

**Dimensions.**
- `Dim` = integer exponents over [L, M, T, Θ, I, N, J, A]. Angle (A) is a pseudo-base, so
  length/angle mix-ups are caught (Fusion also treats angles as their own unit kind).
- Values are stored in the internal coherent system: mm, kg, s, K, A, mol, cd, rad.
- `+`/`-` require equal `Dim`. `*` and `/` combine dimensions.
- `^` needs a dimensionless exponent; a dimensioned base also needs an integer-valued exponent.
- Dimensions are checked during evaluation (all values are known), with the span of the
  offending node.
- Affine units (`degC`, `degF`) are allowed only as the sole unit of an entire expression;
  temperature differences use K.

**Bare numbers.**
- A bare numeric literal as a whole expression in a typed field takes the document default unit
  of that field's dimension (`10` in a length field = 10 mm).
- Inside `+`/`-` next to a length or angle operand it does the same (`d1 + 5` = d1 + 5 mm),
  which matches Fusion's lenient behaviour. This leniency is owner question 3. A strict mode is
  one flag.

**Functions and constants.**
- `sin cos tan` take an angle or a dimensionless value (radians).
- `asin acos atan atan2` return an angle.
- `sqrt` needs even dimension exponents; `abs` keeps the dimension; `hypot`.
- `min max` take arguments of equal dimension.
- `floor ceil round sign exp ln log10 pow` take dimensionless arguments only (users write
  `floor(d1 / 5 mm) * 5 mm`). This deliberately avoids "round in which unit?" ambiguity.
- Constants: `PI`, `pi`, `E`.

**References and cycles.**
- Parameter references bind by name at parse time, to `ParamId`.
- On `SetParamExpr` the table checks for a cycle: DFS from the new references back to the
  parameter → `ExprError::Cycle(path)`. Evaluation runs in topological order (Kahn).
- `RenameParam` rewrites the source text of every referencing expression through token spans,
  so expressions stay human-readable.

**Formatting.** `Quantity::format(unit, precision)` delegates to amg-core's unit formatter.

### 6. Commands, undo/redo, transactions
- `Command` (serde, closed enum in `amg-doc`; see Interfaces) is the **only** mutation path, and
  the Python API mirrors it 1:1. Domain lanes add *feature types*, never commands.
- **Undo decision: snapshot-based, not inverse commands.** With persistent collections a
  snapshot is an `Arc` clone, and an undo entry shares all unchanged structure with its
  neighbours. Inverse commands would need a correct hand-written inverse for every command,
  including cascades (deleting a feature converts model parameters, invalidates refs, and so on),
  and every such inverse is a source of undo bugs.
  - Each entry is `UndoEntry { label, before, after, commands }`.
  - Undo sets `current = before`; redo sets `current = after`. Both are O(1).
  - The recompute that follows hits the cache, because keys are pure functions of authored state.
  - Depth limit: 500 entries by default. Older entries are dropped (memory is bounded by shared
    structure, typically kilobytes per entry, because geometry is not in the authored state).
- `commands` is kept for the crash journal (§8), macro recording and scripting history.
- **Transactions:** `doc.transaction("Fillet")` groups several commands into one undo entry.
  An error inside rolls back to `before`. Tool previews run on a scratch snapshot (`Design`
  clone + `ComputeMode::Preview`) and never touch the undo stack.
- Undo history is not saved in the file (Fusion doesn't either; owner question 4).

### 7. Direct mode
- `DesignMode::Direct`: new features compute once, their outputs are baked
  (`Design::baked: BodyId → BlobRef`) and the node is dropped (no timeline).
- History→Direct bakes every evaluated body and clears the timeline (one undoable command).
- Direct→History creates a single `BaseFeature` node holding the baked bodies (Fusion's "base
  feature") and starts a fresh timeline after it.
- Baked bodies are stored in `bodies/` in the file and keep their `BodyId`s, so body properties
  and extension references survive the switch.

### 8. File format, versioning, caches, recovery
**Container.** Zip (`zip` crate). Entries:

| Entry | Content | Compression |
|---|---|---|
| `manifest.json` | `{ "format": "amalgam", "format_version": N, "app_version", "created_by", "entries": {name: blake3} }` | deflate |
| `design.json` | the serialized `Design` | deflate |
| `bodies/<body-id>.<ext>` | baked bodies; imported meshes (MESH binary from S07, `.brep` from S02) | deflate (mesh) / store |
| `cache/index.json` + `cache/<key-hex>.<ext>` | persisted `GeomCache` outputs for every timeline node, salted (`ENGINE_SALT` stored in the index) | store / deflate |
| `ext/<key>/…` | extension records' blobs (study results, print setups) | per record |
| `thumbnail.png` | headless render (RND-09) when available | store |

**Versioning and migration.**
- `format_version` (document level) and per-feature `v` (per kind) are separate.
- Document migrations form a chain `fn(serde_json::Value) -> Result<Value, MigrationError>`,
  one step per version: `v1→v2→…→current`, applied to the raw JSON before typed
  deserialization.
- Feature migrations run inside the registry: `migrate(from_v, raw)`.
- A file newer than this build → `FormatError::TooNew`. We never guess at it, and the file stays
  untouched.
- Every migration step has a fixture file in `tests/fixtures/format/vN.amg` and a test that
  loads it. Fixtures are never edited once committed.

**Errors that explain themselves.**
- Zip CRC mismatch or a manifest blake3 mismatch → `FormatError::Corrupt { entry }`.
- A JSON error reports the JSON path through `serde_path_to_error`
  (`design.timeline.nodes[12].data.extent: invalid type`).
- A missing feature kind → `UnknownFeature`; the file still loads.
- A missing or stale cache → silent recompute.

**Safe save.**
- Write to `<name>.amg.tmp` in the same directory, flush and `sync_all`, then rename over the
  target (`std::fs::rename` replaces atomically on NTFS/ext4).
- The previous file is kept as `<name>.amg.bak` (preference, on by default).
- Serialization runs on a job against a snapshot, so the UI never blocks. Only the final rename
  is reported back.

**Opening without recompute.**
- On open, `cache/index.json` seeds `GeomCache` lazily. Entries load on first hit, and only if
  the stored salt equals the running `ENGINE_SALT`.
- With a complete cache, opening a design deserializes geometry and computes nothing.
- "Save compact" omits `cache/` (owner question 2: default).

**Autosave and crash recovery (DOC-14).**
- **Journal.** After every committed undo entry, its `commands` are appended as one JSON line to
  `<recovery>/<doc-uuid>/journal.jsonl`. This is cheap and runs on a background writer thread.
- **Recovery snapshot.** Every N minutes (default 5, preference) when dirty, a background job
  writes `<recovery>/<doc-uuid>/snapshot.amg` (compact, no caches) and truncates the journal.
- **Recovery** = load the snapshot + replay the journal through `Document::apply`.
- **Lock.** A `lock` file is held open with `File::try_lock` (std, Rust ≥ 1.89) while the
  document is open. At startup, recovery folders whose lock can be acquired belong to dead
  sessions → offer a restore.
- A clean close deletes the folder.
- FND-07's panic hook calls `RecoveryHook::flush()` (the journal is already on disk; this just
  syncs it).
- The recovery directory is passed in by the app (`%LOCALAPPDATA%\Amalgam\recovery` on Windows,
  `$XDG_STATE_HOME/amalgam/recovery` on Linux). `amg-doc` doesn't hard-code paths.

### 9. Change events
- `DocEvent`s are generated on the UI thread only, at two points:
  - **Commit/undo/redo:** `OrdMap::diff` of the authored maps before and after.
  - **Evaluated swap:** comparing `Arc::ptr_eq` of body geometry before and after, and health
    changes.
- Consumers pull them with `Document::drain_events()` once per frame. There is no channel and
  no callback, so there are no re-entrancy problems with egui.
- Events carry ids only, never geometry. The `amg-ui` adapter reads the new geometry from
  `Document::evaluated()` and updates `RenderScene` (S10), so `amg-render` never sees
  `amg-doc` types.
- **Minimality:** an unchanged body (same `Arc`) produces no event. A properties-only change
  produces `BodyPropsChanged`, not `BodyGeometryChanged`.

### 10. Configurations (P2, DOC-10)
- `ConfigTable { configs: PMap<ConfigId, Config>, active }`. A `Config` overrides parameter
  expressions and suppression flags.
- Activating a config is a command. The overrides apply to the snapshot fed to recompute, so
  each configuration has its own cache keys and switching back is a cache hit.

### 11. Extension records
- Studies (S13/S14), print setups (S16) and similar state live outside the timeline as
  `ExtRecord { kind, version, data: serde_json::Value, blobs: Vec<BlobRef> }` under an `ExtKey`.
- Owning crates register `ExtRegistration { kind, version, migrate }` (`inventory`, like
  features) and get undo, save/load, migration and events (`ExtChanged`) for free.
- `amg-doc` never interprets `data`.

## Interfaces
Provisional Rust signatures. Exact names may change during implementation, but the shapes are
what other lanes code against. Types marked *(stub)* are re-exports once the owning WP lands.

```rust
// ===== ids (amg-doc::id) =====
macro_rules! typed_id { ($($n:ident),*) => {$(
    #[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
    #[serde(transparent)] pub struct $n(pub u64);
)*}}
typed_id!(FeatureId, ParamId, ComponentId, OccurrenceId, GroupId, ConfigId);

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct BodyId { pub origin: FeatureId, pub slot: u32 }
pub type BodyRef = BodyId;                       // name used in S04
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct SketchId(pub FeatureId);
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ConstructionId(pub FeatureId);
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ExtKey { pub kind: String, pub id: u64 }
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Revision(pub u64);                    // bumps on every authored change

// ===== authored state (amg-doc::model) =====
pub type DesignSnapshot = Arc<Design>;
#[derive(Clone)]
pub struct Design { /* fields per §1; persistent maps are crate-private */ }
impl Design {
    pub fn meta(&self) -> &DesignMeta;
    pub fn root(&self) -> ComponentId;
    pub fn component(&self, id: ComponentId) -> Option<&Component>;
    pub fn components(&self) -> impl Iterator<Item = (ComponentId, &Component)> + '_;
    pub fn occurrence(&self, id: OccurrenceId) -> Option<&Occurrence>;
    pub fn body_props(&self, id: BodyId) -> BodyProps;          // default if never set
    pub fn timeline(&self) -> &Timeline;
    pub fn params(&self) -> &ParamTable;
    pub fn ext(&self, key: &ExtKey) -> Option<&ExtRecord>;
    pub fn revision(&self) -> Revision;
}
pub struct DesignMeta { pub uuid: Uuid, pub name: String, pub mode: DesignMode,
                        pub default_units: UnitPrefs, pub tolerance: Tolerance }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DesignMode { History, Direct }
pub struct Component { pub name: String, pub children: Vec<OccurrenceId>, pub joints: Vec<JointId /* S06 */> }
pub struct Occurrence { pub component: ComponentId, pub parent: ComponentId, pub transform: Transform,
                        pub grounded: bool, pub visible: bool, pub name: Option<String> }
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyProps { pub name: Option<String>, pub material: Option<MaterialKey /* S17 */>,
                       pub appearance: Option<AppearanceKey /* S17 */>, pub visible: bool, pub opacity: f32 }

// ===== evaluated state (amg-doc::eval) =====
pub struct Evaluated {
    pub revision: Revision,
    /* crate-private maps */
}
impl Evaluated {
    pub fn body(&self, id: BodyId) -> Option<&Arc<EvalBody>>;
    pub fn bodies(&self) -> impl Iterator<Item = (BodyId, &Arc<EvalBody>)> + '_;
    pub fn sketch(&self, id: SketchId) -> Option<&Arc<SketchGeom /* stub: S03 */>>;
    pub fn construction(&self, id: ConstructionId) -> Option<&ConstructionGeom>;
    pub fn node(&self, id: FeatureId) -> Option<&NodeEval>;
}
pub struct EvalBody { pub component: ComponentId, pub geom: BodyGeom, pub naming: Option<Arc<BodyNaming>> }
#[derive(Clone)]
pub enum BodyGeom {                       // all payloads Send + Sync (§3)
    Solid(amg_brep::Shape),              // stub until BREP-02
    Mesh(Arc<amg_mesh::IndexedMesh>),     // stub until MESH-02
    Implicit(Arc<amg_implicit::SdfGraph>),// stub until IMP-01
    Form(Arc<amg_subd::SubdMesh>),        // stub until FRM-01
}
pub enum ConstructionGeom { Plane(Plane), Axis(Line3), Point(Point3) }
pub struct NodeEval { pub health: Health, pub key: Option<CacheKey>, pub duration: Duration }
#[derive(Clone, Debug, PartialEq)]
pub enum Health { Ok, Warning(Vec<Diagnostic>), Error(Diagnostic), Suppressed, RolledBack }
#[derive(Clone, Debug, PartialEq)]
pub struct Diagnostic { pub message: String, pub cause: Option<ErrorCause>, pub entities: Vec<EntityRef> }
#[derive(Clone, Debug, PartialEq)]
pub enum ErrorCause { Upstream(FeatureId), Resolve(ResolveError), Expr(ParamId), UnknownKind(String) }

// ===== features (amg-doc::feature) =====
pub trait Feature: Send + Sync + std::fmt::Debug + std::any::Any {
    /// Registered kind, e.g. "solid.extrude". Stable forever; used in files.
    fn kind(&self) -> &'static str;
    /// Everything this feature reads (excluding params; those come from `exprs`).
    fn inputs(&self) -> Vec<Dependency>;
    fn exprs(&self) -> Vec<&Expr>;
    fn exprs_mut(&mut self) -> Vec<&mut Expr>;
    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError>;
    /// Provided by `register_feature!` (serde_json of Self).
    fn to_json(&self) -> serde_json::Value;
    fn as_any(&self) -> &dyn std::any::Any;
}
#[derive(Clone, Debug)]
pub struct FeatureBox(pub Arc<dyn Feature>);       // Serialize/Deserialize via registry
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum Dependency {
    Body(BodyId), Entity(EntityRef), Sketch(SketchId), Construction(ConstructionId),
    Occurrence(OccurrenceId), Feature(FeatureId),
}
#[derive(Default)]
pub struct FeatureOutput {
    pub bodies: Vec<BodyChange>,
    pub sketch: Option<Arc<SketchGeom>>,
    pub construction: Option<ConstructionGeom>,
    pub warnings: Vec<Diagnostic>,
}
pub struct BodyChange { pub id: BodyId, pub change: BodyChangeKind }
pub enum BodyChangeKind { Created { geom: BodyGeom, naming: Option<BodyNaming> },
                          Modified { geom: BodyGeom, naming: Option<BodyNaming> }, Deleted }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ComputeMode { Full, Preview }

pub struct RecomputeCtx<'a> { /* snapshot, partial evaluated state, cache, job ctx */ _p: PhantomData<&'a ()> }
impl<'a> RecomputeCtx<'a> {
    pub fn feature_id(&self) -> FeatureId;
    pub fn context(&self) -> ComponentId;
    pub fn mode(&self) -> ComputeMode;
    pub fn tolerance(&self) -> &Tolerance;
    pub fn eval(&self, e: &Expr) -> Result<Quantity, FeatureError>;
    pub fn eval_length(&self, e: &Expr) -> Result<f64, FeatureError>;   // mm
    pub fn eval_angle(&self, e: &Expr) -> Result<f64, FeatureError>;    // rad
    pub fn eval_scalar(&self, e: &Expr) -> Result<f64, FeatureError>;
    pub fn body(&self, id: BodyId) -> Result<&BodyGeom, FeatureError>;  // state before this feature
    pub fn naming(&self, id: BodyId) -> Option<&BodyNaming>;
    pub fn sketch(&self, id: SketchId) -> Result<&SketchGeom, FeatureError>;
    pub fn construction(&self, id: ConstructionId) -> Result<&ConstructionGeom, FeatureError>;
    pub fn resolve(&self, r: &TopoRef) -> Result<Resolved, ResolveError>;
    pub fn new_body_id(&self, slot: u32) -> BodyId;                     // BodyId{origin: self, slot}
    pub fn progress(&self, fraction: f32, msg: &str);
    pub fn check_cancelled(&self) -> Result<(), FeatureError>;
}

#[derive(thiserror::Error, Debug, Clone, PartialEq)]
pub enum FeatureError {
    #[error("input {0:?} is not available")] MissingInput(Dependency),
    #[error(transparent)] Resolve(#[from] ResolveError),
    #[error(transparent)] Expr(#[from] ExprError),
    #[error("invalid value for {field}: {msg}")] InvalidValue { field: &'static str, msg: String },
    #[error("geometry kernel failed: {0}")] Kernel(String),
    #[error("cancelled")] Cancelled,
}

// ===== registry =====
pub struct FeatureRegistration {
    pub kind: &'static str,
    pub version: u32,
    pub deserialize: fn(serde_json::Value) -> Result<Arc<dyn Feature>, serde_json::Error>,
    pub migrate: fn(from_version: u32, raw: serde_json::Value) -> Result<serde_json::Value, MigrationError>,
}
inventory::collect!(FeatureRegistration);
/// `register_feature!(Extrude, "solid.extrude", version = 1, migrate = extrude_migrate);`
#[macro_export] macro_rules! register_feature { /* submits FeatureRegistration + impls to_json/as_any helper */ }
pub fn registry() -> &'static FeatureRegistry;   // built once; duplicate kind → panic at startup
impl FeatureRegistry {
    pub fn get(&self, kind: &str) -> Option<&'static FeatureRegistration>;
    pub fn kinds(&self) -> impl Iterator<Item = &'static str> + '_;
}
#[derive(Debug)]
pub struct UnknownFeature { pub kind: String, pub version: u32, pub raw: serde_json::Value }  // impl Feature: always errors

// ===== timeline =====
pub struct Timeline { /* nodes: PVec<FeatureNode>, marker, groups */ }
impl Timeline {
    pub fn len(&self) -> usize;
    pub fn marker(&self) -> usize;
    pub fn nodes(&self) -> impl Iterator<Item = &FeatureNode> + '_;
    pub fn node(&self, id: FeatureId) -> Option<(usize, &FeatureNode)>;
    pub fn group(&self, id: GroupId) -> Option<&Group>;
}
#[derive(Clone, Debug)]
pub struct FeatureNode { pub id: FeatureId, pub feature: FeatureBox, pub suppressed: bool,
                         pub group: Option<GroupId>, pub name: Option<String>, pub context: ComponentId }
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Group { pub name: String, pub collapsed: bool }

// ===== cache =====
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct CacheKey(pub [u8; 32]);                 // blake3
pub struct GeomCache { /* Mutex<LruByBytes<CacheKey, Arc<FeatureOutput>>> */ }
impl GeomCache {
    pub fn new(budget_bytes: u64) -> Arc<Self>;
    pub fn get(&self, k: &CacheKey) -> Option<Arc<FeatureOutput>>;
    pub fn insert(&self, k: CacheKey, v: Arc<FeatureOutput>);
}

// ===== persistent naming (amg-doc::naming) — PROVISIONAL until DOC-11 is accepted =====
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EntityRef {
    Body(BodyId),
    Topo(TopoRef),
    SketchCurve { sketch: SketchId, curve: SketchEntityKey /* stub: S03 SK-01 */ },
    SketchPoint { sketch: SketchId, point: SketchEntityKey },
    Profile { sketch: SketchId, profile: ProfileKey /* stub: S03 SK-06 */ },
    Construction(ConstructionId),
    /// Entity seen through an occurrence path (assemblies, S06 joint origins).
    InContext { path: Vec<OccurrenceId>, target: Box<EntityRef> },
}
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TopoRef { pub body: BodyId, pub kind: TopoKind, pub name: TopoName,
                     pub fingerprint: Option<GeomFingerprint> }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TopoKind { Face, Edge, Vertex, FaceGroup /* mesh bodies */ }
/// Opaque to callers; grammar defined by DOC-11.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct TopoName(Arc<[NameSeg]>);
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum NameSeg {
    Generated { feature: FeatureId, role: Role, from: Option<NameSource> },
    Modified  { feature: FeatureId },
    Split     { index: u32 },
    Merged    { with: Vec<TopoName> },
}
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Role { pub tag: Cow<'static, str>, pub index: u32 }   // e.g. ("Lateral", 0), ("EndCap", 0)
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum NameSource { SketchCurve { sketch: SketchId, curve: SketchEntityKey }, Topo(TopoName) }
/// Hash/Eq use quantised values (tolerance from DOC-11), so refs are hashable.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeomFingerprint { pub class: GeomClass, pub measure: f64 /* area | length | 0 */,
                             pub centroid: Point3, pub axis: Option<Vec3>, pub bbox: Aabb3 }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum GeomClass { Plane, Cylinder, Cone, Sphere, Torus, BSpline, Other, Line, Circle, Ellipse, Curve, Point }
/// Element map of one evaluated body: index = amg-brep in-shape index of that sub-shape.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct BodyNaming { pub faces: Vec<TopoName>, pub edges: Vec<TopoName>, pub vertices: Vec<TopoName> }
pub struct NamingBuilder { /* DOC-06 */ }
impl NamingBuilder {
    pub fn new(feature: FeatureId) -> Self;
    pub fn generated(&mut self, kind: TopoKind, index: u32, role: Role, from: Option<NameSource>) -> &mut Self;
    pub fn from_history(&mut self, input: &BodyNaming, history: &amg_brep::ShapeHistory) -> &mut Self;
    pub fn finish(self, shape: &amg_brep::Shape) -> Result<BodyNaming, NamingError>;
}
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MatchQuality { Exact, Disambiguated, FingerprintOnly }
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Resolved { pub body: BodyId, pub kind: TopoKind, pub index: u32, pub quality: MatchQuality }
#[derive(thiserror::Error, Clone, Debug, PartialEq)]
pub enum ResolveError {
    #[error("body {0:?} does not exist at this point in the timeline")] BodyMissing(BodyId),
    #[error("referenced {kind:?} no longer exists")] NotFound { kind: TopoKind },
    #[error("reference is ambiguous ({candidates} candidates)")] Ambiguous { candidates: u32 },
    #[error("body has no sub-entity naming (implicit/form body)")] Unsupported,
}

// ===== parameters & expressions (amg-doc::param) =====
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Expr { Inline(ExprSrc), Param(ParamId) }
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)] pub struct ExprSrc(pub String);
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub struct Dim(pub [i8; 8]);                       // L M T Θ I N J A
impl Dim { pub const NONE: Dim; pub const LENGTH: Dim; pub const ANGLE: Dim; /* … */ }
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Quantity { pub value: f64 /* internal: mm, kg, s, K, A, mol, cd, rad */, pub dim: Dim }
pub struct ParamTable { /* PMap<ParamId, Arc<Param>>, name index, eval cache */ }
impl ParamTable {
    pub fn get(&self, id: ParamId) -> Option<&Param>;
    pub fn by_name(&self, name: &str) -> Option<ParamId>;
    pub fn iter(&self) -> impl Iterator<Item = (ParamId, &Param)> + '_;
    pub fn value(&self, id: ParamId) -> Result<Quantity, ExprError>;
    /// Parse + evaluate without committing (dialog validation / autocomplete preview).
    pub fn try_eval(&self, src: &str, expected: Option<Dim>) -> Result<Quantity, ExprError>;
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Param { pub name: String, pub expr: ExprSrc, pub kind: ParamKind, pub unit: Option<UnitSpec>,
                   pub comment: String, pub favorite: bool }
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ParamKind { User, Model { owner: FeatureId } }
pub mod expr {
    pub fn parse(src: &str, units: &UnitTable) -> Result<Ast, ExprError>;
    pub fn eval(ast: &Ast, env: &dyn Fn(&str) -> Option<Quantity>, expected: Option<Dim>,
                prefs: &UnitPrefs) -> Result<Quantity, ExprError>;
    pub fn references(ast: &Ast) -> Vec<(String, std::ops::Range<usize>)>; // for rename/autocomplete
}
#[derive(thiserror::Error, Clone, Debug, PartialEq)]
pub enum ExprError {
    #[error("syntax error at {span:?}: {msg}")] Parse { span: std::ops::Range<usize>, msg: String },
    #[error("unknown parameter `{name}`")] UnknownParam { name: String, span: std::ops::Range<usize> },
    #[error("unit mismatch: {left:?} vs {right:?}")] UnitMismatch { left: Dim, right: Dim, span: std::ops::Range<usize> },
    #[error("expected {expected:?}, got {got:?}")] WrongDimension { expected: Dim, got: Dim },
    #[error("{func}: {msg}")] Domain { func: &'static str, msg: String, span: std::ops::Range<usize> },
    #[error("circular reference: {0:?}")] Cycle(Vec<ParamId>),
    #[error("`{0}` is a reserved unit name")] ReservedName(String),
}

// ===== commands & undo (amg-doc::command) =====
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Command {
    AddFeature { at: Option<usize> /* None = at marker */, feature: FeatureBox,
                 context: ComponentId, name: Option<String> },
    EditFeature { id: FeatureId, feature: FeatureBox },
    DeleteFeatures { ids: Vec<FeatureId> },
    MoveFeature { id: FeatureId, to: usize },
    SetMarker { index: usize },
    SetSuppressed { ids: Vec<FeatureId>, suppressed: bool },
    RenameFeature { id: FeatureId, name: Option<String> },
    GroupFeatures { ids: Vec<FeatureId>, name: String },
    Ungroup { group: GroupId },
    AddParam { name: String, expr: String, unit: Option<UnitSpec>, comment: String },
    SetParamExpr { id: ParamId, expr: String },
    RenameParam { id: ParamId, name: String },
    SetParamMeta { id: ParamId, comment: Option<String>, favorite: Option<bool> },
    DeleteParam { id: ParamId },
    SetBodyProps { body: BodyId, props: BodyProps },
    NewComponent { parent: ComponentId, name: String, transform: Transform },   // S06 ASM-01 extends
    SetOccurrenceTransform { occ: OccurrenceId, transform: Transform },
    SetOccurrenceVisible { occ: OccurrenceId, visible: bool },
    SetDesignMode { mode: DesignMode },
    SetExt { key: ExtKey, record: Option<ExtRecord> },
    SetActiveConfig { config: Option<ConfigId> },                             // DOC-10
}
pub struct CommandOutcome { pub created: Vec<CreatedId>, pub revision: Revision }
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CreatedId { Feature(FeatureId), Param(ParamId), Component(ComponentId), Occurrence(OccurrenceId), Group(GroupId) }
#[derive(thiserror::Error, Debug, Clone, PartialEq)]
pub enum CommandError {
    #[error("no such {what} {id}")] NotFound { what: &'static str, id: u64 },
    #[error("moving would place {feature:?} before its input {input:?}")] WouldBreakDependency { feature: FeatureId, input: FeatureId },
    #[error(transparent)] Expr(#[from] ExprError),
    #[error("name `{0}` already in use")] NameTaken(String),
    #[error("not allowed in {0:?} mode")] WrongMode(DesignMode),
    #[error("{0}")] Invalid(String),
}

// ===== document (amg-doc::document) =====
pub struct Document { /* §1 */ }
impl Document {
    pub fn new(name: &str) -> Self;
    pub fn snapshot(&self) -> DesignSnapshot;                       // O(1)
    pub fn design(&self) -> &Design;
    pub fn evaluated(&self) -> &Arc<Evaluated>;
    pub fn apply(&mut self, cmd: Command) -> Result<CommandOutcome, CommandError>;   // one undo entry
    pub fn transaction(&mut self, label: &str) -> Transaction<'_>;
    pub fn undo(&mut self) -> bool;
    pub fn redo(&mut self) -> bool;
    pub fn undo_label(&self) -> Option<&str>;
    pub fn redo_label(&self) -> Option<&str>;
    /// Call once per UI frame. Never blocks: starts/cancels/collects recompute jobs.
    pub fn poll(&mut self, jobs: &JobSystem);
    /// Headless only (CLI, Python, tests). Panics in debug if called on the UI thread.
    pub fn recompute_blocking(&mut self, jobs: &JobSystem) -> Result<(), FeatureError>;
    pub fn recompute_status(&self) -> RecomputeStatus;              // Idle | Running{progress} | Stale
    pub fn preview(&self, jobs: &JobSystem, cmd: &Command) -> JobHandle<Arc<Evaluated>>;
    pub fn drain_events(&mut self) -> Vec<DocEvent>;
    pub fn is_dirty(&self) -> bool;
}
pub struct Transaction<'d> { /* holds &mut Document; commit on drop-less explicit call */ _d: PhantomData<&'d mut Document> }
impl Transaction<'_> {
    pub fn apply(&mut self, cmd: Command) -> Result<CommandOutcome, CommandError>;
    pub fn commit(self) -> Revision;
    pub fn rollback(self);
}

// ===== events (amg-doc::event) =====
#[derive(Clone, Debug, PartialEq)]
pub enum DocEvent {
    BodyAdded(BodyId), BodyRemoved(BodyId), BodyGeometryChanged(BodyId), BodyPropsChanged(BodyId),
    ComponentTreeChanged, OccurrenceTransformChanged(OccurrenceId), OccurrenceVisibilityChanged(OccurrenceId),
    SketchChanged(SketchId), ConstructionChanged(ConstructionId),
    TimelineChanged, HealthChanged(FeatureId), ParamsChanged,
    ExtChanged(ExtKey),
    RecomputeStarted(Revision), RecomputeFinished { revision: Revision, failed: u32 },
    DocumentReplaced,                    // open/new/recover: consumers rebuild everything
}

// ===== file I/O (amg-doc::io) =====
pub const FORMAT_VERSION: u32 = 1;
pub struct SaveOptions { pub include_cache: bool, pub keep_backup: bool, pub thumbnail: Option<Vec<u8>> }
pub fn save(path: &Path, snap: &DesignSnapshot, eval: &Evaluated, cache: &GeomCache,
            opts: &SaveOptions) -> Result<(), FormatError>;               // run inside a Job
pub fn load(path: &Path) -> Result<LoadedDocument, FormatError>;        // run inside a Job
pub struct LoadedDocument { pub design: Design, pub cache_seed: CacheSeed, pub warnings: Vec<String> }
#[derive(thiserror::Error, Debug)]
pub enum FormatError {
    #[error("not an Amalgam file")] NotAmalgam,
    #[error("file format {file} is newer than supported {supported}; update Amalgam")] TooNew { file: u32, supported: u32 },
    #[error("file is corrupt ({entry})")] Corrupt { entry: String },
    #[error("invalid design data at {path}: {msg}")] Json { path: String, msg: String },
    #[error("migration v{from}→v{to} failed: {msg}")] Migration { from: u32, to: u32, msg: String },
    #[error(transparent)] Io(#[from] std::io::Error),
    #[error(transparent)] Zip(#[from] zip::result::ZipError),
}
#[derive(thiserror::Error, Debug, Clone, PartialEq)]
#[error("migration failed: {0}")] pub struct MigrationError(pub String);

// ===== recovery (amg-doc::recovery) =====
pub struct RecoveryConfig { pub dir: PathBuf, pub snapshot_interval: Duration }
pub struct RecoverySession { /* lock + journal writer */ }
impl RecoverySession {
    pub fn start(cfg: &RecoveryConfig, doc_uuid: Uuid) -> std::io::Result<Self>;
    pub fn on_commit(&self, commands: &[Command]);                 // non-blocking (writer thread)
    pub fn flush(&self);                                            // used by the FND-07 panic hook
}
pub fn find_recoverable(cfg: &RecoveryConfig) -> Vec<RecoveryCandidate>;
pub fn recover(c: &RecoveryCandidate) -> Result<Document, FormatError>;

// ===== extension records =====
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExtRecord { pub kind: String, pub version: u32, pub data: serde_json::Value, pub blobs: Vec<BlobRef> }
pub struct ExtRegistration { pub kind: &'static str, pub version: u32,
    pub migrate: fn(u32, serde_json::Value) -> Result<serde_json::Value, MigrationError> }
inventory::collect!(ExtRegistration);
```

**Needed from other specs (requests, not edits):**
- **S00 / FND-03:**
  - The unit table exposes per unit the `Dim` exponents (8-slot, incl. angle), the factor to the
    internal coherent system, and an `affine` flag.
  - The unit names list is iterable, because they are reserved identifiers in expressions.
  - Ids: `amg-core` slotmap keys stay for domain crates. `amg-doc` uses its own typed `u64` ids
    (§1).
- **S00 / FND-04:** `JobHandle::cancel()`, `JobHandle::try_take()` (non-blocking), and
  `JobCtx::is_cancelled()` reachable from `RecomputeCtx`.
- **S00 / FND-07:** the panic hook calls a registered `fn()` hook (recovery flush).
- **S02:** `Shape: Send + Sync` (internal serialisation of mutating OCCT calls). Stable in-shape
  sub-shape indices for an unmodified `Shape`. `ShapeHistory` per op (BREP-05). Per-sub-shape
  `GeomClass`, area/length, centroid and axis queries for fingerprints.
  `amg_brep::kernel_version() -> &'static str`.
- **S03:** `SketchEntityKey` and `ProfileKey` are stable across sketch edits (S03 already
  promises stable profile ids) and are `Ord + Hash + Serialize`.
- **S07:** `IndexedMesh` binary (de)serialization used for `bodies/` and `cache/` (fast,
  versioned).

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| DOC-01 | Core data model: typed ids, `Design`/`DesignMeta`/`Component`/`Occurrence`/`BodyProps`/`ExtRecord`, `PMap`/`PVec` aliases over `imbl`, `DesignSnapshot`, `amg-doc::stub` geometry placeholders, `Evaluated`/`BodyGeom` skeleton; criterion bench imbl vs rpds vs `Arc<Vec<Arc<T>>>` | FND-03 | P0 | Done | Unit tests create a design with ≥ 3 components, occurrences and body props. Bench at 10⁴ entries shows snapshot clone < 1 µs and one mutation after a clone < 5 µs. Results are recorded in the Refinement log, and the alias is switched if `imbl` misses either number. `serde_json` round-trip of a `Design` is byte-identical twice in a row (determinism). |
| DOC-02 | `Feature` trait, `FeatureBox` serde via registry, `register_feature!` + `inventory` registry, `UnknownFeature`, `Timeline` (marker, suppression, groups, ordering rule), `link()` convention; test-only features (`test.counter`, `test.box`, `test.add_mesh`) | DOC-01 | P0 | Review | 1. Timeline of test features serializes and deserializes to equal JSON. 2. An unknown kind loads as `UnknownFeature` and re-serializes byte-identical. 3. A duplicate kind fails at startup. 4. A registry test asserts expected kinds are linked. 5. `MoveFeature` before a dependency is rejected. |
| DOC-03 | Recompute engine core (synchronous): producer map, `Dependency` resolution, blake3 cache keys with `ENGINE_SALT`, `GeomCache` (LRU by bytes), `RecomputeCtx`, health propagation (error = no effect, upstream causes), suppression/marker handling | DOC-02, DOC-04 | P0 | Todo | Property test on random DAGs of `test.counter` features: after editing feature *k*, exactly the nodes transitively depending on *k* run (counter check); all others hit the cache. A failing feature leaves its input bodies unmodified and gives dependants `ErrorCause::Upstream`. Keys are identical across two process runs. Engine overhead < 5 ms for 200 cached features (criterion). |
| DOC-04 | Parameters & expressions: lexer + Pratt parser per §5 grammar, `Dim`/`Quantity`, unit literals via amg-core unit table, functions/constants, bare-number default-unit rule, `ParamTable`, model-param creation from `exprs_mut`, cycle detection, rename rewriting via spans, formatting | FND-10 | P0 | Review | Test suite of ≥ 150 cases covers: every grammar rule, every function, `1/4 in`, `N/mm^2`, unit mismatch with correct spans, cycle (`a=b`, `b=a`) rejected with path, rename updates all referencing sources, reserved unit names rejected, round-trip parse→format→parse. Eval of 1 000 chained params < 2 ms. |
| DOC-05 | Commands + snapshot undo/redo + transactions: full `Command` enum handlers (except DOC-10/ASM extensions), `CommandError`, `UndoStack` with depth limit, `Transaction` commit/rollback, model-param conversion on feature delete | DOC-02, DOC-04 | P0 | Todo | Property test (proptest): random sequence of ≤ 200 valid commands → undo all → design equals the initial one (`==` on serialized JSON); redo all → equals the final one. Transaction rollback restores `before`. `apply` + undo each < 1 ms on a 200-feature design. Commands round-trip through serde. |
| DOC-12 | Recompute as a job: `Document::poll` (spawn, cancel-and-supersede, stale-result drop, atomic `Evaluated` swap), progress, `recompute_blocking`, preview recompute (`ComputeMode::Preview`), UI-thread debug assertion | DOC-03, FND-04 | P0 | Todo | Test: 50 rapid `SetParamExpr` commands during a slow (sleeping) recompute → only the last revision is swapped in, cancelled work leaves finished features cached, `poll` never blocks > 1 ms (measured). Preview results never enter the persistent cache or undo stack. |
| DOC-09 | Change events: `DocEvent` from authored diff (`OrdMap::diff`) and evaluated swap (`Arc::ptr_eq`), `drain_events` | DOC-12 | P0 | Todo | Scripted tests assert exact event sets for each command type and for undo/redo. Editing one feature in a 3-body design emits `BodyGeometryChanged` only for the affected body. A props-only change emits no geometry event. |
| DOC-07 | `.amg` container: manifest + `design.json` + `bodies/` + `ext/`, `FORMAT_VERSION`, document and per-feature migration chains, `serde_path_to_error` diagnostics, safe save (tmp + fsync + rename + `.bak`), save/load as jobs | DOC-05 | P0 | Todo | 1. Save→load→save gives byte-identical `design.json`. 2. `TooNew`, truncated zip, flipped byte and bad JSON each give the specific `FormatError` (with path for JSON). 3. A v0→v1 fixture migration test exists. 4. Killing the process during save (test hook) leaves the original file intact. |
| DOC-14 | Geometry cache persistence + autosave/recovery: `cache/index.json` + entries with salt check, lazy cache seeding on open, "save compact", journal writer, periodic recovery snapshot job, lock via `File::try_lock`, `find_recoverable`/`recover`, FND-07 hook | DOC-07, DOC-12, FND-07 | P0 | Todo | 1. Opening a saved 50-feature test design with cache performs 0 feature computes (counter) and with a changed salt recomputes all. 2. Kill test: commit 20 commands, abort the process, `recover` yields a design equal to the pre-crash one. 3. A held lock is not offered for recovery. |
| DOC-11 | Persistent naming design note `docs/notes/persistent-naming.md` | DOC-01 | P0 | Review | The note exists with these sections: (a) prior-art study covering all 7 sources in §4 with what we take/reject from each; (b) final `TopoName`/`NameSeg` grammar and `GeomFingerprint` fields + quantisation tolerances (may revise the provisional Interfaces); (c) per-feature naming rules table for every P0/P1 feature in S04 (extrude, revolve, primitives, fillet, chamfer, shell, draft, combine/boolean, split, pattern, mirror, move, sweep, loft, hole) plus mesh face groups and imported bodies; (d) split/merge disambiguation and resolver thresholds; (e) failure UX (warning vs error, re-pick flow); (f) golden corpus of ≥ 20 cases, each listed as base model + edit script (serialized `Command` JSON) + expected referenced entity, including ≥ 5 cases known to break naive index naming. The owner has marked the note `Accepted` in its header. |
| DOC-06 | Persistent naming framework: final `TopoName`/`EntityRef`/`BodyNaming` types per DOC-11, `NamingBuilder` (incl. `from_history` over `amg_brep::ShapeHistory`), element-map storage in `Evaluated` + `.amg` cache, 4-stage resolver with `MatchQuality`, fingerprint computation | DOC-03, DOC-11, BREP-05, BREP-06 | P0 | Todo | 1. Resolver unit tests on synthetic histories cover exact / split-disambiguated / fingerprint-only / not-found / ambiguous, plus unmerged, widened, class-changed (no ∞ arithmetic) and tombstone lookup by lineage. 2. 10 golden cases from the DOC-11 corpus (TNP-01/02/03/05/06/07/08/09/10/12) run on lane-E test features built directly on `BrepKernel` (box, prism from polygon, fillet, boolean cut, probe) and resolve to the intended entity (asserted by fingerprint); the other DOC-06-phase cases (TNP-04/11/24..28) meet their recorded `expect`. 3. Zero silent wrong resolutions (DOC-11 note §11.2 definition) over all DOC-06-phase cases. 4. Naming survives save/load (TNP-28). 5. The note §9 benches (`naming/finish-1000`, `naming/resolve`) run and their results are recorded in the note's §13. |
| DOC-13 | Golden TNP corpus on real features: run the full DOC-11 corpus (≥ 20 cases) through S04 features in `tests/golden/tnp/`, CI-gated | DOC-06, SOL-02, SOL-03, SOL-05, SOL-07, SOL-08 | P0 | Todo | ≥ 90 % of the core corpus cases (DOC-11 note §11.4) meet their recorded `expect` (intended entity and quality, or the expected warning/error); the remainder fail with a feature error or warning; **0 silent wrong resolutions** (note §11.2 definition: a quiet `Ok` on a non-intended entity, or a quiet `Ok` where an error is expected). The harness measures the naive index baseline on every case and ≥ 5 cases are measured to break it (predictions in the corpus are corrected where the measurement differs). The held-out fuzz set (note §11.6) has 0 silent wrong resolutions. Runs in CI under 60 s (the fuzz set may run nightly if it doesn't fit). |

| DOC-08 | Direct mode + bake: `SetDesignMode` both ways, `baked` bodies in `bodies/`, `BaseFeature` for Direct→History | DOC-03, DOC-07 | P1 | Todo | On 3 test designs (solid, mesh, mixed), History→Direct→History keeps volume/area/bbox within 1e-9 relative and identical topology counts. `BodyId`s and body props are preserved. Undo restores the full timeline. Save/load in Direct mode round-trips. |
| DOC-10 | Configurations: `ConfigTable`, param/suppression overrides, `SetActiveConfig`, per-config cache keys, persistence | DOC-05, DOC-12 | P2 | Todo | Design with 3 configurations: switching recomputes to the expected per-config volumes; switching back is a full cache hit (0 computes); configurations survive save/load. |

Phase notes (the roadmap is not edited here; see Refinement log): DOC-12 and DOC-14 are new
and belong in Phase 1 lane E, since DOC-09 (Phase 1) now depends on DOC-12. DOC-13 belongs in
Phase 2 lane E (its SOL deps are Phase 2).

## Testing
- **Unit tests** per module (ids, maps, registry, timeline, expressions, commands, events, io).
- **Property tests** (`proptest`):
  - Command sequences, undo-all and redo-all (DOC-05).
  - Random DAG edits with exact recompute sets (DOC-03).
  - Parse/format/parse round-trip of random expressions (DOC-04).
  - Save/load of random designs (DOC-07).
- **Determinism tests:**
  - The same design serializes to identical bytes.
  - Cache keys are identical across two process runs and across save/load.
- **Golden tests:** TNP corpus (DOC-06 subset with lane-E test features; DOC-13 full corpus on
  S04 features) in `tests/golden/tnp/`, each case = base command script + edit script +
  expected entity fingerprint.
- **Format fixtures:** `tests/fixtures/format/v<N>.amg` for every format version, never edited
  after commit. Corrupt-file fixtures (truncated, bad CRC, bad JSON, too-new version).
- **Crash tests:** a child process committing commands is killed; recovery must restore the
  state (DOC-14). Killing during save must leave the original file intact (DOC-07).
- **Benchmarks** (criterion, wired into FND-08): snapshot clone, single mutation, 200-feature
  cached recompute overhead, 1 000-param evaluation, apply+undo, save/load of a 200-feature
  design without geometry.

## Performance targets
| Operation | Target | Vision link |
|---|---|---|
| `Document::snapshot()` | < 1 µs (O(1)); design spec required < 1 ms | recompute on snapshot |
| Single mutation after snapshot (10⁴ entities) | < 5 µs | UI ≤ 16 ms |
| `apply(cmd)` incl. param re-eval + event diff, 200-feature design | < 1 ms | UI ≤ 16 ms |
| Undo / redo (authored part) | < 1 ms; following recompute fully cached | UI ≤ 16 ms |
| Recompute engine overhead, 200 features, all cached | < 5 ms | 50-feature edit < 500 ms |
| Cache-key computation per feature | < 20 µs | same |
| Evaluate 1 000 chained parameters | < 2 ms | UI ≤ 16 ms |
| `Document::poll()` | < 1 ms, never blocks | UI ≤ 16 ms |
| Save/load `design.json` of a 200-feature design (no geometry) | < 50 ms (on a job) | cold start < 2 s |
| Open a 50-feature part with full cache | 0 feature computes; < 300 ms to first evaluated state excluding mesh upload | cold start < 2 s |

## Open questions
1. ~~`im`/`rpds` persistent collections vs `Arc` copy-on-write per entity?~~ **Resolved
   2026-10-19:** use `imbl` (maintained `im` fork, MPL-2.0) behind crate-private aliases. `rpds`
   is the fallback and per-entity `Arc` COW is the bench baseline. DOC-01's bench confirms or
   switches (§1).
2. **(owner, answered)** Should `.amg` files embed geometry caches by default? They make files larger
   (cached B-rep and meshes, roughly 2–10× the authored size) but open instantly. The
   alternative is compact files plus recompute on open. **Proposed default:** embed caches, with
   a "Save compact" option.
   **Answered 2026-10-20:** embed caches by default, plus a "Save compact" option (recommended default).
3. **(owner, answered)** Should a bare number in an expression take the default unit, as Fusion does
   (`d1 + 5` → `d1 + 5 mm`), or should it be a unit error (strict)? **Proposed default:**
   lenient like Fusion, with a strict-mode preference.
   **Answered 2026-10-20:** lenient like Fusion, with a strict-mode preference (recommended default).
4. **(owner, answered)** Should undo history be saved in the `.amg` file, so it survives closing? Fusion
   doesn't. **Proposed default:** no, and only the crash journal persists commands.
   **Answered 2026-10-20:** no; only the crash journal persists commands (recommended default).
5. How much of TNP can be solved generically vs per feature? Answered by DOC-11 before DOC-06
   starts. The framework (element map + resolver) is generic; the naming rules are per feature.
6. Should independent timeline branches recompute in parallel? Deferred until DOC-03 profiling
   on real S04 features. It needs S02's per-shape-graph locking to be in place.

## Refinement log
**2026-10-19 — refinement (lane E agent).**
- **Researched:**
  - **im / imbl / rpds status.** `im-rc` repository archived 2026-05-03; RUSTSEC-2026-0250
    (issued 2026-08-10) recommends `imbl`; imbl 7.0.1, MSRV 1.85, OrdMap/Vector fixes and
    HashMap/OrdMap speedups over im. rpds 1.2.1 (2026-05-15), `new_sync()` Arc variants.
  - **`inventory`.** Platforms: Linux/macOS/Windows/…, Apache-2.0/MIT. Plugins register only if
    their code is linked, hence the `link()` convention.
  - **Feature serialization: `typetag` vs registry.** A registry was chosen for per-kind
    versioning and preserving unknown kinds.
  - **`evalexpr` licence.** AGPL-3.0 at 13.x (MIT earlier). Led to a hand-written parser.
  - **FreeCAD 1.0 TNP.** realthunder's element-map mitigation, merged in 1.0; the core is
    agnostic to how names are generated, which is the model for the DOC-06 framework.
  - **Build-system theory** for cache keys (constructive traces, early cutoff).
  - **Persistent-naming literature** list for DOC-11.
- **Decided:**
  - Authored/evaluated state split.
  - Monotonic typed `u64` ids persisted in the file, and derived `BodyId = (feature, slot)`.
  - `imbl` persistent maps (open question 1).
  - `inventory` registry with explicit registrations and `UnknownFeature` (no `typetag`).
  - blake3 constructive-trace cache keys with an engine salt.
  - Fusion-style failure semantics (a failed feature has no effect).
  - Recompute job with cancel-and-supersede.
  - Hand-written Pratt parser with 8-slot dimensional analysis and reserved unit identifiers;
    model parameters created from feature `Expr`s (Fusion `d<n>`).
  - Snapshot-based undo (not inverse commands) and transactions.
  - Pull-based `DocEvent`s from map diffs and `Arc::ptr_eq`.
  - Zip format with a manifest, separate document/feature migration chains, safe save.
  - Command-journal crash recovery.
  - Extension records for studies/print setups.
  - Provisional `EntityRef`/`TopoRef`/`TopoName`/`BodyNaming`/`Resolved` types, so S03/S04/S06/S13
    can proceed.
- **Changed:**
  - Status set to Refined; Scope, Interfaces, Testing and Performance targets were rewritten.
  - **Split:** DOC-03 → DOC-03 (engine core) + **DOC-12** (job integration). DOC-07 → DOC-07
    (container/versioning) + **DOC-14** (cache persistence, autosave, recovery). DOC-06 →
    DOC-06 (framework, tested on lane-E test features) + **DOC-13** (full corpus on S04
    features), so DOC-06 doesn't depend on lane-A features.
  - **Deps changed:**
    - DOC-03 +DOC-04, −FND-04 (moved to DOC-12).
    - DOC-05 +DOC-04.
    - DOC-06 +DOC-03.
    - DOC-08 +DOC-07.
    - DOC-09 DOC-03 → DOC-12.
    - DOC-10 +DOC-05, +DOC-12, −DOC-04 (transitive).
  - Every WP now has a measurable Done-when.
- **Roadmap phase-order issues (not edited here):**
  - DOC-12 and DOC-14 are not in the roadmap. DOC-09 (Phase 1) now depends on DOC-12, so
    DOC-12 must be in Phase 1 lane E. DOC-14 should also be Phase 1 ("never lose work").
  - DOC-13 should be Phase 2 lane E; it can only start after SOL-02/03/05/07/08.
  - No existing WP moved earlier than its deps: DOC-03's new dep DOC-04 is Phase 1, DOC-06's new
    dep DOC-03 is Phase 1, and DOC-08's new dep DOC-07 is Phase 1.
- **Dependencies recommended for `THIRD_PARTY.md`:**
  - `imbl` (MPL-2.0, weak copyleft; replaces the "im or rpds" row; `rpds` MPL-2.0 is the
    fallback).
  - `blake3` (CC0-1.0 OR Apache-2.0 OR Apache-2.0 WITH LLVM-exception) — cache keys and the
    manifest hashes.
  - `serde_path_to_error` (MIT/Apache-2.0) — JSON error paths.
  - Already listed: `inventory`, `serde`, `serde_json`, `zip`, `uuid`, `thiserror`.
  - `typetag` is **not** needed.
- **Cross-spec interface requests:** S00 (unit dims/factors, `JobHandle::cancel/try_take`, panic
  hook), S02 (`Shape: Send + Sync`, stable indices, history, fingerprint queries,
  `kernel_version`), S03 (stable `SketchEntityKey`/`ProfileKey`), S07 (mesh binary format).
  See the Interfaces section, "Needed from other specs".
- **Architecture note:** `01-architecture.md` §Core data model shows geometry inside `Body` and
  `Timeline: Vec<FeatureNode>` with health in nodes. This spec moves geometry and health to the
  evaluated state and derives `BodyId`. That is a change to the core data model, so an ADR has
  been proposed to the coordinator (not added to 02-decisions.md).

**2026-10-20 — DOC-01 implementation (lane E).**
- **Benchmark** (`crates/amg-doc/benches/snapshot.rs`, criterion 0.8.2, release, Windows 11 /
  MSVC, machine shared with parallel builds so treat as upper bounds; median of
  `--warm-up-time 1 --measurement-time 3`). Value type `Arc<BodyProps>` keyed by `BodyId`.

  | Operation | n | imbl `OrdMap` | rpds `RedBlackTreeMapSync` | `Arc<Vec<Arc<T>>>` |
  |---|---|---|---|---|
  | clone | 10³ / 10⁴ / 10⁵ | 4.4 ns / **4.4 ns** / 6.6 ns | 4.1 / 4.5 / 4.6 ns | 4.1 / 4.4 / 6.4 ns |
  | 1 mutation after clone | 10³ / 10⁴ / 10⁵ | 0.66 µs / **0.61 µs** / 1.0 µs | 0.56 / 1.07 / 1.60 µs | 5.8 µs / 59 µs / 1.85 ms |
  | 100 mutations after clone | 10³ / 10⁴ / 10⁵ | 26 / 60 / 98 µs | 48 / 100 / 151 µs | 6.0 / 70 µs / 4.2 ms |
  | ordered iteration | 10³ / 10⁴ / 10⁵ | 9.5 µs / 112 µs / 5.0 ms | 15 µs / 169 µs / 28 ms | 1.6 / 22 / 244 µs |
  | diff after 1 mutation | 10³ / 10⁴ / 10⁵ | 1.6 / 1.0 / 2.7 µs (`OrdMap::diff`) | 36 µs / 230 µs / 3.6 ms (no diff; full ordered walk) | 0.5 / 9.8 / 82 µs (full walk) |
  | `Design` clone (`snapshot`) | 10³ / 10⁴ / 10⁵ body props | 191 / **143** / 189 ns | — | — |
  | `Design::set_body_props` after clone | 10³ / 10⁴ / 10⁵ | 1.13 / **1.00** / 0.99 µs | — | — |

  At 10⁴ entries imbl meets both targets with margin (clone 4.4 ns < 1 µs; one mutation after a
  clone 0.61 µs < 5 µs; a whole `Design` clone, which also clones the meta strings, 143 ns).
  **The `PMap` alias stays on `imbl`.** rpds is close on single mutations but has no subtree-
  skipping diff (DOC-09 relies on it) and iterates 2–6× slower. Per-entity `Arc` COW fails the
  single-mutation target at 10⁴ (59 µs) as predicted. Its iteration is fastest, which doesn't
  matter at our sizes.
- **Licence correction:** `rpds` 1.x is MIT (not MPL-2.0); `imbl` is `MPL-2.0+` on crates.io.
- **`design.json` encoding:** persistent maps serialize as arrays of `[key, value]` pairs in key
  order, since most keys are structs (`BodyId`, `ExtKey`) and JSON object keys must be strings.
  Duplicate keys, unknown fields and broken invariants (root missing, dangling occurrence
  references, a child list that disagrees with `Occurrence::parent`, component cycles, ids
  ≥ `next_id`) are load errors. Save → load → save is byte-identical (unit test).
- **Review fixes (same day).** An independent review found that the public edit API could
  produce designs that save but don't load. The rule is now: **every edit either succeeds and
  leaves a design that saves and loads, or fails and changes nothing**, and `Design::validate()`
  (run on load, public) checks the same invariants. The unit tests save and reload after every
  edit. In detail:
  - Body ids (`set_body_props`, `set_baked`) need an allocated origin (`1..next_id`) that isn't a
    component, occurrence or extension id. Once DOC-02 lands, the origin must also be a timeline
    feature (TODO in code).
  - Non-finite values are rejected: transforms must be finite, and opacity must be finite and in
    `0..=1`. `validate()` also checks finiteness and that the tolerance is positive.
    `ExtRecord.data` can't hold non-finite numbers (`serde_json::Number` refuses them).
  - Ids: allocation is checked and bounded by `ID_LIMIT = 2^53`, so ids stay exact in JSON tools
    that read numbers as f64; an exhausted id space gives `ModelError::IdsExhausted`. On load,
    `next_id` must be in `1..=2^53`, and every id has exactly one owner across components,
    occurrences, extensions and body-origin features.
  - Extensions: `ExtRecord.kind` must equal `ExtKey.kind`, an id can't be reused under another
    kind, and `set_ext` rejects ids owned by other entity types.
  - `Component::joints` must be empty until S06 adds a joint table, so any joint id is dangling.
  - Every component must be reachable from the root.
  - No-op edits (identical props or record, clearing an absent entry, same name) don't bump the
    revision.
  - `baked` holds `BakedBody { blob, geom }` (S12 request, see Interface changes).
- **Wire-format evolution policy.** The wire structs use `deny_unknown_fields`, so a field we
  don't know is an error, never silently dropped. Consequences:
  - Every field added later (timeline, params, materials, …) gets `#[serde(default)]` on the
    wire repr, so older files without it still load.
  - A purely additive, defaulted field doesn't bump `FORMAT_VERSION`. The older build then
    refuses the newer file with an unknown-field error rather than misreading it.
  - DOC-07 decides whether to bump `FORMAT_VERSION` for each such field anyway, so old builds
    report `TooNew` instead. Any rename, removal or change of meaning bumps the version and adds
    a migration step with a fixture.
  - `FORMAT_VERSION` stays 1 until DOC-07 freezes the first format.
- Bench after the fixes: `Design::set_body_props` after a clone is 0.83 µs at 10⁴ entries
  (including validation). `Design` clone is 99 ns.

**2026-10-20 — DOC-04 implementation (lane E).**
- Lexer + Pratt parser (`amg-doc::expr`), dimension-checked evaluation and `ParamTable` landed
  with >= 190 table cases, error/span/printing tables and three proptests (print/parse
  round-trip, fuzz no-panic, value format round-trip). Deviations are listed under Interface
  changes (2026-10-20, DOC-04).
- Cycle detection returns the first cycle the DFS reaches, which need not be the longest: with
  `c = b + a`, `e = c - 1 mm`, setting `a = e / 2` reports `[a, e, c, a]`. A self reference
  (`a = a + 1`) is reported as `Cycle([a, a])`, because the name is registered before its
  expression is resolved.
- Bench `benches/params.rs` (1 000 chained params): editing the head parameter re-evaluates all 999 dependents in 0.61 ms (including the `Design` clone and validation; target < 2 ms). A full load of the 1 000-parameter design from JSON takes 4.2 ms, which includes serde parsing of the whole file, parsing 1 000 expressions and evaluating them; no budget applies to it.

**Sources:**
- RustSec RUSTSEC-2026-0250 (https://rustsec.org/advisories/RUSTSEC-2026-0250.html)
- imbl docs (https://docs.rs/crate/imbl/latest)
- rpds docs (https://docs.rs/crate/rpds/latest)
- inventory README (https://github.com/dtolnay/inventory)
- evalexpr on docs.rs (https://docs.rs/crate/evalexpr/latest) and the Debian DFSG review of
  13.1.0
- FreeCAD wiki "Topological naming problem" (https://wiki.freecad.org/Topological_naming_problem/en)
- realthunder FreeCAD_assembly3 wiki "Topological Naming"
  (https://github.com/realthunder/FreeCAD_assembly3/wiki/Topological-Naming)
- Ondsel "toponaming fix phase 2" blog
- Mokhov, Mitchell, Peyton Jones, *Build systems à la carte*, ICFP 2018
- Kripac 1997; Marcheix & Pierra 2002; Farjana & Han 2018

## Interface changes

### 2026-10-20: requests from the wave-2 refinements (S02, S05, S10, S11, S12, S16, S17, S20)

Additive requests recorded by the integration pass; the owning lane confirms or adjusts them when it implements the affected WP.

- **From S02 (B-rep kernel).**
  - DOC-06 now also depends on BREP-06 (its Done-when test features include a fillet); applied in
    the WP table.
  - `NamingBuilder::from_history` should take `&[(u16 /*operand*/, &BodyNaming)]`: booleans and
    patterns have several operands, and `amg_brep::InputSub` carries an `operand` field for this.
  - `GeomFingerprint` comes from `BrepKernel::sub_geom_all` (`SubShapeGeom`).
    `amg_brep::geom::GeomClass` mirrors S01's `GeomClass`; DOC-06 adds the `From` impl in `amg-doc`.
  - The `BodyGeom::Solid` interim mutex can go once BREP-02 lands (`Shape: Send + Sync`). Open
    question 6 no longer waits on S02 locking.
  - `GeomCache` byte accounting can use `Shape::estimated_bytes()` / `TessMesh::estimated_bytes()`.
- **From S05 (inspection).**
  - `BodyGeom::Mesh` should hold `amg_mesh::MeshBody` instead of `Arc<IndexedMesh>` (S07 already
    says `MeshBody` is what is stored), so the BVH and analysis are shared per evaluated body
    across measure, section and picking.
  - `UnitPrefs` exposes the display length unit, angle unit and precision.
  - Phase 2: `Evaluated::body_to_world(BodyId, occurrence path)`; optionally
    `EntityRef::Origin { component, which: OriginEntity }` to unify origin references (S03 uses
    `PlaneInput::OriginXY..`, S05 uses `RefInput::Origin` meanwhile).
- **Extension-record ids (S05 and S16, merged).** Add
  `Command::AddExt { kind, record } -> CreatedId::Ext(ExtKey)` so `amg-doc` allocates
  `ExtKey.id` (S16 assumes "lowest unused id of that kind"; document the rule either way), plus
  `Design::exts_of_kind(&self, kind: &str) -> impl Iterator<Item = (&ExtKey, &ExtRecord)>` so
  `amg-features::{am,inspect}` can list print setups and section analyses.
- **From S11 (UI shell).**
  - R1: `naming::topo_ref_at(eval, body, kind, index) -> Option<TopoRef>`.
  - R2: `Evaluated::resolve_for_display(&Design, &EntityRef) -> Result<Resolved, ResolveError>`.
  - R3: `Command::RenameComponent`. R4: `Command::SetDesignMeta { name, default_units }`.
  - R6: a cache-key rule for UI-only data. Moving a sketch dimension label (`label_pos`) changes
    the sketch feature JSON and so every downstream cache key. Proposal: `Feature::key_data()`,
    defaulting to `to_json()`, which S03's sketch feature overrides to drop UI-only fields.
  - R7: `Timeline::valid_move_range(id)`.
  - Minor: `Document::preview` lacks the error type parameter (should return
    `JobHandle<_, FeatureError>`); the `amg_doc::Diagnostic` struct shares its name with the
    `amg_core::Diagnostic` trait (consider renaming the struct).
- **From S12 (import/export); IO-14 is blocked without these, and MESH-16 already assumes baked
  mesh bodies in Phase 1.**
  1. `RecomputeCtx::baked(slot) -> Result<&BodyGeom, FeatureError>`, returning the baked payload
     for `BodyId { origin: self, slot }`.
  2. Add a feature together with its baked payloads in one undo entry: either
     `Command::AddFeature { …, baked: Vec<(u32, BodyGeom)> }` or `Command::SetBaked` inside a
     `Transaction`.
  3. The baked map is persisted in `bodies/` by DOC-07 in Phase 1, not first in DOC-08 (Phase 3).
- **From S17 (materials; MAT-06 implements and flags the PR; typed tables vs extension records is
  QUESTIONS #39, resolved 2026-10-20: typed tables, owner).**
  - `Design` tables `materials: PMap<MaterialId, Arc<Material>>` and
    `appearances: PMap<AppearanceId, Arc<Appearance>>`.
  - Commands `SetMaterial`, `RemoveMaterial` (refused while in use, `CommandError::InUse`),
    `SetAppearance`, `RemoveAppearance`; events `MaterialChanged`, `AppearanceChanged`.
  - Accessors `body_material`, `body_appearance`, `material_users`; helpers `assign_material`,
    `assign_appearance`, `remove_unused_materials` in `amg-doc`.
  - MAT-02 (Phase 3): `BodyProps.face_appearances: Vec<(FaceSelector, AppearanceKey)>` with
    `#[serde(default)]`.
  - `MaterialKey` and `AppearanceKey` are type aliases exported by `amg-materials`, matching the
    names already in `BodyProps`.
- **From S20 (kernel hardening).** Feature health shows a `Warning` when `OpResult::warnings`
  contains `KernelWarning::Strategy` or `FuzzyUsed`. `ENGINE_SALT` already includes
  `kernel_version()`, which now carries `robust::POLICY_VERSION`.

### 2026-10-20 — DOC-01 (core data model, lane E)
Additive unless marked; the shapes in Interfaces otherwise hold.
- **`DesignMeta::uuid` is `amg_core::PersistentId`** (FND-03's UUID v7 newtype, serialized as
  the hyphenated string), not a raw `Uuid`. `DesignMeta` also carries the crate-private
  `next_id` counter (§1); `Design::revision()` is runtime only and not serialized (loads as 0).
- **`JointId`** added to the typed authored ids (Interfaces referenced `JointId /* S06 */`).
- **`Design` read API** additions: `occurrences()`, `explicit_body_props()`, `baked(id)`,
  `baked_bodies()`, `exts()`, `occurrence_paths(component)`, `validate()`. `timeline()` and `params()` arrive with DOC-02 and
  DOC-04 (the fields don't exist yet, so `design.json` grows then; `FORMAT_VERSION` is fixed by
  DOC-07).
- **Low-level edit API on `Design`** (pub, documented as the layer DOC-05 commands build on;
  application code still mutates documents only via `Document::apply`): `Design::new(name)`,
  `Design::with_uuid(name, uuid)`, `add_component(parent, name, transform) ->
  (ComponentId, OccurrenceId)`, `add_occurrence(parent, component, transform)` (rejects cycles),
  `set_body_props(id, props) -> Result<()>` (default props clear the entry),
  `set_baked(id, Option<BakedBody>) -> Result<()>`, `new_ext_key(kind) -> Result<ExtKey>`,
  `set_ext(key, Option<ExtRecord>) -> Result<()>`, `set_name(name)`. Errors are the new
  `ModelError { NotFound, IdInUse, WouldCycle, InvalidValue, IdsExhausted, Inconsistent }`
  (`#[non_exhaustive]`); DOC-05 maps it into `CommandError`. A failed edit changes nothing, and a
  no-op edit doesn't bump the revision. `pub const ID_LIMIT: u64 = 1 << 53`.
- **`Evaluated`** is `Clone + Default`, with `Evaluated::new(revision)`, `nodes()` and
  builder methods for the engine (`set_body`, `remove_body`, `set_sketch`, `set_construction`,
  `set_node`). `BodyGeom::same_as(&other)` compares geometry by pointer (for DOC-09).
- **`BodyGeom::Solid(stub::Shape)`**: the stub `Shape` is a cheap-clone `Send + Sync` handle with
  `ptr_eq`. The "wrap it in a mutex" fallback (§3) is only needed if BREP-02's real `Shape`
  isn't `Sync`.
- **Stubs in `amg-doc::stub`**: `Shape`, `IndexedMesh`, `SdfGraph`, `SubdMesh`, `SketchGeom`
  (no data), `SketchEntityKey(u64)`, `ProfileKey(u64)`, `MaterialKey(String)`,
  `AppearanceKey(String)`, and `LengthUnit` (same variants and serde names as S00's
  `amg_core::units::LengthUnit`, TODO(FND-10)). `UnitPrefs` is `{ length: LengthUnit }` until
  FND-10 lands. No `amg-jobs` (FND-04) types are needed yet.
- **`BlobRef(pub String)`**: path of a binary entry inside the `.amg` container. DOC-07 may
  refine it.
- **Changed: `Design::baked` is `PMap<BodyId, Arc<BakedBody>>`** (was `PMap<BodyId, BlobRef>`), with
  `pub struct BakedBody { pub blob: BlobRef, pub geom: Option<BodyGeom> }`. This answers S12's
  request that baked geometry be in memory before the first save. `geom` is not in
  `design.json`. DOC-07 writes it to `blob` in `bodies/` on save and fills it back in on load
  (S12 item 3: Phase 1). `PartialEq` compares geometry by pointer. S12 items 1–2
  (`RecomputeCtx::baked`, adding a feature together with its baked payloads) remain for
  DOC-03/DOC-05.
- **Provisional naming types are in `amg-doc::naming` now** (data only: `EntityRef`, `TopoRef`,
  `TopoKind`, `TopoName`, `NameSeg`, `Role`, `NameSource`, `GeomFingerprint`, `GeomClass`,
  `BodyNaming`, `MatchQuality`, `Resolved`, `ResolveError`), because `EvalBody::naming` and
  `eval::Diagnostic::entities` need them. `TopoName` has `new(segments)` and `segments()` and
  serializes as an array. `GeomFingerprint` `Eq`/`Hash` quantise to `QUANTUM = 1e-6`
  (provisional, DOC-11). `NamingBuilder` and the resolver stay in DOC-06.
- **Naming clash:** `amg_doc::Diagnostic` (struct, per Interfaces) and `amg_core::Diagnostic`
  (trait) share a name. Kept as specified, with a doc note. `ModelError` and `ResolveError`
  implement the `amg_core::Diagnostic` trait.

### 2026-10-20 — DOC-02 (feature trait, registry, timeline, lane E)
Deviations from the Interfaces section; the rest holds.
- **`Feature` is split.** `Feature: FeatureMeta + Send + Sync + Debug + Any`. `FeatureMeta`
  (`kind(&self) -> &str`, `version`, `to_json() -> Result<Value, serde_json::Error>`, `as_any`,
  `clone_arc`) is implemented by `register_feature!`, never by hand. `kind` returns `&str`, not
  `&'static str`, so `UnknownFeature` can report the kind it read; `to_json` is fallible
  (non-finite floats).
- **`Feature::validate(&self) -> Result<(), FeatureError>`** (default `Ok`) added: called before a
  feature enters a design and on load. Edits also require the JSON to round-trip byte-equal, which
  rejects NaN/inf.
- **`Feature::exprs`/`exprs_mut` and `FeatureError::Expr` are deferred to DOC-04** (they will be
  defaulted trait methods / a new variant). `FeatureError::UnknownKind { kind, version }` and
  `MissingInput(Box<Dependency>)` added. `RecomputeCtx` is a provisional minimal type
  (`design`, `feature_id`, `context`, `mode`, `tolerance`, `body`, `new_body_id`, `progress`,
  `check_cancelled`); DOC-03 owns it.
- **Wire format.** A feature is `{"kind","v","data"}`; timeline nodes inline these next to
  `id, name, context, suppressed, group`. Unknown kind or a *newer* version loads as
  `UnknownFeature { kind, version, raw }` and saves back byte-identical (needs sorted JSON maps,
  i.e. no `preserve_order`; checked with 20k random f64). Older versions run the registration's
  `migrate`. Unknown features can be suppressed, renamed, removed, but not added, replaced or moved.
- **Registry.** `register_feature!(T, "kind", version = N [, migrate = path])`;
  `registry()` panics and `try_registry()` returns `RegistryError` on duplicate/malformed kind or
  version 0. `link()` convention and the `test-features` cargo feature (self dev-dependency,
  so integration tests see the test kinds). Dependency `inventory` 0.3.25.
- **`Design` timeline API:** `timeline()`, `add_feature`, `replace_feature`, `remove_feature`,
  `move_feature`, `set_marker`, `set_suppressed`, `rename_feature`, `group_features`, `ungroup`,
  `set_group`; `Timeline::{len, marker, nodes, node, node_at, group, groups, group_range,
  dependents, valid_move_range}` (S11 R7). New `ModelError::WouldBreakDependency { feature, input }`.
- **Rules chosen.** Inserting at or before the marker advances it; a move keeps the marker between
  the same neighbours. Groups are contiguous, non-empty, not nested; inserting or moving inside one
  joins it. A deleted producer is allowed (dangling, reported at recompute) on load and after
  `remove_feature`; add/replace need existing earlier producers.
- **Body props origin rule** (resolves the DOC-01 TODO): a body's props need a timeline feature or
  a baked body as origin; `remove_feature` drops the props and baked payloads of its bodies.
- **Tests:** unit tests in `timeline.rs`/`registry.rs`; `tests/registry_link.rs` (separate binary:
  kinds linked) and `tests/registry_duplicate.rs` (duplicate kind fails).

### 2026-10-20 — DOC-04 (parameters & expressions, lane E)
Deviations from the Interfaces/§5 text; everything else holds as specified.
- **`Dim` is `amg_core::Dimension`** (FND-10's 5-slot exponent vector: mm, t, s, K, rad), not the
  8-slot `Dim` with kg. `Quantity` is amg-core's too. Mass is carried in tonnes (mm-t-s system),
  so no kg slot is needed. `stub::LengthUnit` is now `pub use amg_core::units::LengthUnit`
  (same serde names). No amg-core changes were needed.
- **Parse/eval signatures:** `expr::parse(src) -> Result<Ast, ExprError>` (no `UnitTable`
  argument; the table is amg-core's static one). `expr::eval(&Ast, &dyn Env, &EvalOptions) ->
  Result<Quantity, ExprError>` with `EvalOptions { expected, field_unit, length_unit,
  angle_unit, strict }` instead of `(expected, prefs)`. `ParamTable::try_eval` takes `prefs`;
  extra helpers `resolve(&Expr, Dim, prefs)`, `references`, `dependents`. `ExprError` is
  `#[non_exhaustive]` and gains `InvalidName`.
- **`ParamTable::value(id) -> Option<Quantity>`** (was `Quantity`): stored values are always
  valid, `None` only for an unknown id.
- **Fraction rule:** a literal `a/b` followed by a unit (`1/4 in`) is a fraction literal; without a
  following unit it is ordinary division, so `2/4^2` is 0.125 (division of 2 by 4^2) rather than
  the literal EBNF's `(2/4)^2`. This deviates only where the results would differ. Unit `^` binds
  per factor (`N/mm^2` is N per mm squared). Bare unit primaries are allowed; `min` is the
  minute unit unless followed by `(`. Constants count as unit-free literals for the bare-number
  rules (`PI` in an angle field is PI degrees).
- **Strict mode** (open question 3) is an `EvalOptions::strict` flag that disables only the
  "unit-free literal beside a length/angle in `+`/`-` takes the default unit" rule; a whole
  unit-free literal expression still takes the field's default unit. `ParamTable` always
  evaluates leniently; persisting the preference is left to DOC-05/UI. A param with
  `unit: None` accepts any dimension.
- **`Design` API** (low-level, DOC-05 builds on it): `params()`, `try_eval`, `add_param`,
  `set_param_expr`, `rename_param` (rewrites referencing sources via spans), `set_param_meta`,
  `delete_param` (refused while referenced), `add_model_params(owner, Vec<(&mut Expr, Dim)>)`
  (names `d<n>`, all-or-nothing). `Design` gains a `params` field, serialized as `[id, param]`
  entries with `#[serde(default)]` on the wire repr (additive, per the wire-format policy; no
  `FORMAT_VERSION` bump, DOC-07 decides). Name/compiled/value caches are runtime only, rebuilt
  on load. `params` ids are claimed by `validate()`; `owner_of` knows `"parameter"`.
- **Request for DOC-02:** `Feature::exprs_mut` must report each field's dimension, i.e. yield
  `(&mut Expr, Dim)` pairs, because `add_model_params` needs the dimension to pick the unit and
  to type-check. TODO(DOC-02): validate that a model param's owner is a timeline feature.
- **Left for DOC-05:** reject deleting a parameter referenced by a feature field, convert model
  params to user params when their feature is deleted, and re-evaluate params when the default
  unit changes.

### 2026-10-20 — DOC-11 (persistent naming note, lane E)
`docs/notes/persistent-naming.md` (status Proposed until the owner marks it Accepted) fixes the
naming scheme. These changes replace the provisional naming block in Interfaces and the stage
list in §4. DOC-06 implements them; full signatures are in note §10.
- **`NameSeg` grammar changed.**
  - `Modified` is removed: a 1:1-modified face keeps its name.
  - New variants: `Gen { feature, role, from }` (was `Generated`), `Bound { faces, index,
    count }` (edges and vertices, derived from adjacent face names), `Split { feature, index,
    count }` (adds `feature`/`count`), `Merge { feature, with }` (adds `feature`), and
    `Inst { feature, instance }` (pattern/mirror copies).
  - `NameSource` gains `SketchPoint`, `Profile { sketch, outer }` (outer-loop source ids of a
    region, not the `ProfileId`), `Selection { key }`, `Construction` and `Digest(u64)`;
    `MAX_NAME_DEPTH = 8`.
- **`TopoRef`** gains `key: u32` (`#[serde(default)]`): a per-selection key, allocated by
  whoever edits the feature data and kept by Accept match and Re-pick, so generated names
  don't change when a ref is refreshed.
- **`GeomFingerprint`** gains `radius: Option<f64>` (`#[serde(default)]`), and
  `distance(..) -> Option<f64>` (`None` on a class mismatch). `QUANTUM = 1e-6` is confirmed. The resolution metric and thresholds are in `naming::thresholds`.
  `NAMING_VERSION` (= 1) becomes part of `ENGINE_SALT`.
- **`EntityRef::Profile`** gains `fingerprint: Option<ProfileFp>` (`#[serde(default)]`),
  wrapping S03's `ProfileFingerprint`.
- **`BodyNaming`** gains `groups: BTreeMap<u32, TopoName>` (mesh face groups) and
  `removed: Arc<BTreeMap<TopoName, FeatureId>>` (tombstones). New constructors:
  `name_of`, `instanced`, `for_import`, `for_mesh`.
- **`NamingBuilder`**:
  - `from_history` takes `&[(u16, OperandNaming)]`, adopting the S02 request.
  - New `tag`, `role` and `split_order`.
  - `finish(kernel, shape)` (adds the kernel).
  - New `OperandNaming`, `SplitOrder` and `NamingError` variants.
- **Resolution types.**
  - `MatchQuality` gains `Widened`.
  - `Resolved` gains `warning: Option<ResolveWarning>` and loses `Eq`.
  - `ResolveError::NotFound` gains `removed_by: Option<FeatureId>`.
  - `ResolveError::Ambiguous` carries `candidates: Vec<(BodyId, u32)>` instead of a count.
  - New `SetPolicy`, `ResolutionRecord`, the `NamingView` trait, free `resolve()`, and
    `topo_ref_at(view, kernel, body, kind, index)` (S11 R1, with the kernel added).
- **Body lineage.**
  - `BodyChangeKind::Created` gains `derived_from: Option<BodyId>`.
  - New `BodyChangeKind::Consumed { into: BodyId }`.
  - New `Evaluated::successors(BodyId)`.
- **`NodeEval`** gains `resolutions: Arc<[ResolutionRecord]>` (UI re-pick, Accept match,
  corpus harness).
- **`RecomputeCtx`**:
  - `resolve` records a `ResolutionRecord` and turns warnings into node diagnostics
    automatically.
  - New `resolve_all(refs, SetPolicy)`, `resolve_profile`, `naming_builder()`, and
    `kernel()` (if it isn't already exposed).
- **Resolver stages (§4 superseded).**
  - Lineage match with split and bound indices as wildcards.
  - A unique candidate → `Exact`/`Widened`.
  - Several candidates → fingerprint ratio test (`Ambiguous` on ties, `IndexDrift`/`NewSplit`
    warnings).
  - No candidates → tombstone or adjacency check (`NotFound`), else stage-3 fingerprint-only
    with a warning.
- **DOC-06 scope additions:** test features `test.brep_box`, `test.brep_prism` (with
  `unify_same_domain` after Cut/Join), `test.brep_fillet` (with a `policy` field) and
  `test.probe` (note §10), and the corpus harness with its macros (`$pick`, `accept`,
  `saveload`, …; note §11.2). DOC-06's 10 "intended entity" cases are still
  TNP-01/02/03/05/06/07/08/09/10/12; the DOC-06-phase cases TNP-24..28 also run, and TNP-28
  covers Done-when 4 (save/load).
- **DOC-13 Done-when adjusted** in the WP table. The corpus has expected-error cases
  (TNP-04/11/18/20/22), so "resolve to the intended entity" becomes "meet the recorded
  `expect`". "Silent wrong" is defined in note §11.2, and ≥ 5 cases must be *measured* to
  break the naive index baseline. A held-out fuzz set is added (note §11.6).
- **Revision 2026-10-20 (review response, note §16).**
  - `ResolveWarning` is now `NewSplit { feature: Option<FeatureId>, pieces }`, `Unmerged`,
    `Widened { cause: WidenCause }`, `IndexDrift`, `ClassChanged`, `LowMargin`, `FarMatch`,
    `BodyChanged { now_in }`, `FingerprintOnly`, `NoFingerprint` (precedence in note §6.4).
  - `ResolutionRecord { r: EntityRef, result: Result<Vec<Resolved>, ResolveError>, current:
    Vec<EntityRef> }` (covers profile refs and set members expanded to all split pieces).
  - `RecomputeCtx` records resolutions in a `Mutex<Vec<ResolutionRecord>>`; features call
    `resolve*` sequentially from the `compute` thread, so the order is deterministic. New
    `RecomputeCtx::region_source(sketch, key)` and `NamingBuilder::selection(operand, sub,
    key)`.
  - `SetPolicy::DropMissing` expands an `Ambiguous` member whose candidates are all new-split
    pieces to all of them (warned), pending QUESTIONS #44.
  - Cross-spec contracts the note relies on: S04 simplifies every BodyOp result with
    `unify_same_domain`; S02 Q5 (quantised split order) and Q6 (fillet/shell history
    tests); S03 planar-face sketch frame rule and `Profile::outer_sources`. Owner questions
    are QUESTIONS #43–#49.

