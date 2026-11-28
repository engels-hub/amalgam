# Golden TNP corpus

The persistent-naming test corpus of `docs/notes/persistent-naming.md` §11 (DOC-11). The
note is the specification of every case: format §11.2, bases §11.3, cases and the reasoning
behind each expected outcome §11.4. The case files here are authoritative where the note's
copies still use the provisional S04 field names (see "S04 schemas" below).

```
bases/B1.json B2.json   shared base command scripts (box 100 × 60 × 40; polygon prism)
bases/S6.json S17.json  shared scripts named by a case's "script"
cases/TNP-NN.json       one case per file ("phase": "DOC-06" or "DOC-13")
```

## Harnesses

- `amg-doc/tests/tnp.rs` (DOC-06) runs the `"phase": "DOC-06"` cases on the lane-E test
  features (`test.brep_box`, `test.brep_prism`, `test.brep_fillet`, `test.probe`); it needs the
  OCCT backend (`cargo nextest run -p amg-doc --features occt`). It writes the measured naive
  index baseline per case to `<target>/tmp/tnp-report.json`.
- `amg-features/tests/tnp.rs` (DOC-13, CI-gated) runs **all 30 core cases on the S04
  features** and writes `<target>/tmp/tnp-doc13-report.json` (per-case outcome: expect met /
  feature error / warning / silent wrong, and the measured naive baseline). It fails on any
  silent wrong, on < 90 % expect met, on < 5 cases breaking the naive baseline, on a wrong
  naive prediction, and above 60 s.
- `amg-features/tests/tnp_fuzz.rs` (DOC-13, CI-gated) is the held-out fuzz set of note §11.6:
  200 seeded runs on B1/B2 with one random upstream edit, oracle "a quiet `Ok` must be the
  edge containing the mapped pick point".

```
cargo nextest run -p amg-features --features occt --test tnp --test tnp_fuzz --no-capture
```

Both need `--features occt` (and `AMG_OCCT_DIR`); without it only the case files are parsed and
the fuzz generator is checked for determinism. The shared harness is
`amg-features/tests/tnp_common/mod.rs`.

## DOC-06 cases on S04 features

The DOC-13 harness maps the lane-E B-rep test features to S04 features, so the DOC-06 cases
run unchanged on real features:

| Test feature | S04 features |
|---|---|
| `test.brep_box {size: [x, y, z]}` | `sketch.sketch` on XY with lines 1..4 of the rectangle (0,0)-(x,y) + `solid.extrude` by z, `NewBody` |
| `test.brep_prism {points: [[tag, x, y]…], z0, height, op, target}` | `sketch.sketch` on XY, one line per polygon edge keyed by its start-point tag + `solid.extrude` of the region, start offset `z0`, distance `height`, `New`/`Cut`/`Join` → `NewBody`/`Cut`/`Join` on `target` |
| `test.brep_fillet {body, edges, radius, policy}` | `solid.fillet` with one constant-radius set (the S04 fillet always resolves its edges with `DropMissing`) |
| `test.probe` | kept (it only reads a reference; no S04 feature references a non-planar face only) |

A bind names the extrude (`{"origin": "$box", "slot": 0}` is the extruded body); an edit of the
test feature (`EditFeature`, `patch`) moves the lines of kept tags, adds lines for new tags,
removes dropped ones and re-picks the extrude's profile; `SetSuppressed` suppresses sketch and
extrude; `$index_of:` and `MoveFeature.to` count the pair as one timeline entry.

`naive_index_breaks` is the DOC-13 measurement on the S04 features. A DOC-06 case whose lane-E
measurement differs keeps that in `naive_index_breaks_doc06` (TNP-02: the test prism shifts
every index when a vertex is inserted at the start of its point list; the sketch region's
edge order does not change).

## Macros (note §11.2)

`$root`, `$name`, `$index_of:name`, `$pick`, `accept`, `saveload`, `patch` as in the note, and the
DOC-13 macros:

| Macro | Arguments | Expands to |
|---|---|---|
| `sketch` | `{plane: "XY" \| "XZ" \| "YZ" \| {"$pick": …}, entities: [["L", id, p0, p1], ["C", id, c, r], ["P", id, p]]}` | `sketch.sketch` with fixed geometry; on a face, world x, y are converted with the plane the sketch resolves to (S03 planar-face frame) |
| `sketch_edit` | `{id, remove: [ids], add: [entities], move: [[id, p0, p1]]}` | `EditFeature` of the sketch (moved lines keep their entity) |
| `$profile` | `{sketch, at: [x, y]}` | `EntityRef::Profile` of the region containing `at`, with its fingerprint |
| `EXT` / `EXTN` | `[sketch, at \| [at…], extent, op, participants?]` | `solid.extrude` (`Extrude::distance`; `"All"` → `Extent::All`), `OpTarget { op, participants }` |
| `FIL` | `[pick, radius]` | `solid.fillet` (`Fillet::constant`) |
| `CHA` | `[pick, distance]` | `solid.chamfer` (`Chamfer::equal`) |

`patch` merges with RFC 7396, except that an S01 expression `{"Inline": "…"}` replaces the stored
`{"Param": id}` instead of merging into it.

## S04 schemas

The note's provisional field names (§11.2) are renamed to the final SOL-* schemas; the intent of
each case is unchanged:

| Note (provisional) | Case files (final) |
|---|---|
| `solid.pattern_circular {features, axis: "OriginZ", count: "6", angle: "360 deg"}` | `solid.pattern {objects: {"Features": […]}, kind: {"Circular": {axis: {"Origin": "ZAxis"}, count: {"Inline": "6"}, span: "Full"}}}`; count patch `{kind: {Circular: {count: {"Inline": "8"}}}}` |
| `solid.mirror {bodies, plane: "OriginYZ"}` | `solid.mirror {objects: {"Bodies": […]}, plane: {"Origin": "YZPlane"}, op: "NewBody"}` |
| `solid.split_body {body, tool: "OriginYZ"}` | `solid.split_body {body, tool: {"Origin": "YZPlane"}}` |
| `solid.revolve {profiles, axis: "OriginZ", angle: "270 deg", op}` | `solid.revolve {profiles, axis: {"Origin": "ZAxis"}, side1: {extent: {"Distance": {"Inline": "270 deg"}}, taper}, target: {op}}`; 360° is `side1.extent: "Full"` |
| `solid.combine {target, tools, op, keep_tools}` | unchanged |
| extrude patch `{extent: {"Distance": "25 mm"}}` | `{side1: {extent: {"Distance": {"Inline": "25 mm"}}}}` |

A wrong `naive_index_breaks` prediction fails the DOC-13 run (and is only reported by DOC-06);
correct the case file and the note in the same change.
