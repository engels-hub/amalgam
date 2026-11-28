# Fusion-parity harness (S04 SOL-29)

Rebuilds the owner's Fusion designs with Rust scripts and compares each with the STEP file
Fusion exported: volume and area within 1e-4 relative, bounding box within 1e-3 mm, same number
of solids.

## Folder layout

The designs are private and never committed. Put them **directly** in
`$AMG_TESTDATA_DIR/fusion/` (no sub-folders; the IO-06 round-trip tests read the same files):

```text
$AMG_TESTDATA_DIR/
  fusion/
    <name>.step       required: Fusion "Export → STEP" of the finished design (.stp also works)
    <name>.md         the note: purpose, features used, timeline with every value
    <name>-*.png      screenshots of the timeline and of the feature dialogs
```

`<name>` is a plain file stem such as `phone-stand` (no dots), unique in the folder. 5–10
representative designs are wanted (S04 Open question 1).

Export notes:
- The STEP must hold exactly the bodies the script leaves visible. Hide or remove construction
  and tool bodies in Fusion before exporting.
- Keep the design's own origin and axes: the bounding boxes are compared in model coordinates.

What `<name>.md` needs so a script can be written without opening Fusion: for every timeline
entry, in order, the feature, its inputs (which sketch, profile, face or edge, with coordinates)
and every value (distance, angle, radius, count, direction, operation). For sketches: the plane
and the coordinates and sizes of the geometry, in mm.

## Script format

One Rust function per design in `designs.rs` (or a sub-module of it), listed in `DESIGNS` under
the STEP file's stem:

```rust
use amg_doc::Document;
use amg_features::solid::BodyOp;

use super::harness::{Design, Script};
use crate::solid_common::extrude_rects;

fn phone_stand(doc: &mut Document) {
    // A 60 × 40 sketch on XY, extruded 10 mm.
    extrude_rects(doc, &[[0.0, 0.0, 60.0, 40.0]], 10.0, BodyOp::NewBody, vec![]);
    // Later features: build the feature struct, then
    // `crate::solid_common::add(doc, FeatureBox::new(...))`.
}

pub const DESIGNS: &[Design] = &[Design {
    name: "phone-stand",
    script: Script::Build(phone_stand),
    explained: None,
}];
```

- A script receives an empty document and adds features with `solid_common::add`, which
  recomputes after each one. Picking faces and edges: see `edge_at` in `selfmade.rs`
  (`topo_ref_at` on the entity nearest a point).
- A design that needs a feature we do not have yet:
  `script: Script::Pending("needs solid.hole (SOL-12)")`.
- A difference that is understood and accepted: `explained: Some("why")`. The harness prints it
  instead of failing; copy the reason into the S04 refinement log.

The scripts are committed to the repository, so they carry the designs' dimensions. Ask the
owner before committing a script of a design that must stay private.

## Running

```sh
export AMG_TESTDATA_DIR=<dir>  AMG_OCCT_DIR=<occt>
cargo nextest run -p amg-features --features occt --test solid_fusion --no-capture
```

| Situation | Result | With `AMG_REQUIRE_TESTDATA=1` |
|---|---|---|
| `AMG_TESTDATA_DIR` unset or no `fusion/` folder | skip | skip |
| Built without `occt` | skip | skip |
| STEP file without a script | skip, listed | fail |
| Script without a STEP file | skip, listed | fail |
| Un-fetched git-LFS pointer | skip, listed | fail |
| `Script::Pending` | skip, listed | skip |
| Outside the bounds, `explained` set | pass, listed | pass |
| Outside the bounds, script or feature error, unreadable STEP | fail | fail |

For parts whose edge tolerances are above the kernel's 1e-6 mm (loose third-party geometry),
the owner's IO-06 rule applies (S12, 2026-11-26): the volume bound widens to
`area × max tolerance` and the bbox bound to the tolerance, if that is larger than the normal
bound. The report says when it did.

`selfmade.rs` is a stand-in design whose STEP file the test writes itself, so the whole path runs
without private data.
