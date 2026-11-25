# STEP and IGES fixtures (S02 BREP-13)

`step/*.step` and `../iges/*.igs` are written by the kernel itself (no downloads), from the
models in `crates/amg-brep/tests/common/io_models.rs`:

```sh
cargo run -p amg-brep --features occt --example gen_brep_io
```

The generator reads each file back and prints its tree, so a change in what OCCT writes shows
up both in the file diff and in that output. The files carry OCCT's write timestamp, so
regenerating changes their headers even when nothing else changed.

| File | Schema | What it covers |
|---|---|---|
| `box_ap242.step` | AP242 | one named part, 10 × 20 × 30 mm box |
| `cylinder_ap203.step` | AP203 | a part in AP203 (anonymised person and organisation) |
| `coloured_box_ap214.step` | AP214 | part colour, and colours on faces 0 and 3 |
| `inch_block.step` | AP214 | written in inches: 1 × 2 × 3 in, reads back as 25.4 × 50.8 × 76.2 mm |
| `bolt_pattern.step` | AP242 | assembly: a base and 4 instances of one bolt part, one with its own colour |
| `nested_assembly.step` | AP214 | assembly of two placed sub-assemblies (one turned 90°) sharing 2 parts |
| `two_roots.step` | AP242 | two free parts (sphere, torus) |
| `filleted_box.step` | AP242 | the 26-face filleted box (`../filleted_box.brep`), a coloured fillet face |
| `plate_with_holes.step` | AP214 | the 1030-face plate (`../plate_with_holes.brep`), 4 MB |
| `open_shapes.step` | AP242 | non-solids: a B-spline face and an open revolved face |
| `../iges/box_iges.igs` | IGES 5.3 | a box as an MSBO B-rep solid |
| `../iges/coloured_cylinder.igs` | IGES 5.3 | a coloured cylinder (IGES puts the colour on every face too) |
| `../iges/filleted_box_iges.igs` | IGES 5.3 | the filleted box |

`*.step` is git-LFS tracked (root `.gitattributes`); `*.igs` is not. `crates/amg-brep/tests/io.rs`
skips an un-fetched LFS pointer unless `AMG_REQUIRE_TESTDATA=1`, and also writes the same models
to temporary files, so its checks run without LFS.
