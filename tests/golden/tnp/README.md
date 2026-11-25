# Golden TNP corpus

The persistent-naming test corpus of `docs/notes/persistent-naming.md` §11 (DOC-11). The
note is the specification of every case: format §11.2, bases §11.3, cases and the reasoning
behind each expected outcome §11.4. Files here are the cases verbatim.

```
bases/B1.json B2.json   shared base command scripts (box 100 × 60 × 40; polygon prism)
bases/S6.json S17.json  shared scripts named by a case's "script"
cases/TNP-NN.json       one case per file ("phase": "DOC-06" or "DOC-13")
```

- `amg-doc/tests/tnp.rs` (DOC-06) runs the `"phase": "DOC-06"` cases on the lane-E test
  features (`test.brep_box`, `test.brep_prism`, `test.brep_fillet`, `test.probe`); it needs the
  OCCT backend (`cargo nextest run -p amg-doc --features occt`). It writes the measured naive
  index baseline per case to `<target>/tmp/tnp-report.json`.
- DOC-13 runs every core case through the S04 features (`amg-features/tests/tnp.rs`) and adds the
  macros the DOC-13 cases use (`sketch`, `EXT`, `FIL`, ...).

A wrong `naive_index_breaks` prediction is reported, not failed; correct the case file and the
note in the same change.
