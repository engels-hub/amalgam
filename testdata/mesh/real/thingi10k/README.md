# Thingi10K subset

165 STL files, copied byte for byte from the Thingi10K dataset (`raw_meshes/<file_id>.stl`).

- **Dataset:** Qingnan Zhou and Alec Jacobson, "Thingi10K: A Dataset of 10,000 3D-Printing
  Models", arXiv:1605.04797. https://ten-thousand-models.appspot.com,
  https://huggingface.co/datasets/Thingi10K/Thingi10K (the dataset's code is Apache-2.0; the
  models carry the licence of their Thingiverse upload).
- **Per-model licence evidence:** the dataset's `metadata/input_summary.csv` (column `License`)
  and `metadata/contextual_data.csv` (thing id, name, author), reproduced per file in
  [`NOTICE`](NOTICE) and in `../../corpus.toml` (`license`, `author`, `title`, `source_url` =
  the Thingiverse page of the thing).
- **Selection:** only licences that allow redistribution with attribution: "Creative Commons -
  Public Domain Dedication" (CC0-1.0), "Public Domain" (`LicenseRef-PublicDomain`), "Creative
  Commons - Attribution" (CC-BY) and "Creative Commons - Attribution - Share Alike" (CC-BY-SA).
  NonCommercial, NoDerivatives and GPL/LGPL/BSD-licensed models were not taken. The models are
  unmodified, used only as test inputs and are not part of any shipped build, which is what the
  attribution and share-alike terms require here.
- **Licence versions are assumed:** the dataset records only the licence family. Thingiverse
  offered CC 3.0 (unported) for uploads between 2009 and 2015 (the dataset's range), so the SPDX
  ids are `CC-BY-3.0` / `CC-BY-SA-3.0`. If a model's page shows another version, correct its
  `license` in `corpus.toml` and the `NOTICE`.
- **Stratification:** about 14 models per defect category (5 inverted: the dataset has few
  closed, oriented models with negative volume under the size limit), 12 scan-like "hopeless"
  soups, 20 clean controls; every file is at most 1.5 MB (largest 1.35 MB).
  Seeded selection (Python `random.Random(28)`), reproducible from the dataset metadata.
