# Materials: property model, data provenance and library format

Linked from [S17 — Materials & appearances](../specs/S17-materials.md). Written 2026-10-20
during S17 refinement. Consumers: S05 (INS-03 mass properties), S12 (IO-03 3MF materials and
colours), S13 (FEA), S14 (generative), S16 (AM print setup, slicer hand-off), S10 (rendering via
the `amg-ui` adapter). Units follow [`units-and-quantities.md`](units-and-quantities.md)
(coherent mm–t–s–K–rad, accepted 2026-10-20).

## 1. What each consumer needs

| Consumer | Properties read | When |
|---|---|---|
| S05 INS-03 mass properties | density | Phase 2 |
| S12 IO-03 3MF | material name, display colour (sRGB `#RRGGBBAA`), metallic/roughness (3MF `pbmetallic`), slicer filament type | Phase 1 |
| S16 print setup / Orca hand-off | AM process (ISO/ASTM 52900 category), slicer filament type (Orca `filament_type` string), nozzle/bed/chamber temperature ranges, shrinkage, cost per kg, density (part mass, cost) | Phase 1 (AM-01, AM-06) |
| S13 linear static / modal (P1) | isotropic E, ν, density, yield and ultimate strength (safety factor) | Phase 4 |
| S13 thermal (P2) | conductivity, specific heat, CTE + reference temperature | Phase 5 |
| S13 nonlinear (P2) | bilinear plasticity (yield, tangent modulus) or a true stress–plastic strain table | Phase 5 |
| S13 orthotropic printed (P2, QUESTIONS #3) | transversely isotropic constants with the symmetry axis = build direction, directional allowables (XY and Z), Tsai-Wu inputs | Phase 5 |
| S14 generative | same as S13 linear static, plus density | Phase 4 |
| S10 rendering | PBR appearance (glTF 2.0 metallic-roughness core, optional transmission/IOR/clearcoat/emissive, textures at P2) | Phase 1 (colour), Phase 3 (full) |

Out of the material model (lives elsewhere):
- infill, perimeters, layer height, print speed: S16 print setup (and S08 for lattice
  homogenisation). A material describes **fully dense** printed material (100 % infill
  specimens, as in the datasheets).
- the build direction of a body: S16 print setup. The material only says which material axis is
  the layer-normal axis (axis 3).
- fatigue, creep, viscoelasticity, temperature-dependent curves: S19 / later.

## 2. Property model decisions

### 2.1 Elastic models
Three closed variants (enum `Elastic`). The serialized form is fixed in format v1, so MAT-04
(P2) adds only math and data, never a migration.

| Variant | Independent constants | Used for |
|---|---|---|
| `Isotropic { youngs, poisson }` | 2 | metals, resins (owner: resin is isotropic), P1 FEA |
| `TransverselyIsotropic { e_p, e_t, nu_p, nu_pt, g_pt }` | 5 | FFF printed polymers (QUESTIONS #3) |
| `Orthotropic { e: [E1,E2,E3], nu: [ν12,ν13,ν23], g: [G12,G13,G23] }` | 9 | raster-aligned FFF, user data from tests |

Conventions (identical to Abaqus/CalculiX `*ELASTIC, TYPE=ENGINEERING CONSTANTS`, so values
from the literature and CalculiX cross-checks map one-to-one):
- Material axes 1, 2, 3. For printed materials **axis 3 is the layer normal (build
  direction)**, and 1-2 is the layer plane. Transversely isotropic = isotropic in the 1-2 plane.
- `ν_ij = −ε_j / ε_i` under uniaxial stress `σ_i`. So `ν_pt = ν13 = ν23` (load in plane, strain
  through), and `ν_tp = ν_pt · E_t / E_p`.
- Transversely isotropic: `G12 = E_p / (2 (1 + ν_p))` is derived, `G13 = G23 = g_pt`.
- Voigt order in `stiffness_voigt()` / `compliance_voigt()`: `(11, 22, 33, 23, 13, 12)` with
  engineering shear strains (γ = 2ε). This differs from the Abaqus/CalculiX order
  `(11, 22, 33, 12, 13, 23)`. S13 maps explicitly; the constant `VOIGT_ORDER` documents it.

Validation (thermodynamic admissibility = positive-definite compliance; Lempriere, "Poisson's
ratio in orthotropic materials", AIAA J. 6(11), 1968):
- all `E_i > 0`, `G_ij > 0`;
- isotropic: `−1 < ν < 0.5`;
- transversely isotropic: `|ν_p| < 1`, `|ν_pt| < sqrt(E_p / E_t)`,
  `(1 + ν_p)(1 − ν_p − 2 ν_pt ν_tp) > 0`;
- orthotropic: `|ν_ij| < sqrt(E_i / E_j)` and
  `1 − ν12 ν21 − ν23 ν32 − ν13 ν31 − 2 ν21 ν32 ν13 > 0`.

`isotropic_equivalent(policy)` exists so a P1 isotropic solver can use an anisotropic material
**explicitly**, never silently: `Conservative` takes `min(E_i)` and the in-plane ν, `InPlane`
takes the layer-plane values. The S13 UI shows a warning when a non-isotropic material is
reduced.

### 2.2 Directional values
`Directional<T> { xy: T, z: Option<T> }`: `xy` is the in-layer value, `z` the value along axis
3 (inter-layer). `z = None` means isotropic. Used for strengths, elongation, CTE, conductivity
and shrinkage. FFF datasheets that publish "Z" specimens (Bambu Lab, Polymaker) fill `z`
directly. Example (Bambu Lab PLA Basic TDS V2.0, ISO 527): E 2680 ± 130 MPa (X-Y) vs
2160 ± 90 MPa (Z); tensile strength 39 ± 2 MPa (X-Y) vs 35 ± 3 MPa (Z).

**Specimen orientation pitfall.** Datasheets use "vertical", "XZ", "upright" and "Z"
inconsistently. Prusament PETG v1.1 calls an on-edge specimen "vertical (XZ)": it is still
loaded **in** the layer plane and is not a Z value. Each value therefore carries the specimen
orientation in ISO/ASTM 52921 orthogonal notation (`XYZ`, `XZY`, `ZXY`, …; first letter = the
specimen's long axis, which is the load axis). Only `Z**` specimens may fill a `z` slot. The
library loader rejects a `z` value whose provenance orientation does not start with `Z`.

### 2.3 Strength and failure inputs
`Strength { yield_tensile, ultimate_tensile, ultimate_compressive: Option<Directional<Pressure>>,
shear, flexural: Option<Pressure> }`. Tsai-Wu (MAT-04) derives `F1, F11, F3, F33, F44, F66` from
`Xt = ultimate_tensile.xy`, `Xc = ultimate_compressive.xy`, `Zt/Zc` from the `z` slots and
`S = shear`. The interaction term defaults to the common estimate
`F13 = −0.5 · sqrt(F11 F33)` (Tsai & Hahn), and is overridable. Missing compressive values fall
back to tensile with an `Estimate` provenance flag. These are typical values, **not design
allowables**: the UI says so, and safety factors stay a user decision.

### 2.4 Thermal, plasticity, AM
- `Thermal { conductivity: Directional<ThermalConductivity>, specific_heat, expansion:
  Directional<ThermalExpansion>, expansion_reference: Temperature, glass_transition, melting,
  heat_deflection_045 (HDT at 0.45 MPa), max_service }`. Every field is optional.
- `Plasticity::Bilinear { yield_stress, tangent_modulus }` or `Table { points: (true plastic
  strain, true stress) }`, strictly increasing in strain.
- `AmInfo { process, slicer_type, nozzle_temp, bed_temp, chamber_temp, shrinkage, cost }`.
  - `process` uses the seven ISO/ASTM 52900 categories.
  - `slicer_type` is the OrcaSlicer `filament_type` string, so AM-06 can write it into the
    Orca project config. The vocabulary is taken from Orca's `MaterialType::all()`: `PLA`,
    `PETG`, `ABS`, `ASA`, `PA`, `PA12`, `PA-CF`, `PAHT-CF`, `TPU`, `PC`, `PLA-CF`, `PETG-CF`,
    `PCTG`, …. Only the names are used, not Orca's code (AGPL).
  - Resin and metal materials leave `slicer_type` empty. Chitubox takes STL/3MF only.

### 2.5 New units (additive to FND-10)
The FND-10 `Unit` table has no thermal transport units. MAT-01 adds these rows to
`amg_core::units`. The change is additive and the PR is flagged as an Interface change to S00.

| Dimension (L, M, T, Θ, A) | New const | Unit variant | Symbols | Scale to canonical |
|---|---|---|---|---|
| (1, 1, −3, −1, 0) | `THERMAL_CONDUCTIVITY` | `WattPerMetreKelvin` | `W/(m*K)`, `W/mK` | 1 (W/(m·K) = t·mm/(s³·K) exactly) |
| (2, 0, −2, −1, 0) | `SPECIFIC_HEAT` | `JoulePerKilogramKelvin` | `J/(kg*K)`, `J/kgK` | 1e6 |
| (0, 0, 0, −1, 0) | `THERMAL_EXPANSION` | `PerKelvin`, `PpmPerKelvin` | `1/K`; `ppm/K`, `um/(m*K)` | 1; 1e−6 |
| (2, 1, −3, 0, 0) | `POWER` | `Milliwatt`, `Watt` | `mW`; `W` | 1; 1e3 |
| (0, 0, 0, 0, 0) | — | `Percent` | `%` | 0.01 |

Typed quantities `ThermalConductivity`, `SpecificHeat` and `ThermalExpansion` are added with
the same macro. Sanity check: steel, c = 460 J/(kg·K), is 4.6e8 mm²/(s²·K) canonical, which
matches the CalculiX/Abaqus mm–t–s tables.

## 3. Provenance and licence policy for built-in data

### 3.1 Legal basis (not legal advice; personal-use project, ADR-012)
- Individual measured values (facts) are not copyrightable. The **selection and
  arrangement** of a database can be, and in the EU the sui generis database right
  (Directive 96/9/EC) protects against extracting a *substantial part* of a database.
- Policy:
  1. Each built-in value comes from an individually cited primary document: a manufacturer
     datasheet, a standard, a handbook table or a paper.
  2. Never bulk-copy from aggregators or commercial databases: MatWeb, CAMPUS, Granta/Ansys
     Materials, Total Materia, the Autodesk/Fusion material library, or slicer profile packs.
  3. A handful of values from one datasheet per material is fine.
- Community databases with an open data licence may be imported with attribution. Each needs
  a `THIRD_PARTY.md` row when actually used:
  - **SpoolmanDB** (MIT, verified 2026-10-20) has `materials.json` with density in g/cm³ and
    nozzle/bed temperatures.
  - **Open Filament Database** (stated MIT by its steward SimplyPrint; the primary licence file
    is not verified).
  - **OpenPrintTag** (Prusa; repository MIT, the spec-data licence is not separately verified).
  - MAT-05 uses them only to cross-check densities and temperature ranges, not as a primary
    source, unless their licence file is verified first.
- Proprietary-format readers (Fusion `.adsklib`) are out of scope.

### 3.2 Per-value provenance
Every numeric value in the built-in library has a `ValueProv`:
- `source`: the key of a `[[source]]` record (title, publisher, document version or date, URL,
  access date, `kind`);
- `quality`:
  - `measured`: taken verbatim from the source, unit-converted only;
  - `derived`: computed from other values, and the note gives the formula (e.g. `G = E/(2(1+ν))`);
  - `typical`: a handbook or range value, the note gives the range;
  - `estimate`: engineering judgement, shown with a warning icon in the UI;
- `specimen`: ISO/ASTM 52921 orientation for printed specimens;
- `standard`: the test standard, e.g. `ISO 527-2`, `ASTM D638`, `ISO 1183`;
- `note`: the condition, such as "dry", "post-cured 60 min at 60 °C" or "stress relieved 2 h
  at 300 °C".

Rules enforced by tests (MAT-05):
- density, in-plane E and one tensile strength are `measured` for every built-in material;
- no `estimate` on density;
- each material states its `condition` (as printed / dry / post-cured / as-built /
  heat-treated);
- every `source` key resolves.

### 3.3 Candidate sources for library v1 (MAT-05 pins exact versions)

| Material id (`amg.builtin/…`) | Process | Primary sources | Notes |
|---|---|---|---|
| `pla.fff` | ME | Bambu Lab PLA Basic TDS V2.0 (XY + Z, ISO 527); Prusament PLA TDS | ν: literature, `typical`, e.g. Song et al., *Materials & Design* 123 (2017), on printed PLA |
| `petg.fff` | ME | Prusament PETG TDS v1.1 (2022: ρ 1.27 g/cm³, E 1.5 GPa, σy 47 MPa XY, HDT 68 °C); Bambu PETG HF TDS for Z | Mind the "vertical (XZ)" pitfall §2.2 |
| `abs.fff` | ME | Bambu Lab ABS TDS; Polymaker PolyLite ABS TDS | |
| `asa.fff` | ME | Prusament ASA TDS; Bambu Lab ASA TDS | |
| `pa12.fff` | ME | a neat-PA12 filament TDS (MAT-05 picks one that reports XY and Z) | moisture: values "dry" |
| `pa-cf.fff` | ME | Bambu Lab PAHT-CF TDS; Prusament PA11 Carbon Fiber TDS | strongly anisotropic, Z values important |
| `tpu-95a.fff` | ME | Bambu Lab TPU 95A HF TDS; Polymaker PolyFlex TPU95 TDS | hardness Shore 95A; E small, ν ≈ 0.45–0.48 `typical` |
| `pa12.pbf` | PBF-P | EOS PA 2200 or HP 3D HR PA12 datasheet | spec Summary lists PA12; SLS/MJF service parts |
| `resin.standard` | VPP | Formlabs Standard (Grey) TDS (ASTM D638, green vs post-cured); Elegoo Standard TDS for density and hardness | Elegoo TDS gives "tensile modulus 520.96" without a unit, so it is not usable for E |
| `resin.tough` | VPP | Formlabs Tough 2000 TDS or Elegoo/Anycubic ABS-like TDS | |
| `ti6al4v.lpbf` | PBF-M | EOS Titanium Ti64 Grade 5 MDS (heat treated, e.g. Rp0.2 945/965 MPa horizontal/vertical on M 290); ASTM F2924 minimums | condition: stress relieved per MDS |
| `alsi10mg.lpbf` | PBF-M | EOS Aluminium AlSi10Mg MDS; ASTM F3318 | as-built and stress-relieved may be two entries |
| `316l.lpbf` | PBF-M | EOS StainlessSteel 316L MDS; ASTM F3184 | |
| `steel.s235` | wrought | EN 1993-1-1 §3.2.6 (E 210 GPa, G 81 GPa, ν 0.3, α 12e−6/K); EN 10025-2 (fy); EN 1991-1-1 Annex A (ρ 7850 kg/m³) | reference metal for general CAD |
| `aluminium.6061-t6` | wrought | EN 1999-1-1 §3.2.5 (E 70 GPa, ν 0.3, α 23e−6/K, ρ 2700 kg/m³) and Table 3.2b (EN AW-6061 T6) | |
| `stainless.1.4404` | wrought | EN 1993-1-4 (E 200 GPa), EN 10088-2 | wrought counterpart of 316L |
| `ti6al4v.wrought` | wrought | ASTM B348/B265 Grade 5 minimums | |

Thermal properties of polymers are rarely in filament TDS. Use the TDS where given (HDT, Tg,
sometimes CTE), otherwise a cited handbook value marked `typical`. MAT-05 may add materials,
for example the owner's own resin brand (S17 open question 2).

### 3.4 Plausibility bounds (test-enforced)
These are sanity bounds, not values, and catch unit slips (×1000) and typos.

| Family | ρ (g/cm³) | E in-plane | ν | UTS |
|---|---|---|---|---|
| Unfilled FFF thermoplastics (PLA, PETG, ABS, ASA, PA, PC) | 0.95–1.45 | 0.8–4.5 GPa | 0.30–0.42 | 15–90 MPa |
| Fibre-filled FFF (–CF, –GF) | 1.0–1.5 | 2–15 GPa | 0.25–0.42 | 30–150 MPa |
| TPU | 1.05–1.30 | 5–150 MPa | 0.40–0.49 | 10–60 MPa |
| Photopolymer resins | 1.05–1.30 | 0.3–4 GPa | 0.30–0.45 | 15–90 MPa |
| Ti-6Al-4V | 4.35–4.50 | 100–125 GPa | 0.30–0.36 | 850–1300 MPa |
| AlSi10Mg | 2.60–2.72 | 60–80 GPa | 0.30–0.35 | 250–500 MPa |
| 316L | 7.85–8.05 | 160–210 GPa | 0.26–0.31 | 450–750 MPa |
| Carbon / stainless steel, Al alloys (wrought) | 2.6–8.1 | 65–215 GPa | 0.26–0.35 | 200–800 MPa |

## 4. Library file format (TOML, `format_version = 1`)

- One file is one library. Libraries are human-editable, diffable and commented.
- Values are **strings with a unit**, parsed by `amg_core::parse_quantity` and dimension-checked
  against the property. A bare number is an error for dimensioned properties; library files
  are strict, unlike lenient expressions (QUESTIONS #13). Dimensionless values (ν, strain) take
  a bare number or `%`.
- A value is either a plain string (provenance = the material's `default_source`, quality
  `measured`) or an inline table `{ v, src, q, spec, std, note }`.
- Temperatures as absolute values in `degC` or `K`. Shrinkage and elongation are fractions or `%`.

```toml
format = "amalgam-materials"
format_version = 1

[library]
id = "amg.builtin"            # [a-z0-9._-]+ ; user libraries must start with "user."
name = "Amalgam built-in"
version = "2026.1"            # data version, shown in the UI
data_licence = "see SOURCES"  # owner decides the licence of the built-in data file

[[source]]
key = "bambu-pla-basic-tds-v2"
kind = "datasheet"            # datasheet | standard | handbook | paper | community_db | user_measured | estimate
title = "PLA Basic Technical Data Sheet"
publisher = "Bambu Lab"
version = "V2.0"
url = "https://…"
accessed = "2026-10-20"

[[material]]
id = "pla.fff"                # full MaterialId = "amg.builtin/pla.fff"
revision = 1                  # bump on any value change
name = "PLA (FFF printed)"
category = "polymer"
family = "pla"
condition = "printed, 100 % infill, dry, 23 degC"
default_source = "bambu-pla-basic-tds-v2"
appearance = "amg.builtin/plastic.white-matte"
density = { v = "1.24 g/cm^3", src = "prusament-pla-tds", std = "ISO 1183" }

[material.mechanical.elastic]
model = "isotropic"           # isotropic | transversely_isotropic | orthotropic
youngs = { v = "2680 MPa", spec = "XYZ", std = "ISO 527" }
poisson = { v = "0.36", q = "typical", src = "song-2017-pla", note = "range 0.33–0.36" }

[material.mechanical.strength]
ultimate_tensile.xy = { v = "39 MPa", spec = "XYZ", std = "ISO 527" }
ultimate_tensile.z  = { v = "35 MPa", spec = "ZXY", std = "ISO 527" }

[material.thermal]
heat_deflection_045 = { v = "57 degC", std = "ISO 75" }

[material.am]
process = "material_extrusion"
slicer_type = "PLA"
nozzle_temp = { min = "190 degC", max = "230 degC" }
bed_temp = { min = "35 degC", max = "65 degC" }
```
(The numbers above illustrate the syntax. MAT-05 enters the real values.)

Appearances use the same file structure under `[[appearance]]`:
```toml
[[appearance]]
id = "plastic.white-matte"
revision = 1
name = "Plastic – white, matte"
base_color = "#F2F2F0FF"      # sRGB, #RRGGBB or #RRGGBBAA (same as 3MF ST_ColorValue)
metallic = 0.0
roughness = 0.8
```

Writer rules (fixed point after one save):
- materials sorted by id; keys in schema order;
- each quantity printed in the property's *library unit* with shortest round-trip formatting:
  - g/cm³ for density;
  - MPa for stress and moduli;
  - degC for temperatures;
  - W/(m*K), J/(kg*K) and ppm/K for thermal values;
- provenance tables written only where they differ from the default.

Load→save→load keeps every value within 2 ulp. A second save is byte-identical.

**Versioning.**
- `format_version` is the file schema. Loading runs a migration chain over `toml::Value`
  (`v1→v2→…`), like S01 document migrations, and a newer version gives `TooNew`.
- `library.version` is the data version.
- `material.revision` is per material and drives drift detection (§5).

**Locations.**
- Built-in: `crates/amg-materials/data/builtin/{materials,appearances}.toml`, embedded with
  `include_str!`, read-only.
- User: every `*.toml` in a directory the app passes in. Default `%APPDATA%\Amalgam\materials\`
  (roaming; config, not cache). `amg-materials` hard-codes no paths, as with S01 recovery.
- Saves are atomic: write a temp file, then fsync, then rename. A malformed user file is
  reported and skipped, and the other libraries still load.

## 5. Materials inside the document

- **Self-contained documents (Fusion "In this design").** Assigning a library material copies
  it into the document's material table. `BodyProps.material` holds a `MaterialKey` (= the
  `MaterialId`) into that table. A document opens with identical mass/FEA results on a
  machine without the user library, which the determinism principle requires.
- **No silent updates.** A document copy keeps `(id, revision, content hash)`.
  `drift(copy, libraries)` reports:
  - `UpToDate`;
  - `LibraryNewer { revision }`;
  - `LocallyModified`: the hash differs at the same revision;
  - `NotInLibrary`.
  The UI offers "Update from library", one undoable command. Nothing auto-updates.
- **Storage.** Proposed as an additive S01 change: typed tables `materials` and `appearances`
  in `Design` plus commands `SetMaterial`, `RemoveMaterial`, `SetAppearance` and
  `RemoveAppearance`, with events `MaterialChanged` and `AppearanceChanged`. Fallback if the S01
  owner declines: one extension record `ExtKey { kind: "amg.materials", id: 0 }` with the same
  JSON, which needs no S01 change but costs a parse per access (cached by `Revision`).
- **JSON in `design.json`** uses serde with canonical `f64` (typed quantities serialize
  transparently), so it round-trips bit-exactly. TOML unit strings are only for libraries.
- **Default material.** A body without a material uses the preference "default material",
  `amg.builtin/pla.fff` by default (S17 open question 1) for mass properties. FEA refuses an
  unassigned body with a clear diagnostic and does not apply a default.
- **Per-face appearance overrides (MAT-02)** need `EntityRef` face selectors (DOC-06), so they
  are an additive `BodyProps` field. Mesh bodies use S07 face-group ids. Multi-material 3MF
  per-face filament (QUESTIONS #2 consequence) maps from these overrides.

## 6. Appearance model

- **Core (MAT-01).** `base_color: Srgba8` (authored in sRGB, as in 3MF), `metallic`, `roughness`,
  following the glTF 2.0 metallic-roughness model. 3MF export maps it 1:1:
  - `base/@displaycolor` = `#RRGGBBAA`;
  - `m:pbmetallic/@metallicness` and `@roughness`.
  Import is the inverse.
- **Full (MAT-02).**
  - Adds `emissive` (linear RGB), `transmission` + `ior` (glTF `KHR_materials_transmission` /
    `KHR_materials_ior`; 3MF `m:translucent` attenuation/refractiveindex/roughness), optional
    `clearcoat`, and texture references (base colour, normal, ORM) with a triplanar or UV flag
    for RND-10.
  - Serde defaults keep MAT-01 files valid.
  - OpenPBR (ASWF, Apache-2.0) was considered as the parameter set. It is richer, but neither
    3MF nor glTF carries it, and RND-10 is P2, so glTF MR is the interchange-safe choice.
- Colour math lives in `amg-materials`: sRGB↔linear (IEC 61966-2-1 piecewise curve) and
  `to_linear_f32()` for the render adapter. `amg-render` never depends on `amg-materials`; the
  `amg-ui` adapter copies plain floats into `RenderScene`.

## 7. Prior art consulted

- 3MF Materials and Properties Extension v1.2.1 (namespace
  `http://schemas.microsoft.com/3dmanufacturing/material/2015/02`): `basematerials`/`base`
  (name, displaycolor), `colorgroup`, `pbmetallicdisplayproperties` (metallicness default 0,
  roughness default 1), `pbspeculardisplayproperties`, `translucentdisplayproperties`.
  `displaycolor` is sRGB `#RRGGBB[AA]`.
- OrcaSlicer `src/libslic3r/MaterialType.cpp`: filament type vocabulary and default
  temperature ranges.
- Abaqus/CalculiX engineering-constants convention; Lempriere (1968) admissibility bounds;
  Tsai & Hahn (1980) for the Tsai-Wu interaction estimate.
- Literature on FFF anisotropy: transversely isotropic and orthotropic PLA identification
  studies report E 2.6–2.9 GPa, G 0.9–1.2 GPa and ν 0.20–0.32. Others report that inter-layer
  anisotropy dominates in-layer anisotropy. This supports transverse isotropy about the build
  axis as the P2 model, with `Orthotropic` available.
- Fusion 360 material behaviour (copy into design, appearance vs physical material, default
  material preference). Only the behaviour is copied, no data.
- SpoolmanDB, Open Filament Database, OpenPrintTag: open community filament data (§3.1).
