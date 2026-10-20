# S17 — Materials & appearances

| | |
|---|---|
| **Status** | Refined |
| **Priority** | P0 (basic) / P1 (full) / P2 (anisotropic printed models) |
| **Crates** | `amg-materials`, `amg-ui::materials` (material browser and editor); additive changes in `amg-core::units` (MAT-01) and `amg-doc` (MAT-06), flagged as interface changes |
| **Lane** | G |
| **Depends on** | S00 (FND-10 units, agreed repoint from FND-03); S01 (DOC-05 commands, DOC-07 `.amg` container, DOC-06 for per-face overrides); S11 (UI-01, UI-04, UI-07) for UI |
| **Parity** | Data & other: "Physical materials + appearances library" (P0 basic / P1 full); "Export 3MF with colors/materials" (P0, with S12) |

## Summary
Physical materials and appearances in user-editable libraries:
- physical: density, elastic, strength, thermal, plasticity and AM process data;
- printing materials: PLA, PETG, ABS, ASA, PA12, PA-CF, TPU, standard and tough resins,
  Ti-6Al-4V, AlSi10Mg and 316L, plus reference wrought metals;
- appearances: PBR parameters and colours.

Every built-in value cites its source. Materials are assigned per body and copied into the
document, so a design opens with identical results anywhere. They are used by:
- mass properties (S05);
- FEA (S13) and generative design (S14);
- print setup and slicer hand-off (S16);
- 3MF colours and materials (S12);
- rendering (S10).

Property model, provenance policy, file format and anisotropy conventions are in the design note
[`docs/notes/materials-data.md`](../notes/materials-data.md). Units are the coherent mm–t–s–K–rad
system of [`docs/notes/units-and-quantities.md`](../notes/units-and-quantities.md).

## Scope
**In:**
- `amg-materials`:
  - material model: `Material`, three elastic models (isotropic, transversely isotropic,
    orthotropic), strength, plasticity, thermal and AM data, and per-value provenance;
  - appearance model: glTF-style metallic-roughness, plus transmission, clearcoat and
    textures at MAT-02;
  - TOML library format v1 with unit strings, migrations, validation;
  - the embedded built-in library;
  - user libraries (directory of TOML files, atomic save);
  - `LibrarySet` search and name matching;
  - library drift detection;
  - anisotropic stiffness math and directional failure inputs (MAT-04).
- Document integration (MAT-06):
  - material and appearance tables inside `Design`, as an additive S01 change;
  - commands;
  - assignment helper;
  - default-material preference.
- `amg-ui::materials`: browser (MAT-03) and editor / user library management (MAT-08).

**Out:**
- infill, print settings and the build direction of a body: S16 print setup (a material
  describes fully dense printed material);
- the 3MF XML itself: S12 (S17 provides colour/PBR values and name matching);
- shaders and environment lighting: S10 (RND-10);
- FEA element formulation and failure-criterion selection: S13;
- fatigue, creep, viscoelastic and temperature-dependent property curves: later (S19);
- reading proprietary libraries (Fusion `.adsklib`, MatWeb, CAMPUS): not planned.

## Design

### Decisions (evidence in the design note)
1. **Property set** = the union of consumer needs (note §1):
   - density (required);
   - optional `Mechanical`: elastic model, strength, elongation, hardness, plasticity;
   - optional `Thermal`: conductivity, specific heat, CTE + reference temperature, Tg, melting,
     HDT, max service temperature;
   - optional `AmInfo`: ISO/ASTM 52900 process, Orca `filament_type` string, temperature
     ranges, shrinkage, cost per kg.
2. **Anisotropy.** Material axis 3 is the layer normal (build direction).
   - `Elastic::TransverselyIsotropic` (5 constants) is the P2 FFF model asked for in
     QUESTIONS #3. `Orthotropic` (9 constants) is available. Resins and metals are isotropic.
   - Directional values (`Directional { xy, z }`) carry inter-layer strengths and CTE.
   - Engineering-constant conventions follow Abaqus/CalculiX. Admissibility follows
     Lempriere (1968).
   - The material frame is mapped to the body by S16's build direction. The material never
     stores a body orientation.
   - The serialized schema for all three models is fixed in format v1, so MAT-04 needs no
     migration.
3. **Isotropic reduction is explicit.** A P1 isotropic solver calls
   `Elastic::isotropic_equivalent(IsoPolicy)`, and the S13 UI warns. Nothing is silently dropped.
4. **Units.**
   - Library files store strings with units (`"1.24 g/cm^3"`), parsed by
     `amg_core::parse_quantity` and dimension-checked. Bare numbers are rejected for
     dimensioned properties.
   - In memory, values are FND-10 typed quantities (canonical mm–t–s–K). `design.json` stores
     canonical `f64` and is bit-exact.
   - MAT-01 adds thermal-transport units, `ppm/K`, `%` and power to `amg_core::units`. This is
     additive; note §2.5 has the table.
5. **Provenance.**
   - Every built-in value has `{source, quality, specimen orientation (ISO/ASTM 52921), test
     standard, note}`.
   - Values come from individually cited datasheets, standards, handbooks or papers. Nothing is
     bulk-copied from MatWeb, CAMPUS, Granta or the Fusion library (EU database right; note §3.1).
   - Only `Z**`-oriented specimens may fill a `z` slot. This catches the "vertical (XZ)"
     datasheet pitfall.
6. **Documents are self-contained.**
   - Assigning a material copies it into the design's material table. `BodyProps.material`
     (S01) is a `MaterialKey` into that table.
   - Library changes never alter a document silently. Drift is detected per material through
     `(id, revision, content hash)` and fixed by an explicit "Update from library" command.
7. **Ids.**
   - `MaterialId` = `"<library-id>/<local-id>"`, for example `amg.builtin/pla.fff`.
   - User libraries start with `user.`.
   - Materials created only inside a document use `doc/<local>`.
   - The same scheme applies to `AppearanceId`.
8. **Appearance.**
   - sRGB 8-bit base colour as authored, plus `metallic` and `roughness`, which map 1:1 to 3MF
     `displaycolor` and `pbmetallic`.
   - MAT-02 adds glTF `KHR_materials_transmission`/`ior`, clearcoat, emissive and texture
     references, all with serde defaults.
   - `amg-render` gets plain linear floats through the `amg-ui` adapter (no crate dependency).
9. **Default material.** Unassigned bodies use the preference `default_material` for mass
   properties, default `amg.builtin/pla.fff` (AM focus; open question 1). FEA refuses
   unassigned bodies.
10. **Threading.** The built-in library parses lazily (`OnceLock`) in < 5 ms. User libraries
    load off the UI thread as a Background job at startup. `Material`/`Appearance` are
    immutable `Arc`s (`Send + Sync`).

### Built-in library v1 (MAT-05)
About 17 materials. The candidate sources per material are in note §3.3:
- FFF (process `material_extrusion`): `pla.fff`, `petg.fff`, `abs.fff`, `asa.fff`,
  `pa12.fff`, `pa-cf.fff`, `tpu-95a.fff`;
- polymer powder bed: `pa12.pbf`;
- resin (vat photopolymerisation): `resin.standard`, `resin.tough`;
- LPBF metals: `ti6al4v.lpbf`, `alsi10mg.lpbf`, `316l.lpbf`;
- wrought reference metals: `steel.s235`, `aluminium.6061-t6`, `stainless.1.4404`,
  `ti6al4v.wrought`.

Each material has a default appearance. FFF entries are `Isotropic` (XY values) with
directional strengths where the TDS gives Z specimens. MAT-04 upgrades them to
`TransverselyIsotropic` with a revision bump. Plausibility bounds per family (note §3.4) are
test-enforced.

### Document integration (MAT-06)
- `Design` gains two persistent tables and four commands (additive; see Interfaces
  §`amg-doc` additions).
- `assign_material(design, bodies, &Arc<Material>) -> Vec<Command>` builds one transaction:
  1. upsert the copy if absent or different;
  2. `SetBodyProps` for each body;
  3. set the material's default appearance on bodies without an explicit appearance.
- It lives in `amg-doc` next to the commands so UI, CLI and Python (S18) share it.
- `RemoveMaterial` is refused while referenced. "Remove unused" is a separate command.
- Fallback if the S01 owner declines typed tables: a single extension record
  `ExtKey { kind: "amg.materials", id: 0 }` carrying the same JSON (note §5). The public
  `amg-materials` API is identical either way.

### UI (`amg-ui::materials`, MAT-03 / MAT-08)
- **Browser panel** has three sections: "In this design", "Built-in" and one per user library.
  - Search and filters: category, process, family.
  - Property sheet: values in the user's display units (FND-10 formatter); a provenance tooltip
    per value (source title, quality, specimen, standard); a warning glyph for `estimate`.
  - Drift badge with "Update from library".
- **Assignment:** "Assign to selection" (UI-04 selection), drag-drop onto browser-tree bodies
  (UI-07) and onto viewport bodies (RND-04 pick on drop).
- **Editor:** duplicate to a user library, edit with unit-aware fields and live validation
  (same rules as the loader), create and rename user libraries.
- Phase 1 has no browser. Assignment comes from Python/CLI and a minimal "Physical material"
  combo in the body context menu, built by MAT-06 on UI-07, or hand-built like MESH-23
  (QUESTIONS #22).

## Interfaces
Provisional Rust signatures. `amg-materials` depends only on `amg-core` (plus `serde`,
`serde_json`, `toml`, `blake3`, `thiserror`), as the architecture table requires.

```rust
// ───────────────────── amg_core::units additions (MAT-01, additive to FND-10) ─────────────
impl Dimension {
    pub const THERMAL_CONDUCTIVITY: Dimension; // (1, 1,-3,-1, 0)  W/(m·K) = canonical exactly
    pub const SPECIFIC_HEAT: Dimension;        // (2, 0,-2,-1, 0)  1 J/(kg·K) = 1e6 canonical
    pub const THERMAL_EXPANSION: Dimension;    // (0, 0, 0,-1, 0)
    pub const POWER: Dimension;                // (2, 1,-3, 0, 0)  mW canonical
}
// New `Unit` variants: WattPerMetreKelvin `W/(m*K)` `W/mK`; JoulePerKilogramKelvin `J/(kg*K)`
// `J/kgK`; PerKelvin `1/K`; PpmPerKelvin `ppm/K` `um/(m*K)`; Milliwatt `mW`; Watt `W`;
// Percent `%` (dimensionless, 0.01). Typed quantities (same macro as FND-10):
// ThermalConductivity [w_per_m_k], SpecificHeat [j_per_kg_k], ThermalExpansion [per_k, ppm_per_k].

// ───────────────────────────── amg_materials (lib.rs re-exports) ─────────────────────────────
use std::{collections::BTreeMap, path::{Path, PathBuf}, sync::Arc};
use amg_core::units::{Density, Pressure, Temperature, ThermalConductivity, SpecificHeat,
                      ThermalExpansion, Volume, Mass};

// ── ids ──
/// "<library-id>/<local-id>", both `[a-z0-9._-]+`. Library ids: "amg.builtin", "user.<name>", "doc".
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct MaterialId(Arc<str>);
impl MaterialId {
    pub fn new(library: &str, local: &str) -> Result<MaterialId, MaterialError>;
    pub fn parse(s: &str) -> Result<MaterialId, MaterialError>;
    pub fn library(&self) -> &str;
    pub fn local(&self) -> &str;
    pub fn as_str(&self) -> &str;
}
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct AppearanceId(Arc<str>);              // same API as MaterialId
/// Names used by S01 `BodyProps` (`material: Option<MaterialKey>`, `appearance: Option<AppearanceKey>`).
pub type MaterialKey = MaterialId;
pub type AppearanceKey = AppearanceId;

// ── physical material ──
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Material {
    pub id: MaterialId,
    pub revision: u32,
    pub name: String,
    pub category: Category,
    pub family: Option<MaterialFamily>,
    /// "printed, 100 % infill, dry, 23 degC", "post-cured 60 min 60 degC", "stress relieved".
    pub condition: Option<String>,
    pub description: Option<String>,
    pub density: Density,
    pub mechanical: Option<Mechanical>,
    pub thermal: Option<Thermal>,
    pub am: Option<AmInfo>,
    /// Default appearance applied on assignment when the body has none.
    pub appearance: Option<AppearanceId>,
    pub tags: Vec<String>,
    pub provenance: Provenance,
}
impl Material {
    pub fn validate(&self) -> Result<(), MaterialError>;          // all rules in note §2.1–2.4
    /// blake3 over canonical JSON of everything except `provenance` (values only).
    pub fn content_hash(&self) -> [u8; 32];
    pub fn mass_of(&self, v: Volume) -> Mass;
    /// Errors with `MissingProperty` if `mechanical` is absent.
    pub fn elastic(&self) -> Result<&Elastic, MaterialError>;
    pub fn is_anisotropic(&self) -> bool;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Category { Polymer, PolymerComposite, Elastomer, Photopolymer, Metal, Ceramic, Other }

#[derive(Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum MaterialFamily {
    Pla, Petg, Pctg, Abs, Asa, Pc, Pa6, Pa11, Pa12, PaCf, PaGf, PlaCf, PetgCf, Tpu,
    ResinStandard, ResinTough, ResinFlexible, ResinHighTemp,
    Ti6Al4V, AlSi10Mg, Ss316L, In718, MaragingSteel,
    CarbonSteel, StainlessSteel, AluminiumAlloy, TitaniumAlloy,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Mechanical {
    pub elastic: Elastic,
    pub strength: Strength,
    pub elongation_at_break: Option<Directional<f64>>, // fraction (0.12 = 12 %)
    pub hardness: Option<Hardness>,
    pub plasticity: Option<Plasticity>,
}

/// Material axes 1,2,3; axis 3 = layer normal (build direction) for printed materials.
/// ν_ij = −ε_j/ε_i under σ_i (Abaqus/CalculiX engineering constants).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "model", rename_all = "snake_case")]
pub enum Elastic {
    Isotropic { youngs: Pressure, poisson: f64 },
    /// Isotropic in the 1-2 (layer) plane. G12 = e_p / (2(1+nu_p)) derived; G13 = G23 = g_pt.
    TransverselyIsotropic { e_p: Pressure, e_t: Pressure, nu_p: f64, nu_pt: f64, g_pt: Pressure },
    /// e = [E1,E2,E3], nu = [ν12,ν13,ν23], g = [G12,G13,G23].
    Orthotropic { e: [Pressure; 3], nu: [f64; 3], g: [Pressure; 3] },
}
/// Voigt index order used by `stiffness_voigt`/`compliance_voigt` (engineering shear strains).
pub const VOIGT_ORDER: [(u8, u8); 6] = [(1, 1), (2, 2), (3, 3), (2, 3), (1, 3), (1, 2)];
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineeringConstants { pub e: [Pressure; 3], pub nu: [f64; 3], pub g: [Pressure; 3] }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IsoPolicy {
    /// min E_i, in-plane ν, min of directional strengths.
    Conservative,
    /// Layer-plane values.
    InPlane,
}
impl Elastic {
    pub fn validate(&self) -> Result<(), String>;                 // Lempriere bounds (note §2.1)
    pub fn is_isotropic(&self) -> bool;
    /// Some((E, ν)) only for `Isotropic` — what P1 solvers call.
    pub fn as_isotropic(&self) -> Option<(Pressure, f64)>;
    pub fn isotropic_equivalent(&self, policy: IsoPolicy) -> (Pressure, f64);
    pub fn engineering_constants(&self) -> EngineeringConstants;
    /// MAT-04. Canonical MPa entries, material frame.
    pub fn compliance_voigt(&self) -> [[f64; 6]; 6];
    pub fn stiffness_voigt(&self) -> [[f64; 6]; 6];
}
/// MAT-04: rotate a material-frame Voigt stiffness into the frame whose axes are the columns
/// of `r` (orthonormal). S13 passes the build-direction frame from the S16 print setup.
pub fn rotate_stiffness(c: &[[f64; 6]; 6], r: glam::DMat3) -> [[f64; 6]; 6];

/// `xy` = in-layer (axes 1-2); `z` = along axis 3. `z == None` ⇒ isotropic.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Directional<T> { pub xy: T, pub z: Option<T> }
impl<T: Copy + PartialOrd> Directional<T> {
    pub fn iso(v: T) -> Self;
    pub fn z_or_xy(&self) -> T;
    pub fn min(&self) -> T;
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Strength {
    pub yield_tensile: Option<Directional<Pressure>>,      // Rp0.2 for metals
    pub ultimate_tensile: Option<Directional<Pressure>>,
    pub ultimate_compressive: Option<Directional<Pressure>>,
    pub shear: Option<Pressure>,
    pub flexural: Option<Pressure>,
}
/// MAT-04: Tsai-Wu coefficients (canonical MPa^-1 / MPa^-2) for transversely isotropic data;
/// `f13` defaults to −0.5·sqrt(F11·F33) unless given.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TsaiWu { pub f1: f64, pub f3: f64, pub f11: f64, pub f33: f64, pub f44: f64, pub f66: f64, pub f12: f64, pub f13: f64 }
impl Strength { pub fn tsai_wu(&self, f13: Option<f64>) -> Result<TsaiWu, MaterialError>; }

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "scale", content = "value", rename_all = "snake_case")]
pub enum Hardness { ShoreA(f64), ShoreD(f64), Vickers(f64), Brinell(f64) }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "model", rename_all = "snake_case")]
pub enum Plasticity {
    Bilinear { yield_stress: Pressure, tangent_modulus: Pressure },
    /// (true plastic strain, true stress); strain strictly increasing, starts at 0.
    Table { points: Vec<(f64, Pressure)> },
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Thermal {
    pub conductivity: Option<Directional<ThermalConductivity>>,
    pub specific_heat: Option<SpecificHeat>,
    pub expansion: Option<Directional<ThermalExpansion>>,
    pub expansion_reference: Option<Temperature>,
    pub glass_transition: Option<Temperature>,
    pub melting: Option<Temperature>,               // solidus for alloys
    pub heat_deflection_045: Option<Temperature>,   // ISO 75 at 0.45 MPa
    pub max_service: Option<Temperature>,
}

/// ISO/ASTM 52900 process categories.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AmProcess {
    MaterialExtrusion, VatPhotopolymerization, PowderBedFusionPolymer, PowderBedFusionMetal,
    MaterialJetting, BinderJetting, DirectedEnergyDeposition,
}
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TempRange { pub min: Temperature, pub max: Temperature }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Cost { pub per_kg: f64, pub currency: String /* ISO 4217, e.g. "EUR" */ }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AmInfo {
    pub process: AmProcess,
    /// OrcaSlicer `filament_type` ("PLA", "PETG", "PA-CF", "TPU", …); None for resin/metal.
    pub slicer_type: Option<String>,
    pub nozzle_temp: Option<TempRange>,
    pub bed_temp: Option<TempRange>,
    pub chamber_temp: Option<TempRange>,
    /// Linear shrinkage fraction (SLS, resin compensation).
    pub shrinkage: Option<Directional<f64>>,
    pub cost: Option<Cost>,
}

// ── provenance ──
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct SourceKey(pub String);
/// Dotted property path, e.g. "density", "mechanical.elastic.youngs", "mechanical.strength.ultimate_tensile.z".
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct PropPath(pub String);
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Provenance {
    pub default_source: Option<SourceKey>,
    pub values: BTreeMap<PropPath, ValueProv>,
    /// Copies of the `Source` records used, so a document copy keeps its citations.
    pub sources: BTreeMap<SourceKey, Source>,
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ValueProv {
    pub source: Option<SourceKey>,
    pub quality: Quality,
    pub specimen: Option<SpecimenOrientation>,
    pub standard: Option<String>,                // "ISO 527-2", "ASTM D638"
    pub note: Option<String>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Quality { Measured, Derived, Typical, Estimate }
/// ISO/ASTM 52921 orthogonal orientation notation; first letter = specimen long (load) axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum SpecimenOrientation { Xyz, Xzy, Yxz, Yzx, Zxy, Zyx }   // serialized "XYZ", …
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Source {
    pub key: SourceKey, pub kind: SourceKind, pub title: String, pub publisher: Option<String>,
    pub version: Option<String>, pub date: Option<String>, pub url: Option<String>,
    pub accessed: Option<String>, pub licence: Option<String>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceKind { Datasheet, Standard, Handbook, Paper, CommunityDb, UserMeasured, Estimate }
impl Material { pub fn provenance_of(&self, p: &str) -> Option<&ValueProv>; }

// ── appearance ──
/// sRGB, straight alpha, as authored and as stored in 3MF (`#RRGGBBAA`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct Srgba8(pub [u8; 4]);
impl Srgba8 {
    pub fn from_hex(s: &str) -> Result<Srgba8, MaterialError>;   // "#RRGGBB" | "#RRGGBBAA"
    pub fn to_hex(self) -> String;                               // always "#RRGGBBAA"
    pub fn to_linear_f32(self) -> [f32; 4];                      // IEC 61966-2-1, alpha linear
    pub fn from_linear_f32(c: [f32; 4]) -> Srgba8;
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub struct Appearance {
    pub id: AppearanceId,
    pub revision: u32,
    pub name: String,
    pub base_color: Srgba8,
    pub metallic: f32,               // 0..=1
    pub roughness: f32,              // 0..=1
    // ---- MAT-02 (serde defaults keep v1 files valid) ----
    #[serde(default)] pub emissive: [f32; 3],          // linear RGB
    #[serde(default)] pub transmission: f32,           // KHR_materials_transmission
    #[serde(default = "ior_default")] pub ior: f32,     // 1.5
    #[serde(default)] pub clearcoat: Option<Clearcoat>,
    #[serde(default)] pub textures: Option<TextureSet>, // RND-10 (P2)
}
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Clearcoat { pub factor: f32, pub roughness: f32 }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TextureSet { pub base_color: Option<String>, pub normal: Option<String>,
                        pub orm: Option<String>, pub scale_mm: f64, pub triplanar: bool }
impl Appearance {
    pub fn new(id: AppearanceId, name: impl Into<String>, base_color: Srgba8) -> Appearance;
    pub fn validate(&self) -> Result<(), MaterialError>;
    pub fn content_hash(&self) -> [u8; 32];
    /// 3MF mapping helpers for S12 (strings/values only, no XML).
    pub fn to_3mf(&self) -> ThreeMfDisplay;
    pub fn from_3mf(id: AppearanceId, name: &str, d: &ThreeMfDisplay) -> Appearance;
}
#[derive(Clone, Debug, PartialEq)]
pub struct ThreeMfDisplay {
    pub displaycolor: String,                       // "#RRGGBBAA" sRGB
    pub pbmetallic: Option<(f32, f32)>,              // (metallicness, roughness)
    pub translucent: Option<ThreeMfTranslucent>,     // MAT-02
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThreeMfTranslucent { pub attenuation: [f32; 3], pub refractive_index: [f32; 3], pub roughness: f32 }

// ── libraries ──
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LibraryMeta { pub id: String, pub name: String, pub version: String,
                         pub data_licence: Option<String>, pub description: Option<String> }
pub const FORMAT_VERSION: u32 = 1;
#[derive(Clone, Debug)]
pub struct Library { /* meta, sources, materials, appearances: BTreeMap<_, Arc<_>>; path; read_only */ }
impl Library {
    pub fn new(meta: LibraryMeta) -> Library;
    pub fn from_toml_str(src: &str, origin: &str /* path or "builtin" for errors */) -> Result<Library, MaterialError>;
    pub fn to_toml_string(&self) -> String;
    pub fn load(path: &Path) -> Result<Library, MaterialError>;
    /// Atomic (tmp + fsync + rename). Err(ReadOnly) for the built-in library.
    pub fn save(&self) -> Result<(), MaterialError>;
    pub fn meta(&self) -> &LibraryMeta;
    pub fn is_read_only(&self) -> bool;
    pub fn path(&self) -> Option<&Path>;
    pub fn material(&self, id: &MaterialId) -> Option<&Arc<Material>>;
    pub fn materials(&self) -> impl Iterator<Item = &Arc<Material>> + '_;
    pub fn appearance(&self, id: &AppearanceId) -> Option<&Arc<Appearance>>;
    pub fn appearances(&self) -> impl Iterator<Item = &Arc<Appearance>> + '_;
    pub fn source(&self, key: &SourceKey) -> Option<&Source>;
    /// Validates; re-keys the id into this library; bumps `revision` if the content hash changed.
    pub fn upsert_material(&mut self, m: Material) -> Result<&Arc<Material>, MaterialError>;
    pub fn remove_material(&mut self, id: &MaterialId) -> Option<Arc<Material>>;
    pub fn upsert_appearance(&mut self, a: Appearance) -> Result<&Arc<Appearance>, MaterialError>;
    pub fn remove_appearance(&mut self, id: &AppearanceId) -> Option<Arc<Appearance>>;
    pub fn upsert_source(&mut self, s: Source);
}
/// The embedded, validated built-in library (lazy, parsed once).
pub fn builtin() -> &'static Library;
pub const BUILTIN_LIBRARY_ID: &str = "amg.builtin";
pub fn default_material_id() -> MaterialId;          // "amg.builtin/pla.fff"

#[derive(Clone, Debug, Default)]
pub struct MaterialQuery { pub text: Option<String>, pub category: Option<Category>,
                           pub process: Option<AmProcess>, pub family: Option<MaterialFamily> }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Drift { UpToDate, LibraryNewer { library_revision: u32 }, LocallyModified, NotInLibrary }

/// Built-in + user libraries. Not used for resolving a document's materials (the document copy is
/// authoritative); used for picking, search, import matching and drift.
pub struct LibrarySet { /* builtin: &'static Library, user: Vec<Library> */ }
impl LibrarySet {
    pub fn builtin_only() -> LibrarySet;
    /// Loads every `*.toml` in `dir` (created if missing). Bad files are skipped and returned.
    pub fn load_user_dir(&mut self, dir: &Path) -> Vec<MaterialError>;
    pub fn create_user_library(&mut self, dir: &Path, id: &str, name: &str) -> Result<&mut Library, MaterialError>;
    pub fn libraries(&self) -> impl Iterator<Item = &Library> + '_;
    pub fn library_mut(&mut self, id: &str) -> Option<&mut Library>;   // None for builtin
    pub fn material(&self, id: &MaterialId) -> Option<&Arc<Material>>;
    pub fn appearance(&self, id: &AppearanceId) -> Option<&Arc<Appearance>>;
    pub fn search(&self, q: &MaterialQuery) -> Vec<&Arc<Material>>;   // stable order: library, name
    /// Import matching (3MF `base/@name`, STEP names): case-insensitive exact name, then
    /// `slicer_type`, then family keyword. Built-in last, so user libraries win.
    pub fn match_name(&self, name: &str) -> Option<&Arc<Material>>;
    pub fn drift(&self, doc_copy: &Material) -> Drift;
}

// ── errors ──
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum MaterialError {
    #[error("{origin}: TOML syntax error: {message}")]
    Toml { origin: String, message: String, span: Option<std::ops::Range<usize>> },
    #[error("{origin}: `{path}`: {source}")]
    Quantity { origin: String, path: String, #[source] source: amg_core::units::ParseQuantityError },
    #[error("{origin}: `{path}`: expected {expected}, got {found}")]
    Dimension { origin: String, path: String, expected: amg_core::units::Dimension, found: amg_core::units::Dimension },
    #[error("{origin}: `{path}` needs a unit")]
    MissingUnit { origin: String, path: String },
    #[error("material `{id}`: invalid `{property}`: {reason}")]
    Invalid { id: String, property: String, reason: String },
    #[error("material `{id}` has no `{property}`")]
    MissingProperty { id: String, property: &'static str },
    #[error("invalid id `{0}`")] InvalidId(String),
    #[error("{origin}: duplicate id `{id}`")] Duplicate { origin: String, id: String },
    #[error("{origin}: unknown source `{key}`")] UnknownSource { origin: String, key: String },
    #[error("{origin}: format version {found} is newer than supported {supported}")]
    TooNew { origin: String, found: u32, supported: u32 },
    #[error("library `{0}` is read-only")] ReadOnly(String),
    #[error(transparent)] Io(#[from] std::io::Error),
}
impl amg_core::Diagnostic for MaterialError { /* hints: "write `1.24 g/cm^3`", "Z values need a Z** specimen" */ }
```

### `amg-doc` additions (MAT-06; additive change to S01, flagged)
```rust
// Design (authored state) gains two persistent tables:
//   materials:   PMap<MaterialId, Arc<amg_materials::Material>>
//   appearances: PMap<AppearanceId, Arc<amg_materials::Appearance>>
impl Design {
    pub fn material(&self, id: &MaterialId) -> Option<&Arc<Material>>;
    pub fn materials(&self) -> impl Iterator<Item = &Arc<Material>> + '_;
    pub fn appearance(&self, id: &AppearanceId) -> Option<&Arc<Appearance>>;
    pub fn appearances(&self) -> impl Iterator<Item = &Arc<Appearance>> + '_;
    /// BodyProps.material resolved against the document table; None if unassigned.
    pub fn body_material(&self, body: BodyId) -> Option<&Arc<Material>>;
    pub fn body_appearance(&self, body: BodyId) -> Option<&Arc<Appearance>>;
    /// Bodies whose props reference `id`.
    pub fn material_users(&self, id: &MaterialId) -> Vec<BodyId>;
}
// Command enum (additive variants):
//   SetMaterial { material: Arc<Material> }      // upsert; Err if validate() fails
//   RemoveMaterial { id: MaterialId }             // Err(CommandError::InUse) if referenced
//   SetAppearance { appearance: Arc<Appearance> }
//   RemoveAppearance { id: AppearanceId }
// DocEvent (additive): MaterialChanged(MaterialId), AppearanceChanged(AppearanceId)
/// One transaction's worth of commands: upsert copy (if absent/different) + SetBodyProps per body
/// (+ default appearance where the body has none).
pub fn assign_material(design: &Design, bodies: &[BodyId], m: &Arc<Material>) -> Vec<Command>;
pub fn assign_appearance(design: &Design, bodies: &[BodyId], a: &Arc<Appearance>) -> Vec<Command>;
/// Removes every table entry no body references (one command list).
pub fn remove_unused_materials(design: &Design) -> Vec<Command>;
```
Per-face overrides (MAT-02) need an additive `BodyProps` field
`face_appearances: Vec<(FaceSelector, AppearanceKey)>` (`#[serde(default)]`). `FaceSelector`
is `EntityRef` faces for solids (DOC-06) or S07 face-group ids for meshes. It is requested from
S01.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| MAT-01 | `amg-materials` core: ids, `Material` + all sub-types (three elastic models with validation, `Directional`, strength, plasticity, thermal, `AmInfo`), minimal `Appearance` + `Srgba8`, `Provenance`, TOML format v1 reader/writer (unit strings, inline provenance tables, migration hook, `TooNew`), `Library` (in-memory, load/save atomic), `builtin()` embedding with a 2-material fixture, `MaterialError` + `Diagnostic`. Additive `amg-core::units` rows and typed quantities from note §2.5 (PR flagged as S00 interface change). | FND-10 | P0 | Todo | Tests pass. (a) A fixture library using every property kind loads; load→save→load keeps all values within 2 ulp, and a second save is byte-identical. (b) `serde_json` round-trip of every `Material` is bit-exact. (c) Each validation rule in note §2.1–2.4 has a failing fixture giving the expected error with the TOML key path: ν = 0.5 isotropic, transversely isotropic `(1+ν_p)(1−ν_p−2ν_ptν_tp) ≤ 0`, orthotropic determinant ≤ 0, negative E, non-monotone plasticity table, `z` value with an `XZY` specimen. (d) `density = "1.24 MPa"` gives `Dimension` naming `material[0].density`. `density = "1.24"` gives `MissingUnit`. (e) An unknown `src` gives `UnknownSource`. `format_version = 2` gives `TooNew`. (f) New unit rows parse: `0.2 W/(m*K)`, `1800 J/(kg*K)` = 1.8e9 canonical, `68 ppm/K` = 6.8e-5, `12 %` = 0.12. (g) `Srgba8` hex round-trips for all 2^24 colours (alpha FF), and sRGB→linear→sRGB is identity for all 256 levels. (h) A 1000-material generated library parses in < 20 ms (criterion). `clippy -D warnings` is clean. |
| MAT-05 | Built-in library v1 data: the 17 materials of §Design "Built-in library v1" with cited `[[source]]` records, per-value provenance, conditions, AM info (Orca `slicer_type`, temperature ranges) and one default appearance each, plus ~12 generic appearances (plastics in common filament colours, brushed aluminium, steel, titanium, grey and clear resin) | MAT-01 | P0 | Todo | `builtin()` loads and validates. Provenance tests (note §3.2) pass: density, in-plane E and one tensile strength are `measured` for every material; no `estimate` on density; every `src` resolves; every material has a `condition`. Plausibility-bounds test (note §3.4) passes for every material. Each FFF material whose TDS reports Z specimens has `z` strengths. Every material's `appearance` resolves. The PR description lists 10 randomly chosen values (seeded) next to the page or table of the cited document, checked by the reviewer. No source is MatWeb, CAMPUS, Granta or the Fusion library. |
| MAT-07 | `LibrarySet`: user library directory (load, create, skip-and-report bad files), `library_mut` + `upsert`/`remove` with revision bump, `search`, `match_name` (3MF/STEP import), `drift` | MAT-01 | P0 | Todo | Tests pass. (a) A temp dir with 3 valid files and 1 broken file loads 3 libraries and returns exactly one error naming the broken file. (b) Editing a value via `upsert_material` bumps `revision` by 1; an unchanged upsert does not. (c) `save` with a simulated crash between write and rename (test hook) leaves the old file intact. (d) `match_name("PLA")`, `("pla")` and `("PETG Basic")` (via `slicer_type`/family) return the expected built-ins; a user material with the same name wins. (e) `drift` returns each of the 4 variants in a fixture. (f) `search` over 5000 materials < 1 ms median (criterion). (g) The built-in library rejects `library_mut`. |
| MAT-06 | Document integration (additive `amg-doc` change, flagged): `Design` material/appearance tables, 4 commands + 2 events, `body_material`/`body_appearance`/`material_users`, `assign_material`/`assign_appearance`/`remove_unused_materials`, "Update from library" command builder, `default_material` preference read by mass-property callers; minimal Phase 1 assignment (Python/CLI command + body context-menu combo if UI-07 is done, else deferred to MAT-03) | MAT-07, DOC-05, DOC-07 | P0 | Todo | Tests pass. (a) `assign_material` of a built-in to 3 bodies applied as one transaction; one undo restores both the props and the table. (b) Save→load→save of the `.amg` keeps byte-identical `design.json` including materials. Loading with an empty `LibrarySet` resolves all body materials. (c) `RemoveMaterial` of a referenced material gives `CommandError::InUse`; `remove_unused_materials` removes exactly the unreferenced ones. (d) Bumping the library revision gives `Drift::LibraryNewer`, and the update command replaces the copy, undoable. (e) `BodyPropsChanged`/`MaterialChanged` events fire minimally (no `BodyGeometryChanged`). (f) A `Design` with 100 materials clones in < 1 µs (persistent map). |
| MAT-02 | Full appearance: emissive, transmission/IOR, clearcoat, `TextureSet` refs; appearance library entries; `to_3mf`/`from_3mf` incl. `pbmetallic` and `translucent`; per-face appearance overrides (`BodyProps.face_appearances`, additive S01 request) for solids (`EntityRef`) and meshes (face groups); render adapter conversion in `amg-ui::materials` | MAT-06, DOC-06 | P1 | Todo | Tests pass. (a) MAT-01 appearance files still load unchanged. (b) `to_3mf`→`from_3mf` round-trips colour exactly and metallic/roughness within 1/255. (c) A per-face override on a fillet face survives an upstream sketch edit (DOC-06 resolution) in a golden test. (d) The mesh face-group override survives save/load. (e) The adapter output for a fixture appearance matches expected linear floats within 1e-6. |
| MAT-03 | Material browser (`amg-ui::materials`): "In this design" / built-in / user sections, search + filters, property sheet in display units with provenance tooltips and `estimate` warnings, drift badge + update, assign to selection, drag-drop onto browser-tree bodies and viewport bodies | UI-01, UI-04, UI-07, MAT-06 | P1 | Todo | Manual test script (in the PR) passes: assign PLA to 2 selected bodies, drag Ti-6Al-4V onto a third in the viewport, undo/redo each, and drift badge appears after editing the library file. The panel draws in < 2 ms per frame with 5000 materials (search runs only on input change). Every action goes through `Command`s (verified by an undo test). |
| MAT-08 | Material and appearance editor + user library management UI: duplicate to user library, edit with unit-aware fields (FND-10 parse/format) and live validation, provenance fields, colour picker, create/rename user libraries | MAT-03, MAT-07 | P1 | Todo | Manual test script passes: duplicate `petg.fff` into a new user library, change E in GPa, see validation reject ν = 0.6, save, restart, and the edited value is shown with `revision` 2. The library file diff contains only the changed lines. Invalid input never reaches `Library::upsert_*` (unit test on the form model). |
| MAT-04 | Anisotropic printed-material models: `compliance_voigt`/`stiffness_voigt` for all three models, `rotate_stiffness` (Bond transformation), `isotropic_equivalent`, `Strength::tsai_wu`; upgrade FFF built-ins to `TransverselyIsotropic` (Z modulus from TDS, ν_pt/G_pt from cited literature, revision bump); CalculiX `*ELASTIC, TYPE=ENGINEERING CONSTANTS` line export for S13 cross-checks | MAT-01, MAT-05 | P2 | Todo | Tests pass. (a) `C · S = I` within 1e-12 (relative) for random admissible materials (proptest). (b) Isotropic `C` equals the Lamé form. (c) A transversely isotropic `C` is invariant under rotation about axis 3; a 90° rotation about axis 1 swaps E2/E3 in `engineering_constants` of the rotated compliance. (d) Tsai-Wu gives failure index 1 at uniaxial `Xt`, `Xc`, `Zt`, `Zc` and `S`. (e) Every FFF built-in has `e_t ≤ e_p` and passes admissibility. (f) The exported CalculiX line matches a hand-checked fixture. S13 consumes it (FEM orthotropic WP). |

Phase placement for MAT-05..08 is proposed in the Refinement log. The roadmap is not edited
here.

## Testing
- **Unit tests** per module, with the cases in each WP's Done-when.
- **Property tests (`proptest`):**
  - TOML and JSON round-trips of random valid materials;
  - admissibility: random engineering constants accepted iff `compliance_voigt` is positive
    definite (Cholesky), from MAT-04;
  - `C·S = I`;
  - rotation invariants.
- **Data tests (MAT-05):** provenance completeness, plausibility bounds, source resolution and
  appearance resolution run in CI on `builtin()`.
- **Golden:** `tests/golden/materials/builtin.toml` holds the content hash of every built-in
  material, so data changes are visible in review. `AMG_BLESS=1` updates it, and a changed hash
  without a `revision` bump fails.
- **Integration (MAT-06):** `.amg` round-trip; open on a machine profile with no user library;
  undo of assignment.
- **Benchmarks:** library parse (1000 materials), search (5000), `content_hash`.

## Performance targets
| Operation | Target |
|---|---|
| `builtin()` first call (≈ 17 materials + 30 appearances) | < 5 ms (cold-start budget 2 s) |
| Parse a 1000-material library | < 20 ms; user libraries load as a Background job, never on the UI thread |
| `LibrarySet::search` over 5000 materials | < 1 ms median |
| `Material::content_hash` | < 10 µs |
| Browser panel frame (5000 materials) | < 2 ms (UI budget 16 ms) |
| `Design` clone with 100 materials | < 1 µs (persistent map) |

## Open questions
1. **Owner:** default material for unassigned bodies (mass properties only). It could be
   `PLA (FFF)`, recommended for the AM focus, or Steel as in Fusion. *Default:* PLA, changeable
   in preferences. Not blocking (MAT-06).
2. **Owner:** which resin brand and type do you print with on the MSLA printer, and which
   filament brands most often? MAT-05 then cites those TDS first, or adds them as extra
   entries. *Default:* the generic set in note §3.3. Not blocking.
3. **Owner:** default currency for material cost (`AmInfo::cost`, used by S16 estimates).
   *Default:* EUR. Not blocking.
4. **Owner:** licence of the built-in library data file. Amalgam's own licence is still open
   (ADR-012). *Default:* same as the code, with the per-value source citations kept. Not
   blocking.
5. **S01 owner (lane E):** accept typed `materials`/`appearances` tables + 4 commands in
   `Design` (MAT-06), or use the extension-record fallback? *Recommended:* typed tables. Not
   blocking: MAT-06 can start on the fallback.
6. Resolved in refinement: anisotropy representation (transversely isotropic about the build
   axis, orthotropic available; QUESTIONS #3), file format (TOML + unit strings), document
   embedding, appearance model (glTF MR core). See the Refinement log.

## Refinement log
- **2026-10-20 (S17 refinement):**
  - **Read.** AGENTS.md, vision, architecture, ADRs, roadmap, QUESTIONS (#2, #3, #10, #13
    and #22 are binding here), S00 (FND-10 units API), the units note, S01 (`BodyProps`,
    commands, extension records, `.amg` container), S05/S10/S12/S13/S14/S16 material uses.
  - **Dependency repoint.** MAT-01 now depends on **FND-10** (units), not FND-03. This is the
    agreed cross-spec repoint already recorded in S00. MAT-02's FND-03 dependency is replaced
    by MAT-06/DOC-06, since per-face overrides need persistent naming.
  - **Property set.** Derived from consumer needs (note §1).
    - S13 Q2 answer → transversely isotropic with the build axis as material axis 3, plus a
      directional failure criterion (Tsai-Wu or Z allowables).
    - S16 Q1 answer → Orca `filament_type` mapping and resin/FFF process split.
    - Owner: resin is isotropic.
    - Conventions taken from Abaqus/CalculiX, so values map one-to-one to CalculiX
      cross-checks.
  - **Units.** FND-10 lacks thermal transport units. MAT-01 adds them additively (note §2.5).
    Library files are strict about units, unlike expressions, because silent unit defaults in
    data files are a classic ×1000 error source.
  - **Sources researched.**
    - Bambu Lab PLA Basic TDS V2.0 (XY vs Z specimens, ISO 527).
    - Prusament PETG TDS v1.1. Its "vertical (XZ)" is an in-plane specimen, which led to the
      specimen-orientation rule.
    - EOS Ti64 Grade 5 MDS (heat-treated Rp0.2 945/965 MPa).
    - Formlabs Standard resin TDS (green vs post-cured).
    - Elegoo Standard resin TDS. Its modulus has no unit, so the policy needs per-value quality
      flags.
    - Eurocode values for reference metals.
    - SpoolmanDB (MIT, verified), Open Filament Database and OpenPrintTag (MIT stated, data
      licence not verified). These are cross-check sources only for now.
    - The licence policy follows from the EU database right: individual citations, no bulk
      extraction (note §3.1).
  - **Format.** TOML chosen over JSON for libraries: comments, diffs and hand editing, and
    `toml` is already a workspace dependency. JSON stays for the document copy (S01, bit-exact).
    Versioning works at three levels: `format_version` with a migration chain,
    `library.version`, and per-material `revision`.
  - **Appearance.** glTF 2.0 metallic-roughness core, because it maps 1:1 to 3MF
    `displaycolor` + `pbmetallic`. 3MF Materials Extension v1.2.1 was read: displaycolor is sRGB
    `#RRGGBB[AA]`, and pbmetallic defaults are metallicness 0 and roughness 1. OpenPBR was
    considered and rejected for now (no 3MF/glTF carriage).
  - **WP changes.**
    - MAT-01 rescoped as the crate core plus format; new MAT-05 (data), MAT-06 (document
      integration), MAT-07 (user libraries and `LibrarySet`) and MAT-08 (editor UI).
    - MAT-02 adds 3MF display helpers and per-face overrides. MAT-03 deps add UI-04, UI-07 and
      MAT-06. MAT-04 is the math plus data upgrade.
    - Proposed phases: MAT-01, MAT-05, MAT-07, MAT-06 in Phase 1 (lane G). IO-03 and AM-06 need
      per-body materials and colours in the document. MAT-02, MAT-03 and MAT-08 in Phase 3.
      MAT-04 in Phase 5.
  - **Cross-spec requests (in the report, not edited here):**
    - S00: unit rows (additive).
    - S01: typed tables, commands and `face_appearances` (additive).
    - S12: IO-03 should depend on MAT-06 for document colours/materials, and use
      `Appearance::to_3mf`/`from_3mf` + `LibrarySet::match_name`.
    - S05: INS-03 should depend on MAT-06 (body → material resolution) instead of MAT-01.
    - S16: print setup supplies the build direction used as material axis 3 and reads `AmInfo`.
    - S13: the orthotropic FEM WP depends on MAT-04, and P1 solvers use `Elastic::as_isotropic`
      or an explicit `isotropic_equivalent` with a warning.
    - Roadmap: lane G should own `amg-features::materials` if it is ever needed; currently not,
      because the helpers live in `amg-doc`.
  - **Status.** Draft → Refined.

## Interface changes
(none yet; the spec has not reached `Ready`)
