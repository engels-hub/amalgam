# Units and quantities (`amg_core::units`)

Linked from [S00 — Foundation](../specs/S00-foundation.md) (WP FND-10). Written 2026-10-19
during S00 refinement. Consumers: S01 (parameters/expressions, DOC-04), S11 (input fields),
S12 (file unit handling, IO-01), S13 (FEA loads/results), S17 (material properties, MAT-01).

## 1. Problem

Every crate needs to agree on (a) the unit a raw `f64` is in, (b) how a user-typed value like
`1/4 in` or `30 deg` becomes a number, and (c) how a number is displayed again. The vision
fixes "millimetre and radian internally, SI for everything else". That rule is incomplete: with
millimetres as the length unit, the plain SI units for force, stress and density are no
longer *coherent*. Multiplying two stored values then gives a result in an unexpected unit.
S01 expressions (`F * d`, `P * A`) and S13 FEA assembly both multiply stored values, so the
system has to be coherent.

## 2. Decision: coherent mm–t–s–K–rad base

Canonical (stored) values use the **coherent system built on mm, tonne, second, kelvin,
radian**. This is the standard "mm system" of structural FEA codes (Abaqus, CalculiX, Ansys
"nmm"), so FEA input decks and literature values convert one-to-one.

| Quantity | Dimension (L, M, T, Θ, A) | Canonical unit | Note |
|---|---|---|---|
| Length | (1,0,0,0,0) | mm | matches geometry `f64` |
| Area | (2,0,0,0,0) | mm² | |
| Volume | (3,0,0,0,0) | mm³ | |
| Angle | (0,0,0,0,1) | rad | angle is a tracked dimension, see §4 |
| Mass | (0,1,0,0,0) | t (tonne = 1000 kg) | only mass is unusual; UI shows g/kg |
| Time | (0,0,1,0,0) | s | |
| Force | (1,1,−2,0,0) | N | t·mm/s² = N exactly |
| Pressure / stress | (−1,1,−2,0,0) | MPa (N/mm²) | |
| Density | (−3,1,0,0,0) | t/mm³ | steel 7850 kg/m³ = 7.85e−9 t/mm³ |
| Energy | (2,1,−2,0,0) | mJ (N·mm) | |
| Temperature | (0,0,0,1,0) | K | °C/°F are affine, see §5 |

Result: the only place a user-facing SI unit differs from storage is mass (t instead of kg).
Typed accessors (`Mass::kg()`) hide it. **Raw `f64` geometry stays plain mm/rad.** Points and
vectors are not wrapped in `Length`. Quantities appear at the boundaries where units matter:
parameters, dialogs, file import scale, material properties, FEA loads/results.

This refines the vision's "SI for everything else" wording rather than contradicting it.
Because it touches S13/S17, it is listed as an owner question in S00 (open question 4).

## 3. Types

- `Dimension` — five `i8` exponents (length, mass, time, temperature, angle). Multiplication
  adds exponents; `pow(i8)` scales them. `Display` renders `mm^2`-style canonical units.
- `Quantity { value: f64, dim: Dimension }` — dynamically dimensioned. Used by the S01
  expression evaluator. `+`/`−` require equal dimensions (`Err(DimensionMismatch)`); `*`/`/`
  combine them.
- Typed newtypes (`Length`, `Area`, `Volume`, `Angle`, `Mass`, `Time`, `Force`, `Pressure`,
  `Density`, `Energy`, `Temperature`) are `#[repr(transparent)]` over the canonical `f64`. They
  have named constructors and accessors (`Length::inches(0.25)`, `p.mpa()`) and implement
  `From<T> for Quantity` and `TryFrom<Quantity>`. They are for APIs, not for arithmetic-heavy
  inner loops.
- `Unit` — a closed enum of display/parse units (table §6). Each variant knows its
  `Dimension`, scale to canonical, offset (temperature only) and canonical symbol.

## 4. Angles as a dimension

Strict SI treats the radian as dimensionless. Fusion's parameter checker does not: it rejects
`10 mm + 5 deg`. We follow Fusion and track angle as a fifth exponent. Two consequences:

- `sin`, `cos` and `tan` take an angle; `asin`, `acos` and `atan2` return one (S01 enforces this).
- Arc length `r * θ` has dimension L·A. S01 provides `rad_to_unitless()` semantics: an
  explicit conversion function in expressions. Implicit stripping is not allowed. This matches
  Fusion, which requires `r * θ / 1 rad`-style workarounds.

## 5. Temperature

Canonical K. `degC` and `degF` are affine (offset + scale). Rules:
- Parsing `20 degC` gives an absolute temperature: 293.15 K.
- Arithmetic on `Quantity` treats every temperature as absolute K. Temperature *differences*
  (thermal expansion ΔT, S13) are entered in K or `delta_degC`. `delta_degC` is a separate
  unit with scale 1 and offset 0, so `ΔT = 10 delta_degC` does not become 283.15.
- The formatter never applies an offset to a value whose dimension is not exactly Θ¹.

## 6. Unit table (FND-10 implements exactly this)

Symbols are case-sensitive. Aliases are accepted on parse. The first symbol in each row is
the one printed.

| Dimension | Unit variant | Symbols (print first) | Scale to canonical |
|---|---|---|---|
| Length | `Micrometre` | `um`, `µm`, `μm` | 1e−3 |
| | `Millimetre` | `mm` | 1 |
| | `Centimetre` | `cm` | 10 |
| | `Metre` | `m` | 1000 |
| | `Inch` | `in`, `"`, `inch` | 25.4 |
| | `Foot` | `ft`, `'`, `foot`, `feet` | 304.8 |
| | `Thou` | `mil`, `thou` | 0.0254 |
| Angle | `Radian` | `rad` | 1 |
| | `Degree` | `deg`, `°` | π/180 |
| | `Gradian` | `grad` | π/200 |
| | `Turn` | `rev`, `turn` | 2π |
| Area | `SquareMillimetre` … | `mm^2`, `mm²`; `cm^2`; `m^2`; `in^2` | 1, 100, 1e6, 645.16 |
| Volume | `CubicMillimetre` … | `mm^3`, `mm³`; `cm^3`, `cc`; `m^3`; `L`, `l`; `mL`, `ml`; `in^3` | 1, 1e3, 1e9, 1e6, 1e3, 16387.064 |
| Mass | `Gram`, `Kilogram`, `Tonne`, `Pound`, `Ounce` | `g`; `kg`; `t`; `lb`, `lbm`; `oz` | 1e−6, 1e−3, 1, 4.5359237e−4, 2.8349523125e−5 |
| Time | `Millisecond`, `Second`, `Minute`, `Hour` | `ms`; `s`; `min`; `h` | 1e−3, 1, 60, 3600 |
| Force | `Newton`, `Kilonewton`, `PoundForce` | `N`; `kN`; `lbf` | 1, 1e3, 4.4482216152605 |
| Pressure | `Pascal`, `Kilopascal`, `Megapascal`, `Gigapascal`, `Bar`, `Psi`, `Ksi` | `Pa`; `kPa`; `MPa`, `N/mm^2`; `GPa`; `bar`; `psi`; `ksi` | 1e−6, 1e−3, 1, 1e3, 0.1, 6.894757293168e−3, 6.894757293168 |
| Density | `KilogramPerCubicMetre`, `GramPerCubicCentimetre` | `kg/m^3`; `g/cm^3`, `g/cc` | 1e−12, 1e−9 |
| Energy | `Millijoule`, `Joule` | `mJ`, `N*mm`; `J` | 1, 1e3 |
| Temperature | `Kelvin`, `DegreeCelsius`, `DegreeFahrenheit`, `DeltaCelsius` | `K`; `degC`, `°C`; `degF`, `°F`; `delta_degC` | 1 (+0), 1 (+273.15), 5/9 (+459.67·5/9), 1 |

Composite symbols such as `kg/m^3` are **atoms looked up in this table**. They are not
parsed as expressions. Arbitrary unit algebra (`N*m/s`) belongs to the S01 expression
grammar, which can combine `Quantity` values. Adding a unit means adding a row and a test.

## 7. Grammar of a single quantity literal

```
quantity  := ws sign? magnitude (ws unit)? ws EOF
           | ws sign? feet-inch ws EOF
magnitude := mixed | fraction | decimal
mixed     := uint ws1 uint "/" uint          ; "1 1/4"  (whole part + proper fraction)
fraction  := uint "/" uint                   ; "1/4"    (denominator ≠ 0)
decimal   := digits ("." digits?)? exp? | "." digits exp?
exp       := ("e" | "E") sign? digits
feet-inch := magnitude ws ("'" | "ft") ws1 magnitude ws ("\"" | "in")    ; 5' 3 1/2"
sign      := "+" | "-"
ws        := (" " | "\t")*        ; ws1 = at least one
unit      := longest match from the unit table (§6)
```

- A missing unit returns `ParsedQuantity { value, unit: None }`. The caller applies its
  context default: the document length unit for length fields, `deg` for angle fields. This
  matches Fusion, where `10` in a length field means "10 of the document unit".
- `feet-inch` is the only two-term form and is accepted because it is a single conventional
  notation. `1 m 20 cm` and every other compound form belong to S01 expressions.
- The decimal separator is `.` only. A `,` produces `ParseQuantityError::DecimalComma` with
  a hint. Locale-aware input is an owner question (S00 open question 5).
- Errors carry a byte span into the input for UI highlighting.

## 8. Formatting

`format_quantity(q, unit, &FormatSpec)` with `FormatSpec { style, precision, trim_zeros,
show_unit }`:
- `Style::Decimal` prints fixed `precision` decimals (default 3 for mm, 2 for deg), with
  optional trailing-zero trimming. `-0` is normalised to `0`.
- `Style::Fraction { max_denominator }` prints inch-style mixed fractions (`1 1/4 in`). Values
  that need a larger denominator fall back to decimal.
- `Style::FeetInch { max_denominator }` prints `5' 3 1/2"`.

**Round-trip contract (property-tested):** for any finite canonical value `v` and unit `u`,
`parse(format(v, u, spec))` converted to canonical is within `0.5 · 10^−precision · scale(u)`
of `v`. With the `Style::Exact` spec (shortest round-trip `f64` formatting via `{}`), the
result equals `v` bit-for-bit. `.amg` files and the Python API use `Style::Exact`.

## 9. Prior art consulted

- `uom` crate: compile-time dimensional analysis. Rejected as the primary type because its
  heavy generics slow compiles and do not fit runtime-typed expressions (S01). Its unit
  constants and conversion factors were used to cross-check table §6.
- Fusion 360 parameter/expression behaviour (units in literals, angle checking, document
  default unit, `in`/`"` aliases): Autodesk help "Units in expressions".
- NIST SP 811 (2008) for exact conversion factors (inch 25.4 mm exact, lbf, psi).
- CalculiX/Abaqus "consistent units" tables for the mm–t–s system.
