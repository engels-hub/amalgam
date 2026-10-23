//! DXF and SVG import into a [`Sketch`] (SK-11).
//!
//! [`import_dxf`] reads LINE, ARC, CIRCLE, ELLIPSE, LWPOLYLINE, POLYLINE and SPLINE entities
//! (R12 and R2000+); [`import_svg`] reads paths, shapes and groups through `usvg`, which
//! flattens every transform, `viewBox` and unit. Both create ordinary sketch entities with
//! [`EntityOrigin::Imported`]:
//!
//! - **Scale.** DXF coordinates are scaled by the declared `$INSUNITS` (same table as IO-08,
//!   `docs/notes/import-export-formats.md` section 7), by [`ImportOptions::unit`] when set, and
//!   by mm when the file is unitless. SVG user units are CSS pixels at
//!   [`ImportOptions::svg_dpi`] (96), so `width="100mm"` imports as 100 mm.
//!   [`ImportOptions::scale`] multiplies on top. Nothing is scaled silently: the choice is in
//!   [`ImportReport::unit`].
//! - **Layers.** Entities keep their layer name ([`ImportReport::layers`]); layers named in
//!   [`ImportOptions::construction_layers`] (and hidden DXF layers) become construction
//!   geometry, layers in [`ImportOptions::skip_layers`] are dropped. SVG layers are the
//!   outermost enclosing `<g id>`.
//! - **Endpoint merge.** Free ends within [`ImportOptions::merge_tol`] (default 1e-4 mm, below
//!   the 1e-2 mm "near gap" warning of profile detection) are snapped together and joined by a
//!   `Coincident` constraint. Bigger gaps are never healed (QUESTIONS #17); raise the
//!   tolerance for an opt-in heal.
//! - **Curves.** Lines, arcs and circles stay as such; DXF ellipses and splines map to the
//!   SK-22/SK-08 entities. SVG Beziers become degree 3 (2) B-splines, except cubics that are
//!   circular arcs (the <= 90 degree pieces every exporter writes) and closed four-cubic rings
//!   that are affine circles, which become arcs, circles and ellipses.

mod dxf_in;
mod svg_in;

use std::collections::{BTreeMap, HashMap};
use std::f64::consts::TAU;
use std::path::Path;

use amg_core::{Aabb2, LengthUnit, Point2, Tolerance, Unit, Vec2};

use crate::error::SketchError;
use crate::geom::{EllipseArc2, chord_params, clamped_uniform_knots};
use crate::ids::{ConstraintId, EntityId};
use crate::model::{ConstraintKind, End, EntityKind, EntityOrigin, ParamFlags, Sketch, SplineDef};

/// Import failures. Per-entity problems are not errors: they are counted in
/// [`ImportReport::skipped`] and described in [`ImportReport::warnings`].
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ImportError {
    /// The DXF file could not be read.
    #[error("cannot read DXF: {0}")]
    Dxf(String),
    /// The SVG file could not be parsed.
    #[error("cannot read SVG: {0}")]
    Svg(String),
    /// A file could not be read from disk.
    #[error("cannot read {path}: {message}")]
    Io {
        /// The file.
        path: String,
        /// The OS error.
        message: String,
    },
    /// An option is outside its domain.
    #[error("invalid import option: {0}")]
    Options(&'static str),
    /// A sketch operation failed.
    #[error(transparent)]
    Sketch(#[from] SketchError),
}

/// Import settings. `Default` is: declared units (mm if none), scale 1, no construction
/// layers, merge tolerance 1e-4 mm, 96 dpi for SVG.
#[derive(Clone, Debug, PartialEq)]
pub struct ImportOptions {
    /// Overrides the file's unit: DXF coordinates are read as this unit; SVG user units
    /// (CSS px) are read as this unit. `None`: DXF `$INSUNITS` if set, else mm; SVG px at
    /// [`svg_dpi`](ImportOptions::svg_dpi).
    pub unit: Option<LengthUnit>,
    /// Extra uniform factor applied after the unit conversion. Default 1.
    pub scale: f64,
    /// Layers (DXF layer names, SVG group ids; compared ASCII case-insensitively) whose
    /// geometry becomes construction geometry.
    pub construction_layers: Vec<String>,
    /// Layers that are not imported at all.
    pub skip_layers: Vec<String>,
    /// DXF layers that are switched off or frozen become construction geometry. Default true.
    pub hidden_layers_construction: bool,
    /// Free ends closer than this, in mm after scaling, are snapped together and constrained
    /// coincident. Default 1e-4. Lines shorter than this are dropped.
    pub merge_tol: f64,
    /// CSS pixels per inch for SVG. Default 96.
    pub svg_dpi: f64,
}

impl Default for ImportOptions {
    fn default() -> Self {
        ImportOptions {
            unit: None,
            scale: 1.0,
            construction_layers: Vec::new(),
            skip_layers: Vec::new(),
            hidden_layers_construction: true,
            merge_tol: 1e-4,
            svg_dpi: 96.0,
        }
    }
}

impl ImportOptions {
    fn check(&self) -> Result<(), ImportError> {
        if !(self.scale.is_finite() && self.scale > 0.0) {
            return Err(ImportError::Options("scale must be positive and finite"));
        }
        if !(self.merge_tol.is_finite() && self.merge_tol >= 0.0) {
            return Err(ImportError::Options(
                "merge_tol must be non-negative and finite",
            ));
        }
        if !(self.svg_dpi.is_finite() && self.svg_dpi > 0.0) {
            return Err(ImportError::Options("svg_dpi must be positive and finite"));
        }
        Ok(())
    }

    pub(crate) fn is_construction(&self, layer: &str) -> bool {
        self.construction_layers
            .iter()
            .any(|l| l.eq_ignore_ascii_case(layer))
    }

    pub(crate) fn is_skipped(&self, layer: &str) -> bool {
        self.skip_layers
            .iter()
            .any(|l| l.eq_ignore_ascii_case(layer))
    }
}

/// Where the length scale came from.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UnitSource {
    /// The DXF header's `$INSUNITS`.
    Declared,
    /// [`ImportOptions::unit`].
    Override,
    /// The file declares no unit (DXF `$INSUNITS = 0`): read as mm.
    AssumedMm,
    /// SVG CSS pixels at [`ImportOptions::svg_dpi`].
    SvgPixels,
}

/// The length scale applied to the file's coordinates.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct UnitInfo {
    /// Where it came from.
    pub source: UnitSource,
    /// The DXF `$INSUNITS` code of the file (0 = unitless), `None` for SVG.
    pub declared_code: Option<i16>,
    /// Total factor from file coordinates to mm (unit conversion times
    /// [`ImportOptions::scale`]).
    pub scale_to_mm: f64,
}

/// What an import created.
#[derive(Clone, Debug)]
pub struct ImportReport {
    /// The imported curves (not their points), in file order.
    pub entities: Vec<EntityId>,
    /// The `Coincident` constraints that join merged ends.
    pub constraints: Vec<ConstraintId>,
    /// Imported curves by layer (empty name = no layer).
    pub layers: BTreeMap<String, Vec<EntityId>>,
    /// The applied scale.
    pub unit: UnitInfo,
    /// Bounding box of all imported curves (construction included), mm. Empty if nothing
    /// was imported.
    pub bbox: Aabb2,
    /// Free ends joined with another end (= `constraints.len()`).
    pub merged: usize,
    /// Of those, ends that moved to reach the other (the rest already coincided).
    pub snapped: usize,
    /// Skipped inputs by kind (`"TEXT"`, `"HATCH"`, `"degenerate"`, ...), with counts.
    pub skipped: BTreeMap<String, usize>,
    /// Human-readable notes (approximations, skipped content).
    pub warnings: Vec<String>,
}

/// Imports the DXF file at `path`. See [`import_dxf`].
pub fn import_dxf_file(
    sketch: &mut Sketch,
    path: &Path,
    opts: &ImportOptions,
) -> Result<ImportReport, ImportError> {
    let bytes = std::fs::read(path).map_err(|e| ImportError::Io {
        path: path.display().to_string(),
        message: e.to_string(),
    })?;
    import_dxf(sketch, &bytes, opts)
}

/// Imports the SVG file at `path`. See [`import_svg`].
pub fn import_svg_file(
    sketch: &mut Sketch,
    path: &Path,
    opts: &ImportOptions,
) -> Result<ImportReport, ImportError> {
    let text = std::fs::read_to_string(path).map_err(|e| ImportError::Io {
        path: path.display().to_string(),
        message: e.to_string(),
    })?;
    import_svg(sketch, &text, opts)
}

/// Imports an ASCII or binary DXF (R12 or later) into `sketch`'s plane, from the ENTITIES
/// section. Only the XY components are used; entities whose extrusion is neither +Z nor -Z
/// (mirrored in x) are skipped. Blocks (`INSERT`), text, hatches and dimensions are skipped
/// and counted in [`ImportReport::skipped`].
pub fn import_dxf(
    sketch: &mut Sketch,
    bytes: &[u8],
    opts: &ImportOptions,
) -> Result<ImportReport, ImportError> {
    opts.check()?;
    let parsed = dxf_in::parse(bytes, opts)?;
    commit(sketch, parsed, opts)
}

/// Imports an SVG document into `sketch`'s plane. Y points up in the sketch:
/// `y = H - y_svg` with `H` the document height, so the drawing keeps its position in the
/// positive quadrant. Strokes, fills and dashes are ignored (every path is a centre line);
/// text and images are skipped.
pub fn import_svg(
    sketch: &mut Sketch,
    svg: &str,
    opts: &ImportOptions,
) -> Result<ImportReport, ImportError> {
    opts.check()?;
    let parsed = svg_in::parse(svg, opts)?;
    commit(sketch, parsed, opts)
}

// ---- shared pipeline ------------------------------------------------------------------

/// Curve in file coordinates, before the end merge.
#[derive(Clone, Debug)]
pub(crate) enum Prim {
    Line {
        a: Point2,
        b: Point2,
    },
    /// CCW from `s` to `e` about `c`.
    Arc {
        c: Point2,
        s: Point2,
        e: Point2,
    },
    Circle {
        c: Point2,
        r: f64,
    },
    Ellipse {
        c: Point2,
        major_end: Point2,
        minor: f64,
    },
    EllipticArc {
        c: Point2,
        major_end: Point2,
        minor: f64,
        t0: f64,
        t1: f64,
    },
    Control {
        degree: usize,
        poles: Vec<Point2>,
        weights: Option<Vec<f64>>,
        knots: Vec<f64>,
    },
    Fit {
        points: Vec<Point2>,
        start_tangent: Option<Vec2>,
        end_tangent: Option<Vec2>,
    },
}

impl Prim {
    fn scaled(&mut self, k: f64) {
        match self {
            Prim::Line { a, b } => {
                *a *= k;
                *b *= k;
            }
            Prim::Arc { c, s, e } => {
                *c *= k;
                *s *= k;
                *e *= k;
            }
            Prim::Circle { c, r } => {
                *c *= k;
                *r *= k;
            }
            Prim::Ellipse {
                c,
                major_end,
                minor,
            }
            | Prim::EllipticArc {
                c,
                major_end,
                minor,
                ..
            } => {
                *c *= k;
                *major_end *= k;
                *minor *= k;
            }
            Prim::Control { poles, .. } => poles.iter_mut().for_each(|p| *p *= k),
            Prim::Fit {
                points,
                start_tangent,
                end_tangent,
            } => {
                points.iter_mut().for_each(|p| *p *= k);
                for t in [start_tangent, end_tangent].into_iter().flatten() {
                    *t *= k;
                }
            }
        }
    }

    /// The curve's free end positions: lines, arcs, splines and elliptical arcs.
    fn end_pos(&self, end: End) -> Option<Point2> {
        let first = end == End::Start;
        match self {
            Prim::Line { a, b } => Some(if first { *a } else { *b }),
            Prim::Arc { s, e, .. } => Some(if first { *s } else { *e }),
            Prim::Control { poles, .. } => {
                poles.get(if first { 0 } else { poles.len() - 1 }).copied()
            }
            Prim::Fit { points, .. } => points
                .get(if first { 0 } else { points.len() - 1 })
                .copied(),
            Prim::EllipticArc {
                c,
                major_end,
                minor,
                t0,
                t1,
            } => Some(
                EllipseArc2::full(*c, *major_end - *c, *minor).point_at(if first {
                    *t0
                } else {
                    *t1
                }),
            ),
            Prim::Circle { .. } | Prim::Ellipse { .. } => None,
        }
    }

    /// False for elliptical arcs, whose ends follow from their parameters.
    fn ends_movable(&self) -> bool {
        !matches!(self, Prim::EllipticArc { .. })
    }

    fn set_end(&mut self, end: End, p: Point2) {
        let first = end == End::Start;
        match self {
            Prim::Line { a, b } => *(if first { a } else { b }) = p,
            Prim::Arc { s, e, .. } => *(if first { s } else { e }) = p,
            Prim::Control { poles, .. } => {
                let i = if first { 0 } else { poles.len() - 1 };
                poles[i] = p;
            }
            Prim::Fit { points, .. } => {
                let i = if first { 0 } else { points.len() - 1 };
                points[i] = p;
            }
            _ => {}
        }
    }
}

/// One parsed curve with its layer.
#[derive(Clone, Debug)]
pub(crate) struct Item {
    pub layer: String,
    pub construction: bool,
    pub prim: Prim,
}

/// Parser output, in file units.
pub(crate) struct Parsed {
    pub items: Vec<Item>,
    pub skipped: BTreeMap<String, usize>,
    pub warnings: Vec<String>,
    pub unit: UnitInfo,
}

impl Parsed {
    pub(crate) fn skip(&mut self, what: &str) {
        *self.skipped.entry(what.to_owned()).or_default() += 1;
    }
}

fn commit(
    sketch: &mut Sketch,
    mut parsed: Parsed,
    opts: &ImportOptions,
) -> Result<ImportReport, ImportError> {
    let k = parsed.unit.scale_to_mm;
    let mut items = std::mem::take(&mut parsed.items);
    for it in &mut items {
        it.prim.scaled(k);
    }
    // Degenerate curves go before the merge, so their ends do not attract others.
    let tol = Tolerance::DEFAULT;
    let min_len = opts.merge_tol.max(1e-9);
    let before = items.len();
    items.retain(|it| match &it.prim {
        Prim::Line { a, b } => a.distance(*b) > min_len,
        Prim::Arc { c, s, e } => {
            let (ds, de) = (*s - *c, *e - *c);
            ds.length() > min_len
                && de.length() > min_len
                && ds.perp_dot(de).atan2(ds.dot(de)).abs() > tol.angular
        }
        Prim::Circle { r, .. } => *r > min_len,
        Prim::Ellipse {
            c,
            major_end,
            minor,
        }
        | Prim::EllipticArc {
            c,
            major_end,
            minor,
            ..
        } => c.distance(*major_end) > min_len && *minor > min_len,
        _ => true,
    });
    let dropped = before - items.len();
    if dropped > 0 {
        *parsed.skipped.entry("degenerate".to_owned()).or_default() += dropped;
        parsed
            .warnings
            .push(format!("{dropped} zero-size curve(s) were dropped"));
    }

    let merged = merge_ends(&mut items, opts.merge_tol);

    let mut ids = Vec::with_capacity(items.len());
    let mut layers: BTreeMap<String, Vec<EntityId>> = BTreeMap::new();
    let mut bbox = Aabb2::EMPTY;
    let mut created: Vec<Option<EntityId>> = Vec::with_capacity(items.len());
    for it in &items {
        match create(sketch, &it.prim, it.construction) {
            Ok(id) => {
                ids.push(id);
                layers.entry(it.layer.clone()).or_default().push(id);
                if let Some(c) = sketch.curve(id) {
                    let b = c.bbox();
                    if !b.is_empty() {
                        bbox.include_point(b.min);
                        bbox.include_point(b.max);
                    }
                }
                created.push(Some(id));
            }
            Err(e) => {
                parsed.skip("invalid geometry");
                parsed.warnings.push(format!("a curve was skipped: {e}"));
                created.push(None);
            }
        }
    }

    let mut constraints = Vec::new();
    for group in &merged.groups {
        let pts: Vec<EntityId> = group
            .iter()
            .filter_map(|&(i, end)| {
                let id = created[i]?;
                sketch.entity(id)?.kind.endpoint(end)
            })
            .collect();
        for &p in pts.iter().skip(1) {
            constraints
                .push(sketch.add_constraint(ConstraintKind::Coincident { a: pts[0], b: p })?);
        }
    }

    Ok(ImportReport {
        entities: ids,
        merged: constraints.len(),
        constraints,
        layers,
        unit: parsed.unit,
        bbox,
        snapped: merged.moved,
        skipped: parsed.skipped,
        warnings: parsed.warnings,
    })
}

struct Merged {
    /// Clusters of two or more ends `(item index, end)`; the first is the representative.
    groups: Vec<Vec<(usize, End)>>,
    moved: usize,
}

/// Clusters free ends within `tol` (single linkage) and snaps the movable ones onto the
/// representative: a fixed end (elliptical arc) if the cluster has one, else the first.
fn merge_ends(items: &mut [Item], tol: f64) -> Merged {
    let mut ends: Vec<(usize, End, Point2)> = Vec::new();
    for (i, it) in items.iter().enumerate() {
        for end in [End::Start, End::End] {
            if let Some(p) = it.prim.end_pos(end) {
                ends.push((i, end, p));
            }
        }
    }
    let cell = tol.max(1e-9);
    #[allow(clippy::cast_possible_truncation, reason = "grid cell index")]
    let key = |p: Point2| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64);
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (n, e) in ends.iter().enumerate() {
        grid.entry(key(e.2)).or_default().push(n);
    }
    let mut parent: Vec<usize> = (0..ends.len()).collect();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    for (n, e) in ends.iter().enumerate() {
        let (kx, ky) = key(e.2);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for &m in grid.get(&(kx + dx, ky + dy)).into_iter().flatten() {
                    if m > n && e.2.distance(ends[m].2) <= tol {
                        let (a, b) = (find(&mut parent, n), find(&mut parent, m));
                        if a != b {
                            parent[a.max(b)] = a.min(b);
                        }
                    }
                }
            }
        }
    }
    let mut clusters: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for n in 0..ends.len() {
        let r = find(&mut parent, n);
        clusters.entry(r).or_default().push(n);
    }
    let mut groups = Vec::new();
    let mut moved = 0;
    for members in clusters.into_values().filter(|m| m.len() > 1) {
        let rep = members
            .iter()
            .copied()
            .find(|&n| !items[ends[n].0].prim.ends_movable())
            .unwrap_or(members[0]);
        let target = ends[rep].2;
        let mut group = vec![(ends[rep].0, ends[rep].1)];
        for &n in members.iter().filter(|&&n| n != rep) {
            let (i, end, p) = ends[n];
            if items[i].prim.ends_movable() {
                if p != target {
                    moved += 1;
                }
                items[i].prim.set_end(end, target);
            }
            group.push((i, end));
        }
        groups.push(group);
    }
    Merged { groups, moved }
}

/// Creates the entity of `prim` with origin [`EntityOrigin::Imported`].
fn create(sketch: &mut Sketch, prim: &Prim, construction: bool) -> Result<EntityId, SketchError> {
    let kind = match prim {
        Prim::Line { a, b } => EntityKind::Line {
            start: sketch.add_point(*a)?,
            end: sketch.add_point(*b)?,
        },
        Prim::Arc { c, s, e } => {
            let r = s.distance(*c);
            let de = *e - *c;
            // The end is projected onto the start's circle, as `Sketch::add_arc` does.
            let end = *c + de * (r / de.length());
            EntityKind::Arc {
                center: sketch.add_point(*c)?,
                start: sketch.add_point(*s)?,
                end: sketch.add_point(end)?,
            }
        }
        Prim::Circle { c, r } => EntityKind::Circle {
            center: sketch.add_point(*c)?,
            radius: sketch.alloc_param(*r, ParamFlags::default())?,
        },
        Prim::Ellipse {
            c,
            major_end,
            minor,
        } => EntityKind::Ellipse {
            center: sketch.add_point(*c)?,
            major: sketch.add_point(*major_end)?,
            minor_radius: sketch.alloc_param(*minor, ParamFlags::default())?,
        },
        Prim::EllipticArc {
            c,
            major_end,
            minor,
            t0,
            t1,
        } => {
            let e = EllipseArc2::full(*c, *major_end - *c, *minor);
            EntityKind::EllipticArc {
                center: sketch.add_point(*c)?,
                major: sketch.add_point(*major_end)?,
                minor_radius: sketch.alloc_param(*minor, ParamFlags::default())?,
                start: sketch.add_point(e.point_at(*t0))?,
                end: sketch.add_point(e.point_at(*t1))?,
                t0: sketch.alloc_param(*t0, ParamFlags::default())?,
                t1: sketch.alloc_param(*t1, ParamFlags::default())?,
            }
        }
        Prim::Control {
            degree,
            poles,
            weights,
            knots,
        } => {
            let poles: Vec<EntityId> = poles
                .iter()
                .map(|&p| sketch.add_point(p))
                .collect::<Result<_, _>>()?;
            let weights = weights
                .as_ref()
                .map(|w| {
                    w.iter()
                        .map(|&v| sketch.alloc_param(v, ParamFlags::default()))
                        .collect::<Result<Vec<_>, _>>()
                })
                .transpose()?;
            EntityKind::Spline(SplineDef::Control {
                degree: u8::try_from(*degree).unwrap_or(u8::MAX),
                poles,
                weights,
                knots: knots.clone(),
                periodic: false,
            })
        }
        Prim::Fit {
            points,
            start_tangent,
            end_tangent,
        } => {
            let params = chord_params(points)?;
            let ids: Vec<EntityId> = points
                .iter()
                .map(|&p| sketch.add_point(p))
                .collect::<Result<_, _>>()?;
            let start = start_tangent
                .map(|t| sketch.add_point(points[0] + t))
                .transpose()?;
            let end = end_tangent
                .map(|t| sketch.add_point(points[points.len() - 1] + t))
                .transpose()?;
            EntityKind::Spline(SplineDef::Fit {
                points: ids,
                start_tangent: start,
                end_tangent: end,
                params,
                periodic: false,
            })
        }
    };
    sketch.add_entity(kind, construction, EntityOrigin::Imported)
}

/// A control-point B-spline prim; `knots` defaults to clamped uniform.
pub(crate) fn control_prim(
    degree: usize,
    poles: Vec<Point2>,
    weights: Option<Vec<f64>>,
    knots: Option<Vec<f64>>,
) -> Prim {
    let knots = knots.unwrap_or_else(|| clamped_uniform_knots(poles.len(), degree));
    Prim::Control {
        degree,
        poles,
        weights,
        knots,
    }
}

/// Millimetres per unit of a DXF `$INSUNITS` code, `None` for 0 (unitless) and unknown codes.
pub(crate) fn insunits_mm(code: i16) -> Option<f64> {
    Some(match code {
        1 => Unit::Inch.scale(),
        2 => Unit::Foot.scale(),
        3 => 1_609_344.0,
        4 => 1.0,
        5 => Unit::Centimetre.scale(),
        6 => Unit::Metre.scale(),
        7 => 1.0e6,
        8 => 25.4e-6,
        9 => 0.0254,
        10 => 914.4,
        11 => 1.0e-7,
        12 => 1.0e-6,
        13 => Unit::Micrometre.scale(),
        14 => 100.0,
        15 => 1.0e4,
        16 => 1.0e5,
        17 => 1.0e12,
        18 => 1.495_978_707e14,
        19 => 9.460_730_472_580_8e18,
        20 => 3.085_677_581_491_4e19,
        21 => 304.800_609_6,
        22 => 25.400_050_8,
        23 => 914.401_828_8,
        24 => 1_609_347.218_694_4,
        _ => return None,
    })
}

/// Counter-clockwise sweep from angle `a0` to `a1`, in `(0, 2pi]`.
pub(crate) fn ccw_sweep(a0: f64, a1: f64) -> f64 {
    let s = (a1 - a0).rem_euclid(TAU);
    if s == 0.0 { TAU } else { s }
}

#[cfg(test)]
mod tests;
