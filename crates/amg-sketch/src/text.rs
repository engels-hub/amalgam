//! Sketch text (SK-10): text blocks of locked spline outlines.
//!
//! A [`TextBlock`] is an anchor point and a direction point (ordinary, solvable points: the
//! block moves and rotates rigidly with them), a height and a string. [`TextBlock::create`]
//! lays the string out with a [`FontSet`] and adds the glyph outlines to the [`Sketch`]:
//! every contour is a closed chain of line and Bézier ([`SplineDef::Control`], degree 2 or 3)
//! entities of [`EntityOrigin::Imported`] whose point params are *locked*, so the solver never
//! moves them. After the solver moved the anchor or direction point, or the path, call
//! [`TextBlock::update`], which rewrites the outline points in place (entity ids stay).
//! [`detect_profiles`](crate::detect_profiles) then yields the glyph faces. Like every
//! arrangement, it reports the *counter* of an `A` (the region inside its hole) as a profile of
//! its own as well; [`TextBlock::regions`] splits the faces of a block into the `solid` ones
//! (what is extruded, each with its counters as [`Profile::holes`](crate::Profile)) and the
//! `counters`.
//!
//! The block itself is plain serialisable data; the document layer (the sketch feature) stores
//! the `Vec<TextBlock>` next to the [`Sketch`] and the font set it resolves from
//! [`TextBlock::font`].
//!
//! Layout: the first baseline starts at the anchor and runs towards the direction point; lines
//! (`'\n'`) follow downwards. `height` is the cap height in mm. There is no kerning or
//! shaping: advances only (Latin-style simple scripts).
//!
//! Along a path ([`Placement::AlongPath`]) every glyph is placed rigidly, centred on its advance, at the
//! arc-length position on the curve, rotated to the tangent there; the text is not bent.

mod font;

use amg_core::{Point2, Vec2};
use serde::{Deserialize, Serialize};

pub use font::{Contour, Font, FontSet, Glyph, SYNTHETIC_SLANT, Seg};

use crate::geom::{Curve2, clamped_uniform_knots};
use crate::{
    EntityId, EntityKind, EntityOrigin, Loop, ParamFlags, ProfileId, ProfileSet, RemoveReport,
    Sketch, SketchError, SplineDef,
};

/// Errors of text creation and update.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum TextError {
    /// The font data could not be parsed or lacks vertical metrics.
    #[error("cannot use the font: {0}")]
    BadFont(String),
    /// The font set has no face for the style and none can be synthesised.
    #[error("the font family has no {0} face")]
    StyleUnavailable(&'static str),
    /// The font has no glyph for the character.
    #[error("the font has no glyph for {0:?}")]
    MissingGlyph(char),
    /// The text has no outline at all (empty or only spaces).
    #[error("the text has no outlines")]
    Empty,
    /// The text is longer than the path from the offset.
    #[error("the text needs {needed} mm of path, {available} mm are available")]
    PathTooShort {
        /// Text width plus start offset, mm.
        needed: f64,
        /// Path length, mm.
        available: f64,
    },
    /// The text cannot be laid along a path (multi-line text, or a path entity that is not a
    /// curve).
    #[error("text along a path: {0}")]
    BadPath(&'static str),
    /// A sketch operation failed.
    #[error(transparent)]
    Sketch(#[from] SketchError),
}

/// Where a text block sits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Placement {
    /// Straight, the first baseline starts at `anchor` and runs at `angle` (rad, CCW from the
    /// sketch x axis).
    Free {
        /// Start of the first baseline.
        anchor: Point2,
        /// Direction of the baseline.
        angle: f64,
    },
    /// Along the curve entity `curve`, starting `offset` mm of arc length from its start.
    AlongPath(PathPlacement),
}

/// Text along a curve.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PathPlacement {
    /// The curve entity (line, circle, arc, ellipse or spline).
    pub curve: EntityId,
    /// Arc length from the curve start to the first glyph, mm.
    pub offset: f64,
    /// Run along the reversed curve (the text then sits on the other side).
    pub reversed: bool,
}

/// What to write.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TextSpec {
    /// The string; `'\n'` starts a new line.
    pub text: String,
    /// Cap height, mm.
    pub height: f64,
    /// Bold face.
    pub bold: bool,
    /// Italic face (synthesised by shear if the family has none).
    pub italic: bool,
    /// Font identifier for the document layer (family or file); not interpreted here.
    pub font: String,
    /// Baseline pitch relative to the font's line height. Default 1.
    pub line_spacing: f64,
}

impl TextSpec {
    /// Plain regular text of the given cap height.
    pub fn new(text: impl Into<String>, height: f64, font: impl Into<String>) -> TextSpec {
        TextSpec {
            text: text.into(),
            height,
            bold: false,
            italic: false,
            font: font.into(),
            line_spacing: 1.0,
        }
    }
}

/// A text block of a sketch (see the [module docs](self)).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TextBlock {
    /// What is written; edit the fields and call [`update`](TextBlock::update) for a new
    /// height or style. Changing `text` needs a new block.
    pub spec: TextSpec,
    /// Anchor point (start of the first baseline), a free point unless the block is on a path.
    pub anchor: EntityId,
    /// Direction point; the baseline runs from the anchor towards it. A free point unless the
    /// block is on a path.
    pub direction: EntityId,
    /// The path, if any.
    pub path: Option<PathPlacement>,
    /// The outline curves (lines and Bézier splines), contour by contour.
    pub curves: Vec<EntityId>,
    /// The outline points (vertices, then Bézier control points, per contour).
    pub points: Vec<EntityId>,
}

impl TextBlock {
    /// Adds the text to `sketch`.
    ///
    /// # Errors
    /// [`TextError`] for a missing glyph or style, empty text, a bad height, a path that is
    /// too short or not a curve. Nothing is added on error.
    pub fn create(
        sketch: &mut Sketch,
        fonts: &FontSet,
        spec: TextSpec,
        placement: &Placement,
    ) -> Result<TextBlock, TextError> {
        check_spec(&spec)?;
        let (path, frame) = match placement {
            Placement::Free { anchor, angle } => {
                if !anchor.is_finite() || !angle.is_finite() {
                    return Err(SketchError::InvalidValue {
                        what: "text anchor",
                        value: f64::NAN,
                    }
                    .into());
                }
                let dir = Vec2::new(angle.cos(), angle.sin());
                (None, Frame::Rigid(*anchor, dir))
            }
            Placement::AlongPath(p) => (Some(*p), path_frame(sketch, p)?),
        };
        let contours = lay_out(fonts, &spec, &frame)?;
        let (origin, dir) = frame.origin_dir()?;

        // Build everything on a scratch copy so an error leaves `sketch` untouched.
        let mut work = sketch.clone();
        let anchor = work.add_point(origin)?;
        let direction = work.add_point(origin + dir * spec.height)?;
        if path.is_some() {
            for id in [anchor, direction] {
                lock_point(&mut work, id)?;
            }
        }
        let mut block = TextBlock {
            spec,
            anchor,
            direction,
            path,
            curves: Vec::new(),
            points: Vec::new(),
        };
        for c in &contours {
            block.add_contour(&mut work, c)?;
        }
        *sketch = work;
        Ok(block)
    }

    fn add_contour(&mut self, sketch: &mut Sketch, c: &Contour) -> Result<(), TextError> {
        let n = c.ends.len();
        let first = self.points.len();
        for p in contour_points(c) {
            let id = locked_point(sketch, p)?;
            self.points.push(id);
        }
        let verts = &self.points[first..first + n];
        let mut next_control = first + n;
        let mut curves = Vec::with_capacity(n);
        for (i, seg) in c.segs.iter().enumerate() {
            let a = verts[(i + n - 1) % n];
            let b = verts[i];
            let (degree, poles) = match *seg {
                Seg::Line => {
                    let kind = EntityKind::Line { start: a, end: b };
                    curves.push(sketch.add_entity(kind, false, EntityOrigin::Imported)?);
                    continue;
                }
                Seg::Quad { .. } => (2u8, vec![a, self.points[next_control], b]),
                Seg::Cubic { .. } => (
                    3u8,
                    vec![
                        a,
                        self.points[next_control],
                        self.points[next_control + 1],
                        b,
                    ],
                ),
            };
            next_control += usize::from(degree) - 1;
            let kind = EntityKind::Spline(SplineDef::Control {
                degree,
                knots: clamped_uniform_knots(poles.len(), usize::from(degree)),
                poles,
                weights: None,
                periodic: false,
            });
            curves.push(sketch.add_entity(kind, false, EntityOrigin::Imported)?);
        }
        self.curves.extend(curves);
        Ok(())
    }

    /// Re-lays the text out from the current anchor/direction points (or path) and the block's
    /// `spec` (height, style, line spacing) and rewrites the outline points in place.
    ///
    /// # Errors
    /// [`TextError`] as for [`create`](TextBlock::create); `Sketch(NotApplicable)` if the
    /// layout no longer has the topology of the created outline (different font or style).
    pub fn update(&self, sketch: &mut Sketch, fonts: &FontSet) -> Result<(), TextError> {
        check_spec(&self.spec)?;
        let frame = match &self.path {
            Some(p) => path_frame(sketch, p)?,
            None => {
                let a = sketch
                    .point(self.anchor)
                    .ok_or(SketchError::UnknownEntity(self.anchor))?;
                let d = sketch
                    .point(self.direction)
                    .ok_or(SketchError::UnknownEntity(self.direction))?;
                let dir = (d - a).normalize_or_zero();
                if dir == Vec2::ZERO {
                    return Err(SketchError::Degenerate("text direction has zero length").into());
                }
                Frame::Rigid(a, dir)
            }
        };
        let contours = lay_out(fonts, &self.spec, &frame)?;
        let pts: Vec<Point2> = contours.iter().flat_map(contour_points).collect();
        if pts.len() != self.points.len() {
            return Err(SketchError::NotApplicable("the text layout changed topology").into());
        }
        if self.path.is_some() {
            let (origin, dir) = frame.origin_dir()?;
            set_point(sketch, self.anchor, origin)?;
            set_point(sketch, self.direction, origin + dir * self.spec.height)?;
        }
        for (&id, &p) in self.points.iter().zip(&pts) {
            set_point(sketch, id, p)?;
        }
        Ok(())
    }

    /// Removes the whole block (outline, anchor and direction points) from `sketch`.
    pub fn remove(&self, sketch: &mut Sketch) -> Result<RemoveReport, SketchError> {
        let mut ids = self.curves.clone();
        ids.extend(&self.points);
        ids.extend([self.anchor, self.direction]);
        sketch.remove_entities(&ids)
    }
}

/// The faces of a text block, as found by [`TextBlock::regions`].
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct GlyphRegions {
    /// Faces of the glyph material (the profiles to extrude): outer contours, with the counters
    /// as holes. An island inside a counter (as in `®`) is solid again.
    pub solid: Vec<ProfileId>,
    /// Faces filling the counters (the inside of an `A`'s hole), alternating with `solid` by
    /// nesting depth.
    pub counters: Vec<ProfileId>,
}

fn loop_key(l: &Loop) -> Vec<EntityId> {
    let mut k: Vec<EntityId> = l
        .edges
        .iter()
        .flat_map(|e| std::iter::once(e.source).chain(e.also.iter().copied()))
        .collect();
    k.sort_unstable();
    k.dedup();
    k
}

impl TextBlock {
    /// Splits the profiles of the block's outline into material and counter faces. A face is
    /// at nesting depth 0 (solid) if its outer loop is no hole of another face of the block,
    /// otherwise one deeper than the face it is a hole of; even depths are solid.
    pub fn regions(&self, profiles: &ProfileSet) -> GlyphRegions {
        let mine: std::collections::HashSet<EntityId> = self.curves.iter().copied().collect();
        let faces: Vec<&crate::Profile> = profiles
            .profiles
            .iter()
            .filter(|p| p.outer_sources().iter().all(|s| mine.contains(s)))
            .collect();
        let hole_keys: Vec<(usize, Vec<EntityId>)> = faces
            .iter()
            .enumerate()
            .flat_map(|(i, p)| p.holes.iter().map(move |h| (i, loop_key(h))))
            .collect();
        let mut out = GlyphRegions::default();
        for p in &faces {
            let mut depth = 0usize;
            let mut key = p.outer_sources().to_vec();
            // Follow the chain of enclosing faces (bounded by the face count).
            while let Some((parent, _)) = hole_keys.iter().find(|(_, k)| *k == key) {
                depth += 1;
                key = faces[*parent].outer_sources().to_vec();
                if depth > faces.len() {
                    break;
                }
            }
            if depth.is_multiple_of(2) {
                out.solid.push(p.id);
            } else {
                out.counters.push(p.id);
            }
        }
        out
    }
}

fn check_spec(spec: &TextSpec) -> Result<(), SketchError> {
    for (what, v) in [
        ("text height", spec.height),
        ("line spacing", spec.line_spacing),
    ] {
        if !(v.is_finite() && v > 0.0) {
            return Err(SketchError::InvalidValue { what, value: v });
        }
    }
    Ok(())
}

/// Vertices first, then the Bézier control points in segment order.
fn contour_points(c: &Contour) -> Vec<Point2> {
    let mut v = c.ends.clone();
    for s in &c.segs {
        match *s {
            Seg::Line => {}
            Seg::Quad { c } => v.push(c),
            Seg::Cubic { c1, c2 } => v.extend([c1, c2]),
        }
    }
    v
}

fn locked_point(sketch: &mut Sketch, p: Point2) -> Result<EntityId, SketchError> {
    let flags = ParamFlags { locked: true };
    let x = sketch.alloc_param(p.x, flags)?;
    let y = sketch.alloc_param(p.y, flags)?;
    sketch.add_entity(EntityKind::Point { x, y }, false, EntityOrigin::Imported)
}

fn lock_point(sketch: &mut Sketch, id: EntityId) -> Result<(), SketchError> {
    if let Some(EntityKind::Point { x, y }) = sketch.entity(id).map(|e| e.kind.clone()) {
        for q in [x, y] {
            sketch
                .params_mut()
                .set_flags(q, ParamFlags { locked: true });
        }
    }
    Ok(())
}

fn set_point(sketch: &mut Sketch, id: EntityId, p: Point2) -> Result<(), SketchError> {
    match sketch.entity(id).map(|e| e.kind.clone()) {
        Some(EntityKind::Point { x, y }) => {
            sketch.params_mut().set(x, p.x);
            sketch.params_mut().set(y, p.y);
            Ok(())
        }
        Some(_) => Err(SketchError::NotApplicable(
            "an outline entity is not a point",
        )),
        None => Err(SketchError::UnknownEntity(id)),
    }
}

/// How glyph space maps into the sketch.
enum Frame {
    /// Origin and unit baseline direction.
    Rigid(Point2, Vec2),
    /// Curve (already reversed if asked), start offset.
    Path(Curve2, f64),
}

impl Frame {
    fn origin_dir(&self) -> Result<(Point2, Vec2), TextError> {
        match self {
            Frame::Rigid(o, d) => Ok((*o, *d)),
            Frame::Path(c, off) => Ok(frame_at(c, *off)),
        }
    }
}

fn path_frame(sketch: &Sketch, p: &PathPlacement) -> Result<Frame, TextError> {
    if !(p.offset.is_finite() && p.offset >= 0.0) {
        return Err(SketchError::InvalidValue {
            what: "text path offset",
            value: p.offset,
        }
        .into());
    }
    let curve = sketch
        .curve(p.curve)
        .ok_or(TextError::BadPath("the path entity is not a curve"))?;
    Ok(Frame::Path(
        if p.reversed { curve.reversed() } else { curve },
        p.offset,
    ))
}

/// Parameter at arc length `s` from the curve start.
fn param_at(curve: &Curve2, s: f64) -> f64 {
    if let Curve2::BSpline(b) = curve {
        return b.param_at_length(s);
    }
    let (t0, t1) = curve.param_range();
    let (mut lo, mut hi) = (t0, t1);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if curve.trimmed(t0, mid).length() < s {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// Point and unit tangent at arc length `s`.
fn frame_at(curve: &Curve2, s: f64) -> (Point2, Vec2) {
    let t = param_at(curve, s);
    let d = curve.derivative(t).normalize_or_zero();
    (curve.eval(t), if d == Vec2::ZERO { Vec2::X } else { d })
}

/// Lays `spec.text` out in the frame; contours are in sketch coordinates.
fn lay_out(fonts: &FontSet, spec: &TextSpec, frame: &Frame) -> Result<Vec<Contour>, TextError> {
    let (font, shear) = fonts.select(spec.bold, spec.italic)?;
    let scale = spec.height / font.cap_height();
    let pitch = font.line_height() * scale * spec.line_spacing;
    if matches!(frame, Frame::Path(..)) && spec.text.contains('\n') {
        return Err(TextError::BadPath("multi-line text cannot follow a path"));
    }
    let mut out = Vec::new();
    for (row, line) in spec.text.split('\n').enumerate() {
        let mut pen = 0.0;
        // Rows below the first are shifted down by `row` pitches.
        let drop = row as f64 * pitch;
        let glyphs = line
            .chars()
            .map(|ch| font.glyph(ch).ok_or(TextError::MissingGlyph(ch)))
            .collect::<Result<Vec<_>, _>>()?;
        if let Frame::Path(curve, offset) = frame {
            let width: f64 = glyphs.iter().map(|g| g.advance * scale).sum();
            let available = curve.length();
            if offset + width > available + 1e-9 {
                return Err(TextError::PathTooShort {
                    needed: offset + width,
                    available,
                });
            }
        }
        for g in &glyphs {
            let adv = g.advance * scale;
            // Local glyph coordinates (mm) -> sketch.
            let place: Box<dyn Fn(Point2) -> Point2> = match frame {
                Frame::Rigid(o, u) => {
                    let (o, u) = (*o, *u);
                    let v = Vec2::new(-u.y, u.x);
                    Box::new(move |p| {
                        let (x, y) = (pen + p.x + shear * p.y, p.y - drop);
                        o + u * x + v * y
                    })
                }
                Frame::Path(curve, offset) => {
                    let (c, t) = frame_at(curve, offset + pen + 0.5 * adv);
                    let n = Vec2::new(-t.y, t.x);
                    Box::new(move |p| {
                        let x = p.x + shear * p.y - 0.5 * adv;
                        c + t * x + n * p.y
                    })
                }
            };
            for c in &g.contours {
                let m = |p: Point2| place(p * scale);
                out.push(Contour {
                    segs: c
                        .segs
                        .iter()
                        .map(|s| match *s {
                            Seg::Line => Seg::Line,
                            Seg::Quad { c } => Seg::Quad { c: m(c) },
                            Seg::Cubic { c1, c2 } => Seg::Cubic {
                                c1: m(c1),
                                c2: m(c2),
                            },
                        })
                        .collect(),
                    ends: c.ends.iter().map(|&p| m(p)).collect(),
                });
            }
            pen += adv;
        }
    }
    if out.is_empty() {
        return Err(TextError::Empty);
    }
    Ok(out)
}
