//! Fonts and glyph outlines (SK-10), read with `skrifa`.

use std::sync::Arc;

use amg_core::Point2;
use skrifa::attribute::Style;
use skrifa::instance::{LocationRef, Size};
use skrifa::metrics::GlyphMetrics;
use skrifa::outline::{DrawSettings, OutlinePen};
use skrifa::string::StringId;
use skrifa::{FontRef, MetadataProvider};

use super::TextError;

/// One segment of a glyph contour, ending at the contour vertex of the same index.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Seg {
    /// Straight line.
    Line,
    /// Quadratic Bézier with control point `c` (TrueType outlines).
    Quad {
        /// Control point.
        c: Point2,
    },
    /// Cubic Bézier with control points `c1`, `c2` (CFF outlines).
    Cubic {
        /// First control point.
        c1: Point2,
        /// Second control point.
        c2: Point2,
    },
}

/// A closed glyph contour: segment `i` runs from `ends[i - 1]` (`ends[n - 1]` for `i = 0`) to
/// `ends[i]`. Coordinates are in em units (1.0 = font size).
#[derive(Clone, Debug, PartialEq)]
pub struct Contour {
    /// Segment kinds.
    pub segs: Vec<Seg>,
    /// Segment end points (= the contour's vertices).
    pub ends: Vec<Point2>,
}

/// The outline of one glyph, em units, y up, origin on the baseline at the pen position.
#[derive(Clone, Debug, PartialEq)]
pub struct Glyph {
    /// Closed contours (outer boundaries and holes alike; the sketch's profile detection sorts
    /// them out).
    pub contours: Vec<Contour>,
    /// Horizontal advance, em.
    pub advance: f64,
}

/// A parsed font face.
#[derive(Clone)]
pub struct Font {
    data: Arc<[u8]>,
    units_per_em: f64,
    cap_height: f64,
    line_height: f64,
    bold: bool,
    italic: bool,
    family: String,
}

impl std::fmt::Debug for Font {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Font")
            .field("family", &self.family)
            .field("bold", &self.bold)
            .field("italic", &self.italic)
            .finish_non_exhaustive()
    }
}

impl Font {
    /// Parses a TrueType/OpenType font (first face of a collection).
    pub fn from_bytes(data: impl Into<Arc<[u8]>>) -> Result<Font, TextError> {
        let data: Arc<[u8]> = data.into();
        let face = FontRef::from_index(&data, 0).map_err(|e| TextError::BadFont(e.to_string()))?;
        let m = face.metrics(Size::unscaled(), LocationRef::default());
        let upem = f64::from(m.units_per_em);
        let cap = m
            .cap_height
            .map(f64::from)
            .filter(|&c| c > 0.0)
            .unwrap_or_else(|| f64::from(m.ascent));
        if !(upem > 0.0 && cap > 0.0) {
            return Err(TextError::BadFont(
                "the font has no cap height or ascender".to_owned(),
            ));
        }
        let line = f64::from(m.ascent) - f64::from(m.descent) + f64::from(m.leading);
        let family = face
            .localized_strings(StringId::FAMILY_NAME)
            .next()
            .map(|n| n.chars().collect::<String>())
            .unwrap_or_default();
        let attrs = face.attributes();
        Ok(Font {
            units_per_em: upem,
            cap_height: cap / upem,
            line_height: line / upem,
            bold: attrs.weight.value() >= 600.0,
            italic: attrs.style != Style::Normal,
            family,
            data,
        })
    }

    /// Family name from the `name` table (empty if absent).
    pub fn family(&self) -> &str {
        &self.family
    }

    /// True if the face is a bold face.
    pub fn is_bold(&self) -> bool {
        self.bold
    }

    /// True if the face is italic or oblique.
    pub fn is_italic(&self) -> bool {
        self.italic
    }

    /// Cap height (or ascender if the font has none), em.
    pub fn cap_height(&self) -> f64 {
        self.cap_height
    }

    /// Distance between baselines at line spacing 1, em.
    pub fn line_height(&self) -> f64 {
        self.line_height
    }

    /// The outline of `ch`; `None` if the font has no glyph for it. Whitespace has no
    /// contours, only an advance.
    pub fn glyph(&self, ch: char) -> Option<Glyph> {
        let face = FontRef::from_index(&self.data, 0).ok()?;
        let id = face.charmap().map(ch)?;
        let metrics: GlyphMetrics<'_> =
            face.glyph_metrics(Size::unscaled(), LocationRef::default());
        let advance = f64::from(metrics.advance_width(id)?) / self.units_per_em;
        let mut b = Builder::new(1.0 / self.units_per_em);
        // No outline entry is an empty glyph (space).
        if let Some(g) = face.outline_glyphs().get(id) {
            let _ = g.draw(
                DrawSettings::unhinted(Size::unscaled(), LocationRef::default()),
                &mut b,
            );
        }
        Some(Glyph {
            contours: b.finish(),
            advance,
        })
    }
}

/// Lines shorter than this (em) are zero-length and dropped, so the topology of a laid-out
/// glyph does not depend on the text height.
const ZERO: f64 = 1e-9;

struct Builder {
    scale: f64,
    done: Vec<Contour>,
    start: Point2,
    cur: Point2,
    segs: Vec<Seg>,
    ends: Vec<Point2>,
}

impl Builder {
    fn new(scale: f64) -> Builder {
        Builder {
            scale,
            done: Vec::new(),
            start: Point2::ZERO,
            cur: Point2::ZERO,
            segs: Vec::new(),
            ends: Vec::new(),
        }
    }

    fn pt(&self, x: f32, y: f32) -> Point2 {
        Point2::new(f64::from(x) * self.scale, f64::from(y) * self.scale)
    }

    fn push(&mut self, seg: Seg, end: Point2) {
        // A line of zero length vanishes; a curve that starts and ends at the same point stays.
        if matches!(seg, Seg::Line) && self.cur.distance(end) < ZERO {
            return;
        }
        self.segs.push(seg);
        self.ends.push(end);
        self.cur = end;
    }

    fn flush(&mut self) {
        let mut segs = std::mem::take(&mut self.segs);
        let mut ends = std::mem::take(&mut self.ends);
        // The last segment ends at the start: snap, or close with a line.
        if let Some(last) = ends.last_mut() {
            if last.distance(self.start) < ZERO {
                *last = self.start;
            } else {
                segs.push(Seg::Line);
                ends.push(self.start);
            }
        }
        // A one-segment loop cannot be built from distinct vertex entities; real fonts do not
        // produce one.
        if segs.len() >= 2 {
            self.done.push(Contour { segs, ends });
        }
    }

    fn finish(mut self) -> Vec<Contour> {
        self.flush();
        self.done
    }
}

impl OutlinePen for Builder {
    fn move_to(&mut self, x: f32, y: f32) {
        self.flush();
        self.start = self.pt(x, y);
        self.cur = self.start;
    }
    fn line_to(&mut self, x: f32, y: f32) {
        let p = self.pt(x, y);
        self.push(Seg::Line, p);
    }
    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        let (c, p) = (self.pt(x1, y1), self.pt(x, y));
        self.push(Seg::Quad { c }, p);
    }
    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        let (c1, c2, p) = (self.pt(x1, y1), self.pt(x2, y2), self.pt(x, y));
        self.push(Seg::Cubic { c1, c2 }, p);
    }
    fn close(&mut self) {
        self.flush();
        self.cur = self.start;
    }
}

/// The faces of one family. Bold and italic pick the matching face; a missing italic face is
/// synthesised by shearing the regular (or bold) face, a missing bold face is an error.
#[derive(Clone, Debug)]
pub struct FontSet {
    /// Upright regular face.
    pub regular: Font,
    /// Upright bold face.
    pub bold: Option<Font>,
    /// Italic regular face.
    pub italic: Option<Font>,
    /// Italic bold face.
    pub bold_italic: Option<Font>,
}

/// Shear (x per y) of a synthesised italic, `tan 12°`.
pub const SYNTHETIC_SLANT: f64 = 0.2126;

impl FontSet {
    /// A family with only a regular face.
    pub fn new(regular: Font) -> FontSet {
        FontSet {
            regular,
            bold: None,
            italic: None,
            bold_italic: None,
        }
    }

    /// The face for the style and the shear to apply to it (0 unless italic is synthesised).
    pub fn select(&self, bold: bool, italic: bool) -> Result<(&Font, f64), TextError> {
        let missing_bold = TextError::StyleUnavailable("bold");
        Ok(match (bold, italic) {
            (false, false) => (&self.regular, 0.0),
            (true, false) => (self.bold.as_ref().ok_or(missing_bold)?, 0.0),
            (false, true) => match &self.italic {
                Some(f) => (f, 0.0),
                None => (&self.regular, SYNTHETIC_SLANT),
            },
            (true, true) => match (&self.bold_italic, &self.bold) {
                (Some(f), _) => (f, 0.0),
                (None, Some(f)) => (f, SYNTHETIC_SLANT),
                (None, None) => return Err(missing_bold),
            },
        })
    }
}
