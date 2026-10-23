//! SVG reader: `usvg` paths (transforms, units and shapes already flattened) to [`Prim`]s.

use std::collections::BTreeMap;

use amg_core::{Point2, Unit, Vec2};
use usvg::tiny_skia_path::{PathSegment, Point as SkPoint};
use usvg::{Group, Node};

use super::{ImportError, ImportOptions, Item, Parsed, Prim, UnitInfo, UnitSource, control_prim};

/// Handle length of a quarter-circle cubic, in radii.
const KAPPA: f64 = 0.552_284_749_830_793_4;

/// Closing gaps below this (file units, f32 noise of the transformed coordinates) are not a
/// segment; the merge joins the two ends instead.
const CLOSE_EPS: f64 = 2e-4;

pub(super) fn parse(svg: &str, opts: &ImportOptions) -> Result<Parsed, ImportError> {
    let uopts = usvg::Options {
        // Truncation to f32 is what usvg stores anyway.
        #[allow(clippy::cast_possible_truncation, reason = "usvg dpi is f32")]
        dpi: opts.svg_dpi as f32,
        ..usvg::Options::default()
    };
    let tree = usvg::Tree::from_str(svg, &uopts).map_err(|e| ImportError::Svg(e.to_string()))?;
    let (px_mm, source) = match opts.unit {
        Some(u) => (Unit::from(u).scale(), UnitSource::Override),
        None => (25.4 / opts.svg_dpi, UnitSource::SvgPixels),
    };
    let mut parsed = Parsed {
        items: Vec::new(),
        skipped: BTreeMap::new(),
        warnings: Vec::new(),
        unit: UnitInfo {
            source,
            declared_code: None,
            scale_to_mm: px_mm * opts.scale,
        },
    };
    let ctx = Ctx {
        height: f64::from(tree.size().height()),
        opts,
    };
    walk(tree.root(), "", &ctx, &mut parsed);
    Ok(parsed)
}

struct Ctx<'a> {
    height: f64,
    opts: &'a ImportOptions,
}

fn walk(group: &Group, layer: &str, ctx: &Ctx<'_>, parsed: &mut Parsed) {
    for node in group.children() {
        match node {
            Node::Group(g) => {
                // The outermost named group is the layer.
                let layer = if layer.is_empty() { g.id() } else { layer };
                walk(g, layer, ctx, parsed);
            }
            Node::Path(p) => {
                if ctx.opts.is_skipped(layer) {
                    continue;
                }
                let mut prims = Vec::new();
                path(p, ctx.height, &mut prims);
                let construction = ctx.opts.is_construction(layer);
                parsed.items.extend(prims.into_iter().map(|prim| Item {
                    layer: layer.to_owned(),
                    construction,
                    prim,
                }));
            }
            Node::Image(_) => parsed.skip("image"),
            Node::Text(_) => parsed.skip("text"),
        }
    }
}

#[derive(Clone, Copy)]
enum Seg {
    Line([Point2; 2]),
    Quad([Point2; 3]),
    Cubic([Point2; 4]),
}

fn path(p: &usvg::Path, height: f64, out: &mut Vec<Prim>) {
    let ts = p.abs_transform();
    let map = |q: SkPoint| {
        let mut q = q;
        ts.map_point(&mut q);
        Point2::new(f64::from(q.x), height - f64::from(q.y))
    };
    let mut subs: Vec<Vec<Seg>> = Vec::new();
    let mut cur = Point2::ZERO;
    let mut start = Point2::ZERO;
    for seg in p.data().segments() {
        match seg {
            PathSegment::MoveTo(q) => {
                cur = map(q);
                start = cur;
                subs.push(Vec::new());
            }
            PathSegment::LineTo(q) => {
                let e = map(q);
                push(&mut subs, Seg::Line([cur, e]));
                cur = e;
            }
            PathSegment::QuadTo(q1, q2) => {
                let (c, e) = (map(q1), map(q2));
                push(&mut subs, Seg::Quad([cur, c, e]));
                cur = e;
            }
            PathSegment::CubicTo(q1, q2, q3) => {
                let (c1, c2, e) = (map(q1), map(q2), map(q3));
                push(&mut subs, Seg::Cubic([cur, c1, c2, e]));
                cur = e;
            }
            PathSegment::Close => {
                if cur.distance(start) > CLOSE_EPS {
                    push(&mut subs, Seg::Line([cur, start]));
                }
                cur = start;
            }
        }
    }
    for segs in subs.iter().filter(|s| !s.is_empty()) {
        if let Some(ring) = cubic_ring(segs) {
            out.push(ring);
            continue;
        }
        for seg in segs {
            match *seg {
                Seg::Line([a, b]) => out.push(Prim::Line { a, b }),
                Seg::Quad(q) => out.push(control_prim(2, q.to_vec(), None, None)),
                Seg::Cubic(c) => out.push(cubic(c)),
            }
        }
    }
}

fn push(subs: &mut Vec<Vec<Seg>>, seg: Seg) {
    if subs.is_empty() {
        subs.push(Vec::new());
    }
    if let Some(last) = subs.last_mut() {
        last.push(seg);
    }
}

fn bez3(c: &[Point2; 4], t: f64) -> Point2 {
    let u = 1.0 - t;
    c[0] * (u * u * u) + c[1] * (3.0 * u * u * t) + c[2] * (3.0 * u * t * t) + c[3] * (t * t * t)
}

/// A cubic as a line (collinear handles), a circular arc, or a degree 3 Bezier.
fn cubic(c: [Point2; 4]) -> Prim {
    let chord = c[0].distance(c[3]);
    let off = |q: Point2| {
        let d = c[3] - c[0];
        if chord > 0.0 {
            d.perp_dot(q - c[0]).abs() / chord
        } else {
            q.distance(c[0])
        }
    };
    let (m, q1, q3) = (bez3(&c, 0.5), bez3(&c, 0.25), bez3(&c, 0.75));
    let straight = chord > 0.0 && off(c[1]) <= 1e-9 * chord && off(c[2]) <= 1e-9 * chord;
    if straight {
        return Prim::Line { a: c[0], b: c[3] };
    }
    if let Some(centre) = circumcentre(c[0], m, c[3]) {
        let r = c[0].distance(centre);
        let dev = (q1.distance(centre) - r)
            .abs()
            .max((q3.distance(centre) - r).abs());
        if dev <= (3.5e-4 * r).min(5e-4 * chord) {
            let ccw = (m - c[0]).perp_dot(c[3] - m) > 0.0;
            return if ccw {
                Prim::Arc {
                    c: centre,
                    s: c[0],
                    e: c[3],
                }
            } else {
                Prim::Arc {
                    c: centre,
                    s: c[3],
                    e: c[0],
                }
            };
        }
    }
    control_prim(3, c.to_vec(), None, None)
}

fn circumcentre(a: Point2, b: Point2, c: Point2) -> Option<Point2> {
    let (ba, ca) = (b - a, c - a);
    let d = 2.0 * ba.perp_dot(ca);
    if d.abs() < 1e-12 * (ba.length_squared() + ca.length_squared()).max(f64::MIN_POSITIVE) {
        return None;
    }
    let (b2, c2) = (ba.length_squared(), ca.length_squared());
    Some(a + Vec2::new(ca.y * b2 - ba.y * c2, ba.x * c2 - ca.x * b2) / d)
}

/// A closed ring of four quarter cubics that is the affine image of a circle: an ellipse
/// (a circle if the axes agree).
fn cubic_ring(segs: &[Seg]) -> Option<Prim> {
    let cubics: Vec<[Point2; 4]> = segs
        .iter()
        .filter_map(|s| match s {
            Seg::Cubic(c) => Some(*c),
            _ => None,
        })
        .collect();
    if segs.len() != 4 || cubics.len() != 4 {
        return None;
    }
    let anchors: Vec<Point2> = cubics.iter().map(|c| c[0]).collect();
    let centre = (anchors[0] + anchors[1] + anchors[2] + anchors[3]) * 0.25;
    let (a, b) = (anchors[0] - centre, anchors[1] - centre);
    let size = a.length() + b.length();
    let eps = 1e-3 * size;
    let ring = [a, b, -a, -b];
    let closed = (0..4).all(|i| cubics[i][3].distance(cubics[(i + 1) % 4][0]) <= eps);
    if size <= 0.0 || !closed {
        return None;
    }
    for i in 0..4 {
        let (ai, an) = (ring[i], ring[(i + 1) % 4]);
        let c = &cubics[i];
        let mid = centre + (ai + an) * std::f64::consts::FRAC_1_SQRT_2;
        if (anchors[i] - centre).distance(ai) > eps
            || bez3(c, 0.5).distance(mid) > eps
            || c[1].distance(c[0] + an * KAPPA) > 1e-2 * size
            || c[2].distance(c[3] + ai * KAPPA) > 1e-2 * size
        {
            return None;
        }
    }
    // Semi-axes from the conjugate semi-diameters a, b: eigen-decomposition of a a' + b b'.
    let (sxx, syy, sxy) = (
        a.x * a.x + b.x * b.x,
        a.y * a.y + b.y * b.y,
        a.x * a.y + b.x * b.y,
    );
    let mean = 0.5 * (sxx + syy);
    let diff = (0.25 * (sxx - syy) * (sxx - syy) + sxy * sxy).sqrt();
    let (major, minor) = ((mean + diff).sqrt(), (mean - diff).max(0.0).sqrt());
    if minor <= 0.0 {
        return None;
    }
    if major - minor <= 1e-6 * major {
        return Some(Prim::Circle {
            c: centre,
            r: 0.5 * (major + minor),
        });
    }
    let phi = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    Some(Prim::Ellipse {
        c: centre,
        major_end: centre + Vec2::new(phi.cos(), phi.sin()) * major,
        minor,
    })
}
