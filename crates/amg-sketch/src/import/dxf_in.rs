//! DXF reader: `dxf` entities to [`Prim`]s.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;

use amg_core::{Point2, Vec2};
use dxf::Drawing;
use dxf::entities::{Entity, EntityType, LwPolyline, Polyline, Spline};

use super::{
    ImportError, ImportOptions, Item, Parsed, Prim, UnitInfo, UnitSource, ccw_sweep, control_prim,
    insunits_mm,
};

pub(super) fn parse(bytes: &[u8], opts: &ImportOptions) -> Result<Parsed, ImportError> {
    let mut reader = bytes;
    let drawing = Drawing::load(&mut reader).map_err(|e| ImportError::Dxf(e.to_string()))?;

    let code = drawing.header.default_drawing_units as i16;
    let (factor, source) = match (opts.unit, insunits_mm(code)) {
        (Some(u), _) => (amg_core::Unit::from(u).scale(), UnitSource::Override),
        (None, Some(f)) => (f, UnitSource::Declared),
        (None, None) => (1.0, UnitSource::AssumedMm),
    };
    let mut parsed = Parsed {
        items: Vec::new(),
        skipped: BTreeMap::new(),
        warnings: Vec::new(),
        unit: UnitInfo {
            source,
            declared_code: Some(code),
            scale_to_mm: factor * opts.scale,
        },
    };

    let hidden: BTreeSet<String> = drawing
        .layers()
        .filter(|l| !l.is_layer_on || l.color.is_turned_off())
        .map(|l| l.name.to_ascii_lowercase())
        .collect();

    for entity in drawing.entities() {
        let layer = entity.common.layer.clone();
        if opts.is_skipped(&layer) {
            continue;
        }
        let construction = opts.is_construction(&layer)
            || (opts.hidden_layers_construction && hidden.contains(&layer.to_ascii_lowercase()));
        let mut prims = Vec::new();
        convert(entity, &mut prims, &mut parsed);
        parsed.items.extend(prims.into_iter().map(|prim| Item {
            layer: layer.clone(),
            construction,
            prim,
        }));
    }
    Ok(parsed)
}

/// `Some(true)` for an extrusion of -Z (the OCS x axis is mirrored), `Some(false)` for +Z,
/// `None` for tilted planes.
fn mirrored(n: &dxf::Vector) -> Option<bool> {
    if n.x.abs() > 1e-9 || n.y.abs() > 1e-9 || n.z == 0.0 {
        return None;
    }
    Some(n.z < 0.0)
}

fn pt(x: f64, y: f64, flip: bool) -> Point2 {
    Point2::new(if flip { -x } else { x }, y)
}

fn convert(entity: &Entity, out: &mut Vec<Prim>, parsed: &mut Parsed) {
    let name = entity_name(&entity.specific);
    let tilted = |parsed: &mut Parsed| {
        parsed.skip(&format!("{name} (not in the XY plane)"));
    };
    match &entity.specific {
        EntityType::Line(l) => out.push(Prim::Line {
            a: Point2::new(l.p1.x, l.p1.y),
            b: Point2::new(l.p2.x, l.p2.y),
        }),
        EntityType::Circle(c) => match mirrored(&c.normal) {
            Some(f) => out.push(Prim::Circle {
                c: pt(c.center.x, c.center.y, f),
                r: c.radius,
            }),
            None => tilted(parsed),
        },
        EntityType::Arc(a) => match mirrored(&a.normal) {
            Some(f) => {
                let centre = pt(a.center.x, a.center.y, f);
                let (a0, a1) = (a.start_angle.to_radians(), a.end_angle.to_radians());
                if (a1 - a0).rem_euclid(TAU) < 1e-9 {
                    out.push(Prim::Circle {
                        c: centre,
                        r: a.radius,
                    });
                    return;
                }
                let at = |ang: f64| {
                    pt(
                        a.center.x + a.radius * ang.cos(),
                        a.center.y + a.radius * ang.sin(),
                        f,
                    )
                };
                let (s, e) = (at(a0), at(a1));
                // Mirroring reverses the sense: the same arc runs CCW from e to s.
                out.push(if f {
                    Prim::Arc {
                        c: centre,
                        s: e,
                        e: s,
                    }
                } else {
                    Prim::Arc { c: centre, s, e }
                });
            }
            None => tilted(parsed),
        },
        EntityType::Ellipse(e) => match mirrored(&e.normal) {
            Some(f) => {
                let c = pt(e.center.x, e.center.y, f);
                let major = pt(e.major_axis.x, e.major_axis.y, f);
                let minor = e.minor_axis_ratio * major.length();
                let sweep = ccw_sweep(e.start_parameter, e.end_parameter);
                if sweep >= TAU - 1e-9 {
                    out.push(Prim::Ellipse {
                        c,
                        major_end: c + major,
                        minor,
                    });
                } else {
                    let (t0, t1) = if f {
                        (-e.end_parameter, -e.start_parameter)
                    } else {
                        (e.start_parameter, e.end_parameter)
                    };
                    out.push(Prim::EllipticArc {
                        c,
                        major_end: c + major,
                        minor,
                        t0,
                        t1,
                    });
                }
            }
            None => tilted(parsed),
        },
        EntityType::LwPolyline(p) => match mirrored(&p.extrusion_direction) {
            Some(f) => lw_polyline(p, f, out),
            None => tilted(parsed),
        },
        EntityType::Polyline(p) => match mirrored(&p.normal) {
            Some(f) => polyline(p, f, out, parsed),
            None => tilted(parsed),
        },
        EntityType::Spline(s) => match mirrored(&s.normal) {
            Some(f) => spline(s, f, out, parsed),
            None => tilted(parsed),
        },
        _ => parsed.skip(name),
    }
}

fn entity_name(e: &EntityType) -> &'static str {
    match e {
        EntityType::Line(_) => "LINE",
        EntityType::Circle(_) => "CIRCLE",
        EntityType::Arc(_) => "ARC",
        EntityType::Ellipse(_) => "ELLIPSE",
        EntityType::LwPolyline(_) => "LWPOLYLINE",
        EntityType::Polyline(_) => "POLYLINE",
        EntityType::Spline(_) => "SPLINE",
        EntityType::Text(_) => "TEXT",
        EntityType::MText(_) => "MTEXT",
        EntityType::Insert(_) => "INSERT",
        EntityType::Solid(_) => "SOLID",
        EntityType::Face3D(_) => "3DFACE",
        EntityType::ModelPoint(_) => "POINT",
        EntityType::Leader(_) => "LEADER",
        _ => "other entity",
    }
}

/// Segments between consecutive `(point, bulge)` vertices; `closed` adds the last to first.
fn bulge_segments(verts: &[(Point2, f64)], closed: bool, out: &mut Vec<Prim>) {
    let n = verts.len();
    let count = if closed { n } else { n.saturating_sub(1) };
    for i in 0..count {
        let (p0, b) = verts[i];
        let p1 = verts[(i + 1) % n].0;
        let d = p1 - p0;
        let chord = d.length();
        if b.abs() < 1e-12 || chord < 1e-12 {
            out.push(Prim::Line { a: p0, b: p1 });
            continue;
        }
        // Centre on the left of the chord for a CCW arc (b > 0), on the right for b < 0.
        let left = Vec2::new(-d.y, d.x) / chord;
        let c = (p0 + p1) * 0.5 + left * (chord * 0.25 * (1.0 - b * b) / b);
        out.push(if b > 0.0 {
            Prim::Arc { c, s: p0, e: p1 }
        } else {
            Prim::Arc { c, s: p1, e: p0 }
        });
    }
}

fn lw_polyline(p: &LwPolyline, flip: bool, out: &mut Vec<Prim>) {
    let verts: Vec<(Point2, f64)> = p
        .vertices
        .iter()
        .map(|v| (pt(v.x, v.y, flip), if flip { -v.bulge } else { v.bulge }))
        .collect();
    bulge_segments(&verts, p.flags & 1 != 0, out);
}

fn polyline(p: &Polyline, flip: bool, out: &mut Vec<Prim>, parsed: &mut Parsed) {
    // 16 = 3D polygon mesh, 64 = polyface mesh.
    if p.flags & (16 | 64) != 0 {
        parsed.skip("POLYLINE (mesh)");
        return;
    }
    if p.flags & (2 | 4) != 0 {
        parsed.warnings.push(
            "a POLYLINE with curve/spline-fit flags was imported as its control polygon".to_owned(),
        );
    }
    let verts: Vec<(Point2, f64)> = p
        .vertices()
        .map(|v| {
            (
                pt(v.location.x, v.location.y, flip),
                if flip { -v.bulge } else { v.bulge },
            )
        })
        .collect();
    bulge_segments(&verts, p.flags & 1 != 0, out);
}

fn spline(s: &Spline, flip: bool, out: &mut Vec<Prim>, parsed: &mut Parsed) {
    let degree = usize::try_from(s.degree_of_curve).unwrap_or(0);
    let poles: Vec<Point2> = s
        .control_points
        .iter()
        .map(|p| pt(p.x, p.y, flip))
        .collect();
    let fit: Vec<Point2> = s.fit_points.iter().map(|p| pt(p.x, p.y, flip)).collect();
    let n = poles.len();

    if n > degree && degree >= 1 {
        let weights = (s.flags & 4 != 0
            && s.weight_values.len() == n
            && s.weight_values.iter().all(|w| *w > 0.0))
        .then(|| s.weight_values.clone());
        let k = &s.knot_values;
        let valid = k.len() == n + degree + 1 && k.windows(2).all(|w| w[1] >= w[0]);
        let clamped = valid
            && k[..=degree].iter().all(|v| (v - k[0]).abs() < 1e-12)
            && k[n..].iter().all(|v| (v - k[n + degree]).abs() < 1e-12)
            && k[n + degree] > k[0];
        if clamped {
            out.push(control_prim(degree, poles, weights, Some(k.clone())));
            return;
        }
        if valid && k[n] > k[degree] && fit.len() < 2 {
            // Periodic or unclamped: the sketch has clamped splines only, so sample it.
            let pts = sample_nurbs(degree, &poles, weights.as_deref(), k);
            parsed.warnings.push(format!(
                "an unclamped or periodic SPLINE was approximated by a {}-point fit spline",
                pts.len()
            ));
            out.push(Prim::Fit {
                points: pts,
                start_tangent: None,
                end_tangent: None,
            });
            return;
        }
    }
    if fit.len() >= 2 {
        let mut pts: Vec<Point2> = Vec::with_capacity(fit.len());
        for p in fit {
            if pts.last().is_none_or(|q| q.distance(p) > 1e-9) {
                pts.push(p);
            }
        }
        if pts.len() >= 2 {
            let tangent = |v: &dxf::Point, chord: f64| {
                let t = pt(v.x, v.y, flip);
                (t.length() > 1e-12).then(|| t / t.length() * chord)
            };
            let first = pts[0].distance(pts[1]);
            let last = pts[pts.len() - 1].distance(pts[pts.len() - 2]);
            let (st, et) = (
                tangent(&s.start_tangent, first),
                tangent(&s.end_tangent, last),
            );
            out.push(Prim::Fit {
                points: pts,
                start_tangent: st,
                end_tangent: et,
            });
            return;
        }
    }
    parsed.skip("SPLINE (unusable)");
}

/// Samples a (rational) B-spline over its valid domain with de Boor's algorithm.
fn sample_nurbs(
    degree: usize,
    poles: &[Point2],
    weights: Option<&[f64]>,
    k: &[f64],
) -> Vec<Point2> {
    let n = poles.len();
    let (t0, t1) = (k[degree], k[n]);
    let count = (16 * (n - degree)).max(64);
    (0..=count)
        .map(|i| {
            #[allow(clippy::cast_precision_loss, reason = "sample index")]
            let t = t0 + (t1 - t0) * (i as f64 / count as f64);
            // Span with k[s] <= t < k[s + 1], the last one at the end of the domain.
            let s = (degree..n - 1).rev().find(|&s| k[s] <= t).unwrap_or(degree);
            let mut d: Vec<[f64; 3]> = (0..=degree)
                .map(|j| {
                    let idx = s - degree + j;
                    let w = weights.map_or(1.0, |w| w[idx]);
                    [poles[idx].x * w, poles[idx].y * w, w]
                })
                .collect();
            for r in 1..=degree {
                for j in (r..=degree).rev() {
                    let (lo, hi) = (k[s - degree + j], k[s + 1 + j - r]);
                    let a = if hi > lo { (t - lo) / (hi - lo) } else { 0.0 };
                    let prev = d[j - 1];
                    for (cur, pv) in d[j].iter_mut().zip(prev) {
                        *cur = (1.0 - a) * pv + a * *cur;
                    }
                }
            }
            Point2::new(d[degree][0] / d[degree][2], d[degree][1] / d[degree][2])
        })
        .collect()
}
