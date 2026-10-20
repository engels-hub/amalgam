//! Importer for KittyCAD ezpz text problems (`test_cases/*/problem.md`), written for Amalgam.
//!
//! The format has a `# constraints` section (declarations and constraints, one per line) and a
//! `# guesses` section (`label roughly (x, y)` or `label roughly v`). Mapping:
//!
//! | ezpz | Case |
//! |---|---|
//! | `point p` | point at its guess (default origin) |
//! | `circle c` | centre point `c.center` + circle with radius guess `c.radius` (default 1) |
//! | `arc a` | points `a.center`, `a.a`, `a.b` + CCW arc `a.a → a.b`; `a.b` is projected radially onto the circle through `a.a` so the arc's internal equation holds (our arcs always have it, so `is_arc(a)` is a no-op) |
//! | `line(p, q)` | line entity |
//! | `p = (x, y)`, `p.x = v`, `p.y = v` (also `c.center…`) | horizontal/vertical distance dimensions from a synthesised fixed origin point |
//! | `coincident`, `vertical`, `horizontal` (point pairs), `distance` | `Coincident`, `Vertical`/`Horizontal(Points)`, point–point `Distance` |
//! | `parallel`, `perpendicular`, `lines_equal_length`, `lines_at_angle` (four points) | `Parallel`, `Perpendicular`, `Equal`, `Angle` on (construction) lines through the point pairs |
//! | `midpoint(p, q, m)`, `symmetric(p, q, a, b)` | `Midpoint(m, pq)`, `Symmetric(a, b, pq)` |
//! | `point_line_distance(a, p, q, d)` | signed `Distance` (sign of `d` → `Side`; ezpz's sign is also positive on the left) |
//! | `tangent(p, q, c)` | line–circle `Tangent`, `External`/`Internal` from the guesses (ezpz also fixes the side from the initial values) |
//! | `radius`, `arc_radius`, `arc_length`, `point_arc_coincident` | `Radius`, `Radius`, `ArcLength`, `PointOnCurve` |
//!
//! Semantics that differ from ezpz (our sketch model wins, so expected DOF may differ from
//! ezpz's): an arc always has its internal equation; `point_arc_coincident` is one row (on the
//! circle), not ezpz's two; `lines_at_angle` is the signed angle, not ezpz's
//! "parallel after rotation" (which also admits angle + π).

use std::collections::BTreeMap;

use amg_core::Point2;

use crate::ids::EntityId;
use crate::model::{ConstraintKind, Dimension, DimensionKind, HvTarget, Side, TangentKind};

use super::{
    Case, CaseEntity, CaseGeom, CaseSource, CorpusError, FORMAT_VERSION, SourceSet,
    placeholder_expected,
};

fn parse_err(line: usize, message: impl Into<String>) -> CorpusError {
    CorpusError::Parse {
        line,
        message: message.into(),
    }
}

/// Splits `s` at top-level commas.
fn split_args(s: &str) -> Vec<&str> {
    let mut out = Vec::new();
    let (mut depth, mut start) = (0_i32, 0);
    for (i, ch) in s.char_indices() {
        match ch {
            '(' => depth += 1,
            ')' => depth -= 1,
            ',' if depth == 0 => {
                out.push(s[start..i].trim());
                start = i + 1;
            }
            _ => {}
        }
    }
    let last = s[start..].trim();
    if !last.is_empty() {
        out.push(last);
    }
    out
}

/// `name(args)` → `(name, args)`.
fn call(s: &str) -> Option<(&str, Vec<&str>)> {
    let open = s.find('(')?;
    let inner = s[open + 1..].strip_suffix(')')?;
    Some((s[..open].trim(), split_args(inner)))
}

/// A number: literal, `-literal` or `sqrt(literal)`.
fn number(s: &str, line: usize) -> Result<f64, CorpusError> {
    let s = s.trim();
    if let Some(inner) = s.strip_prefix("sqrt(").and_then(|r| r.strip_suffix(')')) {
        return Ok(number(inner, line)?.sqrt());
    }
    s.parse()
        .map_err(|_| parse_err(line, format!("{s:?} is not a number")))
}

/// An angle `<number>deg` or `<number>rad`, in radians.
fn angle(s: &str, line: usize) -> Result<f64, CorpusError> {
    let s = s.trim();
    if let Some(v) = s.strip_suffix("deg") {
        Ok(number(v, line)?.to_radians())
    } else if let Some(v) = s.strip_suffix("rad") {
        number(v, line)
    } else {
        Err(parse_err(line, format!("{s:?} is not an angle (deg/rad)")))
    }
}

/// `(x, y)`.
fn tuple(s: &str, line: usize) -> Result<[f64; 2], CorpusError> {
    let inner = s
        .trim()
        .strip_prefix('(')
        .and_then(|r| r.strip_suffix(')'))
        .ok_or_else(|| parse_err(line, format!("{s:?} is not a point (x, y)")))?;
    match split_args(inner)[..] {
        [x, y] => Ok([number(x, line)?, number(y, line)?]),
        _ => Err(parse_err(line, format!("{s:?} is not a point (x, y)"))),
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Decl {
    Point,
    Circle,
    Arc,
}

struct Builder {
    entities: Vec<CaseEntity>,
    constraints: Vec<ConstraintKind>,
    points: BTreeMap<String, u32>,
    curves: BTreeMap<String, u32>,
    lines: BTreeMap<(u32, u32), u32>,
    origin: Option<u32>,
}

impl Builder {
    fn push(&mut self, name: String, geom: CaseGeom, construction: bool) -> u32 {
        self.entities.push(CaseEntity {
            name,
            geom,
            construction,
        });
        (self.entities.len() - 1) as u32
    }

    fn pos(&self, i: u32) -> Point2 {
        match self.entities[i as usize].geom {
            CaseGeom::Point { at } => Point2::new(at[0], at[1]),
            _ => Point2::NAN,
        }
    }

    fn point(&self, label: &str, line: usize) -> Result<u32, CorpusError> {
        self.points
            .get(label.trim())
            .copied()
            .ok_or_else(|| parse_err(line, format!("unknown point {label:?}")))
    }

    fn curve(&self, label: &str, line: usize) -> Result<u32, CorpusError> {
        self.curves
            .get(label.trim())
            .copied()
            .ok_or_else(|| parse_err(line, format!("unknown circle/arc {label:?}")))
    }

    /// The line `p → q` (declared or a new construction line).
    fn line(&mut self, p: u32, q: u32, construction: bool) -> Result<u32, CorpusError> {
        if p == q {
            return Err(CorpusError::Unsupported(
                "line between a point and itself".into(),
            ));
        }
        if let Some(&l) = self.lines.get(&(p, q)) {
            if !construction {
                self.entities[l as usize].construction = false;
            }
            return Ok(l);
        }
        let name = format!(
            "line({},{})",
            self.entities[p as usize].name, self.entities[q as usize].name
        );
        let l = self.push(name, CaseGeom::Line { start: p, end: q }, construction);
        self.lines.insert((p, q), l);
        Ok(l)
    }

    fn line_of(&mut self, a: &str, b: &str, line: usize) -> Result<u32, CorpusError> {
        let (p, q) = (self.point(a, line)?, self.point(b, line)?);
        self.line(p, q, true)
    }

    fn origin(&mut self) -> u32 {
        if let Some(o) = self.origin {
            return o;
        }
        let o = self.push("origin".into(), CaseGeom::Point { at: [0.0, 0.0] }, true);
        self.constraints.push(ConstraintKind::Fix {
            entity: EntityId(o),
        });
        self.origin = Some(o);
        o
    }

    /// `p.x = v` (`horizontal`) or `p.y = v` as a distance from the fixed origin.
    fn anchor(&mut self, p: u32, v: f64, horizontal: bool) {
        let o = self.origin();
        let side = if v < 0.0 {
            Side::Negative
        } else {
            Side::Positive
        };
        let (a, b) = (EntityId(o), EntityId(p));
        let kind = if horizontal {
            DimensionKind::HorizontalDistance { a, b, side }
        } else {
            DimensionKind::VerticalDistance { a, b, side }
        };
        self.constraints.push(dim(kind, v.abs()));
    }
}

fn dim(kind: DimensionKind, value: f64) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving: true,
        label_pos: Point2::ZERO,
    })
}

/// Imports an ezpz problem as case `id`; `origin` is the upstream path recorded in the case.
/// The expectation is a placeholder that the set's `expected.json` replaces.
pub fn import(text: &str, id: &str, origin: &str) -> Result<Case, CorpusError> {
    let mut constraints = Vec::new();
    let mut guesses = Vec::new();
    let mut section = 0;
    for (i, raw) in text.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(h) = line.strip_prefix('#') {
            section = match h.trim() {
                "constraints" => 1,
                "guesses" => 2,
                other => return Err(parse_err(i + 1, format!("unknown section {other:?}"))),
            };
            continue;
        }
        match section {
            1 => constraints.push((i + 1, line)),
            2 => guesses.push((i + 1, line)),
            _ => return Err(parse_err(i + 1, "text before `# constraints`")),
        }
    }

    // Guesses: label → point or scalar.
    let mut point_guess: BTreeMap<&str, [f64; 2]> = BTreeMap::new();
    let mut scalar_guess: BTreeMap<&str, f64> = BTreeMap::new();
    for &(ln, g) in &guesses {
        let (label, value) = g
            .split_once(" roughly ")
            .ok_or_else(|| parse_err(ln, "expected `<label> roughly <value>`"))?;
        let value = value.trim();
        if value.starts_with('(') {
            point_guess.insert(label.trim(), tuple(value, ln)?);
        } else {
            scalar_guess.insert(label.trim(), number(value, ln)?);
        }
    }

    let mut b = Builder {
        entities: Vec::new(),
        constraints: Vec::new(),
        points: BTreeMap::new(),
        curves: BTreeMap::new(),
        lines: BTreeMap::new(),
        origin: None,
    };

    // Pass 1: declarations, points first.
    let mut decls = Vec::new();
    for &(ln, s) in &constraints {
        for (kw, d) in [
            ("point ", Decl::Point),
            ("circle ", Decl::Circle),
            ("arc ", Decl::Arc),
        ] {
            if let Some(label) = s.strip_prefix(kw) {
                decls.push((ln, label.trim(), d));
            }
        }
    }
    let guess = |label: &str| point_guess.get(label).copied().unwrap_or([0.0, 0.0]);
    for &(ln, label, d) in &decls {
        let names: Vec<String> = match d {
            Decl::Point => vec![label.to_string()],
            Decl::Circle => vec![format!("{label}.center")],
            Decl::Arc => ["center", "a", "b"]
                .iter()
                .map(|s| format!("{label}.{s}"))
                .collect(),
        };
        for n in names {
            if b.points.contains_key(&n) {
                return Err(parse_err(ln, format!("{n} declared twice")));
            }
            let at = guess(&n);
            let i = b.push(n.clone(), CaseGeom::Point { at }, false);
            b.points.insert(n, i);
        }
    }
    for &(ln, label, d) in &decls {
        let geom = match d {
            Decl::Point => continue,
            Decl::Circle => {
                let r = scalar_guess
                    .get(format!("{label}.radius").as_str())
                    .copied()
                    .unwrap_or(1.0)
                    .abs();
                CaseGeom::Circle {
                    center: b.point(&format!("{label}.center"), ln)?,
                    radius: if r > 0.0 { r } else { 1.0 },
                }
            }
            Decl::Arc => {
                let c = b.point(&format!("{label}.center"), ln)?;
                let s = b.point(&format!("{label}.a"), ln)?;
                let e = b.point(&format!("{label}.b"), ln)?;
                let (pc, ps, pe) = (b.pos(c), b.pos(s), b.pos(e));
                let r = ps.distance(pc);
                let de = pe - pc;
                if r <= 0.0 || de.length() <= 0.0 {
                    return Err(CorpusError::Unsupported("degenerate arc guess".into()));
                }
                let pe = pc + de * (r / de.length());
                b.entities[e as usize].geom = CaseGeom::Point { at: [pe.x, pe.y] };
                CaseGeom::Arc {
                    center: c,
                    start: s,
                    end: e,
                }
            }
        };
        let i = b.push(label.to_string(), geom, false);
        b.curves.insert(label.to_string(), i);
    }

    // Pass 2: constraints.
    let e = EntityId;
    for &(ln, s) in &constraints {
        if decls.iter().any(|&(l, _, _)| l == ln) {
            continue;
        }
        if let Some((lhs, rhs)) = s.split_once('=') {
            let (lhs, rhs) = (lhs.trim(), rhs.trim());
            if let Some(p) = lhs
                .strip_suffix(".x")
                .and_then(|l| b.points.get(l).copied())
            {
                let v = number(rhs, ln)?;
                b.anchor(p, v, true);
            } else if let Some(p) = lhs
                .strip_suffix(".y")
                .and_then(|l| b.points.get(l).copied())
            {
                let v = number(rhs, ln)?;
                b.anchor(p, v, false);
            } else {
                let p = b.point(lhs, ln)?;
                let [x, y] = tuple(rhs, ln)?;
                b.anchor(p, x, true);
                b.anchor(p, y, false);
            }
            continue;
        }
        let (name, args) = call(s).ok_or_else(|| parse_err(ln, format!("cannot parse {s:?}")))?;
        let want = |n: usize| -> Result<(), CorpusError> {
            if args.len() == n {
                Ok(())
            } else {
                Err(parse_err(ln, format!("{name} takes {n} arguments")))
            }
        };
        let k = match name {
            "line" => {
                want(2)?;
                let (p, q) = (b.point(args[0], ln)?, b.point(args[1], ln)?);
                b.line(p, q, false)?;
                continue;
            }
            "is_arc" => {
                want(1)?;
                b.curve(args[0], ln)?;
                continue;
            }
            "coincident" => {
                want(2)?;
                ConstraintKind::Coincident {
                    a: e(b.point(args[0], ln)?),
                    b: e(b.point(args[1], ln)?),
                }
            }
            "vertical" | "horizontal" => {
                want(2)?;
                let t = HvTarget::Points(e(b.point(args[0], ln)?), e(b.point(args[1], ln)?));
                if name == "vertical" {
                    ConstraintKind::Vertical(t)
                } else {
                    ConstraintKind::Horizontal(t)
                }
            }
            "distance" => {
                want(3)?;
                let v = number(args[2], ln)?;
                if v < 0.0 {
                    return Err(CorpusError::Unsupported("negative point distance".into()));
                }
                dim(
                    DimensionKind::Distance {
                        a: e(b.point(args[0], ln)?),
                        b: e(b.point(args[1], ln)?),
                        side: Side::Positive,
                    },
                    v,
                )
            }
            "parallel" | "perpendicular" | "lines_equal_length" => {
                want(4)?;
                let la = e(b.line_of(args[0], args[1], ln)?);
                let lb = e(b.line_of(args[2], args[3], ln)?);
                match name {
                    "parallel" => ConstraintKind::Parallel { a: la, b: lb },
                    "perpendicular" => ConstraintKind::Perpendicular { a: la, b: lb },
                    _ => ConstraintKind::Equal { a: la, b: lb },
                }
            }
            "lines_at_angle" => {
                want(5)?;
                let la = e(b.line_of(args[0], args[1], ln)?);
                let lb = e(b.line_of(args[2], args[3], ln)?);
                dim(
                    DimensionKind::Angle {
                        a: la,
                        b: lb,
                        branch: crate::model::AngleBranch {
                            reverse_a: false,
                            reverse_b: false,
                        },
                    },
                    angle(args[4], ln)?,
                )
            }
            "midpoint" => {
                want(3)?;
                let l = b.line_of(args[0], args[1], ln)?;
                ConstraintKind::Midpoint {
                    point: e(b.point(args[2], ln)?),
                    line: e(l),
                }
            }
            "symmetric" => {
                want(4)?;
                let l = b.line_of(args[0], args[1], ln)?;
                ConstraintKind::Symmetric {
                    a: e(b.point(args[2], ln)?),
                    b: e(b.point(args[3], ln)?),
                    axis: e(l),
                }
            }
            "point_line_distance" => {
                want(4)?;
                let p = b.point(args[0], ln)?;
                let l = b.line_of(args[1], args[2], ln)?;
                let v = number(args[3], ln)?;
                dim(
                    DimensionKind::Distance {
                        a: e(p),
                        b: e(l),
                        side: if v < 0.0 {
                            Side::Negative
                        } else {
                            Side::Positive
                        },
                    },
                    v.abs(),
                )
            }
            "tangent" => {
                want(3)?;
                let (p, q) = (b.point(args[0], ln)?, b.point(args[1], ln)?);
                let c = b.curve(args[2], ln)?;
                let l = b.line(p, q, true)?;
                let CaseGeom::Circle { center, .. } = b.entities[c as usize].geom else {
                    return Err(CorpusError::Unsupported("tangent to a non-circle".into()));
                };
                let (a0, a1, pc) = (b.pos(p), b.pos(q), b.pos(center));
                let left = (a1 - a0).perp_dot(pc - a0) > 0.0;
                ConstraintKind::Tangent {
                    a: e(l),
                    b: e(c),
                    kind: if left {
                        TangentKind::External
                    } else {
                        TangentKind::Internal
                    },
                }
            }
            "radius" | "arc_radius" => {
                want(2)?;
                dim(
                    DimensionKind::Radius {
                        curve: e(b.curve(args[0], ln)?),
                    },
                    number(args[1], ln)?,
                )
            }
            "arc_length" => {
                want(2)?;
                dim(
                    DimensionKind::ArcLength {
                        arc: e(b.curve(args[0], ln)?),
                    },
                    number(args[1], ln)?,
                )
            }
            "point_arc_coincident" => {
                want(2)?;
                ConstraintKind::PointOnCurve {
                    point: e(b.point(args[0], ln)?),
                    curve: e(b.curve(args[1], ln)?),
                }
            }
            other => return Err(CorpusError::Unsupported(format!("ezpz constraint {other}"))),
        };
        b.constraints.push(k);
    }

    Ok(Case {
        format: FORMAT_VERSION,
        id: id.to_string(),
        source: CaseSource {
            set: SourceSet::Ezpz,
            origin: origin.to_string(),
            licence: "MIT".into(),
        },
        description: String::new(),
        tags: Vec::new(),
        entities: b.entities,
        constraints: b.constraints,
        expected: placeholder_expected(),
        drag: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbers_and_angles() {
        assert!((number("sqrt(16)", 1).unwrap() - 4.0).abs() < 1e-15);
        assert!((number("-1.5", 1).unwrap() + 1.5).abs() < 1e-15);
        assert!((angle("180deg", 1).unwrap() - std::f64::consts::PI).abs() < 1e-15);
        let t = tuple("(1, -2)", 1).unwrap();
        assert!((t[0] - 1.0).abs() < 1e-15 && (t[1] + 2.0).abs() < 1e-15);
        assert!(number("abc", 3).is_err());
    }

    #[test]
    fn imports_a_small_problem() {
        let text = "# constraints\npoint p\npoint q\nline(p, q)\np = (0, 0)\nq.y = 0\n\
                    distance(p, q, 4)\n\n# guesses\np roughly (1, 1)\nq roughly (3, 1)\n";
        let case = import(text, "ezpz/t", "t").unwrap();
        // p, q, line, origin.
        assert_eq!(case.entities.len(), 4);
        assert!(
            !case.entities[2].construction,
            "declared lines are real geometry"
        );
        // origin fix, p.x, p.y, q.y, distance.
        assert_eq!(case.constraints.len(), 5);
        case.build().unwrap();
    }

    #[test]
    fn unknown_constraints_are_unsupported() {
        let text = "# constraints\npoint p\nfrobnicate(p)\n\n# guesses\np roughly (0, 0)\n";
        assert!(matches!(
            import(text, "ezpz/t", "t"),
            Err(CorpusError::Unsupported(_))
        ));
    }
}
