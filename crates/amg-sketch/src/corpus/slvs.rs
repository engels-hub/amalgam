//! Importer for SolveSpace `.slvs` text files (2D subset), written for Amalgam.
//!
//! The file is a sequence of `Key=value` lines grouped into records, each closed by an
//! `AddGroup` / `AddParam` / `AddRequest` / `AddEntity` / `AddConstraint` line. The importer
//! reads the entities and constraints of a sketch that lies in **one workplane parallel to
//! XY** (SolveSpace's default "sketch-in-plane" group) and maps them onto [`Case`]s:
//!
//! | SolveSpace | Case |
//! |---|---|
//! | `POINT_IN_2D` (2001), `LINE_SEGMENT` (11000), `CIRCLE` (13000, radius from its `DISTANCE` entity), `ARC_OF_CIRCLE` (14000) | point, line, circle, arc (positions from `actPoint` / `actDistance`) |
//! | `POINTS_COINCIDENT` 20, `PT_ON_LINE` 42, `PT_ON_CIRCLE` 100, `AT_MIDPOINT` 70 (line–point) | `Coincident`, `PointOnCurve`, `Midpoint` |
//! | `HORIZONTAL` 80 / `VERTICAL` 81 (line or two points) | `Horizontal` / `Vertical` |
//! | `PARALLEL` 121, `PERPENDICULAR` 122, `EQUAL_LENGTH_LINES` 50, `EQUAL_RADIUS` 130, `EQUAL_LINE_ARC_LEN` 55 | `Parallel`, `Perpendicular`, `Equal` |
//! | `ARC_LINE_TANGENT` 123, `CURVE_CURVE_TANGENT` 125 (arcs and lines) | `Tangent::AtEndpoint` |
//! | `SYMMETRIC_LINE` 63; `SYMMETRIC` 60 about the YZ/ZX reference plane; `SYMMETRIC_HORIZ`/`VERT` 61/62 | `Symmetric` (about a synthesised fixed axis for 60–62) |
//! | `PT_PT_DISTANCE` 30, `PT_LINE_DISTANCE` 32, `DIAMETER` 90, `ANGLE` 120 | `Distance`, `Diameter`, `Angle` dimensions (`reference=1` → driven) |
//! | `WHERE_DRAGGED` 200 | `Fix` |
//! | `COMMENT` 1000 | ignored |
//!
//! References to the origin become a synthesised fixed point at (0, 0). Signs that SolveSpace
//! keeps implicitly (which side of a line a point is on, the angle branch, whether tangent
//! directions agree) are read from the stored configuration, which SolveSpace saves solved.
//! Everything else (3D points, cubics, other constraint types, a non-XY workplane) returns
//! [`CorpusError::Unsupported`].

use std::collections::BTreeMap;

use amg_core::Point2;

use crate::ids::EntityId;
use crate::model::{
    AngleBranch, ConstraintKind, Dimension, DimensionKind, End, HvTarget, Side, TangentKind,
};

use super::{
    Case, CaseEntity, CaseGeom, CaseSource, CorpusError, FORMAT_VERSION, SourceSet,
    placeholder_expected,
};

/// One record: its kind (`Param`, `Entity`, ...) and fields without the kind prefix.
#[derive(Clone, Debug, Default)]
struct Record {
    kind: String,
    fields: BTreeMap<String, String>,
    line: usize,
}

impl Record {
    fn get(&self, key: &str) -> Option<&str> {
        self.fields.get(key).map(String::as_str)
    }
    fn num(&self, key: &str) -> Result<f64, CorpusError> {
        match self.get(key) {
            None => Ok(0.0),
            Some(v) => v.trim().parse().map_err(|_| CorpusError::Parse {
                line: self.line,
                message: format!("{key}={v} is not a number"),
            }),
        }
    }
    fn int(&self, key: &str) -> i64 {
        self.get(key)
            .and_then(|v| v.trim().parse().ok())
            .unwrap_or(0)
    }
    fn handle(&self, key: &str) -> Option<u32> {
        self.get(key)
            .and_then(|v| u32::from_str_radix(v.trim(), 16).ok())
            .filter(|&h| h != 0)
    }
}

/// Splits the text into records.
fn records(bytes: &[u8]) -> Result<Vec<Record>, CorpusError> {
    let text = String::from_utf8_lossy(bytes);
    let mut out = Vec::new();
    let mut cur = Record::default();
    let mut saw_header = false;
    for (i, raw) in text.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if !saw_header {
            if !line.contains("SolveSpaceREV") {
                return Err(CorpusError::Parse {
                    line: i + 1,
                    message: "missing SolveSpace header".into(),
                });
            }
            saw_header = true;
            continue;
        }
        if let Some(kind) = line.strip_prefix("Add")
            && !line.contains('=')
        {
            cur.kind = kind.to_string();
            out.push(std::mem::take(&mut cur));
            continue;
        }
        // Lines without `=` are continuations of multi-line values (`Group.remap={ ... }`),
        // which the importer does not need.
        let Some((key, value)) = line.split_once('=') else {
            continue;
        };
        // "Entity.point[0].v" -> "point[0].v"; the record kind comes from the Add line.
        let key = key.split_once('.').map_or(key, |(_, k)| k);
        if cur.fields.is_empty() {
            cur.line = i + 1;
        }
        cur.fields
            .entry(key.to_string())
            .or_insert_with(|| value.to_string());
    }
    if !saw_header {
        return Err(CorpusError::Parse {
            line: 0,
            message: "empty file".into(),
        });
    }
    Ok(out)
}

fn unsupported(msg: impl Into<String>) -> CorpusError {
    CorpusError::Unsupported(msg.into())
}

const POINT_IN_3D: i64 = 2000;
const POINT_IN_2D: i64 = 2001;
const POINT_N_COPY: i64 = 2012;
const NORMAL_IN_2D: i64 = 3001;
const DISTANCE: i64 = 4000;
const WORKPLANE: i64 = 10000;
const LINE_SEGMENT: i64 = 11000;
const CUBIC: i64 = 12000;
const CUBIC_PERIODIC: i64 = 12001;
const CIRCLE: i64 = 13000;
const ARC_OF_CIRCLE: i64 = 14000;

/// Handles of the reference group's origin and its workplane copy.
const ORIGIN: u32 = 0x0001_0001;
/// Reference planes YZ and ZX (normals along x and y).
const PLANE_YZ: u32 = 0x0002_0000;
const PLANE_ZX: u32 = 0x0003_0000;

struct Builder {
    entities: Vec<CaseEntity>,
    constraints: Vec<ConstraintKind>,
    /// SolveSpace entity handle → case index.
    index: BTreeMap<u32, u32>,
    origin: Option<u32>,
    axes: BTreeMap<bool, u32>,
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

    /// Fixed construction axis through the origin: the y axis (`vertical`) or the x axis.
    fn axis(&mut self, vertical: bool) -> u32 {
        if let Some(&a) = self.axes.get(&vertical) {
            return a;
        }
        let o = self.origin();
        let end = if vertical { [0.0, 10.0] } else { [10.0, 0.0] };
        let name = if vertical { "y_axis" } else { "x_axis" };
        let p = self.push(format!("{name}_end"), CaseGeom::Point { at: end }, true);
        let l = self.push(name.into(), CaseGeom::Line { start: o, end: p }, true);
        self.constraints.push(ConstraintKind::Fix {
            entity: EntityId(l),
        });
        self.axes.insert(vertical, l);
        l
    }

    fn geom(&self, i: u32) -> &CaseGeom {
        &self.entities[i as usize].geom
    }

    /// The point indices of a line or arc end.
    fn end_point(&self, curve: u32, end: End) -> Option<u32> {
        match (self.geom(curve).clone(), end) {
            (CaseGeom::Line { start, .. } | CaseGeom::Arc { start, .. }, End::Start) => Some(start),
            (CaseGeom::Line { end, .. } | CaseGeom::Arc { end, .. }, End::End) => Some(end),
            _ => None,
        }
    }

    /// Direction of travel at an end (line: start→end; arc: CCW tangent).
    fn travel(&self, curve: u32, end: End) -> Option<Point2> {
        match *self.geom(curve) {
            CaseGeom::Line { start, end: e } => Some(self.pos(e) - self.pos(start)),
            CaseGeom::Arc { center, .. } => {
                let p = self.pos(self.end_point(curve, end)?) - self.pos(center);
                Some(Point2::new(-p.y, p.x))
            }
            _ => None,
        }
    }

    fn is_line(&self, i: u32) -> bool {
        matches!(self.geom(i), CaseGeom::Line { .. })
    }
}

fn side(v: f64) -> Side {
    if v < 0.0 {
        Side::Negative
    } else {
        Side::Positive
    }
}

fn dim(kind: DimensionKind, value: f64, driving: bool) -> ConstraintKind {
    ConstraintKind::Dimension(Dimension {
        kind,
        value,
        driving,
        label_pos: Point2::ZERO,
    })
}

/// Imports a `.slvs` file as case `id`; `origin` is the upstream path recorded in the case.
/// The expectation is a placeholder that the set's `expected.json` replaces.
pub fn import(bytes: &[u8], id: &str, origin: &str) -> Result<Case, CorpusError> {
    let recs = records(bytes)?;
    let mut request_group: BTreeMap<u32, u32> = BTreeMap::new();
    let mut entities: BTreeMap<u32, &Record> = BTreeMap::new();
    let mut constraints = Vec::new();
    for r in &recs {
        match r.kind.as_str() {
            "Request" => {
                if let (Some(h), Some(g)) = (r.handle("h.v"), r.handle("group.v")) {
                    request_group.insert(h, g);
                }
            }
            "Entity" => {
                if let Some(h) = r.handle("h.v") {
                    entities.insert(h, r);
                }
            }
            "Constraint" => constraints.push(r),
            _ => {}
        }
    }
    // Entities of the user's sketch: not workplane-derived (0x8000_0000) and not from a
    // request of the reference group 1.
    let is_sketch =
        |h: u32| h & 0x8000_0000 == 0 && request_group.get(&(h >> 16)).is_some_and(|&g| g != 1);
    let mut workplane = None;
    for (&h, e) in &entities {
        if !is_sketch(h) {
            continue;
        }
        match e.int("type") {
            POINT_IN_2D | LINE_SEGMENT | CIRCLE | ARC_OF_CIRCLE | NORMAL_IN_2D | DISTANCE => {
                let wp = e.handle("workplane.v");
                if wp.is_none() || workplane.is_some_and(|w| Some(w) != wp) {
                    return Err(unsupported("entities in 3D or in more than one workplane"));
                }
                workplane = wp;
            }
            POINT_IN_3D => return Err(unsupported("3D point")),
            CUBIC | CUBIC_PERIODIC => return Err(unsupported("cubic spline")),
            t => return Err(unsupported(format!("entity type {t}"))),
        }
    }
    if let Some(wp) = workplane {
        // The workplane must be parallel to XY through the origin (identity normal).
        let ok = entities.get(&wp).is_some_and(|w| {
            w.int("type") == WORKPLANE
                && w.handle("normal.v")
                    .and_then(|n| entities.get(&n))
                    .is_some_and(|n| {
                        n.num("actNormal.w").is_ok_and(|v| (v - 1.0).abs() < 1e-12)
                            && ["actNormal.vx", "actNormal.vy", "actNormal.vz"]
                                .iter()
                                .all(|k| n.num(k).is_ok_and(|v| v.abs() < 1e-12))
                    })
                && w.handle("point[0].v")
                    .and_then(|p| entities.get(&p))
                    .is_some_and(|p| {
                        ["actPoint.x", "actPoint.y", "actPoint.z"]
                            .iter()
                            .all(|k| p.num(k).is_ok_and(|v| v.abs() < 1e-12))
                    })
        });
        if !ok {
            return Err(unsupported("workplane is not the XY plane"));
        }
    }

    let mut b = Builder {
        entities: Vec::new(),
        constraints: Vec::new(),
        index: BTreeMap::new(),
        origin: None,
        axes: BTreeMap::new(),
    };
    let name = |h: u32| format!("e{h:08x}");
    // Points first (curves reference them), then curves, both in handle order.
    for (&h, e) in &entities {
        if is_sketch(h) && e.int("type") == POINT_IN_2D {
            let at = [e.num("actPoint.x")?, e.num("actPoint.y")?];
            let i = b.push(name(h), CaseGeom::Point { at }, e.int("construction") != 0);
            b.index.insert(h, i);
        }
    }
    let point = |b: &Builder, e: &Record, k: usize| -> Result<u32, CorpusError> {
        e.handle(&format!("point[{k}].v"))
            .and_then(|p| b.index.get(&p).copied())
            .ok_or_else(|| CorpusError::Parse {
                line: e.line,
                message: format!("point[{k}] is missing"),
            })
    };
    for (&h, e) in &entities {
        if !is_sketch(h) {
            continue;
        }
        let construction = e.int("construction") != 0;
        let geom = match e.int("type") {
            LINE_SEGMENT => CaseGeom::Line {
                start: point(&b, e, 0)?,
                end: point(&b, e, 1)?,
            },
            CIRCLE => {
                let r = e
                    .handle("distance.v")
                    .and_then(|d| entities.get(&d))
                    .ok_or_else(|| CorpusError::Parse {
                        line: e.line,
                        message: "circle without distance".into(),
                    })?
                    .num("actDistance")?;
                CaseGeom::Circle {
                    center: point(&b, e, 0)?,
                    radius: r.abs(),
                }
            }
            ARC_OF_CIRCLE => CaseGeom::Arc {
                center: point(&b, e, 0)?,
                start: point(&b, e, 1)?,
                end: point(&b, e, 2)?,
            },
            _ => continue,
        };
        let i = b.push(name(h), geom, construction);
        b.index.insert(h, i);
    }

    for r in constraints {
        if r.handle("workplane.v") != workplane || workplane.is_none() {
            // An empty sketch with only a comment has no workplane entities; a comment is
            // harmless either way.
            if r.int("type") == 1000 {
                continue;
            }
            return Err(unsupported("constraint outside the sketch workplane (3D)"));
        }
        let kind = map_constraint(&mut b, &entities, r)?;
        if let Some(k) = kind {
            b.constraints.push(k);
        }
    }

    Ok(Case {
        format: FORMAT_VERSION,
        id: id.to_string(),
        source: CaseSource {
            set: SourceSet::SolveSpace,
            origin: origin.to_string(),
            licence: "GPL-3.0-or-later".into(),
        },
        description: String::new(),
        tags: Vec::new(),
        entities: b.entities,
        constraints: b.constraints,
        expected: placeholder_expected(),
        drag: None,
    })
}

fn map_constraint(
    b: &mut Builder,
    entities: &BTreeMap<u32, &Record>,
    r: &Record,
) -> Result<Option<ConstraintKind>, CorpusError> {
    use ConstraintKind as C;
    let ty = r.int("type");
    let driving = r.int("reference") == 0;
    // A point handle: a sketch point or the origin (also via its workplane copy).
    let pt = |b: &mut Builder, key: &str| -> Result<u32, CorpusError> {
        let h = r
            .handle(key)
            .ok_or_else(|| unsupported(format!("constraint {ty} without {key}")))?;
        if let Some(&i) = b.index.get(&h) {
            return Ok(i);
        }
        let is_origin = h == ORIGIN
            || entities.get(&h).is_some_and(|e| {
                e.int("type") == POINT_N_COPY
                    && e.handle("point[0].v").is_none()
                    && h & 0x8000_0000 != 0
            });
        if is_origin {
            Ok(b.origin())
        } else {
            Err(unsupported(format!("point {h:08x} is not in the sketch")))
        }
    };
    let ent = |b: &Builder, key: &str| -> Result<u32, CorpusError> {
        r.handle(key)
            .and_then(|h| b.index.get(&h).copied())
            .ok_or_else(|| unsupported(format!("constraint {ty}: {key} is not a sketch entity")))
    };
    let e = |i: u32| EntityId(i);
    let val = r.num("valA")?;
    let k = match ty {
        20 => C::Coincident {
            a: e(pt(b, "ptA.v")?),
            b: e(pt(b, "ptB.v")?),
        },
        30 => {
            let (p, q) = (pt(b, "ptA.v")?, pt(b, "ptB.v")?);
            let measured = b.pos(p).distance(b.pos(q));
            dim(
                DimensionKind::Distance {
                    a: e(p),
                    b: e(q),
                    side: Side::Positive,
                },
                if driving { val.abs() } else { measured },
                driving,
            )
        }
        32 => {
            let p = pt(b, "ptA.v")?;
            let l = ent(b, "entityA.v")?;
            let CaseGeom::Line { start, end } = *b.geom(l) else {
                return Err(unsupported("PT_LINE_DISTANCE to a non-line"));
            };
            let (a0, a1) = (b.pos(start), b.pos(end));
            let d = a1 - a0;
            let s = d.perp_dot(b.pos(p) - a0) / d.length();
            dim(
                DimensionKind::Distance {
                    a: e(p),
                    b: e(l),
                    side: side(s),
                },
                if driving { val.abs() } else { s.abs() },
                driving,
            )
        }
        42 | 100 => C::PointOnCurve {
            point: e(pt(b, "ptA.v")?),
            curve: e(ent(b, "entityA.v")?),
        },
        50 | 55 | 130 => C::Equal {
            a: e(ent(b, "entityA.v")?),
            b: e(ent(b, "entityB.v")?),
        },
        60 => {
            let vertical = match r.handle("entityA.v") {
                Some(PLANE_YZ) => true,
                Some(PLANE_ZX) => false,
                _ => return Err(unsupported("SYMMETRIC about a plane other than YZ/ZX")),
            };
            let axis = b.axis(vertical);
            C::Symmetric {
                a: e(pt(b, "ptA.v")?),
                b: e(pt(b, "ptB.v")?),
                axis: e(axis),
            }
        }
        61 | 62 => {
            // HORIZ: same v, opposite u (mirror about the vertical axis); VERT: the reverse.
            let axis = b.axis(ty == 61);
            C::Symmetric {
                a: e(pt(b, "ptA.v")?),
                b: e(pt(b, "ptB.v")?),
                axis: e(axis),
            }
        }
        63 => C::Symmetric {
            a: e(pt(b, "ptA.v")?),
            b: e(pt(b, "ptB.v")?),
            axis: e(ent(b, "entityA.v")?),
        },
        70 => {
            let l = ent(b, "entityA.v").map_err(|_| unsupported("AT_MIDPOINT on a plane"))?;
            C::Midpoint {
                point: e(pt(b, "ptA.v")?),
                line: e(l),
            }
        }
        80 | 81 => {
            let t = match r.handle("entityA.v") {
                Some(_) => HvTarget::Line(e(ent(b, "entityA.v")?)),
                None => HvTarget::Points(e(pt(b, "ptA.v")?), e(pt(b, "ptB.v")?)),
            };
            if ty == 80 {
                C::Horizontal(t)
            } else {
                C::Vertical(t)
            }
        }
        90 => {
            let c = ent(b, "entityA.v")?;
            let measured = match *b.geom(c) {
                CaseGeom::Circle { radius, .. } => 2.0 * radius,
                CaseGeom::Arc { center, start, .. } => 2.0 * b.pos(start).distance(b.pos(center)),
                _ => return Err(unsupported("DIAMETER of a non-circle")),
            };
            dim(
                DimensionKind::Diameter { curve: e(c) },
                if driving { val.abs() } else { measured },
                driving,
            )
        }
        120 => {
            let (la, lb) = (ent(b, "entityA.v")?, ent(b, "entityB.v")?);
            let branch = AngleBranch {
                reverse_a: r.int("other") != 0,
                reverse_b: false,
            };
            let dir = |b: &Builder, l: u32, rev: bool| match *b.geom(l) {
                CaseGeom::Line { start, end } => {
                    let d = b.pos(end) - b.pos(start);
                    Some(if rev { -d } else { d })
                }
                _ => None,
            };
            let (Some(da), Some(db)) = (dir(b, la, branch.reverse_a), dir(b, lb, false)) else {
                return Err(unsupported("ANGLE between non-lines"));
            };
            let measured = da.perp_dot(db).atan2(da.dot(db));
            let value = if driving {
                // SolveSpace's angle is unsigned; keep the sign of the stored configuration.
                measured.signum() * val.abs().to_radians()
            } else {
                measured
            };
            dim(
                DimensionKind::Angle {
                    a: e(la),
                    b: e(lb),
                    branch,
                },
                value,
                driving,
            )
        }
        121 => C::Parallel {
            a: e(ent(b, "entityA.v")?),
            b: e(ent(b, "entityB.v")?),
        },
        122 => C::Perpendicular {
            a: e(ent(b, "entityA.v")?),
            b: e(ent(b, "entityB.v")?),
        },
        123 | 125 => {
            let (ca, cb) = (ent(b, "entityA.v")?, ent(b, "entityB.v")?);
            let end_of = |flag: i64| if flag != 0 { End::End } else { End::Start };
            let a_end = end_of(r.int("other"));
            let b_end = if ty == 125 {
                end_of(r.int("other2"))
            } else {
                // ARC_LINE_TANGENT names only the arc end; take the line end at that point.
                let p = b
                    .end_point(ca, a_end)
                    .ok_or_else(|| unsupported("ARC_LINE_TANGENT on a non-arc"))?;
                let at = b.pos(p);
                let ds = b.end_point(cb, End::Start).map(|q| b.pos(q).distance(at));
                let de = b.end_point(cb, End::End).map(|q| b.pos(q).distance(at));
                match (ds, de) {
                    (Some(ds), Some(de)) if de < ds => End::End,
                    (Some(_), Some(_)) => End::Start,
                    _ => return Err(unsupported("ARC_LINE_TANGENT on a non-line")),
                }
            };
            if b.is_line(ca) && b.is_line(cb) {
                return Err(unsupported("line–line endpoint tangency"));
            }
            let (Some(ta), Some(tb)) = (b.travel(ca, a_end), b.travel(cb, b_end)) else {
                return Err(unsupported("tangency between unsupported curves"));
            };
            C::Tangent {
                a: e(ca),
                b: e(cb),
                kind: TangentKind::AtEndpoint {
                    a_end,
                    b_end,
                    reversed: ta.dot(tb) < 0.0,
                },
            }
        }
        200 => C::Fix {
            entity: e(pt(b, "ptA.v")?),
        },
        1000 => return Ok(None),
        t => return Err(unsupported(format!("constraint type {t}"))),
    };
    Ok(Some(k))
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEADER: &str = "\u{b1}\u{b2}\u{b3}SolveSpaceREVa\n\n";

    fn file(body: &str) -> Vec<u8> {
        format!("{HEADER}{body}").into_bytes()
    }

    #[test]
    fn rejects_missing_header() {
        assert!(matches!(
            import(b"Param.h.v.=1\nAddParam\n", "x", "x"),
            Err(CorpusError::Parse { .. })
        ));
    }

    #[test]
    fn imports_two_points_and_a_distance() {
        let body = "\
Request.h.v=00000004\nRequest.type=101\nRequest.group.v=00000002\nAddRequest\n\n\
Request.h.v=00000005\nRequest.type=101\nRequest.group.v=00000002\nAddRequest\n\n\
Entity.h.v=00040000\nEntity.type=2001\nEntity.workplane.v=80020000\nEntity.actPoint.x=1\nEntity.actPoint.y=2\nAddEntity\n\n\
Entity.h.v=00050000\nEntity.type=2001\nEntity.workplane.v=80020000\nEntity.actPoint.x=4\nEntity.actPoint.y=6\nAddEntity\n\n\
Entity.h.v=80020000\nEntity.type=10000\nEntity.point[0].v=80020002\nEntity.normal.v=80020001\nAddEntity\n\n\
Entity.h.v=80020001\nEntity.type=3010\nEntity.actNormal.w=1.0\nAddEntity\n\n\
Entity.h.v=80020002\nEntity.type=2012\nAddEntity\n\n\
Constraint.h.v=00000001\nConstraint.type=30\nConstraint.workplane.v=80020000\nConstraint.valA=5.0\nConstraint.ptA.v=00040000\nConstraint.ptB.v=00050000\nAddConstraint\n\n\
Constraint.h.v=00000002\nConstraint.type=20\nConstraint.workplane.v=80020000\nConstraint.ptA.v=00040000\nConstraint.ptB.v=80020002\nAddConstraint\n";
        let case = import(&file(body), "solvespace/t", "t.slvs").unwrap();
        assert_eq!(
            case.entities.len(),
            3,
            "two points and the synthesised origin"
        );
        assert_eq!(case.entities[0].geom, CaseGeom::Point { at: [1.0, 2.0] });
        assert!(matches!(
            case.constraints[0],
            ConstraintKind::Dimension(Dimension {
                value: 5.0,
                driving: true,
                ..
            })
        ));
        // Origin point + its Fix were synthesised before the coincident.
        assert!(matches!(case.constraints[1], ConstraintKind::Fix { .. }));
        assert!(matches!(
            case.constraints[2],
            ConstraintKind::Coincident { .. }
        ));
    }

    #[test]
    fn rejects_3d_points() {
        let body = "\
Request.h.v=00000004\nRequest.type=101\nRequest.group.v=00000002\nAddRequest\n\n\
Entity.h.v=00040000\nEntity.type=2000\nEntity.actPoint.x=1\nAddEntity\n";
        assert!(matches!(
            import(&file(body), "x", "x"),
            Err(CorpusError::Unsupported(_))
        ));
    }
}
