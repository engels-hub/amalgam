//! The BREP-04 boolean corpus `testdata/brep/bool/cases.toml`: its format, the primitives it
//! builds and how a case runs. Shared by `boolean.rs` (BREP-04) and `history.rs` (BREP-05).
#![allow(dead_code, clippy::unwrap_used, reason = "shared test helper")]

use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, Dir3, Glue, JobCtx, KernelError, OcctKernel, OpResult, Plane,
    Point3, Shape, Tolerance, Transform, Vec3,
};
use amg_core::{Mat3, Quat};

const TOL: Tolerance = Tolerance::DEFAULT;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

/// An orthonormal right-handed frame at `origin` with `normal`, `x` along `xdir` (or a default
/// perpendicular).
pub fn frame(origin: [f64; 3], normal: Option<[f64; 3]>, xdir: Option<[f64; 3]>) -> Plane {
    let n = Vec3::from(normal.unwrap_or([0.0, 0.0, 1.0])).normalize();
    let x = match xdir {
        Some(x) => Vec3::from(x),
        None if n.x.abs() < 0.9 => Vec3::X,
        None => Vec3::Y,
    };
    let x = (x - n * x.dot(n)).normalize();
    Plane {
        origin: Point3::from(origin),
        x: Dir3::new_unchecked(x),
        y: Dir3::new_unchecked(n.cross(x)),
        normal: Dir3::new_unchecked(n),
    }
}

pub fn rotation(axis: Vec3, angle: f64) -> Transform {
    Transform::from_mat3_translation(
        Mat3::from_quat(Quat::from_axis_angle(axis.normalize(), angle)),
        Vec3::ZERO,
    )
}

// ---- the corpus ---------------------------------------------------------------------------

#[derive(Debug, serde::Deserialize)]
pub struct Corpus {
    pub case: Vec<Case>,
}

#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Case {
    pub name: String,
    pub op: String,
    pub source: String,
    pub volume: Option<f64>,
    pub error: Option<String>,
    pub solids: Option<usize>,
    pub fuzzy: Option<f64>,
    pub glue: Option<String>,
    pub rotate: Option<[f64; 4]>,
    #[serde(default)]
    pub determinism: bool,
    pub objects: Vec<Prim>,
    pub tools: Vec<Prim>,
}

#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Prim {
    pub kind: String,
    pub origin: [f64; 3],
    pub dims: Vec<f64>,
    pub normal: Option<[f64; 3]>,
    pub xdir: Option<[f64; 3]>,
}

pub fn corpus() -> Option<Vec<Case>> {
    let path = amg_testkit::testdata("brep/bool/cases.toml")?;
    let text = std::fs::read_to_string(path).unwrap();
    let corpus: Corpus = toml::from_str(&text).unwrap();
    Some(corpus.case)
}

pub fn build(k: &OcctKernel, p: &Prim) -> Shape {
    use std::f64::consts::TAU;
    let f = frame(p.origin, p.normal, p.xdir);
    let d = &p.dims;
    let made = match p.kind.as_str() {
        "box" => k.make_box(&f, Vec3::new(d[0], d[1], d[2])),
        "cylinder" => k.make_cylinder(&f, d[0], d[1], TAU),
        "cone" => k.make_cone(&f, d[0], d[1], d[2], TAU),
        "sphere" => k.make_sphere(&f, d[0]),
        "torus" => k.make_torus(&f, d[0], d[1], TAU),
        "rect" => {
            let o = f.origin;
            let (u, v) = (f.x.get() * d[0], f.y.get() * d[1]);
            let pts = [o, o + u, o + u + v, o + v];
            return k.make_planar_polygon(&pts, &[], &TOL).unwrap();
        }
        other => panic!("unknown primitive {other}"),
    };
    made.unwrap().shape
}

impl Case {
    pub fn operands(&self, k: &OcctKernel) -> (Vec<Shape>, Vec<Shape>) {
        let place = |p: &Prim| {
            let s = build(k, p);
            match self.rotate {
                Some([x, y, z, angle]) => {
                    s.transformed(&rotation(Vec3::new(x, y, z), angle)).unwrap()
                }
                None => s,
            }
        };
        (
            self.objects.iter().map(place).collect(),
            self.tools.iter().map(place).collect(),
        )
    }

    pub fn opts(&self, parallel: Option<bool>) -> BoolOpts {
        BoolOpts {
            fuzzy: self.fuzzy.unwrap_or(0.0),
            glue: match self.glue.as_deref() {
                None | Some("off") => Glue::Off,
                Some("shift") => Glue::Shift,
                Some("full") => Glue::Full,
                Some(other) => panic!("unknown glue {other}"),
            },
            parallel,
            ..BoolOpts::default()
        }
    }

    pub fn run(
        &self,
        k: &OcctKernel,
        objects: &[Shape],
        tools: &[Shape],
        parallel: Option<bool>,
    ) -> Result<OpResult, KernelError> {
        let opts = self.opts(parallel);
        match self.op.as_str() {
            "fuse" => k.boolean(BoolOp::Fuse, objects, tools, &opts, &ctx()),
            "cut" => k.boolean(BoolOp::Cut, objects, tools, &opts, &ctx()),
            "common" => k.boolean(BoolOp::Common, objects, tools, &opts, &ctx()),
            "split" => k.split(objects, tools, &opts, &ctx()),
            other => panic!("unknown op {other}"),
        }
    }
}
