//! Shared by the PLY tests: byte builders and the `ply-rs-bw` oracle.
#![allow(
    dead_code,
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test support"
)]

use std::io::Cursor;

use amg_io::{Format, ImportOptions, Imported, IoError, Source, import};
use amg_jobs::JobCtx;
use ply_rs_bw::parser::Parser;
use ply_rs_bw::ply::{DefaultElement, Property};

pub fn ctx() -> JobCtx {
    JobCtx::detached()
}

pub fn run(bytes: &[u8]) -> Result<Imported, IoError> {
    run_with(bytes, &ImportOptions::default())
}

pub fn run_with(bytes: &[u8], opts: &ImportOptions) -> Result<Imported, IoError> {
    import(Source::Bytes(bytes), Some(Format::Ply), opts, &ctx())
}

/// What `ply-rs-bw` makes of a file.
pub struct Oracle {
    pub positions: Vec<[f64; 3]>,
    pub faces: Vec<Vec<u32>>,
    pub colors: Option<Vec<[u8; 4]>>,
}

fn scalar(p: &Property) -> f64 {
    match p {
        Property::Char(v) => f64::from(*v),
        Property::UChar(v) => f64::from(*v),
        Property::Short(v) => f64::from(*v),
        Property::UShort(v) => f64::from(*v),
        Property::Int(v) => f64::from(*v),
        Property::UInt(v) => f64::from(*v),
        Property::Float(v) => f64::from(*v),
        Property::Double(v) => *v,
        other => panic!("not a scalar: {other:?}"),
    }
}

fn list(p: &Property) -> Vec<u32> {
    match p {
        Property::ListChar(v) => v.iter().map(|&x| x as u32).collect(),
        Property::ListUChar(v) => v.iter().map(|&x| u32::from(x)).collect(),
        Property::ListShort(v) => v.iter().map(|&x| x as u32).collect(),
        Property::ListUShort(v) => v.iter().map(|&x| u32::from(x)).collect(),
        Property::ListInt(v) => v.iter().map(|&x| x as u32).collect(),
        Property::ListUInt(v) => v.clone(),
        other => panic!("not an index list: {other:?}"),
    }
}

pub fn oracle(bytes: &[u8]) -> Oracle {
    let p = Parser::<DefaultElement>::new();
    let ply = p
        .read_ply(&mut Cursor::new(bytes))
        .expect("ply-rs-bw reads the file");
    let vs = &ply.payload["vertex"];
    let positions = vs
        .iter()
        .map(|v| [scalar(&v["x"]), scalar(&v["y"]), scalar(&v["z"])])
        .collect();
    let colors = vs.first().and_then(|v| v.get("red")).map(|_| {
        vs.iter()
            .map(|v| {
                let c = |n: &str| scalar(&v[n]) as u8;
                let a = v.get("alpha").map_or(255, |a| scalar(a) as u8);
                [c("red"), c("green"), c("blue"), a]
            })
            .collect()
    });
    let faces = ply.payload.get("face").map_or_else(Vec::new, |fs| {
        fs.iter()
            .map(|f| {
                list(
                    f.get("vertex_indices")
                        .or_else(|| f.get("vertex_index"))
                        .expect("index list"),
                )
            })
            .collect()
    });
    Oracle {
        positions,
        faces,
        colors,
    }
}

// ------------------------------------------------------------------------------- builders

pub fn f32b(be: bool, x: f32) -> [u8; 4] {
    if be { x.to_be_bytes() } else { x.to_le_bytes() }
}
pub fn f64b(be: bool, x: f64) -> [u8; 8] {
    if be { x.to_be_bytes() } else { x.to_le_bytes() }
}
pub fn u16b(be: bool, x: u16) -> [u8; 2] {
    if be { x.to_be_bytes() } else { x.to_le_bytes() }
}
pub fn u32b(be: bool, x: u32) -> [u8; 4] {
    if be { x.to_be_bytes() } else { x.to_le_bytes() }
}

pub const TETRA_V: [[f32; 3]; 4] = [
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
];
pub const TETRA_F: [[u32; 3]; 4] = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];

/// A tetrahedron with the given list count and index types. `format` is the header's
/// `binary_little_endian` / `binary_big_endian`.
pub fn binary_tetra(be: bool, count_ty: &str, index_ty: &str) -> Vec<u8> {
    let mut out = format!(
        "ply\nformat {} 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\n\
         element face 4\nproperty list {count_ty} {index_ty} vertex_indices\nend_header\n",
        if be {
            "binary_big_endian"
        } else {
            "binary_little_endian"
        }
    )
    .into_bytes();
    for v in TETRA_V {
        for x in v {
            out.extend_from_slice(&f32b(be, x));
        }
    }
    for f in TETRA_F {
        match count_ty {
            "uchar" => out.push(3),
            "ushort" => out.extend_from_slice(&u16b(be, 3)),
            "uint" => out.extend_from_slice(&u32b(be, 3)),
            other => panic!("count type {other}"),
        }
        for i in f {
            match index_ty {
                "int" | "uint" | "int32" | "uint32" => out.extend_from_slice(&u32b(be, i)),
                "ushort" | "short" => out.extend_from_slice(&u16b(be, i as u16)),
                "uchar" | "char" => out.push(i as u8),
                other => panic!("index type {other}"),
            }
        }
    }
    out
}

pub fn ascii_tetra() -> Vec<u8> {
    let mut s = String::from(
        "ply\nformat ascii 1.0\ncomment a tetrahedron\nelement vertex 4\nproperty float x\n\
         property float y\nproperty float z\nelement face 4\nproperty list uchar int vertex_indices\n\
         end_header\n",
    );
    for v in TETRA_V {
        s.push_str(&format!("{} {} {}\n", v[0], v[1], v[2]));
    }
    for f in TETRA_F {
        s.push_str(&format!("3 {} {} {}\n", f[0], f[1], f[2]));
    }
    s.into_bytes()
}
