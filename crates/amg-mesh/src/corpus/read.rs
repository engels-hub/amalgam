//! Minimal STL / PLY readers for corpus tests. Production import is `amg-io` (S12); these only
//! have to read what the corpus holds, exactly (f32 STL coordinates widen to f64, no repair).

use std::path::Path;

use amg_core::Point3;

use crate::{IndexedMesh, MeshError, WeldOpts};

/// Why a corpus file could not be read.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ReadError {
    /// File system error.
    #[error("{path}: {source}")]
    Io {
        /// File involved.
        path: std::path::PathBuf,
        /// Underlying error.
        source: std::io::Error,
    },
    /// Malformed or unsupported file.
    #[error("{0}")]
    Format(String),
    /// The triangles do not fit a mesh.
    #[error(transparent)]
    Mesh(#[from] MeshError),
}

fn fmt_err<T>(s: impl Into<String>) -> Result<T, ReadError> {
    Err(ReadError::Format(s.into()))
}

/// Reads an `.stl` or `.ply` file as an unwelded-by-index soup welded on bit-equal positions
/// (`WeldOpts::EXACT`); triangle order and count are kept (degenerate triangles included).
///
/// # Errors
/// On I/O errors, malformed files or an unknown extension.
pub fn read_mesh(path: &Path) -> Result<IndexedMesh, ReadError> {
    let bytes = std::fs::read(path).map_err(|source| ReadError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(str::to_ascii_lowercase);
    let soup = match ext.as_deref() {
        Some("stl") => read_stl(&bytes)?,
        Some("ply") => read_ply(&bytes)?,
        _ => return fmt_err(format!("unsupported extension: {}", path.display())),
    };
    Ok(IndexedMesh::from_soup(&soup, WeldOpts::EXACT)?)
}

/// Parses binary or ASCII STL into a triangle soup.
///
/// # Errors
/// If neither layout fits.
pub fn read_stl(bytes: &[u8]) -> Result<Vec<[Point3; 3]>, ReadError> {
    if bytes.len() >= 84 {
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        if bytes.len() == 84 + 50 * n {
            let f = |o: usize| {
                f64::from(f32::from_le_bytes([
                    bytes[o],
                    bytes[o + 1],
                    bytes[o + 2],
                    bytes[o + 3],
                ]))
            };
            return Ok((0..n)
                .map(|i| {
                    let b = 84 + 50 * i + 12;
                    let p =
                        |k: usize| Point3::new(f(b + 12 * k), f(b + 12 * k + 4), f(b + 12 * k + 8));
                    [p(0), p(1), p(2)]
                })
                .collect());
        }
    }
    let text = String::from_utf8_lossy(bytes);
    let mut pts = Vec::new();
    let mut it = text.split_ascii_whitespace();
    while let Some(tok) = it.next() {
        if tok.eq_ignore_ascii_case("vertex") {
            let mut c = [0.0f64; 3];
            for v in &mut c {
                let Some(s) = it.next() else {
                    return fmt_err("truncated ASCII STL");
                };
                *v = s
                    .parse()
                    .map_err(|_| ReadError::Format(format!("bad STL number {s:?}")))?;
            }
            pts.push(Point3::new(c[0], c[1], c[2]));
        }
    }
    if pts.is_empty() || pts.len() % 3 != 0 {
        return fmt_err("not a binary STL and no ASCII `vertex` triples");
    }
    Ok(pts.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect())
}

#[derive(Copy, Clone)]
enum Scalar {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Scalar {
    fn parse(s: &str) -> Result<Scalar, ReadError> {
        Ok(match s {
            "char" | "int8" => Scalar::I8,
            "uchar" | "uint8" => Scalar::U8,
            "short" | "int16" => Scalar::I16,
            "ushort" | "uint16" => Scalar::U16,
            "int" | "int32" => Scalar::I32,
            "uint" | "uint32" => Scalar::U32,
            "float" | "float32" => Scalar::F32,
            "double" | "float64" => Scalar::F64,
            _ => return fmt_err(format!("unknown PLY type {s}")),
        })
    }

    fn size(self) -> usize {
        match self {
            Scalar::I8 | Scalar::U8 => 1,
            Scalar::I16 | Scalar::U16 => 2,
            Scalar::I32 | Scalar::U32 | Scalar::F32 => 4,
            Scalar::F64 => 8,
        }
    }
}

enum Body<'a> {
    Ascii(std::str::SplitAsciiWhitespace<'a>),
    Binary {
        bytes: &'a [u8],
        pos: usize,
        big: bool,
    },
}

impl Body<'_> {
    fn next(&mut self, ty: Scalar) -> Result<f64, ReadError> {
        match self {
            Body::Ascii(it) => {
                let Some(s) = it.next() else {
                    return fmt_err("truncated PLY body");
                };
                s.parse()
                    .map_err(|_| ReadError::Format(format!("bad PLY number {s:?}")))
            }
            Body::Binary { bytes, pos, big } => {
                let n = ty.size();
                let Some(raw) = bytes.get(*pos..*pos + n) else {
                    return fmt_err("truncated PLY body");
                };
                *pos += n;
                let mut b = [0u8; 8];
                b[..n].copy_from_slice(raw);
                if *big {
                    b[..n].reverse();
                }
                Ok(match ty {
                    Scalar::I8 => f64::from(b[0] as i8),
                    Scalar::U8 => f64::from(b[0]),
                    Scalar::I16 => f64::from(i16::from_le_bytes([b[0], b[1]])),
                    Scalar::U16 => f64::from(u16::from_le_bytes([b[0], b[1]])),
                    Scalar::I32 => f64::from(i32::from_le_bytes([b[0], b[1], b[2], b[3]])),
                    Scalar::U32 => f64::from(u32::from_le_bytes([b[0], b[1], b[2], b[3]])),
                    Scalar::F32 => f64::from(f32::from_le_bytes([b[0], b[1], b[2], b[3]])),
                    Scalar::F64 => f64::from_le_bytes(b),
                })
            }
        }
    }
}

enum Prop {
    Scalar(Scalar, String),
    List(Scalar, Scalar, String),
}

/// Parses ASCII or binary (either endianness) PLY into a triangle soup. Only the `vertex` and
/// `face` elements are read; polygons are fan-triangulated; other elements must come after them.
///
/// # Errors
/// On malformed or unsupported files.
pub fn read_ply(bytes: &[u8]) -> Result<Vec<[Point3; 3]>, ReadError> {
    let marker = b"end_header";
    let Some(at) = bytes.windows(marker.len()).position(|w| w == marker) else {
        return fmt_err("PLY: no end_header");
    };
    let mut body_start = at + marker.len();
    // The header line ends with LF or CRLF.
    if bytes.get(body_start) == Some(&b'\r') {
        body_start += 1;
    }
    if bytes.get(body_start) == Some(&b'\n') {
        body_start += 1;
    }
    let header = String::from_utf8_lossy(&bytes[..at]);
    let mut lines = header.lines().map(str::trim);
    if lines.next() != Some("ply") {
        return fmt_err("PLY: bad magic");
    }
    let mut format = "";
    let mut elements: Vec<(String, usize, Vec<Prop>)> = Vec::new();
    for l in lines {
        let t: Vec<&str> = l.split_whitespace().collect();
        match t.as_slice() {
            ["format", f, _] => format = f,
            ["element", name, n] => {
                let n = n
                    .parse()
                    .map_err(|_| ReadError::Format(format!("PLY element count {n:?}")))?;
                elements.push(((*name).to_owned(), n, Vec::new()));
            }
            ["property", "list", c, ty, name] => {
                if let Some(e) = elements.last_mut() {
                    e.2.push(Prop::List(
                        Scalar::parse(c)?,
                        Scalar::parse(ty)?,
                        (*name).to_owned(),
                    ));
                }
            }
            ["property", ty, name] => {
                if let Some(e) = elements.last_mut() {
                    e.2.push(Prop::Scalar(Scalar::parse(ty)?, (*name).to_owned()));
                }
            }
            _ => {}
        }
    }
    let rest = &bytes[body_start..];
    let mut body = match format {
        "ascii" => Body::Ascii(
            std::str::from_utf8(rest)
                .map_err(|_| ReadError::Format("PLY ASCII body is not UTF-8".into()))?
                .split_ascii_whitespace(),
        ),
        "binary_little_endian" => Body::Binary {
            bytes: rest,
            pos: 0,
            big: false,
        },
        "binary_big_endian" => Body::Binary {
            bytes: rest,
            pos: 0,
            big: true,
        },
        other => return fmt_err(format!("PLY format {other:?}")),
    };
    let mut verts: Vec<Point3> = Vec::new();
    let mut soup: Vec<[Point3; 3]> = Vec::new();
    let mut seen_face = false;
    for (name, n, props) in &elements {
        match name.as_str() {
            "vertex" => {
                verts.reserve(*n);
                for _ in 0..*n {
                    let mut c = [0.0f64; 3];
                    for p in props {
                        match p {
                            Prop::Scalar(ty, pn) => {
                                let v = body.next(*ty)?;
                                match pn.as_str() {
                                    "x" => c[0] = v,
                                    "y" => c[1] = v,
                                    "z" => c[2] = v,
                                    _ => {}
                                }
                            }
                            Prop::List(ct, ty, _) => {
                                let k = body.next(*ct)? as usize;
                                for _ in 0..k {
                                    body.next(*ty)?;
                                }
                            }
                        }
                    }
                    verts.push(Point3::new(c[0], c[1], c[2]));
                }
            }
            "face" => {
                seen_face = true;
                soup.reserve(*n);
                for _ in 0..*n {
                    let mut idx: Vec<usize> = Vec::new();
                    for p in props {
                        match p {
                            Prop::Scalar(ty, _) => {
                                body.next(*ty)?;
                            }
                            Prop::List(ct, ty, pn) => {
                                let k = body.next(*ct)? as usize;
                                let vi = pn == "vertex_indices" || pn == "vertex_index";
                                for _ in 0..k {
                                    let v = body.next(*ty)? as usize;
                                    if vi {
                                        idx.push(v);
                                    }
                                }
                            }
                        }
                    }
                    for w in 1..idx.len().saturating_sub(1) {
                        let (Some(a), Some(b), Some(c)) =
                            (verts.get(idx[0]), verts.get(idx[w]), verts.get(idx[w + 1]))
                        else {
                            return fmt_err("PLY face index out of range");
                        };
                        soup.push([*a, *b, *c]);
                    }
                }
            }
            _ if !seen_face => {
                return fmt_err(format!("PLY element {name:?} before `face` is unsupported"));
            }
            _ => break,
        }
    }
    if !seen_face {
        return fmt_err("PLY has no face element");
    }
    Ok(soup)
}
