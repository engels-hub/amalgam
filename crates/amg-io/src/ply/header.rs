//! The PLY header: encoding, elements and properties.

use crate::error::{IoError, Location};
use crate::format::Format;

/// Longest header that is accepted (note §9.1).
pub const MAX_HEADER_BYTES: usize = 64 << 10;

/// A scalar property type.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Ty {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Ty {
    pub(crate) fn parse(s: &str) -> Option<Ty> {
        Some(match s {
            "char" | "int8" => Ty::I8,
            "uchar" | "uint8" => Ty::U8,
            "short" | "int16" => Ty::I16,
            "ushort" | "uint16" => Ty::U16,
            "int" | "int32" => Ty::I32,
            "uint" | "uint32" => Ty::U32,
            "float" | "float32" => Ty::F32,
            "double" | "float64" => Ty::F64,
            _ => return None,
        })
    }

    pub(crate) fn size(self) -> usize {
        match self {
            Ty::I8 | Ty::U8 => 1,
            Ty::I16 | Ty::U16 => 2,
            Ty::I32 | Ty::U32 | Ty::F32 => 4,
            Ty::F64 => 8,
        }
    }

    pub(crate) fn is_int(self) -> bool {
        !matches!(self, Ty::F32 | Ty::F64)
    }
}

/// One property of an element.
#[derive(Clone, Debug)]
pub(crate) enum Kind {
    Scalar(Ty),
    List { count: Ty, item: Ty },
}

#[derive(Clone, Debug)]
pub(crate) struct Prop {
    pub name: String,
    pub kind: Kind,
}

#[derive(Clone, Debug)]
pub(crate) struct Element {
    pub name: String,
    pub count: u64,
    pub props: Vec<Prop>,
    /// 1-based header line of the `element` statement.
    pub line: u64,
}

impl Element {
    /// Smallest possible size of one record in a binary file.
    pub(crate) fn min_record(&self) -> u64 {
        self.props
            .iter()
            .map(|p| match p.kind {
                Kind::Scalar(t) => t.size() as u64,
                Kind::List { count, .. } => count.size() as u64,
            })
            .sum()
    }

    /// Size of one record when no property is a list.
    pub(crate) fn fixed_record(&self) -> Option<usize> {
        let mut n = 0;
        for p in &self.props {
            match p.kind {
                Kind::Scalar(t) => n += t.size(),
                Kind::List { .. } => return None,
            }
        }
        Some(n)
    }

    /// Like [`Element::prop`], but only scalar properties.
    pub(crate) fn scalar(&self, names: &[&str]) -> Option<usize> {
        self.props
            .iter()
            .position(|p| names.contains(&p.name.as_str()) && matches!(p.kind, Kind::Scalar(_)))
    }

    /// Like [`Element::prop`], but only list properties.
    pub(crate) fn list(&self, names: &[&str]) -> Option<usize> {
        self.props
            .iter()
            .position(|p| names.contains(&p.name.as_str()) && matches!(p.kind, Kind::List { .. }))
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Encoding {
    Ascii,
    Le,
    Be,
}

#[derive(Clone, Debug)]
pub(crate) struct Header {
    pub encoding: Encoding,
    pub elements: Vec<Element>,
    /// File offset of the first body byte.
    pub body_offset: u64,
    /// Lines of the header, including `end_header`.
    pub lines: u64,
}

fn bad(line: u64, message: impl Into<String>) -> IoError {
    IoError::Malformed {
        format: Format::Ply,
        location: Location::Line(line),
        message: message.into(),
    }
}

/// Parses the header out of `head` (the first bytes of the file). `file_len` tells a header
/// that is cut off by `head` from a file that simply ends.
pub(crate) fn parse(head: &[u8], file_len: u64) -> Result<Header, IoError> {
    let mut pos = 0usize;
    let mut line_no = 0u64;
    let mut encoding = None;
    let mut elements: Vec<Element> = Vec::new();
    loop {
        let rest = &head[pos..];
        let Some(nl) = memchr::memchr(b'\n', rest) else {
            return Err(
                if file_len > head.len() as u64 || head.len() >= MAX_HEADER_BYTES {
                    IoError::LimitExceeded {
                        what: "PLY header bytes",
                        limit: MAX_HEADER_BYTES as u64,
                        actual: file_len.min(MAX_HEADER_BYTES as u64 + 1),
                    }
                } else {
                    IoError::Malformed {
                        format: Format::Ply,
                        location: Location::Byte(head.len() as u64),
                        message: "the header has no end_header line".into(),
                    }
                },
            );
        };
        let raw = &rest[..nl];
        pos += nl + 1;
        line_no += 1;
        let line = std::str::from_utf8(raw)
            .map_err(|_| bad(line_no, "the header is not text"))?
            .trim();
        if line_no == 1 {
            if line != "ply" {
                return Err(bad(1, "a PLY file starts with the line `ply`"));
            }
            continue;
        }
        let mut it = line.split_ascii_whitespace();
        let Some(word) = it.next() else { continue };
        match word {
            "end_header" => {
                let encoding = encoding.ok_or_else(|| bad(line_no, "no format line"))?;
                return Ok(Header {
                    encoding,
                    elements,
                    body_offset: pos as u64,
                    lines: line_no,
                });
            }
            "comment" | "obj_info" => {}
            "format" => {
                if encoding.is_some() {
                    return Err(bad(line_no, "a second format line"));
                }
                let kind = it.next().unwrap_or("");
                let version = it.next().unwrap_or("");
                if !version.starts_with('1') {
                    return Err(bad(line_no, format!("unsupported PLY version '{version}'")));
                }
                encoding = Some(match kind {
                    "ascii" => Encoding::Ascii,
                    "binary_little_endian" => Encoding::Le,
                    "binary_big_endian" => Encoding::Be,
                    other => return Err(bad(line_no, format!("unknown format '{other}'"))),
                });
            }
            "element" => {
                let name = it
                    .next()
                    .ok_or_else(|| bad(line_no, "element without a name"))?;
                let count = it
                    .next()
                    .and_then(|c| c.parse::<u64>().ok())
                    .ok_or_else(|| bad(line_no, "element without a valid count"))?;
                elements.push(Element {
                    name: name.to_owned(),
                    count,
                    props: Vec::new(),
                    line: line_no,
                });
            }
            "property" => {
                let el = elements
                    .last_mut()
                    .ok_or_else(|| bad(line_no, "property before any element"))?;
                let first = it
                    .next()
                    .ok_or_else(|| bad(line_no, "property without a type"))?;
                let ty = |s: &str| {
                    Ty::parse(s).ok_or_else(|| bad(line_no, format!("unknown property type '{s}'")))
                };
                let kind = if first == "list" {
                    let count = ty(it.next().unwrap_or(""))?;
                    let item = ty(it.next().unwrap_or(""))?;
                    if !count.is_int() {
                        return Err(bad(line_no, "a list count must be an integer type"));
                    }
                    Kind::List { count, item }
                } else {
                    Kind::Scalar(ty(first)?)
                };
                let name = it
                    .next()
                    .ok_or_else(|| bad(line_no, "property without a name"))?;
                el.props.push(Prop {
                    name: name.to_owned(),
                    kind,
                });
            }
            other => {
                return Err(bad(line_no, format!("unknown header statement '{other}'")));
            }
        }
    }
}
