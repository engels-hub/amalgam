//! XML plumbing shared by every 3MF part: namespace scopes, value parsing (`ST_Number`,
//! indices, colours, transforms) and located errors.
//!
//! The readers run quick-xml without its own end-name checks and namespace resolution: they
//! keep their own element stack, which lets the model reader restart quick-xml after a range
//! that the fast scanner consumed.

use amg_core::Transform;
use amg_mesh::Rgba8;
use quick_xml::events::BytesStart;
use quick_xml::events::attributes::Attribute;

use crate::error::{IoError, Location};
use crate::format::Format;

/// 3MF core.
pub(crate) const NS_CORE: &str = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";
/// Materials and properties.
pub(crate) const NS_MAT: &str = "http://schemas.microsoft.com/3dmanufacturing/material/2015/02";
/// Production.
pub(crate) const NS_PROD: &str = "http://schemas.microsoft.com/3dmanufacturing/production/2015/06";
/// Triangle sets (core since 1.3).
pub(crate) const NS_TRISETS: &str =
    "http://schemas.microsoft.com/3dmanufacturing/trianglesets/2021/07";
/// Beam lattice (IO-18).
pub(crate) const NS_BEAM: &str = "http://schemas.microsoft.com/3dmanufacturing/beamlattice/2017/02";
/// PrusaSlicer.
pub(crate) const NS_SLIC3R: &str = "http://schemas.slic3r.org/3mf/2017/06";
/// The `xml:` prefix.
pub(crate) const NS_XML: &str = "http://www.w3.org/XML/1998/namespace";

/// A located, format-tagged error in `part` at byte `offset` of `data`.
pub(crate) fn malformed_at(
    part: &str,
    data: &[u8],
    offset: usize,
    msg: impl Into<String>,
) -> IoError {
    let (line, column) = line_col(data, offset);
    IoError::Malformed {
        format: Format::ThreeMf,
        location: Location::Xml {
            part: part.to_owned(),
            line,
            column,
        },
        message: msg.into(),
    }
}

/// An error about a whole part.
pub(crate) fn malformed_part(part: &str, msg: impl Into<String>) -> IoError {
    IoError::Malformed {
        format: Format::ThreeMf,
        location: Location::ZipEntry(part.to_owned()),
        message: msg.into(),
    }
}

/// 1-based line and column of byte `offset`.
pub(crate) fn line_col(data: &[u8], offset: usize) -> (u64, u64) {
    let offset = offset.min(data.len());
    let head = &data[..offset];
    let line = memchr::memchr_iter(b'\n', head).count() as u64 + 1;
    let col = match memchr::memrchr(b'\n', head) {
        Some(nl) => offset - nl,
        None => offset + 1,
    };
    (line, col as u64)
}

/// Namespace scopes: one frame of `(prefix, uri)` declarations per open element.
#[derive(Default)]
pub(crate) struct Scopes {
    decls: Vec<(String, String)>,
    marks: Vec<usize>,
}

impl Scopes {
    pub(crate) fn new() -> Self {
        let mut s = Scopes::default();
        s.decls.push(("xml".into(), NS_XML.into()));
        s
    }

    /// Opens a scope with the `xmlns` declarations of `e`.
    pub(crate) fn push(&mut self, e: &BytesStart<'_>) {
        self.marks.push(self.decls.len());
        for a in e.attributes().with_checks(false).flatten() {
            let k = a.key.0;
            if k == "xmlns" {
                self.decls.push((String::new(), a.value.to_string()));
            } else if let Some(p) = k.strip_prefix("xmlns:") {
                self.decls.push((p.to_owned(), a.value.to_string()));
            }
        }
    }

    /// Adds one declaration to the innermost scope (a copy of an outer scope).
    pub(crate) fn push_decl(&mut self, prefix: &str, uri: &str) {
        self.decls.push((prefix.to_owned(), uri.to_owned()));
    }

    pub(crate) fn pop(&mut self) {
        if let Some(m) = self.marks.pop() {
            self.decls.truncate(m);
        }
    }

    /// URI bound to `prefix` (`""` = default namespace).
    pub(crate) fn uri(&self, prefix: &str) -> Option<&str> {
        self.decls
            .iter()
            .rev()
            .find(|(p, _)| p == prefix)
            .map(|(_, u)| u.as_str())
            .filter(|u| !u.is_empty() || prefix.is_empty())
    }

    /// Namespace and local name of an element name.
    pub(crate) fn element<'n>(&self, qname: &'n str) -> (Option<&str>, &'n str) {
        match qname.split_once(':') {
            Some((p, l)) => (self.uri(p), l),
            None => (self.uri("").filter(|u| !u.is_empty()), qname),
        }
    }

    /// Prefixes currently bound to `uri`.
    pub(crate) fn prefixes_of(&self, uri: &str) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for (p, u) in self.decls.iter().rev() {
            if out.contains(p) {
                continue;
            }
            // Shadowed by a later declaration of the same prefix?
            let current = self.uri(p);
            if u == uri && current == Some(uri) && !p.is_empty() {
                out.push(p.clone());
            }
        }
        out
    }

    /// Every declaration in scope (prefix, uri), innermost last.
    pub(crate) fn all(&self) -> &[(String, String)] {
        &self.decls
    }
}

/// True for `xmlns` / `xmlns:*` attributes.
pub(crate) fn is_xmlns(key: &str) -> bool {
    key == "xmlns" || key.starts_with("xmlns:")
}

/// Unescaped attribute value, bounded by `max_len`.
pub(crate) fn attr_value<'a>(
    a: &Attribute<'a>,
    max_len: u32,
) -> Result<std::borrow::Cow<'a, str>, String> {
    if a.value.len() > max_len as usize {
        return Err(format!(
            "attribute {} is longer than the limit of {max_len} bytes",
            a.key.0
        ));
    }
    a.normalized_value(quick_xml::XmlVersion::Implicit1_0)
        .map_err(|e| format!("attribute {}: {e}", a.key.0))
}

/// Trims XML whitespace.
pub(crate) fn trim(s: &str) -> &str {
    s.trim_matches(|c| matches!(c, ' ' | '\t' | '\n' | '\r'))
}

/// `ST_Number`: `[+-]?(digits(.digits)?|.digits)([eE][+-]?digits)?`.
pub(crate) fn is_st_number(s: &[u8]) -> bool {
    let mut i = 0;
    let n = s.len();
    if i < n && (s[i] == b'+' || s[i] == b'-') {
        i += 1;
    }
    let int_start = i;
    while i < n && s[i].is_ascii_digit() {
        i += 1;
    }
    let int_digits = i - int_start;
    if i < n && s[i] == b'.' {
        i += 1;
        let f = i;
        while i < n && s[i].is_ascii_digit() {
            i += 1;
        }
        if i == f {
            return false;
        }
    } else if int_digits == 0 {
        return false;
    }
    if i < n && (s[i] == b'e' || s[i] == b'E') {
        i += 1;
        if i < n && (s[i] == b'+' || s[i] == b'-') {
            i += 1;
        }
        let e = i;
        while i < n && s[i].is_ascii_digit() {
            i += 1;
        }
        if i == e {
            return false;
        }
    }
    i == n
}

/// A finite number. Strict mode requires `ST_Number`; otherwise anything Rust parses that is
/// finite (for example `1.`) is accepted.
pub(crate) fn parse_number(raw: &str, strict: bool) -> Result<f64, String> {
    let s = trim(raw);
    if strict && !is_st_number(s.as_bytes()) {
        return Err(format!("{raw:?} is not a valid number"));
    }
    match s.parse::<f64>() {
        Ok(v) if v.is_finite() => Ok(v),
        _ => Err(format!("{raw:?} is not a finite number")),
    }
}

/// A non-negative integer below 2³¹ (`ST_ResourceIndex`, `ST_ResourceID`).
pub(crate) fn parse_index(raw: &str) -> Result<u32, String> {
    let s = trim(raw);
    let d = s.strip_prefix('+').unwrap_or(s);
    if d.is_empty() || !d.bytes().all(|b| b.is_ascii_digit()) {
        return Err(format!("{raw:?} is not a non-negative integer"));
    }
    let digits = d.trim_start_matches('0');
    if digits.len() > 10 {
        return Err(format!("{raw:?} is out of range"));
    }
    match digits.parse::<u64>() {
        Ok(v) if v < (1 << 31) => Ok(v as u32),
        Err(_) if digits.is_empty() => Ok(0),
        _ => Err(format!("{raw:?} is out of range")),
    }
}

/// A resource id: a positive integer below 2³¹.
pub(crate) fn parse_id(raw: &str) -> Result<u32, String> {
    match parse_index(raw)? {
        0 => Err(format!("resource id {raw:?} must be positive")),
        v => Ok(v),
    }
}

/// `#RRGGBB` or `#RRGGBBAA` (sRGB).
pub(crate) fn parse_color(raw: &str) -> Result<Rgba8, String> {
    let s = trim(raw);
    let hex = s
        .strip_prefix('#')
        .filter(|h| (h.len() == 6 || h.len() == 8) && h.bytes().all(|b| b.is_ascii_hexdigit()))
        .ok_or_else(|| format!("{raw:?} is not a #RRGGBB[AA] colour"))?;
    let b = |i: usize| u8::from_str_radix(&hex[i..i + 2], 16).unwrap_or(0);
    Ok(Rgba8([
        b(0),
        b(2),
        b(4),
        if hex.len() == 8 { b(6) } else { 255 },
    ]))
}

/// A 3MF `ST_Matrix3D`: `m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32`, row-vector
/// convention (`p' = p · M`). Each row of the 3×3 part becomes a column of the
/// `DAffine3` (column vectors), and `m3x` is the translation.
pub(crate) fn parse_transform(raw: &str, strict: bool) -> Result<Transform, String> {
    let mut m = [0f64; 12];
    let mut n = 0;
    for tok in raw.split([' ', '\t', '\n', '\r']) {
        if tok.is_empty() {
            continue;
        }
        if n == 12 {
            return Err(format!("transform {raw:?} has more than 12 numbers"));
        }
        m[n] = parse_number(tok, strict)?;
        n += 1;
    }
    if n != 12 {
        return Err(format!("transform {raw:?} has {n} numbers, not 12"));
    }
    Ok(Transform {
        matrix3: amg_core::Mat3::from_cols(
            amg_core::Vec3::new(m[0], m[1], m[2]),
            amg_core::Vec3::new(m[3], m[4], m[5]),
            amg_core::Vec3::new(m[6], m[7], m[8]),
        ),
        translation: amg_core::Vec3::new(m[9], m[10], m[11]),
    })
}

/// Decodes `%XX` escapes of a part name or relationship target.
pub(crate) fn percent_decode(s: &str) -> String {
    if !s.contains('%') {
        return s.to_owned();
    }
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'%'
            && i + 2 < b.len()
            && let Ok(v) = u8::from_str_radix(&s[i + 1..i + 3], 16)
        {
            out.push(v);
            i += 3;
            continue;
        }
        out.push(b[i]);
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// Absolute, percent-decoded part name with dot segments resolved, case kept.
pub(crate) fn normalize_exact(name: &str) -> String {
    let decoded = percent_decode(name);
    let mut segs: Vec<&str> = Vec::new();
    for seg in decoded.split(['/', '\\']) {
        match seg {
            "" | "." => {}
            ".." => {
                segs.pop();
            }
            s => segs.push(s),
        }
    }
    let mut out = String::with_capacity(decoded.len() + 1);
    for s in segs {
        out.push('/');
        out.push_str(s);
    }
    out
}

/// [`resolve_target`] with the case kept.
pub(crate) fn resolve_exact(source: &str, target: &str) -> String {
    let target = target.split('#').next().unwrap_or("");
    if target.starts_with('/') {
        return normalize_exact(target);
    }
    let base = match source.rfind('/') {
        Some(i) => &source[..=i],
        None => "/",
    };
    normalize_exact(&format!("{base}{target}"))
}

/// Canonical form of a part name for lookups: absolute, percent-decoded, dot segments
/// resolved, ASCII lower case (OPC part names compare case-insensitively).
pub(crate) fn part_key(name: &str) -> String {
    normalize_exact(name).to_ascii_lowercase()
}

/// Resolves a relationship `target` against the part `source` (a part name, `/` for the
/// package root). Fragments are dropped.
pub(crate) fn resolve_target(source: &str, target: &str) -> String {
    let target = target.split('#').next().unwrap_or("");
    if target.starts_with('/') {
        return part_key(target);
    }
    let base = match source.rfind('/') {
        Some(i) => &source[..=i],
        None => "/",
    };
    part_key(&format!("{base}{target}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn st_number() {
        for ok in ["0", "-1", "+2.5", ".5", "1e3", "1.25E-7", "-0.0"] {
            assert!(is_st_number(ok.as_bytes()), "{ok}");
        }
        for bad in [
            "", "1.", "inf", "NaN", "0x1", "1e", "--1", ". 5", "1,5", "e3",
        ] {
            assert!(!is_st_number(bad.as_bytes()), "{bad}");
        }
        assert_eq!(parse_number(" 1.5 ", true), Ok(1.5));
        assert!(parse_number("1.", true).is_err());
        assert_eq!(parse_number("1.", false), Ok(1.0));
        assert!(parse_number("inf", false).is_err());
        assert!(parse_number("1e999", false).is_err());
    }

    #[test]
    fn indices_and_ids() {
        assert_eq!(parse_index("0"), Ok(0));
        assert_eq!(parse_index("+007"), Ok(7));
        assert_eq!(parse_index("2147483647"), Ok(2_147_483_647));
        assert!(parse_index("2147483648").is_err());
        assert!(parse_index("-1").is_err());
        assert!(parse_index("1.0").is_err());
        assert!(parse_index("").is_err());
        assert!(parse_id("0").is_err());
        assert_eq!(parse_id("3"), Ok(3));
    }

    #[test]
    fn colors() {
        assert_eq!(parse_color("#FF8000"), Ok(Rgba8([255, 128, 0, 255])));
        assert_eq!(parse_color("#ff800080"), Ok(Rgba8([255, 128, 0, 128])));
        assert!(parse_color("FF8000").is_err());
        assert!(parse_color("#FF80").is_err());
        assert!(parse_color("#GG8000").is_err());
    }

    /// Pins the row-vector convention (note §2.3): translation in m3x, rows → columns.
    #[test]
    fn transform_convention() {
        let t = parse_transform("0 1 0 -1 0 0 0 0 1 10 20 30", true).expect("parse");
        // p' = p · M: (1,0,0) → row 0 = (0,1,0), plus translation.
        let p = t.transform_point3(amg_core::Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(p, amg_core::Vec3::new(10.0, 21.0, 30.0));
        let p = t.transform_point3(amg_core::Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(p, amg_core::Vec3::new(9.0, 20.0, 30.0));
        assert!(parse_transform("1 0 0 0 1 0 0 0 1 0 0", true).is_err());
        assert!(parse_transform("1 0 0 0 1 0 0 0 1 0 0 0 0", true).is_err());
    }

    #[test]
    fn part_names() {
        assert_eq!(part_key("3D/3dmodel.model"), "/3d/3dmodel.model");
        assert_eq!(part_key("/3D/Objects/a%20b.model"), "/3d/objects/a b.model");
        assert_eq!(
            resolve_target("/3D/3dmodel.model", "../Metadata/t.png"),
            "/metadata/t.png"
        );
        assert_eq!(resolve_target("/", "3D/3dmodel.model"), "/3d/3dmodel.model");
        assert_eq!(
            resolve_target("/3D/3dmodel.model", "/3D/Textures/x.png#frag"),
            "/3d/textures/x.png"
        );
        assert_eq!(percent_decode("a%2"), "a%2");
    }

    #[test]
    fn line_and_column() {
        let d = b"ab\ncd\nef";
        assert_eq!(line_col(d, 0), (1, 1));
        assert_eq!(line_col(d, 4), (2, 2));
        assert_eq!(line_col(d, 6), (3, 1));
    }
}
