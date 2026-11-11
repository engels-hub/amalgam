//! Format registry and content sniffing (S12 §3).

use crate::error::IoWarning;

/// A file format that `amg-io` knows about.
#[derive(
    Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
#[non_exhaustive]
pub enum Format {
    /// Binary or ASCII STL.
    Stl,
    /// 3D Manufacturing Format.
    ThreeMf,
    /// Wavefront OBJ.
    Obj,
    /// Polygon File Format.
    Ply,
    /// ISO 10303-21 (STEP).
    Step,
    /// IGES.
    Iges,
    /// AutoCAD DXF (2D).
    Dxf,
    /// SVG (2D).
    Svg,
}

/// What a format holds.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GeometryKind {
    /// Triangle meshes.
    Mesh,
    /// Boundary representations.
    Brep,
    /// 2D curves.
    Curves2d,
}

/// Whether a format states its length unit.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UnitDeclaration {
    /// Always declared (3MF, STEP, IGES).
    Always,
    /// Sometimes (DXF `$INSUNITS`, SVG `width="10mm"`).
    Sometimes,
    /// Never (STL, OBJ, PLY).
    Never,
}

/// Static description of a format.
#[derive(Copy, Clone, Debug)]
pub struct FormatInfo {
    /// The format.
    pub format: Format,
    /// Display name.
    pub name: &'static str,
    /// Lower-case extensions without the dot; the first is the preferred one.
    pub extensions: &'static [&'static str],
    /// Kind of geometry.
    pub kind: GeometryKind,
    /// Unit declaration.
    pub units: UnitDeclaration,
    /// Import is implemented in this build (feature and work package).
    pub can_import: bool,
    /// Export is implemented in this build.
    pub can_export: bool,
    /// One file can hold several objects.
    pub multi_object: bool,
    /// The format can carry colours.
    pub colors: bool,
}

const fn info(
    format: Format,
    name: &'static str,
    extensions: &'static [&'static str],
    kind: GeometryKind,
    units: UnitDeclaration,
    multi_object: bool,
    colors: bool,
) -> FormatInfo {
    FormatInfo {
        format,
        name,
        extensions,
        kind,
        units,
        // The readers and writers land with IO-02..IO-13 (PLY is IO-13); each of those flips its flags.
        can_import: matches!(
            format,
            Format::Stl | Format::Obj | Format::ThreeMf | Format::Ply
        ),
        can_export: matches!(format, Format::Stl | Format::Obj | Format::Ply),
        multi_object,
        colors,
    }
}

/// Mesh and 2D formats first, then the ones that need feature `brep`, so that [`registry`] can
/// cut the slice.
static ALL: [FormatInfo; 8] = [
    info(
        Format::Stl,
        "STL (Stereolithography)",
        &["stl"],
        GeometryKind::Mesh,
        UnitDeclaration::Never,
        true,
        true,
    ),
    info(
        Format::ThreeMf,
        "3MF (3D Manufacturing Format)",
        &["3mf"],
        GeometryKind::Mesh,
        UnitDeclaration::Always,
        true,
        true,
    ),
    info(
        Format::Obj,
        "OBJ (Wavefront)",
        &["obj"],
        GeometryKind::Mesh,
        UnitDeclaration::Never,
        true,
        true,
    ),
    info(
        Format::Ply,
        "PLY (Polygon File Format)",
        &["ply"],
        GeometryKind::Mesh,
        UnitDeclaration::Never,
        false,
        true,
    ),
    info(
        Format::Dxf,
        "DXF (AutoCAD Drawing Exchange)",
        &["dxf"],
        GeometryKind::Curves2d,
        UnitDeclaration::Sometimes,
        true,
        true,
    ),
    info(
        Format::Svg,
        "SVG (Scalable Vector Graphics)",
        &["svg"],
        GeometryKind::Curves2d,
        UnitDeclaration::Sometimes,
        true,
        true,
    ),
    info(
        Format::Step,
        "STEP (ISO 10303)",
        &["step", "stp"],
        GeometryKind::Brep,
        UnitDeclaration::Always,
        true,
        true,
    ),
    info(
        Format::Iges,
        "IGES",
        &["iges", "igs"],
        GeometryKind::Brep,
        UnitDeclaration::Always,
        true,
        true,
    ),
];

/// Formats available in this build, for dialogs. STEP and IGES need cargo feature `brep`.
pub fn registry() -> &'static [FormatInfo] {
    if cfg!(feature = "brep") {
        &ALL
    } else {
        &ALL[..6]
    }
}

impl Format {
    /// Static description. Works for every format, also those that [`registry`] omits.
    pub fn info(self) -> &'static FormatInfo {
        ALL.iter().find(|i| i.format == self).unwrap_or(&ALL[0]) // unreachable: every variant is in `ALL` (tested)
    }

    /// Format for a file extension, case-insensitive; `"stl"` and `".STL"` both work.
    pub fn from_extension(ext: &str) -> Option<Format> {
        let e = ext.strip_prefix('.').unwrap_or(ext).to_ascii_lowercase();
        ALL.iter()
            .find(|i| i.extensions.contains(&e.as_str()))
            .map(|i| i.format)
    }

    /// Content sniffing (S12 §3, note §5.1). `head` is the first ≤ 4 KiB of the file and
    /// `file_len` its full length. The extension is never consulted.
    ///
    /// Zip containers can only be recognised heuristically from the head (3MF, when the first
    /// entry is under `3D/`, or `[Content_Types].xml` and a `3D/` name follows in the head).
    /// [`probe`](crate::probe) reads the central directory and is authoritative.
    pub fn sniff(head: &[u8], file_len: u64) -> Option<Format> {
        let head = &head[..head.len().min(SNIFF_HEAD)];
        if head.starts_with(b"PK\x03\x04") {
            return sniff_zip_head(head).then_some(Format::ThreeMf);
        }
        // An exact size match is the strongest evidence there is.
        if matches!(
            crate::stl::detect_rule(head, file_len),
            Ok((_, crate::stl::Rule::ExactSize))
        ) {
            return Some(Format::Stl);
        }
        if has_foreign_signature(head) {
            return None;
        }
        let text = strip_bom(head);
        if is_ply(text) {
            return Some(Format::Ply);
        }
        if is_step(text) {
            return Some(Format::Step);
        }
        if is_iges(head) {
            return Some(Format::Iges);
        }
        if is_dxf(text) {
            return Some(Format::Dxf);
        }
        if is_svg(text) {
            return Some(Format::Svg);
        }
        match crate::stl::detect_rule(head, file_len) {
            Ok((_, crate::stl::Rule::AsciiKeyword)) => return Some(Format::Stl),
            Ok((_, crate::stl::Rule::CountMismatch | crate::stl::Rule::SizeMismatch))
                if crate::stl::looks_like_binary(head) =>
            {
                return Some(Format::Stl);
            }
            _ => {}
        }
        if is_obj(text) {
            return Some(Format::Obj);
        }
        None
    }
}

/// Bytes of the file the sniffer looks at.
pub(crate) const SNIFF_HEAD: usize = 4096;

/// Warning for an extension that disagrees with the sniffed content (`None` if it agrees, or
/// the file has no known extension).
pub fn check_extension(extension: &str, detected: Format) -> Option<IoWarning> {
    let e = extension.strip_prefix('.').unwrap_or(extension);
    let from_ext = Format::from_extension(e)?;
    (from_ext != detected).then(|| IoWarning::ExtensionMismatch {
        extension: e.to_ascii_lowercase(),
        detected,
    })
}

pub(crate) fn strip_bom(b: &[u8]) -> &[u8] {
    b.strip_prefix(b"\xEF\xBB\xBF").unwrap_or(b)
}

pub(crate) fn skip_ws(b: &[u8]) -> &[u8] {
    let n = b.iter().take_while(|c| c.is_ascii_whitespace()).count();
    &b[n..]
}

/// Magic numbers of formats that are certainly not ours; they keep the loose binary-STL rules
/// from claiming them.
fn has_foreign_signature(h: &[u8]) -> bool {
    const SIGS: [&[u8]; 11] = [
        b"\x89PNG",
        b"\xFF\xD8\xFF",
        b"%PDF-",
        b"GIF8",
        b"\x1F\x8B",
        b"7z\xBC\xAF",
        b"Rar!",
        b"MZ",
        b"\x7FELF",
        b"RIFF",
        b"PK",
    ];
    SIGS.iter().any(|s| h.starts_with(s))
}

fn starts_with_ci(b: &[u8], pat: &[u8]) -> bool {
    b.len() >= pat.len() && b[..pat.len()].eq_ignore_ascii_case(pat)
}

pub(crate) fn contains_ci(hay: &[u8], pat: &[u8]) -> bool {
    !pat.is_empty() && hay.windows(pat.len()).any(|w| w.eq_ignore_ascii_case(pat))
}

fn is_ply(t: &[u8]) -> bool {
    t.starts_with(b"ply\n") || t.starts_with(b"ply\r\n")
}

fn is_step(t: &[u8]) -> bool {
    skip_ws(t).starts_with(b"ISO-10303-21;")
}

fn is_iges(h: &[u8]) -> bool {
    h.len() >= 80
        && h[72] == b'S'
        && h[73..80].iter().all(|c| c.is_ascii_digit() || *c == b' ')
        && h[..72].iter().all(|c| (0x20..0x7F).contains(c))
        && (h.len() == 80 || h[80] == b'\n' || h[80] == b'\r')
}

fn is_dxf(t: &[u8]) -> bool {
    let t = skip_ws(t);
    let Some(rest) = t.strip_prefix(b"0") else {
        return false;
    };
    let rest = rest.strip_prefix(b" ").map_or(rest, skip_spaces);
    let rest = match rest {
        [b'\r', b'\n', r @ ..] | [b'\n', r @ ..] => r,
        _ => return false,
    };
    starts_with_ci(skip_spaces(rest), b"SECTION")
}

fn skip_spaces(b: &[u8]) -> &[u8] {
    let n = b.iter().take_while(|c| **c == b' ').count();
    &b[n..]
}

fn is_svg(t: &[u8]) -> bool {
    let t = skip_ws(t);
    if t.starts_with(b"<svg") {
        return true;
    }
    (t.starts_with(b"<?xml") || t.starts_with(b"<!--") || t.starts_with(b"<!DOCTYPE"))
        && t.windows(4).any(|w| w == b"<svg")
}

fn is_obj(t: &[u8]) -> bool {
    if t.contains(&0) {
        return false;
    }
    for line in t.split(|c| *c == b'\n') {
        let line = skip_ws(line);
        if line.is_empty() || line[0] == b'#' {
            continue;
        }
        let mut it = line
            .split(|c| c.is_ascii_whitespace())
            .filter(|s| !s.is_empty());
        let tok = it.next().unwrap_or_default();
        let next = it.next();
        let number = |s: Option<&[u8]>| {
            s.and_then(|s| std::str::from_utf8(s).ok())
                .is_some_and(|s| s.parse::<f64>().is_ok())
        };
        return match tok {
            b"v" | b"vn" | b"vt" => number(next),
            b"f" => next.is_some_and(|s| s[0].is_ascii_digit() || s[0] == b'-'),
            b"o" | b"g" | b"mtllib" => true,
            _ => false,
        };
    }
    false
}

/// First local file header name of a zip head, lossy UTF-8.
pub(crate) fn first_zip_entry(head: &[u8]) -> Option<String> {
    if head.len() < 30 {
        return None;
    }
    let n = usize::from(u16::from_le_bytes([head[26], head[27]]));
    let name = head.get(30..30 + n)?;
    Some(String::from_utf8_lossy(name).into_owned())
}

fn sniff_zip_head(head: &[u8]) -> bool {
    let Some(first) = first_zip_entry(head) else {
        return false;
    };
    let lower = first.to_ascii_lowercase();
    lower.starts_with("3d/") || (lower == "[content_types].xml" && contains_ci(head, b"3d/"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_format_is_in_the_table_once() {
        for f in [
            Format::Stl,
            Format::ThreeMf,
            Format::Obj,
            Format::Ply,
            Format::Step,
            Format::Iges,
            Format::Dxf,
            Format::Svg,
        ] {
            assert_eq!(ALL.iter().filter(|i| i.format == f).count(), 1, "{f:?}");
            assert_eq!(f.info().format, f);
            assert!(!f.info().extensions.is_empty());
            for e in f.info().extensions {
                assert_eq!(e, &e.to_ascii_lowercase());
                assert_eq!(Format::from_extension(e), Some(f));
            }
        }
    }

    #[test]
    fn registry_follows_the_brep_feature() {
        let has = |f: Format| registry().iter().any(|i| i.format == f);
        assert!(has(Format::Stl) && has(Format::ThreeMf) && has(Format::Svg));
        assert_eq!(has(Format::Step), cfg!(feature = "brep"));
        assert_eq!(has(Format::Iges), cfg!(feature = "brep"));
    }

    #[test]
    fn from_extension_is_case_insensitive_and_accepts_the_dot() {
        assert_eq!(Format::from_extension("STL"), Some(Format::Stl));
        assert_eq!(Format::from_extension(".3mf"), Some(Format::ThreeMf));
        assert_eq!(Format::from_extension("Stp"), Some(Format::Step));
        assert_eq!(Format::from_extension("igs"), Some(Format::Iges));
        assert_eq!(Format::from_extension("amg"), None);
        assert_eq!(Format::from_extension(""), None);
    }

    #[test]
    fn extension_mismatch_warning() {
        assert_eq!(check_extension("stl", Format::Stl), None);
        assert_eq!(check_extension("xyz", Format::Stl), None);
        assert_eq!(
            check_extension(".STL", Format::Obj),
            Some(IoWarning::ExtensionMismatch {
                extension: "stl".into(),
                detected: Format::Obj
            })
        );
    }

    #[test]
    fn small_inline_sniffs() {
        assert_eq!(
            Format::sniff(b"ply\nformat ascii 1.0\n", 21),
            Some(Format::Ply)
        );
        assert_eq!(
            Format::sniff(b"ISO-10303-21;\nHEADER;", 20),
            Some(Format::Step)
        );
        assert_eq!(Format::sniff(b"v 0 0 0\nv 1 0 0\n", 16), Some(Format::Obj));
        assert_eq!(Format::sniff(b"hello world", 11), None);
        assert_eq!(Format::sniff(b"", 0), None);
    }
}
