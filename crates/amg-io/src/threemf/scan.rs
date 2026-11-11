//! The hot loop of the 3MF reader (note §10): the content of `<vertices>` and `<triangles>`.
//!
//! The byte range between the start and end tag is split at element boundaries into chunks of
//! about [`Tuning::chunk_bytes`] that rayon parses in parallel with a hand-written attribute
//! scanner. The scanner only handles the plain form every producer writes (`<vertex x=".."
//! y=".." z=".."/>`, known attribute names, no entities, comments or foreign elements). On
//! anything else it gives up and the whole range is parsed by [`reference`], a plain
//! quick-xml walk with the full namespace and strict-mode rules. Both paths share the value
//! parsers, so their results are identical; a differential test checks that on every fixture.

use amg_core::Point3;
use memchr::memmem;
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use rayon::prelude::*;

use super::Tuning;
use super::xml::{NS_CORE, NS_SLIC3R, Scopes, attr_value, is_xmlns, parse_index, parse_number};
use crate::options::ReadLimits;

/// No property index.
pub(crate) const NONE: u32 = u32::MAX;

/// Triangle properties as written: `pid` (0 = absent) and `p1 p2 p3` ([`NONE`] = absent).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct TriProp {
    pub pid: u32,
    pub p: [u32; 3],
}

impl TriProp {
    pub(crate) const EMPTY: TriProp = TriProp {
        pid: 0,
        p: [NONE; 3],
    };
}

/// What a `<triangles>` range holds.
#[derive(Debug, Default, PartialEq)]
pub(crate) struct Triangles {
    pub tris: Vec<[u32; 3]>,
    /// One entry per triangle, if any triangle has properties.
    pub props: Option<Vec<TriProp>>,
    /// `paint_color` strings by triangle index (Orca/Bambu).
    pub paint: Vec<(u32, Box<str>)>,
    /// `slic3rpe:mmu_segmentation` strings by triangle index (PrusaSlicer).
    pub mmu: Vec<(u32, Box<str>)>,
}

/// Settings the scanner and the reference parser share.
pub(crate) struct ScanCtx<'a> {
    pub strict: bool,
    pub limits: &'a ReadLimits,
    /// Namespace scopes at the `<vertices>` / `<triangles>` element.
    pub scopes: &'a Scopes,
    /// Qualified name of the child element as written in this part (`vertex`, `c:triangle`).
    pub child_qname: String,
    /// Prefixes bound to the PrusaSlicer namespace.
    pub slic3r_prefixes: Vec<String>,
    /// Checked per chunk; a cancelled scan returns early with partial data, and the caller's
    /// next checkpoint reports the cancel.
    pub ctx: &'a amg_jobs::JobCtx,
}

/// A located error: byte offset inside the range and message.
pub(crate) type ScanError = (usize, String);

/// Called with the offset and the attributes of every child element.
type EachChild<'f> = dyn FnMut(usize, Vec<(&str, &str)>) -> Result<(), ScanError> + 'f;

/// Which element a range holds.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) enum Kind {
    Vertices,
    Triangles,
}

impl Kind {
    fn local(self) -> &'static str {
        match self {
            Kind::Vertices => "vertex",
            Kind::Triangles => "triangle",
        }
    }
}

/// Upper bound of the element count of a range, for the limits (a fast byte search).
pub(crate) fn count_elements(range: &[u8], child_qname: &str) -> u64 {
    const CHUNK: usize = 8 << 20;
    let pat = format!("<{child_qname}");
    let finder = memmem::Finder::new(pat.as_bytes());
    let n = range.len().div_ceil(CHUNK);
    (0..n)
        .into_par_iter()
        .map(|k| {
            // Matches that start inside chunk k; the window overlaps the next chunk.
            let a = k * CHUNK;
            let b = ((k + 1) * CHUNK).min(range.len());
            let w = &range[a..(b + pat.len() - 1).min(range.len())];
            finder.find_iter(w).filter(|&i| a + i < b).count() as u64
        })
        .sum()
}

/// Splits `range` at child element starts into chunks of about `target` bytes.
fn split(range: &[u8], child_qname: &str, target: usize) -> Vec<(usize, usize)> {
    let pat = format!("<{child_qname}");
    let finder = memmem::Finder::new(pat.as_bytes());
    let mut out = Vec::new();
    let mut start = 0usize;
    while start < range.len() {
        let want = start.saturating_add(target.max(1));
        if want >= range.len() {
            out.push((start, range.len()));
            break;
        }
        match finder.find(&range[want..]) {
            Some(i) => {
                out.push((start, want + i));
                start = want + i;
            }
            None => {
                out.push((start, range.len()));
                break;
            }
        }
    }
    out
}

// ------------------------------------------------------------------------------- vertices

/// Parses the content of `<vertices>`.
pub(crate) fn vertices(
    range: &[u8],
    cx: &ScanCtx<'_>,
    tuning: &Tuning,
) -> Result<Vec<Point3>, ScanError> {
    if !tuning.reference_only {
        let chunks = split(range, &cx.child_qname, tuning.chunk_bytes);
        let parts: Vec<Option<Result<Vec<Point3>, ScanError>>> = chunks
            .par_iter()
            .map(|&(a, b)| {
                if cx.ctx.is_cancelled() {
                    return Some(Ok(Vec::new()));
                }
                fast_vertices(&range[a..b], cx).map(|r| r.map_err(|(o, m)| (o + a, m)))
            })
            .collect();
        if parts.iter().all(Option::is_some) {
            let total: usize = parts
                .iter()
                .map(|p| p.as_ref().map_or(0, |r| r.as_ref().map_or(0, Vec::len)))
                .sum();
            let mut out = Vec::with_capacity(total);
            for p in parts.into_iter().flatten() {
                out.extend(p?);
            }
            return Ok(out);
        }
    }
    let mut out = Vec::new();
    reference(range, cx, Kind::Vertices, &mut |off, attrs| {
        let mut xyz = [None::<f64>; 3];
        for (name, value) in attrs {
            let k = match name {
                "x" => 0,
                "y" => 1,
                "z" => 2,
                _ => continue,
            };
            xyz[k] = Some(parse_number(value, cx.strict).map_err(|m| (off, m))?);
        }
        match xyz {
            [Some(x), Some(y), Some(z)] => out.push(Point3::new(x, y, z)),
            _ => return Err((off, "a vertex needs x, y and z".into())),
        }
        Ok(())
    })?;
    Ok(out)
}

/// `None` = the scanner gives up on this chunk.
fn fast_vertices(chunk: &[u8], cx: &ScanCtx<'_>) -> Option<Result<Vec<Point3>, ScanError>> {
    let mut out = Vec::with_capacity(chunk.len() / 40);
    let mut sc = Scanner::new(chunk, cx);
    loop {
        match sc.next_element()? {
            None => return Some(Ok(out)),
            Some(start) => {
                let mut xyz = [None::<f64>; 3];
                let mut seen = 0u8;
                while let Some((name, value)) = sc.attribute()? {
                    let k = match name {
                        b"x" => 0,
                        b"y" => 1,
                        b"z" => 2,
                        _ => return None,
                    };
                    if seen & (1 << k) != 0 {
                        return None;
                    }
                    seen |= 1 << k;
                    let v = std::str::from_utf8(value).ok()?;
                    match parse_number(v, cx.strict) {
                        Ok(v) => xyz[k] = Some(v),
                        Err(m) => return Some(Err((start, m))),
                    }
                }
                sc.close()?;
                match xyz {
                    [Some(x), Some(y), Some(z)] => out.push(Point3::new(x, y, z)),
                    _ => return Some(Err((start, "a vertex needs x, y and z".into()))),
                }
            }
        }
    }
}

// ------------------------------------------------------------------------------ triangles

/// One triangle's attributes after value parsing.
#[derive(Default)]
struct TriAttrs {
    v: [Option<u32>; 3],
    pid: Option<u32>,
    p: [Option<u32>; 3],
    paint: Option<Box<str>>,
    mmu: Option<Box<str>>,
}

impl TriAttrs {
    fn finish(self, off: usize, out: &mut Triangles) -> Result<(), ScanError> {
        let [Some(a), Some(b), Some(c)] = self.v else {
            return Err((off, "a triangle needs v1, v2 and v3".into()));
        };
        let i = out.tris.len();
        out.tris.push([a, b, c]);
        let prop = TriProp {
            pid: self.pid.unwrap_or(0),
            p: self.p.map(|p| p.unwrap_or(NONE)),
        };
        if prop != TriProp::EMPTY || out.props.is_some() {
            let props = out.props.get_or_insert_with(|| vec![TriProp::EMPTY; i]);
            props.push(prop);
        }
        if let Some(s) = self.paint {
            out.paint.push((i as u32, s));
        }
        if let Some(s) = self.mmu {
            out.mmu.push((i as u32, s));
        }
        Ok(())
    }
}

/// What an attribute of a triangle means.
enum TriAttr {
    V(usize),
    Pid,
    P(usize),
    Paint,
    Mmu,
    /// Not ours (another namespace): ignored.
    Ignore,
    /// Unprefixed attribute that core does not define.
    Unknown,
}

fn classify_tri_attr(name: &str, cx: &ScanCtx<'_>) -> TriAttr {
    match name {
        "v1" => TriAttr::V(0),
        "v2" => TriAttr::V(1),
        "v3" => TriAttr::V(2),
        "pid" => TriAttr::Pid,
        "p1" => TriAttr::P(0),
        "p2" => TriAttr::P(1),
        "p3" => TriAttr::P(2),
        "paint_color" => TriAttr::Paint,
        _ => match name.split_once(':') {
            Some((p, "mmu_segmentation")) if cx.slic3r_prefixes.iter().any(|x| x == p) => {
                TriAttr::Mmu
            }
            Some(_) => TriAttr::Ignore,
            None => TriAttr::Unknown,
        },
    }
}

fn apply_tri_attr(
    t: &mut TriAttrs,
    kind: TriAttr,
    name: &str,
    value: &str,
    cx: &ScanCtx<'_>,
) -> Result<(), String> {
    match kind {
        TriAttr::V(k) => t.v[k] = Some(parse_index(value)?),
        TriAttr::Pid => t.pid = Some(super::xml::parse_id(value)?),
        TriAttr::P(k) => t.p[k] = Some(parse_index(value)?),
        TriAttr::Paint => {
            if cx.strict {
                return Err(format!("attribute {name} is not defined by 3MF core"));
            }
            let v = super::xml::trim(value);
            if !v.is_empty() {
                t.paint = Some(v.into());
            }
        }
        TriAttr::Mmu => {
            let v = super::xml::trim(value);
            if !v.is_empty() {
                t.mmu = Some(v.into());
            }
        }
        TriAttr::Ignore => {}
        TriAttr::Unknown => {
            if cx.strict {
                return Err(format!("attribute {name} is not defined by 3MF core"));
            }
        }
    }
    Ok(())
}

/// Parses the content of `<triangles>`.
pub(crate) fn triangles(
    range: &[u8],
    cx: &ScanCtx<'_>,
    tuning: &Tuning,
) -> Result<Triangles, ScanError> {
    if !tuning.reference_only {
        let chunks = split(range, &cx.child_qname, tuning.chunk_bytes);
        let parts: Vec<Option<Result<Triangles, ScanError>>> = chunks
            .par_iter()
            .map(|&(a, b)| {
                if cx.ctx.is_cancelled() {
                    return Some(Ok(Triangles::default()));
                }
                fast_triangles(&range[a..b], cx).map(|r| r.map_err(|(o, m)| (o + a, m)))
            })
            .collect();
        if parts.iter().all(Option::is_some) {
            return merge_triangles(parts.into_iter().flatten());
        }
    }
    let mut out = Triangles::default();
    reference(range, cx, Kind::Triangles, &mut |off, attrs| {
        let mut t = TriAttrs::default();
        for (name, value) in attrs {
            let kind = classify_tri_attr(name, cx);
            apply_tri_attr(&mut t, kind, name, value, cx).map_err(|m| (off, m))?;
        }
        t.finish(off, &mut out)
    })?;
    Ok(out)
}

fn merge_triangles(
    parts: impl Iterator<Item = Result<Triangles, ScanError>>,
) -> Result<Triangles, ScanError> {
    let parts: Vec<Triangles> = parts.collect::<Result<_, _>>()?;
    let total: usize = parts.iter().map(|p| p.tris.len()).sum();
    let any_props = parts.iter().any(|p| p.props.is_some());
    let mut out = Triangles {
        tris: Vec::with_capacity(total),
        props: any_props.then(|| Vec::with_capacity(total)),
        ..Triangles::default()
    };
    for p in parts {
        let base = out.tris.len() as u32;
        let n = p.tris.len();
        out.tris.extend(p.tris);
        if let Some(props) = out.props.as_mut() {
            match p.props {
                Some(pp) => props.extend(pp),
                None => props.extend(std::iter::repeat_n(TriProp::EMPTY, n)),
            }
        }
        out.paint
            .extend(p.paint.into_iter().map(|(i, s)| (i + base, s)));
        out.mmu
            .extend(p.mmu.into_iter().map(|(i, s)| (i + base, s)));
    }
    Ok(out)
}

fn fast_triangles(chunk: &[u8], cx: &ScanCtx<'_>) -> Option<Result<Triangles, ScanError>> {
    let mut out = Triangles {
        tris: Vec::with_capacity(chunk.len() / 36),
        ..Triangles::default()
    };
    let mut sc = Scanner::new(chunk, cx);
    loop {
        let start = match sc.next_element()? {
            None => return Some(Ok(out)),
            Some(s) => s,
        };
        let mut t = TriAttrs::default();
        let mut seen = 0u8;
        while let Some((name, value)) = sc.attribute()? {
            let name = std::str::from_utf8(name).ok()?;
            let kind = classify_tri_attr(name, cx);
            let bit = match kind {
                TriAttr::V(k) => k as u8,
                TriAttr::Pid => 3,
                TriAttr::P(k) => 4 + k as u8,
                TriAttr::Paint => 7,
                // Anything with namespace semantics goes to the reference parser.
                TriAttr::Mmu | TriAttr::Ignore | TriAttr::Unknown => return None,
            };
            if seen & (1 << bit) != 0 {
                return None;
            }
            seen |= 1 << bit;
            let value = std::str::from_utf8(value).ok()?;
            if let Err(m) = apply_tri_attr(&mut t, kind, name, value, cx) {
                return Some(Err((start, m)));
            }
        }
        sc.close()?;
        if let Err(e) = t.finish(start, &mut out) {
            return Some(Err(e));
        }
    }
}

// -------------------------------------------------------------------------------- scanner

/// Byte scanner over a chunk of plain child elements. Every method returns `None` to give
/// up (the caller then uses the reference parser).
struct Scanner<'a> {
    b: &'a [u8],
    pos: usize,
    qname: &'a [u8],
    max_attr: usize,
}

fn ws(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | b'\r')
}

impl<'a> Scanner<'a> {
    fn new(b: &'a [u8], cx: &'a ScanCtx<'_>) -> Self {
        Scanner {
            b,
            pos: 0,
            qname: cx.child_qname.as_bytes(),
            max_attr: cx.limits.max_attr_value_len as usize,
        }
    }

    fn skip_ws(&mut self) {
        while self.pos < self.b.len() && ws(self.b[self.pos]) {
            self.pos += 1;
        }
    }

    /// Moves to the next `<qname`; `Some(None)` at the end of the chunk.
    fn next_element(&mut self) -> Option<Option<usize>> {
        self.skip_ws();
        if self.pos == self.b.len() {
            return Some(None);
        }
        let start = self.pos;
        let rest = &self.b[self.pos..];
        if rest.first() != Some(&b'<') || !rest[1..].starts_with(self.qname) {
            return None;
        }
        self.pos += 1 + self.qname.len();
        // The name must end here.
        match self.b.get(self.pos) {
            Some(&c) if ws(c) || c == b'/' || c == b'>' => Some(Some(start)),
            _ => None,
        }
    }

    /// Next `name="value"`; `Some(None)` when the start tag ends (positioned at `/>` or `>`).
    fn attribute(&mut self) -> Option<Option<(&'a [u8], &'a [u8])>> {
        self.skip_ws();
        let c = *self.b.get(self.pos)?;
        if c == b'/' || c == b'>' {
            return Some(None);
        }
        let ns = self.pos;
        while self.pos < self.b.len() {
            let c = self.b[self.pos];
            if c == b'=' || ws(c) {
                break;
            }
            if !(c.is_ascii_alphanumeric() || matches!(c, b'_' | b':' | b'-' | b'.')) {
                return None;
            }
            self.pos += 1;
        }
        let name = &self.b[ns..self.pos];
        if name.is_empty() || name.starts_with(b"xmlns") {
            return None;
        }
        self.skip_ws();
        if self.b.get(self.pos) != Some(&b'=') {
            return None;
        }
        self.pos += 1;
        self.skip_ws();
        let q = *self.b.get(self.pos)?;
        if q != b'"' && q != b'\'' {
            return None;
        }
        self.pos += 1;
        let vs = self.pos;
        let len = memchr::memchr(q, &self.b[vs..])?;
        let value = &self.b[vs..vs + len];
        if len > self.max_attr || memchr::memchr2(b'&', b'<', value).is_some() {
            return None;
        }
        self.pos = vs + len + 1;
        // Attributes must be separated by white space.
        match self.b.get(self.pos) {
            Some(&c) if ws(c) || c == b'/' || c == b'>' => Some(Some((name, value))),
            _ => None,
        }
    }

    /// Consumes `/>` or `>` + `</qname>`.
    fn close(&mut self) -> Option<()> {
        if self.b[self.pos..].starts_with(b"/>") {
            self.pos += 2;
            return Some(());
        }
        if self.b.get(self.pos) != Some(&b'>') {
            return None;
        }
        self.pos += 1;
        self.skip_ws();
        let rest = &self.b[self.pos..];
        if !rest.starts_with(b"</") || !rest[2..].starts_with(self.qname) {
            return None;
        }
        self.pos += 2 + self.qname.len();
        self.skip_ws();
        if self.b.get(self.pos) != Some(&b'>') {
            return None;
        }
        self.pos += 1;
        Some(())
    }
}

// ------------------------------------------------------------------------- reference path

/// Walks the content of `<vertices>` / `<triangles>` with quick-xml and calls `each` with the
/// offset and the attributes of every child element of the expected kind.
///
/// Rules (both modes): child elements in the core namespace other than the expected one are
/// errors in strict mode and ignored otherwise; elements of other namespaces are skipped;
/// text other than white space is an error in strict mode. Duplicate attributes are errors.
pub(crate) fn reference(
    range: &[u8],
    cx: &ScanCtx<'_>,
    kind: Kind,
    each: &mut EachChild<'_>,
) -> Result<(), ScanError> {
    let mut r = Reader::from_reader(range);
    let cfg = r.config_mut();
    cfg.check_end_names = true;
    cfg.allow_unmatched_ends = false;
    let mut scopes = Scopes::new();
    for (p, u) in cx.scopes.all() {
        scopes.push_decl(p, u);
    }
    let mut depth = 0usize;
    let mut skip_depth: Option<usize> = None;
    let mut n = 0u32;
    loop {
        n = n.wrapping_add(1);
        if n.is_multiple_of(4096) && cx.ctx.is_cancelled() {
            return Ok(());
        }
        let off = r.buffer_position() as usize;
        let ev = r
            .read_event()
            .map_err(|e| (r.error_position() as usize, e.to_string()))?;
        match ev {
            Event::Start(e) => {
                depth += 1;
                scopes.push(&e);
                if depth > cx.limits.max_xml_depth as usize {
                    return Err((off, "XML nesting is too deep".into()));
                }
                // A foreign subtree, or children inside a vertex/triangle, is skipped.
                if skip_depth.is_none() && (depth > 1 || !child(&scopes, &e, kind, cx, off, each)?)
                {
                    if depth > 1 && cx.strict && in_core(&scopes, &e) {
                        return Err((off, format!("unexpected element <{}>", e.name().0)));
                    }
                    skip_depth = Some(depth);
                }
            }
            Event::Empty(e) => {
                scopes.push(&e);
                if skip_depth.is_none() {
                    if depth > 0 {
                        if cx.strict && in_core(&scopes, &e) {
                            return Err((off, format!("unexpected element <{}>", e.name().0)));
                        }
                    } else {
                        child(&scopes, &e, kind, cx, off, each)?;
                    }
                }
                scopes.pop();
            }
            Event::End(_) => {
                if skip_depth == Some(depth) {
                    skip_depth = None;
                }
                depth = depth.saturating_sub(1);
                scopes.pop();
            }
            Event::Text(t) => {
                if cx.strict && skip_depth.is_none() && !t.trim_ascii().is_empty() {
                    return Err((off, "unexpected text".into()));
                }
            }
            Event::CData(_) | Event::GeneralRef(_) => {
                if cx.strict && skip_depth.is_none() {
                    return Err((off, "unexpected text".into()));
                }
            }
            Event::DocType(_) => return Err((off, "DTDs are not allowed".into())),
            Event::Eof => break,
            Event::Comment(_) | Event::PI(_) | Event::Decl(_) => {}
        }
    }
    Ok(())
}

fn in_core(scopes: &Scopes, e: &BytesStart<'_>) -> bool {
    scopes.element(e.name().0).0 == Some(NS_CORE)
}

/// Handles a top-level child. Returns false if it is not one of ours (skip it).
fn child(
    scopes: &Scopes,
    e: &BytesStart<'_>,
    kind: Kind,
    cx: &ScanCtx<'_>,
    off: usize,
    each: &mut EachChild<'_>,
) -> Result<bool, ScanError> {
    let (ns, local) = scopes.element(e.name().0);
    if ns != Some(NS_CORE) {
        return Ok(false);
    }
    if local != kind.local() {
        if cx.strict {
            return Err((off, format!("unexpected element <{}>", e.name().0)));
        }
        return Ok(false);
    }
    let mut owned: Vec<(String, String)> = Vec::new();
    for a in e.attributes().with_checks(true) {
        let a = a.map_err(|x| (off, x.to_string()))?;
        if is_xmlns(a.key.0) {
            continue;
        }
        let v = attr_value(&a, cx.limits.max_attr_value_len).map_err(|m| (off, m))?;
        // Resolve the prefix of namespaced attributes against the current scopes, so that
        // a local re-declaration is honoured; the qualified name is rewritten to the
        // canonical prefix for PrusaSlicer attributes.
        let key = a.key.0;
        let key = match key.split_once(':') {
            Some((p, l)) => match scopes.uri(p) {
                Some(NS_SLIC3R) if l == "mmu_segmentation" => cx
                    .slic3r_prefixes
                    .first()
                    .map_or_else(|| key.to_owned(), |c| format!("{c}:{l}")),
                Some(_) => key.to_owned(),
                None if cx.strict => {
                    return Err((off, format!("attribute prefix {p:?} is not declared")));
                }
                None => key.to_owned(),
            },
            None => key.to_owned(),
        };
        owned.push((key, v.into_owned()));
    }
    let attrs: Vec<(&str, &str)> = owned
        .iter()
        .map(|(k, v)| (k.as_str(), v.as_str()))
        .collect();
    each(off, attrs)?;
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cx<'a>(
        scopes: &'a Scopes,
        limits: &'a ReadLimits,
        child: &str,
        strict: bool,
    ) -> ScanCtx<'a> {
        ScanCtx {
            strict,
            limits,
            scopes,
            child_qname: child.into(),
            slic3r_prefixes: vec!["slic3rpe".into()],
            ctx: ctx(),
        }
    }

    fn ctx() -> &'static amg_jobs::JobCtx {
        static C: std::sync::OnceLock<amg_jobs::JobCtx> = std::sync::OnceLock::new();
        C.get_or_init(amg_jobs::JobCtx::detached)
    }

    fn core_scopes() -> Scopes {
        let mut s = Scopes::new();
        s.push_decl("", NS_CORE);
        s.push_decl("slic3rpe", NS_SLIC3R);
        s
    }

    fn both_v(
        range: &str,
        strict: bool,
    ) -> (
        Result<Vec<Point3>, ScanError>,
        Result<Vec<Point3>, ScanError>,
    ) {
        let s = core_scopes();
        let l = ReadLimits::default();
        let c = cx(&s, &l, "vertex", strict);
        let fast = vertices(
            range.as_bytes(),
            &c,
            &Tuning {
                chunk_bytes: 7,
                reference_only: false,
            },
        );
        let slow = vertices(
            range.as_bytes(),
            &c,
            &Tuning {
                chunk_bytes: 7,
                reference_only: true,
            },
        );
        (fast, slow)
    }

    fn both_t(
        range: &str,
        strict: bool,
    ) -> (Result<Triangles, ScanError>, Result<Triangles, ScanError>) {
        let s = core_scopes();
        let l = ReadLimits::default();
        let c = cx(&s, &l, "triangle", strict);
        let fast = triangles(
            range.as_bytes(),
            &c,
            &Tuning {
                chunk_bytes: 9,
                reference_only: false,
            },
        );
        let slow = triangles(
            range.as_bytes(),
            &c,
            &Tuning {
                chunk_bytes: 9,
                reference_only: true,
            },
        );
        (fast, slow)
    }

    #[test]
    fn vertices_agree() {
        let cases = [
            r#"<vertex x="1" y="2" z="3"/><vertex x="-1.5" y='0' z="1e2" />"#,
            "\n  <vertex z=\"3\" x=\"1\"\ty=\"2\"></vertex>\n",
            r#"<vertex x="1" y="2" z="3"/><!-- c --><vertex x="4" y="5" z="6"/>"#,
            r#"<vertex x="&#49;" y="2" z="3"/>"#,
            r#"<vertex x="1" y="2" z="3" q:extra="1" xmlns:q="urn:q"/>"#,
            r#"<vertex x="1" y="2"/>"#,
            r#"<vertex x="1" y="2" z="nan"/>"#,
            r#"<vertex x="1" y="2" z="3" x="4"/>"#,
            r#"<vertex x="1." y="2" z="3"/>"#,
            r#"<vertex x="1,5" y="2" z="3"/>"#,
            "",
        ];
        for c in cases {
            for strict in [false, true] {
                let (f, s) = both_v(c, strict);
                assert_eq!(f.is_ok(), s.is_ok(), "{c} strict={strict}: {f:?} vs {s:?}");
                if let (Ok(f), Ok(s)) = (&f, &s) {
                    assert_eq!(f, s, "{c}");
                }
            }
        }
        let (f, _) = both_v(
            r#"<vertex x="1" y="2" z="3"/><vertex x="4" y="5" z="6"/>"#,
            true,
        );
        assert_eq!(f.expect("ok").len(), 2);
    }

    #[test]
    fn triangles_agree() {
        let cases = [
            r#"<triangle v1="0" v2="1" v3="2"/><triangle v1="2" v2="1" v3="3" pid="5" p1="1"/>"#,
            r#"<triangle v1="0" v2="1" v3="2" paint_color="4"/><triangle v1="0" v2="1" v3="2"/>"#,
            r#"<triangle v1="0" v2="1" v3="2" slic3rpe:mmu_segmentation="8"/>"#,
            r#"<triangle v1="0" v2="1" v3="2" foo="1"/>"#,
            r#"<triangle v1="0" v2="1"/>"#,
            r#"<triangle v1="-1" v2="1" v3="2"/>"#,
            r#"<triangle v1="0" v2="1" v3="2" pid="0"/>"#,
            r#"<triangle v1="0" v2="1" v3="2"/>text"#,
            r#"<other v1="0"/><triangle v1="0" v2="1" v3="2"/>"#,
        ];
        for c in cases {
            for strict in [false, true] {
                let (f, s) = both_t(c, strict);
                assert_eq!(f.is_ok(), s.is_ok(), "{c} strict={strict}: {f:?} vs {s:?}");
                if let (Ok(f), Ok(s)) = (&f, &s) {
                    assert_eq!(f, s, "{c}");
                }
            }
        }
        let (f, _) = both_t(cases[0], false);
        let f = f.expect("ok");
        assert_eq!(f.tris, vec![[0, 1, 2], [2, 1, 3]]);
        let props = f.props.expect("props");
        assert_eq!(props[0], TriProp::EMPTY);
        assert_eq!(
            props[1],
            TriProp {
                pid: 5,
                p: [1, NONE, NONE]
            }
        );
        let (f, _) = both_t(cases[1], false);
        assert_eq!(f.expect("ok").paint, vec![(0, "4".into())]);
        let (f, _) = both_t(cases[2], false);
        assert_eq!(f.expect("ok").mmu, vec![(0, "8".into())]);
        // paint_color is not core: strict mode rejects it.
        assert!(both_t(cases[1], true).0.is_err());
    }

    #[test]
    fn chunks_cover_the_range() {
        let s = "<v a/><v b/><v c/>".as_bytes();
        for t in 1..20 {
            let c = split(s, "v", t);
            assert_eq!(c.first().map(|x| x.0), Some(0));
            assert_eq!(c.last().map(|x| x.1), Some(s.len()));
            for w in c.windows(2) {
                assert_eq!(w[0].1, w[1].0);
            }
        }
        assert_eq!(count_elements(s, "v"), 3);
    }
}
