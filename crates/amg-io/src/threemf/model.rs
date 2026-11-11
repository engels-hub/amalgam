//! One 3MF model part (`*.model`) → plain data: resources, build items, metadata.
//!
//! quick-xml streams the structure; the content of `<vertices>` and `<triangles>` is handed to
//! the parallel scanner (`scan`) as one byte range, after which quick-xml restarts behind it.
//! Namespaces, the element stack and end-name matching are therefore ours, not quick-xml's.

use std::collections::{BTreeMap, BTreeSet};

use amg_core::units::LengthUnit;
use amg_core::{Point3, Transform};
use amg_jobs::JobCtx;
use amg_mesh::Rgba8;
use memchr::memmem;
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use rayon::prelude::*;

use super::Tuning;
use super::package::check_attrs;
use super::scan::{self, ScanCtx, Triangles};
use super::xml::{
    NS_BEAM, NS_CORE, NS_MAT, NS_PROD, NS_SLIC3R, NS_TRISETS, NS_XML, Scopes, attr_value, is_xmlns,
    malformed_at, parse_color, parse_id, parse_index, parse_number, parse_transform, trim,
};
use crate::error::IoError;
use crate::options::ReadLimits;
use crate::scene::ObjectKind;

/// Extensions whose content we read; anything else that is *required* is refused.
pub(crate) const SUPPORTED_REQUIRED: [&str; 3] = [NS_MAT, NS_PROD, NS_TRISETS];

/// Metadata names defined by core (Table 8-1).
const CORE_METADATA: [&str; 9] = [
    "Title",
    "Designer",
    "Description",
    "Copyright",
    "LicenseTerms",
    "Rating",
    "CreationDate",
    "ModificationDate",
    "Application",
];

/// A property group (core base materials and the Materials extension resources).
#[derive(Clone, Debug)]
pub(crate) enum PropGroup {
    Base(Vec<(String, Rgba8)>),
    Colors(Vec<Rgba8>),
    Texture2d {
        /// Part key of the image.
        path: String,
    },
    TexCoords {
        uvs: Vec<[f64; 2]>,
    },
    Composite {
        matid: u32,
        matindices: Vec<u32>,
        values: Vec<Vec<f64>>,
    },
    Multi {
        pids: Vec<u32>,
        multis: Vec<Vec<u32>>,
    },
}

impl PropGroup {
    /// Number of indexable entries.
    pub(crate) fn len(&self) -> usize {
        match self {
            PropGroup::Base(v) => v.len(),
            PropGroup::Colors(v) => v.len(),
            PropGroup::Texture2d { .. } => 0,
            PropGroup::TexCoords { uvs, .. } => uvs.len(),
            PropGroup::Composite { values, .. } => values.len(),
            PropGroup::Multi { multis, .. } => multis.len(),
        }
    }

    fn label(&self) -> &'static str {
        match self {
            PropGroup::Base(_) => "basematerials",
            PropGroup::Colors(_) => "colorgroup",
            PropGroup::Texture2d { .. } => "texture2d",
            PropGroup::TexCoords { .. } => "texture2dgroup",
            PropGroup::Composite { .. } => "compositematerials",
            PropGroup::Multi { .. } => "multiproperties",
        }
    }
}

/// A mesh as read: file numbers, file units.
#[derive(Debug, Default)]
pub(crate) struct MeshData {
    pub positions: Vec<Point3>,
    pub tris: Triangles,
}

/// `<component>`.
#[derive(Clone, Debug)]
pub(crate) struct Component {
    pub objectid: u32,
    pub transform: Transform,
    /// Resolved part key of `p:path`.
    pub path: Option<String>,
    pub uuid: Option<String>,
    pub offset: usize,
}

/// What an object holds.
#[derive(Debug)]
pub(crate) enum Content {
    Mesh(MeshData),
    Components(Vec<Component>),
    /// Neither (an error in strict mode).
    Empty,
}

/// `<object>`.
#[derive(Debug)]
pub(crate) struct Object {
    pub id: u32,
    pub kind: ObjectKind,
    pub kind_other: bool,
    pub name: Option<String>,
    pub pid: Option<u32>,
    pub pindex: Option<u32>,
    pub uuid: Option<String>,
    pub content: Content,
    pub offset: usize,
}

/// `<item>`.
#[derive(Clone, Debug)]
pub(crate) struct Item {
    pub objectid: u32,
    pub transform: Transform,
    pub path: Option<String>,
    pub uuid: Option<String>,
    pub offset: usize,
}

/// A parsed model part.
#[derive(Debug, Default)]
pub(crate) struct ModelPart {
    pub key: String,
    pub name: String,
    pub unit: Option<LengthUnit>,
    pub required: Vec<String>,
    pub metadata: Vec<(String, String)>,
    pub objects: BTreeMap<u32, Object>,
    pub groups: BTreeMap<u32, PropGroup>,
    pub build: Vec<Item>,
    pub has_build: bool,
    pub build_uuid: Option<String>,
    /// Namespaces of ignored optional content.
    pub ignored_ns: BTreeSet<String>,
    /// Ignored content kinds and counts (beam lattices, display properties, ...).
    pub ignored: BTreeMap<String, u64>,
    /// The raw XML, kept for error locations.
    pub data: Vec<u8>,
}

impl ModelPart {
    /// Error at byte `offset` of this part.
    pub(crate) fn err(&self, offset: usize, msg: impl Into<String>) -> IoError {
        malformed_at(&self.name, &self.data, offset, msg)
    }

    /// True if `ns` is listed in `requiredextensions`.
    pub(crate) fn requires(&self, ns: &str) -> bool {
        self.required.iter().any(|r| r == ns)
    }
}

fn unit_of(s: &str) -> Option<LengthUnit> {
    Some(match s {
        "micron" => LengthUnit::Micrometre,
        "millimeter" => LengthUnit::Millimetre,
        "centimeter" => LengthUnit::Centimetre,
        "inch" => LengthUnit::Inch,
        "foot" => LengthUnit::Foot,
        "meter" => LengthUnit::Metre,
        _ => return None,
    })
}

fn kind_of(s: &str) -> Option<(ObjectKind, bool)> {
    Some(match s {
        "model" => (ObjectKind::Model, false),
        "solidsupport" => (ObjectKind::SolidSupport, false),
        "support" => (ObjectKind::Support, false),
        "surface" => (ObjectKind::Surface, false),
        "other" => (ObjectKind::Other, true),
        _ => return None,
    })
}

/// `8-4-4-4-12` hex digits.
pub(crate) fn is_uuid(s: &str) -> bool {
    let parts: Vec<&str> = s.split('-').collect();
    parts.len() == 5
        && parts
            .iter()
            .zip([8, 4, 4, 4, 12])
            .all(|(p, n)| p.len() == n && p.bytes().all(|b| b.is_ascii_hexdigit()))
}

/// One parsed attribute: namespace (None for unprefixed), local name, value.
struct A {
    ns: Option<String>,
    local: String,
    qname: String,
    value: String,
}

/// Event stream with our own element stack.
struct Xml<'d> {
    data: &'d [u8],
    base: usize,
    r: Reader<&'d [u8]>,
    scopes: Scopes,
    stack: Vec<String>,
    pending_pop: bool,
    limits: &'d ReadLimits,
    part: &'d str,
}

enum Ev<'d> {
    Start(BytesStart<'d>, usize),
    Empty(BytesStart<'d>, usize),
    End(usize),
    Text(String),
    Eof,
}

fn reader(data: &[u8]) -> Reader<&[u8]> {
    let mut r = Reader::from_reader(data);
    let cfg = r.config_mut();
    cfg.check_end_names = false;
    cfg.allow_unmatched_ends = true;
    cfg.expand_empty_elements = false;
    r
}

impl<'d> Xml<'d> {
    fn new(data: &'d [u8], part: &'d str, limits: &'d ReadLimits) -> Self {
        Xml {
            data,
            base: 0,
            r: reader(data),
            scopes: Scopes::new(),
            stack: Vec::new(),
            pending_pop: false,
            limits,
            part,
        }
    }

    fn err(&self, offset: usize, msg: impl Into<String>) -> IoError {
        malformed_at(self.part, self.data, offset, msg)
    }

    fn pos(&self) -> usize {
        self.base + self.r.buffer_position() as usize
    }

    fn reposition(&mut self, abs: usize) {
        self.base = abs;
        self.r = reader(&self.data[abs..]);
    }

    fn depth(&self) -> usize {
        self.stack.len()
    }

    fn next(&mut self) -> Result<Ev<'d>, IoError> {
        if self.pending_pop {
            self.pending_pop = false;
            self.scopes.pop();
        }
        loop {
            let off = self.pos();
            let ev = match self.r.read_event() {
                Ok(e) => e,
                Err(e) => {
                    let at = self.base + self.r.error_position() as usize;
                    return Err(self.err(at, e.to_string()));
                }
            };
            return Ok(match ev {
                Event::Start(e) => {
                    if self.stack.len() + 1 > self.limits.max_xml_depth as usize {
                        return Err(IoError::LimitExceeded {
                            what: "XML depth",
                            limit: u64::from(self.limits.max_xml_depth),
                            actual: self.stack.len() as u64 + 1,
                        });
                    }
                    check_attrs(&e, self.limits).map_err(|m| self.err(off, m))?;
                    self.stack.push(e.name().0.to_owned());
                    self.scopes.push(&e);
                    Ev::Start(e, off)
                }
                Event::Empty(e) => {
                    if self.stack.len() + 1 > self.limits.max_xml_depth as usize {
                        return Err(IoError::LimitExceeded {
                            what: "XML depth",
                            limit: u64::from(self.limits.max_xml_depth),
                            actual: self.stack.len() as u64 + 1,
                        });
                    }
                    check_attrs(&e, self.limits).map_err(|m| self.err(off, m))?;
                    self.scopes.push(&e);
                    self.pending_pop = true;
                    Ev::Empty(e, off)
                }
                Event::End(e) => {
                    let name = e.name().0.to_owned();
                    match self.stack.pop() {
                        Some(open) if open == name => {}
                        Some(open) => {
                            return Err(
                                self.err(off, format!("expected </{open}>, found </{name}>"))
                            );
                        }
                        None => return Err(self.err(off, format!("unexpected </{name}>"))),
                    }
                    self.scopes.pop();
                    Ev::End(off)
                }
                Event::Text(t) => Ev::Text(t.to_string()),
                Event::CData(t) => Ev::Text(t.to_string()),
                Event::GeneralRef(r) => {
                    let name = r.to_string();
                    let c = match name.as_str() {
                        "amp" => "&".to_owned(),
                        "lt" => "<".to_owned(),
                        "gt" => ">".to_owned(),
                        "quot" => "\"".to_owned(),
                        "apos" => "'".to_owned(),
                        n => {
                            let v = n
                                .strip_prefix("#x")
                                .and_then(|h| u32::from_str_radix(h, 16).ok())
                                .or_else(|| n.strip_prefix('#').and_then(|d| d.parse().ok()))
                                .and_then(char::from_u32)
                                .ok_or_else(|| self.err(off, format!("unknown entity &{n};")))?;
                            v.to_string()
                        }
                    };
                    Ev::Text(c)
                }
                Event::DocType(_) => return Err(self.err(off, "DTDs are not allowed")),
                Event::Eof => {
                    if let Some(open) = self.stack.last() {
                        return Err(self.err(self.data.len(), format!("<{open}> is not closed")));
                    }
                    Ev::Eof
                }
                Event::Comment(_) | Event::PI(_) | Event::Decl(_) => continue,
            });
        }
    }

    /// Skips the rest of the element whose start was just read.
    fn skip(&mut self) -> Result<(), IoError> {
        let d = self.depth();
        loop {
            match self.next()? {
                Ev::End(_) if self.depth() < d => return Ok(()),
                Ev::Eof => return Ok(()),
                _ => {}
            }
        }
    }

    /// Text content of the element whose start was just read (child elements are skipped).
    fn text(&mut self) -> Result<String, IoError> {
        let d = self.depth();
        let mut s = String::new();
        loop {
            match self.next()? {
                Ev::Text(t) if self.depth() == d => s.push_str(&t),
                Ev::Start(..) => self.skip()?,
                Ev::End(_) if self.depth() < d => return Ok(s),
                Ev::Eof => return Ok(s),
                _ => {}
            }
        }
    }

    /// The attributes of `e`, namespace-resolved.
    fn attrs(&self, e: &BytesStart<'_>, off: usize, strict: bool) -> Result<Vec<A>, IoError> {
        let mut out = Vec::new();
        for a in e.attributes().with_checks(false) {
            let a = a.map_err(|x| self.err(off, x.to_string()))?;
            let q = a.key.0;
            if is_xmlns(q) {
                continue;
            }
            let value = attr_value(&a, self.limits.max_attr_value_len)
                .map_err(|m| self.err(off, m))?
                .into_owned();
            let (ns, local) = match q.split_once(':') {
                Some((p, l)) => match self.scopes.uri(p) {
                    Some(u) => (Some(u.to_owned()), l.to_owned()),
                    None if strict => {
                        return Err(
                            self.err(off, format!("attribute prefix {p:?} is not declared"))
                        );
                    }
                    None => (Some(String::new()), l.to_owned()),
                },
                None => (None, q.to_owned()),
            };
            out.push(A {
                ns,
                local,
                qname: q.to_owned(),
                value,
            });
        }
        Ok(out)
    }

    fn element_ns(&self, e: &BytesStart<'_>) -> (Option<String>, String) {
        let (ns, l) = self.scopes.element(e.name().0);
        (ns.map(str::to_owned), l.to_owned())
    }

    /// Byte range of the content of the element whose start was just read, and the offset
    /// of its end tag. Falls back to an event walk when the content has comments or CDATA
    /// (a `</name>` inside a comment must not end the range).
    fn content_range(&self, qname: &str, ctx: &JobCtx) -> Result<(usize, usize), IoError> {
        // Searched in windows, with a cancel check per window.
        const WINDOW: usize = 32 << 20;
        let start = self.pos();
        let pat = format!("</{qname}");
        let finder = memmem::Finder::new(pat.as_bytes());
        let mut from = start;
        let mut end = None;
        while from < self.data.len() && end.is_none() {
            ctx.checkpoint()?;
            let to = (from + WINDOW).min(self.data.len());
            let w = &self.data[from..(to + pat.len()).min(self.data.len())];
            let mut s = 0;
            while let Some(i) = finder.find(&w[s..]) {
                let at = from + s + i;
                if at >= to {
                    break;
                }
                match self.data.get(at + pat.len()) {
                    Some(b'>' | b' ' | b'\t' | b'\r' | b'\n') => {
                        end = Some(at);
                        break;
                    }
                    _ => s += i + 1,
                }
            }
            from = to;
        }
        let Some(end) = end else {
            return Err(self.err(start, format!("<{qname}> is not closed")));
        };
        let has_bang = self.data[start..end]
            .par_chunks(16 << 20)
            .enumerate()
            .any(|(k, c)| {
                let a = start + k * (16 << 20);
                let w = &self.data[a..(a + c.len() + 1).min(end)];
                memmem::find(w, b"<!").is_some()
            });
        ctx.checkpoint()?;
        if !has_bang {
            return Ok((start, end));
        }
        // Slow path: walk events to the matching end tag.
        let mut r = reader(&self.data[start..]);
        let mut depth = 0usize;
        loop {
            let off = start + r.buffer_position() as usize;
            match r.read_event() {
                Ok(Event::Start(_)) => depth += 1,
                Ok(Event::End(_)) if depth == 0 => return Ok((start, off)),
                Ok(Event::End(_)) => depth -= 1,
                Ok(Event::Eof) => {
                    return Err(self.err(start, format!("<{qname}> is not closed")));
                }
                Ok(_) => {}
                Err(e) => {
                    let at = start + r.error_position() as usize;
                    return Err(self.err(at, e.to_string()));
                }
            }
        }
    }
}

/// Parser state for one part.
struct Parse<'a, 'd> {
    x: Xml<'d>,
    strict: bool,
    root: bool,
    tuning: &'a Tuning,
    ctx: &'a JobCtx,
    out: ModelPart,
    /// Running counts against the limits (shared across parts by the caller).
    totals: &'a mut (u64, u64),
    seen_object: bool,
    progress: &'a mut dyn FnMut(usize),
}

/// Parses the model part `name` (part key `key`).
#[allow(clippy::too_many_arguments)]
pub(crate) fn parse(
    data: Vec<u8>,
    key: &str,
    name: &str,
    root: bool,
    strict: bool,
    limits: &ReadLimits,
    tuning: &Tuning,
    ctx: &JobCtx,
    totals: &mut (u64, u64),
    progress: &mut dyn FnMut(usize),
) -> Result<ModelPart, IoError> {
    let result = {
        let x = Xml::new(&data, name, limits);
        let mut p = Parse {
            x,
            strict,
            root,
            tuning,
            ctx,
            out: ModelPart {
                key: key.to_owned(),
                name: name.to_owned(),
                ..ModelPart::default()
            },
            totals,
            seen_object: false,
            progress,
        };
        p.document().map(|()| p.out)
    };
    match result {
        Ok(mut m) => {
            m.data = data;
            Ok(m)
        }
        Err(e) => Err(e),
    }
}

impl Parse<'_, '_> {
    fn err(&self, off: usize, msg: impl Into<String>) -> IoError {
        self.x.err(off, msg)
    }

    fn core(&self, ns: &Option<String>) -> bool {
        ns.as_deref() == Some(NS_CORE)
    }

    /// Unknown element: skipped if foreign, an error in strict mode if core.
    fn unknown(
        &mut self,
        ns: &Option<String>,
        local: &str,
        off: usize,
        empty: bool,
    ) -> Result<(), IoError> {
        if self.core(ns) || ns.is_none() {
            if self.strict {
                return Err(self.err(off, format!("unexpected element <{local}>")));
            }
        } else if let Some(n) = ns {
            const DISPLAY: [&str; 5] = [
                "pbspeculardisplayproperties",
                "pbmetallicdisplayproperties",
                "pbspeculartexturedisplayproperties",
                "pbmetallictexturedisplayproperties",
                "translucentdisplayproperties",
            ];
            if n == NS_MAT && self.strict && !DISPLAY.contains(&local) {
                return Err(self.err(off, format!("unexpected element <{local}>")));
            }
            if !self.out.requires(n) {
                self.out.ignored_ns.insert(n.clone());
            }
            let what = match n.as_str() {
                NS_BEAM => "beam lattice",
                NS_MAT => "material display properties",
                NS_TRISETS => "triangle sets",
                _ => "",
            };
            if !what.is_empty() {
                *self.out.ignored.entry(what.to_owned()).or_default() += 1;
            }
        }
        if !empty {
            self.x.skip()?;
        }
        Ok(())
    }

    /// Strict check that unprefixed attributes are in `allowed`.
    fn only(&self, attrs: &[A], allowed: &[&str], off: usize, elem: &str) -> Result<(), IoError> {
        if !self.strict {
            return Ok(());
        }
        for a in attrs {
            match &a.ns {
                None if !allowed.contains(&a.local.as_str()) => {
                    return Err(self.err(off, format!("<{elem}> has no attribute {}", a.qname)));
                }
                Some(n) if n == NS_XML && elem == "model" && a.local == "lang" => {}
                Some(n) if n == NS_XML => {
                    return Err(self.err(off, format!("<{elem}> has no attribute {}", a.qname)));
                }
                _ => {}
            }
        }
        Ok(())
    }

    fn get<'a>(attrs: &'a [A], local: &str) -> Option<&'a str> {
        attrs
            .iter()
            .find(|a| a.ns.is_none() && a.local == local)
            .map(|a| a.value.as_str())
    }

    fn get_ns<'a>(attrs: &'a [A], ns: &str, local: &str) -> Option<&'a str> {
        attrs
            .iter()
            .find(|a| a.ns.as_deref() == Some(ns) && a.local == local)
            .map(|a| a.value.as_str())
    }

    fn req<'a>(
        &self,
        attrs: &'a [A],
        local: &str,
        off: usize,
        elem: &str,
    ) -> Result<&'a str, IoError> {
        Self::get(attrs, local)
            .ok_or_else(|| self.err(off, format!("<{elem}> needs attribute {local}")))
    }

    fn uuid(&self, attrs: &[A], off: usize) -> Result<Option<String>, IoError> {
        match Self::get_ns(attrs, NS_PROD, "UUID") {
            Some(u) => {
                let u = trim(u);
                if self.strict && !is_uuid(u) {
                    return Err(self.err(off, format!("{u:?} is not a UUID")));
                }
                Ok(Some(u.to_ascii_lowercase()))
            }
            None => Ok(None),
        }
    }

    fn path(&self, attrs: &[A], off: usize) -> Result<Option<String>, IoError> {
        match Self::get_ns(attrs, NS_PROD, "path") {
            Some(p) => {
                let p = trim(p);
                if self.strict && !p.starts_with('/') {
                    return Err(
                        self.err(off, format!("p:path {p:?} must be an absolute part name"))
                    );
                }
                Ok(Some(super::xml::part_key(p)))
            }
            None => Ok(None),
        }
    }

    fn document(&mut self) -> Result<(), IoError> {
        let mut seen_root = false;
        loop {
            match self.x.next()? {
                Ev::Start(_, off) | Ev::Empty(_, off) if seen_root => {
                    return Err(self.err(off, "more than one root element"));
                }
                Ev::Start(e, off) => {
                    seen_root = true;
                    self.model_start(&e, off)?;
                    self.model_body()?;
                }
                Ev::Empty(e, off) => {
                    seen_root = true;
                    self.model_start(&e, off)?;
                }
                Ev::Text(t) => {
                    if !trim(&t).is_empty() {
                        return Err(self.err(self.x.pos(), "text outside the root element"));
                    }
                }
                Ev::End(_) => {}
                Ev::Eof => break,
            }
        }
        if !seen_root {
            return Err(self.err(0, "the model part has no <model> element"));
        }
        if self.root && self.strict && !self.out.has_build {
            return Err(self.err(0, "the model has no build element"));
        }
        Ok(())
    }

    fn model_start(&mut self, e: &BytesStart<'_>, off: usize) -> Result<(), IoError> {
        let (ns, local) = self.x.element_ns(e);
        if !self.core(&ns) || local != "model" {
            return Err(self.err(
                off,
                format!("the root element is <{}>, not a 3MF <model>", e.name().0),
            ));
        }
        let attrs = self.x.attrs(e, off, self.strict)?;
        self.only(
            &attrs,
            &[
                "unit",
                "requiredextensions",
                "recommendedextensions",
                "thumbnail",
            ],
            off,
            "model",
        )?;
        if let Some(u) = Self::get(&attrs, "unit") {
            self.out.unit =
                Some(unit_of(trim(u)).ok_or_else(|| self.err(off, format!("unknown unit {u:?}")))?);
        }
        if self.strict
            && let Some(l) = Self::get_ns(&attrs, NS_XML, "lang")
            && trim(l).is_empty()
        {
            return Err(self.err(off, "xml:lang is empty"));
        }
        if let Some(req) = Self::get(&attrs, "requiredextensions") {
            for p in req.split_ascii_whitespace() {
                let uri = self
                    .x
                    .scopes
                    .uri(p)
                    .ok_or_else(|| {
                        self.err(
                            off,
                            format!("required extension prefix {p:?} is not declared"),
                        )
                    })?
                    .to_owned();
                if uri == NS_CORE {
                    continue;
                }
                if !SUPPORTED_REQUIRED.contains(&uri.as_str()) {
                    return Err(IoError::UnsupportedRequiredExtension { namespace: uri });
                }
                self.out.required.push(uri);
            }
        }
        Ok(())
    }

    fn model_body(&mut self) -> Result<(), IoError> {
        let mut seen_resources = false;
        let mut seen_build = false;
        let mut names: BTreeSet<String> = BTreeSet::new();
        loop {
            match self.x.next()? {
                Ev::Start(e, off) => {
                    let (ns, local) = self.x.element_ns(&e);
                    match (self.core(&ns), local.as_str()) {
                        (true, "metadata") => {
                            let (k, _) = self.metadata_attrs(&e, off, &mut names)?;
                            let v = self.x.text()?;
                            self.out.metadata.push((k, v));
                        }
                        (true, "resources") => {
                            if self.strict && (seen_resources || seen_build) {
                                return Err(self.err(off, "misplaced <resources>"));
                            }
                            seen_resources = true;
                            self.resources()?;
                        }
                        (true, "build") => {
                            if self.strict && seen_build {
                                return Err(self.err(off, "more than one <build>"));
                            }
                            seen_build = true;
                            self.out.has_build = true;
                            let attrs = self.x.attrs(&e, off, self.strict)?;
                            self.only(&attrs, &[], off, "build")?;
                            self.out.build_uuid = self.uuid(&attrs, off)?;
                            self.build()?;
                        }
                        _ => self.unknown(&ns, &local, off, false)?,
                    }
                }
                Ev::Empty(e, off) => {
                    let (ns, local) = self.x.element_ns(&e);
                    match (self.core(&ns), local.as_str()) {
                        (true, "metadata") => {
                            let (k, _) = self.metadata_attrs(&e, off, &mut names)?;
                            self.out.metadata.push((k, String::new()));
                        }
                        (true, "resources") => {
                            seen_resources = true;
                        }
                        (true, "build") => {
                            seen_build = true;
                            self.out.has_build = true;
                            let attrs = self.x.attrs(&e, off, self.strict)?;
                            self.out.build_uuid = self.uuid(&attrs, off)?;
                        }
                        _ => self.unknown(&ns, &local, off, true)?,
                    }
                }
                Ev::Text(t) => {
                    if self.strict && !trim(&t).is_empty() {
                        return Err(self.err(self.x.pos(), "unexpected text in <model>"));
                    }
                }
                Ev::End(_) => {
                    if self.root && self.strict && !seen_resources {
                        return Err(self.err(self.x.pos(), "the model has no <resources>"));
                    }
                    return Ok(());
                }
                Ev::Eof => return Err(self.err(self.x.data.len(), "<model> is not closed")),
            }
        }
    }

    fn metadata_attrs(
        &self,
        e: &BytesStart<'_>,
        off: usize,
        names: &mut BTreeSet<String>,
    ) -> Result<(String, bool), IoError> {
        let attrs = self.x.attrs(e, off, self.strict)?;
        self.only(&attrs, &["name", "preserve", "type"], off, "metadata")?;
        let name = trim(self.req(&attrs, "name", off, "metadata")?).to_owned();
        if self.strict {
            match name.split_once(':') {
                Some((p, _)) => {
                    if self.x.scopes.uri(p).is_none() {
                        return Err(self.err(
                            off,
                            format!("metadata {name:?} uses an undeclared namespace prefix"),
                        ));
                    }
                }
                None => {
                    if !CORE_METADATA.iter().any(|c| c.eq_ignore_ascii_case(&name)) {
                        return Err(self.err(
                            off,
                            format!("metadata {name:?} is not a core name and has no namespace"),
                        ));
                    }
                }
            }
            if !names.insert(name.to_ascii_lowercase()) {
                return Err(self.err(off, format!("duplicate metadata {name:?}")));
            }
            if let Some(p) = Self::get(&attrs, "preserve")
                && !matches!(trim(p), "0" | "1" | "true" | "false")
            {
                return Err(self.err(off, format!("invalid preserve value {p:?}")));
            }
        }
        Ok((name, true))
    }

    // ------------------------------------------------------------------------ resources

    fn add_group(&mut self, id: u32, g: PropGroup, off: usize) -> Result<(), IoError> {
        if self.strict && self.seen_object && !matches!(g, PropGroup::Base(_)) {
            return Err(self.err(
                off,
                format!(
                    "<{}> after an <object> (property resources come first)",
                    g.label()
                ),
            ));
        }
        if (self.out.groups.contains_key(&id) || self.out.objects.contains_key(&id)) && self.strict
        {
            return Err(self.err(off, format!("duplicate resource id {id}")));
        }
        self.out.groups.insert(id, g);
        Ok(())
    }

    fn resources(&mut self) -> Result<(), IoError> {
        loop {
            self.ctx.checkpoint()?;
            match self.x.next()? {
                Ev::Start(e, off) => self.resource(&e, off, false)?,
                Ev::Empty(e, off) => self.resource(&e, off, true)?,
                Ev::Text(t) => {
                    if self.strict && !trim(&t).is_empty() {
                        return Err(self.err(self.x.pos(), "unexpected text in <resources>"));
                    }
                }
                Ev::End(_) => return Ok(()),
                Ev::Eof => return Err(self.err(self.x.data.len(), "<resources> is not closed")),
            }
        }
    }

    fn resource(&mut self, e: &BytesStart<'_>, off: usize, empty: bool) -> Result<(), IoError> {
        let (ns, local) = self.x.element_ns(e);
        let attrs = self.x.attrs(e, off, self.strict)?;
        let id = || -> Result<u32, IoError> {
            parse_id(
                Self::get(&attrs, "id")
                    .ok_or_else(|| self.err(off, format!("<{local}> needs an id")))?,
            )
            .map_err(|m| self.err(off, m))
        };
        match (ns.as_deref(), local.as_str()) {
            (Some(NS_CORE), "basematerials") => {
                self.only(&attrs, &["id", "displaypropertiesid"], off, "basematerials")?;
                let id = id()?;
                let mut v = Vec::new();
                if !empty {
                    self.children(|p, e, off, attrs| {
                        if p.child_is(e, NS_CORE, "base") {
                            p.only(
                                attrs,
                                &["name", "displaycolor", "displaypropertiesid"],
                                off,
                                "base",
                            )?;
                            let name = p.req(attrs, "name", off, "base")?.to_owned();
                            let c = parse_color(p.req(attrs, "displaycolor", off, "base")?)
                                .map_err(|m| p.err(off, m))?;
                            v.push((name, c));
                            Ok(true)
                        } else {
                            Ok(false)
                        }
                    })?;
                }
                self.add_group(id, PropGroup::Base(v), off)
            }
            (Some(NS_MAT), "colorgroup") => {
                self.only(&attrs, &["id", "displaypropertiesid"], off, "colorgroup")?;
                let id = id()?;
                let mut v = Vec::new();
                if !empty {
                    self.children(|p, e, off, attrs| {
                        if p.child_is(e, NS_MAT, "color") {
                            p.only(attrs, &["color"], off, "color")?;
                            let c = parse_color(p.req(attrs, "color", off, "color")?)
                                .map_err(|m| p.err(off, m))?;
                            v.push(c);
                            Ok(true)
                        } else {
                            Ok(false)
                        }
                    })?;
                }
                self.add_group(id, PropGroup::Colors(v), off)
            }
            (Some(NS_MAT), "texture2d") => {
                self.only(
                    &attrs,
                    &[
                        "id",
                        "path",
                        "contenttype",
                        "tilestyleu",
                        "tilestylev",
                        "filter",
                    ],
                    off,
                    "texture2d",
                )?;
                let id = id()?;
                let path = trim(self.req(&attrs, "path", off, "texture2d")?).to_owned();
                let ct = trim(self.req(&attrs, "contenttype", off, "texture2d")?).to_owned();
                if self.strict && ct != "image/png" && ct != "image/jpeg" {
                    return Err(self.err(
                        off,
                        format!("texture content type {ct:?} is not image/png or image/jpeg"),
                    ));
                }
                if !empty {
                    self.x.skip()?;
                }
                self.add_group(
                    id,
                    PropGroup::Texture2d {
                        path: super::xml::part_key(&path),
                    },
                    off,
                )
            }
            (Some(NS_MAT), "texture2dgroup") => {
                self.only(
                    &attrs,
                    &["id", "texid", "displaypropertiesid"],
                    off,
                    "texture2dgroup",
                )?;
                let id = id()?;
                let texid = parse_id(self.req(&attrs, "texid", off, "texture2dgroup")?)
                    .map_err(|m| self.err(off, m))?;
                if self.strict
                    && !matches!(
                        self.out.groups.get(&texid),
                        Some(PropGroup::Texture2d { .. })
                    )
                {
                    return Err(self.err(
                        off,
                        format!("texid {texid} is not a texture2d defined before"),
                    ));
                }
                let mut uvs = Vec::new();
                if !empty {
                    let strict = self.strict;
                    self.children(|p, e, off, attrs| {
                        if p.child_is(e, NS_MAT, "tex2coord") {
                            p.only(attrs, &["u", "v"], off, "tex2coord")?;
                            let u = parse_number(p.req(attrs, "u", off, "tex2coord")?, strict)
                                .map_err(|m| p.err(off, m))?;
                            let v = parse_number(p.req(attrs, "v", off, "tex2coord")?, strict)
                                .map_err(|m| p.err(off, m))?;
                            uvs.push([u, v]);
                            Ok(true)
                        } else {
                            Ok(false)
                        }
                    })?;
                }
                self.add_group(id, PropGroup::TexCoords { uvs }, off)
            }
            (Some(NS_MAT), "compositematerials") => {
                self.only(
                    &attrs,
                    &["id", "matid", "matindices", "displaypropertiesid"],
                    off,
                    "compositematerials",
                )?;
                let id = id()?;
                let matid = parse_id(self.req(&attrs, "matid", off, "compositematerials")?)
                    .map_err(|m| self.err(off, m))?;
                let matindices = self
                    .req(&attrs, "matindices", off, "compositematerials")?
                    .split_ascii_whitespace()
                    .map(parse_index)
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(|m| self.err(off, m))?;
                if self.strict {
                    match self.out.groups.get(&matid) {
                        Some(PropGroup::Base(b)) => {
                            if matindices.iter().any(|&i| i as usize >= b.len()) {
                                return Err(self.err(off, "matindices out of range"));
                            }
                        }
                        _ => {
                            return Err(self.err(
                                off,
                                format!("matid {matid} is not a basematerials defined before"),
                            ));
                        }
                    }
                }
                let n = matindices.len();
                let mut values = Vec::new();
                if !empty {
                    let strict = self.strict;
                    self.children(|p, e, off, attrs| {
                        if p.child_is(e, NS_MAT, "composite") {
                            p.only(attrs, &["values"], off, "composite")?;
                            let v = p
                                .req(attrs, "values", off, "composite")?
                                .split_ascii_whitespace()
                                .map(|s| parse_number(s, strict))
                                .collect::<Result<Vec<_>, _>>()
                                .map_err(|m| p.err(off, m))?;
                            if strict && v.len() != n {
                                return Err(p.err(off, "composite values do not match matindices"));
                            }
                            values.push(v);
                            Ok(true)
                        } else {
                            Ok(false)
                        }
                    })?;
                }
                self.add_group(
                    id,
                    PropGroup::Composite {
                        matid,
                        matindices,
                        values,
                    },
                    off,
                )
            }
            (Some(NS_MAT), "multiproperties") => {
                self.only(
                    &attrs,
                    &["id", "pids", "blendmethods"],
                    off,
                    "multiproperties",
                )?;
                let id = id()?;
                let pids = self
                    .req(&attrs, "pids", off, "multiproperties")?
                    .split_ascii_whitespace()
                    .map(parse_id)
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(|m| self.err(off, m))?;
                if self.strict {
                    self.check_multi_pids(&pids, off)?;
                }
                let mut multis = Vec::new();
                if !empty {
                    self.children(|p, e, off, attrs| {
                        if p.child_is(e, NS_MAT, "multi") {
                            p.only(attrs, &["pindices"], off, "multi")?;
                            let v = p
                                .req(attrs, "pindices", off, "multi")?
                                .split_ascii_whitespace()
                                .map(parse_index)
                                .collect::<Result<Vec<_>, _>>()
                                .map_err(|m| p.err(off, m))?;
                            if p.strict {
                                for (k, &ix) in v.iter().enumerate() {
                                    let g = pids.get(k).and_then(|g| p.out.groups.get(g));
                                    match g {
                                        Some(g) if (ix as usize) < g.len() => {}
                                        // Extra pindices beyond pids are ignored (P_*_0505_03).
                                        _ if k >= pids.len() => break,
                                        _ => {
                                            return Err(
                                                p.err(off, format!("pindex {ix} out of range"))
                                            );
                                        }
                                    }
                                }
                            }
                            multis.push(v);
                            Ok(true)
                        } else {
                            Ok(false)
                        }
                    })?;
                }
                self.add_group(id, PropGroup::Multi { pids, multis }, off)
            }
            (Some(NS_CORE), "object") => {
                self.seen_object = true;
                self.object(e, off, attrs, empty)
            }
            _ => self.unknown(&ns, &local, off, empty),
        }
    }

    fn check_multi_pids(&self, pids: &[u32], off: usize) -> Result<(), IoError> {
        let mut colors = 0;
        for (k, pid) in pids.iter().enumerate() {
            match self.out.groups.get(pid) {
                None => {
                    return Err(
                        self.err(off, format!("pids {pid} is not a resource defined before"))
                    );
                }
                Some(PropGroup::Multi { .. }) => {
                    return Err(self.err(off, "multiproperties cannot reference multiproperties"));
                }
                Some(PropGroup::Base(_) | PropGroup::Composite { .. }) if k > 0 => {
                    return Err(self.err(
                        off,
                        "basematerials/compositematerials must be the first layer",
                    ));
                }
                Some(PropGroup::Colors(_)) => {
                    colors += 1;
                    if colors > 1 {
                        return Err(self.err(off, "at most one colorgroup per multiproperties"));
                    }
                }
                Some(PropGroup::Texture2d { .. }) => {
                    return Err(self.err(off, "pids cannot reference a texture2d"));
                }
                _ => {}
            }
        }
        Ok(())
    }

    fn child_is(&self, e: &BytesStart<'_>, ns: &str, local: &str) -> bool {
        let (n, l) = self.x.scopes.element(e.name().0);
        n == Some(ns) && l == local
    }

    /// Walks the children of the element whose start was just read. `f` returns false for
    /// elements it does not know (they are skipped, or are errors in strict mode if core).
    fn children(
        &mut self,
        mut f: impl FnMut(&mut Self, &BytesStart<'_>, usize, &[A]) -> Result<bool, IoError>,
    ) -> Result<(), IoError> {
        loop {
            self.ctx.checkpoint()?;
            match self.x.next()? {
                Ev::Start(e, off) => {
                    let attrs = self.x.attrs(&e, off, self.strict)?;
                    let known = f(self, &e, off, &attrs)?;
                    if known {
                        // No children are defined for property entries.
                        self.x.skip()?;
                    } else {
                        let (ns, local) = self.x.element_ns(&e);
                        self.unknown(&ns, &local, off, false)?;
                    }
                }
                Ev::Empty(e, off) => {
                    let attrs = self.x.attrs(&e, off, self.strict)?;
                    if !f(self, &e, off, &attrs)? {
                        let (ns, local) = self.x.element_ns(&e);
                        self.unknown(&ns, &local, off, true)?;
                    }
                }
                Ev::Text(t) => {
                    if self.strict && !trim(&t).is_empty() {
                        return Err(self.err(self.x.pos(), "unexpected text"));
                    }
                }
                Ev::End(_) => return Ok(()),
                Ev::Eof => return Err(self.err(self.x.data.len(), "element is not closed")),
            }
        }
    }

    // --------------------------------------------------------------------------- object

    fn object(
        &mut self,
        _e: &BytesStart<'_>,
        off: usize,
        attrs: Vec<A>,
        empty: bool,
    ) -> Result<(), IoError> {
        self.only(
            &attrs,
            &[
                "id",
                "type",
                "thumbnail",
                "partnumber",
                "name",
                "pid",
                "pindex",
            ],
            off,
            "object",
        )?;
        let id = parse_id(self.req(&attrs, "id", off, "object")?).map_err(|m| self.err(off, m))?;
        if self.strict && (self.out.objects.contains_key(&id) || self.out.groups.contains_key(&id))
        {
            return Err(self.err(off, format!("duplicate resource id {id}")));
        }
        let (kind, kind_other) = match Self::get(&attrs, "type") {
            Some(t) => kind_of(trim(t))
                .ok_or_else(|| self.err(off, format!("unknown object type {t:?}")))?,
            None => (ObjectKind::Model, false),
        };
        let pid = Self::get(&attrs, "pid")
            .map(parse_id)
            .transpose()
            .map_err(|m| self.err(off, m))?;
        let pindex = Self::get(&attrs, "pindex")
            .map(parse_index)
            .transpose()
            .map_err(|m| self.err(off, m))?;
        if self.strict {
            if pindex.is_some() && pid.is_none() {
                return Err(self.err(off, "pindex without pid"));
            }
            if let Some(pid) = pid {
                let g = self.out.groups.get(&pid).ok_or_else(|| {
                    self.err(
                        off,
                        format!("pid {pid} is not a property group defined before"),
                    )
                })?;
                if matches!(g, PropGroup::Texture2d { .. }) {
                    return Err(self.err(
                        off,
                        format!("pid {pid} is a texture2d, not a property group"),
                    ));
                }
                let ix = pindex.unwrap_or(0);
                if ix as usize >= g.len() {
                    return Err(
                        self.err(off, format!("pindex {ix} is out of range of group {pid}"))
                    );
                }
            }
        }
        let uuid = self.uuid(&attrs, off)?;
        let name = Self::get(&attrs, "name").map(str::to_owned);
        let mut obj = Object {
            id,
            kind,
            kind_other,
            name,
            pid,
            pindex,
            uuid,
            content: Content::Empty,
            offset: off,
        };
        if !empty {
            self.object_body(&mut obj, off)?;
        }
        if self.strict {
            match &obj.content {
                Content::Empty => {
                    return Err(self.err(
                        off,
                        format!("object {id} has neither a mesh nor components"),
                    ));
                }
                Content::Components(_) if obj.pid.is_some() => {
                    return Err(self.err(off, format!("object {id} has components and a pid")));
                }
                _ => {}
            }
        }
        self.out.objects.insert(id, obj);
        Ok(())
    }

    fn object_body(&mut self, obj: &mut Object, obj_off: usize) -> Result<(), IoError> {
        loop {
            match self.x.next()? {
                Ev::Start(e, off) => {
                    let (ns, local) = self.x.element_ns(&e);
                    match (self.core(&ns), local.as_str()) {
                        (true, "mesh") => {
                            self.content_once(obj, off)?;
                            let m = self.mesh(obj)?;
                            obj.content = Content::Mesh(m);
                        }
                        (true, "components") => {
                            self.content_once(obj, off)?;
                            let c = self.components()?;
                            obj.content = Content::Components(c);
                        }
                        (true, "metadatagroup") => self.x.skip()?,
                        _ => self.unknown(&ns, &local, off, false)?,
                    }
                }
                Ev::Empty(e, off) => {
                    let (ns, local) = self.x.element_ns(&e);
                    match (self.core(&ns), local.as_str()) {
                        (true, "mesh") => {
                            self.content_once(obj, off)?;
                            if self.strict {
                                return Err(self.err(off, "the mesh has no vertices"));
                            }
                            obj.content = Content::Mesh(MeshData::default());
                        }
                        (true, "components") => {
                            self.content_once(obj, off)?;
                            if self.strict {
                                return Err(self.err(off, "<components> is empty"));
                            }
                            obj.content = Content::Components(Vec::new());
                        }
                        (true, "metadatagroup") => {}
                        _ => self.unknown(&ns, &local, off, true)?,
                    }
                }
                Ev::Text(t) => {
                    if self.strict && !trim(&t).is_empty() {
                        return Err(self.err(self.x.pos(), "unexpected text in <object>"));
                    }
                }
                Ev::End(_) => return Ok(()),
                Ev::Eof => return Err(self.err(obj_off, "<object> is not closed")),
            }
        }
    }

    fn content_once(&self, obj: &Object, off: usize) -> Result<(), IoError> {
        if !matches!(obj.content, Content::Empty) {
            return Err(self.err(
                off,
                format!("object {} has more than one mesh/components", obj.id),
            ));
        }
        Ok(())
    }

    fn mesh(&mut self, obj: &Object) -> Result<MeshData, IoError> {
        let mut m = MeshData::default();
        let mut seen_v = false;
        let mut seen_t = false;
        loop {
            self.ctx.checkpoint()?;
            match self.x.next()? {
                Ev::Start(e, off) => {
                    let (ns, local) = self.x.element_ns(&e);
                    match (self.core(&ns), local.as_str()) {
                        (true, "vertices") if !seen_v => {
                            seen_v = true;
                            m.positions = self.range(&e, off, scan::Kind::Vertices)?.0;
                        }
                        (true, "triangles") if !seen_t => {
                            seen_t = true;
                            m.tris = self.range(&e, off, scan::Kind::Triangles)?.1;
                        }
                        _ => self.unknown(&ns, &local, off, false)?,
                    }
                }
                Ev::Empty(e, off) => {
                    let (ns, local) = self.x.element_ns(&e);
                    match (self.core(&ns), local.as_str()) {
                        (true, "vertices") => seen_v = true,
                        (true, "triangles") => seen_t = true,
                        _ => self.unknown(&ns, &local, off, true)?,
                    }
                }
                Ev::Text(t) => {
                    if self.strict && !trim(&t).is_empty() {
                        return Err(self.err(self.x.pos(), "unexpected text in <mesh>"));
                    }
                }
                Ev::End(off) => {
                    if self.strict && (!seen_v || !seen_t) {
                        return Err(self.err(
                            off,
                            format!("the mesh of object {} needs vertices and triangles", obj.id),
                        ));
                    }
                    return Ok(m);
                }
                Ev::Eof => return Err(self.err(obj.offset, "<mesh> is not closed")),
            }
        }
    }

    /// Hands the content of `<vertices>` / `<triangles>` to the scanner and resumes after it.
    fn range(
        &mut self,
        e: &BytesStart<'_>,
        off: usize,
        kind: scan::Kind,
    ) -> Result<(Vec<Point3>, Triangles), IoError> {
        let attrs = self.x.attrs(e, off, self.strict)?;
        self.only(
            &attrs,
            &[],
            off,
            if kind == scan::Kind::Vertices {
                "vertices"
            } else {
                "triangles"
            },
        )?;
        let qname = e.name().0.to_owned();
        let (start, end) = self.x.content_range(&qname, self.ctx)?;
        let range = &self.x.data[start..end];
        let prefix = qname.split_once(':').map(|(p, _)| p);
        let child_local = if kind == scan::Kind::Vertices {
            "vertex"
        } else {
            "triangle"
        };
        let child_qname = match prefix {
            Some(p) => format!("{p}:{child_local}"),
            None => child_local.to_owned(),
        };
        // Limits before allocating (the allocation rule): an upper bound by element count.
        let count = scan::count_elements(range, &child_qname);
        let lim = self.x.limits;
        match kind {
            scan::Kind::Vertices => {
                let after = self.totals.0 + count;
                if after > lim.max_vertices {
                    return Err(IoError::LimitExceeded {
                        what: "vertices",
                        limit: lim.max_vertices,
                        actual: after,
                    });
                }
            }
            scan::Kind::Triangles => {
                let after = self.totals.1 + count;
                if after > lim.max_triangles {
                    return Err(IoError::LimitExceeded {
                        what: "triangles",
                        limit: lim.max_triangles,
                        actual: after,
                    });
                }
            }
        }
        self.ctx.checkpoint()?;
        let cx = ScanCtx {
            strict: self.strict,
            limits: lim,
            scopes: &self.x.scopes,
            child_qname,
            slic3r_prefixes: self.x.scopes.prefixes_of(NS_SLIC3R),
            ctx: self.ctx,
        };
        let located = |r: (usize, String)| malformed_at(self.x.part, self.x.data, start + r.0, r.1);
        let out = match kind {
            scan::Kind::Vertices => {
                let v = scan::vertices(range, &cx, self.tuning);
                self.ctx.checkpoint()?;
                let v = v.map_err(located)?;
                self.totals.0 += v.len() as u64;
                (v, Triangles::default())
            }
            scan::Kind::Triangles => {
                let t = scan::triangles(range, &cx, self.tuning);
                self.ctx.checkpoint()?;
                let t = t.map_err(located)?;
                self.totals.1 += t.tris.len() as u64;
                (Vec::new(), t)
            }
        };
        (self.progress)(end);
        self.x.reposition(end);
        // The end tag of the range element comes next.
        match self.x.next()? {
            Ev::End(_) => Ok(out),
            _ => Err(self.err(end, "expected the end tag")),
        }
    }

    fn components(&mut self) -> Result<Vec<Component>, IoError> {
        let mut out = Vec::new();
        let strict = self.strict;
        self.children(|p, e, off, attrs| {
            if !p.child_is(e, NS_CORE, "component") {
                return Ok(false);
            }
            p.only(attrs, &["objectid", "transform"], off, "component")?;
            let objectid =
                parse_id(p.req(attrs, "objectid", off, "component")?).map_err(|m| p.err(off, m))?;
            let transform = match Self::get(attrs, "transform") {
                Some(t) => parse_transform(t, strict).map_err(|m| p.err(off, m))?,
                None => Transform::IDENTITY,
            };
            let path = p.path(attrs, off)?;
            if strict && path.is_some() && !p.root {
                return Err(p.err(off, "p:path is only allowed in the root model part"));
            }
            if strict && path.is_none() && !p.out.objects.contains_key(&objectid) {
                return Err(p.err(
                    off,
                    format!("component refers to object {objectid}, which is not defined before"),
                ));
            }
            let uuid = p.uuid(attrs, off)?;
            out.push(Component {
                objectid,
                transform,
                path,
                uuid,
                offset: off,
            });
            Ok(true)
        })?;
        if strict && out.is_empty() {
            return Err(self.err(self.x.pos(), "<components> is empty"));
        }
        Ok(out)
    }

    fn build(&mut self) -> Result<(), IoError> {
        let strict = self.strict;
        let mut items = Vec::new();
        self.children(|p, e, off, attrs| {
            if !p.child_is(e, NS_CORE, "item") {
                return Ok(false);
            }
            p.only(attrs, &["objectid", "transform", "partnumber"], off, "item")?;
            let objectid =
                parse_id(p.req(attrs, "objectid", off, "item")?).map_err(|m| p.err(off, m))?;
            let transform = match Self::get(attrs, "transform") {
                Some(t) => parse_transform(t, strict).map_err(|m| p.err(off, m))?,
                None => Transform::IDENTITY,
            };
            let path = p.path(attrs, off)?;
            let uuid = p.uuid(attrs, off)?;
            items.push(Item {
                objectid,
                transform,
                path,
                uuid,
                offset: off,
            });
            Ok(true)
        })?;
        self.out.build = items;
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact values are written and read back")]
mod tests {
    use super::*;

    fn parse_str(s: &str, strict: bool) -> Result<ModelPart, IoError> {
        let mut totals = (0, 0);
        parse(
            s.as_bytes().to_vec(),
            "/3d/3dmodel.model",
            "3D/3dmodel.model",
            true,
            strict,
            &ReadLimits::default(),
            &Tuning::default(),
            &JobCtx::detached(),
            &mut totals,
            &mut |_| {},
        )
    }

    const CUBE_TRIS: &str = r#"<triangle v1="0" v2="2" v3="1"/><triangle v1="0" v2="3" v3="2"/>
      <triangle v1="4" v2="5" v3="6"/><triangle v1="4" v2="6" v3="7"/>
      <triangle v1="0" v2="1" v3="5"/><triangle v1="0" v2="5" v3="4"/>
      <triangle v1="1" v2="2" v3="6"/><triangle v1="1" v2="6" v3="5"/>
      <triangle v1="2" v2="3" v3="7"/><triangle v1="2" v2="7" v3="6"/>
      <triangle v1="3" v2="0" v3="4"/><triangle v1="3" v2="4" v3="7"/>"#;

    fn cube(extra_model_attrs: &str, body: &str) -> String {
        format!(
            r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" {extra_model_attrs}>
  <metadata name="Title">A &amp; B</metadata>
  <resources>
    <object id="1" type="model" name="cube">
      <mesh>
        <vertices>
          <vertex x="0" y="0" z="0"/><vertex x="10" y="0" z="0"/><vertex x="10" y="10" z="0"/><vertex x="0" y="10" z="0"/>
          <vertex x="0" y="0" z="10"/><vertex x="10" y="0" z="10"/><vertex x="10" y="10" z="10"/><vertex x="0" y="10" z="10"/>
        </vertices>
        <triangles>{CUBE_TRIS}</triangles>
      </mesh>
    </object>
    {body}
  </resources>
  <build><item objectid="1" transform="1 0 0 0 1 0 0 0 1 5 0 0"/></build>
</model>"#
        )
    }

    #[test]
    fn parses_a_cube() {
        let m = parse_str(&cube("", ""), true).expect("parse");
        assert_eq!(m.unit, Some(LengthUnit::Millimetre));
        assert_eq!(m.metadata, vec![("Title".into(), "A & B".into())]);
        let o = &m.objects[&1];
        let Content::Mesh(mesh) = &o.content else {
            panic!("mesh")
        };
        assert_eq!(mesh.positions.len(), 8);
        assert_eq!(mesh.tris.tris.len(), 12);
        assert_eq!(m.build.len(), 1);
        assert_eq!(m.build[0].transform.translation.x, 5.0);
    }

    #[test]
    fn required_extensions() {
        let s = cube(r#"xmlns:q="urn:mock" requiredextensions="q""#, "");
        assert!(matches!(
            parse_str(&s, false),
            Err(IoError::UnsupportedRequiredExtension { namespace }) if namespace == "urn:mock"
        ));
        let s = cube(r#"requiredextensions="zz""#, "");
        assert!(matches!(
            parse_str(&s, false),
            Err(IoError::Malformed { .. })
        ));
        let s = cube(
            r#"xmlns:m="http://schemas.microsoft.com/3dmanufacturing/material/2015/02" requiredextensions="m""#,
            "",
        );
        assert!(parse_str(&s, true).is_ok());
    }

    #[test]
    fn foreign_content_is_skipped_and_core_unknowns_fail_strict() {
        let s = cube(
            r#"xmlns:v="urn:vendor""#,
            r#"<v:thing a="1"><v:sub/></v:thing>"#,
        );
        let m = parse_str(&s, true).expect("parse");
        assert!(m.ignored_ns.contains("urn:vendor"));
        let s = cube("", "<bogus/>");
        assert!(parse_str(&s, true).is_err());
        assert!(parse_str(&s, false).is_ok());
    }

    #[test]
    fn comments_inside_vertices() {
        let s = cube("", "").replace(
            "<vertices>",
            "<vertices><!-- </vertices> is not the end -->",
        );
        let m = parse_str(&s, true).expect("parse");
        let Content::Mesh(mesh) = &m.objects[&1].content else {
            panic!()
        };
        assert_eq!(mesh.positions.len(), 8);
    }

    #[test]
    fn deep_xml_hits_the_limit() {
        let deep = format!("{}{}", "<v:a>".repeat(200), "</v:a>".repeat(200));
        let s = cube(r#"xmlns:v="urn:v""#, &deep);
        assert!(matches!(
            parse_str(&s, false),
            Err(IoError::LimitExceeded {
                what: "XML depth",
                ..
            })
        ));
    }

    #[test]
    fn errors_have_lines() {
        let s = cube("", "").replace(
            r#"<vertex x="10" y="0" z="0"/>"#,
            r#"<vertex x="1,0" y="0" z="0"/>"#,
        );
        match parse_str(&s, true) {
            Err(IoError::Malformed {
                location: crate::Location::Xml { line, .. },
                ..
            }) => assert_eq!(line, 8),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn uuids() {
        assert!(is_uuid("1b2c3d4e-0000-1111-2222-333344445555"));
        assert!(!is_uuid("1b2c3d4e-0000-1111-2222-33334444555"));
        assert!(!is_uuid("x"));
    }
}
