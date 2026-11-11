//! The OPC package of a 3MF file (core §2, ECMA-376 part 2): the zip container with its
//! hostile-input limits, `[Content_Types].xml` and the relationship parts.

use std::collections::HashMap;
use std::io::{Cursor, Read, Seek};

use amg_jobs::JobCtx;
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use zip::ZipArchive;

use super::xml::{
    Scopes, attr_value, is_xmlns, malformed_at, malformed_part, part_key, resolve_target, trim,
};
use crate::endpoint::Source;
use crate::error::IoError;
use crate::options::ReadLimits;
use crate::zipdir;

/// Relationship type of the StartPart (root model).
pub(crate) const REL_MODEL: &str = "http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel";
/// Prefix of the OPC package metadata relationship types.
const OPC_METADATA_REL: &str =
    "http://schemas.openxmlformats.org/package/2006/relationships/metadata/";
/// Package or object thumbnail.
pub(crate) const REL_THUMBNAIL: &str =
    "http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail";
/// Texture used by a model part.
pub(crate) const REL_TEXTURE: &str =
    "http://schemas.microsoft.com/3dmanufacturing/2013/01/3dtexture";
/// Model part content type.
pub(crate) const CT_MODEL: &str = "application/vnd.ms-package.3dmanufacturing-3dmodel+xml";
/// Relationships content type.
pub(crate) const CT_RELS: &str = "application/vnd.openxmlformats-package.relationships+xml";
/// Content types of the Secure Content extension.
const CT_KEYSTORE: &str = "application/vnd.ms-package.3dmanufacturing-keystore+xml";

/// Entries above this size are subject to the compression-ratio limit (note §9.1).
const RATIO_FLOOR: u64 = 64 << 20;
/// Read granularity: the cancel check runs between reads.
const READ_CHUNK: usize = 1 << 20;
/// Deflate cannot expand more than about 1032:1; used to bound up-front allocations.
const DEFLATE_MAX_RATIO: u64 = 1100;

trait ReadSeek: Read + Seek + Send {}
impl<T: Read + Seek + Send> ReadSeek for T {}

/// One `<Relationship>`.
#[derive(Clone, Debug)]
pub(crate) struct Rel {
    pub id: String,
    pub kind: String,
    /// Resolved part key (see [`part_key`]); empty for external targets.
    pub target: String,
    /// Resolved part name with its case as written (for the strict case check).
    pub exact: String,
    pub external: bool,
}

/// `[Content_Types].xml`.
#[derive(Default, Debug)]
pub(crate) struct ContentTypes {
    defaults: HashMap<String, String>,
    overrides: HashMap<String, String>,
}

impl ContentTypes {
    /// Content type of the part `key`.
    pub(crate) fn of(&self, key: &str) -> Option<&str> {
        if let Some(c) = self.overrides.get(key) {
            return Some(c);
        }
        let name = key.rsplit('/').next().unwrap_or(key);
        let ext = name.rsplit_once('.').map(|(_, e)| e)?;
        self.defaults
            .get(&ext.to_ascii_lowercase())
            .map(String::as_str)
    }

    fn mentions_keystore(&self) -> bool {
        self.defaults
            .values()
            .chain(self.overrides.values())
            .any(|c| c.eq_ignore_ascii_case(CT_KEYSTORE))
    }
}

/// An open 3MF package.
pub(crate) struct Package<'a> {
    zip: ZipArchive<Box<dyn ReadSeek + 'a>>,
    /// Part key → zip index.
    index: HashMap<String, usize>,
    /// Zip entry names, by zip index.
    names: Vec<String>,
    limits: ReadLimits,
    used: u64,
    strict: bool,
    pub content_types: ContentTypes,
    pub root_rels: Vec<Rel>,
    /// Total compressed bytes, for progress.
    pub compressed_total: u64,
}

fn zip_error(what: &str, e: zip::result::ZipError) -> IoError {
    match e {
        zip::result::ZipError::Io(io) if io.kind() == std::io::ErrorKind::OutOfMemory => {
            IoError::LimitExceeded {
                what: "memory",
                limit: 0,
                actual: 0,
            }
        }
        e => malformed_part(what, format!("zip: {e}")),
    }
}

fn limit(what: &'static str, limit: u64, actual: u64) -> IoError {
    IoError::LimitExceeded {
        what,
        limit,
        actual,
    }
}

/// A part name segment that OPC forbids (ECMA-376-2 M1.9, M1.10): empty, `.`, or ending in `.`.
fn bad_segment(seg: &str) -> bool {
    seg.is_empty() || seg.ends_with('.') || (seg.starts_with('.') && seg != ".rels")
}

impl<'a> Package<'a> {
    /// Opens the zip and reads the content types and the root relationships.
    pub(crate) fn open(
        src: &Source<'a>,
        limits: &ReadLimits,
        strict: bool,
    ) -> Result<Self, IoError> {
        // Cheap entry-count limit before the zip crate reads the whole central directory.
        if zipdir::entry_names(src, limits)?.is_none() {
            return Err(malformed_part("", "not a readable zip archive"));
        }
        let reader: Box<dyn ReadSeek + 'a> = match src {
            Source::Bytes(b) => Box::new(Cursor::new(*b)),
            Source::Path(p) => {
                let f = std::fs::File::open(p).map_err(|source| IoError::Read {
                    path: (*p).to_owned(),
                    source,
                })?;
                Box::new(std::io::BufReader::with_capacity(1 << 16, f))
            }
        };
        // A 3MF starts at byte 0. `Detect` would scan the whole file for central-directory
        // signatures (0.6 s for a 200 MB package, with no cancel check).
        let config = zip::read::Config {
            archive_offset: zip::read::ArchiveOffset::Known(0),
        };
        let zip = ZipArchive::with_config(config, reader).map_err(|e| zip_error("", e))?;
        let n = zip.len();
        if n as u64 > u64::from(limits.max_zip_entries) {
            return Err(limit(
                "zip entries",
                u64::from(limits.max_zip_entries),
                n as u64,
            ));
        }
        let mut index = HashMap::with_capacity(n);
        let mut names = Vec::with_capacity(n);
        let mut compressed_total = 0u64;
        for i in 0..n {
            let e = zip.by_index_data(i).map_err(|e| zip_error("", e))?;
            let name = e.name().map_err(|e| zip_error("", e))?.into_owned();
            compressed_total = compressed_total.saturating_add(e.compressed_size());
            names.push(name.clone());
            if name.ends_with('/') || name.ends_with('\\') {
                continue;
            }
            if strict {
                if !name.is_ascii() {
                    return Err(malformed_part(
                        &name,
                        "part names must be ASCII (non-ASCII characters percent-encoded)",
                    ));
                }
                if name.split('/').any(bad_segment) {
                    return Err(malformed_part(&name, "invalid part name segment"));
                }
            }
            if index.insert(part_key(&name), i).is_some() {
                return Err(malformed_part(
                    &name,
                    "two zip entries have the same part name (part names are case-insensitive)",
                ));
            }
        }
        let mut pkg = Package {
            zip,
            index,
            names,
            limits: limits.clone(),
            used: 0,
            strict,
            content_types: ContentTypes::default(),
            root_rels: Vec::new(),
            compressed_total,
        };
        let ct_key = part_key("[Content_Types].xml");
        if !pkg.index.contains_key(&ct_key) {
            return Err(malformed_part(
                "[Content_Types].xml",
                "the package has no [Content_Types].xml",
            ));
        }
        let ct = pkg.read_raw(&ct_key, &JobCtx::detached())?;
        pkg.content_types = parse_content_types(&ct, limits, strict)?;
        let rels_key = part_key("_rels/.rels");
        if !pkg.index.contains_key(&rels_key) {
            return Err(malformed_part(
                "_rels/.rels",
                "the package has no root relationships",
            ));
        }
        pkg.root_rels = pkg.rels_of("/")?.unwrap_or_default();
        if pkg.is_encrypted() {
            return Err(IoError::Encrypted);
        }
        if strict {
            pkg.strict_package_checks()?;
        }
        Ok(pkg)
    }

    /// True if the package uses the Secure Content extension (or zip-level encryption).
    fn is_encrypted(&mut self) -> bool {
        if self.content_types.mentions_keystore() {
            return true;
        }
        if self
            .root_rels
            .iter()
            .any(|r| r.kind.ends_with("/keystore") || r.kind.ends_with("/encryptedfile"))
        {
            return true;
        }
        (0..self.names.len()).any(|i| self.zip.by_index_data(i).is_ok_and(|e| e.encrypted()))
    }

    /// Package-wide MUSTs of OPC that do not depend on the model.
    fn strict_package_checks(&mut self) -> Result<(), IoError> {
        let keys: Vec<(String, usize)> = self.index.iter().map(|(k, &i)| (k.clone(), i)).collect();
        for (key, i) in &keys {
            let name = &self.names[*i];
            if key == "/[content_types].xml" {
                continue;
            }
            // Custom parts may lack a content type (P_*_0336); 3MF parts are checked where
            // they are used. Relationship parts must have theirs.
            if key.ends_with(".rels")
                && self
                    .content_types
                    .of(key)
                    .is_some_and(|ct| !ct.eq_ignore_ascii_case(CT_RELS))
            {
                return Err(malformed_part(
                    name,
                    "relationship part has the wrong content type",
                ));
            }
            // A relationship part needs an existing source part.
            if let Some(src) = rels_source(key)
                && src != "/"
                && !self.index.contains_key(&src)
            {
                return Err(malformed_part(
                    name,
                    "relationship part does not belong to any part",
                ));
            }
        }
        Ok(())
    }

    /// Zip entry name of a part key, for messages.
    pub(crate) fn display_name(&self, key: &str) -> String {
        self.index.get(key).map_or_else(
            || key.trim_start_matches('/').to_owned(),
            |&i| self.names[i].clone(),
        )
    }

    /// True if the part exists.
    pub(crate) fn has(&self, key: &str) -> bool {
        self.index.contains_key(key)
    }

    /// Exact zip entry name of a part key, if present.
    pub(crate) fn entry_name(&self, key: &str) -> Option<&str> {
        self.index.get(key).map(|&i| self.names[i].as_str())
    }

    /// Compressed size of a part (0 if absent).
    pub(crate) fn compressed_size(&self, key: &str) -> u64 {
        self.index
            .get(key)
            .and_then(|&i| self.zip.by_index_data(i).ok().map(|e| e.compressed_size()))
            .unwrap_or(0)
    }

    /// Declared uncompressed size of a part (0 if absent).
    pub(crate) fn uncompressed_size(&self, key: &str) -> u64 {
        self.index
            .get(key)
            .and_then(|&i| self.zip.by_index_data(i).ok().map(|e| e.size()))
            .unwrap_or(0)
    }

    /// Reads a whole part, enforcing the size, total and ratio limits and checking `ctx`
    /// between chunks. `on_bytes` gets the compressed bytes consumed so far (progress).
    pub(crate) fn read_part(
        &mut self,
        key: &str,
        ctx: &JobCtx,
        on_bytes: &mut dyn FnMut(u64),
    ) -> Result<Vec<u8>, IoError> {
        let &i = self
            .index
            .get(key)
            .ok_or_else(|| malformed_part(key, "the part does not exist in the package"))?;
        let name = self.names[i].clone();
        let (size, csize, encrypted) = {
            let e = self.zip.by_index_data(i).map_err(|e| zip_error(&name, e))?;
            (e.size(), e.compressed_size(), e.encrypted())
        };
        if encrypted {
            return Err(IoError::Encrypted);
        }
        let lim = &self.limits;
        if size > lim.max_entry_uncompressed {
            return Err(limit("zip entry size", lim.max_entry_uncompressed, size));
        }
        let remaining = lim.max_total_uncompressed.saturating_sub(self.used);
        if size > remaining {
            return Err(limit(
                "total uncompressed size",
                lim.max_total_uncompressed,
                self.used.saturating_add(size),
            ));
        }
        let ratio = u64::from(lim.max_compression_ratio);
        if size > RATIO_FLOOR && size > csize.saturating_mul(ratio) {
            return Err(limit("compression ratio", ratio, size / csize.max(1)));
        }
        // Hard cap while reading: whatever the headers claim.
        let mut cap = lim.max_entry_uncompressed.min(remaining);
        if csize.saturating_mul(ratio) < cap {
            cap = cap.min(RATIO_FLOOR.max(csize.saturating_mul(ratio)));
        }
        let mut f = self.zip.by_index(i).map_err(|e| zip_error(&name, e))?;
        let reserve = size.min(cap).min(
            csize
                .saturating_mul(DEFLATE_MAX_RATIO)
                .saturating_add(1 << 16),
        );
        let mut out: Vec<u8> = Vec::new();
        out.try_reserve_exact(usize::try_from(reserve).unwrap_or(usize::MAX))
            .map_err(|_| limit("memory", reserve, reserve))?;
        let mut chunk = vec![0u8; READ_CHUNK];
        loop {
            ctx.checkpoint()?;
            let got = match f.read(&mut chunk) {
                Ok(0) => break,
                Ok(n) => n,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(malformed_part(&name, format!("zip entry: {e}"))),
            };
            if out.len() as u64 + got as u64 > cap {
                let what = if cap == lim.max_entry_uncompressed {
                    "zip entry size"
                } else if cap < remaining {
                    "compression ratio"
                } else {
                    "total uncompressed size"
                };
                return Err(limit(what, cap, out.len() as u64 + got as u64));
            }
            out.extend_from_slice(&chunk[..got]);
            if size > 0 {
                on_bytes((csize as f64 * (out.len() as f64 / size as f64)) as u64);
            }
        }
        self.used += out.len() as u64;
        Ok(out)
    }

    fn read_raw(&mut self, key: &str, ctx: &JobCtx) -> Result<Vec<u8>, IoError> {
        self.read_part(key, ctx, &mut |_| {})
    }

    /// The relationships of part `source` (`/` = package), `None` if it has no rels part.
    pub(crate) fn rels_of(&mut self, source: &str) -> Result<Option<Vec<Rel>>, IoError> {
        let rels_key = rels_key_for(source);
        if !self.index.contains_key(&rels_key) {
            return Ok(None);
        }
        let name = self.display_name(&rels_key);
        let data = self.read_raw(&rels_key, &JobCtx::detached())?;
        let rels = parse_rels(&data, &name, source, &self.limits, self.strict)?;
        if self.strict {
            for r in &rels {
                if r.external {
                    continue;
                }
                if !self.index.contains_key(&r.target) {
                    // Missing targets of unknown relationship types are not ours to judge,
                    // but the 3MF ones must resolve.
                    if r.kind == REL_MODEL || r.kind == REL_THUMBNAIL || r.kind == REL_TEXTURE {
                        return Err(malformed_part(
                            &name,
                            format!(
                                "relationship {} points to a missing part {}",
                                r.id, r.target
                            ),
                        ));
                    }
                }
            }
        }
        Ok(Some(rels))
    }
}

/// `/a/b/_rels/c.rels` → `/a/b/c`; `/_rels/.rels` → `/`.
fn rels_source(key: &str) -> Option<String> {
    let file = key.strip_suffix(".rels")?;
    let (dir, name) = file.rsplit_once('/')?;
    let dir = dir.strip_suffix("/_rels")?;
    if name.is_empty() {
        return dir.is_empty().then(|| "/".to_owned());
    }
    Some(format!("{dir}/{name}"))
}

/// The rels part key for `source`.
pub(crate) fn rels_key_for(source: &str) -> String {
    if source == "/" {
        return part_key("_rels/.rels");
    }
    let (dir, name) = source.rsplit_once('/').unwrap_or(("", source));
    part_key(&format!("{dir}/_rels/{name}.rels"))
}

/// Minimal element walker for the small package XML parts: calls `f(depth, element)` for
/// every start/empty element, with the namespace scopes, and enforces the depth and
/// attribute limits.
pub(crate) fn walk<F>(data: &[u8], part: &str, limits: &ReadLimits, mut f: F) -> Result<(), IoError>
where
    F: FnMut(usize, &Scopes, &BytesStart<'_>) -> Result<(), String>,
{
    let mut r = Reader::from_reader(data);
    let cfg = r.config_mut();
    cfg.check_end_names = true;
    cfg.expand_empty_elements = false;
    let mut scopes = Scopes::new();
    let mut depth = 0usize;
    let err =
        |r: &Reader<&[u8]>, m: String| malformed_at(part, data, r.buffer_position() as usize, m);
    loop {
        let ev = r
            .read_event()
            .map_err(|e| malformed_at(part, data, r.error_position() as usize, e.to_string()))?;
        match ev {
            Event::Start(e) => {
                depth += 1;
                if depth > limits.max_xml_depth as usize {
                    return Err(limit(
                        "XML depth",
                        u64::from(limits.max_xml_depth),
                        depth as u64,
                    ));
                }
                scopes.push(&e);
                check_attrs(&e, limits).map_err(|m| err(&r, m))?;
                f(depth, &scopes, &e).map_err(|m| err(&r, m))?;
            }
            Event::Empty(e) => {
                if depth + 1 > limits.max_xml_depth as usize {
                    return Err(limit(
                        "XML depth",
                        u64::from(limits.max_xml_depth),
                        depth as u64 + 1,
                    ));
                }
                scopes.push(&e);
                check_attrs(&e, limits).map_err(|m| err(&r, m))?;
                f(depth + 1, &scopes, &e).map_err(|m| err(&r, m))?;
                scopes.pop();
            }
            Event::End(_) => {
                depth = depth.saturating_sub(1);
                scopes.pop();
            }
            Event::DocType(_) => return Err(err(&r, "DTDs are not allowed".into())),
            Event::Eof => break,
            _ => {}
        }
    }
    if depth != 0 {
        return Err(malformed_at(
            part,
            data,
            data.len(),
            "unexpected end of XML",
        ));
    }
    Ok(())
}

/// Duplicate attributes, over-long values and malformed attribute syntax.
pub(crate) fn check_attrs(e: &BytesStart<'_>, limits: &ReadLimits) -> Result<(), String> {
    for a in e.attributes().with_checks(true) {
        let a = a.map_err(|x| x.to_string())?;
        if !is_xmlns(a.key.0) {
            attr_value(&a, limits.max_attr_value_len)?;
        }
    }
    Ok(())
}

fn get_attr(e: &BytesStart<'_>, name: &str, limits: &ReadLimits) -> Result<Option<String>, String> {
    for a in e.attributes().with_checks(false).flatten() {
        if a.key.0 == name {
            return Ok(Some(
                attr_value(&a, limits.max_attr_value_len)?.into_owned(),
            ));
        }
    }
    Ok(None)
}

/// Is `ct` a syntactically plain `type/subtype` (no parameters)?
fn plain_media_type(ct: &str) -> bool {
    let Some((t, s)) = ct.split_once('/') else {
        return false;
    };
    let tok = |x: &str| {
        !x.is_empty()
            && x.bytes()
                .all(|b| b.is_ascii_alphanumeric() || b"!#$&-^_.+".contains(&b))
    };
    tok(t) && tok(s)
}

fn parse_content_types(
    data: &[u8],
    limits: &ReadLimits,
    strict: bool,
) -> Result<ContentTypes, IoError> {
    let part = "[Content_Types].xml";
    let mut ct = ContentTypes::default();
    walk(data, part, limits, |depth, _s, e| {
        let local = e.local_name();
        let local = local.as_ref();
        if depth == 1 {
            if local != "Types" {
                return Err(format!("unexpected root element <{local}>"));
            }
            return Ok(());
        }
        if depth != 2 {
            return Ok(());
        }
        let content = get_attr(e, "ContentType", limits)?.unwrap_or_default();
        let content = trim(&content).to_owned();
        if strict && !plain_media_type(&content) {
            return Err(format!("invalid content type {content:?}"));
        }
        match local {
            "Default" => {
                let ext = get_attr(e, "Extension", limits)?.unwrap_or_default();
                if strict && ext.is_empty() {
                    return Err("a Default content type has an empty Extension".into());
                }
                let key = ext.to_ascii_lowercase();
                if ct.defaults.insert(key, content).is_some() && strict {
                    return Err(format!("duplicate Default content type for {ext:?}"));
                }
            }
            "Override" => {
                let name = get_attr(e, "PartName", limits)?.unwrap_or_default();
                if strict && trim(&name).is_empty() {
                    return Err("an Override has an empty PartName".into());
                }
                if ct.overrides.insert(part_key(&name), content).is_some() && strict {
                    return Err(format!("duplicate Override content type for {name:?}"));
                }
            }
            other if strict => return Err(format!("unexpected element <{other}>")),
            _ => {}
        }
        Ok(())
    })?;
    Ok(ct)
}

/// An `xsd:ID` (NCName): starts with a letter or `_`, then letters, digits, `.`, `-`, `_`.
fn is_ncname(s: &str) -> bool {
    let mut it = s.chars();
    match it.next() {
        Some(c) if c.is_alphabetic() || c == '_' => {}
        _ => return false,
    }
    it.all(|c| c.is_alphanumeric() || matches!(c, '.' | '-' | '_'))
}

fn parse_rels(
    data: &[u8],
    part: &str,
    source: &str,
    limits: &ReadLimits,
    strict: bool,
) -> Result<Vec<Rel>, IoError> {
    let mut rels: Vec<Rel> = Vec::new();
    walk(data, part, limits, |depth, _s, e| {
        let local = e.local_name();
        let local = local.as_ref();
        if depth == 1 {
            if local != "Relationships" {
                return Err(format!("unexpected root element <{local}>"));
            }
            return Ok(());
        }
        if depth != 2 {
            return Ok(());
        }
        if local != "Relationship" {
            return if strict {
                Err(format!("unexpected element <{local}>"))
            } else {
                Ok(())
            };
        }
        let id = get_attr(e, "Id", limits)?.unwrap_or_default();
        let kind = trim(&get_attr(e, "Type", limits)?.unwrap_or_default()).to_owned();
        let target = get_attr(e, "Target", limits)?.unwrap_or_default();
        let mode = get_attr(e, "TargetMode", limits)?;
        let external = mode
            .as_deref()
            .is_some_and(|m| m.eq_ignore_ascii_case("External"));
        if strict {
            if !is_ncname(&id) {
                return Err(format!("relationship Id {id:?} is not a valid xsd:ID"));
            }
            if rels.iter().any(|r| r.id == id) {
                return Err(format!("duplicate relationship Id {id:?}"));
            }
            if let Some(m) = &mode
                && m != "Internal"
                && m != "External"
            {
                return Err(format!("invalid TargetMode {m:?}"));
            }
            if !external {
                if !target.is_ascii() {
                    return Err(format!(
                        "relationship target {target:?} has unescaped non-ASCII characters"
                    ));
                }
                let path = target.split('#').next().unwrap_or("");
                if path
                    .trim_start_matches('/')
                    .split('/')
                    .any(|s| s != ".." && bad_segment(s))
                {
                    return Err(format!(
                        "relationship target {target:?} has an invalid segment"
                    ));
                }
            }
        }
        let exact = if external {
            String::new()
        } else {
            super::xml::resolve_exact(source, &target)
        };
        let target = if external {
            String::new()
        } else {
            resolve_target(source, &target)
        };
        if strict
            && let Some(rest) = kind.strip_prefix(OPC_METADATA_REL)
            && rest != "thumbnail"
            && rest != "core-properties"
        {
            return Err(format!("unknown package relationship type {kind:?}"));
        }
        if strict
            && !external
            && rels
                .iter()
                .any(|r| r.kind == kind && r.target == target && !r.external)
        {
            return Err(format!("duplicate relationship to {target}"));
        }
        rels.push(Rel {
            id,
            kind,
            target,
            exact,
            external,
        });
        Ok(())
    })?;
    Ok(rels)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rels_part_names() {
        assert_eq!(rels_key_for("/"), "/_rels/.rels");
        assert_eq!(
            rels_key_for("/3d/3dmodel.model"),
            "/3d/_rels/3dmodel.model.rels"
        );
        assert_eq!(rels_source("/_rels/.rels").as_deref(), Some("/"));
        assert_eq!(
            rels_source("/3d/_rels/3dmodel.model.rels").as_deref(),
            Some("/3d/3dmodel.model")
        );
        assert_eq!(rels_source("/3d/3dmodel.model"), None);
    }

    #[test]
    fn ncname() {
        assert!(is_ncname("rel0"));
        assert!(is_ncname("_rel-1.x"));
        assert!(!is_ncname("0rel"));
        assert!(!is_ncname(""));
        assert!(!is_ncname("a b"));
    }

    #[test]
    fn media_types() {
        assert!(plain_media_type(CT_MODEL));
        assert!(plain_media_type("image/png"));
        assert!(!plain_media_type("application/xml; cow=\"Moo\""));
        assert!(!plain_media_type("png"));
    }
}
