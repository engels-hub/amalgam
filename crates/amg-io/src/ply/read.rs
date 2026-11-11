//! PLY reader (S12 §7, note §6).
//!
//! The body is streamed in chunks by a helper thread. Vertex elements made of scalars only (the
//! usual binary case) are decoded in parallel blocks; everything else (faces, lists, ASCII,
//! unknown elements) goes through one generic record decoder that works on whole records in
//! the buffered bytes. Nothing is allocated from a count in the header beyond what the file
//! can hold.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

use amg_core::{Point2, Point3, Transform};
use amg_jobs::JobCtx;
use amg_mesh::{FaceGroups, IndexedMesh, MeshAttrs, Rgba8};
use crossbeam_channel::Receiver;
use rayon::prelude::*;

use super::header::{self, Element, Encoding, Header, Kind, MAX_HEADER_BYTES, Ty};
use crate::Imported;
use crate::endpoint::Source;
use crate::error::{ImportReport, IoError, IoWarning, Location};
use crate::format::Format;
use crate::obj::{Method, triangulate};
use crate::options::ImportOptions;
use crate::scene::{MaterialSlot, NO_SLOT, Scene, SceneNode, SceneObject, SlotIndex};
use crate::stl::{count_degenerate, open_at};

/// An index that cannot be valid (negative or beyond `u32`).
const BAD: u32 = u32::MAX;
/// A triangle slot of a polygon that was dropped.
const DEAD: [u32; 3] = [u32::MAX - 1; 3];
/// Longest token in an ASCII body.
const MAX_TOKEN: usize = 64;

/// Knobs that tests turn; production uses the defaults.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Tuning {
    /// Bytes per streamed chunk.
    pub chunk_bytes: usize,
    /// Vertex records per parallel block.
    pub block_records: usize,
}

impl Default for Tuning {
    fn default() -> Self {
        Tuning {
            chunk_bytes: 8 << 20,
            block_records: 1 << 15,
        }
    }
}

/// Reads a PLY file into a [`Scene`] with one object whose numbers are the file's. Units, up
/// axis and `merge_objects` are applied by [`crate::import`].
///
/// # Errors
/// [`IoError::Empty`], [`IoError::PointCloud`], [`IoError::Malformed`],
/// [`IoError::LimitExceeded`], [`IoError::Read`] or [`IoError::Cancelled`].
pub fn read(src: Source<'_>, opts: &ImportOptions, ctx: &JobCtx) -> Result<Imported, IoError> {
    read_tuned(src, opts, ctx, Tuning::default())
}

pub(crate) fn read_tuned(
    src: Source<'_>,
    opts: &ImportOptions,
    ctx: &JobCtx,
    tuning: Tuning,
) -> Result<Imported, IoError> {
    let t0 = Instant::now();
    ctx.checkpoint()?;
    let len = src.len()?;
    if len == 0 {
        return Err(IoError::Empty);
    }
    let (head, _) = src.head(MAX_HEADER_BYTES)?;
    let hdr = header::parse(&head, len)?;
    drop(head);

    let lim = &opts.limits;
    for el in &hdr.elements {
        let (what, limit) = match el.name.as_str() {
            "vertex" => ("vertices", lim.max_vertices),
            "face" => ("triangles", lim.max_triangles),
            _ => continue,
        };
        if el.count > limit {
            return Err(IoError::LimitExceeded {
                what,
                limit,
                actual: el.count,
            });
        }
    }

    let mut report = ImportReport {
        format: Some(Format::Ply),
        bytes: len,
        ..ImportReport::default()
    };
    let mut body = Body::new(opts);
    std::thread::scope(|sc| -> Result<(), IoError> {
        let (tx, rx) = crossbeam_channel::bounded::<Result<Vec<u8>, IoError>>(3);
        let mut r = open_at(&src, hdr.body_offset)?;
        let chunk = tuning.chunk_bytes.max(1);
        let path = src.display_path();
        sc.spawn(move || {
            loop {
                let mut buf = vec![0u8; chunk];
                let mut got = 0;
                while got < chunk {
                    match r.read(&mut buf[got..]) {
                        Ok(0) => break,
                        Ok(n) => got += n,
                        Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
                        Err(source) => {
                            let _ = tx.send(Err(IoError::Read {
                                path: path.clone(),
                                source,
                            }));
                            return;
                        }
                    }
                }
                if got == 0 {
                    return;
                }
                buf.truncate(got);
                if tx.send(Ok(buf)).is_err() || got < chunk {
                    return;
                }
            }
        });
        let feed = Feed::new(rx, hdr.body_offset, len, hdr.lines);
        body.run(feed, &hdr, ctx, tuning)
    })?;
    ctx.checkpoint()?;

    let (mesh, slots) = body.finish(ctx, &mut report)?;
    ctx.progress(1.0);

    let mut scene = Scene::default();
    report.vertices = mesh.vertex_count() as u64;
    report.triangles_read = mesh.face_count() as u64;
    report.degenerate_triangles = count_degenerate(&mesh);
    let mut obj = SceneObject::new(Arc::new(mesh));
    if let Some((materials, faces)) = slots {
        scene.materials = materials;
        obj.face_materials = Some(Arc::from(faces));
    }
    scene.objects.push(obj);
    scene.roots.push(SceneNode::mesh(0, Transform::IDENTITY));
    report.duration = t0.elapsed();
    Ok(Imported { scene, report })
}

// ------------------------------------------------------------------------------------ feed

/// The body bytes, pulled chunk by chunk from the reader thread.
struct Feed {
    rx: Receiver<Result<Vec<u8>, IoError>>,
    buf: Vec<u8>,
    pos: usize,
    eof: bool,
    /// File offset of `buf[0]`.
    base: u64,
    file_len: u64,
    /// Lines consumed so far (ASCII bodies), counting the header.
    lines: u64,
}

impl Feed {
    fn new(rx: Receiver<Result<Vec<u8>, IoError>>, base: u64, file_len: u64, lines: u64) -> Feed {
        Feed {
            rx,
            buf: Vec::new(),
            pos: 0,
            eof: false,
            base,
            file_len,
            lines,
        }
    }

    fn avail(&self) -> usize {
        self.buf.len() - self.pos
    }

    fn slice(&self) -> &[u8] {
        &self.buf[self.pos..]
    }

    fn consume(&mut self, n: usize) {
        self.pos += n;
    }

    fn offset(&self) -> u64 {
        self.base + self.pos as u64
    }

    fn remaining_file(&self) -> u64 {
        self.file_len.saturating_sub(self.offset())
    }

    /// Appends the next chunk. `false` at the end of the file.
    fn pull(&mut self) -> Result<bool, IoError> {
        if self.eof {
            return Ok(false);
        }
        match self.rx.recv() {
            Ok(Ok(chunk)) => {
                if self.pos == self.buf.len() {
                    self.base += self.buf.len() as u64;
                    self.buf = chunk;
                    self.pos = 0;
                } else {
                    if self.pos > 0 {
                        self.buf.drain(..self.pos);
                        self.base += self.pos as u64;
                        self.pos = 0;
                    }
                    self.buf.extend_from_slice(&chunk);
                }
                Ok(true)
            }
            Ok(Err(e)) => Err(e),
            Err(_) => {
                self.eof = true;
                Ok(false)
            }
        }
    }

    /// Makes at least `n` bytes available; `false` if the file ends first.
    fn ensure(&mut self, n: usize) -> Result<bool, IoError> {
        while self.avail() < n {
            if !self.pull()? {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

fn malformed(location: Location, message: impl Into<String>) -> IoError {
    IoError::Malformed {
        format: Format::Ply,
        location,
        message: message.into(),
    }
}

// --------------------------------------------------------------------------------- decoding

/// One decoded scalar. Every PLY type up to 32 bits is exact in an `f64`.
#[inline(always)]
fn rd(t: Ty, be: bool, b: &[u8]) -> f64 {
    macro_rules! num {
        ($ty:ty, $n:expr) => {{
            let mut a = [0u8; $n];
            a.copy_from_slice(&b[..$n]);
            if be {
                <$ty>::from_be_bytes(a)
            } else {
                <$ty>::from_le_bytes(a)
            }
        }};
    }
    match t {
        Ty::U8 => f64::from(b[0]),
        Ty::I8 => f64::from(i8::from_ne_bytes([b[0]])),
        Ty::U16 => f64::from(num!(u16, 2)),
        Ty::I16 => f64::from(num!(i16, 2)),
        Ty::U32 => f64::from(num!(u32, 4)),
        Ty::I32 => f64::from(num!(i32, 4)),
        Ty::F32 => f64::from(num!(f32, 4)),
        Ty::F64 => num!(f64, 8),
    }
}

/// A colour channel as 8 bits: floats are 0..1, integers are scaled from their range.
fn to_u8(v: f64, t: Ty) -> u8 {
    let x = match t {
        Ty::F32 | Ty::F64 => (v.clamp(0.0, 1.0) * 255.0).round(),
        Ty::U8 | Ty::I8 => v.clamp(0.0, 255.0),
        Ty::U16 | Ty::I16 => (v.clamp(0.0, 65535.0) / 257.0).round(),
        Ty::U32 | Ty::I32 => (v.clamp(0.0, 4_294_967_295.0) / 16_843_009.0).round(),
    };
    // NaN becomes 0.
    x as u8
}

/// The values of one record.
struct Rec {
    /// Scalar values by property index (list properties stay at 0).
    scalars: Vec<f64>,
    /// Values of the selected index list.
    idx: Vec<i64>,
    /// Values of the selected texture-coordinate list.
    uv: Vec<f64>,
}

impl Rec {
    fn new(props: usize) -> Rec {
        Rec {
            scalars: vec![0.0; props],
            idx: Vec::new(),
            uv: Vec::new(),
        }
    }
}

/// Which list properties are kept.
struct Sel {
    idx: Option<usize>,
    uv: Option<usize>,
}

enum Step {
    Done,
    /// The record needs this many bytes in total from its start.
    Need(usize),
}

/// Decodes one binary record from `s` at `*pos`.
fn bin_record(
    s: &[u8],
    pos: &mut usize,
    el: &Element,
    be: bool,
    sel: &Sel,
    rec: &mut Rec,
) -> Result<Step, &'static str> {
    let start = *pos;
    let mut p = start;
    rec.idx.clear();
    rec.uv.clear();
    for (i, pr) in el.props.iter().enumerate() {
        match pr.kind {
            Kind::Scalar(t) => {
                let sz = t.size();
                if p + sz > s.len() {
                    return Ok(Step::Need(p + sz - start));
                }
                rec.scalars[i] = rd(t, be, &s[p..]);
                p += sz;
            }
            Kind::List { count, item } => {
                let cs = count.size();
                if p + cs > s.len() {
                    return Ok(Step::Need(p + cs - start));
                }
                let cv = rd(count, be, &s[p..]);
                if cv < 0.0 {
                    return Err("a list has a negative length");
                }
                p += cs;
                let n = cv as usize;
                let isz = item.size();
                let bytes = n.checked_mul(isz).ok_or("a list is too long")?;
                if p.checked_add(bytes).is_none_or(|e| e > s.len()) {
                    return Ok(Step::Need((p - start).saturating_add(bytes)));
                }
                if Some(i) == sel.idx {
                    rec.idx
                        .extend((0..n).map(|k| rd(item, be, &s[p + k * isz..]) as i64));
                } else if Some(i) == sel.uv {
                    rec.uv
                        .extend((0..n).map(|k| rd(item, be, &s[p + k * isz..])));
                }
                p += bytes;
            }
        }
    }
    *pos = p;
    Ok(Step::Done)
}

fn is_ws(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\r' | b'\n' | 0x0B | 0x0C)
}

/// Next whitespace-delimited token of an ASCII body, copied into `out`.
fn token(feed: &mut Feed, out: &mut [u8; MAX_TOKEN]) -> Result<Option<usize>, IoError> {
    loop {
        let s = feed.slice();
        match s.iter().position(|&b| !is_ws(b)) {
            Some(i) => {
                feed.lines += s[..i].iter().filter(|&&b| b == b'\n').count() as u64;
                feed.consume(i);
                break;
            }
            None => {
                let n = s.len();
                feed.lines += s.iter().filter(|&&b| b == b'\n').count() as u64;
                feed.consume(n);
                if !feed.pull()? {
                    return Ok(None);
                }
            }
        }
    }
    loop {
        let s = feed.slice();
        let window = &s[..s.len().min(MAX_TOKEN + 1)];
        let end = match window.iter().position(|&b| is_ws(b)) {
            Some(e) => e,
            None if s.len() > MAX_TOKEN => {
                return Err(malformed(
                    Location::Line(feed.lines + 1),
                    "a value is longer than 64 characters",
                ));
            }
            None => {
                if feed.pull()? {
                    continue;
                }
                feed.slice().len()
            }
        };
        let s = feed.slice();
        out[..end].copy_from_slice(&s[..end]);
        feed.consume(end);
        return Ok(Some(end));
    }
}

/// An ASCII number of type `t`.
fn ascii_num(tok: &[u8], t: Ty, line: u64) -> Result<f64, IoError> {
    let bad = || {
        malformed(
            Location::Line(line),
            format!("invalid number '{}'", String::from_utf8_lossy(tok)),
        )
    };
    let s = std::str::from_utf8(tok).map_err(|_| bad())?;
    if t.is_int() {
        if let Ok(v) = s.parse::<i64>() {
            return Ok(v as f64);
        }
        return s
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite() && v.fract() == 0.0)
            .ok_or_else(bad);
    }
    if t == Ty::F32 {
        // A `float` is an f32 whatever the digits say, as in the binary encoding.
        return s.parse::<f32>().map(f64::from).map_err(|_| bad());
    }
    s.parse::<f64>().map_err(|_| bad())
}

/// Decodes one ASCII record; `false` if the file ends first.
fn ascii_record(feed: &mut Feed, el: &Element, sel: &Sel, rec: &mut Rec) -> Result<bool, IoError> {
    rec.idx.clear();
    rec.uv.clear();
    let mut tok = [0u8; MAX_TOKEN];
    for (i, pr) in el.props.iter().enumerate() {
        match pr.kind {
            Kind::Scalar(t) => {
                let Some(n) = token(feed, &mut tok)? else {
                    return Ok(false);
                };
                rec.scalars[i] = ascii_num(&tok[..n], t, feed.lines + 1)?;
            }
            Kind::List { count, item } => {
                let Some(n) = token(feed, &mut tok)? else {
                    return Ok(false);
                };
                let c = ascii_num(&tok[..n], count, feed.lines + 1)?;
                if c < 0.0 {
                    return Err(malformed(
                        Location::Line(feed.lines + 1),
                        "a list has a negative length",
                    ));
                }
                for _ in 0..c as u64 {
                    let Some(n) = token(feed, &mut tok)? else {
                        return Ok(false);
                    };
                    let v = ascii_num(&tok[..n], item, feed.lines + 1)?;
                    if Some(i) == sel.idx {
                        rec.idx.push(v as i64);
                    } else if Some(i) == sel.uv {
                        rec.uv.push(v);
                    }
                }
            }
        }
    }
    Ok(true)
}

/// Runs `on` for every record of `el` until the element ends or the file does. Returns the
/// number of records decoded.
fn run_generic<F>(
    feed: &mut Feed,
    el: &Element,
    enc: Encoding,
    sel: &Sel,
    ctx: &JobCtx,
    mut on: F,
) -> Result<u64, IoError>
where
    F: FnMut(&Rec) -> Result<(), IoError>,
{
    let mut rec = Rec::new(el.props.len());
    let mut done = 0u64;
    if enc == Encoding::Ascii {
        while done < el.count {
            if done.is_multiple_of(4096) {
                ctx.checkpoint()?;
            }
            if !ascii_record(feed, el, sel, &mut rec)? {
                break;
            }
            on(&rec)?;
            done += 1;
        }
        return Ok(done);
    }
    let be = enc == Encoding::Be;
    while done < el.count {
        ctx.checkpoint()?;
        if feed.avail() == 0 && !feed.pull()? {
            break;
        }
        let before = done;
        let mut need = 0usize;
        let mut pos = 0usize;
        let off = feed.offset();
        {
            let s = feed.slice();
            while done < el.count {
                let start = pos;
                match bin_record(s, &mut pos, el, be, sel, &mut rec)
                    .map_err(|m| malformed(Location::Byte(off + start as u64), m))?
                {
                    Step::Done => {
                        on(&rec)?;
                        done += 1;
                    }
                    Step::Need(n) => {
                        pos = start;
                        need = n;
                        break;
                    }
                }
            }
        }
        feed.consume(pos);
        if done == before
            && need > 0
            && (need as u64 > feed.remaining_file() || !feed.ensure(need)?)
        {
            break;
        }
    }
    Ok(done)
}

// ----------------------------------------------------------------------------------- layout

const COLOR_NAMES: [[&str; 2]; 4] = [
    ["red", "diffuse_red"],
    ["green", "diffuse_green"],
    ["blue", "diffuse_blue"],
    ["alpha", "diffuse_alpha"],
];
const UV_NAMES: [[&str; 2]; 4] = [
    ["s", "t"],
    ["u", "v"],
    ["texture_u", "texture_v"],
    ["texture_s", "texture_t"],
];

/// Types of the scalar properties of an element, by property index (lists get `U8`).
fn prop_types(el: &Element) -> Vec<Ty> {
    el.props
        .iter()
        .map(|p| match p.kind {
            Kind::Scalar(t) => t,
            Kind::List { .. } => Ty::U8,
        })
        .collect()
}

/// RGB (and alpha) scalar properties of an element.
fn color_props(el: &Element) -> (Option<[usize; 3]>, Option<usize>) {
    let c = |k: usize| el.scalar(&COLOR_NAMES[k]);
    let rgb = match (c(0), c(1), c(2)) {
        (Some(r), Some(g), Some(b)) => Some([r, g, b]),
        _ => None,
    };
    (rgb, rgb.and_then(|_| c(3)))
}

struct VLayout {
    xyz: [usize; 3],
    rgb: Option<[usize; 3]>,
    alpha: Option<usize>,
    uv: Option<[usize; 2]>,
    tys: Vec<Ty>,
}

fn vertex_layout(el: &Element) -> Result<VLayout, IoError> {
    let p = |n: &str| el.scalar(&[n]);
    let (Some(x), Some(y), Some(z)) = (p("x"), p("y"), p("z")) else {
        return Err(malformed(
            Location::Line(el.line),
            "the vertex element has no scalar x, y and z properties",
        ));
    };
    let (rgb, alpha) = color_props(el);
    let uv = UV_NAMES.iter().find_map(|[a, b]| match (p(a), p(b)) {
        (Some(a), Some(b)) => Some([a, b]),
        _ => None,
    });
    Ok(VLayout {
        xyz: [x, y, z],
        rgb,
        alpha,
        uv,
        tys: prop_types(el),
    })
}

/// Vertices decoded from a run of records.
#[derive(Default)]
struct VBlock {
    pos: Vec<Point3>,
    col: Vec<Rgba8>,
    uv: Vec<[f64; 2]>,
}

#[inline(always)]
fn push_vertex<G: Fn(usize) -> f64>(b: &mut VBlock, lay: &VLayout, get: G) {
    b.pos.push(Point3::new(
        get(lay.xyz[0]),
        get(lay.xyz[1]),
        get(lay.xyz[2]),
    ));
    if let Some([r, g, bl]) = lay.rgb {
        let c = |i: usize| to_u8(get(i), lay.tys[i]);
        let a = lay.alpha.map_or(255, c);
        b.col.push(Rgba8([c(r), c(g), c(bl), a]));
    }
    if let Some([u, v]) = lay.uv {
        b.uv.push([get(u), get(v)]);
    }
}

// ------------------------------------------------------------------------------------ body

#[derive(Copy, Clone)]
struct PolyRec {
    /// First reserved triangle slot.
    tri: usize,
    /// First corner in `poly_corners` (and `poly_uv`).
    start: usize,
    len: usize,
}

struct FLayout {
    rgb: Option<[usize; 3]>,
    alpha: Option<usize>,
    tys: Vec<Ty>,
}

struct Body {
    max_triangles: u64,
    positions: Vec<Point3>,
    colors: Option<Vec<Rgba8>>,
    vertex_uv: Option<Vec<[f64; 2]>>,
    seen_vertex: bool,
    tri_v: Vec<[u32; 3]>,
    tri_col: Option<Vec<[u8; 4]>>,
    tri_uv: Option<Vec<[[f64; 2]; 3]>>,
    poly_corners: Vec<u32>,
    poly_uv: Vec<[f64; 2]>,
    polys: Vec<PolyRec>,
    /// A face element with index lists was read, with this many declared faces.
    faces_declared: Option<u64>,
    small_faces: u64,
    warnings: Vec<IoWarning>,
}

enum Outcome {
    Full,
    /// The file ended after this many records.
    Short(u64),
}

impl Body {
    fn new(opts: &ImportOptions) -> Body {
        Body {
            max_triangles: opts.limits.max_triangles,
            positions: Vec::new(),
            colors: None,
            vertex_uv: None,
            seen_vertex: false,
            tri_v: Vec::new(),
            tri_col: None,
            tri_uv: None,
            poly_corners: Vec::new(),
            poly_uv: Vec::new(),
            polys: Vec::new(),
            faces_declared: None,
            small_faces: 0,
            warnings: Vec::new(),
        }
    }

    fn run(
        &mut self,
        mut feed: Feed,
        hdr: &Header,
        ctx: &JobCtx,
        tuning: Tuning,
    ) -> Result<(), IoError> {
        let (mut vertex_done, mut face_done) = (false, false);
        let mut truncated = false;
        let total = feed.file_len.max(1) as f32;
        for el in &hdr.elements {
            ctx.checkpoint()?;
            ctx.progress(0.9 * (feed.offset() as f32 / total).min(1.0));
            let outcome = if el.name == "vertex" && !vertex_done {
                vertex_done = true;
                self.vertex(&mut feed, el, hdr.encoding, ctx, tuning)?
            } else if el.name == "face" && !face_done && el.list(FACE_INDEX).is_some() {
                face_done = true;
                self.face(&mut feed, el, hdr.encoding, ctx)?
            } else {
                self.skip(&mut feed, el, hdr.encoding, ctx)?
            };
            if let Outcome::Short(read) = outcome {
                self.warnings.push(IoWarning::Truncated {
                    expected: el.count,
                    read,
                });
                truncated = true;
                break;
            }
        }
        if !self.seen_vertex {
            return Err(malformed(
                Location::Line(1),
                "the file has no vertex element",
            ));
        }
        if !truncated {
            let mut extra = 0u64;
            let ascii = hdr.encoding == Encoding::Ascii;
            loop {
                ctx.checkpoint()?;
                let s = feed.slice();
                extra += if ascii {
                    s.iter().filter(|&&b| !is_ws(b)).count() as u64
                } else {
                    s.len() as u64
                };
                let n = s.len();
                feed.consume(n);
                if !feed.pull()? {
                    break;
                }
            }
            if extra > 0 {
                self.warnings.push(IoWarning::TrailingBytes(extra));
            }
        }
        Ok(())
    }

    /// Elements we do not use are skipped by size (binary, scalars only) or record by record.
    fn skip(
        &mut self,
        feed: &mut Feed,
        el: &Element,
        enc: Encoding,
        ctx: &JobCtx,
    ) -> Result<Outcome, IoError> {
        if el.count == 0 {
            return Ok(Outcome::Full);
        }
        let what = if el.name == "face" && el.list(FACE_INDEX).is_none() {
            "PLY face element without a vertex_indices list".to_owned()
        } else {
            format!("PLY element '{}'", el.name)
        };
        self.warnings.push(IoWarning::UnsupportedContentIgnored {
            what,
            count: el.count,
        });
        if enc != Encoding::Ascii
            && let Some(rec) = el.fixed_record()
        {
            let mut left = (el.count as u128) * (rec as u128);
            while left > 0 {
                ctx.checkpoint()?;
                if feed.avail() == 0 && !feed.pull()? {
                    let read = if rec == 0 {
                        el.count
                    } else {
                        let total = (el.count as u128) * (rec as u128);
                        ((total - left) / rec as u128) as u64
                    };
                    return Ok(Outcome::Short(read));
                }
                let n = (feed.avail() as u128).min(left) as usize;
                feed.consume(n);
                left -= n as u128;
            }
            return Ok(Outcome::Full);
        }
        let sel = Sel {
            idx: None,
            uv: None,
        };
        let done = run_generic(feed, el, enc, &sel, ctx, |_| Ok(()))?;
        Ok(if done < el.count {
            Outcome::Short(done)
        } else {
            Outcome::Full
        })
    }

    fn vertex(
        &mut self,
        feed: &mut Feed,
        el: &Element,
        enc: Encoding,
        ctx: &JobCtx,
        tuning: Tuning,
    ) -> Result<Outcome, IoError> {
        let lay = vertex_layout(el)?;
        self.seen_vertex = true;
        let min = if enc == Encoding::Ascii {
            el.props.len().max(1) as u64 * 2
        } else {
            el.min_record().max(1)
        };
        let cap = el.count.min(feed.remaining_file() / min) as usize;
        self.positions.reserve(cap);
        self.colors = lay.rgb.map(|_| Vec::with_capacity(cap));
        self.vertex_uv = lay.uv.map(|_| Vec::with_capacity(cap));
        let eof = |read: u64, off: u64| {
            malformed(
                Location::Byte(off),
                format!(
                    "the file ends inside the vertex data ({read} of {} vertices)",
                    el.count
                ),
            )
        };

        if enc != Encoding::Ascii
            && let Some(rec) = el.fixed_record()
        {
            let be = enc == Encoding::Be;
            // Offsets of the properties inside a record.
            let mut offs = Vec::with_capacity(el.props.len());
            let mut o = 0;
            for t in &lay.tys {
                offs.push(o);
                o += t.size();
            }
            let mut left = el.count;
            while left > 0 {
                ctx.checkpoint()?;
                if feed.avail() < rec && !feed.ensure(rec)? {
                    return Err(eof(el.count - left, feed.offset()));
                }
                let k = ((feed.avail() / rec.max(1)) as u64).min(left) as usize;
                let bytes = &feed.slice()[..k * rec];
                let per = tuning.block_records.max(1) * rec.max(1);
                let blocks: Vec<VBlock> = bytes
                    .par_chunks(per.max(1))
                    .map(|c| {
                        let mut b = VBlock::default();
                        b.pos.reserve(c.len() / rec.max(1));
                        for r in c.chunks_exact(rec.max(1)) {
                            push_vertex(&mut b, &lay, |i| rd(lay.tys[i], be, &r[offs[i]..]));
                        }
                        b
                    })
                    .collect();
                for b in blocks {
                    self.absorb(b);
                }
                feed.consume(k * rec);
                left -= k as u64;
                ctx.progress(0.9 * (feed.offset() as f32 / feed.file_len.max(1) as f32).min(1.0));
            }
            return Ok(Outcome::Full);
        }

        let sel = Sel {
            idx: None,
            uv: None,
        };
        let mut acc = VBlock::default();
        let done = run_generic(feed, el, enc, &sel, ctx, |rec| {
            push_vertex(&mut acc, &lay, |i| rec.scalars[i]);
            Ok(())
        })?;
        self.absorb(acc);
        if done < el.count {
            return Err(eof(done, feed.offset()));
        }
        Ok(Outcome::Full)
    }

    fn absorb(&mut self, b: VBlock) {
        self.positions.extend(b.pos);
        if let Some(c) = self.colors.as_mut() {
            c.extend(b.col);
        }
        if let Some(u) = self.vertex_uv.as_mut() {
            u.extend(b.uv);
        }
    }

    fn face(
        &mut self,
        feed: &mut Feed,
        el: &Element,
        enc: Encoding,
        ctx: &JobCtx,
    ) -> Result<Outcome, IoError> {
        let ip = el.list(FACE_INDEX);
        let sel = Sel {
            idx: ip,
            uv: el.list(&["texcoord"]),
        };
        let (rgb, alpha) = color_props(el);
        let lay = FLayout {
            rgb,
            alpha,
            tys: prop_types(el),
        };
        if rgb.is_some() {
            self.tri_col = Some(Vec::new());
        }
        if sel.uv.is_some() {
            self.tri_uv = Some(Vec::new());
        }
        self.faces_declared = Some(el.count);
        let min = if enc == Encoding::Ascii {
            el.props.len().max(1) as u64 * 2
        } else {
            el.min_record().max(1)
        };
        self.tri_v
            .reserve(el.count.min(feed.remaining_file() / min) as usize);
        let done = run_generic(feed, el, enc, &sel, ctx, |rec| self.on_face(rec, &lay))?;
        Ok(if done < el.count {
            Outcome::Short(done)
        } else {
            Outcome::Full
        })
    }

    fn on_face(&mut self, rec: &Rec, lay: &FLayout) -> Result<(), IoError> {
        let n = rec.idx.len();
        if n < 3 {
            self.small_faces += 1;
            return Ok(());
        }
        let conv = |v: i64| {
            if v < 0 || v >= i64::from(u32::MAX - 1) {
                BAD
            } else {
                v as u32
            }
        };
        let ntri = n - 2;
        if n == 3 {
            self.tri_v
                .push([conv(rec.idx[0]), conv(rec.idx[1]), conv(rec.idx[2])]);
        } else {
            self.polys.push(PolyRec {
                tri: self.tri_v.len(),
                start: self.poly_corners.len(),
                len: n,
            });
            self.poly_corners.extend(rec.idx.iter().map(|&v| conv(v)));
            self.tri_v.extend(std::iter::repeat_n(DEAD, ntri));
        }
        if let Some(tc) = self.tri_col.as_mut()
            && let Some([r, g, b]) = lay.rgb
        {
            let c = |i: usize| to_u8(rec.scalars[i], lay.tys[i]);
            let a = lay.alpha.map_or(255, c);
            tc.extend(std::iter::repeat_n([c(r), c(g), c(b), a], ntri));
        }
        if let Some(tu) = self.tri_uv.as_mut() {
            let uv = |k: usize| [rec.uv[2 * k], rec.uv[2 * k + 1]];
            let ok = rec.uv.len() == 2 * n;
            if n == 3 {
                tu.push(if ok {
                    [uv(0), uv(1), uv(2)]
                } else {
                    [[0.0; 2]; 3]
                });
            } else {
                tu.extend(std::iter::repeat_n([[0.0; 2]; 3], ntri));
                if ok {
                    self.poly_uv.extend((0..n).map(uv));
                } else {
                    self.poly_uv.extend(std::iter::repeat_n([0.0; 2], n));
                }
            }
        }
        if self.tri_v.len() as u64 > self.max_triangles {
            return Err(IoError::LimitExceeded {
                what: "triangles",
                limit: self.max_triangles,
                actual: self.tri_v.len() as u64,
            });
        }
        Ok(())
    }

    #[allow(clippy::type_complexity)]
    fn finish(
        mut self,
        ctx: &JobCtx,
        report: &mut ImportReport,
    ) -> Result<(IndexedMesh, Option<(Vec<MaterialSlot>, Vec<SlotIndex>)>), IoError> {
        let nv = self.positions.len();
        if nv == 0 {
            return Err(IoError::Empty);
        }
        if self.faces_declared.is_none_or(|n| n == 0) {
            return Err(IoError::PointCloud);
        }
        let nv32 = nv as u32;
        let any_nonfinite = self.positions.par_iter().any(|p| !p.is_finite());
        let mut invalid = 0u64;
        let mut nonfinite = 0u64;

        // Polygons: validated, then ear clipped into their reserved slots.
        let mut fallback = 0u64;
        let mut scratch: Vec<[u32; 3]> = Vec::new();
        let mut pts: Vec<Point3> = Vec::new();
        for (k, p) in self.polys.iter().enumerate() {
            if k % 4096 == 0 {
                ctx.checkpoint()?;
            }
            let corners = &self.poly_corners[p.start..p.start + p.len];
            let slots = p.tri..p.tri + p.len - 2;
            if corners.iter().any(|&c| c >= nv32) {
                invalid += 1;
                self.tri_v[slots].fill(DEAD);
                continue;
            }
            if any_nonfinite
                && corners
                    .iter()
                    .any(|&c| !self.positions[c as usize].is_finite())
            {
                nonfinite += 1;
                self.tri_v[slots].fill(DEAD);
                continue;
            }
            pts.clear();
            pts.extend(corners.iter().map(|&c| self.positions[c as usize]));
            scratch.clear();
            if triangulate(&pts, &mut scratch) == Method::Fan {
                fallback += 1;
            }
            for (j, t) in scratch.iter().enumerate() {
                self.tri_v[p.tri + j] = [
                    corners[t[0] as usize],
                    corners[t[1] as usize],
                    corners[t[2] as usize],
                ];
                if let Some(tu) = self.tri_uv.as_mut() {
                    let uv = |c: u32| self.poly_uv[p.start + c as usize];
                    tu[p.tri + j] = [uv(t[0]), uv(t[1]), uv(t[2])];
                }
            }
        }
        if !self.polys.is_empty() {
            self.warnings.push(IoWarning::PolygonTriangulated {
                polygons: self.polys.len() as u64,
                fallback_fan: fallback,
            });
        }
        drop(self.poly_corners);
        drop(self.poly_uv);

        // Triangles on invalid or non-finite vertices are dropped.
        let positions = &self.positions;
        let classify = |t: &[u32; 3]| -> u8 {
            if *t == DEAD {
                return 3;
            }
            if t.iter().any(|&c| c >= nv32) {
                return 1;
            }
            if any_nonfinite && t.iter().any(|&c| !positions[c as usize].is_finite()) {
                return 2;
            }
            0
        };
        let counts = self
            .tri_v
            .par_iter()
            .fold(
                || [0u64; 4],
                |mut a, t| {
                    a[classify(t) as usize] += 1;
                    a
                },
            )
            .reduce(
                || [0u64; 4],
                |mut a, b| {
                    for i in 0..4 {
                        a[i] += b[i];
                    }
                    a
                },
            );
        invalid += counts[1];
        nonfinite += counts[2];
        if counts[1] + counts[2] + counts[3] > 0 {
            let keep: Vec<bool> = self.tri_v.par_iter().map(|t| classify(t) == 0).collect();
            compact(&mut self.tri_v, &keep);
            if let Some(v) = self.tri_col.as_mut() {
                compact(v, &keep);
            }
            if let Some(v) = self.tri_uv.as_mut() {
                compact(v, &keep);
            }
        }
        if invalid > 0 {
            self.warnings.push(IoWarning::InvalidIndexDropped(invalid));
        }
        if nonfinite > 0 {
            self.warnings.push(IoWarning::NonFiniteDropped(nonfinite));
        }
        if self.small_faces > 0 {
            self.warnings.push(IoWarning::UnsupportedContentIgnored {
                what: "PLY face with fewer than 3 corners".to_owned(),
                count: self.small_faces,
            });
        }
        report.warnings.append(&mut self.warnings);
        if self.tri_v.is_empty() {
            return Err(IoError::Empty);
        }
        ctx.checkpoint()?;

        // Face colours become slots (only 65 534 distinct colours fit).
        let mut slots: Option<(Vec<MaterialSlot>, Vec<SlotIndex>)> = None;
        if let Some(tc) = self.tri_col.take() {
            let mut by_color: HashMap<[u8; 4], SlotIndex> = HashMap::new();
            let mut table: Vec<MaterialSlot> = Vec::new();
            let mut faces: Vec<SlotIndex> = Vec::with_capacity(tc.len());
            let mut overflow = 0u64;
            for c in &tc {
                let slot = match by_color.get(c) {
                    Some(&s) => s,
                    None if table.len() < usize::from(NO_SLOT) => {
                        let s = table.len() as SlotIndex;
                        table.push(MaterialSlot {
                            name: if c[3] == 255 {
                                format!("#{:02X}{:02X}{:02X}", c[0], c[1], c[2])
                            } else {
                                format!("#{:02X}{:02X}{:02X}{:02X}", c[0], c[1], c[2], c[3])
                            },
                            color: Rgba8(*c),
                            filament: None,
                        });
                        by_color.insert(*c, s);
                        s
                    }
                    None => {
                        overflow += 1;
                        NO_SLOT
                    }
                };
                faces.push(slot);
            }
            if overflow > 0 {
                report.warnings.push(IoWarning::UnsupportedContentIgnored {
                    what: "PLY face colours beyond 65 534 distinct values".to_owned(),
                    count: overflow,
                });
            }
            slots = Some((table, faces));
        }

        let mut attrs = MeshAttrs {
            vertex_colors: self.colors.take(),
            ..MeshAttrs::default()
        };
        attrs.corner_uvs = match (self.tri_uv.take(), self.vertex_uv.take()) {
            (Some(tu), _) => Some(
                tu.par_iter()
                    .map(|t| t.map(|[u, v]| Point2::new(u, v)))
                    .collect(),
            ),
            (None, Some(vu)) => Some(
                self.tri_v
                    .par_iter()
                    .map(|t| t.map(|c| Point2::new(vu[c as usize][0], vu[c as usize][1])))
                    .collect(),
            ),
            (None, None) => None,
        };
        let groups = FaceGroups::new(self.tri_v.len());
        let mesh = IndexedMesh {
            positions: self.positions,
            tris: self.tri_v,
            attrs,
            groups,
        };
        mesh.validate()?;
        Ok((mesh, slots))
    }
}

const FACE_INDEX: &[&str] = &["vertex_indices", "vertex_index"];

/// Keeps the entries whose flag is set, in order.
fn compact<T>(v: &mut Vec<T>, keep: &[bool]) {
    let mut w = 0;
    for (r, &k) in keep.iter().enumerate() {
        if k {
            v.swap(w, r);
            w += 1;
        }
    }
    v.truncate(w);
}
