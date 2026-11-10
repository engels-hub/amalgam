//! STL reader (S12 §5, note §5.1-5.3): detection, quirks, streaming exact weld, ASCII in
//! parallel, multi-solid, face colours.

use std::collections::HashMap;
use std::io::Read;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use amg_core::{Point3, Transform};
use amg_jobs::JobCtx;
use amg_mesh::{FaceGroups, IndexedMesh, MeshAttrs, Rgba8, WeldOpts};
use rayon::prelude::*;

use super::ascii::{self, Event};
use super::weld::{Key, Welder};
use super::{Rule, StlKind, detect_rule};
use crate::Imported;
use crate::endpoint::Source;
use crate::error::{ImportReport, IoError, IoWarning, Location};
use crate::format::{Format, SNIFF_HEAD};
use crate::options::{ImportOptions, ReadLimits, StlWeld};
use crate::scene::{MaterialSlot, NO_SLOT, Scene, SceneNode, SceneObject, SlotIndex};

const REC: usize = 50;
/// Binary records per parse block.
const BLOCK_RECS: usize = 2048;
/// Longest stretch of ASCII without an `endfacet` that we are willing to buffer.
const MAX_ASCII_CARRY: usize = 256 << 20;

/// Knobs that tests turn; production uses the defaults.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Tuning {
    /// Bytes per streamed chunk.
    pub chunk_bytes: usize,
}

impl Default for Tuning {
    fn default() -> Self {
        Tuning {
            chunk_bytes: 8 << 20,
        }
    }
}

/// Reads an STL file (binary or ASCII, found by note §5.1) into a [`Scene`] whose numbers are
/// the file's, widened from f32. Units, up axis and `merge_objects` are applied by
/// [`crate::import`], not here.
///
/// The scene has one object per ASCII `solid` block with `split_solids`, otherwise one.
///
/// # Errors
/// [`IoError::Empty`], [`IoError::Malformed`], [`IoError::LimitExceeded`], [`IoError::Read`]
/// or [`IoError::Cancelled`].
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
    let (head, len) = src.head(SNIFF_HEAD)?;
    if len == 0 {
        return Err(IoError::Empty);
    }
    let kind = match detect_rule(&head, len) {
        Ok(k) => k,
        // A short file that is plainly the start of an ASCII STL (`solid x endsolid x`).
        Err(e) => {
            let t = super::skip_ws(super::strip_bom(&head));
            if t.len() >= 5 && t[..5].eq_ignore_ascii_case(b"solid") {
                (StlKind::Ascii, Rule::AsciiKeyword)
            } else {
                return Err(e);
            }
        }
    };
    let mut report = ImportReport {
        format: Some(Format::Stl),
        bytes: len,
        ..ImportReport::default()
    };
    let mut parts = match kind {
        (StlKind::Binary { declared }, rule) => read_binary(
            &src,
            &head,
            len,
            declared,
            rule,
            opts,
            ctx,
            tuning,
            &mut report,
        )?,
        (StlKind::Ascii, _) => read_ascii(&src, len, opts, ctx, tuning, &mut report)?,
    };
    ctx.progress(1.0);

    // Objects -> scene.
    let mut scene = Scene {
        materials: std::mem::take(&mut parts.materials),
        ..Scene::default()
    };
    for (i, o) in parts.objects.into_iter().enumerate() {
        report.vertices += o.mesh.vertex_count() as u64;
        report.degenerate_triangles += count_degenerate(&o.mesh);
        report.triangles_read += o.mesh.face_count() as u64;
        scene
            .roots
            .push(SceneNode::mesh(i as u32, Transform::IDENTITY));
        scene.objects.push(o);
    }
    if scene.objects.is_empty() {
        return Err(IoError::Empty);
    }
    report.duration = t0.elapsed();
    Ok(Imported { scene, report })
}

struct Parts {
    objects: Vec<SceneObject>,
    materials: Vec<MaterialSlot>,
}

/// Collects the triangles of one object under the chosen weld mode.
enum Collector {
    Exact(Welder),
    Soup {
        /// `Some(tol)` merges at the end, `None` keeps the soup.
        tolerance: Option<f64>,
        tris: Vec<[Point3; 3]>,
    },
}

impl Collector {
    fn new(weld: StlWeld, expected: usize, limits: &ReadLimits) -> Self {
        match weld {
            StlWeld::Exact => Collector::Exact(Welder::new(expected, limits.max_vertices)),
            StlWeld::Tolerance(t) => Collector::Soup {
                tolerance: Some(t),
                tris: Vec::new(),
            },
            StlWeld::None => Collector::Soup {
                tolerance: None,
                tris: Vec::new(),
            },
        }
    }

    fn tri_count(&self) -> usize {
        match self {
            Collector::Exact(w) => w.triangle_count(),
            Collector::Soup { tris, .. } => tris.len(),
        }
    }

    fn push(&mut self, corners: &[Key], ctx: &JobCtx) -> Result<(), IoError> {
        match self {
            Collector::Exact(w) => w.push(corners, ctx),
            Collector::Soup { tris, .. } => {
                tris.extend(corners.chunks_exact(3).map(|t| {
                    [t[0], t[1], t[2]].map(|k| {
                        Point3::new(
                            f64::from(f32::from_bits(k[0])),
                            f64::from(f32::from_bits(k[1])),
                            f64::from(f32::from_bits(k[2])),
                        )
                    })
                }));
                ctx.checkpoint()?;
                Ok(())
            }
        }
    }

    fn finish(self, ctx: &JobCtx) -> Result<IndexedMesh, IoError> {
        ctx.checkpoint()?;
        match self {
            Collector::Exact(w) => {
                let (positions, tris) = w.finish();
                Ok(IndexedMesh {
                    groups: FaceGroups::new(tris.len()),
                    positions,
                    tris,
                    attrs: MeshAttrs::default(),
                })
            }
            Collector::Soup {
                tolerance: Some(tolerance),
                tris,
            } => Ok(IndexedMesh::from_soup(&tris, WeldOpts { tolerance })?),
            Collector::Soup {
                tolerance: None,
                tris,
            } => {
                let n = tris.len();
                let positions: Vec<Point3> = tris.into_iter().flatten().collect();
                let faces: Vec<[u32; 3]> = (0..n as u32)
                    .map(|i| [3 * i, 3 * i + 1, 3 * i + 2])
                    .collect();
                Ok(IndexedMesh {
                    groups: FaceGroups::new(n),
                    positions,
                    tris: faces,
                    attrs: MeshAttrs::default(),
                })
            }
        }
    }
}

/// Triangles with a repeated vertex or zero area.
pub(crate) fn count_degenerate(m: &IndexedMesh) -> u64 {
    m.tris
        .par_iter()
        .filter(|t| {
            if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                return true;
            }
            let (a, b, c) = (
                m.positions[t[0] as usize],
                m.positions[t[1] as usize],
                m.positions[t[2] as usize],
            );
            (b - a).cross(c - a).length_squared() <= 0.0
        })
        .count() as u64
}

pub(crate) fn open_at<'a>(
    src: &Source<'a>,
    offset: u64,
) -> Result<Box<dyn Read + Send + 'a>, IoError> {
    match src {
        Source::Bytes(b) => {
            let start = usize::try_from(offset).unwrap_or(usize::MAX).min(b.len());
            Ok(Box::new(&b[start..]))
        }
        Source::Path(p) => {
            use std::io::{Seek, SeekFrom};
            let err = |source| IoError::Read {
                path: (*p).to_owned(),
                source,
            };
            let mut f = std::fs::File::open(p).map_err(err)?;
            f.seek(SeekFrom::Start(offset)).map_err(err)?;
            Ok(Box::new(f))
        }
    }
}

/// Reads `total` bytes (or to the end) in `chunk`-sized pieces on a helper thread and hands
/// them to `consume` in order. Stops at the first error of `consume`.
pub(crate) fn stream<'a, F>(
    mut r: Box<dyn Read + Send + 'a>,
    total: u64,
    chunk: usize,
    path: PathBuf,
    mut consume: F,
) -> Result<(), IoError>
where
    F: FnMut(Vec<u8>) -> Result<(), IoError>,
{
    std::thread::scope(|sc| {
        let (tx, rx) = crossbeam_channel::bounded::<Result<Vec<u8>, IoError>>(4);
        sc.spawn(move || {
            let mut remaining = total;
            while remaining > 0 {
                let want = remaining.min(chunk as u64) as usize;
                let mut buf = vec![0u8; want];
                let mut got = 0;
                while got < want {
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
                remaining -= got as u64;
                if tx.send(Ok(buf)).is_err() || got < want {
                    return;
                }
            }
        });
        for item in rx.iter() {
            consume(item?)?;
        }
        Ok(())
    })
}

// ------------------------------------------------------------------------------- binary

#[inline]
fn le32(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

#[inline]
fn finite_bits(b: u32) -> bool {
    b & 0x7F80_0000 != 0x7F80_0000
}

/// Parses whole records of `chunk` into `corners` (3 per record) and `attrs`; returns the
/// number of records with a non-finite coordinate (they stay in place, flagged by their bits).
fn parse_binary(chunk: &[u8], corners: &mut [Key], attrs: &mut [u16], ctx: &JobCtx) -> u64 {
    corners
        .par_chunks_mut(3 * BLOCK_RECS)
        .zip(attrs.par_chunks_mut(BLOCK_RECS))
        .zip(chunk.par_chunks(REC * BLOCK_RECS))
        .map(|((cs, at), bytes)| {
            if ctx.is_cancelled() {
                return 0;
            }
            let mut bad = 0u64;
            for (i, rec) in bytes.chunks_exact(REC).enumerate() {
                let mut ok = true;
                for j in 0..9 {
                    let mut b = le32(rec, 12 + 4 * j);
                    ok &= finite_bits(b);
                    if b == 0x8000_0000 {
                        b = 0;
                    }
                    cs[3 * i + j / 3][j % 3] = b;
                }
                at[i] = u16::from_le_bytes([rec[48], rec[49]]);
                bad += u64::from(!ok);
            }
            bad
        })
        .sum()
}

/// Face colour scheme of a binary file (note §5.2).
#[derive(Clone, Copy)]
struct ColorScheme {
    /// Materialise Magics: the header holds `COLOR=` and bit 15 is clear for a coloured face.
    magics: bool,
    /// Magics default colour from the header.
    default: Option<[u8; 4]>,
}

impl ColorScheme {
    fn of(header: &[u8]) -> Self {
        let pos = header.windows(6).position(|w| w == b"COLOR=");
        match pos {
            Some(p) if p + 10 <= header.len() => ColorScheme {
                magics: true,
                default: Some([header[p + 6], header[p + 7], header[p + 8], header[p + 9]]),
            },
            Some(_) => ColorScheme {
                magics: true,
                default: None,
            },
            None => ColorScheme {
                magics: false,
                default: None,
            },
        }
    }

    /// RGBA of an attribute word, `None` = no colour.
    fn color(self, attr: u16) -> Option<[u8; 4]> {
        let five = |v: u16| {
            let v = (v & 31) as u8;
            (v << 3) | (v >> 2)
        };
        if self.magics {
            if attr & 0x8000 == 0 {
                // bits 0-4 red, 5-9 green, 10-14 blue
                Some([five(attr), five(attr >> 5), five(attr >> 10), 255])
            } else {
                self.default
            }
        } else if attr & 0x8000 != 0 {
            // VisCAM / SolidView: bits 0-4 blue, 5-9 green, 10-14 red
            Some([five(attr >> 10), five(attr >> 5), five(attr), 255])
        } else {
            None
        }
    }
}

/// Slot table and per-face slots from attribute words.
fn colors_to_slots(
    attrs: &[u16],
    scheme: ColorScheme,
) -> Option<(Vec<MaterialSlot>, Vec<SlotIndex>)> {
    let mut by_color: HashMap<[u8; 4], SlotIndex> = HashMap::new();
    let mut slots = Vec::new();
    let mut faces: Vec<SlotIndex> = Vec::with_capacity(attrs.len());
    let mut any = false;
    // Attribute words repeat a lot; memoise per word.
    let mut memo: HashMap<u16, SlotIndex> = HashMap::new();
    for &a in attrs {
        let slot = *memo.entry(a).or_insert_with(|| match scheme.color(a) {
            None => NO_SLOT,
            Some(c) => *by_color.entry(c).or_insert_with(|| {
                slots.push(MaterialSlot {
                    name: format!("#{:02X}{:02X}{:02X}", c[0], c[1], c[2]),
                    color: Rgba8(c),
                    filament: None,
                });
                (slots.len() - 1) as SlotIndex
            }),
        });
        any |= slot != NO_SLOT;
        faces.push(slot);
    }
    any.then_some((slots, faces))
}

#[allow(clippy::too_many_arguments)]
fn read_binary(
    src: &Source<'_>,
    head: &[u8],
    len: u64,
    declared: u32,
    rule: Rule,
    opts: &ImportOptions,
    ctx: &JobCtx,
    tuning: Tuning,
    report: &mut ImportReport,
) -> Result<Parts, IoError> {
    let actual = (len - 84) / REC as u64;
    let declared = u64::from(declared);
    let n = match rule {
        Rule::CountMismatch => {
            if declared != actual {
                report.warnings.push(IoWarning::HeaderCountMismatch {
                    header: declared,
                    actual,
                });
            }
            actual
        }
        Rule::SizeMismatch => {
            if declared > actual {
                report.warnings.push(IoWarning::Truncated {
                    expected: declared,
                    read: actual,
                });
                actual
            } else {
                report
                    .warnings
                    .push(IoWarning::TrailingBytes(len - 84 - REC as u64 * declared));
                declared
            }
        }
        _ => declared,
    };
    if n > opts.limits.max_triangles {
        return Err(IoError::LimitExceeded {
            what: "triangles",
            limit: opts.limits.max_triangles,
            actual: n,
        });
    }
    if n == 0 {
        return Err(IoError::Empty);
    }

    let want_colors = opts.stl.face_colors;
    let scheme = ColorScheme::of(&head[..80.min(head.len())]);
    let mut collector = Collector::new(opts.stl.weld, n as usize, &opts.limits);
    let mut attr_words: Vec<u16> = Vec::new();
    if want_colors {
        attr_words.reserve(n as usize);
    }
    let mut dropped = 0u64;
    let recs_per_chunk = (tuning.chunk_bytes / REC).max(1);
    let total_bytes = n * REC as u64;
    let mut done = 0u64;

    stream(
        open_at(src, 84)?,
        total_bytes,
        recs_per_chunk * REC,
        src.display_path(),
        |chunk| {
            ctx.checkpoint()?;
            let nrec = chunk.len() / REC;
            let mut corners = vec![[0u32; 3]; 3 * nrec];
            let mut attrs = vec![0u16; nrec];
            let bad = parse_binary(&chunk[..nrec * REC], &mut corners, &mut attrs, ctx);
            ctx.checkpoint()?;
            if bad > 0 {
                dropped += bad;
                let keep: Vec<bool> = corners
                    .chunks_exact(3)
                    .map(|t| t.iter().flatten().all(|&b| finite_bits(b)))
                    .collect();
                corners = corners
                    .chunks_exact(3)
                    .zip(&keep)
                    .filter(|(_, k)| **k)
                    .flat_map(|(t, _)| t.iter().copied())
                    .collect();
                attrs = attrs
                    .iter()
                    .zip(&keep)
                    .filter(|(_, k)| **k)
                    .map(|(a, _)| *a)
                    .collect();
            }
            if want_colors {
                attr_words.extend_from_slice(&attrs);
            }
            collector.push(&corners, ctx)?;
            done += chunk.len() as u64;
            ctx.progress(done as f32 / total_bytes as f32);
            Ok(())
        },
    )?;

    if dropped > 0 {
        report.warnings.push(IoWarning::NonFiniteDropped(dropped));
    }
    if collector.tri_count() == 0 {
        return Err(IoError::Empty);
    }
    let mut mesh = collector.finish(ctx)?;
    let mut materials = Vec::new();
    let mut face_materials = None;
    if want_colors
        && let Some((slots, faces)) = colors_to_slots(&attr_words, scheme)
        && faces.len() == mesh.face_count()
    {
        materials = slots;
        face_materials = Some(Arc::<[SlotIndex]>::from(faces));
    }
    mesh.groups = FaceGroups::new(mesh.tris.len());
    let mut obj = SceneObject::new(Arc::new(mesh));
    obj.face_materials = face_materials;
    Ok(Parts {
        objects: vec![obj],
        materials,
    })
}

// -------------------------------------------------------------------------------- ASCII

struct ObjectAcc {
    name: Option<String>,
    collector: Collector,
}

fn read_ascii(
    src: &Source<'_>,
    len: u64,
    opts: &ImportOptions,
    ctx: &JobCtx,
    tuning: Tuning,
    report: &mut ImportReport,
) -> Result<Parts, IoError> {
    let expected = (len / 250) as usize;
    let mk = |name: Option<String>| ObjectAcc {
        name,
        collector: Collector::new(opts.stl.weld, expected, &opts.limits),
    };
    let mut done: Vec<ObjectAcc> = Vec::new();
    let mut cur: Option<ObjectAcc> = None;
    let mut total_tris = 0u64;
    let (mut non_finite, mut polygons, mut short_loops) = (0u64, 0u64, 0u64);
    let mut line_base = 0u64;
    let mut carry: Vec<u8> = Vec::new();
    let mut read_bytes = 0u64;
    let threads = rayon::current_num_threads().max(1);

    let mut process = |data: &[u8],
                       cur: &mut Option<ObjectAcc>,
                       done: &mut Vec<ObjectAcc>|
     -> Result<(), IoError> {
        // Sub-blocks cut after an `endfacet`.
        let target = (data.len() / (threads * 4)).max(64 << 10);
        let mut ranges = Vec::new();
        let mut pos = 0;
        while pos < data.len() {
            let mut end = (pos + target).min(data.len());
            if end < data.len() {
                end = ascii::after_next_endfacet(data, end).unwrap_or(data.len());
            }
            ranges.push(pos..end);
            pos = end;
        }
        let blocks: Vec<ascii::Block> = ranges
            .par_iter()
            .map(|r| {
                if ctx.is_cancelled() {
                    ascii::Block::default()
                } else {
                    ascii::parse_block(&data[r.clone()])
                }
            })
            .collect();
        ctx.checkpoint()?;
        for b in blocks {
            if let Some((line, message)) = b.error {
                return Err(IoError::Malformed {
                    format: Format::Stl,
                    location: Location::Line(line_base + line),
                    message,
                });
            }
            non_finite += b.non_finite;
            polygons += b.polygons;
            short_loops += b.short_loops;
            let mut at = 0usize; // triangles of the block fed so far
            let feed = |from: usize,
                        to: usize,
                        cur: &mut Option<ObjectAcc>,
                        total: &mut u64|
             -> Result<(), IoError> {
                if to > from {
                    let acc = cur.get_or_insert_with(|| mk(None));
                    acc.collector.push(&b.corners[3 * from..3 * to], ctx)?;
                    *total += (to - from) as u64;
                    if *total > opts.limits.max_triangles {
                        return Err(IoError::LimitExceeded {
                            what: "triangles",
                            limit: opts.limits.max_triangles,
                            actual: *total,
                        });
                    }
                }
                Ok(())
            };
            for (ti, ev) in &b.events {
                feed(at, *ti, cur, &mut total_tris)?;
                at = *ti;
                if let Event::Solid(name) = ev {
                    let name = (!name.is_empty()).then(|| name.clone());
                    if opts.stl.split_solids {
                        if cur.as_ref().is_some_and(|a| a.collector.tri_count() > 0) {
                            done.extend(cur.take());
                        }
                        *cur = Some(mk(name));
                    } else if let Some(a) = cur.as_mut() {
                        a.name = a.name.take().or(name);
                    } else {
                        *cur = Some(mk(name));
                    }
                }
            }
            feed(at, b.corners.len() / 3, cur, &mut total_tris)?;
            line_base += b.lines;
        }
        Ok(())
    };

    stream(
        open_at(src, 0)?,
        u64::MAX,
        tuning.chunk_bytes,
        src.display_path(),
        |chunk| {
            ctx.checkpoint()?;
            read_bytes += chunk.len() as u64;
            let mut buf = std::mem::take(&mut carry);
            buf.extend_from_slice(&chunk);
            drop(chunk);
            match ascii::after_last_endfacet(&buf) {
                Some(cut) => {
                    process(&buf[..cut], &mut cur, &mut done)?;
                    carry = buf.split_off(cut);
                }
                None => {
                    if buf.len() > MAX_ASCII_CARRY {
                        return Err(IoError::Malformed {
                            format: Format::Stl,
                            location: Location::Byte(read_bytes - buf.len() as u64),
                            message: "no `endfacet` within 256 MiB".into(),
                        });
                    }
                    carry = buf;
                }
            }
            ctx.progress((read_bytes as f32 / len.max(1) as f32).min(1.0));
            Ok(())
        },
    )?;
    if !carry.is_empty() {
        let last = std::mem::take(&mut carry);
        process(&last, &mut cur, &mut done)?;
    }
    done.extend(cur.take());

    if non_finite > 0 {
        report
            .warnings
            .push(IoWarning::NonFiniteDropped(non_finite));
    }
    if polygons > 0 {
        report.warnings.push(IoWarning::PolygonTriangulated {
            polygons,
            fallback_fan: polygons,
        });
    }
    if short_loops > 0 {
        report.warnings.push(IoWarning::UnsupportedContentIgnored {
            what: "outer loop with fewer than three vertices".into(),
            count: short_loops,
        });
    }
    let mut objects = Vec::new();
    for acc in done {
        if acc.collector.tri_count() == 0 {
            continue;
        }
        let mesh = acc.collector.finish(ctx)?;
        let mut o = SceneObject::new(Arc::new(mesh));
        o.name = acc.name;
        objects.push(o);
    }
    if objects.is_empty() {
        return Err(IoError::Empty);
    }
    Ok(Parts {
        objects,
        materials: Vec::new(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use std::collections::BTreeSet;

    /// Corner coordinates from a small alphabet, so that welds are common. Includes `-0`.
    fn coord() -> impl Strategy<Value = f32> {
        prop::sample::select(vec![
            0.0f32, -0.0, 1.0, -1.0, 0.5, 2.25, 1e-3, 123.456, -7.0, 1e20,
        ])
    }

    fn tri() -> impl Strategy<Value = [f32; 9]> {
        prop::array::uniform9(coord())
    }

    fn norm(v: f32) -> u32 {
        let b = v.to_bits();
        if b == 0x8000_0000 { 0 } else { b }
    }

    fn binary(tris: &[[f32; 9]]) -> Vec<u8> {
        let mut v = vec![b' '; 80];
        v.extend_from_slice(&(tris.len() as u32).to_le_bytes());
        for t in tris {
            v.extend_from_slice(&[0u8; 12]);
            for f in t {
                v.extend_from_slice(&f.to_le_bytes());
            }
            v.extend_from_slice(&[0, 0]);
        }
        v
    }

    fn ascii(tris: &[[f32; 9]]) -> Vec<u8> {
        let mut s = String::from("solid p\n");
        for t in tris {
            s.push_str("facet normal 0 0 0\nouter loop\n");
            for c in t.chunks(3) {
                s.push_str(&format!("vertex {:e} {:e} {:e}\n", c[0], c[1], c[2]));
            }
            s.push_str("endloop\nendfacet\n");
        }
        s.push_str("endsolid p\n");
        s.into_bytes()
    }

    fn read_in_pool(bytes: &[u8], threads: usize, chunk: usize) -> Result<IndexedMesh, IoError> {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool");
        pool.install(|| {
            let i = read_tuned(
                Source::Bytes(bytes),
                &ImportOptions::default(),
                &JobCtx::detached(),
                Tuning { chunk_bytes: chunk },
            )?;
            Ok((*i.scene.objects[0].mesh).clone())
        })
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(48))]

        /// Done-when 3: the vertex count is the number of distinct normalised f32 positions
        /// (BTreeSet oracle), vertices are numbered in first-seen order, and 1 and 16 threads
        /// agree, for binary and ASCII input and for any chunking.
        #[test]
        fn exact_weld_matches_the_oracle(
            tris in prop::collection::vec(tri(), 1..300),
            chunk_recs in 1usize..40,
        ) {
            let mut seen = BTreeSet::new();
            let mut first_seen = Vec::new();
            for t in &tris {
                for c in t.chunks(3) {
                    let k = [norm(c[0]), norm(c[1]), norm(c[2])];
                    if seen.insert(k) {
                        first_seen.push(k);
                    }
                }
            }
            let bin = binary(&tris);
            let reference = read_in_pool(&bin, 1, 50 * chunk_recs).expect("read");
            prop_assert_eq!(reference.vertex_count(), seen.len());
            prop_assert_eq!(reference.face_count(), tris.len());
            for (p, k) in reference.positions.iter().zip(&first_seen) {
                prop_assert_eq!(
                    [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()],
                    [
                        f64::from(f32::from_bits(k[0])).to_bits(),
                        f64::from(f32::from_bits(k[1])).to_bits(),
                        f64::from(f32::from_bits(k[2])).to_bits(),
                    ]
                );
            }
            // Every face refers to the right position.
            for (f, t) in reference.tris.iter().zip(&tris) {
                for (corner, c) in f.iter().zip(t.chunks(3)) {
                    let p = reference.positions[*corner as usize];
                    prop_assert_eq!(
                        [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()],
                        [
                            (f64::from(c[0]) + 0.0).to_bits(),
                            (f64::from(c[1]) + 0.0).to_bits(),
                            (f64::from(c[2]) + 0.0).to_bits(),
                        ]
                    );
                }
            }
            let many = read_in_pool(&bin, 16, 50 * chunk_recs).expect("read");
            prop_assert_eq!(&many, &reference);
            // One big chunk gives the same mesh as many small ones.
            let whole = read_in_pool(&bin, 4, 1 << 20).expect("read");
            prop_assert_eq!(&whole, &reference);
            // ASCII with tiny chunks gives the same mesh as binary (the f32 values print and
            // parse back exactly).
            let txt = read_in_pool(&ascii(&tris), 16, 97 * chunk_recs).expect("read");
            prop_assert_eq!(&txt, &reference);
        }
    }
}
