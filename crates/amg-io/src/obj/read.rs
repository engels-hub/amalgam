//! OBJ reader (S12 §7, note §6): two-pass parallel parse of streamed chunks, negative
//! indices, polygons by ear clipping, vertex colours, `vt`, `o`/`g`, `usemtl` slots and MTL.

use std::collections::HashMap;
use std::ops::Range;
use std::path::{Component, Path};
use std::sync::Arc;
use std::time::Instant;

use amg_core::{Point2, Point3, Transform};
use amg_jobs::JobCtx;
use amg_mesh::{
    FaceGroupId, FaceGroupInfo, FaceGroups, GroupOrigin, IndexedMesh, MeshAttrs, Rgba8,
};
use rayon::prelude::*;

use super::mtl::{Mtl, parse_mtl};
use super::parse::{
    BAD, Base, Block, DEAD, Event, MAX_ITEMS, NO_VT, PENDING, PolyRec, Skipped, count_block,
    last_cut, next_cut, parse_block,
};
use super::triangulate::{Method, triangulate};
use crate::Imported;
use crate::endpoint::Source;
use crate::error::{ImportReport, IoError, IoWarning, Location};
use crate::format::Format;
use crate::options::{ImportOptions, ObjGroups};
use crate::scene::{MaterialSlot, NO_SLOT, Scene, SceneNode, SceneObject, SlotIndex};
use crate::stl::{count_degenerate, open_at, stream};

/// Longest stretch without a statement-ending newline that we are willing to buffer.
const MAX_CARRY: usize = 256 << 20;
/// Largest MTL file that is read.
const MAX_MTL: u64 = 64 << 20;
/// Colour of a material that the MTL does not define (the MTL default `Kd 0.8`).
const DEFAULT_COLOR: Rgba8 = Rgba8([204, 204, 204, 255]);

/// Knobs that tests turn; production uses the defaults.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Tuning {
    /// Bytes per streamed chunk.
    pub chunk_bytes: usize,
    /// Smallest parallel block.
    pub min_block: usize,
}

impl Default for Tuning {
    fn default() -> Self {
        Tuning {
            chunk_bytes: 16 << 20,
            min_block: 256 << 10,
        }
    }
}

/// Reads an OBJ file (and the MTL files it names, next to it) into a [`Scene`] whose numbers
/// are the file's. Units, up axis and `merge_objects` are applied by [`crate::import`].
///
/// # Errors
/// [`IoError::Empty`], [`IoError::Malformed`] (with the line), [`IoError::LimitExceeded`],
/// [`IoError::Read`] or [`IoError::Cancelled`].
pub fn read(src: Source<'_>, opts: &ImportOptions, ctx: &JobCtx) -> Result<Imported, IoError> {
    read_tuned(src, opts, ctx, Tuning::default())
}

/// Everything read so far, in file order, with global 0-based indices.
#[derive(Default)]
struct Acc {
    positions: Vec<Point3>,
    colors: Option<Vec<[f32; 4]>>,
    max_color: f32,
    uvs: Vec<[f64; 2]>,
    tri_v: Vec<[u32; 3]>,
    tri_vt: Option<Vec<[u32; 3]>>,
    poly_corners: Vec<[u32; 2]>,
    polys: Vec<PolyRec>,
    events: Vec<(usize, Event)>,
    skipped: Skipped,
    lines: u64,
}

impl Acc {
    fn absorb(&mut self, b: Block) {
        let tri_base = self.tri_v.len();
        let corner_base = self.poly_corners.len();
        let had_verts = self.positions.len();
        let nverts = b.verts.len();
        match (self.colors.as_mut(), b.colors) {
            (Some(g), Some(c)) => g.extend(c),
            (Some(g), None) => g.extend(std::iter::repeat_n([f32::NAN; 4], nverts)),
            (None, Some(c)) => {
                let mut g = vec![[f32::NAN; 4]; had_verts];
                g.extend(c);
                self.colors = Some(g);
            }
            (None, None) => {}
        }
        self.max_color = self.max_color.max(b.max_color);
        self.positions.extend(b.verts);
        self.uvs.extend(b.uvs);
        let ntri = b.tri_v.len();
        match (self.tri_vt.as_mut(), b.tri_vt) {
            (Some(g), Some(t)) => g.extend(t),
            (Some(g), None) => g.extend(std::iter::repeat_n([NO_VT; 3], ntri)),
            (None, Some(t)) => {
                let mut g = vec![[NO_VT; 3]; tri_base];
                g.extend(t);
                self.tri_vt = Some(g);
            }
            (None, None) => {}
        }
        self.tri_v.extend(b.tri_v);
        self.poly_corners.extend(b.poly_corners);
        self.polys.extend(b.polys.into_iter().map(|p| PolyRec {
            tri: p.tri + tri_base,
            start: p.start + corner_base,
            len: p.len,
        }));
        self.events
            .extend(b.events.into_iter().map(|(t, e)| (t + tri_base, e)));
        self.skipped.add(&b.skipped);
    }
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
    let mut report = ImportReport {
        format: Some(Format::Obj),
        bytes: len,
        ..ImportReport::default()
    };
    let mut acc = Acc::default();
    let mut carry: Vec<u8> = Vec::new();
    let mut read_bytes = 0u64;
    let mut first = true;

    stream(
        open_at(&src, 0)?,
        u64::MAX,
        tuning.chunk_bytes,
        src.display_path(),
        |chunk| {
            ctx.checkpoint()?;
            read_bytes += chunk.len() as u64;
            let mut buf = std::mem::take(&mut carry);
            buf.extend_from_slice(&chunk);
            drop(chunk);
            if first && buf.len() >= 3 {
                first = false;
                if buf.starts_with(b"\xEF\xBB\xBF") {
                    buf.drain(..3);
                }
            }
            match last_cut(&buf) {
                Some(cut) => {
                    process(&buf[..cut], &mut acc, opts, ctx, tuning)?;
                    carry = buf.split_off(cut);
                }
                None => {
                    if buf.len() > MAX_CARRY {
                        return Err(IoError::Malformed {
                            format: Format::Obj,
                            location: Location::Line(acc.lines + 1),
                            message: "no line break within 256 MiB".into(),
                        });
                    }
                    carry = buf;
                }
            }
            ctx.progress(0.9 * (read_bytes as f32 / len.max(1) as f32).min(1.0));
            Ok(())
        },
    )?;
    if !carry.is_empty() {
        let mut last = std::mem::take(&mut carry);
        if first && last.starts_with(b"\xEF\xBB\xBF") {
            last.drain(..3);
        }
        process(&last, &mut acc, opts, ctx, tuning)?;
    }
    ctx.checkpoint()?;

    let (objects, materials) = finish(acc, &src, opts, ctx, &mut report)?;
    ctx.progress(1.0);

    let mut scene = Scene {
        materials,
        ..Scene::default()
    };
    for (i, o) in objects.into_iter().enumerate() {
        report.vertices += o.mesh.vertex_count() as u64;
        report.degenerate_triangles += count_degenerate(&o.mesh);
        report.triangles_read += o.mesh.face_count() as u64;
        scene
            .roots
            .push(SceneNode::mesh(i as u32, Transform::IDENTITY));
        scene.objects.push(o);
    }
    report.duration = t0.elapsed();
    Ok(Imported { scene, report })
}

fn limit(what: &'static str, limit: u64, actual: u64) -> IoError {
    IoError::LimitExceeded {
        what,
        limit,
        actual,
    }
}

/// Splits `data` into blocks of about `target` bytes at statement boundaries.
fn split_blocks(data: &[u8], target: usize) -> Vec<Range<usize>> {
    let mut out = Vec::new();
    let mut pos = 0;
    while pos < data.len() {
        let want = pos.saturating_add(target);
        let end = if want >= data.len() {
            data.len()
        } else {
            next_cut(data, want).unwrap_or(data.len())
        };
        out.push(pos..end);
        pos = end;
    }
    out
}

/// Parses whole statements `data`: pass 1 counts, pass 2 parses, then the blocks are
/// appended to `acc` in order.
fn process(
    data: &[u8],
    acc: &mut Acc,
    opts: &ImportOptions,
    ctx: &JobCtx,
    tuning: Tuning,
) -> Result<(), IoError> {
    let threads = rayon::current_num_threads().max(1);
    let target = (data.len() / (threads * 4)).max(tuning.min_block);
    let ranges = split_blocks(data, target);
    let counts: Vec<_> = ranges
        .par_iter()
        .map(|r| {
            if ctx.is_cancelled() {
                Default::default()
            } else {
                count_block(&data[r.clone()])
            }
        })
        .collect();
    ctx.checkpoint()?;

    let lim = &opts.limits;
    let (tv, tf): (u64, u64) = counts.iter().fold((0, 0), |a, c| (a.0 + c.v, a.1 + c.f));
    let verts_after = acc.positions.len() as u64 + tv;
    if verts_after > lim.max_vertices.min(MAX_ITEMS) {
        return Err(limit(
            "vertices",
            lim.max_vertices.min(MAX_ITEMS),
            verts_after,
        ));
    }
    let tris_after = acc.tri_v.len() as u64 + tf;
    if tris_after > lim.max_triangles {
        return Err(limit("triangles", lim.max_triangles, tris_after));
    }

    // Prefix sums: what precedes every block.
    let mut bases = Vec::with_capacity(ranges.len());
    let (mut v, mut vt, mut line) = (
        acc.positions.len() as u64,
        acc.uvs.len() as u64,
        acc.lines + 1,
    );
    for c in &counts {
        bases.push(Base {
            v,
            vt,
            first_line: line,
        });
        v += c.v;
        vt += c.vt;
        line += c.lines;
    }
    let blocks: Vec<Block> = ranges
        .par_iter()
        .zip(bases.par_iter().zip(counts.par_iter()))
        .map(|(r, (base, c))| {
            if ctx.is_cancelled() {
                Block::default()
            } else {
                parse_block(&data[r.clone()], *base, *c)
            }
        })
        .collect();
    ctx.checkpoint()?;
    for b in blocks {
        if let Some((line, message)) = b.error {
            return Err(IoError::Malformed {
                format: Format::Obj,
                location: Location::Line(line),
                message,
            });
        }
        acc.absorb(b);
    }
    acc.lines = line - 1;
    if acc.tri_v.len() as u64 > lim.max_triangles {
        return Err(limit(
            "triangles",
            lim.max_triangles,
            acc.tri_v.len() as u64,
        ));
    }
    Ok(())
}

// ------------------------------------------------------------------------ after the parse

/// A run of triangles that share an object, face group and material.
struct Run {
    range: Range<usize>,
    group: Option<u32>,
    slot: SlotIndex,
}

#[derive(Default)]
struct ObjDef {
    name: Option<String>,
    runs: Vec<Run>,
    group_names: Vec<String>,
    group_ix: HashMap<String, u32>,
    unnamed_faces: bool,
}

/// The `o` / `g` / `usemtl` state machine over the event list.
struct Walker {
    mode: ObjGroups,
    objs: Vec<ObjDef>,
    cur: Option<usize>,
    /// Name of the object to start next, and whether it came from `o`.
    pending: Option<(String, bool)>,
    group: Option<String>,
    slot: SlotIndex,
    cursor: usize,
    slot_names: Vec<String>,
    slot_ix: HashMap<String, SlotIndex>,
    mtllibs: Vec<String>,
    empty_objects: u64,
}

impl Walker {
    fn flush(&mut self, upto: usize) {
        if upto <= self.cursor {
            return;
        }
        let oi = match self.cur {
            Some(i) => i,
            None => {
                let name = self.pending.take().map(|p| p.0).filter(|n| !n.is_empty());
                self.objs.push(ObjDef {
                    name,
                    ..ObjDef::default()
                });
                self.cur = Some(self.objs.len() - 1);
                self.objs.len() - 1
            }
        };
        let o = &mut self.objs[oi];
        let group = match (&self.mode, &self.group) {
            (ObjGroups::FaceGroups, Some(g)) => {
                let next = o.group_names.len() as u32;
                let ix = *o.group_ix.entry(g.clone()).or_insert(next);
                if ix == next {
                    o.group_names.push(g.clone());
                }
                Some(ix)
            }
            _ => {
                o.unnamed_faces = true;
                None
            }
        };
        match o.runs.last_mut() {
            Some(r) if r.range.end == self.cursor && r.group == group && r.slot == self.slot => {
                r.range.end = upto;
            }
            _ => o.runs.push(Run {
                range: self.cursor..upto,
                group,
                slot: self.slot,
            }),
        }
        self.cursor = upto;
    }

    fn event(&mut self, pos: usize, e: Event) {
        self.flush(pos);
        match e {
            Event::Object(name) => {
                if self.mode == ObjGroups::Ignore {
                    return;
                }
                if self.pending.as_ref().is_some_and(|p| p.1) {
                    self.empty_objects += 1;
                }
                self.cur = None;
                self.pending = Some((name, true));
                self.group = None;
            }
            Event::Group(name) => {
                let name = if name == "default" {
                    String::new()
                } else {
                    name
                };
                match self.mode {
                    ObjGroups::Ignore => {}
                    ObjGroups::FaceGroups => {
                        self.group = (!name.is_empty()).then_some(name);
                    }
                    ObjGroups::Objects => {
                        if self.cur.is_some() {
                            self.cur = None;
                            self.pending = Some((name, false));
                        } else if !self.pending.as_ref().is_some_and(|p| p.1) {
                            self.pending = Some((name, false));
                        }
                    }
                }
            }
            Event::UseMtl(name) => {
                self.slot = if matches!(name.as_str(), "" | "off" | "(null)") {
                    NO_SLOT
                } else if let Some(&s) = self.slot_ix.get(&name) {
                    s
                } else {
                    // More distinct materials than `SlotIndex` can number share the last slot.
                    let s = if self.slot_names.len() < usize::from(NO_SLOT) {
                        self.slot_names.push(name.clone());
                        (self.slot_names.len() - 1) as SlotIndex
                    } else {
                        NO_SLOT - 1
                    };
                    self.slot_ix.insert(name, s);
                    s
                };
            }
            Event::MtlLib(name) => self.mtllibs.push(name),
        }
    }
}

fn to_byte(v: f32, scale: f32) -> u8 {
    if v.is_nan() {
        return 255;
    }
    // Clamped to [0, 255] before the cast.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    {
        ((v * scale).clamp(0.0, 1.0) * 255.0).round() as u8
    }
}

/// Safe relative path: no root, drive, UNC prefix or `..`.
fn safe_relative(p: &Path) -> bool {
    !p.as_os_str().is_empty()
        && p.components()
            .all(|c| matches!(c, Component::Normal(_) | Component::CurDir))
}

fn load_mtls(
    names: &[String],
    src: &Source<'_>,
    warnings: &mut Vec<IoWarning>,
    textures: &mut u64,
) -> Mtl {
    let mut all = Mtl::default();
    for name in names {
        let missing = |why: &str, warnings: &mut Vec<IoWarning>| {
            warnings.push(IoWarning::MissingReference {
                what: format!("mtllib {name} ({why})"),
            });
        };
        let Source::Path(obj_path) = src else {
            missing("no folder to look in for an in-memory file", warnings);
            continue;
        };
        let dir = obj_path.parent().unwrap_or(Path::new(""));
        // `mtllib` may list several files, or one name with spaces.
        let mut candidates = vec![name.clone()];
        if name.contains([' ', '\t']) {
            candidates.extend(name.split_whitespace().map(str::to_owned));
        }
        let mut found = false;
        for cand in &candidates {
            let rel = Path::new(cand);
            if !safe_relative(rel) {
                continue;
            }
            let full = dir.join(rel);
            let Ok(meta) = std::fs::metadata(&full) else {
                continue;
            };
            if !meta.is_file() {
                continue;
            }
            if meta.len() > MAX_MTL {
                missing("file is larger than 64 MiB", warnings);
                found = true;
                continue;
            }
            match std::fs::read(&full) {
                Ok(bytes) => {
                    let m = parse_mtl(&bytes);
                    *textures += m.textures_ignored;
                    all.materials.extend(m.materials);
                    found = true;
                }
                Err(_) => {
                    missing("cannot be read", warnings);
                    found = true;
                }
            }
        }
        if !found {
            missing("not found next to the OBJ", warnings);
        }
    }
    all
}

/// The triangles of one object, with optional per-face attributes.
struct ObjTris {
    tris: Vec<[u32; 3]>,
    vt: Option<Vec<[u32; 3]>>,
    slots: Option<Vec<SlotIndex>>,
    /// Index into the object's group names, `BAD` = unnamed.
    groups: Option<Vec<u32>>,
}

struct PolyOut {
    /// 0 = ear clipped, 1 = fan, 2 = bad index, 3 = non-finite.
    status: Vec<u8>,
    tris: Vec<[u32; 3]>,
    vts: Vec<[u32; 3]>,
}

fn triangulate_polys(acc: &Acc, polys: &[PolyRec], want_vt: bool, nv: usize) -> PolyOut {
    let mut out = PolyOut {
        status: Vec::with_capacity(polys.len()),
        tris: Vec::new(),
        vts: Vec::new(),
    };
    let mut pts: Vec<Point3> = Vec::new();
    let mut local: Vec<[u32; 3]> = Vec::new();
    for p in polys {
        let corners = &acc.poly_corners[p.start..p.start + p.len as usize];
        if corners.iter().any(|c| c[0] as usize >= nv) {
            out.status.push(2);
            continue;
        }
        pts.clear();
        pts.extend(corners.iter().map(|c| acc.positions[c[0] as usize]));
        if !pts.iter().all(|q| q.is_finite()) {
            out.status.push(3);
            continue;
        }
        local.clear();
        let m = triangulate(&pts, &mut local);
        out.status.push(u8::from(m == Method::Fan));
        for t in &local {
            out.tris.push(t.map(|i| corners[i as usize][0]));
            if want_vt {
                out.vts.push(t.map(|i| corners[i as usize][1]));
            }
        }
    }
    out
}

#[allow(clippy::too_many_lines)]
fn finish(
    mut acc: Acc,
    src: &Source<'_>,
    opts: &ImportOptions,
    ctx: &JobCtx,
    report: &mut ImportReport,
) -> Result<(Vec<SceneObject>, Vec<MaterialSlot>), IoError> {
    let nv = acc.positions.len();
    let nvt = acc.uvs.len();
    let mut invalid = 0u64;
    let mut non_finite = 0u64;

    // 1. Triangulate polygons into their reserved slots.
    let (mut polygons, mut fans) = (0u64, 0u64);
    if !acc.polys.is_empty() {
        let want_vt = acc.tri_vt.is_some();
        let outs: Vec<PolyOut> = acc
            .polys
            .par_chunks(2048)
            .map(|c| {
                if ctx.is_cancelled() {
                    PolyOut {
                        status: vec![2; c.len()],
                        tris: Vec::new(),
                        vts: Vec::new(),
                    }
                } else {
                    triangulate_polys(&acc, c, want_vt, nv)
                }
            })
            .collect();
        ctx.checkpoint()?;
        let polys = std::mem::take(&mut acc.polys);
        let mut pi = 0;
        for o in outs {
            let (mut at, mut vat) = (0, 0);
            for &st in &o.status {
                let p = polys[pi];
                pi += 1;
                let k = p.len as usize - 2;
                match st {
                    0 | 1 => {
                        polygons += 1;
                        fans += u64::from(st == 1);
                        acc.tri_v[p.tri..p.tri + k].copy_from_slice(&o.tris[at..at + k]);
                        at += k;
                        if let Some(vt) = acc.tri_vt.as_mut() {
                            vt[p.tri..p.tri + k].copy_from_slice(&o.vts[vat..vat + k]);
                            vat += k;
                        }
                    }
                    _ => {
                        if st == 2 {
                            invalid += 1;
                        } else {
                            non_finite += 1;
                        }
                        acc.tri_v[p.tri..p.tri + k].fill([DEAD; 3]);
                    }
                }
            }
        }
    }
    debug_assert!(acc.tri_v.iter().all(|t| t[0] != PENDING));

    // 2. Validate triangles: indices in range, finite corners.
    let finite_flags: Option<Vec<bool>> = (!acc.positions.par_iter().all(|p| p.is_finite()))
        .then(|| acc.positions.par_iter().map(|p| p.is_finite()).collect());
    let verdict = |t: &[u32; 3]| -> u8 {
        if t[0] == DEAD {
            return 3;
        }
        if t.iter().any(|&i| i as usize >= nv) {
            return 1;
        }
        if let Some(f) = &finite_flags
            && t.iter().any(|&i| !f[i as usize])
        {
            return 2;
        }
        0
    };
    let (bad_idx, bad_fin, dead) = acc
        .tri_v
        .par_iter()
        .fold(
            || (0u64, 0u64, 0u64),
            |mut a, t| {
                match verdict(t) {
                    1 => a.0 += 1,
                    2 => a.1 += 1,
                    3 => a.2 += 1,
                    _ => {}
                }
                a
            },
        )
        .reduce(|| (0, 0, 0), |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2));
    invalid += bad_idx;
    non_finite += bad_fin;
    let keep: Option<Vec<bool>> = (bad_idx + bad_fin + dead > 0)
        .then(|| acc.tri_v.par_iter().map(|t| verdict(t) == 0).collect());
    ctx.checkpoint()?;

    // 3. Events -> objects, face groups, material slots.
    let mut w = Walker {
        mode: opts.obj.groups_as,
        objs: Vec::new(),
        cur: None,
        pending: None,
        group: None,
        slot: NO_SLOT,
        cursor: 0,
        slot_names: Vec::new(),
        slot_ix: HashMap::new(),
        mtllibs: Vec::new(),
        empty_objects: 0,
    };
    for (pos, e) in std::mem::take(&mut acc.events) {
        w.event(pos, e);
    }
    w.flush(acc.tri_v.len());
    if w.pending.as_ref().is_some_and(|p| p.1) {
        w.empty_objects += 1;
    }

    // 4. Warnings and materials.
    let mut warnings = Vec::new();
    if non_finite > 0 {
        warnings.push(IoWarning::NonFiniteDropped(non_finite));
    }
    if invalid > 0 {
        warnings.push(IoWarning::InvalidIndexDropped(invalid));
    }
    if polygons > 0 {
        warnings.push(IoWarning::PolygonTriangulated {
            polygons,
            fallback_fan: fans,
        });
    }
    let sk = acc.skipped;
    for (what, count) in [
        ("face with fewer than three vertices", sk.short_faces),
        ("line elements (l)", sk.lines),
        ("point elements (p)", sk.points),
        ("curves and surfaces", sk.freeform),
        ("unrecognised statements", sk.unknown),
        ("object without faces", w.empty_objects),
    ] {
        if count > 0 {
            warnings.push(IoWarning::UnsupportedContentIgnored {
                what: what.to_owned(),
                count,
            });
        }
    }
    let mut slots: Vec<MaterialSlot> = w
        .slot_names
        .iter()
        .map(|n| MaterialSlot {
            name: n.clone(),
            color: DEFAULT_COLOR,
            filament: None,
        })
        .collect();
    if opts.obj.load_mtl && !w.mtllibs.is_empty() {
        let mut textures = 0;
        let mtl = load_mtls(&w.mtllibs, src, &mut warnings, &mut textures);
        if textures > 0 {
            warnings.push(IoWarning::UnsupportedContentIgnored {
                what: "textures (map_*)".to_owned(),
                count: textures,
            });
        }
        let mut undefined = 0u64;
        for s in &mut slots {
            match mtl.get(&s.name) {
                Some(m) => s.color = m.color,
                None => undefined += 1,
            }
        }
        if undefined > 0 && !mtl.materials.is_empty() {
            warnings.push(IoWarning::MissingReference {
                what: format!("{undefined} material(s) used but not defined in the MTL"),
            });
        }
    }
    report.warnings.extend(warnings);

    // 5. Build one mesh per object.
    let colors: Option<Vec<Rgba8>> = acc.colors.take().map(|c| {
        let scale = if acc.max_color > 1.0 + 1e-6 {
            1.0 / 255.0
        } else {
            1.0
        };
        c.par_iter()
            .map(|c| {
                Rgba8([
                    to_byte(c[0], scale),
                    to_byte(c[1], scale),
                    to_byte(c[2], scale),
                    to_byte(c[3], scale),
                ])
            })
            .collect()
    });
    let multi = w.objs.len() > 1;
    let mut remap: Vec<u32> = if multi { vec![0; nv] } else { Vec::new() };
    let mut positions = Some(std::mem::take(&mut acc.positions));
    let mut colors = colors;
    let mut tri_all = Some(std::mem::take(&mut acc.tri_v));
    let mut vt_all = acc.tri_vt.take();
    let total_tris = tri_all.as_ref().map_or(0, Vec::len);
    let mut objects = Vec::new();
    for (oi, o) in w.objs.iter().enumerate() {
        ctx.checkpoint()?;
        let whole =
            !multi && keep.is_none() && o.runs.len() == 1 && o.runs[0].range == (0..total_tris);
        // Triangles of this object, with slot and group per face (only when they vary or
        // are set).
        let ObjTris {
            mut tris,
            vt: mut vt_o,
            slots: slot_of,
            groups: group_of,
        } = if whole {
            let n = tri_all.as_ref().map_or(0, Vec::len);
            let r = &o.runs[0];
            ObjTris {
                tris: tri_all.take().unwrap_or_default(),
                vt: vt_all.take(),
                slots: (r.slot != NO_SLOT).then(|| vec![r.slot; n]),
                groups: r.group.map(|g| vec![g; n]),
            }
        } else {
            let tri_src = tri_all.as_deref().unwrap_or(&[]);
            let want_slots = o.runs.iter().any(|r| r.slot != NO_SLOT);
            let want_groups = o.runs.iter().any(|r| r.group.is_some());
            let mut out = ObjTris {
                tris: Vec::new(),
                vt: vt_all.as_ref().map(|_| Vec::new()),
                slots: want_slots.then(Vec::new),
                groups: want_groups.then(Vec::new),
            };
            for r in &o.runs {
                let before = out.tris.len();
                for i in r.range.clone() {
                    if keep.as_ref().is_none_or(|k| k[i]) {
                        out.tris.push(tri_src[i]);
                        if let (Some(vt), Some(all)) = (out.vt.as_mut(), vt_all.as_ref()) {
                            vt.push(all[i]);
                        }
                    }
                }
                let n = out.tris.len() - before;
                if let Some(s) = out.slots.as_mut() {
                    s.extend(std::iter::repeat_n(r.slot, n));
                }
                if let Some(g) = out.groups.as_mut() {
                    g.extend(std::iter::repeat_n(r.group.unwrap_or(BAD), n));
                }
            }
            out
        };
        if tris.is_empty() {
            continue;
        }

        // Vertices: all of them for a single object, else the ones this object uses.
        let (pos_o, col_o) = if multi {
            let mut used: Vec<u32> = tris.iter().flatten().copied().collect();
            used.par_sort_unstable();
            used.dedup();
            for (k, &g) in used.iter().enumerate() {
                remap[g as usize] = k as u32;
            }
            for t in &mut tris {
                for i in t {
                    *i = remap[*i as usize];
                }
            }
            let all_pos = positions.as_deref().unwrap_or(&[]);
            let p: Vec<Point3> = used.iter().map(|&g| all_pos[g as usize]).collect();
            let c = colors
                .as_ref()
                .map(|c| used.iter().map(|&g| c[g as usize]).collect::<Vec<_>>());
            (p, c)
        } else {
            (positions.take().unwrap_or_default(), colors.take())
        };

        let corner_uvs = vt_o.take().and_then(|vts| {
            let uv = |i: u32| {
                acc.uvs
                    .get(i as usize)
                    .map_or(Point2::ZERO, |u| Point2::new(u[0], u[1]))
            };
            let any = vts
                .par_iter()
                .any(|t| t.iter().any(|&i| (i as usize) < nvt));
            any.then(|| vts.par_iter().map(|t| t.map(uv)).collect::<Vec<_>>())
        });

        let n = tris.len();
        let mut groups = FaceGroups::new(n);
        if !o.group_names.is_empty() {
            let mut ids: Vec<FaceGroupId> = Vec::with_capacity(o.group_names.len());
            for (k, name) in o.group_names.iter().enumerate() {
                let info = FaceGroupInfo {
                    name: Some(name.clone()),
                    origin: GroupOrigin::Import { object: oi as u32 },
                    primitive: None,
                };
                if k == 0 && !o.unnamed_faces {
                    // No unnamed faces: the default group becomes the first named one.
                    if let Some(d) = groups.info_mut(FaceGroupId(0)) {
                        *d = info;
                    }
                    ids.push(FaceGroupId(0));
                } else {
                    ids.push(groups.try_alloc(info)?);
                }
            }
            for (f, &g) in group_of.iter().flatten().enumerate() {
                if g != BAD {
                    groups.set(amg_mesh::FaceId(f as u32), ids[g as usize]);
                }
            }
        }
        let mesh = IndexedMesh {
            positions: pos_o,
            tris,
            attrs: MeshAttrs {
                vertex_colors: col_o,
                corner_uvs,
                ..MeshAttrs::default()
            },
            groups,
        };
        mesh.validate()?;
        let mut obj = SceneObject::new(Arc::new(mesh));
        obj.name = o.name.clone();
        obj.source_id = o.name.clone();
        obj.face_materials = slot_of.map(Arc::from);
        objects.push(obj);
    }
    if objects.is_empty() {
        return Err(IoError::Empty);
    }
    Ok((objects, slots))
}
