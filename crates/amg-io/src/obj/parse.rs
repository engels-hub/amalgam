//! OBJ statement scanner and block parser (note §6).
//!
//! A *block* is a run of whole statements (cut after a newline that does not continue the
//! line). Pass 1 counts what a block defines; pass 2 parses it knowing how many vertices and
//! texture coordinates precede it, so negative (relative) indices resolve inside the block.

use amg_core::Point3;

/// An index token that is zero, out of range for negative indices, or too large.
pub(crate) const BAD: u32 = u32::MAX;
/// "No texture coordinate" in a corner.
pub(crate) const NO_VT: u32 = u32::MAX;
/// Placeholder of a polygon's triangles until the polygon is triangulated.
pub(crate) const PENDING: u32 = u32::MAX - 1;
/// Placeholder of a polygon that was dropped (and already counted).
pub(crate) const DEAD: u32 = u32::MAX - 2;
/// Largest vertex / uv count we accept (leaves room for the markers above).
pub(crate) const MAX_ITEMS: u64 = (u32::MAX - 8) as u64;

/// Statement keyword class.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Kw {
    Blank,
    V,
    Vt,
    Vn,
    F,
    O,
    G,
    UseMtl,
    MtlLib,
    /// `l`
    Line,
    /// `p`
    Point,
    /// Curves, surfaces and their parameter space.
    Freeform,
    /// `s`, `mg`, `usemap`, `maplib`, ... (silently ignored).
    Quiet,
    Unknown,
}

#[inline]
fn is_ws(b: u8) -> bool {
    b == b' ' || b == b'\t'
}

/// Splits a statement into its keyword class and the text after the keyword.
pub(crate) fn classify(line: &[u8]) -> (Kw, &[u8]) {
    let mut i = 0;
    while i < line.len() && is_ws(line[i]) {
        i += 1;
    }
    let s = &line[i..];
    match s.first() {
        None | Some(b'#') => return (Kw::Blank, &[]),
        _ => {}
    }
    let end = s.iter().position(|&b| is_ws(b)).unwrap_or(s.len());
    let (kw, rest) = s.split_at(end);
    let k = match kw {
        b"v" => Kw::V,
        b"vt" => Kw::Vt,
        b"vn" => Kw::Vn,
        b"f" => Kw::F,
        b"o" => Kw::O,
        b"g" => Kw::G,
        b"usemtl" => Kw::UseMtl,
        b"mtllib" => Kw::MtlLib,
        b"l" => Kw::Line,
        b"p" => Kw::Point,
        b"vp" | b"cstype" | b"deg" | b"bmat" | b"step" | b"curv" | b"curv2" | b"surf" | b"parm"
        | b"trim" | b"hole" | b"scrv" | b"sp" | b"end" | b"con" => Kw::Freeform,
        b"s" | b"mg" | b"usemap" | b"maplib" | b"bevel" | b"c_interp" | b"d_interp" | b"lod"
        | b"shadow_obj" | b"trace_obj" | b"ctech" | b"stech" | b"call" | b"csh" => Kw::Quiet,
        _ => Kw::Unknown,
    };
    (k, rest)
}

/// Does the physical line `raw` (without its `\n`) continue on the next one?
#[inline]
pub(crate) fn continues(raw: &[u8]) -> bool {
    let raw = raw.strip_suffix(b"\r").unwrap_or(raw);
    raw.last() == Some(&b'\\')
}

/// Calls `f(line_number, statement)` for every statement of `data` (continuation lines are
/// joined with a space; a trailing `\r` is dropped). `first_line` is the 1-based number of
/// the first physical line. Returns the number of physical lines.
pub(crate) fn for_each_stmt<E>(
    data: &[u8],
    first_line: u64,
    mut f: impl FnMut(u64, &[u8]) -> Result<(), E>,
) -> Result<u64, E> {
    let mut scratch: Vec<u8> = Vec::new();
    let mut pos = 0usize;
    let mut line = first_line;
    let phys = |pos: usize| -> (&[u8], usize) {
        let end = data[pos..]
            .iter()
            .position(|&b| b == b'\n')
            .map_or(data.len(), |p| pos + p);
        (&data[pos..end], end)
    };
    while pos < data.len() {
        let stmt_line = line;
        let (raw, end) = phys(pos);
        pos = end + 1;
        line += 1;
        if continues(raw) {
            scratch.clear();
            let mut cur = raw;
            loop {
                let cur_nocr = cur.strip_suffix(b"\r").unwrap_or(cur);
                if let Some(head) = cur_nocr.strip_suffix(b"\\") {
                    scratch.extend_from_slice(head);
                    scratch.push(b' ');
                    if pos >= data.len() {
                        break;
                    }
                    let (next, e) = phys(pos);
                    pos = e + 1;
                    line += 1;
                    cur = next;
                } else {
                    scratch.extend_from_slice(cur_nocr);
                    break;
                }
            }
            f(stmt_line, &scratch)?;
        } else {
            f(stmt_line, raw.strip_suffix(b"\r").unwrap_or(raw))?;
        }
    }
    Ok(line - first_line)
}

/// Position just after the last newline of `buf` that ends a statement, if any.
pub(crate) fn last_cut(buf: &[u8]) -> Option<usize> {
    let mut end = buf.len();
    while let Some(p) = buf[..end].iter().rposition(|&b| b == b'\n') {
        let line_start = buf[..p]
            .iter()
            .rposition(|&b| b == b'\n')
            .map_or(0, |q| q + 1);
        if !continues(&buf[line_start..p]) {
            return Some(p + 1);
        }
        end = p;
    }
    None
}

/// Position just after the first statement-ending newline at or after `from`, or `None`.
pub(crate) fn next_cut(buf: &[u8], from: usize) -> Option<usize> {
    let mut at = from;
    while at < buf.len() {
        let p = at + buf[at..].iter().position(|&b| b == b'\n')?;
        let line_start = buf[..p]
            .iter()
            .rposition(|&b| b == b'\n')
            .map_or(0, |q| q + 1);
        if !continues(&buf[line_start..p]) {
            return Some(p + 1);
        }
        at = p + 1;
    }
    None
}

/// What pass 1 learns about a block.
#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct Counts {
    pub lines: u64,
    pub v: u64,
    pub vt: u64,
    pub f: u64,
}

pub(crate) fn count_block(data: &[u8]) -> Counts {
    let mut c = Counts::default();
    let lines = for_each_stmt::<()>(data, 1, |_, s| {
        match classify(s).0 {
            Kw::V => c.v += 1,
            Kw::Vt => c.vt += 1,
            Kw::F => c.f += 1,
            _ => {}
        }
        Ok(())
    });
    c.lines = lines.unwrap_or(0);
    c
}

/// A statement that changes the state faces are read in.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Event {
    Object(String),
    Group(String),
    UseMtl(String),
    MtlLib(String),
}

/// A polygon with more than three corners; its `len - 2` triangles start at `tri`.
#[derive(Copy, Clone, Debug)]
pub(crate) struct PolyRec {
    pub tri: usize,
    pub start: usize,
    pub len: u32,
}

/// Counts of statements that are skipped, for warnings.
#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct Skipped {
    pub lines: u64,
    pub points: u64,
    pub freeform: u64,
    pub unknown: u64,
    pub short_faces: u64,
}

impl Skipped {
    pub(crate) fn add(&mut self, o: &Skipped) {
        self.lines += o.lines;
        self.points += o.points;
        self.freeform += o.freeform;
        self.unknown += o.unknown;
        self.short_faces += o.short_faces;
    }
}

/// Everything a block defines. Indices in `tri_v` / `poly_corners` are 0-based and global.
#[derive(Debug, Default)]
pub(crate) struct Block {
    pub verts: Vec<Point3>,
    /// Raw RGBA as written (0-1 or 0-255; NaN = not given); `len == verts.len()` when present.
    pub colors: Option<Vec<[f32; 4]>>,
    pub max_color: f32,
    pub uvs: Vec<[f64; 2]>,
    pub tri_v: Vec<[u32; 3]>,
    /// `len == tri_v.len()` when present.
    pub tri_vt: Option<Vec<[u32; 3]>>,
    pub poly_corners: Vec<[u32; 2]>,
    pub polys: Vec<PolyRec>,
    /// (index of the next triangle, event), in file order.
    pub events: Vec<(usize, Event)>,
    pub skipped: Skipped,
    /// First error: (line, message).
    pub error: Option<(u64, String)>,
}

/// What precedes a block.
#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct Base {
    pub v: u64,
    pub vt: u64,
    pub first_line: u64,
}

fn parse_f64(tok: &[u8]) -> Option<f64> {
    std::str::from_utf8(tok).ok()?.parse::<f64>().ok()
}

/// Signed decimal integer; `None` for anything else (including an empty token).
fn parse_i64(tok: &[u8]) -> Option<i64> {
    let (neg, digits) = match tok.first()? {
        b'-' => (true, &tok[1..]),
        b'+' => (false, &tok[1..]),
        _ => (false, tok),
    };
    if digits.is_empty() {
        return None;
    }
    let mut v: i64 = 0;
    for &d in digits {
        if !d.is_ascii_digit() {
            return None;
        }
        v = v.saturating_mul(10).saturating_add(i64::from(d - b'0'));
    }
    Some(if neg { -v } else { v })
}

/// Resolves a 1-based or negative index against `count` items defined so far.
#[inline]
fn resolve(i: i64, count: u64) -> u32 {
    let abs = if i > 0 {
        (i - 1) as u64
    } else if i < 0 {
        match count.checked_sub(i.unsigned_abs()) {
            Some(a) => a,
            None => return BAD,
        }
    } else {
        return BAD;
    };
    if abs < MAX_ITEMS { abs as u32 } else { BAD }
}

fn tokens(s: &[u8]) -> impl Iterator<Item = &[u8]> {
    s.split(|&b| is_ws(b)).filter(|t| !t.is_empty())
}

fn name_of(rest: &[u8]) -> String {
    let s = String::from_utf8_lossy(rest);
    s.trim_matches([' ', '\t', '\r']).to_owned()
}

/// Parses one block (pass 2).
pub(crate) fn parse_block(data: &[u8], base: Base, counts: Counts) -> Block {
    let mut b = Block::default();
    b.verts.reserve(counts.v as usize);
    b.uvs.reserve(counts.vt as usize);
    b.tri_v.reserve(counts.f as usize);
    let mut corners: Vec<[u32; 2]> = Vec::new();
    let r = for_each_stmt(data, base.first_line, |line, stmt| {
        let (kw, rest) = classify(stmt);
        match kw {
            Kw::Blank | Kw::Vn | Kw::Quiet => {}
            Kw::V => vertex(&mut b, rest).map_err(|m| (line, m))?,
            Kw::Vt => {
                let mut t = tokens(rest);
                let u = t.next().and_then(parse_f64);
                let v = t.next().map(parse_f64);
                match (u, v) {
                    (Some(u), Some(Some(v))) => b.uvs.push([u, v]),
                    (Some(u), None) => b.uvs.push([u, 0.0]),
                    _ => return Err((line, "texture coordinate needs numbers".to_owned())),
                }
            }
            Kw::F => face(&mut b, rest, base, &mut corners).map_err(|m| (line, m))?,
            Kw::O => b.events.push((b.tri_v.len(), Event::Object(name_of(rest)))),
            Kw::G => b.events.push((b.tri_v.len(), Event::Group(name_of(rest)))),
            Kw::UseMtl => b.events.push((b.tri_v.len(), Event::UseMtl(name_of(rest)))),
            Kw::MtlLib => b.events.push((b.tri_v.len(), Event::MtlLib(name_of(rest)))),
            Kw::Line => b.skipped.lines += 1,
            Kw::Point => b.skipped.points += 1,
            Kw::Freeform => b.skipped.freeform += 1,
            Kw::Unknown => b.skipped.unknown += 1,
        }
        Ok(())
    });
    if let Err(e) = r {
        b.error = Some(e);
    }
    b
}

fn vertex(b: &mut Block, rest: &[u8]) -> Result<(), String> {
    let mut n = [0.0f64; 7];
    let mut k = 0;
    for t in tokens(rest) {
        if k == 7 {
            break;
        }
        n[k] = parse_f64(t)
            .ok_or_else(|| format!("`{}` is not a number", String::from_utf8_lossy(t)))?;
        k += 1;
    }
    if k < 3 {
        return Err("vertex needs three coordinates".to_owned());
    }
    let have = b.verts.len();
    b.verts.push(Point3::new(n[0], n[1], n[2]));
    // `v x y z w` has a weight; `v x y z r g b [a]` has a colour.
    if k >= 6 {
        let c = b.colors.get_or_insert_with(|| vec![[f32::NAN; 4]; have]);
        let a = if k >= 7 { n[6] as f32 } else { f32::NAN };
        let rgba = [n[3] as f32, n[4] as f32, n[5] as f32, a];
        for x in rgba {
            if x.is_finite() {
                b.max_color = b.max_color.max(x);
            }
        }
        c.push(rgba);
    } else if let Some(c) = b.colors.as_mut() {
        c.push([f32::NAN; 4]);
    }
    Ok(())
}

/// One `v`, `v/vt`, `v//vn` or `v/vt/vn` token.
fn corner(tok: &[u8], cur_v: u64, cur_vt: u64) -> Result<[u32; 2], String> {
    let bad = || {
        format!(
            "`{}` is not a vertex reference",
            String::from_utf8_lossy(tok)
        )
    };
    let mut parts = tok.splitn(3, |&c| c == b'/');
    let v = parse_i64(parts.next().unwrap_or(&[])).ok_or_else(bad)?;
    let vt = match parts.next() {
        None | Some([]) => NO_VT,
        Some(t) => resolve(parse_i64(t).ok_or_else(bad)?, cur_vt),
    };
    if let Some(n) = parts.next()
        && !n.is_empty()
    {
        parse_i64(n).ok_or_else(bad)?;
    }
    Ok([resolve(v, cur_v), vt])
}

fn face(b: &mut Block, rest: &[u8], base: Base, corners: &mut Vec<[u32; 2]>) -> Result<(), String> {
    let cur_v = base.v + b.verts.len() as u64;
    let cur_vt = base.vt + b.uvs.len() as u64;
    corners.clear();
    for t in tokens(rest) {
        corners.push(corner(t, cur_v, cur_vt)?);
    }
    let n = corners.len();
    if n < 3 {
        b.skipped.short_faces += 1;
        return Ok(());
    }
    let has_vt = corners.iter().any(|c| c[1] != NO_VT);
    let tris_before = b.tri_v.len();
    if has_vt && b.tri_vt.is_none() {
        b.tri_vt = Some(vec![[NO_VT; 3]; tris_before]);
    }
    if n == 3 {
        b.tri_v.push([corners[0][0], corners[1][0], corners[2][0]]);
        if let Some(t) = b.tri_vt.as_mut() {
            t.push([corners[0][1], corners[1][1], corners[2][1]]);
        }
    } else {
        let k = n - 2;
        b.tri_v.extend(std::iter::repeat_n([PENDING; 3], k));
        if let Some(t) = b.tri_vt.as_mut() {
            t.extend(std::iter::repeat_n([NO_VT; 3], k));
        }
        b.polys.push(PolyRec {
            tri: tris_before,
            start: b.poly_corners.len(),
            len: n as u32,
        });
        b.poly_corners.extend_from_slice(corners);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stmts(data: &[u8]) -> Vec<(u64, String)> {
        let mut v = Vec::new();
        for_each_stmt::<()>(data, 1, |l, s| {
            v.push((l, String::from_utf8_lossy(s).into_owned()));
            Ok(())
        })
        .expect("ok");
        v
    }

    #[test]
    fn statements_join_continuations_and_strip_cr() {
        let s = stmts(b"v 1 2 3\r\nf 1 \\\r\n 2 \\\n3\nlast");
        assert_eq!(
            s,
            vec![
                (1, "v 1 2 3".to_owned()),
                (2, "f 1   2  3".to_owned()),
                (5, "last".to_owned())
            ]
        );
    }

    #[test]
    fn cuts_never_split_a_continuation() {
        let d = b"f 1 \\\n2 3\nv 1 2 3\n";
        assert_eq!(last_cut(d), Some(d.len()));
        assert_eq!(last_cut(b"f 1 \\\n2 3"), None);
        assert_eq!(next_cut(d, 0), Some(10));
        assert_eq!(next_cut(d, 11), Some(d.len()));
        assert_eq!(last_cut(b"a\\\nb\nc\\\nd"), Some(5));
    }

    #[test]
    fn index_resolution() {
        assert_eq!(resolve(1, 0), 0);
        assert_eq!(resolve(-1, 4), 3);
        assert_eq!(resolve(-5, 4), BAD);
        assert_eq!(resolve(0, 4), BAD);
        assert_eq!(corner(b"3/-1/7", 10, 2).ok(), Some([2, 1]));
        assert_eq!(corner(b"3//7", 10, 2).ok(), Some([2, NO_VT]));
        assert!(corner(b"3/x", 10, 2).is_err());
    }
}
