//! ASCII STL block parser (note §5.2). A block is a byte range that starts at a token boundary
//! and ends after an `endfacet` (or at the end of the file); blocks parse independently, so
//! the driver can run them in parallel.

use super::weld::Key;

/// `solid` / `endsolid` seen between facets.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) enum Event {
    /// `solid <name>`.
    Solid(String),
    /// `endsolid ...`.
    EndSolid,
}

/// What one block holds.
#[derive(Debug, Default)]
pub(super) struct Block {
    /// Three keys per triangle.
    pub corners: Vec<Key>,
    /// Solid markers with the index of the first triangle that follows them.
    pub events: Vec<(usize, Event)>,
    /// `\n` count of the block (valid when `error` is `None`).
    pub lines: u64,
    /// Triangles dropped for NaN/inf.
    pub non_finite: u64,
    /// Loops with more than three vertices (fan-triangulated).
    pub polygons: u64,
    /// Loops with fewer than three vertices (dropped).
    pub short_loops: u64,
    /// First problem: 1-based line inside the block and a message.
    pub error: Option<(u64, String)>,
}

#[inline]
fn is_ws(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\r' | b'\n' | 0x0B | 0x0C)
}

#[inline]
fn key(v: f32) -> u32 {
    let b = v.to_bits();
    if b == 0x8000_0000 { 0 } else { b }
}

struct Lexer<'a> {
    buf: &'a [u8],
    pos: usize,
    line: u64,
}

impl<'a> Lexer<'a> {
    fn token(&mut self) -> Option<&'a [u8]> {
        let b = self.buf;
        let mut p = self.pos;
        while p < b.len() && is_ws(b[p]) {
            if b[p] == b'\n' {
                self.line += 1;
            }
            p += 1;
        }
        if p >= b.len() {
            self.pos = p;
            return None;
        }
        let start = p;
        while p < b.len() && !is_ws(b[p]) {
            p += 1;
        }
        self.pos = p;
        Some(&b[start..p])
    }

    /// The rest of the current line, trimmed (the newline stays for the next `token`).
    fn rest_of_line(&mut self) -> &'a [u8] {
        let b = self.buf;
        let start = self.pos;
        let mut p = start;
        while p < b.len() && b[p] != b'\n' {
            p += 1;
        }
        self.pos = p;
        b[start..p].trim_ascii()
    }
}

fn number(tok: &[u8]) -> Option<f32> {
    std::str::from_utf8(tok).ok()?.parse::<f32>().ok()
}

/// Parses one block.
pub(super) fn parse_block(buf: &[u8]) -> Block {
    let mut out = Block::default();
    let mut lx = Lexer {
        buf,
        pos: 0,
        line: 1,
    };
    let mut verts: Vec<[f32; 3]> = Vec::with_capacity(4);
    out.corners.reserve(buf.len() / 70);

    while let Some(tok) = lx.token() {
        if tok.eq_ignore_ascii_case(b"vertex") {
            let line = lx.line;
            let mut v = [0f32; 3];
            for c in &mut v {
                let Some(t) = lx.token() else {
                    out.error = Some((line, "vertex has fewer than three coordinates".into()));
                    return out;
                };
                let Some(x) = number(t) else {
                    out.error = Some((
                        lx.line,
                        format!("`{}` is not a number", String::from_utf8_lossy(t)),
                    ));
                    return out;
                };
                *c = x;
            }
            verts.push(v);
        } else if tok.eq_ignore_ascii_case(b"endloop") || tok.eq_ignore_ascii_case(b"endfacet") {
            flush(&mut verts, &mut out);
        } else if tok.eq_ignore_ascii_case(b"solid") {
            flush(&mut verts, &mut out);
            let name = String::from_utf8_lossy(lx.rest_of_line()).into_owned();
            out.events.push((out.corners.len() / 3, Event::Solid(name)));
        } else if tok.eq_ignore_ascii_case(b"endsolid") {
            flush(&mut verts, &mut out);
            lx.rest_of_line();
            out.events.push((out.corners.len() / 3, Event::EndSolid));
        }
        // Everything else (`facet`, `normal`, its numbers, `outer`, `loop`) carries nothing
        // we use: the normal is recomputed from the winding.
    }
    flush(&mut verts, &mut out);
    out.lines = lx.line - 1;
    out
}

/// Closes the current loop: triangle, fan, or nothing.
fn flush(verts: &mut Vec<[f32; 3]>, out: &mut Block) {
    match verts.len() {
        0 => {}
        1 | 2 => out.short_loops += 1,
        n => {
            if n > 3 {
                out.polygons += 1;
            }
            for i in 1..n - 1 {
                let tri = [verts[0], verts[i], verts[i + 1]];
                if tri.iter().flatten().all(|c| c.is_finite()) {
                    out.corners
                        .extend(tri.iter().map(|v| [key(v[0]), key(v[1]), key(v[2])]));
                } else {
                    out.non_finite += 1;
                }
            }
        }
    }
    verts.clear();
}

/// Index just after the last `endfacet` (any case) in `buf`, or `None`.
pub(super) fn after_last_endfacet(buf: &[u8]) -> Option<usize> {
    const W: &[u8] = b"endfacet";
    if buf.len() < W.len() {
        return None;
    }
    (0..=buf.len() - W.len())
        .rev()
        .find(|&i| buf[i..i + W.len()].eq_ignore_ascii_case(W))
        .map(|i| i + W.len())
}

/// Index just after the first `endfacet` at or after `from`, or `None`.
pub(super) fn after_next_endfacet(buf: &[u8], from: usize) -> Option<usize> {
    const W: &[u8] = b"endfacet";
    if buf.len() < W.len() || from > buf.len() - W.len() {
        return None;
    }
    (from..=buf.len() - W.len())
        .find(|&i| buf[i..i + W.len()].eq_ignore_ascii_case(W))
        .map(|i| i + W.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_facet_with_odd_spelling() {
        let src = b"SOLID a b\r\n  FACET NORMAL 0 0 1\r\n\tOUTER LOOP\r\n VERTEX 0 0 0\r\nvertex\t1e0 0 -0\r\nVertex 0 1 0.5\r\n ENDLOOP\r\nENDFACET\r\nENDSOLID a b";
        let b = parse_block(src);
        assert!(b.error.is_none());
        assert_eq!(b.corners.len(), 3);
        assert_eq!(b.corners[1], [1.0f32.to_bits(), 0, 0], "-0 is normalised");
        assert_eq!(b.events.len(), 2);
        assert_eq!(b.events[0], (0, Event::Solid("a b".into())));
        assert_eq!(b.events[1], (1, Event::EndSolid));
        assert_eq!(b.lines, 8);
    }

    #[test]
    fn quads_are_fanned_and_short_loops_dropped() {
        let src = b"facet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 1 1 0\nvertex 0 1 0\nendloop\nendfacet\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\n";
        let b = parse_block(src);
        assert_eq!((b.corners.len(), b.polygons, b.short_loops), (6, 1, 1));
    }

    #[test]
    fn non_finite_and_bad_numbers() {
        let b = parse_block(b"vertex 0 0 0\nvertex nan 0 0\nvertex 1 1 1\nendloop\n");
        assert_eq!((b.corners.len(), b.non_finite), (0, 1));
        let b = parse_block(b"\n\nvertex 0 0 abc\n");
        assert_eq!(b.error.as_ref().map(|e| e.0), Some(3));
        let b = parse_block(b"vertex 0 0\n");
        assert!(b.error.is_some());
    }

    #[test]
    fn block_boundaries_search() {
        let s = b"x ENDFACET y endfacet z";
        assert_eq!(after_next_endfacet(s, 0), Some(10));
        assert_eq!(after_next_endfacet(s, 11), Some(21));
        assert_eq!(after_last_endfacet(s), Some(21));
        assert_eq!(after_last_endfacet(b"nothing"), None);
    }
}
