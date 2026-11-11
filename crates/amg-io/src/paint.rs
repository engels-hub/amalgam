//! Slicer paint strings: Orca/Bambu `paint_color` and Prusa `slic3rpe:mmu_segmentation`
//! (note `import-export-formats.md` §4).
//!
//! A string encodes a recursive subdivision tree of one triangle. Hex characters are nibbles
//! in reverse order (the last character is the first nibble), each nibble's bits are read
//! LSB-first. Every node starts with one nibble: the low two bits are the number of split
//! sides (0 = leaf). A split node's high two bits are the special side and it is followed by
//! `split_sides + 1` children in reverse child order. A leaf's high two bits are the state
//! (0-2); `11` means a nibble `state - 3` follows, and a nibble `1111` there means a second
//! nibble `state - 18` follows.
//!
//! Interop facts were read from OrcaSlicer `TriangleSelector.cpp` (serialize, deserialize,
//! perform_split); no code was copied.

/// The largest state a paint string can hold (`0xF` + 18).
pub const MAX_STATE: u8 = 33;

/// A paint string that does not decode.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("invalid paint string at nibble {pos}")]
pub struct PaintError {
    /// 0-based nibble index in decoding order (counted from the last character).
    pub pos: usize,
}

/// Whole-triangle state (0 = object default, 1.. = filament) → slicer string (`""` for 0).
///
/// States above [`MAX_STATE`] cannot be encoded and are written as [`MAX_STATE`].
pub fn encode_state(state: u8) -> String {
    let s = state.min(MAX_STATE);
    let hex = |n: u8| char::from(b"0123456789ABCDEF"[usize::from(n)]);
    match s {
        0 => String::new(),
        1 | 2 => hex(s << 2).to_string(),
        3..=17 => [hex(s - 3), 'C'].iter().collect(),
        _ => [hex(s - 18), 'F', 'C'].iter().collect(),
    }
}

/// Areas of the children of a split node, as fractions of the parent, in child order
/// (OrcaSlicer `perform_split`: one split side bisects an edge; two split sides give a corner
/// quarter, the quarter next to it and the remaining half; three give four quarters).
fn child_areas(split_sides: u8) -> &'static [f64] {
    match split_sides {
        1 => &[0.5, 0.5],
        2 => &[0.25, 0.25, 0.5],
        _ => &[0.25, 0.25, 0.25, 0.25],
    }
}

/// A split node whose children are being read.
struct Frame {
    area: f64,
    split_sides: u8,
    /// Children not read yet. Children are stored in reverse order, so the next one read is
    /// child `remaining - 1`.
    remaining: u8,
}

/// Decodes one `paint_color` / `mmu_segmentation` string. Returns the dominant leaf state by
/// area (unit triangle) and whether the triangle was subdivided.
///
/// An empty string is the unpainted state 0. Ties between states of equal area go to the
/// smaller state. Nibbles after the complete tree are ignored, as the slicers do.
///
/// # Errors
/// [`PaintError`] for a character that is not a hex digit, a split code the slicers reject
/// (one or two split sides with special side 3) or a tree that is cut short.
pub fn decode_dominant(s: &str) -> Result<(u8, bool), PaintError> {
    let bytes = s.as_bytes();
    let n = bytes.len();
    if n == 0 {
        return Ok((0, false));
    }
    let mut pos = 0usize;
    let next = |pos: &mut usize| -> Result<u8, PaintError> {
        let Some(&c) = n.checked_sub(*pos + 1).and_then(|i| bytes.get(i)) else {
            return Err(PaintError { pos: *pos });
        };
        let v = match c {
            b'0'..=b'9' => c - b'0',
            b'A'..=b'F' => c - b'A' + 10,
            b'a'..=b'f' => c - b'a' + 10,
            _ => return Err(PaintError { pos: *pos }),
        };
        *pos += 1;
        Ok(v)
    };
    let mut area = [0f64; MAX_STATE as usize + 1];
    let mut stack: Vec<Frame> = Vec::new();
    let mut current = 1.0f64;
    let mut split_root = false;
    loop {
        let at = pos;
        let code = next(&mut pos)?;
        let split = code & 0b11;
        if split > 0 {
            if split < 3 && code >> 2 == 3 {
                return Err(PaintError { pos: at });
            }
            if stack.is_empty() {
                split_root = true;
            }
            let k = split + 1;
            let child = current * child_areas(split)[usize::from(k - 1)];
            stack.push(Frame {
                area: current,
                split_sides: split,
                remaining: k,
            });
            current = child;
            continue;
        }
        let state = if code >> 2 == 3 {
            let a = next(&mut pos)?;
            if a == 0xF {
                next(&mut pos)? + 18
            } else {
                a + 3
            }
        } else {
            code >> 2
        };
        area[usize::from(state)] += current;
        // This child is done: move to the next sibling, or up to the parent's sibling.
        loop {
            let Some(top) = stack.last_mut() else {
                return Ok((dominant(&area), split_root));
            };
            top.remaining -= 1;
            if top.remaining == 0 {
                stack.pop();
                continue;
            }
            current = top.area * child_areas(top.split_sides)[usize::from(top.remaining - 1)];
            break;
        }
    }
}

fn dominant(area: &[f64]) -> u8 {
    let mut best = 0usize;
    for (i, &a) in area.iter().enumerate() {
        if a > area[best] {
            best = i;
        }
    }
    best as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn whole_triangle_table_matches_the_slicer_constants() {
        // CONST_FILAMENTS (OrcaSlicer Model.cpp), note §4.
        let table = [
            (0, ""),
            (1, "4"),
            (2, "8"),
            (3, "0C"),
            (4, "1C"),
            (16, "DC"),
            (17, "EC"),
            (18, "0FC"),
            (19, "1FC"),
            (33, "FFC"),
        ];
        for (state, s) in table {
            assert_eq!(encode_state(state), s, "encode {state}");
            assert_eq!(decode_dominant(s), Ok((state, false)), "decode {s}");
        }
        assert_eq!(encode_state(200), "FFC");
    }

    #[test]
    fn round_trip_every_state() {
        for s in 0..=MAX_STATE {
            assert_eq!(decode_dominant(&encode_state(s)), Ok((s, false)));
            assert_eq!(
                decode_dominant(&encode_state(s).to_lowercase()),
                Ok((s, false))
            );
        }
    }

    /// Builds a string from nibbles in decoding order.
    fn from_nibbles(n: &[u8]) -> String {
        n.iter()
            .rev()
            .map(|&v| char::from(b"0123456789ABCDEF"[usize::from(v)]))
            .collect()
    }

    #[test]
    fn split_triangles_take_the_largest_leaf() {
        // Two split sides (code 0b0010, special side 0): children read in reverse, so the
        // first leaf read is child 2 (half the area). Child 2 = state 2, child 1 = 1, child 0 = 0.
        let s = from_nibbles(&[0b0010, 2 << 2, 1 << 2, 0]);
        assert_eq!(decode_dominant(&s), Ok((2, true)));
        // Child 2 = state 1, children 1 and 0 = state 2: 0.5 against 0.5, tie → smaller.
        let s = from_nibbles(&[0b0010, 1 << 2, 2 << 2, 2 << 2]);
        assert_eq!(decode_dominant(&s), Ok((1, true)));
        // Three split sides, four quarters: 3 × state 0C (=3) beats one state 1.
        let s = from_nibbles(&[0b0011, 1 << 2, 0b1100, 0, 0b1100, 0, 0b1100, 0]);
        assert_eq!(decode_dominant(&s), Ok((3, true)));
        // Nested: one split side, child 1 (read first) split into four quarters of state 2,
        // child 0 = state 1 (half). Equal areas → 1.
        let s = from_nibbles(&[0b0001, 0b0011, 8, 8, 8, 8, 4]);
        assert_eq!(decode_dominant(&s), Ok((1, true)));
        // Both children split: 7 of 8 eighths are state 2.
        let s = from_nibbles(&[0b0001, 0b0011, 8, 8, 8, 4, 0b0011, 8, 8, 8, 8]);
        assert_eq!(decode_dominant(&s), Ok((2, true)));
    }

    #[test]
    fn errors_name_the_nibble() {
        assert_eq!(decode_dominant("G"), Err(PaintError { pos: 0 }));
        assert_eq!(decode_dominant("4 "), Err(PaintError { pos: 0 }));
        // Truncated: a split root with no children.
        assert_eq!(decode_dominant("1"), Err(PaintError { pos: 1 }));
        // `11` leaf without its nibble.
        assert_eq!(decode_dominant("C"), Err(PaintError { pos: 1 }));
        // One split side with special side 3 is rejected by the slicers.
        assert_eq!(decode_dominant("D"), Err(PaintError { pos: 0 }));
        // Trailing nibbles after a complete tree are ignored.
        assert_eq!(decode_dominant("A4"), Ok((1, false)));
    }

    #[test]
    fn hostile_strings_terminate() {
        // A long chain of nested splits that never closes.
        let s = "3".repeat(10_000);
        assert!(decode_dominant(&s).is_err());
        let s = "F".repeat(10_000);
        let _ = decode_dominant(&s);
    }
}
