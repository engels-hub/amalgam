//! STL detection (note §5.1). The readers and writers land with IO-02.

mod ascii;
mod read;
mod weld;
mod write;

pub use read::read;
pub use write::write;

use crate::error::{IoError, Location};
use crate::format::{Format, contains_ci, skip_ws, strip_bom};

/// Which kind of STL a file is.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum StlKind {
    /// Binary. `declared` is the triangle count in the header (it can be wrong, see
    /// note §5.1 rules 3 and 4).
    Binary {
        /// Count at byte offset 80.
        declared: u32,
    },
    /// ASCII.
    Ascii,
}

/// The rule of note §5.1 that decided.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Rule {
    /// 1: `84 + 50·n == len`.
    ExactSize,
    /// 2: starts with `solid` and has `facet` in the head.
    AsciiKeyword,
    /// 3: `(len − 84) % 50 == 0` but the count disagrees.
    CountMismatch,
    /// 4: binary, truncated or with trailing bytes.
    SizeMismatch,
}

/// Decides between binary and ASCII STL (note §5.1, rules in order). `head` is the first
/// ≤ 4 KiB of the file, `file_len` its full length.
///
/// # Errors
/// [`IoError::Malformed`] at byte 0 if neither fits (rule 5).
pub fn detect(head: &[u8], file_len: u64) -> Result<StlKind, IoError> {
    detect_rule(head, file_len).map(|(k, _)| k)
}

pub(crate) fn detect_rule(head: &[u8], file_len: u64) -> Result<(StlKind, Rule), IoError> {
    let declared = (head.len() >= 84 && file_len >= 84)
        .then(|| u32::from_le_bytes([head[80], head[81], head[82], head[83]]));
    // 1
    if let Some(n) = declared
        && 84 + 50 * u64::from(n) == file_len
    {
        return Ok((StlKind::Binary { declared: n }, Rule::ExactSize));
    }
    // 2
    let t = skip_ws(strip_bom(head));
    if t.len() >= 5 && t[..5].eq_ignore_ascii_case(b"solid") && contains_ci(head, b"facet") {
        return Ok((StlKind::Ascii, Rule::AsciiKeyword));
    }
    if let Some(n) = declared {
        // 3
        if (file_len - 84).is_multiple_of(50) {
            return Ok((StlKind::Binary { declared: n }, Rule::CountMismatch));
        }
        // 4
        return Ok((StlKind::Binary { declared: n }, Rule::SizeMismatch));
    }
    // 5
    Err(IoError::Malformed {
        format: Format::Stl,
        location: Location::Byte(0),
        message: "neither a binary STL (needs at least 84 bytes) nor an ASCII STL".into(),
    })
}

/// Extra plausibility test for the loose binary rules (3 and 4), used only when *sniffing*
/// without a known format. Without it any text file of 84 bytes or more would be an STL.
/// The head must hold a whole first triangle record (so tiny files are never claimed) and:
/// - the 80-byte header holds no control characters;
/// - the record has a NUL byte (a zero in the normal or the attribute word, which text never
///   has; a record without any is rare enough to ask for an explicit format);
/// - the nine vertex coordinates are finite and below 1e12.
pub(crate) fn looks_like_binary(head: &[u8]) -> bool {
    const RECORD: usize = 84 + 50;
    if head.len() < RECORD {
        return false;
    }
    let header_ok = head[..80]
        .iter()
        .all(|&c| matches!(c, 0 | 9 | 10 | 13) || (0x20..0x7F).contains(&c));
    if !header_ok || !head[84..RECORD].contains(&0) {
        return false;
    }
    head[84 + 12..84 + 48].chunks_exact(4).all(|b| {
        let v = f32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        v.is_finite() && v.abs() < 1e12
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn binary(header: &[u8], declared: u32, tris: usize) -> Vec<u8> {
        let mut v = vec![0u8; 80];
        v[..header.len()].copy_from_slice(header);
        v.extend_from_slice(&declared.to_le_bytes());
        v.extend(std::iter::repeat_n(0u8, 50 * tris));
        v
    }

    #[test]
    fn rule_1_exact_size_wins_even_with_a_solid_header() {
        let f = binary(b"solid facet exported by CAD", 2, 2);
        assert_eq!(
            detect(&f, f.len() as u64).ok(),
            Some(StlKind::Binary { declared: 2 })
        );
    }

    #[test]
    fn rule_2_ascii_with_bom_whitespace_and_case() {
        let f = b"\xEF\xBB\xBF  \r\nSOLID x\r\n FACET normal 0 0 1\r\n";
        assert_eq!(detect(f, f.len() as u64).ok(), Some(StlKind::Ascii));
    }

    #[test]
    fn rule_2_needs_facet() {
        let f = b"solid empty\nendsolid empty\n";
        assert!(detect(f, f.len() as u64).is_err());
    }

    #[test]
    fn rule_3_count_zero_but_size_fits() {
        let f = binary(b"made by a lazy writer", 0, 3);
        let (k, r) = detect_rule(&f, f.len() as u64).expect("detect");
        assert_eq!(
            (k, r),
            (StlKind::Binary { declared: 0 }, Rule::CountMismatch)
        );
    }

    #[test]
    fn rule_4_truncated_or_trailing() {
        let mut f = binary(b"h", 5, 2);
        f.extend_from_slice(&[0; 7]);
        let (k, r) = detect_rule(&f, f.len() as u64).expect("detect");
        assert_eq!(
            (k, r),
            (StlKind::Binary { declared: 5 }, Rule::SizeMismatch)
        );
    }

    #[test]
    fn rule_5_malformed_is_byte_zero() {
        let e = detect(b"hello", 5).expect_err("must fail");
        assert!(matches!(
            e,
            IoError::Malformed {
                format: Format::Stl,
                location: Location::Byte(0),
                ..
            }
        ));
        assert!(detect(b"", 0).is_err());
    }

    #[test]
    fn plausibility_rejects_control_characters_nan_and_text() {
        let mut f = binary(b"ok", 1, 1);
        assert!(looks_like_binary(&f));
        f[3] = 0x01;
        assert!(!looks_like_binary(&f));
        let mut g = binary(b"ok", 1, 1);
        g[84 + 12..84 + 16].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(!looks_like_binary(&g));
        let mut h = binary(b"ok", 1, 1);
        h[84 + 20..84 + 24].copy_from_slice(&1e30f32.to_le_bytes());
        assert!(!looks_like_binary(&h));
        assert!(!looks_like_binary(b"short"));
        // Prose of any length has no NUL and is not a binary STL.
        let text = b"This is a plain text file that happens to be long enough. ".repeat(6);
        assert!(!looks_like_binary(&text));
        // A head that stops before the first record cannot be judged.
        assert!(!looks_like_binary(&binary(b"ok", 1, 1)[..120]));
    }
}
