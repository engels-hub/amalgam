//! Parser for a single quantity literal (units note §7).

use std::ops::Range;

use super::{Quantity, Unit};
use crate::error::Diagnostic;

/// A parsed literal: the number as typed, in `unit` (not yet canonical).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParsedQuantity {
    /// The value in `unit` (for `5' 3"` this is 63 in inches).
    pub value: f64,
    /// The typed unit, or `None` for a bare number.
    pub unit: Option<Unit>,
}

impl ParsedQuantity {
    /// Applies a context default unit when none was typed (the document length unit for
    /// length fields, `deg` for angle fields) and returns the canonical [`Quantity`].
    pub fn resolve(self, default: Unit) -> Quantity {
        Quantity::from_unit(self.value, self.unit.unwrap_or(default))
    }
}

/// Why a quantity literal could not be parsed. Spans are byte ranges into the input.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ParseQuantityError {
    /// The input is empty or whitespace only.
    #[error("empty value")]
    Empty,
    /// The magnitude is not a number.
    #[error("`{text}` is not a number")]
    InvalidNumber {
        /// Byte span of the offending token.
        span: Range<usize>,
        /// The offending token.
        text: String,
    },
    /// The unit is not in the unit table.
    #[error("unknown unit `{text}`")]
    UnknownUnit {
        /// Byte span of the unit token.
        span: Range<usize>,
        /// The unit token.
        text: String,
    },
    /// A fraction has denominator zero.
    #[error("division by zero in fraction")]
    ZeroDenominator {
        /// Byte span of the fraction (`1/0`).
        span: Range<usize>,
    },
    /// A `,` was used as decimal separator.
    #[error("use `.` as decimal separator")]
    DecimalComma {
        /// Byte span of the comma.
        span: Range<usize>,
    },
    /// Input continues after a complete literal (`1 m 20 cm`).
    #[error("unexpected `{text}`; compound expressions belong in parameter expressions")]
    Trailing {
        /// Byte span from the first unexpected byte to the end of the input.
        span: Range<usize>,
        /// The unexpected text.
        text: String,
    },
}

impl ParseQuantityError {
    /// Byte span into the input for UI highlighting (`None` for [`Empty`](Self::Empty)).
    pub fn span(&self) -> Option<Range<usize>> {
        match self {
            ParseQuantityError::Empty => None,
            ParseQuantityError::InvalidNumber { span, .. }
            | ParseQuantityError::UnknownUnit { span, .. }
            | ParseQuantityError::ZeroDenominator { span }
            | ParseQuantityError::DecimalComma { span }
            | ParseQuantityError::Trailing { span, .. } => Some(span.clone()),
        }
    }
}

impl Diagnostic for ParseQuantityError {
    fn hint(&self) -> Option<String> {
        Some(
            match self {
                ParseQuantityError::Empty => "Enter a value such as 12.5 mm.",
                ParseQuantityError::InvalidNumber { .. } => {
                    "Write a number such as 12.5, 1/4, 1 1/4 or 5' 3\"."
                }
                ParseQuantityError::UnknownUnit { .. } => {
                    "Use a unit such as mm, cm, m, in, ft, deg, rad, kg, N, MPa or degC."
                }
                ParseQuantityError::ZeroDenominator { .. } => {
                    "The denominator of a fraction cannot be 0."
                }
                ParseQuantityError::DecimalComma { .. } => "Write 12.5, not 12,5.",
                ParseQuantityError::Trailing { .. } => {
                    "Use a parameter expression for compound values, e.g. 1 m + 20 cm."
                }
            }
            .to_owned(),
        )
    }
}

/// Parses ONE quantity literal: `12.5 mm`, `1/4 in`, `1 1/4"`, `5' 3 1/2"`, `30 deg`,
/// `-.5e1`.
///
/// A bare number gives `unit: None`; apply the context default with
/// [`ParsedQuantity::resolve`]. Compound forms other than feet-inch (`1 m 20 cm`) are
/// [`Trailing`](ParseQuantityError::Trailing) errors: they belong to S01 expressions.
///
/// ```
/// use amg_core::{parse_quantity, Unit};
/// let p = parse_quantity("5' 3 1/2\"")?;
/// assert_eq!(p.unit, Some(Unit::Inch));
/// assert_eq!(p.value, 63.5);
/// # Ok::<(), amg_core::ParseQuantityError>(())
/// ```
pub fn parse_quantity(input: &str) -> Result<ParsedQuantity, ParseQuantityError> {
    let mut c = Cursor { s: input, pos: 0 };
    c.skip_ws();
    if c.eof() {
        return Err(ParseQuantityError::Empty);
    }
    let start = c.pos;
    let negative = match c.peek() {
        Some(b'-') => {
            c.pos += 1;
            true
        }
        Some(b'+') => {
            c.pos += 1;
            false
        }
        _ => false,
    };
    let mut value = c.magnitude(start)?;
    c.skip_ws();
    let mut unit = None;
    if !c.eof() {
        if c.at_number() {
            return Err(c.trailing());
        }
        let u = c.unit()?;
        unit = Some(u);
        c.skip_ws();
        if !c.eof() {
            // The only two-term form: feet followed by inches.
            if u != Unit::Foot || !c.at_number() {
                return Err(c.trailing());
            }
            let second = c.pos;
            let inches = c.magnitude(second)?;
            c.skip_ws();
            if c.eof() || c.unit().ok() != Some(Unit::Inch) {
                c.pos = second;
                return Err(c.trailing());
            }
            c.skip_ws();
            if !c.eof() {
                return Err(c.trailing());
            }
            value = value * 12.0 + inches;
            if !value.is_finite() {
                let end = input.trim_end_matches(is_ws_char).len();
                return Err(ParseQuantityError::InvalidNumber {
                    span: start..end,
                    text: input[start..end].to_owned(),
                });
            }
            unit = Some(Unit::Inch);
        }
    }
    Ok(ParsedQuantity {
        value: if negative { -value } else { value },
        unit,
    })
}

struct Cursor<'a> {
    s: &'a str,
    pos: usize,
}

/// Byte length of the whitespace character at the start of `b` (0 if none): space, tab,
/// CR, LF, NBSP (U+00A0) and narrow NBSP (U+202F, a French unit separator).
fn ws_len(b: &[u8]) -> usize {
    match b {
        [b' ' | b'\t' | b'\r' | b'\n', ..] => 1,
        [0xC2, 0xA0, ..] => 2,
        [0xE2, 0x80, 0xAF, ..] => 3,
        _ => 0,
    }
}

/// The characters [`ws_len`] accepts.
fn is_ws_char(c: char) -> bool {
    matches!(c, ' ' | '\t' | '\r' | '\n' | '\u{A0}' | '\u{202F}')
}

impl Cursor<'_> {
    fn peek(&self) -> Option<u8> {
        self.s.as_bytes().get(self.pos).copied()
    }
    fn peek_at(&self, i: usize) -> Option<u8> {
        self.s.as_bytes().get(i).copied()
    }
    fn eof(&self) -> bool {
        self.pos >= self.s.len()
    }
    fn skip_ws(&mut self) -> usize {
        let start = self.pos;
        loop {
            let n = ws_len(&self.s.as_bytes()[self.pos..]);
            if n == 0 {
                break;
            }
            self.pos += n;
        }
        self.pos - start
    }
    fn digits(&mut self) -> usize {
        let start = self.pos;
        while self.peek().is_some_and(|b| b.is_ascii_digit()) {
            self.pos += 1;
        }
        self.pos - start
    }
    /// Next byte starts a number (`digit` or `.digit`).
    fn at_number(&self) -> bool {
        match self.peek() {
            Some(b) if b.is_ascii_digit() => true,
            Some(b'.') => self
                .peek_at(self.pos + 1)
                .is_some_and(|b| b.is_ascii_digit()),
            _ => false,
        }
    }
    /// End of the whitespace-delimited token starting at `from`.
    fn token_end(&self, from: usize) -> usize {
        self.s[from..]
            .find(is_ws_char)
            .map_or(self.s.len(), |p| from + p)
    }
    fn invalid_number(&self, from: usize) -> ParseQuantityError {
        let end = self.token_end(self.pos).max(from);
        ParseQuantityError::InvalidNumber {
            span: from..end,
            text: self.s[from..end].to_owned(),
        }
    }
    fn trailing(&self) -> ParseQuantityError {
        ParseQuantityError::Trailing {
            span: self.pos..self.s.len(),
            text: self.s[self.pos..].to_owned(),
        }
    }
    fn check_comma(&self) -> Result<(), ParseQuantityError> {
        if self.peek() == Some(b',')
            && self
                .peek_at(self.pos + 1)
                .is_some_and(|b| b.is_ascii_digit())
        {
            return Err(ParseQuantityError::DecimalComma {
                span: self.pos..self.pos + 1,
            });
        }
        Ok(())
    }
    fn uint_value(&self, r: Range<usize>) -> f64 {
        // ASCII digits always parse; huge integers round to the nearest f64.
        self.s[r].parse::<f64>().unwrap_or(f64::NAN)
    }

    /// `mixed | fraction | decimal`; `token_start` is where an error span starts (the sign).
    /// Values that overflow `f64` (`1e400`, 400-digit integers) are `InvalidNumber`.
    fn magnitude(&mut self, token_start: usize) -> Result<f64, ParseQuantityError> {
        let v = self.magnitude_raw(token_start)?;
        if v.is_finite() {
            Ok(v)
        } else {
            Err(ParseQuantityError::InvalidNumber {
                span: token_start..self.pos,
                text: self.s[token_start..self.pos].to_owned(),
            })
        }
    }

    fn magnitude_raw(&mut self, token_start: usize) -> Result<f64, ParseQuantityError> {
        let start = self.pos;
        let int_len = self.digits();
        self.check_comma()?;
        if int_len > 0 && self.peek() == Some(b'/') {
            return self.fraction(start, token_start);
        }
        if int_len > 0 && !matches!(self.peek(), Some(b'.' | b'e' | b'E')) {
            // A mixed number `1 1/4`: whole part, ws1, fraction.
            let after_int = self.pos;
            if self.skip_ws() > 0 {
                let num = self.pos;
                if self.digits() > 0 && self.peek() == Some(b'/') {
                    let whole = self.uint_value(start..after_int);
                    let frac = self.fraction(num, num)?;
                    return Ok(whole + frac);
                }
            }
            self.pos = after_int;
            return Ok(self.uint_value(start..after_int));
        }
        let mut frac_len = 0;
        if self.peek() == Some(b'.') {
            self.pos += 1;
            frac_len = self.digits();
            self.check_comma()?;
        }
        if int_len == 0 && frac_len == 0 {
            self.pos = start;
            return Err(self.invalid_number(token_start));
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            let save = self.pos;
            self.pos += 1;
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.pos += 1;
            }
            if self.digits() == 0 {
                // Not an exponent; `e` may start a unit (none does today).
                self.pos = save;
            }
        }
        self.s[start..self.pos]
            .parse::<f64>()
            .map_err(|_| self.invalid_number(token_start))
    }

    /// `uint "/" uint` with the cursor on `/`; the numerator starts at `num`.
    fn fraction(&mut self, num: usize, token_start: usize) -> Result<f64, ParseQuantityError> {
        let slash = self.pos;
        self.pos += 1;
        let den_start = self.pos;
        if self.digits() == 0 {
            return Err(self.invalid_number(token_start));
        }
        let n = self.uint_value(num..slash);
        let d = self.uint_value(den_start..self.pos);
        if d == 0.0 {
            return Err(ParseQuantityError::ZeroDenominator {
                span: num..self.pos,
            });
        }
        Ok(n / d)
    }

    /// Longest table symbol at the cursor that ends on a token boundary.
    fn unit(&mut self) -> Result<Unit, ParseQuantityError> {
        let rest = &self.s[self.pos..];
        let first = rest.as_bytes().first().copied();
        // Longest symbols come first, so the first hit is the longest match.
        let best = Unit::symbols_longest_first().iter().find(|(sym, _)| {
            sym.as_bytes().first().copied() == first
                && rest.starts_with(sym)
                && boundary(sym, &rest[sym.len()..])
        });
        match best.map(|&(sym, u)| (u, sym.len())) {
            Some((u, len)) => {
                self.pos += len;
                Ok(u)
            }
            None => {
                let end = self.token_end(self.pos);
                Err(ParseQuantityError::UnknownUnit {
                    span: self.pos..end,
                    text: self.s[self.pos..end].to_owned(),
                })
            }
        }
    }
}

/// A symbol ends at whitespace or the end; a punctuation symbol (`'`, `"`) may also be
/// followed directly by a number (`5'3"`).
fn boundary(sym: &str, after: &str) -> bool {
    match after.as_bytes().first() {
        None => true,
        Some(_) if ws_len(after.as_bytes()) > 0 => true,
        Some(&b) => {
            let punct = sym.chars().last().is_some_and(|c| !c.is_alphanumeric());
            punct && (b.is_ascii_digit() || b == b'.')
        }
    }
}
