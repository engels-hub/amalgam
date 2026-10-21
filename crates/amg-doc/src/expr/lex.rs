//! Lexer for parameter expressions (S01 §5).

use std::ops::Range;

use amg_core::Unit;

use super::ExprError;

/// Token kinds. Identifier text is `&src[span]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Tok {
    /// A numeric literal. `int` is true when it is digits only (eligible for `1/4`, `1 1/4`).
    Num {
        value: f64,
        int: bool,
    },
    /// Identifier: parameter, unit, constant or function name.
    Ident,
    /// Punctuation unit symbol: `'`, `"`, `°`, `°C`, `°F`.
    Sym(Unit),
    Plus,
    Minus,
    Star,
    Slash,
    Caret,
    LParen,
    RParen,
    Comma,
    /// Superscript exponent `²` or `³` (unit exponents: `mm²`).
    Sup(i8),
    Eof,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Token {
    pub(crate) tok: Tok,
    pub(crate) span: Range<usize>,
}

/// Whitespace: space, tab, CR, LF, NBSP (U+00A0) and narrow NBSP (U+202F), the same set as
/// `amg_core::parse_quantity`.
pub(crate) fn is_ws(c: char) -> bool {
    matches!(c, ' ' | '\t' | '\r' | '\n' | '\u{A0}' | '\u{202F}')
}

pub(crate) fn is_ident_start(c: char) -> bool {
    c.is_alphabetic() || c == '_'
}

/// Superscript digits are not identifier characters, so `mm²` is `mm` + `²`.
pub(crate) fn is_ident_continue(c: char) -> bool {
    c.is_alphabetic() || c.is_ascii_digit() || c == '_'
}

/// True if `s` is one identifier token.
pub(crate) fn is_ident(s: &str) -> bool {
    let mut cs = s.chars();
    cs.next().is_some_and(is_ident_start) && cs.all(is_ident_continue)
}

fn err(span: Range<usize>, msg: impl Into<String>) -> ExprError {
    ExprError::Parse {
        span,
        msg: msg.into(),
    }
}

/// Splits `src` into tokens, ending with [`Tok::Eof`].
pub(crate) fn lex(src: &str) -> Result<Vec<Token>, ExprError> {
    let mut out = Vec::new();
    let b = src.as_bytes();
    let mut it = src.char_indices().peekable();
    while let Some(&(i, c)) = it.peek() {
        if is_ws(c) {
            it.next();
            continue;
        }
        let one = |tok: Tok| Token {
            tok,
            span: i..i + c.len_utf8(),
        };
        let simple = match c {
            '+' => Some(Tok::Plus),
            '-' | '\u{2212}' => Some(Tok::Minus),
            '*' | '\u{B7}' | '\u{D7}' => Some(Tok::Star),
            '/' | '\u{F7}' => Some(Tok::Slash),
            '^' => Some(Tok::Caret),
            '(' => Some(Tok::LParen),
            ')' => Some(Tok::RParen),
            ',' => Some(Tok::Comma),
            '\u{B2}' => Some(Tok::Sup(2)),
            '\u{B3}' => Some(Tok::Sup(3)),
            '\'' => Some(Tok::Sym(Unit::Foot)),
            '"' => Some(Tok::Sym(Unit::Inch)),
            _ => None,
        };
        if let Some(t) = simple {
            out.push(one(t));
            it.next();
            continue;
        }
        if c == '°' {
            it.next();
            let mut end = i + c.len_utf8();
            let mut unit = Unit::Degree;
            if let Some(&(j, n)) = it.peek() {
                let after = src[j + n.len_utf8()..].chars().next();
                if matches!(n, 'C' | 'F') && !after.is_some_and(is_ident_continue) {
                    unit = if n == 'C' {
                        Unit::DegreeCelsius
                    } else {
                        Unit::DegreeFahrenheit
                    };
                    end = j + 1;
                    it.next();
                }
            }
            out.push(Token {
                tok: Tok::Sym(unit),
                span: i..end,
            });
            continue;
        }
        if c.is_ascii_digit() || (c == '.' && b.get(i + 1).is_some_and(u8::is_ascii_digit)) {
            let start = i;
            let mut j = i;
            let digits = |j: &mut usize| {
                while b.get(*j).is_some_and(u8::is_ascii_digit) {
                    *j += 1;
                }
            };
            digits(&mut j);
            let mut int = true;
            if b.get(j) == Some(&b'.') {
                int = false;
                j += 1;
                digits(&mut j);
            }
            if matches!(b.get(j), Some(b'e' | b'E')) {
                let mut k = j + 1;
                if matches!(b.get(k), Some(b'+' | b'-')) {
                    k += 1;
                }
                if b.get(k).is_some_and(u8::is_ascii_digit) {
                    int = false;
                    j = k;
                    digits(&mut j);
                }
            }
            let text = &src[start..j];
            let value = text
                .parse::<f64>()
                .map_err(|_| err(start..j, format!("`{text}` is not a number")))?;
            if !value.is_finite() {
                return Err(err(start..j, format!("`{text}` is too large")));
            }
            out.push(Token {
                tok: Tok::Num { value, int },
                span: start..j,
            });
            while it.peek().is_some_and(|&(k, _)| k < j) {
                it.next();
            }
            continue;
        }
        if is_ident_start(c) {
            let start = i;
            let mut end = i;
            while let Some(&(k, ch)) = it.peek() {
                if k != start && !is_ident_continue(ch) {
                    break;
                }
                end = k + ch.len_utf8();
                it.next();
            }
            out.push(Token {
                tok: Tok::Ident,
                span: start..end,
            });
            continue;
        }
        return Err(err(
            i..i + c.len_utf8(),
            format!("unexpected character `{c}`"),
        ));
    }
    out.push(Token {
        tok: Tok::Eof,
        span: src.len()..src.len(),
    });
    Ok(out)
}
