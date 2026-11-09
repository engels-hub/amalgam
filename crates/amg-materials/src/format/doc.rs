//! A small TOML document tree with a fast path parser.
//!
//! The `toml` crate needs ~20 ms just to *tokenise* a 1000-material library on the reference
//! machine, and ~35 ms to build its `Table`; the budget for the whole load is 20 ms. So the
//! reader parses the subset of TOML that the library writer emits (and that people write by
//! hand: comments, `[table]` / `[[array]]` headers, dotted keys, basic and literal strings,
//! numbers, booleans, arrays and single-line inline tables) with a borrowing parser, and
//! **falls back to the `toml` crate** for anything else (multi-line strings, datetimes, `inf`,
//! hex numbers, duplicate or reopened tables, any syntax error). The fallback gives the
//! authoritative error messages and spans, and full TOML 1.1 generality. A differential test
//! checks that both paths build the same tree.

use std::borrow::Cow;

use crate::error::MaterialError;

/// A TOML value.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Val<'a> {
    Str(Cow<'a, str>),
    Int(i64),
    Float(f64),
    Bool(bool),
    /// `aot`: created by `[[header]]` (an array of tables), so more headers may append.
    Arr {
        items: Vec<Val<'a>>,
        aot: bool,
    },
    Tab(Tab<'a>),
}

/// How a table came to exist (decides whether a later header or dotted key may extend it).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Kind {
    /// A `[header]` or `[[header]]` table, or the root.
    Explicit,
    /// Created on the way to a deeper header (`[a.b.c]` creates `a` and `a.b`).
    ImplicitHeader,
    /// Created by a dotted key (`a.b = 1` creates `a`).
    Dotted,
    /// An inline table.
    Inline,
}

/// A TOML table: few entries, so lookups are linear scans.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Tab<'a> {
    pub(crate) entries: Vec<(Cow<'a, str>, Val<'a>)>,
    kind: Kind,
}

impl<'a> Tab<'a> {
    fn new(kind: Kind) -> Tab<'a> {
        Tab {
            entries: Vec::new(),
            kind,
        }
    }

    pub(crate) fn get(&self, key: &str) -> Option<&Val<'a>> {
        self.entries.iter().find(|(k, _)| k == key).map(|(_, v)| v)
    }

    pub(crate) fn contains_key(&self, key: &str) -> bool {
        self.get(key).is_some()
    }

    pub(crate) fn keys(&self) -> impl Iterator<Item = &str> {
        self.entries.iter().map(|(k, _)| k.as_ref())
    }

    fn index_of(&self, key: &str) -> Option<usize> {
        self.entries.iter().position(|(k, _)| k == key)
    }
}

impl<'a> Val<'a> {
    pub(crate) fn as_str(&self) -> Option<&str> {
        match self {
            Val::Str(s) => Some(s),
            _ => None,
        }
    }

    pub(crate) fn as_table(&self) -> Option<&Tab<'a>> {
        match self {
            Val::Tab(t) => Some(t),
            _ => None,
        }
    }

    pub(crate) fn as_array(&self) -> Option<&[Val<'a>]> {
        match self {
            Val::Arr { items, .. } => Some(items),
            _ => None,
        }
    }
}

/// Parses `src` into a document tree (the root table).
pub(crate) fn parse<'a>(src: &'a str, origin: &str) -> Result<Tab<'a>, MaterialError> {
    if let Ok(t) = Fast::new(src).document() {
        return Ok(t);
    }
    parse_with_toml(src, origin)
}

/// The authoritative path: the `toml` crate (also the source of error messages).
pub(crate) fn parse_with_toml(src: &str, origin: &str) -> Result<Tab<'static>, MaterialError> {
    let table: toml::Table = src
        .parse()
        .map_err(|e: toml::de::Error| MaterialError::Toml {
            origin: origin.to_owned(),
            message: e.message().to_owned(),
            span: e.span(),
        })?;
    Ok(from_toml_table(table, Kind::Explicit))
}

fn from_toml_table(t: toml::Table, kind: Kind) -> Tab<'static> {
    Tab {
        entries: t
            .into_iter()
            .map(|(k, v)| (Cow::Owned(k), from_toml(v)))
            .collect(),
        kind,
    }
}

fn from_toml(v: toml::Value) -> Val<'static> {
    match v {
        toml::Value::String(s) => Val::Str(Cow::Owned(s)),
        toml::Value::Integer(i) => Val::Int(i),
        toml::Value::Float(f) => Val::Float(f),
        toml::Value::Boolean(b) => Val::Bool(b),
        // Datetimes have no meaning in a library file; keep their text so the reader reports
        // "expected a string" at the right key.
        toml::Value::Datetime(d) => Val::Str(Cow::Owned(d.to_string())),
        toml::Value::Array(a) => {
            let aot = !a.is_empty() && a.iter().all(toml::Value::is_table);
            Val::Arr {
                items: a.into_iter().map(from_toml).collect(),
                aot,
            }
        }
        toml::Value::Table(t) => Val::Tab(from_toml_table(t, Kind::Inline)),
    }
}

/// "Use the fallback parser."
#[derive(Debug)]
struct Fallback;

type R<T> = Result<T, Fallback>;

struct Fast<'a> {
    s: &'a str,
    b: &'a [u8],
    i: usize,
}

fn is_bare(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_' || b == b'-'
}

impl<'a> Fast<'a> {
    fn new(s: &'a str) -> Fast<'a> {
        Fast {
            s,
            b: s.as_bytes(),
            i: 0,
        }
    }

    fn peek(&self) -> Option<u8> {
        self.b.get(self.i).copied()
    }

    fn skip_blank(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\t')) {
            self.i += 1;
        }
    }

    fn skip_comment(&mut self) {
        if self.peek() == Some(b'#') {
            while !matches!(self.peek(), None | Some(b'\n')) {
                self.i += 1;
            }
        }
    }

    /// Consumes `\n` or `\r\n`; false if the cursor is not on a newline.
    fn newline(&mut self) -> R<bool> {
        match self.peek() {
            Some(b'\n') => {
                self.i += 1;
                Ok(true)
            }
            Some(b'\r') if self.b.get(self.i + 1) == Some(&b'\n') => {
                self.i += 2;
                Ok(true)
            }
            Some(b'\r') => Err(Fallback),
            _ => Ok(false),
        }
    }

    /// Blanks, comment, then a newline or the end of input.
    fn end_of_line(&mut self) -> R<()> {
        self.skip_blank();
        self.skip_comment();
        if self.peek().is_none() || self.newline()? {
            Ok(())
        } else {
            Err(Fallback)
        }
    }

    fn document(mut self) -> R<Tab<'a>> {
        if self.s.starts_with('\u{FEFF}') {
            return Err(Fallback);
        }
        let mut root = Tab::new(Kind::Explicit);
        let mut cur: Vec<Cow<'a, str>> = Vec::new();
        loop {
            self.skip_blank();
            self.skip_comment();
            if self.peek().is_none() {
                return Ok(root);
            }
            if self.newline()? {
                continue;
            }
            if self.peek() == Some(b'[') {
                cur = self.header(&mut root)?;
                self.end_of_line()?;
            } else {
                let keys = self.key_path()?;
                self.skip_blank();
                if self.peek() != Some(b'=') {
                    return Err(Fallback);
                }
                self.i += 1;
                self.skip_blank();
                let v = self.value(0)?;
                let table = descend(&mut root, &cur)?;
                insert_dotted(table, keys, v)?;
                self.end_of_line()?;
            }
        }
    }

    /// `[a.b]` or `[[a.b]]`; creates the table and returns its path.
    fn header(&mut self, root: &mut Tab<'a>) -> R<Vec<Cow<'a, str>>> {
        self.i += 1;
        let aot = self.peek() == Some(b'[');
        if aot {
            self.i += 1;
        }
        self.skip_blank();
        let mut path = self.key_path()?;
        self.skip_blank();
        for _ in 0..if aot { 2 } else { 1 } {
            if self.peek() != Some(b']') {
                return Err(Fallback);
            }
            self.i += 1;
        }
        let last = path.pop().ok_or(Fallback)?;
        let parent = descend(root, &path)?;
        match parent.index_of(&last) {
            None if aot => parent.entries.push((
                last.clone(),
                Val::Arr {
                    items: vec![Val::Tab(Tab::new(Kind::Explicit))],
                    aot: true,
                },
            )),
            None => parent
                .entries
                .push((last.clone(), Val::Tab(Tab::new(Kind::Explicit)))),
            Some(idx) => match (&mut parent.entries[idx].1, aot) {
                (Val::Arr { items, aot: true }, true) => {
                    items.push(Val::Tab(Tab::new(Kind::Explicit)));
                }
                (Val::Tab(t), false) if t.kind == Kind::ImplicitHeader => t.kind = Kind::Explicit,
                _ => return Err(Fallback),
            },
        }
        path.push(last);
        Ok(path)
    }

    /// `a`, `a.b`, `"quoted".c` (keys with spaces around dots allowed).
    fn key_path(&mut self) -> R<Vec<Cow<'a, str>>> {
        let mut keys = Vec::with_capacity(3);
        loop {
            keys.push(self.key()?);
            self.skip_blank();
            if self.peek() == Some(b'.') {
                self.i += 1;
                self.skip_blank();
            } else {
                return Ok(keys);
            }
        }
    }

    fn key(&mut self) -> R<Cow<'a, str>> {
        match self.peek() {
            Some(b'"') => self.basic_string(),
            Some(b'\'') => self.literal_string(),
            Some(b) if is_bare(b) => {
                let start = self.i;
                while self.peek().is_some_and(is_bare) {
                    self.i += 1;
                }
                Ok(Cow::Borrowed(&self.s[start..self.i]))
            }
            _ => Err(Fallback),
        }
    }

    fn basic_string(&mut self) -> R<Cow<'a, str>> {
        if self.b[self.i..].starts_with(b"\"\"\"") {
            return Err(Fallback); // multi-line
        }
        self.i += 1;
        let start = self.i;
        // Fast scan: no escapes.
        loop {
            match self.peek() {
                Some(b'"') => {
                    let s = &self.s[start..self.i];
                    self.i += 1;
                    return Ok(Cow::Borrowed(s));
                }
                Some(b'\\') => break,
                Some(b) if b < 0x20 && b != b'\t' || b == 0x7F => return Err(Fallback),
                Some(_) => self.i += 1,
                None => return Err(Fallback),
            }
        }
        let mut out = String::from(&self.s[start..self.i]);
        loop {
            match self.peek() {
                Some(b'"') => {
                    self.i += 1;
                    return Ok(Cow::Owned(out));
                }
                Some(b'\\') => {
                    self.i += 1;
                    let esc = self.peek().ok_or(Fallback)?;
                    self.i += 1;
                    out.push(match esc {
                        b'"' => '"',
                        b'\\' => '\\',
                        b'n' => '\n',
                        b't' => '\t',
                        b'r' => '\r',
                        b'b' => '\u{8}',
                        b'f' => '\u{c}',
                        b'u' => self.hex_char(4)?,
                        b'U' => self.hex_char(8)?,
                        _ => return Err(Fallback),
                    });
                }
                Some(b) if b < 0x20 && b != b'\t' || b == 0x7F => return Err(Fallback),
                Some(_) => {
                    // Copy one full UTF-8 character.
                    let ch = self.s[self.i..].chars().next().ok_or(Fallback)?;
                    out.push(ch);
                    self.i += ch.len_utf8();
                }
                None => return Err(Fallback),
            }
        }
    }

    fn hex_char(&mut self, n: usize) -> R<char> {
        let digits = self.s.get(self.i..self.i + n).ok_or(Fallback)?;
        if !digits.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err(Fallback);
        }
        self.i += n;
        u32::from_str_radix(digits, 16)
            .ok()
            .and_then(char::from_u32)
            .ok_or(Fallback)
    }

    fn literal_string(&mut self) -> R<Cow<'a, str>> {
        if self.b[self.i..].starts_with(b"'''") {
            return Err(Fallback);
        }
        self.i += 1;
        let start = self.i;
        loop {
            match self.peek() {
                Some(b'\'') => {
                    let s = &self.s[start..self.i];
                    self.i += 1;
                    return Ok(Cow::Borrowed(s));
                }
                Some(b) if b < 0x20 && b != b'\t' || b == 0x7F => return Err(Fallback),
                Some(_) => self.i += 1,
                None => return Err(Fallback),
            }
        }
    }

    fn value(&mut self, depth: u32) -> R<Val<'a>> {
        if depth > 16 {
            return Err(Fallback);
        }
        match self.peek().ok_or(Fallback)? {
            b'"' => Ok(Val::Str(self.basic_string()?)),
            b'\'' => Ok(Val::Str(self.literal_string()?)),
            b'[' => self.array(depth),
            b'{' => self.inline_table(depth),
            b't' if self.b[self.i..].starts_with(b"true") => self.keyword(4, Val::Bool(true)),
            b'f' if self.b[self.i..].starts_with(b"false") => self.keyword(5, Val::Bool(false)),
            _ => self.number(),
        }
    }

    fn keyword(&mut self, len: usize, v: Val<'a>) -> R<Val<'a>> {
        self.i += len;
        if self.peek().is_some_and(is_bare) {
            Err(Fallback)
        } else {
            Ok(v)
        }
    }

    fn number(&mut self) -> R<Val<'a>> {
        let start = self.i;
        while matches!(self.peek(), Some(b) if b.is_ascii_digit() || matches!(b, b'+' | b'-' | b'.' | b'e' | b'E'))
        {
            self.i += 1;
        }
        let tok = &self.s[start..self.i];
        // Anything glued to the token (letters, `_`, `:` of a datetime, ...) is not ours.
        if self.peek().is_some_and(|b| is_bare(b) || b == b':') {
            return Err(Fallback);
        }
        let digits = tok.strip_prefix(['+', '-']).unwrap_or(tok);
        let int_part_end = digits.find(['.', 'e', 'E']).unwrap_or(digits.len());
        let int_part = &digits[..int_part_end];
        if int_part.is_empty()
            || !int_part.bytes().all(|b| b.is_ascii_digit())
            || (int_part.len() > 1 && int_part.starts_with('0'))
        {
            return Err(Fallback);
        }
        if int_part_end == digits.len() {
            return tok.parse::<i64>().map(Val::Int).map_err(|_| Fallback);
        }
        // Float: digits [. digits] [e [+-] digits].
        let mut rest = &digits[int_part_end..];
        if let Some(r) = rest.strip_prefix('.') {
            let n = r.bytes().take_while(u8::is_ascii_digit).count();
            if n == 0 {
                return Err(Fallback);
            }
            rest = &r[n..];
        }
        if let Some(r) = rest.strip_prefix(['e', 'E']) {
            let r = r.strip_prefix(['+', '-']).unwrap_or(r);
            if r.is_empty() || !r.bytes().all(|b| b.is_ascii_digit()) {
                return Err(Fallback);
            }
            rest = "";
        }
        if !rest.is_empty() {
            return Err(Fallback);
        }
        match tok.parse::<f64>() {
            Ok(f) if f.is_finite() => Ok(Val::Float(f)),
            _ => Err(Fallback),
        }
    }

    fn array(&mut self, depth: u32) -> R<Val<'a>> {
        self.i += 1;
        let mut items = Vec::new();
        loop {
            self.skip_space_comment_newline()?;
            if self.peek() == Some(b']') {
                self.i += 1;
                return Ok(Val::Arr { items, aot: false });
            }
            items.push(self.value(depth + 1)?);
            self.skip_space_comment_newline()?;
            match self.peek() {
                Some(b',') => self.i += 1,
                Some(b']') => {}
                _ => return Err(Fallback),
            }
        }
    }

    fn skip_space_comment_newline(&mut self) -> R<()> {
        loop {
            self.skip_blank();
            self.skip_comment();
            if !self.newline()? {
                return Ok(());
            }
        }
    }

    fn inline_table(&mut self, depth: u32) -> R<Val<'a>> {
        self.i += 1;
        let mut t = Tab::new(Kind::Inline);
        self.skip_blank();
        if self.peek() == Some(b'}') {
            self.i += 1;
            return Ok(Val::Tab(t));
        }
        loop {
            self.skip_blank();
            let keys = self.key_path()?;
            self.skip_blank();
            if self.peek() != Some(b'=') {
                return Err(Fallback);
            }
            self.i += 1;
            self.skip_blank();
            let v = self.value(depth + 1)?;
            insert_dotted(&mut t, keys, v)?;
            self.skip_blank();
            match self.peek() {
                Some(b',') => self.i += 1,
                Some(b'}') => {
                    self.i += 1;
                    return Ok(Val::Tab(t));
                }
                _ => return Err(Fallback),
            }
        }
    }
}

/// The table a `[header]` path names (creating implicit tables on the way). An array of
/// tables resolves to its last element.
fn descend<'t, 'a>(root: &'t mut Tab<'a>, path: &[Cow<'a, str>]) -> R<&'t mut Tab<'a>> {
    let mut t = root;
    for seg in path {
        let idx = match t.index_of(seg) {
            Some(i) => i,
            None => {
                t.entries
                    .push((seg.clone(), Val::Tab(Tab::new(Kind::ImplicitHeader))));
                t.entries.len() - 1
            }
        };
        t = match &mut t.entries[idx].1 {
            Val::Tab(sub) if matches!(sub.kind, Kind::Explicit | Kind::ImplicitHeader) => sub,
            Val::Arr { items, aot: true } => match items.last_mut() {
                Some(Val::Tab(sub)) => sub,
                _ => return Err(Fallback),
            },
            _ => return Err(Fallback),
        };
    }
    Ok(t)
}

/// `a.b.c = v` inside `table`: `a` and `b` are dotted tables, `c` must be new.
fn insert_dotted<'a>(table: &mut Tab<'a>, mut keys: Vec<Cow<'a, str>>, v: Val<'a>) -> R<()> {
    let last = keys.pop().ok_or(Fallback)?;
    let mut t = table;
    for seg in keys {
        let idx = match t.index_of(&seg) {
            Some(i) => i,
            None => {
                t.entries.push((seg, Val::Tab(Tab::new(Kind::Dotted))));
                t.entries.len() - 1
            }
        };
        t = match &mut t.entries[idx].1 {
            Val::Tab(sub) if sub.kind == Kind::Dotted => sub,
            _ => return Err(Fallback),
        };
    }
    if t.index_of(&last).is_some() {
        return Err(Fallback);
    }
    t.entries.push((last, v));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Sorted, kind-free dump so trees built by different parsers compare equal.
    fn dump(v: &Val<'_>) -> String {
        match v {
            Val::Str(s) => format!("{s:?}"),
            Val::Int(i) => format!("i{i}"),
            Val::Float(f) => format!("f{:#x}", f.to_bits()),
            Val::Bool(b) => b.to_string(),
            Val::Arr { items, .. } => {
                format!("[{}]", items.iter().map(dump).collect::<Vec<_>>().join(","))
            }
            Val::Tab(t) => dump_tab(t),
        }
    }

    fn dump_tab(t: &Tab<'_>) -> String {
        let mut e: Vec<String> = t
            .entries
            .iter()
            .map(|(k, v)| format!("{k:?}={}", dump(v)))
            .collect();
        e.sort();
        format!("{{{}}}", e.join(";"))
    }

    fn both(src: &str) -> (Option<String>, String) {
        let fast = Fast::new(src).document().ok().map(|t| dump_tab(&t));
        let slow = dump_tab(&parse_with_toml(src, "t").expect("valid toml"));
        (fast, slow)
    }

    /// The fast path must either decline or agree with the `toml` crate.
    fn agree(src: &str) -> bool {
        let (fast, slow) = both(src);
        match fast {
            Some(f) => {
                assert_eq!(f, slow, "{src}");
                true
            }
            None => false,
        }
    }

    #[test]
    fn fast_path_agrees_with_toml_on_the_subset() {
        let docs = [
            "a = 1\nb = \"x\"\nc = 1.5e3\nd = true\ne = [1, 2,\n 3, # c\n]\nf = { x = 1, y.z = \"q\" }\n",
            "[a]\nb = 1\n[a.c]\nd = 'lit'\n[[m]]\nx = 1\n[m.sub]\ny = 2\n[[m]]\nx = 3\n",
            "[a.b.c]\nv = 1\n[a]\nw = 2\n",
            "x.y.z = 1\nx.y.w = 2\n",
            "s = \"tab\\t \\\"q\\\" \\\\ \\u00e9 \\U0001F600\"\nt = \"ünï\"\n# comment ü\n",
            "\"quoted key\" = 1\n'lit key'.\"b\" = 2\n",
            "a = -5\nb = +7\nc = -0.5\nd = 1e-7\ne = 6.8E+2\n",
            "a=1\r\nb = \"x\"\r\n[t]\r\nc = [1]\r\n",
            "e = []\nt = {}\n",
            "[ a . b ]\nk = 1\n",
        ];
        for d in docs {
            assert!(agree(d), "fast path should handle: {d}");
        }
    }

    #[test]
    fn fast_path_declines_what_it_does_not_handle() {
        for d in [
            "a = \"\"\"multi\nline\"\"\"\n",
            "a = '''x'''\n",
            "a = 1979-05-27T07:32:00Z\n",
            "a = 0x10\n",
            "a = inf\n",
            "a = nan\n",
            "a = 1_000\n",
            "a = 007\n",
            "a = 1.\n",
            "a = .5\n",
            "a = { x = 1,\n y = 2 }\n",
            "a = { x = 1, }\n",
            "a = 1\na = 2\n",
            "[a]\n[a]\n",
            "a.b = 1\n[a]\nc = 2\n",
            "[a]\nb.c = 1\n[a.b]\nd = 2\n",
            "a = { b = 1 }\na.c = 2\n",
            "x = [1,, 2]\n",
            "x = \"unterminated\n",
            "x = \"bad \\q escape\"\n",
            "\u{FEFF}a = 1\n",
            "a = 1 b = 2\n",
            "a = 1\r\n\rb=2\n",
            "a = \"nul\u{1}\"\n",
            "a = 1e400\n",
            "x = tru\n",
        ] {
            assert!(Fast::new(d).document().is_err(), "must fall back: {d:?}");
        }
    }

    #[test]
    fn fallback_gives_toml_errors_and_trees() {
        let t = parse("a = 0x10\nb = 1979-05-27\n", "o").unwrap();
        assert_eq!(t.get("a"), Some(&Val::Int(16)));
        assert!(matches!(t.get("b"), Some(Val::Str(_))));
        match parse("a = [", "o") {
            Err(MaterialError::Toml { origin, span, .. }) => {
                assert_eq!(origin, "o");
                assert!(span.is_some());
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn fast_path_agrees_on_the_test_fixtures_and_builtin() {
        let full = include_str!("../../tests/fixtures/full.toml");
        let builtin = format!(
            "{}\n{}",
            include_str!("../../data/builtin/materials.toml"),
            include_str!("../../data/builtin/appearances.toml")
        );
        assert!(agree(full), "fixture must stay on the fast path");
        assert!(agree(&builtin), "builtin must stay on the fast path");
    }
}
