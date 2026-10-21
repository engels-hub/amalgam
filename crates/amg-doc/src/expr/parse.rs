//! Pratt parser for the S01 §5 grammar.
//!
//! Binding powers (left, right): `+ -` (1, 2), `* /` (3, 4), prefix `- +` (_, 5),
//! `^` (8, 7: right-associative, and its right operand may be a prefix `-`). Quantity
//! literals (`10 mm`, `1/4 in`, `1 1/4"`, `5' 3"`, `9.81 m/s^2`) are atoms, so a unit binds
//! tighter than every operator: `d1 / 5 mm` is `d1 / (5 mm)` and `10 mm^2` is ten square
//! millimetres.

use std::ops::Range;

use amg_core::Unit;

use super::lex::{Tok, Token, lex};
use super::{Ast, BinOp, Const, ExprError, Func, Node, NodeKind, is_affine};

/// Limit for parser recursion (parentheses, prefix operators, calls) and for the depth of
/// the tree, so parsing, evaluation and printing never recurse deeply.
const MAX_DEPTH: u32 = 256;

pub(crate) fn parse(src: &str) -> Result<Ast, ExprError> {
    let toks = lex(src)?;
    if toks.len() == 1 {
        return Err(err(0..src.len(), "empty expression"));
    }
    let mut p = Parser {
        src,
        toks,
        pos: 0,
        depth: 0,
    };
    let root = p.expr(0)?;
    if p.peek() != Tok::Eof {
        return Err(p.unexpected());
    }
    check_affine(&root, true)?;
    Ok(Ast { root })
}

fn err(span: Range<usize>, msg: impl Into<String>) -> ExprError {
    ExprError::Parse {
        span,
        msg: msg.into(),
    }
}

struct Parser<'a> {
    src: &'a str,
    toks: Vec<Token>,
    pos: usize,
    depth: u32,
}

impl Parser<'_> {
    fn peek(&self) -> Tok {
        self.peek_at(0)
    }
    fn peek_at(&self, k: usize) -> Tok {
        self.toks.get(self.pos + k).map_or(Tok::Eof, |t| t.tok)
    }
    fn span_at(&self, k: usize) -> Range<usize> {
        self.toks
            .get(self.pos + k)
            .or(self.toks.last())
            .map_or(0..0, |t| t.span.clone())
    }
    fn text_at(&self, k: usize) -> &str {
        &self.src[self.span_at(k)]
    }
    fn bump(&mut self) -> Token {
        let t = self.toks[self.pos.min(self.toks.len() - 1)].clone();
        if self.pos < self.toks.len() - 1 {
            self.pos += 1;
        }
        t
    }

    /// Builds a node, rejecting trees deeper than [`MAX_DEPTH`].
    fn node(&self, kind: NodeKind, span: Range<usize>) -> Result<Node, ExprError> {
        let n = Node::new(kind, span);
        if n.depth > MAX_DEPTH {
            return Err(err(n.span, "expression is nested too deeply"));
        }
        Ok(n)
    }

    fn enter(&mut self) -> Result<(), ExprError> {
        self.depth += 1;
        if self.depth > MAX_DEPTH {
            return Err(err(self.span_at(0), "expression is nested too deeply"));
        }
        Ok(())
    }

    /// Error for the token at the cursor.
    fn unexpected(&self) -> ExprError {
        let span = self.span_at(0);
        let text = self.text_at(0);
        match self.peek() {
            Tok::Eof => err(span, "unexpected end of expression"),
            Tok::Comma => {
                let b = self.src.as_bytes();
                let digit =
                    |i: Option<usize>| i.and_then(|i| b.get(i)).is_some_and(u8::is_ascii_digit);
                if digit(span.start.checked_sub(1)) && digit(Some(span.end)) {
                    err(span, "use `.` as decimal separator")
                } else {
                    err(span, "unexpected `,`")
                }
            }
            Tok::Num { .. } => err(span, format!("missing operator before `{text}`")),
            Tok::Ident
                if self.pos > 0 && matches!(self.toks[self.pos - 1].tok, Tok::Num { .. }) =>
            {
                err(span, format!("`{text}` is not a unit; write `* {text}`"))
            }
            Tok::Ident | Tok::Sym(_) | Tok::LParen => {
                err(span, format!("missing operator before `{text}`"))
            }
            _ => err(span, format!("unexpected `{text}`")),
        }
    }

    fn expr(&mut self, min_bp: u8) -> Result<Node, ExprError> {
        self.enter()?;
        let mut lhs = self.prefix()?;
        loop {
            let (op, lbp, rbp) = match self.peek() {
                Tok::Plus => (BinOp::Add, 1, 2),
                Tok::Minus => (BinOp::Sub, 1, 2),
                Tok::Star => (BinOp::Mul, 3, 4),
                Tok::Slash => (BinOp::Div, 3, 4),
                Tok::Caret => (BinOp::Pow, 8, 7),
                _ => break,
            };
            if lbp < min_bp {
                break;
            }
            self.bump();
            let rhs = self.expr(rbp)?;
            let span = lhs.span.start..rhs.span.end;
            lhs = self.node(NodeKind::Bin(op, Box::new(lhs), Box::new(rhs)), span)?;
        }
        self.depth -= 1;
        Ok(lhs)
    }

    fn prefix(&mut self) -> Result<Node, ExprError> {
        match self.peek() {
            Tok::Minus | Tok::Plus => {
                let t = self.bump();
                let inner = self.expr(5)?;
                let span = t.span.start..inner.span.end;
                let kind = if t.tok == Tok::Minus {
                    NodeKind::Neg(Box::new(inner))
                } else {
                    NodeKind::Pos(Box::new(inner))
                };
                self.node(kind, span)
            }
            _ => self.atom(),
        }
    }

    fn atom(&mut self) -> Result<Node, ExprError> {
        match self.peek() {
            Tok::Num { .. } => self.quantity(true),
            Tok::Sym(u) => {
                let t = self.bump();
                Ok(Node::new(NodeKind::Unit(u), t.span))
            }
            Tok::LParen => {
                let open = self.bump();
                let mut inner = self.expr(0)?;
                if self.peek() != Tok::RParen {
                    if self.peek() == Tok::Eof {
                        return Err(err(open.span, "unclosed `(`"));
                    }
                    return Err(self.unexpected());
                }
                let close = self.bump();
                inner.span = open.span.start..close.span.end;
                Ok(inner)
            }
            Tok::Ident => self.ident(),
            _ => Err(self.unexpected()),
        }
    }

    fn ident(&mut self) -> Result<Node, ExprError> {
        let t = self.bump();
        let name = &self.src[t.span.clone()];
        let call = self.peek() == Tok::LParen;
        if let Some(func) = Func::from_name(name) {
            if call {
                return self.call(func, t.span);
            }
            if Unit::from_symbol(name).is_none() {
                return Err(err(
                    t.span,
                    format!("`{name}` needs arguments: `{name}(...)`"),
                ));
            }
        }
        if call {
            return Err(err(t.span, format!("unknown function `{name}`")));
        }
        let kind = if let Some(u) = Unit::from_symbol(name) {
            NodeKind::Unit(u)
        } else if let Some(c) = Const::from_name(name) {
            NodeKind::Const(c)
        } else {
            NodeKind::Param(name.to_owned())
        };
        Ok(Node::new(kind, t.span))
    }

    fn call(&mut self, func: Func, name_span: Range<usize>) -> Result<Node, ExprError> {
        self.enter()?;
        self.bump(); // `(`
        let mut args = Vec::new();
        if self.peek() != Tok::RParen {
            loop {
                args.push(self.expr(0)?);
                match self.peek() {
                    Tok::Comma => {
                        self.bump();
                    }
                    Tok::RParen => break,
                    Tok::Eof => return Err(err(name_span, format!("unclosed `{}(`", func.name()))),
                    _ => return Err(self.unexpected()),
                }
            }
        }
        let close = self.bump();
        let span = name_span.start..close.span.end;
        let (min, max) = func.arity();
        let n = args.len();
        if n < min || max.is_some_and(|m| n > m) {
            let want = match (min, max) {
                (a, Some(b)) if a == b => format!("{a}"),
                (a, None) => format!("at least {a}"),
                (a, Some(b)) => format!("{a} to {b}"),
            };
            return Err(err(
                span,
                format!("`{}` takes {want} argument(s), got {n}", func.name()),
            ));
        }
        self.depth -= 1;
        self.node(NodeKind::Call(func, args), span)
    }

    /// True if the token `k` ahead starts a unit: a punctuation unit, or a unit identifier
    /// that is not a function call (`min(` is the function, `10 min` the unit).
    fn unit_start(&self, k: usize) -> bool {
        match self.peek_at(k) {
            Tok::Sym(_) => true,
            Tok::Ident => {
                let s = self.text_at(k);
                Unit::from_symbol(s).is_some()
                    && !(Func::from_name(s).is_some() && self.peek_at(k + 1) == Tok::LParen)
            }
            _ => false,
        }
    }

    fn int_at(&self, k: usize) -> Option<f64> {
        match self.peek_at(k) {
            Tok::Num { value, int: true } => Some(value),
            _ => None,
        }
    }

    /// A number with optional fraction / mixed-number form, unit expression and feet-inch
    /// continuation. The cursor is on the number.
    fn quantity(&mut self, allow_feet_inch: bool) -> Result<Node, ExprError> {
        let first = self.bump();
        let Tok::Num { value, int } = first.tok else {
            unreachable!("quantity() starts on a number");
        };
        let start = first.span.start;
        let mut end = first.span.end;
        let mut mag = value;
        if int {
            if let (Some(n), Tok::Slash, Some(d)) =
                (self.int_at(0), self.peek_at(1), self.int_at(2))
            {
                // Mixed number `1 1/4`.
                let span = self.span_at(0).start..self.span_at(2).end;
                check_den(d, &span)?;
                mag = value + n / d;
                end = span.end;
                self.pos += 3;
            } else if let (Tok::Slash, Some(d)) = (self.peek_at(0), self.int_at(1)) {
                // Fraction literal `1/4 in`: only when a unit follows, else plain division.
                if self.unit_start(2) {
                    let span = start..self.span_at(1).end;
                    check_den(d, &span)?;
                    mag = value / d;
                    end = span.end;
                    self.pos += 2;
                }
            }
        }
        if !mag.is_finite() {
            return Err(err(start..end, "number is too large"));
        }
        let units = self.unit_expr()?;
        if let Some(t) = self
            .toks
            .get(self.pos.wrapping_sub(1))
            .filter(|_| !units.is_empty())
        {
            end = t.span.end;
        }
        let kind = if units.is_empty() {
            NodeKind::Num(mag)
        } else {
            NodeKind::Qty { value: mag, units }
        };
        let node = Node::new(kind, start..end);
        if allow_feet_inch
            && matches!(&node.kind, NodeKind::Qty { units, .. } if units == &[(Unit::Foot, 1)])
            && matches!(self.peek(), Tok::Num { .. })
        {
            // `5' 3"` / `5 ft 3 1/2 in`: feet followed directly by inches is their sum.
            let save = self.pos;
            if let Ok(inches) = self.quantity(false)
                && matches!(&inches.kind, NodeKind::Qty { units, .. } if units == &[(Unit::Inch, 1)])
            {
                let span = node.span.start..inches.span.end;
                return Ok(Node::new(
                    NodeKind::Bin(BinOp::Add, Box::new(node), Box::new(inches)),
                    span,
                ));
            }
            self.pos = save;
        }
        Ok(node)
    }

    /// `unit [^ int | ²] { (* | /) unit [^ int | ²] }`; empty if no unit follows.
    fn unit_expr(&mut self) -> Result<Vec<(Unit, i8)>, ExprError> {
        let mut units = Vec::new();
        if !self.unit_start(0) {
            return Ok(units);
        }
        let mut sign: i8 = 1;
        loop {
            let t = self.bump();
            let u = match t.tok {
                Tok::Sym(u) => u,
                _ => Unit::from_symbol(&self.src[t.span.clone()])
                    .ok_or_else(|| err(t.span.clone(), "unknown unit"))?,
            };
            let mut e: i8 = 1;
            match self.peek() {
                Tok::Sup(n) => {
                    self.bump();
                    e = n;
                }
                Tok::Caret => {
                    let caret = self.bump();
                    let neg = match self.peek() {
                        Tok::Minus => {
                            self.bump();
                            true
                        }
                        Tok::Plus => {
                            self.bump();
                            false
                        }
                        _ => false,
                    };
                    let span = caret.span.start..self.span_at(0).end;
                    let n = self
                        .int_at(0)
                        .filter(|&n| (1.0..=127.0).contains(&n))
                        .ok_or_else(|| {
                            err(
                                span.clone(),
                                "a unit exponent must be an integer from 1 to 127",
                            )
                        })?;
                    self.bump();
                    // 1..=127, so the cast is exact.
                    #[allow(clippy::cast_possible_truncation)]
                    let n = n as i8;
                    e = if neg { -n } else { n };
                }
                _ => {}
            }
            units.push((u, sign * e));
            match self.peek() {
                Tok::Star if self.unit_start(1) => sign = 1,
                Tok::Slash if self.unit_start(1) => sign = -1,
                _ => break,
            }
            self.bump();
        }
        Ok(units)
    }
}

fn check_den(d: f64, span: &Range<usize>) -> Result<(), ExprError> {
    if d == 0.0 {
        Err(err(span.clone(), "division by zero in fraction"))
    } else {
        Ok(())
    }
}

/// Affine units (`degC`, `degF`) may only be the sole unit of the whole expression
/// (`20 degC`, `-40 degF`): S01 §5 Dimensions.
fn check_affine(n: &Node, top: bool) -> Result<(), ExprError> {
    let bad = |span: &Range<usize>| {
        err(
            span.clone(),
            "degC/degF can only be the single unit of a whole expression; use K or delta_degC \
             for temperature differences",
        )
    };
    match &n.kind {
        NodeKind::Qty { units, .. } => {
            let affine = units.iter().any(|&(u, _)| is_affine(u));
            if affine && !(top && units.len() == 1 && units[0].1 == 1) {
                return Err(bad(&n.span));
            }
        }
        NodeKind::Unit(u) if is_affine(*u) => return Err(bad(&n.span)),
        NodeKind::Neg(a) | NodeKind::Pos(a) => check_affine(a, top)?,
        NodeKind::Bin(_, a, b) => {
            check_affine(a, false)?;
            check_affine(b, false)?;
        }
        NodeKind::Call(_, args) => {
            for a in args {
                check_affine(a, false)?;
            }
        }
        _ => {}
    }
    Ok(())
}
