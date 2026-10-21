//! Unit-aware parameter expressions (S01 §5): a hand-written lexer and Pratt parser,
//! dimensional analysis and evaluation.
//!
//! - [`parse`] turns source text into an [`Ast`] (syntax only; references stay names).
//! - [`eval`] evaluates an [`Ast`] against an environment of named values, checking
//!   dimensions on the way. All values are canonical [`Quantity`]s (mm, t, s, K, rad; see
//!   `docs/notes/units-and-quantities.md`).
//! - [`references`] lists parameter references with their spans (rename, autocomplete).
//! - `Ast` implements `Display`, which prints canonical source that parses back to the same
//!   tree (round-trip tested).
//!
//! Single quantity literals and the unit table live in `amg_core::units`; this module only
//! adds the compound grammar on top (units are looked up with [`Unit::from_symbol`]).
//!
//! ```
//! use amg_doc::expr::{self, EvalOptions};
//! use amg_doc::UnitPrefs;
//! use amg_core::{Dimension, Quantity};
//! let ast = expr::parse("(d1 + 5) * 2")?;
//! let env = |name: &str| (name == "d1").then(|| Quantity::new(10.0, Dimension::LENGTH));
//! let opts = EvalOptions::new(&UnitPrefs::default()).expect(Dimension::LENGTH);
//! // Fusion-lenient: the bare `5` next to a length takes the document length unit.
//! assert_eq!(expr::eval(&ast, &env, &opts)?, Quantity::new(30.0, Dimension::LENGTH));
//! # Ok::<(), amg_doc::expr::ExprError>(())
//! ```

mod eval;
mod lex;
mod parse;

#[cfg(test)]
mod tests;

use std::ops::Range;

use amg_core::{Dimension, Quantity, Unit};

use crate::id::ParamId;
use crate::model::UnitPrefs;

pub use eval::eval;

/// Dimension of an expression value. S01 Interfaces called this `Dim` with 8 slots; FND-10
/// settled on [`amg_core::Dimension`] (length, mass, time, temperature, angle).
pub type Dim = Dimension;

/// A parsed expression.
#[derive(Clone, Debug, PartialEq)]
pub struct Ast {
    pub(crate) root: Node,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Node {
    pub(crate) kind: NodeKind,
    /// Byte span in the source.
    pub(crate) span: Range<usize>,
    /// Tree depth (1 for a leaf); bounded by the parser so recursion stays shallow.
    pub(crate) depth: u32,
}

impl Node {
    pub(crate) fn new(kind: NodeKind, span: Range<usize>) -> Node {
        let depth = 1 + match &kind {
            NodeKind::Neg(a) | NodeKind::Pos(a) => a.depth,
            NodeKind::Bin(_, a, b) => a.depth.max(b.depth),
            NodeKind::Call(_, args) => args.iter().map(|a| a.depth).max().unwrap_or(0),
            _ => 0,
        };
        Node { kind, span, depth }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum NodeKind {
    /// Bare numeric literal (no unit), `>= 0`.
    Num(f64),
    /// Literal with a unit expression: magnitude (`>= 0`, fractions and mixed numbers already
    /// folded) times the product of `unit^exp`.
    Qty {
        value: f64,
        units: Vec<(Unit, i8)>,
    },
    /// A unit used on its own (`* 1 mm` style): one of that unit.
    Unit(Unit),
    /// `PI`, `pi`, `E`.
    Const(Const),
    /// Parameter reference by name.
    Param(String),
    Neg(Box<Node>),
    Pos(Box<Node>),
    Bin(BinOp, Box<Node>, Box<Node>),
    Call(Func, Vec<Node>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BinOp {
    Add,
    Sub,
    Mul,
    Div,
    Pow,
}

impl BinOp {
    pub(crate) fn symbol(self) -> &'static str {
        match self {
            BinOp::Add => "+",
            BinOp::Sub => "-",
            BinOp::Mul => "*",
            BinOp::Div => "/",
            BinOp::Pow => "^",
        }
    }
}

/// Named constants.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Const {
    /// `PI`
    PiUpper,
    /// `pi`
    PiLower,
    /// `E`
    E,
}

impl Const {
    pub(crate) const ALL: [Const; 3] = [Const::PiUpper, Const::PiLower, Const::E];
    pub(crate) fn name(self) -> &'static str {
        match self {
            Const::PiUpper => "PI",
            Const::PiLower => "pi",
            Const::E => "E",
        }
    }
    pub(crate) fn value(self) -> f64 {
        match self {
            Const::PiUpper | Const::PiLower => std::f64::consts::PI,
            Const::E => std::f64::consts::E,
        }
    }
    pub(crate) fn from_name(s: &str) -> Option<Const> {
        Const::ALL.into_iter().find(|c| c.name() == s)
    }
}

/// Built-in functions (S01 §5 "Functions and constants").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Func {
    Sin,
    Cos,
    Tan,
    Asin,
    Acos,
    Atan,
    Atan2,
    Sqrt,
    Abs,
    Hypot,
    Min,
    Max,
    Floor,
    Ceil,
    Round,
    Sign,
    Exp,
    Ln,
    Log10,
    Pow,
}

impl Func {
    pub(crate) const ALL: [Func; 20] = [
        Func::Sin,
        Func::Cos,
        Func::Tan,
        Func::Asin,
        Func::Acos,
        Func::Atan,
        Func::Atan2,
        Func::Sqrt,
        Func::Abs,
        Func::Hypot,
        Func::Min,
        Func::Max,
        Func::Floor,
        Func::Ceil,
        Func::Round,
        Func::Sign,
        Func::Exp,
        Func::Ln,
        Func::Log10,
        Func::Pow,
    ];

    pub(crate) fn name(self) -> &'static str {
        match self {
            Func::Sin => "sin",
            Func::Cos => "cos",
            Func::Tan => "tan",
            Func::Asin => "asin",
            Func::Acos => "acos",
            Func::Atan => "atan",
            Func::Atan2 => "atan2",
            Func::Sqrt => "sqrt",
            Func::Abs => "abs",
            Func::Hypot => "hypot",
            Func::Min => "min",
            Func::Max => "max",
            Func::Floor => "floor",
            Func::Ceil => "ceil",
            Func::Round => "round",
            Func::Sign => "sign",
            Func::Exp => "exp",
            Func::Ln => "ln",
            Func::Log10 => "log10",
            Func::Pow => "pow",
        }
    }

    pub(crate) fn from_name(s: &str) -> Option<Func> {
        Func::ALL.into_iter().find(|f| f.name() == s)
    }

    /// (minimum, maximum) argument count; `None` = unbounded.
    pub(crate) fn arity(self) -> (usize, Option<usize>) {
        match self {
            Func::Atan2 | Func::Hypot | Func::Pow => (2, Some(2)),
            Func::Min | Func::Max => (1, None),
            _ => (1, Some(1)),
        }
    }
}

/// Why an expression or a parameter edit was rejected. Spans are byte ranges into the
/// expression source, for highlighting in the expression input widget (UI-05).
#[derive(thiserror::Error, Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum ExprError {
    /// The text is not a valid expression.
    #[error("syntax error at {span:?}: {msg}")]
    Parse {
        /// Offending bytes.
        span: Range<usize>,
        /// What is wrong.
        msg: String,
    },
    /// A name that is neither a parameter, a unit, a constant nor a function.
    #[error("unknown parameter `{name}`")]
    UnknownParam {
        /// The name as written.
        name: String,
        /// Its span.
        span: Range<usize>,
    },
    /// `+`, `-`, `min`, `max`, ... on operands of different dimensions.
    #[error("unit mismatch: {left} vs {right}")]
    UnitMismatch {
        /// Dimension of the left (or first) operand, shown as its canonical unit.
        left: Dim,
        /// Dimension of the right (or offending) operand.
        right: Dim,
        /// Span of the whole operation.
        span: Range<usize>,
    },
    /// The whole expression has the wrong dimension for its field or parameter unit.
    #[error("expected {expected}, got {got}")]
    WrongDimension {
        /// Required dimension (canonical unit).
        expected: Dim,
        /// Dimension of the result.
        got: Dim,
    },
    /// A function or operator outside its domain (`sqrt(-1)`, division by zero, overflow,
    /// a non-integer power of a dimensioned value, ...).
    #[error("{func}: {msg}")]
    Domain {
        /// Function or operator name (`sqrt`, `/`, `^`).
        func: &'static str,
        /// What is wrong.
        msg: String,
        /// Span of the call or operation.
        span: Range<usize>,
    },
    /// Setting the expression would make a parameter depend on itself. The path starts and
    /// ends with the edited parameter: `[a, b, a]`.
    #[error("circular reference: {0:?}")]
    Cycle(Vec<ParamId>),
    /// A unit, function or constant name used as a parameter name.
    #[error("`{0}` is a reserved name")]
    ReservedName(String),
    /// A parameter name that is not an identifier (`(letter | _) {letter | digit | _}`).
    #[error("`{0}` is not a valid parameter name")]
    InvalidName(String),
}

impl ExprError {
    /// Span for highlighting, if the error has one.
    pub fn span(&self) -> Option<Range<usize>> {
        match self {
            ExprError::Parse { span, .. }
            | ExprError::UnknownParam { span, .. }
            | ExprError::UnitMismatch { span, .. }
            | ExprError::Domain { span, .. } => Some(span.clone()),
            _ => None,
        }
    }
}

impl amg_core::Diagnostic for ExprError {
    fn hint(&self) -> Option<String> {
        Some(match self {
            ExprError::Parse { .. } => "Write an expression such as `d1 * 2 + 5 mm`.".to_owned(),
            ExprError::UnknownParam { name, .. } => {
                format!("Create a parameter named `{name}` or fix the spelling.")
            }
            ExprError::UnitMismatch { .. } => {
                "Only values of the same kind can be added or compared; check the units.".to_owned()
            }
            ExprError::WrongDimension { expected, .. } => {
                format!("The result must be in {expected} or a compatible unit.")
            }
            ExprError::Domain { .. } => return None,
            ExprError::Cycle(_) => "Parameters may not depend on themselves.".to_owned(),
            ExprError::ReservedName(_) => {
                "Unit, function and constant names (mm, in, s, sin, PI, ...) can't be used as \
                 parameter names."
                    .to_owned()
            }
            ExprError::InvalidName(_) => {
                "Names start with a letter or `_` and contain letters, digits and `_`.".to_owned()
            }
        })
    }
}

/// How [`eval`] treats bare numbers and the result dimension.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EvalOptions {
    /// Required dimension of the result (`None`: any).
    pub expected: Option<Dim>,
    /// Unit of the field or parameter. A unitless whole expression (`10`, `2 * 3`) is in this
    /// unit when it has the expected dimension.
    pub field_unit: Option<Unit>,
    /// Document length unit: bare numbers in length context.
    pub length_unit: Unit,
    /// Unit for bare numbers in angle context (`deg`).
    pub angle_unit: Unit,
    /// Strict mode: a bare number next to a length/angle in `+`/`-` is a unit error instead of
    /// taking the default unit (owner answer to S01 open question 3: lenient by default).
    pub strict: bool,
}

impl EvalOptions {
    /// Lenient options with the document's default units, no expected dimension.
    pub fn new(prefs: &UnitPrefs) -> EvalOptions {
        EvalOptions {
            expected: None,
            field_unit: None,
            length_unit: prefs.length.into(),
            angle_unit: Unit::Degree,
            strict: false,
        }
    }

    /// Requires the result to have dimension `d`.
    #[must_use]
    pub fn expect(mut self, d: Dim) -> EvalOptions {
        self.expected = Some(d);
        self
    }

    /// Field/parameter unit `u`: the result must have its dimension, and a unitless whole
    /// expression is read in `u`.
    #[must_use]
    pub fn unit(mut self, u: Unit) -> EvalOptions {
        self.field_unit = Some(u);
        self.expected = Some(u.dimension());
        self
    }

    /// Sets strict mode.
    #[must_use]
    pub fn strict(mut self, strict: bool) -> EvalOptions {
        self.strict = strict;
        self
    }

    /// The unit a bare number takes in a context of dimension `d`: the field unit if it
    /// matches, the document length unit, degrees, else the canonical unit of `d`.
    pub fn default_unit(&self, d: Dim) -> Option<Unit> {
        if let Some(u) = self.field_unit.filter(|u| u.dimension() == d) {
            return Some(u);
        }
        if d == Dimension::LENGTH {
            return Some(self.length_unit);
        }
        if d == Dimension::ANGLE {
            return Some(self.angle_unit);
        }
        canonical_unit(d)
    }
}

/// The unit with scale 1 and no offset for `d`, if the amg-core table has one.
pub(crate) fn canonical_unit(d: Dim) -> Option<Unit> {
    Unit::all()
        .iter()
        .copied()
        .find(|u| u.dimension() == d && u.to_canonical(1.0).to_bits() == 1f64.to_bits())
}

/// True for units with an offset (absolute `degC`, `degF`).
pub(crate) fn is_affine(u: Unit) -> bool {
    u.to_canonical(0.0) != 0.0
}

/// Parses `src`. Errors carry the byte span of the offending token.
pub fn parse(src: &str) -> Result<Ast, ExprError> {
    parse::parse(src)
}

/// Every parameter reference in source order, with its byte span.
pub fn references(ast: &Ast) -> Vec<(String, Range<usize>)> {
    let mut out = Vec::new();
    fn walk(n: &Node, out: &mut Vec<(String, Range<usize>)>) {
        match &n.kind {
            NodeKind::Param(name) => out.push((name.clone(), n.span.clone())),
            NodeKind::Neg(a) | NodeKind::Pos(a) => walk(a, out),
            NodeKind::Bin(_, a, b) => {
                walk(a, out);
                walk(b, out);
            }
            NodeKind::Call(_, args) => args.iter().for_each(|a| walk(a, out)),
            NodeKind::Num(_) | NodeKind::Qty { .. } | NodeKind::Unit(_) | NodeKind::Const(_) => {}
        }
    }
    walk(&ast.root, &mut out);
    out
}

/// True if `name` is a unit spelling, function or constant (not usable as a parameter name).
pub fn is_reserved(name: &str) -> bool {
    Unit::from_symbol(name).is_some()
        || Func::from_name(name).is_some()
        || Const::from_name(name).is_some()
}

/// Checks that `name` can name a parameter: an identifier that is not reserved.
pub fn check_name(name: &str) -> Result<(), ExprError> {
    if !lex::is_ident(name) {
        return Err(ExprError::InvalidName(name.to_owned()));
    }
    if is_reserved(name) {
        return Err(ExprError::ReservedName(name.to_owned()));
    }
    Ok(())
}

/// Every reserved identifier (unit spellings that are identifiers, functions, constants),
/// sorted. For autocomplete and documentation.
pub fn reserved_names() -> Vec<&'static str> {
    let mut v: Vec<&'static str> = Unit::all()
        .iter()
        .flat_map(|u| std::iter::once(u.symbol()).chain(u.aliases().iter().copied()))
        .filter(|s| lex::is_ident(s))
        .chain(Func::ALL.iter().map(|f| f.name()))
        .chain(Const::ALL.iter().map(|c| c.name()))
        .collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Formats a canonical value in `unit` with `precision` decimals (trailing zeros trimmed),
/// through amg-core's unit formatter. The text parses back as an expression.
pub fn format_value(q: Quantity, unit: Unit, precision: u8) -> Result<String, amg_core::Error> {
    let spec = amg_core::FormatSpec {
        style: amg_core::FormatStyle::Decimal,
        precision,
        trim_zeros: true,
        show_unit: true,
    };
    amg_core::format_quantity(q, unit, &spec)
}

// ---- printing ---------------------------------------------------------------------------

/// The spelling printed for a unit: the first one that lexes as a single token, so the
/// printed text parses back to the same unit (`cc`, not `cm^3`).
pub(crate) fn unit_spelling(u: Unit) -> &'static str {
    std::iter::once(u.symbol())
        .chain(u.aliases().iter().copied())
        .find(|s| lex::is_ident(s))
        .unwrap_or_else(|| match u {
            Unit::Foot => "'",
            Unit::Inch => "\"",
            Unit::Degree => "°",
            Unit::DegreeCelsius => "°C",
            Unit::DegreeFahrenheit => "°F",
            other => other.symbol(),
        })
}

/// Shortest text that parses back to `x` exactly (`x >= 0`, finite).
pub(crate) fn number_text(x: f64) -> String {
    let a = x.abs();
    if a == 0.0 || (1e-5..1e16).contains(&a) {
        format!("{x}")
    } else {
        format!("{x:e}")
    }
}

/// Printing precedence: 1 sum, 2 product, 3 unary, 4 power, 5 atom.
fn prec(n: &Node) -> u8 {
    match &n.kind {
        NodeKind::Bin(BinOp::Add | BinOp::Sub, ..) => 1,
        NodeKind::Bin(BinOp::Mul | BinOp::Div, ..) => 2,
        NodeKind::Neg(_) | NodeKind::Pos(_) => 3,
        NodeKind::Bin(BinOp::Pow, ..) => 4,
        _ => 5,
    }
}

/// True if the printed form of `n` starts with a unit token (it would be absorbed into a
/// preceding quantity's unit expression after `*` or `/`).
fn starts_with_unit(n: &Node) -> bool {
    match &n.kind {
        NodeKind::Unit(_) => true,
        NodeKind::Bin(_, a, _) => prec(a) >= prec(n) && starts_with_unit(a),
        _ => false,
    }
}

fn write_node(n: &Node, f: &mut String) {
    let paren = |c: &Node, need: bool, f: &mut String| {
        if need {
            f.push('(');
            write_node(c, f);
            f.push(')');
        } else {
            write_node(c, f);
        }
    };
    match &n.kind {
        NodeKind::Num(v) => f.push_str(&number_text(*v)),
        NodeKind::Qty { value, units } => {
            f.push_str(&number_text(*value));
            f.push(' ');
            for (i, &(u, e)) in units.iter().enumerate() {
                let e = if i == 0 {
                    e
                } else if e < 0 {
                    f.push('/');
                    e.saturating_neg()
                } else {
                    f.push('*');
                    e
                };
                f.push_str(unit_spelling(u));
                if e != 1 {
                    f.push('^');
                    f.push_str(&e.to_string());
                }
            }
        }
        NodeKind::Unit(u) => f.push_str(unit_spelling(*u)),
        NodeKind::Const(c) => f.push_str(c.name()),
        NodeKind::Param(p) => f.push_str(p),
        NodeKind::Neg(a) | NodeKind::Pos(a) => {
            f.push(if matches!(n.kind, NodeKind::Neg(_)) {
                '-'
            } else {
                '+'
            });
            paren(a, prec(a) < 3, f);
        }
        NodeKind::Bin(op, a, b) => {
            let (la, lb) = match op {
                BinOp::Add | BinOp::Sub => (1, 2),
                BinOp::Mul | BinOp::Div => (2, 3),
                BinOp::Pow => (5, 3),
            };
            let qty = |x: &Node| matches!(x.kind, NodeKind::Qty { .. });
            // A quantity base needs parens: `(10 mm)^2` is not `10 mm^2`.
            paren(a, prec(a) < la || (*op == BinOp::Pow && qty(a)), f);
            if *op == BinOp::Pow {
                f.push('^');
            } else {
                f.push(' ');
                f.push_str(op.symbol());
                f.push(' ');
            }
            let need_b = prec(b) < lb
                // `1 / (4 in)`: without parens `1/4 in` is the fraction literal.
                || (*op == BinOp::Div && qty(b))
                // `(10 mm) * s` would otherwise read as the unit `mm*s`.
                || (matches!(op, BinOp::Mul | BinOp::Div) && starts_with_unit(b));
            paren(b, need_b, f);
        }
        NodeKind::Call(func, args) => {
            f.push_str(func.name());
            f.push('(');
            for (i, a) in args.iter().enumerate() {
                if i > 0 {
                    f.push_str(", ");
                }
                write_node(a, f);
            }
            f.push(')');
        }
    }
}

impl std::fmt::Display for Ast {
    /// Canonical source text; parsing it gives the same tree (spans aside).
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut s = String::new();
        write_node(&self.root, &mut s);
        f.write_str(&s)
    }
}

impl Ast {
    /// Byte span of the whole expression.
    pub fn span(&self) -> Range<usize> {
        self.root.span.clone()
    }

    /// The tree with all spans zeroed (structural comparison in tests).
    #[cfg(test)]
    pub(crate) fn without_spans(&self) -> Ast {
        fn strip(n: &Node) -> Node {
            let kind = match &n.kind {
                NodeKind::Neg(a) => NodeKind::Neg(Box::new(strip(a))),
                NodeKind::Pos(a) => NodeKind::Pos(Box::new(strip(a))),
                NodeKind::Bin(op, a, b) => {
                    NodeKind::Bin(*op, Box::new(strip(a)), Box::new(strip(b)))
                }
                NodeKind::Call(f, args) => NodeKind::Call(*f, args.iter().map(strip).collect()),
                k => k.clone(),
            };
            Node::new(kind, 0..0)
        }
        Ast {
            root: strip(&self.root),
        }
    }
}
