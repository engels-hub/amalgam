//! Evaluation with dimensional analysis (S01 §5 Semantics, Dimensions, Bare numbers).

use std::ops::Range;

use amg_core::{Dimension, Quantity};

use super::{Ast, BinOp, Dim, EvalOptions, ExprError, Func, Node, NodeKind, is_affine};

/// Evaluates `ast`. `env` resolves parameter names to canonical values (`None` gives
/// [`ExprError::UnknownParam`]).
///
/// - Dimensions are checked on every operation, with the span of the offending node.
/// - Every intermediate value is finite (overflow and division by zero are
///   [`ExprError::Domain`]).
/// - Bare numbers: a unitless whole expression takes the field's default unit
///   ([`EvalOptions::default_unit`]); unless `strict`, a unitless operand of `+`/`-` next to
///   a length or angle takes the document length unit / degrees (`d1 + 5` = `d1 + 5 mm`).
/// - The result must have `opts.expected` if set ([`ExprError::WrongDimension`]).
pub fn eval(
    ast: &Ast,
    env: &dyn Fn(&str) -> Option<Quantity>,
    opts: &EvalOptions,
) -> Result<Quantity, ExprError> {
    let ev = Eval { env, opts };
    let root = &ast.root;
    let mut q = match affine_literal(root) {
        Some(q) => q,
        None => ev.node(root)?,
    };
    if let Some(expected) = opts.expected {
        if q.dim == Dimension::NONE
            && expected != Dimension::NONE
            && is_literal(root)
            && let Some(u) = opts.default_unit(expected)
        {
            q = Quantity::new(u.to_canonical(q.value), expected);
            finite(q, "unit", &root.span)?;
        }
        if q.dim != expected {
            return Err(ExprError::WrongDimension {
                expected,
                got: q.dim,
            });
        }
    }
    Ok(q)
}

/// `20 degC`, `-40 degF` (possibly with several signs): the affine conversion applies to the
/// signed number.
fn affine_literal(n: &Node) -> Option<Quantity> {
    let mut sign = 1.0;
    let mut cur = n;
    loop {
        match &cur.kind {
            NodeKind::Neg(a) => {
                sign = -sign;
                cur = a;
            }
            NodeKind::Pos(a) => cur = a,
            NodeKind::Qty { value, units } if units.len() == 1 && is_affine(units[0].0) => {
                let u = units[0].0;
                return Some(Quantity::new(u.to_canonical(sign * value), u.dimension()));
            }
            _ => return None,
        }
    }
}

/// Unit-free literal arithmetic: numbers, constants, signs and operators only.
fn is_literal(n: &Node) -> bool {
    match &n.kind {
        NodeKind::Num(_) | NodeKind::Const(_) => true,
        NodeKind::Neg(a) | NodeKind::Pos(a) => is_literal(a),
        NodeKind::Bin(_, a, b) => is_literal(a) && is_literal(b),
        _ => false,
    }
}

fn finite(q: Quantity, func: &'static str, span: &Range<usize>) -> Result<Quantity, ExprError> {
    if q.value.is_finite() {
        Ok(q)
    } else {
        Err(ExprError::Domain {
            func,
            msg: "the result is not a finite number".to_owned(),
            span: span.clone(),
        })
    }
}

fn domain(func: &'static str, msg: impl Into<String>, span: &Range<usize>) -> ExprError {
    ExprError::Domain {
        func,
        msg: msg.into(),
        span: span.clone(),
    }
}

/// Exponent-wise combination with range checking (amg-core saturates silently).
fn combine(
    a: Dim,
    b: Dim,
    f: impl Fn(i16, i16) -> i16,
    func: &'static str,
    span: &Range<usize>,
) -> Result<Dim, ExprError> {
    let g = |x: i8, y: i8| {
        i8::try_from(f(i16::from(x), i16::from(y)))
            .map_err(|_| domain(func, "unit exponent out of range", span))
    };
    Ok(Dimension::new(
        g(a.length, b.length)?,
        g(a.mass, b.mass)?,
        g(a.time, b.time)?,
        g(a.temperature, b.temperature)?,
        g(a.angle, b.angle)?,
    ))
}

fn exps(d: Dim) -> [i8; 5] {
    [d.length, d.mass, d.time, d.temperature, d.angle]
}

struct Eval<'a> {
    env: &'a dyn Fn(&str) -> Option<Quantity>,
    opts: &'a EvalOptions,
}

impl Eval<'_> {
    fn node(&self, n: &Node) -> Result<Quantity, ExprError> {
        let span = &n.span;
        match &n.kind {
            NodeKind::Num(v) => Ok(Quantity::dimensionless(*v)),
            NodeKind::Const(c) => Ok(Quantity::dimensionless(c.value())),
            NodeKind::Unit(u) => Ok(Quantity::from_unit(1.0, *u)),
            NodeKind::Qty { value, units } => {
                let mut q = Quantity::dimensionless(*value);
                for &(u, e) in units {
                    let one = Quantity::from_unit(1.0, u);
                    let dim = combine(
                        one.dim,
                        Dimension::NONE,
                        |a, _| a * i16::from(e),
                        "unit",
                        span,
                    )?;
                    let f = Quantity::new(one.value.powi(i32::from(e)), dim);
                    q = Quantity::new(
                        q.value * f.value,
                        combine(q.dim, f.dim, |a, b| a + b, "unit", span)?,
                    );
                }
                finite(q, "unit", span)
            }
            NodeKind::Param(name) => (self.env)(name).ok_or_else(|| ExprError::UnknownParam {
                name: name.clone(),
                span: span.clone(),
            }),
            NodeKind::Neg(a) => Ok(-self.node(a)?),
            NodeKind::Pos(a) => self.node(a),
            NodeKind::Bin(op, a, b) => self.bin(*op, a, b, span),
            NodeKind::Call(f, args) => self.call(*f, args, span),
        }
    }

    fn bin(
        &self,
        op: BinOp,
        a: &Node,
        b: &Node,
        span: &Range<usize>,
    ) -> Result<Quantity, ExprError> {
        let mut l = self.node(a)?;
        let mut r = self.node(b)?;
        match op {
            BinOp::Add | BinOp::Sub => {
                if !self.opts.strict {
                    self.lenient(&mut r, b, l.dim);
                    self.lenient(&mut l, a, r.dim);
                }
                if l.dim != r.dim {
                    return Err(ExprError::UnitMismatch {
                        left: l.dim,
                        right: r.dim,
                        span: span.clone(),
                    });
                }
                let v = if op == BinOp::Add {
                    l.value + r.value
                } else {
                    l.value - r.value
                };
                finite(Quantity::new(v, l.dim), op.symbol(), span)
            }
            BinOp::Mul => {
                let dim = combine(l.dim, r.dim, |x, y| x + y, "*", span)?;
                finite(Quantity::new(l.value * r.value, dim), "*", span)
            }
            BinOp::Div => {
                if r.value == 0.0 {
                    return Err(domain("/", "division by zero", span));
                }
                let dim = combine(l.dim, r.dim, |x, y| x - y, "/", span)?;
                finite(Quantity::new(l.value / r.value, dim), "/", span)
            }
            BinOp::Pow => self.pow(l, r, span),
        }
    }

    /// Fusion leniency: a unitless literal operand next to a length or angle takes the
    /// document default unit of that dimension.
    fn lenient(&self, q: &mut Quantity, n: &Node, other: Dim) {
        if q.dim == Dimension::NONE
            && (other == Dimension::LENGTH || other == Dimension::ANGLE)
            && is_literal(n)
            && let Some(u) = self.opts.default_unit(other)
        {
            *q = Quantity::new(u.to_canonical(q.value), other);
        }
    }

    fn pow(
        &self,
        base: Quantity,
        exp: Quantity,
        span: &Range<usize>,
    ) -> Result<Quantity, ExprError> {
        if exp.dim != Dimension::NONE {
            return Err(domain(
                "^",
                format!("the exponent must be dimensionless, got {}", exp.dim),
                span,
            ));
        }
        let mut dim = Dimension::NONE;
        if base.dim != Dimension::NONE {
            let e = exp.value;
            if e.fract() != 0.0 || e.abs() > 127.0 {
                return Err(domain(
                    "^",
                    format!(
                        "a value in {} can only be raised to an integer power",
                        base.dim
                    ),
                    span,
                ));
            }
            // Integer-valued and within ±127.
            #[allow(clippy::cast_possible_truncation)]
            let n = e as i16;
            dim = combine(base.dim, Dimension::NONE, |a, _| a * n, "^", span)?;
        }
        let v = base.value.powf(exp.value);
        if v.is_nan() {
            return Err(domain(
                "^",
                "a negative number to a fractional power is undefined",
                span,
            ));
        }
        finite(Quantity::new(v, dim), "^", span)
    }

    fn call(&self, f: Func, args: &[Node], span: &Range<usize>) -> Result<Quantity, ExprError> {
        let vals = args
            .iter()
            .map(|a| self.node(a))
            .collect::<Result<Vec<_>, _>>()?;
        let name = f.name();
        let none = Dimension::NONE;
        let x = vals[0];
        let unitless = |q: Quantity| {
            if q.dim == none {
                Ok(q.value)
            } else {
                Err(domain(
                    name,
                    format!("expects a dimensionless value, got {}", q.dim),
                    span,
                ))
            }
        };
        let same = |qs: &[Quantity]| -> Result<Dim, ExprError> {
            let d = qs[0].dim;
            match qs.iter().find(|q| q.dim != d) {
                Some(q) => Err(ExprError::UnitMismatch {
                    left: d,
                    right: q.dim,
                    span: span.clone(),
                }),
                None => Ok(d),
            }
        };
        let angle = Dimension::ANGLE;
        let q = match f {
            Func::Sin | Func::Cos | Func::Tan => {
                if x.dim != angle && x.dim != none {
                    return Err(domain(
                        name,
                        format!("expects an angle, got {}", x.dim),
                        span,
                    ));
                }
                let v = match f {
                    Func::Sin => x.value.sin(),
                    Func::Cos => x.value.cos(),
                    _ => x.value.tan(),
                };
                Quantity::dimensionless(v)
            }
            Func::Asin | Func::Acos => {
                let v = unitless(x)?;
                if !(-1.0..=1.0).contains(&v) {
                    return Err(domain(name, format!("{v} is outside -1..1"), span));
                }
                Quantity::new(if f == Func::Asin { v.asin() } else { v.acos() }, angle)
            }
            Func::Atan => Quantity::new(unitless(x)?.atan(), angle),
            Func::Atan2 => {
                same(&vals)?;
                Quantity::new(x.value.atan2(vals[1].value), angle)
            }
            Func::Sqrt => {
                if exps(x.dim).iter().any(|e| e % 2 != 0) {
                    return Err(domain(
                        name,
                        format!("{} has no square root unit", x.dim),
                        span,
                    ));
                }
                if x.value < 0.0 {
                    return Err(domain(name, "negative argument", span));
                }
                let dim = combine(x.dim, none, |a, _| a / 2, name, span)?;
                Quantity::new(x.value.sqrt(), dim)
            }
            Func::Abs => Quantity::new(x.value.abs(), x.dim),
            Func::Hypot => {
                let d = same(&vals)?;
                Quantity::new(x.value.hypot(vals[1].value), d)
            }
            Func::Min | Func::Max => {
                let d = same(&vals)?;
                let pick = vals
                    .iter()
                    .map(|q| q.value)
                    .reduce(|a, b| if f == Func::Min { a.min(b) } else { a.max(b) });
                Quantity::new(pick.unwrap_or(x.value), d)
            }
            Func::Floor => Quantity::dimensionless(unitless(x)?.floor()),
            Func::Ceil => Quantity::dimensionless(unitless(x)?.ceil()),
            Func::Round => Quantity::dimensionless(unitless(x)?.round()),
            Func::Sign => {
                let v = unitless(x)?;
                Quantity::dimensionless(if v > 0.0 {
                    1.0
                } else if v < 0.0 {
                    -1.0
                } else {
                    0.0
                })
            }
            Func::Exp => Quantity::dimensionless(unitless(x)?.exp()),
            Func::Ln | Func::Log10 => {
                let v = unitless(x)?;
                if v <= 0.0 {
                    return Err(domain(name, "the argument must be positive", span));
                }
                Quantity::dimensionless(if f == Func::Ln { v.ln() } else { v.log10() })
            }
            Func::Pow => {
                let b = unitless(x)?;
                let e = unitless(vals[1])?;
                let v = b.powf(e);
                if v.is_nan() {
                    return Err(domain(
                        name,
                        "a negative number to a fractional power is undefined",
                        span,
                    ));
                }
                Quantity::dimensionless(v)
            }
        };
        finite(q, name, span)
    }
}
