//! Chirality capture: choosing `Side`, `TangentKind` and `AngleBranch` from the current
//! geometry so a new constraint's residual starts at (or nearest to) zero.

use super::residual::{build, characteristic_length};
use super::system::Residual;
use crate::error::SketchError;
use crate::ids::ConstraintId;
use crate::model::{AngleBranch, ConstraintKind, DimensionKind, Side, Sketch, TangentKind};

fn flip(side: Side) -> Side {
    match side {
        Side::Positive => Side::Negative,
        Side::Negative => Side::Positive,
    }
}

/// Every orientation variant of `kind`, `kind` itself first.
fn candidates(kind: &ConstraintKind) -> Vec<ConstraintKind> {
    let mut out = vec![kind.clone()];
    match kind {
        ConstraintKind::Tangent { a, b, kind: tk } => {
            let other = match *tk {
                TangentKind::External => TangentKind::Internal,
                TangentKind::Internal => TangentKind::External,
                TangentKind::AtEndpoint {
                    a_end,
                    b_end,
                    reversed,
                } => TangentKind::AtEndpoint {
                    a_end,
                    b_end,
                    reversed: !reversed,
                },
            };
            out.push(ConstraintKind::Tangent {
                a: *a,
                b: *b,
                kind: other,
            });
        }
        ConstraintKind::Dimension(d) => {
            let mut with = |k: DimensionKind| {
                let mut nd = d.clone();
                nd.kind = k;
                out.push(ConstraintKind::Dimension(nd));
            };
            match d.kind {
                DimensionKind::Distance { a, b, side } => {
                    with(DimensionKind::Distance {
                        a,
                        b,
                        side: flip(side),
                    });
                }
                DimensionKind::HorizontalDistance { a, b, side } => {
                    with(DimensionKind::HorizontalDistance {
                        a,
                        b,
                        side: flip(side),
                    });
                }
                DimensionKind::VerticalDistance { a, b, side } => {
                    with(DimensionKind::VerticalDistance {
                        a,
                        b,
                        side: flip(side),
                    });
                }
                DimensionKind::Angle { a, b, .. } => {
                    for (a, b) in [(a, b), (b, a)] {
                        for reverse_a in [false, true] {
                            for reverse_b in [false, true] {
                                with(DimensionKind::Angle {
                                    a,
                                    b,
                                    branch: AngleBranch {
                                        reverse_a,
                                        reverse_b,
                                    },
                                });
                            }
                        }
                    }
                }
                _ => {}
            }
        }
        _ => {}
    }
    out
}

/// Returns `kind` with its orientation fields chosen from the current geometry: the variant
/// whose residual has the smallest max-abs value (the given variant wins ties).
///
/// - `Side` of distance dimensions (point–point distance has no side; it stays as given).
/// - `TangentKind::External` vs `Internal`; for `AtEndpoint`, `reversed` (the named ends are
///   kept).
/// - `AngleBranch` of an angle dimension. The four branches give only `θ` and `θ + π`, so to
///   match an unsigned value the lines `a` and `b` may also be **swapped** (`−θ`, `π − θ`).
///
/// Kinds without orientation, and kinds whose entities are missing or of the wrong kind, are
/// returned unchanged (`Sketch::add_constraint` then reports the error).
pub fn orient_constraint(sketch: &Sketch, kind: ConstraintKind) -> ConstraintKind {
    let cands = candidates(&kind);
    if cands.len() == 1 {
        return kind;
    }
    let lc = characteristic_length(sketch);
    let mut best: Option<(f64, ConstraintKind)> = None;
    for c in cands {
        let Ok(Some(r)) = build(sketch, &c, &[], lc, true) else {
            continue;
        };
        let mut f = vec![0.0; r.rows()];
        r.eval(sketch.params().as_slice(), &mut f);
        let m = f.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        if best.as_ref().is_none_or(|(bm, _)| m < *bm) {
            best = Some((m, c));
        }
    }
    best.map_or(kind, |(_, c)| c)
}

impl Sketch {
    /// Like [`add_constraint`](Sketch::add_constraint), but first recomputes
    /// `Side` / `TangentKind` / `AngleBranch` in `kind` from the current geometry
    /// ([`orient_constraint`](crate::solve::orient_constraint)), so the new residual is zero
    /// (or as small as the value allows) with the right sign.
    pub fn add_constraint_oriented(
        &mut self,
        kind: ConstraintKind,
    ) -> Result<ConstraintId, SketchError> {
        let kind = orient_constraint(self, kind);
        self.add_constraint(kind)
    }
}
