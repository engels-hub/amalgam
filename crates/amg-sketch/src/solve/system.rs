//! Dimension-agnostic equation-system layer: residual blocks with analytic Jacobians.
//!
//! Nothing here knows about sketch entities. A [`Residual`] is a vector function of some
//! entries of one global parameter vector `x` (indexed by [`ParamId::index`]). SK-03 builds
//! its solver on this layer; SK-12 (3D sketch) and S06 ASM-04 (joints) may reuse it.

use std::fmt;
use std::ops::Range;

use crate::ids::{ConstraintId, EntityId, ParamId};

/// One block of equations with a hand-written analytic Jacobian.
///
/// Contract:
/// - `x` is the **whole** parameter vector, indexed by [`ParamId::index`].
/// - [`params`](Residual::params) lists the parameters the block depends on, each once, in a
///   fixed order. It may include locked parameters; the solver drops their columns.
/// - [`eval`](Residual::eval) writes `rows()` values to `out[..rows()]`.
/// - [`jacobian`](Residual::jacobian) **overwrites** `out[..rows() * params().len()]` with the
///   row-major dense block `∂F_i/∂x_{params[j]}`.
/// - Lengths are mm. Angular rows are already multiplied by the system's characteristic
///   length, so every row has the unit mm and one tolerance applies.
pub trait Residual: Send + Sync {
    /// Number of equations in this block.
    fn rows(&self) -> usize;
    /// The parameters the block depends on (distinct).
    fn params(&self) -> &[ParamId];
    /// Residual values, `out.len() >= rows()`.
    fn eval(&self, x: &[f64], out: &mut [f64]);
    /// Row-major dense block, `rows × params().len()`.
    fn jacobian(&self, x: &[f64], out: &mut [f64]);
}

/// What an equation block comes from, for diagnosis and `unsatisfied` reporting.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EquationOwner {
    /// An internal equation of an entity (the arc's `|s − c| = |e − c|`).
    Entity(EntityId),
    /// A constraint or driving dimension.
    Constraint(ConstraintId),
}

struct Block {
    owner: EquationOwner,
    residual: Box<dyn Residual>,
    first_row: usize,
}

/// An ordered list of residual blocks. Row order is the push order, so a deterministic
/// builder gives a deterministic system.
#[derive(Default)]
pub struct EquationSystem {
    blocks: Vec<Block>,
    rows: usize,
}

impl fmt::Debug for EquationSystem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EquationSystem")
            .field("blocks", &self.blocks.len())
            .field("rows", &self.rows)
            .field(
                "owners",
                &self.blocks.iter().map(|b| b.owner).collect::<Vec<_>>(),
            )
            .finish()
    }
}

impl EquationSystem {
    /// Empty system.
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends a block; its rows follow all previous rows.
    pub fn push(&mut self, owner: EquationOwner, residual: Box<dyn Residual>) {
        let first_row = self.rows;
        self.rows += residual.rows();
        self.blocks.push(Block {
            owner,
            residual,
            first_row,
        });
    }

    /// Number of blocks.
    pub fn len(&self) -> usize {
        self.blocks.len()
    }

    /// True if there are no blocks.
    pub fn is_empty(&self) -> bool {
        self.blocks.is_empty()
    }

    /// Total number of equations (rows).
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// Block `i`.
    ///
    /// # Panics
    /// If `i >= len()`.
    pub fn residual(&self, i: usize) -> &dyn Residual {
        self.blocks[i].residual.as_ref()
    }

    /// Owner of block `i`.
    ///
    /// # Panics
    /// If `i >= len()`.
    pub fn owner(&self, i: usize) -> EquationOwner {
        self.blocks[i].owner
    }

    /// Rows of block `i` in the stacked system.
    ///
    /// # Panics
    /// If `i >= len()`.
    pub fn row_range(&self, i: usize) -> Range<usize> {
        let b = &self.blocks[i];
        b.first_row..b.first_row + b.residual.rows()
    }

    /// Owner of stacked row `row`; `None` if out of range.
    pub fn row_owner(&self, row: usize) -> Option<EquationOwner> {
        if row >= self.rows {
            return None;
        }
        let i = self.blocks.partition_point(|b| b.first_row <= row);
        // `i >= 1` because block 0 starts at row 0 and `row < rows`; skip empty blocks.
        self.blocks[..i]
            .iter()
            .rev()
            .find(|b| row < b.first_row + b.residual.rows())
            .map(|b| b.owner)
    }

    /// Blocks in row order: `(owner, residual, rows)`.
    pub fn iter(&self) -> impl Iterator<Item = (EquationOwner, &dyn Residual, Range<usize>)> + '_ {
        self.blocks.iter().map(|b| {
            (
                b.owner,
                b.residual.as_ref(),
                b.first_row..b.first_row + b.residual.rows(),
            )
        })
    }

    /// All residuals stacked: `out[..rows()]`.
    pub fn eval(&self, x: &[f64], out: &mut [f64]) {
        for b in &self.blocks {
            let n = b.residual.rows();
            b.residual.eval(x, &mut out[b.first_row..b.first_row + n]);
        }
    }

    /// Largest `|F_i(x)|` (0 for an empty system).
    pub fn max_abs_residual(&self, x: &[f64]) -> f64 {
        let mut f = vec![0.0; self.rows];
        self.eval(x, &mut f);
        f.iter().fold(0.0_f64, |m, v| m.max(v.abs()))
    }

    /// Dense stacked Jacobian, row-major `rows() × x.len()` (one column per parameter slot,
    /// in `ParamId` order). Overwrites `out[..rows() * x.len()]`. Meant for tests, rank checks
    /// and small blocks; the production solver assembles per block.
    pub fn jacobian_dense(&self, x: &[f64], out: &mut [f64]) {
        let n = x.len();
        out[..self.rows * n].fill(0.0);
        let mut block = Vec::new();
        for b in &self.blocks {
            let r = b.residual.rows();
            let params = b.residual.params();
            block.clear();
            block.resize(r * params.len(), 0.0);
            b.residual.jacobian(x, &mut block);
            for i in 0..r {
                let row = &mut out[(b.first_row + i) * n..(b.first_row + i + 1) * n];
                for (j, p) in params.iter().enumerate() {
                    row[p.index()] += block[i * params.len() + j];
                }
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact small-integer arithmetic")]
mod tests {
    use super::*;

    /// `F(x) = [x0 − x1, x1²]`.
    struct Toy([ParamId; 2]);
    impl Residual for Toy {
        fn rows(&self) -> usize {
            2
        }
        fn params(&self) -> &[ParamId] {
            &self.0
        }
        fn eval(&self, x: &[f64], out: &mut [f64]) {
            let (a, b) = (x[self.0[0].index()], x[self.0[1].index()]);
            out[0] = a - b;
            out[1] = b * b;
        }
        fn jacobian(&self, x: &[f64], out: &mut [f64]) {
            let b = x[self.0[1].index()];
            out[..4].copy_from_slice(&[1.0, -1.0, 0.0, 2.0 * b]);
        }
    }

    #[test]
    fn stacks_rows_and_owners() {
        let mut sys = EquationSystem::new();
        assert!(sys.is_empty());
        sys.push(
            EquationOwner::Entity(EntityId(7)),
            Box::new(Toy([ParamId(0), ParamId(2)])),
        );
        sys.push(
            EquationOwner::Constraint(ConstraintId(1)),
            Box::new(Toy([ParamId(2), ParamId(1)])),
        );
        assert_eq!(sys.len(), 2);
        assert_eq!(sys.rows(), 4);
        assert_eq!(sys.row_range(1), 2..4);
        assert_eq!(sys.row_owner(0), Some(EquationOwner::Entity(EntityId(7))));
        assert_eq!(
            sys.row_owner(3),
            Some(EquationOwner::Constraint(ConstraintId(1)))
        );
        assert_eq!(sys.row_owner(4), None);

        let x = [1.0, 2.0, 3.0];
        let mut f = [0.0; 4];
        sys.eval(&x, &mut f);
        assert_eq!(f, [-2.0, 9.0, 1.0, 4.0]);
        assert_eq!(sys.max_abs_residual(&x), 9.0);

        let mut j = [f64::NAN; 12];
        sys.jacobian_dense(&x, &mut j);
        #[rustfmt::skip]
        let expect = [
            1.0, 0.0, -1.0,
            0.0, 0.0, 6.0,
            0.0, -1.0, 1.0,
            0.0, 4.0, 0.0,
        ];
        assert_eq!(j, expect);
        assert_eq!(sys.iter().count(), 2);
        assert!(format!("{sys:?}").contains("rows: 4"));
    }
}
