//! Symmetric sparse matrices and sparse direct solvers (CMP-03; design note §5).
//!
//! - [`SymCsr`]: upper triangle in CSR, validated, deterministic [`SymCsr::spmv`];
//!   [`TripletBuilder`] for small or test assembly.
//! - [`SymbolicFactor`]: ordering + symbolic analysis of a pattern, reused by every numeric
//!   factorisation on it, with an exact [`FactorEstimate`].
//! - [`Cholesky`] (`L·Lᵀ`, with a [`PivotReport`]) and [`Ldlt`] (Bunch–Kaufman `L·B·Lᵀ`, with
//!   [`Inertia`]), both on faer's supernodal factorisations, with multi-RHS solves.

mod csr;
mod factor;

pub use csr::{SymCsr, TripletBuilder};
pub use factor::{
    Cholesky, FactorEstimate, FactorOpts, Inertia, Ldlt, Ordering, PivotReport, SymbolicFactor,
    analyses_on_this_thread,
};
