//! The two builds of the shape backend. Both expose the same `Occ` surface to `shape.rs`.
//!
//! With `occt`, `Occ` owns the C++ shape. Without it, `Occ` is uninhabited: no `Shape` can exist,
//! so every query is statically unreachable and every constructor returns `BackendUnavailable`.

#[cfg(feature = "occt")]
mod occt;
#[cfg(feature = "occt")]
pub(crate) use occt::{Occ, init_process, occt_version, progress_probe};

#[cfg(not(feature = "occt"))]
mod none;
#[cfg(not(feature = "occt"))]
pub(crate) use none::{Occ, init_process, occt_version, progress_probe};
