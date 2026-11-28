//! Corpora of the robustness framework (design note `kernel-robustness.md` §5, §6): deterministic
//! generators, the corpus case schema and the process-isolated harness.
//!
//! - The boolean near-coincidence generator (KH-17): [`near_coincident`], [`NearCase`].
//! - [`fillet`] (KH-02): the fillet generators `prism_vertical_edges` (exact volume oracle),
//!   `prism_mixed_edges`, `boolean_stack` and `radius_too_large`.
//! - [`case`] (KH-02): the `case.toml` schema, `seeds.toml` seed lists and corpus loading.
//! - [`harness`] (KH-02): runs cases in worker processes (an OCCT access violation is a
//!   `Crash` record, not the end of the run), the report JSON, baselines and the regression
//!   check.
//!
//! Synthetic cases are not stored as files: `seeds.toml` lists `(generator, seeds)` and the
//! generators rebuild them. Only handmade and promoted cases are case directories
//! (`case.toml` + `op.json` + `inputs/*.brep`).

pub mod case;
pub mod fillet;
pub mod harness;
mod near;

pub use near::{ANGLES, MAX_FUZZY, NearCase, NearFamily, OFFSETS, TILTS, near_coincident};

/// SplitMix64 (Steele, Lea, Flood 2014): a fixed, dependency-free generator, so the case lists
/// are the same on every machine and toolchain.
pub(crate) fn splitmix(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// A uniformly drawn element of `from` (not empty).
pub(crate) fn pick<T: Copy>(state: &mut u64, from: &[T]) -> T {
    #[allow(clippy::cast_possible_truncation, reason = "the modulus fits usize")]
    let i = (splitmix(state) % from.len() as u64) as usize;
    from[i]
}

/// A uniform draw from `[lo, hi)`.
pub(crate) fn uniform(state: &mut u64, lo: f64, hi: f64) -> f64 {
    // The top 53 bits as a fraction in [0, 1).
    #[allow(clippy::cast_precision_loss, reason = "53 bits are exact in f64")]
    let u = (splitmix(state) >> 11) as f64 / (1u64 << 53) as f64;
    lo + (hi - lo) * u
}

/// A uniform integer draw from `lo..=hi`.
pub(crate) fn int(state: &mut u64, lo: usize, hi: usize) -> usize {
    #[allow(clippy::cast_possible_truncation, reason = "the modulus fits usize")]
    let i = (splitmix(state) % (hi - lo + 1) as u64) as usize;
    lo + i
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn draws_stay_in_range_and_repeat() {
        let mut a = 7;
        let mut b = 7;
        for _ in 0..1000 {
            let u = uniform(&mut a, 2.0, 3.0);
            assert!((2.0..3.0).contains(&u));
            assert_eq!(u.to_bits(), uniform(&mut b, 2.0, 3.0).to_bits());
            let i = int(&mut a, 3, 12);
            assert!((3..=12).contains(&i));
            assert_eq!(i, int(&mut b, 3, 12));
        }
    }
}
