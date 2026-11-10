//! Operation kinds and the element types the reductions accept.

use crate::precision::Precision;
use crate::sealed;

/// A reduction operator.
///
/// `Sum` wraps on `u32` and `i32`. `Min` and `Max` use the comparison `b < a` (resp. `a < b`)
/// on both the GPU and the CPU reference, so a NaN operand never replaces a number; the result
/// of reducing NaNs is otherwise unspecified. The identity of `Min` and `Max` on floats is the
/// largest (resp. smallest) *finite* value, because WGSL cannot spell an infinity: reducing an
/// empty input or an empty segment gives `f32::MAX` (resp. `f32::MIN`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ReduceOp {
    /// Sum.
    Sum,
    /// Minimum.
    Min,
    /// Maximum.
    Max,
}

/// Whether a scan includes the element itself.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ScanKind {
    /// `out[i] = sum(in[..i])`.
    Exclusive,
    /// `out[i] = sum(in[..=i])`.
    Inclusive,
}

/// Element types the reductions accept: f32, f64 (needs `SHADER_F64`), u32, i32.
///
/// Sealed. The associated items are the contract between the GPU kernels and the CPU
/// references in [`cpu`](super::cpu); they are not meant to be called directly.
pub trait ReduceScalar: bytemuck::Pod + Send + Sync + sealed::Sealed + 'static {
    /// The precision the kernels for this type are compiled for.
    #[doc(hidden)]
    const KERNEL_PRECISION: Precision;
    /// The WGSL element type: `real` for floats (bound by the prelude), else `u32` / `i32`.
    #[doc(hidden)]
    const WGSL_TYPE: &'static str;
    /// The identity of `op` as a WGSL expression of type `Self`.
    #[doc(hidden)]
    fn wgsl_identity(op: ReduceOp) -> &'static str;
    /// The identity of `op`: what an empty input reduces to.
    fn identity(op: ReduceOp) -> Self;
    /// `a op b`, with the exact semantics of the GPU kernels.
    fn combine(op: ReduceOp, a: Self, b: Self) -> Self;
}

macro_rules! impl_reduce_scalar {
    ($t:ty, $prec:expr, $wgsl:expr, $add:expr, [$sum:expr, $min:expr, $max:expr],
     [$wsum:expr, $wmin:expr, $wmax:expr], $imin:expr, $imax:expr) => {
        impl ReduceScalar for $t {
            const KERNEL_PRECISION: Precision = $prec;
            const WGSL_TYPE: &'static str = $wgsl;
            fn wgsl_identity(op: ReduceOp) -> &'static str {
                match op {
                    ReduceOp::Sum => $wsum,
                    ReduceOp::Min => $wmin,
                    ReduceOp::Max => $wmax,
                }
            }
            fn identity(op: ReduceOp) -> $t {
                match op {
                    ReduceOp::Sum => $sum,
                    ReduceOp::Min => $imin,
                    ReduceOp::Max => $imax,
                }
            }
            #[inline]
            fn combine(op: ReduceOp, a: $t, b: $t) -> $t {
                match op {
                    ReduceOp::Sum => $add(a, b),
                    ReduceOp::Min => {
                        if b < a {
                            b
                        } else {
                            a
                        }
                    }
                    ReduceOp::Max => {
                        if a < b {
                            b
                        } else {
                            a
                        }
                    }
                }
            }
        }
    };
}

impl_reduce_scalar!(
    f32,
    Precision::F32,
    "real",
    |a: f32, b: f32| a + b,
    [0.0f32, 0.0, 0.0],
    [
        "real(0)",
        "real(3.4028234663852886e38)",
        "real(-3.4028234663852886e38)"
    ],
    f32::MAX,
    f32::MIN
);
impl_reduce_scalar!(
    f64,
    Precision::F64,
    "real",
    |a: f64, b: f64| a + b,
    [0.0f64, 0.0, 0.0],
    [
        "real(0)",
        "real(1.7976931348623157e308)",
        "real(-1.7976931348623157e308)"
    ],
    f64::MAX,
    f64::MIN
);
impl_reduce_scalar!(
    u32,
    Precision::F32,
    "u32",
    u32::wrapping_add,
    [0u32, 0, 0],
    ["0u", "4294967295u", "0u"],
    u32::MAX,
    0u32
);
impl_reduce_scalar!(
    i32,
    Precision::F32,
    "i32",
    i32::wrapping_add,
    [0i32, 0, 0],
    ["0i", "i32(2147483647)", "i32(-2147483648)"],
    i32::MAX,
    i32::MIN
);
