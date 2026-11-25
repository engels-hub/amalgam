//! Extents of extrude, revolve, hole and rib (S04 §4, note §3). SOL-01 computes one-sided
//! distance extents from the profile plane; the other variants are data for SOL-02/03.

use amg_doc::{Dim, EntityRef, Expr, ExprSrc};
use serde::{Deserialize, Serialize};

/// How far a side goes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Extent {
    /// A distance (mm, extrude) or an angle (rad, revolve).
    Distance(Expr),
    /// Up to a plane, face, body or vertex, with an offset; `extend` uses the target's
    /// untrimmed surface.
    ToObject {
        /// The target.
        target: EntityRef,
        /// Offset from the target (mm).
        offset: Expr,
        /// Extend the target face.
        extend: bool,
    },
    /// Through everything in the context component.
    All,
    /// Revolve only: 360°.
    Full,
}

/// One side of an extent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExtentSide {
    /// How far.
    pub extent: Extent,
    /// Taper angle (rad; revolve ignores it).
    pub taper: Expr,
}

impl ExtentSide {
    /// A distance side without taper.
    pub fn distance(d: Expr) -> ExtentSide {
        ExtentSide {
            extent: Extent::Distance(d),
            taper: Expr::Inline(ExprSrc("0 deg".into())),
        }
    }
}

/// Which sides of the profile an extent covers.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Direction {
    /// Along the profile normal (or against it with `flip`).
    #[default]
    OneSide,
    /// Side 1 along the normal, side 2 against it, each with its own extent.
    TwoSides,
    /// Both ways by half the distance (or the whole distance each way).
    Symmetric {
        /// Each side gets the whole distance.
        whole_length: bool,
    },
}

/// Where the extent starts.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum Start {
    /// At the profile plane.
    #[default]
    Profile,
    /// Offset along the normal (mm).
    Offset(Expr),
    /// At a planar object, with an offset.
    Object {
        /// The plane or planar face.
        target: EntityRef,
        /// Offset (mm).
        offset: Expr,
    },
}

/// Which side of an open chain a thin feature grows to.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ThinSide {
    /// Side 1.
    #[default]
    Side1,
    /// Side 2.
    Side2,
    /// Centred.
    Center,
}

/// Thin-feature options (SOL-18).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThinSpec {
    /// Wall thickness (mm).
    pub thickness: Expr,
    /// Side.
    pub side: ThinSide,
}

/// Every expression of an extent side with its dimension (`length` for distances: mm for
/// extrude, rad for revolve).
pub(crate) fn side_exprs(s: &mut ExtentSide, length: Dim) -> Vec<(&mut Expr, Dim)> {
    let mut v = Vec::new();
    match &mut s.extent {
        Extent::Distance(e) => v.push((e, length)),
        Extent::ToObject { offset, .. } => v.push((offset, Dim::LENGTH)),
        Extent::All | Extent::Full => {}
    }
    v.push((&mut s.taper, Dim::ANGLE));
    v
}

/// The entity references of a start and its sides.
pub(crate) fn extent_refs<'a>(
    start: &'a Start,
    sides: impl Iterator<Item = &'a ExtentSide> + 'a,
) -> impl Iterator<Item = EntityRef> + 'a {
    let s = match start {
        Start::Object { target, .. } => Some(target.clone()),
        Start::Profile | Start::Offset(_) => None,
    };
    s.into_iter().chain(sides.filter_map(|x| match &x.extent {
        Extent::ToObject { target, .. } => Some(target.clone()),
        _ => None,
    }))
}
