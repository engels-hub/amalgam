//! The boolean near-coincidence generator (KH-17, design note §7): seed lists, not files, each
//! case with an analytic volume oracle.

use std::f64::consts::PI;

use amg_core::{Dir3, Plane, Point3, Vec3};

use super::pick;
use crate::error::KResult;
use crate::kernel::BrepKernel;
use crate::opts::BoolOp;
use crate::shape::Shape;

/// The face offsets drawn by [`near_coincident`], mm (design note §7).
pub const OFFSETS: [f64; 9] = [0.0, 1e-7, -1e-7, 1e-6, -1e-6, 1e-5, -1e-5, 1e-4, -1e-4];

/// The tiny rotations drawn for box cases, rad.
pub const ANGLES: [f64; 5] = [0.0, 1e-7, -1e-7, 1e-6, -1e-6];

/// The tilts drawn for the tilted families, rad.
pub const TILTS: [f64; 8] = [1e-6, -1e-6, 1e-5, -1e-5, 1e-4, -1e-4, 1e-3, -1e-3];

/// The largest fuzzy value a robust result may have used, mm: the volume oracle's slack covers
/// the material a fuzzy rung of this size can add or remove.
pub const MAX_FUZZY: f64 = 1e-3;

/// A near-coincident configuration. Box A is `[0, 10]³` throughout.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum NearFamily {
    /// Box B `[10 + d, 20 + d] × [2, 8] × [2, 8]` against A's `x = 10` face (overlap `−d`).
    BoxFaceOffset,
    /// A pocket `[3, 7]² × [5, 10 + d]` cut from A: its floor-to-top face nearly flush with A's
    /// top (a skin of `−d` remains for `d < 0`).
    FlushPocket,
    /// A boss `[3, 7]² × [10 + d, 15 + d]` fused onto A's top (a gap or overlap of `|d|`).
    StackedBoss,
    /// Two cylinders `r = 5`, `h = 10`, axes `10 + d` apart: line contact.
    TangentCylinders,
    /// Two coaxial cylinders `r = 5` and `r = 5 + d`, `h = 10`, the second from `z = 5`:
    /// coincident cylindrical faces.
    CoaxialCylinders,
    /// A second copy of A moved by `(d, d, 0)`: coplanar faces everywhere.
    ShiftedCopy,
    /// [`NearFamily::BoxFaceOffset`] with B tilted by `angle` about the y axis through the
    /// centre of its contact face: a wedge-shaped near-contact.
    TiltedFace,
    /// A through hole `r = 3` whose axis is `3 + d` from A's `x = 0` wall: the hole touches the
    /// wall (breaks through it in a sliver for `d < 0`).
    TangentHole,
    /// A sphere `r = 5` centred `5 + d` above A's top centre: point contact.
    SphereOnBox,
    /// [`NearFamily::CoaxialCylinders`] with the second axis tilted by `angle` about x through
    /// `(0, 0, 5)`: nearly coincident cylindrical faces that cross.
    SkewCoaxial,
}

impl NearFamily {
    /// Every family, in generation order.
    pub const ALL: [NearFamily; 10] = [
        NearFamily::BoxFaceOffset,
        NearFamily::FlushPocket,
        NearFamily::StackedBoss,
        NearFamily::TangentCylinders,
        NearFamily::CoaxialCylinders,
        NearFamily::ShiftedCopy,
        NearFamily::TiltedFace,
        NearFamily::TangentHole,
        NearFamily::SphereOnBox,
        NearFamily::SkewCoaxial,
    ];
}

/// One generated case: `A op B` with face offset `offset` and B rotated by `angle`: about the
/// z axis through its centre for the box families, as the tilt of the tilted families.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NearCase {
    /// `near-<index>-<family>`.
    pub id: String,
    /// The configuration.
    pub family: NearFamily,
    /// The op.
    pub op: BoolOp,
    /// The face offset `d`, mm.
    pub offset: f64,
    /// B's rotation about z, rad.
    pub angle: f64,
}

/// `n` cases from `seed`: family `i mod 10` (so every family is covered evenly), offset from
/// [`OFFSETS`], angle from [`ANGLES`] (box families) or [`TILTS`] (tilted families) and an op
/// suited to the family.
pub fn near_coincident(n: usize, seed: u64) -> Vec<NearCase> {
    let mut state = seed;
    (0..n)
        .map(|i| {
            let family = NearFamily::ALL[i % NearFamily::ALL.len()];
            let offset = pick(&mut state, &OFFSETS);
            let angle = match family {
                NearFamily::BoxFaceOffset | NearFamily::FlushPocket | NearFamily::ShiftedCopy => {
                    pick(&mut state, &ANGLES)
                }
                NearFamily::TiltedFace | NearFamily::SkewCoaxial => pick(&mut state, &TILTS),
                _ => 0.0,
            };
            let op = match family {
                NearFamily::BoxFaceOffset | NearFamily::TiltedFace => {
                    pick(&mut state, &[BoolOp::Fuse, BoolOp::Cut, BoolOp::Common])
                }
                NearFamily::FlushPocket | NearFamily::TangentHole => BoolOp::Cut,
                NearFamily::StackedBoss
                | NearFamily::CoaxialCylinders
                | NearFamily::SphereOnBox
                | NearFamily::SkewCoaxial => BoolOp::Fuse,
                NearFamily::TangentCylinders | NearFamily::ShiftedCopy => {
                    pick(&mut state, &[BoolOp::Fuse, BoolOp::Cut])
                }
            };
            NearCase {
                id: format!("near-{i:03}-{family:?}"),
                family,
                op,
                offset,
                angle,
            }
        })
        .collect()
}

/// A frame at `origin` rotated by `angle` about z through `centre`.
fn frame(origin: Point3, centre: Point3, angle: f64) -> Plane {
    frame_about(origin, centre, Vec3::Z, angle)
}

/// The world frame moved to `origin`, then rotated by `angle` about the unit `axis` through
/// `centre` (Rodrigues).
fn frame_about(origin: Point3, centre: Point3, axis: Vec3, angle: f64) -> Plane {
    let (s, c) = angle.sin_cos();
    let rot = |v: Vec3| v * c + axis.cross(v) * s + axis * axis.dot(v) * (1.0 - c);
    let x = rot(Vec3::X).normalize();
    let n = rot(Vec3::Z).normalize();
    Plane {
        origin: centre + rot(origin - centre),
        x: Dir3::new_unchecked(x),
        y: Dir3::new_unchecked(n.cross(x)),
        normal: Dir3::new_unchecked(n),
    }
}

/// The area of the circular segment of depth `depth` cut off a circle of radius `r`.
fn segment_area(r: f64, depth: f64) -> f64 {
    if depth <= 0.0 {
        return 0.0;
    }
    let h = r - depth;
    r * r * (h / r).acos() - h * (2.0 * r * depth - depth * depth).sqrt()
}

/// The area of the lens two circles of radius `r` overlap in when their centres are
/// `2r − depth` apart.
fn lens_area(r: f64, depth: f64) -> f64 {
    if depth <= 0.0 {
        return 0.0;
    }
    let d = 2.0 * r - depth;
    2.0 * r * r * (d / (2.0 * r)).acos() - 0.5 * d * (4.0 * r * r - d * d).sqrt()
}

impl NearCase {
    /// The operands `([A], [B])`.
    ///
    /// # Errors
    /// The kernel's error when a primitive cannot be built.
    pub fn operands(&self, k: &dyn BrepKernel) -> KResult<(Vec<Shape>, Vec<Shape>)> {
        let d = self.offset;
        let z = Point3::ZERO;
        let a_box = || {
            k.make_box(&frame(z, z, 0.0), Vec3::splat(10.0))
                .map(|r| r.shape)
        };
        let cyl = |x: f64, z0: f64, r: f64| {
            k.make_cylinder(
                &frame(Point3::new(x, 0.0, z0), Point3::ZERO, 0.0),
                r,
                10.0,
                2.0 * PI,
            )
            .map(|r| r.shape)
        };
        let boxed = |o: Point3, size: Vec3| {
            let centre = o + size * 0.5;
            k.make_box(&frame(o, centre, self.angle), size)
                .map(|r| r.shape)
        };
        let (a, b) = match self.family {
            NearFamily::BoxFaceOffset => (
                a_box()?,
                boxed(Point3::new(10.0 + d, 2.0, 2.0), Vec3::new(10.0, 6.0, 6.0))?,
            ),
            NearFamily::FlushPocket => (
                a_box()?,
                boxed(Point3::new(3.0, 3.0, 5.0), Vec3::new(4.0, 4.0, 5.0 + d))?,
            ),
            NearFamily::StackedBoss => (
                a_box()?,
                boxed(Point3::new(3.0, 3.0, 10.0 + d), Vec3::new(4.0, 4.0, 5.0))?,
            ),
            NearFamily::TangentCylinders => (cyl(0.0, 0.0, 5.0)?, cyl(10.0 + d, 0.0, 5.0)?),
            NearFamily::CoaxialCylinders => (cyl(0.0, 0.0, 5.0)?, cyl(0.0, 5.0, 5.0 + d)?),
            NearFamily::ShiftedCopy => {
                (a_box()?, boxed(Point3::new(d, d, 0.0), Vec3::splat(10.0))?)
            }
            NearFamily::TiltedFace => {
                let o = Point3::new(10.0 + d, 2.0, 2.0);
                let f = frame_about(o, Point3::new(10.0 + d, 5.0, 5.0), Vec3::Y, self.angle);
                (a_box()?, k.make_box(&f, Vec3::new(10.0, 6.0, 6.0))?.shape)
            }
            NearFamily::TangentHole => {
                let f = frame(Point3::new(3.0 + d, 5.0, 0.0), z, 0.0);
                (a_box()?, k.make_cylinder(&f, 3.0, 10.0, 2.0 * PI)?.shape)
            }
            NearFamily::SphereOnBox => {
                let f = frame(Point3::new(5.0, 5.0, 15.0 + d), z, 0.0);
                (a_box()?, k.make_sphere(&f, 5.0)?.shape)
            }
            NearFamily::SkewCoaxial => {
                let o = Point3::new(0.0, 0.0, 5.0);
                let f = frame_about(o, o, Vec3::X, self.angle);
                (
                    cyl(0.0, 0.0, 5.0)?,
                    k.make_cylinder(&f, 5.0 + d, 10.0, 2.0 * PI)?.shape,
                )
            }
        };
        Ok((vec![a], vec![b]))
    }

    /// The analytic result volume (mm³) and the slack a valid result stays within: the
    /// offset's and rotation's effect plus what a fuzzy value up to [`MAX_FUZZY`] can change on
    /// the near-coincident faces, plus 1e-6 relative.
    pub fn expected_volume(&self) -> (f64, f64) {
        let d = self.offset;
        let ad = d.abs();
        let th = self.angle.abs();
        let fz = 2.0 * MAX_FUZZY;
        let (v, slack) = match self.family {
            NearFamily::BoxFaceOffset => {
                let overlap = 36.0 * (-d).max(0.0);
                let v = match self.op {
                    BoolOp::Fuse => 1000.0 + 360.0 - overlap,
                    BoolOp::Cut => 1000.0 - overlap,
                    BoolOp::Common => overlap,
                };
                // The contact face (36 mm²) moves by |d|; the rotation tilts it by 3θ mm.
                (v, 36.0 * (2.0 * ad + 6.0 * th + fz))
            }
            NearFamily::FlushPocket => (
                1000.0 - 16.0 * (5.0 + d.min(0.0)),
                16.0 * (2.0 * ad + fz) + 40.0 * th,
            ),
            NearFamily::StackedBoss => (1080.0 - 16.0 * (-d).max(0.0), 16.0 * (2.0 * ad + fz)),
            NearFamily::TangentCylinders => {
                let cyl = PI * 25.0 * 10.0;
                let lens = 10.0 * lens_area(5.0, -d);
                let v = match self.op {
                    BoolOp::Cut => cyl - lens,
                    _ => 2.0 * cyl - lens,
                };
                // A fuzzy value f merges a strip about 2√(r f) wide along the contact line.
                (v, 10.0 * 2.0 * (5.0 * fz).sqrt() * fz + 1e-6)
            }
            NearFamily::CoaxialCylinders => {
                let r2 = 5.0 + d;
                let v = PI * (25.0 * 10.0 + r2 * r2 * 10.0 - 25.0_f64.min(r2 * r2) * 5.0);
                (v, 2.0 * PI * 5.0 * 15.0 * (2.0 * ad + fz))
            }
            NearFamily::ShiftedCopy => {
                let side = 10.0 + ad;
                let union = side * side * 10.0 - 2.0 * ad * ad * 10.0;
                let v = match self.op {
                    BoolOp::Cut => 1000.0 - (10.0 - ad) * (10.0 - ad) * 10.0,
                    _ => union,
                };
                (v, 400.0 * (2.0 * ad + fz) + 600.0 * th)
            }
            NearFamily::TiltedFace => {
                let overlap = 36.0 * (-d).max(0.0);
                let v = match self.op {
                    BoolOp::Fuse => 1000.0 + 360.0 - overlap,
                    BoolOp::Cut => 1000.0 - overlap,
                    BoolOp::Common => overlap,
                };
                // The tilt moves the contact face's edges by up to 3θ (a wedge of 27θ mm³ per
                // side); B's own volume does not change.
                (v, 36.0 * (2.0 * ad + 3.0 * th + fz))
            }
            NearFamily::TangentHole => {
                let hole = PI * 9.0 - segment_area(3.0, -d);
                (
                    1000.0 - 10.0 * hole,
                    10.0 * (2.0 * (3.0 * fz).sqrt() * fz + 12.0 * ad) + 1e-6,
                )
            }
            NearFamily::SphereOnBox => {
                let depth = (-d).max(0.0);
                let cap = PI * depth * depth * (15.0 - depth) / 3.0;
                (
                    1000.0 + 4.0 / 3.0 * PI * 125.0 - cap,
                    PI * 10.0 * fz * fz + 1e-6,
                )
            }
            NearFamily::SkewCoaxial => {
                let r2 = 5.0 + d;
                let v = PI * (25.0 * 10.0 + r2 * r2 * 10.0 - 25.0_f64.min(r2 * r2) * 5.0);
                // The tilted axis moves the second cylinder's far end by 10θ sideways.
                (
                    v,
                    2.0 * PI * 5.0 * 15.0 * (2.0 * ad + fz) + 2.0 * PI * 5.0 * 100.0 * th,
                )
            }
        };
        (v, slack + 1e-6 * v.abs().max(1.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_case_list_is_fixed() {
        let a = near_coincident(100, 17);
        assert_eq!(a, near_coincident(100, 17));
        assert_ne!(a, near_coincident(100, 18));
        assert_eq!(a.len(), 100);
        for f in NearFamily::ALL {
            assert_eq!(a.iter().filter(|c| c.family == f).count(), 10, "{f:?}");
        }
        assert!(a.iter().any(|c| c.offset == 0.0));
        assert!(a.iter().any(|c| c.offset < 0.0) && a.iter().any(|c| c.offset > 0.0));
        let json = serde_json::to_string(&a[0]).unwrap();
        assert_eq!(serde_json::from_str::<NearCase>(&json).unwrap(), a[0]);
    }

    #[test]
    fn lens_area_matches_small_depth_asymptote() {
        // A ≈ (4/3)·√r·δ^{3/2} for small δ (two circles of radius r).
        let (r, depth) = (5.0, 1e-4);
        let exact = lens_area(r, depth);
        let approx = 4.0 / 3.0 * r.sqrt() * depth.powf(1.5);
        assert!((exact / approx - 1.0).abs() < 1e-3, "{exact} vs {approx}");
        assert!(lens_area(r, 0.0).abs() < f64::MIN_POSITIVE);
        // Segment: small depth ≈ (4/3)·√(2r)·δ^{3/2}.
        let seg = segment_area(r, depth);
        let approx = 4.0 / 3.0 * (2.0 * r).sqrt() * depth.powf(1.5);
        assert!((seg / approx - 1.0).abs() < 1e-3, "{seg} vs {approx}");
        assert!((segment_area(r, r) - PI * r * r / 2.0).abs() < 1e-12);
    }

    #[test]
    #[allow(clippy::float_cmp, reason = "the oracles are exact at these offsets")]
    fn oracles_are_the_nominal_volumes_at_zero_offset() {
        let case = |family, op| NearCase {
            id: String::new(),
            family,
            op,
            offset: 0.0,
            angle: 0.0,
        };
        let v = |c: NearCase| c.expected_volume().0;
        assert_eq!(v(case(NearFamily::BoxFaceOffset, BoolOp::Fuse)), 1360.0);
        assert_eq!(v(case(NearFamily::BoxFaceOffset, BoolOp::Common)), 0.0);
        assert_eq!(v(case(NearFamily::FlushPocket, BoolOp::Cut)), 920.0);
        assert_eq!(v(case(NearFamily::StackedBoss, BoolOp::Fuse)), 1080.0);
        assert!((v(case(NearFamily::CoaxialCylinders, BoolOp::Fuse)) - PI * 375.0).abs() < 1e-9);
        assert_eq!(v(case(NearFamily::ShiftedCopy, BoolOp::Fuse)), 1000.0);
        assert_eq!(v(case(NearFamily::ShiftedCopy, BoolOp::Cut)), 0.0);
    }
}
