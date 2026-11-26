//! Display sizing of construction geometry (CON-04, note `inspection-geometry.md` §6).
//!
//! Pure functions: `amg-ui::construct` draws the result (RND-07) and picks it on the CPU.
//!
//! With `B` the AABB of the visible bodies (in the same space as the geometry):
//! - a **plane** is a square in its own frame, centred at the projection of `B`'s centre onto
//!   the plane, with side `max(20 mm, 1.2 · the largest extent of B's 8 corners projected into
//!   the plane)` (the extent along the plane's x or y axis);
//! - an **axis** is a segment centred at the projection of `B`'s centre onto the line, with
//!   length `max(20 mm, 1.2 · B.diagonal())`;
//! - a **point** is a marker at the point.
//!
//! Without visible bodies (`B` empty) a plane is centred at its own origin and an axis at its
//! own point, both with the 20 mm minimum.

use amg_core::{Aabb3, Dir3, Line3, Plane, Point3, Segment3, Tolerance, Transform};
use amg_doc::ConstructionGeom;

/// Smallest side of a plane square and length of an axis segment, mm.
pub const MIN_SIZE: f64 = 20.0;
/// How much larger than the bodies a plane or axis is drawn.
pub const MARGIN: f64 = 1.2;

/// What to draw for one construction plane, axis or point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConstructionDisplay {
    /// A square: `frame.origin` is its centre, its sides run along `frame.x` and `frame.y`.
    PlaneSquare {
        /// The plane's frame moved to the square's centre.
        frame: Plane,
        /// Half the side, mm.
        half_size: f64,
    },
    /// A segment of the axis, directed like the axis.
    AxisSegment(Segment3),
    /// A point marker.
    PointMarker(Point3),
}

impl ConstructionDisplay {
    /// The centre of what is drawn.
    pub fn center(&self) -> Point3 {
        match self {
            ConstructionDisplay::PlaneSquare { frame, .. } => frame.origin,
            ConstructionDisplay::AxisSegment(s) => (s.a + s.b) * 0.5,
            ConstructionDisplay::PointMarker(p) => *p,
        }
    }

    /// The four corners of a plane square, counter-clockwise about the normal.
    pub fn corners(&self) -> Option<[Point3; 4]> {
        match self {
            ConstructionDisplay::PlaneSquare { frame, half_size } => Some(
                [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].map(|(u, v)| {
                    frame.origin + (frame.x.get() * u + frame.y.get() * v) * *half_size
                }),
            ),
            _ => None,
        }
    }
}

/// `geom` moved from its component's space by the rigid occurrence transform `t` (the identity
/// returns it bit for bit).
pub fn placed(geom: &ConstructionGeom, t: &Transform) -> ConstructionGeom {
    if *t == Transform::IDENTITY {
        return *geom;
    }
    match geom {
        ConstructionGeom::Plane(p) => ConstructionGeom::Plane(super::math::transform_plane(t, p)),
        ConstructionGeom::Axis(l) => ConstructionGeom::Axis(Line3 {
            point: t.transform_point3(l.point),
            dir: Dir3::new(t.transform_vector3(l.dir.get()), &Tolerance::DEFAULT).unwrap_or(l.dir),
        }),
        ConstructionGeom::Point(p) => ConstructionGeom::Point(t.transform_point3(*p)),
    }
}

fn corners_of(b: &Aabb3) -> [Point3; 8] {
    std::array::from_fn(|i| {
        let pick = |bit: usize, lo: f64, hi: f64| if i & bit == 0 { lo } else { hi };
        Point3::new(
            pick(1, b.min.x, b.max.x),
            pick(2, b.min.y, b.max.y),
            pick(4, b.min.z, b.max.z),
        )
    })
}

/// The size and place of `geom` for the visible bodies' bounds (see the module docs).
pub fn display_extent(geom: &ConstructionGeom, visible_bounds: &Aabb3) -> ConstructionDisplay {
    let bounds = (!visible_bounds.is_empty()).then_some(visible_bounds);
    match geom {
        ConstructionGeom::Plane(p) => {
            let center = bounds.map_or(p.origin, |b| p.project(b.center()));
            let extent = bounds.map_or(0.0, |b| {
                let mut lo = [f64::INFINITY; 2];
                let mut hi = [f64::NEG_INFINITY; 2];
                for c in corners_of(b) {
                    let uv = p.to_local(c);
                    for (i, v) in [uv.x, uv.y].into_iter().enumerate() {
                        lo[i] = lo[i].min(v);
                        hi[i] = hi[i].max(v);
                    }
                }
                (hi[0] - lo[0]).max(hi[1] - lo[1])
            });
            let mut frame = *p;
            frame.origin = center;
            ConstructionDisplay::PlaneSquare {
                frame,
                half_size: (MARGIN * extent).max(MIN_SIZE) * 0.5,
            }
        }
        ConstructionGeom::Axis(l) => {
            let dir = l.dir.get();
            let center =
                bounds.map_or(l.point, |b| l.point + dir * (b.center() - l.point).dot(dir));
            let length = (MARGIN * bounds.map_or(0.0, Aabb3::diagonal)).max(MIN_SIZE);
            let half = dir * (length * 0.5);
            ConstructionDisplay::AxisSegment(Segment3 {
                a: center - half,
                b: center + half,
            })
        }
        ConstructionGeom::Point(p) => ConstructionDisplay::PointMarker(*p),
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, reason = "exact sizes")]
    use super::*;
    use amg_core::{Dir3, Line3, Tolerance, Vec3};

    fn aabb(min: [f64; 3], max: [f64; 3]) -> Aabb3 {
        Aabb3 {
            min: Point3::from_array(min),
            max: Point3::from_array(max),
        }
    }

    fn plane(g: &ConstructionGeom, b: &Aabb3) -> (Plane, f64) {
        match display_extent(g, b) {
            ConstructionDisplay::PlaneSquare { frame, half_size } => (frame, half_size),
            other => panic!("not a plane: {other:?}"),
        }
    }

    fn axis(g: &ConstructionGeom, b: &Aabb3) -> Segment3 {
        match display_extent(g, b) {
            ConstructionDisplay::AxisSegment(s) => s,
            other => panic!("not an axis: {other:?}"),
        }
    }

    fn close(a: Point3, b: Point3) {
        assert!((a - b).length() < 1e-9, "{a:?} != {b:?}");
    }

    #[test]
    fn a_plane_is_centred_on_the_projected_centre_and_1_2_times_the_projected_extent() {
        // 100 x 40 x 10 box; XY plane at z = 50.
        let b = aabb([10.0, 20.0, 0.0], [110.0, 60.0, 10.0]);
        let mut p = Plane::XY;
        p.origin = Point3::new(-5.0, 7.0, 50.0);
        let (frame, half) = plane(&ConstructionGeom::Plane(p), &b);
        close(frame.origin, Point3::new(60.0, 40.0, 50.0));
        assert_eq!((frame.x, frame.y, frame.normal), (p.x, p.y, p.normal));
        // Largest projected extent is x: 100 -> side 120.
        assert!((half - 60.0).abs() < 1e-12);
    }

    #[test]
    fn the_extent_is_measured_in_the_plane_frame() {
        let b = aabb([0.0; 3], [30.0, 40.0, 50.0]);
        // A YZ plane sees 40 (y) x 50 (z) -> side 60.
        let (frame, half) = plane(&ConstructionGeom::Plane(Plane::YZ), &b);
        close(frame.origin, Point3::new(0.0, 20.0, 25.0));
        assert!((half - 30.0).abs() < 1e-12);
        // A tilted plane (normal in XY at 45 degrees): the extent along the in-plane horizontal
        // is (30 + 40) / sqrt 2 = 49.497, along z 50 -> z wins.
        let n = Dir3::new(Vec3::new(1.0, 1.0, 0.0), &Tolerance::DEFAULT).unwrap();
        let tilted =
            Plane::from_normal_x(Point3::ZERO, n, Vec3::Z.cross(n.get()), &Tolerance::DEFAULT)
                .unwrap();
        let (frame, half) = plane(&ConstructionGeom::Plane(tilted), &b);
        assert!((half - 30.0).abs() < 1e-9, "{half}");
        assert!(tilted.signed_distance(frame.origin).abs() < 1e-9);
        close(frame.origin, tilted.project(b.center()));
        // Every projected corner lies inside the square.
        for c in corners_of(&b) {
            let uv = frame.to_local(c);
            assert!(uv.x.abs() <= half && uv.y.abs() <= half, "{uv:?}");
        }
    }

    #[test]
    fn small_bodies_give_the_20_mm_minimum() {
        let b = aabb([1.0; 3], [2.0; 3]);
        let (frame, half) = plane(&ConstructionGeom::Plane(Plane::XY), &b);
        close(frame.origin, Point3::new(1.5, 1.5, 0.0));
        assert_eq!(half, 10.0);
        let s = axis(
            &ConstructionGeom::Axis(Line3 {
                point: Point3::ZERO,
                dir: Dir3::Z,
            }),
            &b,
        );
        close(s.a, Point3::new(0.0, 0.0, -8.5));
        close(s.b, Point3::new(0.0, 0.0, 11.5));
        // 1.2 x side just above the minimum is used as is: 17 mm -> 20.4.
        let b = aabb([0.0; 3], [17.0, 1.0, 1.0]);
        let (_, half) = plane(&ConstructionGeom::Plane(Plane::XY), &b);
        assert!((half - 10.2).abs() < 1e-12);
    }

    #[test]
    fn an_axis_goes_through_the_projected_centre_with_1_2_diagonals() {
        let b = aabb([0.0; 3], [20.0, 30.0, 60.0]); // diagonal 70
        let l = Line3 {
            point: Point3::new(100.0, 0.0, 5.0),
            dir: Dir3::X,
        };
        let s = axis(&ConstructionGeom::Axis(l), &b);
        // Centre (10, 15, 30) projects to (10, 0, 5); length 84.
        close(s.a, Point3::new(-32.0, 0.0, 5.0));
        close(s.b, Point3::new(52.0, 0.0, 5.0));
        assert!((s.length() - 84.0).abs() < 1e-9);
    }

    #[test]
    fn without_bodies_planes_and_axes_sit_at_their_own_origin() {
        let mut p = Plane::XZ;
        p.origin = Point3::new(1.0, 2.0, 3.0);
        let (frame, half) = plane(&ConstructionGeom::Plane(p), &Aabb3::EMPTY);
        assert_eq!((frame.origin, half), (p.origin, 10.0));
        let s = axis(
            &ConstructionGeom::Axis(Line3 {
                point: p.origin,
                dir: Dir3::Y,
            }),
            &Aabb3::EMPTY,
        );
        close(s.a, Point3::new(1.0, -8.0, 3.0));
        close(s.b, Point3::new(1.0, 12.0, 3.0));
    }

    #[test]
    fn a_point_is_a_marker_whatever_the_bounds() {
        let p = Point3::new(4.0, 5.0, 6.0);
        for b in [Aabb3::EMPTY, aabb([0.0; 3], [500.0; 3])] {
            let d = display_extent(&ConstructionGeom::Point(p), &b);
            assert_eq!(d, ConstructionDisplay::PointMarker(p));
            assert_eq!(d.center(), p);
            assert_eq!(d.corners(), None);
        }
    }

    #[test]
    fn placed_geometry_moves_rigidly() {
        let t = Transform::from_translation(Vec3::new(1.0, 2.0, 3.0));
        let moved = placed(&ConstructionGeom::Plane(Plane::XY), &t);
        let ConstructionGeom::Plane(p) = moved else {
            panic!("{moved:?}")
        };
        close(p.origin, Point3::new(1.0, 2.0, 3.0));
        assert_eq!(p.normal, Plane::XY.normal);
        let turned = Transform::from_rotation_z(std::f64::consts::FRAC_PI_2);
        let axis = ConstructionGeom::Axis(Line3 {
            point: Point3::X,
            dir: Dir3::X,
        });
        let ConstructionGeom::Axis(l) = placed(&axis, &turned) else {
            panic!("not an axis")
        };
        close(l.point, Point3::Y);
        close(l.dir.get(), Point3::Y);
        let g = ConstructionGeom::Point(Point3::X);
        assert_eq!(placed(&g, &Transform::IDENTITY), g);
        assert_eq!(
            placed(&g, &t),
            ConstructionGeom::Point(Point3::new(2.0, 2.0, 3.0))
        );
    }

    #[test]
    fn corners_span_the_square() {
        let d = display_extent(&ConstructionGeom::Plane(Plane::XY), &Aabb3::EMPTY);
        let c = d.corners().unwrap();
        close(c[0], Point3::new(-10.0, -10.0, 0.0));
        close(c[2], Point3::new(10.0, 10.0, 0.0));
    }
}
