//! Physical properties of closed triangle meshes (S05 §4, INS-03; formulas in
//! `docs/notes/inspection-geometry.md` §4).
//!
//! - [`mesh_mass_props`]: area, volume, mass, centre of mass, inertia about the CoM and the
//!   world origin, principal moments and axes of one mesh body in world space. Divergence
//!   theorem over signed tetrahedra relative to the AABB centre (Tonon 2004, Eberly), so the
//!   result is exact for the polyhedron up to rounding.
//! - [`combine`]: the totals of several bodies by the parallel-axis rule.
//!
//! The reduction is deterministic: faces are cut into fixed chunks of [`CHUNK`] by index,
//! each chunk is summed sequentially with compensated (Neumaier) summation, and the chunk sums
//! are combined in chunk order. The result is therefore bit-identical for any thread count.

#[cfg(test)]
mod tests;

use amg_core::{Area, Density, Dir3, Mass, Mat3, Point3, Tolerance, Transform, Vec3, Volume};
use amg_jobs::JobCtx;
use amg_mesh::MeshBody;
use rayon::prelude::*;

use super::InspectError;
use super::measure::Exactness;

/// Faces per reduction chunk (fixed, so sums do not depend on the thread count).
pub const CHUNK: usize = 8192;

/// The physical properties of one body or of a selection (world space).
#[derive(Clone, Debug, PartialEq)]
pub struct MassProps {
    /// Surface area.
    pub area: Area,
    /// Enclosed volume; `None` if the mesh is not watertight.
    pub volume: Option<Volume>,
    /// The density used.
    pub density: Density,
    /// `density × volume`.
    pub mass: Option<Mass>,
    /// Centre of mass, world.
    pub center_of_mass: Option<Point3>,
    /// Inertia tensor, t·mm², about the CoM, world axes.
    pub inertia_com: Option<Mat3>,
    /// Inertia tensor, t·mm², about the world origin.
    pub inertia_origin: Option<Mat3>,
    /// Principal moments, ascending, t·mm².
    pub principal_moments: Option<[f64; 3]>,
    /// Principal axes in the order of the moments: right-handed; the first two have their
    /// largest component positive (note §4).
    pub principal_axes: Option<[Dir3; 3]>,
    /// Why the volume-based values are `None`.
    pub not_watertight: Option<String>,
    /// Exactness of the values.
    pub exactness: Exactness,
}

/// Compensated running sum (Neumaier / Knuth two-sum, branch-free).
#[derive(Clone, Copy, Default)]
struct Sum {
    s: f64,
    c: f64,
}

impl Sum {
    #[inline]
    fn add(&mut self, x: f64) {
        let t = self.s + x;
        let bb = t - self.s;
        self.c += (self.s - (t - bb)) + (x - bb);
        self.s = t;
    }

    fn value(self) -> f64 {
        self.s + self.c
    }
}

/// Sums of one chunk: Σd, Σd·(a+b+c) (3), Σd·(second moments) (6: xx yy zz xy xz yz), Σ area.
#[derive(Clone, Copy, Default)]
struct Acc([Sum; 11]);

impl Acc {
    /// Adds the (sum, compensation) pairs of `o` in order.
    fn merge(&mut self, o: &Acc) {
        for (a, b) in self.0.iter_mut().zip(&o.0) {
            a.add(b.s);
            a.add(b.c);
        }
    }
}

fn chunk_sums(
    tris: &[[u32; 3]],
    pos: &[Point3],
    t: &Transform,
    r: Point3,
    orient: f64,
) -> Result<Acc, InspectError> {
    let mut acc = Acc::default();
    let m = t.matrix3;
    let tr = t.translation;
    let at = |i: u32| -> Result<Vec3, InspectError> {
        let p = pos
            .get(i as usize)
            .ok_or_else(|| InspectError::BadElement(format!("vertex {i}")))?;
        Ok(m * *p + tr - r)
    };
    for f in tris {
        let (a, b, c) = (at(f[0])?, at(f[1])?, at(f[2])?);
        let d = orient * a.dot(b.cross(c));
        let s = a + b + c;
        let k = &mut acc.0;
        k[0].add(d);
        k[1].add(d * s.x);
        k[2].add(d * s.y);
        k[3].add(d * s.z);
        let q = |u: f64, v: f64, w: f64, x: f64| u * v + w * x;
        // aaᵀ + bbᵀ + ccᵀ + s sᵀ.
        k[4].add(d * (a.x * a.x + b.x * b.x + c.x * c.x + s.x * s.x));
        k[5].add(d * (a.y * a.y + b.y * b.y + c.y * c.y + s.y * s.y));
        k[6].add(d * (a.z * a.z + b.z * b.z + c.z * c.z + s.z * s.z));
        k[7].add(d * (q(a.x, a.y, b.x, b.y) + c.x * c.y + s.x * s.y));
        k[8].add(d * (q(a.x, a.z, b.x, b.z) + c.x * c.z + s.x * s.z));
        k[9].add(d * (q(a.y, a.z, b.y, b.z) + c.y * c.z + s.y * s.z));
        k[10].add(0.5 * (b - a).cross(c - a).length());
    }
    Ok(acc)
}

fn is_finite_transform(t: &Transform) -> bool {
    t.matrix3.is_finite() && t.translation.is_finite()
}

/// Mass properties of `body` placed by `to_world`, with `density` (t/mm³).
///
/// `to_world` may be any finite affine map; a mirroring one flips the winding, which is
/// accounted for. Cancellable through `ctx`.
///
/// A mesh that is not closed and consistently oriented, or is inverted, or encloses no volume,
/// gets `volume`, `mass`, `center_of_mass`, `inertia_*` and the principal values `None` with
/// [`MassProps::not_watertight`] set; the area is always given.
///
/// # Errors
/// [`InspectError::Degenerate`] for a non-finite transform or vertex, [`InspectError::Cancelled`],
/// [`InspectError::Mesh`].
pub fn mesh_mass_props(
    body: &MeshBody,
    to_world: &Transform,
    density: Density,
    ctx: &JobCtx,
) -> Result<MassProps, InspectError> {
    if !is_finite_transform(to_world) {
        return Err(InspectError::Degenerate("the transform is not finite"));
    }
    let mesh = body.mesh();
    let (pos, tris) = (&mesh.positions[..], &mesh.tris[..]);
    let orient = if to_world.matrix3.determinant() < 0.0 {
        -1.0
    } else {
        1.0
    };
    // Reference point: the world AABB centre (min / max are exact, so order does not matter).
    let (lo, hi, finite) = pos
        .par_chunks(CHUNK)
        .map(|c| {
            let mut lo = Point3::splat(f64::INFINITY);
            let mut hi = Point3::splat(f64::NEG_INFINITY);
            let mut finite = true;
            for p in c {
                let w = to_world.transform_point3(*p);
                finite &= w.is_finite();
                lo = lo.min(w);
                hi = hi.max(w);
            }
            (lo, hi, finite)
        })
        .reduce(
            || {
                (
                    Point3::splat(f64::INFINITY),
                    Point3::splat(f64::NEG_INFINITY),
                    true,
                )
            },
            |a, b| (a.0.min(b.0), a.1.max(b.1), a.2 && b.2),
        );
    if !finite {
        return Err(InspectError::Degenerate("a vertex is not finite"));
    }
    ctx.checkpoint()?;
    let r = if tris.is_empty() {
        Point3::ZERO
    } else {
        (lo + hi) * 0.5
    };

    let chunks: Result<Vec<Acc>, InspectError> = tris
        .par_chunks(CHUNK)
        .map(|c| {
            ctx.checkpoint()?;
            chunk_sums(c, pos, to_world, r, orient)
        })
        .collect();
    let mut total = Acc::default();
    for c in &chunks? {
        total.merge(c);
    }
    let k = total.0.map(Sum::value);
    let area = Area::mm2(k[10]);

    let reason = if tris.is_empty() {
        Some("the body has no faces".to_owned())
    } else if !super::cache::oriented(body, ctx)? {
        Some(
            "the mesh is not closed and consistently oriented (holes, non-manifold or flipped faces)"
                .to_owned(),
        )
    } else if k[0] < 0.0 {
        Some("the mesh is inverted (negative volume)".to_owned())
    } else if k[0] == 0.0 {
        Some("the mesh encloses no volume".to_owned())
    } else {
        None
    };
    if let Some(reason) = reason {
        return Ok(MassProps {
            area,
            volume: None,
            density,
            mass: None,
            center_of_mass: None,
            inertia_com: None,
            inertia_origin: None,
            principal_moments: None,
            principal_axes: None,
            not_watertight: Some(reason),
            exactness: Exactness::Exact,
        });
    }

    let vol = k[0] / 6.0;
    let rho = density.get();
    let mass = rho * vol;
    let g = Vec3::new(k[1], k[2], k[3]) / 24.0 / vol;
    let c = |i: usize| k[i] / 120.0;
    // Covariance about r.
    let cov = Mat3::from_cols(
        Vec3::new(c(4), c(7), c(8)),
        Vec3::new(c(7), c(5), c(9)),
        Vec3::new(c(8), c(9), c(6)),
    );
    let trace = c(4) + c(5) + c(6);
    let i_r = (Mat3::IDENTITY * trace - cov) * rho;
    let i_com = i_r - parallel_axis(mass, g);
    let com = r + g;
    let i_origin = i_com + parallel_axis(mass, com);
    let (moments, axes) = principal(&i_com);
    Ok(MassProps {
        area,
        volume: Some(Volume::mm3(vol)),
        density,
        mass: Some(Mass::from_canonical(mass)),
        center_of_mass: Some(com),
        inertia_com: Some(i_com),
        inertia_origin: Some(i_origin),
        principal_moments: Some(moments),
        principal_axes: Some(axes),
        not_watertight: None,
        exactness: Exactness::Exact,
    })
}

/// `m (|d|² 𝟙 − d dᵀ)`: the inertia of a point mass at `d`.
fn parallel_axis(m: f64, d: Vec3) -> Mat3 {
    let outer = Mat3::from_cols(d * d.x, d * d.y, d * d.z);
    (Mat3::IDENTITY * d.length_squared() - outer) * m
}

/// Totals of several bodies: areas, volumes and masses add, the centre of mass is the
/// mass-weighted mean and the inertia tensors add after moving each to the common CoM by the
/// parallel-axis rule (the tensor about the origin is the plain sum). The density is the mean
/// density (total mass / total volume). Parts are combined in slice order.
///
/// If any part lacks volume-based values the total has none either, with a reason naming how
/// many parts are not watertight; the area still adds. An empty slice gives zero area and no
/// volume.
pub fn combine(parts: &[MassProps]) -> MassProps {
    let mut area = Sum::default();
    let mut deflection: Option<f64> = None;
    for p in parts {
        area.add(p.area.get());
        if let Exactness::Tessellated { deflection: d } = p.exactness {
            deflection = Some(deflection.map_or(d, |x| x.max(d)));
        }
    }
    let exactness = deflection.map_or(Exactness::Exact, |deflection| Exactness::Tessellated {
        deflection,
    });
    let area = Area::from_canonical(area.value());
    let bad = parts.iter().filter(|p| p.mass.is_none()).count();
    let none = |reason: String, density: Density| MassProps {
        area,
        volume: None,
        density,
        mass: None,
        center_of_mass: None,
        inertia_com: None,
        inertia_origin: None,
        principal_moments: None,
        principal_axes: None,
        not_watertight: Some(reason),
        exactness,
    };
    if parts.is_empty() {
        return none("nothing to combine".into(), Density::from_canonical(0.0));
    }
    if bad > 0 {
        let reason = if parts.len() == 1 {
            parts[0]
                .not_watertight
                .clone()
                .unwrap_or_else(|| "the mesh is not watertight".into())
        } else {
            format!("{bad} of {} bodies are not watertight", parts.len())
        };
        return none(reason, parts[0].density);
    }
    let (mut vol, mut mass) = (Sum::default(), Sum::default());
    let mut first = [Sum::default(); 3];
    for p in parts {
        let (Some(v), Some(m), Some(c)) = (p.volume, p.mass, p.center_of_mass) else {
            continue;
        };
        vol.add(v.get());
        mass.add(m.get());
        for (s, x) in first.iter_mut().zip(c.to_array()) {
            s.add(m.get() * x);
        }
    }
    let (vol, mass) = (vol.value(), mass.value());
    let density = Density::from_canonical(if vol > 0.0 { mass / vol } else { 0.0 });
    if mass <= 0.0 {
        // Massless (zero density): CoM and inertia are undefined.
        return MassProps {
            volume: Some(Volume::from_canonical(vol)),
            mass: Some(Mass::from_canonical(mass)),
            ..none("the total mass is zero".into(), density)
        };
    }
    let com = Point3::new(first[0].value(), first[1].value(), first[2].value()) / mass;
    let mut i_com = [Sum::default(); 9];
    let mut i_org = [Sum::default(); 9];
    for p in parts {
        let (Some(m), Some(c), Some(ic), Some(io)) =
            (p.mass, p.center_of_mass, p.inertia_com, p.inertia_origin)
        else {
            continue;
        };
        let moved = ic + parallel_axis(m.get(), c - com);
        for (s, x) in i_com.iter_mut().zip(moved.to_cols_array()) {
            s.add(x);
        }
        for (s, x) in i_org.iter_mut().zip(io.to_cols_array()) {
            s.add(x);
        }
    }
    let mat = |s: [Sum; 9]| Mat3::from_cols_array(&s.map(Sum::value));
    let i_com = mat(i_com);
    let (moments, axes) = principal(&i_com);
    MassProps {
        area,
        volume: Some(Volume::from_canonical(vol)),
        density,
        mass: Some(Mass::from_canonical(mass)),
        center_of_mass: Some(com),
        inertia_com: Some(i_com),
        inertia_origin: Some(mat(i_org)),
        principal_moments: Some(moments),
        principal_axes: Some(axes),
        not_watertight: None,
        exactness,
    }
}

/// Eigen decomposition of a symmetric 3×3 matrix by cyclic Jacobi rotations: moments in
/// ascending order and right-handed axes (the first two with their largest-magnitude
/// component positive, the third their cross product). Deterministic.
pub(crate) fn principal(i: &Mat3) -> ([f64; 3], [Dir3; 3]) {
    let mut a = [[0.0f64; 3]; 3];
    for (r, row) in a.iter_mut().enumerate() {
        for (c, x) in row.iter_mut().enumerate() {
            *x = i.col(c)[r];
        }
    }
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let norm: f64 = a.iter().flatten().map(|x| x * x).sum::<f64>().sqrt();
    for _ in 0..64 {
        let off = (a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2]).sqrt();
        if off <= 1e-17 * norm || off == 0.0 {
            break;
        }
        for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            // A ← Jᵀ A J on columns p, q, then rows p, q.
            for row in &mut a {
                let (x, y) = (row[p], row[q]);
                row[p] = c * x - s * y;
                row[q] = s * x + c * y;
            }
            let (rp, rq) = (a[p], a[q]);
            for k in 0..3 {
                a[p][k] = c * rp[k] - s * rq[k];
                a[q][k] = s * rp[k] + c * rq[k];
            }
            for row in &mut v {
                let (x, y) = (row[p], row[q]);
                row[p] = c * x - s * y;
                row[q] = s * x + c * y;
            }
        }
    }
    let mut order = [0usize, 1, 2];
    order.sort_by(|&x, &y| a[x][x].total_cmp(&a[y][y]).then(x.cmp(&y)));
    let moments = order.map(|k| a[k][k]);
    let col = |k: usize| Vec3::new(v[0][k], v[1][k], v[2][k]);
    let fix = |u: Vec3| {
        let big = (0..3).fold(0, |m, k| {
            if u[k].abs() > u[m].abs() + 1e-12 {
                k
            } else {
                m
            }
        });
        if u[big] < 0.0 { -u } else { u }
    };
    let a0 = fix(col(order[0])).normalize();
    let a1 = fix(col(order[1]));
    // Gram-Schmidt against rounding, then the third axis closes the right-handed frame.
    let a1 = (a1 - a0 * a0.dot(a1)).normalize();
    let a2 = a0.cross(a1);
    let d = |u: Vec3| Dir3::new(u, &Tolerance::mesh()).unwrap_or(Dir3::X);
    (moments, [d(a0), d(a1), d(a2)])
}
