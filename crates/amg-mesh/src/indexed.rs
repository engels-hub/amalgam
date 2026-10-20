//! [`IndexedMesh`]: the interchange type. It holds anything a file can hold (non-manifold,
//! duplicate, degenerate and unreferenced elements); only [`IndexedMesh::validate`] rejects
//! structurally broken data.

use amg_core::{Aabb3, Mat3, Point3, Tolerance, Transform, Vec3};
use rayon::prelude::*;

use crate::attrs::MeshAttrs;
use crate::error::{MeshError, MeshResult};
use crate::groups::FaceGroups;
use crate::hash::ContentHasher;
use crate::weld::{compact_roots, to_u32, weld_roots};
use crate::{FaceId, INVALID};

/// Chunk size of ordered parallel reductions (fixed so results do not depend on the thread
/// count).
const REDUCE_CHUNK: usize = 8192;

/// Vertex welding options.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldOpts {
    /// Merge distance in mm; `0.0` merges bit-equal positions only. Repair uses
    /// [`Tolerance::mesh`] (1e-5 mm).
    pub tolerance: f64,
}

impl WeldOpts {
    /// Bit-equal positions only.
    pub const EXACT: WeldOpts = WeldOpts { tolerance: 0.0 };
}

impl Default for WeldOpts {
    /// [`Tolerance::mesh`]`().linear`.
    fn default() -> Self {
        WeldOpts {
            tolerance: Tolerance::mesh().linear,
        }
    }
}

/// Vertex-normal weighting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NormalWeighting {
    /// Every incident face counts the same.
    Uniform,
    /// Weighted by face area.
    Area,
    /// Weighted by the corner angle (Thürmer–Wüthrich).
    Angle,
}

/// Index remap produced by compaction, welding or garbage collection: old index → new index
/// (`INVALID` = dropped).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Remap {
    /// Per old vertex.
    pub vertices: Vec<u32>,
    /// Per old face.
    pub faces: Vec<u32>,
}

/// f32 buffers for `amg-render`, rebased to `origin` before the cast. Face order is preserved,
/// so GPU `primitive_index` == face index for picking.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RenderBuffers {
    /// Subtracted from every position before the f32 cast.
    pub origin: Point3,
    /// Rebased positions.
    pub positions: Vec<[f32; 3]>,
    /// Unit normals, one per position.
    pub normals: Vec<[f32; 3]>,
    /// Three indices per face.
    pub indices: Vec<u32>,
}

/// Triangle mesh as flat arrays plus attributes and face groups.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct IndexedMesh {
    /// Vertex positions, mm.
    pub positions: Vec<Point3>,
    /// Counter-clockwise vertex indices per face.
    pub tris: Vec<[u32; 3]>,
    /// Optional attributes.
    pub attrs: MeshAttrs,
    /// Face groups (`per_face.len() == tris.len()`).
    pub groups: FaceGroups,
}

#[inline]
fn tri_normal_raw(a: Point3, b: Point3, c: Point3) -> Vec3 {
    (b - a).cross(c - a)
}

fn ordered_sum<F: Fn(usize) -> f64 + Sync>(n: usize, f: F) -> f64 {
    let parts: Vec<f64> = (0..n.div_ceil(REDUCE_CHUNK))
        .into_par_iter()
        .map(|c| {
            let lo = c * REDUCE_CHUNK;
            (lo..(lo + REDUCE_CHUNK).min(n)).map(&f).sum::<f64>()
        })
        .collect();
    parts.iter().sum()
}

impl IndexedMesh {
    /// Mesh with one default face group and no attributes. Validates index ranges.
    pub fn new(positions: Vec<Point3>, tris: Vec<[u32; 3]>) -> MeshResult<Self> {
        let groups = FaceGroups::new(tris.len());
        let m = IndexedMesh {
            positions,
            tris,
            attrs: MeshAttrs::default(),
            groups,
        };
        m.validate()?;
        Ok(m)
    }

    /// Welds a triangle soup (three positions per triangle) into a mesh. Vertex ids follow
    /// first appearance; triangles keep their order (including ones that become degenerate).
    pub fn from_soup(tris: &[[Point3; 3]], weld: WeldOpts) -> MeshResult<Self> {
        let n = tris
            .len()
            .checked_mul(3)
            .ok_or(MeshError::TooLarge { count: tris.len() })?;
        to_u32(n)?;
        let pts: &[Point3] = tris.as_flattened();
        let roots = weld_roots(pts, weld.tolerance)?;
        let (new_id, kept) = compact_roots(&roots);
        let positions: Vec<Point3> = kept.iter().map(|&i| pts[i as usize]).collect();
        let tris: Vec<[u32; 3]> = new_id.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
        Ok(IndexedMesh {
            positions,
            groups: FaceGroups::new(tris.len()),
            tris,
            attrs: MeshAttrs::default(),
        })
    }

    /// Checks index ranges, counts, attribute lengths and the face-group table.
    pub fn validate(&self) -> MeshResult<()> {
        let nv = self.positions.len();
        to_u32(nv)?;
        to_u32(self.tris.len())?;
        let max = self
            .tris
            .par_chunks(REDUCE_CHUNK)
            .map(|c| {
                c.iter()
                    .map(|t| t[0].max(t[1]).max(t[2]))
                    .max()
                    .unwrap_or(0)
            })
            .max();
        if max.is_some_and(|m| m as usize >= nv) {
            let index = self
                .tris
                .iter()
                .flatten()
                .copied()
                .find(|&i| i as usize >= nv)
                .unwrap_or(INVALID);
            return Err(MeshError::IndexOutOfRange { index, len: nv });
        }
        self.attrs.validate(nv, self.tris.len())?;
        self.groups.validate(self.tris.len())
    }

    /// Number of vertices (including unreferenced ones).
    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }

    /// Number of faces.
    pub fn face_count(&self) -> usize {
        self.tris.len()
    }

    /// Corner positions of face `f`.
    pub fn triangle(&self, f: FaceId) -> [Point3; 3] {
        self.tris[f.0 as usize].map(|i| self.positions[i as usize])
    }

    /// Unit normal of face `f`; `Vec3::ZERO` if degenerate.
    pub fn face_normal(&self, f: FaceId) -> Vec3 {
        let [a, b, c] = self.triangle(f);
        tri_normal_raw(a, b, c).normalize_or_zero()
    }

    /// Area of face `f`, mm².
    pub fn face_area(&self, f: FaceId) -> f64 {
        let [a, b, c] = self.triangle(f);
        0.5 * tri_normal_raw(a, b, c).length()
    }

    /// Bounds of all vertices (including unreferenced ones).
    pub fn aabb(&self) -> Aabb3 {
        self.positions
            .par_chunks(REDUCE_CHUNK)
            .map(|c| Aabb3::from_points(c.iter().copied()))
            .reduce(|| Aabb3::EMPTY, |a, b| a.union(&b))
    }

    /// Total area, mm².
    pub fn area(&self) -> f64 {
        ordered_sum(self.tris.len(), |f| self.face_area(FaceId(f as u32)))
    }

    /// Signed volume, mm³ (divergence theorem); meaningful only if closed and oriented.
    pub fn signed_volume(&self) -> f64 {
        let Some(&o) = self.positions.first() else {
            return 0.0;
        };
        ordered_sum(self.tris.len(), |f| {
            let [a, b, c] = self.triangle(FaceId(f as u32));
            (a - o).dot((b - o).cross(c - o))
        }) / 6.0
    }

    /// Unit vertex normals; `Vec3::ZERO` for unreferenced vertices or where they cancel.
    pub fn vertex_normals(&self, w: NormalWeighting) -> Vec<Vec3> {
        let mut acc = vec![Vec3::ZERO; self.positions.len()];
        for t in &self.tris {
            let p = t.map(|i| self.positions[i as usize]);
            let n = tri_normal_raw(p[0], p[1], p[2]);
            match w {
                NormalWeighting::Area => {
                    for &i in t {
                        acc[i as usize] += n;
                    }
                }
                NormalWeighting::Uniform => {
                    let u = n.normalize_or_zero();
                    for &i in t {
                        acc[i as usize] += u;
                    }
                }
                NormalWeighting::Angle => {
                    let u = n.normalize_or_zero();
                    for k in 0..3 {
                        let e1 = p[(k + 1) % 3] - p[k];
                        let e2 = p[(k + 2) % 3] - p[k];
                        let ang = e1.angle_between(e2);
                        if ang.is_finite() {
                            acc[t[k] as usize] += u * ang;
                        }
                    }
                }
            }
        }
        acc.par_iter_mut().for_each(|n| *n = n.normalize_or_zero());
        acc
    }

    /// Applies `t` to positions and vertex normals (inverse transpose, renormalised). A
    /// mirroring transform (negative determinant) also reverses the winding of every face, so
    /// normals and signed volume keep their sense.
    pub fn transform(&mut self, t: &Transform) {
        self.positions
            .par_iter_mut()
            .for_each(|p| *p = t.transform_point3(*p));
        let m: Mat3 = t.matrix3;
        if let Some(ns) = &mut self.attrs.vertex_normals {
            let nm = m.inverse().transpose();
            ns.par_iter_mut()
                .for_each(|n| *n = (nm * *n).normalize_or_zero());
        }
        if m.determinant() < 0.0 {
            self.tris.par_iter_mut().for_each(|t| t.swap(1, 2));
            if let Some(uv) = &mut self.attrs.corner_uvs {
                uv.par_iter_mut().for_each(|c| c.swap(1, 2));
            }
            if let Some(tp) = &mut self.attrs.tri_properties {
                tp.par_iter_mut().for_each(|p| p.index.swap(1, 2));
            }
        }
    }

    /// Appends `other`. Its face groups get fresh ids here; an attribute present on only one
    /// side is kept and filled with defaults on the other.
    pub fn append(&mut self, other: &IndexedMesh) {
        let off = self.positions.len() as u32;
        let (sv, sf) = (self.positions.len(), self.tris.len());
        self.attrs.append(
            &other.attrs,
            (sv, sf),
            (other.positions.len(), other.tris.len()),
        );
        self.groups.ensure_len(sf);
        self.groups.append(&other.groups);
        // `other` may come without groups (for example built field by field).
        self.groups.ensure_len(sf + other.tris.len());
        self.positions.extend_from_slice(&other.positions);
        self.tris
            .extend(other.tris.iter().map(|t| t.map(|i| i + off)));
    }

    /// Merges vertices within `opts.tolerance` (transitively). The smallest old index of each
    /// cluster survives with its exact position and vertex attributes. Faces are kept (some may
    /// become degenerate; repair removes them).
    pub fn weld(&mut self, opts: WeldOpts) -> Remap {
        let n_faces = self.tris.len();
        let Ok(roots) = weld_roots(&self.positions, opts.tolerance) else {
            // Unreachable for a validated mesh (vertex count < 2^31 is checked by validate
            // only up to u32; fall back to no welding beyond 2^31 vertices).
            return Remap {
                vertices: (0..self.positions.len() as u32).collect(),
                faces: (0..n_faces as u32).collect(),
            };
        };
        let (new_id, kept) = compact_roots(&roots);
        let keep: Vec<bool> = roots
            .iter()
            .enumerate()
            .map(|(i, &r)| r as usize == i)
            .collect();
        self.positions = kept.iter().map(|&i| self.positions[i as usize]).collect();
        self.attrs.retain(&keep, &vec![true; n_faces]);
        self.tris
            .par_iter_mut()
            .for_each(|t| *t = t.map(|i| new_id[i as usize]));
        Remap {
            vertices: new_id,
            faces: (0..n_faces as u32).collect(),
        }
    }

    /// Drops vertices no face references (order preserved).
    pub fn compact(&mut self) -> Remap {
        let mut used = vec![false; self.positions.len()];
        for &i in self.tris.iter().flatten() {
            used[i as usize] = true;
        }
        let mut map = vec![INVALID; used.len()];
        let mut next = 0u32;
        for (i, &u) in used.iter().enumerate() {
            if u {
                map[i] = next;
                next += 1;
            }
        }
        let mut i = 0;
        self.positions.retain(|_| {
            i += 1;
            used[i - 1]
        });
        self.attrs.retain(&used, &vec![true; self.tris.len()]);
        self.tris
            .par_iter_mut()
            .for_each(|t| *t = t.map(|v| map[v as usize]));
        Remap {
            vertices: map,
            faces: (0..self.tris.len() as u32).collect(),
        }
    }

    /// 64-bit hash of positions (bits), faces, attributes and face groups. Stable across runs
    /// and builds; used as the feature-cache key.
    pub fn content_hash(&self) -> u64 {
        let mut h = ContentHasher::new();
        h.word(self.positions.len() as u64);
        for p in &self.positions {
            h.f64(p.x);
            h.f64(p.y);
            h.f64(p.z);
        }
        h.word(self.tris.len() as u64);
        for t in &self.tris {
            h.u32x2(t[0], t[1]);
            h.word(u64::from(t[2]));
        }
        let a = &self.attrs;
        h.word(
            u64::from(a.vertex_normals.is_some())
                | u64::from(a.vertex_colors.is_some()) << 1
                | u64::from(a.corner_uvs.is_some()) << 2
                | u64::from(a.tri_properties.is_some()) << 3,
        );
        for n in a.vertex_normals.iter().flatten() {
            h.f64(n.x);
            h.f64(n.y);
            h.f64(n.z);
        }
        for c in a.vertex_colors.iter().flatten() {
            h.word(u64::from(u32::from_le_bytes(c.0)));
        }
        for uv in a.corner_uvs.iter().flatten() {
            for p in uv {
                h.f64(p.x);
                h.f64(p.y);
            }
        }
        for tp in a.tri_properties.iter().flatten() {
            h.u32x2(tp.group, tp.index[0]);
            h.u32x2(tp.index[1], tp.index[2]);
        }
        let (per_face, info, next) = self.groups.raw();
        h.word(u64::from(next));
        for pair in per_face.chunks(2) {
            h.u32x2(pair[0].0, pair.get(1).map_or(INVALID, |g| g.0));
        }
        let mut buf = Vec::new();
        for (id, gi) in info {
            buf.clear();
            crate::body::encode_group_info(&mut buf, *id, gi);
            h.bytes(&buf);
        }
        h.finish()
    }

    /// f32 render buffers rebased to `origin`. `smooth_normals` shares vertices and uses the
    /// mesh's vertex normals (angle-weighted ones if absent); otherwise every face gets three
    /// own vertices with the flat face normal.
    pub fn render_buffers(&self, origin: Point3, smooth_normals: bool) -> RenderBuffers {
        let cast = |p: Point3| (p - origin).as_vec3().to_array();
        if smooth_normals {
            let computed;
            let normals = match &self.attrs.vertex_normals {
                Some(n) => n,
                None => {
                    computed = self.vertex_normals(NormalWeighting::Angle);
                    &computed
                }
            };
            RenderBuffers {
                origin,
                positions: self.positions.par_iter().map(|&p| cast(p)).collect(),
                normals: normals.par_iter().map(|n| n.as_vec3().to_array()).collect(),
                indices: self.tris.as_flattened().to_vec(),
            }
        } else {
            let n = self.tris.len();
            let mut positions = vec![[0f32; 3]; 3 * n];
            let mut normals = vec![[0f32; 3]; 3 * n];
            positions
                .par_chunks_mut(3)
                .zip(normals.par_chunks_mut(3))
                .enumerate()
                .for_each(|(f, (pc, nc))| {
                    let tri = self.triangle(FaceId(f as u32));
                    let nn = tri_normal_raw(tri[0], tri[1], tri[2])
                        .normalize_or_zero()
                        .as_vec3()
                        .to_array();
                    for k in 0..3 {
                        pc[k] = cast(tri[k]);
                        nc[k] = nn;
                    }
                });
            RenderBuffers {
                origin,
                positions,
                normals,
                indices: (0..3 * n as u32).collect(),
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // render buffers hold exactly representable values
mod tests {
    use super::*;
    use crate::attrs::{Rgba8, TriProperty};
    use crate::groups::{FaceGroupId, FaceGroupInfo, GroupOrigin};
    use amg_core::{Point2, Quat};

    /// Unit cube, outward.
    pub(crate) fn cube() -> IndexedMesh {
        let p = (0..8)
            .map(|i| Point3::new((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64))
            .collect();
        let t = vec![
            [0, 2, 1],
            [1, 2, 3],
            [4, 5, 6],
            [5, 7, 6],
            [0, 1, 4],
            [1, 5, 4],
            [2, 6, 3],
            [3, 6, 7],
            [0, 4, 2],
            [2, 4, 6],
            [1, 3, 5],
            [3, 7, 5],
        ];
        IndexedMesh::new(p, t).unwrap()
    }

    #[test]
    fn cube_measures() {
        let m = cube();
        assert!((m.area() - 6.0).abs() < 1e-12);
        assert!((m.signed_volume() - 1.0).abs() < 1e-12);
        assert_eq!(m.aabb(), Aabb3::new(Point3::ZERO, Point3::ONE));
        assert_eq!(m.face_normal(FaceId(0)), Vec3::NEG_Z);
        assert!((m.face_area(FaceId(0)) - 0.5).abs() < 1e-15);
        let n = m.vertex_normals(NormalWeighting::Angle);
        assert!((n[7] - Vec3::ONE.normalize()).length() < 1e-12);
        for w in [NormalWeighting::Uniform, NormalWeighting::Area] {
            assert!(m.vertex_normals(w)[0].dot(Vec3::NEG_ONE) > 0.0);
        }
    }

    #[test]
    fn validate_rejects_bad_index() {
        let e = IndexedMesh::new(vec![Point3::ZERO; 2], vec![[0, 1, 2]]).unwrap_err();
        assert!(matches!(e, MeshError::IndexOutOfRange { index: 2, len: 2 }));
    }

    #[test]
    fn mirror_transform_keeps_volume_sign() {
        let mut m = cube();
        m.attrs.vertex_normals = Some(m.vertex_normals(NormalWeighting::Angle));
        m.transform(&Transform::from_scale(Vec3::new(-2.0, 1.0, 1.0)));
        assert!((m.signed_volume() - 2.0).abs() < 1e-12);
        let n = m.attrs.vertex_normals.as_ref().unwrap();
        assert!(n[7].x < 0.0 && n[7].y > 0.0);
        let mut r = cube();
        r.transform(&Transform::from_rotation_translation(
            Quat::from_rotation_z(0.7),
            Vec3::new(5.0, 0.0, 0.0),
        ));
        assert!((r.signed_volume() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn soup_weld_and_compact() {
        let m = cube();
        let soup: Vec<[Point3; 3]> = (0..m.face_count())
            .map(|f| m.triangle(FaceId(f as u32)))
            .collect();
        let w = IndexedMesh::from_soup(&soup, WeldOpts::EXACT).unwrap();
        assert_eq!(w.vertex_count(), 8);
        assert_eq!(w.face_count(), 12);
        assert!((w.signed_volume() - 1.0).abs() < 1e-12);
        // Jittered soup welds with tolerance.
        let jit: Vec<[Point3; 3]> = soup
            .iter()
            .enumerate()
            .map(|(i, t)| t.map(|p| p + Vec3::splat(1e-7 * (i % 3) as f64)))
            .collect();
        assert_eq!(
            IndexedMesh::from_soup(&jit, WeldOpts::EXACT)
                .unwrap()
                .vertex_count(),
            24
        );
        assert_eq!(
            IndexedMesh::from_soup(&jit, WeldOpts::default())
                .unwrap()
                .vertex_count(),
            8
        );
        let mut c = IndexedMesh::new(
            vec![Point3::ZERO, Point3::X, Point3::Y, Point3::Z],
            vec![[0, 1, 3]],
        )
        .unwrap();
        c.attrs.vertex_colors = Some(vec![
            Rgba8([1; 4]),
            Rgba8([2; 4]),
            Rgba8([3; 4]),
            Rgba8([4; 4]),
        ]);
        let r = c.compact();
        assert_eq!(r.vertices, vec![0, 1, INVALID, 2]);
        assert_eq!(c.tris, vec![[0, 1, 2]]);
        assert_eq!(
            c.attrs.vertex_colors.unwrap(),
            vec![Rgba8([1; 4]), Rgba8([2; 4]), Rgba8([4; 4])]
        );
    }

    #[test]
    fn weld_remaps_in_place() {
        let mut m = IndexedMesh::new(
            vec![
                Point3::ZERO,
                Point3::X,
                Point3::Y,
                Point3::X,
                Point3::new(1.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [3, 4, 2]],
        )
        .unwrap();
        let r = m.weld(WeldOpts::EXACT);
        assert_eq!(r.vertices, vec![0, 1, 2, 1, 3]);
        assert_eq!(m.tris, vec![[0, 1, 2], [1, 3, 2]]);
        assert_eq!(m.vertex_count(), 4);
    }

    #[test]
    fn append_reallocates_groups() {
        let mut a = cube();
        let mut b = cube();
        let g = b.groups.alloc(FaceGroupInfo::new(GroupOrigin::Cap));
        b.groups.set(FaceId(3), g);
        b.attrs.tri_properties = Some(vec![
            TriProperty {
                group: 1,
                index: [0; 3]
            };
            12
        ]);
        a.append(&b);
        assert_eq!(a.face_count(), 24);
        assert!(a.validate().is_ok());
        assert_eq!(a.groups.group_of(FaceId(12)), FaceGroupId(1));
        assert_eq!(a.groups.group_of(FaceId(15)), FaceGroupId(2));
        assert_eq!(
            a.attrs.tri_properties.as_ref().unwrap()[0],
            TriProperty::NONE
        );
        assert!((a.signed_volume() - 2.0).abs() < 1e-12);
        let fp = a.groups.fingerprint(&a, FaceGroupId(2)).unwrap();
        assert!((fp.area - 0.5).abs() < 1e-15);
        assert!(a.groups.fingerprint(&a, FaceGroupId(9)).is_none());
    }

    #[test]
    fn content_hash_sensitivity() {
        let a = cube();
        let mut b = cube();
        assert_eq!(a.content_hash(), b.content_hash());
        b.attrs.corner_uvs = Some(vec![[Point2::ZERO; 3]; 12]);
        assert_ne!(a.content_hash(), b.content_hash());
        let mut c = cube();
        c.positions[3].x = f64::from_bits(c.positions[3].x.to_bits() + 1);
        assert_ne!(a.content_hash(), c.content_hash());
    }

    #[test]
    fn render_buffers_shapes() {
        let m = cube();
        let o = Point3::splat(0.5);
        let s = m.render_buffers(o, true);
        assert_eq!(s.positions.len(), 8);
        assert_eq!(s.indices.len(), 36);
        assert_eq!(s.positions[0], [-0.5; 3]);
        let f = m.render_buffers(o, false);
        assert_eq!(f.positions.len(), 36);
        assert_eq!(f.normals[0], [0.0, 0.0, -1.0]);
        assert_eq!(f.indices[35], 35);
    }
}
