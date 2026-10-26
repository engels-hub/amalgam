//! [`MeshBody`]: the mesh body stored by `amg-doc`, and its versioned binary encoding.
//!
//! # `AMGM` v1 layout (all little-endian)
//! ```text
//! magic        4 B  "AMGM"
//! version      u32  1
//! flags        u32  bit 0 vertex_normals, 1 vertex_colors, 2 corner_uvs, 3 tri_properties
//! n_vertices   u64
//! n_faces      u64
//! positions    n_vertices × 3 × f64 (IEEE-754 bits)
//! tris         n_faces × 3 × u32
//! [normals]    n_vertices × 3 × f64
//! [colors]     n_vertices × 4 × u8
//! [uvs]        n_faces × 6 × f64
//! [properties] n_faces × 4 × u32 (group, index[3])
//! next_group   u32
//! n_groups     u32
//! groups       n_groups × group record (see `encode_group_info`)
//! per_face     n_faces × u32
//! checksum     u64  content hash of every preceding byte
//! ```
//! Floats are written as raw bits, so a round trip is bit-exact (NaN payloads included).

use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::sync::Arc;

use amg_core::{Point2, Point3, Vec3};

use crate::attrs::{MeshAttrs, Rgba8, TriProperty};
use crate::error::{MeshError, MeshResult};
use crate::groups::{FaceGroupId, FaceGroupInfo, FaceGroups, GroupOrigin, Primitive, ShellSide};
use crate::hash::ContentHasher;
use crate::indexed::IndexedMesh;

const MAGIC: [u8; 4] = *b"AMGM";
const VERSION: u32 = 1;
const FLAG_NORMALS: u32 = 1;
const FLAG_COLORS: u32 = 2;
const FLAG_UVS: u32 = 4;
const FLAG_PROPS: u32 = 8;
/// Elements read per chunk, so corrupt counts fail on EOF instead of allocating huge buffers.
const CHUNK: usize = 1 << 16;

/// A mesh body: a cheap-to-clone shared [`IndexedMesh`].
///
/// Lazily built derived data (`MeshSpatial`, `AnalysisReport`) is added by MESH-03 / MESH-02.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshBody {
    mesh: Arc<IndexedMesh>,
}

impl MeshBody {
    /// Wraps `mesh`.
    pub fn new(mesh: IndexedMesh) -> Self {
        Self {
            mesh: Arc::new(mesh),
        }
    }

    /// Wraps an already shared `mesh` (no copy).
    pub fn from_arc(mesh: Arc<IndexedMesh>) -> Self {
        Self { mesh }
    }

    /// The shared mesh.
    pub fn mesh(&self) -> &Arc<IndexedMesh> {
        &self.mesh
    }

    /// Writes the `AMGM` v1 encoding. Wrap `w` in a `BufWriter` if it is unbuffered.
    pub fn write_binary<W: Write>(&self, w: W) -> std::io::Result<()> {
        let m = &*self.mesh;
        let mut w = HashWriter::new(w);
        let a = &m.attrs;
        let flags = (u32::from(a.vertex_normals.is_some()) * FLAG_NORMALS)
            | (u32::from(a.vertex_colors.is_some()) * FLAG_COLORS)
            | (u32::from(a.corner_uvs.is_some()) * FLAG_UVS)
            | (u32::from(a.tri_properties.is_some()) * FLAG_PROPS);
        let mut head = Vec::with_capacity(28);
        head.extend_from_slice(&MAGIC);
        head.extend_from_slice(&VERSION.to_le_bytes());
        head.extend_from_slice(&flags.to_le_bytes());
        head.extend_from_slice(&(m.positions.len() as u64).to_le_bytes());
        head.extend_from_slice(&(m.tris.len() as u64).to_le_bytes());
        w.put(&head)?;

        w.put_items(&m.positions, |b, p| put_vec3(b, *p))?;
        w.put_items(&m.tris, |b, t| {
            for i in t {
                b.extend_from_slice(&i.to_le_bytes());
            }
        })?;
        if let Some(n) = &a.vertex_normals {
            w.put_items(n, |b, p| put_vec3(b, *p))?;
        }
        if let Some(c) = &a.vertex_colors {
            w.put_items(c, |b, c| b.extend_from_slice(&c.0))?;
        }
        if let Some(u) = &a.corner_uvs {
            w.put_items(u, |b, uv| {
                for p in uv {
                    b.extend_from_slice(&p.x.to_bits().to_le_bytes());
                    b.extend_from_slice(&p.y.to_bits().to_le_bytes());
                }
            })?;
        }
        if let Some(p) = &a.tri_properties {
            w.put_items(p, |b, p| {
                b.extend_from_slice(&p.group.to_le_bytes());
                for i in p.index {
                    b.extend_from_slice(&i.to_le_bytes());
                }
            })?;
        }
        let (per_face, info, next) = m.groups.raw();
        let mut g = Vec::new();
        g.extend_from_slice(&next.to_le_bytes());
        g.extend_from_slice(&(info.len() as u32).to_le_bytes());
        for (id, gi) in info {
            encode_group_info(&mut g, *id, gi);
        }
        w.put(&g)?;
        w.put_items(per_face, |b, g| b.extend_from_slice(&g.0.to_le_bytes()))?;
        let sum = w.hasher_finish();
        w.inner.write_all(&sum.to_le_bytes())?;
        w.inner.flush()
    }

    /// Reads an `AMGM` v1 encoding and validates the mesh. Fails with [`MeshError::Decode`]
    /// on a bad magic, unknown version or flags, truncation, checksum mismatch or invalid
    /// content.
    pub fn read_binary<R: Read>(r: R) -> MeshResult<Self> {
        let mut r = HashReader::new(r);
        let head = r.take(28)?;
        if head[0..4] != MAGIC {
            return Err(MeshError::Decode("not an AMGM mesh (bad magic)".into()));
        }
        let version = le_u32(&head[4..8]);
        if version != VERSION {
            return Err(MeshError::Decode(format!(
                "unsupported AMGM version {version} (this build reads {VERSION})"
            )));
        }
        let flags = le_u32(&head[8..12]);
        if flags & !(FLAG_NORMALS | FLAG_COLORS | FLAG_UVS | FLAG_PROPS) != 0 {
            return Err(MeshError::Decode(format!("unknown AMGM flags {flags:#x}")));
        }
        let nv = count(le_u64(&head[12..20]))?;
        let nf = count(le_u64(&head[20..28]))?;

        let positions = r.items(nv, 24, get_vec3)?;
        let tris = r.items(nf, 12, |b| {
            [le_u32(&b[0..4]), le_u32(&b[4..8]), le_u32(&b[8..12])]
        })?;
        let mut attrs = MeshAttrs::default();
        if flags & FLAG_NORMALS != 0 {
            attrs.vertex_normals = Some(r.items(nv, 24, get_vec3)?);
        }
        if flags & FLAG_COLORS != 0 {
            attrs.vertex_colors = Some(r.items(nv, 4, |b| Rgba8([b[0], b[1], b[2], b[3]]))?);
        }
        if flags & FLAG_UVS != 0 {
            attrs.corner_uvs = Some(r.items(nf, 48, |b| {
                let f = |i: usize| f64::from_bits(le_u64(&b[8 * i..8 * i + 8]));
                [
                    Point2::new(f(0), f(1)),
                    Point2::new(f(2), f(3)),
                    Point2::new(f(4), f(5)),
                ]
            })?);
        }
        if flags & FLAG_PROPS != 0 {
            attrs.tri_properties = Some(r.items(nf, 16, |b| TriProperty {
                group: le_u32(&b[0..4]),
                index: [le_u32(&b[4..8]), le_u32(&b[8..12]), le_u32(&b[12..16])],
            })?);
        }
        let gh = r.take(8)?;
        let next = le_u32(&gh[0..4]);
        let n_groups = le_u32(&gh[4..8]);
        let mut info = BTreeMap::new();
        for _ in 0..n_groups {
            let (id, gi) = decode_group_info(&mut r)?;
            if info.insert(id, gi).is_some() {
                return Err(MeshError::Decode(format!("duplicate face group {}", id.0)));
            }
        }
        let per_face = r.items(nf, 4, |b| FaceGroupId(le_u32(b)))?;
        let expected = r.hasher.finish();
        let mut sum = [0u8; 8];
        r.inner.read_exact(&mut sum).map_err(eof)?;
        if u64::from_le_bytes(sum) != expected {
            return Err(MeshError::Decode("checksum mismatch".into()));
        }
        let mesh = IndexedMesh {
            positions,
            tris,
            attrs,
            groups: FaceGroups::from_raw(per_face, info, next),
        };
        mesh.validate()
            .map_err(|e| MeshError::Decode(format!("invalid mesh content: {e}")))?;
        Ok(MeshBody::new(mesh))
    }
}

// ---- group records ----------------------------------------------------------------------

/// Group record: `id u32`, `name` (`u8` present, `u32` byte length, UTF-8), `origin` (`u8`
/// tag + payload), `primitive` (`u8` tag, 0 = none, + f64 payload).
pub(crate) fn encode_group_info(b: &mut Vec<u8>, id: FaceGroupId, gi: &FaceGroupInfo) {
    b.extend_from_slice(&id.0.to_le_bytes());
    match &gi.name {
        Some(n) => {
            b.push(1);
            b.extend_from_slice(&(n.len() as u32).to_le_bytes());
            b.extend_from_slice(n.as_bytes());
        }
        None => b.push(0),
    }
    match &gi.origin {
        GroupOrigin::Import { object } => {
            b.push(0);
            b.extend_from_slice(&object.to_le_bytes());
        }
        GroupOrigin::Segmentation => b.push(1),
        GroupOrigin::Boolean { operand, source } => {
            b.push(2);
            b.push(*operand);
            b.extend_from_slice(&source.0.to_le_bytes());
        }
        GroupOrigin::Cap => b.push(3),
        GroupOrigin::Fill => b.push(4),
        GroupOrigin::Shell { side } => {
            b.push(5);
            b.push(match side {
                ShellSide::Inside => 0,
                ShellSide::Outside => 1,
                ShellSide::Both => 2,
            });
        }
        GroupOrigin::User => b.push(6),
    }
    let f = |b: &mut Vec<u8>, v: f64| b.extend_from_slice(&v.to_bits().to_le_bytes());
    match &gi.primitive {
        None => b.push(0),
        Some(Primitive::Plane { origin, normal }) => {
            b.push(1);
            put_vec3(b, *origin);
            put_vec3(b, *normal);
        }
        Some(Primitive::Sphere { center, radius }) => {
            b.push(2);
            put_vec3(b, *center);
            f(b, *radius);
        }
        Some(Primitive::Cylinder {
            origin,
            axis,
            radius,
        }) => {
            b.push(3);
            put_vec3(b, *origin);
            put_vec3(b, *axis);
            f(b, *radius);
        }
        Some(Primitive::Cone {
            apex,
            axis,
            half_angle,
        }) => {
            b.push(4);
            put_vec3(b, *apex);
            put_vec3(b, *axis);
            f(b, *half_angle);
        }
        Some(Primitive::Torus {
            center,
            axis,
            major,
            minor,
        }) => {
            b.push(5);
            put_vec3(b, *center);
            put_vec3(b, *axis);
            f(b, *major);
            f(b, *minor);
        }
    }
}

fn decode_group_info<R: Read>(r: &mut HashReader<R>) -> MeshResult<(FaceGroupId, FaceGroupInfo)> {
    let id = FaceGroupId(le_u32(&r.take(4)?));
    let name = match r.byte()? {
        0 => None,
        1 => {
            let len = le_u32(&r.take(4)?) as usize;
            if len > 1 << 20 {
                return Err(MeshError::Decode("face group name too long".into()));
            }
            let bytes = r.take(len)?;
            Some(
                String::from_utf8(bytes)
                    .map_err(|_| MeshError::Decode("face group name is not UTF-8".into()))?,
            )
        }
        t => return Err(MeshError::Decode(format!("bad name tag {t}"))),
    };
    let origin = match r.byte()? {
        0 => GroupOrigin::Import {
            object: le_u32(&r.take(4)?),
        },
        1 => GroupOrigin::Segmentation,
        2 => {
            let operand = r.byte()?;
            GroupOrigin::Boolean {
                operand,
                source: FaceGroupId(le_u32(&r.take(4)?)),
            }
        }
        3 => GroupOrigin::Cap,
        4 => GroupOrigin::Fill,
        5 => GroupOrigin::Shell {
            side: match r.byte()? {
                0 => ShellSide::Inside,
                1 => ShellSide::Outside,
                2 => ShellSide::Both,
                t => return Err(MeshError::Decode(format!("bad shell side {t}"))),
            },
        },
        6 => GroupOrigin::User,
        t => return Err(MeshError::Decode(format!("bad group origin tag {t}"))),
    };
    let v3 = |r: &mut HashReader<R>| -> MeshResult<Vec3> { Ok(get_vec3(&r.take(24)?)) };
    let f = |r: &mut HashReader<R>| -> MeshResult<f64> { Ok(f64::from_bits(le_u64(&r.take(8)?))) };
    let primitive = match r.byte()? {
        0 => None,
        1 => Some(Primitive::Plane {
            origin: v3(r)?,
            normal: v3(r)?,
        }),
        2 => Some(Primitive::Sphere {
            center: v3(r)?,
            radius: f(r)?,
        }),
        3 => Some(Primitive::Cylinder {
            origin: v3(r)?,
            axis: v3(r)?,
            radius: f(r)?,
        }),
        4 => Some(Primitive::Cone {
            apex: v3(r)?,
            axis: v3(r)?,
            half_angle: f(r)?,
        }),
        5 => Some(Primitive::Torus {
            center: v3(r)?,
            axis: v3(r)?,
            major: f(r)?,
            minor: f(r)?,
        }),
        t => return Err(MeshError::Decode(format!("bad primitive tag {t}"))),
    };
    Ok((
        id,
        FaceGroupInfo {
            name,
            origin,
            primitive,
        },
    ))
}

// ---- byte helpers -----------------------------------------------------------------------

fn put_vec3(b: &mut Vec<u8>, p: Point3) {
    for c in [p.x, p.y, p.z] {
        b.extend_from_slice(&c.to_bits().to_le_bytes());
    }
}

fn get_vec3(b: &[u8]) -> Point3 {
    Point3::new(
        f64::from_bits(le_u64(&b[0..8])),
        f64::from_bits(le_u64(&b[8..16])),
        f64::from_bits(le_u64(&b[16..24])),
    )
}

fn le_u32(b: &[u8]) -> u32 {
    let mut a = [0u8; 4];
    a.copy_from_slice(&b[..4]);
    u32::from_le_bytes(a)
}

fn le_u64(b: &[u8]) -> u64 {
    let mut a = [0u8; 8];
    a.copy_from_slice(&b[..8]);
    u64::from_le_bytes(a)
}

fn count(n: u64) -> MeshResult<usize> {
    usize::try_from(n)
        .ok()
        .filter(|&c| c < u32::MAX as usize)
        .ok_or_else(|| MeshError::Decode(format!("element count {n} exceeds the u32 range")))
}

fn eof(e: std::io::Error) -> MeshError {
    MeshError::Decode(format!("truncated or unreadable data: {e}"))
}

/// Feeds a byte stream into [`ContentHasher`] independently of how it is chunked.
struct ByteHasher {
    h: ContentHasher,
    pending: [u8; 8],
    n: usize,
}

impl ByteHasher {
    fn new() -> Self {
        Self {
            h: ContentHasher::new(),
            pending: [0; 8],
            n: 0,
        }
    }

    fn update(&mut self, mut b: &[u8]) {
        if self.n > 0 {
            let k = (8 - self.n).min(b.len());
            self.pending[self.n..self.n + k].copy_from_slice(&b[..k]);
            self.n += k;
            b = &b[k..];
            if self.n < 8 {
                return;
            }
            self.h.word(u64::from_le_bytes(self.pending));
            self.n = 0;
        }
        let mut chunks = b.chunks_exact(8);
        for c in &mut chunks {
            self.h.word(le_u64(c));
        }
        let rest = chunks.remainder();
        self.pending[..rest.len()].copy_from_slice(rest);
        self.n = rest.len();
    }

    fn finish(&self) -> u64 {
        let mut h = self.h.clone();
        if self.n > 0 {
            let mut a = [0u8; 8];
            a[..self.n].copy_from_slice(&self.pending[..self.n]);
            h.word(u64::from_le_bytes(a));
        }
        h.word(self.n as u64);
        h.finish()
    }
}

struct HashWriter<W: Write> {
    inner: W,
    hasher: ByteHasher,
    buf: Vec<u8>,
}

impl<W: Write> HashWriter<W> {
    fn new(inner: W) -> Self {
        Self {
            inner,
            hasher: ByteHasher::new(),
            buf: Vec::with_capacity(CHUNK * 24),
        }
    }

    fn put(&mut self, b: &[u8]) -> std::io::Result<()> {
        self.hasher.update(b);
        self.inner.write_all(b)
    }

    fn put_items<T>(&mut self, items: &[T], enc: impl Fn(&mut Vec<u8>, &T)) -> std::io::Result<()> {
        for chunk in items.chunks(CHUNK) {
            let mut buf = std::mem::take(&mut self.buf);
            buf.clear();
            for it in chunk {
                enc(&mut buf, it);
            }
            let r = self.put(&buf);
            self.buf = buf;
            r?;
        }
        Ok(())
    }

    fn hasher_finish(&self) -> u64 {
        self.hasher.finish()
    }
}

struct HashReader<R: Read> {
    inner: R,
    hasher: ByteHasher,
}

impl<R: Read> HashReader<R> {
    fn new(inner: R) -> Self {
        Self {
            inner,
            hasher: ByteHasher::new(),
        }
    }

    fn take(&mut self, n: usize) -> MeshResult<Vec<u8>> {
        let mut b = vec![0u8; n];
        self.inner.read_exact(&mut b).map_err(eof)?;
        self.hasher.update(&b);
        Ok(b)
    }

    fn byte(&mut self) -> MeshResult<u8> {
        Ok(self.take(1)?[0])
    }

    fn items<T>(&mut self, n: usize, size: usize, dec: impl Fn(&[u8]) -> T) -> MeshResult<Vec<T>> {
        let mut out = Vec::with_capacity(n.min(CHUNK));
        let mut buf = vec![0u8; CHUNK.min(n.max(1)) * size];
        let mut left = n;
        while left > 0 {
            let k = left.min(CHUNK);
            let b = &mut buf[..k * size];
            self.inner.read_exact(b).map_err(eof)?;
            self.hasher.update(b);
            out.extend(b.chunks_exact(size).map(&dec));
            left -= k;
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FaceId;

    fn sample() -> IndexedMesh {
        let mut m = IndexedMesh::new(
            vec![
                Point3::new(0.1, -0.0, f64::from_bits(0x7ff8_0000_0000_1234)),
                Point3::X,
                Point3::Y,
                Point3::new(1e-300, 5.0, -7.25),
            ],
            vec![[0, 1, 2], [1, 3, 2]],
        )
        .unwrap();
        m.attrs.vertex_normals = Some(vec![Vec3::Z; 4]);
        m.attrs.vertex_colors = Some(vec![Rgba8([1, 2, 3, 4]); 4]);
        m.attrs.corner_uvs = Some(vec![[Point2::new(0.5, 0.25); 3]; 2]);
        m.attrs.tri_properties = Some(vec![
            TriProperty {
                group: 7,
                index: [1, 2, 3]
            };
            2
        ]);
        let g = m.groups.alloc(FaceGroupInfo {
            name: Some("Bottom ✓".into()),
            origin: GroupOrigin::Boolean {
                operand: 1,
                source: FaceGroupId(3),
            },
            primitive: Some(Primitive::Torus {
                center: Point3::ONE,
                axis: Vec3::Z,
                major: 5.0,
                minor: 1.0,
            }),
        });
        m.groups.set(FaceId(1), g);
        for o in [
            GroupOrigin::Import { object: 2 },
            GroupOrigin::Segmentation,
            GroupOrigin::Cap,
            GroupOrigin::Fill,
            GroupOrigin::Shell {
                side: ShellSide::Both,
            },
        ] {
            m.groups.alloc(FaceGroupInfo::new(o));
        }
        m
    }

    fn bits(m: &IndexedMesh) -> Vec<u64> {
        m.positions
            .iter()
            .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
            .collect()
    }

    #[test]
    fn round_trip_is_bit_exact() {
        let m = sample();
        let body = MeshBody::new(m.clone());
        let mut buf = Vec::new();
        body.write_binary(&mut buf).unwrap();
        let back = MeshBody::read_binary(buf.as_slice()).unwrap();
        assert_eq!(bits(back.mesh()), bits(&m));
        assert_eq!(back.mesh().tris, m.tris);
        assert_eq!(back.mesh().groups, m.groups);
        assert_eq!(back.mesh().attrs.corner_uvs, m.attrs.corner_uvs);
        assert_eq!(back.mesh().content_hash(), m.content_hash());
        let mut buf2 = Vec::new();
        back.write_binary(&mut buf2).unwrap();
        assert_eq!(buf, buf2);
    }

    #[test]
    fn rejects_corruption() {
        let mut buf = Vec::new();
        MeshBody::new(sample()).write_binary(&mut buf).unwrap();
        for i in [0usize, 5, 40, buf.len() / 2, buf.len() - 1] {
            let mut b = buf.clone();
            b[i] ^= 0x10;
            assert!(
                matches!(
                    MeshBody::read_binary(b.as_slice()),
                    Err(MeshError::Decode(_))
                ),
                "flip at {i} not detected"
            );
        }
        for n in [0, 3, 27, buf.len() - 9] {
            assert!(matches!(
                MeshBody::read_binary(&buf[..n]),
                Err(MeshError::Decode(_))
            ));
        }
        // A huge declared count fails on EOF, not on allocation.
        let mut b = buf.clone();
        b[12..20].copy_from_slice(&(u64::from(u32::MAX) - 1).to_le_bytes());
        assert!(matches!(
            MeshBody::read_binary(b.as_slice()),
            Err(MeshError::Decode(_))
        ));
    }

    #[test]
    fn empty_mesh_round_trips() {
        let mut buf = Vec::new();
        MeshBody::new(IndexedMesh::default())
            .write_binary(&mut buf)
            .unwrap();
        let back = MeshBody::read_binary(buf.as_slice()).unwrap();
        assert_eq!(**back.mesh(), IndexedMesh::default());
    }
}
