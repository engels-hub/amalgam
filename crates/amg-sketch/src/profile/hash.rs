//! Platform- and version-independent 64-bit hashing of edge keys (FNV-1a over a fixed
//! little-endian encoding). `std`'s `DefaultHasher` is not stable across releases, so it
//! cannot back persistent ids.

use super::VertexKey;
use crate::ids::EntityId;
use crate::model::End;

/// `(source, start, end, forward)`: a boundary edge as traversed by its loop. `forward` is
/// true when the loop runs along the source curve's own direction.
pub(crate) type EdgeKey = (EntityId, VertexKey, VertexKey, bool);

const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const PRIME: u64 = 0x0000_0100_0000_01b3;

pub(crate) struct Fnv(u64);

impl Fnv {
    pub fn new() -> Fnv {
        Fnv(OFFSET)
    }
    fn bytes(&mut self, b: &[u8]) {
        for &x in b {
            self.0 ^= u64::from(x);
            self.0 = self.0.wrapping_mul(PRIME);
        }
    }
    fn u32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }
    fn vertex(&mut self, k: &VertexKey) {
        match *k {
            VertexKey::Endpoint(e, end) => {
                self.bytes(&[0, matches!(end, End::End).into()]);
                self.u32(e.0);
            }
            VertexKey::Intersection { a, b, ordinal } => {
                self.bytes(&[1]);
                self.u32(a.0);
                self.u32(b.0);
                self.bytes(&ordinal.to_le_bytes());
            }
            VertexKey::Seam(e) => {
                self.bytes(&[2]);
                self.u32(e.0);
            }
        }
    }
    pub fn edge(&mut self, k: &EdgeKey) {
        self.u32(k.0.0);
        self.vertex(&k.1);
        self.vertex(&k.2);
        self.bytes(&[k.3.into()]);
    }
    pub fn finish(&self) -> u64 {
        self.0
    }
}
