//! Fast, fixed-seed 64-bit content hash (stable across runs and builds; not cryptographic).

/// Streaming hasher over 64-bit words: four independent multiply-rotate lanes, avalanche at
/// the end. Same input words → same hash on every platform.
#[derive(Debug, Clone)]
pub(crate) struct ContentHasher {
    lanes: [u64; 4],
    i: usize,
    count: u64,
}

const P1: u64 = 0x9e37_79b1_85eb_ca87;
const P2: u64 = 0xc2b2_ae3d_27d4_eb4f;

impl ContentHasher {
    pub(crate) fn new() -> Self {
        Self {
            lanes: [
                0x243f_6a88_85a3_08d3,
                0x1319_8a2e_0370_7344,
                0xa409_3822_299f_31d0,
                0x082e_fa98_ec4e_6c89,
            ],
            i: 0,
            count: 0,
        }
    }

    #[inline]
    pub(crate) fn word(&mut self, w: u64) {
        let l = &mut self.lanes[self.i];
        *l = (*l ^ w.wrapping_mul(P2)).rotate_left(31).wrapping_mul(P1);
        self.i = (self.i + 1) & 3;
        self.count += 1;
    }

    pub(crate) fn f64(&mut self, v: f64) {
        self.word(v.to_bits());
    }

    pub(crate) fn u32x2(&mut self, a: u32, b: u32) {
        self.word(u64::from(a) | (u64::from(b) << 32));
    }

    pub(crate) fn bytes(&mut self, b: &[u8]) {
        self.word(b.len() as u64);
        let mut chunks = b.chunks_exact(8);
        for c in &mut chunks {
            let mut a = [0u8; 8];
            a.copy_from_slice(c);
            self.word(u64::from_le_bytes(a));
        }
        let rest = chunks.remainder();
        if !rest.is_empty() {
            let mut a = [0u8; 8];
            a[..rest.len()].copy_from_slice(rest);
            self.word(u64::from_le_bytes(a));
        }
    }

    pub(crate) fn finish(&self) -> u64 {
        let mut h = self.count.wrapping_mul(P1);
        for (k, l) in self.lanes.iter().enumerate() {
            h ^= l.rotate_left(17 * k as u32 + 7).wrapping_mul(P2);
            h = h.rotate_left(27).wrapping_mul(P1);
        }
        h ^= h >> 33;
        h = h.wrapping_mul(0xff51_afd7_ed55_8ccd);
        h ^= h >> 33;
        h
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn order_and_content_sensitive() {
        let mut a = ContentHasher::new();
        a.word(1);
        a.word(2);
        let mut b = ContentHasher::new();
        b.word(2);
        b.word(1);
        assert_ne!(a.finish(), b.finish());
        let mut c = ContentHasher::new();
        c.word(1);
        c.word(2);
        assert_eq!(a.finish(), c.finish());
        let mut d = ContentHasher::new();
        d.bytes(b"abc");
        let mut e = ContentHasher::new();
        e.bytes(b"abd");
        assert_ne!(d.finish(), e.finish());
    }
}
