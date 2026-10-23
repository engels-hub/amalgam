//! Sorted undirected edge table shared by the repair steps.

use rayon::prelude::*;

use crate::error::{MeshError, MeshResult};

/// All edges of a triangle list, sorted by `(lo, hi)`; the faces sharing an edge are one group.
///
/// A corner is `3 * face + k` and stands for the directed edge `tris[face][k] ->
/// tris[face][(k + 1) % 3]`. Zero-length edges (repeated vertex index) are left out.
pub(crate) struct EdgeTable {
    /// `(lo << 32 | hi, corner)`, sorted.
    pub entries: Vec<(u64, u32)>,
    /// Start of every group in `entries`, plus a final end marker.
    pub starts: Vec<u32>,
}

impl EdgeTable {
    pub(crate) fn build(tris: &[[u32; 3]]) -> MeshResult<EdgeTable> {
        if tris
            .len()
            .checked_mul(3)
            .is_none_or(|n| n >= u32::MAX as usize)
        {
            return Err(MeshError::TooLarge { count: tris.len() });
        }
        let mut entries: Vec<(u64, u32)> = tris
            .par_iter()
            .enumerate()
            .flat_map_iter(|(f, t)| {
                (0..3usize).filter_map(move |k| {
                    let (a, b) = (t[k], t[(k + 1) % 3]);
                    (a != b).then(|| {
                        let (lo, hi) = (a.min(b), a.max(b));
                        ((u64::from(lo) << 32) | u64::from(hi), (3 * f + k) as u32)
                    })
                })
            })
            .collect();
        entries.par_sort_unstable();
        let mut starts = Vec::new();
        for (i, e) in entries.iter().enumerate() {
            if i == 0 || entries[i - 1].0 != e.0 {
                starts.push(i as u32);
            }
        }
        starts.push(entries.len() as u32);
        Ok(EdgeTable { entries, starts })
    }

    /// Number of edges.
    pub(crate) fn len(&self) -> usize {
        self.starts.len() - 1
    }

    /// The corners of edge `g`.
    pub(crate) fn group(&self, g: usize) -> &[(u64, u32)] {
        &self.entries[self.starts[g] as usize..self.starts[g + 1] as usize]
    }

    /// Edge endpoints `(lo, hi)` of group `g`.
    pub(crate) fn ends(&self, g: usize) -> (u32, u32) {
        let k = self.entries[self.starts[g] as usize].0;
        ((k >> 32) as u32, k as u32)
    }
}
