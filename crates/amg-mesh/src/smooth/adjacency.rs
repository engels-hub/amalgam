//! Vertex→face and vertex→vertex adjacency in CSR form, plus boundary detection.

use rayon::prelude::*;

use crate::IndexedMesh;

/// CSR adjacency of an indexed mesh.
pub(super) struct Adjacency {
    vf_off: Vec<u32>,
    vf: Vec<u32>,
    vv_off: Vec<u32>,
    vv: Vec<u32>,
    /// Vertex has an edge used by one face, or by more than two.
    pub boundary: Vec<bool>,
}

impl Adjacency {
    pub fn build(m: &IndexedMesh) -> Adjacency {
        let nv = m.positions.len();
        // Vertex -> faces (counting sort, faces ascending per vertex).
        let mut vf_off = vec![0u32; nv + 1];
        for t in &m.tris {
            for &v in t {
                vf_off[v as usize + 1] += 1;
            }
        }
        for i in 0..nv {
            vf_off[i + 1] += vf_off[i];
        }
        let mut cursor = vf_off.clone();
        let mut vf = vec![0u32; m.tris.len() * 3];
        for (f, t) in m.tris.iter().enumerate() {
            for &v in t {
                let c = &mut cursor[v as usize];
                vf[*c as usize] = f as u32;
                *c += 1;
            }
        }
        // Vertex -> the two other corners of each incident face, then sort + dedup per vertex.
        let mut raw: Vec<u32> = Vec::with_capacity(vf.len() * 2);
        for v in 0..nv {
            for &f in &vf[vf_off[v] as usize..vf_off[v + 1] as usize] {
                for &w in &m.tris[f as usize] {
                    if w as usize != v {
                        raw.push(w);
                    }
                }
            }
        }
        let mut slices: Vec<&mut [u32]> = Vec::with_capacity(nv);
        let mut rest = raw.as_mut_slice();
        for v in 0..nv {
            let len = 2 * (vf_off[v + 1] - vf_off[v]) as usize;
            let (head, tail) = rest.split_at_mut(len);
            slices.push(head);
            rest = tail;
        }
        let info: Vec<(u32, bool)> = slices
            .par_iter_mut()
            .map(|s| {
                s.sort_unstable();
                let mut uniq = 0usize;
                let mut boundary = false;
                let mut i = 0;
                while i < s.len() {
                    let mut j = i;
                    while j < s.len() && s[j] == s[i] {
                        j += 1;
                    }
                    boundary |= j - i != 2;
                    s[uniq] = s[i];
                    uniq += 1;
                    i = j;
                }
                (uniq as u32, boundary)
            })
            .collect();
        let mut vv_off = vec![0u32; nv + 1];
        for v in 0..nv {
            vv_off[v + 1] = vv_off[v] + info[v].0;
        }
        let mut vv = Vec::with_capacity(vv_off[nv] as usize);
        for (s, i) in slices.iter().zip(&info) {
            vv.extend_from_slice(&s[..i.0 as usize]);
        }
        let boundary = info.iter().map(|i| i.1).collect();
        Adjacency {
            vf_off,
            vf,
            vv_off,
            vv,
            boundary,
        }
    }

    #[inline]
    pub fn faces_of(&self, v: usize) -> &[u32] {
        &self.vf[self.vf_off[v] as usize..self.vf_off[v + 1] as usize]
    }

    #[inline]
    pub fn neighbors_of(&self, v: usize) -> &[u32] {
        &self.vv[self.vv_off[v] as usize..self.vv_off[v + 1] as usize]
    }
}
