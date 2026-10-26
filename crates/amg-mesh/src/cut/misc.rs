//! Separate shells, erase & fill, reverse normals (MESH-09).

use std::collections::HashSet;

use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::error::{MeshError, MeshResult};
use crate::groups::FaceGroups;
use crate::holefill::fill_holes_where;
use crate::{
    FaceId, HalfEdgeMesh, HoleFillOpts, IndexedMesh, MeshAttrs, NonManifoldPolicy, TriProperty,
};

fn find(parent: &mut [u32], mut v: u32) -> u32 {
    while parent[v as usize] != v {
        let up = parent[parent[v as usize] as usize];
        parent[v as usize] = up;
        v = up;
    }
    v
}

/// Splits `m` into its shells (connected components over shared vertex indices, the same
/// definition as [`crate::analyze`]: faces with a repeated vertex index join no shell and are
/// dropped). Ordered by descending area, ties by the shell's smallest vertex index. Each shell
/// keeps its vertices in input order, its attributes and the full face-group table.
pub fn separate_shells(m: &IndexedMesh) -> Vec<IndexedMesh> {
    let nv = m.positions.len();
    let mut parent: Vec<u32> = (0..nv as u32).collect();
    let mut referenced = vec![false; nv];
    let degenerate = |t: &[u32; 3]| t[0] == t[1] || t[1] == t[2] || t[0] == t[2];
    for t in m.tris.iter().filter(|t| !degenerate(t)) {
        for &v in t {
            referenced[v as usize] = true;
        }
        let (a, b) = (find(&mut parent, t[0]), find(&mut parent, t[1]));
        if a != b {
            parent[a.max(b) as usize] = a.min(b);
        }
        let (a, c) = (find(&mut parent, t[0]), find(&mut parent, t[2]));
        if a != c {
            parent[a.max(c) as usize] = a.min(c);
        }
    }
    // Shells are numbered by their smallest vertex (the root, since roots are minima).
    let mut shell_of_root = vec![u32::MAX; nv];
    let mut count = 0u32;
    for v in 0..nv {
        if referenced[v] && find(&mut parent, v as u32) == v as u32 {
            shell_of_root[v] = count;
            count += 1;
        }
    }
    if count == 0 {
        return Vec::new();
    }
    let shell_of = |parent: &mut [u32], v: u32| shell_of_root[find(parent, v) as usize];
    let mut shells: Vec<IndexedMesh> = (0..count)
        .map(|_| IndexedMesh {
            groups: FaceGroups::from_raw(Vec::new(), m.groups.raw().1.clone(), m.groups.next_id()),
            ..IndexedMesh::default()
        })
        .collect();
    let a = &m.attrs;
    for s in &mut shells {
        s.attrs = MeshAttrs {
            vertex_normals: a.vertex_normals.as_ref().map(|_| Vec::new()),
            vertex_colors: a.vertex_colors.as_ref().map(|_| Vec::new()),
            corner_uvs: a.corner_uvs.as_ref().map(|_| Vec::new()),
            tri_properties: a.tri_properties.as_ref().map(|_| Vec::new()),
        };
    }
    let mut local = vec![u32::MAX; nv];
    for v in 0..nv {
        if !referenced[v] {
            continue;
        }
        let s = &mut shells[shell_of(&mut parent, v as u32) as usize];
        local[v] = s.positions.len() as u32;
        s.positions.push(m.positions[v]);
        if let (Some(dst), Some(src)) = (&mut s.attrs.vertex_normals, &a.vertex_normals) {
            dst.push(src[v]);
        }
        if let (Some(dst), Some(src)) = (&mut s.attrs.vertex_colors, &a.vertex_colors) {
            dst.push(src[v]);
        }
    }
    let src_groups = m.groups.per_face();
    let mut group_ids: Vec<Vec<crate::FaceGroupId>> = vec![Vec::new(); count as usize];
    for (f, t) in m.tris.iter().enumerate() {
        if degenerate(t) {
            continue;
        }
        let si = shell_of(&mut parent, t[0]) as usize;
        let s = &mut shells[si];
        s.tris.push(t.map(|v| local[v as usize]));
        group_ids[si].push(src_groups[f]);
        if let (Some(dst), Some(src)) = (&mut s.attrs.corner_uvs, &a.corner_uvs) {
            dst.push(src[f]);
        }
        if let (Some(dst), Some(src)) = (&mut s.attrs.tri_properties, &a.tri_properties) {
            dst.push(src[f]);
        }
    }
    for (s, ids) in shells.iter_mut().zip(group_ids) {
        let (_, info, next) = m.groups.raw();
        s.groups = FaceGroups::from_raw(ids, info.clone(), next);
    }
    let mut order: Vec<(f64, usize)> = shells
        .par_iter()
        .enumerate()
        .map(|(i, s)| (s.area(), i))
        .collect();
    order.sort_by(|x, y| y.0.total_cmp(&x.0).then(x.1.cmp(&y.1)));
    let mut slots: Vec<Option<IndexedMesh>> = shells.into_iter().map(Some).collect();
    order
        .into_iter()
        .filter_map(|(_, i)| slots[i].take())
        .collect()
}

/// Deletes `faces` and fills the hole(s) this opens with [`crate::fill_holes`] (MESH-20).
/// Only boundary loops that touch the erased region are filled; holes the mesh already had
/// stay as they are. The patch is one new `Fill` face group. Vertices left without a face are
/// dropped. A closed input gives a watertight output.
///
/// # Errors
/// [`MeshError::InvalidOption`] for a face id out of range, [`MeshError::NotClosed`] when a
/// new loop could not be filled (longer than `opts.max_boundary_edges`, or refused by the
/// half-edge structure), [`MeshError::Cancelled`].
pub fn erase_and_fill(
    m: &IndexedMesh,
    faces: &[FaceId],
    opts: &HoleFillOpts,
    ctx: &JobCtx,
) -> MeshResult<IndexedMesh> {
    let nf = m.tris.len();
    let mut erase = vec![false; nf];
    for f in faces {
        match erase.get_mut(f.0 as usize) {
            Some(e) => *e = true,
            None => {
                return Err(MeshError::InvalidOption {
                    name: "faces",
                    reason: format!("face {} out of range (mesh has {nf} faces)", f.0),
                });
            }
        }
    }
    if !erase.contains(&true) {
        return Ok(m.clone());
    }
    ctx.checkpoint()?;
    // Directed boundary edges the mesh had before (an edge whose reverse is absent).
    let directed: HashSet<(u32, u32)> = m
        .tris
        .iter()
        .flat_map(|t| (0..3).map(move |k| (t[k], t[(k + 1) % 3])))
        .collect();
    let old_boundary: HashSet<(u32, u32)> = directed
        .iter()
        .filter(|&&(a, b)| !directed.contains(&(b, a)))
        .copied()
        .collect();

    let keep: Vec<bool> = erase.iter().map(|e| !e).collect();
    let mut rest = m.clone();
    rest.attrs.retain(&vec![true; rest.positions.len()], &keep);
    rest.groups.retain(&keep);
    let mut i = 0;
    rest.tris.retain(|_| {
        i += 1;
        keep[i - 1]
    });
    let remap = rest.compact();
    let old_boundary: HashSet<(u32, u32)> = old_boundary
        .into_iter()
        .filter_map(|(a, b)| {
            let (a, b) = (remap.vertices[a as usize], remap.vertices[b as usize]);
            (a != crate::INVALID && b != crate::INVALID).then_some((a, b))
        })
        .collect();
    if rest.tris.is_empty() {
        return Ok(rest);
    }
    ctx.progress(0.2);

    let (mut he, _) = HalfEdgeMesh::from_indexed(&rest, NonManifoldPolicy::SplitVertices)?;
    let pick = |h: &HalfEdgeMesh, lp: &[crate::HalfEdgeId]| {
        lp.iter()
            .any(|&e| !old_boundary.contains(&(h.origin(e).0, h.target(e).0)))
    };
    let report = fill_holes_where(&mut he, opts, &ctx.sub(0.2, 1.0), &pick)?;
    let unfilled = report.skipped_too_large + report.failed;
    if unfilled > 0 {
        return Err(MeshError::NotClosed {
            boundary_loops: unfilled,
        });
    }
    Ok(he.to_indexed())
}

/// Flips the winding of `faces` (all faces for `None`; out-of-range ids are ignored), keeping
/// corner UVs and property indices attached to their corners. Vertex normals are negated on
/// the vertices all of whose faces were flipped; a vertex shared with an unflipped face keeps
/// its normal (recompute normals after a partial flip). A full flip negates
/// [`IndexedMesh::signed_volume`].
pub fn reverse_normals(m: &mut IndexedMesh, faces: Option<&[FaceId]>) {
    let nf = m.tris.len();
    let flip: Vec<bool> = match faces {
        None => vec![true; nf],
        Some(list) => {
            let mut f = vec![false; nf];
            for id in list {
                if let Some(x) = f.get_mut(id.0 as usize) {
                    *x = true;
                }
            }
            f
        }
    };
    m.tris
        .par_iter_mut()
        .zip(flip.par_iter())
        .for_each(|(t, &f)| {
            if f {
                t.swap(1, 2);
            }
        });
    if let Some(uv) = &mut m.attrs.corner_uvs {
        uv.par_iter_mut().zip(flip.par_iter()).for_each(|(c, &f)| {
            if f {
                c.swap(1, 2);
            }
        });
    }
    if let Some(tp) = &mut m.attrs.tri_properties {
        tp.par_iter_mut()
            .zip(flip.par_iter())
            .for_each(|(p, &f): (&mut TriProperty, &bool)| {
                if f {
                    p.index.swap(1, 2);
                }
            });
    }
    if let Some(ns) = &mut m.attrs.vertex_normals {
        let mut total = vec![0u32; ns.len()];
        let mut flipped = vec![0u32; ns.len()];
        for (t, &f) in m.tris.iter().zip(&flip) {
            for &v in t {
                total[v as usize] += 1;
                flipped[v as usize] += u32::from(f);
            }
        }
        for (v, n) in ns.iter_mut().enumerate() {
            if total[v] > 0 && total[v] == flipped[v] {
                *n = -*n;
            }
        }
    }
}
