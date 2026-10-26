//! Manifold backend: conversion `IndexedMesh` ⇄ `MeshGL64` and group mapping.
//!
//! Group transport: an operand whose faces all share one group is passed as plain xyz; its
//! output faces are recognised by their run's original id. An operand with several groups
//! gets a fourth vertex property, the dense group index. A vertex shared by faces of different
//! groups is duplicated once per extra group (bit-identical position), and the merge vectors
//! (`mergeFromVert` / `mergeToVert`) tell Manifold the copies are one topological vertex.
//! Every output triangle's corners are interpolated inside a single source triangle, so its
//! group property is that triangle's index (rounded, to absorb barycentric rounding).

use std::collections::HashMap;
use std::time::Instant;

use amg_core::Point3;
use amg_jobs::JobCtx;
use amg_manifold_sys::{Manifold, MeshData, MeshGL64, OpType, Status};
use rayon::prelude::*;

use super::{BoolOp, BoolOpts, BoolReport};
use crate::analysis::{AnalyzeOpts, analyze};
use crate::error::{MeshError, MeshResult};
use crate::groups::{DEFAULT_GROUP, FaceGroupId, FaceGroupInfo, FaceGroups, GroupOrigin};
use crate::repair::{RepairOpts, repair};
use crate::{INVALID, IndexedMesh, MeshAttrs};

/// One converted operand.
struct Operand {
    /// Distinct group ids of its faces, ascending (the dense group index space).
    groups: Vec<FaceGroupId>,
    /// Table entry of each of `groups`.
    infos: Vec<Option<FaceGroupInfo>>,
    /// `runOriginalID` given to Manifold; output runs carry it.
    original_id: u32,
    manifold: Manifold,
}

pub(super) fn run(
    op: BoolOp,
    inputs: &[&IndexedMesh],
    opts: &BoolOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, BoolReport)> {
    let started = Instant::now();
    let mut report = BoolReport::default();
    // Operands convert in parallel: much of Manifold's mesh import (half-edge build, normals,
    // coplanar faces) is serial per mesh. Results come back in input order. Operand k's
    // `runOriginalID` is `first_id + k`.
    let first_id = amg_manifold_sys::reserve_ids(inputs.len() as u32);
    let converted: Vec<MeshResult<(Operand, Option<IndexedMesh>)>> = inputs
        .par_iter()
        .enumerate()
        .map(|(k, m)| {
            ctx.checkpoint()?;
            convert_checked(m, first_id + k as u32, opts, ctx)
        })
        .collect();
    let mut operands = Vec::with_capacity(inputs.len());
    // The group table of operand 0 after an optional repair.
    let mut base_groups = None;
    for (k, c) in converted.into_iter().enumerate() {
        let (operand, repaired) = c?;
        if let Some(r) = &repaired {
            report.repaired_inputs.push(k);
            if k == 0 {
                base_groups = Some(r.groups.clone());
            }
        }
        operands.push(operand);
    }
    ctx.progress(0.3);
    ctx.checkpoint()?;
    let op = match op {
        BoolOp::Union => OpType::Add,
        BoolOp::Difference => OpType::Subtract,
        BoolOp::Intersection => OpType::Intersect,
    };
    let result = match operands.as_slice() {
        [a] => a.manifold.boolean(&Manifold::empty(), OpType::Add),
        [a, b] => a.manifold.boolean(&b.manifold, op),
        _ => {
            let ms: Vec<&Manifold> = operands.iter().map(|o| &o.manifold).collect();
            Manifold::batch_boolean(&ms, op)
        }
    }
    .map_err(boolean_error)?;
    ctx.checkpoint()?;
    ctx.progress(0.8);
    let data = result.to_mesh().to_data();
    drop(result);
    ctx.checkpoint()?;
    let base = base_groups.as_ref().unwrap_or(&inputs[0].groups);
    let mesh = from_meshgl(&data, &operands, base)?;
    ctx.progress(1.0);
    report.millis = u32::try_from(started.elapsed().as_millis()).unwrap_or(u32::MAX);
    Ok((mesh, report))
}

fn boolean_error(s: Status) -> MeshError {
    MeshError::Boolean {
        status: s.to_string(),
    }
}

/// Converts `m`; when Manifold rejects it, repairs it (`repair_inputs`) or explains why.
/// Returns the repaired mesh when a repair happened.
fn convert_checked(
    m: &IndexedMesh,
    original_id: u32,
    opts: &BoolOpts,
    ctx: &JobCtx,
) -> MeshResult<(Operand, Option<IndexedMesh>)> {
    let status = match convert(m, original_id) {
        Ok(o) => return Ok((o, None)),
        Err(s) => s,
    };
    if !matches!(status, Status::NotManifold) {
        return Err(boolean_error(status));
    }
    if opts.repair_inputs {
        let (fixed, _) = repair(m, &RepairOpts::default(), ctx)?;
        return match convert(&fixed, original_id) {
            Ok(o) => Ok((o, Some(fixed))),
            Err(_) => Err(not_manifold_error(&fixed, ctx)?),
        };
    }
    Err(not_manifold_error(m, ctx)?)
}

/// The most specific error for a mesh Manifold calls not manifold.
fn not_manifold_error(m: &IndexedMesh, ctx: &JobCtx) -> MeshResult<MeshError> {
    let a = analyze(m, &AnalyzeOpts::default(), ctx)?;
    Ok(
        if !a.non_manifold_edges.is_empty() || !a.non_manifold_vertices.is_empty() {
            MeshError::NonManifold {
                edges: a.non_manifold_edges.len(),
                vertices: a.non_manifold_vertices.len(),
            }
        } else if !a.boundary_loops.is_empty() || a.boundary_edges > 0 {
            MeshError::NotClosed {
                boundary_loops: a.boundary_loops.len().max(1),
            }
        } else {
            MeshError::Boolean {
                status: format!("{} (inconsistent orientation?)", Status::NotManifold),
            }
        },
    )
}

fn convert(m: &IndexedMesh, original_id: u32) -> Result<Operand, Status> {
    let per_face = m.groups.per_face();
    let first = per_face.first().copied();
    let groups: Vec<FaceGroupId> = if per_face.par_iter().all(|g| Some(*g) == first) {
        first.into_iter().collect()
    } else {
        let mut g = per_face.to_vec();
        g.par_sort_unstable();
        g.dedup();
        g
    };
    let mesh = if groups.len() <= 1 {
        let props: Vec<f64> = m
            .positions
            .par_iter()
            .flat_map_iter(|p| p.to_array())
            .collect();
        let tris: Vec<u64> = m
            .tris
            .par_iter()
            .flat_map_iter(|t| t.map(u64::from))
            .collect();
        MeshGL64::with_options(&props, 3, &tris, &[], &[], original_id)
    } else {
        let s = split_by_group(m, &groups);
        MeshGL64::with_options(
            &s.props,
            4,
            &s.tris,
            &s.merge_from,
            &s.merge_to,
            original_id,
        )
    };
    let manifold = Manifold::from_mesh(&mesh)?;
    let infos = groups.iter().map(|&g| m.groups.info(g).cloned()).collect();
    Ok(Operand {
        groups,
        infos,
        original_id,
        manifold,
    })
}

/// A mesh with one vertex copy per (vertex, group) pair.
struct GroupSplit {
    /// `[x, y, z, group index]` per property vertex.
    props: Vec<f64>,
    tris: Vec<u64>,
    /// Extra copy → original vertex.
    merge_from: Vec<u64>,
    merge_to: Vec<u64>,
}

fn split_by_group(m: &IndexedMesh, groups: &[FaceGroupId]) -> GroupSplit {
    let per_face = m.groups.per_face();
    let nv = m.positions.len();
    let mut first: Vec<u32> = vec![INVALID; nv];
    let mut extra: HashMap<(u32, u32), u32> = HashMap::new();
    // (source vertex, group) of each extra copy, in creation order.
    let mut extra_src: Vec<(u32, u32)> = Vec::new();
    let mut tris = Vec::with_capacity(m.tris.len() * 3);
    for (t, g) in m.tris.iter().zip(per_face) {
        let gi = groups.binary_search(g).unwrap_or(0) as u32;
        for &v in t {
            let slot = &mut first[v as usize];
            let idx = if *slot == INVALID || *slot == gi {
                *slot = gi;
                v
            } else {
                *extra.entry((v, gi)).or_insert_with(|| {
                    extra_src.push((v, gi));
                    (nv + extra_src.len() - 1) as u32
                })
            };
            tris.push(u64::from(idx));
        }
    }
    let mut props = Vec::with_capacity((nv + extra_src.len()) * 4);
    let push = |props: &mut Vec<f64>, p: Point3, g: u32| {
        props.extend([p.x, p.y, p.z, f64::from(g)]);
    };
    for (p, &g) in m.positions.iter().zip(&first) {
        push(&mut props, *p, if g == INVALID { 0 } else { g });
    }
    for &(v, g) in &extra_src {
        push(&mut props, m.positions[v as usize], g);
    }
    GroupSplit {
        props,
        tris,
        merge_from: (nv..nv + extra_src.len()).map(|i| i as u64).collect(),
        merge_to: extra_src.iter().map(|&(v, _)| u64::from(v)).collect(),
    }
}

/// Builds the result mesh: collapses Manifold's property vertices back to topological ones
/// and maps every output triangle to its output group.
fn from_meshgl(d: &MeshData, operands: &[Operand], base: &FaceGroups) -> MeshResult<IndexedMesh> {
    let np = d.n_props.max(3);
    let n_prop_verts = d.vert_props.len() / np;
    let n_tris = d.tri_verts.len() / 3;
    if n_prop_verts > u32::MAX as usize || n_tris > u32::MAX as usize {
        return Err(MeshError::TooLarge {
            count: n_prop_verts.max(n_tris),
        });
    }
    // Output runs, put in operand order: Manifold orders runs by its internal mesh ids, which
    // the parallel import hands out in whatever order the threads get there.
    let mut runs: Vec<(usize, usize, usize)> = Vec::with_capacity(d.run_original_id.len());
    for (r, &orig) in d.run_original_id.iter().enumerate() {
        let k = operands
            .iter()
            .position(|o| o.original_id == orig)
            .ok_or_else(|| MeshError::Boolean {
                status: format!("output run has unknown original id {orig}"),
            })?;
        let hi = ((d.run_index[r + 1] / 3) as usize).min(n_tris);
        let lo = ((d.run_index[r] / 3) as usize).min(hi);
        runs.push((k, lo, hi));
    }
    runs.sort_unstable();

    // Property vertex → topological representative. Vertices are numbered in order of first
    // use by the reordered triangles, so the result does not depend on the run order either.
    let mut rep: Vec<u32> = (0..n_prop_verts as u32).into_par_iter().collect();
    for (&f, &t) in d.merge_from.iter().zip(&d.merge_to) {
        rep[f as usize] = t as u32;
    }
    let mut new_id = vec![INVALID; n_prop_verts];
    let mut src_vert: Vec<u32> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::with_capacity(n_tris);
    // Per output triangle: (operand, dense group index).
    let mut tri_src: Vec<(u32, u32)> = Vec::with_capacity(n_tris);
    for &(k, lo, hi) in &runs {
        for t in lo..hi {
            tris.push([0, 1, 2].map(|c| {
                let r = rep[d.tri_verts[3 * t + c] as usize] as usize;
                if new_id[r] == INVALID {
                    new_id[r] = src_vert.len() as u32;
                    src_vert.push(r as u32);
                }
                new_id[r]
            }));
        }
        let n_groups = operands[k].groups.len();
        let multi = n_groups > 1 && np >= 4;
        tri_src.par_extend((lo..hi).into_par_iter().map(|t| {
            let gi = if multi {
                let v = d.tri_verts[3 * t] as usize;
                let g = d.vert_props[v * np + 3].round();
                (g.max(0.0) as usize).min(n_groups - 1) as u32
            } else {
                0
            };
            (k as u32, gi)
        }));
    }
    let positions: Vec<Point3> = src_vert
        .par_iter()
        .map(|&r| {
            let i = r as usize * np;
            Point3::new(d.vert_props[i], d.vert_props[i + 1], d.vert_props[i + 2])
        })
        .collect();

    // Group table: operand 0's table, then fresh ids for the later operands' groups that occur.
    let (_, info, next) = base.raw();
    let mut groups = FaceGroups::from_raw(Vec::with_capacity(n_tris), info.clone(), next);
    let mut used: Vec<Vec<bool>> = operands
        .iter()
        .map(|o| vec![false; o.groups.len()])
        .collect();
    for &(k, gi) in &tri_src {
        if let Some(u) = used[k as usize].get_mut(gi as usize) {
            *u = true;
        }
    }
    let mut map: Vec<Vec<FaceGroupId>> = Vec::with_capacity(operands.len());
    for (k, o) in operands.iter().enumerate() {
        let ids = if k == 0 {
            o.groups.clone()
        } else {
            let mut ids = vec![DEFAULT_GROUP; o.groups.len()];
            for (gi, &g) in o.groups.iter().enumerate() {
                if used[k][gi] {
                    let mut info = o.infos[gi]
                        .clone()
                        .unwrap_or_else(|| FaceGroupInfo::new(GroupOrigin::User));
                    info.origin = GroupOrigin::Boolean {
                        operand: k as u8,
                        source: g,
                    };
                    ids[gi] = groups.try_alloc(info)?;
                }
            }
            ids
        };
        map.push(ids);
    }
    for &(k, gi) in &tri_src {
        let id = map[k as usize]
            .get(gi as usize)
            .copied()
            .unwrap_or(DEFAULT_GROUP);
        groups.push(id);
    }
    // An output face from operand 0 whose group vanished from a repaired table, or an empty
    // operand 0, must still reference a table entry.
    if groups.per_face().iter().any(|g| groups.info(*g).is_none()) {
        let fallback = groups.try_alloc(FaceGroupInfo::new(GroupOrigin::User))?;
        let fixed: Vec<FaceGroupId> = groups
            .per_face()
            .iter()
            .map(|g| {
                if groups.info(*g).is_some() {
                    *g
                } else {
                    fallback
                }
            })
            .collect();
        let (_, info, next) = groups.raw();
        groups = FaceGroups::from_raw(fixed, info.clone(), next);
    }
    let mesh = IndexedMesh {
        positions,
        tris,
        attrs: MeshAttrs::default(),
        groups,
    };
    debug_assert!(mesh.validate().is_ok());
    Ok(mesh)
}
