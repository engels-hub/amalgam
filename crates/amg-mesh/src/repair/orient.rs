//! Orientation: make the faces of every edge-connected component agree, then point closed
//! components outward (or inward, for cavities).

use std::sync::Arc;

use amg_core::Point3;
use amg_jobs::JobCtx;

use super::edges::EdgeTable;
use crate::INVALID;
use crate::error::MeshResult;
use crate::indexed::IndexedMesh;
use crate::spatial::{BvhOpts, DEFAULT_WINDING_BETA, MeshSpatial};

/// Closed components beyond this many are all oriented outward (nesting is not looked at).
const MAX_NESTING_COMPONENTS: usize = 32;
/// Vertices of a component tested for containment in another one.
const NESTING_SAMPLES: usize = 8;

/// Result of the BFS over manifold edges.
pub(crate) struct Orientation {
    /// Face must be flipped to agree with the first face of its component.
    pub flip: Vec<bool>,
    /// Component of every face.
    pub comp: Vec<u32>,
    /// A component has an edge with other than two faces.
    pub open: Vec<bool>,
}

/// Components are connected through edges shared by exactly two faces only.
pub(crate) fn consistent_flips(
    tris: &[[u32; 3]],
    table: &EdgeTable,
    ctx: &JobCtx,
) -> MeshResult<Orientation> {
    let n = tris.len();
    let mut nbr = vec![INVALID; 3 * n];
    for g in 0..table.len() {
        if let [(_, a), (_, b)] = table.group(g) {
            nbr[*a as usize] = *b;
            nbr[*b as usize] = *a;
        }
    }
    let mut flip = vec![false; n];
    let mut comp = vec![INVALID; n];
    let mut open: Vec<bool> = Vec::new();
    let mut stack: Vec<u32> = Vec::new();
    for seed in 0..n {
        if comp[seed] != INVALID {
            continue;
        }
        if seed % 4096 == 0 {
            ctx.checkpoint()?;
        }
        let c = open.len() as u32;
        open.push(false);
        comp[seed] = c;
        stack.push(seed as u32);
        while let Some(f) = stack.pop() {
            let f = f as usize;
            let t = tris[f];
            for k in 0..3 {
                let other = nbr[3 * f + k];
                if other == INVALID {
                    // Edges of repeated-index faces are not in the table: they do not count.
                    if t[k] != t[(k + 1) % 3] {
                        open[c as usize] = true;
                    }
                    continue;
                }
                let (g, kg) = ((other / 3) as usize, (other % 3) as usize);
                if comp[g] != INVALID {
                    continue;
                }
                // Same raw direction needs exactly one of the two faces flipped.
                let same = tris[g][kg] == t[k];
                flip[g] = same ^ flip[f];
                comp[g] = c;
                stack.push(g as u32);
            }
        }
    }
    Ok(Orientation { flip, comp, open })
}

fn tet6(o: Point3, a: Point3, b: Point3, c: Point3) -> f64 {
    (a - o).dot((b - o).cross(c - o))
}

/// Final `flip` per face: consistent within every edge-connected component, and each
/// component pointing outward by signed volume. A closed component that lies inside another
/// closed component (a cavity) points inward, by the winding number of the other one.
pub(crate) fn oriented_flips(
    m: &IndexedMesh,
    table: &EdgeTable,
    ctx: &JobCtx,
) -> MeshResult<Vec<bool>> {
    let o = consistent_flips(&m.tris, table, ctx)?;
    ctx.checkpoint()?;
    let ncomp = o.open.len();
    // Reference point per component: its mean vertex, so open components are least biased.
    let mut sum = vec![(0.0f64, 0.0f64, 0.0f64, 0usize); ncomp];
    for (f, t) in m.tris.iter().enumerate() {
        let s = &mut sum[o.comp[f] as usize];
        let p = m.positions[t[0] as usize];
        s.0 += p.x;
        s.1 += p.y;
        s.2 += p.z;
        s.3 += 1;
    }
    let refp: Vec<Point3> = sum
        .iter()
        .map(|s| {
            let k = s.3.max(1) as f64;
            Point3::new(s.0 / k, s.1 / k, s.2 / k)
        })
        .collect();
    let mut vol = vec![0.0f64; ncomp];
    for (f, t) in m.tris.iter().enumerate() {
        let c = o.comp[f] as usize;
        let [a, b, d] = t.map(|i| m.positions[i as usize]);
        let v = tet6(refp[c], a, b, d);
        vol[c] += if o.flip[f] { -v } else { v };
    }
    // Desired sign per component: outward unless it is a cavity of another closed component.
    let closed: Vec<usize> = (0..ncomp).filter(|&c| !o.open[c]).collect();
    let depth = if (2..=MAX_NESTING_COMPONENTS).contains(&closed.len()) {
        nesting_depths(m, &o, &closed, ctx)?
    } else {
        vec![0; closed.len()]
    };
    let mut want_negative = vec![false; ncomp];
    for (i, &c) in closed.iter().enumerate() {
        want_negative[c] = depth[i] % 2 == 1;
    }
    let toggle: Vec<bool> = (0..ncomp)
        .map(|c| vol[c] != 0.0 && (vol[c] < 0.0) != want_negative[c])
        .collect();
    Ok((0..m.tris.len())
        .map(|f| o.flip[f] ^ toggle[o.comp[f] as usize])
        .collect())
}

/// Makes every edge-connected component consistently oriented and points it outward (see
/// [`oriented_flips`]). Edges shared by more than two faces are not crossed. Returns the
/// number of faces flipped. Vertex normals are left alone.
pub(crate) fn orient_consistently(m: &mut IndexedMesh, ctx: &JobCtx) -> MeshResult<usize> {
    m.validate()?;
    if m.tris.is_empty() {
        return Ok(0);
    }
    let table = EdgeTable::build(&m.tris)?;
    let flips = oriented_flips(m, &table, ctx)?;
    drop(table);
    let mut flipped = 0usize;
    for (f, _) in flips.iter().enumerate().filter(|(_, x)| **x) {
        m.tris[f].swap(1, 2);
        let (uv, pi) = m.attrs.corners(f);
        m.attrs
            .set_corners(f, [uv[0], uv[2], uv[1]], [pi[0], pi[2], pi[1]]);
        flipped += 1;
    }
    Ok(flipped)
}

/// Number of other closed components that contain (nearly) all sample vertices of each closed
/// component.
fn nesting_depths(
    m: &IndexedMesh,
    o: &Orientation,
    closed: &[usize],
    ctx: &JobCtx,
) -> MeshResult<Vec<u32>> {
    let mut slot_of = vec![usize::MAX; o.open.len()];
    for (s, &c) in closed.iter().enumerate() {
        slot_of[c] = s;
    }
    let mut faces: Vec<Vec<u32>> = vec![Vec::new(); closed.len()];
    for (f, &c) in o.comp.iter().enumerate() {
        let s = slot_of[c as usize];
        if s != usize::MAX {
            faces[s].push(f as u32);
        }
    }
    let mut lo = vec![[f64::INFINITY; 3]; closed.len()];
    let mut hi = vec![[f64::NEG_INFINITY; 3]; closed.len()];
    let mut samples: Vec<Vec<Point3>> = vec![Vec::new(); closed.len()];
    for (s, fl) in faces.iter().enumerate() {
        for &f in fl {
            for &i in &m.tris[f as usize] {
                let p = m.positions[i as usize];
                for (k, x) in [p.x, p.y, p.z].into_iter().enumerate() {
                    lo[s][k] = lo[s][k].min(x);
                    hi[s][k] = hi[s][k].max(x);
                }
            }
        }
        let step = (fl.len() / NESTING_SAMPLES).max(1);
        samples[s] = fl
            .iter()
            .step_by(step)
            .take(NESTING_SAMPLES)
            .map(|&f| m.positions[m.tris[f as usize][0] as usize])
            .collect();
    }
    let mut depth = vec![0u32; closed.len()];
    for t in 0..closed.len() {
        let mut spatial: Option<MeshSpatial> = None;
        for s in 0..closed.len() {
            if s == t || (0..3).any(|k| lo[s][k] < lo[t][k] || hi[s][k] > hi[t][k]) {
                continue;
            }
            ctx.checkpoint()?;
            if spatial.is_none() {
                spatial = Some(sub_spatial(m, &faces[t], ctx)?);
            }
            let Some(sp) = &spatial else { continue };
            let w = sp.winding_numbers(&samples[s], DEFAULT_WINDING_BETA);
            if !w.is_empty() && w.iter().all(|x| x.abs() > 0.5) {
                depth[s] += 1;
            }
        }
    }
    Ok(depth)
}

/// Spatial index over the listed faces only.
fn sub_spatial(m: &IndexedMesh, faces: &[u32], ctx: &JobCtx) -> MeshResult<MeshSpatial> {
    let mut used: Vec<u32> = faces.iter().flat_map(|&f| m.tris[f as usize]).collect();
    used.sort_unstable();
    used.dedup();
    let positions: Vec<Point3> = used.iter().map(|&i| m.positions[i as usize]).collect();
    let tris: Vec<[u32; 3]> = faces
        .iter()
        .map(|&f| m.tris[f as usize].map(|i| used.binary_search(&i).map_or(0, |p| p as u32)))
        .collect();
    let sub = IndexedMesh::new(positions, tris)?;
    MeshSpatial::build(Arc::new(sub), &BvhOpts::default(), ctx)
}
