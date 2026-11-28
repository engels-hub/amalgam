//! Face patterns (S04 §5, SOL-23): the picked faces of a boss or pocket on a planar support,
//! turned into a tool solid and patterned like a feature's tool.
//!
//! The faces must belong to one solid body, and every edge where they meet the rest of the
//! body (the boundary) must lie on one planar face of the body that is not picked, the
//! support. The tool is the picked faces sewn with a cap on the support plane bounded by the
//! boundary loops (largest loop outer). Faces reaching out of the support (along its outward
//! normal) make a **boss**, joined to the body per instance; faces reaching into the material
//! make a **pocket**, cut from it.
//!
//! Tool names: the face built from picked face `j` is `PatternFace#j` with the picked face's
//! name as its source, the cap `PatternCap#0`; the instances add `Inst { self, k }` as every
//! pattern does.

use amg_brep::geom::{CurveSeg, FaceBounds, Surface, Tag};
use amg_brep::{
    BrepKernel, EdgeId, FaceId, SewOpts, Shape, SubKind, SubShape, TessQuality, VertexId,
};
use amg_core::Plane;
use amg_doc::naming::{NameSource, Role};
use amg_doc::{
    BodyId, BodyNaming, FeatureError, FeatureOutput, RecomputeCtx, SetPolicy, TopoKind, TopoRef,
};

use super::invalid;
use crate::solid::common::{BodyOp, finish_naming, input_solid, resolve_subs, with_job};
use crate::solid::error::kernel_failure;

/// A face pattern's tool before placement.
#[derive(Clone, Debug)]
pub(crate) struct FaceTool {
    /// The closed tool solid at the seed position.
    pub(crate) shape: Shape,
    /// Its names.
    pub(crate) naming: BodyNaming,
    /// `Join` for a boss, `Cut` for a pocket.
    pub(crate) op: BodyOp,
    /// The body the faces belong to.
    pub(crate) body: BodyId,
}

/// The support plane with its normal pointing out of the material.
fn outward_plane(shape: &Shape, f: FaceId) -> Option<Plane> {
    match shape.surface(f) {
        Surface::Plane(p) if shape.face_reversed(f) => Some(p.flipped()),
        Surface::Plane(p) => Some(p),
        _ => None,
    }
}

/// The boundary edges chained into closed loops (by shared vertices).
fn loops(shape: &Shape, boundary: &[EdgeId]) -> Result<Vec<Vec<EdgeId>>, FeatureError> {
    let mut left: Vec<EdgeId> = boundary.to_vec();
    let mut out = Vec::new();
    while let Some(first) = left.pop() {
        let [start, mut at] = shape.edge_vertices(first);
        let mut lp = vec![first];
        // A closed edge (a full circle) is a loop by itself.
        while at != start {
            let next = left
                .iter()
                .position(|e| shape.edge_vertices(*e).contains(&at))
                .ok_or_else(|| {
                    invalid(
                        "faces",
                        "the boundary of the faces is not closed on the support face",
                    )
                })?;
            let e = left.swap_remove(next);
            let [a, b] = shape.edge_vertices(e);
            at = if a == at { b } else { a };
            lp.push(e);
        }
        out.push(lp);
    }
    Ok(out)
}

/// The diagonal of a loop's bounding box (sampled edges), to put the outer loop first.
fn loop_size(k: &dyn BrepKernel, shape: &Shape, lp: &[EdgeId]) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for &e in lp {
        let pts = k
            .tessellate_edge(shape, e, &TessQuality::preview())
            .unwrap_or_default();
        let ends = shape
            .edge_vertices(e)
            .map(|v: VertexId| shape.vertex_point(v));
        for p in pts.iter().chain(&ends) {
            for (i, c) in p.to_array().into_iter().enumerate() {
                lo[i] = lo[i].min(c);
                hi[i] = hi[i].max(c);
            }
        }
    }
    (0..3).map(|i| (hi[i] - lo[i]).powi(2)).sum::<f64>().sqrt()
}

/// Builds the tool of the picked faces `refs`.
pub(crate) fn face_tool(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    refs: &[TopoRef],
    out: &mut FeatureOutput,
) -> Result<FaceTool, FeatureError> {
    if refs.is_empty() {
        return Err(invalid("faces", "select at least one face"));
    }
    let groups = resolve_subs(ctx, refs, SetPolicy::DropMissing)?;
    let Some((body, picks)) = groups.first() else {
        return Err(invalid("faces", "none of the picked faces exists any more"));
    };
    if groups.len() > 1 {
        return Err(invalid("faces", "the faces must belong to one body"));
    }
    let body = *body;
    let mut faces: Vec<FaceId> = Vec::new();
    for (sub, _) in picks {
        if sub.kind != SubKind::Face {
            return Err(invalid("faces", "select faces"));
        }
        if !faces.contains(&FaceId(sub.index)) {
            faces.push(FaceId(sub.index));
        }
    }
    let (shape, naming) = input_solid(ctx, body)?;
    let highlight = || ctx.highlight(refs.iter().cloned().map(amg_doc::EntityRef::Topo));

    // Boundary edges and the support faces across them.
    let mut boundary: Vec<EdgeId> = Vec::new();
    let mut support: Vec<FaceId> = Vec::new();
    for &f in &faces {
        for &e in shape.face_edges(f) {
            let others: Vec<FaceId> = shape
                .edge_faces(e)
                .iter()
                .copied()
                .filter(|g| !faces.contains(g))
                .collect();
            match others.as_slice() {
                [] => {}
                [g] => {
                    if !boundary.contains(&e) {
                        boundary.push(e);
                    }
                    if !support.contains(g) {
                        support.push(*g);
                    }
                }
                _ => {
                    highlight();
                    return Err(invalid("faces", "a boundary edge is non-manifold"));
                }
            }
        }
    }
    if boundary.is_empty() {
        highlight();
        return Err(invalid(
            "faces",
            "the faces form a closed body; pick the faces of a boss or a pocket",
        ));
    }
    let tol = ctx.tolerance().linear.max(shape.max_tolerance());
    let plane = outward_plane(shape, support[0]);
    let plane = match plane {
        Some(p)
            if support.iter().all(|&g| {
                outward_plane(shape, g).is_some_and(|q| {
                    q.normal.get().dot(p.normal.get()) > 1.0 - 1e-9
                        && p.signed_distance(q.origin).abs() <= 10.0 * tol
                })
            }) =>
        {
            p
        }
        _ => {
            highlight();
            return Err(invalid(
                "faces",
                "the faces must sit on one planar face (a boss or a pocket on a planar support)",
            ));
        }
    };

    // Boss (out of the support) or pocket (into the material): the farthest vertex decides.
    let mut far = 0.0f64;
    for &f in &faces {
        for &e in shape.face_edges(f) {
            for v in shape.edge_vertices(e) {
                let d = plane.signed_distance(shape.vertex_point(v));
                if d.abs() > far.abs() {
                    far = d;
                }
            }
        }
    }
    if far.abs() <= 10.0 * tol {
        highlight();
        return Err(invalid(
            "faces",
            "the faces are flat on the support; pick the faces of a boss or a pocket",
        ));
    }
    let op = if far > 0.0 { BodyOp::Join } else { BodyOp::Cut };

    // Cap on the support plane, outer loop first.
    let mut lps = loops(shape, &boundary).inspect_err(|_| highlight())?;
    let mut sized: Vec<(f64, Vec<EdgeId>)> = lps
        .drain(..)
        .map(|l| (loop_size(k, shape, &l), l))
        .collect();
    sized.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut segs: Vec<Vec<CurveSeg>> = Vec::with_capacity(sized.len());
    for (_, lp) in &sized {
        let mut v = Vec::with_capacity(lp.len());
        for &e in lp {
            let Some((curve, t0, t1)) = shape.curve(e) else {
                return Err(invalid("faces", "a boundary edge has no curve"));
            };
            v.push(CurveSeg {
                curve,
                t0,
                t1,
                tag: Tag(u64::from(e.0)),
            });
        }
        segs.push(v);
    }
    let cap = k
        .make_face(&Surface::Plane(plane), &FaceBounds::Loops(segs), 10.0 * tol)
        .map_err(|e| kernel_failure(ctx, Some(body), e, out))?
        .face;

    // Sew the picked faces and the cap into a solid.
    let mut parts: Vec<Shape> = Vec::with_capacity(faces.len() + 1);
    for &f in &faces {
        parts.push(
            shape
                .sub_shape(SubShape {
                    kind: SubKind::Face,
                    index: f.0,
                })
                .map_err(|e| kernel_failure(ctx, Some(body), e, out))?,
        );
    }
    parts.push(cap);
    let opts = SewOpts {
        tol: (10.0 * tol).max(1e-6),
        non_manifold: false,
        make_solid: true,
    };
    let r = with_job(ctx, |j| k.sew(&parts, &opts, j))
        .map_err(|e| kernel_failure(ctx, Some(body), e, out))?;
    if r.shape.solids().len() != 1 {
        highlight();
        return Err(invalid(
            "faces",
            "the faces and the support don't close a single solid",
        ));
    }

    // Names: PatternFace#j from picked face j, PatternCap#0 for the cap.
    let cap_operand = u16::try_from(faces.len()).unwrap_or(u16::MAX);
    let mut b = ctx.naming_builder();
    for o in 0..r.shape.count(SubKind::Face) {
        let src = r
            .history
            .sources_of(SubShape {
                kind: SubKind::Face,
                index: o,
            })
            .into_iter()
            .map(|(i, _)| i.operand)
            .min();
        match src {
            Some(j) if j < cap_operand => {
                let picked = faces[usize::from(j)];
                b.generated(
                    TopoKind::Face,
                    o,
                    Role::new("PatternFace", u32::from(j)),
                    naming
                        .name_of(TopoKind::Face, picked.0)
                        .cloned()
                        .map(NameSource::Topo),
                );
            }
            _ => {
                b.generated(TopoKind::Face, o, Role::new("PatternCap", 0), None);
            }
        }
    }
    let mut scratch = FeatureOutput::default();
    let tool_naming = finish_naming(b, k, &r.shape, &mut scratch)?;
    Ok(FaceTool {
        shape: r.shape,
        naming: tool_naming,
        op,
        body,
    })
}
