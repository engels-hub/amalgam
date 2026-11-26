//! `solid.split_face`: faces split by a surface tool (a plane, face or body) or by sketch
//! curves swept along a direction (`BrepKernel::split_faces`). The body stays one solid.
//!
//! Naming (note §5.2 "Split face"): split faces are `name + Split { self, i, n }` in the
//! default geometric order; edges re-derive. Picked faces are registered as selections.

use amg_brep::{BrepKernel, FaceId, KernelError, Shape};
use amg_core::Aabb3;
use amg_doc::naming::OperandNaming;
use amg_doc::{
    BodyChange, BodyChangeKind, Dependency, EntityRef, Feature, FeatureError, FeatureOutput,
    RecomputeCtx, SetPolicy, TopoRef, register_feature,
};
use serde::{Deserialize, Serialize};

use super::{curves_sheet, invalid, split_tool, tool_dependency};
use crate::construct::RefInput;
use crate::solid::common::{
    finish_naming, live_body, op_kernel, resolve_subs, solid_geom, with_job,
};
use crate::solid::error::{SolidError, kernel_failure, with_hint};

/// The tool of a split face.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SplitFaceTool {
    /// A plane (origin or construction), a face (extended with `extend_tool`) or a body.
    Surface(RefInput),
    /// Sketch curves (lines and arcs) swept along `dir` (`None` = the sketch normal).
    CurvesAlong {
        /// The sketch curves.
        curves: Vec<EntityRef>,
        /// The sweep direction: an axis, straight edge or sketch line.
        dir: Option<RefInput>,
    },
}

/// `solid.split_face`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SplitFace {
    /// The faces to split.
    #[serde(default)]
    pub faces: Vec<TopoRef>,
    /// The tool.
    pub tool: SplitFaceTool,
    /// Extend a face tool to its untrimmed surface.
    #[serde(default)]
    pub extend_tool: bool,
}

impl SplitFace {
    /// `faces` split by a surface tool.
    pub fn by_surface(faces: Vec<TopoRef>, tool: RefInput) -> SplitFace {
        SplitFace {
            faces,
            tool: SplitFaceTool::Surface(tool),
            extend_tool: false,
        }
    }

    /// `faces` split by sketch curves along the sketch normal.
    pub fn by_curves(faces: Vec<TopoRef>, curves: Vec<EntityRef>) -> SplitFace {
        SplitFace {
            faces,
            tool: SplitFaceTool::CurvesAlong { curves, dir: None },
            extend_tool: false,
        }
    }

    fn tool_ref(&self) -> Option<&RefInput> {
        match &self.tool {
            SplitFaceTool::Surface(r) => Some(r),
            SplitFaceTool::CurvesAlong { dir, .. } => dir.as_ref(),
        }
    }

    /// The faces' `NoSplit` error, tool highlighted.
    fn no_split(&self, ctx: &RecomputeCtx<'_>) -> FeatureError {
        match &self.tool {
            SplitFaceTool::Surface(r) => super::no_split(ctx, r),
            SplitFaceTool::CurvesAlong { curves, .. } => {
                ctx.highlight(curves.iter().cloned());
                with_hint(&SolidError::NoSplit)
            }
        }
    }

    fn tool_shape(
        &self,
        ctx: &RecomputeCtx<'_>,
        k: &dyn BrepKernel,
        w: &Aabb3,
    ) -> Result<Shape, FeatureError> {
        match &self.tool {
            SplitFaceTool::Surface(r) => {
                split_tool(ctx, k, r, self.extend_tool, w).map(|t| t.shape)
            }
            SplitFaceTool::CurvesAlong { curves, dir } => {
                curves_sheet(ctx, k, curves, dir.as_ref(), w)
            }
        }
    }
}

impl Feature for SplitFace {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d: Vec<Dependency> = self
            .faces
            .iter()
            .cloned()
            .map(|t| Dependency::Entity(EntityRef::Topo(t)))
            .collect();
        if let SplitFaceTool::CurvesAlong { curves, .. } = &self.tool {
            for c in curves {
                if let EntityRef::SketchCurve { sketch, .. } = c {
                    let s = Dependency::Sketch(*sketch);
                    if !d.contains(&s) {
                        d.push(s);
                    }
                }
                d.push(Dependency::Entity(c.clone()));
            }
        }
        d.extend(self.tool_ref().and_then(tool_dependency));
        d
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        crate::solid::fillet::lineage_optional(dep)
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        if self.faces.is_empty() {
            return Err(invalid("faces", "select the faces to split"));
        }
        let picks = resolve_subs(ctx, &self.faces, SetPolicy::DropMissing)?;
        if picks.is_empty() {
            return Err(invalid("faces", "none of the faces exists any more"));
        }
        let mut bodies = Vec::with_capacity(picks.len());
        let mut w: Option<Aabb3> = None;
        for (body, subs) in &picks {
            let Some(p) = live_body(ctx, *body, &mut out)? else {
                continue;
            };
            let b = p.shape.bbox();
            w = Some(w.map_or(b, |x| x.union(&b)));
            bodies.push((p, subs));
        }
        let Some(w) = w else {
            return Err(invalid("faces", "none of the faces exists any more"));
        };
        let k = op_kernel(ctx);
        let tool = self.tool_shape(ctx, k, &w.expanded(1.0))?;
        for (p, subs) in bodies {
            ctx.check_cancelled()?;
            let faces: Vec<FaceId> = subs
                .iter()
                .filter(|(s, _)| s.kind == amg_brep::SubKind::Face)
                .map(|(s, _)| FaceId(s.index))
                .collect();
            if faces.is_empty() {
                return Err(invalid("faces", "pick faces, not edges or vertices"));
            }
            let r = with_job(ctx, |job| k.split_faces(p.shape, &faces, &tool, job)).map_err(
                |e| match e {
                    KernelError::InvalidInput { what: "faces", .. } => self.no_split(ctx),
                    e => kernel_failure(ctx, Some(p.id), e, &mut out),
                },
            )?;
            let mut b = ctx.naming_builder();
            b.from_history(&[(0, OperandNaming::Named(p.naming))], &r.history);
            for (s, key) in subs {
                b.selection(0, *s, *key);
            }
            let naming = finish_naming(b, k, &r.shape, &mut out)?;
            out.bodies.push(BodyChange {
                id: p.id,
                change: BodyChangeKind::Modified {
                    geom: solid_geom(r.shape),
                    naming: Some(naming),
                },
            });
        }
        Ok(out)
    }
}

register_feature!(SplitFace, "solid.split_face", version = 1);

#[cfg(test)]
mod tests {
    use super::*;
    use amg_doc::FeatureId;
    use amg_doc::{SketchId, stub::SketchEntityKey};

    #[test]
    fn curve_inputs_list_sketch_and_curve_once() {
        let sk = SketchId(FeatureId(4));
        let c = |i| EntityRef::SketchCurve {
            sketch: sk,
            curve: SketchEntityKey(i),
        };
        let f = SplitFace::by_curves(Vec::new(), vec![c(1), c(2)]);
        assert_eq!(
            f.inputs(),
            vec![
                Dependency::Sketch(sk),
                Dependency::Entity(c(1)),
                Dependency::Entity(c(2))
            ]
        );
    }

    #[test]
    fn json_round_trips() {
        let f = SplitFace::by_surface(
            Vec::new(),
            RefInput::Origin(crate::construct::OriginEntity::XYPlane),
        );
        let s = serde_json::to_string(&f).unwrap();
        assert_eq!(serde_json::from_str::<SplitFace>(&s).unwrap(), f);
    }
}
