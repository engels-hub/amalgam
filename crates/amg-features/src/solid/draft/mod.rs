//! `solid.draft` (S04 SOL-06, BREP-16): faces tilted by an angle about their intersection with
//! a neutral plane (`BrepKernel::draft`), so a moulded part releases along the pull direction.
//!
//! - **Plane and pull.** The neutral plane is any plane input (origin or construction plane,
//!   planar face); the pull direction is its normal, reversed by `flip`. A positive angle
//!   narrows the part along the pull (removes material on the pull side).
//! - **One side.** The picked faces tilt about the plane.
//! - **Two sides / symmetric.** The faces are split by the plane (`split_faces`); the pieces
//!   on the pull side tilt by `angle` with the pull, those on the other side by `angle2`
//!   (`angle` when symmetric) against it, so both halves narrow away from the parting plane.
//! - **Tangent chain.** OCCT always carries a draft on to the faces tangent to a drafted face,
//!   so the flag only changes [`Draft::plan`] (the preview highlight), as for fillets.
//!
//! Naming (naming note §5.2 "Draft"): none of its own; every face keeps its name (`Modified`
//! 1:1); a face split by a two-sided draft becomes `name + Split { self, i, n }`.

mod dialog;

use amg_brep::{DraftOpts, FaceId, KernelError, OpResult, SubKind, SubShape};
use amg_core::{Dir3, Plane};
use amg_doc::naming::OperandNaming;
use amg_doc::{
    BodyChange, BodyChangeKind, Dependency, Dim, Expr, Feature, FeatureError, FeatureOutput,
    RecomputeCtx, TopoRef, register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{finish_naming, identity_result, input_solid, op_kernel, solid_geom, with_job};
use super::error::{kernel_failure, remap_offending};
use super::shell::{BodyFaces, inline, invalid, plan_faces};
use crate::construct::resolve::resolve_plane_input;
use crate::construct::{OriginEntity, RefInput};

/// How the faces are drafted about the plane.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum DraftSides {
    /// Whole faces about the plane.
    #[default]
    OneSide,
    /// Faces split at the plane; the other side's pieces by `angle2`.
    TwoSides {
        /// The angle on the side opposite to the pull.
        angle2: Expr,
    },
    /// As two sides with `angle2 = angle`.
    Symmetric,
}

fn yes() -> bool {
    true
}

/// `solid.draft`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Draft {
    /// The neutral plane (its normal is the pull direction).
    pub plane: RefInput,
    /// Faces to draft (set-valued; planar, cylindrical or conical; may span bodies).
    pub faces: Vec<TopoRef>,
    /// Draft angle (angle), in (−90°, 90°).
    pub angle: Expr,
    /// Reverse the pull direction.
    #[serde(default)]
    pub flip: bool,
    /// One side, two sides or symmetric.
    #[serde(default)]
    pub sides: DraftSides,
    /// Include the faces tangent to the picked ones in the plan (see the module docs).
    #[serde(default = "yes")]
    pub tangent_chain: bool,
}

impl Draft {
    /// A one-sided draft of `faces` by `angle` (such as `"3 deg"`) about `plane`.
    pub fn one_side(plane: RefInput, faces: Vec<TopoRef>, angle: &str) -> Draft {
        Draft {
            plane,
            faces,
            angle: inline(angle),
            flip: false,
            sides: DraftSides::OneSide,
            tangent_chain: true,
        }
    }

    /// The faces this draft tilts, per body: picks first (with their keys), then the faces
    /// tangent to them when `tangent_chain` is on (OCCT drafts those too).
    ///
    /// # Errors
    /// Resolution errors; [`super::SolidError::Invalid`] for picks that are not faces or no
    /// face left.
    pub fn plan(&self, ctx: &RecomputeCtx<'_>) -> Result<Vec<BodyFaces>, FeatureError> {
        let plan = plan_faces(ctx, &self.faces, self.tangent_chain, "faces")?;
        if plan.iter().all(|p| p.faces.is_empty()) {
            return Err(invalid("faces", "select faces to draft"));
        }
        Ok(plan)
    }
}

impl Default for Draft {
    fn default() -> Draft {
        Draft::one_side(RefInput::Origin(OriginEntity::XYPlane), Vec::new(), "3 deg")
    }
}

fn check_angle(field: &'static str, a: f64) -> Result<f64, FeatureError> {
    if a.is_finite() && a.abs() < std::f64::consts::FRAC_PI_2 - 1e-9 {
        Ok(a)
    } else {
        Err(invalid(
            field,
            "the draft angle must be between -90° and 90°",
        ))
    }
}

/// The images of input faces (operand 0) in `r`.
fn images(r: &OpResult, faces: &[FaceId]) -> Vec<FaceId> {
    let mut v: Vec<FaceId> = Vec::new();
    for &f in faces {
        let input = amg_brep::InputSub {
            operand: 0,
            sub: SubShape {
                kind: SubKind::Face,
                index: f.0,
            },
        };
        for rel in r.history.of_input(input) {
            let outs: Vec<SubShape> = match rel {
                amg_brep::Relation::Modified(o) | amg_brep::Relation::Generated(o) => o.to_vec(),
                amg_brep::Relation::Kept(s) => vec![*s],
                amg_brep::Relation::Deleted => Vec::new(),
            };
            for o in outs {
                if o.kind == SubKind::Face && !v.iter().any(|x| x.0 == o.index) {
                    v.push(FaceId(o.index));
                }
            }
        }
    }
    v
}

/// `a` then `b`, the history composed and the split order re-applied.
fn then(a: OpResult, b: OpResult) -> Result<OpResult, KernelError> {
    let mut history = a.history.compose(&b.history);
    history.order_splits(&b.shape)?;
    let mut warnings = a.warnings;
    warnings.extend(b.warnings);
    Ok(OpResult {
        shape: b.shape,
        history,
        warnings,
    })
}

impl Draft {
    /// Drafts one body's picked faces.
    fn draft_body(
        &self,
        ctx: &RecomputeCtx<'_>,
        p: &BodyFaces,
        plane: &Plane,
        pull: Dir3,
        angles: (f64, Option<f64>),
        out: &mut FeatureOutput,
    ) -> Result<OpResult, FeatureError> {
        let k = op_kernel(ctx);
        let (shape, _) = input_solid(ctx, p.body)?;
        // OCCT propagates along tangent faces itself; give it the picks only.
        let picks: Vec<FaceId> = p
            .faces
            .iter()
            .filter(|f| f.key.is_some())
            .map(|f| f.face)
            .collect();
        let opts = |pull: Dir3, angle: f64| DraftOpts {
            pull,
            angle,
            neutral: Plane::from_normal(plane.origin, pull),
        };
        let fail = |e, out: &mut FeatureOutput| kernel_failure(ctx, Some(p.body), e, out);
        let (a1, a2) = angles;
        let Some(a2) = a2 else {
            return with_job(ctx, |job| k.draft(shape, &picks, &opts(pull, a1), job))
                .map_err(|e| fail(e, out));
        };
        // Two sides: split at the plane, then each side's pieces about it.
        let w = shape.bbox();
        let tool = crate::solid::extrude::sides::plane_face(k, plane, &w, ctx.tolerance().linear)?;
        let split = match with_job(ctx, |job| k.split_faces(shape, &picks, &tool, job)) {
            Ok(r) => r,
            Err(KernelError::InvalidInput { what: "faces", .. }) => identity_result(shape),
            Err(e) => return Err(fail(e, out)),
        };
        let n = pull.get();
        let side_of = |s: &amg_brep::Shape, f: FaceId| -> Result<bool, FeatureError> {
            let g = k
                .sub_geom(
                    s,
                    SubShape {
                        kind: SubKind::Face,
                        index: f.0,
                    },
                )
                .map_err(|e| fail(e, &mut FeatureOutput::default()))?;
            Ok((g.centroid - plane.origin).dot(n) >= 0.0)
        };
        let pieces = images(&split, &picks);
        let mut upper = Vec::new();
        let mut lower = Vec::new();
        for f in pieces {
            if side_of(&split.shape, f)? {
                upper.push(f);
            } else {
                lower.push(f);
            }
        }
        let mut acc = split;
        if !upper.is_empty() {
            let d = with_job(ctx, |job| k.draft(&acc.shape, &upper, &opts(pull, a1), job))
                .map_err(|e| fail(remap_offending(e, &acc.history), out))?;
            lower = images(&d, &lower);
            acc = then(acc, d).map_err(|e| fail(e, out))?;
        }
        if !lower.is_empty() {
            let d = with_job(ctx, |job| {
                k.draft(&acc.shape, &lower, &opts(-pull, a2), job)
            })
            .map_err(|e| fail(remap_offending(e, &acc.history), out))?;
            acc = then(acc, d).map_err(|e| fail(e, out))?;
        }
        Ok(acc)
    }
}

impl Feature for Draft {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d = super::shell::face_inputs(std::iter::empty(), &self.faces);
        if let Some(p) = super::pattern::ref_dependency(&self.plane)
            && !d.contains(&p)
        {
            d.push(p);
        }
        d
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        self.faces.iter().any(|r| *dep == Dependency::Body(r.body))
            || matches!(dep, Dependency::Entity(amg_doc::EntityRef::Topo(t)) if self.faces.contains(t))
    }

    fn exprs(&self) -> Vec<&Expr> {
        let mut v = vec![&self.angle];
        if let DraftSides::TwoSides { angle2 } = &self.sides {
            v.push(angle2);
        }
        v
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        let mut v = vec![(&mut self.angle, Dim::ANGLE)];
        if let DraftSides::TwoSides { angle2 } = &mut self.sides {
            v.push((angle2, Dim::ANGLE));
        }
        v
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self.faces.iter().any(|r| r.kind != amg_doc::TopoKind::Face) {
            return Err(invalid("faces", "select faces"));
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let a1 = check_angle("angle", ctx.eval_angle(&self.angle)?)?;
        let a2 = match &self.sides {
            DraftSides::OneSide => None,
            DraftSides::TwoSides { angle2 } => {
                Some(check_angle("angle2", ctx.eval_angle(angle2)?)?)
            }
            DraftSides::Symmetric => Some(a1),
        };
        let plane = resolve_plane_input(ctx, &self.plane)?;
        let pull = if self.flip {
            -plane.normal
        } else {
            plane.normal
        };
        let plan = self.plan(ctx)?;
        let k = op_kernel(ctx);
        let mut out = FeatureOutput::default();
        for p in plan
            .iter()
            .filter(|p| p.faces.iter().any(|f| f.key.is_some()))
        {
            ctx.check_cancelled()?;
            let r = self.draft_body(ctx, p, &plane, pull, (a1, a2), &mut out)?;
            let (_, naming) = input_solid(ctx, p.body)?;
            let mut b = ctx.naming_builder();
            b.from_history(&[(0, OperandNaming::Named(naming))], &r.history);
            let naming = finish_naming(b, k, &r.shape, &mut out)?;
            out.bodies.push(BodyChange {
                id: p.body,
                change: BodyChangeKind::Modified {
                    geom: solid_geom(r.shape),
                    naming: Some(naming),
                },
            });
        }
        Ok(out)
    }
}

register_feature!(Draft, "solid.draft", version = 1);
