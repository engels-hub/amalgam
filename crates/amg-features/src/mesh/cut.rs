//! `mesh.plane_cut`, `mesh.separate` and `mesh.erase_fill` (MESH-09) as features.

use std::sync::Arc;

use amg_core::{Dimension, Dir3, Plane, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, ConstructionGeom, ConstructionId, Dependency,
    Expr, Feature, FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_mesh::{
    Fairing, HoleFillOpts, IndexedMesh, KeepSide, PlaneCutOpts, erase_and_fill, plane_cut,
    separate_shells,
};
use serde::{Deserialize, Serialize};

use super::base::{check_range, emit, group_faces, input_mesh, same_or_new, warning, with_job};
use super::{MeshEntityRef, MeshFeatureError, OutputMode};

/// An origin plane of the design.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum OriginPlane {
    /// XY, normal +Z.
    XY,
    /// YZ, normal +X.
    YZ,
    /// XZ, normal −Y.
    XZ,
}

/// The cutting plane of [`MeshPlaneCutFeature`] (the S07 `PlaneRef`). The positive side is
/// where the plane's normal points.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum CutPlane {
    /// An origin plane moved along its normal by `offset` (a length).
    Origin {
        /// The plane.
        plane: OriginPlane,
        /// Offset along the normal.
        offset: Expr,
    },
    /// A construction plane (a dependency: editing it recomputes the cut) moved along its
    /// normal by `offset` (a length).
    Construction {
        /// The plane.
        plane: ConstructionId,
        /// Offset along the normal.
        offset: Expr,
    },
    /// A point (three lengths) and a normal (three plain numbers, not all zero).
    Custom {
        /// Point on the plane.
        origin: [Expr; 3],
        /// Normal direction (normalised).
        normal: [Expr; 3],
    },
}

/// Which part of the body a plane cut keeps (the S07 `KeepSideParam`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum KeepSideParam {
    /// The side the plane normal points to.
    Positive,
    /// The other side.
    Negative,
    /// Both: the input body gets the positive part, a new body (slot 0) the negative part.
    Both,
}

/// Cuts a mesh body with a plane, optionally capping the cut (one new `Cap` face group).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshPlaneCutFeature {
    /// The mesh body.
    pub body: BodyId,
    /// The cutting plane.
    pub plane: CutPlane,
    /// What to keep.
    pub keep: KeepSideParam,
    /// Close the cut with a planar cap.
    pub cap: bool,
    /// Vertices this close to the plane count as on it (a length; `0`: exact).
    pub snap_tolerance: Expr,
}

impl MeshPlaneCutFeature {
    /// Cut by an origin plane at `offset` (a length expression), keeping the negative side
    /// (below the plane), capped.
    pub fn origin(body: BodyId, plane: OriginPlane, offset: &str) -> MeshPlaneCutFeature {
        MeshPlaneCutFeature {
            body,
            plane: CutPlane::Origin {
                plane,
                offset: Expr::Inline(offset.into()),
            },
            keep: KeepSideParam::Negative,
            cap: true,
            snap_tolerance: Expr::Inline("0 mm".into()),
        }
    }

    fn plane(&self, ctx: &RecomputeCtx<'_>) -> Result<Plane, FeatureError> {
        let shift = |p: Plane, e: &Expr| -> Result<Plane, FeatureError> {
            let d = check_range(
                "plane.offset",
                ctx.eval_length(e)?,
                f64::NEG_INFINITY,
                false,
                f64::INFINITY,
                false,
            )?;
            Ok(Plane {
                origin: p.origin + p.normal.get() * d,
                ..p
            })
        };
        match &self.plane {
            CutPlane::Origin { plane, offset } => shift(
                match plane {
                    OriginPlane::XY => Plane::XY,
                    OriginPlane::YZ => Plane::YZ,
                    OriginPlane::XZ => Plane::XZ,
                },
                offset,
            ),
            CutPlane::Construction { plane, offset } => match ctx.construction(*plane)? {
                ConstructionGeom::Plane(p) => shift(*p, offset),
                _ => Err(FeatureError::InvalidValue {
                    field: "plane",
                    msg: "the construction is not a plane".to_owned(),
                }),
            },
            CutPlane::Custom { origin, normal } => {
                let mut o = [0.0; 3];
                for (v, e) in o.iter_mut().zip(origin) {
                    *v = ctx.eval_length(e)?;
                }
                let mut n = [0.0; 3];
                for (v, e) in n.iter_mut().zip(normal) {
                    *v = ctx.eval_scalar(e)?;
                }
                let n = Vec3::new(n[0], n[1], n[2]);
                let dir = Dir3::new(n, ctx.tolerance())
                    .filter(|_| n.is_finite() && o.iter().all(|v| v.is_finite()))
                    .ok_or_else(|| FeatureError::InvalidValue {
                        field: "plane.normal",
                        msg: "must be finite and not zero".to_owned(),
                    })?;
                Ok(Plane::from_normal(
                    amg_core::Point3::new(o[0], o[1], o[2]),
                    dir,
                ))
            }
        }
    }
}

impl Feature for MeshPlaneCutFeature {
    fn inputs(&self) -> Vec<Dependency> {
        let mut v = vec![Dependency::Body(self.body)];
        if let CutPlane::Construction { plane, .. } = &self.plane {
            v.push(Dependency::Construction(*plane));
        }
        v
    }

    fn exprs(&self) -> Vec<&Expr> {
        let mut v: Vec<&Expr> = match &self.plane {
            CutPlane::Origin { offset, .. } | CutPlane::Construction { offset, .. } => {
                vec![offset]
            }
            CutPlane::Custom { origin, normal } => origin.iter().chain(normal).collect(),
        };
        v.push(&self.snap_tolerance);
        v
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        let mut v: Vec<(&mut Expr, Dimension)> = match &mut self.plane {
            CutPlane::Origin { offset, .. } | CutPlane::Construction { offset, .. } => {
                vec![(offset, Dimension::LENGTH)]
            }
            CutPlane::Custom { origin, normal } => origin
                .iter_mut()
                .map(|e| (e, Dimension::LENGTH))
                .chain(normal.iter_mut().map(|e| (e, Dimension::NONE)))
                .collect(),
        };
        v.push((&mut self.snap_tolerance, Dimension::LENGTH));
        v
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let plane = self.plane(ctx)?;
        let snap = check_range(
            "snap_tolerance",
            ctx.eval_length(&self.snap_tolerance)?,
            0.0,
            true,
            f64::INFINITY,
            false,
        )?;
        let input = input_mesh(ctx, self.body)?;
        let opts = PlaneCutOpts {
            plane,
            keep: match self.keep {
                KeepSideParam::Positive => KeepSide::Positive,
                KeepSideParam::Negative => KeepSide::Negative,
                KeepSideParam::Both => KeepSide::Both,
            },
            cap: self.cap,
            snap_tolerance: snap,
        };
        ctx.progress(0.0, "cutting mesh");
        let res =
            with_job(ctx, |job| plane_cut(&input, &opts, job)).map_err(MeshFeatureError::from)?;
        let mut out = FeatureOutput::default();
        if res.open_loops > 0 {
            out.warnings.push(warning(format!(
                "{} section loops could not be capped; the cut body is open",
                res.open_loops
            )));
        }
        let nothing = || FeatureError::InvalidValue {
            field: "plane",
            msg: "nothing of the body is on the kept side of the plane".to_owned(),
        };
        let (main, other) = match self.keep {
            KeepSideParam::Positive => (res.positive.ok_or_else(nothing)?, None),
            KeepSideParam::Negative => (res.negative.ok_or_else(nothing)?, None),
            KeepSideParam::Both => match (res.positive, res.negative) {
                (Some(p), Some(n)) => (p, Some(n)),
                (Some(m), None) | (None, Some(m)) => {
                    out.warnings.push(warning(
                        "the plane does not cross the body; nothing was split off".to_owned(),
                    ));
                    (m, None)
                }
                (None, None) => return Err(nothing()),
            },
        };
        emit(
            ctx,
            self.body,
            OutputMode::Replace,
            same_or_new(&input, main),
            &mut out,
        );
        if let Some(n) = other {
            emit(ctx, self.body, OutputMode::NewBody, Arc::new(n), &mut out);
        }
        Ok(out)
    }
}

amg_doc::register_feature!(MeshPlaneCutFeature, "mesh.plane_cut", version = 1);

/// Splits a mesh body into its shells (connected components). The input body keeps the
/// largest shell; the others become new bodies in slots `0, 1, ...` by descending area.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshSeparateFeature {
    /// The mesh body.
    pub body: BodyId,
}

impl Feature for MeshSeparateFeature {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let input = input_mesh(ctx, self.body)?;
        ctx.check_cancelled()?;
        let shells = separate_shells(&input);
        let mut out = FeatureOutput::default();
        let kept: usize = shells.iter().map(IndexedMesh::face_count).sum();
        if kept < input.face_count() {
            out.warnings.push(warning(format!(
                "{} faces with a repeated vertex belong to no shell and were dropped",
                input.face_count() - kept
            )));
        }
        let mut shells = shells.into_iter();
        let first = shells.next().ok_or_else(|| FeatureError::InvalidValue {
            field: "body",
            msg: "the body has no shell".to_owned(),
        })?;
        emit(
            ctx,
            self.body,
            OutputMode::Replace,
            same_or_new(&input, first),
            &mut out,
        );
        for (slot, s) in (0u32..).zip(shells) {
            out.bodies.push(BodyChange {
                id: ctx.new_body_id(slot),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(s)),
                    naming: None,
                },
            });
        }
        Ok(out)
    }
}

amg_doc::register_feature!(MeshSeparateFeature, "mesh.separate", version = 1);

/// Fairing of an erase-and-fill patch.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum FairingParam {
    /// Leave the patch as triangulated and refined.
    None,
    /// Membrane (continuous with the surroundings).
    Harmonic,
    /// Thin plate (tangent-continuous with the surroundings), the default.
    #[default]
    Biharmonic,
}

/// Deletes face groups and fills the holes this opens (MESH-20 hole fill; one new `Fill`
/// group). Holes the body already had stay open.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshEraseFillFeature {
    /// The mesh body.
    pub body: BodyId,
    /// The face groups to erase (at least one).
    pub groups: Vec<MeshEntityRef>,
    /// Patch fairing.
    #[serde(default)]
    pub fairing: FairingParam,
}

impl Feature for MeshEraseFillFeature {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self.groups.is_empty() {
            return Err(FeatureError::InvalidValue {
                field: "groups",
                msg: "select at least one face group to erase".to_owned(),
            });
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let input = input_mesh(ctx, self.body)?;
        let mut out = FeatureOutput::default();
        let faces = group_faces(ctx, &input, &self.groups, &mut out)?;
        if faces.len() == input.face_count() {
            return Err(FeatureError::InvalidValue {
                field: "groups",
                msg: "erasing every face leaves nothing to fill".to_owned(),
            });
        }
        let opts = HoleFillOpts {
            fairing: match self.fairing {
                FairingParam::None => Fairing::None,
                FairingParam::Harmonic => Fairing::Harmonic,
                FairingParam::Biharmonic => Fairing::Biharmonic,
            },
            ..HoleFillOpts::default()
        };
        ctx.progress(0.0, "erasing and filling");
        let mesh = with_job(ctx, |job| erase_and_fill(&input, &faces, &opts, job))
            .map_err(MeshFeatureError::from)?;
        emit(
            ctx,
            self.body,
            OutputMode::Replace,
            Arc::new(mesh),
            &mut out,
        );
        Ok(out)
    }
}

amg_doc::register_feature!(MeshEraseFillFeature, "mesh.erase_fill", version = 1);
