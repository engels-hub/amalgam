//! Option A: the SK-13 native prototype ([`crate::solve::proto`]).

use super::{Contender, Loaded, Outcome};
use crate::corpus::runner::BackendSkip;
use crate::corpus::{BuiltCase, CaseStatus, ExpectedGroup, GroupKind, Values};
use crate::solve::EquationOwner;
use crate::solve::proto::{
    self, DragMethod, ProtoAlgorithm, ProtoDrag, ProtoGroupKind, ProtoOptions, ProtoReport,
    ProtoStatus,
};
use amg_core::Point2;

/// The native prototype with fixed options.
#[derive(Clone, Debug)]
pub struct Native {
    /// Report name.
    pub label: &'static str,
    /// Solve options.
    pub opts: ProtoOptions,
    /// Drag method.
    pub drag: DragMethod,
}

impl Default for Native {
    /// LM, with rank/DOF analysis and diagnosis, weighted min-norm drag.
    fn default() -> Self {
        Native {
            label: "native",
            opts: ProtoOptions::default(),
            drag: DragMethod::WeightedMinNorm,
        }
    }
}

impl Native {
    /// Solve only (no rank test, no diagnosis): the warm-solve / drag inner loop.
    pub fn fast() -> Self {
        Native {
            label: "native-fast",
            opts: ProtoOptions::fast(),
            ..Self::default()
        }
    }

    /// Powell dogleg instead of LM.
    pub fn dogleg() -> Self {
        Native {
            label: "native-dogleg",
            opts: ProtoOptions {
                algorithm: ProtoAlgorithm::DogLeg,
                ..ProtoOptions::default()
            },
            ..Self::default()
        }
    }

    /// PlaneGCS-style SQP dragging.
    pub fn sqp_drag() -> Self {
        Native {
            label: "native-sqp",
            drag: DragMethod::Sqp,
            ..Self::default()
        }
    }
}

/// Maps a prototype status to the corpus status.
pub fn status(s: ProtoStatus) -> CaseStatus {
    match s {
        ProtoStatus::Converged => CaseStatus::Converged,
        ProtoStatus::ConvergedRedundant => CaseStatus::ConvergedRedundant,
        ProtoStatus::Inconsistent => CaseStatus::Inconsistent,
        ProtoStatus::DidNotConverge => CaseStatus::DidNotConverge,
    }
}

/// Dependency groups of a report as case constraint indices (arc-internal rows dropped).
pub fn groups(built: &BuiltCase, report: &ProtoReport) -> Vec<ExpectedGroup> {
    report
        .groups
        .iter()
        .filter_map(|g| {
            let mut cs: Vec<u32> = g
                .owners
                .iter()
                .filter_map(|o| match o {
                    EquationOwner::Constraint(id) => built.constraint_index(*id),
                    EquationOwner::Entity(_) => None,
                })
                .collect();
            cs.sort_unstable();
            cs.dedup();
            (!cs.is_empty()).then_some(ExpectedGroup {
                kind: match g.kind {
                    ProtoGroupKind::Redundant => GroupKind::Redundant,
                    ProtoGroupKind::Conflicting => GroupKind::Conflicting,
                },
                constraints: cs,
            })
        })
        .collect()
}

struct NativeLoaded {
    built: BuiltCase,
    opts: ProtoOptions,
    method: DragMethod,
    drag: Option<(u32, ProtoDrag)>,
}

impl Contender for Native {
    fn name(&self) -> String {
        self.label.into()
    }

    fn load(&self, built: &BuiltCase) -> Result<Box<dyn Loaded>, BackendSkip> {
        Ok(Box::new(NativeLoaded {
            built: built.clone(),
            opts: self.opts.clone(),
            method: self.drag,
            drag: None,
        }))
    }
}

impl Loaded for NativeLoaded {
    fn solve(&mut self) -> Outcome {
        self.drag = None;
        match proto::solve(&mut self.built.sketch, &self.opts) {
            Ok(r) => Outcome {
                status: status(r.status),
                dof: r.dof,
                groups: self.opts.diagnose.then(|| groups(&self.built, &r)),
                iterations: Some(r.iterations),
            },
            Err(_) => Outcome {
                status: CaseStatus::DidNotConverge,
                dof: None,
                groups: None,
                iterations: None,
            },
        }
    }

    fn values(&self) -> Values {
        self.built.values()
    }

    fn set_dimension(&mut self, constraint: u32, value: f64) -> Result<(), String> {
        self.drag = None;
        let id = *self
            .built
            .constraints
            .get(constraint as usize)
            .ok_or("constraint index out of range")?;
        self.built
            .sketch
            .set_dimension_value(id, value)
            .map_err(|e| e.to_string())
    }

    fn drag(&mut self, point: u32, target: [f64; 2]) -> Outcome {
        let fail = Outcome {
            status: CaseStatus::DidNotConverge,
            dof: None,
            groups: None,
            iterations: None,
        };
        if self.drag.as_ref().is_none_or(|(p, _)| *p != point) {
            let Some(&id) = self.built.entities.get(point as usize) else {
                return fail;
            };
            match ProtoDrag::begin(&self.built.sketch, id, self.method, &self.opts) {
                Ok(d) => self.drag = Some((point, d)),
                Err(_) => return fail,
            }
        }
        let Some((_, d)) = self.drag.as_mut() else {
            return fail;
        };
        let frame = d.step(&mut self.built.sketch, Point2::new(target[0], target[1]));
        Outcome {
            status: status(frame.status),
            dof: None,
            groups: None,
            iterations: Some(frame.iterations),
        }
    }
}
