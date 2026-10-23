//! Evaluated state (S01 §1, §3): everything recompute derives from a [`Design`] snapshot.
//!
//! An [`Evaluated`] is built by a recompute job (DOC-03/DOC-12), swapped in on the UI thread in
//! one `Arc` store, and never undone. All payloads are `Send + Sync`.
//!
//! [`Design`]: crate::Design

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use amg_core::{Line3, Plane, Point3};
use serde::{Deserialize, Serialize};

use crate::id::{BodyId, ComponentId, ConstructionId, FeatureId, ParamId, Revision, SketchId};
use crate::naming::{BodyNaming, EntityRef, ResolveError};
use crate::stub::{IndexedMesh, SdfGraph, Shape, SketchGeom, SubdMesh};

/// Recompute results for one authored revision.
#[derive(Clone, Debug, Default)]
pub struct Evaluated {
    /// The authored revision this was computed from.
    pub revision: Revision,
    bodies: BTreeMap<BodyId, Arc<EvalBody>>,
    sketches: BTreeMap<SketchId, Arc<SketchGeom>>,
    construction: BTreeMap<ConstructionId, ConstructionGeom>,
    nodes: BTreeMap<FeatureId, NodeEval>,
}

impl Evaluated {
    /// Empty evaluated state for `revision` (nothing computed yet).
    pub fn new(revision: Revision) -> Evaluated {
        Evaluated {
            revision,
            ..Evaluated::default()
        }
    }

    /// A body as it exists at the end of the rolled-forward timeline.
    pub fn body(&self, id: BodyId) -> Option<&Arc<EvalBody>> {
        self.bodies.get(&id)
    }

    /// All existing bodies in id order.
    pub fn bodies(&self) -> impl Iterator<Item = (BodyId, &Arc<EvalBody>)> + '_ {
        self.bodies.iter().map(|(k, v)| (*k, v))
    }

    /// A solved sketch.
    pub fn sketch(&self, id: SketchId) -> Option<&Arc<SketchGeom>> {
        self.sketches.get(&id)
    }

    /// All solved sketches in id order.
    pub fn sketches(&self) -> impl Iterator<Item = (SketchId, &Arc<SketchGeom>)> + '_ {
        self.sketches.iter().map(|(k, v)| (*k, v))
    }

    /// All construction geometry in id order.
    pub fn constructions(&self) -> impl Iterator<Item = (ConstructionId, &ConstructionGeom)> + '_ {
        self.construction.iter().map(|(k, v)| (*k, v))
    }

    /// A construction plane/axis/point.
    pub fn construction(&self, id: ConstructionId) -> Option<&ConstructionGeom> {
        self.construction.get(&id)
    }

    /// Health and cache key of a timeline node.
    pub fn node(&self, id: FeatureId) -> Option<&NodeEval> {
        self.nodes.get(&id)
    }

    /// All evaluated nodes in id order.
    pub fn nodes(&self) -> impl Iterator<Item = (FeatureId, &NodeEval)> + '_ {
        self.nodes.iter().map(|(k, v)| (*k, v))
    }

    // ---- building (recompute engine) ----------------------------------------------------

    /// Inserts or replaces a body.
    pub fn set_body(&mut self, id: BodyId, body: Arc<EvalBody>) {
        self.bodies.insert(id, body);
    }

    /// Removes a body (deleted by a feature).
    pub fn remove_body(&mut self, id: BodyId) -> Option<Arc<EvalBody>> {
        self.bodies.remove(&id)
    }

    /// Inserts or replaces a sketch.
    pub fn set_sketch(&mut self, id: SketchId, sketch: Arc<SketchGeom>) {
        self.sketches.insert(id, sketch);
    }

    /// Inserts or replaces construction geometry.
    pub fn set_construction(&mut self, id: ConstructionId, geom: ConstructionGeom) {
        self.construction.insert(id, geom);
    }

    /// Records a node's evaluation.
    pub fn set_node(&mut self, id: FeatureId, eval: NodeEval) {
        self.nodes.insert(id, eval);
    }
}

/// One evaluated body.
#[derive(Clone, Debug)]
pub struct EvalBody {
    /// Component the body belongs to.
    pub component: ComponentId,
    /// Geometry.
    pub geom: BodyGeom,
    /// Element map (solid and mesh bodies).
    pub naming: Option<Arc<BodyNaming>>,
}

/// Body geometry by kind. Every payload is a cheap-to-clone shared handle.
#[derive(Clone, Debug)]
pub enum BodyGeom {
    /// B-rep solid or surface body (stub until BREP-02).
    Solid(Shape),
    /// Triangle mesh body (stub until MESH-02).
    Mesh(Arc<IndexedMesh>),
    /// Implicit body (stub until IMP-01).
    Implicit(Arc<SdfGraph>),
    /// SubD form body (stub until FRM-01).
    Form(Arc<SubdMesh>),
}

impl BodyGeom {
    /// True if both refer to the same geometry allocation (no geometry change, DOC-09).
    pub fn same_as(&self, other: &BodyGeom) -> bool {
        match (self, other) {
            (BodyGeom::Solid(a), BodyGeom::Solid(b)) => a.ptr_eq(b),
            (BodyGeom::Mesh(a), BodyGeom::Mesh(b)) => Arc::ptr_eq(a, b),
            (BodyGeom::Implicit(a), BodyGeom::Implicit(b)) => Arc::ptr_eq(a, b),
            (BodyGeom::Form(a), BodyGeom::Form(b)) => Arc::ptr_eq(a, b),
            _ => false,
        }
    }
}

/// Construction geometry output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConstructionGeom {
    /// Construction plane.
    Plane(Plane),
    /// Construction axis.
    Axis(Line3),
    /// Construction point.
    Point(Point3),
}

/// blake3 key of a feature's output (computed by DOC-03).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct CacheKey(pub [u8; 32]);

/// Evaluation record of one timeline node.
#[derive(Clone, Debug, PartialEq)]
pub struct NodeEval {
    /// Outcome.
    pub health: Health,
    /// Cache key (`None` for suppressed / rolled-back nodes).
    pub key: Option<CacheKey>,
    /// Wall time of the compute (zero on a cache hit).
    pub duration: Duration,
}

/// Health of a timeline node (Fusion semantics, S01 §3).
#[derive(Clone, Debug, PartialEq)]
pub enum Health {
    /// Computed without issues.
    Ok,
    /// Computed with caveats.
    Warning(Vec<Diagnostic>),
    /// Failed; the feature has no effect.
    Error(Diagnostic),
    /// Suppressed by the user.
    Suppressed,
    /// After the timeline marker.
    RolledBack,
}

/// A user-facing message attached to a node (not to be confused with the
/// [`amg_core::Diagnostic`] trait, which errors implement).
#[derive(Clone, Debug, PartialEq)]
pub struct Diagnostic {
    /// Short message.
    pub message: String,
    /// Structured cause.
    pub cause: Option<ErrorCause>,
    /// Entities to highlight.
    pub entities: Vec<EntityRef>,
}

/// Structured cause of a node error.
#[derive(Clone, Debug, PartialEq)]
pub enum ErrorCause {
    /// An upstream feature failed, so an input is missing.
    Upstream(FeatureId),
    /// A reference did not resolve.
    Resolve(ResolveError),
    /// A parameter expression failed.
    Expr(ParamId),
    /// The feature kind is not known to this build.
    UnknownKind(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evaluated_skeleton() {
        let mut e = Evaluated::new(Revision(3));
        let id = BodyId {
            origin: FeatureId(1),
            slot: 0,
        };
        let mesh = Arc::new(IndexedMesh::default());
        e.set_body(
            id,
            Arc::new(EvalBody {
                component: ComponentId(1),
                geom: BodyGeom::Mesh(mesh.clone()),
                naming: Some(Arc::new(BodyNaming::default())),
            }),
        );
        e.set_body(
            BodyId {
                origin: FeatureId(2),
                slot: 0,
            },
            Arc::new(EvalBody {
                component: ComponentId(1),
                geom: BodyGeom::Solid(Shape::new()),
                naming: None,
            }),
        );
        e.set_construction(
            ConstructionId(FeatureId(4)),
            ConstructionGeom::Plane(Plane::XY),
        );
        e.set_node(
            FeatureId(2),
            NodeEval {
                health: Health::Error(Diagnostic {
                    message: "boom".into(),
                    cause: Some(ErrorCause::Upstream(FeatureId(1))),
                    entities: vec![EntityRef::Body(id)],
                }),
                key: Some(CacheKey([7; 32])),
                duration: Duration::ZERO,
            },
        );
        assert_eq!(e.revision, Revision(3));
        assert_eq!(e.bodies().count(), 2);
        let b = e.body(id).unwrap();
        assert!(b.geom.same_as(&BodyGeom::Mesh(mesh)));
        assert!(
            !b.geom
                .same_as(&BodyGeom::Mesh(Arc::new(IndexedMesh::default())))
        );
        assert!(matches!(
            e.node(FeatureId(2)).unwrap().health,
            Health::Error(_)
        ));
        assert!(e.construction(ConstructionId(FeatureId(4))).is_some());
        assert!(e.remove_body(id).is_some());
        assert!(e.body(id).is_none());
    }

    #[test]
    fn thread_safety() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Evaluated>();
        assert_send_sync::<Arc<Evaluated>>();
        assert_send_sync::<BodyGeom>();
    }
}
