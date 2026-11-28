//! Physical properties of mesh bodies (S05 INS-03), in world space.

use std::sync::Arc;

use amg_core::{Aabb3, Density, Point3, Transform, Vec3};
use amg_doc::{BodyGeom, BodyId, Design, Evaluated};
use amg_features::inspect::InspectError;
use amg_features::inspect::props::mesh_mass_props;
use amg_jobs::{CancelToken, JobCtx, Priority};
use amg_mesh::{IndexedMesh, MeshBody};
use serde::Serialize;

use super::{BodyFilter, REPORT_SCHEMA, body_name, explicit_bodies};
use crate::ScriptError;
use crate::host::ScriptDoc;

/// Properties of one placed body, or the total of all rows. Values that cannot be computed
/// are `null`.
#[derive(Clone, Debug, Serialize)]
pub struct BodyProps {
    /// The body; `None` for the total.
    pub id: Option<BodyId>,
    /// Its name (`"total"` for the total).
    pub name: String,
    /// `"mesh"` or `"total"`.
    pub kind: &'static str,
    /// Enclosed volume; `None` unless closed and consistently oriented.
    pub volume_mm3: Option<f64>,
    /// Surface area.
    pub area_mm2: f64,
    /// Mass from the body's assigned material; `None` without one or without a volume.
    pub mass_g: Option<f64>,
    /// Centre of mass (uniform density; the total weighs by mass, or by volume if a row has
    /// no mass).
    pub centroid: Option<[f64; 3]>,
    /// Bounding box minimum; `None` for a body without vertices.
    pub bbox_min: Option<[f64; 3]>,
    /// Bounding box maximum.
    pub bbox_max: Option<[f64; 3]>,
    /// Closed, consistently oriented and enclosing a positive volume.
    pub watertight: Option<bool>,
}

/// Result of [`properties`].
#[derive(Clone, Debug, Serialize)]
pub struct PropsReport {
    /// [`REPORT_SCHEMA`].
    pub schema: u32,
    /// One row per placement of each selected body, in body order.
    pub bodies: Vec<BodyProps>,
    /// Sum over the rows; `None` without rows.
    pub total: Option<BodyProps>,
}

/// One placed mesh body to measure.
struct Item {
    id: BodyId,
    mesh: Arc<IndexedMesh>,
    world: Transform,
    density: Option<Density>,
}

fn items(design: &Design, eval: &Evaluated, filter: &BodyFilter) -> Result<Vec<Item>, ScriptError> {
    let explicit = explicit_bodies(design, eval, filter)?;
    let visible_only = explicit.is_none();
    let ids = explicit.unwrap_or_else(|| {
        eval.bodies()
            .map(|(id, _)| id)
            .filter(|id| design.body_props(*id).visible)
            .collect()
    });
    let mut out = Vec::new();
    for id in ids {
        let Some(body) = eval.body(id) else { continue };
        let placements = eval.instances(id).into_iter().filter(|i| {
            !visible_only
                || i.path
                    .iter()
                    .all(|o| design.occurrence(*o).is_none_or(|o| o.visible))
        });
        for placement in placements {
            let what = match &body.geom {
                BodyGeom::Mesh(mesh) => {
                    out.push(Item {
                        id,
                        mesh: Arc::clone(mesh),
                        world: placement.world,
                        density: design.body_material(id).map(|m| m.density),
                    });
                    continue;
                }
                BodyGeom::Solid(_) => "solid properties",
                BodyGeom::Implicit(_) | BodyGeom::Form(_) => "implicit or form body properties",
            };
            return Err(ScriptError::NotFound {
                what,
                name: body_name(design, id),
            });
        }
    }
    Ok(out)
}

fn measure_item(design: &Design, item: &Item, ctx: &JobCtx) -> Result<BodyProps, InspectError> {
    let body = MeshBody::from_arc(Arc::clone(&item.mesh));
    // The density does not change the volume or the centre of mass.
    let density = item.density.unwrap_or(Density::ZERO);
    let p = mesh_mass_props(&body, &item.world, density, ctx)?;
    let mut bbox = Aabb3::EMPTY;
    for v in &item.mesh.positions {
        bbox.include_point(item.world.transform_point3(*v));
    }
    let has_box = !item.mesh.positions.is_empty();
    Ok(BodyProps {
        id: Some(item.id),
        name: body_name(design, item.id),
        kind: "mesh",
        volume_mm3: p.volume.map(|v| v.get()),
        area_mm2: p.area.get(),
        mass_g: item.density.and(p.mass).map(|m| m.to_g()),
        centroid: p.center_of_mass.map(|c| c.to_array()),
        bbox_min: has_box.then(|| bbox.min.to_array()),
        bbox_max: has_box.then(|| bbox.max.to_array()),
        watertight: Some(p.not_watertight.is_none()),
    })
}

/// Sum of the values if every row has one.
fn all_sum(rows: &[BodyProps], value: impl Fn(&BodyProps) -> Option<f64>) -> Option<f64> {
    rows.iter().map(value).sum()
}

/// Mean of the centroids weighted by `weight`, if every row has both and the weights are
/// positive in sum.
fn weighted_centroid(
    rows: &[BodyProps],
    weight: impl Fn(&BodyProps) -> Option<f64>,
) -> Option<[f64; 3]> {
    let mut sum = Vec3::ZERO;
    let mut total = 0.0;
    for r in rows {
        let w = weight(r)?;
        sum += Point3::from_array(r.centroid?) * w;
        total += w;
    }
    (total > 0.0).then(|| (sum / total).to_array())
}

fn total_row(rows: &[BodyProps]) -> Option<BodyProps> {
    if rows.is_empty() {
        return None;
    }
    let fold = |get: fn(&BodyProps) -> Option<[f64; 3]>, pick: fn(Vec3, Vec3) -> Vec3| {
        rows.iter()
            .filter_map(get)
            .map(Vec3::from_array)
            .reduce(pick)
            .map(|v| v.to_array())
    };
    Some(BodyProps {
        id: None,
        name: "total".to_owned(),
        kind: "total",
        volume_mm3: all_sum(rows, |r| r.volume_mm3),
        area_mm2: rows.iter().map(|r| r.area_mm2).sum(),
        mass_g: all_sum(rows, |r| r.mass_g),
        centroid: weighted_centroid(rows, |r| r.mass_g)
            .or_else(|| weighted_centroid(rows, |r| r.volume_mm3)),
        bbox_min: fold(|r| r.bbox_min, Vec3::min),
        bbox_max: fold(|r| r.bbox_max, Vec3::max),
        watertight: Some(rows.iter().all(|r| r.watertight == Some(true))),
    })
}

/// Volume, area, mass, centroid and bounding box of mesh bodies, recomputing first if needed.
///
/// An open mesh is not an error: its row has `watertight: false` and no volume.
///
/// # Errors
/// [`ScriptError::NotFound`] for an unknown body and for a selected solid (`"solid
/// properties"`), implicit or form body; [`ScriptError::Inspect`] for degenerate geometry;
/// [`ScriptError::Cancelled`].
pub fn properties(
    doc: &mut ScriptDoc,
    bodies: &BodyFilter,
    cancel: &CancelToken,
) -> Result<PropsReport, ScriptError> {
    let eval = doc.evaluated(cancel)?;
    let design = doc.document().snapshot();
    let items = items(&design, &eval, bodies)?;
    let jobs = Arc::clone(doc.jobs());
    let rows = jobs.install(Priority::Interactive, || {
        let ctx = JobCtx::with_token(cancel.clone());
        items
            .iter()
            .map(|item| measure_item(&design, item, &ctx))
            .collect::<Result<Vec<_>, _>>()
    })?;
    Ok(PropsReport {
        schema: REPORT_SCHEMA,
        total: total_row(&rows),
        bodies: rows,
    })
}
