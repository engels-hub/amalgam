//! Mixed selections and totals (INS-10): placed mesh and solid bodies measured in one run,
//! and their totals per group (component, occurrence).

use std::collections::BTreeMap;

use amg_brep::Shape;
use amg_core::{Density, Diagnostic, Transform};
use amg_jobs::JobCtx;
use amg_mesh::MeshBody;

use super::{MassProps, combine, mesh_mass_props, solid_mass_props, transform_mass_props};
use crate::inspect::InspectError;

/// The geometry of one placed body.
#[derive(Clone, Debug)]
pub enum PropsGeom {
    /// A mesh body.
    Mesh(MeshBody),
    /// A solid (B-rep) body.
    Solid(Shape),
    /// A solid body without B-rep geometry (a build without the kernel).
    MissingSolid,
}

impl PropsGeom {
    /// A solid body, with its B-rep shape if it has one.
    pub fn solid(shape: Option<Shape>) -> PropsGeom {
        shape.map_or(PropsGeom::MissingSolid, PropsGeom::Solid)
    }
}

/// One body instance to measure.
#[derive(Clone, Debug)]
pub struct PropsItem {
    /// Geometry in body (component) space.
    pub geom: PropsGeom,
    /// Body to world.
    pub to_world: Transform,
    /// Density, t/mm³.
    pub density: Density,
}

/// The properties of every item, in order. A body that cannot be measured (a solid without the
/// kernel, a non-rigid solid placement) gives its own error and leaves the others alone. A
/// solid shown several times is integrated once and moved to each placement.
///
/// # Errors
/// [`InspectError::Cancelled`] only.
pub fn items_mass_props(
    items: &[PropsItem],
    ctx: &JobCtx,
) -> Result<Vec<Result<MassProps, InspectError>>, InspectError> {
    // Local-space results of the solids measured so far, with the index of their item.
    let mut solids: Vec<(usize, MassProps)> = Vec::new();
    let mut out = Vec::with_capacity(items.len());
    for (i, item) in items.iter().enumerate() {
        ctx.checkpoint()?;
        let sub = ctx.sub(
            i as f32 / items.len() as f32,
            (i + 1) as f32 / items.len() as f32,
        );
        let r = match &item.geom {
            PropsGeom::Mesh(m) => mesh_mass_props(m, &item.to_world, item.density, &sub),
            PropsGeom::Solid(shape) => {
                let known = solids.iter().find(|(j, _)| {
                    items[*j].density == item.density
                        && matches!(&items[*j].geom, PropsGeom::Solid(s) if s.ptr_eq(shape))
                });
                match known {
                    Some((_, local)) => transform_mass_props(local, &item.to_world),
                    None => solid_mass_props(shape, &Transform::IDENTITY, item.density, &sub)
                        .and_then(|local| {
                            let placed = transform_mass_props(&local, &item.to_world);
                            solids.push((i, local));
                            placed
                        }),
                }
            }
            PropsGeom::MissingSolid if !amg_brep::OCCT_BACKEND => {
                Err(InspectError::BackendUnavailable)
            }
            PropsGeom::MissingSolid => Err(InspectError::Brep(
                "the solid body has no B-rep geometry".into(),
            )),
        };
        match r {
            Err(e) if e.is_cancelled() => return Err(InspectError::Cancelled),
            r => out.push(r),
        }
    }
    Ok(out)
}

/// The total of one group.
#[derive(Clone, Debug, PartialEq)]
pub struct GroupTotal<K> {
    /// The group.
    pub key: K,
    /// Indices (into the input) of the measured members, ascending.
    pub members: Vec<usize>,
    /// Members that could not be measured and are not in the total.
    pub skipped: usize,
    /// [`combine`] of the measured members; `None` if there are none.
    pub total: Option<MassProps>,
}

/// Totals per group. `parts[i]` lists the groups part `i` belongs to (for assemblies: its
/// component and occurrence chain) and its properties (`None`: it could not be measured).
/// Groups come in key order, members in input order.
pub fn group_totals<K: Ord + Clone>(parts: &[(Vec<K>, Option<&MassProps>)]) -> Vec<GroupTotal<K>> {
    let mut groups: BTreeMap<&K, (Vec<usize>, usize)> = BTreeMap::new();
    for (i, (keys, props)) in parts.iter().enumerate() {
        for k in keys {
            let g = groups.entry(k).or_default();
            if props.is_some() {
                g.0.push(i);
            } else {
                g.1 += 1;
            }
        }
    }
    groups
        .into_iter()
        .map(|(key, (members, skipped))| {
            let measured: Vec<MassProps> = members
                .iter()
                .filter_map(|&i| parts[i].1.cloned())
                .collect();
            GroupTotal {
                key: key.clone(),
                total: (!measured.is_empty()).then(|| combine(&measured)),
                members,
                skipped,
            }
        })
        .collect()
}
