//! Mass properties of B-rep bodies (INS-10) through S02 `mass_props` (BREP-09), and their
//! placement by an occurrence transform.

use std::sync::OnceLock;

use amg_brep::{BrepKernel, KernelConfig, KernelError, OcctKernel, Shape};
use amg_core::{Area, Density, Mass, Mat3, Transform, Volume};
use amg_jobs::JobCtx;

use super::{MassProps, parallel_axis, principal};
use crate::inspect::InspectError;
use crate::inspect::measure::Exactness;

/// Orthonormality tolerance of a placement (the same as S01's occurrence rule).
const ISOMETRY_TOL: f64 = 1e-9;

fn kernel() -> &'static OcctKernel {
    static KERNEL: OnceLock<OcctKernel> = OnceLock::new();
    KERNEL.get_or_init(|| OcctKernel::new(KernelConfig::default()))
}

fn brep_error(e: &KernelError) -> InspectError {
    match e {
        KernelError::BackendUnavailable => InspectError::BackendUnavailable,
        other => InspectError::Brep(other.to_string()),
    }
}

/// Mass properties of the solids of `shape` in its own space, with `density` (t/mm³).
fn local_props(shape: &Shape, density: Density) -> Result<MassProps, InspectError> {
    let k = kernel().mass_props(shape).map_err(|e| brep_error(&e))?;
    let area = Area::mm2(k.area);
    let reason = if shape.solids().is_empty() {
        Some("the body is not a solid (a surface body encloses no volume)")
    } else if k.volume < 0.0 {
        Some("the solid is inverted (negative volume)")
    } else if k.volume == 0.0 {
        Some("the solid encloses no volume (an open shell)")
    } else {
        None
    };
    if let Some(reason) = reason {
        return Ok(MassProps {
            area,
            volume: None,
            density,
            mass: None,
            center_of_mass: None,
            inertia_com: None,
            inertia_origin: None,
            principal_moments: None,
            principal_axes: None,
            not_watertight: Some(reason.to_owned()),
            exactness: Exactness::Exact,
        });
    }
    let rho = density.get();
    let mass = rho * k.volume;
    let com = k.centroid;
    let i_com = k.inertia * rho;
    let (moments, axes) = principal(&i_com);
    Ok(MassProps {
        area,
        volume: Some(Volume::mm3(k.volume)),
        density,
        mass: Some(Mass::from_canonical(mass)),
        center_of_mass: Some(com),
        inertia_com: Some(i_com),
        inertia_origin: Some(i_com + parallel_axis(mass, com)),
        principal_moments: Some(moments),
        principal_axes: Some(axes),
        not_watertight: None,
        exactness: Exactness::Exact,
    })
}

/// True for a finite rotation or rotation-with-mirror plus translation.
fn is_isometry(t: &Transform) -> bool {
    let m = t.matrix3;
    if !(m.is_finite() && t.translation.is_finite()) {
        return false;
    }
    let rtr: Mat3 = m.transpose() * m;
    (0..3).all(|c| (rtr.col(c) - Mat3::IDENTITY.col(c)).abs().max_element() <= ISOMETRY_TOL)
}

/// `props` moved by the placement `t` (rigid or mirrored, as occurrence transforms are):
/// area, volume, mass and principal moments are unchanged, the centre of mass maps through `t`,
/// the inertia about it is `R·I·Rᵀ` and the tensor about the origin follows by the
/// parallel-axis rule.
///
/// # Errors
/// [`InspectError::NonRigidTransform`] for scale, shear or non-finite values.
pub fn transform_mass_props(props: &MassProps, t: &Transform) -> Result<MassProps, InspectError> {
    if !is_isometry(t) {
        return Err(InspectError::NonRigidTransform);
    }
    let mut out = props.clone();
    let (Some(mass), Some(c), Some(i)) = (props.mass, props.center_of_mass, props.inertia_com)
    else {
        return Ok(out);
    };
    let r = t.matrix3;
    let c = t.transform_point3(c);
    let i = r * i * r.transpose();
    let (moments, axes) = principal(&i);
    out.center_of_mass = Some(c);
    out.inertia_com = Some(i);
    out.inertia_origin = Some(i + parallel_axis(mass.get(), c));
    out.principal_moments = Some(moments);
    out.principal_axes = Some(axes);
    Ok(out)
}

/// Exact mass properties of a B-rep body placed by `to_world` (rigid or mirrored), with
/// `density` (t/mm³). The integration is S02's adaptive `mass_props`, done once in the shape's
/// own space and moved by [`transform_mass_props`].
///
/// A shape without a solid, or whose solids enclose no volume, gets its area and a reason in
/// [`MassProps::not_watertight`], as an open mesh does. The integration is one kernel call, so
/// cancellation is checked before and after it.
///
/// # Errors
/// [`InspectError::BackendUnavailable`] without the OCCT backend,
/// [`InspectError::NonRigidTransform`], [`InspectError::Brep`] for a kernel error and
/// [`InspectError::Cancelled`].
pub fn solid_mass_props(
    shape: &Shape,
    to_world: &Transform,
    density: Density,
    ctx: &JobCtx,
) -> Result<MassProps, InspectError> {
    if !is_isometry(to_world) {
        return Err(InspectError::NonRigidTransform);
    }
    ctx.checkpoint()?;
    let local = local_props(shape, density)?;
    ctx.checkpoint()?;
    transform_mass_props(&local, to_world)
}
