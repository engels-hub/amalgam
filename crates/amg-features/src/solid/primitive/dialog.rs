//! Dialog registrations of the four primitives (S11 §B schema, rendered by UI-05): plane
//! (origin or a picked construction plane / planar face), offsets, rotation, the sizes (one
//! distance manipulator each) and the body operation.

use std::sync::OnceLock;

use amg_core::{Dir3, Plane, Point3, Vec3};
use amg_doc::{
    ConstructionGeom, Design, Dim, EntityRef, Evaluated, Expr, Feature, FeatureBox, TopoKind,
};

use super::{BoxPrim, CylinderPrim, PrimitiveFrame, SpherePrim, TorusPrim, placed};
use crate::construct::resolve::resolve_in;
use crate::construct::{OriginEntity, RefInput, math};
use crate::dialog::{
    CommandMeta, DialogInit, DialogRegistration, DialogSchema, EntityFilter, FieldErrors, FieldKey,
    FieldKind, FieldSpec, FieldValue, FieldValues, ManipAnchor, ManipKind,
};
use crate::solid::common::OpTarget;
use crate::solid::extrude::dialog::{OPS, choice, expr, is, not, take_choice, take_expr};
use crate::solid::pattern::dialog::{axis_values, read_axis};
use crate::solid::shell::dialog::{bodies_value, body_ids, preselected_bodies};

const PLANE: &str = "plane";
const PLANE_REF: &str = "plane_ref";
const OFFSET_X: &str = "offset_x";
const OFFSET_Y: &str = "offset_y";
const ROTATION: &str = "rotation";
const CENTERED: &str = "centered";
const OPERATION: &str = "operation";
const BODIES: &str = "bodies";

const LENGTH: &str = "length";
const WIDTH: &str = "width";
const HEIGHT: &str = "height";
const RADIUS: &str = "radius";
const MAJOR: &str = "major_radius";
const MINOR: &str = "minor_radius";

// Option labels differ from every field label (the UI harness finds fields by label).
const PLANES: [(&str, &str); 4] = [
    ("xy", "XY plane"),
    ("yz", "YZ plane"),
    ("xz", "XZ plane"),
    ("picked", "Selected plane"),
];

const ZERO_MM: &str = "0 mm";
const ZERO_DEG: &str = "0 deg";

/// A size field: key, label and default text.
type SizeSpec = (&'static str, &'static str, &'static str);

const BOX_SIZES: [SizeSpec; 3] = [
    (LENGTH, "Length", "10 mm"),
    (WIDTH, "Width", "10 mm"),
    (HEIGHT, "Height", "10 mm"),
];
const CYLINDER_SIZES: [SizeSpec; 2] = [(RADIUS, "Radius", "5 mm"), (HEIGHT, "Height", "10 mm")];
const SPHERE_SIZES: [SizeSpec; 1] = [(RADIUS, "Radius", "5 mm")];
const TORUS_SIZES: [SizeSpec; 2] = [
    (MAJOR, "Major radius", "10 mm"),
    (MINOR, "Minor radius", "2 mm"),
];

// ---- schema ---------------------------------------------------------------------------------

fn schema_of(kind: &'static str, title: &str, sizes: &[SizeSpec], centered: bool) -> DialogSchema {
    let plane = choice(PLANE, "Plane", &PLANES);
    let mut plane_ref = FieldSpec::new(
        PLANE_REF,
        "Plane reference",
        FieldKind::Selection {
            filter: EntityFilter::CONSTRUCTION_PLANE | EntityFilter::PLANAR_FACE,
            min: 1,
            max: Some(1),
        },
    );
    plane_ref.tooltip = Some("Construction plane or planar face the primitive sits on.".into());
    plane_ref.visible_if = Some(is(PLANE, "picked"));
    let mut fields = vec![
        plane,
        plane_ref,
        expr(OFFSET_X, "Offset X", Dim::LENGTH, ZERO_MM),
        expr(OFFSET_Y, "Offset Y", Dim::LENGTH, ZERO_MM),
        expr(ROTATION, "Rotation", Dim::ANGLE, ZERO_DEG),
    ];
    for &(key, label, default) in sizes {
        let mut f = expr(key, label, Dim::LENGTH, default);
        f.manipulator = Some(ManipKind::Distance);
        fields.push(f);
    }
    if centered {
        let mut f = FieldSpec::new(CENTERED, "Centered", FieldKind::Bool);
        f.tooltip = Some("Centre the length and width on the placement point.".into());
        fields.push(f);
    }
    let operation = choice(
        OPERATION,
        "Operation",
        &OPS.map(|(id, _, label)| (id, label)),
    );
    let mut bodies = FieldSpec::new(
        BODIES,
        "Objects",
        FieldKind::Selection {
            filter: EntityFilter::BODY,
            min: 0,
            max: None,
        },
    );
    bodies.tooltip = Some("Bodies the operation applies to.".into());
    bodies.visible_if = Some(not(is(OPERATION, "new_body")));
    fields.extend([operation, bodies]);
    DialogSchema {
        kind,
        title: title.to_owned(),
        help: Some(kind),
        fields,
    }
}

// ---- values ---------------------------------------------------------------------------------

/// The values of the frame, sizes and body operation.
fn values_of(
    frame: &PrimitiveFrame,
    sizes: &[SizeSpec],
    exprs: &[&Expr],
    target: &OpTarget,
    design: &Design,
) -> FieldValues {
    let (plane, plane_ref) = axis_values(&frame.plane, "xy");
    let op = OPS
        .iter()
        .find(|(_, o, _)| *o == target.op)
        .map_or("new_body", |(id, _, _)| id);
    let mut v = FieldValues::default()
        .with(PLANE, FieldValue::Choice(plane))
        .with(PLANE_REF, FieldValue::Selection(plane_ref))
        .with(OFFSET_X, FieldValue::from_expr(&frame.u, design))
        .with(OFFSET_Y, FieldValue::from_expr(&frame.v, design))
        .with(ROTATION, FieldValue::from_expr(&frame.angle, design));
    for (&(key, _, _), e) in sizes.iter().zip(exprs) {
        v = v.with(key, FieldValue::from_expr(e, design));
    }
    v.with(OPERATION, FieldValue::Choice(op))
        .with(BODIES, bodies_value(&target.participants))
}

/// The preselected planar face or construction plane, else the XY plane.
fn preselected_frame(init: &DialogInit<'_>) -> PrimitiveFrame {
    let plane = init
        .preselection
        .iter()
        .find(|e| match e {
            EntityRef::Topo(t) => t.kind == TopoKind::Face,
            EntityRef::Construction(c) => matches!(
                init.evaluated.construction(*c),
                Some(ConstructionGeom::Plane(_))
            ),
            _ => false,
        })
        .cloned()
        .map_or(RefInput::Origin(OriginEntity::XYPlane), RefInput::Ref);
    PrimitiveFrame::on(plane)
}

fn preselected_target(init: &DialogInit<'_>) -> OpTarget {
    OpTarget {
        participants: preselected_bodies(init),
        ..OpTarget::default()
    }
}

/// The frame, sizes and body operation from the fields; `None` (with errors) when one is
/// invalid.
fn build_parts<const N: usize>(
    v: &FieldValues,
    sizes: &[SizeSpec; N],
) -> Result<(PrimitiveFrame, [Expr; N], OpTarget), FieldErrors> {
    let mut errs = FieldErrors::default();
    let plane = read_axis(v, PLANE, PLANE_REF, "xy")
        .map_err(|e| errs.0.push(e))
        .ok();
    let u = take_expr(v, OFFSET_X, &mut errs);
    let v_ = take_expr(v, OFFSET_Y, &mut errs);
    let angle = take_expr(v, ROTATION, &mut errs);
    let size: Vec<Expr> = sizes
        .iter()
        .filter_map(|&(key, _, _)| take_expr(v, key, &mut errs))
        .collect();
    let id = take_choice(v, OPERATION, "new_body", &mut errs);
    let op = OPS.iter().find(|(o, _, _)| *o == id).map(|(_, op, _)| *op);
    if op.is_none() {
        errs.0
            .push((FieldKey(OPERATION), format!("unknown operation `{id}`")));
    }
    let participants = body_ids(v, BODIES, &mut errs);
    let size: Option<[Expr; N]> = size.try_into().ok();
    match (plane, u, v_, angle, size, op) {
        (Some(plane), Some(u), Some(v), Some(angle), Some(size), Some(op)) if errs.is_empty() => {
            Ok((
                PrimitiveFrame { plane, u, v, angle },
                size,
                OpTarget { op, participants },
            ))
        }
        _ => Err(errs),
    }
}

// ---- manipulator anchors --------------------------------------------------------------------

/// The value of a length or angle expression outside a recompute.
fn value(design: &Design, e: &Expr, dim: Dim) -> Option<f64> {
    design
        .params()
        .resolve(e, dim, &design.meta().default_units)
        .ok()
        .map(|q| q.value)
        .filter(|v| v.is_finite())
}

/// The placed frame from the evaluated design (world space of the root component).
fn frame_of(f: &PrimitiveFrame, design: &Design, eval: &Evaluated) -> Option<Plane> {
    let r = resolve_in(eval, design, &f.plane).ok()?;
    let base = math::plane_of(&r.geom).ok()?;
    Some(placed(
        &base,
        value(design, &f.u, Dim::LENGTH)?,
        value(design, &f.v, Dim::LENGTH)?,
        value(design, &f.angle, Dim::ANGLE)?,
    ))
}

/// A distance arrow whose value is zero at `origin`.
fn arrow(field: &'static str, origin: Point3, axis: Vec3) -> ManipAnchor {
    ManipAnchor {
        field: FieldKey(field),
        origin,
        axis: Dir3::new_unchecked(axis),
        ref_dir: None,
        base: 0.0,
    }
}

fn box_anchors(f: &dyn Feature, design: &Design, eval: &Evaluated) -> Vec<ManipAnchor> {
    let build = || {
        let b = f.as_any().downcast_ref::<BoxPrim>()?;
        let p = frame_of(&b.frame, design, eval)?;
        let length = value(design, &b.size[0], Dim::LENGTH)?;
        let width = value(design, &b.size[1], Dim::LENGTH)?;
        let (x, y) = (p.x.get(), p.y.get());
        // The centre of the footprint.
        let c = if b.centered {
            p.origin
        } else {
            p.origin + x * (length / 2.0) + y * (width / 2.0)
        };
        Some(vec![
            // Each arrow starts on the side face its size is measured from.
            arrow(LENGTH, c - x * (length / 2.0), x),
            arrow(WIDTH, c - y * (width / 2.0), y),
            arrow(HEIGHT, c, p.normal.get()),
        ])
    };
    build().unwrap_or_default()
}

fn cylinder_anchors(f: &dyn Feature, design: &Design, eval: &Evaluated) -> Vec<ManipAnchor> {
    let build = || {
        let c = f.as_any().downcast_ref::<CylinderPrim>()?;
        let p = frame_of(&c.frame, design, eval)?;
        Some(vec![
            arrow(RADIUS, p.origin, p.x.get()),
            arrow(HEIGHT, p.origin, p.normal.get()),
        ])
    };
    build().unwrap_or_default()
}

fn sphere_anchors(f: &dyn Feature, design: &Design, eval: &Evaluated) -> Vec<ManipAnchor> {
    let build = || {
        let s = f.as_any().downcast_ref::<SpherePrim>()?;
        let p = frame_of(&s.frame, design, eval)?;
        Some(vec![arrow(RADIUS, p.origin, p.x.get())])
    };
    build().unwrap_or_default()
}

fn torus_anchors(f: &dyn Feature, design: &Design, eval: &Evaluated) -> Vec<ManipAnchor> {
    let build = || {
        let t = f.as_any().downcast_ref::<TorusPrim>()?;
        let p = frame_of(&t.frame, design, eval)?;
        let major = value(design, &t.major, Dim::LENGTH)?;
        Some(vec![
            arrow(MAJOR, p.origin, p.x.get()),
            // From the centre of the tube's cross-section, along the axis.
            arrow(MINOR, p.origin + p.x.get() * major, p.normal.get()),
        ])
    };
    build().unwrap_or_default()
}

// ---- box ------------------------------------------------------------------------------------

fn box_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| schema_of("solid.box", "Box", &BOX_SIZES, true))
}

fn box_values(b: &BoxPrim, design: &Design) -> FieldValues {
    let sizes: Vec<&Expr> = b.size.iter().collect();
    values_of(&b.frame, &BOX_SIZES, &sizes, &b.target, design)
        .with(CENTERED, FieldValue::Bool(b.centered))
}

fn box_defaults(init: &DialogInit<'_>) -> FieldValues {
    let b = BoxPrim::new(
        preselected_frame(init),
        BOX_SIZES.map(|s| s.2),
        preselected_target(init),
    );
    box_values(&b, init.design)
}

fn box_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<BoxPrim>()
        .map(|b| box_values(b, design))
}

fn box_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let parts = build_parts(v, &BOX_SIZES);
    let centered = v.flag(CENTERED);
    match (parts, centered) {
        (Ok((frame, size, target)), Ok(centered)) => Ok(FeatureBox::new(BoxPrim {
            frame,
            size,
            centered,
            target,
        })),
        (parts, centered) => {
            let mut errs = parts.err().unwrap_or_default();
            errs.0.extend(centered.err());
            Err(errs)
        }
    }
}

// ---- cylinder -------------------------------------------------------------------------------

fn cylinder_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| schema_of("solid.cylinder", "Cylinder", &CYLINDER_SIZES, false))
}

fn cylinder_values(c: &CylinderPrim, design: &Design) -> FieldValues {
    let sizes = [&c.radius, &c.height];
    values_of(&c.frame, &CYLINDER_SIZES, &sizes, &c.target, design)
}

fn cylinder_defaults(init: &DialogInit<'_>) -> FieldValues {
    let [r, h] = CYLINDER_SIZES.map(|s| s.2);
    let c = CylinderPrim::new(preselected_frame(init), r, h, preselected_target(init));
    cylinder_values(&c, init.design)
}

fn cylinder_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<CylinderPrim>()
        .map(|c| cylinder_values(c, design))
}

fn cylinder_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let (frame, [radius, height], target) = build_parts(v, &CYLINDER_SIZES)?;
    Ok(FeatureBox::new(CylinderPrim {
        frame,
        radius,
        height,
        target,
    }))
}

// ---- sphere ---------------------------------------------------------------------------------

fn sphere_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| schema_of("solid.sphere", "Sphere", &SPHERE_SIZES, false))
}

fn sphere_values(s: &SpherePrim, design: &Design) -> FieldValues {
    values_of(&s.frame, &SPHERE_SIZES, &[&s.radius], &s.target, design)
}

fn sphere_defaults(init: &DialogInit<'_>) -> FieldValues {
    let [r] = SPHERE_SIZES.map(|s| s.2);
    let s = SpherePrim::new(preselected_frame(init), r, preselected_target(init));
    sphere_values(&s, init.design)
}

fn sphere_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<SpherePrim>()
        .map(|s| sphere_values(s, design))
}

fn sphere_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let (frame, [radius], target) = build_parts(v, &SPHERE_SIZES)?;
    Ok(FeatureBox::new(SpherePrim {
        frame,
        radius,
        target,
    }))
}

// ---- torus ----------------------------------------------------------------------------------

fn torus_schema() -> &'static DialogSchema {
    static S: OnceLock<DialogSchema> = OnceLock::new();
    S.get_or_init(|| schema_of("solid.torus", "Torus", &TORUS_SIZES, false))
}

fn torus_values(t: &TorusPrim, design: &Design) -> FieldValues {
    let sizes = [&t.major, &t.minor];
    values_of(&t.frame, &TORUS_SIZES, &sizes, &t.target, design)
}

fn torus_defaults(init: &DialogInit<'_>) -> FieldValues {
    let [major, minor] = TORUS_SIZES.map(|s| s.2);
    let t = TorusPrim::new(
        preselected_frame(init),
        major,
        minor,
        preselected_target(init),
    );
    torus_values(&t, init.design)
}

fn torus_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    f.as_any()
        .downcast_ref::<TorusPrim>()
        .map(|t| torus_values(t, design))
}

fn torus_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let (frame, [major, minor], target) = build_parts(v, &TORUS_SIZES)?;
    Ok(FeatureBox::new(TorusPrim {
        frame,
        major,
        minor,
        target,
    }))
}

// ---- registrations --------------------------------------------------------------------------

/// Create group, after revolve and before the patterns; not pinned.
const fn command(order: i32, keywords: &'static [&'static str]) -> CommandMeta {
    CommandMeta {
        workspace: "design",
        tab: "solid",
        group: "create",
        order,
        pinned: false,
        icon_svg: &[],
        shortcut: None,
        keywords,
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.box",
        schema: box_schema,
        defaults: box_defaults,
        read: box_read,
        build: box_build,
        anchors: box_anchors,
        command: command(40, &["box", "cube", "block", "cuboid", "primitive"]),
        new_component: None,
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.cylinder",
        schema: cylinder_schema,
        defaults: cylinder_defaults,
        read: cylinder_read,
        build: cylinder_build,
        anchors: cylinder_anchors,
        command: command(41, &["cylinder", "rod", "disc", "primitive"]),
        new_component: None,
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.sphere",
        schema: sphere_schema,
        defaults: sphere_defaults,
        read: sphere_read,
        build: sphere_build,
        anchors: sphere_anchors,
        command: command(42, &["sphere", "ball", "primitive"]),
        new_component: None,
    }
}

inventory::submit! {
    DialogRegistration {
        kind: "solid.torus",
        schema: torus_schema,
        defaults: torus_defaults,
        read: torus_read,
        build: torus_build,
        anchors: torus_anchors,
        command: command(43, &["torus", "ring", "donut", "primitive"]),
        new_component: None,
    }
}
