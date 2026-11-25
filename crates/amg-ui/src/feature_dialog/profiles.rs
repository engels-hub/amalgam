//! Sketch profiles as dialog picks: a CPU hit test of the click ray against the profiles of
//! the visible evaluated sketches, and their shading while a profile field is shown.
//!
//! Profiles have no pick target of their own (the pick map knows whole sketches only), so a
//! profile item is a [`SelItem`] with the sketch as its target and the
//! `EntityRef::Profile` as its entity. It lives in the dialog field, not in the document
//! selection.

use amg_core::{Point2, Point3, Ray3};
use amg_doc::naming::ProfileFp;
use amg_doc::stub::ProfileKey;
use amg_doc::{Design, EntityRef, Evaluated, SketchId};
use amg_render::{DepthMode, OverlayList};

use crate::pick::PickTarget;
use crate::select::SelItem;
use crate::sketch::view;

/// Fill of a picked profile (linear RGBA).
const SELECTED: [f32; 4] = [0.0, 0.70, 1.0, 0.45];
/// Fill of the profile under the cursor.
const HOVER: [f32; 4] = [0.45, 0.85, 1.0, 0.40];

/// The document reference of profile `p` of `sketch`.
pub(crate) fn profile_ref(sketch: SketchId, p: &amg_sketch::Profile) -> EntityRef {
    EntityRef::Profile {
        sketch,
        profile: ProfileKey(p.id.0),
        fingerprint: Some(ProfileFp(p.fingerprint.clone())),
    }
}

/// The dialog item of a profile reference (target: its sketch).
pub(crate) fn item(entity: EntityRef) -> Option<SelItem> {
    let EntityRef::Profile { sketch, .. } = &entity else {
        return None;
    };
    Some(SelItem {
        target: PickTarget::Sketch(*sketch),
        entity,
    })
}

/// Whether `e` is a profile reference.
pub(crate) fn is_profile(e: &EntityRef) -> bool {
    matches!(e, EntityRef::Profile { .. })
}

/// Same sketch and profile id (the fingerprint is matching data, not identity).
pub(crate) fn same(a: &EntityRef, b: &EntityRef) -> bool {
    match (a, b) {
        (
            EntityRef::Profile {
                sketch: s1,
                profile: p1,
                ..
            },
            EntityRef::Profile {
                sketch: s2,
                profile: p2,
                ..
            },
        ) => s1 == s2 && p1 == p2,
        _ => false,
    }
}

/// The visible evaluated sketches.
fn visible<'e>(
    design: &Design,
    eval: &'e Evaluated,
) -> impl Iterator<Item = (SketchId, &'e std::sync::Arc<amg_doc::stub::SketchGeom>)> + 'e {
    let hidden = crate::browser::visibility::hidden(design);
    eval.sketches()
        .filter(move |(id, _)| !hidden.contains(&crate::browser::visibility::sketch_key(*id)))
}

/// The nearest profile along `ray` (in front of its origin) of a visible sketch.
pub(crate) fn at_ray(design: &Design, eval: &Evaluated, ray: &Ray3) -> Option<EntityRef> {
    let d = ray.dir.get();
    let mut best: Option<(f64, EntityRef)> = None;
    for (id, g) in visible(design, eval) {
        let n = g.plane.normal.get();
        let den = d.dot(n);
        if den.abs() < 1e-12 {
            continue;
        }
        let t = (g.plane.origin - ray.origin).dot(n) / den;
        if t < 0.0 || best.as_ref().is_some_and(|(bt, _)| *bt <= t) {
            continue;
        }
        let local: Point2 = g.plane.to_local(ray.origin + d * t);
        if let Some(pid) = g.profiles.at_point(local)
            && let Some(p) = g.profiles.get(pid)
        {
            best = Some((t, profile_ref(id, p)));
        }
    }
    best.map(|(_, e)| e)
}

/// Shades every profile of the visible sketches: picked ones, the hovered one and the rest.
pub(crate) fn draw(
    out: &mut OverlayList,
    design: &Design,
    eval: &Evaluated,
    picked: &[&EntityRef],
    hover: Option<&EntityRef>,
) {
    for (id, g) in visible(design, eval) {
        for p in &g.profiles.profiles {
            let r = profile_ref(id, p);
            let fill = if picked.iter().any(|q| same(q, &r)) {
                SELECTED
            } else if hover.is_some_and(|h| same(h, &r)) {
                HOVER
            } else {
                view::colors::PROFILE
            };
            let mut poly: Vec<Point2> = Vec::new();
            for e in &p.outer.edges {
                let pts = view::sample(&e.curve);
                poly.extend_from_slice(&pts[..pts.len().saturating_sub(1)]);
            }
            let tris = view::triangulate(&poly);
            let w: Vec<Point3> = poly.iter().map(|q| g.plane.to_world(*q)).collect();
            out.polygon(&w, &tris, fill, DepthMode::AlwaysOnTop, None);
        }
    }
}
