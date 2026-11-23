//! The Properties panel (S05 §4, INS-03): volume, area, mass, centre of mass, inertia and
//! principal axes of the selected mesh bodies (all mesh bodies when none is selected), with
//! the density of each body's material (the design's default material, marked
//! "(default material)", when it has none).
//!
//! - Results come from `amg_features::inspect::props` as one cancellable background job
//!   ("Mass properties"); a changed selection, body, placement or material supersedes it.
//! - The panel is opened by [`PROPERTIES_CMD`] (INSPECT ▸ Inspect, SOLID ▸ Inspect, S-key
//!   search) and by the "Properties" entry of the viewport context menu on a mesh body.
//! - The centre-of-mass glyph is drawn through the additive `Panel::overlay` hook (RND-07
//!   rings and a point, always on top).

use std::collections::BTreeSet;
use std::sync::Arc;

use amg_core::{Density, Dir3, Mat3, Point3, Transform};
use amg_doc::{BodyGeom, BodyId, Evaluated};
use amg_features::inspect::InspectError;
use amg_features::inspect::props::{MassProps, combine, mesh_mass_props};
use amg_jobs::{JobCtx, JobError, JobHandle, SpawnOptions};
use amg_materials::{LibrarySet, MaterialPrefs};
use amg_mesh::{IndexedMesh, MeshBody};
use amg_render::{DepthMode, LabelStyle, LineStyle, OverlayList, PointStyle};

use crate::app::{AppCx, CmdEnv};
use crate::command::CmdId;
use crate::design::DESIGN;
use crate::marking::{ContextMenuProvider, ContextTarget};
use crate::module::PanelId;
use crate::panel::{DockSlot, Panel, PanelDef};

/// The panel id.
pub const PROPERTIES_PANEL: PanelId = PanelId("inspect.properties");
/// Opens the Properties panel.
pub const PROPERTIES_CMD: CmdId = CmdId("inspect.properties");
/// Label of the background job.
pub const PROPERTIES_JOB: &str = "Mass properties";

/// Colour of the centre-of-mass glyph (linear RGBA).
const GLYPH: [f32; 4] = [0.95, 0.55, 0.05, 1.0];
/// Radius of the glyph rings, physical pixels.
const GLYPH_RING_PX: f32 = 9.0;

/// One body to measure.
#[derive(Clone, Debug)]
struct Input {
    body: BodyId,
    mesh: Arc<IndexedMesh>,
    world: Transform,
    density: Density,
    material: String,
    defaulted: bool,
}

/// What a result depends on; a change restarts the job.
type Key = Vec<(BodyId, usize, [u64; 12], u64, bool)>;

impl Input {
    fn key(&self) -> (BodyId, usize, [u64; 12], u64, bool) {
        let c = self.world.matrix3.to_cols_array().map(f64::to_bits);
        let t = self.world.translation.to_array().map(f64::to_bits);
        let w = [
            c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8], t[0], t[1], t[2],
        ];
        (
            self.body,
            Arc::as_ptr(&self.mesh) as usize,
            w,
            self.density.get().to_bits(),
            self.defaulted,
        )
    }
}

/// The result for one body.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyEntry {
    /// The body.
    pub body: BodyId,
    /// Its properties in world space.
    pub props: MassProps,
    /// Name of the material whose density was used.
    pub material: String,
    /// The material is the design's default (the body has none).
    pub defaulted: bool,
}

/// The result of one run: per body and the totals.
#[derive(Clone, Debug, PartialEq)]
pub struct PropsResult {
    /// Per body, in id order.
    pub bodies: Vec<BodyEntry>,
    /// The selection total ([`combine`]; equals the body for one body).
    pub total: MassProps,
}

fn compute(inputs: &[Input], ctx: &JobCtx) -> Result<Arc<PropsResult>, InspectError> {
    let mut bodies = Vec::with_capacity(inputs.len());
    for (i, inp) in inputs.iter().enumerate() {
        ctx.checkpoint()?;
        let sub = ctx.sub(
            i as f32 / inputs.len() as f32,
            (i + 1) as f32 / inputs.len() as f32,
        );
        let body = MeshBody::from_arc(Arc::clone(&inp.mesh));
        let props = mesh_mass_props(&body, &inp.world, inp.density, &sub)?;
        bodies.push(BodyEntry {
            body: inp.body,
            props,
            material: inp.material.clone(),
            defaulted: inp.defaulted,
        });
    }
    let parts: Vec<MassProps> = bodies.iter().map(|b| b.props.clone()).collect();
    Ok(Arc::new(PropsResult {
        total: combine(&parts),
        bodies,
    }))
}

// ---- text -------------------------------------------------------------------------------------

/// `v` with `digits` significant digits and no trailing zeros.
pub fn sig(v: f64, digits: usize) -> String {
    if v == 0.0 || !v.is_finite() {
        return if v == 0.0 { "0".into() } else { v.to_string() };
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    let mag = v.abs().log10().floor() as i64;
    let dec = (digits as i64 - 1 - mag).clamp(0, 9) as usize;
    let s = format!("{v:.dec$}");
    if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.').to_owned()
    } else {
        s
    }
}

/// A mass in g, or kg from 1 kg up (canonical t).
pub fn mass_text(t: f64) -> String {
    let g = t * 1.0e6;
    if g.abs() >= 1.0e3 {
        format!("{} kg", sig(g / 1.0e3, 5))
    } else {
        format!("{} g", sig(g, 5))
    }
}

fn point_text(p: Point3) -> String {
    format!("({}, {}, {}) mm", sig(p.x, 5), sig(p.y, 5), sig(p.z, 5))
}

/// kg·mm² from t·mm².
fn kgmm2(v: f64) -> f64 {
    v * 1.0e3
}

fn tensor_rows(m: &Mat3) -> [String; 2] {
    let at = |r: usize, c: usize| kgmm2(m.col(c)[r]);
    [
        format!(
            "{}, {}, {} kg·mm²",
            sig(at(0, 0), 5),
            sig(at(1, 1), 5),
            sig(at(2, 2), 5)
        ),
        format!(
            "{}, {}, {} kg·mm²",
            sig(at(0, 1), 5),
            sig(at(0, 2), 5),
            sig(at(1, 2), 5)
        ),
    ]
}

fn axis_text(d: Dir3) -> String {
    let v = d.get();
    format!("({}, {}, {})", sig(v.x, 4), sig(v.y, 4), sig(v.z, 4))
}

/// A displayed row.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PropRow {
    /// Row label.
    pub label: &'static str,
    /// Value text with its unit.
    pub value: String,
}

fn row(label: &'static str, value: String) -> PropRow {
    PropRow { label, value }
}

/// The rows of `p`: area always, the volume-based rows when the mesh is closed. `note` is
/// appended to the density (e.g. "(default material)").
pub fn rows(p: &MassProps, note: &str) -> Vec<PropRow> {
    let mut out = Vec::new();
    match p.volume {
        Some(v) => out.push(row("Volume", format!("{} mm³", sig(v.get(), 7)))),
        None => out.push(row("Volume", "– (not watertight)".into())),
    }
    out.push(row("Area", format!("{} mm²", sig(p.area.get(), 7))));
    let density = format!("{} g/cm³{note}", sig(p.density.to_g_per_cm3(), 5));
    out.push(row("Density", density));
    if let Some(m) = p.mass {
        out.push(row("Mass", format!("{}{note}", mass_text(m.get()))));
    }
    if let Some(c) = p.center_of_mass {
        out.push(row("Centre of mass", point_text(c)));
    }
    if let Some(i) = &p.inertia_com {
        let [d, o] = tensor_rows(i);
        out.push(row("Inertia about CoM (Ixx, Iyy, Izz)", d));
        out.push(row("Inertia about CoM (Ixy, Ixz, Iyz)", o));
    }
    if let Some(m) = p.principal_moments {
        out.push(row(
            "Principal moments",
            format!(
                "{}, {}, {} kg·mm²",
                sig(kgmm2(m[0]), 5),
                sig(kgmm2(m[1]), 5),
                sig(kgmm2(m[2]), 5)
            ),
        ));
    }
    if let Some(a) = p.principal_axes {
        out.push(row("Principal axes", a.map(axis_text).join(", ")));
    }
    if let Some(i) = &p.inertia_origin {
        let [d, o] = tensor_rows(i);
        out.push(row("Inertia about origin (Ixx, Iyy, Izz)", d));
        out.push(row("Inertia about origin (Ixy, Ixz, Iyz)", o));
    }
    out
}

// ---- gathering --------------------------------------------------------------------------------

fn is_mesh(ev: &Evaluated, body: BodyId) -> bool {
    ev.body(body)
        .is_some_and(|b| matches!(b.geom, BodyGeom::Mesh(_)))
}

/// The panel's bodies: the selected mesh bodies, else every mesh body, in id order.
fn target_bodies(env: &CmdEnv<'_>) -> Vec<BodyId> {
    let ev = env.evaluated;
    let selected: BTreeSet<BodyId> = env
        .selection
        .items()
        .iter()
        .filter_map(|i| i.target.body())
        .filter(|b| is_mesh(ev, *b))
        .collect();
    if selected.is_empty() {
        ev.bodies()
            .filter(|(_, b)| matches!(b.geom, BodyGeom::Mesh(_)))
            .map(|(id, _)| id)
            .collect()
    } else {
        selected.into_iter().collect()
    }
}

fn gather(
    env: &CmdEnv<'_>,
    prefs: &MaterialPrefs,
    libs: &LibrarySet,
) -> Result<Vec<Input>, String> {
    let mut out = Vec::new();
    for body in target_bodies(env) {
        let Some(eb) = env.evaluated.body(body) else {
            continue;
        };
        let BodyGeom::Mesh(mesh) = &eb.geom else {
            continue;
        };
        let m = env.design.mass_material(body, prefs, libs).ok_or_else(|| {
            format!(
                "The default material {} is not available",
                prefs.default_material
            )
        })?;
        let world = env
            .evaluated
            .instances(body)
            .first()
            .map_or(Transform::IDENTITY, |i| i.world);
        out.push(Input {
            body,
            mesh: Arc::clone(mesh),
            world,
            density: m.material.density,
            material: m.material.name.clone(),
            defaulted: m.defaulted,
        });
    }
    Ok(out)
}

/// `"<feature>.<slot>"`.
fn body_label(body: BodyId) -> String {
    format!("{}.{}", body.origin.0, body.slot)
}

// ---- the panel --------------------------------------------------------------------------------

type PropsJob = JobHandle<Arc<PropsResult>, InspectError>;

/// The Properties panel.
pub struct PropertiesPanel {
    libs: LibrarySet,
    prefs: MaterialPrefs,
    key: Option<Key>,
    job: Option<PropsJob>,
    result: Option<Arc<PropsResult>>,
    error: Option<String>,
    /// Draw the centre of mass of the selection total.
    show_total: bool,
    /// Draw the centre of mass of these bodies (several bodies).
    show_bodies: BTreeSet<BodyId>,
}

impl std::fmt::Debug for PropertiesPanel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PropertiesPanel")
            .field("running", &self.job.is_some())
            .field("done", &self.result.is_some())
            .finish_non_exhaustive()
    }
}

/// Creates the panel (`PanelDef::create`).
pub fn create_panel() -> Box<dyn Panel> {
    Box::new(PropertiesPanel {
        libs: LibrarySet::builtin_only(),
        prefs: MaterialPrefs::default(),
        key: None,
        job: None,
        result: None,
        error: None,
        show_total: false,
        show_bodies: BTreeSet::new(),
    })
}

impl PropertiesPanel {
    /// Restarts the job when the inputs changed; collects a finished one.
    fn sync(&mut self, cx: &mut AppCx<'_>) {
        let inputs = gather(&cx.env(), &self.prefs, &self.libs);
        match inputs {
            Err(e) => {
                if let Some(j) = self.job.take() {
                    j.cancel();
                }
                self.key = None;
                self.result = None;
                self.error = Some(e);
            }
            Ok(inputs) => {
                let key: Key = inputs.iter().map(Input::key).collect();
                if self.key.as_ref() != Some(&key) {
                    if let Some(j) = self.job.take() {
                        j.cancel();
                    }
                    self.key = Some(key);
                    self.error = None;
                    self.result = None;
                    if !inputs.is_empty() {
                        self.job = Some(cx.jobs().spawn_fn(
                            PROPERTIES_JOB,
                            SpawnOptions::default(),
                            move |ctx: &JobCtx| compute(&inputs, ctx),
                        ));
                    }
                }
            }
        }
        if let Some(j) = &mut self.job
            && let Some(r) = j.try_take()
        {
            self.job = None;
            match r {
                Ok(a) => self.result = Some(a),
                Err(JobError::Cancelled) => {}
                Err(e) => self.error = Some(e.to_string()),
            }
        }
        if self.job.is_some() {
            cx.request_repaint_after(std::time::Duration::from_millis(50));
        }
    }
}

fn show_rows(ui: &mut egui::Ui, id: &str, rows: &[PropRow]) {
    egui::Grid::new(id)
        .num_columns(1)
        .striped(true)
        .show(ui, |ui| {
            for r in rows {
                ui.label(format!("{}: {}", r.label, r.value));
                ui.end_row();
            }
        });
}

impl Panel for PropertiesPanel {
    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut AppCx<'_>) {
        self.sync(cx);
        if self.job.is_some() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("Computing…");
                if ui.button("Cancel").clicked()
                    && let Some(j) = self.job.take()
                {
                    j.cancel();
                }
            });
        }
        if let Some(e) = &self.error {
            ui.colored_label(ui.visuals().error_fg_color, e);
        }
        let Some(r) = self.result.clone() else {
            if self.job.is_none() && self.error.is_none() {
                ui.label("No mesh body in the design.");
            }
            return;
        };
        let many = r.bodies.len() > 1;
        if many {
            ui.heading(format!("{} bodies", r.bodies.len()));
        } else if let Some(b) = r.bodies.first() {
            ui.heading(format!("Body {}", body_label(b.body)));
        }
        // Material line(s).
        let defaulted_all = r.bodies.iter().all(|b| b.defaulted);
        let mut names: Vec<&str> = r.bodies.iter().map(|b| b.material.as_str()).collect();
        names.dedup();
        let any_default = r.bodies.iter().any(|b| b.defaulted);
        let note = if defaulted_all {
            " (default material)"
        } else if any_default {
            " (default material for some bodies)"
        } else {
            ""
        };
        ui.label(format!("Material: {}{note}", names.join(", ")));
        if let Some(reason) = &r.total.not_watertight {
            ui.colored_label(
                ui.visuals().warn_fg_color,
                format!(
                    "Volume, mass, centre of mass and inertia need a closed mesh: {reason}. \
                     Repair the mesh first (Mesh ▸ Repair)."
                ),
            );
        }
        show_rows(ui, "props_total", &rows(&r.total, note));
        ui.checkbox(&mut self.show_total, "Show centre of mass");
        if many {
            ui.separator();
            for b in &r.bodies {
                let note = if b.defaulted {
                    " (default material)"
                } else {
                    ""
                };
                egui::CollapsingHeader::new(format!("Body {}", body_label(b.body)))
                    .id_salt(("props_body", b.body.origin.0, b.body.slot))
                    .show(ui, |ui| {
                        ui.label(format!("Material: {}{note}", b.material));
                        show_rows(
                            ui,
                            &format!("props_{}_{}", b.body.origin.0, b.body.slot),
                            &rows(&b.props, note),
                        );
                        let mut on = self.show_bodies.contains(&b.body);
                        if ui
                            .checkbox(&mut on, "Show centre of mass of this body")
                            .changed()
                        {
                            if on {
                                self.show_bodies.insert(b.body);
                            } else {
                                self.show_bodies.remove(&b.body);
                            }
                        }
                    });
            }
        }
    }

    fn overlay(&mut self, out: &mut OverlayList, _env: &CmdEnv<'_>) {
        let Some(r) = &self.result else {
            return;
        };
        let mut at: Vec<(Point3, &'static str)> = Vec::new();
        if self.show_total
            && let Some(c) = r.total.center_of_mass
        {
            at.push((c, "CoM"));
        }
        if r.bodies.len() > 1 {
            for b in r
                .bodies
                .iter()
                .filter(|b| self.show_bodies.contains(&b.body))
            {
                if let Some(c) = b.props.center_of_mass {
                    at.push((c, "CoM"));
                }
            }
        }
        for (c, text) in at {
            glyph(out, c, text);
        }
    }
}

/// The centre-of-mass glyph: three rings and a dot, always on top, with a label.
pub fn glyph(out: &mut OverlayList, c: Point3, text: &str) {
    let line = LineStyle {
        width_px: 1.5,
        color: GLYPH,
        dash: None,
        depth: DepthMode::AlwaysOnTop,
    };
    for n in [Dir3::X, Dir3::Y, Dir3::Z] {
        out.ring(c, n, GLYPH_RING_PX, line, None);
    }
    out.points(
        &[c],
        PointStyle {
            size_px: 5.0,
            color: GLYPH,
            depth: DepthMode::AlwaysOnTop,
        },
        None,
    );
    out.label(
        c,
        text,
        LabelStyle {
            size_pt: 12.0,
            color: [1.0, 1.0, 1.0, 1.0],
            offset_px: [18.0, -14.0],
            background: Some([0.02, 0.02, 0.025, 0.85]),
        },
    );
}

/// "Properties" in the viewport context menu of a mesh body.
#[derive(Debug)]
struct PropertiesMenu;

impl ContextMenuProvider for PropertiesMenu {
    fn fill(&self, env: &CmdEnv<'_>, target: &ContextTarget<'_>, out: &mut Vec<CmdId>) {
        if let ContextTarget::Viewport(Some(item)) = target
            && let Some(b) = item.target.body()
            && is_mesh(env.evaluated, b)
        {
            out.push(PROPERTIES_CMD);
        }
    }
}

/// Registers the panel and the context-menu entry (the command is added by `inspect::register`).
pub(crate) fn register(r: &mut crate::module::Registrar<'_>) {
    r.panel(PanelDef {
        id: PROPERTIES_PANEL,
        title: "Properties",
        icon: crate::module::IconId::NONE,
        slot: DockSlot::Right,
        workspaces: Some(vec![DESIGN]),
        create: create_panel,
    });
    r.context_menu(Box::new(PropertiesMenu));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn significant_digits_trim_zeros() {
        assert_eq!(sig(1.24, 5), "1.24");
        assert_eq!(sig(1000.0, 7), "1000");
        assert_eq!(sig(0.000_123_456_789, 4), "0.0001235");
        assert_eq!(sig(123_456.789, 5), "123457");
        assert_eq!(sig(0.0, 5), "0");
        assert_eq!(sig(-2.5, 5), "-2.5");
    }

    #[test]
    fn masses_switch_to_kg() {
        assert_eq!(mass_text(1.24e-6), "1.24 g");
        assert_eq!(mass_text(2.5e-3), "2.5 kg");
        assert_eq!(mass_text(0.0), "0 g");
    }
}
