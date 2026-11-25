//! The Properties panel (S05 §4, INS-03, INS-10): volume, area, mass, centre of mass, inertia
//! and principal axes of the selected mesh and solid body instances and occurrences (every
//! instance of the design when none is selected), with the density of each body's material
//! (the design's default material, marked "(default material)", when it has none). Designs
//! with occurrences also get totals per occurrence and per component.
//!
//! - Results come from `amg_features::inspect::props` as one cancellable background job
//!   ("Mass properties"); a changed selection, body, placement or material supersedes it.
//! - A body that cannot be measured (a solid in a build without the B-rep kernel) is listed
//!   with the reason and left out of the totals.
//! - The panel is opened by [`PROPERTIES_CMD`] (INSPECT ▸ Inspect, SOLID ▸ Inspect, S-key
//!   search) and by the "Properties" entry of the viewport context menu on a mesh body.
//! - The centre-of-mass glyph is drawn through the additive `Panel::overlay` hook (RND-07
//!   rings and a point, always on top).

use std::collections::BTreeSet;
use std::sync::Arc;

use amg_core::{Density, Dir3, Mat3, Point3, Transform};
use amg_doc::{BodyGeom, BodyId, Design, OccPath};
use amg_features::inspect::InspectError;
use amg_features::inspect::doc::assembly::{
    PropsPick, PropsScope, is_measurable, occurrence_name, scopes_of, selected_instances,
};
use amg_features::inspect::props::{
    MassProps, PropsGeom, PropsItem, combine, group_totals, items_mass_props,
};
use amg_jobs::{JobCtx, JobError, JobHandle, SpawnOptions};
use amg_materials::{LibrarySet, MaterialPrefs};
use amg_mesh::MeshBody;
use amg_render::{DepthMode, LabelStyle, LineStyle, OverlayList, PointStyle};

use crate::app::{AppCx, CmdEnv};
use crate::command::{CmdId, Enablement};
use crate::design::DESIGN;
use crate::marking::{ContextMenuProvider, ContextTarget};
use crate::module::PanelId;
use crate::panel::{DockSlot, Panel, PanelDef};
use crate::pick::PickTarget;

use super::colors::COMPONENT_COLORS_FLAG;

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

/// One body instance to measure.
#[derive(Clone, Debug)]
struct Input {
    body: BodyId,
    path: OccPath,
    geom: BodyGeom,
    world: Transform,
    density: Density,
    material: String,
    defaulted: bool,
    /// The totals this instance counts into.
    scopes: Vec<PropsScope>,
}

impl Input {
    /// Whether a result computed for `other` also holds for `self`.
    fn same(&self, other: &Input) -> bool {
        let bits = |t: &Transform| t.to_cols_array().map(f64::to_bits);
        self.body == other.body
            && self.path == other.path
            && self.geom.same_as(&other.geom)
            && bits(&self.world) == bits(&other.world)
            && self.density.get().to_bits() == other.density.get().to_bits()
            && self.material == other.material
            && self.defaulted == other.defaulted
            && self.scopes == other.scopes
    }

    fn item(&self) -> PropsItem {
        PropsItem {
            geom: match &self.geom {
                BodyGeom::Mesh(m) => PropsGeom::Mesh(MeshBody::from_arc(Arc::clone(m))),
                BodyGeom::Solid(s) => PropsGeom::solid(s.brep().cloned()),
                BodyGeom::Implicit(_) | BodyGeom::Form(_) => PropsGeom::MissingSolid,
            },
            to_world: self.world,
            density: self.density,
        }
    }
}

/// The result for one body instance.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyEntry {
    /// The body.
    pub body: BodyId,
    /// The occurrence path of this instance (empty in the root component).
    pub path: OccPath,
    /// Its properties in world space.
    pub props: MassProps,
    /// Name of the material whose density was used.
    pub material: String,
    /// The material is the design's default (the body has none).
    pub defaulted: bool,
    /// The body is a mesh body.
    pub mesh: bool,
}

/// A body instance that could not be measured.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FailedBody {
    /// The body.
    pub body: BodyId,
    /// The occurrence path of this instance.
    pub path: OccPath,
    /// Why.
    pub error: String,
}

/// The total of one occurrence or component.
#[derive(Clone, Debug, PartialEq)]
pub struct ScopeTotal {
    /// Which.
    pub scope: PropsScope,
    /// Body instances in the total.
    pub instances: usize,
    /// Body instances left out because they could not be measured.
    pub skipped: usize,
    /// Some of them use the design's default material.
    pub defaulted: bool,
    /// [`combine`] of the measured instances.
    pub total: MassProps,
}

/// The result of one run: per body instance and the totals.
#[derive(Clone, Debug, PartialEq)]
pub struct PropsResult {
    /// Per measured instance, ordered by body then path.
    pub bodies: Vec<BodyEntry>,
    /// The instances that could not be measured.
    pub failed: Vec<FailedBody>,
    /// The total of the measured instances ([`combine`]; equals the body for one); `None`
    /// if there are none.
    pub total: Option<MassProps>,
    /// Occurrence totals, then component totals; empty in a design without occurrences.
    pub scopes: Vec<ScopeTotal>,
}

fn compute(inputs: &[Input], ctx: &JobCtx) -> Result<Arc<PropsResult>, InspectError> {
    let items: Vec<PropsItem> = inputs.iter().map(Input::item).collect();
    let measured = items_mass_props(&items, ctx)?;
    let mut bodies = Vec::new();
    let mut failed = Vec::new();
    for (inp, r) in inputs.iter().zip(&measured) {
        match r {
            Ok(props) => bodies.push(BodyEntry {
                body: inp.body,
                path: inp.path.clone(),
                props: props.clone(),
                material: inp.material.clone(),
                defaulted: inp.defaulted,
                mesh: matches!(inp.geom, BodyGeom::Mesh(_)),
            }),
            Err(e) => failed.push(FailedBody {
                body: inp.body,
                path: inp.path.clone(),
                error: e.to_string(),
            }),
        }
    }
    let parts: Vec<MassProps> = bodies.iter().map(|b| b.props.clone()).collect();
    let total = (!parts.is_empty()).then(|| combine(&parts));
    // Totals per occurrence and component, once the design has any occurrence to speak of.
    let scopes = if inputs.iter().all(|i| i.path.is_empty()) {
        Vec::new()
    } else {
        let keyed: Vec<(Vec<PropsScope>, Option<&MassProps>)> = inputs
            .iter()
            .zip(&measured)
            .map(|(i, r)| (i.scopes.clone(), r.as_ref().ok()))
            .collect();
        group_totals(&keyed)
            .into_iter()
            .filter_map(|g| {
                Some(ScopeTotal {
                    defaulted: g.members.iter().any(|&i| inputs[i].defaulted),
                    instances: g.members.len(),
                    skipped: g.skipped,
                    total: g.total?,
                    scope: g.key,
                })
            })
            .collect()
    };
    Ok(Arc::new(PropsResult {
        bodies,
        failed,
        total,
        scopes,
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

/// What the selection asks for (an empty list means the whole design).
fn picks(env: &CmdEnv<'_>) -> Vec<PropsPick> {
    env.selection
        .items()
        .iter()
        .filter_map(|i| match i.target {
            PickTarget::Body { body, instance, .. } => Some(PropsPick::Body {
                body,
                path: env.instance_paths.get(instance as usize).cloned(),
            }),
            PickTarget::Occurrence(o) => Some(PropsPick::Occurrence(o)),
            PickTarget::Sketch(_) | PickTarget::Construction(_) => None,
        })
        .collect()
}

fn gather(
    env: &CmdEnv<'_>,
    prefs: &MaterialPrefs,
    libs: &LibrarySet,
) -> Result<Vec<Input>, String> {
    let mut out = Vec::new();
    for placed in selected_instances(env.evaluated, &picks(env)) {
        let Some(eb) = env.evaluated.body(placed.body) else {
            continue;
        };
        let m = env
            .design
            .mass_material(placed.body, prefs, libs)
            .ok_or_else(|| {
                format!(
                    "The default material {} is not available",
                    prefs.default_material
                )
            })?;
        out.push(Input {
            body: placed.body,
            scopes: scopes_of(env.design, &placed.path),
            path: placed.path,
            geom: eb.geom.clone(),
            world: placed.world,
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

fn path_label(design: &Design, path: &[amg_doc::OccurrenceId]) -> String {
    let names: Vec<String> = path.iter().map(|o| occurrence_name(design, *o)).collect();
    names.join(" / ")
}

/// `"Body 3.0"`, with the occurrence path for a placed instance.
fn instance_label(design: &Design, body: BodyId, path: &[amg_doc::OccurrenceId]) -> String {
    if path.is_empty() {
        format!("Body {}", body_label(body))
    } else {
        format!("Body {} in {}", body_label(body), path_label(design, path))
    }
}

/// `"1 body"`, `"2 bodies"`, `"3 body instances"`.
fn count(n: usize, one: &str) -> String {
    match one.strip_suffix('y') {
        _ if n == 1 => format!("1 {one}"),
        Some(stem) => format!("{n} {stem}ies"),
        None => format!("{n} {one}s"),
    }
}

fn scope_label(design: &Design, s: &ScopeTotal) -> String {
    let n = count(s.instances, "body instance");
    match &s.scope {
        PropsScope::Occurrence(path) => {
            format!("Occurrence {} ({n})", path_label(design, path))
        }
        PropsScope::Component(c) => {
            let name = design
                .component(*c)
                .map_or_else(|| c.0.to_string(), |k| k.name.clone());
            format!("Component {name} ({n})")
        }
    }
}

// ---- the panel --------------------------------------------------------------------------------

type PropsJob = JobHandle<Arc<PropsResult>, InspectError>;

/// The Properties panel.
pub struct PropertiesPanel {
    libs: LibrarySet,
    prefs: MaterialPrefs,
    inputs: Option<Vec<Input>>,
    job: Option<PropsJob>,
    result: Option<Arc<PropsResult>>,
    error: Option<String>,
    /// Draw the centre of mass of the selection total.
    show_total: bool,
    /// Draw the centre of mass of these body instances (several instances).
    show_bodies: BTreeSet<(BodyId, OccPath)>,
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
        inputs: None,
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
                self.inputs = None;
                self.result = None;
                self.error = Some(e);
            }
            Ok(inputs) => {
                let unchanged = self.inputs.as_ref().is_some_and(|old| {
                    old.len() == inputs.len() && old.iter().zip(&inputs).all(|(a, b)| a.same(b))
                });
                if !unchanged {
                    if let Some(j) = self.job.take() {
                        j.cancel();
                    }
                    self.inputs = Some(inputs.clone());
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

    /// The material line, the warnings and the rows of the total.
    fn total_ui(&mut self, ui: &mut egui::Ui, r: &PropsResult, total: &MassProps) {
        let defaulted_all = r.bodies.iter().all(|b| b.defaulted);
        let mut names: Vec<&str> = r.bodies.iter().map(|b| b.material.as_str()).collect();
        names.sort_unstable();
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
        if let Some(reason) = &total.not_watertight {
            let open_mesh = r
                .bodies
                .iter()
                .any(|b| b.mesh && b.props.not_watertight.is_some());
            let (needs, hint) = if open_mesh {
                ("mesh", " Repair the mesh first (Mesh ▸ Repair).")
            } else {
                ("solid", "")
            };
            ui.colored_label(
                ui.visuals().warn_fg_color,
                format!(
                    "Volume, mass, centre of mass and inertia need a closed {needs}: \
                     {reason}.{hint}"
                ),
            );
        }
        show_rows(ui, "props_total", &rows(total, note));
        ui.checkbox(&mut self.show_total, "Show centre of mass");
    }

    /// One collapsing section per body instance.
    fn bodies_ui(&mut self, ui: &mut egui::Ui, design: &Design, r: &PropsResult) {
        for b in &r.bodies {
            let note = if b.defaulted {
                " (default material)"
            } else {
                ""
            };
            let id = (b.body, b.path.clone());
            egui::CollapsingHeader::new(instance_label(design, b.body, &b.path))
                .id_salt(("props_body", &id))
                .show(ui, |ui| {
                    ui.label(format!("Material: {}{note}", b.material));
                    ui.push_id(("props_rows", &id), |ui| {
                        show_rows(ui, "props_body_rows", &rows(&b.props, note));
                    });
                    let mut on = self.show_bodies.contains(&id);
                    if ui
                        .checkbox(&mut on, "Show centre of mass of this body")
                        .changed()
                    {
                        if on {
                            self.show_bodies.insert(id.clone());
                        } else {
                            self.show_bodies.remove(&id);
                        }
                    }
                });
        }
    }
}

/// One collapsing section per occurrence and component total.
fn scopes_ui(ui: &mut egui::Ui, design: &Design, scopes: &[ScopeTotal]) {
    ui.separator();
    ui.strong("Occurrence and component totals");
    for s in scopes {
        let note = if s.defaulted {
            " (default material for some bodies)"
        } else {
            ""
        };
        egui::CollapsingHeader::new(scope_label(design, s))
            .id_salt(("props_scope", &s.scope))
            .show(ui, |ui| {
                if s.skipped > 0 {
                    ui.colored_label(
                        ui.visuals().warn_fg_color,
                        format!(
                            "Leaves out {} that could not be measured.",
                            count(s.skipped, "body instance")
                        ),
                    );
                }
                ui.push_id(("props_scope_rows", &s.scope), |ui| {
                    show_rows(ui, "props_scope", &rows(&s.total, note));
                });
            });
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
        let mut colors = cx.env().view_flags.contains(COMPONENT_COLORS_FLAG);
        if ui
            .checkbox(&mut colors, "Display component colours")
            .changed()
        {
            cx.set_view_flag(COMPONENT_COLORS_FLAG, colors);
        }
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
                ui.label("No mesh or solid body to measure.");
            }
            return;
        };
        let env = cx.env();
        let design = env.design;
        let distinct: BTreeSet<BodyId> = r.bodies.iter().map(|b| b.body).collect();
        let many = r.bodies.len() > 1;
        if many && distinct.len() == r.bodies.len() {
            ui.heading(count(r.bodies.len(), "body"));
        } else if many {
            ui.heading(format!(
                "{}, {}",
                count(distinct.len(), "body"),
                count(r.bodies.len(), "body instance")
            ));
        } else if let Some(b) = r.bodies.first() {
            ui.heading(instance_label(design, b.body, &b.path));
        }
        for f in &r.failed {
            ui.colored_label(
                ui.visuals().warn_fg_color,
                format!("{}: {}", instance_label(design, f.body, &f.path), f.error),
            );
        }
        if let Some(total) = &r.total {
            if !r.failed.is_empty() {
                ui.colored_label(
                    ui.visuals().warn_fg_color,
                    format!(
                        "The totals leave out {}.",
                        count(r.failed.len(), "body instance")
                    ),
                );
            }
            self.total_ui(ui, &r, total);
        }
        if many {
            ui.separator();
            self.bodies_ui(ui, design, &r);
        }
        if !r.scopes.is_empty() {
            scopes_ui(ui, design, &r.scopes);
        }
    }

    fn overlay(&mut self, out: &mut OverlayList, _env: &CmdEnv<'_>) {
        let Some(r) = &self.result else {
            return;
        };
        let mut at: Vec<(Point3, &'static str)> = Vec::new();
        if self.show_total
            && let Some(c) = r.total.as_ref().and_then(|t| t.center_of_mass)
        {
            at.push((c, "CoM"));
        }
        if r.bodies.len() > 1 {
            for b in r
                .bodies
                .iter()
                .filter(|b| self.show_bodies.contains(&(b.body, b.path.clone())))
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

/// Whether the design has anything to measure (the command's enabler).
pub(crate) fn enabled(env: &CmdEnv<'_>) -> Enablement {
    if env
        .evaluated
        .bodies()
        .any(|(id, _)| is_measurable(env.evaluated, id))
    {
        Enablement::Enabled
    } else {
        Enablement::disabled("The design has no mesh or solid body")
    }
}

/// "Properties" in the viewport context menu of a mesh or solid body, or of a component.
#[derive(Debug)]
struct PropertiesMenu;

impl ContextMenuProvider for PropertiesMenu {
    fn fill(&self, env: &CmdEnv<'_>, target: &ContextTarget<'_>, out: &mut Vec<CmdId>) {
        let ContextTarget::Viewport(Some(item)) = target else {
            return;
        };
        let measurable = match item.target {
            PickTarget::Occurrence(_) => true,
            other => other
                .body()
                .is_some_and(|b| is_measurable(env.evaluated, b)),
        };
        if measurable {
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
