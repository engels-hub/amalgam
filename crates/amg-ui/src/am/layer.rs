//! The plate scene of the Manufacture workspace: a document layer (INS-09 pattern).
//!
//! Each frame while the workspace is active, [`AmLayer`]:
//! 1. resolves the active setup and the items of its active plate (cheap: `Arc` clones);
//! 2. finds items whose (mesh, placement, build volume) changed since their last fit check and
//!    runs the effective transforms and `check_fit` for them as one latest-wins job (a newer
//!    change supersedes a running one: it is cancelled, unless it started less than
//!    [`FIT_GRACE`] ago, in which case it finishes and the newest change runs right after it, so
//!    a drag does not flood the pool with cancelled jobs), so the UI thread never walks the
//!    vertices;
//! 3. hands the scene the items at their last known plate transforms
//!    ([`crate::layer::LayerOut::body_views`]; other bodies are hidden), red when they do not fit;
//! 4. draws the plate, grid, volume edges and keep-out areas as RND-07 overlays, plus a label
//!    with the reason next to each item that does not fit.
//!
//! The first frame in the workspace fits the camera to the build volume.
//!
//! With Overhang Analysis on (AM-12, [`super::overhang`]), each item's view also carries the
//! RND-08 overhang shading from AM-02 `shader_params` (same transform as the item, so the
//! colours stay in sync every frame), the coarse/fine analysis jobs run for the plate and the
//! island markers are drawn.

use std::collections::BTreeMap;
use std::sync::Arc;

use amg_am::AmError;
use amg_am::machine::BuildVolume;
use amg_am::overhang::AnalysisSettings;
use amg_am::place::{
    BuildVolumeGeometry, FIT_TOLERANCE, FitReport, Placement, build_volume_geometry, check_fit,
    effective_transform,
};
use amg_core::{Point3, Transform};
use amg_doc::{BodyId, ExtKey};
use amg_features::am::{ItemGeom, ItemId, PrintSetup, UnresolvedReason, resolve_plate};
use amg_jobs::{JobHandle, SpawnOptions};
use amg_mesh::IndexedMesh;
use amg_render::{DepthMode, LabelStyle, LineStyle, PointStyle};

use super::overhang::{self, OverhangState, Quality, Want};
use super::{AM, OUT_OF_VOLUME_TINT, active_setup, output};
use crate::app::AppCx;
use crate::layer::{DocLayer, LayerOut};
use crate::scene::{BodyView, BodyViews};

/// Grid spacing of the plate, mm.
pub const GRID_STEP: f64 = 10.0;

/// A running fit check younger than this is not cancelled by a newer change (AM-12: cancelling
/// one per frame during a drag left the pool busy with cancelled jobs and starved the overhang
/// analysis); it finishes and the newest change runs next.
pub const FIT_GRACE: std::time::Duration = std::time::Duration::from_millis(250);

const PLATE_FILL: [f32; 4] = [0.16, 0.17, 0.19, 1.0];
const GRID_COLOR: [f32; 4] = [0.32, 0.34, 0.38, 1.0];
const OUTLINE_COLOR: [f32; 4] = [0.45, 0.62, 0.85, 1.0];
const EXCLUDE_FILL: [f32; 4] = [0.55, 0.18, 0.12, 0.6];

/// The status of one item of the active plate (Plate panel, labels, tests).
#[derive(Clone, Debug, PartialEq)]
pub struct ItemStatus {
    /// The item.
    pub item: ItemId,
    /// Its body.
    pub body: BodyId,
    /// Object name.
    pub name: String,
    /// Filament slot.
    pub filament: Option<u8>,
    /// Body-local → plate transform of the last fit check (`None` before the first one).
    pub transform: Option<Transform>,
    /// The last fit check.
    pub fit: Option<FitReport>,
    /// Why it does not fit, or why it cannot be shown ("3.2 mm too tall").
    pub message: Option<String>,
    /// Why the item cannot be printed (body missing, not a mesh, hidden).
    pub unresolved: Option<UnresolvedReason>,
    /// A newer fit check is running.
    pub pending: bool,
}

/// Why an item does not fit, e.g. "3.2 mm too tall, 4.0 mm outside the bed"; `None` if it fits.
pub fn fit_message(f: &FitReport) -> Option<String> {
    if f.fits {
        return None;
    }
    let mut parts = Vec::new();
    if f.height_excess > FIT_TOLERANCE {
        parts.push(format!("{:.1} mm too tall", f.height_excess));
    }
    if f.max_outside_xy > FIT_TOLERANCE {
        parts.push(format!("{:.1} mm outside the bed", f.max_outside_xy));
    }
    if f.hits_exclude {
        parts.push("over a keep-out area".to_owned());
    }
    if parts.is_empty() {
        parts.push("outside the build volume".to_owned());
    }
    Some(parts.join(", "))
}

/// Text for an unresolved item.
pub fn unresolved_message(why: &UnresolvedReason) -> &'static str {
    match why {
        UnresolvedReason::BodyMissing => "body missing",
        UnresolvedReason::NotAMesh => "convert to mesh first",
        UnresolvedReason::Hidden => "body hidden: not printed",
    }
}

struct Eval {
    mesh: Arc<IndexedMesh>,
    placement: Placement,
    volume: BuildVolume,
    transform: Transform,
    fit: FitReport,
}

type EvalOut = Vec<(ItemId, Arc<IndexedMesh>, Placement, Transform, FitReport)>;

struct Pending {
    inputs: Vec<(ItemId, usize, Placement)>,
    volume: BuildVolume,
    started: std::time::Instant,
    handle: JobHandle<EvalOut, AmError>,
}

/// The Manufacture workspace's per-document state (see the module docs).
pub struct AmLayer {
    /// The setup chosen in the browser or Plate panel (`None`: the newest).
    pub(crate) active: Option<ExtKey>,
    /// A placement shown instead of the item's own (Move/Rotate live preview).
    pub(crate) preview: Option<(ItemId, Placement)>,
    shown: Option<ExtKey>,
    evals: BTreeMap<ItemId, Eval>,
    evals_for: Option<ExtKey>,
    pending: Option<Pending>,
    geometry: Option<(BuildVolume, Arc<BuildVolumeGeometry>)>,
    views: Option<Arc<BodyViews>>,
    status: Vec<ItemStatus>,
    fitted: bool,
    checks: u64,
    pub(crate) export: Option<output::PendingExport>,
    pub(crate) last_output: Option<output::LastOutput>,
    /// Overhang analyses and their jobs (AM-12).
    pub(crate) overhang: OverhangState,
    /// Analysis settings shown instead of the setup's (Overhang Settings live preview).
    pub(crate) settings_preview: Option<AnalysisSettings>,
    /// The settings used in the last frame.
    shown_settings: Option<AnalysisSettings>,
}

impl std::fmt::Debug for AmLayer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AmLayer")
            .field("active", &self.active)
            .field("shown", &self.shown)
            .field("items", &self.status.len())
            .field("pending", &self.pending.is_some())
            .finish_non_exhaustive()
    }
}

/// Creates the layer of a document.
pub(crate) fn create() -> Box<dyn DocLayer> {
    Box::new(AmLayer::new())
}

impl Default for AmLayer {
    fn default() -> Self {
        Self::new()
    }
}

impl AmLayer {
    /// An empty layer.
    pub fn new() -> AmLayer {
        AmLayer {
            active: None,
            preview: None,
            shown: None,
            evals: BTreeMap::new(),
            evals_for: None,
            pending: None,
            geometry: None,
            views: None,
            status: Vec::new(),
            fitted: false,
            checks: 0,
            export: None,
            last_output: None,
            overhang: OverhangState::default(),
            settings_preview: None,
            shown_settings: None,
        }
    }

    /// Makes `key` the setup the workspace works on (`None`: the newest).
    pub fn set_active(&mut self, key: Option<ExtKey>) {
        self.active = key;
    }

    /// The setup shown in the last frame.
    pub fn shown_setup(&self) -> Option<&ExtKey> {
        self.shown.as_ref()
    }

    /// The items of the active plate as of the last frame.
    pub fn status(&self) -> &[ItemStatus] {
        &self.status
    }

    /// A fit check or an overhang analysis is running.
    pub fn busy(&self) -> bool {
        self.pending.is_some() || self.overhang.busy()
    }

    /// The overhang analyses of the plate (AM-12).
    pub fn overhang(&self) -> &OverhangState {
        &self.overhang
    }

    /// The analysis settings of the last frame (the live preview of Overhang Settings, else the
    /// setup's).
    pub fn shown_settings(&self) -> Option<&AnalysisSettings> {
        self.shown_settings.as_ref()
    }

    /// The Overhang Settings preview.
    pub fn settings_preview(&self) -> Option<&AnalysisSettings> {
        self.settings_preview.as_ref()
    }

    /// Fit-check jobs finished so far.
    pub fn checks(&self) -> u64 {
        self.checks
    }

    /// What the last Send to Slicer or Export Plate 3MF wrote.
    pub fn last_output(&self) -> Option<&output::LastOutput> {
        self.last_output.as_ref()
    }

    /// The Move/Rotate preview placement.
    pub fn preview(&self) -> Option<(ItemId, Placement)> {
        self.preview
    }

    fn poll_pending(&mut self) {
        let Some(p) = &mut self.pending else { return };
        let Some(res) = p.handle.try_take() else {
            return;
        };
        let volume = p.volume.clone();
        self.pending = None;
        match res {
            Ok(out) => {
                self.checks += 1;
                for (id, mesh, placement, transform, fit) in out {
                    self.evals.insert(
                        id,
                        Eval {
                            mesh,
                            placement,
                            volume: volume.clone(),
                            transform,
                            fit,
                        },
                    );
                }
            }
            Err(e) => tracing::debug!("am: fit check ended: {e}"),
        }
    }

    fn cancel_pending(&mut self) {
        if let Some(p) = self.pending.take() {
            p.handle.cancel();
        }
    }

    fn spawn(
        &mut self,
        cx: &AppCx<'_>,
        stale: Vec<(ItemId, Arc<IndexedMesh>, Placement)>,
        volume: &BuildVolume,
    ) {
        let inputs: Vec<(ItemId, usize, Placement)> = stale
            .iter()
            .map(|(i, m, p)| (*i, Arc::as_ptr(m) as usize, *p))
            .collect();
        if self.pending.as_ref().is_some_and(|p| {
            (p.inputs == inputs && p.volume == *volume) || p.started.elapsed() < FIT_GRACE
        }) {
            return;
        }
        self.cancel_pending();
        let v = volume.clone();
        let handle = cx.jobs().spawn_fn(
            "Fit check",
            SpawnOptions::default(),
            move |ctx| -> Result<EvalOut, AmError> {
                let mut out = Vec::with_capacity(stale.len());
                for (id, mesh, p) in stale {
                    ctx.checkpoint()?;
                    let t = effective_transform(&mesh, &p);
                    let fit = check_fit(&mesh, &t, &v, ctx)?;
                    out.push((id, mesh, p, t, fit));
                }
                Ok(out)
            },
        );
        self.pending = Some(Pending {
            inputs,
            volume: volume.clone(),
            started: std::time::Instant::now(),
            handle,
        });
    }

    fn geometry(&mut self, volume: &BuildVolume) -> Arc<BuildVolumeGeometry> {
        if let Some((v, g)) = &self.geometry
            && v == volume
        {
            return Arc::clone(g);
        }
        let g = Arc::new(build_volume_geometry(volume, GRID_STEP));
        self.geometry = Some((volume.clone(), Arc::clone(&g)));
        g
    }

    fn update_inner(&mut self, cx: &mut AppCx<'_>, out: &mut LayerOut<'_>) {
        output::poll_export(self, cx);
        if cx.env().workspace != AM {
            self.fitted = false;
            self.cancel_pending();
            self.overhang.suspend();
            return;
        }
        self.poll_pending();
        let (design, ev) = {
            let env = cx.env();
            (env.design.clone(), Arc::clone(env.evaluated))
        };
        let Some((key, setup)) = active_setup(&design, self.active.as_ref()) else {
            // No setup: the design bodies as they are, so they can be selected and added.
            self.shown = None;
            self.status.clear();
            self.views = None;
            self.overhang.reset();
            return;
        };
        if self.evals_for.as_ref() != Some(&key) {
            self.evals.clear();
            self.cancel_pending();
            self.overhang.reset();
            self.evals_for = Some(key.clone());
        }
        self.shown = Some(key);
        let volume = setup.machine.volume.clone();
        let (status, stale, wants) = self.items(&setup, &design, &ev, &volume);
        if !stale.is_empty() {
            self.spawn(cx, stale, &volume);
        }
        self.status = status;
        let settings = self
            .settings_preview
            .clone()
            .unwrap_or_else(|| setup.settings.clone());
        let overlay = overhang::overlay_enabled();
        if overlay {
            self.overhang
                .update(cx.jobs(), &wants, &settings, &setup.machine);
        } else {
            self.overhang.suspend();
        }
        let opts = overhang::analysis_opts(&settings, &setup.machine, Quality::Fine);
        self.shown_settings = Some(settings);
        let bodies: BTreeMap<BodyId, BodyView> = self
            .status
            .iter()
            .filter_map(|s| {
                let t = s.transform?;
                let fits = s.fit.as_ref().is_none_or(|f| f.fits);
                // The overlay's uniforms come from the same fit check as the transform, so
                // colours and position never disagree.
                let shading = match (&s.fit, overlay) {
                    (Some(f), true) if !f.aabb.is_empty() => {
                        Some(overhang::overlay_shading(&t, f.aabb.min.z, &opts))
                    }
                    _ => None,
                };
                Some((
                    s.body,
                    BodyView {
                        transform: t,
                        tint: (!fits).then_some(OUT_OF_VOLUME_TINT),
                        shading,
                    },
                ))
            })
            .collect();
        let views = BodyViews { bodies };
        if self.views.as_deref() != Some(&views) {
            self.views = Some(Arc::new(views));
        }
        *out.body_views = self.views.clone();
        self.draw(&volume, out);
        if overlay {
            self.draw_islands(&wants, out);
        }
        if !self.fitted {
            self.fitted = true;
            use crate::nav::CameraControl as _;
            cx.camera().fit(Some(volume.aabb()));
        }
    }

    /// Status of every item of the active plate, the items whose fit check is stale and the
    /// mesh items with their shown placements (overhang analysis).
    #[allow(clippy::type_complexity, reason = "one-off internal tuple")]
    fn items(
        &mut self,
        setup: &PrintSetup,
        design: &amg_doc::Design,
        ev: &amg_doc::Evaluated,
        volume: &BuildVolume,
    ) -> (
        Vec<ItemStatus>,
        Vec<(ItemId, Arc<IndexedMesh>, Placement)>,
        Vec<Want>,
    ) {
        let resolved = resolve_plate(setup, setup.active_plate, design, ev);
        self.evals
            .retain(|id, _| resolved.iter().any(|r| r.item.id == *id));
        let mut status = Vec::with_capacity(resolved.len());
        let mut stale = Vec::new();
        let mut wants = Vec::new();
        for r in &resolved {
            let placement = match self.preview {
                Some((id, p)) if id == r.item.id => p,
                _ => r.item.placement,
            };
            let mut st = ItemStatus {
                item: r.item.id,
                body: r.item.body,
                name: r.name.clone(),
                filament: r.item.filament,
                transform: None,
                fit: None,
                message: None,
                unresolved: None,
                pending: false,
            };
            match &r.geom {
                ItemGeom::Mesh(m) => {
                    wants.push(Want {
                        item: r.item.id,
                        mesh: Arc::clone(m),
                        placement,
                    });
                    let e = self.evals.get(&r.item.id);
                    let fresh = e.is_some_and(|e| {
                        Arc::ptr_eq(&e.mesh, m) && e.placement == placement && e.volume == *volume
                    });
                    if !fresh {
                        stale.push((r.item.id, Arc::clone(m), placement));
                    }
                    if let Some(e) = e {
                        st.transform = Some(e.transform);
                        st.message = fit_message(&e.fit);
                        st.fit = Some(e.fit.clone());
                    }
                    st.pending = !fresh;
                }
                ItemGeom::Unresolved(why) => {
                    st.message = Some(unresolved_message(why).to_owned());
                    st.unresolved = Some(why.clone());
                }
            }
            status.push(st);
        }
        (status, stale, wants)
    }

    /// Island markers of the analysed items, at the items' shown transforms.
    fn draw_islands(&self, wants: &[Want], out: &mut LayerOut<'_>) {
        let style = PointStyle {
            size_px: overhang::ISLAND_MARKER_PX,
            color: overhang::ISLAND_MARKER_COLOR,
            depth: DepthMode::AlwaysOnTop,
        };
        for w in wants {
            let (Some(a), Some(t)) = (
                self.overhang.analysis(w.item),
                self.status
                    .iter()
                    .find(|s| s.item == w.item)
                    .and_then(|s| s.transform),
            ) else {
                continue;
            };
            let pts = overhang::island_points(&w.mesh, &a.report, &t);
            if !pts.is_empty() {
                out.overlays.points(&pts, style, None);
            }
        }
    }

    /// The island marker positions of the analysed items at their shown transforms (what the
    /// last frame drew while the overlay was on).
    pub fn island_markers(&self) -> Vec<(ItemId, Vec<Point3>)> {
        self.status
            .iter()
            .filter_map(|s| {
                let a = self.overhang.analysis(s.item)?;
                let t = s.transform?;
                let e = self.evals.get(&s.item)?;
                Some((s.item, overhang::island_points(&e.mesh, &a.report, &t)))
            })
            .collect()
    }

    fn draw(&mut self, volume: &BuildVolume, out: &mut LayerOut<'_>) {
        let g = self.geometry(volume);
        let o = &mut *out.overlays;
        o.polygon(
            &g.plate.positions,
            &g.plate.tris,
            PLATE_FILL,
            DepthMode::Test,
            None,
        );
        for ex in &g.exclude {
            o.polygon(&ex.positions, &ex.tris, EXCLUDE_FILL, DepthMode::Test, None);
        }
        let lift = |pts: &[Point3]| -> Vec<Point3> {
            pts.iter()
                .map(|p| Point3::new(p.x, p.y, p.z + 0.02))
                .collect()
        };
        let grid = LineStyle {
            width_px: 1.0,
            color: GRID_COLOR,
            dash: None,
            depth: DepthMode::Test,
        };
        for l in &g.grid {
            o.polyline(&closed(&lift(&l.points), l.closed), grid, None);
        }
        let edge = LineStyle {
            width_px: 1.5,
            color: OUTLINE_COLOR,
            dash: None,
            depth: DepthMode::Test,
        };
        for l in &g.outline {
            o.polyline(&closed(&lift(&l.points), l.closed), edge, None);
        }
        for s in &self.status {
            let (Some(msg), Some(fit)) = (&s.message, &s.fit) else {
                continue;
            };
            let b = fit.aabb;
            let at = Point3::new(
                (b.min.x + b.max.x) * 0.5,
                (b.min.y + b.max.y) * 0.5,
                b.max.z,
            );
            o.label(
                at,
                msg.clone(),
                LabelStyle {
                    size_pt: 13.0,
                    color: [1.0, 0.85, 0.8, 1.0],
                    offset_px: [0.0, -16.0],
                    background: Some([0.45, 0.05, 0.03, 0.85]),
                },
            );
        }
    }
}

fn closed(pts: &[Point3], closed: bool) -> Vec<Point3> {
    let mut v = pts.to_vec();
    if closed && let Some(first) = pts.first() {
        v.push(*first);
    }
    v
}

impl DocLayer for AmLayer {
    fn update(&mut self, cx: &mut AppCx<'_>, out: &mut LayerOut<'_>) {
        self.update_inner(cx, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::Aabb3;

    fn report(fits: bool, h: f64, xy: f64, ex: bool) -> FitReport {
        FitReport {
            fits,
            height_excess: h,
            max_outside_xy: xy,
            hits_exclude: ex,
            footprint_hull: Vec::new(),
            aabb: Aabb3::EMPTY,
        }
    }

    #[test]
    fn fit_messages_name_every_reason() {
        assert_eq!(fit_message(&report(true, 0.0, 0.0, false)), None);
        assert_eq!(
            fit_message(&report(false, 3.24, 0.0, false)).as_deref(),
            Some("3.2 mm too tall")
        );
        assert_eq!(
            fit_message(&report(false, 10.0, 4.0, true)).as_deref(),
            Some("10.0 mm too tall, 4.0 mm outside the bed, over a keep-out area")
        );
        assert_eq!(
            fit_message(&report(false, 0.0, 0.0, false)).as_deref(),
            Some("outside the build volume")
        );
    }

    #[test]
    fn closed_polylines_repeat_their_first_point() {
        let p = [Point3::ZERO, Point3::X, Point3::Y];
        assert_eq!(closed(&p, true).len(), 4);
        assert_eq!(closed(&p, false).len(), 3);
    }
}
