//! Overhang overlay and metrics of the Manufacture workspace (AM-12, S16 Decision 5).
//!
//! - **Colours** come from the S10 RND-08 overhang shader mode, from uniforms only. Every frame
//!   the layer turns the AM-02 [`shader_params`] of each item (its displayed transform, its
//!   lowest plate z and the setup's α / contact tolerance) into
//!   [`Analysis::Overhang`](amg_render::Analysis) on the item's
//!   [`BodyView`](crate::scene::BodyView). Rotating an item or changing α rewrites the
//!   instance record, never the geometry.
//! - **Numbers** come from AM-02 [`analyze_overhang`] run as `amg-jobs` jobs, one latest-wins
//!   job per plate change: *coarse* (cell ×2, ≤ 128² rays) while an item's placement or the
//!   settings change from frame to frame, *fine* (the setup's settings) once they hold still for
//!   one frame ("release"). A newer input cancels and supersedes a running fine job. A running
//!   coarse job (bounded by its ray budget) is allowed to finish unless it is older than
//!   [`COARSE_GRACE`]: its numbers are shown and the newest input runs right after it, so a
//!   continuous drag gets coarse updates at the job rate instead of starving (cancelling it on
//!   every frame would never let one finish).
//! - The fine job's [`OverhangReport::face_class`] is the per-face class buffer of the item
//!   ([`ItemAnalysis`]); it drives the metrics, the island markers and the overlay-vs-CPU check
//!   ([`shader_face_classes`] evaluates the shader's rule on the CPU). Showing it on the GPU
//!   needs a per-face buffer in S10 (requested, not in RND-08).
//! - Island vertices (local minima that need a support of their own) are drawn as RND-07
//!   points over the item.
//!
//! Nothing here runs on the UI thread beyond bookkeeping: no vertex or face pass.

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use amg_am::AmError;
use amg_am::machine::Machine;
use amg_am::overhang::{
    AnalysisSettings, FaceClass, OverhangMetrics, OverhangOpts, OverhangReport, SupportOpts,
    analyze_overhang, shader_params,
};
use amg_am::place::{Placement, effective_transform};
use amg_core::glam::{DVec3, Vec3};
use amg_core::{Dir3, Point3, Tolerance, Transform};
use amg_features::am::ItemId;
use amg_jobs::{JobHandle, SpawnOptions};
use amg_mesh::{BvhOpts, IndexedMesh, MeshSpatial};
use amg_render::{Analysis, Shading};

/// Support-estimate cell factor of the coarse pass (AM-02 log: "cell ×2").
pub const COARSE_CELL_FACTOR: f64 = 2.0;
/// Ray budget of the coarse pass (≤ 128² rays).
pub const COARSE_MAX_RAYS: u32 = 128 * 128;
/// A running coarse job younger than this is not cancelled by a newer input; it finishes and
/// the newest input runs next (so a continuous drag keeps updating). Older ones (a very large
/// mesh) are superseded.
pub const COARSE_GRACE: Duration = Duration::from_millis(1000);
/// Island marker size, px.
pub const ISLAND_MARKER_PX: f32 = 9.0;
/// Island marker colour (linear RGBA).
pub const ISLAND_MARKER_COLOR: [f32; 4] = [0.95, 0.1, 0.85, 1.0];

static OVERLAY_ON: AtomicBool = AtomicBool::new(false);

/// The Overhang Analysis toggle (a session-wide view setting, like a display style: it is not
/// stored in the document).
pub fn overlay_enabled() -> bool {
    OVERLAY_ON.load(Ordering::Relaxed)
}

/// Turns the overlay and the analysis jobs on or off.
pub fn set_overlay_enabled(on: bool) {
    OVERLAY_ON.store(on, Ordering::Relaxed);
}

/// Pass of an analysis job.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Quality {
    /// While dragging: cell ×2, ≤ 128² rays.
    Coarse,
    /// On release: the setup's settings.
    Fine,
}

/// Options of one pass for `settings` on machine `m` (tolerance resolved).
pub fn analysis_opts(settings: &AnalysisSettings, m: &Machine, q: Quality) -> OverhangOpts {
    let mut o = OverhangOpts::from_settings(settings, m);
    if q == Quality::Coarse {
        o.support = Some(SupportOpts {
            cell: settings.support_cell * COARSE_CELL_FACTOR,
            max_rays: settings.max_rays.clamp(1, COARSE_MAX_RAYS),
        });
    }
    o
}

/// The RND-08 overhang shading of an item drawn at `t` (body-local → plate = world) whose
/// lowest plate z is `z_min`, from AM-02 [`shader_params`]: the build direction
/// `R·d_local`, `α = acos(cos α)`, the plate height (the contact plane's zero) and the contact
/// slab.
pub fn overlay_shading(t: &Transform, z_min: f64, opts: &OverhangOpts) -> Shading {
    let p = shader_params(t, z_min, opts);
    let up = Dir3::new(t.matrix3 * p.build_dir_local, &Tolerance::default()).unwrap_or(Dir3::Z);
    // The plane gives h(local) = n·local + w (rigid T: |row| = 1); the shader computes
    // h = dot(world, up) − plate_height, so plate_height = dot(T(0), up) − w.
    let plate_height = t.translation.dot(up.get()) - p.contact_plane.1;
    Shading::Analysis(Analysis::Overhang {
        build_up: up,
        critical: p.cos_critical.clamp(-1.0, 1.0).acos(),
        plate_height,
        plate_tol: p.contact_tolerance,
    })
}

/// The overlay's class of every face, evaluated on the CPU the way the RND-08 shader does it
/// (f32, `amg_render` `analysis_common.wgsl` `overhang_class`): the instance record of
/// `shading` for a display mesh rebased to the mesh's bounding-box centre, the flat world
/// normal of the face and the height at the face centroid (one representative fragment).
/// Faces without a class (shading is not the overhang mode) are `Degenerate`.
pub fn shader_face_classes(m: &IndexedMesh, t: &Transform, shading: &Shading) -> Vec<FaceClass> {
    let Shading::Analysis(Analysis::Overhang {
        build_up,
        critical,
        plate_height,
        plate_tol,
    }) = shading
    else {
        return vec![FaceClass::Degenerate; m.tris.len()];
    };
    let b = build_up.get();
    let bounds = m.aabb();
    let origin = if bounds.is_empty() {
        Point3::ZERO
    } else {
        bounds.center()
    };
    // `build_analysis_record`: dir = (b, cos α), height = (Lᵀ b, dot(T(origin), b) − plate).
    let dir = b.as_vec3();
    let cos_a = critical.cos() as f32;
    let lt_b = (t.matrix3.transpose() * b).as_vec3();
    let h0 = (t.transform_point3(origin).dot(b) - plate_height) as f32;
    let tol = *plate_tol as f32;
    m.tris
        .iter()
        .map(|tri| {
            let [p0, p1, p2] = tri.map(|i| m.positions[i as usize]);
            let [w0, w1, w2] = [p0, p1, p2].map(|p| t.transform_point3(p));
            let n = (w1 - w0).cross(w2 - w0).as_vec3();
            let local = [p0, p1, p2].map(|p| (p - origin).as_vec3());
            let centroid = (local[0] + local[1] + local[2]) / 3.0;
            let height = centroid.dot(lt_b) + h0;
            shader_class_f32(n, dir, cos_a, height, tol)
        })
        .collect()
}

/// `overhang_class` of `analysis_common.wgsl`, in f32.
fn shader_class_f32(n: Vec3, d: Vec3, cos_critical: f32, height: f32, tol: f32) -> FaceClass {
    if n.dot(n) == 0.0 || !n.is_finite() {
        return FaceClass::Degenerate;
    }
    let c = -n.normalize().dot(d.normalize());
    if c > 0.0 && height <= tol {
        FaceClass::Contact
    } else if c > cos_critical {
        FaceClass::Overhang
    } else {
        FaceClass::Supported
    }
}

/// What an analysis was computed for.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct AnalysisKey {
    mesh: usize,
    placement: Placement,
    settings: AnalysisSettings,
}

/// The latest analysis of one item.
#[derive(Clone, Debug)]
pub struct ItemAnalysis {
    /// Coarse or fine.
    pub quality: Quality,
    /// The placement analysed.
    pub placement: Placement,
    /// The settings analysed.
    pub settings: AnalysisSettings,
    /// The report; `report.face_class` is the per-face class buffer.
    pub report: Arc<OverhangReport>,
    /// Job run time (spatial build excluded when cached).
    pub elapsed: Duration,
    /// When the result reached the UI thread.
    pub arrived: Instant,
    key: AnalysisKey,
}

impl ItemAnalysis {
    /// The per-face class buffer (face order of the mesh).
    pub fn face_classes(&self) -> &[FaceClass] {
        &self.report.face_class
    }

    /// The metrics.
    pub fn metrics(&self) -> &OverhangMetrics {
        &self.report.metrics
    }
}

/// Metrics of the whole plate: sums over the analysed items (heights: the tallest).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PlateMetrics {
    /// Items with an analysis / mesh items on the plate.
    pub analysed: usize,
    /// Mesh items on the plate.
    pub items: usize,
    /// Σ overhang area, mm².
    pub overhang_area: f64,
    /// Σ projected overhang area, mm².
    pub overhang_projected_area: f64,
    /// Σ contact area, mm².
    pub contact_area: f64,
    /// Σ footprint hull area, mm².
    pub footprint_area: f64,
    /// Tallest item, mm.
    pub height: f64,
    /// Σ islands.
    pub island_count: usize,
    /// Σ support volume, mm³ (items with an estimate).
    pub support_volume: f64,
    /// Σ support volume standing on the plate, mm³.
    pub support_on_plate: f64,
    /// Σ support volume standing on the part, mm³.
    pub support_on_part: f64,
    /// Items reported to tip over.
    pub unstable: usize,
    /// Some item's numbers are approximate (open mesh) or coarse.
    pub approximate: bool,
}

impl PlateMetrics {
    /// Adds one item's metrics.
    pub fn add(&mut self, m: &OverhangMetrics, coarse: bool) {
        self.analysed += 1;
        self.overhang_area += m.overhang_area;
        self.overhang_projected_area += m.overhang_projected_area;
        self.contact_area += m.contact_area;
        self.footprint_area += m.footprint_area;
        self.height = self.height.max(m.height);
        self.island_count += m.island_count;
        if let Some(s) = &m.support {
            self.support_volume += s.volume;
            self.support_on_plate += s.on_plate_volume;
            self.support_on_part += s.on_part_volume;
        }
        if m.stable == Some(false) {
            self.unstable += 1;
        }
        self.approximate |= m.approximate || coarse;
    }
}

/// One job input.
struct Input {
    item: ItemId,
    mesh: Arc<IndexedMesh>,
    spatial: Option<Arc<MeshSpatial>>,
    placement: Placement,
    settings: AnalysisSettings,
    opts: OverhangOpts,
    quality: Quality,
}

struct Output {
    item: ItemId,
    mesh: Arc<IndexedMesh>,
    spatial: Arc<MeshSpatial>,
    placement: Placement,
    settings: AnalysisSettings,
    quality: Quality,
    report: OverhangReport,
    elapsed: Duration,
}

struct Pending {
    /// (item, mesh, placement, settings, quality) of the inputs.
    inputs: Vec<(ItemId, usize, Placement, AnalysisSettings, Quality)>,
    started: Instant,
    handle: JobHandle<Vec<Output>, AmError>,
}

impl Pending {
    fn all_coarse(&self) -> bool {
        self.inputs.iter().all(|i| i.4 == Quality::Coarse)
    }
}

/// One mesh item of the active plate as the layer sees it this frame.
pub(crate) struct Want {
    pub item: ItemId,
    pub mesh: Arc<IndexedMesh>,
    pub placement: Placement,
}

/// Counters for tests and the report.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OverhangCounters {
    /// Coarse jobs spawned.
    pub coarse_spawned: u64,
    /// Fine jobs spawned.
    pub fine_spawned: u64,
    /// Coarse results stored.
    pub coarse_done: u64,
    /// Fine results stored.
    pub fine_done: u64,
    /// Running jobs cancelled by a newer input.
    pub superseded: u64,
}

/// The overhang state of a document's plate layer.
#[derive(Default)]
pub struct OverhangState {
    analyses: BTreeMap<ItemId, ItemAnalysis>,
    spatial: HashMap<usize, (Arc<IndexedMesh>, Arc<MeshSpatial>)>,
    pending: Option<Pending>,
    last_inputs: BTreeMap<ItemId, (Placement, AnalysisSettings)>,
    counters: OverhangCounters,
}

impl std::fmt::Debug for OverhangState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OverhangState")
            .field("analyses", &self.analyses.len())
            .field("pending", &self.pending.is_some())
            .field("counters", &self.counters)
            .finish_non_exhaustive()
    }
}

impl OverhangState {
    /// The latest analysis of `item`.
    pub fn analysis(&self, item: ItemId) -> Option<&ItemAnalysis> {
        self.analyses.get(&item)
    }

    /// Every item's latest analysis.
    pub fn analyses(&self) -> &BTreeMap<ItemId, ItemAnalysis> {
        &self.analyses
    }

    /// An analysis job is running.
    pub fn busy(&self) -> bool {
        self.pending.is_some()
    }

    /// Job counters.
    pub fn counters(&self) -> OverhangCounters {
        self.counters
    }

    /// Whether `item`'s analysis is the fine one for exactly this input.
    pub(crate) fn is_fine_for(
        &self,
        item: ItemId,
        mesh: &Arc<IndexedMesh>,
        placement: &Placement,
        settings: &AnalysisSettings,
    ) -> bool {
        self.analyses.get(&item).is_some_and(|a| {
            a.quality == Quality::Fine && a.key == key_of(mesh, placement, settings)
        })
    }

    /// Plate metrics over `items` (the mesh items of the active plate).
    pub fn plate_metrics(&self, items: &[ItemId]) -> PlateMetrics {
        let mut p = PlateMetrics {
            items: items.len(),
            ..PlateMetrics::default()
        };
        for id in items {
            if let Some(a) = self.analyses.get(id) {
                p.add(a.metrics(), a.quality == Quality::Coarse);
            }
        }
        p
    }

    /// Stops the running job and forgets the frame-to-frame inputs (overlay off, workspace
    /// left). Results are kept: they are keyed by their inputs.
    pub(crate) fn suspend(&mut self) {
        if let Some(p) = self.pending.take() {
            p.handle.cancel();
        }
        self.last_inputs.clear();
    }

    /// Forgets everything (another setup became active).
    pub(crate) fn reset(&mut self) {
        self.suspend();
        self.analyses.clear();
    }

    fn poll(&mut self) {
        let Some(p) = &mut self.pending else { return };
        let Some(res) = p.handle.try_take() else {
            return;
        };
        self.pending = None;
        match res {
            Ok(out) => {
                let now = Instant::now();
                for o in out {
                    let ptr = Arc::as_ptr(&o.mesh) as usize;
                    self.spatial
                        .insert(ptr, (Arc::clone(&o.mesh), Arc::clone(&o.spatial)));
                    match o.quality {
                        Quality::Coarse => self.counters.coarse_done += 1,
                        Quality::Fine => self.counters.fine_done += 1,
                    }
                    let key = key_of(&o.mesh, &o.placement, &o.settings);
                    self.analyses.insert(
                        o.item,
                        ItemAnalysis {
                            quality: o.quality,
                            placement: o.placement,
                            settings: o.settings,
                            report: Arc::new(o.report),
                            elapsed: o.elapsed,
                            arrived: now,
                            key,
                        },
                    );
                }
            }
            Err(e) => tracing::debug!("am: overhang analysis ended: {e}"),
        }
    }

    /// One frame: takes finished results, then spawns (or supersedes) the job the plate needs.
    /// `want` are the mesh items of the active plate with their shown placements, `settings`
    /// the shown (possibly previewed) settings.
    pub(crate) fn update(
        &mut self,
        jobs: &amg_jobs::JobSystem,
        want: &[Want],
        settings: &AnalysisSettings,
        machine: &Machine,
    ) {
        self.poll();
        self.analyses
            .retain(|id, _| want.iter().any(|w| w.item == *id));
        self.spatial
            .retain(|_, (m, _)| want.iter().any(|w| Arc::ptr_eq(&w.mesh, m)));
        let mut inputs = Vec::new();
        let mut now_inputs = BTreeMap::new();
        for w in want {
            now_inputs.insert(w.item, (w.placement, settings.clone()));
            if self.is_fine_for(w.item, &w.mesh, &w.placement, settings) {
                continue;
            }
            // Moving: the input changed since the last frame (a drag) → coarse; holding still
            // for a frame ("release") → fine.
            let moving = self
                .last_inputs
                .get(&w.item)
                .is_some_and(|(p, s)| *p != w.placement || s != settings);
            let coarse_fresh = self.analyses.get(&w.item).is_some_and(|a| {
                a.quality == Quality::Coarse && a.key == key_of(&w.mesh, &w.placement, settings)
            });
            if moving && coarse_fresh {
                continue;
            }
            let quality = if moving {
                Quality::Coarse
            } else {
                Quality::Fine
            };
            let ptr = Arc::as_ptr(&w.mesh) as usize;
            inputs.push(Input {
                item: w.item,
                mesh: Arc::clone(&w.mesh),
                spatial: self.spatial.get(&ptr).map(|(_, s)| Arc::clone(s)),
                placement: w.placement,
                settings: settings.clone(),
                opts: analysis_opts(settings, machine, quality),
                quality,
            });
        }
        self.last_inputs = now_inputs;
        if inputs.is_empty() {
            return;
        }
        let summary: Vec<_> = inputs
            .iter()
            .map(|i| {
                (
                    i.item,
                    Arc::as_ptr(&i.mesh) as usize,
                    i.placement,
                    i.settings.clone(),
                    i.quality,
                )
            })
            .collect();
        if let Some(p) = &self.pending {
            if p.inputs == summary {
                return;
            }
            // Let a young coarse job finish: its numbers are close and the newest input runs
            // right after it.
            if p.all_coarse() && p.started.elapsed() < COARSE_GRACE {
                return;
            }
        }
        if let Some(p) = self.pending.take() {
            p.handle.cancel();
            self.counters.superseded += 1;
        }
        let fine = inputs.iter().any(|i| i.quality == Quality::Fine);
        if fine {
            self.counters.fine_spawned += 1;
        } else {
            self.counters.coarse_spawned += 1;
        }
        let label = if fine {
            "Overhang analysis"
        } else {
            "Overhang analysis (coarse)"
        };
        let handle = jobs.spawn_fn(
            label,
            SpawnOptions::default(),
            move |ctx| -> Result<Vec<Output>, AmError> {
                let mut out = Vec::with_capacity(inputs.len());
                for i in inputs {
                    ctx.checkpoint()?;
                    let spatial = match i.spatial {
                        Some(s) => s,
                        None => Arc::new(MeshSpatial::build(
                            Arc::clone(&i.mesh),
                            &BvhOpts::default(),
                            ctx,
                        )?),
                    };
                    let t0 = Instant::now();
                    let t = effective_transform(&i.mesh, &i.placement);
                    let report = analyze_overhang(&spatial, &t, &i.opts, ctx)?;
                    out.push(Output {
                        item: i.item,
                        mesh: i.mesh,
                        spatial,
                        placement: i.placement,
                        settings: i.settings,
                        quality: i.quality,
                        report,
                        elapsed: t0.elapsed(),
                    });
                }
                Ok(out)
            },
        );
        self.pending = Some(Pending {
            inputs: summary,
            started: Instant::now(),
            handle,
        });
    }
}

fn key_of(
    mesh: &Arc<IndexedMesh>,
    placement: &Placement,
    settings: &AnalysisSettings,
) -> AnalysisKey {
    AnalysisKey {
        mesh: Arc::as_ptr(mesh) as usize,
        placement: *placement,
        settings: settings.clone(),
    }
}

/// Island markers of an item drawn at `t`: the plate positions of the report's island
/// vertices.
pub fn island_points(m: &IndexedMesh, report: &OverhangReport, t: &Transform) -> Vec<Point3> {
    report
        .islands
        .iter()
        .filter_map(|v| m.positions.get(v.0 as usize))
        .map(|p| t.transform_point3(*p))
        .collect()
}

/// Rows of the metrics panel: (label, value).
pub fn metric_rows(m: &OverhangMetrics) -> Vec<(&'static str, String)> {
    let mut rows = vec![
        ("Overhang area", format!("{:.1} mm²", m.overhang_area)),
        (
            "Projected overhang",
            format!("{:.1} mm²", m.overhang_projected_area),
        ),
        ("Contact area", format!("{:.1} mm²", m.contact_area)),
        ("Footprint", format!("{:.1} mm²", m.footprint_area)),
        ("Height", format!("{:.2} mm", m.height)),
        ("Islands", m.island_count.to_string()),
    ];
    if let Some(s) = &m.support {
        rows.push(("Support volume", format!("{:.0} mm³", s.volume)));
        rows.push((
            "  on plate / on part",
            format!("{:.0} / {:.0} mm³", s.on_plate_volume, s.on_part_volume),
        ));
        rows.push(("Support cell", format!("{:.2} mm, {} rays", s.cell, s.rays)));
    }
    rows.push((
        "Stable",
        match m.stable {
            Some(true) => "yes".to_owned(),
            Some(false) => match m.com_margin {
                Some(c) => format!("no (centre of mass {:.1} mm outside)", -c),
                None => "no".to_owned(),
            },
            None => "n/a".to_owned(),
        },
    ));
    rows
}

/// Rows of the plate metrics.
pub fn plate_rows(p: &PlateMetrics) -> Vec<(&'static str, String)> {
    vec![
        ("Items analysed", format!("{} / {}", p.analysed, p.items)),
        ("Overhang area", format!("{:.1} mm²", p.overhang_area)),
        (
            "Projected overhang",
            format!("{:.1} mm²", p.overhang_projected_area),
        ),
        ("Contact area", format!("{:.1} mm²", p.contact_area)),
        ("Tallest item", format!("{:.2} mm", p.height)),
        ("Islands", p.island_count.to_string()),
        ("Support volume", format!("{:.0} mm³", p.support_volume)),
        (
            "  on plate / on part",
            format!("{:.0} / {:.0} mm³", p.support_on_plate, p.support_on_part),
        ),
        ("Items that tip over", p.unstable.to_string()),
    ]
}

/// Lowest plate z of the vertices of `m` at `t` (tests; the layer takes it from the fit check).
pub fn z_min_of(m: &IndexedMesh, t: &Transform) -> f64 {
    m.positions
        .iter()
        .map(|p| t.transform_point3(*p).z)
        .fold(f64::INFINITY, f64::min)
}

/// `true` for a unit build direction along +Z (the plate frame's), within `tol`.
pub fn is_plate_up(d: DVec3, tol: f64) -> bool {
    (d - DVec3::Z).length() <= tol
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact copies of the inputs")]
mod tests {
    use super::*;
    use amg_am::machine::{MachineKey, MachineLibrary};
    use amg_am::overhang::classify_faces;
    use amg_core::glam::DQuat;
    use amg_jobs::JobCtx;

    fn kcm() -> Machine {
        MachineLibrary::builtin()
            .get(&MachineKey("builtin:creality/ender-3-v3-ke-kcm".to_owned()))
            .cloned()
            .expect("KCM")
    }

    #[test]
    fn coarse_pass_doubles_the_cell_and_caps_the_rays() {
        let m = kcm();
        let s = AnalysisSettings::for_machine(&m);
        let f = analysis_opts(&s, &m, Quality::Fine);
        let c = analysis_opts(&s, &m, Quality::Coarse);
        let (fs, cs) = (f.support.unwrap(), c.support.unwrap());
        assert_eq!(fs.cell, s.support_cell);
        assert_eq!(fs.max_rays, s.max_rays);
        assert_eq!(cs.cell, 2.0 * s.support_cell);
        assert_eq!(cs.max_rays, COARSE_MAX_RAYS);
        assert_eq!(c.critical_angle, f.critical_angle);
        assert_eq!(c.contact_tolerance, f.contact_tolerance);
    }

    /// A unit cube with a downward-facing 45°-ish chamfer replaced by a plain cube; rotated.
    fn tilted_cube() -> (IndexedMesh, Transform) {
        let p: Vec<Point3> = (0..8)
            .map(|i| {
                Point3::new(
                    f64::from(i & 1) * 10.0,
                    f64::from((i >> 1) & 1) * 10.0,
                    f64::from((i >> 2) & 1) * 10.0,
                )
            })
            .collect();
        let t = vec![
            [0, 2, 1],
            [1, 2, 3],
            [4, 5, 6],
            [5, 7, 6],
            [0, 1, 4],
            [1, 5, 4],
            [2, 6, 3],
            [3, 6, 7],
            [0, 4, 2],
            [2, 4, 6],
            [1, 3, 5],
            [3, 7, 5],
        ];
        let m = IndexedMesh::new(p, t).unwrap();
        let r = DQuat::from_rotation_x(0.3) * DQuat::from_rotation_y(0.2);
        let pl = Placement {
            rotation: r,
            xy: amg_core::glam::DVec2::new(100.0, 100.0),
            z: amg_am::place::ZMode::OnPlate { elevation: 0.0 },
        };
        let tr = effective_transform(&m, &pl);
        (m, tr)
    }

    #[test]
    fn overlay_shading_follows_shader_params_and_matches_the_cpu_classes() {
        let m = kcm();
        let s = AnalysisSettings::for_machine(&m);
        let opts = analysis_opts(&s, &m, Quality::Fine);
        let (mesh, t) = tilted_cube();
        let z_min = z_min_of(&mesh, &t);
        let sh = overlay_shading(&t, z_min, &opts);
        let Shading::Analysis(Analysis::Overhang {
            build_up,
            critical,
            plate_height,
            plate_tol,
        }) = &sh
        else {
            panic!("{sh:?}")
        };
        assert!(is_plate_up(build_up.get(), 1e-12));
        assert!((critical - s.critical_angle).abs() < 1e-12);
        assert!((plate_height - z_min).abs() < 1e-9);
        assert_eq!(*plate_tol, opts.contact_tolerance);
        let (cpu, _) = classify_faces(&mesh, &t, &opts, &JobCtx::detached()).unwrap();
        let gpu = shader_face_classes(&mesh, &t, &sh);
        assert_eq!(cpu, gpu);
        assert!(cpu.contains(&FaceClass::Contact) || cpu.contains(&FaceClass::Overhang));
    }

    #[test]
    fn metric_rows_name_every_number() {
        let m = OverhangMetrics {
            overhang_area: 12.0,
            overhang_projected_area: 10.0,
            contact_area: 100.0,
            footprint_area: 100.0,
            height: 10.0,
            island_count: 2,
            stable: Some(false),
            com_margin: Some(-1.5),
            support: None,
            approximate: false,
        };
        let rows = metric_rows(&m);
        assert!(rows.iter().any(|(l, v)| *l == "Islands" && v == "2"));
        assert!(
            rows.iter()
                .any(|(l, v)| *l == "Stable" && v.contains("1.5 mm outside"))
        );
        let mut p = PlateMetrics::default();
        p.add(&m, false);
        p.add(&m, true);
        assert_eq!(p.analysed, 2);
        assert_eq!(p.island_count, 4);
        assert_eq!(p.unstable, 2);
        assert!(p.approximate);
        assert_eq!(p.overhang_area, 24.0);
        assert!(!plate_rows(&p).is_empty());
    }
}
