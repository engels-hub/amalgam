//! The Mesh Analysis panel: the MESH-02 report (plus MESH-19 self-intersections) of the
//! selected mesh body, computed as a cancellable background job, with its defective faces on
//! the `Error` highlight layer.

use std::sync::Arc;

use amg_doc::{BodyGeom, BodyId, Evaluated};
use amg_jobs::{JobCtx, JobHandle, SpawnOptions};
use amg_mesh::{AnalysisReport, AnalyzeOpts, IndexedMesh, MeshError};

use crate::app::{AppCx, CmdEnv};
use crate::module::PanelId;
use crate::panel::Panel;
use crate::pick::PickTarget;
use crate::scene::HighlightLayer;
use crate::select::SelItem;

/// The panel id.
pub const ANALYSIS_PANEL: PanelId = PanelId("mesh.analysis");

/// Defective faces by category (triangle indices, sorted, unique per list).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DefectFaces {
    /// Faces on a boundary edge (holes, open borders).
    pub boundary: Vec<u32>,
    /// Faces on an edge shared by more than two faces.
    pub non_manifold_edges: Vec<u32>,
    /// Faces around a non-manifold vertex.
    pub non_manifold_vertices: Vec<u32>,
    /// Faces on an edge whose two faces disagree in orientation.
    pub orientation: Vec<u32>,
    /// Degenerate faces.
    pub degenerate: Vec<u32>,
    /// Duplicate faces.
    pub duplicate: Vec<u32>,
    /// Faces that intersect another face.
    pub self_intersecting: Vec<u32>,
}

impl DefectFaces {
    /// Every defective face once, sorted.
    pub fn all(&self) -> Vec<u32> {
        let mut v: Vec<u32> = [
            &self.boundary,
            &self.non_manifold_edges,
            &self.non_manifold_vertices,
            &self.orientation,
            &self.degenerate,
            &self.duplicate,
            &self.self_intersecting,
        ]
        .into_iter()
        .flatten()
        .copied()
        .collect();
        v.sort_unstable();
        v.dedup();
        v
    }
}

/// The analysis of one mesh.
#[derive(Clone, Debug)]
pub struct MeshAnalysis {
    /// The MESH-02 report, with self-intersections.
    pub report: AnalysisReport,
    /// The faces to highlight.
    pub defects: DefectFaces,
}

impl MeshAnalysis {
    /// The highlight targets of every defective face of `body` (instance resolved by the app).
    pub fn highlight_targets(&self, body: BodyId) -> Vec<PickTarget> {
        self.defects
            .all()
            .into_iter()
            .map(|f| PickTarget::Body {
                body,
                instance: 0,
                sub: Some((amg_doc::TopoKind::Face, f)),
            })
            .collect()
    }

    /// The report rows: `(label, value, is a defect)`.
    pub fn rows(&self) -> Vec<(&'static str, String, bool)> {
        let r = &self.report;
        let si = r.self_intersecting_pairs.as_ref().map_or(0, Vec::len);
        let yes = |b: bool| if b { "yes" } else { "no" }.to_owned();
        vec![
            ("Watertight", yes(r.is_watertight()), !r.is_watertight()),
            ("Faces", r.faces.to_string(), false),
            ("Vertices", r.vertices.to_string(), false),
            ("Shells", r.shells.len().to_string(), false),
            (
                "Holes",
                r.boundary_loops.len().to_string(),
                !r.boundary_loops.is_empty(),
            ),
            (
                "Boundary edges",
                r.boundary_edges.to_string(),
                r.boundary_edges > 0,
            ),
            (
                "Non-manifold edges",
                r.non_manifold_edges.len().to_string(),
                !r.non_manifold_edges.is_empty(),
            ),
            (
                "Non-manifold vertices",
                r.non_manifold_vertices.len().to_string(),
                !r.non_manifold_vertices.is_empty(),
            ),
            (
                "Orientation conflicts",
                r.orientation_conflicts.to_string(),
                r.orientation_conflicts > 0,
            ),
            ("Inverted", yes(r.is_inverted), r.is_inverted),
            (
                "Degenerate faces",
                r.degenerate_faces.len().to_string(),
                !r.degenerate_faces.is_empty(),
            ),
            (
                "Duplicate faces",
                r.duplicate_faces.len().to_string(),
                !r.duplicate_faces.is_empty(),
            ),
            ("Self-intersections", si.to_string(), si > 0),
            (
                "Unwelded seam edges",
                r.unwelded_boundary_edges.to_string(),
                r.unwelded_boundary_edges > 0,
            ),
            ("Area", format!("{:.2} mm²", r.area), false),
            (
                "Volume",
                r.volume.map_or_else(
                    || "– (not watertight)".to_owned(),
                    |v| format!("{v:.2} mm³"),
                ),
                false,
            ),
        ]
    }
}

/// Analyses `mesh` (MESH-02 with MESH-19) and collects the defective faces. Cancellable.
///
/// # Errors
/// [`MeshError::Cancelled`] (or another analysis error).
pub fn analyse(mesh: &IndexedMesh, ctx: &JobCtx) -> Result<MeshAnalysis, MeshError> {
    let opts = AnalyzeOpts {
        self_intersections: true,
        ..AnalyzeOpts::default()
    };
    let report = amg_mesh::analyze(mesh, &opts, ctx)?;
    ctx.checkpoint()?;
    let defects = defect_faces(mesh, &report);
    Ok(MeshAnalysis { report, defects })
}

fn defect_faces(mesh: &IndexedMesh, r: &AnalysisReport) -> DefectFaces {
    // Directed edge per face corner, sorted by undirected key: runs are the edge's faces.
    let mut edges: Vec<(u64, u32, bool)> = Vec::with_capacity(mesh.tris.len() * 3);
    for (f, t) in (0u32..).zip(&mesh.tris) {
        if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
            continue;
        }
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let key = (u64::from(a.min(b)) << 32) | u64::from(a.max(b));
            edges.push((key, f, a < b));
        }
    }
    edges.sort_unstable();
    let mut d = DefectFaces::default();
    for run in edges.chunk_by(|x, y| x.0 == y.0) {
        match run {
            [one] => d.boundary.push(one.1),
            [a, b] => {
                if a.2 == b.2 {
                    d.orientation.extend([a.1, b.1]);
                }
            }
            many => d.non_manifold_edges.extend(many.iter().map(|e| e.1)),
        }
    }
    if !r.non_manifold_vertices.is_empty() {
        let mut flag = vec![false; mesh.positions.len()];
        for &v in &r.non_manifold_vertices {
            if let Some(x) = flag.get_mut(v as usize) {
                *x = true;
            }
        }
        for (f, t) in (0u32..).zip(&mesh.tris) {
            if t.iter()
                .any(|&v| flag.get(v as usize).copied().unwrap_or(false))
            {
                d.non_manifold_vertices.push(f);
            }
        }
    }
    d.degenerate = r.degenerate_faces.iter().map(|f| f.0).collect();
    d.duplicate = r.duplicate_faces.iter().map(|f| f.0).collect();
    if let Some(p) = &r.self_intersecting_pairs {
        d.self_intersecting = p.iter().flat_map(|[a, b]| [a.0, b.0]).collect();
    }
    for v in [
        &mut d.boundary,
        &mut d.non_manifold_edges,
        &mut d.non_manifold_vertices,
        &mut d.orientation,
        &mut d.degenerate,
        &mut d.duplicate,
        &mut d.self_intersecting,
    ] {
        v.sort_unstable();
        v.dedup();
    }
    d
}

/// The mesh of `body`, if it is a mesh body.
pub(crate) fn mesh_of(ev: &Evaluated, body: BodyId) -> Option<Arc<IndexedMesh>> {
    match &ev.body(body)?.geom {
        BodyGeom::Mesh(m) => Some(Arc::clone(m)),
        _ => None,
    }
}

/// The mesh bodies of `ev`, in id order.
pub(crate) fn mesh_bodies(ev: &Evaluated) -> impl Iterator<Item = BodyId> + '_ {
    ev.bodies()
        .filter(|(_, b)| matches!(b.geom, BodyGeom::Mesh(_)))
        .map(|(id, _)| id)
}

/// The panel's body: the first selected mesh body (or the body of a selected mesh face),
/// else `current` while it is still a mesh body, else the first mesh body.
fn target_body(env: &CmdEnv<'_>, current: Option<BodyId>) -> Option<BodyId> {
    let ev = env.evaluated;
    let selected = env
        .selection
        .items()
        .iter()
        .filter_map(|i| i.target.body())
        .find(|b| mesh_of(ev, *b).is_some());
    selected
        .or(current.filter(|b| mesh_of(ev, *b).is_some()))
        .or_else(|| mesh_bodies(ev).next())
}

type AnalysisJob = JobHandle<Arc<MeshAnalysis>, MeshError>;

/// The Mesh Analysis panel.
pub struct AnalysisPanel {
    body: Option<BodyId>,
    /// The mesh the result (or the running job) is for.
    mesh: Option<Arc<IndexedMesh>>,
    job: Option<AnalysisJob>,
    result: Option<Arc<MeshAnalysis>>,
    error: Option<String>,
    highlight: bool,
    /// What the `Error` layer shows: `(body, result)`; `None` = cleared.
    shown: Option<Option<(BodyId, usize)>>,
}

impl Default for AnalysisPanel {
    fn default() -> Self {
        AnalysisPanel {
            body: None,
            mesh: None,
            job: None,
            result: None,
            error: None,
            highlight: true,
            shown: None,
        }
    }
}

impl std::fmt::Debug for AnalysisPanel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AnalysisPanel")
            .field("body", &self.body)
            .field("running", &self.job.is_some())
            .field("done", &self.result.is_some())
            .finish_non_exhaustive()
    }
}

/// Creates the panel (`PanelDef::create`).
pub fn create_panel() -> Box<dyn Panel> {
    Box::new(AnalysisPanel::default())
}

impl AnalysisPanel {
    /// Restarts the analysis when the body or its mesh changed; collects a finished job.
    fn sync(&mut self, cx: &mut AppCx<'_>) {
        let (body, mesh) = {
            let env = cx.env();
            let body = target_body(&env, self.body);
            (body, body.and_then(|b| mesh_of(env.evaluated, b)))
        };
        let same = self.body == body
            && match (&self.mesh, &mesh) {
                (Some(a), Some(b)) => Arc::ptr_eq(a, b),
                (None, None) => true,
                _ => false,
            };
        if !same {
            if let Some(j) = self.job.take() {
                j.cancel();
            }
            self.body = body;
            self.result = None;
            self.error = None;
            self.mesh.clone_from(&mesh);
            if let Some(m) = mesh {
                self.job = Some(cx.jobs().spawn_fn(
                    "Mesh analysis",
                    SpawnOptions::default(),
                    move |ctx: &JobCtx| analyse(&m, ctx).map(Arc::new),
                ));
            }
        }
        if let Some(j) = &mut self.job
            && let Some(r) = j.try_take()
        {
            self.job = None;
            match r {
                Ok(a) => self.result = Some(a),
                Err(amg_jobs::JobError::Cancelled) => {}
                Err(e) => self.error = Some(e.to_string()),
            }
        }
        if self.job.is_some() {
            cx.request_repaint_after(std::time::Duration::from_millis(50));
        }
        // The Error layer follows the result and the toggle.
        let want = match (&self.result, self.body, self.highlight) {
            (Some(a), Some(b), true) => Some((b, Arc::as_ptr(a) as usize)),
            _ => None,
        };
        if self.shown != Some(want) {
            self.shown = Some(want);
            let targets = match (&self.result, want) {
                (Some(a), Some((b, _))) => a.highlight_targets(b),
                _ => Vec::new(),
            };
            cx.set_highlight(HighlightLayer::Error, targets);
        }
    }
}

impl Panel for AnalysisPanel {
    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut AppCx<'_>) {
        self.sync(cx);
        let Some(body) = self.body else {
            ui.label("No mesh body in the design.");
            return;
        };
        ui.heading(format!("Body {}", body_label(body)));
        if self.job.is_some() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("Analysing…");
                if ui.button("Cancel analysis").clicked()
                    && let Some(j) = self.job.take()
                {
                    j.cancel();
                }
            });
        }
        if let Some(e) = &self.error {
            ui.colored_label(ui.visuals().error_fg_color, e);
        }
        let Some(a) = self.result.clone() else {
            return;
        };
        egui::Grid::new("mesh_analysis_rows")
            .num_columns(1)
            .striped(true)
            .show(ui, |ui| {
                for (label, value, bad) in a.rows() {
                    let text = format!("{label}: {value}");
                    if bad {
                        ui.colored_label(ui.visuals().warn_fg_color, text);
                    } else {
                        ui.label(text);
                    }
                    ui.end_row();
                }
            });
        let n = a.defects.all().len();
        ui.checkbox(
            &mut self.highlight,
            format!("Highlight defects ({n} faces)"),
        );
        ui.horizontal(|ui| {
            if ui.button("Repair…").clicked() {
                select_body(cx, body);
                cx.run_command(super::ops::MeshOp::Repair.cmd());
            }
            if !a.report.is_watertight() && ui.button("Wrap…").clicked() {
                select_body(cx, body);
                cx.run_command(super::ops::MeshOp::Wrap.cmd());
            }
        });
        // Apply a toggle change this frame.
        self.sync(cx);
    }
}

/// `"<feature>.<slot>"` (bodies have no name accessor in the document yet).
pub(crate) fn body_label(body: BodyId) -> String {
    format!("{}.{}", body.origin.0, body.slot)
}

/// Makes `body` the selection (the op tools take a preselected body).
fn select_body(cx: &mut AppCx<'_>, body: BodyId) {
    if let Some(item) = SelItem::from_target(PickTarget::Body {
        body,
        instance: 0,
        sub: None,
    }) {
        let sel = cx.selection_mut();
        sel.clear();
        sel.add(item);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::Point3;

    fn tetra(open: bool) -> IndexedMesh {
        let p = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let mut t = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
        if open {
            t.pop();
        }
        IndexedMesh::new(p, t).unwrap()
    }

    #[test]
    fn a_closed_tetrahedron_has_no_defects() {
        let a = analyse(&tetra(false), &JobCtx::detached()).unwrap();
        assert!(a.report.is_watertight());
        assert_eq!(a.defects, DefectFaces::default());
        assert!(a.rows().iter().all(|(_, _, bad)| !bad));
    }

    #[test]
    fn an_open_tetrahedron_highlights_its_hole_faces() {
        let a = analyse(&tetra(true), &JobCtx::detached()).unwrap();
        assert!(!a.report.is_watertight());
        assert_eq!(a.defects.boundary, vec![0, 1, 2]);
        let t = a.highlight_targets(BodyId {
            origin: amg_doc::FeatureId(1),
            slot: 0,
        });
        assert_eq!(t.len(), 3);
        assert!(
            a.rows()
                .iter()
                .any(|(l, v, bad)| *l == "Holes" && v == "1" && *bad)
        );
    }

    #[test]
    fn flipped_and_duplicated_faces_are_found() {
        let mut m = tetra(false);
        m.tris[1] = [0, 3, 1];
        let a = analyse(&m, &JobCtx::detached()).unwrap();
        assert!(!a.defects.orientation.is_empty());
        assert!(a.defects.orientation.contains(&1));
        let mut m = tetra(false);
        m.tris.push([0, 2, 1]);
        let a = analyse(&m, &JobCtx::detached()).unwrap();
        assert!(!a.defects.duplicate.is_empty() || !a.defects.non_manifold_edges.is_empty());
    }
}
