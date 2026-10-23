//! Mesh repair pipeline (MESH-04): weld, degenerate and duplicate faces, non-manifold
//! resolution, small shells, orientation, hole filling and self-intersection removal, composed
//! from the MESH-02 analysis, the MESH-20 hole filler and the MESH-21 self-intersection remover.
//!
//! One round runs the enabled steps in that order, looking at a fresh [`AnalysisReport`] only
//! where the previous step changed the mesh, and a step does nothing when the analysis shows
//! nothing for it to do (so a clean mesh is returned bit-identical). Rounds repeat while the
//! defect counts do not grow, up to [`RepairOpts::max_rounds`], and stop at the first round that
//! changes nothing (a fixed point, so repairing a repaired mesh changes nothing). A later round
//! that makes the total defect count worse is discarded.
//!
//! Choices worth knowing:
//! - The weld merges only vertices on boundary edges: a seam is a pair of boundary chains, and
//!   interior copies made by non-manifold splitting must not be merged back. It runs in the
//!   first round only.
//! - Non-manifold edges are unzipped (see `nonmanifold`), not deleted: no geometry is lost.
//! - Orientation is outward by signed volume per edge-connected component, inward for a closed
//!   component that lies inside another (a cavity).

mod clean;
mod edges;
mod nonmanifold;
mod orient;

use std::time::Instant;

use amg_jobs::JobCtx;

use crate::analysis::{AnalysisReport, AnalyzeOpts, analyze};
use crate::error::{MeshError, MeshResult};
use crate::halfedge::{HalfEdgeMesh, NonManifoldPolicy};
use crate::holefill::{HoleFillOpts, fill_holes};
use crate::indexed::{IndexedMesh, WeldOpts};
use crate::selfint::{RemoveSelfIntersectionsOpts, remove_self_intersections_with};

/// Which shells [`RepairOpts::remove_small_shells`] removes: those with at most `max_faces`
/// faces whose extent is at most `max_volume_fraction` of the largest shell's. The extent of
/// a closed shell is its absolute signed volume, that of an open one its bounding-box volume.
/// The largest shell always stays.
#[derive(Clone, Debug)]
pub struct SmallShellFilter {
    /// Shells with more faces are kept.
    pub max_faces: usize,
    /// Shells larger than this fraction of the largest shell are kept.
    pub max_volume_fraction: f64,
}

/// What [`repair`] does.
#[derive(Clone, Debug)]
pub struct RepairOpts {
    /// Merge vertices on boundary edges within the tolerance (closes unwelded seams).
    pub weld: Option<WeldOpts>,
    /// Remove faces with zero area or a repeated vertex.
    pub remove_degenerate: bool,
    /// Remove faces repeated with the same vertex set.
    pub remove_duplicates: bool,
    /// Split non-manifold edges and vertices.
    pub fix_non_manifold: bool,
    /// Remove debris shells.
    pub remove_small_shells: Option<SmallShellFilter>,
    /// Make the orientation consistent and outward.
    pub orient: bool,
    /// Fill boundary loops.
    pub fill_holes: Option<HoleFillOpts>,
    /// Remove self-intersections by local delete and refill (MESH-21).
    pub remove_self_intersections: bool,
    /// Pipeline rounds at most (default 3).
    pub max_rounds: u32,
}

impl Default for RepairOpts {
    fn default() -> Self {
        RepairOpts {
            weld: Some(WeldOpts::default()),
            remove_degenerate: true,
            remove_duplicates: true,
            fix_non_manifold: true,
            remove_small_shells: Some(SmallShellFilter {
                max_faces: 12,
                max_volume_fraction: 1e-3,
            }),
            orient: true,
            fill_holes: Some(HoleFillOpts::default()),
            remove_self_intersections: true,
            max_rounds: 3,
        }
    }
}

/// A pipeline step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RepairStepKind {
    /// Merge boundary vertices.
    Weld,
    /// Remove degenerate faces.
    Degenerate,
    /// Remove duplicate faces.
    Duplicate,
    /// Split non-manifold edges and vertices.
    NonManifold,
    /// Remove small shells.
    SmallShells,
    /// Orient consistently and outward.
    Orient,
    /// Fill holes.
    FillHoles,
    /// Remove self-intersections.
    SelfIntersections,
}

/// One executed step.
#[derive(Clone, Debug)]
pub struct RepairStep {
    /// Which step.
    pub kind: RepairStepKind,
    /// Pipeline round, from 0.
    pub round: u32,
    /// Elements changed: vertices merged (weld), faces removed (degenerate, duplicate, small
    /// shells), edges re-paired plus vertices added (non-manifold), faces flipped (orient),
    /// faces added (holes), faces removed plus added (self-intersections). 0 = it had
    /// nothing to do.
    pub changed: usize,
    /// Wall time, ms.
    pub millis: u32,
}

/// A defect the pipeline could not remove.
#[derive(Clone, Debug)]
pub enum UnresolvedIssue {
    /// A boundary loop is still open (too large for [`HoleFillOpts::max_boundary_edges`], or
    /// hole filling is off or was refused).
    HoleTooLarge {
        /// Edges of the loop.
        boundary_edges: usize,
    },
    /// Intersecting face pairs remain.
    SelfIntersections {
        /// Number of pairs.
        pairs: usize,
    },
    /// Edges with other than two faces remain.
    NonManifold {
        /// Number of edges.
        edges: usize,
    },
}

/// What [`repair`] found and did.
#[derive(Clone, Debug)]
pub struct RepairReport {
    /// Analysis of the input (with self-intersections if they are to be removed).
    pub before: AnalysisReport,
    /// Analysis of the result (same options).
    pub after: AnalysisReport,
    /// Executed steps of the rounds that were kept, in order.
    pub steps: Vec<RepairStep>,
    /// What is left.
    pub unresolved: Vec<UnresolvedIssue>,
}

/// Makes every edge-connected component of `m` consistently oriented and points closed
/// components outward by signed volume (a closed component inside another is pointed
/// inward). Edges shared by more than two faces are not crossed. Vertex normals are not
/// touched. Returns the number of faces flipped.
///
/// # Errors
/// [`MeshError::Cancelled`], or the errors of [`IndexedMesh::validate`].
pub fn orient_consistently(m: &mut IndexedMesh, ctx: &JobCtx) -> MeshResult<usize> {
    orient::orient_consistently(m, ctx)
}

/// Per-category defect counts used to decide whether a round helped.
fn defects(r: &AnalysisReport) -> [usize; 8] {
    [
        r.boundary_loops.len(),
        r.non_manifold_edges.len(),
        r.non_manifold_vertices.len(),
        r.orientation_conflicts,
        r.degenerate_faces.len(),
        r.duplicate_faces.len(),
        r.self_intersecting_pairs.as_ref().map_or(0, Vec::len),
        usize::from(r.is_inverted),
    ]
}

struct Run<'a> {
    opts: &'a RepairOpts,
    ctx: &'a JobCtx,
    cur: IndexedMesh,
    rep: AnalysisReport,
    /// `rep` includes self-intersections.
    rep_si: bool,
    /// `cur` changed since `rep`.
    dirty: bool,
    steps: Vec<RepairStep>,
    round: u32,
}

impl Run<'_> {
    fn fresh(&mut self, si: bool) -> MeshResult<()> {
        if self.dirty || (si && !self.rep_si) {
            let o = AnalyzeOpts {
                self_intersections: si,
                ..AnalyzeOpts::default()
            };
            self.rep = analyze(&self.cur, &o, self.ctx)?;
            self.rep_si = si;
            self.dirty = false;
        }
        Ok(())
    }

    fn record(&mut self, kind: RepairStepKind, changed: usize, t0: Instant) {
        if changed > 0 {
            self.dirty = true;
        }
        self.steps.push(RepairStep {
            kind,
            round: self.round,
            changed,
            millis: u32::try_from(t0.elapsed().as_millis()).unwrap_or(u32::MAX),
        });
    }

    fn progress(&self, step: u32) {
        let rounds = self.opts.max_rounds.max(1) as f32;
        self.ctx
            .progress((self.round as f32 + step as f32 / 9.0) / rounds);
    }

    fn orient_step(&mut self) -> MeshResult<()> {
        let t0 = Instant::now();
        let flipped = orient::orient_consistently(&mut self.cur, self.ctx)?;
        self.record(RepairStepKind::Orient, flipped, t0);
        Ok(())
    }

    fn round(&mut self) -> MeshResult<()> {
        let o = self.opts;
        if let Some(w) = &o.weld {
            self.progress(0);
            let t0 = Instant::now();
            let mut changed = 0;
            if self.round == 0 {
                self.fresh(false)?;
                if !self.rep.boundary_loops.is_empty() {
                    changed = clean::weld_boundary(&mut self.cur, &self.rep, w.tolerance)?;
                }
            }
            self.record(RepairStepKind::Weld, changed, t0);
        }
        if o.remove_degenerate {
            self.progress(1);
            let t0 = Instant::now();
            self.fresh(false)?;
            let mut changed = 0;
            if !self.rep.degenerate_faces.is_empty() {
                let list = std::mem::take(&mut self.rep.degenerate_faces);
                changed = clean::remove_degenerate(&mut self.cur, &list, self.ctx)?;
                self.rep.degenerate_faces = list;
            }
            self.record(RepairStepKind::Degenerate, changed, t0);
        }
        if o.remove_duplicates {
            self.progress(2);
            let t0 = Instant::now();
            self.fresh(false)?;
            let mut changed = 0;
            if !self.rep.duplicate_faces.is_empty() {
                changed = clean::remove_duplicates(&mut self.cur);
            }
            self.record(RepairStepKind::Duplicate, changed, t0);
        }
        if o.fix_non_manifold {
            self.progress(3);
            let t0 = Instant::now();
            self.fresh(false)?;
            let mut changed = 0;
            if !self.rep.non_manifold_edges.is_empty() || !self.rep.non_manifold_vertices.is_empty()
            {
                let s = nonmanifold::split_non_manifold(&mut self.cur, self.ctx)?;
                changed = s.edges + s.vertices_added;
            }
            self.record(RepairStepKind::NonManifold, changed, t0);
        }
        if let Some(f) = &o.remove_small_shells {
            self.progress(4);
            let t0 = Instant::now();
            self.fresh(false)?;
            let changed = clean::remove_small_shells(&mut self.cur, &self.rep, f);
            self.record(RepairStepKind::SmallShells, changed, t0);
        }
        if o.orient {
            self.progress(5);
            self.fresh(false)?;
            if self.rep.orientation_conflicts > 0
                || self.rep.is_inverted
                || self.rep.shells.len() > 1
            {
                self.orient_step()?;
            } else {
                self.record(RepairStepKind::Orient, 0, Instant::now());
            }
        }
        if let Some(h) = &o.fill_holes {
            self.progress(6);
            let t0 = Instant::now();
            self.fresh(false)?;
            let mut added = 0;
            if !self.rep.boundary_loops.is_empty() {
                added = self.fill(h)?;
            }
            self.record(RepairStepKind::FillHoles, added, t0);
            if added > 0 && o.orient {
                // The patch may have closed an open shell that had been oriented by a guess.
                self.orient_step()?;
            }
        }
        if o.remove_self_intersections {
            self.progress(7);
            let t0 = Instant::now();
            self.fresh(true)?;
            let mut changed = 0;
            let pairs = self
                .rep
                .self_intersecting_pairs
                .as_ref()
                .map_or(0, Vec::len);
            if pairs > 0 {
                let ropts = RemoveSelfIntersectionsOpts::default();
                let ctx = self.ctx.sub(0.0, 1.0);
                let (mesh, r) = remove_self_intersections_with(&self.cur, &ropts, &ctx)?;
                if r.accepted_rounds > 0 {
                    self.cur = mesh;
                    changed = r.faces_removed + r.faces_added;
                }
            }
            self.record(RepairStepKind::SelfIntersections, changed, t0);
        }
        // Drop vertices nothing uses any more (merged, or freed by deleted faces).
        let mut used = vec![false; self.cur.positions.len()];
        for &i in self.cur.tris.iter().flatten() {
            used[i as usize] = true;
        }
        if used.iter().any(|u| !u) {
            self.cur.compact();
            self.dirty = true;
        }
        self.fresh(o.remove_self_intersections)
    }

    /// Fills the boundary loops; returns the number of faces added.
    fn fill(&mut self, h: &HoleFillOpts) -> MeshResult<usize> {
        let (mut he, _) = HalfEdgeMesh::from_indexed(&self.cur, NonManifoldPolicy::SplitVertices)?;
        let ctx = self.ctx.sub(0.0, 1.0);
        let r = fill_holes(&mut he, h, &ctx)?;
        if r.filled > 0 {
            self.cur = he.to_indexed();
        }
        Ok(if r.filled > 0 { r.new_faces } else { 0 })
    }
}

/// Repairs `m` and reports what was done (see the module docs for the steps and their order).
/// A mesh without defects comes back bit-identical, and repairing a repaired mesh changes
/// nothing. Cancellable between steps and inside the hole filler and the self-intersection
/// remover.
///
/// # Errors
/// [`MeshError::Cancelled`], [`MeshError::InvalidOption`] for a negative or non-finite weld
/// tolerance or small-shell fraction, or the errors of [`IndexedMesh::validate`].
pub fn repair(
    m: &IndexedMesh,
    opts: &RepairOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, RepairReport)> {
    if let Some(w) = &opts.weld
        && !(w.tolerance >= 0.0 && w.tolerance.is_finite())
    {
        return Err(MeshError::InvalidOption {
            name: "weld.tolerance",
            reason: format!("must be finite and not negative, got {}", w.tolerance),
        });
    }
    if let Some(f) = &opts.remove_small_shells
        && !(f.max_volume_fraction >= 0.0 && f.max_volume_fraction.is_finite())
    {
        return Err(MeshError::InvalidOption {
            name: "remove_small_shells.max_volume_fraction",
            reason: format!(
                "must be finite and not negative, got {}",
                f.max_volume_fraction
            ),
        });
    }
    m.validate()?;
    let aopts = AnalyzeOpts {
        self_intersections: opts.remove_self_intersections,
        ..AnalyzeOpts::default()
    };
    let before = analyze(m, &aopts, ctx)?;
    let mut run = Run {
        opts,
        ctx,
        cur: m.clone(),
        rep: before.clone(),
        rep_si: opts.remove_self_intersections,
        dirty: false,
        steps: Vec::new(),
        round: 0,
    };
    for round in 0..opts.max_rounds {
        ctx.checkpoint()?;
        run.round = round;
        let snapshot = (round > 0).then(|| {
            (
                run.cur.clone(),
                run.rep.clone(),
                run.rep_si,
                run.steps.len(),
            )
        });
        let steps_before = run.steps.len();
        run.round()?;
        let changed = run.steps[steps_before..].iter().any(|s| s.changed > 0);
        if let Some((cur, rep, rep_si, len)) = snapshot {
            // A later round is kept unless it makes the total worse; a round that changed
            // nothing is the fixed point and is not recorded.
            let worse =
                defects(&run.rep).iter().sum::<usize>() > defects(&rep).iter().sum::<usize>();
            if worse || !changed {
                run.cur = cur;
                run.rep = rep;
                run.rep_si = rep_si;
                run.steps.truncate(len);
                break;
            }
        } else if !changed {
            break;
        }
    }
    ctx.progress(1.0);
    let after = run.rep;
    let mut unresolved = Vec::new();
    for l in &after.boundary_loops {
        unresolved.push(UnresolvedIssue::HoleTooLarge {
            boundary_edges: l.len(),
        });
    }
    if let Some(p) = &after.self_intersecting_pairs
        && !p.is_empty()
    {
        unresolved.push(UnresolvedIssue::SelfIntersections { pairs: p.len() });
    }
    if !after.non_manifold_edges.is_empty() {
        unresolved.push(UnresolvedIssue::NonManifold {
            edges: after.non_manifold_edges.len(),
        });
    }
    Ok((
        run.cur,
        RepairReport {
            before,
            after,
            steps: run.steps,
            unresolved,
        },
    ))
}
