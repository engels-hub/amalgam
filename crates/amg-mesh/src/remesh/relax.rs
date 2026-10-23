//! Tangential relaxation and projection (one parallel, pure pass).

use amg_core::Point3;
use amg_jobs::JobCtx;
use rayon::prelude::*;

use super::State;
use super::features::{HARD, Snapper};
use crate::{MeshResult, MeshSpatial, VertexId};

/// Step toward the area-weighted centroid / curve midpoint.
const LAMBDA: f64 = 0.7;
/// A relaxed vertex may not rotate an incident triangle by more than `acos` of this.
const MIN_NORMAL_COS: f64 = 0.1;

impl State {
    /// New position of `v`, or `None` to leave it.
    fn relaxed(
        &self,
        v: u32,
        spatial: Option<&MeshSpatial>,
        snapper: &Snapper,
    ) -> Option<(Point3, f64)> {
        if self.fixed[v as usize] {
            return None;
        }
        let vid = VertexId(v);
        let p = self.pos(v);
        let nc = self.ncons[v as usize];
        let target = if nc == 0 {
            let (mut csum, mut asum, mut nsum) = (Point3::ZERO, 0.0, Point3::ZERO);
            for o in self.he.outgoing(vid) {
                let q1 = self.pos(self.he.target(o).0);
                let q2 = self.pos(self.he.target(self.he.next(o)).0);
                let n = (q1 - p).cross(q2 - p);
                let area = n.length();
                csum += (p + q1 + q2) / 3.0 * area;
                asum += area;
                nsum += n;
            }
            if asum <= 0.0 {
                return None;
            }
            let n = nsum.normalize_or_zero();
            let d = csum / asum - p;
            let mut q = p + (d - n * d.dot(n)) * LAMBDA;
            if let (true, Some(sp)) = (self.project, spatial) {
                let cp = sp.closest_point(q, 2.0 * self.vs[v as usize])?;
                q = cp.point;
            }
            q
        } else {
            let [a, b] = self.curve(v)?;
            let mid = (self.pos(a) + self.pos(b)) * 0.5;
            let mut q = p + (mid - p) * LAMBDA;
            let hard = self
                .cons
                .get(&super::key(v, a))
                .is_some_and(|&k| k & HARD != 0);
            if hard {
                q = snapper.snap(q, self.vs[v as usize])?;
            }
            q
        };
        // Reject moves that fold an incident triangle.
        for o in self.he.outgoing(vid) {
            let q1 = self.pos(self.he.target(o).0);
            let q2 = self.pos(self.he.target(self.he.next(o)).0);
            let (old, new) = ((q1 - p).cross(q2 - p), (q1 - target).cross(q2 - target));
            let (lo, ln) = (old.length(), new.length());
            if lo > 0.0 && (ln <= 0.0 || old.dot(new) < MIN_NORMAL_COS * lo * ln) {
                return None;
            }
        }
        let size = match (&self.field, spatial) {
            (Some(f), Some(sp)) => f
                .at(sp, target, 2.0 * self.vs[v as usize])
                .unwrap_or(self.vs[v as usize]),
            _ => self.vs[v as usize],
        };
        Some((target, size))
    }

    pub(super) fn relax_pass(
        &mut self,
        spatial: Option<&MeshSpatial>,
        snapper: &Snapper,
        ctx: &JobCtx,
    ) -> MeshResult<()> {
        let verts: Vec<u32> = self.he.vertices().map(|v| v.0).collect();
        let moves: Vec<Option<(Point3, f64)>> = verts
            .par_iter()
            .map(|&v| self.relaxed(v, spatial, snapper))
            .collect();
        ctx.checkpoint()?;
        for (&v, m) in verts.iter().zip(moves) {
            if let Some((q, size)) = m {
                self.he.set_position(VertexId(v), q);
                self.vs[v as usize] = size;
            }
        }
        Ok(())
    }
}
