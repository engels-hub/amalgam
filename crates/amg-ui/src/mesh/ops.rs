//! The Phase 1 mesh operations: their commands, dialog fields and the feature each builds.
//!
//! Every op is a MESH-16/26/18/12 timeline feature; the dialog only edits its parameters
//! (inline [`Expr`]s, so the document turns them into model parameters) and [`OpParams::feature`]
//! builds the feature for the preview and for OK.

use std::borrow::Cow;

use amg_core::{Aabb3, Dimension};
use amg_doc::param::ParamTable;
use amg_doc::{BodyId, Expr, FeatureBox, UnitPrefs};
use amg_features::mesh::{
    CutPlane, EdgeLengthParam, FairingParam, KeepSideParam, MeshEntityRef, MeshEraseFillFeature,
    MeshFaceGroupsFeature, MeshPlaneCutFeature, MeshReduceFeature, MeshRemeshFeature,
    MeshRepairFeature, MeshReverseNormalFeature, MeshSeparateFeature, MeshShellFeature,
    MeshSmoothFeature, MeshWrapFeature, OriginPlane, OutputMode, ReduceParams, ReduceTarget,
    RemeshParams, RemeshSizing, RepairParams, SegmentParams, SmoothMethodParam, SmoothParams,
};
use amg_mesh::{FaceId, IndexedMesh, ShellSide};

use crate::command::CmdId;
use crate::module::GroupId;
use crate::widgets::{ChoiceField, ExprInput, IntField, check_expr, toggle};

const L: Dimension = Dimension::LENGTH;
const A: Dimension = Dimension::ANGLE;
const N: Dimension = Dimension::NONE;

/// A mesh operation of the MESH tab.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum MeshOp {
    /// MESH-04 repair pipeline (`mesh.repair`).
    Repair,
    /// MESH-06/22 remesh (`mesh.remesh`).
    Remesh,
    /// MESH-07 reduce (`mesh.reduce`).
    Reduce,
    /// MESH-08 smooth (`mesh.smooth`).
    Smooth,
    /// MESH-09 plane cut (`mesh.plane_cut`).
    PlaneCut,
    /// MESH-10 shell (`mesh.shell`).
    Shell,
    /// MESH-09 separate shells (`mesh.separate`).
    Separate,
    /// Reverse normals (`mesh.reverse_normal`).
    ReverseNormal,
    /// MESH-09 erase & fill (`mesh.erase_fill`).
    EraseFill,
    /// MESH-18 wrap (`mesh.wrap`).
    Wrap,
    /// MESH-12 face groups (`mesh.face_groups`).
    FaceGroups,
}

/// Toolbar group ids of the MESH tab.
pub const GROUP_CREATE: GroupId = GroupId("mesh.create");
/// "Prepare".
pub const GROUP_PREPARE: GroupId = GroupId("mesh.prepare");
/// "Modify".
pub const GROUP_MODIFY: GroupId = GroupId("mesh.modify");
/// "Inspect".
pub const GROUP_INSPECT: GroupId = GroupId("mesh.inspect");

impl MeshOp {
    /// Every op, in toolbar order.
    pub const ALL: [MeshOp; 11] = [
        MeshOp::Repair,
        MeshOp::Remesh,
        MeshOp::Reduce,
        MeshOp::Smooth,
        MeshOp::PlaneCut,
        MeshOp::Shell,
        MeshOp::Separate,
        MeshOp::ReverseNormal,
        MeshOp::EraseFill,
        MeshOp::Wrap,
        MeshOp::FaceGroups,
    ];

    /// The command that starts the op's tool.
    pub fn cmd(self) -> CmdId {
        CmdId(match self {
            MeshOp::Repair => "mesh.repair",
            MeshOp::Remesh => "mesh.remesh",
            MeshOp::Reduce => "mesh.reduce",
            MeshOp::Smooth => "mesh.smooth",
            MeshOp::PlaneCut => "mesh.plane_cut",
            MeshOp::Shell => "mesh.shell",
            MeshOp::Separate => "mesh.separate",
            MeshOp::ReverseNormal => "mesh.reverse_normal",
            MeshOp::EraseFill => "mesh.erase_fill",
            MeshOp::Wrap => "mesh.wrap",
            MeshOp::FaceGroups => "mesh.face_groups",
        })
    }

    /// The registered feature kind it creates (equal to the command id).
    pub fn feature_kind(self) -> &'static str {
        self.cmd().0
    }

    /// The op's name (dialog title, undo label).
    pub fn title(self) -> &'static str {
        match self {
            MeshOp::Repair => "Repair",
            MeshOp::Remesh => "Remesh",
            MeshOp::Reduce => "Reduce",
            MeshOp::Smooth => "Smooth",
            MeshOp::PlaneCut => "Plane Cut",
            MeshOp::Shell => "Shell",
            MeshOp::Separate => "Separate",
            MeshOp::ReverseNormal => "Reverse Normal",
            MeshOp::EraseFill => "Erase and Fill",
            MeshOp::Wrap => "Wrap",
            MeshOp::FaceGroups => "Generate Face Groups",
        }
    }

    /// Toolbar / search label: the title with `…`, the convention for commands that open a
    /// dialog.
    pub fn label(self) -> String {
        format!("{}…", self.title())
    }

    /// Tooltip.
    pub fn tooltip(self) -> &'static str {
        match self {
            MeshOp::Repair => {
                "Weld, remove degenerate and duplicate faces, fix non-manifold geometry, orient, fill holes and remove self-intersections"
            }
            MeshOp::Remesh => {
                "Rebuild the triangles at a uniform or curvature-adaptive edge length"
            }
            MeshOp::Reduce => "Reduce the triangle count (quadric decimation)",
            MeshOp::Smooth => "Smooth the surface (Taubin, Laplacian or feature-preserving)",
            MeshOp::PlaneCut => "Cut the mesh with a plane and cap the cut",
            MeshOp::Shell => "Hollow the mesh to a wall of the given thickness",
            MeshOp::Separate => "Split the mesh into one body per shell",
            MeshOp::ReverseNormal => "Flip the normals of the body or of selected face groups",
            MeshOp::EraseFill => "Delete face groups and fill the holes",
            MeshOp::Wrap => "Shrink-wrap the mesh into one watertight solid",
            MeshOp::FaceGroups => "Segment the mesh into face groups by normal and curvature",
        }
    }

    /// Extra S-key search terms.
    pub fn keywords(self) -> &'static [&'static str] {
        match self {
            MeshOp::Repair => &["fix", "heal", "watertight", "holes", "stl"],
            MeshOp::Remesh => &["retopology", "isotropic", "adaptive", "triangles"],
            MeshOp::Reduce => &["decimate", "simplify", "polygons", "qem"],
            MeshOp::Smooth => &["taubin", "laplacian", "denoise", "fair"],
            MeshOp::PlaneCut => &["split", "slice", "section", "bisect"],
            MeshOp::Shell => &["hollow", "offset", "thicken", "wall"],
            MeshOp::Separate => &["split", "shells", "components", "explode"],
            MeshOp::ReverseNormal => &["flip", "invert", "orientation"],
            MeshOp::EraseFill => &["delete", "remove", "patch", "fill"],
            MeshOp::Wrap => &["shrinkwrap", "envelope", "solidify", "voxel"],
            MeshOp::FaceGroups => &["segment", "regions", "groups"],
        }
    }

    /// Toolbar group.
    pub fn group(self) -> GroupId {
        match self {
            MeshOp::FaceGroups => GROUP_PREPARE,
            _ => GROUP_MODIFY,
        }
    }

    /// Also a top-level toolbar button.
    pub fn pinned(self) -> bool {
        matches!(
            self,
            MeshOp::Repair | MeshOp::Remesh | MeshOp::Reduce | MeshOp::PlaneCut | MeshOp::Shell
        )
    }

    /// The dialog has a face-selection field.
    pub fn uses_faces(self) -> bool {
        matches!(self, MeshOp::ReverseNormal | MeshOp::EraseFill)
    }

    /// The faces are required (not just a narrowing).
    pub fn needs_faces(self) -> bool {
        self == MeshOp::EraseFill
    }

    /// The feature has an [`OutputMode`] ("Create new body").
    pub fn has_output_mode(self) -> bool {
        matches!(
            self,
            MeshOp::Repair
                | MeshOp::Remesh
                | MeshOp::Reduce
                | MeshOp::Smooth
                | MeshOp::Shell
                | MeshOp::Wrap
                | MeshOp::FaceGroups
        )
    }
}

/// How the remesh edge length is given.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RemeshMode {
    /// An absolute edge length.
    Absolute,
    /// A factor of the input's mean edge length.
    Relative,
    /// Curvature-adaptive within a tolerance.
    Adaptive,
}

/// What a reduce aims for.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ReduceMode {
    /// Fraction of the input.
    Ratio,
    /// Face count.
    Count,
    /// Largest error.
    MaxError,
}

/// Smoothing algorithm.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SmoothKind {
    /// Taubin λ|μ.
    Taubin,
    /// Cotangent Laplacian.
    Laplacian,
    /// Bilateral normal filtering.
    Bilateral,
}

/// The dialog state of an op: expression texts and choices.
#[derive(Clone, Debug, PartialEq)]
#[allow(
    missing_docs,
    reason = "plain dialog fields, documented by their labels"
)]
pub enum OpParams {
    Repair {
        weld: bool,
        weld_tolerance: String,
        orient: bool,
        fill_holes: bool,
        remove_self_intersections: bool,
    },
    Remesh {
        mode: RemeshMode,
        edge: String,
        factor: String,
        tolerance: String,
        min_length: String,
        max_length: String,
        iterations: i64,
        keep_features: bool,
        feature_angle: String,
        preserve_boundary: bool,
        project: bool,
    },
    Reduce {
        mode: ReduceMode,
        ratio: String,
        count: String,
        max_error: String,
        preserve_boundary: bool,
    },
    Smooth {
        kind: SmoothKind,
        iterations: i64,
        lambda: String,
        mu: String,
        sigma: String,
        fix_boundary: bool,
    },
    PlaneCut {
        plane: OriginPlane,
        offset: String,
        keep: KeepSideParam,
        cap: bool,
    },
    Shell {
        thickness: String,
        side: ShellSide,
        robust: bool,
        auto_voxel: bool,
        voxel: String,
    },
    Separate,
    ReverseNormal,
    EraseFill {
        fairing: FairingParam,
    },
    Wrap {
        voxel: String,
        auto_offset: bool,
        offset: String,
    },
    FaceGroups {
        angle: String,
        min_group_faces: i64,
    },
}

/// A length literal in mm with three significant decimals at most (`1.25 mm`).
pub fn mm(v: f64) -> String {
    let v = if v.is_finite() { v } else { 0.0 };
    let s = format!("{v:.3}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    format!("{} mm", if s == "-0" { "0" } else { s })
}

fn diagonal(b: &Aabb3) -> f64 {
    if b.is_empty() {
        1.0
    } else {
        (b.max - b.min).length()
    }
}

/// Mean edge length of the triangles (each edge counted per face).
fn mean_edge(m: &IndexedMesh) -> f64 {
    let mut sum = 0.0;
    let mut n = 0usize;
    for t in &m.tris {
        for k in 0..3 {
            let (a, b) = (t[k] as usize, t[(k + 1) % 3] as usize);
            if let (Some(p), Some(q)) = (m.positions.get(a), m.positions.get(b)) {
                sum += (*p - *q).length();
                n += 1;
            }
        }
    }
    if n == 0 { 1.0 } else { sum / n as f64 }
}

/// Offset of the bounding-box centre along `plane`'s normal.
fn centre_offset(plane: OriginPlane, b: &Aabb3) -> f64 {
    if b.is_empty() {
        return 0.0;
    }
    let c = (b.min + b.max) * 0.5;
    match plane {
        OriginPlane::XY => c.z,
        OriginPlane::YZ => c.x,
        OriginPlane::XZ => -c.y,
    }
}

impl OpParams {
    /// The default dialog state of `op` (sized for a 100 mm part until a body is known).
    pub fn new(op: MeshOp) -> OpParams {
        let mut p = match op {
            MeshOp::Repair => OpParams::Repair {
                weld: true,
                weld_tolerance: "0.00001 mm".into(),
                orient: true,
                fill_holes: true,
                remove_self_intersections: true,
            },
            MeshOp::Remesh => OpParams::Remesh {
                mode: RemeshMode::Relative,
                edge: "1 mm".into(),
                factor: "1".into(),
                tolerance: "0.05 mm".into(),
                min_length: "0.2 mm".into(),
                max_length: "5 mm".into(),
                iterations: 5,
                keep_features: true,
                feature_angle: "45 deg".into(),
                preserve_boundary: true,
                project: true,
            },
            MeshOp::Reduce => OpParams::Reduce {
                mode: ReduceMode::Ratio,
                ratio: "0.5".into(),
                count: "10000".into(),
                max_error: "0.05 mm".into(),
                preserve_boundary: true,
            },
            MeshOp::Smooth => OpParams::Smooth {
                kind: SmoothKind::Taubin,
                iterations: 10,
                lambda: "0.5".into(),
                mu: "-0.53".into(),
                sigma: "0.35".into(),
                fix_boundary: true,
            },
            MeshOp::PlaneCut => OpParams::PlaneCut {
                plane: OriginPlane::XY,
                offset: "0 mm".into(),
                keep: KeepSideParam::Negative,
                cap: true,
            },
            MeshOp::Shell => OpParams::Shell {
                thickness: "2 mm".into(),
                side: ShellSide::Inside,
                robust: false,
                auto_voxel: true,
                voxel: "0.5 mm".into(),
            },
            MeshOp::Separate => OpParams::Separate,
            MeshOp::ReverseNormal => OpParams::ReverseNormal,
            MeshOp::EraseFill => OpParams::EraseFill {
                fairing: FairingParam::Biharmonic,
            },
            MeshOp::Wrap => OpParams::Wrap {
                voxel: "1 mm".into(),
                auto_offset: true,
                offset: "1.5 mm".into(),
            },
            MeshOp::FaceGroups => OpParams::FaceGroups {
                angle: "30 deg".into(),
                min_group_faces: i64::from(SegmentParams::default().min_group_faces),
            },
        };
        p.fit(&Aabb3::EMPTY, None);
        p
    }

    /// Sizes the length defaults to `mesh` (bounding box, mean edge length).
    pub fn fit_to(&mut self, mesh: &IndexedMesh) {
        let b = mesh.aabb();
        self.fit(&b, Some(mean_edge(mesh)));
    }

    fn fit(&mut self, b: &Aabb3, mean: Option<f64>) {
        let d = diagonal(b);
        match self {
            OpParams::Remesh {
                edge,
                tolerance,
                min_length,
                max_length,
                ..
            } => {
                let e = mean.unwrap_or(d / 100.0);
                *edge = mm(e);
                *tolerance = mm(d / 2000.0);
                *min_length = mm(e / 4.0);
                *max_length = mm(e * 4.0);
            }
            OpParams::Reduce { max_error, .. } => *max_error = mm(d / 2000.0),
            OpParams::PlaneCut { plane, offset, .. } => *offset = mm(centre_offset(*plane, b)),
            OpParams::Shell {
                thickness, voxel, ..
            } => {
                let t = (d * 0.02).max(0.01);
                *thickness = mm(t);
                *voxel = mm(t / 4.0);
            }
            OpParams::Wrap { voxel, offset, .. } => {
                *voxel = mm(d / 100.0);
                *offset = mm(1.5 * d / 100.0);
            }
            _ => {}
        }
    }

    /// The expression fields that are in use: `(label, source, dimension)`.
    pub fn exprs(&self) -> Vec<(&'static str, &str, Dimension)> {
        match self {
            OpParams::Repair {
                weld,
                weld_tolerance,
                ..
            } => {
                if *weld {
                    vec![("Weld tolerance", weld_tolerance.as_str(), L)]
                } else {
                    Vec::new()
                }
            }
            OpParams::Remesh {
                mode,
                edge,
                factor,
                tolerance,
                min_length,
                max_length,
                keep_features,
                feature_angle,
                ..
            } => {
                let mut v = match mode {
                    RemeshMode::Absolute => vec![("Edge length", edge.as_str(), L)],
                    RemeshMode::Relative => vec![("Edge length factor", factor.as_str(), N)],
                    RemeshMode::Adaptive => vec![
                        ("Tolerance", tolerance.as_str(), L),
                        ("Min edge", min_length.as_str(), L),
                        ("Max edge", max_length.as_str(), L),
                    ],
                };
                if *keep_features {
                    v.push(("Feature angle", feature_angle.as_str(), A));
                }
                v
            }
            OpParams::Reduce {
                mode,
                ratio,
                count,
                max_error,
                ..
            } => match mode {
                ReduceMode::Ratio => vec![("Ratio", ratio.as_str(), N)],
                ReduceMode::Count => vec![("Face count", count.as_str(), N)],
                ReduceMode::MaxError => vec![("Max error", max_error.as_str(), L)],
            },
            OpParams::Smooth {
                kind,
                lambda,
                mu,
                sigma,
                ..
            } => match kind {
                SmoothKind::Taubin => vec![("Lambda", lambda.as_str(), N), ("Mu", mu.as_str(), N)],
                SmoothKind::Laplacian => vec![("Lambda", lambda.as_str(), N)],
                SmoothKind::Bilateral => vec![("Sigma normal", sigma.as_str(), N)],
            },
            OpParams::PlaneCut { offset, .. } => vec![("Offset", offset.as_str(), L)],
            OpParams::Shell {
                thickness,
                robust,
                auto_voxel,
                voxel,
                ..
            } => {
                let mut v = vec![("Thickness", thickness.as_str(), L)];
                if *robust && !*auto_voxel {
                    v.push(("Voxel size", voxel.as_str(), L));
                }
                v
            }
            OpParams::Wrap {
                voxel,
                auto_offset,
                offset,
            } => {
                let mut v = vec![("Voxel size", voxel.as_str(), L)];
                if !*auto_offset {
                    v.push(("Offset", offset.as_str(), L));
                }
                v
            }
            OpParams::FaceGroups { angle, .. } => vec![("Angle threshold", angle.as_str(), A)],
            OpParams::Separate | OpParams::ReverseNormal | OpParams::EraseFill { .. } => Vec::new(),
        }
    }

    /// The first invalid field as `"<label>: <message>"`.
    pub fn first_error(&self, params: &ParamTable, prefs: &UnitPrefs) -> Option<String> {
        self.exprs().into_iter().find_map(|(label, src, dim)| {
            check_expr(src, dim, params, prefs)
                .message
                .map(|m| format!("{label}: {m}"))
        })
    }

    /// Draws the op's fields.
    pub fn ui(&mut self, ui: &mut egui::Ui, params: &ParamTable, prefs: &UnitPrefs) {
        let expr = |ui: &mut egui::Ui, label: &'static str, src: &mut String, dim: Dimension| {
            ExprInput::new(src, dim, params, prefs)
                .label(label)
                .id_salt(("mesh_op", label))
                .show(ui);
        };
        match self {
            OpParams::Repair {
                weld,
                weld_tolerance,
                orient,
                fill_holes,
                remove_self_intersections,
            } => {
                toggle(ui, "Weld vertices", weld);
                if *weld {
                    expr(ui, "Weld tolerance", weld_tolerance, L);
                }
                toggle(ui, "Orient faces", orient);
                toggle(ui, "Fill holes", fill_holes);
                toggle(ui, "Remove self-intersections", remove_self_intersections);
            }
            OpParams::Remesh {
                mode,
                edge,
                factor,
                tolerance,
                min_length,
                max_length,
                iterations,
                keep_features,
                feature_angle,
                preserve_boundary,
                project,
            } => {
                ChoiceField::new(
                    "Sizing",
                    mode,
                    &[
                        (RemeshMode::Relative, "Relative"),
                        (RemeshMode::Absolute, "Edge length"),
                        (RemeshMode::Adaptive, "Adaptive"),
                    ],
                )
                .show(ui);
                match mode {
                    RemeshMode::Absolute => expr(ui, "Edge length", edge, L),
                    RemeshMode::Relative => expr(ui, "Edge length factor", factor, N),
                    RemeshMode::Adaptive => {
                        expr(ui, "Tolerance", tolerance, L);
                        expr(ui, "Min edge", min_length, L);
                        expr(ui, "Max edge", max_length, L);
                    }
                }
                IntField::new("Iterations", iterations)
                    .range(1, 50)
                    .show(ui);
                toggle(ui, "Keep sharp edges", keep_features);
                if *keep_features {
                    expr(ui, "Feature angle", feature_angle, A);
                }
                toggle(ui, "Keep boundaries", preserve_boundary);
                toggle(ui, "Project to input", project);
            }
            OpParams::Reduce {
                mode,
                ratio,
                count,
                max_error,
                preserve_boundary,
            } => {
                ChoiceField::new(
                    "Target",
                    mode,
                    &[
                        (ReduceMode::Ratio, "Ratio"),
                        (ReduceMode::Count, "Face count"),
                        (ReduceMode::MaxError, "Max error"),
                    ],
                )
                .show(ui);
                match mode {
                    ReduceMode::Ratio => expr(ui, "Ratio", ratio, N),
                    ReduceMode::Count => expr(ui, "Face count", count, N),
                    ReduceMode::MaxError => expr(ui, "Max error", max_error, L),
                }
                toggle(ui, "Keep boundaries", preserve_boundary);
            }
            OpParams::Smooth {
                kind,
                iterations,
                lambda,
                mu,
                sigma,
                fix_boundary,
            } => {
                ChoiceField::new(
                    "Method",
                    kind,
                    &[
                        (SmoothKind::Taubin, "Taubin"),
                        (SmoothKind::Laplacian, "Laplacian"),
                        (SmoothKind::Bilateral, "Keep features"),
                    ],
                )
                .show(ui);
                match kind {
                    SmoothKind::Taubin => {
                        expr(ui, "Lambda", lambda, N);
                        expr(ui, "Mu", mu, N);
                    }
                    SmoothKind::Laplacian => expr(ui, "Lambda", lambda, N),
                    SmoothKind::Bilateral => expr(ui, "Sigma normal", sigma, N),
                }
                IntField::new("Iterations", iterations)
                    .range(1, 500)
                    .show(ui);
                toggle(ui, "Fix boundaries", fix_boundary);
            }
            OpParams::PlaneCut {
                plane,
                offset,
                keep,
                cap,
            } => {
                ChoiceField::new(
                    "Plane",
                    plane,
                    &[
                        (OriginPlane::XY, "XY"),
                        (OriginPlane::YZ, "YZ"),
                        (OriginPlane::XZ, "XZ"),
                    ],
                )
                .buttons()
                .show(ui);
                expr(ui, "Offset", offset, L);
                ChoiceField::new(
                    "Keep",
                    keep,
                    &[
                        (KeepSideParam::Negative, "Below"),
                        (KeepSideParam::Positive, "Above"),
                        (KeepSideParam::Both, "Both"),
                    ],
                )
                .show(ui);
                toggle(ui, "Cap the cut", cap);
            }
            OpParams::Shell {
                thickness,
                side,
                robust,
                auto_voxel,
                voxel,
            } => {
                expr(ui, "Thickness", thickness, L);
                ChoiceField::new(
                    "Direction",
                    side,
                    &[
                        (ShellSide::Inside, "Inside"),
                        (ShellSide::Outside, "Outside"),
                        (ShellSide::Both, "Both"),
                    ],
                )
                .show(ui);
                toggle(ui, "Robust (voxel)", robust);
                if *robust {
                    toggle(ui, "Automatic voxel size", auto_voxel);
                    if !*auto_voxel {
                        expr(ui, "Voxel size", voxel, L);
                    }
                }
            }
            OpParams::Separate => {
                ui.label("Each shell becomes its own body; the largest stays in the input body.");
            }
            OpParams::ReverseNormal => {
                ui.label("No faces selected: the whole body is flipped.");
            }
            OpParams::EraseFill { fairing } => {
                ChoiceField::new(
                    "Fill",
                    fairing,
                    &[
                        (FairingParam::Biharmonic, "Smooth"),
                        (FairingParam::Harmonic, "Natural"),
                        (FairingParam::None, "Flat"),
                    ],
                )
                .show(ui);
            }
            OpParams::Wrap {
                voxel,
                auto_offset,
                offset,
            } => {
                expr(ui, "Voxel size", voxel, L);
                toggle(ui, "Automatic offset", auto_offset);
                if !*auto_offset {
                    expr(ui, "Offset", offset, L);
                }
            }
            OpParams::FaceGroups {
                angle,
                min_group_faces,
            } => {
                expr(ui, "Angle threshold", angle, A);
                IntField::new("Min group faces", min_group_faces)
                    .range(0, 1_000_000)
                    .show(ui);
            }
        }
    }

    /// Re-centres the cut offset after the plane changed (other ops: nothing).
    pub fn plane_changed(&mut self, before: &OpParams, mesh: &IndexedMesh) {
        if let (OpParams::PlaneCut { plane, offset, .. }, OpParams::PlaneCut { plane: old, .. }) =
            (self, before)
            && plane != old
        {
            *offset = mm(centre_offset(*plane, &mesh.aabb()));
        }
    }

    /// The feature for `body`; `faces` are selected triangles of it (face-group ops).
    ///
    /// # Errors
    /// Why OK is not possible (a missing selection).
    pub fn feature(
        &self,
        op: MeshOp,
        body: BodyId,
        mesh: &IndexedMesh,
        faces: &[u32],
        new_body: bool,
    ) -> Result<FeatureBox, Cow<'static, str>> {
        let inline = |s: &str| Expr::Inline(s.trim().into());
        let output = if new_body && op.has_output_mode() {
            OutputMode::NewBody
        } else {
            OutputMode::Replace
        };
        Ok(match self {
            OpParams::Repair {
                weld,
                weld_tolerance,
                orient,
                fill_holes,
                remove_self_intersections,
            } => FeatureBox::new(MeshRepairFeature {
                body,
                opts: RepairParams {
                    weld: *weld,
                    weld_tolerance: inline(weld_tolerance),
                    orient: *orient,
                    fill_holes: *fill_holes,
                    remove_self_intersections: *remove_self_intersections,
                    ..RepairParams::default()
                },
                output,
            }),
            OpParams::Remesh {
                mode,
                edge,
                factor,
                tolerance,
                min_length,
                max_length,
                iterations,
                keep_features,
                feature_angle,
                preserve_boundary,
                project,
            } => FeatureBox::new(MeshRemeshFeature {
                body,
                params: RemeshParams {
                    sizing: match mode {
                        RemeshMode::Absolute => {
                            RemeshSizing::Uniform(EdgeLengthParam::Absolute(inline(edge)))
                        }
                        RemeshMode::Relative => {
                            RemeshSizing::Uniform(EdgeLengthParam::RelativeToMean(inline(factor)))
                        }
                        RemeshMode::Adaptive => RemeshSizing::Adaptive {
                            tolerance: inline(tolerance),
                            min_length: inline(min_length),
                            max_length: inline(max_length),
                        },
                    },
                    iterations: u32::try_from(*iterations).unwrap_or(1).max(1),
                    feature_angle: keep_features.then(|| inline(feature_angle)),
                    preserve_boundary: *preserve_boundary,
                    project_to_input: *project,
                    ..RemeshParams::default()
                },
                output,
            }),
            OpParams::Reduce {
                mode,
                ratio,
                count,
                max_error,
                preserve_boundary,
            } => FeatureBox::new(MeshReduceFeature {
                body,
                params: ReduceParams {
                    target: match mode {
                        ReduceMode::Ratio => ReduceTarget::Ratio(inline(ratio)),
                        ReduceMode::Count => ReduceTarget::FaceCount(inline(count)),
                        ReduceMode::MaxError => ReduceTarget::MaxError(inline(max_error)),
                    },
                    preserve_boundary: *preserve_boundary,
                    ..ReduceParams::default()
                },
                output,
            }),
            OpParams::Smooth {
                kind,
                iterations,
                lambda,
                mu,
                sigma,
                fix_boundary,
            } => FeatureBox::new(MeshSmoothFeature {
                body,
                params: SmoothParams {
                    method: match kind {
                        SmoothKind::Taubin => SmoothMethodParam::Taubin {
                            lambda: inline(lambda),
                            mu: inline(mu),
                        },
                        SmoothKind::Laplacian => SmoothMethodParam::Laplacian {
                            lambda: inline(lambda),
                            cotangent: true,
                        },
                        SmoothKind::Bilateral => SmoothMethodParam::BilateralNormal {
                            sigma_normal: inline(sigma),
                            normal_iterations: 5,
                            vertex_iterations: 10,
                        },
                    },
                    iterations: u32::try_from(*iterations).unwrap_or(1).max(1),
                    fix_boundary: *fix_boundary,
                    ..SmoothParams::default()
                },
                output,
            }),
            OpParams::PlaneCut {
                plane,
                offset,
                keep,
                cap,
            } => FeatureBox::new(MeshPlaneCutFeature {
                body,
                plane: CutPlane::Origin {
                    plane: *plane,
                    offset: inline(offset),
                },
                keep: *keep,
                cap: *cap,
                snap_tolerance: inline("0 mm"),
            }),
            OpParams::Shell {
                thickness,
                side,
                robust,
                auto_voxel,
                voxel,
            } => FeatureBox::new(MeshShellFeature {
                body,
                thickness: inline(thickness),
                side: *side,
                robust: *robust,
                voxel: (*robust && !*auto_voxel).then(|| inline(voxel)),
                output,
            }),
            OpParams::Separate => FeatureBox::new(MeshSeparateFeature { body }),
            OpParams::ReverseNormal => FeatureBox::new(MeshReverseNormalFeature {
                body,
                groups: if faces.is_empty() {
                    None
                } else {
                    Some(group_refs(mesh, faces))
                },
            }),
            OpParams::EraseFill { fairing } => {
                let groups = group_refs(mesh, faces);
                if groups.is_empty() {
                    return Err("Select the faces to erase".into());
                }
                FeatureBox::new(MeshEraseFillFeature {
                    body,
                    groups,
                    fairing: *fairing,
                })
            }
            OpParams::Wrap {
                voxel,
                auto_offset,
                offset,
            } => FeatureBox::new(MeshWrapFeature {
                body,
                voxel: inline(voxel),
                offset: (!*auto_offset).then(|| inline(offset)),
                output,
            }),
            OpParams::FaceGroups {
                angle,
                min_group_faces,
            } => FeatureBox::new(MeshFaceGroupsFeature {
                body,
                opts: SegmentParams {
                    angle_threshold: inline(angle),
                    min_group_faces: u32::try_from(*min_group_faces).unwrap_or(0),
                    ..SegmentParams::default()
                },
                output,
            }),
        })
    }
}

/// The face groups the selected triangles belong to, as stable references (sorted, unique).
pub fn group_refs(mesh: &IndexedMesh, faces: &[u32]) -> Vec<MeshEntityRef> {
    let mut groups: Vec<_> = faces
        .iter()
        .filter(|&&f| (f as usize) < mesh.face_count())
        .map(|&f| mesh.groups.group_of(FaceId(f)))
        .collect();
    groups.sort_unstable();
    groups.dedup();
    groups
        .into_iter()
        .filter_map(|g| MeshEntityRef::new(mesh, g))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mm_literals_are_short() {
        assert_eq!(mm(1.25), "1.25 mm");
        assert_eq!(mm(2.0), "2 mm");
        assert_eq!(mm(0.0001), "0 mm");
        assert_eq!(mm(-0.0001), "0 mm");
        assert_eq!(mm(-3.5), "-3.5 mm");
    }

    #[test]
    fn every_op_has_a_distinct_command_matching_its_feature_kind() {
        let mut ids: Vec<_> = MeshOp::ALL.iter().map(|o| o.cmd()).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), MeshOp::ALL.len());
        for op in MeshOp::ALL {
            assert!(op.label().ends_with('…'));
            assert!(op.feature_kind().starts_with("mesh."));
        }
    }

    #[test]
    fn default_fields_are_valid_for_every_op() {
        let design = amg_doc::model::Design::new("t");
        let prefs = UnitPrefs::default();
        for op in MeshOp::ALL {
            let p = OpParams::new(op);
            assert_eq!(p.first_error(design.params(), &prefs), None, "{op:?}");
        }
    }
}
