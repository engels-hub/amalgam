//! Op parameter types of the `BrepKernel` trait (S02 Interfaces, "op options").
//!
//! All are plain data and `Clone + Debug + PartialEq + Serialize + DeserializeOwned`, so S20's
//! `RecordingKernel` can capture every call as a `KernelOp` (S20 request R3, BREP-12 Done-when
//! 3). The op WPs (BREP-03..BREP-17) add the trait methods that take them.

use amg_core::{Dir3, Plane, Point3, Vec3};

use crate::shape::FaceId;

/// The three booleans.
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BoolOp {
    /// Union.
    Fuse,
    /// Objects minus tools.
    Cut,
    /// Intersection.
    Common,
}

/// BOPAlgo gluing mode, for arguments that share or overlap faces.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum Glue {
    /// No gluing.
    #[default]
    Off,
    /// Shifted coincident faces.
    Shift,
    /// Fully coincident faces.
    Full,
}

/// Boolean options. Non-destructive mode is always on (S02 §2, category N).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BoolOpts {
    /// Fuzzy value in mm; 0 = exact.
    pub fuzzy: f64,
    /// Gluing mode.
    pub glue: Glue,
    /// Check arguments for inverted solids.
    pub check_inverted: bool,
    /// Use oriented bounding boxes.
    pub use_obb: bool,
    /// OCCT-internal parallelism; `None` = `KernelConfig::bool_parallel`.
    pub parallel: Option<bool>,
}

impl Default for BoolOpts {
    fn default() -> Self {
        BoolOpts {
            fuzzy: 0.0,
            glue: Glue::Off,
            check_inverted: true,
            use_obb: false,
            parallel: None,
        }
    }
}

/// How far a prism extends.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PrismExtent {
    /// By this vector, mm.
    Vector(Vec3),
    /// Infinitely along `dir`, or both ways.
    Infinite {
        /// Direction.
        dir: Dir3,
        /// Both directions.
        both: bool,
    },
}

/// Prism options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PrismOpts {
    /// Taper angle, rad; 0 = none.
    pub taper: f64,
    /// Merge coplanar faces of the result (`UnifySameDomain`).
    pub merge_coplanar: bool,
}

/// Pipe-shell trihedron mode.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PipeMode {
    /// Corrected Frenet.
    CorrectedFrenet,
    /// Frenet.
    Frenet,
    /// Fixed trihedron with this binormal.
    Fixed {
        /// The binormal.
        binormal: Dir3,
    },
    /// Constant binormal direction.
    Binormal(Dir3),
    /// Guided by an auxiliary wire.
    Guide {
        /// Operand index of the guide wire.
        wire: usize,
        /// Keep contact with the guide.
        contact: bool,
    },
}

/// How pipe-shell treats spine corners.
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Transition {
    /// Transformed sections.
    Transformed,
    /// Sharp corners.
    RightCorner,
    /// Rounded corners.
    RoundCorner,
}

/// Pipe-shell options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PipeOpts {
    /// Trihedron mode.
    pub mode: PipeMode,
    /// Twist law: `(param 0..1, rad)` samples.
    pub twist: Option<Vec<(f64, f64)>>,
    /// Scale law: `(param 0..1, factor)` samples.
    pub scale: Option<Vec<(f64, f64)>>,
    /// Make a solid rather than a shell.
    pub solid: bool,
    /// Corner treatment.
    pub transition: Transition,
}

/// Raw `BRepOffsetAPI_ThruSections` options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThruSectionsOpts {
    /// Make a solid rather than a shell.
    pub solid: bool,
    /// Ruled rather than smooth.
    pub ruled: bool,
    /// Run `CheckCompatibility`.
    pub check_compatibility: bool,
    /// Smoothing.
    pub smoothing: bool,
    /// Maximum surface degree.
    pub max_degree: u8,
}

/// One loft section.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum LoftSection {
    /// A wire, by operand index.
    Wire(usize),
    /// A point section.
    Point(Point3),
}

/// Fillet radius law of one edge.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum FilletSpec {
    /// Constant radius, mm.
    Constant(f64),
    /// `(normalized param 0..1, radius mm)` samples.
    Variable(Vec<(f64, f64)>),
    /// Chord length, mm, converted to a radius per edge.
    Chord(f64),
}

/// `ChFi3d_FilletShape`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum FilletShape {
    /// Rational.
    #[default]
    Rational,
    /// Quasi-angular.
    QuasiAngular,
    /// Polynomial.
    Polynomial,
}

/// Fillet blend options (S20 request R7). `None` = OCCT default.
#[derive(Clone, Debug, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct FilletOpts {
    /// Blend surface shape.
    pub shape: FilletShape,
    /// 3D approximation tolerance, mm.
    pub tol_app_3d: Option<f64>,
    /// Deflection, mm.
    pub fleche: Option<f64>,
}

/// Chamfer of one edge.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ChamferSpec {
    /// The same distance on both faces, mm.
    Equal(f64),
    /// `d1` on `face`, `d2` on the other face.
    TwoDistances {
        /// Distance on `face`, mm.
        d1: f64,
        /// Distance on the other face, mm.
        d2: f64,
        /// The reference face.
        face: FaceId,
    },
    /// Distance on `face` and angle.
    DistanceAngle {
        /// Distance on `face`, mm.
        d: f64,
        /// Angle, rad.
        angle: f64,
        /// The reference face.
        face: FaceId,
    },
}

/// Offset join type.
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum OffsetJoin {
    /// Arcs (rounded).
    Arc,
    /// Intersections (sharp).
    Intersection,
}

/// Shell / thick-solid options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ShellOpts {
    /// Wall thickness, mm; negative is inward.
    pub thickness: f64,
    /// Join type.
    pub join: OffsetJoin,
    /// Per-face thickness overrides.
    pub per_face: Vec<(FaceId, f64)>,
    /// Tolerance, mm.
    pub tol: f64,
}

/// Draft options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DraftOpts {
    /// Pull direction.
    pub pull: Dir3,
    /// Draft angle, rad.
    pub angle: f64,
    /// Neutral plane.
    pub neutral: Plane,
}

/// Healing options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct HealOpts {
    /// Working tolerance, mm.
    pub tol: f64,
    /// Remove small edges.
    pub fix_small_edges: bool,
    /// Fix face/shell orientation.
    pub fix_orientation: bool,
    /// Largest tolerance healing may set, mm.
    pub max_tol: f64,
}

/// `UnifySameDomain` options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct UnifyOpts {
    /// Merge faces.
    pub faces: bool,
    /// Merge edges.
    pub edges: bool,
    /// Concatenate B-spline curves.
    pub concat_bsplines: bool,
    /// Linear tolerance, mm.
    pub lin_tol: f64,
    /// Angular tolerance, rad.
    pub ang_tol: f64,
}

/// Sewing options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SewOpts {
    /// Sewing tolerance, mm.
    pub tol: f64,
    /// Allow non-manifold results.
    pub non_manifold: bool,
    /// Make a solid from a closed result.
    pub make_solid: bool,
}

/// A helix.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct HelixSpec {
    /// Axis frame; the helix runs along its normal.
    pub axis: Plane,
    /// Radius at the start, mm.
    pub radius: f64,
    /// Pitch, mm per turn.
    pub pitch: f64,
    /// Height, mm.
    pub height: f64,
    /// Taper angle, rad.
    pub taper: f64,
    /// Left-handed.
    pub left_handed: bool,
}

/// Tessellation quality (S02 §8). Presets: [`TessQuality::preview`], [`TessQuality::display`],
/// [`TessQuality::export`].
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TessQuality {
    /// Linear deflection: mm, or a fraction of the bbox diagonal if `relative`.
    pub linear: f64,
    /// Angular deflection, rad.
    pub angular: f64,
    /// Whether `linear` is relative to the bbox diagonal.
    pub relative: bool,
    /// Minimum element size, mm; 0 = OCCT default.
    pub min_size: f64,
    /// Upper clamp of the linear deflection in relative mode, mm.
    pub max_linear_abs: Option<f64>,
}

impl TessQuality {
    /// 0.5 % of the diagonal, 0.5 rad.
    pub fn preview() -> Self {
        TessQuality {
            linear: 0.005,
            angular: 0.5,
            relative: true,
            min_size: 0.0,
            max_linear_abs: None,
        }
    }

    /// 0.1 % of the diagonal, 0.35 rad.
    pub fn display() -> Self {
        TessQuality {
            linear: 0.001,
            angular: 0.35,
            relative: true,
            min_size: 0.0,
            max_linear_abs: None,
        }
    }

    /// 0.01 % of the diagonal, at most 0.01 mm, 0.2 rad.
    pub fn export() -> Self {
        TessQuality {
            linear: 0.0001,
            angular: 0.2,
            relative: true,
            min_size: 0.0,
            max_linear_abs: Some(0.01),
        }
    }
}
