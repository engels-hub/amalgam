//! Warnings, reports and errors (S12 §Interfaces, "reports, warnings, errors").

use std::path::PathBuf;
use std::time::Duration;

use amg_core::{Diagnostic, Severity, Unit, units::LengthUnit};
use amg_mesh::MeshError;

use crate::format::Format;

/// Where in a file a problem was found.
#[derive(Clone, Debug, PartialEq)]
pub enum Location {
    /// Byte offset from the start of the file.
    Byte(u64),
    /// 1-based line number.
    Line(u64),
    /// Position inside an XML part of a package.
    Xml {
        /// Part name inside the package.
        part: String,
        /// 1-based line.
        line: u64,
        /// 1-based column.
        column: u64,
    },
    /// A whole entry of a zip container.
    ZipEntry(String),
}

/// Something that did not stop the import or export but that the user should know about.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum IoWarning {
    /// The extension disagrees with the content; the content wins.
    ExtensionMismatch {
        /// Extension as given (lower case, no dot).
        extension: String,
        /// Format found by sniffing.
        detected: Format,
    },
    /// A binary header declares another triangle count than the file holds.
    HeaderCountMismatch {
        /// Count in the header.
        header: u64,
        /// Count that fits the file.
        actual: u64,
    },
    /// The file ends before the declared content.
    Truncated {
        /// Declared count.
        expected: u64,
        /// Count actually read.
        read: u64,
    },
    /// Bytes after the declared content were ignored.
    TrailingBytes(u64),
    /// Triangles with NaN or infinite coordinates were dropped.
    NonFiniteDropped(u64),
    /// Faces with out-of-range vertex indices were dropped.
    InvalidIndexDropped(u64),
    /// Polygons were triangulated.
    PolygonTriangulated {
        /// Polygons with more than three vertices.
        polygons: u64,
        /// Polygons that needed the fan fallback.
        fallback_fan: u64,
    },
    /// The file declares no unit and this one was assumed.
    UnitAssumed(LengthUnit),
    /// An optional extension from this namespace was ignored.
    UnknownExtensionIgnored {
        /// XML namespace.
        namespace: String,
    },
    /// Content that is not supported yet was ignored (textures, composites, beams, ...).
    UnsupportedContentIgnored {
        /// What was ignored.
        what: String,
        /// How many items.
        count: u64,
    },
    /// Painted triangles were approximated by their dominant state.
    PaintApproximated {
        /// Number of approximated triangles.
        triangles: u64,
    },
    /// An object is not a closed manifold.
    NotManifold {
        /// Object name.
        object: String,
        /// Edges with exactly one face.
        boundary_edges: u64,
        /// Edges with more than two faces.
        non_manifold_edges: u64,
    },
    /// Writing lost precision.
    PrecisionLoss {
        /// Largest coordinate error, mm.
        max_error_mm: f64,
    },
    /// A referenced companion file or resource is missing (mtllib, texture path).
    MissingReference {
        /// What is missing.
        what: String,
    },
}

impl IoWarning {
    /// One-line, user-facing text.
    pub fn message(&self) -> String {
        match self {
            IoWarning::ExtensionMismatch {
                extension,
                detected,
            } => format!(
                "The file extension .{extension} does not match the content, which is {}. \
                 The content was used.",
                detected.info().name
            ),
            IoWarning::HeaderCountMismatch { header, actual } => format!(
                "The header declares {header} triangles but the file holds {actual}. \
                 The file size was used."
            ),
            IoWarning::Truncated { expected, read } => {
                format!("The file is truncated: {read} of {expected} items could be read.")
            }
            IoWarning::TrailingBytes(n) => format!("{n} unused byte(s) at the end were ignored."),
            IoWarning::NonFiniteDropped(n) => {
                format!("{n} triangle(s) with non-finite coordinates were dropped.")
            }
            IoWarning::InvalidIndexDropped(n) => {
                format!("{n} face(s) with invalid vertex indices were dropped.")
            }
            IoWarning::PolygonTriangulated {
                polygons,
                fallback_fan,
            } => format!(
                "{polygons} polygon(s) were triangulated ({fallback_fan} with the fan fallback)."
            ),
            IoWarning::UnitAssumed(u) => format!(
                "The file declares no unit; {} was assumed.",
                Unit::from(*u).symbol()
            ),
            IoWarning::UnknownExtensionIgnored { namespace } => {
                format!("The optional extension {namespace} is not supported and was ignored.")
            }
            IoWarning::UnsupportedContentIgnored { what, count } => {
                format!("{count} unsupported item(s) were ignored: {what}.")
            }
            IoWarning::PaintApproximated { triangles } => format!(
                "{triangles} painted triangle(s) were approximated by their dominant colour."
            ),
            IoWarning::NotManifold {
                object,
                boundary_edges,
                non_manifold_edges,
            } => format!(
                "Object \"{object}\" is not a closed manifold ({boundary_edges} open edge(s), \
                 {non_manifold_edges} non-manifold edge(s))."
            ),
            IoWarning::PrecisionLoss { max_error_mm } => {
                format!("Coordinates lost precision, by up to {max_error_mm:e} mm.")
            }
            IoWarning::MissingReference { what } => {
                format!("A referenced file is missing: {what}.")
            }
        }
    }
}

/// What an import found, besides the geometry.
#[derive(Clone, Debug, Default)]
pub struct ImportReport {
    /// Detected format.
    pub format: Option<Format>,
    /// File size in bytes.
    pub bytes: u64,
    /// Unit the file declares, if any.
    pub declared_unit: Option<LengthUnit>,
    /// Unit that was applied.
    pub applied_unit: Option<LengthUnit>,
    /// Triangles read from the file.
    pub triangles_read: u64,
    /// Vertices after welding.
    pub vertices: u64,
    /// Degenerate triangles kept.
    pub degenerate_triangles: u64,
    /// Everything that did not stop the import.
    pub warnings: Vec<IoWarning>,
    /// Wall time.
    pub duration: Duration,
}

/// Validity of one object for an export format.
#[derive(Clone, Debug, PartialEq)]
pub struct ObjectCheck {
    /// Index into `Scene::objects`.
    pub object: u32,
    /// Object name.
    pub name: Option<String>,
    /// Edges with exactly one face.
    pub boundary_edges: u64,
    /// Edges with more than two faces.
    pub non_manifold_edges: u64,
    /// Neighbouring faces disagree on orientation.
    pub inconsistent_orientation: bool,
    /// Closed, but the faces point inward.
    pub negative_volume: bool,
}

impl ObjectCheck {
    /// A closed, manifold, consistently oriented solid with positive volume.
    pub fn is_valid(&self) -> bool {
        self.boundary_edges == 0
            && self.non_manifold_edges == 0
            && !self.inconsistent_orientation
            && !self.negative_volume
    }
}

/// Result of [`check_export`](crate::check_export).
#[derive(Clone, Debug, Default)]
pub struct ExportCheck {
    /// One entry per distinct object that is placed in the scene.
    pub objects: Vec<ObjectCheck>,
    /// Warnings that go with the check.
    pub warnings: Vec<IoWarning>,
}

impl ExportCheck {
    /// Every object is a valid solid.
    pub fn is_valid(&self) -> bool {
        self.objects.iter().all(ObjectCheck::is_valid)
    }
}

/// What an export wrote.
#[derive(Clone, Debug, Default)]
pub struct ExportReport {
    /// Bytes written.
    pub bytes: u64,
    /// Triangles written.
    pub triangles: u64,
    /// Files created (several with a one-file-per-object export).
    pub files: Vec<PathBuf>,
    /// Warnings.
    pub warnings: Vec<IoWarning>,
    /// Wall time.
    pub duration: Duration,
}

/// Errors of `amg-io`.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum IoError {
    /// A file could not be read.
    #[error("cannot read {path}: {source}")]
    Read {
        /// File.
        path: PathBuf,
        /// Cause.
        #[source]
        source: std::io::Error,
    },
    /// A file could not be written.
    #[error("cannot write {path}: {source}")]
    Write {
        /// File.
        path: PathBuf,
        /// Cause.
        #[source]
        source: std::io::Error,
    },
    /// The content matches no supported format.
    #[error("unknown or unsupported file format")]
    UnknownFormat,
    /// The format exists but is not available in this build or direction.
    #[error("{format:?} {direction} is not available: {reason}")]
    Unsupported {
        /// Format.
        format: Format,
        /// "import" or "export".
        direction: &'static str,
        /// Why.
        reason: &'static str,
    },
    /// The file is damaged or violates its specification.
    #[error("{format:?} file is malformed at {location:?}: {message}")]
    Malformed {
        /// Format.
        format: Format,
        /// Where.
        location: Location,
        /// What is wrong.
        message: String,
    },
    /// A [`ReadLimits`](crate::ReadLimits) limit was exceeded.
    #[error("file exceeds the {what} limit ({actual} > {limit})")]
    LimitExceeded {
        /// Which limit.
        what: &'static str,
        /// Its value.
        limit: u64,
        /// The value found.
        actual: u64,
    },
    /// A required 3MF extension is not supported.
    #[error("3MF file requires the unsupported extension {namespace}")]
    UnsupportedRequiredExtension {
        /// XML namespace.
        namespace: String,
    },
    /// 3MF Secure Content.
    #[error("3MF file is encrypted (Secure Content extension)")]
    Encrypted,
    /// No triangles survived.
    #[error("file contains no usable triangles")]
    Empty,
    /// A PLY with vertices but no faces.
    #[error("PLY file has vertices but no faces (point clouds are not supported)")]
    PointCloud,
    /// Some objects are not valid solids for a strict format.
    #[error("{count} object(s) are not valid closed solids for {format:?}")]
    InvalidForFormat {
        /// Target format.
        format: Format,
        /// Number of invalid objects.
        count: usize,
        /// Details.
        check: Box<ExportCheck>,
    },
    /// A scene violates its invariants (additive to the S12 interface, see the spec's
    /// Interface changes).
    #[error("scene is inconsistent: {message}")]
    InvalidScene {
        /// What is wrong.
        message: String,
    },
    /// A mesh error.
    #[error(transparent)]
    Mesh(#[from] MeshError),
    /// A B-rep exchange error (S02 owns the kernel error).
    #[error("B-rep exchange failed: {0}")]
    Brep(String),
    /// The user cancelled.
    #[error("operation cancelled")]
    Cancelled,
}

impl From<amg_core::Cancelled> for IoError {
    fn from(_: amg_core::Cancelled) -> Self {
        IoError::Cancelled
    }
}

impl Diagnostic for IoError {
    fn user_message(&self) -> String {
        match self {
            IoError::Read { path, .. } => format!("The file {} could not be read.", path.display()),
            IoError::Write { path, .. } => {
                format!("The file {} could not be written.", path.display())
            }
            IoError::UnknownFormat => "This file type is not recognised.".into(),
            IoError::Unsupported {
                format,
                direction,
                reason,
            } => format!(
                "{} {direction} is not available: {reason}.",
                format.info().name
            ),
            IoError::Malformed {
                format, message, ..
            } => format!("The {} file is damaged: {message}.", format.info().name),
            IoError::LimitExceeded { what, .. } => {
                format!("The file is too large or too complex ({what}).")
            }
            IoError::UnsupportedRequiredExtension { namespace } => {
                format!("The 3MF file needs the extension {namespace}, which is not supported.")
            }
            IoError::Encrypted => "The 3MF file is encrypted.".into(),
            IoError::Empty => "The file contains no usable triangles.".into(),
            IoError::PointCloud => "The PLY file is a point cloud without faces.".into(),
            IoError::InvalidForFormat { format, count, .. } => format!(
                "{count} object(s) are not closed solids, which {} requires.",
                format.info().name
            ),
            IoError::InvalidScene { message } => format!("The scene is inconsistent: {message}."),
            IoError::Mesh(e) => format!("Mesh problem: {e}."),
            IoError::Brep(m) => format!("The B-rep exchange failed: {m}."),
            IoError::Cancelled => "Cancelled.".into(),
        }
    }

    fn hint(&self) -> Option<String> {
        let h = match self {
            IoError::Read { .. } => "Check that the file exists and that you may read it.",
            IoError::Write { .. } => {
                "Check the free disk space and that the folder is writable. \
                 An existing file was left untouched."
            }
            IoError::UnknownFormat => {
                "Amalgam projects (.amg) are opened with File > Open, not imported. \
                 Otherwise check the file type, or pick the format explicitly."
            }
            IoError::Unsupported { .. } => "Use another format, or a build that includes this one.",
            IoError::Malformed { .. } => {
                "Export the file again from the program that made it, or try another format."
            }
            IoError::LimitExceeded { .. } => {
                "Split or decimate the model in its source program, or raise the limit."
            }
            IoError::UnsupportedRequiredExtension { .. } => {
                "Re-export the file from the slicer or CAD program without that extension."
            }
            IoError::Encrypted => "Re-export from the slicer without encryption.",
            IoError::Empty => "Check that the file is the right one and that it holds a model.",
            IoError::PointCloud => "Reconstruct a surface from the points first, then import it.",
            IoError::InvalidForFormat { .. } => {
                "Repair the mesh (Mesh > Repair) or choose Export anyway."
            }
            IoError::InvalidScene { .. } => {
                "This is an internal inconsistency; re-import the file or report the problem."
            }
            IoError::Mesh(_) | IoError::Brep(_) | IoError::Cancelled => return None,
        };
        Some(h.to_owned())
    }

    fn severity(&self) -> Severity {
        match self {
            IoError::Cancelled => Severity::Info,
            _ => Severity::Error,
        }
    }

    fn is_cancelled(&self) -> bool {
        matches!(self, IoError::Cancelled)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One value of every variant. The `match` makes adding a variant without updating this
    /// list a compile error.
    fn all_variants() -> Vec<IoError> {
        fn _exhaustive(e: &IoError) {
            match e {
                IoError::Read { .. }
                | IoError::Write { .. }
                | IoError::UnknownFormat
                | IoError::Unsupported { .. }
                | IoError::Malformed { .. }
                | IoError::LimitExceeded { .. }
                | IoError::UnsupportedRequiredExtension { .. }
                | IoError::Encrypted
                | IoError::Empty
                | IoError::PointCloud
                | IoError::InvalidForFormat { .. }
                | IoError::InvalidScene { .. }
                | IoError::Mesh(_)
                | IoError::Brep(_)
                | IoError::Cancelled => {}
            }
        }
        let io = || std::io::Error::other("boom");
        vec![
            IoError::Read {
                path: "a.stl".into(),
                source: io(),
            },
            IoError::Write {
                path: "a.stl".into(),
                source: io(),
            },
            IoError::UnknownFormat,
            IoError::Unsupported {
                format: Format::Step,
                direction: "import",
                reason: "built without the brep feature",
            },
            IoError::Malformed {
                format: Format::Stl,
                location: Location::Byte(0),
                message: "bad".into(),
            },
            IoError::LimitExceeded {
                what: "triangle count",
                limit: 1,
                actual: 2,
            },
            IoError::UnsupportedRequiredExtension {
                namespace: "urn:x".into(),
            },
            IoError::Encrypted,
            IoError::Empty,
            IoError::PointCloud,
            IoError::InvalidForFormat {
                format: Format::ThreeMf,
                count: 1,
                check: Box::default(),
            },
            IoError::InvalidScene {
                message: "bad".into(),
            },
            IoError::Mesh(MeshError::Empty),
            IoError::Brep("kernel".into()),
            IoError::Cancelled,
        ]
    }

    #[test]
    fn every_variant_has_a_message_and_remedies_have_hints() {
        let variants = all_variants();
        assert_eq!(variants.len(), 15);
        for e in &variants {
            assert!(!e.user_message().trim().is_empty(), "{e:?}");
            assert!(!e.to_string().trim().is_empty(), "{e:?}");
            // No remedy for: the user cancelled, or the cause is another crate's error that
            // carries its own text.
            let no_remedy = matches!(e, IoError::Mesh(_) | IoError::Brep(_) | IoError::Cancelled);
            match (e.hint(), no_remedy) {
                (Some(h), false) => assert!(!h.trim().is_empty(), "{e:?}"),
                (None, true) => {}
                (h, _) => panic!("unexpected hint {h:?} for {e:?}"),
            }
        }
    }

    #[test]
    fn cancelled_is_info_and_flagged() {
        let e = IoError::from(amg_core::Cancelled);
        assert!(e.is_cancelled());
        assert_eq!(e.severity(), Severity::Info);
        assert_eq!(IoError::Empty.severity(), Severity::Error);
    }

    #[test]
    fn unknown_format_hint_points_to_file_open_for_amg() {
        let h = IoError::UnknownFormat.hint().unwrap_or_default();
        assert!(h.contains(".amg") && h.contains("File > Open"), "{h}");
    }

    #[test]
    fn warnings_have_messages() {
        let ws = [
            IoWarning::ExtensionMismatch {
                extension: "stl".into(),
                detected: Format::Obj,
            },
            IoWarning::HeaderCountMismatch {
                header: 0,
                actual: 3,
            },
            IoWarning::Truncated {
                expected: 3,
                read: 1,
            },
            IoWarning::TrailingBytes(2),
            IoWarning::NonFiniteDropped(1),
            IoWarning::InvalidIndexDropped(1),
            IoWarning::PolygonTriangulated {
                polygons: 2,
                fallback_fan: 1,
            },
            IoWarning::UnitAssumed(LengthUnit::Millimetre),
            IoWarning::UnknownExtensionIgnored {
                namespace: "urn:x".into(),
            },
            IoWarning::UnsupportedContentIgnored {
                what: "textures".into(),
                count: 1,
            },
            IoWarning::PaintApproximated { triangles: 4 },
            IoWarning::NotManifold {
                object: "a".into(),
                boundary_edges: 1,
                non_manifold_edges: 0,
            },
            IoWarning::PrecisionLoss { max_error_mm: 1e-7 },
            IoWarning::MissingReference {
                what: "a.mtl".into(),
            },
        ];
        for w in &ws {
            assert!(!w.message().is_empty(), "{w:?}");
        }
    }
}
