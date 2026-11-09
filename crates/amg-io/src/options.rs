//! Read limits and import/export options (S12 §Interfaces, "options").

use amg_core::units::LengthUnit;

use crate::units::{UnitChoice, UpAxis};

/// Hard limits that keep hostile or broken files from exhausting memory or time
/// (note §9.1). Exceeding one is [`IoError::LimitExceeded`](crate::IoError::LimitExceeded).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ReadLimits {
    /// Triangles in the file.
    pub max_triangles: u64,
    /// Vertices in the file.
    pub max_vertices: u64,
    /// Uncompressed size of one zip entry.
    pub max_entry_uncompressed: u64,
    /// Uncompressed size of all zip entries together.
    pub max_total_uncompressed: u64,
    /// Compression ratio per entry, applied once the entry exceeds 64 MiB.
    pub max_compression_ratio: u32,
    /// Entries in a zip container.
    pub max_zip_entries: u32,
    /// XML nesting depth.
    pub max_xml_depth: u32,
    /// Length of one XML attribute value.
    pub max_attr_value_len: u32,
    /// Depth of 3MF component nesting.
    pub max_component_depth: u32,
}

impl Default for ReadLimits {
    fn default() -> Self {
        const GIB: u64 = 1 << 30;
        const MIB: u32 = 1 << 20;
        Self {
            max_triangles: 250_000_000,
            max_vertices: 250_000_000,
            max_entry_uncompressed: 8 * GIB,
            max_total_uncompressed: 16 * GIB,
            max_compression_ratio: 500,
            max_zip_entries: 100_000,
            max_xml_depth: 64,
            max_attr_value_len: MIB,
            max_component_depth: 32,
        }
    }
}

/// How the STL reader merges vertices.
#[derive(Copy, Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum StlWeld {
    /// Bitwise f32 equality with `-0 = +0`: lossless and deterministic.
    Exact,
    /// Merge within a distance, mm.
    Tolerance(f64),
    /// Keep the triangle soup (3 vertices per triangle).
    None,
}

/// STL import options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StlReadOptions {
    /// Vertex merging.
    pub weld: StlWeld,
    /// One object per ASCII `solid` block.
    pub split_solids: bool,
    /// Read the VisCAM/Magics face colours.
    pub face_colors: bool,
}

impl Default for StlReadOptions {
    fn default() -> Self {
        Self {
            weld: StlWeld::Exact,
            split_solids: false,
            face_colors: false,
        }
    }
}

/// 3MF import options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThreeMfReadOptions {
    /// Check every MUST of the specification (conformance tests).
    pub strict: bool,
    /// Read slicer extras: `paint_color`, `mmu_segmentation`, model configs.
    pub read_slicer_extras: bool,
    /// Keep components as groups; false flattens them into one node per instance.
    pub keep_components: bool,
}

impl Default for ThreeMfReadOptions {
    fn default() -> Self {
        Self {
            strict: false,
            read_slicer_extras: true,
            keep_components: true,
        }
    }
}

/// What OBJ `g`/`o` statements become.
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ObjGroups {
    /// `o` and `g` start new objects.
    Objects,
    /// Groups become face groups of one object.
    FaceGroups,
    /// Ignore them.
    Ignore,
}

/// OBJ import options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ObjReadOptions {
    /// Meaning of `o`/`g`.
    pub groups_as: ObjGroups,
    /// Load the `mtllib` file for colours.
    pub load_mtl: bool,
}

impl Default for ObjReadOptions {
    fn default() -> Self {
        Self {
            groups_as: ObjGroups::Objects,
            load_mtl: true,
        }
    }
}

/// Everything an import can be asked.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ImportOptions {
    /// Unit policy.
    pub unit: UnitChoice,
    /// Up axis of the file.
    pub up_axis: UpAxis,
    /// One object for the whole file.
    pub merge_objects: bool,
    /// Safety limits.
    pub limits: ReadLimits,
    /// STL options.
    pub stl: StlReadOptions,
    /// 3MF options.
    pub threemf: ThreeMfReadOptions,
    /// OBJ options.
    pub obj: ObjReadOptions,
}

impl Default for ImportOptions {
    fn default() -> Self {
        Self {
            unit: UnitChoice::default(),
            up_axis: UpAxis::Z,
            merge_objects: false,
            limits: ReadLimits::default(),
            stl: StlReadOptions::default(),
            threemf: ThreeMfReadOptions::default(),
            obj: ObjReadOptions::default(),
        }
    }
}

/// Number format of exported coordinates.
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum CoordPrecision {
    /// Fixed number of decimals.
    Decimals(u8),
    /// Shortest text that reads back to the same f64.
    Roundtrip,
}

/// What to do about objects that are not valid closed solids.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum ValidityPolicy {
    /// Refuse the export.
    #[default]
    Require,
    /// Export and warn.
    Warn,
    /// Export without a check.
    AllowInvalid,
}

/// Which slicer's conventions a 3MF follows.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum SlicerFlavor {
    /// Plain 3MF core.
    #[default]
    Generic,
    /// OrcaSlicer / Bambu Studio.
    Orca,
    /// PrusaSlicer.
    Prusa,
}

/// Zip compression of exported packages.
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Compression {
    /// No compression.
    Stored,
    /// Deflate.
    Deflate {
        /// 1 (fast) to 9 (small).
        level: u8,
    },
}

/// STL export options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StlWriteOptions {
    /// ASCII instead of binary.
    pub ascii: bool,
    /// Merge all objects into one solid (world space); false writes one file per object.
    pub merge: bool,
    /// Write face colours into the attribute word.
    pub face_colors: bool,
}

impl Default for StlWriteOptions {
    fn default() -> Self {
        Self {
            ascii: false,
            merge: true,
            face_colors: false,
        }
    }
}

/// 3MF export options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThreeMfWriteOptions {
    /// Slicer conventions.
    pub flavor: SlicerFlavor,
    /// Zip compression.
    pub compression: Compression,
    /// Write materials and colours.
    pub write_materials: bool,
    /// Longest side of the thumbnail; larger RGBA input is downscaled.
    pub thumbnail_max_px: u32,
}

impl Default for ThreeMfWriteOptions {
    fn default() -> Self {
        Self {
            flavor: SlicerFlavor::Generic,
            compression: Compression::Deflate { level: 1 },
            write_materials: true,
            thumbnail_max_px: 512,
        }
    }
}

/// OBJ export options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ObjWriteOptions {
    /// Write an MTL file next to the OBJ.
    pub mtl: bool,
    /// Write vertex normals.
    pub normals: bool,
}

impl Default for ObjWriteOptions {
    fn default() -> Self {
        Self {
            mtl: true,
            normals: false,
        }
    }
}

/// PLY export options.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PlyWriteOptions {
    /// Binary little-endian instead of ASCII.
    pub binary: bool,
    /// f64 coordinates instead of f32.
    pub double: bool,
}

impl Default for PlyWriteOptions {
    fn default() -> Self {
        Self {
            binary: true,
            double: false,
        }
    }
}

/// Everything an export can be asked.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ExportOptions {
    /// Unit of the written numbers.
    pub unit: LengthUnit,
    /// Coordinate text format.
    pub precision: CoordPrecision,
    /// Validity policy; `None` = per-format default (3MF `Require`, others `Warn`).
    pub validity: Option<ValidityPolicy>,
    /// STL options.
    pub stl: StlWriteOptions,
    /// 3MF options.
    pub threemf: ThreeMfWriteOptions,
    /// OBJ options.
    pub obj: ObjWriteOptions,
    /// PLY options.
    pub ply: PlyWriteOptions,
}

impl Default for ExportOptions {
    fn default() -> Self {
        Self {
            unit: LengthUnit::Millimetre,
            precision: CoordPrecision::Decimals(6),
            validity: None,
            stl: StlWriteOptions::default(),
            threemf: ThreeMfWriteOptions::default(),
            obj: ObjWriteOptions::default(),
            ply: PlyWriteOptions::default(),
        }
    }
}

impl ExportOptions {
    /// The validity policy for `format`: the explicit choice, else 3MF `Require` and the
    /// others `Warn`.
    pub fn validity_for(&self, format: crate::Format) -> ValidityPolicy {
        self.validity.unwrap_or(match format {
            crate::Format::ThreeMf => ValidityPolicy::Require,
            _ => ValidityPolicy::Warn,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Format;

    #[test]
    fn limit_defaults_match_the_note() {
        let l = ReadLimits::default();
        assert_eq!(l.max_triangles, 250_000_000);
        assert_eq!(l.max_vertices, 250_000_000);
        assert_eq!(l.max_entry_uncompressed, 8 << 30);
        assert_eq!(l.max_total_uncompressed, 16 << 30);
        assert_eq!(l.max_compression_ratio, 500);
        assert_eq!(l.max_zip_entries, 100_000);
        assert_eq!(l.max_xml_depth, 64);
        assert_eq!(l.max_attr_value_len, 1 << 20);
        assert_eq!(l.max_component_depth, 32);
    }

    #[test]
    fn option_defaults_match_the_spec() {
        let i = ImportOptions::default();
        assert_eq!(i.up_axis, UpAxis::Z);
        assert_eq!(i.stl.weld, StlWeld::Exact);
        assert!(i.threemf.read_slicer_extras && !i.threemf.strict);
        let e = ExportOptions::default();
        assert_eq!(e.unit, LengthUnit::Millimetre);
        assert_eq!(e.precision, CoordPrecision::Decimals(6));
        assert_eq!(e.threemf.compression, Compression::Deflate { level: 1 });
        assert_eq!(e.threemf.thumbnail_max_px, 512);
        assert!(e.stl.merge && !e.stl.ascii);
        assert_eq!(e.validity_for(Format::ThreeMf), ValidityPolicy::Require);
        assert_eq!(e.validity_for(Format::Stl), ValidityPolicy::Warn);
        let forced = ExportOptions {
            validity: Some(ValidityPolicy::AllowInvalid),
            ..e
        };
        assert_eq!(
            forced.validity_for(Format::ThreeMf),
            ValidityPolicy::AllowInvalid
        );
    }

    #[test]
    fn options_round_trip_through_json() {
        let i = ImportOptions::default();
        let s = serde_json::to_string(&i).expect("ser");
        assert_eq!(serde_json::from_str::<ImportOptions>(&s).expect("de"), i);
        let e = ExportOptions::default();
        let s = serde_json::to_string(&e).expect("ser");
        assert_eq!(serde_json::from_str::<ExportOptions>(&s).expect("de"), e);
    }
}
