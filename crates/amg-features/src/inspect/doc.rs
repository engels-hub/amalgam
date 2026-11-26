//! Document glue of the section analysis (INS-09, S05 §3): the `inspect.section` extension
//! record and the helpers the tool, the browser and the viewport share.
//!
//! A section analysis is **not a timeline feature**: it is an [`ExtRecord`] of kind
//! [`SECTION_KIND`], version [`SECTION_VERSION`], so it gets undo, save/load and `ExtChanged`
//! events from S01. The record's only mutation path is whole-record replace:
//! - create: [`SectionAnalysis::add_command`] (`Command::AddExt`, one undo step);
//! - edit, hide / show, rename: [`SectionAnalysis::command`] (`Command::SetExt`, one undo step);
//! - delete: [`delete_command`] (`Command::SetExt { record: None }`).
//!
//! `amg-doc` has no extension-registration table yet (S01 `ExtRegistration`), so the version
//! check and the migration hook ([`section_migrate`]) run in [`SectionAnalysis::from_record`],
//! the same way as `am.print_setup` (AM-01).
//!
//! **Combining.** Every visible analysis contributes one plane; the kept region is the
//! intersection of their kept half-spaces (each plane removes the side its normal points into),
//! at most [`MAX_SECTION_PLANES`] planes ([`combine`]).

pub mod assembly;
pub mod base;

use amg_core::{Plane, Point3, Tolerance};
use amg_doc::{BodyId, Command, Design, EntityRef, ExtKey, ExtRecord};
use serde::{Deserialize, Serialize};

use super::section::{MAX_SECTION_PLANES, SectionPlanes, section_plane};

/// Extension kind of a section analysis.
pub const SECTION_KIND: &str = "inspect.section";
/// Current record version.
pub const SECTION_VERSION: u32 = 1;

/// An origin plane of the design (Phase 1 bases).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum OriginPlane {
    /// World XY: normal +Z.
    XY,
    /// World YZ: normal +X.
    YZ,
    /// World XZ: normal −Y (right-handed, `Plane::XZ`).
    XZ,
}

impl OriginPlane {
    /// Every origin plane, in menu order.
    pub const ALL: [OriginPlane; 3] = [OriginPlane::XY, OriginPlane::YZ, OriginPlane::XZ];

    /// The world plane.
    pub fn plane(self) -> Plane {
        match self {
            OriginPlane::XY => Plane::XY,
            OriginPlane::YZ => Plane::YZ,
            OriginPlane::XZ => Plane::XZ,
        }
    }

    /// Display name ("XY").
    pub fn name(self) -> &'static str {
        match self {
            OriginPlane::XY => "XY",
            OriginPlane::YZ => "YZ",
            OriginPlane::XZ => "XZ",
        }
    }
}

/// The plane a section starts from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SectionBase {
    /// An origin plane.
    Origin(OriginPlane),
    /// A free world plane, stored by value (Phase 1: fitted to a picked planar region; mesh
    /// faces have no persistent ids).
    Free(Plane),
    /// Phase 2 (INS-11). `last` is used, with a warning, when `entity` stops resolving.
    Ref {
        /// The referenced planar entity.
        entity: EntityRef,
        /// The last resolved plane.
        last: Plane,
    },
}

/// Hatch pattern of the caps.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HatchPattern {
    /// Plain cap colour.
    None,
    /// Parallel lines at [`HatchStyle::angle`].
    Lines45,
    /// Two sets of lines at right angles.
    Cross,
}

/// How caps are hatched.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct HatchStyle {
    /// The pattern.
    pub pattern: HatchPattern,
    /// Distance between lines on screen, px (8).
    pub spacing_px: f32,
    /// Angle of the lines from the plane's `x` axis, rad (π/4).
    pub angle: f64,
}

impl Default for HatchStyle {
    /// No hatch, 8 px, 45°.
    fn default() -> Self {
        HatchStyle {
            pattern: HatchPattern::None,
            spacing_px: 8.0,
            angle: std::f64::consts::FRAC_PI_4,
        }
    }
}

/// Colour of the caps.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum CapColor {
    /// The body's colour, darkened (RND-06 default).
    BodyColor,
    /// One colour (sRGB, linear alpha).
    Uniform(amg_mesh::Rgba8),
}

/// The bodies a section applies to.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SectionScope {
    /// Every visible body.
    AllVisible,
    /// These bodies (visible or not; hidden bodies are not drawn anyway).
    Bodies(Vec<BodyId>),
}

impl SectionScope {
    /// Whether the scope covers `body`, which is `visible` or not.
    pub fn covers(&self, body: BodyId, visible: bool) -> bool {
        match self {
            SectionScope::AllVisible => visible,
            SectionScope::Bodies(list) => list.contains(&body),
        }
    }
}

/// Why a record cannot be read.
fn decode_err(msg: impl std::fmt::Display) -> serde_json::Error {
    <serde_json::Error as serde::de::Error>::custom(msg)
}

/// One section analysis (the record's data).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SectionAnalysis {
    /// Display name (browser row).
    pub name: String,
    /// Shown and cutting.
    pub visible: bool,
    /// The base plane.
    pub base: SectionBase,
    /// Offset along the base normal, mm.
    pub offset: f64,
    /// Rotations, rad: about the base `x` axis, then about the base `y` axis.
    pub tilt: [f64; 2],
    /// Reverse the plane (keep the other side).
    pub flip: bool,
    /// Hatching of the caps.
    pub hatch: HatchStyle,
    /// Colour of the caps.
    pub cap_color: CapColor,
    /// Which bodies it cuts.
    pub scope: SectionScope,
}

impl SectionAnalysis {
    /// A visible analysis of every visible body at `base`, no offset, tilt or flip, no hatch,
    /// body-coloured caps.
    pub fn new(name: impl Into<String>, base: SectionBase) -> SectionAnalysis {
        SectionAnalysis {
            name: name.into(),
            visible: true,
            base,
            offset: 0.0,
            tilt: [0.0; 2],
            flip: false,
            hatch: HatchStyle::default(),
            cap_color: CapColor::BodyColor,
            scope: SectionScope::AllVisible,
        }
    }

    /// The record (kind [`SECTION_KIND`], version [`SECTION_VERSION`], no blobs).
    ///
    /// Non-finite numbers cannot be stored (`serde_json` writes them as `null`), so
    /// [`SectionAnalysis::validate`] should pass first; `from_record` rejects such a record.
    pub fn to_record(&self) -> ExtRecord {
        ExtRecord {
            kind: SECTION_KIND.to_owned(),
            version: SECTION_VERSION,
            data: serde_json::to_value(self).unwrap_or(serde_json::Value::Null),
            blobs: Vec::new(),
        }
    }

    /// Reads a record: kind check, version check (newer → error, older → [`section_migrate`]),
    /// decode and [`SectionAnalysis::validate`].
    ///
    /// # Errors
    /// A `serde_json` error describing the first problem.
    pub fn from_record(r: &ExtRecord) -> Result<Self, serde_json::Error> {
        if r.kind != SECTION_KIND {
            return Err(decode_err(format!(
                "record kind `{}` is not `{SECTION_KIND}`",
                r.kind
            )));
        }
        if r.version == 0 || r.version > SECTION_VERSION {
            return Err(decode_err(format!(
                "section analysis version {} is not supported (this build reads 1..={SECTION_VERSION})",
                r.version
            )));
        }
        let data = if r.version < SECTION_VERSION {
            section_migrate(r.version, r.data.clone()).map_err(decode_err)?
        } else {
            r.data.clone()
        };
        let s: SectionAnalysis = serde_json::from_value(data)?;
        s.validate().map_err(decode_err)?;
        Ok(s)
    }

    /// Checks that every number is finite and the base plane is a unit frame.
    ///
    /// # Errors
    /// What is wrong, in words.
    pub fn validate(&self) -> Result<(), String> {
        if !self.offset.is_finite() {
            return Err("offset is not finite".into());
        }
        if !self.tilt.iter().all(|t| t.is_finite()) {
            return Err("tilt is not finite".into());
        }
        if !(self.hatch.spacing_px.is_finite() && self.hatch.spacing_px > 0.0) {
            return Err("hatch spacing must be positive".into());
        }
        if !self.hatch.angle.is_finite() {
            return Err("hatch angle is not finite".into());
        }
        let p = match &self.base {
            SectionBase::Origin(_) => return Ok(()),
            SectionBase::Free(p) | SectionBase::Ref { last: p, .. } => p,
        };
        let unit = |v: amg_core::Vec3| v.is_finite() && (v.length() - 1.0).abs() < 1e-9;
        if !(p.origin.is_finite() && unit(p.x.get()) && unit(p.y.get()) && unit(p.normal.get())) {
            return Err("base plane is not a finite unit frame".into());
        }
        if (p.x.get().cross(p.y.get()) - p.normal.get()).length() > 1e-9 {
            return Err("base plane frame is not right-handed (normal != x × y)".into());
        }
        Ok(())
    }

    /// The base plane: origin planes are exact, `Ref` asks `resolve` and falls back to its last
    /// plane. The bool is true when that fallback was used.
    pub fn base_plane(&self, resolve: &dyn Fn(&EntityRef) -> Option<Plane>) -> (Plane, bool) {
        match &self.base {
            SectionBase::Origin(o) => (o.plane(), false),
            SectionBase::Free(p) => (*p, false),
            SectionBase::Ref { entity, last } => match resolve(entity) {
                Some(p) => (p, false),
                None => (*last, true),
            },
        }
    }

    /// The world section plane ([`section_plane`] of the base with offset, tilt and flip). The
    /// bool is true when a `Ref` base used its fallback plane.
    pub fn plane(&self, resolve: &dyn Fn(&EntityRef) -> Option<Plane>) -> (Plane, bool) {
        let (base, fallback) = self.base_plane(resolve);
        (
            section_plane(&base, self.offset, self.tilt, self.flip),
            fallback,
        )
    }

    /// Creates this analysis as a new record (`Command::AddExt`).
    pub fn add_command(&self) -> Command {
        Command::AddExt {
            record: self.to_record(),
        }
    }

    /// Replaces the record under `key` (edit, hide / show, rename).
    pub fn command(&self, key: ExtKey) -> Command {
        Command::SetExt {
            key,
            record: Some(self.to_record()),
        }
    }
}

/// Deletes the analysis under `key`.
pub fn delete_command(key: ExtKey) -> Command {
    Command::SetExt { key, record: None }
}

/// Migrates record data from version `from` to [`SECTION_VERSION`]. Version 1 is current, so
/// this is the identity for v1 and an error otherwise; a v2 adds its step here.
///
/// # Errors
/// [`amg_doc::MigrationError`] for versions that never existed or are newer.
pub fn section_migrate(
    from: u32,
    raw: serde_json::Value,
) -> Result<serde_json::Value, amg_doc::MigrationError> {
    match from {
        1 => Ok(raw),
        _ => Err(amg_doc::MigrationError(format!(
            "no migration from section analysis v{from}"
        ))),
    }
}

/// Every section analysis of `d` in key order, each decoded or with its error.
pub fn analyses(d: &Design) -> Vec<(ExtKey, Result<SectionAnalysis, serde_json::Error>)> {
    d.exts()
        .filter(|(k, _)| k.kind == SECTION_KIND)
        .map(|(k, r)| (k.clone(), SectionAnalysis::from_record(r)))
        .collect()
}

/// The first free "Section N" name of `d` (N from 1).
pub fn next_name(d: &Design) -> String {
    let names: Vec<String> = analyses(d)
        .into_iter()
        .filter_map(|(_, r)| r.ok().map(|a| a.name))
        .collect();
    (1..)
        .map(|n| format!("Section {n}"))
        .find(|n| !names.contains(n))
        .unwrap_or_else(|| "Section".to_owned())
}

/// The visible analyses combined into one plane set (S05 §3).
#[derive(Clone, Debug, PartialEq)]
pub struct Combined {
    /// One plane per used analysis, in input order, at most [`MAX_SECTION_PLANES`].
    pub planes: SectionPlanes,
    /// For each plane, the index of its analysis in the input.
    pub owners: Vec<usize>,
    /// Visible analyses beyond [`MAX_SECTION_PLANES`] (not applied).
    pub dropped: Vec<usize>,
    /// Analyses whose `Ref` base used its fallback plane (warning).
    pub fallback: Vec<usize>,
}

/// Combines the visible analyses of `items` (in order): each contributes its world plane; the
/// first [`MAX_SECTION_PLANES`] apply, the rest are reported in [`Combined::dropped`].
pub fn combine(
    items: &[&SectionAnalysis],
    resolve: &dyn Fn(&EntityRef) -> Option<Plane>,
) -> Combined {
    let mut out = Combined {
        planes: SectionPlanes(Vec::new()),
        owners: Vec::new(),
        dropped: Vec::new(),
        fallback: Vec::new(),
    };
    for (i, a) in items.iter().enumerate() {
        if !a.visible {
            continue;
        }
        if out.planes.0.len() == MAX_SECTION_PLANES {
            out.dropped.push(i);
            continue;
        }
        let (p, fb) = a.plane(resolve);
        if fb {
            out.fallback.push(i);
        }
        out.planes.0.push(p);
        out.owners.push(i);
    }
    out
}

/// Whether `p` lies in the kept region of `planes` (on no plane's removed side beyond `tol`).
pub fn is_kept(planes: &SectionPlanes, p: Point3, tol: &Tolerance) -> bool {
    planes
        .0
        .iter()
        .all(|pl| pl.normal.get().dot(p - pl.origin) <= tol.linear)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::float_cmp, reason = "tests")]

    use super::*;
    use amg_core::{Dir3, Vec3};

    fn sample() -> SectionAnalysis {
        let mut a = SectionAnalysis::new("Section 1", SectionBase::Origin(OriginPlane::XY));
        a.offset = 10.0;
        a.tilt = [0.1, -0.25];
        a.flip = true;
        a.hatch.pattern = HatchPattern::Cross;
        a.cap_color = CapColor::Uniform(amg_mesh::Rgba8([200, 40, 30, 255]));
        a.scope = SectionScope::Bodies(vec![BodyId {
            origin: amg_doc::FeatureId(7),
            slot: 1,
        }]);
        a
    }

    #[test]
    fn record_round_trips() {
        let a = sample();
        let r = a.to_record();
        assert_eq!((r.kind.as_str(), r.version), (SECTION_KIND, 1));
        assert_eq!(SectionAnalysis::from_record(&r).unwrap(), a);
        let free = SectionAnalysis::new(
            "Free",
            SectionBase::Free(Plane::from_normal(
                Point3::new(1.0 / 3.0, 2.0, -7.5),
                Dir3::new_unchecked(Vec3::new(1.0, 2.0, 3.0).normalize()),
            )),
        );
        assert_eq!(
            SectionAnalysis::from_record(&free.to_record()).unwrap(),
            free
        );
    }

    #[test]
    fn bad_records_are_rejected() {
        let mut r = sample().to_record();
        r.version = 2;
        assert!(SectionAnalysis::from_record(&r).is_err());
        r.version = 0;
        assert!(SectionAnalysis::from_record(&r).is_err());
        let mut r = sample().to_record();
        r.kind = "am.print_setup".into();
        assert!(SectionAnalysis::from_record(&r).is_err());
        let mut nan = sample();
        nan.offset = f64::NAN;
        assert!(SectionAnalysis::from_record(&nan.to_record()).is_err());
        let mut r = sample().to_record();
        r.data["extra"] = serde_json::json!(1);
        assert!(SectionAnalysis::from_record(&r).is_err());
        assert!(section_migrate(1, serde_json::json!({})).is_ok());
        assert!(section_migrate(0, serde_json::json!({})).is_err());
    }

    #[test]
    fn plane_follows_offset_tilt_and_flip() {
        let mut a = SectionAnalysis::new("s", SectionBase::Origin(OriginPlane::XY));
        a.offset = 10.0;
        let (p, fb) = a.plane(&|_| None);
        assert!(!fb);
        assert_eq!(p.origin, Point3::new(0.0, 0.0, 10.0));
        assert_eq!(p.normal, Dir3::Z);
        a.flip = true;
        assert_eq!(a.plane(&|_| None).0.normal.get(), -Vec3::Z);
        let r = SectionAnalysis::new(
            "r",
            SectionBase::Ref {
                entity: EntityRef::Body(BodyId {
                    origin: amg_doc::FeatureId(3),
                    slot: 0,
                }),
                last: Plane::YZ,
            },
        );
        assert_eq!(r.plane(&|_| None), (Plane::YZ, true));
        assert_eq!(r.plane(&|_| Some(Plane::XY)), (Plane::XY, false));
    }

    #[test]
    fn combine_keeps_visible_in_order_and_caps_at_six() {
        let mut items: Vec<SectionAnalysis> = (0..9)
            .map(|i| {
                let mut a =
                    SectionAnalysis::new(format!("{i}"), SectionBase::Origin(OriginPlane::XY));
                a.offset = f64::from(i);
                a
            })
            .collect();
        items[1].visible = false;
        let refs: Vec<&SectionAnalysis> = items.iter().collect();
        let c = combine(&refs, &|_| None);
        assert_eq!(c.owners, [0, 2, 3, 4, 5, 6]);
        assert_eq!(c.dropped, [7, 8]);
        assert_eq!(c.planes.0[1].origin.z, 2.0);
        assert!(is_kept(
            &c.planes,
            Point3::new(0.0, 0.0, -1.0),
            &Tolerance::mesh()
        ));
        assert!(!is_kept(
            &c.planes,
            Point3::new(0.0, 0.0, 0.5),
            &Tolerance::mesh()
        ));
    }
}
