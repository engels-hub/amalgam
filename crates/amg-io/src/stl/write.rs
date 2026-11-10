//! STL writer (S12 §5): binary (default) or ASCII, merged into one solid or one file per
//! placed object, optional 15-bit face colours.

use std::borrow::Cow;
use std::fmt::Write as _;
use std::ops::Range;
use std::path::{Path, PathBuf};
use std::time::Instant;

use amg_core::{Transform, units::LengthUnit};
use amg_jobs::JobCtx;
use amg_mesh::IndexedMesh;
use rayon::prelude::*;

use crate::endpoint::Sink;
use crate::error::{ExportReport, IoError, IoWarning};
use crate::format::Format;
use crate::options::{ExportOptions, ValidityPolicy};
use crate::scene::{NO_SLOT, Scene, SlotIndex};
use crate::units::{scale_from_mm, scale_to_mm};
use crate::{ExportCheck, check_export};

/// Triangles per serialisation block.
const BLOCK_TRIS: usize = 1 << 15;
/// Largest f32 rounding error (mm) that is not worth a warning (`Tolerance::mesh`).
const PRECISION_WARN_MM: f64 = 1e-5;

/// One solid to write: world-space mesh plus per-face slots.
struct Solid<'a> {
    mesh: Cow<'a, IndexedMesh>,
    slots: Option<Vec<SlotIndex>>,
    name: Option<String>,
}

/// Writes `scene` as STL.
///
/// With `opts.stl.merge` (default) all placed objects go into one solid, in world space. With
/// `merge = false` and more than one placed object, `dst` must be a path: object `n` (1-based)
/// goes to `<stem>_<n>.stl` next to it.
///
/// The validity policy is [`ExportOptions::validity_for`]: `Require` refuses invalid solids,
/// `Warn` (the STL default) reports them, `AllowInvalid` skips the (expensive) analysis.
///
/// # Errors
/// [`IoError::InvalidScene`], [`IoError::Empty`] for a scene without triangles,
/// [`IoError::InvalidForFormat`], [`IoError::Unsupported`] (several files to a memory sink),
/// [`IoError::Write`] or [`IoError::Cancelled`].
pub fn write(
    scene: &Scene,
    dst: Sink<'_>,
    opts: &ExportOptions,
    ctx: &JobCtx,
) -> Result<ExportReport, IoError> {
    let t0 = Instant::now();
    ctx.checkpoint()?;
    scene.validate()?;
    let mut report = ExportReport::default();
    let instances = scene.instances();
    if instances
        .iter()
        .all(|(i, _)| scene.objects[*i as usize].mesh.tris.is_empty())
    {
        return Err(IoError::Empty);
    }

    match opts.validity_for(Format::Stl) {
        ValidityPolicy::AllowInvalid => {}
        policy => {
            let check = check_export(scene, Format::Stl, ctx)?;
            if policy == ValidityPolicy::Require && !check.is_valid() {
                return Err(invalid(check));
            }
            report.warnings.extend(check.warnings);
        }
    }

    let solids: Vec<Solid<'_>> = if opts.stl.merge || instances.len() == 1 {
        vec![merged_solid(scene, &instances, ctx)?]
    } else {
        if matches!(dst, Sink::Memory(_)) {
            return Err(IoError::Unsupported {
                format: Format::Stl,
                direction: "export",
                reason: "one file per object needs a file path, not a memory buffer",
            });
        }
        instances
            .iter()
            .map(|(i, t)| single_solid(scene, *i, t))
            .collect()
    };

    let count = solids.len();
    match dst {
        Sink::Memory(buf) => {
            let (bytes, warn) = write_solid(&solids[0], scene, Sink::Memory(buf), opts, ctx)?;
            report.bytes += bytes;
            report.triangles += solids[0].mesh.face_count() as u64;
            report.warnings.extend(warn);
        }
        Sink::Path(p) => {
            for (k, solid) in solids.iter().enumerate() {
                ctx.checkpoint()?;
                let sub = ctx.sub(k as f32 / count as f32, (k + 1) as f32 / count as f32);
                let target = if count > 1 {
                    numbered(p, k + 1)
                } else {
                    p.to_owned()
                };
                let (bytes, warn) = write_solid(solid, scene, Sink::Path(&target), opts, &sub)?;
                report.bytes += bytes;
                report.triangles += solid.mesh.face_count() as u64;
                report.warnings.extend(warn);
                report.files.push(target);
            }
        }
    }
    ctx.progress(1.0);
    report.duration = t0.elapsed();
    Ok(report)
}

fn invalid(check: ExportCheck) -> IoError {
    IoError::InvalidForFormat {
        format: Format::Stl,
        count: check.objects.iter().filter(|o| !o.is_valid()).count(),
        check: Box::new(check),
    }
}

fn numbered(p: &Path, n: usize) -> PathBuf {
    let stem = p.file_stem().unwrap_or_default().to_string_lossy();
    p.with_file_name(format!("{stem}_{n}.stl"))
}

fn slots_of(scene: &Scene, object: u32) -> Option<Vec<SlotIndex>> {
    let o = &scene.objects[object as usize];
    if o.material.is_none() && o.face_materials.is_none() {
        return None;
    }
    let default = o.material.unwrap_or(NO_SLOT);
    Some(match &o.face_materials {
        Some(fm) => fm
            .iter()
            .map(|&s| if s == NO_SLOT { default } else { s })
            .collect(),
        None => vec![default; o.mesh.face_count()],
    })
}

fn single_solid<'a>(scene: &'a Scene, object: u32, t: &Transform) -> Solid<'a> {
    let o = &scene.objects[object as usize];
    let mesh = if *t == Transform::IDENTITY {
        Cow::Borrowed(&*o.mesh)
    } else {
        let mut m = (*o.mesh).clone();
        m.transform(t);
        Cow::Owned(m)
    };
    Solid {
        mesh,
        slots: slots_of(scene, object),
        name: o.name.clone(),
    }
}

fn merged_solid<'a>(
    scene: &'a Scene,
    instances: &[(u32, Transform)],
    ctx: &JobCtx,
) -> Result<Solid<'a>, IoError> {
    if let [(i, t)] = instances {
        return Ok(single_solid(scene, *i, t));
    }
    let (mesh, slots) = scene.flatten(ctx)?;
    let name = instances
        .first()
        .and_then(|(i, _)| scene.objects[*i as usize].name.clone());
    Ok(Solid {
        mesh: Cow::Owned(mesh),
        slots,
        name,
    })
}

/// Attribute word of a colour (VisCAM/SolidView: bit 15 valid, red in bits 10-14).
fn attr_of(c: [u8; 4]) -> u16 {
    0x8000 | (u16::from(c[0] >> 3) << 10) | (u16::from(c[1] >> 3) << 5) | u16::from(c[2] >> 3)
}

struct Prep<'a> {
    mesh: &'a IndexedMesh,
    unit: LengthUnit,
    /// Slot of every face and attribute word of every slot, when colours are written.
    slots: Option<(&'a [SlotIndex], Vec<u16>)>,
}

impl Prep<'_> {
    #[inline]
    fn corner(&self, i: u32) -> [f32; 3] {
        let p = self.mesh.positions[i as usize];
        [
            scale_from_mm(p.x, self.unit) as f32,
            scale_from_mm(p.y, self.unit) as f32,
            scale_from_mm(p.z, self.unit) as f32,
        ]
    }

    /// Unit normal from the winding (zero for a degenerate face).
    #[inline]
    fn normal(&self, t: &[u32; 3]) -> [f32; 3] {
        let (a, b, c) = (
            self.mesh.positions[t[0] as usize],
            self.mesh.positions[t[1] as usize],
            self.mesh.positions[t[2] as usize],
        );
        let n = (b - a).cross(c - a);
        let len = n.length();
        if len > 0.0 && len.is_finite() {
            let n = n / len;
            [n.x as f32, n.y as f32, n.z as f32]
        } else {
            [0.0; 3]
        }
    }

    #[inline]
    fn attr(&self, f: usize) -> u16 {
        match &self.slots {
            Some((s, table)) => match s[f] {
                NO_SLOT => 0,
                k => table[usize::from(k)],
            },
            None => 0,
        }
    }

    fn binary_block(&self, range: Range<usize>) -> Vec<u8> {
        let mut out = Vec::with_capacity(range.len() * 50);
        let mut rec = [0u8; 50];
        for f in range {
            let t = &self.mesh.tris[f];
            let n = self.normal(t);
            let (a, b, c) = (self.corner(t[0]), self.corner(t[1]), self.corner(t[2]));
            let vals = [
                n[0], n[1], n[2], a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2],
            ];
            for (i, v) in vals.iter().enumerate() {
                rec[4 * i..4 * i + 4].copy_from_slice(&v.to_le_bytes());
            }
            rec[48..50].copy_from_slice(&self.attr(f).to_le_bytes());
            out.extend_from_slice(&rec);
        }
        out
    }

    fn ascii_block(&self, range: Range<usize>) -> Vec<u8> {
        let mut s = String::with_capacity(range.len() * 250);
        for f in range {
            let t = &self.mesh.tris[f];
            let n = self.normal(t);
            // Writing to a String cannot fail.
            let _ = write!(
                s,
                "facet normal {:e} {:e} {:e}\n  outer loop\n",
                n[0], n[1], n[2]
            );
            for &i in t {
                let v = self.corner(i);
                let _ = writeln!(s, "    vertex {:e} {:e} {:e}", v[0], v[1], v[2]);
            }
            s.push_str("  endloop\nendfacet\n");
        }
        s.into_bytes()
    }
}

fn write_solid(
    solid: &Solid<'_>,
    scene: &Scene,
    sink: Sink<'_>,
    opts: &ExportOptions,
    ctx: &JobCtx,
) -> Result<(u64, Option<IoWarning>), IoError> {
    let mesh: &IndexedMesh = &solid.mesh;
    let n = mesh.face_count();
    if u32::try_from(n).is_err() {
        return Err(IoError::LimitExceeded {
            what: "triangles",
            limit: u64::from(u32::MAX),
            actual: n as u64,
        });
    }
    let unit = opts.unit;
    // Coordinates must be finite; the f32 rounding error decides the precision warning.
    let max_err = mesh
        .positions
        .par_iter()
        .map(|p| {
            p.to_array().iter().fold(0.0f64, |m, &x| {
                let v = scale_from_mm(x, unit);
                if v.is_finite() {
                    m.max((v - f64::from(v as f32)).abs())
                } else {
                    f64::NAN
                }
            })
        })
        .reduce(
            || 0.0,
            |a, b| {
                if a.is_nan() || b.is_nan() {
                    f64::NAN
                } else {
                    a.max(b)
                }
            },
        );
    if max_err.is_nan() {
        return Err(IoError::InvalidScene {
            message: "a vertex has a non-finite coordinate".into(),
        });
    }
    let max_err_mm = scale_to_mm(max_err, unit);
    let warn = (max_err_mm > PRECISION_WARN_MM).then_some(IoWarning::PrecisionLoss {
        max_error_mm: max_err_mm,
    });

    let slots = match (&solid.slots, opts.stl.face_colors) {
        (Some(s), true) => Some((
            s.as_slice(),
            scene.materials.iter().map(|m| attr_of(m.color.0)).collect(),
        )),
        _ => None,
    };
    let prep = Prep { mesh, unit, slots };
    let group = BLOCK_TRIS * rayon::current_num_threads().max(1) * 2;
    let ascii = opts.stl.ascii;
    let name = solid
        .name
        .as_deref()
        .map(|s| s.replace(['\n', '\r'], " "))
        .filter(|s| !s.trim().is_empty())
        .unwrap_or_else(|| "Amalgam".to_owned());
    let path = match &sink {
        Sink::Path(p) => (*p).to_owned(),
        Sink::Memory(_) => PathBuf::new(),
    };
    let werr = |source| IoError::Write {
        path: path.clone(),
        source,
    };
    let bytes = sink.write(|w| {
        if ascii {
            w.write_all(format!("solid {name}\n").as_bytes())
                .map_err(werr)?;
        } else {
            let mut head = format!("Amalgam {} STL", env!("CARGO_PKG_VERSION")).into_bytes();
            head.resize(80, b' ');
            w.write_all(&head).map_err(werr)?;
            w.write_all(&(n as u32).to_le_bytes()).map_err(werr)?;
        }
        let mut start = 0;
        while start < n {
            ctx.checkpoint()?;
            let end = (start + group).min(n);
            let ranges: Vec<Range<usize>> = (start..end)
                .step_by(BLOCK_TRIS)
                .map(|a| a..(a + BLOCK_TRIS).min(end))
                .collect();
            let blocks: Vec<Vec<u8>> = ranges
                .into_par_iter()
                .map(|r| {
                    if ascii {
                        prep.ascii_block(r)
                    } else {
                        prep.binary_block(r)
                    }
                })
                .collect();
            for b in &blocks {
                w.write_all(b).map_err(werr)?;
            }
            start = end;
            ctx.progress(start as f32 / n as f32);
        }
        if ascii {
            w.write_all(format!("endsolid {name}\n").as_bytes())
                .map_err(werr)?;
        }
        Ok(())
    })?;
    Ok((bytes, warn))
}
