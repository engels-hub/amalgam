//! PLY writer (S12 §7): one mesh in world space, `binary_little_endian` (default) or ASCII,
//! `float` or `double` positions, `uchar` RGBA vertex colours when the mesh has them.

use std::borrow::Cow;
use std::io::Write;
use std::path::PathBuf;
use std::time::Instant;

use amg_core::{Transform, units::LengthUnit};
use amg_jobs::JobCtx;
use amg_mesh::IndexedMesh;
use rayon::prelude::*;

use crate::endpoint::Sink;
use crate::error::{ExportReport, IoError, IoWarning};
use crate::format::Format;
use crate::options::{ExportOptions, ValidityPolicy};
use crate::scene::Scene;
use crate::units::{scale_from_mm, scale_to_mm};
use crate::{ExportCheck, check_export};

/// Items per serialisation block.
const BLOCK: usize = 1 << 14;
/// Largest rounding error (mm) of `float` positions that is not worth a warning.
const PRECISION_WARN_MM: f64 = 1e-6;

/// Writes `scene` as PLY. Several objects are merged in world space.
///
/// # Errors
/// [`IoError::InvalidScene`], [`IoError::Empty`], [`IoError::InvalidForFormat`],
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
    let instances: Vec<_> = scene
        .instances()
        .into_iter()
        .filter(|(i, _)| !scene.objects[*i as usize].mesh.tris.is_empty())
        .collect();
    if instances.is_empty() {
        return Err(IoError::Empty);
    }
    match opts.validity_for(Format::Ply) {
        ValidityPolicy::AllowInvalid => {}
        policy => {
            let check = check_export(scene, Format::Ply, ctx)?;
            if policy == ValidityPolicy::Require && !check.is_valid() {
                return Err(invalid(check));
            }
            report.warnings.extend(check.warnings);
        }
    }

    let mesh: Cow<'_, IndexedMesh> = if let [(i, t)] = instances.as_slice() {
        let m = &scene.objects[*i as usize].mesh;
        if *t == Transform::IDENTITY {
            Cow::Borrowed(&**m)
        } else {
            let mut m = (**m).clone();
            m.transform(t);
            Cow::Owned(m)
        }
    } else {
        // Only the placed, non-empty objects are merged.
        let mut merged = IndexedMesh::default();
        for (i, t) in &instances {
            ctx.checkpoint()?;
            let mut m = (*scene.objects[*i as usize].mesh).clone();
            if *t != Transform::IDENTITY {
                m.transform(t);
            }
            merged.append(&m)?;
        }
        Cow::Owned(merged)
    };
    let m: &IndexedMesh = &mesh;
    let nv = m.vertex_count();
    let nf = m.face_count();
    u32::try_from(nf).map_err(|_| IoError::LimitExceeded {
        what: "triangles",
        limit: u64::from(u32::MAX),
        actual: nf as u64,
    })?;
    u32::try_from(nv).map_err(|_| IoError::LimitExceeded {
        what: "vertices",
        limit: u64::from(u32::MAX),
        actual: nv as u64,
    })?;

    let unit = opts.unit;
    let double = opts.ply.double;
    // Non-finite and (for `float`) out-of-range coordinates, and the rounding error of `float`.
    let (bad, err) = m
        .positions
        .par_iter()
        .map(|q| {
            let mut bad = false;
            let mut err = 0.0f64;
            for x in q.to_array() {
                let v = scale_from_mm(x, unit);
                if !v.is_finite() || (!double && !(v as f32).is_finite()) {
                    bad = true;
                } else if !double {
                    err = err.max((v - f64::from(v as f32)).abs());
                }
            }
            (bad, err)
        })
        .reduce(|| (false, 0.0), |a, b| (a.0 | b.0, a.1.max(b.1)));
    if bad {
        return Err(IoError::InvalidScene {
            message: if double {
                "a vertex has a non-finite coordinate".into()
            } else {
                "a vertex has a coordinate that a float cannot hold".into()
            },
        });
    }
    let err_mm = scale_to_mm(err, unit);
    if err_mm > PRECISION_WARN_MM {
        report.warnings.push(IoWarning::PrecisionLoss {
            max_error_mm: err_mm,
        });
    }
    let has_slots = instances.iter().any(|(i, _)| {
        let o = &scene.objects[*i as usize];
        o.material.is_some() || o.face_materials.is_some()
    });
    if has_slots {
        report.warnings.push(IoWarning::UnsupportedContentIgnored {
            what: "per-face materials (PLY writes vertex colours only)".to_owned(),
            count: nf as u64,
        });
    }
    if m.attrs.corner_uvs.is_some() {
        report.warnings.push(IoWarning::UnsupportedContentIgnored {
            what: "texture coordinates (PLY writes positions and colours only)".to_owned(),
            count: nf as u64,
        });
    }
    let colors = m.attrs.vertex_colors.as_deref();

    let path = match &dst {
        Sink::Path(p) => (*p).to_owned(),
        Sink::Memory(_) => PathBuf::new(),
    };
    let werr = |source| IoError::Write {
        path: path.clone(),
        source,
    };
    let binary = opts.ply.binary;
    let index_ty = if nv <= i32::MAX as usize {
        "int"
    } else {
        "uint"
    };
    let pos_ty = if double { "double" } else { "float" };
    let mut head = format!(
        "ply\nformat {} 1.0\ncomment Amalgam {} PLY, unit: {}\nelement vertex {nv}\n",
        if binary {
            "binary_little_endian"
        } else {
            "ascii"
        },
        env!("CARGO_PKG_VERSION"),
        unit_name(unit),
    );
    for n in ["x", "y", "z"] {
        head.push_str(&format!("property {pos_ty} {n}\n"));
    }
    if colors.is_some() {
        for n in ["red", "green", "blue", "alpha"] {
            head.push_str(&format!("property uchar {n}\n"));
        }
    }
    head.push_str(&format!(
        "element face {nf}\nproperty list uchar {index_ty} vertex_indices\nend_header\n"
    ));

    let bytes = dst.write(|w| {
        w.write_all(head.as_bytes()).map_err(werr)?;
        let group = BLOCK * 8 * rayon::current_num_threads().max(1);
        // Vertices.
        let mut start = 0;
        while start < nv {
            ctx.checkpoint()?;
            let end = (start + group).min(nv);
            let blocks: Vec<Vec<u8>> = (start..end)
                .step_by(BLOCK)
                .collect::<Vec<_>>()
                .into_par_iter()
                .map(|a| vertex_block(m, colors, a..(a + BLOCK).min(end), unit, double, binary))
                .collect();
            for b in &blocks {
                w.write_all(b).map_err(werr)?;
            }
            start = end;
            ctx.progress(0.5 * start as f32 / nv.max(1) as f32);
        }
        // Faces.
        let mut start = 0;
        while start < nf {
            ctx.checkpoint()?;
            let end = (start + group).min(nf);
            let blocks: Vec<Vec<u8>> = (start..end)
                .step_by(BLOCK)
                .collect::<Vec<_>>()
                .into_par_iter()
                .map(|a| face_block(&m.tris, a..(a + BLOCK).min(end), binary))
                .collect();
            for b in &blocks {
                w.write_all(b).map_err(werr)?;
            }
            start = end;
            ctx.progress(0.5 + 0.5 * start as f32 / nf.max(1) as f32);
        }
        Ok(())
    })?;
    report.bytes = bytes;
    report.triangles = nf as u64;
    report.files.push(path);
    ctx.progress(1.0);
    report.duration = t0.elapsed();
    Ok(report)
}

fn unit_name(u: LengthUnit) -> &'static str {
    match u {
        LengthUnit::Millimetre => "mm",
        LengthUnit::Inch => "in",
        _ => "other",
    }
}

fn invalid(check: ExportCheck) -> IoError {
    IoError::InvalidForFormat {
        format: Format::Ply,
        count: check.objects.iter().filter(|o| !o.is_valid()).count(),
        check: Box::new(check),
    }
}

fn vertex_block(
    m: &IndexedMesh,
    colors: Option<&[amg_mesh::Rgba8]>,
    range: std::ops::Range<usize>,
    unit: LengthUnit,
    double: bool,
    binary: bool,
) -> Vec<u8> {
    let mut out = Vec::with_capacity(range.len() * if binary { 28 } else { 48 });
    for i in range {
        let p = m.positions[i].to_array().map(|x| {
            let v = scale_from_mm(x, unit);
            // `-0` is written as `0`.
            if v == 0.0 { 0.0 } else { v }
        });
        for v in p {
            match (binary, double) {
                (true, true) => out.extend_from_slice(&v.to_le_bytes()),
                (true, false) => out.extend_from_slice(&(v as f32).to_le_bytes()),
                (false, true) => {
                    ascii_f64(&mut out, v);
                    out.push(b' ');
                }
                (false, false) => {
                    ascii_f32(&mut out, v as f32);
                    out.push(b' ');
                }
            }
        }
        if let Some(c) = colors {
            let c = c[i].0;
            if binary {
                out.extend_from_slice(&c);
            } else {
                let _ = write!(out, "{} {} {} {} ", c[0], c[1], c[2], c[3]);
            }
        }
        if !binary {
            // Replace the trailing space by the line break.
            if let Some(last) = out.last_mut() {
                *last = b'\n';
            }
        }
    }
    out
}

/// Shortest text that reads back as the same value; exponent form only for extreme sizes, so
/// that no value is longer than 25 characters.
fn ascii_f64(out: &mut Vec<u8>, v: f64) {
    let a = v.abs();
    if a == 0.0 || (1e-5..1e15).contains(&a) {
        let _ = write!(out, "{v}");
    } else {
        let _ = write!(out, "{v:e}");
    }
}

fn ascii_f32(out: &mut Vec<u8>, v: f32) {
    let a = v.abs();
    if a == 0.0 || (1e-5..1e15).contains(&a) {
        let _ = write!(out, "{v}");
    } else {
        let _ = write!(out, "{v:e}");
    }
}

fn face_block(tris: &[[u32; 3]], range: std::ops::Range<usize>, binary: bool) -> Vec<u8> {
    let mut out = Vec::with_capacity(range.len() * if binary { 13 } else { 32 });
    for t in &tris[range] {
        if binary {
            out.push(3);
            for c in t {
                out.extend_from_slice(&c.to_le_bytes());
            }
        } else {
            let _ = writeln!(out, "3 {} {} {}", t[0], t[1], t[2]);
        }
    }
    out
}
