//! OBJ writer (S12 §7): `o` per placed object, `v` (with `r g b` when the mesh has vertex
//! colours), optional `vt` / `vn`, `usemtl` runs and an optional `.mtl` next to the file.

use std::borrow::Cow;
use std::collections::HashMap;
use std::ops::Range;
use std::path::{Path, PathBuf};
use std::time::Instant;

use amg_core::{Transform, units::LengthUnit};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, NormalWeighting};
use rayon::prelude::*;

use crate::endpoint::Sink;
use crate::error::{ExportReport, IoError, IoWarning};
use crate::format::Format;
use crate::options::{CoordPrecision, ExportOptions, ValidityPolicy};
use crate::scene::{NO_SLOT, Scene, SlotIndex};
use crate::units::scale_from_mm;
use crate::{ExportCheck, check_export};

/// Items per serialisation block.
const BLOCK: usize = 1 << 14;
/// Largest rounding error (mm) of `Decimals(n)` that is not worth a warning.
const PRECISION_WARN_MM: f64 = 1e-5;

/// One placed object.
struct Part<'a> {
    mesh: Cow<'a, IndexedMesh>,
    slots: Option<Vec<SlotIndex>>,
    name: Option<String>,
}

/// Writes `scene` as OBJ (and `<stem>.mtl` next to it when materials are used and
/// `opts.obj.mtl` is set; with a memory sink no MTL is written).
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
    let instances = scene.instances();
    if instances
        .iter()
        .all(|(i, _)| scene.objects[*i as usize].mesh.tris.is_empty())
    {
        return Err(IoError::Empty);
    }
    match opts.validity_for(Format::Obj) {
        ValidityPolicy::AllowInvalid => {}
        policy => {
            let check = check_export(scene, Format::Obj, ctx)?;
            if policy == ValidityPolicy::Require && !check.is_valid() {
                return Err(invalid(check));
            }
            report.warnings.extend(check.warnings);
        }
    }

    let parts: Vec<Part<'_>> = instances
        .iter()
        .filter(|(i, _)| !scene.objects[*i as usize].mesh.tris.is_empty())
        .map(|(i, t)| part(scene, *i, t))
        .collect();
    for p in &parts {
        u32::try_from(p.mesh.face_count()).map_err(|_| IoError::LimitExceeded {
            what: "triangles",
            limit: u64::from(u32::MAX),
            actual: p.mesh.face_count() as u64,
        })?;
    }
    let unit = opts.unit;
    let prec = opts.precision;
    for p in &parts {
        let bad = p.mesh.positions.par_iter().any(|q| {
            q.to_array()
                .iter()
                .any(|&x| !scale_from_mm(x, unit).is_finite())
        });
        if bad {
            return Err(IoError::InvalidScene {
                message: "a vertex has a non-finite coordinate".into(),
            });
        }
    }
    if let CoordPrecision::Decimals(d) = prec {
        let step_mm =
            0.5 * 10f64.powi(-i32::from(d.min(30))) * crate::units::scale_to_mm(1.0, unit);
        if step_mm > PRECISION_WARN_MM {
            report.warnings.push(IoWarning::PrecisionLoss {
                max_error_mm: step_mm,
            });
        }
    }

    // Material names, unique and without line breaks.
    let mtl_names = unique_names(scene);
    let used = parts.iter().any(|p| {
        p.slots
            .as_ref()
            .is_some_and(|s| s.iter().any(|&k| k != NO_SLOT))
    });
    let mtl_target: Option<PathBuf> = match (&dst, opts.obj.mtl && used) {
        (Sink::Path(p), true) => Some(p.with_extension("mtl")),
        _ => None,
    };
    if used && opts.obj.mtl && mtl_target.is_none() {
        report.warnings.push(IoWarning::UnsupportedContentIgnored {
            what: "MTL file (a memory sink has no folder to put it in)".to_owned(),
            count: 1,
        });
    }

    // Which slot is active at the start of every part.
    let mut start_slot: Vec<SlotIndex> = Vec::with_capacity(parts.len());
    let mut state = NO_SLOT;
    for p in &parts {
        start_slot.push(state);
        if let Some(s) = p.slots.as_ref().and_then(|s| s.last()) {
            state = *s;
        }
    }

    // The MTL first: the OBJ then never names a file that does not exist.
    if let Some(mp) = &mtl_target {
        ctx.checkpoint()?;
        let text = mtl_text(scene, &mtl_names);
        let bytes = Sink::Path(mp).write(|w| {
            w.write_all(text.as_bytes())
                .map_err(|source| IoError::Write {
                    path: mp.clone(),
                    source,
                })
        })?;
        report.bytes += bytes;
    }

    let path = match &dst {
        Sink::Path(p) => (*p).to_owned(),
        Sink::Memory(_) => PathBuf::new(),
    };
    let werr = |source| IoError::Write {
        path: path.clone(),
        source,
    };
    let mtllib = mtl_target
        .as_deref()
        .and_then(Path::file_name)
        .map(|n| n.to_string_lossy().replace(['\r', '\n'], " "));
    let multi = parts.len() > 1;
    let total_faces: usize = parts.iter().map(|p| p.mesh.face_count()).sum();
    let mut done_faces = 0usize;
    let bytes = dst.write(|w| {
        let mut head = format!(
            "# Amalgam {} OBJ, unit: {}\n",
            env!("CARGO_PKG_VERSION"),
            unit_name(unit)
        );
        if let Some(m) = &mtllib {
            head.push_str("mtllib ");
            head.push_str(m);
            head.push('\n');
        }
        w.write_all(head.as_bytes()).map_err(werr)?;
        let (mut vbase, mut vtbase) = (1u64, 1u64);
        for (k, p) in parts.iter().enumerate() {
            ctx.checkpoint()?;
            let m: &IndexedMesh = &p.mesh;
            let nv = m.vertex_count();
            let mut o = String::new();
            if multi || p.name.is_some() {
                let n = p
                    .name
                    .clone()
                    .unwrap_or_else(|| format!("Object.{}", k + 1));
                o.push_str("o ");
                o.push_str(&n);
                o.push('\n');
            }
            w.write_all(o.as_bytes()).map_err(werr)?;

            // Vertices.
            let vblocks: Vec<Vec<u8>> = (0..nv)
                .step_by(BLOCK)
                .collect::<Vec<_>>()
                .into_par_iter()
                .map(|a| vertex_block(m, a..(a + BLOCK).min(nv), unit, prec))
                .collect();
            for b in &vblocks {
                w.write_all(b).map_err(werr)?;
            }
            drop(vblocks);

            // Texture coordinates, de-duplicated.
            let mut uv_ix: Option<Vec<[u32; 3]>> = None;
            let mut vt_count = 0u64;
            if let Some(uvs) = &m.attrs.corner_uvs {
                let mut map: HashMap<[u64; 2], u32> = HashMap::new();
                let mut list: Vec<[f64; 2]> = Vec::new();
                let ix: Vec<[u32; 3]> = uvs
                    .iter()
                    .map(|t| {
                        t.map(|uv| {
                            let key = [norm_bits(uv.x), norm_bits(uv.y)];
                            *map.entry(key).or_insert_with(|| {
                                list.push([uv.x, uv.y]);
                                (list.len() - 1) as u32
                            })
                        })
                    })
                    .collect();
                let mut s = Vec::with_capacity(list.len() * 24);
                for uv in &list {
                    s.extend_from_slice(b"vt ");
                    num(&mut s, uv[0], prec);
                    s.push(b' ');
                    num(&mut s, uv[1], prec);
                    s.push(b'\n');
                }
                w.write_all(&s).map_err(werr)?;
                vt_count = list.len() as u64;
                uv_ix = Some(ix);
            }

            // Vertex normals (area weighted).
            if opts.obj.normals {
                let nrm = m.vertex_normals(NormalWeighting::Area);
                let blocks: Vec<Vec<u8>> = (0..nv)
                    .step_by(BLOCK)
                    .collect::<Vec<_>>()
                    .into_par_iter()
                    .map(|a| {
                        let mut s = Vec::with_capacity(BLOCK * 30);
                        for n in &nrm[a..(a + BLOCK).min(nv)] {
                            s.extend_from_slice(b"vn ");
                            fixed(&mut s, n.x, 6);
                            s.push(b' ');
                            fixed(&mut s, n.y, 6);
                            s.push(b' ');
                            fixed(&mut s, n.z, 6);
                            s.push(b'\n');
                        }
                        s
                    })
                    .collect();
                for b in &blocks {
                    w.write_all(b).map_err(werr)?;
                }
            }

            // Faces with `usemtl` runs.
            let runs = slot_runs(p.slots.as_deref(), start_slot[k]);
            let nf = m.face_count();
            let ctxf = FaceCtx {
                mesh: m,
                vbase,
                vtbase,
                uv_ix: uv_ix.as_deref(),
                normals: opts.obj.normals,
                runs: &runs,
                names: &mtl_names,
            };
            let mut start = 0;
            let group = BLOCK * 8 * rayon::current_num_threads().max(1);
            while start < nf {
                ctx.checkpoint()?;
                let end = (start + group).min(nf);
                let blocks: Vec<Vec<u8>> = (start..end)
                    .step_by(BLOCK)
                    .collect::<Vec<_>>()
                    .into_par_iter()
                    .map(|a| ctxf.block(a..(a + BLOCK).min(end)))
                    .collect();
                for b in &blocks {
                    w.write_all(b).map_err(werr)?;
                }
                start = end;
            }
            done_faces += nf;
            ctx.progress(done_faces as f32 / total_faces.max(1) as f32);
            report.triangles += nf as u64;
            vbase += nv as u64;
            vtbase += vt_count;
        }
        Ok(())
    })?;
    report.bytes += bytes;
    report.files.push(path);
    if let Some(mp) = mtl_target {
        report.files.push(mp);
    }
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
        format: Format::Obj,
        count: check.objects.iter().filter(|o| !o.is_valid()).count(),
        check: Box::new(check),
    }
}

fn part<'a>(scene: &'a Scene, object: u32, t: &Transform) -> Part<'a> {
    let o = &scene.objects[object as usize];
    let mesh = if *t == Transform::IDENTITY {
        Cow::Borrowed(&*o.mesh)
    } else {
        let mut m = (*o.mesh).clone();
        m.transform(t);
        Cow::Owned(m)
    };
    let slots = if o.material.is_none() && o.face_materials.is_none() {
        None
    } else {
        let default = o.material.unwrap_or(NO_SLOT);
        Some(match &o.face_materials {
            Some(fm) => fm
                .iter()
                .map(|&s| if s == NO_SLOT { default } else { s })
                .collect(),
            None => vec![default; o.mesh.face_count()],
        })
    };
    Part {
        mesh,
        slots,
        name: o
            .name
            .as_deref()
            .map(|s| s.replace(['\n', '\r'], " "))
            .map(|s| s.trim().to_owned())
            .filter(|s| !s.is_empty()),
    }
}

/// Material names that are safe in `newmtl` / `usemtl` and distinct from each other.
fn unique_names(scene: &Scene) -> Vec<String> {
    let mut seen: HashMap<String, u32> = HashMap::new();
    scene
        .materials
        .iter()
        .enumerate()
        .map(|(i, m)| {
            let base = m.name.replace(['\n', '\r'], " ").trim().to_owned();
            let base = if base.is_empty() || matches!(base.as_str(), "off" | "(null)") {
                format!("Material.{}", i + 1)
            } else {
                base
            };
            let n = seen.entry(base.clone()).or_insert(0);
            *n += 1;
            if *n == 1 { base } else { format!("{base}_{n}") }
        })
        .collect()
}

fn mtl_text(scene: &Scene, names: &[String]) -> String {
    let mut s = format!("# Amalgam {} MTL\n", env!("CARGO_PKG_VERSION"));
    for (m, name) in scene.materials.iter().zip(names) {
        let c = m.color.0;
        let mut v = Vec::new();
        for (i, &b) in c[..3].iter().enumerate() {
            if i > 0 {
                v.push(b' ');
            }
            fixed(&mut v, f64::from(b) / 255.0, 6);
        }
        s.push_str("\nnewmtl ");
        s.push_str(name);
        s.push_str("\nKd ");
        s.push_str(&String::from_utf8_lossy(&v));
        s.push('\n');
        if c[3] != 255 {
            let mut a = Vec::new();
            fixed(&mut a, f64::from(c[3]) / 255.0, 6);
            s.push_str("d ");
            s.push_str(&String::from_utf8_lossy(&a));
            s.push('\n');
        }
    }
    s
}

/// Where the active material changes: (first face, slot). `before` is the slot that is
/// active when the part starts.
fn slot_runs(slots: Option<&[SlotIndex]>, before: SlotIndex) -> Vec<(usize, SlotIndex)> {
    let Some(s) = slots else {
        // Without slots the faces take whatever is active; reset it if something is.
        return if before == NO_SLOT {
            Vec::new()
        } else {
            vec![(0, NO_SLOT)]
        };
    };
    let mut runs = Vec::new();
    let mut cur = before;
    for (i, &k) in s.iter().enumerate() {
        if k != cur {
            runs.push((i, k));
            cur = k;
        }
    }
    runs
}

fn norm_bits(x: f64) -> u64 {
    if x == 0.0 { 0 } else { x.to_bits() }
}

// -------------------------------------------------------------------------- number format

const POW10: [f64; 10] = [1.0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9];

fn push_u64(out: &mut Vec<u8>, mut n: u64) {
    let mut buf = [0u8; 20];
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
        if n == 0 {
            break;
        }
    }
    out.extend_from_slice(&buf[i..]);
}

/// `x` with at most `d` decimals, trailing zeros removed, `-0` written as `0`.
fn fixed(out: &mut Vec<u8>, x: f64, d: u8) {
    let d = usize::from(d);
    if d < POW10.len() {
        let y = (x.abs() * POW10[d]).round();
        if y < 9.0e15 {
            let n = y as u64;
            let p = POW10[d] as u64;
            if n != 0 && x.is_sign_negative() {
                out.push(b'-');
            }
            push_u64(out, n / p);
            let mut frac = n % p;
            if frac != 0 {
                out.push(b'.');
                let mut digits = d;
                while frac.is_multiple_of(10) {
                    frac /= 10;
                    digits -= 1;
                }
                let mark = out.len();
                push_u64(out, frac);
                // Left-pad with zeros to `digits` places.
                let written = out.len() - mark;
                if written < digits {
                    let pad = digits - written;
                    out.splice(mark..mark, std::iter::repeat_n(b'0', pad));
                }
            }
            return;
        }
    }
    let s = format!("{x:.*}", d.min(40));
    let s = if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.')
    } else {
        &s
    };
    out.extend_from_slice(if s == "-0" { b"0" } else { s.as_bytes() });
}

/// Shortest text that reads back to the same f64.
fn roundtrip(out: &mut Vec<u8>, x: f64) {
    if x == 0.0 {
        out.push(b'0');
        return;
    }
    let a = x.abs();
    let s = if (1e-5..1e16).contains(&a) {
        format!("{x}")
    } else {
        format!("{x:e}")
    };
    out.extend_from_slice(s.as_bytes());
}

fn num(out: &mut Vec<u8>, x: f64, prec: CoordPrecision) {
    match prec {
        CoordPrecision::Decimals(d) => fixed(out, x, d),
        CoordPrecision::Roundtrip => roundtrip(out, x),
    }
}

fn vertex_block(
    m: &IndexedMesh,
    r: Range<usize>,
    unit: LengthUnit,
    prec: CoordPrecision,
) -> Vec<u8> {
    let mut out = Vec::with_capacity(r.len() * 36);
    let colors = m.attrs.vertex_colors.as_deref();
    let alpha = colors.is_some_and(|c| c.iter().any(|c| c.0[3] != 255));
    for i in r {
        let p = m.positions[i];
        out.extend_from_slice(b"v ");
        num(&mut out, scale_from_mm(p.x, unit), prec);
        out.push(b' ');
        num(&mut out, scale_from_mm(p.y, unit), prec);
        out.push(b' ');
        num(&mut out, scale_from_mm(p.z, unit), prec);
        if let Some(c) = colors {
            for &b in &c[i].0[..if alpha { 4 } else { 3 }] {
                out.push(b' ');
                fixed(&mut out, f64::from(b) / 255.0, 6);
            }
        }
        out.push(b'\n');
    }
    out
}

struct FaceCtx<'a> {
    mesh: &'a IndexedMesh,
    vbase: u64,
    vtbase: u64,
    uv_ix: Option<&'a [[u32; 3]]>,
    normals: bool,
    runs: &'a [(usize, SlotIndex)],
    names: &'a [String],
}

impl FaceCtx<'_> {
    fn block(&self, r: Range<usize>) -> Vec<u8> {
        let mut out = Vec::with_capacity(r.len() * 28);
        let mut run = self.runs.partition_point(|&(f, _)| f < r.start);
        for f in r {
            while run < self.runs.len() && self.runs[run].0 == f {
                let k = self.runs[run].1;
                out.extend_from_slice(b"usemtl ");
                if k == NO_SLOT {
                    out.extend_from_slice(b"off");
                } else {
                    out.extend_from_slice(self.names[usize::from(k)].as_bytes());
                }
                out.push(b'\n');
                run += 1;
            }
            let t = self.mesh.tris[f];
            out.push(b'f');
            for (c, &v) in t.iter().enumerate() {
                out.push(b' ');
                let vi = self.vbase + u64::from(v);
                push_u64(&mut out, vi);
                if let Some(uv) = self.uv_ix {
                    out.push(b'/');
                    push_u64(&mut out, self.vtbase + u64::from(uv[f][c]));
                    if self.normals {
                        out.push(b'/');
                        push_u64(&mut out, vi);
                    }
                } else if self.normals {
                    out.extend_from_slice(b"//");
                    push_u64(&mut out, vi);
                }
            }
            out.push(b'\n');
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(x: f64, d: u8) -> String {
        let mut v = Vec::new();
        fixed(&mut v, x, d);
        String::from_utf8(v).expect("utf8")
    }

    #[test]
    fn fixed_formats_trim_and_round() {
        assert_eq!(f(1.5, 6), "1.5");
        assert_eq!(f(-0.000_000_4, 6), "0");
        assert_eq!(f(-0.0, 6), "0");
        assert_eq!(f(0.000_001, 6), "0.000001");
        assert_eq!(f(0.000_01, 6), "0.00001");
        assert_eq!(f(-12.345_678_9, 6), "-12.345679");
        assert_eq!(f(100.0, 3), "100");
        assert_eq!(f(0.05, 2), "0.05");
        assert_eq!(f(1e12, 6), "1000000000000");
        assert_eq!(f(3.0, 0), "3");
        // Beyond the integer fast path.
        assert_eq!(f(1e300, 2).len(), 301);
    }

    #[test]
    fn roundtrip_text_reads_back() {
        for x in [
            0.1 + 0.2,
            1e-9,
            -3.5e22,
            123_456.789,
            f64::MIN_POSITIVE,
            1.0 / 3.0,
        ] {
            let mut v = Vec::new();
            roundtrip(&mut v, x);
            let back: f64 = String::from_utf8(v).expect("utf8").parse().expect("parse");
            assert_eq!(back.to_bits(), x.to_bits());
        }
    }
}
