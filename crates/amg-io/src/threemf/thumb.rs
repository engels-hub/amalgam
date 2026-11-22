//! Package thumbnail for the 3MF writer (IO-11): RGBA8 pixels are downscaled to the longest
//! side `max_px` with an area (box) filter in premultiplied alpha, then encoded as PNG with
//! fixed settings, so the same pixels always give the same bytes. PNG input is written as it
//! is.

use crate::error::{IoError, IoWarning};
use crate::scene::Thumbnail;

const PNG_SIGNATURE: &[u8] = b"\x89PNG\r\n\x1a\n";

/// The PNG bytes to store as `/Metadata/thumbnail.png`, or `None` (with a warning) when the
/// thumbnail cannot be used.
///
/// # Errors
/// [`IoError::InvalidScene`] when the RGBA buffer does not match its size.
pub(crate) fn png_bytes(
    t: &Thumbnail,
    max_px: u32,
    warnings: &mut Vec<IoWarning>,
) -> Result<Option<Vec<u8>>, IoError> {
    match t {
        Thumbnail::Png(b) => {
            if b.starts_with(PNG_SIGNATURE) {
                Ok(Some(b.to_vec()))
            } else {
                warnings.push(IoWarning::UnsupportedContentIgnored {
                    what: "thumbnail that is not a PNG".into(),
                    count: 1,
                });
                Ok(None)
            }
        }
        Thumbnail::Rgba8 {
            width,
            height,
            pixels,
        } => {
            let (w, h) = (*width, *height);
            let expected = u64::from(w) * u64::from(h) * 4;
            if w == 0 || h == 0 || pixels.len() as u64 != expected {
                return Err(IoError::InvalidScene {
                    message: format!(
                        "thumbnail of {w}x{h} pixels has {} bytes, not {expected}",
                        pixels.len()
                    ),
                });
            }
            let (pw, ph, px) = downscale(w, h, pixels, max_px);
            encode(pw, ph, &px).map(Some)
        }
    }
}

/// Target size: the longest side becomes `max_px` (never upscaled; at least 1 pixel).
pub(crate) fn fit(w: u32, h: u32, max_px: u32) -> (u32, u32) {
    let side = w.max(h);
    if max_px == 0 || side <= max_px {
        return (w, h);
    }
    let s = |v: u32| {
        let r = (u64::from(v) * u64::from(max_px) + u64::from(side) / 2) / u64::from(side);
        (r as u32).clamp(1, max_px)
    };
    (s(w), s(h))
}

/// Area-weighted downscale in premultiplied alpha.
fn downscale(w: u32, h: u32, px: &[u8], max_px: u32) -> (u32, u32, Vec<u8>) {
    let (nw, nh) = fit(w, h, max_px);
    if (nw, nh) == (w, h) {
        return (w, h, px.to_vec());
    }
    let xs = spans(w, nw);
    let ys = spans(h, nh);
    let mut out = Vec::with_capacity(nw as usize * nh as usize * 4);
    for ys_row in &ys {
        for xs_col in &xs {
            let mut acc = [0f64; 4];
            let mut area = 0f64;
            for &(y, wy) in ys_row {
                for &(x, wx) in xs_col {
                    let i = (y as usize * w as usize + x as usize) * 4;
                    let a = f64::from(px[i + 3]) / 255.0;
                    let k = wx * wy;
                    acc[0] += f64::from(px[i]) * a * k;
                    acc[1] += f64::from(px[i + 1]) * a * k;
                    acc[2] += f64::from(px[i + 2]) * a * k;
                    acc[3] += a * k;
                    area += k;
                }
            }
            let alpha = acc[3] / area;
            for c in &acc[..3] {
                let v = if acc[3] > 0.0 { c / acc[3] } else { 0.0 };
                out.push(v.round().clamp(0.0, 255.0) as u8);
            }
            out.push((alpha * 255.0).round().clamp(0.0, 255.0) as u8);
        }
    }
    (nw, nh, out)
}

/// Source pixels (index, weight) covering each of `n` target pixels over `len` source pixels.
fn spans(len: u32, n: u32) -> Vec<Vec<(u32, f64)>> {
    let scale = f64::from(len) / f64::from(n);
    (0..n)
        .map(|i| {
            let a = f64::from(i) * scale;
            let b = (f64::from(i + 1) * scale).min(f64::from(len));
            let mut v = Vec::new();
            let mut x = a.floor() as u32;
            while f64::from(x) < b && x < len {
                let lo = a.max(f64::from(x));
                let hi = b.min(f64::from(x + 1));
                if hi > lo {
                    v.push((x, hi - lo));
                }
                x += 1;
            }
            v
        })
        .collect()
}

fn encode(w: u32, h: u32, px: &[u8]) -> Result<Vec<u8>, IoError> {
    let err = |e: png::EncodingError| IoError::InvalidScene {
        message: format!("thumbnail PNG encoding failed: {e}"),
    };
    let mut out = Vec::new();
    {
        let mut enc = png::Encoder::new(&mut out, w, h);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        enc.set_compression(png::Compression::Balanced);
        let mut wr = enc.write_header().map_err(err)?;
        wr.write_image_data(px).map_err(err)?;
        wr.finish().map_err(err)?;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fit_keeps_the_aspect_and_never_upscales() {
        assert_eq!(fit(1024, 512, 512), (512, 256));
        assert_eq!(fit(512, 1024, 256), (128, 256));
        assert_eq!(fit(100, 50, 512), (100, 50));
        assert_eq!(fit(4000, 1, 512), (512, 1));
        assert_eq!(fit(300, 300, 0), (300, 300));
    }

    #[test]
    fn box_filter_averages_premultiplied() {
        // 2x2 → 1x1: two opaque red, two transparent pixels.
        let px = [
            255, 0, 0, 255, 0, 255, 0, 0, //
            255, 0, 0, 255, 0, 0, 255, 0,
        ];
        let (w, h, out) = downscale(2, 2, &px, 1);
        assert_eq!((w, h), (1, 1));
        assert_eq!(out, vec![255, 0, 0, 128]);
    }

    #[test]
    fn rgba_thumbnails_encode_deterministically() {
        let (w, h) = (640u32, 480u32);
        let px: Vec<u8> = (0..w * h * 4).map(|i| (i % 251) as u8).collect();
        let t = Thumbnail::Rgba8 {
            width: w,
            height: h,
            pixels: px.into(),
        };
        let mut warn = Vec::new();
        let a = png_bytes(&t, 512, &mut warn).unwrap().unwrap();
        let b = png_bytes(&t, 512, &mut warn).unwrap().unwrap();
        assert_eq!(a, b);
        assert!(a.starts_with(PNG_SIGNATURE));
        let dec = png::Decoder::new(std::io::Cursor::new(&a));
        let r = dec.read_info().unwrap();
        assert_eq!((r.info().width, r.info().height), (512, 384));
        assert!(warn.is_empty());
    }

    #[test]
    fn bad_buffers_are_refused_and_foreign_bytes_skipped() {
        let t = Thumbnail::Rgba8 {
            width: 4,
            height: 4,
            pixels: vec![0u8; 10].into(),
        };
        assert!(matches!(
            png_bytes(&t, 512, &mut Vec::new()),
            Err(IoError::InvalidScene { .. })
        ));
        let mut warn = Vec::new();
        let j = Thumbnail::Png(vec![0xFF, 0xD8, 0xFF].into());
        assert!(png_bytes(&j, 512, &mut warn).unwrap().is_none());
        assert_eq!(warn.len(), 1);
    }
}
