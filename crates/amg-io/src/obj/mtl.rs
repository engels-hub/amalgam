//! MTL parser: material names with their diffuse colour (`Kd`) and opacity (`d` / `Tr`).

use amg_mesh::Rgba8;

/// A material from an MTL file.
#[derive(Clone, Debug, PartialEq)]
pub struct MtlMaterial {
    /// Name given to `newmtl` (the rest of the line, trimmed).
    pub name: String,
    /// `Kd` as sRGB with the opacity of `d` (or `1 - Tr`) as alpha. Opaque grey 0.8, the MTL
    /// default, when the file gives none.
    pub color: Rgba8,
    /// 1-based line of the `newmtl`.
    pub line: u64,
}

/// What an MTL file holds.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mtl {
    /// Materials in file order. A name defined twice keeps both entries; lookups use the last.
    pub materials: Vec<MtlMaterial>,
    /// Texture map statements (`map_*`, `bump`, `disp`, `decal`, `refl`) that were ignored.
    pub textures_ignored: u64,
}

impl Mtl {
    /// The last material called `name`.
    pub fn get(&self, name: &str) -> Option<&MtlMaterial> {
        self.materials.iter().rev().find(|m| m.name == name)
    }
}

/// Default `Kd` of the MTL format.
const DEFAULT_KD: [f64; 3] = [0.8, 0.8, 0.8];

fn to_byte(v: f64) -> u8 {
    // Clamped to [0, 255] before the cast.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    {
        (v.clamp(0.0, 1.0) * 255.0).round() as u8
    }
}

fn is_ws(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\r')
}

fn trim(mut s: &[u8]) -> &[u8] {
    while let [first, rest @ ..] = s {
        if is_ws(*first) {
            s = rest;
        } else {
            break;
        }
    }
    while let [rest @ .., last] = s {
        if is_ws(*last) {
            s = rest;
        } else {
            break;
        }
    }
    s
}

fn nums(s: &[u8]) -> Vec<f64> {
    s.split(|&b| is_ws(b))
        .filter(|t| !t.is_empty())
        .map_while(|t| std::str::from_utf8(t).ok()?.parse::<f64>().ok())
        .collect()
}

struct Building {
    name: String,
    line: u64,
    kd: [f64; 3],
    alpha: f64,
}

impl Building {
    fn done(self) -> MtlMaterial {
        MtlMaterial {
            name: self.name,
            color: Rgba8([
                to_byte(self.kd[0]),
                to_byte(self.kd[1]),
                to_byte(self.kd[2]),
                to_byte(self.alpha),
            ]),
            line: self.line,
        }
    }
}

/// Parses an MTL file. Tolerant: unknown statements are skipped, a `Kd` that is not three
/// numbers (`spectral`, `xyz`) keeps the default, and a `Kd` before the first `newmtl` is
/// ignored. Never fails.
pub fn parse_mtl(bytes: &[u8]) -> Mtl {
    let mut out = Mtl::default();
    let mut cur: Option<Building> = None;
    let bytes = bytes.strip_prefix(b"\xEF\xBB\xBF").unwrap_or(bytes);
    // Continuation lines are joined.
    let mut joined: Vec<u8> = Vec::new();
    let mut line_no = 0u64;
    let mut stmt_line = 0u64;
    for raw in bytes.split(|&b| b == b'\n') {
        line_no += 1;
        let raw = trim(raw);
        if joined.is_empty() {
            stmt_line = line_no;
        }
        if let Some(head) = raw.strip_suffix(b"\\") {
            joined.extend_from_slice(head);
            joined.push(b' ');
            continue;
        }
        let line: &[u8] = if joined.is_empty() {
            raw
        } else {
            joined.extend_from_slice(raw);
            &joined
        };
        let line = trim(line);
        let split = line.iter().position(|&b| is_ws(b)).unwrap_or(line.len());
        let (kw, rest) = line.split_at(split);
        let rest = trim(rest);
        match kw {
            b"newmtl" => {
                if let Some(b) = cur.take() {
                    out.materials.push(b.done());
                }
                cur = Some(Building {
                    name: String::from_utf8_lossy(rest).into_owned(),
                    line: stmt_line,
                    kd: DEFAULT_KD,
                    alpha: 1.0,
                });
            }
            b"Kd" => {
                if let Some(b) = cur.as_mut() {
                    let v = nums(rest);
                    match v.as_slice() {
                        [r, g, bl] => b.kd = [*r, *g, *bl],
                        // `Kd r` alone means grey.
                        [r] => b.kd = [*r; 3],
                        _ => {}
                    }
                }
            }
            b"d" => {
                if let Some(b) = cur.as_mut() {
                    // `d -halo factor` is a rare variant; the factor is the last number.
                    if let Some(a) = nums(rest.strip_prefix(b"-halo").unwrap_or(rest)).first() {
                        b.alpha = *a;
                    }
                }
            }
            b"Tr" => {
                if let Some(b) = cur.as_mut()
                    && let Some(t) = nums(rest).first()
                {
                    b.alpha = 1.0 - *t;
                }
            }
            k if k.starts_with(b"map_") || matches!(k, b"bump" | b"disp" | b"decal" | b"refl") => {
                out.textures_ignored += 1;
            }
            _ => {}
        }
        joined.clear();
    }
    if let Some(b) = cur.take() {
        out.materials.push(b.done());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kd_d_tr_and_defaults() {
        let m = parse_mtl(
            b"# c\r\nnewmtl Red Plastic\r\nKd 1 0 0\r\nd 0.5\r\nmap_Kd a.png\r\n\
              newmtl B\nKd 0.2 0.4 0.6\nTr 0.25\nnewmtl C\nnewmtl D\nKd spectral x.rfl 1\n",
        );
        assert_eq!(m.materials.len(), 4);
        assert_eq!(m.materials[0].name, "Red Plastic");
        assert_eq!(m.materials[0].color, Rgba8([255, 0, 0, 128]));
        assert_eq!(m.materials[1].color, Rgba8([51, 102, 153, 191]));
        assert_eq!(m.materials[2].color, Rgba8([204, 204, 204, 255]));
        assert_eq!(m.materials[3].color, Rgba8([204, 204, 204, 255]));
        assert_eq!(m.textures_ignored, 1);
        assert_eq!(m.get("B").map(|x| x.line), Some(6));
    }

    #[test]
    fn out_of_range_values_clamp_and_garbage_is_ignored() {
        let m = parse_mtl(b"Kd 1 1 1\nnewmtl A\nKd 2 -1 nan\nd abc\n\xFF\xFE\n");
        assert_eq!(m.materials.len(), 1);
        // `nan` is not three usable numbers after clamping: NaN clamps to NaN and rounds to 0.
        assert_eq!(m.materials[0].color.0[3], 255);
        assert_eq!(m.materials[0].color.0[0], 255);
    }

    #[test]
    fn continuation_lines_and_bom() {
        let m = parse_mtl(b"\xEF\xBB\xBFnewmtl A\nKd 0 \\\n 1 0\n");
        assert_eq!(m.materials[0].color, Rgba8([0, 255, 0, 255]));
    }
}
