//! Appearance: sRGB base colour plus glTF metallic-roughness (note §6). MAT-02 adds the rest.

use crate::error::MaterialError;
use crate::id::AppearanceId;

/// sRGB, straight alpha, as authored and as stored in 3MF (`#RRGGBBAA`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct Srgba8(pub [u8; 4]);

fn hex_val(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

fn srgb_to_linear(c: f64) -> f64 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

fn linear_to_srgb(l: f64) -> f64 {
    if l <= 0.003_130_8 {
        l * 12.92
    } else {
        1.055 * l.powf(1.0 / 2.4) - 0.055
    }
}

impl Srgba8 {
    /// Parses `#RRGGBB` (alpha FF) or `#RRGGBBAA`.
    pub fn from_hex(s: &str) -> Result<Srgba8, MaterialError> {
        let bad = || MaterialError::Invalid {
            id: s.to_owned(),
            property: "base_color".to_owned(),
            reason: "expected #RRGGBB or #RRGGBBAA".to_owned(),
        };
        let b = s.as_bytes();
        if !matches!(b.len(), 7 | 9) || b[0] != b'#' {
            return Err(bad());
        }
        let mut out = [0xFF_u8; 4];
        for (i, pair) in b[1..].chunks_exact(2).enumerate() {
            let (hi, lo) = (
                hex_val(pair[0]).ok_or_else(bad)?,
                hex_val(pair[1]).ok_or_else(bad)?,
            );
            out[i] = hi << 4 | lo;
        }
        Ok(Srgba8(out))
    }

    /// Always `#RRGGBBAA`, upper case.
    pub fn to_hex(self) -> String {
        const D: &[u8; 16] = b"0123456789ABCDEF";
        let mut s = String::with_capacity(9);
        s.push('#');
        for v in self.0 {
            s.push(char::from(D[usize::from(v >> 4)]));
            s.push(char::from(D[usize::from(v & 15)]));
        }
        s
    }

    /// Linear-light floats (IEC 61966-2-1 transfer curve); alpha stays linear.
    pub fn to_linear_f32(self) -> [f32; 4] {
        let [r, g, b, a] = self.0;
        let f = |v: u8| srgb_to_linear(f64::from(v) / 255.0) as f32;
        [f(r), f(g), f(b), f64::from(a) as f32 / 255.0]
    }

    /// Inverse of [`to_linear_f32`](Srgba8::to_linear_f32), rounded to the nearest level.
    pub fn from_linear_f32(c: [f32; 4]) -> Srgba8 {
        let enc = |l: f32| (linear_to_srgb(f64::from(l).clamp(0.0, 1.0)) * 255.0).round() as u8;
        Srgba8([
            enc(c[0]),
            enc(c[1]),
            enc(c[2]),
            (f64::from(c[3]).clamp(0.0, 1.0) * 255.0).round() as u8,
        ])
    }
}

impl TryFrom<String> for Srgba8 {
    type Error = MaterialError;
    fn try_from(s: String) -> Result<Srgba8, MaterialError> {
        Srgba8::from_hex(&s)
    }
}

impl From<Srgba8> for String {
    fn from(c: Srgba8) -> String {
        c.to_hex()
    }
}

/// glTF metallic-roughness appearance (core). Maps 1:1 to 3MF `displaycolor` + `pbmetallic`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub struct Appearance {
    /// Id (`<library>/<local>`).
    pub id: AppearanceId,
    /// Bumped on any value change.
    pub revision: u32,
    /// Display name.
    pub name: String,
    /// sRGB base colour.
    pub base_color: Srgba8,
    /// Metallic factor, 0..=1.
    pub metallic: f32,
    /// Roughness factor, 0..=1.
    pub roughness: f32,
}

impl Appearance {
    /// A dielectric, fully rough appearance (3MF defaults: metallic 0, roughness 1).
    pub fn new(id: AppearanceId, name: impl Into<String>, base_color: Srgba8) -> Appearance {
        Appearance {
            id,
            revision: 1,
            name: name.into(),
            base_color,
            metallic: 0.0,
            roughness: 1.0,
        }
    }

    /// Range checks (same rules as the loader).
    pub fn validate(&self) -> Result<(), MaterialError> {
        let bad = |property: &str, reason: &str| MaterialError::Invalid {
            id: self.id.to_string(),
            property: property.to_owned(),
            reason: reason.to_owned(),
        };
        if self.name.trim().is_empty() {
            return Err(bad("name", "must not be empty"));
        }
        if self.revision == 0 {
            return Err(bad("revision", "must be at least 1"));
        }
        if !(0.0..=1.0).contains(&self.metallic) {
            return Err(bad("metallic", "must be within 0..=1"));
        }
        if !(0.0..=1.0).contains(&self.roughness) {
            return Err(bad("roughness", "must be within 0..=1"));
        }
        Ok(())
    }

    /// blake3 over the canonical JSON of the values (not `id` or `revision`).
    pub fn content_hash(&self) -> [u8; 32] {
        #[derive(serde::Serialize)]
        struct View<'a> {
            name: &'a str,
            base_color: Srgba8,
            metallic: f32,
            roughness: f32,
        }
        crate::material::hash_json(&View {
            name: &self.name,
            base_color: self.base_color,
            metallic: self.metallic,
            roughness: self.roughness,
        })
    }
}
