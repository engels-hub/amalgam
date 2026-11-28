//! The parity rule of SOL-29 Done-when 1, on plain numbers (no kernel, so it is tested in
//! every build).

/// Largest relative volume and area difference.
pub const REL: f64 = 1e-4;
/// Largest bounding-box corner difference per coordinate, mm.
pub const BBOX_MM: f64 = 1e-3;
/// Above this largest edge/face/vertex tolerance (mm) a part is "loose" and the owner's IO-06
/// rule (S12, 2026-11-26) widens the volume bound to `area × tolerance` and the bbox bound to
/// the tolerance.
pub const LOOSE_TOL_MM: f64 = 1e-6;

/// What is compared: the sums over every part instance.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Metrics {
    /// Number of solids.
    pub solids: usize,
    /// Volume, mm³.
    pub volume: f64,
    /// Surface area, mm².
    pub area: f64,
    /// Lower bounding-box corner, mm.
    pub min: [f64; 3],
    /// Upper bounding-box corner, mm.
    pub max: [f64; 3],
    /// Largest sub-shape tolerance, mm.
    pub tolerance: f64,
}

/// The differences between a reference and a rebuilt model.
#[derive(Clone, Debug, PartialEq)]
pub struct Comparison {
    /// `|Δvolume| / reference volume`.
    pub volume_rel: f64,
    /// `|Δarea| / reference area`.
    pub area_rel: f64,
    /// Largest bbox corner difference, mm.
    pub bbox_mm: f64,
    /// Whether the loose-tolerance bounds applied.
    pub loose: bool,
    /// One line per violated bound; empty when the models match.
    pub mismatches: Vec<String>,
}

impl Comparison {
    /// The three measured differences on one line.
    pub fn summary(&self) -> String {
        format!(
            "volume {:.2e} rel, area {:.2e} rel, bbox {:.2e} mm{}",
            self.volume_rel,
            self.area_rel,
            self.bbox_mm,
            if self.loose {
                " (loose-tolerance bounds)"
            } else {
                ""
            }
        )
    }
}

/// Compares `rebuilt` with `reference` (the Fusion STEP).
pub fn compare(reference: &Metrics, rebuilt: &Metrics) -> Comparison {
    let tol = reference.tolerance.max(rebuilt.tolerance);
    let loose = tol > LOOSE_TOL_MM;
    let mut mismatches = Vec::new();
    if reference.solids != rebuilt.solids {
        mismatches.push(format!(
            "solids: reference {}, rebuilt {}",
            reference.solids, rebuilt.solids
        ));
    }
    let dv = (reference.volume - rebuilt.volume).abs();
    let mut volume_bound = REL * reference.volume.abs();
    if loose {
        volume_bound = volume_bound.max(reference.area.abs() * tol);
    }
    // `!(d <= bound)` so that NaN is a mismatch.
    if !within(dv, volume_bound) {
        mismatches.push(format!(
            "volume: reference {:.9} mm³, rebuilt {:.9} mm³ (|Δ| {dv:.3e} > {volume_bound:.3e})",
            reference.volume, rebuilt.volume
        ));
    }
    let da = (reference.area - rebuilt.area).abs();
    let area_bound = REL * reference.area.abs();
    if !within(da, area_bound) {
        mismatches.push(format!(
            "area: reference {:.9} mm², rebuilt {:.9} mm² (|Δ| {da:.3e} > {area_bound:.3e})",
            reference.area, rebuilt.area
        ));
    }
    let bbox_bound = if loose { BBOX_MM.max(tol) } else { BBOX_MM };
    let mut bbox_mm = 0.0_f64;
    for (name, a, b) in [
        ("min", reference.min, rebuilt.min),
        ("max", reference.max, rebuilt.max),
    ] {
        for (axis, (x, y)) in ["x", "y", "z"].into_iter().zip(a.into_iter().zip(b)) {
            let d = (x - y).abs();
            bbox_mm = if d.is_nan() { d } else { bbox_mm.max(d) };
            if !within(d, bbox_bound) {
                mismatches.push(format!(
                    "bbox {name}.{axis}: reference {x:.6}, rebuilt {y:.6} mm (> {bbox_bound:e})"
                ));
            }
        }
    }
    Comparison {
        volume_rel: relative(dv, reference.volume),
        area_rel: relative(da, reference.area),
        bbox_mm,
        loose,
        mismatches,
    }
}

fn within(d: f64, bound: f64) -> bool {
    d <= bound
}

fn relative(d: f64, reference: f64) -> f64 {
    if d == 0.0 { 0.0 } else { d / reference.abs() }
}
