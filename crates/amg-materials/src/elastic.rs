//! The three elastic models and their admissibility rules (note §2.1).

use amg_core::units::Pressure;

/// Material axes 1, 2, 3; axis 3 is the layer normal (build direction) for printed materials.
/// `ν_ij = −ε_j/ε_i` under uniaxial `σ_i` (Abaqus/CalculiX engineering constants).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "model", rename_all = "snake_case")]
pub enum Elastic {
    /// Two constants.
    Isotropic {
        /// Young's modulus.
        youngs: Pressure,
        /// Poisson's ratio.
        poisson: f64,
    },
    /// Isotropic in the 1-2 (layer) plane; `G12 = e_p / (2 (1 + nu_p))` is derived and
    /// `G13 = G23 = g_pt`.
    TransverselyIsotropic {
        /// In-plane modulus (axes 1, 2).
        e_p: Pressure,
        /// Through-plane modulus (axis 3).
        e_t: Pressure,
        /// In-plane Poisson's ratio.
        nu_p: f64,
        /// `ν13 = ν23`: strain through the layers under in-plane load.
        nu_pt: f64,
        /// Out-of-plane shear modulus `G13 = G23`.
        g_pt: Pressure,
    },
    /// Nine constants: `e = [E1,E2,E3]`, `nu = [ν12,ν13,ν23]`, `g = [G12,G13,G23]`.
    Orthotropic {
        /// Moduli along the three axes.
        e: [Pressure; 3],
        /// Poisson's ratios `[ν12, ν13, ν23]`.
        nu: [f64; 3],
        /// Shear moduli `[G12, G13, G23]`.
        g: [Pressure; 3],
    },
}

/// Engineering constants in the orthotropic layout of [`Elastic::Orthotropic`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineeringConstants {
    /// `[E1, E2, E3]`.
    pub e: [Pressure; 3],
    /// `[ν12, ν13, ν23]`.
    pub nu: [f64; 3],
    /// `[G12, G13, G23]`.
    pub g: [Pressure; 3],
}

/// How a P1 isotropic solver reduces an anisotropic material (always explicit).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IsoPolicy {
    /// `min E_i` and the in-plane ν.
    Conservative,
    /// Layer-plane values (for orthotropic: the mean of E1, E2 and ν12).
    InPlane,
}

/// The failing constant (name inside the elastic table) and why.
pub(crate) type Fault = (&'static str, String);

fn positive(name: &'static str, v: Pressure) -> Result<(), Fault> {
    if v.get().is_finite() && v.get() > 0.0 {
        Ok(())
    } else {
        Err((
            name,
            format!("must be positive and finite, got {} MPa", v.get()),
        ))
    }
}

fn finite(name: &'static str, v: f64) -> Result<(), Fault> {
    if v.is_finite() {
        Ok(())
    } else {
        Err((name, "must be finite".to_owned()))
    }
}

impl Elastic {
    /// Thermodynamic admissibility (positive-definite compliance), after Lempriere, AIAA J.
    /// 6(11), 1968. The error text starts with the failing constant.
    pub fn validate(&self) -> Result<(), String> {
        self.check().map_err(|(n, r)| format!("`{n}`: {r}"))
    }

    pub(crate) fn check(&self) -> Result<(), Fault> {
        match self {
            Elastic::Isotropic { youngs, poisson } => {
                positive("youngs", *youngs)?;
                finite("poisson", *poisson)?;
                if !(*poisson > -1.0 && *poisson < 0.5) {
                    return Err((
                        "poisson",
                        format!("isotropic ν must satisfy -1 < ν < 0.5, got {poisson}"),
                    ));
                }
            }
            Elastic::TransverselyIsotropic {
                e_p,
                e_t,
                nu_p,
                nu_pt,
                g_pt,
            } => {
                positive("e_p", *e_p)?;
                positive("e_t", *e_t)?;
                positive("g_pt", *g_pt)?;
                finite("nu_p", *nu_p)?;
                finite("nu_pt", *nu_pt)?;
                if nu_p.abs() >= 1.0 {
                    return Err(("nu_p", format!("|ν_p| must be below 1, got {nu_p}")));
                }
                let bound = (e_p.get() / e_t.get()).sqrt();
                if nu_pt.abs() >= bound {
                    return Err((
                        "nu_pt",
                        format!("|ν_pt| must be below sqrt(E_p/E_t) = {bound}, got {nu_pt}"),
                    ));
                }
                let nu_tp = nu_pt * e_t.get() / e_p.get();
                let det = (1.0 + nu_p) * (1.0 - nu_p - 2.0 * nu_pt * nu_tp);
                if det <= 0.0 {
                    return Err((
                        "nu_pt",
                        format!(
                            "(1+ν_p)(1−ν_p−2ν_pt·ν_tp) = {det} must be positive \
                             (compliance not positive definite)"
                        ),
                    ));
                }
            }
            Elastic::Orthotropic { e, nu, g } => {
                for (name, v) in [("e", e), ("g", g)] {
                    for x in v {
                        positive(name, *x)?;
                    }
                }
                for x in nu {
                    finite("nu", *x)?;
                }
                let (e1, e2, e3) = (e[0].get(), e[1].get(), e[2].get());
                let (n12, n13, n23) = (nu[0], nu[1], nu[2]);
                for (nij, ei, ej, label) in [
                    (n12, e1, e2, "ν12"),
                    (n13, e1, e3, "ν13"),
                    (n23, e2, e3, "ν23"),
                ] {
                    let bound = (ei / ej).sqrt();
                    if nij.abs() >= bound {
                        return Err((
                            "nu",
                            format!("|{label}| must be below sqrt(E_i/E_j) = {bound}, got {nij}"),
                        ));
                    }
                }
                let (n21, n31, n32) = (n12 * e2 / e1, n13 * e3 / e1, n23 * e3 / e2);
                let det = 1.0 - n12 * n21 - n23 * n32 - n13 * n31 - 2.0 * n21 * n32 * n13;
                if det <= 0.0 {
                    return Err((
                        "nu",
                        format!(
                            "1−ν12ν21−ν23ν32−ν13ν31−2ν21ν32ν13 = {det} must be positive \
                             (compliance not positive definite)"
                        ),
                    ));
                }
            }
        }
        Ok(())
    }

    /// True only for [`Elastic::Isotropic`].
    pub fn is_isotropic(&self) -> bool {
        matches!(self, Elastic::Isotropic { .. })
    }

    /// `Some((E, ν))` only for `Isotropic`: what P1 solvers call.
    pub fn as_isotropic(&self) -> Option<(Pressure, f64)> {
        match self {
            Elastic::Isotropic { youngs, poisson } => Some((*youngs, *poisson)),
            _ => None,
        }
    }

    /// An explicit reduction to `(E, ν)`; the caller (S13) warns when the model is anisotropic.
    pub fn isotropic_equivalent(&self, policy: IsoPolicy) -> (Pressure, f64) {
        let min = |a: Pressure, b: Pressure| if b < a { b } else { a };
        match (self, policy) {
            (Elastic::Isotropic { youngs, poisson }, _) => (*youngs, *poisson),
            (Elastic::TransverselyIsotropic { e_p, nu_p, .. }, IsoPolicy::InPlane) => (*e_p, *nu_p),
            (Elastic::TransverselyIsotropic { e_p, e_t, nu_p, .. }, IsoPolicy::Conservative) => {
                (min(*e_p, *e_t), *nu_p)
            }
            (Elastic::Orthotropic { e, nu, .. }, IsoPolicy::InPlane) => {
                ((e[0] + e[1]) / 2.0, nu[0])
            }
            (Elastic::Orthotropic { e, nu, .. }, IsoPolicy::Conservative) => {
                (min(min(e[0], e[1]), e[2]), nu[0])
            }
        }
    }

    /// The nine engineering constants (isotropic and transversely isotropic expanded).
    pub fn engineering_constants(&self) -> EngineeringConstants {
        match self {
            Elastic::Isotropic { youngs, poisson } => {
                let g = *youngs / (2.0 * (1.0 + poisson));
                EngineeringConstants {
                    e: [*youngs; 3],
                    nu: [*poisson; 3],
                    g: [g; 3],
                }
            }
            Elastic::TransverselyIsotropic {
                e_p,
                e_t,
                nu_p,
                nu_pt,
                g_pt,
            } => EngineeringConstants {
                e: [*e_p, *e_p, *e_t],
                nu: [*nu_p, *nu_pt, *nu_pt],
                g: [*e_p / (2.0 * (1.0 + nu_p)), *g_pt, *g_pt],
            },
            Elastic::Orthotropic { e, nu, g } => EngineeringConstants {
                e: *e,
                nu: *nu,
                g: *g,
            },
        }
    }
}
